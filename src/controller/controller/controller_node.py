#!/usr/bin/env python3
"""
ACC Controller Node (ROS2 Humble)
=================================
Adaptive Cruise Control node that subscribes to ego vehicle speed and
lead vehicle distance (from the YOLO perception node) and publishes
throttle/brake commands via Twist messages.

Control modes:
    CRUISE    – No lead vehicle detected → maintain target speed.
    ACC       – Lead vehicle detected → maintain safe following distance.
    EMERGENCY – Lead vehicle critically close → full brake immediately.

Control law:
    a = k_p * (d_lead - d_desired) + k_d * closing_rate
    where d_desired = d0 + T_gap * v_ego

Subscribed topics:
    /Car_1/vehicle/speed              (Float64)  – ego velocity [m/s]
    /ACC/lead_vehicle_distance        (Float32)  – bumper-to-bumper gap to
                                                    lead [m]. perception_node
                                                    now publishes this as the
                                                    IPM-projected lead rear-bumper
                                                    X minus ego.extent.x (see
                                                    DEBUG §22). All ACC constants
                                                    below are therefore in *gap*
                                                    units, not camera-to-lead.

Published topics:
    /Car_1/cmd_vel                    (Twist)    – linear.x = throttle,
                                                   linear.y = brake.
                                                   Steer is owned by
                                                   stanley_node and goes
                                                   straight to /Car_1/cmd_steer.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import Float32, Bool, String, Float64 as StdFloat64
from example_interfaces.msg import Float64 as ExFloat64
from geometry_msgs.msg import Twist

# ========================
# CONFIG FLAGS
# ========================
ENABLE_LOGGING = False  # set True to enable terminal output

# Single source of truth for the cruise / shared longitudinal target.
# ACC's cruise mode targets this. Stanley is lateral-only (no speed
# concept) and the bridge's pure-pursuit junction fallback reads
# throttle/brake from this controller via `throttle_brake_provider`
# (see carlaaccsim/pure_pursuit_controller.py:_make_ego_speed_policy)
# — so PP during junctions also tracks this same target indirectly.
CRUISE_SPEED_KMH = 20.0
# Was 25.0 purely to compensate for steady-state droop in the old P-only
# cruise law, which settled 4.3-5.0 km/h below setpoint (measured: 30 ->
# 24.8, 50 -> 44.8, 130 -> 125.5). cruise_control() is now PI, so it
# tracks the setpoint instead and the +5 offset would become a real
# +5 km/h. Back to the ODD's declared 20 km/h, which is what the vehicle
# actually did before. The declared System/Feature Designed Speed Range
# is unchanged.
# MORAI's control-input calibration (steering scale, speed feedback) is
# unvalidated -- start MORAI runs at a crawl until that's tuned. See
# morai_bridge/control_adapter_node.py.
CRUISE_SPEED_KMH_MORAI = 5.0
# ACC's gains (k_p, k_d, cruise-mode throttle/brake gain) were tuned
# against CARLA's vehicle dynamics. First live MORAI runs saturated
# throttle to 100% immediately, so MORAI gets its own 10x-softer gain
# set here -- CARLA's values (k_p=1.2, k_d=0.8, cruise gain=0.3) are
# untouched. Revisit once MORAI's actual vehicle response is validated.
ACC_GAIN_SCALE_MORAI = 0.1


class ACCNode(Node):
    """Adaptive Cruise Control node with PD-based distance control."""

    def __init__(self):
        super().__init__('Controller_Node', namespace='ACC')

        # ── Simulator parameter (carla | morai) ──────────────────────────
        self.declare_parameter('simulator', 'carla')
        simulator = self.get_parameter('simulator').get_parameter_value().string_value
        self.simulator = simulator
        self.get_logger().info(f"[INFO] Simulator: {simulator}")

        # ── Subscriptions ────────────────────────────────────────────────
        SpeedMsg = ExFloat64 if simulator == 'morai' else StdFloat64
        self.create_subscription(SpeedMsg, '/Car_1/vehicle/speed', self.ego_velocity_callback, 20)
        self.create_subscription(Float32, '/ACC/lead_vehicle_distance', self.lead_distance_callback, 20)
        self.create_subscription(Float32, '/ACC/target_speed', self.target_speed_callback, 10)

        # ── Model-ready gate ─────────────────────────────────────────────
        # Hold throttle at 0 until both YOLO (ACC) and UFLD (LKAS) have
        # finished loading — see DEBUG.md. Data-driven via a TRANSIENT_LOCAL
        # "ready" flag from each perception node rather than a fixed sleep
        # in start_adas.sh, so it self-adjusts to however long the models
        # actually take on a given machine/GPU, and doesn't care what
        # order the nodes were started in (matches the QoS the publishers
        # use, which is what makes a late subscriber here still receive
        # a flag that was published before this node even started).
        ready_qos = QoSProfile(depth=1,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        self.yolo_ready = False
        self.ufld_ready = False
        self._models_ready_logged = False
        self.create_subscription(Bool, '/ACC/perception/model_ready',
                                  self._on_yolo_ready, ready_qos)
        self.create_subscription(Bool, '/LKAS/perception/model_ready',
                                  self._on_ufld_ready, ready_qos)

        # ── Publisher ────────────────────────────────────────────────────
        # Twist: linear.x = throttle, linear.y = brake  [0.0 … 1.0]
        self.control_pub = self.create_publisher(Twist, '/Car_1/cmd_vel', 20)

        # Which branch of control_loop's decision hierarchy is driving:
        # GATE | STANDSTILL | CRUISE | EMERGENCY | ACC. Diagnostic only —
        # nothing subscribes to it for control. Exists because from the
        # outside you cannot tell "ACC PD is holding station" from
        # "CRUISE is holding set speed": both can command a steady
        # throttle with no brake. The scenario harness logs it per sample
        # so a run's trace shows exactly when the PD law had authority.
        # Inferring it from /ACC/lead_vehicle_distance is not equivalent —
        # this node low-passes that signal (ALPHA) before thresholding, so
        # an outside observer disagrees with the real branch near the
        # boundaries.
        self.mode_pub = self.create_publisher(String, '/ACC/control_mode', 10)
        self._mode = ''

        # ── ACC parameters ───────────────────────────────────────────────
        # NOTE: all distances below are *bumper-to-bumper gaps*, not
        # camera-to-lead. perception_node was previously publishing
        # pinhole camera→object distance; it now publishes
        # ipm_origin − ego_extent_x. d0=5 here therefore means a literal
        # 5 m gap at standstill (slightly more conservative than the
        # pre-IPM behaviour, which was ~3.25 m gap on a Model 3). See
        # DEBUG §22 for the semantics change.
        cruise_kmh = CRUISE_SPEED_KMH_MORAI if simulator == 'morai' else CRUISE_SPEED_KMH
        self.target_speed      = cruise_kmh / 3.6  # [m/s]
        self.d0                = 2.0       # standstill bumper gap [m]
        # d_desired = d0 + T_gap * v_ego. T_gap=0.3 s → at 20 km/h cruise
        # (5.5 m/s), d_desired ≈ 6.65 m, settling to d0=5 m at rest.
        # That matches the "follow at roughly 5 m gap" mental model and
        # gives the controller enough headroom that closing-rate dominates
        # the PD response on a lead deceleration event.
        gain_scale = ACC_GAIN_SCALE_MORAI if simulator == 'morai' else 1.0
        # Was 0.3 s. Brake onset for a stationary target is
        #     d_brake = d0 + (T_gap + k_d/k_p) * v
        # so T_gap sets how far out the PD law starts decelerating. At
        # 0.3 s that was 15.1 m at 30 km/h against a ~17.7 m first-
        # detection range — the controller sat on an already-detected
        # stationary target for ~3.8 m before reacting, then had to brake
        # at 6.93 m/s^2 when 4.91 would have done. 1.5 s puts d_brake at
        # 25.1 m, beyond first detection, so braking begins as soon as
        # YOLO reports and the peak deceleration drops accordingly.
        #
        # Does NOT make it brake behind a matched-speed lead: the D term
        # is zero there, so only the stationary/closing case changes. It
        # does widen the steady-state following gap to d0 + 1.5*v (15.3 m
        # at 30 km/h), which is a normal ACC headway — 0.3 s was unusually
        # tight. See DEBUG §43.
        #
        # Note this only moves brake onset where d_brake was BELOW the
        # detection range. At 50 km/h d_brake was already 20.4 m vs ~20.0 m
        # detection, so the residual lag there is the ALPHA low-pass
        # settling, not this term.
        self.T_gap             = 1.5
        self.k_p               = 1.2 * gain_scale       # proportional gain
        self.k_d               = 0.8 * gain_scale       # derivative gain
        # Cruise-mode's own P-gain (see cruise_control()). Unlike k_p/k_d,
        # this is NOT 10x-softened for MORAI: at gain_scale=0.1 the actual
        # throttle command (~0.042 at the typical ~1.4 m/s standstill
        # error) was too weak to move the car at all. Full CARLA-strength
        # gain here is safe because `cruise_throttle_cap` below is the
        # real MORAI safety ceiling -- this just lets errors reach it
        # instead of topping out at a fraction of it, and still tapers
        # off smoothly as v_ego approaches target.
        self.cruise_gain       = 0.3
        # Integral term for cruise. Sized so the integrator supplies the
        # steady-state drag throttle (~0.36-0.41 across 30-130 km/h) within
        # roughly two seconds of a ~1 m/s error: 0.2 * (1 m/s * 2 s) = 0.4.
        # i_limit caps its authority at ki*i_limit = 1.0, i.e. full
        # throttle, so a long climb can still saturate but never more.
        self.cruise_ki         = 0.2
        self.cruise_i_limit    = 5.0
        self.cruise_integral   = 0.0
        # Even at gain_scale=0.1, cruise throttle still hit 1.0 on first
        # MORAI runs (speed feedback was/may still be unreliable enough
        # that speed_error stays large). Hard-cap MORAI's cruise throttle
        # well below full send regardless of what the gain computes;
        # CARLA keeps its original 1.0 ceiling.
        self.cruise_throttle_cap = 0.8 if simulator == 'morai' else 1.0
        self.a_max             = 3.0       # [m/s²]
        self.a_min             = -5.0      # [m/s²]
        # Emergency threshold is now a bumper gap, not a camera distance.
        # IPM also saturates near gap ≤ 2 m (bb-bottom clips at frame
        # edge → reported gap drops well below truth) — the saturated
        # value still falls under this 3 m threshold, so saturation
        # itself trips EMERGENCY without any special branch. See §22.
        self.emergency_distance = 1.0      # full brake below this gap [m]
        self.throttle_scale    = 3.0       # a_max → full throttle
        # Calibrated so `a` is in real m/s², i.e. brake_scale is the
        # deceleration CARLA actually delivers at brake = 1.0. Measured
        # over the scenario traces (2-sample actuation lag, v > 10 km/h,
        # gap > 1 m, standstill snap and impact excluded):
        #     brake 0.0 -> 0.38 m/s^2 (drag)   n=187
        #     brake 1.0 -> 6.27 m/s^2 mean     n=24
        # so decel ~= 0.38 + 5.89*brake, and 5 m/s^2 needs brake ~= 0.78.
        #
        # Was 3.0, which mapped a = -3 to full brake — the whole range
        # a in [a_min, -3] was saturated, so the "proportional" brake was
        # not proportional at all. Combined with the D term (k_d*closing
        # rate = -0.8*v, which alone exceeds a_min for any speed above
        # 27 km/h against a STATIONARY target), every ACC braking event
        # commanded brake = 1.0 and achieved 6.3-9.3 m/s^2 even when 3 m/s^2
        # would have done.
        #
        # At 6.4, a_min = -5.0 maps to brake = 0.78 -> ~5 m/s^2, so the ACC
        # branch now cannot exceed the R171 authority limit. EMERGENCY
        # still commands brake = 1.0 as the escape hatch. See DEBUG §43.
        self.brake_scale       = 6.4       # a_min → 0.78 brake ≈ 5 m/s²
        self.prev_throttle     = 0.0
        self.THROTTLE_RATE_LIMIT = 0.05  # max throttle increase per step (20 Hz → 1.0 in ~1 second)
        # Control-loop period, used by the cruise integrator. Must match
        # the create_timer() period at the bottom of __init__.
        self.CRUISE_DT = 0.05
        # Brake-side deadband, in throttle-equivalent units of the PI
        # output. Only the brake side keeps a deadband: the throttle side
        # needs to hold a steady non-zero value to balance drag.
        self.CRUISE_BRAKE_DEADBAND = 0.05

        # ── Distance filter ──────────────────────────────────────────────
        # Low-pass on /ACC/lead_vehicle_distance. ALPHA=0.01 effectively
        # froze the filter (~100-sample memory at variable YOLO Hz), so
        # the controller saw a stale "lots of room" reading and accelerated
        # into the back of a closing lead. 0.4 gives ~3-sample (≈300 ms)
        # response while still smoothing single-frame YOLO jitter.
        self.ALPHA             = 0.4
        self.d_lead_filtered   = None
        # Consecutive no-detection frames bridged before the lead is
        # declared gone. 3 at the perception rate covers the 1-3 frame
        # dropouts measured in the scenario traces without holding a stale
        # track long enough to matter. Note the trade: if the lead really
        # does leave (a lane change, not a missed frame), ACC keeps acting
        # on it for up to 3 frames.
        self.LEAD_MISS_TOLERANCE = 3
        self.lead_miss_count   = 0
        self.last_lead_time    = None

        # ── Internal state ───────────────────────────────────────────────
        self.v_ego             = 0.0
        # ── Closed-loop deceleration control ─────────────────────────────
        # Measured longitudinal acceleration [m/s²], negative when
        # decelerating. Estimated in ego_velocity_callback.
        self.a_ego             = 0.0
        self.prev_v_ego        = None
        self.prev_v_time       = None
        self.ACCEL_MIN_DT      = 0.05   # min window for the derivative [s]
        # Low-pass on the accel estimate. Heavier filtering delays the
        # feedback and lets deceleration overshoot before the loop sees
        # it; lighter filtering feeds derivative noise straight into the
        # brake. 0.5 was the best trade in simulation against the measured
        # plant.
        self.ACCEL_ALPHA       = 0.5
        # Hard ceiling on ACC-commanded deceleration [m/s²]. UN R171 treats
        # ~5 as the limit for system-commanded DCAS braking. EMERGENCY is
        # deliberately NOT bound by this — it stays brake = 1.0.
        self.DECEL_LIMIT       = 5.0
        # PI trim on top of the brake_scale feed-forward, driven by
        # (target decel − measured decel). This is what actually holds the
        # vehicle near DECEL_LIMIT instead of merely asking politely.
        #
        # Tuned in simulation against the plant measured from the scenario
        # traces, decel = brake * (16.70 - 0.814*v_ego): brake authority
        # RISES as speed falls, so a constant brake runs away to 12.6 m/s²
        # over a stop. Peak deceleration for a 50 km/h stop, target 5.0:
        #     kp=0    ki=0.6  -> 6.35     (integral alone is too slow)
        #     kp=0.1  ki=0.6  -> 5.61     <- chosen
        #     kp=0.2  ki=0.6  -> 5.70, but 12.7 once accel noise is added
        #     kp=0.4  any     -> unstable (>14)
        # The proportional term is deliberately small: it acts on a
        # twice-differentiated signal, so it amplifies noise fastest.
        #
        # ~5.6 m/s² peak rather than exactly 5.0 is the honest result. The
        # residual is transient: the accel estimate lags the real
        # deceleration by a few ticks, so the loop cannot react before the
        # first overshoot. Steady-state tracking is at the target. Lowering
        # the setpoint to force a 5.0 peak would cost ~1 m of stopping
        # distance, which 50 km/h cannot currently afford.
        # Retuned once the loop delay was measured. Cross-correlating
        # brake command against achieved deceleration over a 50 km/h stop
        # put the brake -> decel -> estimate path at ~208 ms (4 ticks).
        # The previous kp=0.1/ki=0.6 were tuned against a delay-free model
        # and hunted badly in the real loop: 0.90, 0.91, 0.10, 0.00, 0.15,
        # ... 0.84, 0.86, 0.66, 0.38 — ramp, dump, ramp, dump. Re-simulated
        # WITH the delay, that config peaks at 9.78 m/s^2 (measured: 8.0-9.6),
        # and these gains bring it to 6.45.
        self.brake_kp          = 0.05
        self.brake_ki          = 0.3
        self.brake_i_limit     = 0.8
        self.brake_integral    = 0.0
        # Rate limit on the brake RISE (releases stay instant).
        #
        # Without it the first tick of a braking event commands
        #     ff (5.0/6.4 = 0.781) + kp*error (0.1 * 5.0 = 0.5) = 1.28 -> 1.0
        # because `measured` is still ~0 — the accel estimate has not yet
        # seen the deceleration it is supposed to regulate. Measured at
        # 50 km/h the brake then sat at 1.0 for four ticks (0.22 s) before
        # the loop caught up and correctly backed off to 0.69/0.66/0.61,
        # and the whole 7.69 m/s^2 overshoot was injected in that blind
        # window. 0.15/tick at 20 Hz reaches the 0.78 feed-forward in
        # ~0.26 s, which is about the sensing lag, so the loop is live
        # before full authority is applied.
        #
        # Real brake actuators are rate-limited anyway; this is not a hack.
        self.BRAKE_RATE_UP     = 0.10
        # Releasing is now rate-limited too. It used to be instant, which
        # made the hunting asymmetric: the loop would dump 0.91 -> 0.10 in
        # one tick, undershoot, then ramp all the way back up.
        self.BRAKE_RATE_DOWN   = 0.10
        # Brake authority model, fitted from the scenario traces:
        #     decel [m/s^2] = brake * (16.70 - 0.814 * v_ego)
        # (brake 0.781 measured 5.16 m/s^2 at 12.39 m/s and 9.61 at 5.39).
        # Authority RISES as speed falls, so a fixed feed-forward is wrong
        # at every speed but one. Scheduling it on v_ego leaves the PI only
        # a small residual to trim — which matters a lot with 208 ms of
        # delay, since the PI is the part the delay destabilises.
        self.BRAKE_AUTH_A0     = 16.70
        self.BRAKE_AUTH_A1     = 0.814
        self.BRAKE_AUTH_MIN    = 4.0
        # Brake hold across a lost lead: see control_loop's CRUISE branch.
        self.last_acc_brake    = 0.0
        self.last_acc_brake_t  = None
        self.ACC_BRAKE_HOLD_S  = 0.6
        self.prev_acc_brake    = 0.0
        # Standstill margin used by the kinematic demand below. Smaller
        # than d0: d0 is the comfort spacing the PD law settles into,
        # whereas this is "how close may we come before it counts as a
        # hit", which is what the stopping-distance maths should target.
        self.d_stop_margin     = 2.0
        # Only let the kinematic term take over once it is asking for
        # something meaningful, so a far-away lead doesn't trigger a
        # permanent light brake.
        self.KIN_ENGAGE_MPS2   = 1.5
        # Release well below engage. The gap between the two is what stops
        # the term chattering as braking bleeds off the very closing speed
        # that triggered it.
        self.KIN_RELEASE_MPS2  = 0.4
        self._kin_latched      = False
        # Closing rate is a numerical derivative of an already-noisy
        # distance; the kinematic demand squares it, so it gets its own
        # low-pass before being used.
        self.closing_rate_filt = 0.0
        self.CLOSING_ALPHA     = 0.4
        self.d_lead            = None
        self.prev_d_lead       = None
        self.prev_time         = None
        self.last_log_time     = 0.0

        # ── Control loop @ 20 Hz ─────────────────────────────────────────
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"ACC Node initialized | target={self.target_speed:.1f} m/s | "
            f"d0={self.d0} m | T_gap={self.T_gap} s | "
            f"k_p={self.k_p} | k_d={self.k_d}"
        )

    # ====================================================================
    # CALLBACKS
    # ====================================================================

    def ego_velocity_callback(self, msg):
        """Store the latest ego vehicle speed, and estimate longitudinal
        acceleration from it.

        The acceleration estimate is what closes the loop on braking. The
        brake command used to be pure feed-forward (`brake = -a/brake_scale`)
        with no idea what deceleration actually resulted — and a fixed
        brake produces wildly different deceleration depending on speed:
        measured, a constant brake = 0.781 delivered 5.16 m/s^2 at 44.6 km/h
        but 9.61 m/s^2 at 19.4 km/h. No feed-forward constant can bound
        deceleration against that, which is why the 5 m/s^2 limit needs
        real feedback. See DEBUG §43.
        """
        try:
            v = abs(msg.data)
            now = self.get_clock().now().nanoseconds / 1e9
            if self.prev_v_time is None:
                self.prev_v_time, self.prev_v_ego = now, v
            else:
                dt = now - self.prev_v_time
                # Differentiate over a minimum window: the speed topic
                # arrives at ~20 Hz but jitters, and a 2 ms gap turns
                # rounding noise into tens of m/s^2.
                if dt >= self.ACCEL_MIN_DT:
                    raw = (v - self.prev_v_ego) / dt
                    raw = max(min(raw, 15.0), -15.0)
                    self.a_ego = (self.ACCEL_ALPHA * raw
                                  + (1.0 - self.ACCEL_ALPHA) * self.a_ego)
                    self.prev_v_time, self.prev_v_ego = now, v
            self.v_ego = v
        except Exception as e:
            self.get_logger().error(f"Ego velocity callback error: {e}")

    def lead_distance_callback(self, msg: Float32):
        """Store the latest lead vehicle distance, filtered to reduce noise.

        Tolerates short detection dropouts. A single `inf` used to null
        `d_lead` outright, which sends control_loop down the CRUISE branch
        (tested BEFORE both EMERGENCY and ACC) — so mid-stop the brake was
        released and the throttle opened toward set speed. Measured over a
        50 km/h approach: across the first 7 m of braking, 6 of 11 samples
        were dropouts and the mean brake was 0.095 instead of the ~0.78 the
        controller wanted. Having wasted that distance it then had to reach
        8.40 m/s^2 to still stop in time; braking steadily from the same
        point needs only 5. See DEBUG §43.

        Dropouts are misses, not departures — YOLO rejecting a frame does
        not mean the lead has gone. `LEAD_MISS_TOLERANCE` consecutive
        misses are bridged before the lead is declared lost.
        """
        try:
            d = msg.data
            now = self.get_clock().now().nanoseconds / 1e9

            if d == float('inf') or d <= 0.0:
                self.lead_miss_count += 1
                if (self.lead_miss_count > self.LEAD_MISS_TOLERANCE
                        or self.d_lead_filtered is None):
                    # Genuinely gone (or never seen) — hand back to CRUISE.
                    self.d_lead = None
                    self.d_lead_filtered = None
                    self.closing_rate_filt = 0.0
                    return

                # Bridge the miss. Extrapolating along the last known
                # closing rate rather than merely freezing the value is
                # deliberate: acc_control derives closing_rate by
                # differencing d_lead, so a frozen distance would collapse
                # the D term to zero and *reduce* braking during exactly
                # the interval we are trying to hold it steady through.
                dt = min(max(now - self.last_lead_time, 0.0), 0.2) \
                    if self.last_lead_time is not None else 0.0
                self.d_lead_filtered = max(
                    0.0, self.d_lead_filtered + self.closing_rate_filt * dt)
                self.d_lead = self.d_lead_filtered
                self.last_lead_time = now
                return

            self.lead_miss_count = 0

            # Low-pass filter: smooth out noisy distance measurements
            # d_filtered = ALPHA * d_new + (1 - ALPHA) * d_prev
            if self.d_lead_filtered is None:
                self.d_lead_filtered = d  # initialise on first valid detection
            else:
                self.d_lead_filtered = self.ALPHA * d + (1 - self.ALPHA) * self.d_lead_filtered

            self.d_lead = self.d_lead_filtered
            self.last_lead_time = now

        except Exception as e:
            self.get_logger().error(f"Lead distance callback error: {e}")

    def target_speed_callback(self, msg):
        self.target_speed = msg.data / 3.6
        self.get_logger().info(f"Target speed updated: {self.target_speed:.1f} m/s")

    def _on_yolo_ready(self, msg: Bool):
        if msg.data and not self.yolo_ready:
            self.yolo_ready = True
            self.get_logger().info("[gate] YOLO model ready")

    def _on_ufld_ready(self, msg: Bool):
        if msg.data and not self.ufld_ready:
            self.ufld_ready = True
            self.get_logger().info("[gate] UFLD model ready")

    # ====================================================================
    # ACC CONTROL LAW
    # ====================================================================

    def acc_control(self, v_ego: float, d_lead: float) -> float:
        """
        PD-based ACC control law.

        Computes a desired longitudinal acceleration based on:
          - The distance error:  d_lead − d_desired
          - The closing rate:    d(d_lead)/dt  (estimated numerically)

        Parameters
        ----------
        v_ego  : float – ego vehicle speed [m/s]
        d_lead : float – distance to lead vehicle [m]

        Returns
        -------
        float – desired acceleration [m/s²], clipped to [a_min, a_max]
        """
        # Desired following distance: standstill gap + speed-dependent gap.
        d_desired = self.d0 + self.T_gap * v_ego

        # -- Estimate closing rate from consecutive distance measurements --
        # closing_rate > 0  → gap is increasing  (lead pulls away)
        # closing_rate < 0  → gap is shrinking    (ego approaches lead)
        # closing_rate ≈ 0  → gap is stable
        now = self.get_clock().now().nanoseconds / 1e9
        closing_rate = 0.0

        if self.prev_d_lead is not None and self.prev_time is not None:
            dt = now - self.prev_time
            if dt > 0.01:  # guard against division by zero / tiny dt
                closing_rate = (d_lead - self.prev_d_lead) / dt

        # Store current values for next iteration.
        self.prev_d_lead = d_lead
        self.prev_time = now

        self.closing_rate_filt = (self.CLOSING_ALPHA * closing_rate
                                  + (1.0 - self.CLOSING_ALPHA)
                                  * self.closing_rate_filt)

        # -- PD control --
        # P-term: positive when too far (accelerate), negative when too
        #         close (decelerate).
        # D-term: positive when gap grows (can accelerate), negative when
        #         gap shrinks (should brake earlier).
        distance_error = d_lead - d_desired
        a = self.k_p * distance_error + self.k_d * closing_rate

        # -- Kinematic demand --
        # The PD output is LINEAR in distance error and closing rate, but
        # the deceleration physically required to stop is v_close²/(2·s) —
        # quadratic in closing speed. So the PD law inherently under-reacts
        # as speed rises: its brake-onset distance grows linearly while the
        # stopping distance it must cover grows with the square. This term
        # supplies the missing quadratic, so the higher the approach speed
        # the sooner the controller reaches maximum deceleration.
        #
        # Uses CLOSING speed, not v_ego: against a lead matching our speed
        # the closing rate is ~0 and this term correctly asks for nothing,
        # whereas a v_ego-based version would demand hard braking behind
        # any normally-followed lead.
        v_close = max(0.0, -self.closing_rate_filt)
        margin = d_lead - self.d_stop_margin
        if v_close > 0.1 and margin > 0.1:
            a_req = (v_close * v_close) / (2.0 * margin)
            # Latch with hysteresis. A bare `a_req > threshold` test
            # limit-cycles: braking bleeds off v_close, a_req drops back
            # under the threshold, the term disengages, and the PD — whose
            # distance error is large and positive at these gaps, so it
            # saturates to a_max — immediately commands throttle. Speed
            # rises, a_req crosses back over, brake again. Measured at
            # 30 km/h that produced brake pulses of 0.1-0.2 alternating
            # with a ramping throttle, and the brake never exceeded 0.20
            # when it should have been holding a steady decel. See §43.
            #
            # So: engage on ENGAGE, but stay engaged all the way down to
            # RELEASE. Braking is a commitment, not a per-tick opinion.
            if a_req > self.KIN_ENGAGE_MPS2:
                self._kin_latched = True
            elif a_req < self.KIN_RELEASE_MPS2 or v_close < 0.5:
                self._kin_latched = False
            if self._kin_latched:
                # Take whichever is more urgent, then let the clip below
                # bound it at a_min = -DECEL_LIMIT.
                a = min(a, -a_req)
        elif margin <= 0.1 and v_close > 0.1:
            self._kin_latched = True
            a = min(a, self.a_min)
        else:
            self._kin_latched = False

        # Clip to physical limits.
        a = max(min(a, self.a_max), self.a_min)

        # Deadband: if distance error is small and speed is very low,
        # suppress tiny throttle commands to prevent creeping.
        if abs(distance_error) < 1.0 and v_ego < 0.5 and a > 0:
            a = 0.0

        return a

    # ====================================================================
    # CLOSED-LOOP BRAKE (deceleration tracking)
    # ====================================================================

    def brake_for_decel(self, target_decel: float) -> float:
        """Brake command that makes the vehicle actually decelerate at
        `target_decel` [m/s², positive magnitude], bounded by DECEL_LIMIT.

        Feed-forward (target/brake_scale) gets us in the neighbourhood;
        the PI trim closes the gap against the measured deceleration. The
        feed-forward alone cannot work: brake authority varies strongly
        with speed, so the same brake = 0.781 measured 5.16 m/s² at
        44.6 km/h and 9.61 m/s² at 19.4 km/h. Only feedback bounds that.

        Returns a brake in [0, 1]. EMERGENCY does not come through here —
        it commands brake = 1.0 directly and is intentionally not capped.
        """
        target = max(0.0, min(target_decel, self.DECEL_LIMIT))

        # Measured deceleration magnitude (0 while accelerating/coasting).
        measured = max(0.0, -self.a_ego)

        # PI trim. Anti-windup: stop integrating once the total command is
        # already outside the actuator range in the direction we're pushing.
        error = target - measured
        # Speed-scheduled feed-forward instead of the fixed brake_scale.
        authority = max(self.BRAKE_AUTH_MIN,
                        self.BRAKE_AUTH_A0 - self.BRAKE_AUTH_A1 * self.v_ego)
        ff = target / authority
        unsat = ff + self.brake_kp * error + self.brake_integral
        if (error > 0 and unsat < 1.0) or (error < 0 and unsat > 0.0):
            self.brake_integral += self.brake_ki * error * self.CRUISE_DT
            self.brake_integral = max(min(self.brake_integral,
                                          self.brake_i_limit),
                                      -self.brake_i_limit)

        brake = max(0.0, min(unsat, 1.0))
        # Limit how fast the brake may RISE, so the loop never saturates
        # during the window where it is still blind to the deceleration it
        # is causing. Releasing is not limited — backing off must be able
        # to happen immediately.
        brake = min(brake, self.prev_acc_brake + self.BRAKE_RATE_UP)
        brake = max(brake, self.prev_acc_brake - self.BRAKE_RATE_DOWN)
        brake = max(0.0, min(brake, 1.0))
        self.prev_acc_brake = brake
        return brake

    # ====================================================================
    # CRUISE CONTROL (fallback when no lead vehicle detected)
    # ====================================================================

    def cruise_control(self, integrate: bool = True) -> tuple:
        """
        PI speed controller for cruising.

        Was P-only with a ±0.5 m/s deadband, which could not hold a set
        speed at all: inside the deadband it commanded throttle = 0, the
        car coasted down on drag until the error left the deadband, and
        outside it the throttle settled wherever `gain * error` happened
        to equal drag. That put steady state a fixed distance BELOW
        setpoint — measured at 4.3-5.0 km/h low across 30, 50 and 130 km/h
        (setpoint 30 settled at 24.8; 50 at 44.8; 130 at 125.5). The
        droop is exactly throttle/cruise_gain, which is why a 30 km/h
        setpoint looked like it was "falling back to CRUISE_SPEED_KMH=25".

        The integral term removes that droop, so CRUISE_SPEED_KMH no
        longer needs the +5 km/h offset that used to compensate for it.

        `integrate` is False when called from the ACC branch. That branch
        calls this every tick purely to take min(acc_throttle,
        cruise_throttle) / max(acc_brake, cruise_brake) as a set-speed
        cap. During an ACC braking event v_ego falls far below the set
        speed, so a running integrator would wind up to full throttle and
        slam it on the moment ACC released authority.

        Returns (throttle, brake).
        """
        speed_error = self.target_speed - self.v_ego  # +ve = need to speed up

        # PI. Anti-windup: only accumulate while the unsaturated command
        # is still inside the actuator range, and only when this branch
        # actually owns the vehicle.
        u_p = self.cruise_gain * speed_error
        if integrate:
            u_unsat = u_p + self.cruise_ki * self.cruise_integral
            if -1.0 < u_unsat < self.cruise_throttle_cap:
                self.cruise_integral += speed_error * self.CRUISE_DT
                self.cruise_integral = max(min(self.cruise_integral,
                                               self.cruise_i_limit),
                                           -self.cruise_i_limit)
        u = u_p + self.cruise_ki * self.cruise_integral

        if u > 0.0:
            # No deadband on the throttle side any more — holding a
            # steady speed requires a steady non-zero throttle to balance
            # drag, which the old deadband made impossible.
            return (min(u, self.cruise_throttle_cap), 0.0)

        if self.simulator == 'morai':
            # MORAI's brake leaves the vehicle stuck/unresponsive rather
            # than decelerating smoothly (see DEBUG.md) — coast instead.
            return (0.0, 0.0)

        # Small deadband on the brake side only, so a few cm/s of
        # overshoot doesn't chatter the brake against the throttle.
        if u > -self.CRUISE_BRAKE_DEADBAND:
            return (0.0, 0.0)
        return (0.0, min(-u, 0.6))

    def _legacy_cruise_control(self) -> tuple:
        """Kept for reference: the P-only law described above."""
        speed_error = self.target_speed - self.v_ego  # +ve = need to speed up

        if speed_error > 0.5:
            throttle = min(speed_error * self.cruise_gain, self.cruise_throttle_cap)
            brake    = 0.0
        elif speed_error < -0.5:
            if self.simulator == 'morai':
                # MORAI's brake appears to leave the vehicle stuck/
                # unresponsive rather than just decelerating smoothly
                # (unlike CARLA's, which this branch was originally
                # tuned against) -- see DEBUG.md. Coast instead of
                # actively braking to bleed off overshoot for MORAI;
                # CARLA keeps the original proportional-brake behaviour.
                throttle = 0.0
                brake    = 0.0
            else:
                # Same gain shape on the brake side, capped a touch
                # higher (0.6) so we can actually arrest a large overshoot.
                throttle = 0.0
                brake    = min(-speed_error * self.cruise_gain, 0.6)
        else:
            throttle = 0.0
            brake    = 0.0

        return throttle, brake

    # ====================================================================
    # MAIN CONTROL LOOP  (called at 20 Hz)
    # ====================================================================

    def control_loop(self):
        """
        Main control loop – runs at 20 Hz.

        Decision hierarchy:
        1. Standstill hold  → already stopped and within range
        2. No lead vehicle  → CRUISE mode (maintain target speed)
        3. Lead vehicle critically close → EMERGENCY full brake
        4. Lead vehicle in range → ACC mode (PD distance control)
        """
        control_msg = Twist()

        # ---- MODE 0: MODEL-LOAD GATE ----
        # Refuse to command any throttle until both perception models
        # (YOLO for ACC, UFLD for LKAS) have confirmed they're loaded.
        # Brake stays at 0 too — nothing has moved yet at this point in
        # startup, so there's nothing to arrest; forcing brake=1 here
        # would just fight the sim's own rest state for no reason.
        if not (self.yolo_ready and self.ufld_ready):
            self.cruise_integral = 0.0
            self.brake_integral = 0.0
            self.prev_acc_brake = 0.0
            self._publish_mode('GATE')
            self.control_pub.publish(control_msg)  # throttle=0, brake=0
            self.prev_throttle = 0.0
            waiting_on = []
            if not self.yolo_ready:
                waiting_on.append('YOLO')
            if not self.ufld_ready:
                waiting_on.append('UFLD')
            self.get_logger().info(
                f"[gate] holding throttle at 0 — waiting on: {', '.join(waiting_on)}",
                throttle_duration_sec=2.0)
            return
        elif not self._models_ready_logged:
            self._models_ready_logged = True
            self.get_logger().info("[gate] YOLO + UFLD both ready — throttle unlocked")

        # ---- MODE 1: STANDSTILL HOLD ----
        # Suppress control when stopped and within acceptable distance range.
        # Prevents the derivative term from reacting to sensor noise at rest.
        if self.v_ego < 0.5 and self.d_lead is not None and self.d_lead < self.d0 + 2.0:
            control_msg.linear.y = 0.05  # light hold brake
            self.cruise_integral = 0.0
            self.brake_integral = 0.0
            self.prev_acc_brake = 0.0
            self._publish_mode('STANDSTILL')
            self.control_pub.publish(control_msg)
            self.prev_throttle = 0.0
            if ENABLE_LOGGING:
                self._log_throttled("STANDSTILL", 0.0, 0.05)
            return

        # ---- MODE 2: CRUISE (no lead vehicle detected) ----
        if self.d_lead is None:
            throttle, brake = self.cruise_control()

            # Brake hold across a lost lead. Falling into CRUISE mid-stop
            # used to drop the brake straight to 0 and open the throttle
            # toward set speed — measured, that left a mean brake of 0.095
            # across the first 7 m of a 50 km/h braking event. The miss
            # tolerance in lead_distance_callback bridges short dropouts;
            # this is the backstop for when it expires. Hold the last ACC
            # brake (never add to it) and suppress throttle, so the worst
            # case is coasting at the previous deceleration rather than
            # accelerating at a target we are actively braking for.
            now = self.get_clock().now().nanoseconds / 1e9
            if (self.last_acc_brake > 0.0
                    and self.last_acc_brake_t is not None
                    and (now - self.last_acc_brake_t) < self.ACC_BRAKE_HOLD_S):
                brake = max(brake, self.last_acc_brake)
                throttle = 0.0
            else:
                # Hold expired — the lead really is gone. Release.
                self.last_acc_brake = 0.0

            # Rate limit throttle
            throttle = min(throttle, self.prev_throttle + self.THROTTLE_RATE_LIMIT)
            self.prev_throttle = throttle

            control_msg.linear.x = throttle
            control_msg.linear.y = brake
            self.brake_integral = 0.0
            self._publish_mode('CRUISE')
            self.control_pub.publish(control_msg)

            # Reset closing-rate state since there is no lead vehicle
            self.prev_d_lead = None
            self.prev_time = None

            if ENABLE_LOGGING:
                self._log_throttled("CRUISE", throttle, brake)
            return

        # ---- MODE 3: EMERGENCY BRAKE (critically close) ----
        if self.d_lead < self.emergency_distance:
            control_msg.linear.x = 0.0
            control_msg.linear.y = 1.0  # full brake — no rate limit on braking
            self.cruise_integral = 0.0
            self.brake_integral = 0.0
            self.prev_acc_brake = 0.0
            self._publish_mode('EMERGENCY')
            self.control_pub.publish(control_msg)
            self.prev_throttle = 0.0
            if ENABLE_LOGGING:
                self._log_throttled("EMERGENCY", 0.0, 1.0)
            return

        # ---- MODE 4: ACC (adaptive distance control) ----
        a = self.acc_control(self.v_ego, self.d_lead)

        if a >= 0:
            acc_throttle = min(a / self.throttle_scale, 1.0)
            acc_brake    = 0.0
            # Not braking — don't let the trim wind up for the next event.
            self.brake_integral = 0.0
            self.prev_acc_brake = 0.0
        else:
            acc_throttle = 0.0
            # Closed loop on achieved deceleration, bounded at DECEL_LIMIT.
            acc_brake    = self.brake_for_decel(-a)
            if acc_brake > 0.0:
                self.last_acc_brake = acc_brake
                self.last_acc_brake_t = self.get_clock().now().nanoseconds / 1e9

        # ACC must respect CRUISE_SPEED_KMH as an upper cap. The PD law
        # above has no concept of "we're already at set speed" — with a
        # lead 30 m ahead, distance_error stays large and saturates
        # a → a_max → throttle = 1.0 *indefinitely*, even past 100 km/h.
        # Real ACC behaves as "follow the lead OR hold set speed,
        # whichever is slower" — implemented here as the lower throttle
        # and the higher brake of the two controllers. The cruise side
        # commands brake whenever v_ego > target + 0.5, which arrests
        # the runaway.
        cruise_throttle, cruise_brake = self.cruise_control(integrate=False)
        throttle = min(acc_throttle, cruise_throttle)
        brake    = max(acc_brake,    cruise_brake)

        # Rate limit throttle — braking is always instant
        throttle = min(throttle, self.prev_throttle + self.THROTTLE_RATE_LIMIT)
        self.prev_throttle = throttle

        control_msg.linear.x = throttle
        control_msg.linear.y = brake

        self._publish_mode('ACC')
        self.control_pub.publish(control_msg)

        if ENABLE_LOGGING:
            self._log_throttled("ACC", throttle, brake)

    # ====================================================================
    # LOGGING HELPER
    # ====================================================================

    def _publish_mode(self, mode: str):
        """Publish which control branch is active, at the loop rate.

        Republished every tick rather than only on change: the topic is
        VOLATILE, so a subscriber that joins mid-run (the scenario harness
        arms a run long after this node started) would otherwise see
        nothing until the next transition.
        """
        self._mode = mode
        self.mode_pub.publish(String(data=mode))

    def _log_throttled(self, mode: str, throttle: float, brake: float):
        """Overwrite a single terminal line at most once per second."""
        now = self.get_clock().now().nanoseconds / 1e9

        if now - self.last_log_time > 1.0:
            d_text = f"{self.d_lead:.2f} m" if self.d_lead is not None else "None"
            print(
                f"\r[{mode:>9}] "
                f"v={self.v_ego:.2f} m/s  "
                f"d={d_text}  "
                f"thr={throttle:.2f}  "
                f"brk={brake:.2f}          ",
                end='', flush=True
            )
            self.last_log_time = now


# ========================================================================
# ENTRY POINT
# ========================================================================

def main(args=None):
    rclpy.init(args=args)
    node = ACCNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass  # clean shutdown, no traceback
    finally:
        node.destroy_node()
        if rclpy.ok():          # ← only shutdown if not already shut down
            rclpy.shutdown()


if __name__ == '__main__':
    main()