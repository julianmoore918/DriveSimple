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

import math

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
        # Diagnostics. The gap the controller actually acts on is NOT what
        # /ACC/lead_vehicle_distance carries — it is tracked, predicted to
        # now, and latency-compensated — and the governor's reference speed
        # is invisible from outside entirely. Both were guesswork to debug
        # from the traces, so publish them.
        self.gap_pub = self.create_publisher(Float32, '/ACC/tracked_gap', 10)
        self.vref_pub = self.create_publisher(Float32, '/ACC/speed_reference', 10)

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
        # ── Lead-gap tracker (alpha-beta) ────────────────────────────────
        # Replaces the old first-order low-pass on /ACC/lead_vehicle_distance.
        #
        # That filter was the single largest source of dead time in the
        # whole loop. A first-order low-pass lags by (1-a)/a samples, and
        # this one ran at the PERCEPTION rate (~12 Hz), not the control
        # rate: alpha=0.4 -> 1.5 samples -> ~123 ms, out of a total
        # measured brake->decel->estimate dead time of 208 ms. Simulated
        # against the real plant, cutting dead time to ~104 ms is what
        # brings peak deceleration under 5 m/s^2 (5.75 -> 4.83); no
        # control-law change achieves that on its own. See DEBUG §43.
        #
        # An alpha-beta tracker fixes three things the low-pass could not:
        #   1. No group delay — it PREDICTS between measurements instead
        #      of lagging behind them.
        #   2. Closing rate becomes an estimated STATE, not a numerical
        #      derivative of an already-filtered noisy signal. The old
        #      derivative flipped sign on far-range IPM jitter, which
        #      dropped the kinematic braking latch mid-stop.
        #   3. Between perception frames the control loop gets a fresh
        #      extrapolated gap at 20 Hz rather than a held stale value.
        #
        # Beta is set from alpha by the standard critically-damped
        # relation b = a^2/(2-a), which avoids the ringing an arbitrary
        # pair produces.
        self.AB_ALPHA          = 0.45
        self.AB_BETA           = (self.AB_ALPHA ** 2) / (2.0 - self.AB_ALPHA)
        self.track_d           = None   # estimated gap [m]
        self.track_v           = 0.0    # estimated closing rate [m/s], -ve = closing
        self.track_t           = None   # timestamp of the last tracker update
        # Perception transport lag, measured by comparing the published
        # distance against CARLA ground truth: the over-read scaled with
        # speed (+1.95 m at 9.6 m/s, +1.44 m at 5.0 m/s) and collapsed to
        # a near-constant ~0.25 s when divided by it — a latency, not a
        # calibration bias. The tracker predicts forward by this much so
        # the control law sees where the lead is NOW, not where it was.
        self.PERCEPTION_LAG_S  = 0.25
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
        # Brake authority model: decel [m/s^2] = brake * (A0 - A1*v_ego).
        #
        # REFITTED. The first fit used the scenario harness's logged
        # acceleration, which is an EMA (alpha=0.3) and therefore both
        # lagged and attenuated — it under-reported a 9.11 m/s^2 peak as
        # 7.29. Fitting against a lagged signal produced authority values
        # roughly half the truth, so the feed-forward asked for about
        # twice the brake actually needed and the PI then had to unwind
        # it. That is what drove the ramp-to-0.70-then-collapse cycles.
        #
        # Re-measured with a centred-window slope of v(t) (lag-free,
        # noise-robust) over the 50 km/h traces, n=26, rms 1.60:
        #     decel = brake * (34.2 - 1.84 * v)
        # vs the old (16.7 - 0.81*v), i.e. 1.6-2.0x too small everywhere.
        #
        # Authority varies ~3x across the speed range, which is exactly why
        # a fixed brake_scale could never work and scheduling is required.
        #
        # NOTE the fit is only supported over v ~2-14 m/s (the braking
        # range of these runs). Extrapolated naively the line reaches zero
        # near 18.6 m/s, which is unphysical, so it is clamped both ends.
        # Re-fit before trusting it above ~50 km/h.
        self.BRAKE_AUTH_A0     = 34.2
        self.BRAKE_AUTH_A1     = 1.84
        self.BRAKE_AUTH_MAX    = 30.0
        # Authority assumed by the brake feed-forward. Deliberately the
        # strong end of the measured range, not the middle — see
        # brake_for_decel for why the bias direction matters.
        self.BRAKE_FF_AUTHORITY = 30.0
        # ── Speed governor ───────────────────────────────────────────────
        # Deceleration the reference profile is designed around. The
        # vehicle decelerates at this rate while it tracks the profile, so
        # this — not a brake constant — is what sets the deceleration.
        # Held below DECEL_LIMIT (5.0) so tracking error has somewhere to
        # go before the R171 ceiling is reached: the profile aims for 3.0
        # and the remaining 2.0 is margin for the speed loop's overshoot.
        self.ACC_PROFILE_DECEL = 3.0
        # How fast the reference may RISE again. Kept modest so a gap
        # estimate that jumps outward cannot command a throttle surge —
        # the previous run showed v_ref stepping back up 39.9 -> 44.4 km/h
        # on estimator noise, which released the brake mid-stop.
        self.ACC_PROFILE_ACCEL = 1.0
        # Speed band over which the profile deceleration ramps from the
        # comfort value up to DECEL_LIMIT. Below LO nothing changes, so
        # low-speed stops stay as gentle as before.
        self.ACC_DECEL_V_LO = 10.0   # 36 km/h
        self.ACC_DECEL_V_HI = 30.0   # 108 km/h
        # Largest speed error the governor will let build between the
        # vehicle and its reference [m/s]. With cruise_gain = 0.3 an error
        # of e produces ~0.3*e of brake, so 0.6 m/s keeps the proportional
        # contribution near 0.18 — about 5 m/s^2 even at the strong end of
        # the measured brake authority. Trim down if the late-stop
        # deceleration is still harsh, up if the stop feels sluggish.
        self.MAX_TRACK_ERROR_MPS = 0.6
        self.v_ref_last        = 0.0
        self.BRAKE_AUTH_MIN    = 6.0
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
        # CLOSING_ALPHA is vestigial: the alpha-beta tracker estimates the
        # closing rate directly, so there is nothing left to post-filter.
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
                        or self.track_d is None):
                    # Genuinely gone (or never seen) — hand back to CRUISE.
                    self._reset_tracker()
                    return
                # Bridge the miss by coasting the tracker. No special case
                # needed: prediction is what the tracker does anyway, so a
                # dropout is simply an update-free interval.
                return

            self.lead_miss_count = 0

            # ---- alpha-beta update ----
            if self.track_d is None:
                self.track_d, self.track_v, self.track_t = d, 0.0, now
                self.d_lead = d
                return

            dt = min(max(now - self.track_t, 1e-3), 0.5)
            d_pred = self.track_d + self.track_v * dt
            residual = d - d_pred
            self.track_d = d_pred + self.AB_ALPHA * residual
            self.track_v = self.track_v + (self.AB_BETA / dt) * residual
            self.track_t = now

        except Exception as e:
            self.get_logger().error(f"Lead distance callback error: {e}")

    def _reset_tracker(self):
        """Drop the lead track. Called when the miss tolerance expires or
        control hands back to CRUISE."""
        self.d_lead = None
        self.track_d = None
        self.track_v = 0.0
        self.track_t = None
        self.closing_rate_filt = 0.0

    def _tracked_gap(self):
        """Gap and closing rate predicted to NOW.

        Two corrections are applied on top of the tracker state:
          * `now - track_t` advances the estimate across the interval since
            the last perception frame (~82 ms at 12 Hz), so the 20 Hz
            control loop stops consuming a stale held value between frames.
          * PERCEPTION_LAG_S advances it across the camera->YOLO->IPM
            transport delay, so the control law sees where the lead is now
            rather than where it was a quarter of a second ago.

        Returns (gap [m] or None, closing_rate [m/s], negative = closing).
        """
        if self.track_d is None:
            return None, 0.0
        now = self.get_clock().now().nanoseconds / 1e9
        dt = min(max(now - self.track_t, 0.0), 0.5) + self.PERCEPTION_LAG_S
        return max(0.0, self.track_d + self.track_v * dt), self.track_v

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
          - The closing rate:    taken from the alpha-beta tracker state

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

        # -- Closing rate comes from the tracker, not a difference quotient --
        # closing_rate > 0  → gap is increasing  (lead pulls away)
        # closing_rate < 0  → gap is shrinking    (ego approaches lead)
        # It used to be (d_lead - prev_d_lead)/dt over the already-low-passed
        # distance, then low-passed again. Differentiating a filtered noisy
        # signal is the worst way to get a velocity: on far-range IPM data
        # (which swung 30+ m between frames) the result flipped sign
        # repeatedly, dropping the kinematic braking latch mid-stop. The
        # alpha-beta tracker estimates it as a state instead, so it is both
        # smoother and free of the extra filter delay.
        closing_rate = self.track_v
        self.closing_rate_filt = closing_rate

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
        # Feed-forward deliberately assumes the STRONGEST plausible brake.
        #
        # The speed-scheduled model this replaces cannot be trusted: the
        # same 50 km/h run measured authority 7.6 at 45 km/h and 34.7 at
        # 21 km/h — a 4.6x swing that no linear fit reproduces (the refit
        # predicts only 2.1x). With the plant gain uncertain by that much,
        # the direction of the modelling error decides whether the vehicle
        # overshoots the deceleration limit or merely takes longer to
        # reach it, and only one of those is acceptable in a law meant to
        # BOUND deceleration.
        #
        # Assuming a strong brake makes the opening command small, so a
        # plant that turns out stronger than expected cannot overshoot;
        # the integrator supplies the rest when it turns out weaker.
        # Simulated across the measured authority range, target 5 m/s^2:
        #     ff assumes 8.6  -> peak 5.3 / 10.0 / 13.6   (overshoots badly)
        #     ff assumes 30   -> peak 5.0 /  5.8 /  6.9   (bounded)
        # The cost is onset time when the brake really is weak (~1.1 s to
        # reach target instead of 0.25 s), i.e. stopping distance traded
        # for a bounded peak. That is the correct trade here: collisions
        # are already avoided up to ~70 km/h; the limit is what is not met.
        ff = target / self.BRAKE_FF_AUTHORITY

        # What the loop WANTS this tick, before any actuator limits.
        requested = ff + self.brake_kp * error + self.brake_integral
        commanded = max(0.0, min(requested, 1.0))
        # What the actuator can actually deliver: rising is rate-limited,
        # releasing is not (backing off must be immediate).
        applied = min(commanded, self.prev_acc_brake + self.BRAKE_RATE_UP)
        applied = max(applied, self.prev_acc_brake - self.BRAKE_RATE_DOWN)
        applied = max(0.0, min(applied, 1.0))

        # ---- anti-windup ----
        # Integrate ONLY while the actuator is actually following the
        # request. The previous test (`unsat < 1.0`) let the integrator
        # keep charging all the way through the rate-limited ramp: during
        # the ~0.3 s it takes to ramp to the feed-forward value the
        # measured deceleration is still climbing, so the error stays
        # positive, and by the time it clears the integrator has added
        # enough to drive the command far past what was needed. Measured:
        # the brake ramped to 0.70 where ~0.58 was the feed-forward and
        # ~0.25 turned out to be enough, then had to unwind to zero —
        # the ramp/collapse/ramp cycle in the traces.
        #
        # This is the standard remedy for an actuator with a rate limit:
        # a rate-limited actuator is a *saturated* actuator for as long as
        # it is ramping, and you must not integrate into a saturation.
        tracking = (abs(applied - commanded) < 1e-9
                    and 0.0 < applied < 1.0)
        if tracking:
            self.brake_integral += self.brake_ki * error * self.CRUISE_DT
            self.brake_integral = max(min(self.brake_integral,
                                          self.brake_i_limit),
                                      -self.brake_i_limit)

        self.prev_acc_brake = applied
        return applied

    # ====================================================================
    # CRUISE CONTROL (fallback when no lead vehicle detected)
    # ====================================================================

    # ====================================================================
    # SPEED GOVERNOR  (distance -> reference speed)
    # ====================================================================

    def speed_reference(self, gap: float, closing_rate: float,
                        v_ego: float) -> float:
        """The speed the vehicle should be doing right now, given the gap.

        This replaces commanding a force or an acceleration. Braking is
        expressed as a SPEED PROFILE and handed to the speed loop:

            v_ref = v_lead + sqrt(2 * a_profile * (gap - d_desired))

        Two properties make this the right shape for a law that has to
        BOUND deceleration:

        1. Differentiating the profile along the trajectory gives exactly
           -a_profile. So while the vehicle tracks it, the deceleration is
           whatever we designed, not whatever the brake happened to do.
        2. It closes the loop on SPEED, which is measured directly, at
           20 Hz, with no meaningful lag. Every previous attempt closed on
           deceleration — differentiated, filtered, and delayed — or fed
           brake forward through a gain we could not predict.

        That gain is the reason for the rewrite. Measured cleanly, brake
        authority (achieved decel / brake command) ranges 8-49 across the
        run and varies 2-3x WITHIN a single speed band, so it is not a
        function of speed we failed to fit — it is not predictable at all.
        No feed-forward survives that. A speed loop does, because its
        integrator simply finds whatever brake produces the required
        speed, and a gain error shows up as a small speed error rather
        than a deceleration overshoot.

        The v_lead term generalises it to a moving lead: with the lead
        matching our speed the sqrt term handles the gap error alone, and
        with a stationary lead v_lead is 0 and this reduces to the plain
        distance-to-stop profile.
        """
        v_lead = max(0.0, v_ego + closing_rate)
        # Headway is referenced to the LEAD's speed, not the ego's.
        #
        # With d_safe = d0 + T_gap*v_ego the profile asks the vehicle to
        # stop at its own current-speed headway: at 50 km/h that is 22.8 m,
        # so v_ref hit zero while the gap was still 20 m and the car would
        # have halted 20 m short of a stationary target. Referencing the
        # lead makes the policy degenerate correctly — a stationary lead
        # gives d_safe = d0, so the profile runs all the way down to the
        # standstill gap — while still yielding a T_gap-second headway
        # behind a moving one (as gap -> d_safe, v_ref -> v_lead).
        d_safe = self.d0 + self.T_gap * v_lead
        gap_err = gap - d_safe

        # Profile deceleration scales with speed: the faster we are going,
        # the more of the available budget the profile is allowed to use.
        # Below ACC_DECEL_V_LO the comfort value is kept unchanged; above
        # ACC_DECEL_V_HI the profile is allowed the full DECEL_LIMIT.
        #
        # Note the onset distance, d0 + v^2/(2a), already grows with the
        # SQUARE of speed, so braking starts far earlier at speed whatever
        # `a` is (12.9 m at 30 km/h, ~132 m at 130 km/h). What this adds is
        # the other half of the request — braking harder, not just sooner —
        # which matters because at high speed a fixed comfort rate cannot
        # bring the car down inside the distance perception actually gives.
        span = max(1e-3, self.ACC_DECEL_V_HI - self.ACC_DECEL_V_LO)
        frac = (v_ego - self.ACC_DECEL_V_LO) / span
        frac = max(0.0, min(1.0, frac))
        a = (self.ACC_PROFILE_DECEL
             + (self.DECEL_LIMIT - self.ACC_PROFILE_DECEL) * frac)

        if gap_err >= 0.0:
            v_ref = v_lead + math.sqrt(2.0 * a * gap_err)
        else:
            # Inside the desired gap — target below the lead's speed so the
            # gap reopens, rather than merely matching and staying close.
            v_ref = v_lead - math.sqrt(2.0 * a * (-gap_err))

        v_ref = max(0.0, min(v_ref, self.target_speed))

        # ---- rate-limit the reference ----
        # The gap estimate still steps: measured, the tracked gap moved
        # 27.7 -> 24.6 m within one 50 ms tick, which threw v_ref from
        # 49.3 to 44.7 km/h and had the speed loop slamming the brake from
        # 0.31 to 0 to 0.53 chasing it. The loop was tracking correctly;
        # the reference was not physically realisable.
        #
        # Since deceleration equals -dv_ref/dt while the vehicle tracks the
        # profile, bounding the reference's rate of change bounds the
        # commanded deceleration DIRECTLY — no matter how noisy the gap
        # estimate is. This is what actually makes the limit hold: it moves
        # the guarantee out of the (unpredictable) brake plant and into the
        # trajectory, where it is arithmetic.
        # Rate-limit the DESCENT at the hard limit, not the comfort target.
        #
        # Limiting it to ACC_PROFILE_DECEL looked right — the profile's own
        # time-derivative is exactly that while it is being tracked — but
        # it is only true when tracking is perfect. Running ~3 km/h above
        # the reference, the gap closes faster than the profile assumed, so
        # v_prof(gap(t)) falls faster than the comfort rate and the limiter
        # blocked the reference from ever catching up. Measured: the
        # reference marched down a straight 3.00 m/s^2 line while the true
        # profile dropped away beneath it, and the vehicle was still doing
        # 23.5 km/h with 2.00 m of gap left. It hit the target.
        #
        # The profile still ASKS for ACC_PROFILE_DECEL in the nominal case;
        # this bound only binds when the reference has to catch up, or when
        # estimator noise would otherwise demand something absurd. Placing
        # it at DECEL_LIMIT keeps the R171 guarantee intact while leaving
        # the authority to actually stop.
        dv_down = self.DECEL_LIMIT * self.CRUISE_DT
        dv_up = self.ACC_PROFILE_ACCEL * self.CRUISE_DT
        v_ref = max(v_ref, self.v_ref_last - dv_down)
        # Ratchet. While the gap is CLOSING the reference may fall but not
        # rise. The tracked gap still wobbles a metre or so frame to frame
        # (24.76 -> 25.25 -> 24.50 measured), and without this the profile
        # wobbles with it: v_ref climbed +0.36 twice mid-stop, the brake
        # released to zero, deceleration sagged from 3.9 to 2.3 — and the
        # distance lost there had to be paid back at the end as 6.2 m/s^2.
        # That release WAS the harsh late braking.
        #
        # Rising is still allowed once the gap genuinely opens (the lead
        # pulling away, closing_rate >= 0), so this does not trap the
        # vehicle at a low reference behind a car that has moved on.
        if closing_rate < 0.0:
            v_ref = min(v_ref, self.v_ref_last)
        else:
            v_ref = min(v_ref, self.v_ref_last + dv_up)

        # An earlier version clamped v_ref to v_ego - 0.6 m/s here to stop
        # the tracking error growing. It backfired: holding the reference
        # just under the vehicle starves the loop of the error signal it
        # needs, and with the integrator still carrying the cruise throttle
        # (~2.15, what held 50 km/h) the loop stayed net-POSITIVE and drove
        # throttle into a stationary target. Unwinding at ki*e = 0.09/s
        # would have taken ~12 s; the car arrived first, at 24.6 km/h.
        #
        # The reference must be free to lead. Bounding actual deceleration
        # is the rate limit's job above; keeping the error small is the
        # speed loop's, via the integral handover in speed_control.
        return max(0.0, v_ref)

    def speed_control(self, v_target: float, integrate: bool = True,
                      brake_cap: float = 1.0) -> tuple:
        """PI speed tracking. Shared by CRUISE and ACC.

        CRUISE passes the set speed; ACC passes the governor's reference.
        Having one loop for both removes the old
        `min(acc_throttle, cruise_throttle)` / `max(acc_brake, cruise_brake)`
        combination, which existed only because two independent controllers
        were fighting over the same actuator.

        `brake_cap` lets CRUISE keep its gentle 0.6 ceiling for bleeding off
        overshoot while ACC gets full authority.
        """
        speed_error = v_target - self.v_ego

        # Bumpless handover between accelerating and braking. The
        # integrator holds whatever throttle sustained the PREVIOUS
        # setpoint, and that bias survives into a braking event — it is
        # what kept throttle at 0.43 while closing on a stationary target.
        # Once the required action reverses sign the stored term is no
        # longer the right answer, so dropping it costs nothing and
        # removes a multi-second delay before the brake is reached.
        if speed_error < 0.0 and self.cruise_integral > 0.0:
            self.cruise_integral = 0.0
        elif speed_error > 0.0 and self.cruise_integral < 0.0:
            self.cruise_integral = 0.0

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
            self.prev_acc_brake = 0.0
            return (min(u, self.cruise_throttle_cap), 0.0)

        if self.simulator == 'morai':
            # MORAI's brake leaves the vehicle stuck rather than
            # decelerating smoothly (DEBUG.md) — coast instead.
            self.prev_acc_brake = 0.0
            return (0.0, 0.0)

        if u > -self.CRUISE_BRAKE_DEADBAND:
            self.prev_acc_brake = max(0.0, self.prev_acc_brake
                                      - self.BRAKE_RATE_DOWN)
            return (0.0, self.prev_acc_brake)

        brake = min(-u, brake_cap)
        # Rate-limit both directions. Rising slowly keeps the loop from
        # slamming the brake before the speed response is visible; falling
        # slowly stops the release/re-apply chatter seen in earlier runs.
        brake = min(brake, self.prev_acc_brake + self.BRAKE_RATE_UP)
        brake = max(brake, self.prev_acc_brake - self.BRAKE_RATE_DOWN)
        brake = max(0.0, min(brake, brake_cap))
        self.prev_acc_brake = brake
        return (0.0, brake)

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
        # Bumpless handover between accelerating and braking. The
        # integrator holds whatever throttle sustained the PREVIOUS
        # setpoint, and that bias survives into a braking event — it is
        # what kept throttle at 0.43 while closing on a stationary target.
        # Once the required action reverses sign the stored term is no
        # longer the right answer, so dropping it costs nothing and
        # removes a multi-second delay before the brake is reached.
        if speed_error < 0.0 and self.cruise_integral > 0.0:
            self.cruise_integral = 0.0
        elif speed_error > 0.0 and self.cruise_integral < 0.0:
            self.cruise_integral = 0.0

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

        # Refresh the gap from the tracker every tick. Previously d_lead
        # was only written when a perception message arrived (~12 Hz), so
        # between frames this 20 Hz loop re-used a stale value that was
        # already ~123 ms behind reality thanks to the old low-pass. The
        # tracker predicts to the current instant instead, which is where
        # most of the recovered dead time comes from.
        tracked, _ = self._tracked_gap()
        if tracked is not None:
            self.d_lead = tracked
            self.gap_pub.publish(Float32(data=float(tracked)))
        self.vref_pub.publish(Float32(data=float(self.v_ref_last)))

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
            self.v_ref_last = self.v_ego
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
            self.v_ref_last = self.v_ego
            self._publish_mode('STANDSTILL')
            self.control_pub.publish(control_msg)
            self.prev_throttle = 0.0
            if ENABLE_LOGGING:
                self._log_throttled("STANDSTILL", 0.0, 0.05)
            return

        # ---- MODE 2: CRUISE (no lead vehicle detected) ----
        if self.d_lead is None:
            # Same loop the ACC branch uses, just tracking the set speed.
            # 0.6 brake ceiling: cruise only ever bleeds off overshoot,
            # it never has to stop for anything.
            throttle, brake = self.speed_control(self.target_speed,
                                                 brake_cap=0.6)

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
            self.v_ref_last = self.v_ego
            self._publish_mode('CRUISE')
            self.control_pub.publish(control_msg)

            # Closing rate is a tracker state now; prev_d_lead/prev_time
            # were the old difference-quotient scratch and are unused.

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
            self.v_ref_last = self.v_ego
            self._publish_mode('EMERGENCY')
            self.control_pub.publish(control_msg)
            self.prev_throttle = 0.0
            if ENABLE_LOGGING:
                self._log_throttled("EMERGENCY", 0.0, 1.0)
            return

        # ---- MODE 4: ACC (speed governor) ----
        # The governor turns the gap into a reference SPEED; the speed loop
        # tracks it. Deceleration is set by the shape of the profile rather
        # than by a brake command, so it is bounded by design instead of by
        # calibration — which matters because brake authority here is not
        # predictable (8-49, and 2-3x within a single speed band).
        #
        # This also retires the old min(acc_throttle, cruise_throttle) /
        # max(acc_brake, cruise_brake) combination: the governor already
        # clamps its reference to target_speed, so "follow the lead OR hold
        # set speed, whichever is slower" falls out of one loop instead of
        # two controllers arguing over one actuator.
        v_ref = self.speed_reference(self.d_lead, self.track_v, self.v_ego)
        self.v_ref_last = v_ref
        throttle, brake = self.speed_control(v_ref)

        if brake > 0.0:
            self.last_acc_brake = brake
            self.last_acc_brake_t = self.get_clock().now().nanoseconds / 1e9

        # Rate limit throttle — braking is rate-limited inside speed_control
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