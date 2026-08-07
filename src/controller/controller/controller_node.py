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
# MORAI gain softening. Applied only to the old PD law's k_p/k_d, which
# the speed governor replaced, so nothing reads it now -- MORAI-side
# softening lives in cruise_throttle_cap instead. Kept as a marker that
# MORAI needed 10x softer gains than CARLA once, in case the speed loop
# turns out to need the same treatment when MORAI comes back.
ACC_GAIN_SCALE_MORAI = 0.1   # unused


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
        # Speed error [m/s] beyond which a sign reversal is treated as real
        # rather than as noise around a tracked reference.
        self.INTEGRAL_FLIP_DEADBAND = 0.3
        # Integral shed per tick during a real reversal. 0.5 at 20 Hz
        # unwinds the ~2.15 cruise bias in ~0.25 s.
        self.INTEGRAL_FLIP_DECAY = 0.5
        # ── Powertrain model ─────────────────────────────────────────────
        # Measured CARLA coast-down: deceleration [m/s^2] with throttle AND
        # brake at zero, by speed [m/s]. This is NOT a small correction —
        # at 22-29 km/h the vehicle sheds ~4.9 m/s^2 uncommanded, which
        # alone exceeds the R171 ceiling. See DEBUG §45.2.
        # Indexed by band CENTRE. The first version used each band's lower
        # edge, shifting the whole curve 1 m/s: at 9 m/s it read 4.06 where
        # the measurement is 6.07, so the loop under-counted engine braking
        # by ~2 m/s^2 and commanded that much extra brake — right at the
        # speed where the deceleration peaks were landing.
        #
        # Medians, not means: the samples are few per band (8-29) and the
        # tail is skewed by gear changes. The non-monotonic shape (rising
        # to 6.07 at 9 m/s, falling to 2.33 at 11) is real and is almost
        # certainly the gearbox — which is also why this table cannot be
        # replaced by a smooth drag curve.
        # Re-measured from a CLEAN coast-down: a full 50 km/h -> standstill
        # with throttle and brake at zero for all 113 frames, rather than
        # scraped out of mixed control data. 1 m/s bands.
        #
        # The double peak (6.57 at 5.5 m/s, 6.41 at 9.5, dipping to 3.72
        # between) is two gearbox downshifts. It is real, it is why this
        # cannot be a smooth drag curve, and it is why deceleration used to
        # swing by 3 m/s^2 for no commanded reason.
        #
        # Note the peak EXCEEDS the 5 m/s^2 R171 ceiling with no command at
        # all — see §45.2. Also note 6.57 m/s^2 of engine braking is not
        # plausible for a real Charger (1-2 would be), so this remains a
        # CARLA powertrain characteristic rather than a vehicle one.
        self.COAST_DECEL = ((0.5, 0.81), (1.5, 1.36), (2.5, 2.18),
                            (3.5, 3.29), (4.5, 4.40), (5.5, 6.57),
                            (7.5, 3.72), (8.5, 4.60), (9.5, 6.41),
                            (10.5, 2.79), (11.5, 2.94), (12.5, 3.16),
                            (13.5, 2.73))
        # Deceleration per unit brake, with coast removed and the response
        # lag aligned (n=54; best residual at ~104 ms, giving 4.44 — the
        # unaligned fit gave 6.60, so the number is lag-sensitive and only
        # good to roughly +/-1). The earlier figure of "8 to 49,
        # unpredictable" was this same regression with engine braking left
        # in the residual.
        self.BRAKE_AUTHORITY = 4.44
        # Acceleration per unit throttle. Cross-checked against the cruise
        # trim: ~0.43 throttle holds 50 km/h where coast is ~2.4 m/s^2.
        self.THROTTLE_AUTHORITY = 6.0
        # Quadratic slew on the actuator-acceleration command [m/s^2 per
        # tick]. K sets the curvature, MIN keeps it converging, MAX caps
        # how violently it may move. At 20 Hz, MAX=0.6 allows a full
        # 5 m/s^2 swing in ~0.4 s.
        self.A_ACT_SLEW_K = 0.25
        self.A_ACT_SLEW_MIN = 0.05
        self.A_ACT_SLEW_MAX = 0.60
        self._a_act_prev = 0.0
        # Speed error [m/s] beyond which coast compensation is withheld.
        # Small, so it acts as hysteresis at the crossing rather than as a
        # band in which behaviour changes.
        self.NO_THROTTLE_DEADBAND = 0.15
        # Floor on the brake command whenever the loop is not throttling.
        # 0.1 is worth ~0.44 m/s^2 on top of engine braking — enough to
        # make the transition decisive without dominating it.
        self.MIN_BRAKE_NO_THROTTLE = 0.1
        # Even at gain_scale=0.1, cruise throttle still hit 1.0 on first
        # MORAI runs (speed feedback was/may still be unreliable enough
        # that speed_error stays large). Hard-cap MORAI's cruise throttle
        # well below full send regardless of what the gain computes;
        # CARLA keeps its original 1.0 ceiling.
        self.cruise_throttle_cap = 0.8 if simulator == 'morai' else 1.0
        self.a_max             = 3.0       # [m/s²]
        # Emergency threshold is now a bumper gap, not a camera distance.
        # IPM also saturates near gap ≤ 2 m (bb-bottom clips at frame
        # edge → reported gap drops well below truth) — the saturated
        # value still falls under this 3 m threshold, so saturation
        # itself trips EMERGENCY without any special branch. See §22.
        self.emergency_distance = 1.0      # full brake below this gap [m]
        self.prev_throttle     = 0.0
        self.THROTTLE_RATE_LIMIT = 0.05  # max throttle increase per step (20 Hz → 1.0 in ~1 second)
        # Control-loop period, used by the cruise integrator. Must match
        # the create_timer() period at the bottom of __init__.
        self.CRUISE_DT = 0.05

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
        # Lowered 0.45 -> 0.20. The published gap arrives as a staircase
        # (discrete IPM values, ~5 m steps at range), and at 0.45 the
        # tracker followed the steps closely enough that the derived
        # closing rate changed sign on 27% of samples while closing on a
        # STATIONARY target. That is what releases the ratchet. Heavier
        # smoothing costs nothing here because the tracker predicts to now
        # — the usual reason not to smooth is the lag it adds, and the
        # prediction already removes that.
        self.AB_ALPHA          = 0.20
        # Largest relative acceleration between two vehicles that the
        # tracker will believe [m/s^2]. Anything faster is noise.
        self.AB_MAX_REL_ACCEL  = 6.0
        # Slack on the |closing rate| <= v_ego bound, for a lead genuinely
        # moving toward us or pulling away hard.
        self.AB_CLOSING_MARGIN = 3.0
        self.AB_BETA           = (self.AB_ALPHA ** 2) / (2.0 - self.AB_ALPHA)
        self.track_d           = None   # estimated gap [m]
        self.track_v           = 0.0    # estimated closing rate [m/s], -ve = closing
        self.track_t           = None   # timestamp of the last tracker update
        self._track_seeded     = False  # velocity seeded from the first interval
        # Perception transport lag, measured by comparing the published
        # distance against CARLA ground truth: the over-read scaled with
        # speed (+1.95 m at 9.6 m/s, +1.44 m at 5.0 m/s) and collapsed to
        # a near-constant ~0.25 s when divided by it — a latency, not a
        # calibration bias. The tracker predicts forward by this much so
        # the control law sees where the lead is NOW, not where it was.
        self.PERCEPTION_LAG_S  = 0.25
        # Consecutive no-detection frames bridged before the lead is
        # declared gone. 3 at the perception rate covers the 1-3 frame
        # dropouts measured in the scenario traces without holding a stale
        # track long enough to matter. Note the trade: if the lead really
        # does leave (a lane change, not a missed frame), ACC keeps acting
        # on it for up to 3 frames.
        self.LEAD_MISS_TOLERANCE = 3
        self.lead_miss_count   = 0

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
        # ── Speed governor ───────────────────────────────────────────────
        # Deceleration the reference profile is designed around. The
        # vehicle decelerates at this rate while it tracks the profile, so
        # this — not a brake constant — is what sets the deceleration.
        # Held below DECEL_LIMIT (5.0) so tracking error has somewhere to
        # go before the R171 ceiling is reached: the profile aims for 3.0
        # and the remaining 2.0 is margin for the speed loop's overshoot.
        # Fallback profile deceleration, used whenever the acquisition
        # latch is not armed.
        #
        # Lowered 3.0 -> 1.2. A gentler profile demands slowing EARLIER,
        # because braking starts where sqrt(2a(gap-d0)) first falls below
        # the set speed — d0 + v^2/(2a). At 3.0 that is 34 m at 50 km/h,
        # so the vehicle held full speed for 35 m after seeing the target
        # at 69 m. At 1.2 it is 82 m, i.e. the profile is already asking
        # for a reduction by the time anything is detected, at every speed
        # in the matrix.
        #
        # This deliberately does NOT rely on the latch. The latch computes
        # the ideal rate from the acquisition gap, but it can only arm once
        # the tracker has a real closing rate, and anything that delays
        # that (estimator noise, damping, a late first fix) delays braking
        # with it. The fallback now brakes early on its own; the latch
        # refines it rather than being the only thing that triggers it.
        #
        # Gentler is also the safer direction: the profile converges to
        # d_safe as the gap closes regardless of `a`, so a low value costs
        # a longer, softer approach — not a missed stop.
        self.ACC_PROFILE_DECEL = 1.2
        # How fast the reference may RISE again. Kept modest so a gap
        # estimate that jumps outward cannot command a throttle surge —
        # the previous run showed v_ref stepping back up 39.9 -> 44.4 km/h
        # on estimator noise, which released the brake mid-stop.
        self.ACC_PROFILE_ACCEL = 1.0
        # Speed band over which the profile deceleration ramps from the
        # comfort value up to DECEL_LIMIT. Below LO nothing changes, so
        # low-speed stops stay as gentle as before.
        # Gentlest profile the latch may choose. Without a floor a very
        # distant acquisition produces an absurdly slow crawl.
        self.ACC_PROFILE_DECEL_MIN = 0.7
        # Slightly gentler than the strictly-required rate, so v_ref starts
        # just BELOW the current speed and the loop engages at once instead
        # of sitting exactly on the profile commanding nothing.
        self.ACC_LATCH_MARGIN = 0.95
        # Closing speed below which the latch releases — a lead matching our
        # speed is not something to plan a stop for.
        self.ACC_LATCH_MIN_CLOSING = 0.8   # m/s
        # How much faster than the latched plan the reference may descend
        # when catching up after a gap-estimate step.
        self.REF_DESCENT_MARGIN = 2.0
        self.REF_DESCENT_FLOOR = 2.0   # m/s^2
        self._a_latch = None
        self.v_ref_last        = 0.0
        # Brake hold across a lost lead: see control_loop's CRUISE branch.
        self.last_acc_brake    = 0.0
        self.last_acc_brake_t  = None
        self.ACC_BRAKE_HOLD_S  = 0.6
        self.prev_acc_brake    = 0.0
        self.d_lead            = None
        self.last_log_time     = 0.0

        # ── Control loop @ 20 Hz ─────────────────────────────────────────
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"ACC Node initialized | target={self.target_speed:.1f} m/s | "
            f"d0={self.d0} m | T_gap={self.T_gap} s | "
            f"profile={self.ACC_PROFILE_DECEL} m/s^2 | "
            f"limit={self.DECEL_LIMIT} m/s^2"
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
                self._track_seeded = False
                self.d_lead = d
                return

            dt = min(max(now - self.track_t, 1e-3), 0.5)

            if not self._track_seeded:
                # Seed the velocity from the very first interval instead of
                # letting it converge from zero.
                #
                # The damping clamp limits velocity revisions to
                # AB_MAX_REL_ACCEL, which is right for rejecting staircase
                # noise but wrong at acquisition: starting from 0 it takes
                # ~2.3 s to reach a 13.9 m/s closing rate. During that the
                # governor sees almost no closing, the latch cannot arm,
                # and braking is simply late — which is why the throttle
                # only began dropping ~0.8 s after the target was acquired.
                self.track_v = max(-(self.v_ego + self.AB_CLOSING_MARGIN),
                                   min(self.v_ego + self.AB_CLOSING_MARGIN,
                                       (d - self.track_d) / dt))
                self.track_d = d
                self.track_t = now
                self._track_seeded = True
                return
            d_pred = self.track_d + self.track_v * dt
            residual = d - d_pred
            self.track_d = d_pred + self.AB_ALPHA * residual

            # ---- damped velocity update ----
            # The raw alpha-beta velocity update is (beta/dt) * residual,
            # which at 12 Hz perception works out at 0.271 * residual. The
            # published gap arrives as a staircase, so a single 10 m step
            # kicks the velocity state by 2.7 m/s — and that state is not
            # a private detail: it multiplies the 0.25 s prediction, and it
            # IS the closing rate the governor and the ratchet run on.
            # Measured, the tracker was reducing sample-to-sample jitter by
            # only 6% (0.449 -> 0.424 m) for exactly this reason.
            #
            # Damped by physics rather than by a tuning constant: the
            # RELATIVE acceleration between two road vehicles cannot exceed
            # AB_MAX_REL_ACCEL, so a velocity revision implying more than
            # that over the sample interval is measurement noise, not
            # motion, and is clipped to what is physically reachable.
            v_new = self.track_v + (self.AB_BETA / dt) * residual
            dv_max = self.AB_MAX_REL_ACCEL * dt
            v_new = max(self.track_v - dv_max,
                        min(self.track_v + dv_max, v_new))

            # A lead cannot recede faster than we are driving unless it is
            # genuinely accelerating away, and cannot approach faster than
            # our own speed unless it is driving at us. Bounding on v_ego
            # keeps a far-range outlier from inventing a closing rate the
            # geometry cannot support.
            self.track_v = max(-(self.v_ego + self.AB_CLOSING_MARGIN),
                               min(self.v_ego + self.AB_CLOSING_MARGIN,
                                   v_new))
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
        self._track_seeded = False
        # Drop the acquisition latch with the track: the next lead is a new
        # plan, not a continuation of this one.
        self._a_latch = None

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


    # ====================================================================
    # CLOSED-LOOP BRAKE (deceleration tracking)
    # ====================================================================


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

        Deceleration is the reason for the rewrite, though not for the
        reason first recorded. Deceleration here is dominated by ENGINE
        BRAKING, which the controller does not command: coasting sheds
        ~4.9 m/s² at 22-29 km/h and peaks above 10 (DEBUG §45.2). A law
        that computes a brake force is therefore controlling the smaller
        half of the problem. A speed loop closes on the quantity that
        actually matters — the resulting speed — and its integrator finds
        whatever throttle or brake delivers it, engine braking included.

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

        # Profile deceleration: latched at ACQUISITION so braking begins
        # at first detection rather than at a fixed crossing distance.
        #
        # The profile only demands braking once v_ref falls below the set
        # speed, which for a fixed `a` happens at d0 + v^2/(2a) — about
        # 30 m at 50 km/h. With detection now reaching ~99 m that wasted
        # 70 m of sight: the vehicle coasted at full speed, then braked
        # hard over what was left. Latching `a` to the deceleration that
        # stops from the ACQUISITION gap makes v_ref start at (just under)
        # the current speed, so braking begins immediately and the stop is
        # spread over every metre available:
        #
        #     a = v_close^2 / (2 * (gap - d_safe))
        #
        # At 99 m and 50 km/h that is ~0.99 m/s^2 — the gentlest stop that
        # still works, and far under the limit. Acquiring late (34 m at
        # 70 km/h) yields 5.9 and clamps to DECEL_LIMIT, which is the
        # honest statement that the sighting was too late for comfort.
        #
        # Latched rather than recomputed each tick because the two are not
        # the same: recomputing gives a == a_needed every tick, so v_ref
        # tracks v exactly and the loop never asks for anything. The latch
        # fixes the plan at acquisition; the profile below then converges
        # to d_safe on its own as the gap closes, so a latch that turns out
        # slightly optimistic self-corrects rather than compounding.
        #
        # Released when the lead is lost or is no longer being closed on,
        # so a car matching our speed at 200 m does not trigger a stop.
        v_close_now = max(0.0, -closing_rate)
        if v_close_now < self.ACC_LATCH_MIN_CLOSING:
            self._a_latch = None
        elif self._a_latch is None and gap_err > 0.5:
            self._a_latch = max(self.ACC_PROFILE_DECEL_MIN,
                                min(self.DECEL_LIMIT,
                                    self.ACC_LATCH_MARGIN
                                    * v_close_now * v_close_now
                                    / (2.0 * gap_err)))
        a = self._a_latch if self._a_latch is not None else self.ACC_PROFILE_DECEL
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
        # the guarantee out of the brake plant — which does not command
        # most of the deceleration anyway (§45.2) — and into the
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
        # Descent is bounded by the PLAN, not by the ceiling.
        #
        # DECEL_LIMIT was the wrong bound here. The latched plan asks for
        # ~1.3 m/s^2 on a 72 m acquisition, but at a 5.0 limit the
        # reference was free to chase every downward step in the gap
        # estimate — measured, it descended at up to 9.43 m/s^2, four
        # times the plan, on 33 of 265 samples. Each of those opened a
        # tracking error (the vehicle ran 7 km/h above the reference)
        # which the speed loop then answered with maximum deceleration.
        #
        # Bounding it at a multiple of the latched rate lets the reference
        # catch up after a genuine step without letting estimator noise
        # dictate the deceleration. The floor keeps a plan that latched
        # very gently from becoming unable to respond at all — the failure
        # mode from limiting descent to a fixed comfort rate, which left
        # the vehicle at 23.5 km/h with 2 m of gap left.
        plan = self._a_latch if self._a_latch is not None else self.ACC_PROFILE_DECEL
        dv_down = min(self.DECEL_LIMIT,
                      max(self.REF_DESCENT_FLOOR,
                          self.REF_DESCENT_MARGIN * plan)) * self.CRUISE_DT
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

    def _slew_a_act(self, target: float) -> float:
        """Move the actuator-acceleration command toward `target` on a
        quadratic profile, and return the new value.

        Replaces the two independent LINEAR rate limits that used to sit on
        throttle and brake separately. Because those were separate, every
        crossing of zero collapsed one channel and started the other from
        scratch — the sawtooth in the command trace, and the reason the
        throttle appeared to drop off a cliff rather than taper.

        `a_act` is a single continuous axis through zero (positive =
        throttle, negative = brake), so limiting it once removes the
        discontinuity by construction: near the crossing the step is small,
        so the vehicle eases through it instead of switching.

        Quadratic rather than linear: step magnitude grows with the SQUARE
        of the distance still to cover, so a large correction starts fast
        and settles gently, which is what a linear ramp cannot do — it
        arrives at full slope and stops dead. A linear floor is kept
        underneath so the approach still converges in finite time rather
        than asymptoting.
        """
        err = target - self._a_act_prev
        mag = abs(err)
        step = min(self.A_ACT_SLEW_MAX,
                   max(self.A_ACT_SLEW_K * mag * mag,
                       self.A_ACT_SLEW_MIN))
        if step > mag:
            step = mag
        self._a_act_prev += math.copysign(step, err)
        return self._a_act_prev

    def coast_decel(self, v_mps: float) -> float:
        """Deceleration the powertrain applies with no throttle and no
        brake, interpolated from the measured coast-down table."""
        tbl = self.COAST_DECEL
        if v_mps <= tbl[0][0]:
            return tbl[0][1]
        for (v0, d0), (v1, d1) in zip(tbl, tbl[1:]):
            if v0 <= v_mps <= v1:
                f = (v_mps - v0) / (v1 - v0) if v1 > v0 else 0.0
                return d0 + f * (d1 - d0)
        return tbl[-1][1]

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
        # Only a MEANINGFUL reversal counts, and the term is bled down
        # rather than dropped.
        #
        # The first version zeroed the integrator on any sign change. That
        # fixed the real bug (a cruise bias of ~2.15 held throttle into a
        # stationary target for ~12 s) but created another: while tracking
        # a descending reference the error sits near zero and crosses sign
        # constantly, so the integrator was wiped repeatedly. Measured,
        # that collapsed throttle 0.36 -> 0.00 in one tick on an error of
        # -0.02 m/s, six times in a single stop — the throttle sawtooth.
        # The step came entirely from the reset; the proportional term was
        # worth 0.007 there.
        #
        # The deadband keeps noise-level crossings out of it; the decay
        # unwinds a genuine bias in ~0.25 s — fast enough that it cannot
        # hold throttle into a braking event, smooth enough to taper.
        if (speed_error < -self.INTEGRAL_FLIP_DEADBAND
                and self.cruise_integral > 0.0):
            self.cruise_integral = max(0.0, self.cruise_integral
                                       - self.INTEGRAL_FLIP_DECAY)
        elif (speed_error > self.INTEGRAL_FLIP_DEADBAND
                and self.cruise_integral < 0.0):
            self.cruise_integral = min(0.0, self.cruise_integral
                                       + self.INTEGRAL_FLIP_DECAY)

        u_p = self.cruise_gain * speed_error
        if integrate:
            u_unsat = u_p + self.cruise_ki * self.cruise_integral
            if -1.0 < u_unsat < self.cruise_throttle_cap:
                self.cruise_integral += speed_error * self.CRUISE_DT
                self.cruise_integral = max(min(self.cruise_integral,
                                               self.cruise_i_limit),
                                           -self.cruise_i_limit)
        u = u_p + self.cruise_ki * self.cruise_integral

        # ---- coast-aware split ----
        # `u` is a desired ACCELERATION in disguise, so convert it and add
        # back what the powertrain is already doing. Zero command is not
        # zero acceleration in this vehicle: releasing the throttle at
        # 25 km/h yields ~5.2 m/s^2 of engine braking on its own.
        #
        # Splitting at u = 0, as before, therefore mis-attributed the whole
        # coast term. Asking for a gentle 2 m/s^2 produced a brake command
        # anyway, which landed on top of 5.2 and overshot; the vehicle then
        # fell below the reference and the loop opened the throttle to
        # recover. That is the 0.4-brake-then-throttle-surge pattern in the
        # traces — the brake was not too strong in itself (0.4 is worth
        # only 1.74 m/s^2), it was simply added to something unaccounted.
        #
        # Splitting at `a_des + coast` instead makes the command one
        # CONTINUOUS axis through zero: as the demand softens, brake falls
        # to zero and throttle takes over smoothly to hold the deceleration
        # UP at the requested value rather than letting it run away. No
        # discontinuity, and no brake at all unless coasting is genuinely
        # insufficient.
        a_des = max(-self.DECEL_LIMIT,
                    min(u * self.THROTTLE_AUTHORITY, self.a_max))

        # Coast compensation may cancel engine braking, never exceed it.
        #
        # Holding throttle against the powertrain is the whole point of the
        # split, but it is only ever meant to SOFTEN a deceleration. The
        # coast table is measured, coarse (8-29 samples per band) and
        # certainly wrong somewhere; where it over-reads, the compensation
        # becomes net acceleration. Measured: at 4.5 m/s the loop commanded
        # throttle 0.71 to cancel a modelled 3.66 m/s^2, and the vehicle
        # accelerated 15.7 -> 20.0 km/h while the gap closed from 6.5 m to
        # 2.5 m, arriving at the target under power.
        #
        # Capping a_des at zero while the stop is latched bounds throttle
        # at exactly the coast value, so the worst a table error can do is
        # hold speed — never gain it. This is NOT the earlier blanket
        # `throttle = 0` (which starved the loop and stopped the car 38 m
        # short); throttle is still free to cancel engine braking in full.
        if self._a_latch is not None:
            a_des = min(a_des, 0.0)
        a_act_target = a_des + self.coast_decel(self.v_ego)

        # No throttle while already above the reference.
        #
        # Coast compensation exists to stop engine braking overshooting a
        # planned deceleration. It has no business running while the
        # vehicle is FASTER than the plan — there, engine braking is doing
        # exactly what is wanted and cancelling it is simply wrong.
        # Measured over one stop: 53 frames applied throttle while above
        # the reference, worst case 0.65 throttle at 0.5 km/h too fast,
        # 80 m out. The run ended in contact.
        #
        # Note this is conditioned on the SPEED ERROR, not on the latch.
        # An earlier attempt suppressed throttle whenever a stop was
        # latched, which also suppressed it below the reference and left
        # the vehicle coasting to a halt 38.6 m short. Here, falling under
        # the reference restores throttle immediately, so the loop can
        # still hold the profile up — it just cannot fight a deceleration
        # it is already behind on.
        if speed_error < -self.NO_THROTTLE_DEADBAND:
            a_act_target = min(a_act_target, 0.0)

        a_act = self._slew_a_act(a_act_target)

        if a_act >= 0.0:
            throttle = min(a_act / self.THROTTLE_AUTHORITY,
                           self.cruise_throttle_cap)
            self.prev_acc_brake = max(0.0, self.prev_acc_brake
                                      - self.BRAKE_RATE_DOWN)
            return (throttle, 0.0)

        if self.simulator == 'morai':
            # MORAI's brake leaves the vehicle stuck rather than
            # decelerating smoothly (DEBUG.md) — coast instead.
            self.prev_acc_brake = 0.0
            return (0.0, 0.0)

        brake = min(-a_act / self.BRAKE_AUTHORITY, brake_cap)

        # Minimum brake whenever the throttle is off.
        #
        # Coasting is not a decisive action: engine braking varies 2-6 m/s²
        # with gear, so "throttle to zero" can mean almost anything. That
        # left the speed hovering around the reference, and with coast
        # compensation gated on the speed error the two toggled against
        # each other — one run showed 14 throttle jumps against 1 before,
        # with peak deceleration rising 8.00 -> 8.32.
        #
        # A small definite brake on the no-throttle side gives the
        # transition a floor: the vehicle actually slows, falls clear of
        # the reference, and throttle returns cleanly instead of the two
        # chattering at the boundary.
        if brake < self.MIN_BRAKE_NO_THROTTLE:
            brake = self.MIN_BRAKE_NO_THROTTLE
        brake = min(brake, self.prev_acc_brake + self.BRAKE_RATE_UP)
        brake = max(brake, self.prev_acc_brake - self.BRAKE_RATE_DOWN)
        brake = max(0.0, min(brake, brake_cap))
        self.prev_acc_brake = brake
        return (0.0, brake)


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
        # the only thing acting on the vehicle — engine braking alone
        # sheds 4.9 m/s^2 at 22-29 km/h and peaks above 10 (§45.2).
        #
        # This also retires the old min(acc_throttle, cruise_throttle) /
        # max(acc_brake, cruise_brake) combination: the governor already
        # clamps its reference to target_speed, so "follow the lead OR hold
        # set speed, whichever is slower" falls out of one loop instead of
        # two controllers arguing over one actuator.
        v_ref = self.speed_reference(self.d_lead, self.track_v, self.v_ego)
        self.v_ref_last = v_ref
        throttle, brake = self.speed_control(v_ref)

        # NOTE: an earlier version forced `throttle = 0` here whenever the
        # acquisition latch was set, on the reasoning that a vehicle
        # committed to a stop should never accelerate. That was wrong, and
        # wrong in an instructive way.
        #
        # Measured coast-down in CARLA with throttle AND brake at zero:
        #     22-29 km/h   mean 4.94 m/s^2, peaking 10.42
        #     29-36 km/h   mean 5.49 m/s^2
        # Releasing the throttle in this vehicle is not gentle — it exceeds
        # the 5 m/s^2 limit on its own. Partial throttle is therefore how a
        # SOFT deceleration is produced, and the speed loop applying it
        # through a stop (202 of 238 frames in one run) was doing the right
        # thing, not hunting. Suppressing it left the car coasting on engine
        # braking, which stopped it 38.6 m short of the target with the
        # brake never used at all (verdict: no_reaction).
        #
        # The corollary is uncomfortable and belongs in the record: this
        # stack cannot bound deceleration below 5 m/s^2 by managing the
        # brake alone, because doing nothing already breaches it.
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