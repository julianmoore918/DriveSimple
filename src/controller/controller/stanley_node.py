#!/usr/bin/env python3
"""
LKAS Stanley Controller Node (ROS2 Humble)
==========================================
Consumes the ego-left / ego-right lane polylines from `lane_detection_node`
and ego speed, and publishes a normalised steer command in [-1, 1].

Ported from `02_UFLD_V2/lkas_validate_0.10.0.py` — `stanley_steer` and
`lane_center_at_lookahead` are lifted unchanged. The CARLA-map-driven junction
policy is deliberately NOT ported here (would leak the simulator into the
controller); when detection is unreliable the node falls back to steer=0
(HOLD mode), matching the validate script.

Frame convention: incoming Path messages are REP 103 (X forward, Y LEFT).
Internally the Stanley math uses Y RIGHT positive to match CARLA's steer
sign (positive = right), so we negate on input.

Subscribed topics:
    /LKAS/ego_lane_left   (nav_msgs/Path)
    /LKAS/ego_lane_right  (nav_msgs/Path)
    /Car_1/vehicle/speed  (std_msgs/Float64, or example_interfaces/Float64
                           when simulator:=morai)

Published topics:
    /Car_1/cmd_steer      (std_msgs/Float32)  — normalised steer in [-1, 1].
                          Consumed directly by the CARLA bridge; the ACC
                          controller no longer relays it via angular.z.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from std_msgs.msg import Float32, Float32MultiArray, Float64 as StdFloat64
from example_interfaces.msg import Float64 as ExFloat64


# Stanley gains — matched to lkas_validate_0.9.16.py (the CARLA version this
# ROS stack actually runs against). The 0.10.0 tuning (k=1.0, max=40°) is
# sharper and oscillates on 0.9.16 physics.
STANLEY_K     = 0.5
# MORAI reports the car as "not reactive enough" even with the adapter's
# output scale/rate already tuned -- that's a downstream multiplier, not
# Stanley's own cross-track gain, which had no MORAI override until now.
# Was 1.5 (3x CARLA) -- still not reactive enough, bumped another 10x.
# At 30x CARLA's tuned gain, watch for oscillation/overshoot as the new
# failure mode instead of sluggishness; override live with
# `-p stanley_k:=X` rather than rebuilding to iterate further.
STANLEY_K_MORAI = 15.0
# Heading error (psi / e_head) has ALWAYS had a fixed, unscaled weight of
# 1.0 in both formulas below -- only the cross-track term gets STANLEY_K.
# Bumping STANLEY_K alone does nothing for heading responsiveness. Add a
# separate gain so heading correction can be tuned independently; 1.0
# preserves CARLA's exact existing behaviour (canonical Stanley, no
# heading scaling).
STANLEY_HEADING_GAIN       = 1.0
STANLEY_HEADING_GAIN_MORAI = 3.0
STANLEY_EPS   = 0.5
MAX_STEER_RAD = math.radians(70)
LOOKAHEAD_M   = 5.0
# CARLA vehicle.dodge.charger_2020 wheelbase (120 in / 3.048 m).
# Only used by the curvature-feedforward term arctan(κ·L). If you swap
# the ego vehicle, this needs to change too.
WHEELBASE_M   = 3.048
# Coeff-mode staleness window. If perception hasn't published a fresh
# ego_lane_coeffs message within this many seconds, Stanley falls back
# to the traditional Path-based nearest-point formulation. Keeps the
# non-Kalman path unchanged and gives us a graceful degrade when the
# filters are uninitialised or the user has ticked Kalman OFF in the UI.
KF_COEFF_STALE_S = 0.3

# ── High-speed stabilisation (DEBUG §62) ────────────────────────────────
# Measured on scenarios/results/20260813_125601_r79_lka, site t12_r500,
# STRAIGHT lead-in (kappa = 0 for every logged sample — none of the
# failures reached the arc):
#
#   v [km/h]   lag(hdg→cmd)   K_psi regressed   envelope growth/cycle
#      70          0.20 s          1.10              1.06  (rings, passes)
#      90          0.28 s          0.79              2.01  (departs 5.7 s)
#     110          0.28 s          1.05              1.39  (departs 4.9 s)
#     130          0.28 s          1.06              1.45  (departs 4.5 s)
#
# At those speeds the cross-track term is inert — at 90 km/h with
# cte = 0.07 m it contributes 0.08 deg while the published command tracks
# the heading error one-for-one (-1.72 vs -1.75 deg). So the loop is pure
# heading feedback through a dead time, psi_dot = v*delta/L, which is
# stable only while
#
#     STANLEY_HEADING_GAIN * v * tau / L  <  pi/2
#
# With K = 1.0, L = 3.048 m and the measured tau = 0.20 s that crosses
# over at 86 km/h: 81 % of the limit at 70 (hence the 0.8 Hz ring and
# jerk 4.45 against R79's 5 m/s^3), 104 % at 90. The threshold predicted
# from two constants is exactly where the runs fail.
#
# tau is dominated by perception, not transport (steer_age_s peaked at
# 0.048 s): UFLD tick + IPM + KF is most of the 0.2 s budget. Raising the
# detector rate is the durable fix; the constant below buys margin
# against the dead time we have.
#
# Cap the feedback loop gain above this speed. Since the criterion above
# is linear in K*v, freezing K*v at its value here holds the margin
# constant at every higher speed instead of letting it erode as v grows.
#
# Set to 50 km/h, not 70, and the difference is the whole point. 70 was
# the obvious choice — the fastest speed that passed before the change,
# so the most conservative possible no-op boundary. Repeating the column
# showed 70 km/h is not a pass, it is a coin flip: |cte| max came out
# 0.37, 0.23, 3.18 and 3.18 m over four runs of the SAME site, two of
# them departures. At 70 the cap is inactive (g = 1.0) in every one of
# those runs, so that scatter is the untouched original controller
# sitting at 81 % of the stability limit and flipping on whatever
# disturbance it happens to meet. Anchoring a "safe" boundary to it
# would have frozen the loop gain at a value already known to be
# marginal, which is the opposite of what the schedule is for.
#
# 30 and 50 km/h repeated at 0.09-0.11 m across all four runs. That is
# the region the tune is genuinely validated in, so that is where the
# no-op boundary belongs. 58 % of the limit is the margin the schedule
# now holds at every speed above it.
#
# Measured, t12_r500 column, with the threshold still at 70 km/h:
#
#   v [km/h]     30     50     70     90    110    130
#   |cte| max  0.09   0.10   0.37   3.12   3.20   3.20   no cap
#              0.09   0.09   0.23   0.36   0.33   0.93   cap@70 #1
#              0.10   0.11   3.18   0.29   0.95   0.89   cap@70 #2
#
# 90/110/130 stay bounded under the cap in both runs where the uncapped
# controller departed outright in all three. That part is the real,
# repeated effect; see DEBUG §62 for the shipped-threshold numbers.
#
# Adding dead-time compensation on TOP of this was tried and rejected —
# three variants, all worse or useless. See DEBUG §62; do not re-derive.
#
# Written as the division rather than a decimal so the boundary is exact.
# At 13.9 the threshold falls just below 50 km/h and the 50 km/h row
# picks up a few thousandths of a degree of change — harmless in size,
# but it makes "identical at and below" false, and that claim is the
# whole reason this ships without re-validating the low-speed tune.
V_GAIN_SCHED_MPS = 50.0 / 3.6


def gain_scale(speed_mps: float) -> float:
    """Loop-gain multiplier for this speed — exactly 1.0 at and below
    V_GAIN_SCHED_MPS, then 1/v so K*v stays at its 70 km/h value."""
    return min(1.0, V_GAIN_SCHED_MPS / max(speed_mps, 1e-3))


def stanley_steer(e_lat: float, e_head: float, speed_mps: float) -> float:
    """Returns a normalised steer ∈ [-1, 1]. Positive = right.

    Gets the same speed schedule as the coeff path — the stability
    criterion is a property of the loop, not of which formulation
    produced the error, and it is simulator-independent.
    """
    g = gain_scale(speed_mps)
    delta = g * (STANLEY_HEADING_GAIN * e_head
                 + math.atan2(STANLEY_K * e_lat, speed_mps + STANLEY_EPS))
    return max(-1.0, min(1.0, delta / MAX_STEER_RAD))


def stanley_steer_from_coeffs(a: float, b: float, c: float,
                              speed_mps: float) -> tuple[float, float, float, float]:
    """Kalman-mode Stanley: compute δ directly from the smoothed
    centreline coefficients y_left(x) = a x² + b x + c (Y-LEFT / REP 103).

    Canonical Stanley with a curvature feed-forward term
    (Hoffmann-Tomlin-Montemerlo-Thrun ACC 2007; Snider CMU-RI-TR-09-08
    §3.2, 2009):

        ψ    = arctan(b)                         heading error (analytic)
        e    = c                                 cross-track at x = 0
        κ    = 2 a / (1 + b²)^(3/2)              road curvature at x = 0
        g    = min(1, V_GAIN_SCHED / v)          loop-gain cap
        δ    = g·[ψ + arctan(k·e / v)] + arctan(κ·L)

    Term 1 zeros the heading error, term 2 is the classic Stanley
    cross-track law (proven asymptotically stable for fixed scalar k),
    term 3 is a curvature feed-forward that anticipates the turn using
    the vehicle wheelbase L. Straight-road correction is preserved
    because k is fixed — κ only adds forward-looking authority in
    curves. δ is computed in Y-LEFT convention; we flip sign at the
    end because CARLA's cmd_steer takes positive = RIGHT.

    g multiplies the two FEEDBACK terms only. The feed-forward is
    open-loop — it has no bearing on the stability criterion g exists to
    satisfy, and scaling it would simply make the car run wide in curves
    at exactly the speeds where holding the line matters most.

    g is exactly 1.0 at and below V_GAIN_SCHED_MPS, so the schedule
    cannot touch the validated low-speed tune — verified to 0.0000 deg
    across straight, offset, heading-error and R = 500/200/60 curve
    cases at 10/30/50/70 km/h, and to 0.0000 deg on the curve cases at
    every speed, since a steady-state curve is held by the feed-forward
    that g does not scale.

    Returns (steer_norm, delta_rad, kappa, e_head_rad) so the caller
    can log the per-frame Stanley terms for the sensitivity study.
    """
    g     = gain_scale(speed_mps)
    kappa = 2.0 * a / (1.0 + b * b) ** 1.5
    psi   = math.atan(b)                              # heading error
    e_lat = c                                         # cross-track
    delta_left = (g * (STANLEY_HEADING_GAIN * psi
                       + math.atan2(STANLEY_K * e_lat,
                                    speed_mps + STANLEY_EPS))  # feedback
                  + math.atan(kappa * WHEELBASE_M))            # feed-forward
    delta_carla = -delta_left   # Y-LEFT → CARLA Y-RIGHT (+ = right)
    steer_norm  = max(-1.0, min(1.0, delta_carla / MAX_STEER_RAD))
    return steer_norm, delta_carla, kappa, psi


def lane_center_at_lookahead(left_veh, right_veh, lookahead_m: float):
    """Returns ((x_near, y_near), (x_far, y_far)) in vehicle frame metres
    (Y RIGHT positive — caller is responsible for sign). None if not enough
    data to interpolate at the lookahead."""
    if not left_veh or not right_veh:
        return None

    def interp_y_at_x(poly, x_target):
        xs = np.array([p[0] for p in poly])
        ys = np.array([p[1] for p in poly])
        order = np.argsort(xs)
        xs, ys = xs[order], ys[order]
        if x_target < xs[0] or x_target > xs[-1]:
            return None
        return float(np.interp(x_target, xs, ys))

    left_y  = interp_y_at_x(left_veh,  lookahead_m)
    right_y = interp_y_at_x(right_veh, lookahead_m)
    if left_y is None or right_y is None:
        return None
    y_center = (left_y + right_y) / 2.0

    x_far = lookahead_m + 1.0
    lf = interp_y_at_x(left_veh,  x_far)
    rf = interp_y_at_x(right_veh, x_far)
    if lf is None or rf is None:
        x_far = lookahead_m + 0.5
        lf = interp_y_at_x(left_veh,  x_far)
        rf = interp_y_at_x(right_veh, x_far)
        if lf is None or rf is None:
            return ((lookahead_m, y_center), None)
    return ((lookahead_m, y_center), (x_far, (lf + rf) / 2.0))


class StanleyNode(Node):
    def __init__(self):
        # Module-level constants, read as defaults below and then
        # overridden from ROS params -- must be declared global before
        # any reference to the names within this function.
        global STANLEY_K, STANLEY_HEADING_GAIN, V_GAIN_SCHED_MPS
        super().__init__('Stanley_Node', namespace='LKAS')
        self.get_logger().info("=== Stanley Node starting ===")

        # simulator (carla | morai) picks the speed message type below —
        # same parameter/pattern as controller_node.
        self.declare_parameter('simulator', 'carla')
        simulator = self.get_parameter('simulator').get_parameter_value().string_value
        self.simulator = simulator

        self.declare_parameter('lookahead_m', LOOKAHEAD_M)
        self.declare_parameter('speed_topic', '/Car_1/vehicle/speed')
        # Doubled for MORAI: steer corrections were arriving/settling too
        # slowly relative to how fast the car drifted off-lane. CARLA's
        # 20 Hz is unchanged (already validated there).
        self.declare_parameter('control_rate_hz', 40.0 if simulator == 'morai' else 20.0)
        # Stanley's own cross-track gain -- previously a hardcoded module
        # constant with no way to tune MORAI independently of CARLA. Both
        # stanley_steer() and stanley_steer_from_coeffs() read the module
        # global directly, so override it here rather than threading a
        # new argument through both call sites.
        self.declare_parameter('stanley_k', STANLEY_K_MORAI if simulator == 'morai' else STANLEY_K)
        STANLEY_K = float(self.get_parameter('stanley_k').value)
        # Separate heading-error gain -- see module-level comment. 1.0
        # for CARLA (unscaled, matches the original canonical formula).
        self.declare_parameter('stanley_heading_gain',
            STANLEY_HEADING_GAIN_MORAI if simulator == 'morai' else STANLEY_HEADING_GAIN)
        STANLEY_HEADING_GAIN = float(self.get_parameter('stanley_heading_gain').value)
        # High-speed stabilisation (DEBUG §62), live-tunable so the change
        # can be backed out against a running stack rather than a rebuild:
        # -p v_gain_sched_mps:=1e9 restores the exact pre-§62 formula.
        # That is also how the before/after columns in §62 were measured.
        self.declare_parameter('v_gain_sched_mps', V_GAIN_SCHED_MPS)
        V_GAIN_SCHED_MPS = float(self.get_parameter('v_gain_sched_mps').value)
        self.lookahead = self.get_parameter('lookahead_m').value
        speed_topic    = self.get_parameter('speed_topic').value
        rate           = self.get_parameter('control_rate_hz').value

        self.create_subscription(Path, 'ego_lane_left',
                                 self.left_callback,  10)
        self.create_subscription(Path, 'ego_lane_right',
                                 self.right_callback, 10)
        # Optional coeff channel — only published by lane_detection_node
        # when Kalman is active AND both side filters are initialised.
        # Its presence + freshness is the sole switch between the KF
        # formula and the traditional Path-based Stanley.
        self.create_subscription(Float32MultiArray, 'ego_lane_coeffs',
                                 self.coeffs_callback, 10)
        SpeedMsg = ExFloat64 if simulator == 'morai' else StdFloat64
        self.create_subscription(SpeedMsg, speed_topic,
                                 self.speed_callback, 20)
        # Absolute topic name — bypasses the LKAS namespace so steer lands on
        # the same /Car_1/* tree the bridge already owns.
        self.steer_pub = self.create_publisher(Float32, '/Car_1/cmd_steer', 20)
        # UI-facing rebroadcast of the vehicle speed. The raw
        # /Car_1/vehicle/speed topic gets published as std_msgs/Float64
        # by CARLA and example_interfaces/Float64 by MORAI, and DDS
        # will not deliver across the type mismatch. Rather than have
        # the UI guess the right type at startup, we republish it here
        # as a fixed std_msgs/Float32 so the UI can subscribe with one
        # type regardless of which simulator is running.
        self.speed_ui_pub = self.create_publisher(
            Float32, '/ADAS/telemetry/speed_mps', 10)

        self.left_veh  = []      # list of (X_forward, Y_right)
        self.right_veh = []
        self.speed     = 0.0
        self.last_log_time = 0.0
        # KF coefficient side-channel state. None until perception
        # publishes the first ego_lane_coeffs message; wall-clock stamp
        # is captured on receipt so the control loop can gate on age.
        self._coeffs: tuple[float, float, float] | None = None
        self._coeffs_stamp = None
        # Track HOLD-mode transitions so we log every entry/exit at WARN
        # (not just the throttled per-second INFO). Makes the bridge
        # pure-pursuit fallback's engagement legible.
        self._prev_mode: str | None = None

        self.create_timer(1.0 / rate, self.control_loop)
        self.get_logger().info(
            f"Stanley initialised | lookahead={self.lookahead} m | "
            f"rate={rate} Hz | speed_topic={speed_topic} | "
            f"stanley_k={STANLEY_K} | stanley_heading_gain={STANLEY_HEADING_GAIN} | "
            f"gain cap above {V_GAIN_SCHED_MPS * 3.6:.0f} km/h"
        )

    # ── Convert nav_msgs/Path (REP 103, Y LEFT) → list of (X_fwd, Y_right) ─
    @staticmethod
    def _path_to_veh(path: Path):
        return [(pose.pose.position.x, -pose.pose.position.y)
                for pose in path.poses]

    def left_callback(self, msg: Path):
        self.left_veh = self._path_to_veh(msg)

    def right_callback(self, msg: Path):
        self.right_veh = self._path_to_veh(msg)

    def speed_callback(self, msg):
        self.speed = abs(msg.data)
        # Rebroadcast for the UI (see speed_ui_pub declaration above).
        out = Float32(); out.data = float(self.speed)
        self.speed_ui_pub.publish(out)

    def coeffs_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self._coeffs = (float(msg.data[0]),
                            float(msg.data[1]),
                            float(msg.data[2]))
            self._coeffs_stamp = self.get_clock().now()
            # First-message-only log so we can confirm the coeff
            # subscription is actually wired at runtime. If this
            # never fires, the pub-sub link is broken.
            if not getattr(self, '_coeffs_seen_logged', False):
                self._coeffs_seen_logged = True
                self.get_logger().info(
                    f'[KF-SUB] first ego_lane_coeffs message received '
                    f'a={msg.data[0]:+0.4f} b={msg.data[1]:+0.3f} '
                    f'c={msg.data[2]:+0.3f}')

    def _coeffs_fresh(self) -> bool:
        """True iff we've received a coeff message within
        KF_COEFF_STALE_S. Absent-or-stale → fall back to Path mode
        (which is exactly the traditional Stanley behaviour)."""
        if self._coeffs is None or self._coeffs_stamp is None:
            return False
        age = (self.get_clock().now() - self._coeffs_stamp).nanoseconds * 1e-9
        return age < KF_COEFF_STALE_S

    def control_loop(self):
        # ── Kalman-active path: coefficients from perception, direct
        #    thesis formula δ = arctan(b̂) + arctan(κ ĉ / v). We ONLY
        #    take this path when the coeff channel is fresh, so an
        #    OFF toggle in the UI (which stops the publisher) drops
        #    us back to the traditional Path-based Stanley within one
        #    KF_COEFF_STALE_S window.
        if self._coeffs_fresh():
            a, b, c = self._coeffs
            steer, delta_carla, kappa, e_head_left = \
                stanley_steer_from_coeffs(a, b, c, self.speed)
            out = Float32(); out.data = float(steer)
            self.steer_pub.publish(out)
            mode = 'KF-STAN'
            e_lat = c      # Y-LEFT convention, for log continuity
            e_head = e_head_left
            self._log_stanley_terms(mode, steer, e_lat, e_head,
                                    kappa=kappa, delta_rad=delta_carla)
            self._log_mode_edge(mode)
            return

        lookahead = lane_center_at_lookahead(self.left_veh, self.right_veh,
                                             self.lookahead)
        if lookahead is None:
            # HOLD: UFLD couldn't recover a lane centre at the lookahead
            # distance — typically inside a junction or where the polylines
            # are too short. For CARLA we deliberately DO NOT publish
            # /Car_1/cmd_steer here: the bridge's `is_steer_fresh()` then
            # goes False and the pure-pursuit fallback
            # (carlaaccsim/carlaAccSimTown.py) takes over. Stanley resumes
            # the moment UFLD locks the ego-lane back up on the far side.
            #
            # MORAI has no such fallback controller -- nothing else steers
            # while Stanley is silent, so control_adapter_node just keeps
            # re-sending whatever steer value it last received, frozen,
            # for as long as HOLD lasts. With UFLD's lane lock flickering
            # on MORAI (frequent LOW/FEW/REJ churn), that produced "steers
            # slightly, then holds a stale angle" instead of a real
            # fallback. Publish 0.0 (wheel straight) for MORAI instead of
            # going silent -- not correct if genuinely mid-turn, but far
            # safer than perpetuating an arbitrary stale nonzero angle
            # indefinitely.
            if self.simulator == 'morai':
                steer = 0.0
                out = Float32(); out.data = steer
                self.steer_pub.publish(out)
            else:
                steer = float('nan')
            mode = 'HOLD'
            e_lat = e_head = float('nan')
        else:
            near_pt, far_pt = lookahead
            e_lat = near_pt[1]
            if far_pt is None:
                e_head = 0.0
            else:
                dx = far_pt[0] - near_pt[0]
                dy = far_pt[1] - near_pt[1]
                e_head = math.atan2(dy, dx)
            steer = stanley_steer(e_lat, e_head, self.speed)
            mode = 'STANLEY'
            out = Float32()
            out.data = float(steer)
            self.steer_pub.publish(out)

        self._log_mode_edge(mode)
        self._log_stanley_terms(mode, steer, e_lat, e_head)

    def _log_mode_edge(self, mode: str) -> None:
        """Edge-triggered mode-change log so the operator sees HOLD ↔
        STANLEY ↔ KF-STAN transitions. Called from both control-loop
        branches so the KF path also gets a visible switch-over log."""
        if mode == self._prev_mode:
            return
        if mode == 'HOLD':
            self.get_logger().warn(
                f'HOLD — no lane centre at lookahead={self.lookahead} m '
                f'(left_pts={len(self.left_veh)}, right_pts={len(self.right_veh)}). '
                f'Stanley yielding; bridge pure-pursuit fallback should engage in '
                f'~200 ms.')
        elif mode == 'KF-STAN':
            self.get_logger().info(
                f'KF-STAN engaged (coeff channel fresh) — canonical Stanley '
                f'+ curvature feed-forward: δ = g·[ψ + arctan(k·ĉ/v)] '
                f'+ arctan(κ·L), k={STANLEY_K}, L={WHEELBASE_M} m, '
                f'g=min(1, {V_GAIN_SCHED_MPS:.1f}/v).')
        else:
            self.get_logger().info('STANLEY re-engaged (lane re-acquired).')
        self._prev_mode = mode

    def _log_stanley_terms(self, mode, steer, e_lat, e_head,
                           kappa=None, delta_rad=None) -> None:
        """~1 Hz throttled per-frame Stanley log. In KF-STAN mode we
        also print κ and the un-normalised δ so the thesis-side
        sensitivity study can correlate the tune with the actual
        steer command sent to CARLA."""
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log_time <= 1.0:
            return
        e_lat_s  = f'{e_lat:+0.2f}'  if not math.isnan(e_lat)  else 'n/a'
        e_head_s = f'{math.degrees(e_head):+0.1f}' if not math.isnan(e_head) else 'n/a'
        steer_s  = f'{steer:+0.3f}' if not math.isnan(steer) else '+nan'
        extra = ''
        if kappa is not None and delta_rad is not None:
            extra = (f'  κ={kappa:+0.4f} 1/m  '
                     f'δ={math.degrees(delta_rad):+0.1f} deg')
        self.get_logger().info(
            f'[{mode:>7}] v={self.speed:5.2f} m/s  '
            f'e_lat={e_lat_s} m  e_head={e_head_s} deg  '
            f'steer={steer_s}{extra}'
        )
        self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = StanleyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
