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

from std_msgs.msg import Float32, Bool, Float64 as StdFloat64
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
CRUISE_SPEED_KMH = 25.0
# Effective declared cruising speed for the ODD is 20 km/h. The P-only
# cruise-mode throttle law + 0.5 m/s deadband + drag / rolling
# resistance leaves a steady-state error of ~4 km/h below setpoint at
# these speeds, so the setpoint is offset above the effective target
# so the vehicle actually settles near 20 km/h. If an integral term is
# added to _cruise_control() later, this can be reduced back to 20.
# The ODD's declared System/Feature Designed Speed Range is unchanged.
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
        self.d0                = 3.0       # standstill bumper gap [m]
        # d_desired = d0 + T_gap * v_ego. T_gap=0.3 s → at 20 km/h cruise
        # (5.5 m/s), d_desired ≈ 6.65 m, settling to d0=5 m at rest.
        # That matches the "follow at roughly 5 m gap" mental model and
        # gives the controller enough headroom that closing-rate dominates
        # the PD response on a lead deceleration event.
        gain_scale = ACC_GAIN_SCALE_MORAI if simulator == 'morai' else 1.0
        self.T_gap             = 0.3
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
        # Even at gain_scale=0.1, cruise throttle still hit 1.0 on first
        # MORAI runs (speed feedback was/may still be unreliable enough
        # that speed_error stays large). Hard-cap MORAI's cruise throttle
        # well below full send regardless of what the gain computes;
        # CARLA keeps its original 1.0 ceiling.
        self.cruise_throttle_cap = 0.8 if simulator == 'morai' else 1.0
        self.a_max             = 3.0       # [m/s²]
        self.a_min             = -6.0      # [m/s²]
        # Emergency threshold is now a bumper gap, not a camera distance.
        # IPM also saturates near gap ≤ 2 m (bb-bottom clips at frame
        # edge → reported gap drops well below truth) — the saturated
        # value still falls under this 3 m threshold, so saturation
        # itself trips EMERGENCY without any special branch. See §22.
        self.emergency_distance = 1.0      # full brake below this gap [m]
        self.throttle_scale    = 3.0       # a_max → full throttle
        self.brake_scale       = 3.0       # a_min → full brake
        self.prev_throttle     = 0.0
        self.THROTTLE_RATE_LIMIT = 0.05  # max throttle increase per step (20 Hz → 1.0 in ~1 second)

        # ── Distance filter ──────────────────────────────────────────────
        # Low-pass on /ACC/lead_vehicle_distance. ALPHA=0.01 effectively
        # froze the filter (~100-sample memory at variable YOLO Hz), so
        # the controller saw a stale "lots of room" reading and accelerated
        # into the back of a closing lead. 0.4 gives ~3-sample (≈300 ms)
        # response while still smoothing single-frame YOLO jitter.
        self.ALPHA             = 0.4
        self.d_lead_filtered   = None

        # ── Internal state ───────────────────────────────────────────────
        self.v_ego             = 0.0
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
        """Store the latest ego vehicle speed."""
        try:
            self.v_ego = abs(msg.data)
        except Exception as e:
            self.get_logger().error(f"Ego velocity callback error: {e}")

    def lead_distance_callback(self, msg: Float32):
        """Store the latest lead vehicle distance, filtered to reduce noise."""
        try:
            d = msg.data

            if d == float('inf') or d <= 0.0:
                # No detection — reset filter and fall back to cruise mode
                self.d_lead = None
                self.d_lead_filtered = None
                return

            # Low-pass filter: smooth out noisy distance measurements
            # d_filtered = ALPHA * d_new + (1 - ALPHA) * d_prev
            if self.d_lead_filtered is None:
                self.d_lead_filtered = d  # initialise on first valid detection
            else:
                self.d_lead_filtered = self.ALPHA * d + (1 - self.ALPHA) * self.d_lead_filtered

            self.d_lead = self.d_lead_filtered

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

        # -- PD control --
        # P-term: positive when too far (accelerate), negative when too
        #         close (decelerate).
        # D-term: positive when gap grows (can accelerate), negative when
        #         gap shrinks (should brake earlier).
        distance_error = d_lead - d_desired
        a = self.k_p * distance_error + self.k_d * closing_rate

        # Clip to physical limits.
        a = max(min(a, self.a_max), self.a_min)

        # Deadband: if distance error is small and speed is very low,
        # suppress tiny throttle commands to prevent creeping.
        if abs(distance_error) < 1.0 and v_ego < 0.5 and a > 0:
            a = 0.0

        return a

    # ====================================================================
    # CRUISE CONTROL (fallback when no lead vehicle detected)
    # ====================================================================

    def cruise_control(self) -> tuple:
        """
        Symmetric proportional speed controller for cruising. Settles at
        CRUISE_SPEED_KMH instead of blowing past it.

        Previous law was asymmetric — throttle gain 0.3 (cap 1.0) but
        brake gain 0.1 (cap 0.3) — which let the car overshoot the
        20 km/h target by ~10 m/s under CARLA's inertia before the weak
        brake could bring it back. Symmetric gains + a small 0.5 m/s
        deadband around target avoids hunting and keeps cruise honest.

        Returns (throttle, brake).
        """
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
            self.control_pub.publish(control_msg)
            self.prev_throttle = 0.0
            if ENABLE_LOGGING:
                self._log_throttled("STANDSTILL", 0.0, 0.05)
            return

        # ---- MODE 2: CRUISE (no lead vehicle detected) ----
        if self.d_lead is None:
            throttle, brake = self.cruise_control()

            # Rate limit throttle
            throttle = min(throttle, self.prev_throttle + self.THROTTLE_RATE_LIMIT)
            self.prev_throttle = throttle

            control_msg.linear.x = throttle
            control_msg.linear.y = brake
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
        else:
            acc_throttle = 0.0
            acc_brake    = min(-a / self.brake_scale, 1.0)

        # ACC must respect CRUISE_SPEED_KMH as an upper cap. The PD law
        # above has no concept of "we're already at set speed" — with a
        # lead 30 m ahead, distance_error stays large and saturates
        # a → a_max → throttle = 1.0 *indefinitely*, even past 100 km/h.
        # Real ACC behaves as "follow the lead OR hold set speed,
        # whichever is slower" — implemented here as the lower throttle
        # and the higher brake of the two controllers. The cruise side
        # commands brake whenever v_ego > target + 0.5, which arrests
        # the runaway.
        cruise_throttle, cruise_brake = self.cruise_control()
        throttle = min(acc_throttle, cruise_throttle)
        brake    = max(acc_brake,    cruise_brake)

        # Rate limit throttle — braking is always instant
        throttle = min(throttle, self.prev_throttle + self.THROTTLE_RATE_LIMIT)
        self.prev_throttle = throttle

        control_msg.linear.x = throttle
        control_msg.linear.y = brake

        self.control_pub.publish(control_msg)

        if ENABLE_LOGGING:
            self._log_throttled("ACC", throttle, brake)

    # ====================================================================
    # LOGGING HELPER
    # ====================================================================

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