#!/usr/bin/env python3
"""
MORAI Control Adapter Node (ROS2 Humble)
==========================================
Bridges the ADAS stack's simulator-agnostic control topics to MORAI's
custom morai_v2_1_ros2_msgs/msg/VehicleManualControl subscriber
(throttle, brake, steering_wheel_angle -- all float64).

Subscribed:
    /Car_1/cmd_vel     (geometry_msgs/msg/Twist)
        linear.x = throttle [0..1], linear.y = brake [0..1]
        (owned by controller_node)
    /Car_1/cmd_steer   (std_msgs/msg/Float32)
        normalised steer in [-1..1], positive = right
        (owned by stanley_node)

Published:
    /Car_1/control     (morai_v2_1_ros2_msgs/msg/VehicleManualControl)
                       -- unless `dry_run:=true`, in which case this
                       node still subscribes/tracks/logs normally but
                       never actually publishes, so ACC/LKAS can be
                       validated (via /Car_1/cmd_vel, /Car_1/cmd_steer,
                       and the debug images/BEV) while a human drives
                       the car directly in MORAI.

UNCALIBRATED: MORAI's field is literally named "steering_wheel_angle",
i.e. behind a steering ratio, not the road-wheel angle our steer value
is normalised against. `steer_to_wheel_angle_deg` and `steer_sign` are
exposed as ROS parameters so they can be tuned empirically once the
vehicle is observed turning in MORAI:
  - Drive straight, apply a small constant positive steer, and check
    whether the car turns right (expected -- README: "positive =
    right"). If it turns left, set steer_sign:=-1.0.
  - Tune steer_to_wheel_angle_deg so full steer (+-1.0) yields
    realistic lock -- not so much MORAI clamps/ignores it, not so
    little Stanley can't correct in tight turns.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from morai_v2_1_ros2_msgs.msg import VehicleManualControl


class MoraiControlAdapter(Node):
    def __init__(self):
        super().__init__('Morai_Control_Adapter')

        self.declare_parameter('cmd_vel_topic', '/Car_1/cmd_vel')
        self.declare_parameter('cmd_steer_topic', '/Car_1/cmd_steer')
        self.declare_parameter('control_topic', '/Car_1/control')
        # NEEDS CALIBRATION -- see module docstring. Was 450.0 (a
        # plausible full-lock steering-wheel angle), then 60.0, then 6.0,
        # then 0.6 -- still too sharp/prone to circling at each step. If
        # it's still wrong at 0.1, that's a strong signal this isn't a
        # scale problem at all -- see the rate-vs-absolute-angle note
        # below (still unconfirmed; a faster update rate makes tracking
        # *better* if MORAI treats this as an absolute angle, but makes
        # accumulated drift *worse* if it's actually a per-tick increment).
        self.declare_parameter('steer_to_wheel_angle_deg', 0.1)
        # Confirmed empirically: positive steer_norm made the car turn
        # left in MORAI, not right (README/Stanley convention: positive =
        # right). Flipped.
        self.declare_parameter('steer_sign', -1.0)
        # NEEDS CALIBRATION. Our throttle/brake are 0..1 (matching CARLA's
        # convention); MORAI's VehicleManualControl.throttle/brake range is
        # undocumented -- could be 0..1 or 0..100. Default 1.0 = pass
        # through unscaled; try `-p throttle_scale:=100.0` (and/or
        # brake_scale) live to test the 0..100 hypothesis without a
        # rebuild.
        self.declare_parameter('throttle_scale', 1.0)
        self.declare_parameter('brake_scale', 1.0)
        # Validation mode: keep computing/publishing everything upstream
        # (cmd_vel, cmd_steer, debug images) but never actually command
        # the vehicle -- lets a human drive while ACC/LKAS run alongside
        # for comparison, e.g. while MORAI's GroundTruth speed sensor is
        # known-broken (see DEBUG.md) and end-to-end closed-loop
        # validation isn't trustworthy anyway.
        self.declare_parameter('dry_run', False)

        cmd_vel_topic   = self.get_parameter('cmd_vel_topic').value
        cmd_steer_topic = self.get_parameter('cmd_steer_topic').value
        control_topic   = self.get_parameter('control_topic').value
        self.steer_scale = float(self.get_parameter('steer_to_wheel_angle_deg').value)
        self.steer_sign  = float(self.get_parameter('steer_sign').value)
        self.throttle_scale = float(self.get_parameter('throttle_scale').value)
        self.brake_scale    = float(self.get_parameter('brake_scale').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        self._throttle = 0.0
        self._brake = 0.0
        self._steer = 0.0

        self.control_pub = self.create_publisher(
            VehicleManualControl, control_topic, 20)
        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 20)
        self.create_subscription(Float32, cmd_steer_topic, self._on_cmd_steer, 20)

        self.get_logger().info(
            '=== MORAI Control Adapter starting ===\n'
            f'    cmd_vel in:    {cmd_vel_topic}\n'
            f'    cmd_steer in:  {cmd_steer_topic}\n'
            f'    control out:   {control_topic}\n'
            f'    steer_scale:   {self.steer_scale} deg (UNCALIBRATED)\n'
            f'    steer_sign:    {self.steer_sign}\n'
            f'    throttle_scale: {self.throttle_scale} (UNCALIBRATED)\n'
            f'    brake_scale:    {self.brake_scale} (UNCALIBRATED)\n'
            f'    DRY RUN:        {self.dry_run} -- '
            + ('NOT sending anything to the car' if self.dry_run
               else 'sending control commands normally'))

    def _on_cmd_vel(self, msg: Twist):
        self._throttle = msg.linear.x
        self._brake = msg.linear.y
        self._publish()

    def _on_cmd_steer(self, msg: Float32):
        self._steer = msg.data
        self._publish()

    def _publish(self):
        # Read live rather than using the __init__ snapshot: the UI's Dry
        # Run checkbox is toggled while this node is already spinning, and
        # it refuses to restart an adapter that is already up. Caching the
        # value here made the checkbox a no-op for the whole session.
        if bool(self.get_parameter('dry_run').value):
            return
        out = VehicleManualControl()
        out.throttle = float(self._throttle) * self.throttle_scale
        out.brake = float(self._brake) * self.brake_scale
        out.steering_wheel_angle = self.steer_sign * self._steer * self.steer_scale
        self.control_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MoraiControlAdapter()
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
