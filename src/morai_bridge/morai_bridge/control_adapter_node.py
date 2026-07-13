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
        # plausible full-lock steering-wheel angle) but that turned tiny
        # lane-keeping corrections (e.g. steer=-0.305 from a mere 0.24 m
        # lateral error) into a 137 deg wheel command -- wildly
        # oversized. Dropped to a much gentler starting point pending
        # real calibration against observed front-wheel response.
        self.declare_parameter('steer_to_wheel_angle_deg', 60.0)
        self.declare_parameter('steer_sign', 1.0)

        cmd_vel_topic   = self.get_parameter('cmd_vel_topic').value
        cmd_steer_topic = self.get_parameter('cmd_steer_topic').value
        control_topic   = self.get_parameter('control_topic').value
        self.steer_scale = float(self.get_parameter('steer_to_wheel_angle_deg').value)
        self.steer_sign  = float(self.get_parameter('steer_sign').value)

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
            f'    steer_sign:    {self.steer_sign}')

    def _on_cmd_vel(self, msg: Twist):
        self._throttle = msg.linear.x
        self._brake = msg.linear.y
        self._publish()

    def _on_cmd_steer(self, msg: Float32):
        self._steer = msg.data
        self._publish()

    def _publish(self):
        out = VehicleManualControl()
        out.throttle = float(self._throttle)
        out.brake = float(self._brake)
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
