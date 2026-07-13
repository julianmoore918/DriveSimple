#!/usr/bin/env python3
"""
MORAI State Adapter Node (ROS2 Humble)
=======================================
Bridges MORAI's ROS2_Odometry publisher (nav_msgs/msg/Odometry, a
standard message type, bound to an IMU sensor entity) to the
/Car_1/vehicle/speed scalar topic controller_node expects in
`simulator:=morai` mode (example_interfaces/msg/Float64).

MORAI has no scalar "speed" template that avoids its custom
morai_v2_1_ros2_msgs package (see VehicleInfo), so speed is derived
here from the standard Odometry message instead:

    speed = |twist.twist.linear|

For this to equal the vehicle's own body-frame speed (not a value
skewed by rotation), the IMU sensor entity in MORAI should sit at the
vehicle's local origin (relative position 0, 0, 0) -- otherwise the
reported velocity includes an omega x r lever-arm term from yaw/pitch/
roll rate.

Subscribed:
    <odom_topic>   (nav_msgs/msg/Odometry), default /Car_1/odometry

Published:
    <speed_topic>  (example_interfaces/msg/Float64), default
                   /Car_1/vehicle/speed
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from example_interfaces.msg import Float64


class MoraiStateAdapter(Node):
    def __init__(self):
        super().__init__('Morai_State_Adapter')

        self.declare_parameter('odom_topic', '/Car_1/odometry')
        self.declare_parameter('speed_topic', '/Car_1/vehicle/speed')
        odom_topic = self.get_parameter('odom_topic').value
        speed_topic = self.get_parameter('speed_topic').value

        self.speed_pub = self.create_publisher(Float64, speed_topic, 20)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)

        self.get_logger().info(
            '=== MORAI State Adapter starting ===\n'
            f'    Odometry in:  {odom_topic}\n'
            f'    Speed out:    {speed_topic}')

    def _on_odom(self, msg: Odometry):
        v = msg.twist.twist.linear
        speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        out = Float64()
        out.data = speed
        self.speed_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MoraiStateAdapter()
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
