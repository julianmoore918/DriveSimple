#!/usr/bin/env python3
"""
MORAI State Adapter Node (ROS2 Humble)
=======================================
Bridges MORAI's ROS2 VehicleInfo publisher (morai_v2_1_ros2_msgs/msg/
VehicleInfo, a custom message, bound to a GroundTruth entity) to the
/Car_1/vehicle/speed scalar topic controller_node expects in
`simulator:=morai` mode (example_interfaces/msg/Float64).

Originally this derived speed from an IMU-bound ROS2_Odometry
publisher (nav_msgs/msg/Odometry) via |twist.twist.linear| — see
DEBUG.md. That never worked: MORAI's "MA IMU" sensor model computes
orientation (gyro integration) correctly but never populates linear
velocity/position at all, so speed read a constant 0.0 regardless of
real vehicle motion. Switched to a GroundTruth entity's VehicleInfo
message instead, which reports the simulator's own exact internal
vehicle state — including local_velocity — directly, with no sensor
model in between to be incomplete.

    speed = |local_velocity_x|  (vehicle body-frame forward component)

Deliberately NOT the full 3-axis vector norm: live capture showed
local_velocity_y coming through as a corrupt/garbage value (e.g.
~3.7e19) from MORAI's own VehicleInfo publisher, while x (forward) and
z decoded as sane numbers -- see DEBUG.md. Using only the forward
component sidesteps that bad axis entirely, and matches what "vehicle
speed" conventionally means anyway (a real speedometer reads
longitudinal speed, not the magnitude of the full 3D velocity vector).

Further live capture showed that same garbage magnitude isn't fixed to
one field -- it's landed on local_velocity_y, local_acceleration_x,
local_acceleration_z, and steer_angle across different samples, always
the same ~3.7e19-ish constant. That moving-target pattern (not a fixed
byte offset) points at an uninitialized-memory/buffer-reuse bug inside
MORAI's own serializer, not our .msg field layout -- nothing fixable
from this side. Since any field, including local_velocity_x, could in
principle get hit on a future tick, _on_vehicleinfo sanity-clamps the
reading and falls back to the last known-good speed rather than ever
forwarding an obviously-impossible value downstream. See DEBUG.md.

Subscribed:
    <vehicleinfo_topic>  (morai_v2_1_ros2_msgs/msg/VehicleInfo),
                         default /Ego/vehicleinfo -- MUST match the
                         "Topic" field on the GroundTruth entity's
                         ROS2 Interface in MORAI Studio exactly (an
                         arbitrary, freely-renameable string with no
                         connection to the vehicle's internal `id`
                         field inside the message, or to any other
                         topic's naming convention). If GT speed goes
                         silent after touching that panel, check here
                         first before assuming a sensor bug.

Published:
    <speed_topic>  (example_interfaces/msg/Float64), default
                   /Car_1/vehicle/speed
"""
import math

import rclpy
from rclpy.node import Node
from morai_v2_1_ros2_msgs.msg import VehicleInfo
from example_interfaces.msg import Float64

# Generous upper bound on physically-plausible vehicle speed (~200 km/h).
# MORAI's own serializer bug (see module docstring) produces garbage on
# the order of 1e19-1e20 -- anything past this ceiling is unambiguously
# corrupt data, not a fast car.
MAX_PLAUSIBLE_SPEED_MS = 55.0


class MoraiStateAdapter(Node):
    def __init__(self):
        super().__init__('Morai_State_Adapter')

        self.declare_parameter('vehicleinfo_topic', '/Ego/vehicleinfo')
        self.declare_parameter('speed_topic', '/Car_1/vehicle/speed')
        vehicleinfo_topic = self.get_parameter('vehicleinfo_topic').value
        speed_topic = self.get_parameter('speed_topic').value

        self.speed_pub = self.create_publisher(Float64, speed_topic, 20)
        self.create_subscription(VehicleInfo, vehicleinfo_topic,
                                  self._on_vehicleinfo, 20)
        self._last_good_speed = 0.0

        self.get_logger().info(
            '=== MORAI State Adapter starting ===\n'
            f'    VehicleInfo in: {vehicleinfo_topic}\n'
            f'    Speed out:      {speed_topic}')

    def _on_vehicleinfo(self, msg: VehicleInfo):
        speed = abs(msg.local_velocity_x)
        if not math.isfinite(speed) or speed > MAX_PLAUSIBLE_SPEED_MS:
            self.get_logger().warn(
                f'[sanity] rejected implausible local_velocity_x={msg.local_velocity_x!r} '
                f'-- reusing last good speed={self._last_good_speed:.2f} m/s',
                throttle_duration_sec=2.0)
            speed = self._last_good_speed
        else:
            self._last_good_speed = speed
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
