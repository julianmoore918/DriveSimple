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

    speed = hypot(local_velocity.x, local_velocity.y)   [m/s]

Planar norm of the body-frame velocity: x is forward, y lateral, and
both matter during a slip/yaw transient where the forward component
alone under-reads. z is deliberately excluded -- it is vertical motion
(slope, suspension travel) and is not ground speed.

HISTORY -- the "MORAI serializer bug" recorded here previously was our
own bug. This node used to read flat float32 fields (local_velocity_x)
against a hand-authored .msg, and saw ~3.7e19 / ~1.1e-19 garbage on a
seemingly random set of fields. Root cause: MORAI's VehicleInfo carries
its five vectors as geometry_msgs/Vector3, whose members are float64,
so reading them as float32 consumed half the bytes and split each
double across two fields -- one half landing on the exponent bits
(2^65), the other on the low mantissa (2^-63). Building the official
morai_v2_1_ros2_msgs package (nested Header + Vector3) made every field
decode cleanly: position tracks the map, throttle matches the commanded
value. There was never an uninitialized-memory bug on MORAI's side.

The sanity clamp below is kept anyway -- cheap, and it still guards
against a genuinely bad tick -- but it should no longer ever fire.

Subscribed:
    <vehicleinfo_topic>  (morai_v2_1_ros2_msgs/msg/VehicleInfo),
                         default /Car_1/vehicleinfo -- MUST match the
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
# Anything past this ceiling is unambiguously corrupt data, not a fast
# car. Retained as a cheap guard; with the official nested VehicleInfo
# it should never trip (see module docstring).
MAX_PLAUSIBLE_SPEED_MS = 55.0

# How to collapse MORAI's body-frame local_velocity vector into the
# scalar speed the controllers consume. MORAI-only: this node is never
# launched in `simulator:=carla` mode (start_adas.sh gates it behind the
# morai branch, and CARLA's carlaaccsim bridge publishes
# /Car_1/vehicle/speed itself as std_msgs/Float64), so neither value
# here can reach the CARLA pipeline.
#   'planar'  -- hypot(x, y): includes lateral slip during yaw
#                transients, where forward-only under-reads.
#   'forward' -- |x|: previous behaviour, longitudinal only, what a
#                real speedometer shows.
# z is excluded from both: it is vertical motion (slope, suspension),
# not ground speed.
SPEED_SOURCE_DEFAULT = 'planar'


class MoraiStateAdapter(Node):
    def __init__(self):
        super().__init__('Morai_State_Adapter')

        # Default corrected /Ego/ -> /Car_1/ (2026-08-15): the GroundTruth
        # entity in MORAI Studio publishes on /Car_1/vehicleinfo, so the
        # old default subscribed to a topic with zero publishers and this
        # node never received a single message.
        self.declare_parameter('vehicleinfo_topic', '/Car_1/vehicleinfo')
        self.declare_parameter('speed_topic', '/Car_1/vehicle/speed')
        self.declare_parameter('speed_source', SPEED_SOURCE_DEFAULT)
        vehicleinfo_topic = self.get_parameter('vehicleinfo_topic').value
        speed_topic = self.get_parameter('speed_topic').value
        self.speed_source = str(self.get_parameter('speed_source').value)
        if self.speed_source not in ('planar', 'forward'):
            self.get_logger().warn(
                f"unknown speed_source={self.speed_source!r} -- "
                f"falling back to {SPEED_SOURCE_DEFAULT!r}")
            self.speed_source = SPEED_SOURCE_DEFAULT

        self.speed_pub = self.create_publisher(Float64, speed_topic, 20)
        self.create_subscription(VehicleInfo, vehicleinfo_topic,
                                  self._on_vehicleinfo, 20)
        self._last_good_speed = 0.0

        self.get_logger().info(
            '=== MORAI State Adapter starting ===\n'
            f'    VehicleInfo in: {vehicleinfo_topic}\n'
            f'    Speed out:      {speed_topic}\n'
            f'    Speed source:   {self.speed_source} '
            + ('(hypot of local_velocity.x, .y)' if self.speed_source == 'planar'
               else '(|local_velocity.x|)'))

    def _on_vehicleinfo(self, msg: VehicleInfo):
        v = msg.local_velocity
        speed = (math.hypot(v.x, v.y) if self.speed_source == 'planar'
                 else abs(v.x))
        if not math.isfinite(speed) or speed > MAX_PLAUSIBLE_SPEED_MS:
            self.get_logger().warn(
                f'[sanity] rejected implausible local_velocity=({v.x!r}, {v.y!r}) '
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
