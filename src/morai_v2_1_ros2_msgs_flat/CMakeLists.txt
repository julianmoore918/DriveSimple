cmake_minimum_required(VERSION 3.8)
project(morai_v2_1_ros2_msgs_flat)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleManualControl.msg"
  "msg/VehicleInfo.msg"
)

ament_package()
