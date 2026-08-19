#int32 stamp_sec
#uint32 stamp_nanosec
#string id
#float32 location_x
#float32 location_y
#float32 location_z
#float32 rotation_x
#float32 rotation_y
#float32 rotation_z
#float32 local_velocity_x
#float32 local_velocity_y
#float32 local_velocity_z
#float32 local_acceleration_x
#float32 local_acceleration_y
#float32 local_acceleration_z
#float32 angular_velocity_x
#float32 angular_velocity_y
#float32 angular_velocity_z
#float32 throttle
#float32 brake
#float32 steer_angle

# VehicleInfo
# Status telemetry of the ego vehicle for closed-loop control.
# Topic: /VehicleInfo

std_msgs/Header header
geometry_msgs/Vector3 position  # Current position in ENU coordinates [m]
geometry_msgs/Vector3 rotation  # Current rotation in RPY [deg]
geometry_msgs/Vector3 local_velocity  # Velocity vector [m/s]
geometry_msgs/Vector3 local_acceleration  # Acceleration vector [m/s^2]
geometry_msgs/Vector3 angular_velocity  # Angular velocity [deg/s]

float32 throttle  # Accelerator pedal feedback, range 0~1
float32 brake  # Brake pedal feedback, range 0~1
float32 steer_angle  # Steering cmd feedback [-1~1]