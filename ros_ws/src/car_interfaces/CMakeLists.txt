cmake_minimum_required(VERSION 3.22)
project(car_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BattStatus.msg"
  "msg/ControllerState.msg"
  "msg/DriveMode.msg"
  "msg/LightsCmd.msg"
  "msg/LightsState.msg"
  "msg/SteerCmd.msg"
  "msg/SteerState.msg"
  "msg/WheelOdom.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
