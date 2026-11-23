cmake_minimum_required(VERSION 3.8)
project(vehicle_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChassisCmd.msg"
  "msg/ChassisState.msg"
  "msg/CalibrationResult.msg"
  "msg/CropRowObservation.msg"
  "msg/GnssStatus.msg"
  "msg/ImplementMode.msg"
  "msg/ObstacleStatus.msg"
  "msg/Path.msg"
  "msg/Pose.msg"
  "msg/WorkMode.msg"
  "srv/CalibrationCommand.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
