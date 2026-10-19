cmake_minimum_required(VERSION 3.8)
project(weeder_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChassisCmd.msg"
  "msg/ChassisState.msg"
  "msg/ImplementCmd.msg"
  "msg/ImplementMode.msg"
  "msg/CalibrationResult.msg"
  "msg/RobotStatus.msg"
  "msg/Heartbeat.msg"
  "msg/NodeHealth.msg"
  "msg/SystemHealth.msg"
  "msg/GnssStatus.msg"
  "msg/FaultReport.msg"
  "msg/WorkMode.msg"
  "srv/CalibrationCommand.srv"
  "srv/SetImplementMode.srv"
  "srv/SetPathSource.srv"
  "action/ExecutePath.action"
  DEPENDENCIES std_msgs nav_msgs geometry_msgs builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
