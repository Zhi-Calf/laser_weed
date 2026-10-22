cmake_minimum_required(VERSION 3.8)
project(weeder_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChassisCmd.msg"
  "msg/ChassisState.msg"
  "msg/CropRowObservation.msg"
  "msg/ObstacleStatus.msg"
  "msg/Path.msg"
  "msg/Pose.msg"
  "msg/WorkMode.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
