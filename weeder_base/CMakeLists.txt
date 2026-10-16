cmake_minimum_required(VERSION 3.8)
project(weeder_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_weed_can REQUIRED)

include_directories(include)

add_library(weeder_can_driver
  src/can_driver.cpp
)
ament_target_dependencies(weeder_can_driver
  rclcpp
  laser_weed_can
)

add_executable(chassis_node
  src/chassis_node.cpp
)
target_link_libraries(chassis_node
  weeder_can_driver
)
ament_target_dependencies(chassis_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  laser_weed_can
)

install(TARGETS chassis_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS weeder_can_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
