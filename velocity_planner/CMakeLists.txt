cmake_minimum_required(VERSION 3.8)
project(velocity_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vehicle_interfaces REQUIRED)
find_package(spdlog_vendor REQUIRED)
find_package(spdlog REQUIRED)

include_directories(include)

add_library(obstacle_roi_detector SHARED
  src/obstacle_roi_detector.cpp
)
ament_target_dependencies(obstacle_roi_detector
  sensor_msgs
)

add_library(obstacle_monitor_node_lib SHARED
  src/obstacle_monitor_node.cpp
  src/logging.cpp
)
target_link_libraries(obstacle_monitor_node_lib obstacle_roi_detector)
target_link_libraries(obstacle_monitor_node_lib spdlog::spdlog)
ament_target_dependencies(obstacle_monitor_node_lib
  rclcpp
  sensor_msgs
  std_msgs
  vehicle_interfaces
)

# 让 SPDLOG_* 宏携带源码文件名与行号，并允许运行期切换到 debug/trace。
target_compile_definitions(obstacle_monitor_node_lib PRIVATE
  SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
)

add_executable(obstacle_monitor_node
  apps/obstacle_monitor_node_main.cpp
)
target_link_libraries(obstacle_monitor_node obstacle_monitor_node_lib)
ament_target_dependencies(obstacle_monitor_node
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_obstacle_roi_detector
    test/test_obstacle_roi_detector.cpp
  )
  target_link_libraries(test_obstacle_roi_detector obstacle_roi_detector)
  ament_target_dependencies(test_obstacle_roi_detector sensor_msgs)

  ament_add_gtest_executable(test_obstacle_monitor_integration
    test/test_obstacle_monitor_integration.cpp
  )
  target_link_libraries(test_obstacle_monitor_integration
    obstacle_monitor_node_lib
  )
  ament_target_dependencies(test_obstacle_monitor_integration
    rclcpp
    sensor_msgs
    vehicle_interfaces
  )
  set(integration_result_file
    "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_obstacle_monitor_integration.gtest.xml"
  )
  ament_add_test(test_obstacle_monitor_integration
    COMMAND $<TARGET_FILE:test_obstacle_monitor_integration>
            --gtest_output=xml:${integration_result_file}
    RESULT_FILE ${integration_result_file}
    APPEND_LIBRARY_DIRS ${CMAKE_CURRENT_BINARY_DIR}
    ENV ROS_DOMAIN_ID=41
    TIMEOUT 30
  )
endif()

install(TARGETS
  obstacle_roi_detector
  obstacle_monitor_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS obstacle_monitor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
