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
find_package(weeder_msgs REQUIRED)
find_package(spdlog_vendor REQUIRED)
find_package(spdlog REQUIRED)

include_directories(include)

add_library(obstacle_filter SHARED
  src/obstacle_filter.cpp
)
ament_target_dependencies(obstacle_filter
  sensor_msgs
)

add_library(velocity_planner_node_lib SHARED
  src/velocity_planner_node.cpp
  src/logging.cpp
)
target_link_libraries(velocity_planner_node_lib obstacle_filter)
target_link_libraries(velocity_planner_node_lib spdlog::spdlog)
ament_target_dependencies(velocity_planner_node_lib
  rclcpp
  sensor_msgs
  std_msgs
  weeder_msgs
)

# 让 SPDLOG_* 宏携带源码文件名与行号，并允许运行期切换到 debug/trace。
target_compile_definitions(velocity_planner_node_lib PRIVATE
  SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
)

add_executable(velocity_planner_node
  src/velocity_planner_main.cpp
)
target_link_libraries(velocity_planner_node velocity_planner_node_lib)
ament_target_dependencies(velocity_planner_node
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_obstacle_filter
    test/test_obstacle_filter.cpp
  )
  target_link_libraries(test_obstacle_filter obstacle_filter)
  ament_target_dependencies(test_obstacle_filter sensor_msgs)

  ament_add_gtest_executable(test_velocity_planner_integration
    test/test_velocity_planner_integration.cpp
  )
  target_link_libraries(test_velocity_planner_integration
    velocity_planner_node_lib
  )
  ament_target_dependencies(test_velocity_planner_integration
    rclcpp
    sensor_msgs
    weeder_msgs
  )
  set(integration_result_file
    "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_velocity_planner_integration.gtest.xml"
  )
  ament_add_test(test_velocity_planner_integration
    COMMAND $<TARGET_FILE:test_velocity_planner_integration>
            --gtest_output=xml:${integration_result_file}
    RESULT_FILE ${integration_result_file}
    APPEND_LIBRARY_DIRS ${CMAKE_CURRENT_BINARY_DIR}
    ENV ROS_DOMAIN_ID=41
    TIMEOUT 30
  )
endif()

install(TARGETS
  obstacle_filter
  velocity_planner_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS velocity_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
