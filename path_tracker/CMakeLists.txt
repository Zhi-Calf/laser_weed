cmake_minimum_required(VERSION 3.8)
project(path_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(weeder_msgs REQUIRED)
find_package(spdlog_vendor REQUIRED)
find_package(spdlog REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

set(DEPS
  rclcpp
  geometry_msgs
  std_msgs
  visualization_msgs
  weeder_msgs
)

# ---- 核心库：纯算法，不依赖 ROS 节点 ----
# 单独拆出，使单元测试无需引入完整 ROS 栈即可链接。
# 该结构与 MegaTrak global_path_planner 的弓字形算法库一致。
add_library(${PROJECT_NAME}_lib SHARED
  src/discrete_points_math.cpp
  src/lqr.cpp
  src/row_pid.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
  Eigen3::Eigen
  spdlog::spdlog
)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPS})

# ---- 节点库和可执行文件 ----
add_library(path_tracker_node_lib SHARED
  src/logging.cpp
  src/path_tracker_node.cpp
)
target_link_libraries(path_tracker_node_lib
  ${PROJECT_NAME}_lib
  spdlog::spdlog
)
ament_target_dependencies(path_tracker_node_lib ${DEPS})
target_compile_definitions(path_tracker_node_lib PRIVATE
  SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
)

add_executable(path_tracker_node src/path_tracker_main.cpp)
target_link_libraries(path_tracker_node path_tracker_node_lib)
ament_target_dependencies(path_tracker_node rclcpp)

# ---- 测试 ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_lqr_kinematics test/test_lqr_kinematics.cpp)
  target_link_libraries(test_lqr_kinematics ${PROJECT_NAME}_lib)

  ament_add_gtest(
    test_discrete_points_math test/test_discrete_points_math.cpp)
  target_link_libraries(test_discrete_points_math ${PROJECT_NAME}_lib)

  ament_add_gtest(test_row_pid test/test_row_pid.cpp)
  target_link_libraries(test_row_pid ${PROJECT_NAME}_lib)

  ament_add_gtest_executable(test_path_tracker_obstacle_integration
    test/test_path_tracker_obstacle_integration.cpp
  )
  target_link_libraries(test_path_tracker_obstacle_integration
    path_tracker_node_lib
  )
  ament_target_dependencies(test_path_tracker_obstacle_integration
    geometry_msgs
    rclcpp
    weeder_msgs
  )
  set(path_tracker_integration_result_file
    "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_path_tracker_obstacle_integration.gtest.xml"
  )
  ament_add_test(test_path_tracker_obstacle_integration
    COMMAND $<TARGET_FILE:test_path_tracker_obstacle_integration>
            --gtest_output=xml:${path_tracker_integration_result_file}
    RESULT_FILE ${path_tracker_integration_result_file}
    APPEND_LIBRARY_DIRS ${CMAKE_CURRENT_BINARY_DIR}
    ENV ROS_DOMAIN_ID=42
    TIMEOUT 30
  )
endif()

# ---- 安装 ----
# 共享库安装到 lib/；可执行文件必须安装到 lib/${PROJECT_NAME}/，
# 因为 ros2 run <包名> <可执行文件> 只在该目录搜索。
# 安装到 bin/ 会导致 launch 报找不到可执行文件。
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS path_tracker_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS path_tracker_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${DEPS} spdlog_vendor)
ament_package()
