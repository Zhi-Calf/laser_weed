cmake_minimum_required(VERSION 3.8)
project(lqr_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
# tractor_msgs: WorkMode + Heartbeat. tractor_monitor: header-only heartbeat
# publisher. Both come from MegaTrak (see README), so they must be present in
# the workspace before this package can build.
find_package(tractor_msgs REQUIRED)
find_package(tractor_monitor REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

set(DEPS
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  tf2
  tractor_msgs
  tractor_monitor
)

# ---- Core library: pure algorithms, no ROS node dependencies ----
# Split out so the unit tests can link it without pulling in the ROS stack,
# the same pattern MegaTrak's global_path_planner uses for its bow algorithm.
add_library(${PROJECT_NAME}_lib SHARED
  src/lqr.cpp
  src/path_profile.cpp
)
target_link_libraries(${PROJECT_NAME}_lib Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPS})

# ---- Node executable ----
add_executable(lqr_node src/lqr_node.cpp src/lqr_control_main.cpp)
target_link_libraries(lqr_node ${PROJECT_NAME}_lib)
ament_target_dependencies(lqr_node ${DEPS})

# ---- Tests ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_lqr_kinematics test/test_lqr_kinematics.cpp)
  target_link_libraries(test_lqr_kinematics ${PROJECT_NAME}_lib)

  ament_add_gtest(test_path_profile test/test_path_profile.cpp)
  target_link_libraries(test_path_profile ${PROJECT_NAME}_lib)
endif()

# ---- Install ----
# The shared library goes to lib/; the executable MUST go to lib/${PROJECT_NAME}/
# because `ros2 run <pkg> <exe>` only searches that directory. Installing it to
# bin/ would make the launch file fail with "executable not found".
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS lqr_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
