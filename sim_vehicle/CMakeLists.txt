cmake_minimum_required(VERSION 3.8)
project(sim_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spdlog_vendor REQUIRED)
find_package(spdlog REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(weeder_msgs REQUIRED)

set(dependencies
  geometry_msgs
  rclcpp
  std_msgs
  tf2_msgs
  visualization_msgs
  weeder_msgs
)

add_library(sim_vehicle_kinematics STATIC
  src/kinematics.cpp
)
target_include_directories(sim_vehicle_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(sim_vehicle_logging_lib SHARED
  src/logging.cpp
)
target_include_directories(sim_vehicle_logging_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(sim_vehicle_logging_lib
  spdlog::spdlog
)
target_compile_definitions(sim_vehicle_logging_lib PRIVATE
  SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
)

add_library(sim_vehicle_node_lib SHARED
  src/sim_vehicle_node.cpp
)
target_include_directories(sim_vehicle_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(sim_vehicle_node_lib
  sim_vehicle_kinematics
  sim_vehicle_logging_lib
)
ament_target_dependencies(sim_vehicle_node_lib ${dependencies})
target_compile_definitions(sim_vehicle_node_lib PRIVATE
  SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
)

add_library(sim_orchestrator_node_lib SHARED
  src/sim_orchestrator_node.cpp
)
target_include_directories(sim_orchestrator_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(sim_orchestrator_node_lib
  sim_vehicle_logging_lib
)
ament_target_dependencies(sim_orchestrator_node_lib ${dependencies})
target_compile_definitions(sim_orchestrator_node_lib PRIVATE
  SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
)

add_executable(sim_vehicle
  src/sim_vehicle_main.cpp
)
target_link_libraries(sim_vehicle
  sim_vehicle_node_lib
)

add_executable(sim_orchestrator
  src/sim_orchestrator_main.cpp
)
target_link_libraries(sim_orchestrator
  sim_orchestrator_node_lib
)

install(TARGETS
  sim_vehicle_kinematics
  sim_vehicle_logging_lib
  sim_vehicle_node_lib
  sim_orchestrator_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS sim_vehicle sim_orchestrator
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematics
    test/test_kinematics.cpp
  )
  target_link_libraries(test_kinematics
    sim_vehicle_kinematics
  )

  ament_add_gtest(test_orchestrator
    test/test_orchestrator.cpp
  )
  target_link_libraries(test_orchestrator
    sim_orchestrator_node_lib
  )
  ament_target_dependencies(test_orchestrator
    rclcpp
    std_msgs
    weeder_msgs
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(sim_vehicle_kinematics sim_vehicle_logging_lib)
ament_export_dependencies(${dependencies} spdlog_vendor)
ament_package()
