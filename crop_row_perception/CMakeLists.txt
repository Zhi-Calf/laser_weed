cmake_minimum_required(VERSION 3.16)
project(crop_row_perception)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(weeder_msgs REQUIRED)
find_package(Threads REQUIRED)
option(CROP_ROW_TEST_ONLY "Build only non-installed tests with the fake SDK" OFF)
if(CROP_ROW_TEST_ONLY)
  if(NOT BUILD_TESTING)
    message(FATAL_ERROR "CROP_ROW_TEST_ONLY requires BUILD_TESTING=ON")
  endif()
  find_path(ROW_NAV_HEADER_DIR rk_row_nav.h REQUIRED)
  set(SDK_INCLUDE_DIRS ${ROW_NAV_HEADER_DIR})
else()
  # Install the independent SDK first; pass its prefix via CMAKE_PREFIX_PATH.
  find_package(RowNav 0.1 REQUIRED CONFIG)
  set(SDK_INCLUDE_DIRS $<TARGET_PROPERTY:RowNav::rk_row_nav,INTERFACE_INCLUDE_DIRECTORIES>)
endif()

if(NOT CROP_ROW_TEST_ONLY)
add_library(crop_row_observer_node SHARED src/observer_node.cpp)
target_include_directories(crop_row_observer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(crop_row_observer_node RowNav::rk_row_nav Threads::Threads)
ament_target_dependencies(crop_row_observer_node rclcpp rcl_interfaces sensor_msgs weeder_msgs)
add_executable(crop_row_observer src/main.cpp)
target_link_libraries(crop_row_observer crop_row_observer_node)
ament_target_dependencies(crop_row_observer rclcpp)
# The SDK is installed independently, outside the colcon prefix. Preserve that
# dependency's runtime directory when CMake strips build-tree RPATH on install.
set_target_properties(crop_row_observer_node PROPERTIES
  INSTALL_RPATH "$ORIGIN" INSTALL_RPATH_USE_LINK_PATH TRUE)
set_target_properties(crop_row_observer PROPERTIES
  INSTALL_RPATH "$ORIGIN/.." INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS crop_row_observer_node LIBRARY DESTINATION lib)
install(TARGETS crop_row_observer RUNTIME DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/replay_images.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_crop_row_core test/test_core.cpp test/fake_sdk.cpp)
  target_include_directories(test_crop_row_core PRIVATE include)
  # Headers only. The test backend must never enter the installed observer.
  target_include_directories(test_crop_row_core SYSTEM PRIVATE
    ${SDK_INCLUDE_DIRS})
  target_link_libraries(test_crop_row_core Threads::Threads)
  add_test(NAME crop_row_core COMMAND test_crop_row_core)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_crop_row_observer
    test/test_observer.cpp src/observer_node.cpp test/fake_sdk.cpp)
  target_include_directories(test_crop_row_observer PRIVATE include)
  target_include_directories(test_crop_row_observer SYSTEM PRIVATE
    ${SDK_INCLUDE_DIRS})
  target_link_libraries(test_crop_row_observer Threads::Threads)
  ament_target_dependencies(test_crop_row_observer rclcpp rcl_interfaces sensor_msgs weeder_msgs)
  set_tests_properties(test_crop_row_observer PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=83")
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME crop_row_replay
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_replay.py
            ${CMAKE_CURRENT_SOURCE_DIR}/scripts/replay_images.py)
  set_tests_properties(crop_row_replay PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=84" TIMEOUT 20)
  if(NOT CROP_ROW_TEST_ONLY)
    add_test(NAME crop_row_production_startup
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_startup.py
              $<TARGET_FILE:crop_row_observer>
              ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/crop_row_observer)
    set_tests_properties(crop_row_production_startup PROPERTIES
      ENVIRONMENT "ROS_DOMAIN_ID=85" TIMEOUT 20)
  endif()
endif()

ament_package()
