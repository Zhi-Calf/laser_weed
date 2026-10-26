cmake_minimum_required(VERSION 3.8)
project(weeder_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(weeder_msgs REQUIRED)
find_package(spdlog_vendor REQUIRED)
find_package(spdlog REQUIRED)

# ---- 协议库: 纯 C++ 编解码, 不依赖 ROS ----
add_library(weeder_base_protocol STATIC
  src/protocol/can_protocol.cpp
)
set_target_properties(weeder_base_protocol PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_include_directories(weeder_base_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---- 库: SocketCAN 驱动 + 底盘节点 ----
# 协议层保留独立目标, 驱动与节点逻辑放在 weeder_base_lib, 与 tractor_base
# 的单包分层方式一致。
add_library(${PROJECT_NAME}_lib SHARED
  src/can_driver.cpp
  src/chassis_node.cpp
  src/logging.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  weeder_msgs
)

target_link_libraries(${PROJECT_NAME}_lib
  weeder_base_protocol
  spdlog::spdlog
)
# 让 SPDLOG_* 宏带上源码文件名与行号 (与 tractor_base 一致, 日志格式里的 [%s:%#])
target_compile_definitions(${PROJECT_NAME}_lib PRIVATE
  SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
)

# ---- 可执行文件: 只有 main ----
add_executable(chassis_node
  src/chassis_node_main.cpp
)
target_link_libraries(chassis_node ${PROJECT_NAME}_lib)

# ---- 安装 ----
install(TARGETS ${PROJECT_NAME}_lib weeder_base_protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS chassis_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib weeder_base_protocol)
ament_export_dependencies(rclcpp weeder_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_executable(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol weeder_base_protocol)
  add_test(NAME test_protocol COMMAND test_protocol)
endif()

ament_package()

