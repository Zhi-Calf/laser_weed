cmake_minimum_required(VERSION 3.8)
project(tractor_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tractor_msgs REQUIRED)
find_package(spdlog_vendor REQUIRED)
find_package(spdlog REQUIRED)

# ---- Aggregation core (no rclcpp) ----
# Kept separate so the timeout / startup-grace / aggregation matrix is unit
# testable without spinning a node or faking a clock. Same split as
# tractor_kpi_core. Do not add rclcpp to this target.
add_library(${PROJECT_NAME}_core SHARED
  src/watch_table.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---- Node library ----
add_library(${PROJECT_NAME}_lib SHARED
  src/monitor_node.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_lib ${PROJECT_NAME}_core spdlog::spdlog)
ament_target_dependencies(${PROJECT_NAME}_lib rclcpp tractor_msgs)

# 让 SPDLOG_* 宏携带源码文件名与行号（source location）
target_compile_definitions(${PROJECT_NAME}_lib PRIVATE
  SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
)

# ---- Standalone executable ----
add_executable(${PROJECT_NAME}_node
  src/monitor_node_main.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)

# ---- Install ----
install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 装到 lib/ 而不是 share/, 这样 `ros2 run tractor_monitor health_table.py` 可用。
# 与 tractor_bringup 里 Python 脚本的做法一致。
install(PROGRAMS
  scripts/health_table.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core ${PROJECT_NAME}_lib)
# spdlog 刻意不在导出列表里: 只有 _lib(monitor_node.cpp) 用它,
# heartbeat_publisher.hpp 不用。加进来会让六个被监控包都被迫依赖 spdlog。
# rclcpp and tractor_msgs are exported because heartbeat_publisher.hpp is a
# header-only target consumed by every monitored package: they get the include
# path and both dependencies from a single <depend>tractor_monitor</depend>.
ament_export_dependencies(rclcpp tractor_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_watch_table test/test_watch_table.cpp)
  target_link_libraries(test_watch_table ${PROJECT_NAME}_core)

  # 参数与 yaml 一致性检查 (脚本在仓库根 scripts/, 各包共用同一份): 代码里 declare_parameter
  # 了但 yaml 漏写的键, 现场只表现为"这个参数改了没用", 用构建期测试拦住。
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  ament_add_test(params_in_yaml
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/check_params.py
            --src ${CMAKE_CURRENT_SOURCE_DIR}/src
            --yaml ${CMAKE_CURRENT_SOURCE_DIR}/config/tractor_monitor.yaml
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()

ament_package()
