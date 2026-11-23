cmake_minimum_required(VERSION 3.8)
project(app_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(weeder_msgs REQUIRED)
find_package(spdlog_vendor REQUIRED)
find_package(spdlog REQUIRED)
find_package(nlohmann_json REQUIRED)
# Ubuntu 22.04 提供 Paho MQTT C++ 1.2.0；更高版本保持同一公共 API。
find_package(PahoMqttCpp 1.2 REQUIRED)
find_package(CURL REQUIRED)
find_package(OpenSSL REQUIRED)

# GeographicLib 1.x (Jammy/Humble) 叫 libGeographic, 2.x (Noble/Jazzy) 叫 libGeographicLib,
# 两个名字都找 (与 global_path_planner 相同的处理)。
find_library(GEOGRAPHICLIB_LIBRARY
  NAMES GeographicLib Geographic
  DOC "GeographicLib (2.x: GeographicLib, 1.x: Geographic)")
if(NOT GEOGRAPHICLIB_LIBRARY)
  message(FATAL_ERROR
    "GeographicLib not found. Install libgeographiclib-dev (Noble/Jazzy) or "
    "libgeographic-dev (Jammy/Humble).")
endif()

# ---- 第三方: cpp-httplib (vendored 单头文件, MIT, 见 ThirdParty/cpp-httplib/README.md) ----
# 打开 OpenSSL 支持才有 https (SSLServer)。SYSTEM: 不让第三方头的告警淹没本包的 -Wall。
add_library(cpp_httplib INTERFACE)
target_include_directories(cpp_httplib SYSTEM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/cpp-httplib)
target_compile_definitions(cpp_httplib INTERFACE CPPHTTPLIB_OPENSSL_SUPPORT)
target_link_libraries(cpp_httplib INTERFACE OpenSSL::SSL OpenSSL::Crypto)

# ---- 编解码 / 校验核心 (no rclcpp) ----
# 消息 <-> JSON 的转换、路径文件校验都在这里, 不起节点就能单测。不要往这个 target 加 rclcpp。
add_library(${PROJECT_NAME}_codec SHARED
  src/farm_device.cpp
  src/path_file.cpp
)
target_include_directories(${PROJECT_NAME}_codec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_codec nlohmann_json::nlohmann_json OpenSSL::Crypto)
ament_target_dependencies(${PROJECT_NAME}_codec weeder_msgs)

# ---- 两个节点共用: spdlog 日志配置 ----
add_library(${PROJECT_NAME}_common SHARED
  src/logging.cpp
)
target_include_directories(${PROJECT_NAME}_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_common spdlog::spdlog)

# ---- MQTT 桥 ----
add_library(${PROJECT_NAME}_mqtt SHARED
  src/mqtt_client.cpp
  src/mqtt_bridge_node.cpp
)
target_link_libraries(${PROJECT_NAME}_mqtt
  ${PROJECT_NAME}_codec
  ${PROJECT_NAME}_common
  spdlog::spdlog
  PahoMqttCpp::paho-mqttpp3
)
ament_target_dependencies(${PROJECT_NAME}_mqtt rclcpp weeder_msgs)

add_executable(mqtt_bridge_node src/mqtt_bridge_node_main.cpp)
target_link_libraries(mqtt_bridge_node ${PROJECT_NAME}_mqtt)
ament_target_dependencies(mqtt_bridge_node rclcpp)

# ---- HTTP 服务 ----
add_library(${PROJECT_NAME}_http SHARED
  src/http_bridge_node.cpp
)
target_link_libraries(${PROJECT_NAME}_http ${PROJECT_NAME}_codec ${PROJECT_NAME}_common spdlog::spdlog cpp_httplib CURL::libcurl ${GEOGRAPHICLIB_LIBRARY})
ament_target_dependencies(${PROJECT_NAME}_http rclcpp nav_msgs sensor_msgs)

add_executable(http_bridge_node src/http_bridge_node_main.cpp)
target_link_libraries(http_bridge_node ${PROJECT_NAME}_http)
ament_target_dependencies(http_bridge_node rclcpp)

# 让 SPDLOG_* 宏携带源码文件名与行号（source location）
foreach(target ${PROJECT_NAME}_common ${PROJECT_NAME}_mqtt ${PROJECT_NAME}_http)
  target_compile_definitions(${target} PRIVATE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE)
endforeach()

# ---- Install ----
install(TARGETS ${PROJECT_NAME}_codec ${PROJECT_NAME}_common ${PROJECT_NAME}_mqtt ${PROJECT_NAME}_http
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mqtt_bridge_node http_bridge_node
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_farm_device test/test_farm_device.cpp)
  target_link_libraries(test_farm_device ${PROJECT_NAME}_codec)

  ament_add_gtest(test_path_file test/test_path_file.cpp)
  target_link_libraries(test_path_file ${PROJECT_NAME}_codec)

  # 参数与 yaml 一致性检查 (仓库根 scripts/, 各包共用同一份)。脚本按目录扫源码, 所以两个节点的
  # 声明与两份 yaml 合在一起比对: 抓得住"声明了没进 yaml / yaml 有残留键", 抓不住"写进了另一个节点的 yaml"。
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  ament_add_test(params_in_yaml
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/check_params.py
            --src ${CMAKE_CURRENT_SOURCE_DIR}/src
            --yaml ${CMAKE_CURRENT_SOURCE_DIR}/config/mqtt_bridge.yaml ${CMAKE_CURRENT_SOURCE_DIR}/config/http_bridge.yaml
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()

ament_package()
