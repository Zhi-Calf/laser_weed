cmake_minimum_required(VERSION 3.8)
project(gnss_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

# 优化级别交给 build type (Release = -O3 -DNDEBUG), 这里不要再 add_compile_options(-O2/-O3):
# add_compile_options 排在 CMAKE_CXX_FLAGS_RELEASE 之后, GCC 以**最后一个 -O 为准** —— 曾经这里
# 的 add_compile_options(-O2) 实际把 -O3 降成了 -O2 (实测 enabled 优化项: -O2 = 140 < -O3 = 155)。
# 下面只保留与 CPU 架构相关的选项。
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  # -march=armv8-a 仅对 aarch64 有效; x86_64 开发机加上会编译失败 (unrecognized argument)。
  # 与下方千寻 SDK 的架构判据保持一致。
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    add_compile_options(-march=armv8-a)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(weeder_msgs REQUIRED)
find_package(spdlog_vendor REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# GeographicLib (provides LocalCartesian geodetic projection).
#
# GeographicLib renamed its library between 1.x and 2.x: Jammy/Humble ships
# libGeographic.so, Noble/Jazzy ships libGeographicLib.so. Hard-coding either
# name builds on one and fails on the other. Search for both.
# Mirrors control_node/path_controller/CMakeLists.txt.
find_library(GEOGRAPHIC_LIBRARY
  NAMES GeographicLib Geographic
  DOC "GeographicLib (2.x: GeographicLib, 1.x: Geographic)")
if(NOT GEOGRAPHIC_LIBRARY)
  message(FATAL_ERROR
    "GeographicLib not found. Install libgeographiclib-dev (Noble/Jazzy) or "
    "libgeographic-dev (Jammy/Humble).")
endif()

# 千寻 SDK 1.29.5 (RK3588/aarch64 release)。
# 随包的 release 库是 aarch64 静态库；x86 开发机没有对应版本时跳过 sdk_client_node。
# 头文件在 ThirdParty/qxwz/include, 库在 ThirdParty/qxwz/lib/aarch64。
set(QXWZ_SDK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/qxwz/include)
set(QXWZ_SDK_LIB "" CACHE FILEPATH
  "Path to the qxwz-pssdk 1.29.x static library (default: bundled aarch64 release)")
if(NOT QXWZ_SDK_LIB)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(QXWZ_SDK_LIB
      ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/qxwz/lib/aarch64/libqxwz-pssdk-1.29.5-20260916--unix--production-release.a)
  endif()
endif()
if(QXWZ_SDK_LIB AND EXISTS "${QXWZ_SDK_LIB}")
  set(QXWZ_SDK_AVAILABLE TRUE)
else()
  if(QXWZ_SDK_LIB)
    message(WARNING "千寻 SDK 库不存在: ${QXWZ_SDK_LIB}; sdk_client_node 不会构建")
  else()
    message(WARNING "千寻 SDK 1.29.5 随包 release 库仅支持 aarch64 (RK3588); 当前 ${CMAKE_SYSTEM_PROCESSOR} 不构建 sdk_client_node。可通过 -DQXWZ_SDK_LIB=<path> 指定匹配的库。")
  endif()
  set(QXWZ_SDK_AVAILABLE FALSE)
endif()

find_package(Threads REQUIRED)

add_executable(gnss_driver_node
  src/gnss_driver_main.cpp
  src/serial_port.cpp
  src/giavp_parser.cpp
  src/gnss_driver.cpp
)

target_include_directories(gnss_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(gnss_driver_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  weeder_msgs
)

target_link_libraries(gnss_driver_node ${GEOGRAPHIC_LIBRARY} spdlog::spdlog yaml-cpp)

# 让 SPDLOG_* 宏携带源码文件名与行号（source location）
target_compile_definitions(gnss_driver_node PRIVATE
  SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
)

add_executable(ntrip_client_node
  src/ntrip_client_main.cpp
  src/serial_port.cpp
  src/ntrip_client.cpp
)

target_include_directories(ntrip_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ntrip_client_node
  rclcpp
)

target_link_libraries(ntrip_client_node spdlog::spdlog)

# 让 SPDLOG_* 宏携带源码文件名与行号（source location）
target_compile_definitions(ntrip_client_node PRIVATE
  SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
)

if(QXWZ_SDK_AVAILABLE)
  add_executable(sdk_client_node
    src/sdk_client_main.cpp
    src/serial_port.cpp
    src/sdk_client.cpp
  )

  target_include_directories(sdk_client_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${QXWZ_SDK_INCLUDE_DIR}
  )

  ament_target_dependencies(sdk_client_node
    rclcpp
  )

  # qxwz release 静态库需要 pthread / dl / libm。
  target_link_libraries(sdk_client_node spdlog::spdlog ${QXWZ_SDK_LIB} Threads::Threads ${CMAKE_DL_LIBS} m)

  # 让 SPDLOG_* 宏携带源码文件名与行号（source location）
  target_compile_definitions(sdk_client_node PRIVATE
    SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
  )
else()
  # 没有匹配的 aarch64 库时仍编译一次源码，尽早发现 SDK 头文件/API 漂移。
  add_library(sdk_client_source_check OBJECT src/sdk_client.cpp)
  target_include_directories(sdk_client_source_check PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${QXWZ_SDK_INCLUDE_DIR}
  )
  ament_target_dependencies(sdk_client_source_check rclcpp)
  target_compile_definitions(sdk_client_source_check PRIVATE
    SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
  )
endif()

install(TARGETS gnss_driver_node ntrip_client_node
  DESTINATION lib/${PROJECT_NAME}
)

if(QXWZ_SDK_AVAILABLE)
  install(TARGETS sdk_client_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 调试/监控脚本：装到 lib 下以便 `ros2 run gnss_localization <script>`
install(PROGRAMS
  scripts/gnss_dashboard.py
  scripts/gnss_monitor.sh
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Tests ----
if(BUILD_TESTING)
  # 参数与 yaml 一致性检查 (脚本在仓库根 scripts/, 各包共用同一份): 代码里 declare_parameter
  # 了但 yaml 漏写的键, 现场只表现为"这个参数改了没用", 用构建期测试拦住
  # (calib_offset_max_xy / calib_offset_max_heading 就这样漏过)。
  # 三个 yaml 一起比: 脚本按"并集"判据, 键写在哪个节点的 yaml 里都算已配置。
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  ament_add_test(params_in_yaml
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/check_params.py
            --src ${CMAKE_CURRENT_SOURCE_DIR}/src
            --yaml ${CMAKE_CURRENT_SOURCE_DIR}/config/gnss_driver.yaml
                   ${CMAKE_CURRENT_SOURCE_DIR}/config/ntrip_client.yaml
                   ${CMAKE_CURRENT_SOURCE_DIR}/config/sdk_client.yaml
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)

  # 参数写回 yaml 的契约 (src/yaml_persist.hpp —— 与 tractor_calibration 的同名文件是镜像实现):
  # 运行期改标定参数会把新值落盘, 规则必须"保留注释与缩进、重复键全替换并回报、缺键插进
  # ros__parameters 映射内、等值不重写、原子替换" —— 写坏一次 yaml 就是整车配置丢失, 必须钉住。
  # 与 tractor_calibration 的 test_yaml_persist 是同一套用例 (那份测的是镜像的另一份实现)。
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_yaml_persist test/test_yaml_persist.cpp)
  target_include_directories(test_yaml_persist PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

  # $GIAVP 解析器的有效性校验: 数值字段出现 nan/inf -> 整句判无效 (不给下游发带 NaN 的 fix/odom/imu/vel)。
  # 无接收机的台架上唯一能钉住这条的地方 (报文喂不进节点), 见 tmp/verify_gnss_param_nan.sh 的说明。
  ament_add_gtest(test_giavp_parser test/test_giavp_parser.cpp src/giavp_parser.cpp)
  target_include_directories(test_giavp_parser PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src)
endif()

ament_package()

