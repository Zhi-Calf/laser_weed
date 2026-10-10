cmake_minimum_required(VERSION 3.8)
project(laser_weed_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# ✅ 协议库（默认 SHARED → 生成 .so）
add_library(laser_weed_protocol
  protocol/can_protocol.cpp
)

set_target_properties(laser_weed_protocol PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(laser_weed_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/protocol>
  $<INSTALL_INTERFACE:include>
)

# ✅ 安装库文件
install(TARGETS laser_weed_protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ✅ 安装头文件
install(DIRECTORY protocol/
  DESTINATION include/laser_weed_can/protocol
)

# ✅ ament 导出的正确 API（两个命令，不是 export_targets）
ament_export_libraries(laser_weed_protocol)
ament_export_include_directories(include)

# ✅ 让下游能通过 find_package 找到
ament_export_dependencies()
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # test_protocol.cpp 是自写 main, 用普通可执行 + ctest 注册, 不用 gtest
  add_executable(test_protocol
    test/test_protocol.cpp
  )
  target_link_libraries(test_protocol laser_weed_protocol)
  add_test(NAME test_protocol COMMAND test_protocol)
endif()
ament_package()
