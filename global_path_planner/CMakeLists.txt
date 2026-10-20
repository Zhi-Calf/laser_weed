cmake_minimum_required(VERSION 3.8)
project(global_path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译类型: 本包强制 Release (几何内核必须开优化)。
# 规划一次要跑上万次 Clipper2 偏置 / 扫描线裁剪 + Boost.Geometry 插值, 无优化比 -O3 慢一个
# 量级。注意 colcon 默认**不传** CMAKE_BUILD_TYPE (实测各包 CMakeCache 里该值为空), 仓内
# 每个 C++ 包都在自己的 CMakeLists 里定 build type, 所以这里也必须自己写。
# 用 CACHE + FORCE 而不是裸 set(), 两点用意:
#   * CMAKE_BUILD_TYPE 进 cache, 与真实编译 flags 一致 —— 裸 set() 不改 cache, 排查时会看到
#     "cache 里是空的、编译命令里却是 -O3"这种自相矛盾的现象;
#   * 无条件覆盖命令行传入的值: 即使有人 --cmake-args -DCMAKE_BUILD_TYPE=Debug 也仍是 Release。
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (forced to Release by this package)" FORCE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(weeder_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(spdlog_vendor REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost REQUIRED)

# ---- 第三方: Clipper2 (vendored, BSL-1.0, 见 ThirdParty/clipper2/README.md) ----
# 全覆盖路径规划 (bow.cpp) 用 Clipper2 做边界偏置 / 半平面求交 / 扫描线裁剪。
#
# 随包携带 (包内 ThirdParty/clipper2), 不再用跨包相对路径 —— 本包可整目录单独拷出构建。
# 代价是 cppcheck 的 hook 会以包目录为根递归 glob 全部源码, 因此下方 BUILD_TESTING 里
# 用 AMENT_LINT_AUTO_FILE_EXCLUDE 把第三方目录排除 (否则会报出 7 千多行非本包代码)。
set(CLIPPER2_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/clipper2)
add_library(clipper2 STATIC
  ${CLIPPER2_DIR}/src/clipper.engine.cpp
  ${CLIPPER2_DIR}/src/clipper.offset.cpp
  ${CLIPPER2_DIR}/src/clipper.rectclip.cpp)
target_include_directories(clipper2 PUBLIC ${CLIPPER2_DIR}/include)
# 第三方代码不参与本包的 -Wall -Wextra -Wpedantic 判定 (目标级选项排在目录级之后, -w 生效)
target_compile_options(clipper2 PRIVATE -w)
# 本包的核心库是 SHARED (与 tractor_base / tractor_calibration 一致), 而静态的 clipper2 会
# 被整个链进去 —— 缺少 -fPIC 时链接期直接失败:
#   "relocation R_X86_64_PC32 against symbol ... can not be used when making a shared object;
#    recompile with -fPIC"。
# 第三方源码不改, 只给目标加位置无关代码属性。
set_target_properties(clipper2 PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---- 本包依赖 (与 package.xml 的 <depend> 对应) ----
set(dependencies
  rclcpp
  geometry_msgs
  visualization_msgs
  weeder_msgs
)

# ---- Library ----
# 算法 + 节点实现编进库, 可执行文件只留 main()。
#   * ${PROJECT_NAME}_lib : 规划内核 (src/bow.cpp) + 节点实现 (src/global_path_planner.cpp);
#   * ${PROJECT_NAME}_node: 薄可执行文件 (src/global_path_planner_main.cpp)。
# 单测与其它消费者直接链库: 算法只编一遍, 且与真机跑的是同一份目标文件。
add_library(${PROJECT_NAME}_lib SHARED
  src/bow.cpp
  src/global_path_planner.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}_lib ${dependencies})

# link libraries
#
# GeographicLib renamed its library between 1.x and 2.x: Jammy/Humble ships
# libGeographic.so, Noble/Jazzy ships libGeographicLib.so. Hard-coding either
# name builds on one and fails to LINK on the other with "cannot find
# -lGeographic", after every translation unit has compiled — which reads as a
# broken package rather than a missing distro-renamed library. Search for both.
find_library(GEOGRAPHICLIB_LIBRARY
  NAMES GeographicLib Geographic
  DOC "GeographicLib (2.x: GeographicLib, 1.x: Geographic)")
if(NOT GEOGRAPHICLIB_LIBRARY)
  message(FATAL_ERROR
    "GeographicLib not found. Install libgeographiclib-dev (Noble/Jazzy) or "
    "libgeographic-dev (Jammy/Humble).")
endif()

target_link_libraries(${PROJECT_NAME}_lib ${GEOGRAPHICLIB_LIBRARY} spdlog::spdlog clipper2)

# 让 SPDLOG_* 宏携带源码文件名与行号（source location）。
# 缺了它 SPDLOG_ACTIVE_LEVEL 取默认的 INFO, SPDLOG_DEBUG/TRACE 会被编译掉 ——
# 那样 yaml 里把 log_level 调成 debug 也不会有任何 DEBUG 输出。
target_compile_definitions(${PROJECT_NAME}_lib PRIVATE
  SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_TRACE
)

# ---- Standalone executable (只有 main) ----
add_executable(${PROJECT_NAME}_node
  src/global_path_planner_main.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)

# ---- Install ----
install(TARGETS ${PROJECT_NAME}_lib
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

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(${dependencies} spdlog_vendor)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 本包的规划内核来自既有规划算法实现 (含 Eigen/Boost 适配与 BOW 相关代码), 与仓内其它包
  # 的 ROS 风格并不一致, 但改造它属于"重排上万行第三方衍生代码" —— 收益低、风险高, 因此
  # 只保留不影响构建的检查, 停用与格式/版权头相关的三项:
  #   copyright  : 文件缺版权头; 补什么版权归属需业主确认, 不擅自添加
  #   cpplint    : 63 处 (include 顺序/行宽/版权头等), 修它等于整体重排
  #   uncrustify : 同上 (重排)
  # 将来若要接受这些改动, 删掉下面几行 set(.. TRUE) 即可恢复检查。
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # cppcheck 默认按 C 语言解析 .h —— 实测在 include/global_path_planner/bow.hpp 与随包的
  # clipper 头上报过 "syntaxError: 'std::size_t' is invalid C code"。显式指定语言后按真实
  # 语言解析; hook (ament_cmake_cppcheck_lint_hook.cmake) 会读取该变量并传 --language。
  # 本包仍有 .h (第三方 clipper), 所以这项要留着。
  set(ament_cmake_cppcheck_LANGUAGE c++)

  # 包内随带的第三方代码 (ThirdParty/clipper2) 不参与 cppcheck: 不排除就会把 7 千多行
  # 非本包代码一起报出来 (cppcheck 实测报 danglingTemporaryLifetime 等)。
  # cpplint/uncrustify/copyright 已在上方停用, 这是仅剩的一项静态检查, 故只排除目录。
  #
  # 模式的讲究 (实测结论, 不要改):
  #   * 必须相对于"包源码目录"且末尾带 /*。hook 调 ament_cppcheck 时**不传文件列表**,
  #     cppcheck 在其 WORKING_DIRECTORY (= 包源码目录) 下递归, 内部文件路径是相对形式;
  #     绝对模式 (${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/* 、 */ThirdParty/*) 实测不匹配。
  #   * ament_cppcheck 把 --exclude <x> 转成 cppcheck 的 --suppress=*:<x>, 裸目录
  #     (ThirdParty) 匹配不到 ThirdParty/clipper2/src/*.cpp。
  #   * 大小写要与目录名一致 (目录实际叫 ThirdParty): cppcheck 的 suppress 通配是大小写
  #     敏感的, 写成 third_party/* 在 Linux 上匹配不到, 排除会静默失效 (本机 cppcheck 2.7
  #     默认被 ament 跳过, 这个坑只在放开慢版本检查 AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1
  #     时才会暴露)。
  list(APPEND AMENT_LINT_AUTO_FILE_EXCLUDE ThirdParty/*)
  ament_lint_auto_find_test_dependencies()

  # 参数与 yaml 一致性检查 (脚本在仓库根 scripts/, 各包共用同一份): 代码里 declare_parameter
  # 了但 yaml 漏写的键, 现场只表现为"这个参数改了没用", 用构建期测试拦住。
  # 地块顶点现在由 YAML 的 poly 扁平数组提供, 参数名全是字面量, 不需要 --allow-prefix。
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  ament_add_test(params_in_yaml
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/check_params.py
            --src ${CMAKE_CURRENT_SOURCE_DIR}/src
            --yaml ${CMAKE_CURRENT_SOURCE_DIR}/config/global_path_planner.yaml
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)

  # (这里曾经有 test_yaml_persist: "参数写回 yaml"的契约测试。它随"运行期可改的参数全删 + 参数持久化
  #  (src/yaml_persist.hpp 与 planner_config_path) 一并删除"而移除 —— 本包不再写任何 yaml 文件。
  #  ⚠ gnss_localization / tractor_calibration 各自那份同名实现与它们的测试不受影响。)

  # YAML 配置契约测试: poly 顶点数量/长度/经纬度, 以及 ridge_width /
  # field_margin / first_row_margin 的取值域。
  ament_add_gtest(test_path_config test/test_path_config.cpp)
  target_link_libraries(test_path_config ${PROJECT_NAME}_lib)

  ament_add_gtest(test_bow_detail test/test_bow_detail.cpp)
  target_link_libraries(test_bow_detail ${PROJECT_NAME}_lib)
  target_include_directories(test_bow_detail PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

  # 全覆盖 (弓字形) 路径规划的行为契约测试: 覆盖率、垄间距、分辨率、凸性校验、简单多边形校验、
  # 逆序/旋转起点输入、余量内缩。几何引擎换成 Clipper2 + Boost.Geometry 之后, 这些是"没有被换坏"的可执行
  # 证据 —— 覆盖率那条尤其重要: 旧版首垄锚在 0.3 m 处, 首末垄之间是 ridge_width 的整数倍间
  # 距, 田块宽度不是整数倍时会在远端留下最多 ridge_width/2 的漏覆盖带。
  # 链 ${PROJECT_NAME}_lib: 算法只编一遍, 与节点里跑的是同一份实现。
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bow_planner test/test_bow_planner.cpp)
  target_link_libraries(test_bow_planner ${PROJECT_NAME}_lib)
  target_compile_definitions(test_bow_planner PRIVATE GLOBAL_PATH_PLANNER_TEST_ACCESS)

  # 扫描方向选取 (minWidthDirection) 的单元测试: 私有步骤经公开接口 plan() ->
  # BowOutput::sweep_dir_deg 观测, 覆盖"最小垂直宽度边方向"的选取、并列/epsilon 判据、
  # 退化短边跳过与顶点顺序无关性。
  ament_add_gtest(test_bow_min_width_direction test/test_bow_min_width_direction.cpp)
  target_link_libraries(test_bow_min_width_direction ${PROJECT_NAME}_lib)

  # 并发契约: **每线程各一个 PlanBowShapedPath 实例**并发调用, 每组结果必须与单线程基准逐位相同,
  # 且 plan() 不改动输入 (见 test/test_bow_parallel.cpp)。同一个实例同一时刻只用于一次规划 (状态在
  # 对象成员里, 见 bow.hpp), 所以这条测试按"每线程一个实例"跑 —— 谁的隐藏静态/全局状态都会在这里串。
  find_package(Threads REQUIRED)
  ament_add_gtest(test_bow_parallel test/test_bow_parallel.cpp)
  target_link_libraries(test_bow_parallel ${PROJECT_NAME}_lib Threads::Threads)

  # 节点级集成测试: 启动真实 GlobalPathPlannerNode, 验证 /path 发布、类型数组长度与坐标系。
  ament_add_gtest(test_global_path_planner_node test/test_global_path_planner_node.cpp)
  target_link_libraries(test_global_path_planner_node ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_global_path_planner_node rclcpp weeder_msgs)
endif()

ament_package()

