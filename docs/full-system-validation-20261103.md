# 全仓库编译、测试与短时闭环仿真（2026-11-03）

## 结论与范围

在 MyUbuntu 的 x86_64 ROS 2 Humble 容器中，仓库发现的 **12 个包全部构建成功**。
最终 colcon 汇总为 **282 tests、0 errors、0 failures、27 skipped**。
27 项跳过均来自规划包的 cppcheck：框架因 2.7 版本已知性能问题自动跳过，
本次没有关闭测试来取得通过结果。这里采用 colcon 原始统计，包含 CTest/GTest 等结果封装，
不能理解为 282 个互不重复的业务功能。

短时无显示器仿真验证了全局路径闭环：规划路径发布、模拟定位、切入无人模式、
控制指令发布及模拟车辆运动。这个场景没有启用视觉参考线或激光雷达避停，
它们的独立接口/算法检查由自动测试覆盖；不能据此称为全传感器整车联调。

这次不是板端 Linux 系统镜像编译，也不是完整 ARM64 ROS 工作区编译。

## 源码与环境

- 源码：`d918ff6fe64313b3fa0e5c6d198e2ea084374135`。
- 验证快照：`7a811ea`，加上本次已提交的测试等待时间及 CMake 空白修正。
- MyUbuntu 目录：`/home/zhouboyang/workspace/laser-weed-full-7a811ea-20261103.Vx1f8z`。
- 镜像：`laser-weed-full-check:20261103-7a811ea`。
- 镜像 ID：`sha256:c332a4a278d0158baa974ce517cd626a744b843ddd359132defc55bb57517574`。
- ROS：Humble；C++ 使用容器原生工具链，`RK_NAV_WITH_RKNN=OFF`。
- 基于已有 ROS 验证镜像补齐 Boost、GeographicLib、libpcap、yaml-cpp、Eigen、spdlog_vendor 开发依赖。
- 标定 Python 测试使用独立 Python 3.10 / OpenCV 4.14 环境；Node 使用 20.20.1。
- 编译开启 `BUILD_TESTING=ON`，按整个源码根目录发现包，没有 packages-select 范围限制。

## 编译覆盖

| 包 | 覆盖 |
|---|---|
| weeder_msgs | 整车消息接口生成 |
| weeder_base | 底盘与 CAN 通讯代码 |
| gnss_localization | GNSS 驱动、NTRIP 客户端和相关测试 |
| global_path_planner | 全局路径规划、几何、配置及节点测试 |
| path_tracker | 路径跟踪、视觉参考、限速限转角及节点集成测试 |
| velocity_planner | 点云障碍过滤、避停决策及节点集成测试 |
| sim_vehicle | 模拟车辆运动学、模式编排和节点 |
| rslidar_msg | 雷达消息生成和格式检查 |
| rslidar_sdk | 雷达驱动及随包 rs_driver 核心 |
| row_nav_sdk | 算法库、C 接口和动态 BEV 等测试 |
| crop_row_perception | 正式 ROS 感知适配及接口测试 |
| row_nav_calibration | C++ 采集、网页与 Python 标定测试 |

千寻 SDK 随包只有 aarch64 库，因此 x86 构建按已有 CMake 逻辑不链接 `sdk_client_node`；
仍编译了 `sdk_client_source_check`，检查其源码与 SDK 头文件接口。
这不是千寻客户端的 ARM64 链接或服务验证。

## 首轮发现与处理

1. 测试镜像缺少 Boost 等开发依赖，已在独立派生镜像内补齐。
2. XML 校验无法访问 package.xml 引用的在线 schema。下载 ROS 官方仓库的
   [package_format3.xsd](https://github.com/ros-infrastructure/rep/blob/master/xsd/package_format3.xsd)
   及 package_common.xsd，使用 XML catalog 本地映射后继续执行真实校验；没有跳过 XML 检查。
3. `rslidar_msg` 三份 CMake 文件存在 11 项空白/缩进格式问题，已仅调整排版。
4. 路径跟踪视觉降级测试在 0.12 s 内要求转角精确归零，但代码仍需遵守 180°/s 的速率限制。
   日志确认参考源已经切回全局路径。测试改为按初始转角、速率上限及控制周期计算回正等待时间，
   并断言该时间小于 0.5 s 的视觉自然超时，继续验证状态心跳主动撤销参考线。
   零转角断言精度保持不变，没有放宽控制器限制。

本次生产控制算法没有变化；源码修改是一个集成测试和三份雷达消息 CMake 的排版。
首轮失败日志与最终通过日志都保存在证据包中。

本地 schema：

- package_format3.xsd SHA256：`f096a197ed6d7878984bb2501a55f7f1bd4895d254399fb7857e154bfb644f41`。
- package_common.xsd SHA256：`941ea8645344f3c4b7b9d7e68799898309d65a18225fa9cbef4169d95d1a3211`。

## 无显示器仿真

使用仓库原有 `sim_vehicle/weeder_sim.launch.py`，设置 `rviz:=false`。
测试在独立容器、独立 ROS_DOMAIN_ID 中运行，不连接 CAN 设备和真实车辆。
启动的是模拟车辆、全局规划、路径跟踪和模拟模式编排。

通过条件：有非空路径、持续有效模拟定位、无人模式下有移动控制指令，
且发出移动指令后模拟位置发生至少 0.2 m 的变化。

实际记录：

- 路径点数：3090。
- 有效模拟定位消息：100 条。
- 控制消息：19 条，其中移动指令 9 条。
- 工作模式：UNMANNED。
- 移动指令之后的模拟位移：0.20856 m。
- 达到上述条件耗时：约 2.13 s。
- 停止测试后四个节点均正常退出。

这只是短时闭环连通检查，不是完整田块覆盖、长期稳定性、跟踪精度或避障验收。
仿真沿用项目无雷达场景的 `require_velocity_planner=false`，不能将该关闭配置直接用于实车。

## 证据与复现

本地：`artifacts/full-system-validation-20261103/`。
MyUbuntu：上述验证目录下的 `artifacts/`。

证据包包含：Dockerfile、全量构建/测试脚本、首轮和最终日志、CTest/GTest 结果、
schema 缓存和 catalog、源码修正补丁、仿真脚本/结果/启动日志，以及结果汇总 JSON。
压缩包不纳入 Git，校验和与汇总纳入 Git。

主要执行过程：

```sh
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR=/full-output/ros-logs
export ROS_HOME=/full-output/ros-home
export XML_CATALOG_FILES=/full-output/schemas/catalog.xml
colcon --log-base /full-output/build-log build \
  --base-paths /validation --build-base /full-output/build \
  --install-base /full-output/install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DRK_NAV_BUILD_TESTS=ON -DRK_NAV_WITH_RKNN=OFF -DCMAKE_INSTALL_LIBDIR=lib
```

标定包测试单独配置 Python3_EXECUTABLE 到其隔离环境，其余 ROS 包保留系统 Python。
随后加载安装环境，执行整个工作区的 `colcon test` 和 `colcon test-result`。
具体挂载和命令以证据包的 `validation/run-full.sh` 为准。

## 尚未覆盖

- 整个 ROS 工程的 ARM64 编译与板端运行。
- 千寻 aarch64 SDK 客户端链接、鉴权和真实差分服务。
- 真实 CAN、GNSS/RTK、雷达、相机和 NPU 的硬件联调。
- 真实稻田视觉精度、真实姿态补偿和整车行驶性能。
- 被框架跳过的 cppcheck 静态分析。

之前完成的 ARM64/RKNN 算法 SDK 和相机采集程序交叉编译仍有效，详见
[MyUbuntu SDK/标定验证记录](../row_nav_calibration/docs/validation-mybuntu-20261103.md)。
