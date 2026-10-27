# 稻行视觉接入：观察模式 v0.1

相机/回放 `sensor_msgs/Image` → 本包 → 独立 RowNav SDK →
`/crop_row/observation`（`weeder_msgs/CropRowObservation`）。
本包没有底盘指令发布者，不修改工作模式、GNSS 或全局路径。
可选只读订阅已有 `/gnss_driver/pose`，按图像时间匹配姿态；可只检查静态 BEV 的
有效范围，也可通过 SDK 0.2 逐帧重算动态 BEV。迁入后 SDK/ROS 测试已重跑通过；
默认关闭，合成/契约测试不代表现场补偿已验收。配置与失效规则见
[INTEGRATION.md](INTEGRATION.md)。

### 2026-10-26：已有定位姿态门控验证（动态扩展前）

- 增加独立 `AttitudeGate` 类与有界时间缓存；ROS 适配器只读订阅 `weeder_msgs/Pose`。
  SDK C ABI、定位驱动、TF 广播和车辆控制均未修改。
- macOS 上 `test_attitude_gate.cpp` 与原 `test_core.cpp` 经 ASan/UBSan 测试通过。
  覆盖因果时间匹配、过期、无效姿态、roll/pitch 超限、NaN、源时间故障锁定、
  缓存上限、参数检查以及原有队列/时钟/SDK 契约。
- MyUbuntu 原隔离 Ubuntu 22.04 / ROS2 Humble 容器中，真实 SDK 链接的生产模块
  和测试均重新编译通过；7 个 CTest 测试组通过，ROS 节点 GTest 共 11 个用例通过。
  `colcon test-result` 汇总 18 项、0 errors、0 failures、0 skipped（包含组与内部用例，
  不是 18 次实机推理）。新增 ROS 用例使用 fake SDK，验证缺姿态拒绝/恢复、
  无效/过期/倾斜/帧名错误拒绝，以及未确认标定参考姿态不能启用。
- 首次编译发现测试时间构造方法不兼容 Humble，改用 `std::chrono::nanoseconds`
  后重跑通过。测试工作区仍为 `/tmp/laser-weed-crop-row.8uhjNA`，
  容器 `laser-weed-crop-row-check-env`，不在宿主机全局安装 ROS。
- 未验证真实相机与 GNSS 同步、实际安装轴、现场阈值或动态补偿效果。
  功能默认关闭；只能在现场确认参考姿态及坐标轴后按配置启用。
`path_tracker` 在 `row_pid_enabled=true` 时可选择消费此消息；默认关闭，
启动本包本身仍不会启用视觉转向。
整车侧接入与职责说明见 [INTEGRATION.md](INTEGRATION.md)。
只读观测记录工具 `record_observations.py` 可保存标准 JSONL，检查时间、序号、
会话、无效数值泄漏和回放标记，使用方法见该说明。

## 仓库与依赖

运行算法源码在本仓库 `row_nav_sdk/`，不再要求同级 `../rk` 或预先准备 SDK 安装包。
训练、模型转换、数据及实验资料继续留在 `rk`；运行算法不在两边重复维护。
本仓库 Git 提交统一记录 SDK 和适配层版本，导入基线见 `dependencies/row_nav.lock.json`
和 `row_nav_sdk/migration-origin.json`，不使用 Git submodule。

SDK 需提供 `include/rk_row_nav.h`、动态库和 `lib/cmake/RowNav/RowNavConfig.cmake`。
通过 `find_package(RowNav 0.2 CONFIG)`、`RowNav::rk_row_nav` 链接。
0.2 新增逐帧姿态入口，既有 ABI v1 结构不变；不能混用新头文件与旧动态库。
开发机使用 x86_64/ONNX SDK，RK 板使用 ARM64/RKNN SDK；二者不可混用。
模型必须匹配 SDK 的 Rockchip YOLOv8-seg 解码器，不保证任意 Ultralytics 导出可用。

统一入口从本仓库源码构建 SDK、消息包和感知模块，并运行对应测试：

```bash
source /opt/ros/humble/setup.bash
bash scripts/build_crop_row.sh
source install/crop-row-source/setup.bash
```

此脚本只构建 `row_nav_sdk`、`weeder_msgs` 和 `crop_row_perception`，不会启动任何节点。
SDK 本身不依赖 ROS，可用普通 CMake 独立编译；详见 `../row_nav_sdk/README.md`。
首次使用需安装 OpenCV 开发包，不只是 Python OpenCV 或运行库。
动态库依赖由平台提供；运行前检查 SDK 的 `ldd` 没有 `not found`。
构建会在安装模块的 RPATH 中保留独立 SDK 的库路径，因此该 SDK 前缀需持续存在。
若部署时更换 SDK 安装位置，应重新构建，或明确配置新的库搜索路径，
例如 `export LD_LIBRARY_PATH=/absolute/path/to/row-nav-sdk/lib:$LD_LIBRARY_PATH`。
首次部署 SDK 的 NPU runtime、驱动、模型、精度与板端连续运行仍需实测。

## 配置与启动

复制 `config/observer.yaml` 为现场配置，填写真实 `model_path`、`intrinsic_path`、
`bev_path` 和能追溯标定文件的 `calibration_id`。模板路径为空会明确启动失败。
`backend` 只能是 `onnx` / `rknn`；模型输入尺寸必须与导出一致。
所有参数为启动参数，运行中修改会被明确拒绝；修改模型、标定或时钟配置后需重启。

```bash
ros2 launch crop_row_perception observer.launch.py config:=/absolute/site-observer.yaml
ros2 topic echo /crop_row/observation
```

图像输入约定：

- 原始畸变图像，`bgr8` 或 `rgb8`，尺寸与内参完全一致。SDK 内部执行去畸变，
  输入端不能再次去畸变、裁剪或缩放。缓冲区必须覆盖 `step * height`，允许行 padding。
- `header.frame_id` 必须与 `camera_frame` 一致；这是输入检查，不会执行 TF 转换。
- BEV 必须事先标定到 `vehicle_frame` 的同一个车体参考点。改帧名不能代替外参标定。
- 采集时间为正、严格递增的 ROS 时间；映射到 SDK 同机单调时钟后保留采集帧龄。
  发布端必须使用同源 ROS 时间。跨机器须先保证时钟同步及误差预算，不能直接传设备独立时钟。
  发现大于 100ms 的时钟偏移变化后锁存错误，校正时间后重启节点。
- 不支持 `use_sim_time=true` 或任意历史时间直接输入实时模式；未来、过期、重复/乱序帧被拒绝。
  摄像头驱动必须提供可信采集时间，接收时间不能冒充曝光时间。

## 图像/视频回放

另建回放配置：设置 `image_topic: /crop_row/replay/image`、`replay: true`，
使用与这份影像匹配的真实内参/BEV；不通过把模板的 `calibration_valid` 改成 1 来绕过标定。
回放与真实相机使用独立话题。启动观察节点后，在第二个终端执行：

```bash
ros2 run crop_row_perception replay_images.py --ros-args \
  -p path:=/absolute/clip.mp4 -p fps:=15.0
```

支持单张图片和视频。图片默认仅发一次，`-p repeat:=true` 可重复图片/循环视频。
回放使用当前播放时刻作为输入时间，观察节点会将消息标为 `replay=true`。
这用于图像处理和接口测试，不代表原录像时延，也不能证明闭环控制有效。
文件结束后发布器停止发图并保留终端，观察节点应自动报告过期。
发布器等待图像订阅者发现完成后再读文件，避免单张图在连接建立前丢失。

## 模块、线程与消息

- `FrameClockMapper`：ROS 到同机单调时间映射及跳变检查。
- `LatestFrameQueue`：一个处理中帧、一个待处理帧；新图覆盖待处理旧图并累计丢帧数。
- `RowNavSdk`：RAII 管理句柄，工作线程串行调用同步 C API，图像在返回前始终有效。
- `ObserverNode`：输入契约检查、调用调度、结果发布和 50ms 周期断流检查。
- `replay_images.py`：独立回放输入，不承担板端 GMSL 驱动适配。

图像订阅为 SensorDataQoS / KeepLast(1)，观测发布为 reliable / volatile / KeepLast(1)。
后订阅者不会收到持久缓存的有效观测。每次启动生成会话标识，序号在会话内递增；
状态心跳可重复序号，不代表新观测。客户端要同时检查会话/采集时间、序号和状态。

消息 `header.stamp` 保留原图 ROS 采集时间，`header.frame_id` 为标定车体参考系。
`has_capture_time=false` 表示尚无可用采集时间；此时不能计算采集帧龄。
只有 `status == VALID` 的有限数值可作为后续控制输入。
SDK 调用成功不保证观测有效；非 VALID 时偏差/前视为 NaN、置信度为 0。
无输入、断流、输入错误、SDK 错误、无行、低置信度和过期均明确输出失效状态。
工作线程再次检查发布时帧龄，推理中途过期也不会发布 VALID。

SDK 的正横向误差表示“路径在车左侧”，正航向表示路径向左；
不能直接当作现有 LQR 的“车相对路径误差”。`confidence` 也不是校准后的正确概率。
第一版没有目标作业行身份、TF 转换、运动补偿和自动控制降级策略。

## 测试与验收边界

不依赖 ROS 的契约测试（需要 SDK 头文件，不加载真实模型）：

```bash
c++ -std=c++17 -pthread -I crop_row_perception/include -I ../rk/include \
  crop_row_perception/test/test_core.cpp crop_row_perception/test/fake_sdk.cpp \
  -o /tmp/test_crop_row_core
/tmp/test_crop_row_core
```

完整 ROS 构建后 `colcon test` 执行核心测试和节点通信测试。
通信测试使用独立测试域、测试话题和替身 SDK，检查消息符号/时间、无输入、断流、错误输入、
重复时间戳、SDK 故障、非有限数值和慢推理过期；不加载真实相机或底盘节点。
另用真实 ROS 图像通信和 OpenCV 生成的小样本，测试单图发现等待与视频结束。
生产构建还运行真实可执行程序，检查动态 SDK 能加载，缺配置/缺标定时明确失败退出。
无 SDK 二进制时可仅构建替身测试：

```bash
colcon build --base-paths weeder_msgs crop_row_perception \
  --cmake-args -DBUILD_TESTING=ON -DCROP_ROW_TEST_ONLY=ON \
  -DROW_NAV_HEADER_DIR=/absolute/rk/include
colcon test --packages-select crop_row_perception --return-code-on-test-failure
```

测试模式不生成/安装生产观察节点，不会把替身 SDK 带入发布程序。
真实 SDK 链接、图像推理精度、RKNN/NPU 执行和实车控制是独立验收项。
我们的交付边界是图像到导航观测及联调支持。整车团队负责在跟踪器中消费观测，
处理作业/掉头状态、控制策略、限速和失效停车；不由本模块实现转向或发布底盘指令。

### 2026-10-22 开发验证记录

- macOS：独立 C++ 核心测试及 AddressSanitizer / UndefinedBehaviorSanitizer 通过。
- MyUbuntu 隔离容器：Ubuntu 22.04 / x86_64、ROS 2 Humble、OpenCV 4.5.4。
  使用独立导出的 SDK 固定提交 `1cf533d911a7fbca3525eb627f1e7f24f63b089e`，
  原生编译真实 SDK，生产模块通过 CMake 导入、链接和安装。
- 4 个 CTest 组全部通过：核心契约、8 个 ROS 节点用例、2 个图像/视频回放用例、
  真实程序启动检查。`colcon test-result` 汇总为 12 tests / 0 errors / 0 failures / 0 skipped
  （汇总同时包含 CTest 组及 GTest 子用例，不是 12 次真实图像推理）。
- 启动检查分别运行构建目录和安装目录的真实程序，验证缺配置与缺标定时均以错误退出；
  已修复 ament 链接声明兼容性及独立 SDK 安装后的动态库搜索路径问题。
- 尚未验证真实模型有效帧推理、板端 NPU、现场相机及实车闭环；节点通信测试使用替身 SDK，
  回放样本为测试生成图片/短视频。未修改现有路径跟踪、避障或底盘控制逻辑。

### 补充验证与交付

- 新增只读观测记录工具及 [整车侧接口说明](INTEGRATION.md)，明确我们不承担转向控制实现。
  记录工具检查会话、帧序号、坐标/标定标识、无效数值、接收帧龄和回放标记，
  无效心跳不重复累计为新帧；JSONL 将非有限数值写成标准 null。
- 新增 14 个独立契约用例和真实 ROS 记录工具用例。Humble 的发布者数量查询使用
  `Node.count_publishers`。完整 6 个 CTest 组通过，colcon 汇总 14 tests / 0 errors / 0 failures。
- 真实现有 YOLO11 crop-row ONNX 模型在 3 张 CRDLD 验证图片上完成原生推理、解码和掩码输出。
  预处理加网络前向分别为 64.2164、61.6405、69.4886ms，不含掩码解码、BEV 或完整 SDK/ROS 开销。
  模型/输入/掩码 SHA256 和测试范围见 [model-preflight.json](test/model-preflight.json)。
  这是兼容性检查，不代表已测精度、连续视频、稻田泛化或 RK 板速度，也未更改默认模型选型。
- 完整米制导航观测仍待匹配影像的已验收内参和 BEV；不能给公开数据套用现场相机标定，
  也未通过更改 `calibration_valid` 标记绕过校验。实际 SDK/ROS 有效帧链路仍需这项现场输入。
