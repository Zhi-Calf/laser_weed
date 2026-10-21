# laser_weed · 激光除草机器人无人驾驶系统

基于 ROS 2 Humble 的四轮独立转向激光除草机器人软件栈，包含 RTK 定位、
全覆盖路径规划、LQR 路径跟踪、SocketCAN 底盘通讯、速腾 E1R（RoboSense
E1R）激光雷达 ROS 2 驱动，以及无需实车和 RTK 硬件的全链路仿真。驱动输出的
点云通过独立避停节点接入 `path_tracker` 安全门控，命中 ROI 或感知链路失效时
统一输出停车。

## 系统链路

```text
GNSS 接收机
    │ 串口
    ▼
gnss_localization ──/gnss_driver/odom──► path_tracker
global_path_planner ──/global_path_planner/path──► path_tracker
    (weeder_msgs/msg/Path；YAML poly 首个顶点 = ENU 原点)

path_tracker ──/chassis/cmd──► weeder_base ──CAN──► VCU
                (速度 + 前轮转角)          │
                                           └──/chassis/state

/work_mode：path_tracker 驾驶模式使能
```

激光雷达与避停的数据链路：

```text
速腾 E1R
    │ Ethernet / UDP（默认 6699、7788）
    ▼
rslidar_sdk ──/rslidar_points──► velocity_planner ──/velocity_planner/status──►
                                  (PointCloud2)       (ObstacleStatus)
                                                                  │
                                                                  ▼
                                                     path_tracker 避停安全门控
```

`velocity_planner` 只负责 TF 变换、ROI 判定和故障状态，不发布 `/chassis/cmd`；
`path_tracker` 仍是底盘指令的唯一发布者。

仿真时，`sim_vehicle` 同时替代 `gnss_localization` 和真实底盘，形成闭环：

```text
sim_vehicle ──/gnss_driver/odom──► path_tracker
     ▲                                  │
     └──────/chassis/cmd─────────┘
```

## 包结构

| 包 | 职责 |
|---|---|
| `gnss_localization` | 读取 RTK 接收机 GIAVP 数据，投影到 ENU，发布 odom、origin、status 和 `odom→base_link` TF；提供 NTRIP 与千寻 SDK 两种差分客户端 |
| `global_path_planner` | 根据 YAML 中的田块边界生成弓字形全覆盖路径 |
| `path_tracker` | 四轮转向 LQR 路径跟踪，输出统一底盘指令 `/chassis/cmd` |
| `weeder_base` | CAN 协议编解码、SocketCAN 驱动、50Hz 指令发送、10Hz 状态发布、全部底盘/BMS 状态接收 |
| `weeder_msgs` | 车辆、BMS、GNSS、工作模式、全覆盖路径、点云避停状态等 ROS2 消息 |
| `velocity_planner` | 将 E1R 点云转换到 `base_link`，执行前向矩形 ROI 判定并发布避停状态 |
| `rslidar_sdk` | 速腾 E1R 激光雷达 ROS 2 驱动；解析 UDP 数据包并发布 `PointCloud2` |
| `rslidar_msg` | 速腾激光雷达原始 `RslidarPacket` 消息，供驱动录制/回放 packet 使用 |
| `sim_vehicle` | 无硬件仿真：模拟 RTK、车辆运动学、工作模式，并可一键启动完整链路 |

## 路径消息

`/global_path_planner/path` 使用自定义消息 `weeder_msgs/msg/Path`，
不再发布 `nav_msgs/Path` 和逐点 `geometry_msgs/PoseStamped`：

```text
std_msgs/Header header
float64[] x
float64[] y
uint8[] waypoint_types
```

`x`、`y` 和 `waypoint_types` 按索引一一对应：

- `0` = 作业段
- `1` = 掉头段

规划器以 `transient_local` QoS 按 10Hz 重发该消息；LQR 控制器根据
`x/y` 计算曲率、航向和跟踪目标。当前 `path_tracker` 不读取
`waypoint_types`，该字段保留给后续机具/激光分段控制。

## 激光雷达与点云接口

### 消息类型

`rslidar_msg` 与点云避停使用的消息不是同一种类型：

| 话题 / 类型 | 内容 | 用途 |
|---|---|---|
| `/rslidar_points`<br>`sensor_msgs/msg/PointCloud2` | 已解码的 x、y、z、intensity 点云 | 避停节点的输入 |
| `/velocity_planner/status`<br>`weeder_msgs/msg/ObstacleStatus` | 感知是否有效、是否停车、ROI 点数与最近距离 | `path_tracker` 避停门控 |
| `/rslidar_packets`<br>`rslidar_msg/msg/RslidarPacket` | 雷达原始 MSOP/DIFOP 数据包 | 可选的数据包录制与回放，不直接用于避停 |

当前构建使用 `POINT_TYPE=XYZI`，因此默认点云字段为
`x/y/z/intensity`，`frame_id` 默认为 `rslidar`，点云话题默认为
`/rslidar_points`。默认配置为 `send_point_cloud_ros: true`、
`send_packet_ros: false`，即只发布点云，不发布原始 packet。

`RslidarPacket` 结构如下，其中的 `data` 是厂商数据包字节流：

```text
std_msgs/Header header
uint8 is_difop
uint8 is_frame_begin
uint8[] data
```

避停模块应订阅 `sensor_msgs/msg/PointCloud2`；若要录制原始数据，应同时保存
`rslidar_sdk/config/config.yaml` 和对应的 `RslidarPacket` bag，回放时使用同一份
配置解码。

### E1R 配置

配置文件为 `rslidar_sdk/config/config.yaml`。速腾 E1R 在驱动中的型号标识是
`RSE1`。仓库内配置已按 E1R 设置：

```yaml
lidar:
  - driver:
      lidar_type: RSE1
      msop_port: 6699
      difop_port: 7788
      use_lidar_clock: false       # 未完成雷达与主机时间同步时先使用主机时钟
    ros:
      ros_frame_id: rslidar
      ros_send_point_cloud_topic: /rslidar_points
```

雷达与主机必须网络可达，并允许主机接收 UDP `6699`、`7788` 端口。若现场启用
VLAN、组播或修改了雷达端口，需要同步调整 `use_vlan`、`group_address` 和端口配置。
具体字段说明见 `rslidar_sdk/doc/intro/02_parameter_intro_CN.md`。

### 构建与启动

激光雷达驱动依赖 `libpcap-dev`。在包含本仓库的工作区执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to rslidar_sdk
source install/setup.bash
```

从工作区根目录启动驱动，并显式指定配置文件：

```bash
ros2 run rslidar_sdk rslidar_sdk_node --ros-args \
  -p config_path:=$PWD/src/laser_weed/rslidar_sdk/config/config.yaml
```

驱动默认会把点云发布到 `/rslidar_points`。检查输出：

```bash
ros2 topic info /rslidar_points --verbose
ros2 topic echo /rslidar_points --once --field header
ros2 topic hz /rslidar_points
```

`ros2 topic hz` 会持续统计，确认有稳定频率后按 Ctrl+C 退出。没有实机时只能验证
节点启动、话题声明和配置加载，不能用上述命令替代雷达连通性测试。

### 坐标与时间

驱动不发布外参 TF，点云直接位于 `ros_frame_id`（默认 `rslidar`）坐标系中。
接入避停前必须按实车安装位置配置 `base_link→rslidar` 静态变换，例如：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x <x> --y <y> --z <z> \
  --roll <roll> --pitch <pitch> --yaw <yaw> \
  --frame-id base_link --child-frame-id rslidar
```

示例中的平移单位为 m，旋转单位为 rad，必须替换为实测值。点云默认使用雷达时钟；
如果雷达和主机尚未完成 NTP/PTP 或其他时间同步，应设置
`use_lidar_clock: false`，避免点云时间戳与 `/gnss_driver/odom` 不在同一时间基准。

### 点云避停

`velocity_planner` 将 `/rslidar_points` 转换到 `base_link`，在可配置的前向矩形
ROI 内统计有效点。默认 ROI 为前向 `0~2m`、左右半宽 `0.9m`、高度
`0.1~1.5m`，至少 `3` 个点触发停车。这些数值只是安全起点，必须按车辆包络、
雷达安装位置和实际刹停距离现场标定。

```bash
ros2 launch velocity_planner velocity_planner.launch.py
ros2 topic echo /velocity_planner/status
```

判定规则：

- ROI 内点数达到 `min_obstacle_points`：`stop_required=true`
- 点云、TF 不可用或点云结果超过 `cloud_timeout_s`：
  `sensor_valid=false`、`stop_required=true`
- 障碍消失后继续保持停车 `stop_hold_s`（默认 `0.3s`）
- `path_tracker` 在 `require_velocity_planner=true` 时检查状态新鲜度；默认
  `0.5s` 内没有新状态即按失效停车

避停节点不直接控制车辆。`path_tracker` 触发门控时输出零速、零转角、
`estop=true`，并沿用 10Hz 的 `/chassis/cmd` 控制周期。

## 底盘指令

速度与前轮转角必须来自同一控制周期，因此统一由一条消息发布：

```text
话题：/chassis/cmd
类型：weeder_msgs/msg/ChassisCmd

std_msgs/Header header

float64 speed_m_s
float64 steer_angle_rad    # 左转为正
uint8 gear                 # 0=N, 1=D, 2=R, 3=无效
uint8 steer_mode           # 0~3
bool release_brake
bool estop
bool ctrl_mode
```

`path_tracker` 每拍发布一次该消息，`weeder_base/chassis_node` 接收后转换为
`ADCU_toVCU_01`（`0x0C01D01C`），以 20ms（50Hz）周期发送给 VCU。

手动遥控或测试工具也必须发布 `weeder_msgs/msg/ChassisCmd`，不再使用
`geometry_msgs/Twist` 或单独的转角话题。

## 底盘状态

`weeder_base` 接收协议第二版修中的全部上行帧：

- `State_01~11`：底盘主状态、四轮转速、四轮转角、行走/转向电机电参
- `State_21~22`：BMS SOC、电压、电流（放电为正、充电为负）、容量、循环次数、故障码和电芯电压

统一发布为：

```text
/chassis/state  weeder_msgs/msg/ChassisState
```

每个状态块带有 `*_valid` 标志，用于区分“值为 0”和“尚未收到该帧”。

## 构建

系统依赖（Ubuntu 22.04 / ROS 2 Humble）：

```bash
sudo apt update
sudo apt install -y libeigen3-dev libgeographic-dev libboost-dev \
                    libyaml-cpp-dev libpcap-dev ros-humble-spdlog-vendor \
                    ros-humble-rviz2 ros-humble-tf2-sensor-msgs
```

也可以在包含本仓库的工作区执行：

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

单独构建底盘通讯：

```bash
colcon build --packages-up-to weeder_base
```

单独构建激光雷达驱动（同时构建其消息依赖）：

```bash
colcon build --packages-up-to rslidar_sdk
```

单独构建点云避停及其控制器：

```bash
colcon build --packages-up-to velocity_planner path_tracker
```

## 仿真

```bash
ros2 launch sim_vehicle weeder_sim.launch.py
```

启动内容包括：

- `sim_vehicle`：模拟 RTK、车辆运动学和工作模式
- `global_path_planner`：从 YAML 读取田块并自动规划
- `path_tracker`
- `sim_orchestrator`：确认路径生成后自动切换 UNMANNED
- `rviz2`：预置可视化

无雷达仿真不启动 `velocity_planner`，并通过 launch 显式设置
`require_velocity_planner=false`，保持原有闭环可运行。

常用参数：

```bash
ros2 launch sim_vehicle weeder_sim.launch.py \
  ridge_width:=3.0 target_v:=0.8 rviz:=false
```

查看合并后的控制器输出：

```bash
ros2 topic echo /chassis/cmd
```

详细说明见 `sim_vehicle/RUN_SIM.md`。

## 实车运行

1. 配置 500 kbps CAN 接口，例如：

```bash
sudo ip link set can0 up type can bitrate 500000
```

2. 编辑 `global_path_planner/config/global_path_planner.yaml`，填入现场田块的
   `poly`、`ridge_width` 和 `field_margin`。配置在启动时读取，修改后需要重启规划器。

3. 启动各节点。若 RTK 差分由接收机内置 4G 模块提供，跳过差分客户端；否则
   `ntrip_client` 与 `sdk_client` 二选一：

```bash
ros2 run rslidar_sdk rslidar_sdk_node --ros-args \
  -p config_path:=$PWD/src/laser_weed/rslidar_sdk/config/config.yaml

# 配置 base_link -> rslidar 实车外参后启动点云避停
ros2 run tf2_ros static_transform_publisher \
  --x <x> --y <y> --z <z> \
  --roll <roll> --pitch <pitch> --yaw <yaw> \
  --frame-id base_link --child-frame-id rslidar
ros2 launch velocity_planner velocity_planner.launch.py

ros2 launch gnss_localization localization.launch.py
# 外置差分方案二选一（接收机自带 4G/RTCM 时不要启动）
ros2 launch gnss_localization ntrip_client.launch.py
# 或
ros2 launch gnss_localization sdk_client.launch.py

ros2 launch global_path_planner planner.launch.py
ros2 launch path_tracker path_tracker.launch.py
ros2 launch weeder_base can.launch.py interface:=can0
```

`sdk_client` 使用随包的千寻 SDK 1.29.5 release 库，当前提供的是
**aarch64/RK3588** 版本；x86 开发机默认不构建该可执行文件，可通过
`-DQXWZ_SDK_LIB=<匹配架构的 1.29.x 库>` 指定其他架构。

4. 规划器启动后立即把 `poly` 第一个顶点作为 ENU 原点并生成路径。
   确认 RTK 固定且路径已生成后，再发布 UNMANNED 让控制器接管：

```bash
# 确认 fix_type=4（RTK 固定解）
ros2 topic echo /gnss_driver/status --once

# 确认规划器已发布非空路径（只显示前 5 个 x 坐标）
ros2 topic echo /global_path_planner/path --once \
  --field x --truncate-length 5

# 路径正常后持续发布 UNMANNED
ros2 topic pub -r 1 /work_mode weeder_msgs/msg/WorkMode \
  "{mode: 1, mode_name: unmanned}"
```

`weeder_base` 默认接口是 `vcan0`，实车必须显式设为 `can0` 或实际接口名。

## 实车配置

| 配置 | 位置 | 说明 |
|---|---|---|
| 天线杆臂 `calib_offset_x/y` | `gnss_localization/config/gnss_driver.yaml` | 必须按实车安装位置测量 |
| 串口 | `gnss_localization/config/*.yaml` | 按实际 `/dev/ttyUSB*` 或 `/dev/ttyACM*` 配置 |
| 差分账号 | `ntrip_client.yaml` / `sdk_client.yaml` | 按设备分别配置；SDK 1.29 还需配置 `sdk_openapi_host/port`、`sdk_oss_host/port` 和 NOSR 坐标框架/历元 |
| ENU 原点 | `gnss_localization/config/gnss_driver.yaml` | `origin_lat` / `origin_lon` / `origin_alt`，必须等于 `poly` 第一个顶点 |
| 田块顶点 / 垄宽 / 田边余量 | `global_path_planner/config/global_path_planner.yaml` | `poly` / `ridge_width` / `field_margin`，修改后重启规划器 |
| `wheelbase` | `path_tracker/config/path_tracker.yaml` | 单阿克曼填真实轴距，双阿克曼填 `L/2` |
| `steer_mode` | `path_tracker` | 由路径跟踪控制器随 `/chassis/cmd` 下发 |
| `cmd_topic` | 两侧配置文件 | 默认 `/chassis/cmd` |
| `can_interface` | `weeder_base/config/weeder_base.yaml` | 仿真用 `vcan0`，实车用 `can0` |
| 雷达型号 | `rslidar_sdk/config/config.yaml` | E1R 必须使用 `lidar_type: RSE1`，不能保留 `RSM1` |
| 雷达端口 | 同上 | 默认接收 UDP `6699`（MSOP）和 `7788`（DIFOP） |
| 雷达时钟 | 同上 | `use_lidar_clock: true` 时需先完成雷达与主机时间同步 |
| 雷达外参 | TF | 驱动不发布 TF；避停节点依赖 `base_link→rslidar` 静态变换 |
| 避停 ROI | `velocity_planner/config/velocity_planner.yaml` | 前向范围、半宽、高度和最少点数 |
| 避停超时 | 同上 / `path_tracker` | 点云超时默认 `0.5s`，避停状态超时默认 `0.5s` |

## 安全机制

`path_tracker` 任一安全门控失败都会输出零速、零转角并停车。
下表故障码定义在 `path_tracker/include/path_tracker/fault_codes.hpp`；当前
故障码写入节点日志，尚未通过独立 ROS 话题发布：

| 门控 | 参数 | 故障码 |
|---|---|---|
| 工作模式必须是 UNMANNED | `work_mode_timeout_s` | 10101 |
| RTK 位姿存在、新鲜且为固定解 | `odom_timeout_s` / `rtk_max_sigma` | 10102 |
| 路径存在且新鲜 | `path_timeout_s` | 10103 |
| 转向模式兼容阿克曼模型 | `steer_mode` | 10104 |
| 路径未走完 | — | 10105 |
| 未偏离跟踪目标过远 | `max_track_error_m` | 10106 |
| ROI 内检测到障碍 | `min_obstacle_points` | 10107 |
| 避停状态不可用、超时或感知无效 | `require_velocity_planner` / `velocity_planner_status_timeout_s` | 10108 |

`weeder_base` 另有：

- 未收到指令或 `/chassis/cmd` 超过 `0.3s` 未更新：N 档、遥控模式、不释放抱闸、车速和转角清零
- 协议层最终限幅：车速 `0~1400 mm/s`、转角 `±90.0°`

实车默认启用点云避停门控。雷达断流、TF 缺失、避停节点退出、点云异常或 ROI
命中障碍都会触发失效停车。该门控仍不能替代独立硬件急停。

## 测试

```bash
colcon test
colcon test-result --verbose
```

`weeder_base` 覆盖协议编码、位域、有符号数、范围检查和 BMS 第二版修布局；
`path_tracker` 覆盖四轮转向运动学、前馈、符号、闭环收敛和协议限幅；
`global_path_planner` 覆盖弓字形规划、覆盖率、凸性校验、YAML 配置契约、严格线段相交和节点 `/path` 发布契约；
`gnss_localization` 覆盖 GIAVP 解析、参数写回和 YAML 参数一致性。
`velocity_planner` 覆盖 ROI 边界、NaN 点、障碍点数阈值、点云字段校验和停车保持；
`path_tracker` 额外覆盖避停状态缺失、超时、无效、停车请求和正常放行。
`rslidar_msg` 提供 `RslidarPacket` 接口生成；`rslidar_sdk` 当前没有自动化硬件
测试，需在 E1R 实机上验证 UDP 连通、点云频率、坐标系和时间戳。

## 已知限制

- `global_path_planner` 仅支持凸多边形田块，不支持田块内障碍物挖空
- `global_path_planner` 的 `poly` 必须按边界顺序给出且不能自相交；只允许逆序/旋转起点，
  顶点乱序会被拒绝（`is_simple` 校验，状态码 `self_intersecting`）
- ENU 原点同时配置在 `poly` 第一个顶点和 `gnss_driver.yaml` 的 `origin_*` 中，
  两处不一致时路径与 odom 会产生固定偏移，当前没有运行时一致性校验
- `sdk_client` 随包的千寻 SDK 1.29.5 release 库仅支持 aarch64；x86 开发机默认只编译源码做 API 检查，不生成可执行文件
- 千寻 SDK 1.29 默认服务器域名已关闭，`sdk_client.yaml` 中的 OpenAPI/OSS 域名和端口必须显式配置
- `path_tracker` 仅适用于 `steer_mode=0`（单阿克曼）和 `1`（双阿克曼）
- 点云避停只做前向矩形几何判定，不进行聚类、障碍分类或路径走廊跟踪；
  默认 ROI 必须按实车重新标定
- 无雷达仿真通过 `require_velocity_planner=false` 关闭避停门控，不能用于验证避停
- `rslidar_sdk` 不发布雷达外参 TF，且驱动安装包未单独安装配置文件；
  部署到其他目录时应通过 `config_path` 显式传入配置
- 雷达默认使用自身时钟；接入 E1R 前必须完成时间同步和外参检查
- 当前不支持倒车，底盘档位固定为 D
- 仿真为理想模型，不含轮胎侧偏、执行器延迟、RTK 延迟和地面打滑
- 协议中车速原始值的单位表述仍有歧义（`Value=0.001X` 与 mm/s），实车联调时需用抓包确认；
  BMS 电流已按第二版修实现为 signed、0.1A 分辨率、偏移 1000A

## 来源与许可证

- `gnss_localization`、`global_path_planner` 的部分实现移植自
  组织内部履带拖拉机项目 MegaTrak
- `weeder_base`、`weeder_msgs` 为激光除草机器人项目自有实现
- `rslidar_sdk` v1.5.20（含 `rs_driver` v1.5.20）和 `rslidar_msg` 来自
  [RoboSense-LiDAR](https://github.com/RoboSense-LiDAR)，使用 BSD-3-Clause 许可证
- 弓字形覆盖算法部分概念借鉴 [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover)
  （BSD-3）；使用和发布时需保留其版权与许可证声明

项目整体使用 Apache-2.0 许可证。
