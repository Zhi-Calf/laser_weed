# laser_weed · 激光除草机器人无人驾驶系统

基于 ROS 2 Humble 的四轮独立转向激光除草机器人软件栈，包含 RTK 定位、
全覆盖路径规划、LQR 路径跟踪、SocketCAN 底盘通讯，以及无需实车和 RTK 硬件的
全链路仿真。

## 系统链路

```text
GNSS 接收机
    │ 串口
    ▼
gnss_localization ──/gnss_driver/odom──► lqr_control
global_path_planner ──/global_path_planner/path──► lqr_control
    (weeder_msgs/msg/Path；YAML poly 首个顶点 = ENU 原点)

lqr_control ──/chassis/cmd──► weeder_base ──CAN──► VCU
                (速度 + 前轮转角)          │
                                           └──/chassis/state

/work_mode：lqr_control 驾驶模式使能
```

仿真时，`sim_vehicle` 同时替代 `gnss_localization` 和真实底盘，形成闭环：

```text
sim_vehicle ──/gnss_driver/odom──► lqr_control
     ▲                                  │
     └──────/chassis/cmd─────────┘
```

## 包结构

| 包 | 职责 |
|---|---|
| `gnss_localization` | 读取 RTK 接收机 GIAVP 数据，投影到 ENU，发布 odom、origin、status 和 `odom→base_link` TF；提供 NTRIP 与千寻 SDK 两种差分客户端 |
| `global_path_planner` | 根据 YAML 中的田块边界生成弓字形全覆盖路径 |
| `lqr_control` | 四轮转向 LQR 路径跟踪，输出统一底盘指令 `/chassis/cmd` |
| `weeder_base` | CAN 协议编解码、SocketCAN 驱动、50Hz 指令发送、10Hz 状态发布、全部底盘/BMS 状态接收 |
| `weeder_msgs` | 车辆、BMS、GNSS、工作模式、全覆盖路径等 ROS2 消息 |
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
`x/y` 计算曲率、航向和跟踪目标。当前 `lqr_control` 不读取
`waypoint_types`，该字段保留给后续机具/激光分段控制。

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

`lqr_control` 每拍发布一次该消息，`weeder_base/chassis_node` 接收后转换为
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
                    libyaml-cpp-dev ros-humble-spdlog-vendor
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

## 仿真

```bash
ros2 launch sim_vehicle weeder_sim.launch.py
```

启动内容包括：

- `sim_vehicle`：模拟 RTK、车辆运动学和工作模式
- `global_path_planner`：从 YAML 读取田块并自动规划
- `lqr_control`
- `sim_orchestrator`：确认路径生成后自动切换 UNMANNED
- `rviz2`：预置可视化

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
ros2 launch gnss_localization localization.launch.py
# 外置差分方案二选一（接收机自带 4G/RTCM 时不要启动）
ros2 launch gnss_localization ntrip_client.launch.py
# 或
ros2 launch gnss_localization sdk_client.launch.py

ros2 launch global_path_planner planner.launch.py
ros2 launch lqr_control lqr_control.launch.py
ros2 launch weeder_base can.launch.py interface:=can0
```

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
| 差分账号 | `ntrip_client.yaml` / `sdk_client.yaml` | 按设备分别配置 |
| ENU 原点 | `gnss_localization/config/gnss_driver.yaml` | `origin_lat` / `origin_lon` / `origin_alt`，必须等于 `poly` 第一个顶点 |
| 田块顶点 / 垄宽 / 田边余量 | `global_path_planner/config/global_path_planner.yaml` | `poly` / `ridge_width` / `field_margin`，修改后重启规划器 |
| `wheelbase` | `lqr_control/config/lqr_control.yaml` | 单阿克曼填真实轴距，双阿克曼填 `L/2` |
| `steer_mode` | `lqr_control` | 由路径跟踪控制器随 `/chassis/cmd` 下发 |
| `cmd_topic` | 两侧配置文件 | 默认 `/chassis/cmd` |
| `can_interface` | `weeder_base/config/weeder_base.yaml` | 仿真用 `vcan0`，实车用 `can0` |

## 安全机制

`lqr_control` 任一安全门控失败都会输出零速、零转角并停车。
下表故障码定义在 `lqr_control/include/lqr_control/fault_codes.hpp`；当前节点
尚未通过 ROS 话题发布故障，也未把故障码写入日志，下表是预留映射：

| 门控 | 参数 | 故障码（预留） |
|---|---|---|
| 工作模式必须是 UNMANNED | `work_mode_timeout_s` | 10101 |
| RTK 位姿存在、新鲜且为固定解 | `odom_timeout_s` / `rtk_max_sigma` | 10102 |
| 路径存在且新鲜 | `path_timeout_s` | 10103 |
| 转向模式兼容阿克曼模型 | `steer_mode` | 10104 |
| 路径未走完 | — | 10105 |
| 未偏离跟踪目标过远 | `max_track_error_m` | 10106 |

`weeder_base` 另有：

- 未收到指令或 `/chassis/cmd` 超过 `0.3s` 未更新：N 档、遥控模式、不释放抱闸、车速和转角清零
- 协议层最终限幅：车速 `0~1400 mm/s`、转角 `±90.0°`

## 测试

```bash
colcon test
colcon test-result --verbose
```

`weeder_base` 覆盖协议编码、位域、有符号数、范围检查和 BMS 第二版修布局；
`lqr_control` 覆盖四轮转向运动学、前馈、符号、闭环收敛和协议限幅；
`global_path_planner` 覆盖弓字形规划、覆盖率、凸性校验、YAML 配置契约和严格线段相交；
`gnss_localization` 覆盖 GIAVP 解析、参数写回和 YAML 参数一致性。

## 已知限制

- `global_path_planner` 仅支持凸多边形田块，不支持田块内障碍物挖空
- `global_path_planner` 的 `poly` 必须按边界顺序给出且不能自相交；只允许逆序/旋转起点，
  顶点乱序会被拒绝（`is_simple` 校验，状态码 `self_intersecting`）
- ENU 原点同时配置在 `poly` 第一个顶点和 `gnss_driver.yaml` 的 `origin_*` 中，
  两处不一致时路径与 odom 会产生固定偏移，当前没有运行时一致性校验
- `lqr_control` 仅适用于 `steer_mode=0`（单阿克曼）和 `1`（双阿克曼）
- 当前不支持倒车，底盘档位固定为 D
- 仿真为理想模型，不含轮胎侧偏、执行器延迟、RTK 延迟和地面打滑
- 协议中车速原始值的单位表述仍有歧义（`Value=0.001X` 与 mm/s），实车联调时需用抓包确认；
  BMS 电流已按第二版修实现为 signed、0.1A 分辨率、偏移 1000A

## 来源与许可证

- `gnss_localization`、`global_path_planner` 的部分实现移植自
  组织内部履带拖拉机项目 MegaTrak
- `weeder_base`、`weeder_msgs` 为激光除草机器人项目自有实现
- 弓字形覆盖算法部分概念借鉴 [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover)
  （BSD-3）；使用和发布时需保留其版权与许可证声明

项目整体使用 Apache-2.0 许可证。
