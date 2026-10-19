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
    │                                        │
    └──/gnss_driver/origin──► global_path_planner
                                   │
                                   ▼ /global_path_planner/path

lqr_control ──/chassis/cmd──► weeder_base ──CAN──► VCU
                (速度 + 前轮转角)          │
                                           ├──/weeder/chassis/state
                                           └──/weeder/chassis/state_text

/work_mode：系统级驾驶模式使能
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
| `global_path_planner` | 根据田块边界生成弓字形全覆盖路径，提供 `/global_path_planner/set_source` 服务 |
| `lqr_control` | 四轮转向 LQR 路径跟踪，输出统一底盘指令 `/chassis/cmd` |
| `weeder_base` | CAN 协议编解码、SocketCAN 驱动、10Hz 指令发送与状态上报、全部底盘/BMS 状态接收 |
| `weeder_msgs` | 车辆、BMS、GNSS、工作模式、路径规划等 ROS2 消息、服务与 Action |
| `sim_vehicle` | 无硬件仿真：模拟 RTK、车辆运动学、工作模式，并可一键启动完整链路 |

## 底盘指令

速度与前轮转角必须来自同一控制周期，因此统一由一条消息发布：

```text
话题：/chassis/cmd
类型：weeder_msgs/msg/ChassisCmd

float64 speed_m_s
float64 steer_angle_rad    # 左转为正
```

`lqr_control` 每拍发布一次该消息，`weeder_base/chassis_node` 接收后转换为
`ADCU_toVCU_01`（`0x0C01D01C`），以 10Hz（100ms）周期发送给 VCU。

手动遥控或测试工具也必须发布 `weeder_msgs/msg/ChassisCmd`，不再使用
`geometry_msgs/Twist` 或单独的转角话题。

## 底盘状态

`weeder_base` 接收协议第二版中的全部上行帧：

- `State_01~11`：底盘主状态、四轮转速、四轮转角、行走/转向电机电参
- `State_21~22`：BMS SOC、电压、电流、容量、循环次数、故障码和电芯电压

统一发布为：

```text
/weeder/chassis/state       weeder_msgs/msg/ChassisState
/weeder/chassis/state_text  std_msgs/msg/String（兼容旧订阅者）
```

每个状态块带有 `*_valid` 标志，用于区分“值为 0”和“尚未收到该帧”。

## 构建

系统依赖（Ubuntu 22.04 / ROS 2 Humble）：

```bash
sudo apt update
sudo apt install -y libeigen3-dev libgeographic-dev libboost-dev \
                    nlohmann-json3-dev libyaml-cpp-dev ros-humble-spdlog-vendor
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
- `global_path_planner`
- `lqr_control`
- `sim_orchestrator`：自动完成初始化、规划、模式切换和路径下发
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

2. 启动各节点：

```bash
ros2 launch gnss_localization localization.launch.py
ros2 launch global_path_planner planner.launch.py
ros2 launch lqr_control lqr_control.launch.py
ros2 launch weeder_base can.launch.py interface:=can0
```

3. 以不低于 1 Hz 持续发布无人驾驶模式：

```bash
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
| `wheelbase` | `lqr_control/config/lqr_control.yaml` | 单阿克曼填真实轴距，双阿克曼填 `L/2` |
| `steer_mode` | `lqr_control` | 由路径跟踪控制器随 `/chassis/cmd` 下发 |
| `cmd_topic` | 两侧配置文件 | 默认 `/chassis/cmd` |
| `can_interface` | `weeder_base/config/weeder_base.yaml` | 仿真用 `vcan0`，实车用 `can0` |

## 安全机制

`lqr_control` 任一安全门控失败都会输出零速、零转角并上报故障：

| 门控 | 参数 | 故障码 |
|---|---|---|
| 工作模式必须是 UNMANNED | `work_mode_timeout_s` | 10101 |
| RTK 位姿存在、新鲜且为固定解 | `odom_timeout_s` / `rtk_max_sigma` | 10102 |
| 路径存在且新鲜 | `path_timeout_s` | 10103 |
| 转向模式兼容阿克曼模型 | `steer_mode` | 10104 |
| 路径未走完 | — | 10105 |
| 未偏离跟踪目标过远 | `max_track_error_m` | 10106 |

`weeder_base` 另有：

- VCU `State_01` 超时：停车
- VCU `alive_counter` 不递增：判定 VCU 卡死并停车
- VCU 状态超时或 alive 卡死：目标转角强制回正
- 协议层最终限幅：车速 `0~1400 mm/s`、转角 `±90.0°`

## 测试

```bash
colcon test --packages-select weeder_base lqr_control
colcon test-result --verbose
```

`weeder_base` 覆盖协议编码、位域、有符号数、范围检查和 BMS 第二版布局；
`lqr_control` 覆盖四轮转向运动学、前馈、符号、闭环收敛和协议限幅。

## 已知限制

- `global_path_planner` 仅支持凸多边形田块，不支持田块内障碍物挖空
- `lqr_control` 仅适用于 `steer_mode=0`（单阿克曼）和 `1`（双阿克曼）
- 当前不支持倒车，底盘档位固定为 D
- 仿真为理想模型，不含轮胎侧偏、执行器延迟、RTK 延迟和地面打滑
- 协议中车速与 BMS 分辨率存在文档歧义，实车联调时需用抓包确认

## 来源与许可证

- `gnss_localization`、`global_path_planner` 的部分实现移植自
  组织内部履带拖拉机项目 MegaTrak
- `weeder_base`、`weeder_msgs` 为激光除草机器人项目自有实现
- 弓字形覆盖算法部分概念借鉴 [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover)
  （BSD-3）；使用和发布时需保留其版权与许可证声明

项目整体使用 Apache-2.0 许可证。
