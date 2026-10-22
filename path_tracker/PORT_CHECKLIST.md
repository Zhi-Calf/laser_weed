# 移植配置清单（PORT_CHECKLIST）

> **历史说明：** 当前 CAN 接口已统一为 `/chassis/cmd`
> （`weeder_msgs/ChassisCmd`，同时携带速度与转角）。本文档中
> `/cmd_vel` + `/weeder/steer_cmd` 的双话题方案已废弃，仅保留作移植记录。

整包移植**不改源码，只改配置**。下面每一项都是除草机与履带拖拉机不一样、必须动的地方。
按包分节，改完打勾。**带 🔴 的是不改就会出错的，🟡 是按硬件实际填的，🟢 是可选优化。**

拷贝顺序见 `port_to_weeder.sh`。所有路径按 VM 里实际工作区调整（示例用 `~/ros2_ws/src/`）。

---

## 0. 差分方案先定（决定要不要配 NTRIP / 千寻账号）

除草机 RTK 差分数据从哪来？三选一：

| 方案 | 要跑的节点 | 要配的账号 | 何时用 |
|---|---|---|---|
| A. NTRIP 网络差分 | `ntrip_client_node` | NTRIP 账号密码 | 接收机无 4G，上位机联网取 RTCM |
| B. 千寻 SDK 差分 | `sdk_client_node` | 千寻 `sdk_key/secret/dev_id` | 用千寻位置服务 |
| C. 接收机内置 4G | 都不用跑 | 接收机里配 | 接收机自带网络模块 |

**你选的是"两个都移，稍后再定"**——A、B 的节点都拷进来了，实车确认硬件后：
- 选 A：只配 `ntrip_client.yaml`，只起 `ntrip_client_node`
- 选 B：只配 `sdk_client.yaml`，只起 `sdk_client_node`
- 选 C：两个 client 都不起，只跑 `gnss_driver_node`

⚠️ **账号不能共用**：履带车的 NTRIP/千寻账号绑定的是履带车那台设备，除草机是新设备，
必须重新申请。`ntrip_client.yaml` / `sdk_client.yaml` 里这些字段拷过来是空的，本来就要你填。

---

## 1. gnss_localization（RTK 定位）

文件：`~/ros2_ws/src/gnss_localization/config/gnss_driver.yaml`

| 参数 | 履带车原值 | 除草机要改成 | 级别 |
|---|---|---|---|
| `serial_port` | `/dev/ttyUSB0` | 除草机接收机实际串口（`ls /dev/ttyUSB*` 看） | 🟡 |
| `calib_offset_x` | `0.5` | **除草机天线相对 base_link 的 x 安装偏移（m，前+x）** | 🔴 |
| `calib_offset_y` | `0.0` | 除草机天线 y 安装偏移（m，左+y） | 🔴 |
| `calib_offset_heading` | `0.0` | 天线安装航向偏角（rad），双天线要对齐车身轴向 | 🟡 |
| `log_dir` | `/opt/agri/logs` | 除草机日志目录（如 `~/logs`） | 🟡 |
| `frame_id` | `gnss_link` | 保持 `gnss_link` 即可（除非除草机 TF 树另有约定） | 🟢 |

**为什么杆臂 🔴 必须重标**：`calib_offset_*` 是天线安装位置的物理量，履带车 x=0.5m 是它的天线装在前部 0.5m 处。除草机天线装在别的位置，这个值就不一样。GNSS 用它做杆臂补偿把天线坐标换算到 base_link（`gnss_driver.cpp:651`）——**填错会让 `/gnss_driver/pose` 整体偏移**，下游规划和控制全跟着错，激光打偏。必须按除草机实际安装测量填入，或用 `tractor_calibration` 标定后写回。

文件：`~/ros2_ws/src/gnss_localization/config/ntrip_client.yaml`（方案 A）

| 参数 | 原值 | 改成 | 级别 |
|---|---|---|---|
| `serial_port` | `/dev/ttyUSB1` | 除草机差分串口 | 🟡 |
| `ntrip_user` | 空 | **除草机的 NTRIP 账号** | 🔴 |
| `ntrip_password` | 空 | **除草机的 NTRIP 密码** | 🔴 |
| `ntrip_host/port/mountpoint` | `rtk.ntrip.qxwz.com:8002` / `AUTO` | 按你买的差分服务填 | 🟡 |
| `log_dir` | `/opt/agri/logs` | 除草机日志目录 | 🟡 |

文件：`~/ros2_ws/src/gnss_localization/config/sdk_client.yaml`（方案 B）

| 参数 | 原值 | 改成 | 级别 |
|---|---|---|---|
| `serial_port` | `/dev/ttyUSB1` | 除草机接收机串口 | 🟡 |
| `sdk_key` / `sdk_secret` / `dev_id` / `dev_type` | 空 | **为除草机这台设备申请的千寻账号** | 🔴 |
| `sdk_openapi_host/port` / `sdk_oss_host/port` | 官方 HTTPS 默认值 | 按千寻服务端配置确认 | 🟡 |
| `sdk_coord_frame` / `sdk_coord_epoch` | WGS84 / 2005.0 | 按差分服务坐标系确认 | 🟡 |
| `log_dir` | `/opt/agri/logs` | 除草机日志目录 | 🟡 |

**千寻 SDK 架构**：随包的 SDK 1.29.5 release 库是 aarch64 版本（RK3588）。
x86_64 开发机默认只编译 `sdk_client` 源码做 API 检查，不生成可执行文件；
需要时可用 `-DQXWZ_SDK_LIB=<匹配架构的 1.29.x 库>` 指定库路径。

---

## 2. global_path_planner（全覆盖规划）

文件：`~/ros2_ws/src/global_path_planner/config/global_path_planner.yaml`

| 参数 | 履带车原值 | 除草机要改成 | 级别 |
|---|---|---|---|
| `log_dir` | `~/.ros/logs` | 默认用户目录可写；实车固定目录按部署改 | 🟢 |
| `path_resolution` | `0.2` | 路径插值分辨率(m)，按除草机跟踪精度调；只读参数，改后重启 | 🟡 |
| `first_row_margin` | `0.005` | 首行离内缩边界余量；**必须与 YAML 里的 ridge_width 相容**，落在 [0.001, ridge_width/2] | 🟡 |

`poly`、`ridge_width` 和 `field_margin` 都在 YAML 中配置，
按除草机实际田块和激光作业幅宽填写。`ridge_width` 是全覆盖行距，直接决定除草覆盖率。

**已知限制（重要）**：`global_path_planner` 只接受**凸多边形**田块，**不支持田块内障碍物挖空**
（`bow.cpp:61-66`，非凸直接返回 `kNonConvex`）。除草场景若田块内有不能作业的障碍物（树、石头、
房屋），这个规划器处理不了，需要在上游把田块切成多个凸子块分别规划。履带拖拉机是整片耕地所以没这需求，除草机要确认田块形状。

---

## 3. path_tracker（控制，替代 path_controller）

文件：`~/ros2_ws/src/path_tracker/config/path_tracker.yaml`

| 参数 | 默认值 | 除草机要改成 | 级别 |
|---|---|---|---|
| `wheelbase` | `1.2` | **除草机真实前后轴距(m)** | 🔴 |
| `steer_mode` | `0` | 无有效 `waypoint_types` 时的回退模式 | 🔴 |
| `work_speed_m_s` / `turn_speed_m_s` | `0.6/0.3` | 按路径段固定基础速度 | 🟡 |
| `lqr_target_v` | `0.6` | 无有效分段类型时的回退速度 | 🟡 |
| `log_dir` | `~/.ros/logs` | spdlog 滚动日志目录，实车可按部署改 | 🟡 |
| `lqr_Q_lateral/heading`、`lqr_R_steer` | 50/25/25 | **不能照抄履带调参值**：R 的语义从角速度代价变成了转向角代价，量纲不同，需重标 | 🟡 |

`wheelbase` 填错会直接导致横向跟踪振荡或收敛过慢，是最关键的参数。
`waypoint_types` 有效时节点自动切换模式，并在 TURN 模式下使用 `L/2`
作为等效轴距；模式 2/3 不支持。

---

## 4. CAN 桥（weeder_base/chassis_node.cpp）—— 唯一要改源码的地方

这个包是你除草机自己的（不是 MegaTrak 的）。`patch/chassis_node.cpp` 是**完整替换文件**
（基于你 2026-10-15 提供的版本改写，9 项已有修复全部保留），直接覆盖：

```bash
cp ~/path_tracker/patch/chassis_node.cpp \
   ~/ros2_ws/src/laser_weed_ros2_node/weeder_base/src/chassis_node.cpp
cd ~/ros2_ws
colcon build --packages-select weeder_base --symlink-install
```

改动内容、新增参数、`steer_mode` 一致性要求、台架验证方法见 `patch/PATCH_NOTES.md`。
不改的话 path_tracker 发的转向指令到不了底盘。

一句话：新增 `/weeder/steer_cmd`（Float64，弧度，左正，符号与协议同向不取反）作为权威转向来源；
原 `/cmd_vel` 的 `angular.z` 映射降级为可选回退（供手动遥控），自动作业时不生效。

⚠️ **`steer_mode` 必须与 path_tracker 的同名参数填同一个值**。原代码没设这个字段（恒为 0），
若控制器按双阿克曼（等效轴距 L/2）算转角而底盘按单阿克曼执行，转角会差一倍。

⚠️ **实车自动作业建议关掉回退**：`-p steer_fallback_to_cmd_vel:=false`，
只认 `/weeder/steer_cmd`，超时即回正，避免量纲不严谨的 angular.z 映射影响转向。

---

## 5. 系统依赖预装（构建前必做）

这两个模块依赖若干系统库，干净的 VM 上很可能缺，缺了 `colcon build` 直接失败。
最省事是用 rosdep 自动装（在工作区根目录）：

```bash
cd ~/ros2_ws
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
```

rosdep 装不了或想手动确认，这几个是硬依赖（对应 rosdep 键）：

| 库 | 用途 | Jammy/Humble 包名 | 谁用 |
|---|---|---|---|
| `eigen` | 矩阵/几何 | `libeigen3-dev` | 两个模块 + path_tracker |
| `geographiclib` | 经纬度↔ENU 投影 | ⚠️ **Jammy/Humble 是 `libgeographic-dev`；Noble/Jazzy 是 `libgeographiclib-dev`** | 两个模块 |
| `boost` | 多边形几何适配 | `libboost-dev` | global_path_planner |
| `yaml-cpp` | 参数写回 | `libyaml-cpp-dev` | gnss_localization |
| `spdlog_vendor` | 滚动日志 | ROS 包，`apt install ros-humble-spdlog-vendor` | global_path_planner、path_tracker |

⚠️ **geographiclib 的包名随 Ubuntu/ROS 版本变**（表里那行），装错名字会报找不到。
先确认你的系统：`lsb_release -a` 看 Ubuntu 版本，Humble→Jammy 用 `libgeographic-dev`。

装完验证一下关键库在不在：

```bash
dpkg -l | grep -E 'libeigen3|libgeographic|libboost-dev|libyaml-cpp'
ros2 pkg prefix spdlog_vendor   # 能打印路径说明 spdlog_vendor 就位
```

### 5.1 共享脚本 scripts/ 也要拷（否则 params_in_yaml 测试失败）

`gnss_localization` 和 `global_path_planner` 各有一个 `params_in_yaml` 测试，它检查 C++ 里每个
`declare_parameter()` 是否在 yaml 有对应键。它调用 **MegaTrak 仓库根 `scripts/check_params.py`**
（`CMakeLists.txt` 里写死 `${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/check_params.py`，各包共用一份）。

原始布局里包在 `sensor_node/gnss_localization/`，往上两级正好是仓库根 `scripts/`；但拷进工作区后
包在 `src/gnss_localization/`，`../../scripts/` 变成 `~/ros2_ws/scripts/`——**不存在就会报
`check_params.py: No such file or directory`**。补一句：

```bash
cp -r ~/MegaTrak-main/scripts ~/ros2_ws/     # 注意是工作区根，不是 src/ 里
```

`port_to_weeder.sh` 已内置这一步。这个失败**只影响那条 lint，不影响任何功能测试**——
弓字形算法、GIAVP 解析等核心测试照常通过。

---

## 6. 构建与验证顺序

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# 依赖优先构建（脚本已做，手动的话）
colcon build --packages-up-to path_tracker
source install/setup.bash

# 单元测试：验证四轮转向 LQR 推导 + 路径剖面
colcon test --packages-select path_tracker
colcon test-result --verbose

# 起 GNSS（方案 A 举例）
ros2 launch gnss_localization localization.launch.py
ros2 launch gnss_localization ntrip_client.launch.py

# 看定位对不对（有 fix、pose 不跳）
ros2 topic echo /gnss_driver/pose --once

# 起规划器
ros2 launch global_path_planner planner.launch.py

# 发无人模式（持续 1Hz，否则超时停车）
ros2 topic pub -r 1 /work_mode weeder_msgs/msg/WorkMode "{mode: 1, mode_name: unmanned}"

# 规划器从 YAML 自动读取田块并发布 /global_path_planner/path
ros2 topic echo /global_path_planner/path --once

# 起控制（先不上实车，看它算出的转向角对不对）
ros2 launch path_tracker path_tracker.launch.py
ros2 topic echo /chassis/cmd

# 都对了再起 CAN 桥连实车
```

**⚠️ 没跑通单元测试、没在仿真/台架验证收敛之前，不要直接上实车。** 除草机带激光，
控制算错的代价不是撞墙而是打偏伤人/伤作物。
