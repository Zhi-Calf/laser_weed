# path_tracker

激光除草机器人（四轮转向底盘）的统一 LQR 路径跟踪控制器。

从 MegaTrak 履带拖拉机项目 `control_node/path_controller` 移植而来，**状态方程按阿克曼/四轮转向运动学重新推导**。本包依赖项目统一的 `weeder_msgs` 接口包。

替代早期 Python Stanley 控制器。

---

## 1. 为什么不能照抄 MegaTrak 的 LQR

MegaTrak 是**差速驱动**（左右履带独立调速），除草机是**四轮独立转向**。两者 CAN ID 定义完全相同，但指令语义完全不同：

| | 履带（MegaTrak） | 除草机 |
|---|---|---|
| 控制输入 | 角速度 ω (rad/s) | 前轮转角 δ (rad) |
| CAN 指令字段 | 线速度 + 角速度 | 车速 + 转向角 + steer_mode |
| 转向模式 | 无 | 单阿克曼/双阿克曼/平移/原地 |

MegaTrak 的状态矩阵（`lqr.cpp:127-138`）：

```
A = [[1, v·dt], [0, 1]]      B = [[0], [dt]]        输入 ω
```

除草机必须改成：

```
A = [[1, v·dt], [0, 1]]      B = [[0], [v·dt/L]]    输入 δ
```

**推导**：横向误差与航向误差的连续动力学为

```
ė_y   ≈ v · e_ψ
ė_ψ   = (v/L)·δ − v·κ
```

差速车的 `ė_ψ = ω`，阿克曼车的 `ė_ψ = (v/L)·δ`。离散化后 **A 矩阵恰好相同，只有 B 不同**。

**前馈也随之改变**。稳态过弯时 `ė_ψ = 0`：

| | 前馈 |
|---|---|
| 履带 | `ω_ff = v·κ` |
| 除草机 | `δ_ff = atan(L·κ)` |

这两个公式是本包单元测试重点锁定的对象（`test_lqr_kinematics.cpp` 的
`LqrFeedforward` 系列）。小角度时 `atan(L·κ) ≈ L·κ`，但转弯段不应继续
使用线性近似。

### 可以直接复用的部分

- **Riccati 求解器** `CalRicatti()`：定点迭代解 DARE，与模型无关，逐行沿用
- **`discrete_points_math` 曲率/航向/弧长计算**：与 local_path_planner
  使用同一套 Apollo 离散点算法，并直接填充轨迹点容器
- **WORK/TURN 分段速度**策略：与模型无关
- **参数命名**：沿用 MegaTrak 的 `lqr_*` 前缀，调参经验可迁移

### 一个重要的语义差异

MegaTrak 的 `lqr_R_w` 是**角速度代价**，本包的 `lqr_R_steer` 是**转向角代价**。量纲不同（rad/s vs rad），所以**同一组数值表现会不一样**，Q/R 必须重新标定，不能直接抄履带项目的调参结果。

---

## 2. CAN 指令接口

`path_tracker` 每个控制周期只发布一条 `/chassis/cmd`
（`weeder_msgs/ChassisCmd`），同时携带：

| 字段 | 单位 | 约定 |
|---|---|---|
| `gear` | — | 0=N，1=D，2=R |
| `steer_mode` | — | 0=单阿克曼，1=双阿克曼，2=平移，3=原地 |
| `release_brake` | — | 是否释放抱闸 |
| `estop` | — | 是否远程急停 |
| `ctrl_mode` | — | false=遥控，true=无人驾驶 |
| `speed_m_s` | m/s | 前进速度 |
| `steer_angle_rad` | rad | 前轮转角，左正右负 |

速度与转角放在同一帧，CAN 桥不会把两个不同时刻计算出的值拼在一起。
`weeder_base/chassis_node` 订阅该话题并转换为 `ADCU_toVCU_01`。

`steer_mode` 由本控制器随指令一起发送。WORK 段默认使用单阿克曼，
TURN 段默认使用前后轮反向的对称阿克曼；切换模式时同步把 LQR 的
等效轴距从 `L` 改为 `L/2`。

### 2.4 符号约定（已追踪源码核对）

CAN 协议 `ADCU_toVCU_01` 的转向角是 `int16_t`、**0.1° 分辨率、左正右负**
（`weeder_base/protocol/can_protocol.hpp` 第二版修 Byte3~4）。本控制器输出弧度、左正——
**符号同向，不需要取反**，只做 `弧度 → 度 → ×10` 换算。

字段名已确认：`weeder_base::protocol::AdcuCommand01` 的字段为
`gear`/`steer_mode`/`estop`/`speed_raw`/`steer_angle`/`alive_counter`。

---

## 3. 话题契约

| 方向 | 话题 | 类型 | 上游 |
|---|---|---|---|
| 订阅 | `/gnss_driver/pose` | `weeder_msgs/Pose` | `gnss_localization`（SensorDataQoS） |
| 订阅 | `/global_path_planner/path` | `weeder_msgs/Path` | `global_path_planner`（transient_local） |
| 订阅 | `/work_mode` | `weeder_msgs/WorkMode` | 模式管理方/APP（普通订阅） |
| 订阅 | `/velocity_planner/status` | `weeder_msgs/ObstacleStatus` | `velocity_planner`（reliable） |
| 订阅（可选） | `/crop_row/observation` | `weeder_msgs/CropRowObservation` | `crop_row_perception`（reliable/volatile） |
| 发布 | `/chassis/cmd` | `weeder_msgs/ChassisCmd` | → CAN 桥（速度 + 前轮转角） |
| 发布 | `/path_tracker/target_point` | `visualization_msgs/Marker` | RViz |

当前路径回调读取 `Path.x/y` 和 `waypoint_types`，轨迹点本身不保存速度。
LQR 根据分段类型选择速度：WORK 段默认 `0.6 m/s`，TURN 段及前后过渡区
默认 `0.3 m/s`；速度指令受 `speed_ramp_mps2` 加速度限制。
航向和横向误差只参与转向控制，不改变目标速度。
转角指令受 `max_steer_rate_deg_s` 变化率限制，避免 WORK/TURN 模式切换
时等效转角发生阶跃。

### 视觉稻行与全局路径统一 LQR

`row_vision_enabled=true` 时，普通行内优先消费视觉中心线；默认关闭时只使用全局路径：

```text
普通行内：视觉中心线 -> LQR，单阿克曼
TURN 前后 row_vision_pre_turn_m：全局路径 -> LQR，单/双阿克曼按段切换
TURN 段：连续投影后的全局路径 -> LQR，双阿克曼
视觉失效或出弯后：全局路径 LQR，重新取得连续有效帧后恢复视觉
```

视觉和全局路径共用同一套 `[e_lateral, e_heading]` 反馈及曲率前馈，不再维护独立 PID。
视觉中心线的 `lateral_error_m` 表示路径在左为正，与 LQR 的车辆相对路径符号相反；
`LQR::ComputeLineCommand()` 在单阿克曼后轴控制点处完成符号和参考点转换。

全局路径由 `GlobalPathLocator` 对相邻点组成的折线做连续垂足投影，返回插值后的目标点、
航向、曲率、弧长、线段索引及 TURN 距离。搜索从上一命中线段开始且只向前推进，
不允许自动回到已完成的作业行。

只有 `VALID`、非回放、时间未超过 `crop_row_timeout_s`、置信度不低于
`crop_row_min_confidence`、坐标系匹配且数值有限的观测才参与控制。视觉失效立即降级到
全局路径 LQR，不保持旧视觉值。TURN 前后 `row_vision_pre_turn_m` 范围内暂停视觉恢复计数，
离开该范围后必须重新取得 `row_vision_resume_frames` 个连续有效帧。

参考源和转向模式切换会保留上一转角指令，由 `max_steer_rate_deg_s` 平滑实际输出。
纯二维 LQR 没有积分状态；恒定转向零偏必须通过机械标定或后续 LQI 处理。

规划器会持续重发路径。`path_tracker` 在首次接收有效路径后停止重建轨迹，
但每条重发消息仍用于刷新路径心跳；只有 `/work_mode` 的模式值发生实际变化后，
才会重新接收下一条有效路径。模式重复发布不会触发路径重载。

### spdlog 日志

节点使用与 `global_path_planner` 相同的 spdlog 滚动文件配置，默认写入：

```text
/opt/agri/logs/path_tracker.log
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| `log_dir` | `/opt/agri/logs` | 日志目录，支持 `~` 和 `$VAR` |
| `log_file_size_mb` | `50` | 单个文件大小上限 |
| `log_file_count` | `10` | 保留文件数量 |
| `log_enable_console` | `true` | 同时输出到控制台 |
| `log_level` | `info` | `trace/debug/info/warn/error/critical/off` |

目录不可写时会回退到 `$ROS_HOME/logs` 或 `~/.ros/logs`；
若仍不可写则仅保留控制台日志。

**QoS 配置说明**：`gnss_driver` 用 `SensorDataQoS`（best effort）发 pose，订阅端必须是 best effort，否则一条都收不到；规划器的 path 是 `transient_local`（latched），订阅端必须匹配 durability 才能让晚加入的订阅者立刻收到最后一条。`/work_mode` 使用普通订阅 QoS，不要求锁存，模式发布端需持续发送模式。

### 工作模式门控（/work_mode）

统一用 MegaTrak 的系统级 `/work_mode` 总线（`weeder_msgs/WorkMode`），不再用私有 arm 话题——由模式管理方或 APP 统一发布。只有 `MODE_UNMANNED(1)` 才接管底盘；手动(0)/标定(2)/信号超时都停车。

发布端必须**持续**发（≥ 1/`work_mode_timeout_s` Hz），发一次会在超时后停车：

```bash
ros2 topic pub -r 1 /work_mode weeder_msgs/msg/WorkMode "{mode: 1, mode_name: unmanned}"
```

安全门控失败时只在节点日志中告警并输出停车，不再发布心跳消息。

---

## 4. 安全门控

任一条不满足即输出**停车**（零速 + 零转角），且 CAN 桥自身还有 0.3s 指令超时急停作为第二道防线：

| 门控 | 参数 | 默认 |
|---|---|---|
| 工作模式 | `work_mode_timeout_s` | 1.0 |
| 定位超时 | `pose_timeout_s` | 1.0 |
| **RTK 位姿有效** | `Pose.valid` | true |
| 路径超时 | `path_timeout_s` | 1.0 |
| **横向误差超限** | `max_lateral_error_m` | 0.3 |
| **偏离目标距离** | `max_track_error_m` | 3.0 |
| ROI 内检测到障碍 | `min_obstacle_points` | 3 |
| **避停状态不可用/超时/感知无效** | `require_velocity_planner` / `velocity_planner_status_timeout_s` | true / 1.0 |

门控按顺序判定：工作模式 → RTK 位姿（存在/新鲜/`valid=true`）→ 路径（存在/新鲜）→ **点云避停** → **横向误差超限** → **偏离目标距离**。`gnss_driver` 仅在 RTK 固定解时把 `Pose.valid` 置为 true，浮点/RTD/单点解都会输出停车。

### 点云避停门控

`require_velocity_planner` 默认 `true`。启用后，以下任一情况都输出零速、零转角和
`estop=true`：

- 从未收到 `/velocity_planner/status`
- 最近状态超过 `velocity_planner_status_timeout_s`（默认 1.0s）
- `sensor_valid=false`
- `stop_required=true`

无雷达仿真必须显式设置 `require_velocity_planner=false` 或同时启动
`velocity_planner`。实车启动顺序应先启动雷达驱动和 `velocity_planner`，
再启动 `path_tracker`。

### ⚠️ 偏离距离门为什么是必需的（2026-10-15 仿真实测发现）

LQR 的状态量只有 `[横向误差, 航向误差]`，**没有沿路径方向的误差项**。这意味着：车如果沿着路径自己的方向笔直冲过目标点，两个误差**都恰好为 0**，控制器会判定"完美跟踪"并继续给全速零转角——**没有任何告警**。

仿真实测就是这样：目标点冻结在起点，车跑出 238 m，`steer_angle_rad = 2.4e-14`，所有误差指标都是 0，车径直冲出田块。`max_track_error_m` 这道门是唯一能发现该情况的手段，因此必须配置为正数。

配套的 `snap_forward_points`（默认 20 段）让连续投影每拍从上一命中线段开始，
在向前 `snap_forward_points` 个线段的范围内寻找最近垂足，避免纯弧长累积
在里程计打滑时与车越走越远。目标进度单调不减，所以永远不会把投影点拉回上一行
（弓字形作业不会重跑已覆盖的垄）。

---

## 5. 构建与运行

**依赖前置**：本包依赖 `weeder_msgs`（WorkMode、ChassisCmd、Path、
ObstacleStatus）。

> ⚠️ **前提：`MegaTrak-main` 整棵树不能放在 `~/ros2_ws/src/` 里面。**
> colcon 会递归扫描 `src/` 下所有 `package.xml`；如果 MegaTrak 树在 `src/` 内，
> 它自带的旧消息包等会被扫到，再和下面 `cp` 到 `src/`
> 顶层的副本**重名**，报 `Duplicate package names not supported`。
> 把 MegaTrak-main 放在 `src/` **外面**（如 `~/MegaTrak-main`），下面的 `cp` 才安全。
> 推荐直接用 `port_to_weeder.sh`（已内置此护栏），别手动逐条 cp。

```bash
# 0) 确认 MegaTrak 树在工作区外（示例假定在 ~/MegaTrak-main）
#    若它现在在 ~/ros2_ws/src/MegaTrak-main，先移出来：
#      mv ~/ros2_ws/src/MegaTrak-main ~/MegaTrak-main

# 1) 把依赖包放进工作区（weeder_msgs 已由本项目提供）
cp -r path_tracker ~/ros2_ws/src/

# 2) 先建依赖包，再建本包（colcon 会按拓扑顺序自动处理，但首次建议显式建依赖）
cd ~/ros2_ws
colcon build --packages-up-to path_tracker
source install/setup.bash

# 3) 单元测试（验证四轮转向推导）
colcon test --packages-select path_tracker
colcon test-result --verbose

# 起节点
ros2 launch path_tracker path_tracker.launch.py
```

纯算法层（`discrete_points_math.cpp` + `global_path_locator.cpp` + `lqr.cpp`，编成
`path_tracker_lib`）零 ROS 依赖，但**测试可执行文件依赖 rclcpp/gtest，
且 `path_tracker_node` 依赖 weeder_msgs**，所以整个包必须在完整 ROS2
环境里 `colcon build`，不能脱离 ROS 单独编译。

---

## 6. 调参起点

`config/path_tracker.yaml` 里的默认值是**保守起点，不是可用值**。必须按实车改的：

YAML 参数被视为可信输入，节点运行时不再对参数范围做自动修正；部署前必须自行检查单位、符号和取值范围。

| 参数 | 说明 |
|---|---|
| `wheelbase` | **最关键**。填真实前后轴距；控制器按 WORK/TURN 自动使用 `L` 或 `L/2` |
| `pose_at_vehicle_center` | 默认 `true`；单阿克曼自动换算到后轴，双阿克曼使用中心 |
| `max_steer_rate_deg_s` | 转角指令变化率上限，默认 `90°/s`，需按转向执行器能力标定 |
| `row_vision_enabled` | 是否启用视觉稻行 LQR，默认 `false` |
| `row_vision_pre_turn_m` | TURN 前后退出或暂停视觉的弧长距离，默认 `3.0 m` |
| `row_vision_resume_frames` | 视觉失效或出弯后恢复所需连续有效帧数，默认 `3` |
| `crop_row_min_confidence` | 视觉观测参与 LQR 的最低置信度 |
| `work_speed_m_s` / `turn_speed_m_s` | WORK/TURN 基础速度，默认 0.6/0.3 m/s |
| `turn_speed_preview_m` / `turn_speed_hold_m` | 转弯前预览和转弯后低速保持距离 |
| `lqr_Q_*` / `lqr_R_steer` | 见上方"语义差异"，不能照抄履带项目的值 |

建议调参顺序：先仿真（`sim_demo` 那套思路可复用）验证收敛 → 低速直线（0.3 m/s）验证符号与增益 → 弓字形全覆盖路径验证 WORK/TURN 速度切换 → 逐步提速。

MegaTrak 的 `docs/FTC_TUNING_PLAYBOOK.md` 里 Stanley 段的 `k_e`/`k_s`
经验对 LQR 不直接适用。全覆盖路径默认由 WORK/TURN 分段决定速度和模式；
曲率只用于转角前馈，不参与速度判断；航向和横向误差也不参与速度判断。

---

## 7. 整包移植进度（gnss_localization / global_path_planner）

`weeder_msgs` 已作为本项目自有接口包放入工作区（当前保留 6 个实际使用的 msg，
只依赖标准消息库）。按"整包原样移植"方案，接下来把这几个包整体拷进除草机工作区，只做配置适配、不改源码：

| 包 | 位置 | 移植动作 |
|---|---|---|
| `weeder_msgs` | 消息定义 | 项目自有，`colcon build` 生成接口 |
| `gnss_localization` | `sensor_node/` | 原样拷，改 config |
| `global_path_planner` | `navigation_node/` | 原样拷，改 config |
| `path_tracker` | 本包 | 替代 path_controller，已统一到 WorkMode |

**GNSS 必改配置**（`gnss_driver.yaml` / `ntrip_client.yaml` / `sdk_client.yaml`）：

- `serial_port`：履带车是 `/dev/ttyUSB0`(driver)、`/dev/ttyUSB1`(差分)，按除草机实际接线改
- `calib_offset_x/y/heading`：**杆臂是履带车的物理安装参数（出厂 x=0.5m），除草机天线位置不同，必须重新标定**——填错会让 `/gnss_driver/pose` 整体偏移，下游全部跟错
- `log_dir`：`/opt/agri/logs` 按除草机部署路径改
- NTRIP/千寻账号：`ntrip_user/password` 或 `sdk_key/secret/dev_id` **绑定设备，履带车的不能共用**，需为除草机重新申请；千寻 SDK 1.29 还需按服务端要求配置 `sdk_openapi_*` / `sdk_oss_*` 和 NOSR 坐标框架/历元
- 差分方案二选一：ntrip_client（走 NTRIP caster，需账号）或 sdk_client（走千寻 SDK，需为设备申请 key）。用户暂未定，两个都保留，实车确认硬件后再决定跑哪个

**规划器必改配置**（`global_path_planner.yaml`）：

- `log_dir`：默认 `/opt/agri/logs`；不可写时自动回退到 ROS 日志目录
- `path_resolution`（默认 0.2m）：`/path` 插值分辨率
- `poly` / `ridge_width` / `field_margin`：按除草机实际田块和机具宽度填写；
  `field_margin >= ridge_width/2` 是为了让机具半幅不越过田边（掉头弧本身切在内缩边界上）
- `first_row_margin`：与 `ridge_width` 必须相容（落在 [0.001, ridge_width/2]），否则节点启动失败

**可直接复用、代价小的**：`global_path_planner` 的纯算法层 `bow.cpp/bow.hpp` 零 ROS 依赖（Boustrophedon 弓字形全覆盖，输入输出都是 Eigen/std），Clipper2 是 vendored（拷过去就能编），6 个 gtest 只链 `_lib` 可直接跑。GNSS 的 `giavp_parser`、`serial_port`、ENU 投影逻辑也与车型无关。

**已知限制**：`global_path_planner` 只接受**凸多边形**田块、不支持田块内障碍物挖空（`bow.cpp:61-66` 非凸直接拒）。除草场景若田块内有不可作业的障碍物（树、石头），这个规划器处理不了，需要额外做地块切分。

**坐标系必须对齐**：GNSS 广播的 TF 是 `odom → base_link`（语义 ENU），`Pose` 的 `x/y/yaw` 也位于同一 `odom` 坐标系，`path_tracker` 直接订阅 `/gnss_driver/pose`。若除草机的 `base_link` 定义（后轴中心 vs 几何中心）与履带车不同，杆臂标定那步要一并考虑。

---

## 8. 测试覆盖

| 文件 | 覆盖 |
|---|---|
| `test_lqr_kinematics.cpp` | 精确阿克曼前馈、运行时等效轴距、WORK/TURN 速度区、跟踪误差和曲率不参与降速、直线与对称阿克曼圆弧闭环、左右偏差符号、转角限幅、小位移弧长累计、沿路径盲区和前向吸附 |
| `test_discrete_points_math.cpp` | 少于三点拒绝、空指针拒绝、直线零曲率、弧长累加、左/右转弯曲率符号、端点单边差分、相邻重合点与航向退化检查、四分之一圆曲率量级 |
| `test_global_path_locator.cpp` | 线段内连续投影、终点夹取、单调前进、有限搜索窗口、跨界航向插值、TURN 距离和退化路径拒绝 |

闭环测试同时覆盖直线横向误差收敛和 `L_eff=L/2` 的对称阿克曼圆弧跟踪。
后者能同时检查精确前馈、等效轴距和路径目标推进是否一致。

`LqrAlongTrackBlindSpot.RunningPastGoalReportsZeroError` 是**特意写成"记录盲区存在"**的回归测试：它断言车在目标点前方 200 m 处时横向/航向误差**都是 0**、转向指令为 0、但速度仍大于 0，同时 `distance_to_goal > 199`。这条测试的价值在于——如果将来有人认为 `e_lat`/`e_head` 已经足够而删掉偏离距离门，距离断言会失败，从而暴露那次删除的风险。

`GlobalPathLocator.NeverProjectsBackward` 断言连续投影进度单调不减：车被"传送"回后方时目标点不许跟着倒退，否则弓字形作业会重跑已经除过草的垄。
