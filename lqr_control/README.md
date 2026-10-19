# lqr_control

激光除草机器人（四轮转向底盘）的 LQR 路径跟踪控制器。

从 MegaTrak 履带拖拉机项目 `control_node/path_controller` 移植而来，**状态方程按阿克曼/四轮转向运动学重新推导**。本包依赖项目统一的 `weeder_msgs` 接口包。

替代原有的 Python 版 `stanley_control`。

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
| 除草机 | `δ_ff = L·κ` |

这两个公式是本包单元测试重点锁定的对象（`test_lqr_kinematics.cpp` 的 `LqrFeedforward` 系列）。如果有人误改回差速形式，`FeedforwardIsSpeedIndependent` 会立刻失败——因为 `v·κ` 随速度变化而 `L·κ` 不变。

### 可以直接复用的部分

- **Riccati 求解器** `calRicatti()`：定点迭代解 DARE，与模型无关，逐行沿用
- **`path_profile` 曲率/航向/弧长计算**：纯几何，与模型无关
- **弯道前瞻降速 + 航向误差降速**策略：与模型无关
- **参数命名**：沿用 MegaTrak 的 `lqr_*` 前缀，调参经验可迁移

### 一个重要的语义差异

MegaTrak 的 `lqr_R_w` 是**角速度代价**，本包的 `lqr_R_steer` 是**转向角代价**。量纲不同（rad/s vs rad），所以**同一组数值表现会不一样**，Q/R 必须重新标定，不能直接抄履带项目的调参结果。

---

## 2. CAN 指令接口

`lqr_control` 每个控制周期只发布一条 `/weeder/chassis/cmd`
（`weeder_msgs/ChassisCmd`），同时携带：

| 字段 | 单位 | 约定 |
|---|---|---|
| `speed_m_s` | m/s | 前进速度 |
| `steer_angle_rad` | rad | 前轮转角，左正右负 |

速度与转角放在同一帧，CAN 桥不会把两个不同时刻计算出的值拼在一起。
`weeder_base/chassis_node` 订阅该话题并转换为 `ADCU_toVCU_01`。

`steer_mode` 仍必须与 `weeder_base` 参数一致。双阿克曼模式在控制器中按
`L/2` 的等效轴距计算转角，若底盘按单阿克曼执行，实际转角会差一倍。

### 2.4 符号约定（已追踪源码核对）

CAN 协议 `ADCU_toVCU_01` 的转向角是 `int16_t`、**0.1° 分辨率、左正右负**
（`weeder_base/protocol/can_protocol.hpp` 第二版 Byte3~4）。本控制器输出弧度、左正——
**符号同向，不需要取反**，只做 `弧度 → 度 → ×10` 换算。

字段名已确认：`weeder_base::protocol::AdcuCommand01` 的字段为
`gear`/`steer_mode`/`estop`/`speed_raw`/`steer_angle`/`alive_counter`。
工作区里另有一份 `gear_cmd`/`veh_speed_cmd` 命名的旧头文件，**没被引用、不参与编译**，别照它改。

---

## 3. 话题契约

| 方向 | 话题 | 类型 | 上游 |
|---|---|---|---|
| 订阅 | `/gnss_driver/odom` | `nav_msgs/Odometry` | `gnss_localization`（SensorDataQoS） |
| 订阅 | `/global_path_planner/path` | `nav_msgs/Path` | `global_path_planner`（transient_local） |
| 订阅 | `/work_mode` | `weeder_msgs/WorkMode` | 模式管理方/APP（transient_local） |
| 发布 | `/weeder/chassis/cmd` | `weeder_msgs/ChassisCmd` | → CAN 桥（速度 + 前轮转角） |
| 发布 | `/heartbeat` | `weeder_msgs/Heartbeat` | → `tractor_monitor` 汇总 |
| 发布 | `/lqr_control/target_point` | `visualization_msgs/Marker` | RViz |

**QoS 匹配是硬要求**：`gnss_driver` 用 `SensorDataQoS`（best effort）发 odom，订阅端必须是 best effort，否则一条都收不到；规划器的 path 和 `/work_mode` 都是 `transient_local`（latched），订阅端必须匹配 durability 才能让晚加入的订阅者立刻收到最后一条。本节点已按此配置。

### 工作模式门控（/work_mode）

统一用 MegaTrak 的系统级 `/work_mode` 总线（`weeder_msgs/WorkMode`），不再用私有 arm 话题——这样和 `global_path_planner`、`gnss_driver` 共用同一条使能信号，只有一处需要发布。只有 `MODE_UNMANNED(1)` 才接管底盘；手动(0)/标定(2)/信号超时都停车。

发布端必须**持续**发（≥ 1/`work_mode_timeout_s` Hz），发一次会在超时后停车：

```bash
ros2 topic pub -r 1 /work_mode weeder_msgs/msg/WorkMode "{mode: 1, mode_name: unmanned}"
```

不想启用门控就把 `require_unmanned` 设为 `false`（仅台架调试用，实车务必开着）。

### 心跳（/heartbeat）

本节点通过 `tractor_monitor::HeartbeatPublisher` 以 5Hz 向 `/heartbeat` 发 `Heartbeat`（`node_name="lqr_control"`），由 `tractor_monitor` 汇总成 `/monitor_heartbeat`。故障码沿用 MegaTrak 的 **101xx 段**（与 `path_controller` 同义），所以现有的故障表和 watch table 能直接套用：

| 码 | 含义 | fault_source |
|---|---|---|
| 10101 | 无工作模式/超时/非无人 | app |
| 10102 | 无 RTK 位姿（缺失/超时/非固定解） | gnss_driver |
| 10103 | 无路径（缺失/超时） | global_path_planner |
| 10104 | 转向模式与阿克曼不兼容 | 自身 |
| 10105 | 路径走完 | 自身 |

`fault_source_node` 按责任归属填上游节点名（见 `tractor_monitor/fault_source.hpp`），APP 靠它定位是谁造成的故障。若整包移植了 `tractor_monitor`，**务必同时起 monitor 节点**，否则没人汇总，APP 看不到本节点健康状态。

---

## 4. 安全门控

任一条不满足即输出**停车**（零速 + 零转角），且 CAN 桥自身还有 0.3s 指令超时急停作为第二道防线：

| 门控 | 参数 | 默认 | 失败故障码 |
|---|---|---|---|
| 工作模式 | `work_mode_timeout_s` | 3.0 | 10101 |
| 定位超时 | `odom_timeout_s` | 1.0 | 10102 |
| **RTK 精度** | `rtk_max_sigma` | 0.05 | 10102 |
| 路径超时 | `path_timeout_s` | 5.0 | 10103 |
| 转向模式兼容 | `steer_mode` | 0 | 10104 |
| **偏离目标距离** | `max_track_error_m` | 3.0 | 10106 |
| 启动宽限 | `work_mode_startup_grace_s` | 20.0 | — |

门控按顺序判定：工作模式 → RTK 位姿（存在/新鲜/固定解）→ 路径（存在/新鲜）→ 转向模式兼容 → **偏离目标距离**。RTK 门控取里程计位姿协方差对角项的最大值开方。`gnss_driver` 按 fix_type 填协方差：固定解 `acc_rtk_fixed`（默认 0.02 m），非固定解直接置 `1e3`（`gnss_driver.cpp:661`），所以 `rtk_max_sigma=0.05` 正好卡在两者之间——**除草场景下激光打偏的代价远高于停车**，默认严格卡在固定解。协方差恰好为 0 时视为"质量未知"并拒绝，而不是当作完美。

### ⚠️ 偏离距离门为什么是必需的（2026-10-15 仿真实测发现）

LQR 的状态量只有 `[横向误差, 航向误差]`，**没有沿路径方向的误差项**。这意味着：车如果沿着路径自己的方向笔直冲过目标点，两个误差**都恰好为 0**，控制器会判定"完美跟踪"并继续给全速零转角——**没有任何告警**。

仿真实测就是这样：目标点冻结在起点，车跑出 238 m，`steer_angle_rad = 2.4e-14`，所有误差指标都是 0，车径直冲出田块。`max_track_error_m` 这道门是唯一能发现该情况的手段，所以代码里强制下限 0.1 m，防止误配成 0 把它关掉。

配套的 `snap_window`（默认 25 点）让目标点每拍在窗口内吸附到离车最近的路径点，避免纯弧长累积在里程计打滑时与车越走越远。吸附**从当前索引开始搜索**，索引单调不减，所以永远不会把目标点拉回上一行（弓字形作业不会重跑已覆盖的垄）。

启动宽限**只抑制故障上报**（上电时不刷 FAULT），**从不放开运动**——宽限期内没收到定位/路径照样停车。

---

## 5. 构建与运行

**依赖前置**：本包依赖 `weeder_msgs`（WorkMode/Heartbeat）和 `tractor_monitor`（心跳库），必须先在工作区里有这两个包。

> ⚠️ **前提：`MegaTrak-main` 整棵树不能放在 `~/ros2_ws/src/` 里面。**
> colcon 会递归扫描 `src/` 下所有 `package.xml`；如果 MegaTrak 树在 `src/` 内，
> 它自带的旧消息包、`tractor_monitor` 等包会被扫到，再和下面 `cp` 到 `src/`
> 顶层的副本**重名**，报 `Duplicate package names not supported`。
> 把 MegaTrak-main 放在 `src/` **外面**（如 `~/MegaTrak-main`），下面的 `cp` 才安全。
> 推荐直接用 `port_to_weeder.sh`（已内置此护栏），别手动逐条 cp。

```bash
# 0) 确认 MegaTrak 树在工作区外（示例假定在 ~/MegaTrak-main）
#    若它现在在 ~/ros2_ws/src/MegaTrak-main，先移出来：
#      mv ~/ros2_ws/src/MegaTrak-main ~/MegaTrak-main

# 1) 把依赖包放进工作区（weeder_msgs 已由本项目提供）
cp -r ~/MegaTrak-main/monitoring_node/tractor_monitor ~/ros2_ws/src/
cp -r lqr_control ~/ros2_ws/src/

# 2) 先建依赖包，再建本包（colcon 会按拓扑顺序自动处理，但首次建议显式建依赖）
cd ~/ros2_ws
colcon build --packages-up-to lqr_control
source install/setup.bash

# 3) 单元测试（验证四轮转向推导）
colcon test --packages-select lqr_control
colcon test-result --verbose

# 起节点
ros2 launch lqr_control lqr_control.launch.py
```

纯算法层（`lqr.cpp` + `path_profile.cpp`，编成 `lqr_control_lib`）零 ROS 依赖，但**测试可执行文件依赖 rclcpp/gtest，且 `lqr_node` 依赖 weeder_msgs**，所以整个包必须在完整 ROS2 环境里 `colcon build`，不能脱离 ROS 单独编译。

---

## 6. 调参起点

`config/lqr_control.yaml` 里的默认值是**保守起点，不是可用值**。必须按实车改的：

| 参数 | 说明 |
|---|---|
| `wheelbase` | **最关键**。填错会直接导致横向振荡或收敛过慢。单阿克曼填真实前后轴距；双阿克曼（前后轮反向）填 `L/2` |
| `steer_mode` | 必须与下发给 VCU 的模式一致。模式 2（平移）/3（原地转向）下阿克曼假设不成立，本控制器不适用 |
| `lqr_target_v` | 建议从 0.3 m/s 起调，协议上限 1.4 |
| `lqr_Q_*` / `lqr_R_steer` | 见上方"语义差异"，不能照抄履带项目的值 |

建议调参顺序：先仿真（`sim_demo` 那套思路可复用）验证收敛 → 低速直线（0.3 m/s）验证符号与增益 → 弓字形全覆盖路径验证弯道减速 → 逐步提速。

MegaTrak 的 `docs/FTC_TUNING_PLAYBOOK.md` 里 Stanley 段的 `k_e`/`k_s` 经验对 LQR 不直接适用，但"分段切换 + 段间 blend 避免突变"的思路值得参考——全覆盖路径的 WORK 段和 TURN 段曲率差异很大，本包靠曲率前瞻降速处理。

---

## 7. 整包移植进度（gnss_localization / global_path_planner）

`weeder_msgs` 已作为本项目自有接口包放入工作区（含 12 msg + 3 srv + 1 action，`rosidl_generate_interfaces` 齐全，只依赖标准消息包）。按"整包原样移植"方案，接下来把这几个包整体拷进除草机工作区，只做配置适配、不改源码：

| 包 | 位置 | 移植动作 |
|---|---|---|
| `weeder_msgs` | 消息定义 | 项目自有，`colcon build` 生成接口 |
| `tractor_monitor` | `monitoring_node/` | 原样拷，header-only 心跳库 + monitor 节点 |
| `gnss_localization` | `sensor_node/` | 原样拷，改 config |
| `global_path_planner` | `navigation_node/` | 原样拷，改 config |
| `lqr_control` | 本包 | 替代 path_controller，已统一到 WorkMode |

**GNSS 必改配置**（`gnss_driver.yaml` / `ntrip_client.yaml` / `sdk_client.yaml`）：

- `serial_port`：履带车是 `/dev/ttyUSB0`(driver)、`/dev/ttyUSB1`(差分)，按除草机实际接线改
- `calib_offset_x/y/heading`：**杆臂是履带车的物理安装参数（出厂 x=0.5m），除草机天线位置不同，必须重新标定**——填错会让 `/gnss_driver/odom` 整体偏移，下游全部跟错
- `log_dir`：`/opt/agri/logs` 按除草机部署路径改
- NTRIP/千寻账号：`ntrip_user/password` 或 `sdk_key/secret/dev_id` **绑定设备，履带车的不能共用**，需为除草机重新申请
- 差分方案二选一：ntrip_client（走 NTRIP caster，需账号）或 sdk_client（走千寻 SDK，需为设备申请 key）。用户暂未定，两个都保留，实车确认硬件后再决定跑哪个

**规划器必改配置**（`global_path_planner.yaml`）：

- `log_dir`：`/home/asd/logs` 按部署改
- `path_resolution`（默认 0.2m）：除草机激光作业幅宽和履带车不同，按实际机具宽度调 `ridge_width`（垄宽在 `set_source` 服务请求里给，不是 yaml）
- `first_row_margin`：与请求里的 `ridge_width` 必须相容（落在 [0.001, ridge_width/2]），否则请求被拒
- `require_manual_for_change`：门控是否要求手动模式才能改路径源，实车按需

**可直接复用、代价小的**：`global_path_planner` 的纯算法层 `bow.cpp/bow.hpp` 零 ROS 依赖（Boustrophedon 弓字形全覆盖，输入输出都是 Eigen/std），Clipper2 是 vendored（拷过去就能编），6 个 gtest 只链 `_lib` 可直接跑。GNSS 的 `giavp_parser`、`serial_port`、ENU 投影逻辑也与车型无关。

**已知限制**：`global_path_planner` 只接受**凸多边形**田块、不支持田块内障碍物挖空（`bow.cpp:61-66` 非凸直接拒）。除草场景若田块内有不可作业的障碍物（树、石头），这个规划器处理不了，需要额外做地块切分。

**坐标系必须对齐**：GNSS 广播的 TF 是 `odom → base_link`（语义 ENU，`gnss_driver.cpp:693`），`lqr_control` 的 `frame_id` 默认已是 `odom`、订阅 `/gnss_driver/odom`，三者一致。若除草机的 `base_link` 定义（后轴中心 vs 几何中心）与履带车不同，杆臂标定那步要一并考虑。

---

## 8. 测试覆盖

| 文件 | 覆盖 |
|---|---|
| `test_lqr_kinematics.cpp` | 前馈 = L·κ、前馈与速度无关、左右偏差的转向符号、直线零误差零转角、**闭环收敛仿真**、转角限幅 ±90°、速度限幅、曲率降速、空/单点轨迹停车、零轴距不产生 inf/nan、终点不越界、负位移不回退、**沿路径盲区（3 条）**、**前向吸附（3 条）** |
| `test_path_profile.cpp` | 少于两点拒绝、空指针拒绝、直线零曲率、弧长累加、左/右转弯曲率符号、端点继承内点曲率、重复点不产生 NaN、四分之一圆曲率量级 |

闭环收敛测试（`LqrConvergence.LateralErrorShrinksOverTime`）用自行车模型积分 200 拍，从 0.8 m 横向偏差验证收敛到 5 cm 内——这条能抓住单拍测试抓不到的符号错误（符号写反时单拍看起来合理，但会随时间发散）。

`LqrAlongTrackBlindSpot.RunningPastGoalReportsZeroError` 是**特意写成"记录盲区存在"**的回归测试：它断言车在目标点前方 200 m 处时横向/航向误差**都是 0**、转向指令为 0、但速度仍大于 0，同时 `distToGoal() > 199`。这条测试的价值在于——如果将来有人认为 `e_lat`/`e_head` 已经足够而删掉偏离距离门，`distToGoal()` 断言会失败，从而暴露那次删除的风险。

`LqrSnap.GoalNeverSnapsBackward` 断言目标索引单调不减：车被"传送"回后方时目标点不许跟着倒退，否则弓字形作业会重跑已经除过草的垄。
