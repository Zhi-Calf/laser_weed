# 联调仿真操作指南（RUN_SIM）

在**没有实车、没有 RTK 接收机、没有 CAN 硬件**的条件下，把整条控制链跑通：

```
sim_vehicle ──/gnss_driver/odom──► lqr_control ──/weeder/chassis/cmd──► sim_vehicle
     │                                  ▲
     ├──/gnss_driver/origin──► global_path_planner ──/path──┘
     └──/work_mode──────────►  (两者都订阅)
```

`sim_vehicle` 同时扮演三个角色：GNSS 定位（发 odom/origin/TF）、车辆本体（积分运动学）、模式管理方（发 work_mode）。

---

## 零、一条命令启动（推荐）

不用再手动开 4 个终端，也不用记四段式时序：

```bash
cp -r ~/sim_vehicle ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select sim_vehicle --symlink-install
source install/setup.bash

ros2 launch sim_vehicle weeder_sim.launch.py
```

这一条会起 5 个节点：`sim_vehicle`、`global_path_planner`、`lqr_control`、`sim_orchestrator`、`rviz2`。

其中 **`sim_orchestrator` 自动完成四段式时序**：等 ENU 原点 → 强制切 MANUAL → 调 `set_source` 下发田块 → 等路径发布 → 切 UNMANNED。它每个阶段都等条件满足才推进，所以**启动顺序无关**；任一阶段超时失败会打印该阶段的具体排查建议。

RViz 用的是预置视图 `rviz/weeder_sim.rviz`（Fixed Frame 已设 `odom`，正交俯视，田块边界/弓字形路径/仿真车绿箭头/跟踪目标橙球/掉头关键点全都加好了，**不用手动新建任何显示项**）。

常用参数覆盖：

```bash
# 改垄宽和速度
ros2 launch sim_vehicle weeder_sim.launch.py ridge_width:=3.0 target_v:=0.8

# 不要 RViz（比如无显示器/SSH）
ros2 launch sim_vehicle weeder_sim.launch.py rviz:=false

# 初始横向偏差设大点，收敛过程更明显
ros2 launch sim_vehicle weeder_sim.launch.py align_offset:=2.0
```

⚠️ **`field_margin` 默认 1.5，别改小到 `ridge_width/2` 以下**：地头掉头是半径 = 垄宽/2 的半圆，会从行末端向外鼓，而 `field_margin` 是唯一的内缩量。改小了掉头段会画到田块外面（`ridge_width=2.5` 时掉头半径 1.25m，所以 1.5 刚好够）。编排器启动时会预检并告警。

---

## 下面第一~四节是**手动分终端**的流程，用于排查单个节点。

一键启动正常时不用看，出问题时按它逐段隔离。

---

## ⚠️ 先看懂这个时序陷阱，否则一定卡住

`global_path_planner` 的 `require_manual_for_change` 默认 `true`，意味着：

- **MANUAL 模式**才接受 `set_source` 服务调用（规划田块）
- **UNMANNED 模式**才会让 `lqr_control` 驱动底盘

而 `set_source` 还要求 **ENU 原点已到位**（`origin_timeout_s: 3.0`）。

所以正确顺序是**三段式**，不能跳步：

```
① 起 sim_vehicle（默认 MANUAL）→ 它在发 origin + work_mode
② 起 planner → 它收到 origin，等 set_source
③ 调 set_source（此时仍是 MANUAL，会被接受）→ planner 发布 /path
④ 切 UNMANNED → lqr_control 才开始驱动，车动起来
```

如果在 UNMANNED 下调 `set_source`，服务会返回 `accepted=false, reason="当前非手动模式"`，规划不发生。

---

## 一、安装 sim_vehicle

```bash
cp -r ~/sim_vehicle ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select sim_vehicle --symlink-install
source install/setup.bash
```

依赖只有 `rclpy` + 标准消息 + `weeder_msgs`（已在工作区），无需装系统库。

---

## 二、最小闭环（4 个终端）

### 终端 1：仿真车辆

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 run sim_vehicle sim_vehicle --ros-args \
  -p wheelbase:=1.2 \
  -p align_to_path_start:=true \
  -p align_lateral_offset:=0.8
```

⚠️ **`wheelbase` 必须与 `lqr_control` 的同名参数一致**，否则仿真的"真实车辆"和控制器假设的模型不是同一台车，横向误差会收敛到一个非零稳态值——看起来像控制器没调好，实际是参数不匹配。

启动后应看到：

```
[INFO] 仿真车辆已启动: 轴距=1.200m dt=0.020s 初始位姿=(0.00, 0.00, 0.0°) RTK_sigma=0.020m 模式=MANUAL
```

### 终端 2：全覆盖规划器

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 run global_path_planner global_path_planner_node --ros-args \
  --params-file ~/ros2_ws/src/global_path_planner/config/global_path_planner.yaml
```

应看到收到 ENU 原点的日志。若一直报等原点，检查终端 1 是否活着。

### 终端 3：LQR 控制器

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 run lqr_control lqr_node --ros-args \
  --params-file ~/ros2_ws/src/lqr_control/config/lqr_control.yaml
```

**此时它应该在刷这条警告**（正常，因为还没切 UNMANNED）：

```
[WARN] 未使能：需在 /work_mode 上以 >= 0.33Hz 持续发布 MODE_UNMANNED，输出停车
```

### 终端 4：触发规划 + 切模式

```bash
cd ~/ros2_ws && source install/setup.bash
```

**第 1 步：下发田块边界**（此刻必须还是 MANUAL）

```bash
ros2 service call /global_path_planner/set_source weeder_msgs/srv/SetPathSource \
"{path_source: 0, ridge_width: 2.5, field_margin: 0.5, in_row_resolution: 1.0, poly_count: 4, poly: [34.6198653, 112.4497271, 100.0, 34.6198653, 112.4502729, 100.0, 34.6201347, 112.4502729, 100.0, 34.6201347, 112.4497271, 100.0]}"
```

这是一块 **50m(东西) × 30m(南北)** 的矩形田，中心在仿真原点 `(0,0)`。顶点顺序 lat,lon,alt 各 4 组（SW→SE→NE→NW）。

预期返回：

```
accepted=True
reason='...将在下一个定时周期内生效...'
```

**若 `accepted=False`**，看 `reason`：
- "当前非手动模式" → 你提前切了 UNMANNED，切回 MANUAL 重发
- 垄宽/余量不相容 → `ridge_width` 必须 ≥ 0.01，且 `first_row_margin`(默认0.005) 要落在 `[0.001, ridge_width/2]`

**第 2 步：确认路径生成了**

```bash
ros2 topic echo /global_path_planner/path --once | head -20
```

应看到一串 `frame_id: odom` 的位姿点。`z` 字段是段类型（0=作业段，1=掉头段），**不是高度**，别拿它当地形用。

**第 3 步：切 UNMANNED，车开始跑**

```bash
ros2 topic pub --once /sim/unmanned std_msgs/msg/Bool "{data: true}"
```

⚠️ 用 `--once` 就行——`sim_vehicle` 收到后会**自己以 1Hz 持续转发** `/work_mode`，不需要你一直发。

**第 4 步：看控制器输出**

终端 3 应该从刷警告变成打印跟踪状态。开 DEBUG 能看到详细量：

```bash
# 另开一个终端
ros2 topic echo /weeder/chassis/cmd   # 速度 + 前轮转角（同一控制周期）
ros2 run tf2_ros tf2_echo odom base_link    # 车在动
```

---

## 三、可视化

一键启动已经带好 RViz 了。手动分终端时才需要自己起：

```bash
rviz2 -d ~/ros2_ws/src/sim_vehicle/rviz/weeder_sim.rviz
```

预置视图里已经配好这些（**不用手动新建**）：

| 显示项 | 话题 | 是什么 |
|---|---|---|
| 田块边界 | `/global_path_planner/boundary_points` | 你下发的原始多边形 |
| 内缩边界 | `/global_path_planner/inner_boundary_points` | 扣掉 `field_margin` 后的实际作业范围 |
| 掉头关键点 | `/global_path_planner/key_points` | 每个地头一个（截图左下角那坨橙点） |
| 全覆盖路径 | `/global_path_planner/path` | 弓字形路径本体 |
| **仿真车** | `/sim_vehicle/marker` | **绿色箭头 = 车的位置与朝向** |
| **跟踪目标点** | `/lqr_control/target_point` | **橙色球 = 控制器正在跟的点** |
| RTK轨迹 | `/gnss_driver/odom` | 默认关闭，勾选可看轨迹 |

视图是**正交俯视**（Ortho），Fixed Frame = `odom`，Scale 70 正好一次看全 50×30 的田块。

### ⚠️ 手动加 Odometry 显示时的 QoS 坑

`/gnss_driver/odom` 是 `SensorDataQoS`（**Best Effort**）发布的。RViz 的 Odometry 显示默认用 **Reliable**，而 DDS 规则下 "Best Effort 发布 + Reliable 订阅" 是**不兼容组合，一条消息都收不到**。

所以手动添加时必须把 Reliability Policy 改成 **Best Effort**，否则你会以为"话题名写错了"。预置配置里已经设好了。

（同理，`/global_path_planner/path` 和 `/work_mode` 是 `transient_local` latched 发布的，RViz 的 Path 显示用默认 Volatile 也能收到，因为 planner 每 100ms 重发一次；但如果你把重发关掉就必须改 Durability。）

### 看什么

切 UNMANNED 后，绿色箭头应该：从偏离路径 0.8m 处**收敛贴上去** → 沿弓字形一行行走 → 地头做半圆掉头 → 走完 12 行。橙色球应该**跟着车一起前进**。

**橙球不动 = 目标点没推进**，这是 2026-10-15 那个 bug 的症状（见 README 的指纹去重与偏离距离门修复）。

---

## 四、判断成功的标准

| 检查项 | 成功标志 |
|---|---|
| 规划器出路径 | `/global_path_planner/path` 有几十~上百个点 |
| 控制器不再停车 | 终端 3 从刷"未使能/等待路径"变成打印 `v=... delta=...` |
| 转向收敛 | `/weeder/chassis/cmd` 的 `steer_angle_rad` 从较大值（初始偏差 0.8m）逐渐变小 |
| 车在动 | `tf2_echo odom base_link` 的坐标持续变化 |
| 跟踪上路径 | RViz 里绿箭头贴合灰色路径线前进 |
| 无 NaN | 所有话题值都是有限数，无 `nan`/`inf` |

---

## 五、chassis_node 要不要起？

**最小闭环不需要它。** `sim_vehicle` 直接订阅 `/weeder/chassis/cmd`，已经把控制器输出闭环回车辆运动了。`chassis_node` 的职责是把指令转成 CAN 帧发给真实底盘，仿真里没有底盘，所以它不是必需的。

如果你想顺便验证它的链路，可以起，但**先确认你 VM 里的 `can_driver.cpp` 是哪个版本**：

```bash
grep -n "桩" ~/ros2_ws/src/*/weeder_base/src/can_driver.cpp
```

- **有"桩"字样** → 是桩实现，`open()` 直接返回 true，能起。会打印 `[CanDriver桩] 发送 ID=0x...`，可以顺便看 CAN 帧内容对不对
- **没有** → 是真实 SocketCAN 实现，没有 `can0` 接口会抛"无法打开 CAN 接口"直接退出。需要先建虚拟 CAN：
  ```bash
  sudo ip link add dev vcan0 type vcan
  sudo ip link set up vcan0
  ros2 run weeder_base chassis_node --ros-args -p can_interface:=vcan0
  ```

⚠️ **不管哪个版本，仿真里 `chassis_node` 都会一直报这条**：

```
[WARN] VCU 状态帧超时 X.XXs > 1.00s，已停车
```

这是**预期行为**，不是 bug——没有真实 VCU 回发 `Chassis_State_01`，心跳看门狗必然触发，于是它持续下发零速+零转角。所以**仿真时不要看 `chassis_node` 的输出判断链路好坏**，要看 `/weeder/chassis/cmd`（`lqr_control` 的输出）和 `sim_vehicle` 的运动。

也正因为它会覆盖转向指令为零，**如果同时起了 `chassis_node` 且它连的是真实 CAN，仿真结论不代表实车行为**。仿真阶段建议就不起它。

---

## 六、常见问题

**Q：lqr_control 一直刷"等待路径"**
A：`set_source` 没成功，或是在 UNMANNED 下调的。回终端 4 看 `accepted` 是不是 `True`。

**Q：lqr_control 一直刷"定位精度不足"**
A：`sim_vehicle` 没在跑，或它的 `rtk_sigma` 被设成了 0（门控把 0 视为"质量未知"而拒绝）。默认 0.02 是对的。

**Q：车不动，但也不报错**
A：`/work_mode` 还是 MANUAL。`ros2 topic echo /work_mode --once` 看 `mode` 是不是 1。

**Q：车动了但越来越偏离路径**
A：符号或轴距问题。① 确认两个 `wheelbase` 参数一致；② 确认 `lqr_control` 的 `steer_mode` 与 `chassis_node` 一致（仿真里 `sim_vehicle` 按单阿克曼积分，所以两边都应是 0）；③ 把 `align_lateral_offset` 设成 0 看能否直线跟住——能跟住说明是收敛性问题，跟不住说明是符号问题。

**Q：路径只有一行、没有掉头**
A：田块太窄或 `ridge_width` 太大。30m 南北宽配 2.5m 垄宽应该有 ~12 行。

**Q：`sim_vehicle` 报 `ModuleNotFoundError: weeder_msgs`**
A：`source install/setup.bash` 没执行，或 `weeder_msgs` 没编译。

---

## 七、这套仿真能证明什么、不能证明什么

**能证明**：
- 话题名、消息类型、QoS 在三个包之间**真的匹配**（这是纯静态分析证明不了的）
- LQR 的四轮转向推导**在闭环里能收敛**（单测只验证了开环单拍）
- 弓字形规划 → 路径跟踪的**数据流是通的**
- 各安全门控**按预期触发和解除**

**不能证明**：
- **实车跟踪精度**。仿真是理想模型：无轮胎侧偏、无执行器延迟、无 RTK 延迟、无地面打滑。实车精度取决于这些，全部不在仿真里
- **杆臂标定是否正确**。仿真里 `calib_offset_*` 完全没参与（sim_vehicle 直接输出理想位姿）
- **CAN 通信正确性**。仿真里 `sim_vehicle` 直接吃 ROS 话题，绕过了 CAN 编解码
- **转向模式 1/2/3 的行为**。仿真只实现了单阿克曼（mode 0）

所以：**仿真通过是上实车的必要条件，不是充分条件。** 实车前仍必须做杆臂标定和低速直线验证。
