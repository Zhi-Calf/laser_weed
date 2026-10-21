# 联调仿真操作指南（RUN_SIM）

在没有实车、RTK 接收机和 CAN 硬件的条件下，把整条控制链跑通：

```text
sim_vehicle ──/gnss_driver/odom──► path_tracker ──/chassis/cmd──► sim_vehicle
     └──/work_mode──────────────────► path_tracker

global_path_planner（YAML poly 首个顶点为原点）──/path──► path_tracker
```

`sim_vehicle` 同时扮演三个角色：GNSS 定位、车辆运动学和工作模式管理。
`global_path_planner` 启动时直接从 YAML 读取田块和几何参数，不再需要
`set_source` 服务。

## 一、一条命令启动

```bash
cd /path/to/workspace
colcon build --symlink-install
source install/setup.bash
ros2 launch sim_vehicle weeder_sim.launch.py
```

启动的节点：

| 节点 | 作用 |
|---|---|
| `sim_vehicle` | 发布 RTK odom/origin、车辆运动学、`/work_mode` |
| `global_path_planner` | 读取 YAML 田块，使用 `poly` 首个顶点作为原点并立即规划 |
| `path_tracker` | 跟踪 `/global_path_planner/path` |
| `sim_orchestrator` | 确认路径生成后切 UNMANNED |
| `rviz2` | 预置田块、路径、车辆和目标点可视化 |

仿真不启动激光雷达和 `velocity_planner`；`weeder_sim.launch.py` 会显式设置
`require_velocity_planner=false`。实车默认启用避停，不能直接沿用仿真的关闭配置。

常用参数：

```bash
# 改垄宽和目标速度
ros2 launch sim_vehicle weeder_sim.launch.py ridge_width:=3.0 target_v:=0.8

# 无显示器/SSH
ros2 launch sim_vehicle weeder_sim.launch.py rviz:=false

# 初始横向偏差更大，便于观察收敛
ros2 launch sim_vehicle weeder_sim.launch.py align_offset:=2.0
```

## 二、田块与路径配置

编辑 `global_path_planner/config/global_path_planner.yaml`：

```yaml
global_path_planner:
  ros__parameters:
    poly: [34.6198653, 112.4497271, 100.0,
           34.6198653, 112.4502729, 100.0,
           34.6201347, 112.4502729, 100.0,
           34.6201347, 112.4497271, 100.0]
    ridge_width: 2.5
    field_margin: 1.5
```

- `poly` 是扁平数组，每 3 个数为一个顶点：`lat, lon, alt`。
- `poly` 顶点必须按田块边界顺序给出（逆序/旋转起点可以），不能自相交；乱序输入会被拒绝。
- `poly` 的第一个顶点同时作为 ENU 原点；实车还要把
  `gnss_localization/config/gnss_driver.yaml` 的 `origin_lat/lon/alt`
  配成同一个坐标。
- `ridge_width` 必须大于 0，决定弓字形行距。
- `field_margin` 必须大于等于 0；建议不小于 `ridge_width/2`，保证机具半幅不越过田边
  （掉头弧本身切在内缩边界上）。
- 所有参数在启动时读取，修改后需要重启规划器。

## 三、启动时序

一键启动时无需手工操作：

```text
global_path_planner 启动后按 YAML 田块立即规划
    ↓
sim_orchestrator 检测到非空 /global_path_planner/path
    ↓
sim_orchestrator 切 UNMANNED → path_tracker 开始驱动
```

## 四、手动分终端调试

一键启动正常时不需要看这一节；排查单个节点时使用。

### 终端 1：仿真车辆

```bash
source install/setup.bash
ros2 run sim_vehicle sim_vehicle --ros-args \
  -p wheelbase:=1.2 \
  -p align_to_path_start:=true \
  -p align_lateral_offset:=0.8
```

### 终端 2：规划器

```bash
source install/setup.bash
ros2 launch global_path_planner planner.launch.py
```

规划器会立即按 YAML 田块发布路径。

### 终端 3：控制器

```bash
source install/setup.bash
ros2 launch path_tracker path_tracker.launch.py
```

在切 UNMANNED 前，控制器会持续输出停车。

### 终端 4：确认路径并切无人模式

```bash
ros2 topic echo /global_path_planner/path --once \
  --field x --truncate-length 5
ros2 topic pub --once /sim/unmanned std_msgs/msg/Bool "{data: true}"
```

`sim_vehicle` 收到后会以 1 Hz 持续转发 `/work_mode`。

## 五、可视化

一键启动已带 RViz 预置视图，Fixed Frame 为 `odom`：

| 显示项 | 话题 |
|---|---|
| 田块边界 | `/global_path_planner/boundary_points` |
| 内缩边界 | `/global_path_planner/inner_boundary_points` |
| 掉头关键点 | `/global_path_planner/key_points` |
| 全覆盖路径 | `/global_path_planner/path` |
| 仿真车 | `/sim_vehicle/marker` |
| 跟踪目标点 | `/path_tracker/target_point` |

`/gnss_driver/odom` 使用 SensorDataQoS（Best Effort）。手动添加显示时
必须把 RViz 的 Reliability 改成 **Best Effort**。

## 六、判断成功

| 检查项 | 成功标志 |
|---|---|
| 规划器出路径 | `/global_path_planner/path` 有几十到上百个点 |
| 控制器接管 | 日志从“未使能/等待路径”变为跟踪输出 |
| 转向收敛 | `steer_angle_rad` 从初始偏差逐渐减小 |
| 车在动 | `ros2 run tf2_ros tf2_echo odom base_link` 坐标持续变化 |
| 跟踪正常 | RViz 中绿箭头贴合路径，橙球随车前进 |

## 七、常见问题

**规划器一直不发布路径**

检查：

```bash
ros2 topic echo /work_mode --once
```

以及规划器启动日志中的 YAML 配置摘要。`poly` 至少需要 3 个顶点，
`ridge_width` 必须大于 0，`field_margin` 不能过大。

**path_tracker 一直“定位精度不足”**

`sim_vehicle` 未运行，或 `rtk_sigma` 被设成 0。默认 0.02 是正确的。

**车不动但无报错**

确认 `/work_mode` 为 UNMANNED，且 `/global_path_planner/path` 非空。

**chassis_node 要不要启动**

最小仿真闭环不需要。`sim_vehicle` 已直接订阅 `/chassis/cmd`。启动
`chassis_node` 只用于检查 CAN 编码，它不会收到真实 VCU 状态，因此
`/chassis/state` 中的 `*_valid` 会保持 false。
