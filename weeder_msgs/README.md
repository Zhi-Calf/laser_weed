# weeder_msgs

激光除草机器人使用的 ROS2 消息、服务和 Action 定义。

## Messages

| Message | Description |
|---------|-------------|
| `ChassisCmd` | 车辆运动控制指令：速度 `speed_m_s` + 前轮转角 `steer_angle_rad` + 转向模式 `steer_mode` |
| `ChassisState` | 激光除草机底盘完整状态：State_01~11 四轮底盘/电机数据与 State_21/22 BMS |
| `CalibrationResult` | Calibration status/result: GNSS lever arm + heading offset, `success` + `message` (failure reason) on `/calibration_manager/status` |
| `RobotStatus` | Robot status for cloud reporting (state, battery, services, faults) |
| `Heartbeat` | 预留的节点存活/健康消息，当前系统未使用 |
| `NodeHealth` | 预留的单节点健康快照，当前系统未使用 |
| `SystemHealth` | 预留的系统健康快照，当前系统未使用 |
| `GnssStatus` | GNSS/RTK fix quality and solution status (`/gnss_driver/status`) |
| `WorkMode` | Whole-vehicle work mode (`0`=manual / `1`=unmanned / `2`=calibration), latched so late joiners see the current mode |
| `FaultReport` | Active fault codes of one node + per-code severity (`codes[]` / `levels[]`; 5-digit code = 3-digit module prefix + index) |
| `ImplementCmd` | Implement (PTO + 3-point hitch) command from ADCU to VCU (`ADCU_toVCU_02`) |
| `ImplementMode` | Implement mode actually in effect (manual / autonomous / planting / rotary-till) + who requested it |

## Services

| Service | Description |
|---------|-------------|
| `CalibrationCommand` | 标定指令：`calibration_type` + `action`（`0`=START, `1`=CANCEL, `3`=SAVE），服务名 `/calibration_manager/command`。响应带 `accepted` + `state` + `reason`，调用方当场就知道指令有没有被受理（以及为什么没有），不必轮询 `/calibration_manager/status`。取代旧的 `/calibration_cmd` 话题。 |
| `SetImplementMode` | 农具模式切换，服务名 `/implement/set_mode`。响应带 `accepted` + `active_mode` + `reason`。 |
| `SetPathSource` | 全局路径**来源与几何参数的唯一入口**，服务名 `/global_path_planner/set_source`。请求带 `path_source`（`0`=local 本机规划 / `1`=file 固定路径文件）以及**两种来源各自的输入**：`poly_count` + `poly`（地块顶点，扁平数组每 3 个数 = lat/lon/alt）、`ridge_width` + `field_margin`（垄宽 / 田边余量，local 必填）、`path_file`（file 要加载的 JSON 绝对路径）、`in_row_resolution`（PTO 行内采样步长，两种来源都必填）。响应带 `accepted` + `reason`：**`accepted` 只表示"已受理"**（取值合法 + 已登记），真正的规划 / 读文件 / 发布 `path` 与 `pto_path` 在下一个定时周期（100 ms）内完成。取代旧的 `path_source` / `path_file` / `poly_count` + `poly_N` / `ridge_width` / `field_margin` / `in_row_resolution` 节点参数。 |

## Actions

| Action | Description |
|--------|-------------|
| `ExecutePath` | 沿一条稠密预定路径行驶（可选起转 / 终止前对齐），反馈带每个周期的相位、胡萝卜点序号 / 路径点数、横向与航向误差、下发速度。⚠ 仓内目前**没有任何节点提供或调用它**（原 `tractor_control` 的 `FtcController` 已随该包删除），保留在包内只作接口约定 —— 要启用得自己实现 action server。 |

## Consumers

当前工作区中的 `weeder_base`、`gnss_localization`、
`global_path_planner`、`lqr_control` 和 `sim_vehicle` 依赖本包。

## Build

Requires ROS2 Humble.

```bash
# 在工作区根目录
colcon build --packages-select weeder_msgs
source install/setup.bash
```

## Standalone Build

```bash
colcon build --packages-select weeder_msgs
source install/setup.bash

# Verify messages are available
ros2 interface list | grep weeder_msgs
ros2 interface show weeder_msgs/msg/ChassisState
```

## Versioning

This package follows [Semantic Versioning](https://semver.org/):

- **MAJOR**: Incompatible message structure changes (field removal, type change)
- **MINOR**: New message types or new fields (backward-compatible)
- **PATCH**: Documentation or comment fixes

The version in `package.xml` must match the Git tag before release.

## License

Apache-2.0
