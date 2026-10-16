# tractor_msgs

Custom ROS2 message definitions for the Zhi-Calf agricultural robot platform.

This is a **shared interface package** — all teams in the Zhi-Calf organization can depend on it.

## Messages

| Message | Description |
|---------|-------------|
| `ChassisCmd` | Chassis control command (gear, motor RPM, PTO, hitch, kinematics) |
| `ChassisState` | Chassis state feedback (VCU CAN bus: motors, BMS, PTO, hydraulic) |
| `CalibrationResult` | Calibration status/result: GNSS lever arm + heading offset, `success` + `message` (failure reason) on `/calibration_manager/status` |
| `RobotStatus` | Robot status for cloud reporting (state, battery, services, faults) |
| `Heartbeat` | Per-node liveness + self-reported health (`/heartbeat`, `/monitor_heartbeat`) |
| `NodeHealth` | One module's health as judged by `tractor_monitor` (self-report + liveness) |
| `SystemHealth` | Whole-system snapshot: overall verdict + every `NodeHealth` (`/system_health`) |
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

| Repository | Packages |
|------------|----------|
| `MegaTrak` | `tractor_base`, `path_controller`, `tractor_calibration`, `tractor_bringup`, `tractor_monitor`, `gnss_localization`, `global_path_planner`, `local_path_planner`, `implement_controller` |
| `robot-tool` | `tractor_calibration_ui` |

> `implement_controller` 目前**没有**被任何 launch 拉起（它订阅的 `/path/global`、`/path/seg_types` 已无发布者），
> 仍在表内是因为它声明依赖本包 —— 接入前先确认链路。

## Build

Requires ROS2 Humble.

```bash
# As part of a colcon workspace
mkdir -p ~/zhi_calf_ws/src && cd ~/zhi_calf_ws/src
git clone git@github.com:Zhi-Calf/tractor_msgs.git
git clone git@github.com:Zhi-Calf/robot-core.git

cd ~/zhi_calf_ws
colcon build --packages-select tractor_msgs
source install/setup.bash
```

## Standalone Build

```bash
cd ~/zhi_calf_ws
colcon build --packages-select tractor_msgs
source install/setup.bash

# Verify messages are available
ros2 interface list | grep tractor_msgs
ros2 interface show tractor_msgs/msg/ChassisCmd
```

## Versioning

This package follows [Semantic Versioning](https://semver.org/):

- **MAJOR**: Incompatible message structure changes (field removal, type change)
- **MINOR**: New message types or new fields (backward-compatible)
- **PATCH**: Documentation or comment fixes

The version in `package.xml` must match the Git tag before release.

## License

Apache-2.0
