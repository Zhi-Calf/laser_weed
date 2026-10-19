# weeder_msgs

激光除草机器人使用的 ROS2 消息定义。

## Messages

| Message | Description |
|---------|-------------|
| `ChassisCmd` | 完整底盘指令：档位、转向模式、抱闸、急停、驾驶模式、速度与前轮转角 |
| `ChassisState` | 激光除草机底盘完整状态：State_01~11 四轮底盘/电机数据与 State_21/22 BMS |
| `GnssStatus` | GNSS/RTK fix quality and solution status (`/gnss_driver/status`) |
| `Path` | 全覆盖路径：`x[]`、`y[]` 与 `waypoint_types[]` 按索引一一对应 |
| `WorkMode` | Whole-vehicle work mode (`0`=manual / `1`=unmanned / `2`=calibration), latched so late joiners see the current mode |

当前只保留系统实际使用的 5 个消息。
本包当前没有服务或 Action 接口；规划器的田块和路径来源由
`global_path_planner/config/global_path_planner.yaml` 配置。

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
