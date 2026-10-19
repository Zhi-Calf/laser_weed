# weeder_base

激光除草机底盘 CAN 通讯节点。协议定义在
`weeder_base/include/weeder_base/protocol/can_protocol.hpp`，本包负责 SocketCAN
收发、ROS2 话题和周期调度。

## 通讯链路

| 方向 | CAN 报文 | 周期 | ROS2 接口 |
|---|---|---|---|
| 下发 | `ADCU_toVCU_01` (`0x0C01D01C`) | 100ms | 订阅 `/chassis/cmd` (`weeder_msgs/ChassisCmd`) |
| 上行 | `Chassis_State_01` (`0x18011CD0`) | 20ms（VCU 周期） | `/weeder/chassis/state`（节点固定 10Hz 发布） |
| 上行 | `Chassis_State_02` (`0x18021CD0`) | 100ms | `/weeder/chassis/state` |
| 上行 | `Chassis_State_03` (`0x18031CD0`) | 100ms | `/weeder/chassis/state` |
| 上行 | `Chassis_State_04~07` (`0x18041CD0~0x18071CD0`) | 500ms | `/weeder/chassis/state` |
| 上行 | `Chassis_State_08~11` (`0x18081CD0~0x180B1CD0`) | 500ms | `/weeder/chassis/state` |
| 上行 | `Chassis_State_21` (`0x18211CD0`) | 500ms | `/weeder/chassis/state` |
| 上行 | `Chassis_State_22` (`0x18221CD0`) | 500ms | `/weeder/chassis/state` |

`/weeder/chassis/state` 使用 `weeder_msgs/msg/ChassisState.msg`。每个
报文都有对应的 `*_valid` 标志，用于区分“值为 0”和“尚未收到该帧”。

## 安全逻辑

- VCU 状态帧超时：停车，VCU 恢复后自动恢复。
- VCU `alive_counter` 不递增：判定 VCU 控制程序卡死并停车。
- VCU 状态超时或 `alive_counter` 卡死：目标转角强制回正。
- 下发协议层还会对目标车速 `0~1400 mm/s`、目标转角 `±90.0°` 做最终限幅。

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to weeder_base
source install/setup.bash
ros2 launch weeder_base can.launch.py interface:=can0
```

协议层是本包内的独立静态库 `weeder_base_protocol`，协议测试通过：

```bash
colcon test --packages-select weeder_base
colcon test-result --verbose
```

台架可用 `vcan0`。查看完整状态：

```bash
ros2 topic echo /weeder/chassis/state
ros2 interface show weeder_msgs/msg/ChassisState
ros2 interface show weeder_msgs/msg/ChassisCmd
```
