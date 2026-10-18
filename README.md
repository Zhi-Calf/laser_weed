[README.md](https://github.com/user-attachments/files/33640675/README.md)
# laser_weed · 激光除草机器人无人驾驶系统

ROS 2 Humble 软件栈，面向**四轮独立转向**激光除草机器人。集成 RTK 定位、全覆盖路径规划、
横向跟踪控制与底盘 CAN 桥接，并提供一套**无需实车、无需 RTK 硬件**的全链路联调仿真环境。

```
GNSS 接收机 ──串口──► gnss_localization ──/gnss_driver/odom──┐
                          │                                  │
                          ├──/gnss_driver/origin──► global_path_planner
                          │                              │ 弓字形全覆盖规划
                          │                              ▼ /global_path_planner/path
                          │                          lqr_control ──/weeder/steer_cmd──► weeder_base ──CAN──► 底盘 VCU
                          │                          (四轮转向 LQR)        (/cmd_vel 回退)   (20ms 指令帧)
                          └──/work_mode◄──────────────┴────────────────────────────┘
                             (系统级使能总线)                    ▲
                                                          sim_vehicle（仿真时替代定位+底盘）
```

## 仓库结构

| 包 | 职责 | 说明 |
|---|---|---|
| `gnss_localization` | RTK 定位 | 串口读取接收机 GIAVP 协议，经纬度经 GeographicLib 投影为 ENU 平面坐标；发布 odom / origin / imu / vel / status 与 `odom→base_link` TF。`ntrip_client`（网络差分）与 `sdk_client`（千寻 SDK 差分）二选一 |
| `global_path_planner` | 全覆盖路径规划 | 输入田块凸多边形（经纬度），输出弓字形（Boustrophedon）作业路径；触发入口为 `set_source` 服务。Clipper2 随包 vendored |
| `lqr_control` | 横向跟踪控制 | 四轮转向 LQR：状态量 `[横向误差, 航向误差]`，控制量为前轮转角，前馈 `δ = L·κ`。含 5 道安全门控与偏离距离保护 |
| `weeder_base` | 底盘 CAN 桥 | ROS 指令 ↔ CAN 帧（协议第二版 `ADCU_toVCU_01`，20ms 周期），含指令超时 / VCU 心跳双看门狗 |
| `laser_weed_can` | CAN 协议编解码 | 纯 C++、零 ROS 依赖，被 `weeder_base` 引用 |
| `tractor_msgs` | 消息/服务定义 | `WorkMode`、`GnssStatus`、`SetPathSource`、`Heartbeat` 等 |
| `tractor_monitor` | 健康监测 | 各节点发 `/heartbeat`，本包汇总为 `/monitor_heartbeat` 供底盘判断停车 |
| `sim_vehicle` | 无硬件联调仿真 | 单节点同时模拟 GNSS 定位、车辆运动学（差速/阿克曼可配）、工作模式管理；含一键启动与 RViz 预置视图 |
| `stanley_control` | （已废弃） | 早期 Python Stanley 控制器，已由 `lqr_control` 取代，保留作历史参考 |

## 构建

系统依赖（Ubuntu 22.04 / ROS 2 Humble）：

```bash
sudo apt update
sudo apt install -y libeigen3-dev libgeographic-dev libboost-dev \
                    nlohmann-json3-dev libyaml-cpp-dev ros-humble-spdlog-vendor
# 或 rosdep install --from-paths src --ignore-src -r -y
```

> ⚠️ `geographiclib` 的 apt 包名随系统版本变化：Jammy/Humble 为 `libgeographic-dev`，
> Noble/Jazzy 为 `libgeographiclib-dev`。

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 仿真（无实车 / 无 RTK 硬件）

```bash
ros2 launch sim_vehicle weeder_sim.launch.py
```

一条命令启动 5 个节点：`sim_vehicle`、`global_path_planner`、`lqr_control`、
`sim_orchestrator`（自动完成 等原点 → MANUAL → 下发田块 → 等路径 → UNMANNED 的时序）、`rviz2`（预置视图）。

常用参数：

```bash
ros2 launch sim_vehicle weeder_sim.launch.py ridge_width:=3.0 target_v:=0.8 rviz:=false
```

RViz 中绿色箭头为仿真车、绿色折线为弓字形路径、橙色球为控制器跟踪目标点。
详见 `sim_vehicle/RUN_SIM.md`。

## 实车部署

上实车前必须完成的配置（详见 `lqr_control/PORT_CHECKLIST.md`）：

| 配置 | 位置 | 说明 |
|---|---|---|
| 天线杆臂 `calib_offset_x/y` | `gnss_localization/config/gnss_driver.yaml` | **必须按实车安装位置测量**。填错会导致 odom 整体偏移 |
| 串口 | 同上 + 差分客户端 yaml | 按实际接线（`ls /dev/ttyUSB*`） |
| 差分账号 | `ntrip_client.yaml` 或 `sdk_client.yaml` | 账号绑定设备，需为每台车单独申请 |
| `wheelbase` | `lqr_control/config/lqr_control.yaml` | 按实车轴距填写，且需与仿真/控制器各处一致 |
| `steer_mode` | 同上 + `weeder_base` 参数 | 两侧必须同值；模式 1（双阿克曼）时控制器内部取等效轴距 L/2 |
| `/work_mode` | 外部发布 | 需以 ≥1 Hz 持续发布 `MODE_UNMANNED`，否则控制器 3 s 后停车 |

> ⚠️ **`weeder_base/src/can_driver.cpp` 当前为桩实现**（`open()` 直接返回成功、`send()` 仅打印日志），
> 用于无 CAN 硬件的台架验证。**实车部署前必须替换为真实 SocketCAN 实现。**

建议的现场验证顺序：RTK 固定解（`fix_type == 4`）→ 手动发转向指令看车轮 → 规划路径 → 低速跟踪。

## 安全机制

`lqr_control` 的门控按序判定，任一不满足即输出零速零转角并上报故障码：

| 门控 | 参数 | 故障码 |
|---|---|---|
| 工作模式（UNMANNED 才接管） | `work_mode_timeout_s` | 10101 |
| RTK 位姿存在/新鲜/固定解 | `odom_timeout_s` / `rtk_max_sigma` | 10102 |
| 路径存在/新鲜 | `path_timeout_s` | 10103 |
| 转向模式与阿克曼模型兼容 | `steer_mode` | 10104 |
| 路径走完 | — | 10105 |
| 偏离跟踪目标过远 | `max_track_error_m` | 10106 |

**10106 是必需项而非可选优化**：LQR 状态量只有横向与航向误差，**没有沿路径方向的误差项**——
车辆沿路径方向冲过目标点时两项误差同时为 0，控制器会判定"完美跟踪"并继续全速前进。
该门控是发现这种情况的唯一手段，代码强制下限 0.1 m 防止误配为 0。
`weeder_base` 另有指令超时与 VCU 心跳两道看门狗作为第二道防线。

## 测试

```bash
colcon test --packages-select lqr_control gnss_localization global_path_planner
colcon test-result --verbose
```

- `gnss_localization` + `global_path_planner`：175 项测试通过（另 26 项需硬件在环，预期跳过）
- `lqr_control`：含 19 项单元测试，覆盖四轮转向运动学前馈 `δ_ff = L·κ`、转向符号约定、
  闭环收敛仿真、±90°/1.4 m/s 协议限幅、沿路径盲区回归与目标点前向吸附
  （以最近一次本地 `colcon test` 输出为准）

## 已知限制

- `global_path_planner` **仅接受凸多边形田块，不支持田块内障碍物挖空**；非凸输入直接拒收
- `lqr_control` 仅适用于转向模式 0（单阿克曼）与 1（双阿克曼）；模式 2/3（平移/原地转向）下
  阿克曼假设不成立，节点会拒绝接管
- 仿真为理想模型（无轮胎侧偏、执行器延迟、RTK 延迟、地面打滑），**仿真通过是上实车的必要条件，不是充分条件**
- 不支持倒车（`gear` 固定 D 档）

## 来源与许可证

- `gnss_localization`、`global_path_planner`、`tractor_msgs`、`tractor_monitor` 移植自组织内部
  履带拖拉机项目（MegaTrak），移植时**源码未改动**，仅适配配置
- 弓字形覆盖算法部分概念借鉴自 [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover)
  （BSD-3），借鉴范围与实质性简化说明见 `docs/F2C_COVERAGE_BORROW.md`（选向固定为最小宽度边、
  掉头固定半圆等）。**注意**：BSD-3 要求保留原版权声明，而当前源码文件中未包含该声明，
  建议后续在 `bow.hpp/bow.cpp` 头部补充 Fields2Cover 的版权与许可证文本

