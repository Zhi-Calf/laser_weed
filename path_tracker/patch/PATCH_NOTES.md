# CAN 桥补丁：接入 /weeder/steer_cmd

> **历史说明：** 本补丁已被当前 `weeder_base` 实现取代。现在速度与转角统一由
> `/chassis/cmd`（`weeder_msgs/ChassisCmd`）发布，不要直接应用本目录的旧文件。

**目的**：让 `path_tracker` 算出的转向角能真正下发到底盘。

**用法**：`chassis_node.cpp` 是**完整替换文件**，不是 diff 补丁。直接覆盖：

```bash
cp chassis_node.cpp ~/ros2_ws/src/laser_weed_ros2_node/weeder_base/src/chassis_node.cpp
cd ~/ros2_ws
colcon build --packages-select weeder_base --symlink-install
```

基于你 2026-10-15 提供的版本（含 9 项已有修复）改写，**那些修复全部保留**，只加转向来源。

---

## 改了什么

| # | 改动 | 说明 |
|---|---|---|
| 1 | 新增 `/weeder/steer_cmd` 订阅 | `std_msgs/Float64`，弧度，左正。**这是权威转向来源** |
| 2 | `angular.z` 映射降级为**回退** | 保留手动遥控能力，自动作业时不生效 |
| 3 | `steer_mode` 提为参数 | 原先恒为 0，无法与 `path_tracker` 对齐 |
| 4 | 新增转向超时看门狗 | `steer_timeout_s`（默认 0.5s），超时回正 |

**没动的部分**：速度控制、两个原有看门狗（cmd/VCU）、急停清除逻辑、时钟源、析构顺序、锁、日志节流、`open()`/`send()` 返回值检查——全部原样保留。

---

## 关键设计：转向角优先级

两个来源可能同时发，必须在 `sendCommand()` 里**集中裁决**，否则后到的回调会覆盖看门狗的回正：

```
1. 任一看门狗触发（cmd 超时 / VCU 超时 / 转向超时）→ 回正 0，不给任何来源翻盘机会
2. /weeder/steer_cmd 新鲜（≤ steer_timeout_s）      → 用它（path_tracker 的权威输出）
3. steer_fallback_to_cmd_vel=true                   → 用 angular.z 映射（手动遥控）
4. 否则                                              → 回正 0
```

`onSteerCmd()` **只缓存值，不写 `cmd01_.steer_angle`**——就是为了保证这个优先级不被回调顺序打乱。

---

## 新增参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `steer_cmd_topic` | `/weeder/steer_cmd` | 转向指令话题名 |
| `steer_timeout_s` | `0.5` | 转向超时，与 `cmd_timeout_s` 同量级（path_tracker 默认 10Hz，容忍丢 5 帧） |
| `steer_fallback_to_cmd_vel` | `true` | 超时后是否退回 `angular.z` 映射 |
| `steer_mode` | `0` | 0=单阿克曼 1=双阿克曼 2=平移 3=原地转向 |

### ⚠️ steer_mode 必须与 path_tracker 一致

这是我在改的时候发现的**你没提但会影响联调的问题**：原代码没设 `steer_mode`，恒为结构体默认值 0。

而 `path_tracker` 里 `steer_mode=1`（双阿克曼）时会把等效轴距取 `L/2`。如果控制器按 `L/2` 算转角、底盘却按单阿克曼执行，**转角会差一倍**。两边参数必须填同一个值。

### ⚠️ 实车自动作业建议关掉回退

```bash
ros2 run weeder_base chassis_node --ros-args -p steer_fallback_to_cmd_vel:=false
```

默认 `true` 是为了台架调试和手动遥控方便。实车自动作业时，`angular.z` 那个映射量纲上不严谨（把 rad/s 当角度用），**不该有机会影响转向**——关掉后只认 `/weeder/steer_cmd`，超时即回正。

---

## 符号约定（已核对，不需要取反）

`weeder_base::protocol::AdcuCommand01::steer_angle` 是 `int16_t`，0.1° 分辨率，**左正右负**（协议第二版 Byte3~4）。
`path_tracker` 输出弧度、左正。**同向**，所以只做 `弧度 → 度 → ×10` 换算，不取反。

字段名已追踪确认：`weeder_base::protocol::AdcuCommand01`（`weeder_base/protocol/can_protocol.hpp`），
用的是 `gear`/`steer_mode`/`estop`/`speed_raw`/`steer_angle`/`alive_counter` 这套。
工作区里另有一份 `gear_cmd`/`veh_speed_cmd` 命名的旧头文件，**没被引用、不参与编译**，别照它改。

---

## 依赖

`std_msgs` 已是包依赖（`package.xml:14`、`CMakeLists.txt:14,40`），Float64 不用改构建配置。

---

## 验证

```bash
# 1. 确认订阅建立
ros2 node info /chassis_node    # 应看到 /weeder/steer_cmd 在 Subscribers 里

# 2. 台架测试：手动发转向角，看 CAN 帧
ros2 topic pub -r 10 /weeder/steer_cmd std_msgs/msg/Float64 "{data: 0.1745}"  # 10°
candump can0 | grep 0C01D01C    # Byte3~4 应为 100 (0x64 0x00)，左正

# 3. 超时回正测试：停掉上面的 pub，0.5s 后 Byte3~4 应回到 0
```

`0.1745 rad = 10°`，协议 raw = `10 / 0.1 = 100`。

---

## 已知遗留（本次未处理，与原代码一致）

- State_02~23 解码与发布（含 `/weeder/bms/state_raw`）
- 倒车：`gear` 固定 D 档，`linear.x < 0` 被 clamp 到 0
- `angular.z` 映射量纲不严谨（保留作回退，实车建议关）
