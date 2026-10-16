# 节点心跳 / Per-node liveness and self-reported health.
#
# Published on the shared /heartbeat topic by every monitored module.
# tractor_monitor keys its watch table by node_name and aggregates the lot
# into a single verdict, which it republishes AS THIS SAME TYPE on
# /monitor_heartbeat. tractor_base consumes that one as its stop condition:
# STATUS_FAULT or a stale /monitor_heartbeat zeroes the chassis command and
# asserts e-stop.
#
# Both topics are ABSOLUTE names on purpose. They are system-wide buses whose
# whole point is that they cannot be split by namespace -- a relative name
# resolving into some node's private namespace would yield a monitor that
# watches nothing and reports healthy.

std_msgs/Header header      # header.stamp IS the liveness timestamp

string node_name            # unique; matches the ROS node name

uint8 status
uint8 STATUS_OK             = 0   # 无故障 (正常运行)
uint8 STATUS_STOP_IMPLEMENT = 1   # 停止作业
uint8 STATUS_STOP_MOTION    = 2   # 停止跟踪
uint8 STATUS_FAULT          = 3   # 停止作业和停车

# Per-node monotonic counter, reset to 0 on construction. Distinguishes "node
# restarted" (counter jumps back) from "node never stopped" -- a crash-looping
# node otherwise looks healthy in every sample taken between restarts.
uint32 seq

string detail               # human-readable; empty when OK

# 当前激活的故障码列表 (纯故障 ID, 无等级)。
# 仅供 APP/云端展示"存在某个故障"; 决策动作由 status 表达, 不在此重复。
# 这是原独立故障码话题 (~/faults) 的合并: 故障码退化为纯原因标签。
uint32[] fault_codes

# 故障来源 (责任归属): 这条故障是**谁造成的** —— 合并上报后 APP 必须能直接定位责任方。
#   * 空字符串  = 本节点自身 (自检失败、自己的算法/参数排不出结果、自己打交道的硬件链路失败);
#   * 非空      = 造成这条故障的**具体来源**: 通常是一个 ROS 节点名 (如 "gnss_driver"、
#                 "global_path_planner"); 确实没有对应节点的外部来源用稳定的来源标签
#                 (如 "path_file" 外部工具写的 txt、"vcu" 底盘控制器、"app" 云端/操作手)。
#   * 一条心跳里多个故障码来自不同来源时, 按判定顺序用 ", " 并列 (与合并上报同一格式);
#     全部是自身则为空字符串。
# status = OK 时该字段无意义 (实现上给空字符串)。
string fault_source_node
