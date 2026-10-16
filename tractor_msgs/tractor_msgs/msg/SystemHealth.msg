# 整套系统的健康快照, 由 tractor_monitor 以 check_rate 发布在 /system_health。
#
# 一条消息给出全表。/monitor_heartbeat 只给聚合裁决(tractor_base 消费的那个),
# /heartbeat 只有各模块自报的流 —— 想一眼看到"每个节点现在什么状态", 看这个。
#
# 注意 tractor_base 不在 nodes 里: 它是裁决的消费者, 不是被监控对象。

std_msgs/Header header

# 聚合裁决, 与同一时刻 /monitor_heartbeat 的 status 完全一致。
# 与 Heartbeat.status 同一套 4 态语义。
uint8 overall
uint8 STATUS_OK             = 0   # 无故障 (正常运行)
uint8 STATUS_STOP_IMPLEMENT = 1   # 停止作业
uint8 STATUS_STOP_MOTION    = 2   # 停止跟踪
uint8 STATUS_FAULT          = 3   # 停止作业和停车

# 启动宽限期内。期间"从未出现"的模块不判故障。
bool in_startup_grace

NodeHealth[] nodes
