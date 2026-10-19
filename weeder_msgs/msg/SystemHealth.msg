# 整套系统的健康快照（预留）。
#
# 当前系统未发布该消息。

std_msgs/Header header

# 聚合健康状态，与 Heartbeat.status 使用同一套 4 态语义。
uint8 overall
uint8 STATUS_OK             = 0   # 无故障 (正常运行)
uint8 STATUS_STOP_IMPLEMENT = 1   # 停止作业
uint8 STATUS_STOP_MOTION    = 2   # 停止跟踪
uint8 STATUS_FAULT          = 3   # 停止作业和停车

# 启动宽限期内。期间"从未出现"的模块不判故障。
bool in_startup_grace

NodeHealth[] nodes
