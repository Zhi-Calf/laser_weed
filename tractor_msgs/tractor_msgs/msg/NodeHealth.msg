# 单个被监控模块的健康快照, 由 tractor_monitor 判定。
#
# 与 Heartbeat 的区别: Heartbeat 是模块"自己说"的, NodeHealth 是 tractor_monitor
# "判定"的 —— 它把自报状态和存活性合在一起。这个区别是必要的: 一个已经死掉的
# 模块最后一条自报状态很可能是 OK, 光看 Heartbeat 会一直显示健康。

string node_name

# 有效状态。模块不再送达心跳时为 FAULT, 无论它最后一次自报的是什么。
# 与 Heartbeat.status 同一套 4 态语义。
uint8 status
uint8 STATUS_OK             = 0   # 无故障 (正常运行)
uint8 STATUS_STOP_IMPLEMENT = 1   # 停止作业
uint8 STATUS_STOP_MOTION    = 2   # 停止跟踪
uint8 STATUS_FAULT          = 3   # 停止作业和停车

bool alive           # 收到过心跳且未超时
bool ever_seen       # 是否出现过（false = 从未启动）

# 距最近一条心跳的秒数。ever_seen 为 false 时为 -1。
float64 age_s

uint8 criticality
uint8 CRIT_CRITICAL = 0   # 丢失/FAULT -> 整车停车并置急停
uint8 CRIT_DEGRADED = 1   # 丢失/FAULT -> 仅上报
uint8 CRIT_IGNORE   = 2   # 不参与裁决

# 本模块的状态是否参与了 SystemHealth.overall 的裁决。
# 这一位解释了"为什么这行是 FAULT, 车却没停": ignore 类模块, 或者压根没启动过的
# 可选模块, 都不参与。
bool counted

# seq 回退的次数 = 该模块重启过几次。反复崩溃重启的节点, 每次采样看上去都健康,
# 只有这个计数能看出问题。
uint32 restarts

string detail        # 自报的说明, 正常时为空
