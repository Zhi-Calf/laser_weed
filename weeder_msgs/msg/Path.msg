# 全覆盖路径
#
# x/y 与 waypoint_types 按索引一一对应。
# waypoint_types: 0 = 作业段, 1 = 掉头段。
# 当前 lqr_control 只读取 x/y 做跟踪；类型字段预留给后续机具/激光分段控制。

std_msgs/Header header

float64[] x
float64[] y
uint8[] waypoint_types
