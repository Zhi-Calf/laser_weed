# 全覆盖路径
#
# x/y 与 waypoint_types 按索引一一对应。
# waypoint_types: 0 = 作业段, 1 = 掉头段。
# path_tracker 使用类型字段切换 WORK/TURN 速度和单/对称阿克曼模式。

std_msgs/Header header

float64[] x
float64[] y
uint8[] waypoint_types
