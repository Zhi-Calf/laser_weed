# 点云避停状态
#
# sensor_valid=false 表示点云、TF 或避停节点当前不可用。
# stop_required=true 表示 path_tracker 必须输出停车指令。

std_msgs/Header header

bool sensor_valid
bool stop_required
uint32 obstacle_points       # 当前 ROI 内的有效障碍点数
float32 nearest_distance_m   # 最近障碍的水平距离；无有效障碍时为 -1
