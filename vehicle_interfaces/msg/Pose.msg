# GNSS/RTK 位姿
#
# 坐标与角度均使用 ROS REP-103 约定。
# valid=true 表示当前位姿达到 RTK 固定解，可用于自动驾驶。

std_msgs/Header header

float64 x                 # ENU x (m)
float64 y                 # ENU y (m)
float64 z                 # ENU z (m)
float64 roll              # 横滚角 (rad)
float64 pitch             # 俯仰角 (rad)
float64 yaw               # 偏航角 (rad)
uint32 satellites         # 当前参与解算的卫星数量
bool valid                # 位姿是否可用于自动驾驶
float64 speed_m_s         # 车体前向速度 (m/s，倒车为负)
