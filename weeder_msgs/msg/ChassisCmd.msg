# 车辆运动控制指令
#
# 速度与转角必须由同一条消息发布，保证底盘执行的是同一控制周期内
# 计算出的运动学组合，避免两个话题到达时刻不一致。

std_msgs/Header header

float64 speed_m_s        # 前进速度，m/s，>= 0
float64 steer_angle_rad  # 前轮转角，rad，左转为正，右转为负
uint8 steer_mode         # 0=单阿克曼，1=双阿克曼，2=平移，3=原地
