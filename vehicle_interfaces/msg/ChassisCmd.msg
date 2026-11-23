# 车辆运动控制指令
#
# 速度与转角必须由同一条消息发布，保证底盘执行的是同一控制周期内
# 计算出的运动学组合，避免两个话题到达时刻不一致。

std_msgs/Header header

uint8 gear               # 0=N，1=D，2=R，3=无效
uint8 steer_mode         # 0=单阿克曼，1=双阿克曼，2=平移，3=原地
bool release_brake       # true=强制释放抱闸
bool estop               # true=远程急停
bool ctrl_mode           # false=遥控，true=无人驾驶
float64 speed_m_s        # 前进速度，m/s，>= 0
float64 steer_angle_rad  # 前轮转角，rad，左转为正，右转为负
