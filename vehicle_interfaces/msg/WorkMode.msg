# 整车工作模式 (谁在控制底盘)。
# 0 手动 / 1 无人 / 2 标定; 同一时刻仅一种模式生效, 由模式管理方
# 以 latched (TRANSIENT_LOCAL) QoS 发布, 后加入的订阅者 (如 APP)
# 可立即获知当前模式。

std_msgs/Header header

uint8 mode
uint8 MODE_MANUAL      = 0   # 手动模式: 操作员直接遥控/驾驶底盘
uint8 MODE_UNMANNED    = 1   # 无人模式: 自动作业 (路径跟踪) 控制底盘
uint8 MODE_CALIBRATION = 2   # 标定模式: calibration_manager 控制底盘执行标定动作

# mode 的可读名称 ("manual", "unmanned", "calibration")。与 mode 冗余,
# 方便 ros2 topic echo 与录包回放直接辨认, 不用查常量表。
string mode_name
