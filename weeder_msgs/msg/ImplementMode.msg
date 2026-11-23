# 农具（PTO 与三点悬挂）当前生效模式。
#
# app_bridge 从 /implement/mode 订阅该消息，用于 APP 遥测中的农具状态。
# 本消息只报告已经生效的状态，不表示切换请求是否被受理。

std_msgs/Header header

uint8 mode
uint8 MODE_MANUAL      = 0   # 操作员直接控制农具
uint8 MODE_AUTONOMOUS  = 1   # 按路径自动控制农具
uint8 MODE_PLANTING    = 2   # 播种作业模式
uint8 MODE_ROTARY_TILL = 3   # 旋耕作业模式

string mode_name       # mode 的可读名称，便于日志与 ros2 topic echo
string source          # 当前模式的请求来源

# 进入当前模式的时间，与消息发布时间 header.stamp 不同。
builtin_interfaces/Time since
