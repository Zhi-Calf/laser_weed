# 通用故障上报, 供所有节点发布自身的故障码。
#
# 由 chassis_interface (tractor_base) 和 path_controller 等节点各自发布。
#
# codes 是当前所有激活的故障码列表 (可能同时存在多个故障)。
# levels 与 codes 一一对应, 表示该故障的严重程度 (全局统一语义):
#   0 = None     (无故障, 正常)
#   1 = Minor    (轻微: 降级/待命, 不停止核心功能)
#   2 = Major    (严重: 停止本模块核心功能)
#   3 = Critical (致命: 需整车停机)
#
# 注意: 级别只描述"严重程度", 具体停车动作由消费方决定, 不由本消息语义约定。
#
# 故障码编码: 5 位十进制, 前 3 位是模块号, 后 2 位是模块内故障序号。
#   100 = tractor_base   (底盘节点, 如 10001)
#   101 = path_controller (路径跟踪节点, 如 10101)
# 无故障时 codes 为空数组。

std_msgs/Header header

uint32[] codes
uint8[]  levels
