# 标定流程结果。
#
# comm_gateway 用该消息把本机标定结果转换成 APP 的 CALIBRATION_STATUS。
# accepted 语义不在本消息中；指令是否受理由 CalibrationCommand.srv 的响应表达。

std_msgs/Header header

uint8 calibration_type         # 与 CalibrationCommand.srv 的 CALIB_* 同义
bool success                   # 本次标定是否成功
string message                 # 成功说明或失败原因

float64 calib_offset_x         # 底盘车体系 x 向残余平移，m
float64 calib_offset_y         # 底盘车体系 y 向残余平移，m
float64 calib_offset_heading   # 航向偏角，rad
