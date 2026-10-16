# Calibration result message

std_msgs/Header header

# Which calibration produced this result
uint8 calibration_type       # 与 CalibrationCommand.srv 的 CALIB_* 同义

# Status
bool success
string message

# GNSS calibration results (标定残差: 杆臂补偿后的定位 vs 底盘真实中心)
float64 calib_offset_x       # 底盘车体系 x 向残余平移 (m)
float64 calib_offset_y       # 底盘车体系 y 向残余平移 (m)
float64 calib_offset_heading # 航向偏角 (rad)
