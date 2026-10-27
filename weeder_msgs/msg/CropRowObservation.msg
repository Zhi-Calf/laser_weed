# 仅表示稻行视觉观测，不授予底盘控制权限。
# header.stamp 为原始图像的 ROS 采集时间，不是发布或接收时间。
# header.frame_id 为标定后的车体参考系，通常为 base_link。

std_msgs/Header header
string session_id          # 节点本次运行的会话标识，节点重启后变化
uint64 sequence            # 会话内帧序号，允许因丢帧而产生跳号
bool has_capture_time      # header.stamp 是否包含有效的图像采集时间
bool replay                # true=回放数据，不能作为实车实时观测

uint8 INVALID=0            # 观测无效，导航量不可用
uint8 VALID=1              # 观测有效，导航量为有限值
uint8 LOW_CONFIDENCE=2     # 置信度不足，导航量不可用
uint8 NO_ROW=3             # 未识别到有效稻行
uint8 STALE=4              # 观测已过期
uint8 NO_INPUT=5           # 尚未收到可处理的图像
uint8 INVALID_INPUT=6      # 输入图像、时间或参数不符合约定
uint8 SDK_ERROR=7          # SDK 调用失败或输出不符合约定
uint8 status               # 当前观测状态，取值为上方常量
int32 sdk_return_code      # RowNav SDK 返回码；非 0 时需结合 detail 排查
string detail              # 状态补充说明或 SDK 错误详情

# 车体坐标系采用 x 向前、y 向左；导航量的单位分别为 m 和 rad。
# 路径直线为 y = tan(heading_error_rad) * x + lateral_error_m。
# 横向偏差正值表示路径位于车辆左侧，与 LQR 的 e_y 符号方向相反。
# 所有非 VALID 观测的导航量为 NaN，且 confidence 为 0。
float64 lateral_error_m    # x=0 处的路径横向截距 (m)，路径在左为正
float64 heading_error_rad  # 路径相对车体前向的航向偏差 (rad)，向左为正
float64 lookahead_m        # 拟合器的前视距离 (m)，不是横向误差的计算位置
float64 confidence         # 算法综合置信度 [0, 1]，不是正确概率
float64 processing_ms      # SDK 单次处理耗时 (ms)
float64 frame_age_ms       # 从图像采集到发布的总帧龄 (ms)，包含排队和处理
uint64 dropped_frames      # 本会话中因待处理队列覆盖而丢弃的帧数
string sdk_version         # RowNav SDK 版本
string calibration_id      # 标定文件和模型配置的可追溯标识
