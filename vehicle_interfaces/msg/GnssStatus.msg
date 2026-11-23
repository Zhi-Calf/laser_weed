# GNSS/RTK 状态快照。
#
# 该接口从 tractor_msgs 迁移，供 comm_gateway 兼容既有 FarmDevice 遥测协议。
# 新控制链路优先使用 Pose.msg；这里的经纬度、航向和定位质量字段用于云端上报。

std_msgs/Header header

uint8 FIX_NONE=0
uint8 FIX_SPP=1
uint8 FIX_DGPS=2
uint8 FIX_RTK_FIXED=4
uint8 FIX_RTK_FLOAT=5
uint8 FIX_INS=6

uint8 fix_type          # 当前定位解类型，取上方 FIX_* 常量
uint8 num_satellites    # 参与解算的卫星数
float32 hdop            # 水平精度因子，无量纲
float32 pdop            # 位置精度因子，无量纲
float32 vdop            # 垂直精度因子，无量纲
float32 diff_age        # 差分数据年龄，s
float32 heading_deg     # 航向角，deg，正北为 0，顺时针为正
float32 speed_kmh       # 地速，km/h
float64 latitude_deg    # WGS84 纬度，deg
float64 longitude_deg   # WGS84 经度，deg
float32 altitude_m      # WGS84 椭球高，m
