std_msgs/Header header

uint8 FIX_NONE=0
uint8 FIX_SPP=1
uint8 FIX_DGPS=2
uint8 FIX_RTK_FIXED=4
uint8 FIX_RTK_FLOAT=5
uint8 FIX_INS=6

uint8 fix_type
uint8 num_satellites
float32 hdop
float32 pdop
float32 vdop
float32 diff_age
float32 heading_deg
float32 speed_kmh
float64 latitude_deg
float64 longitude_deg
float32 altitude_m
