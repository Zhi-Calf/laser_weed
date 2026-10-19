# 激光除草机底盘状态 (VCU -> ADCU)
# 依据: 激光除草机无人驾驶通讯协议--第二版.xlsx

std_msgs/Header header

# Chassis_State_01 (0x18011CD0, 20ms)
bool state01_valid
uint8 gear                  # 0=N, 1=D, 2=R, 3=无效
uint8 steer_mode            # 0=单阿克曼, 1=双阿克曼, 2=平移, 3=原地
bool estop
bool reset
bool release_brake
bool ctrl_mode              # false=遥控, true=无人驾驶
uint16 speed_raw            # mm/s (协议 raw 值)
float32 speed_m_s
uint8 wheel_track_cm
bool sound
bool light_green
bool light_yellow
bool light_red
bool headlight
bool turn_left
bool turn_right
bool steer_done_fl
bool steer_done_fr
bool steer_done_rl
bool steer_done_rr
bool bumper
bool bypass
bool battery_charging
bool battery_full
uint8 alive_counter

# Chassis_State_02 (0x18021CD0, 100ms), 顺序 FL/FR/RL/RR
bool state02_valid
uint16[4] motor_speed_rpm

# Chassis_State_03 (0x18031CD0, 100ms), 顺序 FL/FR/RL/RR, 左正右负
bool state03_valid
float32[4] steer_angle_deg

# Chassis_State_04~07 (0x18041CD0~0x18071CD0, 500ms), 顺序 FL/FR/RL/RR
bool[4] drive_motor_valid
uint8[4] drive_bus_voltage       # V
uint16[4] drive_out_current      # A
uint8[4] drive_motor_temp        # degC
uint16[4] drive_error_code

# Chassis_State_08~11 (0x18081CD0~0x180B1CD0, 500ms), 顺序 FL/FR/RL/RR
bool[4] steer_motor_valid
uint8[4] steer_bus_voltage       # V
uint16[4] steer_out_current      # A
uint8[4] steer_motor_temp        # degC
uint16[4] steer_error_code

# Chassis_State_21 (0x18211CD0, 500ms)
bool state21_valid
uint16 bms_soc                   # %
float32 bms_voltage              # V
float32 bms_current              # A
float32 bms_full_charge_capacity # Ah (文档单位写 A)

# Chassis_State_22 (0x18221CD0, 500ms)
bool state22_valid
uint16 bms_cycle_count
uint16 bms_error_code
uint16 bms_max_cell_voltage      # mV
uint16 system_voltage            # mV
