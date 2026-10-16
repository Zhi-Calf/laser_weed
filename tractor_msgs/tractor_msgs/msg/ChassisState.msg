# Chassis state feedback from VCU via CAN bus (V2 protocol)

std_msgs/Header header

# === Chassis_State_01 (0x18011CD0, 20ms) ===
uint8 gear_status            # 废弃，不再使用
uint8 run_mode_status        # 0=I(Init), 1=W(Working), 2=(manual Charging), 3=V(auto Charging)
bool radar_en_status
bool estop_status
bool reset_status
bool ctrl_mode_status        # false=remote, true=autonomous
float64 linear_velocity      # m/s (from VCU, originally mm/s)
float64 angular_velocity     # rad/s (from VCU, originally 0.001 rad/s)
uint8 alive_counter

# === Chassis_State_02 (0x18021CD0, 100ms) ===
bool pto_on_status_rear
bool pto_on_status_front
bool hitch_on_status_rear
bool hitch_on_status_front
uint16 pto_actual_speed_rear   # r/min (0, 540, 720, 1000)
uint16 pto_actual_speed_front  # r/min
float32 hitch_actual_height_rear   # 0~100%
float32 hitch_actual_height_front  # 0~100%

# === Chassis_State_03 (0x18031CD0, 500ms) ===
uint8 mcu_error_level_left   # 0=OK, 1=info, 2=warn, 3=critical
uint8 mcu_error_code_left
float32 mcu_motor_temp_left  # Celsius
float32 mcu_temp_left        # Celsius
uint8 mcu_error_level_right
uint8 mcu_error_code_right
float32 mcu_motor_temp_right # Celsius
float32 mcu_temp_right       # Celsius

# === Chassis_State_04 (0x18041CD0, 100ms) - Left motor ===
uint16 motor_speed_left      # r/min (raw motor-side RPM from MCU)
float32 motor_current_left   # A (phase current)
float32 motor_bus_voltage_left   # V (DC bus)
float32 motor_bus_current_left   # A (DC bus)

# === Chassis_State_05 (0x18051CD0, 100ms) - Right motor ===
uint16 motor_speed_right     # r/min (raw motor-side RPM from MCU)
float32 motor_current_right  # A (phase current)
float32 motor_bus_voltage_right  # V (DC bus)
float32 motor_bus_current_right  # A (DC bus)

# === Chassis_State_06 (0x18061CD0, 500ms) ===
uint8 pto_error_level        # 0=OK, 1=info, 2=warn, 3=critical
uint8 pto_error_code
float32 pto_motor_temp       # Celsius
float32 pto_mcu_temp         # Celsius
uint8 hyd_error_level
uint8 hyd_error_code
float32 hyd_motor_temp       # Celsius
float32 hyd_mcu_temp         # Celsius

# === Chassis_State_07 (0x18071CD0, 100ms) - PTO motor ===
uint16 pto_motor_speed       # r/min
float32 pto_motor_current    # A
float32 pto_bus_voltage      # V
float32 pto_bus_current      # A

# === Chassis_State_08 (0x18081CD0, 500ms) - Hydraulic pump ===
uint16 hyd_motor_speed       # r/min
float32 hyd_motor_current    # A
float32 hyd_bus_voltage      # V
float32 hyd_bus_current      # A

# === Chassis_State_21 (0x18211CD0, 100ms) - BMS status ===
bool bms_power_on
uint8 bms_heating_status     # 0=off, 1=charge-heat, 2=charge+heat, 3=discharge-heat
uint8 bms_charge_status      # 0=standby, 1=discharge, 2=slow charge, 3=fast charge
uint8 bms_soh                # %
float32 bms_soc              # V (0.1 resolution)
float32 bms_voltage          # V
float32 bms_current          # A

# === Chassis_State_22 (0x18221CD0, 500ms) - BMS faults ===
uint8 bms_error_level        # 0=OK, 1=info, 2=warn, 3=critical
uint8 bms_error_code
uint8 bms_fault_count
float32 bms_max_temp         # Celsius
uint8 bms_max_temp_pos
float32 bms_min_temp         # Celsius
uint8 bms_min_temp_pos

# === Chassis_State_23 (0x18231CD0, 500ms) - BMS cell voltages ===
uint16 bms_max_cell_voltage  # mV
uint8 bms_max_cell_voltage_pos
uint16 bms_min_cell_voltage  # mV
uint8 bms_min_cell_voltage_pos
uint16 system_voltage        # mV
