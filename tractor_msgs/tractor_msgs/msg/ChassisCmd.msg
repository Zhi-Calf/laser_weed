# Chassis control command from ADCU to VCU
# ADCU_toVCU_01 (0x0C01D01C) sent at 50Hz (20ms)

std_msgs/Header header

# --- ADCU_toVCU_01 fields (20ms cycle) ---

# Run mode command: 0=I(Init), 1=W(Working), 2=C(Charging), 3=V(void)
uint8 run_mode_cmd  
uint8 RUN_I = 0
uint8 RUN_W = 1
uint8 RUN_C = 2
uint8 RUN_V = 3

# Control flags
bool radar_enable            # Obstacle avoidance enable
bool emergency_stop          # Remote e-stop
bool fault_reset             # Remote fault reset
bool autonomous_mode         # false=remote control, true=autonomous

# --- Convenience fields (chassis_interface does inverse kinematics) ---
float64 linear_velocity      # m/s,
float64 angular_velocity     # rad/s
