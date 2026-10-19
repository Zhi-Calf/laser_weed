# Implement (PTO + hitch) command from ADCU to VCU.
# Maps 1:1 onto ADCU_toVCU_02 (0x0C02D01C, 100ms cycle).
# Owner: implement_controller_node. The chassis merges this with
# the motion-side ChassisCmd into a single CAN02 frame; if this topic
# goes stale (>implement_cmd_timeout_s), chassis_interface reverts to
# safe defaults (PTO=0, hitch raised).

std_msgs/Header header

# PTO (Power Take-Off) on/off switches.
bool pto_on_rear
bool pto_on_front

# PTO target speed (r/min). Protocol allows 0, 540, 720, 1000.
uint16 pto_speed_rear
uint16 pto_speed_front

# Hitch (hydraulic 3-point lift) enable switches. Set true while a
# height change is being commanded; the VCU latches actuators when
# the switch is on. Holding it true at steady state is harmless.
bool hitch_on_rear
bool hitch_on_front

# Hitch target height in percent. 0 = fully raised (top), 100 = fully
# lowered (bottom). The implement_controller rate-limits this internally
# so raising stays slow enough not to disturb chassis dynamics.
uint8 hitch_height_rear
uint8 hitch_height_front

# Optional rolling counter for end-to-end liveness checks.
uint8 alive_counter
