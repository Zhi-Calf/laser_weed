# Overall robot status for cloud reporting via MQTT Bridge
# Published on /control/status, mapped to agri/{robot_id}/state

std_msgs/Header header

string robot_id

uint8 status
uint8 STATUS_IDLE      = 0
uint8 STATUS_WORKING   = 1
uint8 STATUS_ERROR     = 2
uint8 STATUS_UPGRADING = 3

uint64 uptime_sec

# System resource usage
float32 cpu_percent
float32 mem_percent
float32 disk_percent

# Battery
float64 battery_voltage
float64 battery_soc

# Service health (true = running)
bool service_control_node
bool service_sensor_node
bool service_navigation_node
bool service_mqtt_bridge
bool service_ota_agent

# Current software versions
string version_control_node
string version_sensor_node
string version_navigation_node
string version_mqtt_bridge
string version_ota_agent

# Fault summary
bool has_fault
uint32 fault_code
string fault_description
