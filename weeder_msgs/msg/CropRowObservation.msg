# Observation only: this message never grants permission to drive.
# header.stamp is the original image ROS acquisition stamp, not publish time.
# header.frame_id is the calibrated vehicle reference frame (normally base_link).
std_msgs/Header header
string session_id
uint64 sequence
bool has_capture_time
bool replay

uint8 INVALID=0
uint8 VALID=1
uint8 LOW_CONFIDENCE=2
uint8 NO_ROW=3
uint8 STALE=4
uint8 NO_INPUT=5
uint8 INVALID_INPUT=6
uint8 SDK_ERROR=7
uint8 status
int32 sdk_return_code
string detail

# x forward, y left. Path y = tan(heading_error_rad)*x + lateral_error_m.
# Positive lateral means PATH is to the LEFT of vehicle, opposite to LQR e_y.
# All non-VALID observations contain NaN navigation values and confidence=0.
float64 lateral_error_m
float64 heading_error_rad
float64 lookahead_m
float64 confidence
float64 processing_ms
float64 frame_age_ms
uint64 dropped_frames
string sdk_version
string calibration_id
