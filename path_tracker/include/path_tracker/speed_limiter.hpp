#pragma once

namespace path_tracker {

double LimitSpeed(double target_speed_m_s, double previous_speed_m_s, double max_acceleration_mps2, double dt_s);
double LimitSteerAngle(double target_angle_rad, double previous_angle_rad, double max_rate_rad_s, double dt_s);

}   // namespace path_tracker
