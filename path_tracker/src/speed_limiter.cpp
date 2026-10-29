#include "path_tracker/speed_limiter.hpp"

#include <algorithm>

namespace path_tracker {
namespace {

// 将目标值限制在上一拍值附近单拍允许的最大变化量内。
double LimitChange(double target, double previous, double max_change) {
  if (target > previous) {
    return std::min(target, previous + max_change);
  }
  return std::max(target, previous - max_change);
}

}   // namespace

// 按最大加速度限制相邻两拍之间的速度变化。
// 输入:
//   target_speed_m_s       : 本拍期望速度 (m/s)
//   previous_speed_m_s     : 上一拍输出速度 (m/s)
//   max_acceleration_mps2  : 最大速度变化率 (m/s²)
//   dt_s                   : 控制周期 (s)
// 返回:
//   不超过 max_acceleration_mps2 * dt_s 的限速结果 (m/s)
double LimitSpeed(double target_speed_m_s, double previous_speed_m_s, double max_acceleration_mps2, double dt_s) {
  const double target_speed   = std::max(target_speed_m_s, 0.0);
  const double max_speed_step = std::max(max_acceleration_mps2, 0.0) * std::max(dt_s, 0.0);
  return LimitChange(target_speed, previous_speed_m_s, max_speed_step);
}

// 按最大角速度限制相邻两拍之间的转角变化。
// 输入:
//   target_angle_rad    : 本拍期望转角 (rad)
//   previous_angle_rad  : 上一拍输出转角 (rad)
//   max_rate_rad_s      : 最大转角变化率 (rad/s)
//   dt_s                : 控制周期 (s)
// 返回:
//   不超过 max_rate_rad_s * dt_s 的限幅转角 (rad)
double LimitSteerAngle(double target_angle_rad, double previous_angle_rad, double max_rate_rad_s, double dt_s) {
  const double max_angle_step = std::max(max_rate_rad_s, 0.0) * std::max(dt_s, 0.0);
  return LimitChange(target_angle_rad, previous_angle_rad, max_angle_step);
}

}   // namespace path_tracker
