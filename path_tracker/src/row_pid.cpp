#include "path_tracker/row_pid.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace path_tracker {

RowPidController::RowPidController(RowPidConfig config) : config_(std::move(config)) {}

void RowPidController::Configure(RowPidConfig config) {
  config_ = std::move(config);
  Reset();
}

void RowPidController::Reset() {
  integral_error_m_s_ = 0.0;
  previous_error_m_   = 0.0;
  derivative_mps_     = 0.0;
  has_previous_       = false;
}

// 按车体中心线前视误差计算横向 PID 和航向对准转角。
// 输入:
//   target : 车体坐标下的中心线截距、航向误差和前视距离
//   dt_s   : 控制周期 (s)
// 返回:
//   有效性和各分项转角贡献；输入非法时返回无效零输出。
RowPidOutput RowPidController::Compute(const RowPidTarget& target, double dt_s) {
  RowPidOutput output;
  if (
    !std::isfinite(target.lateral_error_m) || !std::isfinite(target.heading_error_rad) || !std::isfinite(target.lookahead_m) || target.lookahead_m < 0.0 || !std::isfinite(dt_s) || dt_s <= 0.0 ||
    std::abs(std::cos(target.heading_error_rad)) < 1e-9) {
    return output;
  }

  const double slope   = std::tan(target.heading_error_rad);
  const double error_m = target.lateral_error_m + slope * target.lookahead_m;
  if (!std::isfinite(error_m)) {
    return output;
  }

  integral_error_m_s_ += error_m * dt_s;
  if (std::abs(config_.ki_rad_per_m_s) > 1e-12) {
    const double max_integral = config_.integral_limit_rad / std::abs(config_.ki_rad_per_m_s);
    integral_error_m_s_       = std::clamp(integral_error_m_s_, -max_integral, max_integral);
  }
  const double raw_derivative = has_previous_ ? (error_m - previous_error_m_) / dt_s : 0.0;
  const double alpha          = dt_s / (config_.derivative_tau_s + dt_s);
  derivative_mps_ += alpha * (raw_derivative - derivative_mps_);
  previous_error_m_ = error_m;
  has_previous_     = true;

  output.cross_track_m    = error_m;
  output.p_term_rad       = config_.kp_rad_per_m * error_m;
  output.i_term_rad       = std::clamp(config_.ki_rad_per_m_s * integral_error_m_s_, -config_.integral_limit_rad, config_.integral_limit_rad);
  output.d_term_rad       = config_.kd_rad_s_per_m * derivative_mps_;
  output.heading_term_rad = config_.heading_gain * target.heading_error_rad;
  output.steer_rad        = output.p_term_rad + output.i_term_rad + output.d_term_rad + output.heading_term_rad;
  output.valid            = std::isfinite(output.steer_rad);
  return output;
}

}   // namespace path_tracker
