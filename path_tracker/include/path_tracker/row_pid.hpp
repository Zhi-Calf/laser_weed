#pragma once

namespace path_tracker {

struct RowPidConfig {
  double kp_rad_per_m       = 0.4;
  double ki_rad_per_m_s     = 0.02;
  double kd_rad_s_per_m     = 0.08;
  double heading_gain       = 0.3;
  double derivative_tau_s   = 0.15;
  double integral_limit_rad = 0.1;
};

struct RowPidTarget {
  double lateral_error_m   = 0.0;
  double heading_error_rad = 0.0;
  double lookahead_m       = 1.0;
};

struct RowPidOutput {
  bool valid              = false;
  double steer_rad        = 0.0;
  double cross_track_m    = 0.0;
  double p_term_rad       = 0.0;
  double i_term_rad       = 0.0;
  double d_term_rad       = 0.0;
  double heading_term_rad = 0.0;
};

// 稻行中心线横向 PID，误差坐标采用车体 x 前、y 左、左转为正。
class RowPidController {
public:
  explicit RowPidController(RowPidConfig config = {});

  void Configure(RowPidConfig config);
  void Reset();
  RowPidOutput Compute(const RowPidTarget& target, double dt_s);

private:
  RowPidConfig config_;
  double integral_error_m_s_ = 0.0;
  double previous_error_m_   = 0.0;
  double derivative_mps_     = 0.0;
  bool has_previous_         = false;
};

}   // namespace path_tracker
