// 版权所有 2026。四轮转向 LQR 横向控制器实现。
#include "path_tracker/lqr.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace path_tracker {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

double LQR::normalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

void LQR::configure(
  double dt,
  double eps,
  int max_iteration,
  double q_lateral,
  double q_heading,
  double r_steer,
  double wheelbase,
  double max_steer_rad,
  double max_speed,
  double min_speed_factor,
  double heading_deadband,
  double heading_max,
  double target_v,
  double min_vel) {
  dt_            = (dt > 0.0) ? dt : 0.1;
  eps_           = (eps > 0.0) ? eps : 1e-6;
  max_iteration_ = (max_iteration > 0) ? max_iteration : 1000;

  Q_       = Eigen::Matrix2d::Zero();
  Q_(0, 0) = std::max(q_lateral, 0.0);
  Q_(1, 1) = std::max(q_heading, 0.0);
  R_       = Eigen::MatrixXd::Zero(1, 1);
  R_(0, 0) = std::max(r_steer, 1e-6);   // 防止矩阵不可逆

  // 轴距必须严格为正：B_d 中含有 v*dt/L，轴距为零或负数会产生 inf/nan。
  wheelbase_     = std::max(wheelbase, 0.05);
  max_steer_rad_ = std::max(max_steer_rad, 0.01);
  max_speed_     = std::max(max_speed, 0.1);

  min_speed_factor_ = std::clamp(min_speed_factor, 0.0, 1.0);
  heading_deadband_ = std::max(heading_deadband, 0.0);
  heading_max_      = std::max(heading_max, heading_deadband_ + 1e-6);
  target_vel_       = std::clamp(target_v, 0.0, max_speed_);
  min_vel_          = std::clamp(min_vel, 0.0, max_speed_);
}

void LQR::setSpeedShaping(double curvature_threshold, double curvature_span, double speed_scale, int lookahead_index, int lookback_start) {
  curvature_threshold_ = std::max(curvature_threshold, 0.0);
  curvature_span_      = std::max(curvature_span, 0.0);
  speed_scale_         = std::clamp(speed_scale, 0.0, 1.0);
  lookahead_index_     = std::max(lookahead_index, 0);
  lookback_start_      = std::max(lookback_start, 0);
}

// 运行时更新等效轴距，供单阿克曼和对称阿克曼切换使用。
void LQR::SetWheelbase(double wheelbase) {
  wheelbase_ = std::max(wheelbase, 0.05);
}

// 控制是否启用曲率和航向误差降速。
void LQR::SetSpeedShapingEnabled(bool enabled) {
  speed_shaping_enabled_ = enabled;
}

// 配置按路径段类型生成的基础速度。
void LQR::SetSegmentSpeedProfile(double work_speed, double turn_speed, double turn_preview_m, double turn_hold_m, bool enabled) {
  work_speed_                    = std::max(work_speed, 0.0);
  turn_speed_                    = std::max(turn_speed, 0.0);
  turn_speed_preview_m_          = std::max(turn_preview_m, 0.0);
  turn_speed_hold_m_             = std::max(turn_hold_m, 0.0);
  segment_speed_profile_enabled_ = enabled;
}

// 判断当前目标点前后给定弧长范围内是否存在掉头点。
bool LQR::HasTurnNear(double before_m, double after_m) const {
  if (path_.empty() || max_pt_idx_ >= path_.size()) {
    return false;
  }
  const double current_s = path_[max_pt_idx_].s;
  const double lower_s   = current_s - std::max(after_m, 0.0);
  const double upper_s   = current_s + std::max(before_m, 0.0);
  for (const TrajectoryPoint& point : path_) {
    if (point.s < lower_s) {
      continue;
    }
    if (point.s > upper_s) {
      break;
    }
    if (point.waypoint_type == 1) {
      return true;
    }
  }
  return false;
}

// 返回当前目标点的路径段类型。
uint8_t LQR::TargetWaypointType() const {
  if (path_.empty() || max_pt_idx_ >= path_.size()) {
    return 0;
  }
  return path_[max_pt_idx_].waypoint_type;
}

// 返回当前跟踪目标索引，用于模式调度和诊断。
size_t LQR::CurrentIndex() const {
  return max_pt_idx_;
}

void LQR::setTrajectory(const std::vector<TrajectoryPoint>& points) {
  path_ = points;
  if (path_.empty()) {
    max_pt_idx_       = 0;
    advance_target_s_ = 0.0;
    return;
  }
  max_pt_idx_       = 0;
  goal_             = path_[0];
  advance_target_s_ = path_[0].s;
}

void LQR::setRobotState(const RobotState& state) {
  robot_state_ = state;
}

void LQR::setSnapWindow(size_t window) {
  snap_window_ = window;
}

double LQR::distToGoal() const {
  return std::hypot(robot_state_.x - goal_.x, robot_state_.y - goal_.y);
}

bool LQR::advanceGoal(double travelled) {
  if (path_.size() < 2) {
    return false;
  }

  // 第一步：按实际行驶距离沿弧长推进目标点。
  // 负位移通常来自里程计异常，因此截断为零，避免目标点回退。
  const double ds = std::max(travelled, 0.0);
  advance_target_s_ += ds;
  while (max_pt_idx_ + 1 < path_.size() && path_[max_pt_idx_ + 1].s <= advance_target_s_) {
    ++max_pt_idx_;
  }

  // 第二步：在有限窗口内向前吸附到最近的路径点。
  //
  // 只依赖弧长累计会完全信任里程计。车辆打滑、被推动或里程计跳变后，
  // 目标点会逐渐偏离车辆。最近点搜索用于重新锚定目标点。
  //
  // 搜索从 max_pt_idx_ 开始，因此索引单调不减。吸附不会将目标点拉回
  // 已完成的路径，避免弓字形作业重复进入上一行。
  if (snap_window_ > 0) {
    const size_t end = std::min(path_.size() - 1, max_pt_idx_ + snap_window_);
    size_t best      = max_pt_idx_;
    double best_d2   = std::numeric_limits<double>::max();
    for (size_t i = max_pt_idx_; i <= end; ++i) {
      const double dx = path_[i].x - robot_state_.x;
      const double dy = path_[i].y - robot_state_.y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        best    = i;
      }
    }
    max_pt_idx_       = best;
    advance_target_s_ = std::max(advance_target_s_, path_[max_pt_idx_].s);
  }

  goal_ = path_[max_pt_idx_];
  return max_pt_idx_ + 1 < path_.size();
}

size_t LQR::nearestIndex() const {
  if (path_.empty()) {
    return 0;
  }
  size_t best    = max_pt_idx_;
  double best_d2 = std::numeric_limits<double>::max();
  // 只从当前目标点向前搜索，使复杂度与剩余路径长度线性相关。
  // 正常运行时车辆不应远落后于目标点。
  for (size_t i = max_pt_idx_; i < path_.size(); ++i) {
    const double dx = path_[i].x - robot_state_.x;
    const double dy = path_[i].y - robot_state_.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best    = i;
    }
  }
  return best;
}

void LQR::calRicatti(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, Eigen::MatrixXd& K) {
  // 算法与 MegaTrak control_node/path_controller/src/lqr.cpp 第 71 至 85 行一致。
  // 定点迭代求解 DARE，与具体被控模型无关，因此可以原样复用。
  // 四轮转向只需要传入不同的 A/B 矩阵。
  Eigen::MatrixXd AT = A.transpose();
  Eigen::MatrixXd BT = B.transpose();
  Eigen::MatrixXd P  = Q_;

  int num_iter = 0;
  double diff  = std::numeric_limits<double>::max();
  while (num_iter++ < max_iteration_ && diff > eps_) {
    Eigen::MatrixXd P_next = Q_ + AT * P * A - (AT * P * B) * (R_ + BT * P * B).inverse() * BT * P * A;
    diff                   = std::fabs((P_next - P).maxCoeff());
    P                      = P_next;
  }

  K = (R_ + BT * P * B).inverse() * BT * P * A;
}

double LQR::scaleSpeed(double base_v, double heading_error, size_t idx) const {
  double v = base_v;

  // 掉头段及其前后保持区使用固定的低速目标。
  if (segment_speed_profile_enabled_) {
    v = std::min(v, work_speed_);
    if (HasTurnNear(turn_speed_preview_m_, turn_speed_hold_m_)) {
      v = std::min(v, turn_speed_);
    }
  }

  if (!speed_shaping_enabled_) {
    return std::clamp(v, min_vel_, max_speed_);
  }

  // 曲率前瞻：在进入急弯前减速，而不是入弯后才减速。
  if (!path_.empty()) {
    const size_t n   = path_.size();
    size_t start     = (idx > static_cast<size_t>(lookback_start_)) ? (idx - static_cast<size_t>(lookback_start_)) : 0;
    size_t end       = std::min(n - 1, idx + static_cast<size_t>(std::max(lookahead_index_, 0)));
    double max_kappa = 0.0;
    for (size_t i = start; i <= end; ++i) {
      max_kappa = std::max(max_kappa, std::fabs(path_[i].kappa));
    }
    if (max_kappa > curvature_threshold_ && curvature_span_ > 0.0) {
      const double ratio  = (max_kappa - curvature_threshold_) / curvature_span_;
      const double factor = 1.0 - std::clamp(ratio, 0.0, 1.0) * (1.0 - speed_scale_);
      v                   = std::min(v, base_v * factor);
    }
  }

  // 航向误差降速：为大误差留出足够的转向修正时间。
  const double he = std::fabs(heading_error);
  if (he > heading_deadband_ && heading_max_ > heading_deadband_) {
    const double ratio  = (he - heading_deadband_) / (heading_max_ - heading_deadband_);
    const double factor = 1.0 - std::clamp(ratio, 0.0, 1.0) * (1.0 - min_speed_factor_);
    v                   = std::min(v, base_v * factor);
  }

  // 跟踪期间不得低于全局速度下限，也不得超过车辆限制。
  v = std::clamp(v, min_vel_, max_speed_);
  return v;
}

LQROutput LQR::getControlOutput() {
  LQROutput output;
  output.target_pt = goal_;

  // 路径点不足时无法跟踪，输出停车。该保护与 MegaTrak lqr.cpp 第 175 至 183 行一致。
  if (path_.size() < 2 || max_pt_idx_ >= path_.size()) {
    output.vel_cmd.linear_v    = 0.0;
    output.vel_cmd.steer_angle = 0.0;
    return output;
  }

  TrajectoryPoint target_pt   = path_[max_pt_idx_];
  const double target_heading = target_pt.heading;
  const double target_kappa   = target_pt.kappa;

  // 在目标点坐标系中计算误差。符号约定为左正，与 CAN 协议的转向角一致，
  // 因此后续无需再取反。
  const double dx            = robot_state_.x - target_pt.x;
  const double dy            = robot_state_.y - target_pt.y;
  const double heading_error = normalizeAngle(robot_state_.heading - target_heading);
  const double lateral_error = dy * std::cos(target_heading) - dx * std::sin(target_heading);

  // 计算本控制周期的目标速度，并按曲率和航向误差进行整形。
  const double v_cmd = scaleSpeed(target_pt.v > 0.0 ? target_pt.v : target_vel_, heading_error, max_pt_idx_);

  // 在指令速度处线性化运动学自行车模型的误差动力学。
  // 使用指令速度而非实测速度，可在车辆短暂停止时保持 B_d 良态。
  const double v = std::max(v_cmd, 1e-3);
  const double L = wheelbase_;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, 2);
  A << 1.0, v * dt_, 0.0, 1.0;
  // 注意：B 与 MegaTrak 的差速驱动模型 [[0],[dt]] 不同。
  // 阿克曼和四轮转向的横摆角速度为 (v/L)*delta。
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(2, 1);
  B << 0.0, (v * dt_) / L;

  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(1, 2);
  calRicatti(A, B, K);

  Eigen::VectorXd X(2);
  X << lateral_error, heading_error;

  // 计算反馈转角。此处 U = -K*x 已经是转角（弧度），
  // 因为阿克曼模型的控制输入是 delta，而不是角速度 omega。
  const Eigen::VectorXd U = -K * X;
  const double steer_fb   = U(0);

  // 定曲率稳态前馈使用精确的自行车模型关系。
  const double steer_ff = std::atan(L * target_kappa);

  double steer = steer_fb + steer_ff;

  // 物理限制：协议允许 ±90 度转角，速度范围为 0～1.4 米/秒。
  steer = std::clamp(steer, -max_steer_rad_, max_steer_rad_);

  output.target_pt           = target_pt;
  output.vel_cmd.linear_v    = v_cmd;
  output.vel_cmd.steer_angle = steer;
  output.lateral_error       = lateral_error;
  output.heading_error       = heading_error;
  output.steer_ff            = steer_ff;
  output.steer_fb            = steer_fb;
  return output;
}

}   // 命名空间 path_tracker
