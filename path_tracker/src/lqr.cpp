// 版权所有 2026。四轮转向 LQR 横向控制器实现。
#include "path_tracker/lqr.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace path_tracker {

namespace {

// 将角度归一化到 [-pi, pi]。
double NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

// 对称阿克曼的等效轴距为物理轴距的一半。
double EffectiveWheelbase(double wheelbase, bool symmetric_ackermann) {
  const double effective = symmetric_ackermann ? wheelbase * 0.5 : wheelbase;
  return std::max(effective, 0.05);
}

}   // namespace

// 使用完整配置结构初始化 LQR。
void LQR::Configure(const LQRConfig& config) {
  config_  = config;
  Q_       = Eigen::Matrix2d::Zero();
  Q_(0, 0) = std::max(config_.q_lateral, 0.0);
  Q_(1, 1) = std::max(config_.q_heading, 0.0);
  R_       = Eigen::MatrixXd::Zero(1, 1);
  R_(0, 0) = std::max(config_.r_steer, 1e-6);   // 防止矩阵不可逆
}

// 更新路径
void LQR::SetTrajectory(const std::vector<TrajectoryPoint>& points) {
  path_             = points;
  turn_mode_active_ = false;
  target_idx_       = 0;
  goal_             = path_[0];
}

// 将固定车体中心位姿转换为当前转向模型的控制参考点。
RobotState LQR::ControlStateForMode(bool symmetric_ackermann) const {
  RobotState control_state = robot_state_;
  if (!config_.pose_at_vehicle_center || symmetric_ackermann) {
    return control_state;
  }

  const double half_wheelbase = config_.wheelbase * 0.5;
  control_state.x -= half_wheelbase * std::cos(control_state.heading);
  control_state.y -= half_wheelbase * std::sin(control_state.heading);
  return control_state;
}

// 更新最近点
void LQR::UpdateTargetIndex() {
  const size_t search_start_idx = target_idx_;
  const size_t end              = std::min(path_.size() - 1, search_start_idx + config_.snap_forward_points);
  size_t best                   = search_start_idx;
  double best_d2                = std::numeric_limits<double>::max();
  for (size_t i = search_start_idx; i <= end; ++i) {
    const double dx = path_[i].x - robot_state_.x;
    const double dy = path_[i].y - robot_state_.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best    = i;
    }
  }
  target_idx_ = best;
  goal_       = path_[target_idx_];
}

// 当前目标已经是路径最后一个点时，表示路径已结束。
bool LQR::IsAtPathEnd() const {
  return target_idx_ + 1 >= path_.size();
}

// 判断当前目标点前后给定弧长范围内是否存在掉头点。
bool LQR::HasTurnNear(double before_m, double after_m) const {
  const double current_s = path_[target_idx_].s;
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

// 更新里卡提方程
void LQR::CalRicatti(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, Eigen::MatrixXd& K) const {
  Eigen::MatrixXd AT = A.transpose();
  Eigen::MatrixXd BT = B.transpose();
  Eigen::MatrixXd P  = Q_;
  int num_iter       = 0;
  double diff        = std::numeric_limits<double>::max();
  while (num_iter++ < config_.max_iteration && diff > config_.eps) {
    Eigen::MatrixXd P_next = Q_ + AT * P * A - (AT * P * B) * (R_ + BT * P * B).inverse() * BT * P * A;
    diff                   = std::fabs((P_next - P).maxCoeff());
    P                      = P_next;
  }
  K = (R_ + BT * P * B).inverse() * BT * P * A;
}

// 计算路径分段对应的基础速度。
double LQR::ScaleSpeed() const {
  double v = config_.work_speed;
  if (HasTurnNear(config_.turn_speed_preview_m, config_.turn_speed_hold_m)) {
    v = config_.turn_speed;
  }
  return v;
}

// 计算反馈控制量
LQROutput LQR::ComputeFeedbackControl(double effective_wheelbase) const {
  LQROutput output;
  output.target_pt = goal_;

  TrajectoryPoint target_pt   = path_[target_idx_];
  const double target_heading = target_pt.heading;
  const double target_kappa   = target_pt.kappa;

  // 在目标点坐标系中计算误差。符号约定为左正，与 CAN 协议的转向角一致，
  // 因此后续无需再取反。
  const double dx            = robot_state_.x - target_pt.x;
  const double dy            = robot_state_.y - target_pt.y;
  const double heading_error = NormalizeAngle(robot_state_.heading - target_heading);
  const double lateral_error = dy * std::cos(target_heading) - dx * std::sin(target_heading);

  // 目标速度只由路径分段和全局速度限幅决定。
  const double v_cmd = ScaleSpeed();

  // 在指令速度处线性化运动学自行车模型的误差动力学。
  // 使用指令速度而非实测速度，可在车辆短暂停止时保持 B_d 良态。
  const double v = v_cmd;
  const double L = effective_wheelbase;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, 2);
  A << 1.0, v * config_.dt, 0.0, 1.0;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(2, 1);
  B << 0.0, (v * config_.dt) / L;

  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(1, 2);
  CalRicatti(A, B, K);

  Eigen::VectorXd X(2);
  X << lateral_error, heading_error;

  // 计算反馈转角。此处 U = -K*x 已经是转角（弧度），因为阿克曼模型的控制输入是 delta，而不是角速度 omega。
  const Eigen::VectorXd U = -K * X;
  const double steer_fb   = U(0);

  // 定曲率稳态前馈使用精确的自行车模型关系。
  const double steer_ff = std::atan(L * target_kappa);

  double steer = steer_fb + steer_ff;

  // 物理限制：协议允许 ±90 度转角，速度范围为 0～1.4 米/秒。
  steer = std::clamp(steer, -config_.max_steer_rad, config_.max_steer_rad);

  output.target_pt           = target_pt;
  output.vel_cmd.linear_v    = v_cmd;
  output.vel_cmd.steer_angle = steer;
  output.lateral_error       = lateral_error;
  output.heading_error       = heading_error;
  output.steer_ff            = steer_ff;
  output.steer_fb            = steer_fb;
  return output;
}

// 根据路径段类型和滞回区选择当前转向模式。
bool LQR::SelectSymmetricAckermann() {
  const bool previous_mode = turn_mode_active_;
  if (!turn_mode_active_ && HasTurnNear(config_.turn_mode_enter_m, 0.0)) {
    turn_mode_active_ = true;
  } else if (turn_mode_active_ && !HasTurnNear(config_.turn_mode_enter_m, config_.turn_mode_exit_m)) {
    turn_mode_active_ = false;
  }

  if (turn_mode_active_ != previous_mode) {
    const double effective_wheelbase = EffectiveWheelbase(config_.wheelbase, turn_mode_active_);
    SPDLOG_INFO("转向模式切换为 {}（{}），等效轴距 {:.3f} m", turn_mode_active_ ? 1 : 0, turn_mode_active_ ? "对称阿克曼" : "单阿克曼", effective_wheelbase);
  }
  return turn_mode_active_;
}

// 完成一次路径推进、转向模式选择和控制量计算。
bool LQR::ComputeCommand(const LQRInput& input, LQROutput& output) {
  output       = LQROutput{};
  robot_state_ = input.robot_state;
  UpdateTargetIndex();
  if (IsAtPathEnd()) {
    return false;
  }

  const bool symmetric_ackermann   = SelectSymmetricAckermann();
  robot_state_                     = ControlStateForMode(symmetric_ackermann);
  const double effective_wheelbase = EffectiveWheelbase(config_.wheelbase, symmetric_ackermann);
  output                           = ComputeFeedbackControl(effective_wheelbase);
  output.steer_mode                = symmetric_ackermann ? 1 : 0;
  output.effective_wheelbase       = effective_wheelbase;
  output.target_index              = target_idx_;
  output.distance_to_goal          = std::hypot(input.robot_state.x - goal_.x, input.robot_state.y - goal_.y);
  return true;
}

}   // namespace path_tracker
