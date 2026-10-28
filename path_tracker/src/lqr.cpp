// 版权所有 2026。四轮转向 LQR 横向控制器实现。
#include "path_tracker/lqr.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

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

}  // namespace

// 使用完整配置结构初始化 LQR 权重和迭代参数。
// 输入:
//   config : LQR 几何、权重、速度和模式切换配置
void LQR::Configure(const LQRConfig& config) {
  config_ = config;
  q_ = Eigen::Matrix2d::Zero();
  q_(0, 0) = std::max(config_.q_lateral, 0.0);
  q_(1, 1) = std::max(config_.q_heading, 0.0);
  r_ = Eigen::MatrixXd::Zero(1, 1);
  r_(0, 0) = std::max(config_.r_steer, 1e-6);
  Reset();
}

// 清除双阿克曼保持状态。
void LQR::Reset() {
  turn_mode_active_ = false;
}

// 判断目标投影点前后给定弧长内是否存在 TURN。
// 输入:
//   match    : 当前连续路径投影
//   before_m : 向前搜索距离 (m)
//   after_m  : 向后保持距离 (m)
// 返回:
//   true = 当前段或给定范围内存在 TURN
bool LQR::HasTurnNear(const PathMatch& match,
                      double before_m,
                      double after_m) const {
  if (match.target_point.waypoint_type == 1) {
    return true;
  }
  constexpr double kArcToleranceM = 1e-9;
  return match.distance_to_next_turn_m <=
             std::max(before_m, 0.0) + kArcToleranceM ||
         match.distance_from_previous_turn_m <=
             std::max(after_m, 0.0) + kArcToleranceM;
}

// 根据路径段和滞回区选择单阿克曼或对称阿克曼。
// 输入:
//   match : 当前连续路径投影
// 返回:
//   true = 对称阿克曼；false = 单阿克曼
bool LQR::SelectSymmetricAckermann(const PathMatch& match) {
  const bool previous_mode = turn_mode_active_;
  if (!turn_mode_active_ &&
      HasTurnNear(match, config_.turn_mode_enter_m, 0.0)) {
    turn_mode_active_ = true;
  } else if (turn_mode_active_ &&
             !HasTurnNear(match,
                          config_.turn_mode_enter_m,
                          config_.turn_mode_exit_m)) {
    turn_mode_active_ = false;
  }

  if (turn_mode_active_ != previous_mode) {
    const double effective_wheelbase =
        EffectiveWheelbase(config_.wheelbase, turn_mode_active_);
    SPDLOG_INFO("转向模式切换为 {}（{}），等效轴距 {:.3f} m",
                turn_mode_active_ ? 1 : 0,
                turn_mode_active_ ? "对称阿克曼" : "单阿克曼",
                effective_wheelbase);
  }
  return turn_mode_active_;
}

// 将车体中心位姿转换为当前转向模式的控制参考点。
// 输入:
//   symmetric_ackermann : true 表示对称阿克曼，false 表示单阿克曼
// 返回:
//   当前控制参考点的位置和航向
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

// 求解离散代数黎卡提方程并得到反馈增益。
// 输入:
//   a / b : 离散状态矩阵和输入矩阵
// 输出:
//   gain : 反馈增益矩阵
void LQR::CalRicatti(const Eigen::MatrixXd& a,
                     const Eigen::MatrixXd& b,
                     Eigen::MatrixXd& gain) const {
  const Eigen::MatrixXd a_transpose = a.transpose();
  const Eigen::MatrixXd b_transpose = b.transpose();
  Eigen::MatrixXd p = q_;
  int iteration = 0;
  double difference = std::numeric_limits<double>::max();
  while (iteration++ < config_.max_iteration && difference > config_.eps) {
    const Eigen::MatrixXd next =
        q_ + a_transpose * p * a -
        (a_transpose * p * b) *
            (r_ + b_transpose * p * b).inverse() * b_transpose * p * a;
    difference = std::fabs((next - p).maxCoeff());
    p = next;
  }
  gain = (r_ + b_transpose * p * b).inverse() *
         b_transpose * p * a;
}

// 根据路径投影计算 WORK/TURN 目标速度。
// 输入:
//   match : 当前连续路径投影
// 返回:
//   TURN 预览和保持区使用低速，其余使用 WORK 速度 (m/s)
double LQR::ScaleSpeed(const PathMatch& match) const {
  if (HasTurnNear(match,
                  config_.turn_speed_preview_m,
                  config_.turn_speed_hold_m)) {
    return config_.turn_speed;
  }
  return config_.work_speed;
}

// 计算指定误差状态和曲率的 LQR 反馈控制量。
// 输入:
//   lateral_error       : 车辆相对路径的横向误差 (m)，左正
//   heading_error       : 车辆航向减路径航向 (rad)
//   target_kappa        : 目标曲率 (1/m)，左正
//   linear_v            : 本拍目标速度 (m/s)
//   effective_wheelbase : 当前模式等效轴距 (m)
//   target_point        : 输出和日志使用的目标点
// 返回:
//   横向、航向误差及各控制分项；前馈使用 atan(L*kappa)
LQROutput LQR::ComputeFeedbackControl(double lateral_error,
                                      double heading_error,
                                      double target_kappa,
                                      double linear_v,
                                      double effective_wheelbase,
                                      const TrajectoryPoint& target_point) const {
  LQROutput output;
  const double speed = std::max(linear_v, 0.0);
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(2, 2);
  a << 1.0, speed * config_.dt, 0.0, 1.0;
  Eigen::MatrixXd b = Eigen::MatrixXd::Zero(2, 1);
  b << 0.0, (speed * config_.dt) / effective_wheelbase;

  Eigen::MatrixXd gain = Eigen::MatrixXd::Zero(1, 2);
  CalRicatti(a, b, gain);
  Eigen::VectorXd error(2);
  error << lateral_error, heading_error;
  const Eigen::VectorXd control = -gain * error;
  const double steer_fb = control(0);
  const double steer_ff = std::atan(effective_wheelbase * target_kappa);
  const double steer = std::clamp(
      steer_fb + steer_ff, -config_.max_steer_rad, config_.max_steer_rad);

  output.target_pt = target_point;
  output.vel_cmd.linear_v = speed;
  output.vel_cmd.steer_angle = steer;
  output.lateral_error = lateral_error;
  output.heading_error = heading_error;
  output.steer_ff = steer_ff;
  output.steer_fb = steer_fb;
  output.effective_wheelbase = effective_wheelbase;
  return output;
}

// 跟踪连续投影后的全局路径。
// 输入:
//   robot_state : 车体中心 ENU 位姿
//   match       : 当前折线投影和 TURN 距离
// 返回:
//   true = 已生成控制量；false = 投影无效或已到路径末端
// 输出:
//   output : 目标模式、速度、误差和转角；失败时全部清空
bool LQR::ComputeCommand(const RobotState& robot_state,
                         const PathMatch& match,
                         LQROutput& output) {
  output = LQROutput{};
  if (!match.valid || match.at_path_end) {
    return false;
  }

  robot_state_ = robot_state;
  const bool symmetric_ackermann = SelectSymmetricAckermann(match);
  const RobotState control_state =
      ControlStateForMode(symmetric_ackermann);
  const double effective_wheelbase =
      EffectiveWheelbase(config_.wheelbase, symmetric_ackermann);
  const TrajectoryPoint& target = match.target_point;
  const double dx = control_state.x - target.x;
  const double dy = control_state.y - target.y;
  const double heading_error =
      NormalizeAngle(control_state.heading - target.heading);
  const double lateral_error =
      dy * std::cos(target.heading) - dx * std::sin(target.heading);
  const double speed = ScaleSpeed(match);
  output = ComputeFeedbackControl(lateral_error,
                                  heading_error,
                                  target.kappa,
                                  speed,
                                  effective_wheelbase,
                                  target);
  output.steer_mode = symmetric_ackermann ? 1 : 0;
  output.target_index = match.segment_index;
  output.distance_to_goal = match.distance_m;
  output.distance_to_turn_m = match.distance_to_next_turn_m;
  return true;
}

// 跟踪视觉中心线的解析直线。
// 输入:
//   robot_state          : 当前车体位姿，仅用于接口一致性
//   reference            : 车体参考点处的中心线截距和航向
//   commanded_speed_m_s  : 沿用全局路径分段的速度 (m/s)
// 返回:
//   true = 已生成单阿克曼控制量；false = 参数非法
// 输出:
//   output : 单阿克曼转角和 LQR 误差；失败时全部清空
bool LQR::ComputeLineCommand(const RobotState& robot_state,
                             const LineReference& reference,
                             double commanded_speed_m_s,
                             LQROutput& output) {
  output = LQROutput{};
  if (!std::isfinite(reference.lateral_error_m) ||
      !std::isfinite(reference.heading_error_rad) ||
      !std::isfinite(reference.curvature) ||
      !std::isfinite(commanded_speed_m_s) ||
      commanded_speed_m_s < 0.0 ||
      std::abs(std::cos(reference.heading_error_rad)) < 1e-9) {
    return false;
  }

  robot_state_ = robot_state;
  const double control_offset =
      config_.pose_at_vehicle_center ? config_.wheelbase * 0.5 : 0.0;
  const double slope = std::tan(reference.heading_error_rad);
  const double path_y_at_control =
      reference.lateral_error_m - slope * control_offset;
  const double lateral_error = -path_y_at_control;
  const double heading_error = -reference.heading_error_rad;
  const double effective_wheelbase =
      EffectiveWheelbase(config_.wheelbase, false);
  TrajectoryPoint target;
  target.kappa = reference.curvature;
  output = ComputeFeedbackControl(lateral_error,
                                  heading_error,
                                  reference.curvature,
                                  commanded_speed_m_s,
                                  effective_wheelbase,
                                  target);
  output.steer_mode = 0;
  return true;
}

}  // namespace path_tracker
