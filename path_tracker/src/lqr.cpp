// 版权所有 2026。四轮转向 LQR 横向控制器实现。
#include "path_tracker/lqr.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "path_tracker/lqr_input.hpp"

namespace path_tracker {
namespace {

// 对称阿克曼的等效轴距为物理轴距的一半。
double EffectiveWheelbase(double wheelbase, bool symmetric_ackermann) {
  const double effective = symmetric_ackermann ? wheelbase * 0.5 : wheelbase;
  return std::max(effective, 0.05);
}

}   // namespace

// 使用完整配置结构初始化 LQR 权重和迭代参数。
// 输入:
//   config : LQR 几何、权重、速度和模式切换配置
// 副作用:
//   保存归一化配置、打印配置日志并重置控制器状态
void LQR::Configure(const LQRConfig& config) {
  config_  = config;
  q_       = Eigen::Matrix2d::Zero();
  q_(0, 0) = std::max(config_.q_lateral, 0.0);
  q_(1, 1) = std::max(config_.q_heading, 0.0);
  r_       = Eigen::MatrixXd::Zero(1, 1);
  r_(0, 0) = std::max(config_.r_steer, 1e-6);
  SPDLOG_INFO("LQR 求解配置: dt={:.3f}s eps={:g} max_iteration={}", config_.dt, config_.eps, config_.max_iteration);
  SPDLOG_INFO("LQR 权重配置: Q_lateral={:.3f} Q_heading={:.3f} R_steer={:.3f}", config_.q_lateral, config_.q_heading, config_.r_steer);
  SPDLOG_INFO(
    "LQR 几何限幅: wheelbase={:.3f}m max_steer={:.3f}rad "
    "pose_at_center={}",
    config_.wheelbase,
    config_.max_steer_rad,
    config_.pose_at_vehicle_center);
  SPDLOG_INFO("LQR 速度配置: work={:.3f}m/s turn={:.3f}m/s", config_.work_speed, config_.turn_speed);
  Reset();
}

// 重置控制器；工作模式滞回由 GlobalPathLocator 持有。
void LQR::Reset() {}

// 根据定位器工作模式选择单阿克曼或对称阿克曼。
// 输入:
//   center_match : 几何中心当前最近点
// 返回:
//   true = 对称阿克曼；false = 单阿克曼
bool LQR::SelectSymmetricAckermann(const PathMatchResult& center_match) {
  return center_match.work_mode == PathWorkMode::kTurn;
}

// 求解离散代数黎卡提方程并得到反馈增益。
// 输入:
//   a / b : 离散状态矩阵和输入矩阵
// 输出:
//   gain : 反馈增益矩阵
void LQR::CalRicatti(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b, Eigen::MatrixXd& gain) const {
  const Eigen::MatrixXd a_transpose = a.transpose();
  const Eigen::MatrixXd b_transpose = b.transpose();
  Eigen::MatrixXd p                 = q_;
  int iteration                     = 0;
  double difference                 = std::numeric_limits<double>::max();
  while (iteration++ < config_.max_iteration && difference > config_.eps) {
    const Eigen::MatrixXd next = q_ + a_transpose * p * a - (a_transpose * p * b) * (r_ + b_transpose * p * b).inverse() * b_transpose * p * a;
    difference                 = std::fabs((next - p).maxCoeff());
    p                          = next;
  }
  gain = (r_ + b_transpose * p * b).inverse() * b_transpose * p * a;
}

// 根据几何中心路径最近点计算 WORK/TURN 目标速度。
// 输入:
//   match : 几何中心当前最近点
// 返回:
//   TURN 预览和保持区使用低速，其余使用 WORK 速度 (m/s)
double LQR::ScaleSpeed(const PathMatchResult& match) const {
  if (match.work_mode == PathWorkMode::kTurn) {
    return config_.turn_speed;
  }
  return config_.work_speed;
}

// 计算指定误差状态和曲率的 LQR 反馈控制量。
// 输入:
//   lateral_error       : 车辆相对路径的横向误差 (m)，左正
//   heading_error       : 车辆航向减路径航向 (rad)
//   target_kappa        : 目标曲率 (1/m)，左正
//   commanded_speed_m_s : 本拍输出速度指令 (m/s)
//   gain_speed_m_s      : 用于 A/B 矩阵的实测车速 (m/s)
//   effective_wheelbase : 当前模式等效轴距 (m)
// 返回:
//   横向、航向误差及各控制分项；前馈使用 atan(L*kappa)
LQROutput LQR::ComputeFeedbackControl(double lateral_error, double heading_error, double target_kappa, double commanded_speed_m_s, double gain_speed_m_s, double effective_wheelbase) const {
  LQROutput output;
  const double commanded_speed = std::max(commanded_speed_m_s, 0.0);
  const double gain_speed      = std::max(gain_speed_m_s, 0.0);
  const double speed_period    = gain_speed * config_.dt;
  Eigen::MatrixXd a            = Eigen::MatrixXd::Zero(2, 2);
  a << 1.0, speed_period, 0.0, 1.0;

  // 连续模型为零阶保持时，转角在本周期内作用于航向，
  // 并通过加速度积分产生 v^2*dt^2/(2L) 的直接横向位移。
  Eigen::MatrixXd b             = Eigen::MatrixXd::Zero(2, 1);
  const double lateral_response = speed_period * speed_period / (2.0 * effective_wheelbase);
  b << lateral_response, speed_period / effective_wheelbase;

  Eigen::MatrixXd gain = Eigen::MatrixXd::Zero(1, 2);
  CalRicatti(a, b, gain);
  Eigen::VectorXd error(2);
  error << lateral_error, heading_error;
  const Eigen::VectorXd control = -gain * error;
  const double steer_fb         = control(0);
  const double steer_ff         = std::atan(effective_wheelbase * target_kappa);
  const double steer            = std::clamp(steer_fb + steer_ff, -config_.max_steer_rad, config_.max_steer_rad);
  output.vel_cmd.linear_v       = commanded_speed;
  output.vel_cmd.steer_angle    = steer;
  output.lateral_error          = lateral_error;
  output.heading_error          = heading_error;
  output.steer_ff               = steer_ff;
  output.steer_fb               = steer_fb;
  output.effective_wheelbase    = effective_wheelbase;
  output.success                = true;
  return output;
}

// 使用已校验的路径误差输入、统一定位结果和车体状态计算 LQR。
// 输入:
//   robot_state         : 车体中心 ENU 位姿
//   match               : 最近点、工作模式、控制状态、目标点和终点信息
//   input               : 已计算并完成安全门控的路径误差输入
// 返回:
//   控制量；失败时 success 为 false
LQROutput LQR::ComputeCommand(
    const RobotState& robot_state,
    const PathMatchResult& match,
    const LQRInputResult& input) {
  if (!match.success || match.at_path_end || !input.success) {
    return LQROutput{};
  }
  if (!std::isfinite(robot_state.linear_v)) {
    return LQROutput{};
  }

  const bool symmetric_ackermann   = SelectSymmetricAckermann(match);
  const double effective_wheelbase = EffectiveWheelbase(config_.wheelbase, symmetric_ackermann);
  const TrajectoryPoint& target    = match.target_point;
  const double speed      = ScaleSpeed(match);
  const double gain_speed = std::max(robot_state.linear_v, 0.0);
  LQROutput output        = ComputeFeedbackControl(input.lateral_error, input.heading_error, input.target_kappa, speed, gain_speed, effective_wheelbase);
  output.target_pt        = target;
  output.steer_mode       = symmetric_ackermann ? 1 : 0;
  output.target_index     = match.point_index;
  output.nearest_path_distance_m = match.nearest_distance_m;
  return output;
}

// 使用已校验的视觉直线误差输入计算单阿克曼 LQR。
// 输入:
//   robot_state          : 当前车体位姿，用于实测车速
//   input                : 已计算并完成安全门控的视觉误差输入
//   commanded_speed_m_s  : 沿用全局路径分段的速度 (m/s)
// 返回:
//   单阿克曼控制量；失败时 success 为 false
LQROutput LQR::ComputeLineCommand(
    const RobotState& robot_state,
    const LQRInputResult& input,
    double commanded_speed_m_s) {
  if (!input.success ||
      !std::isfinite(commanded_speed_m_s) ||
      commanded_speed_m_s < 0.0) {
    return LQROutput{};
  }
  if (!std::isfinite(robot_state.linear_v)) {
    return LQROutput{};
  }

  const double effective_wheelbase = EffectiveWheelbase(config_.wheelbase, false);
  TrajectoryPoint target;
  target.kappa            = input.target_kappa;
  const double gain_speed = std::max(robot_state.linear_v, 0.0);
  LQROutput output        = ComputeFeedbackControl(input.lateral_error, input.heading_error, input.target_kappa, commanded_speed_m_s, gain_speed, effective_wheelbase);
  output.target_pt        = target;
  output.steer_mode       = 0;
  return output;
}

}   // namespace path_tracker
