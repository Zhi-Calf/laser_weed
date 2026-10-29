// 版权所有 2026。四轮转向 LQR 横向控制器实现。
#include "path_tracker/lqr.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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
void LQR::Configure(const LQRConfig& config) {
  config_ = config;
  LQRInputConfig input_config;
  input_config.wheelbase            = config_.wheelbase;
  input_config.pose_at_vehicle_center = config_.pose_at_vehicle_center;
  input_calculator_.Configure(input_config);

  q_       = Eigen::Matrix2d::Zero();
  q_(0, 0) = std::max(config_.q_lateral, 0.0);
  q_(1, 1) = std::max(config_.q_heading, 0.0);
  r_       = Eigen::MatrixXd::Zero(1, 1);
  r_(0, 0) = std::max(config_.r_steer, 1e-6);
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
//   linear_v            : 本拍目标速度 (m/s)
//   effective_wheelbase : 当前模式等效轴距 (m)
//   target_point        : 输出和日志使用的目标点
// 返回:
//   横向、航向误差及各控制分项；前馈使用 atan(L*kappa)
LQROutput LQR::ComputeFeedbackControl(double lateral_error, double heading_error, double target_kappa, double linear_v, double effective_wheelbase, const TrajectoryPoint& target_point) const {
  LQROutput output;
  const double speed = std::max(linear_v, 0.0);
  Eigen::MatrixXd a  = Eigen::MatrixXd::Zero(2, 2);
  a << 1.0, speed * config_.dt, 0.0, 1.0;
  Eigen::MatrixXd b = Eigen::MatrixXd::Zero(2, 1);
  b << 0.0, (speed * config_.dt) / effective_wheelbase;

  Eigen::MatrixXd gain = Eigen::MatrixXd::Zero(1, 2);
  CalRicatti(a, b, gain);
  Eigen::VectorXd error(2);
  error << lateral_error, heading_error;
  const Eigen::VectorXd control = -gain * error;
  const double steer_fb         = control(0);
  const double steer_ff         = std::atan(effective_wheelbase * target_kappa);
  const double steer            = std::clamp(steer_fb + steer_ff, -config_.max_steer_rad, config_.max_steer_rad);

  output.target_pt           = target_point;
  output.vel_cmd.linear_v    = speed;
  output.vel_cmd.steer_angle = steer;
  output.lateral_error       = lateral_error;
  output.heading_error       = heading_error;
  output.steer_ff            = steer_ff;
  output.steer_fb            = steer_fb;
  output.effective_wheelbase = effective_wheelbase;
  return output;
}

// 使用中心最近点确定模式、速度和路径级指标，并以控制点最近点计算 LQR。
// 输入:
//   robot_state         : 车体中心 ENU 位姿
//   symmetric_ackermann : 已由中心最近点选择的转向模式
//   center_match        : 几何中心最近点，提供模式、速度、TURN 距离和终点
//   control_match       : 实际控制点最近点，提供误差、曲率和前馈
// 返回:
//   true = 已生成控制量；false = 匹配无效或已到路径末端
// 输出:
//   output : 目标模式、速度、误差和转角；失败时全部清空
bool LQR::ComputeCommand(const RobotState& robot_state, bool symmetric_ackermann, const PathMatchResult& center_match, const PathMatchResult& control_match, LQROutput& output) {
  output = LQROutput{};
  if (!center_match.success || center_match.at_path_end || !control_match.success) {
    return false;
  }

  const RobotState& control_state  = center_match.control_state;
  const double effective_wheelbase = EffectiveWheelbase(config_.wheelbase, symmetric_ackermann);
  const TrajectoryPoint& target    = control_match.target_point;
  const LQRInputResult input =
      input_calculator_.CalculatePath(control_state, target);
  if (!input.success) {
    return false;
  }
  const double speed               = ScaleSpeed(center_match);
  output                           = ComputeFeedbackControl(input.lateral_error, input.heading_error, input.target_kappa, speed, effective_wheelbase, target);
  output.steer_mode                = symmetric_ackermann ? 1 : 0;
  output.target_index              = control_match.point_index;
  output.distance_to_goal          = std::hypot(robot_state.x - center_match.nearest_point.x, robot_state.y - center_match.nearest_point.y);
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
bool LQR::ComputeLineCommand(const RobotState& robot_state, const LineReference& reference, double commanded_speed_m_s, LQROutput& output) {
  output = LQROutput{};
  if (
    !std::isfinite(commanded_speed_m_s) ||
    commanded_speed_m_s < 0.0) {
    return false;
  }

  // 视觉参考已经位于后轴坐标系；保留 robot_state 以维持既有调用接口。
  static_cast<void>(robot_state);
  const LQRInputResult input =
      input_calculator_.CalculateLine(reference);
  if (!input.success) {
    return false;
  }
  const double effective_wheelbase = EffectiveWheelbase(config_.wheelbase, false);
  TrajectoryPoint target;
  target.kappa      = input.target_kappa;
  output            = ComputeFeedbackControl(input.lateral_error, input.heading_error, input.target_kappa, commanded_speed_m_s, effective_wheelbase, target);
  output.steer_mode = 0;
  return true;
}

}   // namespace path_tracker
