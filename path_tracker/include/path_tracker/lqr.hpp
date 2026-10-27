#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <vector>

#include "path_tracker/discrete_points_math.hpp"

namespace path_tracker {

/// 误差动力学使用的车辆状态。
struct RobotState {
  double x        = 0.0;   // ENU 坐标系 x 坐标（米）
  double y        = 0.0;   // ENU 坐标系 y 坐标（米）
  double heading  = 0.0;   // 车辆航向角（弧度）
  double linear_v = 0.0;   // 实测前向速度（米/秒）
};

/// 控制器输出的运动指令。
struct VelocityCommand {
  double linear_v    = 0.0;   // 目标速度（米/秒），不小于 0
  double steer_angle = 0.0;   // 前轴中心等效转角（弧度），左转为正
};

/// LQR输出
struct LQROutput {
  TrajectoryPoint target_pt;
  VelocityCommand vel_cmd;
  double lateral_error       = 0.0;   // 横向误差 e_y（米），左正，用于日志和调参
  double heading_error       = 0.0;   // 航向误差 e_psi（弧度）
  double steer_ff            = 0.0;   // 前馈转角（弧度）
  double steer_fb            = 0.0;   // 反馈转角（弧度）
  double distance_to_goal    = 0.0;   // 车辆到当前目标点的距离（米）
  double effective_wheelbase = 0.0;   // 当前转向模式使用的等效轴距（米）
  std::size_t target_index   = 0;     // 当前跟踪目标在路径中的索引
  int steer_mode             = 0;     // 当前转向模式，0=单阿克曼，1=对称阿克曼
};

/// 单次控制计算所需的输入。
struct LQRInput {
  RobotState robot_state;
};

/// LQR 的全部静态配置。
struct LQRConfig {
  double dt                       = 0.1;      // 控制周期（秒）
  double eps                      = 1e-6;     // 黎卡提迭代收敛阈值
  int max_iteration               = 1000;     // 黎卡提最大迭代次数
  double q_lateral                = 50.0;     // 横向误差权重
  double q_heading                = 25.0;     // 航向误差权重
  double r_steer                  = 25.0;     // 转角控制代价
  double wheelbase                = 1.0;      // 物理轴距（米）
  double turn_mode_enter_m        = 0.6;      // 进入 TURN 模式的前视距离（米）
  double turn_mode_exit_m         = 0.6;      // 退出 TURN 模式的保持距离（米）
  double max_steer_rad            = 1.5708;   // 最大转角（弧度）
  std::size_t snap_forward_points = 20;       // 从上一目标点向前的搜索点数，必须大于 0
  double work_speed               = 0.6;      // WORK 段目标速度（米/秒）
  double turn_speed               = 0.3;      // TURN 段目标速度（米/秒）
  double turn_speed_preview_m     = 1.0;      // TURN 前预览距离（米）
  double turn_speed_hold_m        = 0.5;      // TURN 后低速保持距离（米）
  bool pose_at_vehicle_center     = true;     // 位姿参考点是否为车体中心
};

/// 运动学自行车（阿克曼）模型的离散 LQR。
/// 调用方必须保证轨迹非空、位姿有效；异常输入由上层停车处理。
///
/// 误差状态 x = [e_y, e_psi]^T，分别表示横向误差和航向误差。
/// 连续模型 A = [[0, v], [0, 0]]，B = [[0], [v/L]]。
/// 离散模型 A_d = I + A*dt = [[1, v*dt], [0, 1]]，
/// B_d = B*dt = [[0], [v*dt/L]]。
///
/// A_d 与 MegaTrak 的差速驱动模型相同，只有 B_d 不同。
/// 曲率为 kappa 的稳态弯道前馈满足：
/// e_psi_dot = (v/L)*tan(delta) - v*kappa = 0，
/// 因此 delta_ff = atan(L * kappa)。
class LQR {
public:
  LQR() = default;

  // 应用控制器配置并初始化权重矩阵。
  void Configure(const LQRConfig& config);

  // 设置待跟踪轨迹并重置目标索引。
  void SetTrajectory(const std::vector<TrajectoryPoint>& points);

  // 计算当前周期的速度、转角和转向模式。
  bool ComputeCommand(const LQRInput& input, LQROutput& output);

private:
  // 根据车辆位置更新跟踪目标点索引。
  void UpdateTargetIndex();

  // 判断车辆是否到达路径末端。
  bool IsAtPathEnd() const;

  // 判断给定距离范围内是否存在掉头段。
  bool HasTurnNear(double before_m, double after_m) const;

  // 判断当前是否应使用对称阿克曼模式。
  bool SelectSymmetricAckermann();

  // 返回当前转向模式对应的控制状态。
  RobotState ControlStateForMode(bool symmetric_ackermann) const;

  // 使用给定等效轴距计算 LQR 反馈控制量。
  LQROutput ComputeFeedbackControl(double effective_wheelbase) const;

  // 求解离散代数黎卡提方程得到反馈增益。
  void CalRicatti(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, Eigen::MatrixXd& K) const;

  // 根据路径段类型计算目标速度。
  double ScaleSpeed() const;

private:
  LQRConfig config_;   // LQR 完整配置
  Eigen::Matrix2d Q_{Eigen::Matrix2d::Identity()};
  Eigen::MatrixXd R_{Eigen::MatrixXd::Identity(1, 1)};

private:
  std::vector<TrajectoryPoint> path_;
  TrajectoryPoint goal_;
  size_t target_idx_{0};
  RobotState robot_state_;
  bool turn_mode_active_{false};
};

}   // namespace path_tracker
