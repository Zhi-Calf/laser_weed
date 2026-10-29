#pragma once

#include <Eigen/Dense>

#include "path_tracker/lqr_input.hpp"

namespace path_tracker {

// 控制器输出的运动指令。
struct VelocityCommand {
  double linear_v    = 0.0;   // 目标速度 (m/s)，不小于 0
  double steer_angle = 0.0;   // 前轴中心等效转角 (rad)，左转为正
};

// LQR 输出的目标状态、误差、前馈和控制量。
struct LQROutput {
  TrajectoryPoint target_pt;          // 当前目标点或默认目标
  VelocityCommand vel_cmd;            // 目标速度和等效转角
  double lateral_error       = 0.0;   // 车辆相对路径，左正 (m)
  double heading_error       = 0.0;   // 车辆航向减路径航向 (rad)
  double steer_ff            = 0.0;   // 曲率前馈转角 (rad)
  double steer_fb            = 0.0;   // LQR 反馈转角 (rad)
  double distance_to_goal    = 0.0;   // 车辆到最近路径点的距离 (m)
  double effective_wheelbase = 0.0;   // 当前模式等效轴距 (m)
  int target_index           = -1;    // 当前目标路径点索引
  int steer_mode             = 0;     // 0=单阿克曼，1=双阿克曼
};

// LQR 的全部静态配置。
struct LQRConfig {
  double dt                   = 0.1;      // 控制周期 (s)
  double eps                  = 1e-6;     // 黎卡提迭代收敛阈值
  int max_iteration           = 1000;     // 黎卡提最大迭代次数
  double q_lateral            = 50.0;     // 横向误差权重
  double q_heading            = 25.0;     // 航向误差权重
  double r_steer              = 25.0;     // 转角控制代价
  double wheelbase            = 1.0;      // 物理轴距 (m)
  double max_steer_rad        = 1.5708;   // 最大转角 (rad)
  double work_speed           = 0.6;      // WORK 段目标速度 (m/s)
  double turn_speed           = 0.3;      // TURN 段目标速度 (m/s)
  bool pose_at_vehicle_center = true;     // 位姿参考点是否为车体中心
};

// 使用离散路径最近点或视觉直线参考的阿克曼 LQR 控制器。
class LQR {
public:
  LQR() = default;

  void Configure(const LQRConfig& config);
  void Reset();
  bool SelectSymmetricAckermann(const PathMatchResult& center_match);
  bool ComputeCommand(const RobotState& robot_state, bool symmetric_ackermann, const PathMatchResult& center_match, const PathMatchResult& control_match, LQROutput& output);
  bool ComputeLineCommand(const RobotState& robot_state, const LineReference& reference, double commanded_speed_m_s, LQROutput& output);

private:
  LQROutput ComputeFeedbackControl(double lateral_error, double heading_error, double target_kappa, double linear_v, double effective_wheelbase, const TrajectoryPoint& target_point) const;
  void CalRicatti(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b, Eigen::MatrixXd& gain) const;
  double ScaleSpeed(const PathMatchResult& match) const;

private:
  LQRConfig config_;                                      // 控制器配置
  LQRInputCalculator input_calculator_;                   // LQR 输入计算器
  Eigen::Matrix2d q_ = Eigen::Matrix2d::Identity();       // 误差权重
  Eigen::MatrixXd r_ = Eigen::MatrixXd::Identity(1, 1);   // 控制权重
};

}   // namespace path_tracker
