#pragma once

#include <cstddef>
#include <limits>

#include <Eigen/Dense>

#include "path_tracker/discrete_points_math.hpp"
#include "path_tracker/global_path_locator.hpp"

namespace path_tracker {

// 误差动力学使用的车辆状态。
struct RobotState {
  double x = 0.0;         // ENU 坐标系 x 坐标 (m)
  double y = 0.0;         // ENU 坐标系 y 坐标 (m)
  double heading = 0.0;   // 车辆航向角 (rad)
  double linear_v = 0.0;  // 实测前向速度 (m/s)
};

// 控制器输出的运动指令。
struct VelocityCommand {
  double linear_v = 0.0;     // 目标速度 (m/s)，不小于 0
  double steer_angle = 0.0;  // 前轴中心等效转角 (rad)，左转为正
};

// 视觉中心线在车体参考点处的解析直线参数。
struct LineReference {
  double lateral_error_m = 0.0;    // x=0 截距，路径在左为正
  double heading_error_rad = 0.0;  // 路径相对车体航向，向左为正
  double curvature = 0.0;          // 路径曲率 (1/m)，左转为正
};

// LQR 输出的目标状态、误差、前馈和控制量。
struct LQROutput {
  TrajectoryPoint target_pt;     // 当前目标点或默认目标
  VelocityCommand vel_cmd;       // 目标速度和等效转角
  double lateral_error = 0.0;    // 车辆相对路径，左正 (m)
  double heading_error = 0.0;    // 车辆航向减路径航向 (rad)
  double steer_ff = 0.0;         // 曲率前馈转角 (rad)
  double steer_fb = 0.0;         // LQR 反馈转角 (rad)
  double distance_to_goal = 0.0; // 车辆到投影点的距离 (m)
  double distance_to_turn_m =
      std::numeric_limits<double>::infinity();  // 到下一 TURN 弧长 (m)
  double effective_wheelbase = 0.0;  // 当前模式等效轴距 (m)
  std::size_t target_index = 0;      // 当前目标线段索引
  int steer_mode = 0;                // 0=单阿克曼，1=双阿克曼
};

// LQR 的全部静态配置。
struct LQRConfig {
  double dt = 0.1;                     // 控制周期 (s)
  double eps = 1e-6;                   // 黎卡提迭代收敛阈值
  int max_iteration = 1000;            // 黎卡提最大迭代次数
  double q_lateral = 50.0;             // 横向误差权重
  double q_heading = 25.0;             // 航向误差权重
  double r_steer = 25.0;               // 转角控制代价
  double wheelbase = 1.0;              // 物理轴距 (m)
  double turn_mode_enter_m = 0.6;      // 进入 TURN 模式前视距离 (m)
  double turn_mode_exit_m = 0.6;       // 退出 TURN 模式保持距离 (m)
  double max_steer_rad = 1.5708;       // 最大转角 (rad)
  double work_speed = 0.6;             // WORK 段目标速度 (m/s)
  double turn_speed = 0.3;             // TURN 段目标速度 (m/s)
  double turn_speed_preview_m = 1.0;   // TURN 前预览距离 (m)
  double turn_speed_hold_m = 0.5;      // TURN 后低速保持距离 (m)
  bool pose_at_vehicle_center = true;  // 位姿参考点是否为车体中心
};

// 使用离散路径投影或视觉直线参考的阿克曼 LQR 控制器。
class LQR {
 public:
  LQR() = default;

  void Configure(const LQRConfig& config);
  void Reset();
  // 跟踪全局路径投影。
  bool ComputeCommand(const RobotState& robot_state,
                      const PathMatch& match,
                      LQROutput& output);
  // 跟踪车体坐标下的视觉直线。
  bool ComputeLineCommand(const RobotState& robot_state,
                          const LineReference& reference,
                          double commanded_speed_m_s,
                          LQROutput& output);

 private:
  // 判断投影点附近是否进入 TURN 模式区。
  bool HasTurnNear(const PathMatch& match,
                   double before_m,
                   double after_m) const;
  // 按路径段和滞回区选择转向模式。
  bool SelectSymmetricAckermann(const PathMatch& match);
  // 将车体中心位姿换算为当前模式控制参考点。
  RobotState ControlStateForMode(bool symmetric_ackermann) const;
  // 由误差状态、曲率和速度计算统一反馈与前馈。
  LQROutput ComputeFeedbackControl(double lateral_error,
                                   double heading_error,
                                   double target_kappa,
                                   double linear_v,
                                   double effective_wheelbase,
                                   const TrajectoryPoint& target_point) const;
  // 迭代求解离散黎卡提方程。
  void CalRicatti(const Eigen::MatrixXd& a,
                  const Eigen::MatrixXd& b,
                  Eigen::MatrixXd& gain) const;
  // 根据 TURN 预览区选择路径分段速度。
  double ScaleSpeed(const PathMatch& match) const;

 private:
  LQRConfig config_;                                     // 控制器配置
  Eigen::Matrix2d q_ = Eigen::Matrix2d::Identity();       // 误差权重
  Eigen::MatrixXd r_ = Eigen::MatrixXd::Identity(1, 1);   // 控制权重
  RobotState robot_state_;                               // 本拍车辆状态
  bool turn_mode_active_ = false;  // 是否保持双阿克曼模式
};

}  // namespace path_tracker
