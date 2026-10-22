// 版权所有 2026。四轮转向激光除草机的 LQR 横向控制器。
//
// 本实现源自 MegaTrak 的 path_controller，并针对阿克曼和四轮转向
// 运动学重新推导。具体推导见 README。MegaTrak 的差速驱动 B 矩阵不可复用。
#ifndef PATH_TRACKER__LQR_HPP_
#define PATH_TRACKER__LQR_HPP_

#include <Eigen/Dense>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace path_tracker
{

/// 控制器所需的轨迹点及其几何属性。
/// 数据结构与 MegaTrak 的 TrajectoryPoint 保持一致，便于沿用调参方法。
struct TrajectoryPoint
{
  double x{0.0};        ///< ENU 坐标系 x 坐标（米）
  double y{0.0};        ///< ENU 坐标系 y 坐标（米）
  double heading{0.0};  ///< 路径航向角（弧度）
  double kappa{0.0};    ///< 路径曲率（1/米），左转为正
  double dkappa{0.0};   ///< 曲率变化率 d(kappa)/ds
  double s{0.0};        ///< 累计路径弧长（米）
  double v{0.0};        ///< 该路径点的目标速度（米/秒）
  uint8_t waypoint_type{0};  // 路径段类型，0=作业，1=掉头
};

/// 误差动力学使用的车辆状态。
struct RobotState
{
  double x{0.0};          // ENU 坐标系 x 坐标（米）
  double y{0.0};          // ENU 坐标系 y 坐标（米）
  double heading{0.0};    // 车辆航向角（弧度）
  double linear_v{0.0};   // 实测前向速度（米/秒）
};

/// 控制器输出的运动指令。
struct VelocityCommand
{
  double linear_v{0.0};     // 目标速度（米/秒），不小于 0
  double steer_angle{0.0};  // 前轴中心等效转角（弧度），左转为正
};

struct LQROutput
{
  TrajectoryPoint target_pt;
  VelocityCommand vel_cmd;
  double lateral_error{0.0};   // 横向误差 e_y（米），左正，用于日志和调参
  double heading_error{0.0};   // 航向误差 e_psi（弧度）
  double steer_ff{0.0};        // 前馈转角（弧度）
  double steer_fb{0.0};        // 反馈转角（弧度）
};

/// 运动学自行车（阿克曼）模型的离散 LQR。
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
class LQR
{
public:
  LQR() = default;

  /// 配置权重和求解器限制。Q 为 diag(Q_xy, Q_heading)，R 为标量 R_delta。
  void configure(
    double dt, double eps, int max_iteration,
    double q_lateral, double q_heading, double r_steer,
    double wheelbase, double max_steer_rad, double max_speed,
    double min_speed_factor, double heading_deadband, double heading_max,
    double target_v, double min_vel);

  /// 配置曲率前瞻和速度整形。参数名沿用 MegaTrak 的 path_controller.yaml，
  /// 便于继续参考原有调参方法。
  void setSpeedShaping(
    double curvature_threshold, double curvature_span, double speed_scale,
    int lookahead_index, int lookback_start);

  void SetWheelbase(double wheelbase);

  void SetSpeedShapingEnabled(bool enabled);

  void SetSegmentSpeedProfile(
    double work_speed, double turn_speed, double turn_preview_m,
    double turn_hold_m, bool enabled);

  bool HasTurnNear(double before_m, double after_m) const;

  uint8_t TargetWaypointType() const;

  size_t CurrentIndex() const;

  /// 替换跟踪轨迹。points 必须包含 x/y；调用前应通过 computePathProfile()
  /// 填充 heading、kappa 和 v。
  void setTrajectory(const std::vector<TrajectoryPoint> & points);

  /// 输入最新的定位位姿。
  void setRobotState(const RobotState & state);

  /// 按实测位移推进目标点，并在吸附窗口内向前搜索最近点。
  /// 轨迹耗尽时返回 false，调用方应停车。
  bool advanceGoal(double travelled);

  /// 车辆到当前目标点的欧氏距离（米）。
  /// LQR 误差状态不含沿路径方向的分量，因此该距离是判断车辆是否冲过
  /// 目标点的唯一依据，并用作安全门控。
  double distToGoal() const;

  /// advanceGoal() 的前向吸附窗口，单位为路径点数。
  /// 设为 0 时关闭吸附，仅使用弧长累计。
  void setSnapWindow(size_t window);

  /// 执行一次控制计算，输出速度和转角。
  LQROutput getControlOutput();

  /// 返回离车辆最近的轨迹索引，用于可视化。
  size_t nearestIndex() const;

private:
  /// 使用定点迭代求解离散代数黎卡提方程。
  /// 算法与 MegaTrak lqr.cpp 第 71 至 85 行一致，不依赖具体车辆模型。
  void calRicatti(const Eigen::MatrixXd & A, const Eigen::MatrixXd & B, Eigen::MatrixXd & K);

  /// 根据曲率和航向误差调整目标速度。
  double scaleSpeed(double base_v, double heading_error, size_t idx) const;

  static double normalizeAngle(double angle);

  // 求解器参数
  double dt_{0.1};
  double eps_{1e-6};
  int max_iteration_{1000};
  Eigen::Matrix2d Q_{Eigen::Matrix2d::Identity()};
  Eigen::MatrixXd R_{Eigen::MatrixXd::Identity(1, 1)};

  // 车辆限制
  double wheelbase_{1.0};         // 当前转向模式下的等效轴距（米）
  double max_steer_rad_{1.5708};  // CAN 协议允许的 ±90 度
  double max_speed_{1.4};         // 协议速度范围 0～1400 毫米/秒

  // 速度整形
  double min_speed_factor_{0.3};
  double heading_deadband_{0.1};
  double heading_max_{1.0};
  double target_vel_{0.5};
  double min_vel_{0.1};
  bool speed_shaping_enabled_{true};

  // 路径段速度
  bool segment_speed_profile_enabled_{false};
  double work_speed_{0.6};
  double turn_speed_{0.3};
  double turn_speed_preview_m_{1.0};
  double turn_speed_hold_m_{0.5};

  // 曲率前瞻
  double curvature_threshold_{0.2};
  double curvature_span_{0.5};
  double speed_scale_{0.4};
  int lookahead_index_{30};
  int lookback_start_{20};

  // 运行时状态
  std::vector<TrajectoryPoint> path_;
  TrajectoryPoint goal_;
  size_t max_pt_idx_{0};
  double advance_target_s_{0.0};  // 累计目标弧长，保留不足一个路径点的位移
  // advanceGoal() 的前向吸附窗口，单位为路径点数
  size_t snap_window_{0};
  RobotState robot_state_;
};

}  // 命名空间 path_tracker

#endif  // PATH_TRACKER__LQR_HPP_
