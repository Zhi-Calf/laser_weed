// Copyright 2026. LQR lateral controller for the four-wheel-steering laser weeder.
//
// Derived from MegaTrak path_controller (control_node/path_controller/src/lqr.cpp),
// re-derived for Ackermann / four-wheel steering kinematics. See README for the
// derivation. The differential-drive B matrix of MegaTrak is NOT reusable.
#ifndef LQR_CONTROL__LQR_HPP_
#define LQR_CONTROL__LQR_HPP_

#include <Eigen/Dense>

#include <vector>

namespace lqr_control
{

/// One trajectory point with the geometric attributes needed by the controller.
/// Mirrors MegaTrak's TrajectoryPoint so the tuning playbook stays applicable.
struct TrajectoryPoint
{
  double x{0.0};        ///< ENU x (m)
  double y{0.0};        ///< ENU y (m)
  double heading{0.0};  ///< path heading (rad)
  double kappa{0.0};    ///< path curvature (1/m), left positive
  double dkappa{0.0};   ///< curvature rate d(kappa)/ds
  double s{0.0};        ///< accumulated arc length (m)
  double v{0.0};        ///< desired speed at this point (m/s)
};

/// Vehicle state used by the error dynamics.
struct RobotState
{
  double x{0.0};
  double y{0.0};
  double heading{0.0};
  double linear_v{0.0};  ///< measured forward speed (m/s)
};

/// Command produced by the controller.
struct VelocityCommand
{
  double linear_v{0.0};   ///< target speed (m/s), >= 0
  double steer_angle{0.0};  ///< front-wheel steering angle (rad), left positive
};

struct LQROutput
{
  TrajectoryPoint target_pt;
  VelocityCommand vel_cmd;
  double lateral_error{0.0};   ///< e_y (m), left positive - for logging/tuning
  double heading_error{0.0};   ///< e_psi (rad)
  double steer_ff{0.0};        ///< feedforward part of the steering angle (rad)
  double steer_fb{0.0};        ///< feedback part of the steering angle (rad)
};

/// Discrete LQR for the kinematic bicycle (Ackermann) model.
///
/// Error state  x = [e_y, e_psi]^T   (lateral error, heading error)
/// Continuous   A = [[0, v], [0, 0]],  B = [[0], [v/L]]
/// Discrete     A_d = I + A*dt = [[1, v*dt], [0, 1]]
///              B_d = B*dt     = [[0], [v*dt/L]]
///
/// Note A_d is identical to MegaTrak's differential-drive A_d; only B_d differs.
/// Feedforward at steady state on a curve of curvature kappa:
///   e_psi_dot = (v/L)*delta - v*kappa = 0  =>  delta_ff = L * kappa
class LQR
{
public:
  LQR() = default;

  /// Configure weights and solver limits. Q is diag(Q_xy, Q_heading); R is scalar R_delta.
  void configure(
    double dt, double eps, int max_iteration,
    double q_lateral, double q_heading, double r_steer,
    double wheelbase, double max_steer_rad, double max_speed,
    double min_speed_factor, double heading_deadband, double heading_max,
    double target_v, double min_vel);

  /// Curvature look-ahead and speed shaping. Parameter names mirror MegaTrak's
  /// path_controller.yaml so the existing tuning playbook still applies.
  void setSpeedShaping(
    double curvature_threshold, double curvature_span, double speed_scale,
    int lookahead_index, int lookback_start);

  /// Replace the tracked trajectory. `points` must carry x/y; heading/kappa/v
  /// are filled in by computePathProfile() before this call.
  void setTrajectory(const std::vector<TrajectoryPoint> & points);

  /// Feed the latest localization (RTK odometry).
  void setRobotState(const RobotState & state);

  /// Advance the tracked goal point by the measured distance, then snap forward
  /// to the nearest point within the snap window. Returns false if the
  /// trajectory is exhausted (caller should stop).
  bool advanceGoal(double travelled);

  /// Euclidean distance from the robot to the currently tracked goal (m).
  /// The LQR error state has no along-track component, so this is the caller's
  /// only handle on "the robot has run past the goal". Used as a safety gate.
  double distToGoal() const;

  /// Forward-snap window for advanceGoal(), in path points. 0 disables snapping
  /// and falls back to pure arc-length accumulation.
  void setSnapWindow(size_t window);

  /// One control tick. Produces speed + steering angle.
  LQROutput getControlOutput();

  /// Nearest trajectory index to the robot, for visualization.
  size_t nearestIndex() const;

private:
  /// Solve the discrete algebraic Riccati equation by fixed-point iteration.
  /// Algorithm copied verbatim from MegaTrak lqr.cpp:71-85 (model-independent).
  void calRicatti(const Eigen::MatrixXd & A, const Eigen::MatrixXd & B, Eigen::MatrixXd & K);

  /// Apply curvature- and heading-error-based speed scaling.
  double scaleSpeed(double base_v, double heading_error, size_t idx) const;

  static double normalizeAngle(double angle);

  // Solver
  double dt_{0.1};
  double eps_{1e-6};
  int max_iteration_{1000};
  Eigen::Matrix2d Q_{Eigen::Matrix2d::Identity()};
  Eigen::MatrixXd R_{Eigen::MatrixXd::Identity(1, 1)};

  // Vehicle limits
  double wheelbase_{1.0};      ///< effective wheelbase for the active steering mode (m)
  double max_steer_rad_{1.5708};  ///< +/-90 deg per CAN protocol
  double max_speed_{1.4};      ///< protocol range 0..1400 mm/s

  // Speed shaping
  double min_speed_factor_{0.3};
  double heading_deadband_{0.1};
  double heading_max_{1.0};
  double target_vel_{0.5};
  double min_vel_{0.1};

  // Curvature look-ahead
  double curvature_threshold_{0.2};
  double curvature_span_{0.5};
  double speed_scale_{0.4};
  int lookahead_index_{30};
  int lookback_start_{20};

  // Runtime
  std::vector<TrajectoryPoint> path_;
  TrajectoryPoint goal_;
  size_t max_pt_idx_{0};
  /// Forward-snap window for advanceGoal(), in path points.
  size_t snap_window_{0};
  RobotState robot_state_;
};

}  // namespace lqr_control

#endif  // LQR_CONTROL__LQR_HPP_
