// Copyright 2026. LQR lateral controller implementation for four-wheel steering.
#include "lqr_control/lqr.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lqr_control
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

double LQR::normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

void LQR::configure(
  double dt, double eps, int max_iteration,
  double q_lateral, double q_heading, double r_steer,
  double wheelbase, double max_steer_rad, double max_speed,
  double min_speed_factor, double heading_deadband, double heading_max,
  double target_v, double min_vel)
{
  dt_ = (dt > 0.0) ? dt : 0.1;
  eps_ = (eps > 0.0) ? eps : 1e-6;
  max_iteration_ = (max_iteration > 0) ? max_iteration : 1000;

  Q_ = Eigen::Matrix2d::Zero();
  Q_(0, 0) = std::max(q_lateral, 0.0);
  Q_(1, 1) = std::max(q_heading, 0.0);
  R_ = Eigen::MatrixXd::Zero(1, 1);
  R_(0, 0) = std::max(r_steer, 1e-6);  // guard against singular inverse

  // Wheelbase must be strictly positive: B_d has v*dt/L, and a zero/negative L
  // would produce inf/nan steering commands.
  wheelbase_ = std::max(wheelbase, 0.05);
  max_steer_rad_ = std::max(max_steer_rad, 0.01);
  max_speed_ = std::max(max_speed, 0.1);

  min_speed_factor_ = std::clamp(min_speed_factor, 0.0, 1.0);
  heading_deadband_ = std::max(heading_deadband, 0.0);
  heading_max_ = std::max(heading_max, heading_deadband_ + 1e-6);
  target_vel_ = std::clamp(target_v, 0.0, max_speed_);
  min_vel_ = std::clamp(min_vel, 0.0, max_speed_);
}

void LQR::setSpeedShaping(
  double curvature_threshold, double curvature_span, double speed_scale,
  int lookahead_index, int lookback_start)
{
  curvature_threshold_ = std::max(curvature_threshold, 0.0);
  curvature_span_ = std::max(curvature_span, 0.0);
  speed_scale_ = std::clamp(speed_scale, 0.0, 1.0);
  lookahead_index_ = std::max(lookahead_index, 0);
  lookback_start_ = std::max(lookback_start, 0);
}

void LQR::setTrajectory(const std::vector<TrajectoryPoint> & points)
{
  path_ = points;
  if (path_.empty()) {
    max_pt_idx_ = 0;
    return;
  }
  max_pt_idx_ = 0;
  goal_ = path_[0];
}

void LQR::setRobotState(const RobotState & state)
{
  robot_state_ = state;
}

void LQR::setSnapWindow(size_t window)
{
  snap_window_ = window;
}

double LQR::distToGoal() const
{
  return std::hypot(robot_state_.x - goal_.x, robot_state_.y - goal_.y);
}

bool LQR::advanceGoal(double travelled)
{
  if (path_.size() < 2) {
    return false;
  }

  // Step 1: advance along arc length by the distance actually travelled.
  // Negative travel (a bad odometry jump) is clamped to zero so the goal can
  // never be rewound.
  const double ds = std::max(travelled, 0.0);
  const double target_s = path_[max_pt_idx_].s + ds;
  while (max_pt_idx_ + 1 < path_.size() && path_[max_pt_idx_ + 1].s <= target_s) {
    ++max_pt_idx_;
  }

  // Step 2: snap forward to the nearest point within a bounded window.
  //
  // Arc-length accumulation alone trusts the odometry completely: if the robot
  // slips, is pushed, or the odometry glitches, the goal drifts away from the
  // robot and nothing ever pulls it back. The nearest-point search re-anchors it.
  //
  // The search starts AT max_pt_idx_, so the index is monotonically
  // non-decreasing -- snapping can never walk the goal backwards, which would
  // otherwise let a boustrophedon row re-trigger a previous row.
  if (snap_window_ > 0) {
    const size_t end = std::min(path_.size() - 1, max_pt_idx_ + snap_window_);
    size_t best = max_pt_idx_;
    double best_d2 = std::numeric_limits<double>::max();
    for (size_t i = max_pt_idx_; i <= end; ++i) {
      const double dx = path_[i].x - robot_state_.x;
      const double dy = path_[i].y - robot_state_.y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        best = i;
      }
    }
    max_pt_idx_ = best;
  }

  goal_ = path_[max_pt_idx_];
  return max_pt_idx_ + 1 < path_.size();
}

size_t LQR::nearestIndex() const
{
  if (path_.empty()) {
    return 0;
  }
  size_t best = max_pt_idx_;
  double best_d2 = std::numeric_limits<double>::max();
  // Search only forward from the current goal to keep this O(n) over the tail;
  // the robot should not drift far behind the goal in normal operation.
  for (size_t i = max_pt_idx_; i < path_.size(); ++i) {
    const double dx = path_[i].x - robot_state_.x;
    const double dy = path_[i].y - robot_state_.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

void LQR::calRicatti(
  const Eigen::MatrixXd & A, const Eigen::MatrixXd & B, Eigen::MatrixXd & K)
{
  // Verbatim algorithm from MegaTrak control_node/path_controller/src/lqr.cpp:71-85.
  // The fixed-point DARE iteration is independent of the plant model, so it is
  // reused unchanged; only A/B fed into it differ for four-wheel steering.
  Eigen::MatrixXd AT = A.transpose();
  Eigen::MatrixXd BT = B.transpose();
  Eigen::MatrixXd P = Q_;

  int num_iter = 0;
  double diff = std::numeric_limits<double>::max();
  while (num_iter++ < max_iteration_ && diff > eps_) {
    Eigen::MatrixXd P_next =
      Q_ + AT * P * A - (AT * P * B) * (R_ + BT * P * B).inverse() * BT * P * A;
    diff = std::fabs((P_next - P).maxCoeff());
    P = P_next;
  }

  K = (R_ + BT * P * B).inverse() * BT * P * A;
}

double LQR::scaleSpeed(double base_v, double heading_error, size_t idx) const
{
  double v = base_v;

  // Curvature look-ahead: slow down before entering a tight section, not after.
  if (!path_.empty()) {
    const size_t n = path_.size();
    size_t start = (idx > static_cast<size_t>(lookback_start_)) ?
      (idx - static_cast<size_t>(lookback_start_)) : 0;
    size_t end = std::min(n - 1, idx + static_cast<size_t>(std::max(lookahead_index_, 0)));
    double max_kappa = 0.0;
    for (size_t i = start; i <= end; ++i) {
      max_kappa = std::max(max_kappa, std::fabs(path_[i].kappa));
    }
    if (max_kappa > curvature_threshold_ && curvature_span_ > 0.0) {
      const double ratio = (max_kappa - curvature_threshold_) / curvature_span_;
      const double factor = 1.0 - std::clamp(ratio, 0.0, 1.0) * (1.0 - speed_scale_);
      v = std::min(v, target_vel_ * factor);
    }
  }

  // Heading-error slowdown: give the steering time to correct a large error.
  const double he = std::fabs(heading_error);
  if (he > heading_deadband_ && heading_max_ > heading_deadband_) {
    const double ratio = (he - heading_deadband_) / (heading_max_ - heading_deadband_);
    const double factor = 1.0 - std::clamp(ratio, 0.0, 1.0) * (1.0 - min_speed_factor_);
    v = std::min(v, target_vel_ * factor);
  }

  // Never command below the global floor while tracking, and never exceed limits.
  v = std::clamp(v, min_vel_, max_speed_);
  return v;
}

LQROutput LQR::getControlOutput()
{
  LQROutput output;
  output.target_pt = goal_;

  // Too-few-points guard: cannot track, command a stop (mirrors MegaTrak lqr.cpp:175-183).
  if (path_.size() < 2 || max_pt_idx_ >= path_.size()) {
    output.vel_cmd.linear_v = 0.0;
    output.vel_cmd.steer_angle = 0.0;
    return output;
  }

  TrajectoryPoint target_pt = path_[max_pt_idx_];
  const double target_heading = target_pt.heading;
  const double target_kappa = target_pt.kappa;

  // Errors in the target frame. Sign convention: left positive, matching the
  // CAN protocol's steer angle (left positive) so no negation is needed later.
  const double dx = robot_state_.x - target_pt.x;
  const double dy = robot_state_.y - target_pt.y;
  const double heading_error = normalizeAngle(robot_state_.heading - target_heading);
  const double lateral_error = dy * std::cos(target_heading) - dx * std::sin(target_heading);

  // Speed for this tick, shaped by curvature and heading error.
  const double v_cmd = scaleSpeed(target_pt.v > 0.0 ? target_pt.v : target_vel_,
      heading_error, max_pt_idx_);

  // Linearized error dynamics for the kinematic bicycle model, evaluated at the
  // commanded speed (using the commanded, not measured, speed keeps B_d well
  // conditioned even when the robot is momentarily stopped).
  const double v = std::max(v_cmd, 1e-3);
  const double L = wheelbase_;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, 2);
  A << 1.0, v * dt_,
       0.0, 1.0;
  // NOTE: B differs from MegaTrak's differential-drive [[0],[dt]].
  // For Ackermann/four-wheel steering, heading rate = (v/L)*delta.
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(2, 1);
  B << 0.0,
       (v * dt_) / L;

  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(1, 2);
  calRicatti(A, B, K);

  Eigen::VectorXd X(2);
  X << lateral_error, heading_error;

  // Feedback steering angle. U = -K*x is already a steering angle (rad) here,
  // because the control input of the Ackermann model is delta, not omega.
  const Eigen::VectorXd U = -K * X;
  const double steer_fb = U(0);

  // Feedforward steering angle to hold the curve: delta_ff = L * kappa.
  // (MegaTrak used omega_ff = v * kappa because its input was angular rate.)
  const double steer_ff = L * target_kappa;

  double steer = steer_fb + steer_ff;

  // Physical limits: protocol allows +/-90 deg, speed 0..1.4 m/s.
  steer = std::clamp(steer, -max_steer_rad_, max_steer_rad_);

  output.target_pt = target_pt;
  output.vel_cmd.linear_v = v_cmd;
  output.vel_cmd.steer_angle = steer;
  output.lateral_error = lateral_error;
  output.heading_error = heading_error;
  output.steer_ff = steer_ff;
  output.steer_fb = steer_fb;
  return output;
}

}  // namespace lqr_control
