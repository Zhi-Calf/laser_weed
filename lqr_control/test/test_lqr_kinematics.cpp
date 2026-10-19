// Copyright 2026. Unit tests for the four-wheel-steering LQR.
//
// These tests are the safety net for the port: they pin down the derivation
// claims made in the README, in particular that the feedforward steering angle
// equals L*kappa (not v*kappa as in MegaTrak's differential-drive model).
#include "lqr_control/lqr.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using lqr_control::LQR;
using lqr_control::RobotState;
using lqr_control::TrajectoryPoint;

namespace
{

/// Build a controller with the documented defaults.
LQR makeController(double wheelbase = 1.2)
{
  LQR lqr;
  lqr.configure(
    /*dt*/ 0.1, /*eps*/ 1e-6, /*max_iter*/ 1000,
    /*q_lat*/ 50.0, /*q_head*/ 25.0, /*r_steer*/ 25.0,
    wheelbase, /*max_steer_rad*/ 90.0 * M_PI / 180.0, /*max_speed*/ 1.4,
    /*min_speed_factor*/ 0.3, /*heading_deadband*/ 0.1, /*heading_max*/ 1.0,
    /*target_v*/ 0.5, /*min_vel*/ 0.1);
  return lqr;
}

/// A straight line along +x with the given length and spacing.
std::vector<TrajectoryPoint> straightLine(double length = 20.0, double step = 0.1)
{
  std::vector<TrajectoryPoint> pts;
  double s = 0.0;
  for (double x = 0.0; x <= length; x += step) {
    TrajectoryPoint p;
    p.x = x;
    p.y = 0.0;
    p.heading = 0.0;
    p.kappa = 0.0;
    p.s = s;
    p.v = 0.5;
    pts.push_back(p);
    s += step;
  }
  return pts;
}

}  // namespace

// ---------------------------------------------------------------------------
// Feedforward: the core difference from the differential-drive model.
// ---------------------------------------------------------------------------

// On a curve of constant curvature kappa, holding the curve requires a steady
// steering angle of L*kappa. The feedforward term must produce exactly that,
// independent of speed. This is the single most important port-specific claim:
// MegaTrak used omega_ff = v*kappa because its input was an angular rate.
TEST(LqrFeedforward, OnConstantCurvatureFeedforwardIsWheelbaseTimesKappa)
{
  const double wheelbase = 1.2;
  const double kappa = 0.1;  // 10 m radius turn

  LQR lqr = makeController(wheelbase);
  auto pts = straightLine();
  for (auto & p : pts) {
    p.kappa = kappa;
  }
  lqr.setTrajectory(pts);

  // Place the robot exactly on the path with zero error, so the feedback term
  // vanishes and only the feedforward remains.
  RobotState state;
  state.x = 0.0;
  state.y = 0.0;
  state.heading = 0.0;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  const auto out = lqr.getControlOutput();
  EXPECT_NEAR(out.steer_ff, wheelbase * kappa, 1e-9);
  // Feedback should be ~0 when the robot sits exactly on the path.
  EXPECT_NEAR(out.steer_fb, 0.0, 1e-6);
  EXPECT_NEAR(out.vel_cmd.steer_angle, wheelbase * kappa, 1e-6);
}

// The feedforward must not scale with speed. If someone reverts to the
// differential-drive formula v*kappa, this test fails at any v != L.
TEST(LqrFeedforward, FeedforwardIsSpeedIndependent)
{
  const double wheelbase = 1.2;
  const double kappa = 0.05;

  std::vector<double> feedforwards;
  for (double v : {0.2, 0.5, 1.0}) {
    LQR lqr = makeController(wheelbase);
    auto pts = straightLine();
    for (auto & p : pts) {
      p.kappa = kappa;
      p.v = v;
    }
    lqr.setTrajectory(pts);

    RobotState state;
    state.x = 0.0;
    state.y = 0.0;
    state.heading = 0.0;
    state.linear_v = v;
    lqr.setRobotState(state);

    feedforwards.push_back(lqr.getControlOutput().steer_ff);
  }

  EXPECT_NEAR(feedforwards[0], feedforwards[1], 1e-9);
  EXPECT_NEAR(feedforwards[1], feedforwards[2], 1e-9);
}

// ---------------------------------------------------------------------------
// Feedback: sign convention and convergence.
// ---------------------------------------------------------------------------

// A robot offset to the LEFT of the path (positive y, path heading +x) must be
// steered RIGHT, i.e. a negative steering angle under the protocol's
// left-positive convention.
TEST(LqrFeedback, LeftLateralErrorCommandsRightSteer)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine());

  RobotState state;
  state.x = 0.0;
  state.y = 0.5;   // 0.5 m to the left of the path
  state.heading = 0.0;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  const auto out = lqr.getControlOutput();
  EXPECT_GT(out.lateral_error, 0.0) << "left offset must be a positive lateral error";
  EXPECT_LT(out.vel_cmd.steer_angle, 0.0) << "must steer right to return to the path";
}

// Mirror case: offset to the right must steer left.
TEST(LqrFeedback, RightLateralErrorCommandsLeftSteer)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine());

  RobotState state;
  state.x = 0.0;
  state.y = -0.5;
  state.heading = 0.0;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  const auto out = lqr.getControlOutput();
  EXPECT_LT(out.lateral_error, 0.0);
  EXPECT_GT(out.vel_cmd.steer_angle, 0.0);
}

// On a straight path with zero initial error, the controller must command no
// steering at all - a nonzero value here would mean the model is biased.
TEST(LqrFeedback, StraightPathNoErrorCommandsZeroSteer)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine());

  RobotState state;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  const auto out = lqr.getControlOutput();
  EXPECT_NEAR(out.vel_cmd.steer_angle, 0.0, 1e-9);
}

// Closed-loop convergence: simulate the bicycle model forward and check the
// lateral error shrinks. This catches sign errors that a single-tick test would
// miss (a wrong sign looks plausible for one tick but diverges over time).
TEST(LqrConvergence, LateralErrorShrinksOverTime)
{
  const double wheelbase = 1.2;
  const double dt = 0.1;
  LQR lqr = makeController(wheelbase);
  lqr.setTrajectory(straightLine(/*length*/ 30.0, /*step*/ 0.1));

  double x = 0.0, y = 0.8, heading = 0.0;  // start 0.8 m left of the path
  double v = 0.5;

  const double initial_err = std::fabs(y);
  double last_err = initial_err;

  for (int i = 0; i < 200; ++i) {
    RobotState state;
    state.x = x;
    state.y = y;
    state.heading = heading;
    state.linear_v = v;
    lqr.setRobotState(state);

    // Goal advancement must be driven by real motion, as in the node.
    lqr.advanceGoal(v * dt);

    const auto out = lqr.getControlOutput();
    const double delta = out.vel_cmd.steer_angle;
    v = out.vel_cmd.linear_v;

    // Kinematic bicycle integration.
    x += v * std::cos(heading) * dt;
    y += v * std::sin(heading) * dt;
    heading += (v / wheelbase) * std::tan(delta) * dt;

    last_err = std::fabs(y);
  }

  EXPECT_LT(last_err, initial_err * 0.05)
    << "lateral error should converge to within 5% of the initial offset";
  EXPECT_LT(last_err, 0.05) << "should settle within 5 cm of the path";
}

// ---------------------------------------------------------------------------
// Limits and guards.
// ---------------------------------------------------------------------------

// The steering command must never exceed the protocol limit of +/-90 deg, even
// for an absurd initial error.
TEST(LqrLimits, SteeringIsClampedToProtocolLimit)
{
  const double max_steer = 90.0 * M_PI / 180.0;
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine());

  RobotState state;
  state.x = 0.0;
  state.y = 50.0;   // absurd 50 m offset
  state.heading = 2.0;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  const auto out = lqr.getControlOutput();
  EXPECT_LE(std::fabs(out.vel_cmd.steer_angle), max_steer + 1e-9);
}

// Speed must stay within the protocol ceiling of 1.4 m/s and above the floor.
TEST(LqrLimits, SpeedStaysWithinProtocolRange)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine());

  RobotState state;
  state.linear_v = 0.0;
  lqr.setRobotState(state);

  const auto out = lqr.getControlOutput();
  EXPECT_GE(out.vel_cmd.linear_v, 0.0);
  EXPECT_LE(out.vel_cmd.linear_v, 1.4 + 1e-9);
}

// A huge curvature must trigger the look-ahead slowdown, not full speed.
TEST(LqrLimits, HighCurvatureTriggersSlowdown)
{
  LQR lqr = makeController(1.2);
  auto pts = straightLine();
  for (auto & p : pts) {
    p.kappa = 1.0;   // 1 m radius: far above the 0.2 threshold
  }
  lqr.setTrajectory(pts);

  RobotState state;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  const auto out = lqr.getControlOutput();
  EXPECT_LT(out.vel_cmd.linear_v, 0.5) << "tight curvature must slow the robot";
}

// Degenerate trajectories must command a stop rather than crash or emit NaN.
TEST(LqrGuards, EmptyTrajectoryCommandsStop)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory({});

  RobotState state;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  const auto out = lqr.getControlOutput();
  EXPECT_DOUBLE_EQ(out.vel_cmd.linear_v, 0.0);
  EXPECT_DOUBLE_EQ(out.vel_cmd.steer_angle, 0.0);
}

// A single point cannot define a heading; must stop, not divide by zero.
TEST(LqrGuards, SinglePointTrajectoryCommandsStop)
{
  LQR lqr = makeController(1.2);
  TrajectoryPoint p;
  lqr.setTrajectory({p});

  RobotState state;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  const auto out = lqr.getControlOutput();
  EXPECT_DOUBLE_EQ(out.vel_cmd.linear_v, 0.0);
  EXPECT_DOUBLE_EQ(out.vel_cmd.steer_angle, 0.0);
}

// A zero wheelbase would divide by zero in B_d. configure() must reject it.
TEST(LqrGuards, NonPositiveWheelbaseIsRejected)
{
  LQR lqr;
  lqr.configure(0.1, 1e-6, 1000, 50.0, 25.0, 25.0,
    /*wheelbase*/ 0.0, 1.5708, 1.4, 0.3, 0.1, 1.0, 0.5, 0.1);
  lqr.setTrajectory(straightLine());

  RobotState state;
  state.y = 0.5;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  const auto out = lqr.getControlOutput();
  EXPECT_TRUE(std::isfinite(out.vel_cmd.steer_angle)) << "must not emit inf/nan";
  EXPECT_TRUE(std::isfinite(out.vel_cmd.linear_v));
}

// Goal advancement must never run off the end of the trajectory.
TEST(LqrGuards, AdvanceGoalStopsAtEnd)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine(/*length*/ 5.0, /*step*/ 0.1));

  // Ask to advance far past the end repeatedly.
  for (int i = 0; i < 100; ++i) {
    lqr.advanceGoal(1.0);
  }

  RobotState state;
  state.x = 5.0;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  const auto out = lqr.getControlOutput();
  EXPECT_TRUE(std::isfinite(out.vel_cmd.steer_angle));
  EXPECT_FALSE(lqr.advanceGoal(1.0)) << "should report the path is exhausted";
}

// Negative travelled distance must not rewind the goal (a bad odometry frame
// would otherwise make the controller chase a stale point).
TEST(LqrGuards, NegativeTravelDoesNotRewindGoal)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine(/*length*/ 20.0, /*step*/ 0.1));

  RobotState state;
  state.x = 2.0;   // robot sits at the advanced goal so the feedback term is ~0
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  // Advance the goal 2 m forward.
  lqr.advanceGoal(2.0);
  const double x_after_advance = lqr.getControlOutput().target_pt.x;
  EXPECT_GT(x_after_advance, 1.5) << "goal should have advanced ~2 m along +x";

  // A negative travel (e.g. from a bad odometry jump) must be clamped to zero,
  // leaving the goal where it was rather than rewinding it.
  lqr.advanceGoal(-5.0);
  const double x_after_negative = lqr.getControlOutput().target_pt.x;
  EXPECT_DOUBLE_EQ(x_after_negative, x_after_advance)
    << "negative travel must not rewind the tracked goal";
}

// ---------------------------------------------------------------------------
// Along-track blind spot. REGRESSION TEST for the 2026-10-15 simulation finding.
//
// Observed: the tracked goal stayed pinned at the path start while the robot ran
// 238 m downrange. steer_angle_rad came out as 2.4e-14 and NO error was reported,
// because the LQR state [e_lateral, e_heading] contains no along-track term:
// a robot that runs straight along the path's own direction has both errors
// exactly zero. The vehicle left the field while every metric read "nominal".
//
// These tests pin down (a) that the blind spot really exists, so nobody later
// assumes e_lat/e_head are sufficient, and (b) that distToGoal() exposes it.
// ---------------------------------------------------------------------------

// The blind spot, stated explicitly: a robot 200 m past the goal along a
// straight path reports ZERO lateral and ZERO heading error.
TEST(LqrAlongTrackBlindSpot, RunningPastGoalReportsZeroError)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine(/*length*/ 20.0, /*step*/ 0.1));

  // Do not advance the goal: this mimics the frozen-goal failure.
  RobotState state;
  state.x = 200.0;   // 200 m downrange of the goal at x=0
  state.y = 0.0;     // exactly on the path line
  state.heading = 0.0;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  const auto out = lqr.getControlOutput();
  EXPECT_NEAR(out.lateral_error, 0.0, 1e-9)
    << "documented blind spot: perpendicular error is zero";
  EXPECT_NEAR(out.heading_error, 0.0, 1e-9)
    << "documented blind spot: heading error is zero";
  EXPECT_NEAR(out.vel_cmd.steer_angle, 0.0, 1e-9)
    << "so the controller commands no correction at all";
  EXPECT_GT(out.vel_cmd.linear_v, 0.0)
    << "and it still commands forward speed - the robot keeps leaving the field";

  // The ONLY signal that anything is wrong is the raw distance to the goal.
  EXPECT_GT(lqr.distToGoal(), 199.0);
}

// distToGoal() must track the real robot-to-goal distance, including after the
// goal advances, since the node's safety gate reads it every tick.
TEST(LqrAlongTrackBlindSpot, DistToGoalTracksRobotAndGoal)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine(/*length*/ 50.0, /*step*/ 0.1));

  RobotState state;
  state.x = 0.0;
  state.y = 0.0;
  state.linear_v = 0.5;
  lqr.setRobotState(state);
  EXPECT_NEAR(lqr.distToGoal(), 0.0, 1e-9) << "robot is on the goal";

  // Move the robot 5 m off in y: distance must be the Euclidean gap.
  state.y = 5.0;
  lqr.setRobotState(state);
  EXPECT_NEAR(lqr.distToGoal(), 5.0, 1e-9);

  // Advance the goal 10 m along +x; the gap is now sqrt(10^2 + 5^2).
  lqr.advanceGoal(10.0);
  EXPECT_NEAR(lqr.distToGoal(), std::hypot(10.0, 5.0), 1e-6);
}

// ---------------------------------------------------------------------------
// Forward snapping: re-anchors the goal to the nearest path point so it cannot
// drift away from the robot on odometry slip alone.
// ---------------------------------------------------------------------------

// With snapping enabled, a robot that has drifted ahead of the arc-length goal
// must pull the goal forward to the nearest point.
TEST(LqrSnap, GoalSnapsForwardToNearestPoint)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine(/*length*/ 50.0, /*step*/ 0.1));
  lqr.setSnapWindow(100);   // 100 points = 10 m of path at 0.1 m spacing

  // Robot sits at x=8, but reports zero travel (odometry slipped / frozen).
  RobotState state;
  state.x = 8.0;
  state.y = 0.0;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  lqr.advanceGoal(0.0);   // no arc-length advance at all
  const auto out = lqr.getControlOutput();
  EXPECT_NEAR(out.target_pt.x, 8.0, 0.1)
    << "snapping must re-anchor the goal to the robot's position";
  EXPECT_LT(lqr.distToGoal(), 0.2);
}

// Snapping must never move the goal backwards: the search starts at the current
// index, so a robot falling behind cannot drag the goal to an earlier row of a
// boustrophedon field (which would re-run already-covered ground).
TEST(LqrSnap, GoalNeverSnapsBackward)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine(/*length*/ 50.0, /*step*/ 0.1));
  lqr.setSnapWindow(100);

  RobotState state;
  state.x = 20.0;
  state.y = 0.0;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  // Advance the goal to x=30, then teleport the robot back to x=5.
  lqr.advanceGoal(30.0);
  const double x_advanced = lqr.getControlOutput().target_pt.x;
  EXPECT_GT(x_advanced, 29.0);

  state.x = 5.0;
  lqr.setRobotState(state);
  lqr.advanceGoal(0.0);
  const double x_after = lqr.getControlOutput().target_pt.x;
  EXPECT_GE(x_after, x_advanced)
    << "goal index is monotonic; snapping must not rewind it";
}

// A zero snap window must disable snapping and fall back to pure arc-length
// accumulation, so the existing advancement tests stay valid.
TEST(LqrSnap, ZeroWindowDisablesSnapping)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine(/*length*/ 50.0, /*step*/ 0.1));
  lqr.setSnapWindow(0);   // disabled (the class default)

  RobotState state;
  state.x = 8.0;
  state.y = 0.0;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  lqr.advanceGoal(0.0);
  const auto out = lqr.getControlOutput();
  EXPECT_NEAR(out.target_pt.x, 0.0, 1e-9)
    << "with snapping off, the goal must stay where arc-length put it";
}

