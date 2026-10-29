// 版权所有 2026。四轮转向 LQR 单元测试。
//
// 这些测试用于约束移植后的运动学推导，
// 重点验证前馈使用 atan(L*kappa)，而不是履带差速模型的 v*kappa。
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "path_tracker/global_path_locator.hpp"
#include "path_tracker/lqr.hpp"

using path_tracker::GlobalPathLocator;
using path_tracker::GlobalPathLocatorConfig;
using path_tracker::LineReference;
using path_tracker::LQRConfig;
using path_tracker::LQROutput;
using path_tracker::PathMatchResult;
using path_tracker::PathWorkMode;
using path_tracker::RobotState;
using path_tracker::TrajectoryPoint;

namespace {

struct LQRInput {
  RobotState robot_state;
};

// 将最近点搜索与 LQR 组合为路径跟踪测试控制器。
class TestPathTracker {
 public:
  void Configure(const LQRConfig& config,
                 int forward_point_window = 20,
                 double turn_mode_enter_m = 0.6,
                 double turn_mode_exit_m = 0.6) {
    lqr_.Configure(config);
    GlobalPathLocatorConfig path_locator_config;
    path_locator_config.forward_point_window = forward_point_window;
    path_locator_config.turn_mode_enter_m    = turn_mode_enter_m;
    path_locator_config.turn_mode_exit_m     = turn_mode_exit_m;
    path_locator_config.wheelbase_m          = config.wheelbase;
    path_locator_config.goal_reached_distance_m = 0.0;
    path_locator_config.pose_at_vehicle_center =
        config.pose_at_vehicle_center;
    locator_.Configure(path_locator_config);
  }

  void SetTrajectory(const std::vector<TrajectoryPoint>& path) {
    locator_.SetPath(path);
    lqr_.Reset();
  }

  LQROutput ComputeCommand(const LQRInput& input) {
    const PathMatchResult center_result =
        locator_.Project(input.robot_state);
    if (!center_result.success ||
        center_result.at_path_end) {
      return LQROutput{};
    }
    const PathMatchResult& center_match = center_result;
    return lqr_.ComputeCommand(
        input.robot_state,
        center_match);
  }

 private:
  path_tracker::LQR lqr_;              // 被测控制器
  GlobalPathLocator locator_;          // 全局路径最近点搜索
};

/// 使用文档中的默认参数构建配置。
LQRConfig makeConfig(double wheelbase = 1.2) {
  LQRConfig config;
  config.wheelbase              = wheelbase;
  config.max_steer_rad          = 90.0 * M_PI / 180.0;
  config.pose_at_vehicle_center = false;   // 既有测试使用后轴中心
  return config;
}

/// 使用测试配置构建控制器。
TestPathTracker makeController(double wheelbase = 1.2,
                               int forward_point_window = 20) {
  TestPathTracker lqr;
  LQRConfig config           = makeConfig(wheelbase);
  lqr.Configure(config, forward_point_window);
  return lqr;
}

// 执行一次完整控制计算；失败时返回零输出，供测试继续检查。
LQROutput RunControl(TestPathTracker& lqr, const RobotState& state) {
  LQRInput input;
  input.robot_state = state;
  const LQROutput output = lqr.ComputeCommand(input);
  EXPECT_TRUE(output.success);
  return output;
}

/// 生成沿 +x 方向、指定长度和点间距的直线路径。
std::vector<TrajectoryPoint> straightLine(double length = 20.0, double step = 0.1) {
  std::vector<TrajectoryPoint> pts;
  double s = 0.0;
  for (double x = 0.0; x <= length; x += step) {
    TrajectoryPoint p;
    p.x       = x;
    p.y       = 0.0;
    p.heading = 0.0;
    p.kappa   = 0.0;
    p.s       = s;
    pts.push_back(p);
    s += step;
  }
  return pts;
}

// 使用独立构造的离散模型求解反馈增益。
Eigen::Matrix<double, 1, 2> SolveFeedbackGain(
    const Eigen::Matrix2d& a,
    const Eigen::Matrix<double, 2, 1>& b,
    const Eigen::Matrix2d& q,
    double r,
    int max_iterations,
    double eps) {
  Eigen::Matrix2d p = q;
  double difference = std::numeric_limits<double>::max();
  for (int iteration = 0;
       iteration < max_iterations && difference > eps;
       ++iteration) {
    const Eigen::Matrix<double, 2, 1> pb = p * b;
    const Eigen::Matrix<double, 2, 1> pa = a.transpose() * pb;
    const double denominator             = r + b.dot(pb);
    const Eigen::Matrix2d next =
        q + a.transpose() * p * a - pa * pa.transpose() / denominator;
    difference = (next - p).cwiseAbs().maxCoeff();
    p          = next;
  }
  return b.transpose() * p * a / (r + b.dot(p * b));
}

}   // namespace

// ---------------------------------------------------------------------------
// 前馈：与差速驱动模型的核心差异。
// ---------------------------------------------------------------------------

// 单阿克曼下自动换算到后轴；双阿克曼保持中心。
TEST(LqrReferencePoint, AutoSelectsPoseReferenceBySteerMode) {
  constexpr double kWheelbase = 1.2;
  constexpr double kTargetY   = 5.0;
  constexpr double kCenterX   = 0.2;

  auto build_path = [](uint8_t waypoint_type) {
    std::vector<TrajectoryPoint> pts;
    for (int i = 0; i <= 100; ++i) {
      TrajectoryPoint p;
      p.x             = 0.0;
      p.y             = 0.1 * static_cast<double>(i);
      p.heading       = M_PI / 2.0;
      p.kappa         = 0.0;
      p.s             = 0.1 * static_cast<double>(i);
      p.waypoint_type = waypoint_type;
      pts.push_back(p);
    }
    return pts;
  };

  LQRConfig config              = makeConfig(kWheelbase);
  config.pose_at_vehicle_center = true;
  RobotState center_state;
  center_state.x        = kCenterX;
  center_state.y        = kTargetY;
  center_state.heading  = 0.0;
  center_state.linear_v = 0.5;

  TestPathTracker single_ackermann;
  single_ackermann.Configure(config, 100);
  single_ackermann.SetTrajectory(build_path(0));
  const auto single_output = RunControl(single_ackermann, center_state);
  EXPECT_NEAR(
      single_output.lateral_error,
      kWheelbase * 0.5 - kCenterX,
      1e-9);

  TestPathTracker symmetric_ackermann;
  symmetric_ackermann.Configure(config, 100);
  symmetric_ackermann.SetTrajectory(build_path(1));
  const auto symmetric_output = RunControl(symmetric_ackermann, center_state);
  EXPECT_NEAR(symmetric_output.lateral_error, -kCenterX, 1e-9);
}

// 弯道上必须使用后轴投影点，而不是几何中心对应的前方路径站。
TEST(LqrReferencePoint, SingleModeUsesRearAxleStationOnCurve) {
  constexpr double kRadius    = 5.0;
  constexpr double kWheelbase = 1.2;
  constexpr double kStep      = 0.05;

  std::vector<TrajectoryPoint> arc;
  const double sweep = M_PI / 2.0;
  const int samples =
      static_cast<int>(std::ceil(kRadius * sweep / kStep));
  for (int index = 0; index <= samples; ++index) {
    const double theta =
        sweep * static_cast<double>(index) / static_cast<double>(samples);
    TrajectoryPoint point;
    point.x       = kRadius * std::sin(theta);
    point.y       = kRadius * (1.0 - std::cos(theta));
    point.heading = theta;
    point.kappa   = 1.0 / kRadius;
    point.s       = kRadius * theta;
    arc.push_back(point);
  }

  LQRConfig config              = makeConfig(kWheelbase);
  config.pose_at_vehicle_center = true;
  TestPathTracker lqr;
  lqr.Configure(config, 100);
  lqr.SetTrajectory(arc);

  constexpr double kCenterTheta = 0.5;
  RobotState state;
  state.x = kRadius * std::sin(kCenterTheta);
  state.y = kRadius * (1.0 - std::cos(kCenterTheta));
  state.heading  = kCenterTheta;
  state.linear_v = 0.5;

  const auto output = RunControl(lqr, state);
  EXPECT_NEAR(
      output.target_pt.s,
      kRadius * kCenterTheta - kWheelbase * 0.5,
      0.06);
  EXPECT_LT(output.target_pt.heading, kCenterTheta);
  EXPECT_LT(output.lateral_error, 0.0);
  EXPECT_EQ(output.steer_mode, 0);
}

// 单阿克曼必须使用统一匹配结果中的控制状态和目标点。
TEST(LqrCommandInputs, SingleModeUsesSelectedTargetAndState) {
  LQRConfig config              = makeConfig(1.2);
  config.pose_at_vehicle_center = false;
  path_tracker::LQR lqr;
  lqr.Configure(config);

  PathMatchResult match;
  match.success                      = true;
  match.nearest_point.x              = 10.0;
  match.nearest_point.y              = 2.0;
  match.target_point.x               = 10.0;
  match.target_point.y               = 2.0;
  match.target_point.heading         = 0.1;
  match.target_point.kappa           = 0.05;
  match.point_index                  = 6;
  match.work_mode                    = PathWorkMode::kWork;

  RobotState state;
  state.x        = 10.0;
  state.y        = 2.2;
  state.linear_v = 0.5;
  match.control_state = state;

  const LQROutput output =
      lqr.ComputeCommand(state, match);
  ASSERT_TRUE(output.success);
  EXPECT_NEAR(output.heading_error, -0.1, 1e-12);
  EXPECT_NEAR(output.lateral_error, 0.2 * std::cos(0.1), 1e-12);
  EXPECT_NEAR(output.steer_ff, std::atan(1.2 * 0.05), 1e-12);
  EXPECT_NEAR(output.target_pt.heading, 0.1, 1e-12);
  EXPECT_EQ(output.target_index, 6);
  EXPECT_NEAR(output.distance_to_goal, 0.2, 1e-12);
}

// 双阿克曼模式必须由统一匹配结果中的工作模式选择。
TEST(LqrCommandInputs, SymmetricModeUsesTurnWorkMode) {
  LQRConfig config              = makeConfig(1.2);
  config.pose_at_vehicle_center = false;
  path_tracker::LQR lqr;
  lqr.Configure(config);

  PathMatchResult match;
  match.success                      = true;
  match.nearest_point.x              = 2.6;
  match.nearest_point.waypoint_type = 1;
  match.target_point.waypoint_type  = 1;
  match.target_point.kappa          = 0.1;
  match.target_point.x              = 3.0;
  match.point_index                 = 12;
  match.work_mode                   = PathWorkMode::kTurn;

  RobotState state;
  state.x        = 3.0;
  state.linear_v = 0.3;
  match.control_state = state;

  const LQROutput output =
      lqr.ComputeCommand(state, match);
  ASSERT_TRUE(output.success);
  EXPECT_EQ(output.steer_mode, 1);
  EXPECT_NEAR(output.steer_ff, std::atan(0.6 * 0.1), 1e-12);
  EXPECT_NEAR(output.distance_to_goal, 0.4, 1e-12);
  EXPECT_EQ(output.target_index, 12);
}

// 定曲率稳态前馈必须使用 atan(L*kappa)，不能在大转角时继续用 L*kappa。
TEST(LqrFeedforward, OnConstantCurvatureUsesExactAckermannFeedforward) {
  const double wheelbase = 1.2;
  const double kappa     = 0.1;   // 半径 10 米的弯道

  TestPathTracker lqr  = makeController(wheelbase);
  auto pts = straightLine();
  for (auto& p : pts) {
    p.kappa = kappa;
  }
  lqr.SetTrajectory(pts);

  // 将车辆精确放在路径上并使误差为零，使反馈项消失，只保留前馈项。
  RobotState state;
  state.x        = 0.0;
  state.y        = 0.0;
  state.heading  = 0.0;
  state.linear_v = 0.5;
  const auto out = RunControl(lqr, state);
  EXPECT_NEAR(out.steer_ff, std::atan(wheelbase * kappa), 1e-9);
  // 车辆精确位于路径上时，反馈转角应接近 0。
  EXPECT_NEAR(out.steer_fb, 0.0, 1e-6);
  EXPECT_NEAR(out.vel_cmd.steer_angle, std::atan(wheelbase * kappa), 1e-6);
}

// 前馈不得随速度变化。如果误改回差速驱动公式 v*kappa，
// 只要 v 不等于 L，本测试就会失败。
TEST(LqrFeedforward, FeedforwardIsSpeedIndependent) {
  const double wheelbase = 1.2;
  const double kappa     = 0.05;

  std::vector<double> feedforwards;
  for (double v : {0.2, 0.5, 1.0}) {
    TestPathTracker lqr  = makeController(wheelbase);
    auto pts = straightLine();
    for (auto& p : pts) {
      p.kappa = kappa;
    }
    lqr.SetTrajectory(pts);

    RobotState state;
    state.x        = 0.0;
    state.y        = 0.0;
    state.heading  = 0.0;
    state.linear_v = v;
    feedforwards.push_back(RunControl(lqr, state).steer_ff);
  }

  EXPECT_NEAR(feedforwards[0], feedforwards[1], 1e-9);
  EXPECT_NEAR(feedforwards[1], feedforwards[2], 1e-9);
}

// WORK/TURN 转向模式必须选择对应的等效轴距和前馈。
TEST(LqrFeedforward, SegmentSteerModeChangesEffectiveWheelbase) {
  constexpr double kPhysicalWheelbase = 1.2;
  constexpr double kKappa             = 0.8;

  TestPathTracker lqr          = makeController(kPhysicalWheelbase);
  LQRConfig config = makeConfig(kPhysicalWheelbase);
  lqr.Configure(config, 100);
  auto work_points = straightLine();
  auto turn_points = straightLine();
  for (auto& p : work_points) {
    p.kappa         = kKappa;
    p.waypoint_type = 0;
  }
  for (auto& p : turn_points) {
    p.kappa         = kKappa;
    p.waypoint_type = 1;
  }

  RobotState state;
  state.linear_v = 0.3;

  lqr.SetTrajectory(work_points);
  const auto work_output = RunControl(lqr, state);
  lqr.SetTrajectory(turn_points);
  const auto turn_output = RunControl(lqr, state);

  EXPECT_NEAR(work_output.steer_ff, std::atan(kPhysicalWheelbase * kKappa), 1e-9);
  EXPECT_NEAR(turn_output.steer_ff, std::atan(kPhysicalWheelbase * 0.5 * kKappa), 1e-9);
  EXPECT_EQ(work_output.steer_mode, 0);
  EXPECT_EQ(turn_output.steer_mode, 1);
  EXPECT_NEAR(work_output.effective_wheelbase, kPhysicalWheelbase, 1e-9);
  EXPECT_NEAR(turn_output.effective_wheelbase, kPhysicalWheelbase * 0.5, 1e-9);
}

// WORK/TURN 分段速度必须在前方预览区和后方保持区生效。
TEST(LqrSpeedProfile, TurnZoneUsesFixedLowSpeed) {
  constexpr double kStep      = 0.1;
  LQRConfig config            = makeConfig();
  config.work_speed           = 0.6;
  config.turn_speed           = 0.3;
  TestPathTracker lqr;
  lqr.Configure(config, 100, 1.0, 0.5);

  auto pts = straightLine(20.0, kStep);
  for (size_t i = 0; i < pts.size(); ++i) {
    const bool is_turn   = (i >= 50 && i <= 60);
    pts[i].waypoint_type = is_turn ? 1 : 0;
  }
  lqr.SetTrajectory(pts);

  RobotState state;
  state.linear_v = 0.6;
  EXPECT_NEAR(RunControl(lqr, state).vel_cmd.linear_v, 0.6, 1e-9);

  state.x = 4.5;
  EXPECT_NEAR(RunControl(lqr, state).vel_cmd.linear_v, 0.3, 1e-9);

  state.x = 6.5;
  EXPECT_NEAR(RunControl(lqr, state).vel_cmd.linear_v, 0.3, 1e-9);

  state.x = 7.5;
  EXPECT_NEAR(RunControl(lqr, state).vel_cmd.linear_v, 0.6, 1e-9);
}

// 速度预览窗口之外的 TURN 不得降低目标速度。
TEST(LqrPathProgress, IgnoresTurnOutsideSpeedPreviewWindow) {
  constexpr double kStep = 0.1;
  auto points            = straightLine(20.0, kStep);
  for (std::size_t index = 50; index <= 60; ++index) {
    points[index].waypoint_type = 1;
  }

  LQRConfig config = makeConfig();
  TestPathTracker lqr;
  lqr.Configure(config, 8);
  lqr.SetTrajectory(points);

  RobotState state;
  state.x           = 1.0;
  const auto output = RunControl(lqr, state);

  EXPECT_NEAR(output.vel_cmd.linear_v, config.work_speed, 1e-9);
}

// ---------------------------------------------------------------------------
// 反馈：符号约定与收敛性。
// ---------------------------------------------------------------------------

// 车辆位于路径左侧（y 为正、路径航向为 +x）时必须向右修正，
// 即按照协议左正右负的约定输出负转角。
TEST(LqrFeedback, LeftLateralErrorCommandsRightSteer) {
  TestPathTracker lqr = makeController(1.2);
  lqr.SetTrajectory(straightLine());

  RobotState state;
  state.x        = 0.0;
  state.y        = 0.5;   // 位于路径左侧 0.5 米
  state.heading  = 0.0;
  state.linear_v = 0.5;
  const auto out = RunControl(lqr, state);
  EXPECT_GT(out.lateral_error, 0.0) << "left offset must be a positive lateral error";
  EXPECT_LT(out.vel_cmd.steer_angle, 0.0) << "must steer right to return to the path";
}

// 对称情形：位于路径右侧时必须向左修正。
TEST(LqrFeedback, RightLateralErrorCommandsLeftSteer) {
  TestPathTracker lqr = makeController(1.2);
  lqr.SetTrajectory(straightLine());

  RobotState state;
  state.x        = 0.0;
  state.y        = -0.5;
  state.heading  = 0.0;
  state.linear_v = 0.5;
  const auto out = RunControl(lqr, state);
  EXPECT_LT(out.lateral_error, 0.0);
  EXPECT_GT(out.vel_cmd.steer_angle, 0.0);
}

// 直线路径且初始误差为零时，控制器不得输出转角；
// 非零输出说明模型存在偏置。
TEST(LqrFeedback, StraightPathNoErrorCommandsZeroSteer) {
  TestPathTracker lqr = makeController(1.2);
  lqr.SetTrajectory(straightLine());

  RobotState state;
  state.linear_v = 0.5;
  const auto out = RunControl(lqr, state);
  EXPECT_NEAR(out.vel_cmd.steer_angle, 0.0, 1e-9);
}

// 视觉路径在左为正，必须转换为 LQR 的车辆相对路径符号并驱动左转。
TEST(LqrLineCommand, ConvertsVisualLateralSignAtRearAxle) {
  path_tracker::LQR lqr;
  LQRConfig config              = makeConfig(1.2);
  config.pose_at_vehicle_center = true;
  lqr.Configure(config);

  LineReference reference;
  reference.lateral_error_m = 0.2;
  RobotState state;
  state.linear_v = 0.5;
  const LQROutput output =
      lqr.ComputeLineCommand(state, reference, 0.6);
  ASSERT_TRUE(output.success);

  EXPECT_NEAR(output.lateral_error, -0.2, 1e-12);
  EXPECT_NEAR(output.heading_error, 0.0, 1e-12);
  EXPECT_GT(output.vel_cmd.steer_angle, 0.0);
  EXPECT_EQ(output.steer_mode, 0);
}

// 视觉中心线向左偏航时必须转换为负的 LQR 航向误差并驱动左转。
TEST(LqrLineCommand, ConvertsVisualHeadingSign) {
  path_tracker::LQR lqr;
  LQRConfig config              = makeConfig(1.2);
  config.pose_at_vehicle_center = true;
  lqr.Configure(config);

  LineReference reference;
  reference.heading_error_rad = 0.1;
  RobotState state;
  state.linear_v = 0.5;
  const LQROutput output =
      lqr.ComputeLineCommand(state, reference, 0.6);
  ASSERT_TRUE(output.success);

  EXPECT_NEAR(output.heading_error, -0.1, 1e-12);
  EXPECT_GT(output.vel_cmd.steer_angle, 0.0);
}

// LQR 增益必须使用实测车速，输出速度仍使用参考指令。
TEST(LqrFeedback, UsesActualSpeedForGainScheduling) {
  path_tracker::LQR lqr;
  lqr.Configure(makeConfig(1.2));

  LineReference reference;
  reference.lateral_error_m = 0.2;

  RobotState slow_state;
  slow_state.linear_v = 0.1;
  RobotState fast_state;
  fast_state.linear_v = 0.6;

  const LQROutput slow =
      lqr.ComputeLineCommand(slow_state, reference, 0.6);
  const LQROutput fast =
      lqr.ComputeLineCommand(fast_state, reference, 0.6);
  ASSERT_TRUE(slow.success);
  ASSERT_TRUE(fast.success);
  EXPECT_NEAR(slow.vel_cmd.linear_v, 0.6, 1e-12);
  EXPECT_NEAR(fast.vel_cmd.linear_v, 0.6, 1e-12);
  EXPECT_GT(std::abs(slow.steer_fb - fast.steer_fb), 1e-9);
}

// 精确零阶保持必须计入转角在本周期内产生的直接横向位移。
TEST(LqrFeedback, UsesExactZeroOrderHoldInputMatrix) {
  constexpr double kActualSpeed = 0.5;
  constexpr double kHeadingError = 0.1;
  LQRConfig config = makeConfig(1.2);
  path_tracker::LQR lqr;
  lqr.Configure(config);

  LineReference reference;
  reference.heading_error_rad = -kHeadingError;
  RobotState state;
  state.linear_v = kActualSpeed;
  const LQROutput output =
      lqr.ComputeLineCommand(state, reference, 0.6);
  ASSERT_TRUE(output.success);

  const double speed_period = kActualSpeed * config.dt;
  const Eigen::Matrix2d a = (Eigen::Matrix2d() <<
      1.0, speed_period,
      0.0, 1.0).finished();
  const Eigen::Matrix<double, 2, 1> exact_b =
      (Eigen::Matrix<double, 2, 1>() <<
          speed_period * speed_period / (2.0 * config.wheelbase),
          speed_period / config.wheelbase)
          .finished();
  const Eigen::Matrix<double, 2, 1> euler_b =
      (Eigen::Matrix<double, 2, 1>() <<
          0.0,
          speed_period / config.wheelbase)
          .finished();
  const Eigen::Matrix2d q =
      (Eigen::Matrix2d() <<
          config.q_lateral, 0.0,
          0.0, config.q_heading)
          .finished();
  const Eigen::Vector2d error(0.0, kHeadingError);
  const double exact_steer =
      -(SolveFeedbackGain(
            a, exact_b, q, config.r_steer, config.max_iteration,
            config.eps) *
        error)(0);
  const double euler_steer =
      -(SolveFeedbackGain(
            a, euler_b, q, config.r_steer, config.max_iteration,
            config.eps) *
        error)(0);

  EXPECT_GT(std::abs(exact_steer - euler_steer), 1e-5);
  EXPECT_NEAR(output.steer_fb, exact_steer, 1e-9);
  EXPECT_NEAR(output.vel_cmd.linear_v, 0.6, 1e-12);
}

// 闭环收敛：向前积分自行车模型并检查横向误差是否减小。
// 单周期测试可能看不出符号错误，但错误符号会随时间发散，可通过本测试发现。
TEST(LqrConvergence, LateralErrorShrinksOverTime) {
  const double wheelbase = 1.2;
  const double dt        = 0.1;
  TestPathTracker lqr                = makeController(wheelbase);
  lqr.SetTrajectory(straightLine(/*length*/ 30.0, /*step*/ 0.1));

  double x = 0.0, y = 0.8, heading = 0.0;   // 初始位于路径左侧 0.8 米
  double v = 0.5;

  const double initial_err = std::fabs(y);
  double last_err          = initial_err;

  for (int i = 0; i < 200; ++i) {
    RobotState state;
    state.x        = x;
    state.y        = y;
    state.heading  = heading;
    state.linear_v = v;
    // 与节点一致，目标点必须由实际运动推进。
    const auto out     = RunControl(lqr, state);
    const double delta = out.vel_cmd.steer_angle;
    v                  = out.vel_cmd.linear_v;

    // 积分运动学自行车模型。
    x += v * std::cos(heading) * dt;
    y += v * std::sin(heading) * dt;
    heading += (v / wheelbase) * std::tan(delta) * dt;

    last_err = std::fabs(y);
  }

  EXPECT_LT(last_err, initial_err * 0.05) << "lateral error should converge to within 5% of the initial offset";
  EXPECT_LT(last_err, 0.05) << "should settle within 5 cm of the path";
}

// 对称阿克曼在定曲率圆弧上必须能由前馈保持，而不是只在直线上收敛。
TEST(LqrConvergence, SymmetricAckermannTracksCircularArc) {
  constexpr double kDt                 = 0.1;
  constexpr double kRadius             = 2.0;
  constexpr double kKappa              = 1.0 / kRadius;
  constexpr double kPhysicalWheelbase  = 1.2;
  constexpr double kEffectiveWheelbase = kPhysicalWheelbase * 0.5;
  constexpr double kArcStep            = 0.05;

  std::vector<TrajectoryPoint> arc;
  double arc_length  = 0.0;
  const double sweep = M_PI / 2.0;
  const int samples  = static_cast<int>(std::ceil(kRadius * sweep / kArcStep));
  for (int i = 0; i <= samples; ++i) {
    const double theta = -M_PI / 2.0 + sweep * static_cast<double>(i) / static_cast<double>(samples);
    TrajectoryPoint point;
    point.x             = kRadius * std::cos(theta);
    point.y             = kRadius + kRadius * std::sin(theta);
    point.heading       = theta + M_PI / 2.0;
    point.kappa         = kKappa;
    point.s             = arc_length;
    point.waypoint_type = 1;
    arc.push_back(point);
    arc_length += kRadius * sweep / static_cast<double>(samples);
  }

  LQRConfig config            = makeConfig(kPhysicalWheelbase);
  config.work_speed           = 0.6;
  config.turn_speed           = 0.3;
  TestPathTracker lqr;
  lqr.Configure(config, 20, 1.0, 0.5);
  lqr.SetTrajectory(arc);

  double x                = 0.0;
  double y                = 0.0;
  double heading          = 0.0;
  double max_radial_error = 0.0;
  for (int i = 0; i < 100; ++i) {
    RobotState state;
    state.x        = x;
    state.y        = y;
    state.heading  = heading;
    state.linear_v = 0.3;
    LQRInput input;
    input.robot_state = state;
    const LQROutput output = lqr.ComputeCommand(input);
    ASSERT_TRUE(output.success);
    const double speed = output.vel_cmd.linear_v;
    const double delta = output.vel_cmd.steer_angle;
    x += speed * std::cos(heading) * kDt;
    y += speed * std::sin(heading) * kDt;
    heading += (speed / kEffectiveWheelbase) * std::tan(delta) * kDt;

    const double radial_error = std::fabs(std::hypot(x, y - kRadius) - kRadius);
    max_radial_error          = std::max(max_radial_error, radial_error);
  }

  EXPECT_LT(max_radial_error, 0.05) << "symmetric Ackermann feedforward should hold a 2 m arc";
}

// ---------------------------------------------------------------------------
// 限幅和保护条件。
// ---------------------------------------------------------------------------

// 即使初始误差极大，转角指令也不得超过协议规定的 ±90 度。
TEST(LqrLimits, SteeringIsClampedToProtocolLimit) {
  const double max_steer = 90.0 * M_PI / 180.0;
  TestPathTracker lqr                = makeController(1.2);
  lqr.SetTrajectory(straightLine());

  RobotState state;
  state.x        = 0.0;
  state.y        = 50.0;   // 极端横向偏差 50 米
  state.heading  = 2.0;
  state.linear_v = 0.5;
  const auto out = RunControl(lqr, state);
  EXPECT_LE(std::fabs(out.vel_cmd.steer_angle), max_steer + 1e-9);
}

// 横向和航向误差不得改变路径分段决定的目标速度。
TEST(LqrLimits, TrackingErrorsDoNotChangeSpeed) {
  TestPathTracker lqr = makeController(1.2);
  lqr.SetTrajectory(straightLine());

  RobotState state;
  state.y = 0.25;
  EXPECT_NEAR(RunControl(lqr, state).vel_cmd.linear_v, 0.6, 1e-9);

  state.y       = 0.0;
  state.heading = 1.0;
  EXPECT_NEAR(RunControl(lqr, state).vel_cmd.linear_v, 0.6, 1e-9);
}

// 曲率只用于转角前馈，不再触发速度降档。
TEST(LqrLimits, HighCurvatureDoesNotChangeSpeed) {
  TestPathTracker lqr  = makeController(1.2);
  auto pts = straightLine();
  for (auto& p : pts) {
    p.kappa = 1.0;
  }
  lqr.SetTrajectory(pts);

  RobotState state;
  state.linear_v = 0.5;
  const auto out = RunControl(lqr, state);
  EXPECT_DOUBLE_EQ(out.vel_cmd.linear_v, 0.6) << "曲率不应再改变分段速度";
}

// 到达轨迹末端时必须停止输出控制指令。
TEST(LqrGuards, StopsAtPathEnd) {
  TestPathTracker lqr = makeController(1.2, 100);
  lqr.SetTrajectory(straightLine(/*length*/ 5.0, /*step*/ 0.1));

  RobotState state;
  state.linear_v = 0.5;
  RunControl(lqr, state);   // 建立上一帧位姿基线

  state.x = 100.0;
  LQRInput input;
  input.robot_state = state;
  const LQROutput out = lqr.ComputeCommand(input);
  EXPECT_FALSE(out.success) << "should report the path is exhausted";
  EXPECT_DOUBLE_EQ(out.vel_cmd.linear_v, 0.0);
  EXPECT_DOUBLE_EQ(out.vel_cmd.steer_angle, 0.0);
}

// ---------------------------------------------------------------------------
// 沿路径方向盲区。针对 2026-10-15 仿真问题的回归测试。
//
// 当时目标点固定在路径起点，车辆向前行驶了 238 米。
// steer_angle_rad 仅为 2.4e-14，且没有任何误差告警，
// 因为 LQR 状态 [e_lateral, e_heading] 不含沿路径方向误差：
// 车辆沿路径自身方向直行时，两项误差都恰为零，所有指标仍显示正常。
//
// 这些测试用于确认盲区确实存在，防止后续误认为 e_lat/e_head 已经足够，
// 同时验证输出的目标距离能暴露该问题。
// ---------------------------------------------------------------------------

// 显式验证盲区：车辆沿直线路径冲过目标点 200 米后，
// 横向误差和航向误差仍然都为零。
TEST(LqrAlongTrackBlindSpot, RunningPastGoalReportsZeroError) {
  TestPathTracker lqr = makeController(1.2);
  lqr.SetTrajectory(straightLine(/*length*/ 20.0, /*step*/ 0.1));

  // 不推进目标点，模拟目标冻结失效。
  RobotState state;
  state.x        = 200.0;   // 位于 x=0 目标点前方 200 米
  state.y        = 0.0;     // 精确位于路径线上
  state.heading  = 0.0;
  state.linear_v = 0.5;
  const auto out = RunControl(lqr, state);
  EXPECT_NEAR(out.lateral_error, 0.0, 1e-9) << "documented blind spot: perpendicular error is zero";
  EXPECT_NEAR(out.heading_error, 0.0, 1e-9) << "documented blind spot: heading error is zero";
  EXPECT_NEAR(out.vel_cmd.steer_angle, 0.0, 1e-9) << "so the controller commands no correction at all";
  EXPECT_GT(out.vel_cmd.linear_v, 0.0) << "and it still commands forward speed - the robot keeps leaving the field";

  // 能够发现异常的唯一信号是车辆到目标点的原始距离。
  EXPECT_GT(out.distance_to_goal, 197.0);
}

// 输出距离必须反映真实的车辆到目标点距离，并在目标推进后继续更新，
// 因为节点每个控制周期都会使用该值执行安全门控。
TEST(LqrAlongTrackBlindSpot, OutputDistanceTracksRobotAndGoal) {
  TestPathTracker lqr = makeController(1.2, 300);
  lqr.SetTrajectory(straightLine(/*length*/ 50.0, /*step*/ 0.1));

  RobotState state;
  state.x        = 0.0;
  state.y        = 0.0;
  state.linear_v = 0.5;
  EXPECT_NEAR(RunControl(lqr, state).distance_to_goal, 0.0, 1e-9) << "robot is on the goal";

  // 沿路径前进 10 米后，目标点应同步推进。
  state.x = 10.0;
  EXPECT_NEAR(RunControl(lqr, state).distance_to_goal, 0.0, 1e-9);

  // 再横向偏移 5 米。
  state.y = 5.0;
  EXPECT_NEAR(RunControl(lqr, state).distance_to_goal, 5.0, 1e-6);
}

// ---------------------------------------------------------------------------
// 前向吸附：将目标重新锚定到最近的路径点，避免仅由里程计打滑导致目标漂移。
// ---------------------------------------------------------------------------

// 开启吸附后，已偏离到弧长目标前方的车辆应将目标点前拉至最近路径点。
TEST(LqrSnap, GoalSnapsForwardToNearestPoint) {
  TestPathTracker lqr = makeController(1.2, 100);
  lqr.SetTrajectory(straightLine(/*length*/ 50.0, /*step*/ 0.1));

  // 车辆位于 x=8，但行驶距离为零，模拟里程计打滑或冻结。
  RobotState state;
  state.x        = 8.0;
  state.y        = 0.0;
  state.linear_v = 0.5;
  const auto out = RunControl(lqr, state);
  EXPECT_NEAR(out.target_pt.x, 8.0, 0.1) << "snapping must re-anchor the goal to the robot's position";
  EXPECT_LT(out.distance_to_goal, 0.2);
}

// 搜索窗口必须从上一目标点开始，并用当前位姿逐步向前更新。
TEST(LqrSnap, WindowStartsAtPreviousNearestPoint) {
  TestPathTracker lqr = makeController(1.2, 100);
  lqr.SetTrajectory(straightLine(/*length*/ 50.0, /*step*/ 0.1));

  RobotState state;
  state.x        = 8.0;
  state.y        = 0.0;
  state.linear_v = 0.5;
  const auto out = RunControl(lqr, state);
  EXPECT_NEAR(out.target_pt.x, 8.0, 0.1) << "吸附应从上一次目标点开始搜索";

  state.x             = 9.0;
  const auto next_out = RunControl(lqr, state);
  EXPECT_NEAR(next_out.target_pt.x, 9.0, 0.1) << "下一帧应使用当前位姿继续向前搜索";
}

// 单次吸附最多向前搜索 forward_point_window 个路径点索引偏移。
TEST(LqrSnap, SearchDoesNotExceedForwardWindow) {
  TestPathTracker lqr = makeController(1.2, 10);
  lqr.SetTrajectory(straightLine(/*length*/ 50.0, /*step*/ 0.1));

  RobotState state;
  state.x        = 8.0;
  state.linear_v = 0.5;
  const auto out = RunControl(lqr, state);
  EXPECT_NEAR(out.distance_to_goal, 7.0, 1e-9)
      << "10 个 0.1 m 点距的窗口只能把中心目标推进到 x=1";
}

// 局部控制目标只能围绕单调推进的中心目标移动，不能回到车辆当前所在路径位置。
TEST(LqrSnap, GoalStaysNearMonotonicCenterProgress) {
  TestPathTracker lqr = makeController(1.2, 300);
  lqr.SetTrajectory(straightLine(/*length*/ 50.0, /*step*/ 0.1));

  RobotState state;
  state.x        = 20.0;
  state.y        = 0.0;
  state.linear_v = 0.5;
  // 将目标推进到 x=30，然后把车辆瞬移回 x=5。
  const double x_advanced = RunControl(lqr, state).target_pt.x;
  EXPECT_NEAR(x_advanced, 20.0, 0.1) << "目标应吸附到车辆 x=20 附近的最近点";

  state.x              = 5.0;
  const auto backward_out = RunControl(lqr, state);
  EXPECT_NEAR(backward_out.distance_to_goal, 15.0, 0.1)
      << "中心最近点进度必须保持在 x=20，不能回退到车辆 x=5";
  EXPECT_GE(backward_out.target_pt.x, 18.8 - 1e-9)
      << "单拍控制点局部搜索不得跳出中心进度附近的一个轴距窗口";
}
