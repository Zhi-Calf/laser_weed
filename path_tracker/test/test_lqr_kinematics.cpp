// 版权所有 2026。四轮转向 LQR 单元测试。
//
// 这些测试用于约束移植后的运动学推导，
// 重点验证前馈使用 atan(L*kappa)，而不是履带差速模型的 v*kappa。
#include "path_tracker/lqr.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using path_tracker::LQR;
using path_tracker::LQROutput;
using path_tracker::RobotState;
using path_tracker::TrajectoryPoint;

namespace
{

/// 使用文档中的默认参数构建控制器。
LQR makeController(double wheelbase = 1.2)
{
  LQR lqr;
  lqr.configure(
    /* 时间步长 */ 0.1, /* 收敛阈值 */ 1e-6, /* 最大迭代次数 */ 1000,
    /* 横向误差权重 */ 50.0, /* 航向误差权重 */ 25.0,
    /* 转角代价 */ 25.0,
    wheelbase, /* 最大转角 */ 90.0 * M_PI / 180.0,
    /* 最大速度 */ 1.4,
    /* 最低速度比例 */ 0.3, /* 航向死区 */ 0.1,
    /* 航向误差上限 */ 1.0,
    /* 目标速度 */ 0.5, /* 最低速度 */ 0.1);
  return lqr;
}

/// 生成沿 +x 方向、指定长度和点间距的直线路径。
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

}  // 匿名命名空间

// ---------------------------------------------------------------------------
// 前馈：与差速驱动模型的核心差异。
// ---------------------------------------------------------------------------

// 定曲率稳态前馈必须使用 atan(L*kappa)，不能在大转角时继续用 L*kappa。
TEST(LqrFeedforward, OnConstantCurvatureUsesExactAckermannFeedforward)
{
  const double wheelbase = 1.2;
  const double kappa = 0.1;  // 半径 10 米的弯道

  LQR lqr = makeController(wheelbase);
  auto pts = straightLine();
  for (auto & p : pts) {
    p.kappa = kappa;
  }
  lqr.setTrajectory(pts);

  // 将车辆精确放在路径上并使误差为零，使反馈项消失，只保留前馈项。
  RobotState state;
  state.x = 0.0;
  state.y = 0.0;
  state.heading = 0.0;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  const auto out = lqr.getControlOutput();
  EXPECT_NEAR(out.steer_ff, std::atan(wheelbase * kappa), 1e-9);
  // 车辆精确位于路径上时，反馈转角应接近 0。
  EXPECT_NEAR(out.steer_fb, 0.0, 1e-6);
  EXPECT_NEAR(
    out.vel_cmd.steer_angle,
    std::atan(wheelbase * kappa), 1e-6);
}

// 前馈不得随速度变化。如果误改回差速驱动公式 v*kappa，
// 只要 v 不等于 L，本测试就会失败。
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

// 运行时切换等效轴距后，前馈必须立即按新模式重新计算。
TEST(LqrFeedforward, RuntimeWheelbaseUpdateChangesFeedforward)
{
  constexpr double kPhysicalWheelbase = 1.2;
  constexpr double kKappa = 0.8;

  LQR lqr = makeController(kPhysicalWheelbase);
  auto pts = straightLine();
  for (auto & p : pts) {
    p.kappa = kKappa;
  }
  lqr.setTrajectory(pts);

  RobotState state;
  state.linear_v = 0.3;
  lqr.setRobotState(state);

  lqr.SetWheelbase(kPhysicalWheelbase);
  const double single_feedforward = lqr.getControlOutput().steer_ff;
  lqr.SetWheelbase(kPhysicalWheelbase * 0.5);
  const double symmetric_feedforward = lqr.getControlOutput().steer_ff;

  EXPECT_NEAR(
    single_feedforward, std::atan(kPhysicalWheelbase * kKappa), 1e-9);
  EXPECT_NEAR(
    symmetric_feedforward,
    std::atan(kPhysicalWheelbase * 0.5 * kKappa), 1e-9);
  EXPECT_LT(
    std::fabs(symmetric_feedforward),
    std::fabs(single_feedforward));
}

// WORK/TURN 分段速度必须在前方预览区和后方保持区生效。
TEST(LqrSpeedProfile, TurnZoneUsesFixedLowSpeed)
{
  constexpr double kStep = 0.1;
  LQR lqr = makeController(1.2);
  lqr.SetSpeedShapingEnabled(false);
  lqr.SetSegmentSpeedProfile(0.6, 0.3, 1.0, 0.5, true);

  auto pts = straightLine(20.0, kStep);
  for (size_t i = 0; i < pts.size(); ++i) {
    const bool is_turn = (i >= 50 && i <= 60);
    pts[i].waypoint_type = is_turn ? 1 : 0;
    pts[i].v = is_turn ? 0.3 : 0.6;
  }
  lqr.setTrajectory(pts);

  RobotState state;
  state.linear_v = 0.6;
  lqr.setRobotState(state);
  EXPECT_NEAR(lqr.getControlOutput().vel_cmd.linear_v, 0.6, 1e-9);

  lqr.advanceGoal(4.5);
  EXPECT_NEAR(lqr.getControlOutput().vel_cmd.linear_v, 0.3, 1e-9);

  lqr.advanceGoal(2.0);
  EXPECT_NEAR(lqr.getControlOutput().vel_cmd.linear_v, 0.3, 1e-9);

  lqr.advanceGoal(1.0);
  EXPECT_NEAR(lqr.getControlOutput().vel_cmd.linear_v, 0.6, 1e-9);
}

// ---------------------------------------------------------------------------
// 反馈：符号约定与收敛性。
// ---------------------------------------------------------------------------

// 车辆位于路径左侧（y 为正、路径航向为 +x）时必须向右修正，
// 即按照协议左正右负的约定输出负转角。
TEST(LqrFeedback, LeftLateralErrorCommandsRightSteer)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine());

  RobotState state;
  state.x = 0.0;
  state.y = 0.5;   // 位于路径左侧 0.5 米
  state.heading = 0.0;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  const auto out = lqr.getControlOutput();
  EXPECT_GT(out.lateral_error, 0.0) << "left offset must be a positive lateral error";
  EXPECT_LT(out.vel_cmd.steer_angle, 0.0) << "must steer right to return to the path";
}

// 对称情形：位于路径右侧时必须向左修正。
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

// 直线路径且初始误差为零时，控制器不得输出转角；
// 非零输出说明模型存在偏置。
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

// 闭环收敛：向前积分自行车模型并检查横向误差是否减小。
// 单周期测试可能看不出符号错误，但错误符号会随时间发散，可通过本测试发现。
TEST(LqrConvergence, LateralErrorShrinksOverTime)
{
  const double wheelbase = 1.2;
  const double dt = 0.1;
  LQR lqr = makeController(wheelbase);
  lqr.setTrajectory(straightLine(/*length*/ 30.0, /*step*/ 0.1));

  double x = 0.0, y = 0.8, heading = 0.0;  // 初始位于路径左侧 0.8 米
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

    // 与节点一致，目标点必须由实际运动推进。
    lqr.advanceGoal(v * dt);

    const auto out = lqr.getControlOutput();
    const double delta = out.vel_cmd.steer_angle;
    v = out.vel_cmd.linear_v;

    // 积分运动学自行车模型。
    x += v * std::cos(heading) * dt;
    y += v * std::sin(heading) * dt;
    heading += (v / wheelbase) * std::tan(delta) * dt;

    last_err = std::fabs(y);
  }

  EXPECT_LT(last_err, initial_err * 0.05)
    << "lateral error should converge to within 5% of the initial offset";
  EXPECT_LT(last_err, 0.05) << "should settle within 5 cm of the path";
}

// 对称阿克曼在定曲率圆弧上必须能由前馈保持，而不是只在直线上收敛。
TEST(LqrConvergence, SymmetricAckermannTracksCircularArc)
{
  constexpr double kDt = 0.1;
  constexpr double kRadius = 2.0;
  constexpr double kKappa = 1.0 / kRadius;
  constexpr double kPhysicalWheelbase = 1.2;
  constexpr double kEffectiveWheelbase = kPhysicalWheelbase * 0.5;
  constexpr double kArcStep = 0.05;

  std::vector<TrajectoryPoint> arc;
  double arc_length = 0.0;
  const double sweep = M_PI / 2.0;
  const int samples =
    static_cast<int>(std::ceil(kRadius * sweep / kArcStep));
  for (int i = 0; i <= samples; ++i) {
    const double theta =
      -M_PI / 2.0 + sweep * static_cast<double>(i) /
      static_cast<double>(samples);
    TrajectoryPoint point;
    point.x = kRadius * std::cos(theta);
    point.y = kRadius + kRadius * std::sin(theta);
    point.heading = theta + M_PI / 2.0;
    point.kappa = kKappa;
    point.s = arc_length;
    point.v = 0.3;
    point.waypoint_type = 1;
    arc.push_back(point);
    arc_length += kRadius * sweep / static_cast<double>(samples);
  }

  LQR lqr = makeController(kPhysicalWheelbase);
  lqr.SetWheelbase(kEffectiveWheelbase);
  lqr.SetSpeedShapingEnabled(false);
  lqr.SetSegmentSpeedProfile(0.6, 0.3, 1.0, 0.5, true);
  lqr.setTrajectory(arc);

  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double max_radial_error = 0.0;
  for (int i = 0; i < 100; ++i) {
    RobotState state;
    state.x = x;
    state.y = y;
    state.heading = heading;
    state.linear_v = 0.3;
    lqr.setRobotState(state);
    if (i > 0) {
      ASSERT_TRUE(lqr.advanceGoal(0.3 * kDt));
    }

    const LQROutput output = lqr.getControlOutput();
    const double speed = output.vel_cmd.linear_v;
    const double delta = output.vel_cmd.steer_angle;
    x += speed * std::cos(heading) * kDt;
    y += speed * std::sin(heading) * kDt;
    heading +=
      (speed / kEffectiveWheelbase) * std::tan(delta) * kDt;

    const double radial_error =
      std::fabs(std::hypot(x, y - kRadius) - kRadius);
    max_radial_error = std::max(max_radial_error, radial_error);
  }

  EXPECT_LT(max_radial_error, 0.05)
    << "symmetric Ackermann feedforward should hold a 2 m arc";
}

// ---------------------------------------------------------------------------
// 限幅和保护条件。
// ---------------------------------------------------------------------------

// 即使初始误差极大，转角指令也不得超过协议规定的 ±90 度。
TEST(LqrLimits, SteeringIsClampedToProtocolLimit)
{
  const double max_steer = 90.0 * M_PI / 180.0;
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine());

  RobotState state;
  state.x = 0.0;
  state.y = 50.0;   // 极端横向偏差 50 米
  state.heading = 2.0;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  const auto out = lqr.getControlOutput();
  EXPECT_LE(std::fabs(out.vel_cmd.steer_angle), max_steer + 1e-9);
}

// 速度必须保持在协议上限 1.4 米/秒和最低速度之间。
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

// 大曲率必须触发前瞻降速，而不能保持全速。
TEST(LqrLimits, HighCurvatureTriggersSlowdown)
{
  LQR lqr = makeController(1.2);
  auto pts = straightLine();
  for (auto & p : pts) {
    p.kappa = 1.0;   // 半径 1 米，远高于 0.2 的曲率阈值
  }
  lqr.setTrajectory(pts);

  RobotState state;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  const auto out = lqr.getControlOutput();
  EXPECT_LT(out.vel_cmd.linear_v, 0.5) << "tight curvature must slow the robot";
}

// 退化轨迹必须输出停车，而不是崩溃或输出 NaN。
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

// 单点无法定义航向，必须停车，不能发生除零。
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

// 零轴距会使 B_d 除零，configure() 必须拒绝该配置。
TEST(LqrGuards, NonPositiveWheelbaseIsRejected)
{
  LQR lqr;
  lqr.configure(0.1, 1e-6, 1000, 50.0, 25.0, 25.0,
    /* 轴距 */ 0.0, 1.5708, 1.4, 0.3, 0.1, 1.0, 0.5, 0.1);
  lqr.setTrajectory(straightLine());

  RobotState state;
  state.y = 0.5;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  const auto out = lqr.getControlOutput();
  EXPECT_TRUE(std::isfinite(out.vel_cmd.steer_angle)) << "must not emit inf/nan";
  EXPECT_TRUE(std::isfinite(out.vel_cmd.linear_v));
}

// 目标推进不得越过轨迹末端。
TEST(LqrGuards, AdvanceGoalStopsAtEnd)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine(/*length*/ 5.0, /*step*/ 0.1));

  // 反复请求远超终点的推进距离。
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

// 单拍位移小于路径点间距时，残余弧长必须累计，不能永久卡在第一个点。
TEST(LqrGuards, SmallTravelAccumulatesAcrossTicks)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine(/*length*/ 2.0, /*step*/ 0.1));

  lqr.advanceGoal(0.03);
  EXPECT_EQ(lqr.CurrentIndex(), 0u);
  lqr.advanceGoal(0.03);
  EXPECT_EQ(lqr.CurrentIndex(), 0u);
  lqr.advanceGoal(0.03);
  EXPECT_EQ(lqr.CurrentIndex(), 0u);
  lqr.advanceGoal(0.03);
  EXPECT_EQ(lqr.CurrentIndex(), 1u);
}

// 负行驶距离不得回退目标点，否则错误的里程计帧会让控制器追逐旧目标。
TEST(LqrGuards, NegativeTravelDoesNotRewindGoal)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine(/*length*/ 20.0, /*step*/ 0.1));

  RobotState state;
  state.x = 2.0;   // 车辆位于推进后的目标点，使反馈项接近 0
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  // 将目标点向前推进 2 米。
  lqr.advanceGoal(2.0);
  const double x_after_advance = lqr.getControlOutput().target_pt.x;
  EXPECT_GT(x_after_advance, 1.5) << "goal should have advanced ~2 m along +x";

  // 负位移（例如里程计异常跳变）必须截断为零，保持目标点位置不变。
  lqr.advanceGoal(-5.0);
  const double x_after_negative = lqr.getControlOutput().target_pt.x;
  EXPECT_DOUBLE_EQ(x_after_negative, x_after_advance)
    << "negative travel must not rewind the tracked goal";
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
// 同时验证 distToGoal() 能暴露该问题。
// ---------------------------------------------------------------------------

// 显式验证盲区：车辆沿直线路径冲过目标点 200 米后，
// 横向误差和航向误差仍然都为零。
TEST(LqrAlongTrackBlindSpot, RunningPastGoalReportsZeroError)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine(/*length*/ 20.0, /*step*/ 0.1));

  // 不推进目标点，模拟目标冻结失效。
  RobotState state;
  state.x = 200.0;   // 位于 x=0 目标点前方 200 米
  state.y = 0.0;     // 精确位于路径线上
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

  // 能够发现异常的唯一信号是车辆到目标点的原始距离。
  EXPECT_GT(lqr.distToGoal(), 199.0);
}

// distToGoal() 必须反映真实的车辆到目标点距离，并在目标推进后继续更新，
// 因为节点每个控制周期都会使用该值执行安全门控。
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

  // 将车辆沿 y 方向偏移 5 米，距离应为欧氏距离。
  state.y = 5.0;
  lqr.setRobotState(state);
  EXPECT_NEAR(lqr.distToGoal(), 5.0, 1e-9);

  // 将目标沿 +x 推进 10 米，此时距离应为 sqrt(10^2 + 5^2)。
  lqr.advanceGoal(10.0);
  EXPECT_NEAR(lqr.distToGoal(), std::hypot(10.0, 5.0), 1e-6);
}

// ---------------------------------------------------------------------------
// 前向吸附：将目标重新锚定到最近的路径点，避免仅由里程计打滑导致目标漂移。
// ---------------------------------------------------------------------------

// 开启吸附后，已偏离到弧长目标前方的车辆应将目标点前拉至最近路径点。
TEST(LqrSnap, GoalSnapsForwardToNearestPoint)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine(/*length*/ 50.0, /*step*/ 0.1));
  lqr.setSnapWindow(100);   // 100 个点，按 0.1 米间距等于 10 米路径

  // 车辆位于 x=8，但行驶距离为零，模拟里程计打滑或冻结。
  RobotState state;
  state.x = 8.0;
  state.y = 0.0;
  state.linear_v = 0.5;
  lqr.setRobotState(state);

  lqr.advanceGoal(0.0);   // 不进行弧长推进
  const auto out = lqr.getControlOutput();
  EXPECT_NEAR(out.target_pt.x, 8.0, 0.1)
    << "snapping must re-anchor the goal to the robot's position";
  EXPECT_LT(lqr.distToGoal(), 0.2);
}

// 吸附不得向后移动目标点。搜索从当前索引开始，
// 因此落后的车辆不能把目标拉回弓字形田块的上一行，避免重复作业。
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

  // 将目标推进到 x=30，然后把车辆瞬移回 x=5。
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

// 吸附窗口为零时必须关闭吸附，退回到纯弧长累计，
// 使原有目标推进测试仍然有效。
TEST(LqrSnap, ZeroWindowDisablesSnapping)
{
  LQR lqr = makeController(1.2);
  lqr.setTrajectory(straightLine(/*length*/ 50.0, /*step*/ 0.1));
  lqr.setSnapWindow(0);   // 关闭吸附，这也是类默认值

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

