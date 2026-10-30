// 版权所有 2026。全局路径最近点搜索单元测试。
//
// 本组测试约束离散最近点、端点夹取、单调推进和 TURN 距离。
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "path_tracker/global_path_locator.hpp"

using path_tracker::GlobalPathLocator;
using path_tracker::GlobalPathLocatorConfig;
using path_tracker::PathMatchResult;
using path_tracker::PathWorkMode;
using path_tracker::RobotState;
using path_tracker::TrajectoryPoint;

namespace {

void ConfigureLocator(
  GlobalPathLocator* locator,
    int forward_point_window = 8,
  double turn_mode_enter_m         = 0.6,
  double turn_mode_exit_m          = 0.6,
  double wheelbase_m               = 1.0,
  bool pose_at_vehicle_center      = false,
  double goal_reached_distance_m   = 1.0) {
  GlobalPathLocatorConfig config;
  config.forward_point_window   = forward_point_window;
  config.turn_mode_enter_m      = turn_mode_enter_m;
  config.turn_mode_exit_m       = turn_mode_exit_m;
  config.wheelbase_m            = wheelbase_m;
  config.goal_reached_distance_m = goal_reached_distance_m;
  config.pose_at_vehicle_center = pose_at_vehicle_center;
  locator->Configure(config);
}

PathMatchResult ProjectAt(
    GlobalPathLocator* locator,
    double x,
    double y,
    double heading = 0.0) {
  RobotState robot_state;
  robot_state.x       = x;
  robot_state.y       = y;
  robot_state.heading = heading;
  return locator->Project(robot_state);
}

// 生成沿 +x 方向的离散路径。
std::vector<TrajectoryPoint> StraightPath(double length_m, double step_m = 0.1) {
  std::vector<TrajectoryPoint> path;
  const std::size_t count = static_cast<std::size_t>(std::round(length_m / step_m));
  for (std::size_t index = 0; index <= count; ++index) {
    TrajectoryPoint point;
    point.x       = step_m * static_cast<double>(index);
    point.y       = 0.0;
    point.heading = 0.0;
    point.s       = point.x;
    path.push_back(point);
  }
  return path;
}

}   // namespace

// 查询点位于两个路径点之间时，必须选择距离最近的离散点。
TEST(GlobalPathLocator, FindsNearestDiscretePoint) {
  GlobalPathLocator locator;
  ConfigureLocator(&locator, 20);
  locator.SetPath(StraightPath(5.0));

  const auto result = ProjectAt(&locator, 1.24, 0.2);
  ASSERT_TRUE(result.success);
  const PathMatchResult& match = result;

  EXPECT_EQ(match.point_index, 12);
  EXPECT_NEAR(match.target_point.x, 1.2, 1e-12);
  EXPECT_NEAR(match.target_point.y, 0.0, 1e-12);
  EXPECT_NEAR(match.nearest_distance_m, std::hypot(0.04, 0.2), 1e-12);
  EXPECT_FALSE(match.at_path_end);
}

// 车辆越过路径终点时最近点必须夹在末点，并显式标记路径结束。
TEST(GlobalPathLocator, ClampsToPathEnd) {
  GlobalPathLocator locator;
  ConfigureLocator(&locator, 20);
  locator.SetPath(StraightPath(1.0));

  const auto result = ProjectAt(&locator, 2.0, 0.2);
  ASSERT_TRUE(result.success);
  const PathMatchResult& match = result;

  EXPECT_NEAR(match.target_point.x, 1.0, 1e-12);
  EXPECT_TRUE(match.at_path_end);
}

// 非法车辆位姿必须在公共入口统一拒绝。
TEST(GlobalPathLocator, RejectsNonFinitePose) {
  GlobalPathLocator locator;
  ConfigureLocator(&locator, 20);
  locator.SetPath(StraightPath(5.0));

  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ProjectAt(&locator, nan, 0.0, 0.0).success);
  EXPECT_FALSE(ProjectAt(&locator, 1.0, nan, 0.0).success);
  EXPECT_FALSE(ProjectAt(&locator, 1.0, 0.0, nan).success);
}

// 当前位姿距终点不超过配置阈值时必须提前标记到达终点。
TEST(GlobalPathLocator, ReportsGoalReachedByPoseDistance) {
  GlobalPathLocator locator;
  ConfigureLocator(&locator, 100, 0.6, 0.6, 1.0, false, 0.5);
  locator.SetPath(StraightPath(10.0));

  const auto outside = ProjectAt(&locator, 9.4, 0.0);
  ASSERT_TRUE(outside.success);
  EXPECT_FALSE(outside.at_path_end);

  locator.Reset();
  const auto inside = ProjectAt(&locator, 9.6, 0.0);
  ASSERT_TRUE(inside.success);
  EXPECT_TRUE(inside.at_path_end);
}

// 单调搜索不能因为车辆位置回退而让几何中心进度后退。
TEST(GlobalPathLocator, NeverProjectsBackward) {
  GlobalPathLocator locator;
  ConfigureLocator(&locator, 100);
  locator.SetPath(StraightPath(5.0));

  const auto first_result = ProjectAt(&locator, 2.0, 0.0);
  ASSERT_TRUE(first_result.success);
  EXPECT_NEAR(first_result.nearest_point.x, 2.0, 1e-12);

  const auto second_result = ProjectAt(&locator, 1.0, 0.0);
  ASSERT_TRUE(second_result.success);
  EXPECT_GE(second_result.nearest_point.x, first_result.nearest_point.x);
  EXPECT_NEAR(second_result.nearest_point.x, 2.0, 1e-12);
}

// 后轴目标必须夹到路径端点，不能因控制点在路径外而判定搜索失败。
TEST(GlobalPathLocator, ClampsRearAxleTargetToPathEndpoints) {
  const auto path = StraightPath(5.0);
  GlobalPathLocator locator;
  ConfigureLocator(&locator, 100, 0.6, 0.6, 1.0, true);
  locator.SetPath(path);

  const auto before_start = ProjectAt(&locator, -0.4, 0.0, 0.0);
  ASSERT_TRUE(before_start.success);
  EXPECT_NEAR(before_start.target_point.x, 0.0, 1e-12);

  locator.Reset();
  const auto after_end = ProjectAt(&locator, 5.5, 0.0, 0.0);
  ASSERT_TRUE(after_end.success);
  EXPECT_NEAR(after_end.target_point.x, 5.0, 1e-12);
  EXPECT_TRUE(after_end.at_path_end);
}

// 前向窗口必须按点索引偏移量限制首次吸附范围。
TEST(GlobalPathLocator, LimitsInitialSearchWindow) {
  GlobalPathLocator locator;
  ConfigureLocator(&locator, 10);
  locator.SetPath(StraightPath(5.0));

  const auto result = ProjectAt(&locator, 3.0, 0.0);
  ASSERT_TRUE(result.success);
  const PathMatchResult& match = result;

  EXPECT_NEAR(match.nearest_point.x, 1.0, 1e-12);
  EXPECT_NEAR(match.target_point.x, 2.0, 1e-12);
}

// 最近点搜索必须直接返回离散点的航向，不进行线段插值。
TEST(GlobalPathLocator, UsesDiscretePointHeading) {
  std::vector<TrajectoryPoint> path(2);
  path[0].heading = 3.1;
  path[1].x       = 0.1;
  path[1].heading = -3.1;
  path[1].s       = 0.1;

  GlobalPathLocator locator;
  ConfigureLocator(&locator, 2);
  locator.SetPath(path);

  const auto first_result = ProjectAt(&locator, 0.04, 0.0);
  ASSERT_TRUE(first_result.success);

  EXPECT_EQ(first_result.point_index, 0);
  EXPECT_DOUBLE_EQ(first_result.target_point.heading, 3.1);

  locator.Reset();
  const auto second_result = ProjectAt(&locator, 0.06, 0.0);
  ASSERT_TRUE(second_result.success);
  EXPECT_EQ(second_result.point_index, 1);
  EXPECT_DOUBLE_EQ(second_result.target_point.heading, -3.1);
}

// 前向 8 点预转弯、后向 5 点保持区必须共同决定模式和目标点。
TEST(GlobalPathLocator, SelectsModeAndTargetWithinTurnWindows) {
  auto path = StraightPath(10.0);
  for (std::size_t index = 18; index <= 20; ++index) {
    path[index].waypoint_type = 1;
  }

  GlobalPathLocator locator;
  ConfigureLocator(&locator, 100, 0.8, 0.5);
  locator.SetPath(path);

  const auto preview_result = ProjectAt(&locator, 1.2, 0.0);
  ASSERT_TRUE(preview_result.success);
  const PathMatchResult& preview = preview_result;
  EXPECT_EQ(preview.work_mode, PathWorkMode::kTurn);
  EXPECT_EQ(preview.nearest_index, 12);
  EXPECT_EQ(preview.point_index, 12);
  EXPECT_NEAR(preview.target_point.x, 1.2, 1e-12);
  EXPECT_NEAR(preview.control_state.x, 1.2, 1e-12);

  const auto in_turn_result = ProjectAt(&locator, 1.8, 0.0);
  ASSERT_TRUE(in_turn_result.success);
  const PathMatchResult& in_turn = in_turn_result;
  EXPECT_EQ(in_turn.work_mode, PathWorkMode::kTurn);
  EXPECT_EQ(in_turn.nearest_index, 18);
  EXPECT_EQ(in_turn.point_index, 18);

  const auto hold_result = ProjectAt(&locator, 2.2, 0.0);
  ASSERT_TRUE(hold_result.success);
  const PathMatchResult& hold = hold_result;
  EXPECT_EQ(hold.work_mode, PathWorkMode::kTurn);
  EXPECT_EQ(hold.nearest_index, 22);
  EXPECT_EQ(hold.point_index, 22);

  const auto work_result = ProjectAt(&locator, 2.6, 0.0);
  ASSERT_TRUE(work_result.success);
  const PathMatchResult& work = work_result;
  EXPECT_EQ(work.work_mode, PathWorkMode::kWork);
  EXPECT_EQ(work.nearest_index, 26);
  EXPECT_EQ(work.point_index, 26);
  EXPECT_NEAR(work.target_point.x, 2.6, 1e-12);
}

// WORK 模式必须使用后轴中心对应的最近点作为目标点。
TEST(GlobalPathLocator, WorkModeSelectsRearAxleTarget) {
  GlobalPathLocator locator;
  ConfigureLocator(&locator, 20, 0.6, 0.6, 1.2, true);
  locator.SetPath(StraightPath(5.0));

  const auto result = ProjectAt(&locator, 2.0, 0.0, 0.0);
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.work_mode, PathWorkMode::kWork);
  EXPECT_EQ(result.nearest_index, 20);
  EXPECT_EQ(result.point_index, 14);
  EXPECT_NEAR(result.target_point.x, 1.4, 1e-12);
  EXPECT_NEAR(result.control_state.x, 1.4, 1e-12);
}

// 后轴局部目标不得回写几何中心的单调进度。
TEST(GlobalPathLocator, ProjectNearDoesNotAdvanceCenterProgress) {
  GlobalPathLocator locator;
  ConfigureLocator(&locator, 100, 0.6, 0.6, 1.2, true);
  locator.SetPath(StraightPath(5.0));

  const auto first = ProjectAt(&locator, 2.0, 0.0);
  ASSERT_TRUE(first.success);
  ASSERT_EQ(first.nearest_index, 20);
  ASSERT_EQ(first.point_index, 14);

  const auto second = ProjectAt(&locator, 1.0, 0.0);
  ASSERT_TRUE(second.success);
  EXPECT_EQ(second.nearest_index, 20);
  EXPECT_NEAR(second.nearest_point.x, 2.0, 1e-12);
  EXPECT_NE(second.point_index, second.nearest_index);
}

// 非法配置必须在 Configure 中归一化，后续搜索不再重复兜底。
TEST(GlobalPathLocator, NormalizesInvalidConfigInConfigure) {
  GlobalPathLocator locator;
  ConfigureLocator(
      &locator,
      100,
      std::numeric_limits<double>::quiet_NaN(),
      -1.0,
      std::numeric_limits<double>::quiet_NaN(),
      true,
      std::numeric_limits<double>::quiet_NaN());
  locator.SetPath(StraightPath(5.0));

  const auto result = ProjectAt(&locator, 2.0, 0.0, 0.0);
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.target_point.x, 1.5, 1e-12);
}

// 空路径和单点路径不能产生可跟踪匹配。
TEST(GlobalPathLocator, RejectsDegeneratePath) {
  GlobalPathLocator locator;
  ConfigureLocator(&locator, 20);

  const auto empty_result = ProjectAt(&locator, 0.0, 0.0);
  EXPECT_FALSE(empty_result.success);
  EXPECT_FALSE(empty_result.success);

  std::vector<TrajectoryPoint> one_point(1);
  locator.SetPath(one_point);
  const auto one_result = ProjectAt(&locator, 0.0, 0.0);
  EXPECT_FALSE(one_result.success);
  EXPECT_FALSE(one_result.success);
}
