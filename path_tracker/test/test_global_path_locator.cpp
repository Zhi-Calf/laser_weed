// 版权所有 2026。全局路径连续投影单元测试。
//
// 本组测试约束线段内连续投影、端点夹取、单调推进、角度插值和 TURN 距离。
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "path_tracker/global_path_locator.hpp"

using path_tracker::GlobalPathLocator;
using path_tracker::PathMatch;
using path_tracker::TrajectoryPoint;

namespace {

// 生成沿 +x 方向的离散路径。
std::vector<TrajectoryPoint> StraightPath(double length_m,
                                          double step_m = 0.1) {
  std::vector<TrajectoryPoint> path;
  const std::size_t count =
      static_cast<std::size_t>(std::round(length_m / step_m));
  for (std::size_t index = 0; index <= count; ++index) {
    TrajectoryPoint point;
    point.x = step_m * static_cast<double>(index);
    point.y = 0.0;
    point.heading = 0.0;
    point.s = point.x;
    path.push_back(point);
  }
  return path;
}

}  // namespace

// 线段内部必须返回垂足而不是最近顶点，距离应保持为纯横向偏差。
TEST(GlobalPathLocator, ProjectsInsideSegment) {
  GlobalPathLocator locator;
  locator.Configure(20);
  locator.SetPath(StraightPath(5.0));

  PathMatch match;
  ASSERT_TRUE(locator.Project(1.25, 0.2, &match));

  EXPECT_TRUE(match.valid);
  EXPECT_EQ(match.segment_index, 12U);
  EXPECT_NEAR(match.segment_fraction, 0.5, 1e-12);
  EXPECT_NEAR(match.target_point.x, 1.25, 1e-12);
  EXPECT_NEAR(match.target_point.y, 0.0, 1e-12);
  EXPECT_NEAR(match.distance_m, 0.2, 1e-12);
  EXPECT_FALSE(match.at_path_end);
}

// 车辆越过路径终点时投影必须夹在末点，并显式标记路径结束。
TEST(GlobalPathLocator, ClampsToPathEnd) {
  GlobalPathLocator locator;
  locator.Configure(20);
  locator.SetPath(StraightPath(1.0));

  PathMatch match;
  ASSERT_TRUE(locator.Project(2.0, 0.2, &match));

  EXPECT_NEAR(match.target_point.x, 1.0, 1e-12);
  EXPECT_NEAR(match.distance_m, std::hypot(1.0, 0.2), 1e-12);
  EXPECT_TRUE(match.at_path_end);
}

// 单调搜索不能因为车辆位置回退而把目标拉回已完成路径。
TEST(GlobalPathLocator, NeverProjectsBackward) {
  GlobalPathLocator locator;
  locator.Configure(100);
  locator.SetPath(StraightPath(5.0));

  PathMatch first;
  ASSERT_TRUE(locator.Project(2.0, 0.0, &first));
  EXPECT_NEAR(first.target_point.x, 2.0, 1e-12);

  PathMatch second;
  ASSERT_TRUE(locator.Project(1.0, 0.0, &second));
  EXPECT_GE(second.target_point.x, first.target_point.x);
  EXPECT_NEAR(second.target_point.x, 2.0, 1e-12);
}

// 前向窗口必须按线段数量限制首次吸附范围。
TEST(GlobalPathLocator, LimitsInitialSearchWindow) {
  GlobalPathLocator locator;
  locator.Configure(10);
  locator.SetPath(StraightPath(5.0));

  PathMatch match;
  ASSERT_TRUE(locator.Project(3.0, 0.0, &match));

  EXPECT_NEAR(match.target_point.x, 1.0, 1e-12);
  EXPECT_NEAR(match.distance_m, 2.0, 1e-12);
}

// 航向跨越 +/-pi 时必须走最短角度插值，不能绕向另一侧。
TEST(GlobalPathLocator, InterpolatesHeadingAcrossPi) {
  std::vector<TrajectoryPoint> path(2);
  path[0].heading = 3.1;
  path[1].x = 0.1;
  path[1].heading = -3.1;
  path[1].s = 0.1;

  GlobalPathLocator locator;
  locator.Configure(2);
  locator.SetPath(path);

  PathMatch match;
  ASSERT_TRUE(locator.Project(0.05, 0.0, &match));

  EXPECT_NEAR(std::abs(match.target_point.heading), M_PI, 2e-3);
}

// 投影点必须同时给出到下一 TURN 和距上一 TURN 的弧长。
TEST(GlobalPathLocator, ReportsTurnDistances) {
  auto path = StraightPath(10.0);
  for (std::size_t index = 50; index <= 60; ++index) {
    path[index].waypoint_type = 1;
  }

  GlobalPathLocator locator;
  locator.Configure(100);
  locator.SetPath(path);

  PathMatch before_turn;
  ASSERT_TRUE(locator.Project(1.0, 0.0, &before_turn));
  EXPECT_NEAR(before_turn.distance_to_next_turn_m, 4.0, 1e-12);
  EXPECT_TRUE(std::isinf(before_turn.distance_from_previous_turn_m));

  locator.Reset();
  PathMatch after_turn;
  ASSERT_TRUE(locator.Project(7.0, 0.0, &after_turn));
  EXPECT_NEAR(after_turn.distance_from_previous_turn_m, 1.0, 1e-12);
  EXPECT_TRUE(std::isinf(after_turn.distance_to_next_turn_m));
}

// 空路径和单点路径不能产生可跟踪投影。
TEST(GlobalPathLocator, RejectsDegeneratePath) {
  GlobalPathLocator locator;
  locator.Configure(20);

  PathMatch empty_match;
  EXPECT_FALSE(locator.Project(0.0, 0.0, &empty_match));
  EXPECT_FALSE(empty_match.valid);

  std::vector<TrajectoryPoint> one_point(1);
  locator.SetPath(one_point);
  PathMatch one_match;
  EXPECT_FALSE(locator.Project(0.0, 0.0, &one_match));
  EXPECT_FALSE(one_match.valid);
}
