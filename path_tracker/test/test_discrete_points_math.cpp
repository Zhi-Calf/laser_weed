// 版权所有 2026。离散点路径参数单元测试。
#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#include "path_tracker/discrete_points_math.hpp"

using path_tracker::TrajectoryPoint;
using path_tracker::ComputePathProfile;

namespace {
std::vector<std::pair<double, double>> makeXY(const std::vector<double>& xs, const std::vector<double>& ys) {
  std::vector<std::pair<double, double>> xy;
  xy.reserve(xs.size());
  for (size_t i = 0; i < xs.size(); ++i) {
    xy.emplace_back(xs[i], ys[i]);
  }
  return xy;
}
}   // namespace

TEST(DiscretePointsMath, RejectsFewerThanThreePoints) {
  std::vector<TrajectoryPoint> out;
  EXPECT_FALSE(ComputePathProfile({}, &out));
  EXPECT_TRUE(out.empty());
  EXPECT_FALSE(ComputePathProfile({{0.0, 0.0}}, &out));
  EXPECT_TRUE(out.empty());
  EXPECT_FALSE(ComputePathProfile({{0.0, 0.0}, {1.0, 0.0}}, &out));
  EXPECT_TRUE(out.empty());
}

TEST(DiscretePointsMath, RejectsNullOutput) {
  const auto xy = makeXY({0.0, 1.0, 2.0}, {0.0, 0.0, 0.0});
  EXPECT_FALSE(ComputePathProfile(xy, nullptr));
}

TEST(DiscretePointsMath, StraightLineHasZeroCurvatureAndHeading) {
  const auto xy = makeXY({0.0, 1.0, 2.0, 3.0, 4.0}, {0.0, 0.0, 0.0, 0.0, 0.0});
  std::vector<TrajectoryPoint> out;
  ASSERT_TRUE(ComputePathProfile(xy, &out));
  ASSERT_EQ(out.size(), 5u);

  for (const auto& p : out) {
    EXPECT_NEAR(p.heading, 0.0, 1e-9);
    EXPECT_NEAR(p.kappa, 0.0, 1e-9);
  }
}

TEST(DiscretePointsMath, ArcLengthAccumulates) {
  // 三段各 1 米，总弧长应为 3 米。
  const auto xy = makeXY({0.0, 1.0, 2.0, 3.0}, {0.0, 0.0, 0.0, 0.0});
  std::vector<TrajectoryPoint> out;
  ASSERT_TRUE(ComputePathProfile(xy, &out));

  EXPECT_NEAR(out.front().s, 0.0, 1e-9);
  EXPECT_NEAR(out[1].s, 1.0, 1e-9);
  EXPECT_NEAR(out[2].s, 2.0, 1e-9);
  EXPECT_NEAR(out.back().s, 3.0, 1e-9);
}

TEST(DiscretePointsMath, LeftTurnHasPositiveCurvature) {
  // 先沿 +x 再沿 +y 是左转（逆时针）。CAN 协议规定左转为正，
  // 因此计算出的曲率必须为正。
  const auto xy = makeXY({0.0, 1.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 2.0});
  std::vector<TrajectoryPoint> out;
  ASSERT_TRUE(ComputePathProfile(xy, &out));

  EXPECT_GT(out[1].kappa, 0.0) << "left turn must yield positive curvature";
  EXPECT_NEAR(out[0].heading, 0.0, 1e-9);
  EXPECT_NEAR(out[1].heading, M_PI / 4.0, 1e-9);
  EXPECT_NEAR(out[2].heading, M_PI / 2.0, 1e-9);
}

TEST(DiscretePointsMath, RightTurnHasNegativeCurvature) {
  // 先沿 +x 再沿 -y 是右转（顺时针），曲率应为负。
  const auto xy = makeXY({0.0, 1.0, 1.0, 1.0}, {0.0, 0.0, -1.0, -2.0});
  std::vector<TrajectoryPoint> out;
  ASSERT_TRUE(ComputePathProfile(xy, &out));

  EXPECT_LT(out[1].kappa, 0.0) << "right turn must yield negative curvature";
}

TEST(DiscretePointsMath, EndpointsUseOneSidedDifferences) {
  const auto xy = makeXY({0.0, 1.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 2.0});
  std::vector<TrajectoryPoint> out;
  ASSERT_TRUE(ComputePathProfile(xy, &out));

  EXPECT_NEAR(out.front().heading, 0.0, 1e-9);
  EXPECT_NEAR(out.back().heading, M_PI / 2.0, 1e-9);
  EXPECT_TRUE(std::isfinite(out.front().kappa));
  EXPECT_TRUE(std::isfinite(out.back().kappa));
}

TEST(DiscretePointsMath, RejectsDuplicatePoints) {
  const auto xy = makeXY({0.0, 0.0, 1.0, 2.0}, {0.0, 0.0, 0.0, 0.0});
  std::vector<TrajectoryPoint> out;
  EXPECT_FALSE(ComputePathProfile(xy, &out));
  EXPECT_TRUE(out.empty());
}

TEST(DiscretePointsMath, RejectsNonAdjacentRepeatedNeighbor) {
  const auto xy = makeXY({0.0, 1.0, 0.0}, {0.0, 0.0, 0.0});
  std::vector<TrajectoryPoint> out;
  EXPECT_FALSE(ComputePathProfile(xy, &out));
  EXPECT_TRUE(out.empty());
}

TEST(DiscretePointsMath, QuarterCircleCurvatureApproximatesTrueValue) {
  // 半径为 2 米、按 30 度间隔采样的四分之一圆。有限差分估计较粗，
  // 因此允许较大容差。本测试关注符号和数量级，而不是精确数值。
  const double r = 2.0;
  std::vector<double> xs, ys;
  for (int i = 0; i <= 3; ++i) {
    const double th = i * (M_PI / 6.0);
    xs.push_back(r * std::cos(th));
    ys.push_back(r * std::sin(th));
  }
  std::vector<TrajectoryPoint> out;
  ASSERT_TRUE(ComputePathProfile(makeXY(xs, ys), &out));

  // 真实曲率为 1/r = 0.5，逆时针圆弧的曲率为正。
  EXPECT_GT(out[1].kappa, 0.0);
  EXPECT_GT(out[1].kappa, 0.5 * 0.5) << "coarse sampling underestimates, allow 50%";
  EXPECT_LT(out[1].kappa, 0.5 * 2.0);
}
