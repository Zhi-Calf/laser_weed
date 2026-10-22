// 版权所有 2026。路径剖面（航向、弧长和曲率）单元测试。
#include "path_tracker/path_profile.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

using path_tracker::TrajectoryPoint;
using path_tracker::computePathProfile;

namespace
{
std::vector<std::pair<double, double>> makeXY(
  const std::vector<double> & xs, const std::vector<double> & ys)
{
  std::vector<std::pair<double, double>> xy;
  xy.reserve(xs.size());
  for (size_t i = 0; i < xs.size(); ++i) {
    xy.emplace_back(xs[i], ys[i]);
  }
  return xy;
}
}  // 匿名命名空间

TEST(PathProfile, RejectsFewerThanTwoPoints)
{
  std::vector<TrajectoryPoint> out;
  EXPECT_FALSE(computePathProfile({}, 0.5, &out));
  EXPECT_TRUE(out.empty());
  EXPECT_FALSE(computePathProfile({{0.0, 0.0}}, 0.5, &out));
  EXPECT_TRUE(out.empty());
}

TEST(PathProfile, RejectsNullOutput)
{
  const auto xy = makeXY({0.0, 1.0}, {0.0, 0.0});
  EXPECT_FALSE(computePathProfile(xy, 0.5, nullptr));
}

TEST(PathProfile, StraightLineHasZeroCurvatureAndHeading)
{
  const auto xy = makeXY({0.0, 1.0, 2.0, 3.0, 4.0}, {0.0, 0.0, 0.0, 0.0, 0.0});
  std::vector<TrajectoryPoint> out;
  ASSERT_TRUE(computePathProfile(xy, 0.7, &out));
  ASSERT_EQ(out.size(), 5u);

  for (const auto & p : out) {
    EXPECT_NEAR(p.heading, 0.0, 1e-9);
    EXPECT_NEAR(p.kappa, 0.0, 1e-9);
    EXPECT_DOUBLE_EQ(p.v, 0.7) << "desired speed must be written into every point";
  }
}

TEST(PathProfile, ArcLengthAccumulates)
{
  // 三段各 1 米，总弧长应为 3 米。
  const auto xy = makeXY({0.0, 1.0, 2.0, 3.0}, {0.0, 0.0, 0.0, 0.0});
  std::vector<TrajectoryPoint> out;
  ASSERT_TRUE(computePathProfile(xy, 0.5, &out));

  EXPECT_NEAR(out.front().s, 0.0, 1e-9);
  EXPECT_NEAR(out[1].s, 1.0, 1e-9);
  EXPECT_NEAR(out[2].s, 2.0, 1e-9);
  EXPECT_NEAR(out.back().s, 3.0, 1e-9);
}

TEST(PathProfile, LeftTurnHasPositiveCurvature)
{
  // 先沿 +x 再沿 +y 是左转（逆时针）。CAN 协议规定左转为正，
  // 因此计算出的曲率必须为正。
  const auto xy = makeXY({0.0, 1.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 2.0});
  std::vector<TrajectoryPoint> out;
  ASSERT_TRUE(computePathProfile(xy, 0.5, &out));

  EXPECT_GT(out[1].kappa, 0.0) << "left turn must yield positive curvature";
  EXPECT_NEAR(out[0].heading, 0.0, 1e-9);
  EXPECT_NEAR(out[1].heading, M_PI / 2.0, 1e-9);
}

TEST(PathProfile, RightTurnHasNegativeCurvature)
{
  // 先沿 +x 再沿 -y 是右转（顺时针），曲率应为负。
  const auto xy = makeXY({0.0, 1.0, 1.0, 1.0}, {0.0, 0.0, -1.0, -2.0});
  std::vector<TrajectoryPoint> out;
  ASSERT_TRUE(computePathProfile(xy, 0.5, &out));

  EXPECT_LT(out[1].kappa, 0.0) << "right turn must yield negative curvature";
}

TEST(PathProfile, EndpointsInheritInteriorCurvature)
{
  // 端点没有双侧差分模板，因此继承最近的内点值，
  // 避免路径首尾被前瞻误判为零曲率。
  const auto xy = makeXY({0.0, 1.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 2.0});
  std::vector<TrajectoryPoint> out;
  ASSERT_TRUE(computePathProfile(xy, 0.5, &out));

  EXPECT_DOUBLE_EQ(out.front().kappa, out[1].kappa);
  EXPECT_DOUBLE_EQ(out.back().kappa, out[out.size() - 2].kappa);
}

TEST(PathProfile, DuplicatePointsDoNotProduceNaN)
{
  // 零长度线段会使曲率公式除零。
  const auto xy = makeXY({0.0, 0.0, 1.0, 2.0}, {0.0, 0.0, 0.0, 0.0});
  std::vector<TrajectoryPoint> out;
  ASSERT_TRUE(computePathProfile(xy, 0.5, &out));

  for (const auto & p : out) {
    EXPECT_TRUE(std::isfinite(p.kappa)) << "duplicate points must not yield inf/nan";
    EXPECT_TRUE(std::isfinite(p.heading));
    EXPECT_TRUE(std::isfinite(p.s));
  }
}

TEST(PathProfile, ArcLengthIgnoresDuplicateSegments)
{
  // 重复点贡献的弧长为零。
  const auto xy = makeXY({0.0, 0.0, 1.0, 2.0}, {0.0, 0.0, 0.0, 0.0});
  std::vector<TrajectoryPoint> out;
  ASSERT_TRUE(computePathProfile(xy, 0.5, &out));
  EXPECT_NEAR(out.back().s, 2.0, 1e-9);
}

TEST(PathProfile, QuarterCircleCurvatureApproximatesTrueValue)
{
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
  ASSERT_TRUE(computePathProfile(makeXY(xs, ys), 0.5, &out));

  // 真实曲率为 1/r = 0.5，逆时针圆弧的曲率为正。
  EXPECT_GT(out[1].kappa, 0.0);
  EXPECT_GT(out[1].kappa, 0.5 * 0.5) << "coarse sampling underestimates, allow 50%";
  EXPECT_LT(out[1].kappa, 0.5 * 2.0);
}
