// Copyright 2026. Unit tests for the path profile (heading / arc length / curvature).
#include "lqr_control/path_profile.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

using lqr_control::TrajectoryPoint;
using lqr_control::computePathProfile;

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
}  // namespace

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
  // Three segments of 1 m each -> total 3 m.
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
  // +x then +y is a left (counter-clockwise) turn; the CAN protocol treats left
  // as positive, so kappa must come out positive.
  const auto xy = makeXY({0.0, 1.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 2.0});
  std::vector<TrajectoryPoint> out;
  ASSERT_TRUE(computePathProfile(xy, 0.5, &out));

  EXPECT_GT(out[1].kappa, 0.0) << "left turn must yield positive curvature";
  EXPECT_NEAR(out[0].heading, 0.0, 1e-9);
  EXPECT_NEAR(out[1].heading, M_PI / 2.0, 1e-9);
}

TEST(PathProfile, RightTurnHasNegativeCurvature)
{
  // +x then -y is a right (clockwise) turn -> negative curvature.
  const auto xy = makeXY({0.0, 1.0, 1.0, 1.0}, {0.0, 0.0, -1.0, -2.0});
  std::vector<TrajectoryPoint> out;
  ASSERT_TRUE(computePathProfile(xy, 0.5, &out));

  EXPECT_LT(out[1].kappa, 0.0) << "right turn must yield negative curvature";
}

TEST(PathProfile, EndpointsInheritInteriorCurvature)
{
  // Endpoints have no two-sided stencil; they must inherit the nearest interior
  // value so the look-ahead does not see a fake zero-curvature at the path ends.
  const auto xy = makeXY({0.0, 1.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 2.0});
  std::vector<TrajectoryPoint> out;
  ASSERT_TRUE(computePathProfile(xy, 0.5, &out));

  EXPECT_DOUBLE_EQ(out.front().kappa, out[1].kappa);
  EXPECT_DOUBLE_EQ(out.back().kappa, out[out.size() - 2].kappa);
}

TEST(PathProfile, DuplicatePointsDoNotProduceNaN)
{
  // Zero-length segments would divide by zero in the curvature formula.
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
  // The duplicated point contributes zero length.
  const auto xy = makeXY({0.0, 0.0, 1.0, 2.0}, {0.0, 0.0, 0.0, 0.0});
  std::vector<TrajectoryPoint> out;
  ASSERT_TRUE(computePathProfile(xy, 0.5, &out));
  EXPECT_NEAR(out.back().s, 2.0, 1e-9);
}

TEST(PathProfile, QuarterCircleCurvatureApproximatesTrueValue)
{
  // A quarter circle of radius 2 m sampled at 30 deg steps. The finite-difference
  // estimate is coarse, so allow a generous tolerance - the point of this test is
  // the sign and the order of magnitude, not precision.
  const double r = 2.0;
  std::vector<double> xs, ys;
  for (int i = 0; i <= 3; ++i) {
    const double th = i * (M_PI / 6.0);
    xs.push_back(r * std::cos(th));
    ys.push_back(r * std::sin(th));
  }
  std::vector<TrajectoryPoint> out;
  ASSERT_TRUE(computePathProfile(makeXY(xs, ys), 0.5, &out));

  // True curvature is 1/r = 0.5, positive for a counter-clockwise arc.
  EXPECT_GT(out[1].kappa, 0.0);
  EXPECT_GT(out[1].kappa, 0.5 * 0.5) << "coarse sampling underestimates, allow 50%";
  EXPECT_LT(out[1].kappa, 0.5 * 2.0);
}
