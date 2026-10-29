// 版权所有 2026。速度与转角变化率限制单元测试。
#include <gtest/gtest.h>

#include "path_tracker/speed_limiter.hpp"

// 加速时每拍速度增量不得超过加速度上限乘以控制周期。
TEST(SpeedLimiter, LimitsAcceleration) {
  EXPECT_NEAR(
      path_tracker::LimitSpeed(0.6, 0.0, 0.5, 0.1),
      0.05,
      1e-12);
  EXPECT_NEAR(
      path_tracker::LimitSpeed(0.6, 0.05, 0.5, 0.1),
      0.10,
      1e-12);
}

// 减速时同样限制每拍速度变化量。
TEST(SpeedLimiter, LimitsDeceleration) {
  EXPECT_NEAR(
      path_tracker::LimitSpeed(0.0, 0.6, 0.5, 0.1),
      0.55,
      1e-12);
}

// 零控制周期或零加速度时保持上一拍速度。
TEST(SpeedLimiter, HoldsPreviousSpeedWithoutTimeOrAcceleration) {
  EXPECT_DOUBLE_EQ(
      path_tracker::LimitSpeed(0.6, 0.2, 0.5, 0.0),
      0.2);
  EXPECT_DOUBLE_EQ(
      path_tracker::LimitSpeed(0.6, 0.2, 0.0, 0.1),
      0.2);
}

// 正负转角均按最大角速度限制单拍变化量。
TEST(SpeedLimiter, LimitsSteerAngleRate) {
  EXPECT_NEAR(
      path_tracker::LimitSteerAngle(0.5, 0.1, 1.0, 0.1),
      0.2,
      1e-12);
  EXPECT_NEAR(
      path_tracker::LimitSteerAngle(-0.5, 0.1, 1.0, 0.1),
      0.0,
      1e-12);
}
