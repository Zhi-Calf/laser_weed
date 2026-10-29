// 版权所有 2026。LQR 输入计算单元测试。
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "path_tracker/lqr_input.hpp"

using path_tracker::LQRInputCalculator;
using path_tracker::LQRInputConfig;
using path_tracker::LQRInputResult;
using path_tracker::LineReference;
using path_tracker::RobotState;
using path_tracker::TrajectoryPoint;

namespace {

// 构造适用于路径输入测试的默认计算器。
LQRInputCalculator MakeCalculator(bool pose_at_vehicle_center = true) {
  LQRInputConfig config;
  config.wheelbase              = 1.2;
  config.pose_at_vehicle_center = pose_at_vehicle_center;
  LQRInputCalculator calculator;
  calculator.Configure(config);
  return calculator;
}

}   // namespace

// 路径输入必须使用目标航向计算左正横向误差和航向误差。
TEST(LQRInputCalculator, CalculatesPathErrors) {
  const LQRInputCalculator calculator = MakeCalculator();

  RobotState state;
  state.x       = 1.0;
  state.y       = 0.2;
  state.heading = 0.1;

  TrajectoryPoint target;
  target.x     = 1.0;
  target.y     = 0.0;
  target.kappa = 0.05;

  const LQRInputResult result = calculator.CalculatePath(state, target);
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.lateral_error, 0.2, 1e-12);
  EXPECT_NEAR(result.heading_error, 0.1, 1e-12);
  EXPECT_NEAR(result.target_kappa, 0.05, 1e-12);
}

// 航向误差必须跨越 +/-pi 归一化。
TEST(LQRInputCalculator, NormalizesHeadingAcrossPi) {
  const LQRInputCalculator calculator = MakeCalculator();

  RobotState state;
  state.heading = 3.13;
  TrajectoryPoint target;
  target.heading = -3.13;

  const LQRInputResult result = calculator.CalculatePath(state, target);
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.heading_error, 3.13 - (-3.13) - 2.0 * M_PI, 1e-12);
}

// 视觉直线在几何中心参考点处必须换算到后轴并反转符号。
TEST(LQRInputCalculator, ConvertsLineErrorsAtRearAxle) {
  const LQRInputCalculator calculator = MakeCalculator(true);
  LineReference reference;
  reference.lateral_error_m   = 0.2;
  reference.heading_error_rad = 0.1;
  reference.curvature         = 0.05;

  const LQRInputResult result = calculator.CalculateLine(reference);
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(
      result.lateral_error,
      -(0.2 - std::tan(0.1) * 0.6),
      1e-12);
  EXPECT_NEAR(result.heading_error, -0.1, 1e-12);
  EXPECT_NEAR(result.target_kappa, 0.05, 1e-12);
}

// 位姿已经位于后轴时不得再次偏移视觉横向误差。
TEST(LQRInputCalculator, KeepsLineErrorWhenPoseIsAtRearAxle) {
  const LQRInputCalculator calculator = MakeCalculator(false);
  LineReference reference;
  reference.lateral_error_m = 0.2;

  const LQRInputResult result = calculator.CalculateLine(reference);
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.lateral_error, -0.2, 1e-12);
}

// 非有限路径输入和无效视觉航向必须被拒绝。
TEST(LQRInputCalculator, RejectsInvalidInputs) {
  const LQRInputCalculator calculator = MakeCalculator();
  RobotState state;
  TrajectoryPoint target;
  state.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(calculator.CalculatePath(state, target).success);

  LineReference reference;
  reference.heading_error_rad = M_PI / 2.0;
  EXPECT_FALSE(calculator.CalculateLine(reference).success);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
