#include <cmath>
#include <cstddef>

#include "gtest/gtest.h"
#include "sim_vehicle/kinematics.hpp"

namespace sim_vehicle {
namespace {

constexpr std::size_t FrontLeft =
  static_cast<std::size_t>(WheelIndex::kFrontLeft);
constexpr std::size_t FrontRight =
  static_cast<std::size_t>(WheelIndex::kFrontRight);
constexpr std::size_t RearLeft =
  static_cast<std::size_t>(WheelIndex::kRearLeft);
constexpr std::size_t RearRight =
  static_cast<std::size_t>(WheelIndex::kRearRight);

void ExpectNear(double actual, double expected) {
  EXPECT_NEAR(actual, expected, 1e-9);
}

}  // namespace

// 验证角度归一化保持零值并处理正负边界。
TEST(KinematicsTest, WrapAngleUsesHalfTurnOffset) {
  constexpr double Pi = 3.14159265358979323846;
  ExpectNear(WrapAngle(0.0), 0.0);
  ExpectNear(WrapAngle(Pi * 1.5), -Pi * 0.5);
  ExpectNear(WrapAngle(-Pi * 1.5), Pi * 0.5);
}

// 验证默认几何参数与 Isaac 场景一致。
TEST(KinematicsTest, GeometryMatchesIsaacStage) {
  ExpectNear(kWheelbaseM, 1.521);
  ExpectNear(kWheelTrackM, 1.306);
  ExpectNear(kWheelRadiusM, 0.29);
  ExpectNear(kVehicleLengthM, 1.8);
  ExpectNear(kVehicleWidthM, 1.2);
  ExpectNear(kVehicleHeightM, 0.5);
}

// 验证可视化轮位始终以车体几何中心为原点。
TEST(KinematicsTest, CenteredWheelPositionsDoNotChangeWithMode) {
  const auto positions =
    ComputeCenteredWheelPositions(kWheelbaseM, kWheelTrackM);
  ExpectNear(positions[FrontLeft].x_m, kWheelbaseM * 0.5);
  ExpectNear(positions[RearLeft].x_m, -kWheelbaseM * 0.5);
  ExpectNear(positions[FrontLeft].y_m, kWheelTrackM * 0.5);
  ExpectNear(positions[FrontRight].y_m, -kWheelTrackM * 0.5);
}

// 验证两种转向模式使用不同的底盘参考点。
TEST(KinematicsTest, WheelPositionsFollowReferencePoint) {
  const auto rear_reference = ComputeWheelPositions(0);
  const auto center_reference = ComputeWheelPositions(1);

  ExpectNear(rear_reference[RearLeft].x_m, 0.0);
  ExpectNear(rear_reference[FrontLeft].x_m, kWheelbaseM);
  ExpectNear(center_reference[FrontLeft].x_m, kWheelbaseM * 0.5);
  ExpectNear(center_reference[RearLeft].x_m, -kWheelbaseM * 0.5);
  for (std::size_t index = 0; index < kWheelCount; ++index) {
    ExpectNear(rear_reference[index].y_m, center_reference[index].y_m);
  }
}

// 验证直行时四个车轮均回正且轮速一致。
TEST(KinematicsTest, StraightMotionKeepsWheelsStraight) {
  const FourWheelState state = ComputeFourWheelState(0.6, 0.0, 0);
  ExpectNear(state.yaw_rate_rad_s, 0.0);
  for (std::size_t index = 0; index < kWheelCount; ++index) {
    ExpectNear(state.steer_rad[index], 0.0);
    ExpectNear(state.wheel_rad_s[index], 0.6 / kWheelRadiusM);
  }
}

// 验证单阿克曼模式仅前轮转向。
TEST(KinematicsTest, SingleAckermannSteersFrontWheels) {
  const double steer_rad = 15.0 * 3.14159265358979323846 / 180.0;
  const FourWheelState state = ComputeFourWheelState(0.6, steer_rad, 0);
  EXPECT_GT(state.steer_rad[FrontLeft], 0.0);
  EXPECT_GT(state.steer_rad[FrontRight], 0.0);
  EXPECT_GT(state.steer_rad[FrontLeft], state.steer_rad[FrontRight]);
  ExpectNear(state.steer_rad[RearLeft], 0.0);
  ExpectNear(state.steer_rad[RearRight], 0.0);
  ExpectNear(state.effective_wheelbase_m, kWheelbaseM);
}

// 验证对称阿克曼模式前后轮反向转动。
TEST(KinematicsTest, SymmetricAckermannTurnsOpposite) {
  const double steer_rad = 15.0 * 3.14159265358979323846 / 180.0;
  const FourWheelState state = ComputeFourWheelState(0.6, steer_rad, 1);
  EXPECT_GT(state.steer_rad[FrontLeft], 0.0);
  EXPECT_GT(state.steer_rad[FrontRight], 0.0);
  EXPECT_LT(state.steer_rad[RearLeft], 0.0);
  EXPECT_LT(state.steer_rad[RearRight], 0.0);
  ExpectNear(state.steer_rad[FrontLeft], -state.steer_rad[RearLeft]);
  ExpectNear(state.effective_wheelbase_m, kWheelbaseM * 0.5);
}

// 验证未定义运动语义的模式保持停车状态。
TEST(KinematicsTest, UnsupportedModesStopWheels) {
  const FourWheelState state = ComputeFourWheelState(0.6, 0.2, 2);
  ExpectNear(state.yaw_rate_rad_s, 0.0);
  for (std::size_t index = 0; index < kWheelCount; ++index) {
    ExpectNear(state.steer_rad[index], 0.0);
    ExpectNear(state.wheel_rad_s[index], 0.0);
  }
}

// 验证零速度时四个车轮的角速度为零。
TEST(KinematicsTest, ZeroSpeedStopsWheelRotation) {
  const FourWheelState state = ComputeFourWheelState(0.0, 0.3, 1);
  ExpectNear(state.yaw_rate_rad_s, 0.0);
  for (std::size_t index = 0; index < kWheelCount; ++index) {
    ExpectNear(state.wheel_rad_s[index], 0.0);
  }
}

}  // namespace sim_vehicle
