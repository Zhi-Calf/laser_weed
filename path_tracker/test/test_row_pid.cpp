#include <gtest/gtest.h>

#include <cmath>

#include "path_tracker/row_pid.hpp"

namespace {

using path_tracker::RowPidConfig;
using path_tracker::RowPidController;
using path_tracker::RowPidTarget;

TEST(RowPidControllerTest, LeftLineCommandsLeftSteer) {
  RowPidConfig config;
  config.ki_rad_per_m_s = 0.0;
  config.kd_rad_s_per_m = 0.0;
  RowPidController controller(config);

  RowPidTarget target;
  target.lateral_error_m = 0.1;
  target.lookahead_m     = 1.0;
  const auto output      = controller.Compute(target, 0.1);

  ASSERT_TRUE(output.valid);
  EXPECT_GT(output.steer_rad, 0.0);
}

TEST(RowPidControllerTest, PositiveHeadingErrorSteersLeft) {
  RowPidConfig config;
  config.kp_rad_per_m   = 0.0;
  config.ki_rad_per_m_s = 0.0;
  config.kd_rad_s_per_m = 0.0;
  RowPidController controller(config);

  RowPidTarget target;
  target.heading_error_rad = 0.1;
  target.lookahead_m       = 1.0;
  const auto output        = controller.Compute(target, 0.1);

  ASSERT_TRUE(output.valid);
  EXPECT_GT(output.steer_rad, 0.0);
  EXPECT_NEAR(output.cross_track_m, std::tan(0.1), 1e-12);
}

TEST(RowPidControllerTest, IntegralOutputIsLimited) {
  RowPidConfig config;
  config.kp_rad_per_m       = 0.0;
  config.ki_rad_per_m_s     = 10.0;
  config.kd_rad_s_per_m     = 0.0;
  config.heading_gain       = 0.0;
  config.integral_limit_rad = 0.1;
  RowPidController controller(config);

  RowPidTarget target;
  target.lateral_error_m = 1.0;
  target.lookahead_m     = 1.0;
  for (int index = 0; index < 20; ++index) {
    (void)controller.Compute(target, 0.1);
  }
  const auto output = controller.Compute(target, 0.1);

  ASSERT_TRUE(output.valid);
  EXPECT_NEAR(output.i_term_rad, 0.1, 1e-12);
}

TEST(RowPidControllerTest, DerivativeUsesFirstOrderLowPass) {
  RowPidConfig config;
  config.kp_rad_per_m     = 0.0;
  config.ki_rad_per_m_s   = 0.0;
  config.kd_rad_s_per_m   = 1.0;
  config.heading_gain     = 0.0;
  config.derivative_tau_s = 0.15;
  RowPidController controller(config);

  RowPidTarget target;
  target.lookahead_m = 1.0;
  (void)controller.Compute(target, 0.1);

  target.lateral_error_m = 0.1;
  const auto rising      = controller.Compute(target, 0.1);
  const auto settling    = controller.Compute(target, 0.1);

  EXPECT_NEAR(rising.d_term_rad, 0.4, 1e-12);
  EXPECT_NEAR(settling.d_term_rad, 0.24, 1e-12);
}

TEST(RowPidControllerTest, ResetClearsIntegralAndDerivativeState) {
  RowPidConfig config;
  config.kp_rad_per_m     = 1.0;
  config.ki_rad_per_m_s   = 1.0;
  config.kd_rad_s_per_m   = 1.0;
  config.heading_gain     = 0.0;
  config.derivative_tau_s = 0.1;
  RowPidController controller(config);

  RowPidTarget target;
  target.lateral_error_m = 0.2;
  target.lookahead_m     = 1.0;
  (void)controller.Compute(target, 0.1);
  (void)controller.Compute(target, 0.1);
  controller.Reset();
  const auto output = controller.Compute(target, 0.1);

  ASSERT_TRUE(output.valid);
  EXPECT_NEAR(output.p_term_rad, 0.2, 1e-12);
  EXPECT_GT(output.i_term_rad, 0.0);
  EXPECT_NEAR(output.d_term_rad, 0.0, 1e-12);
}

TEST(RowPidControllerTest, RejectsInvalidTarget) {
  RowPidController controller;
  RowPidTarget target;
  target.heading_error_rad = M_PI / 2.0;
  const auto output        = controller.Compute(target, 0.1);
  EXPECT_FALSE(output.valid);
}

}   // namespace
