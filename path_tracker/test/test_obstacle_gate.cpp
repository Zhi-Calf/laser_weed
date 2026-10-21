// Copyright 2026. 点云避停门控单元测试。
#include "path_tracker/obstacle_gate.hpp"

#include <gtest/gtest.h>

using path_tracker::EvaluateObstacleGate;
using path_tracker::ObstacleGateDecision;

TEST(ObstacleGate, DisabledGateAllowsMissingStatus)
{
  EXPECT_EQ(
    EvaluateObstacleGate(false, false, 0.0, 0.5, false, true),
    ObstacleGateDecision::kAllow);
}

TEST(ObstacleGate, MissingStatusStops)
{
  EXPECT_EQ(
    EvaluateObstacleGate(true, false, 0.0, 0.5, false, true),
    ObstacleGateDecision::kUnavailable);
}

TEST(ObstacleGate, StaleStatusStops)
{
  EXPECT_EQ(
    EvaluateObstacleGate(true, true, 0.51, 0.5, true, false),
    ObstacleGateDecision::kUnavailable);
}

TEST(ObstacleGate, InvalidSensorStops)
{
  EXPECT_EQ(
    EvaluateObstacleGate(true, true, 0.1, 0.5, false, true),
    ObstacleGateDecision::kSensorInvalid);
}

TEST(ObstacleGate, StopRequestStops)
{
  EXPECT_EQ(
    EvaluateObstacleGate(true, true, 0.1, 0.5, true, true),
    ObstacleGateDecision::kStopRequired);
}

TEST(ObstacleGate, FreshClearStatusAllowsMotion)
{
  EXPECT_EQ(
    EvaluateObstacleGate(true, true, 0.1, 0.5, true, false),
    ObstacleGateDecision::kAllow);
}
