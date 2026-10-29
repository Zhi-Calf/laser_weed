// 版权所有 2026。path_tracker 点云避停与分段调度节点集成测试。
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>
#include <weeder_msgs/msg/chassis_cmd.hpp>
#include <weeder_msgs/msg/crop_row_observation.hpp>
#include <weeder_msgs/msg/obstacle_status.hpp>
#include <weeder_msgs/msg/path.hpp>
#include <weeder_msgs/msg/pose.hpp>
#include <weeder_msgs/msg/work_mode.hpp>

#include "path_tracker/path_tracker_node.hpp"

namespace {

using weeder_msgs::msg::ChassisCmd;
using weeder_msgs::msg::CropRowObservation;
using weeder_msgs::msg::ObstacleStatus;
using weeder_msgs::msg::Path;
using weeder_msgs::msg::Pose;
using weeder_msgs::msg::WorkMode;

struct TestPublishers {
  rclcpp::Publisher<Pose>::SharedPtr pose;                 // 定位位姿
  rclcpp::Publisher<Path>::SharedPtr path;                 // 路径
  rclcpp::Publisher<WorkMode>::SharedPtr mode;             // 工作模式
  rclcpp::Publisher<ObstacleStatus>::SharedPtr obstacle;   // 避停状态
  rclcpp::Publisher<CropRowObservation>::SharedPtr crop;   // 稻行观测
};

Pose MakePose(bool valid = true) {
  Pose pose;
  pose.header.frame_id = "odom";
  pose.valid           = valid;
  pose.satellites      = 30;
  return pose;
}

Path MakePath() {
  Path path;
  path.header.frame_id = "odom";
  for (int index = 0; index <= 50; ++index) {
    path.x.push_back(static_cast<double>(index) * 0.2);
    path.y.push_back(0.0);
    path.waypoint_types.push_back(0);
  }
  return path;
}

Path MakeSegmentPath() {
  Path path;
  path.header.frame_id = "odom";
  for (int index = 0; index <= 100; ++index) {
    path.x.push_back(static_cast<double>(index) * 0.1);
    path.y.push_back(0.0);
    path.waypoint_types.push_back((index >= 20 && index <= 29) ? 1U : 0U);
  }
  return path;
}

WorkMode MakeUnmannedMode() {
  WorkMode mode;
  mode.mode      = WorkMode::MODE_UNMANNED;
  mode.mode_name = "unmanned";
  return mode;
}

ObstacleStatus MakeObstacleStatus(bool stop_required) {
  ObstacleStatus status;
  status.sensor_valid       = true;
  status.stop_required      = stop_required;
  status.obstacle_points    = stop_required ? 3U : 0U;
  status.nearest_distance_m = stop_required ? 1.0F : -1.0F;
  return status;
}

CropRowObservation MakeCropRowObservation(double lateral_error_m = 0.2) {
  CropRowObservation observation;
  observation.header.frame_id   = "base_link";
  observation.session_id        = "row-vision-test";
  observation.has_capture_time  = true;
  observation.replay            = false;
  observation.status            = CropRowObservation::VALID;
  observation.lateral_error_m   = lateral_error_m;
  observation.heading_error_rad = 0.0;
  observation.lookahead_m       = 2.0;
  observation.confidence        = 0.9;
  observation.processing_ms     = 1.0;
  observation.frame_age_ms      = 1.0;
  return observation;
}

void SpinFor(rclcpp::executors::SingleThreadedExecutor& executor, const TestPublishers& publishers, const Pose& pose, const Path& path, const WorkMode& mode, const ObstacleStatus* obstacle, double duration_s) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(duration_s);
  while (std::chrono::steady_clock::now() < deadline) {
    publishers.pose->publish(pose);
    publishers.path->publish(path);
    publishers.mode->publish(mode);
    if (obstacle != nullptr) {
      publishers.obstacle->publish(*obstacle);
    }
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

void SpinForWithCropRow(
  rclcpp::executors::SingleThreadedExecutor& executor,
  const TestPublishers& publishers,
  const rclcpp::Node::SharedPtr& clock_node,
  const Pose& pose,
  const Path& path,
  const WorkMode& mode,
  const CropRowObservation& observation,
  std::uint64_t& sequence,
  double duration_s,
  double capture_age_s = 0.0) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(duration_s);
  while (std::chrono::steady_clock::now() < deadline) {
    publishers.pose->publish(pose);
    publishers.path->publish(path);
    publishers.mode->publish(mode);
    auto crop_row_msg         = observation;
    crop_row_msg.header.stamp = clock_node->now() - rclcpp::Duration::from_seconds(capture_age_s);
    crop_row_msg.sequence     = ++sequence;
    publishers.crop->publish(crop_row_msg);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

}   // namespace

TEST(PathTrackerObstacleIntegration, GatesChassisCommandFromStatus) {
  rclcpp::NodeOptions tracker_options;
  tracker_options.parameter_overrides({
    rclcpp::Parameter("control_hz", 20.0),
    rclcpp::Parameter("require_velocity_planner", true),
    rclcpp::Parameter("velocity_planner_status_timeout_s", 1.0),
    rclcpp::Parameter("max_lateral_error_m", 0.3),
    rclcpp::Parameter("speed_ramp_mps2", 10.0),
    rclcpp::Parameter("viz", false),
  });

  auto tester  = std::make_shared<rclcpp::Node>("path_tracker_obstacle_test");
  auto tracker = std::make_shared<path_tracker::PathTrackerNode>(tracker_options);

  ChassisCmd last_command;
  bool has_command          = false;
  auto command_subscription = tester->create_subscription<ChassisCmd>("/chassis/cmd", 10, [&last_command, &has_command](const ChassisCmd::SharedPtr msg) {
    last_command = *msg;
    has_command  = true;
  });

  TestPublishers publishers;
  publishers.pose        = tester->create_publisher<Pose>("/gnss_driver/pose", rclcpp::SensorDataQoS());
  const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  publishers.path        = tester->create_publisher<Path>("/global_path_planner/path", latched_qos);
  publishers.mode        = tester->create_publisher<WorkMode>("/work_mode", 10);
  publishers.obstacle    = tester->create_publisher<ObstacleStatus>("/velocity_planner/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  const auto pose = MakePose();
  const auto path = MakePath();
  const auto mode = MakeUnmannedMode();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tester);
  executor.add_node(tracker);

  SpinFor(executor, publishers, pose, path, mode, nullptr, 0.8);
  ASSERT_TRUE(has_command);
  EXPECT_TRUE(last_command.estop);
  EXPECT_DOUBLE_EQ(last_command.speed_m_s, 0.0);

  const auto clear_status = MakeObstacleStatus(false);
  SpinFor(executor, publishers, pose, path, mode, &clear_status, 0.5);
  ASSERT_TRUE(has_command);
  EXPECT_FALSE(last_command.estop);
  EXPECT_GT(last_command.speed_m_s, 0.0);

  const auto stop_status = MakeObstacleStatus(true);
  SpinFor(executor, publishers, pose, path, mode, &stop_status, 0.3);
  ASSERT_TRUE(has_command);
  EXPECT_TRUE(last_command.estop);
  EXPECT_DOUBLE_EQ(last_command.speed_m_s, 0.0);

  const auto invalid_pose = MakePose(false);
  SpinFor(executor, publishers, invalid_pose, path, mode, &clear_status, 0.4);
  ASSERT_TRUE(has_command);
  EXPECT_TRUE(last_command.estop);
  EXPECT_DOUBLE_EQ(last_command.speed_m_s, 0.0);

  auto lateral_pose = MakePose();
  lateral_pose.x    = 2.0;
  lateral_pose.y    = 0.2;
  SpinFor(executor, publishers, lateral_pose, path, mode, &clear_status, 0.4);
  ASSERT_TRUE(has_command);
  EXPECT_FALSE(last_command.estop);
  EXPECT_NEAR(last_command.speed_m_s, 0.6, 1e-6);

  lateral_pose.y = 0.29;
  SpinFor(executor, publishers, lateral_pose, path, mode, &clear_status, 0.4);
  EXPECT_FALSE(last_command.estop);
  EXPECT_NEAR(last_command.speed_m_s, 0.6, 1e-6);

  lateral_pose.y = 0.4;
  SpinFor(executor, publishers, lateral_pose, path, mode, &clear_status, 0.4);
  ASSERT_TRUE(has_command);
  EXPECT_TRUE(last_command.estop);
  EXPECT_DOUBLE_EQ(last_command.speed_m_s, 0.0);

  executor.remove_node(tracker);
  executor.remove_node(tester);
}

TEST(PathTrackerSegmentIntegration, SwitchesSpeedAndSteerModeByPathType) {
  rclcpp::NodeOptions tracker_options;
  tracker_options.parameter_overrides({
    rclcpp::Parameter("control_hz", 20.0),
    rclcpp::Parameter("require_velocity_planner", false),
    rclcpp::Parameter("viz", false),
    rclcpp::Parameter("wheelbase", 1.2),
    rclcpp::Parameter("work_speed_m_s", 0.6),
    rclcpp::Parameter("turn_speed_m_s", 0.3),
    rclcpp::Parameter("speed_ramp_mps2", 10.0),
    rclcpp::Parameter("turn_mode_enter_m", 0.6),
    rclcpp::Parameter("turn_mode_exit_m", 0.6),
  });

  auto tester  = std::make_shared<rclcpp::Node>("path_tracker_segment_test");
  auto tracker = std::make_shared<path_tracker::PathTrackerNode>(tracker_options);

  ChassisCmd last_command;
  bool has_command          = false;
  auto command_subscription = tester->create_subscription<ChassisCmd>("/chassis/cmd", 10, [&last_command, &has_command](const ChassisCmd::SharedPtr msg) {
    last_command = *msg;
    has_command  = true;
  });

  TestPublishers publishers;
  publishers.pose        = tester->create_publisher<Pose>("/gnss_driver/pose", rclcpp::SensorDataQoS());
  const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  publishers.path        = tester->create_publisher<Path>("/global_path_planner/path", latched_qos);
  publishers.mode        = tester->create_publisher<WorkMode>("/work_mode", 10);
  publishers.obstacle    = tester->create_publisher<ObstacleStatus>("/velocity_planner/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  auto pose       = MakePose();
  pose.x          = 0.8;
  const auto path = MakeSegmentPath();
  const auto mode = MakeUnmannedMode();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tester);
  executor.add_node(tracker);

  SpinFor(executor, publishers, pose, path, mode, nullptr, 0.4);
  ASSERT_TRUE(has_command);
  EXPECT_EQ(last_command.steer_mode, 0U);
  EXPECT_NEAR(last_command.speed_m_s, 0.6, 1e-6);

  pose.x = 1.5;
  SpinFor(executor, publishers, pose, path, mode, nullptr, 0.4);
  EXPECT_EQ(last_command.steer_mode, 1U);
  EXPECT_NEAR(last_command.speed_m_s, 0.3, 1e-6);

  pose.x = 3.2;
  SpinFor(executor, publishers, pose, path, mode, nullptr, 0.4);
  EXPECT_EQ(last_command.steer_mode, 1U);
  EXPECT_NEAR(last_command.speed_m_s, 0.3, 1e-6);

  pose.x = 4.0;
  SpinFor(executor, publishers, pose, path, mode, nullptr, 0.4);
  EXPECT_EQ(last_command.steer_mode, 0U);
  EXPECT_NEAR(last_command.speed_m_s, 0.6, 1e-6);

  executor.remove_node(tracker);
  executor.remove_node(tester);
}

TEST(PathTrackerSegmentIntegration, IgnoresPathUntilWorkModeChanges) {
  rclcpp::NodeOptions tracker_options;
  tracker_options.parameter_overrides({
    rclcpp::Parameter("control_hz", 20.0),
    rclcpp::Parameter("require_velocity_planner", false),
    rclcpp::Parameter("viz", false),
    rclcpp::Parameter("speed_ramp_mps2", 10.0),
  });

  auto tester  = std::make_shared<rclcpp::Node>("path_tracker_reload_test");
  auto tracker = std::make_shared<path_tracker::PathTrackerNode>(tracker_options);

  ChassisCmd last_command;
  bool has_command          = false;
  auto command_subscription = tester->create_subscription<ChassisCmd>("/chassis/cmd", 10, [&last_command, &has_command](const ChassisCmd::SharedPtr msg) {
    last_command = *msg;
    has_command  = true;
  });

  TestPublishers publishers;
  publishers.pose        = tester->create_publisher<Pose>("/gnss_driver/pose", rclcpp::SensorDataQoS());
  const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  publishers.path        = tester->create_publisher<Path>("/global_path_planner/path", latched_qos);
  publishers.mode        = tester->create_publisher<WorkMode>("/work_mode", 10);
  publishers.obstacle    = tester->create_publisher<ObstacleStatus>("/velocity_planner/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  auto pose              = MakePose();
  pose.x                 = 1.5;
  const auto first_path  = MakePath();
  const auto second_path = MakeSegmentPath();
  auto mode              = MakeUnmannedMode();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tester);
  executor.add_node(tracker);

  SpinFor(executor, publishers, pose, first_path, mode, nullptr, 0.3);
  ASSERT_TRUE(has_command);
  EXPECT_EQ(last_command.steer_mode, 0U);

  SpinFor(executor, publishers, pose, second_path, mode, nullptr, 0.3);
  EXPECT_EQ(last_command.steer_mode, 0U);
  EXPECT_NEAR(last_command.speed_m_s, 0.6, 1e-6);

  mode.mode      = WorkMode::MODE_MANUAL;
  mode.mode_name = "manual";
  SpinFor(executor, publishers, pose, second_path, mode, nullptr, 0.2);
  EXPECT_DOUBLE_EQ(last_command.speed_m_s, 0.0);
  EXPECT_FALSE(last_command.ctrl_mode);

  mode = MakeUnmannedMode();
  SpinFor(executor, publishers, pose, second_path, mode, nullptr, 0.3);
  EXPECT_EQ(last_command.steer_mode, 1U);
  EXPECT_NEAR(last_command.speed_m_s, 0.3, 1e-6);

  executor.remove_node(tracker);
  executor.remove_node(tester);
}

TEST(PathTrackerSteerRateIntegration, LimitsSteeringCommandRate) {
  constexpr double kControlHz       = 20.0;
  constexpr double kMaxSteerRateDeg = 18.0;

  rclcpp::NodeOptions tracker_options;
  tracker_options.parameter_overrides({
    rclcpp::Parameter("control_hz", kControlHz),
    rclcpp::Parameter("require_velocity_planner", false),
    rclcpp::Parameter("viz", false),
    rclcpp::Parameter("wheelbase", 1.2),
    rclcpp::Parameter("max_lateral_error_m", 0.3),
    rclcpp::Parameter("max_steer_rate_deg_s", kMaxSteerRateDeg),
    rclcpp::Parameter("speed_ramp_mps2", 10.0),
  });

  auto tester  = std::make_shared<rclcpp::Node>("path_tracker_steer_rate_test");
  auto tracker = std::make_shared<path_tracker::PathTrackerNode>(tracker_options);

  std::vector<ChassisCmd> commands;
  auto command_subscription = tester->create_subscription<ChassisCmd>("/chassis/cmd", 10, [&commands](const ChassisCmd::SharedPtr msg) {
    commands.push_back(*msg);
  });

  TestPublishers publishers;
  publishers.pose        = tester->create_publisher<Pose>("/gnss_driver/pose", rclcpp::SensorDataQoS());
  const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  publishers.path        = tester->create_publisher<Path>("/global_path_planner/path", latched_qos);
  publishers.mode        = tester->create_publisher<WorkMode>("/work_mode", 10);
  publishers.obstacle    = tester->create_publisher<ObstacleStatus>("/velocity_planner/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  auto pose       = MakePose();
  pose.x          = 0.2;
  pose.y          = 0.29;
  const auto path = MakePath();
  const auto mode = MakeUnmannedMode();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tester);
  executor.add_node(tracker);

  SpinFor(executor, publishers, pose, path, mode, nullptr, 1.0);
  ASSERT_GE(commands.size(), 3U);

  const double max_step_rad = kMaxSteerRateDeg / kControlHz * M_PI / 180.0;
  for (std::size_t index = 1; index < commands.size(); ++index) {
    EXPECT_LE(std::abs(commands[index].steer_angle_rad - commands[index - 1].steer_angle_rad), max_step_rad + 1e-9);
  }
  EXPECT_GT(std::abs(commands.back().steer_angle_rad), max_step_rad);

  executor.remove_node(tracker);
  executor.remove_node(tester);
}

TEST(PathTrackerVisionIntegration, TracksValidVisionAndFallsBackWhenStale) {
  rclcpp::NodeOptions tracker_options;
  tracker_options.parameter_overrides({
    rclcpp::Parameter("control_hz", 20.0),
    rclcpp::Parameter("require_velocity_planner", false),
    rclcpp::Parameter("viz", false),
    rclcpp::Parameter("wheelbase", 1.2),
    rclcpp::Parameter("max_lateral_error_m", 0.3),
    rclcpp::Parameter("max_steer_rate_deg_s", 180.0),
    rclcpp::Parameter("row_vision_enabled", true),
    rclcpp::Parameter("row_vision_resume_frames", 2),
    rclcpp::Parameter("crop_row_timeout_s", 0.2),
  });

  auto tester  = std::make_shared<rclcpp::Node>("path_tracker_vision_test");
  auto tracker = std::make_shared<path_tracker::PathTrackerNode>(tracker_options);

  ChassisCmd last_command;
  auto command_subscription = tester->create_subscription<ChassisCmd>("/chassis/cmd", 10, [&last_command](const ChassisCmd::SharedPtr msg) {
    last_command = *msg;
  });

  TestPublishers publishers;
  publishers.pose     = tester->create_publisher<Pose>("/gnss_driver/pose", rclcpp::SensorDataQoS());
  publishers.path     = tester->create_publisher<Path>("/global_path_planner/path", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  publishers.mode     = tester->create_publisher<WorkMode>("/work_mode", 10);
  publishers.obstacle = tester->create_publisher<ObstacleStatus>("/velocity_planner/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  publishers.crop     = tester->create_publisher<CropRowObservation>("/crop_row/observation", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  auto pose       = MakePose();
  pose.x          = 1.0;
  const auto path = MakePath();
  const auto mode = MakeUnmannedMode();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tester);
  executor.add_node(tracker);

  std::uint64_t sequence = 0;
  auto invalid_replay    = MakeCropRowObservation();
  invalid_replay.replay  = true;
  SpinForWithCropRow(executor, publishers, tester, pose, path, mode, invalid_replay, sequence, 0.3);
  EXPECT_NEAR(last_command.steer_angle_rad, 0.0, 1e-9);

  auto wrong_frame            = MakeCropRowObservation();
  wrong_frame.header.frame_id = "camera_optical_frame";
  SpinForWithCropRow(executor, publishers, tester, pose, path, mode, wrong_frame, sequence, 0.2);

  auto low_confidence       = MakeCropRowObservation();
  low_confidence.confidence = 0.49;
  SpinForWithCropRow(executor, publishers, tester, pose, path, mode, low_confidence, sequence, 0.2);

  auto non_finite            = MakeCropRowObservation();
  non_finite.lateral_error_m = std::numeric_limits<double>::quiet_NaN();
  SpinForWithCropRow(executor, publishers, tester, pose, path, mode, non_finite, sequence, 0.2);

  const auto stale_observation = MakeCropRowObservation();
  SpinForWithCropRow(executor, publishers, tester, pose, path, mode, stale_observation, sequence, 0.2, 0.5);

  const auto future_observation = MakeCropRowObservation();
  SpinForWithCropRow(executor, publishers, tester, pose, path, mode, future_observation, sequence, 0.2, -0.1);
  EXPECT_NEAR(last_command.steer_angle_rad, 0.0, 1e-9);

  const auto valid_observation = MakeCropRowObservation();
  SpinForWithCropRow(executor, publishers, tester, pose, path, mode, valid_observation, sequence, 0.5);
  EXPECT_GT(last_command.steer_angle_rad, 0.0);
  EXPECT_EQ(last_command.steer_mode, 0U);

  const double vision_steer = last_command.steer_angle_rad;
  SpinFor(executor, publishers, pose, path, mode, nullptr, 0.5);
  EXPECT_LT(std::abs(last_command.steer_angle_rad), std::abs(vision_steer));
  EXPECT_EQ(last_command.steer_mode, 0U);

  executor.remove_node(tracker);
  executor.remove_node(tester);
}

TEST(PathTrackerVisionIntegration, UsesGlobalPathAndTurnModeInsideWindow) {
  rclcpp::NodeOptions tracker_options;
  tracker_options.parameter_overrides({
    rclcpp::Parameter("control_hz", 20.0),
    rclcpp::Parameter("require_velocity_planner", false),
    rclcpp::Parameter("viz", false),
    rclcpp::Parameter("wheelbase", 1.2),
    rclcpp::Parameter("row_vision_enabled", true),
    rclcpp::Parameter("turn_mode_enter_m", 0.2),
    rclcpp::Parameter("turn_mode_exit_m", 0.2),
    rclcpp::Parameter("max_steer_rate_deg_s", 180.0),
  });

  auto tester  = std::make_shared<rclcpp::Node>("path_tracker_vision_turn_test");
  auto tracker = std::make_shared<path_tracker::PathTrackerNode>(tracker_options);

  ChassisCmd last_command;
  auto command_subscription = tester->create_subscription<ChassisCmd>("/chassis/cmd", 10, [&last_command](const ChassisCmd::SharedPtr msg) {
    last_command = *msg;
  });

  TestPublishers publishers;
  publishers.pose     = tester->create_publisher<Pose>("/gnss_driver/pose", rclcpp::SensorDataQoS());
  publishers.path     = tester->create_publisher<Path>("/global_path_planner/path", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  publishers.mode     = tester->create_publisher<WorkMode>("/work_mode", 10);
  publishers.obstacle = tester->create_publisher<ObstacleStatus>("/velocity_planner/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  publishers.crop     = tester->create_publisher<CropRowObservation>("/crop_row/observation", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  auto pose       = MakePose();
  pose.x          = 1.8;
  pose.y          = 0.2;
  const auto path = MakeSegmentPath();
  const auto mode = MakeUnmannedMode();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tester);
  executor.add_node(tracker);

  std::uint64_t crop_sequence      = 0;
  const auto prefilled_observation = MakeCropRowObservation(0.2);
  SpinForWithCropRow(executor, publishers, tester, pose, path, mode, prefilled_observation, crop_sequence, 0.5);
  EXPECT_EQ(last_command.steer_mode, 1U);
  EXPECT_LT(last_command.steer_angle_rad, 0.0);

  pose.x = 1.9;
  SpinFor(executor, publishers, pose, path, mode, nullptr, 0.4);
  EXPECT_EQ(last_command.steer_mode, 1U);

  pose.x = 3.5;
  SpinFor(executor, publishers, pose, path, mode, nullptr, 0.4);
  EXPECT_EQ(last_command.steer_mode, 0U);

  pose.x = 4.5;
  SpinFor(executor, publishers, pose, path, mode, nullptr, 0.4);
  EXPECT_EQ(last_command.steer_mode, 0U);

  const auto resumed_observation = MakeCropRowObservation(0.2);
  SpinForWithCropRow(executor, publishers, tester, pose, path, mode, resumed_observation, crop_sequence, 0.5);
  EXPECT_GT(last_command.steer_angle_rad, 0.0);
  EXPECT_EQ(last_command.steer_mode, 0U);

  executor.remove_node(tracker);
  executor.remove_node(tester);
}

TEST(PathTrackerPathValidationIntegration, IgnoresMalformedWaypointTypes) {
  rclcpp::NodeOptions tracker_options;
  tracker_options.parameter_overrides({
    rclcpp::Parameter("control_hz", 20.0),
    rclcpp::Parameter("require_velocity_planner", false),
    rclcpp::Parameter("viz", false),
    rclcpp::Parameter("speed_ramp_mps2", 10.0),
  });

  auto tester =
      std::make_shared<rclcpp::Node>("path_tracker_path_validation_test");
  auto tracker =
      std::make_shared<path_tracker::PathTrackerNode>(tracker_options);

  ChassisCmd last_command;
  bool has_command = false;
  auto command_subscription =
      tester->create_subscription<ChassisCmd>(
          "/chassis/cmd",
          10,
          [&last_command, &has_command](
              const ChassisCmd::SharedPtr msg) {
            last_command = *msg;
            has_command  = true;
          });

  TestPublishers publishers;
  publishers.pose = tester->create_publisher<Pose>(
      "/gnss_driver/pose",
      rclcpp::SensorDataQoS());
  publishers.path = tester->create_publisher<Path>(
      "/global_path_planner/path",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  publishers.mode = tester->create_publisher<WorkMode>(
      "/work_mode",
      10);
  publishers.obstacle = tester->create_publisher<ObstacleStatus>(
      "/velocity_planner/status",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  auto pose = MakePose();
  pose.x    = 1.0;
  const auto mode = MakeUnmannedMode();
  const auto path = MakePath();
  auto malformed_path = path;
  malformed_path.waypoint_types.pop_back();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tester);
  executor.add_node(tracker);

  SpinFor(
      executor,
      publishers,
      pose,
      malformed_path,
      mode,
      nullptr,
      0.3);
  ASSERT_TRUE(has_command);
  EXPECT_TRUE(last_command.estop);
  EXPECT_DOUBLE_EQ(last_command.speed_m_s, 0.0);

  SpinFor(executor, publishers, pose, path, mode, nullptr, 0.3);
  ASSERT_TRUE(has_command);
  EXPECT_FALSE(last_command.estop);
  EXPECT_GT(last_command.speed_m_s, 0.0);

  executor.remove_node(tracker);
  executor.remove_node(tester);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
