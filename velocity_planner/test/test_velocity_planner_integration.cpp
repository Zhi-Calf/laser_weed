// 版权所有 2026。合成点云避停节点集成测试。
#include "velocity_planner/velocity_planner_node.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <weeder_msgs/msg/obstacle_status.hpp>

namespace
{

constexpr const char * kCloudTopic = "/rslidar_points";
constexpr const char * kStatusTopic = "/velocity_planner/status";

struct Point3
{
  float x;  // x 坐标
  float y;  // y 坐标
  float z;  // z 坐标
};

sensor_msgs::msg::PointCloud2 MakeCloud(
  const std::vector<Point3> & points,
  const std::string & frame_id)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  for (const auto & point : points) {
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  return cloud;
}

template<typename Predicate>
bool SpinUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
  const sensor_msgs::msg::PointCloud2 & cloud,
  double timeout_s,
  const Predicate & predicate)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline) {
    publisher->publish(cloud);
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return predicate();
}

}  // namespace

TEST(VelocityPlannerIntegration, PublishesStopThenClearAfterHold)
{
  rclcpp::NodeOptions planner_options;
  planner_options.parameter_overrides({
    rclcpp::Parameter("point_cloud_timeout_s", 0.1),
    rclcpp::Parameter("stop_hold_s", 0.2),
    rclcpp::Parameter("min_obstacle_points", static_cast<int64_t>(2)),
  });

  auto tester = std::make_shared<rclcpp::Node>("velocity_planner_test");
  auto planner_node =
    std::make_shared<velocity_planner::VelocityPlannerNode>(planner_options);
  auto cloud_publisher =
    tester->create_publisher<sensor_msgs::msg::PointCloud2>(
    kCloudTopic, rclcpp::SensorDataQoS());

  weeder_msgs::msg::ObstacleStatus last_status;
  bool has_status = false;
  auto status_subscription =
    tester->create_subscription<weeder_msgs::msg::ObstacleStatus>(
    kStatusTopic, 10,
    [&last_status, &has_status](
      const weeder_msgs::msg::ObstacleStatus::SharedPtr msg) {
      last_status = *msg;
      has_status = true;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tester);
  executor.add_node(planner_node);
  const bool observed_stop = SpinUntil(
    executor, cloud_publisher,
    MakeCloud({{1.0F, 0.0F, 0.5F}, {1.2F, 0.1F, 0.5F}}, "rslidar"),
    2.0,
    [&last_status, &has_status]() {
      return has_status && last_status.sensor_valid &&
             last_status.stop_required;
    });

  ASSERT_TRUE(observed_stop);
  EXPECT_TRUE(last_status.sensor_valid);
  EXPECT_TRUE(last_status.stop_required);
  EXPECT_EQ(last_status.obstacle_points, 2U);
  EXPECT_EQ(last_status.header.frame_id, "lidar");

  const bool observed_clear = SpinUntil(
    executor, cloud_publisher,
    MakeCloud({{4.0F, 0.0F, 0.5F}}, "rslidar"),
    2.0,
    [&last_status, &has_status]() {
      return has_status && last_status.sensor_valid &&
             !last_status.stop_required;
    });

  ASSERT_TRUE(observed_clear);
  EXPECT_TRUE(last_status.sensor_valid);
  EXPECT_FALSE(last_status.stop_required);
  EXPECT_EQ(last_status.obstacle_points, 0U);
  EXPECT_EQ(last_status.header.frame_id, "lidar");

  // 点云超过固定心跳超时后，应失效并输出安全停车。
  has_status = false;
  const auto quiet_deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(350);
  while (std::chrono::steady_clock::now() < quiet_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(has_status);
  EXPECT_FALSE(last_status.sensor_valid);
  EXPECT_TRUE(last_status.stop_required);
  EXPECT_EQ(last_status.obstacle_points, 0U);

  executor.remove_node(planner_node);
  executor.remove_node(tester);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
