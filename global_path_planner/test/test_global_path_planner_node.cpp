// 节点级契约: 用一组有效的 YAML 参数启动 GlobalPathPlannerNode, 验证它确实发布
// /global_path_planner/path, 且路径、类型数组和坐标系与发布端约定一致。
#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "global_path_planner/bow.hpp"
#include "global_path_planner/global_path_planner.hpp"
#include "weeder_msgs/msg/path.hpp"

namespace {

// 20 m x 10 m 的矩形，靠近 YAML 默认仿真区域；第一个顶点同时是 ENU 原点。
std::vector<double> TestPolygon() {
  constexpr double lat0               = 34.62;
  constexpr double lon0               = 112.45;
  constexpr double meters_per_deg_lat = 111320.0;
  const double meters_per_deg_lon     = meters_per_deg_lat * std::cos(lat0 * M_PI / 180.0);
  const double dlat                   = 10.0 / meters_per_deg_lat;
  const double dlon                   = 20.0 / meters_per_deg_lon;
  return {
    lat0,
    lon0,
    100.0,
    lat0,
    lon0 + dlon,
    100.0,
    lat0 + dlat,
    lon0 + dlon,
    100.0,
    lat0 + dlat,
    lon0,
    100.0,
  };
}

class GlobalPathPlannerNodeTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok())
      rclcpp::shutdown();
  }
};

struct TempDirGuard {
  std::filesystem::path path;
  ~TempDirGuard() {
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
  }
};

}   // namespace

TEST_F(GlobalPathPlannerNodeTest, PublishesPathWithMatchingTypesAndFrame) {
  const std::filesystem::path log_dir = std::filesystem::temp_directory_path() / ("global_path_planner_node_test_" + std::to_string(::getpid()));
  TempDirGuard cleanup{log_dir};

  auto options = rclcpp::NodeOptions();
  options.parameter_overrides(std::vector<rclcpp::Parameter>{
    {"log_dir", log_dir.string()},
    {"log_enable_console", false},
    {"log_level", "warn"},
    {"poly", TestPolygon()},
    {"ridge_width", 2.0},
    {"field_margin", 1.0},
    {"path_resolution", 0.5},
    {"first_row_margin", 0.005},
  });

  auto planner_node = std::make_shared<global_path_planner::GlobalPathPlannerNode>(options);
  auto listener     = std::make_shared<rclcpp::Node>("global_path_planner_node_test_listener");

  weeder_msgs::msg::Path::SharedPtr received;
  visualization_msgs::msg::Marker::SharedPtr work_points;
  visualization_msgs::msg::Marker::SharedPtr turn_points;
  auto subscription      = listener->create_subscription<weeder_msgs::msg::Path>("/global_path_planner/path", rclcpp::QoS(1).transient_local(), [&received](weeder_msgs::msg::Path::SharedPtr msg) {
    received = std::move(msg);
  });
  auto work_subscription = listener->create_subscription<visualization_msgs::msg::Marker>("/global_path_planner/work_points", 10, [&work_points](visualization_msgs::msg::Marker::SharedPtr msg) {
    work_points = std::move(msg);
  });
  auto turn_subscription = listener->create_subscription<visualization_msgs::msg::Marker>("/global_path_planner/turn_points", 10, [&turn_points](visualization_msgs::msg::Marker::SharedPtr msg) {
    turn_points = std::move(msg);
  });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner_node);
  executor.add_node(listener);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while ((!received || !work_points || !turn_points) && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(50));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  work_subscription.reset();
  turn_subscription.reset();
  subscription.reset();
  executor.remove_node(listener);
  executor.remove_node(planner_node);

  ASSERT_TRUE(received) << "5 s 内没有收到 /global_path_planner/path";
  ASSERT_FALSE(received->x.empty()) << "规划成功时路径不应为空";
  EXPECT_EQ(received->header.frame_id, "odom");
  ASSERT_EQ(received->x.size(), received->y.size());
  ASSERT_EQ(received->x.size(), received->waypoint_types.size());
  EXPECT_NE(std::find(received->waypoint_types.begin(), received->waypoint_types.end(), kSegTurn), received->waypoint_types.end()) << "20x10 地块应至少包含一段掉头";
  for (size_t i = 0; i < received->x.size(); ++i) {
    EXPECT_TRUE(std::isfinite(received->x[i])) << "第 " << i << " 个 x 不是有限数";
    EXPECT_TRUE(std::isfinite(received->y[i])) << "第 " << i << " 个 y 不是有限数";
  }

  ASSERT_TRUE(work_points) << "5 s 内没有收到 WORK 路径点 Marker";
  ASSERT_TRUE(turn_points) << "5 s 内没有收到 TURN 路径点 Marker";
  EXPECT_EQ(work_points->header.frame_id, "odom");
  EXPECT_EQ(turn_points->header.frame_id, "odom");
  EXPECT_EQ(work_points->type, visualization_msgs::msg::Marker::SPHERE_LIST);
  EXPECT_EQ(turn_points->type, visualization_msgs::msg::Marker::SPHERE_LIST);
  EXPECT_EQ(work_points->action, visualization_msgs::msg::Marker::ADD);
  EXPECT_EQ(turn_points->action, visualization_msgs::msg::Marker::ADD);
  EXPECT_GT(work_points->color.g, work_points->color.r);
  EXPECT_GT(turn_points->color.r, turn_points->color.g);
  EXPECT_EQ(work_points->points.size() + turn_points->points.size(), received->x.size());
}
