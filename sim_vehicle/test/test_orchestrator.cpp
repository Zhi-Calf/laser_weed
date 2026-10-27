#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "sim_vehicle/sim_orchestrator_node.hpp"
#include "std_msgs/msg/bool.hpp"
#include "weeder_msgs/msg/path.hpp"

namespace sim_vehicle {
namespace {

// 等待路径订阅和工作模式发布端完成匹配。
void WaitForDiscovery(rclcpp::executors::SingleThreadedExecutor& executor) {
  for (int index = 0; index < 20; ++index) {
    executor.spin_some(std::chrono::milliseconds(10));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

}   // namespace

// 验证收到有效路径后编排节点会发布 UNMANNED。
TEST(SimOrchestratorNodeTest, PublishesUnmannedAfterPath) {
  rclcpp::init(0, nullptr);
  auto options = rclcpp::NodeOptions();
  options.parameter_overrides({rclcpp::Parameter("timeout_path_s", 2.0)});
  auto orchestrator = std::make_shared<SimOrchestratorNode>(options);
  auto helper       = std::make_shared<rclcpp::Node>("orchestrator_test_helper");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(orchestrator);
  executor.add_node(helper);

  std::atomic<bool> received_unmanned{false};
  const auto latched_qos   = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  auto path_publisher      = helper->create_publisher<weeder_msgs::msg::Path>("/global_path_planner/path", latched_qos);
  auto status_subscription = helper->create_subscription<std_msgs::msg::Bool>("/sim/unmanned", latched_qos, [&received_unmanned](const std_msgs::msg::Bool::SharedPtr message) {
    received_unmanned.store(message->data);
  });

  WaitForDiscovery(executor);
  weeder_msgs::msg::Path path;
  path.x              = {0.0, 1.0};
  path.y              = {0.0, 0.0};
  path.waypoint_types = {0, 0};
  path_publisher->publish(path);

  for (int index = 0; index < 100 && !received_unmanned.load(); ++index) {
    executor.spin_some(std::chrono::milliseconds(10));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(received_unmanned.load());

  executor.remove_node(orchestrator);
  executor.remove_node(helper);
  status_subscription.reset();
  path_publisher.reset();
  helper.reset();
  orchestrator.reset();
  rclcpp::shutdown();
}

}   // namespace sim_vehicle
