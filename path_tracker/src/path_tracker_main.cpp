// 版权所有 2026。path_tracker 节点入口。
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "path_tracker/path_tracker_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_tracker::PathTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
