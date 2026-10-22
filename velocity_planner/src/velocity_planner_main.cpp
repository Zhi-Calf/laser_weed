// 版权所有 2026。点云避停节点入口。
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "velocity_planner/velocity_planner_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<velocity_planner::VelocityPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
