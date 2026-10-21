// Copyright 2026. 点云避停节点入口。
#include "velocity_planner/velocity_planner_node.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<velocity_planner::VelocityPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
