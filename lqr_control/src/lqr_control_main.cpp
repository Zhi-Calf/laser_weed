// Copyright 2026. Entry point for the lqr_control node.
#include "lqr_control/lqr_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lqr_control::LqrNode>());
  rclcpp::shutdown();
  return 0;
}
