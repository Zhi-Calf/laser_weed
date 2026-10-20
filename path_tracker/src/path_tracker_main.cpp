// Copyright 2026. Entry point for the path_tracker node.
#include "path_tracker/path_tracker_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_tracker::PathTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
