#include <rclcpp/rclcpp.hpp>

#include "global_path_planner/global_path_planner.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<global_path_planner::GlobalPathPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
