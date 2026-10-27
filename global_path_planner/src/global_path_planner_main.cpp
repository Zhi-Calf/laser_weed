#include <rclcpp/rclcpp.hpp>

#include "global_path_planner/global_path_planner.hpp"

// ============================================================
// 全局路径规划节点入口。
// 只负责初始化 ROS、构造节点并进入事件循环; 参数、规划和发布逻辑
// 均在 GlobalPathPlannerNode 内。
// ============================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<global_path_planner::GlobalPathPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
