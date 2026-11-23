#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sim_vehicle/sim_orchestrator_node.hpp"

// ============================================================
// 仿真编排器入口。
// 启动状态机节点; 路径就绪后由节点发布 UNMANNED 切换指令。
// ============================================================
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sim_vehicle::SimOrchestratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
