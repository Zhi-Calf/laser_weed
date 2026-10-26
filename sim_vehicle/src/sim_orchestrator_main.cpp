#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sim_vehicle/sim_orchestrator_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sim_vehicle::SimOrchestratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
