#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sim_vehicle/sim_vehicle_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sim_vehicle::SimVehicleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
