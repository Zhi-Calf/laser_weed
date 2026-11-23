#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "comm_gateway/mqtt_bridge_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<comm_gateway::MqttBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
