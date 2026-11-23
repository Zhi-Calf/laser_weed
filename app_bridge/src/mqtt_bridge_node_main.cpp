#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "app_bridge/mqtt_bridge_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<app_bridge::MqttBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
