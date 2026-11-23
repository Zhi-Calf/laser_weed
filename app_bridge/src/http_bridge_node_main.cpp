#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "app_bridge/http_bridge_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<app_bridge::HttpBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
