#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "weeder_comm_gateway/http_bridge_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<weeder_comm_gateway::HttpBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
