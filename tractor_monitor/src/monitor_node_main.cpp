#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "tractor_monitor/monitor_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tractor_monitor::MonitorNode>());
  rclcpp::shutdown();
  return 0;
}
