#include <rclcpp/rclcpp.hpp>

#include "gnss_localization/ntrip_client.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<gnss_localization::NtripClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
