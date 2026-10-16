#include <rclcpp/rclcpp.hpp>

#include "gnss_localization/sdk_client.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<gnss_localization::SdkClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
