#include <rclcpp/rclcpp.hpp>

#include "gnss_localization/gnss_driver.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<gnss_localization::GnssDriver>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
