#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "gnss_localization/gnss_driver.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    auto node = std::make_shared<gnss_localization::GnssDriver>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    // 串口是必需设备，打开失败时直接以非零状态退出。
    RCLCPP_FATAL(rclcpp::get_logger("gnss_driver"),
                 "节点初始化失败: %s", e.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
