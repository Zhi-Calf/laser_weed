#include <rclcpp/rclcpp.hpp>

#include "gnss_localization/ntrip_client.hpp"

// ============================================================
// 外置 NTRIP 差分客户端入口。
// 只负责启动 NtripClient; 串口、NTRIP 和重连逻辑由节点自身管理。
// ============================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<gnss_localization::NtripClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
