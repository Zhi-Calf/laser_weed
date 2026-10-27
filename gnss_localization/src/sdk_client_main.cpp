#include <rclcpp/rclcpp.hpp>

#include "gnss_localization/sdk_client.hpp"

// ============================================================
// 千寻 SDK 差分客户端入口。
// 只负责启动 SdkClient; SDK 会话、串口转发和重连由节点自身管理。
// ============================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<gnss_localization::SdkClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
