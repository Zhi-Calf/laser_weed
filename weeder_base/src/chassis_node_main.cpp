#include <exception>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "weeder_base/chassis_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    auto node = std::make_shared<weeder_base::ChassisNode>();
    rclcpp::spin(node);
    // 显式释放: 确保 CAN 接收线程在 main 返回前 join 完成 (析构里做)
    node.reset();
  } catch (const std::exception& e) {
    // 构造函数里 CAN 打不开会抛到这里: 明确失败, 由 launch respawn 拉起重试
    RCLCPP_FATAL(rclcpp::get_logger("chassis_node"), "节点异常退出: %s", e.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
