#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sim_vehicle/sim_vehicle_node.hpp"

// ============================================================
// 四轮转向仿真车辆入口。
// 启动联合模拟 GNSS、底盘运动学和模式管理的 SimVehicleNode。
// ============================================================
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sim_vehicle::SimVehicleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
