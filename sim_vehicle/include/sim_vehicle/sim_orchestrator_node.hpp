#ifndef SIM_VEHICLE_SIM_ORCHESTRATOR_NODE_H_
#define SIM_VEHICLE_SIM_ORCHESTRATOR_NODE_H_

#include <cstddef>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "weeder_msgs/msg/path.hpp"

namespace sim_vehicle {

// ============================================================
// 仿真实例编排状态机。
// 等待全局路径出现后发布一次 UNMANNED, 让 path_tracker 接管车辆;
// 路径超时只进入失败状态, 不负责关闭仿真进程。
// ============================================================
class SimOrchestratorNode : public rclcpp::Node {
public:
  explicit SimOrchestratorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  enum class Phase : std::uint8_t {
    kWaitPath   = 0,
    kGoUnmanned = 1,
    kDone       = 2,
    kFailed     = 3,
  };

  void Initialize();
  void LoadParameters();
  void SetupInterfaces();
  void SetupTimers();
  void OnPath(const weeder_msgs::msg::Path::SharedPtr message);
  void Tick();
  void HandleWaitPath(double elapsed_s);
  void HandleGoUnmanned();
  void Advance(Phase new_phase);
  void Fail(const std::string& reason, const std::string& hint);
  static const char* PhaseName(Phase phase);

  double timeout_path_s_{30.0};                                        // 等待路径超时时间
  Phase phase_{Phase::kWaitPath};                                      // 当前编排阶段
  rclcpp::Time phase_start_;                                           // 当前阶段开始时刻
  std::size_t got_path_points_{0};                                     // 最近路径的有效点数
  rclcpp::Subscription<weeder_msgs::msg::Path>::SharedPtr path_sub_;   // 全局路径订阅
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr unmanned_pub_;     // 无人模式切换发布
  rclcpp::TimerBase::SharedPtr timer_;                                 // 编排状态机定时器
};

}   // namespace sim_vehicle

#endif   // SIM_VEHICLE_SIM_ORCHESTRATOR_NODE_H_
