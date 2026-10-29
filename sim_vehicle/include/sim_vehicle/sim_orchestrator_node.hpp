#ifndef SIM_VEHICLE_SIM_ORCHESTRATOR_NODE_H_
#define SIM_VEHICLE_SIM_ORCHESTRATOR_NODE_H_

#include <cstddef>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "weeder_msgs/msg/path.hpp"

namespace sim_vehicle {

// 仿真编排状态机：路径就绪后切换 UNMANNED，超时后进入失败状态。
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
  void LoadLogParameters();
  void SetupLogging();
  void LoadParameters();
  void SetupInterfaces();
  void SetupTimers();

private:
  void OnPath(const weeder_msgs::msg::Path::SharedPtr message);
  void Tick();
  void HandleWaitPath(double elapsed_s);
  void HandleGoUnmanned();
  void Advance(Phase new_phase);
  void Fail(const std::string& reason, const std::string& hint);
  static const char* PhaseName(Phase phase);

private:
  std::string log_dir_     = "/opt/agri/logs";   // 日志目录 (支持 ~ 与 $VAR 展开)
  std::string log_level_   = "info";             // 日志级别
  int log_file_size_mb_    = 50;                 // 单个日志文件大小上限 (MB)
  int log_file_count_      = 10;                 // 滚动保留的日志文件数量
  bool log_enable_console_ = true;               // 是否同时输出到终端

private:
  double timeout_path_s_       = 30.0;                               // 等待路径超时时间 (s)
  Phase phase_                 = Phase::kWaitPath;                   // 当前编排阶段
  rclcpp::Time phase_start_    = rclcpp::Time(0, 0, RCL_ROS_TIME);   // 当前阶段开始时刻
  std::size_t got_path_points_ = 0;                                  // 最近路径的有效点数

private:
  rclcpp::Subscription<weeder_msgs::msg::Path>::SharedPtr path_sub_;   // 全局路径订阅
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr unmanned_pub_;     // 无人模式切换发布

private:
  rclcpp::TimerBase::SharedPtr timer_ = nullptr;   // 编排状态机定时器
};

}   // namespace sim_vehicle

#endif   // SIM_VEHICLE_SIM_ORCHESTRATOR_NODE_H_
