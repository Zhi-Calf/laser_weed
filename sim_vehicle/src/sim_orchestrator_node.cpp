#include "sim_vehicle/sim_orchestrator_node.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <exception>
#include <string>

#include "sim_vehicle/logging.hpp"

namespace sim_vehicle {
namespace {

constexpr double kTimerPeriodS           = 0.2;
constexpr std::size_t kMinimumPathPoints = 2;

}   // namespace

// 创建编排节点并启动状态机。
// 输入:
//   options : ROS 节点创建选项
SimOrchestratorNode::SimOrchestratorNode(const rclcpp::NodeOptions& options) : Node("sim_orchestrator", options) {
  Initialize();
}

// 按参数、接口和状态机的顺序完成节点初始化。
// 副作用:
//   创建订阅、发布和定时器，并将状态机切换到等待路径阶段。
void SimOrchestratorNode::Initialize() {
  LoadLogParameters();
  SetupLogging();
  LoadParameters();
  phase_start_ = now();
  SetupInterfaces();
  SetupTimers();
  SPDLOG_INFO(
    "编排节点已启动：等待 YAML 路径生成后"
    "切换 UNMANNED");
}

// 声明并读取 spdlog 滚动日志参数。
// 输出:
//   日志目录、级别、文件大小和文件数量。
void SimOrchestratorNode::LoadLogParameters() {
  declare_parameter("log_dir", "/opt/agri/logs");
  declare_parameter("log_file_size_mb", 50);
  declare_parameter("log_file_count", 10);
  declare_parameter("log_enable_console", true);
  declare_parameter("log_level", "info");

  log_dir_            = get_parameter("log_dir").as_string();
  log_file_size_mb_   = static_cast<int>(std::max<int64_t>(1, get_parameter("log_file_size_mb").as_int()));
  log_file_count_     = static_cast<int>(std::max<int64_t>(1, get_parameter("log_file_count").as_int()));
  log_enable_console_ = get_parameter("log_enable_console").as_bool();
  log_level_          = get_parameter("log_level").as_string();
}

// 配置仿真编排节点的滚动文件日志和控制台输出。
// 失败:
//   文件日志初始化异常时保留 spdlog 默认日志器继续运行。
void SimOrchestratorNode::SetupLogging() {
  LogConfig log_config;
  log_config.logger_name         = "sim_orchestrator";
  log_config.log_file_name       = "sim_orchestrator.log";
  log_config.log_dir             = log_dir_;
  log_config.max_file_size_bytes = static_cast<std::size_t>(log_file_size_mb_) * 1024U * 1024U;
  log_config.max_file_count      = static_cast<std::size_t>(log_file_count_);
  log_config.enable_console      = log_enable_console_;
  log_config.log_level           = log_level_;
  try {
    ConfigureLogging(log_config);
    SPDLOG_INFO("日志配置成功");
  } catch (const std::exception& error) {
    SPDLOG_WARN("文件日志配置失败，使用默认日志器: {}", error.what());
  }
}

// 读取路径等待超时参数。
// 输出:
//   timeout_path_s_ : 等待路径的最长时间 (s)，负值按 0 处理。
void SimOrchestratorNode::LoadParameters() {
  timeout_path_s_ = std::max(declare_parameter<double>("timeout_path_s", 30.0), 0.0);
}

// 创建路径订阅者和无人模式发布者。
// 副作用:
//   路径订阅和无人模式发布均使用 transient_local QoS。
void SimOrchestratorNode::SetupInterfaces() {
  const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  path_sub_              = create_subscription<weeder_msgs::msg::Path>("/global_path_planner/path", latched_qos, [this](const weeder_msgs::msg::Path::SharedPtr message) {
    OnPath(message);
  });
  unmanned_pub_          = create_publisher<std_msgs::msg::Bool>("/sim/unmanned", latched_qos);
}

// 创建周期为 0.2 秒的编排定时器。
// 副作用:
//   每个周期调用 Tick() 推进状态机。
void SimOrchestratorNode::SetupTimers() {
  const auto period = std::chrono::duration<double>(kTimerPeriodS);
  timer_            = rclcpp::create_timer(this, get_clock(), period, [this]() {
    Tick();
  });
}

// 记录最近路径中 x 和 y 同时有效的点数。
// 输入:
//   message : 规划器发布的最新路径
// 输出:
//   got_path_points_ : min(x.size(), y.size())
void SimOrchestratorNode::OnPath(const weeder_msgs::msg::Path::SharedPtr message) {
  got_path_points_ = std::min(message->x.size(), message->y.size());
}

// 根据当前阶段推进一次编排状态机。
// 副作用:
//   可能进入切换 UNMANNED 阶段、完成状态或失败状态。
void SimOrchestratorNode::Tick() {
  if (phase_ == Phase::kDone || phase_ == Phase::kFailed) {
    return;
  }
  const double elapsed_s = (now() - phase_start_).seconds();
  if (phase_ == Phase::kWaitPath) {
    HandleWaitPath(elapsed_s);
  } else if (phase_ == Phase::kGoUnmanned) {
    HandleGoUnmanned();
  }
}

// 路径可用时进入切换阶段，否则在超时后进入失败状态。
// 输入:
//   elapsed_s : 当前等待阶段已经持续的时长 (s)
void SimOrchestratorNode::HandleWaitPath(double elapsed_s) {
  if (got_path_points_ >= kMinimumPathPoints) {
    SPDLOG_INFO("规划器已发布路径，共 {} 个点", got_path_points_);
    Advance(Phase::kGoUnmanned);
    return;
  }
  if (elapsed_s > timeout_path_s_) {
    const std::string reason = "路径一直没发布（收到 " + std::to_string(got_path_points_) + " 个点）";
    Fail(
      reason,
      "检查 global_path_planner.yaml：poly 是否合法、"
      "ridge_width 是否 > 0、field_margin 是否过大。");
  }
}

// 发布一次 UNMANNED 指令并完成时序。
// 副作用:
//   发布 true 到 /sim/unmanned，并进入 kDone 阶段。
void SimOrchestratorNode::HandleGoUnmanned() {
  std_msgs::msg::Bool message;
  message.data = true;
  unmanned_pub_->publish(message);
  SPDLOG_INFO("已切 UNMANNED，path_tracker 现在应该开始驱动。");
  Advance(Phase::kDone);
}

// 切换阶段并更新阶段开始时刻。
// 输入:
//   new_phase : 目标阶段
// 副作用:
//   更新 phase_ 与 phase_start_，并输出阶段日志。
void SimOrchestratorNode::Advance(Phase new_phase) {
  phase_       = new_phase;
  phase_start_ = now();
  if (new_phase == Phase::kDone) {
    SPDLOG_INFO("=== 时序完成，仿真已在跑 ===");
    SPDLOG_INFO(
      "看 RViz：绿色车体应从偏离路径处收敛贴上去，"
      "沿弓字形行走；橙球是 path_tracker 的跟踪"
      "目标点，"
      "应随车前进。");
    return;
  }
  SPDLOG_INFO("→ 阶段 {}: {}", static_cast<int>(new_phase), PhaseName(new_phase));
}

// 记录编排失败原因和排查建议。
// 输入:
//   reason : 失败原因
//   hint   : 现场排查建议
// 副作用:
//   状态机进入 kFailed，并输出错误日志。
void SimOrchestratorNode::Fail(const std::string& reason, const std::string& hint) {
  phase_ = Phase::kFailed;
  SPDLOG_ERROR("编排失败：{}", reason);
  SPDLOG_ERROR("排查建议：{}", hint);
}

// 返回阶段的可读中文名称。
// 输入:
//   phase : 编排阶段
// 返回:
//   中文阶段名称
const char* SimOrchestratorNode::PhaseName(Phase phase) {
  switch (phase) {
    case Phase::kWaitPath:
      return "等待规划器自动发布路径";
    case Phase::kGoUnmanned:
      return "切换 UNMANNED 让控制器接管";
    case Phase::kDone:
      return "完成";
    case Phase::kFailed:
      return "失败";
  }
  return "未知阶段";
}

}   // namespace sim_vehicle
