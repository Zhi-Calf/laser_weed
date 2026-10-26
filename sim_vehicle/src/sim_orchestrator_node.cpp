#include "sim_vehicle/sim_orchestrator_node.hpp"

#include <algorithm>
#include <chrono>
#include <string>

namespace sim_vehicle {
namespace {

constexpr double kTimerPeriodS = 0.2;
constexpr std::size_t kMinimumPathPoints = 2;

}  // namespace

// 初始化编排节点并启动状态机。
SimOrchestratorNode::SimOrchestratorNode(const rclcpp::NodeOptions& options)
    : Node("sim_orchestrator", options) {
  Initialize();
}

// 按参数、接口和状态机的顺序完成节点初始化。
void SimOrchestratorNode::Initialize() {
  LoadParameters();
  phase_start_ = now();
  SetupInterfaces();
  SetupTimers();
  RCLCPP_INFO(get_logger(),
              "编排节点已启动：等待 YAML 路径生成后"
              "切换 UNMANNED");
}

// 读取路径等待超时参数。
void SimOrchestratorNode::LoadParameters() {
  timeout_path_s_ =
    std::max(declare_parameter<double>("timeout_path_s", 30.0), 0.0);
}

// 创建路径订阅者和无人模式发布者。
void SimOrchestratorNode::SetupInterfaces() {
  const auto latched_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  path_sub_ = create_subscription<weeder_msgs::msg::Path>(
    "/global_path_planner/path", latched_qos,
    [this](const weeder_msgs::msg::Path::SharedPtr message) {
      OnPath(message);
    });
  unmanned_pub_ =
    create_publisher<std_msgs::msg::Bool>("/sim/unmanned", latched_qos);
}

// 创建周期为 0.2 秒的编排定时器。
void SimOrchestratorNode::SetupTimers() {
  const auto period = std::chrono::duration<double>(kTimerPeriodS);
  timer_ =
    rclcpp::create_timer(this, get_clock(), period, [this]() { Tick(); });
}

// 记录最近路径中 x 和 y 同时有效的点数。
void SimOrchestratorNode::OnPath(
  const weeder_msgs::msg::Path::SharedPtr message) {
  got_path_points_ = std::min(message->x.size(), message->y.size());
}

// 根据当前阶段推进一次编排状态机。
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
void SimOrchestratorNode::HandleWaitPath(double elapsed_s) {
  if (got_path_points_ >= kMinimumPathPoints) {
    RCLCPP_INFO(get_logger(), "规划器已发布路径，共 %zu 个点",
                got_path_points_);
    Advance(Phase::kGoUnmanned);
    return;
  }
  if (elapsed_s > timeout_path_s_) {
    const std::string reason =
      "路径一直没发布（收到 " +
      std::to_string(got_path_points_) + " 个点）";
    Fail(reason,
         "检查 global_path_planner.yaml：poly 是否合法、"
         "ridge_width 是否 > 0、field_margin 是否过大。");
  }
}

// 发布一次 UNMANNED 指令并完成时序。
void SimOrchestratorNode::HandleGoUnmanned() {
  std_msgs::msg::Bool message;
  message.data = true;
  unmanned_pub_->publish(message);
  RCLCPP_INFO(get_logger(),
              "已切 UNMANNED，path_tracker 现在应该开始驱动。");
  Advance(Phase::kDone);
}

// 切换阶段并更新阶段开始时刻。
void SimOrchestratorNode::Advance(Phase new_phase) {
  phase_ = new_phase;
  phase_start_ = now();
  if (new_phase == Phase::kDone) {
    RCLCPP_INFO(get_logger(), "=== 时序完成，仿真已在跑 ===");
    RCLCPP_INFO(get_logger(),
                "看 RViz：绿色车体应从偏离路径处收敛贴上去，"
                "沿弓字形行走；橙球是 path_tracker 的跟踪"
                "目标点，"
                "应随车前进。");
    return;
  }
  RCLCPP_INFO(get_logger(), "→ 阶段 %d: %s", static_cast<int>(new_phase),
              PhaseName(new_phase));
}

// 记录编排失败原因和排查建议。
void SimOrchestratorNode::Fail(const std::string& reason,
                               const std::string& hint) {
  phase_ = Phase::kFailed;
  RCLCPP_ERROR(get_logger(), "编排失败：%s", reason.c_str());
  RCLCPP_ERROR(get_logger(), "排查建议：%s", hint.c_str());
}

// 返回阶段的可读中文名称。
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

}  // namespace sim_vehicle
