#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "weeder_base/can_driver.hpp"
#include "weeder_base/protocol/can_protocol.hpp"
#include "weeder_msgs/msg/chassis_cmd.hpp"
#include "weeder_msgs/msg/chassis_state.hpp"

namespace weeder_base {

// ============================================================
// 底盘 ROS2 节点
// ============================================================
//
// 下行: /weeder/chassis/cmd (速度 + 前轮转角) -> ADCU_toVCU_01 (20ms)
// 上行: VCU Chassis_State_01~11/21/22
//       -> /weeder/chassis/state (类型化完整状态)
//       -> /weeder/chassis/state_text (兼容旧监控的一行文本)
//
// 安全策略集中在 sendCommand() 一处裁决 (见 .cpp 注释):
//   指令看门狗 (cmd_timeout_s) -> 停车 + 置急停, 由新的底盘指令解除;
//   VCU 看门狗  (vcu_timeout_s) -> 停车, 不锁存 (VCU 恢复即自动恢复);
//   任一看门狗触发 -> 转向强制回正, 不给任何指令来源翻盘的机会。
class ChassisNode : public rclcpp::Node {
public:
  explicit ChassisNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ChassisNode() override;

private:
  void loadLogParameters();
  void loadParameters();

  void onChassisCmd(const weeder_msgs::msg::ChassisCmd::SharedPtr msg);
  void onCanReceive(const protocol::CanFrame& frame);
  void sendCommand();
  void publishState();

  static std::string fmtState(const protocol::ChassisState01& s1, const protocol::ChassisState02& s2, const protocol::ChassisState03& s3);

private:
  static constexpr size_t kWheelCount = 4;

  std::unique_ptr<CanDriver> can_driver_;
  rclcpp::Subscription<weeder_msgs::msg::ChassisCmd>::SharedPtr cmd_sub_;
  rclcpp::Publisher<weeder_msgs::msg::ChassisState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_text_pub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

private:
  std::mutex cmd_mutex_;                                               // 保护 cmd01_
  std::mutex state_mutex_;                                             // 保护下列全部上行状态与有效性标志
  protocol::AdcuCommand01 cmd01_{};                                    // 下发的控制指令 (逐字段累积)
  protocol::ChassisState01 state01_{};                                 // State_01 底盘主状态
  protocol::ChassisState02 state02_{};                                 // State_02 四轮转速
  protocol::ChassisState03 state03_{};                                 // State_03 四轮转向角
  std::array<protocol::MotorElecState, kWheelCount> drive_motors_{};   // State_04~07
  std::array<protocol::MotorElecState, kWheelCount> steer_motors_{};   // State_08~11
  protocol::ChassisState21 state21_{};                                 // State_21 BMS
  protocol::ChassisState22 state22_{};                                 // State_22 BMS 故障
  bool state01_valid_ = false;
  bool state02_valid_ = false;
  bool state03_valid_ = false;
  std::array<bool, kWheelCount> drive_motor_valid_{};
  std::array<bool, kWheelCount> steer_motor_valid_{};
  bool state21_valid_ = false;
  bool state22_valid_ = false;

  std::atomic<int64_t> last_vcu_ns_{0};                // 最近一帧 State_01 的时刻 (ns, RCL_ROS_TIME)
  std::atomic<int64_t> last_vcu_alive_change_ns_{0};   // alive_counter 最近变化的时刻

private:
  rclcpp::Time last_cmd_time_;             // 最近一次底盘运动指令
  rclcpp::Time last_cmd_stale_log_time_;   // 指令超时告警的上次打印时刻 (限流)
  rclcpp::Time last_vcu_stale_log_time_;   // VCU 超时告警的上次打印时刻 (限流)
  rclcpp::Time last_vcu_stuck_log_time_;   // VCU alive 卡住告警的上次打印时刻 (限流)
  rclcpp::Time last_send_fail_log_time_;   // 发送失败告警的上次打印时刻 (限流)

private:
  uint8_t last_vcu_alive_counter_ = 0;
  bool vcu_alive_initialized_     = false;
  uint8_t last_logged_ctrl_mode_  = 0;
  bool ctrl_mode_logged_once_     = false;
  uint16_t last_bms_error_code_   = 0;
  bool bms_error_logged_once_     = false;

private:
  std::string can_interface_;        // CAN 接口名
  std::string cmd_topic_;            // 速度 + 转角合并指令话题
  double cmd_timeout_s_;             // 底盘指令超时 (s) -> 停车 + 急停
  double vcu_timeout_s_;             // VCU 状态帧超时 (s) -> 停车
  double max_speed_m_s_;             // 最大前进速度 (m/s)
  double max_steer_deg_;             // 最大转向角 (deg)
  double publish_hz_;                // 状态文本发布频率 (Hz)
  double watchdog_log_interval_s_;   // 看门狗/发送失败告警的打印间隔 (s)
  uint8_t steer_mode_;               // 0=单阿克曼 1=双阿克曼 2=平移 3=原地

private:
  std::string log_dir_;       // spdlog 滚动日志目录
  int log_file_size_mb_;      // 单个日志文件大小上限 (MB)
  int log_file_count_;        // 滚动保留的文件数量
  bool log_enable_console_;   // 是否同时输出到终端
  std::string log_level_;     // 日志级别
};

}   // namespace weeder_base
