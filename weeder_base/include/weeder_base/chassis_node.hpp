#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
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
// 下行: /chassis/cmd (速度 + 前轮转角) -> ADCU_toVCU_01 (50Hz / 20ms)
// 上行: VCU Chassis_State_01~11/21/22
//       -> /chassis/state (类型化完整状态)
//
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

private:
  static constexpr size_t kWheelCount = 4;

  std::unique_ptr<CanDriver> can_driver_;
  rclcpp::Subscription<weeder_msgs::msg::ChassisCmd>::SharedPtr cmd_sub_;
  rclcpp::Publisher<weeder_msgs::msg::ChassisState>::SharedPtr state_pub_;
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

private:
  uint8_t last_logged_ctrl_mode_ = 0;
  bool ctrl_mode_logged_once_    = false;
  bool cmd_timed_out_            = false;
  uint16_t last_bms_error_code_  = 0;
  bool bms_error_logged_once_    = false;

private:
  std::string can_interface_;    // CAN 接口名
  std::string cmd_topic_;        // 速度 + 转角合并指令话题
  double cmd_timeout_s_;         // 底盘指令超时 (s)
  rclcpp::Time last_cmd_time_;   // 最近一次收到底盘指令的时刻

private:
  std::string log_dir_;       // spdlog 滚动日志目录
  int log_file_size_mb_;      // 单个日志文件大小上限 (MB)
  int log_file_count_;        // 滚动保留的文件数量
  bool log_enable_console_;   // 是否同时输出到终端
  std::string log_level_;     // 日志级别
};

}   // namespace weeder_base
