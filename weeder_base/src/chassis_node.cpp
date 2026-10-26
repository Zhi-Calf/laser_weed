// 底盘 ROS2 节点实现。
//
// 下行: /chassis/cmd (速度 + 转角) -> ADCU_toVCU_01 (50Hz / 20ms)
// 上行: VCU Chassis_State_01~11/21/22 -> /chassis/state
//
// 时钟约定 (踩过的坑, 别改回去):
//   - 全部时间戳用 this->now() (RCL_ROS_TIME)。跨时钟源相减会抛
//     std::runtime_error, 异常从定时器回调冒出会直接 terminate 进程。
//   - VCU 状态时间戳在构造函数里对齐到启动时刻, 避免首拍被误判为超时。
#include "weeder_base/chassis_node.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>
#include <vector>

#include "weeder_base/logging.hpp"

using namespace std::chrono_literals;

namespace {

// 默认值 (与 config/weeder_base.yaml 保持一致, 改一边记得改另一边)
constexpr double kDefaultCmdTimeoutS   = 0.3;
constexpr const char* kDefaultCmdTopic = "/chassis/cmd";

// 未收到有效指令时的安全状态: N 档、遥控模式、抱闸未释放、速度/转角为 0。
weeder_base::protocol::AdcuCommand01 MakeSafeStopCommand() {
  weeder_base::protocol::AdcuCommand01 cmd{};
  cmd.gear          = 0;
  cmd.steer_mode    = 0;
  cmd.release_brake = 0;
  cmd.estop         = 0;
  cmd.reset         = 0;
  cmd.ctrl_mode     = 0;
  cmd.speed_raw     = 0;
  cmd.steer_angle   = 0;
  return cmd;
}

}   // namespace

namespace weeder_base {

ChassisNode::ChassisNode(const rclcpp::NodeOptions& options) : Node("weeder_chassis", options) {

  loadLogParameters();

  // 配置 spdlog 滚动文件输出
  LogConfig log_config;
  log_config.logger_name         = "weeder_base";
  log_config.log_file_name       = "weeder_base.log";
  log_config.log_dir             = log_dir_;
  log_config.max_file_size_bytes = static_cast<std::size_t>(log_file_size_mb_) * 1024U * 1024U;
  log_config.max_file_count      = static_cast<std::size_t>(log_file_count_);
  log_config.enable_console      = log_enable_console_;
  log_config.log_level           = log_level_;
  try {
    ConfigureLogging(log_config);
    SPDLOG_INFO("日志配置成功");
  } catch (const std::exception& e) {
    SPDLOG_WARN("文件日志配置失败, 使用默认日志器: {}", e.what());
  }

  loadParameters();
  cmd01_ = MakeSafeStopCommand();

  // ---- CAN 驱动 ----
  // CAN 是底盘控制的命根子, 打不开即快速失败 (进程退出, 由 launch respawn 拉起):
  // 静默"正常运行"却一帧都发不出去, 比明确报错危险得多。
  can_driver_ = std::make_unique<CanDriver>(can_interface_);
  // 只接收协议文档中 VCU -> ADCU 的全部周期帧。漏配会导致对应解码分支
  // 永远收不到数据, 因此这里与 onCanReceive() 的 switch 保持一一对应。
  can_driver_->setFilters({
    protocol::can_id::CHASSIS_STATE_01,
    protocol::can_id::CHASSIS_STATE_02,
    protocol::can_id::CHASSIS_STATE_03,
    protocol::can_id::CHASSIS_STATE_04,
    protocol::can_id::CHASSIS_STATE_05,
    protocol::can_id::CHASSIS_STATE_06,
    protocol::can_id::CHASSIS_STATE_07,
    protocol::can_id::CHASSIS_STATE_08,
    protocol::can_id::CHASSIS_STATE_09,
    protocol::can_id::CHASSIS_STATE_10,
    protocol::can_id::CHASSIS_STATE_11,
    protocol::can_id::CHASSIS_STATE_21,
    protocol::can_id::CHASSIS_STATE_22,
  });
  if (!can_driver_->open()) {
    throw std::runtime_error("无法打开 CAN 接口: " + can_interface_);
  }
  can_driver_->setReceiveCallback([this](const protocol::CanFrame& frame) {
    onCanReceive(frame);
  });
  can_driver_->startReceiveThread();

  last_cmd_time_ = this->now();

  // ---- ROS 话题 ----
  // 速度与转角必须来自同一条消息、同一控制周期。
  cmd_sub_   = create_subscription<weeder_msgs::msg::ChassisCmd>(cmd_topic_, 10, std::bind(&ChassisNode::onChassisCmd, this, std::placeholders::_1));
  state_pub_ = create_publisher<weeder_msgs::msg::ChassisState>("/chassis/state", 10);

  // ---- 定时器 ----
  // CAN 下发按协议固定为 50 Hz (20ms); ROS 状态上报保持 10 Hz。
  send_timer_ = create_wall_timer(std::chrono::milliseconds(protocol::timing::kAdcuCmdPeriodMs), std::bind(&ChassisNode::sendCommand, this));
  pub_timer_  = create_wall_timer(100ms, std::bind(&ChassisNode::publishState, this));

  SPDLOG_INFO("ChassisNode 启动: 接口={} 下发=50Hz 状态=10Hz", can_interface_);
  SPDLOG_INFO("底盘指令: {}", cmd_topic_);
}

ChassisNode::~ChassisNode() {
  // 析构体在成员销毁之前执行: 先停定时器, 再停接收线程并注销回调,
  // 避免 CAN 线程在 this 已销毁后回调 onCanReceive()。
  send_timer_.reset();
  pub_timer_.reset();
  cmd_sub_.reset();
  if (can_driver_) {
    can_driver_->stopReceiveThread();   // join 返回后才可能销毁 this
    can_driver_->setReceiveCallback(nullptr);
    can_driver_->close();
  }
}

void ChassisNode::loadLogParameters() {
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

void ChassisNode::loadParameters() {
  declare_parameter("can_interface", "vcan0");
  declare_parameter("cmd_timeout_s", kDefaultCmdTimeoutS);
  declare_parameter("cmd_topic", kDefaultCmdTopic);

  can_interface_ = get_parameter("can_interface").as_string();
  cmd_timeout_s_ = get_parameter("cmd_timeout_s").as_double();
  cmd_topic_     = get_parameter("cmd_topic").as_string();

  // ---- 参数边界校验 ----
  // 参数非法时回退到默认值并告警, 不用静默吸收错误值。
  if (cmd_timeout_s_ <= 0.0) {
    SPDLOG_WARN("cmd_timeout_s={:.3f} 非法, 回退为 {:.1f} s", cmd_timeout_s_, kDefaultCmdTimeoutS);
    cmd_timeout_s_ = kDefaultCmdTimeoutS;
  }
  if (cmd_topic_.empty()) {
    SPDLOG_WARN("cmd_topic 为空, 回退为 {}", kDefaultCmdTopic);
    cmd_topic_ = kDefaultCmdTopic;
  }
}

// ---- 下行: 速度 + 前轮转角合并指令 ----
void ChassisNode::onChassisCmd(const weeder_msgs::msg::ChassisCmd::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  last_cmd_time_ = this->now();
  cmd_timed_out_ = false;

  cmd01_.gear          = msg->gear & 0x03;
  cmd01_.steer_mode    = msg->steer_mode & 0x03;
  cmd01_.release_brake = msg->release_brake ? 1 : 0;
  cmd01_.estop         = msg->estop ? 1 : 0;
  cmd01_.ctrl_mode     = msg->ctrl_mode ? 1 : 0;

  // m/s -> mm/s (协议速度 uint16, 分辨率 1 mm/s)
  const double max_speed_mps = static_cast<double>(protocol::limits::kMaxSpeedMmps) / 1000.0;
  const double v             = std::clamp(msg->speed_m_s, 0.0, max_speed_mps);
  cmd01_.speed_raw           = static_cast<uint16_t>(std::llround(v * 1000.0));

  // rad -> 0.1°。左正右负与协议同向, 不取反。
  const double max_steer_deg = static_cast<double>(protocol::limits::kMaxSteerAngleRaw) * 0.1;
  const double steer_deg     = msg->steer_angle_rad * 180.0 / M_PI;
  const double clamped_deg   = std::clamp(steer_deg, -max_steer_deg, max_steer_deg);
  cmd01_.steer_angle         = static_cast<int16_t>(std::llround(clamped_deg * 10.0));
}

// ---- 50Hz (20ms) 周期下发 ADCU_toVCU_01 ----
void ChassisNode::sendCommand() {
  if (!can_driver_ || !can_driver_->isOpen()) {
    return;   // 驱动侧已报过"未打开"
  }

  protocol::AdcuCommand01 out_cmd{};
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    const double cmd_age = (this->now() - last_cmd_time_).seconds();
    if (cmd_age > cmd_timeout_s_) {
      const uint8_t alive_counter = cmd01_.alive_counter;
      cmd01_                      = MakeSafeStopCommand();
      cmd01_.alive_counter        = alive_counter;
      if (!cmd_timed_out_) {
        cmd_timed_out_ = true;
        SPDLOG_WARN("底盘指令超时 {:.2f}s > {:.2f}s, 已清零并停车", cmd_age, cmd_timeout_s_);
      }
    } else {
      cmd_timed_out_ = false;
    }
    cmd01_.alive_counter = static_cast<uint8_t>((cmd01_.alive_counter + 1) & 0xFF);
    out_cmd              = cmd01_;   // 复制出临界区, CAN 发送不持锁
  }

  const protocol::CanFrame f = protocol::CanProtocol::encodeAdcuCmd01(out_cmd);
  if (!can_driver_->send(f)) {
    SPDLOG_ERROR("CAN 发送失败 (ID=0x{:08X}), 底盘未收到指令", f.id);
  }
}

// ---- 上行: CAN 接收线程回调 (VCU -> ADCU) ----
void ChassisNode::onCanReceive(const protocol::CanFrame& frame) {
  switch (frame.id) {
    case protocol::can_id::CHASSIS_STATE_01: {
      protocol::ChassisState01 s;
      if (!protocol::CanProtocol::decodeState01(frame, s)) {
        return;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      state01_       = s;
      state01_valid_ = true;

      if (!ctrl_mode_logged_once_ || s.ctrl_mode != last_logged_ctrl_mode_) {
        SPDLOG_INFO("VCU 驾驶模式: {} -> {}", last_logged_ctrl_mode_ ? "无人驾驶" : "遥控", s.ctrl_mode ? "无人驾驶" : "遥控");
        last_logged_ctrl_mode_ = s.ctrl_mode;
        ctrl_mode_logged_once_ = true;
      }
      break;
    }

    case protocol::can_id::CHASSIS_STATE_02: {
      protocol::ChassisState02 s;
      if (!protocol::CanProtocol::decodeState02(frame, s)) {
        return;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      state02_       = s;
      state02_valid_ = true;
      break;
    }

    case protocol::can_id::CHASSIS_STATE_03: {
      protocol::ChassisState03 s;
      if (!protocol::CanProtocol::decodeState03(frame, s)) {
        return;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      state03_       = s;
      state03_valid_ = true;
      break;
    }

    case protocol::can_id::CHASSIS_STATE_04:
    case protocol::can_id::CHASSIS_STATE_05:
    case protocol::can_id::CHASSIS_STATE_06:
    case protocol::can_id::CHASSIS_STATE_07:
    case protocol::can_id::CHASSIS_STATE_08:
    case protocol::can_id::CHASSIS_STATE_09:
    case protocol::can_id::CHASSIS_STATE_10:
    case protocol::can_id::CHASSIS_STATE_11: {
      protocol::MotorElecState s;
      if (!protocol::CanProtocol::decodeMotorElec(frame, s)) {
        return;
      }
      const bool is_drive = frame.id <= protocol::can_id::CHASSIS_STATE_07;
      const uint32_t base = is_drive ? protocol::can_id::CHASSIS_STATE_04 : protocol::can_id::CHASSIS_STATE_08;
      // ID 低 16 位一致, 每下一路增加 0x10000。
      const size_t index = static_cast<size_t>((frame.id - base) >> 16);
      if (index >= kWheelCount) {
        return;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (is_drive) {
        drive_motors_[index]      = s;
        drive_motor_valid_[index] = true;
      } else {
        steer_motors_[index]      = s;
        steer_motor_valid_[index] = true;
      }
      break;
    }

    case protocol::can_id::CHASSIS_STATE_21: {
      protocol::ChassisState21 s;
      if (!protocol::CanProtocol::decodeState21(frame, s)) {
        return;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      state21_       = s;
      state21_valid_ = true;
      break;
    }

    case protocol::can_id::CHASSIS_STATE_22: {
      protocol::ChassisState22 s;
      if (!protocol::CanProtocol::decodeState22(frame, s)) {
        return;
      }
      std::lock_guard<std::mutex> lock(state_mutex_);
      state22_       = s;
      state22_valid_ = true;
      if (!bms_error_logged_once_ || s.error_code != last_bms_error_code_) {
        if (s.error_code == 0) {
          SPDLOG_INFO("BMS 故障码: {} -> 0 (恢复正常)", last_bms_error_code_);
        } else {
          SPDLOG_WARN("BMS 故障码: {} -> {}", last_bms_error_code_, s.error_code);
        }
        last_bms_error_code_   = s.error_code;
        bms_error_logged_once_ = true;
      }
      break;
    }

    default:
      break;
  }
}

// ---- 上行: 完整状态发布 (10Hz) ----
void ChassisNode::publishState() {
  auto msg             = std::make_unique<weeder_msgs::msg::ChassisState>();
  msg->header.stamp    = this->now();
  msg->header.frame_id = "base_link";
  {
    // 与 CAN 接收线程共享全部状态, 读取必须加锁, 否则是数据竞争 (UB)。
    std::lock_guard<std::mutex> lock(state_mutex_);
    const protocol::ChassisState01& s1 = state01_;
    msg->state01_valid                 = state01_valid_;
    msg->gear                          = s1.gear;
    msg->steer_mode                    = s1.steer_mode;
    msg->estop                         = s1.estop != 0;
    msg->reset                         = s1.reset != 0;
    msg->release_brake                 = s1.release_brake != 0;
    msg->ctrl_mode                     = s1.ctrl_mode != 0;
    msg->speed_raw                     = s1.speed_raw;
    msg->speed_m_s                     = s1.speed_m_s;
    msg->wheel_track_cm                = s1.wheel_track_cm;
    msg->sound                         = s1.sound != 0;
    msg->light_green                   = s1.light_green != 0;
    msg->light_yellow                  = s1.light_yellow != 0;
    msg->light_red                     = s1.light_red != 0;
    msg->headlight                     = s1.headlight != 0;
    msg->turn_left                     = s1.turn_left != 0;
    msg->turn_right                    = s1.turn_right != 0;
    msg->steer_done_fl                 = s1.steer_done_fl != 0;
    msg->steer_done_fr                 = s1.steer_done_fr != 0;
    msg->steer_done_rl                 = s1.steer_done_rl != 0;
    msg->steer_done_rr                 = s1.steer_done_rr != 0;
    msg->bumper                        = s1.bumper != 0;
    msg->bypass                        = s1.bypass != 0;
    msg->battery_charging              = s1.battery_charging != 0;
    msg->battery_full                  = s1.battery_full != 0;
    msg->alive_counter                 = s1.alive_counter;

    msg->state02_valid   = state02_valid_;
    msg->motor_speed_rpm = {state02_.motor_speed_fl, state02_.motor_speed_fr, state02_.motor_speed_rl, state02_.motor_speed_rr};
    msg->state03_valid   = state03_valid_;
    msg->steer_angle_deg = {
      static_cast<float>(state03_.steer_angle_fl) * 0.1f,
      static_cast<float>(state03_.steer_angle_fr) * 0.1f,
      static_cast<float>(state03_.steer_angle_rl) * 0.1f,
      static_cast<float>(state03_.steer_angle_rr) * 0.1f};

    for (size_t i = 0; i < kWheelCount; ++i) {
      msg->drive_motor_valid[i] = drive_motor_valid_[i];
      msg->drive_bus_voltage[i] = drive_motors_[i].bus_voltage;
      msg->drive_out_current[i] = drive_motors_[i].out_current;
      msg->drive_motor_temp[i]  = drive_motors_[i].motor_temp;
      msg->drive_error_code[i]  = drive_motors_[i].error_code;

      msg->steer_motor_valid[i] = steer_motor_valid_[i];
      msg->steer_bus_voltage[i] = steer_motors_[i].bus_voltage;
      msg->steer_out_current[i] = steer_motors_[i].out_current;
      msg->steer_motor_temp[i]  = steer_motors_[i].motor_temp;
      msg->steer_error_code[i]  = steer_motors_[i].error_code;
    }

    msg->state21_valid            = state21_valid_;
    msg->bms_soc                  = state21_.soc_raw;
    msg->bms_voltage              = state21_.voltage;
    msg->bms_current              = state21_.current;
    msg->bms_full_charge_capacity = state21_.full_charge_capacity;

    msg->state22_valid        = state22_valid_;
    msg->bms_cycle_count      = state22_.cycle_count;
    msg->bms_error_code       = state22_.error_code;
    msg->bms_max_cell_voltage = state22_.max_cell_voltage;
    msg->system_voltage       = state22_.system_voltage;
  }
  state_pub_->publish(std::move(msg));
}

}   // namespace weeder_base
