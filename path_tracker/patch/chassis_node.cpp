// 历史补丁文件，请勿直接使用：
// 当前实现已统一为 /chassis/cmd (weeder_msgs/ChassisCmd)。
//
// 依赖：weeder_base/can_driver.hpp（内部引入 weeder_base::protocol 协议符号）
//
// 已修复（相对重构后的上一版）：
//   1. rclcpp::Time 时钟源统一为 RCL_ROS_TIME，消除 "can't subtract times with
//      different time sources" 异常导致的 terminate
//   2. 时间基准在构造函数中对齐到启动时刻，避免 epoch 0 被判定为超时
//   3. onCmdVel 收到新指令时清除 estop，避免超时急停永久锁死
//   4. 实现 VCU 心跳超时看门狗（原先 vcu_timeout_s_ 只声明不使用）
//   5. publishState()/sendCommand() 补锁，消除与 CAN 接收线程的数据竞争
//   6. 新增析构函数：先停接收线程再销毁成员，避免回调悬空 this
//   7. 调用 open() 并检查返回值；send() 失败有日志（原先返回值全部丢弃）
//   8. max_speed_m_s / max_steer_deg / publish_hz 三个 launch 参数真正生效
//   9. 去掉多余 const_cast；补齐 <algorithm> <cstdio> 等头文件
//  10. 转向指令改由 /weeder/steer_cmd (std_msgs/Float64, 弧度, 左正) 提供，与
//      path_tracker 直接对接。原 angular.z -> steer_angle 的量纲不严谨映射降级为
//      **可选回退**（steer_fallback_to_cmd_vel，默认 true），保留手动遥控能力：
//      键盘/手柄只发 /cmd_vel，自动作业时 path_tracker 两个话题都发，steer_cmd 优先。
//  11. steer_mode 提为参数（默认 0 单阿克曼，与改动前行为一致）。原先硬编码为
//      结构体默认值 0，无法与 path_tracker 的 steer_mode 对齐；若控制器按双阿克曼
//      (等效轴距 L/2) 计算而底盘按单阿克曼执行，转角会差一倍。
//
// 仍未实现（不属本次修复范围）：
//   - State_02~23 解码与发布（含 README 承诺的 /weeder/bms/state_raw）
//   - 倒车（gear 固定 D 档，linear.x < 0 被截断为 0）

#include "weeder_base/can_driver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace weeder_base {

class ChassisNode : public rclcpp::Node {
public:
  explicit ChassisNode(const std::string& name) : Node(name) {
    // ---- 参数：全部声明并使用，避免 launch 里改了不生效 ----
    cmd_timeout_s_ = declare_parameter("cmd_timeout_s", 0.5);
    vcu_timeout_s_ = declare_parameter("vcu_timeout_s", 1.0);
    max_speed_m_s_ = declare_parameter("max_speed_m_s", 1.4);
    max_steer_deg_ = declare_parameter("max_steer_deg", 90.0);
    publish_hz_    = declare_parameter("publish_hz", 20.0);
    const std::string can_interface =
        declare_parameter("can_interface", std::string("can0"));

    // ---- 转向指令来源参数 ----
    steer_cmd_topic_ = declare_parameter(
        "steer_cmd_topic", std::string("/weeder/steer_cmd"));
    // 与 cmd_timeout_s_ 同量级：path_tracker 默认 10Hz 发布，0.5s 容忍丢 5 帧
    steer_timeout_s_ = declare_parameter("steer_timeout_s", 0.5);
    // steer_cmd 超时后是否退回 angular.z 映射（保留手动遥控）。false = 严格模式：
    // 只认 /weeder/steer_cmd，超时即回正。实车自动作业建议 false，台架/遥控调试用 true。
    steer_fallback_to_cmd_vel_ =
        declare_parameter("steer_fallback_to_cmd_vel", true);
    // 0=单阿克曼 1=双阿克曼 2=平移 3=原地转向。必须与 path_tracker 的 steer_mode 一致。
    // declare_parameter 返回 int64_t，显式接住再 static_cast，避免窄化转换告警。
    const int64_t steer_mode_param = declare_parameter("steer_mode", 0);
    if (steer_mode_param < 0 || steer_mode_param > 3) {
      RCLCPP_WARN(get_logger(), "steer_mode=%ld 非法（取值 0~3），回退为 0",
                  static_cast<long>(steer_mode_param));
      steer_mode_ = 0;
    } else {
      steer_mode_ = static_cast<uint8_t>(steer_mode_param);
    }

    if (publish_hz_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "publish_hz=%.3f 非法，回退为 20.0 Hz", publish_hz_);
      publish_hz_ = 20.0;
    }

    // ---- CAN 驱动：必须 open()，失败显式抛错，不能静默"正常运行" ----
    can_driver_ = std::make_unique<CanDriver>(can_interface);
    if (!can_driver_->open()) {
      throw std::runtime_error("无法打开 CAN 接口: " + can_interface);
    }
    can_driver_->setReceiveCallback(
        [this](const protocol::CanFrame& frame) { onCanReceive(frame); });
    can_driver_->startReceiveThread();

    // ---- 时间基准 ----
    // 必须与 this->now() 同为 RCL_ROS_TIME：跨时钟源相减会抛 std::runtime_error，
    // 异常从定时器回调冒出会直接 terminate 进程。
    // 同时对齐到启动时刻：否则 last_cmd_time_ 停留在 epoch 0，
    // 启动后第一个 20ms 周期的 age 就是十亿秒级，必然立即进入超时分支。
    const rclcpp::Time now = this->now();
    last_cmd_time_ = now;
    last_steer_time_ = now;  // 同理：不对齐会让启动瞬间 steer_age 巨大
    last_vcu_ns_.store(now.nanoseconds(), std::memory_order_relaxed);

    // ---- 订阅 /cmd_vel（线速度；angular.z 仅作转向回退来源）----
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          onCmdVel(msg);
        });

    // ---- 订阅 /weeder/steer_cmd（权威转向角，来自 path_tracker）----
    // std_msgs/Float64，单位弧度，左转（逆时针）为正 —— 与协议 ADCU_toVCU_01 的
    // 转向角符号同向，因此**不取反**，只做 弧度 -> 0.1 度 换算。
    steer_sub_ = create_subscription<std_msgs::msg::Float64>(
        steer_cmd_topic_, 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          onSteerCmd(msg);
        });

    // ---- 发布状态文本 ----
    state_pub_ = create_publisher<std_msgs::msg::String>(
        "/weeder/chassis/state_text", 10);

    // ---- 20ms 发送定时器 ----
    send_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        [this]() { sendCommand(); });

    // ---- 状态发布定时器（周期由 publish_hz 决定）----
    const auto pub_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_hz_));
    pub_timer_ = create_wall_timer(pub_period, [this]() { publishState(); });

    RCLCPP_INFO(get_logger(),
        "ChassisNode started: interface=%s cmd_timeout=%.2fs vcu_timeout=%.2fs "
        "max_speed=%.2f m/s max_steer=%.1f deg publish=%.1f Hz",
        can_interface.c_str(), cmd_timeout_s_, vcu_timeout_s_,
        max_speed_m_s_, max_steer_deg_, publish_hz_);
    RCLCPP_INFO(get_logger(),
        "转向来源: %s (超时 %.2fs, angular.z 回退=%s) steer_mode=%u",
        steer_cmd_topic_.c_str(), steer_timeout_s_,
        steer_fallback_to_cmd_vel_ ? "开" : "关",
        static_cast<unsigned>(steer_mode_));
  }

  ~ChassisNode() override {
    // 析构体在成员销毁之前执行：先取消定时器，再停接收线程并注销回调，
    // 避免 CAN 线程在 this 已销毁后回调 onCanReceive()。
    send_timer_.reset();
    pub_timer_.reset();
    cmd_sub_.reset();
    steer_sub_.reset();
    if (can_driver_) {
      can_driver_->stopReceiveThread();
      can_driver_->setReceiveCallback(nullptr);
      can_driver_->close();
    }
  }

private:
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    last_cmd_time_ = this->now();

    cmd01_.gear      = 1;  // D 档
    cmd01_.ctrl_mode = 1;  // 无人驾驶
    // 收到新指令 -> 解除超时看门狗锁存的急停。
    // 不清零的话，一旦超时置位就没有任何代码路径能恢复，车辆永久停在急停态。
    // 只由 /cmd_vel 清除（不由 onSteerCmd 清）：速度指令才是权威的"允许走"信号，
    // 单有转向指令不应能解除急停。
    cmd01_.estop     = 0;

    // m/s -> mm/s（分辨率按 1 mm/s 实现，见 README 待确认项）
    const double v = std::clamp(msg->linear.x, 0.0, max_speed_m_s_);
    cmd01_.speed_raw = static_cast<uint16_t>(std::llround(v * 1000.0));

    // angular.z -> 度：仅作**回退来源**缓存，不直接写 cmd01_。
    // 量纲上不严谨（把 rad/s 当角度用），但保留它是为了不破坏手动遥控
    // —— 键盘/手柄只发 /cmd_vel。自动作业时 path_tracker 同时发
    // /weeder/steer_cmd，在 sendCommand() 里优先级更高，此值不生效。
    steer_from_cmd_vel_deg_ = std::clamp(msg->angular.z * 180.0 / M_PI,
                                         -max_steer_deg_, max_steer_deg_);

    // 注意：linear.x < 0 被 clamp 到 0，即当前不支持倒车（gear 固定 D 档）。
    // 若需要 R 档，应在协议确认后按 linear.x 符号切换 gear。
  }

  void onSteerCmd(const std_msgs::msg::Float64::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    last_steer_time_ = this->now();
    // 弧度 -> 度。左正右负与协议同向，不取反。
    const double deg = msg->data * 180.0 / M_PI;
    steer_cmd_deg_ = std::clamp(deg, -max_steer_deg_, max_steer_deg_);
    // 只缓存，不在这里写 cmd01_.steer_angle：两个来源的优先级必须在
    // sendCommand() 里按新鲜度统一裁决，否则后到的回调会覆盖看门狗的回正。
  }

  void sendCommand() {
    protocol::AdcuCommand01 out_cmd{};
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      const rclcpp::Time now = this->now();
      const double cmd_age = (now - last_cmd_time_).seconds();
      const double steer_age = (now - last_steer_time_).seconds();
      const double vcu_age =
          static_cast<double>(now.nanoseconds() -
              last_vcu_ns_.load(std::memory_order_relaxed)) * 1e-9;

      // motion_inhibit：任一看门狗触发即禁止运动，转向强制回正。
      // 集中在一处裁决，避免看门狗置 0 后又被下面的来源选择覆盖。
      bool motion_inhibit = false;

      // 看门狗 1：指令超时 -> 停车 + 急停（可由新的 /cmd_vel 解除）
      if (cmd_age > cmd_timeout_s_) {
        cmd01_.speed_raw = 0;
        cmd01_.estop     = 1;
        motion_inhibit   = true;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "cmd_vel 超时 %.2fs > %.2fs，已停车并置急停", cmd_age, cmd_timeout_s_);
      }

      // 看门狗 2：VCU 心跳超时 -> 停车。
      // 不锁存 estop：VCU 恢复上报后自动回到正常控制，无需人工干预。
      if (vcu_age > vcu_timeout_s_) {
        cmd01_.speed_raw = 0;
        motion_inhibit   = true;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "VCU 状态帧超时 %.2fs > %.2fs，已停车", vcu_age, vcu_timeout_s_);
      }

      // ---- 转向角裁决 ----
      if (motion_inhibit) {
        // 看门狗触发：回正。这是安全侧，不给任何来源翻盘的机会。
        cmd01_.steer_angle = 0;
      } else if (steer_age <= steer_timeout_s_) {
        // 来源 1（权威）：新鲜的 /weeder/steer_cmd，path_tracker 算出的前轮转角
        cmd01_.steer_angle =
            static_cast<int16_t>(std::llround(steer_cmd_deg_ * 10.0));
      } else if (steer_fallback_to_cmd_vel_) {
        // 来源 2（回退）：/cmd_vel 的 angular.z 映射，供手动遥控使用
        cmd01_.steer_angle =
            static_cast<int16_t>(std::llround(steer_from_cmd_vel_deg_ * 10.0));
      } else {
        // 严格模式：没有新鲜 steer_cmd 就回正
        cmd01_.steer_angle = 0;
      }

      // 转向模式：必须与 path_tracker 的 steer_mode 一致，否则等效轴距对不上
      cmd01_.steer_mode = steer_mode_;

      cmd01_.alive_counter =
          static_cast<uint8_t>((cmd01_.alive_counter + 1) & 0xFF);
      out_cmd = cmd01_;  // 复制出临界区，CAN 发送不持锁
    }

    const protocol::CanFrame f = protocol::CanProtocol::encodeAdcuCmd01(out_cmd);
    if (!can_driver_->send(f)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
          "CAN 发送失败 (ID=0x%08X)", f.id);
    }
  }

  void onCanReceive(const protocol::CanFrame& frame) {
    protocol::ChassisState01 s{};
    // decodeState01 声明即为 const protocol::CanFrame&，无需 const_cast
    if (protocol::CanProtocol::decodeState01(frame, s)) {
      // 心跳用 atomic 记录：CAN 接收线程与定时器线程并发，
      // 避免为读一个时间戳去抢 sendCommand() 的 cmd_mutex_。
      last_vcu_ns_.store(this->now().nanoseconds(), std::memory_order_relaxed);
      std::lock_guard<std::mutex> lock(state_mutex_);
      state01_ = s;
    }
    // State_02~23 的 decode 同理，暂略
  }

  void publishState() {
    protocol::ChassisState01 s{};
    {
      // 与 CAN 接收线程共享 state01_，读取必须加锁，否则是数据竞争 (UB)
      std::lock_guard<std::mutex> lock(state_mutex_);
      s = state01_;
    }
    std_msgs::msg::String msg;
    msg.data = fmtState(s);
    state_pub_->publish(msg);
  }

  static std::string fmtState(const protocol::ChassisState01& s) {
    char buf[256];
    snprintf(buf, sizeof(buf),
             "speed=%u mm/s gear=%u estop=%u mode=%u alive=%u bumper=%u",
             static_cast<unsigned>(s.speed_raw),
             static_cast<unsigned>(s.gear),
             static_cast<unsigned>(s.estop),
             static_cast<unsigned>(s.ctrl_mode),
             static_cast<unsigned>(s.alive_counter),
             static_cast<unsigned>(s.bumper));
    return std::string(buf);
  }

  // ---- 成员 ----
  std::unique_ptr<CanDriver> can_driver_;
  protocol::AdcuCommand01 cmd01_{};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steer_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  protocol::ChassisState01 state01_{};
  // static_cast<int64_t>(0) 是必需的：裸写 0 会让
  //   Time(int64_t, rcl_clock_type_t) 与 Time(int32_t, uint32_t, rcl_clock_type_t)
  // 两个重载都成为同等候选，GCC 报 "conversion ... is ambiguous"。
  rclcpp::Time last_cmd_time_{static_cast<int64_t>(0), RCL_ROS_TIME};
  rclcpp::Time last_steer_time_{static_cast<int64_t>(0), RCL_ROS_TIME};
  std::atomic<int64_t> last_vcu_ns_{0};
  std::mutex cmd_mutex_;
  std::mutex state_mutex_;

  double cmd_timeout_s_   = 0.5;
  double vcu_timeout_s_   = 1.0;
  double max_speed_m_s_   = 1.4;
  double max_steer_deg_   = 90.0;
  double publish_hz_      = 20.0;

  // 转向指令来源（均受 cmd_mutex_ 保护）
  std::string steer_cmd_topic_       = "/weeder/steer_cmd";
  double steer_timeout_s_            = 0.5;
  bool   steer_fallback_to_cmd_vel_  = true;
  uint8_t steer_mode_                = 0;
  double steer_cmd_deg_              = 0.0;  // 来自 /weeder/steer_cmd（权威）
  double steer_from_cmd_vel_deg_     = 0.0;  // 来自 /cmd_vel 的 angular.z（回退）
};

}  // namespace weeder_base

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    auto node = std::make_shared<weeder_base::ChassisNode>("chassis_node");
    rclcpp::spin(node);
    node.reset();  // 显式释放：确保 CAN 接收线程在 main 返回前停止
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("chassis_node"), "节点异常退出: %s", e.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
