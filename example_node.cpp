// ROS2 节点集成示例 (参考用法, 非必须编译)
// 展示如何将 can_protocol + can_driver 接入 ROS2
//
// 编译方式: 放入 ROS2 package, 链接 rclcpp + can_driver
//
// ⚠️ 第二版协议已取消 Chassis_State_23 (0x18231CD0),
//    电芯最高电压 / VCU 控制器电压改由 State_22 提供,
//    电芯最低电压及温度类信号在第二版中已无对应报文。

#include "laser_weed_can/protocol/can_protocol.hpp"
// #include "tractor_base/can_driver.hpp"  // 复用底层驱动
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace laser_weed;

class LaserWeedNode : public rclcpp::Node {
public:
  LaserWeedNode() : Node("laser_weed_can_node"), alive_(0) {
    speed_pub_ = create_publisher<std_msgs::msg::Float32>("chassis/actual_speed", 10);
    // CanDriver driver_("can0");  // 实机用 can0, 仿真用 vcan0
    // driver_.setFilters({can_id::CHASSIS_STATE_01, can_id::CHASSIS_STATE_03, ...});
    // driver_.setReceiveCallback([this](const CanFrame& f){ onReceive(f); });
    // driver_.open();
    // driver_.startReceiveThread();

    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               [this]() { sendCommand(); });
  }

private:
  void onReceive(const CanFrame& f) {
    // State_01: 底盘主状态
    if (f.id == can_id::CHASSIS_STATE_01) {
      ChassisState01 s;
      if (CanProtocol::decodeState01(f, s)) {
        auto msg = std_msgs::msg::Float32();
        msg.data = s.speed_m_s;
        speed_pub_->publish(msg);
        // s.estop / s.ctrl_mode / s.steer_done_fl... 按需发布
      }
    }
    // State_03: 四轮转向角
    else if (f.id == can_id::CHASSIS_STATE_03) {
      ChassisState03 s;
      if (CanProtocol::decodeState03(f, s)) {
        // 发布转向角... (物理值 = raw * 0.1)
      }
    }
    // State_22: BMS 故障码 / 电芯最高电压 / VCU 控制器电压 (第二版)
    else if (f.id == can_id::CHASSIS_STATE_22) {
      ChassisState22 s;
      if (CanProtocol::decodeState22(f, s)) {
        // s.error_code       → 电池故障码 (第二版为 16bit)
        // s.max_cell_voltage → 电芯最高电压 mV
        // s.system_voltage   → VCU 控制器电压 mV
        // s.cycle_count      → 放电循环次数
      }
    }
    // ... State_02 / 04~11 / 21 类似 (第二版已取消 State_23)
  }

  void sendCommand() {
    AdcuCommand01 cmd;
    cmd.gear = 1;            // D档
    cmd.ctrl_mode = 1;       // 无人驾驶
    cmd.release_brake = 1;
    cmd.speed_raw = 500;     // 500 mm/s = 0.5 m/s
    cmd.steer_angle = 0;     // 直行
    cmd.alive_counter = alive_++;
    CanFrame f = CanProtocol::encodeAdcuCmd01(cmd);
    // driver_.send(f);
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  // CanDriver driver_;
  uint8_t alive_;
};

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<LaserWeedNode>());
//   rclcpp::shutdown();
// }
