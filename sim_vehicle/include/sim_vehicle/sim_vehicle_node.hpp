#ifndef SIM_VEHICLE_SIM_VEHICLE_NODE_H_
#define SIM_VEHICLE_SIM_VEHICLE_NODE_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sim_vehicle/kinematics.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "weeder_msgs/msg/chassis_cmd.hpp"
#include "weeder_msgs/msg/chassis_state.hpp"
#include "weeder_msgs/msg/path.hpp"
#include "weeder_msgs/msg/pose.hpp"
#include "weeder_msgs/msg/work_mode.hpp"

namespace sim_vehicle {

class SimVehicleNode : public rclcpp::Node {
 public:
  explicit SimVehicleNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void Initialize();
  void LoadParameters();
  void LoadGeometryParameters();
  void LoadPoseParameters();
  void SetupInterfaces();
  void SetupTimers();

  void OnChassisCmd(const weeder_msgs::msg::ChassisCmd::SharedPtr message);
  void OnUnmanned(const std_msgs::msg::Bool::SharedPtr message);
  void OnPath(const weeder_msgs::msg::Path::SharedPtr message);
  void Tick();
  void PublishPose();
  void PublishTf();
  void PublishMarker();
  void AppendBodyMarkers(visualization_msgs::msg::MarkerArray& markers);
  void AppendWheelMarkers(visualization_msgs::msg::MarkerArray& markers);
  void PublishChassisState();
  void PublishSlowTopics();

  double EffectiveWheelbase() const;
  visualization_msgs::msg::Marker CreateMarker(int32_t marker_id,
                                               int32_t marker_type);
  std::pair<double, double> LocalToWorld(double local_x_m,
                                         double local_y_m) const;
  static void SetYawOrientation(visualization_msgs::msg::Marker& marker,
                                double yaw_rad);
  static void SetWheelOrientation(visualization_msgs::msg::Marker& marker,
                                  double yaw_rad);

  double wheelbase_m_{kWheelbaseM};           // 前后轴距
  double wheel_track_m_{kWheelTrackM};        // 左右轮中心距
  double wheel_radius_m_{kWheelRadiusM};      // 车轮半径
  double vehicle_length_m_{kVehicleLengthM};  // 车体长度
  double vehicle_width_m_{kVehicleWidthM};    // 车体宽度
  double vehicle_height_m_{kVehicleHeightM};  // 车体高度
  double dt_s_{0.02};                         // 仿真积分周期
  double max_speed_m_s_{kMaxSpeedMps};        // 底盘最大速度
  double max_plant_steer_rad_{kMaxSteerRad};  // 单轮最大转角
  double cmd_timeout_s_{0.3};                 // 控制指令超时
  std::string cmd_topic_{"/chassis/cmd"};     // 控制指令话题

  double init_x_m_{0.0};                // 初始 x 坐标
  double init_y_m_{0.0};                // 初始 y 坐标
  double init_yaw_rad_{0.0};            // 初始航向
  double origin_lat_{34.6198653};       // 模拟 GNSS 原点纬度
  double origin_lon_{112.4497271};      // 模拟 GNSS 原点经度
  double origin_alt_m_{100.0};          // 模拟 GNSS 原点高度
  bool start_unmanned_{false};          // 启动时是否处于无人模式
  bool align_to_path_start_{true};      // 是否对齐到路径起点
  double align_lateral_offset_m_{0.2};  // 初始横向偏差

  double x_m_{0.0};                     // 当前 x 坐标
  double y_m_{0.0};                     // 当前 y 坐标
  double yaw_rad_{0.0};                 // 当前航向
  double speed_m_s_{0.0};               // 当前速度
  double speed_cmd_m_s_{0.0};           // 当前速度指令
  double steer_cmd_rad_{0.0};           // 当前等效转角指令
  uint8_t steer_mode_{0};               // 当前转向模式
  bool unmanned_{false};                // 当前工作模式
  bool aligned_{false};                 // 是否已完成路径起点对齐
  std::size_t steer_clamped_count_{0};  // 转角夹取累计次数
  FourWheelState wheel_state_;          // 四轮运动学状态

  uint8_t cmd_gear_{0};            // 最近一次档位指令
  bool cmd_release_brake_{false};  // 最近一次释放抱闸指令
  bool cmd_estop_{false};          // 最近一次急停指令
  bool cmd_ctrl_mode_{false};      // 最近一次控制模式指令
  uint8_t alive_counter_{0};       // 底盘状态生命信号
  rclcpp::Time last_cmd_time_;     // 最近一次收到指令的时刻

  rclcpp::Subscription<weeder_msgs::msg::ChassisCmd>::SharedPtr
    chassis_cmd_sub_;  // 底盘指令订阅
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    unmanned_sub_;  // 工作模式订阅
  rclcpp::Subscription<weeder_msgs::msg::Path>::SharedPtr
    path_sub_;  // 路径订阅
  rclcpp::Publisher<weeder_msgs::msg::Pose>::SharedPtr
    pose_pub_;  // 定位位姿发布
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    pose_stamped_pub_;  // 标准位姿发布
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr
    origin_pub_;  // GNSS 原点发布
  rclcpp::Publisher<weeder_msgs::msg::WorkMode>::SharedPtr
    work_mode_pub_;  // 工作模式发布
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr
    tf_pub_;  // 车辆 TF 发布
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_pub_;  // 车辆可视化发布
  rclcpp::Publisher<weeder_msgs::msg::ChassisState>::SharedPtr
    chassis_state_pub_;  // 底盘状态发布

  rclcpp::TimerBase::SharedPtr tick_timer_;           // 仿真主循环
  rclcpp::TimerBase::SharedPtr chassis_state_timer_;  // 底盘状态定时器
  rclcpp::TimerBase::SharedPtr slow_topic_timer_;     // 慢速话题定时器
};

}  // namespace sim_vehicle

#endif  // SIM_VEHICLE_SIM_VEHICLE_NODE_H_
