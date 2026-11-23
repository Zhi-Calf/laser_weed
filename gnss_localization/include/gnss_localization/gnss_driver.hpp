#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <chrono>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <string>

#include "gnss_localization/giavp_parser.hpp"
#include "gnss_localization/gga_status_parser.hpp"
#include "gnss_localization/inspvaa_parser.hpp"
#include "gnss_localization/serial_port.hpp"
#include "vehicle_interfaces/msg/pose.hpp"
namespace gnss_localization {

// GIAVP 解算状态常量 (与 giavp_parser.hpp 中 GiavpData::status 语义一致)。
//   0=初始化/无解, 1=SPP, 2=RTD, 4=RTK 固定解, 5=RTK 浮点解 (无 3)。
constexpr int kStatusRtkFixed = 4;   // RTK 固定解

/// 双天线 MEMS 组合导航 (INS/GNSS) 驱动节点。
///
/// 从串口读取 BR100A 的 $GIAVP，或 NAV3120 的 INSPVAA + GGA。
/// 两种硬件共用以下固定话题，运行时节点名不会改变外部接口：
///   /gnss_driver/fix
///   /gnss_driver/pose
///   /gnss_driver/pose_stamped
///   /gnss_driver/odom_raw
///   /gnss_driver/imu
///   /gnss_driver/vel
///   /gnss_driver/origin
class GnssDriver : public rclcpp::Node {
public:
  explicit GnssDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  explicit GnssDriver(
      const std::string& node_name,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~GnssDriver() override;

private:
  enum class HealthState {
    kWaiting,
    kOk,
    kLowStatus,
    kNavigationTimeout,
    kSerial,
  };

  // 读取并解析串口数据，更新健康状态和输出。
  void timerCallback();
  // 处理一行接收到的设备数据。
  void processLine(const std::string& line);
  // 处理一帧有效的 GIAVP 数据。
  void handleGiavp(const GiavpData& d);
  // 将 INSPVAA 融合导航和最新 GGA 质量合并为统一数据。
  GiavpData mergeInspvaaWithGga(const InspvaaData& d) const;
  // 判断最近一帧 GGA 质量数据是否在有效期内。
  bool ggaFresh() const;
  // 根据数据新鲜度与定位质量更新健康状态。
  void updateHealth();
  // 更新健康状态并限制重复日志。
  void updateHealthState(HealthState state, const std::string& message);
  // 判断当前位置是否可用于输出。
  bool positionUsable(const GiavpData& d) const;
  // 读取日志配置参数。
  void loadLogParameters();
  // 读取 GNSS 与标定参数。
  void loadParameters();

private:
  // 发布原始经纬度定位。
  void publishFix(const GiavpData& d);
  // 发布 IMU 姿态数据。
  void publishImu(const GiavpData& d);
  // 发布车体系速度。
  void publishVelocity(const GiavpData& d);
  // 发布修正后的 ENU 位姿和原始里程计。
  void publishPose(const GiavpData& d);
  // 发布 ENU 原点经纬度。
  void publishOriginLlh();

private:
  // 将设备航向转换为 ENU yaw。
  double headingToEnuYaw(double heading_deg) const;
  // 返回解算状态对应的水平位置标准差。
  double positionAccuracyForStatus(int status) const;
  // 生成设备 GPS 时间对应的时间戳。
  rclcpp::Time timestampFor(const GiavpData& data) const;
  // 返回解算状态的精度排序等级。
  static int statusRank(int status);

private:
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<vehicle_interfaces::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr origin_llh_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

private:
  SerialPort serial_;
  GiavpParser parser_;
  InspvaaParser inspvaa_parser_;
  GgaStatusParser gga_status_parser_;
  std::unique_ptr<GeographicLib::LocalCartesian> datum_proj_;
  mutable std::mutex datum_mutex_;   // 保护 datum_proj_ (防御性: 当前读写均在 timer 回调线程, 无跨线程竞争)

private:
  std::string serial_port_path_;       // 串口设备号
  int baud_rate_{115200};              // 串口波特率
  std::string input_protocol_{"giavp"};   // 输入协议: giavp 或 inspvaa_gga
  std::string frame_id_;               // 发布的坐标系
  double heading_std_deg_{0.2};        // 航向标准差 (deg)
  double acc_rtk_fixed_{0.02};         // rtk 固定解
  double acc_rtk_float_{0.3};          // rtk 浮点解
  double acc_rtd_{1.0};                // rtk 动态解
  double acc_spp_{1.5};                // spp 解
  double calib_offset_x_{0.0};         // 杆臂 x (m, 车体系: 前 +x)
  double calib_offset_y_{0.0};         // 杆臂 y (m, 车体系: 左 +y)
  double calib_offset_heading_{0.0};   // 航向安装偏角 (rad)
  bool debug_giavp_{false};            // 是否打印 giavp 数据到日志
  int gnss_log_interval_{500};         // GNSS 状态日志条数间隔
  int pose_log_interval_{10};          // 位姿日志条数间隔
  double giavp_timeout_sec_{1.0};      // 无有效 GIAVP 数据超时
  double gga_timeout_sec_{1.5};        // INSPVAA 模式下无有效 GGA 质量数据超时
  bool use_gps_timestamp_{true};       // 是否使用 GIAVP GPS 时间戳
  int gps_utc_leap_seconds_{18};       // GPS 与 UTC 当前闰秒差

private:
  std::string log_dir_     = "/opt/agri/logs";   // 日志目录
  int log_file_size_mb_    = 5;                  // 日志文件大小 (MB)
  int log_file_count_      = 10;                 // 日志文件数量
  bool log_enable_console_ = true;               // 是否输出到控制台
  std::string log_level_   = "info";             // 日志级别

private:
  int gnss_log_counter_{0};                                     // GNSS 日志计数
  int pose_log_counter_{0};                                     // 位姿日志计数
  rclcpp::Time last_origin_publish_time_{0, 0, RCL_ROS_TIME};   // 10Hz 节流
  double datum_lat_deg_{0.0};                                   // datum 纬度
  double datum_lon_deg_{0.0};                                   // datum 经度
  double datum_alt_m_{0.0};                                     // datum 高程
  bool datum_from_config_{false};                               // ENU 原点是否来自 YAML
  bool first_heading_logged_{false};                            // 是否已打印过首个航向日志
  bool has_navigation_{false};                                  // 是否收到过有效导航帧
  bool has_gga_{false};                                         // 是否收到过有效 GGA 质量帧
  int last_status_{0};                                          // 最近一帧解算状态
  HealthState last_health_state_{HealthState::kWaiting};        // 最近健康状态
  std::chrono::steady_clock::time_point last_navigation_time_{};   // 最近有效导航帧时刻
  std::chrono::steady_clock::time_point last_gga_time_{};          // 最近有效 GGA 时刻
  GgaFixStatus last_gga_status_;                                // 最近一帧 GGA 质量
};

}   // namespace gnss_localization
