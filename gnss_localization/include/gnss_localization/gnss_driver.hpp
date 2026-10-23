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
#include "gnss_localization/serial_port.hpp"
#include "weeder_msgs/msg/pose.hpp"
namespace gnss_localization {

// GIAVP 解算状态常量 (与 giavp_parser.hpp 中 GiavpData::status 语义一致)。
//   0=初始化/无解, 1=SPP, 2=RTD, 4=RTK 固定解, 5=RTK 浮点解 (无 3)。
constexpr int kStatusRtkFixed = 4;   // RTK 固定解

/// 双天线 MEMS 组合导航 (INS/GNSS) 驱动节点。
///
/// 从串口读取 $GIAVP 语句, 发布以下话题 (均为节点相对名, 完整名为
/// /gnss_driver/<topic>):
///   ~/fix            原始经纬度 (设备 INS 解算, NavSatFix)
///   ~/pose           修正 ENU 位姿 (weeder_msgs/Pose, 含姿态/卫星数/有效性/速度)
///   ~/pose_stamped   修正 ENU 位姿 (geometry_msgs/PoseStamped, 供 RViz 显示)
///   ~/odom_raw       原始里程计 (设备中心 gnss_link + 原始测向航向, 无任何修正), **仅供标定**
///   ~/imu            IMU 姿态 (含 ENU yaw), sensor_msgs/Imu
///   ~/vel            车体系速度 (TwistWithCovarianceStamped, x 前 y 左 z 上)
///   ~/origin     ENU 起点经纬度 (优先取 YAML origin_*, 否则取上电后首次固定解), transient_local +
///                    1 Hz 低频重发, 供 global_path_planner / local_path_planner
///                    作规划原点 (保证全链路同原点; path_controller 直接订阅
///                    ~/pose, 已含同原点的修正 ENU 位姿)。
class GnssDriver : public rclcpp::Node {
public:
  explicit GnssDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~GnssDriver() override;

private:
  enum class HealthState {
    kWaiting,
    kOk,
    kLowStatus,
    kGiavpTimeout,
    kSerial,
  };

  void timerCallback();
  void processLine(const std::string& line);
  void handleGiavp(const GiavpData& d);
  void updateHealth();
  void updateHealthState(HealthState state, const std::string& message);
  bool positionUsable(const GiavpData& d) const;
  void loadLogParameters();
  void loadParameters();

private:
  void publishFix(const GiavpData& d);
  void publishImu(const GiavpData& d);
  void publishVelocity(const GiavpData& d);
  void publishPose(const GiavpData& d);
  void publishOriginLlh();

private:
  double headingToEnuYaw(double heading_deg) const;
  double positionAccuracyForStatus(int status) const;
  rclcpp::Time timestampFor(const GiavpData& data) const;
  static int statusRank(int status);

private:
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<weeder_msgs::msg::Pose>::SharedPtr pose_pub_;
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
  std::unique_ptr<GeographicLib::LocalCartesian> datum_proj_;
  mutable std::mutex datum_mutex_;   // 保护 datum_proj_ (防御性: 当前读写均在 timer 回调线程, 无跨线程竞争)

private:
  std::string serial_port_path_;           // 串口设备号
  int baud_rate_{115200};                  // 串口波特率
  std::string frame_id_;                   // 发布的坐标系
  double heading_std_deg_{0.2};            // 航向标准差 (deg)
  double acc_rtk_fixed_{0.02};             // rtk 固定解
  double acc_rtk_float_{0.3};              // rtk 浮点解
  double acc_rtd_{1.0};                    // rtk 动态解
  double acc_spp_{1.5};                    // spp 解
  double calib_offset_x_{0.0};             // 杆臂 x (m, 车体系: 前 +x)
  double calib_offset_y_{0.0};             // 杆臂 y (m, 车体系: 左 +y)
  double calib_offset_heading_{0.0};       // 航向安装偏角 (rad)
  bool debug_giavp_{false};                // 是否打印 giavp 数据到日志
  int gnss_log_interval_{500};             // GNSS 状态日志条数间隔
  int pose_log_interval_{10};              // 位姿日志条数间隔
  double giavp_timeout_sec_{1.0};          // 无有效 GIAVP 数据超时
  bool use_gps_timestamp_{true};           // 是否使用 GIAVP GPS 时间戳
  int gps_utc_leap_seconds_{18};           // GPS 与 UTC 当前闰秒差

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
  bool has_giavp_{false};                                       // 本次串口连接是否收到过有效帧
  int last_status_{0};                                          // 最近一帧解算状态
  HealthState last_health_state_{HealthState::kWaiting};        // 最近健康状态
  std::chrono::steady_clock::time_point last_giavp_time_{};     // 最近有效帧时刻
};

}   // namespace gnss_localization
