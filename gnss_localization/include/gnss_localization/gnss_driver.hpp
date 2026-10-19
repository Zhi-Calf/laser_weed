#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <utility>
#include <vector>

#include "gnss_localization/giavp_parser.hpp"
#include "gnss_localization/serial_port.hpp"
#include "weeder_msgs/msg/gnss_status.hpp"
namespace gnss_localization {

// GIAVP 解算状态常量 (与 giavp_parser.hpp 中 GiavpData::status 语义一致)。
//   0=初始化/无解, 1=SPP, 2=RTD, 4=RTK 固定解, 5=RTK 浮点解 (无 3)。
constexpr int kStatusRtkFixed = 4;   // RTK 固定解

/// 双天线 MEMS 组合导航 (INS/GNSS) 驱动节点。
///
/// 从串口读取 $GIAVP 语句, 发布以下话题 (均为节点相对名, 完整名为
/// /gnss_driver/<topic>):
///   ~/fix            原始经纬度 (设备 INS 解算, NavSatFix)
///   ~/odom           修正 ENU 里程计 (已减杆臂 calib_offset_* + 航向偏角修正)
///   ~/odom_raw       原始里程计 (设备中心 gnss_link + 原始测向航向, 无任何修正), **仅供标定**
///   ~/imu            IMU 姿态 (含 ENU yaw), sensor_msgs/Imu
///   ~/vel            车体系速度 (TwistWithCovarianceStamped, x 前 y 左 z 上)
///   ~/status         GNSS 状态 (含原始航向、解算状态, GnssStatus)
///   ~/origin     ENU 起点经纬度 (优先取 YAML origin_*, 否则取上电后首次固定解), transient_local +
///                    1 Hz 低频重发, 供 global_path_planner / local_path_planner
///                    作规划原点 (保证全链路同原点; path_controller 直接订阅
///                    ~/odom, 已含同原点的修正 ENU 位姿)。
class GnssDriver : public rclcpp::Node {
public:
  explicit GnssDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~GnssDriver() override;

private:
  void timerCallback();
  void processLine(const std::string& line);
  void handleGiavp(const GiavpData& d);
  void loadLogParameters();
  void loadParameters();

private:
  void publishFix(const GiavpData& d);
  void publishImu(const GiavpData& d);
  void publishVelocity(const GiavpData& d);
  void publishStatus(const GiavpData& d);
  void publishOdom(const GiavpData& d);
  void publishOriginLlh();
  rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter>& parameters);

private:
  double headingToEnuYaw(double heading_deg) const;
  double positionAccuracyForStatus(int status) const;
  static int statusRank(int status);

private:
  enum class PersistOutcome { kPersisted, kSkippedNoPath, kFailed };
  PersistOutcome persistParamsToYaml(const std::vector<std::pair<std::string, double>>& updates);

private:
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<weeder_msgs::msg::GnssStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr origin_llh_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_cb_handle_;

private:
  SerialPort serial_;
  GiavpParser parser_;
  std::unique_ptr<GeographicLib::LocalCartesian> datum_proj_;
  mutable std::mutex datum_mutex_;   // 保护 datum_proj_ (防御性: 当前读写均在 timer 回调线程, 无跨线程竞争)

private:
  std::string serial_port_path_;           // 串口设备号
  int baud_rate_{115200};                  // 串口波特率
  std::string frame_id_;                   // 发布的坐标系
  std::string gnss_config_path_;           // 配置文件路径
  double heading_std_deg_{0.2};            // 航向标准差 (deg)
  double acc_rtk_fixed_{0.02};             // rtk 固定解
  double acc_rtk_float_{0.3};              // rtk 浮点解
  double acc_rtd_{1.0};                    // rtk 动态解
  double acc_spp_{1.5};                    // spp 解
  double calib_offset_x_{0.0};             // 杆臂 x (m, 车体系: 前 +x)
  double calib_offset_y_{0.0};             // 杆臂 y (m, 车体系: 左 +y)
  double calib_offset_heading_{0.0};       // 航向安装偏角 (rad)
  double calib_offset_max_xy_{2.5};        // calib_offset_x/y 范围上限 (m, 绝对杆臂长度)
  double calib_offset_max_heading_{0.6};   // calib_offset_heading 范围上限 (rad, ≈34°)
  mutable std::mutex calib_mutex_;         // 保护 calib_offset_* (参数回调 vs 定时器线程)
  bool debug_giavp_{false};                // 是否打印 giavp 数据到日志
  int status_log_interval_{500};           // 状态日志条数间隔 (10Hz 下 500 条 = 50s)
  int odom_log_interval_{10};              // 里程计日志条数间隔 (10Hz 下 10 条 = 1s)

private:
  std::string log_dir_     = "/opt/agri/logs";   // 日志目录
  int log_file_size_mb_    = 5;                  // 日志文件大小 (MB)
  int log_file_count_      = 10;                 // 日志文件数量
  bool log_enable_console_ = true;               // 是否输出到控制台
  std::string log_level_   = "info";             // 日志级别

private:
  int status_log_counter_{0};                                   // 状态日志计数 (阈值见 status_log_interval_)
  int odom_log_counter_{0};                                     // 里程计日志计数 (阈值见 odom_log_interval_)
  rclcpp::Time last_origin_publish_time_{0, 0, RCL_ROS_TIME};   // 10Hz 节流
  double datum_lat_deg_{0.0};                                   // datum 纬度
  double datum_lon_deg_{0.0};                                   // datum 经度
  double datum_alt_m_{0.0};                                     // datum 高程
  bool datum_from_config_{false};                               // ENU 原点是否来自 YAML
  bool first_heading_logged_{false};                            // 是否已打印过首个航向日志
};

}   // namespace gnss_localization
