#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

namespace gnss_localization {

class SerialPort;

// NTRIP 差分客户端节点。
//
// 功能 (与原独立 ntrip.cpp 一致):
//   1. 打开串口, 读取接收机输出的 GGA 语句;
//   2. 连接千寻 NTRIP caster, 周期上传 GGA (定位请求);
//   3. 接收 RTCM 差分数据并写回串口, 供接收机解算 RTK;
//   4. 断线自动重连。
//
// 这是"外置 NTRIP 客户端"方案 —— 适用于接收机本身没有 4G/网络模块的场合,
// 由上位机代理差分数据。若接收机内置 4G NTRIP, 则用
// startup_commands 里的 `set ntrip ...` 命令, 不需要本节点。
class NtripClient : public rclcpp::Node {
public:
  explicit NtripClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~NtripClient() override;

private:
  void loadLogParameters();
  void loadParameters();
  void workerLoop();
  std::optional<std::string> tryReadGga();
  std::optional<std::pair<std::string, std::chrono::steady_clock::time_point>> waitForGga(double timeout_seconds);
  bool openSerial();
  int openNtripSocket(std::string& pending_data);
  std::string buildRequest() const;
  bool checkGga(const std::string& line) const;

private:
  // 内层数据交互结束原因。
  enum class SessionEnd {
    kGgaTimeout,     // GGA 数据超时 → 重建期间保持 FAULT_GGA_TIMEOUT
    kRtcmTimeout,    // RTCM 数据超时 → 重建期间保持 FAULT_RTCM_TIMEOUT
    kDisconnected,   // 连接断开/收发失败/正常退出 → 重建, 不报数据故障
  };
  SessionEnd runSession(int socket_fd, const std::string& first_gga, std::chrono::steady_clock::time_point first_gga_time, std::string pending_data);

private:
  std::unique_ptr<SerialPort> serial_;
  std::thread worker_thread_;
  std::atomic<bool> running_{false};

private:
  std::string serial_port_{"/dev/ttysWK1"};        // 串口设备
  int serial_baud_{115200};                        // 波特率
  std::string ntrip_host_{"rtk.ntrip.qxwz.com"};   // ntrip 服务器
  int ntrip_port_{8002};                           // ntrip 端口
  std::string ntrip_user_{""};                     // ntrip 用户名
  std::string ntrip_password_{""};                 // ntrip 密码
  std::string ntrip_mountpoint_{"AUTO"};           // ntrip mountpoint
  double gga_interval_sec_{1.0};                   // GGA 上传周期 (s)
  double reconnect_delay_sec_{1.0};                // NTRIP 连接失败后的重连延迟 (s); 串口打开/等待 GGA 等本地重试固定 200ms
  double first_rtcm_timeout_sec_{30.0};            // 等待首帧 RTCM 超时 (s)
  double first_gga_timeout_sec_{30.0};             // 等待首个 GGA 超时 (s): 接收机定位收敛 (<35s)
  double data_timeout_sec_{10.0};                  // 正常后数据超时 (s)
  double connect_timeout_sec_{10.0};               // TCP 连接超时 (s)
  double header_timeout_sec_{15.0};                // 响应头读取累计超时 (s)
  int serial_write_fail_count_{10};                // 串口写 RTCM 连续失败次数阈值
  std::atomic<int> consecutive_write_fail_{0};     // 串口写 RTCM 连续失败计数
  std::string cached_ip_;                          // DNS 缓存: 首次成功后缓存, 后续重连跳过阻塞的 getaddrinfo
private:
  std::string log_dir_     = "/opt/agri/logs";   // 日志目录
  int log_file_size_mb_    = 5;                  // 日志文件大小 (MB)
  int log_file_count_      = 10;                 // 日志文件数量
  bool log_enable_console_ = true;               // 是否输出到控制台
  std::string log_level_   = "info";             // 日志级别
};

}   // namespace gnss_localization
