#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

namespace gnss_localization {

class SerialPort;

// 千寻 SDK 坐标系索引 (qxwz_sdk_set_coord_sys 的 coord_sys_index 参数)。
// 参考《千寻SDK开发指南》13.2 节: 1:ITRF2008; 2:WGS84; 3:CGCS2000。
// SDK 头文件未提供对应宏, 这里按官方文档取值命名。
constexpr uint32_t kQxwzCoordSysWgs84 = 2;

// 千寻 SDK 差分客户端节点 (NOSR/RTCM 差分)。
//
// 功能 (与 rtk_driver 的 rtk_diff_node 一致):
//   1. 打开串口, 读取接收机输出的 GGA 语句;
//   2. 初始化并认证千寻 SDK (DSK 密钥);
//   3. 通过 SDK 周期上传 GGA (定位请求);
//   4. 在 SDK data 回调中接收 RTCM 差分数据并写回串口;
//   5. 断线/异常由 SDK 内部处理, 本节点负责串口层重连。
//
// 与 ntrip_client 的区别: ntrip_client 手动实现 NTRIP 协议 (socket + HTTP),
// 本节点则调用千寻 SDK 静态库 (libqxwz-pssdk), 网络/认证/重连由 SDK 托管。
// 二者为"二选一"的替代差分方案, 由 bringup 的 diff_mode 参数决定启动哪个。
class SdkClient : public rclcpp::Node {
public:
  explicit SdkClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SdkClient() override;
  bool writeRtcm(const uint8_t* data, uint32_t len);

public:
  std::atomic<bool> sdk_auth_ready_{false};          // 认证是否成功 (auth_cb 置 true)
  std::atomic<int32_t> sdk_error_code_{0};           // SDK 错误码: 认证失败 (auth_cb) / 运行中内部错误 (status_cb)
  std::atomic<bool> sdk_nosr_start_failed_{false};   // NOSR 启动异步失败 (start_cb 置 true)
  std::atomic<bool> sdk_coordsys_done_{false};       // 坐标系设置回调是否已返回
  std::atomic<int32_t> sdk_coordsys_code_{0};        // 坐标系设置回调返回码 (0 = 成功)
  std::atomic<int64_t> last_rtcm_ms_{0};             // 最后收到 RTCM 的 steady_clock 毫秒时间戳
  std::atomic<int> consecutive_write_fail_{0};       // 连续写 RTCM 失败的帧数 (写成功即清零)
  std::atomic<int> transient_error_count_{0};        // 会话内累计 SDK 瞬时错误数 (网络/数据质量类, 不因单次成功清零, 会话/重连时重置)

private:
  void loadLogParameters();
  void loadParameters();
  void workerLoop();
  bool openSerial();
  std::optional<std::string> tryReadGga();
  std::optional<std::pair<std::string, std::chrono::steady_clock::time_point>> waitForGga(double timeout_seconds);
  bool checkGga(const std::string& line) const;
  bool initSdk();
  bool startNosr();

private:
  // 内层数据交互结束原因。
  enum class SessionEnd {
    kGgaTimeout,          // GGA 数据超时
    kRtcmTimeout,         // RTCM 数据超时
    kSdkError,            // SDK 内部错误 (status_cb 报告)
    kSdkTransientError,   // SDK 瞬时错误持续高频 (网络/数据质量差, 累计超阈值)
    kSerialWriteFail,     // 串口写 RTCM 失败 (收 RTCM 但写不进串口, data_cb 报告)
    kNosrStartFail,       // NOSR 启动异步失败 (start_cb 报告)
    kDisconnected,        // running_ 变 false / 正常退出
  };
  SessionEnd runSession(const std::string& first_gga, std::chrono::steady_clock::time_point first_gga_time);

private:
  std::unique_ptr<SerialPort> serial_;
  std::thread worker_thread_;
  std::atomic<bool> running_{false};
  std::mutex serial_mutex_;

private:
  std::string serial_port_{"/dev/ttysWK0"};   // 串口设备路径
  int serial_baud_{115200};                   // 串口波特率
  std::string sdk_key_{""};                   // SDK key (DSK)
  std::string sdk_secret_{""};                // SDK secret (DSS)
  std::string dev_id_{""};                    // 设备 ID
  std::string dev_type_{""};                  // 设备类型
  double gga_interval_sec_{1.0};              // GGA 上传周期 (s)
  double sdk_reconnect_delay_sec_{1.0};       // SDK 会话重建延迟 (s); 串口打开/等待 GGA 等本地重试固定 200ms
  double first_gga_timeout_sec_{30.0};        // 等待首个 GGA 超时 (s)
  double first_rtcm_timeout_sec_{30.0};       // 等待首帧 RTCM 超时 (s)
  double sdk_auth_timeout_sec_{10.0};         // SDK 认证回调超时 (s)
  double coordsys_timeout_sec_{1.0};          // 设置坐标系回调超时 (s)
  double gga_timeout_sec_{30.0};              // GGA 停摆超时 (s): 持续无新 GGA 判停摆 (接收机可能因信号差短暂停发, 故较宽容)
  double rtcm_timeout_sec_{10.0};             // RTCM 停摆超时 (s): 差分断流判停摆 (需尽快重连, 故较严格)
  int serial_write_fail_count_{10};           // 串口写 RTCM 连续失败帧数阈值: 连续失败达到才上报 FAULT_SERIAL_WRITE
  int sdk_transient_error_max_{20};           // SDK 瞬时错误累计上限: 达到则判差分链路质量持续差, 重建会话

private:
  std::string log_dir_     = "/opt/agri/logs";   // 日志目录
  int log_file_size_mb_    = 5;                  // 日志文件大小 (MB)
  int log_file_count_      = 10;                 // 日志文件数量
  bool log_enable_console_ = true;               // 是否输出到控制台
  std::string log_level_   = "info";             // 日志级别
};

}   // namespace gnss_localization
