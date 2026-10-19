#include "gnss_localization/ntrip_client.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <system_error>

#include "gnss_localization/fault_codes.hpp"
#include "gnss_localization/serial_port.hpp"

namespace gnss_localization {

namespace {

// 扩展路径中的环境变量
std::string ExpandPath(std::string path) {
  if (path.rfind("~/", 0) == 0) {
    const char* home = std::getenv("HOME");
    if (home) {
      path = std::string(home) + path.substr(1);
    }
  }
  std::string out;
  for (size_t i = 0; i < path.size(); ++i) {
    if (path[i] != '$') {
      out += path[i];
      continue;
    }
    size_t j = i + 1;
    while (j < path.size() && (std::isalnum(static_cast<unsigned char>(path[j])) || path[j] == '_')) {
      ++j;
    }
    const std::string name = path.substr(i + 1, j - i - 1);
    const char* val        = name.empty() ? nullptr : std::getenv(name.c_str());
    if (val) {
      out += val;
    }
    i = j - 1;
  }
  return out;
}

// 解析日志目录
std::string ResolveLogDir(const std::string& configured) {
  std::error_code ec;
  const std::string preferred = ExpandPath(configured);
  if (!preferred.empty()) {
    std::filesystem::create_directories(preferred, ec);
    if (!ec && access(preferred.c_str(), W_OK) == 0) {
      return preferred;
    }
  }

  const char* ros_home = std::getenv("ROS_HOME");
  const char* home     = std::getenv("HOME");
  std::string fallback;
  if (ros_home && *ros_home) {
    fallback = std::string(ros_home) + "/logs";
  } else if (home && *home) {
    fallback = std::string(home) + "/.ros/logs";
  } else {
    return "";
  }

  std::filesystem::create_directories(fallback, ec);
  if (ec || access(fallback.c_str(), W_OK) != 0) {
    return "";
  }
  return fallback;
}

// 配置日志
void ConfigureLogger(const std::string& log_dir, size_t max_file_size_bytes, size_t max_file_count, bool enable_console, const std::string& log_level) {
  constexpr const char* kLogPattern = "[%Y-%m-%d %H:%M:%S.%e] [%l] [%s:%#] %v";
  const std::string dir             = ResolveLogDir(log_dir);
  if (dir.empty()) {
    throw std::runtime_error("no writable log directory (tried \"" + log_dir + "\")");
  }

  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>((std::filesystem::path(dir) / "ntrip_client.log").string(), max_file_size_bytes, max_file_count));
  if (enable_console) {
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  }

  auto logger = std::make_shared<spdlog::logger>("ntrip_client", sinks.begin(), sinks.end());
  logger->set_pattern(kLogPattern);
  const spdlog::level::level_enum level = spdlog::level::from_str(log_level);
  if (level == spdlog::level::off && log_level != "off") {
    logger->set_level(spdlog::level::info);
    SPDLOG_WARN("无法识别的日志级别 \"{}\", 已回退到 info", log_level);
  } else {
    logger->set_level(level);
  }
  spdlog::set_default_logger(logger);
  if (dir != ExpandPath(log_dir)) {
    SPDLOG_WARN("日志目录 \"{}\" 不可写，已回退到 \"{}\"", log_dir, dir);
  }
  spdlog::flush_on(spdlog::level::warn);
  spdlog::flush_every(std::chrono::seconds(3));
}

// ntip要求base64 编码
std::string base64Encode(const std::string& input) {
  static const char kTable[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((input.size() + 2) / 3 * 4);
  int val  = 0;
  int valb = -6;
  for (unsigned char c : input) {
    val = (val << 8) + c;
    valb += 8;
    while (valb >= 0) {
      out.push_back(kTable[(val >> valb) & 0x3F]);
      valb -= 6;
    }
  }
  if (valb > -6) {
    out.push_back(kTable[((val << 8) >> (valb + 8)) & 0x3F]);
  }
  while (out.size() % 4) {
    out.push_back('=');
  }
  return out;
}

// 发送数据 (循环直到全部发完)。失败 (含对端关闭) 返回 false。
[[nodiscard]] bool sendAll(int fd, const char* data, size_t size) {
  size_t sent = 0;
  while (sent < size) {
    // MSG_NOSIGNAL: 对端关闭时避免触发 SIGPIPE 杀死进程。
    const ssize_t n = ::send(fd, data + sent, size - sent, MSG_NOSIGNAL);
    if (n < 0) {
      if (errno == EINTR)
        continue;
      SPDLOG_WARN("发送失败: {}", std::strerror(errno));
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return true;
}

}   // namespace

NtripClient::NtripClient(const rclcpp::NodeOptions& options) : Node("ntrip_client", options) {
  loadLogParameters();

  // 配置 spdlog 滚动文件输出
  try {
    ConfigureLogger(log_dir_, static_cast<size_t>(log_file_size_mb_) * 1024U * 1024U, static_cast<size_t>(log_file_count_), log_enable_console_, log_level_);
    SPDLOG_INFO("日志配置成功");
  } catch (const std::exception& e) {
    SPDLOG_WARN("文件日志配置失败，使用默认日志器: {}", e.what());
  }

  loadParameters();

  serial_ = std::make_unique<SerialPort>();
  running_       = true;
  worker_thread_ = std::thread(&NtripClient::workerLoop, this);
}

NtripClient::~NtripClient() {
  running_ = false;
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void NtripClient::loadLogParameters() {
  // spdlog 滚动日志参数
  declare_parameter("log_dir", "/opt/agri/logs");
  declare_parameter("log_file_size_mb", 5);
  declare_parameter("log_file_count", 10);
  declare_parameter("log_enable_console", true);
  declare_parameter("log_level", "info");

  log_dir_            = get_parameter("log_dir").as_string();
  log_file_size_mb_   = static_cast<int>(std::max<int64_t>(1, get_parameter("log_file_size_mb").as_int()));
  log_file_count_     = static_cast<int>(std::max<int64_t>(1, get_parameter("log_file_count").as_int()));
  log_enable_console_ = get_parameter("log_enable_console").as_bool();
  log_level_          = get_parameter("log_level").as_string();
}

void NtripClient::loadParameters() {
  declare_parameter("serial_port", "/dev/ttysWK1");
  declare_parameter("serial_baud", 115200);
  declare_parameter("ntrip_host", "rtk.ntrip.qxwz.com");
  declare_parameter("ntrip_port", 8002);
  declare_parameter("ntrip_user", "");
  declare_parameter("ntrip_password", "");
  declare_parameter("ntrip_mountpoint", "AUTO");
  declare_parameter("gga_interval_sec", 1.0);
  declare_parameter("reconnect_delay_sec", 1.0);
  declare_parameter("first_rtcm_timeout_sec", 30.0);
  declare_parameter("first_gga_timeout_sec", 30.0);
  declare_parameter("data_timeout_sec", 10.0);
  declare_parameter("connect_timeout_sec", 10.0);
  declare_parameter("header_timeout_sec", 15.0);
  declare_parameter("serial_write_fail_count", 10);

  serial_port_              = get_parameter("serial_port").as_string();
  serial_baud_              = get_parameter("serial_baud").as_int();
  ntrip_host_               = get_parameter("ntrip_host").as_string();
  ntrip_port_               = get_parameter("ntrip_port").as_int();
  ntrip_user_               = get_parameter("ntrip_user").as_string();
  ntrip_password_           = get_parameter("ntrip_password").as_string();
  ntrip_mountpoint_         = get_parameter("ntrip_mountpoint").as_string();
  gga_interval_sec_         = get_parameter("gga_interval_sec").as_double();
  reconnect_delay_sec_      = get_parameter("reconnect_delay_sec").as_double();
  first_rtcm_timeout_sec_   = get_parameter("first_rtcm_timeout_sec").as_double();
  first_gga_timeout_sec_    = get_parameter("first_gga_timeout_sec").as_double();
  data_timeout_sec_         = get_parameter("data_timeout_sec").as_double();
  connect_timeout_sec_      = get_parameter("connect_timeout_sec").as_double();
  header_timeout_sec_       = get_parameter("header_timeout_sec").as_double();
  serial_write_fail_count_  = get_parameter("serial_write_fail_count").as_int();

  SPDLOG_INFO("========== NTRIP 客户端参数加载 ==========");
  SPDLOG_INFO("串口: {}@{}", serial_port_, serial_baud_);
  SPDLOG_INFO("caster: {}:{} mountpoint={} user={}", ntrip_host_, ntrip_port_, ntrip_mountpoint_, ntrip_user_);
  SPDLOG_INFO(
    "GGA 上传周期: {:.1f}s, NTRIP 网络重连延迟: {:.1f}s, RTCM 首帧宽限: {:.1f}s, 等首个 GGA: {:.1f}s, 数据超时: {:.1f}s",
    gga_interval_sec_,
    reconnect_delay_sec_,
    first_rtcm_timeout_sec_,
    first_gga_timeout_sec_,
    data_timeout_sec_);
  SPDLOG_INFO("连接超时: {:.1f}s, 响应头超时: {:.1f}s", connect_timeout_sec_, header_timeout_sec_);
  SPDLOG_INFO("串口写 RTCM 失败容忍次数: {}", serial_write_fail_count_);
  SPDLOG_INFO("日志级别: {}, 目录: {}, 文件上限: {} MB, 文件数量: {}, 控制台: {}", log_level_, log_dir_, log_file_size_mb_, log_file_count_, log_enable_console_ ? "是" : "否");
  SPDLOG_INFO("==========================================");
}

// 构建请求
std::string NtripClient::buildRequest() const {
  const std::string credentials = ntrip_user_ + ":" + ntrip_password_;
  const std::string encoded     = base64Encode(credentials);
  std::ostringstream request;
  request << "GET /" << ntrip_mountpoint_ << " HTTP/1.1\r\n";
  request << "Host: " << ntrip_host_ << "\r\n";
  request << "Ntrip-Version: Ntrip/2.0\r\n";
  request << "User-Agent: NTRIP gnss_localization/1.0\r\n";
  request << "Authorization: Basic " << encoded << "\r\n";
  request << "\r\n";
  return request.str();
}

// GGA 校验: 语句类型必须为 $xxGGA、异或校验和正确, 且定位状态 (Fix Quality) 有效。
bool NtripClient::checkGga(const std::string& line) const {
  if (line.size() < 6 || line[0] != '$' || line.compare(3, 3, "GGA") != 0) {
    return false;
  }
  const size_t star = line.find('*');
  if (star == std::string::npos || star + 3 > line.size()) {
    return false;
  }
  unsigned char cs = 0;
  for (size_t i = 1; i < star; ++i) {
    cs ^= static_cast<unsigned char>(line[i]);
  }
  char hex[3]                    = {line[star + 1], line[star + 2], 0};
  char* end                      = nullptr;
  const unsigned long parsed_val = std::strtoul(hex, &end, 16);
  if (end != hex + 2) {
    return false;   // 校验和字段不是合法的 2 位十六进制
  }
  const unsigned int expected = static_cast<unsigned int>(parsed_val);
  if (cs != expected) {
    return false;
  }

  // 校验定位状态 (Fix Quality): 过滤掉无效定位 (0) 的 GGA,
  // 避免把接收机尚未定位的位置上传给 NTRIP caster, 导致选错差分源。
  // 字段序 (逗号分隔): 1=类型,2=时间,3=纬度,4=纬度半球,5=经度,6=经度半球,7=FixQuality,8=卫星数,...
  // 故 Fix Quality 是第 7 个字段 = 第 6 个逗号之后。
  size_t comma_count = 0;
  size_t field_start = 0;
  for (size_t i = 1; i < star; ++i) {
    if (line[i] == ',') {
      ++comma_count;
      if (comma_count == 6) {
        field_start = i + 1;   // 第 7 个字段 (Fix Quality) 起始位置
        break;
      }
    }
  }
  if (comma_count != 6) {
    return false;   // 字段不足, 无法判断定位状态
  }
  const size_t field_end        = line.find(',', field_start);
  const std::string fix_quality = (field_end == std::string::npos || field_end > star) ? line.substr(field_start, star - field_start) : line.substr(field_start, field_end - field_start);
  if (fix_quality.empty()) {
    return false;
  }
  const int quality = std::atoi(fix_quality.c_str());
  return quality > 0;   // 0 = 无效定位, 其余 (1/2/4/5/6...) 均可用于差分请求
}

// 打开串口 (失败不记日志, 由调用方按节奏记录)。每次都重新打开, 确保干净的重连状态。
bool NtripClient::openSerial() {
  if (!serial_->open(serial_port_, serial_baud_)) {
    return false;
  }
  SPDLOG_INFO("串口已打开: {}", serial_port_);
  return true;
}

// 非阻塞: 读一次串口缓冲区, 返回本次读到的最新合法 GGA (无则 nullopt)。
// read_lines 本身非阻塞 (一次读空当前缓冲区的完整行), 这里不 sleep, 语义纯粹。
std::optional<std::string> NtripClient::tryReadGga() {
  std::optional<std::string> result;
  serial_->read_lines([&](const std::string& line) {
    if (checkGga(line)) {
      result = line + "\r\n";   // 多次命中时后帧覆盖前帧, 正好取最新
    }
  });
  return result;
}

// 阻塞: timeout 秒内反复读取直到读到合法 GGA (超时返回 nullopt)。
// 返回内容 + 接收时刻戳, 供 runSession 初始化 GGA 超时。
std::optional<std::pair<std::string, std::chrono::steady_clock::time_point>> NtripClient::waitForGga(double timeout_seconds) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_seconds);
  while (running_ && std::chrono::steady_clock::now() < deadline) {
    if (auto gga = tryReadGga()) {
      return std::make_pair(std::move(*gga), std::chrono::steady_clock::now());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return std::nullopt;
}

// 打开 NTRIP 套接字
int NtripClient::openNtripSocket(std::string& pending_data) {
  const int connect_timeout_ms = static_cast<int>(connect_timeout_sec_ * 1000.0);

  // 非阻塞 connect + poll (复用逻辑, 被首次连接和缓存重连共用)。
  auto tryConnect = [connect_timeout_ms, hdr_timeout = header_timeout_sec_](int family, int socktype, int protocol, const sockaddr* addr, socklen_t addrlen) -> int {
    int s = ::socket(family, socktype, protocol);
    if (s < 0)
      return -1;
    const int flags = ::fcntl(s, F_GETFL, 0);
    ::fcntl(s, F_SETFL, flags | O_NONBLOCK);
    int ret = ::connect(s, addr, addrlen);
    if (ret < 0 && errno == EINPROGRESS) {
      pollfd pfd{s, POLLOUT, 0};
      ret = ::poll(&pfd, 1, connect_timeout_ms);
      if (ret > 0) {
        int so_error  = 0;
        socklen_t len = sizeof(so_error);
        ::getsockopt(s, SOL_SOCKET, SO_ERROR, &so_error, &len);
        ret = (so_error == 0) ? 0 : -1;
      } else {
        ret = -1;
      }
    }
    ::fcntl(s, F_SETFL, flags);
    if (ret == 0) {
      timeval rcv_timeout{static_cast<int>(hdr_timeout), 0};
      ::setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &rcv_timeout, sizeof(rcv_timeout));
      timeval snd_timeout{5, 0};
      ::setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &snd_timeout, sizeof(snd_timeout));
      return s;
    }
    ::close(s);
    return -1;
  };

  // ---- 1. DNS 解析 (首次阻塞解析后缓存 IP, 后续重连直接跳过 getaddrinfo)。 ----
  addrinfo* results = nullptr;
  if (cached_ip_.empty()) {
    addrinfo hints{};
    hints.ai_family        = AF_UNSPEC;
    hints.ai_socktype      = SOCK_STREAM;
    const std::string port = std::to_string(ntrip_port_);
    const int lookup       = ::getaddrinfo(ntrip_host_.c_str(), port.c_str(), &hints, &results);
    if (lookup != 0) {
      SPDLOG_WARN("NTRIP DNS 解析失败: {}", gai_strerror(lookup));
      return -1;
    }

    // 缓存首个解析到的 IP 地址, 供后续重连复用。
    for (addrinfo* cur = results; cur; cur = cur->ai_next) {
      char ip_str[INET6_ADDRSTRLEN];
      if (cur->ai_family == AF_INET) {
        ::inet_ntop(AF_INET, &reinterpret_cast<sockaddr_in*>(cur->ai_addr)->sin_addr, ip_str, sizeof(ip_str));
        cached_ip_ = ip_str;
        break;
      } else if (cur->ai_family == AF_INET6) {
        ::inet_ntop(AF_INET6, &reinterpret_cast<sockaddr_in6*>(cur->ai_addr)->sin6_addr, ip_str, sizeof(ip_str));
        cached_ip_ = ip_str;
        break;
      }
    }
    if (cached_ip_.empty()) {
      ::freeaddrinfo(results);
      SPDLOG_WARN("NTRIP DNS 解析结果无可用地址");
      return -1;
    }
    SPDLOG_INFO("DNS 解析 {} -> {}", ntrip_host_, cached_ip_);
  }

  // ---- 2. TCP 连接 ----
  int fd = -1;

  if (results) {
    // 首次连接: 遍历 addrinfo 链表。
    for (addrinfo* cur = results; cur; cur = cur->ai_next) {
      fd = tryConnect(cur->ai_family, cur->ai_socktype, cur->ai_protocol, cur->ai_addr, cur->ai_addrlen);
      if (fd >= 0)
        break;
    }
    ::freeaddrinfo(results);
  } else {
    // 后续重连: 使用缓存 IP, 彻底跳过阻塞的 getaddrinfo。
    // 优先尝试 IPv4, 若失败再尝试 IPv6。
    sockaddr_in addr4{};
    addr4.sin_family = AF_INET;
    addr4.sin_port   = htons(ntrip_port_);
    if (::inet_pton(AF_INET, cached_ip_.c_str(), &addr4.sin_addr) == 1) {
      fd = tryConnect(AF_INET, SOCK_STREAM, 0, reinterpret_cast<const sockaddr*>(&addr4), sizeof(addr4));
    }
    if (fd < 0) {
      sockaddr_in6 addr6{};
      addr6.sin6_family = AF_INET6;
      addr6.sin6_port   = htons(ntrip_port_);
      if (::inet_pton(AF_INET6, cached_ip_.c_str(), &addr6.sin6_addr) == 1) {
        fd = tryConnect(AF_INET6, SOCK_STREAM, 0, reinterpret_cast<const sockaddr*>(&addr6), sizeof(addr6));
      }
    }
    if (fd < 0) {
      // 缓存 IP 连不上时清空缓存, 下次重连重新 DNS 解析 (服务器 IP 可能已变更)。
      SPDLOG_WARN("缓存 IP {} 连接失败, 下次重连重新解析 DNS", cached_ip_);
      cached_ip_.clear();
      return -1;
    }
  }

  if (fd < 0) {
    SPDLOG_WARN("NTRIP 连接失败: {}", ntrip_host_);
    return -1;
  }

  SPDLOG_INFO("连接 NTRIP caster {}:{} ...", ntrip_host_, ntrip_port_);

  // RAII 守卫: 下面的任何失败返回都会自动关闭 fd。
  struct FdGuard {
    int fd;
    ~FdGuard() {
      if (fd >= 0)
        ::close(fd);
    }
    void release() {
      fd = -1;
    }
  } guard{fd};

  // 发送握手请求。
  const std::string request = buildRequest();
  if (!sendAll(fd, request.data(), request.size())) {
    return -1;
  }

  // 读响应头: 累计超时上限 (防服务器不发头导致死循环) + 大小上限 (防内存膨胀)。
  constexpr size_t kMaxHeaderSize = 8192;
  const auto header_deadline      = std::chrono::steady_clock::now() + std::chrono::duration<double>(header_timeout_sec_);
  std::string response;
  char buffer[256];
  while (response.find("\r\n\r\n") == std::string::npos) {
    if (std::chrono::steady_clock::now() >= header_deadline) {
      SPDLOG_WARN("NTRIP 响应头超时");
      return -1;
    }
    const ssize_t count = ::recv(fd, buffer, sizeof(buffer), 0);
    if (count < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
        continue;   // 信号打断 / 接收超时, 继续等待 (受 deadline 约束)
      SPDLOG_WARN("NTRIP 响应头接收失败: {}", std::strerror(errno));
      return -1;
    }
    if (count == 0) {
      SPDLOG_WARN("NTRIP 服务器关闭连接 (无响应头)");
      return -1;
    }
    response.append(buffer, static_cast<size_t>(count));
    if (response.size() > kMaxHeaderSize) {
      SPDLOG_WARN("NTRIP 响应头过大");
      return -1;
    }
  }

  // 检查 HTTP 响应状态码: 非 200 说明认证失败/挂载点不存在等配置错误。
  // NTRIP caster 常见 "ICY 200 OK" (非标准), 也可能是 "HTTP/1.1 200 OK"。
  const size_t header_end  = response.find("\r\n\r\n");
  const std::string header = response.substr(0, header_end);
  if (header.find(" 200 ") == std::string::npos && header.find(" 200\r") == std::string::npos) {
    const size_t first_line_end  = header.find("\r\n");
    const std::string first_line = first_line_end == std::string::npos ? header : header.substr(0, first_line_end);
    SPDLOG_WARN("NTRIP 服务器拒绝: {}", first_line);
    return -1;
  }

  pending_data = response.substr(header_end + 4);
  timeval data_timeout{1, 0};
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &data_timeout, sizeof(data_timeout));
  guard.release();   // 成功路径: 交还 fd 所有权给调用方
  return fd;
}

// 发送 GPGGA 并写入 RTCM。
// 返回值区分结束原因: 数据层超时 (链路通但数据停摆) 需在重建期间保持故障码,
// 连接断开/收发失败则交由重建阶段重新暴露链路故障。
NtripClient::SessionEnd NtripClient::runSession(int socket_fd, const std::string& first_gga, std::chrono::steady_clock::time_point first_gga_time, std::string pending_data) {
  // 重置连续失败计数
  consecutive_write_fail_.store(0);

  // 循环前立即发送首个 GGA 作为定位请求, 不延迟到 recv 之后。
  if (!sendAll(socket_fd, first_gga.data(), first_gga.size())) {
    return SessionEnd::kDisconnected;
  }
  SPDLOG_INFO("已发送首个 GGA");

  std::string latest_gga  = first_gga;
  size_t total            = 0;
  auto last_gga_send_time = std::chrono::steady_clock::now();
  auto last_gga_recv_time = first_gga_time;
  char buffer[4096];
  bool first_rtcm_received = false;
  auto last_rtcm_time      = std::chrono::steady_clock::now();

  while (running_) {
    // 更新 GGA: 每次循环都尝试读取最新的 GGA (非阻塞)。
    auto gga = tryReadGga();
    if (gga) {
      latest_gga = *gga;
    }

    // 接收 RTCM 并写回串口。
    ssize_t count = 0;
    if (!pending_data.empty()) {
      count = static_cast<ssize_t>(std::min<size_t>(pending_data.size(), sizeof(buffer)));
      std::memcpy(buffer, pending_data.data(), static_cast<size_t>(count));
      pending_data.erase(0, static_cast<size_t>(count));
    } else {
      // socket 为阻塞模式 + SO_RCVTIMEO (1s): 无数据时 recv 阻塞至多 1s 后返回 EAGAIN。
      count = ::recv(socket_fd, buffer, sizeof(buffer), 0);
      if (count < 0 && errno == EINTR) {
        continue;   // 信号打断: 立即重试 recv, 不当作数据超时
      } else if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        count = 0;   // 接收超时 (无数据): 正常, 走本轮末尾的超时判定
      } else if (count < 0) {
        SPDLOG_WARN("RTCM 接收失败: {}", std::strerror(errno));
        return SessionEnd::kDisconnected;
      } else if (count == 0) {
        SPDLOG_WARN("NTRIP 服务器关闭连接");
        return SessionEnd::kDisconnected;
      }
    }

    // 此处即时取时间戳 (而非循环开头快照), 保证超时/发送周期判定准确。
    const auto now = std::chrono::steady_clock::now();

    // 写入串口
    if (count > 0) {
      total += static_cast<size_t>(count);
      if (!serial_->write_raw(buffer, static_cast<size_t>(count))) {
        consecutive_write_fail_.fetch_add(1);
        SPDLOG_WARN("写 RTCM 到串口失败 ({} bytes, 连续失败 {} 次)", static_cast<long>(count), consecutive_write_fail_.load());
      } else {
        consecutive_write_fail_.store(0);
        SPDLOG_DEBUG("recv {} bytes, total {}", static_cast<long>(count), total);
        last_rtcm_time      = now;
        first_rtcm_received = true;
      }
    }

    // 收到新 GGA 时更新最后接收时间。
    if (gga) {
      last_gga_recv_time = now;
    }

    // GGA 按周期上传 (用即时时间戳)。
    if (!latest_gga.empty() && std::chrono::duration<double>(now - last_gga_send_time).count() >= gga_interval_sec_) {
      if (!sendAll(socket_fd, latest_gga.data(), latest_gga.size())) {
        return SessionEnd::kDisconnected;
      }
      last_gga_send_time = now;
    }

    // GGA 数据超时判定 (稳定后): 持续无 GGA 判异常, 避免一直上传过期位置。
    if (std::chrono::duration<double>(now - last_gga_recv_time).count() >= data_timeout_sec_) {
      SPDLOG_WARN("GGA 数据超时 ({}s 无新 GGA), 重建链路", data_timeout_sec_);
      return SessionEnd::kGgaTimeout;
    }

    // RTCM 数据超时判定: 首次用大宽限 (挂载点建立差分), 收到首帧后用小超时。
    const double rtcm_timeout = first_rtcm_received ? data_timeout_sec_ : first_rtcm_timeout_sec_;
    if (std::chrono::duration<double>(now - last_rtcm_time).count() >= rtcm_timeout) {
      SPDLOG_WARN("RTCM 数据超时 ({}s 无新 RTCM), 重建链路", rtcm_timeout);
      return SessionEnd::kRtcmTimeout;
    }

    // 串口写 RTCM 连续失败判定: 达到阈值才判故障 (与 RTCM 帧间隔解耦, 避免低频帧率误判)。
    if (consecutive_write_fail_.load() >= serial_write_fail_count_) {
      SPDLOG_ERROR("串口写 RTCM 连续失败 {} 次, 结束会话", serial_write_fail_count_);
      return SessionEnd::kDisconnected;
    }
  }
  return SessionEnd::kDisconnected;
}

// 主线程: 外层负责链路建立与重连 (串口 + NTRIP), 内层 runSession 负责数据交互。
void NtripClient::workerLoop() {
  bool serial_failing     = false;   // 处于串口连续失败序列中
  bool connect_failing    = false;   // 处于 NTRIP 连续失败序列中

  while (running_) {
    // ---- 1. 打开串口 (固定 200ms 重试节奏; reconnect_delay_sec_ 只用于 NTRIP 网络重连) ----
    if (!openSerial()) {
      if (!serial_failing) {
        serial_failing = true;
        SPDLOG_WARN("串口打开失败: {}, 以 200ms 节奏重试", serial_port_);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }
    serial_failing = false;
    SPDLOG_INFO("串口打开成功: {}", serial_port_);

    // ---- 2. 等待首个 GGA (接收机定位收敛, 宽限 first_gga_timeout_sec_)。超时上报 GGA 故障。 ----
    SPDLOG_INFO("等待串口 GGA ...");
    const auto first_gga = waitForGga(first_gga_timeout_sec_);
    if (!first_gga) {
      SPDLOG_WARN("等待首个 GGA 超时, 200ms 后重试");
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }
    SPDLOG_INFO("收到首个 GGA");

    // ---- 3. 连接 NTRIP ----
    std::string pending_data;
    int socket_fd = openNtripSocket(pending_data);
    if (socket_fd < 0) {
      if (!connect_failing) {
        connect_failing = true;
        SPDLOG_WARN("NTRIP 连接失败, 以 {}s 重试", reconnect_delay_sec_);
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_delay_sec_));
      continue;
    }
    connect_failing = false;
    SPDLOG_INFO("NTRIP 连接成功");

    // ---- 4. 数据交互 (内层)。 ----
    {
      // socket_fd 由 RAII 守卫管理, 作用域结束自动关闭。
      struct SocketGuard {
        int fd;
        ~SocketGuard() {
          if (fd >= 0)
            ::close(fd);
        }
      } guard{socket_fd};
      runSession(socket_fd, first_gga->first, first_gga->second, pending_data);
      // 会话结束后不立即关闭串口, 由下一轮循环开头统一关闭 (确保干净的重连状态)。
    }
  }

  if (serial_->is_open()) {
    serial_->close();
  }
  SPDLOG_INFO("NTRIP 客户端已停止");
}

}   // namespace gnss_localization
