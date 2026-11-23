#include "app_bridge/http_bridge_node.hpp"

#include <curl/curl.h>
#include <fcntl.h>
#include <httplib.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <system_error>
#include <utility>

#include "app_bridge/logging.hpp"
#include "app_bridge/path_file.hpp"

namespace app_bridge {

using json = nlohmann::json;

namespace {

constexpr const char* kNodeName = "http_bridge";
constexpr const char* kApiPrefix = "/api/v1";

void Reply(httplib::Response& res, int status, const json& body) {
  res.status = status;
  res.set_content(body.dump(), "application/json; charset=utf-8");
}

void ReplyError(httplib::Response& res, int status, const std::string& message) {
  Reply(res, status, {{"success", false}, {"message", message}});
}

// ---- 下载 (libcurl) ----

struct DownloadSink {
  std::string data;
  size_t max_bytes = 0;
  bool too_large   = false;
};

size_t OnCurlWrite(char* ptr, size_t size, size_t nmemb, void* userdata) {
  auto* sink     = static_cast<DownloadSink*>(userdata);
  const size_t n = size * nmemb;
  if (sink->data.size() + n > sink->max_bytes) {
    sink->too_large = true;
    return 0;   // 返回值 != n -> curl 中止传输
  }
  sink->data.append(ptr, n);
  return n;
}

bool Download(const std::string& url, double timeout_s, size_t max_bytes, std::string& out, std::string& err) {
  CURL* curl = curl_easy_init();
  if (!curl) {
    err = "curl 初始化失败";
    return false;
  }
  DownloadSink sink;
  sink.max_bytes = max_bytes;
  char errbuf[CURL_ERROR_SIZE] = {0};

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &OnCurlWrite);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &sink);
  curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, errbuf);
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl, CURLOPT_MAXREDIRS, 5L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(timeout_s * 1000.0));
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, static_cast<long>(std::min(timeout_s, 10.0) * 1000.0));
  // 多线程里必须关信号 (curl 默认用 SIGALRM 做 DNS 超时)
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  // 只允许 http/https, 重定向也一样 (不让 url 指到 file:// 之类)
#if LIBCURL_VERSION_NUM >= 0x075500
  curl_easy_setopt(curl, CURLOPT_PROTOCOLS_STR, "http,https");
  curl_easy_setopt(curl, CURLOPT_REDIR_PROTOCOLS_STR, "http,https");
#else
  curl_easy_setopt(curl, CURLOPT_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
  curl_easy_setopt(curl, CURLOPT_REDIR_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
#endif

  const CURLcode rc = curl_easy_perform(curl);
  long http_code    = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  curl_easy_cleanup(curl);

  if (sink.too_large) {
    err = "文件超过上限 " + std::to_string(max_bytes / (1024 * 1024)) + " MB";
    return false;
  }
  if (rc != CURLE_OK) {
    err = errbuf[0] ? errbuf : curl_easy_strerror(rc);
    return false;
  }
  if (http_code >= 400) {
    err = "服务器返回 HTTP " + std::to_string(http_code);
    return false;
  }
  out = std::move(sink.data);
  return true;
}

// ---- 原子落盘 ----
// 先写同目录下的临时文件并 fsync, 再 rename 覆盖: 掉电 / 写到一半出错都不会留下半个文件,
// 规划器读到的要么是旧文件、要么是完整的新文件。
bool AtomicWrite(const std::string& path, const std::string& content, std::string& err) {
  std::error_code ec;
  const std::filesystem::path parent = std::filesystem::path(path).parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent, ec);
    if (ec) {
      err = "无法创建目录 " + parent.string() + ": " + ec.message();
      return false;
    }
  }
  const std::string tmp = path + ".tmp";
  const int fd          = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0) {
    err = "无法写入 " + tmp + ": " + std::strerror(errno);
    return false;
  }
  size_t written = 0;
  while (written < content.size()) {
    const ssize_t n = ::write(fd, content.data() + written, content.size() - written);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      err = "写入 " + tmp + " 失败: " + std::strerror(errno);
      ::close(fd);
      ::unlink(tmp.c_str());
      return false;
    }
    written += static_cast<size_t>(n);
  }
  if (::fsync(fd) != 0 || ::close(fd) != 0) {
    err = "落盘 " + tmp + " 失败: " + std::strerror(errno);
    ::unlink(tmp.c_str());
    return false;
  }
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    err = "重命名到 " + path + " 失败: " + std::strerror(errno);
    ::unlink(tmp.c_str());
    return false;
  }
  return true;
}

}   // namespace

HttpBridgeNode::HttpBridgeNode(const rclcpp::NodeOptions& options) : Node(kNodeName, options) {
  loadLogParameters();
  try {
    ConfigureLogger(kNodeName, log_dir_, static_cast<size_t>(log_file_size_mb_) * 1024 * 1024, static_cast<size_t>(log_file_count_), log_enable_console_, log_level_);
  } catch (const std::exception& e) {
    SPDLOG_ERROR("日志初始化失败, 仅输出到终端: {}", e.what());
  }
  loadParameters();

  // curl_global_init 不是线程安全的: 必须在服务线程起来之前调一次。
  curl_global_init(CURL_GLOBAL_DEFAULT);

  path_sub_ = create_subscription<nav_msgs::msg::Path>(kGlobalPathTopic, rclcpp::QoS(1).reliable().transient_local(), [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(path_mutex_);
    path_ = std::move(msg);
  });
  origin_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(kGnssOriginTopic, rclcpp::QoS(1).reliable().transient_local(), [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(path_mutex_);
    origin_ = *msg;
  });

  startServer();
}

HttpBridgeNode::~HttpBridgeNode() {
  if (server_) {
    server_->stop();
  }
  if (server_thread_.joinable()) {
    server_thread_.join();
  }
  curl_global_cleanup();
}

void HttpBridgeNode::loadLogParameters() {
  declare_parameter("log_dir", "/home/asd/logs");
  declare_parameter("log_file_size_mb", 50);
  declare_parameter("log_file_count", 10);
  declare_parameter("log_enable_console", true);
  declare_parameter("log_level", "info");

  log_dir_            = get_parameter("log_dir").as_string();
  log_file_size_mb_   = static_cast<int>(get_parameter("log_file_size_mb").as_int());
  log_file_count_     = static_cast<int>(get_parameter("log_file_count").as_int());
  log_enable_console_ = get_parameter("log_enable_console").as_bool();
  log_level_          = get_parameter("log_level").as_string();
}

void HttpBridgeNode::loadParameters() {
  declare_parameter("bind_address", "0.0.0.0");
  declare_parameter("port", 8080);
  declare_parameter("tls_cert_file", "");
  declare_parameter("tls_key_file", "");
  declare_parameter("auth_token", "");
  declare_parameter("path_file", "/opt/agri/test.json");
  declare_parameter("max_body_mb", 20);
  declare_parameter("download_timeout_s", download_timeout_s_);

  bind_address_       = get_parameter("bind_address").as_string();
  port_               = static_cast<int>(get_parameter("port").as_int());
  tls_cert_file_      = ExpandPath(get_parameter("tls_cert_file").as_string());
  tls_key_file_       = ExpandPath(get_parameter("tls_key_file").as_string());
  auth_token_         = get_parameter("auth_token").as_string();
  path_file_          = ExpandPath(get_parameter("path_file").as_string());
  max_body_bytes_     = static_cast<size_t>(std::max<int64_t>(1, get_parameter("max_body_mb").as_int())) * 1024 * 1024;
  download_timeout_s_ = std::max(1.0, get_parameter("download_timeout_s").as_double());

  const bool tls = !tls_cert_file_.empty() && !tls_key_file_.empty();
  SPDLOG_INFO("HTTP 服务: {}://{}:{}{} (鉴权={})", tls ? "https" : "http", bind_address_, port_, kApiPrefix, auth_token_.empty() ? "关" : "Bearer token");
  SPDLOG_INFO("路径文件落盘位置: {} (上限 {} MB, 下载超时 {:.0f} s)", path_file_, max_body_bytes_ / (1024 * 1024), download_timeout_s_);
  if (tls_cert_file_.empty() != tls_key_file_.empty()) {
    SPDLOG_WARN("tls_cert_file 与 tls_key_file 只给了一个, 按 http 运行");
  }
}

void HttpBridgeNode::startServer() {
  const bool tls = !tls_cert_file_.empty() && !tls_key_file_.empty();
  std::string error;
  if (tls) {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
    server_ = std::make_unique<httplib::SSLServer>(tls_cert_file_.c_str(), tls_key_file_.c_str());
#else
    error = "本机 cpp-httplib 未启用 OpenSSL, 不支持 https";
#endif
  } else {
    server_ = std::make_unique<httplib::Server>();
  }
  if (error.empty() && !server_->is_valid()) {
    error = "证书 / 私钥无效: " + tls_cert_file_ + ", " + tls_key_file_;
  }
  if (error.empty()) {
    setupRoutes();
    if (!server_->bind_to_port(bind_address_, port_)) {
      error = "无法监听 " + bind_address_ + ":" + std::to_string(port_) + " (端口被占用或无权限)";
    }
  }

  if (!error.empty()) {
    // HTTP 起不来不影响车辆运动控制，只记录日志并停止该服务。
    SPDLOG_ERROR("HTTP 服务启动失败: {}", error);
    server_.reset();
    return;
  }

  server_thread_ = std::thread([this]() {
    server_->listen_after_bind();
  });
  SPDLOG_INFO("HTTP 服务已启动");
}

void HttpBridgeNode::setupRoutes() {
  server_->set_payload_max_length(max_body_bytes_);

  // httplib 默认还开 SO_REUSEPORT: 端口被另一个进程占着也能 bind 成功, 两个实例静默分摊请求,
  // 上面的 "端口被占用" 故障永远报不出来。只留 SO_REUSEADDR (够重启时复用 TIME_WAIT 端口)。
  server_->set_socket_options([](socket_t sock) {
    int yes = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const void*>(&yes), sizeof(yes));
  });

  // 鉴权放在路由之前: 未授权的请求连 body 解析都不做。
  server_->set_pre_routing_handler([this](const httplib::Request& req, httplib::Response& res) {
    return authorized(req, res) ? httplib::Server::HandlerResponse::Unhandled : httplib::Server::HandlerResponse::Handled;
  });
  server_->set_logger([](const httplib::Request& req, const httplib::Response& res) {
    SPDLOG_INFO("{} {} {} -> {}", req.remote_addr, req.method, req.path, res.status);
  });

  const std::string p = kApiPrefix;
  server_->Put(p + "/path/file", [this](const httplib::Request& req, httplib::Response& res) {
    handlePutPathFile(req, res);
  });
  server_->Post(p + "/path/download", [this](const httplib::Request& req, httplib::Response& res) {
    handlePostPathDownload(req, res);
  });
  server_->Get(p + "/path", [this](const httplib::Request& req, httplib::Response& res) {
    handleGetPath(req, res);
  });
}

bool HttpBridgeNode::authorized(const httplib::Request& req, httplib::Response& res) const {
  if (auth_token_.empty()) {
    return true;
  }
  if (req.get_header_value("Authorization") == "Bearer " + auth_token_) {
    return true;
  }
  ReplyError(res, 401, "未授权: 需要 Authorization: Bearer <token>");
  return false;
}

bool HttpBridgeNode::validateAndSave(const std::string& content, int& status, std::string& message) {
  path_file::Summary summary;
  std::string err;
  if (!path_file::Validate(content, summary, err)) {
    status  = 400;
    message = "路径文件无效: " + err;
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    if (!AtomicWrite(path_file_, content, err)) {
      status  = 500;
      message = err;
      return false;
    }
  }
  status  = 200;
  message = "已保存 " + std::to_string(summary.num_points) + " 个点到 " + path_file_ + (summary.crc32_verified ? " (crc32 已校验)" : " (未带 crc32, 未校验)");
  SPDLOG_INFO("{} (task_name={})", message, summary.task_name);
  return true;
}

void HttpBridgeNode::handlePutPathFile(const httplib::Request& req, httplib::Response& res) {
  int status = 0;
  std::string message;
  if (!validateAndSave(req.body, status, message)) {
    SPDLOG_WARN("上传路径文件失败: {}", message);
    ReplyError(res, status, message);
    return;
  }
  Reply(res, status, {{"success", true}, {"message", message}, {"size", req.body.size()}});
}

void HttpBridgeNode::handlePostPathDownload(const httplib::Request& req, httplib::Response& res) {
  const json body = json::parse(req.body, nullptr, false);
  if (body.is_discarded() || !body.is_object() || !body.contains("url") || !body.at("url").is_string()) {
    ReplyError(res, 400, "请求体应为 {\"url\": \"http(s)://...\"}");
    return;
  }
  const std::string url = body.at("url").get<std::string>();
  if (url.rfind("http://", 0) != 0 && url.rfind("https://", 0) != 0) {
    ReplyError(res, 400, "url 只支持 http:// 或 https://");
    return;
  }

  SPDLOG_INFO("开始下载路径文件: {}", url);
  std::string content;
  std::string err;
  if (!Download(url, download_timeout_s_, max_body_bytes_, content, err)) {
    SPDLOG_WARN("下载失败 {}: {}", url, err);
    ReplyError(res, 502, "下载失败: " + err);
    return;
  }

  int status = 0;
  std::string message;
  if (!validateAndSave(content, status, message)) {
    SPDLOG_WARN("下载的路径文件未保存: {}", message);
    ReplyError(res, status, message);
    return;
  }
  Reply(res, status, {{"success", true}, {"message", message}, {"size", content.size()}});
}

void HttpBridgeNode::handleGetPath(const httplib::Request& /*req*/, httplib::Response& res) {
  nav_msgs::msg::Path::ConstSharedPtr path;
  std::optional<sensor_msgs::msg::NavSatFix> origin;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    path   = path_;
    origin = origin_;
  }
  if (!path || path->poses.empty()) {
    ReplyError(res, 503, "规划器还没有发布路径 (" + std::string(kGlobalPathTopic) + ")");
    return;
  }
  if (!origin) {
    ReplyError(res, 503, "还没有 ENU 原点 (" + std::string(kGnssOriginTopic) + "), 无法换算经纬度");
    return;
  }

  // 与规划器同一个投影: GeographicLib::LocalCartesian, 原点 = /gnss_driver/origin。
  // pose.position.z 在这条路径上是段类型 (0 = 作业, 1 = 掉头), 不是高度, 反投影时按 0 处理。
  const GeographicLib::LocalCartesian proj(origin->latitude, origin->longitude, origin->altitude);
  json points = json::array();
  for (const auto& pose : path->poses) {
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
    proj.Reverse(pose.pose.position.x, pose.pose.position.y, 0.0, lat, lon, alt);
    points.push_back({{"lat", lat}, {"lon", lon}, {"turn", pose.pose.position.z > 0.5}});
  }
  Reply(res, 200, {{"success", true}, {"message", ""}, {"points", std::move(points)}});
}

}   // namespace app_bridge
