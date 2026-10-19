#include "global_path_planner/global_path_planner.hpp"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include "file_path_parser.hpp"
#include "global_path_planner/fault_codes.hpp"

using namespace std::chrono_literals;

namespace {

// 扩展路径中的环境变量
std::string ExpandPath(std::string path) {
  if (path.rfind("~/", 0) == 0) {
    const char* home = std::getenv("HOME");
    if (home)
      path = std::string(home) + path.substr(1);
  }
  std::string out;
  for (size_t i = 0; i < path.size(); ++i) {
    if (path[i] != '$') {
      out += path[i];
      continue;
    }
    size_t j = i + 1;
    while (j < path.size() && (std::isalnum(static_cast<unsigned char>(path[j])) || path[j] == '_'))
      ++j;
    const std::string name = path.substr(i + 1, j - i - 1);
    const char* val        = name.empty() ? nullptr : std::getenv(name.c_str());
    if (val)
      out += val;
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
    if (!ec && access(preferred.c_str(), W_OK) == 0)
      return preferred;
  }

  const char* ros_home = std::getenv("ROS_HOME");
  const char* home     = std::getenv("HOME");
  std::string fallback;
  if (ros_home && *ros_home)
    fallback = std::string(ros_home) + "/logs";
  else if (home && *home)
    fallback = std::string(home) + "/.ros/logs";
  else
    return "";

  std::filesystem::create_directories(fallback, ec);
  if (ec || access(fallback.c_str(), W_OK) != 0)
    return "";
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
  sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>((std::filesystem::path(dir) / "global_path_planner.log").string(), max_file_size_bytes, max_file_count));
  if (enable_console) {
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  }

  auto logger = std::make_shared<spdlog::logger>("global_path_planner", sinks.begin(), sinks.end());
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
  spdlog::flush_every(3s);
}

// 两个顶点序列是否逐点一致 (规划输入比较用: 顶点来源相同, 直接比数值即可)。
bool SameVertices(const Polygon_2& a, const Polygon_2& b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i].x() != b[i].x() || a[i].y() != b[i].y()) {
      return false;
    }
  }
  return true;
}

}   // namespace

namespace global_path_planner {

GlobalPathPlannerNode::GlobalPathPlannerNode(const rclcpp::NodeOptions& options) : Node("global_path_planner", options) {

  loadLogParameters();

  // 配置 spdlog 滚动文件输出
  try {
    ConfigureLogger(log_dir_, static_cast<size_t>(log_file_size_mb_) * 1024U * 1024U, static_cast<size_t>(log_file_count_), log_enable_console_, log_level_);
    SPDLOG_INFO("日志配置成功");
  } catch (const std::exception& e) {
    SPDLOG_WARN("文件日志配置失败，使用默认日志器: {}", e.what());
  }

  loadParameters();

  origin_stamp_     = this->now();
  node_start_stamp_ = this->now();
  work_mode_stamp_  = this->now();

  // 发布
  timer_                     = create_wall_timer(100ms, std::bind(&GlobalPathPlannerNode::timerCallback, this));
  path_points_pub_           = create_publisher<visualization_msgs::msg::Marker>("~/path_points", 10);
  key_points_pub_            = create_publisher<visualization_msgs::msg::Marker>("~/key_points", 10);
  boundary_points_pub_       = create_publisher<visualization_msgs::msg::Marker>("~/boundary_points", 10);
  inner_boundary_points_pub_ = create_publisher<visualization_msgs::msg::Marker>("~/inner_boundary_points", 10);
  path_pub_                  = create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(1).transient_local());
  pto_path_pub_              = create_publisher<nav_msgs::msg::Path>("~/pto_path", rclcpp::QoS(1).transient_local());

  // 订阅
  origin_sub_    = create_subscription<sensor_msgs::msg::NavSatFix>("/gnss_driver/origin", rclcpp::QoS(1).transient_local(), std::bind(&GlobalPathPlannerNode::originLlhCallback, this, std::placeholders::_1));
  work_mode_sub_ = create_subscription<weeder_msgs::msg::WorkMode>("/work_mode", rclcpp::QoS(1).reliable().transient_local(), std::bind(&GlobalPathPlannerNode::workModeCallback, this, std::placeholders::_1));

  // 服务
  set_source_srv_ = create_service<weeder_msgs::srv::SetPathSource>("~/set_source", std::bind(&GlobalPathPlannerNode::onSetPathSource, this, std::placeholders::_1, std::placeholders::_2));

}

void GlobalPathPlannerNode::loadLogParameters() {
  declare_parameter("log_dir", "/opt/agri/logs");
  declare_parameter("log_file_size_mb", 50);
  declare_parameter("log_file_count", 10);
  declare_parameter("log_enable_console", true);
  declare_parameter("log_level", "info");

  log_dir_            = get_parameter("log_dir").as_string();
  log_file_size_mb_   = static_cast<int>(std::max<int64_t>(1, get_parameter("log_file_size_mb").as_int()));
  log_file_count_     = static_cast<int>(std::max<int64_t>(1, get_parameter("log_file_count").as_int()));
  log_enable_console_ = get_parameter("log_enable_console").as_bool();
  log_level_          = get_parameter("log_level").as_string();
}

void GlobalPathPlannerNode::loadParameters() {

  declare_parameter("path_resolution", kDefaultPathResolution);
  declare_parameter("miter_limit", kDefaultMiterLimit);
  declare_parameter("first_row_margin", kDefaultFirstRowMargin);
  declare_parameter("origin_timeout_s", kDefaultOriginTimeout);
  declare_parameter("origin_startup_grace_s", kDefaultOriginStartupGrace);
  declare_parameter("require_manual_for_change", true);
  declare_parameter("work_mode_timeout_s", kDefaultWorkModeTimeout);
  declare_parameter("work_mode_startup_grace_s", kDefaultWorkModeStartupGrace);
  declare_parameter("debug_use_poly0_origin", false);

  path_resolution_           = std::max(0.0, get_parameter("path_resolution").as_double());
  miter_limit_               = std::max(1.0, get_parameter("miter_limit").as_double());
  debug_use_poly0_origin_    = get_parameter("debug_use_poly0_origin").as_bool();
  first_row_margin_          = get_parameter("first_row_margin").as_double();
  origin_timeout_s_          = std::max(0.0, get_parameter("origin_timeout_s").as_double());
  origin_startup_grace_s_    = std::max(0.0, get_parameter("origin_startup_grace_s").as_double());
  require_manual_for_change_ = get_parameter("require_manual_for_change").as_bool();
  work_mode_timeout_s_       = std::max(0.0, get_parameter("work_mode_timeout_s").as_double());
  work_mode_startup_grace_s_ = std::max(0.0, get_parameter("work_mode_startup_grace_s").as_double());

  // ---- 打印生效值 ----
  SPDLOG_INFO("========== 全局路径规划参数加载 ==========");
  SPDLOG_INFO("日志目录: {}, 文件上限: {} MB, 文件数量: {}, 控制台: {}, 级别: {}", log_dir_, log_file_size_mb_, log_file_count_, log_enable_console_ ? "开启" : "关闭", log_level_);
  SPDLOG_INFO("路径分辨率: {:.3f} m (/path 的插值步长; yaml 参数, 只在启动时读一次)", path_resolution_);
  SPDLOG_INFO("拐角上限 miter_limit: {:.3f}; first_row_margin: {:.3f} m", miter_limit_, first_row_margin_);
  SPDLOG_INFO("ENU 原点判定: 收到过之后 {:.1f} s 无新原点判超时 (<=0 = 关闭该判定), 开机宽限 {:.1f} s (期间未收到不判故障; <=0 = 立刻判)", origin_timeout_s_, origin_startup_grace_s_);
  if (!require_manual_for_change_) {
    SPDLOG_WARN("工作模式判定: **关闭** 无人/标定模式下也会受理, 只用于单跑规划器/回归测试");
  } else {
    SPDLOG_INFO("工作模式判定: **开启** 只在手动模式下受理; 收到过之后停更 {:.1f} s 即视为非手动; 从未收到时开机宽限 {:.1f} s **也不放行**", work_mode_timeout_s_, work_mode_startup_grace_s_);
  }
  SPDLOG_INFO("===========================================");
}

// 投影顶点: 把顶点从经纬度投影到 ENU (平面直角坐标系)。
void GlobalPathPlannerNode::projectVerticesToEnu() {
  cell_.clear();
  for (const auto& v : raw_vertices_) {
    double x = 0.0, y = 0.0, z = 0.0;
    geo_converter_.Forward(v[0], v[1], v[2], x, y, z);
    cell_.push_back(Point_2{x, y});
    SPDLOG_INFO("投影顶点: lat={:.8f}, lon={:.8f}, alt={:.2f} -> x={:.2f}, y={:.2f}, z={:.2f}", v[0], v[1], v[2], x, y, z);
  }
}

// 设置ENU原点
void GlobalPathPlannerNode::originLlhCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  // 校验经纬度合法性。
  if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) || std::abs(msg->latitude) > 90.0 || std::abs(msg->longitude) > 180.0) {
    SPDLOG_WARN("收到非法 ENU 原点经纬度, 忽略: lat={} lon={}", msg->latitude, msg->longitude);
    return;
  }

  origin_stamp_ = this->now();

  // 检查更新
  const double lat = msg->latitude;
  const double lon = msg->longitude;
  if (origin_set_ && std::abs(lat - origin_lat_) < 1e-9 && std::abs(lon - origin_lon_) < 1e-9) {
    return;
  }

  origin_lat_             = lat;
  origin_lon_             = lon;
  origin_alt_             = msg->altitude;
  origin_set_             = true;
  origin_changed_pending_ = true;
  geo_converter_.Reset(origin_lat_, origin_lon_, origin_alt_);
  projectVerticesToEnu();
  SPDLOG_INFO("收到 ENU 原点 (来自 gnss): lat={:.8f} lon={:.8f} alt={:.3f}, 已设定投影原点", origin_lat_, origin_lon_, origin_alt_);
}

// 设置工作模式 (值变更/首次到达置 work_mode_changed_pending_ + 刷时间戳; 该标记为什么**不引起重算** -> 见头文件说明)
void GlobalPathPlannerNode::workModeCallback(const weeder_msgs::msg::WorkMode::SharedPtr msg) {

  work_mode_stamp_ = this->now();

  // 检查更新
  if (work_mode_set_ && msg->mode == work_mode_) {
    return;
  }

  work_mode_                 = msg->mode;
  work_mode_set_             = true;
  work_mode_changed_pending_ = true;
  SPDLOG_INFO("收到工作模式: {}", msg->mode == weeder_msgs::msg::WorkMode::MODE_MANUAL ? "手动模式" : "非手动模式");
}

// 当前工作模式是否在宽限期内
bool GlobalPathPlannerNode::workModeStartupGraceActive(const rclcpp::Time& now) const {
  return work_mode_startup_grace_s_ > 0.0 && (now - node_start_stamp_).seconds() < work_mode_startup_grace_s_;
}

// 获取工作模式状态
GlobalPathPlannerNode::WorkModeState GlobalPathPlannerNode::GetWorkModeState(const rclcpp::Time& now) const {
  if (!require_manual_for_change_) {
    return WorkModeState::kReady;
  }
  if (work_mode_set_) {
    const double age_s = (now - work_mode_stamp_).seconds();
    return (work_mode_timeout_s_ > 0.0 && age_s > work_mode_timeout_s_) ? WorkModeState::kTimedOut : WorkModeState::kReady;
  }
  return workModeStartupGraceActive(now) ? WorkModeState::kWarmingUp : WorkModeState::kTimedOut;
}

// 当前工作模式是否允许改路径来源/分辨率
bool GlobalPathPlannerNode::changeAllowedByMode() const {
  if (!require_manual_for_change_) {
    return true;
  }
  const WorkModeState state = GetWorkModeState(this->now());
  if (state != WorkModeState::kReady) {
    return false;
  }
  return work_mode_ == weeder_msgs::msg::WorkMode::MODE_MANUAL;
}

// 现场/APP 要能一眼判断"是我没切手动, 还是模式信号断了, 还是刚上电还没收到"
std::string GlobalPathPlannerNode::changeRejectReason() const {
  const rclcpp::Time now = this->now();
  if (!work_mode_set_) {
    char buf[320];
    const double up_s = (now - node_start_stamp_).seconds();
    if (workModeStartupGraceActive(now)) {
      std::snprintf(buf, sizeof(buf), "拒绝: 开机宽限 (%.1f s) 内还没收到消息(已开机 %.1f s), 工作模式未知", work_mode_startup_grace_s_, up_s);
    } else {
      std::snprintf(buf, sizeof(buf), "拒绝: 开机宽限 (%.1f s) 已过, 仍未收到消息(已开机 %.1f s), 工作模式未知", work_mode_startup_grace_s_, up_s);
    }
    return std::string(buf);
  }
  const double age_s = (now - work_mode_stamp_).seconds();
  if (work_mode_timeout_s_ > 0.0 && age_s > work_mode_timeout_s_) {
    char buf[192];
    std::snprintf(buf, sizeof(buf), "拒绝: 工作模式已 %.1f s 未更新", age_s);
    return std::string(buf);
  }
  return std::string("拒绝: 当前工作模式非手动 (改来源/几何参数/分辨率需要手动模式)");
}

// 调试开关 debug_use_poly0_origin_ 的注入 (真机必须 false, 见头文件说明): 把一个顶点当成 ENU 原点。
void GlobalPathPlannerNode::injectDebugPoly0Origin(const std::array<double, 3>& vertex) {
  auto origin_msg       = std::make_shared<sensor_msgs::msg::NavSatFix>();
  origin_msg->latitude  = vertex[0];
  origin_msg->longitude = vertex[1];
  origin_msg->altitude  = vertex[2];
  SPDLOG_WARN("调试开关 debug_use_poly0_origin=true: 不等 GNSS, 直接用本次 local 请求的第一个顶点 (lat={:.8f}, lon={:.8f}) 作为 ENU 原点; 真机请关闭该开关", origin_msg->latitude, origin_msg->longitude);
  originLlhCallback(origin_msg);
}

// 设置路径来源 (local / file)
void GlobalPathPlannerNode::onSetPathSource(const std::shared_ptr<weeder_msgs::srv::SetPathSource::Request> request, std::shared_ptr<weeder_msgs::srv::SetPathSource::Response> response) {
  // 1) 取值校验 (只认 0 (local) / 1 (file)
  PathSource next = path_source_;
  if (!parsePathSource(static_cast<int64_t>(request->path_source), next)) {
    response->accepted = false;
    response->reason   = "path_source 只能是 " + std::to_string(kPathSourceLocal) + " (local) / " + std::to_string(kPathSourceFile) + " (file), 收到 " + std::to_string(static_cast<int>(request->path_source));
    SPDLOG_WARN("set_source 被拒: {}", response->reason);
    return;
  }
  const bool file_request = (next == PathSource::kFile);

  // 2a) file 请求必须带**能用的文件路径**
  if (file_request) {
    const std::string expanded = ExpandPath(request->path_file);
    if (!isUsableFilePath(expanded)) {
      response->accepted = false;
      response->reason   = request->path_file.empty() ? std::string("file 来源必须带 path_file (要加载的 JSON 文件绝对路径), 本次请求为空")
                                                      : "file 来源的 path_file 必须是绝对路径 (支持 ~ / $VAR 展开), 收到 \"" + request->path_file + "\" 展开为 \"" + expanded + "\"";
      SPDLOG_WARN("set_source 被拒: {}", response->reason);
      return;
    }
  }

  // 2b) local 请求必须带**能用的多边形顶点** = poly_count + poly 两个字段
  std::vector<std::array<double, 3>> vertices;
  if (!file_request) {
    std::string why;
    vertices = parsePolygonVertices(request->poly_count, request->poly, why);
    if (vertices.empty()) {
      response->accepted = false;
      response->reason   = why;
      SPDLOG_WARN("set_source 被拒: {}", response->reason);
      return;
    }
  }

  // 2c) 随请求携带的**几何参数与 PTO 行内分辨率**
  std::string config_reason;
  if (!parseInRowResolution(request->in_row_resolution, config_reason)) {
    response->accepted = false;
    response->reason   = config_reason;
    SPDLOG_WARN("set_source 被拒: {}", response->reason);
    return;
  }

  // 2d) local 请求必须带**能用的垄宽与场边距** (几何参数)
  LocalGeometryConfig geometry;
  if (!file_request && !parseLocalGeometryConfig(request->ridge_width, request->field_margin, geometry, config_reason)) {
    response->accepted = false;
    response->reason   = config_reason;
    SPDLOG_WARN("set_source 被拒: {}", response->reason);
    return;
  }

  // 2e) first_row_margin 与请求里的垄宽必须相容** (仅 local; 取值域 [kMinFirstRowMargin, 垄宽/2])
  if (!file_request && !parseFirstRowMarginForRidge(first_row_margin_, request->ridge_width, config_reason)) {
    response->accepted = false;
    response->reason   = config_reason;
    SPDLOG_WARN("set_source 被拒: {}", response->reason);
    return;
  }

  // 3) 工作模式门: 工作模式可信 (kReady) 且当前是手动模式才允许改路径来源**
  if (!changeAllowedByMode()) {
    response->accepted = false;
    response->reason   = changeRejectReason();
    SPDLOG_WARN("set_source 被拒: {}", response->reason);
    return;
  }

  // 4) 登记请求
  pending_source_ = next;
  if (file_request) {
    pending_path_file_ = request->path_file;
  } else {
    pending_vertices_     = vertices;
    pending_ridge_width_  = geometry.ridge_width;
    pending_field_margin_ = geometry.field_margin;
  }
  pending_in_row_resolution_ = request->in_row_resolution;
  pending_apply_             = true;
  response->accepted         = true;
  response->reason           = "已受理: 来源 = " + std::string(pathSourceName(next)) + "(" + std::to_string(static_cast<int>(next)) + ")";
  if (file_request) {
    response->reason += ", 文件 = " + ExpandPath(request->path_file);
  } else {
    response->reason += ", 多边形顶点 " + std::to_string(vertices.size()) + " 个 (首个 lat=" + std::to_string(vertices.front()[0]) + ", lon=" + std::to_string(vertices.front()[1]) + ")";
    response->reason += ", 垄宽 " + std::to_string(geometry.ridge_width) + " m, 田边余量 " + std::to_string(geometry.field_margin) + " m";
  }
  response->reason += ", PTO 行内分辨率 " + std::to_string(request->in_row_resolution) + " m";

  // 5) 调试开关 (真机必须 false): 不等 GNSS, 直接用这批顶点的第一个当 ENU 原点。
  if (!file_request && debug_use_poly0_origin_ && !origin_set_) {
    injectDebugPoly0Origin(vertices.front());
  }
  response->reason += " (下一个定时周期内生效: 切来源/换顶点/读文件并发布; ENU 原点未就绪时推迟到原点到达后)";
  SPDLOG_INFO("set_source 已受理: {}", response->reason);
}

// 发布当前生效路径 (/global_path_planner/path) + PTO 路径 (/global_path_planner/pto_path) + RViz Markers。
void GlobalPathPlannerNode::publishPath() {
  // path points for rviz
  visualization_msgs::msg::Marker path_point;
  path_point.header.frame_id    = "odom";
  path_point.header.stamp       = get_clock()->now();
  path_point.ns                 = "path_point";
  path_point.id                 = 1;
  path_point.pose.orientation.w = 1.0;
  path_point.type               = visualization_msgs::msg::Marker::SPHERE_LIST;
  path_point.action             = visualization_msgs::msg::Marker::ADD;
  path_point.color.a            = 1.0;
  path_point.color.r            = 1.0;
  path_point.scale.x            = 0.05;
  path_point.scale.y            = 0.05;
  path_point.scale.z            = 0.05;
  for (const auto& wp : waypoints_) {
    geometry_msgs::msg::Point p;
    p.x = wp[0];
    p.y = wp[1];
    path_point.points.emplace_back(p);
  }

  // key points for rviz
  visualization_msgs::msg::Marker key_points;
  key_points.header.frame_id    = "odom";
  key_points.header.stamp       = get_clock()->now();
  key_points.ns                 = "key_points";
  key_points.id                 = 1;
  key_points.pose.orientation.w = 1.0;
  key_points.type               = visualization_msgs::msg::Marker::POINTS;
  key_points.action             = visualization_msgs::msg::Marker::ADD;
  key_points.color.a            = 1.0;
  key_points.color.g            = 1.0;
  key_points.scale.x            = 0.1;
  key_points.scale.y            = 0.1;
  key_points.scale.z            = 0.1;
  for (const auto& kw : key_waypoints_) {
    geometry_msgs::msg::Point p;
    p.x = kw[0];
    p.y = kw[1];
    key_points.points.emplace_back(p);
  }

  // boundary line (from YAML) for rviz。边界只存在于本机弓形规划模式; file 模式下本机
  // 没有多边形, 用 DELETE 清掉 RViz 里上一帧残留的边界, 免得看着像还有边界约束。
  const bool show_local_boundary = (path_source_ == PathSource::kLocal) && cell_.size() >= 3;
  visualization_msgs::msg::Marker boundary_points;
  boundary_points.header.frame_id    = "odom";
  boundary_points.header.stamp       = get_clock()->now();
  boundary_points.ns                 = "boundary_points";
  boundary_points.id                 = 1;
  boundary_points.pose.orientation.w = 1.0;
  boundary_points.type               = visualization_msgs::msg::Marker::LINE_STRIP;
  boundary_points.action             = show_local_boundary ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
  boundary_points.color.a            = 1.0;
  boundary_points.color.b            = 1.0;
  boundary_points.scale.x            = 0.03;
  if (show_local_boundary) {
    for (const auto& cp : cell_) {
      geometry_msgs::msg::Point p;
      p.x = cp[0];
      p.y = cp[1];
      boundary_points.points.emplace_back(p);
    }
    // 闭合多边形
    geometry_msgs::msg::Point p;
    p.x = cell_[0][0];
    p.y = cell_[0][1];
    boundary_points.points.emplace_back(p);
  }

  // 收缩后的内边界，同样以 LINE_STRIP 形式通过 boundary_points_pub_ 发布
  visualization_msgs::msg::Marker inner_boundary;
  inner_boundary.header.frame_id    = "odom";
  inner_boundary.header.stamp       = get_clock()->now();
  inner_boundary.ns                 = "boundary_points";
  inner_boundary.id                 = 2;
  inner_boundary.pose.orientation.w = 1.0;
  inner_boundary.type               = visualization_msgs::msg::Marker::LINE_STRIP;
  inner_boundary.action             = (show_local_boundary && !inner_cell_.empty()) ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
  inner_boundary.color.a            = 1.0;
  inner_boundary.color.r            = 1.0;
  inner_boundary.color.b            = 1.0;
  inner_boundary.scale.x            = 0.03;
  if (inner_boundary.action == visualization_msgs::msg::Marker::ADD) {
    for (const auto& p : inner_cell_) {
      geometry_msgs::msg::Point pt;
      pt.x = p[0];
      pt.y = p[1];
      inner_boundary.points.emplace_back(pt);
    }
    // 闭合
    geometry_msgs::msg::Point pt;
    pt.x = inner_cell_[0][0];
    pt.y = inner_cell_[0][1];
    inner_boundary.points.emplace_back(pt);
  }
  inner_boundary_points_pub_->publish(inner_boundary);

  // ---- /path: 二维路径 (x, y) + **z = 段类型** (0=WORK / 1=TURN) ----
  // 类型逐点取自 waypoint_types_ (本机规划与 file 模式都会填)。注意 z 不再是高度 ——
  // 下游别拿它当地形高度用 (约定见 bow.hpp 的"段类型"说明)
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id   = "odom";
  // ⚠ 时间戳必须填: 这里曾经留空 (0 = 1970), 而同一个函数里 /pto_path 的 header.stamp 是填了的
  // (见下面 pto_msg) —— 两条 Path 自相矛盾, 录包/回放时 kpi_bag_replay 会按 path.header.stamp 排时间轴,
  // 空戳会把整条路径排到 1970 年。在线消费者都不读它 (local_path_planner 用自己的 now();
  // KPI 只用 path.header.frame_id), 所以这纯粹是"补规范", 不影响现有行为。
  // 逐点 pose.header.stamp 不填 (同一条路径共用一个时刻, 与上游一致)。
  path_msg.header.stamp       = get_clock()->now();
  const bool have_path_types = (waypoints_.size() == waypoint_types_.size());
  path_msg.poses.reserve(waypoints_.size());
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id    = "odom";
    pose.pose.position.x    = waypoints_[i][0];
    pose.pose.position.y    = waypoints_[i][1];
    pose.pose.position.z    = have_path_types ? static_cast<double>(waypoint_types_[i]) : 0.0;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.emplace_back(pose);
  }

  path_points_pub_->publish(path_point);
  key_points_pub_->publish(key_points);
  boundary_points_pub_->publish(boundary_points);
  path_pub_->publish(path_msg);

  // ---- /global_path_planner/pto_path (行内粗采样): 同样是二维路径 + **z = 段类型** ----
  // 类型就在 z 里逐点给出, 消费端不必再去另一个数组按索引对 (也就没有"长度必须相等"的约束)。
  nav_msgs::msg::Path pto_msg;
  pto_msg.header.frame_id = "odom";
  pto_msg.header.stamp    = get_clock()->now();
  const bool have_pto     = (in_row_resolution_ > 0.0) && (pto_waypoints_.size() == pto_seg_types_.size());
  if (have_pto) {
    pto_msg.poses.reserve(pto_waypoints_.size());
    for (size_t i = 0; i < pto_waypoints_.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id    = "odom";
      pose.pose.position.x    = pto_waypoints_[i][0];
      pose.pose.position.y    = pto_waypoints_[i][1];
      pose.pose.position.z    = static_cast<double>(pto_seg_types_[i]);   // 0=WORK / 1=TURN
      pose.pose.orientation.w = 1.0;                                      // PTO 点不带航向: 单位四元数 (w 默认 0 是无效四元数)
      pto_msg.poses.emplace_back(pose);
    }
  }
  pto_path_pub_->publish(pto_msg);
}

// 固定路径文件健康判定 (仅 file 模式, 定时器每周期调用一次)。
void GlobalPathPlannerNode::updateFilePathHealth() {
  std::vector<uint32_t> active_faults;
  std::string detail;

  if (file_path_state_ == FilePathState::kInvalid) {
    active_faults.push_back(static_cast<uint32_t>(FAULT_FILE_PATH_INVALID));
    detail = "固定路径文件无效 (" + file_path_invalid_reason_ + "), 已清空 /path";
    clearActivePath();
  }

  for (uint32_t code : active_faults) {
    logFaultOnce(code, detail);
  }
  pruneFaultLog(active_faults);
}

// 本机弓形规划健康判定 (仅 local 模式, 定时器每周期调用一次)。
void GlobalPathPlannerNode::updateLocalPathHealth() {
  std::vector<uint32_t> active_faults;
  std::string detail;

  // 路径只算一次然后一直重发: 路点为空 = 规划失败 (进程还活着, 但下游永远等不到能跟的东西)。
  if (waypoints_.empty()) {
    const bool has_failure_status = (last_plan_status_ != PlanStatus::kOk);
    active_faults.push_back(has_failure_status ? planStatusFaultCode(last_plan_status_) : static_cast<uint32_t>(FAULT_PLAN_FAILED));
    detail =
      has_failure_status ? ("路径规划失败[" + std::string(planStatusName(last_plan_status_)) + "]: " + last_plan_error_) : "路径规划失败: 无路点 (ENU 原点已就绪, 检查 set_source (local) 请求带来的多边形顶点)";
  }

  for (uint32_t code : active_faults) {
    logFaultOnce(code, detail);
  }
  pruneFaultLog(active_faults);
}

// 全流程规划 (local 模式)。
void GlobalPathPlannerNode::planGeometry() {
  BowInput input;

  // 固定路径文件模式下全局路径完全来自外部, 本机不再规划。
  if (path_source_ == PathSource::kFile) {
    SPDLOG_INFO("path_source=1 (file): 跳过本机弓形规划, 全局路径取自固定路径文件 {}", path_file_);
    return;
  }

  // 还没收到 ENU 原点 (local 请求带来的顶点尚未投影到平面) 或顶点不足 3 个: 等它们到位再规划。
  if (cell_.size() < 3) {
    SPDLOG_WARN("暂无可用的多边形顶点 (还没收到 ENU 原点, 或 set_source (local) 请求带来的顶点不足 3 个), 本机弓形规划暂不执行");
    return;
  }

  // 输入 (顶点 + 几何参数 + 分辨率) 与上次成功规划完全一致 => 现成的路径仍然有效, 不重算。
  // 短路里带 !waypoints_.empty(): 切换来源 / 文件失效会把路径清空 (clearActivePath), 那时
  // 即便输入没变也必须重新生成; 规划失败过 (has_planned_=false) 同样不短路 (两个 *Match() 都带它)。
  if (geometryInputsMatch() && inRowResolutionMatches() && !waypoints_.empty()) {
    SPDLOG_INFO(
      "规划输入未变化 (顶点 {} 个, 垄宽 {:.3f}, 分辨率 {:.3f}, 行内分辨率 {:.3f}, 余量 {:.3f}, 拐角上限 {:.3f}, 首行余量 {:.3f}), 沿用已有路径 ({} 个路点), 不重新计算",
      cell_.size(),
      ridge_width_,
      path_resolution_,
      in_row_resolution_,
      field_margin_,
      miter_limit_,
      first_row_margin_,
      waypoints_.size());
    return;
  }

  input.cell              = cell_;
  input.ridge_width       = ridge_width_;
  input.path_resolution   = path_resolution_;
  input.field_margin      = field_margin_;
  input.miter_limit       = miter_limit_;
  input.first_row_margin  = first_row_margin_;
  input.in_row_resolution = in_row_resolution_;

  // ---- 规划 (planner_ 的状态在对象里, 见 bow.hpp; 每次调用只读 input, 结果整包返回) ----
  // F2C 借鉴: 扫描方向固定按两级结构选向 (见 bow.hpp); 掉头 = 与两条垄相切的半圆。
  const auto t_begin   = std::chrono::steady_clock::now();
  const BowOutput out  = planner_.plan(input);
  const double plan_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_begin).count();

  // ---- 提交 + 发布 ----
  if (!out.ok()) {
    last_plan_error_  = out.message.empty() ? "未知原因" : out.message;
    last_plan_status_ = out.status;      // 心跳据此上报细分故障码 (见 fault_codes.hpp)
    has_planned_      = false;           // 失败: 下一次必须重算, 不能被输入快照拦住
    planned_skeleton_ = BowSkeleton{};   // 结构路径也作废 (别让 resampleOnly 复用一份对应失败输入的骨架)
    SPDLOG_ERROR("弓字形路径规划失败: {} (状态码 {}/{} 故障码 {})", last_plan_error_, planStatusName(out.status), static_cast<int>(out.status), planStatusFaultCode(out.status));
    return;
  }
  last_plan_error_.clear();
  last_plan_status_ = PlanStatus::kOk;   // 成功: 清掉上一次的失败状态 (心跳回到无故障码)

  waypoints_        = out.waypoints;
  waypoint_types_   = out.waypoint_types;   // 与 waypoints_ 1:1 的段类型 (发布时写进 pose.position.z)
  key_waypoints_    = out.key_waypoints;
  inner_cell_       = out.inner_polygon;
  pto_waypoints_    = out.pto_waypoints;
  pto_seg_types_    = out.pto_seg_types;
  planned_skeleton_ = out.skeleton;

  // 记下本次输入: 同一组输入再来时直接沿用上面这条路径 (见函数开头的短路)。
  planned_cell_              = input.cell;
  planned_ridge_width_       = input.ridge_width;
  planned_path_resolution_   = input.path_resolution;
  planned_field_margin_      = input.field_margin;
  planned_miter_limit_       = input.miter_limit;
  planned_first_row_margin_  = input.first_row_margin;
  planned_in_row_resolution_ = input.in_row_resolution;
  has_planned_               = true;

  SPDLOG_INFO("[PLAN] 全流程规划完成 (几何 + 采样): 路径点 {} 个, 关键点 {} 个, 结构点 {} 个, 耗时 {:.1f} ms", waypoints_.size(), key_waypoints_.size(), planned_skeleton_.points.size(), plan_ms);
  if (waypoints_.size() != waypoint_types_.size()) {
    SPDLOG_ERROR("路径与段类型长度不一致 ({} vs {}), 类型一律按 WORK 处理", waypoints_.size(), waypoint_types_.size());
    waypoint_types_.clear();
  }
  if (pto_waypoints_.size() != pto_seg_types_.size()) {
    SPDLOG_ERROR("PTO 路径与标签长度不一致 ({} vs {}), 已清空 PTO 输出", pto_waypoints_.size(), pto_seg_types_.size());
    pto_waypoints_.clear();
    pto_seg_types_.clear();
  }
  {
    const size_t rows = key_waypoints_.size() / 2;
    SPDLOG_INFO(
      "[F2C] 扫描方向 {:.1f}°: {} 条垄; 掉头 {} 次 (半圆/直线几何), 掉头最大出界 {:.2f} m, 掉头处最宽盲带 {:.2f} m",
      out.sweep_dir_deg,
      rows,
      rows > 0 ? rows - 1 : 0,
      out.turn_max_overrun,
      out.turn_max_pocket);
  }
}

// 几何输入变了 —— 重跑全流程
bool GlobalPathPlannerNode::geometryInputsMatch() const {
  return has_planned_ && SameVertices(planned_cell_, cell_) && planned_ridge_width_ == ridge_width_ && planned_field_margin_ == field_margin_ && planned_first_row_margin_ == first_row_margin_ &&
         planned_miter_limit_ == miter_limit_ && planned_path_resolution_ == path_resolution_;
}

// 是否跟踪分辨率变了 —— 只影响采样, 不变几何
bool GlobalPathPlannerNode::inRowResolutionMatches() const {
  return has_planned_ && planned_in_row_resolution_ == in_row_resolution_;
}

// 是否有可用的骨架 (几何输入匹配 + 骨架非空)
bool GlobalPathPlannerNode::haveUsableSkeleton() const {
  return geometryInputsMatch() && !planned_skeleton_.empty();
}

// 只重采样: 复用上次全流程缓存的结构路径 (planned_skeleton_), 只做插值 + 分段判型。
void GlobalPathPlannerNode::resampleOnly() {
  if (!haveUsableSkeleton()) {
    SPDLOG_INFO("in_row_resolution 变了, 但没有可复用的结构路径 (还没成功规划过 / 几何输入变了 / 上次失败): 改为全流程重算");
    planGeometry();
    return;
  }

  // 采样是纯函数且代价极小 (只做插值 + 分段判型, 实测 5~10 us; 全流程是 0.2~0.3 ms): 完全不碰几何。
  const auto t_begin = std::chrono::steady_clock::now();
  resampleBowSkeleton(planned_skeleton_, path_resolution_, in_row_resolution_, waypoints_, pto_waypoints_, pto_seg_types_, &waypoint_types_);
  const double resample_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t_begin).count();

  if (waypoints_.size() != waypoint_types_.size()) {
    SPDLOG_ERROR("路径与段类型长度不一致 ({} vs {}), 类型一律按 WORK 处理", waypoints_.size(), waypoint_types_.size());
    waypoint_types_.clear();
  }
  if (pto_waypoints_.size() != pto_seg_types_.size()) {
    SPDLOG_ERROR("PTO 路径与标签长度不一致 ({} vs {}), 已清空 PTO 输出", pto_waypoints_.size(), pto_seg_types_.size());
    pto_waypoints_.clear();
    pto_seg_types_.clear();
  }
  // 分辨率已经生效: 更新快照里那两个值 (几何部分的快照不动 —— 骨架还是同一份)。
  planned_path_resolution_   = path_resolution_;
  planned_in_row_resolution_ = in_row_resolution_;
  has_planned_               = true;
  last_plan_error_.clear();
  last_plan_status_ = PlanStatus::kOk;
  SPDLOG_INFO("[PLAN] 只重采样完成 (跳过几何): 路径点 {} 个, PTO 点 {} 个, 结构点 {} 个, 耗时 {:.1f} us", waypoints_.size(), pto_waypoints_.size(), planned_skeleton_.points.size(), resample_us);
}

// 把一次加载的结果设为当前生效路径
void GlobalPathPlannerNode::activateFilePath(const FilePathResult& result) {
  waypoints_      = result.waypoints;
  key_waypoints_  = result.key_waypoints;    // 关键点 = 投影后的文件顶点 (见 projectFileWaypoints ④)
  waypoint_types_ = result.waypoint_types;   // 采样时逐点定好的段类型 (写进 pose.position.z)
  pto_waypoints_  = result.pto_waypoints;
  pto_seg_types_  = result.pto_seg_types;
  if (waypoint_types_.size() != waypoints_.size()) {
    SPDLOG_ERROR("路径与段类型长度不一致 ({} vs {}), 类型一律按 WORK 处理", waypoints_.size(), waypoint_types_.size());
    waypoint_types_.clear();
  }
  if (pto_waypoints_.size() != pto_seg_types_.size()) {
    SPDLOG_ERROR("PTO 路径与标签长度不一致 ({} vs {}), 已清空 PTO 输出", pto_waypoints_.size(), pto_seg_types_.size());
    pto_waypoints_.clear();
    pto_seg_types_.clear();
  }
}

// 把文件里的经纬度点投影成 ENU 平面
FilePathResult GlobalPathPlannerNode::projectFileWaypoints(const std::vector<std::array<double, 3>>& vertices, const std::vector<uint8_t>& turn_flags) const {
  assert(origin_set_ && vertices.size() >= 2);   // 前置条件, 见上: debug 下钉住; 本函数没有"未就绪"返回值
  FilePathResult result;

  // ① 投影: 文件点 -> ENU 骨架顶点 (文件没给高度, 一律按 0 投)。
  result.skeleton.points.reserve(vertices.size());
  for (const auto& v : vertices) {
    double x = 0.0, y = 0.0, z = 0.0;
    geo_converter_.Forward(v[0], v[1], v[2], x, y, z);
    result.skeleton.points.emplace_back(x, y);
  }

  // ② 类型: **点类型** = 文件里的 turn_flag (这一点是否属于掉头段);
  //    **段类型** = "段 (i -> i+1) 是掉头段 ⟺ 它的**终点** i+1 带 turn_flag"。
  //    外部文件就是这么标的 (turn_flag 落在掉头段走完的那个点上, 也就是下一作业行的起点);
  //    这个口径**表达不成点类型** (掉头段与其后的作业行共用一个端点, 那个端点无论标什么都会让
  //    一侧判错), 所以走 BowSkeleton::seg_types 直接给段类型 —— 见 bow.hpp 的说明。
  const size_t raw_n = result.skeleton.points.size();
  result.skeleton.types.reserve(raw_n);
  for (size_t i = 0; i < raw_n; ++i) {
    const bool turn = (i < turn_flags.size()) && (turn_flags[i] != 0);
    result.skeleton.types.push_back(turn ? kSegTurn : kSegWork);
  }
  if (raw_n > 1) {
    result.skeleton.seg_types.reserve(raw_n - 1);
    for (size_t i = 0; i + 1 < raw_n; ++i) {
      const bool turn = (i + 1 < turn_flags.size()) && (turn_flags[i + 1] != 0);
      result.skeleton.seg_types.push_back(turn ? kSegTurn : kSegWork);
    }
  }

  // ③ 采样 (纯函数, 与 local 模式共用一份实现):
  resampleBowSkeleton(result.skeleton, path_resolution_, in_row_resolution_, result.waypoints, result.pto_waypoints, result.pto_seg_types, &result.waypoint_types);
  if (result.waypoint_types.size() != result.waypoints.size()) {
    SPDLOG_ERROR("文件路径与段类型长度不一致 ({} vs {}), 类型一律按 WORK 处理", result.waypoints.size(), result.waypoint_types.size());
    result.waypoint_types.clear();
  }

  // ④ 关键点 (RViz 绿点)
  result.key_waypoints = result.skeleton.points;
  return result;
}

// 读固定路径文件 (JSON) 并解析 + 投影 + 采样 + 生效 + 发布。
void GlobalPathPlannerNode::loadPathFile(const std::string& trigger) {
  // ① 读取文件: 路径 + 输入快照 (快照供 fileInputsChanged 判"要不要重读"; 5 个量一次写进结构)。
  const std::string file_path = ExpandPath(path_file_);
  file_loaded_                = FileInputs{file_path, origin_set_, origin_lat_, origin_lon_, in_row_resolution_};

  // ② 解析文件: 解析成功 -> 下面 ③ 投影采样后生效并发布; 失败 -> 置无效 + 清空当前路径 (下游停车)
  GeoPathFile parsed;
  std::string reason   = "path_file 为空 (set_source 请求没带要加载的 JSON 文件路径)";
  const bool parsed_ok = !file_path.empty() && ParseGeoPathFileJson(file_path, parsed, reason);
  if (!parsed_ok) {
    const bool first          = file_path_state_ != FilePathState::kInvalid;
    file_path_state_          = FilePathState::kInvalid;
    file_path_invalid_reason_ = reason;
    clearActivePath();
    if (first) {
      SPDLOG_WARN("固定路径文件读取失败 ({}): {}", file_path, reason);
    }
    return;
  }

  // 日志使用
  const bool recovered = (file_path_state_ == FilePathState::kInvalid);

  // 摘出经纬度 + 逐点 turn_flag (两者 1:1 对齐)
  std::vector<std::array<double, 3>> vertices;
  std::vector<uint8_t> turn_flags;
  vertices.reserve(parsed.points.size());
  turn_flags.reserve(parsed.points.size());
  size_t turn_count = 0;
  for (const auto& p : parsed.points) {
    vertices.push_back({p.lat, p.lon, 0.0});   // alt: JSON 里没有高度, 投影时按 0 用
    turn_flags.push_back(p.turn_flag ? 1 : 0);
    turn_count += p.turn_flag ? 1u : 0u;
  }

  // ③ 经纬度 -> ENU 平面 -> 采样。
  const FilePathResult result = projectFileWaypoints(vertices, turn_flags);
  file_path_state_            = FilePathState::kReady;
  file_path_invalid_reason_.clear();

  // ④ 生效 (只更新成员状态; 发布由 timerCallback ⑥ 在本拍统一做, 见 publishPath 的"唯一出口")
  activateFilePath(result);

  // crc32 的结论写进日志: file 模式下"文件对不对"全靠它, 现场排障第一眼就看这条。
  // 后续需要crc32检查
  const std::string crc_note = parsed.crc32_verified ? ("crc32 " + std::to_string(parsed.crc32) + " 已校验通过") : std::string("文件里没有 crc32 -> 未校验");

  // 日志
  char in_row_buf[32];
  std::snprintf(in_row_buf, sizeof(in_row_buf), "%.2f", in_row_resolution_);
  const std::string pto_note = (in_row_resolution_ > 0.0) ? (", PTO " + std::to_string(result.pto_waypoints.size()) + " 点 @ " + in_row_buf + " m") : std::string(", 不出 PTO 路径");
  SPDLOG_INFO(
    "{}: 采用固定路径文件 {} (task={}, {}): 文件 {} 点 ({} 作业 + {} 掉头) -> /path {} 点 @ {:.2f} m{}{}",
    trigger,
    file_path,
    parsed.task_name.empty() ? "-" : parsed.task_name,
    crc_note,
    vertices.size(),
    vertices.size() - turn_count,
    turn_count,
    result.waypoints.size(),
    path_resolution_,
    pto_note,
    recovered ? " (原读取失败已恢复)" : "");
}

// file 模式该不该重读文件 (输入快照比对 + 兜底: 当前没有生效路径就重读一次)
bool GlobalPathPlannerNode::fileInputsChanged() const {
  const FileInputs current{ExpandPath(path_file_), origin_set_, origin_lat_, origin_lon_, in_row_resolution_};
  const bool no_active_path = waypoints_.empty();
  return (current != file_loaded_) || no_active_path;
}

// 更新加载路径
void GlobalPathPlannerNode::updatePathFromInputs(const std::string& trigger) {
  // ---- file 模式
  if (path_source_ == PathSource::kFile) {
    if (fileInputsChanged()) {
      loadPathFile(trigger);   // 读 -> 校验 -> 投影 -> 采样 -> 生效 (只更新成员; 发布由 ⑥ 本拍统一做)
    }
    return;
  }

  // ---- local 模式
  if (!geometryInputsMatch()) {
    SPDLOG_INFO("输入变化 [{}] -> 全流程重算 (几何/原点/分辨率)", trigger);
    planGeometry();
  } else if (!inRowResolutionMatches() || waypoints_.empty()) {
    SPDLOG_INFO("输入变化 [{}] -> 仅重采样 (复用结构路径)", trigger);
    resampleOnly();
  } else {
    SPDLOG_INFO("输入变化 [{}] 未造成实际差异 -> 沿用已有路径 ({} 个路点)", trigger, waypoints_.size());
  }
}

// 待处理标记
std::string GlobalPathPlannerNode::pendingInputChangeText() const {
  std::string text;
  const auto append = [&text](const char* name) {
    if (!text.empty())
      text += " + ";
    text += name;
  };
  if (origin_changed_pending_)
    append("ENU 原点值变更");
  if (origin_recover_pending_)
    append("ENU 原点超时恢复");
  if (work_mode_changed_pending_)
    append("工作模式值变更");
  if (work_mode_recover_pending_)
    append("work_mode 信号恢复");
  return text;
}

// 故障日志去重打印
void GlobalPathPlannerNode::logFaultOnce(uint32_t code, const std::string& detail) {
  if (!logged_faults_.insert(code).second) {
    return;
  }
  SPDLOG_WARN("故障码 {}: {}", code, detail);
}

// 不在 active_codes 里的故障码都去掉
void GlobalPathPlannerNode::pruneFaultLog(const std::vector<uint32_t>& active_codes) {
  for (auto it = logged_faults_.begin(); it != logged_faults_.end();) {
    if (std::find(active_codes.begin(), active_codes.end(), *it) == active_codes.end()) {
      it = logged_faults_.erase(it);
    } else {
      ++it;
    }
  }
}

// 获取ENU原点状态
GlobalPathPlannerNode::OriginState GlobalPathPlannerNode::GetOriginState(const rclcpp::Time& now) const {
  if (debug_use_poly0_origin_ && origin_set_) {
    return OriginState::kReady;
  }
  const double age_s = (now - origin_stamp_).seconds();
  if (origin_set_) {
    return (origin_timeout_s_ > 0.0 && age_s > origin_timeout_s_) ? OriginState::kTimedOut : OriginState::kReady;
  }
  return age_s > origin_startup_grace_s_ ? OriginState::kTimedOut : OriginState::kWarmingUp;
}

// 更新NU原点健康状态
void GlobalPathPlannerNode::updateOriginHealth(const rclcpp::Time& now, OriginState state) {
  if (state != OriginState::kTimedOut) {
    logged_faults_.erase(static_cast<uint32_t>(FAULT_ORIGIN_TIMEOUT));
    return;
  }

  char buf[320];
  const double age_s  = (now - origin_stamp_).seconds();
  const uint32_t code = static_cast<uint32_t>(FAULT_ORIGIN_TIMEOUT);
  if (origin_set_) {
    std::snprintf(buf, sizeof(buf), "ENU 原点超时: %.1f s 没有收到新原点 (阈值 %.1f s; /gnss_driver/origin 应按 1 Hz 重发)", age_s, origin_timeout_s_);
  } else {
    std::snprintf(buf, sizeof(buf), "ENU 原点超时: 开机 %.1f s 仍未收到 (启动宽限 %.1f s; 等 gnss_driver 首次 RTK 固定解)", age_s, origin_startup_grace_s_);
  }
  logFaultOnce(code, buf);
  pruneFaultLog({code});
}

// 清空当前生效路径
void GlobalPathPlannerNode::clearActivePath() {
  waypoints_.clear();
  waypoint_types_.clear();
  key_waypoints_.clear();
  pto_waypoints_.clear();
  pto_seg_types_.clear();
  planned_skeleton_ = BowSkeleton{};
  has_planned_      = false;
}

// 更新工作模式健康状态
void GlobalPathPlannerNode::updateWorkModeHealth(const rclcpp::Time& now, WorkModeState state) {
  if (state != WorkModeState::kTimedOut) {
    logged_faults_.erase(static_cast<uint32_t>(FAULT_WORK_MODE_TIMEOUT));
    return;
  }

  char buf[320];
  const uint32_t code = static_cast<uint32_t>(FAULT_WORK_MODE_TIMEOUT);
  if (!work_mode_set_) {
    const double up_s = (now - node_start_stamp_).seconds();
    std::snprintf(buf, sizeof(buf), "工作模式超时: 开机 %.1f s 仍未收到 /work_mode (启动宽限 %.1f s; 等 APP 首次下发)", up_s, work_mode_startup_grace_s_);
  } else {
    const double age_s = (now - work_mode_stamp_).seconds();
    std::snprintf(buf, sizeof(buf), "工作模式超时: %.1f s 没有收到新 /work_mode (阈值 %.1f s; APP 应按 1 Hz 重发)", age_s, work_mode_timeout_s_);
  }
  logFaultOnce(code, buf);
  pruneFaultLog({code});
}

// 定时器回调函数
void GlobalPathPlannerNode::timerCallback() {

  const rclcpp::Time now = this->now();

  // ---- 检查 ①: ENU 原点
  const OriginState origin_state = GetOriginState(now);
  const bool origin_ready        = (origin_state == OriginState::kReady);
  updateOriginHealth(now, origin_state);
  if (!origin_ready) {
    if (origin_state == OriginState::kTimedOut && origin_set_) {
      origin_recover_pending_ = true;
    }
    if (origin_state == OriginState::kWarmingUp && !origin_grace_warned_) {
      origin_grace_warned_ = true;
      SPDLOG_WARN("开机宽限 ({:.1f} s) 内还没收到 ENU 原点: 本拍发布空轨迹, 心跳保持 OK", origin_startup_grace_s_);
    }
    clearActivePath();
    publishPath();
    return;
  }
  if (origin_recover_pending_) {
    SPDLOG_INFO("ENU 原点恢复 (清 10410): 本拍重新规划并按当前原点发布 /path");
  }

  // ---- 检查 ②: work_mode 信号
  const WorkModeState work_mode_state = GetWorkModeState(now);
  const bool work_mode_ready          = (work_mode_state == WorkModeState::kReady);
  updateWorkModeHealth(now, work_mode_state);
  if (!work_mode_ready) {
    if (work_mode_state == WorkModeState::kTimedOut && work_mode_set_) {
      work_mode_recover_pending_ = true;
    }
    if (work_mode_state == WorkModeState::kWarmingUp && !work_mode_grace_warned_) {
      work_mode_grace_warned_ = true;
      SPDLOG_WARN("开机宽限 ({:.1f} s) 内还没收到工作模式: 本拍发布空轨迹, 心跳保持 OK", work_mode_startup_grace_s_);
    }
    clearActivePath();
    publishPath();
    return;
  }
  if (work_mode_recover_pending_) {
    SPDLOG_INFO("work_mode 恢复 (清 10411): 本拍重新规划并按当前原点发布 /path");
  }

  // ---- 检查 ③: 消费服务的受理请求
  bool source_applied = false;
  if (pending_apply_) {
    pending_apply_               = false;
    const PathSource next_source = pending_source_;

    // file 请求更新路径文件 / local 请求更新多边形顶点
    if (next_source == PathSource::kFile) {
      path_file_ = pending_path_file_;
      file_loaded_.path.clear();
    } else {
      raw_vertices_ = pending_vertices_;
      projectVerticesToEnu();
    }

    // 参数输入
    in_row_resolution_ = pending_in_row_resolution_;
    SPDLOG_INFO("PTO 行内分辨率生效: {:.3f} m ({})", in_row_resolution_, in_row_resolution_ == 0.0 ? "不产出 pto_path" : "行内作业段按它采");
    if (next_source == PathSource::kLocal) {
      ridge_width_  = pending_ridge_width_;
      field_margin_ = pending_field_margin_;
      SPDLOG_INFO("几何参数生效: 垄宽 {:.3f} m, 田边余量 {:.3f} m, 首行余量 (yaml 参数, 已判与垄宽相容) {:.3f} m", ridge_width_, field_margin_, first_row_margin_);
    }

    // 更新来源
    if (next_source != path_source_) {
      path_source_ = next_source;
      clearActivePath();
      file_loaded_.path.clear();
      SPDLOG_INFO(
        "set_source: 来源切换为 {}({}){}",
        pathSourceName(path_source_),
        static_cast<int>(path_source_),
        path_source_ == PathSource::kFile ? ", 文件 = " + ExpandPath(path_file_) : ", 顶点 " + std::to_string(raw_vertices_.size()) + " 个");
    }
    path_source_set_ = true;
    source_applied   = true;
  }

  // ---- 检查 ④: 路径来源 (启动后还没收到 set_source): 不规划也不读文件, 只发空路径 ----
  if (!path_source_set_) {
    publishPath();
    return;
  }

  // ---- ⑤ 唯一的"算/加载"执行点
  const std::string input_change = pendingInputChangeText();
  if (source_applied || !input_change.empty()) {
    std::string trigger = source_applied ? std::string("服务 set_source") : input_change;
    if (source_applied && !input_change.empty()) {
      trigger += " + " + input_change;
    }
    origin_changed_pending_    = false;
    origin_recover_pending_    = false;
    work_mode_changed_pending_ = false;
    work_mode_recover_pending_ = false;
    updatePathFromInputs(trigger);
  }

  // ---- 检查 ⑥: 发布路径
  const bool file_mode = (path_source_ == PathSource::kFile);
  if (file_mode) {
    updateFilePathHealth();
    publishPath();
    return;
  }

  // ---- ⑦ 本机内置弓形规划 (默认) ----
  updateLocalPathHealth();
  publishPath();
}

}   // namespace global_path_planner
