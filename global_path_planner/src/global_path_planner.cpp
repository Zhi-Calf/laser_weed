#include "global_path_planner/global_path_planner.hpp"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

#include <algorithm>
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

  // poly 的第一个顶点同时作为 ENU 原点，与 gnss_driver.yaml 的 origin_* 保持一致。
  const auto& origin = raw_vertices_.front();
  geo_converter_.Reset(origin[0], origin[1], origin[2]);
  SPDLOG_INFO("ENU 原点 (使用 poly 首个顶点): lat={:.8f} lon={:.8f} alt={:.3f}", origin[0], origin[1], origin[2]);
  projectVerticesToEnu();
  planGeometry();

  // 发布
  timer_                     = create_wall_timer(100ms, std::bind(&GlobalPathPlannerNode::timerCallback, this));
  path_points_pub_           = create_publisher<visualization_msgs::msg::Marker>("~/path_points", 10);
  key_points_pub_            = create_publisher<visualization_msgs::msg::Marker>("~/key_points", 10);
  boundary_points_pub_       = create_publisher<visualization_msgs::msg::Marker>("~/boundary_points", 10);
  inner_boundary_points_pub_ = create_publisher<visualization_msgs::msg::Marker>("~/inner_boundary_points", 10);
  path_pub_                  = create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(1).transient_local());
  pto_path_pub_              = create_publisher<nav_msgs::msg::Path>("~/pto_path", rclcpp::QoS(1).transient_local());

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
  declare_parameter("poly", std::vector<double>{});
  declare_parameter("ridge_width", 0.0);
  declare_parameter("field_margin", 0.0);
  declare_parameter("in_row_resolution", 0.0);

  path_resolution_           = std::max(0.0, get_parameter("path_resolution").as_double());
  miter_limit_               = std::max(1.0, get_parameter("miter_limit").as_double());
  first_row_margin_          = get_parameter("first_row_margin").as_double();

  // ---- 田块几何 (全部来自 YAML, 启动后只读) ----
  std::string reason;
  in_row_resolution_ = get_parameter("in_row_resolution").as_double();
  if (!parseInRowResolution(in_row_resolution_, reason)) {
    SPDLOG_ERROR("配置错误: {}", reason);
    throw std::runtime_error(reason);
  }

  const std::vector<double> poly = get_parameter("poly").as_double_array();
  if (poly.size() < 9 || poly.size() % 3 != 0) {
    reason = "poly 长度必须是 3 的倍数且至少 3 个顶点 (9 个数), 收到 " + std::to_string(poly.size()) + " 个数";
    SPDLOG_ERROR("配置错误: {}", reason);
    throw std::runtime_error(reason);
  }
  const uint32_t poly_count = static_cast<uint32_t>(poly.size() / 3u);
  raw_vertices_             = parsePolygonVertices(poly_count, poly, reason);
  if (raw_vertices_.empty()) {
    SPDLOG_ERROR("配置错误: {}", reason);
    throw std::runtime_error(reason);
  }

  LocalGeometryConfig geometry;
  if (!parseLocalGeometryConfig(get_parameter("ridge_width").as_double(), get_parameter("field_margin").as_double(), geometry, reason)) {
    SPDLOG_ERROR("配置错误: {}", reason);
    throw std::runtime_error(reason);
  }
  ridge_width_  = geometry.ridge_width;
  field_margin_ = geometry.field_margin;
  if (!parseFirstRowMarginForRidge(first_row_margin_, ridge_width_, reason)) {
    SPDLOG_ERROR("配置错误: {}", reason);
    throw std::runtime_error(reason);
  }
  if (field_margin_ < 0.5 * ridge_width_) {
    SPDLOG_WARN("field_margin({:.2f} m) < ridge_width/2({:.2f} m): 地头半圆掉头可能超出田块边界, 建议 field_margin >= ridge_width/2",
                field_margin_, 0.5 * ridge_width_);
  }

  // ---- 打印生效值 ----
  SPDLOG_INFO("========== 全局路径规划参数加载 ==========");
  SPDLOG_INFO("日志目录: {}, 文件上限: {} MB, 文件数量: {}, 控制台: {}, 级别: {}", log_dir_, log_file_size_mb_, log_file_count_, log_enable_console_ ? "开启" : "关闭", log_level_);
  SPDLOG_INFO("路径分辨率: {:.3f} m (/path 的插值步长; yaml 参数, 只在启动时读一次)", path_resolution_);
  SPDLOG_INFO("拐角上限 miter_limit: {:.3f}; first_row_margin: {:.3f} m", miter_limit_, first_row_margin_);
  SPDLOG_INFO("田块: 顶点 {} 个, 垄宽 {:.3f} m, 田边余量 {:.3f} m, PTO 行内分辨率 {:.3f} m",
              raw_vertices_.size(), ridge_width_, field_margin_, in_row_resolution_);
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

  // boundary line (from YAML) for rviz
  const bool show_local_boundary = cell_.size() >= 3;
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
  // 类型逐点取自 waypoint_types_。注意 z 不再是高度 ——
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

// 本机弓形规划健康判定 (定时器每周期调用一次)。
void GlobalPathPlannerNode::updateLocalPathHealth() {
  std::vector<uint32_t> active_faults;
  std::string detail;

  // 路径只算一次然后一直重发: 路点为空 = 规划失败 (进程还活着, 但下游永远等不到能跟的东西)。
  if (waypoints_.empty()) {
    const bool has_failure_status = (last_plan_status_ != PlanStatus::kOk);
    active_faults.push_back(has_failure_status ? planStatusFaultCode(last_plan_status_) : static_cast<uint32_t>(FAULT_PLAN_FAILED));
    detail =
      has_failure_status ? ("路径规划失败[" + std::string(planStatusName(last_plan_status_)) + "]: " + last_plan_error_) : "路径规划失败: 无路点 (ENU 原点已就绪, 检查 YAML 中的 poly / ridge_width / field_margin)";
  }

  for (uint32_t code : active_faults) {
    logFaultOnce(code, detail);
  }
  pruneFaultLog(active_faults);
}

// 全流程规划。
void GlobalPathPlannerNode::planGeometry() {
  BowInput input;

  // 顶点不足 3 个: 本机弓形规划不执行。
  if (cell_.size() < 3) {
    SPDLOG_WARN("YAML poly 投影后不足 3 个顶点, 本机弓形规划不执行");
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

  SPDLOG_INFO("[PLAN] 全流程规划完成 (几何 + 采样): 路径点 {} 个, 关键点 {} 个, 结构点 {} 个, 耗时 {:.1f} ms", waypoints_.size(), key_waypoints_.size(), out.skeleton.points.size(), plan_ms);
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

// 定时器回调函数
void GlobalPathPlannerNode::timerCallback() {
  updateLocalPathHealth();
  publishPath();
}

}   // namespace global_path_planner
