#include "global_path_planner/global_path_planner.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "global_path_planner/fault_codes.hpp"
#include "global_path_planner/logging.hpp"

using namespace std::chrono_literals;

namespace global_path_planner {

namespace {

// 构造按路径类型过滤的 RViz 点集。
// 类型数组缺失时按 WORK 处理。
visualization_msgs::msg::Marker MakePathTypeMarker(
  const std::vector<Point_2>& waypoints,
  const std::vector<uint8_t>& waypoint_types,
  uint8_t selected_type,
  const std::string& name,
  int id,
  float red,
  float green,
  float blue,
  const rclcpp::Time& stamp) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id    = "odom";
  marker.header.stamp       = stamp;
  marker.ns                 = name;
  marker.id                 = id;
  marker.pose.orientation.w = 1.0;
  marker.type               = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.color.a            = 1.0F;
  marker.color.r            = red;
  marker.color.g            = green;
  marker.color.b            = blue;
  marker.scale.x            = 0.09;
  marker.scale.y            = 0.09;
  marker.scale.z            = 0.09;

  const bool have_types = waypoint_types.size() == waypoints.size();
  for (size_t i = 0; i < waypoints.size(); ++i) {
    const uint8_t waypoint_type = have_types ? waypoint_types[i] : 0U;
    if (waypoint_type != selected_type) {
      continue;
    }
    geometry_msgs::msg::Point point;
    point.x = waypoints[i][0];
    point.y = waypoints[i][1];
    marker.points.push_back(point);
  }
  marker.action = marker.points.empty() ? visualization_msgs::msg::Marker::DELETE : visualization_msgs::msg::Marker::ADD;
  return marker;
}

}   // namespace

GlobalPathPlannerNode::GlobalPathPlannerNode(const rclcpp::NodeOptions& options) : Node("global_path_planner", options) {

  loadLogParameters();

  // 配置 spdlog 滚动文件输出
  LogConfig log_config;
  log_config.logger_name         = "global_path_planner";
  log_config.log_file_name       = "global_path_planner.log";
  log_config.log_dir             = log_dir_;
  log_config.max_file_size_bytes = static_cast<std::size_t>(log_file_size_mb_) * 1024U * 1024U;
  log_config.max_file_count      = static_cast<std::size_t>(log_file_count_);
  log_config.enable_console      = log_enable_console_;
  log_config.log_level           = log_level_;
  try {
    ConfigureLogging(log_config);
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
  work_points_pub_           = create_publisher<visualization_msgs::msg::Marker>("~/work_points", 10);
  turn_points_pub_           = create_publisher<visualization_msgs::msg::Marker>("~/turn_points", 10);
  key_points_pub_            = create_publisher<visualization_msgs::msg::Marker>("~/key_points", 10);
  boundary_points_pub_       = create_publisher<visualization_msgs::msg::Marker>("~/boundary_points", 10);
  inner_boundary_points_pub_ = create_publisher<visualization_msgs::msg::Marker>("~/inner_boundary_points", 10);
  path_pub_                  = create_publisher<weeder_msgs::msg::Path>("~/path", rclcpp::QoS(1).transient_local());
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
  declare_parameter("path_resolution", 0.2);
  declare_parameter("first_row_margin", 0.005);
  declare_parameter("poly", std::vector<double>{});
  declare_parameter("ridge_width", 0.0);
  declare_parameter("field_margin", 0.0);

  path_resolution_  = std::max(0.0, get_parameter("path_resolution").as_double());
  first_row_margin_ = get_parameter("first_row_margin").as_double();

  // ---- 田块几何 (全部来自 YAML, 启动后只读) ----
  std::string reason;
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
    SPDLOG_WARN("field_margin({:.2f} m) < ridge_width/2({:.2f} m): 机具半幅可能越过田块边界 (掉头弧本身切在内缩边界上), 建议 field_margin >= ridge_width/2", field_margin_, 0.5 * ridge_width_);
  }

  // ---- 打印生效值 ----
  SPDLOG_INFO("========== 全局路径规划参数加载 ==========");
  SPDLOG_INFO("日志目录: {}, 文件上限: {} MB, 文件数量: {}, 控制台: {}, 级别: {}", log_dir_, log_file_size_mb_, log_file_count_, log_enable_console_ ? "开启" : "关闭", log_level_);
  SPDLOG_INFO("路径分辨率: {:.3f} m (/path 的插值步长; yaml 参数, 只在启动时读一次)", path_resolution_);
  SPDLOG_INFO("first_row_margin: {:.3f} m", first_row_margin_);
  SPDLOG_INFO("田块: 顶点 {} 个, 垄宽 {:.3f} m, 田边余量 {:.3f} m", raw_vertices_.size(), ridge_width_, field_margin_);
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

// 全流程规划。
void GlobalPathPlannerNode::planGeometry() {
  BowInput input;
  input.cell             = cell_;
  input.ridge_width      = ridge_width_;
  input.path_resolution  = path_resolution_;
  input.field_margin     = field_margin_;
  input.first_row_margin = first_row_margin_;
  const BowOutput out    = planner_.plan(input);
  if (!out.ok()) {
    waypoints_.clear();
    waypoint_types_.clear();
    key_waypoints_.clear();
    inner_cell_.clear();
    const std::string reason = out.message.empty() ? "未知原因" : out.message;
    SPDLOG_ERROR("弓字形路径规划失败: {} (状态码 {}/{} 故障码 {})", reason, planStatusName(out.status), static_cast<int>(out.status), planStatusFaultCode(out.status));
    return;
  }
  waypoints_      = out.waypoints;
  waypoint_types_ = out.waypoint_types;   // 与 waypoints_ 1:1 的段类型
  key_waypoints_  = out.key_waypoints;
  inner_cell_     = out.inner_polygon;
}

// 100ms 定时器: 发布当前生效路径 (/global_path_planner/path) + RViz Markers。
void GlobalPathPlannerNode::timerCallback() {
  const rclcpp::Time stamp = get_clock()->now();

  // 供 RViz 显示的路径点
  visualization_msgs::msg::Marker path_point;
  path_point.header.frame_id    = "odom";
  path_point.header.stamp       = stamp;
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

  // 按类型分别发布点集，便于在 RViz 中独立显示和关闭。
  const visualization_msgs::msg::Marker work_points = MakePathTypeMarker(waypoints_, waypoint_types_, 0U, "work_points", 1, 0.1F, 0.9F, 0.2F, stamp);
  const visualization_msgs::msg::Marker turn_points = MakePathTypeMarker(waypoints_, waypoint_types_, 1U, "turn_points", 1, 1.0F, 0.35F, 0.0F, stamp);

  // 供 RViz 显示的关键点
  visualization_msgs::msg::Marker key_points;
  key_points.header.frame_id    = "odom";
  key_points.header.stamp       = stamp;
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

  // 供 RViz 显示的 YAML 边界线
  const bool show_local_boundary = cell_.size() >= 3;
  visualization_msgs::msg::Marker boundary_points;
  boundary_points.header.frame_id    = "odom";
  boundary_points.header.stamp       = stamp;
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
  inner_boundary.header.stamp       = stamp;
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

  // ---- /path: x/y 与段类型分开存放 ----
  weeder_msgs::msg::Path path_msg;
  path_msg.header.frame_id   = "odom";
  path_msg.header.stamp      = stamp;
  const bool have_path_types = (waypoints_.size() == waypoint_types_.size());
  path_msg.x.reserve(waypoints_.size());
  path_msg.y.reserve(waypoints_.size());
  path_msg.waypoint_types.reserve(waypoints_.size());
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    path_msg.x.push_back(waypoints_[i][0]);
    path_msg.y.push_back(waypoints_[i][1]);
    path_msg.waypoint_types.push_back(have_path_types ? waypoint_types_[i] : 0);
  }

  path_points_pub_->publish(path_point);
  work_points_pub_->publish(work_points);
  turn_points_pub_->publish(turn_points);
  key_points_pub_->publish(key_points);
  boundary_points_pub_->publish(boundary_points);
  path_pub_->publish(path_msg);
}

}   // namespace global_path_planner
