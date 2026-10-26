#pragma once

#include <GeographicLib/LocalCartesian.hpp>
#include <array>
#include <cmath>
#include <cstdint>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <weeder_msgs/msg/path.hpp>

#include "global_path_planner/bow.hpp"

namespace global_path_planner {

// 多边形顶点数上限
inline constexpr uint32_t kMaxPolygonVertices = 1024;

// YAML poly 的多边形顶点判据
inline std::vector<std::array<double, 3>> parsePolygonVertices(uint32_t poly_count, const std::vector<double>& poly, std::string& out_reason) {
  const auto reject = [&out_reason](const std::string& why) {
    out_reason = why;
    return std::vector<std::array<double, 3>>{};
  };
  out_reason.clear();

  if (poly_count < 3) {
    return reject("local 来源必须带多边形顶点: poly_count = " + std::to_string(poly_count) + " 不足 3 个 (地块边界至少 3 个顶点)");
  }
  if (poly_count > kMaxPolygonVertices) {
    return reject("local 来源的 poly_count = " + std::to_string(poly_count) + " 超过上限 " + std::to_string(kMaxPolygonVertices) + " 个顶点");
  }
  const size_t expected = static_cast<size_t>(poly_count) * 3u;
  if (poly.size() != expected) {
    return reject("local 来源的 poly 长度 (" + std::to_string(poly.size()) + ") 与 poly_count * 3 (" + std::to_string(expected) + ") 不符");
  }

  std::vector<std::array<double, 3>> vertices;
  vertices.reserve(poly_count);
  for (uint32_t i = 0; i < poly_count; ++i) {
    const size_t base = 3u * static_cast<size_t>(i);
    const double lat  = poly[base + 0];
    const double lon  = poly[base + 1];
    const double alt  = poly[base + 2];
    if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(alt)) {
      return reject("local 来源的顶点 " + std::to_string(i) + " 含非有限数 (lat/lon/alt = " + std::to_string(lat) + ", " + std::to_string(lon) + ", " + std::to_string(alt) + ")");
    }
    if (std::abs(lat) > 90.0 || std::abs(lon) > 180.0) {
      return reject("local 来源的顶点 " + std::to_string(i) + " 经纬度越界 (lat = " + std::to_string(lat) + " 须在 [-90, 90], lon = " + std::to_string(lon) + " 须在 [-180, 180])");
    }
    vertices.push_back({lat, lon, alt});
  }
  return vertices;
}

// local 来源的几何参数 (YAML ridge_width + field_margin)
struct LocalGeometryConfig {
  double ridge_width  = 0.0;   // 垄宽 (m): 行距; 必须 > 0
  double field_margin = 0.0;   // 田边余量 (m): 四周统一内缩; 必须 >= 0 (0 = 明确不内缩)
};

// ridge_width + field_margin 的取值判据
inline bool parseLocalGeometryConfig(double ridge_width, double field_margin, LocalGeometryConfig& out, std::string& out_reason) {
  out_reason.clear();
  if (!std::isfinite(ridge_width) || ridge_width <= 0.0) {
    out_reason = "local 来源的 ridge_width 必须是 > 0 的有限数 (单位 m), 收到 " + std::to_string(ridge_width);
    return false;
  }
  if (!std::isfinite(field_margin) || field_margin < 0.0) {
    out_reason = "local 来源的 field_margin 必须是 >= 0 的有限数 (单位 m, 0 = 明确不内缩), 收到 " + std::to_string(field_margin);
    return false;
  }
  out.ridge_width  = ridge_width;
  out.field_margin = field_margin;
  return true;
}

// first_row_margin 与 ridge_width 必须相容的判据
inline bool parseFirstRowMarginForRidge(double first_row_margin, double ridge_width, std::string& out_reason) {
  out_reason.clear();
  const double upper = 0.5 * ridge_width;
  if (!std::isfinite(first_row_margin) || first_row_margin < kMinFirstRowMargin || first_row_margin > upper) {
    out_reason = "first_row_margin (yaml 参数) = " + std::to_string(first_row_margin) + " m 与 ridge_width = " + std::to_string(ridge_width) + " m 不相容: 首行余量必须落在 [" +
                 std::to_string(kMinFirstRowMargin) + ", 垄宽/2 = " + std::to_string(upper) + "] m 内";
    return false;
  }
  return true;
}

/// 全局路径规划节点: 根据 YAML 中的地块多边形生成一条可跟踪的 /path。
///
/// `poly` 的第一个顶点同时作为 ENU 原点。节点启动后立即投影并规划，
/// 定时器 (100 ms) 只负责健康判定与重复发布。
///
/// 发布以下话题 (均为节点相对名, 完整名为 /global_path_planner/<topic>):
///   ~/path                  全局路径 (weeder_msgs/Path, transient_local; x/y + waypoint_types,
///                           0=作业 / 1=掉头), 供 path_tracker 跟踪并选择速度与转向模式
///   ~/path_points           路径点的 RViz marker
///   ~/work_points           WORK(0) 路径点的 RViz marker (绿色)
///   ~/turn_points           TURN(1) 路径点的 RViz marker (橙色)
///   ~/key_points            关键点的 RViz marker (每条垄的两个端点)
///   ~/boundary_points       地块边界点的 RViz marker
///   ~/inner_boundary_points 内缩 field_margin_ 后边界点的 RViz marker
class GlobalPathPlannerNode : public rclcpp::Node {
public:
  explicit GlobalPathPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~GlobalPathPlannerNode() override = default;

private:
  void timerCallback();
  void loadLogParameters();
  void loadParameters();
  void planGeometry();
  void projectVerticesToEnu();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr work_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr turn_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr key_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr boundary_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr inner_boundary_points_pub_;
  rclcpp::Publisher<weeder_msgs::msg::Path>::SharedPtr path_pub_;

private:
  GeographicLib::LocalCartesian geo_converter_;   // 经纬度 <-> ENU 平面 (原点 = /gnss_driver/origin)
  PlanBowShapedPath planner_;                     // 弓形规划器 (见 bow.hpp)

private:
  Polygon_2 cell_;                                    // YAML 配置的多边形顶点 (平面坐标, ENU)
  Polygon_2 inner_cell_;                              // 按 field_margin_ 内缩后的多边形 (平面坐标, ENU)
  std::vector<Point_2> waypoints_;                    // 当前生效的全局路径
  std::vector<uint8_t> waypoint_types_;               // 与 waypoints_ 1:1 的段类型 (0=作业 / 1=掉头)
  std::vector<Point_2> key_waypoints_;                // 关键点 (RViz 绿点): 每条垄的两个端点
  std::vector<std::array<double, 3>> raw_vertices_;   // YAML 配置的原始顶点 (经纬度)

private:
  std::string log_dir_     = "/opt/agri/logs";   // 日志目录 (支持 ~ 与 $VAR 展开)
  std::string log_level_   = "info";          // 日志级别 (info/warn/error)
  int log_file_size_mb_    = 50;              // 单个日志文件最大大小 (MB)
  int log_file_count_      = 10;              // 保留的日志文件个数
  bool log_enable_console_ = true;            // 是否输出到控制台

private:
  double ridge_width_      = 0.0;     // 垄宽 (m): 行距
  double path_resolution_  = 0.2;     // /path 插值分辨率 (m): YAML 参数 path_resolution
  double field_margin_     = 0.0;     // 四周统一内缩的田边余量 (m)
  double first_row_margin_ = 0.005;   // 首行/末行余量 (m, YAML 参数 first_row_margin)
};

}   // namespace global_path_planner
