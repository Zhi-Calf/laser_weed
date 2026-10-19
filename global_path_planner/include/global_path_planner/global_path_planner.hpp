#pragma once

#include <GeographicLib/LocalCartesian.hpp>
#include <array>
#include <cmath>
#include <cstdint>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <set>
#include <string>
#include <weeder_msgs/msg/work_mode.hpp>
#include <weeder_msgs/srv/set_path_source.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "global_path_planner/bow.hpp"

namespace global_path_planner {

// 路径来源
enum class PathSource : uint8_t {
  kLocal = 0,
  kFile  = 1,
};

// PTO路径使用的分辨率 (m)
inline constexpr double kMinInRowResolution = 0.01;
inline constexpr double kMaxInRowResolution = 100.0;

// 路径跟踪使用的分辨率 (m)
inline constexpr double kDefaultPathResolution = 0.2;

// 信号新鲜度判据的默认值 (/work_mode 与 /gnss_driver/origin 共用同一套语义: 超时 + 启动宽限)
inline constexpr double kDefaultWorkModeTimeout      = 3.0;
inline constexpr double kDefaultWorkModeStartupGrace = 20.0;
inline constexpr double kDefaultOriginTimeout        = 3.0;
inline constexpr double kDefaultOriginStartupGrace   = 60.0;

// local 来源的多边形顶点数上限
inline constexpr uint32_t kMaxPolygonVertices = 1024;

// 路径来源的整数表示
inline constexpr int kPathSourceLocal = static_cast<int>(PathSource::kLocal);   // 0
inline constexpr int kPathSourceFile  = static_cast<int>(PathSource::kFile);    // 1

// 路径来源的字符串表示
inline const char* pathSourceName(PathSource source) {
  return source == PathSource::kFile ? "file" : "local";
}

// 校验 path_source 取值
inline bool parsePathSource(int64_t value, PathSource& out) {
  if (value == static_cast<int64_t>(kPathSourceLocal)) {
    out = PathSource::kLocal;
    return true;
  }
  if (value == static_cast<int64_t>(kPathSourceFile)) {
    out = PathSource::kFile;
    return true;
  }
  return false;
}

// file 来源要加载的文件路径判据
inline bool isUsableFilePath(const std::string& expanded_path) {
  return !expanded_path.empty() && expanded_path.front() == '/';
}

// local 请求的多边形顶点判据
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

// in_row_resolution 的取值判据
inline bool parseInRowResolution(double value, std::string& out_reason) {
  out_reason.clear();
  if (!std::isfinite(value) || (value != 0.0 && (value < kMinInRowResolution || value > kMaxInRowResolution))) {
    out_reason = "in_row_resolution 必须是 0.0 或 [" + std::to_string(kMinInRowResolution) + ", " + std::to_string(kMaxInRowResolution) + "] m 内的有限数, 收到 " + std::to_string(value);
    return false;
  }
  return true;
}

// local 来源的几何参数 (请求字段 ridge_width + field_margin)
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

// first_row_margin 与请求里的垄宽必须相容的判据
inline bool parseFirstRowMarginForRidge(double first_row_margin, double ridge_width, std::string& out_reason) {
  out_reason.clear();
  const double upper = 0.5 * ridge_width;
  if (!std::isfinite(first_row_margin) || first_row_margin < kMinFirstRowMargin || first_row_margin > upper) {
    out_reason = "first_row_margin (yaml 参数) = " + std::to_string(first_row_margin) + " m 与请求里的 ridge_width = " + std::to_string(ridge_width) + " m 不相容: 首行余量必须落在 [" +
                 std::to_string(kMinFirstRowMargin) + ", 垄宽/2 = " + std::to_string(upper) + "] m 内";
    return false;
  }
  return true;
}

// file 模式**一次加载**的中间产物 (值语义): 文件经纬度 -> ENU 骨架 -> 按分辨率采样,
struct FilePathResult {
  BowSkeleton skeleton;                  // 文件点投影出的 ENU 骨架 (粗点 + 逐点/逐段类型)
  std::vector<Point_2> waypoints;        // 按 path_resolution 采出的 /path 点
  std::vector<uint8_t> waypoint_types;   // 与 waypoints 1:1 的段类型 (发布时写进 pose.position.z)
  std::vector<Point_2> pto_waypoints;    // 行内按 in_row_resolution 采出的 PTO 点
  std::vector<uint8_t> pto_seg_types;    // 与 pto_waypoints 1:1 的段类型 (同上, 也写进 z)
  std::vector<Point_2> key_waypoints;    // 关键点 = 投影后的文件顶点 (= skeleton.points, RViz 绿点)
};

// file 模式**上次加载时的输入快照** (值语义): fileInputsChanged() 拿它与**当前**输入比, 决定要不要重读文件。
struct FileInputs {
  std::string path;                   // 上次加载用的文件路径 (ExpandPath 之后的绝对路径)
  bool with_origin         = false;   // 上次加载时是否已有 ENU 原点 (false = 原点还没到, 到了必须重读)
  double origin_lat        = 0.0;     // 上次投影用的原点 (配上面那个布尔量判"原点变了")
  double origin_lon        = 0.0;     // 上次投影用的原点
  double in_row_resolution = -1.0;    // 上次加载时的 PTO 行内分辨率
  bool operator!=(const FileInputs& o) const {
    if (path != o.path || in_row_resolution != o.in_row_resolution) {
      return true;
    }
    return with_origin && (!o.with_origin || origin_lat != o.origin_lat || origin_lon != o.origin_lon);
  }
};

/// 全局路径规划节点: 把"地块多边形"(local) 或"固定路径文件"(file) 变成一条可跟踪的
/// /path (以及可选的 PTO 行内路径)。
///
/// 两种路径来源 (由 ~/set_source 服务切换, 见 PathSource):
///   local  请求带多边形顶点 + 垄宽, 本机弓形规划 (见 bow.hpp)
///   file   请求带 JSON 文件路径, 读文件投影到 ENU 后按分辨率采样 (不本机规划)
/// 定时器 (100 ms) 每拍跑一遍检查链: ① ENU 原点 -> ② /work_mode -> ③ 消费来源请求 ->
/// ④ 路径来源 -> ⑤ 计算/加载 -> ⑥/⑦ 健康判定 + 发布 (链路细节见 README)。
///
/// 发布以下话题 (均为节点相对名, 完整名为 /global_path_planner/<topic>):
///   ~/path                  全局路径 (nav_msgs/Path, transient_local; pose.position.z = 段类型
///                           0=作业 / 1=掉头), 供 path_controller 跟踪
///   ~/pto_path              PTO 行内路径 (与 ~/path 同结构; in_row_resolution = 0 时为空)
///   ~/path_points           路径点的 RViz marker
///   ~/key_points            关键点的 RViz marker (local = 每条垄的两个端点; file = 文件顶点)
///   ~/boundary_points       地块边界点的 RViz marker
///   ~/inner_boundary_points 内缩 field_margin_ 后边界点的 RViz marker
///
/// 订阅:
///   /gnss_driver/origin  ENU 原点 (transient_local + 1 Hz 重发; 与 gnss_driver 同原点)
///   /work_mode           工作模式 (只有允许变更的模式才受理 set_source)
/// 服务:
///   ~/set_source         设置路径来源与几何参数 (唯一入口; 受理后由下一拍的 ③ 应用)
class GlobalPathPlannerNode : public rclcpp::Node {
public:
  explicit GlobalPathPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~GlobalPathPlannerNode() override = default;

private:
  // 主循环与启动参数
  void timerCallback();   // 100 ms 主循环: 每拍跑一遍检查链 ①~⑦ (见类注释)
  void loadLogParameters();
  void loadParameters();

private:
  // local 来源: 顶点投影 -> 弓形规划 -> 按分辨率采样 (输入没变时短路, 见两个 *Match())
  void planGeometry();
  void resampleOnly();   // 只重采样: 复用 planned_skeleton_, 不重跑几何规划
  bool geometryInputsMatch() const;
  bool inRowResolutionMatches() const;
  bool haveUsableSkeleton() const;
  void projectVerticesToEnu();   // 顶点经纬度 -> ENU (cell_ / inner_cell_); 换原点或换请求时调
  void clearActivePath();        // 清空当前生效的 /path (状态重置; 发布留给 publishPath())

private:
  // file 来源: 一次加载 (读文件 -> 投影 -> 采样); 输入快照变了才重读 (见 fileInputsChanged())
  void loadPathFile(const std::string& trigger);
  FilePathResult projectFileWaypoints(const std::vector<std::array<double, 3>>& vertices, const std::vector<uint8_t>& turn_flags) const;
  void activateFilePath(const FilePathResult& result);
  bool fileInputsChanged() const;
  enum class FilePathState { kReady, kInvalid };   // file 模式的加载状态 (⑥ 据此报不报 10402)
  void updateFilePathHealth();                     // ⑥ 心跳/故障判定 (file 来源)

private:
  // ③ 应用请求后 / ENU 原点换值后都走这里: 决定重算 (local) 还是重读文件 (file)
  void updatePathFromInputs(const std::string& trigger);
  std::string pendingInputChangeText() const;

private:
  // 唯一发布出口与健康判定 (⑦ local)
  void publishPath();                                              // 唯一发布出口: timerCallback 的 ①/②/④ (提前 return) 与 ⑥/⑦ (正常出口); 规划/加载函数一律不调它
  void updateLocalPathHealth();                                    // ⑦ 心跳/故障判定 (local 来源)
  void logFaultOnce(uint32_t code, const std::string& detail);     // 故障码去重 WARN (恢复后重新武装)
  void pruneFaultLog(const std::vector<uint32_t>& active_codes);   // 清掉已恢复的故障码

private:
  // /gnss_driver/origin: 原点新鲜度 (10410x; 状态机 + 启动宽限)
  void originLlhCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void injectDebugPoly0Origin(const std::array<double, 3>& vertex);   // 仅调试: 拿第一个顶点当原点
  enum class OriginState { kReady, kWarmingUp, kTimedOut };
  OriginState GetOriginState(const rclcpp::Time& now) const;
  void updateOriginHealth(const rclcpp::Time& now, OriginState state);

private:
  // /work_mode: 新鲜度 + 变更门 (10411x); set_source 受不受理由 changeAllowedByMode() 定
  void workModeCallback(const weeder_msgs::msg::WorkMode::SharedPtr msg);
  bool changeAllowedByMode() const;
  std::string changeRejectReason() const;
  bool workModeStartupGraceActive(const rclcpp::Time& now) const;
  enum class WorkModeState { kReady, kWarmingUp, kTimedOut };
  WorkModeState GetWorkModeState(const rclcpp::Time& now) const;
  void updateWorkModeHealth(const rclcpp::Time& now, WorkModeState state);

private:
  // 服务: 只做校验 + 存 pending_*, 真正应用留给下一拍的 ③
  void onSetPathSource(const std::shared_ptr<weeder_msgs::srv::SetPathSource::Request> request, std::shared_ptr<weeder_msgs::srv::SetPathSource::Response> response);

private:
  // 定时器与 IO (话题/服务名与 QoS 见类注释)
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr key_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr boundary_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr inner_boundary_points_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pto_path_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr origin_sub_;
  rclcpp::Subscription<weeder_msgs::msg::WorkMode>::SharedPtr work_mode_sub_;
  rclcpp::Service<weeder_msgs::srv::SetPathSource>::SharedPtr set_source_srv_;

private:
  // 算法对象 (随节点长生命周期)
  GeographicLib::LocalCartesian geo_converter_;   // 经纬度 <-> ENU 平面 (原点 = /gnss_driver/origin)
  PlanBowShapedPath planner_;                     // local 来源的弓形规划器 (见 bow.hpp; 结果落到 planned_skeleton_)

private:
  // 当前生效路径 + local 规划的输入/输出 (发布前都攒在这里)
  Polygon_2 cell_;                                             // 当前请求的多边形顶点 (平面坐标, ENU)
  Polygon_2 inner_cell_;                                       // 按 field_margin_ 内缩后的多边形 (平面坐标, ENU)
  std::vector<Point_2> waypoints_;                             // 当前生效的全局路径 (local 规划结果或 file 文件路径)
  std::vector<uint8_t> waypoint_types_;                        // 与 waypoints_ 1:1 的段类型 (0=作业 / 1=掉头, 写进 pose.position.z)
  std::vector<Point_2> key_waypoints_;                         // 关键点 (RViz 绿点): local = 每条垄的两个端点; file = 文件顶点
  std::string last_plan_error_;                                // 最近一次规划失败的原因 (进 ⑦ 的心跳 detail; 规划成功即清空)
  PlanStatus last_plan_status_ = PlanStatus::kOk;              // 最近一次规划的状态 (心跳据此报 1040x 细分码; kOk = 成功)
  Polygon_2 planned_cell_;                                     // ---- 下面的 planned_* / has_planned_ = 上次成功规划的输入快照 ----
  double planned_ridge_width_      = 0.0;                      // 垄宽 (m)
  double planned_path_resolution_  = 0.0;                      // /path 插值分辨率 (m)
  double planned_field_margin_     = 0.0;                      // 田边余量 (m)
  double planned_miter_limit_      = kDefaultMiterLimit;       // 拐角上限 (Clipper2 语义)
  double planned_first_row_margin_ = kDefaultFirstRowMargin;   // 首行/末行余量 (m)
  bool has_planned_                = false;                    // 快照是否有效 (false = 还没成功过 或 上次失败: 必须重算)
  std::vector<std::array<double, 3>> raw_vertices_;            // 最近一次 local 请求的原始顶点 (经纬度; 换原点时重投影)

private:
  // 日志参数 (启动时读一次; 名字与其它节点保持一致)
  std::string log_dir_     = "/opt/agri/logs";   // 日志目录 (支持 ~ 与 $VAR 展开)
  std::string log_level_   = "info";             // 日志级别 (info/warn/error)
  int log_file_size_mb_    = 50;                 // 单个日志文件最大大小 (MB)
  int log_file_count_      = 10;                 // 保留的日志文件个数
  bool log_enable_console_ = true;               // 是否输出到控制台

private:
  // ENU 原点 (/gnss_driver/origin): 状态 + 新鲜度判据 (10410x; 与 work_mode 那组对称)
  bool origin_set_ = false;                                       // 是否已收到过 /gnss_driver/origin
  rclcpp::Time origin_stamp_;                                     // 最近一次收到原点的时刻 (判 stale)
  double origin_timeout_s_        = kDefaultOriginTimeout;        // stale 判据 (s; <=0 = 不判超时)
  double origin_startup_grace_s_  = kDefaultOriginStartupGrace;   // 启动宽限 (s): 从未收到过时, 超过它才判故障
  bool origin_grace_warned_       = false;                        // 宽限内"暂不发路径"只打一条 WARN (与 work_mode_grace_warned_ 对称)
  bool origin_recover_pending_    = false;                        // 原点恢复后要重投影一次 (由 ③ 消费)
  bool work_mode_recover_pending_ = false;                        // mode 恢复后要重算一次 (由 ③ 消费; 与上面那个对称)
  double origin_lat_              = 0.0;                          // 当前 ENU 原点的纬度 (投影用)
  double origin_lon_              = 0.0;                          // 当前 ENU 原点的经度
  double origin_alt_              = 0.0;                          // 当前 ENU 原点的高程 (m)

private:
  // local 规划参数 (请求字段优先; yaml 参数只在启动时读一次, 见 README 参数表)
  double ridge_width_               = 0.0;                      // 垄宽 (m): 行距 (请求字段 ridge_width, local 必填)
  double path_resolution_           = kDefaultPathResolution;   // /path 插值分辨率 (m): **仍是 yaml 参数** (path_resolution), 只在启动时读一次
  double field_margin_              = 0.0;                      // 四周统一内缩的田边余量 (m; 请求字段 field_margin, local 必填)
  double miter_limit_               = kDefaultMiterLimit;       // 内缩偏置的拐角上限 (Clipper2 语义; 见 bow.hpp; yaml 参数 miter_limit)
  double first_row_margin_          = kDefaultFirstRowMargin;   // 首行/末行兜底离内缩边界的余量 (m; 见 bow.hpp): **yaml 参数 first_row_margin 的生效值** (启动时读一次, 之后恒定)
  double in_row_resolution_         = 0.0;                      // PTO 行内分辨率 (m; 请求字段 in_row_resolution, 两种来源都必填): 0.0 = 不产 PTO 路径 —— 收到第一条请求前就是 0.0
  double planned_in_row_resolution_ = 0.0;                      // 上次成功规划用的值 (与别的 planned_* 一起做输入快照)

private:
  // PTO 输出与结构路径缓存 (当前生效; resampleOnly 复用 planned_skeleton_, 不重跑几何规划)
  std::vector<Point_2> pto_waypoints_;   // PTO 行内路径 (in_row_resolution_ = 0 时为空)
  std::vector<uint8_t> pto_seg_types_;   // 与 pto_waypoints_ 1:1 的段类型 (0=作业 / 1=掉头)
  BowSkeleton planned_skeleton_;         // 上次成功规划出的结构路径 (粗点 + 逐点/逐段类型)

private:
  // 路径来源与"已受理未应用"的请求暂存 (服务写 pending_*, ③ 消费)
  PathSource path_source_    = PathSource::kLocal;        // 当前生效的来源
  bool path_source_set_      = false;                     // 是否收到过并应用过 set_source (④ 用它判"启动后还没收到")
  bool pending_apply_        = false;                     // 有未处理的来源请求 (服务受理后置位; 只剩 ③ 消费它)
  PathSource pending_source_ = PathSource::kLocal;        // 待应用的来源 (file 要读的文件在同批的 pending_path_file_)
  std::string pending_path_file_;                         // 待加载的文件路径 (请求原样; 校验与展开在服务里做)
  std::vector<std::array<double, 3>> pending_vertices_;   // 待应用的 local 顶点 (经纬度, 投影前)
  double pending_ridge_width_       = 0.0;                // 待应用的垄宽 (m)
  double pending_field_margin_      = 0.0;                // 待应用的田边余量 (m)
  double pending_in_row_resolution_ = 0.0;                // 待应用的 PTO 行内分辨率 (m)

private:
  // 工作模式 (/work_mode): 状态 + 新鲜度判据 (10411x)
  uint8_t work_mode_  = 255;                                     // 最近一次收到的 mode; 255 = 还没收到过
  bool work_mode_set_ = false;                                   // 是否收到过 /work_mode (② 用它判"要不要置恢复标记")
  rclcpp::Time work_mode_stamp_;                                 // 最近一次收到的时刻 (判 mode_stale)
  double work_mode_timeout_s_       = kDefaultWorkModeTimeout;   // <=0 = 关闭超时判定 (只看模式值)
  bool work_mode_grace_warned_      = false;                     // 宽限内"暂不发路径"只打一条 WARN (同 path_controller 的 work_mode_missing_warned_; 与 origin_grace_warned_ 对称)
  double work_mode_startup_grace_s_ = kDefaultWorkModeStartupGrace;
  rclcpp::Time node_start_stamp_;   // 节点启动时刻 (算开机时长, 判各种启动宽限)

private:
  // 变更门/调试参数、故障码去重、file 模式状态与变更标记
  bool require_manual_for_change_ = true;                     // false = 整条门短路 (单跑规划器/回归脚本用)
  std::string path_file_;                                     // file 模式当前的文件路径 (③ 应用请求时从 pending_path_file_ 换过来)
  bool debug_use_poly0_origin_ = false;                       // 调试: 拿第一个顶点当 ENU 原点 (真机必须 false; yaml 参数)
  std::set<uint32_t> logged_faults_;                          // 已打印过的故障码 (去重 / 恢复清除)
  FilePathState file_path_state_ = FilePathState::kInvalid;   // 当前 /path 是否来自文件 (只看 file 模式; 枚举见 updateFilePathHealth())
  std::string file_path_invalid_reason_;                      // **仅 kInvalid 有意义**: 那次失败的原因 (进日志与心跳 detail); 读成功即清空
  FileInputs file_loaded_;                                    // 上次加载文件时的输入快照 (fileInputsChanged() 判"要不要重读")
  bool origin_changed_pending_    = false;                    // ENU 原点换值 (originLlhCallback)
  bool work_mode_changed_pending_ = false;                    // 工作模式值变更 (workModeCallback; 首次到达也算)
};

}   // namespace global_path_planner
