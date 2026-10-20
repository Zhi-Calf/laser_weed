#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// 平面几何基本类型 (ENU 平面, 单位 m)。
// Eigen::Vector2d 由 global_path_planner/eigen_boost_adaptor.hpp 注册为 Boost.Geometry 的 2D 点,
// std::vector<Eigen::Vector2d> 注册为其环 (ring), 因此下面的类型可直接喂给
// Boost.Geometry 与 Clipper2 (后者经过一次显式坐标转换)。
using Point_2   = Eigen::Vector2d;
using Polygon_2 = std::vector<Point_2>;
using Segment_2 = std::pair<Point_2, Point_2>;

// ---------------------------------------------------------------------------
// 规划结果 (plan() 的返回状态)。
// ---------------------------------------------------------------------------
enum class PlanStatus : uint8_t {
  kOk = 0,   // 成功: waypoints 非空

  // ---- 输入数据异常的细分 (与 preparePolygon 的判定阶梯一一对应) ----
  kInputTooFew      = 1,   // 输入边界点不足 (< 3 个)
  kDedupTooFew      = 2,   // 去重后的边界点不足 (< 3 个)
  kHullTooFew       = 3,   // 凸包点不足 (< 3 个或退化成线)
  kNonConvex        = 4,   // 地块非凸 (顶点落在地块凸包内部)
  kSelfIntersecting = 5,   // 输入边界自相交 (按给定顶点顺序不是简单多边形)

  // ---- 参数配置不合理的细分 (与 insetPolygon / makeRidgeSegments 的判定对应) ----
  kBadFieldMargin = 6,   // field_margin 不合理: 内缩把可作业区域吃没 / 内缩后区域退化
  kBadRidgeWidth  = 7,   // ridge_width 不合理: <= 0 (plan() 入口) / 内缩区域里排不出垄线 (区域太窄)

  // ---- 内部兜底 ----
  kInternal = 8,   // 流程跑完仍无路点 (未知原因)
};

// 是否属于"输入数据异常"这一类 (含它的五个细分)。调用方要按类别分支时用它 —— 免得每加一个
// 细分状态都要回头改调用点, 那正是细分最容易漏判的地方。
inline bool isBadInput(PlanStatus status) {
  switch (status) {
    case PlanStatus::kInputTooFew:
    case PlanStatus::kDedupTooFew:
    case PlanStatus::kNonConvex:
    case PlanStatus::kHullTooFew:
    case PlanStatus::kSelfIntersecting:
      return true;
    default:
      return false;
  }
}

// 是否属于"参数配置不合理"这一类 (field_margin / ridge_width)。与 isBadInput() 对称:
// 两类要分开 —— 一类该去查 YAML 里的地块顶点 poly, 另一类该去查这两个几何参数。
inline bool isBadParams(PlanStatus status) {
  return status == PlanStatus::kBadFieldMargin || status == PlanStatus::kBadRidgeWidth;
}

// 状态 -> 简短标签 (日志/诊断用): 数值与标签一起给, 现场不必去翻枚举定义。
inline const char* planStatusName(PlanStatus status) {
  switch (status) {
    case PlanStatus::kOk:
      return "ok";
    case PlanStatus::kInputTooFew:
      return "input_too_few";
    case PlanStatus::kDedupTooFew:
      return "dedup_too_few";
    case PlanStatus::kHullTooFew:
      return "hull_too_few";
    case PlanStatus::kNonConvex:
      return "non_convex";
    case PlanStatus::kSelfIntersecting:
      return "self_intersecting";
    case PlanStatus::kBadFieldMargin:
      return "bad_field_margin";
    case PlanStatus::kBadRidgeWidth:
      return "bad_ridge_width";
    case PlanStatus::kInternal:
      return "internal";
  }
  return "unknown";
}

inline constexpr double kMinFirstRowMargin = 0.001;   // 首行余量的**下限** (m): 取 Clipper2 的坐标量化尺度 (≈1 mm)。
inline constexpr uint8_t kSegWork          = 0;       // 行内作业段 (垄线本身 + 沿边界边补出来的那段)
inline constexpr uint8_t kSegTurn          = 1;       // 掉头弧 (地头转弯)

// ---------------------------------------------------------------------------
// 一次规划的全部输入 / 输出 (值语义)
// ---------------------------------------------------------------------------
// 新接口的两个载体: `plan(const BowInput&) -> BowOutput` (注意非 const: 状态写进对象成员)。
//   * 输入由调用方拥有并传入, 规划过程**绝不修改**它 (内部只改自己的"生效值"副本);
//   * 输出整包返回: 路径/关键点/观测值/状态全在里面, 对象上不导出 getter。
// 因此: 同一个 `PlanBowShapedPath` 实例同一时刻只能用于一次规划 (规划状态写在成员上);
// 不同实例之间没有共享状态, "每线程一个实例"可以并行 (见 test/test_bow_parallel.cpp)。
struct BowInput {
  Polygon_2 cell;                    // 地块顶点 (凸、按边界顺序、简单多边形, ENU, 单位 m)
  double ridge_width      = 0.0;     // 垄宽 (m, 必须 > 0)
  double path_resolution  = 0.0;     // 跟踪路径 (/path) 的插值分辨率 (m)
  double field_margin     = 0.0;     // 四周统一内缩的田边余量 (m)
  double first_row_margin = 0.005;   // 首行/末行兜底离内缩边界的余量 (m)
};

// ============================================================
// 全覆盖路径规划算法的输出 (值语义)
// ============================================================
struct BowOutput {
  PlanStatus status = PlanStatus::kOk;    // kOk = 成功 (waypoints 非空)
  std::string message;                    // 失败原因 (成功时为空)
  Polygon_2 inner_polygon;                // 内缩后的垄覆盖区域 (可视化 + 参考)
  std::vector<Point_2> waypoints;         // /path: 跟踪用的细路径
  std::vector<uint8_t> waypoint_types;    // 与 waypoints 1:1 的类型 (0=WORK/1=TURN)
  std::vector<Point_2> key_waypoints;     // 每条垄的两个端点 (掉头点)
  double sweep_dir_deg           = 0.0;   // 选定的扫描方向 (deg)
  double turn_max_overrun        = 0.0;   // 掉头伸出地块边界的最大距离 (m) —— [TURN_BOUNDARY]
  bool ok() const {
    return status == PlanStatus::kOk;
  }
};

// ============================================================
// 履带机器人的全覆盖路径规划算法
// ============================================================
#ifdef GLOBAL_PATH_PLANNER_TEST_ACCESS
class BowPlannerTestAccess;   // 测试专用: 直接验证私有掉头几何
#endif

class PlanBowShapedPath {
public:
  PlanBowShapedPath()  = default;
  ~PlanBowShapedPath() = default;
  BowOutput plan(const BowInput& in);

private:
#ifdef GLOBAL_PATH_PLANNER_TEST_ACCESS
  friend class BowPlannerTestAccess;
#endif
  void generateWaypoints();
  bool preparePolygon(const Polygon_2& cell, Polygon_2& out);
  Polygon_2 offsetUniformly(const Polygon_2& poly, double margin);
  Polygon_2 insetPolygon(const Polygon_2& poly);
  Point_2 minWidthDirection(const Polygon_2& poly);
  std::vector<Segment_2> makeRidgeSegments(const Polygon_2& region, const Point_2& sweep_dir, const Point_2& offset_dir, const Segment_2& far_side_edge);
  std::vector<Point_2> flattenRidges(const std::vector<Segment_2>& ridges);
  std::vector<Point_2> smoothTurns(const std::vector<Point_2>& key_waypoints, std::vector<uint8_t>* types = nullptr);

private:
  struct EndExtensionSpec {
    const char* label;
    Segment_2 edge;
    size_t end_index;
    Point_2 search_dir;
    bool at_front;
  };
  bool tryExtendPathEnd(const EndExtensionSpec& spec, const std::vector<Point_2>& path, std::vector<Point_2>& out, std::vector<uint8_t>* out_types) const;
  std::vector<Point_2> extendEndsToRegionCorners(
    const std::vector<Point_2>& path,
    const Segment_2& near_side_edge,
    const Segment_2& far_side_edge,
    const std::vector<uint8_t>* in_types = nullptr,
    std::vector<uint8_t>* out_types      = nullptr);

private:
  struct TurnGeometry {
    Point_2 p2;
    Point_2 p3;
    Point_2 in_dir;
    Point_2 n_ab;
    double radius = 0.0;
  };
  std::optional<TurnGeometry> makeTurnGeometry(const Point_2& P1, const Point_2& P2, const Point_2& P3, const Point_2& P4) const;
  std::vector<Point_2> sampleCircularArc(const Point_2& center, double radius, double start_angle, double sweep, const Point_2& t1, const Point_2& t2) const;
  std::vector<Point_2> buildHalfCircleTurn(const TurnGeometry& geometry);
  std::vector<Point_2> buildTurn(const Point_2& P1, const Point_2& P2, const Point_2& P3, const Point_2& P4);
  double overrunOf(const Point_2& p);
  double maxOverrunOf(const std::vector<Point_2>& pts);

private:
  void fail(PlanStatus st, const std::string& msg) {
    output_.status  = st;
    output_.message = msg;
  }

private:
  Polygon_2 cell_;                    // 输入地块顶点 (plan() 落进来)
  double ridge_width_      = 0.0;     // 垄宽 (m): 跟踪路径 (/path) 按此宽度;
  double path_resolution_  = 0.0;     // 路径分辨率 (m): 跟踪路径 (/path) 按此插值;
  double field_margin_     = 0.0;     // 边界内缩距离 (m): 边界内缩后的地块 (field_) 按此宽度;
  double first_row_margin_ = 0.005;   // 首行离内缩边界的余量 (m): 首行贴得越近, 田边盲带越小;
  Polygon_2 field_;                   // 规范化后的地块 (凸、顺时针) —— 掉头"不许出界"的判据
  BowOutput output_;                  // 规划结果 (plan() 落出去)
};
