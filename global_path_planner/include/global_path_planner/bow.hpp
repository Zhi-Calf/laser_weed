#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
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
  kInputTooFew = 1,   // 输入边界点不足 (< 3 个)
  kDedupTooFew = 2,   // 去重后的边界点不足 (< 3 个)
  kHullTooFew  = 3,   // 凸包点不足 (< 3 个或退化成线)
  kNonConvex   = 4,   // 地块非凸 (顶点落在地块凸包内部)

  // ---- 参数配置不合理的细分 (与 insetPolygon / makeRidgeSegments 的判定对应) ----
  kBadFieldMargin = 5,   // field_margin 不合理: 内缩把可作业区域吃没 / 内缩后区域退化
  kBadRidgeWidth  = 6,   // ridge_width 不合理: <= 0 (plan() 入口) / 内缩区域里排不出垄线 (区域太窄)

  // ---- 内部兜底 ----
  kInternal = 7,   // 流程跑完仍无路点 (未知原因)
};

// 是否属于"输入数据异常"这一类 (含它的四个细分)。调用方要按类别分支时用它 —— 免得每加一个
// 细分状态都要回头改调用点, 那正是细分最容易漏判的地方。
inline bool isBadInput(PlanStatus status) {
  switch (status) {
    case PlanStatus::kInputTooFew:
    case PlanStatus::kDedupTooFew:
    case PlanStatus::kNonConvex:
    case PlanStatus::kHullTooFew:
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

// 状态 -> 简短标签 (日志/心跳 detail 用): 数值与标签一起给, 现场不必去翻枚举定义。
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
    case PlanStatus::kBadFieldMargin:
      return "bad_field_margin";
    case PlanStatus::kBadRidgeWidth:
      return "bad_ridge_width";
    case PlanStatus::kInternal:
      return "internal";
  }
  return "unknown";
}

// ---- 边界内缩 (Clipper2 偏置) 的拐角上限 ----
inline constexpr double kDefaultMiterLimit = 2.0;

// 首行余量的**下限** (m): 取 Clipper2 的坐标量化尺度 (≈1 mm)。
inline constexpr double kMinFirstRowMargin = 0.001;

// ---- 首行 (第一条垄) 的余量 ----
inline constexpr double kDefaultFirstRowMargin = 0.005;

inline constexpr uint8_t kSegWork = 0;   // 行内作业段 (垄线本身 + 沿边界边补出来的那段)
inline constexpr uint8_t kSegTurn = 1;   // 掉头弧 (地头转弯)

// ---------------------------------------------------------------------------
// 一次规划的全部输入 / 输出 (值语义)
// ---------------------------------------------------------------------------
// 新接口的两个载体: `plan(const BowInput&) -> BowOutput` (注意非 const: 状态写进对象成员)。
//   * 输入由调用方拥有并传入, 规划过程**绝不修改**它 (内部只改自己的"生效值"副本);
//   * 输出整包返回: 路径/关键点/PTO 路径/观测值/状态全在里面, 对象上不导出 getter。
// 因此: 同一个 `PlanBowShapedPath` 实例可被**多线程并发调用**, 没有共享可变状态, 也不会出现
// "读到上一轮结果 / 半成品状态"这类问题 (见 test/test_bow_parallel.cpp)。
struct BowInput {
  Polygon_2 cell;                                      // 地块顶点 (凸, ENU, 单位 m)
  double ridge_width       = 0.0;                      // 垄宽 (m, 必须 > 0)
  double path_resolution   = 0.0;                      // 跟踪路径 (/path) 的插值分辨率 (m)
  double field_margin      = 0.0;                      // 四周统一内缩的田边余量 (m)
  double miter_limit       = kDefaultMiterLimit;       // 内缩偏置的拐角上限 (Clipper2 语义)
  double first_row_margin  = kDefaultFirstRowMargin;   // 首行/末行兜底离内缩边界的余量 (m)
  double in_row_resolution = 0.0;                      // PTO 路径的行内分辨率 (m); <= 0 = 不产出 PTO 路径
};

// ---------------------------------------------------------------------------
// 结构路径 (未采样的中间产物) —— 让"只改了分辨率"不必再跑几何
// ---------------------------------------------------------------------------
// 规划分两步:
//   ① 几何 (重): 内缩 -> 扫描线裁剪 -> 垄线 -> 掉头弧 -> 端头补角, 得到**结构路径**
//      (垄/弧的顶点, 逐点带 WORK/TURN 类型);
//   ② 采样 (轻): 按分辨率插值出两条输出 (/global_path_planner/path 细路径 +
//      /global_path_planner/pto_path 行内粗采样)。
// `BowSkeleton` 就是①的产物, 由 plan() 顺带返回; 之后**只改分辨率** (path_resolution /
// in_row_resolution) 时, 调用方可以直接用 resampleBowSkeleton() 重采样, 完全跳过① (Clipper
// 偏置与扫描线裁剪都不跑)。见 GlobalPathPlannerNode 的 resampleOnly()。
struct BowSkeleton {
  std::vector<Point_2> points;   // 结构顶点: 垄段端点 + 掉头弧上的采样点。
  std::vector<uint8_t> types;    // 与 points 1:1 的类型 (只有 kSegWork / kSegTurn 两种)
  // 可选的**逐段**类型 (size == points.size() - 1): 非空时**优先**用它判段; 为空时按上面的点类型
  // 用"两端都是 WORK 才算行内段"的规则推 (本机规划走这条, 弧点逐点标 TURN, 交界点标 WORK)。
  // 外部骨架 (测试或其他调用方) 走这条: 用"点标记"表示"**上一段**是掉头段"(标记在
  // 掉头段的落点上) —— 这个口径**表达不成点类型**: 掉头段与其后的作业行共用一个端点, 那个端点
  // 无论标什么都会让一侧判错; 所以外部路径直接把段类型给进来。
  std::vector<uint8_t> seg_types;
  bool empty() const {
    return points.empty();
  }
};

// 由结构路径生成两条输出路径 —— **纯函数 (无任何状态, 可多线程并发调用)**。plan() 内部就是调它,
// 所以"只重采样"与"全量重算"的结果逐位相同 (单测 ResampleOnlyMatchesFullPlan 钉住这条):
//   * waypoints     : 整条按 path_resolution 均匀插值 (给路径跟踪的 /path);
//   * waypoint_types: 与 waypoints 逐点 1:1 的类型 (可空; 发布时写进 pose.position.z);
//   * pto_waypoints : **行内段按 in_row_resolution 加密**; 掉头段: 本机规划的弧已在几何阶段采好
//                     (每段都短于 path_resolution) 就原样保留, **比 path_resolution 还长的粗掉头段**
//                     (外部骨架的 2 点掉头) 按 path_resolution 插值 —— 不然 PTO 路径
//                     在掉头处是一条折线 (弧上最近点定位与航向差分最敏感, 见 ② 的说明);
//   * pto_seg_types : 与 pto_waypoints 逐点 1:1 的类型 (同上, 也写进 z; 不再单独发话题)。
// 段类型的判据: skeleton.seg_types 非空 (1:1 对段) 时**以它为准**; 否则"两端都是 WORK 才算行内段"。
// in_row_resolution <= 0 -> pto_* 为空 (不产出 PTO 路径); skeleton 为空 -> 输出全部清空。
void resampleBowSkeleton(
  const BowSkeleton& skeleton,
  double path_resolution,
  double in_row_resolution,
  std::vector<Point_2>& waypoints,
  std::vector<Point_2>& pto_waypoints,
  std::vector<uint8_t>& pto_seg_types,
  std::vector<uint8_t>* waypoint_types = nullptr);

// ============================================================
// 全覆盖路径规划算法的输出 (值语义)
// ============================================================
struct BowOutput {
  PlanStatus status = PlanStatus::kOk;    // kOk = 成功 (waypoints 非空)
  std::string message;                    // 失败原因 (成功时为空)
  Polygon_2 inner_polygon;                // 内缩后的垄覆盖区域 (可视化 + 参考)
  BowSkeleton skeleton;                   // 结构路径 (几何产物; 只改分辨率时可复用它重采样)
  std::vector<Point_2> waypoints;         // /path: 跟踪用的细路径
  std::vector<uint8_t> waypoint_types;    // 与 waypoints 1:1 的类型 (0=WORK/1=TURN; 发布时写进 pose.position.z)
  std::vector<Point_2> key_waypoints;     // 每条垄的两个端点 (掉头点)
  std::vector<Point_2> pto_waypoints;     // /global_path_planner/pto_path: 行内粗采样路径
  std::vector<uint8_t> pto_seg_types;     // 与 pto_waypoints 1:1 的类型 (同上, 也写进 z)
  double sweep_dir_deg           = 0.0;   // 选定的扫描方向 (deg)
  double turn_max_overrun        = 0.0;   // 掉头伸出地块边界的最大距离 (m) —— [TURN_BOUNDARY]
  double turn_max_pocket         = 0.0;   // 掉头处作业区里最大的一条盲带 (m) —— [TURN_POCKET]
  double turn_pocket_margin_hint = 0.0;   // 想盖住那条盲带需要的 field_margin (m) —— [TURN_POCKET] 建议值
  bool ok() const {
    return status == PlanStatus::kOk;
  }
};

// ============================================================
// 履带机器人的全覆盖路径规划算法
// ============================================================
class PlanBowShapedPath {
public:
  PlanBowShapedPath()  = default;
  ~PlanBowShapedPath() = default;
  BowOutput plan(const BowInput& in);

private:
  void generateWaypoints();
  bool preparePolygon(const Polygon_2& cell, Polygon_2& out);
  Polygon_2 offsetUniformly(const Polygon_2& poly, double margin);
  Polygon_2 insetPolygon(const Polygon_2& poly);
  Point_2 minWidthDirection(const Polygon_2& poly);
  std::vector<Segment_2> makeRidgeSegments(const Polygon_2& region, const Point_2& sweep_dir, const Point_2& offset_dir, const Segment_2& far_side_edge);
  std::vector<Point_2> flattenRidges(const std::vector<Segment_2>& ridges);
  std::vector<Point_2> smoothTurns(const std::vector<Point_2>& key_waypoints, std::vector<uint8_t>* types = nullptr);
  std::vector<Point_2> extendEndsToRegionCorners(
    const std::vector<Point_2>& path,
    const Segment_2& near_side_edge,
    const Segment_2& far_side_edge,
    const std::vector<uint8_t>* in_types = nullptr,
    std::vector<uint8_t>* out_types      = nullptr);
  double overrunOf(const Point_2& p);
  double maxOverrunOf(const std::vector<Point_2>& pts);
  std::vector<Point_2> legacyTurn(const Point_2& P1, const Point_2& P2, const Point_2& P3, const Point_2& P4);

private:
  void fail(PlanStatus st, const std::string& msg) {
    output_.status  = st;
    output_.message = msg;
  }

private:
  Polygon_2 cell_;                                      // 输入地块顶点 (plan() 落进来)
  double ridge_width_       = 0.0;                      // 垄宽 (m): 跟踪路径 (/path) 按此宽度;
  double path_resolution_   = 0.0;                      // 路径分辨率 (m): 跟踪路径 (/path) 按此插值;
  double field_margin_      = 0.0;                      // 边界内缩距离 (m): 边界内缩后的地块 (field_) 按此宽度;
  double miter_limit_       = kDefaultMiterLimit;       // Clipper2 拐角上限 (m): 边界内缩后的地块按此内缩;
  double first_row_margin_  = kDefaultFirstRowMargin;   // 首行离内缩边界的余量 (m): 首行贴得越近, 田边盲带越小;
  double in_row_resolution_ = 0.0;                      // PTO 路径的行内(作业段)分辨率 (m): <= 0 = 不发布 PTO 路径;
  Polygon_2 field_;                                     // 规范化后的地块 (凸、顺时针) —— 掉头"不许出界"的判据
  BowOutput output_;                                    // 规划结果 (plan() 落出去)
};
