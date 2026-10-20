#pragma once

// 测试专用适配器: 让既有单测的断言体在 API 改为"结果整包返回"之后**一行都不用改**。
//
// 背景: `PlanBowShapedPath` 现在的接口是 plan(BowInput) -> BowOutput, 结果不再留在对象上供 getter
// 取用, 于是 `init(...) / lastResult() / getWaypoints() ...` 这套旧 API 就没有了。但既有用例的
// 断言体 (垄数/覆盖率/自交/契约) 是**几何契约**, 与"结果从哪取"无关 —— 全部重写成 `out.waypoints`
// 只会制造上千行机械 diff 与误改风险。所以这里放一个**只给测试用**的门面, 语义就是旧 API:
//   组装 BowInput -> 调引擎 -> 把 BowOutput 缓存在自己身上 -> getter 转发到缓存。
// 缓存放在门面上而不是去读引擎内部状态: 引擎内部状态是"一次规划的中间态"(见 bow.hpp), 不对外暴露。
// 门面本身**不是**线程安全的 (它就是"旧的有状态用法"的投影), 并发相关的测试请直接用
// `PlanBowShapedPath + BowInput/BowOutput` (见 test_bow_parallel.cpp)。
//
// 注意: 只用于测试, 不进 install (在 test/ 下), 所以生产代码不会看到这个"历史包袱"。

#include <string>
#include <utility>
#include <vector>

#include "global_path_planner/bow.hpp"

namespace bow_test {

class BowFacade {
public:
  BowFacade() = default;

  // ---- 旧 API: init / plan ----
  bool init(const Polygon_2& cell, double ridge_width, double path_resolution, double field_margin = 0.0,
            double first_row_margin = 0.005) {
    return plan(cell, ridge_width, path_resolution, field_margin, first_row_margin).ok();
  }

  BowOutput plan(const Polygon_2& cell, double ridge_width, double path_resolution, double field_margin = 0.0,
                 double first_row_margin = 0.005) {
    BowInput in;
    in.cell             = cell;
    in.ridge_width      = ridge_width;
    in.path_resolution  = path_resolution;
    in.field_margin     = field_margin;
    in.first_row_margin = first_row_margin;
    out_                = engine_.plan(in);
    return out_;
  }

  // ---- 旧 API: getter (转发到最近一次的 BowOutput) ----
  const BowOutput& lastResult() const {
    return out_;
  }
  double getSweepDirDeg() const {
    return out_.sweep_dir_deg;
  }
  double getTurnMaxOverrun() const {
    return out_.turn_max_overrun;
  }
  const std::vector<Point_2>& getWaypoints() const {
    return out_.waypoints;
  }
  const std::vector<Point_2>& getKeyWaypoints() const {
    return out_.key_waypoints;
  }
  const Polygon_2& getInnerPolygon() const {
    return out_.inner_polygon;
  }
  // 需要直接看新 API (或拿引擎做并发测试) 时用。
  PlanBowShapedPath& engine() {
    return engine_;
  }
  const BowOutput& output() const {
    return out_;
  }

private:
  PlanBowShapedPath engine_;   // 一次规划的状态在它身上 (见 bow.hpp)
  BowOutput out_;              // 最近一次结果 (旧 API 的"状态"由门面自己缓存)
};

}   // namespace bow_test
