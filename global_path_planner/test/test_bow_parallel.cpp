// ============================================================
// 并发契约 (状态在对象里: 见 bow.hpp 的私有成员)
// ============================================================
// 一次规划的状态保存在对象里 —— 所以**同一个实例不能并发调用** (本节点是单线程的, 够用)。
// 但类里没有任何静态/全局状态, 于是"每个线程各用一个实例"必须仍然逐位正确。钉住四条:
//   ① 每线程一个实例 + 同一输入: 各线程各算多次, 每次输出与单线程基准**逐位相同**;
//   ② 混合输入并发: 各线程用不同地块/参数交错执行, 每组输出与各自基准逐位相同
//      (若有隐藏的跨实例共享状态, 这里必然串改);
//   ③ plan() **不修改输入**: 并发跑完, 每个 BowInput 与原值逐位相同;
//   ④ 失败路径也如此: 非法输入 (非凸 / 垄宽为 0) 并发跑, 每次都拿到同一个 status/message。
// 逐位比较而不是 EXPECT_NEAR: 契约是"结果与单线程一模一样", 不是"差不多"。
#include <gtest/gtest.h>

#include <atomic>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "global_path_planner/bow.hpp"

namespace {

bool SameBits(double a, double b) {
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}

bool SamePath(const std::vector<Point_2>& a, const std::vector<Point_2>& b) {
  if (a.size() != b.size())
    return false;
  for (size_t i = 0; i < a.size(); ++i) {
    if (!SameBits(a[i].x(), b[i].x()) || !SameBits(a[i].y(), b[i].y()))
      return false;
  }
  return true;
}

bool SameOutput(const BowOutput& a, const BowOutput& b) {
  if (a.status != b.status || a.message != b.message)
    return false;
  if (!SamePath(a.waypoints, b.waypoints) || !SamePath(a.key_waypoints, b.key_waypoints) || !SamePath(a.inner_polygon, b.inner_polygon))
    return false;
  return SameBits(a.sweep_dir_deg, b.sweep_dir_deg) && SameBits(a.turn_max_overrun, b.turn_max_overrun);
}

Polygon_2 Rect(double w, double h) {
  return {Point_2(0.0, 0.0), Point_2(w, 0.0), Point_2(w, h), Point_2(0.0, h)};
}

// 一组"互不相同"的输入 (地块/垄宽/分辨率/余量都不同), 供混合并发用。
std::vector<BowInput> MakeInputs() {
  std::vector<BowInput> inputs;
  for (int i = 0; i < 8; ++i) {
    BowInput in;
    in.ridge_width       = 0.6 + 0.3 * static_cast<double>(i);
    in.path_resolution   = 0.1 + 0.05 * static_cast<double>(i % 3);
    in.field_margin      = 0.2 * static_cast<double>(i % 4);
    in.first_row_margin  = 0.005;
    if (i % 2 == 0)
      in.cell = Rect(20.0 + static_cast<double>(i), 8.0 + 0.5 * static_cast<double>(i));
    else
      in.cell = {Point_2(0.0, 0.0), Point_2(13.45, 0.57), Point_2(13.34, 5.32), Point_2(0.00, 4.87)};
    inputs.push_back(in);
  }
  return inputs;
}

}   // namespace

// ① + ③: 同一输入 + **每线程一个实例**, 8 线程各算 40 次; 结果逐位一致, 且输入不被改动。
TEST(BowParallel, PerInstanceSameInputIsBitIdentical) {
  PlanBowShapedPath engine;   // 主线程基准用 (状态在对象里, 所以不能给线程们共用)
  BowInput in;
  in.cell              = Rect(20.0, 8.0);
  in.ridge_width       = 1.0;
  in.path_resolution   = 0.2;
  in.field_margin      = 0.5;
  const BowInput in_before = in;

  const BowOutput reference = engine.plan(in);   // 单线程基准
  ASSERT_TRUE(reference.ok());
  constexpr int kThreads = 8;
  constexpr int kIters   = 40;
  std::atomic<int> mismatches{0};
  std::atomic<int> failures{0};
  std::vector<std::thread> workers;
  workers.reserve(kThreads);
  for (int t = 0; t < kThreads; ++t) {
    workers.emplace_back([&] {
      PlanBowShapedPath local_engine;   // 线程自己的实例 (类里没有静态/全局状态)
      for (int it = 0; it < kIters; ++it) {
        const BowOutput out = local_engine.plan(in);
        if (!out.ok())
          ++failures;
        if (!SameOutput(out, reference))
          ++mismatches;
      }
    });
  }
  for (auto& w : workers)
    w.join();

  EXPECT_EQ(failures.load(), 0) << "并发调用里出现了失败";
  EXPECT_EQ(mismatches.load(), 0) << "每线程一个实例时结果与单线程基准不一致 (有静态/全局状态?!)";
  // 输入逐位未被改动 (plan() 只读入参; 夹取写在内部生效值上)
  EXPECT_TRUE(SameBits(in.ridge_width, in_before.ridge_width));
  EXPECT_TRUE(SameBits(in.path_resolution, in_before.path_resolution));
  EXPECT_TRUE(SameBits(in.field_margin, in_before.field_margin));
  EXPECT_TRUE(SameBits(in.first_row_margin, in_before.first_row_margin));
  EXPECT_TRUE(SamePath(in.cell, in_before.cell));
}


// ②: 混合输入并发 —— 8 个不同输入 × 8 线程 (每线程一个实例) 交错, 每组都要与自己的基准逐位一致。
TEST(BowParallel, MixedInputsInterleavedStayBitIdentical) {
  PlanBowShapedPath engine;
  const std::vector<BowInput> inputs = MakeInputs();

  std::vector<BowOutput> reference;
  reference.reserve(inputs.size());
  for (const auto& in : inputs) {
    const BowOutput out = engine.plan(in);
    ASSERT_TRUE(out.ok());
    reference.push_back(out);
  }
  constexpr int kIters = 25;
  std::atomic<int> mismatches{0};
  std::vector<std::thread> workers;
  workers.reserve(inputs.size());
  for (size_t k = 0; k < inputs.size(); ++k) {
    workers.emplace_back([&, k] {
      PlanBowShapedPath local_engine;   // 每线程一个实例
      for (int it = 0; it < kIters; ++it) {
        const BowOutput out = local_engine.plan(inputs[k]);
        if (!SameOutput(out, reference[k]))
          ++mismatches;
      }
    });
  }
  for (auto& w : workers)
    w.join();
  EXPECT_EQ(mismatches.load(), 0) << "混合输入交错执行时结果被串改 (有跨实例的共享状态?!)";
}

// ④: 失败路径并发 —— 非法输入也要每次拿到同一个 status/message, 且不留路点。
TEST(BowParallel, FailurePathsAreAlsoStable) {
  PlanBowShapedPath engine;

  BowInput concave;   // 凹多边形 -> kNonConvex
  concave.cell            = {Point_2(0.0, 0.0), Point_2(10.0, 0.0), Point_2(5.0, 1.0), Point_2(10.0, 10.0), Point_2(0.0, 10.0)};
  concave.ridge_width     = 2.0;
  concave.path_resolution = 0.2;

  BowInput bad_ridge;   // 垄宽 0 -> kBadRidgeWidth
  bad_ridge.cell            = Rect(20.0, 8.0);
  bad_ridge.ridge_width     = 0.0;
  bad_ridge.path_resolution = 0.2;

  const BowOutput ref_concave   = engine.plan(concave);
  const BowOutput ref_bad_ridge = engine.plan(bad_ridge);
  ASSERT_FALSE(ref_concave.ok());
  ASSERT_FALSE(ref_bad_ridge.ok());
  ASSERT_EQ(ref_concave.status, PlanStatus::kNonConvex);
  ASSERT_EQ(ref_bad_ridge.status, PlanStatus::kBadRidgeWidth);

  std::atomic<int> mismatches{0};
  std::vector<std::thread> workers;
  for (int t = 0; t < 8; ++t) {
    workers.emplace_back([&] {
      PlanBowShapedPath local_engine;   // 每线程一个实例
      for (int it = 0; it < 30; ++it) {
        const BowOutput out_c = local_engine.plan(concave);
        const BowOutput out_r = local_engine.plan(bad_ridge);
        if (out_c.status != ref_concave.status || out_c.message != ref_concave.message)
          ++mismatches;
        if (out_r.status != ref_bad_ridge.status || out_r.message != ref_bad_ridge.message)
          ++mismatches;
        if (!out_c.waypoints.empty() || !out_r.waypoints.empty())
          ++mismatches;   // 失败时不该留下路点
      }
    });
  }
  for (auto& w : workers)
    w.join();
  EXPECT_EQ(mismatches.load(), 0) << "失败路径在并发下不稳定";
}
