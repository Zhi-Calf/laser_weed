// The whole point of keeping watch_table rclcpp-free: every case below decides
// whether a real tractor stops, and all of them run in milliseconds with no ROS
// graph, no clock faking and no timing flake.

#include <gtest/gtest.h>

#include "tractor_monitor/watch_table.hpp"

using namespace tractor_monitor;   // NOLINT

namespace {

constexpr double kGrace = 10.0;
constexpr double kT0    = 100.0;   // arbitrary non-zero start, to catch code
                                   // that assumes the clock starts at 0

WatchConfig cfg(const std::string& name, Criticality c, double timeout = 1.0,
                bool required = true) {
  WatchConfig w;
  w.name        = name;
  w.criticality = c;
  w.timeout_s   = timeout;
  w.required    = required;
  return w;
}

WatchTable make(std::vector<WatchConfig> cfgs) {
  return WatchTable(std::move(cfgs), kGrace, kT0);
}

}   // namespace

// ---------------------------------------------------------------- startup ---

TEST(WatchTable, RequiredButNeverSeenIsOkInsideGrace) {
  // The case that would otherwise e-stop the tractor on every single boot.
  auto t = make({cfg("path_controller", Criticality::kCritical)});
  const Verdict v = t.evaluate(kT0 + kGrace - 0.1);
  EXPECT_EQ(v.status, Status::kOk);
  EXPECT_TRUE(v.in_startup_grace);
}

TEST(WatchTable, RequiredButNeverSeenFaultsAfterGrace) {
  auto t = make({cfg("path_controller", Criticality::kCritical)});
  const Verdict v = t.evaluate(kT0 + kGrace + 0.1);
  EXPECT_EQ(v.status, Status::kFault);
  EXPECT_NE(v.detail.find("path_controller"), std::string::npos);
}

TEST(WatchTable, OptionalAndNeverSeenIsAlwaysOk) {
  // A leg that was never launched (control:=false, or a module bringup does not
  // start at all) must never stop the machine.
  auto t = make({cfg("some_optional_node", Criticality::kCritical, 1.0, false)});
  EXPECT_EQ(t.evaluate(kT0 + 1000.0).status, Status::kOk);
  EXPECT_TRUE(t.evaluate(kT0 + 1000.0).detail.empty());
}

TEST(WatchTable, OptionalStillFaultsOnceSeenThenLost) {
  // "Watched only if present" must not mean "never watched".
  auto t = make({cfg("some_optional_node", Criticality::kCritical, 1.0, false)});
  ASSERT_TRUE(t.observe("some_optional_node", Status::kOk, "", 1, kT0 + 20.0));
  EXPECT_EQ(t.evaluate(kT0 + 20.5).status, Status::kOk);
  EXPECT_EQ(t.evaluate(kT0 + 22.0).status, Status::kFault);
}

// ---------------------------------------------------------------- staleness -

TEST(WatchTable, FreshCriticalIsOk) {
  auto t = make({cfg("path_controller", Criticality::kCritical, 0.5)});
  ASSERT_TRUE(t.observe("path_controller", Status::kOk, "", 1, kT0 + 1.0));
  EXPECT_EQ(t.evaluate(kT0 + 1.4).status, Status::kOk);
}

TEST(WatchTable, StaleCriticalFaults) {
  auto t = make({cfg("path_controller", Criticality::kCritical, 0.5)});
  ASSERT_TRUE(t.observe("path_controller", Status::kOk, "", 1, kT0 + 1.0));
  const Verdict v = t.evaluate(kT0 + 1.6);
  EXPECT_EQ(v.status, Status::kFault);
  EXPECT_NE(v.detail.find("path_controller"), std::string::npos);
}

TEST(WatchTable, StaleDegradedClassNeverFaults) {
  // A degraded-class module going away is reported, never a stop. Its loss is
  // recorded in detail but does not change the verdict (stays OK).
  auto t = make({cfg("some_degraded_node", Criticality::kDegraded, 0.5)});
  ASSERT_TRUE(t.observe("some_degraded_node", Status::kOk, "", 1, kT0 + 1.0));
  const Verdict v = t.evaluate(kT0 + 5.0);
  EXPECT_EQ(v.status, Status::kOk);
  EXPECT_NE(v.detail.find("some_degraded_node"), std::string::npos);
}

TEST(WatchTable, StaleIgnoredClassStaysOk) {
  // ignore-class: no module uses it today, but the option is still supported
  // by watch_table and must stay tested.
  auto t = make({cfg("some_telemetry_node", Criticality::kIgnore, 0.5)});
  ASSERT_TRUE(t.observe("some_telemetry_node", Status::kOk, "", 1, kT0 + 1.0));
  EXPECT_EQ(t.evaluate(kT0 + 100.0).status, Status::kOk);
}

TEST(WatchTable, RecoveryIsAutomatic) {
  // The non-latching contract: a fresh sample clears the fault with no operator
  // action, matching tractor_base's existing cmd_timeout behaviour.
  auto t = make({cfg("path_controller", Criticality::kCritical, 0.5)});
  ASSERT_TRUE(t.observe("path_controller", Status::kOk, "", 1, kT0 + 1.0));
  ASSERT_EQ(t.evaluate(kT0 + 2.0).status, Status::kFault);
  ASSERT_TRUE(t.observe("path_controller", Status::kOk, "", 2, kT0 + 2.1));
  EXPECT_EQ(t.evaluate(kT0 + 2.2).status, Status::kOk);
}

// ------------------------------------------------------- reported statuses --

TEST(WatchTable, SelfReportedFaultOnCriticalFaults) {
  // A module that is alive but says it cannot do its job stops the machine just
  // as surely as one that died.
  auto t = make({cfg("gnss_driver", Criticality::kCritical)});
  ASSERT_TRUE(t.observe("gnss_driver", Status::kFault, "no fix", 1, kT0 + 1.0));
  const Verdict v = t.evaluate(kT0 + 1.1);
  EXPECT_EQ(v.status, Status::kFault);
  EXPECT_NE(v.detail.find("no fix"), std::string::npos);
}

TEST(WatchTable, SelfReportedStopMotionOnCriticalStopsMotion) {
  // RTK float instead of fixed, path_controller idle waiting for /work_mode: the
  // node self-reports STOP_MOTION (stop tracking), which surfaces as the verdict.
  auto t = make({cfg("gnss_driver", Criticality::kCritical)});
  ASSERT_TRUE(t.observe("gnss_driver", Status::kStopMotion, "rtk float", 1, kT0 + 1.0));
  EXPECT_EQ(t.evaluate(kT0 + 1.1).status, Status::kStopMotion);
}

TEST(WatchTable, SelfReportedFaultOnIgnoredClassStaysOk) {
  auto t = make({cfg("some_telemetry_node", Criticality::kIgnore)});
  ASSERT_TRUE(t.observe("some_telemetry_node", Status::kFault, "mqtt down", 1, kT0 + 1.0));
  EXPECT_EQ(t.evaluate(kT0 + 1.1).status, Status::kOk);
}

// ------------------------------------------------------------- aggregation --

TEST(WatchTable, FaultWinsOverStopMotion) {
  auto t = make({cfg("path_controller", Criticality::kCritical, 0.5),
                 cfg("gnss_driver", Criticality::kCritical, 0.5)});
  ASSERT_TRUE(t.observe("path_controller", Status::kFault, "boom", 1, kT0 + 1.0));
  ASSERT_TRUE(t.observe("gnss_driver", Status::kStopMotion, "meh", 1, kT0 + 1.0));
  const Verdict v = t.evaluate(kT0 + 1.1);
  EXPECT_EQ(v.status, Status::kFault);
  // The detail names the FAULTING module -- that is the line an operator greps
  // for -- and is not diluted by the merely-stopping one.
  EXPECT_NE(v.detail.find("path_controller"), std::string::npos);
  EXPECT_NE(v.detail.find("gnss_driver"), std::string::npos);
}

TEST(WatchTable, DetailNamesEveryFaultingModule) {
  auto t = make({cfg("path_controller", Criticality::kCritical, 0.5),
                 cfg("gnss_driver", Criticality::kCritical, 0.5)});
  ASSERT_TRUE(t.observe("path_controller", Status::kFault, "", 1, kT0 + 1.0));
  ASSERT_TRUE(t.observe("gnss_driver", Status::kFault, "", 1, kT0 + 1.0));
  const Verdict v = t.evaluate(kT0 + 1.1);
  EXPECT_NE(v.detail.find("path_controller"), std::string::npos);
  EXPECT_NE(v.detail.find("gnss_driver"), std::string::npos);
}

TEST(WatchTable, EmptyTableIsOk) {
  auto t = make({});
  EXPECT_EQ(t.evaluate(kT0 + 1000.0).status, Status::kOk);
  EXPECT_TRUE(t.empty());
}

// ------------------------------------------------------------------ observe -

TEST(WatchTable, UnknownNodeIsRejected) {
  // An unknown node must never be able to fault the machine, nor silently
  // create a watch entry.
  auto t = make({cfg("path_controller", Criticality::kCritical)});
  EXPECT_FALSE(t.observe("some_random_node", Status::kFault, "", 1, kT0 + 1.0));
  EXPECT_EQ(t.modules().size(), 1u);
}

TEST(WatchTable, SeqGoingBackwardsCountsARestart) {
  // A crash-looping node looks healthy in every sample taken between restarts;
  // the seq reset is the only thing that gives it away.
  auto t = make({cfg("path_controller", Criticality::kCritical)});
  ASSERT_TRUE(t.observe("path_controller", Status::kOk, "", 42, kT0 + 1.0));
  ASSERT_TRUE(t.observe("path_controller", Status::kOk, "", 0, kT0 + 1.2));
  EXPECT_EQ(t.modules().at("path_controller").restarts, 1u);
}

// ------------------------------------------------------------------ parsing -

TEST(CriticalityFromString, ParsesKnownValues) {
  bool ok = false;
  EXPECT_EQ(criticalityFromString("critical", &ok), Criticality::kCritical);
  EXPECT_TRUE(ok);
  EXPECT_EQ(criticalityFromString("degraded", &ok), Criticality::kDegraded);
  EXPECT_TRUE(ok);
  EXPECT_EQ(criticalityFromString("ignore", &ok), Criticality::kIgnore);
  EXPECT_TRUE(ok);
}

TEST(CriticalityFromString, TypoFallsBackToDegradedAndReportsFailure) {
  // Neither "critical" (a typo would stop the tractor) nor "ignore" (a typo
  // would disarm the watch).
  bool ok = true;
  EXPECT_EQ(criticalityFromString("CRITICAL", &ok), Criticality::kDegraded);
  EXPECT_FALSE(ok);
}

// ------------------------------------------- per-node snapshot (/system_health)

TEST(WatchTable, EffectiveStatusIsFaultWhenStaleEvenIfLastReportWasOk) {
  // The gap the whole monitor exists to close: a module that died mid-run has
  // a perfectly cheerful final self-report. /heartbeat alone would show it OK
  // forever.
  auto t = make({cfg("path_controller", Criticality::kCritical, 0.5)});
  ASSERT_TRUE(t.observe("path_controller", Status::kOk, "", 1, kT0 + 1.0));
  t.evaluate(kT0 + 5.0);
  const auto& m = t.modules().at("path_controller");
  EXPECT_EQ(m.last_status, Status::kOk);        // what it SAID
  EXPECT_EQ(m.effective, Status::kFault);       // what is actually true
  EXPECT_FALSE(m.alive);
}

TEST(WatchTable, AgeIsNegativeWhenNeverSeen) {
  auto t = make({cfg("some_optional_node", Criticality::kCritical, 1.0, false)});
  t.evaluate(kT0 + 100.0);
  const auto& m = t.modules().at("some_optional_node");
  EXPECT_FALSE(m.ever_seen);
  EXPECT_LT(m.age_s, 0.0);
}

TEST(WatchTable, IgnoredClassIsNeverCounted) {
  // This is the field that answers "why is that row FAULT but nothing stopped?"
  auto t = make({cfg("some_telemetry_node", Criticality::kIgnore, 0.5)});
  ASSERT_TRUE(t.observe("some_telemetry_node", Status::kFault, "mqtt down", 1, kT0 + 1.0));
  t.evaluate(kT0 + 1.1);
  const auto& m = t.modules().at("some_telemetry_node");
  EXPECT_EQ(m.effective, Status::kFault);
  EXPECT_FALSE(m.counted);
}

TEST(WatchTable, NeverLaunchedOptionalIsNotCounted) {
  auto t = make({cfg("some_optional_node", Criticality::kCritical, 1.0, false)});
  t.evaluate(kT0 + 100.0);
  EXPECT_FALSE(t.modules().at("some_optional_node").counted);
}

TEST(WatchTable, LiveCriticalModuleIsCounted) {
  auto t = make({cfg("path_controller", Criticality::kCritical, 0.5)});
  ASSERT_TRUE(t.observe("path_controller", Status::kOk, "", 1, kT0 + 1.0));
  t.evaluate(kT0 + 1.1);
  const auto& m = t.modules().at("path_controller");
  EXPECT_TRUE(m.alive);
  EXPECT_TRUE(m.counted);
  EXPECT_EQ(m.effective, Status::kOk);
  EXPECT_NEAR(m.age_s, 0.1, 1e-9);
}

// ------------------------------------------------------- 线值状态的解析 ---

TEST(StatusFromWire, KnownValuesRoundTrip) {
  for (auto s : {Status::kOk, Status::kStopImplement, Status::kStopMotion, Status::kFault}) {
    bool known = false;
    EXPECT_EQ(statusFromWire(static_cast<uint8_t>(s), &known), s);
    EXPECT_TRUE(known);
  }
}

TEST(StatusFromWire, UnknownValueBecomesFault) {
  // 回归: 直接 static_cast 时, 未知值比 kFault 数值更大, 会赢得"最坏状态"聚合
  // 再在 tractor_base 的 switch 里落进 default 被忽略 —— 既污染裁决又不停车。
  for (uint8_t v : {uint8_t{4}, uint8_t{7}, uint8_t{255}}) {
    bool known = true;
    EXPECT_EQ(statusFromWire(v, &known), Status::kFault);
    EXPECT_FALSE(known);
  }
}

TEST(StatusFromWire, KnownFlagIsOptional) {
  EXPECT_EQ(statusFromWire(200), Status::kFault);
  EXPECT_EQ(statusFromWire(static_cast<uint8_t>(Status::kStopMotion)), Status::kStopMotion);
}

TEST(StatusFromWire, FaultOutranksEveryOtherState) {
  // 聚合按数值比大小 (见 evaluate), 所以严重程度必须与数值同序。
  EXPECT_GT(static_cast<uint8_t>(Status::kFault), static_cast<uint8_t>(Status::kStopMotion));
  EXPECT_GT(static_cast<uint8_t>(Status::kStopMotion), static_cast<uint8_t>(Status::kStopImplement));
  EXPECT_GT(static_cast<uint8_t>(Status::kStopImplement), static_cast<uint8_t>(Status::kOk));
}

// ------------------------------------------------ 故障来源 (责任归属 / fault_source_node) ---
// 故障状态会合并上报 (/monitor_heartbeat), APP 靠 Verdict::fault_source_node 直接定位责任方:
// 存活模块用**它自报**的来源 (空 = 它自己, 否则是具体节点名/来源标签); 缺失或超时的模块没有
// 自报来源, 直接归到**它自己**的名字上 (它不发心跳就是它的问题)。

TEST(WatchTableFaultSource, SelfReportedFaultKeepsEmptySource) {
  auto t = make({cfg("path_controller", Criticality::kCritical, 0.5)});
  ASSERT_TRUE(t.observe("path_controller", Status::kFault, "自己坏了", 1, kT0 + 1.0));
  const Verdict v = t.evaluate(kT0 + 1.2);
  EXPECT_EQ(v.status, Status::kFault);
  EXPECT_TRUE(v.fault_source_node.empty()) << "模块自报'自身'时, 汇总来源应为空";
}

TEST(WatchTableFaultSource, NamesTheReportedCulprit) {
  auto t = make({cfg("path_controller", Criticality::kCritical, 0.5),
                 cfg("gnss_driver", Criticality::kCritical, 0.5)});
  ASSERT_TRUE(t.observe("path_controller", Status::kFault, "没有 RTK 位姿", 1, kT0 + 1.0, "gnss_driver"));
  const Verdict v = t.evaluate(kT0 + 1.2);
  EXPECT_EQ(v.status, Status::kFault);
  EXPECT_EQ(v.fault_source_node, "gnss_driver") << "必须写清具体是哪个节点";
}

TEST(WatchTableFaultSource, ListsAllCulpritsAtTheSameWorstLevel) {
  auto t = make({cfg("a_node", Criticality::kCritical, 0.5),
                 cfg("b_node", Criticality::kCritical, 0.5),
                 cfg("c_node", Criticality::kCritical, 0.5)});
  ASSERT_TRUE(t.observe("a_node", Status::kFault, "x", 1, kT0 + 1.0, "gnss_driver"));
  ASSERT_TRUE(t.observe("b_node", Status::kFault, "y", 1, kT0 + 1.0, "gnss_driver"));   // 同一来源 -> 去重
  ASSERT_TRUE(t.observe("c_node", Status::kStopMotion, "z", 1, kT0 + 1.0, "app"));      // 档次更低 -> 不参与
  const Verdict v = t.evaluate(kT0 + 1.2);
  EXPECT_EQ(v.status, Status::kFault);
  EXPECT_EQ(v.fault_source_node, "gnss_driver");
}

TEST(WatchTableFaultSource, StaleModuleIsItsOwnFault) {
  auto t = make({cfg("gnss_driver", Criticality::kCritical, 0.5)});
  ASSERT_TRUE(t.observe("gnss_driver", Status::kOk, "", 1, kT0 + 1.0));
  const Verdict v = t.evaluate(kT0 + 2.0);   // 超过 0.5 s 超时 -> 归到它自己
  EXPECT_EQ(v.status, Status::kFault);
  EXPECT_EQ(v.fault_source_node, "gnss_driver") << "掉线的模块就是它自己的问题";
}

TEST(WatchTableFaultSource, AllOkGivesEmptySource) {
  auto t = make({cfg("path_controller", Criticality::kCritical, 0.5)});
  ASSERT_TRUE(t.observe("path_controller", Status::kOk, "", 1, kT0 + 1.0, "gnss_driver"));
  const Verdict v = t.evaluate(kT0 + 1.2);
  EXPECT_EQ(v.status, Status::kOk);
  EXPECT_TRUE(v.fault_source_node.empty()) << "没有故障时该字段无意义, 给空串";
}

