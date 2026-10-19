#pragma once

// The aggregation core, deliberately free of rclcpp.
//
// Everything that decides whether the tractor stops lives in here, as pure
// functions over (name, status, seconds). That is what lets test_watch_table
// exercise the whole timeout/grace/aggregation matrix without spinning a node
// or faking a clock -- the same split tractor_kpi_core uses. Keep it that way:
// the moment this file includes rclcpp, the interesting cases stop being
// testable and start being field surprises.

#include <cstdint>
#include <map>
#include <string>
#include <tractor_monitor/fault_source.hpp>
#include <vector>

namespace tractor_monitor {

// What losing a module MEANS. Not how sick the module is -- that is Status.
enum class Criticality {
  kCritical,   // 关键：丢失 -> 整车停车并置急停
  kDegraded,   // 降级：丢失 -> 上报，不停车
  kIgnore,     // 忽略：丢失 -> 仅记录（例如云链路）
};

// Mirrors weeder_msgs::msg::Heartbeat::STATUS_*. Duplicated rather than
// included so this header stays message-free; static_assert'd against the real
// constants in monitor_node.cpp.
enum class Status : uint8_t {
  kOk            = 0,   // 不处理 (正常运行)
  kStopImplement = 1,   // 停止作业
  kStopMotion    = 2,   // 停止跟踪 (停运动)
  kFault         = 3,   // 全局停止
};

// 线上的 status 字节 -> Status。未知值 (旧版本节点、乱码、将来新增的状态) 一律
// 按 kFault 处理: 解释不了的状态不等于"没事", 只有全局停止才是安全的默认。
// 直接 static_cast 会把未知值原样放进裁决 —— 它比 kFault 数值更大, 于是既赢得
// "最坏状态"聚合, 又在 tractor_base 的 switch 里落进 default 被忽略。
// known 为 nullptr 时不上报。
Status statusFromWire(uint8_t v, bool* known = nullptr);

Criticality criticalityFromString(const std::string& s, bool* ok = nullptr);
const char* toString(Criticality c);
const char* toString(Status s);

struct WatchConfig {
  std::string name;
  double timeout_s        = 1.0;
  Criticality criticality = Criticality::kDegraded;

  // required=true  : the module MUST appear within the startup grace, and never
  //                  going away. Absence past grace counts against the verdict.
  // required=false : watched only once it has actually been seen. Absence never
  //                  counts; staleness after it HAS been seen still does.
  //
  // This is what lets one config serve every leg combination. bringup.launch.py
  // starts only the planner, GNSS and control legs, and has gnss:=false /
  // planner:=false / control:=false switches on top -- so a config that demanded
  // every module be present would e-stop the tractor on every boot of every
  // partial stack. Modules outside the default bringup are required=false: you
  // cannot fault on something nobody launched, but if it was launched and then
  // died, that still stops the machine.
  bool required = true;
};

struct ModuleState {
  WatchConfig config;
  bool ever_seen     = false;
  double last_seen   = 0.0;   // monotonic seconds, meaningful iff ever_seen
  Status last_status = Status::kOk;
  std::string last_detail;
  uint32_t last_seq = 0;
  uint32_t restarts = 0;   // times last_seq went backwards
  // 该模块自报的故障来源 (空 = 它自己 / 否则是具体来源: 节点名或 path_file / vcu / app 这类标签)。
  // 只透传, 用于裁决里说清"该怪谁"。
  std::string last_fault_source_node;

  // Derived, filled by evaluate()
  bool stale   = false;
  bool missing = false;   // never seen at all

  // alive = seen at least once AND not stale.
  bool alive = false;

  // The node's EFFECTIVE health, which is not the same as what it last said:
  // a module that has stopped delivering reads FAULT here no matter how
  // cheerful its final self-report was. That gap is the whole reason a monitor
  // exists, so it must not be papered over in the snapshot.
  Status effective = Status::kOk;

  // Whether this module contributed to the aggregate verdict. False for
  // ignore-class modules and for optional ones that were never launched --
  // this is the field that explains "why is that row FAULT but nothing
  // stopped?".
  bool counted = false;

  // Seconds since the last heartbeat; -1 when never seen.
  double age_s = -1.0;
};

struct Verdict {
  Status status = Status::kOk;
  std::string detail;   // names the offending modules, for the log
  bool in_startup_grace = false;
  // 导致这次裁决的模块们自报的故障来源 (空 = 都是它们自己的问题): 把它们各自报的来源去重后
  // 用 ", " 并列 —— 合并上报后 APP 靠它直接定位责任方 (谁报的谁负责, monitor 不替人拍板)。
  // 规则: 存活模块用它**自报**的来源 (空 = 它自己); 缺失/超时 (stale/missing) 的模块没有自报来源,
  // 直接归到**它自己**的名字上 —— 它不发心跳就是它自己的问题 (例如 gnss_driver 掉了 = gnss_driver)。
  // status = kOk 时无意义 (实现上给空字符串)。
  std::string fault_source_node = kFaultSourceSelf;
};

class WatchTable {
public:
  // startup_grace_s is load-bearing, not a nicety: nodes come up over several
  // seconds, so without it the monitor declares FAULT and e-stops the tractor
  // on EVERY boot.
  WatchTable(std::vector<WatchConfig> configs, double startup_grace_s, double start_time);

  // Returns false and ignores the sample when name is not in the watch list.
  // An unknown node must never be able to fault the machine.
  // fault_source_node: 该节点自报的故障来源 (空 = 它自己; 否则是具体来源), 透传给裁决用。
  bool observe(const std::string& name, Status status, const std::string& detail, uint32_t seq, double now, const std::string& fault_source_node = kFaultSourceSelf);

  Verdict evaluate(double now);

  const std::map<std::string, ModuleState>& modules() const {
    return modules_;
  }
  bool empty() const {
    return modules_.empty();
  }

private:
  std::map<std::string, ModuleState> modules_;
  double startup_grace_s_;
  double start_time_;
};

}   // namespace tractor_monitor
