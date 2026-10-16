#include "tractor_monitor/watch_table.hpp"

#include <algorithm>
#include <sstream>
#include <utility>

namespace tractor_monitor {

Criticality criticalityFromString(const std::string& s, bool* ok) {
  if (ok) {
    *ok = true;
  }
  if (s == "critical") {
    return Criticality::kCritical;
  }
  if (s == "degraded") {
    return Criticality::kDegraded;
  }
  if (s == "ignore") {
    return Criticality::kIgnore;
  }
  // An unrecognised value must NOT silently become "critical" -- a typo in a
  // yaml would then stop the tractor -- nor silently become "ignore", which
  // would disarm the monitor. Fall back to degraded (report, do not stop) and
  // let the caller log loudly.
  if (ok) {
    *ok = false;
  }
  return Criticality::kDegraded;
}

const char* toString(Criticality c) {
  switch (c) {
    case Criticality::kCritical:
      return "critical";
    case Criticality::kDegraded:
      return "degraded";
    case Criticality::kIgnore:
      return "ignore";
  }
  return "?";
}

Status statusFromWire(uint8_t v, bool* known) {
  switch (v) {
    case static_cast<uint8_t>(Status::kOk):
    case static_cast<uint8_t>(Status::kStopImplement):
    case static_cast<uint8_t>(Status::kStopMotion):
    case static_cast<uint8_t>(Status::kFault):
      if (known) *known = true;
      return static_cast<Status>(v);
    default:
      if (known) *known = false;
      return Status::kFault;
  }
}

const char* toString(Status s) {
  switch (s) {
    case Status::kOk:
      return "OK";
    case Status::kStopImplement:
      return "STOP_IMPLEMENT";
    case Status::kStopMotion:
      return "STOP_MOTION";
    case Status::kFault:
      return "FAULT";
  }
  return "?";
}

WatchTable::WatchTable(std::vector<WatchConfig> configs, double startup_grace_s, double start_time) : startup_grace_s_(startup_grace_s), start_time_(start_time) {
  for (auto& cfg : configs) {
    ModuleState st;
    st.config = std::move(cfg);
    modules_.emplace(st.config.name, std::move(st));
  }
}

bool WatchTable::observe(const std::string& name, Status status, const std::string& detail, uint32_t seq, double now, const std::string& fault_source_node) {
  auto it = modules_.find(name);
  if (it == modules_.end()) {
    return false;
  }
  ModuleState& m = it->second;
  if (m.ever_seen && seq < m.last_seq) {
    ++m.restarts;
  }
  m.ever_seen             = true;
  m.last_seen             = now;
  m.last_status           = status;
  m.last_detail           = detail;
  m.last_seq              = seq;
  m.last_fault_source_node = fault_source_node;
  return true;
}

Verdict WatchTable::evaluate(double now) {
  Verdict v;
  v.in_startup_grace = (now - start_time_) < startup_grace_s_;

  std::vector<std::string> reasons;
  Status worst = Status::kOk;
  // 导致裁决的那些模块自报的来源标签 (空 = 它们自己): 去重后并列, 见 Verdict::fault_source_node。
  std::vector<std::string> worst_sources;

  for (auto& kv : modules_) {
    ModuleState& m = kv.second;

    m.missing = !m.ever_seen;
    m.stale   = m.ever_seen && (now - m.last_seen) > m.config.timeout_s;
    m.alive   = m.ever_seen && !m.stale;
    m.age_s   = m.ever_seen ? (now - m.last_seen) : -1.0;

    // Effective health: not delivering outranks whatever it last claimed.
    m.effective = m.alive ? m.last_status : Status::kFault;

    // A module never yet seen does not count against anything while the stack
    // is still coming up -- nor ever, if it was not required to show up.
    const bool absent = m.missing ? (m.config.required && !v.in_startup_grace) : m.stale;

    m.counted = (m.config.criticality != Criticality::kIgnore) && !(m.missing && !m.config.required);

    // Nothing to say about an optional module that simply was not launched.
    if (m.missing && !m.config.required) {
      continue;
    }

    // 该模块对裁决的贡献状态:
    //   - 缺失/超时 (absent) -> FAULT (最严重)
    //   - 存活 -> 自报状态
    // ignore 类模块不参与裁决。
    Status contrib = Status::kOk;
    if (m.config.criticality == Criticality::kIgnore) {
      continue;
    }
    if (absent) {
      // degraded 类模块缺失: 仅记录, 不进裁决 (丢失只上报、不停车)。
      if (m.config.criticality == Criticality::kDegraded) {
        std::ostringstream why;
        why << m.config.name << " (缺失/lost)";
        reasons.push_back(why.str());
        continue;
      }
      contrib = Status::kFault;
    } else {
      contrib = m.last_status;
    }

    if (static_cast<uint8_t>(contrib) > static_cast<uint8_t>(worst)) {
      worst = contrib;
      worst_sources.clear();
      worst_sources.push_back(absent ? m.config.name : m.last_fault_source_node);
    } else if (contrib == worst && contrib != Status::kOk) {
      // 同一档的模块都算致因: 来源一起收集, 最后去重并列 (APP 能看到全部责任方)。
      worst_sources.push_back(absent ? m.config.name : m.last_fault_source_node);
    }

    std::ostringstream why;
    why << m.config.name;
    if (m.missing) {
      why << " (未出现/never started)";
    } else if (m.stale) {
      why << " (心跳超时/stale " << (now - m.last_seen) << "s)";
    } else {
      why << " (" << toString(m.last_status) << ")";
      if (!m.last_detail.empty()) {
        why << ": " << m.last_detail;
      }
    }
    reasons.push_back(why.str());
  }

  v.status            = worst;
  v.fault_source_node = (worst == Status::kOk) ? std::string(kFaultSourceSelf) : joinFaultSources(worst_sources);

  // This string is what an operator reads in a log at 2am in a field. Name the
  // modules, in order, and do not truncate the first few.
  std::ostringstream detail;
  for (size_t i = 0; i < reasons.size(); ++i) {
    if (i) {
      detail << "; ";
    }
    detail << reasons[i];
  }
  v.detail = detail.str();
  return v;
}

}   // namespace tractor_monitor
