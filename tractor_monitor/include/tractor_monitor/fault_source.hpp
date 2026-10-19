#pragma once

// 故障来源 (责任归属) 的公共小工具 —— 语义见 weeder_msgs/msg/Heartbeat.msg 的 fault_source_node:
//   * 空字符串 = 报故障的那个节点自己 (自检 / 自己的算法 / 自己打交道的链路);
//   * 非空     = 造成这条故障的**具体来源**: 通常是一个 ROS 节点名 ("gnss_driver"),
//                确实没有对应节点的用稳定的来源标签 ("path_file" 外部工具写的 txt、
//                "vcu" 底盘控制器、"app" 云端/操作手)。
// 一次心跳里多个故障码来源各不相同时按 ", " 并列 —— 与 /monitor_heartbeat 的合并格式一致。
//
// 这个头刻意**不 include 消息**: watch_table.hpp 要保持 message-free (见该文件开头的说明),
// 它也需要这里的合并工具。

#include <algorithm>
#include <string>
#include <vector>

namespace tractor_monitor {

// 自身 (缺省): 写成常量只是为了调用点读起来有名字, 值就是空串。
inline constexpr const char* kFaultSourceSelf = "";

// 合并来源标签: 全空 -> "" (都是各自的问题); 否则去重后按出现顺序用 ", " 连接。
inline std::string joinFaultSources(const std::vector<std::string>& sources) {
  std::vector<std::string> uniq;
  for (const auto& s : sources) {
    if (s.empty() || std::find(uniq.begin(), uniq.end(), s) != uniq.end()) {
      continue;
    }
    uniq.push_back(s);
  }
  std::string out;
  for (const auto& s : uniq) {
    if (!out.empty()) {
      out += ", ";
    }
    out += s;
  }
  return out;
}

}   // namespace tractor_monitor
