#pragma once

// 本包两个节点 (mqtt_bridge / http_bridge) 共用的 spdlog 滚动日志配置。
// 配置的目录不可写时退回 $ROS_HOME/logs (~/.ros/logs)。

#include <cstddef>
#include <string>

namespace weeder_comm_gateway {

// 展开开头的 ~ 与路径里的 $VAR
std::string ExpandPath(std::string path);

// 日志写到 <log_dir>/<name>.log。没有任何可写目录时抛 std::runtime_error (调用方退回仅终端输出)。
void ConfigureLogger(const std::string& name, const std::string& log_dir, size_t max_file_size_bytes, size_t max_file_count, bool enable_console, const std::string& log_level);

}   // namespace weeder_comm_gateway
