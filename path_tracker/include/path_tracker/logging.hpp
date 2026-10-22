#pragma once

#include <cstddef>
#include <string>

namespace path_tracker {

struct LogConfig {
  std::string log_dir             = "~/.ros/logs";         // 日志目录，支持 ~ 和环境变量
  std::size_t max_file_size_bytes = 50U * 1024U * 1024U;   // 单文件大小上限
  std::size_t max_file_count      = 10;                    // 保留文件数量
  bool enable_console             = true;                  // 是否输出到控制台
  std::string log_level           = "info";                // 日志级别
};

void ConfigureLogging(const LogConfig& config);

}   // namespace path_tracker
