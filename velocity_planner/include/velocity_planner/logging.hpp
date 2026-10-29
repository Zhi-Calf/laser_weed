#ifndef VELOCITY_PLANNER_LOGGING_H_
#define VELOCITY_PLANNER_LOGGING_H_

#include <cstddef>
#include <string>

namespace velocity_planner {

struct LogConfig {
  std::string logger_name         = "velocity_planner";       // 日志器名称
  std::string log_file_name       = "velocity_planner.log";   // 日志文件名
  std::string log_dir             = "/opt/agri/logs";         // 日志目录
  std::size_t max_file_size_bytes = 50U * 1024U * 1024U;      // 单文件大小上限
  std::size_t max_file_count      = 10;                       // 保留文件数量
  bool enable_console             = true;                     // 是否输出到控制台
  std::string log_level           = "info";                   // 日志级别
};

// 按配置初始化滚动文件日志和可选控制台输出。
void ConfigureLogging(const LogConfig& config);

}   // namespace velocity_planner

#endif   // VELOCITY_PLANNER_LOGGING_H_
