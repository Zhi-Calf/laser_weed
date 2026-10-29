#include "global_path_planner/logging.hpp"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <system_error>
#include <vector>

namespace global_path_planner {

namespace {

constexpr const char* kLogPattern = "[%Y-%m-%d %H:%M:%S.%e] [%l] [%s:%#] %v";

// 展开路径中的 ~ 和环境变量。
std::string ExpandPath(std::string path) {
  if (path.rfind("~/", 0) == 0) {
    const char* home = std::getenv("HOME");
    if (home != nullptr) {
      path = std::string(home) + path.substr(1);
    }
  }

  std::string expanded;
  expanded.reserve(path.size());
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (path[i] != '$') {
      expanded.push_back(path[i]);
      continue;
    }

    std::size_t end = i + 1;
    while (end < path.size() && (std::isalnum(static_cast<unsigned char>(path[end])) != 0 || path[end] == '_')) {
      ++end;
    }
    const std::string name = path.substr(i + 1, end - i - 1);
    if (!name.empty()) {
      const char* value = std::getenv(name.c_str());
      if (value != nullptr) {
        expanded.append(value);
      }
    }
    i = end - 1;
  }
  return expanded;
}

// 返回可写日志目录，优先使用配置目录。
std::string ResolveLogDir(const std::string& configured) {
  std::error_code error;
  const std::string preferred = ExpandPath(configured);
  if (!preferred.empty()) {
    std::filesystem::create_directories(preferred, error);
    if (!error && access(preferred.c_str(), W_OK) == 0) {
      return preferred;
    }
  }

  const char* ros_home = std::getenv("ROS_HOME");
  const char* home     = std::getenv("HOME");
  std::string fallback;
  if (ros_home != nullptr && *ros_home != '\0') {
    fallback = std::string(ros_home) + "/logs";
  } else if (home != nullptr && *home != '\0') {
    fallback = std::string(home) + "/.ros/logs";
  } else {
    return "";
  }

  std::filesystem::create_directories(fallback, error);
  if (error || access(fallback.c_str(), W_OK) != 0) {
    return "";
  }
  return fallback;
}

}   // namespace

// 按配置初始化滚动文件日志和可选控制台输出。
// 输入:
//   config : 日志器名称、目录、文件数量、级别和输出开关
void ConfigureLogging(const LogConfig& config) {
  const std::string resolved_dir = ResolveLogDir(config.log_dir);
  std::vector<spdlog::sink_ptr> sinks;
  if (!resolved_dir.empty()) {
    const auto log_path = std::filesystem::path(resolved_dir) / config.log_file_name;
    sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_path.string(), config.max_file_size_bytes, config.max_file_count));
  }
  if (config.enable_console || sinks.empty()) {
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  }

  auto logger = std::make_shared<spdlog::logger>(config.logger_name, sinks.begin(), sinks.end());
  logger->set_pattern(kLogPattern);

  const spdlog::level::level_enum level = spdlog::level::from_str(config.log_level);
  if (level == spdlog::level::off && config.log_level != "off") {
    logger->set_level(spdlog::level::info);
    logger->warn("无法识别的日志级别 \"{}\"，已回退到 info", config.log_level);
  } else {
    logger->set_level(level);
  }

  spdlog::set_default_logger(logger);
  spdlog::flush_on(spdlog::level::warn);
  spdlog::flush_every(std::chrono::seconds(3));

  if (resolved_dir.empty()) {
    SPDLOG_WARN("日志目录 \"{}\" 不可写，文件日志已禁用", config.log_dir);
  } else if (resolved_dir != ExpandPath(config.log_dir)) {
    SPDLOG_WARN("日志目录 \"{}\" 不可写，已回退到 \"{}\"", config.log_dir, resolved_dir);
  }
}

}   // namespace global_path_planner
