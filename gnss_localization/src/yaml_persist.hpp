#pragma once

// YAML 参数写回: 把运行期改过的参数落盘, 重启后仍是新值。
//
// ⚠ 本文件与 **tractor_calibration/src/yaml_persist.hpp 互为镜像**: 同一套规则、同一套分层与命名
//   (YamlUpdateReport / ApplyYamlKeyUpdates / WriteFileAtomically / YamlPersistStatus /
//   PersistYamlKeys), 只有命名空间不同 —— 两份改过之后 diff 一下, 别让它们悄悄分叉。
//   (原先的第三份 global_path_planner/src/yaml_persist.hpp 已随"运行期可改的参数全删 + 参数持久化
//   一并删除"移除 —— 规划器不再写任何 yaml 文件, 见它的 src/global_path_planner.cpp 里的说明。)
//   本节点原来把这两段内联在 gnss_driver.cpp 里, 这次按上面这套分层抽出来, 节点层
//   `persistParamsToYaml` 就只剩下"double -> 字符串 + 日志 + 结果映射"。
// 规则与 tractor_calibration 的 persistCalibToYaml **完全一致** —— 现场同一份 yaml 可能被多个节点
// 写, 各写各的规则迟早互相踩 (一个保留注释、另一个重排全文)。四条规则:
//   * 逐行替换: 只认"行首 (允许前置空白) 的 key:", 跳过纯注释行 —— 绝不误改注释里提到的参数名;
//   * 同一键出现多行时**全部**替换, 并把重复次数回报: rcl_yaml_param_parser 对重复键取最后
//     一个, 只改第一处会出现"写文件成功、重启后生效值却没变";
//   * 只有键没有值 (例如 "key:   # 待标定") 视为键已存在, 把值填到冒号后, 不再追加 (否则写出重复键);
//   * 键完全没出现过才追加, 且追加到 **ros__parameters 映射内** —— 追加到文件末尾可能落在别的
//     节点段里, 现象同样是"写成功但没生效" (回报 mapping_found=false 让调用方告警)。
// 落盘是**原子**的: 同目录临时文件 -> fsync -> rename 覆盖目标 —— 直接 trunc 原文件逐行写时,
// 写到一半掉电/磁盘满会留下被截断的 yaml, 下次启动读到的就是坏配置 (参数全丢)。
//
// 三层分工 (调用方只用第三层; 前两层是纯函数, 不打日志, 事实通过 report 回传):
//   ① ApplyYamlKeyUpdates —— 纯文本: 逐行替换 / 填值 / 追加, 返回"文本是否被改动过";
//   ② WriteFileAtomically —— 文件: 临时文件 + fsync + rename, 返回是否落盘成功;
//   ③ PersistYamlKeys     —— 读 -> ① -> ②, 返回 YamlPersistStatus (一步到位, 节点层用这个)。
//
// 放在 src/ 下 (不进 install): 与 global_path_planner 的同名文件一样, 只给本包的节点用。

#include <fcntl.h>
#include <unistd.h>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace gnss_localization {

// 写回的附带信息 (供调用方告警; 纯函数本身不打日志)。
struct YamlUpdateReport {
  size_t duplicate_keys = 0;       // "同一键出现多行"的额外出现次数 (> 0 要告警)
  bool mapping_found    = true;    // 追加缺失键时是否找到 ros__parameters (false = 追加到末尾, 可能不生效)
  bool dir_sync_failed  = false;   // 落盘后父目录 fsync 失败 (内容已写入, 只有掉电才可能丢这次 rename)
};

// 把 (键, 值) 逐行写回 lines, 保留缩进与注释。
// 返回 true = 文本被改动过 (调用方据此决定要不要落盘; 等值替换不算改动)。
inline bool ApplyYamlKeyUpdates(std::vector<std::string>& lines, const std::vector<std::pair<std::string, std::string>>& updates, YamlUpdateReport* report = nullptr) {
  bool modified     = false;
  size_t duplicates = 0;
  bool mapping_ok   = true;

  for (const auto& [key, value] : updates) {
    const std::string pattern = key + ":";
    size_t hits               = 0;   // 该键在文件里出现的次数
    for (auto& ln : lines) {
      const size_t first = ln.find_first_not_of(" \t");
      if (first == std::string::npos || ln[first] == '#') {
        continue;   // 空行 / 纯注释行
      }
      if (ln.find(pattern, first) != first) {
        continue;   // "key:" 必须出现在行首 (允许前置空白)
      }
      ++hits;

      // 值区间 = ':' 之后第一个非空白 到 '#' 或行尾之前的最后一个非空白
      const std::string before = ln;   // 用于判断文本是否真的变了 (等值替换不落盘)
      const size_t val_begin   = ln.find_first_not_of(" \t", first + pattern.size());
      const size_t hash        = ln.find('#', first + pattern.size());
      const size_t val_end     = (hash == std::string::npos) ? ln.size() : hash;
      size_t val_last          = val_end;
      while (val_last > first + pattern.size() && (ln[val_last - 1] == ' ' || ln[val_last - 1] == '\t')) {
        --val_last;
      }

      if (val_begin != std::string::npos && val_begin < val_last) {
        ln.replace(val_begin, val_last - val_begin, value);
      } else {
        // 只有键没有值 (空值 / 值被注释掉): 把值插到 ':' 之后, 原注释保持不动。
        // 冒号后紧跟 '#' 时补一个空格, 否则会拼出 "0.123# 注释" 这种非法标量
        // (YAML 里 '#' 前面必须有空白才算注释开始)。
        const bool hash_right_after_colon = (hash == first + pattern.size());
        ln.insert(first + pattern.size(), hash_right_after_colon ? (" " + value + " ") : (" " + value));
      }
      if (ln != before) {
        modified = true;
      }
    }

    if (hits == 0) {
      // 键不存在: 插到 **ros__parameters: 映射内** (而不是文件末尾) —— 文件末尾可能是另一个
      // 节点段或顶层键, 写进去的键不会被解析成本节点的参数。
      // 本函数按"单节点 yaml"定位: 文件里第一个 ros__parameters 即目标映射。
      size_t append_at                  = lines.size();   // 兜底: 找不到映射才追加到末尾
      static const std::string kMapping = "ros__parameters";
      for (size_t i = 0; i < lines.size(); ++i) {
        const size_t first = lines[i].find_first_not_of(" \t");
        if (first != std::string::npos && lines[i].compare(first, kMapping.size(), kMapping) == 0) {
          append_at = i + 1;
          break;
        }
      }
      if (append_at == lines.size()) {
        mapping_ok = false;
      }
      lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(append_at), "    " + key + ": " + value);
      modified = true;
    } else if (hits > 1) {
      duplicates += hits - 1;
    }
  }

  if (report != nullptr) {
    report->duplicate_keys = duplicates;
    report->mapping_found  = mapping_ok;
  }
  return modified;
}

// 原子落盘: 写同目录临时文件 -> fsync -> rename 覆盖目标 (再 fsync 父目录, 让目录项也落盘)。
// 直接用 trunc 打开原文件逐行写时, 写到一半掉电/磁盘满会留下被截断的 yaml, 下次启动
// 读到的就是坏配置 (参数全丢)。同文件系统内 rename 是原子的: 结果要么是旧内容, 要么
// 是新内容, 不存在中间态。fsync 用来保证 rename 之前数据已落盘, 否则掉电后可能
// rename 到一个空文件。
inline bool WriteFileAtomically(const std::string& path, const std::vector<std::string>& lines, YamlUpdateReport* report = nullptr) {
  const std::string tmp = path + ".tmp";
  {
    std::ofstream ofs(tmp, std::ios::trunc);
    if (!ofs.is_open()) {
      return false;
    }
    for (size_t i = 0; i < lines.size(); ++i) {
      ofs << lines[i];
      if (i + 1 < lines.size()) {
        ofs << '\n';
      }
    }
    // 末尾补一个换行: 文本文件以换行结尾才算完整的一行。不补的话, 下次用编辑器/脚本追加
    // 参数时会粘到最后一行上, 拼出 "log_level: \"info\"    new_key: 1" 这种非法 YAML。
    ofs << '\n';
    ofs.flush();
    if (!ofs) {
      std::error_code rm_ec;
      std::filesystem::remove(tmp, rm_ec);
      return false;
    }
  }

  const int fd = ::open(tmp.c_str(), O_RDONLY);
  if (fd < 0) {
    std::error_code rm_ec;
    std::filesystem::remove(tmp, rm_ec);   // 别把 .tmp 留在现场误导排查
    return false;
  }
  const int fsync_rc = ::fsync(fd);
  ::close(fd);
  if (fsync_rc != 0) {
    std::error_code rm_ec;
    std::filesystem::remove(tmp, rm_ec);
    return false;
  }

  std::error_code ec;
  std::filesystem::rename(tmp, path, ec);
  if (ec) {
    std::error_code rm_ec;
    std::filesystem::remove(tmp, rm_ec);
    return false;
  }

  // rename 本身原子, 但**目录项**也要落盘: 否则掉电后可能回退到"没发生这次 rename"
  // (旧内容还在 = 丢一次写入, 但不会出现半截文件)。这一步失败不影响返回值, 只回报给调用方。
  {
    const std::filesystem::path parent = std::filesystem::path(path).parent_path();
    const std::string dir              = parent.empty() ? std::string(".") : parent.string();
    const int dir_fd                   = ::open(dir.c_str(), O_RDONLY | O_DIRECTORY);
    if (dir_fd >= 0) {
      if (::fsync(dir_fd) != 0 && report != nullptr) {
        report->dir_sync_failed = true;
      }
      ::close(dir_fd);
    }
  }
  return true;
}

// 写回的结果 (调用方据此决定日志级别与给客户端的 reason)。
enum class YamlPersistStatus {
  kPersisted,     // 已写回
  kUnchanged,     // 值与文件里已有内容一致 -> 没重写 (少一次落盘窗口); 对调用方同样算成功
  kReadFailed,    // 读不到目标文件
  kWriteFailed,   // 落盘失败
};

// 把 updates 写回 path 指向的 yaml 文件 (读 -> 逐行替换 -> 原子替换)。
inline YamlPersistStatus PersistYamlKeys(const std::string& path, const std::vector<std::pair<std::string, std::string>>& updates, YamlUpdateReport* report = nullptr) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    return YamlPersistStatus::kReadFailed;
  }
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(ifs, line)) {
    lines.push_back(line);
  }
  ifs.close();

  if (!ApplyYamlKeyUpdates(lines, updates, report)) {
    return YamlPersistStatus::kUnchanged;
  }
  if (!WriteFileAtomically(path, lines, report)) {
    return YamlPersistStatus::kWriteFailed;
  }
  return YamlPersistStatus::kPersisted;
}

}   // namespace gnss_localization
