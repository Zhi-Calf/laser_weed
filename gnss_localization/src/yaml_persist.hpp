#ifndef GNSS_LOCALIZATION_YAML_PERSIST_H_
#define GNSS_LOCALIZATION_YAML_PERSIST_H_

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#include <yaml.h>

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace gnss_localization {

struct YamlUpdateReport {
  size_t duplicate_keys = 0;  // 目标映射内重复键的额外出现次数
  bool target_node_found = false;  // 是否找到目标节点
  bool mapping_found = false;  // 是否找到目标节点的 ros__parameters
  bool updates_valid = true;  // 待写入键和值是否安全
  bool yaml_validated = false;  // 修改后的文本是否通过 YAML 解析
  bool dir_sync_failed = false;  // 父目录 fsync 是否失败
  bool metadata_preserved = false;  // 是否保留原文件权限和所有者
};

enum class YamlPersistStatus {
  kPersisted,  // 已写回
  kUnchanged,  // 内容没有变化
  kReadFailed,  // 读取失败或读取不完整
  kInvalidStructure,  // 节点结构或修改结果无效
  kLockFailed,  // 获取写锁失败
  kWriteFailed,  // 落盘失败
};

namespace yaml_persist_detail {

struct MappingRange {
  bool target_node_found = false;  // 是否找到唯一目标节点
  bool mapping_found = false;  // 是否找到唯一参数映射
  bool valid = false;  // 参数映射是否可安全修改
  size_t mapping_line = 0;  // ros__parameters 所在行
  size_t begin = 0;  // 参数映射内容起始行
  size_t end = 0;  // 参数映射内容结束行
  size_t parameter_indent = 0;  // 直接参数键缩进
};

// 返回第一个非空白字符位置，全空白行返回 npos。
inline size_t FirstNonSpace(const std::string& line) {
  return line.find_first_not_of(" \t\r");
}

// 返回行首空白字符数。
inline size_t LeadingIndent(const std::string& line) {
  const size_t first = FirstNonSpace(line);
  return first == std::string::npos ? 0 : first;
}

// 判断空行或纯注释行。
inline bool IsIgnorable(const std::string& line) {
  const size_t first = FirstNonSpace(line);
  return first == std::string::npos || line[first] == '#';
}

// 查找引号外的 YAML 注释起点。
inline size_t FindCommentStart(const std::string& line, size_t start) {
  char quote = '\0';
  bool escaped = false;
  for (size_t i = start; i < line.size(); ++i) {
    const char character = line[i];
    if (quote == '\'') {
      if (character == '\'') {
        if (i + 1 < line.size() && line[i + 1] == '\'') {
          ++i;
        } else {
          quote = '\0';
        }
      }
      continue;
    }
    if (quote == '"') {
      if (escaped) {
        escaped = false;
      } else if (character == '\\') {
        escaped = true;
      } else if (character == '"') {
        quote = '\0';
      }
      continue;
    }
    if (character == '\'' || character == '"') {
      quote = character;
      continue;
    }
    if (character == '#' &&
        (i == 0 || line[i - 1] == ' ' || line[i - 1] == '\t')) {
      return i;
    }
  }
  return std::string::npos;
}

// 判断一行在指定缩进处是否为精确的 "key:" 映射键。
inline bool LineHasKey(const std::string& line, const std::string& key) {
  const size_t first = FirstNonSpace(line);
  if (first == std::string::npos ||
      line.compare(first, key.size(), key) != 0) {
    return false;
  }
  const size_t colon = first + key.size();
  if (colon >= line.size() || line[colon] != ':')
    return false;
  if (colon + 1 == line.size())
    return true;
  const char next = line[colon + 1];
  return next == ' ' || next == '\t' || next == '#' || next == '\r';
}

// 校验参数键为普通 YAML 标识符。
inline bool IsIdentifier(const std::string& text) {
  if (text.empty())
    return false;
  const auto is_alpha = [](char character) {
    return (character >= 'A' && character <= 'Z') ||
           (character >= 'a' && character <= 'z') ||
           character == '_';
  };
  const auto is_digit = [](char character) {
    return character >= '0' && character <= '9';
  };
  if (!is_alpha(text[0]))
    return false;
  for (size_t i = 1; i < text.size(); ++i) {
    if (!is_alpha(text[i]) && !is_digit(text[i]))
      return false;
  }
  return true;
}

// 校验待写入值是单个安全行，具体 YAML 语法由写后解析兜底。
inline bool IsSafeValue(const std::string& value) {
  return !value.empty() && value.find('\n') == std::string::npos &&
         value.find('\r') == std::string::npos &&
         value.find('\0') == std::string::npos;
}

// 在目标节点内定位唯一的 ros__parameters 映射范围。
inline MappingRange FindMappingRange(
    const std::vector<std::string>& lines, const std::string& node_name) {
  MappingRange range;
  if (!IsIdentifier(node_name))
    return range;

  std::vector<size_t> node_lines;
  for (size_t i = 0; i < lines.size(); ++i) {
    if (IsIgnorable(lines[i]) || LeadingIndent(lines[i]) != 0)
      continue;
    if (LineHasKey(lines[i], node_name))
      node_lines.push_back(i);
  }
  if (node_lines.size() != 1)
    return range;
  range.target_node_found = true;

  const size_t node_line = node_lines.front();
  std::vector<size_t> mapping_lines;
  for (size_t i = node_line + 1; i < lines.size(); ++i) {
    if (IsIgnorable(lines[i]))
      continue;
    const size_t indent = LeadingIndent(lines[i]);
    if (indent == 0)
      break;
    if (LineHasKey(lines[i], "ros__parameters"))
      mapping_lines.push_back(i);
  }
  if (mapping_lines.size() != 1)
    return range;

  const size_t mapping_line = mapping_lines.front();
  const size_t mapping_indent = LeadingIndent(lines[mapping_line]);
  const size_t after_colon =
      FirstNonSpace(lines[mapping_line]) + std::strlen("ros__parameters:");
  const size_t value_begin =
      lines[mapping_line].find_first_not_of(" \t", after_colon);
  const size_t comment_begin =
      FindCommentStart(lines[mapping_line], after_colon);
  const size_t content_end =
      comment_begin == std::string::npos
          ? (lines[mapping_line].empty() ||
                     lines[mapping_line].back() != '\r'
                 ? lines[mapping_line].size()
                 : lines[mapping_line].size() - 1)
          : comment_begin;
  if (value_begin != std::string::npos && value_begin < content_end) {
    return range;  // 内联映射暂不支持，避免猜测缩进
  }

  size_t range_end = lines.size();
  for (size_t i = mapping_line + 1; i < lines.size(); ++i) {
    if (IsIgnorable(lines[i]))
      continue;
    if (LeadingIndent(lines[i]) <= mapping_indent) {
      range_end = i;
      break;
    }
  }

  size_t parameter_indent = mapping_indent + 2;
  for (size_t i = mapping_line + 1; i < range_end; ++i) {
    if (IsIgnorable(lines[i]))
      continue;
    const size_t indent = LeadingIndent(lines[i]);
    if (indent > mapping_indent) {
      parameter_indent = indent;
      break;
    }
  }

  range.mapping_found = true;
  range.valid = true;
  range.mapping_line = mapping_line;
  range.begin = mapping_line + 1;
  range.end = range_end;
  range.parameter_indent = parameter_indent;
  return range;
}

// 解析并验证目标节点、参数映射以及所有待写键。
inline bool ValidateTargetYaml(
    const std::vector<std::string>& lines, const std::string& node_name,
    const std::vector<std::pair<std::string, std::string>>& updates) {
  std::string text;
  for (const std::string& line : lines) {
    text.append(line);
    text.push_back('\n');
  }

  yaml_parser_t parser{};
  if (!yaml_parser_initialize(&parser))
    return false;
  yaml_parser_set_input_string(
      &parser, reinterpret_cast<const unsigned char*>(text.data()),
      text.size());

  yaml_document_t document{};
  bool valid = yaml_parser_load(&parser, &document) != 0;
  if (valid) {
    yaml_node_t* root = yaml_document_get_root_node(&document);
    valid = root != nullptr && root->type == YAML_MAPPING_NODE;

    yaml_node_t* node_mapping = nullptr;
    if (valid) {
      size_t matches = 0;
      const yaml_char_t* key =
          reinterpret_cast<const yaml_char_t*>(node_name.data());
      for (yaml_node_pair_t* pair = root->data.mapping.pairs.start;
           pair != root->data.mapping.pairs.top; ++pair) {
        yaml_node_t* key_node = yaml_document_get_node(&document, pair->key);
        if (key_node != nullptr && key_node->type == YAML_SCALAR_NODE &&
            key_node->data.scalar.length == node_name.size() &&
            std::memcmp(key_node->data.scalar.value, key,
                        node_name.size()) == 0) {
          node_mapping = yaml_document_get_node(&document, pair->value);
          ++matches;
        }
      }
      valid = matches == 1 && node_mapping != nullptr &&
              node_mapping->type == YAML_MAPPING_NODE;
    }

    yaml_node_t* parameters = nullptr;
    if (valid) {
      size_t matches = 0;
      constexpr char kParameters[] = "ros__parameters";
      for (yaml_node_pair_t* pair = node_mapping->data.mapping.pairs.start;
           pair != node_mapping->data.mapping.pairs.top; ++pair) {
        yaml_node_t* key_node = yaml_document_get_node(&document, pair->key);
        if (key_node != nullptr && key_node->type == YAML_SCALAR_NODE &&
            key_node->data.scalar.length == sizeof(kParameters) - 1 &&
            std::memcmp(key_node->data.scalar.value, kParameters,
                        sizeof(kParameters) - 1) == 0) {
          parameters = yaml_document_get_node(&document, pair->value);
          ++matches;
        }
      }
      valid = matches == 1 && parameters != nullptr &&
              parameters->type == YAML_MAPPING_NODE;
    }

    for (const auto& [key_name, value] : updates) {
      if (!valid)
        break;
      size_t matches = 0;
      yaml_node_t* value_node = nullptr;
      for (yaml_node_pair_t* pair = parameters->data.mapping.pairs.start;
           pair != parameters->data.mapping.pairs.top; ++pair) {
        yaml_node_t* key_node = yaml_document_get_node(&document, pair->key);
        if (key_node != nullptr && key_node->type == YAML_SCALAR_NODE &&
            key_node->data.scalar.length == key_name.size() &&
            std::memcmp(key_node->data.scalar.value, key_name.data(),
                        key_name.size()) == 0) {
          value_node = yaml_document_get_node(&document, pair->value);
          ++matches;
        }
      }
      valid = matches >= 1 && value_node != nullptr &&
              value_node->type == YAML_SCALAR_NODE && !value.empty();
    }
    yaml_document_delete(&document);
  }

  if (valid) {
    yaml_document_t extra_document{};
    valid = yaml_parser_load(&parser, &extra_document) != 0;
    if (valid) {
      if (yaml_document_get_root_node(&extra_document) != nullptr)
        valid = false;
      yaml_document_delete(&extra_document);
    }
  }
  yaml_parser_delete(&parser);
  return valid;
}

// 解析路径末尾的软链，保护配置文件和软链本身。
inline bool ResolveTargetPath(const std::string& path, std::string* resolved) {
  if (resolved == nullptr)
    return false;
  std::filesystem::path current(path);
  for (int depth = 0; depth < 16; ++depth) {
    std::error_code error;
    if (!std::filesystem::is_symlink(current, error)) {
      if (error == std::errc::no_such_file_or_directory) {
        error.clear();
      }
      if (error)
        return false;
      *resolved = current.string();
      return !resolved->empty();
    }
    const std::filesystem::path link =
        std::filesystem::read_symlink(current, error);
    if (error)
      return false;
    current = link.is_absolute() ? link : current.parent_path() / link;
    current = current.lexically_normal();
  }
  return false;
}

// 持有跨进程排他锁。
class FileLock {
 public:
  explicit FileLock(std::string path) : path_(std::move(path)) {}

  ~FileLock() {
    if (fd_ >= 0)
      ::close(fd_);
  }

  FileLock(const FileLock&) = delete;
  FileLock& operator=(const FileLock&) = delete;

  bool Acquire() {
    fd_ = ::open(path_.c_str(),
                 O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (fd_ < 0)
      return false;
    while (::flock(fd_, LOCK_EX) != 0) {
      if (errno != EINTR)
        return false;
    }
    return true;
  }

 private:
  std::string path_;  // 锁文件路径
  int fd_{-1};  // 锁文件描述符
};

}  // yaml_persist_detail 命名空间

// 修改目标节点 ros__parameters 内指定参数，保留其它节点、缩进和注释。
inline bool ApplyYamlKeyUpdates(
    std::vector<std::string>& lines, const std::string& node_name,
    const std::vector<std::pair<std::string, std::string>>& updates,
    YamlUpdateReport* report = nullptr) {
  YamlUpdateReport local_report;
  YamlUpdateReport* result = report != nullptr ? report : &local_report;
  *result = YamlUpdateReport();

  for (const auto& [key, value] : updates) {
    if (!yaml_persist_detail::IsIdentifier(key) ||
        !yaml_persist_detail::IsSafeValue(value)) {
      result->updates_valid = false;
      return false;
    }
  }

  const yaml_persist_detail::MappingRange range =
      yaml_persist_detail::FindMappingRange(lines, node_name);
  result->target_node_found = range.target_node_found;
  result->mapping_found = range.mapping_found;
  if (!range.valid)
    return false;

  bool modified = false;
  size_t duplicates = 0;
  std::vector<std::pair<std::string, std::string>> missing;
  for (const auto& [key, value] : updates) {
    size_t hits = 0;
    for (size_t i = range.begin; i < range.end; ++i) {
      std::string& line = lines[i];
      if (yaml_persist_detail::IsIgnorable(line) ||
          yaml_persist_detail::LeadingIndent(line) !=
              range.parameter_indent ||
          !yaml_persist_detail::LineHasKey(line, key)) {
        continue;
      }
      ++hits;

      const std::string before = line;
      const size_t first = yaml_persist_detail::FirstNonSpace(line);
      const size_t after_colon = first + key.size() + 1;
      const size_t content_end =
          (!line.empty() && line.back() == '\r') ? line.size() - 1
                                                  : line.size();
      const size_t comment =
          yaml_persist_detail::FindCommentStart(line, after_colon);
      const size_t value_end =
          comment == std::string::npos ? content_end : comment;
      const size_t value_begin =
          line.find_first_not_of(" \t", after_colon);
      size_t value_last = value_end;
      while (value_last > after_colon &&
             (line[value_last - 1] == ' ' ||
              line[value_last - 1] == '\t')) {
        --value_last;
      }

      if (value_begin != std::string::npos && value_begin < value_last) {
        line.replace(value_begin, value_last - value_begin, value);
      } else {
        line.insert(after_colon, " " + value);
      }
      if (line != before)
        modified = true;
    }
    if (hits == 0) {
      missing.emplace_back(key, value);
    } else if (hits > 1) {
      duplicates += hits - 1;
    }
  }

  if (!missing.empty()) {
    size_t insert_at = range.mapping_line + 1;
    for (size_t i = range.begin; i < range.end; ++i) {
      if (!yaml_persist_detail::IsIgnorable(lines[i]))
        insert_at = i + 1;
    }
    const bool crlf = !lines[range.mapping_line].empty() &&
                      lines[range.mapping_line].back() == '\r';
    const std::string suffix = crlf ? "\r" : "";
    for (auto it = missing.rbegin(); it != missing.rend(); ++it) {
      const std::string new_line =
          std::string(range.parameter_indent, ' ') + it->first + ": " +
          it->second + suffix;
      lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(insert_at),
                   new_line);
    }
    modified = true;
  }

  result->duplicate_keys = duplicates;
  return modified;
}

// 写入同目录唯一临时文件，保留原权限和所有者，再原子替换目标。
inline bool WriteFileAtomically(
    const std::string& path, const std::vector<std::string>& lines,
    YamlUpdateReport* report = nullptr) {
  if (report != nullptr) {
    report->dir_sync_failed = false;
    report->metadata_preserved = false;
  }

  std::string target;
  if (!yaml_persist_detail::ResolveTargetPath(path, &target))
    return false;

  struct stat original {};
  const bool target_exists = ::stat(target.c_str(), &original) == 0;
  if (!target_exists && errno != ENOENT)
    return false;

  std::string pattern = target + ".tmp.XXXXXX";
  std::vector<char> temp_path(pattern.begin(), pattern.end());
  temp_path.push_back('\0');
  const int fd = ::mkstemp(temp_path.data());
  if (fd < 0)
    return false;
  ::fcntl(fd, F_SETFD, FD_CLOEXEC);

  auto fail = [&]() {
    ::close(fd);
    ::unlink(temp_path.data());
    return false;
  };

  if (target_exists) {
    if (::fchown(fd, original.st_uid, original.st_gid) != 0 ||
        ::fchmod(fd, original.st_mode & 07777) != 0) {
      return fail();
    }
  } else if (::fchmod(fd, 0644) != 0) {
    return fail();
  }

  for (const std::string& line : lines) {
    const std::string content = line + "\n";
    size_t written = 0;
    while (written < content.size()) {
      const ssize_t count =
          ::write(fd, content.data() + written, content.size() - written);
      if (count < 0) {
        if (errno == EINTR)
          continue;
        return fail();
      }
      if (count == 0)
        return fail();
      written += static_cast<size_t>(count);
    }
  }

  if (::fsync(fd) != 0)
    return fail();
  if (::close(fd) != 0) {
    ::unlink(temp_path.data());
    return false;
  }

  std::error_code rename_error;
  std::filesystem::rename(temp_path.data(), target, rename_error);
  if (rename_error) {
    ::unlink(temp_path.data());
    return false;
  }
  if (report != nullptr)
    report->metadata_preserved = target_exists;

  const std::filesystem::path parent =
      std::filesystem::path(target).parent_path();
  const std::string directory =
      parent.empty() ? std::string(".") : parent.string();
  const int directory_fd =
      ::open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (directory_fd < 0) {
    if (report != nullptr)
      report->dir_sync_failed = true;
    return true;
  }
  if (::fsync(directory_fd) != 0 && report != nullptr)
    report->dir_sync_failed = true;
  ::close(directory_fd);
  return true;
}

// 在跨进程锁保护下读取、修改、验证并原子写回目标节点参数。
inline YamlPersistStatus PersistYamlKeys(
    const std::string& path, const std::string& node_name,
    const std::vector<std::pair<std::string, std::string>>& updates,
    YamlUpdateReport* report = nullptr) {
  YamlUpdateReport local_report;
  YamlUpdateReport* result = report != nullptr ? report : &local_report;
  *result = YamlUpdateReport();

  std::string target;
  if (!yaml_persist_detail::ResolveTargetPath(path, &target))
    return YamlPersistStatus::kInvalidStructure;

  yaml_persist_detail::FileLock lock(target + ".lock");
  if (!lock.Acquire())
    return YamlPersistStatus::kLockFailed;

  std::ifstream input(target, std::ios::binary);
  if (!input.is_open())
    return YamlPersistStatus::kReadFailed;

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(input, line))
    lines.push_back(line);
  if (input.bad())
    return YamlPersistStatus::kReadFailed;

  if (!ApplyYamlKeyUpdates(lines, node_name, updates, result)) {
    if (!result->updates_valid || !result->target_node_found ||
        !result->mapping_found) {
      return YamlPersistStatus::kInvalidStructure;
    }
    return YamlPersistStatus::kUnchanged;
  }
  if (!yaml_persist_detail::ValidateTargetYaml(lines, node_name, updates))
    return YamlPersistStatus::kInvalidStructure;
  result->yaml_validated = true;

  if (!WriteFileAtomically(target, lines, result))
    return YamlPersistStatus::kWriteFailed;
  return YamlPersistStatus::kPersisted;
}

}  // gnss_localization 命名空间

#endif  // 防止重复包含
