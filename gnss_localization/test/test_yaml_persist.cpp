// 验证目标节点内的 YAML 参数写回、结构校验、权限保持和并发安全。
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include <sys/stat.h>

#include "yaml_persist.hpp"

using gnss_localization::ApplyYamlKeyUpdates;
using gnss_localization::PersistYamlKeys;
using gnss_localization::YamlPersistStatus;
using gnss_localization::YamlUpdateReport;

namespace {

class TempYaml {
 public:
  TempYaml(const std::string& name, const std::string& content)
      : path_("/tmp/megatrak_yaml_persist_gnss_" + name + ".yaml") {
    Write(content);
  }

  ~TempYaml() {
    std::error_code error;
    std::filesystem::remove(path_, error);
    std::filesystem::remove(path_ + ".lock", error);
    for (const auto& entry :
         std::filesystem::directory_iterator("/tmp", error)) {
      if (error)
        break;
      const std::string filename = entry.path().filename().string();
      if (filename.rfind(
              std::filesystem::path(path_).filename().string() + ".tmp.",
              0) == 0) {
        std::filesystem::remove(entry.path(), error);
      }
    }
  }

  TempYaml(const TempYaml&) = delete;
  TempYaml& operator=(const TempYaml&) = delete;

  void Write(const std::string& content) const {
    std::ofstream output(path_, std::ios::binary | std::ios::trunc);
    output << content;
  }

  const std::string& path() const { return path_; }

  std::vector<std::string> ReadLines() const {
    std::vector<std::string> lines;
    std::ifstream input(path_);
    std::string line;
    while (std::getline(input, line))
      lines.push_back(line);
    return lines;
  }

  std::string ReadText() const {
    std::ifstream input(path_, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(input),
                       std::istreambuf_iterator<char>());
  }

 private:
  std::string path_;  // 临时文件路径
};

const char* kSampleYaml =
    "gnss_driver:\n"
    "  ros__parameters:\n"
    "    # 标定值 (运行期改会写回本文件)\n"
    "    calib_offset_x: 0.0\n"
    "    calib_offset_y: 0.0        # 行尾注释要保留\n"
    "    calib_offset_heading: 0.0\n"
    "    calib_offset_max_xy: 2.5\n"
    "    startup_warmup_sec: 30.0\n";

}  // 匿名命名空间

TEST(YamlPersist, ReplacesValueKeepingCommentsAndOtherLines) {
  const TempYaml file("replace", kSampleYaml);
  YamlUpdateReport report;

  EXPECT_EQ(PersistYamlKeys(file.path(), "gnss_driver",
                            {{"calib_offset_x", "1.250000"}}, &report),
            YamlPersistStatus::kPersisted);
  EXPECT_EQ(report.duplicate_keys, 0u);
  EXPECT_TRUE(report.target_node_found);
  EXPECT_TRUE(report.mapping_found);
  EXPECT_TRUE(report.yaml_validated);
  EXPECT_TRUE(report.metadata_preserved);
  EXPECT_FALSE(report.dir_sync_failed);

  const std::vector<std::string> lines = file.ReadLines();
  ASSERT_GE(lines.size(), 8u);
  EXPECT_EQ(lines[0], "gnss_driver:");
  EXPECT_EQ(lines[1], "  ros__parameters:");
  EXPECT_EQ(lines[2], "    # 标定值 (运行期改会写回本文件)");
  EXPECT_EQ(lines[3], "    calib_offset_x: 1.250000");
  EXPECT_EQ(lines[4],
            "    calib_offset_y: 0.0        # 行尾注释要保留");
  EXPECT_EQ(lines[5], "    calib_offset_heading: 0.0");
  EXPECT_EQ(lines[6], "    calib_offset_max_xy: 2.5");
  EXPECT_EQ(lines[7], "    startup_warmup_sec: 30.0");
}

TEST(YamlPersist, ReplacesEveryDuplicateInTargetAndReportsThem) {
  const TempYaml file(
      "duplicate",
      "gnss_driver:\n"
      "  ros__parameters:\n"
      "    calib_offset_x: 0.0\n"
      "    calib_offset_y: 0.0\n"
      "    calib_offset_x: 0.5\n");
  YamlUpdateReport report;

  EXPECT_EQ(PersistYamlKeys(file.path(), "gnss_driver",
                            {{"calib_offset_x", "1.250000"}}, &report),
            YamlPersistStatus::kPersisted);
  EXPECT_EQ(report.duplicate_keys, 1u);

  const std::vector<std::string> lines = file.ReadLines();
  EXPECT_EQ(lines[2], "    calib_offset_x: 1.250000");
  EXPECT_EQ(lines[4], "    calib_offset_x: 1.250000");
}

TEST(YamlPersist, FillsKeyWithoutValueInsteadOfAppending) {
  const TempYaml file(
      "empty_value",
      "gnss_driver:\n"
      "  ros__parameters:\n"
      "    calib_offset_heading:      # 现场标定值\n");

  EXPECT_EQ(PersistYamlKeys(
                file.path(), "gnss_driver",
                {{"calib_offset_heading", "0.005000"}}),
            YamlPersistStatus::kPersisted);
  const std::vector<std::string> lines = file.ReadLines();
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_EQ(lines[2],
            "    calib_offset_heading: 0.005000      # 现场标定值");
}

TEST(YamlPersist, AppendsMissingKeyInsideTargetNode) {
  const TempYaml file(
      "append",
      "ntrip_client:\n"
      "  ros__parameters:\n"
      "    mount_point: rtcm32\n"
      "\n"
      "gnss_driver:\n"
      "  ros__parameters:\n"
      "    calib_offset_x: 0.0\n");
  YamlUpdateReport report;

  EXPECT_EQ(PersistYamlKeys(
                file.path(), "gnss_driver",
                {{"calib_offset_heading", "0.010000"}}, &report),
            YamlPersistStatus::kPersisted);
  EXPECT_TRUE(report.target_node_found);
  EXPECT_TRUE(report.mapping_found);

  const std::vector<std::string> lines = file.ReadLines();
  ASSERT_EQ(lines.size(), 8u);
  EXPECT_EQ(lines[2], "    mount_point: rtcm32");
  EXPECT_EQ(lines[6], "    calib_offset_x: 0.0");
  EXPECT_EQ(lines[7], "    calib_offset_heading: 0.010000");
}

TEST(YamlPersist, OnlyUpdatesTargetNode) {
  const TempYaml file(
      "scope",
      "other_node:\n"
      "  ros__parameters:\n"
      "    calib_offset_x: 9.0\n"
      "gnss_driver:\n"
      "  ros__parameters:\n"
      "    calib_offset_x: 0.0\n");

  EXPECT_EQ(PersistYamlKeys(file.path(), "gnss_driver",
                            {{"calib_offset_x", "1.250000"}}),
            YamlPersistStatus::kPersisted);
  const std::vector<std::string> lines = file.ReadLines();
  EXPECT_EQ(lines[2], "    calib_offset_x: 9.0");
  EXPECT_EQ(lines[5], "    calib_offset_x: 1.250000");
}

TEST(YamlPersist, PreservesHashInsideQuotedValue) {
  const TempYaml file(
      "quoted_hash",
      "gnss_driver:\n"
      "  ros__parameters:\n"
      "    calib_offset_x: \"text#still-value\" # real-comment\n");

  EXPECT_EQ(PersistYamlKeys(file.path(), "gnss_driver",
                            {{"calib_offset_x", "1.25"}}),
            YamlPersistStatus::kPersisted);
  EXPECT_EQ(file.ReadLines()[2],
            "    calib_offset_x: 1.25 # real-comment");
}

TEST(YamlPersist, PreservesCrlfAndInsertsCrlfLine) {
  const TempYaml file(
      "crlf",
      "gnss_driver:\r\n"
      "  ros__parameters:\r\n"
      "    calib_offset_x: 0.0\r\n"
      "\r\n");

  EXPECT_EQ(PersistYamlKeys(file.path(), "gnss_driver",
                            {{"calib_offset_y", "1.0"}}),
            YamlPersistStatus::kPersisted);
  const std::string text = file.ReadText();
  EXPECT_NE(text.find("calib_offset_x: 0.0\r\n"), std::string::npos);
  EXPECT_NE(text.find("calib_offset_y: 1.0\r\n"), std::string::npos);
}

TEST(YamlPersist, RejectsMissingTargetOrMappingWithoutChangingFile) {
  const std::string no_mapping =
      "gnss_driver:\n"
      "  foo: 1\n";
  const TempYaml mapping_file("no_mapping", no_mapping);
  const TempYaml missing_file("missing_node", "other_node: {}\n");

  EXPECT_EQ(PersistYamlKeys(mapping_file.path(), "gnss_driver",
                            {{"calib_offset_x", "1.0"}}),
            YamlPersistStatus::kInvalidStructure);
  EXPECT_EQ(mapping_file.ReadText(), no_mapping);

  EXPECT_EQ(PersistYamlKeys(missing_file.path(), "gnss_driver",
                            {{"calib_offset_x", "1.0"}}),
            YamlPersistStatus::kInvalidStructure);
  EXPECT_EQ(missing_file.ReadText(), "other_node: {}\n");
}

TEST(YamlPersist, RejectsInvalidYamlWithoutChangingFile) {
  const std::string original =
      "gnss_driver:\n"
      "  ros__parameters:\n"
      "    calib_offset_x: 0.0\n";
  const TempYaml file("invalid_result", original);

  EXPECT_EQ(PersistYamlKeys(file.path(), "gnss_driver",
                            {{"calib_offset_y", "a: b"}}),
            YamlPersistStatus::kInvalidStructure);
  EXPECT_EQ(file.ReadText(), original);
}

TEST(YamlPersist, PreservesExistingFileMode) {
  const TempYaml file("mode", kSampleYaml);
  ASSERT_EQ(::chmod(file.path().c_str(), 0775), 0);

  EXPECT_EQ(PersistYamlKeys(file.path(), "gnss_driver",
                            {{"calib_offset_x", "1.250000"}}),
            YamlPersistStatus::kPersisted);
  struct stat status {};
  ASSERT_EQ(::stat(file.path().c_str(), &status), 0);
  EXPECT_EQ(status.st_mode & 0777, 0775u);
}

TEST(YamlPersist, WritesThroughSymlinkWithoutReplacingIt) {
  const TempYaml file("symlink_target", kSampleYaml);
  const std::string link_path = file.path() + ".link";
  std::error_code error;
  std::filesystem::remove(link_path, error);
  std::filesystem::create_symlink(file.path(), link_path, error);
  ASSERT_FALSE(error);

  EXPECT_EQ(PersistYamlKeys(link_path, "gnss_driver",
                            {{"calib_offset_x", "2.500000"}}),
            YamlPersistStatus::kPersisted);
  EXPECT_TRUE(std::filesystem::is_symlink(link_path));
  EXPECT_EQ(file.ReadLines()[3], "    calib_offset_x: 2.500000");

  std::filesystem::remove(link_path, error);
  std::filesystem::remove(link_path + ".lock", error);
}

TEST(YamlPersist, EqualValueIsUnchangedAndRewritesNothing) {
  const TempYaml file("unchanged", kSampleYaml);
  const auto before = std::filesystem::last_write_time(file.path());

  EXPECT_EQ(PersistYamlKeys(file.path(), "gnss_driver",
                            {{"calib_offset_x", "0.0"}}),
            YamlPersistStatus::kUnchanged);
  EXPECT_EQ(std::filesystem::last_write_time(file.path()), before);
}

TEST(YamlPersist, MissingFileIsReported) {
  EXPECT_EQ(PersistYamlKeys(
                "/tmp/megatrak_yaml_persist_gnss_does_not_exist.yaml",
                "gnss_driver", {{"calib_offset_x", "1.0"}}),
            YamlPersistStatus::kReadFailed);
}

TEST(YamlPersist, ConcurrentWritersPreserveBothUpdates) {
  const TempYaml file(
      "concurrent",
      "gnss_driver:\n"
      "  ros__parameters:\n"
      "    calib_offset_x: 0.0\n"
      "    calib_offset_y: 0.0\n");
  std::atomic<int> ready{0};
  std::atomic<bool> start{false};
  std::atomic<int> failures{0};

  auto writer = [&](const std::string& key, const std::string& value) {
    ready.fetch_add(1);
    while (!start.load())
      std::this_thread::yield();
    if (PersistYamlKeys(file.path(), "gnss_driver", {{key, value}}) !=
        YamlPersistStatus::kPersisted) {
      failures.fetch_add(1);
    }
  };
  std::thread first(writer, "calib_offset_x", "1.0");
  std::thread second(writer, "calib_offset_y", "2.0");
  while (ready.load() != 2)
    std::this_thread::yield();
  start.store(true);
  first.join();
  second.join();
  EXPECT_EQ(failures.load(), 0);

  const std::vector<std::string> lines = file.ReadLines();
  EXPECT_EQ(lines[2], "    calib_offset_x: 1.0");
  EXPECT_EQ(lines[3], "    calib_offset_y: 2.0");
}

TEST(YamlPersist, AppliesMultipleUpdatesInTargetScope) {
  std::vector<std::string> lines = {
      "gnss_driver:",
      "  ros__parameters:",
      "    calib_offset_x: 0.0",
      "    calib_offset_y: 0.0",
  };
  YamlUpdateReport report;

  EXPECT_TRUE(ApplyYamlKeyUpdates(
      lines, "gnss_driver",
      {{"calib_offset_x", "1.250000"}, {"calib_offset_y", "-0.400000"}},
      &report));
  EXPECT_TRUE(report.target_node_found);
  EXPECT_TRUE(report.mapping_found);
  EXPECT_EQ(lines[2], "    calib_offset_x: 1.250000");
  EXPECT_EQ(lines[3], "    calib_offset_y: -0.400000");
  EXPECT_FALSE(ApplyYamlKeyUpdates(
      lines, "gnss_driver", {{"calib_offset_x", "1.250000"}}));
}
