// 参数写回 yaml 的契约 (src/yaml_persist.hpp —— 与 tractor_calibration 的同名文件是**镜像实现**)。
//
// 运行期改标定参数 (calib_offset_x/y/heading) 会把新值落盘 (重启后仍是新值) —— 规则与
// tractor_calibration 的写回**完全一致**。这条测试最该钉住, 因为写坏一次
// yaml 就是整车配置丢失: 现场那份文件里有大量注释与顺序约定, 任何"重排全文/丢注释/写出重复键"
// 的实现都会让下一次启动读到别的配置。
//
// 与 tractor_calibration 的 test_yaml_persist 是同一套用例 (那边测镜像的另一份实现);
// 临时文件名带 gnss_ 前缀, 两个包的测试并行跑也不会互相踩。
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

#include "yaml_persist.hpp"

using gnss_localization::ApplyYamlKeyUpdates;
using gnss_localization::PersistYamlKeys;
using gnss_localization::YamlPersistStatus;
using gnss_localization::YamlUpdateReport;

namespace {

// 临时 yaml: 构造时写内容, 析构时删文件 (名字带用例名, 失败时可直接去 /tmp 看原文)。
class TempYaml {
 public:
  TempYaml(const std::string& name, const std::string& content)
      : path_("/tmp/megatrak_yaml_persist_gnss_" + name + ".yaml") {
    std::ofstream out(path_, std::ios::binary | std::ios::trunc);
    out << content;
  }
  ~TempYaml() {
    std::error_code ec;
    std::filesystem::remove(path_, ec);
    std::filesystem::remove(path_ + ".tmp", ec);
  }
  TempYaml(const TempYaml&)            = delete;
  TempYaml& operator=(const TempYaml&) = delete;
  const std::string& path() const { return path_; }

  std::vector<std::string> ReadLines() const {
    std::vector<std::string> lines;
    std::ifstream in(path_);
    std::string line;
    while (std::getline(in, line)) {
      lines.push_back(line);
    }
    return lines;
  }

 private:
  std::string path_;
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

}   // namespace

// 替换值: 保留行尾注释与缩进, 其它行逐字不动。
TEST(YamlPersist, ReplacesValueKeepingCommentsAndOtherLines) {
  const TempYaml file("replace", kSampleYaml);
  YamlUpdateReport report;

  EXPECT_EQ(PersistYamlKeys(file.path(), {{"calib_offset_x", "1.250000"}}, &report), YamlPersistStatus::kPersisted);
  EXPECT_EQ(report.duplicate_keys, 0u);
  EXPECT_TRUE(report.mapping_found);
  EXPECT_FALSE(report.dir_sync_failed);   // 正常路径: 父目录 fsync 没失败 (现场只有掉电/文件系统错才会为真)

  const std::vector<std::string> lines = file.ReadLines();
  ASSERT_GE(lines.size(), 8u);
  EXPECT_EQ(lines[0], "gnss_driver:");
  EXPECT_EQ(lines[1], "  ros__parameters:");
  EXPECT_EQ(lines[2], "    # 标定值 (运行期改会写回本文件)");   // 注释行原样保留
  EXPECT_EQ(lines[3], "    calib_offset_x: 1.250000");        // 只改值
  EXPECT_EQ(lines[4], "    calib_offset_y: 0.0        # 行尾注释要保留");
  EXPECT_EQ(lines[5], "    calib_offset_heading: 0.0");       // 其它键不动
  EXPECT_EQ(lines[6], "    calib_offset_max_xy: 2.5");
  EXPECT_EQ(lines[7], "    startup_warmup_sec: 30.0");
  // 落盘后不留临时文件 (现场排查时一个 .tmp 会让人怀疑写失败)。
  EXPECT_FALSE(std::filesystem::exists(file.path() + ".tmp"));
}

// 重复键: **全部**替换 (解析器对重复键取最后一个, 只改第一处会出现"写成功却没生效"), 并回报次数。
TEST(YamlPersist, ReplacesEveryDuplicateAndReportsThem) {
  const TempYaml file("duplicate",
                      "gnss_driver:\n"
                      "  ros__parameters:\n"
                      "    calib_offset_x: 0.0\n"
                      "    calib_offset_y: 0.0\n"
                      "    calib_offset_x: 0.5\n");
  YamlUpdateReport report;

  EXPECT_EQ(PersistYamlKeys(file.path(), {{"calib_offset_x", "1.250000"}}, &report), YamlPersistStatus::kPersisted);
  EXPECT_EQ(report.duplicate_keys, 1u);

  const std::vector<std::string> lines = file.ReadLines();
  EXPECT_EQ(lines[2], "    calib_offset_x: 1.250000");
  EXPECT_EQ(lines[4], "    calib_offset_x: 1.250000");
}

// 只有键没有值 (空值 / 值被注释掉): 把值填到冒号后, 原注释保留, **不**追加新行。
TEST(YamlPersist, FillsKeyWithoutValueInsteadOfAppending) {
  const TempYaml file("empty_value",
                      "gnss_driver:\n"
                      "  ros__parameters:\n"
                      "    calib_offset_heading:      # 现场标定定\n");
  YamlUpdateReport report;

  EXPECT_EQ(PersistYamlKeys(file.path(), {{"calib_offset_heading", "0.005000"}}, &report), YamlPersistStatus::kPersisted);
  const std::vector<std::string> lines = file.ReadLines();
  ASSERT_EQ(lines.size(), 3u);   // 没有多出一行 (多出一行就是重复键)
  EXPECT_EQ(lines[2], "    calib_offset_heading: 0.005000      # 现场标定定");
}

// 缺键: 插到 **ros__parameters 映射内** (追加到文件末尾可能落在别的节点段里, 写成功但不生效)。
TEST(YamlPersist, AppendsMissingKeyInsideRosParametersMapping) {
  const TempYaml file("append",
                      "gnss_driver:\n"
                      "  ros__parameters:\n"
                      "    calib_offset_x: 0.0\n"
                      "\n"
                      "ntrip_client:\n"
                      "  ros__parameters:\n"
                      "    mount_point: \"RTCM32\"\n");
  YamlUpdateReport report;

  EXPECT_EQ(PersistYamlKeys(file.path(), {{"calib_offset_heading", "0.010000"}}, &report), YamlPersistStatus::kPersisted);
  EXPECT_TRUE(report.mapping_found);

  const std::vector<std::string> lines = file.ReadLines();
  ASSERT_EQ(lines.size(), 8u);
  EXPECT_EQ(lines[2], "    calib_offset_heading: 0.010000");   // 落在第一个 ros__parameters 里
  EXPECT_EQ(lines[3], "    calib_offset_x: 0.0");
  EXPECT_EQ(lines[6], "  ros__parameters:");                   // 另一个节点段不受影响
  EXPECT_EQ(lines[7], "    mount_point: \"RTCM32\"");
}

// 没有 ros__parameters (不是什么正常情况下会有的文件, 但部署时可能被写坏): 追加到末尾并**回报**
// (调用方据此告警"可能不生效"), 而不是静默写一个不会被解析的位置。
TEST(YamlPersist, ReportsMissingMappingWhenAppending) {
  const TempYaml file("no_mapping", "gnss_driver:\n  foo: 1\n");
  YamlUpdateReport report;

  EXPECT_EQ(PersistYamlKeys(file.path(), {{"calib_offset_x", "1.250000"}}, &report), YamlPersistStatus::kPersisted);
  EXPECT_FALSE(report.mapping_found);
  const std::vector<std::string> lines = file.ReadLines();
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_EQ(lines[2], "    calib_offset_x: 1.250000");
}

// 注释里提到的参数名不能被改; 行中间的相似键名也不能被改 (只认"行首的 key:")。
TEST(YamlPersist, NeverTouchesCommentsOrSubstringKeys) {
  const TempYaml file("no_false_positive",
                      "gnss_driver:\n"
                      "  ros__parameters:\n"
                      "    # calib_offset_x: 9.9   <-- 注释里的不能动\n"
                      "    fake_calib_offset_x: 8.8\n"
                      "    calib_offset_x: 0.0\n");
  YamlUpdateReport report;

  EXPECT_EQ(PersistYamlKeys(file.path(), {{"calib_offset_x", "1.250000"}}, &report), YamlPersistStatus::kPersisted);
  const std::vector<std::string> lines = file.ReadLines();
  EXPECT_EQ(lines[2], "    # calib_offset_x: 9.9   <-- 注释里的不能动");
  EXPECT_EQ(lines[3], "    fake_calib_offset_x: 8.8");
  EXPECT_EQ(lines[4], "    calib_offset_x: 1.250000");
}

// 等值替换: 不重写文件 (少一次落盘窗口, 也不动 mtime), 状态仍是"成功"(kUnchanged)。
TEST(YamlPersist, EqualValueIsUnchangedAndRewritesNothing) {
  const TempYaml file("unchanged", kSampleYaml);
  const auto before = std::filesystem::last_write_time(file.path());

  EXPECT_EQ(PersistYamlKeys(file.path(), {{"calib_offset_x", "0.0"}}), YamlPersistStatus::kUnchanged);
  EXPECT_EQ(std::filesystem::last_write_time(file.path()), before);
  ASSERT_EQ(file.ReadLines().size(), 8u);
  EXPECT_EQ(file.ReadLines()[3], "    calib_offset_x: 0.0");
}

// 文件读不到: 明确回报 kReadFailed (调用方据此告诉现场"参数只在内存生效")。
TEST(YamlPersist, MissingFileIsReported) {
  EXPECT_EQ(PersistYamlKeys("/tmp/megatrak_yaml_persist_gnss_does_not_exist.yaml", {{"calib_offset_x", "1.0"}}), YamlPersistStatus::kReadFailed);
}

// 纯函数层: 多个键一次写回; 返回值表示"文本有没有被改动" (调用方据此决定落不落盘)。
TEST(YamlPersist, AppliesMultipleUpdatesAndReportsModification) {
  std::vector<std::string> lines = {
    "gnss_driver:",
    "  ros__parameters:",
    "    calib_offset_x: 0.0",
    "    calib_offset_y: 0.0",
  };

  EXPECT_TRUE(ApplyYamlKeyUpdates(lines, {{"calib_offset_x", "1.250000"}, {"calib_offset_y", "-0.400000"}}));
  EXPECT_EQ(lines[2], "    calib_offset_x: 1.250000");
  EXPECT_EQ(lines[3], "    calib_offset_y: -0.400000");

  // 再写同一个值: 文本没变 -> 返回 false (调用方不落盘)。
  EXPECT_FALSE(ApplyYamlKeyUpdates(lines, {{"calib_offset_x", "1.250000"}}));
}
