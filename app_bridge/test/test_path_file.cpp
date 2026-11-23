#include <gtest/gtest.h>

#include "app_bridge/path_file.hpp"

using app_bridge::path_file::Crc32Ieee;
using app_bridge::path_file::Summary;
using app_bridge::path_file::Validate;

namespace {

// 与 global_path_planner/test/test_file_path_parser.cpp 的 kSampleJson **同一份**:
// crc32 4025357972 是规划器单测里核过的值, 两边算法一致的证据就是它们都过。
constexpr const char* kSampleJson = R"({
  "task_name": "farm_full_coverage",
  "unit": "wgs84",
  "crc32": 4025357972,
  "path": [
    {"lon":113.6470,"lat":34.7580,"turn_flag":false},
    {"lon":113.6480,"lat":34.7580,"turn_flag":false},
    {"lon":113.6490,"lat":34.7580,"turn_flag":false},
    {"lon":113.6490,"lat":34.7588,"turn_flag":true},
    {"lon":113.6480,"lat":34.7588,"turn_flag":false},
    {"lon":113.6470,"lat":34.7588,"turn_flag":false},
    {"lon":113.6470,"lat":34.7596,"turn_flag":true},
    {"lon":113.6480,"lat":34.7596,"turn_flag":false},
    {"lon":113.6490,"lat":34.7596,"turn_flag":false},
    {"lon":113.6490,"lat":34.7604,"turn_flag":true},
    {"lon":113.6480,"lat":34.7604,"turn_flag":false},
    {"lon":113.6470,"lat":34.7604,"turn_flag":false}
  ]
})";

// 必须拒绝, 且给出原因
void ExpectRejected(const std::string& text) {
  Summary s;
  std::string err;
  EXPECT_FALSE(Validate(text, s, err)) << text;
  EXPECT_FALSE(err.empty()) << text;
}

}   // namespace

TEST(PathFile, Crc32KnownVector) {
  EXPECT_EQ(Crc32Ieee("123456789"), 0xCBF43926u);
}

TEST(PathFile, AcceptsPlannerSample) {
  Summary s;
  std::string err;
  ASSERT_TRUE(Validate(kSampleJson, s, err)) << err;
  EXPECT_EQ(s.num_points, 12u);
  EXPECT_TRUE(s.crc32_verified);
  EXPECT_EQ(s.task_name, "farm_full_coverage");
}

TEST(PathFile, Crc32Optional) {
  Summary s;
  std::string err;
  ASSERT_TRUE(Validate(R"({"unit":"wgs84","path":[{"lon":1,"lat":2},{"lon":1.1,"lat":2.1}]})", s, err)) << err;
  EXPECT_FALSE(s.crc32_verified);
}

TEST(PathFile, RejectsCorruptedPoint) {
  std::string bad = kSampleJson;
  bad.replace(bad.find("113.6480"), 8, "113.6481");   // 改一个点 -> crc32 不再匹配
  ExpectRejected(bad);
}

TEST(PathFile, RejectsBadInput) {
  ExpectRejected("not json");
  ExpectRejected("[]");
  ExpectRejected(R"({"path":[{"lon":1,"lat":2},{"lon":1,"lat":2}]})");                              // 缺 unit
  ExpectRejected(R"({"unit":"enu","path":[{"lon":1,"lat":2},{"lon":1,"lat":2}]})");                 // unit 不支持
  ExpectRejected(R"({"unit":"wgs84"})");                                                             // 缺 path
  ExpectRejected(R"({"unit":"wgs84","path":[{"lon":1,"lat":2}]})");                                 // 点数不足
  ExpectRejected(R"({"unit":"wgs84","path":[{"lon":1,"lat":95},{"lon":1,"lat":2}]})");              // 纬度越界
  ExpectRejected(R"({"unit":"wgs84","path":[{"lon":1},{"lon":1,"lat":2}]})");                       // 缺 lat
  ExpectRejected(R"({"unit":"wgs84","path":[{"lon":1,"lat":2,"turn_flag":1},{"lon":1,"lat":2}]})"); // turn_flag 非布尔
  ExpectRejected(R"({"unit":"wgs84","crc32":-1,"path":[{"lon":1,"lat":2},{"lon":1,"lat":2}]})");    // crc32 非法
}
