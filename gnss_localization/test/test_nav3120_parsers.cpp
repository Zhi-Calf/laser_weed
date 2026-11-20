#include <gtest/gtest.h>

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

#include "gnss_localization/gga_status_parser.hpp"
#include "gnss_localization/inspvaa_parser.hpp"

namespace {

constexpr uint32_t kCrc32Polynomial = 0xEDB88320U;

uint32_t Crc32(const std::string& body) {
  uint32_t crc = 0;
  for (const unsigned char byte : body) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U ? (crc >> 1U) ^ kCrc32Polynomial : crc >> 1U;
    }
  }
  return crc;
}

std::string WithInspvaaCrc(const std::string& body) {
  std::ostringstream sentence;
  sentence << '#' << body << '*' << std::hex << std::nouppercase << std::setw(8) << std::setfill('0') << Crc32(body);
  return sentence.str();
}

std::string Inspvaa(const std::string& status = "INS_HIGH_VARIANCE") {
  return WithInspvaaCrc(
    "INSPVAA,COM1,0,54%,FINE,2445,294303.900,0,14,18;2445,294303.900,34.76569927,113.55006586,135.934052,0.000000,0.000000,0.000000,0.182769,-0.947825,90.765137," +
    status);
}

}   // namespace

// 验证真实 INSPVAA 样例的 CRC32 和字段顺序。
TEST(InspvaaParser, AcceptsReceiverSample) {
  gnss_localization::InspvaaParser parser;
  ASSERT_TRUE(parser.parse(
    "#INSPVAA,COM1,0,54%,FINE,2445,294303.900,0,14,18;"
    "2445,294303.900,34.76569927,113.55006586,135.934052,"
    "0.000000,0.000000,0.000000,0.182769,-0.947825,90.765137,"
    "INS_HIGH_VARIANCE*554cfd21"));

  const auto& data = parser.data();
  EXPECT_EQ(data.gps_week, 2445U);
  EXPECT_DOUBLE_EQ(data.gps_time_sec, 294303.900);
  EXPECT_DOUBLE_EQ(data.latitude_deg, 34.76569927);
  EXPECT_DOUBLE_EQ(data.longitude_deg, 113.55006586);
  EXPECT_DOUBLE_EQ(data.height_m, 135.934052);
  EXPECT_DOUBLE_EQ(data.heading_deg, 90.765137);
  EXPECT_EQ(data.ins_status, 2);
  EXPECT_TRUE(data.valid);
}

// 状态 3 是融合导航正常，必须能映射。
TEST(InspvaaParser, MapsSolutionGoodStatus) {
  gnss_localization::InspvaaParser parser;
  ASSERT_TRUE(parser.parse(Inspvaa("INS_SOLUTION_GOOD")));
  EXPECT_EQ(parser.data().ins_status, 3);
}

// CRC 覆盖从 INSPVAA 标识到状态字段，破坏任意一字节都必须拒绝。
TEST(InspvaaParser, RejectsCorruptedCrc) {
  gnss_localization::InspvaaParser parser;
  std::string line = Inspvaa();
  line[line.find('*') + 1] = line[line.find('*') + 1] == '0' ? '1' : '0';
  EXPECT_FALSE(parser.parse(line));
}

// GGA 质量仍由 GGA 自身决定；4 表示 RTK Fixed。
TEST(GgaStatusParser, ExtractsFixedQualityAndSatellites) {
  gnss_localization::GgaStatusParser parser;
  ASSERT_TRUE(parser.parse("$GNGGA,094445.900,3445.94194793,N,11333.00395055,E,4,18,1.6,155.6865,M,-19.7205,M,1.9,2189*43"));
  EXPECT_EQ(parser.data().fix_quality, 4);
  EXPECT_EQ(parser.data().satellites, 18);
  EXPECT_DOUBLE_EQ(parser.data().hdop, 1.6);
  EXPECT_DOUBLE_EQ(parser.data().differential_age_sec, 1.9);
  EXPECT_TRUE(parser.data().valid);
}

// 无解 GGA 必须更新为 quality=0，不能用上一帧固定状态继续放行位姿。
TEST(GgaStatusParser, AcceptsNoFixSentence) {
  gnss_localization::GgaStatusParser parser;
  ASSERT_TRUE(parser.parse("$GNGGA,,,,,,0,,,,,,,,*78"));
  EXPECT_EQ(parser.data().fix_quality, 0);
  EXPECT_EQ(parser.data().satellites, 0);
  EXPECT_TRUE(parser.data().valid);
}
