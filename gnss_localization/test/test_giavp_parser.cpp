// GIAVP 解析器只接受校验和正确、字段完整且数值有限的语句。
#include <gtest/gtest.h>

#include <iomanip>
#include <sstream>
#include <string>

#include "gnss_localization/giavp_parser.hpp"

namespace {

std::string WithChecksum(const std::string& body) {
  unsigned char checksum = 0;
  for (const char character : body) {
    checksum ^= static_cast<unsigned char>(character);
  }
  std::ostringstream sentence;
  sentence << '$' << body << '*' << std::uppercase << std::hex
           << std::setw(2) << std::setfill('0')
           << static_cast<unsigned int>(checksum);
  return sentence.str();
}

std::string Sentence(const std::string& lat = "34.70316078",
                     const std::string& ve = "0.5",
                     const std::string& gyro_z = "0.1") {
  return WithChecksum(
      "GIAVP,2200,123456.0,90.0,0.0,0.0," + lat +
      ",112.39118753,102.0," + ve +
      ",0.0,0.0,1.5,20,19,4,1,1.8,0.01,0.02,9.81,0.0,0.0," +
      gyro_z);
}

}  // 匿名命名空间

TEST(GiavpParser, AcceptsFiniteSentence) {
  gnss_localization::GiavpParser parser;
  ASSERT_TRUE(parser.parse(Sentence()));
  const auto& data = parser.data();
  EXPECT_TRUE(data.valid);
  EXPECT_DOUBLE_EQ(data.latitude_deg, 34.70316078);
  EXPECT_DOUBLE_EQ(data.ve_ms, 0.5);
  EXPECT_DOUBLE_EQ(data.gyro_z_dps, 0.1);
  EXPECT_EQ(data.status, 4);
}

TEST(GiavpParser, RejectsNanOrInfInAnyNumericField) {
  gnss_localization::GiavpParser parser;
  EXPECT_FALSE(parser.parse(Sentence("nan")));
  EXPECT_FALSE(parser.parse(Sentence("inf")));
  EXPECT_FALSE(parser.parse(Sentence("34.70316078", "nan")));
  EXPECT_FALSE(parser.parse(Sentence("34.70316078", "inf")));
  EXPECT_FALSE(parser.parse(Sentence("34.70316078", "0.5", "nan")));
}

TEST(GiavpParser, RejectsOutOfRangeLatLon) {
  gnss_localization::GiavpParser parser;
  EXPECT_FALSE(parser.parse(Sentence("91.0")));
  EXPECT_FALSE(parser.parse(Sentence("-90.5")));
}

TEST(GiavpParser, RejectsEmptyOrGarbageFields) {
  gnss_localization::GiavpParser parser;
  EXPECT_FALSE(parser.parse(Sentence("", "")));
  EXPECT_FALSE(parser.parse(Sentence("34.7x", "0.5")));
}

TEST(GiavpParser, RejectsMissingOrInvalidChecksum) {
  gnss_localization::GiavpParser parser;
  const std::string valid = Sentence();
  EXPECT_FALSE(parser.parse(valid.substr(0, valid.size() - 3)));

  std::string invalid = valid;
  invalid.back() = invalid.back() == '0' ? '1' : '0';
  EXPECT_FALSE(parser.parse(invalid));
}
