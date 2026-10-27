#include <gtest/gtest.h>

#include <iomanip>
#include <sstream>
#include <string>

#include "gnss_localization/gga_parser.hpp"

namespace {

std::string WithChecksum(const std::string& body) {
  unsigned char checksum = 0;
  for (char character : body) {
    checksum ^= static_cast<unsigned char>(character);
  }
  std::ostringstream sentence;
  sentence << '$' << body << '*' << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(checksum);
  return sentence.str();
}

std::string Gga(
  const std::string& type          = "GPGGA",
  const std::string& utc           = "123519",
  const std::string& latitude      = "4807.038",
  const std::string& latitude_dir  = "N",
  const std::string& longitude     = "01131.000",
  const std::string& longitude_dir = "E",
  const std::string& quality       = "4",
  const std::string& satellites    = "08",
  const std::string& hdop          = "0.9",
  const std::string& altitude      = "545.4",
  const std::string& altitude_unit = "M",
  const std::string& geoid         = "46.9",
  const std::string& geoid_unit    = "M",
  const std::string& optional      = ",,") {
  return WithChecksum(
    type + "," + utc + "," + latitude + "," + latitude_dir + "," + longitude + "," + longitude_dir + "," + quality + "," + satellites + "," + hdop + "," + altitude + "," + altitude_unit + "," + geoid + "," +
    geoid_unit + optional);
}

}   // namespace

TEST(GgaParser, AcceptsCompleteStandardSentence) {
  EXPECT_TRUE(gnss_localization::IsValidGga(Gga()));
}

TEST(GgaParser, AcceptsSentenceWithoutOptionalDifferentialFields) {
  EXPECT_TRUE(gnss_localization::IsValidGga(Gga("GPGGA", "123519", "4807.038", "N", "01131.000", "E", "4", "08", "0.9", "545.4", "M", "46.9", "M", "")));
}

TEST(GgaParser, AcceptsOtherTalkerAndHemispheres) {
  EXPECT_TRUE(gnss_localization::IsValidGga(Gga("GNGGA", "235960", "9000.000", "S", "18000.000", "W")));
}

TEST(GgaParser, RejectsInvalidChecksumOrTrailingData) {
  std::string invalid = Gga();
  invalid.back()      = invalid.back() == '0' ? '1' : '0';
  EXPECT_FALSE(gnss_localization::IsValidGga(invalid));
  EXPECT_FALSE(gnss_localization::IsValidGga(Gga() + "garbage"));
}

TEST(GgaParser, RejectsInvalidFixQuality) {
  EXPECT_FALSE(gnss_localization::IsValidGga(Gga("GPGGA", "123519", "4807.038", "N", "01131.000", "E", "0")));
  EXPECT_FALSE(gnss_localization::IsValidGga(Gga("GPGGA", "123519", "4807.038", "N", "01131.000", "E", "9")));
}

TEST(GgaParser, RejectsInvalidCoordinates) {
  EXPECT_FALSE(gnss_localization::IsValidGga(Gga("GPGGA", "123519", "", "N")));
  EXPECT_FALSE(gnss_localization::IsValidGga(Gga("GPGGA", "123519", "4807.038", "X")));
  EXPECT_FALSE(gnss_localization::IsValidGga(Gga("GPGGA", "123519", "9000.001", "N")));
  EXPECT_FALSE(gnss_localization::IsValidGga(Gga("GPGGA", "123519", "4807.038", "N", "18000.001", "E")));
}

TEST(GgaParser, RejectsIncompleteAndNonFiniteFields) {
  EXPECT_FALSE(gnss_localization::IsValidGga(WithChecksum("GPGGA,123519,4807.038,N,01131.000,E,4")));
  EXPECT_FALSE(gnss_localization::IsValidGga(Gga("GPGGA", "123519", "4807.038", "N", "01131.000", "E", "4", "08", "nan")));
  EXPECT_FALSE(gnss_localization::IsValidGga(Gga("GPGGA", "123519", "4807.038", "N", "01131.000", "E", "4", "08", "0.9", "inf")));
  EXPECT_FALSE(gnss_localization::IsValidGga(Gga("GPGGA", "123519", "4807.038", "N", "01131.000", "E", "4", "08", "0.9", "545.4", "X")));
}

TEST(GgaParser, RejectsInvalidTimeAndTalker) {
  EXPECT_FALSE(gnss_localization::IsValidGga(Gga("GPGGA", "246000", "4807.038", "N")));
  EXPECT_FALSE(gnss_localization::IsValidGga(Gga("gpgga", "123519", "4807.038", "N")));
}
