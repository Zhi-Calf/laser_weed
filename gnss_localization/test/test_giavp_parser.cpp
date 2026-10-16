// $GIAVP 解析器的**有效性校验**契约 (与 gnss_driver 的 NaN/Inf 校验同一件事, 见 giavp_parser.cpp):
//   报文里的数值字段凡是 nan/inf, 整句判无效 —— 那些字段会被原样发到 fix / odom / imu / vel / status,
//   下游从数值本身看不出"这帧坏了"。空字段/解析不了的值仍按 0 处理 (不算无效)。
// 用例里的报文**不带** "*cs" (解析器对无校验和的句子是接受的), 这样只测数值校验本身。
#include <gtest/gtest.h>

#include "gnss_localization/giavp_parser.hpp"

namespace {

// 字段顺序见 giavp_parser.hpp: Week,Time,Heading,Pitch,Roll,Lat,Lon,Alt,Ve,Vn,Vu,Baseline,
//                             Nsv1,Nsv2,Status,SpeedStatus,VehicleSpeed,AccX,AccY,AccZ,GyroX,GyroY,GyroZ
std::string Sentence(const std::string& lat = "34.70316078", const std::string& ve = "0.5", const std::string& gyro_z = "0.1") {
  return "$GIAVP,2200,123456.0,90.0,0.0,0.0," + lat + ",112.39118753,102.0," + ve +
         ",0.0,0.0,1.5,20,19,4,1,1.8,0.01,0.02,9.81,0.0,0.0," + gyro_z;
}

}   // namespace

// 正常报文: 解析成功且字段值正确 (基线行为, 免得"收紧校验"顺手把好数据也丢了)。
TEST(GiavpParser, AcceptsFiniteSentence) {
  gnss_localization::GiavpParser parser;
  ASSERT_TRUE(parser.parse(Sentence()));
  const auto& d = parser.data();
  EXPECT_TRUE(d.valid);
  EXPECT_DOUBLE_EQ(d.latitude_deg, 34.70316078);
  EXPECT_DOUBLE_EQ(d.ve_ms, 0.5);
  EXPECT_DOUBLE_EQ(d.gyro_z_dps, 0.1);
  EXPECT_EQ(d.status, 4);
}

// 核心契约: 任一数值字段非有限 -> 整句无效 (不给下游发带 NaN 的消息)。
TEST(GiavpParser, RejectsNanOrInfInAnyNumericField) {
  gnss_localization::GiavpParser parser;
  EXPECT_FALSE(parser.parse(Sentence("nan"))) << "纬度 nan 应判无效";
  EXPECT_FALSE(parser.parse(Sentence("inf"))) << "纬度 inf 应判无效";
  EXPECT_FALSE(parser.parse(Sentence("34.70316078", "nan"))) << "东向速度 nan 应判无效";
  EXPECT_FALSE(parser.parse(Sentence("34.70316078", "inf"))) << "东向速度 inf 应判无效";
  EXPECT_FALSE(parser.parse(Sentence("34.70316078", "0.5", "nan"))) << "陀螺 z nan 应判无效";
  // 判无效之后不允许留下"半句有效数据" (data_.valid 仍是上一次的值), 调用方只看 parse() 的返回值。
}

// 越界经纬度仍然判无效 (原有判据, 别在校验改动里丢掉)。
TEST(GiavpParser, RejectsOutOfRangeLatLon) {
  gnss_localization::GiavpParser parser;
  EXPECT_FALSE(parser.parse(Sentence("91.0")));
  EXPECT_FALSE(parser.parse(Sentence("-90.5")));
}

// 空字段/垃圾字段按 0.0 处理 (safeStod 的既有语义), **不算无效** —— 真机断流时字段可能是空的,
// 把它当"坏句"会让本来可用的帧被丢掉。
TEST(GiavpParser, EmptyOrGarbageFieldsFallBackToZero) {
  gnss_localization::GiavpParser parser;
  ASSERT_TRUE(parser.parse(Sentence("", "")));
  EXPECT_DOUBLE_EQ(parser.data().latitude_deg, 0.0);
  EXPECT_DOUBLE_EQ(parser.data().ve_ms, 0.0);
}
