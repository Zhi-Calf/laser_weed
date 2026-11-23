#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "app_bridge/farm_device.hpp"

using namespace app_bridge::farm;

namespace {

const std::string kDevice = "111";

TimePoint At(const std::string& rfc3339) {
  return *ParseRfc3339(rfc3339);
}

// 一条合法的 MODE_SET 下行, 测试按需改字段
json ValidCommand() {
  return {
      {"commandId", "cmd-mode-set-001"},
      {"eventId", nullptr},
      {"traceId", "trace-device-111-001"},
      {"schema", "farm-device"},
      {"schemaRevision", 1},
      {"deviceId", "111"},
      {"messageType", "MODE_SET"},
      {"timestamp", "2026-10-12T08:00:00Z"},
      {"expiresAt", "2026-10-12T08:00:30Z"},
      {"payload", {{"mode", "MANUAL"}}},
  };
}

ParseResult Parse(const json& j, Command& cmd, std::string& code, std::string& err, const std::string& now = "2026-10-12T08:00:10Z") {
  return ParseCommand(j.dump(), kDevice, At(now), cmd, code, err);
}

// §7.3.1: 已声明字段的显式 null 会被网关拒收 —— 上行里任何位置都不能出现 null (信封的可空字段除外)
bool ContainsNull(const json& j) {
  if (j.is_null()) {
    return true;
  }
  if (j.is_structured()) {
    for (const auto& v : j) {
      if (ContainsNull(v)) {
        return true;
      }
    }
  }
  return false;
}

}   // namespace

// ---- 认证 (§2.2 / §2.2.1 固定测试向量) ----

TEST(Auth, MatchesDocumentTestVector) {
  EXPECT_EQ(MqttClientId("tractor_v1", "robot_001"), "tractor_v1.robot_001");
  EXPECT_EQ(MqttUsername("tractor_v1", "robot_001"), "robot_001&tractor_v1");
  EXPECT_EQ(MqttPassword("tractor_v1", "robot_001", "iot-doc-test-secret-2026"), "58fd37af959da8c7cf6d5cfd15c351c854353d2357991b91cfff0104a27a36c4");
}

TEST(Auth, DeviceId) {
  EXPECT_TRUE(IsValidDeviceId("111"));
  EXPECT_TRUE(IsValidDeviceId("9223372036854775807"));
  EXPECT_FALSE(IsValidDeviceId("9223372036854775808"));
  EXPECT_FALSE(IsValidDeviceId("0111"));
  EXPECT_FALSE(IsValidDeviceId("0"));
  EXPECT_FALSE(IsValidDeviceId(""));
  EXPECT_FALSE(IsValidDeviceId("robot_001"));
  EXPECT_FALSE(IsValidDeviceId("-1"));
}

TEST(Topics, MatchDocument) {
  const auto t = Topics::For("tractor_v1", "robot_001");
  EXPECT_EQ(t.control_downstream, "/sys/tractor_v1/robot_001/control/downstream");
  EXPECT_EQ(t.status, "/sys/tractor_v1/robot_001/status/upstream");
  EXPECT_EQ(t.control_upstream, "/sys/tractor_v1/robot_001/control/upstream");
  EXPECT_EQ(t.telemetry, "/sys/tractor_v1/robot_001/telemetry/upstream");
  EXPECT_EQ(t.fault, "/sys/tractor_v1/robot_001/thing.event.post/upstream");
}

// ---- 时间 ----

TEST(Time, ParsesZuluOffsetAndFraction) {
  EXPECT_EQ(At("2026-10-12T08:00:00Z"), At("2026-10-12T16:00:00+08:00"));
  EXPECT_EQ(At("2026-10-12T08:00:00Z"), At("2026-10-12T03:00:00-05:00"));
  EXPECT_EQ(At("2026-10-12T08:00:00.500Z") - At("2026-10-12T08:00:00Z"), std::chrono::milliseconds(500));
  EXPECT_FALSE(ParseRfc3339("2026-10-12 08:00:00Z"));
  EXPECT_FALSE(ParseRfc3339("2026-10-12T08:00:00"));     // 无时区
  EXPECT_FALSE(ParseRfc3339("2026-10-12T08:00:00.Z"));
  EXPECT_FALSE(ParseRfc3339("2026-13-12T08:00:00Z"));
  EXPECT_FALSE(ParseRfc3339("1791976438"));
}

TEST(Time, FormatRoundTrips) {
  const auto t = At("2026-10-12T08:00:00.123Z");
  EXPECT_EQ(FormatRfc3339(t), "2026-10-12T08:00:00.123Z");
}

TEST(Ids, PrefixedAndBounded) {
  const auto a = NewId("evt");
  EXPECT_EQ(a.rfind("evt-", 0), 0u);
  EXPECT_LE(a.size(), kMaxIdLength);
  EXPECT_NE(a, NewId("evt"));
}

// ---- 下行信封 ----

TEST(Command, AcceptsValid) {
  Command cmd;
  std::string code, err;
  ASSERT_EQ(Parse(ValidCommand(), cmd, code, err), ParseResult::kOk) << err;
  EXPECT_EQ(cmd.command_id, "cmd-mode-set-001");
  EXPECT_EQ(cmd.trace_id, "trace-device-111-001");
  EXPECT_EQ(cmd.message_type, "MODE_SET");
  EXPECT_EQ(cmd.payload.at("mode"), "MANUAL");
}

TEST(Command, EventIdMayBeAbsent) {
  auto j = ValidCommand();
  j.erase("eventId");
  Command cmd;
  std::string code, err;
  EXPECT_EQ(Parse(j, cmd, code, err), ParseResult::kOk) << err;
}

TEST(Command, DropsWhenCannotAck) {
  Command cmd;
  std::string code, err;
  EXPECT_EQ(ParseCommand("not json", kDevice, At("2026-10-12T08:00:10Z"), cmd, code, err), ParseResult::kDrop);

  auto no_id = ValidCommand();
  no_id.erase("commandId");
  EXPECT_EQ(Parse(no_id, cmd, code, err), ParseResult::kDrop);

  auto other = ValidCommand();
  other["deviceId"] = "222";
  EXPECT_EQ(Parse(other, cmd, code, err), ParseResult::kDrop);

  auto numeric = ValidCommand();
  numeric["deviceId"] = 111;   // deviceId 必须是字符串
  EXPECT_EQ(Parse(numeric, cmd, code, err), ParseResult::kDrop);
}

TEST(Command, RejectsBadEnvelope) {
  const auto expect_reject = [](json j, const char* what) {
    Command cmd;
    std::string code, err;
    EXPECT_EQ(Parse(j, cmd, code, err), ParseResult::kReject) << what;
    EXPECT_EQ(code, code::kInvalidMessage) << what;
    EXPECT_EQ(cmd.command_id, "cmd-mode-set-001") << what;   // 仍能回 ACK
  };
  auto j = ValidCommand();
  j["extra"] = 1;
  expect_reject(j, "未知顶层字段");
  j = ValidCommand();
  j["schemaRevision"] = 2;
  expect_reject(j, "schemaRevision");
  j = ValidCommand();
  j["schema"] = "farm";
  expect_reject(j, "schema");
  j = ValidCommand();
  j["eventId"] = "evt-1";
  expect_reject(j, "下行带 eventId");
  j = ValidCommand();
  j["messageType"] = "TELEMETRY";
  expect_reject(j, "上行类型");
  j = ValidCommand();
  j["expiresAt"] = nullptr;
  expect_reject(j, "缺 expiresAt");
  j = ValidCommand();
  j["timestamp"] = 1791976438;
  expect_reject(j, "数字时间");
  j = ValidCommand();
  j["payload"] = json::array();
  expect_reject(j, "payload 为数组");
}

TEST(Command, MissingTraceIdRejectedWithCommandIdAsTrace) {
  auto j = ValidCommand();
  j.erase("traceId");
  Command cmd;
  std::string code, err;
  EXPECT_EQ(Parse(j, cmd, code, err), ParseResult::kReject);
  EXPECT_EQ(cmd.trace_id, "cmd-mode-set-001");
}

TEST(Command, Expired) {
  Command cmd;
  std::string code, err;
  EXPECT_EQ(Parse(ValidCommand(), cmd, code, err, "2026-10-12T08:00:31Z"), ParseResult::kReject);
  EXPECT_EQ(code, code::kExpired);
}

// ---- MODE_SET (§6.7) ----

TEST(ModeSet, ManualAndCalibrationForbidTaskFields) {
  ModeSet m;
  std::string err;
  EXPECT_TRUE(ParseModeSet({{"mode", "MANUAL"}, {"deadmanToken", "t"}}, m, err)) << err;
  EXPECT_FALSE(ParseModeSet({{"mode", "MANUAL"}, {"taskId", nullptr}}, m, err));   // null 键也不行
  EXPECT_FALSE(ParseModeSet({{"mode", "CALIBRATION"}, {"workMode", "TRACKING"}}, m, err));
}

TEST(ModeSet, TaskModesNeedTaskAndAllowedWorkMode) {
  ModeSet m;
  std::string err;
  ASSERT_TRUE(ParseModeSet({{"mode", "UNMANNED"}, {"taskId", 1001}, {"workMode", "GRASS_GRID"}}, m, err)) << err;
  EXPECT_EQ(*m.task_id, 1001);
  EXPECT_EQ(*m.work_mode, "GRASS_GRID");
  EXPECT_FALSE(ParseModeSet({{"mode", "UNMANNED"}, {"workMode", "TRACKING"}}, m, err));
  EXPECT_FALSE(ParseModeSet({{"mode", "UNMANNED"}, {"taskId", 0}, {"workMode", "TRACKING"}}, m, err));
  EXPECT_FALSE(ParseModeSet({{"mode", "WORK"}, {"taskId", 1}, {"workMode", "GRASS_GRID"}}, m, err));
  EXPECT_FALSE(ParseModeSet({{"mode", "PLANTING"}, {"taskId", 1}, {"workMode", "TRACKING"}}, m, err));
  EXPECT_TRUE(ParseModeSet({{"mode", "PLANTING"}, {"taskId", 1}, {"workMode", "GRASS_GRID"}}, m, err)) << err;
}

TEST(ModeSet, RejectsUnknownModeAndKeys) {
  ModeSet m;
  std::string err;
  EXPECT_FALSE(ParseModeSet({{"mode", "manual"}}, m, err));
  EXPECT_FALSE(ParseModeSet({{"mode", "MANUAL"}, {"speed", 1}}, m, err));
  EXPECT_FALSE(ParseModeSet({{"mode", "MANUAL"}, {"deadmanToken", "  "}}, m, err));
}

TEST(ModeSet, RosMapping) {
  EXPECT_EQ(ModeToRos("MANUAL"), 0);
  EXPECT_EQ(ModeToRos("UNMANNED"), 1);
  EXPECT_EQ(ModeToRos("CALIBRATION"), 2);
  EXPECT_FALSE(ModeToRos("WORK"));
  EXPECT_FALSE(ModeToRos("PLANTING"));
  EXPECT_EQ(ModeFromRos(1), "UNMANNED");
}

// ---- CALIBRATION_COMMAND (§6.8) ----

TEST(Calibration, ParsesAndMaps) {
  CalibrationCommand c;
  std::string err;
  ASSERT_TRUE(ParseCalibrationCommand({{"action", "START"}, {"moduleCode", "GNSS"}, {"calibrationType", "GNSS_FULL"}, {"parameters", json::object()}, {"deadmanToken", "t"}}, c, err)) << err;
  EXPECT_EQ(CalibrationTypeToRos(c.calibration_type), 0);
  EXPECT_EQ(CalibrationActionToRos("START"), 0);
  EXPECT_EQ(CalibrationActionToRos("CANCEL"), 1);
  EXPECT_EQ(CalibrationActionToRos("COMMIT"), 3);
  EXPECT_FALSE(CalibrationActionToRos("RESET"));
  EXPECT_FALSE(CalibrationTypeToRos("STEERING_CENTER"));
}

TEST(Calibration, RequiresTokenAndScalarParameters) {
  CalibrationCommand c;
  std::string err;
  EXPECT_FALSE(ParseCalibrationCommand({{"action", "START"}, {"moduleCode", "GNSS"}, {"calibrationType", "GNSS_FULL"}, {"parameters", json::object()}}, c, err));
  EXPECT_FALSE(ParseCalibrationCommand({{"action", "START"}, {"moduleCode", "GNSS"}, {"calibrationType", "GNSS_FULL"}, {"deadmanToken", "t"}}, c, err));
  EXPECT_FALSE(ParseCalibrationCommand({{"action", "START"}, {"moduleCode", "GNSS"}, {"calibrationType", "GNSS_FULL"}, {"parameters", {{"a", {1, 2}}}}, {"deadmanToken", "t"}}, c, err));
  EXPECT_FALSE(ParseCalibrationCommand({{"action", "GO"}, {"moduleCode", "GNSS"}, {"calibrationType", "GNSS_FULL"}, {"parameters", json::object()}, {"deadmanToken", "t"}}, c, err));
}

// ---- 上行 ----

TEST(Upstream, EnvelopeHasExactlyTenFields) {
  UpstreamHeader h{"111", type::kCommandAck, "evt-1", "trace-1", "2026-10-12T08:00:00.000Z", std::string("cmd-1")};
  const json ack = json::parse(EncodeUpstream(h, AckPayload(stage::kCompleted, code::kOk, "执行完成", json::object())));
  EXPECT_EQ(ack.size(), 10u);
  EXPECT_EQ(ack.at("commandId"), "cmd-1");
  EXPECT_EQ(ack.at("eventId"), "evt-1");
  EXPECT_EQ(ack.at("schema"), "farm-device");
  EXPECT_EQ(ack.at("schemaRevision"), 1);
  EXPECT_TRUE(ack.at("expiresAt").is_null());
  EXPECT_EQ(ack.at("payload").at("stage"), "COMPLETED");
  EXPECT_EQ(ack.at("payload").at("code"), "OK");
  EXPECT_TRUE(ack.at("payload").contains("message"));

  h.message_type = type::kDeviceOnline;
  h.command_id.reset();
  const json online = json::parse(EncodeUpstream(h, DeviceOnlinePayload()));
  EXPECT_TRUE(online.at("commandId").is_null());   // 非 ACK 上行 commandId 必须为 null
  EXPECT_EQ(online.at("payload").at("transport"), "WAN");
  EXPECT_EQ(online.at("payload").at("contractRevision"), 1);
  EXPECT_FALSE(online.at("payload").contains("reasonCode"));
  EXPECT_EQ(DeviceOfflinePayload("SHUTDOWN").at("reasonCode"), "SHUTDOWN");
}

TEST(Upstream, StatusPayloadsOmitAbsentOptionals) {
  const json m = ModeStatusPayload("MANUAL", "ACTIVE", std::nullopt, std::nullopt, "");
  EXPECT_EQ(m, (json{{"mode", "MANUAL"}, {"state", "ACTIVE"}}));
  const json m2 = ModeStatusPayload("UNMANNED", "ACTIVE", std::string("cmd-1"), int64_t{1001}, "");
  EXPECT_EQ(m2.at("relatedCommandId"), "cmd-1");
  EXPECT_EQ(m2.at("taskId"), 1001);

  const json f = FaultEventPayload("gnss_driver", "10201", true, "", json::object());
  EXPECT_EQ(f, (json{{"moduleCode", "gnss_driver"}, {"faultCode", "10201"}, {"active", true}}));
}

TEST(Telemetry, MapsUnitsAndOmitsStaleGroups) {
  weeder_msgs::msg::GnssStatus g;
  g.fix_type       = weeder_msgs::msg::GnssStatus::FIX_RTK_FIXED;
  g.num_satellites = 24;
  g.latitude_deg   = 34.758;
  g.longitude_deg  = 113.647;
  g.altitude_m     = 102.0f;
  g.speed_kmh      = 3.6f;
  g.heading_deg    = -90.0f;
  g.hdop           = 0.6f;
  g.diff_age       = 1.0f;
  weeder_msgs::msg::ChassisState c;
  c.estop = false;
  c.speed_m_s = 1.0f;
  c.bms_soc = 85;

  TelemetryExtras ex;
  ex.control_mode = "UNMANNED";
  const json p    = TelemetryPayload(&g, &c, ex);
  EXPECT_EQ(p.at("location").at("rtkStatus"), "FIXED");
  EXPECT_DOUBLE_EQ(p.at("location").at("speed").get<double>(), 1.0);   // km/h -> m/s
  EXPECT_DOUBLE_EQ(p.at("location").at("direction").get<double>(), 270.0);   // 归一到 [0, 360)
  EXPECT_DOUBLE_EQ(p.at("location").at("hdop").get<double>(), 0.6);
  EXPECT_EQ(p.at("safety").at("emergencyStopReleased"), true);
  EXPECT_EQ(p.at("emergencyStopReleased"), true);
  EXPECT_FALSE(p.contains("implement"));
  EXPECT_EQ(p.at("controlMode"), "UNMANNED");
  EXPECT_FALSE(ContainsNull(p));

  // GNSS / 底盘过期: 整组省略, 绝不发 null, 也不把急停默认为已释放
  const json stale = TelemetryPayload(nullptr, nullptr, ex);
  EXPECT_FALSE(stale.contains("location"));
  EXPECT_FALSE(stale.contains("safety"));
  EXPECT_FALSE(stale.contains("emergencyStopReleased"));
  EXPECT_FALSE(ContainsNull(stale));
}

TEST(Telemetry, NoFixOmitsCoordinatesAndNanIsDropped) {
  weeder_msgs::msg::GnssStatus g;
  g.fix_type      = weeder_msgs::msg::GnssStatus::FIX_NONE;
  g.latitude_deg  = 0.0;
  g.longitude_deg = 0.0;
  g.hdop          = std::numeric_limits<float>::quiet_NaN();
  TelemetryExtras ex;
  ex.control_mode = "MANUAL";
  const json p    = TelemetryPayload(&g, nullptr, ex);
  EXPECT_EQ(p.at("location").at("rtkStatus"), "NONE");
  EXPECT_FALSE(p.at("location").contains("latitude"));
  EXPECT_FALSE(p.at("location").contains("hdop"));
  EXPECT_FALSE(ContainsNull(p));
}

TEST(Telemetry, RtkStatusNeverOverstates) {
  using G = weeder_msgs::msg::GnssStatus;
  EXPECT_EQ(RtkStatus(G::FIX_NONE), "NONE");
  EXPECT_EQ(RtkStatus(G::FIX_SPP), "SINGLE");
  EXPECT_EQ(RtkStatus(G::FIX_DGPS), "SINGLE");
  EXPECT_EQ(RtkStatus(G::FIX_INS), "SINGLE");
  EXPECT_EQ(RtkStatus(G::FIX_RTK_FLOAT), "FLOAT");
  EXPECT_EQ(RtkStatus(G::FIX_RTK_FIXED), "FIXED");
  EXPECT_EQ(RtkStatus(99), "NONE");
}
