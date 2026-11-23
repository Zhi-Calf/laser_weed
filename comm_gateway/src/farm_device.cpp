#include "comm_gateway/farm_device.hpp"

#include <openssl/evp.h>
#include <openssl/hmac.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <limits>
#include <random>
#include <set>

namespace comm_gateway::farm {

namespace {

// ---- 字段校验: 失败时 err 带字段名 ----

bool IsBlank(const std::string& s) {
  return std::all_of(s.begin(), s.end(), [](unsigned char c) {
    return std::isspace(c);
  });
}

// §6/§7 表里 "长度1–64；不得为空白" 的字符串
bool GetIdString(const json& j, const char* key, std::string& out, std::string& err) {
  if (!j.contains(key) || !j.at(key).is_string()) {
    err = std::string("字段 ") + key + (j.contains(key) ? " 应为字符串" : " 缺失");
    return false;
  }
  out = j.at(key).get<std::string>();
  if (out.empty() || out.size() > kMaxIdLength || IsBlank(out)) {
    err = std::string("字段 ") + key + " 长度应为 1–64 且不能为空白";
    return false;
  }
  return true;
}

bool GetEnum(const json& j, const char* key, const std::set<std::string>& allowed, std::string& out, std::string& err) {
  if (!j.contains(key) || !j.at(key).is_string()) {
    err = std::string("字段 ") + key + (j.contains(key) ? " 应为字符串" : " 缺失");
    return false;
  }
  out = j.at(key).get<std::string>();
  if (!allowed.count(out)) {
    err = std::string("字段 ") + key + " 取值无效: " + out;
    return false;
  }
  return true;
}

// payload 字段集合是封闭的 (§5.3): 出现未声明的键直接拒收
bool OnlyKeys(const json& j, const std::set<std::string>& allowed, std::string& err) {
  for (const auto& item : j.items()) {
    if (!allowed.count(item.key())) {
      err = "不支持的字段 " + item.key();
      return false;
    }
  }
  return true;
}

// float32 直接转 double 会带出 0.6000000238418579 这样的尾巴; 按 6 位小数取整后 nlohmann 输出最短表示。
double Round6(double v) {
  return std::round(v * 1e6) / 1e6;
}

// NaN / Inf 经 nlohmann 会被写成 null, 而已声明字段的 null 会被网关拒收: 非有限值直接省略。
void PutFinite(json& obj, const char* key, double v) {
  if (std::isfinite(v)) {
    obj[key] = Round6(v);
  }
}

const std::set<std::string> kDownstreamTypes = {
    "CONTROL_LOCK", "CONTROL_RELEASE", "CONTROL_MOVE", "EMERGENCY_STOP", "PTO_SET",   "IMPLEMENT_SET", "MODE_SET",
    "CALIBRATION_COMMAND", "TASK_DISPATCH", "TASK_START", "TASK_PAUSE", "TASK_RESUME", "TASK_CANCEL", "CONFIG_GET",
    "CONFIG_SET", "OTA_START", "OTA_CANCEL", "LOG_REQUEST", "FRP_START", "FRP_STOP", "VIDEO_CONTROL",
};

const std::set<std::string> kEnvelopeKeys = {
    "commandId", "eventId", "traceId", "schema", "schemaRevision", "deviceId", "messageType", "timestamp", "expiresAt", "payload",
};

const std::set<std::string> kModes        = {"MANUAL", "UNMANNED", "WORK", "PLANTING", "CALIBRATION"};
const std::set<std::string> kWorkModes    = {"TRACKING", "HARROWING", "GRASS_GRID"};
const std::set<std::string> kCalibActions = {"START", "COMMIT", "CANCEL", "RESET"};

}   // namespace

bool IsDownstreamType(const std::string& message_type) {
  return kDownstreamTypes.count(message_type) > 0;
}

Topics Topics::For(const std::string& product_key, const std::string& device_name) {
  const std::string prefix = "/sys/" + product_key + "/" + device_name;
  Topics t;
  t.control_downstream = prefix + "/control/downstream";
  t.status             = prefix + "/status/upstream";
  t.control_upstream   = prefix + "/control/upstream";
  t.telemetry          = prefix + "/telemetry/upstream";
  t.fault              = prefix + "/thing.event.post/upstream";
  return t;
}

std::string MqttClientId(const std::string& product_key, const std::string& device_name) {
  return product_key + "." + device_name;
}

std::string MqttUsername(const std::string& product_key, const std::string& device_name) {
  return device_name + "&" + product_key;
}

std::string MqttPassword(const std::string& product_key, const std::string& device_name, const std::string& device_secret) {
  const std::string sign = "clientId" + MqttClientId(product_key, device_name) + "deviceName" + device_name + "deviceSecret" + device_secret + "productKey" + product_key;
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int len = 0;
  HMAC(EVP_sha256(), device_secret.data(), static_cast<int>(device_secret.size()), reinterpret_cast<const unsigned char*>(sign.data()), sign.size(), digest, &len);
  std::string hex;
  hex.reserve(len * 2);
  char buf[3];
  for (unsigned int i = 0; i < len; ++i) {
    std::snprintf(buf, sizeof(buf), "%02x", digest[i]);
    hex += buf;
  }
  return hex;
}

bool IsValidDeviceId(const std::string& device_id) {
  if (device_id.empty() || device_id.size() > 19 || device_id[0] == '0') {
    return false;
  }
  if (!std::all_of(device_id.begin(), device_id.end(), [](unsigned char c) {
        return std::isdigit(c);
      })) {
    return false;
  }
  // 19 位时按字典序与 INT64_MAX 比 (位数相同, 字典序即数值序)
  return device_id.size() < 19 || device_id <= "9223372036854775807";
}

std::string FormatRfc3339(TimePoint t) {
  const auto ms       = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch()).count() % 1000;
  const std::time_t s = std::chrono::system_clock::to_time_t(t);
  std::tm tm{};
  gmtime_r(&s, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm);
  char out[48];
  std::snprintf(out, sizeof(out), "%s.%03dZ", buf, static_cast<int>(ms));
  return out;
}

std::optional<TimePoint> ParseRfc3339(const std::string& text) {
  int y = 0, mo = 0, d = 0, h = 0, mi = 0, s = 0, consumed = 0;
  if (std::sscanf(text.c_str(), "%4d-%2d-%2dT%2d:%2d:%2d%n", &y, &mo, &d, &h, &mi, &s, &consumed) != 6 || consumed != 19) {
    return std::nullopt;
  }
  if (mo < 1 || mo > 12 || d < 1 || d > 31 || h > 23 || mi > 59 || s > 60) {
    return std::nullopt;
  }
  size_t pos = 19;
  std::chrono::nanoseconds frac{0};
  if (pos < text.size() && text[pos] == '.') {
    ++pos;
    const size_t start = pos;
    int64_t scale      = 100000000;
    while (pos < text.size() && std::isdigit(static_cast<unsigned char>(text[pos]))) {
      frac += std::chrono::nanoseconds((text[pos] - '0') * scale);
      scale /= 10;
      ++pos;
    }
    if (pos == start) {
      return std::nullopt;
    }
  }
  int offset_s = 0;
  if (pos < text.size() && (text[pos] == 'Z' || text[pos] == 'z')) {
    ++pos;
  } else if (pos + 6 == text.size() && (text[pos] == '+' || text[pos] == '-') && text[pos + 3] == ':') {
    int oh = 0, om = 0;
    if (std::sscanf(text.c_str() + pos + 1, "%2d:%2d", &oh, &om) != 2 || oh > 23 || om > 59) {
      return std::nullopt;
    }
    offset_s = (oh * 3600 + om * 60) * (text[pos] == '+' ? 1 : -1);
    pos += 6;
  } else {
    return std::nullopt;
  }
  if (pos != text.size()) {
    return std::nullopt;
  }
  std::tm tm{};
  tm.tm_year = y - 1900;
  tm.tm_mon  = mo - 1;
  tm.tm_mday = d;
  tm.tm_hour = h;
  tm.tm_min  = mi;
  tm.tm_sec  = s;
  const std::time_t utc = timegm(&tm) - offset_s;
  return std::chrono::system_clock::from_time_t(utc) + std::chrono::duration_cast<std::chrono::system_clock::duration>(frac);
}

std::string NewId(const std::string& prefix) {
  thread_local std::mt19937_64 rng{std::random_device{}()};
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%016llx%016llx", static_cast<unsigned long long>(rng()), static_cast<unsigned long long>(rng()));
  return prefix + "-" + buf;
}

// ---------------------------------------------------------------------------
// 下行
// ---------------------------------------------------------------------------

ParseResult ParseCommand(const std::string& raw, const std::string& device_id, TimePoint now, Command& out, std::string& code, std::string& err) {
  const json j = json::parse(raw, nullptr, false);
  if (j.is_discarded() || !j.is_object()) {
    err = "不是合法 JSON 对象";
    return ParseResult::kDrop;
  }
  // 先取回 ACK 必需的 commandId 与 deviceId; 取不到就无从回应。
  if (!j.contains("commandId") || !j.at("commandId").is_string() || j.at("commandId").get<std::string>().empty() || j.at("commandId").get<std::string>().size() > kMaxIdLength) {
    err = "commandId 缺失或无效";
    return ParseResult::kDrop;
  }
  out.command_id = j.at("commandId").get<std::string>();
  if (!j.contains("deviceId") || !j.at("deviceId").is_string() || j.at("deviceId").get<std::string>() != device_id) {
    err = "deviceId 与本设备 (" + device_id + ") 不符: " + (j.contains("deviceId") ? j.at("deviceId").dump() : "缺失");
    return ParseResult::kDrop;
  }
  if (j.contains("messageType") && j.at("messageType").is_string()) {
    out.message_type = j.at("messageType").get<std::string>();
  }
  // traceId 无效时仍要回 ACK: 用 commandId 代替, 让平台能对上号
  std::string trace_err;
  if (!GetIdString(j, "traceId", out.trace_id, trace_err)) {
    out.trace_id = out.command_id;
  }

  code = code::kInvalidMessage;
  if (!OnlyKeys(j, kEnvelopeKeys, err)) {
    err = "信封含" + err;
    return ParseResult::kReject;
  }
  if (!trace_err.empty()) {
    err = trace_err;
    return ParseResult::kReject;
  }
  if (!j.contains("schema") || j.at("schema") != kSchema || !j.contains("schemaRevision") || !j.at("schemaRevision").is_number_integer() || j.at("schemaRevision").get<int64_t>() != kSchemaRevision) {
    err = std::string("schema 应为 ") + kSchema + " / schemaRevision 应为 1";
    return ParseResult::kReject;
  }
  if (j.contains("eventId") && !j.at("eventId").is_null()) {
    err = "下行 eventId 必须为 null";
    return ParseResult::kReject;
  }
  if (!IsDownstreamType(out.message_type)) {
    err = "messageType 不是下行指令: " + (j.contains("messageType") ? j.at("messageType").dump() : std::string("缺失"));
    return ParseResult::kReject;
  }
  if (!j.contains("timestamp") || !j.at("timestamp").is_string() || !ParseRfc3339(j.at("timestamp").get<std::string>())) {
    err = "timestamp 应为 RFC3339 字符串";
    return ParseResult::kReject;
  }
  std::optional<TimePoint> expires;
  if (!j.contains("expiresAt") || !j.at("expiresAt").is_string() || !(expires = ParseRfc3339(j.at("expiresAt").get<std::string>()))) {
    err = "下行 expiresAt 必填且应为 RFC3339 字符串";
    return ParseResult::kReject;
  }
  out.expires_at = *expires;
  if (!j.contains("payload") || !j.at("payload").is_object()) {
    err = "payload 应为对象";
    return ParseResult::kReject;
  }
  out.payload = j.at("payload");
  if (now > out.expires_at) {
    code = code::kExpired;
    err  = "指令已过期 (expiresAt=" + j.at("expiresAt").get<std::string>() + ", 设备时间=" + FormatRfc3339(now) + ")";
    return ParseResult::kReject;
  }
  code.clear();
  return ParseResult::kOk;
}

bool ParseModeSet(const json& payload, ModeSet& out, std::string& err) {
  if (!OnlyKeys(payload, {"mode", "taskId", "workMode", "deadmanToken"}, err) || !GetEnum(payload, "mode", kModes, out.mode, err)) {
    return false;
  }
  const bool needs_task = out.mode == "UNMANNED" || out.mode == "WORK" || out.mode == "PLANTING";
  if (!needs_task) {
    // MANUAL / CALIBRATION 连 null 键也不能带 (§6.7)
    if (payload.contains("taskId") || payload.contains("workMode")) {
      err = out.mode + " 不能携带 taskId / workMode";
      return false;
    }
  } else {
    if (!payload.contains("taskId") || !payload.at("taskId").is_number_integer() || payload.at("taskId").get<int64_t>() < 1) {
      err = out.mode + " 必须携带 taskId (>= 1 的整数)";
      return false;
    }
    out.task_id = payload.at("taskId").get<int64_t>();
    std::string work_mode;
    if (!GetEnum(payload, "workMode", kWorkModes, work_mode, err)) {
      err = out.mode + " 必须携带有效的 workMode: " + err;
      return false;
    }
    if ((out.mode == "WORK" && work_mode == "GRASS_GRID") || (out.mode == "PLANTING" && work_mode != "GRASS_GRID")) {
      err = out.mode + " 不允许 workMode " + work_mode;
      return false;
    }
    out.work_mode = work_mode;
  }
  if (payload.contains("deadmanToken")) {
    std::string token;
    if (!GetIdString(payload, "deadmanToken", token, err)) {
      return false;
    }
    out.deadman_token = token;
  }
  return true;
}

bool ParseCalibrationCommand(const json& payload, CalibrationCommand& out, std::string& err) {
  if (!OnlyKeys(payload, {"action", "moduleCode", "calibrationType", "parameters", "deadmanToken"}, err) || !GetEnum(payload, "action", kCalibActions, out.action, err) ||
      !GetIdString(payload, "moduleCode", out.module_code, err) || !GetIdString(payload, "calibrationType", out.calibration_type, err) ||
      !GetIdString(payload, "deadmanToken", out.deadman_token, err)) {
    return false;
  }
  if (!payload.contains("parameters") || !payload.at("parameters").is_object()) {
    err = "字段 parameters 必填且应为对象";
    return false;
  }
  for (const auto& item : payload.at("parameters").items()) {
    if (item.value().is_object() || item.value().is_array()) {
      err = "parameters." + item.key() + " 只能是 string / number / boolean / null";
      return false;
    }
  }
  out.parameters = payload.at("parameters");
  return true;
}

std::optional<uint8_t> CalibrationTypeToRos(const std::string& calibration_type) {
  if (calibration_type == kCalibrationTypeGnssFull) {
    return 0;   // CalibrationCommand::CALIB_GNSS_FULL
  }
  return std::nullopt;
}

std::string CalibrationTypeFromRos(uint8_t calibration_type) {
  return calibration_type == 0 ? kCalibrationTypeGnssFull : "TYPE_" + std::to_string(calibration_type);
}

std::optional<uint8_t> CalibrationActionToRos(const std::string& action) {
  // 取值见 CalibrationCommand.srv 的 ACTION_*
  if (action == "START") {
    return 0;
  }
  if (action == "CANCEL") {
    return 1;
  }
  if (action == "COMMIT") {
    return 3;
  }
  return std::nullopt;
}

std::optional<uint8_t> ModeToRos(const std::string& mode) {
  // 取值见 WorkMode.msg 的 MODE_*
  if (mode == "MANUAL") {
    return 0;
  }
  if (mode == "UNMANNED") {
    return 1;
  }
  if (mode == "CALIBRATION") {
    return 2;
  }
  return std::nullopt;
}

std::string ModeFromRos(uint8_t work_mode) {
  switch (work_mode) {
    case 0:
      return "MANUAL";
    case 1:
      return "UNMANNED";
    case 2:
      return "CALIBRATION";
  }
  return "MANUAL";
}

// ---------------------------------------------------------------------------
// 上行
// ---------------------------------------------------------------------------

std::string EncodeUpstream(const UpstreamHeader& header, json payload) {
  json j;
  j["commandId"]      = header.command_id ? json(*header.command_id) : json(nullptr);
  j["eventId"]        = header.event_id;
  j["traceId"]        = header.trace_id;
  j["schema"]         = kSchema;
  j["schemaRevision"] = kSchemaRevision;
  j["deviceId"]       = header.device_id;
  j["messageType"]    = header.message_type;
  j["timestamp"]      = header.timestamp;
  j["expiresAt"]      = nullptr;
  j["payload"]        = payload.is_object() ? std::move(payload) : json::object();
  return j.dump();
}

json DeviceOnlinePayload() {
  return {{"transport", "WAN"}, {"driverVersion", kDriverVersion}, {"contractRevision", kContractRevision}};
}

json DeviceOfflinePayload(const std::string& reason_code) {
  json p          = DeviceOnlinePayload();
  p["reasonCode"] = reason_code;
  return p;
}

json AckPayload(const std::string& ack_stage, const std::string& ack_code, const std::string& message, std::optional<json> result) {
  json p = {{"stage", ack_stage}, {"code", ack_code.substr(0, 64)}, {"message", message.substr(0, 500)}};
  if (result) {
    p["result"] = std::move(*result);
  }
  return p;
}

json FaultEventPayload(const std::string& module_code, const std::string& fault_code, bool active, const std::string& raw_message, json values) {
  json p = {{"moduleCode", module_code.substr(0, 64)}, {"faultCode", fault_code.substr(0, 64)}, {"active", active}};
  if (!raw_message.empty()) {
    p["rawMessage"] = raw_message.substr(0, 1000);
  }
  if (values.is_object() && !values.empty()) {
    p["values"] = std::move(values);
  }
  return p;
}

json ModeStatusPayload(const std::string& mode, const std::string& state, const std::optional<std::string>& related_command_id, const std::optional<int64_t>& task_id,
                       const std::string& message) {
  json p = {{"mode", mode}, {"state", state}};
  if (related_command_id) {
    p["relatedCommandId"] = *related_command_id;
  }
  if (task_id) {
    p["taskId"] = *task_id;
  }
  if (!message.empty()) {
    p["message"] = message.substr(0, 500);
  }
  return p;
}

json CalibrationStatusPayload(const std::string& calibration_type, const std::string& state, const std::optional<std::string>& related_command_id, const std::string& message,
                              std::optional<json> result) {
  json p = {{"calibrationType", calibration_type}, {"state", state}};
  if (related_command_id) {
    p["relatedCommandId"] = *related_command_id;
  }
  if (!message.empty()) {
    p["message"] = message.substr(0, 500);
  }
  if (result) {
    p["result"] = std::move(*result);
  }
  return p;
}

std::string RtkStatus(uint8_t fix_type) {
  using vehicle_interfaces::msg::GnssStatus;
  switch (fix_type) {
    case GnssStatus::FIX_RTK_FIXED:
      return "FIXED";
    case GnssStatus::FIX_RTK_FLOAT:
      return "FLOAT";
    case GnssStatus::FIX_SPP:
    case GnssStatus::FIX_DGPS:
    case GnssStatus::FIX_INS:
      // 枚举只有四档: 非 RTK 的有解一律按 SINGLE, 绝不往高报。原始值另见扩展字段 gnssFixType。
      return "SINGLE";
  }
  return "NONE";
}

json TelemetryPayload(const vehicle_interfaces::msg::GnssStatus* gnss,
                      const vehicle_interfaces::msg::ChassisState* chassis,
                      const TelemetryExtras& extras) {
  json p = json::object();

  if (gnss) {
    json loc         = json::object();
    loc["rtkStatus"] = RtkStatus(gnss->fix_type);
    loc["satellites"] = gnss->num_satellites;
    // 无解时经纬度不可信: 不报坐标, 只报"无解"本身
    if (gnss->fix_type != vehicle_interfaces::msg::GnssStatus::FIX_NONE &&
        std::abs(gnss->latitude_deg) <= 90.0 &&
        std::abs(gnss->longitude_deg) <= 180.0) {
      PutFinite(loc, "longitude", gnss->longitude_deg);
      PutFinite(loc, "latitude", gnss->latitude_deg);
      PutFinite(loc, "altitude", gnss->altitude_m);
    }
    const double speed_ms = std::max(0.0, static_cast<double>(gnss->speed_kmh) / 3.6);
    PutFinite(loc, "speed", speed_ms);
    if (std::isfinite(gnss->heading_deg)) {
      double dir = std::fmod(static_cast<double>(gnss->heading_deg), 360.0);
      if (dir < 0.0) {
        dir += 360.0;
      }
      loc["direction"] = Round6(dir);
      p["heading"]     = Round6(dir);
    }
    if (gnss->hdop >= 0.0f) {
      PutFinite(loc, "hdop", gnss->hdop);
    }
    if (gnss->diff_age >= 0.0f) {
      PutFinite(loc, "differentialAge", gnss->diff_age);
    }
    p["location"]    = std::move(loc);
    PutFinite(p, "speed", speed_ms);
    p["gnssFixType"] = gnss->fix_type;   // 扩展: GnssStatus.fix_type 原值 (0 无 1 单点 2 差分 4 固定 5 浮动 6 惯导)
  }

  if (chassis) {
    json motion = json::object();
    PutFinite(motion, "linearSpeed", chassis->speed_m_s);
    p["motion"] = motion;
    PutFinite(p, "linearSpeed", chassis->speed_m_s);

    // 只有新鲜的底盘帧才走到这里；过期时 chassis 为 nullptr，安全字段整组省略。
    const bool estop_released = !chassis->estop;
    p["safety"]               = {{"emergencyStopReleased", estop_released}};
    p["emergencyStopReleased"] = estop_released;

    PutFinite(p, "battery", chassis->bms_soc);
  }

  p["controlMode"] = extras.control_mode;
  p["link"]        = {{"transport", "WAN"}};
  return p;
}

}   // namespace comm_gateway::farm
