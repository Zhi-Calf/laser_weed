#pragma once

// FarmDevice v1 设备协议的全部编解码与字段校验 (不依赖 rclcpp, 节点之外也能单测)。
// 协议: robot-cloud docs/IoT接口文档.md (2026-10-12, schema "farm-device", schemaRevision 1)。
// 下文 "§x.y" 都指那份文档的章节。
//
// 分工: 这里只管"报文长什么样" —— 信封、枚举、单位换算、必填/取值校验; 指令能不能执行
// (服务在不在线、规划器受不受理) 由节点调 ROS 服务判, 结果经 COMMAND_ACK 回给平台。
//
// 时间由调用方传入, 单测时可以固定。

#include <chrono>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vehicle_interfaces/msg/chassis_state.hpp>
#include <vehicle_interfaces/msg/gnss_status.hpp>
#include <vector>

namespace comm_gateway::farm {

using json      = nlohmann::json;
using TimePoint = std::chrono::system_clock::time_point;

inline constexpr const char* kSchema         = "farm-device";
inline constexpr int kSchemaRevision         = 1;
inline constexpr int kContractRevision       = 1;
inline constexpr const char* kDriverVersion  = "comm_gateway/0.2.0";
inline constexpr size_t kMaxIdLength         = 64;

// ---- 消息类型 (§6, §7) ----
namespace type {
// 下行
inline constexpr const char* kModeSet            = "MODE_SET";
inline constexpr const char* kCalibrationCommand = "CALIBRATION_COMMAND";
// 上行
inline constexpr const char* kDeviceOnline      = "DEVICE_ONLINE";
inline constexpr const char* kDeviceOffline     = "DEVICE_OFFLINE";
inline constexpr const char* kTelemetry         = "TELEMETRY";
inline constexpr const char* kCommandAck        = "COMMAND_ACK";
inline constexpr const char* kFaultEvent        = "FAULT_EVENT";
inline constexpr const char* kModeStatus        = "MODE_STATUS";
inline constexpr const char* kCalibrationStatus = "CALIBRATION_STATUS";
}   // namespace type

// 21 类下行指令之一 (§6 表)
bool IsDownstreamType(const std::string& message_type);

// ---- COMMAND_ACK (§7.4) ----
namespace stage {
inline constexpr const char* kReceived  = "RECEIVED";
inline constexpr const char* kExecuting = "EXECUTING";
inline constexpr const char* kCompleted = "COMPLETED";
inline constexpr const char* kRejected  = "REJECTED";
inline constexpr const char* kFailed    = "FAILED";
}   // namespace stage

// ACK 的 code (§7.4 只规定长度 0–64, 取值由设备定义; 平台据此区分失败原因)
namespace code {
inline constexpr const char* kOk                 = "OK";
inline constexpr const char* kInvalidMessage     = "INVALID_MESSAGE";      // 信封或 payload 不合规
inline constexpr const char* kExpired            = "EXPIRED";              // 收到时 expiresAt 已过
inline constexpr const char* kUnsupported        = "UNSUPPORTED";          // 本设备还没有这项能力
inline constexpr const char* kDeviceRejected     = "DEVICE_REJECTED";      // 目标 ROS 服务拒绝 (原因在 message)
inline constexpr const char* kServiceUnavailable = "SERVICE_UNAVAILABLE";  // 目标 ROS 服务不在线
inline constexpr const char* kTimeout            = "TIMEOUT";              // 目标 ROS 服务应答超时
}   // namespace code

// ---- 设备 MQTT Topic (§4.1) ----
struct Topics {
  std::string control_downstream;   // 订阅: 全部下行指令
  std::string status;               // DEVICE_ONLINE / DEVICE_OFFLINE / MODE_STATUS / CALIBRATION_STATUS
  std::string control_upstream;     // COMMAND_ACK
  std::string telemetry;            // TELEMETRY
  std::string fault;                // FAULT_EVENT (thing.event.post)

  static Topics For(const std::string& product_key, const std::string& device_name);
};

// ---- MQTT 认证 (§2.2) ----
std::string MqttClientId(const std::string& product_key, const std::string& device_name);
std::string MqttUsername(const std::string& product_key, const std::string& device_name);
// lowercase_hex(HMAC_SHA256(deviceSecret, "clientId…deviceName…deviceSecret…productKey…"))
std::string MqttPassword(const std::string& product_key, const std::string& device_name, const std::string& device_secret);

// deviceId: 无前导零的正整数十进制字符串, 最多 19 位且 <= INT64_MAX (§5.1)
bool IsValidDeviceId(const std::string& device_id);

// ---- 时间 (§5.2): RFC3339 ----
std::string FormatRfc3339(TimePoint t);   // "2026-10-12T08:00:00.000Z"
// 接受 Z 或 ±HH:MM, 小数秒可有可无。失败返回 nullopt。
std::optional<TimePoint> ParseRfc3339(const std::string& text);

// 随机标识: prefix + "-" + 32 位十六进制 (总长 <= 64)
std::string NewId(const std::string& prefix);

// ---------------------------------------------------------------------------
// 下行: 信封 (§5.1)
// ---------------------------------------------------------------------------

struct Command {
  std::string command_id;
  std::string trace_id;
  std::string message_type;
  TimePoint expires_at;
  json payload = json::object();
};

enum class ParseResult {
  kOk,
  kDrop,     // 取不到 commandId 或不是发给本设备的: 无法 / 不应回 ACK, 只记日志
  kReject,   // 能回 ACK: 以 code + err 回 REJECTED
};

// 校验: 只允许 10 个信封字段; schema / schemaRevision / deviceId 匹配; commandId 非空 <= 64;
// eventId 为 null; messageType 为 21 类下行之一; timestamp / expiresAt 为 RFC3339 且未过期;
// payload 为对象。kReject 时 out.command_id / out.trace_id 已尽量填好, 供回 ACK。
ParseResult ParseCommand(const std::string& raw, const std::string& device_id, TimePoint now, Command& out, std::string& code, std::string& err);

// ---------------------------------------------------------------------------
// 下行: payload
// ---------------------------------------------------------------------------

// MODE_SET (§6.7)
struct ModeSet {
  std::string mode;   // MANUAL / UNMANNED / WORK / PLANTING / CALIBRATION
  std::optional<int64_t> task_id;
  std::optional<std::string> work_mode;
  std::optional<std::string> deadman_token;
};
bool ParseModeSet(const json& payload, ModeSet& out, std::string& err);

// CALIBRATION_COMMAND (§6.8)
struct CalibrationCommand {
  std::string action;   // START / COMMIT / CANCEL / RESET
  std::string module_code;
  std::string calibration_type;
  json parameters = json::object();
  std::string deadman_token;
};
bool ParseCalibrationCommand(const json& payload, CalibrationCommand& out, std::string& err);

// 本机标定的 FarmDevice 编码 <-> CalibrationCommand.srv。
// 平台的标定项目编码由产品定义决定, 这里先约定 moduleCode "GNSS" + calibrationType "GNSS_FULL"。
inline constexpr const char* kCalibrationModuleGnss   = "GNSS";
inline constexpr const char* kCalibrationTypeGnssFull = "GNSS_FULL";
std::optional<uint8_t> CalibrationTypeToRos(const std::string& calibration_type);
std::string CalibrationTypeFromRos(uint8_t calibration_type);
// START -> ACTION_START, CANCEL -> ACTION_CANCEL, COMMIT -> ACTION_SAVE; RESET 没有对应动作 -> nullopt
std::optional<uint8_t> CalibrationActionToRos(const std::string& action);

// 运行模式 <-> WorkMode.msg (0 手动 / 1 无人 / 2 标定)。WORK / PLANTING 本机没有对应模式 -> nullopt
std::optional<uint8_t> ModeToRos(const std::string& mode);
std::string ModeFromRos(uint8_t work_mode);

// ---------------------------------------------------------------------------
// 上行: 信封与 payload
// ---------------------------------------------------------------------------

struct UpstreamHeader {
  std::string device_id;
  std::string message_type;
  std::string event_id;
  std::string trace_id;
  std::string timestamp;
  std::optional<std::string> command_id;   // 仅 COMMAND_ACK 带, 其他上行为 null
};
std::string EncodeUpstream(const UpstreamHeader& header, json payload);

json DeviceOnlinePayload();
json DeviceOfflinePayload(const std::string& reason_code);

json AckPayload(const std::string& ack_stage, const std::string& ack_code, const std::string& message, std::optional<json> result = std::nullopt);

json FaultEventPayload(const std::string& module_code, const std::string& fault_code, bool active, const std::string& raw_message, json values);

json ModeStatusPayload(const std::string& mode, const std::string& state, const std::optional<std::string>& related_command_id, const std::optional<int64_t>& task_id,
                       const std::string& message);

json CalibrationStatusPayload(const std::string& calibration_type, const std::string& state, const std::optional<std::string>& related_command_id, const std::string& message,
                              std::optional<json> result = std::nullopt);

// TELEMETRY (§7.3)。gnss / chassis 传 nullptr 表示没有或已过期 -> 对应字段整体省略
// (§7.3.1: 已声明字段显式 null 会被拒收, 所以不能发 null)。
struct TelemetryExtras {
  std::string control_mode;   // ModeFromRos(当前 /work_mode)
};
json TelemetryPayload(const vehicle_interfaces::msg::GnssStatus* gnss,
                      const vehicle_interfaces::msg::ChassisState* chassis,
                      const TelemetryExtras& extras);

// GnssStatus.fix_type -> rtkStatus (NONE / SINGLE / FLOAT / FIXED)
std::string RtkStatus(uint8_t fix_type);

}   // namespace comm_gateway::farm
