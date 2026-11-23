#pragma once

#include <cstdint>

namespace weeder_comm_gateway {

// ============================================================
// APP 通信桥故障码 (纯故障 ID, 无等级)
// ============================================================
//
// 编码规则: 5 位十进制, 前 3 位是模块号, 后 2 位是模块内故障序号。
//   108 = mqtt_bridge (robot-cloud IoT 网关 MQTT 桥, FarmDevice v1)
//   109 = http_bridge (APP HTTP 服务)
//
// 故障码只是"原因标签", 供 APP/云端展示; 决策动作由心跳 status 表达。
namespace mqtt_bridge {
enum FaultCode : uint32_t {
  FAULT_NONE              = 10800,
  FAULT_BROKER_DISCONNECT = 10801,   // 未连上 / 断开 MQTT broker
  FAULT_CONFIG            = 10802,   // 设备身份参数无效 (device_id 等), 未连接网关
};
}   // namespace mqtt_bridge

namespace http_bridge {
enum FaultCode : uint32_t {
  FAULT_NONE         = 10900,
  FAULT_SERVER_START = 10901,   // HTTP 服务起不来 (端口被占 / 证书读不到)
};
}   // namespace http_bridge

}   // namespace weeder_comm_gateway
