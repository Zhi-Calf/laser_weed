#pragma once

#include <cstdint>

namespace gnss_localization {

// ============================================================
// GNSS 定位模块统一故障码 (纯故障 ID, 无等级)
// ============================================================
//
// 编码规则: 5 位十进制, 前 3 位是模块号, 后 2 位是模块内故障序号。
//   103 = gnss_driver           (GNSS 驱动)
//   106 = ntrip_client_node     (外置 NTRIP 差分客户端)
//   107 = sdk_client_node       (千寻 SDK 差分客户端)
//
// 故障码只是"原因标签", 供 APP/云端展示; 不含等级。
// 决策动作由心跳 status 表达。
namespace gnss_driver {
enum FaultCode : uint32_t {
  FAULT_NONE          = 10300,
  FAULT_GIAVP_TIMEOUT = 10301,   // GIAVP 数据中断/超时
  FAULT_STATUS_LOW    = 10302,   // 解算状态低于 RTK 固定解
  FAULT_SERIAL        = 10303,   // 串口打开失败/掉线
};
}   // namespace gnss_driver

namespace ntrip_client {
enum FaultCode : uint32_t {
  FAULT_NONE         = 10600,
  FAULT_SERIAL       = 10601,   // 串口持续打开/读取失败
  FAULT_GGA_TIMEOUT  = 10602,   // 持续无 GPGGA 数据 (GGA 超时)
  FAULT_CONNECT      = 10603,   // NTRIP 持续连接失败
  FAULT_RTCM_TIMEOUT = 10604,   // 持续无 RTCM 数据 (RTCM 超时)
};
}   // namespace ntrip_client

namespace sdk_client {
enum FaultCode : uint32_t {
  FAULT_NONE             = 10700,
  FAULT_SERIAL           = 10701,   // 串口持续打开/读取失败
  FAULT_GGA_TIMEOUT      = 10702,   // 持续无 GPGGA 数据 (GGA 超时)
  FAULT_RTCM_TIMEOUT     = 10703,   // 持续无 RTCM 数据 (RTCM 超时)
  FAULT_SDK_INIT_FAIL    = 10704,   // 千寻 SDK 初始化失败
  FAULT_SDK_AUTH_TIMEOUT = 10705,   // 千寻 SDK 认证超时
  FAULT_SDK_NOSR_FAIL    = 10706,   // 千寻 SDK 启动 NOSR 能力失败
  FAULT_SDK_ERROR        = 10707,   // SDK 内部错误 (服务停止/网络不可用等)
  FAULT_SERIAL_WRITE     = 10708,   // 串口写 RTCM 失败 (收 RTCM 但写不进串口)
};
}   // namespace sdk_client

}   // namespace gnss_localization