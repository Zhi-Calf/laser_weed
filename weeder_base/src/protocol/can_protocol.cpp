#include "weeder_base/protocol/can_protocol.hpp"

#include <algorithm>
#include <cstring>

namespace weeder_base {
namespace protocol {

namespace {

// 位提取只允许访问经典 CAN 帧的 64 个 bit。
// 检查位提取范围是否落在经典 CAN 帧内。
// 输入:
//   data      : 8 字节 CAN 数据
//   start_bit : 起始位，从 0 开始
//   len       : 提取位数
// 返回:
//   true = 指针有效且范围位于 0..63 bit
bool IsValidBitRange(const uint8_t* data, uint8_t start_bit, uint8_t len) {
  return data != nullptr && len != 0 && len <= 32 && start_bit < 64 && static_cast<uint16_t>(start_bit) + len <= 64;
}

}   // namespace

// ============================================================
// 字节工具 (Intel / little-endian)
// ============================================================
// 从两字节小端数据读取无符号 16 位整数。
// 输入:
//   p : 至少两字节的数据指针
// 返回:
//   小端解析结果
uint16_t CanProtocol::readU16LE(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

// 从两字节小端数据读取有符号 16 位整数。
// 输入:
//   p : 至少两字节的数据指针
// 返回:
//   小端解析结果
int16_t CanProtocol::readI16LE(const uint8_t* p) {
  return static_cast<int16_t>(readU16LE(p));
}

// 将无符号 16 位整数写成两字节小端数据。
// 输出:
//   p : 至少两字节的可写缓冲区
void CanProtocol::writeU16LE(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

// 将有符号 16 位整数写成两字节小端数据。
// 输出:
//   p : 至少两字节的可写缓冲区
void CanProtocol::writeI16LE(uint8_t* p, int16_t v) {
  writeU16LE(p, static_cast<uint16_t>(v));
}

// 通用位提取 (Intel 格式: 起始bit所在字节的对应bit为最低位, 小端拼接)
// 从 Intel 格式位域中提取无符号数值。
// 输入:
//   data      : CAN 数据
//   start_bit : 起始位
//   len       : 位数
// 返回:
//   提取值；范围非法时返回 0
uint32_t CanProtocol::extractUnsigned(const uint8_t* data, uint8_t start_bit, uint8_t len) {
  if (!IsValidBitRange(data, start_bit, len)) {
    return 0;
  }
  uint32_t raw = 0;
  for (uint8_t i = 0; i < len; ++i) {
    uint8_t bit      = start_bit + i;
    uint8_t byte_idx = bit / 8;
    uint8_t bit_idx  = bit % 8;
    if (data[byte_idx] & (1u << bit_idx)) {
      raw |= (1u << i);
    }
  }
  return raw;
}

// 从 Intel 格式位域中提取二进制补码有符号数值。
// 输入:
//   data      : CAN 数据
//   start_bit : 起始位
//   len       : 位数
// 返回:
//   符号扩展后的数值；范围非法时返回 0
int32_t CanProtocol::extractSigned(const uint8_t* data, uint8_t start_bit, uint8_t len) {
  if (!IsValidBitRange(data, start_bit, len)) {
    return 0;
  }
  uint32_t raw = extractUnsigned(data, start_bit, len);
  // 符号扩展
  if (len == 32) {
    return static_cast<int32_t>(raw);
  }
  if (raw & (1u << (len - 1))) {
    raw |= ~((1u << len) - 1u);
    return static_cast<int32_t>(raw);
  }
  return static_cast<int32_t>(raw);
}

// ============================================================
// 编码: ADCU -> VCU
// ============================================================
// 将 ADCU 控制指令编码为扩展帧。
// 输入:
//   cmd : 速度、转角、模式和灯光控制字段
// 返回:
//   CAN ID、DLC、扩展帧标志和 8 字节数据
CanFrame CanProtocol::encodeAdcuCmd01(const AdcuCommand01& cmd) {
  CanFrame f{};
  f.id          = can_id::ADCU_TO_VCU_01;
  f.dlc         = 8;
  f.is_extended = true;
  std::memset(f.data, 0, 8);

  // Byte0 位域 (start bit 0, Intel 逐位填充):
  //   bit0,1=GearCmd   bit2,3=SteerModeCmd
  //   bit4=ReleaseBrakeCmd   bit5=EStopCmd
  //   bit6=ResetCmd          bit7=CtrlModeCmd
  // ⚠️ 与接收帧 State_01 的 bit4~6 顺序相反:
  //    State_01 是 bit4=EStop, bit5=Reset, bit6=ReleaseBrake
  f.data[0] = static_cast<uint8_t>(
    (cmd.gear & 0x03) | ((cmd.steer_mode & 0x03) << 2) | ((cmd.release_brake & 0x01) << 4) |   // bit 4: ReleaseBrakeCmd
    ((cmd.estop & 0x01) << 5) |                                                                // bit 5: EStopCmd
    ((cmd.reset & 0x01) << 6) |                                                                // bit 6: ResetCmd
    ((cmd.ctrl_mode & 0x01) << 7));                                                            // bit 7: CtrlModeCmd

  // Byte1~2: 目标车速 (uint16, LE, start bit 8)
  //    文档写 Value=0.001X 但 16bit 无法覆盖 0~1400 mm/s, 按 raw=mm/s 实现
  const uint16_t speed_raw = cmd.speed_raw > limits::kMaxSpeedMmps ? limits::kMaxSpeedMmps : cmd.speed_raw;
  writeU16LE(&f.data[1], speed_raw);

  // Byte3~4: 目标转向角 (int16, LE, start bit 24) 0.1°, 左正右负, ±90°
  const int16_t steer_raw = std::clamp(cmd.steer_angle, static_cast<int16_t>(-limits::kMaxSteerAngleRaw), static_cast<int16_t>(limits::kMaxSteerAngleRaw));
  writeI16LE(&f.data[3], steer_raw);

  // Byte5 位域 (start bit 40, 占 bit40~46, 7个灯光信号):
  //   bit40=Sound bit41=Green bit42=Yellow bit43=Red
  //   bit44=Headlight bit45=TurnLeft bit46=TurnRight
  f.data[5] = static_cast<uint8_t>(
    (cmd.sound & 0x01) | ((cmd.light_green & 0x01) << 1) | ((cmd.light_yellow & 0x01) << 2) | ((cmd.light_red & 0x01) << 3) | ((cmd.headlight & 0x01) << 4) | ((cmd.turn_left & 0x01) << 5) |
    ((cmd.turn_right & 0x01) << 6));

  // Byte7: AliveCounter (start bit 56)
  f.data[7] = cmd.alive_counter;

  return f;
}

// ============================================================
// 解码: VCU -> ADCU
// ============================================================

// State_01: 20ms - 底盘主状态
// 解码底盘主状态帧。
// 输入:
//   f : State_01 CAN 帧
// 输出:
//   out : 底盘主状态；失败时保持旧值
// 返回:
//   true = ID 和 DLC 合法
bool CanProtocol::decodeState01(const CanFrame& f, ChassisState01& out) {
  if (f.id != can_id::CHASSIS_STATE_01 || f.dlc < 8)
    return false;

  // 以下严格按协议文档: 起始位置(bit) + 信号长度 + Intel 字节顺序
  // Byte0 位域 (start bit 0)
  out.gear          = extractUnsigned(f.data, 0, 2);   // bit0~1
  out.steer_mode    = extractUnsigned(f.data, 2, 2);   // bit2~3
  out.estop         = extractUnsigned(f.data, 4, 1);   // bit4
  out.reset         = extractUnsigned(f.data, 5, 1);   // bit5
  out.release_brake = extractUnsigned(f.data, 6, 1);   // bit6
  out.ctrl_mode     = extractUnsigned(f.data, 7, 1);   // bit7

  // 车速 (start bit 8, 16bit, unsigned) → Byte1~2
  out.speed_raw = static_cast<uint16_t>(extractUnsigned(f.data, 8, 16));

  //    若分辨率=1 mm/s, 则改为 out.speed_raw * 0.001f
  out.speed_m_s = out.speed_raw * 0.001f;

  // 轮距 (start bit 24, 8bit, unsigned) → Byte3
  out.wheel_track_cm = static_cast<uint8_t>(extractUnsigned(f.data, 24, 8));

  // 灯光 (start bit 32, 各 1bit) → Byte4
  out.sound        = extractUnsigned(f.data, 32, 1);
  out.light_green  = extractUnsigned(f.data, 33, 1);
  out.light_yellow = extractUnsigned(f.data, 34, 1);
  out.light_red    = extractUnsigned(f.data, 35, 1);
  out.headlight    = extractUnsigned(f.data, 36, 1);
  out.turn_left    = extractUnsigned(f.data, 37, 1);
  out.turn_right   = extractUnsigned(f.data, 38, 1);

  // 转向到位 (start bit 39, 各 1bit)
  out.steer_done_fl = extractUnsigned(f.data, 39, 1);
  out.steer_done_fr = extractUnsigned(f.data, 40, 1);
  out.steer_done_rl = extractUnsigned(f.data, 41, 1);
  out.steer_done_rr = extractUnsigned(f.data, 42, 1);

  // 其他状态 (start bit 43, 各 1bit)
  out.bumper           = extractUnsigned(f.data, 43, 1);
  out.bypass           = extractUnsigned(f.data, 44, 1);
  out.battery_charging = extractUnsigned(f.data, 45, 1);
  out.battery_full     = extractUnsigned(f.data, 46, 1);

  // 生命信号 (start bit 56, 8bit) → Byte7
  out.alive_counter = static_cast<uint8_t>(extractUnsigned(f.data, 56, 8));
  return true;
}

// State_02: 100ms - 四轮电机转速 (各 uint16 LE, r/min)
// 解码四轮转速状态帧。
// 输入:
//   f : State_02 CAN 帧
// 输出:
//   out : 四轮转速；失败时保持旧值
// 返回:
//   true = ID 和 DLC 合法
bool CanProtocol::decodeState02(const CanFrame& f, ChassisState02& out) {
  if (f.id != can_id::CHASSIS_STATE_02 || f.dlc < 8)
    return false;
  out.motor_speed_fl = readU16LE(&f.data[0]);   // bit0
  out.motor_speed_fr = readU16LE(&f.data[2]);   // bit16
  out.motor_speed_rl = readU16LE(&f.data[4]);   // bit32
  out.motor_speed_rr = readU16LE(&f.data[6]);   // bit48
  return true;
}

// State_03: 100ms - 四轮转向角 (各 int16 LE, 0.1°, 左正右负)
// 解码四轮转向角状态帧。
// 输入:
//   f : State_03 CAN 帧
// 输出:
//   out : 四轮转向角；失败时保持旧值
// 返回:
//   true = ID 和 DLC 合法
bool CanProtocol::decodeState03(const CanFrame& f, ChassisState03& out) {
  if (f.id != can_id::CHASSIS_STATE_03 || f.dlc < 8)
    return false;
  out.steer_angle_fl = readI16LE(&f.data[0]);   // bit0
  out.steer_angle_fr = readI16LE(&f.data[2]);   // bit16
  out.steer_angle_rl = readI16LE(&f.data[4]);   // bit32
  out.steer_angle_rr = readI16LE(&f.data[6]);   // bit48
  return true;
}

// State_04~11: 500ms - 电机电参 (行走/转向电机, 结构相同)
//   布局: Byte0=母线电压, Byte1~2=输出电流, Byte3=温度, Byte4~5=故障码
// 判断 CAN ID 是否属于电机电参状态帧。
// 输入:
//   id : 29 位扩展帧 ID
// 返回:
//   true = State_04~11
bool CanProtocol::isMotorElecStateId(uint32_t id) {
  switch (id) {
    case can_id::CHASSIS_STATE_04:
    case can_id::CHASSIS_STATE_05:
    case can_id::CHASSIS_STATE_06:
    case can_id::CHASSIS_STATE_07:
    case can_id::CHASSIS_STATE_08:
    case can_id::CHASSIS_STATE_09:
    case can_id::CHASSIS_STATE_10:
    case can_id::CHASSIS_STATE_11:
      return true;
    default:
      return false;
  }
}

// 解码行走或转向电机电参状态帧。
// 输入:
//   f : State_04~11 CAN 帧
// 输出:
//   out : 电压、电流、温度和故障码；失败时保持旧值
// 返回:
//   true = ID 和 DLC 合法
bool CanProtocol::decodeMotorElec(const CanFrame& f, MotorElecState& out) {
  if (!isMotorElecStateId(f.id) || f.dlc < 8)
    return false;
  out.bus_voltage = f.data[0];               // bit0,  V
  out.out_current = readU16LE(&f.data[1]);   // bit8,  A
  out.motor_temp  = f.data[3];               // bit24, ℃
  out.error_code  = readU16LE(&f.data[4]);   // bit32
  return true;
}

// State_21: 500ms - BMS 基本状态 (第二版修)
//   bit0  = BMS_SOC                    %, 分辨率 1
//   bit16 = BMS_Voltage                V, Value=X*0.01
//   bit32 = BMS_Current                A, Value=X*0.1-1000 (signed, 充电为负)
//   bit48 = BMS_FullChargeCapacity     AH, Value=X*0.01
// 解码 BMS 基本状态帧。
// 输入:
//   f : State_21 CAN 帧
// 输出:
//   out : SOC、电压、电流和容量；失败时保持旧值
// 返回:
//   true = ID 和 DLC 合法
bool CanProtocol::decodeState21(const CanFrame& f, ChassisState21& out) {
  if (f.id != can_id::CHASSIS_STATE_21 || f.dlc < 8)
    return false;
  out.soc_raw                  = readU16LE(&f.data[0]);   // bit0
  out.soc                      = static_cast<float>(out.soc_raw);
  out.voltage_raw              = readU16LE(&f.data[2]);   // bit16
  out.voltage                  = out.voltage_raw * 0.01f;
  out.current_raw              = readI16LE(&f.data[4]);   // bit32
  out.current                  = out.current_raw * 0.1f - 1000.0f;
  out.full_charge_capacity_raw = readU16LE(&f.data[6]);   // bit48
  out.full_charge_capacity     = out.full_charge_capacity_raw * 0.01f;
  return true;
}

// State_22: 500ms - BMS 故障与电压 (第二版修)
//   bit0  = BMS_BatDischargeCycle   放电循环次数, 次
//   bit16 = BMS_ErrorCode       电池故障码 (第二版扩为 16bit)
//   bit32 = BMS_MaxVoltage      电芯最高电压, mV
//   bit48 = System_Voltage      VCU 控制器电压, mV
// 解码 BMS 故障与电压状态帧。
// 输入:
//   f : State_22 CAN 帧
// 输出:
//   out : 循环次数、故障码和电压；失败时保持旧值
// 返回:
//   true = ID 和 DLC 合法
bool CanProtocol::decodeState22(const CanFrame& f, ChassisState22& out) {
  if (f.id != can_id::CHASSIS_STATE_22 || f.dlc < 8)
    return false;
  out.cycle_count      = readU16LE(&f.data[0]);   // bit0
  out.error_code       = readU16LE(&f.data[2]);   // bit16
  out.max_cell_voltage = readU16LE(&f.data[4]);   // bit32
  out.system_voltage   = readU16LE(&f.data[6]);   // bit48
  return true;
}

// 第二版修协议已取消 State_23 (0x18231CD0), decodeState23 一并删除。
// 原字段去向:
//   max_cell_voltage → State_22 的 BMS_MaxVoltage
//   system_voltage   → State_22 的 System_Voltage
//   min_cell_voltage / max_cell_pos / min_cell_pos → 无对应信号, 已丢失

}   // namespace protocol
}   // namespace weeder_base
