#pragma once

#include <cstdint>

namespace laser_weed {

// ============================================================
// CAN ID 定义 (29位扩展帧)
// 依据: 激光除草机无人驾驶通讯协议--第二版.xlsx
// ============================================================
namespace can_id {

// VCU -> ADCU
constexpr uint32_t CHASSIS_STATE_01 = 0x18011CD0;  // 20ms  底盘主状态
constexpr uint32_t CHASSIS_STATE_02 = 0x18021CD0;  // 100ms 四轮电机转速
constexpr uint32_t CHASSIS_STATE_03 = 0x18031CD0;  // 100ms 四轮转向角
constexpr uint32_t CHASSIS_STATE_04 = 0x18041CD0;  // 500ms 行走电机 FL
constexpr uint32_t CHASSIS_STATE_05 = 0x18051CD0;  // 500ms 行走电机 FR
constexpr uint32_t CHASSIS_STATE_06 = 0x18061CD0;  // 500ms 行走电机 RL
constexpr uint32_t CHASSIS_STATE_07 = 0x18071CD0;  // 500ms 行走电机 RR
constexpr uint32_t CHASSIS_STATE_08 = 0x18081CD0;  // 500ms 转向电机 FL
constexpr uint32_t CHASSIS_STATE_09 = 0x18091CD0;  // 500ms 转向电机 FR
constexpr uint32_t CHASSIS_STATE_10 = 0x180A1CD0;  // 500ms 转向电机 RL
constexpr uint32_t CHASSIS_STATE_11 = 0x180B1CD0;  // 500ms 转向电机 RR
constexpr uint32_t CHASSIS_STATE_21 = 0x18211CD0;  // 500ms BMS1
constexpr uint32_t CHASSIS_STATE_22 = 0x18221CD0;  // 500ms BMS2
// 第二版协议已取消 State_23 (0x18231CD0):
//   max_cell_voltage → State_22 BMS_MaxVoltage
//   system_voltage   → State_22 System_Voltage
//   min_cell_voltage / max_cell_pos / min_cell_pos 无对应信号, 直接丢失

// ADCU -> VCU
constexpr uint32_t ADCU_TO_VCU_01 = 0x0C01D01C;    // 20ms

}  // namespace can_id

// ============================================================
// 原始 CAN 帧
// ============================================================
struct CanFrame {
  uint32_t id = 0;
  uint8_t  dlc = 0;
  bool     is_extended = true;
  uint8_t  data[8] = {0};
};

// ============================================================
// 接收结构体 (VCU -> ADCU)
// ============================================================

// State_01: 20ms - 底盘主状态
struct ChassisState01 {
  // Byte0 位域
  uint8_t  gear = 0;           // bit0~1  0=N,1=D,2=R,3=无效
  uint8_t  steer_mode = 0;     // bit2~3  0=单阿克曼,1=双阿克曼,2=平移,3=原地
  uint8_t  estop = 0;          // bit4    0=无动作,1=远程急停
  uint8_t  reset = 0;          // bit5    0=无动作,1=故障复位
  uint8_t  release_brake = 0;  // bit6    0=无动作,1=强制解除抱闸
  uint8_t  ctrl_mode = 0;      // bit7    0=遥控,1=无人驾驶
  // Byte1~2: 车速 uint16, mm/s
  uint16_t speed_raw = 0;      // 范围 0~1400(mm/s)
  float    speed_m_s = 0.0f;   // 换算后 m/s
  // Byte3: 轮距 uint8, cm
  uint8_t  wheel_track_cm = 0; // 范围 120~160
  // Byte4 位域: 灯光 (bit32~38)
  uint8_t  sound = 0;          // 蜂鸣器
  uint8_t  light_green = 0;
  uint8_t  light_yellow = 0;
  uint8_t  light_red = 0;
  uint8_t  headlight = 0;
  uint8_t  turn_left = 0;
  uint8_t  turn_right = 0;
  // Byte4~5 位域: 转向到位 (bit39~42)
  uint8_t  steer_done_fl = 0;
  uint8_t  steer_done_fr = 0;
  uint8_t  steer_done_rl = 0;
  uint8_t  steer_done_rr = 0;
  // Byte5 位域: 其他状态 (bit43~46)
  uint8_t  bumper = 0;         // 防撞条
  uint8_t  bypass = 0;         // 旁通开关
  uint8_t  battery_charging = 0;
  uint8_t  battery_full = 0;
  // Byte7: 生命信号
  uint8_t  alive_counter = 0;
};

// State_02: 100ms - 四轮电机转速
struct ChassisState02 {
  uint16_t motor_speed_fl = 0;  // r/min, 0~3000
  uint16_t motor_speed_fr = 0;
  uint16_t motor_speed_rl = 0;
  uint16_t motor_speed_rr = 0;
};

// State_03: 100ms - 四轮转向角
struct ChassisState03 {
  int16_t steer_angle_fl = 0;  // 0.1°, 左正右负, ±90.0°
  int16_t steer_angle_fr = 0;
  int16_t steer_angle_rl = 0;
  int16_t steer_angle_rr = 0;
};

// State_04~07: 500ms - 四轮行走电机电参 (结构相同)
struct MotorElecState {
  uint8_t  bus_voltage = 0;     // V
  uint16_t out_current = 0;     // A
  uint8_t  motor_temp = 0;      // ℃
  uint16_t error_code = 0;      // 见故障码
};

// State_08~11: 500ms - 四轮转向电机电参 (结构同 MotorElecState)
// 复用 MotorElecState

// ------------------------------------------------------------
// State_21: 500ms - BMS 基本状态 (第二版, 全部信号重排)
//
// ⚠️ 以下按第二版文档字面实现, 3 处疑点待 VCU 侧确认:
//   (1) BMS_SOC 分辨率写 1 (Value=X), 第一版为 0.1;
//       16bit 存整数百分比不合理, 疑为表格漏改分辨率
//   (2) BMS_Current 为 unsigned 且偏移 0, 无法表示充电时的负电流
//       (第一版为 Value=X*0.1-3200, 偏移正是为负电流设计)
//   (3) BMS_FullChargeCapacity 单位写 A, 按惯例应为 Ah
//
// 第二版已删除: power_on / heating_status / charge_status / soh
// ------------------------------------------------------------
struct ChassisState21 {
  uint16_t soc_raw = 0;                    // bit0,  %, Value=X
  float    soc = 0.0f;
  uint16_t voltage_raw = 0;                // bit16, V, Value=X*0.01
  float    voltage = 0.0f;
  uint16_t current_raw = 0;                // bit32, A, Value=X*0.01
  float    current = 0.0f;
  uint16_t full_charge_capacity_raw = 0;   // bit48, A(疑为Ah), Value=X*0.01
  float    full_charge_capacity = 0.0f;
};

// ------------------------------------------------------------
// State_22: 500ms - BMS 故障与电压 (第二版, 全部信号重排)
//
// ⚠️ BMS_StateOfCharge 信号名与描述"放电循环次数"不符,
//    命名疑为复制未改, 此处按描述语义命名为 cycle_count
//
// 第二版已删除: error_level / fault_count / 最高最低温度及温度位置(-40 偏移)
// ------------------------------------------------------------
struct ChassisState22 {
  uint16_t cycle_count = 0;        // bit0,  次, 放电循环次数
  uint16_t error_code = 0;         // bit16, 原 8bit 扩为 16bit
  uint16_t max_cell_voltage = 0;   // bit32, mV, 电芯最高电压
  uint16_t system_voltage = 0;     // bit48, mV, VCU 控制器电压
};

// 第二版已取消 ChassisState23 (原 BMS 电芯电压报文)

// ============================================================
// 发送结构体 (ADCU -> VCU)
// ============================================================

// ADCU_toVCU_01: 20ms - 控制指令
// ⚠️ 注意: 本帧 Byte0 的 bit4~6 顺序与接收帧 State_01 相反
//    发送帧: bit4=ReleaseBrake  bit5=EStop  bit6=Reset
//    接收帧: bit4=EStop         bit5=Reset  bit6=ReleaseBrake
struct AdcuCommand01 {
  // Byte0 位域
  uint8_t  gear = 1;          // bit0~1  0=N,1=D,2=R
  uint8_t  steer_mode = 0;    // bit2~3  0=单阿克曼
  uint8_t  release_brake = 1; // bit4    1=释放抱闸
  uint8_t  estop = 0;         // bit5    0=无动作
  uint8_t  reset = 0;         // bit6    0=无动作
  uint8_t  ctrl_mode = 1;     // bit7    1=无人驾驶
  // Byte1~2: 目标车速 uint16, mm/s (范围 0~1400)
  uint16_t speed_raw = 0;
  // Byte3~4: 目标转向角 int16, 0.1°, 左正右负, ±90.0°
  int16_t  steer_angle = 0;
  // Byte5 位域: 灯光 (bit40~46)
  uint8_t  sound = 0;
  uint8_t  light_green = 0;
  uint8_t  light_yellow = 0;
  uint8_t  light_red = 0;
  uint8_t  headlight = 0;
  uint8_t  turn_left = 0;
  uint8_t  turn_right = 0;
  // Byte7: 生命信号
  uint8_t  alive_counter = 0;
};

// ============================================================
// 协议编解码接口
// ============================================================
class CanProtocol {
public:
  // 字节工具 (Intel 小端)
  static uint16_t readU16LE(const uint8_t* p);
  static int16_t  readI16LE(const uint8_t* p);
  static void     writeU16LE(uint8_t* p, uint16_t v);
  static void     writeI16LE(uint8_t* p, int16_t v);

  // 通用位提取 (按起始bit + 长度, Intel 格式)
  // start_bit: 从 0 开始; len: 位数 (1~32)
  static uint32_t extractUnsigned(const uint8_t* data, uint8_t start_bit, uint8_t len);
  static int32_t  extractSigned(const uint8_t* data, uint8_t start_bit, uint8_t len);

  // 编码 ADCU -> VCU
  static CanFrame encodeAdcuCmd01(const AdcuCommand01& cmd);

  // 解码 VCU -> ADCU
  static bool decodeState01(const CanFrame& f, ChassisState01& out);
  static bool decodeState02(const CanFrame& f, ChassisState02& out);
  static bool decodeState03(const CanFrame& f, ChassisState03& out);

  // State_04~11 电参 (行走/转向电机, 结构相同)
  static bool decodeMotorElec(const CanFrame& f, MotorElecState& out);

  // State_21 / 22 BMS (第二版)
  // 第二版已取消 decodeState23
  static bool decodeState21(const CanFrame& f, ChassisState21& out);
  static bool decodeState22(const CanFrame& f, ChassisState22& out);
};

}  // namespace laser_weed
