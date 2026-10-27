#include <cstdio>

#include "weeder_base/protocol/can_protocol.hpp"

using namespace weeder_base::protocol;

int main() {
  bool ok    = true;
  auto check = [&](bool cond, const char* name) {
    if (!cond) {
      ok = false;
      printf("  [FAIL] %s\n", name);
    } else {
      printf("  [PASS] %s\n", name);
    }
  };

  // ============================================================
  // 1. encodeAdcuCmd01: 构造一条控制指令, 验证字节布局
  // ============================================================
  printf("\n[encodeAdcuCmd01]\n");
  AdcuCommand01 cmd{};
  cmd.gear          = 1;   // D档
  cmd.steer_mode    = 0;   // 单阿克曼
  cmd.release_brake = 1;   // 释放抱闸
  cmd.estop         = 0;
  cmd.reset         = 0;
  cmd.ctrl_mode     = 1;     // 无人驾驶
  cmd.speed_raw     = 500;   // 500 mm/s = 0.5 m/s (合法范围 0~1400)
  cmd.steer_angle   = 100;   // 10.0° 左转 (0.1° 分辨率)
  cmd.alive_counter = 123;

  CanFrame f = CanProtocol::encodeAdcuCmd01(cmd);
  printf("  id=0x%08X dlc=%u ext=%d\n", f.id, f.dlc, f.is_extended);
  printf("  data: ");
  for (int i = 0; i < 8; i++)
    printf("%02X ", f.data[i]);
  printf("\n");

  check(f.id == can_id::ADCU_TO_VCU_01, "ID 匹配");
  check(f.dlc == 8, "DLC=8");
  check(f.is_extended, "扩展帧");
  // Byte0 按第二版修 ADCU_toVCU_01 布局:
  //   bit0~1=GearCmd=1(0x01)  bit2~3=SteerModeCmd=0
  //   bit4=ReleaseBrakeCmd=1(0x10)  bit5=EStopCmd=0
  //   bit6=ResetCmd=0  bit7=CtrlModeCmd=1(0x80)
  //   → 0x01 | 0x10 | 0x80 = 0x91
  check(f.data[0] == 0x91, "Byte0 位域 (0x91)");
  // Byte1~2: speed_raw=500=0x01F4 → 小端 0xF4 0x01
  check(f.data[1] == 0xF4 && f.data[2] == 0x01, "Byte1~2 车速小端 (500)");
  // Byte3~4: steer_angle=100=0x0064 → 小端 0x64 0x00
  check(f.data[3] == 0x64 && f.data[4] == 0x00, "Byte3~4 转向角小端 (100)");
  check(f.data[5] == 0x00, "Byte5 灯光全关");
  check(f.data[7] == 123, "Byte7 生命信号");
  check(timing::kAdcuCmdPeriodMs == 20, "ADCU 下发周期=20ms");

  // 协议层兜底限幅: 调用方传入越界值时, 不能把非法值原样写到总线上。
  AdcuCommand01 cmd_high{};
  cmd_high.speed_raw   = 60000;
  cmd_high.steer_angle = 2000;
  CanFrame f_high      = CanProtocol::encodeAdcuCmd01(cmd_high);
  check(CanProtocol::readU16LE(&f_high.data[1]) == limits::kMaxSpeedMmps, "车速越界被限幅到 1400");
  check(CanProtocol::readI16LE(&f_high.data[3]) == limits::kMaxSteerAngleRaw, "转向角越界被限幅到 900");

  // ============================================================
  // 1b. 位提取范围检查: 只允许访问 8 字节帧内的 1~32 bit
  // ============================================================
  printf("\n[位提取范围检查]\n");
  uint8_t bits[8] = {0};
  bits[7]         = 0xA5;
  check(CanProtocol::extractUnsigned(bits, 56, 8) == 0xA5, "末字节有效范围");
  check(CanProtocol::extractUnsigned(bits, 63, 1) == 1, "bit63 有效范围");
  check(CanProtocol::extractUnsigned(nullptr, 0, 8) == 0, "空指针返回 0");
  check(CanProtocol::extractUnsigned(bits, 64, 1) == 0, "起始位 64 返回 0");
  check(CanProtocol::extractUnsigned(bits, 60, 8) == 0, "跨越 64 bit 返回 0");
  check(CanProtocol::extractUnsigned(bits, 0, 0) == 0, "长度 0 返回 0");
  check(CanProtocol::extractUnsigned(bits, 0, 33) == 0, "长度 33 返回 0");
  check(CanProtocol::extractSigned(bits, 60, 8) == 0, "有符号越界返回 0");
  check(CanProtocol::extractSigned(bits, 0, 33) == 0, "有符号长度越界返回 0");

  // ============================================================
  // 2. decodeState01: 模拟一帧 VCU 状态, 验证解析
  // ============================================================
  printf("\n[decodeState01]\n");
  CanFrame rx{};
  rx.id          = can_id::CHASSIS_STATE_01;
  rx.dlc         = 8;
  rx.is_extended = true;
  // Byte0 按接收帧 State_01 布局 (与发送帧 bit4~6 顺序相反):
  //   bit4=EStop=0  bit5=Reset=0  bit6=ReleaseBrake=1(0x40)  bit7=CtrlMode=1(0x80)
  //   bit0~1=Gear=1(0x01) → 0x01 | 0x40 | 0x80 = 0xC1
  rx.data[0] = 0xC1;
  rx.data[1] = 0x64;   // 车速低字节
  rx.data[2] = 0x01;   // 车速高字节 → 0x0164 = 356
  rx.data[3] = 130;    // 轮距 130cm
  // Byte4: 0b1001'0010 = 0x92
  //   bit33=Green=1, bit36=Headlight=1, bit39(=bit7)=SteerDone_FL=1
  rx.data[4] = 0b1001'0010;
  // Byte5: 0b0000'1000 → bit43(=bit3)=Bumper=1
  rx.data[5] = 0b0000'1000;
  rx.data[6] = 0x00;
  rx.data[7] = 0xAB;   // alive

  ChassisState01 s1{};
  check(CanProtocol::decodeState01(rx, s1), "返回 true");
  check(s1.gear == 1, "Gear=1");
  check(s1.steer_mode == 0, "SteerMode=0(单阿克曼)");
  check(s1.estop == 0, "EStop=0(bit4)");
  check(s1.reset == 0, "Reset=0(bit5)");
  check(s1.release_brake == 1, "ReleaseBrake=1(bit6)");
  check(s1.ctrl_mode == 1, "CtrlMode=1(bit7, 无人)");
  check(s1.speed_raw == 0x0164, "speed_raw=356");
  // 当前实现按分辨率 1 mm/s 处理 (文档写 0.001 与 0~1400 范围矛盾, 见 README):
  //   356 mm/s → 0.356 m/s
  check(s1.speed_m_s > 0.3559f && s1.speed_m_s < 0.3561f, "speed_m_s=0.356");
  check(s1.wheel_track_cm == 130, "轮距=130cm");
  check(s1.sound == 0, "蜂鸣器=0(bit32)");
  check(s1.light_green == 1 && s1.headlight == 1, "灯光位域(bit33/bit36)");
  check(s1.steer_done_fl == 1, "转向到位FL=1(bit39)");
  check(s1.steer_done_fr == 0, "转向到位FR=0(bit40)");
  check(s1.bumper == 1, "防撞条=1(bit43)");
  check(s1.bypass == 0, "旁通=0(bit44)");
  check(s1.alive_counter == 0xAB, "alive=0xAB");

  // ============================================================
  // 3. decodeState02 / 03: 电机转速 + 转向角 (含负数)
  // ============================================================
  printf("\n[decodeState02/03]\n");
  CanFrame rx2{};
  rx2.id          = can_id::CHASSIS_STATE_02;
  rx2.dlc         = 8;
  rx2.is_extended = true;
  rx2.data[0]     = 0xD0;
  rx2.data[1]     = 0x07;   // FL=0x07D0=2000 r/min
  rx2.data[2]     = 0x00;
  rx2.data[3]     = 0x00;
  rx2.data[4]     = 0x00;
  rx2.data[5]     = 0x00;
  rx2.data[6]     = 0x00;
  rx2.data[7]     = 0x00;

  ChassisState02 s2{};
  check(CanProtocol::decodeState02(rx2, s2), "State02 返回 true");
  check(s2.motor_speed_fl == 2000, "FL转速=2000");
  check(s2.motor_speed_fr == 0 && s2.motor_speed_rr == 0, "其余轮转速=0");

  // 转向角: 0x01F4=500 → 50.0°, 0xFF38=-200 → -20.0°
  CanFrame rx3{};
  rx3.id          = can_id::CHASSIS_STATE_03;
  rx3.dlc         = 8;
  rx3.is_extended = true;
  rx3.data[0]     = 0xF4;
  rx3.data[1]     = 0x01;   // FL=500  → 50.0°
  rx3.data[2]     = 0x38;
  rx3.data[3]     = 0xFF;   // FR=-200 → -20.0°
  rx3.data[4]     = 0x00;
  rx3.data[5]     = 0x00;
  rx3.data[6]     = 0x00;
  rx3.data[7]     = 0x00;
  ChassisState03 s3{};
  check(CanProtocol::decodeState03(rx3, s3), "State03 返回 true");
  check(s3.steer_angle_fl == 500, "FL转向角raw=500");
  check(s3.steer_angle_fr == -200, "FR转向角 signed=-200");

  // ============================================================
  // 4. decodeMotorElec: State_04~11 电参 (结构相同)
  // ============================================================
  printf("\n[decodeMotorElec]\n");
  CanFrame rxe{};
  rxe.id          = can_id::CHASSIS_STATE_04;
  rxe.dlc         = 8;
  rxe.is_extended = true;
  rxe.data[0]     = 48;   // 母线电压 48V
  rxe.data[1]     = 0x64;
  rxe.data[2]     = 0x00;   // 电流 100A
  rxe.data[3]     = 65;     // 温度 65℃
  rxe.data[4]     = 0x02;
  rxe.data[5]     = 0x00;   // 故障码 2 (过压)
  MotorElecState se{};
  check(CanProtocol::decodeMotorElec(rxe, se), "返回 true");
  check(se.bus_voltage == 48, "母线电压=48V");
  check(se.out_current == 100, "电流=100A");
  check(se.motor_temp == 65, "温度=65℃");
  check(se.error_code == 2, "故障码=2(过压)");
  rxe.id = can_id::CHASSIS_STATE_21;   // 21 帧不是电机电参帧
  check(CanProtocol::decodeMotorElec(rxe, se) == false, "非 State_04~11 返回 false");

  // ============================================================
  // 5. decodeState21 / 22: BMS (第二版修布局)
  // ============================================================
  printf("\n[decodeState21/22]\n");
  CanFrame rx21{};
  rx21.id          = can_id::CHASSIS_STATE_21;
  rx21.dlc         = 8;
  rx21.is_extended = true;
  rx21.data[0]     = 0x50;
  rx21.data[1]     = 0x00;   // bit0  SOC   raw=0x0050=80   → 80%
  rx21.data[2]     = 0x10;
  rx21.data[3]     = 0x13;   // bit16 电压  raw=0x1310=4880 → 48.80V
  rx21.data[4]     = 0x04;
  rx21.data[5]     = 0x29;   // bit32 电流  raw=0x2904=10500 → 50.00A
  rx21.data[6]     = 0xE8;
  rx21.data[7]     = 0x03;   // bit48 容量  raw=0x03E8=1000 → 10.00

  ChassisState21 s21{};
  check(CanProtocol::decodeState21(rx21, s21), "State21 返回 true");
  check(s21.soc_raw == 80 && s21.soc > 79.9f && s21.soc < 80.1f, "SOC=80% (分辨率1)");
  check(s21.voltage_raw == 4880 && s21.voltage > 48.79f && s21.voltage < 48.81f, "电压=48.80V (×0.01)");
  check(s21.current_raw == 10500 && s21.current > 49.99f && s21.current < 50.01f, "放电电流=50.00A (×0.1-1000)");
  check(s21.full_charge_capacity_raw == 1000 && s21.full_charge_capacity > 9.99f && s21.full_charge_capacity < 10.01f, "充满容量=10.00 (×0.01)");

  // 充电时电流为负: raw=9000 → 0.1*9000-1000 = -100.0 A
  CanFrame rx21_charge = rx21;
  rx21_charge.data[4]  = 0x28;
  rx21_charge.data[5]  = 0x23;
  ChassisState21 s21_charge{};
  check(CanProtocol::decodeState21(rx21_charge, s21_charge), "State21 充电帧返回 true");
  check(s21_charge.current_raw == 9000 && s21_charge.current > -100.01f && s21_charge.current < -99.99f, "充电电流=-100.00A (signed, 偏移1000)");

  CanFrame rx22{};
  rx22.id          = can_id::CHASSIS_STATE_22;
  rx22.dlc         = 8;
  rx22.is_extended = true;
  rx22.data[0]     = 0x2A;
  rx22.data[1]     = 0x00;   // bit0  放电循环次数=42
  rx22.data[2]     = 0x05;
  rx22.data[3]     = 0x00;   // bit16 故障码=5 (欠压)
  rx22.data[4]     = 0x40;
  rx22.data[5]     = 0x0F;   // bit32 电芯最高=0x0F40=3904 mV
  rx22.data[6]     = 0x00;
  rx22.data[7]     = 0x30;   // bit48 控制器电压=0x3000=12288 mV
  ChassisState22 s22{};
  check(CanProtocol::decodeState22(rx22, s22), "State22 返回 true");
  check(s22.cycle_count == 42, "放电循环次数=42");
  check(s22.error_code == 5, "BMS故障码=5(欠压)");
  check(s22.max_cell_voltage == 3904, "电芯最高电压=3904mV");
  check(s22.system_voltage == 12288, "VCU控制器电压=12288mV");

  // ============================================================
  // 6. 边界: ID 不匹配 / dlc 不足应返回 false
  // ============================================================
  printf("\n[边界检查]\n");
  CanFrame bad{};
  bad.id  = 0x99999999;
  bad.dlc = 8;
  ChassisState01 sb{};
  check(CanProtocol::decodeState01(bad, sb) == false, "ID不匹配返回false");
  bad.id  = can_id::CHASSIS_STATE_01;
  bad.dlc = 4;
  check(CanProtocol::decodeState01(bad, sb) == false, "dlc<8返回false");

  ChassisState21 sb21{};
  bad.id  = can_id::CHASSIS_STATE_22;   // 用 22 的 ID 解 21
  bad.dlc = 8;
  check(CanProtocol::decodeState21(bad, sb21) == false, "State21 ID串号返回false");
  bad.id  = can_id::CHASSIS_STATE_21;
  bad.dlc = 4;
  check(CanProtocol::decodeState21(bad, sb21) == false, "State21 dlc<8返回false");
  ChassisState22 sb22{};
  bad.id = can_id::CHASSIS_STATE_22;
  check(CanProtocol::decodeState22(bad, sb22) == false, "State22 dlc<8返回false");

  // ============================================================
  // 7. 往返一致性: encode(cmd) → frame, 验证位域与数值
  //    发送帧 Byte0: bit4=ReleaseBrake bit5=EStop bit6=Reset bit7=CtrlMode
  //    ⚠️ 与接收帧 State_01 的 bit4~6 顺序相反, 不可互换
  // ============================================================
  printf("\n[往返一致性 encode]\n");
  AdcuCommand01 cmd2{};
  cmd2.gear          = 2;   // R档
  cmd2.steer_mode    = 3;   // 原地转向
  cmd2.release_brake = 1;
  cmd2.estop         = 1;   // 急停
  cmd2.reset         = 0;
  cmd2.ctrl_mode     = 1;
  cmd2.speed_raw     = 1200;   // 1200 mm/s
  cmd2.steer_angle   = -300;   // -30.0°
  cmd2.sound         = 1;
  cmd2.light_red     = 1;
  cmd2.headlight     = 1;
  cmd2.alive_counter = 0x42;

  CanFrame f2 = CanProtocol::encodeAdcuCmd01(cmd2);
  printf("  data: ");
  for (int i = 0; i < 8; i++)
    printf("%02X ", f2.data[i]);
  printf("\n");

  // Byte0 = 0x02 | (3<<2) | (1<<4) | (1<<5) | (0<<6) | (1<<7) = 0xBE
  check(f2.data[0] == 0xBE, "Byte0=0xBE");
  check((f2.data[0] & 0x03) == 2, "Gear=2 (bit0~1)");
  check(((f2.data[0] >> 2) & 0x03) == 3, "SteerMode=3 (bit2~3)");
  check(((f2.data[0] >> 4) & 0x01) == 1, "ReleaseBrake=1 (bit4)");
  check(((f2.data[0] >> 5) & 0x01) == 1, "EStop=1 (bit5)");
  check(((f2.data[0] >> 6) & 0x01) == 0, "Reset=0 (bit6)");
  check(((f2.data[0] >> 7) & 0x01) == 1, "CtrlMode=1 (bit7)");
  check(CanProtocol::readU16LE(&f2.data[1]) == 1200, "speed=1200 (bit8)");
  check(CanProtocol::readI16LE(&f2.data[3]) == -300, "steer_angle=-300 (bit24)");
  // Byte5 = Sound(bit40) | Red(bit43) | Headlight(bit44) = 0x01|0x08|0x10 = 0x19
  check(f2.data[5] == 0x19, "Byte5 灯光=0x19");
  check(((f2.data[5] >> 0) & 0x01) == 1, "Sound=1 (bit40)");
  check(((f2.data[5] >> 1) & 0x01) == 0, "Green=0 (bit41)");
  check(((f2.data[5] >> 3) & 0x01) == 1, "Red=1 (bit43)");
  check(((f2.data[5] >> 4) & 0x01) == 1, "Headlight=1 (bit44)");
  check(((f2.data[5] >> 5) & 0x01) == 0, "TurnLeft=0 (bit45)");
  check(((f2.data[5] >> 6) & 0x01) == 0, "TurnRight=0 (bit46)");
  check(f2.data[6] == 0x00, "Byte6 保留=0");
  check(f2.data[7] == 0x42, "alive=0x42 (bit56)");

  printf("\n========================================\n");
  if (ok)
    printf("全部测试通过\n");
  else
    printf("存在失败用例\n");
  return ok ? 0 : 1;
}
