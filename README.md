laser_weed_can - 激光除草机 CAN 协议层

基于 tractor_base (大履带/拖拉机) 项目的架构，迁移为激光除草机无人驾驶通讯协议（第二版）。

协议依据：激光除草机无人驾驶通讯协议--第二版.xlsx（500kbps / Intel 字节序 / 29 位扩展帧）

架构

协议文档 (xlsx)
    ↓
can_protocol.hpp   ← 报文结构体 + CAN ID 定义
can_protocol.cpp   ← 字节 ↔ 结构体 编解码 (Intel 小端)
    ↓
can_driver         ← 收发原始 CanFrame (复用 tractor_base)
    ↓
ROS2 Node          ← 业务层: 发布话题 / 定时发送指令

完全复用 tractor_base 的分层思想：协议层与 ROS 解耦，纯 C++ 可单独测试。

目录结构

laser_weed_can/
├── CMakeLists.txt
├── README.md
├── example_node.cpp     # ROS2 接入示例
├── protocol/
│   ├── can_protocol.hpp   # 所有报文结构体 + ID + 编解码接口
│   └── can_protocol.cpp   # 实现 (Intel 小端, 通用位提取)
└── test/
    └── test_protocol.cpp  # 单元测试

已实现的报文

接收 (VCU → ADCU)
报文   ID   周期   内容
State_01   0x18011CD0   20ms   底盘主状态: 档位/转向模式/急停/复位/抱闸/驾驶模式/车速/轮距/灯光/转向到位/防撞条/旁通

State_02   0x18021CD0   100ms   四轮电机转速 (r/min, 0~3000)

State_03   0x18031CD0   100ms   四轮转向角 (0.1°, 左正右负, ±90°)

State_04~07   0x1804~07 1CD0   500ms   四轮行走电机电参 (复用 MotorElecState)

State_08~11   0x1808~0B 1CD0   500ms   四轮转向电机电参 (复用 MotorElecState)

State_21   0x18211CD0   500ms   BMS: SOC / 电压(×0.01V) / 电流(×0.01A) / 充满容量(×0.01)

State_22   0x18221CD0   500ms   BMS: 放电循环次数 / 故障码(16bit) / 电芯最高电压(mV) / VCU控制器电压(mV)

第二版已取消 State_23 (0x18231CD0)。原字段去向：max_cell_voltage → State_22 BMS_MaxVoltage；
system_voltage → State_22 System_Voltage；min_cell_voltage / max_cell_pos / min_cell_pos
无对应信号，已丢失。

发送 (ADCU → VCU)
报文   ID   周期   内容
ADCU_toVCU_01   0x0C01D01C   20ms   控制指令: 档位/转向模式/抱闸/急停/复位/驾驶模式/目标车速/目标转向角/灯光/生命信号

第二版相对第一版的变化
项目   第一版   第二版
State_01~11   —   无变化

ADCU_toVCU_01   —   无变化

State_21   power_on / heating / charge / soh / soc(×0.1) / voltage(×0.1) / current(×0.1−3200)   SOC / Voltage(×0.01) / Current(×0.01) / FullChargeCapacity(×0.01)，全部重排

State_22   error_level / error_code(8bit) / fault_count / 最高最低温度(−40) 及位置   cycle_count / error_code(16bit) / max_cell_voltage / system_voltage，全部重排

State_23   电芯电压报文   整条取消

核心设计: 通用位提取

decodeState01 严格按协议文档的起始位置(bit) + 信号长度逐行对应，用
extractUnsigned/extractSigned 自动处理字节边界，避免手写位移错位:

out.gear       = extractUnsigned(f.data, 0,  2);   // bit0~1
out.estop      = extractUnsigned(f.data, 4,  1);   // bit4
out.speed_raw  = extractUnsigned(f.data, 8, 16);   // bit8~23 (Byte1~2)
out.light_green= extractUnsigned(f.data, 33, 1);   // bit33
out.bumper     = extractUnsigned(f.data, 43, 1);   // bit43
out.alive      = extractUnsigned(f.data, 56, 8);   // bit56~63 (Byte7)

⚠️ 收发帧 Byte0 的 bit4~6 顺序相反

协议本身的设定，不是笔误，写代码时极易搞混：
bit4   bit5   bit6   bit7
接收 State_01   EStopStatus   ResetStatus   ReleaseBrakeStatus   CtrlModeStatus

发送 ADCU_toVCU_01   ReleaseBrakeCmd   EStopCmd   ResetCmd   CtrlModeCmd

decodeState01 与 encodeAdcuCmd01 已分别按各自顺序实现，不要互相复用位偏移。

编译 & 测试 (纯 C++ 无需 ROS)

g++ -std=c++17 -I. test/test_protocol.cpp protocol/can_protocol.cpp -o test_protocol
./test_protocol

测试覆盖: 编码字节布局 / 解码位域 / 有符号数 / 分辨率换算 / 电机电参 / BMS 第二版布局 /
边界检查(ID 串号、dlc 不足) / 往返一致性。

注: CMakeLists.txt 目前只构建协议库，未把 test_protocol.cpp 纳入 ament 构建，
测试按上面的 g++ 命令单独编译。

⚠️ 待硬件确认项

代码已按第二版文档字面实现，以下几处文档疑点会影响换算结果，确认后按注释改对应行即可。

车速分辨率: 文档写 0.001, 但字段仅 16bit (最大 65.5 mm/s), 与范围 0~1400 mm/s 矛盾。
   推断实际分辨率 = 1 mm/s (raw 0~1400)。第一版、第二版均未修正此矛盾，
   当前代码按 1 mm/s 处理 (speed_m_s = speed_raw * 0.001f)。
BMS_SOC 分辨率: 第二版写 1 (第一版为 0.1)。用 16bit 存整数百分比不合理，疑为表格漏改。
   若确认为 0.1，改 decodeState21 中 out.soc = out.soc_raw * 0.1f。
BMS_Current 符号: 第二版改为 unsigned + 偏移 0，无法表示充电时的负电流。
   第一版的 −3200 偏移正是为此设计。若确认保留偏移，改 decodeState21 中
   out.current = out.current_raw * 0.01f - 3200.0f（需同步调整精度）。
BMS_FullChargeCapacity 单位: 文档写 A, 按惯例应为 Ah。不影响换算，仅影响语义与话题命名。
BMS_StateOfCharge 命名: 信号名与描述"放电循环次数"不符，疑为复制未改。
   代码按描述语义命名为 cycle_count，若实际含义是 SOC 需重命名。
State_23 取消后的信息缺口: min_cell_voltage（电芯最低电压）在第二版无任何报文承载，
   而压差是判断电芯均衡的关键指标；error_level（故障等级）与温度类信号同样消失。
   若业务需要，需与 VCU 侧确认是否另有报文或恢复该帧。
State_04~11: 布局与第一版一致，建议实机抓包验证字节位置。
State_03 备注笔误: 四轮上限分别写作 90.0°/90.1°/90.2°/90.3°，应为一致的 ±90.0°。

迁移自 tractor_base 的主要改动
项目   tractor_base (原)   laser_weed (现)
速度单位   mm/s   mm/s (文档标 0.001, 见待确认项 1)

转向单位   0.001 rad/s   0.1°

信号集   PTO/悬挂/液压   灯光/转向到位/防撞条/BMS(第二版精简)

报文 ID   0x0C0xD01C 等   0x180x1CD0 / 0x0C01D01C
