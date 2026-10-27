#ifndef SIM_VEHICLE_KINEMATICS_H_
#define SIM_VEHICLE_KINEMATICS_H_

#include <array>
#include <cstddef>

namespace sim_vehicle {

// ============================================================
// 四轮转向运动学模型。
// 车体坐标系: x 向前、y 向左, 角度单位 rad。
// steer_mode 0 = 后轴参考的单阿克曼, 1 = 车体中心参考的对称阿克曼。
// ============================================================
inline constexpr double kWheelbaseM      = 1.521;                          // 前后轴距 (m)
inline constexpr double kWheelTrackM     = 1.306;                          // 左右轮中心距 (m)
inline constexpr double kWheelRadiusM    = 0.29;                           // 车轮半径 (m)
inline constexpr double kVehicleLengthM  = 1.8;                            // Isaac 场景车体长度
inline constexpr double kVehicleWidthM   = 1.2;                            // Isaac 场景车体宽度
inline constexpr double kVehicleHeightM  = 0.5;                            // Isaac 场景车体高度
inline constexpr double kMaxSpeedMps     = 1.4;                            // 最大线速度 (m/s)
inline constexpr double kMaxSteerRad     = 3.14159265358979323846 / 2.0;   // 单轮最大转角 (rad)
inline constexpr std::size_t kWheelCount = 4;                              // 当前运动学固定支持四轮

enum class WheelIndex : std::size_t {
  kFrontLeft  = 0,
  kFrontRight = 1,
  kRearLeft   = 2,
  kRearRight  = 3,
};

struct WheelPosition {
  double x_m{0.0};   // 车体坐标系前向位置
  double y_m{0.0};   // 车体坐标系左向位置
};

struct FourWheelState {
  std::array<double, kWheelCount> steer_rad{};     // 四轮转向角
  std::array<double, kWheelCount> wheel_rad_s{};   // 四轮角速度
  double yaw_rate_rad_s{0.0};                      // 车辆横摆角速度
  double effective_wheelbase_m{0.0};               // 等效轴距
};

// 将角度归一化到 [-pi, pi)。
double WrapAngle(double angle);

// 按转向模式返回四轮位置。
// steer_mode 0 以指定后轴为参考; 其它取值以车体几何中心为参考。
// 非正几何尺寸会被下限保护, 返回值顺序由 WheelIndex 定义。
std::array<WheelPosition, kWheelCount> ComputeWheelPositions(int steer_mode, double wheelbase_m = kWheelbaseM, double wheel_track_m = kWheelTrackM);

// 返回以车体几何中心为原点的四轮位置; 非正尺寸同样应用下限保护。
std::array<WheelPosition, kWheelCount> ComputeCenteredWheelPositions(double wheelbase_m = kWheelbaseM, double wheel_track_m = kWheelTrackM);

// 根据统一速度与等效转角计算四轮转角、轮速和横摆角速度。
// 非有限速度/转角按 0 处理; 不支持的 steer_mode 返回全零状态。
// 输入速度会被限制为 >= 0, 不表示倒车。
FourWheelState ComputeFourWheelState(
  double speed_m_s,
  double steer_angle_rad,
  int steer_mode,
  double wheelbase_m    = kWheelbaseM,
  double wheel_track_m  = kWheelTrackM,
  double wheel_radius_m = kWheelRadiusM,
  double max_steer_rad  = kMaxSteerRad);

}   // namespace sim_vehicle

#endif   // SIM_VEHICLE_KINEMATICS_H_
