#ifndef SIM_VEHICLE_KINEMATICS_H_
#define SIM_VEHICLE_KINEMATICS_H_

#include <array>
#include <cstddef>

namespace sim_vehicle {

inline constexpr double kWheelbaseM = 1.521;
inline constexpr double kWheelTrackM = 1.306;
inline constexpr double kWheelRadiusM = 0.29;
inline constexpr double kVehicleLengthM = 1.8;   // Isaac 场景车体长度
inline constexpr double kVehicleWidthM = 1.2;    // Isaac 场景车体宽度
inline constexpr double kVehicleHeightM = 0.5;   // Isaac 场景车体高度
inline constexpr double kMaxSpeedMps = 1.4;
inline constexpr double kMaxSteerRad = 3.14159265358979323846 / 2.0;
inline constexpr std::size_t kWheelCount = 4;

enum class WheelIndex : std::size_t {
  kFrontLeft = 0,
  kFrontRight = 1,
  kRearLeft = 2,
  kRearRight = 3,
};

struct WheelPosition {
  double x_m{0.0};  // 车体坐标系前向位置
  double y_m{0.0};  // 车体坐标系左向位置
};

struct FourWheelState {
  std::array<double, kWheelCount> steer_rad{};    // 四轮转向角
  std::array<double, kWheelCount> wheel_rad_s{};  // 四轮角速度
  double yaw_rate_rad_s{0.0};                     // 车辆横摆角速度
  double effective_wheelbase_m{0.0};              // 等效轴距
};

double WrapAngle(double angle);

std::array<WheelPosition, kWheelCount> ComputeWheelPositions(
  int steer_mode, double wheelbase_m = kWheelbaseM,
  double wheel_track_m = kWheelTrackM);

FourWheelState ComputeFourWheelState(double speed_m_s, double steer_angle_rad,
                                     int steer_mode,
                                     double wheelbase_m = kWheelbaseM,
                                     double wheel_track_m = kWheelTrackM,
                                     double wheel_radius_m = kWheelRadiusM,
                                     double max_steer_rad = kMaxSteerRad);

}  // namespace sim_vehicle

#endif  // SIM_VEHICLE_KINEMATICS_H_
