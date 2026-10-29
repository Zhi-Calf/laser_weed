#ifndef SIM_VEHICLE_KINEMATICS_H_
#define SIM_VEHICLE_KINEMATICS_H_

#include <array>
#include <cmath>
#include <cstddef>

namespace sim_vehicle {

// ============================================================
// 四轮转向运动学模型。
// 车体坐标系: x 向前、y 向左, 角度单位 rad。
// steer_mode 0 = 后轴参考的单阿克曼, 1 = 车体中心参考的对称阿克曼。
// ============================================================
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
  std::array<double, 4> steer_rad{};     // 四轮转向角
  std::array<double, 4> wheel_rad_s{};   // 四轮角速度
  double yaw_rate_rad_s{0.0};            // 车辆横摆角速度
  double effective_wheelbase_m{0.0};     // 等效轴距
};

// 将角度归一化到 [-pi, pi]。
double WrapAngle(double angle);

// 按转向模式返回四轮相对参考点的位置。
std::array<WheelPosition, 4> ComputeWheelPositions(int steer_mode, double wheelbase_m = 1.521, double wheel_track_m = 1.306);

// 返回以车体几何中心为原点的四轮位置。
std::array<WheelPosition, 4> ComputeCenteredWheelPositions(double wheelbase_m = 1.521, double wheel_track_m = 1.306);

// 根据统一速度与等效转角计算四轮转角和轮速。
FourWheelState ComputeFourWheelState(
  double speed_m_s,
  double steer_angle_rad,
  int steer_mode,
  double wheelbase_m    = 1.521,
  double wheel_track_m  = 1.306,
  double wheel_radius_m = 0.29,
  double max_steer_rad  = M_PI / 2.0);

}   // namespace sim_vehicle

#endif   // SIM_VEHICLE_KINEMATICS_H_
