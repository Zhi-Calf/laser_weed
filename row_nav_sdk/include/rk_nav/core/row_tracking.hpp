#pragma once
#include <cstdint>
#include <limits>
#include <vector>

namespace rk_nav {
enum class RowTrackingState { kDisabled, kAcquiring, kTracking, kCoasting, kLost, kRejected };
inline const char* rowTrackingStateName(RowTrackingState state) {
    switch (state) {
        case RowTrackingState::kDisabled: return "disabled";
        case RowTrackingState::kAcquiring: return "acquiring";
        case RowTrackingState::kTracking: return "tracking";
        case RowTrackingState::kCoasting: return "coasting";
        case RowTrackingState::kLost: return "lost";
        case RowTrackingState::kRejected: return "rejected";
    }
    return "unknown";
}
struct RowTrackSnapshot {
    std::uint64_t id{0};
    double lateral_m{0.0}, heading_rad{0.0};
    std::uint32_t missed_frames{0};
    bool confirmed{false}, observed{false};
};
struct RowTrackingDiagnostics {
    RowTrackingState state{RowTrackingState::kDisabled};
    bool raw_valid{false}, prediction_available{false}, reset_after_gap{false};
    double raw_lateral_m{std::numeric_limits<double>::quiet_NaN()};
    double raw_heading_rad{std::numeric_limits<double>::quiet_NaN()};
    double predicted_lateral_m{std::numeric_limits<double>::quiet_NaN()};
    double predicted_heading_rad{std::numeric_limits<double>::quiet_NaN()};
    std::uint64_t left_id{0}, right_id{0};
    std::vector<RowTrackSnapshot> tracks;
};
} // namespace rk_nav
