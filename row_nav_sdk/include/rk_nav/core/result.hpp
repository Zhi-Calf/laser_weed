#pragma once
#include <cstdint>
#include "rk_nav/core/row_tracking.hpp"

namespace rk_nav {
enum class TrackingStatus { kValid = 0, kLowConfidence, kNoRow, kInputError, kInferenceError, kStaleFrame };

struct NavigationObservation {
    std::uint64_t sequence{0};
    std::int64_t capture_time_ns{0};
    double lateral_error_m{0.0};
    double heading_error_rad{0.0};
    double lookahead_m{0.0};
    float confidence{0.0F};
    double latency_ms{0.0};
    TrackingStatus status{TrackingStatus::kInputError};
    RowTrackingDiagnostics tracking;
};
}  // namespace rk_nav
