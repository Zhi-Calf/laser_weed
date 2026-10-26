#pragma once
#include <cstdint>
#include <opencv2/core/mat.hpp>

namespace rk_nav {
struct Frame {
    cv::Mat image;
    std::int64_t capture_time_ns{0};
    std::uint64_t sequence{0};
};
}  // namespace rk_nav
