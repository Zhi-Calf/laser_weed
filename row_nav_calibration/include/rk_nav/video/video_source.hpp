#pragma once
#include "rk_nav/core/frame.hpp"
#include "rk_nav/core/status.hpp"

namespace rk_nav {
class VideoSource {
public:
    virtual ~VideoSource() = default;
    virtual Status open() = 0;
    virtual Result<Frame> read() = 0;
    virtual void close() = 0;
};
}  // namespace rk_nav
