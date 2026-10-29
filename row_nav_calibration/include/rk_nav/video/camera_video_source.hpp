#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "rk_nav/video/video_source.hpp"

namespace rk_nav {

struct CameraOptions {
    std::string device{"/dev/video12"};
    int width{1920};
    int height{1080};
    int fps{30};
    int rotation_degrees{0};
    int io_mode{4};
};

class CameraVideoSource final : public VideoSource {
public:
    explicit CameraVideoSource(CameraOptions options);
    ~CameraVideoSource() override;
    CameraVideoSource(const CameraVideoSource&) = delete;
    CameraVideoSource& operator=(const CameraVideoSource&) = delete;

    Status open() override;
    Result<Frame> read() override;
    void close() override;
    [[nodiscard]] double fps() const;
    [[nodiscard]] std::uint64_t framesRead() const;
    [[nodiscard]] const std::string& pipeline() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rk_nav
