#include "rk_nav/video/camera_video_source.hpp"

#include <chrono>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#ifdef RK_NAV_WITH_RGA
#include <im2d.h>
#include <rga.h>
#endif

#ifdef __linux__
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace rk_nav {
namespace {

Status validateOptions(const CameraOptions& options) {
    if (options.device.empty()) return Status::invalidArgument("camera device is empty");
    if (options.width <= 0 || options.height <= 0) {
        return Status::invalidArgument("camera width and height must be greater than zero");
    }
    if (options.fps <= 0) return Status::invalidArgument("camera fps must be greater than zero");
    if (options.io_mode < 0) return Status::invalidArgument("camera io_mode cannot be negative");
    if (options.rotation_degrees != 0 && options.rotation_degrees != 90 &&
        options.rotation_degrees != 180 && options.rotation_degrees != 270) {
        return Status::invalidArgument("camera rotation must be one of 0, 90, 180, 270");
    }
    return Status::ok();
}

std::string buildPipeline(const CameraOptions& options) {
#ifdef __linux__
    std::ostringstream description;
    description << "v4l2-mmap device=" << options.device << " format=NV12,width="
                << options.width << ",height=" << options.height;
    return description.str();
#else
    std::ostringstream pipeline;
    pipeline << "v4l2src device=" << options.device << " io-mode=" << options.io_mode
             << " ! video/x-raw,format=NV12,width=" << options.width
             << ",height=" << options.height << ",framerate=" << options.fps << "/1"
             << " ! videoconvert ! video/x-raw,format=BGR"
             << " ! appsink drop=true max-buffers=1 sync=false";
    return pipeline.str();
#endif
}

void rotateFrame(cv::Mat& image, int degrees) {
    if (degrees == 90) cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);
    else if (degrees == 180) cv::rotate(image, image, cv::ROTATE_180);
    else if (degrees == 270) cv::rotate(image, image, cv::ROTATE_90_COUNTERCLOCKWISE);
}

#ifdef __linux__
int retryIoctl(int descriptor, unsigned long request, void* argument) {
    int result = 0;
    do {
        result = ioctl(descriptor, request, argument);
    } while (result < 0 && errno == EINTR);
    return result;
}

Status systemFailure(const std::string& operation) {
    return Status::unavailable(operation + " failed: " + std::strerror(errno));
}
#endif

}  // namespace

class CameraVideoSource::Impl {
public:
    explicit Impl(CameraOptions camera_options) : options(std::move(camera_options)) {}

    CameraOptions options;
    cv::VideoCapture capture;
    std::string gst_pipeline;
    std::chrono::steady_clock::time_point started_at;
    std::uint64_t sequence{0};
#ifdef __linux__
    struct MappedBuffer {
        void* address{MAP_FAILED};
        std::size_t length{0};
    };
    int descriptor{-1};
    int bytes_per_line{0};
    bool streaming{false};
    std::vector<MappedBuffer> buffers;
#endif
#ifdef RK_NAV_WITH_RGA
    bool rga_enabled{true};
#endif
};

CameraVideoSource::CameraVideoSource(CameraOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}

CameraVideoSource::~CameraVideoSource() { close(); }

Status CameraVideoSource::open() {
    close();
#ifdef RK_NAV_WITH_RGA
    impl_->rga_enabled = true;
#endif
    const auto validation = validateOptions(impl_->options);
    if (!validation.isOk()) return validation;
    if (!std::filesystem::exists(impl_->options.device)) {
        return Status::notFound("camera device not found: " + impl_->options.device);
    }

    impl_->gst_pipeline = buildPipeline(impl_->options);
#ifdef __linux__
    impl_->descriptor = ::open(impl_->options.device.c_str(), O_RDWR | O_NONBLOCK);
    if (impl_->descriptor < 0) return systemFailure("open camera");
    v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    format.fmt.pix_mp.width = static_cast<__u32>(impl_->options.width);
    format.fmt.pix_mp.height = static_cast<__u32>(impl_->options.height);
    format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
    format.fmt.pix_mp.field = V4L2_FIELD_NONE;
    if (retryIoctl(impl_->descriptor, VIDIOC_S_FMT, &format) < 0) {
        const auto failure = systemFailure("VIDIOC_S_FMT");
        close();
        return failure;
    }
    if (format.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_NV12 ||
        static_cast<int>(format.fmt.pix_mp.width) != impl_->options.width ||
        static_cast<int>(format.fmt.pix_mp.height) != impl_->options.height ||
        format.fmt.pix_mp.num_planes != 1) {
        close();
        return Status::unavailable("camera did not accept single-plane NV12 format");
    }
    impl_->bytes_per_line = static_cast<int>(format.fmt.pix_mp.plane_fmt[0].bytesperline);
    if (impl_->bytes_per_line != impl_->options.width) {
        close();
        return Status::unavailable("padded NV12 camera stride is not supported");
    }
    v4l2_requestbuffers request{};
    request.count = 4;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    request.memory = V4L2_MEMORY_MMAP;
    if (retryIoctl(impl_->descriptor, VIDIOC_REQBUFS, &request) < 0) {
        const auto failure = systemFailure("VIDIOC_REQBUFS");
        close();
        return failure;
    }
    if (request.count < 2) {
        close();
        return Status::unavailable("camera allocated fewer than two mmap buffers");
    }
    impl_->buffers.resize(request.count);
    for (__u32 index = 0; index < request.count; ++index) {
        v4l2_plane plane{};
        v4l2_buffer buffer{};
        buffer.type = request.type;
        buffer.memory = request.memory;
        buffer.index = index;
        buffer.length = 1;
        buffer.m.planes = &plane;
        if (retryIoctl(impl_->descriptor, VIDIOC_QUERYBUF, &buffer) < 0) {
            const auto failure = systemFailure("VIDIOC_QUERYBUF");
            close();
            return failure;
        }
        auto& mapped = impl_->buffers[index];
        mapped.length = plane.length;
        mapped.address = mmap(nullptr, mapped.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                              impl_->descriptor, static_cast<off_t>(plane.m.mem_offset));
        if (mapped.address == MAP_FAILED) {
            const auto failure = systemFailure("mmap camera buffer");
            close();
            return failure;
        }
        if (retryIoctl(impl_->descriptor, VIDIOC_QBUF, &buffer) < 0) {
            const auto failure = systemFailure("VIDIOC_QBUF");
            close();
            return failure;
        }
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (retryIoctl(impl_->descriptor, VIDIOC_STREAMON, &type) < 0) {
        const auto failure = systemFailure("VIDIOC_STREAMON");
        close();
        return failure;
    }
    impl_->streaming = true;
#else
    if (!impl_->capture.open(impl_->gst_pipeline, cv::CAP_GSTREAMER)) {
        return Status::unavailable("cannot open camera pipeline: " + impl_->gst_pipeline);
    }
#endif
    impl_->started_at = std::chrono::steady_clock::now();
    impl_->sequence = 0;
    return Status::ok();
}

Result<Frame> CameraVideoSource::read() {
    std::int64_t capture_ns = 0; // Zero means driver timestamp is unverified.
#ifdef __linux__
    if (impl_->descriptor < 0 || !impl_->streaming) {
        return Status::unavailable("camera source is not open");
    }
    pollfd descriptor{impl_->descriptor, POLLIN, 0};
    int poll_result = 0;
    do {
        poll_result = poll(&descriptor, 1, 2000);
    } while (poll_result < 0 && errno == EINTR);
    if (poll_result == 0) return Status::unavailable("camera frame timed out");
    if (poll_result < 0) return systemFailure("poll camera");

    v4l2_plane plane{};
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.length = 1;
    buffer.m.planes = &plane;
    if (retryIoctl(impl_->descriptor, VIDIOC_DQBUF, &buffer) < 0) {
        return systemFailure("VIDIOC_DQBUF");
    }
    if (buffer.index >= impl_->buffers.size()) {
        return Status::internal("camera returned an invalid buffer index");
    }
    // Preserve dequeue metadata before re-queueing; only exposure-start monotonic timestamps qualify.
    if ((buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC &&
        (buffer.flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) == V4L2_BUF_FLAG_TSTAMP_SRC_SOE &&
        buffer.timestamp.tv_sec >= 0 && buffer.timestamp.tv_usec >= 0 && buffer.timestamp.tv_usec < 1000000) {
        capture_ns = static_cast<std::int64_t>(buffer.timestamp.tv_sec) * 1000000000LL +
                     static_cast<std::int64_t>(buffer.timestamp.tv_usec) * 1000LL;
    }
    const std::size_t required = static_cast<std::size_t>(impl_->options.width) *
        static_cast<std::size_t>(impl_->options.height) * 3U / 2U;
    auto& mapped = impl_->buffers[buffer.index];
    if (plane.bytesused < required || mapped.length < required) {
        retryIoctl(impl_->descriptor, VIDIOC_QBUF, &buffer);
        return Status::unavailable("camera returned a truncated NV12 frame");
    }
    cv::Mat nv12(impl_->options.height * 3 / 2, impl_->options.width, CV_8UC1,
                 mapped.address, static_cast<std::size_t>(impl_->bytes_per_line));
    cv::Mat image;
#ifdef RK_NAV_WITH_RGA
    if (impl_->rga_enabled) {
        image.create(impl_->options.height, impl_->options.width, CV_8UC3);
        const rga_buffer_t source = wrapbuffer_virtualaddr_t(
            nv12.data, impl_->options.width, impl_->options.height,
            impl_->options.width, impl_->options.height, RK_FORMAT_YCbCr_420_SP);
        const rga_buffer_t destination = wrapbuffer_virtualaddr_t(
            image.data, impl_->options.width, impl_->options.height,
            impl_->options.width, impl_->options.height, RK_FORMAT_BGR_888);
        const IM_STATUS conversion = imcvtcolor_t(
            source, destination, RK_FORMAT_YCbCr_420_SP, RK_FORMAT_BGR_888,
            IM_COLOR_SPACE_DEFAULT, 1);
        if (conversion != IM_STATUS_SUCCESS) {
            impl_->rga_enabled = false;
            std::cerr << "warning: RGA NV12 to BGR conversion failed; "
                         "falling back to OpenCV CPU conversion: "
                      << imStrError(conversion) << '\n';
        }
    }
    if (!impl_->rga_enabled) {
        cv::cvtColor(nv12, image, cv::COLOR_YUV2BGR_NV12);
    }
#else
    cv::cvtColor(nv12, image, cv::COLOR_YUV2BGR_NV12);
#endif
    if (retryIoctl(impl_->descriptor, VIDIOC_QBUF, &buffer) < 0) {
        return systemFailure("VIDIOC_QBUF");
    }
#else
    if (!impl_->capture.isOpened()) return Status::unavailable("camera source is not open");
    cv::Mat image;
    if (!impl_->capture.read(image) || image.empty()) {
        return Status::unavailable("camera frame read failed: " + impl_->options.device);
    }
#endif
    rotateFrame(image, impl_->options.rotation_degrees);

    Frame frame;
    frame.image = std::move(image);
    frame.capture_time_ns = capture_ns;
    frame.sequence = impl_->sequence++;
    return frame;
}

void CameraVideoSource::close() {
#ifdef __linux__
    if (impl_->descriptor >= 0 && impl_->streaming) {
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        retryIoctl(impl_->descriptor, VIDIOC_STREAMOFF, &type);
    }
    impl_->streaming = false;
    for (auto& buffer : impl_->buffers) {
        if (buffer.address != MAP_FAILED) munmap(buffer.address, buffer.length);
    }
    impl_->buffers.clear();
    if (impl_->descriptor >= 0) ::close(impl_->descriptor);
    impl_->descriptor = -1;
#else
    if (impl_->capture.isOpened()) impl_->capture.release();
#endif
    impl_->sequence = 0;
}

double CameraVideoSource::fps() const { return static_cast<double>(impl_->options.fps); }
std::uint64_t CameraVideoSource::framesRead() const { return impl_->sequence; }
const std::string& CameraVideoSource::pipeline() const { return impl_->gst_pipeline; }

}  // namespace rk_nav
