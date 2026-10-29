#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "rk_nav/video/camera_video_source.hpp"

namespace {

std::atomic<bool> stop_requested{false};

void requestStop(int) { stop_requested.store(true); }

std::string timestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t value = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
#ifdef _WIN32
    localtime_s(&local, &value);
#else
    localtime_r(&value, &local);
#endif
    std::ostringstream stream;
    stream << std::put_time(&local, "%Y%m%d_%H%M%S");
    return stream.str();
}

struct ImageQuality {
    double sharpness{0.0};
    double mean_brightness{0.0};
    double dark_percent{0.0};
    double bright_percent{0.0};
};

ImageQuality measureQuality(const cv::Mat& image) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);
    cv::Scalar mean;
    cv::Scalar deviation;
    cv::meanStdDev(laplacian, mean, deviation);
    const double pixels = static_cast<double>(gray.total());
    ImageQuality quality;
    quality.sharpness = deviation[0] * deviation[0];
    quality.mean_brightness = cv::mean(gray)[0];
    quality.dark_percent = 100.0 * static_cast<double>(cv::countNonZero(gray < 5)) / pixels;
    quality.bright_percent = 100.0 * static_cast<double>(cv::countNonZero(gray > 250)) / pixels;
    return quality;
}

std::uint64_t parseUnsigned(const char* value, const char* name, std::uint64_t minimum,
                            std::uint64_t maximum) {
    std::size_t parsed = 0;
    const std::string input(value);
    const auto result = std::stoull(input, &parsed);
    if (parsed != input.size() || result < minimum || result > maximum) {
        throw std::invalid_argument(name);
    }
    return result;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2 || argc > 6) {
        std::cerr << "usage: charuco_capture <output_dir> [images=40] [interval_ms=3000] "
                     "[start_delay_s=10] [device=/dev/video12]\n";
        return EXIT_FAILURE;
    }

    std::uint64_t requested = 40;
    std::uint64_t interval_ms = 3000;
    std::uint64_t start_delay_s = 10;
    try {
        if (argc >= 3) requested = parseUnsigned(argv[2], "images", 1, 1000);
        if (argc >= 4) interval_ms = parseUnsigned(argv[3], "interval_ms", 100, 600000);
        if (argc >= 5) start_delay_s = parseUnsigned(argv[4], "start_delay_s", 0, 3600);
    } catch (...) {
        std::cerr << "error: images must be 1..1000, interval_ms 100..600000, and "
                     "start_delay_s 0..3600\n";
        return EXIT_FAILURE;
    }

    const std::filesystem::path output_dir(argv[1]);
    std::error_code filesystem_error;
    std::filesystem::create_directories(output_dir, filesystem_error);
    if (filesystem_error) {
        std::cerr << "error: cannot create output directory: " << filesystem_error.message() << '\n';
        return EXIT_FAILURE;
    }

    rk_nav::CameraOptions options;
    if (argc >= 6) options.device = argv[5];
    rk_nav::CameraVideoSource camera(std::move(options));
    const auto open_status = camera.open();
    if (!open_status.isOk()) {
        std::cerr << "error: " << open_status.message() << '\n';
        return EXIT_FAILURE;
    }

    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    std::cout << "pipeline=" << camera.pipeline() << '\n'
              << "output_dir=" << output_dir << '\n'
              << "images=" << requested << " interval_ms=" << interval_ms
              << " start_delay_s=" << start_delay_s << '\n'
              << "Keep the board still around each capture. Press Ctrl-C to stop safely.\n";

    using Clock = std::chrono::steady_clock;
    const auto warmup_until = Clock::now() + std::chrono::seconds(start_delay_s);
    while (!stop_requested.load() && Clock::now() < warmup_until) {
        auto frame = camera.read();
        if (!frame.isOk()) {
            std::cerr << "error during camera warmup: " << frame.status().message() << '\n';
            return EXIT_FAILURE;
        }
    }

    const std::string session = timestamp();
    auto next_capture = Clock::now();
    std::uint64_t saved = 0;
    while (!stop_requested.load() && saved < requested) {
        auto frame = camera.read();
        if (!frame.isOk()) {
            std::cerr << "error: " << frame.status().message() << '\n';
            return EXIT_FAILURE;
        }
        const auto now = Clock::now();
        if (now < next_capture) continue;

        ++saved;
        std::ostringstream filename;
        filename << "cmd108_" << session << '_' << std::setw(3) << std::setfill('0') << saved
                 << ".jpg";
        const auto path = output_dir / filename.str();
        const std::vector<int> encode_parameters{cv::IMWRITE_JPEG_QUALITY, 95};
        if (!cv::imwrite(path.string(), frame.value().image, encode_parameters)) {
            std::cerr << "error: failed to write " << path << '\n';
            return EXIT_FAILURE;
        }
        const auto quality = measureQuality(frame.value().image);
        std::cout << "saved=" << saved << '/' << requested << " path=" << path
                  << std::fixed << std::setprecision(1)
                  << " sharpness=" << quality.sharpness
                  << " brightness=" << quality.mean_brightness
                  << " dark_pct=" << quality.dark_percent
                  << " bright_pct=" << quality.bright_percent;
        if (quality.mean_brightness < 30.0) std::cout << " warning=too_dark";
        else if (quality.mean_brightness > 225.0) std::cout << " warning=too_bright";
        std::cout << '\n';
        next_capture = now + std::chrono::milliseconds(interval_ms);
    }

    std::cout << "summary saved=" << saved << " requested=" << requested
              << (stop_requested.load() ? " interrupted=1" : " interrupted=0") << '\n';
    return saved > 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
