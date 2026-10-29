#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "rk_nav/geometry/bev_calibration.hpp"

namespace {

struct PickerState {
    double display_scale{1.0};
    std::atomic<bool> click_pending{false};
    cv::Point2d clicked_image_pixel;
};

void onMouse(int event, int x, int y, int, void* user_data) {
    if (event != cv::EVENT_LBUTTONDOWN) return;
    auto* state = static_cast<PickerState*>(user_data);
    if (state->click_pending.load()) return;
    state->clicked_image_pixel = {
        static_cast<double>(x) / state->display_scale,
        static_cast<double>(y) / state->display_scale,
    };
    state->click_pending.store(true);
}

bool readGroundCoordinate(cv::Point2d& coordinate) {
    std::cout << "Enter forward_m left_m (or 'cancel'): " << std::flush;
    std::string line;
    if (!std::getline(std::cin, line) || line == "cancel") return false;
    std::stringstream stream(line);
    double forward = 0.0;
    double left = 0.0;
    std::string trailing;
    if (!(stream >> forward >> left) || (stream >> trailing) ||
        !std::isfinite(forward) || !std::isfinite(left)) {
        std::cout << "Invalid coordinate; click was discarded.\n";
        return false;
    }
    coordinate = {forward, left};
    return true;
}

cv::Mat render(const cv::Mat& source,
               const std::vector<rk_nav::BevGroundControlPoint>& points,
               bool fit_role,
               double scale) {
    cv::Mat overlay = source.clone();
    for (std::size_t index = 0; index < points.size(); ++index) {
        const auto& point = points[index];
        const cv::Scalar color = point.fit ? cv::Scalar(0, 210, 0)
                                           : cv::Scalar(0, 170, 255);
        const cv::Point center(cvRound(point.image_pixel.x), cvRound(point.image_pixel.y));
        cv::circle(overlay, center, 8, color, 2, cv::LINE_AA);
        cv::putText(overlay, std::to_string(index + 1), center + cv::Point(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv::LINE_AA);
    }
    cv::rectangle(overlay, cv::Rect(0, 0, 1120, 58), cv::Scalar(0, 0, 0), cv::FILLED);
    const std::string instructions =
        std::string("role=") + (fit_role ? "FIT" : "CHECK") +
        " | F/C role | click point | U undo | S save | Q save+quit | Esc abort";
    cv::putText(overlay, instructions, cv::Point(12, 38), cv::FONT_HERSHEY_SIMPLEX,
                0.72, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    if (scale == 1.0) return overlay;
    cv::Mat displayed;
    cv::resize(overlay, displayed, cv::Size(), scale, scale, cv::INTER_AREA);
    return displayed;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3 && argc != 4) {
        std::cerr << "usage: bev_point_picker <undistorted_image> <output.csv> [--resume]\n";
        return EXIT_FAILURE;
    }
    const bool resume = argc == 4 && std::string(argv[3]) == "--resume";
    if (argc == 4 && !resume) {
        std::cerr << "error: the only optional argument is --resume\n";
        return EXIT_FAILURE;
    }
    const std::filesystem::path output_path(argv[2]);
    if (std::filesystem::exists(output_path) && !resume) {
        std::cerr << "error: output already exists; use --resume to edit it without overwriting "
                     "unseen data\n";
        return EXIT_FAILURE;
    }
    cv::Mat image = cv::imread(argv[1]);
    if (image.empty()) {
        std::cerr << "error: cannot read image " << argv[1] << '\n';
        return EXIT_FAILURE;
    }
    std::vector<rk_nav::BevGroundControlPoint> points;
    if (resume) {
        const auto loaded = rk_nav::loadBevGroundControlPointsCsv(output_path);
        if (!loaded.isOk()) {
            std::cerr << "error: " << loaded.status().message() << '\n';
            return EXIT_FAILURE;
        }
        points = loaded.value();
    }

    PickerState state;
    state.display_scale = std::min(1.0, std::min(1440.0 / static_cast<double>(image.cols),
                                                810.0 / static_cast<double>(image.rows)));
    bool fit_role = true;
    const std::string window_name = "BEV ground control point picker";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(window_name, onMouse, &state);
    std::cout << "The image must already be undistorted. Coordinates use forward-positive, "
                 "left-positive vehicle axes.\n";

    bool saved = false;
    while (true) {
        cv::imshow(window_name, render(image, points, fit_role, state.display_scale));
        if (state.click_pending.exchange(false)) {
            cv::Point2d ground;
            if (readGroundCoordinate(ground)) {
                points.push_back({state.clicked_image_pixel, ground, fit_role});
                std::cout << "added point=" << points.size()
                          << " image_u=" << state.clicked_image_pixel.x
                          << " image_v=" << state.clicked_image_pixel.y
                          << " forward_m=" << ground.x << " left_m=" << ground.y
                          << " role=" << (fit_role ? "fit" : "check") << '\n';
            }
        }
        const int key = cv::waitKey(30) & 0xff;
        if (key == 'f' || key == 'F') fit_role = true;
        else if (key == 'c' || key == 'C') fit_role = false;
        else if ((key == 'u' || key == 'U') && !points.empty()) points.pop_back();
        else if (key == 's' || key == 'S' || key == 'q' || key == 'Q') {
            const auto parent = output_path.parent_path();
            std::error_code error;
            if (!parent.empty()) std::filesystem::create_directories(parent, error);
            if (error) {
                std::cerr << "error: cannot create output directory: " << error.message() << '\n';
                cv::destroyAllWindows();
                return EXIT_FAILURE;
            }
            const auto status = rk_nav::saveBevGroundControlPointsCsv(output_path, points);
            if (!status.isOk()) {
                std::cerr << "error: " << status.message() << '\n';
                if (key == 'q' || key == 'Q') continue;
            } else {
                saved = true;
                std::cout << "saved=" << output_path << " points=" << points.size() << '\n';
                if (key == 'q' || key == 'Q') break;
            }
        } else if (key == 27) {
            std::cout << "aborted_without_saving=1\n";
            break;
        }
    }
    cv::destroyAllWindows();
    return saved ? EXIT_SUCCESS : EXIT_FAILURE;
}
