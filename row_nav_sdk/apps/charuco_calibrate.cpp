#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/imgcodecs.hpp>

#include "rk_nav/calibration/camera_calibration.hpp"

namespace {

int usage() {
    std::cerr << "usage: charuco_calibrate <image_glob> <output.yaml> "
                 "<squares_x> <squares_y> <square_m> <marker_m> [camera_name]\n";
    return EXIT_FAILURE;
}

}  // namespace

int main(int argc, char** argv) {
#if CV_VERSION_MAJOR < 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR < 7)
    std::cerr << "error: OpenCV " << CV_VERSION
              << " uses the legacy ChArUco board layout. It is incompatible with the "
                 "project's modern even-row printed board. Use "
                 "scripts/calibration/calibrate_charuco.py with OpenCV >= 4.7.\n";
    return EXIT_FAILURE;
#endif
    if (argc != 7 && argc != 8) return usage();
    const int squares_x = std::stoi(argv[3]);
    const int squares_y = std::stoi(argv[4]);
    const float square_m = std::stof(argv[5]);
    const float marker_m = std::stof(argv[6]);
    if (squares_x < 3 || squares_y < 3 || square_m <= 0.0F || marker_m <= 0.0F || marker_m >= square_m) {
        std::cerr << "error: invalid Charuco board dimensions\n";
        return EXIT_FAILURE;
    }

    std::vector<cv::String> paths;
    cv::glob(argv[1], paths, false);
    if (paths.size() < 5) {
        std::cerr << "error: at least five calibration images are required\n";
        return EXIT_FAILURE;
    }

    const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
    const auto board = cv::aruco::CharucoBoard::create(squares_x, squares_y, square_m, marker_m, dictionary);
    std::vector<std::vector<cv::Point2f>> all_corners;
    std::vector<std::vector<int>> all_ids;
    cv::Size image_size;
    std::size_t rejected_images = 0;

    for (const auto& path : paths) {
        const cv::Mat image = cv::imread(path);
        if (image.empty() || (!image_size.empty() && image.size() != image_size)) {
            ++rejected_images;
            continue;
        }
        image_size = image.size();
        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<int> marker_ids;
        cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids);
        if (marker_ids.empty()) {
            ++rejected_images;
            continue;
        }
        std::vector<cv::Point2f> charuco_corners;
        std::vector<int> charuco_ids;
        cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, image, board,
                                             charuco_corners, charuco_ids);
        if (charuco_ids.size() < 6) {
            ++rejected_images;
            continue;
        }
        all_corners.push_back(std::move(charuco_corners));
        all_ids.push_back(std::move(charuco_ids));
    }
    if (all_corners.size() < 5 || image_size.empty()) {
        std::cerr << "error: fewer than five usable Charuco views\n";
        return EXIT_FAILURE;
    }

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distortion;
    std::vector<cv::Mat> rotations;
    std::vector<cv::Mat> translations;
    const double rms = cv::aruco::calibrateCameraCharuco(
        all_corners, all_ids, board, image_size, camera_matrix, distortion, rotations, translations,
        cv::CALIB_RATIONAL_MODEL);

    rk_nav::CameraCalibration calibration;
    calibration.camera_name = argc == 8 ? argv[7] : "camera";
    calibration.image_size = image_size;
    calibration.camera_matrix = camera_matrix;
    distortion.convertTo(distortion, CV_64F);
    calibration.distortion_coefficients.assign(
        distortion.ptr<double>(), distortion.ptr<double>() + distortion.total());
    calibration.reprojection_rms_px = rms;
    const auto status = rk_nav::saveCameraCalibration(argv[2], calibration);
    if (!status.isOk()) {
        std::cerr << "error: " << status.message() << '\n';
        return EXIT_FAILURE;
    }

    std::cout << "saved=" << argv[2] << " usable_views=" << all_corners.size()
              << " rejected_views=" << rejected_images << " rms_px=" << rms
              << " image_size=" << image_size.width << 'x' << image_size.height << '\n';
    return EXIT_SUCCESS;
}
