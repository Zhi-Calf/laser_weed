#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "rk_nav/calibration/camera_calibration.hpp"

namespace {

void label(cv::Mat& image, const std::string& text) {
    cv::rectangle(image, cv::Rect(0, 0, 260, 48), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(image, text, cv::Point(14, 33), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "usage: camera_calibration_validate <calibration.yaml> <image_glob> "
                     "<output_dir>\n";
        return EXIT_FAILURE;
    }
    const auto calibration = rk_nav::loadCameraCalibration(argv[1]);
    if (!calibration.isOk()) {
        std::cerr << "error: " << calibration.status().message() << '\n';
        return EXIT_FAILURE;
    }
    std::vector<cv::String> paths;
    cv::glob(argv[2], paths, false);
    if (paths.empty()) {
        std::cerr << "error: image glob matched no files\n";
        return EXIT_FAILURE;
    }
    const std::filesystem::path output_dir(argv[3]);
    std::error_code filesystem_error;
    std::filesystem::create_directories(output_dir, filesystem_error);
    if (filesystem_error) {
        std::cerr << "error: cannot create output directory: "
                  << filesystem_error.message() << '\n';
        return EXIT_FAILURE;
    }

    std::size_t processed = 0;
    std::size_t rejected = 0;
    for (const auto& path_string : paths) {
        const std::filesystem::path path(path_string);
        cv::Mat original = cv::imread(path.string());
        if (original.empty() || original.size() != calibration.value().image_size) {
            ++rejected;
            std::cerr << "rejected=" << path << " reason=unreadable_or_wrong_resolution\n";
            continue;
        }
        const auto undistorted_result = calibration.value().undistort(original);
        if (!undistorted_result.isOk()) {
            ++rejected;
            std::cerr << "rejected=" << path
                      << " reason=" << undistorted_result.status().message() << '\n';
            continue;
        }
        cv::Mat undistorted = undistorted_result.value();
        cv::Mat original_labelled = original.clone();
        cv::Mat undistorted_labelled = undistorted.clone();
        label(original_labelled, "original");
        label(undistorted_labelled, "undistorted");
        cv::Mat comparison;
        cv::hconcat(original_labelled, undistorted_labelled, comparison);
        const auto stem = path.stem().string();
        const std::vector<int> jpeg_parameters{cv::IMWRITE_JPEG_QUALITY, 95};
        if (!cv::imwrite((output_dir / (stem + "_undistorted.jpg")).string(),
                         undistorted, jpeg_parameters) ||
            !cv::imwrite((output_dir / (stem + "_comparison.jpg")).string(),
                         comparison, jpeg_parameters)) {
            std::cerr << "error: failed to write validation output for " << path << '\n';
            return EXIT_FAILURE;
        }
        ++processed;
    }
    if (processed == 0) {
        std::cerr << "error: no validation image was processed\n";
        return EXIT_FAILURE;
    }

    const auto diagnostics = calibration.value().undistortDiagnostics();
    if (!diagnostics.isOk()) {
        std::cerr << "error: " << diagnostics.status().message() << '\n';
        return EXIT_FAILURE;
    }
    const auto summary_path = output_dir / "validation-summary.yaml";
    cv::FileStorage summary(summary_path.string(), cv::FileStorage::WRITE);
    if (!summary.isOpened()) {
        std::cerr << "error: cannot write " << summary_path << '\n';
        return EXIT_FAILURE;
    }
    summary << "schema_version" << 1;
    summary << "human_review_required" << 1;
    summary << "camera_name" << calibration.value().camera_name;
    summary << "image_width" << calibration.value().image_size.width;
    summary << "image_height" << calibration.value().image_size.height;
    summary << "calibration_reprojection_rms_px" << calibration.value().reprojection_rms_px;
    summary << "processed_images" << static_cast<int>(processed);
    summary << "rejected_images" << static_cast<int>(rejected);
    summary << "mean_displacement_px" << diagnostics.value().mean_displacement_px;
    summary << "p95_displacement_px" << diagnostics.value().p95_displacement_px;
    summary << "max_displacement_px" << diagnostics.value().max_displacement_px;
    summary << "edge_p95_displacement_px" << diagnostics.value().edge_p95_displacement_px;
    summary << "invalid_border_percent" << diagnostics.value().invalid_border_percent;
    summary.release();

    std::cout << std::fixed << std::setprecision(2)
              << "processed=" << processed << " rejected=" << rejected
              << " rms_px=" << calibration.value().reprojection_rms_px
              << " warp_p95_px=" << diagnostics.value().p95_displacement_px
              << " edge_warp_p95_px=" << diagnostics.value().edge_p95_displacement_px
              << " invalid_border_pct=" << diagnostics.value().invalid_border_percent
              << " output_dir=" << output_dir << '\n'
              << "review_required=1 check straight structures near all four image edges\n";
    return EXIT_SUCCESS;
}
