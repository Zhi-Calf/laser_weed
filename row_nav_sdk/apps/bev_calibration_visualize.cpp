#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "rk_nav/geometry/bev_calibration.hpp"
#include "rk_nav/geometry/bev_projector.hpp"

namespace {

struct ErrorAccumulator {
    std::size_t count{0};
    double squared_meters{0.0};
    double max_meters{0.0};

    void add(double error_meters) {
        ++count;
        squared_meters += error_meters * error_meters;
        max_meters = std::max(max_meters, error_meters);
    }

    [[nodiscard]] double rmse() const {
        return count == 0 ? 0.0 : std::sqrt(squared_meters / static_cast<double>(count));
    }
};

void drawCross(cv::Mat& image, const cv::Point2d& point, const cv::Scalar& color) {
    const cv::Point center(cvRound(point.x), cvRound(point.y));
    cv::line(image, center + cv::Point(-8, -8), center + cv::Point(8, 8), color, 2,
             cv::LINE_AA);
    cv::line(image, center + cv::Point(-8, 8), center + cv::Point(8, -8), color, 2,
             cv::LINE_AA);
}

bool inside(const cv::Mat& image, const cv::Point2d& point) {
    return point.x >= 0.0 && point.x < image.cols && point.y >= 0.0 && point.y < image.rows;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr << "usage: bev_calibration_visualize <undistorted_image> <points.csv> "
                     "<bev.yaml> <output_prefix>\n";
        return EXIT_FAILURE;
    }
    cv::Mat image = cv::imread(argv[1]);
    if (image.empty()) {
        std::cerr << "error: cannot read image " << argv[1] << '\n';
        return EXIT_FAILURE;
    }
    const auto points = rk_nav::loadBevGroundControlPointsCsv(argv[2]);
    if (!points.isOk()) {
        std::cerr << "error: " << points.status().message() << '\n';
        return EXIT_FAILURE;
    }
    const auto config = rk_nav::loadBevConfigForInspection(argv[3]);
    if (!config.isOk()) {
        std::cerr << "error: " << config.status().message() << '\n';
        return EXIT_FAILURE;
    }
    rk_nav::BevProjector projector(config.value());
    cv::Mat image_overlay = image.clone();
    cv::Mat bev_overlay;
    cv::warpPerspective(image, bev_overlay, cv::Mat(config.value().image_to_bev),
                        config.value().output_size);
    const cv::Mat inverse = cv::Mat(config.value().image_to_bev).inv();
    ErrorAccumulator fit_errors;
    ErrorAccumulator check_errors;

    for (std::size_t index = 0; index < points.value().size(); ++index) {
        const auto& point = points.value()[index];
        const auto expected_bev = projector.groundToBev(point.ground_m);
        std::vector<cv::Point2d> expected_bev_vector{expected_bev};
        std::vector<cv::Point2d> predicted_image_vector;
        cv::perspectiveTransform(expected_bev_vector, predicted_image_vector, inverse);
        const auto predicted_image = predicted_image_vector[0];

        std::vector<cv::Point2d> clicked_image_vector{point.image_pixel};
        std::vector<cv::Point2d> projected_bev_vector;
        cv::perspectiveTransform(clicked_image_vector, projected_bev_vector,
                                 cv::Mat(config.value().image_to_bev));
        const auto projected_ground = projector.imageToGround(point.image_pixel);
        if (!projected_ground.isOk()) continue;
        const double error_m = cv::norm(projected_ground.value() - point.ground_m);
        (point.fit ? fit_errors : check_errors).add(error_m);

        const cv::Scalar actual_color = point.fit ? cv::Scalar(0, 210, 0)
                                                   : cv::Scalar(0, 170, 255);
        const cv::Point actual(cvRound(point.image_pixel.x), cvRound(point.image_pixel.y));
        const cv::Point predicted(cvRound(predicted_image.x), cvRound(predicted_image.y));
        if (inside(image_overlay, point.image_pixel)) {
            cv::circle(image_overlay, actual, 7, actual_color, 2, cv::LINE_AA);
            cv::putText(image_overlay, std::to_string(index + 1), actual + cv::Point(9, -9),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, actual_color, 2, cv::LINE_AA);
        }
        if (inside(image_overlay, predicted_image)) {
            drawCross(image_overlay, predicted_image, cv::Scalar(255, 0, 255));
            cv::line(image_overlay, actual, predicted, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }

        const cv::Point expected_bev_i(cvRound(expected_bev.x), cvRound(expected_bev.y));
        const cv::Point projected_bev_i(cvRound(projected_bev_vector[0].x),
                                        cvRound(projected_bev_vector[0].y));
        cv::circle(bev_overlay, expected_bev_i, 6, actual_color, 2, cv::LINE_AA);
        drawCross(bev_overlay, projected_bev_vector[0], cv::Scalar(255, 0, 255));
        cv::line(bev_overlay, expected_bev_i, projected_bev_i, cv::Scalar(0, 0, 255), 2,
                 cv::LINE_AA);
    }

    cv::rectangle(image_overlay, cv::Rect(0, 0, 700, 54), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(image_overlay,
                "circle=measured (green fit, orange check), cross=reprojected",
                cv::Point(12, 35), cv::FONT_HERSHEY_SIMPLEX, 0.65,
                cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    cv::circle(bev_overlay,
               cv::Point(cvRound(config.value().vehicle_origin_u),
                         cvRound(config.value().vehicle_origin_v)),
               8, cv::Scalar(255, 255, 0), cv::FILLED, cv::LINE_AA);

    const std::filesystem::path prefix(argv[4]);
    const auto parent = prefix.parent_path();
    std::error_code filesystem_error;
    if (!parent.empty()) std::filesystem::create_directories(parent, filesystem_error);
    if (filesystem_error ||
        !cv::imwrite(prefix.string() + "-image.jpg", image_overlay) ||
        !cv::imwrite(prefix.string() + "-bev.jpg", bev_overlay)) {
        std::cerr << "error: cannot write visualization outputs\n";
        return EXIT_FAILURE;
    }
    std::cout << std::fixed << std::setprecision(4)
              << "fit_count=" << fit_errors.count << " fit_rmse_m=" << fit_errors.rmse()
              << " fit_max_m=" << fit_errors.max_meters
              << " check_count=" << check_errors.count
              << " check_rmse_m=" << check_errors.rmse()
              << " check_max_m=" << check_errors.max_meters << '\n'
              << "image_output=" << prefix.string() + "-image.jpg" << '\n'
              << "bev_output=" << prefix.string() + "-bev.jpg" << '\n';
    return EXIT_SUCCESS;
}
