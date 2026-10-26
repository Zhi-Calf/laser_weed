#include <cstdlib>
#include <iomanip>
#include <iostream>

#include "rk_nav/geometry/bev_calibration.hpp"

int main(int argc, char** argv) {
    if (argc < 3 || argc > 10) {
        std::cerr << "usage: bev_calibrate <points.csv> <output.yaml> [width=640] [height=800] "
                     "[meters_per_pixel=0.01] [origin_u=320] [origin_v=799] "
                     "[max_check_rmse_m=0.05] [ransac_threshold_px=3.0]\n";
        return EXIT_FAILURE;
    }

    rk_nav::BevConfig layout;
    double max_check_rmse_m = 0.05;
    double ransac_threshold_pixels = 3.0;
    try {
        if (argc >= 4) layout.output_size.width = std::stoi(argv[3]);
        if (argc >= 5) layout.output_size.height = std::stoi(argv[4]);
        if (argc >= 6) layout.meters_per_pixel = std::stod(argv[5]);
        if (argc >= 7) layout.vehicle_origin_u = std::stod(argv[6]);
        if (argc >= 8) layout.vehicle_origin_v = std::stod(argv[7]);
        if (argc >= 9) max_check_rmse_m = std::stod(argv[8]);
        if (argc >= 10) ransac_threshold_pixels = std::stod(argv[9]);
    } catch (...) {
        std::cerr << "error: invalid numeric output-layout argument\n";
        return EXIT_FAILURE;
    }

    const auto points = rk_nav::loadBevGroundControlPointsCsv(argv[1]);
    if (!points.isOk()) {
        std::cerr << "error: " << points.status().message() << '\n';
        return EXIT_FAILURE;
    }
    const auto calibration = rk_nav::calibrateBev(points.value(), layout, max_check_rmse_m,
                                                   ransac_threshold_pixels);
    if (!calibration.isOk()) {
        std::cerr << "error: " << calibration.status().message() << '\n';
        return EXIT_FAILURE;
    }
    const auto save_status = rk_nav::saveBevCalibration(argv[2], calibration.value());
    if (!save_status.isOk()) {
        std::cerr << "error: " << save_status.message() << '\n';
        return EXIT_FAILURE;
    }

    const auto& result = calibration.value();
    std::cout << std::fixed << std::setprecision(4)
              << "fit points=" << result.fit_metrics.count
              << " inliers=" << result.fit_inliers
              << " rmse_m=" << result.fit_metrics.rmse_m
              << " max_m=" << result.fit_metrics.max_m << '\n'
              << "check points=" << result.check_metrics.count
              << " rmse_m=" << result.check_metrics.rmse_m
              << " max_m=" << result.check_metrics.max_m << '\n'
              << "output=" << argv[2]
              << " calibration_valid=" << (result.accepted ? 1 : 0) << '\n';
    if (!result.accepted) {
        std::cerr << "rejected: require at least four independent check points, >=75% fit "
                     "inliers, check RMSE within threshold, and max error within 2x threshold\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
