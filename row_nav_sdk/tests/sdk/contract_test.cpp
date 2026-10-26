#include "rk_row_nav.h"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <iterator>
#define CHECK(x) do { if (!(x)) throw std::runtime_error(#x); } while(0)

int main() {
    const auto dir = std::filesystem::temp_directory_path() / ("rk-sdk-contract-" + std::to_string(rk_row_nav_now_ns()));
    std::filesystem::create_directory(dir);
    rk_row_nav_handle_t h = nullptr;
    try {
        const auto intrinsic = (dir / "intrinsic.yaml").string(), bev = (dir / "bev.yaml").string();
        std::ofstream(intrinsic) << "%YAML:1.0\nschema_version: 1\ncalibration_valid: 1\nimage_width: 320\nimage_height: 240\ncamera_matrix: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n   data: [300.,0.,160.,0.,300.,120.,0.,0.,1.]\ndistortion_coefficients: !!opencv-matrix\n   rows: 1\n   cols: 5\n   dt: d\n   data: [0.,0.,0.,0.,0.]\n";
        std::ofstream(bev) << "%YAML:1.0\nschema_version: 1\ncalibration_valid: 1\noutput_width: 320\noutput_height: 240\nmeters_per_pixel: 0.02\nvehicle_origin_u: 160.\nvehicle_origin_v: 239.\nimage_to_bev: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n   data: [1.,0.,0.,0.,1.,0.,0.,0.,1.]\n";
        rk_row_nav_config_t c; rk_row_nav_config_init(&c);
        c.model_path = "test-only"; c.intrinsic_path = intrinsic.c_str(); c.bev_path = bev.c_str(); c.max_frame_age_ms = 10000;
        c.abi_version = 2; CHECK(rk_row_nav_create(&c, &h) == RK_ROW_NAV_INVALID_ARGUMENT && !h); c.abi_version = 1;
        c.backend = 100; CHECK(rk_row_nav_create(&c, &h) == RK_ROW_NAV_UNSUPPORTED); c.backend = RK_ROW_NAV_ONNX;
        c.model_path = "missing"; CHECK(rk_row_nav_create(&c, &h) == RK_ROW_NAV_MODEL_ERROR && !h); c.model_path = "test-only";
        CHECK(rk_row_nav_create(&c, &h) == 0 && h);
        // Padded non-contiguous rows; blue lines at u=130,180 => centre to left by 0.10m.
        std::vector<uint8_t> image(240 * 976, 0);
        for (int y = 0; y < 240; ++y) for (int x : {128,129,130,131,132,178,179,180,181,182}) image[y*976+x*3] = 255;
        const auto original = image;
        rk_row_nav_frame_t f; rk_row_nav_frame_init(&f);
        f.width=320; f.height=240; f.stride_bytes=976; f.data=image.data(); f.data_size_bytes=image.size();
        f.capture_time_ns=rk_row_nav_now_ns(); f.frame_id=42;
        rk_row_nav_result_t r; rk_row_nav_result_init(&r);
        CHECK(rk_row_nav_process(h,&f,&r)==0);
        CHECK(r.observation_status==RK_ROW_NAV_OBSERVATION_VALID && r.frame_id==42);
        CHECK(std::abs(r.lateral_error_m-.10)<.03 && std::abs(r.heading_error_rad)<.001);
        CHECK(image==original);
        struct ExtendedResult { rk_row_nav_result_t value; uint64_t sentinel; } extended;
        rk_row_nav_result_init(&extended.value);
        extended.value.struct_size=sizeof(extended); extended.sentinel=0x12345678;
        CHECK(rk_row_nav_process(h,&f,&extended.value)==0 && extended.sentinel==0x12345678);
        CHECK(extended.value.struct_size==sizeof(extended));
        f.struct_size=1; CHECK(rk_row_nav_process(h,&f,&r)==RK_ROW_NAV_INVALID_ARGUMENT && std::isnan(r.lateral_error_m));
        f.struct_size=sizeof(f);
        f.pixel_format=RK_ROW_NAV_RGB888; f.capture_time_ns=rk_row_nav_now_ns();
        CHECK(rk_row_nav_process(h,&f,&r)==0 && r.observation_status==RK_ROW_NAV_OBSERVATION_NO_ROW);
        for (size_t i=0;i<image.size();i+=1) image[i]=0;
        for (int y=0;y<240;++y) for (int x : {128,129,130,131,132,178,179,180,181,182}) image[y*976+x*3+2]=255;
        CHECK(rk_row_nav_process(h,&f,&r)==0 && r.observation_status==RK_ROW_NAV_OBSERVATION_VALID);
        f.frame_id=98; CHECK(rk_row_nav_process(h,&f,&r)==0 && r.observation_status==RK_ROW_NAV_OBSERVATION_LOW_CONFIDENCE && std::isnan(r.lateral_error_m));
        f.frame_id=99; CHECK(rk_row_nav_process(h,&f,&r)==RK_ROW_NAV_INTERNAL_ERROR && std::isnan(r.lateral_error_m)); f.frame_id=42;
        f.frame_id=96; CHECK(rk_row_nav_process(h,&f,&r)==RK_ROW_NAV_INFERENCE_ERROR && std::isnan(r.lateral_error_m)); f.frame_id=42;
        f.capture_time_ns=1; CHECK(rk_row_nav_process(h,&f,&r)==0 && r.observation_status==RK_ROW_NAV_OBSERVATION_STALE && std::isnan(r.lateral_error_m));
        f.capture_time_ns=rk_row_nav_now_ns()+10000000000ULL; CHECK(rk_row_nav_process(h,&f,&r)==RK_ROW_NAV_INVALID_ARGUMENT);
        f.capture_time_ns=rk_row_nav_now_ns(); f.data_size_bytes=1; CHECK(rk_row_nav_process(h,&f,&r)==RK_ROW_NAV_INVALID_ARGUMENT);
        f.data_size_bytes=image.size(); f.pixel_format=99; CHECK(rk_row_nav_process(h,&f,&r)==RK_ROW_NAV_UNSUPPORTED);
        f.pixel_format=RK_ROW_NAV_RGB888; f.width=319;
        CHECK(rk_row_nav_process(h,&f,&r)==RK_ROW_NAV_INVALID_ARGUMENT); f.width=320;
        rk_row_nav_destroy(h); h=nullptr;
        for (int i=0;i<20;++i) { CHECK(rk_row_nav_create(&c,&h)==0); rk_row_nav_destroy(h); h=nullptr; }
        // Configure row spacing through the existing BEV file without changing the C ABI.
        std::ifstream bev_source(bev);
        const std::string original_bev((std::istreambuf_iterator<char>(bev_source)),std::istreambuf_iterator<char>());
        bev_source.close();
        std::ofstream(bev,std::ios::app)<<"row_extractor:\n   expected_spacing_m: 0.4\n   spacing_tolerance_m: 0.05\n";
        CHECK(rk_row_nav_create(&c,&h)==0);
        f.capture_time_ns=rk_row_nav_now_ns();
        CHECK(rk_row_nav_process(h,&f,&r)==0 && r.observation_status==RK_ROW_NAV_OBSERVATION_NO_ROW && std::isnan(r.lateral_error_m));
        rk_row_nav_destroy(h); h=nullptr;
        std::ofstream(bev)<<original_bev;
        std::ofstream(bev,std::ios::app)<<"row_tracker:\n   enabled: 1\n   confirmation_frames: 3\n";
        CHECK(rk_row_nav_create(&c,&h)==0);
        for (std::uint64_t i=1;i<=3;++i) {
            f.frame_id=i; f.capture_time_ns=rk_row_nav_now_ns();
            CHECK(rk_row_nav_process(h,&f,&r)==0);
            CHECK(r.observation_status==(i<3 ? RK_ROW_NAV_OBSERVATION_LOW_CONFIDENCE : RK_ROW_NAV_OBSERVATION_VALID));
        }
        CHECK(rk_row_nav_process(h,&f,&r)==RK_ROW_NAV_INVALID_ARGUMENT);
        f.frame_id=4; f.capture_time_ns=rk_row_nav_now_ns();
        CHECK(rk_row_nav_process(h,&f,&r)==0 && r.observation_status==RK_ROW_NAV_OBSERVATION_VALID);
        f.frame_id=5; f.capture_time_ns=1;
        CHECK(rk_row_nav_process(h,&f,&r)==0 && r.observation_status==RK_ROW_NAV_OBSERVATION_STALE);
        f.capture_time_ns=rk_row_nav_now_ns();
        CHECK(rk_row_nav_process(h,&f,&r)==0 && r.observation_status==RK_ROW_NAV_OBSERVATION_VALID);
        rk_row_nav_destroy(h); h=nullptr;
        std::ofstream(bev)<<original_bev;
        c.max_frame_age_ms=10; CHECK(rk_row_nav_create(&c,&h)==0);
        f.frame_id=97; f.capture_time_ns=rk_row_nav_now_ns();
        CHECK(rk_row_nav_process(h,&f,&r)==0 && r.observation_status==RK_ROW_NAV_OBSERVATION_STALE);
        CHECK(r.processing_ms>=20 && std::isnan(r.lateral_error_m));
        rk_row_nav_destroy(h); h=nullptr;
        std::ofstream(bev) << "%YAML:1.0\nschema_version: 1\ncalibration_valid: 0\n";
        CHECK(rk_row_nav_create(&c,&h)==RK_ROW_NAV_CALIBRATION_ERROR && !h);
        std::cout << "SDK contract passed: padded RGB/BGR, signs, stale, invalid, exception, lifecycle\n";
    } catch (const std::exception& e) { rk_row_nav_destroy(h); std::cerr<<e.what()<<'\n'; return 1; }
    std::filesystem::remove_all(dir);
}
