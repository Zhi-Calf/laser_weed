#include "rk_row_nav.h"
#include "dynamic_bev_fixture.hpp"
#include "rk_nav/perception/row_extractor.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
#define CHECK(x) do {if (!(x)) throw std::runtime_error(std::string(#x)+": "+rk_row_nav_get_last_error());} while (false)
int main() {
    const auto dir=std::filesystem::temp_directory_path()/("rk-dynamic-sdk-"+std::to_string(rk_row_nav_now_ns()));
    std::filesystem::create_directory(dir);
    rk_row_nav_handle_t h=nullptr;
    try {
        const auto c=dynamic_fixture::config();
        const auto intrinsic=(dir/"intrinsic.yaml").string(),bev=(dir/"bev.yaml").string();
        {
            cv::FileStorage s(intrinsic,cv::FileStorage::WRITE);
            s<<"schema_version"<<1<<"calibration_valid"<<1<<"image_width"<<640<<"image_height"<<480
             <<"camera_matrix"<<cv::Mat(dynamic_fixture::intrinsics())
             <<"distortion_coefficients"<<cv::Mat::zeros(1,5,CV_64F);
        }
        dynamic_fixture::writeBev(bev,c);
        rk_row_nav_config_t config; rk_row_nav_config_init(&config);
        config.model_path="test-only"; config.intrinsic_path=intrinsic.c_str(); config.bev_path=bev.c_str();
        config.max_frame_age_ms=5000;
        CHECK(rk_row_nav_create(&config,&h)==0);
        CHECK(rk_row_nav_dynamic_bev_enabled(h)==1);
        rk_row_nav_frame_t f; rk_row_nav_frame_init(&f);
        rk_row_nav_result_t r; rk_row_nav_result_init(&r);
        rk_row_nav_attitude_t a; rk_row_nav_attitude_init(&a); a.valid=1;
        cv::Mat image;
        for (double roll : {-.08,.08}) for (double pitch : {-.08,.08}) {
            image=dynamic_fixture::rowImage(c,roll,pitch);
            f.data=image.data; f.width=640; f.height=480; f.stride_bytes=static_cast<uint32_t>(image.step);
            f.data_size_bytes=image.total()*image.elemSize();
            f.capture_time_ns=rk_row_nav_now_ns(); ++f.frame_id;
            a.sample_time_ns=f.capture_time_ns-1000000; a.roll_rad=roll; a.pitch_rad=pitch;
            CHECK(rk_row_nav_process(h,&f,&r)==RK_ROW_NAV_INVALID_ARGUMENT);
            CHECK(std::isnan(r.lateral_error_m));
            CHECK(rk_row_nav_process_with_attitude(h,&f,&a,&r)==0);
            std::cout<<"roll="<<roll<<" pitch="<<pitch<<" status="<<r.observation_status
                     <<" lateral="<<r.lateral_error_m<<" heading="<<r.heading_error_rad<<'\n';
            if (r.observation_status!=RK_ROW_NAV_OBSERVATION_VALID) {
                auto p=rk_nav::DynamicBevProjector(dynamic_fixture::measuredBev(c),dynamic_fixture::intrinsics(),c).at(roll,pitch);
                cv::Mat mask; cv::extractChannel(image,mask,0);
                auto projected=p.value().projectMask(mask);
                auto rows=rk_nav::RowExtractor{}.extractBetweenRows(projected.value(),p.value());
                if (rows.isOk()) std::cerr<<"rows="<<rows.value().fitted_rows.size()<<" candidates="<<rows.value().candidate_points.size()
                    <<" reason="<<rows.value().rejection_reason<<'\n';
                else std::cerr<<rows.status().message()<<'\n';
            }
            CHECK(r.observation_status==RK_ROW_NAV_OBSERVATION_VALID);
            CHECK(std::abs(r.lateral_error_m-.1)<.04 && std::abs(r.heading_error_rad)<.03);
        }
        for (int failure=0;failure<8;++failure) {
            auto bad=a;
            if (failure==0) bad.valid=0;
            if (failure==1) bad.sample_time_ns=f.capture_time_ns+1;
            if (failure==2) bad.sample_time_ns=f.capture_time_ns-100000000;
            if (failure==3) bad.roll_rad=std::numeric_limits<double>::quiet_NaN();
            if (failure==4) bad.pitch_rad=.8;
            if (failure==5) bad.struct_size=1;
            if (failure==6) bad.reserved=1;
            if (failure==7) bad.abi_version=99;
            CHECK(rk_row_nav_process_with_attitude(h,&f,&bad,&r)==RK_ROW_NAV_INVALID_ARGUMENT);
            CHECK(std::isnan(r.lateral_error_m) && r.confidence==0 && r.observation_status==RK_ROW_NAV_OBSERVATION_INVALID);
        }
        CHECK(rk_row_nav_process_with_attitude(h,&f,nullptr,&r)==RK_ROW_NAV_INVALID_ARGUMENT);
        CHECK(rk_row_nav_process_with_attitude(h,&f,&a,&r)==0 && r.observation_status==RK_ROW_NAV_OBSERVATION_VALID);
        rk_row_nav_destroy(h); h=nullptr;
        dynamic_fixture::writeBev(bev,c,false);
        CHECK(rk_row_nav_create(&config,&h)==RK_ROW_NAV_CALIBRATION_ERROR && !h);
        dynamic_fixture::writeBev(bev,c,true,false);
        CHECK(rk_row_nav_create(&config,&h)==0);
        CHECK(rk_row_nav_dynamic_bev_enabled(h)==0);
        CHECK(rk_row_nav_process_with_attitude(h,&f,&a,&r)==RK_ROW_NAV_UNSUPPORTED && std::isnan(r.lateral_error_m));
        rk_row_nav_destroy(h); h=nullptr;
        std::filesystem::remove_all(dir);
        std::cout<<"dynamic SDK geometry/ABI/freshness/fail-closed contracts passed\n";
    } catch (const std::exception& e) {rk_row_nav_destroy(h); std::cerr<<e.what()<<'\n'; return 1;}
}
