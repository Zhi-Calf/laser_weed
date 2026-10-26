#include "test_framework.hpp"
#include "dynamic_bev_fixture.hpp"
#include <limits>
#include <fstream>

TEST_CASE("Dynamic BEV matches independent 3D projection across combined roll pitch") {
    const auto c=dynamic_fixture::config(); const auto bev=dynamic_fixture::measuredBev(c);
    rk_nav::DynamicBevProjector dynamic(bev,dynamic_fixture::intrinsics(),c);
    REQUIRE(dynamic.validate().isOk());
    double static_squared=0,corrected_squared=0; int count=0;
    for (double roll : {-0.12,0.0,0.10}) for (double pitch : {-0.10,0.0,0.12}) {
        auto projector=dynamic.at(roll,pitch); REQUIRE(projector.isOk());
        const auto r=dynamic_fixture::orientation(roll,pitch);
        for (double x : {1.5,3.0,5.0}) for (double y : {-0.4,0.6}) {
            const auto body=r.t()*(cv::Vec3d(x,y,0)-cv::Vec3d(0,0,c.origin_height_m));
            const auto pixel=dynamic_fixture::imagePoint(body,c);
            const auto out=projector.value().imageToGround(pixel); REQUIRE(out.isOk());
            const cv::Point2d truth(body[0],body[1]);
            const double error=cv::norm(out.value()-truth);
            REQUIRE(error<1e-8); corrected_squared+=error*error;
            const auto fixed=rk_nav::BevProjector(bev).imageToGround(pixel); REQUIRE(fixed.isOk());
            const double fixed_error=cv::norm(fixed.value()-truth);
            static_squared+=fixed_error*fixed_error; ++count;
        }
    }
    REQUIRE(std::sqrt(static_squared/count)>0.1);
    REQUIRE(std::sqrt(corrected_squared/count)<1e-8);
}

TEST_CASE("Dynamic BEV rejects missing physical calibration and mismatched reference") {
    const auto c=dynamic_fixture::config(); const auto b=dynamic_fixture::measuredBev(c);
    auto bad=c; bad.camera_to_vehicle_rotation(0,0)+=.1;
    REQUIRE(!rk_nav::DynamicBevProjector(b,dynamic_fixture::intrinsics(),bad).validate().isOk());
    bad=c; bad.origin_height_m=std::numeric_limits<double>::quiet_NaN();
    REQUIRE(!rk_nav::DynamicBevProjector(b,dynamic_fixture::intrinsics(),bad).validate().isOk());
    bad=c; bad.camera_position_vehicle_m[2]+=.3;
    REQUIRE(!rk_nav::DynamicBevProjector(b,dynamic_fixture::intrinsics(),bad).validate().isOk());
    bad=c; bad.reference_roll_rad=.2;
    REQUIRE(!rk_nav::DynamicBevProjector(b,dynamic_fixture::intrinsics(),bad).validate().isOk());
    rk_nav::DynamicBevProjector d(b,dynamic_fixture::intrinsics(),c);
    REQUIRE(!d.at(.8,0).isOk()); REQUIRE(!d.at(0,std::numeric_limits<double>::infinity()).isOk());
    REQUIRE(!d.at(std::numeric_limits<double>::quiet_NaN(),0).isOk());
}

TEST_CASE("Dynamic BEV suppresses ground behind camera and preserves forward ground") {
    auto c=dynamic_fixture::config();
    auto b=dynamic_fixture::measuredBev(c); b.output_size.height=1000; b.vehicle_origin_v=500;
    rk_nav::DynamicBevProjector d(b,dynamic_fixture::intrinsics(),c);
    auto p=d.at(0,0); REQUIRE(p.isOk());
    cv::Mat input(480,640,CV_8U,cv::Scalar(255));
    auto out=p.value().projectMask(input); REQUIRE(out.isOk());
    for (int v=0;v<out.value().rows;++v) for (int u=0;u<out.value().cols;++u) {
        const auto xy=p.value().bevToGround({double(u),double(v)});
        const auto camera=c.camera_to_vehicle_rotation.t()*(cv::Vec3d(xy.x,xy.y,-c.origin_height_m)-c.camera_position_vehicle_m);
        if (camera[2]<=.05) REQUIRE(out.value().at<uint8_t>(v,u)==0);
    }
    REQUIRE(cv::countNonZero(out.value())>100);
}

TEST_CASE("Dynamic BEV loader requires explicit acceptance and measured fields") {
    const auto path=std::filesystem::temp_directory_path()/"rk_dynamic_bev_test.yaml";
    const auto c=dynamic_fixture::config();
    dynamic_fixture::writeBev(path.string(),c,false);
    REQUIRE(!rk_nav::loadDynamicBevConfig(path).isOk());
    dynamic_fixture::writeBev(path.string(),c,true);
    auto loaded=rk_nav::loadDynamicBevConfig(path); REQUIRE(loaded.isOk());
    REQUIRE(rk_nav::DynamicBevProjector(dynamic_fixture::measuredBev(c),dynamic_fixture::intrinsics(),loaded.value()).validate().isOk());
    std::ofstream(path)<<"%YAML:1.0\ndynamic_bev:\n   enabled: 1\n   calibration_valid: 1\n";
    REQUIRE(!rk_nav::loadDynamicBevConfig(path).isOk());
    std::filesystem::remove(path);
}
