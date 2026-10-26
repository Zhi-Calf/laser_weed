#pragma once
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "rk_nav/geometry/dynamic_bev.hpp"

namespace dynamic_fixture {
inline cv::Matx33d orientation(double roll,double pitch) {
    const cv::Matx33d rx(1,0,0,0,std::cos(roll),-std::sin(roll),0,std::sin(roll),std::cos(roll));
    const cv::Matx33d ry(std::cos(pitch),0,std::sin(pitch),0,1,0,-std::sin(pitch),0,std::cos(pitch));
    return ry*rx;
}
inline cv::Matx33d intrinsics() { return {400,0,320,0,400,240,0,0,1}; }
inline rk_nav::DynamicBevConfig config() {
    rk_nav::DynamicBevConfig c;
    c.enabled=true;
    c.camera_to_vehicle_rotation=orientation(0,0.45)*cv::Matx33d(0,0,1,-1,0,0,0,-1,0);
    c.camera_position_vehicle_m={0.4,0.1,0.7};
    c.origin_height_m=0.5;
    c.reference_roll_rad=0.02; c.reference_pitch_rad=-0.01;
    return c;
}
// Independent 3D pinhole projection of a known point in vehicle axes.
inline cv::Point2d imagePoint(const cv::Vec3d& body,const rk_nav::DynamicBevConfig& c) {
    const auto p=intrinsics()*(c.camera_to_vehicle_rotation.t()*(body-c.camera_position_vehicle_m));
    return {p[0]/p[2],p[1]/p[2]};
}
inline rk_nav::BevConfig measuredBev(const rk_nav::DynamicBevConfig& c) {
    rk_nav::BevConfig b;
    b.output_size={400,600}; b.meters_per_pixel=0.01;
    b.vehicle_origin_u=200; b.vehicle_origin_v=599;
    const auto world_body=orientation(c.reference_roll_rad,c.reference_pitch_rad);
    std::vector<cv::Point2d> image,bev;
    for (double x : {1.0,5.0}) for (double y : {-1.0,1.0}) {
        const auto p=world_body.t()*(cv::Vec3d(x,y,0)-cv::Vec3d(0,0,c.origin_height_m));
        image.push_back(imagePoint(p,c));
        bev.emplace_back(b.vehicle_origin_u-p[1]/b.meters_per_pixel,
                         b.vehicle_origin_v-p[0]/b.meters_per_pixel);
    }
    const auto h=cv::findHomography(image,bev);
    for (int r=0;r<3;++r) for (int col=0;col<3;++col) b.image_to_bev(r,col)=h.at<double>(r,col);
    return b;
}
inline cv::Mat rowImage(const rk_nav::DynamicBevConfig& c,double roll,double pitch) {
    cv::Mat image=cv::Mat::zeros(480,640,CV_8UC3);
    const auto rotation=orientation(roll,pitch);
    for (double y : {-0.4,0.6}) {
        std::vector<cv::Point> points;
        for (int i=0;i<=120;++i) {
            const double x=0.8+i*.05;
            const double z=(-c.origin_height_m-rotation(2,0)*x-rotation(2,1)*y)/rotation(2,2);
            const auto pixel=imagePoint({x,y,z},c);
            points.emplace_back(cvRound(pixel.x),cvRound(pixel.y));
        }
        cv::polylines(image,points,false,cv::Scalar(255,0,0),3);
    }
    return image;
}
inline void writeBev(const std::string& path,const rk_nav::DynamicBevConfig& c,bool accepted=true,bool enabled=true) {
    const auto b=measuredBev(c);
    cv::FileStorage s(path,cv::FileStorage::WRITE);
    s<<"schema_version"<<1<<"calibration_valid"<<1<<"output_width"<<b.output_size.width
     <<"output_height"<<b.output_size.height<<"meters_per_pixel"<<b.meters_per_pixel
     <<"vehicle_origin_u"<<b.vehicle_origin_u<<"vehicle_origin_v"<<b.vehicle_origin_v
     <<"image_to_bev"<<cv::Mat(b.image_to_bev);
    // This forward camera sees ground from ~1.5m, not the legacy 0.4m ROI.
    // Keep the production selector's near-support gate; configure the test rig ROI.
    s<<"row_extractor"<<"{"<<"min_forward_m"<<1.5<<"max_forward_m"<<5.5<<"}";
    s<<"dynamic_bev"<<"{"<<"enabled"<<(enabled?1:0)<<"calibration_valid"<<(accepted?1:0)
     <<"camera_to_vehicle_rotation"<<cv::Mat(c.camera_to_vehicle_rotation)
     <<"camera_position_vehicle_m"<<cv::Mat(c.camera_position_vehicle_m)
     <<"origin_height_m"<<c.origin_height_m<<"reference_roll_rad"<<c.reference_roll_rad
     <<"reference_pitch_rad"<<c.reference_pitch_rad<<"}";
}
} // namespace dynamic_fixture
