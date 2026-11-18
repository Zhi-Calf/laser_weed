#include "test_framework.hpp"
#include <opencv2/core.hpp>
#include "rk_nav/inference/yolov8_seg_postprocess.hpp"
#include <limits>
#include <algorithm>
namespace {
std::vector<cv::Mat> tensors(bool ordered) {
    std::vector<cv::Mat> out;
    for (int n : {80,40,20}) {
        for (int c : {64,1,32}) {
            int shape[]={1,c,n,n};
            out.emplace_back(4,shape,CV_32F,cv::Scalar(0));
            if (ordered && c==1) out.push_back(out.back().clone());
        }
    }
    int shape[]={1,32,160,160};
    out.emplace_back(4,shape,CV_32F,cv::Scalar(1));
    return out;
}
}
TEST_CASE("YOLO decoder validates both layouts and empty detections") {
    rk_nav::ModelConfig config;
    rk_nav::Frame frame{cv::Mat::zeros(240,320,CV_8UC3),1,1};
    const auto transform=rk_nav::letterboxImage(frame.image,640,640);
    for (bool ordered : {false,true}) {
        auto out=tensors(ordered);
        REQUIRE(rk_nav::validateRockchipYoloV8SegLayout(out,config).isOk());
        auto result=rk_nav::decodeRockchipYoloV8Seg(out,frame,config,transform,0);
        REQUIRE(result.isOk()); REQUIRE(result.value().instances.empty());
        REQUIRE(cv::countNonZero(result.value().combined_mask)==0);
        out[1].ptr<float>()[40*80+40]=.9F;
        out[ordered?3:2].setTo(cv::Scalar(1));
        config.materialize_instance_masks=true;
        auto full=rk_nav::decodeRockchipYoloV8Seg(out,frame,config,transform,0);
        config.materialize_instance_masks=false;
        auto fast=rk_nav::decodeRockchipYoloV8Seg(out,frame,config,transform,0);
        REQUIRE(full.isOk()); REQUIRE(fast.isOk());
        REQUIRE(full.value().instances.size()==1); REQUIRE(fast.value().instances.size()==1);
        cv::Mat intersection,union_mask;
        cv::bitwise_and(full.value().combined_mask,fast.value().combined_mask,intersection);
        cv::bitwise_or(full.value().combined_mask,fast.value().combined_mask,union_mask);
        REQUIRE(cv::countNonZero(union_mask)>0);
        REQUIRE(double(cv::countNonZero(intersection))/cv::countNonZero(union_mask)>.85);
        out[1].ptr<float>()[0]=std::numeric_limits<float>::quiet_NaN();
        REQUIRE(!rk_nav::decodeRockchipYoloV8Seg(out,frame,config,transform,0).isOk());
    }
}
TEST_CASE("YOLO decoder rejects incompatible model contracts") {
    rk_nav::ModelConfig c;
    auto out=tensors(false);
    c.input_width=320; REQUIRE(!rk_nav::validateRockchipYoloV8SegLayout(out,c).isOk());
    c.input_width=640; out[0]=out[1]; REQUIRE(!rk_nav::validateRockchipYoloV8SegLayout(out,c).isOk());
    out=tensors(true); std::swap(out[1],out[3]); REQUIRE(!rk_nav::validateRockchipYoloV8SegLayout(out,c).isOk());
    out=tensors(false); c.mask_threshold=std::numeric_limits<float>::infinity();
    REQUIRE(!rk_nav::validateRockchipYoloV8SegLayout(out,c).isOk());
}
#include "rk_nav/inference/tensor_conversion.hpp"
TEST_CASE("INT8 UINT8 and float conversion preserve layout and reject short buffers") {
    using namespace rk_nav::detail;
    const std::int8_t values[]={-3,1,5,9};
    auto result=tensorToNchw(values,sizeof(values),{1,1,2,2},true,TensorStorage::Int8,1,.5F);
    REQUIRE(result.isOk());
    const auto* p=result.value().ptr<float>();
    REQUIRE(p[0]==-2 && p[1]==2 && p[2]==0 && p[3]==4);
    const std::uint8_t unsigned_values[]={0,128,255,129};
    result=tensorToNchw(unsigned_values,sizeof(unsigned_values),{1,2,1,2},false,TensorStorage::UInt8,128,.25F);
    REQUIRE(result.isOk()); REQUIRE(result.value().ptr<float>()[0]==-32);
    REQUIRE(result.value().ptr<float>()[2]==31.75F);
    const float floats[]={1,2,3,4};
    result=tensorToNchw(floats,sizeof(floats),{1,2,1,2},false,TensorStorage::Float32,0,1);
    REQUIRE(result.isOk()); REQUIRE(result.value().ptr<float>()[3]==4);
    REQUIRE(!tensorToNchw(values,1,{1,1,2,2},true,TensorStorage::Int8,1,.5F).isOk());
    REQUIRE(!tensorToNchw(values,4,{1,1,2,2},true,TensorStorage::Int8,1,0).isOk());
}
#include "rk_nav/core/scope_exit.hpp"
TEST_CASE("RKNN output cleanup guard releases once on normal return and exception") {
    int releases=0;
    { rk_nav::detail::ScopeExit lease([&]() noexcept { ++releases; }); }
    REQUIRE(releases==1);
    try {
        rk_nav::detail::ScopeExit lease([&]() noexcept { ++releases; });
        throw std::bad_alloc();
    } catch (const std::bad_alloc&) {}
    REQUIRE(releases==2);
}

TEST_CASE("ONNX outputs normalize reordered duplicate score branches") {
    auto out=tensors(true);
    std::reverse(out.begin(),out.end());
    auto normalized=rk_nav::normalizeOnnxYoloSegOutputs(out);
    REQUIRE(normalized.isOk());
    REQUIRE(rk_nav::validateRockchipYoloV8SegLayout(normalized.value(),{}).isOk());
    out=tensors(true); out[2].setTo(cv::Scalar(.8));
    REQUIRE(!rk_nav::normalizeOnnxYoloSegOutputs(out).isOk());
    out=tensors(true); out[0]=out[3];
    REQUIRE(!rk_nav::normalizeOnnxYoloSegOutputs(out).isOk());
}
TEST_CASE("NMS detection limit applies after suppression of duplicate candidates") {
    auto out=tensors(true);
    // Three neighboring high-score duplicate boxes, then a separate weaker row.
    auto* score=out[1].ptr<float>();
    score[40*80+39]=.95F;score[40*80+40]=.94F;score[40*80+41]=.93F;
    score[10*80+10]=.8F;
    out[2]=out[1].clone();
    rk_nav::ModelConfig c;c.max_detections=2;
    rk_nav::Frame f{cv::Mat::zeros(640,640,CV_8UC3),1,1};
    auto result=rk_nav::decodeRockchipYoloV8Seg(out,f,c,rk_nav::letterboxImage(f.image,640,640),0);
    REQUIRE(result.isOk()); REQUIRE(result.value().instances.size()==2);
    REQUIRE(result.value().instances[1].confidence==.8F);
}
