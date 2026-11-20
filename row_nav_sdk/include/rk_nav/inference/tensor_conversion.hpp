#pragma once
#include <array>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <limits>
#include <opencv2/core.hpp>
#include "rk_nav/core/result.hpp"
namespace rk_nav::detail {
enum class TensorStorage { Float32, Int8, UInt8 };
// Dimensions are in source layout; result is an owned continuous NCHW float tensor.
inline Result<cv::Mat> tensorToNchw(const void* data, std::size_t bytes,
    std::array<int,4> dims, bool nhwc, TensorStorage storage, int zero_point, float scale) {
    std::size_t count=1;
    for (int d : dims) {
        if (d<=0 || d>8192 || count>std::numeric_limits<std::size_t>::max()/static_cast<std::size_t>(d))
            return Status::invalidArgument("invalid tensor dimensions");
        count*=static_cast<std::size_t>(d);
    }
    if ((storage==TensorStorage::Int8 && (zero_point < -128 || zero_point > 127)) ||
        (storage==TensorStorage::UInt8 && (zero_point < 0 || zero_point > 255)))
        return Status::invalidArgument("invalid tensor zero point");
    const auto element_size=storage==TensorStorage::Float32 ? sizeof(float) : 1u;
    if (!data || count>bytes/element_size || dims[0]!=1 ||
        (storage!=TensorStorage::Float32 && (!std::isfinite(scale) || scale<=0)))
        return Status::invalidArgument("invalid tensor buffer or quantization");
    auto shape=dims;
    if (nhwc) shape={dims[0],dims[3],dims[1],dims[2]};
    cv::Mat out(4,shape.data(),CV_32F);
    auto* dst=out.ptr<float>();
    if (!nhwc && storage==TensorStorage::Float32) {
        std::memcpy(dst,data,count*sizeof(float));
        return out;
    }
    const auto read=[&](std::size_t i) {
        if (storage==TensorStorage::Float32) return static_cast<const float*>(data)[i];
        const int value=storage==TensorStorage::Int8 ? static_cast<const std::int8_t*>(data)[i] :
            static_cast<const std::uint8_t*>(data)[i];
        return static_cast<float>(value-zero_point)*scale;
    };
    const auto h=static_cast<std::size_t>(shape[2]), w=static_cast<std::size_t>(shape[3]);
    const auto channels=static_cast<std::size_t>(shape[1]);
    for (std::size_t c=0;c<channels;++c) for (std::size_t y=0;y<h;++y) for (std::size_t x=0;x<w;++x)
        dst[(c*h+y)*w+x]=read(nhwc ? (y*w+x)*channels+c : (c*h+y)*w+x);
    return out;
}
}
