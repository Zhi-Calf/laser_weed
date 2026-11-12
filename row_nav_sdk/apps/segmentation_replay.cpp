// Segmentation-only inspection: no fabricated camera/BEV calibration.
#include <algorithm>
#include <chrono>
#include <cctype>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "rk_nav/inference/onnx_yolov8_segmenter.hpp"
#ifdef RK_NAV_WITH_RKNN
#include "rk_nav/inference/rknn_yolov8_segmenter.hpp"
#endif
int main(int argc,char** argv) {
    if (argc!=6) { std::cerr<<"usage: segmentation_replay onnx|rknn model image-directory new-output-directory limit\n";return 1; }
    try {
        const std::string backend=argv[1];
        std::unique_ptr<rk_nav::Segmenter> segmenter;
        if (backend=="onnx") segmenter=std::make_unique<rk_nav::OnnxYoloV8Segmenter>();
#ifdef RK_NAV_WITH_RKNN
        else if (backend=="rknn") segmenter=std::make_unique<rk_nav::RknnYoloV8Segmenter>();
#endif
        else throw std::runtime_error("unsupported backend in this build");
        std::size_t consumed=0; const int limit=std::stoi(argv[5],&consumed);
        if (limit<1 || consumed!=std::string(argv[5]).size()) throw std::runtime_error("invalid limit");
        const std::filesystem::path input=std::filesystem::canonical(argv[3]);
        const auto output=std::filesystem::absolute(argv[4]).lexically_normal();
        if (std::filesystem::exists(output)) throw std::runtime_error("output already exists");
        std::vector<std::filesystem::path> images;
        for (const auto& e : std::filesystem::directory_iterator(input)) {
            auto suffix=e.path().extension().string();
            std::transform(suffix.begin(),suffix.end(),suffix.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
            if (e.is_regular_file() && (suffix==".jpg" || suffix==".jpeg" || suffix==".png")) images.push_back(e.path());
        }
        std::sort(images.begin(),images.end());
        if (images.empty()) throw std::runtime_error("no images");
        rk_nav::ModelConfig config;config.path=argv[2];config.materialize_instance_masks=false;config.max_detections=16;
        const auto loaded=segmenter->load(config);
        if (!loaded.isOk()) throw std::runtime_error(loaded.message());
        std::filesystem::create_directories(output/"masks");std::filesystem::create_directories(output/"overlays");
        std::ofstream csv(output/"frames.csv"); csv<<"index,instances,mask_fraction,inference_ms\n";
        if (!csv) throw std::runtime_error("cannot write report");
        int failures=0,index=0;
        for (const auto& path : images) {
            if (index>=limit) break;
            cv::Mat image=cv::imread(path.string());
            if (image.empty()) throw std::runtime_error("cannot decode: "+path.string());
            const auto started=std::chrono::steady_clock::now();
            auto result=segmenter->infer({image,0,static_cast<std::uint64_t>(index+1)});
            if (!result.isOk()) { ++failures;std::cerr<<path.filename()<<": "<<result.status().message()<<'\n';++index;continue; }
            const double total_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
            const auto& seg=result.value();
            cv::Mat tint=image.clone(),overlay;
            tint.setTo(cv::Scalar(0,255,0),seg.combined_mask);
            cv::addWeighted(image,.6,tint,.4,0,overlay);
            const auto name=std::to_string(index)+".png";
            if (!cv::imwrite((output/"masks"/name).string(),seg.combined_mask) ||
                !cv::imwrite((output/"overlays"/name).string(),overlay)) throw std::runtime_error("image write failed");
            csv<<index<<','<<seg.instances.size()<<','<<double(cv::countNonZero(seg.combined_mask))/double(seg.combined_mask.total())<<','<<total_ms<<'\n';
            std::ofstream identity(output/(std::to_string(index)+".source.txt"));identity<<path.string()<<'\n';
            ++index;
        }
        std::cout<<"frames="<<index<<" failures="<<failures<<"\n";
        return failures?2:0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
