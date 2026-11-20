// Diagnostic only: expose actual C++ OpenCV output values for export parity audits.
#include <fstream>
#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include "rk_nav/inference/yolov8_seg_postprocess.hpp"
int main(int argc,char** argv) {
    if(argc!=4)return 1;
    try {
        const auto image=cv::imread(argv[2]);if(image.empty())throw std::runtime_error("image unreadable");
        auto net=cv::dnn::readNetFromONNX(argv[1]);
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        const auto transformed=rk_nav::letterboxImage(image,640,640);
        const auto blob=cv::dnn::blobFromImage(transformed.image,1./255.,cv::Size(640,640),cv::Scalar(),true,false,CV_32F);
        net.setInput(blob);const auto names=net.getUnconnectedOutLayersNames();std::vector<cv::Mat> outputs;net.forward(outputs,names);
        const std::filesystem::path out=argv[3];if(std::filesystem::exists(out))throw std::runtime_error("output exists");std::filesystem::create_directories(out);
        std::ofstream input_file(out/"input.bin",std::ios::binary);
        input_file.write(reinterpret_cast<const char*>(blob.data),static_cast<std::streamsize>(blob.total()*sizeof(float)));
        std::ofstream version(out/"opencv.txt");version<<CV_VERSION<<'\n';
        std::ofstream meta(out/"tensors.tsv");
        for(std::size_t i=0;i<outputs.size();++i) {
            const auto& tensor=outputs[i];if(tensor.type()!=CV_32F || !tensor.isContinuous())throw std::runtime_error("unsupported tensor");
            meta<<i<<'\t'<<names[i];for(int d=0;d<tensor.dims;++d)meta<<'\t'<<tensor.size[d];meta<<'\n';
            std::ofstream file(out/(std::to_string(i)+".bin"),std::ios::binary);file.write(reinterpret_cast<const char*>(tensor.data),static_cast<std::streamsize>(tensor.total()*sizeof(float)));
            if(!file)throw std::runtime_error("write failed");
        }
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
