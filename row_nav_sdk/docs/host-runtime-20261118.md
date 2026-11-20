# 主机ONNX运行时与NMS修复（SDK 0.2.3）

## 结论

在MyUbuntu独立目录构建OpenCV4.8.1，同一SDK重编译后原始张量与Python4.8.1参考接近；同时修复NMS上限的语义错误。430张同一派生测试集的micro IoU由20.36%提高到34.40%，底部1/3像素召回由20.78%提高到54.19%。仍不是实际稻田/车辆精度验收。

系统OpenCV4.5.4不替换，未执行sudo/ldconfig。隔离库位于 `/home/zhouboyang/workspace/opencv-parity-20261118/runtime/install`，SDK位于同目录sdk-install。通过ldd确认SDK加载独立路径的.so.408，不依赖修改全局LD_LIBRARY_PATH。正常车辆ROS进程没有切换到新库。

## 修改

1. ONNX后端规范化13输出顺序：按空间尺寸/通道匹配boxes、coefficients、两个单类score/sum及proto；score与sum必须在1e-5内一致。10输出仍由原解码器按唯一形状识别。RKNN按原声明顺序处理，不套用ONNX规范化。
2. OpenCV NMSBoxes的top_k限制的是**抑制前**按分数排序的候选，并不是最终max_det。旧代码传16，使重复候选挤掉较弱的其他行；现在先完整NMS，再截取最终16个。此修复作用于共用后处理，包括RKNN，需板端重新验证精度和耗时，不沿用旧版板端结果作为新版本验收。
3. ONNX加载时对低于4.8的OpenCV明确返回不可用，防止继续产生已观察到的错误主机基线。这是项目支持下限策略，不是断言所有旧版/模型都错误，也不保证所有4.8+版本自动正确。RKNN不受此加载检查限制，目标板仍可用4.5.4提供基础图像处理。
4. 新增scripts/build/isolated_opencv.sh：从已有源码构建最少CPU模块并装到指定独立前缀，不写系统目录。

公开C ABI/结构保持不变，版本更新0.2.3。调试/诊断接口不变，未增加异步流水线。

## 复现构建

实际源码包为PyPI opencv-python4.8.1.78源码分发内的opencv目录，镜像下载并按PyPI元数据SHA256校验：`cc7adbbcd1112877a39274106cb2752e04984bc01a031162952e97450d6117f6`。源码版本4.8.1，CPU共享库，关闭IPP/OpenCL/CUDA，避免额外下载。镜像下载替代了失败的GitHub直连，未改网络设置。

```bash
bash row_nav_sdk/scripts/build/isolated_opencv.sh /absolute/opencv-source /absolute/runtime 8
cmake -S row_nav_sdk -B /absolute/sdk-build \
  -DOpenCV_DIR=/absolute/runtime/install/lib/cmake/opencv4 \
  -DCMAKE_BUILD_TYPE=Release -DROW_NAV_BUILD_TOOLS=ON \
  -DROW_NAV_OPENCV_RUNTIME_DIR=/absolute/runtime/install/lib
cmake --build /absolute/sdk-build -j8
cmake --install /absolute/sdk-build --prefix /absolute/sdk-install
```

隔离SDK/install带自身库路径；OpenCV独立前缀仍需保留。暂非可移动的完整依赖包，跨机器交付不能只复制librk_row_nav.so。不得将主机x86库放到ARM板端。

## 数值与性能

单张100.jpg、保存的相同float输入、相同优化ONNX：C++4.8.1对Python4.8.1的13个输出最大绝对差2.84e-5；旧C++4.5.4某DFL输出差约6.03。此逐张量证据只覆盖该图，不是全量逐张量误差上界。

同一前50张对PyTorch参考的平均mask相互IoU：

| 运行环境/后处理 | 相互IoU |
|---|---:|
| 4.8.1 fast，旧NMS上限 | 51.50% |
| 4.8.1 full，旧NMS上限 | 55.75% |
| 4.8.1 full，修正NMS | 90.84% |
| 4.8.1 fast，修正NMS（当前SDK路径） | 82.07% |

剩余差异与重建分辨率、裁剪和阈值顺序等路径差别有关，尚未完成逐项归因。不声称与PyTorch像素完全一致，不擅自把默认fast改为full。

同430张输入、同标签、0.25置信度、NMS0.7、最多16个实例：

| 指标 | 历史4.5.4+旧NMS | 4.8.1+旧NMS | 4.8.1+修正NMS |
|---|---:|---:|---:|
| micro IoU | 20.36% | 23.61% | 34.40% |
| 像素recall | 24.62% | 30.07% | 54.32% |
| 近端recall | 20.78% | 29.82% | 54.19% |
| 覆盖不到50%的标签行带 | 1268/1541 | 1116/1541 | 685/1541 |

最终CPU infer均值53.45ms，P95 57.78ms；含冷帧和解码，不含图片IO，非板端/端到端速度。多轮统计未做，环境并发负载不锁定。430张均完成、无空mask仍不等于达标；标签为派生行带，未量化真实地面误差。

[张量对照](host-runtime-20261118/runtime-comparison.json)、[mask相互对照](host-runtime-20261118/mask-parity.json)、[430张指标](host-runtime-20261118/metrics-summary.json)、[运行指纹](host-runtime-20261118/manifest.json)、[耗时](host-runtime-20261118/timing-summary.json)。指标记录的binary hash是相同算法的预版本更新构建，随后仅更新SDK版本号和旧OpenCV拒绝检查并重建测试；不把最终二进制hash冒充该次回放hash。

## 验证与后续

新版环境六组CTest通过，包含新增输出重排、重复score不一致、NMS重复候选挤占检测名额回归测试。安装后的纯C consumer可构建并输出0.2.3。ARM64/RKNN开启交叉编译通过，未执行实机。后续优先固定参考后处理模式，评估剩余mask差异，再用实际稻田样本和板端对照决定训练投入。
