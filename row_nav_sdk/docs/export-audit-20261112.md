# 原始权重与ONNX一致性核查（2026-11-12）

结论：本次50张核查中，PyTorch与标准ONNX几乎一致；RKNN适配版ONNX的13个原始输出也与PyTorch对应导出头接近。当前SDK主机OpenCV4.5.4构建的原始输出却与4.8.1明显不同，差异在mask解码前已经存在。因此，之前SDK低IoU不能直接归因于训练不足或标签问题；下一步应先修正/替换并验证主机ONNX推理运行时，再重跑全量评测。本次没有修改SDK正式推理行为或系统OpenCV。

## 配对条件

从之前430张推理manifest取相同前50张，逐张核对输入hash。使用同次训练best.pt、标准2输出ONNX和RKNN适配13输出ONNX。文件hash保存在[manifest](export-audit-20261112/manifest.json)。reference依赖项目本地ultralytics_yolo11导出适配代码，非在线下载的新版本。

CPU float32、RGB/255、640输入、置信度0.25、NMS IoU0.7、最多16个实例；本次图片均为方形，消除padding歧义。mask采用同一Ultralytics native重建流程。PyTorch2.1.0+cu121（运行在CPU）、Python OpenCV4.8.1。脚本按ONNX graph声明的输出名字取张量，避免不同版本返回顺序变化导致误比。没有安装onnxruntime，也没有做板端RKNN/INT8对照。

## 结果

| 配对指标（50张） | 数值 |
|---|---:|
| PyTorch与标准ONNX平均mask相互IoU | 99.99989% |
| 标准ONNX decoded张量最大绝对差 | 0.003540（框坐标含输入像素单位） |
| 标准ONNX proto最大绝对差 | 0.00003815 |
| RKNN适配ONNX对PyTorch导出头原始张量最大绝对差 | 0.00007677 |
| PyTorch与当前SDK平均mask相互IoU | 42.55% |

相互IoU只衡量两个输出一致性，不是对标注的精度；双方mask为空时一致性记1。

同50张、同派生标签的像素micro指标：

| 路径 | IoU | recall | 图像底部1/3 recall |
|---|---:|---:|---:|
| PyTorch参考 | 37.21% | 55.74% | 54.17% |
| 标准ONNX / OpenCV4.8.1参考 | 37.21% | 55.74% | 54.17% |
| 现有SDK / OpenCV4.5.4 | 23.18% | 28.29% | 24.39% |

50张SDK mask与此前430张运行的前50张逐张hash相同；不是把不同样本分数直接相减。37.21%仍不代表识别够用，标签是派生行带、而非人工植被像素真值。旧430张20.36%是更大样本的历史SDK结果，不应拿它直接与50张参考37.21%比较。

## 运行时定位证据

使用100.jpg，将C++ SDK同样的letterbox/RGB/255输入及每个输出写为二进制。再把**保存的同一份float32输入**交给Python OpenCV4.8.1执行相同适配ONNX，按节点名比较。输入与Python重新处理图像的最大差仅5.96e-8。proto输出491最大差约2.0024，DFL输出544最大差约6.0317，分类输出587最大差约0.3384。说明存在明显的主机DNN前向差异，而不是仅mask阈值/重建差异。

这项逐张量运行时对照目前只做100.jpg；不能声称已确认所有图片、某个具体算子或所有OpenCV4.5版本都有同一问题。4.5.4优化器保留10个唯一输出，4.8.1暴露13个，重复的单类score/sum在旧版被合并；本次按共同节点名比较10个输出，未拿不同位置硬对齐。[数值记录](export-audit-20261112/runtime-comparison.json)。

## 工具与复现

- compare_exports.py：需要已有torch、Ultralytics、onnx、opencv环境；原始权重与两种导出逐图比较，输出参考mask和原始张量误差。不安装依赖、不训练、不改权重。
- onnx_tensor_dump：`ROW_NAV_BUILD_TOOLS=ON`构建。参数为模型、图片、新输出目录，保存input.bin、tensors.tsv、逐张量.bin和OpenCV版本。仅限诊断，文件较大，不入Git。
- compare_cpp_dump.py：参数--model、--dump、--output，用当前Python OpenCV对完全相同输入执行比较。
- evaluate_masks.py：对两份参考mask和SDK mask使用同一派生标签计算指标。

```bash
PYTHONPATH=/absolute/ultralytics_yolo11 YOLO_CONFIG_DIR=/absolute/isolated-config \
  python compare_exports.py --weights /absolute/best.pt \
  --standard /absolute/standard.onnx --optimized /absolute/rknnopt.onnx \
  --run /absolute/real-seg-430-v1 --output /absolute/new-export-audit --limit 50
```

完整输出在MyUbuntu `/home/zhouboyang/workspace/sdk-replay-20261112/export-audit-50-v2`、`raw-cpp-100-v3`。仓库保留摘要和指纹，不存张量大文件。六组CTest继续通过；一致性检查另有实际50张执行证据，不将普通合约测试当成模型数值验证。

后续顺序：隔离构建一个数值与参考一致的主机ONNX运行环境，确认输出节点排序契约，重跑同样430张；然后再判断标签/训练的剩余问题。板端RKNN必须另行核查，不能据此推断也有问题。

最终工具显式设置OpenCV backend/CPU target后，100.jpg原始张量对照再次复现；最终Python工具另以2张完成烟测，参考mask相互IoU=1。50张统计与2张烟测分别保留，未混合样本。
