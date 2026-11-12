# 离线回放与评测

工具直接通过公开 C ABI 调用同一份 SDK，不另写推理或选行算法。不依赖 ROS。需要 Python、NumPy、opencv-python；库的 OpenCV/RKNN 依赖仍由系统安装。SDK 至少需要含诊断接口的 0.2.1。

```bash
python3 row_nav_sdk/scripts/evaluate_replay.py \
  --library /absolute/install/lib/librk_row_nav.so \
  --model /absolute/model.onnx --backend onnx \
  --intrinsics /absolute/intrinsics.yaml --bev /absolute/bev.yaml \
  --input /absolute/images_or_video --output /absolute/new-run-directory \
  --fps 15 --limit 300 --warmup 10 --overlay-every 10
```

安装后脚本位于 tools/，与 summarize_diagnostics.py 放在同一目录。输出目录必须不存在，不混入历史结果。图片目录仅读取第一层并按文件名字典序排序，建议补零命名；不相关的图片应增加 `--independent-images`，每张重建 handle 并关闭跳变统计。该模式初始化时间不包含在 SDK 帧耗时内。视频逐帧解码；视频中途解码失败可能被解码器当作结束，需核对实际帧数与预期。

- frames.jsonl：包括预热的逐帧输入身份、返回码、状态、导航结果和阶段计时；不可用数值用 null，异常不沿用上一帧结果。
- timing.csv：去除预热的计时，可再次运行 summarize_diagnostics.py。available=0 样本不纳入耗时统计。
- summary.json：有效帧率（比例）、状态计数、连续有效相邻帧的跳变次数及各阶段均值/P50/P95/最大值。跳变阈值默认 0.15m、5°，可通过 --jump-m/--jump-deg 修改。无效帧会打断相邻关系，预热和独立图片不参加时序跳变统计。状态计数的负数是 SDK 错误码，非负数为 observation_status。
- overlays/：原图上标注导航数值和状态。当前 C API 不提供 mask 或像素线，叠加图仅为结果注释，不画伪造分割轮廓。
- manifest.json：模型、库、内外参及输入指纹、SDK 版本、运行选项和完成标记。目录图片指纹逐张记录在 frames.jsonl；视频记录整个文件指纹。completed=false 为未完成，不能作为完整评测结果。

`--fps` 控制回放提交间隔，推理慢时不会丢帧赶进度。时间戳由 SDK 单调时钟在提交时生成，不是原始曝光时间；时序跟踪使用实际回放时刻，跨设备/负载会有差别。总耗时是 SDK 调用耗时，不含解码、文件哈希、叠加图写盘和主动等待；不能用 1000/P95 宣称系统帧率。--max-age-ms 默认250，超时帧会保持 STALE；不要为提高有效率而隐藏修改此阈值。

本版只接受已通过 SDK 检查的静态 BEV 参数。动态 BEV 需要原始曝光与姿态的配对记录，工具目前明确拒绝，不模拟真实同步。没有地面真值时，有效率与跳变不等于横向/航向精度。公开数据缺少当前相机标定时，不能用任意 BEV 参数宣称车辆坐标准确。

## 2026-11-12 验证

MyUbuntu 隔离目录 `/home/zhouboyang/workspace/sdk-replay-20261112`。集成测试使用仅测试构建的 `librk_replay_test_sdk.so`，通过真实 C ABI、去畸变及几何链路，替换神经网络输出；该库不安装发布。覆盖图片/视频、预热剔除、已知0.1m偏移、无行无效值、诊断统计、覆盖写入拒绝、坏图片未完成记录。不能将这些结果当作真实模型精度或 NPU 性能。

可在 CMake 配置 `ROW_NAV_REPLAY_TEST_PYTHON` 为具有 cv2/numpy 的解释器，将 sdk_replay 加入 CTest。该选项仅用于主机且 RKNN=OFF 的测试构建。

本次五组 CTest 全部通过，安装到独立前缀后的脚本也通过同一图片/视频集成测试；人工查看了结果注释图。报告与测试图片保存在上述隔离目录的 test-artifacts/，属于合成软件测试数据。
