# mask重建一致性修复（SDK0.2.4，2026-11-18）

## 决定

保留现有API和默认不保存实例mask的配置，但取消两套不同像素语义的重建。materialize_instance_masks现在只控制是否保存各实例mask；合并mask按同一原图分辨率浮点重建生成。旧CLI fast/full参数保留兼容，fast不再表示低分辨率二值化近似路径。

原因：省内存不应改变送入BEV/几何的行带。当前50张公开测试图与参考像素对齐显著改善，没有据此声称田间精度或NPU性能达标。

## 修复点

- NMS使用浮点框，避免先将框转整数再进行重叠抑制。
- 按原图比例裁去prototype padding，将logits直接双线性插值到原图尺寸，不先变成640二值图再用最近邻缩放。
- 在原图坐标中按浮点框进行裁剪：整数像素满足x>=x1且x<x2、y>=y1且y<y2，因此使用ceil边界。
- 最后做logit阈值，默认0.5等价logit>0。自定义阈值使用log(p/(1-p))，不忽略已有配置。
- 不保存实例mask时也保持同样合并结果；只省去各实例完整mask的长期分配/保存，仍需暂存每个实例浮点图。这可能增加原“fast”路径板端耗时，需实测。

参考来自本项目已使用Ultralytics源码ops.py的process_mask_native、scale_masks和crop_mask；50张参考输出与模型hash见之前export-audit-20261112，未更新权重或修改标注。

## 验证

同一模型、同一前50张，CPU OpenCV4.8.1，conf0.25/NMS0.7/max16：

| 指标 | 0.2.3默认合并路径 | 0.2.4合并路径 |
|---|---:|---:|
| 对PyTorch平均mask相互IoU | 82.0742% | 99.999892% |
| 最差图mask相互IoU | 78.8535% | 99.994605% |
| 扫描行中非空行段数量不一致 | 86/2595 | 0/2586 |
| 数量相同行段条件中心差P95 | 2px | 0px |

扫描从图高1/5到图底每8px取一行，比较水平连续前景段数量；双方空行不计入，故两次分母可不同。只有数量相同时按水平顺序匹配中心，上述P95是条件统计，不是拟合路径误差。它能揭示碎段/裁剪差异，但不能区分所有错行或代替导航精度。新增compare_mask_paths.py保留输入manifest校验和逐图结果。

同50张compact/full模式像素完全相同（最差与平均IoU=1）。非方形横/竖图与自定义阈值有合成单元测试，验证模式一致性；尚未做非方形实拍图的PyTorch逐像素对照。50张mask还残留极少接近零logit的数值边界差异，不声称bitwise等于PyTorch。

同430张派生标签全量重测：

| 指标 | 0.2.3 | 0.2.4 |
|---|---:|---:|
| micro像素IoU | 34.397% | 34.833% |
| 像素precision | 48.400% | 49.784% |
| 像素recall | 54.316% | 53.701% |
| 底部1/3recall | 54.188% | 53.499% |
| 标签行带覆盖<50% | 685/1541 | 714/1541 |

并非每个标注指标均提高，本次首先修复跨实现一致性。仍有标注/训练/域差距问题；不将派生标注像素分数解释为车辆导航准确率。

430张CPU infer均值54.13ms、P95 68.14ms，含冷启动、解码，不含读写；另一次50张full均值43.92ms，说明未锁定负载的单轮性能波动明显，不能据此说full比compact更快。未进行公平多轮速度排名或板端NPU评测。

六组CTest通过，新增非方形padding、两模式一致、自定义阈值测试；安装后纯C consumer输出0.2.4。ARM64 RKNN交叉编译通过。共用后处理变化同样作用于RKNN，但板端效果/耗时没有验证，不能沿用历史板端验收数字。

## 产物

MyUbuntu独立目录 `/home/zhouboyang/workspace/opencv-parity-20261118`：real-seg-430-native-final、metrics-430-native-final、real-seg-50-native-full-final，以及old/native-mask-paths.json、compact-full-native-parity.json。

仓库保留[参考对齐](mask-alignment-20261118/reference-parity.json)、[历史对齐](mask-alignment-20261118/previous-parity.json)、[模式一致性](mask-alignment-20261118/storage-mode-parity.json)、[全量指标](mask-alignment-20261118/metrics-summary.json)、[指纹](mask-alignment-20261118/manifest.json)及[耗时](mask-alignment-20261118/timing-summary.json)。

没有用公开图片伪造地面标定来计算米制横向/航向偏差。本次确认的是像素行带几何稳定性；车辆坐标导航对照仍需对应实测内外参和同一段视频。下一步可准备新版板端回归包，由现场同事执行。
