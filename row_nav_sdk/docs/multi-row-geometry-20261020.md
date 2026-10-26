# 多行关联与联合拟合实现和验证（2026-10-20）

## 实现范围

SDK继续接受调用方的图像，输出导航观测，公开C头文件和ABI保持不变。此次修改内部
单帧几何后处理；模型、模型权重、标定计算和车辆控制没有变化。没有实现跨帧跟踪。

| 模块 | 文件 | 职责 |
|---|---|---|
| RowCandidateExtractor | src/perception/row_candidate_extractor.cpp | BEV横带提取独立候选；过滤过宽粘连段 |
| MultiRowAssociator | src/perception/multi_row_associator.cpp | 最近局部方向预测、米制门限、一对一匹配、不交叉匹配；允许短缺口 |
| RowBoundaryFitter | src/perception/row_boundary_fitter.cpp | 每条关联行用已有RANSAC独立拟合；左右行使用四参数联合拟合、软方向约束和Huber IRLS |
| NavigationRowSelector | src/perception/multi_row_associator.cpp | 按车辆x=0处的行位置选择包围车辆的相邻两行，并检查共同覆盖、方向和行距 |
| RowExtractor | src/perception/row_extractor.cpp | 编排上述模块，保留旧逐横带最近邻基线，生成观测和调试数据 |

关联是确定性的贪心一对一关联，不是全局图优化。ID仅在一帧内有效。
联合模型保留左右两条不同斜率，软约束不会强制它们完全平行；行距用于容差筛选，
不会强行把所有稻行变成固定等间距。最终中心线仍是直线。

只在左右边界同时有有效观测的横带上生成中心点。剔除短碎片、远处才出现的孤立行、
方向不一致或行距不合法的组合，不推断缺失整侧边界，不自动跳过异常相邻行寻找外侧行。
候选不足时保留调试数据并返回NO_ROW，不产生可用导航观测。

## 选择算法和配置行距

在已实测、已接受的BEV YAML中增加可选配置（不是新增C接口参数）：

```yaml
row_extractor:
   method: associated
   expected_spacing_m: 0.4
   spacing_tolerance_m: 0.08
```

SDK、row_nav_image和row_nav_camera读取同一配置。没有该块时默认使用associated。
`method: nearest_band`可回退旧算法，旧算法不使用新增多行参数。

expected_spacing_m为0表示未知。实际0.3、0.4、0.8、1.2、1.4米地块分别配置其实际值；
混合或不规则行距不要强设某个统一值。参数在创建SDK时校验，错误值返回标定/配置错误。
默认允许行距0.25–1.5米，可配置min_row_spacing_m/max_row_spacing_m。

其他可选项及默认值：

- min_forward_m/max_forward_m：0.4/6.0米。
- association_gate_m：0.08米；maximum_gap_m：0.65米。
- minimum_span_m：1米；maximum_run_width_m：0.18米。
- residual_threshold_m：0.05米；maximum_heading_difference_rad：0.15弧度。
- parallel_soft_weight：0.1；spacing_tolerance_m：0.1米。

这些是开发默认值，必须按真实相机与标签宽度评估，不能把合成实验中的数值当作现场最优值。

## 调试输出

现有JSON增加row_method、pair_valid、selected_left/right、rejection_reason、joint_rms_m、
joint_boundaries、candidate_points、rejected_points、centerline_points及逐行rows数组。
所有点为地面坐标：x向前、y向左，单位米；选中索引指向rows数组，不是长期行ID。
joint_boundaries依次是左斜率、左截距、右斜率、右截距，仅pair_valid为true时可用。

BEV图中：紫色为其他拟合行，青色为选中的两侧行，绿色为导航中心线，红叉为剔除候选，
橙色为中心点，蓝点为车体原点。SDK自身不会在每次送图时写图像或JSON；调试工具按需调用。

## 固定基线与实验方法

复用当前100次RANSAC和最多5轮内点优化作为A基线，保留其逐横带最近邻选择方式。
B为新多行关联和联合拟合。两者使用完全相同的BEV mask、投影、中心线拟合器和导航
最低置信度0.35。无神经网络参与，避免模型差异影响几何测试。

固定场景生成器：tests/support/row_scenes.hpp；评估器：tools/row_geometry_eval.cpp。
22个场景包含每种行距的完整行、0.4米短缺苗、局部粘连、车辆偏心（共20个），
另加局部杂草和已知0.4米行距时整侧缺失。

预设有效场景要求VALID、横向误差<2厘米、航向误差<0.01弧度；预设无效场景要求不输出VALID。
这些是合成几何回归标准，不是已经和控制团队确认的真车验收标准。
已知行距场景的B使用显式行距先验，A不支持该功能，所以该项应解读为新增拒绝能力，
不能据此单独归因于联合拟合更准确。

## 本次结果

| 项目 | A基线 | B多行关联 |
|---|---:|---:|
| 全部22个合成场景判定正确 | 20/22 | 22/22 |
| 不使用行距先验的21个场景 | 20/21 | 21/21 |
| 窄行距偏心场景横向误差 | 约0.299米，错误返回VALID | <0.001米 |
| 整侧缺失且已知行距 | 误选外侧行，误差0.2米 | NO_ROW |
| 所有场景的几何耗时P95最大值 | 0.916毫秒 | 0.930毫秒 |

耗时来自MyUbuntu x86_64、GCC 11.4、OpenCV 4.5.4、Release，640×640 BEV，
每场景20次计时并按nearest-rank取P95；不含采集、去畸变、模型推理、JSON序列化和磁盘写入，
也不是RK3588耗时。精度接近零主要因为输入是理想直线合成mask，不代表真实数据。

原始CSV、44个JSON、BEV叠加图和输入mask保存在：
artifacts/multirow-20261020-v3/eval-v3/。
JSON已全部解析检查，偏心场景A/B叠加图已查看确认。
对照CSV同时归档在docs/evaluation-data/multirow-20261020.csv并纳入Git。

验证：Release全工程构建与4/4 CTest组通过，包含59个单元用例；ASan/UBSan下SDK两组
和全部单元测试通过；ARM64/RKNN全工程交叉编译成功。ARM64二进制未在本轮实际板端执行。

源码提交92d9800对应的x86_64和ARM64/RKNN SDK包均已生成，安装后独立C示例分别完成
主机运行和ARM64链接检查；ARM64安装库只导出9个原有C接口，RPATH为$ORIGIN，
依赖librknnrt.so，包元数据中tracked_changes_at_packaging为空。
两侧压缩包SHA256已验证，文件保存在artifacts/，文件名末尾带92d9800。

复现：

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DRK_NAV_BUILD_CALIBRATION_TOOLS=OFF
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
build/row_geometry_eval /tmp/row-geometry-new-output
```

评估输出目录必须不存在。该程序不会训练模型或触发车辆控制。

## 后续门槛

真实长杂草带、超过0.65米的缺口、密集粘连和坡面仍可能导致行关联错误或拒绝结果。
未知实际行距时，缺失内侧行与较宽行距可能不可区分，不能承诺识别整侧缺失。
固定BEV的地面假设、实际标签宽度和模型误检仍需真实数据验证。

下一阶段才是跨帧关联、运动补偿、失效与重捕获；当前帧内ID不能当成视频跟踪结果。
进入现场前需复核标定、板端动态依赖、NPU推理和连续视频导航误差。
