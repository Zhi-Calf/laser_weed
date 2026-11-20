# 可选调试结果接口（SDK 0.2.2）

现有 create/process/result 结构及 ABI 版本不变。新接口需要链接0.2.2或更新的库；旧调用方可继续使用原接口。同一handle所有操作须串行，查询与复制之间不可处理下一帧。

```c
rk_row_nav_set_debug_enabled(handle, 1); /* 默认关闭；开关均清除旧结果 */
/* rk_row_nav_process / rk_row_nav_process_with_attitude */
rk_row_nav_debug_t debug;
rk_row_nav_debug_init(&debug);
if (rk_row_nav_get_debug(handle, &debug) == RK_ROW_NAV_OK && debug.available) {
    uint64_t bytes = 0;
    rk_row_nav_copy_debug_mask(handle, debug.frame_id, NULL, 0, &bytes);
    /* 调用方按 bytes 分配缓冲区，检查容量及返回码，再复制：
       rk_row_nav_copy_debug_mask(handle, debug.frame_id, buffer, capacity, &bytes);
       处理完成后由调用方释放 buffer。库不保留这个指针。 */
}
```

- mask是去畸变输入图坐标下的单通道uint8图，0/255，紧密排列，无行填充。不能直接叠加到未去畸变原图上。调试开启时保留一次mask副本，复制给调用方还有一次拷贝，可能增加处理时间和内存。
- instance_count是分割实例数量；total_rows是几何拟合行数，两者不等价。rows返回前32条，returned_rows为实际返回数量；total_rows更大表示截断。selected_left/right是完整行列表索引，可能超出返回数组，使用前必须检查。
- rows的id仅帧内有效，不是跨帧跟踪ID。每行有斜率、截距、RMS残差和内点数；坐标是车体x前/y左、y=slope*x+intercept。它们是单帧独立拟合，joint_left/right字段才是选中双行联合拟合结果。baseline模式可能没有逐行拟合列表。
- pair_valid表示几何配对成功，centerline_valid表示单帧中心线拟合成功；提供中心线RMS、点数与内点数。它们都不等于最终导航有效，不包含时序确认后的完整判定。
- available只表示本次成功调用产生了调试数据，不表示可控制。observation_status始终对应最终结果，处理后超时仍可保留带STALE状态的调试数据；处理前已过期、处理失败和未开启时available=0。错误后查询不回退旧帧。
- mask复制先核对frame_id；不足容量返回INVALID_ARGUMENT且不写像素，required告知所需字节数。无数据时required=0。每次process开始清空旧快照，即使参数检查失败也一样。初始化、关闭或重开调试也不会返回旧帧。

离线evaluate_replay.py增加 `--debug-masks`，保存debug-masks/以及frames.jsonl中的逐行信息；这仍要求有效静态相机标定。公开数据无标定时继续使用segmentation_replay，仅导出分割，不伪造地面/车体坐标。

## 验证

合约测试新增：默认关闭、开关取值、mask尺寸/帧号、已知像素、容量不足不写、错误帧号、扩展结构尾部保留、选中双行/拟合内点、错误后失效、关闭清空，以及后置STALE标记。Python回放测试验证实际ctypes布局、mask导出和逐行结果。这些使用模拟神经网络输出与真实几何链路，不能替代真机精度测试。

2026-11-12 实测：MyUbuntu sdk-replay-20261112 隔离构建五组CTest全部通过；安装后的 evaluate_replay.py 开启 --debug-masks，通过同一图片/视频回归。AWU202 ARM64、RKNN=ON 交叉编译通过，nm确认四个新符号导出；未在板端执行，也未验证调试开启后的实际NPU性能影响。现有SDK导航结构未增加字段，版本更新为0.2.2。
