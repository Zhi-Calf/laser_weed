#pragma once

#include <cstdint>

#include "global_path_planner/bow.hpp"   // PlanStatus: 规划失败的细分状态 (本文件的映射用它)

namespace global_path_planner {

// ============================================================
// 全局路径规划统一故障码 (纯故障 ID, 无等级)
// ============================================================
//
// 编码规则: 5 位十进制, 前 3 位是模块号, 后 2 位是模块内故障序号。
//   104 = global_path_planner (全局路径规划节点)
//
// 故障码列表 (**连续编号, 无空号** —— 10400 起逐个递增; 新增时取下一个序号):
//   10400  无故障
//   10401  路径规划失败 (无可用路点, 下游永远等不到可跟踪的路径) —— **通用/兜底码**: 走完规划
//          流程仍无路点、且没有细分状态时用它 (例如还没收到带地块顶点的 set_source (local) 请求)。等 ENU 原点不再用它:
//          原点有自己的专码 10410 (见下)
//   10402  固定路径文件无效 (文件不存在/不可读, 不是合法 JSON, unit 不支持, crc32 不匹配,
//          路点不足 2 个, 或经纬度超出范围) —— file 模式**唯一**的文件码 (判定在 updateFilePathHealth;
//          失效那拍 loadPathFile 已把路径清空 (改成员), 同拍发出的空 /path 就是下游的停车信号)。
//          (旧 10402 "固定路径文件未就绪" 已于 2026-10 删除 —— 那条状态在调用图上**不可达**: 缺 ENU 原点时
//           timerCallback ① 先 return, 连文件都不读, 所以"文件读到了、只是没原点"到不了; 见第三次重排记录。)
//   10403  路径规划失败: 输入边界点不足 (< 3 个, kInputTooFew)
//   10404  路径规划失败: 去重后边界点不足 (< 3 个, kDedupTooFew)
//   10405  路径规划失败: 凸包点不足 (顶点退化/共线, kHullTooFew)
//   10406  路径规划失败: 地块非凸 (kNonConvex)
//   10407  路径规划失败: field_margin 不合理 (四周统一内缩把可作业区域吃没 / 内缩后区域退化,
//          kBadFieldMargin)
//   10408  路径规划失败: ridge_width 不合理 (<= 0, 或内缩区域里排不出垄线——区域相对垄宽太窄,
//          kBadRidgeWidth)
//   10409  路径规划失败: 内部兜底 (流程跑完仍无路点, kInternal)
//   10410  ENU 原点超时 (**local / file 两种模式共用** —— 原点是两种模式的公共必需输入, 只有一个
//          码): 收不到 gnss 的 ENU 原点 —— 开机宽限 origin_startup_grace_s 内从未收到, 或**收到过**
//          之后超过 origin_timeout_s 没有新原点。两种情况共用这一个码, 具体是哪种写在心跳 detail 里:
//          "开机 X s 仍未收到" / "X s 没有收到新原点"。
//          判定在 timerCallback ① (**整支最前**, 对两种模式统一): 宽限 origin_startup_grace_s 两模式一致
//          (宽限内不报, 用尽才报); 未就绪即撤路径并**直接 return** (连服务受理都不消费, 也不规划/读文件 ——
//          那一拍同样**每拍发一条空 /path**; **顶点投影不受这条判定影响**: 投影只跟着原点
//          变化走, 在 originLlhCallback 里就做完了)。**未超时就把它去掉** —— 加/去都只在这一个函数里 (updateOriginHealth),
//          所以原点恢复的那一拍码就消失, 不会挂在心跳上。来源统一写 gnss_driver (是它没给原点)
//   10411  /work_mode 信号超时 (从未收到且宽限已过 / 超过 work_mode_timeout_s 未更新):
//          判定在 timerCallback ② (紧跟 ENU 原点 ① 之后), 细节见下面枚举处那一大段说明。
//
// 顺序与分组: 通用/兜底 -> 文件路径 (模式 2) -> 规划失败·输入类 -> 规划失败·参数类 -> 规划失败·
// 内部兜底。后三组与 PlanStatus 的两组细分 (isBadInput() / isBadParams()) 一一对应。
// **PlanStatus 的取值重排不影响这里**: 心跳发出去的是本表的 ID, 与枚举内部的号无关 —— 枚举
// 重排后这一列一个字都不用动 (planStatusFaultCode 按符号映射)。
//
// 故障码只是"原因标签", 供 APP/云端展示"存在某个故障"; 不含等级。
// 决策动作由心跳的 status (STOP_MOTION 等) 表达, 不在此重复 —— 与 tractor_base
// 同一分工: 那边的 FaultSeverity 也只服务于"把底盘自检故障映射成本模块动作",
// 对外发布的 fault_codes 同样是纯 ID。本节点没有自己的执行器, 只发 ID + 心跳状态。
enum GlobalPathPlannerFaultCode : uint32_t {
  FAULT_NONE                  = 10400,   // 无故障 (模块号 104 + 序号 00)
  FAULT_PLAN_FAILED           = 10401,   // 路径规划失败 (无路点; 通用/兜底: 未细分状态)
  FAULT_FILE_PATH_INVALID     = 10402,   // 固定路径文件无效 (缺失/非 JSON/unit 不支持/crc32 不匹配/点数不足/经纬度越界)
  FAULT_PLAN_INPUT_TOO_FEW    = 10403,   // kInputTooFew   : 输入边界点不足 (< 3 个)
  FAULT_PLAN_DEDUP_TOO_FEW    = 10404,   // kDedupTooFew   : 去重后的边界点不足 (< 3 个)
  FAULT_PLAN_HULL_TOO_FEW     = 10405,   // kHullTooFew    : 凸包点不足 (顶点退化/共线)
  FAULT_PLAN_NON_CONVEX       = 10406,   // kNonConvex     : 地块非凸
  FAULT_PLAN_BAD_FIELD_MARGIN = 10407,   // kBadFieldMargin: field_margin 不合理 (内缩把可作业区域吃没/内缩后区域退化)
  FAULT_PLAN_BAD_RIDGE_WIDTH  = 10408,   // kBadRidgeWidth : ridge_width 不合理 (<= 0; 或内缩区域里排不出垄线)
  FAULT_PLAN_INTERNAL         = 10409,   // kInternal      : 内部兜底 (流程跑完仍无路点)
  FAULT_ORIGIN_TIMEOUT        = 10410,   // ENU 原点超时 (从未收到 / 收到过但超过 origin_timeout_s 没新原点)
  FAULT_WORK_MODE_TIMEOUT     = 10411,   // 工作模式信号超时 (从未收到且宽限已过 / 超过 work_mode_timeout_s 未更新)
};

// PlanStatus -> 故障码: 心跳里 fault_codes 发出去的就是这些 ID, APP/云端据此区分"为什么没有
// 路径"(点数不够 / 重复点太多 / 退化共线 / 非凸 / field_margin 不合理 / ridge_width 不合理 /
// 内部错误), 不必去看日志文本。
// 三点约定:
//   * kOk 映射到 FAULT_NONE —— 函数是全的, 调用方不用特判"成功"这一支;
//   * 每个失败状态都有**唯一**专码 (10403..10409), 不与通用码 10401 混用;
//   * 新增 PlanStatus 细分时**必须**在这里登记: 漏了 case 会被 -Wswitch 警告顶出来, 真漏了
//     也还有末尾的 FAULT_PLAN_FAILED 兜底 (退化成通用码), 不会静默丢故障。
inline uint32_t planStatusFaultCode(PlanStatus status) {
  switch (status) {
    case PlanStatus::kOk:
      return static_cast<uint32_t>(FAULT_NONE);
    case PlanStatus::kInputTooFew:
      return static_cast<uint32_t>(FAULT_PLAN_INPUT_TOO_FEW);
    case PlanStatus::kDedupTooFew:
      return static_cast<uint32_t>(FAULT_PLAN_DEDUP_TOO_FEW);
    case PlanStatus::kHullTooFew:
      return static_cast<uint32_t>(FAULT_PLAN_HULL_TOO_FEW);
    case PlanStatus::kNonConvex:
      return static_cast<uint32_t>(FAULT_PLAN_NON_CONVEX);
    case PlanStatus::kBadFieldMargin:
      return static_cast<uint32_t>(FAULT_PLAN_BAD_FIELD_MARGIN);
    case PlanStatus::kBadRidgeWidth:
      return static_cast<uint32_t>(FAULT_PLAN_BAD_RIDGE_WIDTH);
    case PlanStatus::kInternal:
      return static_cast<uint32_t>(FAULT_PLAN_INTERNAL);
  }
  return static_cast<uint32_t>(FAULT_PLAN_FAILED);
}

}   // namespace global_path_planner
