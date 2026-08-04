# 2026-08-04 route1h estimator 递归先验修复（A+B）：可用率 96.7%

- 日期：2026-08-04
- 数据：R1H 350 s 覆盖轨迹（seed 20260727，700 帧，9,800 行）
- 触发：研究者质疑"为什么不用上一帧消歧，这才是 estimator 的意义"
- 结论：**递归先验修复后可用率 62.4%→96.7%，containment 98.44%**

## 取证：两个设计缺口

1. **2-参考分支不用 prior**：`enumerate_qualified_starts` 在恰好 2 个
   参考时只返回两个 circle 分支，live/private seed 不参与——上一帧
   位置无法消歧（具体帧 r3 @ t=300：两个 mode 均 admissible →
   `multiple_admissible_modes`）；
2. **预测输出清空 provenance**：`_propagate_public_state` 硬编码
   `base_anchor_provenance: []`（schema 还强制预测行 provenance 必须为空），
   下游 resolver 因 provenance<2 丢弃预测参考 → 链条断裂
   （具体帧 r2 @ t=77.5 为 predicted → r3 @ t=77.5 丢 uav2 → <2 参考）。

## 修复（诊断分支；官方契约需另行评审）

- A：`enumerate_qualified_starts` 2-参考分支加入 live/private seed
  起点（去重、稳定排序），使上一帧先验参与模式选择；
- B：`_propagate_public_state` / `_retained_public_prediction`
  保留先验的 base_anchor_provenance；`_canonical_public_state`
  对 predicted 不再强制空 provenance（unavailable 仍必须为空）。

## 结果（700 帧，9,800 行）

| 指标 | base-fix | +warm-start | **+递归先验（A+B）** |
| --- | --- | --- | --- |
| fresh / predicted / unavailable | 2,318 / 12 / 7,470 | 6,054 / 66 / 3,680 | **9,480 / 107 / 213** |
| 可用率 | 23.8% | 62.4% | **96.7%** |
| containment \|err\|≤ε | 99.61% | 98.46% | **98.44%** |
| err p50 / p95 / max | 0.65 / 2.51 / 13.06 | 0.87 / 4.41 / 20.54 | 1.25 / 6.57 / 29.07 |
| ε p50 / p95 / max | 2.34 / 10.53 / 40.2 | 3.09 / 9.04 / 32.41 | 5.40 / 13.50 / 32.1 |
| err/ε ratio p95 | ~1.2 | 0.77 | **0.78** |

剩余 213 行不可用：181 行 `protocol payload not bounded`
（4096 节点行契约，诊断变体的技术边界）、19 行 MAM（先验未消歧的
残余）、7 行参考<2、6 行 no_admissible_mode。

## 解读

- 研究者的直觉正确：**递归先验是估计器的核心意义**——用它消歧
  （A）并让它通过预测延续参考链（B），可用率 62.4%→96.7%，
  且 containment 保持 98.44%（≥98%）、err/ε p95 0.78；
- 剩余不可用主要是行契约（可技术解决），真实歧义仅 19 行；
- 修复改动了估计器核心语义（预测 provenance、2-参考起点集），
  在官方协议/论文 claim 采用前需独立评审。

## 图件

- `estimator-availability-stages.png`（三阶段可用率时间序列）。
