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

## 追加：cap-3 消行契约（可用率 98.6%）

剩余 213 行不可用中 181 行为 4096 节点行契约。将动态参考上限从 4
降为 3（固定参考 + 最近 base/UAV，base 优先）后完整 700 帧重放：

| 指标 | cap-4（A+B） | **cap-3（A+B）** |
| --- | --- | --- |
| fresh / predicted / unavailable | 9,480 / 107 / 213 | **9,659 / 129 / 12** |
| 可用率 | 96.7% | **98.6%** |
| containment | 98.44% | **98.47%** |
| err p50 / p95 / max | 1.25 / 6.57 / 29.07 m | 1.34 / 6.72 / 29.07 m |
| ε p50 / p95 / max | 5.40 / 13.50 / 32.1 m | 5.52 / 13.47 / 36.21 m |
| err/ε ratio p95 | 0.78 | **0.78** |

- 行契约失败（181 行）与 MAM（19 行）均清零；
- 剩余 12 行全部 `no_admissible_mode`（3 参考下少数几何仍无合格 mode）；
- cap-3 的代价是参考略少（误差 p95 6.57→6.72 m，可忽略）；
- 结论：递归先验 + 3 参考 = **98.6% 可用率 + 98.47% containment**，
  估计器主体可支撑进回路设计。

## 追加：剩余 12 行 no_admissible_mode 机理

12 行不可用全部是 `no_admissible_mode`，分两种：

1. **11 行：单模式但 innovation 超阈值**。估计收敛且所有候选一致
   （如 f143 r14：cost 0.494、估计误差 ~6.2 m），但
   `history_innovation_exceeds_limit` 分数 12.77–86,303.88
   超过冻结 q-threshold 11.83 → 拒绝发布。这些行位于任务边缘
   （r7/r13/r14 在 y≈930–1470 或 y≈-870），3 个 UAV 锚点距离
   230–800 m，锚点自身协方差 ~1.4–4.1 m²（估计误差数米）。
   本质：**锚点估计误差使量测一致性略微超出模型/阈值**——
   ε 校准缺口在尾部的表现，估计器按设计拒绝低置信解。
2. **1 行（f153 r14）：无候选起点**。三个锚点（r11 F、r12 F、
   r13 P，距离 537/340/230 m）的测距圆互不相交，代数/圆交点
   起点为空 → candidates=0。本质同样是锚点误差使量测几何互斥。

两类的共同根因：任务边缘 UAV 锚点的累积估计误差，使量测集与
冻结的 innovation 阈值/圆几何不完全一致；估计器 fail-closed。
这不是 bug，是校准缺口（锚点协方差欠估）的尾部表现，占比 0.12%。
