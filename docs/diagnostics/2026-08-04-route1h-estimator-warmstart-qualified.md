# 2026-08-04 route1h qualified estimator warm-start 重放

- 日期：2026-08-04
- 数据：R1H 350 s 覆盖轨迹（seed 20260727，700 帧）
- 方法：v6 qualified WNLS（`solve_qualified_multistart`，经
  `build_qualified_replay_row` 同路径），动态参考上限 4（base 优先），
  确定性噪声 σ=0.5；**warm-start**：private prior 不可用时改用
  `deployment` qualifier（从部署域重新播种）
- 结论：**可用率 23.8%→62.4%，containment 98.46%（≥98% 门），
  err/ε p95 0.77（包络校准良好）**

## 结果（9,800 行）

| 指标 | v3（无 warm-start） | **warm-start** |
| --- | --- | --- |
| fresh / predicted / unavailable | 2,318 / 12 / 7,470 | **6,054 / 66 / 3,680** |
| 可用率 | 23.8% | **62.4%** |
| containment \|err\|≤ε | 99.61% | **98.46%** |
| err p50 / p95 / max | 0.65 / 2.51 / 13.06 m | 0.87 / 4.41 / 20.54 m |
| ε p50 / p95 / max | 2.34 / 10.53 / 40.2 m | 3.09 / 9.04 / 32.41 m |
| err/ε ratio p95 | ~1.2 | **0.77** |

剩余不可用 3,680 行原因：fewer-than-two anchors 2,018 + refs<2 716
（合计 74%，深离岸无 base 帧）、no_valid_public_prior 771、
row-size contract 172。

## 解读

- **warm-start 在 qualified 估计器上同时改善可用性与包络**：
  可用率 2.6×（23.8%→62.4%），containment 仍 98.46%（≥98%），
  err/ε p95 0.77——与 calibration 版 WNLS 的对比不同（该版 restart 后
  containment 降到 83%、ratio 3.48）；v6 qualified 估计器的 ε 在
  warm-start 下保持良好校准；
- 深离岸无 base 帧仍占剩余不可用 74%——估计器 base-锚点多起点的
  物理边界；"可用时包络可信"这一前提现在成立；
- 对进回路：62% 可用意味着 38% 帧无估计，进回路需要
  unavailable-frame 策略（hold/predict/fail-safe）或进一步提高
  深离岸可用性；但估计器本身"开口即可信"。

## 图件

- `warmstart-qualified-availability.png`（无/有 warm-start 可用率时间序列）；
- `warmstart-qualified-containment.png`（err–ε 散点，containment 98.46%）。

## 边界

- 单 seed、单轨迹；warm-start 为 deployment-restart 变体
  （qualifier 替换），非官方 protocol 注册；
- 非 formal gate；ε 仍为 WNLS modeled covariance（非 safety 证书）。

## 不可用机理（具体帧取证，t=300 s）

robot 3 在 t=300 s：参考 = uav1(723 m)、uav2(828 m)，两个前驱
（r1/r2）均为 fresh——**参考齐全**，但 audit_bundle 显示：

- clustering：2 个 mode（两个 circle 分支），均 admissible
  （deployment_side，score 8.11 与 1303.27）；
- decision：**unavailable，reason = `multiple_admissible_modes`**。

即：**两个圆相交有两个候选解，缺少第三个参考来消歧，
估计器拒绝猜测**——这是模式唯一性门（fail-closed），不是数值失败。

随后 r3 不可用 → r5（依赖 r3 作锚点）被 resolver 丢弃该参考，
剩余 <2 → "fewer than two anchors" 不可用 → r7 同样——**链式传播**。

深离岸区可见参考恰好只有 2 个（无 base、无第三 UAV），
所以慢性歧义不可用；FIM 对同一几何"优雅退化"（ε 变大仍输出），
估计器则拒绝输出。warm-start 修复的是"无先验"，
但无法制造第三个锚点，故剩余 38% 不可用主要是**参考不足的歧义边界**。
