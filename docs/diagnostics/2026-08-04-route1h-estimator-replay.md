# 2026-08-04 route1h 真实 WNLS estimator 离线重放

- 日期：2026-08-04
- 数据：R1H 350 s 覆盖运行（seed 20260727，700 帧）
- 方法：真实 qualified WNLS estimator（`predictive_wnls`，经
  `run_qualified_closure_campaign._build_condition_replay_row` 同路径），
  动态参考图（固定参考优先 + 最近可见补齐，**上限 4 参考**以满足
  4096 节点行契约），量测 = 真值距离 + 确定性噪声（σ=0.5，
  `default_rng(2026081301)`），无 warm-start。

## 结果（9,800 行 = 700 帧 × 14 机器人）

| 指标 | 值 |
| --- | --- |
| fresh / predicted / unavailable | 2,172 / 9 / 7,619 |
| 可用率 | **22.2%** |
| containment \|err\|≤ε | **99.63%** |
| containment \|err\|≤3ε | **100%** |
| 误差 p50 / p95 / max | 0.69 / 2.54 / 13.06 m |
| ε p50 / p95 / max | 2.47 / 8.99 / 40.21 m |
| corr(err, ε) | 0.659 |

## 解读

- **可用时包络校准良好**：真实估计器的 ε 包络在 99.63% 的可用行内
  覆盖真值误差，3ε 全覆盖；误差中位 0.69 m；
- **瓶颈是可用性（77.7% unavailable）**：与 DRA estimator calibration
  负证据一致（invalid/unavailable 比例高）；管线中的 warm-start
  recovery（deployment restart）曾把 dynamic exact-direct 不可用从
  15,469/139,720 降至 0/139,720，本重放未应用；
- **对进回路的意义**：包络可信是前提，但可用性必须先解决——
  否则控制器 78% 的帧没有可用估计；
- 行契约说明：动态全参考集（如 5 参考）会超出 producer 行 4096 节点
  上限（robot 3 实测 4,362 节点），官方 edge schedule 未知；本重放
  采用固定参考优先 + 最近可见补齐至 4 的**有界参考策略**（诊断变体）。

## 图件

- `estimator-error-distribution.png`（误差直方图 + err–ε 散点/containment）；
- `estimator-availability.png`（可用率时间序列）。

## 边界

- 噪声/参考策略为自建诊断变体，非官方 measurement stream；
- 无 warm-start；未跑 official registrar/analyzer 全链；
- 该结果不改变 estimator calibration 负证据结论，但提供了
  "可用时包络良好"的正向补充。
