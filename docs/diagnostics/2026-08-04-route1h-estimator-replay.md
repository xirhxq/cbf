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

## 修正（v2 重跑，适配器状态处理修正后）

状态计数不变（fresh 2,172 / predicted 9 / unavailable 7,619），
unavailable 原因分类（修正前有 2,146 行 "previous public state
schema is invalid" 为适配器伪影，修正后消失）：

| 原因 | 行数 | 含义 |
| --- | --- | --- |
| fewer than two condition-local reference anchors remain | 4,299 | 参考链断裂：低 index 队友不可用后，剩余可用锚点 <2 |
| no_valid_public_prior | 2,160 | 上一帧无有效先验（自身连续不可用的结果） |
| qualified references must contain at least two records | 1,151 | 解析后参考 <2（链断的另一种表现） |
| qualified protocol payload is not bounded | 8 | 行超 4096 节点契约（罕见） |
| prediction_expired | 1 | 预测过期 |

**时序模式**：robot 1（纯 base 参考）全程 fresh；大多数机器人
t≈150–300 s（远端、ε 峰值时段）开始断链不可用；robot 5 最早
（t≈1.5–2.5 s）。即"大部分时间不说话"发生在任务后半程的
多跳远端区域——上游不可用沿参考链向下游传播。

**warm-start 对比尝试**：`replay_localization_calibration` 的
strict/restart 双策略对 R1H data+manifest 返回 runner_setup_error
（输入 manifest 格式不兼容，需要官方 calibration bundle）；
管线记录的 warm-start recovery 在官方轨迹上曾把 dynamic
exact-direct 不可用从 15,469/139,720 降至 0/139,720。

## 修正 v3（base 优先参考策略；研究者质疑驱动）

研究者质疑"为什么没有有效先验 / 是否与 FIM 用同一动态 DAG"，
取证发现适配器策略 bug：cap-4 的"固定参考 + 最近补齐"会把可见 base
挤掉（robot 5 在 t=1.5 s 距 base 仅 220/235 m，却选了 4 个更近 UAV），
而 WNLS 多起点引导依赖 base 锚点。修正为**固定参考 → 可见 base →
最近 UAV** 后：

| 指标 | v2（最近补齐） | v3（base 优先） |
| --- | --- | --- |
| fresh / predicted / unavailable | 2,172 / 9 / 7,619 | **2,318 / 12 / 7,470** |
| 可用率 | 22.2% | **23.8%** |
| containment \|err\|≤ε | 99.63% | 99.61% |
| err p50 / p95 | 0.69 / 2.54 m | 0.65 / 2.51 m |

- robot 5 最早断链从 t≈1.5 s 推迟到 **t≈250 s**；
- 剩余不可用 7,470 中 **5,155（69%）无 base 可见**（深离岸区域），
  2,315 有 base 但仍不可用；
- 结论：估计器与 FIM 使用同一动态 DAG（`covariance_formation`），
  但两者失效模式不同——FIM 优雅退化（ε 增大、仍输出），WNLS
  fail-closed（无 base 锚点/无合格 mode 就拒绝输出）；
  深离岸区动态图无 base 可加，与 FIM ε 峰值同源；
- "无有效先验"是 fail-closed 的连锁结果：上一帧 unavailable →
  无 prior → 继续 unavailable（适配器异常分支清空状态使该效应更强）；
  warm-start（从部署位置重启先验）正是针对此的管线机制。
