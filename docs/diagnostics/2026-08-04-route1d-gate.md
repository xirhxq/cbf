# 2026-08-04 route1d gate：目标架构（动态 FIM + allocated-pairwise + ±25）20 s gate

- 日期：2026-08-04
- 运行：**唯一一次，no-retry**；seed `2026080201`（v6 admissible initial family v3，
  semantic `273174759820…`）、range-noise `2026081301`、horizon 20 s
- 结论：**NEGATIVE（t=1.0 s，2 个完成区间）**——
  `continuous certificate flow produced an infeasible hard QP`

## 身份

| 项 | 值 |
| --- | --- |
| source branch | `codex/cbf2026-route1d-whitelist-restore` @ `9537ca6`（基于 `06be565`） |
| binary SHA-256 | `a1943314c1115fb7bd1810e765062af6e97a51ccc3fc1dcd6e9f1b5a19f93dcd` |
| family | `cbf2026-v6-initial`（v3，semantic `273174759820…`），trajectory `2026080201` |
| config SHA-256 | 见 run root `preregistration.json` |
| run root | `/private/tmp/cbf2026-route1d-gate-outputs/20260804T101640_ac1400bdeb46` |
| 终止 | `loop_failure`，`completed_intervals=2`，`declared_frames=40`，returncode 1 |

## 前置修复

`06be565` 存在已知 v3 evidence whitelist bug（`controller evidence interior selection
is malformed`，修复提交 `b36a545` 已丢失）。本次按 DRA 已批准规格
（design SHA `65101939…`）恢复版本感知 exact-set validator：
`include/diagnostics/EvidenceStream.hpp`（纯 validator）+ `include/Swarm.hpp`
（按 `qualified-controller.schema-version` 派生，node 序列化前校验）。
恢复后 R1D 可正常产生 evidence（483 条记录）。

## 结果

| 区间 | 帧 | 结果 |
| --- | --- | --- |
| 0 | t=0.0 | 232 endpoint rows，全部 `optimal` |
| 1 | t=0.5 | 232 endpoint rows，全部 `optimal` |
| 2 | t=1.0 | **infeasible hard QP**（continuous certificate flow），无帧 2 rows |

- 动态 FIM 生效（reset 记录显示 active references = bases −3/−2/−1 + 低 index
  anchors），未出现协方差发散（与 Phase B 固定信息集负结果不同）；
- 失败是纯 bounded-input hard-QP infeasibility，与 route-1 单侧/广播协议无关
  （allocated-pairwise + bar-nu 保守界已消除实际命令耦合）。

## 解读

三层负证据现在完整闭环：

1. Phase B（固定 FIM + 旧控制器）：t=1.0 协方差发散（ε=447 m）→ 不可行；
2. R1D 动态 FIM（本 gate）：无发散，但 t=1.0 硬 QP 仍不可行；
3. 结论：**±25 输入集与该场景在 t≈1.0 的 CBF 率需求不匹配是独立于
   FIM 信息集与控制器语义的固有瓶颈**，论文"bounded-input joint feasibility
   为受监控假设"在 t≈1.0 被违反（与历史 250 s/v4/v5/v6 开发门一致）。

route-1 集成到当前分支的必要性需重新评估：当前架构（allocated-pairwise +
ν 界）在理论层已避开实际命令耦合，route-1 的增量价值主要在语义表述，
而非修复本 gate 的失败模式。

## 后续选项（交研究者）

1. **输入集重设计**：`±25 m/s` 的平台依据审计 + 初始队形/参考几何适配，
   使 t≈1.0 的 CBF 率需求落入输入集（原选项 1 的实质工作，现在确认为主瓶颈）；
2. **论文边界**：把 monitored joint-feasibility 的实证失败区间（t≈1.0）写为
   明确限制，不宣称 bounded 20 s 可行性；
3. **几何适配**：调整 admissible family 初始几何/队形参数并重新预注册 gate。
