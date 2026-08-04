# 2026-08-04 route1h estimator warm-start 对比（strict vs deployment-restart）

- 日期：2026-08-04
- 数据：R1H 350 s 覆盖轨迹（seed 20260727，700 帧）
- 方法：`replay_localization_calibration`（variable-weight WNLS，
  动态 DAG），`STRICT_PREVIOUS_POLICY` vs
  `RESTART_BEFORE_FIRST_FINITE_POLICY`，range-noise seed `2026081301`
- 结论：**deployment-restart 把 invalid 从 4,194 打到 3，
  可用率 54%→91.8%，containment 51.6%→83.3%**

## 结果（700 帧 × 14 机器人 × 2 graph cases，dynamic_dag_wnls）

| 指标 | strict | restart（warm-start） |
| --- | --- | --- |
| converged / failed / invalid | 5,321 / 271 / 4,194 | **8,988 / 795 / 3** |
| 可用率（converged/total） | 54.4% | **91.8%** |
| containment（error ≤ eps） | 51.58% | **83.28%** |
| ε p95 | 14.31 m | 13.17 m |
| err/ε ratio p95 | 1.85 | 3.48 |

## 解读

- **warm-start 解决了"不说话"问题的主体**：invalid 几乎归零
  （4,194→3），可用率 54%→91.8%——与 DRA 官方记录一致
  （dynamic exact-direct 15,469→0）；
- **代价是 ε 校准变差**：restart 后 err/ε p95 从 1.85 升到 3.48，
  containment 虽升至 83.3%，仍未达 0.98 adequacy 门——与 DRA
  "conditional containment 下降"的官方发现同向；
- 结论：可用性是可控的（warm-start 修），**ε 包络校准是真正的剩余
  障碍**；这仍是 estimator calibration 负证据的一部分，不能直接
  进回路当 safety 保证。

## 图件

- `warmstart-comparison.png`（attempt 构成 + 可用率时间序列）。

## 边界

- 该模块是 calibration 版 WNLS（`variable_weight_nls_full_residual_jacobian_v1`），
  与 v6 qualified estimator（`solve_qualified_multistart`）是两代实现；
  前者验证机制（warm-start 收益），后者是论文目标估计器；
- 单 seed、单轨迹；非 formal gate。
