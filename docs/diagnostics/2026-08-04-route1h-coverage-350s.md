# 2026-08-04 route1h 长时覆盖运行：350 s，覆盖完成 t=301.5 s

- 日期：2026-08-04
- 预注册：`plans/2026-08-04-route1h-coverage-long-run-design.md`
- 运行：唯一一次，no-retry；`R1H / seed 20260727 / horizon 350 s`
- 结论：**搜索覆盖完成（100% @ t=301.5 s），全程可行**

## 身份

| 项 | 值 |
| --- | --- |
| binary | `fa5ee8b4b33026d775eac24bb0236b5ba32d0de8f0d3aaa2b2fd1021d0b8a34e` |
| 架构 | route-1 current-command + 动态 FIM + U=50（legacy 路径） |
| 搜索 | front-sector（r=400 m、半角 60°），3000×3000 m，300×300=90,000 单元 |
| run root | `/private/tmp/cbf2026-r1h-coverage-outputs/R1H/20260804T024807.619174Z_a646012dfe534a8c9188132e4c88c7c3` |
| 终止 | `completed`，returncode 0 |

## 结果

| 指标 | 值 |
| --- | --- |
| 覆盖完成 | **100% @ t=301.5 s**（首个 coverage==100% 帧） |
| 帧数 | 700（t=0→349.5 s），全部 14/14 `success` |
| 最差 stacked residual | `9.437e-15`（t=110 s，robot 3 `fixedCommCBF(#2)` 贴界） |
| ε 轨迹 | 峰值 30.31 m @ t=150–200 s（中段远离基地），随后回落至 ~9.5 m |
| 最大链延迟 | 15.28 ms |

### coverage–time 采样

| t (s) | coverage |
| --- | --- |
| 50 | 24.63% |
| 100 | 43.89% |
| 150 | 61.93% |
| 196 | 87.09% |
| 250 | 99.09% |
| 300 | 99.97% |
| 301.5 | 100.00% |

## 解读

- 主要目标达成：**算法在 301.5 s 完成 3000×3000 m 区域搜索覆盖**，
  与 legacy 全稿声明的 322 s 同量级（本次为独立测量，未与旧基线做
  同条件对比）；
- 覆盖全程保持 bounded 可行性：无不可行帧、无约束违反、ε 无发散
  （中段瞬态 30.3 m 后回落）；
- ε 峰值对应任务中段（远离海岸 base 的多跳定位链），未破坏可行性；
  estimator-bound 经验覆盖仍是独立负证据项。

## 边界

- 单 seed（20260727）、单几何（RGP 名义）；3×3 矩阵只覆盖 20 s；
- legacy 路径 + 仿真真值位置；非 formal gate；
- 搜索参数沿用 base config，未做参数扫描；
- 覆盖完成时间对 seed/队形/传感器参数的敏感性未测。

## 下一步建议

1. 多 seed 长时覆盖（如 3 seeds × 350 s）确认完成时间与可行性的稳健性；
2. ε 中段峰值（30 m）的机理与 estimator calibration 关联分析；
3. 论文主结果以"301.5 s 覆盖完成 + 全程可行"入稿；
4. TVT 14 页压缩。
