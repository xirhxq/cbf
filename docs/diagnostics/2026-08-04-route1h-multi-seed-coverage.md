# 2026-08-04 route1h 多 seed 长时覆盖：3/3 PASS（结果级确定性）

- 日期：2026-08-04
- 预注册：`plans/2026-08-04-route1h-multi-seed-coverage-design.md`
- 结论：**3/3 seed 全部完成覆盖（100% @ t=301.5 s），全程可行；
  三 seed 结果逐位一致（legacy 路径对 seed 确定性）**

## 运行

| seed | run root | returncode |
| --- | --- | --- |
| 20260727 | `/private/tmp/cbf2026-r1h-coverage-outputs/R1H/20260804T024807.619174Z_…` | 0 |
| 20260801 | `/private/tmp/cbf2026-r1h-multi-coverage-outputs/R1H/20260804T025119.784209Z_…` | 0 |
| 20260802 | `/private/tmp/cbf2026-r1h-multi-coverage-outputs/R1H/20260804T025141.782918Z_…` | 0 |

binary `fa5ee8b4…`；case `R1H`（route-1 + 动态 FIM + U=50）；350 s；no-retry。

## 结果

| seed | 覆盖完成 | 帧 | 14/14 | 最差 residual | ε max | ε 峰值 t | 链延迟 max |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 20260727 | 301.5 s | 700 | ✅ | 9.4e-15 | 30.31 m | 184.5 s | 15.3 ms |
| 20260801 | 301.5 s | 700 | ✅ | 9.4e-15 | 30.31 m | 184.5 s | 39.3 ms |
| 20260802 | 301.5 s | 700 | ✅ | 9.4e-15 | 30.31 m | 184.5 s | 27.7 ms |

## 观察

- **结果级确定性**：三 seed 的覆盖完成时间、residual、ε 轨迹逐位一致；
  仅链延迟（墙钟）不同 → legacy 路径未消费随机噪声（seed 不改变仿真路径）；
- 因此"多 seed 稳健性"在本路径退化为确定性确认；真正的多样性由
  几何变体（20 s 矩阵 G2/G3）与未来的噪声/估计器路径提供；
- 覆盖完成 301.5 s、全程可行、ε 中段峰值 30.31 m 后回落，
  与单 seed 覆盖报告一致。

## 边界与含义

- 论文可写：该配置下覆盖完成时间为确定性结果（301.5 s）；
- 随机噪声/估计器不确定性未在本 legacy 路径中消费——多轨迹 estimator
  验证（含 range noise seeds）仍是独立证据项；
- 几何多样性覆盖（G2/G3）仅 20 s；长时几何变体可作后续。
