# 2026-08-04 route1h gate：route-1 current-command + 动态 FIM + U=50（20 s PASS）

- 日期：2026-08-04
- 运行：**唯一一次，no-retry**；`R1H / seed 20260727 / horizon 20 s`
- 结论：**PASS**——40 帧全部 14/14 optimal，stacked residual 全程 ≥ -1e-7

## 身份

| 项 | 值 |
| --- | --- |
| source branch | `codex/cbf2026-route1d-whitelist-restore`（含 route-1 补丁提交） |
| binary SHA-256 | `fa5ee8b4b33026d775eac24bb0236b5ba32d0de8f0d3aaa2b2fd1021d0b8a34e` |
| config case | `R1H`（RGP 几何 + `route1.on` + 动态 FIM + `planar-component-max=50` + backward-difference） |
| run root | `/private/tmp/cbf2026-r1h-gate-outputs/R1H/20260804T023840.036286Z_d922830c0ae34f4a9c0e0e145b77591a` |
| 终止 | `completed`，returncode 0 |

## 架构（route-1 语义，v6 分支 legacy 路径）

- 责任分区：robot j 只建与 i<j 的 pair 行（safety 行数 = j−1，全低 index）；
- 拓扑顺序求解：Swarm 主循环按 id 顺序逐架 `optimise()`，每架求解后立即
  `receiveVelocity2D` 广播当帧命令，后继者用 **current 命令**（替代
  allocated-pairwise 的 ν=√2·U worst-case 假设）；
- 动态 FIM：bases 按 range、低 index anchors 动态加入（v6 legacy 路径默认）；
- 输入集：`planar-component-max=50`（研究者 2026-08-04 决定，
  论文以设计参数表述，不声称具体平台型号上限）。

## 结果

| 指标 | 值 |
| --- | --- |
| 帧数 / 末帧 | 40 / t=19.5 s（完整 20 s 视界） |
| 每帧最优 | 40/40 帧全部 14/14 `success` |
| 最差 stacked residual | `1.376e-11`（要求 ≥ -1e-7） |
| ε 范围 | 0.987–6.982 m（动态 FIM，无发散） |
| 最大链延迟 | 4.6 ms（要求 < 0.5 s） |
| 分区 | robot j safety 行 = j−1，引用全为低 index |

## 意义

三层负证据（固定 FIM 发散 → 动态 FIM 无发散但 ν 保守界不可行 →
输入集单独无效）之后，**route-1 current-command 语义 + 动态 FIM +
U=50 首次完整通过 20 s bounded gate**：

- 证明瓶颈确实在 ν worst-case 保守语义，移除后 ±50 自身界足够；
- 与研究者直觉一致："误差合理时速度不会离谱"——速度需求由实际命令
  与误差驱动，而非最坏假设；
- ε 全程 <7 m，动态 FIM 正常，无协方差发散。

## 边界与后续

- 本 gate 是诊断运行（legacy 行路径），不是 v6 formal gate 机制；
- route-1 实现语义（current command）与论文 ν-envelope 定理的衔接
  需按 theory §12 完成理论↔实现一致性说明（定理条件更新或实现
  标注为协议对齐）；
- 下一步建议：预注册多 seed/多轨迹扩展、定理更新、论文 U=50 与
  route-1 语义入稿、TVT 压缩规划。
