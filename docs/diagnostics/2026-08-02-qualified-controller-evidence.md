# CBF2026 qualified controller evidence 实现归档

日期：2026-08-02

## 结论与边界

Task 8 已实现并通过独立审查，最终审查结论为 Critical `0`、Important `0`、Minor `0`。
本任务建立的是可逐行保留、可独立重构、默认失败关闭的 controller evidence 机制，
而不是一次科学实验。
本阶段没有运行 development/confirmatory campaign 或完整任务仿真，
没有形成 Monte Carlo、任务成功率、理论假设成立或论文可声明的经验结论，
也没有修改论文主张。

本次实现提交为
`d6e25f547ea045d74c12c4988e429a31d9ea8ae7`
（`feat(evidence): reconstruct hybrid distributed certificates`），
父提交为 `9cae23b7fec1f49644c0525935b06bc3ab964083`。
提交共修改 9 个文件，增加 6,327 行、删除 70 行。

| 项目 | 已核验状态 |
| --- | --- |
| worktree | `/private/tmp/cbf2026-diagnostic` |
| branch | `codex/cbf2026-diagnostic` |
| upstream | 未配置 |
| archive 开始时工作区 | tracked/staged clean；仅保留未跟踪 `build-diagnostic/` |
| push | 本任务未 push |

## Evidence contract

1. 冻结宇宙不是从观测输出反推。
每个注册任务固定包含 14 架 UAV；每帧固定有 119 条 canonical full-edge/reconstructed rows
和 232 条 allocated endpoint rows。
初始化、estimator tuple、controller interval、endpoint、reset 与 mission terminal
均由声明的任务、条件、种子、帧和 UAV/edge 身份产生精确分母。
缺失、崩溃或畸形记录保留在分母中，不能静默消失。

2. analyzer 从 raw primitives 独立重构。
controller 记录提供节点状态、reference 身份与几何、测距方差、前驱协方差与速率界、
canonical edge、分配、normal/hard-only QP problem、solution、status 和 applied command。
Python analyzer 以这些原始量重算信息矩阵、协方差、不确定性半径及速率、
local/full CBF residual、输入界和 QP 可行性；序列化的派生汇总只用于交叉核验，
不作为证明输入。

3. 身份、版本与 reset 构成连续状态机。
公共 controller schema 将 `expected_node_count` 冻结为 14，
并要求 runtime nodes 与 `analyzer_only.truth` 的 robot ID 都精确等于 `1..14`。
审计从概念 bootstrap snapshot version 0 开始，按 interval 顺序验证版本链；
版本推进必须有唯一、已接受且连续的 reset，未变化版本不得夹带 committed reset，
allocation version 固定为 1。
reset 记录分阶段保存 proposal、closure、pre/post active set、proposed snapshot、
barrier/allocated rows、guard decision 与 commit/abort outcome。

4. reset QP witness 是同一次求解的证据。
`ExactResetWitnessStore` 保存 guard 实际消费的 feasibility 与 solution；
发射阶段只能读取并交叉核验这一 witness，不能为了生成 evidence 再求解一次。
accepted reset 缺少 exact solution 时必须失败关闭。

5. truth 与 runtime authority 分离。
真实位置只允许出现在 `analyzer_only` projection，用于事后 containment 检查；
它不是 reference 选择、FIM、CBF、reset guard 或控制输入。
reference 几何由同帧 interface estimates 和冻结 base positions 重新计算，
而不是信任序列化的 direction/distance。

6. 所有公共入口失败关闭。
非对象、非有限数、错误字段集合、越界整数/控制索引、错误 hash、非 PSD 协方差、
错序 reference、错误拓扑、缺失 endpoint/reset/terminal 以及不连续版本链，
都会形成 integrity error 或降低相应 numerator；异常不能逃出公共审计入口。

7. 故障注入不会进入生产行为。
环境变量 failure hooks 仅在定义 `CBF_EVIDENCE_TEST_HOOKS` 的测试构建中编译，
并额外受测试 campaign ID 限制。
同源 `SwarmNoEvidenceTestHooks` 与 `BUILD_TESTING=OFF` 的生产构建均验证了这些变量无效。
evidence 模式的 stdout 只包含逐行 flush 的有限 JSON object；人类可读进度进入 stderr，
崩溃时已 flush 的前缀仍可逐行解析，未完成 interval 有显式 incomplete marker。

因此，该证据边界只支持以下判断：producer/analyzer schema 对齐、原始量足以独立重构、
版本/reset/witness 链可审计、缺失或篡改会失败关闭。
它不支持飞行性能、长期稳定性、Monte Carlo 覆盖率、真实系统有效性或论文效果量结论。

## 四轮独立审查

| 轮次 | 结论 | 关键处理主题 |
| --- | --- | --- |
| 初审 | C4 / I4 / M1 | 修正冻结 edge/owner 宇宙；补齐精确 lifecycle schema、raw controller problem/QP witness、reset 生命周期与可重构字段；加入真实进程 crash-prefix/failure-retention 检查。 |
| 修复后复审 | C3 / I3 / M1 | 建立有序 snapshot/reset 连续链；以 interface/base position 为 reference geometry authority；统一 canonical 3-control/six-bound QP；封闭数值、hash、索引、拓扑与 proposed-snapshot 输入；消除 reset evidence 二次求解；加入 production compile-out gate。 |
| 第二次复审 | C0 / I1 / M0 | 唯一剩余问题是 controller truth 只检查唯一性、没有绑定冻结 14-UAV 身份宇宙。加入 duplicate ID 与 `expected_node_count + 1` 两个真实记录 mutation，并使 controller/mission numerator 同时下降。 |
| 最终复审 | C0 / I0 / M0 | 所有原始发现关闭，无新增 scope creep；Task 8 接受。 |

上述过程保留了 assertion-level RED 再 GREEN 的证据。
最后一轮中，两种 truth-ID mutation 在修复前均被公共 schema 接受；
修复后 schema 返回 false，controller availability numerator 从 2 降至 1，
mission success numerator 从 1 降至 0。

## 最终验证命令与结果

以下命令均从 `/private/tmp/cbf2026-diagnostic` 运行。

Task 8 Python schema、独立重构与真实短进程测试：

```bash
CBF_SWARM_BINARY=build-diagnostic/Swarm \
CBF_NO_HOOK_SWARM_BINARY=build-diagnostic/SwarmNoEvidenceTestHooks \
conda run -n cbf_env python -m unittest \
  tests.test_qualified_closure_evidence \
  tests.test_swarm_evidence_stream -v
```

结果：24/24 tests passed；其中 exact truth universe 测试包含 duplicate 与 out-of-range
两个 subtests。

C++ stream 与 exact reset witness：

```bash
./build-diagnostic/testEvidenceStream
```

结果：4/4 cases、27/27 assertions passed。

Task 7 compatibility gate：

```bash
for binary in \
  testFimRateCertificate \
  testBarrierEdgeRegistry \
  testAllocatedPairwiseCBF \
  testHybridCertificateGuard \
  testRobotDiagnostics \
  testRobustConstraintConstruction \
  testSwarmFailureHandling \
  testDiagnosticConfiguration
do
  "./build-diagnostic/${binary}"
done
```

结果合计 105/105 cases、1,074/1,074 assertions passed：
FIM 11/213、registry 8/188、allocated pairwise 11/55、hybrid guard 18/147、
Robot diagnostics 9/46、robust construction 34/161、Swarm failure handling 9/235、
diagnostic configuration 5/29。

冻结 qualified 配置：

```bash
conda run -n cbf_env python -m unittest tests.test_qualified_config -v
```

结果：3/3 tests passed。

production/no-hook gate 的可重跑命令：

```bash
cmake -S . -B /private/tmp/cbf2026-production-no-tests -DBUILD_TESTING=OFF
cmake --build /private/tmp/cbf2026-production-no-tests --target Swarm -j2
CBF_SWARM_BINARY=build-diagnostic/Swarm \
CBF_NO_HOOK_SWARM_BINARY=/private/tmp/cbf2026-production-no-tests/Swarm \
conda run -n cbf_env python -m unittest \
  tests.test_swarm_evidence_stream.SwarmEvidenceStdoutTests.test_failure_hooks_are_absent_from_no_hook_build -v
```

结果：`BUILD_TESTING=OFF` 的 Swarm 编译成功，显式 test campaign 下也忽略两个 failure-hook
环境变量；同源 `build-diagnostic/SwarmNoEvidenceTestHooks` 检查同样通过。

静态与冻结输入检查：

```bash
conda run -n cbf_env python -m py_compile \
  scripts/diagnostics/qualified_closure_evidence.py \
  tests/test_qualified_closure_evidence.py \
  tests/test_swarm_evidence_stream.py
git diff --check
shasum -a 256 config/diagnostics/rbp_pairwise.json
```

结果：`py_compile` 与 `git diff --check` 通过；历史配置哈希保持为
`4210765c7d2012280756cc5884b54d1b79eaf558046f59d2627d45689eefa8bc`。

## 未关闭事项与下一步

完整 Python discovery 中既有的 legacy `RegistrationTests` 问题仍未解决：
5 个 failure 加 5 个 error，涉及已存在的 v2 protocol-pair/circular-parent 基线问题。
Task 8 未修改这些文件，也不把 focused gate 解释为完整 discovery 已关闭。

下一步只进入 Task 9 tooling implementation：实现并测试 versioned development campaign
runner、analyzer、registrar 与 measurement generator。
这不自动授权运行 campaign、完整任务、调参、生成经验结论或修改论文；
任何真实 development/confirmatory 执行仍需其独立规格、冻结 provenance 与明确授权。
