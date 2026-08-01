# CBF2026 qualified controller evidence 独立审查

日期：2026-08-02

## 审查范围

本审查只核验作者归档
`docs/diagnostics/2026-08-02-qualified-controller-evidence.md`、实现提交、Task 8 源码与测试、
四轮独立审查结论及当前可重跑门限。
未运行 development/confirmatory campaign，未形成或审查飞行性能、Monte Carlo、任务成功率、
理论有效性或论文效果量结论，也未修改作者报告或实现代码。

实现提交为 `d6e25f547ea045d74c12c4988e429a31d9ea8ae7`
（`feat(evidence): reconstruct hybrid distributed certificates`），
父提交为 `9cae23b7fec1f49644c0525935b06bc3ab964083`。
Git 独立核验为 9 个文件、增加 6,327 行、删除 70 行；分支为
`codex/cbf2026-diagnostic`，无 upstream。

作者报告独立读取结果：

- bytes：`8912`
- SHA-256：`1d47c228278e0130e782afbb6696d2d972d76b3947ea5cfcd32bc432526fab43`

## 独立重跑

以下命令均从 `/private/tmp/cbf2026-diagnostic` 运行：

```bash
CBF_SWARM_BINARY=build-diagnostic/Swarm \
CBF_NO_HOOK_SWARM_BINARY=build-diagnostic/SwarmNoEvidenceTestHooks \
conda run -n cbf_env env PYTHONPATH=. python -m unittest \
  tests.test_qualified_closure_evidence \
  tests.test_swarm_evidence_stream -v

./build-diagnostic/testEvidenceStream

conda run -n cbf_env env PYTHONPATH=. python -m unittest \
  tests.test_qualified_config -v

CBF_SWARM_BINARY=build-diagnostic/Swarm \
CBF_NO_HOOK_SWARM_BINARY=/private/tmp/cbf2026-production-no-tests/Swarm \
conda run -n cbf_env env PYTHONPATH=. python -m unittest \
  tests.test_swarm_evidence_stream.SwarmEvidenceStdoutTests.test_failure_hooks_are_absent_from_no_hook_build -v

conda run -n cbf_env env PYTHONPATH=. python -m py_compile \
  scripts/diagnostics/qualified_closure_evidence.py \
  tests/test_qualified_closure_evidence.py \
  tests/test_swarm_evidence_stream.py

git diff --check
shasum -a 256 config/diagnostics/rbp_pairwise.json
```

结果分别为：Task 8 Python `24/24`；C++ evidence stream/exact reset witness
`4/4` cases、`27/27` assertions；qualified config `3/3`；production/no-hook
`1/1`；`py_compile` 与 `git diff --check` 通过；冻结配置 SHA-256 为
`4210765c7d2012280756cc5884b54d1b79eaf558046f59d2627d45689eefa8bc`。

逐个重跑
`testFimRateCertificate`、`testBarrierEdgeRegistry`、`testAllocatedPairwiseCBF`、
`testHybridCertificateGuard`、`testRobotDiagnostics`、
`testRobustConstraintConstruction`、`testSwarmFailureHandling`、
`testDiagnosticConfiguration`，合计 `105/105` cases、`1,074/1,074` assertions。

作为边界检查，`python -m unittest discover -s tests -v` 当前运行 `1003` tests，
结果为 5 failures、8 errors。其中作者报告所指的 legacy `RegistrationTests`
子集确为 5 failures、5 errors；另 3 errors 来自缺失的外部 `/private/tmp/cbf2026-results/...`
truth fixture。该 discovery 未关闭，focused gate 不代表完整 discovery 通过。

## 四轮 chronology 核验

四轮结论与实际审查记录一致：初审 `C4/I4/M1`；第一轮修复后
`C3/I3/M1`；第二轮修复后 `C0/I1/M0`；最终轮 `C0/I0/M0`。
依次关闭了冻结 edge/owner 宇宙、lifecycle 与 reset 原始证据、controller/QP
独立重构、版本链与联合 reset 伪造、reference geometry、精确输入界、数值与索引
失败关闭、proposed snapshot/拓扑、同次 reset solve witness、production hook compile-out，
以及最终的 14-UAV truth identity 边界。

最终 truth-ID 回归确认 duplicate 与 out-of-universe mutation 均使公共 schema 返回 false，
controller numerator 从 2 降至 1，mission numerator 从 1 降至 0；公共 controller
schema/reconstruction 固定为 14 UAV，单节点 seam 仅存在于下划线私有测试实现中。

## Evidence boundary

可接受的证据边界为：冻结 14 UAV、119 canonical full rows、232 endpoint rows；
analyzer 从 raw primitives 重构并仅比较派生汇总；snapshot/reset/同次 QP witness 连续可审计；
畸形、缺失、非有限或身份/版本不一致证据失败关闭；`BUILD_TESTING=OFF` 不含 failure hooks。
该边界证明实现与审计协议闭合，不证明 campaign、长期稳定性、真实系统性能或论文经验主张。

## 结论

- Critical：`0`
- Important：`0`
- Minor：`0`
- Ready：`Yes`

作者报告与实现提交、测试结果、四轮 chronology 和声明边界一致，可作为 Task 8 source-author
归档的独立审查记录。
