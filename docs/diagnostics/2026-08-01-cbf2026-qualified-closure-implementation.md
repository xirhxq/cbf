# Qualified-Mode / Hybrid Distributed-CBF Implementation Verification

Date: 2026-08-02 (Asia/Shanghai)

Verification scope: Task 10 implementation archive plus failed-v1/failed-v2
to development-v3 authorization-lifecycle recovery

Worktree: `/private/tmp/cbf2026-diagnostic`

Branch: `codex/cbf2026-diagnostic`

## Status and claim boundary

The current code-fix snapshot is
`64f64d00a1fd6b7ab01760e340eca883be58f338`
(tree `b2c7cc555e5ee6c684ac6fb2eb14d44157c5f9dd`, parent `02f0f13...`). It
preserves the verified qualified-mode/hybrid distributed-CBF implementation
and repairs the postcommit binding failure found after the immutable
development-v2 authorization child. Fresh post-fix Python verification found
no new changed-contract failure, error, or skip. Complete discovery still
exits nonzero: the exact same 13 legacy/external-input outcomes recorded below
remain nonpassing. This report does not waive them or describe the complete
repository suite as green.

The original implementation review found Critical C1/C2 and Important I1;
`ec7879404604ca19e12a94cdfa8cb00524ecd305` closed all three and its scoped
re-review returned C0/I0/M0. The reviewed archive was committed at `709f4ef`,
but development-v1 independent preflight then found a new lifecycle C1 and
withheld authorization. Commit `eafa68c...` closes that lifecycle through TDD;
its review rounds were I1 for the protocol declaration, M1 for the
authorization fixture, and finally C0/I0/M0. Development-v2 subsequently
passed its four-artifact topology but failed mandatory production postcommit
validation before root allocation. Commit `64f64d0...` closes that failure and
its independent review returned C0/I0/M0. Only development-v3 registration,
independent preflight, and four-artifact authorization closure may proceed.
Execution remains closed. This report is not itself a protocol,
preflight, authorization, development result, or confirmatory result. No
development, confirmatory, smoke, full-mission, or other campaign was run, and
no execution root was created.

## Implementation identity

The planning execution baseline was
`19c1bab7825d9e974751f9223867124abd6b790c`. The complete source commit
sequence from that baseline through the verified snapshot is:

1. `cbf43c0adb8c2cb66e59a69b1bba3a66ab8a93f0` -- `feat(estimator): cluster globally distinct solution modes`
2. `ba20ce81a52919b91402f62082c8f63b3a07dff7` -- `fix(estimator): preserve distinct modes and validate seeds`
3. `df7f4d10709636691c53626c8bd453152b6826b9` -- `feat(estimator): qualify modes with deployment and private history`
4. `0bb7947bdb337f3ad96ea26b3235942175d0eef3` -- `fix(estimator): close qualified lifecycle validation gaps`
5. `736ffeede3e25e16d78fcb2de21867cd3daccee2` -- `fix(estimator): fail closed on recursive and oversized inputs`
6. `ccc1f111a82430ce37594e2030aafe2a691fcf5d` -- `feat(estimator): publish only one qualified global mode`
7. `d08aa5eac65b9123f3985213e72e0e7c85118725` -- `fix(estimator): validate complete ordered audit evidence`
8. `e001fe23d1f59f1241d27b9d7fc641e003491245` -- `fix(estimator): preserve legacy validation and ordered projections`
9. `a398002c4461d613f2b16c948ef4e71d715977e2` -- `feat(cbf): certify topological fim radius rates`
10. `9ac967d93e73e89f55ccbdbec4b78b1819dfa802` -- `fix(cbf): preserve legacy covariance without input bounds`
11. `44aedd80008516afc04c852e53e99efd2428794b` -- `feat(cbf): allocate canonical pairwise hard rows`
12. `906a204b3a61db5624e6ea65bab7b44083bb6a07` -- `fix(cbf): close allocated edge snapshot invariants`
13. `a5442980bd23e8dd2968d8750128b1fbe180213d` -- `feat(cbf): guard hybrid certificate resets`
14. `9cae23b7fec1f49644c0525935b06bc3ab964083` -- `fix(estimator): enforce exact audit schema parity`
15. `d6e25f547ea045d74c12c4988e429a31d9ea8ae7` -- `feat(evidence): reconstruct hybrid distributed certificates`
16. `01dc8ab357d45a2617fc44644293bef4cb77c75f` -- `docs(evidence): archive controller evidence review`
17. `0e8e9031f83a62a6de327383e86735aede0af733` -- `feat(experiments): add qualified closure development campaign`
18. `ec7879404604ca19e12a94cdfa8cb00524ecd305` -- `fix(experiments): close campaign evidence gates`
19. `709f4ef6cef60b0754527fc62d8ca64ac7a88a12` -- `docs(cbf2026): approve qualified closure implementation`
20. `0e4c439ae35f8490b27019aeb26b1c46ba9ab3f7` -- `docs(cbf2026): record qualified closure v1 preflight failure`
21. `eafa68c3631c590b9f1070bcc8d313ca4f3b7705` -- `fix(experiments): close protocol authorization lifecycle`
22. `857de7ee8dd563fdd5fa15b6860ed54da6758b37` -- `docs(cbf2026): approve qualified closure v2 recovery`
23. `ae7f68c547eaccfb5c4be0b44a08895ab8beeb36` -- `docs(cbf2026): authorize qualified closure development v2`
24. `02f0f13ce1b3803b84840221221694ab02d2441d` -- `docs(cbf2026): record qualified closure v2 gate failure`
25. `64f64d00a1fd6b7ab01760e340eca883be58f338` -- `fix(experiments): close postcommit campaign bindings`

The exact 60 paths changed over
`19c1bab7825d9e974751f9223867124abd6b790c..64f64d00a1fd6b7ab01760e340eca883be58f338`
are:

```text
CMakeLists.txt
config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json
config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json
docs/diagnostics/2026-08-01-cbf2026-qualified-closure-development-v1-protocol.json
docs/diagnostics/2026-08-01-cbf2026-qualified-closure-development-v1-protocol.md
docs/diagnostics/2026-08-01-cbf2026-qualified-closure-implementation.md
docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.json
docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.md
docs/diagnostics/2026-08-02-qualified-controller-evidence.md
docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-development-v1-preflight.md
docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-implementation-review.md
docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-authorization.json
docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-postcommit-validation.md
docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-preflight.md
docs/diagnostics/reviews/2026-08-02-qualified-controller-evidence-review.md
docs/superpowers/plans/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-implementation.md
include/Robot.hpp
include/Swarm.hpp
include/cbf/AllocatedPairwiseCBF.hpp
include/cbf/BarrierEdgeRegistry.hpp
include/cbf/FimRateCertificate.hpp
include/cbf/HybridCertificateGuard.hpp
include/communicators/CommunicatorBase.hpp
include/communicators/CommunicatorCentral.hpp
include/diagnostics/EvidenceStream.hpp
scripts/diagnostics/analyze_qualified_closure_campaign.py
scripts/diagnostics/analyze_qualified_estimator.py
scripts/diagnostics/estimator_lifecycle.py
scripts/diagnostics/generate_qualified_measurements.py
scripts/diagnostics/predictive_wnls.py
scripts/diagnostics/qualified_closure_evidence.py
scripts/diagnostics/qualified_config.py
scripts/diagnostics/qualified_modes.py
scripts/diagnostics/register_qualified_closure_campaign.py
scripts/diagnostics/replay_qualified_estimator.py
scripts/diagnostics/run_qualified_closure_campaign.py
scripts/diagnostics/two_range_reacquisition.py
src/Swarm.cpp
tests/testAllocatedPairwiseCBF.cpp
tests/testBarrierEdgeRegistry.cpp
tests/testDiagnosticConfiguration.cpp
tests/testEvidenceStream.cpp
tests/testFimRateCertificate.cpp
tests/testHybridCertificateGuard.cpp
tests/testRobotDiagnostics.cpp
tests/testRobustConstraintConstruction.cpp
tests/testSwarmFailureHandling.cpp
tests/test_analyze_qualified_closure_campaign.py
tests/test_analyze_qualified_estimator.py
tests/test_estimator_lifecycle.py
tests/test_generate_qualified_measurements.py
tests/test_predictive_wnls.py
tests/test_qualified_closure_evidence.py
tests/test_qualified_config.py
tests/test_qualified_modes.py
tests/test_register_qualified_closure_campaign.py
tests/test_replay_qualified_estimator.py
tests/test_run_qualified_closure_campaign.py
tests/test_swarm_evidence_stream.py
tests/test_two_range_reacquisition.py
```

The approved design, design review, current recovery plan, and archived plan
review identities are respectively:

```text
ced4824b0cb8b2919579469e876650aa45e71c26f1b5853ddf2633550822127b  docs/superpowers/specs/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-design.md
897881d421d1bf78b4d0b1aed5dca86b574430aed0290e35f843fe6c840d6c9c  docs/superpowers/specs/reviews/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-design-review.md
ffcf158ec6ed89fd313d8ae6479e1339f23b570fda502a637239c2bc4864feb1  docs/superpowers/plans/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-implementation.md
cf5bd776bb99183015dce678c4136964edcabd81a64624ff1c6b446d7a107d90  docs/superpowers/plans/reviews/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-implementation-plan-review.md
```

The recovery-plan hash is a working-document identity for this update. The
plan, this report, and the independent recovery review must first enter a clean
documentation-closure commit after `64f64d0...`; they are not permitted
members of the later exact four-artifact authorization commit.

## Fresh Python discovery

The required complete discovery was run exactly with Python 3.11.12 from the
`cbf_env` conda environment:

```bash
conda run -n cbf_env python -m unittest discover -s tests -p 'test_*.py' -v
```

Fresh post-postcommit-fix result at
`64f64d00a1fd6b7ab01760e340eca883be58f338`: 1,162 tests; 1,149 passed,
5 failed, 8 errored, and 0 skipped. All changed-contract tests passed. The
exact nonpassing tests and attribution are unchanged and are as follows.

Five failures in the untouched legacy
`test_register_two_range_reacquisition.RegistrationTests` class were:

```text
test_broken_symlink_retired_root_rejects_before_outputs
test_dirty_required_source_rejects_before_outputs
test_markdown_json_mismatch_across_dry_runs_rejects_publication
test_required_source_symlink_rejects_before_outputs
test_source_path_size_and_sha_drift_between_reads_removes_outputs
```

Five errors in that same legacy class were:

```text
test_cli_main_registers_paired_outputs
test_partial_output_write_removes_both_protocol_files
test_preexisting_registered_root_rejects_before_outputs
test_registers_bound_sources_and_comparators_to_paired_outputs
test_two_dry_runs_are_byte_identical_and_semantically_paired
```

All ten first encounter the same fail-closed guard:

```text
ValueError: circular implementation parent already contains generated protocol:
docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.md
```

This is a pre-existing repository-state incompatibility rather than a changed
contract regression. Read-only proof:

- `git diff --quiet 19c1bab..HEAD --` over
  `tests/test_register_two_range_reacquisition.py`,
  `scripts/diagnostics/register_two_range_reacquisition.py`,
  `tests/test_extract_predictive_wnls_stage0.py`, and
  `tests/test_extract_two_range_reacquisition_fixture.py` returned 0: none of
  these legacy files changed in Tasks 2--10.
- Commit `736ffeede3e25e16d78fcb2de21867cd3daccee2`, the already-recorded
  Task 4 baseline, already tracks both the v2 Markdown and JSON protocol
  files. The tests clone the current implementation parent, so the circular
  parent guard wins before each test's intended later guard.
- The same 5-failure/5-error class-level condition was reproduced and
  attributed at Task 4 before the controller/evidence/campaign changes.

The remaining three errors were exact external-fixture dependencies:

```text
test_extract_predictive_wnls_stage0.Stage0ExtractorContractTests.test_first_transition_uses_preceding_applied_command
test_extract_two_range_reacquisition_fixture.FixtureIdentityTests.test_extracts_only_approved_mechanism_deterministically
test_extract_two_range_reacquisition_fixture.FixtureIdentityTests.test_v4_root_aba_during_process_read_keeps_original_child_inode
```

Each fails while opening the same absent external truth file, before a
changed qualified contract is exercised:

```text
/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/
20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/
2026-07-28_14-27-53_R_seed_20260727_250s/data.json
```

The impact boundary is narrow and explicit: this worktree cannot certify
those three historical fixture-backed extraction cases or the ten old
registrar cases under a parent that already contains their generated v2
protocol. It does certify, through this discovery, that every other Python
test including all Tasks 2--10 qualified and lifecycle contracts passed. No
test or gate was weakened, skipped, or edited to obtain that attribution.

## Independent review and fix closure

The initial independent specification/conformance review of
`19c1bab7825d9e974751f9223867124abd6b790c..0e8e9031f83a62a6de327383e86735aede0af733`
reported Critical 2, Important 1, Minor 0 and blocked protocol generation:

- C1: the campaign consumed but did not retain the complete frame-zero
  qualified-estimator audit, leaving the condition-local lifecycle base case
  unavailable for independent reconstruction;
- C2: fresh retention used an outcome-filtered subset instead of the frozen
  complete primary post-initialization tuple universe; and
- I1: the 6 GB hard floor did not cover synchronous measurement generation.

Commit `ec7879404604ca19e12a94cdfa8cb00524ecd305` changed exactly three
production scripts and two tests while leaving the then-current 50-path
baseline-to-`ec78794` set unchanged. C1 is closed by retaining and independently reconstructing the
C++ initialization record plus both condition-specific frame-zero qualified
audits as one composite initialization witness; frame zero remains outside
the post-initialization estimator denominator. C2 is closed by using the
protocol-frozen `estimator_per_condition` primary universe for fresh
retention, so missing, predicted, ambiguous, or unavailable observed tuples
cannot shrink the denominator. I1 is closed by checking the 6 GB floor before
each synchronous runtime/audit measurement pair, publishing an equal paired
prefix with exact `disk_hard_floor` terminal status, and propagating that
reason through the frozen remaining schedule.

The independent scoped re-review of
`0e8e9031f83a62a6de327383e86735aede0af733..ec7879404604ca19e12a94cdfa8cb00524ecd305`
found no new Critical or Important issue and returned Critical 0, Important
0, Minor 0, Ready Yes. Its fresh evidence was: the eight exact C1/C2/I1
regressions 8/8, the five-module campaign suite 160/160, all three amended
production scripts through `py_compile`, `git diff --check`, direct C++
111/111 cases and 1,106/1,106 assertions, and CTest 1/1. At that historical
snapshot, the root reran a 1,149-test discovery and the same five-module suite
160/160. The implementation was therefore approved only for the next Task 10
protocol-generation step; preflight, execution authorization, and all campaign
gates remained closed.

## Failed-v1/failed-v2 to development-v3 lifecycle chronology

The subsequent registration lifecycle is part of the audit record and does
not alter the scientific verification above:

1. Commit `709f4ef6cef60b0754527fc62d8ca64ac7a88a12` archived this
   implementation report and its independent review. Its tree was
   `8a92bba68977792d7e97ece996bed2fcf61e859e`.
2. Task 10 Step 7 generated the development-v1 protocol pair. The JSON is
   41,979 bytes with SHA-256
   `9a5e9d2503d083bdbb239468a88cf5ceed2cc283978e8beed5badc020ae74343`;
   the Markdown is 312 bytes with SHA-256
   `19cf657d787bfe76bfa2aecf35ce06baf9fd2f57ff2deeab12e894429ccebffc`.
3. Independent Step 8 preflight returned C1/I0/M0 and
   `FAILED — NOT READY`. Its 9,274 bytes have SHA-256
   `7ef7fbbf09d1ef95ecad482282a75398158fb97008fdb8a08745efeb51523515`.
   The exact-HEAD validator rejected the registrar's own relevant untracked
   outputs before commit, while the required later artifact commit advanced
   HEAD and was rejected after commit. No permitted state could authorize
   execution.
4. The v1 authorization was deliberately withheld and both v1 roots remained
   absent. Commit `0e4c439ae35f8490b27019aeb26b1c46ba9ab3f7` added exactly the
   JSON, Markdown, and failed preflight as immutable evidence. Development-v1
   is consumed and terminally failed; it may not be executed, retried,
   overwritten, repaired in place, or reinterpreted.
5. Commit `eafa68c3631c590b9f1070bcc8d313ca4f3b7705` repairs the
   lifecycle under TDD. Development alone advances to version `v2` with roots
   `/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v2` and
   `/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v2`;
   confirmatory remains version `v1`.
6. The updated report, plan, and independent recovery review must first be
   committed in one clean documentation-closure state after code-fix identity
   `eafa68c...`. The registrar will record that then-current exact commit as
   `protocol.repository.head`; it is intentionally unknown until the
   documentation closure is committed. The new authorization must bind the
   exact protocol SHA-256, that registered implementation identity, exact
   independent preflight SHA-256, date
   `2026-08-02`, and the researcher's exact standing-authorization text
   `批准`. That text's UTF-8 SHA-256 is
   `8cbe697b157364a5b13646285b38409dc53ec5287deeb7913493e65b275cd14d`.
   Unit-test fixture text is test data only and must not be copied into the
   production authorization.
   Execution validation accepts only a current commit whose sole parent is
   the documentation-closure commit recorded in the protocol and whose
   complete diff is exactly the v2 protocol JSON, protocol Markdown, preflight
   Markdown, and authorization JSON, all four add-only with no extra path.

The lifecycle-fix review first reported I1 for an incomplete protocol
declaration and then M1 for a fixture that did not exercise the exact user
authorization binding. After both corrections the final review returned
C0/I0/M0.

Fresh post-fix verification was:

- registrar/runner/analyzer lifecycle suite: 142/142 passed;
- complete five-module campaign suite: 171/171 passed;
- complete discovery: 1,160 total = 1,147 passed + the unchanged 5 failures
  + unchanged 8 errors, with 0 skipped;
- all three amended production scripts
  (`register_qualified_closure_campaign.py`,
  `run_qualified_closure_campaign.py`, and
  `analyze_qualified_closure_campaign.py`) passed `py_compile`;
- the implementation diff passed `git diff --check`; and
- all three committed v1 artifact byte counts and hashes remained exact.

The C++ source and binary were unchanged by the lifecycle repair, so the prior
direct result remains 111/111 cases and 1,106/1,106 assertions, CTest remains
1/1, and the configured backend remains `ENABLE_GUROBI:BOOL=ON`.

The subsequent v2 postcommit failure and reviewed v3 recovery are:

7. Commit `857de7ee8dd563fdd5fa15b6860ed54da6758b37` froze the clean
   report/plan/review documentation parent. Commit
   `ae7f68c547eaccfb5c4be0b44a08895ab8beeb36` was its sole direct child and
   added exactly the v2 protocol JSON/Markdown, independent preflight, and
   authorization, with no extra path. Their SHA-256 identities are
   `fd138abf50c95db5b3d43c602b91508e6c13521dbcabb3354c87bd32b9fc7538`,
   `1102a7a1663678762c126066685e835991b3449a8147c8eb8d7f6c62fc460da0`,
   `c790cd2252e9c73209f7b3d19994233f1997695173e8ca9d6e4ceb2fba07019d`,
   and `aa0756887dbdf0d24f6d81bc845986c98713964844cbf4434cd65389cc2d9179`.
8. Mandatory production postcommit validation accepted that topology and
   those byte bindings, then failed because it attempted
   `git show 857de7e:build-diagnostic/Swarm`. The registered binary is
   deliberately untracked and therefore has no parent-commit blob. Neither v2
   root was claimed and no runner, analyzer, mission, or empirical computation
   ran.
9. Commit `02f0f13ce1b3803b84840221221694ab02d2441d` archived the
   3,711-byte / 91-line failure report at
   `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-postcommit-validation.md`,
   SHA-256
   `815988e3149d6adcffb61d323e242905ae3e89a145eb37560abf1b47de9fb235`.
   Development-v2 is consumed and non-executable; its protocol, preflight,
   authorization, failure report, and commits are immutable evidence and may
   not be retried, overwritten, amended, or reinterpreted.
10. Commit `64f64d00a1fd6b7ab01760e340eca883be58f338` (tree
    `b2c7cc555e5ee6c684ac6fb2eb14d44157c5f9dd`) changes exactly the registrar,
    runner, analyzer, and their three test modules. Only exact path tokens in
    `repository.allowed_untracked_paths` bypass registered-parent Git-blob
    lookup; each still must match exact live/status/path/hash/size and build or
    dependency bindings. Every other source, configuration, report, review,
    and tool identity remains verified from the registered parent blob. The
    analyzer independently derives the full trusted development-v3,
    confirmatory-v1, or confirmatory-smoke schedule envelope before allocation.
11. The genuine-untracked Git regression and three campaign-tamper regressions
    completed their RED/GREEN cycles; the tamper group passed 3/3, the focused
    five-module suite passed 173/173, all three scripts passed `py_compile`,
    the diff passed `git diff --check`, and full discovery was 1,162 total =
    1,149 passed + the unchanged 5 failures + unchanged 8 errors, with 0
    skipped. Independent final review returned C0/I0/M0.

Development now requires version `v3` and roots
`/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v3` and
`/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v3`;
confirmatory remains version `v1`. This report, the updated plan, and the
future independent recovery review must first be one clean documentation
closure after `64f64d0...`. The registrar will bind that future exact HEAD,
not the code-fix commit directly. The authorization boundary remains the
registered protocol SHA-256, that future implementation/docs identity, exact
preflight SHA-256, date `2026-08-02`, and exact user text `批准` with UTF-8
SHA-256
`8cbe697b157364a5b13646285b38409dc53ec5287deeb7913493e65b275cd14d`.
No development-v3 authorization has been created or granted.

## Fresh C++ verification

The build and backend were freshly configured and verified with:

```bash
cmake -S . -B build-diagnostic -DBUILD_TESTING=ON
rg '^ENABLE_GUROBI:BOOL=ON$' build-diagnostic/CMakeCache.txt
cmake --build build-diagnostic -j2
```

Configuration and build returned 0. CMake reported Release mode and Gurobi,
HiGHS, and OSQP enabled; the required cache check returned exactly
`ENABLE_GUROBI:BOOL=ON`.

Every required binary was then executed explicitly, because only the Swarm
failure-exit target is registered with CTest:

| Command | Cases | Assertions | Result |
|---|---:|---:|---|
| `./build-diagnostic/testFimRateCertificate` | 11 | 213 | pass |
| `./build-diagnostic/testBarrierEdgeRegistry` | 8 | 188 | pass |
| `./build-diagnostic/testAllocatedPairwiseCBF` | 11 | 55 | pass |
| `./build-diagnostic/testHybridCertificateGuard` | 18 | 147 | pass |
| `./build-diagnostic/testEvidenceStream` | 4 | 27 | pass |
| `./build-diagnostic/testRobustConstraintConstruction` | 34 | 161 | pass |
| `./build-diagnostic/testRobotDiagnostics` | 9 | 46 | pass |
| `./build-diagnostic/testDiagnosticConfiguration` | 5 | 29 | pass |
| `./build-diagnostic/testSwarmFailureHandling` | 9 | 235 | pass |
| `./build-diagnostic/testOptimisers` | 2 | 5 | pass |

Direct-binary total: 111 cases and 1,106 assertions, all passed, zero failed
and zero skipped. This includes the five pure-contract executables for FIM
rate, edge registry, allocated pairwise rows, hybrid guard, and evidence
stream.

The final command was:

```bash
ctest --test-dir build-diagnostic --output-on-failure
```

Result: 1/1 (`testSwarmFailureExit`) passed; 0 failures in 0.25 s.

## Build, binary, dependency, and configuration identities

The fresh build identities are:

```text
02f1c30575a04194fe2d459469a36ba0e099c72d2c4979dc29bf2627fef6b63f  build-diagnostic/Swarm
034eab2688f36fa330838513ab597e6ef83283c85cda6310b2aa639a24c9a264  build-diagnostic/CMakeCache.txt
361248f9642ee3c46023f5d5a7253e6c03c7a8cde71eae525bd71d4465aeb3c0  conda list -n cbf_env --explicit stdout
```

The binary is a Mach-O 64-bit arm64 executable. The build used CMake 3.28.3,
Apple clang 21.0.0 (`arm64-apple-darwin25.5.0`), Unix Makefiles, Release mode,
and macOS SDK `MacOSX26.5.sdk`. Its observed `otool -L` identity is:

```text
/Library/gurobi1200/macos_universal2/lib/libgurobi120.dylib (12.0.0)
/opt/homebrew/opt/highs/lib/libhighs.1.dylib (1.13.0)
@rpath/libOsqpEigen.0.11.0.dylib (0.11.0)
/opt/homebrew/opt/osqp/lib/libosqp.dylib (0.0.0)
/usr/lib/libc++.1.dylib (2100.43.0)
/usr/lib/libSystem.B.dylib (1356.0.0)
```

Configuration hashes are:

```text
0920ca21fc57e51d44b0df3c00bc32bf4d9fe5bbc7f6fa8bb826def1b88184ac  config/config.json
6a198e17b8c3bbf6777caaa325fa593dd24ee9ce3981f90769075112be1cacb3  config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json
6ad20e1e32fff5e8950f8de31864d77a238f6d418abafe3a50cdb292b8c0bc66  config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json
```

The two qualified overlays differ at exactly one leaf:
`position_covariance.reference-selection` is `dynamic-lower-index` for the
primary arm and `fixed-cbf-only` for the ablation. Both otherwise freeze the
1 mm primary and 0.5/1/2 mm sensitivity tolerances, deployment domain,
private/public history policy, 0.5 m ranging sigma, analytic-topological
uncertainty rate, allocated pairwise fixed-communication and safety rows,
componentwise planar bound 25, yaw-rate bound 0.35, distributed execution,
and `time-step=0.5` s.

## Static and provenance checks

The required commands and exact outcomes were:

```bash
git diff --check
```

Exit 0, no output.

```bash
rg -n "truth_position|future_estimate|realized_error" \
  scripts/diagnostics/qualified_modes.py \
  scripts/diagnostics/two_range_reacquisition.py
```

The only matches are lines 22, 23, and 25 of `qualified_modes.py`, inside the
`FORBIDDEN_RUNTIME_KEYS` rejection set. That set is consumed by recursive
runtime-payload validation and causes `forbidden runtime qualifier field`;
there is no truth-field read or decision use. There were no matches in
`two_range_reacquisition.py`. These matches therefore prove explicit
rejection rather than truth leakage.

```bash
rg -n "_othersVel|positiveBackwardUncertaintyRate" \
  include/cbf/AllocatedPairwiseCBF.hpp include/cbf/HybridCertificateGuard.hpp
```

Exit 1 with no matches, which is the expected successful absence result:
allocated hard rows and the reset guard contain neither neighbor velocity nor
backward uncertainty differences.

```bash
test ! -e /private/tmp/cbf2026-two-range-reacquisition-analysis/v2
```

Exit 0. Additional read-only immutability checks confirmed that
`/private/tmp/cbf2026-two-range-reacquisition-development/v2` still exists,
contains exactly the two regular files `manifest.json` and
`two-range-reacquisition.jsonl.gz`, and has the required sorted per-file tree
commitment:

```text
57670b00a303c4c655559097d2a80589c4cea663bc88d3033dfe30fb5fbea6cd
```

The new namespace has not reused the historical roots or identities.

## Contract summary and known conservatism

The verified architecture keeps the offline WNLS qualified-mode sidecar out
of controller state, FIM construction, CBF rows, and QPs. Runtime mode
qualification accepts only declared deployment data, current measurements,
published present/past state, the exactly preceding held command, private
history, and frozen configuration. It rejects truth, future, analyzer labels,
and realized errors recursively. All-record clustering precedes qualification,
the two-range case preserves both circle branches, and publication requires
one uniquely qualified global mode.

The controller computes command-independent topological FIM radius-rate
certificates before the current QPs. Canonical fixed-localization and collision
edges receive versioned endpoint allocations; dynamic FIM-only references do
not create CBF rows. Each UAV builds a local hard problem without reading a
neighbor's current command. A topology reset is a versioned all-or-none
transaction over the changed-node descendant closure and commits only after
exact nonnegative post-reset barriers and fresh bounded hard-QP feasibility
for every UAV.

The scientific conservatism and premises remain explicit:

- `epsilon_i = 3 sqrt(lambda_max(P_i))` is a mode-conditional design radius,
  not a deterministic or distribution-free error bound.
- The topological rate certificate uses the component-bound speed envelope
  `sqrt(2)*25` for an sUAV and zero for a hovering base. This is deliberately
  command-independent and can be more conservative than realized motion.
- The analytic rate is segment-local to a fixed active reference graph;
  graph changes are handled by guarded resets, not by a backward-difference
  derivative claim.
- Local numerical-minimum completeness for the multi-reference WNLS remains
  a premise. The deterministic seed enumeration and sensitivity audit do not
  prove that an arbitrary nonlinear solver found every mathematical local
  minimum.
- Pairwise UAV rows use a frozen half allocation at each endpoint, while a
  hovering-base edge assigns the full row to the UAV. Dynamic FIM-only edges
  improve the covariance model but supply no distance-CBF constraint.
- The evidence tolerances (`1e-7` for residual/barrier and input audits,
  `1e-9` for rate dominance, and `1e-12` for allocation sums) are numerical
  audit tolerances only. They do not relax exact analytic premises.
- Yaw and lower-level dynamics are outside the continuous planar theorem.

## Continuous-time theorem versus 2 Hz implementation

The continuous-time claim remains conditional: on each fixed-active-set
segment, the selected mode and covariance/rate premises, exact allocated hard
rows, component bounds, QP feasibility, and locally finite valid resets imply
the stated barrier result. No sampled-data reserve has been introduced.

The implementation runs at 2 Hz (`dt=0.5` s). It provides empirical evidence
at the frozen controller intervals for certificate availability, reconstructed
local/full residuals, rate inequalities, commands, reset integrity, and true
distances. Its at-most-one-transaction-per-frame anti-chatter check is an
implementation property, not a continuous no-Zeno proof. Passing sampled rows
would not, by itself, establish continuous-time invariance between samples.

## Immutable roots and experiment state

At report time:

```text
PRESENT /private/tmp/cbf2026-two-range-reacquisition-development/v2
ABSENT  /private/tmp/cbf2026-two-range-reacquisition-analysis/v2
ABSENT  /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v1
ABSENT  /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v1
ABSENT  /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v2
ABSENT  /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v2
ABSENT  /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v3
ABSENT  /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v3
ABSENT  /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-a
ABSENT  /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-a-analysis
ABSENT  /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-b
ABSENT  /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-b-analysis
ABSENT  /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory/v1
ABSENT  /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-analysis/v1
```

The development-v1 protocol JSON/Markdown and failed preflight are present and
tracked at `0e4c439...`; their authorization remains absent. The exact four
development-v2 lifecycle artifacts are present and tracked at failed child
`ae7f68c...`, and the postcommit failure report is present at `02f0f13...`:

```text
PRESENT docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.json
PRESENT docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.md
PRESENT docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-preflight.md
PRESENT docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-authorization.json
PRESENT docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-postcommit-validation.md
ABSENT  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2.md
ABSENT  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-review.md
```

All four named v3 lifecycle artifacts are absent:

```text
ABSENT  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.json
ABSENT  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.md
ABSENT  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-preflight.md
ABSENT  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-authorization.json
ABSENT  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3.md
ABSENT  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-review.md
```

Consequently there is no development-v2 or development-v3 empirical result,
no confirmatory result, and no authority in this report to launch a campaign.
The unrelated allowed-untracked `build-diagnostic/` and all historical
evidence remain preserved. This report, the plan, and the future independent
recovery review belong in one prerequisite clean documentation-closure commit
after `64f64d0...`; they must not enter the later exact four-artifact child.

## Exact development-v3 recovery commands (recorded, not executed)

All development-v1 and development-v2 registrar, runner, analyzer, protocol,
authorization, and root commands are terminally superseded. They are retained
in the source plan only as failed history and must not be executed.

The following is the exact current registrar command. It must run only from
the clean final report/plan/review documentation-closure commit after
`64f64d0...`; the registrar, not this report, supplies that future commit's
exact identity. No later working edit may be present or enter the future
artifact-only child:

```bash
conda run -n cbf_env python \
  scripts/diagnostics/register_qualified_closure_campaign.py \
  --kind development --version v3 \
  --implementation-report docs/diagnostics/2026-08-01-cbf2026-qualified-closure-implementation.md \
  --implementation-review docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-implementation-review.md \
  --binary build-diagnostic/Swarm \
  --base-config config/config.json \
  --primary-config config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json \
  --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json \
  --trajectory-seeds 2026080101:2026080110 \
  --range-noise-seeds 2026081101:2026081110 \
  --frames 1000 \
  --raw-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v3 \
  --analysis-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v3 \
  --protocol-json docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.json \
  --protocol-md docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.md
```

After independent C0/I0/M0 preflight, exact authorization construction, and
the exact four-artifact add-only direct-child commit have all succeeded, Task
11 may execute the protocol-bound development-v3 runner exactly once with the
following token sequence:

```bash
conda run -n cbf_env python \
  scripts/diagnostics/run_qualified_closure_campaign.py \
  --kind development \
  --version v3 \
  --protocol docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.json \
  --authorization docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-authorization.json \
  --binary build-diagnostic/Swarm \
  --base-config config/config.json \
  --primary-config config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json \
  --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json \
  --trajectory-seeds 2026080101:2026080110 \
  --range-noise-seeds 2026081101:2026081110 \
  --frames 1000 \
  --output-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v3
```

Only after that immutable raw bundle is terminal may Task 11 run the
protocol-bound analyzer with this exact token sequence:

```bash
conda run -n cbf_env python \
  scripts/diagnostics/analyze_qualified_closure_campaign.py \
  --kind development \
  --version v3 \
  --protocol docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.json \
  --authorization docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-authorization.json \
  --input-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v3 \
  --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json \
  --output-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v3
```

None of these three commands was run during this recovery update. The v3
protocol pair, preflight, authorization, raw root, and analysis root all
remain absent. There is no new empirical result. Any failed v3 gate is
once-only and must retain v3 unchanged; it requires a newly reviewed,
non-colliding development version rather than retry or reinterpretation.

## Development-v3 launch failure and operative development-v4 recovery

This append-only section supersedes the preceding prospective statement that
the v3 lifecycle artifacts were absent or executable. It does not alter the
archived v1/v2/v3 records. The v3 documentation parent was
`70affe063e358a22c2139984429e741ff5a03023`, and its sole direct child
`16c0aca90f578259b16a75659433ad12841ae10e` added exactly the protocol JSON,
protocol Markdown, independent preflight, and authorization. Production
postcommit authorization validation passed. The four immutable v3 hashes are:

```text
8359bc7beffb676208a50f770a43817dbd71fc551dd8687790caecaef49f7c2b  protocol JSON
21f59658bfa7ad051f936e497ff45e583fefa2fa75b0e166db7127ca3d62cab2  protocol Markdown
a810d91cd01f8c0583e05182335300c8ddc81964a270fce72c9e7245c54eef3c  independent preflight
f1f3494e881e37711092be6243252c5ddd46464e526045d9f800101fd8fb0661  authorization JSON
```

The exact registered v3 runner was invoked once. It stopped immediately with
`qualified campaign failed: No module named 'scripts'`. Failure occurred in
the first runtime absolute import during registration validation: direct
script execution placed `scripts/diagnostics`, rather than the repository
root, on `sys.path`. The failure preceded implementation-identity collection,
schedule construction, output-root claiming, every Swarm process, every
measurement/replay producer, and all empirical computation. Both v3 roots
therefore remain absent. Commit `e7a9bde54172f61216475e37b69524e987daf1a6`
archives the exact 3,470-byte, 75-line failure record at
`docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-launch-failure.md`;
its SHA-256 is
`54f78e86c4b1dc9eff1a031a8ed2121caff305afc1892eddc28212fc492a2818`.
Development-v3 is consumed and cannot be rerun or repaired in place.

An independent audit established that repository-cwd module invocation is the
smallest safe correction for top-level runner/analyzer and smoke commands.
The internal replay child remains an absolute direct script because it already
adds the repository root before imports and intentionally executes from an
isolated producer cwd. The TDD repair is commit
`edb9bd00ee00a98510dfe106bf3ed4ef9fede281` with parent
`e7a9bde54172f61216475e37b69524e987daf1a6` and tree
`55cb8e6f6db2998b37d33cabb43f25ac5534d461`. It changes exactly six files:

```text
scripts/diagnostics/analyze_qualified_closure_campaign.py
scripts/diagnostics/register_qualified_closure_campaign.py
scripts/diagnostics/run_qualified_closure_campaign.py
tests/test_analyze_qualified_closure_campaign.py
tests/test_register_qualified_closure_campaign.py
tests/test_run_qualified_closure_campaign.py
```

All protocol-frozen development, confirmatory, and smoke runner/analyzer argv
arrays now use `conda run -n cbf_env python -m scripts.diagnostics...` from
the registered repository cwd. Runtime reconstruction requires the identical
tokens. Development v1/v2/v3 are rejected, only v4 is current, and
confirmatory stays v1. The new roots are exactly:

```text
/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v4
/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v4
```

Seeds `2026080101..2026080110` and `2026081101..2026081110`, 1,000 frames,
500 s horizon, thresholds, scientific schedules, estimator/controller
semantics, authorization topology, no-retry behavior, and the 8 GB launch /
6 GB hard-stop / 2 GB cache policy are unchanged.

The TDD record reproduced both direct-script failures with `PYTHONPATH`
removed and no root creation, then proved the module commands reach a
controlled registration error without `ModuleNotFoundError`. The focused
registrar/runner/analyzer/replay suite passed 166/166; the five-module suite
passed 186/186. Eight inspected source/test files passed `py_compile`, and
`git diff --check` passed. Full discovery reported 1,166 total = 1,153 passed
+ the same 5 legacy failures + the same 8 legacy/external errors, with no new
nonpassing test. Independent source review returned C0/I0/M0. At documentation
time all development v1/v2/v3/v4 raw and analysis roots are absent,
`ENABLE_GUROBI:BOOL=ON`, `build-diagnostic/Swarm` is executable, and about
15.5 million KiB remain free on `/private/tmp`.

The final operative recovery plan is
`docs/superpowers/plans/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-implementation.md`,
178,976 bytes / 3,243 lines, SHA-256
`f3cb2c0c9bc047e1873ca27c2ce56cd5ab5732ade0264c3ce8f90c80b7979fbb`.
It supersedes every earlier command or dependency that calls v3 executable,
and it explicitly replaces Task 12's later v3 dependencies and direct `.py`
examples with development-v4 inputs and exact confirmatory-v1 module commands.

No v4 protocol, preflight, authorization, execution root, result, or review
exists at this report update. After this report, the plan, and the independent
implementation review are committed together, the only exact registrar
command is the plan's development-v4 module command. A later sole direct child
must add exactly the four v4 lifecycle artifacts and pass production
postcommit validation. Only then may the protocol-bound module runner execute
once, followed by the module analyzer once after a terminal raw manifest.
Only a reproduced v4 PASS with C0/I0 may open confirmatory-v1.

## Development-v4 terminal failure and operative development-v5 closure

This append-only section supersedes the preceding prospective v4 statement
without changing the archived v3/v4 chronology. Development-v4 was authorized
and its protocol-bound runner was invoked exactly once. Mission 1 stopped
after the fourteen frame-zero initialization records because the reset guard
proved UAV 3's bounded local hard QP infeasible. The terminal raw bundle and
the subsequently produced terminal analysis bundle are immutable:

```text
raw root       /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v4
raw files      28
raw bytes      12618979
raw tree       9b915ad84aeda2b22aafbd259bf61bbc9515e3b082559bf3e5f217e14af9b5ac
raw manifest   12f71bb00720636ca610ad8d9d380dde7e92147e7bdf57589bb8915c1bf72f71

analysis root      /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v4
analysis files     3
analysis bytes     8639
analysis tree      155b6b9372f91bb7b4b2476581b4270e7a372c680cd8b2ea1847ef9c5766729a
analysis manifest  890174908e5df83e367edd33cb0d83960635cc94ac57fd79fafd553aef40e6f0
```

The failure report and independent audits were archived at
`e178e67bd86fe6ad9b698cfba350b7a0fefaeac8`. The reconstructed maximum
minimum local-QP residual was approximately `-2.243905861 m/s`; this is a
real incompatibility between the v4 random frame-zero geometry, the registered
fixed-half allocation, the rate term, and the componentwise `+-25 m/s` input
box. It is not a solver, row-sign, DAG, or retry failure. Development-v4 is
consumed and must not be deleted, resumed, overwritten, or reinterpreted.

Three scoped TDD commits close the prospective v5 interfaces while preserving
the theorem, controller, input bounds, fixed CBF references, and dynamic
FIM-only localization graph:

1. `2ed6460ea06fb4d9eb9b2b082a077be0731f8d2c` preserves every rejected
   reset-QP witness and propagates the exact rejected stage through controller
   abort and mission terminal evidence.
2. `cb3b8b7b794ca51291116c1d76ac49ca6cb4d0c3` freezes the deterministic
   `cbf2026-v5-initial` family and its complete 100-seed admission audit.
3. `6ac6e088ee5d2c6dbed9ed24c128a3225252de6f` binds that family into the
   registrar, runner, materializers, Swarm stream validator, and independent
   analyzer. Its parent is `cb3b8b7...` and its tree is
   `654fea846f9fe67361842c07a527befa1fc2d3e2`.

Commit `6ac6e08...` changes exactly the registrar, runner, analyzer, and their
three test modules. Development is now protocol schema v2, protocol version
v5, and campaign `development-v5`. The trajectory seeds are exactly
`2026080201..2026080210`; range-noise seeds are exactly
`2026081201..2026081210`. The canonical initial-family binding is
`config/diagnostics/qualified_initial_family_v1.json`, whose file SHA-256 is
`7fb8597cd89a41315cbd86ca87c0bc6e3ecb22ea3dcab8ee6278aca44743fe24`
and semantic SHA-256 is
`4f8070127c86149de8ebde10197e5c7a639814e03dac7d56ab17befc6d5bdb8b`.

Every development lifecycle independently reloads strict duplicate-safe JSON,
checks the literal regular-file path and byte identity, recomputes the full
100-seed family before root allocation, and binds the exact registered-seed
position hashes into the top-level `initial_state` and ten mission records.
No clamp, retry, substitution, or resampling path exists. Primary and
fixed-FIM materializations replace the complete base position generator with
the same fourteen `method=specified` positions. The runner and analyzer both
compare all fourteen initialization truth positions and all fourteen frame-0
controller truth positions against the frozen seed record. The analyzer also
authenticates all three materialized configs from one immutable byte snapshot
each and closes the measurement-manifest, runtime-measurement, primary-config,
and Swarm-config SHA chain.

Development v1--v3 must remain absent, and both v4 trees are verified against
the exact identities above before registration or execution can proceed.
Confirmatory remains protocol v1 and does not consume the current development
family. Its fixed-FIM arm retains the pre-v5 base initial configuration rather
than reading a nonexistent `initial_family` binding. The exact-four
protocol/Markdown/preflight/authorization topology is unchanged.

The final implementation review first found one Critical confirmatory-path
failure, two Important integrity gaps, and two Minor maintainability/test-entry
issues. Direct RED tests reproduced the confirmatory `KeyError`, an external
same-byte self-rehashed family substitution accepted at verify time, and a
hash/stat/parse time-of-check-time-of-use replacement. The final code rejects
all three, leaves one `unittest.main()` per test module, and preserves the
development and confirmatory contracts. Non-author cross-review returned
Critical 0 / Important 0; the Standards axis retains one Fowler-judgement
Minor because the initial-state projection is duplicated across three
lifecycle modules.

Fresh pre-commit and post-commit integration verification each passed all
206 selected tests. The post-commit run completed in `81.247 s`; its real
100-seed audit took `11.605 s`. The four production modules passed
`py_compile`, `git diff --check` passed, and the exact v4 predecessor verifier
passed. The source porcelain after `6ac6e08...` was only the preserved
`?? build-diagnostic/`. Both prospective v5 roots remained absent, and no
registrar, runner, analyzer, Swarm, measurement producer, replay, empirical
gate, paper edit, submission, push, or new scientific claim occurred.

The next permitted action is a clean documentation-closure commit containing
this report and its appended independent review. The registrar must bind that
future exact documentation HEAD, not `6ac6e08...` directly. Only after that
commit may the development-v5 protocol pair be generated. Independent
preflight, exact authorization, the sole exact-four add-only child, and
production postcommit validation are still mandatory before a single runner
invocation. This implementation report is not a protocol, preflight,
authorization, result, or permission to execute. Confirmatory and paper claims
remain closed.

The operative future v5 paths are:

```text
protocol JSON  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v5-protocol.json
protocol MD    docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v5-protocol.md
preflight      docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v5-preflight.md
authorization  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v5-authorization.json
raw root       /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v5
analysis root  /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v5
```
