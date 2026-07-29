# CBF2026 Initialization-Persistence Gate 1 Run Registration

## Registration status

This document freezes one retrospective, read-only Gate 1 analysis.
The registered production invocation has not been executed.
It may be invoked exactly once only after the full test suite passes,
the tracked worktree is clean,
the disk gates pass,
and an independent code/protocol review records no Critical or Important
finding.

This registration does not authorize an estimator, graph, trajectory, noise,
controller, CBF, paper, source bundle, output path, category, threshold, or
retry change.

## Registered source and invocation

| Field | Frozen value |
| --- | --- |
| Worktree | `/private/tmp/cbf2026-diagnostic` |
| Analyzer source commit | `8ca4e08f279465f09f314a794682761b89514341` |
| Analyzer module | `scripts.diagnostics.analyze_initialization_persistence` |
| Analyzer file | `scripts/diagnostics/analyze_initialization_persistence.py` |
| Analyzer SHA-256 | `388685805985446647e152ed981f8ac442ce1e026f40984c9d1a289b1305fb46` |
| Conda environment | `cbf_env` |
| Output root | `/private/tmp/cbf2026-initialization-persistence` |
| Final output directory | `/private/tmp/cbf2026-initialization-persistence/registered-gate-1` |
| Staging directory | `/private/tmp/cbf2026-initialization-persistence/registered-gate-1.incomplete` |

The sole registered production command is:

```bash
conda run -n cbf_env python -m \
  scripts.diagnostics.analyze_initialization_persistence \
  --bundle-dir /private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288 \
  --output-dir /private/tmp/cbf2026-initialization-persistence/registered-gate-1
```

It must be launched at most once.
No automatic or manual retry is permitted under this registration.
If that invocation raises an error or leaves any terminal evidence,
record the failure and stop before Task 3.
The analyzer may publish only `initialization-persistence.json` and
`initialization-persistence.md` after all its integrity checks pass.

## Immutable source bundle

The sole source bundle is:

```text
/private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288
```

The bundle manifest is SHA-256
`39ee3b5494ed623d20d16766244427476cd081ed0869feecb9e5072c6c8f481d`.
Its four artifact hashes are frozen below;
the analyzer verifies the compressed and decompressed process stream as well
as both summaries before processing and re-verifies unchanged inputs before
publication.

| Source artifact | SHA-256 |
| --- | --- |
| `calibration.jsonl.gz` compressed bytes | `38325523e395983d83491c81556bac1aae72ebd99508b3cdf148fdf72f3f8803` |
| Decompressed `calibration.jsonl` stream bytes | `2c8a3f0f5684a281d5fbef3d7facea16f45d757d96b458827928dcac9e241a4f` |
| `summary.json` | `f4f780dd4c9c1399d3f838c74054c9c0ba3159415bc13089aee53d0ebf7250bc` |
| `summary.md` | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |

The manifest must state `termination_reason == "completed"` and the estimator
contract `variable_weight_nls_full_residual_jacobian_v1`.

## Frozen protocol and decision rule

| Field | Frozen value |
| --- | --- |
| Schema | `cbf2026-initialization-persistence-v1` |
| Exact reason | `non-finite or malformed WNLS input` |
| Graph cases | `dynamic_dag_wnls`, `fixed_refs_wnls` |
| Dominance threshold | at least 95% separately in each graph case |
| Compound threshold | zero `prior_missing_and_recorded_measurement_invalid` rows separately in each graph case |

Every row whose `attempt_status == "invalid"` and whose
`attempt_failure_reason` equals the exact reason belongs to exactly one of
these mutually exclusive categories:

| Category | Frozen predicate |
| --- | --- |
| `exclusive_prior_self_estimate_missing_or_malformed` | The same `(seed, graph_case, robot_id)` preceding row has no finite 2-D estimate and every recorded noisy range is finite. |
| `prior_missing_and_recorded_measurement_invalid` | The preceding estimate is missing or malformed and a recorded noisy range is missing or non-finite. |
| `recorded_measurement_missing_or_nonfinite` | The preceding estimate is finite and a recorded noisy range is missing or non-finite. |
| `unresolved_compound_validator` | None of the preceding predicates is supported by recorded fields. |

Gate 1 passes only when, separately for both graph cases, every exact-reason
row is classified once, exact-reason plus other non-upstream invalid rows
reconciles to the completed-audit `invalid_input_or_numeric` total, the
exclusive-prior fraction is at least 0.95, the compound-prior-and-measurement
count is zero, and no source-integrity, ordering, reconciliation,
non-finite-JSON, disk-limit, or independent-review failure occurs.

## Frozen resource and launch gates

The registered limits are:

| Gate | Limit |
| --- | ---: |
| Available bytes before launch | at least `8,000,000,000` |
| Live available-byte floor | `6,000,000,000` |
| Allocated bytes under output root | at most `2,000,000,000` |
| Allocated bytes for this Gate 1 run | at most `10,000,000` |

Before the one invocation, record successful completion of:

```bash
conda run -n cbf_env python -m unittest discover -s tests -p 'test_*.py' -v
git diff --check
df -Pk /private/tmp
du -sk /Users/xirhxq/.cache/codex-runtimes 2>/dev/null
```

The launch is prohibited unless the tests pass, `git status --porcelain -uno`
is empty, available bytes meet the registered pre-launch threshold, and the
independent review has no Critical or Important finding.
Untracked `build-diagnostic/` is excluded from the tracked-status test and is
never to be staged, deleted, or altered by this run.
