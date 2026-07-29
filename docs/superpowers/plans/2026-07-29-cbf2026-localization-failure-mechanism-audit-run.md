# CBF2026 Localization Failure-Mechanism Audit Run Registration

## Registration status

This document freezes one post-hoc exploratory mechanism audit.
The registered production analysis has not been executed.
Execution is permitted only after the independent analyzer review records a
`CLEAN` verdict and all pre-registration verification and Task 6 launch gates
pass.

This registration does not authorize a changed estimator,
graph,
epsilon definition,
controller,
trajectory,
noise realization,
taxonomy,
bin,
threshold,
example cap,
output path,
or retry.

## Registered source and output

The sole production source bundle is:

```text
/private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288
```

The sole final output directory is:

```text
/private/tmp/cbf2026-localization-failure-analysis/20260729T045931Z_d7c1cdcbf6434b6ca944ffacec262f4e
```

Its sibling staging directory is:

```text
/private/tmp/cbf2026-localization-failure-analysis/20260729T045931Z_d7c1cdcbf6434b6ca944ffacec262f4e.incomplete
```

The source bundle,
final output directory,
and staging directory must remain distinct.
The analyzer may write only:

```text
failure-mechanisms.json
failure-mechanisms.md
```

## Registered executable

| Field | Frozen value |
| --- | --- |
| Worktree | `/private/tmp/cbf2026-diagnostic` |
| Branch | `codex/cbf2026-diagnostic` |
| Module | `scripts.diagnostics.analyze_localization_failures` |
| Source file | `scripts/diagnostics/analyze_localization_failures.py` |
| Executable commit | `7e44ab4ad445d925c8a39062e9701cb093b2cd99` |
| Executable source SHA-256 | `1e3a7b7cd615b258fb83af53f7264d7aff327685f42c85787d0d9ae7df0b6315` |
| Conda environment | `cbf_env` |
| Python | `3.11.12` |
| NumPy | `1.24.4` |

Task 6 must verify before any operational setup or production invocation
that:

```bash
git log -1 --format=%H -- scripts/diagnostics/analyze_localization_failures.py
```

equals the registered executable commit,
that no later commit changes the executable or its tests,
and that the executable SHA-256 still matches the registered value.

## Frozen source hashes

The five source identities are:

| Source | SHA-256 |
| --- | --- |
| `calibration.jsonl.gz` compressed bytes | `38325523e395983d83491c81556bac1aae72ebd99508b3cdf148fdf72f3f8803` |
| Decompressed JSONL stream bytes | `2c8a3f0f5684a281d5fbef3d7facea16f45d757d96b458827928dcac9e241a4f` |
| `summary.json` | `f4f780dd4c9c1399d3f838c74054c9c0ba3159415bc13089aee53d0ebf7250bc` |
| `summary.md` | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |
| `manifest.json` | `39ee3b5494ed623d20d16766244427476cd081ed0869feecb9e5072c6c8f481d` |

All five must match before the setup command and remain unchanged after the
analyzer exits.
A mismatch is an integrity failure and prohibits a production retry.

## Frozen input contract

The analyzer requires:

- `termination_reason == "completed"`;
- estimator contract
  `variable_weight_nls_full_residual_jacobian_v1`;
- `process_rows == expected_process_rows == 280000`;
- exactly 279,440 primary rows;
- 20 paired range-noise seeds;
- graph cases `dynamic_dag_wnls` and `fixed_refs_wnls`;
- 500 frames including frame zero and 499 primary frames;
- 14 UAVs; and
- canonical stream order
  `frame_index -> seed -> graph_case -> robot_id`.

Every expected key must appear exactly once.
Primary and initialization attempt-status and retained-status counts must
separately reconcile to the frozen summary.
The exact run settings remain bound by the registered manifest and summary
hashes and are reproduced in the derived JSON `source.settings`.

## Frozen output schema

The schema identifier is:

```text
cbf2026-localization-failure-analysis-v1
```

The strict JSON top level contains:

```text
schema
status
source
integrity
protocol
initialization
cases
comparisons
bootstrap
limitations
```

Serialization is deterministic,
uses sorted keys and finite standard JSON values,
and ends with one newline.

## Frozen classification

Classification uses the current `attempt_status`,
never the retained `status`.
Every primary row belongs to exactly one class:

| Class | Frozen predicate |
| --- | --- |
| `contained` | `attempt_status == "converged"` and `containment is true` |
| `converged_outside_radius` | `attempt_status == "converged"` and `containment is false` |
| `upstream_unavailable` | `attempt_status == "invalid"` and `attempt_failure_reason == "invalid upstream UAV reference"` |
| `invalid_input_or_numeric` | `attempt_status == "invalid"` and any other reason |
| `wnls_nonconvergence` | `attempt_status == "failed"` and `attempt_failure_reason == "maximum WNLS iterations exceeded"` |
| `other_failed` | `attempt_status == "failed"` and any other reason |

The six counts must sum to their denominator for every graph case,
depth,
time bin,
dynamic depth/time cell,
and seed.

At most 32 distinct raw reason labels are retained.
Additional labels increment the overflow count,
and at most five overflow examples are retained because the registered
`max_examples_per_bucket` is five.

## Frozen bins

All bounds and closure rules are fixed.
`null` denotes an unbounded numeric endpoint in the JSON protocol.

### Error-to-epsilon ratio

| Label | Lower | Upper | Lower closed | Upper closed |
| --- | ---: | ---: | :---: | :---: |
| `[0,0.5)` | 0 | 0.5 | yes | no |
| `[0.5,1)` | 0.5 | 1 | yes | no |
| `[1,2)` | 1 | 2 | yes | no |
| `[2,5)` | 2 | 5 | yes | no |
| `[5,infinity)` | 5 | unbounded | yes | no |

### Normalized squared error

| Label | Lower | Upper | Lower closed | Upper closed |
| --- | ---: | ---: | :---: | :---: |
| `[0,2.295748929)` | 0 | 2.295748929 | yes | no |
| `[2.295748929,5.991464547)` | 2.295748929 | 5.991464547 | yes | no |
| `[5.991464547,9]` | 5.991464547 | 9 | yes | yes |
| `(9,infinity)` | 9 | unbounded | no | no |

The tail metrics are exactly \(q>5.991464547\) and \(q>9\).
Covariance must be finite,
symmetric within `1e-12` relative tolerance,
and positive definite;
otherwise the converged row is excluded from the \(q\) denominator and
counted as conditional-covariance-invalid.

### FIM condition number

| Label | Lower | Upper | Lower closed | Upper closed |
| --- | ---: | ---: | :---: | :---: |
| `[1,10)` | 1 | 10 | yes | no |
| `[10,30)` | 10 | 30 | yes | no |
| `[30,100)` | 30 | 100 | yes | no |
| `[100,infinity)` | 100 | unbounded | yes | no |

### FIM minimum eigenvalue

| Label | Lower | Upper | Lower closed | Upper closed |
| --- | ---: | ---: | :---: | :---: |
| `(0,0.05)` | 0 | 0.05 | no | no |
| `[0.05,0.2)` | 0.05 | 0.2 | yes | no |
| `[0.2,1)` | 0.2 | 1 | yes | no |
| `[1,infinity)` | 1 | unbounded | yes | no |

### Depth, reference count, and time

- Squad-local depth is an integer from 1 through 7,
  inclusive.
- Active-reference count is
  `len(active_references.base_ids) + len(active_references.uav_ids)`.
  The bins are singleton values 0 through 9 and `10_or_more`,
  defined as the lower-closed range from 10 to unbounded.
- Frame-index bins are the inclusive integer ranges 1--100,
  101--200,
  201--300,
  301--400,
  and 401--499.
- The dynamic graph additionally reports the complete 7-by-5 depth/time
  table.

All frozen bins are emitted,
including zero-count bins.

## Frozen bootstrap

The descriptive paired bootstrap for
\(D_{\mathrm{upstream}}\) is frozen as follows:

| Field | Frozen value |
| --- | --- |
| Resampling unit | paired seed record |
| Paired seed records | 20 |
| Draws per resample | 20 |
| Sampling | with replacement |
| Aggregate counts | `dyn_up`, `fix_up`, `dyn_invalid`, `fix_invalid` |
| Resamples | 10,000 |
| RNG seed | `20260729` |
| Percentiles | 2.5 and 97.5 |
| Confidence level | 0.95 |
| Minimum estimable resamples | 9,500 |
| P-value | not reported |

The ratio is recomputed after summing the four counts in each paired
resample:

\[
D_{\mathrm{upstream}} =
\frac{\sum N^{\mathrm{dyn}}_{\mathrm{upstream}}-
      \sum N^{\mathrm{fix}}_{\mathrm{upstream}}}
     {\sum N^{\mathrm{dyn}}_{\mathrm{invalid}}-
      \sum N^{\mathrm{fix}}_{\mathrm{invalid}}}.
\]

The point estimate,
each seed-specific estimate,
and every bootstrap resample are `not_estimable` or `null` when the
corresponding denominator is non-positive.
A percentile interval is emitted only when at least 9,500 resamples are
estimable.

## Frozen denominators

The JSON protocol records these exact denominator rules:

| Field | Frozen rule |
| --- | --- |
| `overall` | all `primary_statistics` rows for the graph case |
| `depth` | all `primary_statistics` rows in the squad-local depth bin |
| `time` | all `primary_statistics` rows in the frame-index time bin |
| `dynamic_depth_time_cell` | all `dynamic_dag_wnls` `primary_statistics` rows in the depth and time cell |
| `seed` | all `primary_statistics` rows for the graph case and seed |
| `initialization` | all frame-zero rows for the graph case and depth |
| `containment` | converged rows in the reported graph case or stratum |
| `q` | converged rows with finite, symmetric, positive-definite covariance and finite normalized squared error |
| `ratio` | converged rows with finite non-negative error-to-epsilon ratio |
| `persistence` | one record per observed `(seed, graph_case, robot_id)` primary sequence |
| `lineage` | `upstream_unavailable` rows and their unavailable UAV-reference edges |
| `d_upstream` | `sum(dyn_invalid) - sum(fix_invalid)`; `not_estimable` when non-positive |

Frame-zero initialization is reported separately from primary statistics.
Lineage counts distinguish observer rows from unavailable UAV-reference
edges and use only already observed same-frame predecessors.

## Frozen persistence records

For each observed `(seed, graph_case, robot_id)` primary sequence,
the analyzer reports:

- first primary frame with a converged attempt;
- number of primary frames before first convergence;
- longest consecutive upstream-unavailable streak;
- longest consecutive WNLS-nonconvergence streak; and
- whether primary convergence never occurs.

These are descriptions,
not causal evidence about initialization,
geometry,
or noise.

## Frozen limitations

The JSON and Markdown contain exactly these six limitations:

1. post-hoc exploratory mechanism audit;
2. one preserved trajectory with 20 paired noise seeds;
3. offline estimator outside the controller;
4. no shared-ancestor cross-covariance model;
5. no causal estimator or graph comparison; and
6. no radius,
   mission-probability,
   robustness,
   or safety validation.

## Operational limits

| Limit | Frozen value |
| --- | ---: |
| Persistent-output launch free space | at least 8,000,000,000 bytes |
| Live free-space floor | at least 6,000,000,000 bytes |
| Complete derived-analysis root allocation | at most 2,000,000,000 bytes |
| One staging/run allocation | at most 10,000,000 bytes |
| Live-space check cadence | every 10,000 streamed rows |
| Raw reason labels | at most 32 |
| Overflow examples | at most 5 |
| `max_examples_per_bucket` | 5 |

No decompressed JSONL copy,
pandas data frame,
raw-row list,
external sort,
or copied evidence is permitted.

## Pre-registration gates

Before this registration and the independent review are committed:

1. the independent analyzer review must record `CLEAN`;
2. Critical and Important findings must both be absent;
3. the executable commit and source SHA-256 must match this registration;
4. the owned analyzer and tests must be clean at that commit; and
5. the following commands must pass:

```bash
conda run -n cbf_env python -m unittest discover -s tests -p 'test_*.py' -v
conda run -n cbf_env python -m py_compile \
  scripts/diagnostics/analyze_localization_failures.py \
  tests/test_analyze_localization_failures.py
git diff --check
```

These are verification commands,
not production analysis.

## Task 6 launch and setup sequence

Task 6 must perform this sequence exactly.

### 1. Non-mutating launch probes

Before creating any output directory,
verify:

- available bytes are at least 8,000,000,000;
- all five source hashes match this registration;
- executable commit and source SHA-256 match this registration;
- the registered output parent does not exist;
- the registered final output directory does not exist;
- the registered `.incomplete` sibling does not exist; and
- the prospective derived-analysis root is below its
  2,000,000,000-byte allocation cap.

If free space is below the launch gate,
only the separately authorized exact rebuildable cache
`/Users/xirhxq/.cache/codex-runtimes` may be measured and removed.
If that cleanup does not restore the launch gate,
stop before setup.
No evidence,
user documents,
or Git history may be deleted,
moved,
or overwritten.

### 2. Create the registered empty output parent

Only after every non-mutating launch probe passes,
verify again that the parent is absent,
then execute this setup command exactly once:

```bash
mkdir -m 700 /private/tmp/cbf2026-localization-failure-analysis
```

This command is an operational setup action,
not an analyzer invocation or evidence transformation.
Afterward,
verify that the parent:

- has the exact registered path;
- is empty;
- has mode `0700`; and
- remains below the 2,000,000,000-byte root cap.

If the parent already exists,
the setup command fails,
or any post-setup check fails,
stop before production execution.
Do not substitute another parent,
change permissions,
or retry setup.

### 3. Run the sole production command

Run this exact command once,
without added flags,
redirection,
retry,
or parameter change:

```bash
conda run -n cbf_env python -m scripts.diagnostics.analyze_localization_failures \
  --bundle-dir /private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288 \
  --output-dir /private/tmp/cbf2026-localization-failure-analysis/20260729T045931Z_d7c1cdcbf6434b6ca944ffacec262f4e \
  --max-examples-per-bucket 5
```

The setup command does not consume the one analyzer invocation.
The analyzer command above is the sole production analysis invocation.

## Stop rules and one-run policy

Stop before setup if:

- the independent review is not `CLEAN`;
- pre-registration verification is not clean;
- executable commit or SHA-256 differs;
- any source hash differs;
- launch free space is below 8,000,000,000 bytes after the one authorized
  cache-cleanup option;
- the registered output parent,
  final directory,
  or `.incomplete` sibling already exists; or
- any other launch probe fails.

Stop after setup but before analysis if:

- setup fails;
- the created parent is not empty,
  is not mode `0700`,
  or has the wrong path;
- root allocation reaches or exceeds the registered cap; or
- the final directory or `.incomplete` sibling appears.

During analysis,
stop on:

- missing,
  malformed,
  nonterminal,
  or hash-mismatched source metadata;
- estimator-contract mismatch;
- non-strict JSON,
  compressed/decompressed hash mismatch,
  row-count mismatch,
  noncanonical,
  duplicate,
  or missing keys;
- summary-count mismatch;
- any six-class,
  depth,
  time,
  seed,
  dynamic-cell,
  calibration,
  or lineage reconciliation failure;
- live free space below 6,000,000,000 bytes;
- derived-root allocation above 2,000,000,000 bytes;
- staging/run allocation above 10,000,000 bytes; or
- any source hash change before completed output registration.

If the analyzer command fails:

- preserve the `.incomplete` directory and all failure evidence;
- stop production execution;
- do not invoke the analyzer again;
- do not overwrite or delete failed output;
- do not change the output path in this task;
- do not change bins,
  categories,
  thresholds,
  examples,
  flags,
  or command arguments; and
- do not interpret failure as permission to change the estimator,
  graph,
  epsilon,
  controller,
  or trajectory.

A software defect requires a regression test,
an explicit defect record,
a reviewed executable commit,
a new registration,
and a new derived-analysis run ID.
There is no result-dependent retry,
taxonomy change,
bin change,
threshold change,
or estimator change in this registered audit.
