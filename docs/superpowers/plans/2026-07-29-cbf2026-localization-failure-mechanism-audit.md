# CBF2026 Localization Failure-Mechanism Audit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and run one immutable-input, streaming, post-hoc analyzer that decomposes the corrected Stage 2 localization-calibration failure budget without rerunning WNLS, the controller, or Monte Carlo.

**Architecture:** A new standalone Python module verifies the completed bundle, streams strict gzip JSONL in canonical order, and accumulates bounded failure, lineage, initialization, calibration, and stratified counters. It writes a deterministic compact JSON/Markdown analysis outside the evidence bundle; a separately committed run registration freezes the one production invocation before execution.

**Tech Stack:** Python 3.11.12 in conda environment `cbf_env`, NumPy 1.24.4, standard-library `gzip`, `hashlib`, `json`, `argparse`, and `unittest`.

## Global Constraints

- Code,
  tests,
  plans,
  and diagnostic reports are changed only in
  `/private/tmp/cbf2026-diagnostic`,
  branch `codex/cbf2026-diagnostic`;
  it is already an isolated linked worktree.
- Task 6 updates the separate DRA repository
  `/Users/xirhxq/Documents/Clones/doctoral-research-agent`
  on branch `main`,
  stages only the five absolute CBF2026 paths named in that task,
  and preserves all unrelated PodSearch2026/JFR worktree changes.
- Do not modify or invoke `solve_wnls`, `fim_radius`, `active_references`, `replay_calibration`, the controller, or Monte Carlo.
- Do not change the estimator, dynamic DAG, fixed CBF references, noise, epsilon definition, radius multiplier, or any existing bundle.
- Treat the audit as post-hoc exploratory mechanism analysis, not validation of the radius, graph, probability, robustness, or safety.
- The sole production input bundle is `/private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288`.
- Preserve source hashes: gzip `38325523e395983d83491c81556bac1aae72ebd99508b3cdf148fdf72f3f8803`, decompressed stream `2c8a3f0f5684a281d5fbef3d7facea16f45d757d96b458827928dcac9e241a4f`, summary JSON `f4f780dd4c9c1399d3f838c74054c9c0ba3159415bc13089aee53d0ebf7250bc`, summary Markdown `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9`, and manifest `39ee3b5494ed623d20d16766244427476cd081ed0869feecb9e5072c6c8f481d`.
- Persistent analysis launch requires at least 8,000,000,000 free bytes; live free space must remain at least 6,000,000,000 bytes.
- The complete derived-analysis root is capped at 2,000,000,000 allocated bytes and the one run at 10,000,000 allocated bytes.
- No decompressed JSONL copy, pandas frame, raw-row list, external sort, or copied evidence is allowed.
- Use test-first RED–GREEN–REFACTOR for every production behavior.
- Exact source files are staged explicitly; do not stage or delete the intentionally untracked `build-diagnostic/`.
- The user has already authorized scoped commits and exact rebuildable-cache
  cleanup for this continuing task;
  no agent may broaden that authority to pushing,
  merging,
  deleting evidence,
  or cleaning unnamed paths.
- Commit messages use `type(scope): description` and contain no `Co-Authored-By`.
- Keep every failed derived-analysis output if a production defect is found; do not overwrite or silently rerun it.

---

### Task 1: Immutable bundle verification and canonical streaming

**Files:**
- Create: `scripts/diagnostics/analyze_localization_failures.py`
- Create: `tests/test_analyze_localization_failures.py`

**Interfaces:**
- Produces: `InputIntegrityError`, `AnalysisLimitError`, `analyze_localization_failures(bundle_dir: Path, *, verify_hashes: bool = True, output_dir: Path | None = None, max_examples_per_bucket: int = 5) -> dict`.
- Produces: private `_iter_verified_rows(bundle_dir: Path, manifest: dict, summary: dict)` yielding one strict row at a time after compressed preflight.
- Consumes later: Tasks 2–4 add bounded accumulators and output to the verified stream.

- [ ] **Step 1: Write the completed-bundle fixture and failing acceptance test**

Create a test helper that writes four canonical rows for:

```python
seeds = [17]
graph_cases = ["dynamic_dag_wnls", "fixed_refs_wnls"]
effective_frame_count = 1
robot_ids = [1, 2]
```

The helper writes strict JSONL with `gzip.GzipFile(..., mtime=0)`,
computes compressed/decompressed hashes,
writes `summary.json`,
`summary.md`,
and a terminal manifest with both summary hashes.

Add:

```python
def test_accepts_complete_bundle_and_reports_exact_source_identity(self):
    report = analyze_localization_failures(self.bundle)
    self.assertEqual(report["schema"], "cbf2026-localization-failure-analysis-v1")
    self.assertEqual(report["status"], "completed")
    self.assertEqual(report["integrity"]["observed_rows"], 4)
    self.assertEqual(report["integrity"]["primary_rows"], 0)
    self.assertTrue(report["integrity"]["hashes_match"])
```

The production mutation caught is accepting an unverified or incomplete source.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_localization_failures.LocalizationFailureInputTests.test_accepts_complete_bundle_and_reports_exact_source_identity -v
```

Expected: import failure because `scripts.diagnostics.analyze_localization_failures` does not exist.

- [ ] **Step 3: Implement the smallest strict loader and stream**

Implement these exact constants and exceptions:

```python
SCHEMA_ID = "cbf2026-localization-failure-analysis-v1"
ESTIMATOR_CONTRACT_ID = "variable_weight_nls_full_residual_jacobian_v1"

class InputIntegrityError(RuntimeError):
    pass

class AnalysisLimitError(RuntimeError):
    pass
```

Use:

```python
def _strict_json_line(raw: bytes) -> dict:
    value = json.loads(
        raw,
        parse_constant=lambda token: (_ for _ in ()).throw(
            ValueError(f"non-finite JSON token: {token}")
        ),
    )
    if not isinstance(value, dict):
        raise InputIntegrityError("every process line must be a JSON object")
    return value
```

Hash the compressed file with fixed-size reads.
During `gzip.open(process_path, "rb")`,
update the decompressed digest from each exact raw line before parsing.
Advance a canonical expected-key cursor in:

```text
frame -> seed -> graph case -> robot_id
```

Derive robot count as:

```python
expected_rows // (
    effective_frame_count * len(run_seeds) * len(graph_cases)
)
```

Require robot IDs `1..robot_count`.
Do not retain a key set.

- [ ] **Step 4: Add failing corruption and ordering tests**

Add separate tests that mutate:

- one gzip byte;
- the decompressed hash in the manifest;
- `summary.json` after its manifest hash is recorded;
- `summary.md` after its manifest hash is recorded;
- estimator contract;
- termination reason;
- one row key;
- one duplicate row;
- the summary expected row count; and
- primary `overall` attempt counts;
- primary `overall` retained counts;
- `initialization_frame` attempt counts;
- `initialization_frame` retained counts; and
- one `NaN` JSON token.

Each must raise `InputIntegrityError`.
The duplicate-row test must fail specifically at the canonical expected-key comparison,
not from a retained key set.
Add malformed-summary cases where the expected row count is not divisible by
`effective_frame_count * seed_count * graph_case_count`,
or produces a non-positive robot count.

- [ ] **Step 5: Run RED, implement validation, and verify GREEN**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_localization_failures.LocalizationFailureInputTests -v
```

Expected before implementation: at least the first newly added corruption case is accepted.
At end of stream,
reconcile primary and initialization attempt/retained counts separately
against each graph case's `overall` and `initialization_frame` summary blocks.
Wrap strict-JSON `ValueError`,
gzip `OSError`,
and truncated-stream errors as `InputIntegrityError`.
After minimal validation: all input tests pass.

- [ ] **Step 6: Commit Task 1**

Run:

```bash
git add scripts/diagnostics/analyze_localization_failures.py \
  tests/test_analyze_localization_failures.py
git commit -m "feat(diagnostics): verify localization analysis input"
```

---

### Task 2: Mutually exclusive failure budget and predecessor lineage

**Files:**
- Modify: `scripts/diagnostics/analyze_localization_failures.py`
- Modify: `tests/test_analyze_localization_failures.py`

**Interfaces:**
- Produces: `_attempt_class(row: dict) -> str`.
- Produces: bounded case/depth/time/seed counters and current-frame predecessor map.
- Consumes: Task 1 verified canonical row iterator.

- [ ] **Step 1: Write the failing attempt-versus-retained classification test**

Use literal rows:

```python
rows = [
    {"attempt_status": "converged", "containment": True},
    {"attempt_status": "converged", "containment": False},
    {
        "attempt_status": "invalid",
        "attempt_failure_reason": "invalid upstream UAV reference",
        "status": "invalid",
    },
    {
        "attempt_status": "invalid",
        "attempt_failure_reason": "non-finite or malformed WNLS input",
        "status": "invalid",
    },
    {
        "attempt_status": "failed",
        "attempt_failure_reason": "maximum WNLS iterations exceeded",
        "status": "stale",
    },
    {
        "attempt_status": "failed",
        "attempt_failure_reason": "new future failure",
        "status": "failed",
    },
]
```

Assert the six output classes are:

```python
[
    "contained",
    "converged_outside_radius",
    "upstream_unavailable",
    "invalid_input_or_numeric",
    "wnls_nonconvergence",
    "other_failed",
]
```

The mutation caught is counting retained `status="stale"` as a successful current attempt.

- [ ] **Step 2: Verify RED and implement `_attempt_class`**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_localization_failures.FailureBudgetTests.test_classifies_current_attempt_not_retained_state -v
```

Expected RED: `_attempt_class` is absent.

Implement exact branching on `attempt_status`,
`attempt_failure_reason`,
and `containment`.
Reject an unknown attempt status with `InputIntegrityError`.

- [ ] **Step 3: Write the failing partition-reconciliation test**

Build a two-frame fixture whose primary rows contain all six classes.
Assert:

```python
self.assertEqual(sum(case["failure_budget"]["counts"].values()), 12)
self.assertEqual(case["failure_budget"]["denominator"], 12)
self.assertEqual(case["attempt_status_counts"], {
    "converged": 4,
    "invalid": 4,
    "failed": 4,
})
```

Also assert that raw reason labels retain at most 32 keys,
additional labels increment `reason_label_overflow_count`,
and overflow examples stop at five.

- [ ] **Step 4: Write the failing same-frame predecessor-lineage test**

In canonical order,
write robot 1 with a direct WNLS failure,
then robot 2 with:

```python
"attempt_status": "invalid",
"attempt_failure_reason": "invalid upstream UAV reference",
"measurements": [{
    "kind": "uav",
    "id": 1,
    "estimated_reference_available": False,
}]
```

Assert:

```python
self.assertEqual(case["lineage"]["upstream_unavailable_rows"], 1)
self.assertEqual(case["lineage"]["unavailable_uav_reference_edges"], 1)
self.assertEqual(
    case["lineage"]["predecessor_attempt_classes"]["wnls_nonconvergence"],
    1,
)
```

Add a missing-predecessor fixture and require `not_observed`,
not an inferred cause.

- [ ] **Step 5: Implement bounded counters and predecessor state**

Reset the predecessor map whenever `(seed, graph_case, frame_index)` changes.
Store only:

```text
robot_id
attempt_class
attempt_status
retained_status
attempt_failure_reason
propagation_depth
```

For each unavailable UAV edge,
link to the stored lower-index predecessor.
Use `1 + max(observed parent propagation depth)` for a downstream lineage depth,
and `1` when the observed parent is a direct root failure.
Never call graph or estimator functions.

- [ ] **Step 6: Run Task 2 tests and regression**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_localization_failures.FailureBudgetTests \
  tests.test_analyze_localization_failures.PredecessorLineageTests -v
conda run -n cbf_env python -m unittest \
  tests.test_replay_localization_calibration -v
```

Expected: all tests pass and the frozen replay tests remain unchanged.

- [ ] **Step 7: Commit Task 2**

Run:

```bash
git add scripts/diagnostics/analyze_localization_failures.py \
  tests/test_analyze_localization_failures.py
git commit -m "feat(diagnostics): attribute localization failure budget"
```

---

### Task 3: Conditional calibration, persistence, strata, and paired bootstrap

**Files:**
- Modify: `scripts/diagnostics/analyze_localization_failures.py`
- Modify: `tests/test_analyze_localization_failures.py`

**Interfaces:**
- Produces: `_normalized_squared_error(error_vector, covariance) -> float`.
- Produces: `_paired_seed_bootstrap(seed_counts: list[dict], *, resamples: int = 10000, rng_seed: int = 20260729) -> dict`.
- Produces: fixed histograms for ratio, normalized squared error, reference count, FIM condition, FIM minimum eigenvalue, depth, and time.
- Produces: initialization/persistence metrics and paired-seed bootstrap for `D_upstream`.

- [ ] **Step 1: Write the failing hand-derived calibration test**

Use:

```python
error = [2.0, 1.0]
covariance = [[4.0, 0.0], [0.0, 1.0]]
```

The exact normalized squared error is:

```python
q = 2.0
```

Use rows on fixed bin boundaries:

```text
error_to_epsilon_ratio = 0.0, 0.5, 1.0, 2.0, 5.0
q = 0.0, 2.295748929, 5.991464547, 9.0, 9.000000001
```

Assert half-open bins exactly as the design specifies,
with `q==9` in `[5.991464547,9]`.

- [ ] **Step 2: Verify RED and implement finite covariance validation**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_localization_failures.ConditionalCalibrationTests.test_hand_derived_nees_and_fixed_bins -v
```

Expected RED: normalized-squared-error support is absent.

Implement symmetry tolerance:

```python
relative_error <= 1e-12 * max(1.0, max_abs_covariance)
```

Require finite positive eigenvalues.
Count invalid conditional covariance separately and exclude it from the `q` denominator.

- [ ] **Step 3: Write failing depth/time/reference/FIM stratum tests**

Use literal rows at:

```text
depth: 1 and 7
frame: 1, 100, 101, 200, 201, 300, 301, 400, 401, 499
active-reference count: 0, 1, 2, 9, 10
phi_condition: 1, 9.999, 10, 29.999, 30, 99.999, 100
phi_min: 0.01, 0.05, 0.199, 0.2, 0.999, 1
```

Assert every fixed bin is present,
including empty bins,
and all stratum denominators reconcile to the parent population.
For every depth,
time,
and dynamic 7-by-5 cell,
assert all six failure-budget class keys are present and sum to the cell
denominator.
Assert the comparison block contains dynamic-minus-fixed count and
percentage-point differences for all six classes.

- [ ] **Step 4: Write failing frame-zero and covariance-validity tests**

Create frame-zero rows at depths 1 and 7 containing:

- one contained attempt;
- one converged-outside attempt;
- one upstream-unavailable row with two unavailable UAV edges; and
- one WNLS-nonconvergence row.

Assert the frame-zero six-class/depth budgets,
upstream observer-row count,
unavailable-edge count,
and predecessor lineage reconcile.

Add converged rows whose covariance is:

```text
absent
contains Infinity
asymmetric beyond 1e-12 relative tolerance
has eigenvalues [1, 0]
```

Each increments `conditional_covariance_invalid`,
is excluded from the `q` denominator,
and remains in the epsilon-containment denominator.

- [ ] **Step 5: Write failing initialization and persistence tests**

For one key,
use frame-zero WNLS failure,
primary frames 1–2 upstream unavailable,
frame 3 converged,
frames 4–6 upstream unavailable,
and frame 7 converged.

Assert:

```python
{
    "first_primary_converged_frame": 3,
    "primary_frames_before_first_convergence": 2,
    "longest_upstream_unavailable_streak": 3,
    "longest_wnls_nonconvergence_streak": 0,
    "never_primary_converged": False,
}
```

Add a key with no primary convergence and require `never_primary_converged=True`.

- [ ] **Step 6: Write the failing paired-bootstrap test**

Use two seed records:

```python
seed_counts = [
    {"dyn_up": 8, "fix_up": 2, "dyn_invalid": 10, "fix_invalid": 4},
    {"dyn_up": 1, "fix_up": 3, "dyn_invalid": 2, "fix_invalid": 5},
]
```

Assert the point aggregate is:

```python
(9 - 5) / ((12 - 9)) == 4 / 3
```

Use a small injected resample count and seed in the unit test.
Assert every resample recomputes summed counts,
non-positive denominators increment `non_estimable_resamples`,
seed-specific non-estimable ratios are `None`,
and percentile bounds are `None` when fewer than 95% of resamples are estimable.

- [ ] **Step 7: Implement bounded metric accumulators**

Keep only:

- integer counters;
- finite sums and maxima;
- fixed histogram bins;
- 20 seed count records;
- 560 case/seed/robot persistence states;
- seven depth aggregates;
- five time aggregates; and
- the complete 7-by-5 dynamic table.

Do not retain error,
ratio,
q,
or covariance arrays.

- [ ] **Step 8: Run Task 3 tests and all Python tests**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_localization_failures.ConditionalCalibrationTests \
  tests.test_analyze_localization_failures.StratificationTests \
  tests.test_analyze_localization_failures.PersistenceTests \
  tests.test_analyze_localization_failures.PairedBootstrapTests -v
conda run -n cbf_env python -m unittest discover -s tests -p 'test_*.py' -v
```

Expected: all focused tests and the full Python suite pass.

- [ ] **Step 9: Commit Task 3**

Run:

```bash
git add scripts/diagnostics/analyze_localization_failures.py \
  tests/test_analyze_localization_failures.py
git commit -m "feat(diagnostics): summarize localization failure mechanisms"
```

---

### Task 4: Deterministic output, CLI, and disk guards

**Files:**
- Modify: `scripts/diagnostics/analyze_localization_failures.py`
- Modify: `tests/test_analyze_localization_failures.py`

**Interfaces:**
- Produces: CLI module invocation.
- Produces: `failure-mechanisms.json` and `failure-mechanisms.md` only in an explicit external non-existing output directory.
- Consumes: `available_bytes`, `allocated_bytes`, `require_start_space`, `HARD_FLOOR_BYTES`, and `OUTPUT_ROOT_CAP_BYTES` from `scripts.diagnostics.run_diagnostic`.

- [ ] **Step 1: Write the failing source-immutability and external-output test**

Hash the fixture source gzip,
manifest,
summary JSON,
and summary Markdown before analysis.
Run the real analyzer with an external output directory.
Assert:

```python
self.assertEqual(sorted(path.name for path in output.iterdir()), [
    "failure-mechanisms.json",
    "failure-mechanisms.md",
])
self.assertEqual(source_hashes_before, source_hashes_after)
self.assertLess(len((output / "failure-mechanisms.json").read_bytes()), 10_000_000)
self.assertFalse((output / "calibration.jsonl.gz").exists())
```

Run twice against two new output directories and require byte-identical JSON and Markdown.
The analyzer itself must rehash the gzip,
manifest,
summary JSON,
and summary Markdown after streaming and before it can return or write
`status="completed"`.
The test replaces one small source file at the post-stream hook and requires
`InputIntegrityError`,
proving a caller-side before/after check is not the only TOCTOU defense.

- [ ] **Step 2: Verify RED and implement deterministic rendering**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_localization_failures.OutputContractTests.test_external_output_is_deterministic_and_source_immutable -v
```

Expected RED: no output files exist.

Use:

```python
json.dumps(report, sort_keys=True, indent=2, allow_nan=False)
```

The Markdown must print:

- integrity status;
- the full six-class budget for both cases;
- `D_upstream` with estimable/non-estimable bootstrap counts;
- conditional containment and `q` thresholds;
- initialization/persistence summary;
- all depth/time cells; and
- the fixed limitations text.

- [ ] **Step 3: Write failing path and disk-limit tests**

Patch only the OS probe boundary,
not classification logic.
Test:

```text
launch free = 7,999,999,999 -> AnalysisLimitError
live free = 5,999,999,999 -> AnalysisLimitError
output-root allocation = 2,000,000,001 -> AnalysisLimitError
run allocation = 10,000,001 -> AnalysisLimitError
```

Reject an output equal to,
inside,
or containing the source bundle.
Reject a pre-existing output directory.
No failed limit case may leave a file whose JSON status is `completed`.
Define the derived-analysis root as `output_dir.parent`;
the 2 GB cap applies exactly to that directory.
Add a fixture where `allocated_bytes(output_dir.parent)==2_000_000_001`.

- [ ] **Step 4: Implement CLI and guards**

The parser accepts:

```text
--bundle-dir PATH
--output-dir PATH
--max-examples-per-bucket INTEGER
```

`max_examples_per_bucket` must be between 0 and 20.
No output directory means strict JSON to stdout and no persistent write.

For persistent output:

1. validate path separation;
2. require 8 GB at the nearest existing ancestor;
3. require both `output_dir` and the sibling
   `output_dir.name + ".incomplete"` directory to be absent;
4. create only the sibling incomplete directory;
5. stream with a 6 GB check every 10,000 rows and immediately before final
   writes;
6. rehash all four source files;
7. check the 2 GB `output_dir.parent` cap and 10 MB incomplete-directory cap;
8. write final-named JSON/Markdown inside the incomplete directory;
9. check limits again;
10. atomically rename the complete directory to `output_dir`; and
11. return `status="completed"`.

A crash or limit failure may leave only the sibling `.incomplete` directory.
It must never create the registered output directory or a JSON document with
`status="completed"`.

- [ ] **Step 5: Run focused, full, and syntax verification**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_localization_failures.OutputContractTests \
  tests.test_analyze_localization_failures.AnalysisLimitTests -v
conda run -n cbf_env python -m unittest discover -s tests -p 'test_*.py' -v
conda run -n cbf_env python -m py_compile \
  scripts/diagnostics/analyze_localization_failures.py \
  tests/test_analyze_localization_failures.py
git diff --check
```

Expected: all tests pass,
syntax compilation succeeds,
and whitespace check is clean.

- [ ] **Step 6: Commit Task 4**

Run:

```bash
git add scripts/diagnostics/analyze_localization_failures.py \
  tests/test_analyze_localization_failures.py
git commit -m "feat(diagnostics): add bounded failure-analysis output"
```

---

### Task 5: Freeze and independently review the single production audit

**Files:**
- Create: `docs/superpowers/plans/2026-07-29-cbf2026-localization-failure-mechanism-audit-run.md`
- Create: `docs/diagnostics/reviews/2026-07-29-localization-failure-analyzer-review.md`

**Interfaces:**
- Consumes: reviewed Task 1–4 executable commit and test evidence.
- Produces: exact one-run registration and independent code/method review.

- [ ] **Step 1: Write the exact run registration**

The registration records:

```text
source bundle:
/private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288

output directory:
/private/tmp/cbf2026-localization-failure-analysis/20260729T045931Z_d7c1cdcbf6434b6ca944ffacec262f4e
```

The sole production command is:

```bash
conda run -n cbf_env python -m scripts.diagnostics.analyze_localization_failures \
  --bundle-dir /private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288 \
  --output-dir /private/tmp/cbf2026-localization-failure-analysis/20260729T045931Z_d7c1cdcbf6434b6ca944ffacec262f4e \
  --max-examples-per-bucket 5
```

The registered output parent does not exist at planning time.
Before the production command,
and only after the Task 6 launch probes pass,
create that empty parent exactly once with:

```bash
mkdir -m 700 /private/tmp/cbf2026-localization-failure-analysis
```

This is an operational setup action,
not an analyzer invocation.
If the parent already exists at that point or the command fails,
stop before production execution and do not substitute another path.

Record:

- executable source SHA-256;
- executable commit;
- all source-bundle hashes from Global Constraints;
- exact schema,
  categories,
  bins,
  bootstrap seed/resamples,
  denominators,
  limitations,
  disk gates,
  and stop rules;
- no changed estimator,
  graph,
  epsilon,
  controller,
  or trajectory; and
- no result-dependent retry or taxonomy/bin change.

- [ ] **Step 2: Run independent analyzer review**

The reviewer checks:

- design/spec compliance;
- strict hash and key verification;
- attempt versus retained-state handling;
- mutual exclusivity and reconciliation;
- predecessor lineage boundedness;
- hand-derived NEES/bin tests;
- paired-seed bootstrap;
- output/source immutability;
- disk guards; and
- absence of estimator/replay/controller calls.

The review file records Critical,
Important,
and Minor findings plus a CLEAN or NEEDS FIXES verdict.
Any Critical or Important finding must be fixed and re-reviewed before registration commit.

- [ ] **Step 3: Run final pre-registration verification**

Run:

```bash
conda run -n cbf_env python -m unittest discover -s tests -p 'test_*.py' -v
conda run -n cbf_env python -m py_compile \
  scripts/diagnostics/analyze_localization_failures.py \
  tests/test_analyze_localization_failures.py
git diff --check
```

Expected: all tests pass and review verdict is CLEAN.

- [ ] **Step 4: Commit registration and review**

Run:

```bash
git add \
  docs/superpowers/plans/2026-07-29-cbf2026-localization-failure-mechanism-audit-run.md \
  docs/diagnostics/reviews/2026-07-29-localization-failure-analyzer-review.md
git commit -m "docs(diagnostics): register localization failure audit"
```

---

### Task 6: Execute once, audit the output, and record the decision

**Files:**
- Create: `docs/diagnostics/2026-07-29-localization-failure-mechanism-audit.md`
- Create: `docs/diagnostics/reviews/2026-07-29-localization-failure-evidence-review.md`
- Modify: `docs/diagnostics/README.md`
- Modify in DRA: `meta-log/2026-07-28-cbf2026-dynamic-localization-calibration-results.md`
- Modify in DRA: `papers/cbf2026/open-questions.md`
- Modify in DRA: `papers/cbf2026/sources.md`
- Modify in DRA: `papers/cbf2026/status.md`
- Modify in DRA: `papers/cbf2026/timeline.md`

**Interfaces:**
- Consumes: exact Task 5 registration and reviewed executable commit.
- Produces: one derived output bundle,
  immutable evidence report,
  independent counter/hash review,
  and DRA decision record.

- [ ] **Step 1: Check launch gates without changing state**

Verify:

```text
available bytes >= 8,000,000,000
source hashes equal the five registered hashes
output directory does not exist
derived-analysis root allocation < 2,000,000,000
```

If free space is below 8 GB,
measure and remove only the exact rebuildable cache directory
`/Users/xirhxq/.cache/codex-runtimes`,
record its pre-cleanup allocation,
then repeat the probe.
If that exact cleanup does not restore 8 GB,
stop before the production command.
Do not delete evidence,
user documents,
or Git history.

- [ ] **Step 2: Create the registered empty output parent**

Verify that
`/private/tmp/cbf2026-localization-failure-analysis`
still does not exist,
then run the exact registered `mkdir -m 700` setup command once.
Verify that the new directory is empty,
has the registered path,
and is below the 2,000,000,000-byte root cap.
If any check or the setup command fails,
stop before production execution.

- [ ] **Step 3: Run the registered command exactly once**

Run the exact Task 5 command without added flags,
redirection,
retry,
or parameter change.

If it fails,
preserve the sibling `.incomplete` directory and stop production execution.
Do not run it again in this task.

- [ ] **Step 4: Audit source immutability and output integrity**

Verify:

- the five source hashes are unchanged;
- output contains exactly JSON and Markdown;
- output allocation is below 10,000,000 bytes;
- root allocation is below 2,000,000,000 bytes;
- JSON is strict and `status=="completed"`;
- integrity counts reconcile to 280,000 total and 279,440 primary rows;
- dynamic/fixed status totals match the frozen evidence;
- every six-class,
  depth,
  time,
  and 7-by-5 table reconciles; and
- JSON/Markdown SHA-256 values are recorded.

- [ ] **Step 5: Write the evidence report**

The report must state:

- post-hoc exploratory status;
- exact source/output paths and hashes;
- exact command,
  executable commit/hash,
  environment,
  disk before/after,
  and allocations;
- the full failure budget for both graph cases;
- upstream row/edge lineage and `D_upstream`;
- conditional containment and normalized-squared-error results;
- initialization/persistence,
  depth,
  and time results;
- one-run/no-retry status;
- supported and unsupported inferences; and
- which separately designed next investigation is prioritized by the largest observed mechanism.

Do not update the paper in this task.

- [ ] **Step 5: Independently review raw counters and claims**

The evidence reviewer independently streams the immutable gzip,
recomputes the overall category/status counts and source hashes,
and checks the derived JSON/Markdown/report.
It does not rerun the analyzer or retain decompressed data.

Any integrity disagreement is Critical.
Any unsupported causal,
radius,
probability,
robustness,
or safety claim is Important.

- [ ] **Step 6: Commit code-repository evidence**

Run:

```bash
git add \
  docs/diagnostics/2026-07-29-localization-failure-mechanism-audit.md \
  docs/diagnostics/reviews/2026-07-29-localization-failure-evidence-review.md \
  docs/diagnostics/README.md
git commit -m "docs(diagnostics): record localization failure audit"
```

- [ ] **Step 7: Update and commit only CBF2026 DRA files**

Work in
`/Users/xirhxq/Documents/Clones/doctoral-research-agent`
on branch `main`.
Append the derived analysis to the DRA result log and timeline.
Update status,
sources,
and open questions with:

- code commit/hash anchors;
- source/output paths and hashes;
- all mechanism numbers;
- post-hoc limitation;
- no paper claim upgrade; and
- the next separately registered investigation.

Stage exactly:

```text
/Users/xirhxq/Documents/Clones/doctoral-research-agent/meta-log/2026-07-28-cbf2026-dynamic-localization-calibration-results.md
/Users/xirhxq/Documents/Clones/doctoral-research-agent/papers/cbf2026/open-questions.md
/Users/xirhxq/Documents/Clones/doctoral-research-agent/papers/cbf2026/sources.md
/Users/xirhxq/Documents/Clones/doctoral-research-agent/papers/cbf2026/status.md
/Users/xirhxq/Documents/Clones/doctoral-research-agent/papers/cbf2026/timeline.md
```

Do not stage or alter PodSearch2026/JFR work.

Commit:

```bash
git commit -m "docs(cbf2026): record localization failure mechanisms"
```

- [ ] **Step 8: Final whole-change verification**

Run:

```bash
conda run -n cbf_env python -m unittest discover -s tests -p 'test_*.py' -v
conda run -n cbf_env python -m py_compile \
  scripts/diagnostics/analyze_localization_failures.py \
  tests/test_analyze_localization_failures.py
git diff --check
```

Require an independent whole-change review before selecting the next estimator or calibration design.
