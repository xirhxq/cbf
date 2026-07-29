# CBF2026 Warm-Start Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Diagnose the immutable Stage 2 malformed-initialization chain, then run one registered strict-versus-restart replay that isolates deployment restart before first finite estimate.

**Architecture:** Add one read-only streaming Gate 1 classifier. Add an explicit initialization-policy selector to the existing replay, then use a small supervisor to generate strict and restart child bundles from one committed executable and one source snapshot. Add a paired analyzer that first requires normalized strict-arm row equality to the immutable baseline, then evaluates the exact registered event at the seed level.

**Tech Stack:** Python 3 in conda environment `cbf_env`, NumPy, Python standard library (`argparse`, `gzip`, `hashlib`, `json`, `pathlib`, `tarfile`, `unittest`), existing CBF2026 diagnostic helpers.

## Global Constraints

- Work only in `/private/tmp/cbf2026-diagnostic` on branch `codex/cbf2026-diagnostic`.
- Preserve the untracked `build-diagnostic/` directory; never stage, delete, or rewrite it.
- Run Python with `conda run -n cbf_env`.
- Use immutable baseline `/private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288`.
- Preserve estimator contract `variable_weight_nls_full_residual_jacobian_v1`.
- Use policies `strict_previous_v1` and `deployment_restart_before_first_finite_v1`.
- Use the baseline seed list `20260727` through `20260746`, exactly once per production policy.
- Keep dynamic and fixed graph cases, lower-index same-squad DAG membership, measurement-seed derivation, fixed references, WNLS, FIM, and coefficient-3 epsilon unchanged.
- The estimator remains offline and outside the controller.
- Gate 1 and paired-analysis output runs are each capped at `10,000,000` allocated bytes.
- The paired replay parent is capped at `250,000,000` allocated bytes and its output root at `2,000,000,000` allocated bytes.
- Require `8,000,000,000` available bytes before launch and stop below `6,000,000,000` live available bytes.
- Only `/Users/xirhxq/.cache/codex-runtimes` may be removed if the launch-space gate fails; do not delete evidence or unrelated data.
- Do not automatically retry a production Gate 1 analysis or paired replay.
- Do not change the paper from Gate 1. Change it after Gate 2 only if the registered gates and independent evidence review justify a narrow correction.
- Keep the DRA on `main` and touch only the five CBF2026 records named in Task 7.

---

### Task 1: Gate 1 temporal classifier

**Files:**
- Create: `scripts/diagnostics/analyze_initialization_persistence.py`
- Create: `tests/test_analyze_initialization_persistence.py`

**Interfaces:**
- Consumes: immutable localization bundle accepted by `scripts.diagnostics.analyze_localization_failures`.
- Produces:

```python
SCHEMA_ID = "cbf2026-initialization-persistence-v1"
EXACT_REASON = "non-finite or malformed WNLS input"

def classify_exact_reason(
    row: dict,
    preceding_row: dict | None,
) -> str:
    """Return one frozen mutually exclusive Gate 1 category."""

def analyze_initialization_persistence(
    bundle_dir: Path,
    *,
    verify_hashes: bool = True,
    output_dir: Path | None = None,
    max_examples_per_category: int = 5,
) -> dict:
    """Stream, verify, classify, and optionally publish one compact report."""
```

- Output files: `initialization-persistence.json` and `initialization-persistence.md`.

- [ ] **Step 1: Write failing pure-classification tests**

Create rows using the real field names:

```python
def malformed_row(*ranges):
    return {
        "attempt_status": "invalid",
        "attempt_failure_reason": EXACT_REASON,
        "measurements": [{"noisy_range": value} for value in ranges],
    }

class ExactReasonClassificationTests(unittest.TestCase):
    def test_missing_prior_with_finite_measurements_is_exclusive_prior(self):
        previous = {"estimate": None}
        self.assertEqual(
            classify_exact_reason(malformed_row(1.0, 2.0), previous),
            "exclusive_prior_self_estimate_missing_or_malformed",
        )

    def test_missing_prior_and_bad_measurement_is_compound(self):
        previous = {"estimate": None}
        self.assertEqual(
            classify_exact_reason(malformed_row(1.0, None), previous),
            "prior_missing_and_recorded_measurement_invalid",
        )

    def test_finite_prior_and_bad_measurement_is_measurement_category(self):
        previous = {"estimate": [3.0, 4.0]}
        self.assertEqual(
            classify_exact_reason(malformed_row(None), previous),
            "recorded_measurement_missing_or_nonfinite",
        )
```

- [ ] **Step 2: Run the tests and verify RED**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_initialization_persistence.ExactReasonClassificationTests -v
```

Expected: import failure because the new module does not exist.

- [ ] **Step 3: Implement the minimal pure classifier**

Use strict finite-vector and finite-measurement predicates:

```python
def _finite_estimate(row: dict | None) -> bool:
    if not isinstance(row, dict):
        return False
    try:
        estimate = np.asarray(row.get("estimate"), dtype=float)
    except (TypeError, ValueError, OverflowError):
        return False
    return estimate.shape == (2,) and np.isfinite(estimate).all()

def _recorded_measurements_finite(row: dict) -> bool:
    records = row.get("measurements")
    if not isinstance(records, list) or not records:
        return False
    values = [record.get("noisy_range") for record in records
              if isinstance(record, dict)]
    return len(values) == len(records) and all(
        type(value) in (int, float) and math.isfinite(value)
        for value in values
    )

def classify_exact_reason(row: dict, preceding_row: dict | None) -> str:
    prior_missing = not _finite_estimate(preceding_row)
    measurements_finite = _recorded_measurements_finite(row)
    if prior_missing and measurements_finite:
        return "exclusive_prior_self_estimate_missing_or_malformed"
    if prior_missing and not measurements_finite:
        return "prior_missing_and_recorded_measurement_invalid"
    if not measurements_finite:
        return "recorded_measurement_missing_or_nonfinite"
    return "unresolved_compound_validator"
```

- [ ] **Step 4: Run the pure tests and verify GREEN**

Run the Step 2 command.
Expected: all classification tests pass.

- [ ] **Step 5: Write failing streaming, integrity, gate, and output tests**

Build compact real-schema bundles using the same manifest/summary/hash layout
as `tests/test_analyze_localization_failures.py`.
Add a test helper
`self.analyze_fixture(rows: list[dict], robot_count: int = 1) -> dict`
that writes the bundle,
recomputes its manifest hashes,
and calls `analyze_initialization_persistence`.
Use these concrete assertions:

```python
class InitializationPersistenceAnalyzerTests(unittest.TestCase):
    def test_499_frame_persistence_reconciles_by_seed_case_and_depth(self):
        rows = self.persistence_rows(primary_frames=499)
        report = self.analyze_fixture(rows)
        dynamic = report["cases"]["dynamic_dag_wnls"]
        self.assertEqual(dynamic["exact_reason_rows"], 499)
        self.assertEqual(
            dynamic["categories"][
                "exclusive_prior_self_estimate_missing_or_malformed"
            ],
            499,
        )

    def test_gate_requires_95_percent_exclusive_prior_in_each_case(self):
        report = self.analyze_fixture(self.rows_with_dominance(0.94))
        self.assertFalse(report["gate_passed"])

    def test_gate_rejects_any_compound_prior_measurement_row(self):
        report = self.analyze_fixture(self.rows_with_one_compound_event())
        self.assertEqual(
            report["cases"]["dynamic_dag_wnls"]["categories"][
                "prior_missing_and_recorded_measurement_invalid"
            ],
            1,
        )
        self.assertFalse(report["gate_passed"])

    def test_exact_reason_plus_other_invalid_reasons_reconciles_broad_class(self):
        report = self.analyze_fixture(self.rows_with_two_invalid_reasons())
        integrity = report["integrity"]
        self.assertTrue(integrity["invalid_reason_reconciliation"])

    def test_source_hash_mutation_and_out_of_order_keys_fail_closed(self):
        bundle = self.write_fixture(self.persistence_rows(primary_frames=2))
        self.rewrite_first_process_key(bundle, frame_index=1)
        with self.assertRaises(InputIntegrityError):
            analyze_initialization_persistence(bundle)

    def test_external_output_is_atomic_and_below_10_mb(self):
        bundle = self.write_fixture(self.persistence_rows(primary_frames=2))
        output = self.root / "analysis" / "run"
        analyze_initialization_persistence(bundle, output_dir=output)
        self.assertEqual(
            {path.name for path in output.iterdir()},
            {
                "initialization-persistence.json",
                "initialization-persistence.md",
            },
        )
        self.assertLess(allocated_bytes(output), 10_000_000)
```

The expected report keys are:

```python
{
    "schema": SCHEMA_ID,
    "status": "completed",
    "source": {"bundle_dir": "absolute path", "hashes": {"name": "sha256"}},
    "protocol": {"exact_reason": EXACT_REASON, "dominance_gate": 0.95},
    "integrity": {
        "observed_rows": 0,
        "invalid_reason_reconciliation": True,
        "source_hashes_unchanged": True,
    },
    "cases": {
        "dynamic_dag_wnls": {
            "exact_reason_rows": int,
            "categories": {"frozen-category-name": int},
            "dominance_fraction": float,
            "gate_passed": bool,
            "by_seed": {"seed": {"frozen-category-name": int}},
            "by_frame": {"frame": {"frozen-category-name": int}},
            "by_depth": {"depth": {"frozen-category-name": int}},
            "preceding_attempt_reasons": {"raw-reason": int},
        },
        "fixed_refs_wnls": {"same schema as dynamic_dag_wnls": True},
    },
    "gate_passed": bool,
}
```

- [ ] **Step 6: Run the analyzer tests and verify RED**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_initialization_persistence.InitializationPersistenceAnalyzerTests -v
```

Expected: failures because streaming analysis and output publication are not implemented.

- [ ] **Step 7: Implement bounded streaming analysis**

Reuse the existing verified reader rather than duplicating its integrity
contract:

```python
from scripts.diagnostics.analyze_localization_failures import (
    AnalysisLimitError,
    InputIntegrityError,
    _iter_verified_rows,
    _read_object,
    _stream_dimensions,
    _verify_unchanged_inputs,
)
```

Maintain only:

```python
preceding: dict[tuple[int, str, int], dict]
case_counters: dict[str, dict]
examples: dict[tuple[str, str], list[dict]]
```

For every verified row:

1. store the old preceding row under `(seed, graph_case, robot_id)`;
2. if the row has the exact reason, classify against that old row;
3. separately count all non-upstream invalid raw reasons for reconciliation;
4. update bounded seed/frame/depth/reason counters; and
5. replace the preceding row.

Copy the existing sibling-incomplete/atomic-rename pattern, but publish only
the two registered filenames and enforce the 10 MB cap before and after each
write.

- [ ] **Step 8: Run focused and regression tests**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_initialization_persistence -v
conda run -n cbf_env python -m unittest \
  tests.test_analyze_localization_failures -v
conda run -n cbf_env python -m py_compile \
  scripts/diagnostics/analyze_initialization_persistence.py
```

Expected: all pass.

- [ ] **Step 9: Commit Gate 1 implementation**

```bash
git add \
  scripts/diagnostics/analyze_initialization_persistence.py \
  tests/test_analyze_initialization_persistence.py
git commit -m "feat(diagnostics): classify initialization persistence"
```

### Task 2: Register, review, and execute Gate 1 once

**Files:**
- Create: `docs/superpowers/plans/2026-07-29-cbf2026-initialization-persistence-run.md`
- Create after execution: `docs/diagnostics/2026-07-29-initialization-persistence.md`
- Create after review: `docs/diagnostics/reviews/2026-07-29-initialization-persistence-review.md`

**Interfaces:**
- Consumes: Task 1 analyzer commit and immutable baseline bundle.
- Produces: one reviewed Gate 1 decision and immutable compact output under `/private/tmp/cbf2026-initialization-persistence`.

- [ ] **Step 1: Freeze the Gate 1 run registration**

The registration must record:

```text
source commit
analyzer file SHA-256
immutable bundle path and all four source hashes
schema cbf2026-initialization-persistence-v1
exact reason string
four mutually exclusive categories
95% per-case dominance gate
zero compound-row gate
8 GB launch / 6 GB live / 2 GB root / 10 MB run limits
one execution and no automatic retry
```

- [ ] **Step 2: Commit the registration before execution**

```bash
git add docs/superpowers/plans/2026-07-29-cbf2026-initialization-persistence-run.md
git commit -m "docs(diagnostics): register initialization persistence audit"
```

- [ ] **Step 3: Run the full test suite and independent code/protocol review**

Run:

```bash
conda run -n cbf_env python -m unittest discover -s tests -p 'test_*.py' -v
git diff --check
df -Pk /private/tmp
du -sk /Users/xirhxq/.cache/codex-runtimes 2>/dev/null
```

Do not execute Gate 1 unless tests pass,
tracked status is clean,
free bytes are at least `8,000,000,000`,
and an independent reviewer reports no Critical or Important finding.

- [ ] **Step 4: Execute the registered analyzer exactly once**

Run:

```bash
conda run -n cbf_env python -m \
  scripts.diagnostics.analyze_initialization_persistence \
  --bundle-dir /private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288 \
  --output-dir /private/tmp/cbf2026-initialization-persistence/registered-gate-1
```

Do not retry this production command.
If it fails,
record the terminal failure and stop this plan before Task 3.

- [ ] **Step 5: Independently audit the compact evidence**

The reviewer must independently reproduce:

- source hashes and row count;
- exact-reason and broad-class reconciliation;
- all four category counts;
- per-case dominance fractions;
- zero compound rows;
- disk allocation; and
- the final Gate 1 decision.

If either graph case fails the registered gate,
record the negative result,
update the DRA,
and stop before Task 3.

- [ ] **Step 6: Commit Gate 1 report and review**

```bash
git add \
  docs/diagnostics/2026-07-29-initialization-persistence.md \
  docs/diagnostics/reviews/2026-07-29-initialization-persistence-review.md
git commit -m "docs(diagnostics): record initialization persistence evidence"
```

### Task 3: Explicit initialization-policy selector

**Files:**
- Modify: `scripts/diagnostics/replay_localization_calibration.py`
- Modify: `tests/test_replay_localization_calibration.py`

**Interfaces:**
- Produces:

```python
STRICT_PREVIOUS_POLICY = "strict_previous_v1"
RESTART_BEFORE_FIRST_FINITE_POLICY = (
    "deployment_restart_before_first_finite_v1"
)
INITIALIZATION_POLICIES = (
    STRICT_PREVIOUS_POLICY,
    RESTART_BEFORE_FIRST_FINITE_POLICY,
)

def select_initial_estimate(
    policy: str,
    *,
    frame_index: int,
    deployment: np.ndarray,
    previous_result: dict | None,
    ever_acquired_finite: bool,
) -> tuple[np.ndarray, str]:
    """Return initial estimate and auditable source label."""
```

- Source labels:

```text
deployment_frame_zero
previous_finite
strict_previous_missing
deployment_restart_before_first_finite
```

- [ ] **Step 1: Write failing selector tests**

Add:

```python
class InitializationPolicyTests(unittest.TestCase):
    def test_frame_zero_uses_deployment_for_both_policies(self):
        deployment = np.array([10.0, 20.0])
        for policy in INITIALIZATION_POLICIES:
            initial, source = select_initial_estimate(
                policy,
                frame_index=0,
                deployment=deployment,
                previous_result=None,
                ever_acquired_finite=False,
            )
            np.testing.assert_array_equal(initial, deployment)
            self.assertEqual(source, "deployment_frame_zero")

    def test_finite_previous_estimate_wins_for_both_policies(self):
        previous = {
            "status": "converged",
            "estimate": [1.0, 2.0],
            "covariance": [[1.0, 0.0], [0.0, 1.0]],
            "epsilon": 3.0,
            "phi_min_eigenvalue": 1.0,
            "phi_condition": 1.0,
        }
        for policy in INITIALIZATION_POLICIES:
            initial, source = select_initial_estimate(
                policy,
                frame_index=1,
                deployment=np.array([10.0, 20.0]),
                previous_result=previous,
                ever_acquired_finite=True,
            )
            np.testing.assert_array_equal(initial, [1.0, 2.0])
            self.assertEqual(source, "previous_finite")

    def test_strict_policy_preserves_null_warm_start(self):
        initial, source = select_initial_estimate(
            STRICT_PREVIOUS_POLICY,
            frame_index=1,
            deployment=np.array([10.0, 20.0]),
            previous_result={"estimate": None},
            ever_acquired_finite=False,
        )
        self.assertEqual(initial.shape, ())
        self.assertEqual(source, "strict_previous_missing")

    def test_restart_uses_deployment_only_before_first_finite(self):
        initial, source = select_initial_estimate(
            RESTART_BEFORE_FIRST_FINITE_POLICY,
            frame_index=1,
            deployment=np.array([10.0, 20.0]),
            previous_result={"estimate": None},
            ever_acquired_finite=False,
        )
        np.testing.assert_array_equal(initial, [10.0, 20.0])
        self.assertEqual(source, "deployment_restart_before_first_finite")

    def test_restart_does_not_restart_after_ever_finite(self):
        initial, source = select_initial_estimate(
            RESTART_BEFORE_FIRST_FINITE_POLICY,
            frame_index=2,
            deployment=np.array([10.0, 20.0]),
            previous_result={"estimate": None},
            ever_acquired_finite=True,
        )
        self.assertEqual(initial.shape, ())
        self.assertEqual(source, "strict_previous_missing")
```

- [ ] **Step 2: Run selector tests and verify RED**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_replay_localization_calibration.InitializationPolicyTests -v
```

Expected: import/name failures.

- [ ] **Step 3: Implement the minimal selector**

```python
def select_initial_estimate(
    policy,
    *,
    frame_index,
    deployment,
    previous_result,
    ever_acquired_finite,
):
    if policy not in INITIALIZATION_POLICIES:
        raise ValueError(f"unknown initialization policy: {policy}")
    if frame_index == 0:
        return np.asarray(deployment, dtype=float), "deployment_frame_zero"
    valid_previous = _valid_prior_result(previous_result)
    if valid_previous is not None:
        return valid_previous[0], "previous_finite"
    if (
        policy == RESTART_BEFORE_FIRST_FINITE_POLICY
        and not ever_acquired_finite
    ):
        return (
            np.asarray(deployment, dtype=float),
            "deployment_restart_before_first_finite",
        )
    raw = (
        previous_result.get("estimate", deployment)
        if isinstance(previous_result, dict)
        else deployment
    )
    return np.asarray(raw, dtype=float), "strict_previous_missing"
```

- [ ] **Step 4: Run selector and solver regression tests**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_replay_localization_calibration.InitializationPolicyTests \
  tests.test_replay_localization_calibration.WnlsAndFimTests -v
```

Expected: all pass and no WNLS/FIM constant changes.

- [ ] **Step 5: Commit the selector**

```bash
git add \
  scripts/diagnostics/replay_localization_calibration.py \
  tests/test_replay_localization_calibration.py
git commit -m "feat(diagnostics): add explicit restart policy"
```

### Task 4: One supervised four-cell replay

**Files:**
- Modify: `scripts/diagnostics/replay_localization_calibration.py`
- Modify: `tests/test_replay_localization_calibration.py`
- Create: `scripts/diagnostics/run_warm_start_recovery.py`
- Create: `tests/test_run_warm_start_recovery.py`

**Interfaces:**
- Extend:

```python
def replay_calibration(
    data_path,
    manifest_path,
    output_root,
    run_seeds,
    project_root,
    max_frames=None,
    *,
    initialization_policy: str = STRICT_PREVIOUS_POLICY,
) -> dict:
    """Replay one explicit initialization policy into one child bundle."""
```

- Add row fields:

```text
initialization_policy
initial_estimate_source
ever_acquired_finite_before_attempt
```

- Add supervisor:

```python
def run_warm_start_recovery(
    data_path: Path,
    input_manifest_path: Path,
    immutable_baseline_dir: Path,
    output_root: Path,
    seeds: list[int],
    project_root: Path,
    max_frames: int,
) -> dict:
    """Create one parent with strict and restart child bundles."""
```

- Parent layout:

```text
timestamped-parent-run/
  manifest.json
  source-snapshot.tar.gz
  strict/timestamped-localization-calibration-run/
  restart/timestamped-localization-calibration-run/
```

- [ ] **Step 1: Write failing replay-state and row-provenance tests**

Patch `solve_wnls` to force a frame-zero failure,
then allow convergence on frame one.
Assert:

```python
self.assertEqual(strict_row["initial_estimate_source"],
                 "strict_previous_missing")
self.assertEqual(restart_row["initial_estimate_source"],
                 "deployment_restart_before_first_finite")
self.assertFalse(restart_row["ever_acquired_finite_before_attempt"])
self.assertEqual(strict_row["active_references"],
                 restart_row["active_references"])
self.assertEqual(strict_row["measurements"], restart_row["measurements"])
```

Add a synthetic post-acquisition missing state and assert restart preserves
`strict_previous_missing`.

- [ ] **Step 2: Run the replay tests and verify RED**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_replay_localization_calibration.ReplayEvidenceBundleTests -v
```

- [ ] **Step 3: Thread policy and ever-finite state through replay**

For each child replay,
maintain:

```python
ever_finite: dict[tuple[int, str, int], bool]
```

where the key is `(seed, graph_case, robot_id)`.
Capture `ever_before`,
call `select_initial_estimate`,
write all three provenance fields,
and set the flag after:

```python
if _valid_prior_result(result) is not None:
    ever_finite[(seed, graph_case, robot_id)] = True
```

Record the policy in settings,
summary,
and manifest.
Keep default `strict_previous_v1` so existing callers preserve behavior.

- [ ] **Step 4: Write failing supervisor tests**

In `setUp`,
materialize valid one-frame data,
trajectory manifest,
immutable baseline,
and project Git fixtures.
Add
`self.run_fixture(child_reasons: tuple[str, str] = ("completed", "completed"))`
to patch the two child calls while leaving parent validation active.
Use these assertions:

```python
class WarmStartRecoveryRunnerTests(unittest.TestCase):
    def test_one_parent_runs_both_policies_with_same_inputs_and_seeds(self):
        manifest, child_calls = self.run_fixture()
        self.assertEqual(manifest["termination_reason"], "completed")
        self.assertEqual(
            [call.kwargs["initialization_policy"] for call in child_calls],
            [
                STRICT_PREVIOUS_POLICY,
                RESTART_BEFORE_FIRST_FINITE_POLICY,
            ],
        )
        self.assertEqual(child_calls[0].args[3], child_calls[1].args[3])

    def test_parent_records_commit_snapshot_and_baseline_hashes(self):
        manifest, _ = self.run_fixture()
        self.assertEqual(manifest["source_commit"], "deadbeef")
        self.assertEqual(len(manifest["source_snapshot_sha256"]), 64)
        self.assertEqual(
            set(manifest["immutable_baseline_hashes"]),
            {"manifest.json", "summary.json", "summary.md", "calibration.jsonl.gz"},
        )

    def test_child_failure_prevents_completed_parent_registration(self):
        manifest, _ = self.run_fixture(
            child_reasons=("completed", "runner_setup_error")
        )
        self.assertEqual(manifest["termination_reason"], "child_failure")

    def test_parent_enforces_8gb_6gb_2gb_and_250mb_limits(self):
        with patch(
            "scripts.diagnostics.run_warm_start_recovery.available_bytes",
            return_value=7_999_999_999,
        ):
            with self.assertRaises(DiskSpaceError):
                self.run_fixture()
```

- [ ] **Step 5: Implement the supervisor**

Before creating output:

1. verify immutable baseline hashes;
2. require 8 GB;
3. capture `git rev-parse HEAD`,
   branch,
   and tracked working-tree status;
4. allocate one collision-resistant parent;
5. create one source snapshot using the existing
   `run_diagnostic._create_source_snapshot` exclusion policy; and
6. record all input paths and hashes.

Call the same imported `replay_calibration` function once for strict and once
for restart,
with identical data,
manifest,
seed list,
and frame count.
Do not retry either child.
Register parent `termination_reason="completed"` only when both child
manifests are completed and all caps pass.

- [ ] **Step 6: Run focused and full replay tests**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_replay_localization_calibration \
  tests.test_run_warm_start_recovery -v
conda run -n cbf_env python -m py_compile \
  scripts/diagnostics/replay_localization_calibration.py \
  scripts/diagnostics/run_warm_start_recovery.py
```

- [ ] **Step 7: Commit the four-cell runner**

```bash
git add \
  scripts/diagnostics/replay_localization_calibration.py \
  scripts/diagnostics/run_warm_start_recovery.py \
  tests/test_replay_localization_calibration.py \
  tests/test_run_warm_start_recovery.py
git commit -m "feat(diagnostics): run paired warm-start policies"
```

### Task 5: Strict-anchor validation and paired analyzer

**Files:**
- Create: `scripts/diagnostics/compare_warm_start_recovery.py`
- Create: `tests/test_compare_warm_start_recovery.py`

**Interfaces:**
- Produces:

```python
SCHEMA_ID = "cbf2026-warm-start-recovery-comparison-v1"
BOOTSTRAP_RESAMPLES = 10_000
BOOTSTRAP_SEED = 20260729

def compare_warm_start_recovery(
    paired_bundle_dir: Path,
    immutable_baseline_dir: Path,
    *,
    output_dir: Path | None = None,
    verify_hashes: bool = True,
) -> dict:
    """Validate the strict anchor, pair seeds, and evaluate frozen gates."""
```

- Output files: `warm-start-recovery-comparison.json` and
  `warm-start-recovery-comparison.md`.

- [ ] **Step 1: Write failing strict-anchor tests**

Construct tiny immutable,
strict,
and restart streams.
The strict row may differ only by:

```python
PROSPECTIVE_FIELDS = {
    "initialization_policy",
    "initial_estimate_source",
    "ever_acquired_finite_before_attempt",
}
```

Create
`self.compare_fixture(strict_mutation: Callable[[dict], None] | None = None,
key_mutation: str | None = None) -> dict`
to write three valid tiny bundles and call the public comparator.
Use these assertions:

```python
def normalized(row):
    return {
        key: value
        for key, value in row.items()
        if key not in PROSPECTIVE_FIELDS
    }

class StrictAnchorTests(unittest.TestCase):
    def test_strict_rows_equal_immutable_rows_after_normalization(self):
        report = self.compare_fixture()
        self.assertTrue(report["strict_anchor"]["normalized_rows_equal"])
        self.assertEqual(report["strict_anchor"]["rows_compared"], 8)

    def test_any_scientific_field_drift_stops_before_policy_statistics(self):
        def mutate(row):
            row["epsilon"] = 999.0
        with self.assertRaises(InputIntegrityError):
            self.compare_fixture(strict_mutation=mutate)

    def test_missing_duplicate_or_reordered_key_stops(self):
        for mutation in ("missing", "duplicate", "reordered"):
            with self.subTest(mutation=mutation):
                with self.assertRaises(InputIntegrityError):
                    self.compare_fixture(key_mutation=mutation)
```

- [ ] **Step 2: Run strict-anchor tests and verify RED**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_compare_warm_start_recovery.StrictAnchorTests -v
```

- [ ] **Step 3: Implement lockstep normalized equality**

Stream the immutable baseline and strict child in lockstep.
Compare exact keys and normalized dictionaries.
Do not retain raw rows.
Return:

```python
{
    "rows_compared": 280000,
    "normalized_rows_equal": True,
    "immutable_hashes_unchanged": True,
}
```

Raise `InputIntegrityError` on the first mismatch and include only the compact
key,
not the full row,
in the exception.

- [ ] **Step 4: Write failing exact-outcome and hierarchy tests**

The exact primary predicate is:

```python
def exact_direct_event(row):
    return (
        row["primary_statistics"] is True
        and row["attempt_status"] == "invalid"
        and row["attempt_failure_reason"]
        == "non-finite or malformed WNLS input"
    )
```

Tests must prove:

- denominator includes every primary row;
- rows are aggregated to one fraction per seed and policy;
- the dynamic primary gate requires both CI below zero and 90% count
  reduction;
- upstream inference is omitted when the primary gate fails;
- fixed results remain descriptive;
- \(q>9\) and containment safeguards use aggregate converged attempts;
- restart provenance counts reconcile to restart child rows; and
- the 10 MB atomic output cap is enforced.

- [ ] **Step 5: Implement paired statistics and safeguards**

Use 10,000 resamples from a fresh
`np.random.default_rng(20260729)`.
Each resample draws 20 seed indices with replacement and computes the mean
paired fraction difference.
Report raw seed records,
aggregate counts,
percentage-point and relative differences,
median paired difference,
two-sided percentile interval,
and gate booleans.

For calibration,
call `analyze_localization_failures` separately on strict and restart child
bundles and extract the already reviewed aggregate calibration counters.
Do not treat rows as independent.

- [ ] **Step 6: Run analyzer and regression tests**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_compare_warm_start_recovery \
  tests.test_analyze_localization_failures -v
conda run -n cbf_env python -m py_compile \
  scripts/diagnostics/compare_warm_start_recovery.py
```

- [ ] **Step 7: Commit the paired analyzer**

```bash
git add \
  scripts/diagnostics/compare_warm_start_recovery.py \
  tests/test_compare_warm_start_recovery.py
git commit -m "feat(diagnostics): compare warm-start recovery"
```

### Task 6: Freeze and independently review the production protocol

**Files:**
- Create: `docs/superpowers/plans/2026-07-29-cbf2026-warm-start-recovery-run.md`
- Create: `docs/diagnostics/reviews/2026-07-29-warm-start-recovery-code-protocol-review.md`

**Interfaces:**
- Consumes: passing Gate 1, Tasks 3–5 commits, exact trajectory input, exact seed list.
- Produces: immutable commands and one independent approval before execution.

- [ ] **Step 1: Record exact production inputs**

Freeze:

```text
data:
/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json

trajectory manifest:
/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json

immutable baseline:
/private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288

output root:
/private/tmp/cbf2026-warm-start-recovery

seeds:
20260727..20260746 inclusive

frames:
500
```

Record current hashes,
source commit,
executable module hashes,
policy IDs,
outcome schema,
gates,
bootstrap constants,
disk limits,
one-run/no-retry rule,
and stop behavior.

- [ ] **Step 2: Commit the protocol before review**

```bash
git add docs/superpowers/plans/2026-07-29-cbf2026-warm-start-recovery-run.md
git commit -m "docs(diagnostics): register warm-start recovery run"
```

- [ ] **Step 3: Run full verification**

```bash
conda run -n cbf_env python -m unittest discover -s tests -p 'test_*.py' -v
conda run -n cbf_env python -m py_compile \
  scripts/diagnostics/analyze_initialization_persistence.py \
  scripts/diagnostics/replay_localization_calibration.py \
  scripts/diagnostics/run_warm_start_recovery.py \
  scripts/diagnostics/compare_warm_start_recovery.py
git diff --check
git status --short --branch
df -Pk /private/tmp
du -sk /Users/xirhxq/.cache/codex-runtimes 2>/dev/null
```

Tracked files must be clean.
The only tolerated unrelated status is `?? build-diagnostic/`.

- [ ] **Step 4: Obtain independent code and protocol review**

The reviewer must inspect:

- Gate 1 decision and evidence review;
- policy selector and `ever_acquired_finite` transition;
- identical references and measurements;
- supervisor single-source/four-cell contract;
- strict normalized anchor;
- exact primary denominator and bootstrap;
- hierarchical secondary and calibration safeguards;
- provenance and source snapshot;
- disk guards and no-retry behavior; and
- claim boundary.

Resolve every Critical or Important finding in a new scoped commit,
rerun the full suite,
and repeat review before production.

- [ ] **Step 5: Commit the clean review**

```bash
git add \
  docs/diagnostics/reviews/2026-07-29-warm-start-recovery-code-protocol-review.md
git commit -m "docs(diagnostics): review warm-start recovery protocol"
```

### Task 7: Execute once, audit evidence, and update DRA

**Files:**
- Create: `docs/diagnostics/2026-07-29-warm-start-recovery.md`
- Create: `docs/diagnostics/reviews/2026-07-29-warm-start-recovery-evidence-review.md`
- Modify in `/Users/xirhxq/Documents/Clones/doctoral-research-agent`:
  `meta-log/2026-07-28-cbf2026-dynamic-localization-calibration-results.md`
- Modify in `/Users/xirhxq/Documents/Clones/doctoral-research-agent`:
  `papers/cbf2026/open-questions.md`
- Modify in `/Users/xirhxq/Documents/Clones/doctoral-research-agent`:
  `papers/cbf2026/sources.md`
- Modify in `/Users/xirhxq/Documents/Clones/doctoral-research-agent`:
  `papers/cbf2026/status.md`
- Modify in `/Users/xirhxq/Documents/Clones/doctoral-research-agent`:
  `papers/cbf2026/timeline.md`

**Interfaces:**
- Consumes: clean reviewed production commit and registered Task 6 protocol.
- Produces: one paired parent bundle, one compact comparison, independent evidence review, and durable CBF2026 records.

- [ ] **Step 1: Perform final preflight without changing state**

Verify:

```bash
git rev-parse HEAD
git status --short --branch
df -Pk /private/tmp
du -sk \
  /private/tmp/cbf2026-warm-start-recovery \
  /Users/xirhxq/.cache/codex-runtimes 2>/dev/null
shasum -a 256 \
  /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json \
  /private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288/manifest.json
```

If free space is below 8 GB,
measure the exact authorized cache.
Remove only that cache if necessary,
then repeat the free-space probe.
If 8 GB is still unavailable,
do not launch.

- [ ] **Step 2: Execute the paired supervisor exactly once**

Run the exact command frozen in Task 6 with:

```text
module: scripts.diagnostics.run_warm_start_recovery
data: exact Task 6 data path
input manifest: exact Task 6 manifest path
immutable baseline: exact Task 6 baseline path
output root: /private/tmp/cbf2026-warm-start-recovery
20 explicit --seed arguments: 20260727 through 20260746
max frames: 500
```

Capture its strict JSON terminal manifest.
Do not retry if either child or the parent fails.

- [ ] **Step 3: Execute the registered paired analyzer exactly once**

Use the exact completed parent `output_dir` recorded by Step 2 and write to a
new child of:

```text
/private/tmp/cbf2026-warm-start-recovery-analysis
```

Run `scripts.diagnostics.compare_warm_start_recovery` with the exact immutable
baseline path and the registered `--output-dir`.
Do not retry production analysis.

- [ ] **Step 4: Independently audit all evidence**

The independent reviewer must stream the raw bundles and reproduce:

- parent,
  child,
  input,
  source-snapshot,
  summary,
  and process hashes;
- 560,000 total new process rows;
- normalized equality of all 280,000 strict rows to the immutable baseline;
- identical policy denominators and seed sets;
- exact primary event counts by seed;
- 10,000-resample paired interval with seed 20260729;
- 90% aggregate-reduction gate;
- hierarchical upstream result;
- calibration safeguards;
- restart provenance counts;
- start,
  minimum-live,
  and end disk probes; and
- output allocations.

The review must state the exact claim that is and is not supported.

- [ ] **Step 5: Write and commit the diagnostic report and evidence review**

```bash
git add \
  docs/diagnostics/2026-07-29-warm-start-recovery.md \
  docs/diagnostics/reviews/2026-07-29-warm-start-recovery-evidence-review.md
git commit -m "docs(diagnostics): record warm-start recovery evidence"
```

- [ ] **Step 6: Update exactly five CBF2026 DRA records on DRA `main`**

Record:

- design,
  implementation,
  registration,
  and evidence commit hashes;
- exact bundle and analysis paths and hashes;
- Gate 1 and Gate 2 outcomes;
- supported and unsupported claims;
- disk/cache probes;
- the next decision:
  multi-trajectory validation if all gates pass,
  or availability-aware graph-policy design if propagation remains.

Append to `timeline.md`; do not rewrite prior entries.
Preserve unrelated DRA dirt.

- [ ] **Step 7: Commit only the five DRA files**

From `/Users/xirhxq/Documents/Clones/doctoral-research-agent`:

```bash
git add \
  meta-log/2026-07-28-cbf2026-dynamic-localization-calibration-results.md \
  papers/cbf2026/open-questions.md \
  papers/cbf2026/sources.md \
  papers/cbf2026/status.md \
  papers/cbf2026/timeline.md
git diff --cached --name-only
git commit -m "docs(cbf2026): record warm-start recovery evidence"
```

The staged-name output must contain exactly those five paths.

- [ ] **Step 8: Decide paper action without broadening claims**

If any registered integrity,
primary,
hierarchical,
or calibration gate fails,
do not modify `main.tex`.
Record the negative or limited result and stop.

If the mechanism and safeguards pass,
prepare a separate narrow paper-edit design.
Do not claim graph superiority,
radius validation,
controller safety,
mission success,
or multi-trajectory generality from this single trajectory.

- [ ] **Step 9: Final verification**

Run:

```bash
conda run -n cbf_env python -m unittest discover -s tests -p 'test_*.py' -v
git diff --check
git status --short --branch
df -Pk /private/tmp
du -sk \
  /private/tmp/cbf2026-warm-start-recovery \
  /private/tmp/cbf2026-warm-start-recovery-analysis \
  /Users/xirhxq/.cache/codex-runtimes 2>/dev/null
```

Report exact commits,
tests,
evidence paths,
gate outcomes,
and remaining limitation.
