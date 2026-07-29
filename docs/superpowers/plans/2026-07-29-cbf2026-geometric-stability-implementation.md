# CBF2026 Geometric Stability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prove conditional nondegeneracy of the modeled dynamic-DAG FIM, then audit whether the existing finite-depth trajectory exhibits usable active-set geometry, stable absolute localization error, and acceptable estimator availability without treating the inverse FIM as a true-error bound.

**Architecture:** Add one read-only analyzer that reuses the completed Gate 2 integrity contract, reconstructs fixed and dynamic reference geometry from immutable trajectory and replay records, and publishes only compact aggregate JSON and Markdown. Keep geometry, absolute error, availability, and radius calibration as separate result families. Freeze an exploratory protocol before the authoritative run, independently review the code and evidence, then update DRA and the paper only to the extent supported.

**Tech Stack:** Python 3 in conda environment `cbf_env`, NumPy, Python standard library (`argparse`, `gzip`, `hashlib`, `json`, `math`, `pathlib`, `unittest`), existing CBF2026 diagnostic helpers, LaTeX for the independent paper repository.

## Global Constraints

- Work on code only in `/private/tmp/cbf2026-diagnostic` on branch `codex/cbf2026-diagnostic`.
- Preserve the untracked `/private/tmp/cbf2026-diagnostic/build-diagnostic/` directory; never stage, delete, or rewrite it.
- Run every Python command with `conda run -n cbf_env`.
- Treat commit `b2fda72` and `docs/superpowers/specs/2026-07-29-cbf2026-geometric-nondegeneracy-and-empirical-stability-design.md` as the approved scientific contract.
- Reuse the completed Gate 2 parent bundle `/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b`.
- Require parent manifest SHA-256 `c110275910c3e44ecf97c66ba7346fbb0b340cfb844d050c0ecf56944e5ff55a`.
- Reuse the registered comparison `/private/tmp/cbf2026-warm-start-recovery-analysis/registered-gate-2/warm-start-recovery-comparison.json`.
- Require comparison SHA-256 `ef8014915b3cced6a75e094b3d16f01f17b5f881c6357dff6b885b6e871498ca`.
- Preserve estimator contract `variable_weight_nls_full_residual_jacobian_v1`, dynamic active-reference membership, WNLS, FIM, coefficient-3 epsilon, truth trajectory, and range-noise seeds.
- The estimator remains offline and never enters the controller, vehicle state, CVT state, communication state, or dynamics.
- This is post-hoc exploratory analysis of one truth trajectory and 20 nested range-noise seeds; geometry has one trajectory-level replicate.
- Do not claim a deterministic true-error bound, arbitrary-depth stability, unconditional safety, fixed-pair nondegeneracy, graph superiority, or cross-trajectory generality.
- Use `squad_local_index`, not the term hop count, because dynamic base references can shorten graph distance.
- Use five frozen 50-second bins over the 250-second input trajectory.
- Compute true-geometry scores for every dynamic primary tuple independently of estimator convergence; compute estimated-geometry, FIM, error, and calibration statistics only under their explicit valid denominators.
- Use one range-noise seed as the statistical unit for estimator outcomes; never treat rows, frames, UAVs, or edges as independent.
- The authoritative compact output directory is `/private/tmp/cbf2026-geometric-stability-analysis/exploratory-v1`.
- Compact analysis output is capped at `10,000,000` allocated bytes and must not duplicate any raw process or truth stream.
- Require `8,000,000,000` available bytes before a production launch, stop below `6,000,000,000` live available bytes, and keep rebuildable cache usage within `2,000,000,000` bytes.
- Do not delete evidence. If launch space is insufficient, only `/Users/xirhxq/.cache/codex-runtimes` may be removed after measuring it.
- Do not retry the authoritative exploratory run automatically.
- Do not edit the paper until the analyzer, evidence report, and independent evidence review are complete.
- Update DRA only in `/private/tmp/dra-cbf2026-diagnostic` on `main`; do not touch the dirty primary DRA checkout.
- The independent paper repository `/Users/xirhxq/Documents/Clones/cbf/papers/CBF2026` is dirty. If a paper edit is justified, create a new isolated worktree from its current `main` commit and never stage or alter its existing `main.pdf`, `rebuttal.pdf`, `rebuttal.tex`, or `AGENTS.md`.

---

### Task 1: Geometry primitives and immutable trajectory contract

**Files:**
- Create: `scripts/diagnostics/analyze_geometric_stability.py`
- Create: `tests/test_analyze_geometric_stability.py`

**Interfaces:**
- Consumes: two-dimensional observer and reference positions; the immutable input `data.json`; fixed reference topology from `replay_localization_calibration.fixed_references`.
- Produces:

```python
SCHEMA_ID = "cbf2026-geometric-stability-v1"
TIME_BIN_SECONDS = 50.0
OUTPUT_JSON_NAME = "geometric-stability.json"
OUTPUT_MARKDOWN_NAME = "geometric-stability.md"

from scripts.diagnostics.compare_warm_start_recovery import (
    InputIntegrityError,
)

class AnalysisLimitError(RuntimeError):
    pass

def geometry_metrics(
    observer: object,
    references: object,
) -> dict:
    """Return count, eigenvalues, normalized minimum, and finite/SPD flags."""

def fixed_pair_metrics(
    observer: object,
    references: object,
) -> dict:
    """Return included angle, noncollinearity angle, cosine magnitude, and area."""

def load_trajectory(
    path: Path,
    *,
    expected_sha256: str,
) -> dict:
    """Verify and load the immutable config, frames, truth, and target centers."""
```

- `geometry_metrics` returns:

```python
{
    "reference_count": int,
    "lambda_min": float,
    "lambda_max": float,
    "normalized_lambda_min": float,
    "finite": True,
    "positive_definite": bool,
}
```

- `fixed_pair_metrics` returns:

```python
{
    "included_angle_rad": float,
    "noncollinearity_angle_rad": float,
    "absolute_cosine": float,
    "twice_triangle_area": float,
}
```

- [ ] **Step 1: Write failing geometry tests**

Use exact orthogonal, collinear, and augmented geometries:

```python
class GeometryPrimitiveTests(unittest.TestCase):
    def test_orthogonal_pair_has_identity_geometry(self):
        result = geometry_metrics([0.0, 0.0], [[1.0, 0.0], [0.0, 1.0]])
        self.assertAlmostEqual(result["lambda_min"], 1.0)
        self.assertAlmostEqual(result["lambda_max"], 1.0)
        self.assertAlmostEqual(result["normalized_lambda_min"], 1.0)
        self.assertTrue(result["positive_definite"])

    def test_collinear_pair_is_not_positive_definite(self):
        result = geometry_metrics([0.0, 0.0], [[1.0, 0.0], [-2.0, 0.0]])
        self.assertAlmostEqual(result["lambda_min"], 0.0)
        self.assertFalse(result["positive_definite"])

    def test_adding_reference_cannot_reduce_geometry_eigenvalue(self):
        pair = geometry_metrics([0.0, 0.0], [[1.0, 0.0], [2.0, 0.1]])
        active = geometry_metrics(
            [0.0, 0.0],
            [[1.0, 0.0], [2.0, 0.1], [0.0, 1.0]],
        )
        self.assertGreaterEqual(active["lambda_min"], pair["lambda_min"])

    def test_fixed_pair_reports_smaller_angle_to_collinearity(self):
        result = fixed_pair_metrics(
            [0.0, 0.0],
            [[1.0, 0.0], [-math.sqrt(3.0) / 2.0, 0.5]],
        )
        self.assertAlmostEqual(
            result["noncollinearity_angle_rad"],
            math.pi / 6.0,
        )
```

- [ ] **Step 2: Run the primitive tests and verify RED**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_geometric_stability.GeometryPrimitiveTests -v
```

Expected: import failure because the analyzer does not exist.

- [ ] **Step 3: Implement strict geometry primitives**

Use finite two-dimensional arrays and `eigvalsh`:

```python
def _point(value: object, name: str) -> np.ndarray:
    try:
        point = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError) as error:
        raise InputIntegrityError(f"{name} is not a numeric point") from error
    if point.shape != (2,) or not np.isfinite(point).all():
        raise InputIntegrityError(f"{name} is not a finite 2D point")
    return point

def geometry_metrics(observer: object, references: object) -> dict:
    center = _point(observer, "observer")
    points = np.asarray(references, dtype=float)
    if points.ndim != 2 or points.shape[1:] != (2,) or points.shape[0] < 2:
        raise InputIntegrityError("geometry requires at least two 2D references")
    differences = center[None, :] - points
    ranges = np.linalg.norm(differences, axis=1)
    if not np.isfinite(points).all() or np.any(ranges <= 0.0):
        raise InputIntegrityError("geometry contains a coincident or non-finite reference")
    directions = differences / ranges[:, None]
    matrix = directions.T @ directions
    eigenvalues = np.linalg.eigvalsh(matrix)
    minimum, maximum = map(float, eigenvalues)
    return {
        "reference_count": int(points.shape[0]),
        "lambda_min": minimum,
        "lambda_max": maximum,
        "normalized_lambda_min": minimum / maximum,
        "finite": True,
        "positive_definite": minimum > 0.0,
    }
```

Implement `fixed_pair_metrics` from the clipped direction dot product and
the two-dimensional determinant. Reject any count other than two.

- [ ] **Step 4: Run the primitive tests and verify GREEN**

Run the Step 2 command.
Expected: all primitive tests pass.

- [ ] **Step 5: Write failing immutable-trajectory tests**

Create a two-frame fixture with real field names:

```python
def trajectory_fixture() -> dict:
    return {
        "config": {
            "bases": [[-1.0, 0.0], [0.0, 1.0]],
            "num": 1,
            "execute": {"time-step": 0.5},
            "formation": {"parts": 1, "bases-id": [[0, 1]]},
        },
        "state": [
            {
                "robots": [
                    {
                        "id": 1,
                        "state": {"x": 0.0, "y": 0.0},
                        "cvt": {"center": [1.0, 1.0]},
                    }
                ]
            },
            {
                "robots": [
                    {
                        "id": 1,
                        "state": {"x": 0.1, "y": 0.2},
                        "cvt": {"center": [1.1, 1.2]},
                    }
                ]
            },
        ],
    }

class TrajectoryContractTests(unittest.TestCase):
    def test_loader_returns_truth_targets_and_time_step(self):
        path, digest = self.write_trajectory(trajectory_fixture())
        loaded = load_trajectory(path, expected_sha256=digest)
        self.assertEqual(loaded["time_step"], 0.5)
        self.assertEqual(loaded["truth"][1][1], [0.1, 0.2])
        self.assertEqual(loaded["targets"][1][1], [1.1, 1.2])

    def test_wrong_hash_missing_target_duplicate_id_and_nonfinite_fail_closed(self):
        path, digest = self.write_trajectory(trajectory_fixture())
        with self.assertRaises(InputIntegrityError):
            load_trajectory(path, expected_sha256="0" * 64)
        for mutation in (
            self.remove_target,
            self.duplicate_robot_id,
            self.insert_nonfinite_state,
        ):
            bad_path, bad_digest = self.write_trajectory(
                mutation(trajectory_fixture())
            )
            with self.assertRaises(InputIntegrityError):
                load_trajectory(bad_path, expected_sha256=bad_digest)
```

- [ ] **Step 6: Run trajectory tests and verify RED**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_geometric_stability.TrajectoryContractTests -v
```

Expected: failures because `load_trajectory` is absent.

- [ ] **Step 7: Implement the immutable trajectory loader**

The loader must:

1. hash the file before reading;
2. reject non-standard JSON constants through `parse_constant`;
3. require top-level `config` and `state`;
4. require one finite positive `config.execute.time-step`;
5. require each frame to contain exactly one record for every configured
   robot ID;
6. read truth from `robot.state.{x,y}`;
7. read the time-varying target from `robot.cvt.center`; and
8. hash the file again after reading and require the same digest.

Return:

```python
{
    "path": str(path.resolve()),
    "sha256": expected_sha256,
    "config": config,
    "frame_count": len(frames),
    "time_step": float(time_step),
    "truth": {frame_index: {robot_id: [x, y]}},
    "targets": {frame_index: {robot_id: [target_x, target_y]}},
}
```

- [ ] **Step 8: Run Task 1 tests and commit**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_geometric_stability.GeometryPrimitiveTests \
  tests.test_analyze_geometric_stability.TrajectoryContractTests -v
git diff --check
```

Expected: all tests pass and no whitespace error.

Commit:

```bash
git add scripts/diagnostics/analyze_geometric_stability.py \
  tests/test_analyze_geometric_stability.py
git commit -m "feat(diagnostics): add geometric stability primitives"
```

---

### Task 2: Verified paired streaming and scientific aggregations

**Files:**
- Modify: `scripts/diagnostics/analyze_geometric_stability.py`
- Modify: `tests/test_analyze_geometric_stability.py`

**Interfaces:**
- Consumes:

```python
compare_warm_start_recovery._load_bundle
compare_warm_start_recovery._verified_rows
compare_warm_start_recovery._compact_key
compare_warm_start_recovery._reconcile_replay_row_inputs
compare_warm_start_recovery._paired_inputs_equal
compare_warm_start_recovery._validate_policy_row
compare_warm_start_recovery._verify_bundle_unchanged
analyze_localization_failures._normalized_squared_error
replay_localization_calibration.fixed_references
```

- Produces:

```python
def analyze_geometric_stability(
    comparison_path: Path,
    *,
    expected_comparison_sha256: str,
    expected_parent_manifest_sha256: str,
    output_dir: Path | None = None,
    live_guard: Callable[[], None] | None = None,
) -> dict:
    """Verify paired evidence, aggregate geometry/error/availability, publish optionally."""
```

- Report families:

```python
{
    "geometry": {
        "true_dynamic_all_primary": {},
        "true_fixed_pair_all_primary": {},
        "estimated_dynamic_finite": {},
        "estimated_fixed_pair_finite": {},
        "target_tracking": {},
    },
    "absolute_error": {},
    "availability": {},
    "calibration": {},
}
```

- [ ] **Step 1: Write failing trust-root and row-pairing tests**

Build strict/restart real-schema bundle fixtures and one comparison fixture.
Assert:

```python
class EvidenceIntegrityTests(unittest.TestCase):
    def test_exact_trust_roots_and_every_pair_are_verified(self):
        fixture = self.write_complete_fixture()
        report = analyze_geometric_stability(**fixture.call_args())
        self.assertEqual(report["integrity"]["paired_rows"], 8)
        self.assertTrue(report["integrity"]["paired_inputs_equal"])
        self.assertTrue(report["integrity"]["source_hashes_unchanged"])

    def test_wrong_comparison_or_parent_hash_fails_before_streaming(self):
        fixture = self.write_complete_fixture()
        with self.assertRaises(InputIntegrityError):
            analyze_geometric_stability(
                fixture.comparison,
                expected_comparison_sha256="0" * 64,
                expected_parent_manifest_sha256=fixture.parent_sha256,
            )
        with self.assertRaises(InputIntegrityError):
            analyze_geometric_stability(
                fixture.comparison,
                expected_comparison_sha256=fixture.comparison_sha256,
                expected_parent_manifest_sha256="f" * 64,
            )

    def test_key_noise_reference_or_truth_mutation_fails_closed(self):
        for mutation in (
            self.mutate_restart_key,
            self.mutate_restart_noise,
            self.mutate_restart_reference,
            self.mutate_restart_truth,
        ):
            fixture = self.write_complete_fixture(mutation=mutation)
            with self.assertRaises(InputIntegrityError):
                analyze_geometric_stability(**fixture.call_args())
```

- [ ] **Step 2: Run integrity tests and verify RED**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_geometric_stability.EvidenceIntegrityTests -v
```

Expected: failures because the top-level analyzer is absent.

- [ ] **Step 3: Implement trust-root loading and paired streaming**

The top-level function must:

1. verify the comparison hash before reading;
2. require comparison schema
   `cbf2026-warm-start-recovery-comparison-v1`;
3. read the strict/restart child paths and hashes from `comparison["source"]`;
4. require the exact parent-manifest hash;
5. load both child bundles with `_load_bundle`;
6. load the immutable trajectory from the child manifest's `input_data`;
7. stream strict/restart rows with `zip_longest`;
8. require exact keys, paired external inputs, replay-input reconciliation,
   and policy-state reconciliation;
9. hash strict/restart bundles and comparison again after streaming; and
10. reject any row or trajectory cardinality mismatch.

Use the completed policy identifiers:

```python
STRICT_PREVIOUS_POLICY = "strict_previous_v1"
RESTART_BEFORE_FIRST_FINITE_POLICY = (
    "deployment_restart_before_first_finite_v1"
)
```

- [ ] **Step 4: Write failing geometry and denominator tests**

Use two seeds over the same two-frame truth trajectory:

```python
class ScientificAggregationTests(unittest.TestCase):
    def test_true_geometry_is_counted_once_per_trajectory_tuple(self):
        report = self.analyze(self.two_seed_fixture())
        geometry = report["geometry"]["true_dynamic_all_primary"]["overall"]
        self.assertEqual(geometry["trajectory_tuple_count"], 2)
        self.assertEqual(geometry["range_seed_repetitions_verified"], 2)

    def test_estimated_geometry_uses_only_finite_restart_rows(self):
        report = self.analyze(self.one_invalid_restart_row_fixture())
        estimated = report["geometry"]["estimated_dynamic_finite"]["overall"]
        self.assertEqual(estimated["denominator"], 1)
        self.assertEqual(
            report["availability"]["overall"]["primary_attempts"],
            2,
        )

    def test_depth_time_squad_and_seed_strata_reconcile(self):
        report = self.analyze(self.seven_depth_two_squad_fixture())
        error = report["absolute_error"]
        self.assertEqual(
            sum(item["denominator"] for item in error["by_depth"].values()),
            error["overall"]["denominator"],
        )
        self.assertEqual(
            sum(item["denominator"] for item in error["by_time_bin"].values()),
            error["overall"]["denominator"],
        )

    def test_q_is_recomputed_from_error_and_covariance(self):
        report = self.analyze(self.q_fixture(error=[3.1, 0.0], covariance=np.eye(2)))
        self.assertEqual(report["calibration"]["overall"]["q_above_9"], 1)

    def test_error_and_containment_denominators_remain_separate(self):
        report = self.analyze(self.invalid_covariance_fixture())
        calibration = report["calibration"]["overall"]
        self.assertGreater(
            calibration["converged_error_denominator"],
            calibration["finite_q_denominator"],
        )
```

- [ ] **Step 5: Run aggregation tests and verify RED**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_geometric_stability.ScientificAggregationTests -v
```

Expected: failures because scientific accumulation is absent.

- [ ] **Step 6: Implement frozen strata and additive accumulators**

Use these frozen time bins:

```python
TIME_BIN_LABELS = (
    "0_to_lt_50_s",
    "50_to_lt_100_s",
    "100_to_lt_150_s",
    "150_to_lt_200_s",
    "200_to_250_s",
)
```

For every dynamic primary restart row:

1. derive squad and local depth from the frozen config;
2. reconstruct fixed and active reference truth positions;
3. compute true geometry once per `(frame_index, robot_id)` and require exact
   equality across all 20 seed repetitions;
4. reconstruct current-frame estimated reference positions in topological
   row order from bases and earlier finite restart rows;
5. compute estimated geometry only if the observer and all admitted
   references have finite positions;
6. compute true and estimated target-tracking deviations against
   `cvt.center`;
7. record absolute error whenever `error_norm` is finite;
8. recompute \(q\) with `_normalized_squared_error`;
9. retain seed-level additive counts and finite sample lists for exact
   quantiles; and
10. never silently omit an inapplicable row—increment an explicit
    inapplicability reason.

Use `np.percentile` with fixed probabilities
`(50.0, 90.0, 95.0, 99.0)`.
Report `maximum` but do not turn it into a pass/fail gate.

- [ ] **Step 7: Write failing availability and lineage tests**

```python
class AvailabilityTests(unittest.TestCase):
    def test_longest_unavailable_streak_uses_seed_robot_series(self):
        report = self.analyze(
            self.status_series(["converged", "invalid", "invalid", "converged"])
        )
        self.assertEqual(
            report["availability"]["overall"]["maximum_consecutive_unavailable_frames"],
            2,
        )

    def test_first_finite_lag_is_stratified_without_dropping_never_finite_series(self):
        report = self.analyze(self.first_finite_and_never_finite_fixture())
        lineage = report["availability"]["first_finite_acquisition"]
        self.assertEqual(lineage["acquired_series"], 1)
        self.assertEqual(lineage["never_acquired_series"], 1)

    def test_failure_reasons_reconcile_to_all_primary_attempts(self):
        report = self.analyze(self.mixed_status_fixture())
        overall = report["availability"]["overall"]
        self.assertEqual(
            sum(overall["attempt_status_counts"].values()),
            overall["primary_attempts"],
        )
```

- [ ] **Step 8: Implement availability and lineage aggregation**

Maintain state keyed by `(seed, robot_id)` for the restart dynamic arm.
Availability means `attempt_status == "converged"` for attempt-level metrics;
retained stale state must be reported separately.
Track:

- counts by attempt status and raw failure reason;
- converged, finite retained, stale, invalid, and failed denominators;
- first finite acquisition frame;
- frames since first finite acquisition;
- maximum consecutive nonconverged and nonfinite-retained streaks; and
- never-acquired series.

- [ ] **Step 9: Write failing dependency and two-reference branch tests**

Expose two focused diagnostic helpers:

```python
def shared_uav_ancestor_metrics(
    active_uav_references: list[int],
    lineage_by_robot: dict[int, frozenset[int]],
) -> dict:
    """Count UAV-reference pairs with shared uncertain UAV ancestry."""

def opposite_baseline_side(
    estimate: object,
    truth: object,
    reference_positions: object,
    *,
    tolerance_metres: float = 1e-6,
) -> bool | None:
    """Return branch-side mismatch for exactly two references, or None near baseline."""
```

Tests:

```python
class DependencyAndBranchTests(unittest.TestCase):
    def test_shared_uav_ancestor_counts_overlap_not_known_bases(self):
        lineage = {
            2: frozenset({1, 2}),
            3: frozenset({1, 3}),
            4: frozenset({4}),
        }
        result = shared_uav_ancestor_metrics([2, 3, 4], lineage)
        self.assertEqual(result["reference_pair_count"], 3)
        self.assertEqual(result["pairs_with_shared_uav_ancestor"], 1)
        self.assertEqual(result["shared_uav_ancestor_instances"], 1)

    def test_opposite_side_detects_mirror_proxy(self):
        references = [[-1.0, 0.0], [1.0, 0.0]]
        self.assertTrue(
            opposite_baseline_side(
                estimate=[0.0, -1.0],
                truth=[0.0, 1.0],
                reference_positions=references,
            )
        )
        self.assertFalse(
            opposite_baseline_side(
                estimate=[0.0, 0.5],
                truth=[0.0, 1.0],
                reference_positions=references,
            )
        )
        self.assertIsNone(
            opposite_baseline_side(
                estimate=[0.0, 0.0],
                truth=[0.0, 1.0],
                reference_positions=references,
            )
        )

    def test_calibration_is_stratified_by_uav_refs_ancestry_and_side(self):
        report = self.analyze(self.dependency_and_branch_fixture())
        mechanisms = report["calibration"]["mechanism_diagnostics"]
        self.assertIn("by_uav_reference_count", mechanisms)
        self.assertIn("by_shared_uav_ancestor_pair_count", mechanisms)
        self.assertIn("two_reference_opposite_baseline_side", mechanisms)
```

- [ ] **Step 10: Implement compact mechanism diagnostics**

At each frame and squad, build dynamic UAV-reference lineage in increasing
local-index order:

```python
lineage_by_robot[robot_id] = frozenset(
    {robot_id}
    | {
        ancestor
        for reference_id in active_uav_reference_ids
        for ancestor in lineage_by_robot[reference_id]
    }
)
```

Known bases are excluded from shared-error ancestry because their configured
covariance is zero.
For each converged restart row, stratify containment and recomputed \(q>9\)
by:

- active UAV-reference count;
- number of reference pairs sharing at least one UAV ancestor; and
- for exactly two estimated references, whether estimate and truth lie on
  opposite sides of the reference baseline.

The side test uses signed perpendicular distance and returns `None` when the
estimate or truth is within `1e-6` m of the baseline.
Name this an `opposite_baseline_side` diagnostic, not proof of a mirror
solution.

- [ ] **Step 11: Run Task 2 tests and commit**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_geometric_stability.EvidenceIntegrityTests \
  tests.test_analyze_geometric_stability.ScientificAggregationTests \
  tests.test_analyze_geometric_stability.AvailabilityTests \
  tests.test_analyze_geometric_stability.DependencyAndBranchTests -v
git diff --check
```

Expected: all Task 2 tests pass.

Commit:

```bash
git add scripts/diagnostics/analyze_geometric_stability.py \
  tests/test_analyze_geometric_stability.py
git commit -m "feat(diagnostics): analyze geometric error stability"
```

---

### Task 3: Compact publication, disk enforcement, and CLI

**Files:**
- Modify: `scripts/diagnostics/analyze_geometric_stability.py`
- Modify: `tests/test_analyze_geometric_stability.py`

**Interfaces:**
- Produces exactly:

```text
geometric-stability.json
geometric-stability.md
```

- CLI:

```bash
python -m scripts.diagnostics.analyze_geometric_stability \
  --comparison PATH \
  --expected-comparison-sha256 SHA256 \
  --expected-parent-manifest-sha256 SHA256 \
  --output-dir PATH
```

- [ ] **Step 1: Write failing output and resource tests**

```python
class PublicationTests(unittest.TestCase):
    def test_output_is_atomic_exact_and_below_ten_megabytes(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        analyze_geometric_stability(**fixture.call_args(output_dir=output))
        self.assertEqual(
            {path.name for path in output.iterdir()},
            {OUTPUT_JSON_NAME, OUTPUT_MARKDOWN_NAME},
        )
        self.assertLess(run_diagnostic.allocated_bytes(output), 10_000_000)

    def test_existing_output_or_nested_source_output_is_rejected(self):
        fixture = self.write_complete_fixture()
        for output in (fixture.restart_bundle / "analysis", self.root / "exists"):
            output.mkdir(parents=True)
            with self.assertRaises((InputIntegrityError, AnalysisLimitError)):
                analyze_geometric_stability(**fixture.call_args(output_dir=output))

    def test_live_floor_failure_leaves_no_published_output(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        with self.assertRaises(AnalysisLimitError):
            analyze_geometric_stability(
                **fixture.call_args(output_dir=output),
                live_guard=lambda: (_ for _ in ()).throw(
                    AnalysisLimitError("below live floor")
                ),
            )
        self.assertFalse(output.exists())

    def test_markdown_states_exploratory_claim_boundary(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        analyze_geometric_stability(**fixture.call_args(output_dir=output))
        text = (output / OUTPUT_MARKDOWN_NAME).read_text()
        self.assertIn("post-hoc exploratory", text)
        self.assertIn("not a deterministic true-error bound", text)
        self.assertIn("one truth trajectory", text)
```

- [ ] **Step 2: Run publication tests and verify RED**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_geometric_stability.PublicationTests -v
```

Expected: failures because publication is not implemented.

- [ ] **Step 3: Implement fail-closed atomic publication**

Use a sibling staging directory and enforce:

```python
START_BYTES = 8_000_000_000
HARD_FLOOR_BYTES = 6_000_000_000
OUTPUT_CAP_BYTES = 10_000_000
LIVE_CHECK_INTERVAL_ROWS = 10_000
```

The JSON must include:

```python
{
    "schema": SCHEMA_ID,
    "status": "completed",
    "source": {
        "comparison_path": str,
        "comparison_sha256": str,
        "parent_manifest_sha256": str,
        "strict_bundle": str,
        "restart_bundle": str,
        "input_data_path": str,
        "input_data_sha256": str,
        "source_commit": str,
        "analyzer_path": str,
        "analyzer_sha256": str,
    },
    "protocol": {
        "scope": "post-hoc exploratory",
        "truth_trajectory_count": 1,
        "range_noise_seed_count": 20,
        "time_bins_seconds": [0, 50, 100, 150, 200, 250],
        "primary_geometry": "dynamic active-set geometry matrix",
        "statistical_unit": "range-noise seed nested in one truth trajectory",
    },
    "integrity": {},
    "geometry": {},
    "absolute_error": {},
    "availability": {},
    "calibration": {},
    "claim_boundary": [
        "not a deterministic true-error bound",
        "not an arbitrary-depth stability result",
        "not an unconditional controller-safety result",
        "not a cross-trajectory generality result",
    ],
}
```

Call the live guard before reading, every 10,000 rows, before each persistent
write, after each write, and immediately before publication.
Rehash all source files after the last read and before rename.

- [ ] **Step 4: Implement deterministic Markdown and CLI**

The Markdown must contain:

1. source paths and hashes;
2. theory/empirical separation;
3. true active geometry and fixed-pair explanatory geometry;
4. estimated geometry and FIM validity denominators;
5. absolute-error profiles in metres;
6. availability and longest outages;
7. epsilon containment and \(q>9\) as calibration diagnostics;
8. q/containment strata by UAV-reference count, shared UAV ancestry, and the
   two-reference opposite-baseline-side proxy;
9. one-trajectory and post-hoc limitations; and
10. no pass/fail or paper recommendation not present in the JSON.

Serialize JSON with sorted keys, finite-number validation, UTF-8, and one
terminal newline.

- [ ] **Step 5: Run focused and full regression tests**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_geometric_stability -v
conda run -n cbf_env python -m unittest discover \
  -s tests -p 'test_*.py' -v
git diff --check
```

Expected: all new tests and all existing Python diagnostic tests pass.

- [ ] **Step 6: Request two-stage code review and commit**

Dispatch one reviewer against the approved spec and one reviewer against code
standards, tests, integrity, and scope.
Fix every Critical or Important issue and rerun Step 5.

Commit:

```bash
git add scripts/diagnostics/analyze_geometric_stability.py \
  tests/test_analyze_geometric_stability.py
git commit -m "feat(diagnostics): publish geometric stability audit"
```

---

### Task 4: Freeze theory note and exploratory execution protocol

**Files:**
- Create: `docs/diagnostics/2026-07-29-geometric-nondegeneracy-theory.md`
- Create: `docs/diagnostics/2026-07-29-geometric-stability-protocol.md`
- Create: `docs/diagnostics/reviews/2026-07-29-geometric-stability-protocol-review.md`

**Interfaces:**
- Consumes: approved design, committed analyzer source hash, exact Gate 2
  trust roots.
- Produces: one mathematical note and one immutable exploratory execution
  command.

- [ ] **Step 1: Write the mathematical note**

The note must distinguish:

```text
P_i := Phi_i^{-1}                  modeled inverse-FIM matrix
C_i := Cov(p_i - p_hat_i)          true estimator-error covariance
G_i := sum_j g_ij g_ij^T           unweighted active geometry matrix
```

Include proofs of:

\[
G_i\succeq\gamma_iI,\quad
v_{ij}\le\bar v_i
\;\Longrightarrow\;
\Phi_i\succeq\frac{\gamma_i}{\bar v_i}I
\]

and finite lower-index-DAG induction.
Include the finite-depth model recursion

\[
B_0=0,\qquad
B_d=\frac{\sigma_r^2+B_{d-1}}{\gamma},
\]

but state that it need not contract and is not a true-error bound.
Include the fixed-pair determinant as a sufficient special case and explain
why the current soft task CBF and range upper bound do not guarantee it.

- [ ] **Step 2: Write the frozen exploratory protocol**

Record:

- exact input paths and hashes from Global Constraints;
- source commit and analyzer SHA-256;
- exact CLI command;
- output directory;
- one execution with no automatic retry;
- disk gates and probes;
- expected two output filenames;
- predeclared report families and denominators;
- five 50-second bins;
- no confirmatory gate and no \(p\)-value;
- exact claim boundary; and
- abort behavior for trust-root, geometry, target, disk, output-cap, or
  integrity failure.

The production command is:

```bash
conda run -n cbf_env python -m \
  scripts.diagnostics.analyze_geometric_stability \
  --comparison \
  /private/tmp/cbf2026-warm-start-recovery-analysis/registered-gate-2/warm-start-recovery-comparison.json \
  --expected-comparison-sha256 \
  ef8014915b3cced6a75e094b3d16f01f17b5f881c6357dff6b885b6e871498ca \
  --expected-parent-manifest-sha256 \
  c110275910c3e44ecf97c66ba7346fbb0b340cfb844d050c0ecf56944e5ff55a \
  --output-dir \
  /private/tmp/cbf2026-geometric-stability-analysis/exploratory-v1
```

- [ ] **Step 3: Independently review the protocol**

The review must explicitly check:

- formulas and model/true-covariance distinction;
- dynamic active-set geometry as primary;
- fixed pair as explanatory only;
- one-trajectory geometry replication;
- seed-level estimator statistics;
- immutable trust roots;
- no raw duplication;
- disk and output caps;
- no retry; and
- paper-edit gate.

Resolve every Critical or Important issue before execution.

- [ ] **Step 4: Verify and commit**

Run:

```bash
rg -n 'TBD|TODO|FIXME|PLACEHOLDER' \
  docs/diagnostics/2026-07-29-geometric-nondegeneracy-theory.md \
  docs/diagnostics/2026-07-29-geometric-stability-protocol.md \
  docs/diagnostics/reviews/2026-07-29-geometric-stability-protocol-review.md
git diff --check
```

Expected: no placeholder match and no whitespace error.

Commit:

```bash
git add \
  docs/diagnostics/2026-07-29-geometric-nondegeneracy-theory.md \
  docs/diagnostics/2026-07-29-geometric-stability-protocol.md \
  docs/diagnostics/reviews/2026-07-29-geometric-stability-protocol-review.md
git commit -m "docs(diagnostics): register geometric stability audit"
```

---

### Task 5: Run once, independently audit, and record evidence

**Files:**
- Create externally: `/private/tmp/cbf2026-geometric-stability-analysis/exploratory-v1/geometric-stability.json`
- Create externally: `/private/tmp/cbf2026-geometric-stability-analysis/exploratory-v1/geometric-stability.md`
- Create: `docs/diagnostics/2026-07-29-geometric-stability.md`
- Create: `docs/diagnostics/reviews/2026-07-29-geometric-stability-evidence-review.md`

**Interfaces:**
- Consumes: committed and reviewed Task 4 protocol.
- Produces: immutable compact exploratory evidence and a reviewed repository
  record.

- [ ] **Step 1: Perform exact preflight**

Run read-only checks:

```bash
df -Pk /private/tmp
du -sk /private/tmp/cbf2026-warm-start-recovery
du -sk /private/tmp/cbf2026-warm-start-recovery-analysis
du -sk /Users/xirhxq/.cache/codex-runtimes
shasum -a 256 \
  /private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/manifest.json \
  /private/tmp/cbf2026-warm-start-recovery-analysis/registered-gate-2/warm-start-recovery-comparison.json
git status --short --branch
```

Require at least 8 GB free and exact registered hashes.
Require status to contain no uncommitted tracked changes and only the permitted
`?? build-diagnostic/`.

- [ ] **Step 2: Execute the registered command exactly once**

Run the command frozen in Task 4.
Do not alter thresholds, paths, or source after inspecting output.
Do not automatically retry on failure.

- [ ] **Step 3: Verify compact evidence**

Run:

```bash
du -sk /private/tmp/cbf2026-geometric-stability-analysis/exploratory-v1
shasum -a 256 \
  /private/tmp/cbf2026-geometric-stability-analysis/exploratory-v1/geometric-stability.json \
  /private/tmp/cbf2026-geometric-stability-analysis/exploratory-v1/geometric-stability.md
df -Pk /private/tmp
```

Require output below 10 MB, final free space above 6 GB, and no source hash
change.

- [ ] **Step 4: Write the evidence report**

Record exact:

- source commit and hashes;
- output paths and hashes;
- true dynamic and fixed-pair geometry minima and quantiles;
- estimated geometry and FIM denominators;
- absolute-error quantiles by squad-local index, squad, and 50-second bin;
- depth-seven p95 and full maximum in metres;
- availability rates and longest outage;
- containment and \(q>9\) separately;
- whether time bins show sustained monotone growth;
- whether any geometry/FIM singularity was observed;
- exploratory and one-trajectory limitations; and
- whether evidence justifies a narrow paper theory update.

- [ ] **Step 5: Perform an independent evidence audit**

The reviewer must independently:

1. hash every source and output;
2. stream all strict/restart raw rows;
3. reproduce trajectory-tuple geometry without using analyzer output;
4. reproduce depth/time absolute-error quantiles;
5. reproduce longest outages;
6. reproduce containment and \(q>9\);
7. verify no raw duplication and output cap;
8. inspect claim language; and
9. report Critical, Important, and Minor issues.

Resolve every Critical or Important issue.
Do not overwrite the authoritative output; if analyzer code must change, stop
and create a new design for any rerun.

- [ ] **Step 6: Run final regression and commit evidence records**

Run:

```bash
conda run -n cbf_env python -m unittest discover \
  -s tests -p 'test_*.py' -v
git diff --check
git status --short
```

Commit:

```bash
git add \
  docs/diagnostics/2026-07-29-geometric-stability.md \
  docs/diagnostics/reviews/2026-07-29-geometric-stability-evidence-review.md
git commit -m "docs(diagnostics): record geometric stability evidence"
```

---

### Task 6: Update DRA on isolated main

**Files:**
- Create in `/private/tmp/dra-cbf2026-diagnostic`: `papers/cbf2026/theory/2026-07-29-geometric-nondegeneracy-and-empirical-stability.md`
- Create in `/private/tmp/dra-cbf2026-diagnostic`: `meta-log/2026-07-29-cbf2026-geometric-stability.md`
- Modify in `/private/tmp/dra-cbf2026-diagnostic`: `papers/cbf2026/status.md`
- Modify in `/private/tmp/dra-cbf2026-diagnostic`: `papers/cbf2026/open-questions.md`
- Modify in `/private/tmp/dra-cbf2026-diagnostic`: `papers/cbf2026/timeline.md`

**Interfaces:**
- Consumes: reviewed Task 5 evidence.
- Produces: a claim-bounded research record on DRA `main`.

- [ ] **Step 1: Verify isolated DRA state**

Run:

```bash
git status --short --branch
git branch --show-current
```

Expected in `/private/tmp/dra-cbf2026-diagnostic`:

```text
## main...origin/main [ahead 19]
main
```

If tracked or untracked changes appear, stop rather than touching the primary
DRA checkout.

- [ ] **Step 2: Write the theory record**

Include:

- \(P_i\), \(C_i\), and \(G_i\) definitions;
- the active-geometry FIM lower bound;
- finite-DAG induction;
- optional finite-depth noncontractive recursion;
- fixed-pair sufficient special case;
- dynamic-reference PSD scope;
- conditional robust-CBF interface;
- exact empirical evidence paths and hashes; and
- prohibited claims.

- [ ] **Step 3: Update status, open questions, timeline, and meta-log**

Set the next research question to choosing task-scale confirmation thresholds:

- minimum sampled dynamic geometry score;
- depth-seven error p95;
- acceptable late-versus-middle p95 change;
- minimum estimator availability; and
- maximum acceptable outage.

State that a prospective multi-trajectory run requires a separate approved
design and must prioritize independent truth trajectories over additional
seeds on the same trajectory.

- [ ] **Step 4: Review and commit DRA**

Verify exact links and hashes against Task 5, then run:

```bash
git diff --check
git status --short
```

Commit only the five named records:

```bash
git add \
  papers/cbf2026/theory/2026-07-29-geometric-nondegeneracy-and-empirical-stability.md \
  meta-log/2026-07-29-cbf2026-geometric-stability.md \
  papers/cbf2026/status.md \
  papers/cbf2026/open-questions.md \
  papers/cbf2026/timeline.md
git commit -m "docs(cbf2026): record geometric stability evidence"
```

Do not push DRA because its isolated `main` already contains 18 pre-existing
unpushed commits before the prior CBF2026 update.

---

### Task 7: Apply only evidence-supported paper changes

**Files:**
- Modify conditionally in a new paper worktree: `main.tex`
- Build only in that isolated paper worktree or an external temporary build directory.

**Interfaces:**
- Consumes: reviewed Task 5 evidence and Task 6 DRA claim boundary.
- Produces: a narrow theory/experiment correction or an explicit no-edit
  decision.

- [ ] **Step 1: Make the paper-edit decision before touching the paper**

Edit only if Task 5 independently verifies:

1. dynamic active-set geometry remains nonsingular on every sampled primary
   trajectory tuple;
2. absolute-error time bins show no sustained monotone divergence;
3. depth and long-tail growth are reported without hiding outages;
4. all geometry, error, availability, and calibration denominators reconcile;
   and
5. the evidence reviewer has no open Critical or Important issue.

Otherwise write the no-edit decision into the Task 5 report and end this task
without creating a paper worktree.

- [ ] **Step 2: Create an isolated paper worktree if justified**

From `/Users/xirhxq/Documents/Clones/cbf/papers/CBF2026`, preserve its dirty
working tree and current `main` commit.
Use the `superpowers:using-git-worktrees` skill to create:

```text
/private/tmp/cbf2026-paper-geometric-stability
```

on branch:

```text
codex/cbf2026-geometric-stability-paper
```

Do not copy uncommitted `main.pdf`, rebuttal files, or `AGENTS.md`.

- [ ] **Step 3: Revise the theory narrowly**

In `main.tex`:

- retain inverse FIM as a covariance approximation/CRLB, not an upper bound;
- define or explain the active geometry matrix \(G_i\);
- add the conditional implication
  \(G_i\succeq\gamma_iI\Rightarrow
  \Phi_i\succeq(\gamma_i/\bar v_i)I\);
- add finite lower-index-DAG induction for modeled covariance;
- present fixed-pair noncollinearity only as a sufficient special case;
- state that dynamic references add PSD model information for fixed existing
  terms;
- retain the conditional robust-CBF premise;
- state explicitly that the soft formation task and range CBF do not
  independently prove the geometry condition; and
- use MC only for finite-depth actual-error, availability, and calibration
  evidence.

Do not introduce a deterministic true-error bound, cross-covariance estimator,
new radius coefficient, or controller-in-the-loop estimator claim.

- [ ] **Step 4: Add only reviewed empirical results**

Report:

- dynamic active geometry;
- fixed-pair near-degeneracy as motivation for dynamic information;
- absolute error by depth and time;
- longest estimator outage;
- coefficient-3 containment and \(q>9\) as calibration limitations; and
- one-trajectory exploratory scope.

Do not call this a confirmatory Monte Carlo study.

- [ ] **Step 5: Build without touching the dirty primary paper checkout**

Run the repository build in the isolated worktree.
Require successful PDF generation, no undefined references, no missing
citations, and no BibTeX warning about missing DOI entries.
Do not use `open`.

- [ ] **Step 6: Independently review paper claims and commit**

The reviewer must compare every numerical sentence with the reviewed JSON and
verify that no conditional theorem became unconditional prose.
Resolve every Critical or Important issue and rebuild.

Commit only tracked paper source changes:

```bash
git add main.tex
git commit -m "docs(paper): clarify geometric stability evidence"
```

Do not push without a separate decision because the source paper repository's
`main` is already ahead of origin and the new worktree branch has no upstream.

---

## Final verification

- [ ] Run all diagnostic Python tests in `cbf_env`.
- [ ] Run `git diff --check` in the CBF code worktree, DRA worktree, and any paper worktree.
- [ ] Confirm the code worktree contains only permitted `?? build-diagnostic/`.
- [ ] Confirm the primary paper and primary DRA checkouts are unchanged.
- [ ] Confirm `/private/tmp` remains above the 6 GB floor.
- [ ] Confirm compact analysis remains below 10 MB and no raw evidence was duplicated.
- [ ] Confirm every committed evidence hash matches the live file.
- [ ] Confirm no Critical or Important review issue remains.
- [ ] Record whether a separate prospective multi-trajectory design is warranted; do not launch it from this plan.
