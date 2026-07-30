# CBF2026 Predictive WNLS Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement and audit a distributed predictive WNLS estimator that prevents unbounded stale publication and stale-anchor cascades, while preserving the original range objective/FIM and reporting every error-versus-availability trade-off.

**Architecture:** Add one pure-NumPy estimator module, one immutable Stage 0 fixture extractor, one paired Stage 1 replay, one compact analyzer, and one protocol registrar. The estimator propagates the last applied command for at most two published prediction frames, filters UAV anchors by current-frame freshness and recursive base provenance, evaluates deterministic WNLS starts, applies a covariance-normalized branch-consistency gate, and retains a private but unpublished seed for truth-free re-acquisition. Stage 1 reuses the old unbounded-input trajectory only as development evidence; a separate future Stage 2 plan must generate hard-bounded trajectories before the paper gate can open.

**Tech Stack:** Python 3 in conda environment `cbf_env`, NumPy, Python standard library (`argparse`, `dataclasses`, `gzip`, `hashlib`, `json`, `math`, `pathlib`, `subprocess`, `unittest`), existing CBF2026 diagnostic integrity and disk helpers.

## Global Constraints

- Work only in `/private/tmp/cbf2026-diagnostic` on branch `codex/cbf2026-diagnostic`.
- Preserve untracked `build-diagnostic/`; never stage, delete, or rewrite it.
- Run every Python command with `conda run -n cbf_env`.
- The binding design is `docs/superpowers/specs/2026-07-30-cbf2026-bounded-predictive-wnls-recovery-design.md` at commit `a2ae1f0`.
- Do not modify `scripts/diagnostics/replay_localization_calibration.py`; its SHA-256 must remain `0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8`.
- Do not rerun or overwrite `/private/tmp/cbf2026-geometric-stability-analysis/exploratory-v1`.
- Preserve the distributed strictly lower-index DAG, fixed CBF distance pair, full variable-weight residual Jacobian, range-only FIM, and coefficient-3 fresh epsilon.
- The estimator remains offline and cannot change controller, CBF, CVT, vehicle, communication, truth trajectory, or applied command.
- Use frame-\((k-1)\) `opt.result.{vx,vy}` for transition \(k-1\to k\); never infer a command from truth.
- Truth may create only measurement-presence bits and noisy scalar ranges at the simulation sensor boundary. Runtime estimator calls receive no truth coordinates, true ranges, truth errors, or offline percentiles.
- Freeze:

```text
MAX_PREDICTION_AGE_FRAMES = 2
FRAME_DT_SECONDS = 0.5
MOTION_SIGMA_M_PER_FRAME = 0.5
INNOVATION_Q_MAX = 11.829007011943707
REACQUISITION_REDUCED_COST_MAX = 9.0
E_CAT_M = 50.0
MAX_PROPOSALS_PER_CANDIDATE = 50
INITIAL_DAMPING = 1e-3
MIN_DAMPING = 1e-15
MAX_DAMPING = 1e15
DAMPING_FACTOR = 10.0
SCALE_AWARE_STATIONARITY = 1e-6
RELATIVE_SPECTRAL_THRESHOLD = 1e-12
REPRESENTABLE_STEP_RELATIVE_THRESHOLD = 1e-12
CANDIDATE_DEDUP_M = 1e-9
RELATIVE_TIE_TOLERANCE = 1e-12
```

- Output statuses are exactly `fresh`, `predicted`, `unavailable`.
- Attempt statuses are exactly `accepted`, `rejected`, `failed`, `invalid`, `reference_unavailable`.
- A fresh output requires all mandatory and optional UAV references to be current-frame fresh and recursive provenance to contain at least two distinct base IDs.
- A predicted/unavailable UAV can never be an anchor in qualification-enabled variants.
- At prediction age three the public output has no estimate, covariance, epsilon, or provenance; a private re-acquisition seed remains unpublished and reference-ineligible.
- Re-acquisition requires at least three current-frame fresh references, non-collinear range FIM, two-base provenance, and whitened WNLS cost divided by `max(1, m - 2)` no greater than `9.0`.
- Process UAVs in ascending global ID inside every `(variant, seed, frame)` block.
- The `prediction_expiry` ablation alone may preserve the baseline stale-reference policy; every such use is recorded as a violation and that ablation is never the complete estimator.
- Stage 1 uses exact seeds `20260727` through `20260746` and the exact three cumulative variants `prediction_expiry`, `fresh_reference_qualification`, `predictive_multistart`.
- The preserved Stage 1 data violates the 25 m/s component assumption (`243/7000` rows); report that fact and keep the paper gate closed.
- Require at least `8_000_000_000` free bytes before a registered launch, stop below `6_000_000_000`, cap rebuildable cache/raw bundle at `2_000_000_000`, and cap compact output at `10_000_000` allocated bytes.
- Registered replay/analyzer output roots are exact bundle directories, created exclusively; pre-existing empty or nonempty targets fail closed.
- Every registered replay/analyzer attempt writes one terminal manifest on success or failure and is never retried under the same protocol.
- Commit implementation before registering the protocol; commit protocol review before creating the registered output directory.
- Update DRA only in `/private/tmp/dra-cbf2026-diagnostic` on isolated `main`; do not push and do not touch the dirty primary DRA checkout.
- Do not edit the paper in this plan.

---

### Task 1: Estimator lifecycle and nonabsorbing expiry

**Files:**
- Create: `scripts/diagnostics/predictive_wnls.py`
- Create: `tests/test_predictive_wnls.py`

**Interfaces:**

```python
OUTPUT_STATUSES = ("fresh", "predicted", "unavailable")
ATTEMPT_STATUSES = (
    "accepted",
    "rejected",
    "failed",
    "invalid",
    "reference_unavailable",
)

def make_unavailable_output(reason: str) -> dict:
    """Return a public unavailable output with no finite localization fields."""

def propagate_estimator_prior(
    previous_output: dict | None,
    previous_private_seed: dict | None,
    held_velocity: object,
    *,
    dt: float = FRAME_DT_SECONDS,
) -> dict:
    """Return public_prediction and private_reacquisition_seed for one transition."""

def output_is_fresh(output: dict | None) -> bool:
    """Validate a current-frame fresh public output."""

def reference_is_eligible(output: dict | None) -> bool:
    """Accept only finite fresh outputs with at least two base roots."""

def finalize_attempt(
    attempt: dict,
    prior_bundle: dict,
    *,
    frame_index: int,
) -> dict:
    """Publish fresh after acceptance, otherwise predicted/unavailable by age."""
```

- [ ] **Step 1: Write lifecycle RED tests**

Use literal test helpers in `tests/test_predictive_wnls.py`:

```python
import math
import unittest
from pathlib import Path
from unittest import mock

import numpy as np


def make_test_output(
    status="fresh",
    estimate=(10.0, 20.0),
    covariance=((1.0, 0.0), (0.0, 4.0)),
    prediction_age=0,
    provenance=(0, 1),
):
    return {
        "output_status": status,
        "estimate": list(estimate),
        "modeled_covariance": [list(row) for row in covariance],
        "epsilon": 6.0 if status == "fresh" else None,
        "prediction_age": prediction_age,
        "base_anchor_provenance": list(provenance),
    }


class PredictiveLifecycleTests(unittest.TestCase):
    def test_command_prediction_ages_covariance_and_public_status(self):
        bundle = propagate_estimator_prior(
            make_test_output(),
            None,
            [4.0, -2.0],
        )
        prediction = bundle["public_prediction"]
        self.assertEqual(prediction["output_status"], "predicted")
        self.assertEqual(prediction["prediction_age"], 1)
        np.testing.assert_allclose(prediction["estimate"], [12.0, 19.0])
        np.testing.assert_allclose(
            prediction["modeled_covariance"],
            [[1.25, 0.0], [0.0, 4.25]],
        )
        self.assertIsNone(prediction["epsilon"])

    def test_age_three_is_publicly_unavailable_but_keeps_private_seed(self):
        age_two = make_test_output(
            status="predicted",
            prediction_age=2,
            provenance=(),
        )
        bundle = propagate_estimator_prior(age_two, None, [2.0, 0.0])
        public = bundle["public_prediction"]
        private = bundle["private_reacquisition_seed"]
        self.assertEqual(public["output_status"], "unavailable")
        self.assertIsNone(public["estimate"])
        self.assertEqual(public["base_anchor_provenance"], [])
        np.testing.assert_allclose(private["estimate"], [11.0, 20.0])

    def test_unavailable_public_state_can_continue_private_seed_only(self):
        bundle = propagate_estimator_prior(
            make_unavailable_output("expired"),
            {
                "estimate": [5.0, 6.0],
                "modeled_covariance": [[2.0, 0.0], [0.0, 2.0]],
            },
            [2.0, 4.0],
        )
        self.assertEqual(
            bundle["public_prediction"]["output_status"],
            "unavailable",
        )
        np.testing.assert_allclose(
            bundle["private_reacquisition_seed"]["estimate"],
            [6.0, 8.0],
        )

    def test_reference_eligibility_requires_fresh_finite_two_base_roots(self):
        self.assertTrue(reference_is_eligible(make_test_output()))
        self.assertFalse(
            reference_is_eligible(make_test_output(provenance=(0, 0)))
        )
        self.assertFalse(
            reference_is_eligible(
                make_test_output(status="predicted", provenance=())
            )
        )

    def test_invalid_velocity_fails_closed(self):
        for velocity in ([math.nan, 0.0], [1.0], "bad"):
            with self.subTest(velocity=velocity):
                with self.assertRaises(ValueError):
                    propagate_estimator_prior(
                        make_test_output(),
                        None,
                        velocity,
                    )
```

- [ ] **Step 2: Verify RED**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_predictive_wnls.PredictiveLifecycleTests -v
```

Expected: import failure for the missing module.

- [ ] **Step 3: Implement minimal lifecycle**

Validate every vector as finite shape `(2,)` and every covariance as finite,
symmetric positive definite shape `(2,2)`.
Propagate with:

```python
next_estimate = previous_estimate + dt * held_velocity
next_covariance = previous_covariance + 0.25 * np.eye(2)
```

For age one/two, publish `predicted` with `epsilon=None` and
`aged_modeled_radius = 3 * sqrt(max(eigvalsh(next_covariance)))`.
At age three, publish `unavailable` and retain the same finite state only in
`private_reacquisition_seed`.
When the previous public output is already unavailable, propagate only the
private seed.

`finalize_attempt` uses this precedence:

```text
accepted candidate -> fresh age 0
nonaccepted + age-1/2 public prediction -> predicted
nonaccepted + expired/no prediction -> unavailable
```

- [ ] **Step 4: Verify GREEN and regression**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_predictive_wnls.PredictiveLifecycleTests \
  tests.test_replay_localization_calibration.InitializationPolicyTests -v
git diff --check
```

- [ ] **Step 5: Commit**

```bash
git add scripts/diagnostics/predictive_wnls.py \
  tests/test_predictive_wnls.py
git commit -m "feat(diagnostics): add predictive estimator lifecycle"
```

---

### Task 2: Sensor-boundary records, fresh DAG references, and candidates

**Files:**
- Modify: `scripts/diagnostics/predictive_wnls.py`
- Modify: `tests/test_predictive_wnls.py`

**Interfaces:**

```python
def merge_base_anchor_provenance(
    direct_base_ids: object,
    uav_outputs: object,
) -> tuple[int, ...]:
    """Return sorted unique current-frame recursive base roots."""

def qualify_active_references(
    *,
    mandatory: dict[str, list[int]],
    optional_keys: object,
    measurement_records: dict[tuple[str, int], dict],
    uav_outputs: dict[int, dict],
    variant: str,
) -> dict:
    """Return active measurement/reference records or reference_unavailable."""

def algebraic_multilateration_candidate(
    reference_positions: object,
    measured_ranges: object,
) -> np.ndarray | None:
    """Return one deterministic finite least-squares point."""

def best_conditioned_pair(
    observer_center: object | None,
    reference_positions: object,
    measured_ranges: object,
    reference_keys: object,
) -> tuple[int, int] | None:
    """Use predicted directions or re-acquisition cosine-law geometry."""

def two_circle_candidates(
    first_position: object,
    first_range: float,
    second_position: object,
    second_range: float,
) -> tuple[np.ndarray, ...]:
    """Return negative-oriented then positive-oriented branches."""

def initial_candidates(
    *,
    live_prediction: dict | None,
    private_seed: dict | None,
    reference_positions: object,
    measured_ranges: object,
    reference_keys: object,
) -> tuple[dict, ...]:
    """Return at most four deterministic deduplicated candidates."""
```

Candidate source labels are exactly:

```text
prediction
private_reacquisition_seed
algebraic
circle_negative
circle_positive
```

- [ ] **Step 1: Write reference/candidate RED tests**

Add test classes with literal expected values:

```python
class ReferenceQualificationTests(unittest.TestCase):
    def test_missing_mandatory_measurement_is_reference_unavailable(self):
        result = qualify_active_references(
            mandatory={"base_ids": [0], "uav_ids": [1]},
            optional_keys=[("uav", 2)],
            measurement_records={
                ("base", 0): {"present": True, "noisy_range": 10.0},
                ("uav", 2): {"present": True, "noisy_range": 12.0},
            },
            uav_outputs={
                1: make_test_output(),
                2: make_test_output(),
            },
            variant="predictive_multistart",
        )
        self.assertEqual(result["status"], "reference_unavailable")
        self.assertEqual(result["missing_mandatory"], [("uav", 1)])

    def test_qualification_filters_predicted_optional_without_truth(self):
        result = qualify_active_references(
            mandatory={"base_ids": [0], "uav_ids": [1]},
            optional_keys=[("uav", 2), ("uav", 3)],
            measurement_records={
                ("base", 0): {"present": True, "noisy_range": 10.0},
                ("uav", 1): {"present": True, "noisy_range": 11.0},
                ("uav", 2): {"present": True, "noisy_range": 12.0},
                ("uav", 3): {"present": True, "noisy_range": 13.0},
            },
            uav_outputs={
                1: make_test_output(),
                2: make_test_output(
                    status="predicted",
                    provenance=(),
                ),
                3: make_test_output(provenance=(0, 2)),
            },
            variant="predictive_multistart",
        )
        self.assertEqual(
            result["active_keys"],
            [("base", 0), ("uav", 1), ("uav", 3)],
        )
        self.assertEqual(
            result["excluded"],
            [{"key": ("uav", 2), "reason": "not_current_frame_fresh"}],
        )

    def test_prediction_expiry_ablation_records_stale_anchor_violation(self):
        result = qualify_active_references(
            mandatory={"base_ids": [0], "uav_ids": [1]},
            optional_keys=[("uav", 2)],
            measurement_records={
                ("base", 0): {"present": True, "noisy_range": 10.0},
                ("uav", 1): {"present": True, "noisy_range": 11.0},
                ("uav", 2): {"present": True, "noisy_range": 12.0},
            },
            uav_outputs={
                1: make_test_output(),
                2: make_test_output(
                    status="predicted",
                    provenance=(),
                ),
            },
            variant="prediction_expiry",
        )
        self.assertIn(("uav", 2), result["active_keys"])
        self.assertEqual(
            result["violations"],
            [
                {
                    "key": ("uav", 2),
                    "reason": "stale_or_predicted_anchor_used",
                }
            ],
        )


class CandidateGeometryTests(unittest.TestCase):
    def test_two_circle_branches_have_fixed_orientation_order(self):
        candidates = two_circle_candidates([0, 0], 5, [6, 0], 5)
        np.testing.assert_allclose(candidates[0], [3, -4])
        np.testing.assert_allclose(candidates[1], [3, 4])

    def test_reacquisition_pair_uses_cosine_law_without_observer_center(self):
        pair = best_conditioned_pair(
            None,
            [[0.0, 0.0], [6.0, 0.0], [0.0, 8.0]],
            [5.0, 5.0, 5.0],
            [("base", 0), ("base", 1), ("base", 2)],
        )
        self.assertEqual(pair, (0, 1))

    def test_initial_candidates_are_unique_ordered_and_capped_at_four(self):
        candidates = initial_candidates(
            live_prediction={
                "estimate": [3.0, 3.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
            },
            private_seed=None,
            reference_positions=[[0.0, 0.0], [6.0, 0.0], [0.0, 8.0]],
            measured_ranges=[5.0, 5.0, 6.0],
            reference_keys=[("base", 0), ("base", 1), ("base", 2)],
        )
        self.assertEqual(len(candidates), 4)
        self.assertEqual(
            [candidate["source"] for candidate in candidates],
            [
                "prediction",
                "algebraic",
                "circle_negative",
                "circle_positive",
            ],
        )

    def test_prediction_wins_deduplication_with_exact_geometry(self):
        candidates = initial_candidates(
            live_prediction={
                "estimate": [3.0, 4.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
            },
            private_seed=None,
            reference_positions=[[0.0, 0.0], [6.0, 0.0], [0.0, 8.0]],
            measured_ranges=[5.0, 5.0, 5.0],
            reference_keys=[("base", 0), ("base", 1), ("base", 2)],
        )
        self.assertEqual(
            [candidate["source"] for candidate in candidates],
            ["prediction", "circle_negative"],
        )
```

- [ ] **Step 2: Verify RED**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_predictive_wnls.ReferenceQualificationTests \
  tests.test_predictive_wnls.CandidateGeometryTests -v
```

- [ ] **Step 3: Implement reference qualification**

Base references use configured positions and are fresh when a measurement is
present.
UAV references use only already-completed current-frame outputs.
For qualification-enabled variants, mandatory UAVs must satisfy
`reference_is_eligible`; optional predicted/unavailable UAVs are excluded.
Return all exclusions and violations.
Do not accept observer position, truth position, true range, or offline error
parameters.

For the `prediction_expiry` ablation only, preserve the supplied baseline
active keys even when an output is not fresh, and append
`stale_or_predicted_anchor_used` to violations.

- [ ] **Step 4: Implement deterministic geometry**

For live prediction, maximize absolute cross product of predicted unit
directions.
For re-acquisition, compute implied cosine:

```python
cosine = (range_a**2 + range_b**2 - baseline**2) / (
    2.0 * range_a * range_b
)
score = math.sqrt(max(0.0, 1.0 - cosine**2))
```

Skip infeasible `abs(cosine) > 1 + 1e-12`, zero range, and coincident
reference pairs.
Use `(0, id)` for base and `(1, id)` for UAV lexical ordering.
Use relative `1e-12` score ties and `1e-9 m` candidate deduplication.

- [ ] **Step 5: Verify GREEN and commit**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_predictive_wnls -v
git diff --check
git add scripts/diagnostics/predictive_wnls.py \
  tests/test_predictive_wnls.py
git commit -m "feat(diagnostics): qualify fresh DAG references"
```

---

### Task 3: Finite-budget multi-start WNLS and online gates

**Files:**
- Modify: `scripts/diagnostics/predictive_wnls.py`
- Modify: `tests/test_predictive_wnls.py`

**Interfaces:**

```python
def solve_finite_budget_wnls(
    reference_positions: object,
    reference_covariances: object,
    measurements: object,
    initial_estimate: object,
    ranging_sigma: float,
) -> dict:
    """Run the unchanged residual/Jacobian with exact finite LM rules."""

def scale_aware_stationary(
    gradient: object,
    residual: object,
) -> bool:
    """Apply the frozen infinity-norm stationarity rule."""

def normalized_innovation(
    candidate_estimate: object,
    candidate_covariance: object,
    live_prediction: dict,
) -> dict:
    """Return q_innov and covariance validity diagnostics."""

def candidate_acceptance(
    candidate_result: dict,
    *,
    live_prediction: dict | None,
    active_reference_count: int,
    base_anchor_provenance: object,
) -> tuple[bool, str, dict]:
    """Apply numerical/FIM/provenance/innovation or re-acquisition gates."""

def solve_predictive_multistart(
    *,
    reference_positions: object,
    reference_covariances: object,
    measurements: object,
    reference_keys: object,
    live_prediction: dict | None,
    private_seed: dict | None,
    ranging_sigma: float,
    base_anchor_provenance: object,
) -> dict:
    """Retain every candidate/trace and return one deterministic attempt."""

def select_candidate_result(
    candidate_records: object,
    *,
    has_live_prediction: bool,
) -> dict | None:
    """Select accepted candidates by cost, q innovation, then source order."""
```

- [ ] **Step 1: Write solver RED tests**

```python
class FiniteBudgetWnlsTests(unittest.TestCase):
    def test_scale_aware_stationarity_accepts_expected_solution(self):
        self.assertTrue(
            scale_aware_stationary(
                [1.5e-6, 0.0],
                [1.0],
            )
        )
        self.assertFalse(
            scale_aware_stationary(
                [2.1e-6, 0.0],
                [1.0],
            )
        )

    def test_rejected_proposals_increase_damping_without_overflow(self):
        with mock.patch(
            "scripts.diagnostics.predictive_wnls._linearized_terms",
            return_value=(
                np.eye(2),
                np.ones(2),
                np.ones(2),
                np.eye(2),
                np.array([1e6, 0.0]),
                2.0,
            ),
        ):
            result = solve_finite_budget_wnls(
                [[0.0, 0.0], [2.0, 0.0]],
                [np.eye(2), np.eye(2)],
                [1.0, 1.0],
                [1.0, 1.0],
                0.5,
            )
        damping = [row["damping"] for row in result["proposal_trace"]]
        self.assertEqual(damping[:2], [1e-3, 1e-2])
        self.assertLessEqual(damping[-1], 1e15)
        self.assertEqual(result["failure_reason"], "maximum_damping_exceeded")

    def test_tiny_nonstationary_step_fails_with_explicit_reason(self):
        tiny_terms = (
            np.eye(2),
            np.ones(2),
            np.ones(2),
            1e20 * np.eye(2),
            np.array([1e-3, 0.0]),
            2.0,
        )
        with mock.patch(
            "scripts.diagnostics.predictive_wnls._linearized_terms",
            return_value=tiny_terms,
        ):
            result = solve_finite_budget_wnls(
                [[0.0, 0.0], [2.0, 0.0]],
                [np.eye(2), np.eye(2)],
                [1.0, 1.0],
                [1.0, 1.0],
                0.5,
            )
        self.assertEqual(
            result["failure_reason"],
            "no_representable_improving_step",
        )

    def test_trace_records_every_proposal(self):
        result = solve_finite_budget_wnls(
            [[0.0, 0.0], [6.0, 0.0], [0.0, 8.0]],
            [np.eye(2), np.eye(2), np.eye(2)],
            [5.0, 5.0, 5.0],
            [3.1, 4.1],
            0.5,
        )
        self.assertGreaterEqual(len(result["proposal_trace"]), 1)
        self.assertEqual(
            set(result["proposal_trace"][0]),
            {
                "proposal",
                "damping",
                "cost",
                "stationarity_norm",
                "raw_step_norm",
                "trial_cost",
                "invalid_trial_reason",
                "accepted",
            },
        )


class CandidateAcceptanceTests(unittest.TestCase):
    def test_mahalanobis_gate_uses_sum_of_prediction_and_range_covariance(self):
        result = normalized_innovation(
            [4.0, 0.0],
            [[1.0, 0.0], [0.0, 1.0]],
            {
                "estimate": [0.0, 0.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
            },
        )
        self.assertAlmostEqual(result["q_innov"], 8.0)
        self.assertTrue(result["valid"])

    def test_candidate_above_11_829007011943707_is_rejected(self):
        candidate = {
            "status": "converged",
            "estimate": [5.0, 0.0],
            "covariance": [[1.0, 0.0], [0.0, 1.0]],
            "cost": 0.0,
            "fim_valid": True,
        }
        accepted, reason, diagnostics = candidate_acceptance(
            candidate,
            live_prediction={
                "estimate": [0.0, 0.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
            },
            active_reference_count=3,
            base_anchor_provenance=[0, 1],
        )
        self.assertFalse(accepted)
        self.assertEqual(reason, "innovation_q_exceeds_reference_quantile")
        self.assertAlmostEqual(diagnostics["q_innov"], 12.5)

    def test_reacquisition_requires_three_refs_and_reduced_cost_at_most_nine(self):
        candidate = {
            "status": "converged",
            "estimate": [1.0, 1.0],
            "covariance": [[1.0, 0.0], [0.0, 1.0]],
            "cost": 9.0,
            "fim_valid": True,
        }
        accepted, _, _ = candidate_acceptance(
            candidate,
            live_prediction=None,
            active_reference_count=3,
            base_anchor_provenance=[0, 1],
        )
        self.assertTrue(accepted)
        too_large = dict(candidate, cost=math.nextafter(9.0, math.inf))
        self.assertFalse(
            candidate_acceptance(
                too_large,
                live_prediction=None,
                active_reference_count=3,
                base_anchor_provenance=[0, 1],
            )[0]
        )
        self.assertFalse(
            candidate_acceptance(
                candidate,
                live_prediction=None,
                active_reference_count=2,
                base_anchor_provenance=[0, 1],
            )[0]
        )

    def test_multistart_ties_use_q_then_fixed_source_order(self):
        selected = select_candidate_result(
            [
                {
                    "source": "algebraic",
                    "accepted": True,
                    "cost": 1.0,
                    "q_innov": 2.0,
                },
                {
                    "source": "circle_negative",
                    "accepted": True,
                    "cost": 1.0 + 5e-13,
                    "q_innov": 1.0,
                },
            ],
            has_live_prediction=True,
        )
        self.assertEqual(selected["source"], "circle_negative")
```

- [ ] **Step 2: Verify RED**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_predictive_wnls.FiniteBudgetWnlsTests \
  tests.test_predictive_wnls.CandidateAcceptanceTests -v
```

- [ ] **Step 3: Implement the exact solver**

Import, without modifying:

```python
from scripts.diagnostics.replay_localization_calibration import (
    _linearized_terms,
    _validated_inputs,
    fim_radius,
)
```

For each candidate, reset damping to `1e-3`.
At most 50 proposals are allowed.
Use infinity-norm stationarity:

```python
stationary = np.linalg.norm(gradient, ord=np.inf) <= (
    1e-6 * (1.0 + np.linalg.norm(residual))
)
```

Accept a finite trial only when `trial_cost < current_cost`.
On acceptance divide damping by ten but not below `1e-15`.
On rejection multiply by ten; fail before a value above `1e15`.
Do not clip a step using velocity.
Record proposal index, damping, current cost, stationarity, raw step norm,
trial cost or invalid reason, and acceptance.
After numerical convergence call the unchanged `fim_radius`.

- [ ] **Step 4: Implement online gates and selection**

Symmetrize `S = P_prediction + P_range`, require the same relative spectral
positive-definiteness rule, and compute `q_innov` with `np.linalg.solve`.
For live prediction accept only `q_innov <= 11.829007011943707`.
For re-acquisition accept only `m >= 3` and
`cost / max(1, m - 2) <= 9.0`.
Always require two-base provenance and finite positive-definite range FIM.

Attempt aggregation precedence is:

```text
selected accepted candidate -> accepted
completed but gate-rejected candidate exists -> rejected
well-formed candidate exhausted budget -> failed
otherwise -> invalid
```

Retain every candidate initial point, source, complete proposal trace, final
result, gate diagnostics, acceptance boolean, and rejection reason.

- [ ] **Step 5: Verify GREEN, legacy math, and commit**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_predictive_wnls \
  tests.test_replay_localization_calibration.WnlsAndFimTests -v
git diff --check
git add scripts/diagnostics/predictive_wnls.py \
  tests/test_predictive_wnls.py
git commit -m "feat(diagnostics): add predictive multi-start WNLS"
```

---

### Task 4: Immutable Stage 0 fixtures and mechanism report

**Files:**
- Create: `scripts/diagnostics/extract_predictive_wnls_stage0.py`
- Create: `tests/test_extract_predictive_wnls_stage0.py`
- Create: `tests/fixtures/cbf2026_predictive_wnls/frame44_recovery.json`
- Create: `tests/fixtures/cbf2026_predictive_wnls/frame177_cascade.json`
- Create: `tests/fixtures/cbf2026_predictive_wnls/reacquisition.json`
- Create: `tests/fixtures/cbf2026_predictive_wnls/manifest.json`
- Modify: `tests/test_predictive_wnls.py`
- Create: `docs/diagnostics/2026-07-30-predictive-wnls-stage0.md`

**Interfaces:**

```python
STAGE0_SCHEMA_ID = "cbf2026-predictive-wnls-stage0-fixtures-v2"

def extract_stage0_fixtures(
    *,
    truth_data_path: Path,
    input_manifest_path: Path,
    baseline_process_path: Path,
    output_dir: Path,
) -> dict:
    """Write deterministic prefix-replay runtime/offline fixture subtrees."""

def run_stage0_fixture(path: Path) -> dict:
    """Run online estimator first, then attach offline fixture evaluation."""
```

Frozen inputs:

```text
truth data:
/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json
SHA-256:
3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527

input manifest:
/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json
SHA-256:
6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb

baseline process:
/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/restart/localization-calibration/20260729T112437.413270Z_efb8e7137776415c833f11204cf519d5/calibration.jsonl.gz
SHA-256:
c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003
```

- [ ] **Step 1: Write extractor RED tests**

Use tiny temporary truth/baseline fixtures and assert:

- hashes are checked before and after reads;
- frame-44 extraction selects seed `20260736` and includes frames 0--44 for
  all 14 UAVs, so every current reference/output is recomputed by the new
  estimator before evaluating frame-44 UAV 14;
- frame-177 extraction selects seed `20260730` and includes frames 0--177 for
  all 14 UAVs, so the old upstream cascade is tested without injecting
  baseline stale reference states;
- every transition uses the preceding frame's applied `opt.result`, and every
  runtime frame stores only measurement-presence/noisy-range sensor records;
- runtime and offline subtrees are disjoint;
- deterministic JSON serialization produces identical hashes; and
- an existing output directory fails closed.

- [ ] **Step 2: Verify RED**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_extract_predictive_wnls_stage0 -v
```

- [ ] **Step 3: Implement extractor and generate fixtures once**

Reuse immutable parsing helpers from the legacy replay.
The extractor may read truth while producing the offline subtree and sensor
records, but serialized runtime inputs contain no truth coordinates or true
ranges.
The historical fixture runtime subtree is a complete deterministic prefix,
not a one-row transplant of baseline estimates.
The offline subtree stores truth only for post-runtime assertions.
The baseline process contributes target-row comparison records but never
supplies runtime reference estimates or covariances.
The synthetic `reacquisition.json` contains:

```text
frame A: age-two public prediction
frame B: two fresh ranges -> public unavailable
frame C: three non-collinear fresh ranges with hand-derived noisy values
         -> accepted fresh re-acquisition
```

Generate directly into the absent committed fixture directory:

```bash
conda run -n cbf_env python \
  scripts/diagnostics/extract_predictive_wnls_stage0.py \
  --truth-data-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  --input-manifest-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json \
  --baseline-process-path /private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/restart/localization-calibration/20260729T112437.413270Z_efb8e7137776415c833f11204cf519d5/calibration.jsonl.gz \
  --output-dir tests/fixtures/cbf2026_predictive_wnls
```

Do not rerun this command against an existing fixture directory.

- [ ] **Step 4: Write Stage 0 RED tests**

```python
FIXTURE_ROOT = Path("tests/fixtures/cbf2026_predictive_wnls")
FRAME44_PATH = FIXTURE_ROOT / "frame44_recovery.json"
FRAME177_PATH = FIXTURE_ROOT / "frame177_cascade.json"
REACQUISITION_PATH = FIXTURE_ROOT / "reacquisition.json"


class HistoricalMechanismFixtureTests(unittest.TestCase):
    def test_frame44_is_fresh_accepted_and_within_five_metres(self):
        result = run_stage0_fixture(FRAME44_PATH)
        self.assertEqual(result["attempt_status"], "accepted")
        self.assertEqual(result["output_status"], "fresh")
        self.assertIsNotNone(result["selected_candidate_source"])
        self.assertLessEqual(result["offline_error_norm"], 5.0)

    def test_frame177_stale_u8_cannot_create_fresh_catastrophic_cascade(self):
        result = run_stage0_fixture(FRAME177_PATH)
        self.assertFalse(
            any(
                ref["kind"] == "uav"
                and ref["used"]
                and ref["output_status"] != "fresh"
                for row in result["rows"]
                for ref in row["reference_freshness"]
            )
        )
        self.assertFalse(
            any(
                row["output_status"] == "fresh"
                and row["offline_error_norm"] > 50.0
                for row in result["rows"]
            )
        )

    def test_expiry_reacquires_only_on_three_consistent_ranges(self):
        result = run_stage0_fixture(REACQUISITION_PATH)
        self.assertEqual(
            [row["output_status"] for row in result["rows"]],
            ["unavailable", "fresh"],
        )
```

Patch runtime call boundaries in the fixture adapter and assert that no call
keyword or dictionary contains `truth`, `true_range`, or `offline_error`.

- [ ] **Step 5: Verify Stage 0, write report, and commit**

The report records every fixture hash, baseline/new status and error, every
candidate rejection, whether improvement is fresh estimation or availability,
and no generalization beyond the frozen fixtures.

```bash
conda run -n cbf_env python -m unittest \
  tests.test_extract_predictive_wnls_stage0 \
  tests.test_predictive_wnls.HistoricalMechanismFixtureTests -v
git diff --check
git add scripts/diagnostics/extract_predictive_wnls_stage0.py \
  tests/test_extract_predictive_wnls_stage0.py \
  tests/test_predictive_wnls.py \
  tests/fixtures/cbf2026_predictive_wnls \
  docs/diagnostics/2026-07-30-predictive-wnls-stage0.md
git commit -m "test(diagnostics): freeze predictive WNLS mechanisms"
```

---

### Task 5: Paired Stage 1 replay with exact-output semantics

**Files:**
- Create: `scripts/diagnostics/replay_predictive_wnls_recovery.py`
- Create: `tests/test_replay_predictive_wnls_recovery.py`

**Interfaces:**

```python
DEVELOPMENT_VARIANTS = (
    "prediction_expiry",
    "fresh_reference_qualification",
    "predictive_multistart",
)
RAW_SCHEMA_ID = "cbf2026-predictive-wnls-development-rows-v2"
RAW_PROCESS_NAME = "predictive-wnls-development.jsonl.gz"
TERMINAL_MANIFEST_NAME = "manifest.json"
RAW_BUNDLE_CAP_BYTES = 2_000_000_000

def replay_predictive_recovery(
    *,
    data_path: Path,
    input_manifest_path: Path,
    protocol_path: Path,
    output_root: Path,
    run_seeds: tuple[int, ...],
    max_frames: int | None = None,
) -> dict:
    """Create one exact, disk-guarded raw Stage 1 bundle."""
```

- [ ] **Step 1: Write replay RED tests**

Hermetic two-frame/two-UAV tests assert:

- exact row count `3 variants * seeds * frames * UAVs`;
- execution and serialization order is variant, seed, frame, ascending UAV ID;
- frame \(k\) uses frame-\((k-1)\) `opt.result`, and a deliberately different
  nominal command is ignored;
- sensor boundary may use truth to emit presence/noisy range, while patched
  estimator callbacks receive no truth fields;
- qualification-enabled variants never use predicted/unavailable anchors;
- the diagnostic ablation records every stale/predicted anchor violation;
- output status, attempt status, prediction age, private-seed presence, and
  reference provenance are separate;
- every candidate and proposal trace is serialized as compact arrays with
  manifest-declared field order;
- `offline_fresh_containment`, `offline_aged_radius_containment`, fresh/aged
  `q_error`, and inapplicable nulls are correct;
- each source/input is rehashed after reading;
- protocol-bound replay/estimator source hashes and frozen input hashes are
  verified before allocation and again before completed publication;
- measurement noise uses `stable_measurement_seed`;
- exact target creation rejects pre-existing empty/nonempty directories;
- output/input/project path overlap and symlink targets fail closed;
- success and injected failure both create one terminal manifest;
- deterministic gzip uses `filename=""` and `mtime=0`, so compressed and
  decompressed hashes match across identical smoke runs;
- a final source/input mutation probe cannot leave a completed manifest;
- ordinary exceptions, `KeyboardInterrupt`, and `SystemExit` after allocation
  write a failed terminal manifest before being re-raised;
- start/floor/raw-cap violations fail closed; and
- no baseline process rows are copied into the bundle.

- [ ] **Step 2: Verify RED**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_replay_predictive_wnls_recovery -v
```

- [ ] **Step 3: Implement the sensor/replay loop**

Reuse read-only helpers:

```python
from scripts.diagnostics.replay_localization_calibration import (
    _frames,
    _initial_positions,
    _strict_load,
    _truth_positions,
    active_references,
    fixed_references,
    stable_measurement_seed,
)
```

Only a local `sensor_records(...)` wrapper may call legacy
`active_references` with truth; it returns reference key, present bit, and
noisy range without truth coordinates or true range.
Runtime processing consumes only those records, configuration, commands,
prior estimator outputs, and fresh lower-index current-frame outputs.

Rows contain:

```text
variant, seed, frame_index, robot_id, squad_local_index
applied_command_source_frame, applied_command
attempt_status, attempt_failure_reason
output_status, prediction_age
estimate, fresh_modeled_covariance, fresh_epsilon
aged_modeled_covariance, aged_modeled_radius
private_reacquisition_seed
base_anchor_provenance
mandatory_references, optional_candidates, active_references
reference_freshness, excluded_references, reference_violations
candidates, selected_candidate_source
offline_truth_position, offline_error_norm
offline_fresh_containment, offline_aged_radius_containment
offline_fresh_q_error, offline_aged_q_error
```

Write JSON lines through:

```python
gzip.GzipFile(filename="", fileobj=raw, mode="wb", mtime=0)
```

Check allocated bundle size while writing and free space at deterministic
frame boundaries.

- [ ] **Step 4: Implement exclusive terminal-manifest behavior**

Reuse `available_bytes`, `allocated_bytes`, `require_start_space`,
`_nearest_existing_ancestor`, `_sha256`, and `_validate_output_root` from
`scripts.diagnostics.run_diagnostic`.
Add local path-overlap validation protecting the project, truth data and its
bundle, input manifest and its bundle, and immutable baseline.
Reject symlink/non-regular trust roots.
Safely create a missing registered parent directory, then create the exact
`output_root` with exclusive `Path.mkdir(exist_ok=False)`.
Never call a helper that creates a nested timestamp/run directory.
In particular, do not call `_allocate_run_root`.

On any `BaseException` after exclusive creation, write a terminal manifest with
`status=failed`, exception class/message, rows written, file identities, and
allocated bytes, then re-raise without swallowing interrupts.
Serialize strict JSON with `allow_nan=False` and sorted keys.
On success close/hash the process, rehash inputs and bound source files,
enforce caps, first stage a `finalizing` manifest, perform one final disk and
identity probe, and only then atomically no-replace publish `manifest.json`
with `status=completed`.

- [ ] **Step 5: Verify GREEN, full baseline, and commit**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_replay_predictive_wnls_recovery -v
conda run -n cbf_env python -m unittest discover \
  -s tests -p 'test_*.py' -q
git diff --check
git add scripts/diagnostics/replay_predictive_wnls_recovery.py \
  tests/test_replay_predictive_wnls_recovery.py
git commit -m "feat(diagnostics): add predictive WNLS replay"
```

---

### Task 6: Compact analyzer and protocol registrar

**Files:**
- Create: `scripts/diagnostics/analyze_predictive_wnls_recovery.py`
- Create: `tests/test_analyze_predictive_wnls_recovery.py`
- Create: `scripts/diagnostics/register_predictive_wnls_stage1.py`
- Create: `tests/test_register_predictive_wnls_stage1.py`

**Interfaces:**

```python
ANALYSIS_SCHEMA_ID = "cbf2026-predictive-wnls-development-analysis-v2"
OUTPUT_JSON_NAME = "predictive-wnls-development.json"
OUTPUT_MARKDOWN_NAME = "predictive-wnls-development.md"
ANALYZER_MANIFEST_NAME = "manifest.json"
COMPACT_OUTPUT_CAP_BYTES = 10_000_000

def analyze_predictive_recovery(
    *,
    baseline_process_path: Path,
    development_manifest_path: Path,
    protocol_path: Path,
    expected_baseline_sha256: str,
    output_root: Path,
) -> dict:
    """Aggregate exact-key paired tails, status transitions, and calibration."""

def register_stage1_protocol(
    *,
    repository_root: Path,
    output_markdown: Path,
    output_json: Path,
) -> dict:
    """Bind parent commit, exact file/input hashes, commands, seeds, and gates."""
```

- [ ] **Step 1: Write analyzer RED tests**

Tiny paired fixtures assert:

- only baseline `dynamic_dag_wnls` rows enter mapped denominators;
- baseline `converged` maps fresh, finite `stale` maps legacy published but
  not fresh, and nonfinite failed/invalid maps unavailable;
- duplicate, missing, extra, or out-of-order exact keys fail closed;
- fresh/predicted/unavailable and all attempt counts reconcile;
- fresh and all-published p50/p95/p99/max have explicit denominators;
- the baseline-published transition table counts new unavailable attrition
  and reports baseline errors on those keys;
- the baseline-fresh transition table counts fresh→fresh/predicted/unavailable,
  and fresh-error change uses only both-fresh exact keys;
- fresh→predicted is a downgrade, not fresh improvement or unavailable
  attrition;
- rejected candidate offline errors are retained;
- `E_CAT_M=50` catastrophic counts use full denominators;
- fresh coefficient-3 containment, fresh q-error exceedance above
  `5.991464547107979` and `9.0`, aged containment/q, and online q-innovation
  use separate denominators;
- depth, squad, time, seed, input-limit, frame44, and frame177 summaries exist;
- input-limit audit loads the protocol-bound truth trajectory and audits every
  optimizer output exactly once by `(frame_index, robot_id)`, therefore
  reporting `243/7000`, not one copy per range-noise seed or variant;
- raw-row `applied_command` is used only to validate predecessor-command
  prediction timing because frame zero has no predecessor and the raw rows
  therefore cover only `6986` of the `7000` physical optimizer outputs;
- prediction age above two, invalid provenance, nonascending DAG use, or
  qualification-enabled predicted anchor use is a protocol failure;
- the exact old `999.3318962079554` and `168.90169712504604` rows are resolved
  by keys, not approximate maximum matching;
- compact output cap and exclusive output-root behavior fail closed; and
- analyzer success/failure writes one terminal manifest.

- [ ] **Step 2: Write registrar RED tests**

Create a temporary Git repository and literal source/input files.
Assert the registrar:

- records current `HEAD` as implementation parent;
- records SHA-256 for replay, analyzer, estimator, truth, input manifest, and
  baseline process;
- records exact 20 seeds, three variants, constants, gates, paths, disk caps,
  no-retry rule, and paper gate `CLOSED`;
- emits exact replay/analyzer commands;
- refuses dirty tracked implementation files, missing inputs, and pre-existing
  protocol targets; and
- emits deterministic JSON/Markdown apart from the bound commit/hash values.

- [ ] **Step 3: Verify RED**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_predictive_wnls_recovery \
  tests.test_register_predictive_wnls_stage1 -v
```

- [ ] **Step 4: Implement aggregation**

Use exact `(seed, frame_index, robot_id)` keys inside each variant.
Publish for baseline and each variant:

```text
attempt/output budgets
fresh and all-published error distributions
both-published and both-fresh paired error changes
baseline-published attrition table and excluded baseline errors
baseline-fresh status-transition table and downgraded baseline errors
catastrophic counts
fresh and fresh-or-predicted availability
prediction-age and unavailable-streak maxima
reference/provenance/DAG violations
depth/squad/time/seed summaries
fresh FIM/epsilon/q calibration
aged-radius/q calibration
accepted/rejected q-innovation distributions
input-limit violations and applied-command maxima on 7000 unique
physical `(frame, robot)` rows
frame44/frame177 exact mechanism records
every predeclared gate with numerator and denominator
```

Use unrounded values in JSON and denominator-labelled rounded values in
Markdown.
Do not run row-level hypothesis tests as if frames/noise seeds were
independent trajectories.
The analyzer independently implements the same protected-path validation,
exclusive exact-root allocation, strict failed terminal manifest, finalizing
probe, and atomic no-replace completed manifest as the replay.
Its successful exact root contains JSON, Markdown, and `manifest.json`;
ordinary failure or interrupt after allocation contains only partial
unpublished files plus a failed terminal manifest.
It never uses the legacy timestamped `_allocate_run_root` or a success-only
transaction that would erase failure evidence.

- [ ] **Step 5: Implement registrar and verify GREEN**

The registrar writes:

```text
docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.md
docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.json
```

but Task 6 tests run it only in temporary directories.
The real protocol is generated after the Task 6 implementation commit.

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_predictive_wnls_recovery \
  tests.test_register_predictive_wnls_stage1 -v
conda run -n cbf_env python -m unittest discover \
  -s tests -p 'test_*.py' -q
git diff --check
git add scripts/diagnostics/analyze_predictive_wnls_recovery.py \
  scripts/diagnostics/register_predictive_wnls_stage1.py \
  tests/test_analyze_predictive_wnls_recovery.py \
  tests/test_register_predictive_wnls_stage1.py
git commit -m "feat(diagnostics): add predictive WNLS audit"
```

---

### Task 7: Freeze the Stage 1 protocol in a separate commit

**Files:**
- Create: `docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.md`
- Create: `docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.json`

**Interfaces:**
- Consumes the clean Task 6 implementation `HEAD`.
- Produces a protocol that binds the implementation parent without referring
  to its own future containing commit.

- [ ] **Step 1: Verify implementation and frozen identities**

```bash
conda run -n cbf_env python -m unittest discover \
  -s tests -p 'test_*.py' -q
shasum -a 256 scripts/diagnostics/replay_localization_calibration.py
shasum -a 256 \
  /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json \
  /private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/restart/localization-calibration/20260729T112437.413270Z_efb8e7137776415c833f11204cf519d5/calibration.jsonl.gz
```

Require the exact hashes from Global Constraints/Task 4.

- [ ] **Step 2: Generate the real protocol once**

```bash
conda run -n cbf_env python \
  scripts/diagnostics/register_predictive_wnls_stage1.py \
  --repository-root /private/tmp/cbf2026-diagnostic \
  --output-markdown docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.md \
  --output-json docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.json
```

The command fails if either target exists.

- [ ] **Step 3: Validate generated protocol**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_register_predictive_wnls_stage1 -v
git diff --check
git status --short
```

Only the two protocol files plus preserved `?? build-diagnostic/` may be new.

- [ ] **Step 4: Commit protocol separately**

```bash
git add docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.md \
  docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.json
git commit -m "docs(diagnostics): register predictive WNLS stage 1"
```

---

### Task 8: Preflight commit and exactly-once Stage 1 evidence

**Files:**
- Create and commit before run: `docs/diagnostics/reviews/2026-07-30-predictive-wnls-stage1-protocol-review.md`
- Create after run: `docs/diagnostics/2026-07-30-predictive-wnls-stage1.md`
- Create after run: `docs/diagnostics/reviews/2026-07-30-predictive-wnls-stage1-evidence-review.md`

**Interfaces:**
- Consumes the committed protocol and exact implementation it binds.
- Produces one raw bundle, one compact bundle, and independent raw audit.

- [ ] **Step 1: Obtain and commit independent preflight approval**

The reviewer checks runtime truth separation, sensor boundary, command timing,
fresh DAG/provenance, re-acquisition, numerical constants, all-row retention,
status/cohort denominators, source/input identities, exclusive targets,
terminal manifests, disk caps, and no outcome-tunable values.
Resolve all Critical/Important findings.

```bash
git add docs/diagnostics/reviews/2026-07-30-predictive-wnls-stage1-protocol-review.md
git commit -m "docs(diagnostics): approve predictive WNLS preflight"
```

Do not create the registered output directories before this commit succeeds.

- [ ] **Step 2: Run deterministic two-frame smoke twice**

Use distinct absent targets:

```bash
test ! -e /private/tmp/cbf2026-predictive-wnls-smoke-a
test ! -e /private/tmp/cbf2026-predictive-wnls-smoke-b

conda run -n cbf_env python \
  scripts/diagnostics/replay_predictive_wnls_recovery.py \
  --data-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  --input-manifest-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json \
  --protocol-json docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.json \
  --output-root /private/tmp/cbf2026-predictive-wnls-smoke-a \
  --run-seeds 20260727 \
  --max-frames 2

conda run -n cbf_env python \
  scripts/diagnostics/replay_predictive_wnls_recovery.py \
  --data-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  --input-manifest-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json \
  --protocol-json docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.json \
  --output-root /private/tmp/cbf2026-predictive-wnls-smoke-b \
  --run-seeds 20260727 \
  --max-frames 2
```

Compare terminal-manifest process hashes and exact row counts.
Timestamps/absolute output paths are manifest metadata and do not enter the
deterministic process stream.

- [ ] **Step 3: Verify registered targets absent and launch replay once**

Exact raw target:

```text
/private/tmp/cbf2026-predictive-wnls-development/stage1-v2
```

Require clean protocol-bound files, target absence, and start space:

```bash
git diff --quiet HEAD -- \
  scripts/diagnostics/predictive_wnls.py \
  scripts/diagnostics/replay_predictive_wnls_recovery.py \
  scripts/diagnostics/analyze_predictive_wnls_recovery.py \
  docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.json
test ! -e /private/tmp/cbf2026-predictive-wnls-development/stage1-v2
df -k /private/tmp
```

Run exactly:

```bash
conda run -n cbf_env python \
  scripts/diagnostics/replay_predictive_wnls_recovery.py \
  --data-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  --input-manifest-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json \
  --protocol-json docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.json \
  --output-root /private/tmp/cbf2026-predictive-wnls-development/stage1-v2 \
  --run-seeds 20260727,20260728,20260729,20260730,20260731,20260732,20260733,20260734,20260735,20260736,20260737,20260738,20260739,20260740,20260741,20260742,20260743,20260744,20260745,20260746
```

No retry is allowed.
On failure, preserve the terminal manifest and stop this plan before analysis.

- [ ] **Step 4: Launch analyzer once**

Exact compact target:

```text
/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v2
```

Before launch:

```bash
test ! -e /private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v2
```

Run exactly:

```bash
conda run -n cbf_env python \
  scripts/diagnostics/analyze_predictive_wnls_recovery.py \
  --baseline-process-path /private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/restart/localization-calibration/20260729T112437.413270Z_efb8e7137776415c833f11204cf519d5/calibration.jsonl.gz \
  --development-manifest-path /private/tmp/cbf2026-predictive-wnls-development/stage1-v2/manifest.json \
  --protocol-json docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.json \
  --expected-baseline-sha256 c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003 \
  --output-root /private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v2
```

No retry is allowed.

- [ ] **Step 5: Independently audit raw evidence**

The evidence reviewer recomputes from raw rows:

- every attempt/output/status-transition count;
- fresh/all-published/both-fresh/both-published tails;
- baseline errors on fresh→predicted and newly unavailable keys;
- catastrophic counts and availability gates;
- maximum prediction age and unavailable streak;
- reference/provenance/DAG violations;
- frame44/frame177 records;
- fresh/aged containment and q calibration;
- `243/7000` input-limit violations and command maxima; and
- all source/input/output hashes.

Require zero scalar mismatch with compact JSON and no Critical/Important
finding.

- [ ] **Step 6: Write reports, verify, and commit compact records**

The author report separates fresh-estimate improvement, prediction continuity,
status downgrade, unavailability attrition, rejected candidates, remaining
tails, every gate result, and the invalid old 25 m/s premise.
It states that Stage 1 is one-trajectory development evidence and cannot open
the paper gate.

```bash
conda run -n cbf_env python -m unittest discover \
  -s tests -p 'test_*.py' -q
git diff --check
git add docs/diagnostics/2026-07-30-predictive-wnls-stage1.md \
  docs/diagnostics/reviews/2026-07-30-predictive-wnls-stage1-evidence-review.md
git commit -m "docs(diagnostics): record predictive WNLS stage 1"
```

Do not commit raw process streams.

---

### Task 9: DRA closeout and bounded-input Stage 2 decision

**Files in `/private/tmp/dra-cbf2026-diagnostic`:**
- Create: `meta-log/2026-07-30-cbf2026-predictive-wnls-recovery.md`
- Modify: `papers/cbf2026/status.md`
- Modify: `papers/cbf2026/open-questions.md`
- Modify: `papers/cbf2026/timeline.md`
- Modify: `papers/cbf2026/submissions.md`

**Conditional file in `/private/tmp/cbf2026-diagnostic`:**
- Create only if every Stage 1 code/evidence gate passes:
  `docs/superpowers/plans/2026-07-30-cbf2026-bounded-input-stage2.md`

- [ ] **Step 1: Update DRA append-only**

Record design/plan/implementation/review commits, frozen source/input hashes,
the `999.3319 m` stale and `168.9017 m` fresh mechanisms, the discovered
disabled input limits, every Stage 0/1 metric with denominator, fresh versus
prediction versus attrition effects, every failed gate, raw/compact manifest
paths/hashes, and why paper editing remains closed.

- [ ] **Step 2: Independently review DRA**

Require exact agreement with compact JSON, evidence review, and Git history.
Preserve old negative evidence and historical plans.
Resolve every Critical/Important finding.

- [ ] **Step 3: Commit DRA without pushing**

```bash
git diff --check
git add meta-log/2026-07-30-cbf2026-predictive-wnls-recovery.md \
  papers/cbf2026/status.md \
  papers/cbf2026/open-questions.md \
  papers/cbf2026/timeline.md \
  papers/cbf2026/submissions.md
git commit -m "docs(cbf2026): record predictive WNLS recovery"
```

- [ ] **Step 4: Make Stage 2 go/no-go decision**

If Stage 1 passes, write a separate implementation plan bound to the approved
design for:

```text
12 independent 250 s trajectories
5 nested predeclared range-noise seeds per trajectory
cbfs.input-limits.on = true
planar-component-max = 25.0
per-row opt.input_limits.enabled = true
abs(vx), abs(vy) <= 25.0 + 1e-7
componentwise state delta = 0.5 * opt.result within 1e-7 m
two-sided 95% Wilson trajectory-incidence intervals
the same baseline-fresh and baseline-published fixed-cohort audits
```

Do not generate Stage 2 trajectories in this plan.
If Stage 1 fails, record the failure and do not tune against the same output.

---

## Final verification

- [ ] Run the full Python diagnostic suite in `cbf_env`.
- [ ] Confirm the legacy replay SHA-256 remains `0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8`.
- [ ] Confirm old exactly-once JSON/Markdown hashes remain:

```text
a02098e231c3b45b65f6213d6708dcae2fb68d9cbcc6e52103b0e8c6f013d54b
1e60aac9eb7dd8f9a2e1ad89ed25614382fa8b7fdfe01c980ae12bb21d97982a
```

- [ ] Run `git diff --check` in code and DRA worktrees.
- [ ] Confirm code worktree has only permitted `?? build-diagnostic/`.
- [ ] Confirm primary paper and primary DRA checkouts are unchanged.
- [ ] Confirm registered raw/compact targets each contain one terminal
  manifest and no nested run directory.
- [ ] Confirm `/private/tmp` remains above 6 GB, raw/cache below 2 GB, and
  compact output below 10 MB.
- [ ] Confirm every registered key and adverse row is present.
- [ ] Confirm no Critical/Important review issue remains.
- [ ] Confirm the paper was not edited.
- [ ] Confirm Stage 2, if warranted, is a separate plan and no new trajectory
  was generated by this plan.
