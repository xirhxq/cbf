# CBF2026 Two-Range Reacquisition Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement and audit a fail-closed exactly-two-current-fresh-UAV-range reacquisition path that solves both circle-origin WNLS branches independently and uses the tagged private ZOH state only for disclosed discrete branch selection.

**Architecture:** Keep the completed Stage 1 v4 producer, analyzer, registrar, schemas, and evidence roots immutable. Add one focused pure estimator/lifecycle module, one single-method producer, one independent analyzer, and one registrar; change the existing candidate gate only through a default-disabled two-reference option so every old call remains byte-for-byte semantically unchanged. Freeze implementation before protocol generation, permit only synthetic/unit tests and the single approved mechanism fixture before registration, then pass a separate preflight and deterministic-smoke gate before asking for authorization to execute the one no-retry full grid.

**Tech Stack:** Python 3 in conda environment `cbf_env`, NumPy, `unittest`, gzip JSONL, SHA-256, existing CBF2026 diagnostic evidence-lifecycle helpers, Git.

## Global Constraints

- Work only in `/private/tmp/cbf2026-diagnostic` on branch `codex/cbf2026-diagnostic`; preserve the unrelated untracked `build-diagnostic/`.
- Approved design commit: `20a61aad96af35ee7e16434fab0a5edaaea38ef0`.
- Approved design SHA-256: `d28d6bfe297a6e37b0a878dc716bd5b91f26c5b11568c3f6fc42bd07760a266b`.
- Approved review SHA-256: `6d857be965de5b83bb05ce06381b79108fdc4abbccf6b96fe4ec456d7a7df570`.
- Stable method ID: `two_range_private_branch_reacquisition`.
- Protocol schema: `cbf2026-two-range-reacquisition-protocol-v1`.
- Raw schema: `cbf2026-two-range-reacquisition-raw-v1`.
- Analysis schema: `cbf2026-two-range-reacquisition-analysis-v1`.
- Registration/authorization schema: `cbf2026-two-range-reacquisition-registration-v1`.
- Before any implementation task starts,
  this final approved plan must be reviewed,
  committed on the working branch,
  and recorded in the DRA.
  The implementation report records that planning-closure commit and the
  plan byte size/SHA-256;
  the registrar later binds the committed plan as an exact protocol source.
- The new selector is considered only with no live public prediction, exactly two active mandatory UAV references, no active base reference, no optional reference, both fixed CBF references present, current-fresh strictly lower-index states, finite ranges, and valid recursive base provenance.
- The only continuous WNLS starts on the new path are canonical `circle_negative` and `circle_positive`; private, algebraic, prediction, and every other start are forbidden.
- The private state may affect only the post-solve discrete branch identity; it must never be a range, anchor, continuous-update input, FIM term, covariance source, or publication representative.
- Keep `MAX_PUBLIC_PREDICTION_AGE = 2`, `INNOVATION_REFERENCE_QUANTILE = 11.829007011943707`, `CANDIDATE_DEDUP_M = 1e-9 m`, \(Q=0.25I\), and all Stage 1 scientific gates unchanged.
- For \(N=2\), the selected candidate must satisfy `cost / max(1, N - 2) <= 9.0`, hence `cost <= 9.0`; equality passes.
- `fresh` means a current-frame continuous WNLS measurement update. Its reported covariance and epsilon are local modeled quantities conditional on the selected mirror branch being correct.
- Production key order is exactly `(method, seed, frame, robot)` over one method, seeds `20260727..20260746`, frames `0..499`, and robots `1..14`: exactly `140000` rows.
- No unregistered full grid, partitioned equivalent, trajectory subset aggregate, or scientific error/availability aggregate may be produced before the registered run.
- Before registration, the new method may execute only unit/adversarial tests, declared synthetic smoke cases, and fixture key `(20260727, 180, 12)`.
- Full registered execution requires a clean committed implementation, committed protocol, independent preflight, deterministic smoke review, and a separate explicit authorization checkpoint.
- Require at least `8_000_000_000` free bytes at launch, stop below `6_000_000_000`, cap raw allocated bytes at `2_000_000_000`, and cap compact output at `10_000_000`.
- Never delete, overwrite, rename, or rebind Stage 1 v2/v3/v4 roots or any failed new root.
- A failed registered replay or analyzer is retained as forensic evidence and ends this protocol; it is never retried under the same ID.
- Do not modify `main.tex`, generate Stage 2 trajectories, update paper claims, or describe the estimator as entering the controller.
- Run Python through `conda run -n cbf_env`.
- Use `apply_patch` for source and documentation edits; make no commit with `Co-Authored-By`.
- Apply every behavioral matrix as a vertical microcycle:
  write one smallest behavior test,
  run it and confirm it is red for the intended behavior,
  add only the minimum implementation,
  rerun to green,
  then move to the next case.
  Only the first bootstrap test for a genuinely absent production API may
  be red because of an import/missing symbol;
  later tests must fail on an assertion,
  not because implementation was deferred in bulk.
- Update the DRA only in `/private/tmp/dra-cbf2026-diagnostic` on `main`, append-only, without touching `papers/cbf2026/submissions.md` or other papers, and do not push unless the user asks.

## File Structure

- Create `scripts/diagnostics/two_range_reacquisition.py`: pure tagged-private-state lifecycle, branch-score rule, and two-circle WNLS selector.
- Modify `scripts/diagnostics/predictive_wnls.py`: add a default-disabled `allow_two_reference_reacquisition` gate argument; preserve every existing caller.
- Create `scripts/diagnostics/extract_two_range_reacquisition_fixture.py`: deterministic read-only extraction of the approved mechanism fixture from preserved inputs.
- Create `scripts/diagnostics/replay_two_range_reacquisition.py`: exact one-method raw schema, routing, row production, ordering, disk, and terminal lifecycle.
- Create `scripts/diagnostics/analyze_two_range_reacquisition.py`: independent reconstruction, adversarial validation, paired gates, compact output, and lifecycle.
- Create `scripts/diagnostics/register_two_range_reacquisition.py`: exact protocol/authorization generation and source/comparator/root binding.
- Create `tests/test_two_range_reacquisition.py`: pure state, branch, cost, threshold, and failure tests.
- Create `tests/test_extract_two_range_reacquisition_fixture.py`: source-identity and deterministic fixture tests.
- Create `tests/test_replay_two_range_reacquisition.py`: producer/schema/routing/order/lifecycle tests.
- Create `tests/test_analyze_two_range_reacquisition.py`: independent reconstruction, gate, and tamper tests.
- Create `tests/test_register_two_range_reacquisition.py`: registrar/protocol/preflight contract tests.
- Create `tests/fixtures/cbf2026_two_range_reacquisition/mechanism_20260727_180_12.json`: the only real pre-registration mechanism fixture.
- Create `tests/fixtures/cbf2026_two_range_reacquisition/manifest.json`: exact source and fixture hashes.
- Generate only after implementation review:
  `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json`.
- Generate only after implementation review:
  `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.md`.
- Create during preflight:
  `docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1-review.md`.
- Create after smoke:
  `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-smoke.md`.
- Create only after explicit registered-run authorization:
  `docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-registered-authorization.json`.
- Create after registered evidence:
  `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition.md`.
- Create after registered evidence:
  `docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-evidence-review.md`.

---

### Task 1: Tagged private-state lifecycle

**Files:**
- Create: `scripts/diagnostics/two_range_reacquisition.py`
- Create: `tests/test_two_range_reacquisition.py`
- Test: `tests/test_predictive_wnls.py`

**Interfaces:**
- Consumes:
  `canonical_spd_covariance`,
  `finalize_attempt`,
  `make_unavailable_output`,
  `propagate_estimator_prior`,
  and `FRAME_DT_SECONDS`
  from `scripts.diagnostics.predictive_wnls`.
- Produces:
  - `METHOD_ID: str`
  - `PRIVATE_STATE_FIELDS: tuple[str, ...]`
  - `canonical_private_state(value: object) -> dict | None`
  - `reset_private_state(candidate: object, *, frame_index: int) -> dict | None`
  - `propagate_private_state(previous_state: object, held_velocity: object, *, next_frame_index: int, dt: float = FRAME_DT_SECONDS) -> dict | None`
  - `advance_two_range_prior(previous_public: object, previous_private: object, held_velocity: object, *, next_frame_index: int) -> dict`
  - `finalize_two_range_lifecycle(attempt: object, prior_bundle: object, *, frame_index: int) -> dict`

- [ ] **Step 1: Write failing canonical-state and reset tests**

```python
class TaggedPrivateStateTests(unittest.TestCase):
    def test_reset_uses_current_fresh_candidate_and_age_zero(self):
        candidate = {
            "estimate": [1.0, -2.0],
            "modeled_covariance": [[2.0, 0.1], [0.1, 1.0]],
        }
        state = reset_private_state(candidate, frame_index=17)
        self.assertEqual(state["status"], "available")
        self.assertEqual(state["source_fresh_frame"], 17)
        self.assertEqual(state["propagated_to_frame"], 17)
        self.assertEqual(state["age_frames"], 0)
        self.assertEqual(state["estimate"], [1.0, -2.0])
```

- [ ] **Step 2: Run the focused test and verify the red state**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_two_range_reacquisition.TaggedPrivateStateTests.test_reset_uses_current_fresh_candidate_and_age_zero -v
```

Expected: FAIL because `scripts.diagnostics.two_range_reacquisition` does not exist.

- [ ] **Step 3: Implement exact private-state canonicalization and reset**

```python
from collections.abc import Mapping
from numbers import Integral, Real

import numpy as np

METHOD_ID = "two_range_private_branch_reacquisition"
PRIVATE_STATE_FIELDS = (
    "status",
    "estimate",
    "modeled_covariance",
    "source_fresh_frame",
    "propagated_to_frame",
    "age_frames",
)

def _finite_vector(value: object) -> np.ndarray | None:
    try:
        vector = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return None
    if vector.shape != (2,) or not np.isfinite(vector).all():
        return None
    return vector

def canonical_private_state(value: object) -> dict | None:
    if not isinstance(value, Mapping):
        return None
    if set(value) != set(PRIVATE_STATE_FIELDS):
        return None
    if value.get("status") != "available":
        return None
    estimate = _finite_vector(value.get("estimate"))
    covariance = canonical_spd_covariance(
        value.get("modeled_covariance")
    )
    if estimate is None or covariance is None:
        return None
    indices = (
        value.get("source_fresh_frame"),
        value.get("propagated_to_frame"),
        value.get("age_frames"),
    )
    if any(
        isinstance(item, bool)
        or not isinstance(item, Integral)
        or item < 0
        for item in indices
    ):
        return None
    source, propagated, age = (int(item) for item in indices)
    if propagated - source != age:
        return None
    return {
        "status": "available",
        "estimate": estimate.tolist(),
        "modeled_covariance": covariance.tolist(),
        "source_fresh_frame": source,
        "propagated_to_frame": propagated,
        "age_frames": age,
    }

def reset_private_state(candidate: object, *, frame_index: int) -> dict | None:
    if not isinstance(candidate, Mapping):
        return None
    estimate = _finite_vector(candidate.get("estimate"))
    covariance = canonical_spd_covariance(
        candidate.get("modeled_covariance")
    )
    if estimate is None or covariance is None:
        return None
    if isinstance(frame_index, bool) or not isinstance(frame_index, Integral):
        return None
    if frame_index < 0:
        return None
    return {
        "status": "available",
        "estimate": estimate.tolist(),
        "modeled_covariance": covariance.tolist(),
        "source_fresh_frame": int(frame_index),
        "propagated_to_frame": int(frame_index),
        "age_frames": 0,
    }
```

- [ ] **Step 4: Add failing exact-once propagation tests**

The three tests in this step are an ordered microcycle queue,
not a batch.
Add only `test_propagation_is_exactly_one_transition`,
run it red,
implement only the valid one-step path from Step 5,
and rerun it green.
Next add only the same/skipped-frame rejection test,
run it red,
add the frame-transition guard,
and rerun it green.
Finally add only the no-public-age-expiry test,
run it red,
add the long-lived-private-state behavior,
and rerun it green.
The code below freezes the three end-state tests,
but they must not all be written before the first implementation.

```python
def test_propagation_is_exactly_one_transition(self):
    state = reset_private_state(
        {
            "estimate": [1.0, 2.0],
            "modeled_covariance": [[1.0, 0.0], [0.0, 2.0]],
        },
        frame_index=8,
    )
    propagated = propagate_private_state(
        state,
        [4.0, -2.0],
        next_frame_index=9,
    )
    self.assertEqual(propagated["estimate"], [3.0, 1.0])
    self.assertEqual(
        propagated["modeled_covariance"],
        [[1.25, 0.0], [0.0, 2.25]],
    )
    self.assertEqual(propagated["source_fresh_frame"], 8)
    self.assertEqual(propagated["propagated_to_frame"], 9)
    self.assertEqual(propagated["age_frames"], 1)

def test_same_frame_or_skipped_frame_propagation_rejects(self):
    state = reset_private_state(
        {
            "estimate": [0.0, 0.0],
            "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
        },
        frame_index=4,
    )
    self.assertIsNone(
        propagate_private_state(state, [0.0, 0.0], next_frame_index=4)
    )
    self.assertIsNone(
        propagate_private_state(state, [0.0, 0.0], next_frame_index=6)
    )

def test_private_state_has_no_public_prediction_age_expiry(self):
    state = {
        "status": "available",
        "estimate": [10.0, 20.0],
        "modeled_covariance": [[25.0, 0.0], [0.0, 25.0]],
        "source_fresh_frame": 0,
        "propagated_to_frame": 99,
        "age_frames": 99,
    }
    propagated = propagate_private_state(
        state,
        [0.0, 0.0],
        next_frame_index=100,
    )
    self.assertEqual(propagated["age_frames"], 100)
    self.assertEqual(propagated["propagated_to_frame"], 100)
```

- [ ] **Step 5: Implement one-step ZOH propagation without public feedback**

Build this function incrementally through the three Step 4 microcycles.
The complete green definition must equal:

```python
def propagate_private_state(
    previous_state: object,
    held_velocity: object,
    *,
    next_frame_index: int,
    dt: float = FRAME_DT_SECONDS,
) -> dict | None:
    state = canonical_private_state(previous_state)
    velocity = _finite_vector(held_velocity)
    if state is None or velocity is None:
        return None
    if (
        isinstance(next_frame_index, bool)
        or not isinstance(next_frame_index, Integral)
        or isinstance(dt, bool)
        or not isinstance(dt, Real)
        or not np.isfinite(float(dt))
        or float(dt) <= 0.0
    ):
        return None
    if next_frame_index != state["propagated_to_frame"] + 1:
        return None
    estimate = np.asarray(state["estimate"]) + float(dt) * velocity
    covariance = (
        np.asarray(state["modeled_covariance"])
        + 0.25 * np.eye(2)
    )
    return {
        "status": "available",
        "estimate": estimate.tolist(),
        "modeled_covariance": covariance.tolist(),
        "source_fresh_frame": state["source_fresh_frame"],
        "propagated_to_frame": next_frame_index,
        "age_frames": next_frame_index - state["source_fresh_frame"],
    }
```

- [ ] **Step 6: Add failing public/private separation tests**

Treat the five cases below as five ordered vertical microcycles.
For each case,
add only that test,
run it red for the intended assertion,
implement the smallest corresponding branch of
`advance_two_range_prior` or `finalize_two_range_lifecycle`,
and rerun it green before adding the next case.
The listing freezes the final suite,
not a batch-red instruction:

```python
def test_expired_public_prediction_does_not_delete_private_state(self):
    bundle = advance_two_range_prior(
        previous_public={
            "output_status": "predicted",
            "prediction_age": 2,
            "estimate": [99.0, 99.0],
            "modeled_covariance": [[9.0, 0.0], [0.0, 9.0]],
            "epsilon": None,
            "aged_modeled_radius": 9.0,
            "base_anchor_provenance": [],
        },
        previous_private={
            "status": "available",
            "estimate": [1.0, 2.0],
            "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
            "source_fresh_frame": 0,
            "propagated_to_frame": 2,
            "age_frames": 2,
        },
        held_velocity=[2.0, 0.0],
        next_frame_index=3,
    )
    self.assertEqual(
        bundle["public_prediction"]["output_status"], "unavailable"
    )
    self.assertEqual(
        bundle["branch_selection_prior"]["estimate"], [2.0, 2.0]
    )
    self.assertEqual(
        bundle["branch_selection_prior"]["propagated_to_frame"], 3
    )

def test_live_public_prediction_never_uses_private_state_as_source(self):
    bundle = advance_two_range_prior(
        previous_public={
            "output_status": "fresh",
            "prediction_age": 0,
            "estimate": [100.0, 100.0],
            "modeled_covariance": [[4.0, 0.0], [0.0, 4.0]],
            "epsilon": 6.0,
            "aged_modeled_radius": None,
            "base_anchor_provenance": [0, 1],
        },
        previous_private={
            "status": "available",
            "estimate": [1.0, 2.0],
            "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
            "source_fresh_frame": 0,
            "propagated_to_frame": 0,
            "age_frames": 0,
        },
        held_velocity=[2.0, 0.0],
        next_frame_index=1,
    )
    self.assertEqual(
        bundle["public_prediction"]["estimate"], [101.0, 100.0]
    )
    self.assertEqual(
        bundle["branch_selection_prior"]["estimate"], [2.0, 2.0]
    )

def test_frame_zero_has_no_incoming_public_or_private_prior(self):
    bundle = advance_two_range_prior(
        None,
        None,
        None,
        next_frame_index=0,
    )
    self.assertEqual(
        bundle["public_prediction"]["output_status"], "unavailable"
    )
    self.assertIsNone(bundle["branch_selection_prior"])

def test_accepted_attempt_resets_both_public_and_private_state(self):
    prior = {
        "public_prediction": make_unavailable_output("expired"),
        "branch_selection_prior": {
            "status": "available",
            "estimate": [9.0, 9.0],
            "modeled_covariance": [[5.0, 0.0], [0.0, 5.0]],
            "source_fresh_frame": 1,
            "propagated_to_frame": 7,
            "age_frames": 6,
        },
    }
    result = finalize_two_range_lifecycle(
        {
            "attempt_status": "accepted",
            "candidate": {
                "estimate": [1.0, 2.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 4.0]],
                "base_anchor_provenance": [0, 1],
            },
        },
        prior,
        frame_index=7,
    )
    self.assertEqual(result["public_output"]["output_status"], "fresh")
    self.assertEqual(result["next_private_state"]["estimate"], [1.0, 2.0])
    self.assertEqual(result["next_private_state"]["source_fresh_frame"], 7)
    self.assertEqual(result["next_private_state"]["age_frames"], 0)

def test_rejected_attempt_preserves_already_propagated_private_state(self):
    incoming = {
        "status": "available",
        "estimate": [9.0, 9.0],
        "modeled_covariance": [[5.0, 0.0], [0.0, 5.0]],
        "source_fresh_frame": 1,
        "propagated_to_frame": 7,
        "age_frames": 6,
    }
    result = finalize_two_range_lifecycle(
        {
            "attempt_status": "rejected",
            "candidate": None,
        },
        {
            "public_prediction": make_unavailable_output("expired"),
            "branch_selection_prior": incoming,
        },
        frame_index=7,
    )
    self.assertEqual(
        result["public_output"]["output_status"], "unavailable"
    )
    self.assertEqual(result["next_private_state"], incoming)
```

- [ ] **Step 7: Implement `advance_two_range_prior` and finalization**

Build these functions incrementally through the five Step 6 microcycles.
Do not wait for all five tests to be red.
The complete green definitions must equal:

```python
def advance_two_range_prior(
    previous_public: object,
    previous_private: object,
    held_velocity: object,
    *,
    next_frame_index: int,
) -> dict:
    unavailable = make_unavailable_output("no_live_public_prediction")
    if (
        isinstance(next_frame_index, bool)
        or not isinstance(next_frame_index, Integral)
        or next_frame_index < 0
    ):
        return {
            "public_prediction": unavailable,
            "branch_selection_prior": None,
        }
    if next_frame_index == 0:
        return {
            "public_prediction": unavailable,
            "branch_selection_prior": None,
        }
    velocity = _finite_vector(held_velocity)
    if velocity is None:
        return {
            "public_prediction": unavailable,
            "branch_selection_prior": None,
        }
    incoming = propagate_private_state(
        previous_private,
        velocity,
        next_frame_index=next_frame_index,
    )
    public_bundle = propagate_estimator_prior(
        (
            dict(previous_public)
            if isinstance(previous_public, Mapping)
            else None
        ),
        None,
        velocity,
    )
    return {
        "public_prediction": public_bundle["public_prediction"],
        "branch_selection_prior": incoming,
    }

def finalize_two_range_lifecycle(
    attempt: object,
    prior_bundle: object,
    *,
    frame_index: int,
) -> dict:
    if not isinstance(attempt, Mapping) or not isinstance(
        prior_bundle, Mapping
    ):
        raise ValueError("attempt and prior_bundle must be mappings")
    public_output = finalize_attempt(
        dict(attempt),
        {"public_prediction": prior_bundle.get("public_prediction")},
        frame_index=frame_index,
    )
    if public_output["output_status"] == "fresh":
        next_private_state = reset_private_state(
            public_output,
            frame_index=frame_index,
        )
    else:
        next_private_state = canonical_private_state(
            prior_bundle.get("branch_selection_prior")
        )
    return {
        "public_output": public_output,
        "next_private_state": next_private_state,
}
```

The returned `private_reacquisition_seed` from
`propagate_estimator_prior` is deliberately discarded.
The public channel is propagated only from the previous public output,
while the tagged private channel is propagated only from the previous
private state.

- [ ] **Step 8: Run focused and legacy lifecycle tests**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_two_range_reacquisition.TaggedPrivateStateTests \
  tests.test_predictive_wnls.PredictiveLifecycleTests \
  tests.test_predictive_wnls.FinalizeAttemptTests -v
```

Expected: all tests PASS; no old status or age semantics change.

- [ ] **Step 9: Commit the independently testable lifecycle**

```bash
git add \
  scripts/diagnostics/two_range_reacquisition.py \
  tests/test_two_range_reacquisition.py
git commit -m "feat(diagnostics): add tagged private-state lifecycle"
```

### Task 2: Fail-closed two-circle branch selector

**Files:**
- Modify: `scripts/diagnostics/predictive_wnls.py:1283`
- Modify: `scripts/diagnostics/two_range_reacquisition.py`
- Modify: `tests/test_predictive_wnls.py:967`
- Modify: `tests/test_two_range_reacquisition.py`

**Interfaces:**
- Consumes:
  - `solve_finite_budget_wnls(reference_positions: object, reference_covariances: object, measurements: object, initial_estimate: object, ranging_sigma: float) -> dict`
  - `_complete_converged_solver_result(result: object) -> bool`
  - `two_circle_candidates(first_position: object, first_range: float, second_position: object, second_range: float) -> tuple[np.ndarray, ...]`
  - `normalized_innovation(candidate_estimate: object, candidate_covariance: object, live_prediction: dict) -> dict`
  - `candidate_acceptance(candidate_result: dict, *, live_prediction: dict | None, active_reference_count: int, base_anchor_provenance: object, allow_two_reference_reacquisition: bool = False) -> tuple[bool, str, dict]`
  - tagged private-state interfaces from Task 1.
- Produces:
  - `BRANCH_IDS = ("circle_negative", "circle_positive")`
  - `branch_gate_passes(q_value: object) -> bool`
  - `validate_solver_branches(branches: object) -> tuple[bool, str]`
  - `solve_two_range_reacquisition(*, robot_id: int, reference_positions: object, reference_covariances: object, measurements: object, reference_keys: object, private_prior: object, ranging_sigma: float, base_anchor_provenance: object) -> dict`
  - the exact extended `candidate_acceptance` signature listed above.

- [ ] **Step 1: Pin unchanged default candidate-gate behavior with a red option test**

Add:

```python
def test_two_reference_reacquisition_requires_explicit_option(self):
    result = make_converged_candidate_result(cost=9.0)
    default = candidate_acceptance(
        result,
        live_prediction=None,
        active_reference_count=2,
        base_anchor_provenance=[0, 1],
    )
    enabled = candidate_acceptance(
        result,
        live_prediction=None,
        active_reference_count=2,
        base_anchor_provenance=[0, 1],
        allow_two_reference_reacquisition=True,
    )
    self.assertEqual(default[1], "reacquisition_requires_three_active_references")
    self.assertTrue(enabled[0])
```

- [ ] **Step 2: Run the focused gate test and verify it fails**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_predictive_wnls.CandidateAcceptanceTests.test_two_reference_reacquisition_requires_explicit_option -v
```

Expected: FAIL because the keyword argument is not accepted.

- [ ] **Step 3: Add failing option-type and exact-cost tests**

After Step 1/2 is made green by adding only the default-disabled Boolean
option,
add the cases in this step one at a time.
For each case,
run only the smallest focused test red,
implement the minimum gate branch,
and rerun it green before adding the next case.
The final cases cover `9.0`, `nextafter(9.0, -inf)`, and
`nextafter(9.0, +inf)`.
The first two pass with the option enabled;
the last rejects with
`reacquisition_reduced_cost_exceeds_nine`.
Also pass `allow_two_reference_reacquisition=1`
and
`allow_two_reference_reacquisition="true"`;
each must reject as
`invalid_two_reference_reacquisition_option`.
After every microcycle is green,
run the entire `CandidateAcceptanceTests` class and verify all new and
legacy cases pass.

- [ ] **Step 4: Add the default-disabled two-reference gate**

Change the signature to:

```python
def candidate_acceptance(
    candidate_result: dict,
    *,
    live_prediction: dict | None,
    active_reference_count: int,
    base_anchor_provenance: object,
    allow_two_reference_reacquisition: bool = False,
) -> tuple[bool, str, dict]:
```

Place the option-type validation immediately after `diagnostics` is created,
before any live-prediction or active-count branch:

```python
if not isinstance(allow_two_reference_reacquisition, bool):
    return (
        False,
        "invalid_two_reference_reacquisition_option",
        diagnostics,
    )
```

Leave the existing live-prediction branch unchanged.
Only in the existing no-live-prediction reacquisition branch,
replace the three-reference check with:

```python
if active_count < 3 and not (
    allow_two_reference_reacquisition and active_count == 2
):
    diagnostics["gate_outcome"] = "rejected"
    return (
        False,
        "reacquisition_requires_three_active_references",
        diagnostics,
    )
reduced_cost = cost / max(1, active_count - 2)
```

Do not change any existing caller.
Rerun `CandidateAcceptanceTests`;
all cases must pass.

- [ ] **Step 5: Write failing two-branch selection tests**

Use two references at `[0, 0]` and `[2, 0]`,
both ranges `sqrt(2)`,
and priors near `[1, -1]` and `[1, 1]`.
Create and reuse this exact helper:

```python
def valid_two_range_kwargs() -> dict:
    return {
        "robot_id": 12,
        "reference_positions": [[0.0, 0.0], [2.0, 0.0]],
        "reference_covariances": [
            [[0.1, 0.0], [0.0, 0.1]],
            [[0.1, 0.0], [0.0, 0.1]],
        ],
        "measurements": [math.sqrt(2.0), math.sqrt(2.0)],
        "reference_keys": [("uav", 10), ("uav", 11)],
        "private_prior": reset_private_state(
            {
                "estimate": [1.0, 1.75],
                "modeled_covariance": [[0.2, 0.0], [0.0, 0.2]],
            },
            frame_index=20,
        ),
        "ranging_sigma": 0.5,
        "base_anchor_provenance": [0, 1],
    }
```

Assert:

```python
attempt = solve_two_range_reacquisition(**valid_two_range_kwargs())
self.assertEqual(
    [row["branch_id"] for row in attempt["branches"]],
    ["circle_negative", "circle_positive"],
)
self.assertEqual(attempt["selected_branch_id"], "circle_positive")
self.assertTrue(
    math.isclose(
        attempt["branches"][0]["q_branch"],
        13.750000000000005,
        rel_tol=1e-12,
        abs_tol=1e-12,
    )
)
self.assertTrue(
    math.isclose(
        attempt["branches"][1]["q_branch"],
        1.0227272727272725,
        rel_tol=1e-12,
        abs_tol=1e-12,
    )
)
self.assertFalse(attempt["branches"][0]["passes_branch_gate"])
self.assertTrue(attempt["branches"][1]["passes_branch_gate"])
self.assertTrue(attempt["prior_used_for_branch_selection"])
self.assertFalse(attempt["prior_used_in_fim"])
self.assertFalse(attempt["prior_used_for_continuous_update"])
```

In the first selector microcycle,
add only the direct `branch_gate_passes` tests:

```python
q_star = 11.829007011943707
self.assertTrue(branch_gate_passes(q_star))
self.assertTrue(branch_gate_passes(np.nextafter(q_star, -np.inf)))
self.assertFalse(branch_gate_passes(np.nextafter(q_star, np.inf)))
self.assertFalse(branch_gate_passes(float("nan")))
self.assertFalse(branch_gate_passes(float("inf")))
```

Run only these gate tests now.
Expected:
FAIL because `branch_gate_passes` does not yet exist.
Implement only `branch_gate_passes`,
rerun these tests,
and require PASS before starting the next microcycle.

In the second selector microcycle,
call `validate_solver_branches` with two branch records whose
`solver_result` values are separate copies of the same complete converged
test result;
the red expectation is
`(False, "two_range_solver_branches_merged")`.
Run that one test,
implement only structural validation and the merged-result guard,
then rerun it to PASS.

In the third selector microcycle,
write only the valid two-branch selection test above.
Run it and require a red failure because
`solve_two_range_reacquisition` does not exist.
Implement only deterministic circle construction,
the two WNLS calls,
both branch-score computations,
and unique admissible selection.
Rerun the valid case to PASS.

Then handle every fail-closed/adversarial case enumerated in Step 7 and
every private-isolation assertion in Step 8 as its own smallest
red/implementation/green microcycle.
Do not pre-write the entire adversarial suite before the valid selector
microcycle is green.

- [ ] **Step 6: Implement deterministic branch construction and scoring**

The code below freezes the final green target,
not a license to add every guard at once.
The two helper bodies already made green in Step 5 are retained.
Add only the valid deterministic branch/scoring path needed by the third
Step 5 microcycle.
Every remaining validation guard shown below must wait until its matching
Step 7 adversarial test has first failed for the intended assertion;
then add only that guard and rerun that test green.
The final implementation order inside the completed function is fixed:

```python
def branch_gate_passes(q_value: object) -> bool:
    if isinstance(q_value, bool) or not isinstance(q_value, Real):
        return False
    q = float(q_value)
    return bool(
        np.isfinite(q)
        and q >= 0.0
        and q <= INNOVATION_REFERENCE_QUANTILE
    )

def validate_solver_branches(
    branches: object,
) -> tuple[bool, str]:
    if (
        not isinstance(branches, list)
        or len(branches) != 2
        or any(not isinstance(branch, Mapping) for branch in branches)
        or any(
            not _complete_converged_solver_result(
                branch.get("solver_result")
            )
            for branch in branches
        )
    ):
        return False, "two_range_branch_solver_invalid"
    estimates = [
        np.asarray(
            branch["solver_result"]["estimate"],
            dtype=float,
        )
        for branch in branches
    ]
    if (
        np.linalg.norm(estimates[0] - estimates[1])
        <= CANDIDATE_DEDUP_M
    ):
        return False, "two_range_solver_branches_merged"
    return True, "accepted"

def _rejected_attempt(
    reason: str,
    *,
    branches: list[dict] | None = None,
    prior_used: bool = False,
    selected_branch: dict | None = None,
) -> dict:
    return {
        "attempt_status": "rejected",
        "status": "rejected",
        "failure_reason": reason,
        "branches": [] if branches is None else branches,
        "selected_branch_id": (
            None
            if selected_branch is None
            else selected_branch["branch_id"]
        ),
        "selected_candidate": selected_branch,
        "candidate": None,
        "prior_used_for_branch_selection": prior_used,
        "prior_used_in_fim": False,
        "prior_used_for_continuous_update": False,
    }

def solve_two_range_reacquisition(
    *,
    robot_id: int,
    reference_positions: object,
    reference_covariances: object,
    measurements: object,
    reference_keys: object,
    private_prior: object,
    ranging_sigma: float,
    base_anchor_provenance: object,
) -> dict:
    if (
        isinstance(robot_id, bool)
        or not isinstance(robot_id, Integral)
        or robot_id < 1
    ):
        return _rejected_attempt("two_range_robot_id_invalid")
    try:
        positions = np.asarray(reference_positions, dtype=float)
        ranges = np.asarray(measurements, dtype=float)
        raw_covariances = tuple(reference_covariances)
        raw_keys = tuple(reference_keys)
    except (TypeError, ValueError, OverflowError):
        return _rejected_attempt("two_range_input_invalid")
    if (
        positions.shape != (2, 2)
        or ranges.shape != (2,)
        or len(raw_covariances) != 2
        or len(raw_keys) != 2
        or not np.isfinite(positions).all()
        or not np.isfinite(ranges).all()
        or np.any(ranges <= 0.0)
    ):
        return _rejected_attempt("two_range_input_invalid")
    canonical_covariances = tuple(
        canonical_spd_covariance(value)
        for value in raw_covariances
    )
    if any(value is None for value in canonical_covariances):
        return _rejected_attempt(
            "two_range_reference_covariance_invalid"
        )
    covariances = np.asarray(canonical_covariances, dtype=float)
    keys = tuple(
        (str(key[0]), int(key[1]))
        if (
            isinstance(key, (list, tuple))
            and len(key) == 2
            and not isinstance(key[1], bool)
            and isinstance(key[1], Integral)
        )
        else None
        for key in raw_keys
    )
    if (
        any(key is None for key in keys)
        or any(
            key[0] != "uav"
            or key[1] >= robot_id
            or key[1] < 1
            for key in keys
        )
        or len(set(keys)) != 2
    ):
        return _rejected_attempt("two_range_reference_keys_invalid")
    order = tuple(sorted(range(2), key=lambda index: keys[index][1]))
    positions = positions[list(order)]
    ranges = ranges[list(order)]
    covariances = covariances[list(order)]
    keys = tuple(keys[index] for index in order)
    prior = canonical_private_state(private_prior)
    if prior is None:
        return _rejected_attempt("two_range_private_prior_invalid")
    if not isinstance(base_anchor_provenance, (list, tuple)):
        return _rejected_attempt(
            "two_range_base_anchor_provenance_invalid"
        )
    if any(
        isinstance(root, bool)
        or not isinstance(root, Integral)
        or root < 0
        for root in base_anchor_provenance
    ):
        return _rejected_attempt(
            "two_range_base_anchor_provenance_invalid"
        )
    provenance = sorted({int(root) for root in base_anchor_provenance})
    if len(provenance) < 2:
        return _rejected_attempt(
            "two_range_base_anchor_provenance_invalid"
        )
    starts = two_circle_candidates(
        positions[0], ranges[0], positions[1], ranges[1]
    )
    if len(starts) != 2:
        return _rejected_attempt("two_range_circle_geometry_invalid")
    if np.linalg.norm(starts[0] - starts[1]) <= CANDIDATE_DEDUP_M:
        return _rejected_attempt(
            "two_range_circle_starts_not_distinct"
        )

    branches = []
    for branch_id, start in zip(BRANCH_IDS, starts, strict=True):
        result = solve_finite_budget_wnls(
            positions,
            covariances,
            ranges,
            start,
            ranging_sigma,
        )
        branches.append({
            "branch_id": branch_id,
            "circle_start": np.asarray(start, dtype=float).tolist(),
            "solver_result": result,
            "q_branch": None,
            "passes_branch_gate": None,
        })
```

Only after both results are complete,
converged,
SPD,
and mutually separated by more than `1e-9 m`,
compute each \(q_b\) with `normalized_innovation`.
Set `passes_branch_gate` from `q_b <= 11.829007011943707`.
Require exactly one passing branch.
Then call `candidate_acceptance` only for the selected branch with
`allow_two_reference_reacquisition=True`.
Implement that sequence exactly as:

```python
branches_valid, branch_reason = validate_solver_branches(branches)
if not branches_valid:
    return _rejected_attempt(
        branch_reason,
        branches=branches,
    )
innovations = [
    normalized_innovation(
        branch["solver_result"]["estimate"],
        branch["solver_result"]["covariance"],
        prior,
    )
    for branch in branches
]
if any(not innovation["valid"] for innovation in innovations):
    reason = next(
        str(innovation["failure_reason"])
        for innovation in innovations
        if not innovation["valid"]
    )
    return _rejected_attempt(
        reason,
        branches=branches,
    )
for branch, innovation in zip(branches, innovations, strict=True):
    branch["q_branch"] = float(innovation["q_innov"])
    branch["passes_branch_gate"] = branch_gate_passes(
        branch["q_branch"]
    )
passing = [
    branch for branch in branches
    if branch["passes_branch_gate"] is True
]
if len(passing) != 1:
    return _rejected_attempt(
        (
            "two_range_no_branch_passes"
            if not passing
            else "two_range_multiple_branches_pass"
        ),
        branches=branches,
        prior_used=True,
    )
selected_branch = passing[0]
accepted, reason, gate = candidate_acceptance(
    selected_branch["solver_result"],
    live_prediction=None,
    active_reference_count=2,
    base_anchor_provenance=provenance,
    allow_two_reference_reacquisition=True,
)
if not accepted:
    return _rejected_attempt(
        reason,
        branches=branches,
        prior_used=True,
        selected_branch=selected_branch,
    )
```

If that gate accepts, return this exact publication binding:

```python
selected_result = selected_branch["solver_result"]
candidate = {
    "estimate": np.asarray(
        selected_result["estimate"], dtype=float
    ).tolist(),
    "modeled_covariance": np.asarray(
        selected_result["covariance"], dtype=float
    ).tolist(),
    "epsilon": float(selected_result["epsilon"]),
    "base_anchor_provenance": provenance,
}
return {
    "attempt_status": "accepted",
    "status": "accepted",
    "failure_reason": None,
    "branches": branches,
    "selected_branch_id": selected_branch["branch_id"],
    "selected_candidate": selected_branch,
    "candidate": candidate,
    "prior_used_for_branch_selection": True,
    "prior_used_in_fim": False,
    "prior_used_for_continuous_update": False,
}
```

- [ ] **Step 7: Close fail-closed branch cases vertically**

Implement any still-missing fail-closed path until the tests written in
Step 5 cover all exact outcomes:

- zero passing branches;
- two passing branches;
- tangent circles;
- disjoint circles;
- contained circles;
- coincident centers;
- zero range;
- nearly collinear FIM;
- one solver failure;
- one non-SPD FIM;
- invalid private covariance;
- non-finite \(q_b\);
- failed solve of \(P_b+P^-\);
- original circle starts separated but both WNLS results merge;
- swapped reference input order;
- same-index and future-index UAV keys;
- private prior near each mirror branch.

The swapped-order case must produce the same canonical branch IDs,
selected estimate,
covariance,
epsilon,
and provenance as the sorted input,
not merely avoid an exception.
Also pin unchanged acceptance behavior for live public prediction and
three-or-more-reference reacquisition with the new option omitted.
For a mocked invalid/non-finite innovation on either branch,
assert both branches retain
`q_branch=None`,
both retain
`passes_branch_gate=None`,
and
`prior_used_for_branch_selection is False`.
For zero or multiple passing branches after two valid scores,
assert the flag is `True`.

For merged results:

```python
with mock.patch.object(
    two_range,
    "solve_finite_budget_wnls",
    side_effect=[same_result, same_result],
):
    attempt = solve_two_range_reacquisition(
        **valid_two_range_kwargs()
    )
self.assertEqual(
    attempt["failure_reason"],
    "two_range_solver_branches_merged",
)
```

- [ ] **Step 8: Close private-isolation cases vertically**

Use the tests already written in Step 5 to patch
`solve_finite_budget_wnls`,
capture its fourth positional argument,
and assert the two starts equal the raw circle intersections and differ from
the private prior.
Also assert no branch/source name contains
`private`,
`algebraic`,
or `prediction`.

- [ ] **Step 9: Rerun the frozen threshold boundary tests**

Run the `branch_gate_passes` tests first in isolation,
then the full selector class.
Expected:
the exact threshold and `nextafter` cases written in Step 5 all pass;
no constant has been adjusted after observing a solver outcome.

- [ ] **Step 10: Run estimator-focused regression**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_two_range_reacquisition \
  tests.test_predictive_wnls.CandidateGeometryTests \
  tests.test_predictive_wnls.FiniteBudgetWnlsTests \
  tests.test_predictive_wnls.CandidateAcceptanceTests \
  tests.test_predictive_wnls.PredictiveMultistartBoundaryTests -v
```

Expected: PASS with the legacy default still rejecting all two-reference
reacquisition attempts.

- [ ] **Step 11: Commit the selector**

```bash
git add \
  scripts/diagnostics/predictive_wnls.py \
  scripts/diagnostics/two_range_reacquisition.py \
  tests/test_predictive_wnls.py \
  tests/test_two_range_reacquisition.py
git commit -m "feat(diagnostics): add two-range branch selector"
```

### Task 3: Approved mechanism fixture and one-method raw producer

**Files:**
- Create: `scripts/diagnostics/extract_two_range_reacquisition_fixture.py`
- Create: `scripts/diagnostics/replay_two_range_reacquisition.py`
- Create: `tests/test_extract_two_range_reacquisition_fixture.py`
- Create: `tests/test_replay_two_range_reacquisition.py`
- Create: `tests/fixtures/cbf2026_two_range_reacquisition/mechanism_20260727_180_12.json`
- Create: `tests/fixtures/cbf2026_two_range_reacquisition/manifest.json`
- Test: `tests/test_replay_predictive_wnls_recovery.py`

**Interfaces:**
- Consumes:
  - Task 1 lifecycle and Task 2 selector.
  - Preserved v4 replay root
    `/private/tmp/cbf2026-predictive-wnls-development/stage1-v4`.
  - Preserved v4 raw manifest/process hashes from the approved design.
  - Preserved truth and baseline paths bound in the v4 manifest.
  - Read-only helpers from
    `scripts.diagnostics.replay_predictive_wnls_recovery` for finite checks,
    no-follow file identity,
    fixed-reference/range generation,
    public-reference arrays,
    reference evidence,
    output transactions,
    disk checks,
    and terminal publication.
    Do not reuse its permissive native-JSON conversion:
    the new row/object builders must explicitly canonicalize NumPy arrays to
    lists and NumPy scalar values to Python `int`/`float` values before the
    strict writer boundary,
    while the strict writer itself rejects every non-native scalar,
    array,
    set,
    or generator.
    Do not reuse its `_strict_json_bytes` writer,
    because that writer sorts keys alphabetically and is incompatible with
    the new declared non-alphabetical schema order.
- Produces:
  - `METHODS = (METHOD_ID,)`
  - `RAW_SCHEMA_ID = "cbf2026-two-range-reacquisition-raw-v1"`
  - `RAW_PROCESS_NAME = "two-range-reacquisition.jsonl.gz"`
  - `PRODUCER_INVOCATIONS = ("unit_fixture", "smoke_a", "smoke_b", "registered_replay")`
  - `ROW_INVOCATION_NAMES = ("unit_fixture", "smoke_validation", "registered_replay")`
  - `SMOKE_CASE_IDS` as the exact ordered tuple frozen below and imported
    unchanged by the registrar.
  - `SYNTHETIC_CASE_FIELDS`, `COMMON_SELECTOR_INPUT`,
    and the exact ordered `SYNTHETIC_CASES`.
  - `ROW_FIELDS`, `BRANCH_FIELDS`, `PRIVATE_PRIOR_FIELDS`,
    `NEXT_PRIVATE_STATE_FIELDS`,
    `RAW_MANIFEST_FIELDS`,
    `FILE_IDENTITY_FIELDS`,
    `PROCESS_IDENTITY_FIELDS`,
    and `RAW_KEY_CONTRACT_FIELDS`.
  - `ordered_strict_json_bytes(value: Mapping, fields: tuple[str, ...]) -> bytes`
  - `extract_mechanism_fixture(*, v4_root: Path, output: Path) -> Path`
  - `selector_consideration(*, robot_id: int, live_prediction: object, mandatory: Mapping, optional_keys: object, qualification: Mapping) -> tuple[bool, str]`
  - `produce_method_row(*, seed: int, frame_index: int, robot_id: int, config: Mapping, truth_positions: Mapping[int, np.ndarray], current_public: dict[int, dict], previous_state: Mapping | None, applied_command: object, ranging_sigma: float) -> tuple[dict, dict]`
  - `produce_smoke_row(*, case_id: str, mechanism_fixture: Mapping) -> dict`
  - `replay_two_range_reacquisition(*, protocol_path: Path, data_path: Path, input_manifest_path: Path, output_root: Path, run_seeds: tuple[int, ...], max_frames: int, invocation_name: str, authorization_json: Path | None = None) -> Path`
  - CLI `main(argv: list[str] | None = None) -> int`.

- [ ] **Step 1: Write failing fixture-identity tests**

Pin these exact preserved inputs:

```python
V4_ROOT = Path(
    "/private/tmp/cbf2026-predictive-wnls-development/stage1-v4"
)
V4_MANIFEST_SHA256 = (
    "123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223"
)
V4_COMPRESSED_SHA256 = (
    "9710d1a5792e682bb272d0bb3305162180d1e0b9a0636647d6e071d1782a699b"
)
V4_DECOMPRESSED_SHA256 = (
    "7047989804fd03bd4cadffac8212349b7183857623ac201d14ef2fc3e30ee2e1"
)
MECHANISM_KEY = (20260727, 180, 12)
```

Bootstrap with only the exact v4-manifest path/SHA test.
After Step 2 runs it red and Step 3 supplies the minimal no-follow
identity reader,
add each remaining path,
size,
SHA,
variant,
key,
reference,
and source-order mismatch as its own
red/implementation/green microcycle.

- [ ] **Step 2: Run the extractor test and verify it fails**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_extract_two_range_reacquisition_fixture.FixtureIdentityTests.test_v4_manifest_identity -v
```

Expected: FAIL because the extractor and fixture do not exist.

- [ ] **Step 3: Implement read-only deterministic fixture extraction**

First implement only the no-follow v4-manifest identity check required by
the bootstrap and rerun it green.
Then add each remaining extractor behavior below only after its matching
focused test has failed,
and rerun that test green before proceeding.
The complete extractor must:

1. no-follow open and hash the v4 manifest and compressed stream;
2. recompute the decompressed stream hash while reading;
3. select only `predictive_multistart` rows needed to reconstruct
   seed `20260727`,
   frame `180`,
   robot `12`,
   its lower-index references,
   and robot 12's preceding private-state source;
4. locate robot 12's last preceding `fresh` publication,
   set it as `source_fresh_frame`,
   replay only the recorded held commands and \(Q=0.25I\) transitions through
   frame 180,
   and require its estimate/covariance projection to match the v4 incoming
   private seed before attaching
   `propagated_to_frame=180` and the exact age;
5. bind the truth-data identity from the v4 manifest;
6. emit canonical strict JSON with no offline favorable-result expectation.

The fixture top-level fields are exactly:

```python
FIXTURE_FIELDS = (
    "schema_id",
    "fixture_id",
    "source_identities",
    "key",
    "mandatory_references",
    "optional_references",
    "current_reference_outputs",
    "measurements",
    "preceding_public_output",
    "preceding_private_state",
    "held_command",
    "expected_mechanism",
)
```

`expected_mechanism` may assert only:

```python
{
    "active_reference_count": 2,
    "active_reference_keys": [["uav", 10], ["uav", 11]],
    "old_failure_reason": "reacquisition_requires_three_active_references",
    "old_candidate_count": 3,
    "old_all_candidates_converged": True,
}
```

It must not assert the new method's error,
availability,
gate result,
or selected branch.

- [ ] **Step 4: Materialize and hash the fixture**

Run once:

```bash
conda run -n cbf_env python \
  scripts/diagnostics/extract_two_range_reacquisition_fixture.py \
  --v4-root /private/tmp/cbf2026-predictive-wnls-development/stage1-v4 \
  --output tests/fixtures/cbf2026_two_range_reacquisition/mechanism_20260727_180_12.json
```

Then write `manifest.json` with exact fixture byte size,
fixture SHA-256,
all preserved source SHA-256 values,
the extractor source SHA-256,
and the approved design commit.
Run the extractor a second time to a temporary path only for deterministic
byte comparison,
then discard that temporary copy without running the new estimator.

- [ ] **Step 5: Write failing exact raw-schema tests**

Pin this exact ordered `ROW_FIELDS` tuple:

```python
SMOKE_CASE_IDS = (
    "mechanism_20260727_180_12",
    "select_negative",
    "select_positive",
    "q_equal_threshold",
    "q_below_threshold",
    "q_above_threshold",
    "none_pass",
    "multiple_pass",
    "tangent",
    "disjoint",
    "contained",
    "coincident",
    "zero_range",
    "nearly_collinear",
    "merged_solver_branches",
    "invalid_private_covariance",
    "cost_equal_nine",
    "cost_above_nine",
)
SYNTHETIC_CASE_FIELDS = (
    "case_id",
    "case_kind",
    "input",
    "expected",
)
COMMON_SELECTOR_INPUT = {
    "selector_base_id": "symmetric_two_range_v1",
    "robot_id": 12,
    "reference_positions": [[0.0, 0.0], [2.0, 0.0]],
    "reference_covariances": [
        [[0.1, 0.0], [0.0, 0.1]],
        [[0.1, 0.0], [0.0, 0.1]],
    ],
    "measurements": [math.sqrt(2.0), math.sqrt(2.0)],
    "reference_keys": [["uav", 10], ["uav", 11]],
    "private_prior_estimate": [1.0, 1.75],
    "private_prior_covariance": [[0.2, 0.0], [0.0, 0.2]],
    "private_prior_source_fresh_frame": 20,
    "private_prior_propagated_to_frame": 20,
    "private_prior_age_frames": 0,
    "ranging_sigma": 0.5,
    "base_anchor_provenance": [0, 1],
}
CANDIDATE_TEMPLATES = {
    "canonical_spd_zero_residual_v1": {
        "status": "converged",
        "estimate": [1.0, 1.0],
        "covariance": [[1.0, 0.0], [0.0, 1.0]],
        "epsilon": 3.0,
        "phi_min_eigenvalue": 1.0,
        "phi_condition": 1.0,
        "fim_valid": True,
        "proposal_count": 0,
        "iterations": 0,
        "cost": 0.0,
        "stationarity_norm": 0.0,
        "failure_reason": None,
        "proposal_trace": [],
    },
}
SMOKE_SOLVER_OVERRIDES = {
    "both_positive_common_solution_v1": (
        {
            "status": "converged",
            "estimate": [1.0, 1.0000000000000002],
            "covariance": [
                [0.3500000000000001, 8.05917096752434e-19],
                [8.05917096752434e-19, 0.34999999999999987],
            ],
            "epsilon": 1.774823934929885,
            "phi_min_eigenvalue": 2.8571428571428563,
            "phi_condition": 1.0000000000000007,
            "fim_valid": True,
            "proposal_count": 0,
            "iterations": 0,
            "cost": 0.0,
            "stationarity_norm": 0.0,
            "failure_reason": None,
            "proposal_trace": [],
        },
        {
            "status": "converged",
            "estimate": [1.0, 1.0000000000000002],
            "covariance": [
                [0.3500000000000001, 8.05917096752434e-19],
                [8.05917096752434e-19, 0.34999999999999987],
            ],
            "epsilon": 1.774823934929885,
            "phi_min_eigenvalue": 2.8571428571428563,
            "phi_condition": 1.0000000000000007,
            "fim_valid": True,
            "proposal_count": 0,
            "iterations": 0,
            "cost": 0.0,
            "stationarity_norm": 0.0,
            "failure_reason": None,
            "proposal_trace": [],
        },
    ),
}
SMOKE_BRANCH_PAIR_RECORDS = {
    "both_positive_common_solution_v1": (
        {
            "branch_id": "circle_negative",
            "circle_start": [1.0, -1.0000000000000002],
            "solver_result": dict(
                SMOKE_SOLVER_OVERRIDES[
                    "both_positive_common_solution_v1"
                ][0]
            ),
            "q_branch": None,
            "passes_branch_gate": None,
        },
        {
            "branch_id": "circle_positive",
            "circle_start": [1.0, 1.0000000000000002],
            "solver_result": dict(
                SMOKE_SOLVER_OVERRIDES[
                    "both_positive_common_solution_v1"
                ][1]
            ),
            "q_branch": None,
            "passes_branch_gate": None,
        },
    ),
}

def frozen_candidate_gate_record(
    *,
    cost: float,
    accepted: bool,
    reason: str,
) -> dict:
    result = {
        **CANDIDATE_TEMPLATES["canonical_spd_zero_residual_v1"],
        "cost": cost,
    }
    diagnostics = {
        "innovation_gate": "not_applicable_reacquisition",
        "q_innov": None,
        "gate_outcome": "accepted" if accepted else "rejected",
        "valid": None,
        "failure_reason": None,
        "reduced_whitened_cost": cost,
    }
    return {
        "source": "circle_positive",
        "initial_estimate": [1.0, 1.0],
        "result": result,
        "accepted": accepted,
        "rejection_reason": None if accepted else reason,
        "gate_diagnostics": diagnostics,
        "status": result["status"],
        "estimate": result["estimate"],
        "covariance": result["covariance"],
        "cost": result["cost"],
        "q_innov": None,
    }

CANDIDATE_GATE_RECORDS = {
    "cost_equal_nine": frozen_candidate_gate_record(
        cost=9.0,
        accepted=True,
        reason="accepted",
    ),
    "cost_above_nine": frozen_candidate_gate_record(
        cost=np.nextafter(9.0, np.inf).item(),
        accepted=False,
        reason="reacquisition_reduced_cost_exceeds_nine",
    ),
}
SYNTHETIC_CASES = (
    {
        "case_id": "select_negative",
        "case_kind": "selector",
        "input": {
            "selector_base_id": "symmetric_two_range_v1",
            "overrides": {"private_prior_estimate": [1.0, -1.75]},
            "solver_override_id": None,
        },
        "expected": {
            "attempt_status": "accepted",
            "failure_reason": None,
            "selected_branch_id": "circle_negative",
            "score_outcome": "exactly_one",
            "prior_used_for_branch_selection": True,
        },
    },
    {
        "case_id": "select_positive",
        "case_kind": "selector",
        "input": {
            "selector_base_id": "symmetric_two_range_v1",
            "overrides": {},
            "solver_override_id": None,
        },
        "expected": {
            "attempt_status": "accepted",
            "failure_reason": None,
            "selected_branch_id": "circle_positive",
            "score_outcome": "exactly_one",
            "prior_used_for_branch_selection": True,
        },
    },
    *(
        {
            "case_id": case_id,
            "case_kind": "branch_gate",
            "input": {"q_source": q_source},
            "expected": {"passes": passes},
        }
        for case_id, q_source, passes in (
            ("q_equal_threshold", "threshold", True),
            ("q_below_threshold", "nextafter_down", True),
            ("q_above_threshold", "nextafter_up", False),
        )
    ),
    {
        "case_id": "none_pass",
        "case_kind": "selector",
        "input": {
            "selector_base_id": "symmetric_two_range_v1",
            "overrides": {"private_prior_estimate": [10.0, 0.0]},
            "solver_override_id": None,
        },
        "expected": {
            "attempt_status": "rejected",
            "failure_reason": "two_range_no_branch_passes",
            "selected_branch_id": None,
            "score_outcome": "none",
            "prior_used_for_branch_selection": True,
        },
    },
    {
        "case_id": "multiple_pass",
        "case_kind": "selector",
        "input": {
            "selector_base_id": "symmetric_two_range_v1",
            "overrides": {"private_prior_estimate": [1.0, 0.0]},
            "solver_override_id": None,
        },
        "expected": {
            "attempt_status": "rejected",
            "failure_reason": "two_range_multiple_branches_pass",
            "selected_branch_id": None,
            "score_outcome": "multiple",
            "prior_used_for_branch_selection": True,
        },
    },
    *(
        {
            "case_id": case_id,
            "case_kind": "selector",
            "input": {
                "selector_base_id": "symmetric_two_range_v1",
                "overrides": overrides,
                "solver_override_id": None,
            },
            "expected": {
                "attempt_status": "rejected",
                "failure_reason": failure_reason,
                "selected_branch_id": None,
                "score_outcome": "not_evaluated",
                "prior_used_for_branch_selection": False,
            },
        }
        for case_id, overrides, failure_reason in (
            (
                "tangent",
                {"measurements": [1.0, 1.0]},
                "two_range_circle_starts_not_distinct",
            ),
            (
                "disjoint",
                {"measurements": [0.5, 0.5]},
                "two_range_circle_geometry_invalid",
            ),
            (
                "contained",
                {"measurements": [5.0, 1.0]},
                "two_range_circle_geometry_invalid",
            ),
            (
                "coincident",
                {
                    "reference_positions": [
                        [0.0, 0.0],
                        [0.0, 0.0],
                    ],
                    "measurements": [1.0, 1.0],
                },
                "two_range_circle_geometry_invalid",
            ),
            (
                "zero_range",
                {"measurements": [0.0, 2.0]},
                "two_range_input_invalid",
            ),
            (
                "nearly_collinear",
                {
                    "measurements": [
                        math.hypot(1.0, 1e-7),
                        math.hypot(1.0, 1e-7),
                    ],
                },
                "two_range_branch_solver_invalid",
            ),
            (
                "invalid_private_covariance",
                {
                    "private_prior_covariance": [
                        [1.0, 2.0],
                        [2.0, 1.0],
                    ],
                },
                "two_range_private_prior_invalid",
            ),
        )
    ),
    {
        "case_id": "merged_solver_branches",
        "case_kind": "branch_pair_gate",
        "input": {
            "solver_override_id": "both_positive_common_solution_v1",
        },
        "expected": {
            "valid": False,
            "reason": "two_range_solver_branches_merged",
        },
    },
    *(
        {
            "case_id": case_id,
            "case_kind": "candidate_gate",
            "input": {
                "candidate_template_id": (
                    "canonical_spd_zero_residual_v1"
                ),
                "cost_source": cost_source,
            },
            "expected": {
                "accepted": accepted,
                "reason": reason,
            },
        }
        for case_id, cost_source, accepted, reason in (
            ("cost_equal_nine", "nine", True, "accepted"),
            (
                "cost_above_nine",
                "nextafter_nine_up",
                False,
                "reacquisition_reduced_cost_exceeds_nine",
            ),
        )
    ),
)
SELECTOR_SMOKE_INPUT_FIELDS = (
    "selector_base_id",
    "overrides",
    "solver_override_id",
)
SELECTOR_SMOKE_EXPECTED_FIELDS = (
    "attempt_status",
    "failure_reason",
    "selected_branch_id",
    "score_outcome",
    "prior_used_for_branch_selection",
)
BRANCH_GATE_SMOKE_INPUT_FIELDS = ("q_source",)
BRANCH_GATE_SMOKE_EXPECTED_FIELDS = ("passes",)
BRANCH_PAIR_GATE_SMOKE_INPUT_FIELDS = ("solver_override_id",)
BRANCH_PAIR_GATE_SMOKE_EXPECTED_FIELDS = ("valid", "reason")
CANDIDATE_GATE_SMOKE_INPUT_FIELDS = (
    "candidate_template_id",
    "cost_source",
)
CANDIDATE_GATE_SMOKE_EXPECTED_FIELDS = ("accepted", "reason")
MECHANISM_SMOKE_INPUT_FIELDS = (
    "fixture_id",
    "fixture_sha256",
)
MECHANISM_SMOKE_EXPECTED_FIELDS = (
    "selector_considered",
    "active_reference_count",
    "active_reference_keys",
    "old_failure_reason",
    "two_branches_required",
    "favorable_new_outcome_required",
)
MECHANISM_SMOKE_EXPECTED = {
    "selector_considered": True,
    "active_reference_count": 2,
    "active_reference_keys": [["uav", 10], ["uav", 11]],
    "old_failure_reason": (
        "reacquisition_requires_three_active_references"
    ),
    "two_branches_required": True,
    "favorable_new_outcome_required": False,
}
ROW_FIELDS = (
    "method",
    "invocation_name",
    "smoke_case_id",
    "smoke_case_kind",
    "smoke_case_input",
    "smoke_case_expected",
    "seed",
    "frame_index",
    "robot_id",
    "squad_local_index",
    "applied_command_source_frame",
    "applied_command",
    "attempt_path",
    "selector_considered",
    "selector_consideration_reason",
    "attempt_status",
    "attempt_failure_reason",
    "output_status",
    "prediction_age",
    "estimate",
    "fresh_modeled_covariance",
    "fresh_epsilon",
    "aged_modeled_covariance",
    "aged_modeled_radius",
    "branch_selection_prior_status",
    "branch_selection_prior_estimate",
    "branch_selection_prior_covariance",
    "branch_selection_prior_source_fresh_frame",
    "branch_selection_prior_propagated_to_frame",
    "branch_selection_prior_age_frames",
    "prior_used_for_branch_selection",
    "prior_used_in_fim",
    "prior_used_for_continuous_update",
    "branches",
    "selected_branch_id",
    "next_private_state_status",
    "next_private_state_estimate",
    "next_private_state_covariance",
    "next_private_state_source_fresh_frame",
    "next_private_state_propagated_to_frame",
    "next_private_state_age_frames",
    "existing_candidates",
    "existing_selected_candidate_source",
    "attempt_base_anchor_provenance",
    "base_anchor_provenance",
    "mandatory_references",
    "optional_candidates",
    "active_references",
    "reference_evidence",
    "reference_freshness",
    "excluded_references",
    "reference_violations",
    "offline_truth_position",
    "offline_error_norm",
    "offline_fresh_containment",
    "offline_aged_radius_containment",
    "offline_fresh_q_error",
    "offline_aged_q_error",
)
```

At module import,
assert that the 17 `SYNTHETIC_CASES` IDs are exactly
`SMOKE_CASE_IDS[1:]`,
that every object has exactly `SYNTHETIC_CASE_FIELDS`,
that each kind has exactly its declared input/expected fields,
and that every referenced base/template/override ID exists once.
Create one canonical strict-JSON declaration containing
`COMMON_SELECTOR_INPUT`,
`CANDIDATE_TEMPLATES`,
`SMOKE_SOLVER_OVERRIDES`,
`SMOKE_BRANCH_PAIR_RECORDS`,
`CANDIDATE_GATE_RECORDS`,
and
`SYNTHETIC_CASES`;
the registrar embeds that declaration,
its source-file identity,
byte size,
and SHA-256 in the protocol.
The raw manifest repeats the declaration SHA-256.
`produce_smoke_row` accepts no caller-supplied synthetic payload:
it looks up only by `case_id`,
and writes input/expected objects byte-for-byte equal to the
protocol-bound declaration.
The mechanism input is exactly the fixture ID and fixture SHA-256 from the
fixture manifest,
with `MECHANISM_SMOKE_EXPECTED`.

The `branch_pair_gate` case invokes only
`validate_solver_branches` on deep copies of the frozen override records.
The `candidate_gate` cases deep-copy the frozen candidate template and
replace only its `cost` from the declared source
(`9.0` or `nextafter(9.0, +inf)`).
The `branch_gate` cases derive their value only from the frozen threshold
and declared `nextafter` direction.
Solver overrides,
candidate templates,
and any non-null smoke-case field are forbidden in
`registered_replay`.
The smoke analyzer does not import producer decisions:
it reads the protocol declaration,
reconstructs the appropriate pure-function input,
and requires exact row equality before recomputing the outcome.

`BRANCH_FIELDS` must be:

```python
(
    "branch_id",
    "circle_start",
    "solver_result",
    "q_branch",
    "passes_branch_gate",
)
```

Copy these literal nested orders into the new raw schema;
do not serialize only their constant names:

```python
SOLVER_RESULT_FIELDS = (
    "status",
    "estimate",
    "covariance",
    "epsilon",
    "phi_min_eigenvalue",
    "phi_condition",
    "fim_valid",
    "proposal_count",
    "iterations",
    "cost",
    "stationarity_norm",
    "failure_reason",
    "proposal_trace",
)
PROPOSAL_TRACE_FIELDS = (
    "proposal",
    "damping",
    "cost",
    "stationarity_norm",
    "raw_step_norm",
    "trial_cost",
    "invalid_trial_reason",
    "accepted",
)
EXISTING_CANDIDATE_FIELDS = (
    "source",
    "initial_estimate",
    "result",
    "accepted",
    "rejection_reason",
    "gate_diagnostics",
    "status",
    "estimate",
    "covariance",
    "cost",
    "q_innov",
)
GATE_DIAGNOSTIC_FIELDS = (
    "innovation_gate",
    "q_innov",
    "gate_outcome",
    "valid",
    "failure_reason",
    "reduced_whitened_cost",
)
REFERENCE_EVIDENCE_FIELDS = (
    "reference_kind",
    "reference_id",
    "role",
    "measurement_present",
    "noisy_range",
    "noise_seed",
    "current_freshness",
    "eligible",
    "used",
    "exclusion_reason",
    "base_anchor_provenance",
)
REFERENCE_FRESHNESS_FIELDS = (
    "reference_kind",
    "reference_id",
    "current_freshness",
)
MANDATORY_REFERENCE_FIELDS = ("base_ids", "uav_ids")
REFERENCE_KEY_FIELDS = ("reference_kind", "reference_id")
EXCLUSION_FIELDS = ("reference_kind", "reference_id", "reason")
VIOLATION_FIELDS = ("reference_kind", "reference_id", "reason")
```

Pin the nested state-field declarations as:

```python
PRIVATE_PRIOR_FIELDS = (
    "branch_selection_prior_status",
    "branch_selection_prior_estimate",
    "branch_selection_prior_covariance",
    "branch_selection_prior_source_fresh_frame",
    "branch_selection_prior_propagated_to_frame",
    "branch_selection_prior_age_frames",
)
NEXT_PRIVATE_STATE_FIELDS = (
    "next_private_state_status",
    "next_private_state_estimate",
    "next_private_state_covariance",
    "next_private_state_source_fresh_frame",
    "next_private_state_propagated_to_frame",
    "next_private_state_age_frames",
)
```

Freeze this schema DSL:

```python
ROW_SCALAR_CONTRACTS = {
    "method": f"literal:{METHOD_ID}",
    "invocation_name": "enum:ROW_INVOCATION_NAMES",
    "smoke_case_id": "nullable:enum:SMOKE_CASE_IDS",
    "smoke_case_kind": (
        "nullable:enum:mechanism_fixture,selector,"
        "branch_gate,branch_pair_gate,candidate_gate"
    ),
    "seed": "nullable:int:20260727:20260746",
    "frame_index": "nullable:int:0:499",
    "robot_id": "nullable:int:1:14",
    "squad_local_index": "nullable:int:1:7",
    "applied_command_source_frame": "nullable:int:0:498",
    "attempt_path": (
        "enum:reference_unavailable,"
        "existing_predictive_multistart,two_range_reacquisition,"
        "smoke_branch_gate,smoke_branch_pair_gate,"
        "smoke_candidate_gate"
    ),
    "selector_considered": "bool",
    "selector_consideration_reason": (
        "enum:CONSIDERATION_REASONS|smoke_component_case"
    ),
    "attempt_status": "enum:ATTEMPT_STATUSES",
    "attempt_failure_reason": "nullable:str",
    "output_status": "enum:OUTPUT_STATUSES",
    "prediction_age": "nullable:int:0:2",
    "fresh_epsilon": "nullable:finite_nonnegative",
    "aged_modeled_radius": "nullable:finite_nonnegative",
    "branch_selection_prior_status": "enum:available,absent",
    "branch_selection_prior_source_fresh_frame": (
        "nullable:int:0:499"
    ),
    "branch_selection_prior_propagated_to_frame": (
        "nullable:int:0:499"
    ),
    "branch_selection_prior_age_frames": "nullable:int:0:499",
    "prior_used_for_branch_selection": "bool",
    "prior_used_in_fim": "literal:false",
    "prior_used_for_continuous_update": "literal:false",
    "selected_branch_id": "nullable:enum:BRANCH_IDS",
    "next_private_state_status": "enum:available,absent",
    "next_private_state_source_fresh_frame": "nullable:int:0:499",
    "next_private_state_propagated_to_frame": "nullable:int:0:499",
    "next_private_state_age_frames": "nullable:int:0:499",
    "existing_selected_candidate_source": "nullable:str",
    "offline_error_norm": "nullable:finite_nonnegative",
    "offline_fresh_containment": "nullable:bool",
    "offline_aged_radius_containment": "nullable:bool",
    "offline_fresh_q_error": "nullable:finite_nonnegative",
    "offline_aged_q_error": "nullable:finite_nonnegative",
}
ROW_ARRAY_CONTRACTS = {
    "smoke_case_input": (
        "nullable:exact_case_object:"
        "SELECTOR_SMOKE_INPUT_FIELDS|BRANCH_GATE_SMOKE_INPUT_FIELDS|"
        "BRANCH_PAIR_GATE_SMOKE_INPUT_FIELDS|"
        "CANDIDATE_GATE_SMOKE_INPUT_FIELDS|MECHANISM_SMOKE_INPUT_FIELDS"
    ),
    "smoke_case_expected": (
        "nullable:exact_case_object:"
        "SELECTOR_SMOKE_EXPECTED_FIELDS|"
        "BRANCH_GATE_SMOKE_EXPECTED_FIELDS|"
        "BRANCH_PAIR_GATE_SMOKE_EXPECTED_FIELDS|"
        "CANDIDATE_GATE_SMOKE_EXPECTED_FIELDS|"
        "MECHANISM_SMOKE_EXPECTED_FIELDS"
    ),
    "applied_command": "nullable:finite_vec2",
    "estimate": "nullable:finite_vec2",
    "fresh_modeled_covariance": "nullable:finite_spd_2x2",
    "aged_modeled_covariance": "nullable:finite_spd_2x2",
    "branch_selection_prior_estimate": "nullable:finite_vec2",
    "branch_selection_prior_covariance": "nullable:finite_spd_2x2",
    "branches": "list:strict_object:BRANCH_FIELDS",
    "next_private_state_estimate": "nullable:finite_vec2",
    "next_private_state_covariance": "nullable:finite_spd_2x2",
    "existing_candidates": (
        "list:strict_object:EXISTING_CANDIDATE_FIELDS"
    ),
    "attempt_base_anchor_provenance": (
        "sorted_unique_list:int_nonnegative"
    ),
    "base_anchor_provenance": "sorted_unique_list:int_nonnegative",
    "mandatory_references": (
        "strict_object:MANDATORY_REFERENCE_FIELDS"
    ),
    "optional_candidates": "list:strict_object:REFERENCE_KEY_FIELDS",
    "active_references": "list:strict_object:REFERENCE_KEY_FIELDS",
    "reference_evidence": (
        "list:strict_object:REFERENCE_EVIDENCE_FIELDS"
    ),
    "reference_freshness": (
        "list:strict_object:REFERENCE_FRESHNESS_FIELDS"
    ),
    "excluded_references": "list:strict_object:EXCLUSION_FIELDS",
    "reference_violations": "list:strict_object:VIOLATION_FIELDS",
    "offline_truth_position": "nullable:finite_vec2",
}
BRANCH_FIELD_CONTRACTS = {
    "branch_id": "enum:BRANCH_IDS",
    "circle_start": "finite_vec2",
    "solver_result": "strict_object:SOLVER_RESULT_FIELDS",
    "q_branch": "nullable:finite_nonnegative",
    "passes_branch_gate": "nullable:bool",
}
RAW_MANIFEST_FIELDS = (
    "schema_id",
    "protocol_id",
    "invocation_name",
    "status",
    "method",
    "output_root",
    "protocol_identity",
    "source_identities",
    "authorization_identity",
    "process_identity",
    "expected_rows",
    "observed_rows",
    "key_contract",
    "disk_contract",
    "started_at",
    "completed_at",
    "error",
)
FILE_IDENTITY_FIELDS = (
    "path",
    "device",
    "inode",
    "size",
    "mtime_ns",
    "sha256",
)
PROCESS_IDENTITY_FIELDS = (
    "path",
    "device",
    "inode",
    "size",
    "allocated_bytes",
    "mtime_ns",
    "compressed_sha256",
    "decompressed_sha256",
)
RAW_KEY_CONTRACT_FIELDS = (
    "method",
    "seeds",
    "frames",
    "robots",
    "smoke_case_ids",
    "order",
)
RAW_SOURCE_MEMBER_NAMES = {
    "unit_fixture": (
        "two_range_reacquisition_source",
        "predictive_wnls_source",
        "replay_source",
        "mechanism_fixture",
        "mechanism_fixture_manifest",
    ),
    "smoke_a": (
        "two_range_reacquisition_source",
        "predictive_wnls_source",
        "replay_source",
        "mechanism_fixture",
        "mechanism_fixture_manifest",
    ),
    "smoke_b": (
        "two_range_reacquisition_source",
        "predictive_wnls_source",
        "replay_source",
        "mechanism_fixture",
        "mechanism_fixture_manifest",
    ),
    "registered_replay": (
        "two_range_reacquisition_source",
        "predictive_wnls_source",
        "replay_source",
        "truth_data",
        "input_manifest",
    ),
}
```

Reuse the exact nested field orders from the preserved v4 producer for
solver results,
proposal traces,
existing multistart candidates,
mandatory references,
reference keys/evidence/freshness/exclusions/violations.
Reuse the mapping-producing numerical/evidence helpers,
but do not call v4's positional compact serializers
(`_compact_candidates`,
`_compact_reference_evidence`,
or their list-array equivalents).
The new producer converts each source mapping to a newly allocated ordered
object in the literal field order above and normally rejects a missing/extra
source key before serialization.
The sole source-side exception is preserved gate diagnostics:
the producer accepts only the preserved discriminated required subset,
rejects unknown keys,
and canonicalizes allowed-but-absent keys to `None` with
`.get(field)` before serializing all `GATE_DIAGNOSTIC_FIELDS`.
Tests must feed the old positional-list encoding to every new nested field
and require rejection.

Nested types are fixed as follows:

- solver results and proposal traces use the preserved strict validators;
  integer counts are non-Boolean,
  every present numeric value is finite,
  converged covariance is canonical SPD,
  and converged summary/trace fields agree exactly;
- existing candidate source/status are frozen enums,
  initial/terminal estimates are finite vec2 when present,
  covariance is canonical SPD when present,
  accepted/reason/gate fields follow the preserved discriminated null rules,
  and every duplicated summary equals its nested solver result;
- gate diagnostics always serialize all six ordered fields;
  `innovation_gate/q_innov/gate_outcome` are source-required,
  `valid/failure_reason` must appear together only after an innovation
  evaluation,
  and `reduced_whitened_cost` appears only after a no-live-prediction
  reacquisition cost evaluation;
  every other allowed-but-inapplicable field is null;
- reference kind is `base` or `uav`,
  ID/noise seed are non-Boolean non-negative integers when present,
  role is `mandatory` or `optional`,
  measurement/eligible/used are Booleans,
  noisy range is finite non-negative when present,
  freshness is `fresh/predicted/unavailable/missing`,
  exclusion/reason fields are nullable strings,
  and provenance is a sorted unique non-negative integer list;
- mandatory base/UAV ID arrays,
  reference-key arrays,
  exclusions,
  and violations are sorted in canonical base-before-UAV/numeric-ID order.

`BRANCH_FIELDS`,
`PRIVATE_PRIOR_FIELDS`,
and
`NEXT_PRIVATE_STATE_FIELDS`
are the new ordered additions.
At import time and in tests,
assert that the scalar and array contract keys are disjoint and their union
is exactly `set(ROW_FIELDS)`;
assert that `BRANCH_FIELD_CONTRACTS` keys are exactly
`set(BRANCH_FIELDS)`;
the raw schema JSON serializes the contracts in `ROW_FIELDS` order,
not Python set/dict iteration order.

The discriminated null rules are exact:

- registered rows require non-null seed/frame/robot/squad/truth,
  require `smoke_case_id=None`,
  require all three smoke-case fields null,
  and use command/source-frame null only at frame zero;
- smoke rows require a non-null `smoke_case_id`;
  require a non-null kind/input/expected object equal to the protocol-bound
  case record;
  only the real mechanism row may carry real production identifiers/truth,
  while synthetic rows require those identifier/truth/offline fields null;
- `fresh` requires prediction age zero,
  estimate/fresh covariance/fresh epsilon,
  and null aged fields;
- `predicted` requires age one or two,
  estimate/aged covariance/aged radius,
  and null fresh fields;
- `unavailable` requires all public numeric fields null;
- private status `available` requires every corresponding state field;
  `absent` requires all five corresponding numeric/index fields null;
- `prior_used_for_branch_selection=True` requires two non-null finite scores
  and two Boolean pass values;
  `False` requires both branch scores and pass values null;
- a selected branch ID is permitted only after exactly one branch passes;
  `candidate` publication is permitted only for accepted/fresh output.

Smoke component-row encoding is exact:

- `branch_gate`,
  `branch_pair_gate`,
  and
  `candidate_gate`
  use their corresponding `attempt_path`,
  set `selector_considered=False`,
  reason `smoke_component_case`,
  `attempt_status="invalid"`,
  `attempt_failure_reason=None`,
  and public output `unavailable` with every public numeric field null;
- all prior/next-private statuses are `absent`,
  all private fields null,
  and all three prior-use flags false;
- branch-gate rows use an empty branch list;
  branch-pair rows serialize the two frozen solver-result branches with null
  scores/pass values;
  candidate-gate rows serialize one strict existing-candidate object;
- selected IDs/sources are null,
  provenance/reference/evidence/exclusion/violation arrays are empty,
  mandatory base/UAV arrays are empty,
  and every offline field is null.

Selector synthetic rows use `two_range_reacquisition`,
the full common/override input,
and real selector/finalization semantics.
They retain null production keys/truth but use synthetic frame 20 for the
tagged prior and outgoing private-state indices.
Mechanism rows use the full real producer path and real key.

Production `attempt_path` is a strict discriminant:

- `two_range_reacquisition` requires
  `selector_considered=True`,
  `existing_candidates=[]`,
  and
  `existing_selected_candidate_source=None`;
  it permits only zero or two ordered branches,
  never one.
  Zero branches are required for rejection before both distinct circle
  starts have been solved
  (invalid robot/input/reference covariance/reference keys/private
  prior/provenance,
  invalid circle geometry,
  or non-distinct starts).
  Two branches are required once both WNLS calls were attempted,
  including a branch solver/FIM failure,
  merged results,
  innovation failure,
  zero/multiple score pass,
  selected cost/provenance rejection,
  and acceptance;
- `existing_predictive_multistart` requires
  `selector_considered=False`,
  `branches=[]`,
  `selected_branch_id=None`,
  `prior_used_for_branch_selection=False`,
  both other prior-use flags false,
  and only `existing_candidates` may be non-empty;
- `reference_unavailable` requires attempt status
  `reference_unavailable`,
  both candidate collections empty,
  both selected fields null,
  and all prior-use flags false;
- component smoke paths use only the component encoding above and are
  forbidden in registered rows.

No row may carry both two-range branches and existing multistart candidates.
Add one mutation test for every cross-path field substitution,
including a private/algebraic existing candidate inserted into a two-range
row,
a circle branch inserted into an existing row,
one-branch two-range evidence,
a zero-branch post-solver failure,
a two-branch pre-geometry failure,
and a selected branch attached to `reference_unavailable`.
Add gate-diagnostic tests for each preserved discriminant,
one legal partial source mapping canonicalized to six fields,
one missing required base key,
one half-present `valid/failure_reason` pair,
and one unknown key.

The raw manifest is also strict and ordered.
`schema_id`, `protocol_id`, `method`, and `output_root` are exact literals;
`status` is one of
`creating/running/completed/failed`;
source identities are a strict ordered object whose exact member names,
order,
and cardinality equal
`RAW_SOURCE_MEMBER_NAMES[invocation_name]`,
with every value a strict `FILE_IDENTITY_FIELDS` object;
protocol identity is null only for the unit fixture and otherwise a strict
file identity matching the protocol-bound source;
authorization identity is non-null only for registered replay;
process identity is non-null only after the raw descriptor has been
terminally hashed;
row counts are non-Boolean non-negative integers;
the key contract uses exact production ranges or the exact ordered
`SMOKE_CASE_IDS`,
never both;
disk contract is byte-for-byte equal to the protocol section;
timestamps are canonical UTC strings;
`completed_at` is null before terminal state;
`error` is null on `completed` and otherwise a strict ordered
`("type", "message")` object on `failed`.
Every transition rewrites the complete manifest transactionally;
partial/abbreviated terminal manifests are invalid.
Add missing/extra/reordered/substituted source-member and protocol-identity
mutation tests for every invocation.

Bootstrap `OrderedStrictJsonTests` with only one deliberately
non-alphabetical native mapping and require that its emitted keys retain
the supplied declared tuple order.
Run it red because `ordered_strict_json_bytes` does not exist,
implement only the native finite scalar/mapping/list path and declared
top-level order check,
then rerun it green.
After that bootstrap,
add each remaining requirement as its own red/implementation/green
microcycle:

- emitted object keys retain the supplied declared tuple order,
  including deliberately non-alphabetical orders;
- nested row/branch/solver/reference objects retain each declared order;
- raw manifest,
  analysis,
  and protocol top-level orders are not alphabetically sorted;
- missing/extra/misordered mappings,
  NaN,
  infinity,
  non-string object keys,
  and non-native NumPy scalars reject;
- tuple values canonicalize to JSON arrays,
  while sets and generators reject;
- UTF-8 is emitted directly,
  separators are exactly `(",", ":")`,
  writer output has no newline,
  and the raw-row caller appends exactly one `b"\n"`.

Every field is required.
Inapplicable values use the declared null/empty representation;
fields are never omitted.
For `registered_replay`,
`invocation_name` is exactly `"registered_replay"` and
`smoke_case_id` is `None`.
For `smoke_a` or `smoke_b`,
the manifest/CLI invocation retains its distinct A/B name,
but every row uses the common semantic
`invocation_name="smoke_validation"`
so the two process streams can be byte-identical.
`smoke_case_id` is the corresponding member of the protocol's exact ordered
18-case tuple;
the mechanism case retains its real seed/frame/robot,
while synthetic cases use `None` for those three production-key fields.

- [ ] **Step 6: Write failing routing tests**

Use mocks around `solve_two_range_reacquisition` and
`solve_predictive_multistart`.
Bootstrap with only the non-robot-12 positive routing case described below,
run it red,
implement the minimal canonical-fixed-reference considered path in Step 7,
and rerun it green.
Then add the mechanism-fixture positive case and every existing-path
fallback bullet one at a time;
for each,
run the focused test red,
add only the matching reason/guard,
and rerun it green.
For the robot currently being reconstructed,
derive `fixed_keys` from its canonical configuration and public current
frame,
rather than from fixture constants.
Assert the new selector is called only for:

```python
(
    live_prediction is None
    and len(fixed_keys) == 2
    and all(key[0] == "uav" for key in fixed_keys)
    and mandatory_keys == fixed_keys
    and active_keys == fixed_keys
    and optional_active_keys == []
    and all(key[1] < robot_index for key in fixed_keys)
    and all(reference_is_current_fresh(output) for output in fixed_outputs)
)
```

Freeze the mechanism fixture separately as
`robot_index=12` and
`fixed_keys=(("uav", 10), ("uav", 11))`.
Add at least one positive routing case for a different robot whose two
canonical fixed CBF references are not UAVs 10 and 11;
it must call the new selector.

Assert the existing path is used,
unchanged,
for:

- a live prediction;
- any active base;
- any optional reference;
- fewer than two or more than two active references;
- a predicted or unavailable UAV reference;
- a same/future-index UAV;
- missing fixed mandatory reference;
- invalid range;
- insufficient provenance.

- [ ] **Step 7: Implement structural consideration and method routing**

Use an exact reason enum:

```python
CONSIDERATION_REASONS = (
    "considered",
    "live_public_prediction",
    "qualification_not_ok",
    "active_reference_count_not_two",
    "active_reference_not_mandatory_uav",
    "active_base_present",
    "active_optional_present",
    "fixed_reference_missing",
    "reference_not_current_fresh",
    "reference_not_strictly_lower_index",
    "range_invalid",
    "provenance_invalid",
)
```

If considered,
call only `solve_two_range_reacquisition`.
Otherwise call the frozen complete `solve_predictive_multistart` path.
Pass only the estimate/covariance projection of the tagged private state to
that existing path.
Never pass the tagged private state to a continuous solver on the new path.

- [ ] **Step 8: Write failing serialization, ordering, and lifecycle tests**

Before implementing the serializer,
call the planned row builder only for the accepted record.
Require that row to flatten the incoming branch-selection prior and outgoing
private state into the exact fields.
The eventual matrix also covers
rejected-with-prior,
rejected-without-prior,
predicted,
unavailable,
mechanism-smoke,
synthetic-smoke,
and registered records,
but those tests are added one at a time only after the accepted-record
microcycle is green.
For an accepted test row:

```python
assert next_private_state["source_fresh_frame"] == frame_index
assert next_private_state["propagated_to_frame"] == frame_index
assert next_private_state["age_frames"] == 0
```

For a rejected row with a valid incoming prior,
assert exact canonical equality between incoming and outgoing state.
For absence,
set status to `"absent"` and all other corresponding fields to `None`.
The public output must never contain private-state fields.

After the first accepted-row serializer microcycle is green,
add and close the remaining row forms one at a time:
rejected-with-prior,
rejected-without-prior,
predicted,
unavailable,
mechanism-smoke,
synthetic-smoke,
and registered.
For each form,
write its smallest field/order/null-semantics test,
run it red,
implement only that form,
and rerun it green.

Then close each iterator/count case in its own microcycle:

- the two 18-case smoke invocations;
- a tiny two-seed/two-frame/three-robot hermetic grid;
- the declared 140000-key production iterator without executing the solver;
- duplicate,
  missing,
  extra,
  and out-of-order keys.

Before Step 9 implementation,
also write the two-root deterministic producer test:
run the same hermetic fixture/synthetic invocation into two temporary roots
and require equal decompressed and compressed process hashes,
while allowing timestamped manifest hashes to differ.
Run it red because the transactional producer does not yet exist.

Finally,
close each of the nine lifecycle failure-injection cases listed in Step 10
as a separate red/green microcycle while implementing the corresponding
transaction boundary.
Never batch all nine as one red suite before implementation.

- [ ] **Step 9: Implement exact serialization, ordering, and row count**

Implement the exact row/object conversions from Step 5 first:

```python
def _native_json_value(value: object) -> object:
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, int) and not isinstance(value, bool):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("JSON numbers must be finite")
        return value
    if isinstance(value, (list, tuple)):
        return [_native_json_value(item) for item in value]
    if isinstance(value, Mapping):
        if any(not isinstance(key, str) for key in value):
            raise ValueError("JSON object keys must be strings")
        return {
            key: _native_json_value(item)
            for key, item in value.items()
        }
    raise ValueError("value is not native strict JSON")

def ordered_strict_json_bytes(
    value: Mapping,
    fields: tuple[str, ...],
) -> bytes:
    if tuple(value) != fields:
        raise ValueError("object differs from declared field order")
    native = _native_json_value(value)
    return json.dumps(
        native,
        allow_nan=False,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=False,
    ).encode("utf-8")
```

Rerun `OrderedStrictJsonTests` to green.
The analyzer and registrar import this writer and validate their own field
tuples before calling it.
Initialize state once per seed and current public outputs once per frame.
The production loop is:

```python
for method in (METHOD_ID,):
    for seed in range(20260727, 20260747):
        method_state: dict[int, dict] = {}
        for frame_index in range(500):
            current_public: dict[int, dict] = {}
            for robot_id in range(1, 15):
                row, next_state = produce_method_row(
                    seed=seed,
                    frame_index=frame_index,
                    robot_id=robot_id,
                    config=config,
                    truth_positions=truth,
                    current_public=current_public,
                    previous_state=method_state.get(robot_id),
                    applied_command=(
                        None
                        if frame_index == 0
                        else commands[frame_index - 1][robot_id]
                    ),
                    ranging_sigma=ranging_sigma,
                )
                method_state[robot_id] = next_state
                current_public[robot_id] = next_state["public_output"]
                yield row
```

Reject duplicate,
missing,
extra,
or out-of-order keys.
The production manifest must declare and finish with exactly `140000` rows.
A hermetic test invocation may use only declared synthetic records and the
single mechanism fixture;
it must use a distinct test authorization token and can never be accepted as
production evidence.
`registered_replay` must reject any seed/frame/robot subset,
and must validate the exact separately committed authorization record from
Task 9 before creating its output root.
The three non-production invocation names must reject the 140000-key grid
and any trajectory-level scientific aggregation.
`produce_smoke_row` must dispatch only by the exact case ID,
must consume the mechanism fixture only for
`mechanism_20260727_180_12`,
must consume a canonical in-source synthetic record for every other case,
and must reject a missing,
extra,
duplicate,
or reordered case before output-root creation.

- [ ] **Step 10: Implement and green the pinned evidence lifecycle**

Reuse the v4 no-follow,
descriptor-pinned,
transactional stage,
fsync,
terminal-manifest,
free-space,
and allocated-byte helpers.
Do not weaken them.
Tests must inject:

- start-space failure;
- live-floor failure;
- raw-cap failure;
- preexisting root;
- symlink component;
- write failure;
- fsync failure;
- final-identity mismatch;
- terminal-manifest failure.

Every failure must retain a `failed` or emergency forensic manifest and must
not publish `completed`.
After each Step 8 failure-injection case is first red,
implement its lifecycle component and rerun that case green;
then run the full producer class.
Expected:
all previously red serialization/order/lifecycle tests pass.

- [ ] **Step 11: Rerun deterministic tiny producer tests**

Rerun the two-root hermetic fixture/synthetic test written in Step 8.
Assert:

- decompressed process hashes match;
- compressed process hashes match;
- timestamped manifest hashes may differ;
- rows are strict JSON;
- exact field order is canonical;
- no private state enters public output;
- all considered rejected/unavailable rows retain complete branch fields.

- [ ] **Step 12: Run producer and preserved-v4 regression**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_extract_two_range_reacquisition_fixture \
  tests.test_replay_two_range_reacquisition \
  tests.test_replay_predictive_wnls_recovery -v
```

Expected: PASS.

- [ ] **Step 13: Commit fixture and producer**

```bash
git add \
  scripts/diagnostics/extract_two_range_reacquisition_fixture.py \
  scripts/diagnostics/replay_two_range_reacquisition.py \
  tests/test_extract_two_range_reacquisition_fixture.py \
  tests/test_replay_two_range_reacquisition.py \
  tests/fixtures/cbf2026_two_range_reacquisition
git commit -m "feat(diagnostics): add two-range replay producer"
```

### Task 4: Independent analyzer and adversarial reconstruction

**Files:**
- Create: `scripts/diagnostics/analyze_two_range_reacquisition.py`
- Create: `tests/test_analyze_two_range_reacquisition.py`
- Test: `tests/test_analyze_predictive_wnls_recovery.py`

**Interfaces:**
- Consumes:
  - exact raw declarations and identities from Task 3;
  - preserved legacy baseline process;
  - preserved v4 raw process;
  - protocol-bound truth data;
  - Task 2 pure solver only for independent recomputation from raw runtime
    inputs, never for trusting producer decisions.
- Produces:
  - `ANALYSIS_SCHEMA_ID = "cbf2026-two-range-reacquisition-analysis-v1"`
  - `OUTPUT_JSON_NAME = "two-range-reacquisition.json"`
  - `OUTPUT_MARKDOWN_NAME = "two-range-reacquisition.md"`
  - `ANALYSIS_FIELDS`, `IDENTITY_FIELDS`, `BUDGET_FIELDS`,
    `STATUS_COUNT_FIELDS`, `SELECTOR_ACCOUNTING_FIELDS`,
    `BASELINE_FRESH_TRANSITION_FIELDS`,
    `V4_DESCRIPTIVE_COMPARISON_FIELDS`,
    `V4_FRESH_TRANSITION_FIELDS`,
    `V4_PAIRED_COMPARISON_FIELDS`,
    `ANALYSIS_IDENTITY_RECORD_FIELDS`,
    `PAIRED_COMPARISON_FIELDS`, `GATE_RECORD_FIELDS`,
    `TAIL_RECORD_FIELDS`, `TAIL_METRICS`, `TAIL_STRATIFIERS`,
    `TIME_BINS`, `SMOKE_SEMANTIC_FIELDS`,
    `ANALYSIS_MANIFEST_FIELDS`,
    `ANALYSIS_SOURCE_MEMBER_NAMES`,
    `ANALYSIS_OUTPUT_MEMBER_NAMES`,
    `ANALYSIS_MANIFEST_IDENTITY_FIELDS`,
    and `INTEGRITY_GATE_IDS`.
  - `linear_percentile(values: Iterable[float], fraction: float) -> float | None`
  - `validate_and_reconstruct_row(row: Mapping, *, expected_key: tuple[str, int, int, int], protocol: Mapping, truth_position: np.ndarray, current_public: dict[int, dict], previous_private: dict | None, held_command: object) -> dict`
  - `aggregate_two_range_reacquisition(*, baseline_rows: Iterable[Mapping], v4_rows: Iterable[Mapping], new_rows: Iterable[Mapping], truth_data: Mapping, protocol: Mapping) -> dict`
  - `ANALYZER_INVOCATIONS = ("smoke_analyzer_a", "smoke_analyzer_b", "registered_analyzer")`
  - `analyze_two_range_reacquisition(*, protocol_path: Path, raw_root: Path, output_root: Path, invocation_name: str, authorization_json: Path | None = None) -> Path`
  - CLI `main(argv: list[str] | None = None) -> int`.

- [ ] **Step 1: Bootstrap exact-schema validation, then close the key grid vertically**

Build a minimal valid row through the producer test helper,
then add only the missing-field rejection test.
Run that one test red because the analyzer module does not exist.

After Step 3 creates the minimal module,
ordered decoder,
and missing-field guard,
rerun that test green.
Then add each remaining rejection as its own
red/implementation/green microcycle:

- one extra field;
- reordered serialized fields;
- a wrong method ID;
- a seed outside `20260727..20260746`;
- a frame outside `0..499`;
- a robot outside `1..14`;
- a duplicate key;
- a missing key;
- an extra key;
- a key in the wrong order.

The production exact key assertion is:

```python
self.assertEqual(
    summary["budgets"]["expected_rows"],
    1 * 20 * 500 * 14,
)
self.assertEqual(summary["budgets"]["observed_rows"], 140000)
```

- [ ] **Step 2: Run the bootstrap schema test and verify the red state**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_two_range_reacquisition.ExactSchemaTests.test_missing_field_rejects -v
```

Expected: FAIL because the analyzer module does not exist.

- [ ] **Step 3: Implement strict row and stream validation**

Create the minimal analyzer module and ordered-object decoder,
then implement only the missing-field guard required by the Step 1
bootstrap and rerun it green.
For each later Step 1 mutation,
first run that mutation red and only then tighten the relevant
field-set,
field-order,
literal,
range,
duplicate,
or stream-order guard.
The final green row validator must require:

```python
tuple(row) == replay.ROW_FIELDS
set(row) == set(replay.ROW_FIELDS)
```

For every nested branch:

```python
tuple(branch) == replay.BRANCH_FIELDS
```

Validate exact null/absence semantics before using values.
Do not coerce strings to numbers,
Booleans to integers,
or noncanonical covariance matrices.

Freeze the compact analysis schema as:

```python
ANALYSIS_FIELDS = (
    "schema_id",
    "protocol_id",
    "invocation_name",
    "decision",
    "semantic_payload_sha256",
    "identities",
    "budgets",
    "status_counts",
    "selector_accounting",
    "baseline_fresh_transitions",
    "v4_descriptive_comparison",
    "paired_comparison",
    "scientific_gates",
    "integrity_gates",
    "tails",
    "limitations",
)
IDENTITY_FIELDS = (
    "protocol",
    "authorization",
    "mechanism_fixture",
    "synthetic_case_source",
    "raw_manifest",
    "raw_compressed_process",
    "raw_decompressed_process",
    "v4_manifest",
    "v4_compressed_process",
    "v4_decompressed_process",
    "v4_analysis_manifest",
    "v4_analysis_json",
    "v4_analysis_markdown",
    "legacy_baseline_process",
    "legacy_baseline_protocol_json",
    "truth_data",
)
ANALYSIS_IDENTITY_RECORD_FIELDS = (
    "path",
    "device",
    "inode",
    "size",
    "mtime_ns",
    "sha256",
    "hash_domain",
)
BUDGET_FIELDS = (
    "expected_rows",
    "observed_rows",
    "unique_rows",
    "raw_allocated_bytes",
    "compact_allocated_bytes",
)
STATUS_COUNT_FIELDS = (
    "attempt_accepted",
    "attempt_rejected",
    "attempt_failed",
    "attempt_invalid",
    "attempt_reference_unavailable",
    "output_fresh",
    "output_predicted",
    "output_unavailable",
)
SELECTOR_ACCOUNTING_FIELDS = (
    "considered",
    "accepted",
    "rejected",
    "unavailable",
    "pre_score_rejections",
    "post_score_rejections",
    "score_exactly_one",
    "score_none",
    "score_multiple",
    "score_not_evaluated",
    "root_rejections",
    "downstream_unavailable",
    "outage_episode_count",
    "outage_episode_lengths",
    "fresh_contained",
    "fresh_not_contained",
)
BASELINE_FRESH_TRANSITION_FIELDS = (
    "baseline_fresh_total",
    "new_fresh",
    "new_predicted",
    "new_unavailable",
)
V4_DESCRIPTIVE_COMPARISON_FIELDS = (
    "fresh_transitions",
    "paired_both_fresh",
    "tails",
)
V4_FRESH_TRANSITION_FIELDS = (
    "v4_fresh_total",
    "new_fresh",
    "new_predicted",
    "new_unavailable",
)
V4_PAIRED_COMPARISON_FIELDS = (
    "cohort_size",
    "v4_p95_m",
    "new_p95_m",
    "new_minus_v4_p95_m",
)
PAIRED_COMPARISON_FIELDS = (
    "cohort_size",
    "baseline_p95_m",
    "new_p95_m",
    "new_minus_baseline_p95_m",
)
GATE_RECORD_FIELDS = (
    "gate_id",
    "operator",
    "threshold",
    "numerator",
    "denominator",
    "value",
    "passed",
)
TAIL_RECORD_FIELDS = (
    "metric",
    "stratifier",
    "stratum",
    "population",
    "count",
    "minimum",
    "p50",
    "p95",
    "maximum",
)
TAIL_METRICS = (
    "offline_error_norm",
    "offline_fresh_q_error",
)
TAIL_STRATIFIERS = (
    "depth",
    "seed",
    "robot",
    "time_bin",
    "private_age",
)
V4_TAIL_STRATIFIERS = (
    "depth",
    "seed",
    "robot",
    "time_bin",
)
TIME_BINS = (
    (0, 99),
    (100, 199),
    (200, 299),
    (300, 399),
    (400, 499),
)
TAIL_POPULATIONS = (
    "two_range_accepted_fresh",
    "v4_fresh",
)
SMOKE_SEMANTIC_FIELDS = (
    "schema_id",
    "protocol_id",
    "decision",
    "expected_rows",
    "observed_rows",
    "unique_rows",
    "status_counts",
    "selector_accounting",
    "baseline_fresh_transitions",
    "v4_descriptive_comparison",
    "paired_comparison",
    "scientific_gates",
    "integrity_gates",
    "tails",
    "limitations",
)
ANALYSIS_MANIFEST_FIELDS = (
    "schema_id",
    "protocol_id",
    "invocation_name",
    "status",
    "method",
    "output_root",
    "protocol_identity",
    "authorization_identity",
    "source_identities",
    "output_identities",
    "expected_rows",
    "observed_rows",
    "disk_contract",
    "started_at",
    "completed_at",
    "error",
)
ANALYSIS_MANIFEST_IDENTITY_FIELDS = (
    "path",
    "device",
    "inode",
    "size",
    "allocated_bytes",
    "mtime_ns",
    "sha256",
    "hash_domain",
)
ANALYSIS_OUTPUT_MEMBER_NAMES = (
    "analysis_json",
    "analysis_markdown",
)
ANALYSIS_SOURCE_MEMBER_NAMES = {
    "smoke_analyzer_a": (
        "raw_manifest",
        "raw_compressed_process",
        "raw_decompressed_process",
        "mechanism_fixture",
        "synthetic_case_source",
    ),
    "smoke_analyzer_b": (
        "raw_manifest",
        "raw_compressed_process",
        "raw_decompressed_process",
        "mechanism_fixture",
        "synthetic_case_source",
    ),
    "registered_analyzer": (
        "raw_manifest",
        "raw_compressed_process",
        "raw_decompressed_process",
        "v4_manifest",
        "v4_compressed_process",
        "v4_decompressed_process",
        "v4_analysis_manifest",
        "v4_analysis_json",
        "v4_analysis_markdown",
        "legacy_baseline_process",
        "legacy_baseline_protocol_json",
        "truth_data",
    ),
}
ANALYSIS_ERROR_FIELDS = (
    "type",
    "message",
)
```

Every `identities` value is a strict object with ordered
`ANALYSIS_IDENTITY_RECORD_FIELDS`,
or is invocation-specifically inapplicable and represented by `None`,
never by an abbreviated object.
`hash_domain` is exactly
`file_bytes`
or
`decompressed_jsonl_bytes`;
a decompressed record repeats the retained compressed descriptor's
path/device/inode/size/mtime but carries the decompressed stream digest.
Counts and byte budgets are non-Boolean non-negative integers.
All four baseline-fresh transition fields are non-Boolean non-negative
integers and must satisfy
`baseline_fresh_total = new_fresh + new_predicted + new_unavailable`.
The registered-only `v4_descriptive_comparison` is a strict object in
`V4_DESCRIPTIVE_COMPARISON_FIELDS` order.
Its `fresh_transitions` is a strict
`V4_FRESH_TRANSITION_FIELDS` object of non-Boolean non-negative integers
that satisfies
`v4_fresh_total = new_fresh + new_predicted + new_unavailable`;
its `paired_both_fresh` is a strict
`V4_PAIRED_COMPARISON_FIELDS` object computed on the v4/new both-fresh
intersection;
its `tails` is an exact ordered v4-only tail list.
These values are descriptive and never substitute for,
waive,
or alter a scientific gate.
Gate numerator/denominator are either both non-negative integers or both
`None`;
threshold/value are finite numbers or declared `None`;
`passed` is Boolean.
Tail statistics are all finite or all `None` when count is zero.
Outage episode lengths are non-Boolean positive integers sorted ascending.
For registered analysis only,
fresh-contained plus fresh-not-contained equals the accepted fresh
two-range count.
For smoke analysis,
both fields are the integer zero because synthetic rows have no offline
truth and containment is intentionally not inferred.
Paired cohort size is a non-Boolean non-negative integer.
When it is positive,
both p95 values and their difference are finite non-negative numbers
except that the signed difference may be negative;
when it is zero,
all three numeric fields are `None` and the paired gate fails.
Top-level tail `metric` and `stratifier` are exact members of
`TAIL_METRICS` and `TAIL_STRATIFIERS`;
v4 tail `metric` and `stratifier` are exact members of
`TAIL_METRICS` and `V4_TAIL_STRATIFIERS`.
For the top-level `tails`,
`population` is exactly `two_range_accepted_fresh`;
for `v4_descriptive_comparison["tails"]`,
it is exactly `v4_fresh`;
neither container accepts the other population literal.
`stratum` is a non-Boolean integer for depth/seed/robot/private age and the
literal `"start-end"` string for a time bin;
`count` is a non-Boolean non-negative integer.
For a positive count,
minimum/p50/p95/maximum are finite non-negative and monotonically ordered;
for zero count,
all four are `None`.
`limitations` is the exact ordered literal list:

```python
LIMITATIONS = (
    "single_preserved_truth_trajectory",
    "estimator_outside_controller",
    "diagonal_marginal_covariance_fim_approximation",
    "shared_ancestor_cross_covariance_unmodeled",
    "three_sigma_radius_is_conditional_modeled_surrogate",
    "source_has_243_of_7000_component_bound_violations",
)
```

For `smoke_analyzer_a` and `smoke_analyzer_b`,
`decision` is `smoke_pass` or `smoke_fail`,
authorization,
the v4 direct-comparator fields,
the v4 analysis cross-check fields,
legacy baseline,
legacy baseline protocol,
and truth identities are `None`,
while mechanism-fixture and synthetic-case-source identities are non-null,
`paired_comparison` is `None`,
`baseline_fresh_transitions` is `None`,
`v4_descriptive_comparison` is `None`,
`scientific_gates` and `tails` are empty,
and `expected_rows=observed_rows=unique_rows=18`.
For `registered_analyzer`,
mechanism-fixture and synthetic-case-source identities are `None`,
all other identities and paired fields are non-null,
`baseline_fresh_transitions` is a strict non-null object in the order above,
`v4_descriptive_comparison` is a strict non-null object in the order above,
`decision` is `pass` or `fail`,
and scientific gates contain the exact nine records in protocol order.
`integrity_gates` contains every selector-integrity record in protocol
order in both modes.
The scientific-gate list length is exactly nine and its IDs equal
`tuple(GATES)`;
the integrity-gate list length is exactly
`len(INTEGRITY_GATE_IDS)`
and its IDs match that tuple.
Tail records are ordered by `TAIL_METRICS`,
then `TAIL_STRATIFIERS`,
then canonical stratum:
depth `1..7`,
seed `20260727..20260746`,
robot `1..14`,
private age `0..499`,
and
`TIME_BINS` order.
No empty stratum is omitted;
it is represented with count zero and null statistics.
Therefore registered analysis contains exactly
`2 * (7 + 20 + 14 + 5 + 500) = 1092`
top-level tail records and exactly
`2 * (7 + 20 + 14 + 5) = 92`
nested v4 tail records.
The v4 list intentionally excludes `private_age`,
because the preserved v4 stream has no tagged private-state age and the
analyzer must not invent one;
smoke analysis contains zero.
The limitations list length and order are exactly `LIMITATIONS`.
The JSON writer emits `ANALYSIS_FIELDS` and every nested tuple in exactly
the order above;
the Markdown is rendered only from the already validated canonical object.
`semantic_payload_sha256` is computed from one canonical strict-JSON
projection in `SMOKE_SEMANTIC_FIELDS` order;
its three row-count values come from `budgets`,
and it excludes invocation name,
paths,
file identities,
allocated bytes,
timestamps,
and itself.
The analyzer must verify the projection hash after final object assembly.

The analyzer terminal manifest is a separate strict top-level artifact,
not an abbreviated copy of the compact result.
Its schema ID reuses exactly `ANALYSIS_SCHEMA_ID`;
the strict `ANALYSIS_MANIFEST_FIELDS` tuple and manifest artifact path
discriminate it from the compact-result object without introducing a fifth
schema ID;
its keys equal `ANALYSIS_MANIFEST_FIELDS` in order.
`invocation_name` is an exact `ANALYZER_INVOCATIONS` member;
`status` is one of `creating/running/completed/failed`;
`method`, `protocol_id`, and `output_root` match the invocation contract.
`protocol_identity` is always a strict
`ANALYSIS_MANIFEST_IDENTITY_FIELDS` object with
`hash_domain="file_bytes"`.
`authorization_identity` is always null for both smoke analyzers and is a
strict matching file identity for every registered-analyzer lifecycle that
reaches output-root creation.
`source_identities` is a strict ordered object whose exact member names,
order,
and cardinality equal
`ANALYSIS_SOURCE_MEMBER_NAMES[invocation_name]`.
Every member is a full `ANALYSIS_MANIFEST_IDENTITY_FIELDS` object;
on an identity mismatch the manifest records the observed descriptor and
digest while the error reports the mismatch.
Compressed files use `hash_domain="file_bytes"`;
decompressed process records use
`hash_domain="decompressed_jsonl_bytes"`;
all other members use `file_bytes`.
`output_identities` always has exactly
`ANALYSIS_OUTPUT_MEMBER_NAMES`;
both values are null in creating/running/failed states and both are strict
`file_bytes` identities only in completed state.
`expected_rows` is exactly 18 for smoke or 140000 for registered;
`observed_rows` is a non-Boolean integer in
`0..expected_rows`,
and equals `expected_rows` on completion.
`disk_contract` is byte-for-byte equal to the protocol section.
Timestamps are canonical UTC strings;
`completed_at` is null only in creating/running and is non-null in either
terminal state.
`error` is null in creating/running/completed and is a strict ordered
`ANALYSIS_ERROR_FIELDS` object in failed state.
Every transition rewrites the complete manifest transactionally.
The JSON and Markdown become named completed outputs only after both
descriptors and hashes are pinned;
failure never publishes either as a completed artifact.

Add one mutation microcycle each for:
missing/extra/reordered manifest fields;
wrong schema/protocol/method/invocation;
missing/extra/reordered/substituted source members for every invocation;
wrong source hash domain;
smoke authorization present;
registered authorization absent;
partial or non-null preterminal output identities;
completed null output identity;
row-count overflow or incomplete completion;
disk substitution;
timestamp/null-rule violation;
and failed/completed error-rule violation.

After the minimal analyzer module,
strict row validator,
and schema tests are green,
close the invocation split as three small interface microcycles:

- `smoke_analyzer_a` accepts only the correspondingly named exact 18-row
  raw root,
  reconstructs it,
  and rejects production aggregation or gate output;
- `smoke_analyzer_b` has the same constraints for its own raw root;
- `registered_analyzer` accepts only the exact 140000-row registered root
  and rejects absent,
  dirty,
  uncommitted,
  or mismatched authorization JSON.

For each invocation,
first run one acceptance/rejection test red,
implement only the invocation dispatch/guard required by that test,
then rerun green.
After those three cycles,
add the cross-invocation tests requiring that no invocation accepts another
invocation's raw root or schema.

- [ ] **Step 4: Write failing private-state reconstruction tests**

Bootstrap with only the valid one-frame private-state recursion test,
run it red,
implement only that reconstruction in Step 5,
and rerun it green.
Then add each invariant and each tamper axis below as its own
red/implementation/green microcycle.
The final analyzer independently verifies:

```text
age_frames = propagated_to_frame - source_fresh_frame
propagated_to_frame = current frame for the incoming prior
accepted outgoing source/propagated frame = current frame
accepted outgoing age = 0
rejected outgoing state = incoming state exactly
```

Independently tamper each estimate,
covariance,
source frame,
propagated frame,
age,
held command,
and command source frame independently.
Each tamper must fail.

- [ ] **Step 5: Implement private recursion reconstruction**

Build this reconstruction incrementally through the Step 4 microcycles.
Maintain independent per-`(seed, robot)` prior state.
For frame zero,
require no incoming prior unless reset by a current accepted existing path.
For later frames,
recompute:

```python
expected_prior = propagate_private_state(
    previous_private[key],
    protocol_bound_command,
    next_frame_index=row["frame_index"],
)
```

Compare canonical values exactly to
`branch_selection_prior_*`.
After validating the attempt,
derive the expected outgoing state independently and compare it to
`next_private_state_*`.
Never source expected state from the producer's outgoing fields.

- [ ] **Step 6: Write failing branch reconstruction tests**

Bootstrap with only a valid considered attempt and its exact two mandatory
UAV reference keys,
run it red,
implement only that reconstruction in Step 7,
and rerun it green.
Then add each remaining reconstruction item and each tamper below as its
own red/implementation/green microcycle.
For every considered attempt,
the final analyzer independently reconstructs:

1. exact two mandatory UAV reference keys;
2. current-fresh lower-index public states;
3. recursive base provenance;
4. current ranges;
5. negative and positive circle starts;
6. both WNLS traces/results;
7. result separation;
8. \(q_b\) from raw incoming prior;
9. threshold pass/fail;
10. selected branch;
11. selected `cost <= 9`;
12. public candidate/covariance/epsilon binding.

Tamper and reject:

- forged \(q_b\);
- swapped branch labels;
- hidden/deleted branch;
- private prior as reference;
- private prior as WNLS start;
- algebraic or prediction start;
- distinct circle branches collapsed to one result;
- selected result rebound to the other branch;
- zero/multiple-pass publication;
- predicted output from the new selector;
- FIM/covariance/epsilon substitution;
- truth-dependent or future-frame branch choice;
- missing fixed reference;
- optional-reference substitution.

- [ ] **Step 7: Implement independent branch validation**

Build the validator incrementally through the Step 6 microcycles.
The completed analyzer must call the circle geometry and finite-budget WNLS primitives
from raw current reference states,
not deserialize producer `solver_result` as truth.
Compare every proposal trace field,
terminal solver field,
circle start,
estimate,
covariance,
cost,
FIM statistic,
\(q_b\),
pass Boolean,
and selected branch to the reconstruction.

Use exact canonical equality for discrete fields and the same declared
floating representation for recomputed numeric fields.
Any mismatch raises `ValueError` before aggregation.

- [ ] **Step 8: Freeze and test the exact p95 rule**

Bootstrap with only the one-element case,
run it red because `linear_percentile` does not exist,
implement the minimal one-element path,
and rerun it green.
Then add odd,
even,
empty,
non-finite,
and preserved-v4 NumPy-linear parity cases one at a time;
for each,
run it red,
implement only the next interpolation/validation branch,
and rerun it green.

Then implement:

```python
def linear_percentile(
    values: Iterable[float],
    fraction: float,
) -> float | None:
    data = np.sort(np.asarray(tuple(values), dtype=float))
    if data.size == 0 or not np.isfinite(data).all():
        return None
    h = fraction * (data.size - 1)
    lower = int(math.floor(h))
    upper = int(math.ceil(h))
    weight = h - lower
    return float(data[lower] + weight * (data[upper] - data[lower]))
```

Rerun the analyzer invocation-split tests closed after Step 3 together with
the percentile tests.
No new invocation behavior is first specified after implementation.

- [ ] **Step 9: Write failing aggregate gates, then implement paired joins**

The items in this step and Steps 10--11 form one ordered microcycle queue;
they are not one batch-red suite.
For each item,
write the smallest boundary test,
run it red for the intended missing behavior,
implement only the corresponding join,
gate record,
or tail record,
and rerun it green before moving to the next item.
Cover:

- empty,
  one-row,
  equality,
  improvement,
  and worsening paired both-fresh cohorts;
- fresh retention at `124646/124647`;
- fresh-or-predicted at `132999/133000`;
- maximum error at
  `nextafter(50,-inf)`,
  `50`,
  and
  `nextafter(50,+inf)`;
- prediction age `2/3`;
- each of the three zero-violation scientific gates;
- each ordered selector-integrity gate at zero/one violation;
- exact baseline-fresh transition sum;
- exact 1092 top-level-tail order/cardinality;
- exact 92 v4-descriptive-tail order/cardinality with no private-age
  stratum.

Begin with the empty paired cohort,
then one-row,
equality,
improvement,
and worsening cohorts.
Implement the legacy-baseline,
v4 `predictive_multistart`,
and new method on exact
`(seed, frame, robot)`.
For the paired both-fresh gate:

```python
cohort = [
    key
    for key in exact_keys
    if baseline[key]["output_status"] == "fresh"
    and new[key]["output_status"] == "fresh"
]
baseline_p95 = linear_percentile(
    (baseline[key]["error"] for key in cohort), 0.95
)
new_p95 = linear_percentile(
    (new[key]["error"] for key in cohort), 0.95
)
passed = bool(cohort) and new_p95 <= baseline_p95
```

Do not reuse the old v4 scalar when the intersection changes.
Report v4 transitions and tails descriptively,
but do not substitute them for any frozen gate.

- [ ] **Step 10: Implement all nine scientific gates**

Implement each record only when its corresponding Step 9 boundary test is
red,
then rerun that boundary test green before adding the next record.
After the ninth microcycle,
the complete ordered gate mapping is:

The gate records are exactly:

```python
GATES = {
    "maximum_published_error_m_strictly_below": 50.0,
    "maximum_fresh_error_m_strictly_below": 50.0,
    "paired_both_fresh_p95_must_not_worsen": True,
    "fresh_availability_max_drop_fraction": 0.02,
    "fresh_or_predicted_min_fraction": 0.95,
    "maximum_prediction_age_frames": 2,
    "qualification_anchor_violations_allowed": 0,
    "current_frame_provenance_violations_allowed": 0,
    "ascending_dag_violations_allowed": 0,
}
```

Pin integer availability thresholds:

```python
fresh_retention_required = 124647
fresh_or_predicted_required = 133000
```

Favorable conditional errors never override an availability failure.

- [ ] **Step 11: Implement additional selector-integrity gates**

Implement each integrity record only through its own zero/one Step 9
boundary microcycle.
After all fourteen are green,
freeze this order:

```python
INTEGRITY_GATE_IDS = (
    "nonfresh_anchor_use",
    "selector_reference_set_violation",
    "missing_fixed_reference_publication",
    "private_prior_role_violation",
    "noncircle_continuous_start",
    "noncircle_publication_representative",
    "nonruntime_branch_score",
    "branch_selection_reconstruction_mismatch",
    "nonunique_passing_branch_publication",
    "selected_result_binding_mismatch",
    "private_state_recursion_mismatch",
    "predicted_selector_output",
    "exact_denominator_violation",
    "preserved_contract_violation",
)
```

Every record uses operator `equal`,
threshold `0`,
and requires zero:

- nonfresh/predicted/unavailable anchor uses;
- invocation with active base/optional/non-UAV reference;
- missing fixed-reference fresh publications;
- private prior used as anchor/FIM/continuous input/covariance/representative;
- non-circle start or representative;
- branch reconstruction mismatch;
- zero/multiple-pass fresh publication;
- selected-result mismatch;
- private-state recursion mismatch;
- predicted selector output;
- exact-key-grid violation.
- mutation of the frozen covariance,
  candidate,
  command,
  reference,
  or provenance contracts.

Report considered counts,
rejection stage,
branch-score outcomes
`exactly_one/none/multiple/not_evaluated`,
accepted/rejected/unavailable counts,
outage lineage/episodes,
and tails by depth/seed/robot/time/private age.
Run the aggregate tests written in Step 9.
Expected:
all paired,
scientific-gate,
integrity-gate,
transition,
and tail boundary cases pass.

- [ ] **Step 12: Add compact-output lifecycle tests**

Bootstrap this step with one creating-state
`ANALYSIS_MANIFEST_FIELDS` order test,
run it red,
implement only the full-field manifest builder and transactional manifest
rewrite,
and rerun it green.
Then close every analysis-manifest mutation listed in Step 3 as its own
red/implementation/green microcycle before the failure injections.

Treat each injection below as a separate vertical microcycle:
add one failing test,
run it red for the intended missing transaction guard,
implement only that guard/cleanup path,
and rerun it green before adding the next injection.
Inject:

- raw identity mismatch;
- v4 comparator mismatch;
- baseline identity mismatch;
- compact cap overflow;
- output-root preexistence;
- JSON write/fsync failure;
- analyzer crash after root creation.

Require a retained failed terminal manifest and no completed compact result.
Incrementally implement the no-follow,
descriptor-pinned,
10 MB capped,
transactional compact lifecycle through those seven cycles.
After all are green,
run the complete lifecycle class once as regression.

- [ ] **Step 13: Run analyzer and preserved analyzer regression**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_two_range_reacquisition \
  tests.test_analyze_predictive_wnls_recovery -v
```

Expected: PASS.

- [ ] **Step 14: Commit the analyzer**

```bash
git add \
  scripts/diagnostics/analyze_two_range_reacquisition.py \
  tests/test_analyze_two_range_reacquisition.py
git commit -m "feat(diagnostics): add two-range evidence analyzer"
```

### Task 5: Exact registrar and authorization protocol

**Files:**
- Create: `scripts/diagnostics/register_two_range_reacquisition.py`
- Create: `tests/test_register_two_range_reacquisition.py`
- Test: `tests/test_register_predictive_wnls_stage1.py`

**Interfaces:**
- Consumes:
  - method/raw/analysis declarations from Tasks 2–4;
  - approved design/review commit and hashes;
  - repository source identities;
  - exact preserved comparator identities.
- Produces:
  - `PROTOCOL_SCHEMA_ID`
  - `REGISTRATION_SCHEMA_ID`
  - `PROTOCOL_ID = "cbf2026-two-range-reacquisition-v1"`
  - `production_invocation_contract() -> dict`
  - `production_command_contract(sources: Mapping) -> dict[str, list[str]]`
  - `register_two_range_protocol(*, repository_root: Path, output_markdown: Path, output_json: Path) -> tuple[Path, Path]`
  - CLI `main(argv: list[str] | None = None) -> int`.

- [ ] **Step 1: Write failing exact protocol-field tests**

Bootstrap with one test that imports the registrar and requires only the
exact `PROTOCOL_FIELDS` top-level tuple.
Step 2 runs it red because the registrar does not exist.
Step 3 then creates only the module,
schema IDs,
and that tuple and reruns the test green.
After that first green,
Steps 3--9 add each section tuple,
strict-order rule,
value-type rule,
and cardinality rule below as its own
red/implementation/green microcycle.
The final green protocol requires this exact top-level order:

```python
PROTOCOL_FIELDS = (
    "schema_id",
    "registration_schema_id",
    "protocol_id",
    "implementation_parent_commit",
    "binding_design",
    "sources",
    "comparators",
    "experiment",
    "method_contract",
    "estimator_constants",
    "status_contract",
    "raw_schema",
    "analysis_schema",
    "gates",
    "disk_contract",
    "invocations",
    "evidence_lifecycle",
    "authorization",
    "commands",
)
SOURCE_MEMBER_NAMES = (
    "implementation_plan",
    "two_range_reacquisition_source",
    "predictive_wnls_source",
    "fixture_extractor_source",
    "replay_source",
    "analyzer_source",
    "registrar_source",
    "mechanism_fixture",
    "mechanism_fixture_manifest",
    "truth_data",
    "input_manifest",
)
INVOCATION_MEMBER_NAMES = (
    "smoke_a",
    "smoke_b",
    "smoke_analyzer_a",
    "smoke_analyzer_b",
    "registered_replay",
    "registered_analyzer",
)
PROTOCOL_SECTION_FIELDS = {
    "binding_design": (
        "path",
        "commit",
        "sha256",
        "review_path",
        "review_sha256",
    ),
    "source_record": (
        "path",
        "device",
        "inode",
        "size",
        "mtime_ns",
        "sha256",
    ),
    "sources": SOURCE_MEMBER_NAMES,
    "comparators": (
        "v4_replay_root",
        "v4_replay_manifest",
        "v4_compressed_process",
        "v4_decompressed_process",
        "v4_analysis_root",
        "v4_analysis_manifest",
        "v4_analysis_json",
        "v4_analysis_markdown",
        "legacy_baseline_process",
        "legacy_baseline_protocol_json_sha256",
    ),
    "experiment": (
        "method",
        "seeds",
        "frames",
        "robots",
        "expected_rows",
        "key_order",
        "ranging_sigma_m",
        "frame_dt_seconds",
        "measurement_seed_contract",
        "evidence_class",
    ),
    "method_contract": (
        "structural_conditions",
        "branch_ids",
        "continuous_starts",
        "private_prior_allowed_roles",
        "private_prior_forbidden_roles",
        "branch_score_rule",
        "publication_rule",
        "failure_rule",
        "synthetic_declaration",
        "synthetic_declaration_sha256",
    ),
    "estimator_constants": (
        "maximum_public_prediction_age",
        "innovation_reference_quantile",
        "candidate_dedup_m",
        "motion_covariance_per_frame",
        "reacquisition_reduced_cost_max",
        "maximum_error_m",
    ),
    "status_contract": (
        "attempt_statuses",
        "output_statuses",
        "private_statuses",
        "prior_used_semantics",
    ),
    "raw_schema": (
        "schema_id",
        "row_fields",
        "row_scalar_contracts",
        "row_array_contracts",
        "nested_field_orders",
        "branch_field_contracts",
        "manifest_fields",
        "null_rules",
    ),
    "analysis_schema": (
        "schema_id",
        "analysis_fields",
        "nested_field_orders",
        "value_contracts",
        "list_order_and_cardinality",
        "semantic_payload_fields",
        "manifest_fields",
        "manifest_nested_field_orders",
        "manifest_source_member_names",
        "manifest_null_rules",
    ),
    "gates": (
        "scientific_gate_order",
        "scientific_gate_records",
        "integrity_gate_order",
        "integrity_gate_records",
        "aggregate_decision_rule",
    ),
    "disk_contract": (
        "launch_minimum_free_bytes",
        "live_minimum_free_bytes",
        "raw_bundle_max_allocated_bytes",
        "compact_bundle_max_allocated_bytes",
    ),
    "invocation_record": (
        "invocation_name",
        "input_root",
        "output_root",
        "expected_rows",
        "authorization_required",
        "retry_allowed",
    ),
    "invocations": INVOCATION_MEMBER_NAMES,
    "evidence_lifecycle": (
        "preexisting_target_allowed",
        "no_follow",
        "descriptor_pinning",
        "transactional_publication",
        "fsync_required",
        "terminal_manifest_required",
        "failure_retained",
        "paper_gate",
    ),
    "authorization": (
        "implementation_plan_approved",
        "protocol_preflight_required",
        "deterministic_smoke_review_required",
        "registered_full_grid_authorization",
        "authorization_record_schema",
        "authorization_record_path",
    ),
}
COMMAND_FIELDS = INVOCATION_MEMBER_NAMES
```

Assert all four exact schema IDs
(protocol,
registration,
raw,
and analysis,
with the strict analysis-manifest field tuple bound under the analysis
schema)
and the stable method ID.
Assert every protocol section is a strict object in the declared order;
all field-order and contract maps embed their literal content rather than
only a Python constant name.
No section is null.
`sources` is a strict ordered object with exactly eleven
`SOURCE_MEMBER_NAMES`,
each value a strict `source_record`;
`invocations` is a strict ordered object with exactly six
`INVOCATION_MEMBER_NAMES`,
each value a strict `invocation_record`;
`commands` has the same six member names and order.
Counts/bytes/indices are non-Boolean integers in their declared ranges;
all estimator constants are finite;
hashes/OIDs use the canonical forms required by the authorization schema;
paths are exact absolute roots or exact repository-relative artifact paths;
Booleans are not coerced.
`commands` has exactly `COMMAND_FIELDS`,
and each value is a non-empty ordered string argv with no shell syntax,
wildcards,
or implicit defaults.
After the minimal valid schema case has passed,
close every mutation/lifecycle case listed in Step 9 as a separate
red/implementation/green microcycle against the planned registrar
interface.
Do not batch-write the complete mutation suite before the first canonical
protocol object and strict validator are green.

- [ ] **Step 2: Run the registrar bootstrap test and verify the red state**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_register_two_range_reacquisition.ProtocolSchemaTests.test_top_level_field_order -v
```

Expected: FAIL because the registrar does not exist.

- [ ] **Step 3: Bind the approved design and implementation parent**

First create the minimal registrar module,
schema IDs,
and `PROTOCOL_FIELDS` required by the Step 1 bootstrap,
then rerun that focused test green.
Only then add the binding-design test red,
implement this binding,
and rerun it green.

Require:

```python
BINDING_DESIGN = {
    "path": (
        "docs/superpowers/specs/"
        "2026-07-30-cbf2026-two-range-reacquisition-design.md"
    ),
    "commit": "20a61aad96af35ee7e16434fab0a5edaaea38ef0",
    "sha256": (
        "d28d6bfe297a6e37b0a878dc716bd5b91f26c5b11568c3f6fc42bd07760a266b"
    ),
    "review_path": (
        "docs/superpowers/specs/reviews/"
        "2026-07-30-cbf2026-two-range-reacquisition-design-review.md"
    ),
    "review_sha256": (
        "6d857be965de5b83bb05ce06381b79108fdc4abbccf6b96fe4ec456d7a7df570"
    ),
}
```

The registrar must reject dirty required sources,
a detached/unrelated implementation parent,
an implementation parent that already contains the generated protocol,
and any source whose path/size/SHA changes while read.

- [ ] **Step 4: Bind authoritative v4 comparator evidence**

Pin:

```python
EXPECTED_COMPARATOR_BINDINGS = {
    "v4_replay_root": (
        "/private/tmp/cbf2026-predictive-wnls-development/stage1-v4"
    ),
    "v4_manifest_sha256": (
        "123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223"
    ),
    "v4_compressed_sha256": (
        "9710d1a5792e682bb272d0bb3305162180d1e0b9a0636647d6e071d1782a699b"
    ),
    "v4_decompressed_sha256": (
        "7047989804fd03bd4cadffac8212349b7183857623ac201d14ef2fc3e30ee2e1"
    ),
    "v4_analysis_root": (
        "/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v4"
    ),
    "v4_analysis_manifest_sha256": (
        "e50352825665bb94c0e3b32c7922a081f093100daf0ed8e5c8ce9e9f32bd9238"
    ),
    "v4_analysis_json_sha256": (
        "8a0a417f2d3b139fa248bef8157a3d4871fa04042aaad82ce5cd31f470a6ff1e"
    ),
    "v4_analysis_markdown_sha256": (
        "986dcf72f72ab6dc3d424d0de6cdbf410cf271d174d16caef32945acc7d4e13b"
    ),
    "legacy_baseline_process_sha256": (
        "c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003"
    ),
    "legacy_baseline_protocol_json_sha256": (
        "09b236978e2e21bac226dc3d00c36a2ee5cdf5fea0e616f2c1d875b029d4e4e0"
    ),
}
```

`EXPECTED_COMPARATOR_BINDINGS` is registrar input,
not the serialized `comparators` object.
The registrar no-follow resolves each expected digest under its exact root
into the correspondingly named strict
`PROTOCOL_SECTION_FIELDS["comparators"]`
record:
manifest/JSON/Markdown/baseline entries are complete source-record
identities,
the compressed/decompressed entries use the same retained process
descriptor with their respective hash domains,
and root/protocol-hash entries remain exact literals.
Tests assert this one-to-one transformation and reject any unresolved,
extra,
or renamed key.

Read the legacy baseline path,
size,
device,
inode,
mtime,
and SHA from the exact v4 manifest.
Resolve the three v4 analysis files only underneath
`v4_analysis_root`;
reject relocation,
symlink substitution,
or a hash-matching copy at any other path.
Missing,
relocated,
or mismatched evidence fails registration.

- [ ] **Step 5: Freeze exact invocations and roots**

Use:

```python
ROOTS = {
    "smoke_a": "/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-a",
    "smoke_b": "/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-b",
    "smoke_analyzer_a": (
        "/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-a"
    ),
    "smoke_analyzer_b": (
        "/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-b"
    ),
    "registered_replay": (
        "/private/tmp/cbf2026-two-range-reacquisition-development/v1"
    ),
    "registered_analyzer": (
        "/private/tmp/cbf2026-two-range-reacquisition-analysis/v1"
    ),
}
```

All six must be absent at registration and preflight.
The registrar must reject retired/current v2/v3/v4 roots as any new output
target.

- [ ] **Step 6: Freeze the exact smoke cases**

Import `SMOKE_CASE_IDS` from the producer and reassert,
without a local alternate declaration,
these exact ordered case IDs:

```python
assert replay.SMOKE_CASE_IDS == (
    "mechanism_20260727_180_12",
    "select_negative",
    "select_positive",
    "q_equal_threshold",
    "q_below_threshold",
    "q_above_threshold",
    "none_pass",
    "multiple_pass",
    "tangent",
    "disjoint",
    "contained",
    "coincident",
    "zero_range",
    "nearly_collinear",
    "merged_solver_branches",
    "invalid_private_covariance",
    "cost_equal_nine",
    "cost_above_nine",
)
```

Smoke A/B may contain only these 18 rows and may not compute trajectory-level
error or availability aggregates.

- [ ] **Step 7: Freeze exact production commands**

The registrar emits canonical argv arrays for:

```text
smoke_a
smoke_b
smoke_analyzer_a
smoke_analyzer_b
registered_replay
registered_analyzer
```

The registered replay command binds seeds `20260727..20260746`,
500 frames,
14 robots,
one method,
and `140000` expected rows.
Each smoke-analyzer command binds exactly one correspondingly named
18-row smoke root,
one distinct compact output root,
its exact analyzer invocation name,
and the 10 MB cap.
It contains no production comparator,
authorization,
or scientific-gate argument.
Both registered commands contain the exact argument:

```text
--authorization-json
docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-registered-authorization.json
```

The analyzer command binds the registered replay root and all preserved
comparators.
No command may contain a wildcard,
relative evidence root,
or alternate output root.

- [ ] **Step 8: Freeze disk, retry, and authorization semantics**

The exact `disk_contract` is:

```python
{
    "launch_minimum_free_bytes": 8_000_000_000,
    "live_minimum_free_bytes": 6_000_000_000,
    "raw_bundle_max_allocated_bytes": 2_000_000_000,
    "compact_bundle_max_allocated_bytes": 10_000_000,
}
```

The exact `evidence_lifecycle` is:

```python
{
    "preexisting_target_allowed": False,
    "no_follow": True,
    "descriptor_pinning": True,
    "transactional_publication": True,
    "fsync_required": True,
    "terminal_manifest_required": True,
    "failure_retained": True,
    "paper_gate": "CLOSED",
}
```

Every invocation record has `retry_allowed=False`;
in particular the two registered invocations are the scientific no-retry
contract.

`authorization` must distinguish:

```python
{
    "implementation_plan_approved": True,
    "protocol_preflight_required": True,
    "deterministic_smoke_review_required": True,
    "registered_full_grid_authorization": "pending_external_record",
    "authorization_record_schema": (
        "cbf2026-two-range-reacquisition-registration-v1"
    ),
    "authorization_record_path": (
        "docs/diagnostics/reviews/"
        "2026-07-30-cbf2026-two-range-reacquisition-"
        "registered-authorization.json"
    ),
}
```

The protocol remains unchanged.
After smoke review,
authorization is supplied only by the separately committed exact record at
that path.
The producer and analyzer reject an absent,
uncommitted,
dirty,
wrong-schema,
wrong-protocol,
or wrong-smoke authorization record.

- [ ] **Step 9: Green registrar mutation/lifecycle tests**

The tests already written in Step 1 must reject:

- wrong schema ID;
- changed threshold;
- changed seed/frame/robot grid;
- reordered smoke cases;
- changed comparator path/hash;
- dirty required source;
- preexisting root;
- symlink target;
- non-ancestor implementation parent;
- circular protocol parent;
- partial output file;
- protocol Markdown/JSON byte mismatch across deterministic dry runs.

Run the full registrar test class.
Expected:
all exact-schema,
source/comparator,
root,
authorization,
command,
mutation,
and deterministic-output cases pass.

- [ ] **Step 10: Run registrar and legacy registrar regression**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_register_two_range_reacquisition \
  tests.test_register_predictive_wnls_stage1 -v
```

Expected: PASS.

- [ ] **Step 11: Commit the registrar**

```bash
git add \
  scripts/diagnostics/register_two_range_reacquisition.py \
  tests/test_register_two_range_reacquisition.py
git commit -m "feat(diagnostics): add two-range protocol registrar"
```

### Task 6: Full implementation verification and independent review

**Files:**
- Modify only if tests expose a defect:
  `scripts/diagnostics/two_range_reacquisition.py`
- Modify only if tests expose a defect:
  `scripts/diagnostics/predictive_wnls.py`
- Modify only if tests expose a defect:
  `scripts/diagnostics/replay_two_range_reacquisition.py`
- Modify only if tests expose a defect:
  `scripts/diagnostics/analyze_two_range_reacquisition.py`
- Modify only if tests expose a defect:
  `scripts/diagnostics/register_two_range_reacquisition.py`
- Create: `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-implementation.md`
- Create: `docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-implementation-review.md`
- Update append-only:
  `/private/tmp/dra-cbf2026-diagnostic/meta-log/2026-07-30-cbf2026-predictive-wnls-recovery.md`
- Update append-only:
  `/private/tmp/dra-cbf2026-diagnostic/papers/cbf2026/status.md`
- Update append-only:
  `/private/tmp/dra-cbf2026-diagnostic/papers/cbf2026/timeline.md`
- Update append-only:
  `/private/tmp/dra-cbf2026-diagnostic/papers/cbf2026/open-questions.md`

**Interfaces:**
- Consumes all implementation commits from Tasks 1–5.
- Produces a clean implementation commit,
  a complete verification report,
  an independent C0/I0 review,
  and a DRA checkpoint.

- [ ] **Step 1: Run all new focused tests**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_two_range_reacquisition \
  tests.test_extract_two_range_reacquisition_fixture \
  tests.test_replay_two_range_reacquisition \
  tests.test_analyze_two_range_reacquisition \
  tests.test_register_two_range_reacquisition -v
```

Expected: PASS.

- [ ] **Step 2: Run all preserved predictive-WNLS tests**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_predictive_wnls \
  tests.test_extract_predictive_wnls_stage0 \
  tests.test_replay_predictive_wnls_recovery \
  tests.test_analyze_predictive_wnls_recovery \
  tests.test_register_predictive_wnls_stage1 -v
```

Expected: PASS with no v4 declaration,
schema,
command,
hash,
or lifecycle change.

- [ ] **Step 3: Run complete repository Python discovery**

Run:

```bash
conda run -n cbf_env python -m unittest discover -s tests -p 'test*.py' -v
```

Expected: PASS with zero failures and zero errors.

- [ ] **Step 4: Compile every changed Python module**

Run:

```bash
conda run -n cbf_env python -m py_compile \
  scripts/diagnostics/two_range_reacquisition.py \
  scripts/diagnostics/predictive_wnls.py \
  scripts/diagnostics/extract_two_range_reacquisition_fixture.py \
  scripts/diagnostics/replay_two_range_reacquisition.py \
  scripts/diagnostics/analyze_two_range_reacquisition.py \
  scripts/diagnostics/register_two_range_reacquisition.py
```

Expected: exit `0`.

- [ ] **Step 5: Run direct-CLI and shadow-import regressions**

Run each `--help` command from the repository root and from a temporary
directory with an unrelated `scripts` package earlier on `PYTHONPATH`.
Each command must import the implementation-root module and exit `0`:

```bash
conda run -n cbf_env python \
  scripts/diagnostics/extract_two_range_reacquisition_fixture.py --help
conda run -n cbf_env python \
  scripts/diagnostics/replay_two_range_reacquisition.py --help
conda run -n cbf_env python \
  scripts/diagnostics/analyze_two_range_reacquisition.py --help
conda run -n cbf_env python \
  scripts/diagnostics/register_two_range_reacquisition.py --help
```

- [ ] **Step 6: Verify immutable identities and root absence**

Recompute:

```text
legacy solver source:
  0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8
v4 replay manifest:
  123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223
v4 compressed process:
  9710d1a5792e682bb272d0bb3305162180d1e0b9a0636647d6e071d1782a699b
v4 decompressed process:
  7047989804fd03bd4cadffac8212349b7183857623ac201d14ef2fc3e30ee2e1
```

Assert all six new execution roots are absent.
Do not create them during this task.

- [ ] **Step 7: Run repository hygiene checks**

Run:

```bash
git diff --check
git status --short
```

Expected:
only intended source/test/fixture/report changes plus the preserved untracked
`build-diagnostic/`.

- [ ] **Step 8: Write the implementation report**

Record:

- branch and commit chain;
- planning-closure commit plus exact path,
  byte size,
  and SHA-256 of this committed implementation plan;
- exact files and interfaces;
- test commands and counts;
- fixture path/bytes/hash;
- immutable comparator/source hashes;
- proof that private prior never enters WNLS starts/FIM/public output;
- proof that default Stage 1 behavior remains unchanged;
- proof no new execution root exists;
- remaining paper/theory limitations.

- [ ] **Step 9: Dispatch independent implementation reviews**

Use separate reviewers for:

1. estimator/state theory and status semantics;
2. producer/schema/provenance and lifecycle;
3. analyzer/registrar adversarial and evidence boundaries.

Require zero Critical and zero Important findings.
Fix findings test-first,
rerun the focused and full suites,
and record every finding/resolution in the review artifact.

- [ ] **Step 10: Commit implementation closure**

```bash
git add \
  scripts/diagnostics/two_range_reacquisition.py \
  scripts/diagnostics/predictive_wnls.py \
  scripts/diagnostics/extract_two_range_reacquisition_fixture.py \
  scripts/diagnostics/replay_two_range_reacquisition.py \
  scripts/diagnostics/analyze_two_range_reacquisition.py \
  scripts/diagnostics/register_two_range_reacquisition.py \
  tests/test_two_range_reacquisition.py \
  tests/test_extract_two_range_reacquisition_fixture.py \
  tests/test_replay_two_range_reacquisition.py \
  tests/test_analyze_two_range_reacquisition.py \
  tests/test_register_two_range_reacquisition.py \
  tests/fixtures/cbf2026_two_range_reacquisition \
  docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-implementation.md \
  docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-implementation-review.md
git commit -m "docs(diagnostics): approve two-range implementation"
```

Do not stage generated protocol files in this commit.

- [ ] **Step 11: Append the implementation checkpoint to DRA main**

Record source worktree,
branch,
commit,
upstream,
file bytes/hashes,
test counts,
review C/I/M,
fixture identity,
and the fact that no protocol/smoke/full-grid root exists.
Have a separate reviewer check the DRA diff,
then commit:

```bash
git add \
  meta-log/2026-07-30-cbf2026-predictive-wnls-recovery.md \
  papers/cbf2026/status.md \
  papers/cbf2026/timeline.md \
  papers/cbf2026/open-questions.md
git commit -m "docs(cbf2026): record two-range implementation"
```

Do not push.

### Task 7: Generate and independently preflight the immutable protocol

**Files:**
- Create:
  `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json`
- Create:
  `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.md`
- Create:
  `docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1-review.md`
- Update append-only DRA CBF2026 meta/status/timeline/open-questions files.

**Interfaces:**
- Consumes the clean implementation closure commit from Task 6.
- Produces one committed protocol generated exactly once at its production
  targets and one independent preflight approval commit.

- [ ] **Step 1: Recheck preconditions without creating roots**

Verify:

- working tree clean except `build-diagnostic/`;
- HEAD is the reviewed implementation commit;
- all required source hashes match;
- design/review hashes match;
- authoritative v4 comparator identities match;
- all six new roots are absent;
- `/private/tmp` has at least `8_000_000_000` free bytes.

Any failure stops protocol generation.

- [ ] **Step 2: Run registrar dry determinism only in temporary files**

Use two distinct temporary directories.
Generate protocol JSON/Markdown in each,
compare corresponding bytes,
then remove only those known temporary directories.
This does not execute the new estimator.
The production output paths must remain absent.

- [ ] **Step 3: Generate production protocol exactly once**

Run:

```bash
conda run -n cbf_env python \
  scripts/diagnostics/register_two_range_reacquisition.py \
  --repository-root /private/tmp/cbf2026-diagnostic \
  --output-markdown \
    docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.md \
  --output-json \
    docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json
```

Expected:
both targets are created once,
strictly,
with no execution root.
Never run this command again for these production paths.

- [ ] **Step 4: Validate the generated protocol**

Run the registrar/protocol test suite against the production files.
Confirm:

- non-circular implementation parent;
- exact source/comparator path/size/SHA binding;
- exact 18 smoke cases;
- exact 140000 production keys;
- all nine gates;
- all selector-integrity gates;
- four schema IDs,
  with the analysis manifest bound by its exact field tuple under the
  analysis schema;
- six exact roots;
- canonical command arrays;
- disk caps;
- no retry;
- full-grid authorization remains `pending_external_record`;
- the authorization-record path does not yet exist.

- [ ] **Step 5: Commit the protocol**

```bash
git add \
  docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json \
  docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.md
git commit -m "docs(diagnostics): freeze two-range protocol"
```

- [ ] **Step 6: Conduct independent preflight**

The preflight reviewer must read source and protocol directly and verify all
Task 7 Step 4 items,
plus:

- no new root exists;
- no unregistered scientific run has occurred;
- smoke commands contain only 18 approved cases;
- registered command is still separately unauthorized;
- protocol commit contains no implementation source change.

Record exact file hashes,
commit OIDs,
root absence,
disk free bytes,
and C/I/M.

- [ ] **Step 7: Commit preflight approval**

```bash
git add \
  docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1-review.md
git commit -m "docs(diagnostics): approve two-range preflight"
```

- [ ] **Step 8: Append protocol/preflight checkpoint to DRA main**

Record protocol/preflight commit OIDs,
file bytes/hashes,
review C/I/M,
root absence,
and authorization state.
Review the DRA diff independently,
then run:

```bash
git add \
  meta-log/2026-07-30-cbf2026-predictive-wnls-recovery.md \
  papers/cbf2026/status.md \
  papers/cbf2026/timeline.md \
  papers/cbf2026/open-questions.md
git commit -m "docs(cbf2026): record two-range protocol preflight"
```

and do not push.

### Task 8: Execute only the two deterministic fixture/synthetic smokes

**Files:**
- Create:
  `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-smoke.md`
- Update append-only DRA CBF2026 meta/status/timeline/open-questions files.
- External immutable roots:
  `/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-a`
  and
  `/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-b`;
  `/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-a`
  and
  `/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-b`.

**Interfaces:**
- Consumes the committed preflight-approved protocol.
- Produces exactly two 18-row smoke streams,
  two independently reconstructed compact smoke-validation bundles,
  and a reviewed smoke gate.

- [ ] **Step 1: Verify smoke-A preconditions**

Check clean source/protocol identities,
preflight approval commit,
root absence,
at least 8 GB free,
and exact canonical `smoke_a` argv from the protocol.

- [ ] **Step 2: Execute smoke A exactly once**

Execute the exact `commands.smoke_a` array from the committed protocol.
Do not reconstruct or edit the command manually.
If it fails,
retain the failed root,
stop the plan,
and do not execute smoke B.

- [ ] **Step 3: Verify smoke-B preconditions**

Recheck all source/protocol identities,
root B absence,
and at least 8 GB free.
Root A must be terminal and unchanged.

- [ ] **Step 4: Execute smoke B exactly once**

Execute the exact `commands.smoke_b` array.
If it fails,
retain both roots and stop.

- [ ] **Step 5: Compare deterministic streams and manifests**

Require:

- A/B decompressed process SHA-256 equal;
- A/B compressed process SHA-256 equal;
- exactly 18 rows each;
- exact ordered smoke case IDs;
- strict schema and complete terminal manifests;
- manifest hashes recorded separately rather than required equal;
- no trajectory-level scientific error/availability aggregate.

- [ ] **Step 6: Verify and execute smoke analyzer A exactly once**

Verify source/protocol/raw-A identities,
analysis-root-A absence,
at least 8 GB free,
and exact
`commands.smoke_analyzer_a`
from the committed protocol.
Execute that argv byte-for-byte.
It may validate only the 18 declared cases and must independently reconstruct
branch starts,
solver traces,
q values,
state recursion,
selection,
failure cases,
and output binding.
It must not emit or evaluate production scientific gates.
If it fails,
retain the failed analysis root,
stop the protocol,
and do not run analyzer B.

- [ ] **Step 7: Verify and execute smoke analyzer B exactly once**

Recheck every immutable identity,
require analysis root B absent and root A terminal/unchanged,
and execute exact
`commands.smoke_analyzer_b`.
If it fails,
retain all four smoke roots and stop.
Neither smoke producer nor analyzer root may be retried,
overwritten,
or replaced under this protocol.

- [ ] **Step 8: Compare the two compact validation bundles**

Require:

- A/B `semantic_payload_sha256` values equal and independently recomputed;
- A/B analysis JSON,
  Markdown,
  and terminal-manifest hashes recorded separately,
  not required equal because invocation names and file identities differ;
- both decisions equal `smoke_pass`;
- both budgets equal exactly 18/18/18 rows;
- both scientific-gate arrays and tail arrays empty;
- both paired-comparison and baseline-transition objects null;
- every selector-integrity gate present in exact order and passing;
- both compact bundles within 10 MB with complete terminal manifests;
- terminal manifest hashes recorded separately rather than required equal.

- [ ] **Step 9: Write and independently review the smoke report**

Record roots,
manifest/process hashes,
allocated bytes,
free-space guards,
case counts,
analyzer result,
and all limitations.
Require independent C0/I0/M0 before proceeding.

- [ ] **Step 10: Commit smoke closure and update DRA**

```bash
git add \
  docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-smoke.md
git commit -m "docs(diagnostics): record two-range smoke"
```

Append the same facts and hashes to DRA main,
review the append-only diff,
then run:

```bash
git add \
  meta-log/2026-07-30-cbf2026-predictive-wnls-recovery.md \
  papers/cbf2026/status.md \
  papers/cbf2026/timeline.md \
  papers/cbf2026/open-questions.md
git commit -m "docs(cbf2026): record two-range smoke"
```

and do not push.

- [ ] **Step 11: Stop for registered-full-grid authorization**

Present the smoke report and exact registered command to the user.
Do not create the external authorization record
and do not create either registered root until the user explicitly
authorizes this one no-retry execution.

### Task 9: Run the registered grid and analyzer exactly once

**Files:**
- Create after successful analysis:
  `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition.md`
- Create after successful analysis:
  `docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-evidence-review.md`
- Update append-only DRA CBF2026 meta/status/timeline/open-questions files.
- External immutable replay root:
  `/private/tmp/cbf2026-two-range-reacquisition-development/v1`.
- External immutable analysis root:
  `/private/tmp/cbf2026-two-range-reacquisition-analysis/v1`.

**Interfaces:**
- Consumes explicit user authorization,
  the preflight-approved protocol,
  and the C0/I0/M0 deterministic smoke gate.
- Produces one immutable `140000`-row replay,
  one compact analysis,
  an author report,
  an independent evidence review,
  and a DRA terminal record.

- [ ] **Step 1: Commit a separate authorization record**

Create
`docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-registered-authorization.json`
with exactly:

```python
AUTHORIZATION_FIELDS = (
    "schema_id",
    "protocol_id",
    "protocol_sha256",
    "protocol_commit",
    "preflight_commit",
    "smoke_commit",
    "smoke_a_compressed_sha256",
    "smoke_a_decompressed_sha256",
    "smoke_b_compressed_sha256",
    "smoke_b_decompressed_sha256",
    "smoke_analyzer_a_json_sha256",
    "smoke_analyzer_a_markdown_sha256",
    "smoke_analyzer_b_json_sha256",
    "smoke_analyzer_b_markdown_sha256",
    "smoke_semantic_payload_sha256",
    "user_authorization_date",
    "user_authorization_text",
    "user_authorization_text_sha256",
    "registered_replay_root",
    "registered_analyzer_root",
    "registered_retry_allowed",
)
```

Set:

```python
{
    "schema_id": (
        "cbf2026-two-range-reacquisition-registration-v1"
    ),
    "protocol_id": "cbf2026-two-range-reacquisition-v1",
    "registered_replay_root": (
        "/private/tmp/cbf2026-two-range-reacquisition-development/v1"
    ),
    "registered_analyzer_root": (
        "/private/tmp/cbf2026-two-range-reacquisition-analysis/v1"
    ),
    "registered_retry_allowed": False,
}
```

All authorization fields are non-null.
Schema/protocol/root/retry fields are exact literals;
all artifact hashes and the user-message hash are lowercase 64-character
SHA-256 hex strings;
all commit fields are lowercase 40-character Git OIDs;
the date is canonical ISO `YYYY-MM-DD`;
authorization text is a non-empty exact UTF-8 string whose byte hash matches
the adjacent field;
the JSON object order is exactly `AUTHORIZATION_FIELDS`.
Reject Boolean-as-integer coercion,
abbreviated OIDs,
uppercase hashes,
extra fields,
or alternate root spellings.

Compute `protocol_sha256` from the committed JSON bytes;
set `protocol_commit`,
`preflight_commit`,
and `smoke_commit` to the exact commits that first contain those artifacts;
copy the four smoke process hashes,
four compact analyzer artifact hashes,
and the independently recomputed common semantic-payload hash
from the independently reviewed terminal manifests;
set `user_authorization_date` to the authorization message's ISO date;
set `user_authorization_text` to the exact unmodified user message;
and set `user_authorization_text_sha256` to SHA-256 of that field's exact
UTF-8 bytes,
without Markdown normalization,
whitespace trimming,
or newline insertion.
The record must bind:

- user authorization message/date;
- protocol and preflight commit OIDs;
- smoke report commit and hashes;
- the committed protocol hash that contains the exact registered
  replay/analyzer argv;
- both registered roots absent;
- no retry rule acknowledged.

Validate the exact field order and hashes with registrar tests.
Commit it without modifying implementation,
schema,
threshold,
or protocol content.

```bash
git add \
  docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-registered-authorization.json
git commit -m "docs(diagnostics): authorize two-range registered run"
```

- [ ] **Step 2: Recheck registered replay preconditions**

Immediately before execution,
verify:

- clean required source files;
- exact protocol/design/comparator hashes;
- exact implementation parent;
- smoke roots unchanged;
- registered replay root absent;
- at least 8 GB free;
- production command expects exactly 140000 rows.

Any failure stops without creating a replacement protocol.

- [ ] **Step 3: Execute registered replay exactly once**

Execute `commands.registered_replay` byte-for-byte from the protocol.
Do not retry.
On success,
record terminal manifest identity,
compressed/decompressed process hashes,
row count,
allocated bytes,
and free bytes.
On failure,
retain the failed root,
record the failure,
mark the protocol terminally failed,
and stop before analysis.

- [ ] **Step 4: Recheck analyzer preconditions**

Verify:

- replay terminal status `completed`;
- exactly 140000 rows;
- raw bundle within 2 GB;
- raw path/device/inode/size/mtime/SHA pinned;
- analyzer root absent;
- at least 8 GB free;
- every comparator identity still exact.

- [ ] **Step 5: Execute registered analyzer exactly once**

Execute `commands.registered_analyzer` byte-for-byte from the protocol.
Do not retry or rebind raw evidence.
On failure,
retain the failed analysis root and mark the protocol terminally failed.

- [ ] **Step 6: Verify all denominators and gates independently**

Require:

- `140000 / 140000` exact keys;
- no missing/duplicate/extra/out-of-order row;
- all nine scientific gate records;
- fresh-retention numerator at least `124647`;
- fresh-or-predicted numerator at least `133000`;
- fresh and all-published maximum error strictly below `50 m`;
- paired both-fresh new p95 no greater than paired baseline p95;
- maximum public prediction age at most 2;
- zero reference/provenance/DAG violations;
- every additional selector-integrity gate zero;
- complete considered/branch/rejection/outage/tail accounting.

The aggregate decision is fail if any gate fails.
Do not hide a failure behind favorable conditional errors.

- [ ] **Step 7: Write the author evidence report**

Record:

- exact source/protocol/authorization commits;
- all root/manifest/process identities;
- disk and lifecycle metrics;
- exact denominators;
- every gate result;
- branch-score/rejection/outage breakdown;
- error/containment tails;
- the fixed mechanism-key outcome;
- comparison to v4;
- source input-bound audit;
- paper gate status;
- single-trajectory/outside-controller/correlation/conditional-radius limits.

- [ ] **Step 8: Conduct independent raw evidence review**

The reviewer must independently read raw/comparator streams,
recompute exact keys,
states,
branches,
q,
selection,
gates,
hashes,
and report tables.
Require C0/I0/M0 for evidence approval.
A scientifically failed gate can still have evidence integrity approved;
record those decisions separately.

- [ ] **Step 9: Commit evidence closure**

```bash
git add \
  docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition.md \
  docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-evidence-review.md
git commit -m "docs(diagnostics): record two-range evidence"
```

- [ ] **Step 10: Append terminal result to DRA main**

Record exact CBF commit/branch/upstream/worktree,
all artifact bytes/hashes,
root identities,
gate results,
review C/I/M,
paper gate,
and next authorized action.
Keep `submissions.md`,
other papers,
and dissertation records unchanged.
Review the DRA diff,
then run:

```bash
git add \
  meta-log/2026-07-30-cbf2026-predictive-wnls-recovery.md \
  papers/cbf2026/status.md \
  papers/cbf2026/timeline.md \
  papers/cbf2026/open-questions.md
git commit -m "docs(cbf2026): record two-range evidence"
```

and do not push.

- [ ] **Step 11: Stop before Stage 2 or paper changes**

Even if every estimator gate passes,
do not generate Stage 2 trajectories and do not edit `main.tex`.
Present the evidence and request a separate decision for:

1. independent multi-trajectory bounded-input confirmation; and
2. the eventual paper update.

## Plan Completion Checks

- Every approved design requirement maps to a task above.
- The only old-source behavior change is a default-disabled gate option;
  preserved callers and evidence remain unchanged.
- The new method has one pure selector,
  one producer,
  one analyzer,
  and one registrar with exact interfaces.
- All status,
  private-state,
  branch,
  schema,
  key-grid,
  gate,
  disk,
  retry,
  and comparator boundaries are explicit.
- There is no execution path from plan approval directly to the full grid:
  implementation review,
  protocol preflight,
  deterministic smoke review,
  and explicit registered-run authorization intervene.
- No paper or Stage 2 work is included.
