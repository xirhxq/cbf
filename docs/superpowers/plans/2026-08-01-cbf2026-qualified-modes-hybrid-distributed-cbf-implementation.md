# CBF2026 Qualified Modes and Hybrid Distributed CBF Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and prospectively validate a fail-closed global-mode-qualified offline localization sidecar together with a theorem-aligned hybrid distributed hard-CBF controller, then revise CBF2026 only from independently reviewed evidence.

**Architecture:** Keep global mode qualification outside the controller and keep the paper's diagonal FIM, dynamic lower-index localization DAG, two fixed distance-CBF references, coefficient-three design radius, componentwise planar input bounds, and independent local QPs. Add pure Python contracts for post-solver mode clustering and qualification; add focused C++ contracts for command-independent FIM-rate bounds, canonical edge snapshots, allocated endpoint rows, and versioned hybrid reset transactions. A new evidence namespace accounts for every declared mission, tuple, initialization, and controller interval before any result is observed, first in a development campaign and then in an exactly 60-mission confirmatory campaign.

**Tech Stack:** C++17, Eigen, doctest, nlohmann/json, Gurobi-backed local QPs, Python 3 in conda environment `cbf_env`, NumPy, Python `unittest`, gzip JSONL, SHA-256 manifests, LaTeX/BibTeX, Git.

## Global Constraints

- Source worktree: `/private/tmp/cbf2026-diagnostic`, branch `codex/cbf2026-diagnostic`, planning parent `533863ce87658895d83c3d303f1be2753253a935`.
- Approved specification: `docs/superpowers/specs/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-design.md`, 43530 bytes, SHA-256 `ced4824b0cb8b2919579469e876650aa45e71c26f1b5853ddf2633550822127b`.
- The specification's technical body was independently reviewed at pre-approval SHA-256 `bcaa312fa8bb80dbb880dee0391035b3f2dcf9044ecdc7d3f46336ab4f74bc36`; the only post-review specification change is the researcher-approval status line recorded in the review appendix.
- Approved specification review: `docs/superpowers/specs/reviews/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-design-review.md`, 5846 bytes, post-approval SHA-256 `897881d421d1bf78b4d0b1aed5dca86b574430aed0290e35f843fe6c840d6c9c`.
- Preserve the unrelated untracked `build-diagnostic/`; never stage it.
- Preserve the consumed v2 replay root `/private/tmp/cbf2026-two-range-reacquisition-development/v2` byte-for-byte and keep `/private/tmp/cbf2026-two-range-reacquisition-analysis/v2` absent.
- The consumed v2 root contains exactly two regular files; the sorted per-file SHA-256 tree commitment is `57670b00a303c4c655559097d2a80589c4cea663bc88d3033dfe30fb5fbea6cd`.
- Use a new method/protocol namespace containing `qualified-mode-hybrid-dcbf`; no v2 path, schema ID, or protocol ID may be reused.
- The offline WNLS sidecar never enters the controller state, FIM, CBF row, or QP. It supplies empirical localization evidence only.
- Runtime mode decisions may consume only current measurements, predeclared deployment-domain data, published current/past state, exact preceding applied held commands, and frozen configuration. Truth, future frames, analyzer output, source labels across modes, and realized errors are forbidden.
- Local eligibility excludes deployment-domain and history-innovation tests. The frozen order is local eligibility, all-record clustering, nonseparable-chain rejection, within-cluster representative selection, mode qualification, unique-mode publication.
- Primary mode tolerance is exactly `0.001 m`; sensitivity outputs are exactly `0.0005`, `0.001`, and `0.002 m`. A connected component with diameter above the selected tolerance rejects the complete frame.
- Exactly-two-range candidate enumeration returns both distinct circle branches and uses no live/private WNLS seed. For three or more references, enumerate the algebraic seed, every applicable pairwise-circle seed, and valid live/private seeds with deterministic 1 nm deduplication and complete raw records; mathematical local-minimum completeness remains an explicit premise, not a claim.
- Frame-zero qualification uses only the predeclared ocean-side deployment domain. Later qualification uses only the private ZOH history propagated with the actual held command.
- Public prediction age remains two frames. Private history uses the exact mission horizon `H` and `K_priv_max = H - 1`; age `H` is absent. Private state never enters FIM/WNLS.
- Keep the original diagonal marginal-covariance FIM and `epsilon_i = 3 * sqrt(lambda_max(P_i))`. This is a design radius conditional on the selected global mode, not a deterministic error bound.
- For each sUAV, use `V_i_max = sqrt(2) * 25`; for a hovering bUAV use zero. Compute `bar_nu_i` topologically before any current QP. Backward differences are descriptive fields only.
- Fixed localization edges create CBF rows; dynamic FIM-only edges create none. Every unordered sUAV collision pair creates two endpoint rows.
- Use allocation `a_ij^i = a_ij^j = 0.5` for sUAV--sUAV edges and full sUAV allocation for a hovering bUAV edge. Do not reuse `eta`, which denotes ranging noise in the paper.
- Every hard row is bound to one canonical edge ID, frame, snapshot version, allocation version, and reset transaction. Any mismatch makes the controller certificate unavailable.
- Planar commands use component bounds `-25 <= u_ix,u_iy <= 25`. Yaw and lower-level dynamics are outside the theorem and outside this plan's theoretical claim.
- No sampled-data reserve is introduced. The continuous-time proposition and the 2 Hz implementation evidence must remain explicitly separate.
- Reset transactions cover every changed node and its complete transitive descendant closure in topological order. A reset commits only when every post-reset hard barrier is nonnegative and every bounded local hard QP is feasible.
- Development uses exactly 10 independently randomized mission seeds, disjoint from confirmatory seeds. Confirmatory evidence uses exactly 60 independently generated mission trajectories and noise realizations. Each mission lasts 500 s at 2 Hz: estimator frames `0..999`, 1000 controller intervals `0..999`, frame 0 in the initialization universe, and estimator frames `1..999` in the post-initialization tuple universe.
- The full mission, initialization, estimator-tuple, and controller-interval universes are frozen before launch. Never-started and launch-failed missions remain in every applicable denominator and are unsuccessful.
- Development/confirmatory gates are exactly those in the approved specification: containment `>=0.98`, minimum-depth containment `>=0.95`, joint available-and-contained `>=0.93`, fresh retention `>=0.98`, availability `>=0.95`, controller-certificate availability `>=0.99`, primary mission success `>=0.95`, and zero listed structural/safety failures.
- Freeze implementation evidence tolerances before development: local/full hard-row residuals and post-reset barriers may be no lower than `-1e-7`; planar input components may exceed 25 only by `1e-7`; realized `nu_inst` may exceed `bar_nu` only by `1e-9`; allocation sums use `1e-12`.
- These tolerances are floating-point audit tolerances only. The analytic propositions retain exact nonnegative barriers/rows and exact component bounds.
- Stream and compress raw output. Require at least 8 GB free at launch, stop cleanly before free space falls below 6 GB, cap only reusable cache at 2 GB, and cap each compact analysis bundle at 25 MB. Raw evidentiary output has no artificial 2 GB cap; it is bounded by streaming, the 6 GB free-space stop, and the frozen schedule. Do not accumulate decompressed copies.
- Freeze per-mission supervision at a 3600 s wall-clock deadline and a 300 s no-complete-JSON-line deadline; both values are protocol fields and apply even when the child emits no newline.
- Run Python with `conda run -n cbf_env`. Use `apply_patch` for edits. Do not add `Co-Authored-By` lines.
- DRA changes occur only in `/private/tmp/dra-cbf2026-diagnostic` on `main`; update the four CBF2026 index files and the named CBF2026 theory/meta-log records only. Keep other papers and `papers/cbf2026/submissions.md` untouched. Do not push unless the researcher asks.
- Latest paper worktree: `/private/tmp/cbf2026-paper-geometric-stability`, branch `codex/cbf2026-geometric-stability-paper`, parent `d6a95736d2939728d9418ae02deadedeca822348`. Numerical claims remain unchanged until terminal confirmatory evidence passes independent review.
- Every behavior follows a vertical TDD microcycle: add one focused failing test, run it and confirm the intended assertion failure, implement only that behavior, rerun it green, then proceed. Each task ends with focused tests, regression tests, `git diff --check`, an independent review, and one scoped commit.
- Every new standalone `tests/*.cpp` doctest source begins with `#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN` before including `doctest/doctest.h`; CMake is rerun after each new file because the current glob lacks `CONFIGURE_DEPENDS`.

## Scope Decomposition Decision

The approved specification spans estimator qualification, controller realization, evidence, and paper revision. They remain in one integration plan because the primary scientific gate requires all four to share a frozen mission universe and certificate semantics. The plan nevertheless has four independently rejectable phase gates:

1. Tasks 1--4: estimator-mode and lifecycle closure;
2. Tasks 5--8: analytic-rate, allocated-row, hybrid-controller closure;
3. Tasks 9--12: development and confirmatory evidence closure; and
4. Tasks 13--15: paper, DRA, and submission-readiness closure.

No later phase may reinterpret a failed earlier gate.

Before Task 1 begins, this specification, its review, this implementation
plan, and the implementation-plan review must already be independently
reviewed and committed together in the source repository. That scoped
planning commit is the immutable execution baseline; `build-diagnostic/`
remains untracked and excluded. The plan cannot contain its own final Git blob
hash, so Task 1 obtains the committed plan identity with
`git show HEAD:docs/superpowers/plans/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-implementation.md`
and records that identity in DRA.

## File Structure

- Create `scripts/diagnostics/qualified_modes.py`: pure local-eligibility projection, permutation-invariant clustering, representative selection, deployment/history qualification, and unique-mode publication decision.
- Create `scripts/diagnostics/estimator_lifecycle.py`: shared public/private state canonicalization, exact held-command chronology, public age two, and private age `H-1` expiry without circular imports.
- Modify `scripts/diagnostics/predictive_wnls.py`: expose locally eligible solver records without applying innovation across modes; preserve existing legacy entry points.
- Modify `scripts/diagnostics/two_range_reacquisition.py`: use qualified modes, enforce exact private-age expiry, and keep the two-circle enumeration path.
- Create `scripts/diagnostics/replay_qualified_estimator.py` and `scripts/diagnostics/analyze_qualified_estimator.py`: new-namespace raw serialization and independent reconstruction; leave the v2 producer/analyzer untouched.
- Create `tests/test_qualified_modes.py`, `tests/test_estimator_lifecycle.py`, `tests/test_replay_qualified_estimator.py`, and `tests/test_analyze_qualified_estimator.py`; modify `tests/test_predictive_wnls.py` and `tests/test_two_range_reacquisition.py`.
- Create `include/cbf/FimRateCertificate.hpp`: pure fixed-active-set topological `Phi`, `P`, `epsilon`, `L_P`, and `bar_nu` calculations.
- Create `include/cbf/BarrierEdgeRegistry.hpp`: reverse fixed-localization incidence, every unordered collision edge, and zero CBF edges for dynamic FIM-only references.
- Create `include/cbf/AllocatedPairwiseCBF.hpp`: canonical edges, allocation records, endpoint rows, and independent full-row reconstruction.
- Create `include/cbf/HybridCertificateGuard.hpp`: versioned propose/validate/commit/reset transaction contracts.
- Create `tests/testFimRateCertificate.cpp`, `tests/testBarrierEdgeRegistry.cpp`, `tests/testAllocatedPairwiseCBF.cpp`, and `tests/testHybridCertificateGuard.cpp`.
- Modify `include/communicators/CommunicatorBase.hpp` and `include/communicators/CommunicatorCentral.hpp`: exchange certificate snapshot/version data, not neighbor commands for hard-row construction.
- Modify `include/Robot.hpp`: compute/store certificate inputs, build local allocated rows, apply bounded hard QPs, and serialize local certificate evidence.
- Modify `include/Swarm.hpp`: bases-first rate propagation, canonical edge registry, transitive reset coordination, and same-version commit before optimization.
- Create `include/diagnostics/EvidenceStream.hpp`: theorem-mode incremental JSONL serialization to a dedicated stdout stream while human diagnostics use stderr.
- Modify `tests/testRobotDiagnostics.cpp`, `tests/testRobustConstraintConstruction.cpp`, and `tests/testSwarmFailureHandling.cpp`; keep the historical `config/diagnostics/rbp_pairwise.json` unchanged.
- Create `config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json` and `config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json`: theorem-aligned primary arm and paired FIM-only ablation.
- Create `scripts/diagnostics/qualified_closure_evidence.py`: exact schemas, universes, keys, statuses, gates, and Wilson/bootstrap helpers.
- Create `scripts/diagnostics/run_qualified_closure_campaign.py`: development/confirmatory orchestration, absent-root allocation, seed schedules, streaming compression, disk monitoring, and terminal manifests.
- Create `scripts/diagnostics/analyze_qualified_closure_campaign.py`: independent row reconstruction, denominator accounting, structural/safety gates, confidence intervals, and compact outputs.
- Create `scripts/diagnostics/register_qualified_closure_campaign.py`: freeze protocol, code/config/seed identities, roots, authorization, and no-retry semantics.
- Create `scripts/diagnostics/generate_qualified_measurements.py`: generate one immutable noisy-range stream from frozen truth and the separately registered range-noise seed.
- Create `scripts/diagnostics/render_qualified_closure_figures.py`: render paper assets only from independently reviewed compact JSON.
- Create `tests/test_render_qualified_closure_figures.py`: hash-binding and deterministic-series selection tests for paper assets.
- Create `tests/test_qualified_closure_evidence.py`, `tests/test_run_qualified_closure_campaign.py`, `tests/test_analyze_qualified_closure_campaign.py`, and `tests/test_register_qualified_closure_campaign.py`.
- Create `tests/test_generate_qualified_measurements.py`: deterministic RNG, sigma binding, and primary/ablation shared-measurement identity tests.
- Create `tests/testEvidenceStream.cpp`: incremental/no-aggregate logger and abnormal-stop prefix tests.
- Create `tests/test_swarm_evidence_stream.py`: real `Swarm` subprocess test proving evidence-mode stdout contains JSONL only and human output is isolated on stderr.
- Create `docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3.md` and `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-review.md` after the recovery development campaign becomes terminal; the development-v1 and development-v2 result pairs are historical/non-executable and must never be created by execution.
- Create each versioned protocol, preflight, and authorization before its corresponding authorized run under the Task 10/12 lifecycle; create the terminal report and independent review only after that bundle is terminal, and permit confirmatory registration only after development-v3 PASS with C0/I0 review.
- Modify `/private/tmp/cbf2026-paper-geometric-stability/main.tex` and add only the reviewed compact result assets under `assets/2026-08-qualified-closure/` after evidence gates permit each edit.
- Modify `papers/cbf2026/status.md`, `papers/cbf2026/timeline.md`, `papers/cbf2026/sources.md`, and `papers/cbf2026/open-questions.md` in the DRA.
- Create `/private/tmp/dra-cbf2026-diagnostic/papers/cbf2026/theory/2026-08-01-qualified-global-modes-and-hybrid-distributed-cbf.md` and `/private/tmp/dra-cbf2026-diagnostic/meta-log/2026-08-01-cbf2026-qualified-mode-hybrid-dcbf.md` as append-only scientific/process records.

---

### Task 1: Planning Closure and Traceability Checkpoint

**Files:**
- Verify: `docs/superpowers/specs/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-design.md`
- Verify: `docs/superpowers/specs/reviews/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-design-review.md`
- Verify: `docs/superpowers/plans/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-implementation.md`
- Verify: `docs/superpowers/plans/reviews/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-implementation-plan-review.md`
- Modify: `/private/tmp/dra-cbf2026-diagnostic/papers/cbf2026/status.md`
- Modify: `/private/tmp/dra-cbf2026-diagnostic/papers/cbf2026/timeline.md`
- Modify: `/private/tmp/dra-cbf2026-diagnostic/papers/cbf2026/sources.md`
- Modify: `/private/tmp/dra-cbf2026-diagnostic/papers/cbf2026/open-questions.md`
- Create: `/private/tmp/dra-cbf2026-diagnostic/papers/cbf2026/theory/2026-08-01-qualified-global-modes-and-hybrid-distributed-cbf.md`
- Create: `/private/tmp/dra-cbf2026-diagnostic/meta-log/2026-08-01-cbf2026-qualified-mode-hybrid-dcbf.md`

**Interfaces:**
- Consumes: the scoped planning-baseline commit descended from `533863c` and its final committed plan identity.
- Produces: one DRA checkpoint naming exact commits, paths, hashes, constraints, and next gate; no implementation behavior.

- [ ] **Step 1: Verify the planning artifacts and immutable roots**

Run:

```bash
cd /private/tmp/cbf2026-diagnostic
git diff --check
test "$(git status --short)" = "?? build-diagnostic/"
git show HEAD:docs/superpowers/plans/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-implementation.md \
  | shasum -a 256
shasum -a 256 \
  docs/superpowers/specs/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-design.md \
  docs/superpowers/specs/reviews/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-design-review.md \
  docs/superpowers/plans/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-implementation.md \
  docs/superpowers/plans/reviews/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-implementation-plan-review.md
test -d /private/tmp/cbf2026-two-range-reacquisition-development/v2
test ! -e /private/tmp/cbf2026-two-range-reacquisition-analysis/v2
find /private/tmp/cbf2026-two-range-reacquisition-development/v2 \
  -type f -exec shasum -a 256 {} + | LC_ALL=C sort | shasum -a 256
```

Expected: the source is clean except for preserved `build-diagnostic/`; the
committed plan blob and worktree plan have the same SHA-256; the specification
and specification-review hashes match Global Constraints; the plan and its
review identities are taken from the planning commit and recorded in DRA. The
consumed replay root exists with tree commitment
`57670b00a303c4c655559097d2a80589c4cea663bc88d3033dfe30fb5fbea6cd`;
the analyzer root is absent.

- [ ] **Step 2: Append the planning checkpoint to the four DRA files**

Record exact source/paper/DRA heads, the design/review/plan absolute paths and hashes, the four phase gates, the immutable v2 paths, the 8/6/2 GB disk policy, and the fact that no implementation or experiment has yet run. Preserve historical failures and do not edit `submissions.md`.

The theory record captures the five claim layers, the approved `bar_nu`
recursion, allocated endpoint proposition, reset premises, and empirical-only
radius claim. The meta-log captures approval chronology, exact commands,
identities, failures, and future version transitions.

- [ ] **Step 3: Validate DRA scope**

Run:

```bash
cd /private/tmp/dra-cbf2026-diagnostic
git diff --check
git status --short
```

Expected paths exactly (status prefixes may be `M` or `??`):

```text
papers/cbf2026/open-questions.md
papers/cbf2026/sources.md
papers/cbf2026/status.md
papers/cbf2026/timeline.md
papers/cbf2026/theory/2026-08-01-qualified-global-modes-and-hybrid-distributed-cbf.md
meta-log/2026-08-01-cbf2026-qualified-mode-hybrid-dcbf.md
```

- [ ] **Step 4: Commit the DRA checkpoint**

```bash
git add papers/cbf2026/open-questions.md papers/cbf2026/sources.md \
  papers/cbf2026/status.md papers/cbf2026/timeline.md \
  papers/cbf2026/theory/2026-08-01-qualified-global-modes-and-hybrid-distributed-cbf.md \
  meta-log/2026-08-01-cbf2026-qualified-mode-hybrid-dcbf.md
git commit -m "docs(cbf2026): record qualified closure plan"
```

---

### Task 2: Pure Post-Solver Mode Clustering

**Files:**
- Create: `scripts/diagnostics/qualified_modes.py`
- Create: `tests/test_qualified_modes.py`
- Modify: `scripts/diagnostics/predictive_wnls.py:1094-1400`
- Test: `tests/test_predictive_wnls.py`

**Interfaces:**
- Consumes: complete solver-result dictionaries already recomputed by `candidate_acceptance`.
- Produces:
  - `MODE_TOLERANCE_M = 0.001`
  - `MODE_SENSITIVITY_M = (0.0005, 0.001, 0.002)`
  - `LocalCandidate(attempt_id: str, estimate: tuple[float, float], objective_cost: float, payload: Mapping)`
  - `project_local_candidate(attempt_id: str, result: Mapping) -> LocalCandidate | None`
  - `recompute_local_candidate_diagnostics(result: object, *, active_reference_count: int, base_anchor_provenance: object) -> dict`
  - `candidate_local_eligibility(result: object, *, active_reference_count: int, base_anchor_provenance: object) -> tuple[bool, str, dict]`
  - `enumerate_qualified_starts(references: Sequence[Mapping], *, live_seed: object, private_seed: object) -> tuple[QualifiedStart, ...]`
  - `stable_attempt_id(start: QualifiedStart) -> str`
  - `cluster_candidates(candidates: Sequence[LocalCandidate], tolerance_m: float) -> ModeClustering`
  - `canonical_mode_id(mode: NumericalMode) -> str`
  - `select_representative(mode: NumericalMode) -> LocalCandidate`
  - `sensitivity_cluster_counts(candidates: Sequence[LocalCandidate]) -> dict[str, int]`

- [ ] **Step 1: Add the failing one-mode and mirror-pair tests**

```python
class ModeClusteringTests(unittest.TestCase):
    def candidate(self, key, xy, cost=1.0):
        return LocalCandidate(key, tuple(xy), cost, {"key": key})

    def test_submillimetre_starts_form_one_mode(self):
        result = cluster_candidates([
            self.candidate("a", (0.0, 0.0)),
            self.candidate("b", (0.0008, 0.0)),
        ], 0.001)
        self.assertTrue(result.separable)
        self.assertEqual(len(result.modes), 1)

    def test_registered_mirror_pair_forms_two_modes(self):
        result = cluster_candidates([
            self.candidate("ocean", (-1490.1017, -120.0)),
            self.candidate("land", (-1609.8983, -120.0)),
        ], MODE_TOLERANCE_M)
        self.assertTrue(result.separable)
        self.assertEqual(len(result.modes), 2)
```

- [ ] **Step 2: Add a signature-only scaffold and verify assertion-level red**

After writing the tests, add only the importable constants/dataclasses and a
`cluster_candidates` scaffold that deterministically returns a separable
zero-mode result for every input. It contains no clustering behavior.

```bash
conda run -n cbf_env python -m unittest \
  tests.test_qualified_modes.ModeClusteringTests.test_submillimetre_starts_form_one_mode \
  tests.test_qualified_modes.ModeClusteringTests.test_registered_mirror_pair_forms_two_modes -v
```

Expected: the command imports and executes, then FAILS the one-mode assertion
because the scaffold returns zero modes. Record that assertion text; an import
or syntax error does not count as red.

- [ ] **Step 3: Implement immutable candidates and connected components with a diameter check**

Use these exact public types and ordering rule:

```python
@dataclass(frozen=True)
class LocalCandidate:
    attempt_id: str
    estimate: tuple[float, float]
    objective_cost: float
    payload: Mapping[str, object]

@dataclass(frozen=True)
class NumericalMode:
    member_ids: tuple[str, ...]
    members: tuple[LocalCandidate, ...]
    diameter_m: float

@dataclass(frozen=True)
class ModeClustering:
    tolerance_m: float
    separable: bool
    reason: str
    modes: tuple[NumericalMode, ...]

def _distance(a: LocalCandidate, b: LocalCandidate) -> float:
    return math.hypot(a.estimate[0] - b.estimate[0],
                      a.estimate[1] - b.estimate[1])
```

Build the undirected graph using `_distance <= tolerance_m`, sort input by `attempt_id` before traversal, sort members by `attempt_id`, compute every pairwise diameter, and return `ModeClustering(tolerance, False, "nonseparable_chain", ())` if any component diameter exceeds tolerance. Do not drop only the bad component.

- [ ] **Step 4: Add and pass chaining and permutation tests**

```python
def test_chained_component_over_diameter_rejects_complete_frame(self):
    result = cluster_candidates([
        self.candidate("a", (0.0, 0.0)),
        self.candidate("b", (0.0008, 0.0)),
        self.candidate("c", (0.0016, 0.0)),
    ], 0.001)
    self.assertFalse(result.separable)
    self.assertEqual(result.reason, "nonseparable_chain")
    self.assertEqual(result.modes, ())

def test_clustering_is_order_and_source_label_invariant(self):
    first = [self.candidate("stable-a", (0, 0)),
             self.candidate("stable-b", (2, 0))]
    second = [
        replace(self.candidate("stable-b", (2, 0)),
                payload={"source_label": "renamed-y"}),
        replace(self.candidate("stable-a", (0, 0)),
                payload={"source_label": "renamed-x"}),
    ]
    self.assertEqual(
        tuple(mode.member_ids for mode in cluster_candidates(first, .001).modes),
        tuple(mode.member_ids for mode in cluster_candidates(second, .001).modes),
    )
```

Run:

```bash
conda run -n cbf_env python -m unittest tests.test_qualified_modes -v
```

Expected: PASS.

- [ ] **Step 5: Add a new local-eligibility API without changing the legacy wrapper**

Keep the existing `candidate_acceptance` signature, three-value return, and
legacy behavior unchanged. Add a new API that receives every input required
to validate current-reference/provenance/data-fit state but deliberately has
no domain/history innovation argument:

```python
def candidate_local_eligibility(
    result: object,
    *,
    active_reference_count: int,
    base_anchor_provenance: object,
) -> tuple[bool, str, dict]:
    """Validate one local solver result without global-mode qualification."""
    diagnostics = recompute_local_candidate_diagnostics(
        result,
        active_reference_count=active_reference_count,
        base_anchor_provenance=base_anchor_provenance,
    )
    if diagnostics["failure_reason"] is not None:
        return False, diagnostics["failure_reason"], diagnostics
    return True, "locally_eligible", diagnostics

def locally_eligible_candidate(
    result: object,
    *,
    attempt_id: str,
    active_reference_count: int,
    base_anchor_provenance: object,
) -> LocalCandidate | None:
    eligible, _, diagnostics = candidate_local_eligibility(
        result,
        active_reference_count=active_reference_count,
        base_anchor_provenance=base_anchor_provenance,
    )
    if not eligible:
        return None
    return project_local_candidate(attempt_id, diagnostics["recomputed_result"])
```

Internally reuse pure validators already used by the legacy wrapper, but do not
route the qualified path through `candidate_acceptance`. Add a regression that
calls the legacy wrapper with its current full signature and confirms its
existing three-value contract is unchanged.

- [ ] **Step 6: Add a regression proving innovation cannot hide a false mode**

Create two locally data-fit candidates 120 m apart, mark one with a failing innovation diagnostic, and assert both reach `cluster_candidates`. Also assert `select_representative` uses `(objective_cost, estimate_x.hex(), estimate_y.hex(), attempt_id)` only inside one mode. `canonical_mode_id` hashes the canonical JSON encoding of sorted `(float.hex(x), float.hex(y))` member estimates; it includes no record/source label and remains unchanged under input/source permutation.

- [ ] **Step 7: Add complete deterministic start-enumeration tests**

For exactly two references, cover two roots, tangent, disjoint, contained, and
coincident-center cases and assert no algebraic/live/private start is present.
For three and four references, independently enumerate all reference pairs,
retain every applicable circle-intersection root, add the algebraic and valid
live/private seeds, deduplicate at `1e-9 m`, and assert complete stable raw
attempt IDs. Permuting references or solver-result records must leave stable
attempt IDs, representative, qualification, and publication unchanged.

Implement `stable_attempt_id` from the start kind, canonical sorted reference
keys, analytic branch orientation, and `float.hex` start coordinates; never
derive it from list index or source display label. Within a mode, the frozen
representative key is `(objective_cost, estimate_x.hex(), estimate_y.hex(),
stable_attempt_id)`, not source rank/order.

- [ ] **Step 8: Run focused and legacy regressions**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_qualified_modes tests.test_predictive_wnls -v
```

Expected: PASS, with all previous public-selection tests unchanged.

- [ ] **Step 9: Review and commit**

```bash
git diff --check
git add scripts/diagnostics/qualified_modes.py scripts/diagnostics/predictive_wnls.py \
  tests/test_qualified_modes.py tests/test_predictive_wnls.py
git commit -m "feat(estimator): cluster globally distinct solution modes"
```

---

### Task 3: Deployment/History Qualification and Fail-Closed Lifecycle

**Files:**
- Modify: `scripts/diagnostics/qualified_modes.py`
- Create: `scripts/diagnostics/estimator_lifecycle.py`
- Modify: `scripts/diagnostics/two_range_reacquisition.py:23-455`
- Modify: `tests/test_qualified_modes.py`
- Create: `tests/test_estimator_lifecycle.py`
- Modify: `tests/test_two_range_reacquisition.py`

**Interfaces:**
- Consumes: `ModeClustering`, canonical representatives, the predeclared deployment half-plane, previous private state, actual held velocity, frame index, and exact horizon `H`.
- Produces:
  - `DeploymentContract(anchor_ids: tuple[int, int], anchor_coordinates: tuple[tuple[float, float], tuple[float, float]], deployment_vertices: tuple[tuple[float, float], ...], unit_normal: tuple[float, float], offset: float, ocean_side: int, margin_m: float, domain_version: str)`
  - `ModeQualification(mode_id: str, admissible: bool, reason: str, score: float | None)`
  - `PublicationDecision(status: str, reason: str, mode_id: str | None, representative: LocalCandidate | None)`
  - `PriorBundle(public_prediction: Mapping | None, private_prior: Mapping | None, history_version: int)`
  - `qualify_deployment_mode(mode: NumericalMode, representative: LocalCandidate, domain: DeploymentContract) -> ModeQualification`
  - `qualify_history_mode(mode: NumericalMode, representative: LocalCandidate, propagated_private_prior: Mapping, innovation_limit: float) -> ModeQualification`
  - `qualify_all(modes: Sequence[NumericalMode], representatives: Sequence[LocalCandidate], qualifier_kind: str, qualifier_payload: Mapping) -> tuple[ModeQualification, ...]`
  - `publish_unique_mode(clustering, qualifications) -> PublicationDecision`
  - `advance_qualified_prior(previous_public: object, previous_private: object, held_velocity: object, *, next_frame_index: int, applied_command_frame: int | None, mission_horizon_frames: int, history_version: int) -> PriorBundle`
  - `finalize_qualified_lifecycle(decision: PublicationDecision, prior_bundle: PriorBundle, *, frame_index: int, mission_horizon_frames: int) -> dict`
  - `propagate_private_state(..., applied_command_frame: int, history_version: int, mission_horizon_frames: int) -> dict | None`

- [ ] **Step 1: Add failing cold-start domain tests**

```python
def test_ocean_side_domain_selects_one_mirror_mode(self):
    domain = DeploymentContract(
        anchor_ids=(0, 2),
        anchor_coordinates=((-1550.0, -300.0), (-1550.0, 300.0)),
        deployment_vertices=((-1491.0, -201.0), (-1369.0, -201.0),
                             (-1369.0, 201.0), (-1491.0, 201.0)),
        unit_normal=(1.0, 0.0), offset=1550.0, ocean_side=1,
        margin_m=1.0, domain_version="ocean-side-v1",
    )
    ocean = self.mode("ocean", (-1490.0, 0.0))
    land = self.mode("land", (-1610.0, 0.0))
    decisions = [qualify_deployment_mode(mode, select_representative(mode), domain)
                 for mode in (ocean, land)]
    publication = publish_unique_mode(self.clustering(ocean, land), decisions)
    self.assertEqual(publication.status, "fresh")
    self.assertEqual(publication.mode_id, canonical_mode_id(ocean))

def test_cross_line_or_invalid_domain_fails_closed(self):
    invalid = DeploymentContract(
        anchor_ids=(0, 2), anchor_coordinates=((0.0, 0.0), (1.0, 0.0)),
        deployment_vertices=((-1.0, -1.0), (1.0, 1.0)),
        unit_normal=(0.0, 0.0), offset=0.0, ocean_side=1,
        margin_m=1.0, domain_version="ocean-side-v1",
    )
    with self.assertRaises(ValueError):
        mode = self.mode("invalid", (0.0, 0.0))
        qualify_deployment_mode(mode, select_representative(mode), invalid)
```

- [ ] **Step 2: Run red, then implement strict finite half-plane qualification**

Run the new domain tests before adding `DeploymentContract`:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_qualified_modes.DeploymentQualificationTests.test_ocean_side_domain_selects_one_mirror_mode \
  tests.test_qualified_modes.DeploymentQualificationTests.test_cross_line_or_invalid_domain_fails_closed -v
```

Before this run, add only importable `DeploymentContract`/qualification
signatures whose qualifier rejects every mode. Expected: the command executes
and FAILS the `publication.status == "fresh"` assertion. Import/syntax errors
do not count as red. Implement only the strict domain behavior below and rerun
the same command to green before adding history behavior.

Recompute the line from the two distinct predeclared anchors, require the
stored normal to be finite and unit length within `1e-12`, require its
orientation to agree with the anchor ordering/domain version, and require a
finite strictly positive `margin_m`. The signed value is
`ocean_side * (unit_normal dot estimate + offset)`; a mode is admissible iff
`signed >= margin_m`. Test equality at the margin, a value just below it,
unnormalized/reversed normals, duplicate anchors, and a deployment region that
crosses the line. Every declared deployment vertex must satisfy the same
`signed >= margin_m` predicate; otherwise the entire deployment contract is
invalid before any candidate is considered.

```python
@dataclass(frozen=True)
class DeploymentContract:
    anchor_ids: tuple[int, int]
    anchor_coordinates: tuple[tuple[float, float], tuple[float, float]]
    deployment_vertices: tuple[tuple[float, float], ...]
    unit_normal: tuple[float, float]
    offset: float
    ocean_side: int
    margin_m: float
    domain_version: str

def qualify_deployment_mode(mode, candidate, domain):
    signed = domain.ocean_side * (
        domain.unit_normal[0] * candidate.estimate[0]
        + domain.unit_normal[1] * candidate.estimate[1]
        + domain.offset
    )
    admissible = signed >= domain.margin_m
    return ModeQualification(canonical_mode_id(mode), admissible,
                             "deployment_side" if admissible else "outside_deployment_side",
                             signed)
```

- [ ] **Step 3: Add failing history-qualification and incorrect-prior tests**

Assert exactly one low-innovation mode publishes, zero/two passing modes do not, and an incorrect but self-consistent private prior is reported as `wrong_unique` by the analyzer-facing truth label without changing the runtime publication reason.

```bash
conda run -n cbf_env python -m unittest \
  tests.test_qualified_modes.HistoryQualificationTests \
  tests.test_estimator_lifecycle.PrivateHistoryTests -v
```

Add importable history/lifecycle signatures that always return no admissible
mode and no propagated private state. Expected: assertion-level FAIL on the
single-low-innovation publication/age assertion, not an import error.
Implement one assertion at a time and rerun this exact focused command after
each microcycle.

- [ ] **Step 4: Implement Mahalanobis history qualification after representative selection**

Propagate private covariance once per 0.5 s held-command transition with
`Q = 0.25 I_2`. For representative covariance `P_c` and current propagated
private covariance `P^-`, compute
`q=(p_c-p^-)^T(P_c+P^-)^{-1}(p_c-p^-)`. Require both covariance inputs finite
SPD under the frozen spectral tolerance and admit exactly when
`q <= 11.829007011943707`; equality passes. Return the scalar score and
threshold reason. Do not pass private state into any WNLS call or FIM
function. Add tests at `q*`, one ULP below/above, and a covariance propagation
test that obtains `P+0.25 I_2` after one frame.

- [ ] **Step 5: Add and pass exact private-age and command-chronology tests**

```python
def test_private_age_h_minus_one_is_present_and_h_is_absent(self):
    state = reset_private_state(self.fresh_candidate(), frame_index=0)
    for frame in range(1, 10):
        state = propagate_private_state(
            state, [0.0, 0.0], next_frame_index=frame,
            applied_command_frame=frame - 1,
            history_version=frame,
            mission_horizon_frames=10,
        )
    self.assertEqual(state["age_frames"], 9)
    self.assertIsNone(propagate_private_state(
        state, [0.0, 0.0], next_frame_index=10,
        applied_command_frame=9,
        history_version=10,
        mission_horizon_frames=10,
    ))
```

The private state also stores the exact `source_fresh_frame`,
`propagated_to_frame`, `last_command_frame`, `last_held_velocity`, and
`history_version`. Reject a skipped/repeated command frame, a history-version
regression, a nonfinite held velocity, or a propagation whose
`next_frame_index != propagated_to_frame + 1`.

`estimator_lifecycle.py` is a leaf module: it may import the standard library
and NumPy but must not import `predictive_wnls`, `two_range_reacquisition`, or
controller certificate types.

- [ ] **Step 6: Freeze publication/private-state transitions**

`fresh` resets private state; `predicted`, `unavailable`, ambiguous, rejected, and nonseparable decisions retain only the exactly propagated previous private state. Public prediction expires after age two independently of private history.

- [ ] **Step 7: Add forbidden-input tests**

Pass mappings containing `truth_position`, `future_estimate`, `analyzer_label`, and `realized_error` through every runtime qualifier. Assert the accepted input schema rejects them rather than ignores them silently.

- [ ] **Step 8: Run regressions and commit**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_qualified_modes tests.test_estimator_lifecycle \
  tests.test_two_range_reacquisition \
  tests.test_predictive_wnls -v
git diff --check
git add scripts/diagnostics/qualified_modes.py \
  scripts/diagnostics/estimator_lifecycle.py \
  scripts/diagnostics/two_range_reacquisition.py \
  tests/test_qualified_modes.py tests/test_estimator_lifecycle.py \
  tests/test_two_range_reacquisition.py
git commit -m "feat(estimator): qualify modes with deployment and private history"
```

---

### Task 4: Qualified Estimator Integration and Auditable Row Schema

**Files:**
- Modify: `scripts/diagnostics/predictive_wnls.py:1403-end`
- Create: `scripts/diagnostics/replay_qualified_estimator.py`
- Create: `scripts/diagnostics/analyze_qualified_estimator.py`
- Modify: `tests/test_predictive_wnls.py`
- Create: `tests/test_replay_qualified_estimator.py`
- Create: `tests/test_analyze_qualified_estimator.py`

**Interfaces:**
- Consumes: current references, live/private start seeds, a one-start WNLS solver callback, and Task 3 publication/lifecycle state.
- Produces: one row per scheduled estimator tuple with explicit `local_candidate_count`, `mode_count`, `mode_members`, `nonseparable_chain`, `qualification_kind`, `admissible_mode_count`, `published_mode_id`, three sensitivity counts, private age, and output status.

- [ ] **Step 1: Add a failing end-to-end two-circle ambiguity test**

Use the preserved mechanism fixture and assert that, without a domain/history qualifier, both ~119.796 m-separated modes are serialized and the public status is not `fresh`.

```bash
conda run -n cbf_env python -m unittest \
  tests.test_replay_qualified_estimator.QualifiedReplayTests.test_two_circle_ambiguity_serializes_both_modes -v
```

Before the run, add an importable qualified-entry/schema scaffold that returns
an unavailable row with empty attempts/modes. Expected: assertion-level FAIL
because the two registered modes are missing; import/syntax errors do not
count as red.

- [ ] **Step 2: Replace cross-mode winner selection in the qualified entry point**

Add a new function without changing the legacy call:

```python
def solve_qualified_multistart(
    *, references, solver_from_start, live_seed, private_seed,
    qualifier_kind, qualifier_payload,
    previous_public, previous_private, held_velocity,
    frame_index, applied_command_frame, history_version,
    mission_horizon_frames, active_reference_count,
    base_anchor_provenance,
) -> dict:
    starts = enumerate_qualified_starts(
        references, live_seed=live_seed, private_seed=private_seed,
    )
    solver_results = tuple(
        {
            **solver_from_start(start),
            "start_record": start,
        }
        for start in starts
    )
    prior_bundle = advance_qualified_prior(
        previous_public, previous_private, held_velocity,
        next_frame_index=frame_index,
        applied_command_frame=applied_command_frame,
        history_version=history_version,
        mission_horizon_frames=mission_horizon_frames,
    )
    local = tuple(filter(None, (
        locally_eligible_candidate(
            result,
            attempt_id=stable_attempt_id(result["start_record"]),
            active_reference_count=active_reference_count,
            base_anchor_provenance=base_anchor_provenance,
        )
        for result in solver_results
    )))
    clustering = cluster_candidates(local, MODE_TOLERANCE_M)
    representatives = tuple(select_representative(mode) for mode in clustering.modes)
    current_qualifier_payload = dict(qualifier_payload)
    current_qualifier_payload["propagated_private_prior"] = prior_bundle.private_prior
    qualifications = qualify_all(
        clustering.modes,
        representatives,
        qualifier_kind,
        current_qualifier_payload,
    )
    decision = publish_unique_mode(clustering, qualifications)
    lifecycle = finalize_qualified_lifecycle(
        decision, prior_bundle,
        frame_index=frame_index,
        mission_horizon_frames=mission_horizon_frames,
    )
    return {
        "starts": tuple(serialize_start(start) for start in starts),
        "solver_attempts": tuple(serialize_solver_attempt(result)
                                 for result in solver_results),
        "local_candidates": tuple(serialize_candidate(candidate)
                                  for candidate in local),
        "clustering": serialize_clustering(clustering),
        "representatives": tuple(serialize_candidate(candidate)
                                 for candidate in representatives),
        "transition_inputs": serialize_transition_inputs(
            previous_public=previous_public,
            previous_private=previous_private,
            held_velocity=held_velocity,
            frame_index=frame_index,
            applied_command_frame=applied_command_frame,
            history_version=history_version,
            mission_horizon_frames=mission_horizon_frames,
        ),
        "prior_bundle": serialize_prior_bundle(prior_bundle),
        "qualifier_context": serialize_qualifier_context(
            qualifier_kind=qualifier_kind,
            qualifier_payload=qualifier_payload,
            propagated_private_prior=prior_bundle.private_prior,
        ),
        "qualifications": tuple(serialize_qualification(item)
                                for item in qualifications),
        "sensitivity": sensitivity_cluster_counts(local),
        "decision": serialize_publication_decision(decision),
        "lifecycle": lifecycle,
    }
```

`solver_from_start` is the existing one-start WNLS mechanism extracted behind
a callable boundary; it receives exactly one `QualifiedStart`. This is the
only qualified production path from references to solver records. It calls
`enumerate_qualified_starts` first, calls the solver exactly once for every
returned stable attempt, and cannot consume the legacy `initial_candidates`
winner list. Add spies proving that a two-reference frame invokes exactly both
circle starts and that a three-reference frame invokes every enumerated stable
attempt before any local-eligibility filtering or clustering.

`serialize_solver_attempt` retains every enumerated start, convergence/status,
finite recomputed estimate/covariance/objective/residual/provenance diagnostic,
and local-eligibility outcome, including failed/ineligible attempts; it rejects
truth/analyzer fields. The replay row is constructed from this returned audit
bundle, not from hidden locals. Thus the analyzer can independently reproduce
candidate completeness, clustering, representatives, qualifications,
sensitivity counts, publication, and lifecycle from the serialized row.

`serialize_transition_inputs` preserves the pre-decision public/private state,
exact held velocity, applied-command frame, next frame, history version, and
mission horizon. `serialize_prior_bundle` preserves the propagated public and
private priors before a fresh decision can overwrite them.
`serialize_qualifier_context` preserves only runtime-legal deployment/history
configuration plus that propagated private prior. All three reject truth,
future, realized-error, and analyzer labels. The independent analyzer first
recomputes `advance_qualified_prior` from transition inputs, compares the
serialized prior bundle, then recomputes every qualification and lifecycle
transition; it never attempts to recover the pre-decision prior from the
post-decision fresh state.

For frame zero, `applied_command_frame=None` is legal only when no previous
public/private state exists and the qualifier is deployment-domain. For every
later frame, it must equal `frame_index - 1`; the propagated private prior is
therefore current-frame state before any history qualification is evaluated.

- [ ] **Step 3: Define a new raw schema before connecting the qualified producer**

In `replay_qualified_estimator.py`, set a new schema ID containing
`qualified-mode-hybrid-dcbf`; do not edit or import the v2 protocol constants.
Add exact ordered schema fields and validators first. The independent analyzer
must recompute `mode_count`, every cluster diameter, representative identity,
qualification count, and publication status from serialized local records; it
must reject a row where an innovation-failing false mode was omitted.

- [ ] **Step 4: Add sensitivity and lifecycle serialization**

Serialize cluster counts at 0.5/1/2 mm, with only the 1 mm result determining `published_mode_id`. Serialize public age and private age separately. Preserve the old v2 method/schema behavior under its old invocation.

- [ ] **Step 5: Add tamper tests**

Independently mutate: source order, cluster ID, member omission, diameter, admissible count, primary tolerance, sensitivity count, private age, and published mode. Each mutation must raise a specific validation error.

- [ ] **Step 6: Run the estimator phase gate**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_qualified_modes \
  tests.test_estimator_lifecycle \
  tests.test_predictive_wnls \
  tests.test_two_range_reacquisition \
  tests.test_replay_qualified_estimator \
  tests.test_analyze_qualified_estimator \
  tests.test_replay_two_range_reacquisition \
  tests.test_analyze_two_range_reacquisition \
  tests.test_register_two_range_reacquisition -v
conda run -n cbf_env python -m py_compile \
  scripts/diagnostics/qualified_modes.py \
  scripts/diagnostics/estimator_lifecycle.py \
  scripts/diagnostics/predictive_wnls.py \
  scripts/diagnostics/two_range_reacquisition.py \
  scripts/diagnostics/replay_qualified_estimator.py \
  scripts/diagnostics/analyze_qualified_estimator.py
```

Expected: all pass; no production/full-grid command has run.

- [ ] **Step 7: Independent phase review and commit**

Review local eligibility/qualification separation, truth isolation, two-circle completeness, permutation invariance, private-age expiry, and v2 immutability.

```bash
git diff --check
git add scripts/diagnostics/qualified_modes.py \
  scripts/diagnostics/estimator_lifecycle.py \
  scripts/diagnostics/predictive_wnls.py \
  scripts/diagnostics/two_range_reacquisition.py \
  scripts/diagnostics/replay_qualified_estimator.py \
  scripts/diagnostics/analyze_qualified_estimator.py \
  tests/test_qualified_modes.py tests/test_predictive_wnls.py \
  tests/test_estimator_lifecycle.py tests/test_two_range_reacquisition.py \
  tests/test_replay_qualified_estimator.py \
  tests/test_analyze_qualified_estimator.py
git commit -m "feat(estimator): publish only one qualified global mode"
```

- [ ] **Step 8: Record the estimator phase in DRA**

Append the source commit, focused/regression test counts, review verdict,
remaining controller/evidence blockers, and the explicit statement that no
development or confirmatory campaign has run. Commit only the named CBF2026
DRA files.

---

### Task 5: Command-Independent Topological FIM-Rate Certificate

**Files:**
- Create: `include/cbf/FimRateCertificate.hpp`
- Create: `tests/testFimRateCertificate.cpp`
- Modify: `include/Robot.hpp:481-716`
- Modify: `tests/testRobotDiagnostics.cpp`

**Interfaces:**
- Consumes: one fixed-active-set snapshot in bases-first/increasing-local-index order, each node/reference local index, reference direction/distance, predecessor certificate/version, ranging variance, and component bound 25.
- Produces:
  - `PredecessorRateSnapshot`
  - `ReferenceRateInput`
  - `NodeRateInput`
  - `FrozenReferenceTerm { int referenceId; Eigen::Vector2d direction; double effectiveVariance; }`
  - `ReferenceRealizedDerivative { int referenceId; double dotEffectiveVariance; Eigen::Vector2d dotDirection; }`
  - `NodeRateCertificate`
  - `baseRateCertificate(int baseId, uint64_t version) -> NodeRateCertificate`
  - `computeNodeRateCertificate(const NodeRateInput& input) -> NodeRateCertificate`
  - `realizedEpsilonRate(const NodeRateCertificate&, const std::vector<ReferenceRealizedDerivative>&) -> double`

- [ ] **Step 1: Add the failing hovering-base certificate test**

```cpp
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

TEST_CASE("hovering base has zero covariance and epsilon rate bounds") {
    const auto base = cbf2026::baseRateCertificate(0, 7);
    CHECK(base.covarianceRateBound == doctest::Approx(0.0));
    CHECK(base.epsilonRateBound == doctest::Approx(0.0));
    CHECK(base.snapshotVersion == 7);
}
```

After writing the test, create a signature-only header whose base certificate
uses a sentinel snapshot version and negative rate. Then run red:

```bash
cmake -S . -B build-diagnostic -DBUILD_TESTING=ON
cmake --build build-diagnostic --target testFimRateCertificate -j2
./build-diagnostic/testFimRateCertificate
```

Expected: compilation succeeds and the executable FAILS the zero-rate/version
assertions. A missing-header/compiler error does not count as red. Add the pure
base case, rebuild, and run `./build-diagnostic/testFimRateCertificate` green
before proceeding.

- [ ] **Step 2: Add the pure types and base case**

```cpp
namespace cbf2026 {
struct PredecessorRateSnapshot {
    int referenceId;
    int localIndex;
    std::uint64_t snapshotVersion;
    bool hoveringBase;
    Eigen::Matrix2d covariance;
    double covarianceRateBound;
    double speedBound;
};

struct ReferenceRateInput {
    PredecessorRateSnapshot predecessor;
    double distance;
    Eigen::Vector2d direction;
    double rangingVariance;
};

struct NodeRateInput {
    int robotId;
    int localIndex;
    std::uint64_t snapshotVersion;
    double planarComponentMax;
    std::vector<ReferenceRateInput> references;
};

struct NodeRateCertificate {
    int robotId;
    std::uint64_t snapshotVersion;
    Eigen::Matrix2d information;
    Eigen::Matrix2d covariance;
    double epsilon;
    double informationRateBound;
    double covarianceRateBound;
    double epsilonRateBound;
    std::vector<FrozenReferenceTerm> frozenReferences;
};
}
```

- [ ] **Step 3: Add a hand-computed two-reference test**

For orthogonal unit directions, unit ranging variances, zero predecessor covariance/rate, distance 10, and component max 25, independently compute `w`, `beta`, `overline_dot_w`, `L_phi`, `L_P`, and `bar_nu`; compare each field to 1e-12 relative tolerance.

- [ ] **Step 4: Implement the approved recursion**

For each reference use:

```cpp
if (ref.predecessor.snapshotVersion != input.snapshotVersion
    || (!ref.predecessor.hoveringBase
        && ref.predecessor.localIndex >= input.localIndex)) {
    throw std::invalid_argument("reference is not a same-version predecessor");
}
const double w = (ref.direction.transpose()
                  * ref.predecessor.covariance * ref.direction)(0, 0)
                 + ref.rangingVariance;
const double beta = (std::sqrt(2.0) * input.planarComponentMax
                     + ref.predecessor.speedBound) / ref.distance;
const double dotW = 2.0 * beta
                    * spectralNormSymmetric(ref.predecessor.covariance)
                    + ref.predecessor.covarianceRateBound;
Lphi += dotW / std::pow(w, 2) + 2.0 * beta / w;
```

Then compute `P = Phi.inverse()`, obtain every matrix norm from the largest
absolute eigenvalue of the symmetric matrix (never Eigen's Frobenius
`.norm()`), compute `L_P = ||P||_2^2 L_phi`, and compute
`bar_nu = 3 L_P /(2 sqrt(lambda_max(P)))`. Reject nonfinite inputs, distance
at/below the frozen singular tolerance, nonpositive variance,
non-SPD/ill-conditioned `Phi`, version mismatch, and non-topological
predecessor data.

The function constructs `Phi` internally from the same `direction`, computed
`w`, and references used for the bound. It never accepts a caller-supplied
`effectiveVariance` or covariance that could disagree with those inputs.
It stores the resulting `(referenceId, direction, w)` terms in the immutable
certificate. `realizedEpsilonRate` requires exactly one same-ID derivative per
frozen term and reconstructs `dotPhi` from those terms; missing, duplicate, or
extra derivatives reject the interval.

- [ ] **Step 5: Add finite-perturbation and realized-bound property tests**

Generate deterministic fixed-active-set fixtures with seeds `11..30`; central differences at `1e-7 s` must match analytic `dotPhi` within the declared numeric tolerance, and every component-bounded realized command must satisfy `nu_inst <= bar_nu + 1e-9`.

- [ ] **Step 6: Route `bar_nu`, never backward difference, to theorem-aligned state**

Keep `uncertaintyRate` as a descriptive legacy field. Add `rateCertificate`
and `certificateAvailable` to `Robot`. In theorem-aligned mode,
`computeNodeRateCertificate` is the single source of `Phi`,
`positionCovariance`, `epsilon`, and `bar_nu`; make the old `getCovariance`
delegate to this result or remove its duplicate FIM inversion path. Assign the
certificate covariance/radius directly to the Robot state used by every edge
snapshot. Add a same-active-set regression that independently invokes the old
public covariance entry point and requires exact equality with the certificate
`P/epsilon`, so two FIM implementations cannot drift. The hard-row path reads
`rateCertificate.epsilonRateBound`; a test perturbs `previousUncertainty` and
proves the hard-row certificate is unchanged.

- [ ] **Step 7: Build, test, and commit**

```bash
cmake -S . -B build-diagnostic -DBUILD_TESTING=ON
cmake --build build-diagnostic --target testFimRateCertificate testRobotDiagnostics -j2
./build-diagnostic/testFimRateCertificate
./build-diagnostic/testRobotDiagnostics
git diff --check
git add include/cbf/FimRateCertificate.hpp include/Robot.hpp \
  tests/testFimRateCertificate.cpp tests/testRobotDiagnostics.cpp
git commit -m "feat(cbf): certify topological fim radius rates"
```

---

### Task 6: Canonical Edge Registry and Allocated Endpoint Rows

**Files:**
- Create: `include/cbf/BarrierEdgeRegistry.hpp`
- Create: `include/cbf/AllocatedPairwiseCBF.hpp`
- Create: `tests/testBarrierEdgeRegistry.cpp`
- Create: `tests/testAllocatedPairwiseCBF.cpp`
- Modify: `include/communicators/CommunicatorBase.hpp`
- Modify: `include/communicators/CommunicatorCentral.hpp`
- Modify: `include/Robot.hpp:719-1000`
- Modify: `tests/testRobustConstraintConstruction.cpp`

**Interfaces:**
- Consumes: same-version endpoint estimate/radius/rate certificates, fixed localization references, all unordered sUAV pairs, and allocation version.
- Produces:
  - `EdgeKind { Localization, Collision }`
  - `EdgeId { EdgeKind kind; int low; int high; int baseId; }`
  - `EdgeSnapshot`
  - `EndpointRow { EdgeId edge; int owner; Eigen::Vector2d coefficient; double constant; double allocation; uint64_t snapshotVersion; uint64_t allocationVersion; }`
  - `BarrierEdgeRegistry::fixedLocalizationEdges()`, `BarrierEdgeRegistry::collisionEdges()`, `BarrierEdgeRegistry::incidentEdges(int)`, `allocatedRows(...)`, and `reconstructFullRow(...)`.
  - `endpointRowToModelControl(const EndpointRow& row, int controlSize) -> Eigen::VectorXd`

- [ ] **Step 1: Add failing algebraic reconstruction tests**

Start both `testBarrierEdgeRegistry.cpp` and
`testAllocatedPairwiseCBF.cpp` with the required standalone doctest main
definition from Global Constraints.

```cpp
TEST_CASE("localization endpoint rows sum to coupled row") {
    const auto rows = cbf2026::allocatedLocalizationRows(snapshot(), 0.5, 0.5);
    REQUIRE(rows.size() == 2);
    const auto full = cbf2026::reconstructFullRow(rows);
    CHECK(full.coefficientI.isApprox(-snapshot().normal));
    CHECK(full.coefficientJ.isApprox(+snapshot().normal));
    CHECK(full.constant == doctest::Approx(
        -snapshot().nuI - snapshot().nuJ + snapshot().alpha));
}
```

Add the sign-reversed collision equivalent and a hovering-bUAV localization test with one sUAV row at allocation 1.

After writing both tests, create signature-only registry/row headers whose
public functions return empty vectors. This scaffold belongs to Task 6 and is
not staged or committed by Task 5. Then run:

```bash
cmake -S . -B build-diagnostic -DBUILD_TESTING=ON
cmake --build build-diagnostic \
  --target testBarrierEdgeRegistry testAllocatedPairwiseCBF -j2
./build-diagnostic/testBarrierEdgeRegistry
./build-diagnostic/testAllocatedPairwiseCBF
```

Expected: compilation succeeds and the executable FAILS `rows.size() == 2`.
A missing-header/compiler error does not count as red. Implement one
registry/algebra assertion at a time and rerun both executables after each
green microcycle.

- [ ] **Step 2: Implement strict canonical edge and allocation validation**

For sUAV pairs store `low=min(i,j)`, `high=max(i,j)`, `baseId=-1`; for base edges store the sUAV in `low==high` and the nonnegative base ID. Reject nonfinite/negative allocations, sums unequal to one within `1e-12`, zero separation, different snapshot/allocation versions, or duplicate owner rows.

Implement `BarrierEdgeRegistry` separately from row algebra. It consumes the
mission's two fixed references per sUAV, adds reverse incidence for a fixed
sUAV reference, emits every unordered sUAV collision pair, and never consumes
`activeLocalizationReferences` optional FIM-only additions.

- [ ] **Step 3: Implement exact endpoint formulas**

Localization owner `i`:

```cpp
row.coefficient = -normal;
row.constant = -nuI + allocationI * alphaValue;
```

Localization owner `j` uses `+normal` and `-nuJ`. Collision reverses both normal signs. `alphaValue` is computed once from the shared tightened barrier snapshot and reused by both endpoints.

- [ ] **Step 4: Add complete edge-registry and cardinality tests**

For the four-UAV fixture assert:

- each fixed sUAV--sUAV localization edge has two rows;
- each fixed sUAV--bUAV localization edge has one row;
- every unordered collision pair has two rows;
- every dynamic FIM-only reference has zero rows; and
- there are no missing/duplicate canonical IDs.

For the 14-sUAV paper topology, freeze the expected primary counts per
post-initialization interval: 28 fixed localization barriers represented by
50 allocated localization endpoint rows, 91 collision barriers represented
by 182 endpoint rows, 232 local hard-row records in total, and 119
independently reconstructed coupled barriers.

- [ ] **Step 5: Remove neighbor-command dependence**

Replace the old `otherVel` and neighbor backward-rate terms in `setFixedCommCBF`/`setSafetyCBF` only for the new theorem-aligned mode. Add a test that changes `_othersVel[otherId]` while keeping the shared snapshot fixed and asserts byte-equal endpoint coefficients/constants.

Adapt the planar endpoint coefficient to the existing three-input local model
with `endpointRowToModelControl`: require `controlSize >= 2`, copy x/y into
indices 0/1, and set yaw and every remaining coefficient exactly to zero. Add
a sign test for localization/collision rows and a test that the resulting
`VectorXd` has size 3 with coefficient 2 equal to zero.

- [ ] **Step 6: Exchange certificate snapshots and versions**

Add communicator maps for finite `epsilon`, `barNu`, `snapshotVersion`, and `allocationVersion`. Do not add a current-neighbor-command input to allocated-row construction. Missing or mismatched data returns certificate-unavailable rather than substituting zero.

- [ ] **Step 7: Add the conservatism stress fixture**

Construct one coupled row feasible with all responsibility assigned to endpoint `j`, then show fixed 1/2 allocation makes endpoint `i`'s bounded local QP infeasible. Assert the result is labeled `allocation_conservatism`, not a theorem failure or missing data.

- [ ] **Step 8: Build, test, and commit**

```bash
cmake -S . -B build-diagnostic -DBUILD_TESTING=ON
cmake --build build-diagnostic --target \
  testBarrierEdgeRegistry testAllocatedPairwiseCBF \
  testRobustConstraintConstruction -j2
./build-diagnostic/testBarrierEdgeRegistry
./build-diagnostic/testAllocatedPairwiseCBF
./build-diagnostic/testRobustConstraintConstruction
git diff --check
git add include/cbf/BarrierEdgeRegistry.hpp \
  include/cbf/AllocatedPairwiseCBF.hpp \
  include/communicators/CommunicatorBase.hpp \
  include/communicators/CommunicatorCentral.hpp include/Robot.hpp \
  tests/testBarrierEdgeRegistry.cpp tests/testAllocatedPairwiseCBF.cpp \
  tests/testRobustConstraintConstruction.cpp
git commit -m "feat(cbf): allocate canonical pairwise hard rows"
```

---

### Task 7: Versioned Hybrid Reset Guard and Distributed Integration

**Files:**
- Create: `include/cbf/HybridCertificateGuard.hpp`
- Create: `tests/testHybridCertificateGuard.cpp`
- Modify: `include/Robot.hpp:1109-1330`
- Modify: `include/Swarm.hpp:300-390`
- Modify: `tests/testSwarmFailureHandling.cpp`
- Modify: `tests/testDiagnosticConfiguration.cpp`
- Create: `scripts/diagnostics/qualified_config.py`
- Create: `tests/test_qualified_config.py`
- Create: `config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json`
- Create: `config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json`

**Interfaces:**
- Consumes: previous committed snapshot, proposed active sets/estimates/FIM/radii, complete descendant closure, canonical edges, bounded local QP feasibility checks.
- Produces:
  - `ResetCause`, `ResetProposal`, `NodeResetRecord`, `ResetTransaction`
  - `transitiveDescendants(changed, activeDag) -> std::vector<int>`
  - `validateResetTransaction(...) -> GuardDecision`
  - `Robot::buildHardConstraintProblem(const CommittedCertificateState&) const -> HardConstraintProblem`
  - `Robot::checkLocalHardQpFeasibility(const HardConstraintProblem&) const -> FeasibilityResult`
  - one atomic `commitVersion` or a certificate-unavailable reason.

- [ ] **Step 1: Add failing descendant-closure and version tests**

Start `testHybridCertificateGuard.cpp` with the required standalone doctest
main definition from Global Constraints.

For DAG `base -> 1 -> 2 -> 3` plus `base -> 4`, changing node 1 must propose exactly `[1,2,3]` in topological order. Missing node 3 or mixing version 7/8 must reject before any robot commits.

After writing the test, create a signature-only guard header whose descendant
function returns an empty vector and whose validator rejects every proposal.
This scaffold belongs to Task 7 and is not staged or committed by Task 6.
Then run:

```bash
cmake -S . -B build-diagnostic -DBUILD_TESTING=ON
cmake --build build-diagnostic --target testHybridCertificateGuard -j2
./build-diagnostic/testHybridCertificateGuard
```

Expected: compilation succeeds and the executable FAILS the expected
`[1,2,3]` descendant assertion. A missing-header/compiler error does not count
as red. Implement closure/version validation first, run the executable green,
then add the reset/QP cases in separate red/green microcycles.

- [ ] **Step 2: Implement propose/validate/commit state machine**

```cpp
enum class GuardStatus { Accepted, Rejected };
struct ResetTransaction {
    std::uint64_t predecessorVersion;
    std::uint64_t proposedVersion;
    std::vector<int> changedNodes;
    std::vector<int> descendantClosure;
    std::vector<NodeResetRecord> nodes;
    GuardStatus status;
    std::string reason;
};
```

Require `proposedVersion == predecessorVersion + 1`, one record for each descendant, bases-first order, complete same-version hard edges, and no mutation of committed robot state during validation.

- [ ] **Step 3: Add post-reset barrier and QP rejection tests**

One fixture has a negative reconstructed localization barrier; another has all barriers nonnegative but one bounded local hard QP infeasible. Both must reject atomically and preserve the predecessor version/state.

- [ ] **Step 4: Implement guard validation**

Reconstruct every fixed localization and collision barrier after the proposed
reset. The runtime commit gate requires exact computed `b >= 0.0`, every
required endpoint row present, and a feasible hard-only local QP for every
sUAV with component bounds ±25. Only then swap all proposed node snapshots
into committed state. The analyzer may separately report values down to
`-1e-7` as floating-point audit tolerance, but such tolerance never authorizes
a negative runtime reset.

Extract hard-row assembly from `Robot::optimise` into the pure
`buildHardConstraintProblem` function. `checkLocalHardQpFeasibility` creates a
fresh solver instance, adds exactly that problem's rows and component bounds,
and returns status/residuals without mutating the live optimizer, model
control, communicator maps, or committed certificate. The later normal
`optimise` call consumes the same `HardConstraintProblem`; it may add soft
task rows/slacks but may not reconstruct a different hard-row set.

Add a test that snapshots live control, optimizer status, communicator data,
and committed certificate bytes before guard precheck and proves all remain
unchanged afterward. Add a second test that hashes the guard's hard problem
and the problem consumed by `optimise` and requires equality.

- [ ] **Step 5: Integrate topological two-phase refresh in `Swarm`**

Replace the old covariance/rate broadcast sequence for theorem-aligned mode with:

1. collect active-set/estimate changes;
2. compute the complete descendant closure;
3. propose all node FIM/rate certificates in topological order;
4. build same-version edges and endpoint rows;
5. validate barriers and bounded hard QPs;
6. atomically commit and broadcast; and
7. solve each robot's normal local QP using the committed rows.

If validation fails without a valid predecessor certificate, terminate the theorem-aligned mission segment and log the exact reason. A zero-command fallback is descriptive only.

- [ ] **Step 6: Freeze new-namespace configuration and input bounds**

Leave historical `rbp_pairwise.json` byte-identical. Create the new primary
overlay with:

First add strict Python configuration tests and run them red:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_qualified_config.QualifiedConfigTests -v
```

Before the run, add an importable validator scaffold that returns `False` for
every configuration and the two syntactically valid empty overlay files.
Expected: assertion-level FAIL because the registered primary is not accepted;
an import/parse error does not count as red. Implement the primary schema,
rerun green, then add the one-key ablation-difference test as the next
red/green microcycle.

```json
{
  "qualified-estimator": {
    "mode-tolerance-m": 0.001,
    "sensitivity-tolerances-m": [0.0005, 0.001, 0.002],
    "deployment": {
      "anchor-ids": [0, 2],
      "anchor-coordinates": [[-1550.0, -300.0], [-1550.0, 300.0]],
      "deployment-vertices": [[-1490.0, -200.0], [-1370.0, -200.0],
                              [-1370.0, 200.0], [-1490.0, 200.0]],
      "unit-normal": [1.0, 0.0],
      "offset": 1550.0,
      "ocean-side": 1,
      "margin-m": 1.0,
      "domain-version": "ocean-side-v1"
    },
    "history": {
      "q-threshold": 11.829007011943707,
      "process-noise-diagonal": [0.25, 0.25],
      "public-max-age-frames": 2,
      "private-max-age-policy": "mission-frames-minus-one"
    }
  },
  "position_covariance": {
    "reference-selection": "dynamic-lower-index",
    "ranging_sigma": 0.5,
    "singular-distance-tolerance-m": 1e-8,
    "relative-spectral-threshold": 1e-12
  },
  "cbfs": {
    "uncertainty-rate": {"mode": "analytic-topological"},
    "input-limits": {"on": true, "planar-component-max": 25.0, "yaw-rate-max": 0.35},
    "without-slack": {
      "safety": {"on": true, "mode": "allocated-pairwise"},
      "comm-fixed": {"on": true, "mode": "allocated-pairwise"}
    }
  },
  "execute": {"execution-mode": "distributed", "time-step": 0.5}
}
```

The yaw bound remains an implementation limit, not part of the planar theorem.
The paired ablation changes only dynamic FIM reference selection to the fixed
reference set by setting `position_covariance.reference-selection` to
`fixed-cbf-only`; every other scientific field is byte-equivalent after that
single-key projection. It reuses the primary truth/noise inputs and never
changes the primary controller trajectory.

Implement `validate_qualified_config` in Python and equivalent strict C++
startup checks. Reject absent/defaulted fields, reordered sensitivity values,
nonunit normals, inconsistent anchors/offset, deployment vertices below the
margin, nonpositive tolerances, any q/Q/age mismatch, unknown reference
selection, input bound other than 25, or non-2 Hz time step. Tests validate
both configs and prove their only allowed semantic difference is the
reference-selection value.

- [ ] **Step 7: Add lifecycle/no-Zeno instrumentation tests**

Serialize reset time/version/cause and assert versions are strictly increasing, no two accepted resets share the same simulation timestamp, and the finite 2 Hz mission schedule admits at most one transaction per frame. Describe this as an implementation anti-chatter check; the continuous proposition still lists locally finite resets as a premise.

- [ ] **Step 8: Run the controller phase gate**

```bash
cmake -S . -B build-diagnostic -DBUILD_TESTING=ON
cmake --build build-diagnostic --target \
  testFimRateCertificate testBarrierEdgeRegistry testAllocatedPairwiseCBF \
  testHybridCertificateGuard testRobotDiagnostics \
  testRobustConstraintConstruction testSwarmFailureHandling \
  testDiagnosticConfiguration -j2
./build-diagnostic/testFimRateCertificate
./build-diagnostic/testBarrierEdgeRegistry
./build-diagnostic/testAllocatedPairwiseCBF
./build-diagnostic/testHybridCertificateGuard
./build-diagnostic/testRobotDiagnostics
./build-diagnostic/testRobustConstraintConstruction
./build-diagnostic/testSwarmFailureHandling
./build-diagnostic/testDiagnosticConfiguration
conda run -n cbf_env python -m unittest tests.test_qualified_config -v
ctest --test-dir build-diagnostic --output-on-failure
```

Expected: all C++ tests pass with Gurobi available; hard rows contain no neighbor-command predictor or backward-difference certificate.

- [ ] **Step 9: Independent controller review and commit**

```bash
git diff --check
git add include/cbf/HybridCertificateGuard.hpp include/Robot.hpp include/Swarm.hpp \
  scripts/diagnostics/qualified_config.py \
  tests/testHybridCertificateGuard.cpp tests/testSwarmFailureHandling.cpp \
  tests/testDiagnosticConfiguration.cpp tests/test_qualified_config.py \
  config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json \
  config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json
git commit -m "feat(cbf): guard hybrid distributed certificate resets"
```

- [ ] **Step 10: Record the controller phase in DRA**

Append the analytic-rate, edge-cardinality, allocation-conservatism,
transaction, and regression results with exact source/review identities.
Keep development and confirmatory gates closed.

---

### Task 8: Complete Raw Controller Evidence and Independent Reconstruction

**Files:**
- Create: `scripts/diagnostics/qualified_closure_evidence.py`
- Create: `tests/test_qualified_closure_evidence.py`
- Create: `include/diagnostics/EvidenceStream.hpp`
- Create: `tests/testEvidenceStream.cpp`
- Create: `tests/test_swarm_evidence_stream.py`
- Modify: `include/Robot.hpp:1430-1490`
- Modify: `include/Swarm.hpp:119-155`
- Modify: `src/Swarm.cpp`

**Interfaces:**
- Consumes: estimator mode row, committed controller snapshot, reset transaction, local QP status, applied input, true analyzer-only state, and frozen mission schedule.
- Produces: exact keys and schemas for initialization, estimator tuple, controller interval, edge row, reset, and mission terminal records, plus:
  - `FrozenMissionSchedule(campaign_id: str, trajectory_seed: int, range_noise_seed: int, frames: int, robots: tuple[int, ...], estimator_conditions: tuple[str, ...], controller_condition: str)`
  - `synthesize_missing_mission(schedule: FrozenMissionSchedule, reason: str) -> MissingMissionRows`
  - `EvidenceStream::write(const json& row)` and `EvidenceStream::flush()` for one validated JSON object per stdout line.

- [ ] **Step 1: Add failing universe-construction tests**

```python
def test_launch_failed_mission_contributes_every_frozen_key(self):
    schedule = FrozenMissionSchedule(
        campaign_id="development-v1",
        trajectory_seed=101,
        range_noise_seed=201,
        frames=1000,
        robots=tuple(range(1, 15)),
        estimator_conditions=("dynamic_primary", "fixed_fim_ablation"),
        controller_condition="dynamic_primary",
    )
    rows = synthesize_missing_mission(schedule, reason="launch_failed")
    self.assertEqual(len(rows.estimator), 2 * 999 * 14)
    self.assertEqual(len(rows.initialization), 14)
    self.assertEqual(len(rows.controller), 1000)
    self.assertFalse(rows.mission.success)
```

```bash
conda run -n cbf_env python -m unittest \
  tests.test_qualified_closure_evidence.EvidenceUniverseTests.test_launch_failed_mission_contributes_every_frozen_key -v
```

Before the run, add importable schedule/result dataclasses and a synthesis
scaffold that returns empty row tuples. Expected: assertion-level FAIL on the
frozen estimator-row cardinality; an import error does not count as red.
Implement only schedule/key synthesis and rerun this focused test green before
adding the C++ stream and reconstruction contracts.

- [ ] **Step 2: Implement exact ordered universe keys**

Use `(campaign_id, trajectory_seed, range_noise_seed, 0, robot)` for the
single shared initialization/domain universe; use `(campaign_id, condition,
trajectory_seed, range_noise_seed, frame, robot)` for post-initialization
estimator rows; use `(campaign_id, "dynamic_primary",
trajectory_seed, range_noise_seed, interval)` for controller,
`(campaign_id, "dynamic_primary", trajectory_seed, range_noise_seed, frame,
edge_id, owner)` for endpoint rows, the same key without owner plus edge ID for
reconstructed rows, `(campaign_id, "dynamic_primary", trajectory_seed,
range_noise_seed, frame, reset_version)` for reset records, and
`(campaign_id, trajectory_seed, range_noise_seed)` for mission records.
Validate cardinality from the frozen schedule, not observed output. Primary
and fixed-FIM-ablation estimator denominators are computed separately; the
ablation has no controller interval, edge, reset, or mission-success rows.

- [ ] **Step 3: Add assertion-red stream/schema tests, then serialize controller fields**

Before modifying `Robot`/`Swarm`, write `testEvidenceStream.cpp`, the focused
controller-primitive schema test, and the real two-frame Swarm stdout test.
Create only a signature-compatible `EvidenceStream` scaffold whose `write`
discards rows and whose `flush` is a no-op; add the new CMake-discovered test
source and reconfigure. Run:

```bash
cmake -S . -B build-diagnostic -DBUILD_TESTING=ON
cmake --build build-diagnostic --target Swarm testEvidenceStream -j2
./build-diagnostic/testEvidenceStream
conda run -n cbf_env python -m unittest \
  tests.test_qualified_closure_evidence.ControllerPrimitiveSchemaTests \
  tests.test_swarm_evidence_stream.SwarmEvidenceStdoutTests -v
```

Expected: C++ executes and FAILS the three-line/prefix assertion because the
scaffold emits nothing; after the stream itself is implemented, the Python
schema/real-Swarm tests execute and FAIL their missing-primitives/stdout
assertions before `Robot`/`Swarm` routing is implemented. Import, compile, or
fixture-launch errors do not count as red. Make each focused assertion green
before advancing to the next behavior.

`testEvidenceStream.cpp` writes three rows, flushes after each, and verifies
that every prefix ending at a newline parses independently even when no
mission-terminal row follows. This permits the Python supervisor to close its
gzip stream and synthesize missing scheduled rows after a child crash. The
file begins with the required standalone doctest main definition.
`test_swarm_evidence_stream.py` launches the real `build-diagnostic/Swarm`
with a two-frame evidence fixture, parses every nonempty stdout line with
`json.loads`, and asserts progress/terminal prose is present only on stderr.

Each interval records version, complete finite snapshot flag, per-node
`P/Pdot/epsilon/Dplus_epsilon/nu_inst/bar_nu`, edge counts,
local/reconstructed residual minima, guard status, all QP statuses, applied
component maxima, abort reason, and completion state. Every local row records
canonical edge and allocation versions.

The raw interval—not a trusted aggregate—must also contain every input needed
for independent reconstruction. For every active FIM reference record its
canonical reference ID, direction, distance, ranging variance, predecessor
local index, predecessor snapshot version, predecessor covariance `P_j`,
predecessor covariance-rate bound `L_j^P`, predecessor speed bound, and the
node component bound. For every canonical hard edge record its kind/endpoints,
shared normal, tightened barrier value, alpha value, endpoint coefficient,
constant, owner, allocation, endpoint snapshot/allocation versions, the
applied owner command, and—for a two-sUAV edge—the independently published
other endpoint command joined only by the analyzer. These primitives are the
sole inputs used to reconstruct `Phi`, `P`, rates, endpoint residuals, and the
coupled row; serialized derived booleans are comparison fields only.

Every reset record includes trigger/cause, predecessor/proposed version,
changed nodes and full descendant closure, pre/post active sets, per-node
`delta_p` and `delta_epsilon`, every incident edge's `b_minus/b_plus`,
allocations and row versions, hard-QP feasibility/status, final guard decision,
and commit/abort outcome. Truth appears only in the analyzer projection, where
post-reset true error versus the proposed radius is reported as empirical
premise satisfaction; it is never a runtime guard input.

In theorem evidence mode, replace `Swarm::logOnce` accumulation into
`data["state"]` with `EvidenceStream::write` followed by a flush. Evidence
JSONL is the only stdout content; route progress/human messages to stderr.
Legacy non-evidence runs retain their existing aggregate JSON behavior.
Audit every print in `src/Swarm.cpp` and `include/Swarm.hpp`; evidence mode
must send elapsed-time, progress, termination, failure, and summary text to
stderr. No non-JSON byte may reach stdout before, between, or after evidence
rows.

- [ ] **Step 4: Add assertion-red reconstruction tests, then implement independent algebra**

Add a complete valid raw fixture with deliberately incorrect serialized
`Phi/P/nu/residual` fields and assert the analyzer returns the independently
reconstructed values, not the serialized derived fields. Initially add
signature-only reconstruction functions returning the serialized values and
run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_qualified_closure_evidence.IndependentReconstructionTests -v
```

Expected: assertion-level FAIL on the deliberately corrupted derived value.
An import/schema error does not count as red. Then implement the independent
algebra below and rerun green.

The Python analyzer recomputes `Phi`, `P`, `epsilon`, the upper Dini/eigenvalue
directional derivative `Dplus_epsilon`, realized `nu_inst`, `bar_nu`,
allocations, local residuals, reconstructed full residuals, reset deltas and
barriers, cardinalities, and guard verdict from raw state/command fields. It
must verify `Dplus_epsilon <= nu_inst + 1e-9` and
`nu_inst <= bar_nu + 1e-9`; it must not trust the C++ booleans.

- [ ] **Step 5: Add denominator and zero-denominator tests**

Verify conditional containment uses only finite available outputs, joint containment uses the full tuple universe, every registered depth has a positive denominator, controller availability uses the full interval universe, and mission success includes exactly every declared seed.

- [ ] **Step 6: Add tamper and failure-retention tests**

Mutate/miss one snapshot version, edge row, reset descendant, input component, QP status, estimator tuple, mission terminal row, and launch record. Each mutation must fail integrity or reduce the relevant gate denominator; none may disappear.

- [ ] **Step 7: Run evidence tests and commit**

```bash
cmake -S . -B build-diagnostic -DBUILD_TESTING=ON
cmake --build build-diagnostic --target Swarm testEvidenceStream -j2
./build-diagnostic/testEvidenceStream
conda run -n cbf_env python -m unittest \
  tests.test_qualified_closure_evidence \
  tests.test_swarm_evidence_stream -v
conda run -n cbf_env python -m py_compile \
  scripts/diagnostics/qualified_closure_evidence.py
git diff --check
git add scripts/diagnostics/qualified_closure_evidence.py \
  include/diagnostics/EvidenceStream.hpp include/Robot.hpp include/Swarm.hpp \
  src/Swarm.cpp tests/testEvidenceStream.cpp \
  tests/test_qualified_closure_evidence.py tests/test_swarm_evidence_stream.py
git commit -m "feat(evidence): reconstruct hybrid distributed certificates"
```

---

### Task 9: Versioned Development Campaign Runner and Analyzer

**Files:**
- Verify: `config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json`
- Verify: `config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json`
- Create: `scripts/diagnostics/run_qualified_closure_campaign.py`
- Create: `scripts/diagnostics/analyze_qualified_closure_campaign.py`
- Create: `scripts/diagnostics/register_qualified_closure_campaign.py`
- Create: `scripts/diagnostics/generate_qualified_measurements.py`
- Create: `tests/test_run_qualified_closure_campaign.py`
- Create: `tests/test_analyze_qualified_closure_campaign.py`
- Create: `tests/test_register_qualified_closure_campaign.py`
- Create: `tests/test_generate_qualified_measurements.py`

**Interfaces:**
- Consumes: clean reviewed binary/source identity, `config/config.json` base, primary/ablation overlays, disjoint seed lists, frozen schedules, and output root.
- Produces: streamed gzip JSONL, terminal manifest, compact JSON/Markdown analysis, and nonzero process return on any lifecycle failure.

- [ ] **Step 1: Add failing absent-root and disk-policy tests**

Assert launch rejects an existing root, less than 8 GB start space, a symlinked root, or a root inside the repo. During a fake run, crossing 6 GB free space must publish a terminal failure manifest and stop the child. A separate reusable-cache fixture must stop before its cache exceeds 2 GB; raw evidentiary bytes are not classified as cache.

```bash
conda run -n cbf_env python -m unittest \
  tests.test_run_qualified_closure_campaign.RunnerRootTests \
  tests.test_run_qualified_closure_campaign.RunnerDiskPolicyTests -v
```

Before the run, add an importable runner scaffold whose root/disk validator
accepts every request and performs no allocation. Expected: assertion-level
FAIL because an existing root is not rejected; an import error does not count
as red. Implement absent-root allocation first and rerun that class green;
then add disk monitoring with a separate red/green cycle. Repeat the same
signature-scaffold plus assertion-red pattern for analyzer, registrar, and
measurement-generator tests before implementing each behavior.

- [ ] **Step 2: Implement development root allocation**

The exact first raw root is
`/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v1` and the exact
analysis root is
`/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v1`.
Before execution, require both absent. The runner atomically claims only the
raw root after source/config/seed/preflight identities are recorded. The
analyzer does not run until the raw manifest is terminal; it then independently
rechecks that the analysis root is absent and atomically claims only the
analysis root. Neither process pre-creates the other's root. Never delete or
retry the same version.

Recovery note (2026-08-02): the v1 root allocation above is preserved as the
original frozen design but was never executed. Development-v2 also failed its
postcommit authorization gate before either v2 root was allocated. Both are
terminally superseded. The implemented development allocation constants now
require the non-colliding
`/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v3` and
`/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v3`
roots recorded in Tasks 10--11.

- [ ] **Step 3: Freeze development seeds and schedules**

Use exactly trajectory seeds `2026080101..2026080110` and separately serialized
range-noise seeds `2026081101..2026081110` for v1. Each pair generates its own
random-in-polygon initial condition and measurement-noise stream for a
1000-frame, 500 s mission at 2 Hz. Write the full ten-mission schedule before
launching mission 1; do not derive the noise seed implicitly from the
trajectory seed at runtime.

Treat both Task 7 JSON files as overlays, never complete Swarm configs. For
each primary mission, call the existing `deep_merge` semantics on
`config/config.json` plus the primary overlay, then set only the registered
trajectory seed, `time-total=500.0`, mission-local output identity, and
evidence-stream mode. Write `config.materialized.json` inside the raw mission
bundle before launch, run Python strict validation, and let Swarm run the same
C++ validation at startup. Record separate SHA-256 identities for base,
overlay, and every materialized mission config. The fixed-FIM overlay is
merged only for offline replay and is never passed to Swarm.

Run exactly one primary `Swarm` process per mission. It streams the dynamic
DAG controller trajectory and analyzer-only truth. After that trajectory is
terminal, `generate_qualified_measurements.py` reads the committed truth
stream once, uses `numpy.random.default_rng(range_noise_seed)` and the exact
configured `ranging_sigma=0.5`, and writes two physically separate immutable
streams. The runtime measurement JSONL.gz contains only endpoint/frame keys,
noisy range, current reference/provenance fields, sigma/config identity, and
measurement-stream identity. The analyzer-only audit JSONL.gz contains the
matching truth/noiseless range, sampled noise, RNG identity, and audit key.
Each has its own manifest and hash.

The runner then invokes `replay_qualified_estimator.py` twice as producer:
`condition=dynamic_primary` with dynamic lower-index references and
`condition=fixed_fim_ablation` with fixed CBF references. Both consume the
same runtime measurement file identity and the same exact held-command/ZOH
command history; each condition owns and evolves an independent
public/private estimator publication/lifecycle history from its own selected
branches. Neither condition may read the other's publication, prior, branch,
or status. The immutable measurement stream covers the union of every
role-tagged reference edge requested by either registered condition at every
frame; the fixed-CBF edge set is validated as a subset of that union. Their
argv and file descriptors contain no truth/audit path. Neither runs a controller.
Their estimator JSONL.gz/manifest files become immutable members
of the raw bundle before its terminal campaign manifest is published. The
campaign analyzer only reconstructs/validates/aggregates these producer rows;
it never generates noise, selects modes, or produces method outputs.

- [ ] **Step 4: Implement one mission at a time with supervised stdout-to-gzip streaming**

Refactor only reusable read-only helpers from `run_diagnostic`; the new runner
owns a generic subprocess supervisor. Launch `Swarm` with
`stdout=subprocess.PIPE` and a separate stderr log. For each complete stdout
line, validate that it is one JSON object and write those exact UTF-8 bytes
through `gzip.GzipFile(..., mode="wb", mtime=0)`. Poll free/allocated bytes
between lines. On malformed output, child error, signal, or disk stop, close
the gzip member normally, retain the valid prefix, publish the failure
manifest, and synthesize every missing scheduled row plus one unsuccessful
mission record. Never hold the mission or campaign JSON population in memory.

Use a selector or reader-thread/queue so disk polling, the 3600 s wall-clock
deadline, and the 300 s complete-line stall deadline continue while the child
is silent or has written only a partial line. A deadline sends a graceful
termination first, escalates after the frozen grace period, closes gzip, and
publishes `wallclock_timeout` or `line_stall_timeout`.

Add fake-child tests for normal rows, malformed row 3, nonzero exit after row
2, signal after row 2, a child that never writes/exits, a child that writes
only half a JSON line, 6 GB stop, and 2 GB reusable-cache cap. In every failure case the gzip
prefix must decompress, the manifest must be terminal, and the frozen key
universe must remain complete after synthesis.

Add measurement/producer orchestration tests that prove identical input
measurement SHA-256 in both estimator conditions, deterministic regeneration
for the same range seed, different measurements for a different range seed,
and analyzer rejection when either condition substitutes or regenerates a
measurement. Inspect/monkeypatch producer argv and file-open calls to prove it
cannot access truth, noiseless range, sampled noise, or analyzer-only audit
paths; wrong-mode/error labels arise only when the independent analyzer joins
the audit stream after publication.

Also assert no public/private lifecycle object identity crosses conditions,
changing the primary publication history cannot change the fixed-FIM replay,
and every condition-requested role-tagged edge exists in the shared union
measurement stream exactly once per scheduled key.

- [ ] **Step 5: Implement all approved gates**

`analyze_qualified_closure_campaign.py` must emit numerator, denominator,
comparison, threshold, and pass/fail for: complete keys/manifests; zero runtime
truth reads; zero initialization-accounting omissions; zero cross-mode
source-order publication; zero fresh publication with multiple admissible
modes; zero wrong-mode fresh publication; zero finite fresh/predicted error
above 50 m; aggregate containment at least 0.98; every registered nonempty
depth containment at least 0.95; full-universe joint available-and-contained
at least 0.93; fresh retention at least 0.98; fresh-or-bounded-predicted
availability at least 0.95; zero missing/duplicate hard rows; controller
certificate availability at least 0.99; zero `nu_inst > bar_nu + 1e-9`;
zero accepted reset violations; zero input-limit violations beyond `1e-7`;
zero primary local hard-QP infeasibility; zero local/reconstructed residual
below `-1e-7`; zero true localization/collision distance violation in a
successful mission; and successful mission fraction at least 0.95 over all
ten declared missions. It also emits max/p95/p99/depth errors, private-age
strata, reset reasons, certificate failures, allocation-conservatism stress
outcomes, deadlocks, and incomplete missions.

Stage compact JSON/Markdown outputs under no-replace temporary names, measure
allocated bytes before publication, and fail terminally if their combined
allocated size exceeds 25 MB. Only after schema/hash/size validation may the
analyzer atomically publish both files and its manifest; an oversized bundle
produces a compact terminal-failure manifest and no partial result.

- [ ] **Step 6: Add deterministic synthetic campaign tests**

Create one all-pass two-mission in-memory fixture and separate fixtures for launch failure, wrong unique mode, missing depth, catastrophic error, missing edge, `nu_inst > bar_nu`, negative residual, infeasible QP, input violation, reset violation, true distance violation, and incomplete mission. Each must fail the exact intended gate.
Add compact-writer fixtures just below and just above 25 MB; the first
publishes atomically and the second produces only the terminal failure
manifest.

- [ ] **Step 7: Implement the production registrar before implementation freeze**

Add exact source/config/binary/dependency/schema/threshold/seed/root bindings,
60 distinct trajectory/range seed pairs, development-seed disjointness,
the deterministic confirmatory smoke schedule, absent-root checks,
authorization binding, and no-retry semantics. The smoke schedule is exactly
one 20-frame/10 s mission with trajectory seed `2026089001` and range-noise
seed `2026089101`, reused byte-for-byte by smoke A and B, disjoint from every
development and confirmatory seed, and excluded from every scientific
denominator. Tests cover
59/61 missions, duplicate/colliding seeds, dirty relevant source, existing or
symlinked roots, threshold/config mismatch, and mutation after registration.
Registrar tests may use temporary roots only; this task does not generate a
production protocol.

- [ ] **Step 8: Run runner/analyzer/registrar regressions and commit**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_run_qualified_closure_campaign \
  tests.test_analyze_qualified_closure_campaign \
  tests.test_register_qualified_closure_campaign \
  tests.test_generate_qualified_measurements \
  tests.test_qualified_closure_evidence \
  tests.test_run_diagnostic tests.test_analyze_diagnostic -v
conda run -n cbf_env python -m py_compile \
  scripts/diagnostics/run_qualified_closure_campaign.py \
  scripts/diagnostics/analyze_qualified_closure_campaign.py \
  scripts/diagnostics/register_qualified_closure_campaign.py \
  scripts/diagnostics/generate_qualified_measurements.py
git diff --check
git add scripts/diagnostics/run_qualified_closure_campaign.py \
  scripts/diagnostics/analyze_qualified_closure_campaign.py \
  scripts/diagnostics/register_qualified_closure_campaign.py \
  scripts/diagnostics/generate_qualified_measurements.py \
  tests/test_run_qualified_closure_campaign.py \
  tests/test_analyze_qualified_closure_campaign.py \
  tests/test_register_qualified_closure_campaign.py \
  tests/test_generate_qualified_measurements.py
git commit -m "feat(experiments): add qualified closure development campaign"
```

---

### Task 10: Full Implementation Verification and Review Freeze

**Files:**
- Create: `docs/diagnostics/2026-08-01-cbf2026-qualified-closure-implementation.md`
- Create: `docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-implementation-review.md`
- Historical failed v1: `docs/diagnostics/2026-08-01-cbf2026-qualified-closure-development-v1-protocol.json`
- Historical failed v1: `docs/diagnostics/2026-08-01-cbf2026-qualified-closure-development-v1-protocol.md`
- Historical failed v1: `docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-development-v1-preflight.md`
- Historical absent v1: `docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-development-v1-authorization.json`
- Historical failed v2: `docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.json`
- Historical failed v2: `docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.md`
- Historical failed v2: `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-preflight.md`
- Historical failed v2: `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-authorization.json`
- Historical failed v2: `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-postcommit-validation.md`
- Recovery v3 generate: `docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.json`
- Recovery v3 generate: `docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.md`
- Recovery v3 create: `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-preflight.md`
- Recovery v3 create: `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-authorization.json`
- Modify: DRA four CBF2026 files from Task 1.

**Interfaces:**
- Consumes: Tasks 2--9 code and tests.
- Produces: one reviewed clean implementation commit approved only for the
  development-v3 registration/preflight/authorization lifecycle. Execution
  remains closed until the exact four-artifact direct-child commit below is
  independently validated.

- [ ] **Step 1: Run complete Python discovery**

```bash
conda run -n cbf_env python -m unittest discover -s tests -p 'test_*.py' -v
```

Expected: zero failures/errors/skips attributable to the changed contracts.

- [ ] **Step 2: Run complete C++ suite**

```bash
cmake -S . -B build-diagnostic -DBUILD_TESTING=ON
rg '^ENABLE_GUROBI:BOOL=ON$' build-diagnostic/CMakeCache.txt
cmake --build build-diagnostic -j2
./build-diagnostic/testFimRateCertificate
./build-diagnostic/testBarrierEdgeRegistry
./build-diagnostic/testAllocatedPairwiseCBF
./build-diagnostic/testHybridCertificateGuard
./build-diagnostic/testEvidenceStream
./build-diagnostic/testRobustConstraintConstruction
./build-diagnostic/testRobotDiagnostics
./build-diagnostic/testDiagnosticConfiguration
./build-diagnostic/testSwarmFailureHandling
./build-diagnostic/testOptimisers
ctest --test-dir build-diagnostic --output-on-failure
```

Expected: Gurobi is explicitly enabled and zero tests fail, including all five new pure-contract test executables.
Explicit execution is mandatory because the current CMake file builds each
`tests/*.cpp` source but registers only `testSwarmFailureExit` with CTest.

- [ ] **Step 3: Run static/provenance checks**

```bash
git diff --check
rg -n "truth_position|future_estimate|realized_error" \
  scripts/diagnostics/qualified_modes.py \
  scripts/diagnostics/two_range_reacquisition.py
rg -n "_othersVel|positiveBackwardUncertaintyRate" \
  include/cbf/AllocatedPairwiseCBF.hpp include/cbf/HybridCertificateGuard.hpp
test ! -e /private/tmp/cbf2026-two-range-reacquisition-analysis/v2
```

Expected: forbidden fields appear only in explicit rejection/tests; allocated hard-row code contains neither neighbor velocity nor backward difference.

- [ ] **Step 4: Write the implementation report**

Record exact commits, changed files, tests/counts, configuration identities, known conservatism, continuous/2 Hz claim separation, immutable roots, and the exact development command. State that no development or confirmatory result exists yet.

- [ ] **Step 5: Dispatch independent specification/conformance review**

The reviewer checks all ten architecture contracts, every required test, v2 immutability, denominator construction, truth isolation, fail-closed paths, and disk/no-retry behavior. Resolve every Critical/Important issue by a new red/green cycle; do not waive it in prose.

- [ ] **Step 6: Commit reports and update DRA**

```bash
git add docs/diagnostics/2026-08-01-cbf2026-qualified-closure-implementation.md \
  docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-implementation-review.md
git commit -m "docs(cbf2026): approve qualified closure implementation"
```

- [x] **Historical Step 7: Generate the exact development-v1 protocol from the reviewed implementation identity**

This command is retained only as the exact record of the failed-v1 lifecycle.
It was run against implementation/report archive commit
`709f4ef6cef60b0754527fc62d8ca64ac7a88a12`; it is superseded and must never
be run again.

Run the registrar in development mode after the implementation report/review
commit exists. Bind that exact code tree, `build-diagnostic/Swarm` SHA-256,
`otool -L` dependency identity, CMake cache identity, conda explicit package
list, both config hashes, both ten-seed lists, all schemas/tolerances/gates,
the exact Task 11 runner/analyzer argv arrays, and both absent roots. The
protocol generator writes only the named JSON/Markdown pair atomically.

```bash
conda run -n cbf_env python \
  scripts/diagnostics/register_qualified_closure_campaign.py \
  --kind development --version v1 \
  --implementation-report docs/diagnostics/2026-08-01-cbf2026-qualified-closure-implementation.md \
  --implementation-review docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-implementation-review.md \
  --binary build-diagnostic/Swarm \
  --base-config config/config.json \
  --primary-config config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json \
  --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json \
  --trajectory-seeds 2026080101:2026080110 \
  --range-noise-seeds 2026081101:2026081110 \
  --frames 1000 \
  --raw-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v1 \
  --analysis-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v1 \
  --protocol-json docs/diagnostics/2026-08-01-cbf2026-qualified-closure-development-v1-protocol.json \
  --protocol-md docs/diagnostics/2026-08-01-cbf2026-qualified-closure-development-v1-protocol.md
```

Expected: both output files are new, their embedded runner/analyzer token
arrays equal Task 11 exactly, neither execution root is created, and a second
invocation refuses to replace either protocol file.

Observed historical result: the protocol pair was generated without creating
either v1 root. The JSON is 41,979 bytes with SHA-256
`9a5e9d2503d083bdbb239468a88cf5ceed2cc283978e8beed5badc020ae74343`;
the Markdown is 312 bytes with SHA-256
`19cf657d787bfe76bfa2aecf35ce06baf9fd2f57ff2deeab12e894429ccebffc`.

- [x] **Historical Step 8: Perform independent development-v1 preflight — FAILED**

Recompute every identity from the implementation commit, prove both roots
absent, prove the v2 tree commitment unchanged, verify 8 GB launch space, and
validate exact command arrays. Create an authorization JSON bound to protocol
SHA-256 and implementation commit that records the researcher's standing
authorization for development execution. It does not authorize root reuse,
retry, threshold changes, or confirmatory execution.

Observed historical result: independent preflight returned C1/I0/M0 and
`FAILED — NOT READY` because the exact-HEAD validator made its own publication
and required later commit mutually unsatisfiable. The 9,274-byte preflight has
SHA-256
`7ef7fbbf09d1ef95ecad482282a75398158fb97008fdb8a08745efeb51523515`.
The v1 authorization and both v1 execution roots remained absent. This failed
v1 protocol/preflight is immutable audit evidence and may not be repaired,
retried, overwritten, reinterpreted, or used for execution.

- [x] **Historical Step 9: Commit the exact three failed-v1 audit artifacts**

```bash
git add docs/diagnostics/2026-08-01-cbf2026-qualified-closure-development-v1-protocol.json \
  docs/diagnostics/2026-08-01-cbf2026-qualified-closure-development-v1-protocol.md \
  docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-development-v1-preflight.md
git commit -m "docs(cbf2026): record qualified closure v1 preflight failure"
```

Observed: exact three-artifact commit
`0e4c439ae35f8490b27019aeb26b1c46ba9ab3f7`; no authorization was created.

- [x] **Historical v2 Recovery Step 10: Repair the first authorization lifecycle deadlock**

Commit `eafa68c3631c590b9f1070bcc8d313ca4f3b7705` (tree
`6fe358619a78fe259c2fb33998be4170bd8786f6`) closes the lifecycle deadlock.
Development now requires version `v2` and the exact v2 roots below;
confirmatory remains version `v1`. Authorization remains bound to exact
protocol bytes, the registered implementation identity, exact independent
preflight bytes, and the researcher's exact dated authorization text. The
authorization gate accepts only one current commit whose sole parent is the
registered implementation commit and whose diff is exactly the four named
artifacts below, all added with no extra path. Review chronology was I1 for
the protocol declaration, then M1 for the authorization fixture, then final
C0/I0/M0 after both were corrected.

Before registration, commit the updated implementation report, source plan,
and independent recovery review as a clean documentation-closure commit after
`eafa68c...`. The registrar must discover and record that then-current exact
commit as `protocol.repository.head`. Its identity is intentionally not
hard-coded here because it does not exist until these documentation updates
are complete.

- [x] **Historical v2 Recovery Step 11: Generate the exact development-v2 protocol**

Run only from the clean, final report/plan/review documentation-closure commit
described above. Its source code-fix ancestor is
`eafa68c3631c590b9f1070bcc8d313ca4f3b7705`; its exact current HEAD is what the
registrar must bind. No working edit may contaminate registration or the later
exact artifact-only child.

```bash
conda run -n cbf_env python \
  scripts/diagnostics/register_qualified_closure_campaign.py \
  --kind development --version v2 \
  --implementation-report docs/diagnostics/2026-08-01-cbf2026-qualified-closure-implementation.md \
  --implementation-review docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-implementation-review.md \
  --binary build-diagnostic/Swarm \
  --base-config config/config.json \
  --primary-config config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json \
  --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json \
  --trajectory-seeds 2026080101:2026080110 \
  --range-noise-seeds 2026081101:2026081110 \
  --frames 1000 \
  --raw-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v2 \
  --analysis-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v2 \
  --protocol-json docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.json \
  --protocol-md docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.md
```

- [x] **Historical v2 Recovery Step 12: Independently preflight and create the bound authorization**

Create exactly:

```text
docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-preflight.md
docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-authorization.json
```

The preflight must independently recompute every registered identity and argv,
confirm both v2 roots absent, retain the historical two-range v2 commitment,
verify launch space, and return C0/I0/M0. The authorization schema is exactly
`schema_version`, `authorized`, `kind`, `version`, `protocol_sha256`,
`implementation_identity`, `preflight_sha256`, `user_authorization_date`,
`user_authorization_text`, and `user_authorization_text_sha256`. It binds date
`2026-08-02` and the researcher's exact standing-authorization text `批准`,
whose UTF-8 SHA-256 is
`8cbe697b157364a5b13646285b38409dc53ec5287deeb7913493e65b275cd14d`.
Any different text used inside a unit-test fixture is test data only and does
not authorize the production protocol.

- [x] **Historical v2 Recovery Step 13: Commit the exact four-artifact direct child; postcommit validation FAILED**

The commit must be the sole direct child of the exact documentation-closure
commit recorded by the v2 protocol as `repository.head`, and must add exactly
these four paths, with no modification, deletion, rename, or additional path:

```bash
git add docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.json \
  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.md \
  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-preflight.md \
  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-authorization.json
git commit -m "docs(cbf2026): authorize qualified closure development v2"
```

Observed historical result: documentation parent
`857de7ee8dd563fdd5fa15b6860ed54da6758b37` and exact four-artifact child
`ae7f68c547eaccfb5c4be0b44a08895ab8beeb36` satisfied the required topology
and byte bindings, but the production postcommit validator attempted to read
the deliberately untracked `build-diagnostic/Swarm` from the parent Git tree
and failed before either root was claimed. Failure artifact
`docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-postcommit-validation.md`
is 3,711 bytes / 91 lines with SHA-256
`815988e3149d6adcffb61d323e242905ae3e89a145eb37560abf1b47de9fb235`.
The v2 protocol, preflight, authorization, failure report, and commits are
immutable failed-gate evidence. Both v2 roots remain absent; no runner,
analyzer, mission, or empirical computation ran. Development-v2 is consumed,
non-executable, and may not be retried, overwritten, amended, or reinterpreted.

- [x] **Recovery Step 14: Repair postcommit bindings under TDD and freeze the v3 code-fix identity**

Commit `64f64d00a1fd6b7ab01760e340eca883be58f338` (parent
`02f0f13ce1b3803b84840221221694ab02d2441d`,
tree `b2c7cc555e5ee6c684ac6fb2eb14d44157c5f9dd`) changes exactly the registrar,
runner, analyzer, and their three test modules. Only exact path tokens already
recorded in `repository.allowed_untracked_paths` bypass parent-blob lookup;
each still must remain allowed-untracked in live Git status and match its
registered live path, regular-file status, SHA-256, byte count, and build or
dependency binding. Every other bound source, configuration, report, review,
and tool remains verified from the registered parent Git blob. The analyzer
now independently derives and binds the complete development-v3,
confirmatory-v1, or confirmatory-smoke schedule envelope before claiming its
analysis root.

The genuinely untracked Git integration regression passed after RED, all
three campaign schedule/tamper regressions passed 3/3 after RED, the focused
five-module suite passed 173/173, all three production scripts passed
`py_compile`, the fix diff passed `git diff --check`, and the complete Python
discovery was 1,162 total = 1,149 passed + the unchanged 5 failures + unchanged
8 errors, with 0 skipped. Independent final review returned C0/I0/M0.

Before v3 registration, commit this updated plan, implementation report, and
future independent recovery review as one clean documentation-closure commit
after `64f64d0...`. The registrar must record that future exact HEAD as
`protocol.repository.head`; its hash is intentionally unknown until the three
documents are final and committed.

- [ ] **Recovery Step 15: Generate the exact development-v3 protocol**

Run only from that clean future documentation-closure commit, with both v3
roots and all four v3 lifecycle paths absent:

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

- [ ] **Recovery Step 16: Independently preflight and prepare the exact v3 authorization boundary**

Create exactly, but only after an independent C0/I0/M0 preflight:

```text
docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-preflight.md
docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-authorization.json
```

Recompute every protocol/source/blob/allowed-untracked/build/config/dependency,
seed, schedule-envelope, threshold, argv, and root-absence identity. The
authorization retains the exact ten-field schema and binds the protocol
SHA-256, registered documentation-closure implementation identity, preflight
SHA-256, kind `development`, version `v3`, date `2026-08-02`, and the
researcher's exact standing-authorization text `批准` with UTF-8 SHA-256
`8cbe697b157364a5b13646285b38409dc53ec5287deeb7913493e65b275cd14d`.
This plan records the boundary only; it does not create or grant authorization.

- [ ] **Recovery Step 17: Commit and validate the exact v3 four-artifact direct child**

Commit exactly the protocol JSON, protocol Markdown, preflight Markdown, and
authorization JSON as one four-add-only commit whose sole parent is the
documentation-closure identity recorded in the v3 protocol, with no extra,
modified, deleted, or renamed path:

```bash
git add docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.json \
  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.md \
  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-preflight.md \
  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-authorization.json
git commit -m "docs(cbf2026): authorize qualified closure development v3"
```

Before any root allocation, independently run the production postcommit
validator and recheck the exact direct-child diff, every parent blob, both
exact allowed-untracked path tokens and their live/status/hash/size/build
bindings, all authorization bindings, and both v3 roots absent. Only a fully
passing committed state makes Task 11 eligible. Any failure consumes v3
without retry and requires the next non-colliding reviewed version. No v3
lifecycle artifact or root exists and no command above has been run now.

---

### Task 11: Execute and Adversarially Review Development v3

This is the only operative development execution task. Development-v1 and
development-v2 are immutable failed/non-executable histories. No command in
either historical block below may be run.

**Files:**
- Generate: `/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v3/**`
- Generate: `/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v3/**`
- Consume: `docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.json`
- Consume: `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-authorization.json`
- Create: `docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3.md`
- Create: `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-review.md`
- Modify: DRA four CBF2026 files from Task 1.

**Interfaces:**
- Consumes: the independently validated exact Task 10 development-v3
  protocol/preflight/authorization direct-child commit and frozen ten-seed
  primary/ablation configuration.
- Produces: one once-only immutable terminal development-v3 raw/analysis
  bundle, author report, independent review, and Task 12 gate decision.

- [ ] **Step 1: Execute development-v3 exactly once after the complete Task 10 gate**

Execute the protocol-serialized argv token-for-token; the expanded command is:

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

- [ ] **Step 2: Analyze the immutable development-v3 bundle exactly once**

Only after the raw manifest is terminal, execute the registered analyzer argv
token-for-token:

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

- [ ] **Step 3: Stop and retain on any failed execution, integrity, or scientific gate**

Retain every v3 lifecycle artifact and allocated root unchanged, write the
exact failure in the named v3 report/review, and update DRA. Never retry,
resume, delete, overwrite, repartition, reinterpret, or rerun v3. Any changed
code, configuration, threshold, evidence contract, or authorization lifecycle
requires a reviewed fix and the next non-colliding development version.

- [ ] **Step 4: Independently recompute raw accounting and every gate**

Without importing reported booleans, independently recompute protocol and
authorization hashes, the full trusted development-v3 schedule envelope,
mission/root manifests, seed and condition cardinalities, initialization and
estimator universes, controller intervals, certificate availability,
`nu_inst <= bar_nu`, allocated/full edge rows and residuals, reset
transactions, command bounds, true-distance violations, completion, and
mission-success arithmetic. Record every mismatch and an explicit C/I/M
verdict.

- [ ] **Step 5: Commit report/review, update DRA, and open or close Task 12**

```bash
git add docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3.md \
  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-review.md
git commit -m "docs(cbf2026): record qualified closure development v3"
```

Update and separately commit the four Task 1 DRA files with all lifecycle,
root, manifest, analysis, report, review, and failed/passed gate identities;
require both worktrees clean. Only an independently reproduced development-v3
PASS with C0/I0 review permits Task 12. Any failed gate, Critical, or Important
finding keeps confirmatory closed.

No v3 protocol, authorization, result artifact, or root exists at this plan
update, and neither execution command has been run.

---

### Historical Task 11: Development v2 — superseded and never executed

The entire v2 block below is retained only as the pre-postcommit-validation
plan that produced immutable failed-gate evidence. Its production validator
failed before root allocation. No command in it confers authority or may be
executed.

**Files:**
- Generate: `/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v2/**`
- Generate: `/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v2/**`
- Consume: `docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.json`
- Consume: `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-authorization.json`
- Create: `docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2.md`
- Create: `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-review.md`
- Modify: DRA four CBF2026 files from Task 1.

**Interfaces:**
- Consumes: exact Task 10 development-v2 protocol/authorization artifact
  commit and the frozen ten-seed primary/ablation configuration.
- Produces: one immutable terminal development-v2 raw/analysis bundle, author
  report, independent gate review, and explicit Task 12 gate decision.

- [ ] **Step 1: Execute development-v2 exactly once after the complete Task 10 gate**

Execute the serialized runner argv from the v2 protocol; the expanded command
below must compare token-for-token before launch:

```bash
conda run -n cbf_env python \
  scripts/diagnostics/run_qualified_closure_campaign.py \
  --kind development \
  --version v2 \
  --protocol docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.json \
  --authorization docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-authorization.json \
  --binary build-diagnostic/Swarm \
  --base-config config/config.json \
  --primary-config config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json \
  --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json \
  --trajectory-seeds 2026080101:2026080110 \
  --range-noise-seeds 2026081101:2026081110 \
  --frames 1000 \
  --output-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v2
```

- [ ] **Step 2: Analyze the immutable development-v2 bundle**

Only after the raw bundle is terminal, execute the exact serialized analyzer
argv:

```bash
conda run -n cbf_env python \
  scripts/diagnostics/analyze_qualified_closure_campaign.py \
  --kind development \
  --version v2 \
  --protocol docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.json \
  --authorization docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-authorization.json \
  --input-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v2 \
  --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json \
  --output-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v2
```

- [ ] **Step 3: Stop and retain on any failed execution or analysis gate**

If the runner, analyzer, integrity accounting, or scientific gate fails,
retain every v2 artifact and root unchanged, write the exact failure in the
named v2 report and review, and update the DRA. Never retry, resume, delete,
overwrite, repartition, or reinterpret v2. Any changed code, configuration,
threshold, or authorization lifecycle requires another reviewed fix and the
next non-colliding development version.

- [ ] **Step 4: Independently recompute raw accounting and every gate**

Without importing reported result booleans, independently recompute raw and
compact hashes, schedule/seed cardinalities, all ten declared mission
outcomes, initialization and per-condition estimator universes, controller
intervals, certificate availability, `nu_inst <= bar_nu`, allocated/full edge
rows and residuals, reset transactions, command bounds, true-distance
violations, completion, and mission-success arithmetic. Record every mismatch
and return an explicit C/I/M review verdict.

- [ ] **Step 5: Commit the v2 report/review, update DRA, and open or close Task 12**

```bash
git add docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2.md \
  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-review.md
git commit -m "docs(cbf2026): record qualified closure development v2"
```

Historical disposition superseding every step above: v2 failed production
postcommit validation, so no v2 result/report/review or DRA PASS gate may be
created and Task 12 may never consume v2. Neither execution command ran; both
v2 roots and both v2 result files remain absent, and there is no empirical
result.

---

### Historical Task 11: Development v1 — superseded and never executed

The entire block below is retained only to preserve the original audit trail.
No command in it confers authority or may be executed.

**Files:**
- Generate: `/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v1/**`
- Generate: `/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v1/**`
- Create: `docs/diagnostics/2026-08-01-cbf2026-qualified-closure-development-v1.md`
- Create: `docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-development-v1-review.md`
- Modify: DRA four CBF2026 files.

**Interfaces:**
- Consumes: reviewed Task 10 implementation and frozen ten-seed v1 config.
- Produces: one immutable terminal development bundle and gate decision.

- [ ] **Step 1: Preflight exact identities and absence**

```bash
cd /private/tmp/cbf2026-diagnostic
git status --short
rg '^ENABLE_GUROBI:BOOL=ON$' build-diagnostic/CMakeCache.txt
test -x build-diagnostic/Swarm
test ! -e /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v1
test ! -e /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v1
df -k /private/tmp
shasum -a 256 \
  config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json \
  config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json \
  docs/diagnostics/2026-08-01-cbf2026-qualified-closure-development-v1-protocol.json \
  docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-development-v1-authorization.json
```

Expected: only preserved `?? build-diagnostic/`, both roots absent, at least 8
GB free, and every observed identity equal to the committed protocol and
authorization. Execute the serialized runner argv from the protocol; the
expanded command below is explanatory and must compare token-for-token before
launch.

- [ ] **Step 2: Execute development once**

```bash
conda run -n cbf_env python \
  scripts/diagnostics/run_qualified_closure_campaign.py \
  --kind development \
  --version v1 \
  --protocol docs/diagnostics/2026-08-01-cbf2026-qualified-closure-development-v1-protocol.json \
  --authorization docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-development-v1-authorization.json \
  --binary build-diagnostic/Swarm \
  --base-config config/config.json \
  --primary-config config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json \
  --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json \
  --trajectory-seeds 2026080101:2026080110 \
  --range-noise-seeds 2026081101:2026081110 \
  --frames 1000 \
  --output-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v1
```

Expected: terminal manifest always exists; exit zero only if all mission processes terminate normally.

- [ ] **Step 3: Analyze the immutable bundle**

```bash
conda run -n cbf_env python \
  scripts/diagnostics/analyze_qualified_closure_campaign.py \
  --kind development \
  --version v1 \
  --protocol docs/diagnostics/2026-08-01-cbf2026-qualified-closure-development-v1-protocol.json \
  --authorization docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-development-v1-authorization.json \
  --input-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v1 \
  --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json \
  --output-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v1
```

- [ ] **Step 4: Stop on any failed gate**

If a gate fails, retain v1 unchanged, write the failure report/review, update DRA, and return to the smallest responsible implementation task. Any changed code/config/threshold requires development `v2`; it may not overwrite or reinterpret v1.

- [ ] **Step 5: Independently review raw accounting and gates**

Recompute hashes, key cardinalities, ten declared mission outcomes, estimator/controller denominators, rate inequalities, edge reconstruction, reset transactions, true distance violations, and mission-success arithmetic without importing result booleans.

- [ ] **Step 6: Commit reports and open/close confirmatory gate**

```bash
git add docs/diagnostics/2026-08-01-cbf2026-qualified-closure-development-v1.md \
  docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-development-v1-review.md
git commit -m "docs(cbf2026): record qualified closure development v1"
```

Only a PASS with C0/I0 review permits Task 12. Update DRA with both failed and passed gates.

---

### Task 12: Freeze and Execute the Exactly 60-Mission Confirmatory Campaign

Recovery dependency override: confirmatory itself remains scientific protocol
version `v1`, with its existing roots and seeds, but every Task 12 dependency
on development-v1 or development-v2 is historical and superseded. Task 12 may
consume only a terminal PASS development-v3 protocol/bundle/report/review from
the operative Task 11. This override does not authorize any confirmatory
command.

**Files:**
- Verify: `scripts/diagnostics/register_qualified_closure_campaign.py`
- Verify: `tests/test_register_qualified_closure_campaign.py`
- Consume: `docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.json`
- Consume: `docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3.md`
- Consume: `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-review.md`
- Generate: `docs/diagnostics/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-protocol.json`
- Generate: `docs/diagnostics/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-protocol.md`
- Create: `docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-preflight.md`
- Create: `docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-authorization.json`
- Create: `docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-smoke-review.md`
- Create: `docs/diagnostics/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-report.md`
- Create: `docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-evidence-review.md`
- Create: `docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-evidence-review.json`
- Generate: `/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-a/**`
- Generate: `/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-a-analysis/**`
- Generate: `/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-b/**`
- Generate: `/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-b-analysis/**`
- Generate: `/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory/v1/**`
- Generate: `/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-analysis/v1/**`
- Modify: DRA four CBF2026 files.

**Interfaces:**
- Consumes: terminal passing development version and clean source/config/analyzer identities.
- Produces: immutable prospective 60-mission evidence with exact denominator 60.

- [ ] **Step 1: Re-run the frozen registrar tests before generating a protocol**

Assert exact source/config/seed/schema/threshold/root bindings, 60 distinct
scientific seeds, the exact one-mission smoke schedule/universes, all six
execution roots absent, smoke/development/confirmatory seed disjointness, no
retry, and no post-registration mutation. A 59/61-seed list, altered smoke
seed/frame/universe, existing path, dirty relevant source, or threshold
mismatch must fail before output allocation.

- [ ] **Step 2: Verify the reviewed registrar's exact confirmatory contract**

Use trajectory seeds exactly `2026082001..2026082060` and separately frozen
range-noise seeds `2026083001..2026083060`; bind estimator frames `0..999`,
controller intervals `0..999`, 14 sUAVs, the full schedule, every threshold,
all code/config hashes, and distinct
roots for smoke A/B, confirmatory raw, and confirmatory analysis. The exact
registered universes are 840 initialization tuples, 839160
post-initialization estimator tuples in each estimator condition (1678320
across primary plus fixed-FIM ablation), 60000 primary controller intervals,
13920000 allocated endpoint-row records, 7140000 reconstructed full-row
records, and exactly 60 primary missions. Initialization is a single shared
840-tuple deployment/domain universe, not duplicated by estimator condition.

The six exact roots listed above are protocol fields. All six must be absent
at registration and again at authorization preflight. Each smoke runner owns
only its smoke raw root, each smoke analyzer owns only its paired analysis
root, the scientific runner owns only the confirmatory raw root, and the
scientific analyzer owns only the confirmatory analysis root.

Smoke A and B both consume the same frozen one-mission schedule: trajectory
seed `2026089001`, range-noise seed `2026089101`, estimator frames `0..19`,
and controller intervals `0..19`. Its exact non-scientific universes are 14
shared initialization tuples, 266 post-initialization estimator tuples per
condition (532 across primary and fixed-FIM), 20 primary controller intervals,
4640 allocated endpoint-row records, 2380 reconstructed full-row records, and
one smoke terminal mission record. These keys are tagged `confirmatory-smoke`
and can never enter the 60-mission universes or gates. The protocol stores one
canonical semantic schedule identity used by both A and B; only run ID, root,
timestamps, and compression metadata may differ, and the smoke analyzer
removes only those declared transport fields before comparing semantic hashes.

```bash
conda run -n cbf_env python -m unittest \
  tests.test_register_qualified_closure_campaign -v
conda run -n cbf_env python -m py_compile \
  scripts/diagnostics/register_qualified_closure_campaign.py
git diff --check
```

Expected: PASS on the same registrar identity independently reviewed in Task
10; no production output path exists.

- [ ] **Step 3: Generate, preflight, authorize, and commit the exact confirmatory four-artifact child**

Begin from the clean Task 11 development-v3 PASS report/review commit in the
main repository and a separately clean committed DRA checkpoint, with all six
confirmatory roots absent. The registrar records the exact current main-repo
commit as `protocol.repository.head`; it remains the scientific
implementation/docs parent for confirmatory-v1. No working edit or extra
untracked relevant path may be present.

```bash
conda run -n cbf_env python \
  scripts/diagnostics/register_qualified_closure_campaign.py \
  --kind confirmatory --version v1 \
  --development-protocol docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.json \
  --development-report docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3.md \
  --development-review docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-review.md \
  --binary build-diagnostic/Swarm \
  --base-config config/config.json \
  --primary-config config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json \
  --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json \
  --trajectory-seeds 2026082001:2026082060 \
  --range-noise-seeds 2026083001:2026083060 \
  --smoke-trajectory-seed 2026089001 \
  --smoke-range-noise-seed 2026089101 \
  --smoke-frames 20 \
  --frames 1000 \
  --smoke-a-raw-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-a \
  --smoke-a-analysis-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-a-analysis \
  --smoke-b-raw-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-b \
  --smoke-b-analysis-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-b-analysis \
  --raw-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory/v1 \
  --analysis-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-analysis/v1 \
  --protocol-json docs/diagnostics/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-protocol.json \
  --protocol-md docs/diagnostics/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-protocol.md
```

Do not commit the protocol pair separately. While all four lifecycle files are
still new, independently reconstruct the JSON/Markdown pairing and every bound
source, development-v3 PASS, binary, configuration, dependency, seed,
threshold, universe, argv, and six-root-absence identity. Write the result to
`docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-preflight.md`;
it must be C0/I0/M0 before authorization may exist.

Create the authorization JSON at
`docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-authorization.json`
with the exact ten-field schema: `schema_version` equal to
`cbf2026-qualified-authorization-v1`, `authorized` equal to `true`, `kind`
equal to `confirmatory`, `version` equal to `v1`, exact protocol SHA-256,
the implementation identity from `protocol.repository.head`, exact preflight
SHA-256, `user_authorization_date` equal to `2026-08-02`, exact UTF-8
`user_authorization_text` equal to `批准`, and
`user_authorization_text_sha256` equal to
`8cbe697b157364a5b13646285b38409dc53ec5287deeb7913493e65b275cd14d`.
No unit-test fixture text is production authorization.

Commit exactly the four new lifecycle artifacts as the sole direct child of
the registered implementation/docs parent, with no modified, deleted,
renamed, or additional path:

```bash
git add docs/diagnostics/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-protocol.json \
  docs/diagnostics/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-protocol.md \
  docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-preflight.md \
  docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-authorization.json
git commit -m "docs(cbf2026): authorize qualified closure confirmatory v1"
```

Before allocating any smoke or scientific root, independently validate that
the current commit has exactly one parent equal to `protocol.repository.head`,
its complete parent-to-current diff is exactly four `A` entries for the named
files, all authorization hashes/text/date/identity bindings pass the production
validator, every bound byte identity remains unchanged, and all six roots are
still absent. Any failure consumes no root and returns to a new reviewed,
non-colliding protocol lifecycle; it never amends this commit in place. Keep
this exact artifact-only HEAD and a clean relevant worktree through both smoke
runs, the confirmatory run, and all three analyses.

- [ ] **Step 4: Run and review deterministic smoke A/B**

Execute the protocol's smoke A replay/analyzer and smoke B replay/analyzer
arrays exactly once in the frozen order, using distinct absent roots. Require
byte-identical semantic analysis hashes, complete expected keys, all integrity
gates, and an independent C0/I0 smoke review. Smoke contains no primary
scientific aggregate and cannot change the registered thresholds or code.

The serialized commands must expand token-for-token to the following, first
with suffix A and then with suffix B:

```bash
conda run -n cbf_env python scripts/diagnostics/run_qualified_closure_campaign.py \
  --kind confirmatory-smoke --smoke-id a \
  --protocol docs/diagnostics/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-protocol.json \
  --authorization docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-authorization.json \
  --binary build-diagnostic/Swarm --base-config config/config.json \
  --primary-config config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json \
  --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json \
  --trajectory-seeds 2026089001:2026089001 \
  --range-noise-seeds 2026089101:2026089101 --frames 20 \
  --output-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-a
conda run -n cbf_env python scripts/diagnostics/analyze_qualified_closure_campaign.py \
  --kind confirmatory-smoke --version v1 --smoke-id a \
  --protocol docs/diagnostics/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-protocol.json \
  --authorization docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-authorization.json \
  --input-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-a \
  --output-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-a-analysis
```

For B replace only `a` by `b` and the two A roots by their B roots. Independently
recompute the comparison and require C0/I0 before the scientific run, but do
not yet create or commit a repository review: any extra tracked or untracked
relevant path, or any commit beyond the exact four-artifact child, invalidates
the production authorization gate. Preserve the interim smoke decision in an
immutable external operator ledger outside the repository and all registered
roots. After the scientific runner and analyzer are terminal, reproduce that
decision into the named smoke review and commit it only with the final evidence
closure in Step 8.

- [ ] **Step 5: Execute the registered campaign once**

Run the exact argv array serialized in the protocol, not a retyped approximation:

```bash
conda run -n cbf_env python scripts/diagnostics/run_qualified_closure_campaign.py \
  --kind confirmatory --version v1 \
  --protocol docs/diagnostics/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-protocol.json \
  --authorization docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-authorization.json \
  --binary build-diagnostic/Swarm --base-config config/config.json \
  --primary-config config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json \
  --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json \
  --trajectory-seeds 2026082001:2026082060 \
  --range-noise-seeds 2026083001:2026083060 --frames 1000 \
  --output-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory/v1
```

Record process start/end, return code, terminal manifest, and allocated/free
bytes. No retry or partitioned equivalent is allowed.

- [ ] **Step 6: Run the registered analyzer once**

Run the exact analyzer argv from the protocol:

```bash
conda run -n cbf_env python scripts/diagnostics/analyze_qualified_closure_campaign.py \
  --kind confirmatory --version v1 \
  --protocol docs/diagnostics/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-protocol.json \
  --authorization docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-authorization.json \
  --input-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory/v1 \
  --output-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-analysis/v1
```

It must account for all 60 missions even when a process never starts or aborts.

- [ ] **Step 7: Perform independent evidence review**

Verify raw hashes, schedules, exact mission denominator 60, all tuple/interval universes, mode and domain outcomes, private-age strata, containment/availability, `nu_inst <= bar_nu`, row completeness/residuals, reset guards, input/QP/true-distance outcomes, completion, and mission-level confidence intervals. Report wrong-mode mission incidence and its one-sided 95% upper bound.

- [ ] **Step 8: Freeze terminal evidence and update DRA**

Write the named author report, smoke review, and both evidence-review files.
Only the protocol, preflight, and authorization are already committed; do not
rewrite them. Reproduce the independently recorded external smoke decision
into the named smoke review now that no further authorized runner/analyzer
invocation remains. The JSON review
attestation contains at least `verdict`, `critical_count`, `important_count`,
`protocol_sha256`, `manifest_sha256`, `compact_json_path`, and
`reviewed_compact_sha256`. Commit the smoke review, terminal author report, and
both terminal reviews in one scoped evidence commit. Record every result and
failure in DRA.
Paper numerical gates remain closed unless the registered scientific verdict
passes and independent review has C0/I0.

---

### Task 13: Revise the Paper's Estimator and Theory Chain

**Files:**
- Modify: `/private/tmp/cbf2026-paper-geometric-stability/build.sh`
- Create: `/private/tmp/cbf2026-paper-geometric-stability/review/test_build_fail_closed.sh`
- Modify: `/private/tmp/cbf2026-paper-geometric-stability/main.tex:43-177`
- Modify: `/private/tmp/cbf2026-paper-geometric-stability/main.tex:229-358`
- Modify: `/private/tmp/cbf2026-paper-geometric-stability/main.tex:416-715`
- Modify: `/private/tmp/cbf2026-paper-geometric-stability/main.tex:741-1205`
- Create: `/private/tmp/cbf2026-paper-geometric-stability/assets/robust-cbf-boundary.drawio`
- Create: `/private/tmp/cbf2026-paper-geometric-stability/assets/robust-cbf-boundary.drawio.png`
- Create: `/private/tmp/cbf2026-paper-geometric-stability/assets/decoupled-cbf-trajectory.drawio`
- Create: `/private/tmp/cbf2026-paper-geometric-stability/assets/decoupled-cbf-trajectory.drawio.png`

**Interfaces:**
- Consumes: approved specification, passing unit/property proofs, implementation identity, and reviewed development/confirmatory contract.
- Produces: a conditional, logically closed continuous-time theory and an honest offline-estimator/2 Hz implementation description; no unsupported numerical claim.

This task may begin immediately after Task 7's controller review passes and
may run alongside Tasks 8--12. It must not edit numerical results, abstract
performance claims, or conclusions that depend on Task 12.

- [ ] **Step 1: Make the paper build fail closed before using it as a gate**

First create `review/test_build_fail_closed.sh`. It copies the paper into an
exact `mktemp -d` fixture, records the fixture's preexisting `main.pdf` hash,
mechanically injects one undefined citation into the copied `main.tex`, runs
the copied `build.sh`, and asserts both a nonzero exit and an unchanged stale
PDF hash. The test owns/traps only its exact temporary directory.

```bash
cd /private/tmp/cbf2026-paper-geometric-stability
bash -n review/test_build_fail_closed.sh
bash review/test_build_fail_closed.sh
```

Expected before changing `build.sh`: the shell test itself exits nonzero on
its explicit `test "$build_status" -ne 0` or stale-hash assertion. A missing
command, syntax failure, or unrelated LaTeX failure does not count as red.

Rewrite `build.sh` so every pdflatex and BibTeX nonzero status aborts, all
passes write into a unique `mktemp -d` directory, and the script atomically
replaces `main.pdf` only after the final pass produces a nonempty PDF. Copy
the final `.log/.blg/.bbl` back for audit, reject undefined citations or
references, and clean only the exact temporary directory via `trap`. A stale
preexisting `main.pdf` must never convert a failing build into success.

Run `bash -n build.sh`, a successful clean build, and the same
`bash review/test_build_fail_closed.sh` assertion green without changing the
previous PDF hash. Commit this regression with the later theory edits.

- [ ] **Step 2: Replace estimator-role conflation**

Define an abstract estimated-state/controller interface and separately define the offline WNLS sidecar. Remove sentences that imply LIO, WNLS, modeled FIM covariance, true-error envelope, and controller nominal state are the same object.

- [ ] **Step 3: Separate the five claim layers**

Rewrite graph assumptions so lower-index references prove only DAG well-foundedness. State local `Phi_i \succ 0` as local differential observability, global mode qualification as a separate premise/mechanism, coefficient-three FIM radius as conditional design radius, and true-error envelope as the explicit premise of robust distance transfer.

State as a limitation that the retained diagonal FIM propagation omits
cross-covariance created when two descendants share an uncertain ancestor.
Consequently neither DAG structure nor local `Phi_i \succ 0` turns the
coefficient-three radius into a guaranteed joint confidence region; that
calibration is evaluated only by the prospective Monte Carlo evidence.

- [ ] **Step 4: Add the two-range reflection and deployment-domain proposition**

Give the collinear-anchor mirror counterexample, define the predeclared ocean-side half-plane, and state that exactly one admissible mode is required; zero/multiple modes fail closed. Make clear that later offshore two-reference cases need ZOH-history qualification or abstention.

Add the estimator branch conditional induction separately from the safety
proof. The base case assumes the cold-start deployment contract selects the
true branch. For a later fresh publication, assume the previous fresh branch
was true, the true motion is consistent with the exact held-command history,
the complete enumerated/solved candidate set contains the relevant true mode,
the true mode satisfies the frozen qualifier, and every false mode fails it.
Under those premises unique-mode publication selects the true branch; induction
then applies only across consecutive fresh publications. An abstention,
predicted output, incomplete local-minimum enumeration, violated motion model,
or false-mode qualifier pass breaks the induction and is not converted into a
correctness guarantee. Monte Carlo evidence measures how often these premises
hold; it does not prove them.

- [ ] **Step 5: Replace the rate heuristic in the theorem-aligned derivation**

Add `beta_ij`, `overline{dot w}_ij`, `L_i^Phi`, `L_i^P`, and `bar_nu_i` exactly as approved. Retain backward difference only as an archived/descriptive implementation field if needed.

- [ ] **Step 6: Add allocated endpoint proposition**

State both localization/collision endpoint inequalities, allocations summing to one, bUAV full responsibility, and prove their sum recovers the coupled row. Explicitly state the local split is sufficient and more conservative; centralized feasibility does not imply local split feasibility.

- [ ] **Step 7: Add hybrid reset proposition**

Define flows on fixed active sets and jumps as complete transitive-descendant, two-phase, same-version transactions. List nonnegative post-reset barriers, feasible bounded local hard QPs, locally finite resets, and the post-reset true-error envelope as premises.

Make the temporal induction explicit: assume the correct qualified branch and
true-error envelope hold initially; prove flow invariance while the active set
is fixed; at every accepted reset require branch correctness and the envelope
again for every changed node and descendant, plus the guard premises; then
induct over the locally finite flow/jump sequence. The FIM and reset guard do
not prove branch correctness or the envelope—they only make the safety result
conditional on those premises at initialization and after each accepted jump.

- [ ] **Step 8: Separate the continuous theorem from 2 Hz evidence**

Do not add a sampled-data reserve. State that the 2 Hz implementation provides empirical certificate/residual/true-distance evidence and does not inherit continuous-time invariance merely from sampled rows.

- [ ] **Step 9: Complete the two pending explanatory figures**

Create a robust-distance figure showing nominal separation, both coefficient-three radii, the tightened localization upper boundary, and collision lower boundary. Create a decoupled-search figure comparing the circular motion caused by a coupled distance/yaw objective with the forward-progress trajectory of separate distance and yaw task CBFs. Keep editable `.drawio` sources and export PNGs with 10 px padding; use normal or italic text rather than bold headings.

```bash
cd /private/tmp/cbf2026-paper-geometric-stability
/Applications/draw.io.app/Contents/MacOS/draw.io -x -f png -b 10 \
  -o assets/robust-cbf-boundary.drawio.png \
  assets/robust-cbf-boundary.drawio
/Applications/draw.io.app/Contents/MacOS/draw.io -x -f png -b 10 \
  -o assets/decoupled-cbf-trajectory.drawio.png \
  assets/decoupled-cbf-trajectory.drawio
```

Inspect the exported dimensions and embed each only where it replaces enough prose to fit the journal page budget.

- [ ] **Step 10: Compile and run a theory-only review**

```bash
cd /private/tmp/cbf2026-paper-geometric-stability
./build.sh
rg -n "undefined|Warning--|Citation.*undefined|Reference.*undefined" main.log main.blg
```

Dispatch independent theorem, notation, and claim review before committing.

- [ ] **Step 11: Commit theory edits**

```bash
git add build.sh review/test_build_fail_closed.sh main.tex \
  assets/robust-cbf-boundary.drawio \
  assets/robust-cbf-boundary.drawio.png \
  assets/decoupled-cbf-trajectory.drawio \
  assets/decoupled-cbf-trajectory.drawio.png
git commit -m "docs(paper): close qualified hybrid distributed cbf theory"
```

- [ ] **Step 12: Record the theory-paper checkpoint in DRA**

Record the paper commit/PDF hash, theorem review, exact claim limitations,
and the still-closed numerical evidence gate.

---

### Task 14: Replace Archived Negative Results Only from Passing Confirmatory Evidence

**Files:**
- Modify: `/private/tmp/cbf2026-paper-geometric-stability/main.tex:1213-1438`
- Modify: `/private/tmp/cbf2026-paper-geometric-stability/main.tex:1629-1660`
- Modify: `/private/tmp/cbf2026-paper-geometric-stability/main.tex:1772-1810`
- Create: `scripts/diagnostics/render_qualified_closure_figures.py`
- Create: `tests/test_render_qualified_closure_figures.py`
- Create: `/private/tmp/cbf2026-paper-geometric-stability/assets/2026-08-qualified-closure/qualified-closure-summary.pdf`
- Create: `/private/tmp/cbf2026-paper-geometric-stability/assets/2026-08-qualified-closure/error-depth-and-tail.pdf`

**Interfaces:**
- Consumes: terminal PASS confirmatory JSON, manifest, review, and exact source/protocol identities.
- Produces: compact tables/figures and prose whose numbers can be regenerated from committed evidence.

- [ ] **Step 1: Add and run the renderer tests red**

Plot mode availability/containment/controller certificate/mission success with mission-level intervals, and depth plus p95/p99/max errors. Do not ingest mutable raw paths directly into the paper asset generator.

Add a deterministic renderer test that supplies a tiny reviewed-compact
fixture, asserts the expected series/denominators are selected, and verifies
two nonempty vector PDFs are produced. The renderer accepts the compact JSON
path, its expected SHA-256, and an output directory; a hash mismatch aborts.

```bash
conda run -n cbf_env python -m unittest \
  tests.test_render_qualified_closure_figures -v
```

Before the run, add an importable renderer/CLI scaffold that validates
arguments but publishes no PDFs. Expected: assertion-level FAIL on the two
required PDF paths; an import error does not count as red. The fixture must
also prove that a mismatched expected SHA-256 aborts before an output directory
is published.

- [ ] **Step 2: Implement, verify, independently review, and commit the renderer**

Implement deterministic field selection, vector-PDF output, atomic absent
output-directory publication, stable metadata, and no raw-root access. Run the
focused test green, `py_compile`, `git diff --check`, and an independent source
review. Then commit the renderer before generating paper assets:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_render_qualified_closure_figures -v
conda run -n cbf_env python -m py_compile \
  scripts/diagnostics/render_qualified_closure_figures.py
git diff --check
git add scripts/diagnostics/render_qualified_closure_figures.py \
  tests/test_render_qualified_closure_figures.py
git commit -m "feat(paper): render reviewed qualified closure evidence"
```

Expected: source worktree returns to exactly `?? build-diagnostic/` after the
commit, and the renderer source/test commit is recorded in the evidence review
chain.

- [ ] **Step 3: Generate the two paper PDFs from the reviewed real compact JSON**

The only accepted compact input is
`/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-analysis/v1/compact/qualified-closure-confirmatory-v1.json`.
Extract the expected hash from the committed machine-readable evidence-review
attestation, independently verify the actual file hash, and execute:

```bash
cd /private/tmp/cbf2026-diagnostic
reviewed_compact_sha256="$(conda run -n cbf_env python -c \
  'import json,sys; print(json.load(open(sys.argv[1]))["reviewed_compact_sha256"])' \
  docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-confirmatory-v1-evidence-review.json)"
test "$(shasum -a 256 /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-analysis/v1/compact/qualified-closure-confirmatory-v1.json | cut -d ' ' -f 1)" \
  = "$reviewed_compact_sha256"
conda run -n cbf_env python \
  scripts/diagnostics/render_qualified_closure_figures.py \
  --compact-json /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-analysis/v1/compact/qualified-closure-confirmatory-v1.json \
  --expected-sha256 "$reviewed_compact_sha256" \
  --output-dir /private/tmp/cbf2026-paper-geometric-stability/assets/2026-08-qualified-closure
```

Expected: exactly `qualified-closure-summary.pdf` and
`error-depth-and-tail.pdf` are published, both nonempty vector PDFs, with
embedded source/compact hashes matching the reviewed identities.

- [ ] **Step 4: Replace the 44.7438%, 14.6443%, and 999.332 m main-result discussion**

Move them to one concise historical-development sentence or remove them from the main narrative only after the new prospective PASS. Keep the immutable failure record in diagnostics and DRA.

- [ ] **Step 5: Report every primary denominator and failure class**

Include 60 missions, initialization/tuple/interval counts, fresh/availability/containment, wrong-mode incidence and upper bound, rate/row/reset/QP/input/true-distance outcomes, and mission success. Abstentions and launch failures remain visible.

- [ ] **Step 6: Update abstract, contributions, comparison table, simulations, limitations, and conclusion**

Claims must match the conditional theory and observed evidence. Do not call the offline sidecar an online estimator or claim mission-wide deterministic localization bounds.

- [ ] **Step 7: Verify assets and paper**

```bash
cd /private/tmp/cbf2026-paper-geometric-stability
./build.sh
pdfinfo main.pdf | rg 'Pages|Page size|File size'
pdffonts main.pdf
rg -n "44\.7438|14\.6443|999\.332|before submission|unresolved calibration" main.tex
git diff --check
```

Every retained old number must be explicitly historical; no `before submission` blocker may remain.

- [ ] **Step 8: Independent evidence-to-claim and presentation review**

Review every number against the compact JSON, inspect all PDF pages, check captions/legibility/line overflow, citations/DOIs, terminology (`squad`, `UAV`), and absence of unsupported invariance/calibration language.

- [ ] **Step 9: Commit paper evidence edits**

```bash
git add main.tex assets/2026-08-qualified-closure
git commit -m "docs(paper): add qualified closure monte carlo evidence"
```

- [ ] **Step 10: Record the reviewed paper-evidence checkpoint in DRA**

Record the compact-result/figure/PDF hashes, evidence-to-claim review, all
reported denominators, and the remaining journal/package blockers.

---

### Task 15: Final Submission-Readiness Audit and DRA Closure

**Files:**
- Modify: DRA four CBF2026 files.
- Create: source/paper final audit reports under their respective `docs/diagnostics/reviews/` or paper review directory.
- Generate: final paper PDF and submission package only after all audits pass.

**Interfaces:**
- Consumes: clean reviewed source, terminal evidence, revised paper, and current DRA.
- Produces: a traceable submit-ready verdict or an explicit remaining-blocker list.

- [ ] **Step 1: Run full source verification from a clean commit**

Run complete Python and C++ suites, validate all evidence hashes/manifests, verify immutable roots, and confirm no uncommitted relevant changes.

- [ ] **Step 2: Run full paper verification**

Build twice from clean auxiliary state, check undefined references/citations, BibTeX DOI policy, fonts, page count, overfull boxes, figure resolution, and package contents.

- [ ] **Step 3: Run three independent reviews**

Commission separate theory, experiment/evidence, and journal-style manuscript reviews. Any Critical/Important issue reopens the responsible task and requires a scoped fix plus regression/rebuild.

- [ ] **Step 4: Update the DRA current-state rows**

Record final source/paper/DRA commits; design/plan/protocol/config/seed/manifest/report/review/PDF hashes; exact commands; all passed/failed gates; target-journal status; and any remaining submission blocker. Preserve historical negative evidence.

- [ ] **Step 5: Commit DRA and final artifacts**

Use scoped conventional commits without AI attribution. Do not push or submit externally unless the researcher explicitly requests it.

- [ ] **Step 6: Declare completion only if every criterion is met**

Completion requires: reviewed design/plan, all implementation tests, development PASS, terminal independently reviewed 60-mission confirmatory PASS, paper theory/results aligned to evidence, clean PDF/citation/presentation audits, current DRA, and no required work remaining. Otherwise report the exact open gate and continue there.
