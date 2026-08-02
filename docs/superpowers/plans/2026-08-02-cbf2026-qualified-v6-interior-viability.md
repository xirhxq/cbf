# CBF2026 Qualified v6 Interior-Viability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a distributed two-stage local hard-row interior-selection policy and an exact one-step viability development gate so that a future development-v6 campaign cannot repeat the consumed v5 frame-one infeasibility without detection. After the single permitted Task 6 integration exposed insufficient next-state margin under the original `fraction=0.10` policy, the reviewed forward path is an additive, pre-registered margin-policy identity; the gate threshold and frozen seed universe do not change.

**Architecture:** Keep the existing fixed hard-CBF graph, dynamic lower-index FIM graph, allocated endpoint rows, componentwise planar bounds, local Gurobi task QPs, and continuous-time conditional theorem. Before each local task QP, solve a pure two-dimensional Chebyshev LP over that UAV's allocated hard rows and planar component box, then minimize the unchanged task/slack objective subject to a frozen fractional/capped interior floor. Separately, run the exact production binary for the unchanged frozen initial seed universe and admit development-v6 registration only if every actual frame-zero command maps to a one-step state with positive robust barriers and a strictly positive local Chebyshev radius.

**Tech Stack:** C++17, Eigen, doctest, nlohmann/json, Gurobi-backed local QPs, Python 3 in conda environment `cbf_env`, NumPy, Python `unittest`, gzip JSONL, SHA-256 manifests, CMake, Git.

## Global Constraints

- Source worktree: `/private/tmp/cbf2026-diagnostic`, branch `codex/cbf2026-diagnostic`.
- Other workers may modify the same repository. Re-read `git status`, `git diff`, and the touched file immediately before every patch; never revert, overwrite, or reformat another worker's changes.
- Phase 0 creates and independently reviews only the planning baseline. Do not begin implementation in that phase. Tasks 1 onward execute one at a time through subagent-driven development, with a fresh worker and independent review gate per task.
- Obtain the planning baseline commit, tree, parent, worktree status, plan blob identity, v5 terminal-root identities, and current dependency identities from the worktree at execution time. Freeze the observed values in the planning-baseline review; do not copy an unverified SHA from prose or this plan.
- Preserve untracked `build-diagnostic/`; never stage it. Preserve all unrelated tracked or untracked work.
- Consumed v5 raw root `/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v5` and analysis root `/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v5` are immutable. Never delete, edit, resume, repartition, overwrite, or reuse either root. Task 1 independently derives and freezes their exact regular-file tree and terminal-manifest identities.
- Preserve every committed v5 protocol, preflight, authorization, terminal report, review, and lifecycle commit byte-for-byte. v6 uses new filenames, schema/version fields where required, and new absent versioned roots.
- Keep both hard classes at exactly `alpha.coe=0.1` and `alpha.pow=1` for `comm-fixed` and `safety`. Do not test or register `0.2` or `0.5` in this plan.
- The Chebyshev LP consumes only one UAV's allocated hard rows and the planar component box `-25 <= u_x,u_y <= 25`. The yaw coordinate and yaw bounds never enter `rho`.
- Preserve the completed original policy as historical `mode=planar-chebyshev-fraction-cap-v1`, `fraction=0.10`, and `cap=0.10 m/s`; it produced the consumed Task 6 one-seed negative result and must not be reinterpreted or rerun. Before any formal gate, Task 6b adds `mode=planar-chebyshev-fraction-cap-v2`, `fraction=0.131`, and the unchanged `cap=0.10 m/s`. In both versions, `mu = min(cap, fraction * max(0, rho - feasibility_tolerance))`.
- The second-stage task QP keeps the current nominal, soft-CBF slack variables, `k_delta=10`, and objective unchanged. Only local allocated hard-row right-hand sides are strengthened from residual `>= 0` to residual `>= mu`.
- Do not modify canonical hard-row constants or hard-problem hashes. The interior floor is a controller selection policy applied after the exact hard problem has been built and verified.
- Keep execution distributed. No joint swarm QP, centralized command selection, cross-UAV optimization variable, estimator comparison, estimator redesign, or lower-level dynamics enters this plan.
- Do not call the interior floor a sampled-data reserve or use it to prove recursive feasibility. It is a local optimization selection rule that implies the original continuous-time CBF inequalities at the current state.
- Freeze one-step development-gate integration at `dt=0.5 s` with `SingleIntegrate2D` update `x_plus = x + dt * u_applied`.
- At the one-step state, require every robust hard barrier to be strictly greater than `0 m` and every UAV's planar local Chebyshev radius to be at least `0.05 m/s`.
- Use exactly the existing nonreplaceable 100 audit trajectory seeds and the existing first 10 registered trajectory seeds. Do not change their order or values. No seed substitution, clamp, resample, retry, or post-hoc selection is allowed.
- The development gate uses the exact production `Swarm` binary, exact base config, exact primary overlay, exact initial family, and exact implementation identity that will be bound by v6 registration. Test fixtures and mocked subprocesses never satisfy the production gate.
- The formal development gate is once-only for one committed implementation identity. Before launching any seed, it creates an immutable no-replace claim bound to that identity. A failed or interrupted gate is terminal for that identity; repair requires a new implementation commit and newly versioned claim/artifact paths, never overwriting the consumed claim or failed artifact. The Task 6b policy is a single pre-registered candidate: if its unchanged 100-seed formal gate fails, stop this plan; do not retune `fraction`, change the threshold, substitute seeds, or launch a second formal gate under another ad hoc policy.
- Do not create a production development-v6 protocol or preflight until all 100 audit seeds and all 10 registered seeds pass the exact development gate and the gate artifact receives an independent C0/I0/M0 review.
- A generated v6 protocol and preflight are not authorization. Do not create the authorization JSON, authorization commit, campaign root, or analysis root until the researcher supplies new exact user-origin authorization text after seeing that exact protocol and preflight.
- The authorization JSON must bind the exact UTF-8 user text, its SHA-256, the exact protocol and preflight hashes, the exact registered implementation identity, date, kind, and version. Historical approvals, test fixtures, inferred intent, and review verdicts are invalid authorization.
- The registered v6 runner and analyzer are each once-only. No retry, resume, overwrite, reinterpretation, or second analyzer invocation is allowed after their versioned roots are claimed.
- Use `conda run -n cbf_env` for Python. Use `apply_patch` for source/document edits. Generated no-replace evidence artifacts may be emitted only by their tested producer.
- Source-task workers never modify the paper worktree, DRA worktree, submission files, or estimator implementation. The primary agent may separately mirror an independently reviewed checkpoint into `/private/tmp/dra-cbf2026-diagnostic` under the researcher's standing DRA authorization; that documentation-only action has its own DRA commit/review, is not a source-plan artifact, and cannot authorize or substitute for any source gate, experiment, paper claim, or campaign execution.
- Before every suggested commit, obtain explicit user approval as required by `AGENTS.md`. Never add `Co-Authored-By` lines.
- Every implementation task uses a vertical TDD cycle: focused RED test with the intended assertion failure, minimal GREEN implementation, focused regression, full relevant regression, `git diff --check`, independent review, then a scoped commit only after explicit approval. Task 6b is the sole frozen-candidate exception: it first requires the committed pre-implementation amendment review, then TDD and the explicitly approved exact 18-file candidate commit, followed by the separate post-implementation identity review and any scoped fix commits/re-reviews until C0/I0/M0.
- Every new standalone `tests/*.cpp` file starts with `#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN` before `doctest/doctest.h`. Rerun CMake after adding a new test source because the existing test glob lacks `CONFIGURE_DEPENDS`.

## Phase Gates

1. **Phase 0 — planning baseline only:** create this plan's committed planning baseline and independent plan review. No implementation, config, test, binary, gate, protocol, paper, or DRA change is allowed.
2. **Phase 1 — pure controller primitives:** Tasks 1--3 freeze v5 immutability, add the exact Chebyshev primitive, and integrate the distributed interior policy. No production one-step gate or protocol is run.
3. **Phase 2 — evidence and initial-family closure:** Tasks 4--6 independently validate the original policy, freeze the unchanged seed family, and implement the exact one-step gate producer. Task 6's one permitted real seed is a development integration only; its negative result is retained and never rerun.
4. **Phase 2b — pre-registered controller-margin amendment:** Task 6b adds the single `fraction=0.131` policy identity, independently reconstructible evidence, a byte-identical seed-position family, and gate-v2 bindings. Only pure/unit/fixture tests are allowed; no real `Swarm` launch or formal gate occurs in Task 6b.
5. **Phase 3 — lifecycle closure and formal development gate:** Tasks 7--8 bind the amended v6 lifecycle semantics, commit the implementation, execute the unchanged once-only 100-seed development gate, and obtain independent review. Any failure stops the plan before protocol generation and forbids sequential controller tuning in this plan.
6. **Phase 4 — protocol, fresh authorization, and v6 execution:** Tasks 9--10 are conditional on Phase 3 PASS. Generate and independently review the exact protocol/preflight, wait for new verbatim user authorization, then execute the registered runner and analyzer once.

## Exact File Map

Planning baseline only:

- Create `docs/superpowers/plans/reviews/2026-08-02-cbf2026-qualified-v6-interior-viability-plan-review.md`: independent review of this committed plan and its planning identities.
- Create after Task 6: `docs/superpowers/plans/reviews/2026-08-02-cbf2026-qualified-v6-controller-margin-amendment-review.md`: independent review of the committed Task 6b amendment, including its one-candidate/no-retuning rule and execution prohibition.
- Create after Task 6b implementation: `docs/superpowers/plans/reviews/2026-08-02-cbf2026-qualified-v6-controller-margin-implementation-review.md`: independent review of the frozen additive implementation commit and exact staged universe; this never overwrites the pre-implementation amendment review.

Controller primitives and integration:

- Create `include/cbf/HardInteriorSelection.hpp`: pure validation and deterministic planar Chebyshev LP over one `HardConstraintProblem`.
- Create `tests/testHardInteriorSelection.cpp`: exact feasible, infeasible, degenerate, yaw-exclusion, tie-break, and bound-universe tests.
- Modify `include/Robot.hpp`: compute `rho`, derive `mu`, strengthen only the current local task-QP rows, and serialize policy state in `opt`.
- Modify `include/cbf/HybridCertificateGuard.hpp`: require the exact v6 interior-policy config in theorem-aligned runtime validation without changing canonical row/problem hashes.
- Create `config/diagnostics/qualified_mode_hybrid_dcbf_development_v2.json`: dynamic-FIM primary overlay with exact alpha and interior policy.
- Create `config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v2.json`: fixed-FIM replay overlay with the same controller-policy contract.
- Modify `scripts/diagnostics/qualified_config.py`: exact v2 overlay schema and constants while preserving v1 validation.
- Modify `tests/testRobustConstraintConstruction.cpp`: task-QP floor, unchanged objective, component bounds, yaw exclusion, and local-only consumption tests.
- Modify `tests/test_qualified_config.py`: exact v1/v2 acceptance and mutation rejection.
- Create `config/diagnostics/qualified_mode_hybrid_dcbf_development_v3.json` and `config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v3.json`: additive `hard-interior-v3` overlays with `planar-chebyshev-fraction-cap-v2`, pre-registered `fraction=0.131`, and unchanged alpha, cap, tolerance, graph, objective, bounds, and execution mode.

Evidence and independent reconstruction:

- Create `scripts/diagnostics/hard_interior_selection.py`: independent Python reconstruction of planar Chebyshev radius and frozen floor.
- Create `tests/test_hard_interior_selection.py`: parity fixtures and mutation/degeneracy tests.
- Modify `include/Swarm.hpp`: emit exact policy, `rho`, `mu`, and original applied hard-row residual per UAV.
- Modify `scripts/diagnostics/qualified_closure_evidence.py`: exact evidence schema plus independent `rho`, `mu`, and residual verification.
- Modify `scripts/diagnostics/analyze_qualified_closure_campaign.py`: aggregate interior-selection distributions without weakening existing gates or failed-prefix accounting.
- Modify `tests/test_swarm_evidence_stream.py`, `tests/test_qualified_closure_evidence.py`, and `tests/test_analyze_qualified_closure_campaign.py`: real-stream and forged-field rejection.

Frozen v6 initial family and one-step gate:

- Create `config/diagnostics/qualified_initial_family_v2.json`: unchanged template, perturbation algorithm, 100 audit seeds, and first 10 registered seeds, plus the exact v6 gate declaration.
- Create `scripts/diagnostics/qualified_v6_initial_state.py`: exact v2 loader, unchanged seed materialization, static certificate/barrier reconstruction, and one-step audit structures.
- Create `tests/test_qualified_v6_initial_state.py`: v1/v2 positional identity, fixed universes, mutation rejection, and v5 counterexample fixture.
- Create `scripts/diagnostics/audit_qualified_v6_one_step_viability.py`: bounded exact-binary producer for the once-only 100-seed development gate.
- Create `tests/test_audit_qualified_v6_one_step_viability.py`: mocked orchestration, real one-seed subprocess integration, no-retry, and no-replace publication tests.
- Create `config/diagnostics/qualified_initial_family_v3.json`: byte-identical 100-seed positions and gate thresholds; relative to v2, only the additive controller-policy identity, family schema version, and recomputed semantic hash change.
- Modify `scripts/diagnostics/qualified_v6_initial_state.py`, `scripts/diagnostics/audit_qualified_v6_one_step_viability.py`, and their tests: retain historical v2/v1-policy validation while binding qualifying execution to the exact v3/v2-policy bytes and gate-v2 schema.

Lifecycle and immutable predecessor binding:

- Create `config/diagnostics/qualified_v6_predecessor_v5_identity_v1.json`: exact identities derived from current immutable v5 roots during Task 1.
- Modify `scripts/diagnostics/register_qualified_closure_campaign.py`: version-keyed v5/v6 predecessor validation, v6 family/gate bindings, absent v6 roots, and authorization binding.
- Modify `scripts/diagnostics/run_qualified_closure_campaign.py`: development-v6 schedule/materialization and pre-root verification while preserving consumed v5 behavior.
- Modify `tests/test_register_qualified_closure_campaign.py` and `tests/test_run_qualified_closure_campaign.py`: v5 regression, v6 gate/pass binding, root collision, schedule, and fresh-authorization tests.

Conditional artifacts created only after the required gates:

- Create `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2-claim.json`: immutable pre-launch claim for the pre-registered margin-policy identity, created before the first formal-gate child launch.
- Create `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2.json`: terminal once-only formal development-gate artifact, when execution reaches terminal publication.
- Create `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2-review.md`: independent gate review.
- Create `docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v6-protocol.json` and `.md`: only after gate PASS/review.
- Create `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v6-preflight.md`: independent C0/I0/M0 preflight.
- Create `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v6-authorization.json`: only after fresh verbatim user approval.
- Create `docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v6.md` and `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v6-review.md`: only after the registered roots are terminal.

---

### Task 0: Commit and Independently Review the Planning Baseline

**Files:**
- Verify: `docs/superpowers/plans/2026-08-02-cbf2026-qualified-v6-interior-viability.md`
- Create: `docs/superpowers/plans/reviews/2026-08-02-cbf2026-qualified-v6-interior-viability-plan-review.md`

**Interfaces:**
- Consumes: current source HEAD/tree/status and this complete plan.
- Produces: one planning-only commit and an independent review that freezes the observed planning identities; no implementation behavior.

- [ ] **Step 1: Run the planning RED precondition**

```bash
cd /private/tmp/cbf2026-diagnostic
test -f docs/superpowers/plans/2026-08-02-cbf2026-qualified-v6-interior-viability.md
test ! -e docs/superpowers/plans/reviews/2026-08-02-cbf2026-qualified-v6-interior-viability-plan-review.md
git status --short
git diff --check -- docs/superpowers/plans/2026-08-02-cbf2026-qualified-v6-interior-viability.md
```

Expected: the plan exists, the review is absent, the plan has no whitespace errors, and `build-diagnostic/` plus this plan are the only newly introduced paths. If another worker has changed the worktree, record and preserve those paths rather than staging them.

- [ ] **Step 2: Freeze the planning baseline from live git state**

```bash
git rev-parse HEAD
git rev-parse 'HEAD^{tree}'
git rev-parse 'HEAD^'
git branch --show-current
git status --porcelain=v1
shasum -a 256 docs/superpowers/plans/2026-08-02-cbf2026-qualified-v6-interior-viability.md
wc -c -l docs/superpowers/plans/2026-08-02-cbf2026-qualified-v6-interior-viability.md
```

Expected: every command succeeds. Record these observed values verbatim in the review after the plan is committed; do not predict the plan blob or commit identity.

- [ ] **Step 3: Self-review the plan before staging**

```bash
rg -n 'T[B]D|T[O]DO|implement l[a]ter|fill i[n]|similar to T[a]sk|appropriate error handlin[g]' \
  docs/superpowers/plans/2026-08-02-cbf2026-qualified-v6-interior-viability.md
rg -n '0\.1|0\.10|0\.05|0\.5|100 audit|10 registered|no clamp|no retry|fresh.*authorization' \
  docs/superpowers/plans/2026-08-02-cbf2026-qualified-v6-interior-viability.md
```

Expected: the placeholder scan returns no matches; the specification scan finds every frozen numeric and lifecycle requirement.

- [ ] **Step 4: Commit only the plan after explicit user approval**

Suggested commit:

```bash
git add docs/superpowers/plans/2026-08-02-cbf2026-qualified-v6-interior-viability.md
git diff --cached --name-only
git commit -m "docs(cbf2026): plan v6 interior viability"
```

Expected staged path: exactly the plan. Do not stage `build-diagnostic/` or another worker's paths.

- [ ] **Step 5: Create the independent plan review**

The reviewer must compare every Global Constraint with the tasks, verify all interfaces and commands, record the committed plan blob SHA obtained with `git show HEAD:<path> | shasum -a 256`, and report Critical/Important/Minor counts. The acceptable verdict is C0/I0/M0. Any issue is patched into a new planning commit and reviewed again before Task 1.

- [ ] **Step 6: Run the planning GREEN gate**

```bash
git show HEAD:docs/superpowers/plans/2026-08-02-cbf2026-qualified-v6-interior-viability.md >/dev/null
test -f docs/superpowers/plans/reviews/2026-08-02-cbf2026-qualified-v6-interior-viability-plan-review.md
git diff --check -- docs/superpowers/plans/reviews/2026-08-02-cbf2026-qualified-v6-interior-viability-plan-review.md
rg -n 'C0/I0/M0|Critical: 0|Important: 0|Minor: 0' \
  docs/superpowers/plans/reviews/2026-08-02-cbf2026-qualified-v6-interior-viability-plan-review.md
```

Expected: committed plan exists, review exists, review is whitespace-clean, and it records an all-zero finding verdict.

- [ ] **Step 7: Commit only the review after explicit user approval**

Suggested commit:

```bash
git add docs/superpowers/plans/reviews/2026-08-02-cbf2026-qualified-v6-interior-viability-plan-review.md
git diff --cached --name-only
git commit -m "docs(cbf2026): review v6 interior viability plan"
```

After this commit, switch to `superpowers:subagent-driven-development`; dispatch a fresh worker for Task 1 and a separate reviewer before Task 2.

---

### Task 1: Freeze the Immutable v5 Predecessor and Baseline Contracts

**Files:**
- Create: `config/diagnostics/qualified_v6_predecessor_v5_identity_v1.json`
- Modify: `scripts/diagnostics/register_qualified_closure_campaign.py`
- Modify: `tests/test_register_qualified_closure_campaign.py`
- Verify: `scripts/diagnostics/analyze_qualified_closure_campaign.py`
- Verify: `tests/test_analyze_qualified_closure_campaign.py`

**Interfaces:**
- Consumes: `_directory_tree_identity(root: Path) -> dict`, immutable v5 roots, and terminal report identities.
- Produces: `load_v6_predecessor_identity(path: Path) -> dict` and `verify_v6_predecessor_state(identity: Mapping) -> None`; v5 registration behavior remains intact.

- [ ] **Step 1: Derive the live predecessor identities without writing them**

```bash
cd /private/tmp/cbf2026-diagnostic
conda run -n cbf_env python - <<'PY'
import json
from pathlib import Path
from scripts.diagnostics.register_qualified_closure_campaign import _directory_tree_identity, _sha256
for label, root_text in (
    ("raw", "/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v5"),
    ("analysis", "/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v5"),
):
    root = Path(root_text)
    manifest = root / "manifest.json"
    print(json.dumps({"label": label, "root": str(root), **_directory_tree_identity(root), "manifest_name": manifest.name, "manifest_sha256": _sha256(manifest)}, sort_keys=True))
PY
```

Expected: two finite exact identity records. Independently compare them with the committed v5 terminal report; any disagreement stops Task 1 and forbids editing the roots.

- [ ] **Step 2: Write the failing predecessor tests**

```python
def test_v6_predecessor_identity_matches_terminal_v5_roots(self):
    identity = registrar.load_v6_predecessor_identity(
        Path("config/diagnostics/qualified_v6_predecessor_v5_identity_v1.json")
    )
    registrar.verify_v6_predecessor_state(identity)

def test_v6_predecessor_rejects_tree_or_manifest_mutation(self):
    identity = self.copy_v6_predecessor_fixture()
    identity["terminal"]["raw"]["tree_sha256"] = "0" * 64
    with self.assertRaisesRegex(ValueError, "v5 predecessor"):
        registrar.verify_v6_predecessor_state(identity)
```

- [ ] **Step 3: Run RED**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_register_qualified_closure_campaign.QualifiedClosureV6PredecessorTests -v
```

Expected: FAIL because the literal v6 predecessor file and loader do not exist. An import/syntax error does not count.

- [ ] **Step 4: Add the exact identity file and minimal verifier**

Create the JSON with `schema_version`, both literal root paths, `files`, `logical_bytes`, `tree_sha256`, `manifest_name`, and `manifest_sha256` copied exactly from Step 1. The loader rejects duplicate keys, symlinks, extra/missing fields, noncanonical hashes, alternate paths, and self-rehashed substitutions. The verifier recomputes both trees and manifests without writing either root.

Preserve the current v5 predecessor verifier as a versioned historical path. Do not replace its constants with v6 values.

- [ ] **Step 5: Run GREEN and the existing analyzer regression**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_register_qualified_closure_campaign.QualifiedClosureV6PredecessorTests \
  tests.test_analyze_qualified_closure_campaign.QualifiedClosureAnalyzerExecutionTests.test_incomplete_controller_abort_remains_in_synthetic_missing_universe -v
```

Expected: PASS. The analyzer test confirms the consumed v5 incomplete frame remains missing; do not modify its already-fixed `runtime.complete is True` predicate.

- [ ] **Step 6: Verify scope and review**

```bash
git diff --check
git diff --name-only
git diff -- scripts/diagnostics/analyze_qualified_closure_campaign.py
```

Expected: only the three Task 1 implementation paths differ; analyzer diff is empty. Independent reviewer recomputes both v5 trees and returns C0/I0/M0.

- [ ] **Step 7: Commit after explicit approval**

Suggested commit:

```bash
git add config/diagnostics/qualified_v6_predecessor_v5_identity_v1.json \
  scripts/diagnostics/register_qualified_closure_campaign.py \
  tests/test_register_qualified_closure_campaign.py
git commit -m "test(cbf2026): freeze v5 predecessor for v6"
```

---

### Task 2: Implement the Pure Planar Chebyshev Primitive

**Files:**
- Create: `include/cbf/HardInteriorSelection.hpp`
- Create: `tests/testHardInteriorSelection.cpp`

**Interfaces:**
- Consumes: `cbf2026::HardConstraintProblem` from `include/cbf/HybridCertificateGuard.hpp`.
- Produces:

```cpp
namespace cbf2026 {
struct PlanarChebyshevResult {
    double radius;
    Eigen::Vector2d witness;
    std::vector<std::size_t> tightHardRows;
};
PlanarChebyshevResult solvePlanarHardRowChebyshev(
    const HardConstraintProblem& problem,
    double feasibilityTolerance = 1e-9
);
double frozenInteriorFloor(
    double radius,
    double fraction = 0.10,
    double capMps = 0.10,
    double feasibilityTolerance = 1e-9
);
}
```

- [ ] **Step 1: Write focused failing tests**

```cpp
TEST_CASE("planar Chebyshev radius uses local hard rows and component box") {
    const auto problem = boxWithRows({
        row({1.0, 0.0, 0.0}, 2.0),
        row({-1.0, 0.0, 0.0}, 2.0),
        row({0.0, 1.0, 0.0}, 3.0),
        row({0.0, -1.0, 0.0}, 3.0),
    }, 25.0, 0.35);
    const auto result = cbf2026::solvePlanarHardRowChebyshev(problem);
    CHECK(result.radius == doctest::Approx(2.0));
    CHECK(result.witness.isApprox(Eigen::Vector2d(0.0, -1.0), 1e-12));
}

TEST_CASE("yaw never enters planar Chebyshev radius") {
    auto problem = boxWithRows({row({1.0, 0.0, 0.0}, 1.0)}, 25.0, 1e-12);
    CHECK(cbf2026::solvePlanarHardRowChebyshev(problem).radius
          == doctest::Approx(26.0));
}

TEST_CASE("frozen floor is fractional capped and tolerance aware") {
    CHECK(cbf2026::frozenInteriorFloor(0.8) == doctest::Approx(0.08));
    CHECK(cbf2026::frozenInteriorFloor(2.0) == doctest::Approx(0.10));
    CHECK(cbf2026::frozenInteriorFloor(5e-10) == doctest::Approx(0.0));
}
```

Also add rejection tests for missing `+/-25` planar bounds, nonzero hard-row yaw coefficients, nonfinite data, duplicate bounds, no finite vertex, and deterministic lexicographic witness selection when radii tie.

- [ ] **Step 2: Reconfigure and run RED**

```bash
conda run -n cbf_env cmake -S . -B build-diagnostic -DBUILD_TESTING=ON
conda run -n cbf_env cmake --build build-diagnostic --target testHardInteriorSelection -j2
```

Expected: compile FAIL because `HardInteriorSelection.hpp` and its functions are absent.

- [ ] **Step 3: Implement exact three-plane enumeration**

Represent (z=(u_x,u_y,\rho)). Each hard row becomes
`(-a_x, -a_y, 1) dot z <= c`; component bounds become
`(+/-1, 0, 0) dot z <= 25` and `(0, +/-1, 0) dot z <= 25`.
Enumerate every three-plane intersection, reject singular/nonfinite candidates,
check the entire plane universe with `feasibilityTolerance`, maximize `rho`,
and break equal-radius ties lexicographically by `(u_x,u_y)`.

Validate that every hard coefficient has exactly three entries and zero yaw
coefficient, and that the six input bounds contain the exact four planar and
two yaw bounds even though yaw is excluded from `rho`.

- [ ] **Step 4: Run GREEN and primitive regressions**

```bash
conda run -n cbf_env cmake --build build-diagnostic --target testHardInteriorSelection -j2
./build-diagnostic/testHardInteriorSelection
./build-diagnostic/testHybridCertificateGuard
./build-diagnostic/testAllocatedPairwiseCBF
```

Expected: all doctest cases PASS.

- [ ] **Step 5: Scope review and commit**

```bash
git diff --check
git diff --name-only
```

Expected paths: exactly the new header and test. Independent reviewer checks the LP signs against `a^T u + c >= rho`, confirms yaw exclusion, and returns C0/I0/M0.

Suggested commit after explicit approval:

```bash
git add include/cbf/HardInteriorSelection.hpp tests/testHardInteriorSelection.cpp
git commit -m "feat(cbf): add planar hard-row Chebyshev solver"
```

---

### Task 3: Apply the Frozen Interior Policy in Each Distributed Task QP

**Files:**
- Modify: `include/Robot.hpp`
- Modify: `include/cbf/HybridCertificateGuard.hpp`
- Create: `config/diagnostics/qualified_mode_hybrid_dcbf_development_v2.json`
- Create: `config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v2.json`
- Modify: `scripts/diagnostics/qualified_config.py`
- Modify: `tests/testRobustConstraintConstruction.cpp`
- Modify: `tests/test_qualified_config.py`

**Interfaces:**
- Consumes: `solvePlanarHardRowChebyshev` and `frozenInteriorFloor` from Task 2.
- Produces: exact merged config object `cbfs.hard-interior-selection` and `Robot::opt["hard_interior_selection"]` with `mode`, `fraction`, `cap_mps`, `feasibility_tolerance_mps`, `planar_chebyshev_radius_mps`, and `enforced_floor_mps`.

- [ ] **Step 1: Add RED tests for exact config and strengthened local rows**

```cpp
TEST_CASE("theorem task QP enforces the frozen local interior floor") {
    Robot robot = paperRobotWithCommittedHardProblem();
    robot.optimise();
    const auto policy = robot.opt.at("hard_interior_selection");
    CHECK(policy.at("mode") == "planar-chebyshev-fraction-cap-v1");
    CHECK(policy.at("fraction") == doctest::Approx(0.10));
    CHECK(policy.at("cap_mps") == doctest::Approx(0.10));
    const double floor = policy.at("enforced_floor_mps");
    for (const auto& row : robot.lastConsumedHardConstraintProblem->rows) {
        CHECK(row.constant + row.coefficients.dot(robot.model->getControlInput())
              >= floor - 1e-7);
    }
}

TEST_CASE("interior selection consumes no other UAV control variable") {
    Robot robot = paperRobotWithCommittedHardProblem();
    robot.optimise();
    CHECK(robot.lastConsumedHardConstraintProblem->owner == robot.id);
    CHECK(robot.model->getControlInput().size() == 3);
}
```

Python tests require both v2 overlays to include exact alpha `{coe: 0.1, pow: 1}` for both hard classes and exact policy values; reject `0.2`, `0.5`, altered fraction/cap, extra keys, yaw-in-radius flags, and centralized execution.

- [ ] **Step 2: Run RED**

```bash
conda run -n cbf_env python -m unittest tests.test_qualified_config -v
conda run -n cbf_env cmake --build build-diagnostic --target testRobustConstraintConstruction -j2
./build-diagnostic/testRobustConstraintConstruction \
  --test-case="theorem task QP enforces the frozen local interior floor"
```

Expected: Python FAIL because v2 schema/overlays are absent; C++ FAIL because policy evidence and strengthened RHS are absent.

- [ ] **Step 3: Add the exact v2 config schema**

Use this exact overlay fragment in both v2 files:

```json
{
  "cbfs": {
    "hard-interior-selection": {
      "mode": "planar-chebyshev-fraction-cap-v1",
      "fraction": 0.1,
      "cap-mps": 0.1,
      "feasibility-tolerance-mps": 1e-9
    },
    "without-slack": {
      "safety": {"on": true, "mode": "allocated-pairwise", "alpha": {"coe": 0.1, "pow": 1}},
      "comm-fixed": {"on": true, "mode": "allocated-pairwise", "alpha": {"coe": 0.1, "pow": 1}}
    }
  },
  "execute": {"execution-mode": "distributed", "time-step": 0.5}
}
```

Retain every estimator/deployment field from the v1 overlays exactly. Primary keeps `dynamic-lower-index`; ablation keeps `fixed-cbf-only`.

- [ ] **Step 4: Integrate the two-stage local solve**

In theorem-aligned `Robot::optimise`, compute the Chebyshev result from the already committed local problem before adding task-QP hard rows. Reject a radius below `-1e-9`. Compute `mu` with Task 2. Add each hard row to the existing optimizer with RHS `-row.constant + mu`; leave bounds, nominal, slack dimensions, `k_delta`, and objective untouched. Store the exact policy and values in `opt`.

Extend theorem-mode config validation to require exact policy and exact hard-class alpha values. Do not add `mu` to `HardConstraintRow`, `HardConstraintProblem`, or canonical hashes.

- [ ] **Step 5: Run GREEN and regressions**

```bash
conda run -n cbf_env python -m unittest tests.test_qualified_config -v
conda run -n cbf_env cmake --build build-diagnostic --target \
  testRobustConstraintConstruction testHybridCertificateGuard -j2
./build-diagnostic/testRobustConstraintConstruction
./build-diagnostic/testHybridCertificateGuard
```

Expected: all PASS. Existing non-theorem and v1 config paths remain accepted exactly as before.

- [ ] **Step 6: Review and commit**

```bash
git diff --check
git diff --name-only
```

Independent reviewer must verify: local problem only, no neighbor commands, no changed hard hashes, alpha still 0.1, yaw excluded from radius, `mu <= rho`, and original task objective unchanged. Required verdict: C0/I0/M0.

Suggested commit after explicit approval:

```bash
git add include/Robot.hpp include/cbf/HybridCertificateGuard.hpp \
  config/diagnostics/qualified_mode_hybrid_dcbf_development_v2.json \
  config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v2.json \
  scripts/diagnostics/qualified_config.py tests/testRobustConstraintConstruction.cpp \
  tests/test_qualified_config.py
git commit -m "feat(cbf): select distributed hard-row interior commands"
```

---

### Task 4: Add Independent Interior-Policy Evidence Reconstruction

**Files:**
- Create: `scripts/diagnostics/hard_interior_selection.py`
- Create: `tests/test_hard_interior_selection.py`
- Modify: `include/Swarm.hpp`
- Modify: `scripts/diagnostics/qualified_closure_evidence.py`
- Modify: `scripts/diagnostics/analyze_qualified_closure_campaign.py`
- Modify: `tests/test_swarm_evidence_stream.py`
- Modify: `tests/test_qualified_closure_evidence.py`
- Modify: `tests/test_analyze_qualified_closure_campaign.py`

**Interfaces:**
- Consumes: serialized `normal_problem`, applied command, and `Robot::opt["hard_interior_selection"]`.
- Produces:

```python
@dataclass(frozen=True)
class PlanarChebyshevAudit:
    radius_mps: float
    witness: tuple[float, float]
    tight_hard_row_indices: tuple[int, ...]

def solve_planar_hard_row_chebyshev(problem: Mapping, *, tolerance_mps: float = 1e-9) -> PlanarChebyshevAudit: ...
def frozen_interior_floor(radius_mps: float, *, fraction: float = 0.10, cap_mps: float = 0.10, tolerance_mps: float = 1e-9) -> float: ...
```

- [ ] **Step 1: Add RED parity and forgery tests**

```python
def test_python_chebyshev_matches_registered_cpp_fixture(self):
    audit = solve_planar_hard_row_chebyshev(self.fixture_problem)
    self.assertAlmostEqual(audit.radius_mps, self.expected_cpp_radius, places=12)

def test_controller_node_rejects_forged_radius_or_floor(self):
    row = self.valid_controller_row()
    row["runtime"]["nodes"][0]["hard_interior_selection"]["enforced_floor_mps"] += 0.01
    with self.assertRaisesRegex(ValueError, "interior"):
        validate_controller_primitive_schema(row)
```

Add mutations for alpha, fraction, cap, tolerance, `rho`, `mu`, applied residual below `mu`, nonzero yaw coefficient, altered normal problem, and a missing policy field.

- [ ] **Step 2: Run RED**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_hard_interior_selection \
  tests.test_qualified_closure_evidence \
  tests.test_analyze_qualified_closure_campaign -v
```

Expected: focused new cases FAIL because the Python reconstructor and evidence fields are absent.

- [ ] **Step 3: Implement independent Python enumeration and exact schema**

Port the mathematical LP, not the C++ control flow. Parse only canonical numeric problem data, enumerate all three-plane vertices with NumPy, apply the same tolerance and lexicographic tie-break, and independently derive `mu`.

Emit this exact per-node object from `Swarm`:

```json
{
  "mode": "planar-chebyshev-fraction-cap-v1",
  "fraction": 0.1,
  "cap_mps": 0.1,
  "feasibility_tolerance_mps": 1e-9,
  "planar_chebyshev_radius_mps": 0.0,
  "enforced_floor_mps": 0.0,
  "minimum_original_hard_residual_mps": 0.0
}
```

Numeric zeros above denote schema types only; runtime values must be exact measured values. Validator independently recomputes radius/floor/problem hash and requires applied original hard residual `>= mu - 1e-7`.

- [ ] **Step 4: Run GREEN including a real Swarm stream**

```bash
conda run -n cbf_env cmake --build build-diagnostic --target Swarm -j2
conda run -n cbf_env python -m unittest \
  tests.test_hard_interior_selection \
  tests.test_qualified_closure_evidence \
  tests.test_analyze_qualified_closure_campaign \
  tests.test_swarm_evidence_stream -v
```

Expected: all PASS; real evidence has exactly 14 policy records at a complete controller frame and every recorded value independently reconstructs.

- [ ] **Step 5: Review and commit**

Independent reviewer checks C++/Python sign parity, exact field universe, no weakening of failed-prefix denominators, and no acceptance based solely on producer booleans. Required verdict: C0/I0/M0.

Suggested commit after explicit approval:

```bash
git add scripts/diagnostics/hard_interior_selection.py \
  tests/test_hard_interior_selection.py include/Swarm.hpp \
  scripts/diagnostics/qualified_closure_evidence.py \
  scripts/diagnostics/analyze_qualified_closure_campaign.py \
  tests/test_swarm_evidence_stream.py tests/test_qualified_closure_evidence.py \
  tests/test_analyze_qualified_closure_campaign.py
git commit -m "feat(evidence): verify local interior selection"
```

---

### Task 5: Freeze the Unchanged v6 Initial-State Universe

**Files:**
- Create: `config/diagnostics/qualified_initial_family_v2.json`
- Create: `scripts/diagnostics/qualified_v6_initial_state.py`
- Create: `tests/test_qualified_v6_initial_state.py`

**Interfaces:**
- Consumes: v1 template positions, perturbation algorithm, exact seed sequences, certificate formulas, and Task 4 Python Chebyshev solver.
- Produces: `load_qualified_v6_initial_family`, `audit_frozen_v6_initial_family`, `materialize_v6_seed_positions`, and `reconstruct_v6_one_step_state`.

- [ ] **Step 1: Add RED identity and mutation tests**

```python
def test_v2_positions_are_bitwise_identical_to_v1_for_all_audit_seeds(self):
    for seed in range(2026080201, 2026080301):
        self.assertEqual(
            materialize_v6_seed_positions(self.v2, seed),
            materialize_seed_positions(self.v1, seed),
        )

def test_v2_freezes_same_registered_prefix_without_selection(self):
    self.assertEqual(
        self.v2["schedule"]["registered_trajectory_seeds"],
        list(range(2026080201, 2026080211)),
    )
    self.assertFalse(self.v2["perturbation"]["clamp"])
    self.assertFalse(self.v2["perturbation"]["resample"])
```

Add exact rejection tests for any template, perturbation radius/method, seed/order, alpha, graph count, component bound, policy fraction/cap, `dt`, next-barrier threshold, next-radius threshold, clamp, resample, or retry mutation.

- [ ] **Step 2: Run RED**

```bash
conda run -n cbf_env python -m unittest tests.test_qualified_v6_initial_state -v
```

Expected: FAIL because the v2 family/module do not exist.

- [ ] **Step 3: Implement v2 as a strict additive contract**

The v2 family repeats the exact v1 template and static frozen summaries, changes only its schema/namespace, and adds:

```json
{
  "controller_policy": {
    "class_k_coefficient": 0.1,
    "class_k_power": 1,
    "mode": "planar-chebyshev-fraction-cap-v1",
    "fraction": 0.1,
    "cap_mps": 0.1,
    "planar_component_max_mps": 25.0,
    "yaw_enters_radius": false
  },
  "one_step_gate": {
    "dt_s": 0.5,
    "minimum_next_barrier_m": 0.0,
    "barrier_comparison": "strictly-greater",
    "minimum_next_local_radius_mps": 0.05,
    "clamp": false,
    "resample": false,
    "retry": false
  }
}
```

Do not call this static family file proof that the gate passed; it declares the gate. The later exact-binary artifact supplies the observed result.

- [ ] **Step 4: Run GREEN and v1 regressions**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_qualified_v6_initial_state tests.test_qualified_initial_state -v
```

Expected: PASS; v1 remains byte-compatible and the v2 positions are exactly equal over all 100 seeds.

- [ ] **Step 5: Review and commit**

Independent reviewer independently hashes every materialized v1/v2 position sequence, checks the first-10 subset, and reports C0/I0/M0.

Suggested commit after explicit approval:

```bash
git add config/diagnostics/qualified_initial_family_v2.json \
  scripts/diagnostics/qualified_v6_initial_state.py \
  tests/test_qualified_v6_initial_state.py
git commit -m "feat(diagnostics): freeze v6 one-step initial family"
```

---

### Task 6: Implement the Exact-Binary One-Step Development Gate

**Files:**
- Create: `scripts/diagnostics/audit_qualified_v6_one_step_viability.py`
- Create: `tests/test_audit_qualified_v6_one_step_viability.py`

**Interfaces:**
- Consumes: exact binary/base/primary/family paths; all 100 frozen seeds; actual frame-zero controller evidence; Task 4/5 independent reconstruction.
- Produces an immutable pre-launch claim and a no-replace terminal JSON artifact through:

```python
class OneStepOperations(Protocol):
    def launch_seed(self, *, seed: int, config_path: Path) -> Mapping: ...

def audit_one_step_universe(
    *, binary: Path, base_config: Path, primary_config: Path,
    initial_family: Path, claim: Path, output: Path, project_root: Path,
    operations: OneStepOperations | None = None,
) -> dict: ...
```

- [ ] **Step 1: Add RED orchestration tests**

```python
def test_gate_launches_each_frozen_seed_exactly_once(self):
    operations = FakeOperations()
    result = audit_one_step_universe(**self.arguments(operations=operations))
    self.assertEqual(operations.launched_seeds, list(range(2026080201, 2026080301)))
    self.assertEqual(len(set(operations.launched_seeds)), 100)

def test_any_nonpositive_barrier_or_small_next_radius_terminally_fails(self):
    operations = FakeOperations(next_barrier=0.0, next_radius=0.049999)
    result = audit_one_step_universe(**self.arguments(operations=operations))
    self.assertEqual(result["status"], "failed")
    self.assertFalse(result["passed"])

def test_existing_claim_or_output_is_never_overwritten(self):
    self.claim.write_text("occupied", encoding="utf-8")
    with self.assertRaises(FileExistsError):
        audit_one_step_universe(**self.arguments())
    self.claim.unlink()
    self.output.write_text("occupied", encoding="utf-8")
    with self.assertRaises(FileExistsError):
        audit_one_step_universe(**self.arguments())

def test_hard_interruption_consumes_identity_before_first_launch(self):
    operations = InterruptingOperations()
    with self.assertRaises(KeyboardInterrupt):
        audit_one_step_universe(**self.arguments(operations=operations))
    self.assertTrue(self.claim.is_file())
    self.assertFalse(self.output.exists())
    with self.assertRaises(FileExistsError):
        audit_one_step_universe(**self.arguments())
```

Also test child nonzero exit, timeout, malformed/multiple frame-zero rows, wrong seed, altered config hash, applied residual below `mu`, component-bound violation, retry attempt, immutable claim identity, claim/output cross-binding, and partial terminal publication.

- [ ] **Step 2: Run RED**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_audit_qualified_v6_one_step_viability -v
```

Expected: FAIL because the producer does not exist.

- [ ] **Step 3: Implement bounded once-per-seed execution**

Before creating any temporary config or launching any child, require both `claim` and `output` absent. Create `claim` with an `O_CREAT|O_EXCL` no-replace write, fsync its exact implementation/binary/config/family/seed-universe identities, and never modify or delete it. An uncatchable interruption may leave only this claim; that still permanently consumes the implementation identity and blocks another launch. Caught child failures/timeouts publish a separate terminal failed `output` that binds the claim SHA-256.

For each frozen seed, materialize one config in a fresh private temporary directory under `/private/tmp`, set exact positions, exact seed, `time-total=0.5`, evidence mode, and no campaign root. Invoke the binary once, parse exactly one complete frame-zero controller interval, reject any extra/missing identity, and discard temporary files after a no-replace terminal output is published. Do not retain decompressed evidence or retry a seed.

Independently compute every applied original hard residual, `x_plus`, dynamic FIM certificate, 119 robust barriers, 14 next local hard problems, and 14 next planar radii. A passing record requires:

```python
all(applied_original_residual >= enforced_floor - 1e-7)
and max(abs(u_x), abs(u_y)) <= 25.0 + 1e-7
and all(next_barrier > 0.0)
and all(next_radius >= 0.05)
```

The terminal artifact records and verifies the immutable claim SHA-256, live HEAD/tree, binary/config/family identities, every seed result, registered/audit summaries, child-launch count, zero retry count, and a terminal status. It must never claim long-horizon safety or recursive feasibility.

- [ ] **Step 4: Run GREEN with fixtures, then a real one-seed integration test**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_audit_qualified_v6_one_step_viability -v
conda run -n cbf_env cmake --build build-diagnostic --target Swarm -j2
CBF_SWARM_BINARY=build-diagnostic/Swarm \
  conda run -n cbf_env python -m unittest \
  tests.test_audit_qualified_v6_one_step_viability.QualifiedV6OneStepRealBinaryTests -v
```

Expected: unit tests PASS; real integration launches exactly one declared seed once, validates its frame-zero command, and writes only to its test temporary directory. This is not the formal 100-seed gate.

- [ ] **Step 5: Review and commit**

Independent reviewer checks the claim is published and fsynced before launch, process count, exact seed universe, no retry/no replace, hard-interruption behavior, temporary cleanup, state-update chronology, graph distinction, strict barrier comparison, and `rho>=0.05`. Required verdict C0/I0/M0.

Suggested commit after explicit approval:

```bash
git add scripts/diagnostics/audit_qualified_v6_one_step_viability.py \
  tests/test_audit_qualified_v6_one_step_viability.py
git commit -m "feat(diagnostics): add v6 one-step viability gate"
```

---

### Task 6b: Pre-register the Additive Controller-Margin Policy

**Why this task exists:** The single permitted Task 6 real integration for seed `2026080201` was structurally valid but terminally negative only because the reconstructed next local radius was `0.045836701551219286 m/s`, below the unchanged `0.05 m/s` gate. Its current minimum radius was `0.862613426899052 m/s`; the applied minimum residual was already the v1 floor to numerical tolerance. This is evidence that the controller selected a command on the strengthened current-row boundary, not evidence that the gate threshold or seed should be changed. The old execution is consumed and must never be rerun.

The candidate below is chosen once from the pre-existing frozen 100-seed current-state universe, not by fitting the observed next-radius deficit. The independently reconstructed minimum current local radius across all `100 * 14 = 1400` frame-zero problems is `0.7658252531927233 m/s`. Therefore a fraction of at least `0.1 / (0.7658252531927233 - 1e-9) = 0.1305780917...` saturates every frame-zero floor at the already frozen `0.10 m/s` cap. Pre-register exactly `0.131`; do not search a grid or try another fraction if the formal gate later fails.

**Files:**
- Create before implementation: `docs/superpowers/plans/reviews/2026-08-02-cbf2026-qualified-v6-controller-margin-amendment-review.md`
- Create after the implementation commit: `docs/superpowers/plans/reviews/2026-08-02-cbf2026-qualified-v6-controller-margin-implementation-review.md`
- Create: `config/diagnostics/qualified_mode_hybrid_dcbf_development_v3.json`
- Create: `config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v3.json`
- Create: `config/diagnostics/qualified_initial_family_v3.json`
- Modify: `include/Robot.hpp`
- Modify: `include/cbf/HybridCertificateGuard.hpp`
- Modify: `scripts/diagnostics/qualified_config.py`
- Modify: `scripts/diagnostics/qualified_closure_evidence.py`
- Modify: `scripts/diagnostics/analyze_qualified_closure_campaign.py`
- Modify: `scripts/diagnostics/qualified_v6_initial_state.py`
- Modify: `scripts/diagnostics/audit_qualified_v6_one_step_viability.py`
- Modify the directly corresponding C++ and Python tests only.

**Exact additive identity:**

```json
{
  "qualified-controller": {"schema-version": "hard-interior-v3"},
  "hard-interior-selection": {
    "mode": "planar-chebyshev-fraction-cap-v2",
    "fraction": 0.131,
    "cap-mps": 0.1,
    "feasibility-tolerance-mps": 1e-9
  }
}
```

The v3 overlays must otherwise be structurally and numerically identical to their v2 predecessors. Both hard classes remain `alpha.coe=0.1`, `alpha.pow=1`; component bounds remain `+/-25 m/s`; yaw remains excluded from `rho`; `k_delta=10`, task/slack objective, hard rows, allocated ownership, fixed hard-CBF graph, dynamic-lower-index primary FIM graph, fixed-FIM ablation graph, and distributed execution remain unchanged. The v3 initial family freezes `schema_version=cbf2026-qualified-initial-family-v3` and retains `namespace=cbf2026-v6-initial`. Relative to v2, every JSON token except `schema_version`, `controller_policy`, and `semantic_sha256` is identical; the semantic SHA-256 is recomputed over canonical UTF-8 JSON after excluding only `semantic_sha256`, using sorted keys, no whitespace, `ensure_ascii=False`, and `allow_nan=False`. It materializes exactly the same IEEE-754 positions, ordered 100 audit seeds, first 10 registered seeds, `dt=0.5 s`, barrier threshold `>0`, radius threshold `>=0.05`, and no clamp/resample/retry fields as v2.

The v1 policy and v2 config/family files remain accepted only as historical identities. Do not alter their bytes. Evidence reconstruction and analysis must select the exact `(mode, fraction, cap, tolerance)` tuple from the declared controller marker; mismatched markers, cross-version policy tuples, integers-for-floats, booleans, nonfinite values, missing fields, or extra fields fail closed. Qualifying one-step execution moves to gate schema `cbf2026-qualified-v6-one-step-viability-v2`, campaign ID `qualified-v6-one-step-development-gate-v2`, the canonical v3 primary overlay, and canonical v3 initial family. Injected operations remain irreversibly nonqualifying.

- [ ] **Step 1: Add distinguishing RED tests before implementation**

Required RED cases:

1. Exact v3 config/family acceptance is initially absent; changing `0.131` to `0.13`, `0.132`, an integer, or another mode/marker fails.
2. All 100 v3 materialized position sequences are bitwise equal to v2 and v1, and the v2 config/family SHA-256 identities remain unchanged.
3. Pure reconstruction over all 1400 frozen frame-zero local problems observes minimum `rho=0.7658252531927233 m/s` within a frozen numeric tolerance, proves `mu <= rho`, and produces exactly `mu=0.1 m/s` for every problem under v2 policy. The same test records, but does not reinterpret, the historical v1 floors.
4. For paired v2/v3 materialized settings, canonical hard-problem rows/hashes, row ownership, component bounds, nominal task target, soft-CBF slack variables, `k_delta`, and objective coefficients are unchanged. Only the policy marker/mode/fraction and resulting floor may differ.
5. Real-v3 fixture evidence independently reconstructs `rho`, `mu`, and applied residual. Cross-version marker/policy mutations fail schema, reconstruction, and analyzer validation; legacy v1 and historical v2 fixtures still pass their own exact contracts.
6. Gate fake/unit tests bind exact v3 paths, gate-v2 schema, full retained recomputation evidence, and unchanged numerical predicate. Old v2 paths cannot qualify. No test may launch the real binary.

Before any implementation dispatch, the amendment reviewer must use the committed pre-negative blob `f68cdc7:config/diagnostics/qualified_initial_family_v2.json` (SHA-256 `21d04b79e9e81ba867e28826ad43615120a4889d16e082d602e933a6a73177ef`), not a working-tree copy, to independently reconstruct all 100 seeds and 1400 current local problems. The review records the minimum `rho`, the saturation ratio, exact v2 primary/ablation/family blob hashes, and that those blobs predate the Task 6 negative integration. It must explicitly state that the observed next-radius value `0.045836701551219286` was not an input to choosing `0.131`.

- [ ] **Step 2: Run focused RED, then implement the minimal additive version**

Do not change the generic Chebyshev algorithm or its historical default. Select the policy constants explicitly from the validated marker/version. Apply the v2 floor to the same local allocated hard rows only; no neighbor command, joint QP, predictive constraint, new slack, or changed row enters the controller.

- [ ] **Step 3: Run pure/unit GREEN and relevant regressions only**

At minimum run the qualified-config, robust-construction, hybrid-guard, hard-interior, evidence reconstruction, analyzer, initial-family, and gate fake/unit suites. The gate's real-binary test class must be skipped or excluded. Rebuild `Swarm` only as a compilation check if source changes require it; do not execute it.

- [ ] **Step 4: Freeze the exact implementation commit, then independently review it before Task 7**

The implementation commit may stage exactly these 18 paths and no others:

```text
config/diagnostics/qualified_mode_hybrid_dcbf_development_v3.json
config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v3.json
config/diagnostics/qualified_initial_family_v3.json
include/Robot.hpp
include/cbf/HybridCertificateGuard.hpp
scripts/diagnostics/qualified_config.py
scripts/diagnostics/qualified_closure_evidence.py
scripts/diagnostics/analyze_qualified_closure_campaign.py
scripts/diagnostics/qualified_v6_initial_state.py
scripts/diagnostics/audit_qualified_v6_one_step_viability.py
tests/testRobustConstraintConstruction.cpp
tests/testHybridCertificateGuard.cpp
tests/test_qualified_config.py
tests/test_qualified_closure_evidence.py
tests/test_analyze_qualified_closure_campaign.py
tests/test_qualified_v6_initial_state.py
tests/test_audit_qualified_v6_one_step_viability.py
tests/test_swarm_evidence_stream.py
```

Commit this exact staged universe as `feat(cbf): preregister v6 controller margin`, freeze its commit/tree, and then create the separate post-implementation review at `docs/superpowers/plans/reviews/2026-08-02-cbf2026-qualified-v6-controller-margin-implementation-review.md`. The reviewer recomputes the 100-seed/1400-problem result without trusting a stored summary; hashes v1/v2 historical bytes before and after; verifies the additive v3 diff does not change the graph, CBF rows, objective, component bounds, seeds, thresholds, or retry policy; verifies no real-binary launch or formal artifact occurred; and returns C0/I0/M0. Any review fix uses a new scoped fix commit and a scoped re-review. After final PASS, commit only the review artifact as `docs(cbf2026): review v6 controller margin implementation`; never edit the already committed pre-implementation amendment review.

This task establishes a stronger *current-row command-selection margin*. It does not prove that the next-state radius is bounded below, recursive feasibility, sampled-data safety, or long-horizon localization accuracy. Those remain empirical questions for the unchanged one-step gate and later Monte Carlo campaign.

---

### Task 7: Bind Development-v6 Lifecycle Without Generating Its Protocol

**Files:**
- Modify: `scripts/diagnostics/register_qualified_closure_campaign.py`
- Modify: `scripts/diagnostics/run_qualified_closure_campaign.py`
- Modify: `tests/test_register_qualified_closure_campaign.py`
- Modify: `tests/test_run_qualified_closure_campaign.py`

**Interfaces:**
- Consumes: passed gate-v2 artifact schema/path, v6 predecessor identity, exact v3 primary/ablation overlays, exact v3 initial family, both committed Task 6b review artifacts, the frozen Task 6b implementation commit/tree, and the existing protocol-v2 authorization model.
- Produces: production registration support for `kind=development, version=v6` that refuses absent/failed/mismatched gate evidence, plus a pure `validate_authorization_payload(...)` helper for precommit field/hash checks that deliberately does not inspect Git topology; it does not itself create production artifacts in this task.

- [ ] **Step 1: Add RED lifecycle tests**

```python
def test_v6_protocol_rejects_absent_or_failed_one_step_gate(self):
    with self.assertRaisesRegex(ValueError, "one-step gate"):
        self.build_v6(gate=None)
    with self.assertRaisesRegex(ValueError, "one-step gate"):
        self.build_v6(gate=self.failed_gate())

def test_v6_protocol_binds_exact_gate_family_configs_and_predecessor(self):
    protocol = self.build_v6(gate=self.passed_gate())
    self.assertEqual(protocol["version"], "v6")
    self.assertEqual(protocol["development_gate"]["status"], "completed")
    self.assertEqual(protocol["development_gate"]["schema_version"], "cbf2026-qualified-v6-one-step-viability-v2")
    self.assertEqual(protocol["development_gate"]["campaign_id"], "qualified-v6-one-step-development-gate-v2")
    self.assertEqual(protocol["primary_config_path"], "config/diagnostics/qualified_mode_hybrid_dcbf_development_v3.json")
    self.assertEqual(protocol["ablation_config_path"], "config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v3.json")
    self.assertEqual(protocol["initial_family_path"], "config/diagnostics/qualified_initial_family_v3.json")
    self.assertEqual(protocol["schedule"]["trajectory_seeds"], list(range(2026080201, 2026080211)))

def test_v6_runtime_rejects_historical_or_inferred_authorization(self):
    with self.assertRaisesRegex(ValueError, "authorization"):
        self.validate_v6_with_v5_authorization()

def test_v6_precommit_authorization_payload_validation_is_pure(self):
    authorization = self.exact_v6_authorization_payload()
    validated = registrar.validate_authorization_payload(
        self.protocol, authorization,
        protocol_sha256=self.protocol_sha256,
        preflight_sha256=self.preflight_sha256,
    )
    self.assertEqual(validated, authorization)
```

Add tests for gate HEAD/tree mismatch, binary/config/family hash mismatch, changed seed order, v5/v6 root collision, existing v6 root, altered predecessor file, runner argv omission, analyzer argv mutation, and v5 regression verification.

- [ ] **Step 2: Run RED**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_register_qualified_closure_campaign \
  tests.test_run_qualified_closure_campaign -v
```

Expected: only new v6 cases FAIL; all existing v5 cases remain green.

- [ ] **Step 3: Add version-keyed validation**

Require literal v3 family/overlay paths, gate-v2 paths, predecessor path, and exact identities. The v6 schedule uses the same 10 registered trajectory seeds, a newly frozen 10-seed range-noise schedule, 1000 frames, new absent `/v6` raw and analysis roots, no retry, and the existing resource/supervision limits. Do not mutate v5 argv or protocol verification.

Add a pure `validate_authorization_payload` that accepts already-parsed protocol/authorization values plus the exact protocol/preflight hashes and checks the exact ten-field schema, values, date, verbatim UTF-8 text, and text digest. It must not call `_repository_identity`, `_verify_committed_registration_state`, or inspect filesystem/Git topology. Keep `validate_authorization_binding` as the production postcommit validator that performs those topology and committed-blob checks.

Before claiming any v6 root, the runner revalidates authorization, protocol, gate, family, configs, binary, predecessor, schedule, and root absence. It may verify the committed gate artifact but must not rerun or reinterpret the formal gate.

- [ ] **Step 4: Run GREEN and combined lifecycle regressions**

```bash
conda run -n cbf_env python -m unittest \
  tests.test_register_qualified_closure_campaign \
  tests.test_run_qualified_closure_campaign \
  tests.test_analyze_qualified_closure_campaign \
  tests.test_qualified_closure_evidence -v
```

Expected: all PASS. Production v6 protocol/preflight/authorization paths and v6 roots remain absent.

- [ ] **Step 5: Review and commit**

Independent reviewer checks backward-compatible v5 verification, exact gate-before-protocol ordering, fresh authorization enforcement, root absence/no-retry, and exact seed preservation. Required verdict C0/I0/M0.

Suggested commit after explicit approval:

```bash
git add scripts/diagnostics/register_qualified_closure_campaign.py \
  scripts/diagnostics/run_qualified_closure_campaign.py \
  tests/test_register_qualified_closure_campaign.py \
  tests/test_run_qualified_closure_campaign.py
git commit -m "feat(experiments): bind v6 interior viability lifecycle"
```

---

### Task 8: Verify the Committed Implementation and Run the Formal Development Gate

**Files:**
- Create before any formal-gate launch: `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2-claim.json`
- Create on PASS or terminal failure: `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2.json`
- Create: `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2-review.md`

**Interfaces:**
- Consumes: clean committed Tasks 1--7 implementation and exact production binary/config/family.
- Produces: one immutable pre-launch claim, one terminal formal gate artifact when terminal publication is reachable, and one independent review. No v6 protocol yet.

- [ ] **Step 1: Run the full implementation verification before the formal gate**

```bash
cd /private/tmp/cbf2026-diagnostic
git status --short
git diff --check
conda run -n cbf_env cmake -S . -B build-diagnostic -DBUILD_TESTING=ON
conda run -n cbf_env cmake --build build-diagnostic --target \
  Swarm testHardInteriorSelection testRobustConstraintConstruction \
  testHybridCertificateGuard testAllocatedPairwiseCBF -j2
./build-diagnostic/testHardInteriorSelection
./build-diagnostic/testRobustConstraintConstruction
./build-diagnostic/testHybridCertificateGuard
./build-diagnostic/testAllocatedPairwiseCBF
conda run -n cbf_env python -m unittest \
  tests.test_hard_interior_selection \
  tests.test_qualified_config \
  tests.test_qualified_v6_initial_state \
  tests.test_audit_qualified_v6_one_step_viability \
  tests.test_qualified_closure_evidence \
  tests.test_swarm_evidence_stream \
  tests.test_register_qualified_closure_campaign \
  tests.test_run_qualified_closure_campaign \
  tests.test_analyze_qualified_closure_campaign -v
```

Expected: all PASS. Tracked worktree is clean; only preserved `build-diagnostic/` may be untracked. If implementation changes are uncommitted, stop and complete their review/approved commits first.

- [ ] **Step 2: Run the formal-gate RED precondition**

```bash
test ! -e docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2.json
test ! -e docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2-claim.json
test ! -e docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v6-protocol.json
test ! -e /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v6
test ! -e /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v6
git rev-parse HEAD
git rev-parse 'HEAD^{tree}'
shasum -a 256 build-diagnostic/Swarm config/config.json \
  config/diagnostics/qualified_mode_hybrid_dcbf_development_v3.json \
  config/diagnostics/qualified_initial_family_v3.json
```

Expected: gate/protocol/roots are absent and identities are recorded from live state.

- [ ] **Step 3: Execute the formal 100-seed gate exactly once**

```bash
conda run -n cbf_env python -m \
  scripts.diagnostics.audit_qualified_v6_one_step_viability \
  --binary build-diagnostic/Swarm \
  --base-config config/config.json \
  --primary-config config/diagnostics/qualified_mode_hybrid_dcbf_development_v3.json \
  --initial-family config/diagnostics/qualified_initial_family_v3.json \
  --claim docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2-claim.json \
  --output docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2.json
```

Expected PASS evidence: the immutable claim predates every child launch; the terminal artifact binds its SHA-256 and reports terminal/completed, `launch_count=100`, `retry_count=0`, audit `100/100`, registered `10/10`, minimum next barrier strictly positive, minimum next local radius `>=0.05 m/s`, zero applied residual/floor violations, zero component-bound violations, and exact implementation/binary/config/family identities. If the process or any seed fails, preserve the claim and terminal artifact. If an uncatchable interruption leaves only the claim, preserve it and write a separate interruption review without relaunching. In every non-PASS case stop before protocol generation and begin a newly versioned development design from a new implementation commit.

- [ ] **Step 4: Independently review the formal artifact**

Reviewer first verifies the claim identity and creation chronology, then recomputes all 100 position hashes, launch universe, every frame-zero applied residual, every one-step state, all barrier/radius minima, identities, and no-retry/no-replace evidence without trusting `passed`. Review must distinguish this one-step empirical gate from recursive-feasibility proof and return C0/I0/M0.

- [ ] **Step 5: Run the formal-gate GREEN check**

```bash
conda run -n cbf_env python - <<'PY'
import json
import hashlib
from pathlib import Path
c = Path("docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2-claim.json")
p = Path("docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2.json")
d_claim = json.loads(c.read_text())
d = json.loads(p.read_text())
assert d_claim["claimed"] is True
assert d["claim_sha256"] == hashlib.sha256(c.read_bytes()).hexdigest()
assert d["terminal"] is True and d["status"] == "completed" and d["passed"] is True
assert d["launch_count"] == 100 and d["retry_count"] == 0
assert d["audit_summary"]["passed_count"] == 100
assert d["registered_summary"]["passed_count"] == 10
assert d["audit_summary"]["minimum_next_barrier_m"] > 0.0
assert d["audit_summary"]["minimum_next_local_radius_mps"] >= 0.05
PY
git diff --check -- \
  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2-claim.json \
  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2.json \
  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2-review.md
```

Expected: PASS and review C0/I0/M0.

- [ ] **Step 6: Commit gate and review after explicit approval**

Suggested commit:

```bash
git add docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2.json \
  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2-claim.json \
  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2-review.md
git commit -m "docs(cbf2026): approve v6 one-step viability gate"
```

Only this committed PASS identity unlocks Task 9.

---

### Task 9: Generate Protocol and Preflight, Then Wait for Fresh Authorization

**Files:**
- Create: `docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v6-protocol.json`
- Create: `docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v6-protocol.md`
- Create: `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v6-preflight.md`
- Create only after fresh approval: `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v6-authorization.json`

**Interfaces:**
- Consumes: committed gate PASS/review and clean exact implementation identity.
- Produces: exact development-v6 protocol/preflight, then an authorization document bound to new user text.

- [ ] **Step 1: Run lifecycle RED checks**

```bash
test ! -e docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v6-protocol.json
test ! -e docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v6-authorization.json
test ! -e /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v6
test ! -e /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v6
git status --short
```

Expected: protocol, authorization, and roots absent; tracked worktree clean.

- [ ] **Step 2: Generate the exact protocol with the production registrar**

Use the registrar's production CLI with `kind=development`, `version=v6`, exact binary/base/v3 primary/v3 ablation/v3 family/gate-v2 paths, the unchanged 10 trajectory seeds, newly frozen 10 range-noise seeds, 1000 frames, and absent `/v6` roots. Do not manually edit the generated JSON.

Expected: protocol records all frozen controller/gate parameters, predecessor identities, exact argv, roots, hashes, no-retry, resource limits, and the committed gate PASS identity. Authorization path remains absent.

- [ ] **Step 3: Run independent preflight**

Preflight must rebuild/run all relevant tests, validate protocol semantics and exact identities from a clean worktree, recompute the v5 predecessor and v6 gate identities, verify v6 roots absent, verify disk/resource limits, and record C0/I0/M0. It must explicitly state that its verdict is not user authorization.

- [ ] **Step 4: Run protocol/preflight GREEN checks**

```bash
test -f docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v6-protocol.json
test -f docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v6-protocol.md
test -f docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v6-preflight.md
test ! -e docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v6-authorization.json
git diff --check
```

Expected: exact protocol/preflight exist, authorization absent, and no v6 root exists.

- [ ] **Step 5: Present exact hashes and wait**

Present the exact protocol SHA-256, preflight SHA-256, implementation identity, gate identity, kind/version, roots, schedule, and command. Ask for new exact authorization text. Stop. Do not infer approval from earlier messages.

- [ ] **Step 6: After new user text, create and validate authorization**

The JSON contains only the schema's exact fields and binds the verbatim UTF-8 text plus its SHA-256. Before commit, run only the dedicated pure authorization-payload validator added and tested in Task 7: exact field set, kind/version, implementation identity, protocol/preflight hashes, canonical date, verbatim UTF-8 text, and text SHA-256. This precommit helper must not inspect committed-child topology. Do not call `validate_authorization_binding` yet because the exact-four child does not exist.

- [ ] **Step 7: Commit exact lifecycle artifacts after explicit approval**

Suggested commit:

```bash
git add \
  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v6-protocol.json \
  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v6-protocol.md \
  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v6-preflight.md \
  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v6-authorization.json
git diff --cached --name-only
git commit -m "docs(cbf2026): authorize v6 closure campaign"
```

Expected staged universe: exactly those four files; the commit is their sole direct add-only child of the registered implementation identity.

- [ ] **Step 8: Run production postcommit validation before any root allocation**

```bash
conda run -n cbf_env python - <<'PY'
from pathlib import Path
from scripts.diagnostics.register_qualified_closure_campaign import validate_authorization_binding
protocol = Path("docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v6-protocol.json")
authorization = Path("docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v6-authorization.json")
validated = validate_authorization_binding(protocol, authorization)
assert validated["authorized"] is True
assert validated["kind"] == "development" and validated["version"] == "v6"
PY
git status --short
```

Expected: production validation proves current HEAD is the sole direct exact-four add-only child of the registered implementation identity, every live artifact equals its committed blob, hashes and fresh user text bind exactly, v6 roots remain absent, and porcelain contains only the registered allowed `build-diagnostic/` path. Any failure keeps Task 10 closed; do not amend the authorization commit or allocate a root.

---

### Task 10: Execute and Independently Analyze Registered Development-v6 Once

**Files:**
- Create: `docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v6.md`
- Create: `docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v6-review.md`
- Create externally and immutably: `/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v6`
- Create externally and immutably: `/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v6`

**Interfaces:**
- Consumes: exact authorized runner/analyzer argv serialized by the protocol.
- Produces: terminal raw/analysis roots, one evidence-bounded report, and independent source review; no paper mutation. Any DRA synchronization is the separate documentation-only action defined by the global constraint and is not performed by the runner, analyzer, or source-task worker.

- [ ] **Step 1: Run final RED preconditions**

```bash
test ! -e /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v6
test ! -e /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v6
git status --short
```

Expected: both roots absent and tracked worktree clean. If either root exists, do not launch or delete it; inspect and terminalize only through the registered lifecycle.

- [ ] **Step 2: Invoke the protocol-serialized runner exactly once**

Copy the exact argv array from the protocol without retyping parameters. Record start/end time and exit code. Do not retry a nonzero exit.

Expected: runner claims the v6 raw root once and writes terminal manifests for all 10 missions, including never-started/failed missions in denominators.

- [ ] **Step 3: Invoke the protocol-serialized analyzer exactly once**

Run only after the raw root is terminal. Copy exact analyzer argv from the protocol. Do not invoke it a second time even if it returns nonzero.

Expected: analyzer claims the v6 analysis root once and produces either a terminal compact PASS bundle or a terminal failed manifest.

- [ ] **Step 4: Independently review terminal evidence**

Reviewer verifies root trees/manifests, schedule completeness, all controller policy records, `rho/mu` reconstruction, original hard residuals, input bounds, barriers, reset/history semantics, localization metrics, primary/fixed-FIM comparison, and every denominator. Missing later rows are failures, never successes. Required claim review is C0/I0/M0 before any positive paper handoff.

- [ ] **Step 5: Write the terminal report without overclaiming**

The report records exact identities/commands/root commitments and separates:

- continuous conditional CBF theory;
- current local interior-selection implementation;
- one-step admission evidence;
- long-horizon v6 empirical outcomes.

It must explicitly say B and C do not prove recursive feasibility and that no sampled-data reserve was introduced.

- [ ] **Step 6: Run final GREEN checks**

```bash
test -f docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v6.md
test -f docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v6-review.md
test -d /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v6
test -d /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v6
git diff --check
```

Expected: both roots and both reports are terminal and immutable. A PASS requires every registered analysis gate plus independent C0/I0/M0 review; otherwise preserve the failure and stop.

- [ ] **Step 7: Commit terminal report/review after explicit approval**

Suggested commit:

```bash
git add docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v6.md \
  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v6-review.md
git commit -m "docs(cbf2026): record v6 closure evidence"
```

This source plan ends at independently reviewed source evidence. Under the researcher's separate standing authorization, the primary agent may append the reviewed checkpoint to `/private/tmp/dra-cbf2026-diagnostic` on `main`, limited to `meta-log/2026-08-01-cbf2026-qualified-mode-hybrid-dcbf.md`, `papers/cbf2026/sources.md`, `papers/cbf2026/open-questions.md`, and `papers/cbf2026/timeline.md`, followed by a separate DRA documentation review/commit. That DRA action is outside the source staged universe and source completion matrix; it cannot authorize execution or strengthen a scientific claim. Paper claims and figures remain unchanged until the corresponding source evidence is independently reviewed.

## Final Verification Matrix

Before declaring this plan implemented, an independent reviewer must obtain all of the following from live artifacts:

```text
planning plan/review committed and C0/I0/M0
v5 roots and manifests unchanged from Task 1 identities
alpha(comm-fixed) = alpha(safety) = 0.1, power = 1
distributed execution; no joint swarm QP
rho uses only local allocated hard rows plus +/-25 planar bounds
yaw absent from rho
historical policy v1 remains fraction = 0.10, cap = 0.10 m/s and is not rerun
formal policy v2 is pre-registered once at fraction = 0.131, cap = 0.10 m/s
all 1400 frozen frame-zero local problems have mu = 0.10 m/s and mu <= rho under policy v2
task/slack objective and k_delta = 10 unchanged
dt = 0.5 s
same ordered 100 audit seeds and first 10 registered seeds
clamp = false, resample = false, retry = false
formal gate launch_count = 100, retry_count = 0
formal gate next barriers all > 0 m
formal gate every next local rho >= 0.05 m/s
formal gate and review committed before protocol generation
protocol/preflight generated from that exact identity
fresh verbatim user authorization after protocol/preflight
runner invoked once; analyzer invoked once
v6 roots terminal and immutable
terminal report/review distinguish theorem, one-step gate, and empirical campaign
paper unchanged until the relevant independently reviewed evidence exists
```

## Execution Handoff

Phase 0 is planning-only. After the plan and its independent review are committed, use **Subagent-Driven Development**: dispatch a fresh implementation worker for Task 1, then a separate standards/spec reviewer before continuing to Task 2. Do not batch Tasks 1--7 into one worker or one commit. Stop automatically at any failed RED/GREEN, independent-review, formal-gate, protocol, authorization, runner, or analyzer boundary.
