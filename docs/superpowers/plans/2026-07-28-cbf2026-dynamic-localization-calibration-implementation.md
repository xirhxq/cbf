# CBF2026 Dynamic Localization Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore the intended dynamic localization DAG, implement a simple offline WNLS range estimator, run disk-guarded paired calibration, and align the paper and DRA with the resulting evidence.

**Architecture:** Keep the two fixed formation/connectivity CBF references separate from the dynamic localization-information graph. Restore the C++ FIM membership to the fixed-CBF-reference union with visible eligible references, then use a pure-NumPy offline replay over preserved truth trajectories so estimates never enter the controller. Produce a hash-bound evidence bundle for one 20 s smoke and a 20-seed 250 s paired graph ablation before updating manuscript claims.

**Tech Stack:** C++17, Eigen, nlohmann/json, doctest, CMake, Python 3.11 in conda environment `cbf_env`, NumPy 1.24.4, standard-library `unittest`, gzip JSON Lines, LaTeX/BibTeX.

## Global Constraints

- Work on `/private/tmp/cbf2026-diagnostic` branch `codex/cbf2026-diagnostic`; do not touch `cbf-research-2026` or its stash.
- Preserve every existing commit and result bundle. Correct the fixed-information-set interpretation with new commits; never rewrite history.
- The two assigned CBF references, including assigned bases, always enter the FIM even when temporarily beyond `max-range`.
- Add every other base within `max-range` and every other same-squad, strictly lower-local-index UAV within `max-range`; use all admitted references, with no best-subset selection.
- Every UAV covariance dependency must point to a strictly lower squad-local index; bases are roots.
- Keep the existing ranging variance, propagated reference covariance, Jacobian, \(\Phi=J^\top\Sigma^{-1}J\), covariance inversion, and FIM/covariance validity gates.
- Use exactly one estimator: per-frame weighted nonlinear least squares with pure NumPy Levenberg--Marquardt damping. Do not install SciPy and do not add EKF, UKF, factor-graph, or estimator comparisons.
- Initialize at the known deployment position at frame zero; later frames use the previous estimate. Never truth-reset after initialization and never feed estimates into CBF, CVT, communication, or dynamics.
- Use `ranging_sigma=0.5 m`; generate each noise sample from a stable key `(run_seed, frame_index, observer_id, reference_kind, reference_id)`.
- Compute \(\epsilon_i=3\sqrt{\lambda_{\max}(P_i)}\) once; do not apply another factor of three.
- Retain frame zero in process logs but exclude it from primary containment statistics.
- Compare `dynamic_dag_wnls` with `fixed_refs_wnls` using the same estimator, truth trajectory, initialization, and common-edge noise.
- Use start-space `8_000_000_000` bytes, live floor `6_000_000_000` bytes, output-root cap `2_000_000_000` bytes, and per-bundle cap `250_000_000` bytes.
- Do not duplicate truth trajectories or source snapshots in calibration output. Bind them by absolute path and SHA-256.
- Run Python only as `conda run -n cbf_env python ...`.
- Make every production change test-first and record the observed RED command/output before implementation.
- Do not add `Co-Authored-By`. Stage only named files; never use `git add .`.

---

### Task 1: Restore the Dynamic Localization Information Set

**Files:**
- Modify: `tests/testRobustConstraintConstruction.cpp:480-580`
- Modify: `include/Robot.hpp:449-685`
- Include in documentation commit: `docs/superpowers/specs/2026-07-28-cbf2026-dynamic-localization-dag-calibration-design.md`
- Include in documentation commit: `docs/superpowers/plans/2026-07-28-cbf2026-dynamic-localization-calibration-implementation.md`

**Interfaces:**
- Keeps: `Robot::fixedLocalizationReferences() const -> json` as the source of assigned CBF base/UAV references.
- Produces: `Robot::activeLocalizationReferences(const json& config) const -> json`.
- Changes: `Robot::getCovariance()` to `Robot::getCovariance(const json& config)`.
- Consumes: `settings`, `bases`, `comm->_othersPos`, `myNeighboursId`, `isSamePartAsMe`, `getIdInPart`, and `model->xy()`.
- Preserves: `setupFormation()` consuming only `fixedLocalizationReferences()`.

- [ ] **Step 1: Replace the fixed-only assertions with failing dynamic-union assertions**

Rename the test
`"covariance FIM excludes in-range references outside the fixed localization graph"`
to
`"covariance FIM includes eligible in-range references outside the fixed CBF graph"`.
For the four-UAV fixture's highest-index robot, assert:

```cpp
robot.getCovariance(settings["cbfs"]["without-slack"]["comm-fixed"]);
CHECK(robot.myCovarianceFormation.at("anchorIds")
      == json::array({2, 3, 1}));
CHECK(robot.myCovarianceFormation.at("baseIds")
      == json::array({0, 1, 2}));
```

Sort expected/actual IDs in the test if communicator map iteration order is
not stable; production must emit each ID exactly once.

Replace
`"covariance information set is invariant across optional-anchor range crossing"`
with a test that places optional UAV 1 inside range, records its inclusion,
moves it outside range, recomputes, and asserts it disappears while fixed UAVs
2 and 3 remain. Keep the existing assigned-base-beyond-range test and update
all calls to pass the communication configuration.

Add a test where an assigned base is outside `max-range` and an unassigned base
is inside it; assert both are present. Then move the unassigned base outside
range and assert only the assigned base remains.

- [ ] **Step 2: Run the focused test and record RED**

Run:

```bash
cmake --build build-diagnostic --target testRobustConstraintConstruction -j2
./build-diagnostic/testRobustConstraintConstruction
```

Expected RED: compilation fails because `getCovariance(const json&)` and
`activeLocalizationReferences(const json&)` do not exist, or the dynamic
membership assertions fail against the current fixed implementation.
Record the exact failing output in the task report before editing
`include/Robot.hpp`.

- [ ] **Step 3: Implement the active-reference union**

Add after `fixedLocalizationReferences()`:

```cpp
json activeLocalizationReferences(const json& config) const {
    json references = fixedLocalizationReferences();
    const double maxRange = config.at("max-range").get<double>();
    const Point myPosition = model->xy();

    auto appendUnique = [](json& ids, int candidate) {
        for (const json& existing : ids) {
            if (existing.get<int>() == candidate) return;
        }
        ids.push_back(candidate);
    };

    for (std::size_t baseId = 0; baseId < bases.size(); ++baseId) {
        if (bases.at(baseId).distance_to(myPosition) <= maxRange) {
            appendUnique(references["baseIds"], static_cast<int>(baseId));
        }
    }

    for (const auto& [otherId, otherPosition] : comm->_othersPos) {
        if (!isSamePartAsMe(otherId)) continue;
        if (getIdInPart(otherId) >= idInMyPart) continue;
        if (otherPosition.distance_to(myPosition) > maxRange) continue;
        appendUnique(references["anchorIds"], otherId);
    }
    return references;
}
```

Assigned fixed references are already present before range-gated additions,
so assigned bases and UAVs persist beyond range. Optional robot edges retain
the same-squad/strict-lower-index filters.

Change `getCovariance()` to:

```cpp
void getCovariance(const json& config) {
    if (!enablePositionCovariance) return;
    const json references = activeLocalizationReferences(config);
    // existing reference fetch and unchanged FIM/validity code follows
}
```

Change error text from `"fixed reference"` to `"active reference"` where the
message can now describe either fixed or optional data. Update
`updateCovarianceAndRate(double dt)` to read
`settings["cbfs"]["without-slack"]["comm-fixed"]` and call
`getCovariance(commConfig)`. Do not make `setupFormation()` consume the active
query.

- [ ] **Step 4: Verify GREEN and complete C++ regression**

Run:

```bash
cmake --build build-diagnostic \
  --target testRobustConstraintConstruction testRobotDiagnostics \
  testDiagnosticConfiguration testSwarmFailureHandling Swarm -j2
./build-diagnostic/testRobustConstraintConstruction
./build-diagnostic/testRobotDiagnostics
./build-diagnostic/testDiagnosticConfiguration
./build-diagnostic/testSwarmFailureHandling
ctest --test-dir build-diagnostic --output-on-failure
```

Expected: every command exits zero; every new dynamic-membership assertion
passes; no existing validity-gate test regresses.

- [ ] **Step 5: Commit Task 1**

```bash
git add -- \
  docs/superpowers/specs/2026-07-28-cbf2026-dynamic-localization-dag-calibration-design.md \
  docs/superpowers/plans/2026-07-28-cbf2026-dynamic-localization-calibration-implementation.md \
  include/Robot.hpp \
  tests/testRobustConstraintConstruction.cpp
git commit -m "fix(cbf): restore dynamic localization DAG"
```

The commit must contain exactly these four files.

---

### Task 2: Implement the Pure-NumPy WNLS Core

**Files:**
- Create: `scripts/diagnostics/replay_localization_calibration.py`
- Create: `tests/test_replay_localization_calibration.py`

**Interfaces:**
- Produces: `stable_measurement_seed(...) -> int`.
- Produces: `fixed_references(...) -> dict[str, list[int]]`.
- Produces: `active_references(...) -> dict[str, list[int]]`.
- Produces: `solve_wnls(...) -> dict`.
- Produces: `fim_radius(...) -> dict`.
- Consumes later: `replay_calibration(...)` in Task 3.

- [ ] **Step 1: Write failing deterministic-noise and graph-membership tests**

Create `tests/test_replay_localization_calibration.py` using
`unittest.TestCase`. Add tests that:

1. call `stable_measurement_seed(20260727, 5, 14, "uav", 12)` twice and
   assert equality, then change each tuple component separately and assert a
   different integer;
2. use `np.random.default_rng(stable_measurement_seed(...)).normal(0, 0.5)`
   in both graph cases and assert bitwise-equal common-edge noise;
3. construct 14 positions and the production formation config, then assert
   fixed references are assigned bases and/or the previous two same-squad
   UAVs;
4. assert active references equal fixed references union all visible bases
   and same-squad lower-index UAVs;
5. assert an assigned base and assigned UAV persist beyond range, while an
   unassigned base/UAV disappears beyond range; and
6. assert every returned UAV reference has a smaller squad-local index and no
   cross-squad or higher-index reference appears.

- [ ] **Step 2: Run the new module tests and record RED**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_replay_localization_calibration -v
```

Expected RED: import fails because
`scripts.diagnostics.replay_localization_calibration` does not exist.
Record the exact output in the task report.

- [ ] **Step 3: Implement stable keys and graph reconstruction**

Implement:

```python
def stable_measurement_seed(
    run_seed: int,
    frame_index: int,
    observer_id: int,
    reference_kind: str,
    reference_id: int,
) -> int:
    payload = (
        f"cbf2026-range-v1|{run_seed}|{frame_index}|{observer_id}|"
        f"{reference_kind}|{reference_id}"
    ).encode("utf-8")
    return int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")
```

Implement squad mapping with:

```python
squad_size = math.ceil(config["num"] / config["formation"]["parts"])
part = (robot_id - 1) // squad_size
local_index = (robot_id - 1) % squad_size
```

`fixed_references()` must reproduce the current
`fixedLocalizationReferences()` rule exactly. With one-based `local_index`,
zero-based `part_index`, and the part's ordered
`assigned_bases=config["formation"]["bases-id"][part_index]`, compute:

```python
comm = config["cbfs"]["without-slack"]["comm-fixed"]
min_offset = int(comm.get("min-neighbour-id-offset", -2))
max_offset = int(comm.get("max-neighbour-id-offset", 0))
maximum_base_index = -local_index - min_offset
fixed_base_ids = [
    int(base_id)
    for index, base_id in enumerate(assigned_bases)
    if index <= maximum_base_index
]
fixed_uav_ids = [
    candidate
    for candidate in range(1, int(config["num"]) + 1)
    if candidate != observer_id
    and observer_id + min_offset <= candidate <= observer_id + max_offset
    and squad_part(candidate) == part_index
    and squad_local(candidate) < local_index
]
```

Return sorted unique IDs.
`active_references()` must start from that fixed set, append every base within
`max-range`, then append every visible same-part strict-lower-index UAV, with
stable sorted unique IDs.

- [ ] **Step 4: Write failing WNLS/FIM tests**

Add tests for:

- zero-noise recovery of \([1.0, 1.0]\) from bases \([0,0]\) and \([2,0]\)
  using initial \([1.0,0.8]\);
- continuous mirror selection: initial \([1.0,0.8]\) converges to positive
  \(y\), while initial \([1.0,-0.8]\) converges to negative \(y\);
- a third non-collinear reference is accepted without changing API;
- a hand case with zero reference covariance and `sigma=0.5` yields
  \(\Phi=\sum 4e_je_j^\top\), `P=inv(Phi)`, and
  `epsilon=3*sqrt(eigvalsh(P).max())`;
- a singular collinear case returns `status="invalid"` with no fabricated
  covariance;
- non-finite measurement input returns `status="invalid"`; and
- a later-frame wrapper marks held previous estimate/covariance as
  `status="stale"` while retaining a failure record.

Run the focused tests and confirm they fail because `solve_wnls()` and
`fim_radius()` are missing.

- [ ] **Step 5: Implement the minimal damped WNLS**

Use pure NumPy with defaults:

```python
MAX_ITERATIONS = 50
INITIAL_DAMPING = 1e-3
STEP_TOLERANCE = 1e-9
COST_TOLERANCE = 1e-12
RANGE_EPSILON = 1e-12
RELATIVE_SPECTRAL_THRESHOLD = 1e-12
```

At each iteration:

```python
diff = estimate[None, :] - reference_positions
distance = np.linalg.norm(diff, axis=1)
direction = diff / np.maximum(distance[:, None], RANGE_EPSILON)
projected_variance = np.einsum(
    "ni,nij,nj->n", direction, reference_covariances, direction
)
weight = 1.0 / (ranging_sigma**2 + projected_variance)
residual = distance - measurements
phi = direction.T @ (weight[:, None] * direction)
gradient = direction.T @ (weight * residual)
delta = np.linalg.solve(
    phi + damping * np.eye(2),
    -gradient,
)
```

Accept a trial only when its weighted cost is finite and lower; divide damping
by 10 after acceptance and multiply it by 10 after rejection. Stop on step or
cost tolerance. At convergence, recompute directions/weights, reject any FIM
whose minimum eigenvalue is not greater than
`1e-12 * maximum_eigenvalue`, invert it, and compute epsilon once.
Return explicit fields:

```python
{
    "status": "converged" | "failed" | "invalid" | "stale",
    "estimate": [x, y] | None,
    "covariance": [[xx, xy], [yx, yy]] | None,
    "epsilon": float | None,
    "phi_min_eigenvalue": float | None,
    "phi_condition": float | None,
    "iterations": int,
    "cost": float | None,
    "failure_reason": str | None,
}
```

- [ ] **Step 6: Verify GREEN and commit Task 2**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_replay_localization_calibration -v
```

Expected: all Task 2 tests pass.

Commit:

```bash
git add -- \
  scripts/diagnostics/replay_localization_calibration.py \
  tests/test_replay_localization_calibration.py
git commit -m "feat(diagnostics): add range WNLS calibration core"
```

---

### Task 3: Build the Disk-Guarded Replay Evidence Bundle

**Files:**
- Modify: `scripts/diagnostics/replay_localization_calibration.py`
- Modify: `tests/test_replay_localization_calibration.py`

**Interfaces:**
- Produces: `replay_calibration(data_path, manifest_path, output_root, run_seeds, project_root, max_frames=None) -> dict`.
- Produces CLI: `python -m scripts.diagnostics.replay_localization_calibration`.
- Reuses from `scripts.diagnostics.run_diagnostic`: `_sha256`, `available_bytes`, `allocated_bytes`, `require_start_space`, `_nearest_existing_ancestor`, `_validate_output_root`, `_allocate_run_root`, `_write_manifest`, `START_BYTES`, `HARD_FLOOR_BYTES`, `OUTPUT_ROOT_CAP_BYTES`, `RUN_CAP_BYTES`.
- Produces one bundle containing `calibration.jsonl.gz`, `summary.json`, `summary.md`, and `manifest.json`.

- [ ] **Step 1: Write failing replay/manifest tests**

Use `TemporaryDirectory()` to build a two-squad synthetic `data.json` with
three frames, bases, fixed formations, and robot truth states. Add tests that
assert:

- the output root is outside the project and start-space guard is invoked;
- the input data and manifest absolute paths and SHA-256 are recorded;
- no `data.json` or source snapshot is copied into the output;
- one process row exists for every
  `(seed, graph_case, non-base robot, frame)`;
- frame zero is logged with `primary_statistics=false`;
- process rows contain active references, true/noisy ranges/noise, estimate,
  error vector/norm, convergence/failure/stale fields, covariance, epsilon,
  containment, FIM minimum eigenvalue/condition, and active-set transition;
- common edges have identical noisy ranges across graph cases;
- identical input/seed reruns produce identical decompressed process content
  and identical summary content;
- disk floor/root-cap/run-cap failures write a terminal manifest; and
- summaries are valid strict JSON with `allow_nan=False`.

- [ ] **Step 2: Run replay tests and record RED**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_replay_localization_calibration -v
```

Expected RED: `replay_calibration()` or required bundle fields are missing.
Record the exact failure before implementing replay I/O.

- [ ] **Step 3: Implement topological replay**

Parse `data["config"]` and each frame's `robots` array. Build truth positions
by global 1-based ID. For each seed and graph case:

1. process each squad in increasing local index;
2. at frame zero, initialize estimates from
   `config["initial"]["position"]["positions"]`;
3. at later frames, initialize from the previous estimate;
4. reconstruct graph membership from truth geometry;
5. synthesize every keyed Gaussian range using `ranging_sigma`;
6. use exact base positions/covariance zero and current-frame upstream
   estimated UAV positions/covariances;
7. run WNLS and retain explicitly stale previous results after later failures;
8. compute truth error, epsilon containment, and transition metadata; and
9. write compact process rows to `calibration.jsonl.gz`.

Do not read current frame `covariance_formation` to define dynamic membership;
that field comes from whichever code version generated the trajectory.

- [ ] **Step 4: Implement summaries and adequacy gate**

Exclude frame zero from primary statistics. For each graph case, seed, and
squad-local depth compute:

- robot-frame count;
- converged/stale/invalid/failed counts;
- Euclidean containment count/rate;
- error/epsilon ratio median, 95th, 99th percentile, and maximum;
- FIM minimum-eigenvalue 1st/5th/50th percentiles;
- FIM condition and epsilon 50th/95th/99th percentiles and maximum; and
- information-set transition count and signed epsilon changes.

Compute a deterministic 95% bootstrap interval over per-seed containment
rates using 10,000 resamples and RNG seed `20260728`; never resample
robot-frames directly.

Set:

```python
adequacy = {
    "aggregate_containment_at_least_0_98": dynamic_rate >= 0.98,
    "every_depth_containment_at_least_0_95": min(dynamic_depth_rates) >= 0.95,
    "zero_silently_discarded_failures": processed_count == expected_count,
    "dynamic_invalid_rate_no_worse": dynamic_invalid_rate <= fixed_invalid_rate,
    "dynamic_failure_rate_no_worse": dynamic_failure_rate <= fixed_failure_rate,
}
adequacy["passed"] = all(adequacy.values())
```

The summary must report actual numbers even when the gate fails.

- [ ] **Step 5: Implement disk lifecycle and CLI**

Accept:

```text
--data PATH
--manifest PATH
--output-root PATH
--seed INTEGER          repeatable
--max-frames INTEGER    optional
```

Default to one seed from input config only when `--seed` is omitted.
Before allocation require 8 GB. After every frame batch check the 6 GB floor,
2 GB output-root cap, and 250 MB bundle cap. On any error, close gzip safely
and atomically write a terminal manifest with a non-success termination reason.
On success, record hashes for process, summary JSON, and summary Markdown.
Print the manifest JSON to stdout; return 0 only for
`termination_reason="completed"`.

- [ ] **Step 6: Verify GREEN and full Python regression**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_replay_localization_calibration \
  tests.test_analyze_diagnostic \
  tests.test_run_diagnostic \
  tests.test_swarm_failure_exit \
  tests.test_cmake_configuration -v
```

Expected: all tests pass with no warnings or errors.

- [ ] **Step 7: Commit Task 3**

```bash
git add -- \
  scripts/diagnostics/replay_localization_calibration.py \
  tests/test_replay_localization_calibration.py
git commit -m "feat(diagnostics): add localization calibration replay"
```

---

### Task 4: Execute the Smoke and Paired Monte Carlo Calibration

**Files:**
- Create after evidence: `docs/diagnostics/2026-07-28-dynamic-localization-calibration.md`
- Modify after evidence: `docs/diagnostics/README.md`
- Evidence root outside Git: `/private/tmp/cbf2026-localization-calibration`

**Interfaces:**
- Consumes Task 3 CLI and preserved truth bundle:
  `/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json`.
- Consumes manifest:
  `/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json`.
- Produces Stage 1 one-seed/40-frame bundle.
- Produces Stage 2 seeds `20260727` through `20260746`, 500-frame bundle.

- [ ] **Step 1: Verify disk and input hashes**

Run:

```bash
df -k /private/tmp
du -sk /private/tmp/cbf2026-results
shasum -a 256 \
  /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json
```

Require at least `8_000_000_000` free bytes. Expected data hash:
`3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527`.
Stop if it differs.

- [ ] **Step 2: Run the single registered 20 s smoke**

Run exactly once:

```bash
conda run -n cbf_env python -m \
  scripts.diagnostics.replay_localization_calibration \
  --data /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  --manifest /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json \
  --output-root /private/tmp/cbf2026-localization-calibration/stage1 \
  --seed 20260727 \
  --max-frames 40
```

Require completed terminal manifest, 40 frames, both graph cases, complete
process count, deterministic hashes, and no unexplained non-finite output.
If it fails, stop before Stage 2 and enter the reviewed fix loop.

- [ ] **Step 3: Re-run Stage 1 determinism check**

Run the identical command once more into the same collision-resistant output
root. Compare decompressed process SHA-256 and summary SHA-256 recorded in both
manifests. They must be equal. The different run-root names and timestamps are
not part of deterministic content.

- [ ] **Step 4: Run the 20-seed 250 s paired calibration**

Run exactly once after Stage 1 passes:

```bash
conda run -n cbf_env python -m \
  scripts.diagnostics.replay_localization_calibration \
  --data /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  --manifest /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json \
  --output-root /private/tmp/cbf2026-localization-calibration/stage2 \
  --seed 20260727 --seed 20260728 --seed 20260729 --seed 20260730 \
  --seed 20260731 --seed 20260732 --seed 20260733 --seed 20260734 \
  --seed 20260735 --seed 20260736 --seed 20260737 --seed 20260738 \
  --seed 20260739 --seed 20260740 --seed 20260741 --seed 20260742 \
  --seed 20260743 --seed 20260744 --seed 20260745 --seed 20260746
```

Do not tune the estimator, graph rule, noise, or epsilon after observing the
result.

- [ ] **Step 5: Write the compact evidence report**

Record exact source commit, input/output hashes, commands, disk probes, bundle
paths/sizes, graph memberships, optimizer status counts, coverage with
seed-level confidence interval, per-depth minima, FIM/condition/epsilon
quantiles, transition counts, paired dynamic-versus-fixed changes, adequacy
gate, and limitations.

State explicitly:

- the trajectory geometry is one preserved seed while ranging noise uses 20
  independent seeds;
- estimator output is offline and never enters the controller;
- fixed-reference comparison is a graph ablation;
- FIM covariance ignores shared-ancestor cross-correlations;
- frame zero is excluded from primary coverage; and
- no mission-level probability or closed-loop estimator guarantee follows.

- [ ] **Step 6: Commit Task 4**

```bash
git add -- \
  docs/diagnostics/2026-07-28-dynamic-localization-calibration.md \
  docs/diagnostics/README.md
git commit -m "docs(diagnostics): record localization calibration evidence"
```

---

### Task 5: Align the Paper and DRA with Reviewed Evidence

**Files:**
- Modify in paper repo: `/Users/xirhxq/Documents/Clones/cbf/papers/CBF2026/main.tex`
- Modify in DRA: `/Users/xirhxq/Documents/Clones/doctoral-research-agent/papers/cbf2026/status.md`
- Modify in DRA: `/Users/xirhxq/Documents/Clones/doctoral-research-agent/papers/cbf2026/open-questions.md`
- Modify in DRA: `/Users/xirhxq/Documents/Clones/doctoral-research-agent/papers/cbf2026/timeline.md`
- Modify in DRA: `/Users/xirhxq/Documents/Clones/doctoral-research-agent/papers/cbf2026/theory/2026-07-28-rate-aware-robust-cbf-closure.md`
- Modify in DRA: `/Users/xirhxq/Documents/Clones/doctoral-research-agent/meta-log/2026-07-28-cbf2026-dynamic-localization-dag-calibration-design.md`
- Create in DRA: `/Users/xirhxq/Documents/Clones/doctoral-research-agent/meta-log/2026-07-28-cbf2026-dynamic-localization-calibration-results.md`

**Interfaces:**
- Consumes reviewed Task 4 summary/report only; never transcribe numbers from terminal memory.
- Produces manuscript graph definition, piecewise derivative scope, minimal calibration-estimator description, one compact calibration result table, and evidence limitations.
- Produces DRA provenance for code commits, bundle hashes, pass/fail decision, and remaining gaps.

- [ ] **Step 1: Correct the manuscript's two-graph semantics**

In `main.tex`, replace claims that the FIM graph/reference sets are fixed with:

\[
\mathcal R_i(t)
=
\mathcal R_i^{\mathrm{CBF}}
\cup
\mathcal B_i^{\mathrm{visible}}(t)
\cup
\mathcal A_i^{\mathrm{visible}}(t).
\]

Define assigned CBF references as always admitted, other bases as range gated,
and other same-squad strict-lower-index UAVs as range gated. Prove every active
graph is a DAG using the bases-first/increasing-local-index order. Retain the
fixed two-reference localization CBF constraints; do not apply connectivity
rows to optional FIM-only references.

Change derivative language to piecewise validity on intervals with unchanged
active set. State that transitions are logged empirically and no jump reserve
is claimed.

- [ ] **Step 2: Add the minimal estimator/calibration paragraph**

Describe only the approved WNLS objective, `ranging_sigma=0.5 m`, known
deployment initialization, previous-estimate initialization, and topological
per-frame processing. State it is an offline calibration estimator and not a
paper contribution. Do not add estimator comparison prose or EKF/UKF
discussion.

- [ ] **Step 3: Add evidence according to the actual gate result**

If `adequacy.passed=true`, add one compact table with dynamic/fixed graph:
containment, failures, \(\lambda_{\min}\) lower-tail, condition upper-tail,
\(\epsilon\) upper-tail, and transitions. Use “empirical containment” and
“paired graph ablation,” not “probabilistic safety guarantee.”

If `adequacy.passed=false`, still correct the graph/theory text but do not
upgrade epsilon to a validated bound. Report the observed calibration gap in
the limitations paragraph and state that the current radius remains a design
radius pending further calibration. Do not tune or inflate in this task.

In either case, disclose that the 20 noise seeds reuse one preserved trajectory
geometry and that estimates did not enter the controller.

- [ ] **Step 4: Build the paper without overwriting the user's PDF**

Create an isolated temporary build directory and copy the named submission
sources/assets there. Run the repository build sequence (`pdflatex`, `bibtex`,
`pdflatex`, `pdflatex`) in that directory. Verify:

- exit code zero;
- no unresolved references or citations;
- no LaTeX warning;
- no overfull box; and
- resulting page count is recorded.

Delete only the temporary build directory after recording hashes. Do not
overwrite the tracked/user-modified `main.pdf`.

- [ ] **Step 5: Update DRA from the evidence files**

Update only the named CBF2026 files. Preserve PodSearch dirty files. Record:

- the fixed-versus-dynamic correction;
- code commits and input/output hashes;
- Stage 1/2 commands and sizes;
- coverage and graph-ablation values;
- disk cache deletion and final free-space probes;
- whether the adequacy gate passed;
- paper commit/build evidence; and
- remaining controller/cross-correlation/geometry limitations.

- [ ] **Step 6: Commit the paper and DRA with exact staging**

Paper repo:

```bash
git add -- main.tex
git commit -m "feat(paper): add dynamic DAG calibration evidence"
```

DRA repo:

```bash
git add -- \
  papers/cbf2026/status.md \
  papers/cbf2026/open-questions.md \
  papers/cbf2026/timeline.md \
  papers/cbf2026/theory/2026-07-28-rate-aware-robust-cbf-closure.md \
  meta-log/2026-07-28-cbf2026-dynamic-localization-dag-calibration-design.md \
  meta-log/2026-07-28-cbf2026-dynamic-localization-calibration-results.md
git commit -m "docs(cbf2026): record dynamic DAG calibration"
```

Do not stage `main.pdf`, `rebuttal.*`, `AGENTS.md`, or any PodSearch file.

- [ ] **Step 7: Final cross-repository verification**

Run:

```bash
git -C /private/tmp/cbf2026-diagnostic status --short --branch
git -C /Users/xirhxq/Documents/Clones/cbf/papers/CBF2026 status --short --branch
git -C /Users/xirhxq/Documents/Clones/doctoral-research-agent status --short --branch
df -k /private/tmp
du -sk /private/tmp/cbf2026-results \
  /private/tmp/cbf2026-localization-calibration
```

Verify named commits contain only their intended files, all code/Python tests
pass freshly, the isolated paper build passes freshly, result cache stays
below 2 GB, and free space remains above 6 GB.
