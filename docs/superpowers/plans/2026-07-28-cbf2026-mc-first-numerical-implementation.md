# CBF2026 MC-First Numerical Revision Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and execute a disk-guarded 20 s numerical smoke matrix for rate-aware, bounded-input, explicit-pair CBF control.

**Architecture:** Split covariance/rate computation from CBF construction so every UAV uses a consistent two-phase uncertainty snapshot. Add configurable analytical rate-aware communication/collision rows and hard command bounds, then extend the existing reproducible runner/analyzer with four new cases and MC-facing metrics.

**Tech Stack:** C++17, CMake 3.16+, doctest, Eigen, nlohmann/json, Gurobi 12, Python 3.11 in conda environment `cbf_env`, NumPy 1.24, Python `unittest`.

## Global Constraints

- Work only in `/private/tmp/cbf2026-diagnostic` on `codex/cbf2026-diagnostic`; do not touch `cbf-research-2026` or its stash.
- Preserve the original FIM calculation and lower-local-index anchor rule.
- Scope the controller to the planar velocity-command model; add no lower-level dynamics or sampled-data reserve.
- Do not add uncertainty-jump or previous-command-mismatch reserves; log their empirical magnitudes.
- New hard CBFs use `alpha.coefficient=0.1` and `alpha.power=1`.
- Bounded cases enforce `|vx|<=25`, `|vy|<=25`, and `|yawRateRad|<=0.35` inside the QP; never clip after solve.
- Use positive backward-difference uncertainty rate; the first valid sample has rate zero.
- Use start-space `8,000,000,000` bytes, live floor `6,000,000,000` bytes, output-root cap `2,000,000,000` bytes, and per-run cap `250,000,000` bytes.
- Run commands in Python environment `cbf_env`.
- Make all production changes test-first and record the observed RED failure in each task report.
- Do not delete or overwrite existing raw diagnostic evidence.
- Do not commit without the user's standing approval; never add `Co-Authored-By`.

---

### Task 1: Consistent covariance and uncertainty-rate snapshots

**Files:**
- Modify: `include/Robot.hpp`
- Modify: `include/Swarm.hpp`
- Modify: `include/communicators/CommunicatorBase.hpp`
- Modify: `include/communicators/CommunicatorCentral.hpp`
- Create: `tests/testRobustConstraintConstruction.cpp`
- Modify: `CMakeLists.txt` only if the existing test glob does not pick up the new file

**Interfaces:**
- Produces: `Robot::updateCovarianceAndRate(double dt)`.
- Produces: `Robot::currentUncertainty`, `Robot::previousUncertainty`, `Robot::uncertaintyRate`, and `Robot::hasUncertaintyHistory`.
- Produces: communicator map `_othersUncertaintyRate` and virtual methods `sendUncertaintyRate(int,double)` / `receiveUncertaintyRate(int,double)`.
- Preserves: `Robot::getCovariance(json&)` as the original FIM implementation.

- [ ] **Step 1: Write the failing snapshot/rate tests**

Add doctest cases that set a robot's covariance-derived uncertainty history
directly and exercise the wished-for helper:

```cpp
TEST_CASE("positive backward uncertainty rate ignores decreases") {
    Robot robot = makeDiagnosticRobot();
    robot.hasUncertaintyHistory = true;
    robot.previousUncertainty = 10.0;
    CHECK(robot.positiveBackwardUncertaintyRate(12.0, 0.5) == doctest::Approx(4.0));
    CHECK(robot.positiveBackwardUncertaintyRate(8.0, 0.5) == doctest::Approx(0.0));
}

TEST_CASE("first covariance sample has zero uncertainty rate") {
    Robot robot = makeDiagnosticRobot();
    robot.hasUncertaintyHistory = false;
    robot.updateUncertaintyHistory(7.0, 0.5);
    CHECK(robot.currentUncertainty == doctest::Approx(7.0));
    CHECK(robot.uncertaintyRate == doctest::Approx(0.0));
}

TEST_CASE("communicator exchanges the current uncertainty rate") {
    CommunicatorCentral communicator(settings);
    communicator.receiveUncertaintyRate(3, 1.25);
    CHECK(communicator._othersUncertaintyRate.at(3) == doctest::Approx(1.25));
}

TEST_CASE("two-phase refresh publishes current covariance and rate before CBF construction") {
    auto robots = makeTwoDiagnosticRobots();
    exchangeDiagnosticData(robots);
    refreshDiagnosticUncertaintySnapshot(robots, 0.5);
    CHECK(robots[0]->comm->_othersPositionCovariance.at(2).isApprox(
        robots[1]->positionCovariance
    ));
    CHECK(robots[0]->comm->_othersUncertaintyRate.at(2)
          == doctest::Approx(robots[1]->uncertaintyRate));

    const auto covarianceBefore = robots[0]->positionCovariance;
    const auto rateBefore = robots[0]->uncertaintyRate;
    robots[0]->setFixedCommCBF(
        robots[0]->settings["cbfs"]["without-slack"]["comm-fixed"]
    );
    CHECK(robots[0]->positionCovariance.isApprox(covarianceBefore));
    CHECK(robots[0]->uncertaintyRate == doctest::Approx(rateBefore));
}

TEST_CASE("covariance refresh is independent of comm-fixed enforcement") {
    auto robots = makeTwoDiagnosticRobots();
    robots[0]->settings["cbfs"]["without-slack"]["comm-fixed"]["on"] = false;
    refreshDiagnosticUncertaintySnapshot(robots, 0.5);
    CHECK(robots[0]->hasUncertaintyHistory);
}
```

Define the minimal factories and exchange/refresh fixture locally in
`tests/testRobustConstraintConstruction.cpp`, because each `tests/*.cpp`
translation unit is built as a separate executable. Do not import an anonymous
namespace helper from `tests/testRobotDiagnostics.cpp`, and do not mock the
covariance math.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
cmake --build build-diagnostic --target testRobustConstraintConstruction -j2
./build-diagnostic/testRobustConstraintConstruction
```

Expected: compilation fails because the rate/history and communicator
interfaces do not exist.

- [ ] **Step 3: Implement the minimal history and exchange interfaces**

Add the following public Robot state and behavior:

```cpp
double previousUncertainty = 0.0;
double currentUncertainty = 0.0;
double uncertaintyRate = 0.0;
bool hasUncertaintyHistory = false;

double positiveBackwardUncertaintyRate(double nextUncertainty, double dt) const;
void updateUncertaintyHistory(double nextUncertainty, double dt);
void updateCovarianceAndRate(double dt);
```

`positiveBackwardUncertaintyRate` rejects non-positive `dt` with
`std::invalid_argument`. `updateCovarianceAndRate` calls the unchanged
`getCovariance()` with the fixed-communication configuration, converts the
new covariance using `uncertaintyFromCovarianceFunction`, and updates history.
Remove the `getCovariance()` call from `setFixedCommCBF()`.

Extend `exchangeData()` to send `uncertaintyRate`. In the loop, use:

```cpp
exchangeData();
for (auto &robot : robots) robot->updateCovarianceAndRate(tStep);
exchangeData();
for (auto &robot : robots) robot->postsetCBF();
```

Do not advance state or solve between the two exchanges.

- [ ] **Step 4: Verify GREEN and regression tests**

Run:

```bash
cmake --build build-diagnostic --target testRobustConstraintConstruction testRobotDiagnostics Swarm -j2
./build-diagnostic/testRobustConstraintConstruction
./build-diagnostic/testRobotDiagnostics
```

Expected: all doctest cases pass.

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt include/Robot.hpp include/Swarm.hpp \
  include/communicators/CommunicatorBase.hpp \
  include/communicators/CommunicatorCentral.hpp \
  tests/testRobustConstraintConstruction.cpp
git commit -m "feat(cbf): synchronize uncertainty-rate snapshots"
```

### Task 2: Analytical rate-aware communication and collision rows

**Files:**
- Modify: `include/Robot.hpp`
- Modify: `tests/testRobustConstraintConstruction.cpp`
- Modify: `config/config.json`

**Interfaces:**
- Consumes: current/neighbor uncertainty rates from Task 1.
- Produces: config `cbfs.uncertainty-rate.mode` with values `off` and `backward-difference-positive`.
- Produces: config `cbfs.without-slack.safety.mode` with values `minimum` and `pairwise`.
- Produces: stable pair names `safetyCBF(#<id>)`.

- [ ] **Step 1: Write failing analytical-row tests**

Create hand-derived fixtures with \(k=1\), \(p_i=(3,4)\), \(p_j=(0,0)\),
\(v_j=(2,-1)\), and \(\nu_i+\nu_j=0.7\). Assert:

```cpp
CHECK(localizationCBF.dhdt(x, 0.0) == doctest::Approx(0.4 - 0.7));
CHECK(collisionCBF.dhdt(x, 0.0) == doctest::Approx(-0.4 - 0.7));
```

where \(n=(0.6,0.8)\) and \(n^Tv_j=0.4\); use the exact hand-derived value
from the fixture rather than computing the expectation with production
helpers. Add a three-UAV test that expects two named pairwise safety rows
for one robot.

- [ ] **Step 2: Verify RED**

Run:

```bash
cmake --build build-diagnostic --target testRobustConstraintConstruction -j2
./build-diagnostic/testRobustConstraintConstruction
```

Expected: tests fail because the analytical helpers and pairwise mode are
missing.

- [ ] **Step 3: Implement rate-aware analytical rows**

For communication:

```text
dhdt = k * n dot received_neighbor_velocity
       - k * (my_rate + neighbor_rate)
```

when rate mode is enabled. Base anchors use zero velocity and zero neighbor
rate.

For collision pair `(i,j)`:

```text
h    = k * (distance - safe_distance - epsilon_i - epsilon_j)
dhdx = k * n
dhdt = -k * n dot received_neighbor_velocity
       - k * (my_rate + neighbor_rate)
```

`minimum` mode constructs the analytical row for the pair with the smallest
current robust `h`. `pairwise` mode constructs one row for every other UAV.
Keep `consider-uncertainty=false` behavior by zeroing uncertainty and rate
terms together.

- [ ] **Step 4: Verify GREEN and build**

Run:

```bash
cmake --build build-diagnostic --target testRobustConstraintConstruction testDiagnosticConfiguration Swarm -j2
./build-diagnostic/testRobustConstraintConstruction
./build-diagnostic/testDiagnosticConfiguration
```

Expected: all tests pass and `Swarm` links.

- [ ] **Step 5: Commit**

```bash
git add include/Robot.hpp tests/testRobustConstraintConstruction.cpp config/config.json
git commit -m "feat(cbf): add rate-aware pairwise constraints"
```

### Task 3: Hard command bounds and safe infeasibility handling

**Files:**
- Modify: `include/Robot.hpp`
- Modify: `include/optimisers/Gurobi.hpp`
- Modify: `tests/testOptimisers.cpp`
- Modify: `tests/testRobustConstraintConstruction.cpp`
- Modify: `config/config.json`

**Interfaces:**
- Produces: config `cbfs.input-limits.on`, `planar-component-max`, and `yaw-rate-max`.
- Produces: `opt.input_limits` with configured limits, bound-row count,
  saturation tolerance, and observed saturation flags.

- [ ] **Step 1: Write failing optimizer and bound tests**

Add an optimizer test with `x>=9` and `x<=8`; `solve()` must return without
reading an unavailable primal value and `getStatus().status` must equal
`infeasible`. Add a Robot test that enables limits, optimizes an otherwise
unconstrained distance task, and asserts:

```cpp
CHECK(std::abs(control[0]) <= 25.0 + 1e-7);
CHECK(std::abs(control[1]) <= 25.0 + 1e-7);
CHECK(std::abs(control[2]) <= 0.35 + 1e-7);
```

- [ ] **Step 2: Verify RED**

Run:

```bash
cmake --build build-diagnostic --target testOptimisers testRobustConstraintConstruction -j2
./build-diagnostic/testOptimisers
./build-diagnostic/testRobustConstraintConstruction
```

Expected: the infeasible solve accesses unavailable Gurobi attributes or the
bound test exceeds the wished-for limit.

- [ ] **Step 3: Implement hard QP rows**

After `optimiser->start(totalSize,uSize)`, add six `>=` rows with slack
coefficients zero:

```text
 vx >= -25, -vx >= -25
 vy >= -25, -vy >= -25
 yawRateRad >= -0.35, -yawRateRad >= -0.35
```

when input limits are enabled. Log the limits and saturation state separately
from CBF rows.

After `model->optimize()`, query `GRB_IntAttr_Status` before any `X` or
objective access. Only read primal values and objective for `GRB_OPTIMAL`.
Return a zero vector for non-optimal status; `Robot::optimise()` already
rejects non-optimal status before applying the result.

- [ ] **Step 4: Verify GREEN and all C++ tests**

Run:

```bash
cmake --build build-diagnostic --target testOptimisers testRobustConstraintConstruction testRobotDiagnostics testDiagnosticConfiguration testPolygon Swarm -j2
for test_bin in testOptimisers testRobustConstraintConstruction testRobotDiagnostics testDiagnosticConfiguration testPolygon; do
  ./build-diagnostic/$test_bin || exit $?
done
```

Expected: every doctest executable reports zero failed cases.

- [ ] **Step 5: Commit**

```bash
git add include/Robot.hpp include/optimisers/Gurobi.hpp \
  tests/testOptimisers.cpp tests/testRobustConstraintConstruction.cpp \
  config/config.json
git commit -m "feat(cbf): enforce bounded velocity commands"
```

### Task 4: New diagnostic cases, cache guards, and MC-facing metrics

**Files:**
- Create: `config/diagnostics/c0_corrected.json`
- Create: `config/diagnostics/r_rate_aware.json`
- Create: `config/diagnostics/rb_bounded.json`
- Create: `config/diagnostics/rbp_pairwise.json`
- Modify: `scripts/diagnostics/run_diagnostic.py`
- Modify: `scripts/diagnostics/analyze_diagnostic.py`
- Modify: `tests/test_run_diagnostic.py`
- Modify: `tests/test_analyze_diagnostic.py`

**Interfaces:**
- Extends CLI cases to `C0`, `R`, `RB`, and `RBP` while preserving `H0`,
  `C1`, and `U0`.
- Produces manifest fields `output_root_allocated_bytes`,
  `run_allocated_bytes`, `output_root_cap_bytes`, and `run_cap_bytes`.
- Produces summary blocks `input_limits`, `uncertainty_jumps`,
  `control_mismatch`, `pairwise_row_coverage`, and `practical_tolerance`.

- [ ] **Step 1: Write failing runner tests**

Add literal expected case truth:

```python
expected = {
    "C0": ("off", False, "minimum"),
    "R": ("backward-difference-positive", False, "minimum"),
    "RB": ("backward-difference-positive", True, "minimum"),
    "RBP": ("backward-difference-positive", True, "pairwise"),
}
```

Add tests where mocked allocated sizes exceed `2_000_000_000` for the output
root or `250_000_000` for the run. The child must be stopped and the manifest
must contain `cache_root_cap` or `cache_run_cap`.

- [ ] **Step 2: Verify runner RED**

Run:

```bash
conda run -n cbf_env python -m unittest tests.test_run_diagnostic -v
```

Expected: unknown new cases and missing cache guards fail.

- [ ] **Step 3: Implement case materialization and cache guards**

Use filesystem allocated bytes from `st_blocks * 512`, recursively summed
without following symlinks. Monitor both the new output root and current run
beside the existing free-space floor. Preserve terminal-manifest behavior for
all guard exits.

- [ ] **Step 4: Write failing analyzer tests**

Construct a two-frame literal fixture with known positions, controls,
uncertainties, pairwise row names, and input limits. Assert hand-derived:

- maximum positive uncertainty jump;
- projected neighbor-command mismatch;
- expected/observed pairwise rows;
- strict tightened minima and `-0.5 m` practical-tolerance classification;
- zero input-bound violations and saturation counts;
- bounded feasibility headroom `25 - minimum_required_linf`.

- [ ] **Step 5: Verify analyzer RED**

Run:

```bash
conda run -n cbf_env python -m unittest tests.test_analyze_diagnostic -v
```

Expected: new summary keys are absent.

- [ ] **Step 6: Implement analyzer metrics**

Derive metrics only from materialized configuration and logged consecutive
frames. Use `None`/`unavailable` when the required fields are absent. Preserve
all historical H0/C1/U0 analysis behavior.

- [ ] **Step 7: Verify all Python tests**

Run:

```bash
conda run -n cbf_env python -m unittest discover -s tests -p 'test_*.py' -v
```

Expected: all tests pass.

- [ ] **Step 8: Commit**

```bash
git add config/diagnostics scripts/diagnostics \
  tests/test_run_diagnostic.py tests/test_analyze_diagnostic.py
git commit -m "feat(diagnostics): add MC-first numerical cases"
```

### Task 5: Execute the 20 s smoke gate and record evidence

**Files:**
- Generated outside Git: `/private/tmp/cbf2026-results/mc-first-smoke/<case>/...`
- Create: `docs/diagnostics/2026-07-28-mc-first-smoke-gate.md`
- Modify: DRA files only after the compact report is complete

**Interfaces:**
- Consumes: `build-diagnostic/Swarm` and cases from Task 4.
- Produces: one terminal manifest and one analyzer summary per executed case.

- [ ] **Step 1: Fresh preflight**

Run:

```bash
df -k /private/tmp
du -sk /private/tmp/cbf2026-results
git status --short --branch
cmake --build build-diagnostic --target Swarm -j2
conda run -n cbf_env python -c \
  "from pathlib import Path; from scripts.diagnostics.run_diagnostic import require_start_space; print(require_start_space(Path('/private/tmp')))"
```

Stop before allocation if any gate fails.

- [ ] **Step 2: Run and analyze `C0`**

Run:

```bash
conda run -n cbf_env python -m scripts.diagnostics.run_diagnostic \
  --case C0 --horizon 20 --seed 20260727 \
  --binary build-diagnostic/Swarm \
  --output-root /private/tmp/cbf2026-results/mc-first-smoke
```

Analyze the exact manifest-selected run root. Stop on a technical-invalid
result.

- [ ] **Step 3: Repeat guarded execution for `R`, `RB`, and `RBP`**

Before each case rerun the free-space and allocated-cache probes. Apply the
dependency stops from the design: do not run `RB` after an invalid `R`, and do
not attribute an infeasible `RBP` to pairwise semantics if `RB` is infeasible.

- [ ] **Step 4: Write the compact smoke report**

The report records exact commands, commit, dirty status, config/source/binary
hashes, disk values, row counts, solver outcomes, strict/tolerance margins,
input metrics, uncertainty/rate/jump/mismatch metrics, and the stop decision.
Do not copy raw arrays.

- [ ] **Step 5: Verify evidence bindings**

Run:

```bash
git diff --check
shasum -a 256 docs/diagnostics/2026-07-28-mc-first-smoke-gate.md
```

For every executed bundle, independently recompute and compare the
materialized-config, binary, and source-snapshot hashes stored in its manifest.

- [ ] **Step 6: Commit the compact code-repository report**

```bash
git add docs/diagnostics/2026-07-28-mc-first-smoke-gate.md
git commit -m "docs(diagnostics): record MC-first smoke gate"
```

- [ ] **Step 7: Update DRA**

Record the design, commands, hashes, evidence boundary, result, and next
decision in `doctoral-research-agent/papers/cbf2026/`. Keep DRA on local
`main`, do not push, and do not copy raw trajectories.
