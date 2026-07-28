# CBF2026 Minimal Diagnostic Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a reproducible, disk-guarded diagnostic harness that establishes whether the archived CBF2026 controller supports the manuscript-facing safety, localization-connectivity, bounded-input, and runtime claims.

**Architecture:** Keep the C++ simulator as the source of trajectory and QP evidence, adding only deterministic initialization, trustworthy solver handling, correct configuration parsing, and diagnostic logging before the first runs. A standard-library Python runner materializes three deep-merged configurations, enforces disk limits while supervising the simulator, and writes collision-resistant per-run bundles whose manifests bind complete source snapshots. A NumPy-only analyzer derives constraint, pairwise-distance, control-demand, uncertainty-rate, solver, and coverage metrics from each retained `data.json`.

**Tech Stack:** C++17, CMake 3.16+, doctest, Eigen, nlohmann/json, Gurobi 12, Python 3.11 in conda environment `cbf_env`, NumPy 1.24, Python `unittest`.

## Global Constraints

- Authoritative base commit: `47e7b3d6f5b1be74a3dfa32150b4246c619e9a0d`.
- Implementation branch: `codex/cbf2026-diagnostic`.
- Worktree: `/private/tmp/cbf2026-diagnostic`.
- Do not apply, drop, rewrite, or otherwise modify `stash@{0}` in the main `cbf-research-2026` checkout.
- Do not import second-order, HOCBF, ActiveSearch, or LinkDenied research code.
- Use `conda run -n cbf_env python ...` for every Python command.
- Use Gurobi as the authoritative diagnostic solver; configure HiGHS and OSQP off.
- Disk start threshold: 8,000,000,000 available bytes.
- Disk hard floor: 6,000,000,000 available bytes.
- Do not launch a build or run below the start threshold.
- Apply the start guard to the nearest existing output ancestor before creating
  output, case, or run directories.
- Require `output_root` to resolve outside `project_root`; reject equal or
  nested paths before the start guard and before any allocation.
- Terminate a running simulator before available space falls below the hard floor.
- Do not delete raw evidence, Git objects, or stashes automatically.
- Do not edit the manuscript during this diagnostic gate.
- Do not make any Git commit without explicit user confirmation.

---

### Task 1: Disk-Guarded Configuration Materialization

**Files:**
- Create: `scripts/diagnostics/__init__.py`
- Create: `scripts/diagnostics/run_diagnostic.py`
- Create: `config/diagnostics/h0_historical.json`
- Create: `config/diagnostics/c1_claim_aligned.json`
- Create: `config/diagnostics/u0_uncertainty_ablation.json`
- Test: `tests/test_run_diagnostic.py`

**Interfaces:**
- Produces: `deep_merge(base: dict, patch: dict) -> dict`.
- Produces: `available_bytes(path: pathlib.Path) -> int`.
- Produces: `require_start_space(path: pathlib.Path, threshold_bytes: int) -> int`.
- Produces: `materialize_config(base_path: Path, patch_path: Path, output_path: Path, horizon_s: float, seed: int) -> dict`.
- Produces: CLI `python -m scripts.diagnostics.run_diagnostic --case CASE --horizon SECONDS --seed SEED --binary PATH --output-root PATH`.
- The CLI's default output root is the external
  `<project-parent>/cbf2026-diagnostic-results`; explicit in-project roots are
  unsupported.
- Produces: one atomic, collision-resistant
  `<output-root>/<case>/<runner-run-id>/` per invocation.
- Produces inside that run root: `manifest.json`,
  `config.materialized.json`, `stdout.log`, `stderr.log`, and
  `source-snapshot.tar.gz`; the simulator writes its timestamped `data.json`
  child below the same root.
- Prints a manifest whose `output_dir` identifies the unique run root; callers
  must use that path rather than the case directory.

- [ ] **Step 1: Write deep-merge and disk-threshold tests**

```python
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts.diagnostics.run_diagnostic import (
    DiskSpaceError,
    deep_merge,
    require_start_space,
)


class DiagnosticRunnerTests(unittest.TestCase):
    def test_deep_merge_preserves_unmodified_nested_values(self):
        base = {"execute": {"time-step": 0.5, "time-total": 500}, "cbfs": {"safety": {"on": False}}}
        patch_value = {"execute": {"time-total": 20}}
        self.assertEqual(
            deep_merge(base, patch_value),
            {"execute": {"time-step": 0.5, "time-total": 20}, "cbfs": {"safety": {"on": False}}},
        )

    @patch("scripts.diagnostics.run_diagnostic.available_bytes", return_value=7_999_999_999)
    def test_start_space_below_eight_gb_aborts(self, _available):
        with self.assertRaises(DiskSpaceError):
            require_start_space(Path("/private/tmp"), 8_000_000_000)
```

- [ ] **Step 2: Run the tests and verify import failure**

Run:

```bash
conda run -n cbf_env python -m unittest discover -s tests -p 'test_run_diagnostic.py' -v
```

Expected: `ModuleNotFoundError` for `scripts.diagnostics.run_diagnostic`.

- [ ] **Step 3: Implement merge, free-space check, and manifest primitives**

Add these definitions to `scripts/diagnostics/run_diagnostic.py`:

```python
START_BYTES = 8_000_000_000
HARD_FLOOR_BYTES = 6_000_000_000


class DiskSpaceError(RuntimeError):
    pass


def deep_merge(base: dict, patch_value: dict) -> dict:
    result = copy.deepcopy(base)
    for key, value in patch_value.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def available_bytes(path: Path) -> int:
    return shutil.disk_usage(path).free


def require_start_space(path: Path, threshold_bytes: int = START_BYTES) -> int:
    free = available_bytes(path)
    if free < threshold_bytes:
        raise DiskSpaceError(f"available={free} below start threshold={threshold_bytes}")
    return free
```

The manifest must contain `case`, `base_commit`, `working_tree_dirty`, `branch`,
`seed`, `horizon_s`, `solver`, `config_path`, `config_sha256`,
`binary_sha256`, `binary_path`,
`source_snapshot_sha256`, `source_snapshot_path`, `source_snapshot_format`,
`source_snapshot_policy`, `free_bytes_before`, `free_bytes_after`,
`started_at`, `ended_at`, `returncode`, `termination_reason`, and `output_dir`.
The source archive is created with the Python standard library before simulator
launch and includes tracked and untracked reconstruction inputs, including
`scripts/diagnostics/__init__.py`; it excludes Git metadata, build products,
raw output, hidden SDD coordination, and
`docs/diagnostics/source-snapshots`.
After allocation, configuration materialization, source snapshot creation,
hashing, or log setup failure returns a terminal manifest with nullable
unavailable identity fields, `returncode=null`,
`termination_reason="runner_setup_error"`, and structured `error.type` and
`error.message`.
`Popen` failure uses `termination_reason="simulator_launch_error"`.
Any live-monitor exception terminates the child through the five-second
terminate-to-kill fallback before publishing
`termination_reason="runner_monitor_error"`; a failed terminal disk probe
sets `free_bytes_after=null`.
The API returns these manifests rather than re-raising; the CLI prints them
and exits `2`.
Pre-allocation validation and disk-guard failures still raise without creating
a bundle.

- [ ] **Step 4: Add exact case patches**

`config/diagnostics/h0_historical.json`:

```json
{
  "case": "H0",
  "overrides": {
    "initial": {"position": {"method": "specified"}},
    "execute": {
      "random-seed": 20260727,
      "check-constraint-violation": false
    },
    "debug": {"opt-cbc": false}
  }
}
```

`config/diagnostics/c1_claim_aligned.json`:

```json
{
  "case": "C1",
  "overrides": {
    "initial": {"position": {"method": "specified"}},
    "cbfs": {
      "without-slack": {
        "safety": {"on": true, "consider-uncertainty": true},
        "comm-fixed": {"on": true, "consider-uncertainty": true}
      }
    },
    "execute": {
      "random-seed": 20260727,
      "check-constraint-violation": true
    },
    "debug": {"opt-cbc": false}
  }
}
```

`config/diagnostics/u0_uncertainty_ablation.json`:

```json
{
  "case": "U0",
  "overrides": {
    "initial": {"position": {"method": "specified"}},
    "cbfs": {
      "without-slack": {
        "safety": {"on": true, "consider-uncertainty": false},
        "comm-fixed": {"on": true, "consider-uncertainty": false}
      }
    },
    "execute": {
      "random-seed": 20260727,
      "check-constraint-violation": true
    },
    "debug": {"opt-cbc": false}
  }
}
```

`materialize_config` must deep-merge `overrides` into `config/config.json`, overwrite `execute.time-total` from `--horizon`, overwrite `execute.random-seed` from `--seed`, set `output_path` to the unique runner-owned run root, and set `run_suffix` to `_<case>_seed_<seed>_<horizon>s`.

- [ ] **Step 5: Supervise the simulator with a hard-floor monitor**

Use `subprocess.Popen` and poll at 0.5 s.
Create both log files before launch.
Define the terminal manifest contract immediately after allocating the run
root, then catch all materialization/archive/hash/log setup exceptions as
`runner_setup_error` and `Popen` exceptions as `simulator_launch_error`.
Publish every terminal manifest through a temporary file and atomic rename.
If setup or launch fails, preserve the complete-as-failed bundle, return its
manifest from `run_diagnostic`, and make the CLI exit `2`.
Wrap the entire live monitor.
On any monitor exception, terminate the child, wait at most five seconds,
escalate to kill when necessary, safely allow a missing final disk
measurement, publish `runner_monitor_error`, and make the CLI exit `2`.
If `available_bytes(output_root) < HARD_FLOOR_BYTES`, call `terminate()`, wait five seconds, then call `kill()` only if still running.
Write `termination_reason="disk_hard_floor"` in the manifest.
Otherwise record `completed`, `simulator_nonzero_exit`, or `simulator_signal`.

- [ ] **Step 6: Run runner tests**

Run:

```bash
conda run -n cbf_env python -m unittest discover -s tests -p 'test_run_diagnostic.py' -v
```

Expected: all Task 1 tests pass.

- [ ] **Step 7: Preserve an uncommitted checkpoint**

Run:

```bash
git diff --check
git status --short
```

Expected: only Task 1 files plus the approved design/plan documents are uncommitted.
Do not commit; ask for explicit approval at a later milestone.

---

### Task 2: Deterministic Seed and Class-\(\mathcal K\) Configuration

**Files:**
- Modify: `include/utils.h`
- Modify: `src/Swarm.cpp`
- Create: `include/cbf/CBFConfig.hpp`
- Modify: `include/Robot.hpp`
- Test: `tests/testDiagnosticConfiguration.cpp`

**Interfaces:**
- Produces: `unsigned int resolveRandomSeed(const json&, unsigned int)`.
- Produces: `unsigned int seedRandomFromConfig(const json&, unsigned int)`.
- Produces: `ClassKParameters readClassKParameters(const json&, double, int)`.
- `ClassKParameters` has fields `double coefficient` and `int power`.

- [ ] **Step 1: Write failing deterministic-seed and nested-alpha tests**

```cpp
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "utils.h"
#include "cbf/CBFConfig.hpp"

TEST_CASE("ConfiguredRandomSeedOverridesFallback") {
    json settings = {{"execute", {{"random-seed", 1234}}}};
    CHECK(resolveRandomSeed(settings, 99) == 1234u);
    CHECK(resolveRandomSeed(json::object(), 99) == 99u);
}

TEST_CASE("NestedClassKConfigurationIsRead") {
    json config = {{"alpha", {{"coe", 0.5}, {"pow", 3}}}};
    ClassKParameters parameters = readClassKParameters(config, 0.1, 1);
    CHECK(parameters.coefficient == doctest::Approx(0.5));
    CHECK(parameters.power == 3);
}
```

- [ ] **Step 2: Configure a Gurobi-only build and verify compilation fails**

Before configuring, run:

```bash
conda run -n cbf_env python -c "from pathlib import Path; from scripts.diagnostics.run_diagnostic import require_start_space; require_start_space(Path('/private/tmp'))"
```

Then run:

```bash
cmake -S . -B build-diagnostic -DCMAKE_BUILD_TYPE=Release -DENABLE_GUROBI=ON -DENABLE_HIGHS=OFF -DENABLE_OSQP=OFF
cmake --build build-diagnostic --target testDiagnosticConfiguration -j2
```

Expected: build fails because `CBFConfig.hpp`, `resolveRandomSeed`, and `readClassKParameters` do not exist.

- [ ] **Step 3: Implement deterministic seeding**

Add the two seed helpers to `include/utils.h`.
Replace `srand(time(NULL))` in `src/Swarm.cpp` with:

```cpp
json settings = json::parse(std::ifstream(configPath));
seedRandomFromConfig(settings, static_cast<unsigned int>(time(NULL)));
```

- [ ] **Step 4: Implement nested alpha parsing and wire every local Robot CBF**

`include/cbf/CBFConfig.hpp`:

```cpp
#ifndef CBF_CBF_CONFIG_HPP
#define CBF_CBF_CONFIG_HPP

#include "utils.h"

struct ClassKParameters {
    double coefficient;
    int power;
};

inline ClassKParameters readClassKParameters(
    const json &config,
    double defaultCoefficient,
    int defaultPower
) {
    const json alpha = config.value("alpha", json::object());
    return {
        alpha.value("coe", defaultCoefficient),
        alpha.value("pow", defaultPower)
    };
}

#endif
```

Replace every `config.value("alpha/coe", ...)` and `config.value("alpha/pow", ...)` in `include/Robot.hpp`.
Include `cbf/CBFConfig.hpp` from `include/Robot.hpp`.
Call `setAlphaClassK` for the safety CBF as well, using its configured nested values.

- [ ] **Step 5: Build and run the focused test**

Run:

```bash
cmake --build build-diagnostic --target testDiagnosticConfiguration -j2
./build-diagnostic/testDiagnosticConfiguration
```

Expected: all tests pass.

- [ ] **Step 6: Preserve an uncommitted checkpoint**

Run `git diff --check` and inspect `git diff -- include/utils.h include/cbf/CBFConfig.hpp include/Robot.hpp src/Swarm.cpp tests/testDiagnosticConfiguration.cpp`.
Do not commit.

---

### Task 3: Trustworthy Nominal Control and Failed-Solve Handling

**Files:**
- Modify: `include/Robot.hpp`
- Test: `tests/testRobotDiagnostics.cpp`

**Interfaces:**
- Consumes: existing `OptimiserBase`.
- Produces: `Robot::optimise()` applies a result only when `getStatus()["status"] == "optimal"`.
- Produces: failed solve sets `robot.opt.status="failed"` and throws `std::runtime_error` before changing the model control.

- [ ] **Step 1: Write a failing zero-nominal integration test**

Create a one-UAV, no-CBF configuration in `makeSingleRobotNoCbfConfig`.
Select Gurobi from `getAvailableOptimisers`.
Define `EIGEN_INITIALIZE_MATRICES_BY_NAN` before including `Robot.hpp` so the archived uninitialized vector fails deterministically:

```cpp
#define EIGEN_INITIALIZE_MATRICES_BY_NAN
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "Robot.hpp"
```

```cpp
TEST_CASE("RobotUsesZeroNominalWhenNoPolicyIsConfigured") {
    json settings = makeSingleRobotNoCbfConfig("Gurobi");
    Robot robot(1, settings);
    robot.optimise();
    Eigen::VectorXd control = robot.model->getControlInput();
    REQUIRE(control.size() == 3);
    CHECK(control[0] == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(control[1] == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(control[2] == doctest::Approx(0.0).epsilon(1e-12));
}
```

- [ ] **Step 2: Build and run the zero-nominal test**

Run:

```bash
cmake --build build-diagnostic --target testRobotDiagnostics -j2
./build-diagnostic/testRobotDiagnostics
```

Expected: the test fails because the archived nominal vector is initialized to NaN by the test build.

- [ ] **Step 3: Initialize nominal control**

Change:

```cpp
VectorXd uNominal(model->uSize());
```

to:

```cpp
VectorXd uNominal = VectorXd::Zero(model->uSize());
```

- [ ] **Step 4: Write a fake-infeasible-optimizer test**

Implement `FakeInfeasibleOptimiser : public OptimiserBase`.
Its `solve()` returns a nonzero vector, while `getStatus()` returns `{{"status", "infeasible"}}`.

```cpp
TEST_CASE("RobotDoesNotApplyControlFromNonOptimalSolve") {
    json settings = makeSingleRobotNoCbfConfig("Gurobi");
    Robot robot(1, settings);
    robot.model->setControlInput((Eigen::Vector3d() << 0.25, -0.5, 0.1).finished());
    robot.optimiser = std::make_unique<FakeInfeasibleOptimiser>(settings["cbfs"]["objective-function"]);

    CHECK_THROWS_AS(robot.optimise(), std::runtime_error);
    Eigen::VectorXd control = robot.model->getControlInput();
    CHECK(control[0] == doctest::Approx(0.25));
    CHECK(control[1] == doctest::Approx(-0.5));
    CHECK(control[2] == doctest::Approx(0.1));
    CHECK(robot.opt.at("status") == "failed");
}
```

- [ ] **Step 5: Run the fake-infeasible test and verify failure**

Expected before implementation: the fake result is applied or no exception is thrown.

- [ ] **Step 6: Check status before applying the result**

Immediately after `solve()`:

```cpp
json solverStatus = optimiser->getStatus();
opt["solver_info"] = solverStatus;
const std::string status = solverStatus.value("status", "unknown");
if (status != "optimal") {
    opt["status"] = "failed";
    opt["error"] = solverStatus.value("error", status);
    throw std::runtime_error("QP solve failed: " + status);
}
```

Only then extract `u`, call `model->setControlInput(u)`, and set `opt["status"]="success"`.

- [ ] **Step 7: Build and run both robot tests**

Run:

```bash
cmake --build build-diagnostic --target testRobotDiagnostics -j2
./build-diagnostic/testRobotDiagnostics
```

Expected: both tests pass.

- [ ] **Step 8: Preserve an uncommitted checkpoint**

Run `git diff --check`.
Do not commit.

---

### Task 4: Diagnostic Logging and NumPy-Only Metric Extraction

**Files:**
- Modify: `include/Robot.hpp`
- Create: `scripts/diagnostics/analyze_diagnostic.py`
- Test: `tests/test_analyze_diagnostic.py`

**Interfaces:**
- Simulator hard-constraint records add `alpha_coe`, `alpha_pow`, and `residual`.
- Produces: `minimum_linf_bound(constraints: list[dict], control_names: list[str]) -> float`.
- Produces: `analyze_run(data_path: Path, manifest_path: Path) -> dict`.
- Produces: `diagnostic-summary.json` and `diagnostic-summary.md`.

- [ ] **Step 1: Write LP and metric-extraction tests**

```python
import math
import unittest

from scripts.diagnostics.analyze_diagnostic import minimum_linf_bound


class FeasibilityBoundTests(unittest.TestCase):
    def test_single_axis_lower_bound(self):
        constraints = [{"coe": {"vx": 1.0, "vy": 0.0, "yawRateRad": 0.0}, "rhs": 2.0}]
        self.assertAlmostEqual(minimum_linf_bound(constraints, ["vx", "vy", "yawRateRad"]), 2.0)

    def test_diagonal_constraint(self):
        constraints = [{"coe": {"vx": 1.0, "vy": 1.0, "yawRateRad": 0.0}, "rhs": 2.0}]
        self.assertAlmostEqual(minimum_linf_bound(constraints, ["vx", "vy", "yawRateRad"]), 1.0)

    def test_conflicting_constraints_are_infeasible(self):
        constraints = [
            {"coe": {"vx": 1.0}, "rhs": 1.0},
            {"coe": {"vx": -1.0}, "rhs": 1.0}
        ]
        self.assertTrue(math.isinf(minimum_linf_bound(constraints, ["vx"])))
```

- [ ] **Step 2: Run analyzer tests and verify import failure**

Run:

```bash
conda run -n cbf_env python -m unittest discover -s tests -p 'test_analyze_diagnostic.py' -v
```

Expected: module import fails.

- [ ] **Step 3: Implement minimum-\(\ell_\infty\) feasibility by vertex enumeration**

Represent \(z=[u^\top,t]^\top\).
Add the logged hard constraints \(a_k^\top u\ge r_k\), the box epigraph constraints \(u_j+t\ge0\), \(-u_j+t\ge0\), and \(t\ge0\).
Enumerate every combination of `len(control_names)+1` active rows, solve with `numpy.linalg.solve`, reject rank-deficient and infeasible candidates, and return the smallest feasible `t`.
Return `0.0` for no hard constraints and `math.inf` if no candidate is feasible.

- [ ] **Step 4: Log instantiated class-\(\mathcal K\) parameters and residuals**

For every no-slack CBF record in `Robot::optimise()`, add:

```cpp
{"alpha_coe", cbf.getAlphaCoefficient()},
{"alpha_pow", cbf.getAlphaPower()}
```

After an optimal control is available, compute and store:

```cpp
item["residual"] = item["const"].get<double>() + dot(item["coe"], opt["result"]);
```

Use the existing Eigen coefficient vector directly in C++ rather than reparsing JSON.

- [ ] **Step 5: Implement run analysis**

`analyze_run` must:

1. reject missing/invalid JSON with a nonzero CLI exit;
2. count solver statuses and collect `solve_time_ms`;
3. compute finite-value failures;
4. compute hard residual minima and negative counts at tolerance `-1e-7`;
5. compute required-reference nominal and tightened margins from `formation`, `position_covariance`, `uncertainty`, and `config.bases`;
6. compute all-pair nominal and tightened collision margins;
7. compute control norm maxima and quantiles;
8. compute per-frame minimum feasible \(\ell_\infty\) bounds;
9. compute uncertainty maxima and maximum positive one-step rate;
10. compute coverage from unique `update` cells divided by `para.gridWorld.xNum * yNum`;
11. mark RMSE, NEES, ANEES, tracking error, and saturation as `"unavailable_no_distinct_truth_estimate_or_input_bounds"`.

- [ ] **Step 6: Run analyzer tests**

Run:

```bash
conda run -n cbf_env python -m unittest discover -s tests -p 'test_analyze_diagnostic.py' -v
```

Expected: all tests pass.

- [ ] **Step 7: Build and run all focused C++ tests**

Run:

```bash
cmake --build build-diagnostic --target testDiagnosticConfiguration testRobotDiagnostics testOptimisers testPolygon Swarm -j2
./build-diagnostic/testDiagnosticConfiguration
./build-diagnostic/testRobotDiagnostics
./build-diagnostic/testPolygon
```

Run `testOptimisers` only after confirming free space remains above 8 GB because it writes LP diagnostics on failure.

- [ ] **Step 8: Preserve an uncommitted checkpoint**

Run `git diff --check` and `git status --short`.
Do not commit.

---

### Task 5: Run the 20-Second Diagnostic Gate

**Files:**
- Generated, not tracked: `/private/tmp/cbf2026-results/smoke/<case>/...`
- Create: `docs/diagnostics/2026-07-27-smoke-gate.md`

**Interfaces:**
- Consumes: `build-diagnostic/Swarm`.
- Consumes: case patches from Task 1.
- Produces: one unique, source-bound run bundle and summary for H0, C1, and U0.

- [ ] **Step 1: Check disk and repository state**

Run:

```bash
conda run -n cbf_env python -c "from pathlib import Path; from scripts.diagnostics.run_diagnostic import require_start_space; print(require_start_space(Path('/private/tmp')))"
git status --short --branch
```

Expected: free bytes at least `8_000_000_000`; only planned source, test, config, and documentation changes are present.

- [ ] **Step 2: Run H0 for 20 simulated seconds**

```bash
conda run -n cbf_env python -m scripts.diagnostics.run_diagnostic \
  --case H0 --horizon 20 --seed 20260727 \
  --binary build-diagnostic/Swarm \
  --output-root /private/tmp/cbf2026-results/smoke
```

- [ ] **Step 3: Analyze H0**

```bash
conda run -n cbf_env python -m scripts.diagnostics.analyze_diagnostic \
  <H0-output_dir>/<simulator-timestamp-child>/data.json \
  <H0-output_dir>/manifest.json
```

Use the preceding runner JSON's `output_dir` value for `<H0-output_dir>`.
Expected: valid summary explicitly identifies safety as disabled.

- [ ] **Step 4: Recheck disk and run C1**

Repeat the start-space check, then run and analyze C1 with the same horizon and seed.

Expected: summary identifies safety enabled and reports solver/residual/pairwise metrics.

- [ ] **Step 5: Recheck disk and run U0**

Repeat for U0.

Expected: summary identifies collision and communication uncertainty tightening as disabled.

- [ ] **Step 6: Write the smoke-gate report**

`docs/diagnostics/2026-07-27-smoke-gate.md` must contain:

- exact commit, branch, config hashes, and result paths;
- free space before and after each case;
- a H0/C1/U0 comparison table;
- each gate failure with the first time and affected UAV/link;
- whether C1 versus U0 implicates uncertainty tightening;
- whether any full 500 s run is authorized.

- [ ] **Step 7: Preserve an uncommitted checkpoint**

Run `git diff --check`.
Do not delete smoke raw data until the user has seen the report.
Do not commit.

---

### Task 6: Conditional Single Full-Horizon Runs

**Files:**
- Generated, not tracked:
  `/private/tmp/cbf2026-results/full/<case>/<runner-run-id>/...`
- Create: `docs/diagnostics/2026-07-27-full-gate.md`

**Interfaces:**
- Runs only cases whose smoke output is valid and interpretable.
- Produces: H0/C1/U0 500 s summaries where authorized.

- [ ] **Step 1: Apply smoke decision rule**

Do not run a case for 500 s if its smoke result has invalid JSON, non-finite state/control, uncontrolled application of a failed solve, or disk termination.

- [ ] **Step 2: Check disk before each full case**

Require at least `8_000_000_000` free bytes before every case.
The runner must retain the 6 GB live hard-floor monitor.

- [ ] **Step 3: Run each authorized case**

Use the Task 5 command with `--horizon 500` and output root `/private/tmp/cbf2026-results/full`.

- [ ] **Step 4: Analyze immediately after each case**

Use the just-printed `output_dir` and its single simulator timestamp child.
Do not launch the next case until `diagnostic-summary.json` and `.md` exist
beside that child's raw `data.json`, and its size is recorded.

- [ ] **Step 5: Write the full-gate report**

Compare:

- coverage and completion;
- solver status and timing;
- hard residuals;
- nominal and tightened localization/safety margins;
- uncertainty growth;
- maximum actual control and minimum required \(\ell_\infty\) bound.

State one of:

1. `implementation_gate_failed`;
2. `time_varying_uncertainty_gate_failed`;
3. `bounded_input_feasibility_gate_failed`;
4. `minimal_gate_passed_for_five_seed_stage`.

- [ ] **Step 6: Stop before Monte Carlo**

Even if all cases pass, do not run five or twenty seeds in this plan.
Present the evidence and theory recommendation first.

---

### Task 7: Evidence-Bounded DRA Update

**Files:**
- Modify in an isolated DRA worktree: `papers/cbf2026/status.md`
- Modify in an isolated DRA worktree: `papers/cbf2026/timeline.md`
- Modify in an isolated DRA worktree: `papers/cbf2026/open-questions.md`
- Create in an isolated DRA worktree: `meta-log/2026-07-27-cbf2026-minimal-diagnostic.md`

**Interfaces:**
- Consumes: smoke/full manifests and diagnostic reports.
- Produces: factual DRA record without copying raw experiment data.

- [ ] **Step 1: Create an isolated DRA worktree from `main`**

Create the worktree under `/private/tmp`.
Do not switch or dirty the shared DRA main checkout.

- [ ] **Step 2: Record only verified facts**

Record:

- correct code base SHA and diagnostic branch;
- stash preservation and repository isolation;
- H0/C1/U0 configurations actually run;
- exact completed/failed gate status;
- metrics supported by logs;
- metrics still unavailable;
- selected next theory action.

Do not write that the method is safe, feasible, scalable, or validated unless the corresponding gate evidence supports that wording.

- [ ] **Step 3: Cross-link evidence without copying raw data**

Link the source-repository report paths and manifest hashes.
Do not copy `data.json`, binaries, build directories, or plots into DRA.

- [ ] **Step 4: Review DRA diff**

Run `git diff --check` and inspect only the four intended DRA files.

- [ ] **Step 5: Request commit approval**

Keep both code and DRA changes uncommitted.
Present the diffs and ask the user for explicit approval before any commit or push.

---

## Final Verification

- [ ] Run all Python unit tests:

```bash
conda run -n cbf_env python -m unittest discover -s tests -p 'test_*diagnostic.py' -v
```

- [ ] Run all focused C++ tests:

```bash
./build-diagnostic/testDiagnosticConfiguration
./build-diagnostic/testRobotDiagnostics
./build-diagnostic/testPolygon
```

- [ ] Run `git diff --check` in both code and DRA worktrees.
- [ ] Verify the original `cbf-research-2026` checkout is clean and `stash@{0}` still exists.
- [ ] Verify available disk space exceeds 6 GB and report the exact value.
- [ ] Report generated raw-data sizes and retained paths.
- [ ] Do not claim success from exit codes alone; cite the diagnostic summaries.
- [ ] Do not commit or push without explicit user approval.
