# CBF2026 Fixed Localization Information-Set Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the CBF2026 covariance FIM use exactly the predeclared fixed localization DAG, verify the behavior test-first, and run a disk-guarded 20 s plus 250 s R mechanism gate.

**Architecture:** Extract one pure fixed-reference query in `Robot` and reuse it for both formation CBF construction and covariance construction. Remove range-based FIM membership without changing the \(J^\top\Sigma^{-1}J\) calculation, extend the analyzer with a durable information-set transition audit, then run the existing reproducible R configuration against new evidence roots.

**Tech Stack:** C++17, CMake 3.16+, doctest, Eigen, nlohmann/json, Gurobi 12, Python 3.11 in conda environment `cbf_env`, NumPy 1.24, Python `unittest`.

## Global Constraints

- Work only in `/private/tmp/cbf2026-diagnostic` on `codex/cbf2026-diagnostic`; do not touch `cbf-research-2026` or its stash.
- Use exactly the fixed base references and strict lower-squad-index UAV references already declared by the localization formation.
- Keep the ranging variance, propagated anchor covariance, Jacobian, \(\Phi_i=J_i^\top\Sigma_i^{-1}J_i\), and \(\hat\Sigma_i=\Phi_i^{-1}\) calculations unchanged.
- Do not add a configuration switch or retain optional range-gated FIM anchors.
- `comm-fixed.max-range=850 m` remains a localization CBF limit, but it no longer selects FIM members.
- Missing data for a declared fixed UAV reference is an explicit information-contract failure.
- Do not add lower-level dynamics, sampled-data reserve, uncertainty-jump reserve, input-limit changes, class-\(\mathcal K\) changes, or held-command timing changes.
- Use seed `20260727`, Gurobi, 0.5 s period, and the existing unbounded-input R configuration for mechanism runs.
- Use start-space `8,000,000,000` bytes, live floor `6,000,000,000` bytes, output-root cap `2,000,000,000` bytes, and per-run cap `250,000,000` bytes.
- Run Python commands in conda environment `cbf_env`.
- Make every production change test-first and record the observed RED failure before implementation.
- Preserve all existing `/private/tmp/cbf2026-results` evidence; do not overwrite or delete any bundle.
- Do not modify the paper, push a branch, or add `Co-Authored-By`.
- Stage only named files; never use `git add .`.

---

### Task 1: Fixed localization reference selection and unchanged FIM

**Files:**
- Modify: `tests/testRobustConstraintConstruction.cpp`
- Modify: `include/Robot.hpp`

**Interfaces:**
- Produces: `Robot::fixedLocalizationReferences() const -> json`.
- Changes: `Robot::getCovariance(json&)` to `Robot::getCovariance()`.
- Consumes: `myBasesId`, `myNeighboursId`, `idInMyPart`, `getIdInPart`, and `comm-fixed.min-neighbour-id-offset`.
- Preserves: `Robot::myFormation`, `Robot::myCovarianceFormation`, and the existing FIM numerical formula.

- [ ] **Step 1: Add a non-collinear four-UAV test fixture**

Add this fixture beside the existing diagnostic factories in
`tests/testRobustConstraintConstruction.cpp`:

```cpp
json makeFourRobotFixedGraphSettings() {
    json settings = makeThreeRobotChainSettings();
    settings["num"] = 4;
    settings["all"] = json::array({1, 2, 3, 4});
    settings["world"]["boundary"] = json::array({
        json::array({-2000.0, -2000.0}),
        json::array({2000.0, -2000.0}),
        json::array({2000.0, 2000.0}),
        json::array({-2000.0, 2000.0})
    });
    settings["initial"]["position"]["positions"] = json::array({
        json::array({0.0, 0.0}),
        json::array({2.0, 1.0}),
        json::array({4.0, 3.0}),
        json::array({6.0, 6.0})
    });
    settings["bases"] = json::array({
        json::array({-20.0, 0.0}),
        json::array({0.0, -20.0}),
        json::array({10.0, 10.0})
    });
    settings["formation"]["bases-id"] =
        json::array({json::array({0, 1, 2})});
    settings["cbfs"]["without-slack"]["comm-fixed"]["max-range"] = 100.0;
    return settings;
}

std::vector<std::unique_ptr<Robot>> makeFourFixedGraphRobots() {
    json settings = makeFourRobotFixedGraphSettings();
    std::vector<std::unique_ptr<Robot>> robots;
    for (int id = 1; id <= 4; ++id) {
        json robotSettings = settings;
        robots.emplace_back(std::make_unique<Robot>(id, robotSettings));
    }
    exchangeDiagnosticData(robots);
    return robots;
}
```

Also change the existing three-UAV bootstrap fixture's third position from
`{4.0, 2.0}` to `{4.0, 3.0}`.
This removes an accidental collinearity while leaving the bootstrap
self-consistency assertion unchanged.

- [ ] **Step 2: Write the failing fixed-membership tests**

Add these doctest cases:

```cpp
TEST_CASE("covariance FIM excludes in-range references outside the fixed localization graph") {
    auto robots = makeFourFixedGraphRobots();
    Robot& robot = *robots.at(3);

    robot.getCovariance();

    CHECK(robot.myCovarianceFormation.at("anchorIds")
          == json::array({2, 3}));
    CHECK(robot.myCovarianceFormation.at("baseIds")
          == json::array());
}

TEST_CASE("covariance FIM retains declared base references beyond max range") {
    json settings = makeDiagnosticSettings();
    settings["cbfs"]["without-slack"]["comm-fixed"]["max-range"] = 5.0;
    Robot robot(1, settings);

    CHECK_NOTHROW(robot.getCovariance());
    CHECK(robot.myCovarianceFormation.at("anchorIds")
          == json::array());
    CHECK(robot.myCovarianceFormation.at("baseIds")
          == json::array({0, 1}));
    CHECK(robot.positionCovariance.isApprox(
        Eigen::Matrix2d::Identity(), 1e-12
    ));
}

TEST_CASE("covariance information set is invariant across optional-anchor range crossing") {
    auto robots = makeFourFixedGraphRobots();
    Robot& robot = *robots.at(3);
    robot.getCovariance();
    const json before = robot.myCovarianceFormation;

    robots.at(0)->model->setPosition2D(Point(1500.0, 1500.0));
    exchangeDiagnosticData(robots);
    robot.getCovariance();

    CHECK(robot.myCovarianceFormation == before);
}
```

Update the existing startup bootstrap test to call parameterless
`getCovariance()`.

- [ ] **Step 3: Run the focused target and verify RED**

Run:

```bash
cmake --build build-diagnostic --target testRobustConstraintConstruction -j2
./build-diagnostic/testRobustConstraintConstruction
```

Expected: compilation fails because `getCovariance()` has no parameterless
overload.
Record the compiler diagnostic in the task notes before editing production
code.

- [ ] **Step 4: Implement the fixed-reference query**

Add this public const query before `getCovariance()` in `include/Robot.hpp`:

```cpp
json fixedLocalizationReferences() const {
    json references = {
        {"anchorIds", json::array()},
        {"baseIds", json::array()}
    };

    const auto commConfig =
        settings["cbfs"]["without-slack"]["comm-fixed"];
    const int minOffset =
        commConfig.value("min-neighbour-id-offset", -2);
    const int maximumBaseIndex =
        -idInMyPart - minOffset;

    for (std::size_t index = 0; index < myBasesId.size(); ++index) {
        if (static_cast<int>(index) > maximumBaseIndex) {
            continue;
        }
        references["baseIds"].push_back(myBasesId[index]);
    }

    for (int otherId : myNeighboursId) {
        if (!isSamePartAsMe(otherId)) {
            continue;
        }
        if (getIdInPart(otherId) >= idInMyPart) {
            continue;
        }
        references["anchorIds"].push_back(otherId);
    }
    return references;
}
```

This is the only source of truth for fixed reference IDs.

- [ ] **Step 5: Make covariance construction consume only fixed references**

Change `getCovariance(json &config)` to `getCovariance()`.
Replace the base and robot membership loops with:

```cpp
const json references = fixedLocalizationReferences();
myCovarianceFormation = {
    {"id", id},
    {"anchorIds", references.at("anchorIds")},
    {"baseIds", references.at("baseIds")}
};

for (const json& baseIdJson : references.at("baseIds")) {
    const int baseId = baseIdJson.get<int>();
    if (baseId < 0 || baseId >= static_cast<int>(bases.size())) {
        throw std::invalid_argument(
            "#" + std::to_string(id)
            + " has an invalid fixed base reference "
            + std::to_string(baseId)
        );
    }
    anchorPoints.push_back(bases.at(baseId));
    anchorCovariances.push_back(Eigen::Matrix2d::Zero());
}

for (const json& anchorIdJson : references.at("anchorIds")) {
    const int otherId = anchorIdJson.get<int>();
    const auto positionIt = comm->_othersPos.find(otherId);
    const auto covarianceIt =
        comm->_othersPositionCovariance.find(otherId);
    if (positionIt == comm->_othersPos.end()
        || covarianceIt == comm->_othersPositionCovariance.end()) {
        throw std::invalid_argument(
            "#" + std::to_string(id)
            + " is missing fixed localization data for #"
            + std::to_string(otherId)
        );
    }
    anchorPoints.push_back(positionIt->second);
    anchorCovariances.push_back(covarianceIt->second);
}
```

Delete the local `maxRange` and both distance gates.
Do not edit the angle, variance, `J`, `Sigma`, `Phi`, or inverse code below
these loops.
Update `updateCovarianceAndRate()` to call `getCovariance()`.

- [ ] **Step 6: Reuse the same reference query in formation construction**

Replace the independent ID-selection portion of `setupFormation()` with:

```cpp
const json references = fixedLocalizationReferences();
myFormation = {
    {"id", id},
    {"anchorPoints", json::array()},
    {"anchorIds", references.at("anchorIds")},
    {"baseIds", references.at("baseIds")}
};

for (const json& baseIdJson : references.at("baseIds")) {
    const int baseId = baseIdJson.get<int>();
    const Point& base = bases.at(baseId);
    myFormation["anchorPoints"].push_back({base.x, base.y});
}
```

Do not add range checks to this helper or to `setupFormation()`.

- [ ] **Step 7: Verify GREEN and the complete C++ regression set**

Run:

```bash
cmake --build build-diagnostic \
  --target testRobustConstraintConstruction testRobotDiagnostics \
  testDiagnosticConfiguration testSwarmFailureHandling Swarm -j2
./build-diagnostic/testRobustConstraintConstruction
./build-diagnostic/testRobotDiagnostics
./build-diagnostic/testDiagnosticConfiguration
./build-diagnostic/testSwarmFailureHandling
```

Expected: every test executable exits 0 and the new three cases pass.

- [ ] **Step 8: Commit Task 1**

```bash
git add -- include/Robot.hpp tests/testRobustConstraintConstruction.cpp
git commit -m "fix(cbf): freeze localization FIM references"
```

The commit must contain exactly these two files.

---

### Task 2: Durable information-set transition audit

**Files:**
- Modify: `scripts/diagnostics/analyze_diagnostic.py`
- Modify: `tests/test_analyze_diagnostic.py`

**Interfaces:**
- Produces: analysis object `covariance_information_set`.
- Produces fields: `status`, `robot_frame_record_count`,
  `transition_count`, `transitions`, `required_reference_mismatch_count`,
  `required_reference_mismatches`, and `malformed_record_count`.
- Consumes: each frame's `formation` and `covariance_formation` arrays.

- [ ] **Step 1: Write a failing analyzer test**

Add a test to `DiagnosticAnalyzerTests`:

```python
def test_covariance_information_set_transitions_and_reference_mismatches_are_reported(self):
    data = {
        "para": {"gridWorld": {"xNum": 1, "yNum": 1}},
        "config": {},
        "state": [
            {
                "runtime": 0.0,
                "formation": [
                    {"id": 1, "anchorIds": [], "baseIds": [0]},
                    {"id": 2, "anchorIds": [1], "baseIds": []},
                ],
                "covariance_formation": [
                    {"id": 1, "anchorIds": [], "baseIds": [0]},
                    {"id": 2, "anchorIds": [1], "baseIds": []},
                ],
                "update": [],
                "robots": [],
            },
            {
                "runtime": 0.5,
                "formation": [
                    {"id": 1, "anchorIds": [], "baseIds": [0]},
                    {"id": 2, "anchorIds": [1], "baseIds": []},
                ],
                "covariance_formation": [
                    {"id": 1, "anchorIds": [], "baseIds": [0]},
                    {"id": 2, "anchorIds": [1, 3], "baseIds": []},
                ],
                "update": [],
                "robots": [],
            },
        ],
    }

    summary = self.analyze_fixture(data)
    audit = summary["covariance_information_set"]
    self.assertEqual(audit["status"], "available")
    self.assertEqual(audit["robot_frame_record_count"], 4)
    self.assertEqual(audit["transition_count"], 1)
    self.assertEqual(audit["required_reference_mismatch_count"], 1)
    self.assertEqual(audit["transitions"][0]["robot_id"], 2)
    self.assertEqual(audit["transitions"][0]["frame_index"], 1)
```

- [ ] **Step 2: Run the analyzer test and verify RED**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_diagnostic.DiagnosticAnalyzerTests.test_covariance_information_set_transitions_and_reference_mismatches_are_reported -v
```

Expected: `KeyError: 'covariance_information_set'`.

- [ ] **Step 3: Add a strict reference-ID normalizer**

Add this helper near the existing JSON validation helpers:

```python
def _normalized_reference_ids(value: Any) -> tuple[tuple[int, ...], tuple[int, ...]] | None:
    if not isinstance(value, dict):
        return None
    anchor_ids = value.get("anchorIds")
    base_ids = value.get("baseIds")
    if not isinstance(anchor_ids, list) or not isinstance(base_ids, list):
        return None
    if not all(isinstance(item, int) and not isinstance(item, bool) for item in anchor_ids):
        return None
    if not all(isinstance(item, int) and not isinstance(item, bool) for item in base_ids):
        return None
    return tuple(sorted(anchor_ids)), tuple(sorted(base_ids))
```

- [ ] **Step 4: Collect transitions and required-reference mismatches**

Initialize these accumulators before the frame loop:

```python
previous_covariance_references: dict[
    int, tuple[tuple[int, ...], tuple[int, ...]]
] = {}
covariance_information_set_records = 0
covariance_information_set_transitions: list[dict] = []
covariance_reference_mismatches: list[dict] = []
covariance_information_set_malformed_count = 0
```

After `formation_by_id` is built for each frame, parse
`frame["covariance_formation"]`.
For every valid robot entry:

```python
normalized = _normalized_reference_ids(covariance_formation)
if normalized is None:
    covariance_information_set_malformed_count += 1
    continue

robot_id = covariance_formation["id"]
covariance_information_set_records += 1
previous = previous_covariance_references.get(robot_id)
if previous is not None and previous != normalized:
    covariance_information_set_transitions.append({
        "frame_index": frame_index,
        "runtime": runtime,
        "robot_id": robot_id,
        "previous": {
            "anchor_ids": list(previous[0]),
            "base_ids": list(previous[1]),
        },
        "current": {
            "anchor_ids": list(normalized[0]),
            "base_ids": list(normalized[1]),
        },
    })

required = _normalized_reference_ids(formation_by_id.get(robot_id))
if required is None or required != normalized:
    covariance_reference_mismatches.append({
        "frame_index": frame_index,
        "runtime": runtime,
        "robot_id": robot_id,
        "required": None if required is None else {
            "anchor_ids": list(required[0]),
            "base_ids": list(required[1]),
        },
        "covariance": {
            "anchor_ids": list(normalized[0]),
            "base_ids": list(normalized[1]),
        },
    })
previous_covariance_references[robot_id] = normalized
```

Treat a missing/non-list `covariance_formation` as unavailable evidence; do
not invent zero transitions for absent logs.

- [ ] **Step 5: Add the audit to the JSON and Markdown summaries**

Add:

```python
"covariance_information_set": {
    "status": (
        "available"
        if covariance_information_set_records
        else "unavailable_no_covariance_formation_records"
    ),
    "robot_frame_record_count": covariance_information_set_records,
    "transition_count": len(covariance_information_set_transitions),
    "transitions": covariance_information_set_transitions,
    "required_reference_mismatch_count": len(covariance_reference_mismatches),
    "required_reference_mismatches": covariance_reference_mismatches,
    "malformed_record_count": covariance_information_set_malformed_count,
},
```

Extend `_write_markdown()` with one compact line reporting status,
transition count, mismatch count, and malformed count.
Do not copy full transition arrays into Markdown.

- [ ] **Step 6: Verify GREEN and complete Python regression**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_diagnostic.DiagnosticAnalyzerTests.test_covariance_information_set_transitions_and_reference_mismatches_are_reported -v
conda run -n cbf_env python -m unittest \
  tests.test_analyze_diagnostic tests.test_run_diagnostic \
  tests.test_swarm_failure_exit tests.test_cmake_configuration -v
```

Expected: all tests pass with no error or failure.

- [ ] **Step 7: Reanalyze the preserved dynamic R bundle**

Run:

```bash
conda run -n cbf_env python -m scripts.diagnostics.analyze_diagnostic \
  /private/tmp/cbf2026-diagnostic/build-diagnostic/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/data.json \
  /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json
```

Expected: the regenerated summary reports exactly 68 robot-frame
information-set transitions.
Do not overwrite the preserved bundle's existing `analysis.json`; the
analyzer writes beside the temporary raw-data link under
`build-diagnostic/`.

- [ ] **Step 8: Commit Task 2**

```bash
git add -- scripts/diagnostics/analyze_diagnostic.py \
  tests/test_analyze_diagnostic.py
git commit -m "feat(diagnostics): audit covariance information sets"
```

The commit must contain exactly these two files.

---

### Task 3: Full implementation verification

**Files:**
- Verify only; no planned source modification.

**Interfaces:**
- Consumes: Task 1 fixed-reference implementation.
- Consumes: Task 2 `covariance_information_set` analyzer block.
- Produces: a clean committed source state and a fresh `Swarm` binary.

- [ ] **Step 1: Reconfigure and build all diagnostic targets**

Run:

```bash
cmake -S . -B build-diagnostic -DCMAKE_BUILD_TYPE=Release
cmake --build build-diagnostic -j2
```

Expected: configuration and build both exit 0.

- [ ] **Step 2: Run complete CTest**

Run:

```bash
ctest --test-dir build-diagnostic --output-on-failure
```

Expected: every discovered CTest exits successfully.

- [ ] **Step 3: Run the explicit focused suites**

Run:

```bash
./build-diagnostic/testRobustConstraintConstruction
./build-diagnostic/testRobotDiagnostics
./build-diagnostic/testDiagnosticConfiguration
./build-diagnostic/testSwarmFailureHandling
conda run -n cbf_env python -m unittest \
  tests.test_analyze_diagnostic tests.test_run_diagnostic \
  tests.test_swarm_failure_exit tests.test_cmake_configuration -v
```

Expected: all doctest and unittest cases pass.

- [ ] **Step 4: Verify scope and source state**

Run:

```bash
git diff --check
git status --short --branch
git log -3 --oneline
shasum -a 256 build-diagnostic/Swarm
df -Pk /private/tmp
du -sk /private/tmp/cbf2026-results
```

Expected:

- no tracked modifications;
- only intentionally untracked `build-diagnostic/`;
- the latest two source commits are the Task 1 and Task 2 commits;
- at least 8,000,000,000 bytes are free before experiment launch;
- retained results remain below the 2 GB cache cap.

---

### Task 4: Disk-guarded fixed-information-set mechanism gate

**Files:**
- Create temporary evidence under:
  `/private/tmp/cbf2026-results/mc-first-fixed-information-set-smoke`
- Create temporary evidence under:
  `/private/tmp/cbf2026-results/mc-first-fixed-information-set-250s`

**Interfaces:**
- Consumes: `build-diagnostic/Swarm`.
- Consumes: `config/diagnostics/r_rate_aware.json`.
- Produces: one 20 s R bundle and, only if smoke passes, one 250 s R bundle.

- [ ] **Step 1: Record immutable preflight anchors**

Run:

```bash
git rev-parse HEAD
git status --short --branch
shasum -a 256 build-diagnostic/Swarm
df -Pk /private/tmp
du -sk /private/tmp/cbf2026-results
```

Do not launch unless the source is committed, only `build-diagnostic/` is
untracked, free bytes are at least 8,000,000,000, and the results root is
below 2,000,000,000 bytes.

- [ ] **Step 2: Run the 20 s R smoke**

Run:

```bash
conda run -n cbf_env python -m scripts.diagnostics.run_diagnostic \
  --case R \
  --horizon 20 \
  --seed 20260727 \
  --binary build-diagnostic/Swarm \
  --output-root /private/tmp/cbf2026-results/mc-first-fixed-information-set-smoke
```

Expected: runner exits 0 and creates exactly one terminal bundle.

- [ ] **Step 3: Enforce the smoke gate**

Run:

```bash
conda run -n cbf_env python -c '
import json
from pathlib import Path
root = Path("/private/tmp/cbf2026-results/mc-first-fixed-information-set-smoke/R")
bundles = sorted(path for path in root.iterdir() if path.is_dir())
assert len(bundles) == 1, bundles
analysis = json.loads((bundles[0] / "analysis.json").read_text())
manifest = json.loads((bundles[0] / "manifest.json").read_text())
audit = analysis["covariance_information_set"]
assert manifest["termination_reason"] == "completed", manifest
assert analysis["frame_count"] == 40, analysis["frame_count"]
assert analysis["applied_solver_records"]["count"] == 560
assert analysis["unapplied_solver_records"]["count"] == 0
assert analysis["finite_value_failures"]["count"] == 0
assert analysis["hard_constraints"]["negative_count"] == 0
assert audit["status"] == "available"
assert audit["transition_count"] == 0
assert audit["required_reference_mismatch_count"] == 0
assert audit["malformed_record_count"] == 0
assert analysis["practical_tolerance"]["localization"]["below_negative_tolerance_count"] == 0
assert analysis["practical_tolerance"]["collision"]["below_negative_tolerance_count"] == 0
print(bundles[0])
'
```

If any assertion fails, stop and preserve the bundle.
Do not launch 250 s.

- [ ] **Step 4: Recheck disk before the 250 s run**

Run:

```bash
df -Pk /private/tmp
du -sk /private/tmp/cbf2026-results \
  /private/tmp/cbf2026-results/mc-first-fixed-information-set-smoke
```

Require at least 8,000,000,000 free bytes and keep the cache below 2 GB.

- [ ] **Step 5: Run the 250 s R gate**

Run:

```bash
conda run -n cbf_env python -m scripts.diagnostics.run_diagnostic \
  --case R \
  --horizon 250 \
  --seed 20260727 \
  --binary build-diagnostic/Swarm \
  --output-root /private/tmp/cbf2026-results/mc-first-fixed-information-set-250s
```

Expected: runner exits 0 and creates exactly one terminal bundle.

- [ ] **Step 6: Audit technical integrity and the mechanism gate**

Run:

```bash
conda run -n cbf_env python -c '
import json
from pathlib import Path
root = Path("/private/tmp/cbf2026-results/mc-first-fixed-information-set-250s/R")
bundles = sorted(path for path in root.iterdir() if path.is_dir())
assert len(bundles) == 1, bundles
analysis = json.loads((bundles[0] / "analysis.json").read_text())
manifest = json.loads((bundles[0] / "manifest.json").read_text())
audit = analysis["covariance_information_set"]
assert manifest["termination_reason"] == "completed", manifest
assert analysis["frame_count"] == 500
assert analysis["applied_solver_records"]["count"] == 7000
assert analysis["unapplied_solver_records"]["count"] == 0
assert analysis["finite_value_failures"]["count"] == 0
assert analysis["hard_constraints"]["count"] == 21000
assert analysis["hard_constraints"]["negative_count"] == 0
assert audit["status"] == "available"
assert audit["robot_frame_record_count"] == 7000
assert audit["transition_count"] == 0
assert audit["required_reference_mismatch_count"] == 0
assert audit["malformed_record_count"] == 0
print(json.dumps({
    "bundle": str(bundles[0]),
    "localization": analysis["practical_tolerance"]["localization"],
    "collision": analysis["practical_tolerance"]["collision"],
    "uncertainty": analysis["uncertainty"],
    "coverage": analysis["coverage"],
    "minimum_linf_bound": analysis["minimum_linf_bound"],
}, indent=2))
'
```

Technical integrity requires all assertions above.
The mechanism gate passes only when localization and collision both have
`below_negative_tolerance_count == 0`, no consecutive negative frames, and
no other mandatory failure.
If localization fails, stop before any bounded, pairwise, 500 s, or
multi-geometry run.

- [ ] **Step 7: Compare against the preserved dynamic R evidence**

Run a read-only comparison of:

```text
/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/analysis.json
the `analysis.json` inside the sole directory discovered under
`/private/tmp/cbf2026-results/mc-first-fixed-information-set-250s/R`
```

Use a Python script that discovers the sole generated fixed bundle and prints:

- tightened localization and collision minima;
- observations below `-0.5 m`;
- uncertainty maximum, maximum positive rate, and maximum jump;
- information-set transition and mismatch counts;
- minimum-required and actual \(L_\infty\) maxima;
- applied solve count, hard residual minimum, coverage, and solve time.

This comparison is descriptive.
Do not claim causal closure if the fixed run still fails.

- [ ] **Step 8: Record final disk and hashes**

Run:

```bash
df -Pk /private/tmp
du -sk /private/tmp/cbf2026-results \
  /private/tmp/cbf2026-results/mc-first-fixed-information-set-smoke \
  /private/tmp/cbf2026-results/mc-first-fixed-information-set-250s
```

For each generated bundle, record SHA-256 for:

- `manifest.json`;
- `config.materialized.json`;
- `analysis.json`;
- the raw `data.json` at
  `/private/tmp/cbf2026-diagnostic/build-diagnostic/` plus the generated
  bundle directory name; and
- `source-snapshot.tar.gz`.

Do not delete any raw, sidecar, superseded, or prior mechanism data.

---

### Task 5: Independent audit, durable report, and DRA update

**Files:**
- Create: `docs/diagnostics/2026-07-28-mc-first-fixed-information-set-gate.md`
- Modify: `/Users/xirhxq/Documents/Clones/doctoral-research-agent/papers/cbf2026/status.md`
- Modify: `/Users/xirhxq/Documents/Clones/doctoral-research-agent/papers/cbf2026/open-questions.md`
- Modify: `/Users/xirhxq/Documents/Clones/doctoral-research-agent/papers/cbf2026/sources.md`
- Modify: `/Users/xirhxq/Documents/Clones/doctoral-research-agent/papers/cbf2026/timeline.md`
- Create: `/Users/xirhxq/Documents/Clones/doctoral-research-agent/meta-log/2026-07-28-cbf2026-fixed-information-set-gate.md`

**Interfaces:**
- Consumes: Task 4 manifests, hashes, analyses, and disk probes.
- Produces: one tracked CBF diagnostic report and one isolated DRA commit.

- [ ] **Step 1: Dispatch two read-only subagent audits**

Give one subagent the raw-to-summary integrity audit:

- recompute frames, solver counts, hard residuals, margins, information-set
  transitions, and covariance uncertainty consistency;
- confirm the old bundles were not modified;
- report Critical/Important/Minor findings only.

Give the second subagent the mechanism interpretation:

- compare dynamic R with fixed R;
- verify whether optional-anchor transitions disappeared;
- distinguish improvement, regression, or unresolved failure;
- recommend whether held-command timing is now the next smallest mechanism.

Neither subagent may edit, build, run simulation, commit, push, or delete.

- [ ] **Step 2: Write the compact tracked report**

The report must contain:

- source commit and binary hash;
- exact bundle paths and all evidence hashes;
- normalized configuration comparison;
- test/build commands and results;
- 20 s and 250 s gate decisions;
- dynamic-versus-fixed metric table;
- information-set transition and required-reference mismatch counts;
- disk start/end probes and retained sizes;
- limitations: single deterministic mechanism run, no bounded-input proof,
  estimator calibration, multi-geometry MC, or manuscript claim;
- explicit next-step decision.

Do not paste raw arrays.

- [ ] **Step 3: Review and commit the CBF report**

Run:

```bash
git diff --check -- \
  docs/diagnostics/2026-07-28-mc-first-fixed-information-set-gate.md
git status --short --branch
```

Require no tracked source changes and only the report plus
`build-diagnostic/`.
Then:

```bash
git add -- docs/diagnostics/2026-07-28-mc-first-fixed-information-set-gate.md
git commit -m "docs(diagnostics): record fixed information-set gate"
```

- [ ] **Step 4: Update DRA without touching other-paper changes**

Record:

- the fixed-information-set decision and formula-preservation boundary;
- focused test and analyzer evidence;
- smoke/250 s result and stop decision;
- zero or nonzero transition/mismatch evidence;
- whether held-command timing becomes the next mechanism;
- exact commits, hashes, bundle paths, and disk state.

Append one row to `timeline.md`; do not rewrite prior events.
Do not modify `submissions.md`, because no submission event occurred.

- [ ] **Step 5: Validate and commit only five DRA files**

Run:

```bash
git diff --check -- \
  papers/cbf2026/status.md \
  papers/cbf2026/open-questions.md \
  papers/cbf2026/sources.md \
  papers/cbf2026/timeline.md \
  meta-log/2026-07-28-cbf2026-fixed-information-set-gate.md
git add -- \
  papers/cbf2026/status.md \
  papers/cbf2026/open-questions.md \
  papers/cbf2026/sources.md \
  papers/cbf2026/timeline.md \
  meta-log/2026-07-28-cbf2026-fixed-information-set-gate.md
git diff --cached --name-status
git commit -m "docs(cbf2026): record fixed information-set gate"
```

The staged list must contain exactly these five CBF2026 files.
Existing PodSearch2026 modifications remain unstaged.

- [ ] **Step 6: Final verification**

In the CBF worktree, run:

```bash
git status --short --branch
git log -4 --oneline
```

Expected: only `build-diagnostic/` is untracked and there is no upstream.

In DRA, run:

```bash
git show --stat --oneline --summary HEAD
git status --short --branch
```

Expected: the new commit contains exactly five CBF2026 files and all
pre-existing other-paper changes remain unstaged.
Report both commit IDs, final free space, retained evidence size, and the
next decision.
