# CBF2026 Corrected WNLS Calibration Run Registration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to execute this registration sequentially. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Execute one preregistered calibration with the corrected variable-weight WNLS objective, preserve complete audit evidence, and prevent any result-dependent tuning or claim change.

**Architecture:** Reuse the preserved truth trajectory as an immutable input and run the offline replay at frozen code commit `f6e995d6dc4711f0d2869636beb70f061e73063a`. Two identical 40-frame Stage 1 runs must match in deterministic content before exactly one 500-frame, 20-seed Stage 2 run is allowed. All bundles remain outside Git and are accepted only after independent hash, row-completeness, stationarity, pairing, JSON, and adequacy audits.

**Tech Stack:** Python 3.11 in conda environment `cbf_env`, NumPy 1.24.4, gzip JSON Lines, SHA-256, standard-library JSON.

## Global Constraints

- This document registers execution only. Do not modify code, tests, existing evidence, the paper, DRA, or `build-diagnostic`.
- Do not stage or commit during the registered run.
- The independent final mathematical re-review returned `CLEAN/READY` for commit `f6e995d6dc4711f0d2869636beb70f061e73063a`; this exact source is final for the registered run.
- Freeze estimator contract `variable_weight_nls_full_residual_jacobian_v1`.
- Freeze graph cases, in order, as `dynamic_dag_wnls` and `fixed_refs_wnls`.
- Freeze `ranging_sigma=0.5`, seeds `20260727` through `20260746`, and 500 frames for Stage 2.
- Freeze `MAX_ITERATIONS=50`, `INITIAL_DAMPING=0.001`, `STEP_TOLERANCE=1e-9`, `COST_TOLERANCE=1e-12`, `RANGE_EPSILON=1e-12`, and `RELATIVE_SPECTRAL_THRESHOLD=1e-12`.
- Freeze the seed-level bootstrap at 10,000 resamples with RNG seed `20260728`.
- Compute covariance from the localization FIM, not the estimator Gauss--Newton matrix, and compute `epsilon` exactly once as `3*sqrt(lambda_max(P))`.
- Retain frame zero in the process stream and report it separately; exclude it from primary adequacy statistics.
- Use stable-keyed common-edge noise so every shared edge has identical noise in both graph cases for the same seed, frame, observer, reference kind, and reference ID.
- Do not tune, retry, inflate epsilon, change graph membership, alter noise, or change the estimator after observing either stage.
- Require at least `8_000_000_000` free bytes before every registered invocation, enforce a live floor of `6_000_000_000` bytes, keep the complete corrected output root below `2_000_000_000` allocated bytes, and keep every bundle below `250_000_000` allocated bytes.
- Never delete, overwrite, move, or relabel any old or partial bundle. A controlled non-success bundle is evidence and terminates this registration.
- The corrected output root is exactly `/private/tmp/cbf2026-localization-calibration-corrected`. It must not exist before Stage 1 A; if it already exists, stop without deleting it.
- All Python commands use `conda run -n cbf_env python ...`.

---

## Registration Freeze

### Source identity and review gate

The executable source is frozen at:

```text
commit: f6e995d6dc4711f0d2869636beb70f061e73063a
scripts/diagnostics/replay_localization_calibration.py SHA-256:
9af743f9445813e5b2f89f1989963319929d3f16b52b244563e2dca4fa52a90b
implementation identity: cbf2026-localization-calibration-v2
estimator contract: variable_weight_nls_full_residual_jacobian_v1
```

The independent final mathematical re-review returned `CLEAN/READY` on
2026-07-29 for this exact commit. It confirmed the complete residual Jacobian,
the `J_q.T @ q` stationarity gate, final-iteration convergence, and separation
of `J_q.T @ J_q` from the final localization FIM. The executor must cite the
durable review artifact in the corrected evidence report. Any source change
after this verdict invalidates this registration and requires a new review and
new registration before execution.

### Immutable inputs

| Input | Exact path | Size (bytes) | SHA-256 |
| --- | --- | ---: | --- |
| Truth trajectory | `/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json` | 16,237,150 | `3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527` |
| Input manifest | `/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json` | 1,226 | `6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb` |

The input manifest records source commit
`cff3852831f969ad4bedf7d9b2f43b4f2a0cf42f` and materialized-configuration
SHA-256
`cb13a87615da21e2f52ab41d039f41c4547e8d7311f1f2b791afe8474d10153d`.
Do not copy either input into a corrected bundle.

### Frozen graph cases

`fixed_refs_wnls` uses only these references:

| UAV | Fixed references |
| ---: | --- |
| 1 | B0, B1 |
| 2 | B1, U1 |
| 3 | U1, U2 |
| 4 | U2, U3 |
| 5 | U3, U4 |
| 6 | U4, U5 |
| 7 | U5, U6 |
| 8 | B1, B2 |
| 9 | B1, U8 |
| 10 | U8, U9 |
| 11 | U9, U10 |
| 12 | U10, U11 |
| 13 | U11, U12 |
| 14 | U12, U13 |

`dynamic_dag_wnls` uses those fixed references plus every visible base and
every visible strictly lower squad-local UAV. Both cases use identical truth
geometry, initialization, optimizer constants, FIM equations, and
stable-keyed noise on common edges. This remains a reference-graph ablation,
not an estimator comparison.

### Preserved but excluded old Stage 2

The old Stage 2 remains at:

```text
/private/tmp/cbf2026-localization-calibration/stage2/localization-calibration/20260728T164130.657823Z_6832cfbd6592469085adb88b5a165f59
```

Its preserved hashes are:

| Artifact | SHA-256 |
| --- | --- |
| `calibration.jsonl.gz` | `34a6045d4ac5d23f72ac6d049d4d98a8f927ac9835b71f3a3050a00f8efaf524` |
| decompressed JSONL | `6e1d6ea42e3c65117d62a734f8771491e1ec4181ef5778503bb9ca65a2bd54c7` |
| `summary.json` | `0dddcc119e00abbcdb7dfad6491f39bde6bbf4107fee81a09a556ecf53a1c2b6` |
| `summary.md` | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |
| `manifest.json` | `74f37cb4c4ea7e8e49eef660eade7d047fa137dfc27f1a36b4c675c4b364fa89` |

This bundle is reproducible evidence of the prior implementation, but it is
excluded from objective-correct estimator adequacy because that implementation
omitted the position-dependent weight derivative.

## Registered Adequacy Gate

Stage 2 passes only if all five preregistered conditions are true:

```text
dynamic aggregate primary containment >= 0.98
every dynamic squad-local-depth primary containment >= 0.95
processed rows == expected rows, with zero silently discarded failures
dynamic invalid-attempt rate <= fixed invalid-attempt rate
dynamic failed-attempt rate <= fixed failed-attempt rate
```

No threshold may be altered after observing results. Actual measurements,
including a failed adequacy result, must be reported.

---

### Task 1: Verify the Frozen Source, Inputs, Output Root, and Disk

**Files:**
- Read only: `scripts/diagnostics/replay_localization_calibration.py`
- Read only: the two immutable inputs above
- Inspect only: `/private/tmp/cbf2026-localization-calibration-corrected`

**Interfaces:**
- Consumes: the approved final math re-review for exact commit `f6e995d6dc4711f0d2869636beb70f061e73063a`.
- Produces: a written preflight record containing source, input, output-root, and free-space checks.

- [ ] **Step 1: Verify the independent math approval**

Verify that the durable review artifact records `CLEAN/READY` for the exact
frozen commit. Record its path and SHA-256 in the later corrected evidence
report. Stop if the artifact identifies another source or qualifies the
verdict.

- [ ] **Step 2: Verify source identity**

Run:

```bash
git rev-parse HEAD
git diff --exit-code f6e995d6dc4711f0d2869636beb70f061e73063a -- \
  scripts/diagnostics/replay_localization_calibration.py \
  scripts/diagnostics/run_diagnostic.py
shasum -a 256 scripts/diagnostics/replay_localization_calibration.py
```

Require the exact full commit and script hash registered above. Documentation
changes and the pre-existing untracked `build-diagnostic/` do not alter the
frozen executable, but any executable-source difference is a hard stop.

- [ ] **Step 3: Verify the inputs and the unused corrected root**

Run:

```bash
stat -f '%N|%z' \
  /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json
shasum -a 256 \
  /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json
test ! -e /private/tmp/cbf2026-localization-calibration-corrected
```

Any path, size, or hash mismatch is a hard stop. If the corrected root exists,
preserve it and stop; do not reuse or delete it.

- [ ] **Step 4: Verify the start-space gate**

Run:

```bash
conda run -n cbf_env python -c \
  'import shutil; print(shutil.disk_usage("/private/tmp").free)'
```

Require at least `8_000_000_000` free bytes immediately before Stage 1 A.

---

### Task 2: Execute and Audit Stage 1 A

**Files:**
- Create only through the runner: `/private/tmp/cbf2026-localization-calibration-corrected/stage1/localization-calibration/<allocated-run-id>/`

**Interfaces:**
- Produces: exactly one seed `20260727`, 40 frames, 1,120 process rows.
- Blocks: Stage 1 B and Stage 2 on any execution or audit failure.

- [ ] **Step 1: Run Stage 1 A exactly once**

Run exactly:

```bash
conda run -n cbf_env python -m scripts.diagnostics.replay_localization_calibration \
  --data /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  --manifest /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json \
  --output-root /private/tmp/cbf2026-localization-calibration-corrected/stage1 \
  --seed 20260727 \
  --max-frames 40
```

Do not rerun on a nonzero exit, non-`completed` manifest, interruption, or
partial bundle. Preserve the bundle and stop this registration.

- [ ] **Step 2: Audit Stage 1 A before continuing**

Require all of the following:

- terminal manifest state is exactly `completed`;
- manifest top-level, manifest `settings`, summary top-level, and summary
  `settings` estimator contracts are exactly
  `variable_weight_nls_full_residual_jacobian_v1`;
- manifest settings contain exactly one seed `20260727`, both registered graph
  cases, `max_frames=40`, `effective_frame_count=40`, and all frozen constants;
- exactly 1,120 rows exist: 28 frame-zero rows and 1,092 primary rows;
- every expected `(seed, graph_case, frame_index, robot_id)` key appears
  exactly once;
- every converged attempt has a finite
  `attempt_stationarity_norm <= 1e-9`;
- every failed or invalid attempt remains in its process row, and every stale
  retained state preserves its current nested failure;
- every common-edge noise value and noisy range matches across graph cases;
- every JSON file and decompressed JSONL row parses as strict JSON, with no
  `NaN`, `Infinity`, or `-Infinity`; and
- compressed JSONL, independently decompressed JSONL, `summary.json`,
  `summary.md`, and `manifest.json` SHA-256 values are independently computed
  and recorded. Recomputed artifact hashes must match hashes stored in the
  manifest where such fields exist.

- [ ] **Step 3: Recheck disk gates**

Run:

```bash
conda run -n cbf_env python -c \
  'import shutil; print(shutil.disk_usage("/private/tmp").free)'
conda run -n cbf_env python -c \
  'from pathlib import Path; from scripts.diagnostics.run_diagnostic import allocated_bytes; print(allocated_bytes(Path("/private/tmp/cbf2026-localization-calibration-corrected")))'
```

Require free bytes at least `8_000_000_000` before Stage 1 B, the complete
corrected root below `2_000_000_000` allocated bytes, and the Stage 1 A bundle
below `250_000_000` allocated bytes.

---

### Task 3: Execute Stage 1 B and Require Deterministic Content Identity

**Files:**
- Create only through the runner: a second collision-resistant bundle under `/private/tmp/cbf2026-localization-calibration-corrected/stage1/localization-calibration/`

**Interfaces:**
- Consumes: audited Stage 1 A.
- Produces: exactly one second seed `20260727`, 40-frame bundle.
- Blocks: Stage 2 unless both bundles pass individually and match.

- [ ] **Step 1: Run the identical Stage 1 command exactly once more**

Run exactly the Stage 1 A command from Task 2, without changing any argument.
Do not retry a failure or interruption.

- [ ] **Step 2: Audit Stage 1 B independently**

Apply every Stage 1 A audit gate to Stage 1 B before comparing the bundles.

- [ ] **Step 3: Compare deterministic content**

Require exact equality between Stage 1 A and Stage 1 B for:

- compressed `calibration.jsonl.gz` SHA-256;
- decompressed JSONL SHA-256;
- `summary.json` SHA-256;
- `summary.md` SHA-256;
- process-row count and complete expected-key set;
- terminal completion state, estimator contract, and complete settings block.

Manifest file hashes are recorded but are not expected to match because run
IDs, paths, and timestamps differ. Any deterministic-content mismatch is a
hard stop before Stage 2; preserve both bundles.

- [ ] **Step 4: Recheck Stage 2 launch gates**

Repeat the free-space and allocation commands from Task 2. Require at least
`8_000_000_000` free bytes, corrected root allocation below
`2_000_000_000` bytes, and both Stage 1 bundles individually below
`250_000_000` bytes.

---

### Task 4: Execute Exactly One Stage 2 and Audit It

**Files:**
- Create only through the runner: `/private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/<allocated-run-id>/`

**Interfaces:**
- Consumes: two individually valid, deterministic-content-identical Stage 1 bundles.
- Produces: exactly one 20-seed, 500-frame bundle with 280,000 process rows.

- [ ] **Step 1: Run Stage 2 exactly once**

Run exactly:

```bash
conda run -n cbf_env python -m scripts.diagnostics.replay_localization_calibration \
  --data /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  --manifest /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json \
  --output-root /private/tmp/cbf2026-localization-calibration-corrected/stage2 \
  --seed 20260727 --seed 20260728 --seed 20260729 --seed 20260730 \
  --seed 20260731 --seed 20260732 --seed 20260733 --seed 20260734 \
  --seed 20260735 --seed 20260736 --seed 20260737 --seed 20260738 \
  --seed 20260739 --seed 20260740 --seed 20260741 --seed 20260742 \
  --seed 20260743 --seed 20260744 --seed 20260745 --seed 20260746 \
  --max-frames 500
```

This is the only permitted Stage 2 invocation. On nonzero exit,
non-`completed` manifest, interruption, disk stop, or audit failure, preserve
the partial or terminal bundle, record the failure, and do not rerun.

- [ ] **Step 2: Enforce live disk limits**

The runner checks after every frame batch and must terminate before crossing
the `6_000_000_000`-byte live floor, `2_000_000_000`-byte stage output cap, or
`250_000_000`-byte bundle cap. Read-only bounded probes may monitor the
complete corrected root during execution. Do not delete evidence to regain
space.

- [ ] **Step 3: Audit Stage 2**

Require all Stage 1 per-bundle audit gates, with these exact Stage 2 counts:

```text
total rows: 280,000
frame-zero rows reported separately: 560
primary rows: 279,440
rows per graph case: 140,000 total and 139,720 primary
```

Also require:

- seeds are exactly `20260727` through `20260746`, in order;
- `max_frames=500` and `effective_frame_count=500`;
- all attempt and retained status categories reconcile to total rows;
- adequacy booleans equal direct recomputation from process rows;
- the seed bootstrap uses exactly 10,000 seed-level resamples with RNG seed
  `20260728`, never robot-frame resampling;
- the frame-zero summary is distinct from primary containment;
- every shared measurement passes the paired common-edge-noise audit; and
- no process failure or invalid attempt is silently dropped.

- [ ] **Step 4: Record final disk and artifact integrity**

Require the complete corrected root below `2_000_000_000` allocated bytes,
the Stage 2 bundle below `250_000_000` allocated bytes, and at least
`6_000_000_000` free bytes. Independently record compressed, decompressed,
both summary, and manifest hashes for all three corrected bundles.

---

### Task 5: Report, Review, and Preserve the Claim Boundary

**Files:**
- Create after execution: `docs/diagnostics/2026-07-29-corrected-wnls-localization-calibration.md`
- Modify after execution: `docs/diagnostics/README.md`
- Create independent review: `.superpowers/sdd/2026-07-28-cbf2026-dynamic-localization-calibration-implementation/task-7-evidence-review.md`

**Interfaces:**
- Consumes: bundle files only, not terminal recollection.
- Produces: a corrected evidence report and an independent evidence-review verdict.
- Blocks: all paper and DRA updates until both artifacts are complete.

- [ ] **Step 1: Write a new corrected evidence report**

Do not overwrite or revise
`docs/diagnostics/2026-07-28-dynamic-localization-calibration.md`.
The new report must record the math-review provenance, frozen source and input
identities, exact commands, disk probes, all three bundle paths/sizes/hashes,
Stage 1 content comparison, complete audit results, graph memberships,
attempt/retained status counts, primary and frame-zero containment, seed-level
confidence intervals, per-depth values, FIM/condition/epsilon quantiles,
transition statistics, paired changes, and the exact adequacy decision.

State that 20 ranging-noise seeds reuse one preserved trajectory geometry,
estimates are offline and never enter the controller, fixed references form a
graph ablation, the FIM ignores shared-ancestor cross-correlations, and no
mission-level probability or closed-loop safety guarantee follows.

- [ ] **Step 2: Obtain an independent evidence review**

The reviewer must recompute hashes, row counts, expected-key completeness,
strict-JSON validity, converged-attempt stationarity, common-edge pairing,
summary aggregates, bootstrap configuration, disk caps, and every adequacy
boolean from the preserved bundles. The review must state PASS or FAIL and
identify the exact bundle paths and report SHA-256.

- [ ] **Step 3: Apply the claim gate**

Do not update the paper or DRA, and do not upgrade any claim, before the
corrected report and independent evidence review are complete. A failed
adequacy gate must be reported as a calibration gap and cannot authorize an
epsilon validation claim. A passed gate may support only the carefully bounded
empirical claim justified by the reviewed report.

- [ ] **Step 4: Hand off paper and DRA updates**

Only after independent evidence-review PASS may a separate, newly scoped task
update the paper and DRA from the reviewed evidence files. That task must
preserve the one-trajectory, offline-estimator, graph-ablation,
cross-correlation, and non-closed-loop limitations.
