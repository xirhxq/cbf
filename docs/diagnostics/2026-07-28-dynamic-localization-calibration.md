# Dynamic localization DAG calibration evidence

## Status

The registered Stage 1 smoke/repeat and the single registered Stage 2 paired
calibration completed on implementation commit
`c88c340745bc41d9b6364cd12af727936aafd992`.
The Stage 1 deterministic-content check passed.
The Stage 2 adequacy gate did not pass, so these results are recorded as
mechanism evidence only and do not authorize a manuscript upgrade.
No estimator, graph rule, noise setting, or radius was changed after observing
the result, and Stage 2 was not retried.

## Registered inputs and execution

The replay consumed this preserved truth trajectory, without copying it into
the evidence bundle:

```text
/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json
```

Its SHA-256 is
`3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527`,
matching the preregistered value.
The accompanying input manifest SHA-256 is
`6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb`;
it records source commit `cff3852831f969ad4bedf7d9b2f43b4f2a0cf42f` and
materialized-configuration SHA-256
`cb13a87615da21e2f52ab41d039f41c4547e8d7311f1f2b791afe8474d10153d`.
The implementation identity recorded by every output manifest is
`cbf2026-localization-calibration-v2`, with implementation SHA-256
`218ba696c83a437fcf51f812f1b21156bc0baeab96dfde4b15a9bd31b1d9e5a8`.

Before the first registered run, `/private/tmp` had 9,002,622,976 free bytes.
The evidence root was new:
`/private/tmp/cbf2026-localization-calibration`.
The two Stage 1 invocations used the following registered command, exactly
once each:

```bash
conda run -n cbf_env python -m scripts.diagnostics.replay_localization_calibration \
  --data /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  --manifest /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json \
  --output-root /private/tmp/cbf2026-localization-calibration/stage1 \
  --seed 20260727 --max-frames 40
```

Stage 2 used this registered command exactly once:

```bash
conda run -n cbf_env python -m scripts.diagnostics.replay_localization_calibration \
  --data /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  --manifest /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json \
  --output-root /private/tmp/cbf2026-localization-calibration/stage2 \
  --seed 20260727 --seed 20260728 --seed 20260729 --seed 20260730 \
  --seed 20260731 --seed 20260732 --seed 20260733 --seed 20260734 \
  --seed 20260735 --seed 20260736 --seed 20260737 --seed 20260738 \
  --seed 20260739 --seed 20260740 --seed 20260741 --seed 20260742 \
  --seed 20260743 --seed 20260744 --seed 20260745 --seed 20260746
```

The runner used the materialized `ranging_sigma=0.5`, WNLS/LM maximum of 50
iterations and initial damping `0.001`, and the registered 10,000-resample,
seed-level bootstrap (`seed=20260728`).

## Disk probes and bundle integrity

The Stage 1 terminal manifests recorded free space before/after of
8,999,886,848/9,012,932,608 bytes and
9,011,716,096/9,011,249,152 bytes, respectively.
Their bundle allocation was 258,048 bytes each.
Immediately before Stage 2, the pre-launch probe had 8,992,907,264 bytes
free; the Stage 2 manifest recorded 8,996,065,280 bytes before and
9,050,406,912 bytes after execution.
Stage 2 allocated 33,456,128 bytes for both its bundle and its Stage 2 output
root, below the 250,000,000-byte bundle cap and the 2,000,000,000-byte root
cap.  The complete evidence root occupied 33,188 KiB at the final probe.
The runner completed without crossing its 6 GB live-space floor.

All JSON files in all three bundles were strictly parsed after execution.
The decompressed JSONL digest was independently recomputed rather than trusted
from the terminal output.

| Run | Terminal state | Process rows | Compressed process SHA-256 | Decompressed process SHA-256 | Summary JSON SHA-256 |
| --- | --- | ---: | --- | --- | --- |
| Stage 1 A | `completed` | 1,120 | `85340036bdd2b7b646c91657a7b2bd2cb63742d8c773d0190c599755494ab77d` | `1e2758bb1ad3f6234678371ca3a34a3d9d14f36eefc98823830168dcf9b1e982` | `fec93f2dbdbf99e88fbe292fde524b58f4fd46a42126ab29fd627e8abbca1f0e` |
| Stage 1 B | `completed` | 1,120 | `85340036bdd2b7b646c91657a7b2bd2cb63742d8c773d0190c599755494ab77d` | `1e2758bb1ad3f6234678371ca3a34a3d9d14f36eefc98823830168dcf9b1e982` | `fec93f2dbdbf99e88fbe292fde524b58f4fd46a42126ab29fd627e8abbca1f0e` |
| Stage 2 | `completed` | 280,000 | `34a6045d4ac5d23f72ac6d049d4d98a8f927ac9835b71f3a3050a00f8efaf524` | `6e1d6ea42e3c65117d62a734f8771491e1ec4181ef5778503bb9ca65a2bd54c7` | `0dddcc119e00abbcdb7dfad6491f39bde6bbf4107fee81a09a556ecf53a1c2b6` |

The exact collision-resistant Stage 1 bundle directories and their remaining
output-file hashes are:

| Run | Bundle directory | Manifest SHA-256 | Summary Markdown SHA-256 |
| --- | --- | --- | --- |
| Stage 1 A | `/private/tmp/cbf2026-localization-calibration/stage1/localization-calibration/20260728T164036.740337Z_36c8b0fcc66e4d6db93e00dd14c86c3d` | `8d118b7bc717e35d1ee8989d3fa0a5cd989789e975eee4940e16c1e864585ce3` | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |
| Stage 1 B | `/private/tmp/cbf2026-localization-calibration/stage1/localization-calibration/20260728T164101.774402Z_52a8641c802c461eb6af84f12775ef77` | `20b8378e86d0d69e3df41407209db7eaf76a8edd591c78207c57b32df3c9c45b` | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |

The two Stage 1 manifests also had identical settings and completion states;
the process-row count, decompressed-process SHA-256, and summary SHA-256 all
matched.  Thus the registered deterministic Stage 1 comparison passed.
The Stage 2 `summary.md` SHA-256 is
`6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9`;
its manifest SHA-256 is
`74f37cb4c4ea7e8e49eef660eade7d047fa137dfc27f1a36b4c675c4b364fa89`.
The Stage 2 files are:

```text
/private/tmp/cbf2026-localization-calibration/stage2/localization-calibration/20260728T164130.657823Z_6832cfbd6592469085adb88b5a165f59/
  calibration.jsonl.gz  (32 MiB apparent size)
  summary.json          (88 KiB)
  summary.md            (262 bytes)
  manifest.json         (2.7 KiB)
```

## Graph cases and paired definition

The paired cases use the same preserved truth geometry, initialization,
optimizer settings, FIM equations, and stable-keyed noise on every edge shared
between cases. `fixed_refs_wnls` retained only these predeclared references:

| UAVs | Fixed references |
| --- | --- |
| 1 | B0, B1 |
| 2 | B1, U1 |
| 3 | U1, U2 |
| 4--7 | U2--U3 through U5--U6, respectively |
| 8 | B1, B2 |
| 9 | B1, U8 |
| 10--14 | U8--U9 through U12--U13, respectively |

`dynamic_dag_wnls` used those fixed references together with every visible
base and every visible strictly lower squad-local UAV.  Its process records
showed changing memberships while preserving the lower-index DAG rule;
the 20-seed aggregate has 1,360 active-set transitions (68 per shared
trajectory).  The fixed-reference ablation has zero transitions.  This is a
reference-graph ablation, not an estimator comparison.

## Stage 2 measurements

Frame zero was retained in all 280,000 process records but excluded from the
primary statistics.  Its separately reported containment rates are 15.3571%
for the dynamic case and 47.5000% for the fixed case.

| Case | Primary containment | Seed-bootstrap 95% CI | Attempt status counts (converged / failed / invalid) | Retained status counts (converged / stale / invalid / failed) |
| --- | ---: | --- | --- | --- |
| Dynamic DAG | 18,319 / 139,720 = 13.1112% | [10.9597%, 15.1210%] | 18,578 / 3,378 / 117,764 | 18,578 / 3,378 / 117,764 / 0 |
| Fixed refs | 36,932 / 139,720 = 26.4329% | [24.6570%, 28.3618%] | 67,172 / 192 / 72,356 | 67,172 / 193 / 72,355 / 0 |

The seed-level bootstrap used exactly 10,000 resamples rather than treating
robot-frames as independent.  The dynamic seed rates range from 0 to 23.4183%;
the fixed seed rates range from 21.3856% to 35.2705%.

| Squad-local depth | Dynamic containment | Fixed containment |
| ---: | ---: | ---: |
| 1 | 80.5561% | 99.6994% |
| 2 | 9.0882% | 61.2425% |
| 3 | 2.1343% | 21.3076% |
| 4 | 0% | 0.5261% |
| 5 | 0% | 1.6232% |
| 6 | 0% | 0.1253% |
| 7 | 0% | 0.5060% |

The minimum depth containment is therefore 0% for the dynamic case and
0.1253% for the fixed case.  The following quantiles are calculated from the
finite values recorded in `summary.json`:

| Case | epsilon p50 / p95 / p99 / max | FIM lambda-min p1 / p5 / p50 | FIM condition p50 / p95 / p99 / max |
| --- | --- | --- | --- |
| Dynamic DAG | 2.5581 / 5.6108 / 7.0253 / 13.8491 | 0.18235 / 0.28589 / 1.37529 | 7.5899 / 26.5654 / 36.1549 / 217.1326 |
| Fixed refs | 7.6311 / 25.0562 / 122.5985 / 2,620,193.4515 | 0.0005988 / 0.0143354 / 0.154549 | 20.9792 / 118.3545 / 597.7556 / 153,097,136,126.8055 |

For the dynamic case, the 69 finite signed epsilon changes have p5 / p50 /
p95 of -1.9063 / 0 / 2.6256 m, mean 0.3907 m, minimum -1.9093 m, and maximum
2.7289 m.  The fixed case has no active-set transitions or epsilon changes.

The paired dynamic-minus-fixed changes are -13.3216 percentage points in
primary containment, +32.4993 percentage points in invalid-attempt rate, and
+2.2803 percentage points in failed-attempt rate.  Thus the dynamic case is
not no-worse on either invalid or failed attempts in this registered replay.

## Adequacy decision and limits

The Stage 2 summary records this gate outcome:

| Gate condition | Result |
| --- | --- |
| Aggregate dynamic containment at least 98% | Fail |
| Every dynamic squad-local depth at least 95% | Fail |
| Zero silently discarded failures | Pass (all 280,000 expected rows retained) |
| Dynamic invalid-attempt rate no worse than fixed | Fail |
| Dynamic failed-attempt rate no worse than fixed | Fail |
| Overall adequacy | Fail |

This failure is evidence to diagnose the approximation in a separate reviewed
round.  It is not a basis to inflate epsilon, change the active graph, alter
noise, replace the estimator, or claim an improved result from this evidence
round.

## Interpretation boundary

The 20 noise seeds reuse one preserved trajectory geometry.  The WNLS
estimates are offline sidecar outputs and never enter the controller, CBF,
CVT, communication state, or simulated vehicle dynamics.  The fixed-reference
case is a graph ablation.  The FIM covariance approximation propagates diagonal
reference uncertainty but ignores cross-correlations induced by shared
ancestors.  Frame zero is excluded from the primary coverage rate only because
the deployment position initializes the estimator.  Consequently this evidence
does not establish a mission-level probability guarantee, closed-loop
estimator robustness, or a closed-loop safety guarantee.
