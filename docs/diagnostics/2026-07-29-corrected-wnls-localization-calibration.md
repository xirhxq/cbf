# Corrected variable-weight WNLS localization calibration evidence

## Status

The two registered Stage 1 runs and the single registered Stage 2 run
completed with the corrected variable-weight WNLS estimator contract
`variable_weight_nls_full_residual_jacobian_v1`.
Stage 1 was deterministic at the content level, and every registered audit
retained its expected rows.
The Stage 2 adequacy decision is nevertheless an exact **FAIL**:
dynamic primary containment was only 44.7438%, every dynamic
squad-local-depth containment gate failed, and both no-worse attempt-rate
gates failed.
This is a calibration gap.
It does not validate the registered epsilon radius and does not
authorize an upgraded paper claim.
No result-dependent tuning, retry, graph change, noise change, or radius
inflation was made.

## Frozen implementation, review, environment, and inputs

The executable source was frozen at commit
`f6e995d6dc4711f0d2869636beb70f061e73063a`.
The actual execution checkout was
`2b4b5ac266d05f3f3ebb28b8129d8ec9876fd588`;
the frozen commit was its ancestor, both executable paths were unchanged
relative to the frozen commit, and the only tracked differences allowed by
the preflight were the registration and mathematical-review documents.
Every corrected manifest records implementation identity
`cbf2026-localization-calibration-v2` and runner SHA-256
`9af743f9445813e5b2f89f1989963319929d3f16b52b244563e2dca4fa52a90b`.

The independent mathematical review was tracked at
`docs/diagnostics/reviews/2026-07-29-variable-weight-wnls-math-review.md`
with SHA-256
`6c1dc9497175a602d58ef1ba53d8a8eca572b12567c063d52ae74d24faa3f09a`.
It returned `CLEAN / READY` for the frozen source after checking the full
position-dependent residual Jacobian, the \(J_q^\mathsf{T}q\) stationarity
gate, final-iteration convergence, and the separation between the
Gauss--Newton matrix and the final localization FIM.

The registered environment check returned exactly:

```text
Python 3.11.12
NumPy 1.24.4
```

This linked worktree's frozen CBF2026 task registration explicitly used
conda environment `cbf_env` with those versions.
The original checkout's external, untracked `.agents/AGENTS.md` convention
requiring `.venv` was not present or applicable in this linked worktree;
this clarification does not change any historical command.

The replay read, but did not copy, these immutable inputs:

| Input | Exact path | Bytes | SHA-256 |
| --- | --- | ---: | --- |
| Truth trajectory | `/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json` | 16,237,150 | `3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527` |
| Input manifest | `/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json` | 1,226 | `6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb` |

The input manifest records trajectory source commit
`cff3852831f969ad4bedf7d9b2f43b4f2a0cf42f` and materialized-configuration
SHA-256
`cb13a87615da21e2f52ab41d039f41c4547e8d7311f1f2b791afe8474d10153d`.

The frozen numerical settings were `ranging_sigma=0.5`,
`MAX_ITERATIONS=50`, `INITIAL_DAMPING=0.001`,
`STEP_TOLERANCE=1e-9`, `COST_TOLERANCE=1e-12`,
`RANGE_EPSILON=1e-12`, and
`RELATIVE_SPECTRAL_THRESHOLD=1e-12`.
The covariance came from the localization FIM, separately from the estimator
Gauss--Newton matrix, and the radius was computed exactly once as
\(\epsilon=3\sqrt{\lambda_{\max}(P)}\).

## Registered commands

Stage 1 A and Stage 1 B each used this exact command once:

```bash
conda run -n cbf_env python -m scripts.diagnostics.replay_localization_calibration \
  --data /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  --manifest /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json \
  --output-root /private/tmp/cbf2026-localization-calibration-corrected/stage1 \
  --seed 20260727 \
  --max-frames 40
```

Stage 2 used this exact command once:

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

## Bundle paths, disk readings, sizes, and hashes

The three collision-resistant bundle directories are:

| Run | Exact bundle directory |
| --- | --- |
| Stage 1 A | `/private/tmp/cbf2026-localization-calibration-corrected/stage1/localization-calibration/20260728T184323.997907Z_74cdb2fed5294fc6b52fa5f688f71f12` |
| Stage 1 B | `/private/tmp/cbf2026-localization-calibration-corrected/stage1/localization-calibration/20260728T184607.307681Z_f49e35405e914d938fc5e957af5f318d` |
| Stage 2 | `/private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288` |

All three terminal manifests record `termination_reason="completed"`.
Their manifest top level, manifest settings, summary top level, and summary
settings all record the exact estimator contract
`variable_weight_nls_full_residual_jacobian_v1`.
The runner-level disk readings and allocations were:

| Run | Free before / after (bytes) | Manifest bundle allocation (bytes) | Manifest output-root allocation (bytes) |
| --- | ---: | ---: | ---: |
| Stage 1 A | 9,516,396,544 / 9,515,708,416 | 303,104 | 303,104 |
| Stage 1 B | 9,356,353,536 / 9,355,980,800 | 303,104 | 610,304 |
| Stage 2 | 9,290,559,488 / 9,171,673,088 | 47,710,208 | 47,710,208 |

Every pre-invocation value exceeded the registered 8,000,000,000-byte launch
floor, and the terminal values exceeded the 6,000,000,000-byte live floor.
An independent post-run allocation probe measured Stage 1 A and B at 307,200
bytes each, Stage 2 at 47,714,304 bytes, and the complete corrected root at
48,328,704 bytes.
Thus every bundle remained below 250,000,000 bytes and the complete root
remained below 2,000,000,000 bytes.

Each artifact path is the bundle directory above followed by the listed file
name.
The decompressed JSONL rows were streamed for the virtual decompressed size
and digest; no decompressed evidence copy was accumulated.

| Run | Artifact | Apparent bytes | Independently recomputed SHA-256 |
| --- | --- | ---: | --- |
| Stage 1 A | `calibration.jsonl.gz` | 263,801 | `e3fd21c9431dd4e2dfc5373c7bf1e767283801bfd8f8393d27de2d18757cb127` |
| Stage 1 A | decompressed JSONL stream | 1,916,361 | `c1378d7ae1eda110004443ca2916ad8078d23c91d93576d99dce2e518beccb6a` |
| Stage 1 A | `summary.json` | 30,458 | `5d6b3afb826732c24db20c646f065f5cb71693b72ff5c7c8e9a1a5792a57864b` |
| Stage 1 A | `summary.md` | 262 | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |
| Stage 1 A | `manifest.json` | 2,610 | `9f244f56a5a83d2f9612d5035b455a26760fab20784dabc987dc6296096a06df` |
| Stage 1 B | `calibration.jsonl.gz` | 263,801 | `e3fd21c9431dd4e2dfc5373c7bf1e767283801bfd8f8393d27de2d18757cb127` |
| Stage 1 B | decompressed JSONL stream | 1,916,361 | `c1378d7ae1eda110004443ca2916ad8078d23c91d93576d99dce2e518beccb6a` |
| Stage 1 B | `summary.json` | 30,458 | `5d6b3afb826732c24db20c646f065f5cb71693b72ff5c7c8e9a1a5792a57864b` |
| Stage 1 B | `summary.md` | 262 | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |
| Stage 1 B | `manifest.json` | 2,610 | `4742c0a7743f9340b1f742ac3aa1ab5e833db97109080526d790c6953aa3223b` |
| Stage 2 | `calibration.jsonl.gz` | 47,608,272 | `38325523e395983d83491c81556bac1aae72ebd99508b3cdf148fdf72f3f8803` |
| Stage 2 | decompressed JSONL stream | 414,488,008 | `2c8a3f0f5684a281d5fbef3d7facea16f45d757d96b458827928dcac9e241a4f` |
| Stage 2 | `summary.json` | 91,573 | `f4f780dd4c9c1399d3f838c74054c9c0ba3159415bc13089aee53d0ebf7250bc` |
| Stage 2 | `summary.md` | 262 | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |
| Stage 2 | `manifest.json` | 2,920 | `39ee3b5494ed623d20d16766244427476cd081ed0869feecb9e5072c6c8f481d` |

The independently recomputed process and summary hashes match the hashes
stored in each terminal manifest.
The unequal manifest hashes are expected because run IDs, paths, and
timestamps differ.

## Stage 1 deterministic-content and audit result

Each Stage 1 bundle has exactly 1,120 rows: 28 frame-zero rows and 1,092
primary rows.
Each graph case contributes 560 total and 546 primary rows.
All 1,120 expected `(seed, graph_case, frame_index, robot_id)` keys occur
exactly once, with no duplicates, omissions, or extras.

The compressed and decompressed process hashes, `summary.json` hash,
`summary.md` hash, settings block, terminal state, row count, and complete key
set are exactly equal between Stage 1 A and B.
This passes the preregistered deterministic-content gate.
Their all-row status counts are also identical:

| Case | Attempt: converged / failed / invalid | Retained: converged / stale / invalid / failed |
| --- | ---: | ---: |
| Dynamic DAG | 166 / 36 / 358 | 166 / 34 / 358 / 2 |
| Fixed refs | 351 / 10 / 199 | 351 / 9 / 199 / 1 |

Strict JSON parsing rejected nonstandard constants and accepted every
manifest, summary, and process row.
The maximum stationarity norm among converged Stage 1 attempts was
`9.96011852283137e-10`; all were finite and at most `1e-9`.
Every failed or invalid attempt retained a nested failure object and reason,
and all 43 stale states retained the current failed attempt.
The fixed-reference membership audit, dynamic lower-index DAG audit, and
status reconciliation had zero mismatches.
The 1,120 shared measurements in each Stage 1 run had zero noise or noisy-range
mismatches across graph cases.

## Graph cases and paired-noise audit

`fixed_refs_wnls` used only these preregistered references:

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

`dynamic_dag_wnls` used these fixed references plus visible bases and visible,
strictly lower squad-local UAVs.
The raw Stage 2 membership audit found zero fixed-membership mismatches and
zero same-or-higher-index dynamic UAV references.
The cases used identical truth geometry, initialization, optimizer constants,
FIM equations, and stable-keyed noise on shared measurements.
Across all 280,000 Stage 2 shared measurements, both the noise and noisy range
matched exactly; there were zero paired mismatches.
This is a reference-graph ablation, not an estimator comparison.

## Stage 2 completeness and status audit

Stage 2 contains exactly 280,000 rows.
Frame zero contributes 560 rows and is reported separately; 279,440 rows are
primary.
Each graph case has 140,000 total rows and 139,720 primary rows, with seeds
`20260727` through `20260746` in order, 500 frames, and 14 UAVs.
All expected keys occur exactly once, with no duplicate, omitted, or extra
keys.

The maximum stationarity norm among all converged attempts was
`9.999536187738804e-10`; every converged value was finite and at most `1e-9`.
All 280,000 rows parsed as strict JSON without `NaN`, `Infinity`, or
`-Infinity`.
Every failed or invalid attempt remained in its process row with its nested
failure and reason.
Every stale retained state preserved the corresponding current failed
attempt.
No failure was silently discarded.

Primary attempt and retained-state counts are:

| Case | Attempt: converged / failed / invalid | Retained: converged / stale / invalid / failed |
| --- | ---: | ---: |
| Dynamic DAG | 67,042 / 6,311 / 66,367 | 67,042 / 6,311 / 66,367 / 0 |
| Fixed refs | 82,151 / 184 / 57,385 | 82,151 / 184 / 57,385 / 0 |

Frame-zero attempt and retained-state counts are:

| Case | Attempt: converged / failed / invalid | Retained: converged / stale / invalid / failed |
| --- | ---: | ---: |
| Dynamic DAG | 147 / 31 / 102 | 147 / 0 / 102 / 31 |
| Fixed refs | 165 / 29 / 86 | 165 / 0 / 86 / 29 |

These tables reconcile to the all-row raw counts:
dynamic 67,189 / 6,342 / 66,469 attempts and
67,189 / 6,311 / 66,469 / 31 retained states;
fixed 82,316 / 213 / 57,471 attempts and
82,316 / 184 / 57,471 / 29 retained states.

## Containment, bootstrap, and depth results

Frame zero was retained but excluded from the primary adequacy statistics.
Its dynamic containment was 144/280 = 51.4286%, and its fixed containment was
163/280 = 58.2143%.

| Case | Primary containment | Seed-bootstrap 95% CI |
| --- | ---: | --- |
| Dynamic DAG | 62,516 / 139,720 = 44.7438% | [34.2591%, 54.5785%] |
| Fixed refs | 41,913 / 139,720 = 29.9979% | [27.4098%, 32.5809%] |

The confidence intervals use exactly 10,000 seed-level resamples with RNG
seed `20260728`.
They do not resample robot-frame rows as independent observations.
The per-seed primary containment rates are:

| Seed | Dynamic | Fixed |
| ---: | ---: | ---: |
| 20260727 | 32.1786% | 27.9273% |
| 20260728 | 38.5056% | 21.7005% |
| 20260729 | 6.6275% | 27.8987% |
| 20260730 | 25.7944% | 27.6124% |
| 20260731 | 43.4583% | 21.4429% |
| 20260732 | 70.5840% | 34.5405% |
| 20260733 | 61.6232% | 34.5691% |
| 20260734 | 36.6304% | 35.7286% |
| 20260735 | 82.1930% | 40.8245% |
| 20260736 | 0% | 27.7269% |
| 20260737 | 13.0547% | 35.6284% |
| 20260738 | 49.7280% | 35.6427% |
| 20260739 | 25.0930% | 21.7292% |
| 20260740 | 56.9997% | 21.7292% |
| 20260741 | 52.3332% | 27.6267% |
| 20260742 | 60.5640% | 33.6816% |
| 20260743 | 58.8892% | 33.9393% |
| 20260744 | 71.8437% | 21.5288% |
| 20260745 | 82.2645% | 34.4403% |
| 20260746 | 26.5102% | 34.0395% |

Every depth has 19,960 primary rows per case:

| Squad-local depth | Dynamic containment | Fixed containment |
| ---: | ---: | ---: |
| 1 | 16,082 / 19,960 = 80.5711% | 19,900 / 19,960 = 99.6994% |
| 2 | 12,877 / 19,960 = 64.5140% | 15,689 / 19,960 = 78.6022% |
| 3 | 10,952 / 19,960 = 54.8697% | 5,521 / 19,960 = 27.6603% |
| 4 | 8,844 / 19,960 = 44.3086% | 127 / 19,960 = 0.6363% |
| 5 | 6,828 / 19,960 = 34.2084% | 236 / 19,960 = 1.1824% |
| 6 | 4,010 / 19,960 = 20.0902% | 100 / 19,960 = 0.5010% |
| 7 | 2,923 / 19,960 = 14.6443% | 340 / 19,960 = 1.7034% |

The minimum dynamic depth containment is therefore 14.6443%, far below the
registered 95% per-depth threshold.

## FIM, epsilon, transitions, and paired changes

The following primary-row quantiles are calculated from finite values in the
preserved Stage 2 summary:

| Case | \(\epsilon\) p50 / p95 / p99 / max (m) | FIM \(\lambda_{\min}\) p1 / p5 / p50 | FIM condition p50 / p95 / p99 / max |
| --- | --- | --- | --- |
| Dynamic DAG | 4.7383 / 12.7890 / 18.8103 / 30.9788 | 0.0254362 / 0.0550259 / 0.400871 | 9.12645 / 43.8795 / 229.533 / 733.343 |
| Fixed refs | 8.4241 / 29.1751 / 200.4103 / 4,438,537.1415 | 0.000224080 / 0.0105734 / 0.126823 | 19.8189 / 108.476 / 517.507 / 152,901,004,909.943 |

The corresponding error-to-\(\epsilon\) ratios are:

| Case | p50 / p95 / p99 / max |
| --- | --- |
| Dynamic DAG | 0.327501 / 3.10781 / 10.8339 / 252.481 |
| Fixed refs | 0.794416 / 211.225 / 242.435 / 332.337 |

The dynamic graph has 1,360 active-set transitions, 68 for each reused
trajectory.
Of those transitions, 600 have finite signed epsilon changes:
p5 / p50 / p95 are -1.25684 / 0.152429 / 2.60394 m,
the mean is 0.338188 m, and the minimum / maximum are
-6.03046 / 3.60718 m.
The other 760 transitions have no finite epsilon change because the current
and preceding retained states do not both provide finite epsilon values.
The fixed graph has zero active-set transitions.

The paired primary-containment discordance table is:

| Dynamic contained? | Fixed contained? | Paired rows |
| --- | --- | ---: |
| Yes | Yes | 30,701 |
| Yes | No | 31,815 |
| No | Yes | 11,212 |
| No | No | 65,992 |

Thus the paired dynamic-minus-fixed aggregate containment change is
+14.7459 percentage points.
Among the 48,902 paired primary rows with finite epsilon in both cases, the
dynamic-minus-fixed epsilon change has median -3.51289 m, p5 -16.1924 m,
p95 approximately \(8.88\times10^{-16}\) m, mean -202.644 m,
minimum -4,438,520.572 m, and maximum 2.61734 m.
The very negative mean is driven by the fixed-case extreme tail and is
descriptive only; this finite-pair subset is not an adequacy statistic.

## Registered invalid and failed rates

For graph case \(c\), the registered denominator includes every primary row
in that case:

\[
N_c^{\mathrm{primary}}=139{,}720.
\]

The rates use attempt status, not retained status:

\[
R_c^{\mathrm{invalid}}
=
\frac{\#\{r:\ r.\mathrm{graph\_case}=c
\land r.\mathrm{primary\_statistics}
\land r.\mathrm{attempt\_status}=\texttt{invalid}\}}
{139{,}720},
\]

\[
R_c^{\mathrm{failed}}
=
\frac{\#\{r:\ r.\mathrm{graph\_case}=c
\land r.\mathrm{primary\_statistics}
\land r.\mathrm{attempt\_status}=\texttt{failed}\}}
{139{,}720}.
\]

The exact results are:

| Case | Invalid-attempt rate | Failed-attempt rate |
| --- | ---: | ---: |
| Dynamic DAG | 66,367 / 139,720 = 47.5000% | 6,311 / 139,720 = 4.5169% |
| Fixed refs | 57,385 / 139,720 = 41.0714% | 184 / 139,720 = 0.1317% |

The dynamic-minus-fixed changes are therefore +6.4286 percentage points for
invalid attempts and +4.3852 percentage points for failed attempts.
Both registered no-worse gates fail.

## Adequacy decision

The direct raw-row recomputation and preserved summary agree:

| Preregistered condition | Exact result | Gate |
| --- | --- | --- |
| Dynamic aggregate primary containment at least 98% | 44.7438% | Fail |
| Every dynamic squad-local depth at least 95% | minimum 14.6443% | Fail |
| Processed rows equal expected rows with zero silent drops | 280,000 / 280,000 | Pass |
| Dynamic invalid-attempt rate no worse than fixed | 47.5000% > 41.0714% | Fail |
| Dynamic failed-attempt rate no worse than fixed | 4.5169% > 0.1317% | Fail |
| Overall adequacy | four of five conditions fail | **FAIL** |

This failed registered run is a calibration gap.
It must not be described as validating the epsilon radius.

## Preserved but excluded old bundle

The previous Stage 2 remains preserved at:

```text
/private/tmp/cbf2026-localization-calibration/stage2/localization-calibration/20260728T164130.657823Z_6832cfbd6592469085adb88b5a165f59
```

Its artifact hashes remain:

| Artifact | SHA-256 |
| --- | --- |
| `calibration.jsonl.gz` | `34a6045d4ac5d23f72ac6d049d4d98a8f927ac9835b71f3a3050a00f8efaf524` |
| decompressed JSONL | `6e1d6ea42e3c65117d62a734f8771491e1ec4181ef5778503bb9ca65a2bd54c7` |
| `summary.json` | `0dddcc119e00abbcdb7dfad6491f39bde6bbf4107fee81a09a556ecf53a1c2b6` |
| `summary.md` | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |
| `manifest.json` | `74f37cb4c4ea7e8e49eef660eade7d047fa137dfc27f1a36b4c675c4b364fa89` |

That bundle is evidence of the previous implementation, but it is excluded
from objective-correct estimator adequacy because its solver omitted the
derivative of the position-dependent weight.
It was not deleted, overwritten, moved, or relabeled.

## Interpretation boundary

The 20 ranging-noise seeds reuse one preserved trajectory geometry; they are
not 20 independent mission trajectories.
The WNLS estimates are offline sidecar outputs and never enter the controller,
CBF, CVT, communication state, or simulated vehicle dynamics.
The fixed-reference case is a graph ablation under the same estimator, not an
estimator comparison.
The localization FIM propagates marginal reference covariance but omits
shared-ancestor cross-correlations.
Consequently these results provide neither a mission-level probability
guarantee nor a closed-loop safety or estimator-robustness guarantee.
The failed adequacy gate is empirical evidence of a remaining calibration
gap, not empirical validation of the coefficient-3 FIM-derived epsilon
radius.
