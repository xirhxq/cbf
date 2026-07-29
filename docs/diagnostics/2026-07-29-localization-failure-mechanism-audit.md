# Localization failure-mechanism audit (2026-07-29)

## Decision

This is a post-hoc exploratory mechanism audit of one preserved localization
calibration trajectory with 20 paired range-noise seeds.
The registered analyzer was executed once,
completed,
and was not retried.
The result is descriptive mechanism evidence only.
It does not support a causal graph comparison,
validate the localization uncertainty radius,
or establish mission,
robustness,
or safety claims.
No paper claim or manuscript text is updated from this audit.

Within this frozen replay,
`upstream_unavailable` is the largest observed non-contained class in both
graph cases:
50,898 rows for `dynamic_dag_wnls` and 42,914 rows for
`fixed_refs_wnls`.
Its row,
edge,
depth,
and persistence profile prioritizes a separately designed and registered
investigation of upstream-reference availability and cascade propagation.
That next investigation must use prospective interventions or controlled
reference-availability perturbations,
multiple independently generated trajectories,
and repeated seeds;
the present audit cannot identify the cause of the observed differences.

## Immutable inputs and outputs

The exact source bundle was:

```text
/private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288
```

The exact derived output directory is:

```text
/private/tmp/cbf2026-localization-failure-analysis/20260729T045931Z_d7c1cdcbf6434b6ca944ffacec262f4e
```

The five source identities matched before execution and remained unchanged
afterward:

| Source identity | SHA-256 |
| --- | --- |
| `calibration.jsonl.gz`, compressed bytes | `38325523e395983d83491c81556bac1aae72ebd99508b3cdf148fdf72f3f8803` |
| Decompressed JSONL stream bytes | `2c8a3f0f5684a281d5fbef3d7facea16f45d757d96b458827928dcac9e241a4f` |
| `summary.json` | `f4f780dd4c9c1399d3f838c74054c9c0ba3159415bc13089aee53d0ebf7250bc` |
| `summary.md` | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |
| `manifest.json` | `39ee3b5494ed623d20d16766244427476cd081ed0869feecb9e5072c6c8f481d` |

The completed output contains exactly the registered JSON and Markdown files:

| Derived output | SHA-256 |
| --- | --- |
| `failure-mechanisms.json` | `0e86f884a262fa7259b4a3c13ea004c9cb711176f4240ae8d7d100bcb87a375e` |
| `failure-mechanisms.md` | `0fcb6c84875c88c9e18169accf663fdc4321b4995dc434032edbf67ba973f27f` |

The strict JSON reports schema
`cbf2026-localization-failure-analysis-v1`,
status `completed`,
280,000 observed rows,
and 279,440 primary rows.
Its estimator contract is
`variable_weight_nls_full_residual_jacobian_v1`,
and its internal source hashes all match the identities above.

## Registered execution

| Field | Executed value |
| --- | --- |
| Worktree | `/private/tmp/cbf2026-diagnostic` |
| Branch | `codex/cbf2026-diagnostic` |
| Analyzer module | `scripts.diagnostics.analyze_localization_failures` |
| Executable commit | `7e44ab4ad445d925c8a39062e9701cb093b2cd99` |
| Executable source SHA-256 | `1e3a7b7cd615b258fb83af53f7264d7aff327685f42c85787d0d9ae7df0b6315` |
| Conda environment | `cbf_env` |
| Python | `3.11.12` |
| NumPy | `1.24.4` |

The output parent was created once with the registered setup command:

```bash
mkdir -m 700 /private/tmp/cbf2026-localization-failure-analysis
```

The sole production analyzer invocation was:

```bash
conda run -n cbf_env python -m scripts.diagnostics.analyze_localization_failures \
  --bundle-dir /private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288 \
  --output-dir /private/tmp/cbf2026-localization-failure-analysis/20260729T045931Z_d7c1cdcbf6434b6ca944ffacec262f4e \
  --max-examples-per-bucket 5
```

There was one production invocation,
it completed,
and there was no result-dependent retry,
path substitution,
threshold change,
taxonomy change,
or estimator change.

The launch probe reported 8,217,591,808 available bytes.
The latest read-only post-run `df -k /private/tmp` probe reported
7,877,688 KiB,
or 8,066,752,512 available bytes.
The difference between these two filesystem-wide observations is not assigned
to the analyzer.
An independent `du -sk` probe measured the exact output directory at 408 KiB,
or 417,792 allocated bytes,
below the registered 10,000,000-byte run cap.
Because the derived-analysis root contains only this completed run,
its allocation is also 417,792 bytes,
below the registered 2,000,000,000-byte root cap.
The post-run free-space probe remained above the 6,000,000,000-byte live
floor.
During subsequent documentation and review work,
the exact rebuildable cache
`/Users/xirhxq/.cache/codex-runtimes`
regrew to 1,583,072 KiB and free space reached 6,940,770,304 bytes.
The previously authorized cache-only cleanup removed that directory,
after which a read-only probe reported 8,573,808,640 available bytes.
The source bundle and 417,792-byte derived output remained unchanged;
no evidence,
user document,
or Git history was removed.

## Full primary-row failure budgets

Classification uses current `attempt_status`,
not retained status.
Each graph-case denominator is 139,720 primary rows.
The six classes reconcile exactly to that denominator.

| Class | `dynamic_dag_wnls` | Percent | `fixed_refs_wnls` | Percent | Dynamic minus fixed |
| --- | ---: | ---: | ---: | ---: | ---: |
| `contained` | 62,516 | 44.744% | 41,913 | 29.998% | +20,603 |
| `converged_outside_radius` | 4,526 | 3.239% | 40,238 | 28.799% | -35,712 |
| `upstream_unavailable` | 50,898 | 36.429% | 42,914 | 30.714% | +7,984 |
| `invalid_input_or_numeric` | 15,469 | 11.071% | 14,471 | 10.357% | +998 |
| `wnls_nonconvergence` | 6,311 | 4.517% | 184 | 0.132% | +6,127 |
| `other_failed` | 0 | 0.000% | 0 | 0.000% | 0 |
| Total | 139,720 | 100.000% | 139,720 | 100.000% | 0 |

These differences describe this replay only.
They are not estimates of causal effects or evidence that either graph design
is superior.

## Upstream lineage and descriptive bootstrap

An upstream-unavailable row is an observer row;
it can contain more than one unavailable UAV-reference edge.
The lineage counters were:

| Lineage counter | `dynamic_dag_wnls` | `fixed_refs_wnls` |
| --- | ---: | ---: |
| Upstream-unavailable observer rows | 50,898 | 42,914 |
| Unavailable UAV-reference edges | 105,286 | 71,856 |
| Edge predecessor: `invalid_input_or_numeric` | 33,182 | 25,948 |
| Edge predecessor: `upstream_unavailable` | 72,104 | 45,908 |
| Edge predecessor: any other/not observed | 0 | 0 |

The propagation-depth counts reconcile to the observer-row totals:

| Propagation depth | `dynamic_dag_wnls` | `fixed_refs_wnls` |
| ---: | ---: | ---: |
| 1 | 14,471 | 13,972 |
| 2 | 11,477 | 11,976 |
| 3 | 9,481 | 9,980 |
| 4 | 7,485 | 6,986 |
| 5 | 5,489 | 0 |
| 6 | 2,495 | 0 |
| Not observed | 0 | 0 |
| Total | 50,898 | 42,914 |

The registered descriptive statistic is:

\[
D_{\mathrm{upstream}}
=
\frac{50{,}898-42{,}914}
     {(50{,}898+15{,}469)-(42{,}914+14{,}471)}
=
\frac{7{,}984}{8{,}982}
=
\frac{8}{9}.
\]

Only 7,509 of 10,000 paired-seed bootstrap resamples were estimable;
2,491 had a non-positive denominator.
This is below the registered minimum of 9,500,
so the percentile confidence interval is not reported (`null`).
No p-value was registered or reported.
The \(8/9\) point estimate is therefore retained only as a descriptive ratio,
not an inferential or causal estimate.

## Conditional containment, ratio, and normalized squared error

All quantities in this section are conditional on a converged attempt.
The normalized-squared-error denominator additionally requires finite,
symmetric,
positive-definite covariance and finite \(q\).
There were no conditional-covariance exclusions in either case.

| Metric | `dynamic_dag_wnls` | `fixed_refs_wnls` |
| --- | ---: | ---: |
| Converged denominator | 67,042 | 82,151 |
| Epsilon-contained count | 62,516 | 41,913 |
| Conditional containment | 93.249% | 51.019% |
| Finite ratio count / invalid ratio count | 67,042 / 0 | 82,151 / 0 |
| Mean error-to-epsilon ratio | 0.4610 | 57.2633 |
| Maximum error-to-epsilon ratio | 26.3631 | 332.3372 |
| Finite \(q\) count / covariance-invalid count | 67,042 / 0 | 82,151 / 0 |
| \(q>5.991464547\) | 9,488 (14.152%) | 42,871 (52.186%) |
| \(q>9\) | 5,986 (8.929%) | 41,215 (50.170%) |

The complete error-to-epsilon ratio bins were:

| Ratio bin | `dynamic_dag_wnls` | `fixed_refs_wnls` |
| --- | ---: | ---: |
| `[0,0.5)` | 50,201 | 35,739 |
| `[0.5,1)` | 12,315 | 6,174 |
| `[1,2)` | 2,759 | 1,307 |
| `[2,5)` | 1,402 | 2,137 |
| `[5,infinity)` | 365 | 36,794 |
| Total | 67,042 | 82,151 |

The complete \(q\) bins were:

| \(q\) bin | `dynamic_dag_wnls` | `fixed_refs_wnls` |
| --- | ---: | ---: |
| `[0,2.295748929)` | 40,594 | 28,507 |
| `[2.295748929,5.991464547)` | 16,960 | 10,773 |
| `[5.991464547,9]` | 3,502 | 1,656 |
| `(9,infinity)` | 5,986 | 41,215 |
| Total | 67,042 | 82,151 |

These conditional values diagnose the frozen offline replay.
They do not validate the epsilon radius,
nominal coverage,
mission probability,
robustness,
or safety.

## Frame-zero initialization and persistence

Frame zero is separate from the 279,440 primary-row budget.
Across 280 frame-zero rows per graph case,
the initialization budgets were:

| Case | Contained | Outside radius | Upstream unavailable | Invalid/numeric | WNLS nonconvergence | Other failed |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `dynamic_dag_wnls` | 144 | 3 | 102 | 0 | 31 | 0 |
| `fixed_refs_wnls` | 163 | 2 | 86 | 0 | 29 | 0 |

The frame-zero upstream counts increased with squad-local depth:

| Depth | Dynamic upstream / 40 | Fixed upstream / 40 |
| ---: | ---: | ---: |
| 1 | 0 | 0 |
| 2 | 5 | 0 |
| 3 | 11 | 0 |
| 4 | 15 | 14 |
| 5 | 19 | 20 |
| 6 | 23 | 24 |
| 7 | 29 | 28 |

Persistence is one record per
`(seed, graph_case, robot_id)` primary sequence.
There are 280 records per case:

| Persistence result | `dynamic_dag_wnls` | `fixed_refs_wnls` |
| --- | ---: | ---: |
| Never primary converged | 133 | 115 |
| Maximum primary frames before first convergence | 499 | 499 |
| Maximum upstream-unavailable streak | 499 | 499 |
| Maximum WNLS-nonconvergence streak | 34 | 5 |

Across both cases this is 560 persistence records,
248 never-primary-converged records,
and maxima of 499 frames before first convergence,
499 upstream-unavailable frames,
and 34 consecutive WNLS-nonconvergence frames.
These persistence summaries are descriptive and do not identify an
initialization or geometry cause.

## Depth and time results

Every squad-local depth has 19,960 primary rows per graph case.
The table reports the five nonzero-capable classes followed by conditional
containment and the conditional \(q>9\) rate;
`other_failed` is zero at every depth.

| Case | Depth | Contained | Outside | Upstream | Invalid | WNLS | Containment | \(q>9\) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Dynamic | 1 | 16,082 | 38 | 0 | 2,495 | 1,345 | 99.764% | 1.098% |
| Dynamic | 2 | 12,877 | 314 | 2,495 | 2,994 | 1,280 | 97.620% | 3.449% |
| Dynamic | 3 | 10,952 | 633 | 5,489 | 1,996 | 890 | 94.536% | 7.933% |
| Dynamic | 4 | 8,844 | 855 | 7,485 | 1,996 | 780 | 91.185% | 11.465% |
| Dynamic | 5 | 6,828 | 877 | 9,481 | 1,996 | 778 | 88.618% | 15.055% |
| Dynamic | 6 | 4,010 | 847 | 11,477 | 2,994 | 632 | 82.561% | 20.898% |
| Dynamic | 7 | 2,923 | 962 | 14,471 | 998 | 606 | 75.238% | 29.550% |
| Fixed | 1 | 19,900 | 60 | 0 | 0 | 0 | 99.699% | 1.152% |
| Fixed | 2 | 15,689 | 4,196 | 0 | 0 | 75 | 78.899% | 22.183% |
| Fixed | 3 | 5,521 | 7,396 | 0 | 6,986 | 57 | 42.742% | 58.442% |
| Fixed | 4 | 127 | 9,836 | 6,986 | 2,994 | 17 | 1.275% | 99.538% |
| Fixed | 5 | 236 | 7,739 | 9,980 | 1,996 | 9 | 2.959% | 99.260% |
| Fixed | 6 | 100 | 5,874 | 11,976 | 1,996 | 14 | 1.674% | 99.012% |
| Fixed | 7 | 340 | 5,137 | 13,972 | 499 | 12 | 6.208% | 96.348% |

For the dynamic graph,
upstream-unavailable counts rose from zero at depth 1 to 14,471 at depth 7,
while conditional containment fell from 99.764% to 75.238% and the
conditional \(q>9\) rate rose from 1.098% to 29.550%.
For fixed references,
upstream-unavailable rows first appeared at depth 4 and reached 13,972 at
depth 7;
conditional containment was below 6.3% at depths 4--7.
These are depth associations,
not effects attributed to depth.

The time-bin budgets were:

| Case | Frames | Denominator | Contained | Outside | Upstream | Invalid | WNLS | Containment | \(q>9\) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Dynamic | 1--100 | 28,000 | 9,915 | 927 | 10,200 | 3,100 | 3,858 | 91.450% | 12.276% |
| Dynamic | 101--200 | 28,000 | 12,734 | 1,079 | 10,200 | 3,100 | 887 | 92.189% | 9.860% |
| Dynamic | 201--300 | 28,000 | 13,432 | 728 | 10,200 | 3,100 | 540 | 94.859% | 6.490% |
| Dynamic | 301--400 | 28,000 | 13,778 | 579 | 10,200 | 3,100 | 343 | 95.967% | 6.088% |
| Dynamic | 401--499 | 27,720 | 12,657 | 1,213 | 10,098 | 3,069 | 683 | 91.255% | 10.815% |
| Fixed | 1--100 | 28,000 | 7,922 | 8,394 | 8,600 | 2,900 | 184 | 48.554% | 55.534% |
| Fixed | 101--200 | 28,000 | 8,518 | 7,982 | 8,600 | 2,900 | 0 | 51.624% | 48.909% |
| Fixed | 201--300 | 28,000 | 8,487 | 8,013 | 8,600 | 2,900 | 0 | 51.436% | 49.012% |
| Fixed | 301--400 | 28,000 | 8,565 | 7,935 | 8,600 | 2,900 | 0 | 51.909% | 48.564% |
| Fixed | 401--499 | 27,720 | 8,421 | 7,914 | 8,514 | 2,871 | 0 | 51.552% | 48.877% |

Dynamic WNLS nonconvergence was concentrated in frames 1--100
(3,858 of 6,311 total) and reached its minimum in frames 301--400 (343);
the upstream-unavailable count was 10,200 in each full 100-frame bin.
Fixed-reference WNLS nonconvergence occurred only in frames 1--100,
while its other large categories were nearly stable across time bins.
The shorter final bin contains 99 frames and therefore has a denominator of
27,720.

The registered dynamic 7-by-5 depth/time table also reconciles:
each of its first four time cells has 4,000 rows per depth,
and each final-bin cell has 3,960.
Within every full time bin,
dynamic upstream-unavailable counts increase from zero at depth 1 through
2,900 at depth 7.
Dynamic WNLS nonconvergence is largest in the first time bin at depths 2--7
(553,
663,
645,
609,
524,
and 395 rows),
then is substantially smaller in the middle bins.
The complete 35-cell six-class counters remain in the immutable JSON and
Markdown outputs identified above.

## Supported and unsupported inference

Supported within the frozen replay:

- the exact six-class,
  lineage,
  initialization,
  persistence,
  calibration,
  depth,
  and time counts reported above;
- the descriptive association of upstream-unavailable rows with greater
  squad-local depth and long persistence;
- the observation that upstream unavailability is the largest non-contained
  class in both graph cases; and
- the descriptive \(D_{\mathrm{upstream}}=8/9\) point estimate,
  with no interval because the registered estimability gate failed.

Not supported:

- a causal conclusion about graph choice,
  initialization,
  geometry,
  estimator behavior,
  depth,
  or upstream propagation;
- a population,
  multi-trajectory,
  mission-level,
  or controller-in-the-loop generalization;
- validation or recalibration of the epsilon radius or any nominal coverage
  probability;
- a robustness,
  safety,
  collision-avoidance,
  connectivity,
  or mission-success guarantee;
- a graph-superiority claim or a paper claim upgrade; or
- treating the absent bootstrap interval as evidence of precision or
  statistical significance.

The audit remains limited to one preserved trajectory with 20 paired seeds,
an offline estimator outside the controller,
and no shared-ancestor cross-covariance model.
It includes no causal estimator or graph comparison and no radius,
mission-probability,
robustness,
or safety validation.
