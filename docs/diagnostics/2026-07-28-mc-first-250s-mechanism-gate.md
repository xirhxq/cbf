# MC-First 250-Second C0/R Mechanism Gate

Date: 2026-07-28

## Decision

The controlled 250-second C0/R pair is technically valid and scientifically
informative, but the rate-aware case does not pass the localization-margin
gate.
Both runs completed 500 frames and 7,000 applied optimal solves without a
non-finite value, failed solve, unapplied control, covariance-to-uncertainty
mismatch, or hard-QP residual below `-1e-7`.

Enabling the positive backward-difference uncertainty-rate term substantially
improved the required-reference margins:

- the worst tightened localization margin improved from `-6.737167 m` to
  `-3.516101 m`;
- observations below `-0.5 m` fell from 373 to 45;
- the first observation below `-0.5 m` moved from 125.5 s to 128.0 s.

The remaining R violations are material and consecutive, not the mild
sampled-data events accepted for reporting.
R also required a minimum hard-feasible planar \(L_\infty\) value of
`32.470567 m/s` at its peak, with two frames above the proposed `25 m/s`
component limit.
Consequently the pre-registered stop rule applies:

1. do not extend C0/R to 500 seconds;
2. do not run 250-second RB or RBP from the present controller;
3. diagnose and repair the rate realization and command-timing contract
   before another long run.

## Controlled comparison

After deleting only the case-specific `output_path` and `run_suffix`, the
materialized configurations differ in exactly one value:

```diff
- "cbfs.uncertainty-rate.mode": "off"
+ "cbfs.uncertainty-rate.mode": "backward-difference-positive"
```

Both cases otherwise use the same initial geometry, seed, model, hard CBFs,
minimum safety-row semantics, all required fixed-communication rows,
unbounded input setting, class-\(\mathcal K\) parameters, Gurobi solver,
binary, horizon, and logging.
This supports a within-scenario attribution of the observed improvement to
the implemented rate mode.
It does not establish a general effect size or a successful safety repair.

## Reproducibility

Common metadata:

- worktree: `/private/tmp/cbf2026-diagnostic`
- branch: `codex/cbf2026-diagnostic`
- manifest base commit:
  `cff3852831f969ad4bedf7d9b2f43b4f2a0cf42f`
- evidence-producing binary:
  `/private/tmp/cbf2026-diagnostic/build-diagnostic/Swarm`
- binary SHA-256:
  `748537d461fbcb22b3b8d70de62835ce4c352f735381c9661f1894b3fe994c8b`
- source-code state for that binary:
  `88dfeab81d754a811e173269d89438234e98ed51`
- solver: Gurobi
- seed: `20260727`
- horizon: 250 simulated seconds
- control/logging period: 0.5 seconds
- output root:
  `/private/tmp/cbf2026-results/mc-first-mechanism-250s`

The runner command was:

```bash
conda run -n cbf_env python -m scripts.diagnostics.run_diagnostic \
  --case CASE \
  --horizon 250 \
  --seed 20260727 \
  --binary build-diagnostic/Swarm \
  --output-root /private/tmp/cbf2026-results/mc-first-mechanism-250s
```

| Case | Bundle | Config SHA-256 | Manifest SHA-256 | Data SHA-256 | Analysis SHA-256 |
|---|---|---|---|---|---|
| C0 | `C0/20260728T062642.898445Z_b1cf0b5304a94b60a44eab513d47c55a` | `623440db40c1104ad1fcf0b6b1b05c313ad7151f9b707746155b3f680fb3fbf8` | `f5ecc5108ef4a6a03c6ab20fde89f9aa62840e8a1dc28319e48cfc388d65c0a6` | `3cb3368d671e30ab051e1632452424de513829c682500982b7fe43f460c81736` | `b6b7032b9138d93d79a1bf1fac276ee56706449b9dcfdd684bc43e96310c1e5f` |
| R | `R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725` | `cb13a87615da21e2f52ab41d039f41c4547e8d7311f1f2b791afe8474d10153d` | `6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb` | `3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527` | `0e4f19dcd15ad222403a0d7cf1addd32f8d9e590f2c9c5e67439bf482b69e80d` |

Bundle paths are relative to the output root above.
The C0 and R source-snapshot archive SHA-256 values are respectively
`71dffc94a47125cfd31db8ba4763441c32a8f24236c6cd4d50fb5dbef72d85c7`
and
`7edf5cad0ef182ac7693ce4dadf3bb6e9685facc701e99558975623c31bc066d`.

## Compact results

| Metric | C0 | R |
|---|---:|---:|
| Frames | 500 | 500 |
| Applied optimal solves | 7,000 | 7,000 |
| Other / unapplied solves | 0 | 0 |
| Non-finite values | 0 | 0 |
| Applied hard rows | 21,000 | 21,000 |
| Hard residual minimum | `5.396e-14` | `5.329e-14` |
| Hard residual missing / mismatch | 0 / 0 | 0 / 0 |
| Nominal localization minimum (m) | 2.609 | 2.617 |
| Tightened localization minimum (m) | -6.737 | -3.516 |
| First localization value below -0.5 m | 125.5 s | 128.0 s |
| Localization observations below -0.5 m | 373 | 45 |
| All negative localization observations | 642 | 505 |
| Tightened collision minimum (m) | 0.000306 | 3.460 |
| Collision observations below 0 m | 0 | 0 |
| Maximum minimum-required \(L_\infty\) (m/s) | 23.231 | 32.471 |
| Frames requiring \(L_\infty>25\) m/s | 0 | 2 |
| Applied-control \(L_\infty\) maximum (m/s) | 30.074 | 35.040 |
| Maximum scalar uncertainty (m) | 30.130 | 30.465 |
| Maximum positive logged uncertainty rate (m/s) | 6.879 | 6.881 |
| Maximum positive one-step uncertainty jump (m) | 3.440 | 3.440 |
| Maximum projected held-command mismatch (m/s) | 38.975 | 49.252 |
| Coverage cells / fraction | 89,078 / 0.989756 | 88,708 / 0.985644 |
| Solve time mean / p95 / max (ms) | 0.136 / 0.257 / 7.068 | 0.132 / 0.245 / 6.001 |

Each case cross-checked all 7,000 covariance-derived scalar uncertainty
values against the logged position covariance with zero mismatches and a
maximum absolute error of `7.105427357601002e-15`.
All 21,000 instantiated class-\(\mathcal K\) parameters matched the
materialized configuration.

## Failure localization

For C0, the first value below `-0.5 m` occurs at frame 251 (125.5 s) on
the UAV 2--4 required-reference relation.
The worst value occurs at frame 346 (173.0 s) on UAV 6--7.

For R, the first value below `-0.5 m` occurs at frame 256 (128.0 s) on
UAV 4--5.
The worst value again occurs at frame 346 (173.0 s) on UAV 6--7.
The 45 severe R observations are distributed across the same downstream
portion of the first squad's localization chain, with the largest groups on
UAV 4--5, UAV 4--6, and UAV 6--7 relations.

At the critical window, every active fixed-communication hard row remains
algebraically satisfied.
The raw transition decomposition separates the exact next-step margin from
the QP's linear prediction into held-command timing, uncertainty-rate
forecast error, and geometric curvature.
Across the 14 R transitions from a nonnegative to a negative localization
margin:

- the held-command term is the largest negative contribution in 8;
- the backward-difference rate term is largest in 2;
- geometric curvature is largest in 4 small base-link crossings;
- the mean contributions are `-0.6205 m`, `-0.0682 m`, and `-0.0196 m`
  respectively.

The first strict crossing is held-command dominated.
For the first crossing below `-0.5 m`, the QP predicts `+0.0152 m`, while
the held-command, rate-forecast, and curvature corrections are
`-0.6306 m`, `-0.0828 m`, and `-0.0322 m`.
At the single worst 172.5--173.0 s transition, the held-command and rate
forecast corrections are both material: `-1.0899 m` and `-1.3108 m`,
with only `-0.0225 m` from curvature.
Thus they are the same order at the worst event, but it would be too strong
to generalize that statement to all crossings.

There is a separate code/theory mismatch.
Although every robot anchor has a lower local index and the dependency graph
therefore remains acyclic, `getCovariance()` uses an 850 m range gate to add
and remove bases and non-formation lower-index anchors.
The R log contains 68 robot-frame covariance-information-set transitions.
Examples include:

- at 171.0 s UAV 1 loses base 2; the resulting uncertainty increase
  propagates to UAV 2 and the `base:2-1` margin reaches `-1.3998 m`;
- at 184.5 s UAV 9 loses base 2; its uncertainty increases by `1.4071 m`
  and the `base:9-1` margin reaches `-1.6091 m`;
- the maximum one-step uncertainty jump, `3.4404 m`, occurs when UAV 7
  loses optional anchor 4 at 86.0 s.

The worst UAV 6--7 chain crossing itself has no anchor-set change, so dynamic
anchors are not the sole explanation.
They do, however, directly explain the severe base-link failures and conflict
with a theorem that treats the localization/FIM reference set as immutable.

The evidence does not support adding a large sampled-data reserve.
Before changing the rate formula, the implementation and paper must first
choose one of two explicit information-set contracts:

1. compute the same FIM formula using only the predeclared lower-index
   localization DAG; or
2. retain optional range-gated FIM anchors and explicitly treat their
   transitions as part of the uncertainty contract.

Under the researcher's stated preference for a fixed localization graph and
no jump reserve, option 1 is the smaller implementation experiment.
After that rerun, any remaining material crossing should be addressed by
making the assumed held neighbor command equal the command actually applied
over the integration interval.
The analytic or forward-bounded rate remains a later theory-aligned option,
but the present Monte Carlo evidence does not make it the first repair.

The minimum safety-row mode is not a cause of the localization failure:
all 28 required fixed-communication rows are instantiated each frame.
The analyzer reproduces their raw geometric margins and consumed rate terms
to numerical precision.

## Input-bound implication

The task CBF does not constitute a hard component-wise input guarantee.
Both unbounded runs apply a component above `25 m/s`, and R's hard constraint
set itself requires more than `25 m/s` in two frames.
Therefore the explicit QP bounds added for RB/RBP remain necessary.
The present R trajectory also shows that `25 m/s` joint feasibility cannot be
assumed; it must be restored by the revised controller/geometry and then
tested, rather than inferred from the task CBF form.

This is an algorithm-level input result.
It does not extend the paper below the planar velocity-command layer.

## Disk and retention

The manifests recorded 12,429,852,672 bytes and 12,365,430,784 bytes free
before C0 and R respectively, both above the 8 GB start gate.
Their post-run values remained above the 6 GB hard floor.
After both analyses, `df -Pk` reported 11,996,152 KiB available.
The paired root occupied 118,064 KiB and all retained
`/private/tmp/cbf2026-results` occupied 304,120 KiB.
No evidence was deleted.

## Evidence boundary and next step

This single deterministic pair supports the limited statement that the
implemented backward-difference rate term reduces, but does not eliminate,
the reproduced long-horizon localization-margin failure.
It does not support Monte Carlo robustness, strict forward invariance,
bounded-input feasibility, estimator calibration, or a manuscript safety
claim.

The minimum next experiment is not another longer run.
It is a focused, decision-gated counterexample cycle:

1. decide whether the FIM information set is the fixed localization DAG or
   may contain range-gated optional anchors;
2. if fixed, retain the original FIM calculation but restrict its anchors to
   that declared set and rerun 250-second R;
3. if material violations remain, make the assumed held neighbor command
   equal the command actually applied over the interval;
4. retain the 172.5--173.5 s UAV 6--7 event as a regression fixture;
5. pass a short smoke and the same 250-second C0/R gate before reconsidering
   bounded or longer experiments.
