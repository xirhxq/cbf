# MC-First Fixed Information-Set Gate

Date: 2026-07-28

## Decision

The fixed-reference R mechanism gate did not pass its 20 s smoke gate.
The simulator stopped at time 0 with an infeasible QP, so the fixed-reference
run is not technically evaluable and no 250 s run was launched.  This is not
evidence that the fixed information-set change improves or degrades the
preserved dynamic-R trajectory.

The implementation preserves the FIM calculation
\(\Phi_i=J_i^\top\Sigma_i^{-1}J_i\) and changes only membership of its
reference set: optional range-visible anchors are excluded and the declared
lower-index/base references are retained.  The immediate blocker is instead
the production initial geometry.  Deterministic reconstruction in the
independent mechanism audit shows that the collinear fixed-reference geometry
yields a rank-one FIM and invalid (huge or indefinite) covariance, while UAV 6
receives an exact contradictory pair of hard halfspaces.  A fixed set has therefore
been structurally established, but not longitudinally validated.

The next action is a geometry/validity gate, not a 250 s rerun and not a
held-command change: fail fast on FIM SPD, \(\lambda_{\min}\), and condition
number, and on finite positive-semidefinite covariance and epsilon; use a
non-collinear triangular-ladder initial geometry (or explicitly decide a
predeclared lateral/base fixed anchor).  Only then rerun the 20 s smoke.
Held-command alignment remains deferred until that gate passes.

## Reproducibility and scope

| Item | Fixed-reference gate | Preserved dynamic-R comparison source |
|---|---|---|
| Worktree / branch | `/private/tmp/cbf2026-diagnostic` / `codex/cbf2026-diagnostic` | same diagnostic worktree |
| Source commit | `bcd369405f9fdb970e39416fa3c7cbe6df414d7d` | source code `88dfeab81d754a811e173269d89438234e98ed51`; report commit `9cb1786db01096e9d2f1bbdd6ee38fc6e29bbe29` |
| Binary | `build-diagnostic/Swarm` | `build-diagnostic/Swarm` |
| Binary SHA-256 | `8dc32e7d6e77fab5a11e17ccfd83f4b9f3671409cfc692499314574b41c0173e` | `748537d461fbcb22b3b8d70de62835ce4c352f735381c9661f1894b3fe994c8b` |
| Solver / seed / step | Gurobi / `20260727` / 0.5 s | Gurobi / `20260727` / 0.5 s |
| Requested case / horizon | R / 20 s | R / 250 s |
| FIM information set | declared fixed bases and strictly lower local-index UAV references only | declared references plus optional 850 m range-visible lower-index UAV/base anchors |
| Formula boundary | variance, propagated covariance, Jacobian, FIM, and inverse-covariance calculation unchanged | prior dynamic membership implementation |

The fixed smoke command was:

```bash
conda run -n cbf_env python -m scripts.diagnostics.run_diagnostic \
  --case R --horizon 20 --seed 20260727 \
  --binary build-diagnostic/Swarm \
  --output-root /private/tmp/cbf2026-results/mc-first-fixed-information-set-smoke
```

It returned shell exit code 2; the manifest records simulator return code 1
and `termination_reason="simulator_nonzero_exit"`.  The only simulator error
was `QP solve failed: infeasible`.  The runner preserved exactly one bundle:

```text
/private/tmp/cbf2026-results/mc-first-fixed-information-set-smoke/R/20260728T075434.056381Z_30f17034dc054dd4bf4ce26ab0fab00c
```

The planned 250 s root
`/private/tmp/cbf2026-results/mc-first-fixed-information-set-250s` remains
absent.  Its command was not launched, as required by the smoke stop rule.

## Build and focused verification before the gate

At `bcd3694`, configuration and build both succeeded:

```text
cmake -S . -B build-diagnostic -DCMAKE_BUILD_TYPE=Release     exit 0
cmake --build build-diagnostic -j2                            exit 0
ctest --test-dir build-diagnostic --output-on-failure          exit 0 (1/1)
```

Gurobi was enabled; HiGHS and OSQP were unavailable.  Four explicit C++
suites passed 27/27 doctest cases and 90/90 assertions.  The focused Python
command

```bash
conda run -n cbf_env python -m unittest \
  tests.test_analyze_diagnostic tests.test_run_diagnostic \
  tests.test_swarm_failure_exit tests.test_cmake_configuration -v
```

passed 51/51 tests.  `git diff --check` passed and the source worktree had no
tracked changes; its only status-visible entry was the allowed untracked
`build-diagnostic/` directory.

## Gate result and raw-audit boundary

The terminal raw data contain one runtime-zero state, with 14 robot,
formation, and covariance records.  The solver records are five `optimal`
(robots 1--5), one infeasible (UAV 6), and eight not attempted (UAVs 7--14).
At frame zero, 15 hard rows were optimal/applied and three were
infeasible/unapplied.  These compact counts establish that the failure occurs
before a usable transition sequence; no raw arrays are reproduced here.

Consequently the required smoke assertions cannot be evaluated: no
`analysis.json` was generated, there are no 40 completed frames, and there is
no numerical information-set transition count, required-reference mismatch
count, finite-value result, residual gate, or practical localization/collision
margin result for the fixed run.  It would be incorrect to report zeros for
any of them.

The independent integrity audit found the terminal bundle internally
consistent.  The preserved dynamic-R evidence was not modified: its
`analysis.json` remains SHA-256
`0e4f19dcd15ad222403a0d7cf1addd32f8d9e590f2c9c5e67439bf482b69e80d`.

## Dynamic versus fixed evidence

Only the left column is an existing, completed trajectory.  The right column
is deliberately recorded as unavailable rather than imputed from the
frame-zero failure.

| Metric | Dynamic R, existing 250 s evidence | Fixed R, attempted 20 s evidence |
|---|---:|---:|
| Completed frames / applied optimal solves | 500 / 7,000 | not completed / 5 |
| Other solver records | 0 | 1 infeasible; 8 not attempted |
| Information-set robot-frame records / transitions / required mismatches / malformed | 7,000 / 68 / 3,688 / 0 | 14 / not evaluable / not evaluable / not evaluable |
| Tightened localization minimum (m) | -3.516101 | not evaluable |
| Localization observations below -0.5 m | 45 | not evaluable |
| Tightened collision minimum (m) | 3.460 | not evaluable |
| Maximum scalar uncertainty / positive rate / jump (m; m/s; m) | 30.465 / 6.881 / 3.440 | not evaluable |
| Minimum-required / applied \(L_\infty\) maximum (m/s) | 32.471 / 35.040 | not evaluable |
| Hard residual minimum / coverage fraction | `5.329e-14` / 0.985644 | not evaluable |

The old dynamic metrics are retained only as the previously reported
mechanism evidence; they are not a baseline-to-fixed outcome comparison.
The code shape eliminates optional anchors structurally, but a successful
multi-frame run is required before claiming zero transitions or zero
required-reference mismatches over time.

## Retained evidence, hashes, and disk guard

| Artifact | SHA-256 or state |
|---|---|
| `manifest.json` | `9f98cf9937ab0ad93b25e73a72401024d3a650b69ebe67aa5a736488055a54c8` |
| `config.materialized.json` | `3967286632ecdbe10bb1f13f985b694b00875f2203fd9b42b327657d556efbcd` |
| bundle-local raw `2026-07-28_15-54-34_R_seed_20260727_20s/data.json` | `d996e50b312eadd0b8fccea05cd68a0b36e78bb3cadabfe0b8445ac5307f5de7` |
| `source-snapshot.tar.gz` | `20882e5eb2e7a77f75cefbfcf1eed9c3a8b3671f1c219aaa7ac3b9472ae48011` |
| `stdout.log` | `8b0e8e1e0690e31d8d7b85510b5aabcc69ec0ab0cabf6eefac229a90682dbf2a` |
| `stderr.log` | `e25c21fefaae9c5225005824ae86270e7db77c87984e8170c0350750a9c11937` |
| `analysis.json` | missing after nonzero simulator exit |
| legacy plan-path raw data | missing; raw data are bundle-local |

The recorded preflight free space was 12,688,461,824 B.  After the failed
smoke and at Task 4 finalization it was 12,689,330,176 B; the retained results
root was 305,984 KiB and this smoke bundle was 1,864 KiB.  The fixed 250 s
root was absent.  The manifest recorded 1,904,640 B allocated, below both the
2 GB output-root cap and 250 MB per-run cap.  No evidence was deleted,
renamed, compressed, or overwritten.

## Limitations and next decision

This is one deterministic, frame-zero counterexample.  It supplies no
bounded-input proof, estimator calibration, cross-correlation treatment,
multi-geometry Monte Carlo evidence, long-horizon fixed-set validation, or
manuscript claim.  It also does not test whether held-command timing is the
next residual mechanism.

Before any new trajectory, add a fail-fast rejection for non-SPD or
ill-conditioned \(\Phi_i\), non-finite/non-PSD covariance, and non-finite
epsilon, then repair or explicitly decide the non-collinear fixed-reference
geometry.  After the 20 s smoke passes with all records available and zero
fixed-set transitions/mismatches, reassess whether held-command alignment is
the smallest remaining mechanism before a 250 s rerun.
