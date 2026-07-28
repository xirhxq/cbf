# MC-First Fixed-Geometry Gate

Date: 2026-07-28

## Decision

The 20 s `RG` run is a fixed-reference *geometry-admissibility gate*, not a
causal comparison with the earlier dynamic-information-set `R` run.  `RG`
changes the initial geometry while retaining the rate-aware, fixed-information-
set, minimum-safety-row controller semantics.  It therefore tests whether the
declared non-collinear geometry can execute coherently; it cannot isolate a
geometry, information-set, or controller treatment effect.

The run completed all 40 frames and passed the solver, hard-row, finite-value,
information-set, and localization gates.  It nevertheless failed the
pre-registered collision practical-tolerance gate because one isolated
all-pair robust margin was `-0.6566649907195625 m`, which is
`0.1566649907195625 m` below the `-0.5 m` threshold.  The required stop rule
was applied: the 250 s run was not launched and its output root remains absent.
No threshold was relaxed and no geometry was changed after observing the
result.

Primary evidence is the Task 3 execution report,
`.superpowers/sdd/2026-07-28-cbf2026-fixed-geometry-gate-implementation/task-3-report.md`,
the retained bundle below, and the corrected independent mechanism audit,
`.superpowers/sdd/2026-07-28-cbf2026-fixed-geometry-gate-implementation/task-4-mechanism-audit.md`.
The first audit pass's zero-based `partId` interpretation was erroneous and is
not used here.

## Guard, source, and binary provenance

The fail-fast invalid-FIM geometry guard entered at commit
`43a8c86` (`fix(cbf): reject invalid FIM geometry`).  It rejects non-finite,
non-symmetric, non-positive-definite, or excessively ill-conditioned FIM
geometry rather than adding regularization or a controller reserve.  The
evidence-producing source anchor,
`b80994189a9f5eba8078e8a512d836941233990f`
(`feat(diagnostics): add fixed geometry gate`), is a descendant of that guard
commit and adds the provenance-isolated `RG` overlay.  At launch and completion
the branch was `codex/cbf2026-diagnostic`; the only worktree entry was the
intentionally untracked `build-diagnostic/`, with no tracked source,
configuration, or documentation change.

The exact command was:

```sh
conda run -n cbf_env python -m scripts.diagnostics.run_diagnostic \
  --case RG --horizon 20 --seed 20260727 \
  --binary build-diagnostic/Swarm \
  --output-root /private/tmp/cbf2026-results/mc-first-fixed-geometry-smoke
```

The manifest records Gurobi, seed `20260727`, horizon `20.0 s`, return code
zero, and `termination_reason=completed`.  The evidence-producing binary is
`1,121,992` bytes.  The retained bundle is:

```text
/private/tmp/cbf2026-results/mc-first-fixed-geometry-smoke/RG/20260728T085831.702834Z_bb20af664f474ca5bef7c61abd7d658a
```

| Artifact | SHA-256 |
| --- | --- |
| Evidence-producing binary `build-diagnostic/Swarm` | `19d76c88e4f777f3648e259807a90ace637d238926a0e391f966544fc2c1acc7` |
| `config/diagnostics/rg_fixed_geometry.json` | `997b43f4482dddf23401ed2bc32e827f47c9aed85e1b9a6ccd67358a85de678d` |
| Evidence-time `scripts/diagnostics/run_diagnostic.py` at `b809941` | `290186581b532230b4b40149de707918f0b6bbe778444d0624fb66c393d7a27a` |
| `config.materialized.json` | `25e048b17dd9a13865abdba6c78fbf988f12057984a6f85235d05e0561bd3685` |
| `source-snapshot.tar.gz` | `d7e24f17b7372dc4ab1f7b4ee9bc8761eea6cde5e450899a6ce799651ed4d26d` |
| Raw `2026-07-28_16-58-32_RG_seed_20260727_20s/data.json` | `7fd2db4e0361c0692f617c6f9d29bdf5030d8b12935977532dc5e7dcd1edcf9d` |
| `diagnostic-summary.json` | `7946a4dc815d1945a49f2bfb9a447c0224f76aee352c206d3356d3fa84082045` |
| `diagnostic-summary.md` | `606e7089dca9bf2c1d1e7fa63dddb90ffd18c0da9ef1d46123372dfd47c949de` |
| `manifest.json` | `3576a77b7f4850c24ad585bc0df19eeb2e6833003c3742e21ffe2132429fcb1a` |
| `stdout.log` | `6ba34bad2d761f39609582c337439c53a8e4ac0996d80d62fe2affc7feeca475` |
| Empty `stderr.log` | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| Preserved dynamic-`R` `analysis.json` (identity check only) | `0e4f19dcd15ad222403a0d7cf1addd32f8d9e590f2c9c5e67439bf482b69e80d` |

The source snapshot uses policy `cbf2026-source-snapshot-v1` and binds the run
to the evidence-producing tree rather than to later `RGP` runner changes.
The preserved dynamic-`R` hash was independently reverified, but that run is
not treated as a causal comparator because `RG` uses a different geometry.

## Gate results

| Requirement | Observed | Decision |
| --- | ---: | --- |
| Frames | 40, spanning 0.0--19.5 s | PASS |
| Applied optimal solves / unapplied solves | 560 / 0 | PASS |
| Finite-value failures | 0 | PASS |
| Hard rows / negative residuals / logged mismatches | 1,680 / 0 / 0 | PASS |
| Hard residual minimum | `1.7763568394002505e-13` | PASS |
| Information-set records / transitions / required-reference mismatches / malformed | 560 / 0 / 0 / 0 | PASS |
| Localization observations below `-0.5 m` / consecutive negative frames | 0 / false | PASS |
| Strict nominal / tightened localization minimum | `395.4976523027863 / 380.3848494398347 m` | PASS |
| Collision observations below `-0.5 m` / consecutive negative frames | 1 / false | FAIL |
| Strict nominal / tightened all-pair collision minimum | `29.78340526525514 / -0.6566649907195625 m` | FAIL |

The sole below-threshold record is pair `robots:12-14` at frame 20
(`t=10.0 s`), with
\[
h_{12,14}=-0.6566649907195625\ {\rm m}.
\]
It exceeds the allowed negative tolerance in magnitude by
`0.1566649907195625 m`.  The physical separation is
`65.6340371233 m`, so the nominal collision margin remains
`55.6340371233 m`; this is an uncertainty-tightened robust-margin event, not a
physical near-collision.

Other descriptive results are limited to this smoke: 560 uncertainty records,
maximum scalar uncertainty `47.572322845005694 m`, maximum positive logged
rate `10.839003474328095 m/s`, and maximum positive one-step uncertainty jump
`5.4195017371640475 m`.  The maximum minimum-required planar
\(L_\infty\) value is `25.852487208478756 m/s`; applied-control
\(L_\infty\) reaches `34.29906065540013 m/s`.  Because input limits are
disabled in `RG`, these values are diagnostic and do not establish bounded
input feasibility.  Coverage is `9,820/90,000 = 0.10911111111111112`.

## Frame 19-to-20 mechanism

The corrected raw-data audit reconstructs the only event over
`t=9.5--10.0 s`:

| Quantity | Frame 19 | Frame 20 | Change |
| --- | ---: | ---: | ---: |
| Pair 12--14 distance | `70.6792608565 m` | `65.6340371233 m` | `-5.0452237332 m` |
| \(\epsilon_{12}\) | `23.8088936460 m` | `24.7542130365 m` | `+0.9453193906 m` |
| \(\epsilon_{14}\) | `26.1169873403 m` | `31.5364890775 m` | `+5.4195017372 m` |
| \(h=d-10-\epsilon_{12}-\epsilon_{14}\) | `+10.7533798702 m` | `-0.6566649907 m` | `-11.4100448610 m` |

The frame-19 backward rates predict `4.6722770982 m` of combined uncertainty
growth over the next 0.5 s, while the realized growth is
`6.3648211277 m`; the rate term therefore underpredicts the one-step growth by
`1.6925440295 m`.

Minimum-pair switching leaves the same interval exposed to the distributed
held-command contract.  Robot 12 alternates its selected minimum-safety pair
between robots 14 and 10 over frames 16--23.  At frame 19 its margins are
\(h_{12,10}=2.6034289559\) and \(h_{12,14}=10.7533798702\), so robot 12
constrains pair 12--10 rather than 12--14.  Robot 14 does constrain 14--12;
its `safetyCBF(#12)` residual is `1.07e-10`.

That row uses robot 12's held frame-18 command
`(-6.2173272837, 20.7706409137)` with radial projection
`2.8575164154 m/s`.  Robot 12 instead applies the new frame-19 command
`(9.7087494685, 30.1279452024)` with radial projection
`21.2210984804 m/s`.  The projected stale-to-applied mismatch is therefore
`18.3635820651 m/s`, or approximately `9.1818 m` over the 0.5 s hold.
Logged position increments equal the frame-19 commands times 0.5, excluding a
log-index explanation.  At frame 20 both robots select each other and solve
optimally; the margin recovers to `+12.8592679042 m` at frame 21.

This evidence supports the narrow mechanism description: a one-step
distributed zero-order-hold prediction mismatch, minimum-pair switching, and
underpredicted uncertainty growth jointly produce the isolated robust-margin
event.  It is not a QP-row residual failure.

## Geometry-sign consistency

`Robot.hpp` defines
\[
\texttt{partId}=(\texttt{id}-1)/\texttt{numSquad}+1,
\]
so `partId` and squad index are one-based.  The implementation uses
\(-\pi/3\) for squad 1 and \(+\pi/3\) for squad 2, and `RG` uses those same
signs.  The manuscript expression
\[
\theta_k=(-1)^{k+1}\pi/3
\]
gives the opposite signs for one-based \(k\).  The implementation-consistent
expression is
\[
\theta_k=(-1)^k\pi/3,
\]
unless the manuscript explicitly redefines its orientation convention.
Thus this is a manuscript-to-implementation sign error, not an `RG`
provenance error.  Flipping `RG` after seeing the result would create a new
geometry variant and is not justified as a mechanical conformance correction.

## Disk, evidence boundary, and next gate

The direct preflight recorded `10,245,804,032` free bytes and
`314,138,624` retained-result bytes, satisfying the 8 GB start and 2 GB cache
guards.  The smoke manifest records `10,245,365,760 / 10,241,908,736` free
bytes before/after and `3,153,920` bytes for both output-root and run
allocation, below the `2,000,000,000 / 250,000,000` byte caps.  After analysis
and at finalization, free space was `10,236,579,840` bytes and retained
results occupied `319,524,864` bytes.  The 6 GB live floor was not approached.
No evidence was deleted.

The planned root
`/private/tmp/cbf2026-results/mc-first-fixed-geometry-250s` is absent.
Accordingly there is no 250 s bundle, no 500-frame/7,000-solve/21,000-row
result, and no long-horizon fixed-geometry claim.

This single-seed 20 s gate does not establish a causal `RG`-versus-`R` effect,
strict continuous-time or sampled-data forward invariance, bounded-input joint
feasibility, pairwise-row completeness, estimator calibration, Monte Carlo
robustness, or generalization across geometries.  `RG` uses the `minimum`
safety mode and disabled input limits; nonnegative instantaneous hard-row
residuals therefore cannot certify every pair over the following hold
interval.  The practical threshold remains `-0.5 m`, the observed geometry is
retained, and the physical separation does not reclassify the failed robust
margin.

The next evidence-producing experiment was pre-registered as a matched 20 s
`RGP` run: the same verified controller binary, seed, fixed geometry, and
uncertainty semantics, changing only the collision-row mode from `minimum` to
explicit pairwise.  The runner/config source snapshot is necessarily
provenance-distinct because it adds the `RGP` case.  The unique execution and
its stop decision are recorded below.

## RGP focused pairwise gate

### Decision and execution lifecycle

The unique 20 s `RGP` launch did not complete its requested horizon.  At frame
3 (`t=1.5 s`), robot 11 returned `inf_or_unbounded`; the distributed
optimization loop aborted before applying any frame-3 control or advancing the
state.  The simulator returned one and the runner recorded
`termination_reason=simulator_nonzero_exit`.  The evidence was preserved
without a rerun, and no 250 s run was launched.  No threshold, geometry,
uncertainty reserve, or controller parameter was changed after observing the
failure.

The corrected analyzer at commit `719252a`
(`fix(diagnostics): classify aborted frame evidence`) separates the three
confirmed-applied frames from the terminal partial frame:

| Evidence class | Observed | Interpretation |
| --- | ---: | --- |
| Confirmed-applied frames / controls | 3 / 42 | Frames 0--2 only |
| Applied hard rows / negative rows | 630 / 0 | Minimum residual `1.7497114868092467e-13` |
| Applied pairwise rows | 546 / 546 | Complete directed all-pair coverage |
| Fresh optimal but unapplied records / hard rows | 10 / 150 | Robots 1--10 in frame 3; all 150 logged residuals valid |
| Failed record / hard rows | 1 / 15 | Robot 11; one result placeholder, not applied evidence |
| Not-attempted records | 3 | Robots 12--14; logged solver values are stale |
| Fresh partial-frame pairwise rows | 143 / 143 | Complete for the 11 fresh attempts, but unapplied |
| Information-set records / transitions / mismatches / malformed | 56 / 0 / 0 / 0 | Includes the terminal diagnostic snapshot |

The applied trajectory prefix retained positive state margins: the strict
all-pair uncertainty-tightened collision minimum was
`+6.070647948346206 m`.  Thus `RGP` stopped because the next local hard
constraint set was infeasible, not because an applied robust margin or applied
hard-row residual had already failed.  The partial frame is diagnostic only;
its fresh optimal solutions were never applied, robot 11 has no feasible
result, and the later three records do not represent new solve attempts.

### Local infeasibility certificate

At the terminal state, robot 11 lies between robots 9 and 13 in an almost
collinear arrangement.  Its pairwise local QP freezes the neighbors'
previously communicated commands and imposes, among the other rows, two nearly
opposed half-spaces.  In radial form, the robot-9 row requires approximately
\[
(0.894087807,\ 0.447891721)^\top u_{11}
\geq 11.611159469,
\]
whereas the robot-13 row gives an upper requirement of approximately
`5.699770095` along the nearly same positive normal.  Their normals differ
from exact antiparallel alignment by only `0.3347008506` degrees.

The robot-9 and robot-13 rows alone still intersect, but only at an extreme
command: their least feasible planar \(L_\infty\) norm is
`899.538788`, with intersection approximately
\((463.609,-899.539)\).  Adding any one of the 11 remaining safety rows
present in the local problem produces a minimal infeasible triple with those
two rows.  In particular, rows 9, 13, and 10 admit the nonnegative Farkas
weights
\[
(0.497735042,\ 0.499064375,\ 0.003200583).
\]
The weighted velocity coefficient cancels while the weighted right-hand side
is `+3.018300109`, yielding the contradiction \(0\geq3.018300109\).
Removing the robot-9 or robot-13 row restores feasibility with least
\(L_\infty\) norms `25.2168` and `26.1272`, respectively; removing any other
single row does not restore feasibility.  This local half-space contradiction,
rather than an information/FIM invalidity, explains the solver status.

### Coupled-system counter-check

The same frame does not make the corresponding globally coupled hard CBF
system infeasible.  Substituting the explicit stacked witness
\[
v_i=0.2\left(p_i-p_{\mathrm{global\ centroid}}\right)
\]
into the complete 210-row hard-constraint system gives minimum residual
`+3.947720614`.  More generally, the scalar family
\(v_i=c(p_i-p_{\mathrm{global\ centroid}})\) is feasible for
\[
c\in[0.116870760,\ 0.380089824].
\]
The upper bound is set by robot 9's robot-to-base-1 `fixedCommCBF` row.
Input limits are disabled in this gate, so this witness establishes
unbounded-input feasibility of the stacked constraint system only; it is not
a bounded-input feasibility claim.

Consequently, `pairwise` local mode is not a drop-in repair for the
minimum-row `RG` controller.  The failure does not negate a theorem stated for
the coupled stacked CBF inequalities.  It does show that independent local
QPs using frozen neighbor commands are not automatically equivalent to that
stacked system, even when every local problem receives all pairwise rows.
Before another simulation, the manuscript and controller contract should
distinguish joint feasibility from local feasibility and choose a compatible
architecture, such as a coupled solve or a provably feasible reciprocal
allocation.  The evidence does not justify tuning the tolerance, geometry, or
uncertainty reserve, and no new run is authorized by this gate.

### RGP provenance, hashes, and disk guard

The exact launch used seed `20260727`, horizon `20.0 s`, Gurobi, the
provenance-distinct `RGP` materialization, and the same verified controller
binary as `RG`:

```sh
conda run -n cbf_env python -m scripts.diagnostics.run_diagnostic \
  --case RGP --horizon 20 --seed 20260727 \
  --binary build-diagnostic/Swarm \
  --output-root /private/tmp/cbf2026-results/mc-first-fixed-geometry-pairwise-smoke
```

The retained bundle is:

```text
/private/tmp/cbf2026-results/mc-first-fixed-geometry-pairwise-smoke/RGP/20260728T093412.839385Z_7f3c93231d394700af4c9c4bc72c0872
```

| Artifact | SHA-256 |
| --- | --- |
| Evidence-producing binary `build-diagnostic/Swarm` | `19d76c88e4f777f3648e259807a90ace637d238926a0e391f966544fc2c1acc7` |
| `manifest.json` | `ed333427859ea623525863f7df3fb9bed1d26a30f1e9e83a2e7ecf6af887d6c7` |
| Raw `2026-07-28_17-34-13_RGP_seed_20260727_20s/data.json` | `30858f94983d4029696b4e6f55320d86fa3915ec7577d70caef2bc2035aee7bc` |
| `config.materialized.json` | `a078fc41f28d451eda7db042968e72e8ece91702ff237f549a3dc71791a6f9d8` |
| `source-snapshot.tar.gz` | `a3c408a2e5f7d1e0b20078bd74455f7350e13ddc453d39e12a80a97eab001896` |
| `stdout.log` | `02537a379c3d20eadffd0cb3d47fa78039b981f7b816da7b28bd557964388a62` |
| `stderr.log` | `a535fdfd0afe2606e1782703aba3543de716f4e538e8f588d189559b0534d63e` |
| Analyzer-`719252a` `diagnostic-summary.json` | `074cef6ca0f0be2dc2c18e0ca9e1629431fddeff3df92221408d91df327f37a2` |
| Analyzer-`719252a` `diagnostic-summary.md` | `d3a7351b43efa770621f80d0bb95cdc141651edceac817d404db99fd6c167d06` |

The manifest anchors the trajectory to base commit
`f27d4a3dc2590c6f186bfdcd4521575b25f5412a`, branch
`codex/cbf2026-diagnostic`, and source-snapshot policy
`cbf2026-source-snapshot-v1`; the regenerated summaries change only the
analysis of the preserved raw evidence.  Free space was
`9,858,990,080 / 9,856,696,320` bytes before/after the run, and the run and
output-root allocation were both `2,183,168` bytes.  These values satisfy the
8 GB launch guard, 6 GB live floor, 2 GB retained-cache cap, and 250 MB
per-run cap.  The final workspace-wide disk reading is intentionally recorded
separately because later analysis and documentation activity is not part of
this run's manifest.
