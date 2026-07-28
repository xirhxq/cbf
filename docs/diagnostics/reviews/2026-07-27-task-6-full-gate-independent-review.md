# Task 6 Independent Full-Gate Evidence Review

Date: 2026-07-27

## Verdict

`APPROVED` for the documented stop-rule completion of the full gate.

There are no Critical or Important findings.  H0 and C1 are the only full
runs that should exist after this gate: C1's claim-aligned,
uncertainty-tightened required-link failure is sufficient to stop before U0.
This approval does not validate safety, a physical input limit, an estimator,
or a causal C1--U0 effect.

This was a read-only review.  I did not edit source or evidence, run a
simulation, test, or build, delete data, commit, or push.

## Evidence reviewed

- Task brief:
  `.superpowers/sdd/2026-07-27-cbf2026-minimal-diagnostic-implementation/task-6-brief.md`
- design and implementation gate criteria:
  `docs/plans/2026-07-27-cbf2026-minimal-diagnostic-design.md` and
  `docs/superpowers/plans/2026-07-27-cbf2026-minimal-diagnostic-implementation.md`
- approved Task 5 review and smoke decision:
  `.superpowers/sdd/2026-07-27-cbf2026-minimal-diagnostic-implementation/task-5-review.md` and
  `docs/diagnostics/2026-07-27-smoke-gate.md`
- full report and Task 6 report:
  `docs/diagnostics/2026-07-27-full-gate.md` and
  `.superpowers/sdd/2026-07-27-cbf2026-minimal-diagnostic-implementation/task-6-report.md`
- all retained H0/C1 manifests, materialized configurations, disk probes,
  logs, raw `data.json`, and diagnostic summaries below
  `/private/tmp/cbf2026-results/full`.

The worktree is `codex/cbf2026-diagnostic` at
`47e7b3d6f5b1be74a3dfa32150b4246c619e9a0d`, matching both manifests.
The binary SHA-256 recomputes to
`4ee6c5f07eebe2ac7ada95f592f1785bb213a90e3320022f1a6c569810e35221`.

## Artifact integrity and sequencing

I recomputed every retained artifact hash named in the full report.  The H0
and C1 manifest/config/data/summary/stdout/stderr/disk-probe hashes all match
the report.  In particular, the raw data hashes are respectively
`785d3f4c721a8cdbde80f49c67ea8f6322ddd3789adf8300a02161e122a14333`
and `72733f40e01ac951878f80a28e6109350d5767f4bbe4b239cc82cc02858c88dc`.
The report's raw byte counts (24,920,424 H0; 27,873,663 C1), allocated case
sizes (52,172 KiB; 65,116 KiB), and aggregate allocation (117,288 KiB) match
`stat` and `du -sk`.

Each executed case root has exactly one timestamped child and one raw data
file.  Its matching `stdout.log` has an `[OUTPUT_DIR]` line and reports
`After 500.0000 seconds`; both `stderr.log` files are zero bytes.  The H0
summary was produced at 21:44:34 local time, before C1's materialized
configuration at 21:47:38; C1's summary was produced at 21:49:09--10, before
the C1 external post-analysis disk probe at 21:50:30.  This is consistent
with sequential launch and analysis before the next decision.

Both manifests and retained configuration/data/summary/probe JSON parse
strictly.  H0 and C1 manifests record Gurobi, seed `20260727`, horizon
`500.0`, return code `0`, and termination `completed`; their branch, commit,
and binary/config hashes agree with the report.  The two disk-probe files
exactly reproduce the reported preflight, runner-before/after, and external
post-analysis values.  Every launched preflight exceeds 8,000,000,000 bytes
and every runner-after value exceeds 6,000,000,000 bytes; neither manifest
reports `disk_hard_floor`.

`/private/tmp/cbf2026-results/full/U0` does not exist.  There is no U0 raw
file, manifest, configuration, log, or timestamped directory to misidentify
as a run.

## Independent metric checks

I directly re-derived the following from both raw trajectories, using the
materialized `max-range`, `safe-distance`, bases, logged formation references,
logged covariance-derived uncertainty, and logged optimal controls:

| Check | H0 | C1 |
|---|---:|---:|
| Frames / logged interval | 1,000 / 0.0--499.5 s | 1,000 / 0.0--499.5 s |
| Unique coverage cells | 90,000 | 90,000 |
| Optimal / applied / unapplied records | 14,000 / 14,000 / 0 | 14,000 / 14,000 / 0 |
| Hard records / recomputed minimum / below `-1e-7` | 28,000 / about `5.4e-14` / 0 | 42,000 / about `4.4e-14` / 0 |
| Required-link nominal / tightened minimum (m) | 2.402552 / -6.090116 | 1.604319 / -6.044873 |
| All-pair nominal / tightened minimum (m) | -8.746495 / -16.357186 | 4.857228 / -2.649090 |
| Maximum actual L2 / Linf | 43.673360 / 41.601259 | 43.345760 / 41.320156 |
| Maximum uncertainty / positive one-step rate | 30.607526 / 6.760209 | 30.388526 / 6.774165 |

The retained summaries also support the reported maximum per-frame
minimum-Linf hard-set bounds (27.124387 H0 and 26.852149 C1), with zero
infeasible logged hard-constraint frames.  Their scope is correctly limited
to logged, applied no-slack hard constraints; it is not proof of compatibility
with any physical vehicle limit.  All 14,000 covariance-to-uncertainty
records per executed case match (maximum absolute error
`7.105427357601002e-15`), and the class-K summaries have zero missing,
mismatched, or unmapped records.  The raw records also support zero non-finite
values, zero solver failures, and zero un-applied results.

The C1 attributions reproduce exactly.  At 95.0 s, UAV 6's required reference
UAV 4 has distance 832.0827298271114 m, nominal margin
17.917270172888607 m, uncertainty-tightened margin
-4.397216567183898 m, and its explicit `fixedCommCBF(#4)` logged hard-row
residual remains positive at approximately `8.51e-12`.  The worst required
link is UAV 7 referenced to UAV 6 at 171.5 s (-6.04487325815715 m).  At
212.0 s, UAVs 4 and 12 have nominal separation margin 16.345226181570816 m
but tightened margin -2.6490902654634922 m; this is both first and worst
pairwise tightened finding.

H0's first tightened pairwise breach is also reproduced at 2.0 s, UAVs 12--13
(-1.237156130043356 m), and its first nominal breach at 2.5 s.  Its safety
CBF is demonstrably off in the materialized configuration.  The report
correctly treats this as unsafe historical/provenance evidence, not a safety
claim and not a reason to prevent C1.

## Gate decision and interpretation

The approved smoke review authorized sequential H0/C1/U0 runs, but the Task 6
brief makes a C1 required-link or safety-margin breach a full-gate failure and
requires a stop before the next case.  C1 has both the direct required-link
breach and a supporting all-pair tightened breach.  Therefore not launching U0
was the required, rather than omitted, decision.  A U0 run would have violated
the stated stop rule; its absence rules out any long-horizon C1--U0 causal
comparison.

`time_varying_uncertainty_gate_failed` is the most fitting one of the four
permitted empirical-outcome labels, provided it retains the report's existing
qualification.  It is not an `implementation_gate_failed`: artifact validity,
finite values, solver application, residual checks, and covariance conversion
all pass.  It is not a `bounded_input_feasibility_gate_failed`: the logged
unbounded hard sets have finite minimum-Linf values, while no defensible
physical bound was selected or enforced.

The label must not be read as a uniquely identified causal mechanism.  The
raw C1 evidence establishes a failure of snapshot uncertainty-tightened
geometric conditions despite instantaneous logged hard-row satisfaction, and
the recorded uncertainty grows positively.  It does not separate the absence
of an uncertainty-rate term from 0.5-s sampled-data evolution.  Nor can the
all-pair result alone establish a missing QP row, because the implementation
uses a minimum safety barrier per UAV.  The full report makes these limits
explicit, so its phrasing is adequately bounded.

Static inspection corroborates the structural claims: Gurobi initializes the
control variables with infinite lower/upper bounds while applying only slack
nonnegativity (`include/optimisers/Gurobi.hpp`); `Robot::setFixedCommCBF` and
`Robot::setSafetyCBF` use snapshot uncertainty without a rate term
(`include/Robot.hpp`); and the safety barrier takes the minimum over other
UAVs rather than adding a separate safety row for each pair.  The raw logs
contain state and covariance but no distinct truth/estimate channel.  The
full report's next-step recommendation--sampled-data/uncertainty-rate reserve,
explicit bounded joint feasibility, explicit pairwise constraints for a
pairwise claim, and estimator-bound framing--does not exceed these facts.

## Findings

### Critical

None.

### Important

None.

### Minor

None.

## Review conclusion

The retained evidence warrants the full report's claim-limited failure and
the mandatory stop before U0.  It does not warrant a safety, bounded-input,
estimator, or counterfactual-ablation validation claim.  Theory and
implementation repair before any new smoke/full sequence is the evidence-
bounded next step.
