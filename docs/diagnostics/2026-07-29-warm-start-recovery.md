# Warm-start recovery Gate 2 evidence

## Decision

The registered paired intervention completed once and was not retried.
On the frozen trajectory and the 20 paired range-noise seeds,
the deployment restart policy removed the registered direct initialization
failure event and sharply reduced observed downstream unavailability.
The confirmatory primary gate and hierarchical secondary gate both passed.

The dynamic calibration safeguards did not pass.
Conditional coefficient-3 epsilon containment fell by
`2.151505664044029` percentage points,
slightly beyond the registered 2-point limit,
and the conditional q-above-9 rate rose by
`2.936542719257862` percentage points,
beyond the same limit.
Therefore `advance_to_multi_trajectory == false`.
This run does not authorize a `main.tex` change.

## Frozen protocol and implementation

The prospective run registration is
[`2026-07-29-cbf2026-warm-start-recovery-run.md`](../superpowers/plans/2026-07-29-cbf2026-warm-start-recovery-run.md).
Its independent code/protocol review is
[`2026-07-29-warm-start-recovery-code-protocol-review.md`](reviews/2026-07-29-warm-start-recovery-code-protocol-review.md)
and returned `APPROVED`, with zero open Critical or Important findings.

The production run used source commit
`12ca41cd9f403bef2182e57e980329829c16c4bc`
on `codex/cbf2026-diagnostic`.
The principal implementation and registration history is:

| Role | Commit |
| --- | --- |
| Intervention design and execution plan | `03cd23e` |
| Explicit restart policy | `71533f9` |
| Paired supervisor and hardening | `401647b`, `6cbbeff` |
| Paired comparator and hardening | `f37a3c5`, `f55a78d`, `7f68e12`, `6646a6b` |
| Frozen registration and trust-root repair | `66b6e96`, `f33035e`, `9fde377` |
| Independent code/protocol review | `12ca41c` |

The estimator remained
`variable_weight_nls_full_residual_jacobian_v1`.
Both policies used the same trajectory,
20 seeds `20260727` through `20260746`,
500 frames,
dynamic and fixed graph cases,
reference memberships,
measurements,
WNLS,
FIM,
and coefficient-3 epsilon construction.
Only pre-first-acquisition initialization selection differed.

## Preflight and exactly-once execution

The exact supervisor preflight exited `0`.
The native suite passed all 219 tests,
Python byte-compilation and `git diff --check` passed,
all eight frozen hashes matched,
the full Git status contained only untracked paths under
`build-diagnostic/`,
and both production roots were absent.
The final supervisor preflight observed `9,348,048` KiB available,
above the 8 GB launch gate.

Before this preflight,
the separately authorized rebuildable cache
`/Users/xirhxq/.cache/codex-runtimes`
was removed.
It was recreated as an empty directory so the frozen `du -sk` command could
run unchanged.
No evidence,
Git object,
source,
document,
or unrelated cache was removed.

The one supervisor invocation completed both policies:

```text
/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b
```

Its parent manifest SHA-256,
computed once for the registered comparator handoff,
is:

```text
c110275910c3e44ecf97c66ba7346fbb0b340cfb844d050c0ecf56944e5ff55a
```

The parent reports:

| Field | Value |
| --- | ---: |
| Parent schema | `cbf2026-warm-start-recovery-parent-v1` |
| Termination reason | `completed` |
| Parent allocated bytes | 134,279,168 |
| Output-root allocated bytes | 134,279,168 |
| Free bytes before | 9,568,464,896 |
| Minimum live free bytes | 7,363,014,656 |
| Free bytes after | 72,883,662,848 |

The strict child process SHA-256 is
`e84e43c332aba21664b63f52281416d47c21be684d2b5b83e26edca233efe67b`;
the restart child process SHA-256 is
`c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003`.
The source snapshot SHA-256 is
`4ac5ca7d3608dcce4c54574ae33dd04b0683069e22bd344a34264d3dda3ef5cf`.

The exact comparator preflight then exited `0`.
All 219 native tests,
byte-compilation,
Git and hash checks passed again.
The paired parent measured `131,132` KiB,
the comparison root was absent,
and `71,156,624` KiB was available.
The parent manifest was not externally hashed a second time.

The one comparator invocation completed and published exactly:

```text
/private/tmp/cbf2026-warm-start-recovery-analysis/registered-gate-2/
├── warm-start-recovery-comparison.json
└── warm-start-recovery-comparison.md
```

| Comparison artifact | Content bytes | SHA-256 |
| --- | ---: | --- |
| `warm-start-recovery-comparison.json` | 51,767 | `ef8014915b3cced6a75e094b3d16f01f17b5f881c6357dff6b885b6e871498ca` |
| `warm-start-recovery-comparison.md` | 403 | `e10ba2c137a5edc229e9fa9492b1992d0651f38d8df9067c47b295b4e8d5cc33` |

The complete comparison directory occupied 56 KiB,
below the 10 MB cap.
At the post-run probe,
the replay root occupied 131,132 KiB,
the analysis root occupied 56 KiB,
the rebuildable cache occupied 0 KiB,
and `/private/tmp` had 71,153,936 KiB available.

## Paired integrity

The comparator reports:

- all 280,000 strict rows equal the immutable baseline after removing only
  `initialization_policy`,
  `initial_estimate_source`,
  and `ever_acquired_finite_before_attempt`;
- exact strict/restart key order,
  truth,
  active-reference,
  and registered measurement-noise equality for 280,000 paired rows;
- unchanged child hashes;
- identical 20-seed sets and `139,720` dynamic primary denominators; and
- reconciled restart provenance across all 280,000 restart rows.

The restart policy made 476 pre-first-acquisition restart selections:
236 in `dynamic_dag_wnls` and 240 in `fixed_refs_wnls`.
Across both cases these produced 248 convergences,
80 maximum-iteration failures,
and 148 upstream-unavailable short circuits.

## Confirmatory and hierarchical results

The statistical unit is one paired range-noise seed.
All intervals use 10,000 percentile-bootstrap resamples,
RNG seed `20260729`,
and 20 paired-seed draws per resample.

| Dynamic event | Strict | Restart | Count reduction | Paired 95% interval |
| --- | ---: | ---: | ---: | --- |
| Exact direct initialization failure | 15,469 / 139,720 | 0 / 139,720 | 100% | `[-0.1285714285714286, -0.08928571428571427]` |
| Upstream-unavailable secondary | 50,898 / 139,720 | 57 / 139,720 | 99.888% | `[-0.47458613655883203, -0.25334239908388206]` |

The exact-direct interval upper endpoint is below zero and its aggregate
count reduction exceeds 90%,
so the primary gate passes.
Because the primary gate passes and the upstream interval upper endpoint is
also below zero,
the registered hierarchical cascade-interruption result passes.

The fixed-reference direct event,
reported descriptively only,
changed from `14,471 / 139,720` to `0 / 139,720`,
with interval
`[-0.12142857142857144, -0.0857142857142857]`.
It is not evidence of graph superiority.

## Calibration safeguards

| Dynamic conditional metric | Strict | Restart | Change | Registered decision |
| --- | ---: | ---: | ---: | --- |
| Coefficient-3 epsilon containment | 0.9324900808448435 | 0.9109750242044032 | −2.1515 pp | Fail |
| q above 9 | 0.0892873124310134 | 0.11865273962359202 | +2.9365 pp | Fail |

The restart arm created many more finite localization estimates,
but the newly recovered conditional population is measurably harder:
dynamic converged attempts rose from 67,042 to 127,043,
while both prespecified calibration changes crossed their 2-point limits.
The registered decision is therefore not to advance directly to
multi-trajectory validation.

## Review status and paper boundary

The required fresh independent evidence audit is
[`2026-07-29-warm-start-recovery-evidence-review.md`](reviews/2026-07-29-warm-start-recovery-evidence-review.md).
It returned `APPROVED / CLEAN`,
with zero Critical,
Important,
or Minor findings.
Its SHA-256 is
`cc7ab8027bb9cf3832d8956698d45db2d3abb218e3c2c856a9719e62368cfb93`.
The independent script read 840,000 raw process rows,
performed 280,000 baseline-to-strict and 280,000 strict-to-restart
comparisons,
and reproduced the hashes,
paired bootstrap intervals,
calibration statistics,
restart provenance,
publication shape,
and disk decisions without importing the production comparator.

the bounded supported statement is:
under the preserved trajectory and exact frozen paired range-noise seeds,
deployment restart removed the registered local initialization failure event
and reduced observed downstream unavailability.

This result does not validate coefficient-3 epsilon,
does not establish graph superiority,
and does not support controller,
collision-avoidance,
connectivity,
mission,
production-estimator,
geometry-generalization,
or cross-trajectory guarantees.
Because the registered dynamic calibration safeguards failed,
`main.tex` remains unchanged.
The next technical step is to diagnose whether recovered,
previously unestimable rows require an availability-aware localization-graph
policy or a more conservative uncertainty treatment before any broader
experiment or manuscript claim.
