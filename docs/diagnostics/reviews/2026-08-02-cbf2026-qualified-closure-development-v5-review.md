# Independent review — qualified-closure development-v5 terminal report

## Verdict

`C0 / I0 / M0 — VERIFIED FAIL REPORT`

The reviewed author report is evidence-consistent.  It correctly classifies
development-v5 as a terminal failed development lifecycle, not as positive
simulation evidence.  It must not be used for a positive paper claim about
long-horizon performance, fixed-FIM comparison, localization-error
distribution, or implementation-level distributed-QP safety.  The appropriate
next lifecycle is a newly registered v6 lifecycle; v5 is not retryable or
qualifying.

No C- or I-severity discrepancy was found.  This review itself made no change
to the report, campaign roots, source, tests, protocol, DRA, or paper.

## Scope and method

This was a read-only forensic review at Git commit
`9f9c8352cdd3dd32cf6c907b0e8b7f592559ce46`.  I independently checked:

1. Git identity and the exact-four authorization topology with `rev-parse`,
   `show`, and `diff-tree`.
2. SHA-256 and byte identities of the four registered additions, the live
   binary/configuration bindings, and both retained output trees.  Tree hashes
   were recomputed over sorted `relative-path<TAB>bytes<TAB>sha256<LF>` rows.
3. Raw/analysis JSON manifests, all gzip JSONL record counts, mission terminal
   states, and the frame-1 abort schema.
4. A non-campaign one-step reconstruction from retained initialization rows
   and frame-0 commands, using the repository's frozen initial-state formulas
   and the `SingleIntegrate2D` update rule.  No runner or analyzer was started.
5. Fixed CBF registry versus dynamic-FIM reference selection in the retained
   configuration and source, plus an in-memory class-K coefficient sweep at
   the retained one-step state.
6. The runner/analyzer failed-prefix complement predicates which explain the
   independent analyzer exception.

## Identity and tree checks

The HEAD tree is `1dbf14f4b39b13177eaf6439a80d3928f0d825e5`, with sole
parent `55a9a783a94a1d59a60db99552a6a4421acca25b` (tree
`1e7dacbeadf96fd00353e198daed4110ef7304e9`).  The child adds exactly four
paths and no fifth path:

| Item | SHA-256 | Bytes |
|---|---|---:|
| protocol JSON | `a821e6d2abbc9a85a088892fb33413387e8233b6d67646ab346e64d5698f8123` | 47,677 |
| protocol Markdown | `91d7cc6a01c53b7ac2627b9318dd75d7328eb401cf70c25869916715823269cf` | 312 |
| independent preflight | `fe2426d850a66f7ed086b771d7ba58b3651b473262e8977f29d437c6a029c380` | 13,900 |
| authorization JSON | `0e92fef88049807e00e925fd52844645c78f6e9d6fb342968640ae5408853bd1` | 565 |

The registered branch has no upstream.  The authorization's exact user text
hash, protocol semantic hash, binary/configuration hashes, frozen-family hash,
and frozen 100-seed admission summary all match the report and the retained
protocol/identity records.

The raw root has 28 regular files, no symbolic links, 12,764,382 logical
bytes, and tree commitment
`7ead4049e0d13f63458ba8d1a522f055ffed56357ffa1719e1bfc1da38a66870`.
Its terminal manifest is 1,516 bytes with SHA-256
`a3970bcf588e938f8487a794b00d1bbbe8bbd181c0d5a919cc0297b258e64a32`.
The analysis root has precisely one regular file, no symbolic links, 176
bytes, and tree commitment
`e42ce3e1cd83c60366b2901e3adec58a4282cfc70df7f9b13609a16f600020f5`;
its manifest SHA-256 is
`82457d6a4838b2396b16fa36d99b15b6ccd0ab16823cfbf486be51613e862a63`.

## Raw evidence and terminal classification

Mission-01's gzip stream contains exactly 250 rows:

| Record type | Count |
|---|---:|
| initialization | 14 |
| reset | 1 |
| endpoint_row | 232 |
| controller_interval | 2 |
| mission_terminal | 1 |

The reset contains exactly 119 hard edges: 28 localization and 91 collision.
All 14 reset local hard QPs have `status=optimal`.  The supervisor manifest
reports `valid_rows=250`, `missing_rows=0`, `returncode=1`, and
`reason=child_nonzero_exit`; stderr ends in the reported continuous-flow
infeasible-hard-QP error.  The completed frame-0 interval has maximum planar
component `24.18197080343183` and local applied-row residual minimum
`7.631673071273326e-13`.  Frame 1 is `complete:false`, gives the same abort
reason, and has 14 `not-attempted` nodes.

All ten mission manifests are terminal failed with
`child_nonzero_exit`; only Mission-01 has retained Swarm evidence.  The
synthetic missing streams have the exact expected retained-prefix complements:
Mission-01 omits only the completed frame-0 controller and its 119
reconstructed keys, while Missions 02--10 contain their complete unlaunched
universes.  This supports the author's no-retry/failed-closure conclusion.

## Independent one-step and graph checks

Using the 14 retained initialization positions and the retained frame-0
commands under `X += (f + g u) * 0.5`, I independently obtained:

| State | Minimum robust barrier | Bounded max-min local hard-QP result |
|---|---:|---|
| t=0 | 35.955406543965935 m, collision (2,14) | +0.862613426899051 m/s, UAV 13 |
| t=0.5 | 37.24931436014771 m, collision (1,7) | UAV 4: -0.017308452413389; UAV 12: -0.044958268439302 m/s |

Thus the primary failure is a loss of bounded local hard-QP feasibility with
positive robust barriers, rather than a negative barrier or a solver-only
artifact.  The tight conflicts reproduce as `(4,10)` with `(4,13)`, and
`(6,12)` with `(12,13)`.

The dynamic lower-index FIM reference set of every one of the 14 UAVs is
unchanged over this first interval.  The failing QPs nevertheless rebuild
FIM-derived radii/rate bounds at the new positions.  The hard problem still
uses the fixed formation localization registry plus all-pairs collision
edges; dynamic FIM references are not promoted to CBF distance edges.  This
confirms both the report's fixed-versus-dynamic graph distinction and its
conclusion that no graph-switch reset caused the failure.

The independently repeated static coefficient sweep at the same retained
one-step state gives worst max-min margins:

| class-K coefficient | Worst margin (m/s) | UAV |
|---:|---:|---:|
| 0.100 | -0.044958268439302 | 12 |
| 0.105 | +0.081613870967155 | 12 |
| 0.150 | +1.220763125625275 | 12 |
| 0.200 | +2.486484519689854 | 12 |

This supports the stated *static diagnostic* boundary only.  It cannot be
treated as campaign evidence because changing the coefficient changes the
frame-0 task command and hence the trajectory.

## Analyzer failure check

The report's analyzer explanation is exact.  The runner counts controller and
reconstructed keys as retained only when `runtime.complete is true`; the
analyzer's failed-prefix reconstruction counted the frame-1 incomplete abort
row unconditionally.  The synthetic stream therefore correctly begins its
missing-controller suffix at frame 1, while the analyzer incorrectly expected
frame 2, causing `synthetic missing key universe differs`.  The proposed
shared `complete` predicate and a failed-prefix regression test are the
minimal correction.  Repairing that predicate can explain or improve a future
lifecycle, but cannot turn the consumed v5 campaign into qualifying evidence.

## Scientific boundary

The report's negative-evidence boundary is warranted: no fixed-FIM replay,
later-frame result, mission-success statistic, or positive containment/error
evidence exists in these terminal roots.  Missing later rows are not success.
Before a separate v6 registration, the paper must not describe the failed
distributed local-QP implementation as a demonstrated recursive-feasibility
or safety guarantee.
