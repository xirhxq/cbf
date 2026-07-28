# Task 5 Independent Smoke-Gate Review

Date: 2026-07-27

## Verdict

`APPROVED` for the stated next action: one *sequential* 500 s run of H0, C1,
and U0, with the existing 8,000,000,000-byte launch gate, 6,000,000,000-byte
live floor, and analysis of each actual output before the next launch.

There are no Critical or Important findings. H0 is approved only as the
specified unsafe historical/provenance baseline; it cannot be used for a safety
claim. C1 and U0 are approved only for the claim-limited, single-run
full-horizon diagnostic. This decision does not establish long-horizon safety,
a statistical effect, or a performance claim.

This was a read-only evidence review. No simulation, test, build, delete,
commit, or push was performed.

## Evidence examined

- Task brief:
  `.superpowers/sdd/2026-07-27-cbf2026-minimal-diagnostic-implementation/task-5-brief.md`
- Smoke report: `docs/diagnostics/2026-07-27-smoke-gate.md`
- Task report:
  `.superpowers/sdd/2026-07-27-cbf2026-minimal-diagnostic-implementation/task-5-report.md`
- Retained case roots: `/private/tmp/cbf2026-results/smoke/H0`,
  `/private/tmp/cbf2026-results/smoke/C1`, and
  `/private/tmp/cbf2026-results/smoke/U0`.

The worktree resolves to branch `codex/cbf2026-diagnostic` at
`47e7b3d6f5b1be74a3dfa32150b4246c619e9a0d`, exactly matching every manifest.
The retained binary hash is
`4ee6c5f07eebe2ac7ada95f592f1785bb213a90e3320022f1a6c569810e35221`.

Each case root has exactly one timestamped child and its `stdout.log` contains
the matching `[OUTPUT_DIR]` line. The result paths are unambiguous:

| Case | Timestamped child | Data SHA-256 | Config SHA-256 |
|---|---|---|---|
| H0 | `2026-07-27_21-26-16_H0_seed_20260727_20s` | `59342c98f1b06adcf888fb1e53fb431752a58f4d939a8d2c0064532b34599107` | `aa4d537543f3caec42c2224284ae9fb0723eba10cd8fc26a84e5171aca98f526` |
| C1 | `2026-07-27_21-27-52_C1_seed_20260727_20s` | `7b928b37ddb5b004e6d4224363d743290d1a9739678b13c8cd82bfec716a5ef0` | `f0a0f8d4161fd9af4fcdf3e5fc474eff7c55e0431f1364d6b7e8b63774aa9d80` |
| U0 | `2026-07-27_21-28-45_U0_seed_20260727_20s` | `c87c3aef93207421d9c1b42f958f787a0746a078e2621a8970beb171f361d8fe` | `841909695182452fe471582f9a95904a0216903a28b17ba43c5e00ba27c991bc` |

I recomputed the file hashes above, the three manifest hashes, the three
summary JSON hashes, the three summary Markdown hashes, and the binary hash;
all equal the smoke report. All twelve JSON artifacts (manifest, materialized
config, data, and summary for each case) strictly parse. A recursive scan of
their numeric values found no non-finite value. All three Markdown summaries
are retained, and all three `stderr.log` files have zero bytes.

## Reproducibility and completion checks

Every manifest records case identity, the branch and commit above, Gurobi,
seed `20260727`, requested horizon `20.0`, return code `0`, and termination
reason `completed`. Direct raw-data inspection finds 40 frames of 14 robots
in each case, with logged runtime from 0.0 through 19.5 s. The retained stdout
for each case then reports `After 20.0000 seconds`; thus 19.5 s is correctly
reported as the final *logged* frame, not an early simulator termination.

The materialized config and manifest hashes match their reported values. The
manifest `free_bytes_before` values are H0 11,687,211,008, C1 11,682,422,784,
and U0 11,673,915,392, all above the 8 GB start requirement; the corresponding
`free_bytes_after` values are H0 11,685,576,704, C1 11,680,776,192, and U0
11,672,412,160, all above the 6 GB live floor. No manifest has
`termination_reason: disk_hard_floor`.

## Solver, residual, and consistency checks

I independently recomputed the raw optimization checks from every robot in
every frame:

| Case | `success` / `optimal` records | Non-finite control components | Hard records | Recomputed hard minimum | Below -1e-7 | Missing/mismatched logged residual |
|---|---:|---:|---:|---:|---:|---:|
| H0 | 560 | 0 | 1,120 | 6,561,323.462376105 | 0 | 0 / 0 |
| C1 | 560 | 0 | 1,680 | 8.44213587924969e-13 | 0 | 0 / 0 |
| U0 | 560 | 0 | 1,680 | 6.718181566611747e-12 | 0 | 0 / 0 |

For every hard record, `const + coe dot result` exactly equals the logged
residual at retained floating-point precision. The summary JSONs also report
zero unapplied, failed, unconfirmed, or placeholder records and no infeasible
minimum-L-infinity-bound frame (maximum bound `0.0` for all cases).

The retained summaries report zero class-K missing, mismatch, or unmapped
records: 1,120 checks for H0 and 1,680 for C1 and U0. Their covariance-to-
uncertainty checks are 560/560 matched per case, with maximum absolute error
`2.6645352591003757e-15` (H0) and `1.7763568394002505e-15` (C1/U0). These
facts support use of the logged uncertainty values in the tightened margins.

## Margin checks and H0 interpretation

The reported minimum margins are supported by the retained summaries:

| Case | Localization nominal / tightened minimum (m) | Collision nominal / tightened minimum (m) |
|---|---:|---:|
| H0 | 404.698830 / 403.333170 | -3.001689 / -5.680972 |
| C1 | 406.269740 / 404.904993 | 4.857228 / 1.890026 |
| U0 | 404.698830 / 403.333170 | 3.606157 / 0.836949 |

I independently recomputed H0 pair distances from the raw state and
uncertainty records (nominal margin = separation - 10 m; tightened margin
subtracts both UAV uncertainty values). The first tightened breach is frame 4,
2.0 s, UAVs 12--13: separation 11.292016400293964 m, nominal margin
1.2920164002939636 m, tightened margin -1.237156130043356 m. The first
nominal breach is the same pair at frame 5, 2.5 s, -0.841401248812808 m. The
worst nominal and tightened results are frame 9, 4.5 s, UAVs 10--11:
separation 6.998310599303972 m, nominal -3.001689400696028 m, tightened
-5.680971508313708 m. This exactly matches the smoke report.

This does not block the smoke gate under the brief's mandatory conditions. It
is the expected behavior of H0's materialized configuration:
`cbfs.without-slack.safety.on` is `false`, while the design plan defines H0 as
a historical replay/provenance control that must not be reported as safety
validation. H0's 1,120 hard records are fixed-communication constraints, not
evidence that a safety constraint was applied. The report correctly limits the
claim and does not infer physical contact from a 10 m safety-margin breach.

## C1--U0 isolation and inference

After removing only `output_path` and `run_suffix`, a sorted materialized-
configuration diff contains exactly two changes:

- `cbfs.without-slack.safety.consider-uncertainty`: `true` (C1) to `false` (U0)
- `cbfs.without-slack.comm-fixed.consider-uncertainty`: `true` (C1) to `false` (U0)

Both retain safety and fixed communication CBFs enabled. Raw frame-zero robot
IDs, states, uncertainty scalars, and position-covariance records are exactly
equal for all 14 UAVs. The observed 1.251071336 m nominal and 1.053077096 m
tightened collision-clearance advantages for C1, along with the changed
trajectory, are therefore a valid single-seed diagnostic indication that the
two tightening switches affect behavior. The smoke report appropriately does
not promote this to a general causal effect size, performance advantage, or
long-horizon safety conclusion.

## Findings

### Critical

None. No mandatory gate failure is present: no invalid or missing JSON,
ambiguous output, non-finite value, disk termination, uncontrolled failed
solve, hard residual below -1e-7, or material logged-residual discrepancy.

### Important

None. The only negative pairwise margins are accurately attributed to H0's
deliberately disabled safety CBF and are claim-limited in both the specification
and the smoke report.

### Minor

1. The standalone output of the required explicit `require_start_space()`
   calls and the external post-run free-space probes is not preserved as a
   separate raw artifact. The values are recorded in the smoke report, while
   the independently retained machine-readable evidence is the manifest's
   before/after values above. This is not gate-blocking because each manifest
   independently proves the relevant threshold conditions and no disk
   termination occurred. For the full runs, retain those probe outputs (or add
   them to the manifest) to make every reported disk value independently
   auditable.

## Authorization

The smoke report's `authorized_for_sequential_single_500s_runs` decision is
valid under the exact Task 5 gate rules. H0 remains authorization for unsafe
historical comparison only; C1 and U0 remain diagnostic, not validation, runs.
Any full run must fresh-check the 8 GB start threshold, preserve the 6 GB live
floor, stop on the specified hard failures, and analyze the unique actual
`data.json` before launching the next case.
