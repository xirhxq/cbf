# 20-Second H0/C1/U0 Diagnostic Smoke Gate

Date: 2026-07-27

## Decision

All three 20-second cases produced valid, unambiguous, and interpretable
evidence.
No case met a mandatory implementation-gate failure condition:
there was no missing or invalid JSON, ambiguous output path, non-finite
state/control value, disk termination, uncontrolled failed solve, hard
residual below `-1e-7`, or material logged-residual discrepancy.
H0 did violate the nominal and uncertainty-tightened pairwise safety margins,
but this is an expected and attributable result of the historical baseline's
explicitly disabled safety CBF rather than an uncontrolled implementation
failure.

The smoke evidence authorizes one sequential 500-second run of each of H0, C1,
and U0 under the same per-case 8,000,000,000-byte start gate, 6,000,000,000-byte
live floor, and immediate post-run analysis.
H0 is authorized only as an unsafe historical baseline and cannot support a
safety claim.
This authorization is not evidence of long-horizon safety, feasibility,
scalability, or statistical validation.

## Reproducibility metadata

All cases used:

- worktree: `/private/tmp/cbf2026-diagnostic`
- branch: `codex/cbf2026-diagnostic`
- base commit: `47e7b3d6f5b1be74a3dfa32150b4246c619e9a0d`
- binary: `/private/tmp/cbf2026-diagnostic/build-diagnostic/Swarm`
- binary SHA-256:
  `4ee6c5f07eebe2ac7ada95f592f1785bb213a90e3320022f1a6c569810e35221`
- solver: Gurobi
- seed: `20260727`
- requested horizon: 20 simulated seconds
- output root: `/private/tmp/cbf2026-results/smoke`

| Case | Config SHA-256 | Manifest SHA-256 | Started (UTC) | Ended (UTC) | Return / termination |
|---|---|---|---|---|---|
| H0 | `aa4d537543f3caec42c2224284ae9fb0723eba10cd8fc26a84e5171aca98f526` | `840378c4c67e18bc09763d3731ef4a17641f0c4588eb73bafd763dadb9770c5a` | `2026-07-27T13:26:15.884644+00:00` | `2026-07-27T13:26:17.902157+00:00` | `0` / `completed` |
| C1 | `f0a0f8d4161fd9af4fcdf3e5fc474eff7c55e0431f1364d6b7e8b63774aa9d80` | `0cce5265217316c66b9c3f07fed0c953c75b051ae3f23255012efe171203d4f2` | `2026-07-27T13:27:52.567704+00:00` | `2026-07-27T13:27:53.578747+00:00` | `0` / `completed` |
| U0 | `841909695182452fe471582f9a95904a0216903a28b17ba43c5e00ba27c991bc` | `1f94543c6e731b076c54d8b5c52d9398b5b74ce1876e251613183dda0519e485` | `2026-07-27T13:28:45.659440+00:00` | `2026-07-27T13:28:46.667410+00:00` | `0` / `completed` |

The materialized configuration truth values were:

| Case | Safety CBF on | Safety uncertainty tightening | Fixed-communication CBF on | Communication uncertainty tightening | Constraint-violation logging |
|---|---:|---:|---:|---:|---:|
| H0 | false | true (inoperative while safety is off) | true | true | false |
| C1 | true | true | true | true | true |
| U0 | true | false | true | false | true |

After excluding case-specific `output_path` and `run_suffix`, the only
materialized-configuration differences between C1 and U0 are the two
`consider-uncertainty` truth values shown above.
Their frame-zero UAV positions, yaws, and uncertainty values are identical.

## Exact result paths and artifact hashes

### H0

- case root: `/private/tmp/cbf2026-results/smoke/H0`
- manifest:
  `/private/tmp/cbf2026-results/smoke/H0/manifest.json`
- data:
  `/private/tmp/cbf2026-results/smoke/H0/2026-07-27_21-26-16_H0_seed_20260727_20s/data.json`
- summary JSON:
  `/private/tmp/cbf2026-results/smoke/H0/2026-07-27_21-26-16_H0_seed_20260727_20s/diagnostic-summary.json`
- summary Markdown:
  `/private/tmp/cbf2026-results/smoke/H0/2026-07-27_21-26-16_H0_seed_20260727_20s/diagnostic-summary.md`
- data SHA-256:
  `59342c98f1b06adcf888fb1e53fb431752a58f4d939a8d2c0064532b34599107`
- summary JSON SHA-256:
  `e8eb5703f524b5ae03433f98ea147eedda73278f3f18f6af684c7b72b35217d2`
- summary Markdown SHA-256:
  `b51ea19f1999b7f4618f74edb7f6f05bb697a83e13c9d07cc110eea36f114460`

### C1

- case root: `/private/tmp/cbf2026-results/smoke/C1`
- manifest:
  `/private/tmp/cbf2026-results/smoke/C1/manifest.json`
- data:
  `/private/tmp/cbf2026-results/smoke/C1/2026-07-27_21-27-52_C1_seed_20260727_20s/data.json`
- summary JSON:
  `/private/tmp/cbf2026-results/smoke/C1/2026-07-27_21-27-52_C1_seed_20260727_20s/diagnostic-summary.json`
- summary Markdown:
  `/private/tmp/cbf2026-results/smoke/C1/2026-07-27_21-27-52_C1_seed_20260727_20s/diagnostic-summary.md`
- data SHA-256:
  `7b928b37ddb5b004e6d4224363d743290d1a9739678b13c8cd82bfec716a5ef0`
- summary JSON SHA-256:
  `022918e6a6fbaf38ff4f38ca96ea8a96d6b12cb422bb74812eb3369583152c0b`
- summary Markdown SHA-256:
  `23cc641623c7431b79a2549f150b45de4d161b0b8d58d0a9ece95ad2b7fa0885`

### U0

- case root: `/private/tmp/cbf2026-results/smoke/U0`
- manifest:
  `/private/tmp/cbf2026-results/smoke/U0/manifest.json`
- data:
  `/private/tmp/cbf2026-results/smoke/U0/2026-07-27_21-28-45_U0_seed_20260727_20s/data.json`
- summary JSON:
  `/private/tmp/cbf2026-results/smoke/U0/2026-07-27_21-28-45_U0_seed_20260727_20s/diagnostic-summary.json`
- summary Markdown:
  `/private/tmp/cbf2026-results/smoke/U0/2026-07-27_21-28-45_U0_seed_20260727_20s/diagnostic-summary.md`
- data SHA-256:
  `c87c3aef93207421d9c1b42f958f787a0746a078e2621a8970beb171f361d8fe`
- summary JSON SHA-256:
  `ee444af511db9124d38c3c00aea7caf42aef7e830f89981ec7b8ec2a7cca3c20`
- summary Markdown SHA-256:
  `e42b1e3478bfe9fa33f3b97761dfd0cef9c8221e59a40744f5e93fb98eb1bb9c`

In each case, the runner's `[OUTPUT_DIR]` line exactly identified the
timestamped child listed above, and it was the unique child containing
`data.json`.

## Non-destructive source binding

These three runs predate the Task 8 prospective unique-bundle/source-snapshot
runner policy.
Their original manifests remain byte-for-byte unchanged, with the same
SHA-256 values shown in the reproducibility table.
Each timestamped run directory now contains a
`provenance-amendment.json` sidecar:

| Case | Sidecar SHA-256 |
|---|---|
| H0 | `50935900f7fa1da49cc9847a33ed242d3cfe50997fdb779bfba212204ab47b58` |
| C1 | `fd34e9e723a31da5b82c0f5860c6c5946c739dd5ee4de8ef120e1829c50aa2e3` |
| U0 | `5ddea5c5ce8c4703e6bc2cd7120c992b93cd53f936052384bfd5f87a05b6d491` |

The sidecars bind each original manifest, materialized configuration, raw
data, summaries, and binary hash to the preserved source archive
`docs/diagnostics/source-snapshots/2026-07-27-evidence-source.tar.gz`,
SHA-256
`5d1be79f896f89c96840194255a14f2de2186419bbcaa3dc89d14269ffe21847`.
That archive captures the dirty Tasks 1--4 evidence-producing source state
after these runs and before the later Task 8 runner repair; it does not claim
to contain that prospective repair.

The central index
`docs/diagnostics/2026-07-27-evidence-provenance.json`, SHA-256
`9dc16127c9c644f085e2cbb67589c9901d83269ec59b147c7102f0b987c59d22`,
enumerates all five retained smoke/full bindings and the prospective Task 8
policy.
The approved independent reviews are preserved byte-for-byte at the
durable-intent paths
`docs/diagnostics/reviews/2026-07-27-task-5-smoke-gate-independent-review.md`
(SHA-256
`54d5438d92073dcaa54d18869faa9581f7be83048c923ba674400c6eb862c5ff`)
and
`docs/diagnostics/reviews/2026-07-27-task-6-full-gate-independent-review.md`
(SHA-256
`f450a92334d283442641c3659c502d568fb4a7be186229d5e51841b399cffc59`).
This is a non-destructive provenance repair, not an original manifest field or
an empirical reinterpretation.
The archive, index, compact reports, review copies, reviewed source, and DRA
record remain uncommitted and must be versioned before temporary worktree
cleanup.
The raw evidence and sidecars remain under `/private/tmp`, which is not a
durable archival repository.

## Disk gate and retained size

The explicit preflight value is from the required independent
`require_start_space(Path('/private/tmp'))` call.
The manifest before/after values are the runner's own measurements.
The external post-run value was measured immediately after the runner returned
and before analysis.

| Case | Explicit preflight bytes | Manifest before bytes | Manifest after bytes | External post-run bytes | Start / live-floor result |
|---|---:|---:|---:|---:|---|
| H0 | 11,684,446,208 | 11,687,211,008 | 11,685,576,704 | 11,683,991,552 | pass / pass |
| C1 | 11,682,557,952 | 11,682,422,784 | 11,680,776,192 | 11,680,776,192 | pass / pass |
| U0 | 11,669,938,176 | 11,673,915,392 | 11,672,412,160 | 11,660,296,192 | pass / pass |

Filesystem free space after all analyses and artifact checks was
11,661,131,776 bytes.
Allocated retained size was 8,756 KiB overall: H0 2,580 KiB, C1 3,088 KiB,
and U0 3,088 KiB.
No raw smoke evidence was deleted.

## Compact comparison

| Metric | H0 | C1 | U0 |
|---|---:|---:|---:|
| Frames / final simulated runtime (s) | 40 / 19.5 | 40 / 19.5 | 40 / 19.5 |
| Optimal solves / other solves | 560 / 0 | 560 / 0 | 560 / 0 |
| Solve time mean / p95 / max (ms) | 0.134588 / 0.240100 / 5.111 | 0.139836 / 0.255400 / 4.014 | 0.148343 / 0.283200 / 3.995 |
| Non-finite state/control values | 0 | 0 | 0 |
| Applied hard constraints | 1,120 | 1,680 | 1,680 |
| Hard residual minimum | 6,561,323.462376 | 8.44269e-13 | 6.71818e-12 |
| Hard residuals below `-1e-7` | 0 | 0 | 0 |
| Logged residual missing / mismatch | 0 / 0 | 0 / 0 | 0 / 0 |
| Localization nominal minimum (m) | 404.698830 | 406.269740 | 404.698830 |
| Localization tightened minimum (m) | 403.333170 | 404.904993 | 403.333170 |
| Collision nominal minimum (m) | -3.001689 | 4.857228 | 3.606157 |
| Collision tightened minimum (m) | -5.680972 | 1.890026 | 0.836949 |
| Actual control L2 mean / max | 21.135436 / 25.014843 | 21.050865 / 25.014843 | 21.085387 / 25.014843 |
| Actual control Linf mean / max | 19.305324 / 24.999999 | 19.278942 / 24.999999 | 19.245782 / 25.000000 |
| Maximum per-frame minimum feasible Linf bound | 0.0 | 0.0 | 0.0 |
| Infeasible bound frames | 0 | 0 | 0 |
| Maximum uncertainty | 7.574288 | 7.574288 | 7.574287 |
| Maximum positive one-step uncertainty rate | 1.684780 | 1.627771 | 1.668012 |
| Coverage cells / fraction | 9,304 / 0.103378 | 9,381 / 0.104233 | 9,309 / 0.103433 |
| Class-K missing / mismatch / unmapped | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| Covariance-to-uncertainty checked / mismatch | 560 / 0 | 560 / 0 | 560 / 0 |

All applied records required both `opt.status=success` and
`solver_info.status=optimal`.
There were zero unapplied, failed, or unconfirmed solver records in every
case.
The covariance-to-uncertainty maximum absolute errors were
`2.6645352591003757e-15` for H0 and `1.7763568394002505e-15` for both C1 and
U0.

## Findings and attribution

There were no mandatory implementation-gate failures.

H0 nevertheless has a safety-margin finding:

- The first uncertainty-tightened pairwise margin below zero occurs at
  simulated time 2.0 s (frame 4) for UAVs 12 and 13:
  tightened margin `-1.237156130043356` m while the nominal margin remains
  `1.2920164002939636` m.
- The first nominal pairwise margin below zero occurs at 2.5 s (frame 5) for
  UAVs 12 and 13: nominal margin `-0.841401248812808` m.
- The worst nominal and tightened margins both occur at 4.5 s (frame 9) for
  UAVs 10 and 11. Their separation is `6.998310599303972` m, giving nominal
  margin `-3.001689400696028` m and tightened margin
  `-5.680971508313708` m.

The logs support UAV-pair attribution for this finding.
They do not support physical contact or collision attribution beyond the
configured 10 m safety-margin violation.
C1 and U0 have no nominal or tightened pairwise-margin breach in the smoke
horizon.

## C1 versus U0

C1 and U0 start from identical logged UAV states and differ operationally only
in the two uncertainty-tightening flags.
C1 retains 1.251071336 m more minimum nominal collision clearance and
1.053077096 m more minimum tightened collision clearance than U0.
C1 also covers 72 more grid cells (0.0008 greater coverage fraction).
This controlled single-seed, 20-second contrast implicates uncertainty
tightening as a cause of the changed trajectory and clearance.
It does not establish a general causal effect size, long-horizon safety, or a
statistical performance advantage.

## Artifact validation

For each case, `manifest.json`, `config.materialized.json`, `data.json`, and
`diagnostic-summary.json` passed strict Python JSON loading.
Each `diagnostic-summary.md` decoded as UTF-8, had balanced fenced blocks, and
rendered from GitHub-Flavored Markdown to HTML with Pandoc.
All standard-error logs were empty.

RMSE, NEES, ANEES, tracking error, and saturation remain explicitly
unavailable because the retained logs lack distinct truth/estimate or input
bound evidence.
The maximum actual control norms above therefore must not be interpreted as a
saturation count.

## Per-case 500-second authorization

| Case | Smoke gate | 500-second authorization | Scope |
|---|---|---|---|
| H0 | pass with historical safety-margin finding | authorized | unsafe historical baseline only; no safety claim |
| C1 | pass | authorized | claim-aligned single run, still subject to full-horizon gate |
| U0 | pass | authorized | uncertainty ablation, still subject to full-horizon gate |

Overall decision: `authorized_for_sequential_single_500s_runs`.
The full runs must remain sequential, recheck at least 8,000,000,000 available
bytes before each launch, retain the 6,000,000,000-byte live termination floor,
and analyze each actual timestamped `data.json` before starting the next case.
