# MC-First 20-Second Numerical Smoke Gate

Date: 2026-07-28

## Decision

The coherent C0, R, RB, and RBP smoke suite passed its technical-validity
gate.
Every case completed 40 frames and 560 applied solves with Gurobi reporting
`optimal`.
There were no non-finite values, unapplied or failed solves, hard-CBF
residuals below `-1e-7`, input-limit violations, covariance-to-scalar
uncertainty mismatches, or practical collision/localization-margin failures.
The bounded cases exercised the limits rather than merely enabling them in
configuration.
RBP instantiated all 7,280 expected directed pairwise rows with no missing or
unexpected rows.

This result authorizes the planned paired 250-second C0/R mechanism test.
It does not yet support a paper claim of long-horizon safety, statistical
robustness, physical input feasibility, or a general benefit from the
uncertainty-rate term.
RB and RBP remain short mechanism checks; the current single seed does not
constitute a Monte Carlo population.

## Reproducibility

All four cases used:

- worktree: `/private/tmp/cbf2026-diagnostic`
- branch: `codex/cbf2026-diagnostic`
- source commit: `88dfeab81d754a811e173269d89438234e98ed51`
- binary: `/private/tmp/cbf2026-diagnostic/build-diagnostic/Swarm`
- binary SHA-256:
  `748537d461fbcb22b3b8d70de62835ce4c352f735381c9661f1894b3fe994c8b`
- solver: Gurobi
- seed: `20260727`
- requested horizon: 20 simulated seconds
- output root:
  `/private/tmp/cbf2026-results/mc-first-smoke-bootstrap-fixed`
- start-space gate: 8,000,000,000 bytes
- live hard floor: 6,000,000,000 bytes
- output-root cap: 2,000,000,000 bytes
- per-run cap: 250,000,000 bytes

The manifests report `working_tree_dirty=true` because the intentionally
untracked `build-diagnostic/` directory exists.
Immediately before the suite, `git status --short` otherwise contained no
tracked source changes.
Each run retained its own source snapshot and recorded the common binary hash.

| Case | Uncertainty-rate term | Hard input bounds | Collision-row mode |
|---|---:|---:|---|
| C0 | off | off | minimum |
| R | on | off | minimum |
| RB | on | on | minimum |
| RBP | on | on | pairwise |

The runner command was:

```bash
conda run -n cbf_env python -m scripts.diagnostics.run_diagnostic \
  --case CASE \
  --horizon 20 \
  --seed 20260727 \
  --binary build-diagnostic/Swarm \
  --output-root /private/tmp/cbf2026-results/mc-first-smoke-bootstrap-fixed
```

Each `data.json` was then analyzed with:

```bash
conda run -n cbf_env python -m scripts.diagnostics.analyze_diagnostic \
  DATA_JSON MANIFEST_JSON
```

## Exact bundles and hashes

| Case | Bundle | Config SHA-256 | Manifest SHA-256 | Data SHA-256 | Analysis SHA-256 |
|---|---|---|---|---|---|
| C0 | `C0/20260728T055713.859190Z_703fe1db992a46b38cd3f0cd57c1ff68` | `75d21bb04bb850826df93c6e8cfb20a12963f5519401b0ed549ecc688be510b0` | `639aaea0325b96820767a2932e7e990f4787f7fc3a51cfae5b444a3daf382816` | `92ee7cb925ef7dbd339e6bb5e1f073d9b085199a2d2b222c2e14f27f76e0ee2f` | `30389978a62e0654da6a38bfba883a9b707960794d2943172729a7a61a8d8229` |
| R | `R/20260728T055758.898433Z_4484d80f34a74eaf91346977a73b2733` | `0a15e279180105f2e212bc9a0698063adf0108a1f37de214c3711eaa13dc16c5` | `9e422552a1245f1cce852acd1fe8e1e16504a963ddf370a3ee290512100f900c` | `a3f16d1b19f04181d307ca9660058d2dc331d65a08bc097d8ad053d5418aa85f` | `5a36ba3f15afad6ff7c5d4fbd6a885d666fa29e918ddb2f05f37a5da4d39a5ee` |
| RB | `RB/20260728T055837.555946Z_f1e97709acdf4458ba4e18d66e82839d` | `02f2cc024d4d7708be449637b153299b6c1865291d6063d79a77f86139571c5b` | `b3450eb737be2cb26653688e0d3857d0b6d9b070a7b2ce5875ca5c68161dfd96` | `882007414cbbb6a7f977067b4eb97b79c3b9f238a8c010d58c95abe8a0c1db14` | `9dc25961d8437878020fc632f8923702f8f371122ec62ffeeb5a787e2ce57863` |
| RBP | `RBP/20260728T055858.518817Z_eb47bfe300ba44e897dae63420978dd0` | `93ced106b70b6ce24a661bf28c16a2c88a009adf45c8e15802523f510af8605e` | `0839860ca7ff66b2562170531258fbec91cc4fc21e2eb2bd77219e56e1814b07` | `59b42f6865d23c985c3533c6a064d3d1735b4913b6dfdafbc5273daae02d7c05` | `7499cb6564abc42b9a0e24c6ecd31e030e938e930ed3b2490fcb3771e8c7081b` |

Bundle paths are relative to the output root above.
The source-snapshot SHA-256 values are respectively
`0242b1f760d31f5b9740568f435b6cd444287a0ed15480d3ea3a83e94ad85efa`,
`e84e91b6630db843f7273bdefa64c7762e6aa71d3eb497b834c3915e122ae02a`,
`198c2a81b3b0c9900cc12cfac1fb404136cf80cdef98be54d5a89b3dcea08461`,
and
`3417ce082f316849dee6a8644a4f248033970c23bcee3e8768f01c7fd60e9dea`.

## Compact results

| Metric | C0 | R | RB | RBP |
|---|---:|---:|---:|---:|
| Frames | 40 | 40 | 40 | 40 |
| Applied optimal solves | 560 | 560 | 560 | 560 |
| Other / unapplied solves | 0 | 0 | 0 | 0 |
| Non-finite values | 0 | 0 | 0 | 0 |
| Hard residual minimum | 1.126e-12 | 1.155e-12 | 3.784e-13 | 9.504e-13 |
| Hard residuals below `-1e-7` | 0 | 0 | 0 | 0 |
| Tightened collision margin minimum (m) | 3.448 | 3.460 | 4.264 | 7.483 |
| Tightened localization margin minimum (m) | 403.149 | 403.149 | 398.589 | 400.068 |
| Maximum per-frame minimum feasible \(L_\infty\) (m/s) | 23.231 | 23.342 | 23.884 | 23.688 |
| Maximum logged \(\dot\epsilon\) (m/s) | 1.572 | 1.572 | 1.658 | 1.598 |
| Maximum positive one-step uncertainty jump (m) | 0.786 | 0.786 | 0.829 | 0.799 |
| Maximum projected held-command mismatch (m/s) | 18.839 | 18.839 | 22.939 | 16.786 |
| Input records validated / invalid | n/a | n/a | 560 / 0 | 560 / 0 |
| Input-limit violations | n/a | n/a | 0 | 0 |
| At least one input component saturated | n/a | n/a | 32 | 24 |
| Bounded-feasibility headroom (m/s) | n/a | n/a | 1.116 | 1.312 |
| Pairwise expected / observed | n/a | n/a | n/a | 7,280 / 7,280 |
| Pairwise missing / unexpected | n/a | n/a | n/a | 0 / 0 |
| Solve time mean / p95 / max (ms) | 0.143 / 0.259 / 6.945 | 0.142 / 0.252 / 5.999 | 0.135 / 0.265 / 5.239 | 0.146 / 0.229 / 4.265 |

All 560 covariance-derived scalar uncertainties in each case matched the
logged uncertainty within the registered tolerance, with zero mismatches and
a maximum absolute error of `1.7763568394002505e-15`.
All instantiated class-\(\mathcal K\) parameters matched their materialized
configuration.

The uncertainty jump and held-command mismatch are empirical diagnostics,
not added sampled-data reserves.
Their presence does not invalidate this requested numerical study, but the
20-second observations cannot be promoted to analytical global bounds.

## Superseded first attempt

The earlier root `/private/tmp/cbf2026-results/mc-first-smoke` is retained
without deletion.
Its C0, R, and RB runs completed, but its RBP run stopped at frame 0.
The original runner also returned a misleading successful process status
after the internal failure.
Diagnosis showed that a second runtime-zero covariance update created
artificial startup uncertainty rates, producing contradictory pairwise rows
for UAV 2.

The coherent suite in this report supersedes that attempt after:

1. topological startup covariance publication with zero initial rate;
2. suppression of the duplicate runtime-zero covariance refresh;
3. propagation of internal run failures to a nonzero process exit;
4. preservation of the original exception through best-effort cleanup;
5. independent re-review of the failure path.

The superseded bundles remain useful failure evidence but must not be mixed
with the coherent comparison.

## Disk and retention

Before the coherent suite, free space was 13,150,871,552 bytes.
The lowest post-run value recorded by a manifest was 13,128,867,840 bytes,
well above the 6 GB hard floor.
After all four analyses, `df -Pk` reported 12,798,956 KiB available.
The coherent output root occupied 43,032 KiB and all retained
`/private/tmp/cbf2026-results` occupied 186,056 KiB.
No raw evidence or prior result was deleted.

## Next gate

Run C0 and R for 250 simulated seconds with the same source, solver, initial
geometry, input semantics, logging, and disk guards.
This pair reaches beyond the previous failures near 95 s and 212 s and is the
first experiment that can distinguish whether the rate-aware term resolves
the observed long-horizon mechanism.
If both pass, the original plan requires extending C0/R to 500 s because the
negative control would not have reproduced the failure.
Only after that mechanism gate should RB/RBP be promoted to long-horizon and
multi-geometry experiments.
