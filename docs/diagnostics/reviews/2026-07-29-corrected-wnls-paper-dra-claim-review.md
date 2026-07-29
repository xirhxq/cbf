# Final independent cross-repository claim review for corrected WNLS evidence

## Verdict

**CLEAN / READY at the reviewed three-repository snapshot.**

Issues by severity:

- Critical: 0.
- Important: 0.
- Minor: 0.

The paper, diagnostic evidence, and DRA records consistently preserve the
negative scientific result.  They do not promote the registered radius to a
validated error bound and do not claim a mission-level probability,
closed-loop estimator-robustness, or closed-loop safety guarantee.

This review did not rerun Monte Carlo or re-read the approximately 400 MB
decompressed Stage 2 stream.  It reviews the already independently audited
immutable evidence identity, directly checks the small summary and manifest
files, and audits every downstream claim against the reviewed report.

## Reviewed Git anchors

The diagnostic repository is on branch `codex/cbf2026-diagnostic` with no
upstream.  The distinct provenance anchors are:

| Role | Commit |
| --- | --- |
| Frozen objective-correct executable | `f6e995d6dc4711f0d2869636beb70f061e73063a` |
| Preregistered corrected run | `2b4b5ac266d05f3f3ebb28b8129d8ec9876fd588` |
| Corrected evidence and independent raw-row review | `7653938eb2e370e79d01ba98fe8d67f6a8aa3495` |
| Earlier paper/DRA claim-review snapshot | `0374137` |
| Radius-notation and registered-environment provenance correction | `d3a6eda516b5dddc3e03936f80e886c9e48fd123` |

The executable, registration, evidence, and review roles are not conflated.
The report and evidence-review working blobs at `d3a6eda` are:

| File | Bytes | SHA-256 |
| --- | ---: | --- |
| `docs/diagnostics/2026-07-29-corrected-wnls-localization-calibration.md` | 20,571 | `dd410d5607b7e4ed24a785d2c8780bcbfa4051b46cacc573faf26e681b21f00d` |
| `docs/diagnostics/reviews/2026-07-29-corrected-wnls-calibration-evidence-review.md` | 12,330 | `fdea3aefc5ff7413e6e4dc0dbba79b78a4eb68eec71da14f1d0dd8a3b2ba8f66` |

The paper repository is reviewed at
`62f414b84d243a53fa49136838630a4663a86ab2`.
The DRA repository is reviewed at
`7651dcc6a608f8cacaaacba1064a8e626e5a145c`.

## Numerical and decision audit

Every downstream claim agrees with the corrected evidence:

| Quantity | Dynamic DAG | Fixed references |
| --- | ---: | ---: |
| Primary tuples per graph case | 139,720 | 139,720 |
| Primary containment | 62,516 / 139,720 = 44.7438% | 41,913 / 139,720 = 29.9979% |
| Seed-bootstrap 95% CI | [34.2591%, 54.5785%] | [27.4098%, 32.5809%] |
| Minimum squad-local-depth containment | 2,923 / 19,960 = 14.6443% | 100 / 19,960 = 0.5010% |
| Invalid-attempt rate | 66,367 / 139,720 = 47.5000% | 57,385 / 139,720 = 41.0714% |
| Failed-attempt rate | 6,311 / 139,720 = 4.5169% | 184 / 139,720 = 0.1317% |

The 280,000 process rows include 560 frame-zero initialization rows.
The 279,440 primary rows therefore contain 139,720 tuples for each graph
case.  The confidence intervals are consistently labeled as seed-level
bootstrap intervals; no reviewed file treats robot-frame tuples as
independent bootstrap units.

Only the complete-row/no-silent-drop condition passes.  The dynamic
aggregate result misses the preregistered 98% threshold, every-depth
adequacy misses the 95% threshold, and both dynamic invalid and failed rates
are worse than the fixed-reference ablation.  Evidence/report integrity is
therefore **PASS**, while preregistered scientific adequacy is **FAIL**.

The registered radius is defined exactly as
\(\epsilon=3\sqrt{\lambda_{\max}(P)}\).
Across the report, evidence review, paper, and seven DRA files, no instance
misnames this quantity as \(3\epsilon\).  The downstream wording correctly
uses “epsilon radius” or “coefficient-3 FIM-derived epsilon radius.”

## Immutable bundle and process-provenance audit

The DRA source index preserves the excluded old Stage 2 and all three
corrected bundles as distinct records:

| Bundle | Path | Allocation |
| --- | --- | ---: |
| Corrected Stage 1 A | `/private/tmp/cbf2026-localization-calibration-corrected/stage1/localization-calibration/20260728T184323.997907Z_74cdb2fed5294fc6b52fa5f688f71f12` | 307,200 bytes |
| Corrected Stage 1 B | `/private/tmp/cbf2026-localization-calibration-corrected/stage1/localization-calibration/20260728T184607.307681Z_f49e35405e914d938fc5e957af5f318d` | 307,200 bytes |
| Corrected Stage 2 | `/private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288` | 47,714,304 bytes |
| Excluded old Stage 2 | `/private/tmp/cbf2026-localization-calibration/stage2/localization-calibration/20260728T164130.657823Z_6832cfbd6592469085adb88b5a165f59` | 33,460,224 bytes |

All four paths remain present.  Fresh direct hashes of their
`summary.json`, `summary.md`, and `manifest.json` files match the values in
`papers/cbf2026/sources.md` and the corrected report.  The corrected Stage 1
summary and content identities match across A and B while their manifests
remain run-specific.  The corrected Stage 2 summary reports
280,000/280,000 rows, contract
`variable_weight_nls_full_residual_jacobian_v1`, the five adequacy booleans,
and the aggregate values above.  The compressed and decompressed stream
identities remain bound by the prior independent raw-row review; they were
not recomputed in this claim-only review.

The DRA results log records the exact registered Stage 1 and Stage 2
commands, `cbf_env` with Python 3.11.12 and NumPy 1.24.4, the launch and
runner disk probes, the 6 GB live floor, the 8 GB launch gate, and the 2 GB
evidence/cache policy.  Its final recorded probe is 9,284,841,472 bytes.
It also records that cleanup was restricted to named rebuildable application
and runtime caches and did not remove evidence, user documents, or Git
history.  These statements agree with the reviewed process record.

## Paper audit

The paper commit changes only `main.tex`.  Its reviewed identities are:

| File | Bytes | SHA-256 |
| --- | ---: | --- |
| `main.tex` | 95,964 | `66d20272fabb08d943c9d53d84d1577b423c13b4f3809f0525b4b3fd0f716faa` |
| Existing working-tree `main.pdf` | 6,189,611 | `6c369c183e67f8c711836be4fac2943bb236e37fc6e2670c5525bc3a307f37b0` |

The WNLS objective uses position-dependent variance, and the prose states
that the corrected implementation differentiates the whitened residuals of
the full objective with the `1e-9` residual--Jacobian stationarity gate.
It keeps the localization FIM distinct from the estimator Gauss--Newton
matrix.  The results table contains the exact counts, intervals, rates, and
units above.

The interpretation boundaries are complete: this is an offline sidecar
outside the controller, dynamic versus fixed is a graph ablation rather than
an estimator comparison, 20 noise seeds reuse one preserved trajectory,
shared-ancestor cross-correlations are omitted, and higher dynamic aggregate
containment is not claimed as graph superiority.  The conclusion explicitly
calls the radius a design quantity with an unresolved calibration gap.

The paper worktree retains the user's intended dirty state: modified
`main.pdf` plus untracked `AGENTS.md`, `rebuttal.pdf`, and `rebuttal.tex`.
Their presence is not part of commit `62f414b`, and the current `main.pdf`
hash matches the preserved user artifact.

## DRA audit

Commit `7651dcc` contains exactly these seven CBF2026 paths:

| File | Bytes | SHA-256 |
| --- | ---: | --- |
| `meta-log/2026-07-28-cbf2026-dynamic-localization-calibration-results.md` | 14,670 | `0f56f4374d68454576f82e981bdb80efea3fd8e11527f8a5bb04a66f59040629` |
| `meta-log/2026-07-28-cbf2026-dynamic-localization-dag-calibration-design.md` | 4,552 | `02bbcee14a29e144c0fbf4be300649f9baded7c4c9132de8fc3942ca2a0c0ab1` |
| `papers/cbf2026/open-questions.md` | 8,244 | `9428548526b80ad576c80ef784877fd3b49517fe6689dc3550e3de10271e9f08` |
| `papers/cbf2026/status.md` | 13,365 | `f5071dbde3d4b9e41bbed6821d7dac4eab9669ac80f6985afdd18f8717aee339` |
| `papers/cbf2026/sources.md` | 9,196 | `1d812a52231cda642e9b55258b5a8479f5d9f3098c8de105385ada0b000d22a9` |
| `papers/cbf2026/theory/2026-07-28-rate-aware-robust-cbf-closure.md` | 32,616 | `3bcb4f7a6f36f45ef81522fab49ffb60bcd338f55ff8e0a985dc79dcdec6a922` |
| `papers/cbf2026/timeline.md` | 18,178 | `40ece819c90f1ed8d84943513e483524dcfd09629fae2df05376775422a7a652` |

Those seven files reproduce the corrected numbers, thresholds, failed
adequacy decision, notation, estimator limitations, paper anchor, report
hashes, bundle identities, and process provenance.  The timeline preserves
the historical rows and changes only the latest correction row.

The commit contains no PodSearch2026 or JFR path.  The DRA worktree's
remaining eight modified PodSearch2026 files, PodSearch2026 untracked files,
and three untracked JFR plan/spec files are unrelated inherited work and
were not included in the CBF2026 commit.

## Final checks

`git diff --check` and commit-level whitespace checks are clean in the
diagnostic, paper, and DRA repositories.  The only diagnostic worktree
change introduced by this review is this review file; the unrelated
untracked `build-diagnostic/` directory remains untouched.
