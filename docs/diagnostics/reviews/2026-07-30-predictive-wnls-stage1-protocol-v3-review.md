# Predictive WNLS Stage 1 v3 independent protocol review

## Decision

**APPROVED for the exact frozen v3 smoke, replay, and analyzer
invocations.**

Findings:

- Critical: 0
- Important: 0
- Minor: 0

The paper-edit gate remains **CLOSED**.
This approval is limited to the v3 protocol committed at
`5d7fbbb88ef6a08d25e5ebca587e03c182935e2d`,
which non-circularly binds implementation parent
`e70110f161d42691efcb6f992c949b5053399360`.
It does not authorize a changed source, command, constant, cohort,
denominator, path, retry, confirmatory interpretation, or paper edit.

No smoke, replay, or analyzer evidence command was run during this review.
Only the ordinary replay and analyzer direct CLIs in `--help` mode were
executed.
No output root was created.

## V2 retirement and v3 recovery boundary

The v2 protocol, preflight, and failure record remain immutable at
`5e5dd36`, `47426f5`, and `7addd76`, respectively.
Both v2 smoke commands were invoked exactly once and failed before
output-root allocation with:

```text
ModuleNotFoundError: No module named 'scripts'
```

The registered v2 replay and analyzer were never invoked.
The v2 command token and all four v2 roots are permanently retired despite
remaining absent.
This review does not revive or authorize any v2 invocation.

The secure direct-CLI repair was committed at `22c740a`.
The replacement production contract uses these distinct v3 identities:

```text
schema: cbf2026-predictive-wnls-stage1-protocol-v3
protocol ID: cbf2026-predictive-wnls-stage1-v3
protocol token:
  docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol-v3.json
smoke A:
  /private/tmp/cbf2026-predictive-wnls-smoke-v3-a
smoke B:
  /private/tmp/cbf2026-predictive-wnls-smoke-v3-b
registered replay:
  /private/tmp/cbf2026-predictive-wnls-development/stage1-v3
registered analyzer:
  /private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v3
```

The invocation names remain exactly `smoke_a`, `smoke_b`,
`registered_replay`, and `registered_analyzer`.
No retired v2 token or root appears in either committed v3 protocol file.
The raw-row schema and compact-analysis schema remain at v2 because their
serialized structures are unchanged.
The hermetic protocol schema and ID remain at v1 and are not production
authorization.

## Protocol and source identities

The committed v3 protocol files have SHA-256 identities:

```text
d8b4aad1c383b55f6171435d69606fe07409f491841fc81d82e2936bd1d957e8
  docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol-v3.json
130a240ecd6eaf391ebc8338b6ca530afe2c464cc6276a37c2008fd636223175
  docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol-v3.md
```

The JSON has exactly 16 top-level fields and exactly eight source
declarations.
It and the Markdown file reproduce byte-for-byte from the bound parent,
current source bytes, and deterministic registrar renderers.
The implementation parent contains neither future v3 protocol file.

The binding design is
`docs/superpowers/specs/2026-07-30-cbf2026-bounded-predictive-wnls-recovery-design.md`
at `a2ae1f0`.
Its current bytes and bound-commit bytes both have SHA-256
`f3820b8c912ed1a019334f0e7e273a1bcf3d3b1f4ec32cb059f5aaef597c152d`.
That design commit is an ancestor of the implementation parent.

The eight protocol sources were independently rehashed:

| Source | Frozen absolute path | Verified SHA-256 |
| --- | --- | --- |
| Analyzer | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/analyze_predictive_wnls_recovery.py` | `3e15783729b462b3a9990b8a676aa75e9d1dd9388c1eefbd37f248b68cdb302d` |
| Baseline process | `/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/restart/localization-calibration/20260729T112437.413270Z_efb8e7137776415c833f11204cf519d5/calibration.jsonl.gz` | `c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003` |
| Disk/integrity helper | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/run_diagnostic.py` | `ccde94fa739649cb9b94a8304ef3867ae946f28b0e3a812d3feb3f30bb76fd23` |
| Estimator | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/predictive_wnls.py` | `1c66b0d58f1f9c56464086529437fe9ed08c5888098266bd0519a8cbb7e576da` |
| Input manifest | `/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json` | `6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb` |
| Legacy solver | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/replay_localization_calibration.py` | `0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8` |
| Replay | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/replay_predictive_wnls_recovery.py` | `1e5864087faf0875c5e7db5ea8df441c1999acb9d15fd9cf68b6362262373d4e` |
| Truth trajectory | `/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json` | `3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527` |

Each of the five repository source hashes equals both its live bytes and its
blob at `e70110f161d42691efcb6f992c949b5053399360`.
The three external input hashes match their current bytes.
The protocol-bound implementation diff from the parent is empty.

## Scientific and runtime audit

| Review item | Result | Independent assessment |
| --- | --- | --- |
| Truth separation and sensor boundary | Pass | Truth is used at the simulation sensor boundary to determine measurement visibility/presence and generate noisy scalar ranges. Estimator qualification copies only the sensor fields `present` and `noisy_range`. Prediction, reference qualification, candidate generation, selection, and online acceptance receive communicated estimator states rather than truth coordinates, true ranges, truth errors, or offline percentiles. Truth-dependent error and containment metrics are computed only after public output finalization. |
| Predecessor command timing | Pass | Frame preflight reads finite applied `opt.result.vx/vy` from frame \(k-1\), and frame \(k\) propagates with exactly that predecessor command. Each raw row records the source frame and command. The analyzer independently reloads the protocol-bound trajectory, requires exact equality, and requires `6986/6986` unique predecessor coverage. |
| Dynamic fresh DAG and provenance | Pass | Execution order is variant, seed, frame, then ascending global UAV ID. Qualification-enabled variants accept only sensor-present bases and current-frame `fresh`, strictly lower-index UAVs. Predicted and unavailable UAVs cannot be anchors. Recursive current-frame base provenance is recomputed from used references, and a fresh publication requires at least two distinct base roots. The `prediction_expiry` diagnostic exception records every stale-anchor use as a violation and is ineligible as the complete estimator. |
| Expiry and re-acquisition | Pass | Prediction age is capped at two frames. At age three, public position, covariance, epsilon, and provenance are absent. The private propagated seed remains unpublished and reference-ineligible. Truth-free re-acquisition requires at least three current-frame fresh references, non-collinear geometry, two-base provenance, a finite positive-definite range FIM, and reduced whitened cost no greater than `9.0`. |
| Frozen finite numerics | Pass | The protocol freezes motion sigma `0.5`, innovation threshold `11.829007011943707`, catastrophic threshold `50 m`, at most `50` proposals, initial damping `1e-3`, damping bounds `1e-15` and `1e15`, damping factor `10`, stationarity threshold `1e-6`, spectral and representable-step thresholds `1e-12`, candidate deduplication `1e-9 m`, and relative tie tolerance `1e-12`. Candidate ordering, pair selection, tie-breaking, and solver budgets are deterministic and finite. |
| All-row and adverse retention | Pass | Every attempted exact `(variant, seed, frame, robot)` key must be serialized. Rejected, failed, invalid, predicted, and unavailable outcomes are not filtered. Candidate traces, rejection reasons, reference exclusions and violations, rejected-candidate offline errors, and the frame-44, 999 m, and 168 m mechanism keys remain auditable. Missing, duplicate, extra, or reordered keys fail closed. |
| Status semantics | Pass | Attempt and output states remain separate exact enums. Accepted publishes fresh at age zero. Every nonaccepted attempt publishes only a valid age-one/two prediction or unavailable. Predicted output is never relabelled fresh, and nonfresh output exposes no fresh provenance. |
| Fixed cohorts and denominators | Pass | The analyzer filters the immutable baseline to `dynamic_dag_wnls` and requires identical exact keys for all three cumulative variants. It separately retains the baseline-published and baseline-fresh fixed cohorts. Fresh-to-predicted downgrades and newly unavailable attrition never enter error-improvement denominators. Attempt, output, availability, error, containment, calibration, adverse, catastrophic, and stratum reports carry explicit denominators. |
| Catastrophic and input-bound audit | Pass | `E_CAT=50 m` is frozen before Stage 1 and applied to every applicable adverse family. The analyzer independently requires the known `243/7000` applied component-bound violations and complete predecessor coverage. Because this preserved trajectory violates the paper's applied-input premise, the paper gate stays closed regardless of Stage 1 metrics. |
| Seeds, variants, and gates | Pass | The registered replay uses exactly the 20 seeds `20260727` through `20260746`, the ordered cumulative variants `prediction_expiry`, `fresh_reference_qualification`, and `predictive_multistart`, and no frame cap. Maximum fresh and published error must be strictly below `50 m`; paired fresh p95 may not worsen; fresh availability may drop by at most `0.02`; fresh-or-predicted availability must be at least `0.95`; prediction age is at most two; and qualifying-anchor, provenance, and DAG violation allowances are zero. |
| No outcome-tunable values | Pass | Estimator thresholds, solver budgets, variants, seeds, schemas, gates, and commands are compiled constants copied into and source-hash-bound by the protocol. Registered CLIs expose no estimator-threshold or outcome-selection option. A failed gate stops paper editing and cannot authorize parameter tuning, outcome-based replacement, or same-protocol retry. |

## Direct-CLI repair verification

The two ordinary direct CLIs were executed from the registered repository
working directory in validation-only mode:

```bash
conda run -n cbf_env python \
  scripts/diagnostics/replay_predictive_wnls_recovery.py --help
conda run -n cbf_env python \
  scripts/diagnostics/analyze_predictive_wnls_recovery.py --help
```

Both exited zero and printed the expected argument-parser usage.
Neither parsed or wrote an evidence bundle.

The two adversarial shadow-import regressions were run explicitly:

```bash
conda run -n cbf_env python -m unittest -v \
  tests.test_replay_predictive_wnls_recovery.ProtocolAndPreflightTests.test_direct_path_help_rejects_preceding_shadow_scripts_package \
  tests.test_analyze_predictive_wnls_recovery.DirectCliBootstrapTests.test_direct_script_cannot_import_preceding_shadow_scripts_package
```

Result: `2/2` passed.
The tests place a regular `scripts` package or replay module before the
implementation on `PYTHONPATH` and prove that it is not imported.
Together with the ordinary direct help probes, this closes the v2
pre-allocation import mechanism without relying on ambient `PYTHONPATH`.

## Lifecycle, disk, and exactly-once contract

The protocol binds four token-for-token command arrays.
The runtime validator accepts all three replay declarations, and every replay
and analyzer command equals its canonical implementation output.
Only the two distinct smoke roots have the frozen two-frame limit; the
registered replay has no frame cap.

Replay and analyzer use exact output roots rather than timestamped children.
They reject pre-existing targets, symlinks, protected-path overlap,
source/input drift, invocation drift, and foreign replacement.
Before allocation, a failure creates no evidence target.
After allocation, success publishes a durable `completed` terminal manifest;
ordinary failure or interrupt publishes a `failed` terminal manifest and
retains partial adverse evidence.
Manifests bind protocol, source/input, process/output, row-count, disk, and
root identities.
The analyzer independently verifies the replay manifest and both compressed
and decompressed process hashes.

The frozen disk contract is:

```text
launch minimum free bytes:      8,000,000,000
live minimum free bytes:        6,000,000,000
raw/cache allocated-byte cap:   2,000,000,000
compact allocated-byte cap:        10,000,000
```

The review-time `/private/tmp` probe reported `48,338,591,744` available
bytes, above the launch minimum.
This does not replace the immediately-before-smoke or
immediately-before-registered-run probes.

All retired and v3 roots remained absent at the final review probe:

```text
/private/tmp/cbf2026-predictive-wnls-smoke-a
/private/tmp/cbf2026-predictive-wnls-smoke-b
/private/tmp/cbf2026-predictive-wnls-development/stage1-v2
/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v2
/private/tmp/cbf2026-predictive-wnls-smoke-v3-a
/private/tmp/cbf2026-predictive-wnls-smoke-v3-b
/private/tmp/cbf2026-predictive-wnls-development/stage1-v3
/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v3
```

V2 remains retired.
For v3, each smoke root may be used once in order only after this review is
committed.
The registered replay may run once only after both smokes pass.
The registered analyzer may run once only after the registered replay
completes successfully.
Any unsuccessful registered terminal state is preserved and cannot be
retried.

## Fresh verification

The complete diagnostic suite was run:

```bash
conda run -n cbf_env python -m unittest discover \
  -s tests -p 'test_*.py' -q
```

Result: `479` tests passed in `42.888 s`, exit code zero.
The printed argument-parser usage is the expected output of a negative CLI
test and is not a suite failure.

Protocol rendering, source binding, runtime shape, and canonical commands
were checked without writing any file.
The read-only validator reported:

```text
V3_PREFLIGHT_PROTOCOL_OK
16 top-level fields
8 exact sources
parent e70110f161d42691efcb6f992c949b5053399360
smoke_a smoke_b registered_replay
```

All repository source bytes were independently compared with the exact parent
blobs, and all external sources were independently rehashed.
`git diff --quiet` confirmed no implementation drift from the bound parent.
`git diff --check` passed.
Before this review was created, `git status --short` contained only the
preserved untracked `build-diagnostic/`.

## Final verdict

**APPROVED, with 0 Critical, 0 Important, and 0 Minor findings.**

Execution is authorized only in the v3 recovery-plan order:
commit this review, recheck the two v3 smoke roots and disk floor, execute the
exact committed `commands.smoke_a` and `commands.smoke_b` arrays once each,
require both completed 84-row bundles and matching process hashes, and only
then proceed through the registered replay and analyzer gates.
Any identity, byte, command, threshold, schema, cohort, denominator, path, or
precondition change invalidates this approval.
The paper-edit gate remains **CLOSED** through Stage 1.
