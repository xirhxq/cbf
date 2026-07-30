# Predictive WNLS Stage 1 independent protocol review

## Decision

**APPROVED for the exact frozen Stage 1 smoke, replay, and analyzer
invocations.**

Findings:

- Critical: 0
- Important: 0
- Minor: 0

The paper-edit gate remains **CLOSED**.
This approval is limited to the protocol committed at
`5e5dd36d1234659266d7990c44a6e765061a4f6f`,
which non-circularly binds implementation parent
`f3864177b9f36c75de2c7b650742394bedbaa3bc`.
It does not approve changed commands, sources, constants, cohorts,
denominators, output roots, a retry, or a paper claim.

No smoke, registered replay, or registered analyzer command was run during
this review.
None of their four exact output roots was created.

## Scope and immutable identities

The review checked the committed machine-readable protocol and its Markdown
rendering against the approved design, the bound implementation and tests,
the three external inputs, and the Task 8 preflight requirements.
The JSON has exactly the required 16 top-level fields and exactly eight source
declarations.
It deterministically reproduces byte-for-byte from the bound implementation
and current source bytes.
The protocol JSON and Markdown SHA-256 values are, respectively:

```text
9513cfc06caa80333dc6c6d368c7ce6de79ea05450bc561ef7275429eaf6a5b1
34f44ea467faf9e54dc26804b323c120d55cbe560a27e451140c358002f72581
```

The binding design is
`docs/superpowers/specs/2026-07-30-cbf2026-bounded-predictive-wnls-recovery-design.md`
at commit `a2ae1f0`.
Its current bytes and its bytes at that commit both have SHA-256
`f3820b8c912ed1a019334f0e7e273a1bcf3d3b1f4ec32cb059f5aaef597c152d`.
That commit is an ancestor of the bound implementation parent.
Neither protocol file exists in the implementation-parent tree, so the
provenance is non-circular.

The eight frozen sources were independently rehashed:

| Source | Frozen absolute path | Verified SHA-256 |
| --- | --- | --- |
| Analyzer | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/analyze_predictive_wnls_recovery.py` | `b5117e67ea2d5d4601bd8300918836e88ef789c6fe7603f36d09367c4781c7df` |
| Baseline process | `/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/restart/localization-calibration/20260729T112437.413270Z_efb8e7137776415c833f11204cf519d5/calibration.jsonl.gz` | `c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003` |
| Diagnostic-integrity helper | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/run_diagnostic.py` | `ccde94fa739649cb9b94a8304ef3867ae946f28b0e3a812d3feb3f30bb76fd23` |
| Estimator | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/predictive_wnls.py` | `1c66b0d58f1f9c56464086529437fe9ed08c5888098266bd0519a8cbb7e576da` |
| Input manifest | `/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json` | `6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb` |
| Legacy solver | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/replay_localization_calibration.py` | `0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8` |
| Replay | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/replay_predictive_wnls_recovery.py` | `bff33f93e3f9baf82c9808e5233395d4ea6ba7763ac72a7df2a193b9d775c380` |
| Truth trajectory | `/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json` | `3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527` |

For all five repository sources, the live SHA-256 equals both the protocol pin
and the blob at `f3864177b9f36c75de2c7b650742394bedbaa3bc`.
The implementation-source diff from that parent is empty.
All three external-input hashes match their current bytes.

## Scientific and runtime audit

| Review item | Result | Independent assessment |
| --- | --- | --- |
| Runtime truth separation and sensor boundary | Pass | Truth is used in `_sensor_records` only to determine simulated measurement presence/visibility and generate noisy scalar ranges. The estimator-facing qualification path copies only `present` and `noisy_range`; prediction, reference qualification, candidate construction, selection, and online acceptance receive communicated estimator states rather than truth coordinates or offline errors. `_offline_metrics` is applied only after the public output is finalized. The dedicated truth-boundary regression passes. |
| Predecessor command timing | Pass | Frame preflight extracts finite `opt.result.vx/vy` from frame \(k-1\), and frame \(k\) propagates with exactly `applied_commands[frame_index - 1][robot_id]`. Each raw row records that predecessor frame. The analyzer independently reloads the protocol-bound trajectory, requires exact command equality, and requires complete `6986/6986` predecessor coverage. |
| Dynamic fresh DAG and provenance | Pass | Execution order is variant, seed, frame, then ascending global UAV ID. Qualification-enabled variants may use only sensor-present bases and current-frame `fresh`, strictly lower-index UAVs. Predicted and unavailable UAVs cannot be anchors. Current-frame recursive base provenance is recomputed from used references, and a fresh publication requires at least two distinct base roots. The `prediction_expiry` diagnostic exception preserves stale-use evidence as an explicit violation and is ineligible as the complete estimator. |
| Re-acquisition | Pass | At prediction age three the public state becomes unavailable and exposes no position, covariance, epsilon, or provenance. The retained private seed is unpublished and reference-ineligible. Truth-free re-acquisition requires at least three current-frame fresh references, non-collinear geometry, two-base provenance, a finite positive-definite range FIM, and reduced whitened cost no greater than `9.0`. |
| Frozen finite numerics | Pass | The protocol exactly freezes age `2`, motion sigma `0.5`, innovation threshold `11.829007011943707`, re-acquisition threshold `9.0`, catastrophic threshold `50 m`, `50` proposals, damping bounds `1e-15` and `1e15`, initial damping `1e-3`, damping factor `10`, stationarity `1e-6`, spectral and representable-step thresholds `1e-12`, candidate deduplication `1e-9 m`, and relative tie tolerance `1e-12`. Candidate generation and tie-breaking are deterministic and finite. |
| All-row and adverse retention | Pass | The raw schema requires every attempted `(variant, seed, frame, robot)` row, including rejected, failed, invalid, predicted, and unavailable outcomes. Candidate traces, rejection reasons, reference exclusions/violations, offline rejected-candidate errors, and the predeclared catastrophic mechanism rows remain represented. Exact-key completeness and ordering fail closed. |
| Status semantics | Pass | Attempt statuses and output statuses are disjoint exact enums. Accepted means fresh at age zero; a failed/rejected/invalid/reference-unavailable attempt may publish only an age-one/two prediction or unavailable. Predicted output is never relabelled fresh, and nonfresh output cannot expose fresh provenance. |
| Paired cohorts and denominators | Pass | The analyzer filters the immutable baseline to `dynamic_dag_wnls`, requires identical exact keys for all three cumulative variants, and keeps separate baseline-published and baseline-fresh fixed cohorts. Fresh-to-predicted downgrades and newly unavailable attrition are reported separately and never enter error-improvement denominators. Attempt, output, error, containment, calibration, rejected-candidate, catastrophic, and availability summaries retain explicit denominators. |
| Catastrophic and input-bound evidence | Pass | `E_CAT=50 m` is frozen before Stage 1 and applied to fresh, all-published, downgraded, unavailable, and rejected-candidate evidence as applicable. The analyzer independently requires the known `243/7000` component-bound audit and reports the paper gate as closed regardless of estimator outcome. |
| Seeds, variants, and gates | Pass | Registered replay uses exactly the 20 seeds `20260727` through `20260746`, the three ordered cumulative variants `prediction_expiry`, `fresh_reference_qualification`, and `predictive_multistart`, and no frame cap. Maximum fresh and published error must be strictly below `50 m`; fresh p95 may not worsen; fresh availability may drop by at most `0.02`; combined availability must be at least `0.95`; age is at most two; and qualifying-anchor, provenance, and ascending-DAG violation allowances are all zero. |
| No outcome-tunable values | Pass | Estimator thresholds, finite solver budgets, ablations, seeds, gates, schemas, and commands are module constants copied into and hash-bound by the protocol. The registered CLIs expose no estimator-threshold or outcome-selection option. A failed gate stops paper editing and does not authorize tuning or a same-protocol retry. |

## Evidence lifecycle and execution contract

The protocol binds four token-for-token command arrays:
`smoke_a`, `smoke_b`, `registered_replay`, and `registered_analyzer`.
The runtime protocol validator accepts all three replay invocation declarations,
and all four arrays equal the canonical implementation contract.
The registered replay has `max_frames = null`; only the two distinct smoke
roots use the predeclared two-frame limit.

Replay and analyzer use exact output roots rather than timestamped child
directories.
They reject pre-existing empty or nonempty targets, symlinks, protected-path
overlap, source/input drift, invocation drift, and foreign replacement.
Before allocation they fail closed without creating an evidence root.
After allocation, success publishes a durable `completed` terminal manifest;
ordinary failure or interrupt publishes a `failed` terminal manifest while
preserving partial adverse evidence.
The manifests bind protocol, source/input, process/output, row-count, disk,
and output-root identities.
The analyzer independently verifies the replay manifest and compressed and
decompressed raw-process hashes before aggregation.
Lifecycle, crash/fault-injection, and foreign-replacement regressions pass.

The frozen disk contract is:

```text
launch minimum free bytes:      8,000,000,000
live minimum free bytes:        6,000,000,000
raw/cache allocated-byte cap:   2,000,000,000
compact allocated-byte cap:        10,000,000
```

Both registered components apply the 8 GB launch gate and 6 GB live floor;
the replay enforces the 2 GB raw cap, and the analyzer enforces the 10 MB
compact cap including its manifest.
The review-time probe reported `49,020,563,456` available bytes on
`/private/tmp`, above the launch threshold.
This observation does not replace the mandatory immediately-before-launch
probe.

Registered retry is `false`.
If the registered replay fails, the analyzer is not authorized.
If either registered command fails, its terminal state is frozen and neither
automatic nor manual same-protocol retry is authorized.

At the end of review, all exact roots remained absent:

```text
/private/tmp/cbf2026-predictive-wnls-smoke-a
/private/tmp/cbf2026-predictive-wnls-smoke-b
/private/tmp/cbf2026-predictive-wnls-development/stage1-v2
/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v2
```

## Fresh verification

The focused implementation, replay, analyzer, and registrar suite was run
without invoking any frozen smoke or registered command:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_predictive_wnls \
  tests.test_replay_predictive_wnls_recovery \
  tests.test_analyze_predictive_wnls_recovery \
  tests.test_register_predictive_wnls_stage1 -q
```

Result: `147` tests passed in `19.200 s`.

The full diagnostic suite was also run:

```bash
conda run -n cbf_env python -m unittest discover \
  -s tests -p 'test_*.py' -q
```

Result: `475` tests passed in `42.114 s`, exit code zero.
The printed argument-parser usage is the expected output of a negative CLI
test and is not a suite failure.

Protocol regeneration and parent/source binding were checked read-only by
calling the registrar's source verification, protocol builder, strict JSON
renderer, and Markdown renderer against the committed files.
The result was:

```text
COMMITTED_PROTOCOL_DETERMINISTIC_AND_BOUND_SOURCES_OK
16 top-level fields
8 exact sources
bound parent f3864177b9f36c75de2c7b650742394bedbaa3bc
```

The production runtime validator was called in validation-only mode for all
three replay declarations and the canonical four-command contract:

```text
EXACT_PROTOCOL_INVOCATIONS_AND_COMMAND_TOKENS_OK
smoke_a smoke_b registered_replay
registered_analyzer registered_replay smoke_a smoke_b
```

Read-only SHA-256 checks used `openssl dgst -sha256` on all eight sources and
the design.
Repository blobs were separately streamed from the bound parent and rehashed.
All values matched the protocol.
`git diff --quiet` confirmed no implementation drift from the bound parent;
`git diff --check` passed.
Final `git status --short` before writing this review contained only the
preserved untracked `build-diagnostic/`.

## Final verdict

**APPROVED, with 0 Critical, 0 Important, and 0 Minor findings.**

Execution is authorized only in the frozen Task 8 order:
commit this review, run the two exact distinct-root smokes and compare their
deterministic process identities, recheck the registered root and disk
preconditions, run the exact registered replay once, and only after its
successful terminal manifest run the exact registered analyzer once.
Any identity, byte, command, threshold, schema, cohort, denominator, path, or
precondition change invalidates this approval.
The paper-edit gate remains **CLOSED** through Stage 1.
