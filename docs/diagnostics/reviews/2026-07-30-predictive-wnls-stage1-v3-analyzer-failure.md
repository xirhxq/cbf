# Predictive WNLS Stage 1 v3 analyzer failure record

## Immutable outcome

The Stage 1 v3 registered replay completed successfully,
but the registered analyzer is retired after its predeclared command reached
an internal aggregate-consistency assertion and exited with status `1`.
The replay command and analyzer command were each invoked exactly once.
Neither v3 command may be retried,
and neither registered v3 output root may be reused by a replacement
protocol.

The failed analyzer published no compact scientific result.
Consequently,
none of the nine predeclared estimator gates has an authorized v3 outcome,
and the paper-edit gate remains closed.

## Frozen protocol and approval

The machine-readable v3 protocol was committed by:

```text
5d7fbbb88ef6a08d25e5ebca587e03c182935e2d
docs(diagnostics): freeze predictive WNLS Stage 1 v3 protocol
```

Its independent preflight approval was committed by:

```text
2cf3ea7a97f897263670afa9ab673fb8d94677a8
docs(diagnostics): approve predictive WNLS Stage 1 v3 preflight
```

The frozen protocol files had the independently recorded SHA-256 identities:

```text
d8b4aad1c383b55f6171435d69606fe07409f491841fc81d82e2936bd1d957e8  docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol-v3.json
130a240ecd6eaf391ebc8338b6ca530afe2c464cc6276a37c2008fd636223175  docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol-v3.md
```

Immediately before each registered invocation,
the repository was at the exact approved preflight commit.
Tracked files were clean,
`build-diagnostic/` was the only untracked path,
all eight protocol-bound source identities matched their live bytes,
the intended output root was absent,
and the launch-time free-space requirement was satisfied.

## Successful registered replay

The exact frozen `commands.registered_replay` token array was invoked once.
It exited with status `0` after `614.096 s`.
Its terminal bundle is preserved at:

```text
/private/tmp/cbf2026-predictive-wnls-development/stage1-v3
```

The independent terminal audit verified:

```text
status:                    completed
rows_written:              420000
expected_rows:             420000
rows per variant:          140000
raw logical bytes:         186277769
allocated bundle bytes:    201379840
raw bundle cap bytes:      2000000000
compressed SHA-256:        57503b331f52ac036f88d33b4377d093d624fc9bf5283286e452af819b72f88b
decompressed SHA-256:      e842e84a29393d2ef8b70a9275af41c102188d8809204af89e1c7b012fea5224
free bytes before:         48256057344
free bytes after:          48066084864
```

The directory contains exactly `manifest.json` and
`predictive-wnls-development.jsonl.gz`.
All `20 seeds x 500 frames x 14 robots x 3 variants` fixed keys are unique
and complete.
No staging,
quarantine,
or analyzer output existed at replay acceptance.

## Exactly-once registered analyzer failure

Only after the replay terminal audit passed,
the exact frozen `commands.registered_analyzer` token array was invoked once:

```text
conda run -n cbf_env python scripts/diagnostics/analyze_predictive_wnls_recovery.py --baseline-process-path /private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/restart/localization-calibration/20260729T112437.413270Z_efb8e7137776415c833f11204cf519d5/calibration.jsonl.gz --development-manifest-path /private/tmp/cbf2026-predictive-wnls-development/stage1-v3/manifest.json --protocol-json docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol-v3.json --expected-baseline-sha256 c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003 --output-root /private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v3
```

It exited with status `1` at:

```text
main
  -> analyze_predictive_recovery
  -> aggregate_predictive_recovery
  -> ValueError:
       prediction-expiry violations differ from derived stale anchors
```

The terminal analyzer root is preserved at:

```text
/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v3
```

It contains exactly one `manifest.json`,
whose SHA-256 is:

```text
ad5fb00f0b64b75446c84caf97a9313803c1f31352cc58a7408f4d1eda67b69c
```

The terminal manifest records:

```text
status:                    failed
allocated_bytes:           0
outputs:                   {}
error type:                ValueError
error message:             prediction-expiry violations differ from derived stale anchors
development manifest SHA:  3297b25bf569a9748f4314be4ab9c103a1319c95d08d1c7a0dabfe76c2d68a4b
raw process SHA:            57503b331f52ac036f88d33b4377d093d624fc9bf5283286e452af819b72f88b
free bytes before:         48054042624
free bytes after:          48052568064
```

No compact JSON,
Markdown,
partial output,
staging file,
or quarantine entry was created.

## Read-only failure localization

A streaming read-only comparison over the `140000`
`prediction_expiry` rows found `85562` rows for which the actual and
freshness-derived violation lists differ:

```text
freshness-derived list empty but actual nonempty:     72884
freshness-derived list nonempty but actual superset:  11339
same-set ordering-only mismatch:                       1339
```

The first terminal divergence is ordering.
Replay emits violations while visiting mandatory references before optional
references,
then separately canonicalizes active-reference keys.
The analyzer compares the serialized violation sequence against
active-reference order,
so otherwise equal violation sets can fail by order alone.

The full-stream audit also found a separate predicate divergence.
Replay records a `stale_or_predicted_anchor_used` violation when a selected
reference fails its full eligibility predicate,
whereas the analyzer reconstructs the expected violation only from
`current_freshness != fresh`.
One reproduced case has `current_freshness=fresh`,
but covariance off-diagonal entries differ after floating-point computation
by approximately `1.39e-17`.
The replay eligibility check uses exact matrix symmetry,
so that reference is marked ineligible even though its serialized freshness
label remains `fresh`.

These observations localize the failed assertion;
they do not authorize a repair,
a v3 retry,
or a scientific result.

## Retirement and remediation boundary

The successful v3 raw bundle and failed v3 analyzer root are immutable
historical evidence.
The registered v3 analyzer command and its output root are permanently
retired.

Before any replacement analysis,
the eligibility/freshness predicate and violation-order semantics must be
traced to a single declared source of truth,
covered by failing regression tests,
fixed in committed implementation,
and independently reviewed.
Any replacement analyzer must use a distinct protocol version,
command token,
and output root,
and must receive a new exactly-once approval.
Whether the immutable v3 raw stream is compatible with that replacement
must be decided from its declared schema and semantics,
not from desired gate outcomes.

No paper edit is authorized by this record.
