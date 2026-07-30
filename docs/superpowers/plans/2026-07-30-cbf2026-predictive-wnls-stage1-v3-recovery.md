# Predictive WNLS Stage 1 v3 recovery plan

> This plan supersedes only the execution portion of
> `2026-07-30-cbf2026-bounded-predictive-wnls-recovery.md`.
> The v2 protocol, preflight, and failed smoke invocations remain immutable
> historical evidence and must not be retried.

## Frozen failure boundary

The v2 protocol was committed at `5e5dd36`,
its independent preflight at `47426f5`,
and its exactly-once smoke failure is recorded by
`docs/diagnostics/reviews/2026-07-30-predictive-wnls-stage1-v2-smoke-failure.md`
at `7addd76`.
Both v2 smoke commands exited before output-root allocation because direct
execution could not import the repository `scripts` package.
The registered v2 replay and analyzer were never run.

The following paths and command tokens are retired even though the paths are
absent:

```text
/private/tmp/cbf2026-predictive-wnls-smoke-a
/private/tmp/cbf2026-predictive-wnls-smoke-b
/private/tmp/cbf2026-predictive-wnls-development/stage1-v2
/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v2
docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.json
```

## Invariants retained from the approved design

- The truth trajectory, input manifest, immutable baseline process, 20 seeds,
  three cumulative variants, estimator constants, gates, and all statistical
  denominators remain unchanged.
- `RAW_SCHEMA_ID` and the compact analysis schema remain at v2 because their
  serialized structures are unchanged.
- The production protocol schema and protocol ID advance to v3.
- Every v3 smoke and registered output uses a new exact root.
- The paper gate remains `CLOSED`.
- A failed registered v3 replay or analyzer is never retried.
- No paper edit is authorized by Stage 1.

## Task A: close the direct-CLI defect

The secure direct-mode bootstrap is committed at `22c740a`.
Replay and analyzer must:

- work when invoked as `python scripts/diagnostics/<script>.py --help`;
- force the resolved implementation repository to the trusted import
  namespace before any `scripts.*` import;
- reject a preceding regular `PYTHONPATH` shadow package;
- leave normal package-import `sys.path` unchanged.

Before continuing, require the direct-CLI and shadow-import regressions,
full discovery, `py_compile`, diff checks, and the immutable legacy solver
SHA-256
`0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8`.

## Task B: commit the distinct production v3 contract

Freeze these exact production identities in implementation and tests:

```text
protocol schema:
  cbf2026-predictive-wnls-stage1-protocol-v3
protocol ID:
  cbf2026-predictive-wnls-stage1-v3
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

Keep the invocation names
`smoke_a`, `smoke_b`, `registered_replay`, and `registered_analyzer`.
Add negative tests proving that no v2 token or target remains in the
production declaration.
Commit implementation and tests before generating the protocol.

## Task C: generate and commit the v3 protocol

Require clean tracked implementation files,
all frozen external hashes,
the four absent v3 output roots,
and at least 8 GB free.

Generate exactly once:

```bash
conda run -n cbf_env python \
  scripts/diagnostics/register_predictive_wnls_stage1.py \
  --repository-root /private/tmp/cbf2026-diagnostic \
  --output-markdown \
    docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol-v3.md \
  --output-json \
    docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol-v3.json
```

The registrar must fail if either v3 target already exists.
Independently verify deterministic bytes,
the non-circular implementation parent,
all eight source hashes,
all constants and gates,
the four canonical commands,
and absence of every v3 execution root.
Commit the two v3 protocol files separately.

## Task D: independent v3 preflight

Create and commit:

```text
docs/diagnostics/reviews/
  2026-07-30-predictive-wnls-stage1-protocol-v3-review.md
```

The review repeats the runtime truth-boundary,
predecessor-command,
fresh DAG/provenance,
re-acquisition,
all-row retention,
fixed-cohort denominator,
source identity,
terminal lifecycle,
disk-cap,
no-retry,
and no-outcome-tuning audits.
It must additionally execute the exact replay and analyzer direct CLIs in
`--help` or another validation-only mode and cite the shadow-import tests.
Resolve every Critical or Important finding before committing approval.
Do not create any v3 smoke or registered root before this review commit.

## Task E: run the v3 smoke pair exactly once

Require the two roots to be absent:

```text
/private/tmp/cbf2026-predictive-wnls-smoke-v3-a
/private/tmp/cbf2026-predictive-wnls-smoke-v3-b
```

Run the exact `commands.smoke_a` and `commands.smoke_b` token arrays from the
committed v3 JSON, in that order, once each.
Do not reconstruct or modify the commands.
For each root require a `completed` terminal manifest,
`84` exact rows,
and verified compressed and decompressed hashes.
The two process hashes must match;
only timestamps and absolute output-root metadata may differ.
Preserve both smoke roots.

If either smoke fails,
record and retire the v3 protocol without retrying either command.

## Task F: run registered v3 evidence exactly once

Immediately before launch require:

- clean protocol-bound implementation and protocol bytes;
- absent registered replay and analyzer roots;
- at least 8 GB free;
- the exact committed v3 command arrays.

Run `commands.registered_replay` exactly once.
On any unsuccessful terminal state,
preserve it and stop before analysis.

Only after a successful replay manifest,
run `commands.registered_analyzer` exactly once.
On failure,
preserve it and do not retry.

The registered output roots are:

```text
/private/tmp/cbf2026-predictive-wnls-development/stage1-v3
/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v3
```

## Task G: audit, report, and update DRA

Independently audit raw and compact evidence,
including all source and process hashes,
row counts,
fixed-key cohorts,
`243/7000` input-limit violations,
`6986/6986` predecessor coverage,
the frame-44, 999 m, and 168 m mechanisms,
all predeclared gates,
and output sizes.

Create:

```text
docs/diagnostics/2026-07-30-predictive-wnls-stage1-v3.md
docs/diagnostics/reviews/
  2026-07-30-predictive-wnls-stage1-v3-evidence-review.md
```

Record the same lifecycle and scientific conclusions in DRA.
Stage 1 remains one-trajectory development evidence.
Only a successful, independently approved Stage 1 may justify a separate
hard-input-bounded Stage 2 plan;
it never opens the paper-edit gate by itself.
