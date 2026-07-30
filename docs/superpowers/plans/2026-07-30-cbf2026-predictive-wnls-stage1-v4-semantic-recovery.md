# Predictive WNLS Stage 1 v4 semantic recovery plan

> This plan supersedes the execution portion of the Stage 1 v3 recovery.
> The successful v3 replay and failed v3 analyzer remain immutable forensic
> evidence and must never be retried or reused as successful Stage 1 input.

## Frozen v3 boundary

The v3 protocol and preflight are committed at `5d7fbbb` and `2cf3ea7`.
The registered replay ran exactly once and completed `420000/420000` rows.
The registered analyzer then ran exactly once and failed before compact-output
publication with:

```text
ValueError:
  prediction-expiry violations differ from derived stale anchors
```

The retirement record is committed at `8f79e4d`:

```text
docs/diagnostics/reviews/
  2026-07-30-predictive-wnls-stage1-v3-analyzer-failure.md
```

These paths are permanently retired:

```text
/private/tmp/cbf2026-predictive-wnls-smoke-v3-a
/private/tmp/cbf2026-predictive-wnls-smoke-v3-b
/private/tmp/cbf2026-predictive-wnls-development/stage1-v3
/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v3
```

The v3 raw stream is forensic evidence only.
It cannot be rebound to a replacement analyzer because the
`fresh_reference_qualification` ablation was materially altered by the
producer's exact floating-point symmetry predicate.

## Root cause and invariant

Two independent semantic defects are fixed together because both are required
to make the frozen violation audit meaningful:

1. fresh legacy covariances with only machine-roundoff asymmetry were
   published as `fresh` but rejected by `reference_is_eligible`;
2. violations were appended in mandatory-then-optional order after active
   references were canonicalized in a different order.

The v4 single source of truth is:

- finite 2-D covariance is symmetric when it satisfies the frozen
  `rtol=1e-12`, `atol=1e-12` comparison;
- an accepted covariance is canonicalized as
  `0.5 * (Sigma + Sigma.T)` before publication or propagation;
- material asymmetry remains invalid;
- `freshness` is a lifecycle label and never substitutes for full reference
  eligibility;
- outer violation records have a declared canonical
  `(reference kind rank, reference id, reason)` order,
  where `base < uav`,
  identifiers ascend numerically,
  and reasons use the frozen enum and lexicographic order;
- analyzer integrity is reconstructed from current-frame lower-index public
  outputs and measurement presence, not trusted from serialized `eligible`;
- duplicate, missing, extra, noncanonical, or semantically inconsistent
  violation records fail closed.

No estimator constant, noise seed, truth trajectory, baseline process,
ablation definition, scientific gate, or result threshold may be changed.

## Task A: TDD the covariance/publication boundary

Add failing tests before implementation:

1. a fresh SPD covariance with the observed `1.39e-17` off-diagonal
   difference is eligible;
2. the accepted covariance is returned in exact symmetric canonical form;
3. a materially asymmetric covariance remains invalid;
4. a legacy fresh result with roundoff asymmetry remains eligible for the
   next lower-index reference and is not excluded by the qualified variant;
5. every producer-published `fresh` row contains an exactly symmetric
   covariance equal to the shared canonicalizer's returned value;
6. a tolerance-valid but noncanonical raw fresh covariance is rejected by
   the analyzer;
7. predicted propagation begins from the canonical covariance stored in
   `current_public` and the private seed.

Implement one public shared covariance canonicalizer in
`scripts/diagnostics/predictive_wnls.py`.
Use its returned matrix,
not only its Boolean validity,
in fresh/predicted public-state validation and in `_finalize_public`.
Replace the covariance in both `current_public` and the private seed before
the current output is visible to the next UAV and before row serialization.
The analyzer and raw audit must additionally require
`np.array_equal(Sigma, Sigma.T)` and equality with the canonicalizer output
for every published fresh or predicted covariance.
Do not modify
`scripts/diagnostics/replay_localization_calibration.py`;
its immutable SHA-256 remains:

```text
0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8
```

## Task B: TDD deterministic violation semantics

Add failing tests before implementation:

1. mandatory UAVs `[4,5]` plus optional UAV `[3]` serialize violations in
   canonical `[3,4,5]` order;
2. duplicate, missing, and extra violations are rejected;
3. analyzer reconstructs lower-index public-output eligibility rather than
   trusting serialized `eligible`;
4. a tampered `fresh`/`eligible` pair is rejected;
5. canonical actual violations exactly match independently reconstructed
   expected facts;
6. state resets at every variant,
   seed,
   and frame boundary,
   with regressions for cross-frame/seed/variant leakage;
7. same-index,
   future-index,
   provenance,
   freshness,
   measurement-presence,
   unknown-kind,
   unknown-reason,
   duplicate,
   missing,
   and extra-reference tampering all fail closed.

Canonicalize producer violations only after mandatory and optional reference
processing is complete.
In the analyzer,
start with an empty current-output map for every exact
`(variant, seed, frame)` group;
require globally ascending robot ID within that group;
for each row,
reconstruct every referenced UAV only from complete public fields serialized
by an already accepted lower-index row in the same group
(`output_status`,
`prediction_age`,
`estimate`,
fresh or aged covariance,
`epsilon` or aged radius,
and provenance).
Validate current-row evidence against those reconstructed lower-index
outputs before adding the current row's public output to the group state.
Base eligibility is reconstructed separately from measurement validity.
Expected violations are derived only from active UAV keys whose independently
reconstructed reference output is ineligible.
Serialized `eligible` is checked against that result but is never its source.
Then reject duplicate facts and compare the canonical expected and actual
violation sequences exactly.

## Task C: version and regression closure

Advance the production identities to:

```text
protocol schema:
  cbf2026-predictive-wnls-stage1-protocol-v4
protocol ID:
  cbf2026-predictive-wnls-stage1-v4
protocol token:
  docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol-v4.json
raw schema:
  cbf2026-predictive-wnls-development-rows-v3
analysis schema:
  cbf2026-predictive-wnls-development-analysis-v3
smoke A:
  /private/tmp/cbf2026-predictive-wnls-smoke-v4-a
smoke B:
  /private/tmp/cbf2026-predictive-wnls-smoke-v4-b
registered replay:
  /private/tmp/cbf2026-predictive-wnls-development/stage1-v4
registered analyzer:
  /private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v4
```

Keep the hermetic protocol separate.
Add positive tests pinning raw schema v3 and analysis schema v3.
Add negative tests proving that no retired v2/v3 protocol schema,
protocol ID,
protocol token,
or v2/v3 execution root remains in current production declarations.
Prove that the v4 analyzer accepts only a v4 protocol and raw-schema-v3
manifest,
rejects every v3 protocol/raw-schema-v2 manifest,
and cannot rebind either preserved v3 forensic file to v4.

Run focused tests,
full discovery,
`py_compile`,
`git diff --check`,
direct-CLI help,
shadow-import regressions,
and the immutable legacy-solver hash check.
Obtain an independent implementation review with zero Critical and zero
Important findings before protocol registration.
Commit the implementation before generating the protocol.

## Task D: freeze and preflight v4

Generate the v4 Markdown and JSON protocol exactly once from the committed
implementation.
Require all four v4 roots absent and at least 8 GB free.
Use this exact registrar command once:

```bash
conda run -n cbf_env python \
  scripts/diagnostics/register_predictive_wnls_stage1.py \
  --repository-root /private/tmp/cbf2026-diagnostic \
  --output-markdown \
    docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol-v4.md \
  --output-json \
    docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol-v4.json
```

Independently verify deterministic bytes,
the non-circular implementation parent,
all frozen sources,
20 seeds,
three cumulative variants,
all estimator constants,
all nine scientific gates,
the new covariance/violation semantic contract,
disk limits,
and four canonical command arrays.

Commit protocol files separately.
Then create and commit:

```text
docs/diagnostics/reviews/
  2026-07-30-predictive-wnls-stage1-protocol-v4-review.md
```

No v4 execution root may exist before the preflight approval commit.
The v3 replay raw SHA
`57503b331f52ac036f88d33b4377d093d624fc9bf5283286e452af819b72f88b`,
v3 failed-analyzer manifest SHA
`ad5fb00f0b64b75446c84caf97a9313803c1f31352cc58a7408f4d1eda67b69c`,
and all four retired v3 paths must remain unchanged throughout v4.

## Task E: execute v4 exactly once

Immediately before each of `smoke_a`,
`smoke_b`,
registered replay,
and registered analyzer,
repeat the clean-source/protocol checks,
require the exact target root absent,
and require at least `8_000_000_000` free bytes.
During each invocation,
stop below `6_000_000_000` free bytes,
keep rebuildable cache and raw evidence below `2_000_000_000` allocated
bytes,
and never delete a terminal or retired evidence root.

Run the exact committed `smoke_a` then `smoke_b` arrays,
once each,
with new roots.
Require `84/84` rows,
completed manifests,
identical compressed and decompressed process hashes,
canonical violations,
and zero fresh-but-ineligible public outputs.
Preserve both roots.
If either smoke fails,
preserve its terminal state,
do not run or retry the other smoke or any registered command,
retire the entire v4 protocol and all four v4 roots,
commit a failure record,
update DRA,
and stop this execution plan.

Only after smoke acceptance,
run the exact registered replay once.
Require `420000/420000` rows,
exact hashes,
all fixed keys,
the 2 GB cap,
canonical violations,
and zero fresh-but-ineligible public outputs.
On failure,
preserve terminal state,
retire v4 without retry,
commit a failure record,
update DRA,
and stop before analysis.

Only after registered replay acceptance,
run the exact registered analyzer once.
On failure,
preserve terminal state,
retire v4 without retry,
commit a failure record,
update DRA,
and stop before scientific reporting.

## Task F: audit and report

Independently audit:

- all source and process identities;
- exact fixed cohorts and denominators;
- `243/7000` input-component violations;
- `6986/6986` predecessor-command coverage;
- frame-44,
  legacy-999 m,
  and legacy-168 m fixed mechanisms;
- all nine predeclared gates;
- complete status transitions and attrition;
- raw 2 GB and compact 10 MB caps;
- `paper_gate=CLOSED`.

Create and independently review:

```text
docs/diagnostics/2026-07-30-predictive-wnls-stage1-v4.md
docs/diagnostics/reviews/
  2026-07-30-predictive-wnls-stage1-v4-evidence-review.md
```

Stage 1 remains a paired,
single-trajectory,
offline estimator development replay.
Even a successful v4 does not authorize paper edits.
It may only justify a separately frozen hard-input-bounded Stage 2.

Record the same lifecycle,
failure provenance,
scientific limits,
and approved v4 outcome in DRA.

DRA work is restricted to the isolated
`/private/tmp/dra-cbf2026-diagnostic` worktree on `main`.
Do not touch the dirty primary DRA checkout and do not push.
Update append-only:

```text
meta-log/2026-07-30-cbf2026-predictive-wnls-recovery.md
papers/cbf2026/status.md
papers/cbf2026/open-questions.md
papers/cbf2026/timeline.md
papers/cbf2026/submissions.md
```

Perform this DRA update for either a successful or failed v4 terminal
outcome.
Independently reconcile it against terminal manifests,
compact evidence when present,
the failure/evidence review,
and Git history.
Resolve every Critical or Important finding,
commit on isolated `main` without pushing,
and verify both the primary paper checkout and primary DRA checkout remain
unchanged.
