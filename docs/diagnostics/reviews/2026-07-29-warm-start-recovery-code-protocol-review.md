# Warm-start recovery independent code/protocol review

## Decision

**APPROVED for code and protocol.**

The fix-round review found zero open Critical findings,
zero open Important findings,
and no new findings.
All five Important findings from the first review are addressed.
The two previously identified Minor observations remain deferred and do not
change the registered claim boundary.

This approval does not authorize a production launch in the current disk
state.
The native full preflight currently fails closed because actual free space is
below the registered 8 GB launch threshold.
Production becomes authorized exactly once only after the conditional
preflight and cache-handling sequence below succeeds in full.

No production supervisor or comparator command was run during this review.
Neither registered production output root was created.

## Reviewed identities

The executable repair commit is
`f33035e828b892d4fec3e587928266356f82c599`.
The reviewed registration commit and current review base are
`9fde377d3654d08110954f54f4aa6501ca69e7d3`.
The branch is `codex/cbf2026-diagnostic`.
The registration document SHA-256 is
`f0744b955d4396ac7d43acbb9b115044f5b8ca8590cb6dc5a58c9e7b6c77ab6c`.

The later commit that records this review may change `HEAD` and the source
snapshot.
It may not change any frozen executable byte,
input,
command,
policy,
threshold,
or output location.

Recomputed executable hashes are:

| Artifact | SHA-256 |
| --- | --- |
| Supervisor, `scripts/diagnostics/run_warm_start_recovery.py` | `76733bb0748a63a63858ea24fc89240998f0dc1c58dbe96888da86d3c31e9ae5` |
| Replay, `scripts/diagnostics/replay_localization_calibration.py` | `0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8` |
| Comparator, `scripts/diagnostics/compare_warm_start_recovery.py` | `9a54c312183f7f2864aba104a5624ea247e15e1107deaa4590e276ab1eda120d` |
| Failure analyzer, `scripts/diagnostics/analyze_localization_failures.py` | `1e3a7b7cd615b258fb83af53f7264d7aff327685f42c85787d0d9ae7df0b6315` |
| Shared helper, `scripts/diagnostics/run_diagnostic.py` | `ccde94fa739649cb9b94a8304ef3867ae946f28b0e3a812d3feb3f30bb76fd23` |

Recomputed scientific-input and immutable-baseline hashes are:

| Artifact | SHA-256 |
| --- | --- |
| Trajectory data | `3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527` |
| Trajectory manifest | `6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb` |
| Baseline manifest | `39ee3b5494ed623d20d16766244427476cd081ed0869feecb9e5072c6c8f481d` |
| Baseline summary JSON | `f4f780dd4c9c1399d3f838c74054c9c0ba3159415bc13089aee53d0ebf7250bc` |
| Baseline summary Markdown | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |
| Baseline compressed stream | `38325523e395983d83491c81556bac1aae72ebd99508b3cdf148fdf72f3f8803` |
| Baseline decompressed stream | `2c8a3f0f5684a281d5fbef3d7facea16f45d757d96b458827928dcac9e241a4f` |

The decompressed-stream hash was independently recomputed from the compressed
artifact.
All hashes match the current run registration.

## Review method and verification

The review inspected the repair diff,
the complete current supervisor,
replay,
comparator,
helper and failure-analyzer sources,
their tests,
the Gate 2 registration,
and the earlier independent review.
Two independent review axes separately checked repository standards and the
frozen scientific/protocol specification.
Both reported zero open Critical or Important findings.

Fresh verification at the review base produced:

- focused replay/supervisor/comparator suite:
  85 tests passed;
- controlled full suite with only the disk-availability observation replaced
  by 9,000,000,000 bytes:
  219 tests passed in 9.918 seconds;
- native full suite:
  219 tests ran in 9.558 seconds,
  with exactly six `AnalysisLimitError` errors and no assertion failure;
- every native error reported available space below the exact
  8,000,000,000-byte start threshold;
- byte compilation of the repaired modules and their tests:
  passed;
- `git diff --check`:
  passed; and
- the full pre-review status contained only untracked paths under
  `build-diagnostic/`.

The controlled suite verifies the complete functional path.
The native result verifies that the real disk gate is not bypassed.
The native result is not waived:
the entire registered native preflight must pass after authorized cache
handling before any production command can run.

At the final review probe,
`df -Pk /private/tmp` reported 7,660,224 KiB available,
equal to 7,844,069,376 bytes.
This is 155,930,624 bytes below the 8,000,000,000-byte launch gate.
The exact authorized cache
`/Users/xirhxq/.cache/codex-runtimes`
occupied 1,583,072 KiB,
equal to 1,621,065,728 bytes.
The cache was measured but not changed by this review.

The following production roots remained absent:

```text
/private/tmp/cbf2026-warm-start-recovery
/private/tmp/cbf2026-warm-start-recovery-analysis
```

## Disposition of the five Important findings

| Prior finding | Disposition | Review evidence |
| --- | --- | --- |
| Supervisor lacked externally registered trust roots and comparator did not bind parent inputs to the anchored baseline | Addressed | The supervisor now requires and verifies six external hashes before allocation, records them, and rechecks them before completed publication. The comparator type-exactly binds parent data and trajectory-manifest records to the externally anchored baseline. Adversarial replacement tests pass. |
| Completed parent publication had a finalization TOCTOU gap | Addressed | A unified closure rechecks all six external roots, the full immutable baseline including decompressed stream, the source snapshot, and exact Git commit/branch/full-status state. It runs after both children and again immediately before atomic completed publication. Mutation regressions pass. |
| Nested supervised children reapplied the 8 GB launch gate | Addressed | Standalone replay retains the 8 GB start gate. A supervised child requires the parent probe and uses the 6 GB live floor without reapplying the start gate. The real four-cell end-to-end test observes one 8 GB parent gate and later 7 GB live observations. |
| Final preflight did not cover full untracked status and exact hash/path absence checks | Addressed | The registration now freezes full untracked-status validation, an exact `build-diagnostic/` allowlist, eight explicit SHA-256 checks, exact replay/comparator root and staging absence checks, allocation checks, and explicit 8 GB arithmetic. |
| No real compact four-cell supervisor end-to-end test | Addressed | The end-to-end test runs the real supervisor and both real child policies for both graph cases, preserving paired seeds, references, measurements, and noise while exercising the 8 GB/6 GB contract. |

## Deferred Minor observations

Two earlier Minor observations remain deferred:

1. The completed parent manifest records resource metrics before the final
   successful staging probe and performs no additional post-publication disk
   probe.
   The required safety probe still runs and can prevent publication;
   the limitation is only that the published minimum/free metrics can omit a
   later lower-but-safe observation.
2. During comparator nested failure analysis,
   the every-10,000-row scratch-space check uses the analyzer-local allocation
   guard rather than the comparator's encompassing staging-allocation guard.
   The comparator still applies its encompassing guard when the nested
   analysis returns and before publication.

Neither observation weakens the external trust roots,
the no-retry contract,
the launch/live hard floors,
the output caps,
or the frozen scientific claim.
They should remain recorded for later maintenance and must not be silently
represented as fixed.

## Conditional exactly-once production authorization

The registered supervisor remains unauthorized in the current state.
The following conditions are mandatory and sequential:

1. Because actual free space is below 8,000,000,000 bytes,
   capacity may be recovered only through the separately authorized cache
   target
   `/Users/xirhxq/.cache/codex-runtimes`.
   No production input,
   immutable baseline,
   repository file,
   `build-diagnostic/`,
   or other cache/output location is authorized for deletion.
   The exact cleanup action and before/after allocation and free-space values
   must be recorded in the executor evidence.
2. After that authorized cache handling,
   rerun the complete frozen final preflight in
   `docs/superpowers/plans/2026-07-29-cbf2026-warm-start-recovery-run.md`
   from its first command.
   It must exit zero with the native 219-test suite passing,
   byte compilation and `git diff --check` passing,
   the tracked worktree clean,
   every untracked line under `build-diagnostic/`,
   all frozen hashes matching,
   both production roots and every registered staging candidate absent,
   all allocation caps satisfied,
   and actual free space at least 8,000,000,000 bytes.
   A partial or resumed preflight is not sufficient.
3. Only after step 2 passes,
   the exact frozen supervisor command is authorized once.
   It may not be edited,
   retried,
   resumed,
   or replaced.
   Any nonzero exit,
   non-completed terminal manifest,
   missing terminal JSON,
   trust-root change,
   disk-limit event,
   or other failure stops the protocol and leaves the comparator
   unauthorized.
4. If and only if the supervisor completes,
   take the exact `output_dir` from that command's terminal JSON.
   Do not discover it through directory listing,
   sorting,
   globbing,
   or a `latest` convention.
   Record that literal parent path,
   hash that exact parent's `manifest.json` externally exactly once,
   and carry the literal path and one-time hash into the registered comparator
   command template.
5. Before invoking the comparator,
   rerun the complete registered comparator-stage native preflight.
   It must exit zero,
   reconfirm at least 8,000,000,000 actual free bytes,
   the literal parent identity and one-time manifest hash,
   the frozen source/input hashes,
   the parent and replay allocation caps,
   and absence of the comparator root,
   final directory,
   and staging candidates.
6. Only after step 5 passes,
   the exact instantiated comparator command is authorized once.
   It may not be retried.

Any executable,
input,
hash,
command,
policy,
threshold,
output-path,
or review-condition change invalidates this approval and requires a new
independent review.

## Handoff and claim boundary

The executor must preserve the terminal JSON,
the literal parent directory,
the one-time parent-manifest hash,
all preflight output,
the cache-handling record,
and both production command exit states.
The registered execution count is still zero for the supervisor and zero for
the comparator.

Even after successful replay and comparison,
the strongest authorized conclusion remains the registered
single-trajectory,
paired-noise,
offline-estimator claim.
This review authorizes no graph-superiority,
controller/CBF,
safety,
mission,
coefficient-3 radius-sufficiency,
production-robustness,
or cross-trajectory claim.
