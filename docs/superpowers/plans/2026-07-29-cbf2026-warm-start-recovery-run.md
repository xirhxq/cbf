# CBF2026 Warm-Start Recovery Gate 2 Run Registration

## Registration status and authority

This document prospectively freezes one supervised four-cell replay and,
only after that replay registers a completed parent,
one paired comparison.
Neither production command has been executed.
The replay may be launched exactly once,
and the comparison may be launched exactly once.
Neither command may be retried under this registration.

Gate 1 is open.
The registered Gate 1 output has
`schema == "cbf2026-initialization-persistence-v1"`,
`status == "completed"`,
and `gate_passed == true`.
Its independent evidence review is `APPROVED / CLEAN`,
with zero Critical,
Important,
and Minor findings.
The exact authorization artifacts are:

| Artifact | SHA-256 |
| --- | --- |
| Gate 1 registration | `6003f25fa43a697b7abb3b3cec7d8735271b3bc721d8ab174f65f614aa7a8e05` |
| Gate 1 executor evidence | `c1d9be8b1d22d335dccf91bbad8ce05952e543768d2d3c1471a671b4690b694c` |
| Gate 1 independent review | `3f15b6247042284c49f22625d0c9dfc1ccf7b2f90c57ff28feae4dfb069a122d` |
| Gate 1 JSON | `ac06baa585427e0ae293c37d0d61fa30a36e8d60c2986b86c2573c77dee02cca` |
| Gate 1 Markdown | `9683efe8fa066ef96733f0d7ee554cf983c32b528f7730235f7f20c96bbaf7f2` |

Production remains prohibited until an independent Task 6 code/protocol
review reports no Critical or Important finding,
the full final preflight below passes,
and the tracked worktree is clean.
This registration authorizes no estimator,
graph,
measurement,
trajectory,
seed,
policy,
threshold,
controller,
CBF,
paper,
output-path,
or retry change.

## Frozen source identity and executables

The reviewed executable-code commit is
`6646a6b7476c48ebd314778495ebce2c4e04202e`
on branch `codex/cbf2026-diagnostic`.
The production environment is conda environment `cbf_env`.

| Executable source | SHA-256 |
| --- | --- |
| `scripts/diagnostics/run_warm_start_recovery.py` | `2e144d552fdf3e7d2cef63a4402c3b1e911e9d6cc3544176c7112e6614bba6f1` |
| `scripts/diagnostics/replay_localization_calibration.py` | `5c246b6b09be9cacbeec6a2f94b1a22e89f2d066d7177423e95e363e4ea665ec` |
| `scripts/diagnostics/compare_warm_start_recovery.py` | `227f887dfa148500a16db6622b8b8cdcf7c5ba7d584eb717f1a0655eb0dbd12d` |
| `scripts/diagnostics/analyze_localization_failures.py` | `1e3a7b7cd615b258fb83af53f7264d7aff327685f42c85787d0d9ae7df0b6315` |
| `scripts/diagnostics/run_diagnostic.py` | `ccde94fa739649cb9b94a8304ef3867ae946f28b0e3a812d3feb3f30bb76fd23` |

The registration commit and the later protocol-review documentation commit
will change Git `HEAD` and the source snapshot recorded by the supervisor.
Those documentation-only commits are permitted,
but they may not change any frozen executable byte,
hash,
command,
input,
policy,
gate,
or output location in this document.
Immediately before launch,
all five executable hashes must be recomputed and equal this table,
the tracked worktree must be clean,
and the final review must bind the unchanged hashes.

## Frozen scientific inputs

The sole trajectory data file is:

```text
/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json
```

Its SHA-256 is
`3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527`.
The sole trajectory manifest is:

```text
/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json
```

Its SHA-256 is
`6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb`.
It records completed source commit
`cff3852831f969ad4bedf7d9b2f43b4f2a0cf42f`,
materialized configuration SHA-256
`cb13a87615da21e2f52ab41d039f41c4547e8d7311f1f2b791afe8474d10153d`,
and source snapshot SHA-256
`7edf5cad0ef182ac7693ce4dadf3bb6e9685facc701e99558975623c31bc066d`.

The sole immutable strict anchor is:

```text
/private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288
```

| Immutable baseline artifact | SHA-256 |
| --- | --- |
| `manifest.json` | `39ee3b5494ed623d20d16766244427476cd081ed0869feecb9e5072c6c8f481d` |
| `summary.json` | `f4f780dd4c9c1399d3f838c74054c9c0ba3159415bc13089aee53d0ebf7250bc` |
| `summary.md` | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |
| `calibration.jsonl.gz` compressed bytes | `38325523e395983d83491c81556bac1aae72ebd99508b3cdf148fdf72f3f8803` |
| Decompressed `calibration.jsonl` stream bytes | `2c8a3f0f5684a281d5fbef3d7facea16f45d757d96b458827928dcac9e241a4f` |

The baseline manifest must remain completed,
must retain estimator contract
`variable_weight_nls_full_residual_jacobian_v1`,
and must retain the same input data and trajectory-manifest hashes above.

The seed order is exactly:

```text
20260727, 20260728, 20260729, 20260730, 20260731,
20260732, 20260733, 20260734, 20260735, 20260736,
20260737, 20260738, 20260739, 20260740, 20260741,
20260742, 20260743, 20260744, 20260745, 20260746
```

Every seed is used exactly once per policy.
The registered frame limit is exactly `500`.
No seed may be added,
removed,
reordered,
or rerun in response to an outcome.

## Frozen replay command

The replay output root is exactly:

```text
/private/tmp/cbf2026-warm-start-recovery
```

It was absent at registration preflight.
The sole registered supervisor invocation is:

```bash
conda run -n cbf_env python -m scripts.diagnostics.run_warm_start_recovery \
  --data /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  --input-manifest /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json \
  --immutable-baseline /private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288 \
  --output-root /private/tmp/cbf2026-warm-start-recovery \
  --seed 20260727 \
  --seed 20260728 \
  --seed 20260729 \
  --seed 20260730 \
  --seed 20260731 \
  --seed 20260732 \
  --seed 20260733 \
  --seed 20260734 \
  --seed 20260735 \
  --seed 20260736 \
  --seed 20260737 \
  --seed 20260738 \
  --seed 20260739 \
  --seed 20260740 \
  --seed 20260741 \
  --seed 20260742 \
  --seed 20260743 \
  --seed 20260744 \
  --seed 20260745 \
  --seed 20260746 \
  --max-frames 500
```

The supervisor must import one committed replay implementation and call it
once for each policy,
in this order:

1. `strict_previous_v1`;
2. `deployment_restart_before_first_finite_v1`.

The parent schema is
`cbf2026-warm-start-recovery-parent-v1`.
The child replay implementation identity is
`cbf2026-localization-calibration-v3`.
The estimator contract remains
`variable_weight_nls_full_residual_jacobian_v1`.
Both children must use identical data,
trajectory manifest,
seed order,
frame count,
dynamic and fixed graph cases,
reference membership,
measurements,
WNLS,
FIM,
and coefficient-3 epsilon construction.
Only pre-first-acquisition initialization selection may differ.

The supervisor prints its terminal parent manifest as JSON.
A usable replay requires process exit `0`,
`termination_reason == "completed"`,
exactly the two registered children,
and both child manifests completed.
Any nonzero exit,
exception,
child failure,
integrity failure,
cap failure,
or other terminal reason consumes the sole replay invocation.
Record it and stop without a replay retry or comparator launch.

## One-time replay-to-comparator handoff

The timestamped paired parent cannot be known before the one supervisor
invocation.
The following handoff is therefore frozen:

1. Accept only the absolute `output_dir` value printed in the terminal JSON
   of the single registered supervisor invocation.
   Do not discover a directory by listing,
   sorting,
   globbing,
   selecting “latest,”
   or accepting any other candidate.
2. Require that printed JSON to have `termination_reason == "completed"` and
   require the printed directory to contain its regular `manifest.json`.
3. Copy that exact absolute directory string into the executor evidence as
   `RECORDED_COMPLETED_PARENT_OUTPUT_DIR`.
4. Compute SHA-256 of
   `RECORDED_COMPLETED_PARENT_OUTPUT_DIR/manifest.json`
   read-only exactly once for this external handoff.
   Record the result in executor evidence as
   `RECORDED_PAIRED_PARENT_MANIFEST_SHA256`
   before invoking the comparator.
   Do not hash an alternate parent candidate.
   The one registered external-hash command template is:

   ```bash
   shasum -a 256 RECORDED_COMPLETED_PARENT_OUTPUT_DIR/manifest.json
   ```

   Replace the uppercase path token with the literal already recorded in
   Step 3 and execute this command once.
5. Substitute those two recorded literals into the frozen comparator command
   template below.
   No other template field may change.

The comparator output is exactly:

```text
/private/tmp/cbf2026-warm-start-recovery-analysis/registered-gate-2
```

Both that final directory and its staging form were absent at registration
preflight.
The parent
`/private/tmp/cbf2026-warm-start-recovery-analysis`
was also absent;
the comparator is permitted to create only that exact parent beneath the
already existing `/private/tmp`.

The sole frozen comparator command template is:

```bash
conda run -n cbf_env python -m scripts.diagnostics.compare_warm_start_recovery \
  --paired-bundle-dir RECORDED_COMPLETED_PARENT_OUTPUT_DIR \
  --immutable-baseline-dir /private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288 \
  --output-dir /private/tmp/cbf2026-warm-start-recovery-analysis/registered-gate-2 \
  --expected-paired-parent-manifest-sha256 RECORDED_PAIRED_PARENT_MANIFEST_SHA256 \
  --expected-baseline-manifest-sha256 39ee3b5494ed623d20d16766244427476cd081ed0869feecb9e5072c6c8f481d \
  --expected-comparator-source-sha256 227f887dfa148500a16db6622b8b8cdcf7c5ba7d584eb717f1a0655eb0dbd12d \
  --expected-failure-analyzer-source-sha256 1e3a7b7cd615b258fb83af53f7264d7aff327685f42c85787d0d9ae7df0b6315
```

The two uppercase handoff tokens are placeholders,
not environment variables or shell expressions.
They must be replaced by the exact literal values already recorded in the
executor evidence.
The comparator may be invoked only after that evidence entry exists.
Any comparator error consumes its sole invocation;
record it and stop without retry.

## Frozen policies, outcomes, and gates

The comparison schema is
`cbf2026-warm-start-recovery-comparison-v1`.
The statistical unit is one paired range-noise seed.
The strict child must reproduce every one of the immutable baseline's
`280,000` process rows in exact key order after removing only:

```text
initialization_policy
initial_estimate_source
ever_acquired_finite_before_attempt
```

Any other strict-arm difference is an implementation-integrity failure and
stops before policy statistics.
Paired strict and restart rows must preserve exact keys,
truth,
active references,
and measurement-noise identity.

The confirmatory primary event applies only to `dynamic_dag_wnls` and is
exactly:

```text
primary_statistics == true
attempt_status == "invalid"
attempt_failure_reason == "non-finite or malformed WNLS input"
```

For each seed and policy,
the denominator is every row with `primary_statistics == true`,
including converged,
failed,
and invalid attempts.
The paired effect is restart fraction minus strict fraction.
The comparator reports all seed-level records,
aggregate percentage-point and relative differences,
median paired difference,
and a two-sided percentile bootstrap 95% interval.
The bootstrap is exactly `10,000` resamples,
RNG seed `20260729`,
and `20` paired-seed draws with replacement per resample.

The dynamic primary gate passes only if both:

1. the bootstrap interval upper endpoint is strictly below zero; and
2. the aggregate exact-event count reduction is at least `0.90`.

Only if the primary gate passes may the hierarchical
`invalid upstream UAV reference` secondary be inferred.
Cascade interruption requires its paired bootstrap upper endpoint to be
strictly below zero.
The `fixed_refs_wnls` case is descriptive only.
The broader non-upstream invalid class is a prespecified component check and
cannot replace the exact primary event.

For both graph cases,
the comparator reports converged attempts,
WNLS nonconvergence,
conditional coefficient-3 epsilon containment,
conditional \(q>5.991\) and \(q>9\),
error-to-epsilon-ratio quantiles,
maximum registered event streaks,
and restart provenance.
The dynamic calibration safeguards pass only if the conditional \(q>9\)
rate increases by at most `2` percentage points and conditional
coefficient-3 epsilon containment falls by at most `2` percentage points.
Advancement to a separately registered multi-trajectory stage requires the
dynamic primary gate and both safeguards.

The comparator may publish exactly:

```text
warm-start-recovery-comparison.json
warm-start-recovery-comparison.md
```

A passing result supports only a single-trajectory,
paired-noise statement about recovery of the registered offline replay
initialization mechanism and,
if the hierarchical secondary also passes,
reduced observed downstream unavailability.
It does not support graph superiority,
controller or CBF guarantees,
safety,
mission success,
coefficient-3 epsilon sufficiency,
production-estimator robustness,
or generalization across trajectories or geometries.

## Frozen disk, publication, and stop contract

| Gate | Frozen limit |
| --- | ---: |
| Available bytes before each production launch | at least `8,000,000,000` |
| Live available-byte floor | stop below `6,000,000,000` |
| Allocated bytes under a production output root | at most `2,000,000,000` |
| Allocated bytes for the paired replay parent | at most `250,000,000` |
| Allocated bytes for paired comparison output | at most `10,000,000` |
| Streamed-row live probe interval | at least every `10,000` rows |

The supervisor also probes before allocation,
before and after source snapshot creation,
between children,
at child frame checkpoints,
before manifest publication,
and after persistent writes.
The comparator probes before staging,
during streaming,
before and after each write,
and after atomic no-replace publication.
All immutable and executable trust roots are reverified before completed
comparison publication.

If launch space is below `8,000,000,000` bytes,
do not launch.
Only the exact rebuildable cache
`/Users/xirhxq/.cache/codex-runtimes`
may be considered for separately recorded cleanup;
no evidence,
Git object,
document,
source,
or unrelated cache may be removed.
Cleanup was not performed during registration.
After any allowed pre-launch cache cleanup,
repeat the complete final preflight before the first production invocation.

A controlled supervisor failure may leave only its timestamped terminal
failure parent and must not claim completion.
A comparator failure must not leave the registered completed comparison
directory.
No production command is automatically or manually retried under this
registration.

## Registration preflight record

At executable commit
`6646a6b7476c48ebd314778495ebce2c4e04202e`,
the following read-only preflight completed:

- `212` tests passed in `8.489` seconds;
- Python byte-compilation passed for the Gate 1 analyzer,
  replay,
  supervisor,
  and comparator;
- `git diff --check` passed;
- tracked Git status was clean;
- the only status entry was the tolerated unrelated
  `?? build-diagnostic/`;
- the supervisor output root and comparator final,
  staging,
  and parent paths were absent;
- `/private/tmp` had `8,069,160,960` available bytes at the final recorded
  probe,
  above the `8,000,000,000` launch threshold; and
- `/Users/xirhxq/.cache/codex-runtimes` occupied `1,591,384` KiB
  (`1,629,577,216` bytes) and was not changed.

Immediately before the single replay invocation,
and again before the single comparator invocation,
record:

```bash
conda run -n cbf_env python -m unittest discover -s tests -p 'test_*.py' -v
conda run -n cbf_env python -m py_compile \
  scripts/diagnostics/analyze_initialization_persistence.py \
  scripts/diagnostics/replay_localization_calibration.py \
  scripts/diagnostics/run_warm_start_recovery.py \
  scripts/diagnostics/compare_warm_start_recovery.py
git diff --check
git status --short --untracked-files=no
df -Pk /private/tmp
du -sk /Users/xirhxq/.cache/codex-runtimes 2>/dev/null
```

The full suite must pass,
all frozen executable and input hashes must match,
tracked status must be empty,
the exact next output path must be absent,
and available bytes must meet the launch threshold.
The unrelated untracked `build-diagnostic/` remains outside the tracked-state
test and must never be staged,
deleted,
or rewritten.
