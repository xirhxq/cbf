# Initialization-persistence Gate 1 evidence

## Execution record

The registered Gate 1 production command was executed once on the approved
source at HEAD `59a591f38b024d69c5803c66a34d83d934285dff` and exited with
status `0`. It was not retried.

```bash
conda run -n cbf_env python -m scripts.diagnostics.analyze_initialization_persistence \
  --bundle-dir /private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288 \
  --output-dir /private/tmp/cbf2026-initialization-persistence/registered-gate-1 \
  --expected-manifest-sha256 39ee3b5494ed623d20d16766244427476cd081ed0869feecb9e5072c6c8f481d \
  --expected-summary-json-sha256 f4f780dd4c9c1399d3f838c74054c9c0ba3159415bc13089aee53d0ebf7250bc \
  --expected-compressed-process-sha256 38325523e395983d83491c81556bac1aae72ebd99508b3cdf148fdf72f3f8803 \
  --expected-dynamic-broad-invalid-total 15469 \
  --expected-fixed-broad-invalid-total 14471
```

The compact output root is:

```text
/private/tmp/cbf2026-initialization-persistence/registered-gate-1
```

| Artifact | Content bytes | SHA-256 |
| --- | ---: | --- |
| `initialization-persistence.json` | 279,387 | `ac06baa585427e0ae293c37d0d61fa30a36e8d60c2986b86c2573c77dee02cca` |
| `initialization-persistence.md` | 661 | `9683efe8fa066ef96733f0d7ee554cf983c32b528f7730235f7f20c96bbaf7f2` |

The directory occupied 280 KiB (286,720 allocated bytes), within both the
10,000,000-byte Gate 1 run cap and the 2,000,000,000-byte output-root cap.
After publication, `/private/tmp` had 8,671,040 KiB available
(8,879,144,960 bytes), above the 6,000,000,000-byte live floor.

## Analyzer-reported result

The published JSON has schema `cbf2026-initialization-persistence-v1`, status
`completed`, and reports `gate_passed: true`. Its per-case values are:

| Graph case | Broad invalid rows | Exact-reason rows | Exclusive-prior rows | Dominance | Compound rows | Analyzer gate |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| `dynamic_dag_wnls` | 15,469 | 15,469 | 15,469 | 1.0 | 0 | pass |
| `fixed_refs_wnls` | 14,471 | 14,471 | 14,471 | 1.0 | 0 | pass |

For both graph cases, the other two frozen categories
`recorded_measurement_missing_or_nonfinite` and
`unresolved_compound_validator` are also zero. Thus the analyzer's recorded
broad-invalid totals reconcile to the registered trust-root values, all
exact-reason rows fall in the exclusive-prior category, the 0.95 per-case
dominance gate is satisfied, and the zero-compound-row gate is satisfied.

## Review status

The required independent evidence review is
[`2026-07-29-initialization-persistence-review.md`](reviews/2026-07-29-initialization-persistence-review.md).
It returned `APPROVED / CLEAN`, with zero Critical, Important, and Minor
findings. Its separate streaming audit reproduced 280,000 unique canonical
rows, all source and derived hashes, per-case reconciliation and category
counts, 1.0 dominance fractions, zero compound rows, and 286,720 allocated
output bytes. The independently evaluated final Gate 1 decision is `true`.

This result remains a bounded retrospective temporal classification of the
immutable bundle. It does not itself establish causality or justify a paper,
estimator-performance, controller, CBF, safety, robustness, or mission-level
claim change.
