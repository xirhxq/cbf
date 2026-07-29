# Independent Gate 1 Evidence Review — Initialization Persistence

Date: 2026-07-29

Scope: the frozen Gate 1 registration, executor draft, Task 2 report,
immutable source bundle, and published derived bundle. This review did not
rerun, import, or call
`scripts.diagnostics.analyze_initialization_persistence`.

## Decision

**APPROVED / CLEAN.**

The independently streamed source reproduces the published Gate 1 result.
The dynamic case has `15,469 / 15,469` exact-reason rows in the
exclusive-prior category, and the fixed case has `14,471 / 14,471`.
All three other frozen categories are zero in both cases. The per-case 95%
dominance, zero-compound, and invalid-reason reconciliation gates pass.
No Critical, Important, or Minor finding was identified.

| Severity | Findings |
| --- | ---: |
| Critical | 0 |
| Important | 0 |
| Minor | 0 |

## Immutable inputs and outputs

The reviewed control documents were:

| Review input | SHA-256 |
| --- | --- |
| Run registration | `6003f25fa43a697b7abb3b3cec7d8735271b3bc721d8ab174f65f614aa7a8e05` |
| Executor evidence draft | `de985c9d5b2819da8b472166c07c9ecedbeddf61d62e0a783d25448b47da820b` |
| Task 2 report | `ac9f5c74900f568dc958be04993685cac236a69a81b2b0aba94c8f38964b1392` |

The registration freezes analyzer commit
`567b39d330d7ab4afb1169f0a3085574dc600ea0` and analyzer SHA-256
`0c9a19ec97c2b58c337abf30bc342f4e6a78f347593dcb0567e566be9a8050eb`.
The file at execution HEAD
`59a591f38b024d69c5803c66a34d83d934285dff` has that hash; the analyzer
has a zero-byte diff between the frozen commit and execution HEAD.

The source bundle is:

```text
/private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288
```

The independently recomputed source hashes are:

| Source artifact | SHA-256 |
| --- | --- |
| `manifest.json` | `39ee3b5494ed623d20d16766244427476cd081ed0869feecb9e5072c6c8f481d` |
| `calibration.jsonl.gz` compressed bytes | `38325523e395983d83491c81556bac1aae72ebd99508b3cdf148fdf72f3f8803` |
| Decompressed `calibration.jsonl` stream bytes | `2c8a3f0f5684a281d5fbef3d7facea16f45d757d96b458827928dcac9e241a4f` |
| `summary.json` | `f4f780dd4c9c1399d3f838c74054c9c0ba3159415bc13089aee53d0ebf7250bc` |
| `summary.md` | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |

All five source hashes were recomputed before and after the row audit and
were unchanged. The four artifact hashes embedded in the manifest match the
corresponding bytes. The manifest states
`termination_reason == "completed"` and estimator contract
`variable_weight_nls_full_residual_jacobian_v1`; its settings equal the
summary settings.

The derived bundle is:

```text
/private/tmp/cbf2026-initialization-persistence/registered-gate-1
```

| Derived artifact | Content bytes | SHA-256 |
| --- | ---: | --- |
| `initialization-persistence.json` | 279,387 | `ac06baa585427e0ae293c37d0d61fa30a36e8d60c2986b86c2573c77dee02cca` |
| `initialization-persistence.md` | 661 | `9683efe8fa066ef96733f0d7ee554cf983c32b528f7730235f7f20c96bbaf7f2` |

These hashes and byte counts match the executor draft and Task 2 report.
The JSON declares schema `cbf2026-initialization-persistence-v1`, status
`completed`, and `gate_passed: true`.

## Independent method

The review used a separate standard-library streaming audit. It did not
import production analyzer code or call its public API or CLI.

The audit:

1. parsed `manifest.json`, `summary.json`, the derived JSON, and each gzip
   JSONL record with duplicate-key and nonstandard-constant rejection;
2. recursively rejected every non-finite parsed numeric value;
3. hashed the compressed source, each decompressed raw line, both summaries,
   the manifest, and both derived artifacts independently;
4. reconstructed the complete canonical cursor as
   `(frame_index, seed, graph_case, robot_id)`, using manifest order for the
   20 seeds and two graph cases, 500 frames, and robot IDs `1..14`;
5. required `primary_statistics == (frame_index != 0)`;
6. maintained only the preceding row for each
   `(seed, graph_case, robot_id)`;
7. counted a primary broad-invalid row only when
   `attempt_status == "invalid"` and the reason was not
   `invalid upstream UAV reference`;
8. classified every invalid row with exact reason
   `non-finite or malformed WNLS input` from the preceding estimate and the
   current row's recorded `measurements[*].noisy_range`; and
9. compared the independently reproduced aggregates and gate predicates with
   the published JSON and Markdown.

The audit observed 280,000 rows and 280,000 unique canonical keys, with zero
missing, duplicate, extra, or out-of-order key and zero primary-flag
mismatch. Strict JSON and finite-value checks passed.

## Reproduced counts

Each graph case has 140,000 total rows and 139,720 primary rows.

| Graph case | All primary invalid | Excluded upstream invalid | Broad invalid | Exact reason | Other non-upstream invalid |
| --- | ---: | ---: | ---: | ---: | ---: |
| `dynamic_dag_wnls` | 66,367 | 50,898 | 15,469 | 15,469 | 0 |
| `fixed_refs_wnls` | 57,385 | 42,914 | 14,471 | 14,471 | 0 |

The all-primary-invalid totals match the source summary. For each case,
`excluded upstream invalid + broad invalid == all primary invalid`, and
`exact reason + other non-upstream invalid == broad invalid`. There are zero
broad-invalid rows at frame zero, so the independently checked all-row and
primary-row broad totals are identical.

| Frozen category | Dynamic | Fixed |
| --- | ---: | ---: |
| `exclusive_prior_self_estimate_missing_or_malformed` | 15,469 | 14,471 |
| `prior_missing_and_recorded_measurement_invalid` | 0 | 0 |
| `recorded_measurement_missing_or_nonfinite` | 0 | 0 |
| `unresolved_compound_validator` | 0 | 0 |
| Classification total | 15,469 | 14,471 |

Thus every exact-reason row is classified exactly once. In both cases, the
preceding estimate is not a finite two-element estimate and every current
recorded noisy range is finite.

## Gate evaluation

| Gate | Dynamic | Fixed | Result |
| --- | ---: | ---: | --- |
| Exclusive-prior dominance | `15,469 / 15,469 = 1.0` | `14,471 / 14,471 = 1.0` | Pass; both are at least 0.95 |
| Compound count | 0 | 0 | Pass |
| Classification reconciliation | 15,469 | 14,471 | Pass |
| Broad-invalid trust-root reconciliation | 15,469 | 14,471 | Pass |

The source-integrity, canonical-order, strict-JSON/finite, output-shape, and
disk gates also pass. The independently evaluated final Gate 1 decision is
therefore `true`.

## Disk and output-shape record

The final output directory contains exactly the two registered regular files
and no other entry. The staging directory
`registered-gate-1.incomplete` is absent.

The two-file directory occupies 280 KiB, or 286,720 allocated bytes. This is
below the 10,000,000-byte Gate 1 run cap and the complete analysis root has
the same allocation, below the 2,000,000,000-byte root cap.

The executor draft and Task 2 report agree on the post-publication reading of
8,671,040 KiB, or 8,879,144,960 available bytes, above the registered
6,000,000,000-byte live floor. The independent review-time reading was
8,699,596 KiB, or 8,908,386,304 available bytes, also above that floor.
The Task 2 report separately records a pre-launch reading of 8,915,890,176
bytes, above the 8,000,000,000-byte launch threshold.

## Supported and unsupported claims

The evidence supports the following bounded statement: in this immutable
bundle, every primary broad-invalid event in both registered graph cases has
the exact WNLS-input reason, and every such row is exclusively associated in
the recorded fields with a non-finite or malformed preceding self estimate
while all current noisy-range measurements are finite.

The executor draft stays within that boundary. It reports counts,
classification, reconciliation, hashes, resource use, and a pending-review
status. It does not convert temporal association into a causal root-cause
claim and does not make or upgrade a manuscript, estimator-performance,
controller, CBF, safety, robustness, or mission-level claim.

This review does not establish that initialization persistence causes the
invalid attempts, that a warm-start change will remove them, or that any
paper claim should be changed. Those questions require a separately
registered intervention and evidence review. The executor's exactly-once and
no-retry statements are operational provenance recorded consistently in the
draft and Task 2 report; they are not inferred from row contents alone.

## Findings

### Critical

None.

### Important

None.

### Minor

None.
