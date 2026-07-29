# Independent review: localization-failure analyzer

Date: 2026-07-29  
Review scope: Tasks 1--4 through executable checkpoint `7e44ab4` (`fix(diagnostics): align reported protocol with execution`).

## Verdict

CLEAN for pre-registration. No active Critical, Important, or Minor findings remain in the reviewed analyzer and its Task 1--4 tests.

This is a static code-and-test review. It does not run the analyzer, inspect the production gzip, create an output directory, or validate any production result. Those source/output and counter checks remain the separate Task 6 evidence review.

## Evidence reviewed

The reviewed implementation is `scripts/diagnostics/analyze_localization_failures.py`, with its test coverage in `tests/test_analyze_localization_failures.py`. The Task 1--4 implementation history reviewed was:

- `600ce63`, `7d47d04`, `baca803`, and `920260b` for source-contract validation;
- `81306d8` and `eed0d7a` for the failure budget and lineage;
- `7f6d3ca`, `3ec3abc`, `8b0c8f4`, `5b8b1a1`, and `f713357` for conditional metrics, reconciliation, persistence, strata, and bootstrap safeguards; and
- `7fb0960`, `0a3df3f`, `62f6dd6`, and `7e44ab4` for deterministic output, provenance, protocol, and final review fixes.

The review also used the frozen design, the execution plan, Task 1--4 briefs and reports, and the generated review packages. Root verification reported the focused Task 4 suite as 12 passing tests, the complete Python suite as 157 passing tests, and successful `py_compile` and `git diff --check`. This reviewer did not rerun those commands.

## Findings

### Critical

None.

### Important

None.

### Minor

None.

## Design and implementation checks

### Input integrity and bounded streaming

The analyzer requires a completed manifest, the frozen estimator contract, matching manifest/summary settings, and hash-verified process and small source files. It validates strict JSON lines while streaming the gzip, advances the canonical `frame -> seed -> graph case -> robot_id` key cursor, reconciles both primary and frame-zero status totals with the summary, and performs an unconditional final rehash before producing completed output. It retains counters and bounded state rather than a decompressed copy or a complete key set.

For the registered production audit, the plan fixes the sole source bundle and requires pre-launch checks of its registered hashes and expected dimensions. The analyzer's compact `source` record binds the resolved paths, manifest/small-file/process hashes, estimator contract, and settings. This review does not replace Task 6's independent verification of the registered source's 280,000 rows, 279,440 primary rows, 20 seeds, two graph cases, 499 primary frames, and 14 UAVs.

### Current-attempt taxonomy and reconciliation

Classification depends on `attempt_status`, never retained `status`. The six classes are mutually exclusive: contained, converged outside epsilon radius, upstream unavailable, other invalid/numeric, WNLS nonconvergence, and other failure. The analyzer reconciles each six-class budget with its denominator at overall, seed, depth, time, dynamic depth/time, and initialization scopes. It also reconciles conditional calibration families, including unstratified applicability buckets and finite metric sums.

### Bounded lineage

For current upstream-unavailable attempts, lineage examines only unavailable UAV measurement references within the current `(seed, graph_case, frame_index)` predecessor state. It reports observer-row and unavailable-edge counts, predecessor attempt classes, and bounded propagation depths; an absent predecessor is recorded as `not_observed` rather than inferred. No graph replay, counterfactual, or causal conclusion is introduced.

### Conditional metrics and fixed bins

The hand-derived two-dimensional normalized-squared-error calculation is tested, including finite, symmetric, positive-definite covariance validation. Ratio, normalized-squared-error, FIM-condition, and FIM-minimum-eigenvalue bins now use one structured definition per family. The executable classifier, labels, empty tables, and persisted protocol derive from those definitions, while tests cover the frozen boundary placements and calibration-family reconciliation.

### Paired bootstrap and protocol

`D_upstream` is calculated from aggregate dynamic-minus-fixed upstream and invalid counts. The bootstrap resamples paired seed records with replacement, recomputes the aggregate ratio, records non-positive denominators as non-estimable, uses the fixed 10,000-resample/seed-`20260729` configuration, and withholds percentile bounds below the 95% estimability threshold. The persisted protocol records the actual verified `run_seeds` count and matching draws per resample, so a unit fixture is reported truthfully while the registered production source will record 20.

The protocol also records structured predicates, numeric bin boundaries and interval closure, denominator universes, operational disk limits and cadence, and bounded reason/example caps. The six frozen limitations are persisted verbatim and rendered in Markdown: post-hoc scope; one preserved trajectory with 20 paired noise seeds; offline estimator outside the controller; no shared-ancestor cross-covariance model; no causal estimator/graph comparison; and no radius, mission-probability, robustness, or safety validation.

### Immutability, output isolation, and disk controls

Persistent output is rejected when it overlaps the source or uses an output/incomplete-sibling symlink. It is staged only in a sibling `.incomplete` directory, checked before and after writes, then atomically renamed. Source mutation after streaming is rejected even when preflight hash checks are disabled. The implementation enforces the 8 GB launch gate, 6 GB live floor at the 10,000-row cadence and before writes, 2 GB output-parent allocation limit, and 10 MB incomplete-directory limit; controlled failures do not register completed output.

The earlier operational gap was that the registered output parent did not exist while the analyzer intentionally requires an existing parent. The latest plan and design close this without changing analyzer behavior: after Task 6 launch probes pass, create only the registered empty parent exactly once using `mkdir -m 700 /private/tmp/cbf2026-localization-failure-analysis`; if it exists already or creation fails, stop and do not substitute a path. This is setup for the one registered run, not a production analyzer invocation.

### Scope control

Static inspection found no call from the analyzer to an estimator, replay, controller, Monte Carlo routine, or production execution helper. The analyzer reads recorded artifacts, computes bounded descriptive summaries, and writes only the explicitly requested external derived output. The frozen limitations and protocol prevent this review from supporting causal, estimator-comparison, radius, mission-probability, robustness, or safety claims.

## Pre-registration decision

The executable checkpoint is suitable for the single registered production run once the separate registration document is committed and Task 6's read-only launch checks pass. Any source-hash mismatch, launch-gate failure, pre-existing registered output path, or inability to create the exact `0700` output parent is a stop condition, not a reason to alter the source, retry at another path, or revise the frozen taxonomy or bins.
