# Evidence Review — Localization-Failure Mechanism Audit

Date: 2026-07-29  
Scope: post-hoc evidence report and diagnostics index only; no analyzer rerun, source-bundle modification, or paper-claim update.

## Verdict

**CLEAN.**

The report is numerically and procedurally consistent with the frozen derived-output bundle and with an independent, streaming audit of the immutable raw source.  No finding meets the Critical, Important, or Minor thresholds below.

| Severity | Findings |
|---|---|
| Critical | None |
| Important | None |
| Minor | None |

## Review method and immutable evidence

This review read the report, the diagnostics index, the registration, and the derived JSON/Markdown outputs.
It independently streamed the gzip JSONL source one raw line at a time to recompute its integrity cursor, status and mechanism counters, lineage counters, and hashes.
It did not invoke the analyzer, materialize decompressed records, modify either evidence bundle, or repeat the registered run.

The source and output identifiers agree across the registration, report, and outputs:

| Item | SHA-256 |
|---|---|
| Source gzip JSONL | `38325523e395983d83491c81556bac1aae72ebd99508b3cdf148fdf72f3f8803` |
| Source decompressed stream | `2c8a3f0f5684a281d5fbef3d7facea16f45d757d96b458827928dcac9e241a4f` |
| Source summary JSON | `f4f780dd4c9c1399d3f838c74054c9c0ba3159415bc13089aee53d0ebf7250bc` |
| Source summary Markdown | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |
| Source manifest | `39ee3b5494ed623d20d16766244427476cd081ed0869feecb9e5072c6c8f481d` |
| Derived JSON | `0e86f884a262fa7259b4a3c13ea004c9cb711176f4240ae8d7d100bcb87a375e` |
| Derived Markdown | `0fcb6c84875c88c9e18169accf663fdc4321b4995dc434032edbf67ba973f27f` |

The raw audit found 280,000 records, including 279,440 primary records, with no canonical-key mismatch or missing expected key.
The derived JSON declares `completed`, records the same integrity totals and source hashes, and parses strictly.

## Numerical reconciliation

Both reported primary denominators are 139,720.
The six-class budgets match the raw audit and reconcile to those denominators:

| Class | Dynamic graph | Fixed graph |
|---|---:|---:|
| Converged and contained | 62,516 | 41,913 |
| Converged outside radius | 4,526 | 40,238 |
| Upstream unavailable | 50,898 | 42,914 |
| Invalid input / numeric | 15,469 | 14,471 |
| WNLS nonconvergence | 6,311 | 184 |
| Other failed | 0 | 0 |

The report's rounded percentages recompute from those integers.
Its status totals, lineage rows and edges, root-predecessor counts, and depth distributions match both the raw audit and derived JSON.
In particular, the reported `D_upstream = 7,984 / 8,982 = 8/9` is exact.

The report correctly withholds the bootstrap interval: 7,509 of 10,000 resamples are estimable, below the pre-registered 9,500 threshold, so the reported percentile interval is `null`.
The conditional-containment, ratio-bin, Mahalanobis-exceedance, initialization, persistence, squad-depth, and time-bin values likewise match the frozen JSON and the independently checked raw counters.

## Procedure, storage, and claims boundary

The exact command, executable commit (`7e44ab4ad445d925c8a39062e9701cb093b2cd99`), executable SHA-256, environment, launch conditions, and output path agree with the run registration.
The report records one analyzer execution and no retry; this review did not create or observe another execution.
It also records only the two permitted output files, whose total allocation is 408 KiB (417,792 bytes), below both the registered 10,000,000-byte run cap and the registered 2,000,000,000-byte analysis-root cap.

The post-review disk-maintenance note is internally consistent and remains outside the evidence computation.
It records a rebuildable cache at `/Users/xirhxq/.cache/codex-runtimes` of 1,583,072 KiB, free space of 6,940,770,304 bytes before its previously authorized cache-only cleanup, and 8,573,808,640 bytes after the read-only probe.
The reported free-space increase is 1,633,038,336 bytes; the cache allocation is 1,621,065,728 bytes, leaving 11,972,608 bytes unassigned rather than attributing the full filesystem change to the cleanup.
The cache directory is now absent, and the derived JSON and Markdown hashes still match the frozen values above.
The report correctly states that no evidence, user document, or Git history was removed.

The evidence report and `docs/diagnostics/README.md` consistently describe a post-hoc, offline diagnostic over one trajectory with 20 paired seeds.
They do not assert causal effects, radius validation, mission-level probability, robustness, or safety conclusions.
They preserve the stated limits: no estimator replay/controller comparison, no shared-ancestor cross-covariance treatment, and no claim upgrade to the paper.

## Scope note

This is a consistency and evidence-integrity review, not a causal validation.
The observed root-invalid/numeric and multi-hop-unavailability patterns identify a bounded follow-up target; any causal investigation or claim beyond the recorded diagnostic requires separate design, registration, execution, and review.
