# Independent review of corrected variable-weight WNLS calibration evidence

## Verdict

**Evidence/report integrity: PASS.**

**Scientific adequacy: FAIL, as reported.**
These verdicts are deliberately separate.
The preserved artifacts, the corrected evidence report, and the diagnostics
README are internally consistent and reproducible from the raw rows.
The registered experiment itself fails four of its five adequacy conditions,
so this review does not validate the \(3\epsilon\) radius or authorize a
stronger paper claim.

Issues by severity:

- Critical: 0.
- Important: 0.
- Minor: 0.

The reviewed corrected report is
`docs/diagnostics/2026-07-29-corrected-wnls-localization-calibration.md`,
20,223 bytes, SHA-256
`92e71a8642b67a88e8aceddd8b70052afbce3549c66d9f3b660b5b68c5b0d3e2`.
The entry in `docs/diagnostics/README.md` links to that report and accurately
describes the result as an objective-correct replay whose registered adequacy
gate failed, rather than as a \(3\epsilon\) validation.

## Reviewed bundles

The review streamed and strictly parsed every process row from these exact
preserved directories:

1. Stage 1 A:
   `/private/tmp/cbf2026-localization-calibration-corrected/stage1/localization-calibration/20260728T184323.997907Z_74cdb2fed5294fc6b52fa5f688f71f12`
2. Stage 1 B:
   `/private/tmp/cbf2026-localization-calibration-corrected/stage1/localization-calibration/20260728T184607.307681Z_f49e35405e914d938fc5e957af5f318d`
3. Stage 2:
   `/private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288`

The independently recomputed artifacts are:

| Run | Artifact | Bytes | SHA-256 |
| --- | --- | ---: | --- |
| Stage 1 A | `calibration.jsonl.gz` | 263,801 | `e3fd21c9431dd4e2dfc5373c7bf1e767283801bfd8f8393d27de2d18757cb127` |
| Stage 1 A | decompressed JSONL | 1,916,361 | `c1378d7ae1eda110004443ca2916ad8078d23c91d93576d99dce2e518beccb6a` |
| Stage 1 A | `summary.json` | 30,458 | `5d6b3afb826732c24db20c646f065f5cb71693b72ff5c7c8e9a1a5792a57864b` |
| Stage 1 A | `summary.md` | 262 | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |
| Stage 1 A | `manifest.json` | 2,610 | `9f244f56a5a83d2f9612d5035b455a26760fab20784dabc987dc6296096a06df` |
| Stage 1 B | `calibration.jsonl.gz` | 263,801 | `e3fd21c9431dd4e2dfc5373c7bf1e767283801bfd8f8393d27de2d18757cb127` |
| Stage 1 B | decompressed JSONL | 1,916,361 | `c1378d7ae1eda110004443ca2916ad8078d23c91d93576d99dce2e518beccb6a` |
| Stage 1 B | `summary.json` | 30,458 | `5d6b3afb826732c24db20c646f065f5cb71693b72ff5c7c8e9a1a5792a57864b` |
| Stage 1 B | `summary.md` | 262 | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |
| Stage 1 B | `manifest.json` | 2,610 | `4742c0a7743f9340b1f742ac3aa1ab5e833db97109080526d790c6953aa3223b` |
| Stage 2 | `calibration.jsonl.gz` | 47,608,272 | `38325523e395983d83491c81556bac1aae72ebd99508b3cdf148fdf72f3f8803` |
| Stage 2 | decompressed JSONL | 414,488,008 | `2c8a3f0f5684a281d5fbef3d7facea16f45d757d96b458827928dcac9e241a4f` |
| Stage 2 | `summary.json` | 91,573 | `f4f780dd4c9c1399d3f838c74054c9c0ba3159415bc13089aee53d0ebf7250bc` |
| Stage 2 | `summary.md` | 262 | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |
| Stage 2 | `manifest.json` | 2,920 | `39ee3b5494ed623d20d16766244427476cd081ed0869feecb9e5072c6c8f481d` |

All four hashes stored in each terminal manifest match the corresponding
compressed process, decompressed process, JSON summary, and Markdown summary.
Both Stage 1 runs have byte-identical compressed and decompressed process
streams and byte-identical summaries.
Their manifest differences are limited to expected run-specific metadata.

## Raw-row and graph audit

Strict standard-library JSON parsing, with nonstandard constants rejected and
all parsed floating-point values checked for finiteness, accepted all 1,120
rows in each Stage 1 bundle and all 280,000 Stage 2 rows.
Every row had the registered field set.
The complete expected key products were reconstructed independently:

- each Stage 1 run contains exactly 40 frames, one seed, two graph cases, and
  14 UAVs, with 1,120 unique keys;
- Stage 2 contains exactly 500 frames, 20 seeds, two graph cases, and 14 UAVs,
  with 280,000 unique keys;
- all three bundles have zero duplicate, missing, extra, or out-of-domain
  keys.

The input truth trajectory was read independently at 16,237,150 bytes,
SHA-256
`3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527`.
Its input manifest was read at 1,226 bytes, SHA-256
`6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb`,
and records source commit
`cff3852831f969ad4bedf7d9b2f43b4f2a0cf42f` and configuration SHA-256
`cb13a87615da21e2f52ab41d039f41c4547e8d7311f1f2b791afe8474d10153d`.

For every row, the review reconstructed the fixed reference table from the
registration and independently formed the dynamic set by adding every
in-range base and every in-range, strictly lower squad-local UAV.
There were zero fixed-membership errors, zero dynamic-membership errors, and
zero measurement-list/reference-set errors.
The recorded truth positions, true ranges, and
`noisy_range = true_range + noise` identities had zero mismatches.
All fixed-case measurements were matched against the same edge in the
dynamic case: both Stage 1 bundles had 1,120 shared measurements and Stage 2
had 280,000, with zero noise and zero noisy-range mismatches.
The experiment is therefore a paired graph ablation, not an estimator
comparison.

## Solver state, FIM, and radius audit

Every nonconverged attempt retained a nested failure object with the same
attempt status, reason, and stationarity value.
Every stale state retained that current failed or invalid attempt.
Attempt and retained-state counts reconcile exactly with the corrected
report, including all 43 Stage 1 stale states and all Stage 2 primary and
frame-zero counts.
No attempted failure is absent from its process row.

All converged attempts have finite stationarity norm at most `1e-9`.
The independent maxima are
`9.96011852283137e-10` in both Stage 1 bundles and
`9.999536187738804e-10` in Stage 2.

The review reconstructed the localization FIM directly for every converged
row from its final estimate, active reference estimates and covariances,
recorded geometry, and `ranging_sigma=0.5`.
This covered 517 converged rows in each Stage 1 bundle and 149,505 in Stage 2.
There were zero FIM, covariance, eigenvalue, condition-number, or epsilon
identity failures.
For Stage 2, the worst relative discrepancies were
`4.08e-12` for covariance,
`2.04e-12` for \(\epsilon\),
`1.61e-11` for the minimum FIM eigenvalue, and
`1.61e-11` for the FIM condition number.
All retained covariance matrices were symmetric to the registered relative
tolerance and positive definite, and every finite state satisfied
\(\epsilon=3\sqrt{\lambda_{\max}(P)}\).
Error vectors, norms, error-to-epsilon ratios, state-containment flags, and
attempt-containment flags had zero identity mismatches.

Transition metadata also reproduced exactly.
Stage 2 has 1,360 dynamic active-set transitions, exactly 68 per seed, and no
fixed-reference transitions.
There are 600 finite dynamic epsilon changes; their p5, median, p95, mean,
minimum, and maximum match the report.
The paired containment table
`30,701 / 31,815 / 11,212 / 65,992` and the 48,902 finite paired epsilon
differences, including every reported quantile and extreme, also match.

## Statistics and adequacy

Independent primary-row recomputation gives:

| Case | Containment | Invalid attempts | Failed attempts |
| --- | ---: | ---: | ---: |
| Dynamic DAG | 62,516 / 139,720 = 44.7438% | 66,367 / 139,720 = 47.5000% | 6,311 / 139,720 = 4.5169% |
| Fixed refs | 41,913 / 139,720 = 29.9979% | 57,385 / 139,720 = 41.0714% | 184 / 139,720 = 0.1317% |

Frame-zero containment independently reproduces 144/280 for the dynamic case
and 163/280 for the fixed case.
Every dynamic depth denominator is 19,960, with containment counts
`16,082, 12,877, 10,952, 8,844, 6,828, 4,010, 2,923`.
Thus the minimum depth rate is 14.6443%.
All 20 reported per-seed rates reproduce from the raw primary rows.

Using exactly 10,000 resamples of the 20 seed-level rates with
`numpy.random.default_rng(20260728)`, the independent 95% intervals are
`[0.34259125393644424, 0.5457854995705697]` for the dynamic graph and
`[0.2740978385342112, 0.3258091182364729]` for fixed references.
These are the report's rounded intervals.
All reported primary epsilon, error-ratio, FIM-eigenvalue, FIM-condition,
transition-change, and paired-difference quantiles and extrema reproduce.

The five preregistered booleans independently evaluate as:

| Condition | Result |
| --- | --- |
| Dynamic aggregate containment at least 98% | Fail |
| Every dynamic squad-local depth at least 95% | Fail |
| Complete rows with no silent drops | Pass |
| Dynamic invalid-attempt rate no worse than fixed | Fail |
| Dynamic failed-attempt rate no worse than fixed | Fail |

The overall scientific adequacy verdict is therefore **FAIL**.
The preserved summaries contain the same booleans.

## Provenance, preservation, and disk limits

The frozen executable commit
`f6e995d6dc4711f0d2869636beb70f061e73063a` is an ancestor of execution
checkout `2b4b5ac266d05f3f3ebb28b8129d8ec9876fd588`.
The only committed paths differing between them are the registered run plan
and the tracked mathematical review; both executable diagnostic paths have
zero diff.
The runner SHA-256 is
`9af743f9445813e5b2f89f1989963319929d3f16b52b244563e2dca4fa52a90b`.
The tracked mathematical review is present at
`docs/diagnostics/reviews/2026-07-29-variable-weight-wnls-math-review.md`,
SHA-256
`6c1dc9497175a602d58ef1ba53d8a8eca572b12567c063d52ae74d24faa3f09a`,
and records `CLEAN / READY`.
The checked environment reports Python 3.11.12 and NumPy 1.24.4, matching
the frozen registration and all manifest settings.

The manifest free-space readings before every invocation exceed
8,000,000,000 bytes and their terminal readings exceed 6,000,000,000 bytes.
The independent current allocated sizes are 307,200 bytes for each Stage 1
bundle, 47,714,304 bytes for Stage 2, and 48,328,704 bytes for the complete
corrected root.
Every bundle is below 250,000,000 bytes and the root is below
2,000,000,000 bytes.

The excluded old Stage 2 remains at
`/private/tmp/cbf2026-localization-calibration/stage2/localization-calibration/20260728T164130.657823Z_6832cfbd6592469085adb88b5a165f59`.
Its independently recomputed hashes are:

| Artifact | SHA-256 |
| --- | --- |
| `calibration.jsonl.gz` | `34a6045d4ac5d23f72ac6d049d4d98a8f927ac9835b71f3a3050a00f8efaf524` |
| decompressed JSONL | `6e1d6ea42e3c65117d62a734f8771491e1ec4181ef5778503bb9ca65a2bd54c7` |
| `summary.json` | `0dddcc119e00abbcdb7dfad6491f39bde6bbf4107fee81a09a556ecf53a1c2b6` |
| `summary.md` | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |
| `manifest.json` | `74f37cb4c4ea7e8e49eef660eade7d047fa137dfc27f1a36b4c675c4b364fa89` |

Its runner hash is the previous
`218ba696c83a437fcf51f812f1b21156bc0baeab96dfde4b15a9bd31b1d9e5a8`,
not the corrected runner hash.
It is still present at its original path, and none of the corrected
manifests or summaries use it as corrected evidence.
Its preservation and exclusion from objective-correct adequacy are therefore
accurately reported.

The report also preserves the required interpretation boundary: all seeds
reuse one trajectory geometry, the estimator is an offline sidecar outside
the controller and CBF loop, the FIM omits shared-ancestor
cross-correlations, and the failed result supplies neither a mission-level
probability guarantee nor a closed-loop safety guarantee.
