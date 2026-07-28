# CBF2026 Aborted-Frame Evidence Implementation Plan

**Goal:** Correct partial-frame classification in the diagnostic analyzer and
regenerate the preserved RGP summary without rerunning the trajectory.

**Design:** Follow
`docs/superpowers/specs/2026-07-28-cbf2026-aborted-frame-evidence-design.md`.

### Task 1: Fix the analyzer test-first

**Files:**

- Modify: `scripts/diagnostics/analyze_diagnostic.py`
- Modify: `tests/test_analyze_diagnostic.py`

1. Add a failing sequential-distributed fixture containing:
   - one fully successful frame;
   - one frame with a fresh optimal record, a failed record, and a later stale
     optimal record copied from the previous frame.
2. Require the full prior frame to be applied, the aborted frame to be wholly
   unapplied, the failed record and preceding fresh attempts to be counted as
   attempts, and later records to be counted as not attempted.
3. Require applied hard/pairwise/control metrics to exclude the aborted frame
   while state/covariance/margin metrics remain available.
4. Update the existing failed-placeholder test to the same frame-level
   semantics.
5. Record RED.
6. Implement the smallest distributed sequential classification and explicit
   summary fields.
7. Run focused analyzer tests, the four-module Python suite, and
   `git diff --check`.
8. Independently review and commit only the two files.

### Task 2: Reanalyze preserved RGP evidence

1. Verify the raw, manifest, config, source-snapshot, stdout, and stderr hashes
   before analysis.
2. Run the corrected analyzer on the existing raw and manifest only.
3. Verify the exact RGP acceptance truth from the design, the local
   infeasibility certificate, and unchanged raw/manifest hashes.
4. Record new summary hashes and disk probes.
5. Do not rerun the simulator or overwrite any primary evidence.

### Task 3: Durable scientific records

1. Extend the fixed-geometry report with the RGP stop result, corrected
   evidence counts, local Farkas certificate, and full stacked feasible
   witness.
2. Record that direct all-pair rows in independent local QPs are not equivalent
   to the coupled stacked CBF condition.
3. Correct the manuscript triangular-ladder sign and narrow any unsupported
   all-pair decentralized-feasibility claim before new experiments.
4. Update only the intended CBF2026 DRA files and leave unrelated PodSearch
   changes unstaged.
5. Run independent evidence and manuscript reviews before separate commits.
