# CBF2026 Fixed-Reference Geometry-Gate Implementation Plan

**Goal:** Add a provenance-isolated non-collinear `RG` diagnostic case and
run its disk-guarded 20 s plus conditional 250 s fixed-reference gate.

**Design:** Follow
`docs/superpowers/specs/2026-07-28-cbf2026-fixed-geometry-gate-design.md`.

### Task 1: Add the `RG` configuration test-first

**Files:**

- Create: `config/diagnostics/rg_fixed_geometry.json`
- Modify: `scripts/diagnostics/run_diagnostic.py`
- Modify: `tests/test_run_diagnostic.py`

1. Add failing tests that `RG` is accepted and materializes:
   - the exact fourteen coordinates from the design;
   - the same R uncertainty-rate, CBF, input-limit, solver, seed, and
     execution semantics;
   - reflection symmetry between squads;
   - nonzero signed area for every consecutive fixed-reference triple.
2. Run the focused tests and record RED.
3. Add the smallest runner mapping/CLI choice and the new overlay.
4. Run focused and full Python diagnostic tests and record GREEN.
5. Review and commit only these three files.

### Task 2: Build and verify

1. Reconfigure/build `build-diagnostic`.
2. Run CTest, all focused C++ suites, and the full Python diagnostic suite.
3. Require no tracked changes, only `build-diagnostic/`, and record the new
   binary SHA-256.

### Task 3: Run the disk-guarded gate

1. Confirm both new output roots are absent:
   - `/private/tmp/cbf2026-results/mc-first-fixed-geometry-smoke`
   - `/private/tmp/cbf2026-results/mc-first-fixed-geometry-250s`
2. Confirm at least 8,000,000,000 free bytes and less than 2,000,000,000
   retained-result bytes.
3. Run:

```bash
conda run -n cbf_env python -m scripts.diagnostics.run_diagnostic \
  --case RG --horizon 20 --seed 20260727 \
  --binary build-diagnostic/Swarm \
  --output-root /private/tmp/cbf2026-results/mc-first-fixed-geometry-smoke
```

4. Require:
   - completed terminal manifest;
   - 40 frames and 560 applied optimal solves;
   - zero unapplied, finite-value, and negative hard-residual records;
   - `covariance_information_set` available with 560 records and zero
     transitions, mismatches, or malformed records;
   - zero localization/collision observations below `-0.5 m`;
   - no consecutive negative localization/collision frames.
5. If any smoke requirement fails, stop and preserve the bundle.
6. If it passes, recheck disk and run the same command with `--horizon 250`
   and the 250 s output root.
7. Require 500 frames, 7000 applied optimal solves, 21000 hard rows, the same
   zero-integrity counts, and the same practical-tolerance gate.
8. Record exact paths, hashes, source/binary anchors, disk probes, uncertainty,
   margins, control bounds, coverage, and runtime.

### Task 4: Independent audit and durable records

1. Dispatch separate raw-integrity and mechanism-interpretation audits.
2. Create a compact CBF report.
3. Record the result in the five CBF2026 DRA files only, leaving PodSearch
   changes unstaged.
4. Commit the CBF report and isolated DRA update after review.
5. Run a broad final review.
