# CBF2026 Fixed-Geometry Pairwise Mechanism-Gate Implementation Plan

**Goal:** Add one exactly matched `RGP` case and run a disk-guarded 20 s
pairwise-row mechanism gate.

**Design:** Follow
`docs/superpowers/specs/2026-07-28-cbf2026-fixed-geometry-pairwise-gate-design.md`.

### Task 1: Add `RGP` test-first

**Files:**

- Create: `config/diagnostics/rgp_fixed_geometry_pairwise.json`
- Modify: `scripts/diagnostics/run_diagnostic.py`
- Modify: `tests/test_run_diagnostic.py`

1. Add a failing normalized-materialization test proving that `RG` and `RGP`
   differ only at `/cbfs/without-slack/safety/mode`.
2. Add a failing runner-level mapping test and add `RGP` to the CLI
   acceptance test.
3. Record RED.
4. Add the smallest overlay, runner mapping, and CLI choice.
5. Run focused and full Python diagnostic suites and record GREEN.
6. Review and commit only these three files.

### Task 2: Build and verify

1. Reconfigure and build `build-diagnostic`.
2. Run CTest, the full focused C++ suites, and the full Python diagnostic
   suite.
3. Require no tracked changes beyond the intended commit and record the
   binary SHA-256.
4. Independently review the exact normalized config diff and row-count gate.

### Task 3: Run the 20 s gate

1. Confirm the RGP output root is absent.
2. Confirm at least 8,000,000,000 free bytes and less than 2,000,000,000
   retained-result bytes.
3. Run:

```bash
conda run -n cbf_env python -m scripts.diagnostics.run_diagnostic \
  --case RGP --horizon 20 --seed 20260727 \
  --binary build-diagnostic/Swarm \
  --output-root \
  /private/tmp/cbf2026-results/mc-first-fixed-geometry-pairwise-smoke
```

4. Apply every technical and practical requirement in the design, including
   40 frames, 560 applied optimal solves, 7,280 directed pairwise rows, fixed
   information set 560/0/0/0, no hard residual below \(-10^{-7}\), and no
   localization/collision observation below \(-0.5\) m.
5. Preserve the bundle and stop after this one 20 s run, regardless of
   outcome.  Do not launch 250 s.
6. Record exact paths, hashes, disk probes, solver/runtime, uncertainty,
   margins, row coverage, control demand, coverage, and the robot-12--14
   frame-20 counterfactual.

### Task 4: Audit and durable records

1. Dispatch independent raw-integrity and mechanism-interpretation reviews.
2. Update the compact fixed-geometry report with the RGP result without
   erasing the failed RG record.
3. Correct the manuscript rotation formula from
   \(\theta_k=(-1)^{k+1}\pi/3\) to \(\theta_k=(-1)^k\pi/3\), subject to a
   targeted paper diff/build review.
4. Update only the five intended CBF2026 DRA records, leaving unrelated
   PodSearch changes unstaged.
5. Commit reviewed CBF, paper, and isolated DRA changes separately.
6. Run a broad final review and final verification.
