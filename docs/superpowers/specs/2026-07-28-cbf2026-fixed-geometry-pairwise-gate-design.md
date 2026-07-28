# CBF2026 Fixed-Geometry Pairwise Mechanism-Gate Design

## Decision

Add a provenance-distinct diagnostic case, `RGP`, that differs from `RG`
only in the safety-row selection mode:

```text
RG:  cbfs.without-slack.safety.mode = minimum
RGP: cbfs.without-slack.safety.mode = pairwise
```

Do not modify `RG`, its implementation-aligned non-collinear geometry, or its
retained evidence.  Keep the seed, 0.5 s step, horizon, solver, uncertainty
rate, class-\(\mathcal K\) parameters, communication constraints, fixed FIM
information set, input-limit state, and distributed execution unchanged.

## Evidence question

Under the same fixed geometry and distributed/ZOH controller semantics, does
instantiating every directed pairwise collision row remove the isolated
below-\(-0.5\) m tightened collision event observed with minimum-row
selection?

This tests whether minimum-pair omission and switching are necessary
contributors.  It does not remove the use of held neighbour commands or the
backward-difference uncertainty-rate prediction.

## Configuration proof

Create `config/diagnostics/rgp_fixed_geometry_pairwise.json` by explicitly
materializing the same truth fields and fourteen positions as `RG`, with the
top-level case changed to `RGP` and only the safety mode changed to
`pairwise`.

Tests must materialize `RG` and `RGP` with the same 20 s horizon and seed,
remove only runner-generated `output_path` and `run_suffix`, and prove that
the only remaining JSON-pointer difference is
`/cbfs/without-slack/safety/mode`.

In particular, equality must cover:

- all initial positions and `method=specified`;
- time step, horizon, seed, solver, distributed execution, and constraint
  checking;
- uncertainty-rate mode and the complete input-limit object;
- safety and fixed-communication enable flags, uncertainty flags,
  class-\(\mathcal K\) parameters, range, offsets, and held-velocity
  compensation;
- formation, base, covariance, ranging, world, search, model, and debug
  settings.

## Pre-registered 20 s gate

Use the same verified Gurobi binary, seed `20260727`, 0.5 s step, and a unique
absent output root:

```text
/private/tmp/cbf2026-results/mc-first-fixed-geometry-pairwise-smoke
```

Before launch require at least 8,000,000,000 free bytes and less than
2,000,000,000 bytes in the retained results root.  During the run enforce a
6,000,000,000-byte free-space floor, a 2,000,000,000-byte output-root cap,
and a 250,000,000-byte per-run cap.  Preserve all evidence on any failure.

Technical integrity requires:

- completed terminal manifest and simulator return code 0;
- 40 frames, 560 applied optimal solves, and zero failed/unapplied solves;
- zero non-finite values;
- no hard-row residual below \(-10^{-7}\);
- 560 fixed-information-set records with zero transitions, required-reference
  mismatches, and malformed records;
- covariance/epsilon consistency and frame-zero FIM validity;
- 7,280 expected directed pairwise safety rows, with none missing or
  unexpected.

The practical tolerance remains exactly \(-0.5\) m.  Require zero localization
and collision observations below it and no consecutive negative frames.
Report strict tightened minima, every event in \([-0.5,0)\), and the
robot-12--14 margin at frame 20 even when the practical gate passes.  A
practical pass does not establish strict forward invariance.

Run exactly one 20 s trajectory.  Do not launch 250 s, multiple seeds, a
different step, or any parameter adjustment in this gate.

## Interpretation

- If RGP passes while RG fails, pair switching/omission is empirically
  important for this seed and horizon.  This does not solve stale-neighbour
  prediction, prove strict continuous-time safety, establish bounded-input
  feasibility, or generalize to other trajectories.
- If RGP retains the same event, pairwise coverage is insufficient and the
  held-command and/or rate-prediction mismatch remains.
- If a different event occurs, the trajectory has diverged; do not make an
  exact post-divergence state-by-state causal claim.
- If the QP is infeasible or any provenance, configuration, row-coverage, or
  integrity check fails, no collision-mechanism inference is allowed.

Do not relax the tolerance, change geometry, or add a reserve after observing
the result.
