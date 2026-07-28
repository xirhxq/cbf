# CBF2026 Fixed-Reference Geometry-Gate Design

## Decision

Add a new diagnostic case, `RG`, for the fixed-reference geometry gate.
Do not modify the existing `R` overlay or any preserved R evidence.

`RG` keeps the R controller, seed, solver, time step, class-\(\mathcal K\),
unbounded-input setting, fixed information set, and FIM formula unchanged.
It changes only six initial positions: the three alternating vertices in each
seven-UAV squad.

## Geometry

The legacy specified start places each squad on a straight line.
For every local UAV after the second, the two fixed predecessors then give
collinear range directions, so the fixed-reference FIM is rank deficient.

The `RG` start preserves all odd local-index positions, including both
leaders, and replaces only the even local-index positions:

```text
Squad 1:
(-1490.000000, -120.000000)
(-1487.320508, -164.641016)
(-1450.000000, -140.000000)
(-1447.320508, -184.641016)
(-1410.000000, -160.000000)
(-1407.320508, -204.641016)
(-1370.000000, -180.000000)

Squad 2:
(-1490.000000,  120.000000)
(-1487.320508,  164.641016)
(-1450.000000,  140.000000)
(-1447.320508,  184.641016)
(-1410.000000,  160.000000)
(-1407.320508,  204.641016)
(-1370.000000,  180.000000)
```

The even vertices use the implementation-aligned alternating
\(-\pi/3\)/\(+\pi/3\) rotation of the legacy odd-node section vector.
This is a controlled triangular-ladder admissibility repair, not a claim that
the initial coordinates are the literal midpoint-to-leader targets in the
manuscript.

Static reconstruction gives:

- worst fixed-reference FIM condition number approximately `45.1`;
- maximum initial scalar uncertainty approximately `15.11 m`;
- minimum pairwise distance approximately `44.72 m`;
- minimum tightened collision margin approximately `6.07 m`; and
- minimum tightened fixed-link margin approximately `657.49 m`.

These values are design prechecks, not simulation outcomes.

## Provenance and evidence boundary

Use a dedicated `config/diagnostics/rg_fixed_geometry.json` overlay and extend
the diagnostic runner with case `RG`.
The legacy `R` overlay and its old materialized configurations remain
unchanged.

An `RG` run answers only:

> Can the fixed-reference controller execute from a predeclared,
> non-collinear, fixed-graph-admissible initial geometry?

It does not isolate the causal effect of fixed versus dynamic information-set
membership because the preserved dynamic-R run used the collinear legacy
start.
Any later causal comparison must rerun both memberships from the same
predeclared admissible geometry.

## Gates

1. Test materialization, mirror symmetry, non-collinearity, and preservation
   of the R controller settings.
2. Require a clean committed source state, at least 8 GB free before launch,
   a 6 GB live floor, a 2 GB results-root cap, and a 250 MB run cap.
3. Run one 20 s `RG` smoke.
4. Require 40 frames, 560 applied optimal solves, no finite/hard failures,
   zero information-set transitions/mismatches/malformed records, and no
   localization or collision observation below `-0.5 m`.
5. Only if the smoke passes, run one 250 s `RG` gate with the corresponding
   500-frame/7000-solve/21000-hard-row integrity requirements.
6. Stop on any failure and preserve every bundle.

No bounded, pairwise, multi-geometry, or manuscript-claim upgrade belongs to
this gate.
