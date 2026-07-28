# CBF2026 Fixed Localization Information-Set Design

## Decision

The covariance propagation used by the CBF2026 diagnostic branch shall use
the predeclared localization graph as its complete information set.
For each UAV, the FIM shall contain exactly the fixed base references and
fixed lower-index UAV references used by the localization formation.
Range-visible bases or lower-index UAVs that are not in that graph shall not
be added, and fixed references shall not be removed when their distance
crosses the communication range.

This changes anchor selection only.
The ranging variance, propagated anchor covariance, Jacobian, and
\(\Phi_i=J_i^\top\Sigma_i^{-1}J_i\),
\(\hat\Sigma_i=\Phi_i^{-1}\) calculations remain unchanged.

## Context

The current `Robot::getCovariance()` admits:

1. every base currently within `comm-fixed.max-range`; and
2. every same-squad lower-index UAV that is either a fixed neighbour or
   currently within that range.

The union of all possible robot edges is acyclic because robot anchors always
have a smaller local index.
However, the active FIM information set is not fixed.
The 250 s R run recorded 68 robot-frame set transitions and positive
uncertainty jumps when optional anchors were lost.
That behaviour conflicts with the manuscript-level contract that the
localization graph is invariant.

## Considered Approaches

### A. Make the predeclared localization graph the only FIM information set

This is the selected approach.
It matches the intended theory, removes range-triggered information-set
switching, keeps the FIM formula intact, and requires the smallest experiment
needed to test the hypothesis.

### B. Add a configuration switch for fixed and range-gated information sets

This would preserve the legacy dynamic mode for other experiments, but it
adds an unneeded branch and creates a risk that the paper, theory, and
materialized experiment configuration silently use different contracts.
The diagnostic branch already exists to isolate the CBF2026 revision, so this
option is not selected.

### C. Retain dynamic optional anchors and model active-set transitions

This keeps current trajectories but requires a hybrid or post-transition
safety condition and explicit jump evidence.
It conflicts with the research decision not to prioritize jump reserves and
is not selected.

## Fixed Reference Definition

The implementation shall expose one source of truth for fixed localization
references and reuse it in both covariance construction and formation CBF
construction.

For UAV \(i\):

- fixed robot references are the IDs in `myNeighboursId` whose squad-local
  index is strictly smaller than \(i\)'s;
- fixed base references are exactly the base IDs currently selected by
  `setupFormation()` from `formation.bases-id` and
  `comm-fixed.min-neighbour-id-offset`;
- an unassigned base or UAV is excluded even when it is within
  `comm-fixed.max-range`;
- an assigned reference remains in the information set even when it is
  outside `comm-fixed.max-range`;
- a missing position or covariance for a fixed UAV reference is an
  information-contract failure, not permission to change the active set.

The current CBF2026 configuration has
`min-neighbour-id-offset=-2` and `max-neighbour-id-offset=0`.
Together with the strict lower-index covariance rule, this gives a
topological order rooted at the assigned bases.

## Code Shape

`include/Robot.hpp` shall gain a small pure query helper that builds the fixed
base and UAV reference IDs from `myBasesId`, `myNeighboursId`, the local
index, and the existing formation rule.
`setupFormation()` and `getCovariance()` shall consume that helper instead of
independently recreating the selection logic.
Covariance selection shall not depend on the mutable `myFormation`, which may
also be used by other formation setup paths.
Because covariance construction no longer consumes `max-range`,
`getCovariance(json& config)` shall become `getCovariance()`.

`getCovariance()` shall:

1. initialize `myCovarianceFormation` from the fixed ID lists;
2. fetch every fixed base position and use zero base covariance;
3. fetch every fixed UAV position and propagated covariance;
4. run the existing FIM calculation without distance-based filtering; and
5. fail explicitly if fixed reference data are missing or fewer than two
   usable anchors remain.

No lower-level dynamics, sampled-data reserve, uncertainty-jump reserve,
class-\(\mathcal K\) change, input-limit change, or held-command timing change
belongs in this patch.

## Test-First Contract

The first regression test shall construct a UAV whose:

- fixed references include anchors outside `max-range`; and
- communication map also contains an unassigned lower-index UAV and
  unassigned bases inside `max-range`.

Against the current implementation, the test must fail because the logged
`myCovarianceFormation` contains optional range-visible references.
The focused red-green set shall show that:

- `anchorIds` and `baseIds` equal the predeclared localization graph;
- an in-range lower-index UAV outside that graph is excluded;
- assigned non-collinear base references outside `max-range` are retained;
- moving assigned and unassigned references across the range threshold does
  not change those IDs; and
- every UAV reference has a strictly smaller squad-local index;
- the covariance remains finite and is recomputed from the same fixed set;
  and
- a two-anchor hand calculation still matches the unchanged FIM formula.

The existing three-UAV bootstrap fixture is geometrically collinear.
Its initial positions shall be changed to a non-collinear chain before it is
used to validate a two-reference FIM; the bootstrap assertion itself shall
remain unchanged.

Existing covariance bootstrap, uncertainty-rate, robust-row, failure-path,
runner, and analyzer tests must remain green.

## Experiment Gate

After focused tests and a clean build:

1. run a 20 s R smoke using the same source lineage, seed `20260727`, Gurobi,
   0.5 s step, class-\(\mathcal K\), and unbounded-input R configuration;
2. require zero information-set transitions, no non-finite data, all solver
   results applied and optimal, and no hard residual below `-1e-7`;
3. if the smoke passes, run the same 250 s R gate;
4. compare against the preserved dynamic-R bundle, without deleting or
   overwriting it; and
5. stop if the fixed-R tightened localization gate fails.

The fixed-R run is a mechanism test, not Monte Carlo evidence.
If it still fails, the next mechanism to address is the mismatch between the
neighbour command assumed by the CBF and the command actually held over the
interval.

## Disk and Evidence Rules

- require at least 8 GB free before a run;
- stop the run below the 6 GB free-space floor;
- keep total experiment cache below 2 GB;
- keep each run below the existing 250 MB abnormal-growth limit;
- use a new output root and preserve the earlier C0/R bundles;
- record source commit, binary hash, materialized configuration, manifest,
  raw-data hash, analysis hash, disk probes, and the stop decision;
- do not push, delete data, or modify the paper until the mechanism evidence
  has been reviewed.

## Acceptance Boundary

This work is accepted as an implementation result only when the regression
test has demonstrated a red-green cycle and the complete focused test set
passes.
It is accepted as an experimental mechanism result only when the 20 s smoke
and 250 s R evidence satisfy the gate above.
Neither result alone closes bounded-input joint feasibility, estimator-bound
calibration, cross-correlation, multi-geometry Monte Carlo, or manuscript
revision.
