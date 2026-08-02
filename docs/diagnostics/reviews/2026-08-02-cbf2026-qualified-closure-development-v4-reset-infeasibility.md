# Development-v4 frame-zero reset infeasibility audit

## Decision

The development-v4 controller failure is a genuine fail-closed infeasibility
of UAV 3's bounded distributed local hard QP. It is not a sign error, solver
false negative, reset-order defect, fixed/dynamic-reference mix-up, or yaw
effect. The controller guard behaved correctly. Development-v4 remains
consumed and cannot be retried.

The responsible experiment-design condition is the combination of dense
random initialization, all-pair collision hard edges, fixed half endpoint
allocation, command-independent uncertainty-rate certificates, and frozen
planar component bounds. A second evidence defect caused the rejected reset's
serialized QP witness to be discarded as `schema_error`; the numerical
witness below is therefore independently reconstructed from immutable
initialization rows and the registered production formulas.

## Immutable evidence boundary

Mission-01 emitted exactly 14 initialization rows at frame zero and then
returned `1`. Its exact stderr is:

```text
[SIMULATION_ERROR] theorem certificate reset rejected: post-reset bounded local hard QP is infeasible for UAV 3 (infeasible)
```

The supervisor manifest records 14 valid rows, zero missing input rows, and
terminal `failed/schema_error`. No reset row survived in `swarm.jsonl.gz`.
The materialized primary config uses `random-in-polygon` over the frozen
120-by-400 m deployment polygon, all-pair safety, allocated-pairwise fixed
connectivity, dynamic-lower-index FIM references, and component bounds
`[-25,25]` m/s.

Relevant immutable files are:

```text
/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v4/mission-01/swarm.jsonl.gz
/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v4/mission-01/swarm.stderr.log
/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v4/mission-01/swarm.supervisor.manifest.json
/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v4/mission-01/materialized-primary.json
```

## Reconstructed UAV 3 certificate

The immutable frame-zero position and production certificate formulas give:

```text
p3                         (-1477.84094256, -19.31075110) m
epsilon3                   1.38787244 m
bar_nu3                    5.15438424 m/s
dynamic FIM references     base 0, base 1, base 2, UAV 1, UAV 2
```

All reconstructed post-reset barriers are nonnegative. For example, the
tightened collision barriers for edges 3--5 and 3--8 are approximately
83.65 m and 12.48 m. The guard therefore correctly passes the post-reset
geometric barrier check before reaching local-QP feasibility.

Two UAV 3 collision endpoint rows alone imply:

```text
edge 3--5:  (-0.46902613,  0.88318429)^T u3 >= 0.97181796
edge 3--8:  ( 0.38681465, -0.92215748)^T u3 >= 4.53057653
```

Their normals have inner product approximately `-0.99586`. A positive linear
combination eliminating the y component yields the necessary condition

```text
u3_x <= approximately -53.89 m/s.
```

This contradicts the registered/theorem component bound `u3_x >= -25 m/s`.
Independent half-plane enumeration gives maximum minimum residual about
`-2.24391` at `u3 approximately (-9.3136,-6.3864)`, with edges 3--5, 3--8,
and 3--14 active. The feasible set is empty; Gurobi's `infeasible` status is
therefore mathematically consistent with the registered constraints.

## Ruled-out explanations

- No initial collision barrier is negative; the closest pair is still about
  15.30 m apart for a 10 m safety distance.
- The fixed two distance-maintenance references are loose at UAV 3 and are not
  the binding rows.
- Dynamic references affect only the FIM covariance, radius, and rate; they do
  not create distance-CBF edges.
- Collision and localization row signs agree with the continuous derivation.
- Yaw is absent from the contradictory two-dimensional half planes.
- Reset sequencing is correct: exact post-reset barriers are checked before
  all 14 same-version bounded local QPs, and rejection is atomic at UAV 3.
- Enlarging the speed bound, dropping the rate term, adding hard-row slack, or
  bypassing the guard would weaken the stated theorem and is not acceptable.

The outcome is a concrete instance of the documented conservatism of fixed
endpoint allocation: a coupled centralized problem may be feasible while the
chosen set of local half-allocated problems is not.

## Evidence defect

The runner preserves the human-readable stderr reason, but the rejected reset
record failed the Python supervisor's evidence schema immediately after the 14
initialization rows. The exact hard-QP rows and solver witness were therefore
not retained in the immutable raw stream. Future failure evidence must preserve
a `hard_qp_verified` rejected reset, controller abort, and mission terminal
without converting the first rejected-reset record into a generic
`schema_error`.

This is Important for auditability but does not change the primary scientific
verdict: the reconstructed infeasibility is already sufficient to reject v4.

## Theoretically honest v5 recovery boundary

The controller, hard rows, uncertainty-rate term, fixed two CBF references,
dynamic FIM-only graph, and component bounds should remain unchanged. V5 must
instead define and pre-register an admissible initial-state family. A suitable
direction is a geometrically dispersed convex arc/ring with only bounded,
seed-determined perturbations.

Before authorization, every registered seed must pass an admission audit over
all 119 post-reset barriers and all 14 exact bounded local hard QPs. The rule
must never silently resample until a seed passes; rejected proposed states and
the admission acceptance rate must be reported. Only admitted initial states
may enter the conditional theorem's mission-success experiment.

The existing specified triangular-ladder coordinates are not an adequate
drop-in fix: reconstructing them leaves multiple UAVs infeasible because
opposing close-neighbor collision endpoint rows remain.

Mandatory RED tests before production changes are:

1. Freeze the v4 mission-01 positions and assert UAV 3 infeasibility, including
   the 3--5/3--8 conflict witness.
2. Preserve a rejected `hard_qp_verified` reset, controller abort, and mission
   terminal through the supervisor schema and raw manifest.
3. Reject any v5 materialization that still uses unqualified
   `random-in-polygon` initialization.
4. For every registered v5 seed, require all 119 barriers nonnegative and all
   14 local QPs `optimal` with unchanged `+-25` component bounds.
5. Perturb an admissible template back toward the v4 conflict and require an
   explicit admission rejection without automatic resampling.
6. Retain tests proving fixed distance-CBF references and dynamic FIM-only
   references remain separate.

## Findings

Critical: `1` for the positive closure goal: development-v4 has 0/10 completed
missions and cannot support the paper's numerical claim.

Important: `1`: the exact rejected-reset/QP witness was lost at the supervisor
schema boundary and must be retained in v5 diagnostics.

Minor: `0`.

These counts do not label the controller's atomic fail-closed behavior as a
defect. They describe the failed experiment gate and its diagnostic gap.
