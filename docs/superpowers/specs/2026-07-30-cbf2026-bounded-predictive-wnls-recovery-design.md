# CBF2026 Bounded Predictive WNLS Recovery Design

## Goal

Replace the current unbounded stale-retention behavior with a simple,
distributed estimator that uses the known zero-order-held planar command for
short-horizon prediction, rejects globally untrustworthy range solutions, and
prevents stale lower-index states from being promoted as fresh localization
anchors.

The primary engineering objective is to eliminate kilometre-scale published
position errors without hiding failed attempts or changing the original
FIM/radius construction.

## Frozen diagnosis

The completed restart replay remains immutable.
Its `999.3318962079554 m` maximum is not a newly converged WNLS estimate.
For seed `20260736`, UAV 14 last converged at frame 43 with `1.6863 m`
error, then all attempts in frames 44--138 failed.
The replay copied the frame-43 estimate and covariance unchanged for 95
frames while the UAV continued moving.
The retained-state error therefore grew to `999.3319 m`.

The stale tail is not the only defect.
Among current-frame converged dynamic attempts, the maximum error is
`168.90169712504604 m`.
At seed `20260730`, frame 177, stale or inaccurate upstream anchors enter
two-range, no-base subproblems.
Several downstream WNLS solves have essentially zero residual while their
global errors grow from `82 m` to `169 m`.
This is an anchor-integrity and global-branch problem, not a local FIM-rank
failure.

A one-row diagnostic also shows that the absolute `1e-9` stationarity
threshold is too strict for some useful solutions:
at seed `20260736`, UAV 14, frame 44, a `1e-6` threshold accepts a
`2.657 m` solution.
However, changing only that threshold in a 95-frame local replay still
produces a maximum error above `700 m`.
Solver tolerance relaxation alone is therefore prohibited as the remedy.

The read-only diagnosis is recorded in:

- `/private/tmp/cbf2026-tail-provenance-report.md`;
- `/private/tmp/cbf2026-estimator-audit-report.md`; and
- `/private/tmp/cbf2026-better-result-design-report.md`.

## Scientific boundaries

The implementation must preserve the following:

- the estimator remains outside the controller and does not alter the
  vehicle, CVT, communication, or CBF state;
- the distributed lower-index DAG is retained;
- the two fixed CBF distance references remain fixed and their distance CBFs
  remain active;
- optional localization references remain bases or strictly lower-index UAVs;
- the variable-weight range residual and its full residual Jacobian remain
  the measurement objective;
- the existing FIM and coefficient-3 epsilon formulas remain unchanged for
  a newly accepted measurement update;
- no truth position, truth error, future frame, or offline percentile may
  enter initialization, reference selection, acceptance, or re-acquisition;
- the continuous-time planar velocity-command model and communicated previous
  command with zero-order hold are the only motion model;
- no lower-level dynamics, sampled-data reserve, or estimator-in-controller
  claim is introduced; and
- the old exactly-once analyzer and evidence are never replaced or rerun.

The new evidence may support improved finite-horizon estimation performance.
It must not be described as a deterministic true-error bound, arbitrary-depth
stability proof, unconditional safety guarantee, or proof that the modeled
inverse FIM equals actual covariance.

## Alternatives considered

### Stale timeout only

Marking an estimate unavailable after a short timeout would remove the
`999 m` value from the accepted-output denominator and stop downstream stale
propagation.
It would not improve the underlying estimate and would mainly exchange error
for unavailability.
This is required as a safety/validity policy but is insufficient alone.

### Solver-only improvement

Increasing iterations, relaxing the gradient threshold, or adding a generic
optimizer may recover some failed frames.
It can also accept a low-residual wrong branch when upstream anchors are
globally wrong.
This is insufficient without motion continuity and reference freshness.

### Combined predictive recovery

The selected design combines:

1. command-based prediction;
2. deterministic bounded multi-start WNLS;
3. online innovation and numerical acceptance;
4. a two-frame prediction lifetime; and
5. freshness-qualified lower-index references.

This is the smallest design that addresses both the `999 m` stale tail and
the `168.9 m` current-frame cascade while preserving the paper's distributed
dynamic-reference contribution.

## Estimator state and output semantics

Each UAV and range-noise seed carries an estimator state with:

```text
estimate
modeled_covariance
epsilon
last_measurement_frame
prediction_age
output_status
attempt_status
reference_freshness
base_anchor_provenance
```

`output_status` is one of:

- `fresh`: a current-frame WNLS update was accepted using bases and only
  current-frame fresh UAV references;
- `predicted`: the current WNLS update was not accepted, but a command-based
  prediction is published and `prediction_age` is 1 or 2; or
- `unavailable`: no accepted update exists and the prediction lifetime has
  expired.

Failed, rejected, and invalid attempts remain separately visible through
`attempt_status`.
A `predicted` output must never be relabelled as `fresh`.

The maximum prediction lifetime is frozen at:

```text
MAX_PREDICTION_AGE_FRAMES = 2
```

At 2 Hz this is one second.
At age 3 the output becomes `unavailable`.

## Command-based prediction

For a previous finite output and the previous applied planar command,

\[
\hat p^{\rm out}_{i,k|k-1}
=
\hat p^{\rm out}_{i,k-1}
+\Delta t\,u_{i,k-1}.
\]

Here \(u_{i,k-1}\) is the command stored in frame \(k-1\) and applied on the
state transition from frame \(k-1\) to frame \(k\).
The replay must read it from the preserved frame-\((k-1)\) applied QP result.
It must not infer velocity from truth.
When a measurement update fails, the next prediction continues from the
previous output state---whether `fresh` or `predicted`---using the next known
applied command rather than freezing the last accepted position.
An `unavailable` state cannot be propagated into a finite prediction.

For a predicted output, the modeled covariance is aged as

\[
P^{\rm out}_{i,k|k-1}
=
P^{\rm out}_{i,k-1}
+Q_{\rm motion},
\]

where

\[
Q_{\rm motion}
=
\sigma_{\rm motion}^{2}I,
\qquad
\sigma_{\rm motion}=0.5~\mathrm{m/frame}.
\]

The `0.5 m/frame` scale is frozen to the registered one-frame range-noise
standard deviation rather than fitted to the observed `999 m` or `168.9 m`
tails.
This inflation is an explicit protocol penalty, not a proven physical process
covariance.
It is added again on every consecutive prediction, so age-two covariance is
the previous fresh covariance plus \(2Q_{\rm motion}\).
It must be reported as such and must not be used to claim a deterministic
true-error envelope.
The coefficient-3 radius may be reported for a predicted output only with
the label `aged modeled radius`.

## Reference freshness and DAG behavior

Base references are always fresh.
A UAV reference is fresh only if its current-frame `output_status` is
`fresh`.

Every fresh estimate also carries `base_anchor_provenance`, the union of
base IDs that enter its current lower-index dependency tree.
A direct base reference contributes its own ID.
A UAV reference contributes only the provenance attached to its own
current-frame fresh estimate.
A current WNLS output may be labelled `fresh` only when this union contains
at least two distinct base IDs.
This is a structural global-anchor certificate, not a proof that the
reference estimates are accurate or independent.
The empirical evaluation must still report globally wrong fresh solutions.

The two fixed localization reference IDs remain mandatory inputs whenever a
current WNLS update is attempted.
They are never silently removed in order to improve conditioning.
If a mandatory fixed UAV reference is not fresh, that WNLS update is marked
`reference_unavailable`; the observer may publish its own short-horizon
prediction, but it may not substitute a different graph and call the result
equivalent.

Optional dynamic references are selected only from visible, current-frame
fresh bases and strictly lower-index fresh UAVs.
Predicted or unavailable UAV states cannot be optional anchors.
This preserves the lower-index DAG and stops predicted/stale states from
being laundered into fresh high-confidence downstream estimates.

The runtime visibility predicate must not use truth.
A base is optional-visible when its ranging/communication record is present
and the observer's command-predicted position places it within the configured
`max-range`.
A UAV is optional-visible when its current communication record is present,
it is in the same squad with a strictly smaller local index, its output is
current-frame `fresh`, and the predicted observer-to-estimated-reference
distance is within `max-range`.
All optional-visible references are included in sorted base-ID then UAV-ID
order, matching the original include-all-visible policy after the freshness
filter.
No outcome-dependent subset search is added.

The estimator-validity rule does not deactivate either fixed distance CBF.
It changes only whether a localization update has trustworthy reference
positions.

## Deterministic bounded multi-start WNLS

The measurement objective remains the current variable-weight whitened range
residual with the full residual Jacobian.
SciPy is not available in `cbf_env`, so the implementation remains pure
NumPy.

At most four deterministic initial candidates are generated:

1. the command-predicted position;
2. a deterministic algebraic multilateration point from all current fresh
   references;
3. the first admissible two-circle branch from the best-conditioned
   deterministic reference pair; and
4. the second branch from the same pair.

Invalid or duplicate candidates are removed with a deterministic ordering.
No random restart is permitted.
The best-conditioned pair is chosen from the already constructed mandatory
plus optional active set using only command-predicted observer position and
estimated reference positions.
It maximizes the absolute two-dimensional cross product of the two predicted
unit directions, with lexicographic `(kind, id)` order as the tie break.
Truth positions and offline errors are prohibited.

Each candidate is solved with the same bounded damped Gauss--Newton routine.
Every proposed step is clipped to the task-derived Euclidean trust radius

\[
R_{\rm step}
=
\sqrt{2}\,v_{\max}\Delta t
=
17.67766953~\mathrm{m},
\]

using the frozen `25 m/s` component bound and `0.5 s` frame interval.
The numerical stationarity test is scale-aware:

\[
\|J^\top r\|_\infty
\leq
10^{-6}(1+\|r\|_2).
\]

The solver must also stop cleanly when no representable improving step
exists; it must not increase damping until overflow.

All valid candidate results are retained in the process record.
Candidate selection is deterministic:

1. reject a candidate that fails the online acceptance rules below;
2. among remaining candidates, select the minimum full whitened objective;
3. break an objective tie by smaller distance to the command prediction; and
4. break any remaining tie by candidate order.

## Online acceptance

A current update is accepted as `fresh` only when:

- all mandatory references are current-frame fresh;
- all optional references are current-frame fresh;
- at least two non-collinear directions are available;
- the bounded solver satisfies its scale-aware termination criterion;
- the final FIM is finite and positive definite;
- the selected estimate is within \(R_{\rm step}=17.67766953~\mathrm{m}\)
  of the command prediction; and
- every recorded value is finite.

The innovation gate uses the maximum possible one-frame planar displacement
under the already frozen component input bound.
Because command prediction has already applied that displacement, the gate is
deliberately conservative.
It is an online branch-continuity rule, not a truth-error threshold.

Objective cost remains a diagnostic rather than a hard universal threshold
in the first implementation.
The frame-177 cascade demonstrates that a near-zero cost can accompany a
large global error.

If no candidate passes, the attempt is `rejected` or `failed`, its candidate
details remain logged, and the output follows the prediction-age policy.

## Development stages

### Stage 0: deterministic regression fixtures

Create immutable fixtures from the already observed mechanisms:

1. **Frame-44 recovery fixture.**
   The command prediction must move UAV 14 from its frame-43 estimate rather
   than freeze it.
   A valid bounded solve must recover an output with offline fixture error no
   greater than `5 m` without truth entering the runtime branch decision.
2. **Frame-177 reference-integrity fixture.**
   A stale UAV 8 must not be accepted as a fresh reference, and its status
   must not produce a fresh UAV 10--14 cascade.
   No fixture output labelled `fresh` may have offline error above `50 m`.
3. **Two-circle ambiguity fixture.**
   Both range-consistent branches must be represented and the online
   innovation rule must reject the branch inconsistent with the command
   prediction.
4. **Prediction-expiry fixture.**
   Ages 1 and 2 are `predicted`; age 3 is `unavailable`; predicted outputs
   cannot act as UAV anchors.
5. **Evidence fixture.**
   Every attempted row remains present with attempt/output status, prediction
   age, candidate diagnostics, reference freshness, and offline truth error.

Truth may appear only in test assertions and the offline evaluator.
It may not be passed to prediction, candidate generation, visibility,
reference qualification, candidate selection, or runtime acceptance.

### Stage 1: preserved-trajectory exploratory replay

Use the preserved 250 s truth trajectory and the existing 20 range-noise
seeds only as a paired development replay.
This is not new confirmatory evidence.

Evaluate:

1. the immutable baseline;
2. prediction and expiry only;
3. prediction, expiry, and fresh-reference qualification; and
4. the complete predictive multi-start estimator.

Report, for every variant:

- all attempted rows;
- fresh, predicted, unavailable, rejected, failed, and invalid counts;
- fresh-error and all-published-output error p50/p95/p99/maximum;
- offline errors of rejected candidates;
- maximum prediction age and unavailable streak;
- current-frame reference freshness and active-set composition;
- depth, squad, time-bin, and seed summaries;
- FIM, coefficient-3 containment, and \(q\) diagnostics with separate
  denominators; and
- the previous `999.3319 m` and `168.9017 m` mechanism rows explicitly.

Stage 1 is promising only if the complete estimator:

- reduces the maximum published-output error below `50 m`;
- reduces the maximum fresh accepted error below `50 m`;
- does not worsen fresh-error p95 relative to its paired baseline;
- has no prediction older than two frames;
- never uses a predicted or unavailable optional UAV reference; and
- does not improve an error statistic solely by dropping its denominator.

Failure of a gate stops paper editing but does not authorize parameter tuning
on the same output.

### Stage 2: independent trajectory confirmation

After Stage 1 code and parameters are frozen and independently reviewed,
generate a disjoint confirmation set of:

```text
12 independent 250 s truth trajectories
5 predeclared range-noise seeds per trajectory
```

Trajectory seeds are the independent unit.
Noise seeds, frames, UAVs, and rows are nested repeated observations.
All variants use paired trajectories and paired range noise.

The catastrophic threshold is frozen at:

```text
E_CAT = 50 m
```

This threshold is chosen before new trajectories are generated.
It is five times the paper's configured `10 m` safety distance and exceeds
the `35.36 m` maximum planar travel during the one-second prediction lifetime.
It is not selected from the observed `999 m` or `168.9 m` values.

For each trajectory, report whether any fresh accepted estimate exceeds
`E_CAT` and whether any published fresh-or-predicted output exceeds `E_CAT`,
together with the trajectory's maximum fresh error, maximum published-output
error, fresh-update availability, fresh-or-predicted availability, and
longest unavailable streak.

The complete estimator passes only if:

- its trajectory-level catastrophic fresh-error incidence is lower than the
  paired baseline and is zero on the 12 registered trajectories;
- its trajectory-level catastrophic all-published-output incidence is lower
  than the paired baseline and is zero on the 12 registered trajectories;
- its paired fresh-error p95 does not worsen;
- its fresh-update availability is no more than two percentage points below
  the paired baseline;
- its overall fresh-or-predicted availability is at least `95%`;
- no predicted output exceeds age two;
- no predicted/unavailable UAV is used as an optional anchor;
- coefficient-3 containment does not degrade by more than two percentage
  points relative to the paired baseline; and
- all registered trajectories complete without outcome-based replacement.

The confirmation report must include separate adverse tables for every
predicted output, rejected candidate, unavailable row, and failed/invalid
attempt.
Neither the fresh-only table nor combined availability may replace these
tables.

If fewer than ten trajectories complete for integrity-independent reasons,
the result remains exploratory.

## Evidence lifecycle

The implementation and analyzer must:

- use a new schema and new output directory;
- preserve the old restart stream and exactly-once geometric-stability
  output byte-for-byte;
- fail closed on source, input, manifest, or output-identity drift;
- cap compact output at `10 MB`;
- keep raw bundles outside Git with manifests and SHA-256 identities;
- keep rebuildable cache below `2 GB`;
- require at least `8 GB` before launch and stop below `6 GB`;
- never retry a registered run automatically; and
- never delete or omit a catastrophic, failed, rejected, or unavailable row.

The current disk has ample space, but the same guards remain in force.

## Paper and DRA decision

Stage 0 and Stage 1 update code and DRA only.
They do not replace the paper's existing negative/exploratory evidence.

Paper revision is allowed only after:

1. Stage 1 passes all gates;
2. the new evidence receives an independent raw-row audit;
3. no Critical or Important review issue remains; and
4. Stage 2 is either completed as registered confirmation or explicitly
   labelled exploratory.

The paper must show both error and availability.
It may not report only the disappearance of the `999 m` stale row.

## Explicit non-goals

- estimator comparison or estimator survey;
- controller-in-the-loop localization;
- changing the CBF distance-reference graph;
- changing the FIM formula or coefficient-3 radius;
- modeling lower-level tracking dynamics;
- adding sampled-data reserve;
- using truth to select a solver branch or reference;
- suppressing failed rows or stale-tail evidence; or
- proving arbitrary-depth stochastic stability.
