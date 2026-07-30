# CBF2026 Bounded Predictive WNLS Recovery Design

## Goal

Replace the current unbounded stale-retention behavior with a simple,
distributed estimator that uses the known zero-order-held planar command for
short-horizon prediction, rejects motion-inconsistent range solutions, and
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

The preserved trajectory also falsifies the previously assumed applied-input
bound.
Although the configuration stores `planar-component-max = 25.0`, it also
stores `input-limits.on = false`.
All `7,000` UAV-frame optimizer records confirm
`opt.input_limits.enabled = false`; `243` records have an applied
`opt.result` component above `25 m/s`.
The maximum is `35.03975016117459 m/s` at frame 346 for UAV 7, the maximum
planar norm is `35.660262715891044 m/s`, and the maximum observed one-frame
displacement is `17.830131357945504 m`.
The next recorded state equals the previous state plus
`0.5 * opt.result` to numerical precision, so these are applied commands,
not unused nominal values.
Consequently, the preserved trajectory may diagnose estimator mechanisms,
but it cannot substantiate a `25 m/s` applied-input assumption or a
`17.6777 m` maximum-displacement bound.

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
- a simulator may use truth only at the sensor boundary to emit a
  measurement-present bit and a noisy scalar range; runtime estimator
  functions receive neither truth coordinates nor true range;
- no lower-level dynamics, sampled-data reserve, or estimator-in-controller
  claim is introduced; and
- the old exactly-once analyzer and evidence are never replaced or rerun.

The new evidence may support improved finite-horizon estimation performance.
It must not be described as a deterministic true-error bound, arbitrary-depth
stability proof, unconditional safety guarantee, or proof that the modeled
inverse FIM equals actual covariance.
The preserved Stage 1 evidence must explicitly report the applied-input-bound
violations above and keep the paper gate closed.
Every new Stage 2 trajectory must enable and verify the hard `25 m/s`
component limits before it may support the bounded-input paper assumption.

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
private_reacquisition_seed
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

`attempt_status` is exactly one of:

- `accepted`: a selected candidate passed every enabled online gate;
- `rejected`: at least one numerical candidate completed, but none passed
  online acceptance;
- `failed`: every well-formed candidate exhausted the finite solver budget;
- `invalid`: malformed or non-finite estimator inputs or outputs prevented a
  valid attempt; or
- `reference_unavailable`: a mandatory reference or measurement was absent
  or not current-frame fresh.

An `accepted` attempt always publishes `fresh` at prediction age zero.
Any other attempt publishes `predicted` only when a prior published
fresh/predicted output can be propagated to age one or two; otherwise it
publishes `unavailable`.
Attempt status and output status are never conflated.
Attempt aggregation is deterministic:
missing mandatory input gives `reference_unavailable`; otherwise any selected
accepted candidate gives `accepted`; otherwise any numerically completed but
gate-rejected candidate gives `rejected`; otherwise any well-formed candidate
that exhausted its solver budget gives `failed`; otherwise the attempt is
`invalid`.

The maximum prediction lifetime is frozen at:

```text
MAX_PREDICTION_AGE_FRAMES = 2
```

At 2 Hz this is one second.
At age 3 the output becomes `unavailable`.

Expiry is not absorbing.
The public unavailable state contains no estimate, covariance, epsilon, or
provenance and cannot be used as a reference.
For re-acquisition only, the estimator may retain a private propagated seed
with an explicitly aged covariance.
That seed is never published, never communicated as a localization state,
never used for visibility, and is only one of the deterministic initial
candidates.
Every unavailable frame still attempts truth-free measurement-only
re-acquisition.
A re-acquisition may become `fresh` only with at least three current-frame
fresh references, at least two non-collinear directions, two distinct base
IDs in recursive provenance, a finite positive-definite final FIM, and a
numerically accepted WNLS result.
With fewer than three references it remains unavailable because a
two-reference mirror ambiguity cannot be resolved without a live motion
prior.

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
An `unavailable` public output cannot be propagated into a finite public
prediction.
Its private re-acquisition seed may continue the same command propagation,
but it remains ineligible for publication and reference use until a current
measurement update independently passes the re-acquisition rules.

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
For a fresh candidate with the unchanged range-only FIM covariance
\(P^{\rm range}_{i,k}\), the online prediction innovation covariance is

\[
S_{i,k}
=
P^{\rm pred}_{i,k}
+P^{\rm range}_{i,k}.
\]

This additive model is an explicit local Gaussian approximation.
The histories are not proven independent, so its normalized innovation is a
branch-consistency diagnostic and gate, not a calibrated stochastic theorem.

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
If a mandatory scalar range is absent, or a mandatory fixed UAV reference is
not fresh, that WNLS update is marked `reference_unavailable`; the observer
may publish its own short-horizon prediction, but it may not substitute a
different graph and call the result equivalent.

Optional dynamic references are selected only from visible, current-frame
fresh bases and strictly lower-index fresh UAVs.
Predicted or unavailable UAV states cannot be optional anchors.
This preserves the lower-index DAG and stops predicted/stale states from
being laundered into fresh high-confidence downstream estimates.

The runtime visibility predicate must not use truth coordinates.
At the simulation sensor boundary, truth may determine whether a range sensor
returns a measurement and may generate the noisy scalar range.
The runtime estimator receives only `(reference_key, measurement_present,
noisy_range)` plus communicated estimator states.
The experiment's global communication assumption makes communication
availability true; it does not manufacture an absent range measurement.
A base is optional-visible when its current range record is present.
A UAV is optional-visible when its current range record is present, it is in
the same squad with a strictly smaller local index, and its output is
current-frame `fresh`.
All optional-visible references are included in sorted base-ID then UAV-ID
order, matching the original include-all-visible policy after the freshness
filter.
No outcome-dependent subset search is added.
Legacy `active_references(..., truth_positions)` may be used only by the
sensor simulator to construct measurement-present records and must never be
called by prediction, reference qualification, candidate generation,
selection, or acceptance.

Within every `(variant, seed, frame)` block, UAVs execute strictly in
ascending global ID.
Only already completed, smaller-ID current-frame outputs may contribute
provenance or reference positions.
This execution order is part of the distributed DAG contract and is tested,
not inferred from serialized output order.

The diagnostic `prediction_expiry` Stage 1 ablation is the sole exemption
from fresh-reference qualification.
It preserves the immutable baseline active-reference policy so that the
effect of expiry can be attributed, and therefore may expose stale or
predicted anchor use.
Every such use is retained and counted as a violation.
The ablation is never eligible as the complete estimator or as paper
evidence; the qualification and complete variants obey all rules above.

The estimator-validity rule does not deactivate either fixed distance CBF.
It changes only whether a localization update has trustworthy reference
positions.

## Deterministic bounded multi-start WNLS

The measurement objective remains the current variable-weight whitened range
residual with the full residual Jacobian.
SciPy is not available in `cbf_env`, so the implementation remains pure
NumPy.

At most four deterministic initial candidates are generated:

1. the command-predicted position, or the private re-acquisition seed when no
   live public prediction exists;
2. a deterministic algebraic multilateration point from all current fresh
   references;
3. the first admissible two-circle branch from the best-conditioned
   deterministic reference pair; and
4. the second branch from the same pair.

Invalid or duplicate candidates are removed with a deterministic ordering.
No random restart is permitted.
The best-conditioned pair is chosen from the already constructed mandatory
plus optional active set.
When a public prediction exists, the pair maximizes the absolute
two-dimensional cross product of directions from that prediction to the
estimated references.
During re-acquisition, it instead maximizes the absolute sine of the implied
included angle computed from the two noisy ranges and estimated
reference-to-reference distance by the cosine law.
Infeasible pairs are skipped.
Reference keys use the total order `(base, id)` before `(uav, id)`, and that
order breaks pair scores tied within
`1e-12 * max(1, abs(score_a), abs(score_b))`.
Truth positions and offline errors are prohibited.

Candidate source order is `prediction` or `private_reacquisition_seed`,
`algebraic`, negative-oriented circle branch, then positive-oriented circle
branch.
Two finite candidates are duplicates when their Euclidean separation is at
most `1e-9 m`; the earlier source is retained.

Each candidate is solved with the same finite-budget damped Gauss--Newton
routine.
There is no velocity-derived solver-step or innovation radius in Stage 1.
The exact numerical rules are:

```text
maximum proposals per candidate = 50
initial damping = 1e-3
damping decrease after an accepted proposal = divide by 10
damping increase after a rejected proposal = multiply by 10
minimum damping = 1e-15
maximum damping = 1e15
representable-step relative threshold = 1e-12
```

One damped linear solve consumes one proposal.
A candidate always starts with damping `1e-3`; damping is not carried between
candidates, UAVs, frames, variants, or seeds.
A trial is accepted only when all terms are finite and its full whitened
objective is strictly less than the current floating-point objective.
After acceptance the next damping is `max(1e-15, damping / 10)`.
After rejection it is `damping * 10`; attempting to exceed `1e15` terminates
with a finite failure record.
If

\[
\|\delta\|_2
\leq
10^{-12}(1+\|\hat p\|_2)
\]

while the stationarity condition is false, the candidate terminates as
`no_representable_improving_step`.
Every proposal records its index, damping, current cost, stationarity norm,
raw step norm, trial cost or invalid-trial reason, and accepted boolean.
The numerical stationarity test is scale-aware:

\[
\|J^\top r\|_\infty
\leq
10^{-6}(1+\|r\|_2).
\]

All valid candidate results are retained in the process record.
Candidate selection is deterministic:

1. reject a candidate that fails the online acceptance rules below;
2. among remaining candidates, select the minimum full whitened objective;
3. treat costs as tied when their absolute difference is at most
   `1e-12 * max(1, abs(cost_a), abs(cost_b))`;
4. for a live prediction, break a cost tie by smaller normalized innovation;
   for re-acquisition, proceed directly to candidate order; and
5. break any remaining tie by the fixed candidate source order.

## Online acceptance

A current update is accepted as `fresh` only when:

- all mandatory references are current-frame fresh;
- all optional references are current-frame fresh;
- at least two non-collinear directions are available;
- the finite-budget solver satisfies its scale-aware termination criterion;
- the final FIM is finite and positive definite;
- every recorded value is finite.

The non-collinearity and positive-definiteness checks use the existing
relative spectral rule: the minimum eigenvalue must be strictly greater than
`1e-12` times the maximum eigenvalue, and the maximum must be positive.
The same rule applies to \(S\).

When a live public prediction exists, a candidate must also satisfy

\[
q^{\rm innov}
=
(\hat p^{\rm cand}-\hat p^{\rm pred})^\top
S^{-1}
(\hat p^{\rm cand}-\hat p^{\rm pred})
\leq
11.829007011943707.
\]

The threshold is the exact two-degree-of-freedom chi-square quantile for a
predeclared `0.9973` reference probability,
`-2 * log(0.0027)`.
It uses only command prediction and modeled covariances.
Because the covariance independence and calibration assumptions are
unproven, this is a frozen heuristic branch-consistency statistic referenced
to a chi-square quantile, not an achieved acceptance probability,
truth-error threshold, or deterministic bound.
If \(S\) is non-finite or not positive definite, the candidate is invalid.

When no live public prediction exists, the innovation gate is inapplicable.
The stricter three-reference re-acquisition conditions defined above apply,
and the row records `innovation_gate = not_applicable_reacquisition`.
A re-acquisition candidate must additionally satisfy the frozen
measurement-consistency rule

\[
\frac{\|r_{\rm white}\|_2^2}{\max(1,m-2)}
\leq 9,
\]

where \(m\) is the number of active scalar ranges and the numerator is the
unchanged full whitened WNLS objective at the candidate.
This three-whitened-residual RMS rule is an online heuristic selected before
Stage 1, not a calibrated goodness-of-fit theorem.
Every candidate failing it remains recorded as `rejected`.

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
   An accepted attempt with a selected WNLS candidate must publish `fresh`
   with offline fixture error no greater than `5 m` without truth entering
   the runtime branch decision.
   A prediction-only result does not pass this fixture and is reported
   separately.
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
6. **Re-acquisition fixture.**
   After an age-three unavailable output, a two-reference frame remains
   unavailable, while a later truth-free, three-reference non-collinear
   measurement update can publish a newly accepted `fresh` output.

Truth may appear only in test assertions and the offline evaluator.
It may not be passed to prediction, candidate generation, visibility,
reference qualification, candidate selection, or runtime acceptance.

### Stage 1: preserved-trajectory exploratory replay

Use the preserved 250 s truth trajectory and the existing 20 range-noise
seeds only as a paired development replay.
This is not new confirmatory evidence.
The seeds are exactly the integers `20260727` through `20260746`.

Evaluate:

1. the immutable baseline;
2. prediction and expiry only;
3. prediction, expiry, and fresh-reference qualification; and
4. the complete predictive multi-start estimator.

The ablations are cumulative.
`prediction_expiry` changes failed publication only and intentionally
preserves the baseline active-reference policy.
`fresh_reference_qualification` adds sensor-record visibility, freshness,
provenance, and re-acquisition rules while retaining the baseline
single-start solver.
`predictive_multistart` adds deterministic candidates, the finite-budget
solver, and the normalized innovation gate.
Only the final variant is eligible as the complete estimator.

For paired analysis, the immutable baseline maps as follows:

- baseline `attempt_status = converged` maps to accepted attempt;
- baseline attempt `failed` or `invalid` retains that attempt class;
- baseline output `status = converged` maps to fresh publication;
- a finite baseline `status = stale` remains a legacy published output but
  is never mapped to fresh; and
- a non-finite failed/invalid baseline row maps to unavailable.

Only immutable `dynamic_dag_wnls` baseline rows enter paired denominators.
Every variant must contain the same exact `(seed, frame, robot)` key set.

Report, for every variant:

- all attempted rows;
- fresh, predicted, unavailable, rejected, failed, and invalid counts;
- fresh-error and all-published-output error p50/p95/p99/maximum;
- offline errors of rejected candidates;
- maximum prediction age and unavailable streak;
- current-frame reference freshness and active-set composition;
- depth, squad, time-bin, and seed summaries;
- fresh-only FIM and coefficient-3 containment;
- fresh-only
  \(q^{\rm error}=e^\top(P^{\rm range})^{-1}e\), with its finite denominator,
  distribution, and exceedance counts above `5.991464547107979` and `9.0`;
- predicted-output aged-radius containment and aged
  \(q^{\rm error}\) with separate denominators;
- online \(q^{\rm innov}\) distributions for accepted and rejected
  candidates;
- the `243/7,000` applied-input component-bound violations, maximum applied
  component, maximum planar norm, and maximum displacement; and
- the previous `999.3319 m` and `168.9017 m` mechanism rows explicitly.

Raw rows use two unambiguous offline fields:
`offline_fresh_containment` is boolean only for `fresh` outputs and otherwise
null; `offline_aged_radius_containment` is boolean only for `predicted`
outputs and otherwise null.
Fresh and aged \(q^{\rm error}\) fields follow the same null/inapplicable
rule.

Stage 1 is promising only if the complete estimator:

- reduces the maximum published-output error below `50 m`;
- reduces the maximum fresh accepted error below `50 m`;
- does not worsen fresh-error p95 relative to its paired baseline;
- has fresh-update availability no more than two percentage points below the
  mapped paired baseline;
- has fresh-or-predicted availability of at least `95%`;
- has no prediction older than two frames;
- never uses a predicted or unavailable optional UAV reference; and
- has no current-frame provenance or ascending-DAG execution violation.

The anti-denominator audit uses the exact baseline-published cohort.
For every baseline-published key, it records the new output status, paired
new error when published, and the baseline error when newly unavailable.
Newly unavailable rows are attrition, never error improvements.
The report must give attrition count and rate, its baseline-error
p50/p95/p99/maximum, and catastrophic attrition count.
Error changes are claimed only on exact-key pairs where both systems publish;
the availability gates above decide whether attrition is acceptable.

A second fixed-cohort audit starts from every baseline-`fresh` key.
It publishes the full transition table from baseline fresh to new
fresh/predicted/unavailable.
Fresh-error change and fresh p95 are computed only on exact keys where both
systems are fresh.
For every baseline-fresh key excluded because the new status is predicted or
unavailable, the report gives its baseline-error distribution and
catastrophic count.
A fresh-to-predicted transition is a status downgrade, not fresh-error
improvement or unavailable attrition.
The two-percentage-point fresh-availability gate applies to this complete
baseline-fresh transition table.

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

Every Stage 2 configuration must set:

```text
cbfs.input-limits.on = true
cbfs.input-limits.planar-component-max = 25.0
```

Every optimizer row must then record `opt.input_limits.enabled = true`.
Before a trajectory is admitted, audit all applied commands and state
transitions:

```text
abs(opt.result.vx) <= 25.0 + 1e-7
abs(opt.result.vy) <= 25.0 + 1e-7
planar one-frame displacement <= sqrt(2) * 25.0 * 0.5 + 1e-7 m
abs((x[k+1] - x[k]) - 0.5 * opt.result.vx[k]) <= 1e-7 m
abs((y[k+1] - y[k]) - 0.5 * opt.result.vy[k]) <= 1e-7 m
```

A violation is an integrity failure, not an outlier to remove.
No trajectory is regenerated or replaced because of its estimator outcome.

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
Stage 2 repeats both fixed-cohort status-transition audits from Stage 1 for
each trajectory and in aggregate; fresh-to-predicted and newly unavailable
keys never enter an error-improvement denominator.

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
- every input-limit and state-transition integrity check passes;
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
Every trajectory-level catastrophic incidence is reported with a two-sided
`95%` Wilson score interval using `z = 1.959963984540054`.
Thus even `0/12` is finite-sample evidence with a nonzero population-incidence
upper limit, not proof that the population incidence is zero.

## Evidence lifecycle

The implementation and analyzer must:

- use a new schema and new output directory;
- preserve the old restart stream and exactly-once geometric-stability
  output byte-for-byte;
- fail closed on source, input, manifest, or output-identity drift;
- treat each registered `output_root` as the exact bundle directory, create
  it exclusively, and refuse any pre-existing target whether empty or
  nonempty;
- write a terminal manifest for replay and analyzer on both success and
  failure, with source/input/output hashes, status, and failure reason;
- never allocate a timestamped or nested run beneath a registered
  `output_root`;
- cap compact output at `10 MB`;
- keep raw bundles outside Git with manifests and SHA-256 identities;
- keep rebuildable cache below `2 GB`;
- require at least `8 GB` before launch and stop below `6 GB`;
- never retry a registered run automatically; and
- never delete or omit a catastrophic, failed, rejected, or unavailable row.

Protocol provenance is non-circular.
Implementation and tests are committed first.
The Stage 1 protocol then binds that parent commit plus exact replay,
analyzer, truth, input-manifest, and baseline-process hashes, and is committed
separately.
An independent protocol/source review must be resolved and committed before
the registered replay directory is created.
The registered replay runs once, followed by one analyzer run.
If either terminates unsuccessfully, its terminal manifest is frozen and no
automatic or manual same-protocol retry is permitted.

The current disk has ample space, but the same guards remain in force.

## Paper and DRA decision

Stage 0 and Stage 1 update code and DRA only.
They do not replace the paper's existing negative/exploratory evidence.
Because the preserved trajectory violates the assumed applied-input bound,
the paper-edit gate remains closed regardless of its Stage 1 estimator
metrics.

Paper revision is allowed only after:

1. Stage 1 passes all gates;
2. the new evidence receives an independent raw-row audit;
3. no Critical or Important review issue remains; and
4. Stage 2 completes its registered bounded-input confirmation and passes
   every input-integrity and estimator gate.

An incomplete or explicitly exploratory Stage 2 may be recorded in DRA but
does not open the paper-edit gate.

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
