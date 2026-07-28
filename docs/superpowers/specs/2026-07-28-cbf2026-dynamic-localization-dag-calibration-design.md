# CBF2026 Dynamic Localization DAG Calibration Design

## Decision

The CBF formation graph and the localization-information graph are different
objects and shall no longer be conflated.

- The formation/connectivity CBF uses two predeclared references per search
  UAV. These references are fixed and the controller tries to keep them
  within the communication/ranging limit.
- The localization graph is an active, time-varying DAG. Its Fisher
  information matrix (FIM) uses the two fixed references plus every currently
  visible eligible base and lower-index UAV reference.
- A simple range-only estimator shall run as an offline sidecar over existing
  truth trajectories. Its estimates shall not enter the CBF, CVT, communication
  state, or simulated vehicle dynamics.
- Only one estimator, weighted nonlinear least squares (WNLS), shall be used.
  Estimator comparisons are out of scope because localization-algorithm design
  is not a paper contribution.

The experiment shall test whether the existing FIM-derived radius empirically
contains localization error and whether the dynamic localization DAG reduces
geometric degeneration relative to the fixed-reference-only ablation.

## Research Boundary

This design does not add:

- EKF, UKF, factor-graph, motion-model, or process-noise estimation;
- estimated positions in the control loop;
- lower-level flight dynamics or sampled-data safety theory;
- an uncertainty-jump reserve;
- a new controller architecture or a repeat of the pairwise-local QP gate;
- an estimator-performance comparison in the manuscript.

The estimator is a calibration instrument for the uncertainty model, not a
claimed contribution.

## Two Graphs

Let \(\mathcal G_{\mathrm{CBF}}\) be the fixed formation/connectivity graph.
For search UAV \(i\), let
\(\mathcal R_i^{\mathrm{fixed}}\) contain its two predeclared CBF references,
which may be assigned bases or strictly lower-index UAVs.
These references remain active for the CBF and the localization FIM even if a
realized trajectory temporarily exceeds `max-range`; maintaining that range
is the purpose of the fixed communication CBF.

Let the active localization reference set be

\[
\mathcal R_i(t)
=
\mathcal R_i^{\mathrm{fixed}}
\cup
\mathcal B_i^{\mathrm{visible}}(t)
\cup
\mathcal A_i^{\mathrm{visible}}(t),
\]

where:

- \(\mathcal B_i^{\mathrm{visible}}(t)\) contains every base within
  `max-range`;
- \(\mathcal A_i^{\mathrm{visible}}(t)\) contains every same-squad UAV within
  `max-range` whose squad-local index is strictly smaller than \(i\)'s; and
- duplicate fixed references are included only once.

All eligible references are used. There is no information-gain ranking,
best-pair search, or maximum-anchor count.

Bases are roots. Every UAV covariance dependency points from a strictly lower
squad-local index to a higher one. Therefore the active graph can switch while
remaining acyclic at every time. Increasing squad-local index is a valid
topological processing order for every active set.

This target preserves the historical CBF2026 hybrid-union behavior and makes
the researcher's fixed-CBF-reference contract explicit. In baseline
`47e7b3d`, fixed lower-index UAV neighbours were retained unconditionally,
optional lower-index UAVs were range gated, and all bases were range gated.
The approved target additionally retains an assigned fixed base outside
`max-range`; within the intended CBF-maintained range the rules coincide.
The later diagnostic commits that froze the complete FIM information set
record a disproven design interpretation; their history shall be preserved,
but a future implementation commit shall restore the dynamic rule rather than
rewrite those commits.

## Measurement Model

For every active directed range edge \(i\leftarrow j\),

\[
z_{ij}(t)
=
\lVert p_i(t)-p_j(t)\rVert
+\eta_{ij}(t),
\qquad
\eta_{ij}(t)\sim\mathcal N(0,\sigma_r^2).
\]

The first experiment uses the paper configuration
\(\sigma_r=0.5\,\mathrm{m}\).
The truth trajectory supplies only the physical distance and active-set
visibility. It is not supplied to the estimator after initialization.

Noise must be reproducible and paired across graph ablations. Each sample shall
be keyed by the tuple

`(run_seed, frame_index, observer_id, reference_kind, reference_id)`

using a stable seed derivation, not Python's process-randomized `hash()`.
Consequently, every common edge receives the same synthetic measurement in the
dynamic-DAG and fixed-reference-only cases even though the two cases contain
different numbers of edges.

## Minimal Estimator

At each frame, search UAVs are processed in increasing squad-local index.
Bases have exact known positions and zero covariance. For UAV \(i\), use the
estimates and covariance approximations already produced for its active
lower-index references.

The estimate is the weighted nonlinear least-squares solution

\[
\hat p_i
=
\arg\min_p
\sum_{j\in\mathcal R_i(t)}
w_{ij}
\left(
\lVert p-\hat p_j\rVert-z_{ij}
\right)^2,
\]

with

\[
w_{ij}
=
\frac{1}{
\sigma_r^2+
e_{ij}^{\mathsf T}P_j e_{ij}
},
\qquad
e_{ij}
=
\frac{\hat p_i-\hat p_j}
{\lVert\hat p_i-\hat p_j\rVert}.
\]

For a base, \(P_j=0\). The weights may be updated within each Gauss--Newton or
Levenberg--Marquardt iteration. Levenberg--Marquardt damping is preferred as a
numerical safeguard, but it does not change the estimator model.

At \(t=0\), initialize every UAV at its known deployment position. At later
frames, initialize with its previous estimate. There is no motion prediction,
truth reset, or temporal covariance update. The previous estimate resolves
the two-range mirror branch and provides a stable starting point when more
references are active.

If a later-frame optimization does not converge or produces a non-finite
result, record the failure and retain the previous finite estimate and
covariance as explicitly stale values so downstream DAG nodes can still be
evaluated. If no finite previous result exists, mark the current node and its
dependent downstream results invalid rather than inventing a covariance.
Failed and stale frames remain in failure and coverage statistics; they shall
not be silently discarded.

## FIM and Radius

At the converged estimate, form

\[
\Phi_i
=
\sum_{j\in\mathcal R_i(t)}
w_{ij}e_{ij}e_{ij}^{\mathsf T},
\qquad
P_i=\Phi_i^{-1}.
\]

The scalar design radius remains

\[
\epsilon_i
=
3\sqrt{\lambda_{\max}(P_i)}.
\]

The calibration event is

\[
\lVert p_i-\hat p_i\rVert\leq\epsilon_i.
\]

No additional factor of three is applied to \(\epsilon_i\).

The diagonal range-weight model propagates each reference covariance but
does not maintain cross-covariances created by shared upstream references.
Accordingly, \(P_i\) remains a FIM-derived covariance approximation rather
than a deterministic error upper bound. The Monte Carlo experiment measures
whether the resulting radius is empirically adequate despite this
approximation.

The continuous \(\dot\Phi_i\) and \(D^+\epsilon_i\) expressions are valid
between active-set transitions. This experiment records switch events and
their observed \(\epsilon_i\) changes; it does not introduce a jump reserve or
claim a global differentiable FIM across switches.

## Offline Replay Architecture

The first implementation shall be a Python offline replay tool. It shall:

1. read an existing diagnostic `data.json` truth trajectory without modifying
   it;
2. reconstruct the historical dynamic localization reference set at every
   frame;
3. synthesize keyed Gaussian ranges;
4. execute the topological WNLS pass for both the dynamic-DAG case and the
   fixed-reference-only ablation;
5. write a separate compact calibration bundle; and
6. analyze the bundle without rerunning the CBF simulator.

The output bundle shall reference the input trajectory by absolute path,
content hash, source commit, and materialized configuration hash. It shall not
duplicate the truth trajectory.

Required process records are:

- active base and UAV reference IDs;
- true and noisy ranges and generated noise for each active edge;
- estimate, error vector, error norm, convergence status, iteration count,
  and final residual cost;
- \(P_i\), \(\epsilon_i\), containment indicator,
  \(\lambda_{\min}(\Phi_i)\), FIM condition number, and finite/SPD status;
- active-set transitions and signed changes in \(\epsilon_i\); and
- run seed, estimator settings, input hashes, timestamps, and disk probes.

Use compressed JSON Lines or an equivalently inspectable compact format.
Keep summaries as ordinary JSON and Markdown. Do not retain duplicate
intermediate arrays that can be reconstructed from the recorded measurements
and estimates.

## Experimental Comparison

Use one estimator and two reference-graph cases:

1. `dynamic_dag_wnls`: the historical hybrid-union localization graph; and
2. `fixed_refs_wnls`: only the predeclared fixed base/UAV localization
   references.

This is a graph-design ablation, not an estimator comparison. Both cases use
the same truth trajectory, initialization, optimizer settings, common-edge
noise samples, and FIM equations.

Primary calibration outputs are:

- empirical \(\epsilon_i\) containment rate;
- distribution of
  \(\lVert p_i-\hat p_i\rVert/\epsilon_i\);
- estimator failure/non-finite rate;
- lower-tail \(\lambda_{\min}(\Phi_i)\);
- upper-tail FIM condition number and \(\epsilon_i\); and
- results stratified by squad-local depth and run seed.

The \(t=0\) initialization frame is retained in the process log but excluded
from the primary containment rate because its position is supplied by the
known deployment initialization. Its result shall be reported separately.

Primary dynamic-graph outputs are paired changes in:

- invalid or near-singular FIM frequency;
- lower quantiles of \(\lambda_{\min}(\Phi_i)\);
- upper quantiles and maxima of \(\epsilon_i\);
- estimator failure rate; and
- containment rate.

Frame and robot observations are temporally and genealogically correlated.
Confidence intervals shall therefore use independent run seeds as the
resampling unit, not treat every robot-frame as an independent sample.

## Staged Gate

### Stage 0: deterministic and synthetic checks

Verify:

- keyed measurement noise is exactly reproducible;
- common edges have identical noise across graph cases;
- the lower-index rule makes every active set acyclic;
- a non-collinear two-reference zero-noise case recovers the target;
- adding a third reference is handled without changing estimator type;
- the previous estimate selects the continuous two-range branch;
- singular/failed cases are logged and not discarded; and
- a hand-computed FIM and \(\epsilon_i\) match the implementation.

### Stage 1: one minimal replay

Replay one preserved 20 s trajectory. Require:

- complete process records for every expected UAV-frame;
- no unexplained missing or non-finite values;
- deterministic rerun hashes;
- graph memberships matching the historical hybrid rule; and
- summary values recomputable from the process log.

This stage is a software/evidence smoke test, not paper evidence.

### Stage 2: paired Monte Carlo calibration

After Stage 1 review, run paired independent seeds on the preregistered
trajectory/initial-geometry set. Do not tune noise, graph rules, or radius
after observing coverage. Report the measured coverage and uncertainty
instead of asserting a mission-level probability guarantee.

The initial adequacy gate is:

- aggregate Euclidean containment at least \(98\%\);
- no squad-local depth with containment below \(95\%\);
- zero silently discarded optimizer failures; and
- dynamic-DAG invalid-FIM and estimator-failure frequencies no worse than the
  fixed-reference-only ablation.

If the gate fails, stop manuscript upgrading and diagnose the approximation.
Do not automatically add covariance inflation, replace the estimator, or
change \(\epsilon_i\) in the same evidence round.

## Disk and Evidence Rules

- require at least 8 GB free before launching a replay batch;
- stop below the 6 GB live free-space floor;
- keep retained experiment cache below 2 GB;
- keep each calibration bundle below the existing 250 MB abnormal-growth cap;
- reuse truth trajectories by hash rather than copying them;
- use new output roots and never overwrite previous diagnostic evidence;
- record size and free-space probes before and after every run; and
- do not delete evidence without separate researcher authorization.

## Manuscript and DRA Boundary

Before calibration evidence is reviewed, do not add estimator results to the
paper. The eventual paper revision should:

- replace the current fixed-FIM-graph wording with the two-graph distinction;
- prove acyclicity of the dynamic localization graph from the lower-index
  order;
- describe WNLS only as the calibration estimator;
- retain “FIM-derived covariance approximation” and empirical-radius language;
- report graph ablation and coverage without estimator comparisons; and
- state that derivative expressions are piecewise between active-set
  transitions and that switching is evaluated empirically.

DRA shall record the correction from the fixed-information-set interpretation,
the approved design, all implementation commits, input/output hashes, stop
decisions, and the paper-evidence boundary.

## Acceptance Boundary

The design is accepted when the researcher approves this document.
Implementation is accepted only after the test-first Stage 0 contract and the
Stage 1 evidence smoke pass. Paper evidence is accepted only after the
preregistered Stage 2 paired calibration is reviewed.

None of these results, by themselves, proves closed-loop estimator robustness,
mission-level probabilistic safety, cross-correlation consistency, bounded
joint QP feasibility, or lower-level flight-dynamics tracking.
