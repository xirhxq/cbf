# CBF2026 Geometric Nondegeneracy and Empirical Stability Design

## Decision

The paper shall not attempt to derive a deterministic upper bound on the true
localization error from the inverse Fisher information matrix (FIM).
In the analysis and new theory text, \(P_i:=\Phi_i^{-1}\) shall denote the
modeled inverse-FIM matrix and
\(C_i:=\operatorname{Cov}(p_i-\hat p_i)\) shall denote the true estimator-error
covariance when that statistical quantity is discussed.
They must not be identified without an explicit calibration assumption.
Instead, it shall make and test three separate claims:

1. an explicit lower bound on the active reference-set geometry matrix
   prevents the modeled range FIM from becoming singular;
2. a finite lower-index localization DAG under that geometric condition
   produces
   finite FIM-derived covariance matrices whenever its root covariances and
   ranging variances are finite; and
3. Monte Carlo evidence determines whether the triangular-ladder controller
   and dynamic active reference set actually maintain the geometric
   condition, whether actual localization error remains operationally stable,
   and whether the coefficient-3 FIM radius is empirically calibrated under
   the tested finite squad depth, trajectories, and range-noise model.

The first two claims are analytic.
The third is empirical.
They must not be combined into an end-to-end deterministic true-error or
controller-safety guarantee.

## Motivation

The completed warm-start recovery intervention showed that changing
initialization did not alter any of the 67,042 dynamic-DAG primary rows on
which both policies converged.
It instead recovered 60,001 additional converged rows that were concentrated
at deeper squad-local indices.
These rows exposed weaker conditional calibration:

- coefficient-3 epsilon containment was lower in the recovered population;
- normalized squared error \(q=e^{\mathsf T}P^{-1}e\) had a heavier upper
  tail; and
- the aggregate safeguard changes were explained exactly by the changed
  converged population.

This result does not show that localization error diverges.
It shows that inverse-FIM calibration and absolute error stability are
different questions.
The paper's intended mechanism is geometric:
the triangular-ladder target construction and its tracking controller preserve
a formation skeleton, while the dynamic information set can add visible
reference directions when the fixed pair becomes poorly conditioned.
Together they are intended to maintain a usable active-set localization
geometry so that the modeled uncertainty does not become singular through
geometric degeneration.
The localization-connectivity CBF alone supplies an upper range constraint;
it does not, by itself, prevent the three members of a localization triangle
from becoming collinear.

A preliminary read-only inspection found near-collinear fixed-pair samples
while the complete dynamic active set remained nonsingular on the inspected
trajectory.
The committed analyzer must reproduce and audit this signal before it is used
as paper evidence.

## Claim boundary

### Analytic claims in scope

The theory may establish:

- nominal noncollinearity of target triangles where the target construction
  supplies it;
- a sufficient conditional fixed-pair angle margin when estimated positions
  track those targets within an explicit perturbation radius;
- positive definiteness of the modeled FIM under an explicit active-set
  geometry-matrix lower bound;
- monotonicity of the modeled FIM when eligible dynamic references are added;
- well-posedness and finiteness of every modeled covariance in a finite
  lower-index DAG, by topological induction; and
- the existing robust CBF result conditional on the adopted localization
  error radius containing the true error.

### Empirical claims in scope

Monte Carlo evidence may establish, only for the tested configurations:

- observed fixed-pair angle margins and modeled FIM eigenvalue margins;
- absolute localization-error distributions by squad-local depth and time;
- the absence or presence of temporal runaway over the tested horizon;
- the finite-depth growth profile across the seven-UAV squad;
- empirical coefficient-3 epsilon containment and \(q\) calibration; and
- estimator availability and failure modes.

### Claims out of scope

The work shall not claim:

- that \(P_i=\Phi_i^{-1}\) upper-bounds the true estimator covariance;
- that \(3\sqrt{\lambda_{\max}(P_i)}\) is a deterministic true-error bound;
- depth-uniform stability for arbitrarily large localization DAGs;
- independence of reference errors that share upstream ancestors;
- elimination of two-range mirror ambiguity;
- an unconditional end-to-end controller safety guarantee;
- estimator-in-the-loop vehicle or mission performance; or
- trajectory, geometry, or noise-distribution generality beyond the
  registered experiments.

## Graph and geometry contract

For each search UAV \(i\), let \(j(i)\) and \(k(i)\) be the two predeclared
fixed references used by the distance-maintenance CBF.
These references are always included in the localization FIM.
Additional visible bases and strictly lower squad-local-index UAVs may be
included dynamically.

Let
\(\boldsymbol p_i^\star,\boldsymbol p_{j(i)}^\star,
\boldsymbol p_{k(i)}^\star\)
be the target positions generated by the triangular-ladder construction.
Define the nominal pairwise distances

\[
a_i^\star=\lVert p_i^\star-p_{j(i)}^\star\rVert,\qquad
b_i^\star=\lVert p_i^\star-p_{k(i)}^\star\rVert,\qquad
c_i^\star=\lVert p_{j(i)}^\star-p_{k(i)}^\star\rVert .
\]

For every fixed-reference triple proposed as a nominal geometric support,
the target construction must be checked for strict nondegeneracy.
For its included angle \(\theta_i^\star\),

\[
\cos\theta_i^\star
=
\frac{(a_i^\star)^2+(b_i^\star)^2-(c_i^\star)^2}
{2a_i^\star b_i^\star},
\]

that target triple must imply

\[
|\cos\theta_i^\star|\leq \bar c_i^\star<1.
\]

If a root or transient target triple fails this check, it cannot support the
fixed-pair perturbation lemma.
The full active-set geometry theorem below remains applicable when dynamic
references provide the required directional rank.

The FIM uses estimated positions rather than nominal targets.
Let the estimated-position target deviations of the three vertices be no
greater than a finite \(\delta_i\).
The triangle inequality then places each estimated pairwise distance in its
nominal distance interval expanded by \(2\delta_i\).
If every triple in those expanded intervals still satisfies strict triangle
inequalities, the cosine law gives a computable
\(\bar c_i(\delta_i)<1\) for the estimated bearing angle.
This is a conditional perturbation lemma:
the current soft task-CBF objective does not itself provide a hard invariant
bound on \(\delta_i\).

The implementation plan shall extract the actual fixed-pair topology and
target geometry from the frozen configuration and controller code.
It shall compute the nominal angle margins and the largest admissible
target-tracking deviation for which each expanded distance interval remains
nondegenerate.
The exploratory replay shall then measure, rather than assume, whether the
estimated positions satisfy those margins.

The localization-connectivity CBF remains important because it preserves the
upper ranging-distance contract for the two assigned references.
However, the proof must not use that upper-distance constraint as a
collinearity barrier:
upper bounds on the two observer-to-reference distances alone do not bound the
reference-to-reference distance or triangle area away from zero.

For the complete active reference set, define the unweighted estimated
geometry matrix

\[
G_i
=
\sum_{j\in\mathcal R_i(t)}
g_{ij}g_{ij}^{\mathsf T}.
\]

The primary geometric condition is

\[
G_i\succeq\gamma_i I,\qquad \gamma_i>0.
\]

This condition allows dynamic references to supply missing directional
information when the fixed pair is nearly collinear.
The fixed-pair target-tracking lemma is retained only as one sufficient
mechanism for a positive \(\gamma_i\), not as an assumption that the current
soft controller satisfies at every frame.

## Modeled FIM nondegeneracy

Let

\[
g_{ij}
=
\frac{\hat p_i-\hat p_j}
{\lVert\hat p_i-\hat p_j\rVert}
\]

be the unit range direction used by the estimator and let

\[
v_{ij}
=
\sigma_r^2+g_{ij}^{\mathsf T}P_jg_{ij}
\]

be the implemented effective scalar variance.
For the active set, assume:

- all admitted directions are finite and defined;
- \(G_i\succeq\gamma_iI\) for a finite \(\gamma_i>0\); and
- all effective variances are finite and no greater than a finite
  \(\bar v_i\).

The active-set FIM then satisfies

\[
\begin{aligned}
\Phi_i
&=
\sum_{j\in\mathcal R_i(t)}
\frac{g_{ij}g_{ij}^{\mathsf T}}{v_{ij}},\\
\lambda_{\min}\!\left(\Phi_i\right)
&\geq
\frac{\gamma_i}{\bar v_i}>0.
\end{aligned}
\]

For a fixed pair, the angle condition remains a useful sufficient special
case.
With
\(\alpha_{ij}=v_{ij}^{-1}\) and
\(\alpha_{ik}=v_{ik}^{-1}\),

\[
\det\!\left(\Phi_i^{\mathrm{fixed}}\right)
=
\alpha_{ij}\alpha_{ik}
\left(1-(g_{ij}^{\mathsf T}g_{ik})^2\right)>0.
\]

Consequently, the implemented covariance is finite and obeys the
model-internal inequality

\[
\lambda_{\max}\!\left(P_i\right)
\leq
\frac{\bar v_i}{\gamma_i}.
\]

This is a statement about the modeled inverse FIM.
It is not an upper bound on the true localization error or true estimator
covariance.

For the dynamic information set,

\[
\Phi_i^{\mathrm{dynamic}}
=
\Phi_i^{\mathrm{fixed}}
+
\sum_{\ell\in\mathcal R_i^{\mathrm{extra}}}
\frac{g_{i\ell}g_{i\ell}^{\mathsf T}}{v_{i\ell}}
\succeq
\Phi_i^{\mathrm{fixed}}.
\]

Therefore dynamic references cannot reduce the modeled minimum FIM
eigenvalue when the fixed-pair terms and all weights are held unchanged.
This Loewner-order statement does not imply that correlated dynamic
references improve true-error calibration.
It also does not compare two complete replays in which upstream covariances,
WNLS solutions, weights, or bearing directions differ.

## Finite-DAG induction

Bases are roots with finite configured covariance.
Every UAV reference edge points from a strictly lower squad-local index to a
higher one.
Increasing squad-local index is therefore a topological order.

Assume, at one frame:

1. every root covariance and the range-noise variance are finite;
2. each UAV has its two required fixed-reference inputs available;
3. the estimator returns finite reference positions;
4. each complete active reference set satisfies
   \(G_i\succeq\gamma_iI\) for a finite \(\gamma_i>0\); and
5. the FIM calculation uses finite positive effective variances.

If every predecessor of UAV \(i\) has finite modeled covariance, then every
\(v_{ij}\) entering \(i\)'s active reference set is finite.
The geometric result makes \(\Phi_i\) positive definite, so \(P_i\) is finite.
Induction over the finite topological order establishes finite modeled
covariance for every UAV.

This result excludes singular geometric blow-up on the finite DAG.
It does not establish a contraction with depth.
Any statement uniform in arbitrary DAG depth would require an additional
recursive contraction condition and is outside this design.

As an optional finite-depth corollary, if every active geometry matrix has one
uniform \(\gamma>0\), every parent satisfies
\(P_j\preceq B_{d-1}I\), and every node at depth \(d\) uses the same
range-noise variance, define

\[
B_0=0,\qquad
B_d=\frac{\sigma_r^2+B_{d-1}}{\gamma}.
\]

Then the model-internal recursion gives

\[
P_i\preceq B_d I,\qquad
\epsilon_i\leq3\sqrt{B_d}
\]

for nodes at depth no greater than \(d\).
This bound is deliberately not described as a contraction:
it can grow rapidly with depth and only proves finiteness for a fixed finite
depth.

## Relation to the robust CBF result

The robust distance-CBF theorem shall retain the explicit premise

\[
\lVert p_i-\hat p_i\rVert\leq\epsilon_i.
\]

The implemented radius remains

\[
\epsilon_i=3\sqrt{\lambda_{\max}(P_i)}.
\]

The active-set geometry theorem supports the fact that this modeled radius
remains finite when its assumptions hold.
The nominal-target and tracking perturbation analysis explains one source of
that geometry, while dynamic references may supply additional noncollinear
directions.
Monte Carlo evidence measures how often the premise contains the actual
error.
The manuscript must not turn that measured coverage into a deterministic
premise.

The potential circularity shall be stated explicitly:
the controller uses a localization radius to robustify formation maintenance,
while maintained formation geometry supports a well-conditioned localization
model.
The theory therefore consists of two conditional implications plus empirical
closed-loop consistency evidence, not a single unconditional theorem.

## Existing-data exploratory analysis

The first analysis shall reuse the immutable Gate 2 paired replay bundle.
It shall not rerun the controller or modify WNLS, FIM, graph selection, or the
coefficient-3 radius.
The restart-policy dynamic-DAG arm is the primary descriptive population
because it contains the recovered deep-DAG estimates.
The strict arm is retained only to explain missingness and selection.
Time-varying target positions shall be read from the immutable input
trajectory's per-robot `cvt.center` records and aligned with replay rows by
frame and robot ID.
If a target record is missing or has different semantics for any robot class,
the target-tracking analysis must fail closed or exclude that class under an
explicit recorded rule rather than reconstruct an unobserved target.

The analyzer shall report:

1. nominal-target, true-position, and estimated-position fixed-pair
   bearing-angle distributions by depth and time;
2. target-tracking deviations and fixed-pair triangle-area or normalized-area
   margins as explanatory quantities;
3. the primary dynamic active-set geometry scores
   \(\lambda_{\min}(G_i)\) and
   \(\lambda_{\min}(G_i)/\lambda_{\max}(G_i)\), computed once from true
   trajectory geometry for every sampled tuple independently of estimator
   convergence and again from estimated geometry on finite estimator rows;
4. \(\lambda_{\min}(\Phi_i)\), condition number, and modeled radius by depth
   and time on valid estimator rows;
5. absolute error norm median, 90th, 95th, 99th percentile, and maximum by
   paired seed, depth, and time;
6. the finite-depth growth profile from squad-local depth one through seven;
7. post-acquisition error profiles as a function of frames since the first
   finite estimate;
8. coefficient-3 epsilon containment and \(q\) calibration, reported
   separately from absolute error;
9. availability, nonconvergence, invalid-upstream counts, and longest
   consecutive unavailable duration; and
10. compact diagnostics for shared-ancestor correlation and two-reference
   mirror ambiguity when these are needed to explain calibration failure.

The statistical unit is one paired range-noise seed.
Rows, UAVs, frames, and DAG edges are correlated repeated observations.
Seed-level summaries and paired-seed bootstrap intervals may describe
uncertainty, but no row-level independence or post-hoc \(p\)-value is allowed.

Because the Gate 2 trajectory and seeds have already been inspected, all
mechanism and stability analysis on this bundle is exploratory.
It may refine the theory and select confirmation metrics, but it cannot be
relabeled as a prospectively registered confirmation.

## Operational meaning of empirical stability

For the existing finite experiment, "error does not explode" shall mean only:

- no sustained temporal runaway is visible in depth-stratified absolute-error
  quantiles over the tested horizon;
- the depth-one through depth-seven absolute-error profile remains finite and
  interpretable without a terminal-depth discontinuity caused by FIM
  singularity or estimator loss;
- dynamic active-set geometry margins remain separated from zero on sampled
  trajectory tuples, modeled FIM eigenvalue margins remain separated from
  zero on valid estimator rows, and any fixed-pair degeneration or gap from
  nominal target geometry is reported;
- the maximum and upper-tail errors are reported in physical units rather
  than hidden by conditional coverage averages.

These are descriptive criteria, not mathematical boundedness.
The exploratory report shall avoid the bare word "bounded" unless it is
qualified as "within the tested finite horizon and squad depth."

## Prospective confirmation

After the exploratory report is frozen, a separate design shall register:

- new trajectories not used to choose the metrics;
- new range-noise seeds;
- the same offline estimator and dynamic localization graph;
- the same WNLS, FIM, and coefficient-3 radius;
- active-set geometry, absolute-error, availability, and calibration metrics;
- explicit operational thresholds chosen before inspecting confirmation
  results; and
- the existing disk limits and evidence-integrity requirements.

The confirmation set shall be invalidated if the estimator, graph, metrics,
or thresholds are changed after its results are inspected.
No new large simulation shall be launched until the exploratory analyzer and
theory audit determine the minimum confirmation matrix.

## Implementation boundary

The existing-data work shall add one focused analyzer and tests.
It must:

- stream the existing compressed rows and avoid duplicating raw evidence;
- reuse the existing bundle, row-order, pairing, hash, q, and disk-integrity
  validators;
- reconstruct fixed and dynamic active references from the frozen
  configuration and recorded trajectory rather than infer them from observed
  outcomes;
- fail closed on malformed geometry, missing required topology, non-finite
  values, row-order mismatch, or source-hash drift;
- write only compact JSON and Markdown outputs under 10 MB;
- preserve at least 6 GB of live free space and keep rebuildable cache usage
  within 2 GB;
- leave `build-diagnostic/` untracked and untouched; and
- make no paper edit until the analysis and independent review are complete.

The paper and DRA updates shall cite exact source commits, evidence paths, and
hashes.
The paper shall be updated only with claims supported by the final reviewed
analysis.

## Alternatives considered

### Deterministic true-error upper bound

Deriving a deterministic bound from the inverse FIM would require much
stronger bounded-noise, estimator-branch, model-consistency, and dependency
assumptions.
It would misrepresent the current method and is rejected.

### Full cross-covariance or set-membership estimator

Maintaining cross-covariances, covariance intersection, or deterministic
reachable error sets could support a more conservative estimator.
It would change the FIM model, the estimator scope, and the experiment matrix.
It is deferred unless the existing method fails the operational stability
criteria.

### Geometric nondegeneracy plus empirical stability

This option proves what the formation design directly controls, measures what
the FIM approximation cannot guarantee, preserves the distributed estimator,
and keeps localization as supporting machinery rather than the paper's main
contribution.
It is selected.

## Completion criteria

This design is complete when:

1. every fixed-reference target triangle, target-tracking quantity, and
   dynamic active-set rule, and connectivity constraint relevant to the
   geometry margin is identified from code and configuration without treating
   the upper range CBF as a collinearity barrier;
2. the geometric nondegeneracy and finite-DAG proofs pass an independent
   mathematical review;
3. the exploratory analyzer passes adversarial and regression tests;
4. the immutable evidence hashes remain unchanged;
5. the compact report separates geometry, modeled covariance, absolute error,
   calibration, and availability;
6. an independent evidence review has no open Critical or Important issue;
7. the DRA record is updated on its isolated `main` worktree; and
8. paper changes, if justified, preserve all claim boundaries in this design.
