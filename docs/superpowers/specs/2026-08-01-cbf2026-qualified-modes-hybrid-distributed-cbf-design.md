# CBF2026 Qualified Modes and Hybrid Distributed CBF Design

Date: 2026-08-01

Status: conceptual design approved by the researcher; written specification
awaiting researcher review

## Purpose

The registered two-range-reacquisition v2 replay terminated on its first row.
The failure was not an evidence-integrity failure and must not be retried.
It exposed two distinct problems:

1. the row validator counted accepted solver records while the estimator
   selected among them, so the validator and publisher had incompatible
   contracts; and
2. the first sUAV in each squad had two physically distinct, locally valid
   mirror solutions about the collinear bUAV line, and the publisher could
   select one by cost or source order even though the ranges did not identify
   a unique global mode.

The second problem is a scientific gap, not merely a validator bug.
A positive-definite local FIM proves local differential observability around a
candidate. It does not prove that the range map is globally injective.

The current paper also contains three independent theory-to-implementation
gaps:

- active-reference changes can reset the estimate and FIM-derived radius, but
  the current invariance proposition assumes local absolute continuity;
- the implemented local QPs use neighbor commands and do not instantiate the
  ideal stacked CBF rows; and
- the backward difference of the FIM-derived radius is not a certified upper
  bound on its future flow derivative.

This design closes those interfaces without changing the scientific identity
of the method. It preserves the dynamic lower-index localization DAG, the two
fixed distance-maintenance references, the original FIM recursion, the
coefficient-three radius, planar velocity commands, and decentralized local
QP execution.

## Authoritative baselines

The design is based on the following immutable or current baselines:

- source/diagnostic worktree:
  /private/tmp/cbf2026-diagnostic;
- source branch at design start: codex/cbf2026-diagnostic;
- source HEAD at design start:
  0f0619128850de6c377b23fd14d9119b77770740;
- latest paper worktree:
  /private/tmp/cbf2026-paper-geometric-stability;
- latest paper branch:
  codex/cbf2026-geometric-stability-paper;
- latest paper HEAD at design start:
  d6a95736d2939728d9418ae02deadedeca822348;
- consumed registered v2 root, retained read-only:
  /private/tmp/cbf2026-two-range-reacquisition-development/v2; and
- absent v2 analyzer root, which must remain absent:
  /private/tmp/cbf2026-two-range-reacquisition-analysis/v2.

No future development, registration, replay, or analysis may write into or
reinterpret either v2 path.

## Scope and non-goals

The design has four coupled scopes:

1. an offline mode-qualified WNLS sidecar;
2. a flow/reset uncertainty-radius contract for the controller-facing dynamic
   FIM;
3. allocated pairwise hard-CBF rows solved by independent local QPs; and
4. new development and confirmatory Monte Carlo evidence followed by paper
   revision.

The WNLS sidecar remains outside the controller. It must not change the UAV
truth trajectory, controller state, applied command, CVT target, active CBF
edge set, or simulated vehicle dynamics. Its purpose is to evaluate whether a
simple range estimator supports the modeled localization claims. The final
paper must not describe the sidecar as an estimator-in-loop validation.

The controller continues to use the planar velocity-command abstraction. The
design does not model attitude, acceleration, motor dynamics, command
tracking, or any layer below planar velocity. It does not introduce a
sampled-data reserve. Reference and radius transitions are treated as hybrid
resets, not as sampling error.

The design preserves:

- two fixed CBF references per sUAV;
- dynamic optional FIM-only bUAVs and strictly lower-local-index sUAVs;
- a bases-first, increasing-local-index topological order;
- the original diagonal marginal-covariance FIM;
- the effective variance
  \(\sigma_r^2+n_{ij}^{\mathsf T}P_jn_{ij}\);
- \(\epsilon_i=3\sqrt{\lambda_{\max}(P_i)}\);
- exact knowledge of the preceding applied command and zero-order hold for
  private estimator-state propagation;
- a maximum public prediction age of two frames;
- componentwise planar input bounds
  \(|u_{ix}|,|u_{iy}|\le 25\); and
- hard localization/collision rows and soft task rows.

The design does not:

- replace the FIM with the estimator's Gauss--Newton matrix;
- add a private history prior to the FIM;
- claim that the inverse FIM is the actual covariance or a deterministic error
  upper bound;
- claim that a finite DAG alone provides global range identifiability;
- use truth, future frames, analyzer labels, or realized error in any runtime
  estimator or controller gate;
- compare multiple estimator families;
- make dynamic FIM-only references into CBF edges;
- reuse the consumed v2 namespace;
- tune a confirmatory threshold after confirmatory outputs are visible; or
- claim mission-wide invariance when a required envelope, flow certificate,
  reset guard, or hard-QP feasibility premise fails.

## Alternatives considered

### A. Qualified modes plus allocated pairwise CBFs

This is the selected design.

Solver records are clustered by their final estimates before publication.
Cold-start modes are qualified by a predeclared deployment domain on the ocean
side of the bUAV line. Later modes are qualified by the private
ZOH-propagated history. Exactly one admissible mode is required for a fresh
publication. The history prior selects a discrete branch only and remains
outside the continuous WNLS objective and FIM.

For controller realization, every coupled localization or collision row is
split into complementary endpoint rows. Every UAV still solves only for its
own three-dimensional command. Summing the two endpoint rows recovers the
paper's full stacked row exactly.

### B. Ocean-side filtering alone

The ocean-side half-plane removes the base-bootstrap reflection in the studied
geometry. It does not resolve a later two-sUAV-reference reflection when both
branches lie inside the search side. It is insufficient by itself.

### C. Noncollinear bUAVs or a mandatory third current reference

This changes the deployment geometry or communication requirement and still
does not remove every downstream two-reference branch flip. It would also
discard the scientifically relevant two-reference distributed structure.

### D. Retain the ideal stacked theorem only

This is mathematically honest if the paper labels the local controller as a
heuristic. It leaves the main distributed-realization contribution
uninstantiated and is therefore not selected for the target submission.

## Five-layer claim architecture

The revised theory and paper must keep the following layers separate.

1. Graph well-foundedness. The lower-index rule makes every active
   localization graph a DAG and permits a finite topological FIM recursion.
2. Global-mode qualification. Deployment-domain or temporal-history
   information determines whether one discovered global mode may be
   published.
3. Local modeled information. Conditional on the selected mode, the FIM
   describes local geometry and a modeled inverse-information matrix.
4. Envelope calibration. Whether the coefficient-three radius contains
   realized error is an empirical/conditional statement, not a consequence of
   the CRLB.
5. Hybrid distributed CBF invariance. Conditional on a valid error envelope,
   certified flow rate, guarded resets, bounded inputs, and feasible local hard
   rows, the complementary endpoint constraints imply the coupled robust
   constraints.

No theorem, algorithm caption, table heading, abstract claim, or conclusion
may collapse two of these layers into one implication.

## Estimator mode contract

### Definitions

An eligible record is a complete solver result that passes the existing
finite, convergence, stationarity, residual, local-FIM/SPD,
current-reference, provenance, and local innovation/cost checks. Eligibility
is necessary but does not authorize publication.

A numerical mode is a cluster of eligible records whose final estimates are
numerically indistinguishable under the frozen clustering rule below.

An admissible mode is a numerical mode that passes the applicable
deployment-domain or temporal-history qualifier.

A representative is the deterministic record chosen inside one already
selected admissible mode. Cost, innovation, and source order may select a
representative inside a mode. They must never select between distinct modes.

Fresh is permitted if and only if exactly one admissible mode exists and its
representative passes all current-frame continuous-update checks. Zero or
multiple admissible modes fail closed.

### Post-solver clustering

The primary clustering threshold is frozen at

\[
\tau_{\mathrm{mode}}=10^{-3}\ \mathrm{m}.
\]

This is a development-derived numerical threshold. Preserved pre-design data
showed same-basin accepted estimates separated by at most approximately
\(4.45\times10^{-4}\,\mathrm m\), while true mirror modes were separated by
at least approximately \(115\,\mathrm m\). Those data may justify the new
prospective threshold but may not be counted as confirmatory evidence.

Create an undirected graph whose vertices are eligible records and whose edge
condition is final-estimate distance at most \(\tau_{\mathrm{mode}}\). Its
connected components are provisional clusters. A provisional cluster is
valid only if its complete diameter is also at most
\(\tau_{\mathrm{mode}}\). A component connected only by a chaining sequence
but with larger diameter is nonseparable_chain and cannot be published.

This rule is deterministic, source-label invariant, and input-order
invariant. Sensitivity outputs for 0.5, 1, and 2 mm are mandatory, but only
the 1 mm result controls the primary decision.

### Candidate completeness

For exactly two current range references, the implementation must enumerate
both analytic circle-intersection branches when they exist and independently
refine them with WNLS. Property tests must cover tangent, disjoint, contained,
coincident-reference, noisy, and two-distinct-root cases. The main
two-reference global-mode statement may rely on this analytic enumeration.

For three or more references, finite multistart WNLS is not claimed to
enumerate every mathematical local minimum. Any generic statement is
conditional on candidate-set completeness. The implementation must include
the algebraic seed, every applicable pairwise-circle seed, the live/private
seed when valid, deterministic deduplication, and complete raw solver records.
The paper must not equate one discovered mode with a proof that the objective
has one global mode.

## Cold-start global identifiability

Let two distinct known bUAV positions \(a_1,a_2\) define a line \(L\), and
define the oriented signed distance

\[
s_L(p)=
\sigma\frac{\det(a_2-a_1,p-a_1)}{\|a_2-a_1\|},
\qquad \sigma\in\{-1,1\}.
\]

For every bootstrapped sUAV, the mission configuration must declare an
admissible deployment set \(D_i^0\), the orientation \(\sigma\), and a
positive margin \(\delta_0\) satisfying

\[
D_i^0\subseteq\{p:s_L(p)\ge\delta_0\}.
\]

For exact ranges to two distinct known anchors, the two-circle solutions are
either empty, tangent, or a reflection pair about \(L\). Consequently, the
two-range map is injective on \(D_i^0\). This proposition establishes a
non-circular cold-start base case. It does not prove finite-noise WNLS
correctness.

The deployment set, anchor coordinates, orientation, and margin must be frozen
before registered seeds are produced. The runtime gate reads only these
configuration values and candidate estimates. Truth is used only by the
analyzer to verify the declared initial-domain assumption.

If the anchors are not distinct, the deployment set crosses the anchor line,
the orientation cannot be derived canonically, or zero/multiple modes pass the
domain gate, cold-start publication fails closed.

## Temporal mode qualification

After a valid cold start, the private state is propagated by

\[
\bar x_{k+1}^{-}=\bar x_k+\Delta t\,u_k,
\qquad
P_{k+1}^{-}=P_k+Q,
\]

using the command actually applied and held over the preceding interval. The
existing declared \(Q=0.25I\) remains a modeled development reserve. It is
not treated as a proved process-error bound.

For every discovered mode \(m\) with representative \((x_m,P_m)\), compute

\[
q_m=(x_m-\bar x^-)^{\mathsf T}
(P_m+P^-)^{-1}(x_m-\bar x^-).
\]

The existing threshold

\[
q_\star=11.829007011943707
\]

and its inclusive boundary remain frozen. A temporal update is branch
qualified if and only if exactly one numerical mode satisfies
\(q_m\le q_\star\). Zero or multiple passing modes fail closed.

This is a deterministic modeled-consistency gate. Because the diagonal model
omits shared-ancestor and prior/reference correlations, \(q_m\) is not
described as an exact chi-square test or a true-error probability guarantee.

The private prior:

- may be created only by an accepted fresh state;
- may be propagated only by logged applied held commands;
- may remain available as unpublished branch-history state after the public
  two-frame prediction horizon expires;
- has an age no greater than the finite registered mission horizon minus one,
  with every propagation and age serialized;
- is not a WNLS range, anchor, continuous residual, or FIM term;
- is not reset by predicted, unavailable, ambiguous, or rejected output; and
- must become absent when its source, command chronology, covariance, or age
  is invalid.

The temporal correctness result is explicitly inductive and conditional:

1. the cold-start mode is correct;
2. the previous fresh mode is correct;
3. the true current mode satisfies the declared prediction-consistency
   condition;
4. the true mode passes the gate and every false mode fails it; and
5. the candidate set contains the relevant true and false modes.

Under those premises, unique-mode publication preserves the correct branch.
The FIM does not prove any premise in this list. An incorrect overconfident
prior that remains self-consistent with a false branch is a required negative
test and Monte Carlo failure mode.

## Publication and lifecycle semantics

The public statuses remain fresh, predicted, and unavailable.

- Fresh means a current-frame WNLS update from current-valid references plus
  unique global-mode qualification.
- Predicted means the last fresh state propagated by actual held commands,
  with age no greater than two.
- Unavailable means neither a qualified current update nor an in-age public
  prediction exists.

A rejection preserves an already propagated valid private state byte for byte.
It never promotes a rejected candidate to private fresh state. After public
prediction expires, the private history may remain available only as the
disclosed branch qualifier described above. It cannot itself become a public
estimate, and it cannot persist beyond the finite registered mission horizon.

The raw schema must serialize every eligible record, numerical cluster,
cluster membership, diameter, qualifier score, qualifier decision,
representative, prior source and age, output status, and next private state.
The analyzer must recompute clustering, scores, state transitions, and
publication independently from raw fields.

## Dynamic FIM and flow-rate certificate

The dynamic active set remains

\[
\mathcal R_i(t)=
\mathcal R_i^{\mathrm{CBF}}
\cup\mathcal B_i^{\mathrm{visible}}(t)
\cup\mathcal A_i^{\mathrm{visible}}(t),
\]

where optional sUAV references have strictly lower squad-local index. Dynamic
FIM references never create a CBF distance edge.

On an open flow interval with a fixed active set, compute the analytic FIM
derivative from the current estimated directions, velocity-command interface,
and topologically preceding modeled-matrix derivatives. The
controller-facing certificate is

\[
\nu_i^{\mathrm{ub}}(t)=
\frac{3\|P_i\|_2^2\|\dot\Phi_i\|_2}
{2\sqrt{\lambda_{\max}(P_i)}}.
\]

The calculation proceeds in the same bases-first, increasing-local-index order
as the FIM. Every input must be finite, every active distance must exceed the
declared singular-distance tolerance, and every active FIM must pass the
declared SPD/conditioning gate. The analyzer independently recomputes
\(\dot\Phi_i\), \(\nu_i^{\mathrm{ub}}\), and
\(D^+\epsilon_i\le\nu_i^{\mathrm{ub}}\) on flow intervals.

The positive backward difference may remain as a descriptive log field. It
must not be supplied to a theorem-aligned hard CBF in place of the analytic
certificate.

If the analytic rate certificate is invalid, the affected controller interval
does not satisfy the invariance premises and must not be counted as a
theorem-aligned controlled interval.

On every theorem-aligned flow interval, the controller-facing nominal estimate
must satisfy

\[
\dot{\hat p}_i=u_i.
\]

It may be reset only through the hybrid guard below. The offline WNLS sidecar
does not supply this state. If a future implementation instead uses an
estimator with a nonzero flow residual
\(\dot{\hat p}_i-u_i\), that residual requires a separately derived bound and
a new design version; it may not be silently absorbed into
\(\nu_i^{\mathrm{ub}}\).

## Allocated pairwise distributed CBFs

Use \(a_{ij}^i\) and \(a_{ij}^j\) for responsibility allocation; do not use
\(\eta\), which already denotes ranging noise in the paper. For every
controlled pair,

\[
a_{ij}^i\ge0,\qquad a_{ij}^j\ge0,\qquad
a_{ij}^i+a_{ij}^j=1.
\]

The primary allocation is frozen as follows:

- sUAV--sUAV fixed localization edge:
  \(a_{ij}^i=a_{ij}^j=1/2\);
- sUAV--sUAV collision pair:
  \(a_{ij}^i=a_{ij}^j=1/2\); and
- sUAV--hovering-bUAV localization edge: the sUAV has allocation one and the
  zero-command, zero-radius bUAV row is omitted.

Both endpoints must use the same canonical unordered edge identifier,
oriented normal, state/radius snapshot version, class-K parameters, and
allocation. A mismatch prevents row construction.

For

\[
n_{ij}=\frac{\hat p_i-\hat p_j}
{\|\hat p_i-\hat p_j\|},
\]

the localization barrier is

\[
b_{\mathrm{loc}}^{ij}=
d_{\mathrm{loc}}-\|\hat p_i-\hat p_j\|-\epsilon_i-\epsilon_j.
\]

The two endpoint rows are

\[
-n_{ij}^{\mathsf T}u_i-\nu_i^{\mathrm{ub}}
+a_{ij}^i\alpha(b_{\mathrm{loc}}^{ij})\ge0,
\]

\[
+n_{ij}^{\mathsf T}u_j-\nu_j^{\mathrm{ub}}
+a_{ij}^j\alpha(b_{\mathrm{loc}}^{ij})\ge0.
\]

Their sum is exactly the coupled localization row.

For

\[
b_{\mathrm{col}}^{ij}=
\|\hat p_i-\hat p_j\|-d_{\mathrm{safe}}-\epsilon_i-\epsilon_j,
\]

the endpoint rows are

\[
+n_{ij}^{\mathsf T}u_i-\nu_i^{\mathrm{ub}}
+a_{ij}^i\alpha(b_{\mathrm{col}}^{ij})\ge0,
\]

\[
-n_{ij}^{\mathsf T}u_j-\nu_j^{\mathrm{ub}}
+a_{ij}^j\alpha(b_{\mathrm{col}}^{ij})\ge0.
\]

Their sum is exactly the coupled collision row.

The full fixed localization edge registry must therefore provide reverse
incidence information to references. Every sUAV--sUAV fixed edge has exactly
two complementary local rows. Every sUAV--bUAV edge has exactly one sUAV row.
Dynamic FIM-only edges have no CBF rows.

Collision mode must use every declared unordered collision pair, not a
per-agent minimum-only pair. Each unordered pair has exactly two
complementary local rows.

The old single-end full row with a neighbor command in its temporal derivative
must be replaced, not retained in parallel with the allocated row.

### Distributed-realization proposition

Suppose every endpoint uses the same pair snapshot and allocation, every local
hard QP is feasible in the explicit componentwise input set, and every
endpoint applies a command satisfying all of its local allocated rows. Summing
the two endpoint inequalities for each sUAV pair, or using the single sUAV row
for a hovering bUAV pair, gives the complete coupled CBF row. The existing
stacked conditional invariance argument then applies on the flow interval.

This is a distributed sufficient realization. It is more conservative than a
centralized coupled row. Centralized feasibility does not imply that every
fixed-allocation local QP is feasible. Local infeasibility, deadlock, and task
noncompletion are therefore mandatory experimental outcomes.

## Hybrid reset guard

An active-reference change, estimate-interface reset, or discontinuous change
in \(P_i\), \(\epsilon_i\), or the hard-edge snapshot is a hybrid reset for
the controller-facing certificate. Continuous evolution of these quantities
with a fixed active set remains part of the flow. The physical position
remains continuous at a reset, but the tightened barrier may jump.

For every proposed reset at time \(t_s\), recompute with post-reset values:

\[
b_e^+=b_e(\hat p^+,\epsilon^+)
\quad\text{for every hard edge }e,
\]

and solve/check all post-reset local hard QPs in the explicit input set. The
reset is admissible only if

\[
b_e^+\ge0\quad\forall e
\]

and all required post-reset local hard QPs are feasible.

Every reset record must include
\(\Delta\hat p\), \(\Delta\epsilon\), pre/post active sets,
\(b_e^-\), \(b_e^+\), allocation versions, QP feasibility, and the guard
decision.

If a proposed new FIM set fails the guard while the preceding set remains
fully valid, the preceding set and certificate may remain active. If the
preceding set is no longer valid, the theorem-aligned controlled interval ends
and the mission enters an explicit unavailable/abort state. The implementation
must not silently keep stale fresh data or claim invariant continuation.

The offline WNLS sidecar does not enter this reset guard because it does not
enter the controller. Its own mode transitions are analyzed separately.

### Hybrid invariance proposition

Assume:

1. all initial tightened barriers are nonnegative;
2. on every flow interval, the valid error-envelope, analytic radius-rate,
   allocated-row, input-bound, and local-feasibility premises hold; and
3. every accepted reset maps the pre-reset state into the post-reset tightened
   set and post-reset feasible input domain.

The flow comparison argument preserves each tightened set between resets, and
the reset guard preserves membership at every reset. Induction over the finite
reset sequence therefore gives hybrid conditional forward invariance.

The conclusion remains conditional on
\(\|p_i-\hat p_i\|\le\epsilon_i\). Neither the FIM nor the reset guard proves
that premise.

## Controller behavior when evidence is unavailable

Estimator unavailable in the offline sidecar has no effect on control.

Controller certificate unavailable means a required FIM/rate/edge snapshot or
reset premise is invalid. The implementation must log the exact reason and
terminate the theorem-aligned mission segment. A coordinated zero-command
fallback may be studied as operational behavior, but it is not described as
guaranteed safe unless it independently satisfies all active hard rows.

No unavailable interval may be silently removed from the scientific
denominator.

## Implementation architecture

The source implementation should be separated into testable contracts:

1. candidate local eligibility;
2. post-solver mode clustering;
3. deployment/history mode qualification;
4. representative selection within one mode;
5. publication/private-state transition;
6. topological FIM and analytic rate certificate;
7. canonical fixed-edge and collision-pair registry;
8. allocated endpoint-row construction;
9. hybrid reset guard; and
10. raw evidence serialization and independent validation.

The implementation must not use one Boolean named accepted to conflate local
solver eligibility, mode admissibility, and publication authorization.

## Required unit and property tests

The implementation plan must include, at minimum:

### Modes and publication

- multiple starts converging within 1 mm form one valid mode;
- the observed approximately 119.796 m mirror pair forms two modes;
- 0.5, 1, and 2 mm sensitivity outputs do not control the primary decision
  except at 1 mm;
- connected-component chaining with diameter above 1 mm fails closed;
- clustering is invariant to record order and source-label permutation;
- two-circle enumeration returns both distinct roots;
- no domain/history prior with multiple modes fails closed;
- the declared deployment half-plane selects only the ocean-side mode;
- invalid or cross-line deployment domains fail closed;
- exactly one history-qualified mode publishes fresh;
- zero or multiple qualified modes do not publish fresh;
- rejected candidates do not reset private state;
- truth, future-frame, and analyzer-derived fields cannot enter runtime gates;
  and
- an incorrect self-consistent private prior is retained as a negative case,
  not relabeled as proof of correctness.

### FIM and hybrid rate

- bases-first topological derivative propagation matches finite perturbation on
  fixed-active-set fixtures;
- the analytic \(\nu_i^{\mathrm{ub}}\) independently upper-bounds the observed
  flow derivative within numerical tolerance;
- backward difference is never routed into a theorem-aligned hard row;
- active-set additions and removals create explicit reset records;
- a post-reset negative barrier is rejected;
- a post-reset infeasible local QP is rejected; and
- every accepted reset has nonnegative post-reset barriers and feasible local
  hard QPs.

### Allocated CBF rows

- complementary localization coefficients and constants sum to the full
  coupled row;
- complementary collision coefficients and constants sum to the full coupled
  row;
- a bUAV edge produces the correct single sUAV full-allocation row;
- every fixed sUAV--sUAV edge has exactly two complementary rows;
- every collision pair has exactly two complementary rows;
- every dynamic FIM-only edge has zero CBF rows;
- allocation values are finite, nonnegative, and sum to one;
- edge, snapshot, or allocation-version disagreement rejects construction;
- allocated rows do not change when a logged neighbor command is perturbed;
- all implemented local hard-row residuals and independently reconstructed
  full-row residuals agree; and
- a stress fixture demonstrates that a centralized coupled row can be feasible
  while a fixed half-allocation local QP is infeasible, so conservatism is not
  hidden.

## Development experiment

Development uses a new namespace, never a v2 descendant. The implementation
plan must allocate paths under a name such as

/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v1.

The development campaign uses at least ten mission trajectories with seeds
disjoint from all confirmatory seeds. It may inspect truth only in analyzers
and may diagnose structural failure. It may change code and thresholds only
through a new documented development round. Every change must retain the
failed run and previous development evidence.

The development gates are:

- complete registered-key and manifest accounting;
- zero truth reads by runtime decision code;
- zero cross-mode source-order publications;
- zero fresh publications with multiple admissible modes;
- zero wrong-mode fresh publications in development evidence;
- no errors above the retained 50 m catastrophic-error threshold;
- aggregate valid-output containment at least 0.98;
- minimum squad-local-depth containment at least 0.95;
- fresh retention at least 0.98;
- fresh-or-bounded-predicted availability at least 0.95;
- zero missing or duplicated allocated hard rows;
- zero analytic-rate-certificate violations on admitted flow intervals;
- zero accepted reset-guard violations;
- zero componentwise input-limit violations;
- zero local hard-QP infeasibility in the primary allocation arm;
- zero negative implemented local or reconstructed full hard-row residuals,
  within a frozen numerical tolerance; and
- at least 0.95 mission completion without a persistent safe-but-stopped
  deadlock.

Failure of a development gate permits diagnosis and a new versioned design
round. It does not permit relabeling a failed arm as confirmatory evidence.

## Confirmatory Monte Carlo design

After all development gates pass, freeze a new protocol, code identity,
configuration, seed list, thresholds, schemas, analyzer, registrar, and output
namespace before execution. An independent review must verify the hashes and
that no output path exists.

The primary confirmatory population contains 60 independent mission seeds.
Each seed generates its own mission trajectory and measurement-noise
realization. Repeating range noise on a single preserved truth trajectory does
not satisfy this requirement.

The dynamic DAG is the primary controller and estimator-analysis condition. A
fixed-reference FIM replay may be retained as a paired reference-graph
ablation on the same generated truth trajectories and range measurements. It
is not an estimator comparison and must not alter the primary controller
trajectory.

The confirmatory adequacy gates retain the development thresholds above. In
addition:

- wrong-mode fresh publications must be zero;
- mission incidence of any wrong-mode fresh publication must be zero and its
  one-sided 95% upper confidence bound must be reported;
- fresh publication with multiple admissible modes must be zero;
- deployment-domain assumption violations must be zero;
- accepted reset with a negative post-reset barrier or infeasible post-reset
  QP must be zero;
- controller-certificate unavailable and mission-abort outcomes remain in the
  denominator;
- true localization-distance and collision-distance violations are reported
  separately from estimated tightened-barrier violations;
- maximum, p95, p99, and depth-stratified localization error are reported;
- per-mission and mission-level confidence intervals are reported without
  treating frames or UAV rows as independent missions; and
- all estimator failures, abstentions, expired priors, invalid certificates,
  guard rejections, QP failures, deadlocks, and incomplete missions are
  retained in terminal manifests.

The old 44.7438% containment, 14.6443% minimum-depth containment, and
999.332 m retained-error tail remain development-history evidence. They may be
removed from the main results only after the revised method passes a new
prospective campaign; the diagnostic and DRA records remain immutable.

## Evidence and integrity requirements

Every development and confirmatory campaign must provide:

- a specification and implementation-plan commit;
- source and dependency identities;
- frozen configuration and seed hashes;
- a preflight manifest proving the output root is absent;
- an authorization artifact bound to the exact execution identity;
- terminal replay state and process return code;
- a complete raw-row manifest;
- an independently recomputed analyzer result;
- an author report;
- an independent adversarial review; and
- a DRA checkpoint on main containing specifications, commands, hashes,
  results, failures, and next actions.

The standing researcher authorization permits development and, after the
freeze and independent preregistration gates pass, execution of the newly
bound registered campaign without another conversational approval. It does
not authorize reuse, overwrite, retry, or mutation of an already consumed
registered root.

## Disk policy

Raw outputs must be streamed and compressed. Temporary decompressed copies are
prohibited unless bounded and deleted after verification. Reusable cache is
capped at 2 GB. The runner must monitor free space and stop cleanly before free
space falls below 6 GB. Development artifacts without evidentiary value must
not accumulate across versions.

## Paper revision gates

Paper theory edits may begin after the design and implementation plan are
approved and the relevant unit or property proofs pass. Numerical claims and
tables may change only after the confirmatory evidence is terminal and
independently reviewed.

The revised paper must:

1. define the estimator as a separate offline sidecar and remove wording that
   treats LIO, WNLS, modeled FIM covariance, and the controller interface as
   interchangeable;
2. replace valid cooperative localization with separate graph,
   local-geometry, and global-mode conditions;
3. state the cold-start half-plane injectivity proposition and its exact
   deployment assumptions;
4. state temporal mode qualification as a conditional induction, not an FIM
   guarantee;
5. condition every FIM-radius statement on the selected global mode;
6. retain the distinction between modeled inverse FIM and actual error;
7. add the allocated pairwise distributed-realization proposition;
8. extend interval-wise invariance to the guarded hybrid reset proposition;
9. replace the backward-difference controller claim with the analytic flow
   certificate or explicitly downgrade any interval that lacks it;
10. state componentwise planar input bounds and hard-QP feasibility premises;
11. describe dynamic FIM-only references and fixed CBF references without
    conflating them;
12. report branch correctness, availability, containment, reset validity,
    local/full hard-row residuals, and true constraint outcomes separately;
13. preserve the limitation that shared-ancestor cross-covariances are
    omitted;
14. avoid mission-wide probabilistic safety language; and
15. revise the abstract, contributions, comparison table, conclusion, and
    submission-readiness statements to match only the verified evidence.

The final submission PDF must compile without errors, have no unresolved
references or citations, contain no internal diagnostic language or stale
failed-threshold text, and pass independent theory, methods, evidence, and
journal-fit reviews.

## DRA integration

The doctoral-research-agent repository remains on main because it tracks
multiple papers. Each material milestone must update the CBF2026 record with:

- source, paper, and DRA commit identities;
- the approved design and implementation-plan paths;
- exact development and registered commands;
- threshold and seed provenance;
- disk and output locations;
- terminal manifests and hashes;
- failed and passed gates without selective omission;
- paper edits and compiled-PDF identity; and
- remaining submission blockers.

## Completion criteria

This design is complete only when all of the following are true:

1. the written design and implementation plan are reviewed and committed;
2. mode clustering, cold-start qualification, temporal qualification, and
   lifecycle semantics pass their unit or property tests;
3. analytic FIM-rate certificates and guarded resets pass independent
   recomputation;
4. allocated localization and collision rows are complete and reconstruct
   every full coupled row;
5. the bounded-input local hard QPs satisfy all primary development gates;
6. a new independently reviewed registered campaign reaches a terminal state;
7. the scientific adequacy gates pass without post-registration changes;
8. the paper is revised to match the resulting evidence;
9. the final PDF passes build, citation, theory, method, evidence, and
   presentation audits; and
10. the DRA record is current and independently traceable.

Passing only the estimator gates, only the controller tests, or only the paper
build is not completion of the research objective.
