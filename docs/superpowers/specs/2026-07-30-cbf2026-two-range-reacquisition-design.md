# CBF2026 Two-Range Reacquisition Design

Date: 2026-07-30

Status: independent theory/evidence review passed; awaiting final user review

Independent review:
`docs/superpowers/specs/reviews/2026-07-30-cbf2026-two-range-reacquisition-design-review.md`

## Purpose

Stage 1 v4 removed catastrophic published errors on the frozen replay but
passed only seven of nine estimator gates.
Its complete arm published `119655` fresh,
`1971` predicted,
and `18374` unavailable outputs.
Fresh retention was
`107460 / 127190 = 0.844877741960846`,
below `0.98`,
and fresh-or-predicted availability was
`121626 / 140000 = 0.8687571428571429`,
below `0.95`.

A read-only causal trace shows that `18148 / 18374 = 98.77%` of unavailable
outputs are attributed through lineage to reacquisition rejection,
whereas only `226 / 18374 = 1.23%` are rooted in predicted-anchor
ineligibility.
Every one of the `7679` row-level rejected-to-unavailable lineage terminals
has exactly two
active current-fresh UAV references,
no optional reference,
and three numerically converged candidates.
All `23037` candidates are rejected solely by
`reacquisition_requires_three_active_references`.
These are not `7679` independent episode onsets;
the replay contains `109` outage episodes.

This design therefore changes only the observed
exactly-two-current-fresh-UAV-reference reacquisition decision.
It does not change reference freshness,
the dynamic lower-index DAG,
the two mandatory CBF references,
the WNLS objective,
the FIM formula,
the public prediction horizon,
the innovation threshold,
or any Stage 1 scientific threshold.

## Scope and non-goals

The design is an offline estimator development round.
It remains outside the controller and cannot change the CBF,
CVT,
vehicle model,
communication model,
truth trajectory,
applied command,
range-noise realization,
or fixed/dynamic reference definitions.

The design preserves:

- the planar velocity-command model;
- exact communication of the preceding applied command and zero-order hold;
- two fixed CBF references always included among the mandatory FIM terms
  whenever their current measurements and reference states are valid;
- dynamic optional bases and strictly lower-index UAV references;
- current-fresh UAV anchors only;
- the original diagonal marginal-covariance FIM;
- `MAX_PUBLIC_PREDICTION_AGE = 2`;
- `INNOVATION_REFERENCE_QUANTILE = 11.829007011943707`;
- the `50 m` catastrophic-error threshold;
- the `0.98` fresh-retention threshold;
- the `0.95` fresh-or-predicted availability threshold; and
- all DAG,
  provenance,
  covariance,
  publication,
  and fixed-cohort integrity checks.

This design does not:

- permit a predicted or unavailable UAV to become a WNLS anchor;
- relabel a predicted-anchor update as fresh;
- extend the public prediction horizon;
- treat the private prior as a third measurement;
- add the private prior to the FIM or reported fresh covariance;
- loosen innovation,
  residual,
  convergence,
  SPD,
  or provenance thresholds;
- use truth or offline error to select a branch;
- modify or reinterpret the completed Stage 1 v4 evidence;
- generate bounded-input Stage 2 trajectories;
- open the paper-edit gate; or
- establish a deterministic true-error bound for the FIM-derived radius.

## Alternatives considered

### A. Two fresh ranges with private-prior branch disambiguation

This is the selected design.
It keeps both mandatory anchors current-fresh and retains the current FIM.
The two globally ambiguous range-intersection branches are solved
independently from the canonical negative and positive circle starts.
Only after both continuous WNLS results exist is the private ZOH-propagated
state used to decide which branch is consistent with the robot's unpublished
state history.
On this new path,
the private state is never a WNLS start,
continuous-update input,
publishable representative,
or FIM term.
The final estimate and conditional covariance come from the selected
current two-range WNLS branch.

This option directly targets the lineage attribution of `98.77%` of
unavailable outputs,
preserves `fresh` as a current-frame measurement update while explicitly
disclosing the historical branch-identity decision,
and does not require predicted-anchor covariance or space-time provenance.

### B. Bounded-age predicted UAV anchors

This is theoretically possible under the declared velocity/ZOH model if
predicted means,
covariances,
ages,
and full ancestry are propagated.
It would require a new `updated` status,
a space-time DAG contract,
and explicit disclosure that shared-ancestor cross-covariances remain
omitted.
It cannot honestly count as current `fresh` and therefore does not directly
repair the failed fresh-retention gate.

The causal audit also shows that predicted-anchor ineligibility roots only
`1.23%` of current unavailable outputs.
It is excluded from this development round.

### C. A longer public prediction horizon

There are only 109 outage onsets.
Sixty-three episodes exceed 100 frames and contribute
`17492 / 18374 = 95.20%` of unavailable rows.
A modest horizon increase would mainly delay most of these long outages,
could not remove the three-reference reacquisition mechanism,
and would not directly repair fresh retention.
It would also change the frozen age gate without addressing the rejected
reacquisition mechanism.

The horizon remains two frames.

## Mathematical contract

Let the two current-fresh mandatory UAV references have positions
\(a_1,a_2\),
current noisy ranges
\(\rho_1,\rho_2\),
unit range directions at a candidate \(x\),
\(n_1(x),n_2(x)\),
and positive directional variances
\(s_1^2,s_2^2\).
The existing local information matrix is

\[
\Phi(x)
=
\frac{n_1n_1^{\mathsf T}}{s_1^2}
+
\frac{n_2n_2^{\mathsf T}}{s_2^2}.
\]

For two non-collinear directions,

\[
\det\Phi
=
\frac{\sin^2\theta}{s_1^2s_2^2}
>0,
\]

where \(\theta\) is their included angle.
Thus two ranges can provide a locally full-rank two-dimensional FIM.
They do not,
however,
remove the global mirror ambiguity of two circle intersections.

The private state is propagated by

\[
\bar x^-_{k+1}
=
\bar x_k+\Delta t\,u_k,
\qquad
P^-_{k+1}
=
P_k+Q,
\]

using the existing declared \(Q=0.25I\) development reserve.
Its state machine is frozen as follows.
If frame \(k\) publishes an accepted fresh estimate,
the outgoing private state is reset to that selected
\((x_k,P_k)\),
with source-fresh frame \(k\),
propagated-to frame \(k\),
and age zero.
Before the attempt at frame \(k+1\),
the state is propagated exactly once with the applied held command \(u_k\).
If that attempt is not accepted,
the once-propagated private state is preserved;
neither a public predicted output nor an unavailable output overwrites or
re-propagates it.
If the preceding private state,
command,
or propagation is invalid,
the private state becomes absent until an accepted current-frame estimate
from an existing valid path resets it.
For every valid incoming or outgoing propagated state,

\[
\texttt{private\_age\_frames}
=
\texttt{private\_propagated\_to\_frame}
-
\texttt{private\_source\_fresh\_frame}.
\]

The public prediction may expose the same propagated mean and covariance
while its age is at most two,
but it is not the source of the private recursion.
The private state remains unpublished after the public two-frame prediction
expires.
On the new path,
it is historical information for the discrete branch identity,
not a range measurement,
continuous WNLS input,
or term in \(\Phi\).

For each distinct converged circle/WNLS branch \(b\),
compute the existing modeled consistency score

\[
q_b
=
(x_b-\bar x^-)^{\mathsf T}
(P_b+P^-)^{-1}
(x_b-\bar x^-).
\]

Use the already frozen threshold

\[
q_\star=11.829007011943707.
\]

Reacquisition is branch-identifiable only when exactly one distinct branch
satisfies \(q_b\le q_\star\)
and every other distinct branch satisfies \(q_b>q_\star\).
If zero or multiple branches pass,
the attempt fails closed.

This score is a deterministic,
truth-free modeled-consistency gate.
Because the current diagonal covariance model omits possible correlations
between the private prior and the current reference estimates,
the threshold is not claimed to be an exact chi-square hypothesis test or a
true-error probability guarantee.
Its empirical behavior must be reported in the new development evidence.

## State and publication semantics

No new public status is introduced.

- `fresh` means the estimate received a current-frame continuous WNLS update
  using only current-valid base references and current-fresh strictly
  lower-index UAV references.
  On the new path,
  its branch identity is additionally selected using the disclosed
  historical private prior.
- `predicted` retains the existing meaning and maximum public age of two.
- `unavailable` retains the existing fail-closed meaning.

Using a private prior to select between two global WNLS branches does not make
the prior an anchor,
a range measurement,
a continuous WNLS input,
or an FIM term.
It does provide historical information about the discrete branch identity
and must not be described as information-free.
The public localization output retains the existing public status,
estimate,
covariance,
epsilon,
age,
and provenance fields.
It must not expose the private estimate or covariance.

The raw attempt diagnostic for every considered new-path row,
including every accepted,
rejected,
and unavailable outcome,
must serialize exact required fields:

- the two mandatory active references;
- their current-fresh status and recursive base provenance;
- both branch candidates and complete solver results;
- each branch consistency score and pass/fail result;
- the selected branch identity;
- `branch_selection_prior_estimate`,
  `branch_selection_prior_covariance`,
  `branch_selection_prior_source_fresh_frame`,
  `branch_selection_prior_propagated_to_frame`,
  and `branch_selection_prior_age_frames`;
- `next_private_state_status`,
  `next_private_state_estimate`,
  `next_private_state_covariance`,
  `next_private_state_source_fresh_frame`,
  `next_private_state_propagated_to_frame`,
  and `next_private_state_age_frames`;
- an explicit Boolean `prior_used_for_branch_selection` flag; and
- explicit `prior_used_in_fim=false` and
  `prior_used_for_continuous_update=false` flags.

`prior_used_for_branch_selection` is true if and only if both branch scores
were evaluated against a valid incoming prior.
If rejection occurs before scoring,
both branch-score and pass fields are required null and this flag is false.
For an accepted row,
the outgoing `next_private_state_*` fields must encode the selected
\((x_k,P_k)\),
source-fresh frame \(k\),
propagated-to frame \(k\),
and age zero.
For a rejected row with a valid incoming prior,
the outgoing state must be byte-for-byte equal to the already once-propagated
incoming branch-selection prior.
When rejection occurs before a valid incoming prior exists,
the diagnostic must use the schema's explicit absent status and required null
state fields.
The raw values must be sufficient for the analyzer to independently
recompute every \(q_b\);
runtime-provided scores are not trusted.

The published covariance and epsilon must be recomputed solely from the
selected candidate's existing current-range FIM.
They are local modeled quantities conditional on the selected branch being
the correct physical branch;
they do not cover the discrete event of selecting the wrong mirror branch.

## Invocation, acceptance, and decision flow

The new path is considered when all of the following structural conditions
hold:

1. the normal current attempt has no live public prediction;
2. reference qualification returns exactly two active mandatory UAV
   references and no active base reference;
3. there are no active optional references;
4. every UAV reference is current-fresh,
   strictly lower-index,
   finite,
   canonical,
   SPD,
   and recursively supported by valid base provenance;
5. both mandatory fixed CBF references are present in the active FIM set; and
6. both current range measurements are present and finite.

Every considered attempt emits a raw diagnostic even if a later acceptance
condition fails.
Acceptance additionally requires:

7. the incoming private prior estimate and covariance are finite,
   canonical,
   and SPD;
8. both circle branches can be deterministically enumerated;
9. all retained branch solver results are converged and have valid SPD FIMs;
10. exactly one distinct branch passes the modeled consistency threshold; and
11. the selected candidate passes all existing numerical,
    provenance,
    cost,
    stationarity,
    and publication checks.

At exactly two active references,
the unchanged reduced-cost definition becomes

\[
\frac{\text{WNLS cost}}{\max(1,N-2)}
=
\text{WNLS cost}
\le 9,
\qquad N=2.
\]

Failure uses the existing reason
`reacquisition_reduced_cost_exceeds_nine`.
The three-reference minimum is the only existing reacquisition acceptance
condition removed on this path.

The decision sequence is:

```text
qualify current references
  -> enumerate canonical negative/positive circle starts
  -> solve one independent continuous WNLS result from each circle start
  -> recompute q for each branch against the private prior
  -> require exactly one passing branch
  -> apply unchanged candidate/FIM/provenance validation
  -> publish fresh or fail closed through the existing prediction lifecycle
```

The branch selector cannot inspect truth,
offline error,
future frames,
legacy status,
seed-specific aggregate results,
or downstream availability.

## Determinism and branch identity

Circle candidates retain the existing negative-oriented then
positive-oriented order.
Reference keys retain canonical
`(base < uav, numeric ID)` ordering.

Distinct converged branches are reconstructed only from the two circle-origin
solver results.
The existing frozen
`CANDIDATE_DEDUP_M = 1e-9 m`
Euclidean threshold defines whether two finite solver results represent the
same branch.
Both original circle starts must be distinct by more than
`CANDIDATE_DEDUP_M`,
and the two converged WNLS results must also remain distinct by more than that
threshold.
If distinct circle starts converge to the same finite solution,
the missing mirror solution has not been reconstructed and reacquisition is
rejected.
If a circle branch is missing,
invalid,
non-converged,
or has an invalid FIM,
two-branch identifiability is not established and reacquisition is rejected.

The private seed,
algebraic start,
and every other existing start are excluded from this new path's continuous
WNLS calls,
branch construction,
and publication representative.
The selected publication must bind directly to the unique passing
circle-origin solver result.

No new tolerance may be selected from Stage 1 offline errors.

## Failure handling

The method remains fail closed:

| Condition | Result |
| --- | --- |
| missing/invalid mandatory range | existing `reference_unavailable` path |
| predicted/unavailable/future-index UAV anchor | qualification failure and integrity violation if used |
| any active base, optional, or non-UAV reference, or fewer/more than two active mandatory UAV references | use the existing applicable path; do not invoke this selector |
| invalid or absent private state | reject reacquisition |
| no valid two-circle geometry | reject reacquisition |
| branch FIM not SPD | reject reacquisition |
| neither branch passes \(q_\star\) | reject reacquisition |
| both branches pass \(q_\star\) | reject as mirror-ambiguous |
| selected branch fails existing solver/provenance/publication validation | reject reacquisition |
| accepted branch | publish current-range `fresh`; reset public prediction age and private-state age to zero |
| rejected branch after public expiry | publish `unavailable`; preserve private seed only |

No exception can silently drop either fixed mandatory reference,
substitute an optional reference,
or publish the private state.

## Evidence design

The completed Stage 1 v4 roots are immutable forensic and comparison
evidence.
They are not rerun,
renamed,
or overwritten.

The authoritative preserved comparator identities are:

| Artifact | Frozen identity |
| --- | --- |
| v4 replay root | `/private/tmp/cbf2026-predictive-wnls-development/stage1-v4` |
| v4 replay manifest | `123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223` |
| v4 compressed process | `9710d1a5792e682bb272d0bb3305162180d1e0b9a0636647d6e071d1782a699b` |
| v4 decompressed process | `7047989804fd03bd4cadffac8212349b7183857623ac201d14ef2fc3e30ee2e1` |
| v4 analysis root | `/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v4` |
| v4 analysis manifest | `e50352825665bb94c0e3b32c7922a081f093100daf0ed8e5c8ce9e9f32bd9238` |
| v4 analysis JSON | `8a0a417f2d3b139fa248bef8157a3d4871fa04042aaad82ce5cd31f470a6ff1e` |
| v4 analysis Markdown | `986dcf72f72ab6dc3d424d0de6cdbf410cf271d174d16caef32945acc7d4e13b` |
| preserved legacy baseline process | `c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003` |

The baseline-process path is the exact path bound in the authoritative v4
replay manifest.
The new protocol and analyzer must exact-match the replay root,
all listed file/process hashes,
and that manifest-bound baseline path,
size,
and SHA-256 before reading comparator data.
Missing,
relocated,
or mismatched comparator evidence fails closed.
The new analyzer recomputes its paired gates from the preserved baseline and
v4 raw streams;
the analysis JSON and Markdown are cross-check records,
not substitutes for raw recomputation.

The new development round must use these distinct,
exact identifiers:

- the stable method identifier
  `two_range_private_branch_reacquisition`;
- protocol schema
  `cbf2026-two-range-reacquisition-protocol-v1`;
- raw row/manifest schema
  `cbf2026-two-range-reacquisition-raw-v1`;
- analysis schema
  `cbf2026-two-range-reacquisition-analysis-v1`;
- registration/authorization schema
  `cbf2026-two-range-reacquisition-registration-v1`;
- new smoke A,
  smoke B,
  raw,
  and analysis roots;
- an implementation commit frozen before protocol generation;
- exact source/input/hash binding;
- two byte-identical deterministic smokes;
- one no-retry registered replay of only the new method on the same preserved
  development trajectory and 20 frozen noise seeds;
- a compact analyzer paired to the preserved Stage 1 baseline and complete
  arm; and
- a fresh independent evidence review.

Before the registered full-grid replay,
only deterministic unit/adversarial tests,
synthetic fixtures,
and the single frozen mechanism key may execute the new method.
No unregistered run may cover the full 20-seed trajectory grid,
an equivalent partition or union of that grid,
or enough of that grid to inspect scientific availability/error aggregates.
The implementation commit,
analyzer commit,
all four schemas,
thresholds,
input hashes,
exact key grid,
gates,
and stop rules must all be frozen in a hashed protocol and pass independent
preflight review before the registered replay is authorized.
Smoke A and smoke B are byte-identical executions of only the frozen
mechanism fixture and synthetic records;
they are not subset scientific runs.
Byte identity applies separately to the decompressed process stream and its
compressed stream;
timestamped manifests and root directories are not required to be identical.
Each schema must enumerate its exact top-level and nested fields,
required-null cases,
types,
finite-number rules,
and canonical serialization order.
Missing,
extra,
misordered,
or noncanonical fields fail closed.

The registered raw key grid is exactly:

```text
method:
  two_range_private_branch_reacquisition
range-noise seeds:
  every integer 20260727 through 20260746 inclusive
frames:
  every integer 0 through 499 inclusive
robots:
  every integer 1 through 14 inclusive
```

It contains exactly
`1 * 20 * 500 * 14 = 140000`
unique `(method, seed, frame, robot)` rows,
ordered canonically by method,
then ascending seed,
frame,
and robot.
Missing,
duplicate,
extra,
or out-of-order keys fail the registered run.

The run retains the existing evidence lifecycle:
at least `8 GB` free before launch,
fail closed below `6 GB`,
at most `2 GB` raw output,
at most `10 MB` compact output,
strict JSON,
one no-retry registered run,
and exact-once terminal accounting.
All Stage 1 v2,
v3,
and v4 roots remain immutable.

If the registered replay aborts or violates its lifecycle,
its root is retained as forensic evidence and that protocol is a failed
scientific run.
It cannot be silently replaced by a new root.
Any later attempt requires a newly identified method/protocol round,
an explicit reason,
fresh preflight review,
and user authorization;
it cannot be reported as the original no-retry run.

Using the same truth trajectory is permitted only as explicitly labeled
development evidence.
It is not an independent confirmation sample.
No bounded-input Stage 2 plan or trajectory may be created by this design.

The post-hoc mechanism key

```text
seed 20260727, frame 180, robot 12
mandatory UAV references 10 and 11
```

may be frozen as a regression fixture because it exhibits the exact
two-reference rejection mechanism.
It cannot define a threshold,
serve as a statistical denominator,
or replace the full registered grid.

## Frozen scientific gates

The new method must retain all nine Stage 1 gates without reinterpretation:

| Gate | Required result |
| --- | --- |
| maximum fresh error | strictly below `50 m` |
| maximum all-published error | strictly below `50 m` |
| paired both-fresh p95 | exact-intersection new p95 must be at most its paired preserved-baseline p95 |
| fresh retention | at least `0.98` of the `127190` baseline-fresh cohort; integer numerator at least `124647` |
| fresh-or-predicted availability | at least `0.95` of `140000`; integer numerator at least `133000` |
| maximum public prediction age | at most `2` |
| qualification/reference violations | zero |
| current-frame provenance violations | zero |
| ascending-DAG violations | zero |

The paired both-fresh gate retains the Stage 1 v4 semantics exactly.
The comparator is the preserved legacy `dynamic_dag_wnls` baseline stream
bound by the v4 protocol whose JSON SHA-256 is
`09b236978e2e21bac226dc3d00c36a2ee5cdf5fea0e616f2c1d875b029d4e4e0`.
Rows are joined exactly on
`(seed, frame, robot)`.
The gate cohort is the intersection on which both that baseline and the new
method publish `fresh`.
Both baseline and new p95 values are recomputed on that same intersection;
the old v4 scalar is not reused when the intersection changes.
For sorted finite values \(x_0,\ldots,x_{n-1}\),
the p95 uses the frozen linear rule
\(h=0.95(n-1)\) and linear interpolation between
\(x_{\lfloor h\rfloor}\) and \(x_{\lceil h\rceil}\).
An empty cohort fails.
The gate passes when
\(\mathrm{p95}_{new}\le\mathrm{p95}_{baseline}\),
so exact equality passes.

Additional two-range integrity gates are:

- zero predicted/unavailable anchor uses;
- zero invocation with an active base,
  optional,
  or non-UAV reference;
- zero missing fixed-CBF-reference publications;
- zero publications that serialize the private prior as an anchor,
  FIM term,
  continuous-update input,
  covariance source,
  or estimate representative;
- zero private-prior,
  algebraic,
  or non-circle WNLS starts on the new path;
- zero non-circle publication representatives;
- zero branch scores recomputed from non-runtime information;
- zero branch-selection reconstruction mismatches;
- zero cases where zero or multiple passing branches publish fresh;
- zero selected-branch/result binding mismatches;
- zero private-seed propagation/source-frame mismatches;
- zero `predicted` output emitted by the new selector;
- complete denominator accounting for every considered,
  accepted,
  rejected,
  and unavailable two-range row; and
- exact preservation of all existing covariance,
  candidate,
  command,
  and provenance contracts.

The analyzer must separately report:

- considered two-range attempts;
- rejection-stage counts before and after branch scoring;
- branch-score outcomes:
  exactly one,
  none,
  multiple passing,
  or not evaluated;
- two-range accepted/rejected/publication counts;
- root-rejection and downstream-unavailability counts;
- outage episode count and length distribution;
- errors and modeled containment for two-range accepted publications;
- error tails stratified by depth,
  seed,
  robot,
  time,
  and private-seed age; and
- baseline-fresh transition counts into fresh,
  predicted,
  and unavailable.

Conditional error improvements cannot replace either availability gate.

## Test strategy

### Pure estimator tests

Tests must cover:

- positive and negative mirror branches selected by opposite private priors;
- exactly one branch inside the frozen consistency gate;
- neither branch inside;
- both branches inside;
- \(q=q_\star\),
  `nextafter(q_star, -inf)`,
  and `nextafter(q_star, +inf)`;
- non-finite \(q\) and failure to solve
  \(P_b+P^-\);
- swapped reference input order with identical canonical result;
- tangent,
  disjoint,
  contained,
  coincident,
  zero-range,
  and nearly collinear circle geometry;
- invalid/non-SPD private covariance;
- long-lived private-seed propagation without public publication;
- distinct original circle starts whose WNLS results merge,
  which must reject;
- selected representative bound to the physical passing branch;
- no private covariance contribution to the published FIM;
- no private,
  algebraic,
  or non-circle continuous WNLS start or representative;
- no predicted-anchor eligibility change; and
- unchanged behavior for three-or-more-reference reacquisition and live public
  prediction.

### Producer and schema tests

Tests must prove:

- every branch and score field is complete and canonical;
- the private prior cannot appear in active references or FIM terms;
- the private prior cannot appear as a WNLS start or publication
  representative;
- fixed mandatory references cannot be omitted or substituted;
- publication is impossible after zero/multiple passing branches;
- public fresh covariance/epsilon exactly match the selected current-range
  candidate;
- current command/private-seed source-frame propagation is exact;
- raw output ordering and strict JSON are deterministic;
- missing,
  duplicate,
  extra,
  and out-of-order registered keys reject;
- every considered accepted/rejected/unavailable attempt has complete
  branch-selection-prior,
  branch,
  score,
  outcome,
  and outgoing-private-state fields;
- schema-specific null and absence cases reject every alternative encoding;
- smoke A/B compressed streams match each other,
  decompressed streams match each other,
  and manifests remain separately hashed; and
- old Stage 1 protocol and evidence roots remain immutable.

### Analyzer adversarial tests

The analyzer must independently reject:

- forged branch scores;
- forged incoming or outgoing private-state lineage;
- swapped branch labels;
- hidden or deleted circle branches;
- distinct raw circle branches collapsed into one converged branch;
- private prior inserted as a reference;
- private prior inserted as a WNLS start or publication representative;
- predicted anchor relabeled fresh;
- selected result rebound to a non-passing branch;
- FIM/covariance/publication substitution;
- truth-dependent or future-frame branch choice;
- reference/provenance/order mutation;
- incorrect outage lineage;
- omitted adverse rows;
- duplicate,
  extra,
  or out-of-order denominator rows; and
- any attempt to reuse a Stage 1 v4 evidence root.

### Real integration fixtures

At minimum,
the mechanism key
`(20260727, 180, 12)`
must pass through the real producer,
compact schema,
and analyzer.
The test asserts only deterministic mechanism behavior,
not a favorable full-grid scientific result.

## Paper and theory boundary

Even if every new development gate passes:

- the evidence still uses one preserved truth trajectory;
- the estimator remains outside the controller;
- the FIM remains a diagonal,
  marginal-covariance approximation;
- shared-ancestor cross-covariances remain unmodeled;
- the three-standard-deviation radius remains a modeled surrogate;
- the source trajectory still contains `243 / 7000` applied component-bound
  violations; and
- the paper-edit and bounded-input Stage 2 gates remain closed.

The result may support a development claim that a truth-free two-range branch
selector repairs the observed reacquisition cascade without degrading the
frozen error/integrity gates.
It cannot support cross-trajectory reliability,
closed-loop estimator robustness,
unconditional CBF safety,
or a deterministic localization-error guarantee.

## Completion criterion

This design is complete when:

1. the specification has no unresolved placeholder,
   contradiction,
   or ambiguous status/branch rule;
2. an independent theory review confirms the private prior is used only for
   the disclosed discrete branch-identity selection;
3. an independent evidence-design review confirms all old roots,
   thresholds,
   and exact-once boundaries are preserved;
4. the user approves this written specification; and
5. a separate test-first implementation plan is written before any code
   change.
