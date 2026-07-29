# Independent geometric-stability protocol review

**Date:** 2026-07-29

**Current execution verdict after re-review:** **APPROVED**

**Paper-edit gate:** **CLOSED**

## Scope

This review covers Task 4 Step 3 only.  It independently checks the frozen
mathematical note and exploratory protocol against:

- the approved scientific contract at
  `b2fda72a420c96899549fbe11ba254fd3d68c62c`;
- the Task 4 brief and Steps 1--2 implementation report;
- the analyzer and tests committed at
  `c9f73bab712e602426860fa2d7b04245f18dee48`;
- the immutable Gate 2 comparison, parent manifest, child bundles, and truth
  trajectory; and
- the analyzer's actual report, integrity, disk, and publication interfaces.

The review did not execute the registered production command and did not
create, inspect, or modify an authoritative analysis output.  No paper edit is
authorized.

## Evidence checked

- `docs/superpowers/specs/2026-07-29-cbf2026-geometric-nondegeneracy-and-empirical-stability-design.md`
- `.superpowers/sdd/2026-07-29-cbf2026-geometric-stability-implementation/task-4-brief.md`
- `.superpowers/sdd/2026-07-29-cbf2026-geometric-stability-implementation/task-4-report.md`
- `docs/diagnostics/2026-07-29-geometric-nondegeneracy-theory.md`
- `docs/diagnostics/2026-07-29-geometric-stability-protocol.md`
- `scripts/diagnostics/analyze_geometric_stability.py`
- `tests/test_analyze_geometric_stability.py`
- the registered comparison, paired parent manifest, strict/restart child
  artifacts, truth data, and truth-run manifest at the absolute paths frozen in
  the protocol

Fresh read-only checks established:

- the approved design is byte-identical to its version at `b2fda72`;
- repository `HEAD` is
  `c9f73bab712e602426860fa2d7b04245f18dee48`;
- the analyzer is unchanged from that commit and has SHA-256
  `ee58241cf295f70d0b2bac753b5d8770a8aa910c746fa44c65299942f6a002e6`;
- the comparison and parent hashes are respectively
  `ef8014915b3cced6a75e094b3d16f01f17b5f881c6357dff6b885b6e871498ca`
  and
  `c110275910c3e44ecf97c66ba7346fbb0b340cfb844d050c0ecf56944e5ff55a`;
- all ten protocol-listed truth/strict/restart artifact hashes match their
  current bytes;
- the registered output directory does not exist;
- current `/private/tmp` available space is above the 8,000,000,000-byte
  launch threshold and the measured runtime cache is below the
  2,000,000,000-byte ceiling; and
- `conda run -n cbf_env python -m unittest
  tests.test_analyze_geometric_stability -q` passed all 81 tests.

## Checklist

| Review item | Result | Independent assessment |
| --- | --- | --- |
| \(P_i\), \(C_i\), and \(G_i\) distinction | Pass | The note defines \(P_i=\Phi_i^{-1}\) as modeled, \(C_i\) as true estimator-error covariance when it exists, and \(G_i\) as unweighted geometry.  It explicitly rejects \(P_i=C_i\), deterministic containment, and unconditional safety interpretations. |
| Active-set FIM inequality | Pass | The quadratic-form proof correctly uses \(v_{ij}^{-1}\geq1/\bar v_i\) to obtain \(\Phi_i\succeq(\gamma_i/\bar v_i)I\), then reverses Loewner order on inversion. |
| Dynamic geometry primary; fixed pair explanatory | Pass | The complete dynamic active set is the theorem's primary geometry and the protocol's primary empirical family.  The fixed-pair determinant and target tracking are explicitly sufficient/explanatory only. |
| Soft task CBF and range-bound limitation | Pass | The note correctly observes that neither a soft tracking objective nor two observer-to-reference upper range bounds impose a positive reference separation, triangle-area, or included-angle margin. |
| Finite lower-index DAG proof | Pass | The induction has a finite topological order, finite positive effective variances, an explicit active-set geometry premise, and no unsupported true-covariance or estimator-convergence conclusion. |
| \(B_d\) recursion and limits | Pass | \(B_0=0\) is introduced under the additional zero-root-covariance corollary.  The note states that the recursion need not contract, applies only at fixed finite depth, and is not a bound on \(C_i\), realized error, or containment probability. |
| One-trajectory geometry replication | Pass | True geometry is counted once per unique `(frame_index, robot_id)` tuple; exact equality across all 20 seed repetitions is an integrity condition, not replication. |
| Seed-level estimator statistics | Pass | One range-noise seed nested in one trajectory is declared as the estimator-outcome statistical unit.  Seed strata are retained, while rows, frames, UAVs, and edges are explicitly treated as correlated repeated observations.  No row-level inferential test is authorized. |
| Exact immutable sources | Pass | Absolute comparison, parent, child, and trajectory paths and hashes match the current read-only files.  The source commit and analyzer SHA-256 match the committed analyzer, and the analyzer diff from the implementation commit is empty. |
| Exact command, output, and no retry | Pass | The CLI, authoritative output directory, exactly two output names, atomic no-replace publication, one-invocation budget, and no-automatic-retry rule agree with the committed interface. |
| 8 GB, 6 GB, 2 GB, and 10 MB rules | Pass with Minor note | The thresholds equal the analyzer constants.  The protocol's strict postflight requirement of allocation below 10,000,000 bytes safely narrows the analyzer's rejection of allocation above the cap; equality would be invalid evidence and would not authorize a retry. |
| No raw duplication | Pass | The analyzer streams the compressed inputs and finalizes sample lists into counts and quantiles.  The frozen output permits only compact aggregate JSON and Markdown, with no process-row or trajectory copy. |
| Denominators and five time bins | Pass | True geometry uses trajectory tuples; estimated geometry and modeled FIM use their finite/valid subsets; absolute error, availability, containment, and \(q\) retain distinct denominators.  The five half-open/final-closed 50-second bins match the implementation. |
| Abort and claim boundaries | Pass | Trust-root, row, geometry, target, domain, disk, output, path, publication, and final-rehash failures fail closed.  The post-hoc, one-trajectory, finite-depth claim boundary is explicit. |
| Nominal-target fixed-pair family required by approved design | **Round 1 fail -- closed on re-review** | Round 1 found that the analyzer did not publish this predeclared design family.  Commit `52100827b09434e91b1ded140d7d786bc817a981` adds the required nominal and perturbation-margin families; the closure is audited below. |
| Paper-edit gate | Pass | Execution alone cannot open the gate.  Reviewed evidence, reconciled integrity and denominators, resolution of every Critical/Important issue, and a separate paper-edit decision remain prerequisites. |

## Findings

### Critical

None.

### Important

#### I-1. The frozen analyzer and protocol omit a report family required by the approved scientific contract

The approved design requires the implementation to extract the fixed-reference
target topology, compute nominal fixed-pair angle margins and the largest
admissible target-tracking deviation, and report nominal-target,
true-position, and estimated-position fixed-pair bearing-angle distributions.
The analyzer instead publishes true-position and estimated-position fixed-pair
geometry plus separate per-robot target-tracking distance.  The protocol
accurately discloses that omission and refuses a nominal-target nondegeneracy
claim, but that is a unilateral scope reduction from the still-frozen approved
contract.

This omission does not invalidate the primary dynamic active-set geometry or
the finite-DAG theorem.  It does prevent the registered evidence from testing
the approved explanatory link between triangular-ladder target geometry,
realized target-tracking deviation, and fixed-pair angle degradation.  It is
therefore Important rather than Critical, and execution is not approved while
it remains open.

**Required correction before execution:** add and adversarially test a compact
nominal-target fixed-pair family constructed from the frozen per-frame target
centers, fixed-reference topology, and configured base positions.  It must
report the predeclared nominal bearing-angle/noncollinearity margins and the
corresponding admissible tracking-deviation quantity (or an explicitly
equivalent perturbation margin), once per trajectory tuple with seed
repetitions used only for integrity.  Then commit the revised analyzer/tests,
freeze the new full source commit and analyzer SHA-256 in the protocol, update
the family and denominator/abort descriptions, and repeat independent protocol
review before launch.

### Minor

#### M-1. The permitted cache-deletion branch cannot literally repeat the listed complete preflight

The protocol permits deletion of
`/Users/xirhxq/.cache/codex-runtimes` when launch space is insufficient and
then requires the complete preflight to be repeated.  That preflight contains
an unconditional `du -sk` of the deleted directory and also says any command
failure aborts.  If the directory is absent, the repeated `du` fails even
though the intended cache allocation is zero.

**Required editorial correction:** make the cache probe explicitly
absence-safe and record an absent cache as zero bytes, or specify recreation of
an empty cache directory before repeating the complete preflight.  This issue
does not affect the current state because available space already exceeds the
launch threshold and no cache deletion is needed.

## Adjudication of writer concerns

1. **Absent nominal-target fixed-pair angle family:** Important and
   execution-blocking under the current approved contract.  Merely narrowing
   the claim in the protocol does not replace the registered family and
   perturbation-margin analysis.
2. **Strict postflight `<10,000,000` versus analyzer rejection only for
   `>10,000,000`:** not an execution-blocking inconsistency.  The frozen
   protocol imposes the stricter acceptance rule, declares equality invalid,
   and forbids a retry.  Aligning the operator in a future analyzer revision
   would improve clarity, but the present protocol does not permit an
   at-cap artifact to be accepted as evidence.

## Round 1 verdict

**NEEDS FIXES.**  There are no Critical findings and one open Important
finding.  The authoritative exploratory command must not be executed until
I-1 is corrected and independently re-reviewed.  The paper-edit gate remains
closed regardless of execution status.

## Re-review after I-1 correction

### Re-review scope and immutable identity

The correction was independently re-reviewed against the same approved
scientific contract and all Round 1 pass items.  The corrected analyzer and
tests are committed at
`52100827b09434e91b1ded140d7d786bc817a981`.  Fresh byte checks give analyzer
SHA-256
`6240d1488f92c259b6b545e987746e151808709c10e377d5284ba02d569078d8`,
exactly as frozen in the updated protocol.  The working-tree analyzer and tests
have no diff from that commit.  The approved design remains byte-identical to
commit `b2fda72`.

The registered comparison, paired parent, truth trajectory, truth-run
manifest, and all eight strict/restart child artifacts retain every hash listed
in Round 1 and in the updated protocol.  The updated preflight uses the full
corrected analyzer commit, the exact corrected analyzer hash, the same absolute
input paths, and the same exactly-once production command and output
directory.  The authoritative output directory still does not exist.

### I-1 correction verification

The correction closes I-1 as follows:

1. `geometry.nominal_fixed_pair_all_primary` reconstructs the observer and
   fixed-UAV targets from same-frame immutable `cvt.center` records and treats
   each configured base position as its zero-deviation target.  Missing or
   unknown target semantics and a nonpositive, nonfinite, or degenerate
   nominal triangle fail closed.
2. For nominal side lengths \(a_i^\star,b_i^\star,c_i^\star\), it computes all
   three strict triangle slacks and freezes

   \[
   \delta_i^\star
   =
   \frac{
   \min\{
   a_i^\star+b_i^\star-c_i^\star,\,
   a_i^\star+c_i^\star-b_i^\star,\,
   b_i^\star+c_i^\star-a_i^\star
   \}
   }{6}.
   \]

   The factor six is correct for the registered uniform three-vertex
   certificate: each of the two positive side terms can decrease by
   \(2\delta\), while the subtracted side can increase by \(2\delta\).
   Therefore every expanded-interval triangle inequality remains strict only
   for \(\delta<\delta_i^\star\).  The implementation correctly calls
   \(\delta_i^\star\) a supremum and places equality in the
   not-strictly-satisfied count.
3. Nominal metrics are recorded once per unique
   `(frame_index, robot_id)` trajectory tuple.  Exact repetition across the 20
   range-noise seeds is an integrity condition; the nominal family has no seed
   stratum and does not count those repetitions as geometric replicates.
4. `geometry.estimated_fixed_pair_tracking_margin_finite` uses the same
   explicit finite observer-and-two-reference requirement as estimated
   fixed-pair geometry.  It reports the maximum three-vertex deviation, the
   nominal supremum, their signed margin, strict satisfied/not-satisfied
   counts, and named inapplicability reasons.  The two strict counts reconcile
   to the finite-row denominator, and the family retains depth, time-bin,
   squad, and seed strata.
5. Nominal bearing, angle-to-collinearity, cosine, area, side-length, slack,
   geometry-eigenvalue, and perturbation-margin summaries are present in the
   compact JSON.  Deterministic Markdown renders the nominal and estimated
   perturbation families.  Tests confirm that raw measurements, truth rows,
   and process-stream names are not duplicated into either artifact.
6. Dynamic active-set geometry remains the primary family.  Nominal, true, and
   estimated fixed-pair geometry and perturbation satisfaction remain
   explanatory and do not create a hard tracking invariant, deterministic
   true-error bound, or unconditional safety claim.

Fresh verification produced:

- 9 focused nominal, strict-boundary, seed-semantic, denominator,
  missing-target, and compact-publication tests: pass;
- all 88 analyzer-module tests: pass;
- all 307 Python diagnostic tests: pass; and
- no analyzer/test working-tree drift from the corrected implementation
  commit.

### Re-review findings

#### Critical

None.

#### Important

None.  I-1 is closed.

#### Minor

M-1 remains open and unchanged: after deletion of
`/Users/xirhxq/.cache/codex-runtimes`, the protocol's unconditional repeated
`du -sk` probe would fail unless an absent cache is explicitly recorded as zero
or an empty directory is recreated.  This does not block the registered
execution in the current state because available launch space already exceeds
8,000,000,000 bytes and no cache deletion is needed.

The exact-10-MB adjudication is also unchanged and nonblocking.  The analyzer
rejects allocation above 10,000,000 bytes, while postflight accepts only
allocation strictly below 10,000,000 bytes.  An exactly-at-cap artifact would
be invalid evidence and would not authorize a retry.

### Final execution verdict

**APPROVED.**  No Critical or Important issue remains.  This approval is
limited to the exact frozen, exactly-once exploratory command after the Task 4
documents are registered and every preflight requirement passes.  It does not
approve a retry, a changed analyzer or denominator, a confirmatory
interpretation, or any paper edit.

The paper-edit gate remains **CLOSED** until the analyzer output, repository
evidence report, and independent evidence review are complete; all
denominators and integrity checks reconcile; and a separate paper-edit
decision supports a narrow statement.
