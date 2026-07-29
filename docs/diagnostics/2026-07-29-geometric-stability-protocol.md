# Frozen post-hoc geometric-stability exploratory protocol

## Registration status and scope

This protocol freezes one post-hoc exploratory analysis before its
authoritative execution.  It reuses the completed Gate 2 paired replay and
does not rerun the controller, truth trajectory, WNLS, active-reference
selection, FIM, coefficient-3 radius, or range-noise draws.

The restart-policy `dynamic_dag_wnls` arm is the primary descriptive
population because it contains the recovered deeper-index estimates.  The
strict arm is read and verified in lockstep to preserve the paired integrity
contract and explain selection and missingness.  Dynamic active-set geometry
is primary.  Fixed-pair geometry is sufficient and explanatory only.

The statistical unit for estimator outcomes is one range-noise seed nested
within one truth trajectory.  Rows, frames, UAVs, and reference edges are
correlated repeated observations and will not be treated as independent.
True geometry has one trajectory-level replicate; its 20 seed repetitions are
an integrity check, not 20 geometric replicates.

There is no confirmatory decision gate, no pass/fail scientific threshold, and
no \(p\)-value.  No exploratory result may later be relabeled as prospective
confirmation.

## Frozen scientific and implementation sources

The approved scientific contract is commit `b2fda72` together with
`docs/superpowers/specs/2026-07-29-cbf2026-geometric-nondegeneracy-and-empirical-stability-design.md`.

The committed analyzer implementation is frozen as follows:

| Field | Frozen value |
| --- | --- |
| Repository | `/private/tmp/cbf2026-diagnostic` |
| Branch | `codex/cbf2026-diagnostic` |
| Analyzer implementation commit | `52100827b09434e91b1ded140d7d786bc817a981` |
| Analyzer path | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/analyze_geometric_stability.py` |
| Analyzer SHA-256 | `6240d1488f92c259b6b545e987746e151808709c10e377d5284ba02d569078d8` |
| Analyzer schema | `cbf2026-geometric-stability-v1` |
| Estimator contract | `variable_weight_nls_full_residual_jacobian_v1` |

The implementation commit identifies the reviewed analyzer revision.  A
later documentation-only registration commit may become repository `HEAD`
before execution; the analyzer bytes must still have the exact SHA-256 above,
and the diff from the implementation commit to launch `HEAD` must contain no
change to the analyzer.  The output records both launch `HEAD` and the
analyzer SHA-256 so this distinction remains explicit.

## Immutable Gate 2 trust roots

The external roots supplied directly to the production command are:

| Input | Absolute path | Required SHA-256 |
| --- | --- | --- |
| Registered comparison JSON | `/private/tmp/cbf2026-warm-start-recovery-analysis/registered-gate-2/warm-start-recovery-comparison.json` | `ef8014915b3cced6a75e094b3d16f01f17b5f881c6357dff6b885b6e871498ca` |
| Paired parent manifest | `/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/manifest.json` | `c110275910c3e44ecf97c66ba7346fbb0b340cfb844d050c0ecf56944e5ff55a` |

The comparison pins the following consumed child bundles and immutable
trajectory:

| Input | Absolute path | Required SHA-256 |
| --- | --- | --- |
| Truth trajectory | `/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json` | `3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527` |
| Truth-run manifest | `/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json` | `6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb` |
| Strict process stream | `/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/strict/localization-calibration/20260729T112154.899215Z_b6f8a00b032541329a6e975d402f7d49/calibration.jsonl.gz` | `e84e43c332aba21664b63f52281416d47c21be684d2b5b83e26edca233efe67b` |
| Strict child manifest | `/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/strict/localization-calibration/20260729T112154.899215Z_b6f8a00b032541329a6e975d402f7d49/manifest.json` | `96eb398836320594e8f1ec82d8511273b48eb711f8a76de0aab89711c1a85d3a` |
| Strict summary JSON | `/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/strict/localization-calibration/20260729T112154.899215Z_b6f8a00b032541329a6e975d402f7d49/summary.json` | `e088ec11d1f5d87a33be89ac23db2d5548e8070a14636187575f629dd6b88c66` |
| Strict summary Markdown | `/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/strict/localization-calibration/20260729T112154.899215Z_b6f8a00b032541329a6e975d402f7d49/summary.md` | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` |
| Restart process stream | `/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/restart/localization-calibration/20260729T112437.413270Z_efb8e7137776415c833f11204cf519d5/calibration.jsonl.gz` | `c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003` |
| Restart child manifest | `/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/restart/localization-calibration/20260729T112437.413270Z_efb8e7137776415c833f11204cf519d5/manifest.json` | `5c33539a4030393ae5ca02939bbe204fdbf7f322adc2b874deacaf8364d99f18` |
| Restart summary JSON | `/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/restart/localization-calibration/20260729T112437.413270Z_efb8e7137776415c833f11204cf519d5/summary.json` | `6edcf53b0fcbc6cb03f0319ede1ee5ed2c4e7ab5ce42274c593927aed0064f65` |
| Restart summary Markdown | `/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/restart/localization-calibration/20260729T112437.413270Z_efb8e7137776415c833f11204cf519d5/summary.md` | `33217f673ee08ed888dd6c7a603f4451d46c2e7c26f32941b4f8c754d4e9464e` |

The comparison also records the immutable baseline directory
`/private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288`,
baseline manifest hash
`39ee3b5494ed623d20d16766244427476cd081ed0869feecb9e5072c6c8f481d`,
source-snapshot hash
`4ac5ca7d3608dcce4c54574ae33dd04b0683069e22bd344a34264d3dda3ef5cf`,
replay-implementation hash
`0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8`,
comparator-source hash
`9a54c312183f7f2864aba104a5624ea247e15e1107deaa4590e276ab1eda120d`,
and failure-analyzer-source hash
`1e3a7b7cd615b258fb83af53f7264d7aff327685f42c85787d0d9ae7df0b6315`.
These remain Gate 2 provenance; the geometric analyzer directly revalidates
the comparison, parent, child bundles, and trajectory used by this analysis.

## Resource preflight

Run all commands from `/private/tmp/cbf2026-diagnostic`.  The authoritative
output directory must not exist.  Before launch, execute:

```bash
df -Pk /private/tmp
du -sk /private/tmp/cbf2026-warm-start-recovery
du -sk /private/tmp/cbf2026-warm-start-recovery-analysis
du -sk /Users/xirhxq/.cache/codex-runtimes
shasum -a 256 \
  /private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/manifest.json \
  /private/tmp/cbf2026-warm-start-recovery-analysis/registered-gate-2/warm-start-recovery-comparison.json \
  /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  scripts/diagnostics/analyze_geometric_stability.py
git rev-parse HEAD
git diff --quiet 52100827b09434e91b1ded140d7d786bc817a981 -- \
  scripts/diagnostics/analyze_geometric_stability.py
git status --short --branch
test ! -e /private/tmp/cbf2026-geometric-stability-analysis/exploratory-v1
```

The `df -Pk` available-block count multiplied by 1024 must be at least
`8,000,000,000` bytes.  The rebuildable
`/Users/xirhxq/.cache/codex-runtimes` allocation from `du -sk`, multiplied by
1024, must not exceed `2,000,000,000` bytes.  Git status must show branch
`codex/cbf2026-diagnostic`, no uncommitted tracked change, and no untracked
entry except `build-diagnostic/`.  All four displayed hashes must exactly
match the frozen values above, and the analyzer diff check must exit zero.

If launch space is below `8,000,000,000` bytes, first record the cache
measurement.  The only permitted deletion is the rebuildable directory
`/Users/xirhxq/.cache/codex-runtimes`; no evidence, source, Git object, or
other cache may be deleted.  After that deletion, repeat the complete
preflight.  A preflight repetition is not an analysis retry because the
production command has not launched.

The analyzer enforces a live available-space floor of `6,000,000,000` bytes.
It checks before reading, every 10,000 paired rows, before and after each
persistent write, and immediately before publication.  Falling below that
floor aborts the run.  Compact output is limited to `10,000,000` allocated
bytes and must contain no duplicate raw process or truth stream.  The analyzer
rejects allocation above that cap; the postflight acceptance check below
requires allocation strictly below `10,000,000` bytes.

## Exactly-once production command

Execute the following command exactly once:

```bash
conda run -n cbf_env python -m \
  scripts.diagnostics.analyze_geometric_stability \
  --comparison \
  /private/tmp/cbf2026-warm-start-recovery-analysis/registered-gate-2/warm-start-recovery-comparison.json \
  --expected-comparison-sha256 \
  ef8014915b3cced6a75e094b3d16f01f17b5f881c6357dff6b885b6e871498ca \
  --expected-parent-manifest-sha256 \
  c110275910c3e44ecf97c66ba7346fbb0b340cfb844d050c0ecf56944e5ff55a \
  --output-dir \
  /private/tmp/cbf2026-geometric-stability-analysis/exploratory-v1
```

The execution budget is one invocation, with no automatic retry.  Any exit,
signal, exception, integrity rejection, or resource rejection after launch
consumes that invocation.  Do not change code, paths, hashes, metrics,
denominators, or interpretation and rerun after inspecting a partial result.
A failed authoritative attempt requires a new reviewed design before any
replacement execution.

## Frozen outputs

Successful publication creates exactly this directory and these two files:

```text
/private/tmp/cbf2026-geometric-stability-analysis/exploratory-v1/
├── geometric-stability.json
└── geometric-stability.md
```

No raw row, trajectory, process stream, cache, or replay copy may appear in
the output.  JSON is the structured evidence record; Markdown is a
deterministic rendering of its registered families and claim boundary.
Publication is atomic and refuses an existing output path.

After the one execution, run:

```bash
du -sk /private/tmp/cbf2026-geometric-stability-analysis/exploratory-v1
shasum -a 256 \
  /private/tmp/cbf2026-geometric-stability-analysis/exploratory-v1/geometric-stability.json \
  /private/tmp/cbf2026-geometric-stability-analysis/exploratory-v1/geometric-stability.md
df -Pk /private/tmp
shasum -a 256 \
  /private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/manifest.json \
  /private/tmp/cbf2026-warm-start-recovery-analysis/registered-gate-2/warm-start-recovery-comparison.json \
  /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json \
  scripts/diagnostics/analyze_geometric_stability.py
```

Require exactly the two registered names, output allocation below
`10,000,000` bytes, at least `6,000,000,000` available bytes, and unchanged
source hashes.  Record both output SHA-256 values.  A postflight failure
invalidates the output and does not authorize a retry.

## Frozen time bins and strata

The 250-second trajectory is divided into exactly five bins:

| Label | Interval |
| --- | --- |
| `0_to_lt_50_s` | \(0\leq t<50\) s |
| `50_to_lt_100_s` | \(50\leq t<100\) s |
| `100_to_lt_150_s` | \(100\leq t<150\) s |
| `150_to_lt_200_s` | \(150\leq t<200\) s |
| `200_to_250_s` | \(200\leq t\leq250\) s |

Where applicable, results are stratified by `squad_local_index`, time bin,
squad, and seed.  The term hop count is not used because dynamic base
references can shorten graph distance.

## Predeclared report families and denominators

### Integrity and provenance

The report records comparison, parent, strict child, restart child,
trajectory, analyzer, and launch-commit provenance.  Strict and restart
streams are zipped in exact key order.  Every paired row must have identical
external inputs and reconciled policy state, row order, truth, active
references, and registered range noise.  The paired-row count must equal the
trajectory/configuration dimensions, and all sources are rehashed before
publication.

### Primary dynamic active-set geometry

`geometry.true_dynamic_all_primary` is computed once for each unique
`(frame_index, robot_id)` dynamic primary tuple from the immutable truth
trajectory, independently of estimator convergence.  Equality across all 20
range-seed repetitions is required.  Its denominator is
`trajectory_tuple_count`; seed stratification is inapplicable because this is
one-trajectory geometry.  Metrics are active reference count,
\(\lambda_{\min}(G_i)\), \(\lambda_{\max}(G_i)\), and
\(\lambda_{\min}(G_i)/\lambda_{\max}(G_i)\).

`geometry.estimated_dynamic_finite` uses only restart dynamic primary rows
with a finite observer estimate and finite positions for every admitted
active reference.  Its explicit denominator is the count of those valid
rows; all other primary rows enter named inapplicability counts.  It reports
the same geometry metrics.  This family is conditional on estimator
availability and is not substituted for the all-primary true-geometry
family.

### Explanatory nominal, true, and estimated fixed-pair geometry

`geometry.nominal_fixed_pair_all_primary` reconstructs the observer target
from that frame's recorded `cvt.center`.  Each fixed UAV reference uses its
same-frame `cvt.center`, while each fixed base uses its frozen configured
position.  Unknown target semantics, a missing target, or a nominal target
triangle without finite positive side lengths, positive area, and strict
triangle inequalities aborts the analysis.

The nominal family is computed once per unique `(frame_index, robot_id)`
trajectory tuple.  All 20 range-seed rows must repeat the same immutable
tuple, but they are not geometric replicates.  The family is stratified
overall, by `squad_local_index`, time bin, and squad; seed stratification is
explicitly inapplicable.  It reports geometry eigenvalues, included angle,
angle-to-collinearity, absolute cosine, twice triangle area, the three nominal
side lengths, the minimum strict triangle slack, and

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

The value \(\delta_i^\star\) is the supremum for the registered uniform
three-vertex sufficient certificate.  If every vertex deviates from its
target by no more than \(\delta\), each side can change by \(2\delta\), and
every strict triangle inequality is guaranteed only when
\(\delta<\delta_i^\star\).  Equality is not certified.

`geometry.true_fixed_pair_all_primary` is also computed once per unique
trajectory tuple and verifies all 20 seed repetitions, with the same
trajectory-level denominator rule.  It reports the realized true-position
geometry eigenvalues and normalized minimum together with included angle,
angle-to-collinearity, absolute cosine, and twice triangle area.

`geometry.estimated_fixed_pair_finite` uses only restart dynamic primary rows
with a finite observer estimate and finite estimated positions for both fixed
references.  Its denominator is the count of those valid rows, with explicit
inapplicability reasons.  It preserves the estimated-position bearing
distribution separately from the nominal and true-position distributions.

`geometry.estimated_fixed_pair_tracking_margin_finite` uses the same finite
restart-row requirement.  It compares the observer estimate with its
same-frame target and both fixed-reference estimates with their target
positions.  A base contributes zero deviation because its configured
position is both its estimate and target; a fixed UAV reference uses the
current-frame topologically prior estimate and its recorded `cvt.center`.
For each valid row it reports the maximum of the three vertex deviations,
\(\delta_i^\star\), and the signed margin
\(\delta_i^\star-\max_\ell\lVert\hat p_\ell-p_\ell^\star\rVert\).
Its explicit denominator is the count of rows with all required finite
estimates.  Strictly satisfied and not-strictly-satisfied counts reconcile to
that denominator, and equality belongs to the latter.  Missing upstream UAV
estimates and other invalid rows enter named inapplicability counts.  This
family is stratified overall, by `squad_local_index`, time bin, squad, and
seed.

`geometry.target_tracking.true_position` is the truth-position distance to the
recorded per-robot `cvt.center`, computed once per trajectory tuple.
`geometry.target_tracking.estimated_position_finite` is the corresponding
estimated-position distance on finite restart dynamic primary rows.
Nominal, true-position, estimated-position, perturbation-margin, and
target-tracking quantities remain explanatory and never replace the primary
dynamic active-set geometry.

### Modeled FIM

`geometry.modeled_fim_valid` uses restart dynamic primary rows marked finite
whose recorded modeled metrics are finite and in domain.  Its denominator is
the number of those valid rows.  It reports
\(\lambda_{\min}(\Phi_i)\), FIM condition number, and modeled
\(\epsilon_i=3\sqrt{\lambda_{\max}(P_i)}\), with named inapplicability
reasons for other primary rows.  These are properties of \(P_i=\Phi_i^{-1}\),
not of the true covariance
\(C_i=\operatorname{Cov}(p_i-\hat p_i)\).

### Absolute error

`absolute_error` contains every restart dynamic primary row whose reconciled
error norm is finite, whether or not its current attempt converged.  Its
denominator is the number of finite error norms in each stratum.  It reports
median, p90, p95, p99, and maximum in metres overall and by
`squad_local_index`, time bin, squad, and seed.  Missing or nonfinite errors
enter explicit inapplicability counts.

Temporal behavior and the depth-one through depth-seven profile are
descriptive.  “Error does not explode” may be used only as the operational
observation that no sustained temporal runaway is visible over this tested
finite horizon, the finite-depth profile remains interpretable without a
terminal discontinuity from FIM singularity or estimator loss, geometry and
valid-row FIM margins remain separated from zero in the observed samples, and
upper-tail and maximum errors are reported in metres.  The word “bounded”
must be qualified as “within the tested finite horizon and squad depth.”

### Availability and acquisition

`availability` counts every restart dynamic primary attempt.  Its
`primary_attempts` denominator reconciles exactly to the `converged`,
`invalid`, and `failed` attempt-status counts and to raw failure-reason
counts.  Finite retained, stale retained, invalid, failed, pre-acquisition,
and frames-since-first-finite counts remain separate.

Longest consecutive unavailable and nonfinite-retained durations are computed
over each `(seed, robot_id)` series and reported in frames.  First-finite
acquisition reports acquired and never-acquired series out of the expected
seed-by-robot series, stratified by `squad_local_index`, squad, and seed.

### Radius calibration and mechanism diagnostics

`calibration` keeps two denominators separate.  Coefficient-3 containment uses
`converged_error_denominator`: restart dynamic primary attempts that converged
and have a finite reconciled error.  The `contained` count uses the recorded
state-containment result.  The \(q\) family recomputes

\[
q=(p_i-\hat p_i)^{\mathsf T}P_i^{-1}(p_i-\hat p_i)
\]

and uses `finite_q_denominator`: the subset with a valid finite covariance and
error vector.  It reports q median, p90, p95, p99, maximum, and the descriptive
count \(q>9\).  Containment and \(q>9\) are calibration diagnostics, not
deterministic true-error bounds and not hypothesis tests.

The same two denominators and counts are stratified by active UAV-reference
count, pairs sharing at least one uncertain UAV ancestor, and the
two-reference `opposite_baseline_side` proxy.  The proxy is not proof of a
mirror solution.  No row-level independence, post-hoc \(p\)-value, or
confirmatory threshold is permitted.

## Fail-closed and abort rules

Abort before launch if the output already exists; Git/source state is not the
frozen state; a registered hash, path, schema, policy, or manifest link does
not match; launch space is below `8,000,000,000` bytes after the only
permitted cache action; cache allocation exceeds `2,000,000,000` bytes; or
any preflight command fails.

After launch, abort without publication if any trust root changes; child
bundle or row integrity fails; strict/restart order, cardinality, inputs,
truth, active set, policy state, or seed repetitions disagree; geometry has
missing, coincident, malformed, or nonfinite positions or too few references;
a required `cvt.center` target is missing, malformed, nonfinite, or has
unrecognized semantics; a nominal fixed-reference target triangle has a
nonpositive or nonfinite side, area, strict triangle slack, or
\(\delta_i^\star\); a modeled FIM field is nonfinite or outside its domain;
free space falls below `6,000,000,000` bytes; output exceeds the allocation
cap; the output path overlaps or follows a protected input or symlink;
publication would replace an existing path; either artifact is missing,
extra, nonregular, renamed, replaced, or differs from generated bytes; or any
final source rehash differs.

A pre-publication failure leaves no authoritative output.  Any output that
fails postflight is invalid evidence.  In either case, do not retry
automatically, do not loosen a check, and do not interpret a partial result.

## Exact claim boundary and paper-edit gate

The analysis is post-hoc exploratory evidence from one truth trajectory and
20 nested range-noise seeds.  It may describe observed dynamic and fixed-pair
geometry, nominal target nondegeneracy, observed satisfaction of the
sufficient uniform target-tracking perturbation margin, modeled FIM margins,
finite-horizon absolute-error distributions, finite-depth profiles, estimator
availability, and coefficient-3 calibration only for this registered
configuration.  Observed perturbation-margin satisfaction does not make the
soft task CBF a hard tracking invariant and does not elevate explanatory
fixed-pair geometry over the complete dynamic active set.

It is not:

- a deterministic true-error bound;
- a proof that \(P_i=C_i\) or that \(P_i\) upper-bounds \(C_i\);
- an arbitrary-depth stability result;
- a guarantee of fixed-pair nondegeneracy;
- proof of independent reference errors or elimination of mirror ambiguity;
- an unconditional controller-safety or estimator-in-the-loop mission result;
- a graph-superiority result;
- a confirmatory result; or
- a cross-trajectory or noise-distribution generality result.

Do not edit the paper after execution alone.  Paper editing remains prohibited
until the analyzer output, repository evidence report, and independent
evidence review are complete, every Critical or Important review issue is
resolved, all denominators and integrity checks reconcile, and a separate
paper-edit decision concludes that a narrow theory or evidence statement is
supported.  A stronger stability or generality claim requires a new
prospectively registered confirmation design with new trajectories, new
range-noise seeds, and thresholds fixed before inspecting confirmation
results.
