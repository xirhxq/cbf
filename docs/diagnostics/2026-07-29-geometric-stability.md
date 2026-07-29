# Post-hoc geometric-stability exploratory evidence

## Status and claim boundary

The registered exploratory analyzer completed once and published the two
authoritative artifacts listed below.  The structured artifact reports
`schema = cbf2026-geometric-stability-v1` and `status = completed`.
This is post-hoc descriptive evidence from one truth trajectory with 20
range-noise seeds nested in that trajectory; it is not a confirmatory result.
Rows, frames, UAVs, and reference edges are correlated repeated observations,
not independent replicates.

The exact registered claim boundary is recorded at `claim_boundary`: this is
not a deterministic true-error bound, not an arbitrary-depth stability result,
not an unconditional controller-safety result, and not a cross-trajectory
generality result.  No \(p\)-value or confirmatory threshold is reported here.

## Source and output identity

The launch repository commit is
`fdbe66dce59691e248ad491160a0b13325d78013`
(`source.source_commit`).  The reviewed analyzer implementation is commit
`52100827b09434e91b1ded140d7d786bc817a981`; the launch-time analyzer bytes
were unchanged from that implementation commit.

| Identity | Absolute path or value | SHA-256 / JSON key |
| --- | --- | --- |
| Analyzer | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/analyze_geometric_stability.py` | `6240d1488f92c259b6b545e987746e151808709c10e377d5284ba02d569078d8` (`source.analyzer_path`, `source.analyzer_sha256`) |
| Registered comparison | `/private/tmp/cbf2026-warm-start-recovery-analysis/registered-gate-2/warm-start-recovery-comparison.json` | `ef8014915b3cced6a75e094b3d16f01f17b5f881c6357dff6b885b6e871498ca` (`source.comparison_path`, `source.comparison_sha256`) |
| Paired parent manifest | `/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/manifest.json` | `c110275910c3e44ecf97c66ba7346fbb0b340cfb844d050c0ecf56944e5ff55a` (`source.parent_manifest_sha256`) |
| Truth trajectory | `/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json` | `3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527` (`source.input_data_path`, `source.input_data_sha256`) |
| Strict bundle | `/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/strict/localization-calibration/20260729T112154.899215Z_b6f8a00b032541329a6e975d402f7d49` | `source.strict_bundle`; child hashes are frozen in the approved protocol and transitively pinned by the parent and comparison |
| Restart bundle | `/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/restart/localization-calibration/20260729T112437.413270Z_efb8e7137776415c833f11204cf519d5` | `source.restart_bundle`; child hashes are frozen in the approved protocol and transitively pinned by the parent and comparison |
| Authoritative JSON | `/private/tmp/cbf2026-geometric-stability-analysis/exploratory-v1/geometric-stability.json` | `a02098e231c3b45b65f6213d6708dcae2fb68d9cbcc6e52103b0e8c6f013d54b` |
| Authoritative Markdown | `/private/tmp/cbf2026-geometric-stability-analysis/exploratory-v1/geometric-stability.md` | `1e60aac9eb7dd8f9a2e1ad89ed25614382fa8b7fdfe01c980ae12bb21d97982a` |

The output directory contains exactly those two regular files and allocates
1,380 KiB, or 1,413,120 bytes, which is strictly below the registered
10,000,000-byte acceptance cap.  Postflight free space was 69,750,852 KiB,
or 71,424,872,448 bytes.  The postflight source hashes were unchanged.

The paired integrity record reports 280,000 paired rows, equal paired inputs,
and unchanged source hashes (`integrity.paired_rows`,
`integrity.paired_inputs_equal`, and
`integrity.source_hashes_unchanged`).  The registered statistical scope is
one truth trajectory and 20 nested range-noise seeds
(`protocol.truth_trajectory_count`,
`protocol.range_noise_seed_count`, and `protocol.statistical_unit`).

## Geometry and modeled FIM

The dynamic active set is the primary geometry.  The nominal, true, and
estimated fixed-pair families are explanatory.  For the trajectory-level
families, 6,986 unique `(frame_index, robot_id)` tuples were counted once and
their 20 seed repetitions were used only for integrity
(`geometry.*_all_primary.overall.trajectory_tuple_count` and
`range_seed_repetitions_verified`).

The registered summaries publish the distribution of the minimum eigenvalue
\(\lambda_{\min}\), not the sample minimum of that distribution.  Therefore
the table preserves the published median, p90, p95, p99, and maximum exactly;
no unregistered sample-minimum statistic was reconstructed.

| Family and exact JSON metric key | Denominator | Median | p90 | p95 | p99 | Maximum |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| True dynamic `geometry.true_dynamic_all_primary.overall.metrics.lambda_min` | 6986 | 0.4841966735867322 | 1.2091327975976425 | 1.5296391589548737 | 2.7112435940726844 | 3.463108663778235 |
| True dynamic `geometry.true_dynamic_all_primary.overall.metrics.normalized_lambda_min` | 6986 | 0.22082189351171155 | 0.5339453794549619 | 0.6053764155865793 | 0.7260616254838302 | 0.9928527704799202 |
| Nominal fixed pair `geometry.nominal_fixed_pair_all_primary.overall.metrics.lambda_min` | 6986 | 0.4999999999999999 | 0.5000000000000001 | 0.5000000000000002 | 0.7447764615055965 | 0.9993587372496214 |
| Nominal fixed pair `geometry.nominal_fixed_pair_all_primary.overall.metrics.normalized_lambda_min` | 6986 | 0.33333333333333326 | 0.3333333333333335 | 0.33333333333333354 | 0.5933439831844826 | 0.9987182964080136 |
| True fixed pair `geometry.true_fixed_pair_all_primary.overall.metrics.lambda_min` | 6986 | 0.3634707069260106 | 0.7474627044881432 | 0.8229299980529101 | 0.9744971298826463 | 0.9995868079296151 |
| True fixed pair `geometry.true_fixed_pair_all_primary.overall.metrics.normalized_lambda_min` | 6986 | 0.22209851505753286 | 0.5967588546787472 | 0.6991342896428732 | 0.9502627141059128 | 0.9991739571735758 |

The true dynamic active-reference count had median 3, p90 5, p95 6, p99 9,
and maximum 9 over denominator 6,986
(`geometry.true_dynamic_all_primary.overall.metrics.reference_count`).
The following explanatory fixed-pair quantities retain their published units:

| Family and exact JSON metric key | Denominator | Median | p90 | p95 | p99 | Maximum |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Nominal included angle, rad, `geometry.nominal_fixed_pair_all_primary.overall.metrics.included_angle_rad` | 6986 | 1.0471975511965976 | 1.0471975511965979 | 1.047197551196598 | 1.4199475084617053 | 2.498091544796509 |
| Nominal angle to collinearity, rad, `geometry.nominal_fixed_pair_all_primary.overall.metrics.noncollinearity_angle_rad` | 6986 | 1.0471975511965976 | 1.0471975511965979 | 1.047197551196598 | 1.3127171181782746 | 1.5701550640005684 |
| Nominal twice-triangle area, m\(^2\), `geometry.nominal_fixed_pair_all_primary.overall.metrics.twice_triangle_area` | 6986 | 231412.81320874934 | 519701.8448110416 | 552999.16842186 | 610731.9400663334 | 622001.0956330785 |
| Nominal minimum triangle slack, m, `geometry.nominal_fixed_pair_all_primary.overall.metrics.minimum_triangle_slack` | 6986 | 435.66041821583923 | 766.2285886078643 | 789.149067033599 | 834.8970385476875 | 847.4815632212892 |
| Nominal admissible tracking-deviation supremum, m, `geometry.nominal_fixed_pair_all_primary.overall.metrics.admissible_tracking_deviation_supremum` | 6986 | 72.61006970263988 | 127.70476476797738 | 131.52484450559982 | 139.14950642461457 | 141.2469272035482 |
| True included angle, rad, `geometry.true_fixed_pair_all_primary.overall.metrics.included_angle_rad` | 6986 | 0.9162272241463605 | 1.3909093420700116 | 1.7305817544062454 | 2.4840719045563397 | 2.5671130599760716 |
| True angle to collinearity, rad, `geometry.true_fixed_pair_all_primary.overall.metrics.noncollinearity_angle_rad` | 6986 | 0.8808065382746992 | 1.315494672073482 | 1.3927877180111357 | 1.5452906913000446 | 1.5703831347127544 |
| True twice-triangle area, m\(^2\), `geometry.true_fixed_pair_all_primary.overall.metrics.twice_triangle_area` | 6986 | 215158.6202782381 | 319553.9605815719 | 352059.87218863016 | 409577.4616011726 | 428052.35928825923 |

Estimated dynamic geometry, estimated fixed-pair geometry, estimated
fixed-pair tracking margin, and modeled FIM each have denominator 139,617.
Each records exactly 103 inapplicable primary rows with reason
`restart_row_not_finite`
(`geometry.estimated_dynamic_finite.overall`,
`geometry.estimated_fixed_pair_finite.overall`,
`geometry.estimated_fixed_pair_tracking_margin_finite.overall`, and
`geometry.modeled_fim_valid.overall`).

| Estimated/model metric and exact JSON key | Denominator | Median | p90 | p95 | p99 | Maximum |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Estimated dynamic \(\lambda_{\min}\), `geometry.estimated_dynamic_finite.overall.metrics.lambda_min` | 139617 | 0.48319361389513055 | 1.1942524885254366 | 1.563602551137271 | 2.7507535229535067 | 4.3216016401580815 |
| Estimated fixed-pair \(\lambda_{\min}\), `geometry.estimated_fixed_pair_finite.overall.metrics.lambda_min` | 139617 | 0.3593668962293606 | 0.7478169644426154 | 0.8227727529005752 | 0.9720004732606761 | 0.9999555049061628 |
| Modeled \(\lambda_{\min}(\Phi_i)\), `geometry.modeled_fim_valid.overall.metrics.phi_min_eigenvalue` | 139617 | 0.2851635018322036 | 2.206380478390097 | 4.166406989289898 | 6.7326632666202855 | 9.088990638476844 |
| Modeled FIM condition number, `geometry.modeled_fim_valid.overall.metrics.phi_condition` | 139617 | 9.440985951235982 | 33.42717103544151 | 47.570451935278065 | 221.7683656917698 | 806.8836155465261 |
| Modeled \(\epsilon_i\), m, `geometry.modeled_fim_valid.overall.metrics.epsilon` | 139617 | 5.617903631182234 | 12.881883754249941 | 15.557422144966925 | 24.719725439892073 | 31.43763748077591 |

For the sufficient nominal tracking perturbation certificate, 32,969 of
139,617 finite rows were strictly within the admissible deviation and 106,648
were not strictly within it
(`geometry.estimated_fixed_pair_tracking_margin_finite.overall.strictly_within_admissible_count`
and `not_strictly_within_admissible_count`).  Its signed-margin median was
-152.5515537436229 m, p90 36.0482952690363 m, p95
56.323480375562944 m, p99 77.16607562005242 m, and maximum
93.43889174367648 m
(`geometry.estimated_fixed_pair_tracking_margin_finite.overall.metrics.tracking_deviation_margin`).
This does not establish a hard tracking invariant.

The modeled-FIM valid-row contract admits only positive
\(\lambda_{\min}(\Phi_i)\); the completed artifact therefore records no
modeled-FIM singularity among its 139,617 valid rows.  Successful nominal
fixed-pair processing also establishes strict nondegeneracy of every admitted
nominal target triangle.  The compact JSON does not publish a sample minimum
or positive-definite count for true or estimated geometry, so these aggregates
do not independently exclude isolated exact geometry singularities.  A
stronger “no geometry singularity” statement must await the independent raw
evidence audit.

## Absolute error

Every value in this section is in metres.  The overall finite-error
denominator was 139,617; 103 rows were inapplicable because
`error_norm_not_finite`.  Overall median, p90, p95, p99, and maximum were
1.8959195197213623, 11.455705930383873, 16.980039636508756,
51.61642201639948, and 999.3318962079554, respectively
(`absolute_error.overall`).

### By squad-local index

| Index (`absolute_error.by_depth`) | Denominator | Nonfinite | Median | p90 | p95 | p99 | Maximum |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 19960 | 0 | 0.7055289471565742 | 2.12374582496989 | 3.343598192632282 | 12.288126571219308 | 38.31531373802089 |
| 2 | 19959 | 1 | 1.0248239359573408 | 4.161076162773056 | 7.1483353375868655 | 14.583788481560052 | 62.97392601824971 |
| 3 | 19958 | 2 | 1.6606713209315997 | 7.004888348937779 | 12.127033179224426 | 25.251659561728516 | 82.44019486498244 |
| 4 | 19951 | 9 | 2.257763826017265 | 10.008839936271189 | 16.910740128635553 | 40.21286582497456 | 150.1971872261691 |
| 5 | 19945 | 15 | 2.9787158369976647 | 12.658429258325786 | 22.109025949323975 | 50.85251996784194 | 199.1797513231246 |
| 6 | 19930 | 30 | 3.736517764300717 | 16.10942974547003 | 30.969243791220286 | 136.8265314435704 | 732.9502528453658 |
| 7 | 19914 | 46 | 4.46336568146059 | 17.98836212916327 | 32.97443745072205 | 151.4278111195968 | 999.3318962079554 |

The requested depth-seven p95 is exactly 32.97443745072205 m
(`absolute_error.by_depth.7.p95`).  The full finite-error maximum is exactly
999.3318962079554 m (`absolute_error.overall.maximum`), and it occurs in the
depth-seven stratum as its reported maximum.

### By squad

| Squad (`absolute_error.by_squad`) | Denominator | Nonfinite | Median | p90 | p95 | p99 | Maximum |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 69791 | 69 | 1.839205036995861 | 9.275399593653221 | 12.927678100439222 | 26.56748901671183 | 134.16779351507844 |
| 2 | 69826 | 34 | 1.952194959953545 | 13.091004640629837 | 25.100888892408538 | 85.90210424225543 | 999.3318962079554 |

### By registered 50-second bin

| Bin (`absolute_error.by_time_bin`) | Denominator | Nonfinite | Median | p90 | p95 | p99 | Maximum |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `0_to_lt_50_s` | 27617 | 103 | 1.634536274174411 | 25.08468406381538 | 41.60464993389576 | 175.17409501313952 | 637.7953229755071 |
| `50_to_lt_100_s` | 28000 | 0 | 1.9161480447698218 | 8.679330255620778 | 13.066680826592846 | 36.76870429566077 | 999.3318962079554 |
| `100_to_lt_150_s` | 28000 | 0 | 1.7785513045918462 | 8.599138627050854 | 12.966130800455778 | 29.18797672287471 | 130.44281107928043 |
| `150_to_lt_200_s` | 28000 | 0 | 1.944807568456142 | 7.465846797927907 | 10.241668697907537 | 17.000299794540663 | 50.31536510769437 |
| `200_to_250_s` | 28000 | 0 | 2.0988643196780212 | 10.341471914912084 | 16.36699496600941 | 37.82111920983583 | 134.16779351507844 |

The chronological medians, upper quantiles, and maxima do not show sustained
monotone growth: the median decreases in the third bin; p90 and p95 decline
through the fourth bin before increasing in the fifth; and the maximum peaks
in the second bin before dropping.  This is a finite-horizon descriptive
observation, not a stability proof.

## Availability and outages

Overall, the exact finite-retained availability rate is the fraction
139,617/139,720 (`availability.overall.finite_retained_rows` /
`primary_attempts`).  The converged-attempt rate is 127,043/139,720,
while the mutually exclusive attempt statuses are 127,043 converged, 12,620
failed, and 57 invalid (`availability.overall.attempt_status_counts`).
There were 12,574 stale retained rows and 103 pre-acquisition attempts
(`availability.overall.stale_retained_rows` and
`pre_acquisition_attempts`).  Fractions are retained exactly rather than
rounded into percentages.

| Index (`availability.by_depth`) | Finite-retained rate | Converged-attempt rate | Failed | Invalid | Pre-acquisition | Stale retained |
| ---: | --- | --- | ---: | ---: | ---: | ---: |
| 1 | 19960/19960 | 18402/19960 | 1558 | 0 | 0 | 1558 |
| 2 | 19959/19960 | 18166/19960 | 1794 | 0 | 1 | 1793 |
| 3 | 19958/19960 | 18537/19960 | 1422 | 1 | 2 | 1421 |
| 4 | 19951/19960 | 18453/19960 | 1505 | 2 | 9 | 1498 |
| 5 | 19945/19960 | 18266/19960 | 1685 | 9 | 15 | 1679 |
| 6 | 19930/19960 | 17827/19960 | 2118 | 15 | 30 | 2103 |
| 7 | 19914/19960 | 17392/19960 | 2538 | 30 | 46 | 2522 |

The longest consecutive unavailable duration is 95 frames, and the longest
consecutive nonfinite-retained duration is 7 frames
(`availability.overall.maximum_consecutive_unavailable_frames` and
`maximum_consecutive_nonfinite_retained_frames`).  First-finite acquisition
was achieved by all 280 expected seed-by-robot series, with zero
never-acquired series (`availability.first_finite_acquisition`).  The exact
first-finite frame counts were 147 at frame 0, 79 at frame 1, 24 at frame 2,
23 at frame 3, 3 at frame 4, 2 at frame 6, and 2 at frame 8.

## Containment and \(q>9\)

Coefficient-3 containment and the recomputed \(q\) diagnostic use separately
named denominators and are not treated as interchangeable:

- containment was 115,733/127,043
  (`calibration.overall.contained` /
  `calibration.overall.converged_error_denominator`);
- \(q>9\) was 15,074/127,043
  (`calibration.overall.q_above_9` /
  `calibration.overall.finite_q_denominator`); and
- 12,677 rows were inapplicable because
  `attempt_not_converged_or_error_missing`
  (`calibration.overall.inapplicability_reasons`).

The exact \(q\) median, p90, p95, p99, and maximum were
1.8047086851386034, 11.020668814654849, 30.443204564627933,
221.74232335252387, and 8868.484143257352, respectively
(`calibration.overall.q`).  These are calibration diagnostics, not
deterministic bounds or hypothesis tests.

## Provisional paper-update assessment

Subject to independent reproduction of the raw evidence, the narrowest
potential update is a conditional finite-depth statement: for this registered
configuration, the observed dynamic active-set geometry and admitted modeled
FIM rows are compatible with the finite-DAG theory, while the finite-horizon
error bins show no sustained monotone temporal growth.  Any paper text would
need to retain the depth-seven p95 and full maximum, availability/outage
limits, separate containment and \(q>9\) denominators, and the fact that the
strict nominal tracking-deviation certificate held for only 32,969 of 139,617
finite rows.

The paper-edit gate remains **CLOSED**.  This execution and author report do
not authorize a paper change.  An independent evidence audit must first hash
and reproduce the registered families from the raw strict/restart streams,
resolve every Critical or Important issue, and issue a separate paper-edit
decision.  Even after such review, this one-trajectory, post-hoc result cannot
support deterministic containment, arbitrary-depth stability, unconditional
safety, or cross-trajectory generality claims.
