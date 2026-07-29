# Warm-Start Recovery Gate 2 Independent Evidence Review

## Verdict

`APPROVED / CLEAN`

There are zero open Critical findings,
zero open Important findings,
and zero open Minor findings.
The two failed dynamic calibration safeguards are the correctly reproduced
scientific outcome of the registered experiment,
not evidence-integrity defects.

The dynamic exact-direct primary gate passes,
and its hierarchical downstream-unavailability secondary passes.
However,
the dynamic coefficient-3 epsilon-containment safeguard and the dynamic
\(q>9\) safeguard both fail.
Consequently,
`advance_to_multi_trajectory == false`,
and this result does not authorize an edit to `main.tex`.

## Independent method

The review used a fresh,
standalone,
read-only script at
`/private/tmp/task7_independent_audit.py`.
It imports no production diagnostic or comparator module.
It was executed from the diagnostic worktree with:

```bash
conda run -n cbf_env python /private/tmp/task7_independent_audit.py
```

The script used only Python standard-library readers and NumPy for the
prospectively specified percentile and RNG operations.
It read the baseline,
strict,
and restart gzip JSONL streams directly,
validated their expected ordered grid,
and recomputed the statistical records from row data.
The source snapshot was inspected in place through `tarfile`;
it was not extracted.

The three streams each contained exactly `280,000` JSON objects.
The audit therefore read `840,000` process rows and performed:

- `280,000` type-exact baseline-to-strict normalized comparisons; and
- `280,000` strict-to-restart paired-input comparisons.

This is `560,000` independently executed paired comparisons.
The baseline-to-strict normalization removed only
`initialization_policy`,
`initial_estimate_source`,
and `ever_acquired_finite_before_attempt`.
Every remaining field was equal in type and value.

For each strict/restart pair,
the audit type-exactly compared the ordered key
`(seed, graph_case, frame_index, robot_id)`,
the squad-local index,
truth,
and active references.
It also compared the range-noise identity
`(kind, id, true_range, noise, noisy_range)` for every active edge.
The policy-dependent
`estimated_reference_available`
field was not treated as noise identity;
it was instead independently reconciled against the pre-WNLS outcome and
restart-source provenance.
All active UAV references were independently confirmed to be lower-index,
same-squad references.

The audit required the exact ordered grid of
`500` frames,
`20` seeds,
the two registered graph cases,
and UAV IDs `1` through `14`.
It also checked that only frame zero is excluded from primary statistics,
giving exactly `6,986` primary rows per seed and graph case and
`139,720` primary rows per graph case and policy.

## Identity and hash closure

The executor's one-time handoff parent was used literally:

```text
/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b
```

Its independently observed manifest SHA-256 is
`c110275910c3e44ecf97c66ba7346fbb0b340cfb844d050c0ecf56944e5ff55a`,
exactly matching the executor handoff and comparison trust root.
The parent schema is
`cbf2026-warm-start-recovery-parent-v1`,
its termination reason is `completed`,
and its source tuple is commit
`12ca41cd9f403bef2182e57e980329829c16c4bc`
on `codex/cbf2026-diagnostic`.

The independently observed artifact hashes are:

| Bundle | Manifest | Summary JSON | Summary Markdown | Compressed JSONL | Decompressed JSONL |
| --- | --- | --- | --- | --- | --- |
| immutable baseline | `39ee3b5494ed623d20d16766244427476cd081ed0869feecb9e5072c6c8f481d` | `f4f780dd4c9c1399d3f838c74054c9c0ba3159415bc13089aee53d0ebf7250bc` | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` | `38325523e395983d83491c81556bac1aae72ebd99508b3cdf148fdf72f3f8803` | `2c8a3f0f5684a281d5fbef3d7facea16f45d757d96b458827928dcac9e241a4f` |
| strict child | `96eb398836320594e8f1ec82d8511273b48eb711f8a76de0aab89711c1a85d3a` | `e088ec11d1f5d87a33be89ac23db2d5548e8070a14636187575f629dd6b88c66` | `6289376563a5a76ccf5ba38194fe1900be9d50baab085e170df275f9e1cca9a9` | `e84e43c332aba21664b63f52281416d47c21be684d2b5b83e26edca233efe67b` | `a70a5d3bc676cbe5ff26eef347e9bafd76410b34879b4c27e9b1934b7bb738b0` |
| restart child | `5c33539a4030393ae5ca02939bbe204fdbf7f322adc2b874deacaf8364d99f18` | `6edcf53b0fcbc6cb03f0319ede1ee5ed2c4e7ab5ce42274c593927aed0064f65` | `33217f673ee08ed888dd6c7a603f4451d46c2e7c26f32941b4f8c754d4e9464e` | `c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003` | `a55e1ac5167e089e7f72252fb1f9351e483b1e11b9b1821c26c9b9fd81bc14f2` |

All three manifests are completed,
all three summary row counts equal `280,000`,
and each summary's settings equal its manifest settings.
Both children use
`cbf2026-localization-calibration-v3`,
the estimator contract
`variable_weight_nls_full_residual_jacobian_v1`,
the exact registered seeds and graph cases,
and the exact explicit policy assigned to that arm.

The parent source-snapshot SHA-256 is
`4ac5ca7d3608dcce4c54574ae33dd04b0683069e22bd344a34264d3dda3ef5cf`.
The following hashes were independently equal in three places where
applicable:
the frozen registration,
the live worktree,
and the corresponding regular-file member inside that source snapshot.

| Trust root | SHA-256 |
| --- | --- |
| supervisor | `76733bb0748a63a63858ea24fc89240998f0dc1c58dbe96888da86d3c31e9ae5` |
| replay | `0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8` |
| shared disk/snapshot helper | `ccde94fa739649cb9b94a8304ef3867ae946f28b0e3a812d3feb3f30bb76fd23` |
| comparator | `9a54c312183f7f2864aba104a5624ea247e15e1107deaa4590e276ab1eda120d` |
| failure analyzer | `1e3a7b7cd615b258fb83af53f7264d7aff327685f42c85787d0d9ae7df0b6315` |

The trajectory data hash is
`3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527`,
and the trajectory-manifest hash is
`6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb`.
The trajectory manifest is completed and independently retains source commit
`cff3852831f969ad4bedf7d9b2f43b4f2a0cf42f`,
source-snapshot hash
`7edf5cad0ef182ac7693ce4dadf3bb6e9685facc701e99558975623c31bc066d`,
and materialized-configuration hash
`cb13a87615da21e2f52ab41d039f41c4547e8d7311f1f2b791afe8474d10153d`.

For completeness,
the published comparison JSON and Markdown hashes are respectively
`ef8014915b3cced6a75e094b3d16f01f17b5f881c6357dff6b885b6e871498ca`
and
`e10ba2c137a5edc229e9fa9492b1992d0651f38d8df9067c47b295b4e8d5cc33`.

## Independently recomputed paired outcomes

The seed order in every vector below is exactly
`20260727` through `20260746`.
Every seed denominator is `6,986`.

For the dynamic exact-direct event,
the strict per-seed counts are:

```text
[998, 998, 998, 998, 499, 499, 998, 998, 0, 998,
  998, 998, 998, 998, 499, 998, 499, 499, 0, 998]
```

All restart counts are zero.
The aggregate therefore changes from
`15,469 / 139,720`
to
`0 / 139,720`,
with count reduction `1.0`.
Using exactly `10,000` paired-seed resamples,
RNG seed `20260729`,
and `20` draws per resample,
the independently reproduced percentile interval is
`[-0.1285714285714286, -0.08928571428571427]`.
Its upper endpoint is strictly below zero,
and the count reduction exceeds `0.90`;
the primary gate passes.

For dynamic upstream unavailability,
the strict per-seed counts are:

```text
[3493, 2994, 5489, 3992, 2994, 499, 998, 2994, 0, 5988,
  4990, 1996, 3992, 1497, 1996, 998, 1497, 499, 0, 3992]
```

The restart per-seed counts are:

```text
[0, 0, 1, 6, 0, 1, 7, 0, 0, 12,
  10, 4, 7, 1, 0, 1, 3, 1, 0, 3]
```

The aggregate changes from
`50,898 / 139,720`
to
`57 / 139,720`,
with count reduction
`0.9988801131675115`.
The independently reproduced percentile interval is
`[-0.47458613655883203, -0.25334239908388206]`.
Because the primary gate is open and this upper endpoint is strictly below
zero,
the hierarchical cascade-interruption secondary passes.

The fixed-reference arm is descriptive only.
Its exact-direct event changes from
`14,471 / 139,720`
to zero,
with interval
`[-0.12142857142857144, -0.0857142857142857]`.
Its strict per-seed exact-direct counts are:

```text
[499, 499, 499, 998, 998, 998, 998, 499, 998, 499,
  0, 499, 499, 499, 998, 998, 998, 499, 998, 998]
```

Its upstream-unavailability event changes from
`42,914 / 139,720`
to
`91 / 139,720`,
with count reduction
`0.9978794798900126`
and interval
`[-0.3704983180647009, -0.24235614085313487]`.
The strict and restart per-seed upstream counts are:

```text
strict:  [1996, 1996, 1996, 3493, 3992, 2495, 3493, 1497, 2495, 0,
          0, 1996, 1996, 1996, 2994, 1996, 3493, 1996, 1497, 1497]
restart: [4, 4, 0, 10, 4, 8, 6, 3, 4, 0,
          0, 0, 11, 11, 11, 1, 10, 4, 0, 0]
```

The broader non-upstream invalid component exactly equals the exact-direct
event in both graph cases.
It remains a component check and does not replace the registered primary
event.

## Calibration and restart-provenance recomputation

The audit recomputed each normalized squared error as
\(e^\mathsf{T}P^{-1}e\)
after independent finite,
symmetry,
and positive-definiteness checks.
Containment was recomputed from
`error_norm <= epsilon`,
and error-to-epsilon ratios were checked against
`error_norm / epsilon`
before independent percentile calculation.

| Graph/policy | Converged | WNLS nonconvergence | 3-epsilon contained | \(q>5.991464547\) | \(q>9\) |
| --- | ---: | ---: | ---: | ---: | ---: |
| dynamic strict | 67,042 | 6,311 | 62,516 (`0.9324900808448435`) | 9,488 (`0.14152322424748665`) | 5,986 (`0.0892873124310134`) |
| dynamic restart | 127,043 | 12,620 | 115,733 (`0.9109750242044032`) | 22,364 (`0.1760348858260589`) | 15,074 (`0.11865273962359202`) |
| fixed strict | 82,151 | 184 | 41,913 (`0.5101946415746613`) | 42,871 (`0.521856094265438`) | 41,215 (`0.5016980925369138`) |
| fixed restart | 139,328 | 301 | 46,334 (`0.3325533991731741`) | 96,945 (`0.6958041456132292`) | 95,008 (`0.6819016995865871`) |

The independently reproduced
`(median, p90, p95, p99, maximum)`
error-to-epsilon-ratio vectors are:

```text
dynamic strict:
(0.29854651864236326, 0.8005066799752972, 1.2187912530118385,
 3.7554040715420656, 26.363081543576957)

dynamic restart:
(0.3153698568093028, 0.929689554923014, 1.4975339360269129,
 4.24918491412413, 28.471227902772508)

fixed strict:
(0.7938638334983859, 166.58600685827017, 211.29916838998398,
 242.4866786265627, 332.33720984131236)

fixed restart:
(69.65694716166999, 184.67407117221182, 216.33343584898256,
 248.2608063896986, 333.409960690343)
```

The independently reproduced maximum streaks
`(exact-direct, upstream-unavailable, WNLS-nonconvergence)`
are:

```text
dynamic strict:  (499, 499, 34)
dynamic restart: (0, 7, 95)
fixed strict:    (499, 499, 5)
fixed restart:   (0, 3, 5)
```

Restart-source provenance reconciles exactly.
Across both graph cases,
the `280,000` rows contain
`560` deployment-frame-zero selections,
`476` deployment-restart-before-first-finite selections,
`278,964` previous-finite selections,
and zero strict-previous-missing selections.
The `476` restart selections split into
`328` valid-input WNLS attempts and
`148` upstream-unavailable short circuits.
The valid-input attempts split into
`248` convergences and
`80` maximum-iteration failures.

For dynamic DAG rows,
the corresponding counts are
`236` restart selections,
`179` valid-input attempts,
`57` upstream short circuits,
`133` convergences,
and `46` maximum-iteration failures.
For fixed-reference rows,
they are
`240`,
`149`,
`91`,
`115`,
and `34`,
respectively.

## Safeguards, advancement, and claim boundary

The dynamic conditional coefficient-3 epsilon-containment change is
`-2.151505664044029` percentage points.
This is below the registered
`-2`-percentage-point floor,
so the containment safeguard fails.

The dynamic conditional \(q>9\) change is
`+2.936542719257862` percentage points.
This exceeds the registered
`+2`-percentage-point ceiling,
so the \(q>9\) safeguard fails.

Therefore:

| Decision | Independent result |
| --- | --- |
| dynamic exact-direct primary gate | pass |
| hierarchical dynamic upstream secondary | pass |
| dynamic 3-epsilon containment safeguard | fail |
| dynamic \(q>9\) safeguard | fail |
| advance to separately registered multi-trajectory stage | no |
| modify `main.tex` from this result | no |

The evidence supports only this narrow diagnostic statement:
under the preserved single trajectory and exact frozen paired range-noise
seeds,
deployment restart reduces the registered local initialization-failure
event,
and the hierarchical paired result reduces observed downstream
unavailability on that trajectory.

It does not support graph superiority,
coefficient-3 epsilon sufficiency,
production-estimator robustness,
cross-trajectory or cross-geometry generalization,
or controller,
CBF,
collision-avoidance,
connectivity,
mission-success,
or safety guarantees.
The failed calibration safeguards are specifically why the recovery result
must not be promoted into the paper as a validation claim.

## Disk and publication audit

The comparison publication directory contains exactly the two registered
regular files:

```text
warm-start-recovery-comparison.json
warm-start-recovery-comparison.md
```

Their directory tree uses `57,344` allocated bytes,
below the `10,000,000`-byte cap.
The paired parent and the entire replay root each use
`134,279,168` allocated bytes,
below the respective
`250,000,000`-byte parent cap and
`2,000,000,000`-byte root cap.
The parent manifest's independently checked allocation fields equal these
filesystem measurements.

At the independent audit probe,
`/private/tmp` had
`72,854,937,600` available bytes.
The parent records a minimum live free-space observation of
`7,363,014,656` bytes,
which remains above the registered
`6,000,000,000`-byte live floor.
No evidence file,
source file,
Git object,
or `build-diagnostic/` path was changed or removed during this review.

## Limitations

This review validates the frozen artifact identities,
pairing,
statistics,
publication set,
and registered decision logic.
It does not rerun the estimator,
trajectory generator,
controller,
or CBF simulation.
The scientific evidence remains limited to one preserved trajectory,
20 paired noise seeds,
and an offline estimator outside the control loop.
