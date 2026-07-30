# Predictive WNLS Stage 1 v4 terminal evidence

## Verdict

The registered Stage 1 v4 execution completed successfully,
but the complete `predictive_multistart` estimator passed only seven of the
nine prospectively frozen estimator gates.
The two failures are availability gates:

- fresh retention is `107460 / 127190 = 0.844877741960846`,
  below the frozen `0.98` threshold; and
- fresh-or-predicted availability is
  `121626 / 140000 = 0.868757...`,
  below the frozen `0.95` threshold.

Within this frozen replay,
the estimator eliminates catastrophic published errors.
Its maximum fresh and all-published errors are both
`25.851622195360374 m`,
with zero errors at or above the frozen `50 m` catastrophic threshold.
On the `107460` rows where both the legacy baseline and complete estimator
publish fresh estimates,
the error p95 improves from `9.149727355215518 m` to
`6.217761243713245 m`.

This is a scientifically adverse but useful development result:
strict qualification,
predictive multistart,
and bounded prediction suppress the `999.331896 m` stale-publication failure,
but qualification,
candidate rejection,
and the finite prediction horizon jointly convert too many downstream rows
into unavailable outputs.
No threshold was changed after observing the result,
and no registered command was rerun.
`all_estimator_gates_passed == false` and the paper-edit gate remains
`CLOSED`.

## Frozen identity

The registered protocol is
`cbf2026-predictive-wnls-stage1-v4`.
Its frozen file identities are:

| Artifact | SHA-256 |
| --- | --- |
| protocol JSON | `09b236978e2e21bac226dc3d00c36a2ee5cdf5fea0e616f2c1d875b029d4e4e0` |
| protocol Markdown | `78acce325749a96fbe1ae8c79245e4e73eaac66773998940cf2291726f1854f0` |
| unchanged legacy replay source | `0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8` |

The corresponding Git commit OIDs are:

| Role | Git commit OID |
| --- | --- |
| implementation parent | `68a9a0c1c85abc33ce85b9b846d43b139a1d5ebf` |
| protocol commit | `1ed3fc0557c07e33bbf0ed5e3f963702edaf4b00` |
| preflight commit | `4608a315b904803d8e6bb54af800e79d285f44b4` |

The registered replay manifest binds the estimator,
replay,
analyzer,
legacy solver,
truth trajectory,
input manifest,
baseline stream,
and diagnostic-integrity source.
The analyzer manifest independently binds the registered replay manifest and
raw stream by path,
device,
inode,
size,
mtime,
and SHA-256.

## Execution closure

The execution ledger records that the two deterministic smoke invocations
were each executed exactly once.
Both completed `84 / 84` rows,
and their compressed and decompressed streams are byte-identical.

| Evidence root | Manifest SHA-256 | Compressed process SHA-256 | Decompressed process SHA-256 |
| --- | --- | --- | --- |
| `/private/tmp/cbf2026-predictive-wnls-smoke-v4-a` | `5dae8be4353fe034c7d2df3b69d173311a65aea6673fcd0c56206876850e20ba` | `a4f1d719637ffec46b5ad81ae64feac93399bafed7aa77c998b38e9d8d30efc8` | `72b2cadf90b657325c86ad154d22d2c8a7a27d5d7dee91a5d41adf738cc476ed` |
| `/private/tmp/cbf2026-predictive-wnls-smoke-v4-b` | `8cdfa071c5250bfb9f7c5b7ac11149390a1a783d6072a351bb564ade79a9063b` | `a4f1d719637ffec46b5ad81ae64feac93399bafed7aa77c998b38e9d8d30efc8` | `72b2cadf90b657325c86ad154d22d2c8a7a27d5d7dee91a5d41adf738cc476ed` |

The same ledger records that the registered replay and analyzer were then
each executed exactly once,
without retry.
Their lifecycle-guarded terminal roots and manifests are consistent with that
record:

| Artifact | Terminal state | Rows/bytes | SHA-256 |
| --- | --- | ---: | --- |
| replay manifest | completed | `420000 / 420000` | `123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223` |
| compressed raw process | completed | `206565170 B` logical | `9710d1a5792e682bb272d0bb3305162180d1e0b9a0636647d6e071d1782a699b` |
| decompressed raw process | completed | `420000` rows | `7047989804fd03bd4cadffac8212349b7183857623ac201d14ef2fc3e30ee2e1` |
| analyzer manifest | completed | `176128 B` allocated | `e50352825665bb94c0e3b32c7922a081f093100daf0ed8e5c8ce9e9f32bd9238` |
| compact JSON | completed | `123611 B` | `8a0a417f2d3b139fa248bef8157a3d4871fa04042aaad82ce5cd31f470a6ff1e` |
| compact Markdown | completed | `39055 B` | `986dcf72f72ab6dc3d424d0de6cdbf410cf271d174d16caef32945acc7d4e13b` |

The compressed raw process allocated `218165248 B`;
including its manifest,
the two-file raw bundle allocated `218177536 B`,
below the frozen `2 GB` raw cap.
The compact bundle allocated `176128 B`,
below the frozen `10 MB` cap.
The registered `8 GB` start and `6 GB` stop disk guard did not trigger.
All evidence roots are retained unchanged.

The replay contains the exact frozen ordered grid of three variants,
20 seeds,
500 frames,
and 14 UAVs.
All public covariance matrices and violation records satisfy the canonical
schema,
and the complete arm has zero nonfresh-anchor uses,
zero insufficient-provenance records,
and zero provenance mismatches over `140000` rows.

## Prospectively frozen gates

| Gate | Observed | Frozen threshold | Result |
| --- | ---: | ---: | --- |
| ascending-DAG violations | `0 / 140000` | `0` allowed | pass |
| current-frame provenance violations | `0 / 140000` | `0` allowed | pass |
| fresh retention among legacy-fresh rows | `107460 / 127190 = 0.844877741960846` | at least `0.98` | **fail** |
| fresh-or-predicted availability | `121626 / 140000 = 0.868757...` | at least `0.95` | **fail** |
| maximum fresh error | `25.851622195360374 m` | strictly below `50 m` | pass |
| maximum prediction age | `2` frames | at most `2` | pass |
| maximum published error | `25.851622195360374 m` | strictly below `50 m` | pass |
| paired both-fresh p95 | `9.149727355215518 -> 6.217761243713245 m`, `n=107460` | must not worsen | pass |
| qualification-anchor violations | `0 / 140000` | `0` allowed | pass |

The aggregate decision is therefore `7 / 9` gates passed.
The failed availability decisions are not replaced by the favorable
conditional error statistics.

## Complete-estimator result

The complete `predictive_multistart` arm has:

| Metric | Result |
| --- | ---: |
| attempts | `119655` accepted, `0` failed, `0` invalid, `11531` reference-unavailable, `8814` rejected |
| outputs | `119655` fresh, `1971` predicted, `18374` unavailable |
| fresh availability | `0.8546785714285714` |
| fresh-or-predicted availability | `0.8687571428571429` |
| fresh error p50 / p95 / p99 / max | `1.2223917938 / 6.2227869773 / 10.0737045459 / 25.8516221954 m` |
| all-published error p50 / p95 / p99 / max | `1.2452185671 / 6.4462213574 / 10.4854593908 / 25.8516221954 m` |
| fresh or published catastrophic errors | `0` |
| ascending, provenance, nonfresh-anchor, or reference violations | `0` |
| maximum prediction age | `2` frames |
| maximum unavailable streak | `356` consecutive frames within one seed--UAV sequence |

The maximum-unavailable-streak statistic is descriptive,
not a substitute for the two availability gates.

## Mechanism closure

The three frozen mechanism records explain what was fixed and what was
traded away:

| Frozen key | Legacy baseline | Complete estimator |
| --- | --- | --- |
| seed `20260736`, frame `44`, UAV `14` | attempt failed; stale legacy publication; `12.6308723232 m` error | accepted fresh from predictive multistart; `1.6078152779 m` error |
| seed `20260736`, frame `138`, UAV `14` | attempt failed; stale legacy publication; `999.3318962080 m` error | current attempt reference-unavailable; bounded one-frame prediction; `0.7055898408 m` error |
| seed `20260730`, frame `177`, UAV `14` | accepted fresh; `168.9016971250 m` error | reference-unavailable and unpublished |

The frame-44 record shows predictive multistart recovering a failed update;
its legacy `12.6308723232 m` output was not catastrophic under the frozen
`50 m` definition.
The frame-138 record separately shows bounded prediction preventing the
legacy `999.3318962080 m` stale publication.
At the third key,
the complete estimator reports `reference_unavailable` and withholds
publication.
Across the full registered grid,
however,
the same strict dependency propagates unavailability deeply enough to fail
both availability gates.

The ablation evidence supports this interpretation.
`prediction_expiry` retains `0.98275` fresh-or-predicted availability,
but preserves `168.901697 m` maximum error and uses a nonfresh UAV anchor in
`27423` violation facts across `17877` rows.
Of these,
`5592` rows contain multiple facts,
with at most six in one row.
`fresh_reference_qualification` removes catastrophic publications but falls
to `0.539579` fresh-or-predicted availability.
The complete arm recovers substantial availability through predictive
multistart,
but its `0.868757` rate remains below the frozen target.

The depth strata localize the cascade.
Depths one and two retain `20000 / 20000` fresh-or-predicted outputs,
depth three retains `19996 / 20000`,
but depths four through seven retain only
`18299`,
`17871`,
`13275`,
and `12185` of `20000`,
respectively.
At depth seven,
`7219` attempts short-circuit as reference-unavailable and
`1595` are rejected.
This concentration in deeper dependency layers is consistent with an
availability-propagation failure,
not a fleet-wide loss of numerical accuracy.

## Input-bound and transition audits

All `6986 / 6986` expected unique physical predecessor keys
(`499` transitions times `14` UAVs)
are present in the prediction-transition audit.
Each key is repeated across the three variants and 20 seeds in the raw stream,
and every repeated command was checked.
This closes the process-evidence coverage check;
it does not prove the paper's velocity-input premise.

The preserved trajectory has `243 / 7000` unique physical rows with a
recorded applied command component above the paper's `25 m/s` component
limit.
This input audit covers `500` frames times `14` UAVs,
is separate from the nine estimator gates,
and found the input-limit flag disabled on all `7000` source rows.
The maximum component is `35.03975016117459 m/s`,
the maximum planar norm is `35.66026271589104 m/s`,
and the maximum one-frame displacement is `17.83013135794552 m`.
These are properties of the frozen source trajectory,
not failures introduced by the estimator.
They independently prevent this Stage 1 trajectory from serving as paper
confirmation evidence.

## Scientific interpretation and next stage

Stage 1 v4 supports a narrow conclusion:
for this frozen noisy-ranging replay,
the complete estimator keeps every published localization error below
`25.852 m`,
improves conditional p95 error,
uses only ascending current-fresh anchors for fresh updates,
and never publishes the legacy `999 m` stale branch.
It does not support the stronger claim that the current policy maintains
adequate localization availability.

Because Stage 1 failed,
the frozen plan does not authorize Stage 2 confirmation.
The next work is a new development cycle targeting the availability cascade,
while preserving the already achieved error and integrity properties.
Its minimal sequence is:

1. partition unavailable rows by depth,
   time,
   attempt failure/rejection reason,
   and first upstream reference that became unavailable;
2. determine whether the dominant loss comes from estimator rejection,
   the two-frame publication horizon,
   or the rule that only current-frame fresh UAVs may anchor a fresh update;
3. evaluate candidate availability-recovery policies in explicitly labeled
   development evidence,
   such as allowing a bounded-age predicted reference with explicit
   covariance propagation and provenance,
   without relabeling that reference as fresh; and
4. freeze one selected policy and all thresholds before any new registered
   confirmation evidence.

The predicted-reference example is not authorized by the current approved
architecture,
which forbids predicted or unavailable optional UAV anchors.
It would be a new development design and would require explicit theoretical,
integrity-contract,
and protocol review before implementation.
In particular,
it could not silently reuse the current qualification-anchor gate;
a replacement integrity gate would have to be defined and approved before
observing confirmation outcomes.

Only after a future development gate passes may a separate Stage 2 plan be
written.
Under the already approved architecture,
confirmation retains all nine Stage 1 gates,
including both failed availability gates and the qualification-anchor gate,
and requires
12 independent `250 s` truth trajectories,
five nested predeclared range-noise seeds per trajectory,
enabled `25 m/s` componentwise input limits,
per-row command/state-transition audits,
fixed-cohort comparisons,
and two-sided 95% Wilson trajectory-incidence intervals.
No Stage 2 trajectory is generated by this Stage 1 closeout.
Until then,
the Stage 1 v4 evidence is development evidence only and `main.tex` should
not be changed to claim successful estimator validation.
