# Predictive WNLS Stage 1 v4 independent evidence review

## Verdict

`APPROVED / CLEAN`

There are zero open Critical findings,
zero open Important findings,
and zero open Minor findings.
The evidence and analyzer contain no bug that changes the scientific verdict.
The complete estimator passes seven of nine prospectively frozen gates;
fresh retention and fresh-or-predicted availability fail.
`all_estimator_gates_passed == false`,
and the paper gate is correctly `CLOSED`.

The two failures are an observed accuracy--availability tradeoff,
not an evidence-integrity defect.
The favorable published-error tail applies to `121626` available outputs and
does not erase the `18374` unavailable rows.
Likewise,
the paired p95 improvement is conditional on the `107460` both-fresh
survivors and does not cover the `19730` baseline-fresh rows excluded from
that cohort.

## Independent method

Two read-only audits inspected the preserved replay and analysis roots
without modifying evidence,
rerunning a registered command,
or changing a threshold.
They directly read the compressed 420000-row process stream,
the compact JSON and Markdown,
both terminal manifests,
the frozen protocol,
and all manifest-bound inputs and sources.

The audits independently checked:

- 22 path/device/inode/mtime/size/SHA identity records;
- the exact three-variant,
  20-seed,
  500-frame,
  14-UAV ordered grid;
- all 420000 strict row keys,
  from
  `(prediction_expiry, 20260727, 0, 1)` through
  `(predictive_multistart, 20260746, 499, 14)`;
- all nine gate numerators,
  denominators,
  extrema,
  percentiles,
  and Boolean decisions;
- all three frozen mechanism records;
- canonical covariance,
  reference,
  provenance,
  DAG,
  and violation semantics;
- all 419160 non-frame-zero predecessor-command applications;
- the `6986 / 6986` unique physical predecessor-key coverage; and
- the separate `243 / 7000` source-trajectory input-bound audit.

No command/source-frame mismatch was found.
The `6986` unique keys equal `(500 - 1) * 14`;
each is repeated across three variants and 20 seeds in the raw stream.
The input audit covers `500 * 14 = 7000` unique source commands and is not one
of the nine estimator gates.

## Identity closure

The reviewed evidence roots are:

```text
/private/tmp/cbf2026-predictive-wnls-smoke-v4-a
/private/tmp/cbf2026-predictive-wnls-smoke-v4-b
/private/tmp/cbf2026-predictive-wnls-development/stage1-v4
/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v4
```

The independently observed terminal hashes are:

| Artifact | SHA-256 |
| --- | --- |
| protocol JSON | `09b236978e2e21bac226dc3d00c36a2ee5cdf5fea0e616f2c1d875b029d4e4e0` |
| smoke A manifest | `5dae8be4353fe034c7d2df3b69d173311a65aea6673fcd0c56206876850e20ba` |
| smoke B manifest | `8cdfa071c5250bfb9f7c5b7ac11149390a1a783d6072a351bb564ade79a9063b` |
| both smoke compressed processes | `a4f1d719637ffec46b5ad81ae64feac93399bafed7aa77c998b38e9d8d30efc8` |
| both smoke decompressed processes | `72b2cadf90b657325c86ad154d22d2c8a7a27d5d7dee91a5d41adf738cc476ed` |
| registered replay manifest | `123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223` |
| registered compressed process | `9710d1a5792e682bb272d0bb3305162180d1e0b9a0636647d6e071d1782a699b` |
| registered decompressed process | `7047989804fd03bd4cadffac8212349b7183857623ac201d14ef2fc3e30ee2e1` |
| analyzer manifest | `e50352825665bb94c0e3b32c7922a081f093100daf0ed8e5c8ce9e9f32bd9238` |
| compact JSON | `8a0a417f2d3b139fa248bef8157a3d4871fa04042aaad82ce5cd31f470a6ff1e` |
| compact Markdown | `986dcf72f72ab6dc3d424d0de6cdbf410cf271d174d16caef32945acc7d4e13b` |
| unchanged legacy replay source | `0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8` |

Every one of the 22 manifest identity records matched the live preserved
object.
Both smoke streams are byte-identical.
The registered replay is completed at `420000 / 420000` rows,
and the analyzer is completed.

The compressed raw process allocates `218165248 B`;
the raw process plus manifest allocate `218177536 B`,
or approximately `10.91%` of the frozen `2 GB` cap.
The analyzer bundle allocates `176128 B`,
below its `10 MB` cap.
The replay manifest's `allocated_bytes` field counts the gzip allocation,
although its associated cap is named as a raw-bundle cap.
This is a minor accounting-label semantic,
not a capacity or verdict error,
and the author report discloses both values.

## Independently recomputed gates

| Gate | Independent result | Decision |
| --- | ---: | --- |
| maximum published error | `0 / 121626` catastrophic; max `25.851622195360374 < 50 m` | pass |
| maximum fresh error | `0 / 119655` catastrophic; max `25.851622195360374 < 50 m` | pass |
| paired both-fresh p95 | `n=107460`; `9.149727355215518 -> 6.217761243713245 m` | pass |
| fresh retention | `107460 / 127190 = 0.844877741960846 < 0.98` | **fail** |
| fresh-or-predicted availability | `121626 / 140000 = 0.8687571428571429 < 0.95` | **fail** |
| prediction age | `0 / 1971` above two; observed max `2` | pass |
| qualification-anchor violations | `0 / 140000` | pass |
| current-frame provenance violations | `0 / 140000` | pass |
| ascending-DAG violations | `0 / 140000` | pass |

The aggregate is exactly `7 / 9`.
The compact output,
author report,
and both independent audits agree.

## Mechanism and attrition audit

The complete estimator records
`119655` accepted,
zero failed,
zero invalid,
`11531` reference-unavailable,
and `8814` rejected attempts.
It publishes `119655` fresh and `1971` predicted outputs,
leaving `18374` unavailable.
Of those unavailable outputs,
`10695` follow reference-unavailable attempts with no surviving prediction,
and `7679` follow rejected attempts with no surviving prediction.

The fixed frame-44,
frame-138,
and frame-177 records all match the compact report.
Frame 44 demonstrates multistart recovery,
frame 138 demonstrates bounded prediction preventing the legacy
`999.3318962079554 m` stale publication,
and frame 177 demonstrates withheld publication of the legacy
`168.90169712504604 m` fresh branch.

Attrition is concentrated in deeper dependency layers.
Depths one through three retain respectively
`20000`,
`20000`,
and `19996` fresh-or-predicted outputs of `20000`;
depths four through seven retain
`18299`,
`17871`,
`13275`,
and `12185`.
This supports the report's interpretation that qualification,
candidate rejection,
and the finite prediction horizon jointly create a downstream availability
cascade.
It does not justify weakening a gate after observing the result.

The `prediction_expiry` ablation contains `27423` nonfresh-anchor violation
facts across `17877` rows.
There are `5592` multi-violation rows and at most six facts in one row.
The author report uses these units correctly after review.

## Input and paper-evidence boundary

The separate source-trajectory audit independently reproduces
`243 / 7000` component-bound violations,
a maximum component of `35.03975016117459 m/s`,
a maximum planar norm of `35.66026271589104 m/s`,
and a maximum one-frame displacement of `17.83013135794552 m`.
All 7000 source rows record disabled input limits.
This is inherited source-trajectory evidence rather than an estimator defect,
but it prevents this single-trajectory replay from confirming the paper's
`25 m/s` premise.

Stage 1 failure does not authorize the already designed 12-trajectory Stage 2
confirmation.
An availability-recovery change is a new development design.
In particular,
using predicted UAV anchors would conflict with the currently approved
qualification contract and requires an explicit theory,
integrity-gate,
and protocol review;
it cannot be introduced by silently reinterpreting the passing
qualification-anchor gate.

The paper must not claim successful estimator validation from this result.
No Stage 2 trajectory was generated,
no threshold was tuned,
and the preserved Stage 1 evidence was not rerun during review.

## Author-report review closure

The final author report is:

```text
docs/diagnostics/2026-07-30-predictive-wnls-stage1-v4.md
SHA-256 acdf59af7f97e2c5012c5e6b25ce99a4e644eec6573eb55635e969d714c898a2
```

Review findings corrected before approval included:

- distinguishing 27423 violation facts from 17877 affected rows;
- naming a 356-frame streak as consecutive frames within one seed--UAV
  sequence;
- separating frame-44 multistart recovery from the frame-138 catastrophe;
- attributing attrition jointly to qualification,
  rejection,
  and the finite prediction horizon;
- separating file SHA-256 values from Git commit OIDs;
- disclosing process versus two-file-bundle allocation;
- grounding exactly-once execution in the execution ledger and guarded
  terminal roots;
- keeping failed Stage 1 separate from future 12-trajectory confirmation;
  and
- making the predicted-anchor contract conflict explicit.

After those corrections,
the author report has zero open Critical,
Important,
or Minor findings.
