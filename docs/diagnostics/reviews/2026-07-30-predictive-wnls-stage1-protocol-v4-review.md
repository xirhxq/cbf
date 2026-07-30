# Predictive WNLS Stage 1 v4 protocol preflight review

Date: 2026-07-30

Decision: APPROVED

Review result: 0 Critical, 0 Important, 0 Minor.

This review covers only protocol registration and execution preflight.
It does not claim a scientific Stage 1 result,
and the paper gate remains `CLOSED`.
No v4 smoke,
registered replay,
or registered analyzer command had run when this review was committed.

## Frozen identities

The implementation was committed first as
`68a9a0c1c85abc33ce85b9b846d43b139a1d5ebf`.
The registrar then created the two protocol files exactly once.
The protocol commit is
`1ed3fc0557c07e33bbf0ed5e3f963702edaf4b00`.
This ordering is non-circular:
the protocol files are absent from the implementation parent that they bind.

| Artifact | SHA-256 |
|---|---|
| Protocol JSON | `09b236978e2e21bac226dc3d00c36a2ee5cdf5fea0e616f2c1d875b029d4e4e0` |
| Protocol Markdown | `78acce325749a96fbe1ae8c79245e4e73eaac66773998940cf2291726f1854f0` |
| Preserved v3 raw gzip | `57503b331f52ac036f88d33b4377d093d624fc9bf5283286e452af819b72f88b` |
| Preserved v3 failed-analyzer manifest | `ad5fb00f0b64b75446c84caf97a9313803c1f31352cc58a7408f4d1eda67b69c` |
| Immutable legacy solver | `0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8` |

The JSON and Markdown bytes were independently rebuilt in memory from the
registered implementation parent and source declarations.
Both rebuilt byte strings matched the committed files exactly.

## Source audit

All eight live source hashes matched the protocol.
The five repository-owned sources also matched their Git blobs at the
implementation parent.

| Source | SHA-256 |
|---|---|
| Analyzer | `bbc4e6e0940580f48222f641cba0f705c8b8f419c585a18999cb1cdc9e4146a8` |
| Baseline process | `c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003` |
| Diagnostic integrity helper | `ccde94fa739649cb9b94a8304ef3867ae946f28b0e3a812d3feb3f30bb76fd23` |
| Predictive estimator | `3decd0d1eabfb0e54f56852d2e7c91726579ddfee806835502b1da6333724ea7` |
| Input manifest | `6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb` |
| Legacy solver | `0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8` |
| Predictive replay | `10922bf369822a6898053b7d326bc4937a7da3e0b44a898eaeea4d18fc752661` |
| Truth trajectory | `3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527` |

## Scientific and semantic contract

The protocol fixes 20 range-noise seeds,
three cumulative variants,
all estimator constants,
and nine outcome gates.
The raw identity is
`cbf2026-predictive-wnls-development-rows-v3`,
and the analysis identity is
`cbf2026-predictive-wnls-development-analysis-v3`.

The raw-v3 contract includes the 19-field multistart compact candidate.
It preserves every field required to reconstruct the complete solver result.
The analyzer therefore reconstructs candidate acceptance,
gate diagnostics,
terminal attempt status,
cost/q/source selection,
and the selected fresh publication without trusting serialized decisions.
Nonaccepted publications are bound to the reconstructed prior-public
propagation.

The public covariance contract permits only
`numpy.allclose` roundoff asymmetry at `rtol=atol=1e-12`,
publishes `0.5*(Sigma+Sigma.T)`,
and requires the serialized covariance to equal that canonical output.
Violation facts are unique and sorted by
`(base < uav, reference id, reason)`.

Before registration,
the implementation passed:

- analyzer tests: 65/65;
- producer, schema, and registrar tests: 139/139;
- full test discovery: 532/532;
- `py_compile`;
- direct CLI help checks;
- `git diff --check`;
- two independent final implementation reviews with
  0 Critical, 0 Important, and 0 Minor findings.

## Execution and disk preflight

The protocol fixes four command arrays:
`smoke_a`,
`smoke_b`,
`registered_replay`,
and `registered_analyzer`.
They must execute in that order and at most once each.
No failed command may be retried under this protocol.

At approval time,
all four v4 output roots were absent:

```text
/private/tmp/cbf2026-predictive-wnls-smoke-v4-a
/private/tmp/cbf2026-predictive-wnls-smoke-v4-b
/private/tmp/cbf2026-predictive-wnls-development/stage1-v4
/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v4
```

All four retired v3 roots remained present.
Free space was approximately 45.5 GB,
above the 8 GB launch threshold.
The live floor is 6 GB,
the rebuildable/raw allocation cap is 2 GB,
and the compact analysis cap is 10 MB.

The exact committed `smoke_a` command is approved as the next action.
Acceptance requires a completed terminal manifest,
84/84 exact rows,
matching compressed and decompressed hashes across both later smokes,
canonical violations,
and zero fresh-but-ineligible public outputs.
This approval does not authorize any parameter,
gate,
seed,
trajectory,
or command change after observing results.
