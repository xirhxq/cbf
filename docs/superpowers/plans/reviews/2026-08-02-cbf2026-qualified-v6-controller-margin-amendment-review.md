# Independent Review — CBF2026 v6 Controller-Margin Amendment

## Verdict

PASS — Critical 0, Important 0, Minor 0.

This is the pre-implementation review required by Task 6b. It approves one
frozen candidate and its evidence/lifecycle specification only. It is not a
controller implementation review, one-step gate result, campaign result,
recursive-feasibility proof, or execution authorization.

## Frozen identities

- Source repository: `/private/tmp/cbf2026-diagnostic`
- Final amendment commit: `b90c1a30ed5ae4588dde4dd153e9d2b140d73ce1`
- Final amendment tree: `7b8d32a7a37fe7646b114ef8243a2ae8630e3f61`
- Reviewed plan: `docs/superpowers/plans/2026-08-02-cbf2026-qualified-v6-interior-viability.md`
- Final plan SHA-256: `2e169d4ad9c36b8a52b5a48847cac5cf646aa3b3b88d3032cea4b15fdef09843`
- Amendment chain: `a041910`, `cd6da21`, `de4da39`, `75baac2`, `b90c1a3`
- Pre-amendment controller checkpoint: `4321c579...`

The direct plan delta from `4321c57` to `b90c1a3` is one tracked plan file,
141 insertions and 37 deletions. The later Task 6 producer fix is outside the
scientific choice and did not change the plan's candidate.

## Pre-negative provenance and numerical check

The reviewer used the committed pre-negative blob
`f68cdc7:config/diagnostics/qualified_initial_family_v2.json`, SHA-256
`21d04b79e9e81ba867e28826ad43615120a4889d16e082d602e933a6a73177ef`.
The matching v2 primary and fixed-FIM overlay hashes are
`ad71ca5d3e7580022b7af4d8f21767aff74dba9c0edb4d347c3c7f174614382d`
and `13d5a3f2dcf41ce99579c342c64008c8e915b9ff7c91bb028eb98737a958372c`.
These identities predate the Task 6 one-seed negative integration.

Independent reconstruction of the frozen 100 seeds and 1,400 frame-zero local
hard problems gives minimum current radius
`rho=0.7658252531927233 m/s`. With tolerance `1e-9`, the fraction needed to
reach the already frozen `0.1 m/s` cap is
`0.1/(rho-1e-9)=0.13057809169086337`; the single registered value `0.131`
gives `0.131*(rho-1e-9)=0.10032310803724677`, so every frozen frame-zero floor
is exactly the cap and remains no greater than its local radius. The observed
next-state radius `0.045836701551219286 m/s` was not used to compute or tune
the fraction.

## Review chronology

The initial review returned C0/I3/M2. It required separate immutable pre- and
post-implementation review paths, exact v3/gate-v2 lifecycle identities,
separation of DRA synchronization from the source plan, an exact v3 family
schema/namespace/semantic-hash contract, and explicit pre-negative provenance.
Commit `de4da39` addressed those findings. A first scoped re-review left one
minor global commit-order conflict; `75baac2` fixed Task 6b but broadened the
rule and created an important conflict with Task 7. Commit `b90c1a3` restored
the general review-before-commit rule and made Task 6b the sole explicit
frozen-candidate exception. Final scoped re-review found all prior findings
addressed and no new breakage.

## Approved boundary

Task 6b may implement exactly the additive `hard-interior-v3` /
`planar-chebyshev-fraction-cap-v2` identity at fraction `0.131`, with cap
`0.1 m/s`, tolerance `1e-9`, class-K coefficient/power `0.1/1`, and the exact
18-file staged universe in the plan. Historical v1/v2 bytes, the fixed hard-CBF
graph, dynamic-lower-index primary FIM graph, fixed-FIM ablation graph, local
allocated rows, distributed QPs, objective/slacks, component bounds, yaw
exclusion, seed universe, `dt`, gate thresholds, and no-clamp/no-resample/
no-retry rules remain unchanged.

Only RED/GREEN, pure reconstruction, fixtures, relevant regression tests, and
compilation checks are allowed in Task 6b. It may not run the real `Swarm`
binary, the formal gate, or a campaign. A later failure of the unchanged
100-seed formal gate terminates this candidate and forbids sequential fraction
tuning in this plan. The policy is a current-row command-selection margin; it
does not prove a lower bound on next-state radius, sampled-data safety,
recursive feasibility, or long-horizon localization accuracy.

The reviewer did not modify files or run any real binary, gate, or campaign.
Unrelated working-tree deletions were excluded from this review and left
untouched.
