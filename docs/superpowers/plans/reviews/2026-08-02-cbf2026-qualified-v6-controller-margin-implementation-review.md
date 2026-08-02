# CBF2026 Qualified-v6 Controller-Margin Implementation Review

Date: 2026-08-03
Final verdict: PASS — Critical 0 / Important 0 / Minor 0

## Reviewed identities and scope

The committed planning and pre-implementation boundary is `902b2def50783991615c9cfd495b7187335ac3ca`.
Its reviewed Task 6b plan is commit `b90c1a30ed5ae4588dde4dd153e9d2b140d73ce1`, tree `7b8d32a7a37fe7646b114ef8243a2ae8630e3f61`, with plan SHA-256 `2e169d4ad9c36b8a52b5a48847cac5cf646aa3b3b88d3032cea4b15fdef09843`.
The pre-implementation amendment review is commit `902b2def50783991615c9cfd495b7187335ac3ca`; its review file SHA-256 is `c4cb33250da05e12cceb9e729a72f7f29ccd77e0253ad397d97206468a222692` and its verdict is C0/I0/M0.

The frozen implementation candidate is `9e60b2794b866c8e95fcbe5e97347e01977f91e4`, tree `40a40b15e343a9ef0fe6b3142e320ceeaea8b5b5`, parent `902b2def50783991615c9cfd495b7187335ac3ca`.
It contains exactly the 18 paths preregistered by Task 6b: 15 modified files and three new v3 JSON files, with 996 insertions and 138 deletions.

The only review fix is `de6939228f01e2600aeeeba2151ec2162f737cc4`, tree `f9bdc4e72d962f4881f52a8ad69db2d42ea34554`, parent `9e60b2794b866c8e95fcbe5e97347e01977f91e4`.
It modifies exactly four evidence/analyzer source-and-test paths, with 203 insertions and 7 deletions.
The existing unrelated tracked deletions and untracked `build-diagnostic/` were excluded from both commits.

## Review chronology

The first independent post-implementation review returned C0/I1/M0.
The single Important finding was that individually valid historical v2 and registered v3 node policies could be mixed in one controller frame because schema reconstruction and the campaign analyzer did not require a unique frame-level policy identity.

The scoped fix gives every valid policy a normalized identity.
Historical unmarked seven-field evidence and explicitly marked v2 evidence both normalize to `hard-interior-v2`; registered eight-field v3 evidence normalizes to `hard-interior-v3`.
Schema validation, reconstruction, and analyzer validation now require one identity per controller frame.
Pure policy-free v1 frames, pure v2 frames, and pure v3 frames remain accepted, while v2/v3 mixtures fail closed.
All helper call sites were checked after the return type changed.

The independent re-review of the candidate plus fix returned C0/I0/M0.

## Frozen policy and compatibility checks

The v3 primary and fixed-FIM ablation overlays differ from their v2 predecessors only in the controller marker, policy mode, and fraction.
The frozen tuple is `hard-interior-v3`, `planar-chebyshev-fraction-cap-v2`, `fraction=0.131`, `cap=0.1 m/s`, and `feasibility_tolerance=1e-9 m/s`.
The v3 initial family differs from v2 only in `schema_version`, `controller_policy`, and the recomputed `semantic_sha256`.

The six historical v1/v2 file identities remain byte-for-byte unchanged:

- v1 primary: `6a198e17b8c3bbf6777caaa325fa593dd24ee9ce3981f90769075112be1cacb3`
- v1 ablation: `6ad20e1e32fff5e8950f8de31864d77a238f6d418abafe3a50cdb292b8c0bc66`
- v1 family: `7fb8597cd89a41315cbd86ca87c0bc6e3ecb22ea3dcab8ee6278aca44743fe24`
- v2 primary: `ad71ca5d3e7580022b7af4d8f21767aff74dba9c0edb4d347c3c7f174614382d`
- v2 ablation: `13d5a3f2dcf41ce99579c342c64008c8e915b9ff7c91bb028eb98737a958372c`
- v2 family: `21d04b79e9e81ba867e28826ad43615120a4889d16e082d602e933a6a73177ef`

The new v3 file identities are:

- v3 primary: `830bf6a0dc0c62596fe25f3a988e96bf28430fa2400783237eedf21e50d86d8a`
- v3 ablation: `2c2cbac9c772aa738812a5b18c1ae1c68efd74e220e822e8c8300cfae91f30b5`
- v3 family: `8492960b57ba2bba0efd7453359060e81b434ee12b89abdf28aa9a691225fae5`

All 100 v1/v2/v3 materialized position sequences are IEEE-754 identical, including the first ten registered seeds.
Independent reconstruction over the frozen 1,400 frame-zero local problems preserves minimum `rho=0.7658252531927233 m/s`; all v3 floors are exactly `0.1 m/s` and satisfy `mu <= rho`.
This uses the preregistered current-state universe and does not use the consumed next-radius value to choose `0.131`.

The fixed hard-CBF graph, dynamic lower-index primary FIM graph, fixed-FIM ablation graph, local allocated hard rows, componentwise `+/-25 m/s` planar limits, yaw exclusion from the radius LP, class-K coefficient/power `0.1/1`, nominal task target, soft-CBF slack variables, `k_delta=10`, objective, 100/10 seed universes, `dt=0.5 s`, barrier threshold `>0`, radius threshold `>=0.05 m/s`, and no-clamp/no-resample/no-retry rules remain unchanged.

## Verification evidence

For the candidate, fresh root verification passed Python config/initial-family/evidence tests 55/55, 39/39 non-real gate tests, and 1/1 pure Swarm policy fixture, with the real-binary class excluded.
Fresh C++ builds passed RobustConstraintConstruction 40/40 cases and 208/208 assertions, HybridCertificateGuard 19/19 and 152/152, and HardInteriorSelection 14/14 and 28/28.
The independent initial review additionally passed 94 Python tests, the 100-seed/1,400-problem reconstruction, and direct analyzer selection checks accepting exact v3 while rejecting five cross-version mutations.

For the scoped fix, root verification passed evidence tests 31/31, analyzer gate tests 23/23 using an in-memory import stub only for the externally deleted replay module, four-file `py_compile`, and `git diff --check`.
The final independent reviewer also passed short mixed-policy, seed-identity, and fresh C++ directional checks.

No production `Swarm` run, formal 100-seed gate, campaign, retry, protocol, preflight, authorization, paper edit, or positive empirical claim occurred during Task 6b.
The formal claim/output paths and raw/analysis v6 roots remain absent.

The natural import of the complete analyzer test module remains blocked by the unrelated working-tree deletion of `scripts/diagnostics/replay_localization_calibration.py`.
That file was not restored because it is outside this task's authorized scope.
Two reviewers accidentally ran an old-build C++ regression once each, creating six small directories in total under the already-untracked `build-diagnostic/`; neither run executed the production binary or any gate/campaign, and the directories were left untouched.

## Scientific boundary

This implementation establishes only a stronger current-row command-selection margin for each distributed local QP.
It does not prove a lower bound on the next-state radius, recursive feasibility, sampled-data safety, or long-horizon localization accuracy.
Those remain empirical questions for the unchanged formal one-step gate and any later separately authorized Monte Carlo campaign.
