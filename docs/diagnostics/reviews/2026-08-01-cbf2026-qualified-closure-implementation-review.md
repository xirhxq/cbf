# Qualified-mode / hybrid distributed-CBF implementation review

Date: 2026-08-02 (Asia/Shanghai)

Reviewed worktree: `/private/tmp/cbf2026-diagnostic`

Reviewed branch: `codex/cbf2026-diagnostic`

Initial reviewed range:
`19c1bab7825d9e974751f9223867124abd6b790c..0e8e9031f83a62a6de327383e86735aede0af733`

Initial reviewed tree: `fcaee04c836730a4db43525cd53b6e771cda443f`

Final reviewed range:
`19c1bab7825d9e974751f9223867124abd6b790c..ec7879404604ca19e12a94cdfa8cb00524ecd305`

Final reviewed HEAD: `ec7879404604ca19e12a94cdfa8cb00524ecd305`

Final reviewed tree: `3efa308786c64a65bdf66208c60d49ed5a69d955`

## Decision

Following the scoped Fix Round 1 re-review at
`ec7879404604ca19e12a94cdfa8cb00524ecd305`, the Tasks 2--9 implementation
and Task 10 Steps 1--4 closure are **approved only for development-v1
protocol generation**.

| Severity | Count |
| --- | ---: |
| Critical | `0` |
| Important | `0` |
| Minor | `0` |

Ready for development-v1 protocol generation: **Yes**.

This decision authorizes development-v1 protocol generation only. It
authorizes no preflight approval, authorization publication, development
campaign, confirmatory campaign, DRA append, paper edit, or paper claim. No
implementation, test, plan, report, protocol, preflight, authorization, or
execution-root file was changed by this review.

## Reviewed identities and artifacts

The independent review used the approved design, its review, the original
implementation plan and plan review, the Task 10 brief, the implementation
verification report, and the supplied extended-context review package. The
live identities that can be recomputed from the reviewed worktree are:

```text
ced4824b0cb8b2919579469e876650aa45e71c26f1b5853ddf2633550822127b  docs/superpowers/specs/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-design.md
897881d421d1bf78b4d0b1aed5dca86b574430aed0290e35f843fe6c840d6c9c  docs/superpowers/specs/reviews/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-design-review.md
82e2783d9d045f239da164eafdcca8a3198b5a87d440d1f7dd4d1467d11f1ff7  docs/superpowers/plans/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-implementation.md
cf5bd776bb99183015dce678c4136964edcabd81a64624ff1c6b446d7a107d90  docs/superpowers/plans/reviews/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-implementation-plan-review.md
abc52265b84636f3eec2ea859f360872cd95f28fa77486538ee6003a924d5be8  docs/diagnostics/2026-08-01-cbf2026-qualified-closure-implementation.md
```

The supplied review package identity was:

```text
a7c14f83782824dd765cb0bc602e4b8796fd35df6d07e88a3ca4cf5c0b72ef3c  .superpowers/sdd/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-implementation/review-19c1bab..0e8e903.diff
```

The review package covered 50 changed paths, 17 commits, and the exact
baseline-to-HEAD implementation range. The live worktree retained the
pre-existing untracked `build-diagnostic/` directory and the untracked Task 10
implementation report; neither was treated as a clean full-repository state.

The preserved two-range v2 development root still contained exactly the two
regular files whose sorted tree commitment is:

```text
57670b00a303c4c655559097d2a80589c4cea663bc88d3033dfe30fb5fbea6cd
```

`/private/tmp/cbf2026-two-range-reacquisition-analysis/v2` remained absent.
The qualified development raw and analysis roots, development protocol JSON,
and development authorization remained absent during the review.

## Review method

The review inspected the baseline-to-HEAD diff and the live source along two
axes: repository standards and conformance to the approved scientific and
operational specification. It traced each frozen contract from production
entry point through raw evidence, analyzer reconstruction, gate computation,
registration binding, and test coverage. It did not run a campaign and did
not allocate any protocol, authorization, or execution root.

The following focused Python suite was freshly rerun:

```text
tests.test_qualified_closure_evidence
tests.test_generate_qualified_measurements
tests.test_run_qualified_closure_campaign
tests.test_analyze_qualified_closure_campaign
tests.test_register_qualified_closure_campaign
```

Result: `153 / 153` passed in `3.323 s`.

All ten direct C++ binaries were freshly rerun. Result: `111 / 111` cases and
`1106 / 1106` assertions passed with zero skipped. `ctest` passed `1 / 1`, and
the fresh cache check returned `ENABLE_GUROBI:BOOL=ON`.

Passing focused tests do not close the findings below: none of the tests
asserts that the frame-zero qualified-estimator audit is retained as the
initialization evidence, that fresh retention uses the full primary tuple
universe, or that the synchronous measurement-generation phase observes the
6 GB hard floor.

## Contract verdicts

| Contract class | Verdict | Review result |
| --- | --- | --- |
| Global-mode completeness, qualification, and publication | Qualified pass | All locally eligible records are clustered before qualification, nonseparable chains fail closed, both two-range branches are retained, and only one admissible mode publishes. The campaign nevertheless discards the frame-zero audit that is the base case for this lifecycle; see C1. |
| Truth-free runtime and condition-local history | Pass | Runtime qualification rejects truth/future/realized-error fields recursively, uses only the immediately preceding held command and private history, and keeps primary and ablation lifecycle state separate. |
| Command-independent topological rate certificate | Pass | The rate certificate is built before the QPs from the topological speed envelope, does not use a backward difference or neighbor command, and does not promote dynamic FIM-only edges into distance-CBF rows. |
| Canonical edge, allocation, and local hard QP | Pass | Fixed localization and collision edges are canonical and complete; UAV--UAV rows allocate one half to each endpoint, base rows allocate the full row to the UAV, and each local hard QP is reconstructed from the exact committed problem without a neighbor's current command. |
| Exact-version hybrid reset and anti-chatter | Pass | The guard checks exact successor version, descendant closure, post-reset barriers, every fresh local hard QP, atomic all-node commit, and at most one reset transaction per frame. This is correctly reported as sampled implementation anti-chatter, not a continuous no-Zeno proof. |
| Raw evidence, version, and exact-witness reconstruction | **Fail** | Post-initialization estimator and controller primitives are independently reconstructible, but the scientific frame-zero estimator decision is not retained or reconstructed; see C1. |
| Campaign identity, key universe, and failure complement | **Fail** | Schedules, identities, exact complements, launch-failure denominators, and primary-only catastrophic gates are otherwise frozen, but the fresh-retention denominator removes primary tuples according to the observed outcome; see C2. |
| Shared measurements and truth isolation | Pass | Both conditions consume the same immutable runtime measurement identity, truth/audit data stays analyzer-only, and each condition resolves UAV references from its own publication history. |
| Atomic/no-retry/lifecycle behavior | Pass | Root claiming, exact schedule publication, streamed-prefix retention, terminal failure synthesis, no-replace publication, authorization binding, and no campaign retry are implemented and covered. |
| 8/6/2 GB, 25 MB, and timeouts | **Fail** | The 8 GB launch check, 2 GB reusable-cache cap, 25 MB compact cap, 3600 s wallclock, and 300 s complete-line stall checks are present. The 6 GB floor does not cover synchronous measurement generation; see I1. |

The historical two-range v2 root and identity remained immutable. The exact
Task 11 runner and analyzer token sequences in the implementation report
match the plan and registrar derivation, including the development-v1 roots,
seed pairs, 1000 frames, primary/ablation bindings, authorization, and
analyzer-only ablation binding. They remain recorded commands only and must
not be executed under this decision.

The continuous-time theorem and 2 Hz implementation are kept separate. The
implementation report correctly limits runtime evidence to sampled controller
intervals, states that no sampled-data reserve exists, and does not convert
one-transaction-per-frame anti-chatter into a continuous no-Zeno claim.

## Critical findings

### C1. The campaign consumes but discards the complete frame-zero qualified-estimator audit

The approved design freezes frame zero as a distinct initialization universe
and requires every initialization outcome to report the deployment-domain
assumption, mode enumeration, qualification, and publication. This is the
base case on which the condition-local private/public history induction
depends (`design.md:801-819`).

The replay producer computes the complete qualified row for every
`(frame=0, robot)` at
`scripts/diagnostics/run_qualified_closure_campaign.py:1502-1529`, then emits
rows only under `if frame > 0` at lines 1530--1536. Despite suppressing the
row, `_build_condition_replay_row` selects the deployment qualifier at lines
1624--1627 and advances the public, private, and history state from that
unretained decision at lines 1659--1662.

The separate C++ initialization stream cannot close the gap. Its runtime
payload is only `local_index`
(`include/Swarm.hpp:1142-1157`), and the exact initialization validator
requires only that field plus analyzer truth position
(`scripts/diagnostics/qualified_closure_evidence.py:170-191`). The production
analyzer consequently only validates/order-counts 14 such records at
`scripts/diagnostics/analyze_qualified_closure_campaign.py:1694-1698`; it
cannot independently reconstruct the deployment domain, enumerated modes,
qualifications, publication, or next private state used by frame 1.

A read-only reproduction found `if frame > 0` in the real producer and
printed the real initialization validator, confirming that the discarded
audit has no substitute in the accepted schema. Thus a wrong or malformed
cold-start publication can seed all later condition-local history while the
campaign still reports complete initialization accounting. This breaks the
scientific base case and exact-witness requirement rather than merely omitting
diagnostic detail.

### C2. Fresh retention is computed over an outcome-filtered subset instead of the full primary tuple universe

The approved definition is explicit: every scheduled post-initialization
primary tuple remains in the primary tuple universe, and fresh retention is
fresh outputs divided by that universe; no runtime or analyzer status may
remove a tuple (`design.md:801-811`).

The synthetic analyzer first filters rows whose observed
`fresh_expected` flag is true and uses only that subset as the denominator
(`scripts/diagnostics/analyze_qualified_closure_campaign.py:431-437`). The
production path derives the same flag from the observed result
`admissible_mode_count == 1` at lines 1948--1981, then restricts both SQL
counts to `fresh_expected=1` at lines 2174--2177. Unavailable, ambiguous,
multi-admissible, and no-admissible primary tuples are therefore removed from
this gate rather than counted against retention.

The defect is directly reproducible through the public synthetic analyzer.
Changing one of two primary rows to bounded `predicted` and setting
`fresh_expected=False` returns:

```text
{'numerator': 1, 'denominator': 1, 'comparison': '>=',
 'threshold': 0.98, 'passed': True, 'pass_fail': 'PASS'}
```

The frozen definition requires denominator `2`, which would fail at `0.5`.
The separate 0.95 availability gate does not repair this error: a campaign
can satisfy availability while excluding enough nonfresh tuples to turn a
sub-0.98 full-universe retention rate into `1.0`. The registered scientific
gate can therefore return a false PASS.

## Important findings

### I1. The 6 GB hard floor is not monitored while synchronous measurement bundles are generated

The approved resource contract requires the runner to stop cleanly before
free space falls below 6 GB (`design.md:958-959`; implementation plan line
41). `supervise_child_to_gzip` enforces that floor while a child stream is
active at `scripts/diagnostics/run_qualified_closure_campaign.py:322-363`.

Production measurement generation does not use that supervisor. It calls
`generate_measurement_bundle` synchronously at
`scripts/diagnostics/run_qualified_closure_campaign.py:851-870`.
The generator signature has no disk probe or guard, and its truth loop writes
both compressed runtime and audit streams from
`scripts/diagnostics/generate_qualified_measurements.py:80-103` without a
free-space check. The fresh signature/source reproduction confirmed that no
`available_bytes_fn` or equivalent guard is passed through this production
phase.

The existing six-GB regression exercises `supervise_child_to_gzip`, not this
synchronous writer. A measurement bundle can therefore cross the frozen hard
floor before ordinary exception cleanup occurs; the runner has no contract
mechanism to stop cleanly at the threshold during that phase.

## Minor findings

None.

## Verification qualification

The implementation report's fresh full Python discovery is accurately
qualified, not green: `1142` total, `1129` passed, `5` failed, `8` errored,
and `0` skipped. Ten nonpassing legacy registrar cases all encounter the
pre-existing circular generated-v2-parent guard in unchanged files. Three
errors require the same absent external truth fixture. The source and test
attribution supports the narrow statement that no nonpassing result in that
run was attributable to a Tasks 2--9 changed-contract test; it does not
certify the complete repository suite as green, and it does not cover the
three defects above.

The fresh focused rerun and C++ rerun corroborate the implementation report's
test counts, but test success does not override a contradictory frozen
denominator, missing evidence member, or uncovered operational phase.

`git diff --check` returned zero. This review document is the only
review-scope path created by this independent review.

## Initial-review readiness

Critical: `2`; Important: `1`; Minor: `0`.

Ready for development-v1 protocol generation: **No**.

Finding summary: frame-zero qualified initialization is consumed but not
retained, fresh retention uses an outcome-filtered denominator, and the 6 GB
hard floor omits synchronous measurement generation.

The decision above records the original review state before Fix Round 1 and
is superseded by the scoped re-review below. The original findings and their
evidence remain unchanged as review chronology.

## Fix Round 1 scoped re-review

### Scope and identity

This re-review is limited to commit
`ec7879404604ca19e12a94cdfa8cb00524ecd305`, tree
`3efa308786c64a65bdf66208c60d49ed5a69d955`, and its exact five-file diff
from `0e8e9031f83a62a6de327383e86735aede0af733`:

```text
scripts/diagnostics/analyze_qualified_closure_campaign.py
scripts/diagnostics/generate_qualified_measurements.py
scripts/diagnostics/run_qualified_closure_campaign.py
tests/test_analyze_qualified_closure_campaign.py
tests/test_run_qualified_closure_campaign.py
```

The supplied fix review package has SHA-256:

```text
632fabbef21535c5cbe8ee3ebedbd7d4a10fcf48c4b8ab30b7d62f36b622f8f5  .superpowers/sdd/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-implementation/review-0e8e903..ec78794.diff
```

No campaign, protocol, preflight, authorization, or execution root was
created or used in this re-review. The historical two-range v2 tree
commitment remained
`57670b00a303c4c655559097d2a80589c4cea663bc88d3033dfe30fb5fbea6cd`,
and all qualified development and confirmatory raw/analysis roots remained
absent.

### Finding dispositions

#### C1 — ADDRESSED

The replay producer now emits every frame-zero qualified audit as well as all
post-initialization rows
(`scripts/diagnostics/run_qualified_closure_campaign.py:1517-1552`). A
completed mission is accepted only when its C++ initialization record and
both condition-specific frame-zero qualified audits exist for each shared
initialization key. The production analyzer stores those three evidence
members separately, validates every raw qualified row through independent
recomputation, and forms the composite initialization count with both
condition joins
(`scripts/diagnostics/analyze_qualified_closure_campaign.py:1326-1336`,
`1475-1529`, and `2220-2237`).

The frame-zero audit is independently checked for the deployment qualifier,
mode enumeration, all qualifications, publication, lifecycle, and next
private state before its key is counted. It is inserted only into
`initialization_audit`; frame zero never enters the post-initialization
`estimator` table or estimator counters
(`scripts/diagnostics/analyze_qualified_closure_campaign.py:1499-1509` and
`2050-2071`). Thus it cannot contaminate the primary tuple denominator.

Failure handling reconstructs the same composite key on both sides of the
producer/analyzer boundary. The runner and analyzer each intersect the C++
initialization prefix with the frame-zero keys retained by both registered
conditions, while retaining only frame `>0` rows in the estimator prefix
(`scripts/diagnostics/run_qualified_closure_campaign.py:976-1074` and
`scripts/diagnostics/analyze_qualified_closure_campaign.py:1598-1704`). A
missing C++ row or either condition audit therefore remains in the exact
failure complement.

#### C2 — ADDRESSED

The synthetic analyzer now requires an explicit frozen primary-tuple count,
rejects an observed primary set larger than that declaration, and uses the
frozen count for joint containment, fresh retention, and availability
(`scripts/diagnostics/analyze_qualified_closure_campaign.py:315-324` and
`424-450`). A missing observed row cannot shrink any of those denominators.

The production database no longer has or derives `fresh_expected` from the
observed admissible-mode count. Fresh retention counts only primary
`status='fresh'` rows in the numerator and uses the protocol's frozen
`estimator_per_condition` universe in the denominator
(`scripts/diagnostics/analyze_qualified_closure_campaign.py:1335-1337` and
`2275-2282`). Frame-zero rows take the separate initialization path described
under C1, so the registered post-initialization universe remains exactly
frames `1..H-1`.

#### I1 — ADDRESSED

The synchronous measurement writer now receives the production free-space
probe and checks the 6 GB floor before each logical runtime/audit pair
(`scripts/diagnostics/run_qualified_closure_campaign.py:852-872` and
`scripts/diagnostics/generate_qualified_measurements.py:91-112`). A floor
stop occurs before either member of the next pair is written; both gzip
streams close together and their manifests bind the same retained row count,
stream identity, individual SHA-256, `status='failed'`, and exact
`reason='disk_hard_floor'`
(`scripts/diagnostics/generate_qualified_measurements.py:152-194`). The
failure bundle is atomically published, and the owned hidden stage is removed
in `finally` if publication did not consume it.

The campaign coordinator treats that terminal failed result as an expected
terminal operation, preserves the exact reason, retains the paired-prefix
measurement bundle as a failed-mission member, synthesizes the exact remaining
universe, and propagates `disk_hard_floor` through every remaining mission and
the campaign terminal manifest
(`scripts/diagnostics/run_qualified_closure_campaign.py:1916-1933` and
`2121-2133`). No temporary measurement or mission stage remained in the
targeted failure probes.

### New-finding audit

No new Critical or Important breakage was identified in the five-file fix
diff. The independent Standards axis found no documented-standard violation
or Important-level smell in the changed hunks. In particular, the composite
initialization witness, paired stream prefix, terminal reason propagation,
and cleanup paths do not introduce a new gate or lifecycle bypass.

### Fresh verification

The eight exact C1/C2/I1 regressions were independently rerun and passed
`8 / 8` in `0.097 s`. They cover:

- full-universe fresh retention in both synthetic and production paths;
- a missing primary row that cannot shrink the synthetic denominator;
- full frame-zero replay retention;
- the C++ plus both-condition composite initialization key in both runner and
  analyzer failure-complement reconstruction;
- independent base-case audit recomputation without post-initialization
  metric contamination;
- paired-prefix synchronous 6 GB termination; and
- exact `disk_hard_floor` propagation through the complete schedule.

The complete five-module focused suite passed `160 / 160` in `4.124 s`.
All three changed production scripts passed `py_compile`. Both the live
worktree and `0e8e903...ec78794` fix diff passed `git diff --check`.

These focused results do not rewrite the original full-discovery
qualification: the prior `1142`-test repository run remains non-green for the
ten unchanged circular-parent registrar outcomes and three missing external
fixture outcomes documented above.

### Final decision

Critical: `0`; Important: `0`; Minor: `0`.

Ready for development-v1 protocol generation: **Yes**.

Finding summary: C1, C2, and I1 are addressed; no new Critical or Important
breakage was found in Fix Round 1.

## Final report-only identity review

### Identity closure

This final pass reviewed no new implementation diff. It binds the preserved
initial review range
`19c1bab7825d9e974751f9223867124abd6b790c..0e8e9031f83a62a6de327383e86735aede0af733`
and Fix Round 1 to the final source identity:

```text
HEAD  ec7879404604ca19e12a94cdfa8cb00524ecd305
tree  3efa308786c64a65bdf66208c60d49ed5a69d955
```

The final report-only update is 24,053 bytes / 496 lines and has SHA-256:

```text
abc52265b84636f3eec2ea859f360872cd95f28fa77486538ee6003a924d5be8  docs/diagnostics/2026-08-01-cbf2026-qualified-closure-implementation.md
```

The current report correctly records the complete 18-commit sequence from
the planning baseline, the unchanged 50-path baseline-to-final-HEAD set, the
initial C2/I1/C1 finding chronology, commit `ec78794` closure, and the scoped
C0/I0/M0 re-review. Its development-v1 statement is limited to protocol
generation and does not confer preflight, authorization, or execution
authority.

### Verification reconciliation

The full Python discovery was independently rerun from the final HEAD with:

```bash
conda run -n cbf_env python -m unittest discover -s tests -p 'test_*.py'
```

The fresh result was exactly `1149` tests, `5` failures, `8` errors, hence
`1136` passed, with no skip reported. The five failures and five errors in
the untouched legacy two-range registrar class all reached the already
generated-v2 circular-parent guard. The other three errors all reached the
same absent external truth fixture. This matches the report's names,
attribution, and impact boundary. The result remains explicitly non-green;
no failure, error, or skip was reclassified or waived.

Read-only reconciliation also confirmed:

- exactly 50 changed paths over `19c1bab..ec78794`, while the four legacy
  registrar/extractor files used for attribution remain unchanged;
- binary, CMake cache, base/primary/ablation config, and conda-explicit hashes
  exactly match the report;
- `ENABLE_GUROBI:BOOL=ON` and the reported dynamic dependencies remain
  present;
- truth/future/realized-error matches remain confined to the explicit runtime
  rejection set, and allocated/reset headers contain neither forbidden
  neighbor velocity nor backward uncertainty-rate use;
- the two-range v2 development root retains tree commitment
  `57670b00a303c4c655559097d2a80589c4cea663bc88d3033dfe30fb5fbea6cd`;
  and
- the two-range v2 analysis root, qualified development raw/analysis roots,
  development protocol, preflight, authorization, and all confirmatory roots
  remain absent.

No campaign, protocol, preflight, authorization, result, or execution root
was created by this identity review. The exact Task 11 commands remain
documentary and unexecuted.

### Final identity verdict

Critical: `0`; Important: `0`; Minor: `0`.

Ready for development-v1 protocol generation: **Yes**.

Finding summary: the final implementation report is identity-consistent with
`ec78794`/`3efa308`, its qualified verification record, and the observed
no-campaign state; no report-only C/I/M finding remains.
