# CBF2026 two-range reacquisition registered-v2 terminal report

Date: 2026-08-01

Registered replay: `TERMINAL FAILED`, attempt `1/1`

Registered analyzer: `NOT STARTED`, attempt `0/1`

Integrity verdict: `PROTOCOL FAILURE WITH COMPLETE TERMINAL FORENSICS`

Scientific verdict: `NOT EVALUATED`

Paper and Stage-2 gates: `CLOSED`

This report records the only authorized registered-v2 replay.  It preserves a
zero-row producer failure and does not authorize a retry, analyzer execution,
estimator tuning, paper edit, or scientific claim.

## Authorization and frozen provenance

The researcher's exact UTF-8 authorization was `继续`, dated `2026-08-01`,
with no whitespace or newline.  Its SHA-256 is
`7c9691192f1b73408bbe4c0cb6d00db94375ca9d8fce0a0d5985e7a5178f083f`.
It authorized one `registered_replay` and, only after a completed
`140000/140000` replay, one `registered_analyzer`; retries were explicitly
forbidden.

| Role | Identity |
| --- | --- |
| execution plan commit | `b61a5117be4237d121c8cd33d6f2726153b631f1` |
| authorization commit / execution HEAD | `4e76f45db52f7ca79b41c41d82fcab4794cc67f0` |
| authorization commit parent | `b61a5117be4237d121c8cd33d6f2726153b631f1` |
| authorization JSON SHA-256 | `674b61236af9969bdd3106cb0b3ffeb706e46d911485f35405f0c3f9bcaa284a` |
| protocol commit | `5241d6a4d3ab8144ec065dc2f92c4c454a1d3760` |
| implementation parent | `526e1418138426e54bff5a41e3847b6d2a9f8203` |
| preflight/execution-review commit | `84aa6851bfa61468f3e0a6555d081fe124fb2395` |
| prior smoke-evidence commit | `01e1b93384587a6443384fe3a35f4cf22e5ffd94` |
| protocol JSON SHA-256 | `9a0985e00e1b6bd801f7b46c3811c2e135c29f86f75b5045e7aff1ad71206bb2` |

The authorization JSON is the committed 21-field record at
`docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-registered-v2-authorization.json`.
Private committed-state validation and an independent prelaunch contract
review passed.  Immediately before launch, all 11 source identities, retained
smoke artifacts, protocol and authorization bytes, Git state, root absences,
and disk gates matched the frozen contract.

Execution occurred in linked worktree `/private/tmp/cbf2026-diagnostic` on
branch `codex/cbf2026-diagnostic`.  The branch had no configured upstream and
was not pushed.  HEAD stayed at the authorization commit above.  Tracked and
staged paths were clean; the sole ordinary porcelain entry was the preserved
untracked `build-diagnostic/` directory.

The first commit containing this report is necessarily not self-recordable in
the report's own bytes.  Its distinct author-evidence commit is recorded by
the independent review and DRA.  The independent-review commit is likewise a
later evidence role, not the execution HEAD or authorization commit.

## Exact-once operator record

The operator loaded `commands.registered_replay` directly from the committed
protocol JSON.  It did not reconstruct the command.  Compact ordered JSON of
that argv has SHA-256
`b1bbcda65c458b1875407383f979ede8707f4d79b9502824836594870500ee77`.

| Field | Replay | Analyzer |
| --- | --- | --- |
| ledger state | `terminal_failed` | `not_started` |
| attempt | `1/1` | `0/1` |
| PID | `72951` | n/a |
| return code | `1` | n/a |
| operator started UTC | `2026-07-31T17:08:40.712489+00:00` | n/a |
| `Popen` UTC | `2026-07-31T17:08:41.170364+00:00` | n/a |
| operator completed UTC | `2026-07-31T17:08:45.186484+00:00` | n/a |

The exact-once ledger is retained at
`.superpowers/sdd/2026-08-01-cbf2026-two-range-registered-v2/registered-operator-ledger.json`
with SHA-256
`08f0338b6b7acb4c00f875448b595e09800e440ba1d70321a219d5b67c3e93a4`.
It was written before `Popen` and records `retry_allowed=false`.  The operator
lock was normally removed after the child became terminal.  No second replay
process was started.

Prelaunch operator code passed `py_compile`, eight focused tests, a
side-effect-free registered replay preflight, and an independent final review
with Critical `0`, Important `0`, Minor `0`.  The final review explicitly
approved launching the one replay and did not launch either registered
process.

## Disk observations

| Observation | Bytes | Contract |
| --- | ---: | ---: |
| immediate preflight free | `9,580,863,488` | launch minimum `8,000,000,000` |
| first runtime free | `9,577,545,728` | live minimum `6,000,000,000` |
| minimum / terminal free | `9,550,475,264` | live minimum `6,000,000,000` |
| maximum / terminal raw allocation | `8,192` | maximum `2,000,000,000` |

No disk gate failed.  The analyzer root was never created, so compact
allocation was zero.  The earlier cache cleanup removed only recorded,
rebuildable cache contents and did not touch Git, source/DRA worktrees,
protocol inputs, comparators, smoke evidence, Codex sessions, or paper assets.

## Retained terminal evidence

The immutable failed root is
`/private/tmp/cbf2026-two-range-reacquisition-development/v2`.  It is retained
in place and must not be deleted, renamed, reused, overwritten, hard-linked,
or symlinked onto.

| Artifact | Bytes | Allocated bytes | SHA-256 |
| --- | ---: | ---: | --- |
| `manifest.json` | `3,298` | `4,096` | `57dfd97890dec9fef6137791b9d8f24807cdfbeccada9046f449605fa1fedbf6` |
| `two-range-reacquisition.jsonl.gz` | `20` | `4,096` | `9ceffb7310338057cfe71a4ae1e2c98d2c485d81cdef906532a801f457a38d64` |
| decompressed JSONL | `0` | n/a | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| operator stdout log | `0` | n/a | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| operator stderr log | `2,308` | n/a | `1a1d7fd47a6f64afb1234658d5329cf47e0cee7601c6a1f0db5d0fa1574c424a` |

The gzip stream passes `gzip -t` and decompresses to exactly zero bytes and
zero rows.  The producer manifest has:

```text
schema_id       cbf2026-two-range-reacquisition-raw-v1
protocol_id     cbf2026-two-range-reacquisition-v2
invocation_name registered_replay
status          failed
expected_rows   140000
observed_rows   0
process_identity null
started_at      2026-07-31T17:08:44.034559+00:00
completed_at    2026-07-31T17:08:44.088890+00:00
error           ValueError: candidate publication is not accepted/fresh
```

Its protocol, authorization, replay source, WNLS source, truth-data, and input
manifest identities independently rehash to their frozen values.  The stderr
traceback and terminal manifest report the same producer exception.

## Direct failure mechanism

Because `observed_rows=0`, the failed ordered key is necessarily the first
registered key:

```text
method      two_range_private_branch_reacquisition
seed        20260727
frame       0
UAV         1
attempt     existing_predictive_multistart
```

A separate read-only reconstruction using the same internal pure functions,
without invoking replay or writing an output root, established this state:

- the mandatory bases were 0 and 1 and optional base 2 was active;
- their positions were `[-1550,-300]`, `[-1550,0]`, and `[-1550,300]`, hence
  strictly collinear;
- there was no live prediction or private prior;
- the selector was not considered because the active-reference count was
  three, so this was the existing base-bootstrap path, not the new two-UAV
  selector;
- `circle_negative` converged at approximately
  `[-1490.10185,-119.89233]`, while `circle_positive` converged at
  `[-1609.89815,-119.89233]`;
- both candidates had cost `4.160177901918222e-05`, both passed the reduced
  cost gate, and their estimates were `119.796 m` apart; and
- `select_candidate_result()` used its deterministic source ordering to pick
  `circle_negative`, producing an accepted attempt and fresh public output.

`solve_predictive_multistart()` therefore permits several accepted candidates
and selects one by cost, innovation, and source order.  In contrast,
`_validate_row()` requires that a selected existing candidate coexist with
exactly one accepted candidate.  The two implementation contracts contradict
each other, so validation rejected the first row before serialization.

## Scientific interpretation

The immediate outcome is a protocol/implementation contract failure, not a
registered scientific-gate rejection: no scientific row, compact analysis,
integrity-gate vector, or nine-gate scientific vector exists.

The underlying case nevertheless reveals a real modeling limitation.  The
three collinear anchors produce a global mirror ambiguity.  A locally finite
FIM and a small residual/cost do not imply globally unique localization.
The fixed source-order winner happened to be close to offline truth, but truth
is unavailable at runtime, so that ordering is not a theoretically valid
disambiguation signal.  This failure occurs in the existing base-bootstrap
path and does not by itself reject the proposed private-prior two-UAV
reacquisition mechanism.

Simply deleting the validator's uniqueness check would make rows flow but
would silently retain arbitrary mirror selection.  A future, separately
specified design should unify solver/selector/validator semantics and either
fail closed on multiple separated accepted modes or add truth-free global
disambiguation, such as a declared initialization half-plane, a valid history
prior, or non-collinear anchor geometry.  This is a recommendation for a new
decision, not authorization to change or rerun v2.

## Terminal boundary

The failed replay consumed its only authorized invocation.  The required
analyzer predecessor condition—return zero plus independently verified
`completed` and `140000/140000`—is false.  Therefore:

- replay is not retried;
- analyzer remains `0/1` and is not started;
- the failed root and all logs/ledger are retained;
- no estimator comparison or controller-loop injection is performed;
- no thresholds, noise, seeds, geometry, estimator constants, or evidence are
  tuned after observing this failure;
- no paper text, figure, Stage 2, journal decision, submission, or claim is
  changed; and
- any future experiment requires a new reviewed specification, new immutable
  namespace, new protocol, and explicit authorization.

The DRA main branch recorded the immediate terminal checkpoint in commit
`ce110c9` without push.  This report and its independent review complete only
the forensic/archive obligations of the failed registered-v2 workflow.
