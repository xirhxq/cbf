# Independent review of CBF2026 two-range registered-v2 terminal evidence

Date: 2026-08-01

Author report commit: `2da1c4b669cdeb6ebe2a9a9480d8564f797bc920`

Author report path:
`docs/diagnostics/2026-08-01-cbf2026-two-range-reacquisition-registered-v2.md`

Integrity verdict: `APPROVED: COMPLETE TERMINAL FORENSICS; EXACT-ONCE FAILURE CONSUMED`

Protocol verdict: `FAIL: FIRST REGISTERED ROW EXPOSES A PRODUCER/VALIDATOR CONTRACT CONTRADICTION`

Scientific verdict: `NOT EVALUATED`

Registered analyzer: `NOT STARTED`, attempt `0/1`

Analyzer, paper, Stage-2, tuning, submission, and claim gates: `CLOSED`

This was a read-only independent review.  I reopened the author report from its
committed blob, reopened the retained failed root, ledger, logs, protocol,
authorization, Git history, and DRA terminal checkpoint, and independently
recomputed the relevant byte identities.  I did not execute either registered
command, modify any retained evidence, modify source/protocol/paper/DRA files,
or create another experimental root.

## Findings

Critical: `0`

Important: `0`

Minor: `1`

The one Minor finding is an archival chronology item, not an integrity defect.
The inspected DRA terminal checkpoint commit
`ce110c96214a2b413acc6eb7f71fbf7a0999a884` predates the author-evidence
commit and therefore does not yet record author commit
`2da1c4b669cdeb6ebe2a9a9480d8564f797bc920` or the committed report's byte
identity.  A later controller-owned DRA archival checkpoint should record the
author and independent-review roles after this review exists.  The current DRA
checkpoint does accurately record the execution failure, hashes, zero-row
boundary, no-retry decision, analyzer `0/1`, and closed gates.

No finding changes the author report's terminal, protocol, or scientific
verdict.

## Commit and evidence roles

The roles are distinct and must not be collapsed:

| Role | Commit or identity | Independent observation |
| --- | --- | --- |
| frozen protocol | `5241d6a4d3ab8144ec065dc2f92c4c454a1d3760` | Protocol-v2 contract, not execution evidence |
| implementation parent | `526e1418138426e54bff5a41e3847b6d2a9f8203` | Frozen implementation lineage |
| preflight/execution review | `84aa6851bfa61468f3e0a6555d081fe124fb2395` | Parent is the protocol commit; this is the approved prelaunch code/review role |
| prior smoke evidence | `01e1b93384587a6443384fe3a35f4cf22e5ffd94` | Evidence-only smoke report/review commit; parent is `84aa6851...` |
| registered execution plan | `b61a5117be4237d121c8cd33d6f2726153b631f1` | Parent is the prior smoke-evidence commit |
| authorization and execution HEAD | `4e76f45db52f7ca79b41c41d82fcab4794cc67f0` | Parent is the plan commit; ledger independently records this exact execution HEAD |
| author terminal evidence | `2da1c4b669cdeb6ebe2a9a9480d8564f797bc920` | Evidence-only child of execution HEAD; adds only the author report |
| DRA immediate terminal checkpoint | `ce110c96214a2b413acc6eb7f71fbf7a0999a884` | DRA `main`; records the terminal failure before author/review archival identities existed |

The committed author report is `10,293` bytes and independently hashes to
`8ebfacf84ebba1402c4884bc1d91339e0d02575f567a83ccd285372f838036b8`.
Its commit has parent
`4e76f45db52f7ca79b41c41d82fcab4794cc67f0`, tree
`41b4e1967ea403e23a9982292a0fcf675f221c01`, and subject
`docs(diagnostics): record registered v2 replay failure`.

Before this review file was created, source HEAD was the author-evidence commit
on `codex/cbf2026-diagnostic`, and ordinary porcelain contained only
`?? build-diagnostic/`.  This later author-report state must not be confused
with execution HEAD `4e76f45...`, which is fixed by the ledger.

The first commit containing this independent review does not yet exist.  Its
identity cannot be self-recorded in these bytes without circularity and must be
recorded later by the controller, together with this review's externally
computed byte size and SHA-256.

## Authorization and protocol binding

The authorization is the committed 21-field v2 record.  It binds exact UTF-8
text `继续`, dated `2026-08-01`, to one replay, a conditional analyzer, exact
registered roots, and `registered_retry_allowed=false`.  Its independently
recomputed SHA-256 is
`674b61236af9969bdd3106cb0b3ffeb706e46d911485f35405f0c3f9bcaa284a`.
The authorization-text SHA-256 is
`7c9691192f1b73408bbe4c0cb6d00db94375ca9d8fce0a0d5985e7a5178f083f`.

The protocol JSON independently hashes to
`9a0985e00e1b6bd801f7b46c3811c2e135c29f86f75b5045e7aff1ad71206bb2`.
The working-tree bytes equal the blobs at the author commit.  Compact ordered
JSON for `commands.registered_replay` and for the ledger's stored argv both
independently hash to
`b1bbcda65c458b1875407383f979ede8707f4d79b9502824836594870500ee77`.
Thus the consumed command is the frozen protocol command, not a reconstructed
or substituted argv.

The terminal manifest's source/input identities also rehash exactly:

| Identity | SHA-256 |
| --- | --- |
| two-range method source | `d226c0184d89aadfe7b25a4a52d27379a396f868cfaab2c5a8a13ee7cdc04f8c` |
| predictive WNLS source | `0269baa715c087661f2fa4c4760345250b380e2a14cce021fc3aca1b62f2defc` |
| replay source | `c86fd4b017b69e85749372acfe516cabd4e58aebd9749a55ac7eb5e5ec5ea52c` |
| preserved truth data | `3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527` |
| preserved input manifest | `6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb` |

## Exact-once ledger and process terminal state

The retained ledger is `5,614` bytes with SHA-256
`08f0338b6b7acb4c00f875448b595e09800e440ba1d70321a219d5b67c3e93a4`.
It independently reports:

| Field | Replay | Analyzer |
| --- | --- | --- |
| state | `terminal_failed` | `not_started` |
| attempt count | `1` | `0` |
| ordinal | `1/1` | `0/1` |
| PID | `72951` | n/a |
| return code | `1` | n/a |

The ledger has `retry_allowed=false`, protocol and authorization hashes equal
to the committed bytes, and execution HEAD `4e76f45...`.  Replay started at
`2026-07-31T17:08:40.712489+00:00`, `Popen` occurred at
`2026-07-31T17:08:41.170364+00:00`, and the operator completed at
`2026-07-31T17:08:45.186484+00:00`.  Its terminal error correctly states that
the replay manifest was not completed `140000/140000`.

The analyzer root is absent and no analyzer stdout/stderr evidence exists.
The attempt accounting therefore supports exactly replay `1/1` and analyzer
`0/1`; it does not support a retry, a second replay, or an analyzer start.

## Retained root, manifest, gzip, and logs

The retained root contains only the failed manifest and gzip stream.  Direct
`stat`, SHA-256, gzip integrity, decompression, and line-count checks produced:

| Artifact | Logical bytes | Allocated bytes | SHA-256 |
| --- | ---: | ---: | --- |
| `manifest.json` | `3,298` | `4,096` | `57dfd97890dec9fef6137791b9d8f24807cdfbeccada9046f449605fa1fedbf6` |
| `two-range-reacquisition.jsonl.gz` | `20` | `4,096` | `9ceffb7310338057cfe71a4ae1e2c98d2c485d81cdef906532a801f457a38d64` |
| decompressed JSONL | `0` | n/a | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| operator stdout | `0` | `0` | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| operator stderr | `2,308` | `4,096` | `1a1d7fd47a6f64afb1234658d5329cf47e0cee7601c6a1f0db5d0fa1574c424a` |

`gzip -t` returns `OK`.  Independent decompression produces exactly zero bytes
and zero lines.  The retained root's two files occupy exactly `8,192`
allocated bytes, matching the ledger maximum and terminal allocation.

The terminal manifest is internally coherent:

```text
schema_id        cbf2026-two-range-reacquisition-raw-v1
protocol_id      cbf2026-two-range-reacquisition-v2
invocation_name  registered_replay
status           failed
expected_rows    140000
observed_rows    0
process_identity null
started_at       2026-07-31T17:08:44.034559+00:00
completed_at     2026-07-31T17:08:44.088890+00:00
error            ValueError: candidate publication is not accepted/fresh
```

The stderr traceback reaches `_validate_row()` and reports the same exception.
There is no completed process identity because no row was serialized.  This is
complete terminal failure evidence, not a completed raw scientific bundle.

## Disk contract

The ledger records `9,580,863,488` free bytes at prelaunch, above the
`8,000,000,000` launch floor.  Its first runtime observation is
`9,577,545,728` bytes; the minimum and terminal observation is
`9,550,475,264` bytes, above the `6,000,000,000` live floor.  Maximum and final
root allocation are `8,192` bytes, below the `2,000,000,000` raw cap.  No disk
contract caused this failure.

## Independent first-row reconstruction

Because the gzip contains zero rows, the ordered grid identifies the rejected
candidate record as the first key:

```text
method  two_range_private_branch_reacquisition
seed    20260727
frame   0
UAV     1
```

I independently reconstructed that key in memory through the frozen pure
producer functions, without invoking replay/analyzer or allocating an output
root.  Reapplying the unmodified `_validate_row()` reproduced exactly
`ValueError: candidate publication is not accepted/fresh`.

The reconstruction independently established:

- mandatory bases were `0` and `1`, optional base `2` was active, and their
  configured positions were `[-1550,-300]`, `[-1550,0]`, and `[-1550,300]`;
- no live prediction or private prior existed;
- the selector was not considered because the active-reference count was
  three (`active_reference_count_not_two`), so the attempt path was
  `existing_predictive_multistart`, not `two_range_reacquisition`;
- `circle_negative` converged and was accepted at
  `[-1490.101851763178,-119.89233121686485]`;
- `circle_positive` converged and was accepted at
  `[-1609.898148236822,-119.89233121686485]`;
- both candidates had cost and reduced whitened cost
  `4.160177901918222e-05`, and their estimate separation was
  `119.7962964736439 m`;
- deterministic candidate ordering selected `circle_negative`; and
- the attempt was `accepted` and public output was `fresh`, but the row carried
  two accepted existing candidates.

The direct contradiction is exact: `solve_predictive_multistart()` permits
multiple accepted candidates and selects one by cost, innovation when
available, and fixed source ordering; `_validate_row()` requires a selected
existing candidate to coexist with exactly one accepted candidate.  It
therefore rejected the first row before serialization.

The deeper interpretation in the author report is appropriately bounded.
Three collinear anchors admit a global mirror ambiguity; locally acceptable
cost/FIM diagnostics do not identify the globally correct mirror.  The
selected branch's offline error was approximately `0.1482104872 m`, but truth
is unavailable at runtime and fixed source order is not a valid truth-free
disambiguator.  Because the two-reference selector was not considered, this
failure does not by itself scientifically reject the proposed private-prior
two-UAV reacquisition mechanism.

## Integrity versus scientific verdict

The integrity/protocol verdict and scientific verdict must remain separate:

- Evidence integrity is approved.  The single authorized replay was launched
  once from the frozen argv, terminated nonzero, retained a coherent failed
  manifest/empty gzip/log/ledger set, and permanently consumed attempt `1/1`.
- Protocol execution failed.  The first row exposed the solver/selector versus
  validator contract contradiction described above.
- Scientific performance was not evaluated.  There are zero serialized rows,
  no compact analyzer evidence, no 14-gate integrity vector, and no nine-gate
  scientific vector.  This is not a scientific-gate pass or failure.

The DRA terminal checkpoint agrees with these boundaries.  It records the
failure as protocol/implementation-level evidence, keeps the failed root, and
does not upgrade the result into a localization, robustness, safety, or paper
claim.

## Mandatory terminal boundary

The plan and authorization state that any nonzero exit, exception, or failed
terminal manifest consumes the invocation permanently.  Analyzer launch
requires replay return zero plus an independently verified completed
`140000/140000` manifest.  Here return code is `1`, status is `failed`, and row
count is `0/140000`.

Therefore replay retry is forbidden; analyzer remains `0/1` and must not be
started; the failed root, ledger, and logs must remain in place; and analyzer,
paper, Stage-2, estimator-tuning, manuscript-claim, submission, and push gates
remain `CLOSED`.  Any future experiment requires a new reviewed design and
protocol, a new immutable namespace, and new explicit authorization.  It may
not reuse this root or attempt.

Final independent verdict:
`APPROVED TERMINAL FORENSICS; REGISTERED-V2 PROTOCOL FAILURE; SCIENTIFIC NOT EVALUATED; NO RETRY; ANALYZER 0/1; ALL DOWNSTREAM GATES CLOSED`.
