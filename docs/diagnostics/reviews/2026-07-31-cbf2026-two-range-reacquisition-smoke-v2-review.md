# CBF2026 two-range reacquisition v2 deterministic-smoke independent review

Date: 2026-07-31

Reviewed worktree: `/private/tmp/cbf2026-diagnostic`

Reviewed HEAD: `84aa6851bfa61468f3e0a6555d081fe124fb2395`

## Decision

| Severity | Count |
| --- | ---: |
| Critical | `0` |
| Important | `0` |
| Minor | `0` |

APPROVED: V2 DETERMINISTIC SMOKE PASS; REGISTERED GATE CLOSED

This approval covers only deterministic execution and integrity of the four
frozen v2 smoke commands.  Both analyzers deliberately evaluated zero
scientific gates.  Their `smoke_pass` decisions are not evidence of
scientific performance, safety, robustness, or a paper claim.

The registered replay, registered analyzer, authorization, Stage 2, DRA, and
paper gates remain `CLOSED`.

## Review scope and evidence boundary

The review read the exact Task 5 brief, author smoke report, Task 5 operator
report and execution ledger, committed protocol v2 JSON and Markdown, formal
protocol-v2 preflight review, and Task 4 preflight/process reports.  It then
read and revalidated the manifests, process data, analysis JSON, and analysis
Markdown directly from all four retained roots.

No protocol command was executed.  No root was created, deleted, renamed,
overwritten, or modified.  No source, test, protocol, author report, DRA,
diagnostic index, Git index, commit, authorization, or paper file was edited.

Current artifact bytes, filesystem identities, hashes, schemas, row content,
semantic projections, Git state, binding identities, disk allocation, and
root/authorization absences are independently reproducible from retained
state.  Historical process facts such as exact `Popen` counts, command return
codes, stdout/stderr capture, and live free-space extrema are not
independently reconstructible after process exit; those facts are supported
by the preserved execution ledger and author/operator reports.  The review
does not silently promote ledger-only process history into independently
observed runtime evidence.

## Frozen documents and source state

The worktree still resolves exactly to the smoke-only preflight commit above.
Its sole parent is the protocol commit
`5241d6a4d3ab8144ec065dc2f92c4c454a1d3760`, whose declared implementation
parent is `526e1418138426e54bff5a41e3847b6d2a9f8203`.
Unstaged tracked and staged diffs are empty.  Before this review was created,
normal porcelain state contained only the pre-existing untracked
`build-diagnostic/` directory and author smoke report.

The live files and their blobs at `84aa685` are byte-identical:

| Frozen document | SHA-256 |
| --- | --- |
| protocol v2 JSON | `9a0985e00e1b6bd801f7b46c3811c2e135c29f86f75b5045e7aff1ad71206bb2` |
| protocol v2 Markdown | `945a98760b2af676dedda513ea10c1e9c9c2a940d22a9ded689a11c0299f4251` |
| formal smoke-only preflight | `980a99f2cdcafc7ccf6cd316fb1690e629d619cd6d6d594b3490c0e4b338e6e0` |

All 11 current source records independently matched their pinned path,
device, inode, size, nanosecond mtime, and SHA-256 fields.  The nine repository
sources remain bound to the implementation-parent bytes; the truth data and
input manifest remain exact external bindings.  All seven comparator records
also matched their pinned filesystem identities and hashes.  The v4 gzip
comparator was independently hashed in both the compressed-file and
decompressed-JSONL domains.

The enclosing identities are exact:

```text
protocol schema     cbf2026-two-range-reacquisition-protocol-v2
registration schema cbf2026-two-range-reacquisition-registration-v2
protocol ID         cbf2026-two-range-reacquisition-v2
raw schema          cbf2026-two-range-reacquisition-raw-v1
analysis schema     cbf2026-two-range-reacquisition-analysis-v1
```

The raw and analysis schemas correctly remain v1 because their serialized
field contracts did not change.

## Retained-root identities and terminal evidence

Each retained root is the exact absolute root declared by its invocation, is
a real non-symlink directory, and contains only the frozen terminal file set.
The directory device/inode identities observed during review and the
independently recomputed artifact hashes are:

| Root | Device:inode | Terminal manifest SHA-256 | Bound output identities |
| --- | --- | --- | --- |
| raw A | `16777234:181126098` | `dc98d774db8300c3b397c3e40422d15b1ba46c394eb29b22b3ee85f7f4391207` | compressed `8073457c8e7a21a5ec7c6d85baaec4c77a549ea66422e474ad927f323160677c`; decompressed `cdbde96b17219f69cf0c00f403718048d8d0d04b4943ce53d3580978f240d435` |
| analysis A | `16777234:181126119` | `07079824d6abb613da295056f587ca514a5d6bc75437aabb9d08c15165434f0f` | JSON `26753574fa3e9217f18180a2bf0c51a34d30b763134913e483a2dbbe9e378c0d`; Markdown `3f49d592cfd127ef751d638838975b5f2a55655a88944290b5eaf2a4a485b526` |
| raw B | `16777234:181126143` | `3684a69121d273a6eaec364b1fbd3d173a6648571f800f20beccc79d1012e628` | compressed `8073457c8e7a21a5ec7c6d85baaec4c77a549ea66422e474ad927f323160677c`; decompressed `cdbde96b17219f69cf0c00f403718048d8d0d04b4943ce53d3580978f240d435` |
| analysis B | `16777234:181126160` | `a0d6bd059c18a2d93241a4478aec63e3f9953b42d7e2373512898c0fbbe6eb85` | JSON `0679640eb15f77a66ad3c8b2ecbcf9f8d6e6290b1d8821ce4869d789d73f2c3c`; Markdown `80d68acd68cf84d7f90bb072b0496743fad51856aee45954cc7d5063440fb13e` |

Both raw manifests have `status="completed"`, the exact raw schema and v2
protocol ID, null authorization and error records, exact invocation/root
bindings, and `expected_rows=observed_rows=18`.  Their process identities
match the current no-follow device, inode, size, allocated-byte, nanosecond
mtime, compressed-hash, and decompressed-hash observations.

Independent decompression produced 18 strict rows in each root.  Every row
has the exact 58-field order declared by the raw schema, the frozen method,
and `invocation_name="smoke_validation"`.  Both files contain the same exact
ordered cases:

```text
mechanism_20260727_180_12
select_negative
select_positive
q_equal_threshold
q_below_threshold
q_above_threshold
none_pass
multiple_pass
tangent
disjoint
contained
coincident
zero_range
nearly_collinear
merged_solver_branches
invalid_private_covariance
cost_equal_nine
cost_above_nine
```

Both analyzer manifests have `status="completed"`, the exact analysis schema
and v2 protocol ID, null authorization and error records, exact raw-source and
output identities, and `expected_rows=observed_rows=18`.  Both analysis JSON
objects preserve the declared field order, report zero scientific gates,
contain the exact ordered 14 integrity gates with all `14/14` passing, and
have `decision="smoke_pass"`.

## Independent deterministic comparison

| Comparison | A | B | Result |
| --- | --- | --- | --- |
| compressed raw SHA-256 | `8073457c8e7a21a5ec7c6d85baaec4c77a549ea66422e474ad927f323160677c` | same | equal |
| decompressed raw SHA-256 | `cdbde96b17219f69cf0c00f403718048d8d0d04b4943ce53d3580978f240d435` | same | equal |
| semantic payload SHA-256 | `55c636e41c121e60eaa2341d0524ff5c08838374969343eec1e9f916a1758247` | same | equal |
| ordered raw rows | exact 18-case order | exact 18-case order | equal |
| normalized Markdown | decision, rows, semantic payload | same | equal |

The semantic hash was independently recomputed without calling the analyzer.
The review projected the exact 15 protocol-declared semantic fields, sourced
the three row budgets from `budgets`, serialized strict UTF-8 compact JSON in
declared order with no non-finite values, and obtained
`55c636e41c121e60eaa2341d0524ff5c08838374969343eec1e9f916a1758247`
for both A and B.  Removing only the permitted invocation line from each
Markdown file leaves byte-equal semantic lines for `smoke_pass`, `18/18`, and
the same semantic hash.

## Command identity, sequence, and history boundary

The four ledger argv arrays are byte-for-byte value/order equal to the
corresponding committed protocol arrays.  Independent compact-JSON hashing
produced:

| Invocation | Tokens | argv SHA-256 | Ledger command window UTC |
| --- | ---: | --- | --- |
| `smoke_a` | `19` | `647ad407d13c0d6cf9dba83582216d7bea5205f0d850bc714d598abacbe14067` | `10:32:02.383859`--`10:32:03.468347` |
| `smoke_analyzer_a` | `14` | `b1fbd68545a2da0626c1fcb0e480503171020afa035919def6f1787b1e7ddbf8` | `10:32:08.183197`--`10:32:09.160499` |
| `smoke_b` | `19` | `fdaef40962e90e1fa03f8d69e803b20ffa4804986f36fd87a6647b1a04147196` | `10:32:13.962658`--`10:32:14.946691` |
| `smoke_analyzer_b` | `14` | `1b08d1e595eb12931d2c0c964b01cb00fd2639dc0fca907279c04bab18556384` | `10:32:19.254233`--`10:32:20.129144` |

The ledger sequence and successful-invocation sequence are both exactly
`smoke_a`, `smoke_analyzer_a`, `smoke_b`, `smoke_analyzer_b`; each counter is
one.  The four command windows are strictly ordered and non-overlapping.  For
each command, the retained manifest's producer start and completion times lie
inside its ledger window and in the expected order.  These cross-artifact
relations are independently checkable.  The facts that each process was
actually created once, returned zero, emitted empty stdout/stderr, and had no
live monitor failure remain process-history claims supported by the ledger
and concordant author/operator reports, not by a second execution.

The corrected harness statically restricts execution to the four-name
sequence above; registered names and roots are absent from that loop.  It
persists a counter before direct `Popen(argv)` and performs frozen-document
validation before ledger initialization and before the only `Popen` site.

## Attempt-1 defect and transparent fix round

The preserved failed value is exactly 62 characters:

```text
980a99f2cdcafc7ccf6cd316fb1690e629d619cd6d594b3490c0e4b338e6e0
```

The intact committed preflight value is exactly 64 characters and includes
the omitted `6d`:

```text
980a99f2cdcafc7ccf6cd316fb1690e629d619cd6d6d594b3490c0e4b338e6e0
```

The current committed preflight blob and live file independently hash to the
64-character value.  Static control-flow inspection confirms that this
frozen-document mismatch is raised before ledger creation and before the
sole protocol-command `Popen` site.  The author report and operator report
preserve the initial failure rather than erasing it, and the final ledger
records attempt 1 as return `1`, protocol `Popen` count `0`, counters/roots
unconsumed, followed by the explicitly authorized corrected attempt 2 with
four protocol `Popen` calls and terminal success.

The exact historical bytes of the discarded attempt-1 harness and its live
process state are not retained, so its zero-`Popen`, zero-counter, and
zero-root facts cannot be re-observed directly now.  They are supported by
the mutually consistent author/operator chronology, final ledger history,
the preserved post-failure absence snapshot, and the corrected harness's
control-flow shape.  This limitation is disclosed rather than treated as
independent runtime proof; it creates no report discrepancy or gate finding.

## Disk, absence, and closed-boundary checks

The protocol caps are launch free space `8,000,000,000`, live free space
`6,000,000,000`, raw root allocation `2,000,000,000`, and compact root
allocation `10,000,000` bytes.  The ledger's four launch readings all exceed
8 GB and its minimum recorded live reading is `17,295,896,576` bytes.  It
records maximum raw-root allocation `8,192` bytes, maximum compact-root
allocation `16,384` bytes, and final aggregate allocation `49,152` bytes.
Independent current allocation is exactly the same: `8,192` bytes for each
raw root, `16,384` bytes for each compact root, and `49,152` bytes total.
Current `/private/tmp` free space during review was `14,677,073,920` bytes,
above both floors.  Historical live extrema remain ledger-supported process
observations; current root allocation and free space are independently
observed.

Fresh broken-symlink-aware checks found all six retired v1 roots absent, both
registered v2 roots absent, and both v1/v2 authorization records absent.
The v2 protocol still declares
`registered_full_grid_authorization="pending_external_record"`.
The ledger and both reports record registered replay/analyzer counters as
zero, while the current corrected operator allowlist contains only the four
smoke commands.  As with all exited-process history, non-execution cannot be
reconstructed solely from current filesystem state; it is supported by that
allowlist, the ledger/report chronology, and continued absence of both
authorization records and both registered roots.

HEAD, tracked files, and the Git index remain unchanged by smoke execution.
No registered root or authorization exists, and the author report explicitly
keeps registered, Stage 2, DRA, and paper work closed.  No scientific gate
was run, so the deterministic smoke result must not be used as performance
evidence.

## Findings

### Critical

None.

### Important

None.

### Minor

None.

## Final verdict

APPROVED: V2 DETERMINISTIC SMOKE PASS; REGISTERED GATE CLOSED
