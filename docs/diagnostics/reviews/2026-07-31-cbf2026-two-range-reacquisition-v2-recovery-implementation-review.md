# CBF2026 two-range reacquisition v2 recovery implementation review

Date: 2026-07-31

Reviewed worktree: `/private/tmp/cbf2026-diagnostic`

Reviewed branch: `codex/cbf2026-diagnostic`

Reviewed implementation range:
`12c00ff63c5757a57053e6f026e0e3cf2c9aa07b` through
`f5e1b8ff52b2590f3b8f3c18e5e81587923b9ca6`.

## Decision

The implementation closure is **not approved**.
The v2 protocol-generation gate remains `CLOSED`.

| Severity | Count |
| --- | ---: |
| Critical | `0` |
| Important | `1` |
| Minor | `0` |

This decision authorizes no protocol generation, deterministic smoke,
registered replay or analyzer, DRA append, paper edit, or paper claim.

## Critical findings

None.

## Important findings

### I1. The author report audits the wrong closed set of preserved v1 documents

The recovery plan's Global Constraints require these exact four artifacts to
remain byte-for-byte unchanged:

1. the v1 protocol JSON;
2. the v1 protocol Markdown;
3. the independent v1 protocol review; and
4. the v1 smoke report.

The author report instead labels the v1 implementation report as one of the
four preserved documents and omits
`docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1-review.md`.
Its v2 absence inventory also lists only the protocol JSON and omits the
paired v2 protocol Markdown; it does not state the complete gated v2 artifact
set.
The implementation itself did not rewrite the omitted historical review, and
the omitted v2 paths are absent, but the closure report does not preserve the
required closed-set evidence.
That omission is gate-relevant rather than editorial because Task 3 is the
non-circular provenance parent for any later protocol-v2 generation.

Required repair:

1. replace the implementation-report row in the required four-document table
   with the v1 protocol-review row below;
2. retain the implementation-report identity only as an explicitly
   supplemental historical check, if desired;
3. expand the absent v2 artifact inventory to include both protocol outputs,
   the authorization record, preflight review, smoke report, and smoke review;
4. rerun the read-only byte/hash/absence checks and obtain a fresh independent
   C0/I0/M0 review before protocol generation.

The independently observed omitted identity is:

| Document | Bytes | SHA-256 | Range state |
| --- | ---: | --- | --- |
| `docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1-review.md` | `14948` | `6ac2da2d1fdcd173de04d1e9658560ac25439e79d03f1b5649d63f62da805cfa` | unchanged from `12c00ff` through `f5e1b8f` |

## Minor findings

None.

## Standards

No documented-style or code-quality finding was identified in the
implementation range.
The six-file implementation diff is bounded to the recovery plan, replay and
registrar production modules, and their direct tests.
No unrelated tracked file or DRA file changed.
No new smell in the changed production logic rises to a finding.

## Spec verification

Except for I1, the implementation satisfies the recovery specification.

### Serialized-entry RED and GREEN

The new regression independently constructs all 11 declared identities with
`Path.stat()` and SHA-256, serializes and reloads JSON, calls the real
`replay_two_range_reacquisition(...)` public entry point for `smoke_a`, and
checks the completed 18-row manifest and exact local source order.

An independent in-memory restoration of the v1 equality defect,
`tuple(declared_sources) == tuple(observed_sources)`, made that exact
serialized-entry regression fail with:

```text
ValueError: protocol source members differ from contract
Ran 1 test in 0.008s
FAILED (errors=1)
```

No output root was allocated.
The current implementation passed the exact focused regression:

```text
Ran 1 test in 0.045s
OK
```

This independently corroborates the Task 1 historical RED record
(`1` error in `0.029s`) and GREEN record (`1/1` in `0.038s`).

### Exact 11-global/5-local source contract

The global ordered declaration remains:

```text
implementation_plan
two_range_reacquisition_source
predictive_wnls_source
fixture_extractor_source
replay_source
analyzer_source
registrar_source
mechanism_fixture
mechanism_fixture_manifest
truth_data
input_manifest
```

The exact ordered local `smoke_a` observation remains:

```text
two_range_reacquisition_source
predictive_wnls_source
replay_source
mechanism_fixture
mechanism_fixture_manifest
```

Every global declaration must have the exact ordered identity fields:

```text
path
device
inode
size
mtime_ns
sha256
```

The public validator rejects missing, extra, or reordered global members;
missing or reordered identity fields; local declaration path/hash drift; and
missing, extra, reordered, or identity-drifted local observations before
`_create_exact_root(...)`.
The table-driven mutation regression passed all 10 subcases in `0.037s` and
confirmed root absence.

Registered replay first applies the same exact global/local binding, then
rereads and compares every one of the 11 declared live files.
It therefore remains stricter than smoke validation and was not weakened by
the recovery.

### Exact v2 namespace and v1 retirement

The production identities are exactly:

| Contract | Identity |
| --- | --- |
| Protocol schema | `cbf2026-two-range-reacquisition-protocol-v2` |
| Registration schema | `cbf2026-two-range-reacquisition-registration-v2` |
| Protocol ID | `cbf2026-two-range-reacquisition-v2` |
| Raw schema | `cbf2026-two-range-reacquisition-raw-v1` |
| Analysis schema | `cbf2026-two-range-reacquisition-analysis-v1` |
| Measurement seed contract | `cbf2026-range-v1` |

The exact current roots are:

```text
/private/tmp/cbf2026-two-range-reacquisition-smoke-v2-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-v2-b
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v2-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v2-b
/private/tmp/cbf2026-two-range-reacquisition-development/v2
/private/tmp/cbf2026-two-range-reacquisition-analysis/v2
```

The exact retired tuple contains the corresponding six v1 roots in the same
invocation order.
`_assert_registered_roots_absent()` binds both tuples and rejects a directory
or broken symlink at any of the 12 paths before protocol publication.

The exact gated v2 document paths are:

```text
docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.md
docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json
docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-registered-v2-authorization.json
docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2-review.md
docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-smoke-v2.md
docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-smoke-v2-review.md
```

The six literal command arrays are present in exact order for `smoke_a`,
`smoke_b`, `smoke_analyzer_a`, `smoke_analyzer_b`, `registered_replay`, and
`registered_analyzer`.
Their protocol, raw-root, output-root, authorization, seed, frame, and
invocation tokens match the v2 contract; current declarations and arrays
contain no retired v1 protocol/registration identity, path, or root.
The targeted namespace/command/root-guard verification passed `11/11` tests
in `1.303s`.

### Scientific-contract preservation

The implementation range does not change
`two_range_reacquisition.py`, `predictive_wnls.py`,
`extract_two_range_reacquisition_fixture.py`,
`analyze_two_range_reacquisition.py`, or the fixture tree.
No changed production line touches row fields, raw-manifest fields, analysis
fields, experiment constants, estimator constants, thresholds, comparators,
scientific gates, 14 integrity gates, the 18 smoke cases, the 20 seeds,
500-frame budget, 14-UAV grid, 140000-key order, truth data, input manifest,
fixture identity, or no-retry semantics.

The raw and analysis schemas correctly remain v1 because their field orders
and validators did not change.
Only the enclosing protocol/registration provenance namespace advances to v2.

### Historical preservation and absence

The correct four required v1 artifacts are unchanged across the reviewed
range:

| Document | Bytes | SHA-256 |
| --- | ---: | --- |
| `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json` | `69464` | `8e01df56623bb2e759fb37108ddc68fbc41e301b8305ade7322e212ae6e3ea16` |
| `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.md` | `4137` | `c9e3930d22355747ee12e96303c1702aa0385de2881f6b430a72030b89d7e0ff` |
| `docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1-review.md` | `14948` | `6ac2da2d1fdcd173de04d1e9658560ac25439e79d03f1b5649d63f62da805cfa` |
| `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-smoke.md` | `13904` | `404de547db5fb7711f1fa95132cff8399b3fd432a9871c4e600121784f0af8db` |

All six retired v1 roots and all six current v2 roots were absent under
`-e`/`-L` checks.
Both old and current authorization records were absent.
All six gated v2 document paths above were absent.
The reviewed commit range changes no DRA file.

The preserved v1 smoke report still records terminal `FAIL`, the exact source
member exception, `retry_allowed: false` for every invocation, no later v1
command, absent registered roots/authorization, and paper gate `CLOSED`.

### Verification evidence

Task 2 recorded:

```text
register module: 50/50 OK
replay module: 63/63 OK
analyzer module: 172/172 OK
full discovery: 870/870 OK in 152.566s
```

The author closure independently recorded a second full-discovery pass:
`870/870` in `164.895s`.
This reviewer did not repeat the full suite.

Independent focused verification additionally established:

```text
serialized public-entry GREEN: 1/1 OK
source mutation matrix: 1/1 test, 10 subcases, OK
namespace/commands/root guards: 11/11 OK
shadow/direct-bootstrap regressions: 3/3 OK
six-file py_compile: exit 0
replay/analyzer/registrar --help: 3/3 exit 0
git diff --check for 12c00ff..f5e1b8f: exit 0
```

At review start, `HEAD` was exactly
`f5e1b8ff52b2590f3b8f3c18e5e81587923b9ca6`;
the tracked and staged diffs were empty.
Only the pre-existing untracked `build-diagnostic/` and the author
implementation report were outside `HEAD`.
No protocol command was executed, no exact evidence root was created, and no
file was staged or committed during this review.

## Closure boundary

The implementation code is technically clean against the requested recovery
contract, but Important finding I1 keeps the implementation closure open.
After the author report is corrected, the complete review must be rerun and
all three severity counts must be zero before any v2 protocol-generation
step.

## Fix round 1 scoped re-review

Scope: only Important finding I1 and the directly changed preservation and
absence inventories were rereviewed.
No new broad implementation review was performed.

### Finding disposition

I1 is **ADDRESSED**.

The author report now lists exactly the four v1 artifacts required by Global
Constraints:

| Document | Bytes | SHA-256 | `12c00ff..HEAD` |
| --- | ---: | --- | --- |
| `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json` | `69464` | `8e01df56623bb2e759fb37108ddc68fbc41e301b8305ade7322e212ae6e3ea16` | unchanged |
| `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.md` | `4137` | `c9e3930d22355747ee12e96303c1702aa0385de2881f6b430a72030b89d7e0ff` | unchanged |
| `docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1-review.md` | `14948` | `6ac2da2d1fdcd173de04d1e9658560ac25439e79d03f1b5649d63f62da805cfa` | unchanged |
| `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-smoke.md` | `13904` | `404de547db5fb7711f1fa95132cff8399b3fd432a9871c4e600121784f0af8db` | unchanged |

The incorrect implementation-report row is no longer in the required
four-document table.
The absent-v2 inventory now includes both exact protocol outputs:

```text
docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json
docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.md
```

Independent `-e`/`-L` checks confirmed both paths remain absent.

### New breakage

None within the scoped fix.

### Final counts

| Severity | Final count |
| --- | ---: |
| Critical | `0` |
| Important | `0` |
| Minor | `0` |

APPROVED FOR V2 PROTOCOL GENERATION; EXECUTION GATES CLOSED

This appended scoped disposition supersedes the initial decision and closure
boundary above.
This approval closes only the implementation-recovery parent and permits the
separately gated protocol-v2 generation task to begin.
It does not approve any smoke command, registered command, authorization
record, DRA append, paper edit, or paper claim.

## Fix round 2 scoped re-review

Scope: only the two open findings concerning the six-path gated-v2 artifact
inventory and the incomplete round 1 re-review were examined.
No new broad implementation review was performed.

The round 1 approval was incomplete because it checked only the protocol JSON
and Markdown paths and did not verify the required smoke-review path.
Its approval sentence is retained above solely as error history and carries
no authority.
This round 2 section supersedes the entire round 1 disposition.

### Finding dispositions

1. **Author inventory omitted the smoke-v2 review path — ADDRESSED.**
   The author report now says `six` and contains exactly these ordered paths,
   with no missing or extra member:

   ```text
   docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json
   docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.md
   docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-registered-v2-authorization.json
   docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2-review.md
   docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-smoke-v2.md
   docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-smoke-v2-review.md
   ```

2. **Round 1 marked I1 addressed without checking all six paths —
   ADDRESSED.**
   This reviewer performed an independent lstat-aware check of every path
   above.
   For each path, both `-e` and `-L` were false, so all six are absent and
   none is a broken symlink.

### New breakage

None within the scoped round 2 fix.

### Final counts

| Severity | Final count |
| --- | ---: |
| Critical | `0` |
| Important | `0` |
| Minor | `0` |

APPROVED FOR V2 PROTOCOL GENERATION; EXECUTION GATES CLOSED

This round 2 approval closes only the implementation-recovery parent and
permits the separately gated protocol-v2 generation task to begin.
It does not approve protocol execution, deterministic smoke, registered
replay or analysis, an authorization record, DRA changes, paper edits, or
paper claims.
