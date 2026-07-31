# CBF2026 Two-Range Smoke v2 Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development to implement this plan task-by-task.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Repair the protocol-wide/source-subset integration defect that
terminated protocol v1, freeze a provenance-isolated protocol v2, and execute
only its deterministic 18-case smoke pairs while keeping the 140000-row
registered run unauthorized.

**Architecture:** The serialized protocol keeps one exact ordered global map
of 11 source identities, while each replay invocation keeps its exact ordered
five-member runtime snapshot.  Smoke validation first checks the global map
shape and then compares its five invocation-local identities exactly; it never
shrinks the protocol map or accepts undeclared members.  Protocol and
authorization identities, artifact paths, and all six execution roots advance
to v2, while the unchanged raw and analysis record layouts retain schema v1.

**Tech Stack:** Python 3, `unittest`, canonical JSON, conda environment
`cbf_env`, Git, Markdown, and append-only DRA records.

## Global Constraints

- Work only in `/private/tmp/cbf2026-diagnostic` on
  `codex/cbf2026-diagnostic`; DRA updates work only in
  `/private/tmp/dra-cbf2026-diagnostic` on `main`.
- Use `conda run -n cbf_env` for every Python command.
- Follow strict RED-GREEN-REFACTOR: no production change before a real failing
  regression has been observed and recorded.
- Preserve these v1 artifacts byte-for-byte:
  `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json`,
  `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.md`,
  `docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1-review.md`,
  and
  `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-smoke.md`.
- The six v1 root names are permanently retired even though the pre-root
  failure left them absent.  They must remain absent and must never be created,
  reused, copied, hard-linked, or symlinked.
- The old authorization path
  `docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-registered-authorization.json`
  remains absent permanently.
- Preserve the approved design, 18 smoke cases, fixture, truth trajectory,
  input manifest, seven comparator identities, estimator constants, 20 seeds,
  500 frames, 14 UAVs, 140000-key order, nine scientific gates, 14 integrity
  gates, thresholds, noise model, and no-retry semantics.
- Protocol schema is
  `cbf2026-two-range-reacquisition-protocol-v2`.
- Registration schema is
  `cbf2026-two-range-reacquisition-registration-v2`.
- Protocol ID is `cbf2026-two-range-reacquisition-v2`.
- Raw schema remains `cbf2026-two-range-reacquisition-raw-v1`.
- Analysis schema remains
  `cbf2026-two-range-reacquisition-analysis-v1`.
- The exact v2 protocol paths are
  `docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json`
  and
  `docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.md`.
- The exact v2 authorization path is
  `docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-registered-v2-authorization.json`.
- The exact v2 preflight review path is
  `docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2-review.md`.
- The exact v2 smoke report and review paths are
  `docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-smoke-v2.md`
  and
  `docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-smoke-v2-review.md`.
- The exact v2 roots are:

```text
/private/tmp/cbf2026-two-range-reacquisition-smoke-v2-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-v2-b
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v2-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v2-b
/private/tmp/cbf2026-two-range-reacquisition-development/v2
/private/tmp/cbf2026-two-range-reacquisition-analysis/v2
```

- Before registration, execution is limited to unit/adversarial tests,
  non-publishing protocol dry builds, and the four exact deterministic v2
  smoke command arrays.
- The latest user instruction `继续`, following the explicit v2 recovery
  sequence, authorizes this implementation, protocol v2 generation,
  smoke-only preflight, and the four deterministic v2 smoke invocations.  It
  does not authorize either registered command.
- Never create the v2 registered replay or analyzer root and never create the
  v2 authorization JSON in this plan.
- A failed v2 smoke invocation consumes the complete v2 root namespace:
  retain any terminal evidence, stop immediately, execute no later command,
  record the failure, update DRA, and do not retry.
- Require at least `8_000_000_000` free bytes before every execution,
  stop below `6_000_000_000`, and keep the rebuildable raw/cache allocation
  below `2_000_000_000`.
- Commits are authorized for this recovery workflow.  Use
  `type(scope): description`, add no `Co-Authored-By`, do not push, and do not
  modify unrelated `build-diagnostic/`.
- DRA changes are append-only in
  `meta-log/2026-07-30-cbf2026-predictive-wnls-recovery.md`,
  `papers/cbf2026/open-questions.md`,
  `papers/cbf2026/status.md`, and
  `papers/cbf2026/timeline.md`.

---

## File Map

- Modify
  `scripts/diagnostics/replay_two_range_reacquisition.py`:
  validate an exact 11-member protocol source map against the exact
  invocation-local five-member runtime source snapshot; bind v2 registered
  identities and paths.
- Modify
  `scripts/diagnostics/register_two_range_reacquisition.py`:
  generate only the v2 protocol, commands, authorization path, and roots;
  bind this recovery plan as `implementation_plan`; reject retired v1 roots.
- Inspect and modify only if a failing version-path test requires it:
  `scripts/diagnostics/analyze_two_range_reacquisition.py`.
  Its registered protocol and authorization paths should continue to derive
  from the replay module rather than duplicating literals.
- Modify
  `tests/test_replay_two_range_reacquisition.py`:
  add the serialized full-protocol smoke regression and fail-closed source
  mutations; update registered v2 identities.
- Modify
  `tests/test_register_two_range_reacquisition.py`:
  pin v2 identities, roots, paths, commands, retired-v1 guards, and unchanged
  raw/analysis schemas.
- Modify only if required by existing literal assertions:
  `tests/test_analyze_two_range_reacquisition.py`.
- Create
  `docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-v2-recovery-implementation.md`.
- Create
  `docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-v2-recovery-implementation-review.md`.
- Create the exact v2 protocol, preflight, smoke, and smoke-review paths listed
  in Global Constraints only at their gated tasks.
- Append the four DRA files listed in Global Constraints after implementation,
  protocol/preflight, and smoke closure.

### Task 1: TDD the global-protocol/local-snapshot source contract

**Files:**
- Modify: `tests/test_replay_two_range_reacquisition.py`
- Modify: `scripts/diagnostics/replay_two_range_reacquisition.py`

**Interfaces:**
- Consumes:
  `REGISTERED_PROTOCOL_SOURCE_NAMES`,
  `RAW_SOURCE_MEMBER_NAMES`,
  `FILE_IDENTITY_FIELDS`,
  `_source_snapshots(...)`, and
  `replay_two_range_reacquisition(...)`.
- Produces:
  `_validate_protocol_source_binding(*, declared_sources: Mapping,
  observed_sources: Mapping, invocation_name: str) -> None`.

- [ ] **Step 1: Write a real serialized-protocol regression**

Add
`ProducerLifecycleTests.test_serialized_full_protocol_smoke_accepts_exact_local_source_subset`.
The test must:

1. construct all 11 declarations in exact
   `REGISTERED_PROTOCOL_SOURCE_NAMES` order;
2. derive complete literal file identities independently with `Path.stat()`,
   `hashlib.sha256(path.read_bytes()).hexdigest()`, and the exact
   `FILE_IDENTITY_FIELDS` order;
3. use the five actual paths required by `smoke_a`;
4. use existing regular files for the six declarations not read by
   `smoke_a`;
5. serialize then reload the protocol JSON;
6. call the real `replay_two_range_reacquisition(...)` entry point with
   `invocation_name="smoke_a"`;
7. assert a completed 18-row manifest and exact local source-member order
   `RAW_SOURCE_MEMBER_NAMES["smoke_a"]`.

Before writing the test body, record the mutation it catches:
changing nonregistered source validation back to
`tuple(declared_sources) == tuple(observed_sources)` must fail this test.

- [ ] **Step 2: Run the regression and verify RED**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_replay_two_range_reacquisition.ProducerLifecycleTests.test_serialized_full_protocol_smoke_accepts_exact_local_source_subset \
  -v
```

Expected: FAIL before root allocation with
`ValueError: protocol source members differ from contract`, proving the v1
terminal failure is reproduced rather than merely asserted as source text.
Remove the temporary directory automatically; never point this test at a v1
or v2 production root.

- [ ] **Step 3: Implement the minimal exact binding validator**

Implement
`_validate_protocol_source_binding(...)` with this behavior:

```python
if not isinstance(declared_sources, Mapping):
    raise ValueError("protocol source members differ from contract")
if tuple(declared_sources) != REGISTERED_PROTOCOL_SOURCE_NAMES:
    raise ValueError("protocol source members differ from contract")
if tuple(observed_sources) != RAW_SOURCE_MEMBER_NAMES[invocation_name]:
    raise ValueError("runtime source members differ from invocation")
for name, declaration in declared_sources.items():
    if (
        not isinstance(declaration, Mapping)
        or tuple(declaration) != FILE_IDENTITY_FIELDS
    ):
        raise ValueError(f"protocol source identity differs: {name}")
for name, observed_identity in observed_sources.items():
    if declared_sources[name] != observed_identity:
        raise ValueError(f"protocol source identity differs: {name}")
```

Call it whenever `declared_sources is not None`, for smoke and registered
replay.  Keep the existing registered-only validation of every global source
against its live file, including reading global declarations outside the
registered local snapshot.  Do not reduce the 11-member protocol declaration,
do not allow reordered or extra members, and do not weaken a registered
identity check.

- [ ] **Step 4: Verify GREEN**

Run the exact Step 2 command.

Expected: PASS, one real 18-row completed smoke bundle in the test temporary
directory, removed by test cleanup.

- [ ] **Step 5: Add fail-closed mutation tests**

Add table-driven tests using independently constructed serialized protocol
fixtures for:

- one missing global declaration;
- one extra global declaration;
- two reordered global declarations;
- one required local declaration with changed path;
- one required local declaration with changed `sha256`;
- one global declaration with missing or reordered identity field;
- an observed local map with a missing, extra, or reordered member.

Each case must fail before `_create_exact_root` runs.  Assert the real
exception category/message and root absence; do not assert on mocks as the
test outcome.

- [ ] **Step 6: Run focused and producer suites**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_replay_two_range_reacquisition.ProducerLifecycleTests -v
conda run -n cbf_env python -m unittest \
  tests.test_replay_two_range_reacquisition -v
```

Expected: all pass with no warning or unexpected output.

- [ ] **Step 7: Commit Task 1**

```bash
git add \
  docs/superpowers/plans/2026-07-31-cbf2026-two-range-smoke-v2-recovery.md \
  scripts/diagnostics/replay_two_range_reacquisition.py \
  tests/test_replay_two_range_reacquisition.py
git commit -m "fix(diagnostics): validate smoke source subset"
```

### Task 2: TDD the provenance-isolated v2 production namespace

**Files:**
- Modify: `tests/test_replay_two_range_reacquisition.py`
- Modify: `tests/test_register_two_range_reacquisition.py`
- Modify if required: `tests/test_analyze_two_range_reacquisition.py`
- Modify: `scripts/diagnostics/replay_two_range_reacquisition.py`
- Modify: `scripts/diagnostics/register_two_range_reacquisition.py`
- Modify only if required:
  `scripts/diagnostics/analyze_two_range_reacquisition.py`

**Interfaces:**
- Consumes: Task 1 source-binding behavior and all frozen scientific schemas.
- Produces: exact v2 protocol/authorization identities, paths, root map,
  retired-v1 root guard, and six literal command arrays.

- [ ] **Step 1: Write v2 namespace tests first**

Update or add tests that require exactly:

```text
protocol schema  cbf2026-two-range-reacquisition-protocol-v2
registration     cbf2026-two-range-reacquisition-registration-v2
protocol ID      cbf2026-two-range-reacquisition-v2
raw schema       cbf2026-two-range-reacquisition-raw-v1
analysis schema  cbf2026-two-range-reacquisition-analysis-v1
```

Pin the six v2 roots and all four v2 document paths from Global Constraints.
Pin the complete six command arrays so every protocol, raw-root, output-root,
and authorization token is v2.  Pin
`REPOSITORY_SOURCE_PATHS["implementation_plan"]` to this plan file.

Add a test proving every v1 root is in an immutable ordered
`RETIRED_ROOTS` tuple and `_assert_registered_roots_absent()` rejects any
pre-existing current v2 root or retired v1 root, including a broken symlink,
before protocol output creation.  Never create an exact v1 path during a
test: patch the root tuple to an isolated temporary-path analogue, exercise
the real filesystem check there, and restore the production tuple
automatically.

Add negative tests proving no v1 protocol schema, protocol ID, authorization
schema/path, protocol path, or root appears in current production
declarations or command arrays.  The historical v1 documents are not imported
as current declarations and are not modified by these tests.

- [ ] **Step 2: Run version tests and verify RED**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_register_two_range_reacquisition -v
conda run -n cbf_env python -m unittest \
  tests.test_replay_two_range_reacquisition -v
conda run -n cbf_env python -m unittest \
  tests.test_analyze_two_range_reacquisition -v
```

Expected: the new literal tests fail because current production declarations
still name v1.  Existing behavior tests may also fail only where their
expected production literals need the same v2 update.

- [ ] **Step 3: Implement the minimal v2 namespace**

Change only production protocol/registration identities, document paths,
root paths, current authorization path, current report paths, command arrays,
and recovery-plan binding.  Add:

```python
RETIRED_ROOTS = (
    "/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-a",
    "/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-b",
    "/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-a",
    "/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-b",
    "/private/tmp/cbf2026-two-range-reacquisition-development/v1",
    "/private/tmp/cbf2026-two-range-reacquisition-analysis/v1",
)
```

Keep `RAW_SCHEMA_ID` and `ANALYSIS_SCHEMA_ID` unchanged.  Keep the method,
row fields, analysis fields, gates, comparators, experiment, estimator,
fixtures, and thresholds unchanged.  Analyzer literals should continue to
derive registered protocol/authorization paths and ID from `replay`; do not
duplicate v2 literals unless a strict local schema requires them.

- [ ] **Step 4: Verify GREEN and run all implementation regressions**

Run:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_register_two_range_reacquisition -v
conda run -n cbf_env python -m unittest \
  tests.test_replay_two_range_reacquisition -v
conda run -n cbf_env python -m unittest \
  tests.test_analyze_two_range_reacquisition -v
conda run -n cbf_env python -m unittest discover -s tests -p 'test_*.py' -v
conda run -n cbf_env python -m py_compile \
  scripts/diagnostics/two_range_reacquisition.py \
  scripts/diagnostics/predictive_wnls.py \
  scripts/diagnostics/extract_two_range_reacquisition_fixture.py \
  scripts/diagnostics/replay_two_range_reacquisition.py \
  scripts/diagnostics/analyze_two_range_reacquisition.py \
  scripts/diagnostics/register_two_range_reacquisition.py
git diff --check
```

Also run direct CLI help from the repository root and the existing shadow
import regressions for replay, analyzer, and registrar.  Expected: all pass;
the only working-tree item outside the task diff remains
`?? build-diagnostic/`.

- [ ] **Step 5: Commit Task 2**

Stage only modified Task 2 sources and tests:

```bash
git commit -m "fix(diagnostics): retire two-range protocol v1"
```

### Task 3: Independently close the v2 implementation

**Files:**
- Create:
  `docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-v2-recovery-implementation.md`
- Create:
  `docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-v2-recovery-implementation-review.md`
- Append the four DRA files from Global Constraints.

**Interfaces:**
- Consumes: Task 1 and Task 2 commits and their test reports.
- Produces: a committed C0/I0/M0 implementation closure suitable as the
  non-circular v2 protocol parent.

- [ ] **Step 1: Write the implementation report**

Record:

- the v1 terminal failure and no-retry boundary;
- exact Task 1 RED command and expected failure;
- exact GREEN and full-suite commands/counts;
- source-contract semantics: 11 global declarations, five exact local
  observations;
- v2 identities and six v2 roots;
- proof raw/analysis schemas remain v1 because their fields and validation
  semantics did not change;
- before/after Git commits and exact changed files;
- hashes of the four preserved v1 documents;
- all six v1 and all six v2 roots absent;
- old and new authorization records absent;
- no protocol command executed;
- registered run and paper gate both CLOSED.

- [ ] **Step 2: Obtain an independent implementation review**

The reviewer must inspect the diff from v1 failure commit `12c00ff` through
Task 2 HEAD and independently verify:

- the test really traverses serialized protocol loading and the real replay
  entry point;
- the regression failed for the v1 defect before implementation;
- global missing/extra/order and local missing/extra/order/identity drift fail
  closed before root allocation;
- registered all-source validation remains at least as strict;
- every current production identity/path/root is v2;
- every v1 execution root is retired;
- no scientific constant, gate, schema field, fixture, data, or comparator
  changed;
- v1 artifacts and DRA records were not rewritten;
- all required test and CLI evidence passes.

The review file must state separate Critical, Important, and Minor counts.
Do not proceed unless all three counts are zero.

- [ ] **Step 3: Commit implementation closure**

```bash
git add \
  docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-v2-recovery-implementation.md \
  docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-v2-recovery-implementation-review.md
git commit -m "docs(diagnostics): approve two-range v2 recovery"
```

- [ ] **Step 4: Append and commit DRA**

Append implementation closure, v1 retirement, exact source commit, test
counts, review result, v2 paths, and remaining smoke/registered gates to the
four DRA files.  Do not rewrite prior Task 7/8 entries.

```bash
git commit -m "docs(cbf2026): record two-range v2 recovery"
```

### Task 4: Freeze and independently preflight protocol v2

**Files:**
- Create:
  `docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json`
- Create:
  `docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.md`
- Create:
  `docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2-review.md`
- Append the four DRA files from Global Constraints.

**Interfaces:**
- Consumes: committed C0/I0/M0 Task 3 implementation parent.
- Produces: one immutable v2 protocol and a smoke-only preflight approval.

- [ ] **Step 1: Prove generation preconditions**

Require:

- source worktree tracked/staged diff empty and status otherwise only
  `?? build-diagnostic/`;
- all six v1 roots and all six v2 roots absent and not symlinks;
- both v1 and v2 authorization paths absent;
- both v2 protocol output paths absent;
- at least `8_000_000_000` free bytes;
- all 11 source and seven comparator identities live and exact;
- implementation HEAD contains no v2 protocol output path;
- implementation review is committed with C0/I0/M0.

- [ ] **Step 2: Perform two non-publishing deterministic dry builds**

In two separate `mktemp -d` directories, call the registrar with
`--repository-root /private/tmp/cbf2026-diagnostic` and temporary Markdown
and JSON output paths.  Compare the two JSON files byte-for-byte and the two
Markdown files byte-for-byte.  Record sizes and SHA-256 values, then remove
only those two validated temporary directories.  This is not a protocol
command-array execution and creates no evidence root.

- [ ] **Step 3: Generate the production pair exactly once**

Run exactly once:

```bash
conda run -n cbf_env python \
  scripts/diagnostics/register_two_range_reacquisition.py \
  --repository-root /private/tmp/cbf2026-diagnostic \
  --output-markdown docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.md \
  --output-json docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json
```

Require production bytes equal both dry builds.  Do not regenerate, overwrite,
or edit either generated file.

- [ ] **Step 4: Commit the immutable protocol**

```bash
git add \
  docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json \
  docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.md
git commit -m "docs(diagnostics): freeze two-range protocol v2"
```

- [ ] **Step 5: Independently preflight the real runtime path**

The independent reviewer must:

- rebuild canonical JSON/Markdown in memory and compare exact bytes;
- re-stat/re-hash all 11 sources and seven comparators;
- validate protocol v2 schema/ID, unchanged raw/analysis schemas, 18 smoke
  cases, 140000 registered keys, gates, constants, disk limits, and all six
  command arrays;
- invoke the same pre-root runtime validation used by both `smoke_a` and
  `smoke_b` against the committed serialized protocol, with root allocation
  intercepted only after validation succeeds;
- prove all 12 retired/current roots remain absent;
- prove registered commands and v2 authorization remain unauthorized;
- report separate C0/I0/M0 counts and approve only the four deterministic
  v2 smoke commands.

Do not proceed unless C0/I0/M0.

- [ ] **Step 6: Commit preflight and append DRA**

```bash
git add \
  docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2-review.md
git commit -m "docs(diagnostics): approve two-range v2 preflight"
```

Append exact protocol/preflight commits, hashes, C0/I0/M0, and the still-closed
registered gate to the four DRA files, then:

```bash
git commit -m "docs(cbf2026): record two-range v2 preflight"
```

### Task 5: Execute and independently close only the v2 deterministic smokes

**Files:**
- Create:
  `docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-smoke-v2.md`
- Create:
  `docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-smoke-v2-review.md`
- Append the four DRA files from Global Constraints.

**Interfaces:**
- Consumes: exact command arrays from the committed and independently approved
  protocol v2.
- Produces: at most two 18-row raw smoke bundles, two compact smoke-analysis
  bundles, and a reviewed PASS or terminal-failure record.

- [ ] **Step 1: Execute smoke A exactly once**

Recheck clean committed sources, protocol/preflight hashes, all 12 root
absences, both authorization absences, and disk floor.  Load
`commands.smoke_a` directly from the committed JSON and execute it as argv
without shell reconstruction.  Record command-array SHA-256, start/end free
space, return code, manifest status, row count, and process hashes.

If it fails, stop immediately, preserve any evidence, write the terminal v2
report/review, update DRA, and execute no later command.

- [ ] **Step 2: Execute smoke analyzer A exactly once**

Require smoke A completed with exactly 18 ordered cases.  Recheck the analyzer
A root absence and disk floor, then execute exact
`commands.smoke_analyzer_a`.  Require terminal `completed`,
`observed_rows=18`, zero scientific gates, all 14 integrity gates passing,
and `decision="smoke_pass"`.

If it fails, stop exactly as in Step 1.

- [ ] **Step 3: Execute smoke B exactly once**

Require smoke A and analyzer A terminal success, smoke B root absence, clean
committed bindings, and disk floor.  Execute exact `commands.smoke_b`.
Require exactly the same ordered 18 cases and terminal completion.

If it fails, stop exactly as in Step 1.

- [ ] **Step 4: Execute smoke analyzer B exactly once**

Require smoke B terminal success and analyzer B root absence.  Execute exact
`commands.smoke_analyzer_b`.  Require terminal `completed`,
`observed_rows=18`, all 14 integrity gates passing, and
`decision="smoke_pass"`.

If it fails, stop exactly as in Step 1.

- [ ] **Step 5: Compare deterministic evidence**

Require:

- raw A/B compressed SHA-256 equal;
- raw A/B decompressed SHA-256 equal;
- analyzer A/B JSON semantic payload hashes equal;
- analyzer A/B Markdown semantic content equal after independently permitted
  path/provenance differences are excluded by the frozen semantic hash;
- ordered case IDs equal `SMOKE_CASE_IDS`;
- both manifests and analyzer manifests are terminal and self-consistent;
- no v1 or registered root was created;
- v2 authorization remains absent.

- [ ] **Step 6: Write and independently review the smoke closure**

The author report records all prechecks, invocation counters, command hashes,
artifact identities, decisions, deterministic comparisons, disk observations,
and the registered/paper CLOSED boundary.  The independent review recomputes
the evidence from the four roots and reports C0/I0/M0 separately.

For success, the review verdict is
`APPROVED: V2 DETERMINISTIC SMOKE PASS; REGISTERED GATE CLOSED`.
For any failure, the verdict is
`REPORT APPROVED; V2 SMOKE GATE FAIL`, and v2 is permanently retired.

- [ ] **Step 7: Commit smoke closure and append DRA**

```bash
git add \
  docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-smoke-v2.md \
  docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-smoke-v2-review.md
git commit -m "docs(diagnostics): record two-range v2 smoke"
```

Append exact smoke commit, root hashes, PASS/failure verdict, limitations, and
remaining registered authorization gate to the four DRA files, then:

```bash
git commit -m "docs(cbf2026): record two-range v2 smoke"
```

- [ ] **Step 8: Stop before the registered run**

Do not create the v2 authorization JSON.  Do not execute
`commands.registered_replay` or `commands.registered_analyzer`.
Present the reviewed smoke result, scientific limitations, expected
140000-row cost, and exact no-retry risk to the user for a new explicit
registered-run decision.
