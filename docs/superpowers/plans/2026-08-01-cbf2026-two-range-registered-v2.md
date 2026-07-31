# CBF2026 Two-Range Registered v2 Execution Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development to execute this plan task-by-task.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Execute the single authorized 140000-row two-range registered v2
replay and, only after a completed replay, its single registered analyzer;
preserve exact provenance, obtain an independent evidence verdict, and update
the DRA without opening the paper or Stage-2 gates.

**Architecture:** The already frozen v2 protocol is the sole executable
contract.  A new committed authorization record binds the researcher's exact
2026-08-01 message, the frozen protocol/preflight/smoke commits, and every
retained smoke artifact.  An external exact-once operator reads the two argv
arrays directly from the committed protocol JSON, enforces the disk and root
lifecycle before launch, and never reconstructs commands by hand.  Raw and
compact registered evidence remain immutable after terminal publication and
are reviewed independently before any DRA conclusion is recorded.

**Tech Stack:** Python 3, `unittest`, canonical JSON, gzip JSONL, conda
environment `cbf_env`, Git, Markdown, and append-only DRA evidence records.

## Global Constraints

- Source work only in `/private/tmp/cbf2026-diagnostic` on
  `codex/cbf2026-diagnostic`; DRA work only in
  `/private/tmp/dra-cbf2026-diagnostic` on `main`, as the user explicitly
  requested for the shared multi-paper archive.
- The source worktree was relinked to its preserved Git administration after
  `/private/tmp` cleanup and all absent tracked files were restored from HEAD.
  At plan start, source HEAD is
  `01e1b93384587a6443384fe3a35f4cf22e5ffd94`; the only allowed pre-existing
  porcelain entry is untracked `build-diagnostic/`.
- DRA starts at main HEAD
  `c38471176563255bdef56ab92fcc680dc347d044`, ahead of its configured remote;
  do not push either repository.
- Use `conda run -n cbf_env` for every Python command.
- Do not change replay/analyzer implementation, protocol bytes, schemas,
  estimator constants, measurement noise, seeds, frame count, UAV count,
  thresholds, scientific gates, integrity gates, comparator identities,
  smoke roots, or no-retry semantics.
- The exact protocol path is
  `docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json`
  and its SHA-256 is
  `9a0985e00e1b6bd801f7b46c3811c2e135c29f86f75b5045e7aff1ad71206bb2`.
- The exact authorization path is
  `docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-registered-v2-authorization.json`.
- The researcher's exact authorization text is the two UTF-8 characters
  `继续`, without added whitespace or newline, dated `2026-08-01`.  This
  authorizes exactly one `registered_replay` and, conditionally, exactly one
  `registered_analyzer`; it does not authorize a retry.
- The registered replay grid is exactly 20 frozen seeds times 500 frames times
  14 UAVs, producing the exact ordered 140000 keys with `ranging_sigma_m=0.5`.
- The exact registered roots are:

```text
/private/tmp/cbf2026-two-range-reacquisition-development/v2
/private/tmp/cbf2026-two-range-reacquisition-analysis/v2
```

- Each registered root must be absent under an existence-or-symlink test
  before its one invocation.  Never delete, rename, reuse, copy onto, hard-link
  onto, or symlink either root after an invocation begins.
- A protocol invocation starts only when its exact argv is handed to the
  process.  Once started, any exit status, signal, exception, or terminal
  failed manifest consumes that invocation permanently.  Retain the root and
  stop; do not retry or run any later protocol command.
- The analyzer may start only if replay returns zero and the independently
  checked raw terminal manifest is `completed`, names `registered_replay`,
  observes 140000 of 140000 rows, and binds the committed authorization.
- Require at least `8_000_000_000` free bytes at each launch, at least
  `6_000_000_000` live free bytes, raw allocated bytes at most
  `2_000_000_000`, and compact allocated bytes at most `10_000_000`.
- Safe disk cleanup may remove only narrowly identified rebuildable caches.
  Never clean protocol inputs, comparators, retained smoke evidence, Git data,
  source/DRA worktrees, Codex sessions, or paper assets.  Record exact removed
  paths and before/after free bytes.
- Use the exact ordered argv arrays loaded from the committed protocol JSON.
  Never type, splice, normalize, or substitute a registered argv by hand.
- Paper edits, manuscript claims, Stage 2, journal decisions, submissions,
  and any additional experiment remain closed regardless of scientific
  outcome.  A scientific failure is still valid registered evidence if the
  integrity contract passes.
- Commits remain authorized for this workflow.  Use
  `type(scope): description`, add no `Co-Authored-By`, and do not push.

## Frozen Provenance

- Protocol commit:
  `5241d6a4d3ab8144ec065dc2f92c4c454a1d3760`.
- Implementation parent:
  `526e1418138426e54bff5a41e3847b6d2a9f8203`.
- Preflight/execution-review commit:
  `84aa6851bfa61468f3e0a6555d081fe124fb2395`.
- First commit containing the smoke report:
  `01e1b93384587a6443384fe3a35f4cf22e5ffd94`.
- Smoke A/B compressed SHA-256:
  `8073457c8e7a21a5ec7c6d85baaec4c77a549ea66422e474ad927f323160677c`.
- Smoke A/B decompressed SHA-256:
  `cdbde96b17219f69cf0c00f403718048d8d0d04b4943ce53d3580978f240d435`.
- Smoke analyzer A JSON/Markdown SHA-256:
  `26753574fa3e9217f18180a2bf0c51a34d30b763134913e483a2dbbe9e378c0d`
  and
  `3f49d592cfd127ef751d638838975b5f2a55655a88944290b5eaf2a4a485b526`.
- Smoke analyzer B JSON/Markdown SHA-256:
  `0679640eb15f77a66ad3c8b2ecbcf9f8d6e6290b1d8821ce4869d789d73f2c3c`
  and
  `80d68acd68cf84d7f90bb072b0496743fad51856aee45954cc7d5063440fb13e`.
- Common smoke semantic payload SHA-256:
  `55c636e41c121e60eaa2341d0524ff5c08838374969343eec1e9f916a1758247`.

## Files

- Create this implementation plan.
- Create the exact authorization JSON path listed above.
- Create
  `docs/diagnostics/2026-08-01-cbf2026-two-range-reacquisition-registered-v2.md`.
- Create
  `docs/diagnostics/reviews/2026-08-01-cbf2026-two-range-reacquisition-registered-v2-review.md`.
- Update DRA in place/append-only as appropriate:
  `papers/cbf2026/status.md`, `papers/cbf2026/timeline.md`,
  `papers/cbf2026/sources.md`, `papers/cbf2026/open-questions.md`, and new
  `meta-log/2026-08-01-cbf2026-two-range-registered-v2.md`.
- Do not modify `main.tex`, paper figures, protocol/replay/analyzer code, tests,
  old reports, old reviews, old manifests, or smoke/registered bundles.

### Task 1: Freeze the execution plan and restore launch eligibility

**Files:**
- Create: this plan only in the source repository.

- [ ] Record source and DRA HEAD, branch, upstream, porcelain, common Git
  directory, and worktree paths.
- [ ] Rehash the committed protocol and verify all 11 source identities and
  retained smoke artifacts before cleanup.
- [ ] Confirm both registered roots and the authorization path are absent,
  including broken symlinks.
- [ ] Commit only this plan as
  `docs(diagnostics): plan two-range v2 registered run`.
- [ ] Identify narrowly scoped rebuildable caches by allocated size.  Preserve
  a before-inventory, clean only approved safe cache classes, and verify actual
  free bytes are at least `8_000_000_000`.  If that cannot be reached safely,
  stop before authorization and report the disk blocker.
- [ ] Run focused non-publishing tests for protocol validation and public
  registered entry guards.  Tests may use isolated temporary roots only and
  must not create either registered root.
- [ ] Obtain an independent task review confirming plan completeness, frozen
  identities, root absence, and disk eligibility.

### Task 2: Create and commit the exact authorization record

**Files:**
- Create:
  `docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-registered-v2-authorization.json`.

- [ ] Derive every hash from the retained manifest/artifact bytes, not from
  prose alone.  Derive the UTF-8 SHA-256 of exact text `继续` without newline.
- [ ] Write exactly these 21 ordered fields with no extras:
  `schema_id`, `protocol_id`, `protocol_sha256`, `protocol_commit`,
  `preflight_commit`, `smoke_commit`, `smoke_a_compressed_sha256`,
  `smoke_a_decompressed_sha256`, `smoke_b_compressed_sha256`,
  `smoke_b_decompressed_sha256`, `smoke_analyzer_a_json_sha256`,
  `smoke_analyzer_a_markdown_sha256`, `smoke_analyzer_b_json_sha256`,
  `smoke_analyzer_b_markdown_sha256`, `smoke_semantic_payload_sha256`,
  `user_authorization_date`, `user_authorization_text`,
  `user_authorization_text_sha256`, `registered_replay_root`,
  `registered_analyzer_root`, and `registered_retry_allowed`.
- [ ] Set `schema_id` to
  `cbf2026-two-range-reacquisition-registration-v2`, `protocol_id` to
  `cbf2026-two-range-reacquisition-v2`, exact roots as frozen above, and
  `registered_retry_allowed` to JSON `false`.
- [ ] Run schema, byte-hash, retained-smoke, Git-blob, and dirty-path checks
  through non-publishing validation paths.  Neither registered root may exist
  after validation.
- [ ] Commit only the authorization JSON as
  `docs(diagnostics): authorize two-range v2 registered run`.
- [ ] Independently review the committed authorization bytes at the new HEAD,
  verify the 21-field order and all live/historical bindings, and record the
  authorization JSON byte SHA-256, authorization commit SHA, and that commit's
  parent SHA before execution.
- [ ] Update DRA main with an authorization-only checkpoint that says the job
  is authorized but unrun; commit it without pushing.

### Task 3: Execute the exact registered replay once

**Files:**
- Create external immutable raw evidence under the exact replay root only.
- Create an ignored operator ledger/review package under `.superpowers/sdd/`.

- [ ] An exact-once operator reads `commands.registered_replay` directly from
  the committed protocol JSON, proves it equals the frozen array, and records
  its hash without manually reconstructing it.
- [ ] Immediately before process start, recheck authorization commit/bytes,
  protocol/source cleanliness and hashes, input/comparator/smoke identities,
  replay root absence, analyzer root absence, and free bytes at least 8 GB.
- [ ] Atomically mark the operator ledger `launching`, then pass the unchanged
  argv array once to the subprocess from the source worktree.
- [ ] While it runs, monitor process state, free bytes, root allocated bytes,
  and user-visible progress without interrupting normal in-process lifecycle.
  Never issue a second process start.
- [ ] On any nonzero return, signal, missing/failed terminal manifest, disk
  violation, or identity mismatch: retain all evidence, mark the ledger
  terminal, update DRA with failure, and stop this plan without analyzer.
- [ ] On return zero, independently verify a completed terminal manifest,
  exact invocation/protocol/authorization identity, 140000 expected and
  observed rows, ordered-key completeness, raw compressed/decompressed hashes,
  allocation cap, and free-space floor.  Do not edit the bundle.
- [ ] Update DRA main with a replay-terminal checkpoint that says exactly
  whether the analyzer start condition is satisfied; commit without pushing.

### Task 4: Execute the exact registered analyzer once

**Files:**
- Create external immutable compact evidence under the exact analyzer root
  only.

- [ ] Proceed only if Task 3 has an independently verified completed replay.
- [ ] Read `commands.registered_analyzer` directly from the committed protocol
  JSON and record its exact array/hash; never reconstruct it manually.
- [ ] Immediately before process start, recheck the completed raw manifest and
  bundle, authorization/protocol/source identities, analyzer root absence,
  at least 8 GB free, and the raw root's immutability.
- [ ] Atomically mark the operator ledger `launching`, then pass the unchanged
  analyzer argv once to the subprocess.
- [ ] On any failure, retain the terminal analyzer evidence, mark the ledger,
  update DRA, and stop without retry.
- [ ] On success, independently verify the completed analyzer manifest,
  invocation/protocol/authorization/raw identities, 140000 rows, all output
  hashes, compact allocation cap, all 14 integrity gates, and all nine
  scientific gate values.  Scientific pass is not presumed.

### Task 5: Report and independently review registered evidence

**Files:**
- Create the author registered report and independent review paths listed in
  `Files`.

- [ ] The author report records: exact authorization text/date/hash and commit;
  authorization JSON byte hash and authorization commit parent;
  execution HEAD/branch/upstream/push state/porcelain/worktree; protocol,
  implementation-parent, preflight, and smoke commits; exact argv hashes;
  exact-once ledger and zero retries; timestamps/return codes; disk before,
  minimum, after, and allocated maxima; raw/compact manifests, identities,
  hashes, row counts, gate values, and limitations.
- [ ] Clearly separate execution HEAD, authorization commit, author-evidence
  commit, independent-review commit, and prior smoke-evidence commit.
- [ ] State that no estimator comparison, control-loop injection, paper edit,
  Stage 2, or resubmission is part of this evidence.
- [ ] A fresh independent reviewer reopens raw gzip, compact JSON/Markdown,
  both manifests, authorization, protocol, input/comparator evidence, and the
  author report.  It recomputes hashes, row/key integrity, allocation, and
  gates instead of trusting author prose.
- [ ] The review records the reviewer code/commit identity, complete reviewed
  input path/hash inventory, finding counts, verdict, review byte hash, and
  review commit; it states explicitly that the subject is registered evidence,
  not an extrapolation of smoke evidence.
- [ ] The review assigns separate integrity and scientific verdicts and may
  approve integrity while rejecting the scientific hypothesis.  It explicitly
  leaves paper and Stage-2 gates closed.
- [ ] Commit the author report and independent review with intentional staging
  and no unrelated files.  Run a final source repository status and evidence
  hash check.

### Task 6: Archive the terminal result and stop

**Files:**
- Update the five DRA paths listed in `Files`.

- [ ] Update the sole current DRA status in place; append dated timeline and
  sources evidence; update the current open question; create the dated
  meta-log.  Do not modify submissions.
- [ ] Record source repository versus paper worktree identity, execution and
  evidence commits, branch/upstream/push state, pre/post porcelain, roots,
  manifests, artifact/report/review hashes, row counts, disk observations,
  retries, integrity verdict, scientific verdict, and claim boundary.
- [ ] Commit DRA main using the normal convention without pushing.
- [ ] Run plan-task review, an adversarial final whole review, and repository
  cleanliness checks.  Preserve `build-diagnostic/` untouched.
- [ ] Stop after reporting the registered result.  Do not start Stage 2, change
  the theory/manuscript, tune thresholds, or run follow-up experiments without
  a new explicit user decision.

## Terminal Outcomes

- **Success:** both registered commands ran once, both terminal manifests are
  completed and independently validated, author/reviewer/DRA evidence is
  committed, and integrity/scientific verdicts are reported separately.
- **Scientific rejection:** lifecycle and integrity pass, one or more frozen
  scientific gates fail, and the negative result is preserved and reported
  without tuning or retry.
- **Protocol failure:** a registered invocation fails or violates lifecycle;
  its root and terminal evidence are retained, no retry occurs, later protocol
  commands do not run, and DRA records the terminal failure.
- **Prelaunch block:** disk, Git, authorization, identity, root, or source
  precondition fails before argv handoff; no registered invocation is consumed,
  no registered root is created, and the blocker is reported.
