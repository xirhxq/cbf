# CBF2026 two-range reacquisition implementation

## Final outcome and scope

The test-first implementation is complete at the final pre-commit state
reviewed in
[`2026-07-30-cbf2026-two-range-reacquisition-implementation-review.md`](reviews/2026-07-30-cbf2026-two-range-reacquisition-implementation-review.md).
The final independent implementation verdict is:

- Critical: `0`;
- Important: `0`; and
- Minor: `0`.

The implementation adds a truth-free two-circle branch selector,
an independently propagated tagged private state used only to identify the
discrete branch,
an approved historical mechanism fixture,
strict raw and compact evidence paths,
and a deterministic protocol registrar.
All final changes were reviewed before the implementation-closure commit.

Worktree: `/private/tmp/cbf2026-diagnostic`

Branch: `codex/cbf2026-diagnostic`

Reviewed base `HEAD`:
`c69b1db10a8686e88500f29bb1d9ad715461dca2`
(`fix(diagnostics): harden protocol registration`).

The pre-existing untracked `build-diagnostic/` directory was preserved.
No production protocol,
authorization record,
smoke root,
registered root,
full-grid result,
Stage 2 trajectory,
paper edit,
or DRA update was created before this source-repository closure.
The DRA milestone is recorded separately after the closure commit exists.

## Planning and implementation ancestry

The planning-closure commit is
`ff6b1eb72aec2be8e50816c9efaea0ca05e841d6`
(`docs(diagnostics): review two-range implementation plan`),
whose parent is
`58158395747ee86e50954717871f3bf143502645`.
The committed plan is
`docs/superpowers/plans/2026-07-30-cbf2026-two-range-reacquisition-implementation.md`,
exactly `164746` bytes with SHA-256
`644bbe01b1cf759f700b68d9ae5113cd9ebc62b33b3454769e6c057f409cc8bd`.
The approved design commit is
`20a61aad96af35ee7e16434fab0a5edaaea38ef0`.

The committed implementation ancestry before the final pre-commit review
repairs is:

| Commit | Subject |
| --- | --- |
| `7e8ddcb0d4004d1ba6c7a13d6b8d6d71ca1c8a2e` | `feat(diagnostics): add tagged private-state lifecycle` |
| `5d5beab9691e73295143e1fe63006adf5a8292df` | `test(diagnostics): strengthen frame-zero lifecycle guard` |
| `f434a18d060b9a34dd40b4fcad5a3f19871d62ad` | `feat(diagnostics): add two-range branch selector` |
| `163a6de21f544eb70edd28b3a71a163211dde11d` | `fix(diagnostics): classify two-range cardinality as input invalid` |
| `566f98ab892a2bbe0154cf0eda11410b86c3bcd8` | `feat(diagnostics): add two-range replay producer` |
| `ada9f87852d7bbcf85b94d85e75299f4baa6051d` | `fix(diagnostics): close replay evidence boundaries` |
| `2b92352f97104ddf16dd87d5a45c6780850c1536` | `fix(diagnostics): bind registered replay evidence` |
| `fbd23cb59d114001d65dc49440a75c37eb3a53b6` | `fix(diagnostics): enforce replay semantic closure` |
| `e0d4df254f2f4a20823c59feab7b05c3aa2762c6` | `fix(diagnostics): validate solver fim summaries` |
| `809676ad74a828210d4ba77a366b2c1a25de7bc3` | `feat(diagnostics): add two-range evidence analyzer` |
| `dc5d13dce5d6aa1154066cd2eee907a7405e73a5` | `fix(diagnostics): close analyzer evidence gaps` |
| `8ab6ed1f1dd67e2a28c6f037ec8b060c4806eb63` | `fix(diagnostics): harden analyzer terminal evidence` |
| `613f9c4390ed76e941f512d23c6ddfb76a519ad9` | `fix(diagnostics): bind analyzer gate evidence` |
| `561d935bd32e9220f747c957ee0bf8c870733922` | `fix(diagnostics): bind compact analysis to raw evidence` |
| `12a90e8e6d7b1b10291bbc79998a28ad946b7adb` | `fix(diagnostics): enforce raw projection origin` |
| `b0ba1956a570c84547a0b5ccfbf93cdd8a2e492f` | `fix(diagnostics): bind raw origin to aggregation` |
| `5365c9086c27e8bca3592fbb2612902ed44aef87` | `fix(diagnostics): close analyzer unwrap lifecycle` |
| `0062253171340957f9cc62d50f9601fc49ed2565` | `feat(diagnostics): add two-range protocol registrar` |
| `c69b1db10a8686e88500f29bb1d9ad715461dca2` | `fix(diagnostics): harden protocol registration` |

The final review repairs after `c69b1db` were kept uncommitted throughout
independent review and are included in the implementation-closure commit.
They affect the five implementation modules,
their five direct test modules,
and the fixture manifest.
No historical commit was rewritten or removed.

## Final interfaces and safeguards

### Two-range estimator

`scripts/diagnostics/two_range_reacquisition.py` provides the method
`two_range_private_branch_reacquisition`,
strict private-state canonicalization,
reset and one-frame propagation,
independent public/private prior advance,
lifecycle finalization,
two-circle branch validation,
and `solve_two_range_reacquisition`.

The final implementation validates `ranging_sigma` at the public solver
boundary.
It accepts only a non-Boolean real number that converts to a finite,
strictly positive float.
Boolean values,
strings,
lists,
zero-dimensional arrays,
zero,
negative values,
NaN,
and either infinity fail closed before WNLS.
The normalized float is the exact value supplied to both finite-budget WNLS
branches.

`scripts/diagnostics/predictive_wnls.py` retains the strict default
`allow_two_reference_reacquisition=False`.
Only the new method opts in,
so the previous three-reference reacquisition rule remains unchanged for all
default callers.

### Analyzer

`scripts/diagnostics/analyze_two_range_reacquisition.py` independently
reconstructs the registered and smoke compact results from retained parsed
raw inputs.
The earlier process-local secret,
mint authority,
capability,
registry,
and wrapper designs were removed.
The public analyzer is the actual implementation function:
it has no lower `__wrapped__` callable,
closure-injected authority,
or hidden cleanup boundary.

Immediately before publication,
validation reruns the registered or smoke aggregation from the retained raw
rows,
truth data,
protocol,
and branch representatives.
It independently recomputes the source projection,
compares every semantic result field and exact row budget,
and requires the pinned protocol and raw identity commitments.
A candidate-derived projection alone cannot authorize a compact result.
The raw recomputation context remains local and is not serialized into the
analysis JSON,
manifest,
or publication representative.

The analyzer revalidates the pinned source identities before completed
publication.
All faults after output-root creation are covered by the public function's
own transaction `finally`;
pre-root faults create no output transaction or provenance registry.

### Registrar

`scripts/diagnostics/register_two_range_reacquisition.py` constructs the
exact protocol and authorization records,
validates all source,
comparator,
root,
command,
schema,
and parameter contracts,
and publishes the paired registration files transactionally.

The final registrar opens both outputs read/write,
keeps the descriptors pinned,
and,
after the last external `final_probe`,
rereads every byte from each descriptor.
It verifies stable full descriptor metadata,
exact size,
and SHA-256 against the intended payload before reporting success.
An in-place,
same-inode,
same-length rewrite therefore fails closed and enters the existing
ownership-aware rollback path.

### Replay producer

`scripts/diagnostics/replay_two_range_reacquisition.py` provides the strict
raw schema,
exact deterministic smoke matrix,
registered key iterator,
producer,
transactional manifests,
and direct CLI.

Registered preflight now validates each raw smoke manifest with the complete
raw schema.
It binds the exact root-derived process path,
the complete process descriptor identity,
compressed and decompressed hashes,
protocol identity,
all declared source identities,
and exact row counts.
Every raw smoke record is revalidated again after analyzer-smoke inspection.

The terminal producer retains the exact parent and output-root descriptors,
checks their name binding around completed publication,
and retracts an owned completed manifest if a later terminal check fails.
Retraction uses a forensic quarantine and a zero-unlink tombstone path.
No-replace movement is atomic:
Darwin uses `renameatx_np` with `RENAME_EXCL`,
Linux uses `renameat2` with `RENAME_NOREPLACE`,
and an unsupported platform,
missing symbol,
or syscall failure fails closed without a clobbering fallback.
An existing foreign quarantine destination is preserved.
File,
root-directory,
and parent-directory durability checks are independent;
a failed `fsync` is retained as an explicit indeterminate-durability error
and cannot silently continue to completed publication.

### Fixture extractor and direct execution

`scripts/diagnostics/extract_two_range_reacquisition_fixture.py` binds the
exact V4 `output_root`.
It retains the exact V4 root descriptor and opens the manifest and compressed
process relative to that descriptor with `dir_fd`.
The same root and child identities are checked across manifest reading,
process hashing and decompression,
fixture construction,
publication,
and return.
A relocated hash-matching root,
a persistent root-name swap,
and an ABA swap that temporarily substitutes path-selected child bytes all
fail closed.

The extractor,
replay,
analyzer,
and registrar direct CLIs resolve and seed the implementation-root
`scripts` namespace before importing diagnostics.
The hostile-shadow regressions use a real temporary regular
`scripts` package whose import raises,
then execute each implementation script by absolute path from that unrelated
working directory.

## Fixture and immutable identities

The approved mechanism remains
`(seed=20260727, frame_index=180, robot_id=12)`.

| Artifact | Bytes | SHA-256 |
| --- | ---: | --- |
| `scripts/diagnostics/extract_two_range_reacquisition_fixture.py` | `26185` | `4c35e8c0ce8858ff247f07639b9b190f3196d31d80721b12b90bc507c24b8775` |
| `tests/fixtures/cbf2026_two_range_reacquisition/mechanism_20260727_180_12.json` | `3432` | `9febff2393017b7b0fbd1a02dd76a13d1d70639f2b176df236931fe29a8601c7` |
| `tests/fixtures/cbf2026_two_range_reacquisition/manifest.json` | `863` | `cb704d10d439dac55170ce14a9feef27c92cd60ec2a95ccb4c47a2cec73a722d` |

Only `extractor_identity.sha256` changed in the fixture manifest.
The mechanism filename,
bytes,
size,
hash,
source hashes,
approved design commit,
and content are unchanged.

The immutable source identities recomputed exactly:

| Source | SHA-256 |
| --- | --- |
| `scripts/diagnostics/replay_localization_calibration.py` | `0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8` |
| V4 manifest | `123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223` |
| V4 compressed process | `9710d1a5792e682bb272d0bb3305162180d1e0b9a0636647d6e071d1782a699b` |
| V4 decompressed process | `7047989804fd03bd4cadffac8212349b7183857623ac201d14ef2fc3e30ee2e1` |

## Final verification

All test commands ran from `/private/tmp/cbf2026-diagnostic` with
`conda run -n cbf_env`.

| Gate | Result | unittest | Wall | User | Sys |
| --- | ---: | ---: | ---: | ---: | ---: |
| Focused implementation suite | `327 / 327`, `OK` | `72.382 s` | `73.35 s` | `68.77 s` | `13.54 s` |
| Preserved predictive-WNLS suite | `230 / 230`, `OK` | `18.209 s` | `19.09 s` | `33.90 s` | `8.73 s` |
| Full Python discovery | `864 / 864`, `OK` | `113.424 s` | `114.37 s` | `124.33 s` | `29.53 s` |
| Six implementation modules, `py_compile` | `6 / 6`, exit `0` | — | — | — | — |
| Repository-root direct CLI help | `4 / 4`, exit `0` | — | — | — | — |
| Hostile-shadow direct CLI help | `4 / 4`, exit `0` | — | — | — | — |

The focused suite comprises:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_two_range_reacquisition \
  tests.test_extract_two_range_reacquisition_fixture \
  tests.test_replay_two_range_reacquisition \
  tests.test_analyze_two_range_reacquisition \
  tests.test_register_two_range_reacquisition -v
```

The preserved suite comprises:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_predictive_wnls \
  tests.test_extract_predictive_wnls_stage0 \
  tests.test_replay_predictive_wnls_recovery \
  tests.test_analyze_predictive_wnls_recovery \
  tests.test_register_predictive_wnls_stage1 -v
```

Full discovery used:

```bash
conda run -n cbf_env python -m unittest discover \
  -s tests -p 'test*.py' -v
```

`git diff --check` passed.
Markdown hygiene checks passed.
The final immutable hashes matched the table above.
Both existence and symbolic-link checks confirmed these six exact roots
absent:

```text
/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-b
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-b
/private/tmp/cbf2026-two-range-reacquisition-development/v1
/private/tmp/cbf2026-two-range-reacquisition-analysis/v1
```

The exact RED/GREEN and independent-review history is recorded in the linked
implementation review.

## Private-prior isolation and Stage 1 preservation

The private prior does not enter WNLS starts.
The selector constructs exactly the negative and positive raw circle
intersections from the two reference positions and ranges,
then calls `solve_finite_budget_wnls` with each raw intersection.
Only after both solver results are complete and distinct is the private prior
supplied to normalized innovation to score discrete branch identity.

The private prior does not enter the FIM or continuous update.
Each solver receives only reference positions,
reference covariances,
ranges,
one circle start,
and the validated ranging sigma.
The accepted and rejected attempt records fix
`prior_used_in_fim=False` and
`prior_used_for_continuous_update=False`.

The private prior does not enter public estimator output.
An accepted public candidate is copied from the selected solver result.
Nonaccepted public propagation uses only the preceding public output.
The tagged private state advances separately by one held-command transition.
Raw diagnostic evidence intentionally discloses tagged private state for
audit,
but does not use it as a public estimator state or publication
representative.

Default Stage 1 behavior remains unchanged.
The complete preserved suite passes `230 / 230`.
The V4 declaration,
schema,
commands,
source and comparator hashes,
evidence roots,
and lifecycle artifacts were not modified.

## Claim and threat boundaries

This implementation closure supports only code,
unit/integration verification,
immutable-input identity,
transactional-publication,
and independent implementation-review claims.
It does not claim that any registered production command,
smoke protocol,
full 140000-row grid,
compact production analysis,
or Stage 2 run was executed.

The analyzer's recomputation boundary rejects candidate result,
projection,
context,
and reuse substitutions through the real public flow.
It does not claim protection against arbitrary same-process mutation of
module globals,
function bytecode,
tracing or local-frame access,
or direct monkeypatching of the validator itself.
Such isolation would require a separate process or stronger execution
boundary.

The no-replace publication contract protects owned evidence from name-swap
and foreign-destination clobbering during the reviewed transaction.
It does not prevent a privileged external actor from modifying files after
the function has returned.

The scientific limitations remain:

- one preserved truth trajectory;
- estimator outside the controller;
- diagonal marginal-covariance FIM approximation;
- unmodeled shared-ancestor cross-covariances;
- conditional modeled three-standard-deviation radius;
- `243 / 7000` applied source component-bound violations; and
- closed paper-edit and bounded-input Stage 2 gates.

Consequently,
the implementation does not establish cross-trajectory reliability,
closed-loop estimator robustness,
unconditional CBF safety,
or a deterministic localization-error guarantee.
A later production claim still requires a separately authorized protocol,
clean preflight,
deterministic smoke review,
registered execution,
raw-evidence review,
and the later paper and Stage 2 gates.
