# CBF2026 two-range reacquisition smoke terminal-failure report

Date: 2026-07-31

Status: TERMINAL FAILURE

Task 8 gate: FAIL

Paper gate: CLOSED

Independent review: PENDING

This is an author-side terminal-failure record.
It does not claim an independent C0/I0/M0 review,
does not report a scientific result,
and does not authorize a retry under protocol v1.

## Approved execution boundary

The approved protocol commit is
`98708018e0076eb5bd38f20b2cbeea6b971a9fd3`
(`docs(diagnostics): freeze two-range protocol`).
Its reviewed implementation parent is
`5a14eb66ffeb541540183041ce2966c5dfd6d949`.
The smoke-only preflight approval commit is
`e738b8b1ac5d0756f7eb207886d85c0c9625a00c`
(`docs(diagnostics): approve two-range preflight`).
The approved design ancestor is
`20a61aad96af35ee7e16434fab0a5edaaea38ef0`.

The preflight authorized only the two deterministic smoke producers and
their corresponding smoke analyzers,
subject to the exact order and stop conditions in Task 8.
It did not authorize either registered command,
Stage 2,
or a paper edit.

| Artifact | Bytes | SHA-256 |
|---|---:|---|
| Protocol JSON | 69,464 | `8e01df56623bb2e759fb37108ddc68fbc41e301b8305ade7322e212ae6e3ea16` |
| Protocol Markdown | 4,137 | `c9e3930d22355747ee12e96303c1702aa0385de2881f6b430a72030b89d7e0ff` |
| Smoke-only preflight review | 14,948 | `6ac2da2d1fdcd173de04d1e9658560ac25439e79d03f1b5649d63f62da805cfa` |
| Approved design | 29,343 | `d28d6bfe297a6e37b0a878dc716bd5b91f26c5b11568c3f6fc42bd07760a266b` |
| Approved design review | 3,272 | `6d857be965de5b83bb05ce06381b79108fdc4abbccf6b96fe4ec456d7a7df570` |

Immediately before the smoke-A invocation,
the repository HEAD was exactly the preflight commit,
the working-tree status was exactly `?? build-diagnostic/`,
all bound source and comparator identities matched,
all six protocol roots were absent,
the authorization record and this report were absent,
and `/private/tmp` had exactly `26,388,066,304` free bytes.
The launch floor was `8,000,000,000` bytes.

The terminal-state check after the failure observed exactly
`26,381,086,720` free bytes.
It also confirmed that all six protocol roots remained absent,
the authorization record remained absent,
the tracked and staged diffs remained empty,
and the working-tree status remained exactly `?? build-diagnostic/`.

## Immutable source identities

All 11 protocol source records were re-statted and rehashed immediately
before execution.
The nine repository-owned records also matched their Git blobs at the
reviewed implementation parent.

| Member | Bytes | SHA-256 |
|---|---:|---|
| `implementation_plan` | 164,746 | `644bbe01b1cf759f700b68d9ae5113cd9ebc62b33b3454769e6c057f409cc8bd` |
| `two_range_reacquisition_source` | 14,633 | `d226c0184d89aadfe7b25a4a52d27379a396f868cfaab2c5a8a13ee7cdc04f8c` |
| `predictive_wnls_source` | 54,412 | `0269baa715c087661f2fa4c4760345250b380e2a14cce021fc3aca1b62f2defc` |
| `fixture_extractor_source` | 26,185 | `4c35e8c0ce8858ff247f07639b9b190f3196d31d80721b12b90bc507c24b8775` |
| `replay_source` | 160,792 | `d8d670934a3d0e21479dac4484a658e253478a8d3ccc709aa19ad3699f72db23` |
| `analyzer_source` | 175,462 | `0818bdb944d2270d98efdb130e5a96a7dbaedfc090a4a35693e4e803437a6305` |
| `registrar_source` | 100,660 | `87885a19288e879a44ea4ef4cfecc2c5576f381a8599047fb0eca360355c3bee` |
| `mechanism_fixture` | 3,432 | `9febff2393017b7b0fbd1a02dd76a13d1d70639f2b176df236931fe29a8601c7` |
| `mechanism_fixture_manifest` | 863 | `cb704d10d439dac55170ce14a9feef27c92cd60ec2a95ccb4c47a2cec73a722d` |
| `truth_data` | 16,237,150 | `3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527` |
| `input_manifest` | 1,226 | `6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb` |

The externally preserved truth and input paths were:

```text
/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json
/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json
```

## Immutable comparator identities

Both authoritative v4 roots and all seven comparator file records were
re-statted and rehashed before execution.
The compressed and decompressed v4 process hashes were recomputed in their
separate hash domains.

| Member | Bytes | SHA-256 |
|---|---:|---|
| `v4_replay_manifest` | 9,560 | `123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223` |
| `v4_compressed_process` | 206,565,170 | `9710d1a5792e682bb272d0bb3305162180d1e0b9a0636647d6e071d1782a699b` |
| `v4_decompressed_process` | 206,565,170 storage bytes | `7047989804fd03bd4cadffac8212349b7183857623ac201d14ef2fc3e30ee2e1` over decompressed bytes |
| `v4_analysis_manifest` | 4,827 | `e50352825665bb94c0e3b32c7922a081f093100daf0ed8e5c8ce9e9f32bd9238` |
| `v4_analysis_json` | 123,611 | `8a0a417f2d3b139fa248bef8157a3d4871fa04042aaad82ce5cd31f470a6ff1e` |
| `v4_analysis_markdown` | 39,055 | `986dcf72f72ab6dc3d424d0de6cdbf410cf271d174d16caef32945acc7d4e13b` |
| `legacy_baseline_process` | 68,754,629 | `c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003` |
| `legacy_baseline_protocol_json_sha256` | 17,245 | `09b236978e2e21bac226dc3d00c36a2ee5cdf5fea0e616f2c1d875b029d4e4e0` |

The exact comparator roots remained:

```text
/private/tmp/cbf2026-predictive-wnls-development/stage1-v4
/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v4
```

## Exact invocation record

The compact JSON encoding of the committed
`commands.smoke_a` array contains 19 tokens and has SHA-256
`d82b48d43fd777e46119a0ca4b7c7441842f6652ba6f4a962e01f4b09c78ec25`.
The array was loaded directly from the committed protocol JSON and executed
as argv without token reconstruction or substitution.

The observed invocation counters for this Task 8 execution session are:

| Invocation | Count | Return |
|---|---:|---|
| `smoke_a` | 1 | Exit 1 |
| `smoke_b` | 0 | Not executed |
| `smoke_analyzer_a` | 0 | Not executed |
| `smoke_analyzer_b` | 0 | Not executed |

The smoke-A traceback terminated at:

```text
scripts/diagnostics/replay_two_range_reacquisition.py:4307
  raise SystemExit(main())
scripts/diagnostics/replay_two_range_reacquisition.py:4293
  replay_two_range_reacquisition(...)
scripts/diagnostics/replay_two_range_reacquisition.py:3910
  raise ValueError("protocol source members differ from contract")
```

The exact terminal exception was:

```text
ValueError: protocol source members differ from contract
```

Protocol v1 has `retry_allowed: false` for every invocation.
Accordingly,
smoke A was not retried,
and no later smoke or analyzer command was executed.

This count is the reported process history for this bounded Task 8 session.
Filesystem and Git state cannot independently reconstruct all historical
local or remote process launches.
The command return,
the pre-root exception boundary,
and the continued absence of every target root are the independently
observable evidence available here.

## Deterministic root cause

The protocol `sources` object contains these 11
`REGISTERED_PROTOCOL_SOURCE_NAMES`,
in this exact order:

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

For `smoke_a`,
`_source_snapshots(...)` returns only these five
`RAW_SOURCE_MEMBER_NAMES["smoke_a"]` records:

```text
two_range_reacquisition_source
predictive_wnls_source
replay_source
mechanism_fixture
mechanism_fixture_manifest
```

At source lines 3900–3910,
the producer selects all 11 registered names only when
`invocation_name == "registered_replay"`.
For every other invocation,
including `smoke_a`,
it sets `expected_declared_names = tuple(sources)`,
which is the five-member smoke snapshot.
It then requires the complete protocol `sources` key tuple to equal that
five-member tuple exactly.

The committed 11-member protocol declaration therefore cannot equal the
five-member smoke declaration.
The mismatch deterministically raises
`ValueError("protocol source members differ from contract")`
before the producer opens or publishes the output root.

This is a protocol/source integration defect.
It is not input-data drift,
comparator drift,
disk exhaustion,
root preoccupation,
or a scientific-gate failure.

## Root and evidence state

The exact four smoke roots remained absent,
including broken-symlink checks:

```text
/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-b
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-b
```

The smoke-A exception occurred before root creation.
Consequently,
there was no failed root or manifest to retain.
The plan's retained-failure-root rule could not attach to an artifact because
the root transaction had not begun.
No root was deleted,
overwritten,
replaced,
or reused.

The two registered roots also remained absent:

```text
/private/tmp/cbf2026-two-range-reacquisition-development/v1
/private/tmp/cbf2026-two-range-reacquisition-analysis/v1
```

There are no smoke rows,
compressed or decompressed smoke-process identities,
raw manifests,
analysis JSON files,
analysis Markdown files,
analysis manifests,
semantic payloads,
selector-integrity gate results,
or compact-bundle hashes to report.
No scientific gate was evaluated.
No availability,
error,
tail,
paired-comparison,
or baseline-transition result exists.

This terminal failure is not negative scientific evidence about the
two-range method.
The code rejected the protocol declaration before estimator or analyzer
evidence production.

## Authorization and closure

The protocol authorization state remains exactly
`pending_external_record`.
The required authorization path remains absent:

```text
docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-registered-authorization.json
```

Task 8 is closed as FAIL under protocol v1.
The paper gate remains CLOSED.
The registered replay,
registered analyzer,
Stage 2,
and every paper-result action remain forbidden.

Protocol v1 and its four smoke roots must not be retried,
overwritten,
replaced,
or repurposed.
A corrective attempt requires:

1. a reviewed source-level integration fix;
2. a new protocol version with a new immutable registration and root plan;
3. a fresh independent preflight of the new source/protocol closure; and
4. fresh authorization for the new smoke execution boundary.

This report does not propose rerunning protocol v1.
Independent review of this terminal-failure record remains pending,
so no C0/I0/M0 closure is claimed.

## Independent terminal-failure report review

Review time: `2026-07-31T07:36:46Z`

Decision: `REPORT APPROVED; SMOKE GATE FAIL`

Report-integrity findings:

- Critical: `0`;
- Important: `0`; and
- Minor: `0`.

This verdict approves the accuracy and completeness of the terminal-failure
record.
It does not convert the failed smoke result into a pass.
Task 8 remains `FAIL`,
and the paper gate remains `CLOSED`.

The reviewed author text before this append-only adjudication was exactly
`10,774` bytes with SHA-256
`9ed08413f5d6f4436b3346b85764f98b9ef51b7e231add01888a2f69fb593dc1`.
The protocol,
preflight,
and implementation commit chain and scopes were independently rechecked.
The tracked and staged diffs were empty;
the only untracked entries were the preserved `build-diagnostic/` directory
and this smoke report.
No hidden source or protocol change was found.

The compact JSON identity of the exact committed 19-token
`commands.smoke_a` array was independently recomputed as
`d82b48d43fd777e46119a0ca4b7c7441842f6652ba6f4a962e01f4b09c78ec25`.
The ignored bounded-execution ledger records exactly one smoke-A invocation,
exit 1,
the exception
`ValueError: protocol source members differ from contract`,
and zero invocations of smoke B or either analyzer.
The report correctly separates this reported process history from the
independently observable Git and filesystem state.

All 11 source identities,
all seven comparator file identities,
and the implementation-parent Git blobs were rechecked.
The committed protocol contains 11 ordered source keys,
whereas
`RAW_SOURCE_MEMBER_NAMES["smoke_a"]`
contains exactly five.
The source at lines 3900--3910 selects those five smoke snapshot names and
compares them against the complete protocol key tuple before line 4061 can
create the output root.
The reported guard and pre-root failure boundary are therefore deterministic.

At review time,
`/private/tmp` had exactly `26,418,257,920` free bytes.
Using broken-symlink-aware checks,
all six protocol roots were absent:

```text
/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-b
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-b
/private/tmp/cbf2026-two-range-reacquisition-development/v1
/private/tmp/cbf2026-two-range-reacquisition-analysis/v1
```

The external registered-authorization record remained absent,
and the protocol authorization state remained
`pending_external_record`.
No smoke row,
raw or analysis manifest,
process identity,
compact analysis,
selector-integrity result,
or scientific result exists.
The report correctly makes no method-performance inference from this
integration failure.

Protocol v1 has consumed its single smoke-A opportunity.
It must not be retried,
and no later v1 command is authorized.
Any correction requires a reviewed source-level fix,
a new immutable protocol version and root plan,
a fresh independent preflight,
and fresh authorization for the new smoke boundary.
Protocol v1 and its roots must never be reused for that correction.
