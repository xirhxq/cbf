# CBF2026 two-range reacquisition v2 smoke terminal-failure report

Date: 2026-07-31

Status: TERMINAL FAILURE BEFORE FIRST PROTOCOL COMMAND

Deterministic smoke gate: FAIL

Registered gate: CLOSED

Paper gate: CLOSED

Independent review: NOT CREATED

This is the author-side terminal record for the authorized protocol-v2
deterministic smoke workflow.  It is not an independent review, contains no
scientific result, and does not authorize a retry or either registered
command.

## Approved boundary

The required and observed worktree HEAD before the operator attempt was
`84aa6851bfa61468f3e0a6555d081fe124fb2395`, the smoke-only preflight commit.
Its parent is the protocol commit
`5241d6a4d3ab8144ec065dc2f92c4c454a1d3760`, whose implementation parent is
`526e1418138426e54bff5a41e3847b6d2a9f8203`.

The formal preflight records C0/I0/M0 and authorizes only the exact committed
argv arrays `smoke_a`, `smoke_analyzer_a`, `smoke_b`, and
`smoke_analyzer_b`, in that order.  It does not authorize either registered
command, creation of an authorization record, DRA changes, or a paper claim.

| Frozen document | Bytes | SHA-256 |
| --- | ---: | --- |
| protocol v2 JSON | `69461` | `9a0985e00e1b6bd801f7b46c3811c2e135c29f86f75b5045e7aff1ad71206bb2` |
| protocol v2 Markdown | `4143` | `945a98760b2af676dedda513ea10c1e9c9c2a940d22a9ded689a11c0299f4251` |
| smoke-only preflight review | `14693` | `980a99f2cdcafc7ccf6cd316fb1690e629d619cd6d6d594b3490c0e4b338e6e0` |

The live files have these exact hashes and remain byte-identical to their
blobs at the required HEAD.  The failure described below was caused by the
operator harness transcribing the preflight hash incorrectly; it was not
caused by drift in any committed document.

## Invocation counters and immutable stop

The operator was designed to load each argv list directly from the committed
JSON, atomically advance its counter before `subprocess.Popen`, monitor
`/private/tmp` free space and allocated evidence bytes, and validate exact
terminal evidence before allowing the next command.  The initial
frozen-document gate failed before ledger initialization and before any
`Popen` call.

| Invocation | Compact argv tokens | Compact argv SHA-256 | Counter | Return | Terminal evidence |
| --- | ---: | --- | ---: | --- | --- |
| `smoke_a` | `19` | `647ad407d13c0d6cf9dba83582216d7bea5205f0d850bc714d598abacbe14067` | `0` | not executed | none |
| `smoke_analyzer_a` | `14` | `b1fbd68545a2da0626c1fcb0e480503171020afa035919def6f1787b1e7ddbf8` | `0` | not executed | none |
| `smoke_b` | `19` | `fdaef40962e90e1fa03f8d69e803b20ffa4804986f36fd87a6647b1a04147196` | `0` | not executed | none |
| `smoke_analyzer_b` | `14` | `1b08d1e595eb12931d2c0c964b01cb00fd2639dc0fca907279c04bab18556384` | `0` | not executed | none |

The compact hashes above were computed directly from the committed JSON
arrays with UTF-8 compact JSON encoding and no token reconstruction.
They identify the commands that were authorized but not launched.

The single operator-harness process returned exit `1` after approximately
`0.934` seconds.  No protocol-command start time, end time, return code,
stdout, stderr, disk minimum, or evidence allocation exists because the
failure occurred before ledger creation and before the first protocol
process.  The operator process wrote no stdout.  Its terminal stderr began:

```text
operator_failure=OperatorFailure: protocol/preflight hash differs
```

and the traceback terminated in
`load_and_validate_frozen_protocol()` before `main()` created the execution
ledger.

Under the exact no-retry and fail-closed Task 5 rule, the operator did not
correct the harness and did not start `smoke_a` or any later command.  This
authorized Task 5 workflow is therefore terminally failed; no retry is
authorized by this report.

## Root cause

The operator harness encoded this 62-character expected preflight value:

```text
980a99f2cdcafc7ccf6cd316fb1690e629d619cd6d594b3490c0e4b338e6e0
```

The actual committed SHA-256 is the valid 64-character value:

```text
980a99f2cdcafc7ccf6cd316fb1690e629d619cd6d6d594b3490c0e4b338e6e0
```

The missing `6d` in the harness constant made the initial dictionary
comparison fail even though the protocol JSON, protocol Markdown, and
preflight review were intact.  This is an operator precheck-transcription
failure.  It is not a producer failure, analyzer failure, source-binding
failure, comparator-binding failure, disk failure, or scientific-gate
failure.

## Forensic state

A read-only post-failure check at `2026-07-31T10:28:12.988044Z` found:

- HEAD remained exactly
  `84aa6851bfa61468f3e0a6555d081fe124fb2395`;
- tracked and staged diffs remained empty;
- normal porcelain state remained exactly `?? build-diagnostic/` before
  creation of this report;
- all 12 retired/current guarded roots were absent under broken-symlink-aware
  checks (`12/12`);
- both v1 and v2 authorization records were absent (`2/2`);
- the independent smoke-review path was absent;
- `/private/tmp` had `18,942,640,128` free bytes, above both the
  `8,000,000,000` launch floor and `6,000,000,000` live floor; and
- no execution ledger, raw bundle, compact bundle, manifest, process file,
  analysis JSON, or analysis Markdown existed.

Because no evidence root was created, allocated evidence was exactly zero
and there is no failed root to quarantine, delete, overwrite, or reuse.

## Evidence and scientific closure

No ordered 18-case raw evidence exists.  No raw compressed or decompressed
hash exists, no analyzer observed 18 rows, no integrity gate was evaluated,
no semantic payload was produced, and no A/B deterministic comparison was
possible.  Consequently, this failure provides neither positive nor
negative scientific evidence about the two-range reacquisition method.

The v2 authorization record remains absent and
`registered_full_grid_authorization` remains
`pending_external_record`.  `registered_replay` and `registered_analyzer`
were not executed.  Their expected 140000-row cost was not incurred.
Registered, Stage 2, DRA, and paper work remain CLOSED.

## Pre-execution harness fix round 1

The controller subsequently classified the event above as a non-consuming
operator precheck defect under the recovery plan's Global Constraints.
Because every protocol invocation counter remained `0`, no protocol-command
`Popen` occurred, no root existed, and no v2 evidence namespace was
consumed, the earlier terminal/no-retry disposition is superseded for this
specific pre-execution defect.  The original failure record above is
preserved without collapsing its history.

The controller explicitly authorized one corrected operator-harness attempt
followed, only on successful precheck, by the original exact sequence
`smoke_a` -> `smoke_analyzer_a` -> `smoke_b` ->
`smoke_analyzer_b`.  A failed protocol invocation remains terminal and
non-retryable.

The corrected gate no longer relies only on the mistyped transcription.  It
computes SHA-256 independently over both the committed
`84aa6851bfa61468f3e0a6555d081fe124fb2395` preflight blob and the live
working file, requires their bytes and full hashes to be equal, and requires
the resulting 64-character value to be
`980a99f2cdcafc7ccf6cd316fb1690e629d619cd6d6d594b3490c0e4b338e6e0`.
All other protocol invocation counters remain `0` at this supersession
boundary.

## Final authoritative disposition after fix round 1

Current status: V2 DETERMINISTIC SMOKE PASS

Registered gate: CLOSED

Paper gate: CLOSED

Independent review: NOT CREATED BY TASK 5 OPERATOR

This section supersedes the initial terminal disposition only because the
controller explicitly authorized pre-execution harness fix round 1 after
confirming that attempt 1 made zero protocol `Popen` calls and consumed no v2
namespace.  The initial failure and its counters remain part of the immutable
history above.

Corrected operator attempt 2 ran from
`2026-07-31T10:31:57.726879Z` through
`2026-07-31T10:32:20.326463Z` and exited `0`.  It loaded each argv list
directly from the committed JSON and called `subprocess.Popen` on that list
without shell reconstruction or substitution.

| Operator harness attempt | Return | Protocol `Popen` count | v2 namespace consumed | Result |
| --- | ---: | ---: | --- | --- |
| attempt 1 | `1` | `0` | no | 62-character preflight-hash transcription defect |
| authorized fix-round attempt 2 | `0` | `4` | yes | all four deterministic commands terminal success |

The final protocol invocation counters are exactly:

| Invocation | Counter | Return | Duration (s) | stdout | stderr |
| --- | ---: | ---: | ---: | --- | --- |
| `smoke_a` | `1` | `0` | `1.083844` | empty | empty |
| `smoke_analyzer_a` | `1` | `0` | `0.976413` | empty | empty |
| `smoke_b` | `1` | `0` | `0.982967` | empty | empty |
| `smoke_analyzer_b` | `1` | `0` | `0.872761` | empty | empty |
| `registered_replay` | `0` | not executed | n/a | n/a | n/a |
| `registered_analyzer` | `0` | not executed | n/a | n/a | n/a |

No command was retried.

## Per-command prechecks and resource observations

Immediately before every command, the operator independently rechecked:

- HEAD exactly
  `84aa6851bfa61468f3e0a6555d081fe124fb2395`, empty tracked and staged
  diffs, and exact porcelain state containing only the pre-existing
  `build-diagnostic/` plus this preserved author report;
- protocol JSON SHA-256
  `9a0985e00e1b6bd801f7b46c3811c2e135c29f86f75b5045e7aff1ad71206bb2`,
  protocol Markdown SHA-256
  `945a98760b2af676dedda513ea10c1e9c9c2a940d22a9ded689a11c0299f4251`,
  and committed/live preflight SHA-256
  `980a99f2cdcafc7ccf6cd316fb1690e629d619cd6d6d594b3490c0e4b338e6e0`;
- all 11 source identities and their applicable implementation-parent blobs;
- all seven comparator identities, including the decompressed gzip hash
  domain;
- approved design SHA-256
  `d28d6bfe297a6e37b0a878dc716bd5b91f26c5b11568c3f6fc42bd07760a266b`
  and design-review SHA-256
  `6d857be965de5b83bb05ce06381b79108fdc4abbccf6b96fe4ec456d7a7df570`;
- all six retired v1 roots, both v2 registered roots, and both authorization
  paths absent under broken-symlink-aware checks;
- the current target and every later root absent, while every earlier root
  was revalidated as exact terminal evidence; and
- the `8,000,000,000`-byte launch floor, `6,000,000,000`-byte live floor,
  `2,000,000,000`-byte raw cap, and `10,000,000`-byte compact cap.

| Invocation | Precheck UTC | Target/future absent | Free before precheck | Start free | Minimum live free | End free | Max current allocation | Max aggregate smoke allocation |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `smoke_a` | `2026-07-31T10:32:02.383699Z` | `4/4` | `17,399,803,904` | `17,399,803,904` | `17,399,754,752` | `17,399,754,752` | `8,192` | `8,192` |
| `smoke_analyzer_a` | `2026-07-31T10:32:08.183087Z` | `3/3` | `17,365,041,152` | `17,365,041,152` | `17,364,987,904` | `17,364,987,904` | `16,384` | `24,576` |
| `smoke_b` | `2026-07-31T10:32:13.962572Z` | `2/2` | `17,297,850,368` | `17,297,850,368` | `17,296,003,072` | `17,296,003,072` | `8,192` | `32,768` |
| `smoke_analyzer_b` | `2026-07-31T10:32:19.254144Z` | `1/1` | `17,295,974,400` | `17,295,974,400` | `17,295,896,576` | `17,295,896,576` | `16,384` | `49,152` |

Every observed free-space value exceeded both floors.  Every raw root used
`8,192` allocated bytes, every compact root used `16,384` allocated bytes,
and the final aggregate was `49,152` bytes, all within the frozen limits.
No live monitor failure occurred.

The source hashes revalidated on each precheck were:

| Source member | SHA-256 |
| --- | --- |
| `implementation_plan` | `d4bcbff50c557d954ce8c1f0d0ab2ccb8808be721808a48f028a44ed9bbd7037` |
| `two_range_reacquisition_source` | `d226c0184d89aadfe7b25a4a52d27379a396f868cfaab2c5a8a13ee7cdc04f8c` |
| `predictive_wnls_source` | `0269baa715c087661f2fa4c4760345250b380e2a14cce021fc3aca1b62f2defc` |
| `fixture_extractor_source` | `4c35e8c0ce8858ff247f07639b9b190f3196d31d80721b12b90bc507c24b8775` |
| `replay_source` | `c86fd4b017b69e85749372acfe516cabd4e58aebd9749a55ac7eb5e5ec5ea52c` |
| `analyzer_source` | `0818bdb944d2270d98efdb130e5a96a7dbaedfc090a4a35693e4e803437a6305` |
| `registrar_source` | `b918ce2715d26f691dc7de7ed60c1e654a1083ba6bed0ca9b12a6682a46036d4` |
| `mechanism_fixture` | `9febff2393017b7b0fbd1a02dd76a13d1d70639f2b176df236931fe29a8601c7` |
| `mechanism_fixture_manifest` | `cb704d10d439dac55170ce14a9feef27c92cd60ec2a95ccb4c47a2cec73a722d` |
| `truth_data` | `3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527` |
| `input_manifest` | `6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb` |

The comparator hashes revalidated on each precheck were:

| Comparator member | SHA-256 |
| --- | --- |
| `v4_replay_manifest` | `123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223` |
| `v4_compressed_process` | `9710d1a5792e682bb272d0bb3305162180d1e0b9a0636647d6e071d1782a699b` |
| `v4_decompressed_process` | `7047989804fd03bd4cadffac8212349b7183857623ac201d14ef2fc3e30ee2e1` |
| `v4_analysis_manifest` | `e50352825665bb94c0e3b32c7922a081f093100daf0ed8e5c8ce9e9f32bd9238` |
| `v4_analysis_json` | `8a0a417f2d3b139fa248bef8157a3d4871fa04042aaad82ce5cd31f470a6ff1e` |
| `v4_analysis_markdown` | `986dcf72f72ab6dc3d424d0de6cdbf410cf271d174d16caef32945acc7d4e13b` |
| `legacy_baseline_process` | `c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003` |

## Exact command and terminal evidence

| Invocation | Compact argv SHA-256 | Start UTC | End UTC | Manifest SHA-256 | Manifest status | Rows |
| --- | --- | --- | --- | --- | --- | ---: |
| `smoke_a` | `647ad407d13c0d6cf9dba83582216d7bea5205f0d850bc714d598abacbe14067` | `2026-07-31T10:32:02.383859Z` | `2026-07-31T10:32:03.468347Z` | `dc98d774db8300c3b397c3e40422d15b1ba46c394eb29b22b3ee85f7f4391207` | `completed` | `18/18` |
| `smoke_analyzer_a` | `b1fbd68545a2da0626c1fcb0e480503171020afa035919def6f1787b1e7ddbf8` | `2026-07-31T10:32:08.183197Z` | `2026-07-31T10:32:09.160499Z` | `07079824d6abb613da295056f587ca514a5d6bc75437aabb9d08c15165434f0f` | `completed` | `18/18` |
| `smoke_b` | `fdaef40962e90e1fa03f8d69e803b20ffa4804986f36fd87a6647b1a04147196` | `2026-07-31T10:32:13.962658Z` | `2026-07-31T10:32:14.946691Z` | `3684a69121d273a6eaec364b1fbd3d173a6648571f800f20beccc79d1012e628` | `completed` | `18/18` |
| `smoke_analyzer_b` | `1b08d1e595eb12931d2c0c964b01cb00fd2639dc0fca907279c04bab18556384` | `2026-07-31T10:32:19.254233Z` | `2026-07-31T10:32:20.129144Z` | `a0d6bd059c18a2d93241a4478aec63e3f9953b42d7e2373512898c0fbbe6eb85` | `completed` | `18/18` |

Both raw manifests are terminal and self-consistent.  Each process file is
`3,595` bytes with `4,096` allocated bytes.  Both contain the exact ordered
case IDs:

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

Both analyzer manifests are terminal and self-consistent.  Each reports
`observed_rows=18`, exactly zero scientific gates, exactly 14 integrity
gates with `14/14` passing, and `decision="smoke_pass"`.

| Analyzer | Analysis JSON SHA-256 | Analysis Markdown SHA-256 | Semantic payload SHA-256 |
| --- | --- | --- | --- |
| A | `26753574fa3e9217f18180a2bf0c51a34d30b763134913e483a2dbbe9e378c0d` | `3f49d592cfd127ef751d638838975b5f2a55655a88944290b5eaf2a4a485b526` | `55c636e41c121e60eaa2341d0524ff5c08838374969343eec1e9f916a1758247` |
| B | `0679640eb15f77a66ad3c8b2ecbcf9f8d6e6290b1d8821ce4869d789d73f2c3c` | `80d68acd68cf84d7f90bb072b0496743fad51856aee45954cc7d5063440fb13e` | `55c636e41c121e60eaa2341d0524ff5c08838374969343eec1e9f916a1758247` |

The complete JSON and Markdown file hashes differ only because their bound
invocation/path provenance differs.  The frozen semantic projection excludes
those permitted provenance differences.  The independently recomputed
semantic hashes are equal, and the Markdown semantic lines after excluding
the invocation line are byte-equal.

## Deterministic comparison

| Comparison | A | B | Result |
| --- | --- | --- | --- |
| compressed raw SHA-256 | `8073457c8e7a21a5ec7c6d85baaec4c77a549ea66422e474ad927f323160677c` | `8073457c8e7a21a5ec7c6d85baaec4c77a549ea66422e474ad927f323160677c` | equal |
| decompressed raw SHA-256 | `cdbde96b17219f69cf0c00f403718048d8d0d04b4943ce53d3580978f240d435` | `cdbde96b17219f69cf0c00f403718048d8d0d04b4943ce53d3580978f240d435` | equal |
| analyzer semantic SHA-256 | `55c636e41c121e60eaa2341d0524ff5c08838374969343eec1e9f916a1758247` | `55c636e41c121e60eaa2341d0524ff5c08838374969343eec1e9f916a1758247` | equal |
| ordered case IDs | exact 18-case declaration | exact 18-case declaration | equal |
| normalized Markdown semantics | `smoke_pass`, `18/18`, same semantic hash | `smoke_pass`, `18/18`, same semantic hash | equal |

The deterministic v2 smoke gate therefore passes.

## Final closed boundary

At `2026-07-31T10:32:20.326149Z`, the execution ledger's final closure check
reconfirmed exact HEAD, empty tracked/staged diffs, six retired v1 roots
absent, two v2 registered roots absent, both authorization records absent,
and the independent smoke-review path absent.  `/private/tmp` had
`17,295,884,288` free bytes and the four retained smoke roots occupied
`49,152` allocated bytes.

Neither `registered_replay` nor `registered_analyzer` was executed.  The v2
authorization record remains absent, the expected registered cost remains
140000 rows, and registered, Stage 2, DRA, and paper gates remain CLOSED.
The smoke result validates deterministic execution and integrity only; with
zero scientific gates it is not a scientific-performance claim.
