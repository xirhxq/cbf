# CBF2026 two-range reacquisition protocol v2 independent preflight review

Date: 2026-07-31

Reviewed worktree: `/private/tmp/cbf2026-diagnostic`

Reviewed branch: `codex/cbf2026-diagnostic`

Reviewed protocol commit:
`5241d6a4d3ab8144ec065dc2f92c4c454a1d3760`

Declared and actual implementation parent:
`526e1418138426e54bff5a41e3847b6d2a9f8203`

## Decision

| Severity | Count |
| --- | ---: |
| Critical | `0` |
| Important | `0` |
| Minor | `0` |

APPROVED FOR THE FOUR DETERMINISTIC V2 SMOKE COMMANDS ONLY

This approval is limited to the exact committed command arrays named
`smoke_a`, `smoke_b`, `smoke_analyzer_a`, and `smoke_analyzer_b`.
It does not authorize `registered_replay` or `registered_analyzer`, creation
of either registered root, creation of an authorization record, Stage 2,
DRA changes, paper edits, or a paper claim.
No smoke command has been executed yet.
The paper gate remains `CLOSED`.

## Reviewed inputs and review boundary

The independent preflight read the exact Task 4 brief and its complete
process report, including the failed `/var` dry attempt and the corrected dry
history; the formal implementation review through its latest fix round 2;
the committed protocol JSON and Markdown; the recovery-plan Global
Constraints; and the current registrar, replay, analyzer, scientific source,
fixture, and binding declarations used by the protocol.

The implementation review's latest round 2 records final Critical `0`,
Important `0`, Minor `0` and opens only protocol generation.
It explicitly leaves execution, registered, authorization, DRA, and paper
gates closed.

This preflight did not execute any of the six protocol command arrays.
It did not create an evidence root, authorization record, smoke report,
smoke review, DRA append, index entry, or paper artifact.
It did not modify production source, tests, protocol bytes, or Git state.

## Canonical in-memory reconstruction

The reviewer independently resolved all live source and comparator bindings,
assembled the canonical pre-serialization protocol with the declared
implementation parent, ran the registrar's strict protocol validator on that
pre-serialization object, and generated JSON and Markdown bytes entirely in
memory.
Both reconstructed payloads matched the committed artifacts byte-for-byte:

| Artifact | Bytes | SHA-256 | Exact byte comparison |
| --- | ---: | --- | --- |
| protocol v2 JSON | `69461` | `9a0985e00e1b6bd801f7b46c3811c2e135c29f86f75b5045e7aff1ad71206bb2` | equal |
| protocol v2 Markdown | `4143` | `945a98760b2af676dedda513ea10c1e9c9c2a940d22a9ded689a11c0299f4251` | equal |

The committed JSON is strict, canonical JSON with no duplicate member or
non-finite token.
The committed Markdown contains the exact JSON SHA-256 binding and the exact
six canonical command arrays.

## Protocol commit and non-circular provenance

The reviewed HEAD is exactly the protocol commit above.
It is a single-parent commit, and its sole parent is exactly the declared
implementation parent.
Its complete changed-path set contains only:

```text
docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json
docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.md
```

Both live files are byte-identical to their blobs at the protocol commit.
Neither path exists in the implementation-parent tree.
The protocol commit changes no source, test, implementation review, DRA,
index, smoke, authorization, or paper file.
The parent is therefore non-circular and the protocol commit contains only
the two frozen protocol documents.

## Source identity revalidation

All 11 ordered source declarations were independently lstat-checked,
opened without following the leaf, re-statted through the open descriptor,
and re-hashed.
Every live identity matched the committed ordered six-field record
`path`, `device`, `inode`, `size`, `mtime_ns`, `sha256`.
The nine repository sources were also compared byte-for-byte with their Git
blobs at the declared implementation parent and at the protocol commit.

| Source member | Bytes | SHA-256 | Parent blob |
| --- | ---: | --- | --- |
| `implementation_plan` | `27313` | `d4bcbff50c557d954ce8c1f0d0ab2ccb8808be721808a48f028a44ed9bbd7037` | exact |
| `two_range_reacquisition_source` | `14633` | `d226c0184d89aadfe7b25a4a52d27379a396f868cfaab2c5a8a13ee7cdc04f8c` | exact |
| `predictive_wnls_source` | `54412` | `0269baa715c087661f2fa4c4760345250b380e2a14cce021fc3aca1b62f2defc` | exact |
| `fixture_extractor_source` | `26185` | `4c35e8c0ce8858ff247f07639b9b190f3196d31d80721b12b90bc507c24b8775` | exact |
| `replay_source` | `160343` | `c86fd4b017b69e85749372acfe516cabd4e58aebd9749a55ac7eb5e5ec5ea52c` | exact |
| `analyzer_source` | `175462` | `0818bdb944d2270d98efdb130e5a96a7dbaedfc090a4a35693e4e803437a6305` | exact |
| `registrar_source` | `101339` | `b918ce2715d26f691dc7de7ed60c1e654a1083ba6bed0ca9b12a6682a46036d4` | exact |
| `mechanism_fixture` | `3432` | `9febff2393017b7b0fbd1a02dd76a13d1d70639f2b176df236931fe29a8601c7` | exact |
| `mechanism_fixture_manifest` | `863` | `cb704d10d439dac55170ce14a9feef27c92cd60ec2a95ccb4c47a2cec73a722d` | exact |
| `truth_data` | `16237150` | `3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527` | external binding exact |
| `input_manifest` | `1226` | `6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb` | external binding exact |

## Comparator identity revalidation

All seven comparator identity records were independently re-statted and
re-hashed.
The gzip comparator was hashed both as its compressed bytes and through a
pinned decompression stream.
All path, device, inode, size, mtime, and applicable hash fields matched the
committed protocol.

| Comparator member | File bytes | SHA-256 |
| --- | ---: | --- |
| `v4_replay_manifest` | `9560` | `123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223` |
| `v4_compressed_process` | `206565170` | `9710d1a5792e682bb272d0bb3305162180d1e0b9a0636647d6e071d1782a699b` |
| `v4_decompressed_process` | `206565170` | `7047989804fd03bd4cadffac8212349b7183857623ac201d14ef2fc3e30ee2e1` |
| `v4_analysis_manifest` | `4827` | `e50352825665bb94c0e3b32c7922a081f093100daf0ed8e5c8ce9e9f32bd9238` |
| `v4_analysis_json` | `123611` | `8a0a417f2d3b139fa248bef8157a3d4871fa04042aaad82ce5cd31f470a6ff1e` |
| `v4_analysis_markdown` | `39055` | `986dcf72f72ab6dc3d424d0de6cdbf410cf271d174d16caef32945acc7d4e13b` |
| `legacy_baseline_process` | `68754629` | `c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003` |

## Protocol contract validation

The preflight independently confirmed the exact enclosing identities:

| Contract | Identity |
| --- | --- |
| Protocol schema | `cbf2026-two-range-reacquisition-protocol-v2` |
| Registration schema | `cbf2026-two-range-reacquisition-registration-v2` |
| Protocol ID | `cbf2026-two-range-reacquisition-v2` |
| Raw schema | `cbf2026-two-range-reacquisition-raw-v1` |
| Analysis schema | `cbf2026-two-range-reacquisition-analysis-v1` |
| Measurement seed contract | `cbf2026-range-v1` |

The raw and analysis schemas correctly remain v1 because their serialized
field contracts are unchanged.

The exact 18 ordered smoke case IDs were verified, including the one bound
mechanism fixture and 17 serialized synthetic cases.
The registered grid was verified as 20 ordered seeds, 500 ordered frames,
14 ordered UAV IDs, and exactly `140000` ordered keys under
`method`, `seed`, `frame_index`, `robot_id`.

All six estimator constants were exact:

```text
maximum_public_prediction_age = 2
innovation_reference_quantile = 11.829007011943707
candidate_dedup_m = 1e-09
motion_covariance_per_frame = 0.25
reacquisition_reduced_cost_max = 9.0
maximum_error_m = 50.0
```

The exact disk contract was:

```text
launch_minimum_free_bytes = 8000000000
live_minimum_free_bytes = 6000000000
raw_bundle_max_allocated_bytes = 2000000000
compact_bundle_max_allocated_bytes = 10000000
```

The live `/private/tmp` free-space reading during preflight was
`22688927744` bytes, above the launch minimum.

The exact nine scientific gate IDs, operators, and thresholds and the exact
14 integrity gate IDs with zero-tolerance equality records matched the
registrar and analyzer declarations.
The aggregate decision remains pass if and only if every scientific and
integrity gate passes.

Every one of the six invocation records has `retry_allowed: false`.
The two registered invocation records require authorization; the four smoke
records do not.
The evidence lifecycle retains no-follow, descriptor pinning,
transactional publication, fsync, terminal-manifest, failure-retention, and
closed-paper requirements.

## Exact command arrays

All six serialized argv arrays were compared with an independent canonical
reconstruction and matched exactly in order and value.
Their protocol path, data or raw input, output root, seed and frame tokens,
invocation name, and authorization tokens are exact.

| Command array | Rows/grid | Authorization | Preflight disposition |
| --- | --- | --- | --- |
| `smoke_a` | 18 cases | not required | approved, not executed |
| `smoke_b` | 18 cases | not required | approved, not executed |
| `smoke_analyzer_a` | 18 cases | not required | approved, not executed |
| `smoke_analyzer_b` | 18 cases | not required | approved, not executed |
| `registered_replay` | 140000 keys | required, record absent | unauthorized |
| `registered_analyzer` | 140000 keys | required, record absent | unauthorized |

No command array was executed during this review.

## Root, authorization, and gated-output absence

Before creating this review, independent `lstat`-aware checks found all six
retired v1 roots and all six current v2 roots absent under both existence and
symlink semantics.
The exact 12 guarded roots were:

```text
/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-b
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-b
/private/tmp/cbf2026-two-range-reacquisition-development/v1
/private/tmp/cbf2026-two-range-reacquisition-analysis/v1
/private/tmp/cbf2026-two-range-reacquisition-smoke-v2-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-v2-b
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v2-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v2-b
/private/tmp/cbf2026-two-range-reacquisition-development/v2
/private/tmp/cbf2026-two-range-reacquisition-analysis/v2
```

The retired v1 and current v2 authorization paths were both absent and were
not symlinks.
The v2 preflight-review path, smoke-report path, and smoke-review path were
all absent and were not symlinks before this review was written.

## Real serialized public-entry boundary probes

An ignored temporary harness called the real public
`replay_two_range_reacquisition(...)` entry point separately for `smoke_a`
and `smoke_b`.
For each call, it parsed the exact arguments from the corresponding committed
serialized command array: the committed protocol path, exact truth and input
manifest paths, exact v2 output root, empty `--run-seeds`, `--max-frames 0`,
the exact invocation name, and no authorization argument.

The harness replaced only the root-allocation boundary
`_create_exact_root(...)` with a private sentinel and asserted the exact
requested root at that boundary.
Because the public entry intentionally catches any preallocation exception
and normally publishes a forensic failure manifest, the harness also
installed a no-write assertion guard on that failure publisher.
The guard verified that the sentinel was the sole preallocation failure and
then propagated it without writing.

Both committed serialized invocations completed all pre-root validation and
reached the private sentinel at their exact roots:

```text
/private/tmp/cbf2026-two-range-reacquisition-smoke-v2-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-v2-b
```

Post-probe checks found neither output root and neither corresponding
preallocation-failure manifest.
All 12 guarded roots, both authorization paths, and the preflight/smoke
outputs remained absent.
These calls were boundary probes, not executions of either command array,
and produced no scientific evidence.

## Generation history audit

The Task 4 process history was preserved rather than collapsed to two dry
attempts.
The first non-publishing dry attempt used a default macOS `/var/folders/...`
temporary path and failed before output creation with:

```text
ValueError: symbolic-link path component is forbidden: /var
```

After explicit dry-path fix-round authorization, two new canonical
`/private/tmp` dry builds both succeeded and matched byte-for-byte.
The immutable totals are dry attempted `3`, successful `2`, failed `1`.
The two successful dry identities are the same JSON and Markdown identities
listed above.

The production registrar was attempted exactly once and succeeded exactly
once, for production `1/1`.
Its bytes matched both successful dry builds.
No production regeneration occurred.
Protocol command arrays remained executed `0`, and evidence roots remained
created `0`, through protocol freeze.

## Non-finding transparency note

The initial local harness included one inapplicable extra check: it passed
the already JSON-deserialized protocol to the registrar's private
pre-serialization `_validate_protocol(...)` interface.
JSON round-trip conversion of tuple-valued synthetic declarations to lists
correctly made that private generation-time interface report
`serialized smoke case declaration differs`.
No mutation, root, or manifest occurred.

That check was removed rather than weakening or changing source or protocol
bytes.
The correct interface evidence then passed: the live bound state was rebuilt
as the tuple-valued pre-serialization object and validated, its deterministic
bytes exactly equaled the committed JSON/Markdown, and both public replay
calls consumed the committed serialized JSON through the root-allocation
sentinel.
The inapplicable private-interface call is therefore recorded as harness
history and is not a protocol finding.

## Findings

### Critical findings

None.

### Important findings

None.

### Minor findings

None.

## Authorization boundary

Only the four exact deterministic v2 smoke command arrays are approved.
The exact registered authorization path remains absent, and the protocol
still declares `registered_full_grid_authorization` as
`pending_external_record`.
The registered replay and registered analyzer commands remain unauthorized;
both registered roots must remain absent.
No smoke has been run yet, so Stage 2, DRA, and the paper remain closed.
