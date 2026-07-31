# CBF2026 two-range reacquisition protocol v1 preflight review

Date: 2026-07-31

Decision: APPROVED FOR THE TWO DETERMINISTIC SMOKE PAIRS ONLY

Review result: 0 Critical, 0 Important, 0 Minor.

The registered replay and registered analyzer remain unauthorized.
This review does not authorize either registered command,
does not report a scientific result,
does not open the paper gate,
and does not authorize Stage 2 or any paper edit.

## Commit and artifact closure

The protocol commit is
`98708018e0076eb5bd38f20b2cbeea6b971a9fd3`
(`docs(diagnostics): freeze two-range protocol`).
Its sole parent is the reviewed implementation closure
`5a14eb66ffeb541540183041ce2966c5dfd6d949`.
The protocol declares that exact parent.

The commit adds exactly these two paths and changes no implementation source:

- `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json`;
- `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.md`.

Both paths are absent from the implementation parent,
so the binding is non-circular.
The approved design ancestor is
`20a61aad96af35ee7e16434fab0a5edaaea38ef0`.

| Artifact | Bytes | SHA-256 |
|---|---:|---|
| Protocol JSON | 69,464 | `8e01df56623bb2e759fb37108ddc68fbc41e301b8305ade7322e212ae6e3ea16` |
| Protocol Markdown | 4,137 | `c9e3930d22355747ee12e96303c1702aa0385de2881f6b430a72030b89d7e0ff` |
| Approved design | 29,343 | `d28d6bfe297a6e37b0a878dc716bd5b91f26c5b11568c3f6fc42bd07760a266b` |
| Approved design review | 3,272 | `6d857be965de5b83bb05ce06381b79108fdc4abbccf6b96fe4ec456d7a7df570` |

The implementation-parent registrar interfaces were used to resolve the
bound sources and comparators and build the protocol in memory.
The in-memory object passed the registrar's strict validator.
Its canonical JSON and Markdown serialization matched the two committed
files byte for byte.
The committed Markdown contains the committed JSON SHA-256 exactly once,
and its six rendered commands match the JSON arrays.

The internal validator operates on the registrar's pre-serialization object,
whose smoke declaration contains tuple-valued members.
A direct call on `json.loads(...)` changes those members to JSON lists and
therefore triggers its type-sensitive
`serialized smoke case declaration differs` guard.
This is not artifact drift:
the parsed declaration equals the JSON-normalized source declaration,
its declared hash matches,
and the canonical in-memory rebuild is byte-identical to the commit.

## Exact source binding

All 11 declared source records were independently re-statted and rehashed.
The nine repository-owned files also matched their Git blobs at
`5a14eb66ffeb541540183041ce2966c5dfd6d949`.

| Member | Exact path | Bytes | SHA-256 |
|---|---|---:|---|
| `implementation_plan` | `/private/tmp/cbf2026-diagnostic/docs/superpowers/plans/2026-07-30-cbf2026-two-range-reacquisition-implementation.md` | 164,746 | `644bbe01b1cf759f700b68d9ae5113cd9ebc62b33b3454769e6c057f409cc8bd` |
| `two_range_reacquisition_source` | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/two_range_reacquisition.py` | 14,633 | `d226c0184d89aadfe7b25a4a52d27379a396f868cfaab2c5a8a13ee7cdc04f8c` |
| `predictive_wnls_source` | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/predictive_wnls.py` | 54,412 | `0269baa715c087661f2fa4c4760345250b380e2a14cce021fc3aca1b62f2defc` |
| `fixture_extractor_source` | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/extract_two_range_reacquisition_fixture.py` | 26,185 | `4c35e8c0ce8858ff247f07639b9b190f3196d31d80721b12b90bc507c24b8775` |
| `replay_source` | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/replay_two_range_reacquisition.py` | 160,792 | `d8d670934a3d0e21479dac4484a658e253478a8d3ccc709aa19ad3699f72db23` |
| `analyzer_source` | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/analyze_two_range_reacquisition.py` | 175,462 | `0818bdb944d2270d98efdb130e5a96a7dbaedfc090a4a35693e4e803437a6305` |
| `registrar_source` | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/register_two_range_reacquisition.py` | 100,660 | `87885a19288e879a44ea4ef4cfecc2c5576f381a8599047fb0eca360355c3bee` |
| `mechanism_fixture` | `/private/tmp/cbf2026-diagnostic/tests/fixtures/cbf2026_two_range_reacquisition/mechanism_20260727_180_12.json` | 3,432 | `9febff2393017b7b0fbd1a02dd76a13d1d70639f2b176df236931fe29a8601c7` |
| `mechanism_fixture_manifest` | `/private/tmp/cbf2026-diagnostic/tests/fixtures/cbf2026_two_range_reacquisition/manifest.json` | 863 | `cb704d10d439dac55170ce14a9feef27c92cd60ec2a95ccb4c47a2cec73a722d` |
| `truth_data` | `/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json` | 16,237,150 | `3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527` |
| `input_manifest` | `/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json` | 1,226 | `6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb` |

## Exact comparator binding

The comparator roots are exactly:

- `/private/tmp/cbf2026-predictive-wnls-development/stage1-v4`;
- `/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v4`.

All seven comparator file records were independently re-statted and rehashed.
The compressed and decompressed v4 hashes were recomputed through one
pinned gzip path and matched their distinct hash domains.

| Member | Exact path | Bytes | SHA-256 |
|---|---|---:|---|
| `v4_replay_manifest` | `/private/tmp/cbf2026-predictive-wnls-development/stage1-v4/manifest.json` | 9,560 | `123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223` |
| `v4_compressed_process` | `/private/tmp/cbf2026-predictive-wnls-development/stage1-v4/predictive-wnls-development.jsonl.gz` | 206,565,170 | `9710d1a5792e682bb272d0bb3305162180d1e0b9a0636647d6e071d1782a699b` |
| `v4_decompressed_process` | `/private/tmp/cbf2026-predictive-wnls-development/stage1-v4/predictive-wnls-development.jsonl.gz` | 206,565,170 storage bytes | `7047989804fd03bd4cadffac8212349b7183857623ac201d14ef2fc3e30ee2e1` over decompressed bytes |
| `v4_analysis_manifest` | `/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v4/manifest.json` | 4,827 | `e50352825665bb94c0e3b32c7922a081f093100daf0ed8e5c8ce9e9f32bd9238` |
| `v4_analysis_json` | `/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v4/predictive-wnls-development.json` | 123,611 | `8a0a417f2d3b139fa248bef8157a3d4871fa04042aaad82ce5cd31f470a6ff1e` |
| `v4_analysis_markdown` | `/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v4/predictive-wnls-development.md` | 39,055 | `986dcf72f72ab6dc3d424d0de6cdbf410cf271d174d16caef32945acc7d4e13b` |
| `legacy_baseline_process` | `/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/restart/localization-calibration/20260729T112437.413270Z_efb8e7137776415c833f11204cf519d5/calibration.jsonl.gz` | 68,754,629 | `c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003` |
| `legacy_baseline_protocol_json_sha256` | `docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol-v4.json` | 17,245 | `09b236978e2e21bac226dc3d00c36a2ee5cdf5fea0e616f2c1d875b029d4e4e0` |

## Schema, grid, smoke, and gate audit

The four exact schema IDs are:

| Role | Schema ID |
|---|---|
| Protocol | `cbf2026-two-range-reacquisition-protocol-v1` |
| Registration authorization | `cbf2026-two-range-reacquisition-registration-v1` |
| Raw evidence | `cbf2026-two-range-reacquisition-raw-v1` |
| Compact analysis | `cbf2026-two-range-reacquisition-analysis-v1` |

The analysis-manifest field tuple exactly matches the analyzer source:

```text
schema_id, protocol_id, invocation_name, status, method, output_root,
protocol_identity, authorization_identity, source_identities,
output_identities, expected_rows, observed_rows, disk_contract,
started_at, completed_at, error
```

The registered Cartesian grid is exactly 20 seeds
`20260727` through `20260746`,
500 frames `0` through `499`,
and 14 robots `1` through `14`.
It therefore declares exactly
`20 * 500 * 14 = 140000`
ordered `(method, seed, frame_index, robot_id)` keys.

The producer source and serialized raw schema bind these 18 smoke cases in
this exact order:

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

Both smoke replay invocations declare 18 rows.
Their canonical arrays place `--max-frames 0` immediately after the empty
`--run-seeds` option,
so they request no trajectory seed and only the approved fixture/synthetic
matrix.
Both smoke analyzers bind only their corresponding 18-row smoke roots.
None of the four smoke arrays contains an authorization option.

The nine scientific gates are exact and ordered:

| Gate | Operator | Threshold |
|---|---|---:|
| `maximum_published_error_m_strictly_below` | `strictly_below` | 50.0 |
| `maximum_fresh_error_m_strictly_below` | `strictly_below` | 50.0 |
| `paired_both_fresh_p95_must_not_worsen` | `less_than_or_equal` | 0.0 |
| `fresh_availability_max_drop_fraction` | `less_than_or_equal` | 0.02 |
| `fresh_or_predicted_min_fraction` | `greater_than_or_equal` | 0.95 |
| `maximum_prediction_age_frames` | `less_than_or_equal` | 2 |
| `qualification_anchor_violations_allowed` | `equal` | 0 |
| `current_frame_provenance_violations_allowed` | `equal` | 0 |
| `ascending_dag_violations_allowed` | `equal` | 0 |

The 14 selector-integrity gates are exact and ordered,
each with operator `equal` and non-Boolean integer threshold `0`:

1. `nonfresh_anchor_use`;
2. `selector_reference_set_violation`;
3. `missing_fixed_reference_publication`;
4. `private_prior_role_violation`;
5. `noncircle_continuous_start`;
6. `noncircle_publication_representative`;
7. `nonruntime_branch_score`;
8. `branch_selection_reconstruction_mismatch`;
9. `nonunique_passing_branch_publication`;
10. `selected_result_binding_mismatch`;
11. `private_state_recursion_mismatch`;
12. `predicted_selector_output`;
13. `exact_denominator_violation`;
14. `preserved_contract_violation`.

## Command, authorization, root, and disk preflight

All six arrays are literal non-shell argument arrays.
They match `production_command_contract(...)` exactly,
and the Markdown rendering preserves their token order.
For an independently reproducible compact identity,
each hash below is over the UTF-8 compact JSON encoding of the named array.

| Command | Tokens | Array SHA-256 | Current authorization |
|---|---:|---|---|
| `smoke_a` | 19 | `d82b48d43fd777e46119a0ca4b7c7441842f6652ba6f4a962e01f4b09c78ec25` | Approved by this preflight |
| `smoke_b` | 19 | `7a8715f032e831a091a086ec0ce016d548ef1d905a6dc7a408086117a1d834d5` | Approved by this preflight |
| `smoke_analyzer_a` | 14 | `6b5aff0346044ec1a1f59b33d3ce2846ad9077b0af08272cbcd05b2e4dbcefe4` | Approved only after smoke A completes |
| `smoke_analyzer_b` | 14 | `dd0acd2d3f27b6baf2e5bcc7a43834e3b46e9c2ff90e0a39eba345984fc785f8` | Approved only after smoke B completes |
| `registered_replay` | 41 | `2e86cd676a8c905c7e23a0ed3c8d80e1110934c00ce15d5b3206d00359484c15` | Not authorized |
| `registered_analyzer` | 16 | `0d9cee47a99af1e239b81243681894746a2b8331354c23b3a3e702c766202151` | Not authorized |

The registered replay separately requests all 20 seeds and 500 frames.
Both registered arrays require
`docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-registered-authorization.json`.
That path is absent,
and the protocol state is exactly `pending_external_record`.

At `2026-07-31T07:12:02Z`,
all six exact roots were absent,
including broken-symlink checks:

```text
/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-b
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-b
/private/tmp/cbf2026-two-range-reacquisition-development/v1
/private/tmp/cbf2026-two-range-reacquisition-analysis/v1
```

Free space under `/private/tmp` was exactly `26,479,374,336` bytes.
This exceeds the `8,000,000,000`-byte launch floor.
The live floor is `6,000,000,000` bytes,
the raw allocation cap is `2,000,000,000` bytes,
and the compact allocation cap is `10,000,000` bytes.

Every invocation has `retry_allowed: false`.
The protocol also forbids pre-existing targets,
requires no-follow and descriptor pinning,
requires transactional publication,
`fsync`,
terminal manifests,
and retained failure evidence.
Its paper gate is `CLOSED`.

## Test and invocation-history evidence

On a private temporary clone placed on a local branch at the exact
implementation parent,
the registrar/protocol suite passed 47/47 tests.
The temporary clone was then removed.

For transparency,
running that full suite directly at the later protocol commit produced
37 passes,
5 failures,
and 5 errors.
All ten non-passes were the expected success-fixture precedence effect:
those fixtures bind current `HEAD`,
and the registrar correctly rejects a parent that already contains the
generated protocol as circular.
The pure schema/contract tests and the dedicated circular-parent rejection
passed in that context.
The applicable implementation-parent run is the 47/47 result above.

The implementation closure records that no production protocol,
authorization record,
smoke root,
registered root,
or full-grid result existed before the implementation commit.
Git history introduces both protocol files together in exactly one commit,
their committed bytes match one canonical in-memory construction,
all six guarded roots remain absent,
and the external authorization record remains absent.
These independently observable facts are consistent with the reported
single registrar publication and with no smoke or registered execution.

Filesystem and Git state cannot independently reconstruct all historical
process launches or shell invocations.
Accordingly,
this review does not claim remote process-history knowledge beyond the
reported invocation record and the artifact/root evidence above.
It also does not infer scientific success from absent roots.

## Approval boundary

Task 8 may execute only the exact committed smoke arrays,
in the planned order and without retry or substitution.
Each analyzer smoke remains conditional on the successful terminal evidence
of its corresponding replay smoke.

The registered replay and registered analyzer remain separately
unauthorized while the authorization record is absent and the protocol
state remains `pending_external_record`.
Any source,
comparator,
protocol,
command,
root,
authorization,
disk,
schema,
gate,
or cardinality drift closes this preflight.
