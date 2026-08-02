# Qualified-closure development-v4 terminal evidence review

Date: 2026-08-02 (Asia/Shanghai)

Worktree: `/private/tmp/cbf2026-diagnostic`

## Decision

Critical: `0`; Important: `0`; Minor: `0`.

The author report is an accurate fail-closed archive:

```text
FAIL — INITIAL THEOREM-CERTIFICATE RESET INFEASIBLE
0/10 missions completed
development-v4 consumed; no retry
confirmatory and paper-evidence gates closed
```

This decision distinguishes two different terminal facts. The analysis bundle
is operationally `completed/terminal`, meaning the analyzer successfully
published an independently inspectable result. Its scientific verdict is
nevertheless `passed=false`, so the analyzer's nonzero result is expected and
does not turn the failed campaign into positive evidence.

The reviewed author report is exactly 7,381 bytes / 150 lines with SHA-256:

```text
b85095899b7c41741c08e6d9eb5deff7e2385c4798c5b6d5f598080f9664f28c
```

## Registration and source identity

Independent hashing reproduces the protocol JSON, protocol Markdown,
preflight, and authorization identities as `edfad6cb...` (42,237 bytes),
`a5b6450e...` (312 bytes), `5cc7b245...` (10,163 bytes), and `0089f427...`
(565 bytes), respectively. Removing only `semantic_sha256` and canonically
serializing the protocol reproduces
`9cf24ba0455163d8b08877f98ff211eb4729f5dd8f37570ebc1631d7c7580647`.
The authorization's literal UTF-8 user text is `批准`; its independently
computed SHA-256 is
`8cbe697b157364a5b13646285b38409dc53ec5287deeb7913493e65b275cd14d`.
Its protocol, preflight, kind/version, and implementation-parent bindings all
match.

Git independently resolves the execution child to
`1478f68a9576fb55a4bca65c607a7eaf76ff7a6d`, tree
`254cb41f878672c88b18ef52485df8e6d5854be0`, with sole parent
`14a27b7cedb704aeaa49945bc21dab1df04dfef4`. The parent-to-child diff contains
exactly four add-only paths: the v4 protocol JSON/Markdown, preflight, and
authorization. There is no modification, deletion, rename, merge parent, or
fifth path. The branch is `codex/cbf2026-diagnostic` with no upstream.

All 14 distinct registered `path/bytes/SHA-256` bindings were re-read. The 12
tracked paths match both their live bytes and exact blobs at `14a27b7...`; the
two allowed-untracked identities match live regular files. In particular,
`Swarm` is executable and hashes to `02f1c305...` at 1,616,232 bytes, the
three configurations reproduce `0920ca21...`, `6a198e17...`, and
`6ad20e1e...`, `ENABLE_GUROBI:BOOL=ON`, and primary/ablation select
`dynamic-lower-index`/`fixed-cbf-only`. The raw `identities.json` repeats
these execution bindings and the same authorization, protocol, and
implementation identities.

## Raw terminal bundle

The raw schedule is exactly the ten ordered `development-v4` missions, with
trajectory seeds `2026080101..2026080110`, range-noise seeds
`2026081101..2026081110`, 1,000 frames, 500 s, and conditions
`dynamic_primary` then `fixed_fim_ablation`. Canonical reconstruction of the
trusted mission list gives 2,041 bytes and SHA-256 `8b491445...`; the published
schedule file is 2,132 bytes and hashes to `1459f92b...`. Every mission
manifest equals its corresponding registered schedule row.

The raw root contains exactly 28 regular files, no symbolic member, and
12,618,979 logical bytes. I independently hashed every file, checked all ten
top-level mission-manifest identities, and checked every mission member
identity. With sorted rows
`relative-path<TAB>bytes<TAB>sha256<LF>`, the complete tree commitment is:

```text
9b915ad84aeda2b22aafbd259bf61bbc9515e3b082559bf3e5f217e14af9b5ac
```

The root manifest independently hashes to `12f71bb0...` and is terminal
`failed/schema_error`, with ten declared missions and zero completed. All ten
mission manifests are likewise terminal `failed/schema_error`.

Mission-01's ordered Swarm stream has exactly 14 valid initialization rows:
frame 0, robots 1 through 14 in order, the registered campaign and seeds, and
no estimator or controller row. Its supervisor is terminal
`failed/schema_error`, return code 1, `valid_rows=14`, and `missing_rows=0`.
The 125-byte stderr is exactly one newline-terminated message:

```text
[SIMULATION_ERROR] theorem certificate reset rejected: post-reset bounded local hard QP is infeasible for UAV 3 (infeasible)
```

No later mission has a Swarm stream. Each of the ten synthetic-complement
streams contains exactly 379,987 rows: 14 missing initialization, 27,972
missing estimator, 1,000 missing controller, 232,000 missing endpoint,
119,000 missing reconstructed, and one missing mission. Their schedule keys,
first/last keys, conditions, seeds, and `schema_error` reason match their
mission. Across ten missions this exactly reconstructs the registered
universes: 140 initialization, 279,720 estimator total (139,860 per
condition), 10,000 controller, 2,320,000 endpoint, 1,190,000 reconstructed,
and ten mission outcomes.

## Analysis and gate arithmetic

The analysis root contains exactly three regular files, no symbolic member,
and 8,639 logical bytes. Their hashes are `831bf600...` (`analysis.json`),
`85ee3916...` (`analysis.md`), and `89017490...` (`manifest.json`). The same
sorted per-file method gives tree commitment:

```text
155b6b9372f91bb7b4b2476581b4270e7a372c680cd8b2ea1847ef9c5766729a
```

The manifest is terminal `completed/completed`. Recomputing the analysis
semantic identity while excluding exactly `semantic_analysis_sha256` and the
raw-manifest byte identity reproduces
`623f6e4d21f0463bdcf4aa89a53a4bbe0d324b2b996fb42e7e0e375cd5edff33`.
The analysis binds the exact raw-manifest, protocol, authorization, and
implementation identities and includes development-v4 in its scientific
denominator.

The synthetic universes and the absence of estimator/controller records
independently reproduce the decisive arithmetic: successful missions `0/10`,
complete keys/manifests `0/1`, initialization omissions `140/140`, available
estimator tuples `0/139860`, jointly available-and-contained tuples
`0/139860`, controller-certificate intervals `0/10000`, and missing/duplicate
endpoint rows `2320000/2320000`. Both primary and fixed-FIM error sample counts
are zero. These are all FAIL gates; the full gate conjunction is therefore
false.

The zero-count safety/error fields are not evidence of safety or estimator
quality. There is no localization-error sample, no estimator output, no
controller interval, no CBF residual sample, and no ablation estimate. Thus
the report correctly refuses to promote zero true-distance/input/residual/50 m
counts into localization or safety claims. The identical-measurement
declaration is an integrity check only; execution stopped before either
estimator condition produced a row.

## Disk, no-retry, and scientific boundary

The registered protocol has `no_retry=true`, an 8,000,000,000-byte launch
floor, 6,000,000,000-byte hard stop, and 2,000,000,000-byte cache cap. The
preflight recorded 15,820,259,328 free bytes; the current audit still found
about 15 GB free. The two terminal roots occupy far below 2 GB, and their
current hashes and tree commitments match the values above. Both roots are
now consumed and policy-immutable: this review did not modify, delete, resume,
reuse, or rerun either one.

All six confirmatory smoke/raw/analysis roots remain absent. Development-v4
therefore supplies a precise initialization/reset failure witness and a valid
fail-closed archive, but no empirical localization, controller, CBF, safety,
mission-performance, or ablation evidence. Confirmatory execution, paper
numerical changes, new performance claims, and submission remain closed. A
new version would require a separately reviewed diagnosis and new lifecycle;
v4 itself must not be retried or reinterpreted.

## Review method

The review used read-only Git plumbing, canonical JSON reserialization,
independent SHA-256 and byte counts, complete regular-file enumeration,
manifest-to-live identity comparison, gzip streaming counts and key checks,
and direct gate arithmetic. It did not invoke the registrar, runner, analyzer,
Swarm, replay, measurement generator, or any campaign command. No execution
root or source path was changed.
