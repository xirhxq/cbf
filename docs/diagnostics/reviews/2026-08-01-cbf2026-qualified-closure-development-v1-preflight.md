# Qualified Closure Development v1 Independent Preflight

Review date: 2026-08-02

Verdict: FAILED — NOT READY

Authorization artifact: deliberately not published. The required path
`docs/diagnostics/reviews/2026-08-01-cbf2026-qualified-closure-development-v1-authorization.json`
remains absent because the exact authorization validator cannot succeed in any
state permitted by Task 10 Steps 8–9.

The researcher's standing development-execution decision is recorded as
`批准`, but it is conditional on a successful independent preflight and does
not override a fail-closed gate. It authorizes at most one development v1
execution. It does not authorize root reuse, retry, threshold changes,
configuration changes, code changes, protocol replacement, or confirmatory
execution. Because this preflight failed, none of those permissions became
effective and no campaign was started.

## Scope and independently observed identity

- Worktree: `/private/tmp/cbf2026-diagnostic`
- Registered/current implementation HEAD before publication commit:
  `709f4ef6cef60b0754527fc62d8ca64ac7a88a12`
- Registered/current tree before publication commit:
  `8a92bba68977792d7e97ece996bed2fcf61e859e`
- Protocol JSON: 41,979 bytes,
  SHA-256 `9a5e9d2503d083bdbb239468a88cf5ceed2cc283978e8beed5badc020ae74343`
- Protocol Markdown: 312 bytes,
  SHA-256 `19cf657d787bfe76bfa2aecf35ce06baf9fd2f57ff2deeab12e894429ccebffc`
- Embedded semantic SHA-256:
  `9bb640b5a4a9fa31849460e1cdd996b6eebc4868d5f1537ab4d01769fe2b7214`

The JSON is the exact canonical serialization plus one newline. Independently
reconstructing the companion Markdown from the JSON identity produced the
existing bytes exactly. Recomputing the semantic digest after removing only
`semantic_sha256` produced the embedded value.

## Identity recomputation

Every declared regular-file byte count and SHA-256 matched the current file;
none of the declared files was a symlink.

| Declaration | Bytes | Recomputed SHA-256 |
|---|---:|---|
| implementation report | 24,053 | `abc52265b84636f3eec2ea859f360872cd95f28fa77486538ee6003a924d5be8` |
| implementation review | 24,536 | `340aa0ffa4b65113730c20c2918230d8006b52feacbc1a42c169b20ea81cd99f` |
| `build-diagnostic/Swarm` | 1,616,232 | `02f1c30575a04194fe2d459469a36ba0e099c72d2c4979dc29bf2627fef6b63f` |
| `build-diagnostic/CMakeCache.txt` | 19,242 | `034eab2688f36fa330838513ab597e6ef83283c85cda6310b2aa639a24c9a264` |
| base config | 4,783 | `0920ca21fc57e51d44b0df3c00bc32bf4d9fe5bbc7f6fa8bb826def1b88184ac` |
| primary config | 1,330 | `6a198e17b8c3bbf6777caaa325fa593dd24ee9ce3981f90769075112be1cacb3` |
| ablation config | 1,325 | `6ad20e1e32fff5e8950f8de31864d77a238f6d418abafe3a50cdb292b8c0bc66` |
| evidence schema | 108,247 | `c1e2965c61ff71b7384bbf9ee02e80d41c842faa77734e66dc444be3a02803c3` |
| campaign runner | 94,660 | `c3cf23ddf761868688f8aba6f6a3276bcbe4b10e9e1bce21469f956191f1fa37` |
| campaign analyzer | 103,545 | `7ced5b02cc55eaaa6bd588049501c3749e02a696b829163cac5872348bf23a47` |
| estimator analyzer | 57,353 | `e0eeb0bd2d54ac49160aa862734a0b188c1da7587d005e7357e48a1c69b3b60f` |
| measurement generator | 27,558 | `2b354a7a629baa4a8f937365079764bfeb0d8667f8c675a5b2814721ff26ae9b` |
| registrar | 48,456 | `5790b421c328f8deff367396f3c043b6cb17b9e83a77e248dee989471226eacf` |
| estimator replay | 49,283 | `4a47920e715c274c4e41a5813334f2de5dd36cfac45a0e462d261356646b4aea` |

`Swarm` is an executable regular file. The live `otool -L` payload was 621
bytes with SHA-256
`1cdf696769ab1ba14df66af7d6c47a9a0ed8ffda1356f52e59389f0124832492`.
The live `conda list --explicit` payload in `cbf_env` was 7,040 bytes with
SHA-256
`361248f9642ee3c46023f5d5a7253e6c03c7a8cde71eae525bd71d4465aeb3c0`.
The CMake cache contained the exact line `ENABLE_GUROBI:BOOL=ON`.

## Scientific and operational contract checks

- The protocol has exactly ten missions, trajectory seeds
  `2026080101`–`2026080110`, range-noise seeds
  `2026081101`–`2026081110`, and 1,000 frames per mission.
- The serialized runner and analyzer token arrays match Task 11 Steps 2–3
  token-for-token, including the protocol, authorization, binary, three
  configuration paths, seed ranges, frame count, and distinct raw/analysis
  roots.
- The frozen thresholds match exactly: 8,000,000,000-byte launch floor,
  6,000,000,000-byte stop floor, 2,000,000,000-byte cache cap, and
  25,000,000-byte compact-output cap, together with every scientific
  tolerance and rate gate.
- Supervision is exactly 3,600 s wallclock, 300 s line stall, and 5 s
  termination grace; `no_retry` is exactly `true`.
- Both registered development roots were absent under both existence and
  lexical-symlink checks. The historical two-range analysis v2 root was also
  absent.
- `/private/tmp` had 17,128,165,376 free bytes at the independent check,
  exceeding the 8 GB launch floor.
- The historical two-range development v2 root contained exactly
  `manifest.json` and `two-range-reacquisition.jsonl.gz`; its independently
  recomputed sorted per-file tree commitment remained
  `57670b00a303c4c655559097d2a80589c4cea663bc88d3033dfe30fb5fbea6cd`.

These checks establish that the protocol payload is internally correct and
its scientific inputs are unchanged. They do not cure the authorization
lifecycle failure below.

## Findings

### Critical C1 — protocol publication and required authorization commit make the repository gate unsatisfiable

The protocol froze `dirty_relevant_paths: []` and HEAD
`709f4ef6cef60b0754527fc62d8ca64ac7a88a12`. The registrar's live repository
probe permits untracked files only below `build-diagnostic/`; every other
untracked path is relevant. Immediately after the registrar atomically
published its own declared JSON/Markdown pair, the live repository identity
therefore gained exactly these two relevant paths:

- `docs/diagnostics/2026-08-01-cbf2026-qualified-closure-development-v1-protocol.json`
- `docs/diagnostics/2026-08-01-cbf2026-qualified-closure-development-v1-protocol.md`

The registered HEAD and tree still matched, and the protocol pair and derived
contract passed, but `verify_registered_protocol()` failed with:

```text
ValueError: registered repository identity or relevant Git status mutated
```

This was reproduced through the public authorization gate with a temporary,
non-repository candidate containing exactly the required six fields,
`authorized: true`, the protocol SHA-256 above, and implementation identity
`709f4ef6cef60b0754527fc62d8ca64ac7a88a12`. The candidate was removed after
the check. `validate_authorization_binding()` reached
`verify_registered_protocol()` and failed with the same exception. No
authorization file was published in the worktree.

Committing the artifacts cannot repair the failure. Task 10 Step 9 explicitly
requires a new commit containing the protocol pair, preflight, and
authorization. Any such commit advances current HEAD beyond the registered
implementation HEAD. `verify_registered_protocol()` first requires
`observed["head"] == repository["head"]` and raises `registered implementation
is not the exact current HEAD` otherwise. The focused test
`test_verifier_rejects_post_registration_head_even_when_original_is_ancestor`
explicitly enforces that rejection. Conversely, before that commit the
protocol/preflight/authorization files are relevant untracked paths. There is
therefore no pre-commit or post-commit state satisfying the gate.

The production runner calls `validate_authorization_binding()` before campaign
execution, and the production analyzer calls the same validator before reading
the bundle. This is an execution-blocking lifecycle deadlock, not a reporting
defect.

Minimal recovery requires a new red/green lifecycle contract that separates
the immutable implementation identity from the later administrative artifact
commit. A safe repair should:

1. keep the authorization bound to the registered implementation identity and
   exact protocol bytes;
2. permit only the explicitly declared protocol/preflight/authorization
   publication transition, both while those exact paths are untracked and
   after their administrative commit;
3. require the registered implementation commit to remain an ancestor and
   reject any intervening change outside an exact allowlist, while continuing
   to rehash every bound source, config, binary, dependency, schema, report,
   review, and tooling file;
4. add an unmocked temporary-Git integration test covering publication,
   candidate authorization, administrative commit, runner validation, and
   rejection of source/config/unlisted-path mutations.

The existing v1 protocol includes the registrar hash and cannot remain valid
after that code change. Preserve this failed protocol/preflight as evidence;
do not overwrite or reinterpret it. Generate a fresh non-colliding protocol
and root version only after the repair is independently reviewed.

## Severity and readiness

| Severity | Count | Disposition |
|---|---:|---|
| Critical | 1 | Open: C1 blocks authorization and execution |
| Important | 0 | None |
| Minor | 0 | None |

Readiness: NOT READY. Development v1 authorization is withheld; confirmatory
authorization remains closed. No runner, analyzer, campaign root, retry, or
commit was executed by this preflight.
