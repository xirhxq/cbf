# Qualified-closure development-v4 independent preflight

Date: 2026-08-02 (Asia/Shanghai)

Worktree: `/private/tmp/cbf2026-diagnostic`

Branch: `codex/cbf2026-diagnostic`

## Decision

Critical: `0`; Important: `0`; Minor: `0`.

Ready only to create the exact bound authorization and commit exactly the four
development-v4 lifecycle artifacts as one add-only direct child: **Yes**.

Execution authorized now: **No**.

This preflight creates no authorization and claims no execution root.
Development-v4 becomes execution-eligible only after the separate
authorization binds the actual user-origin approval, the exact protocol, the
registered parent, and this file's final SHA-256; all four artifacts must then
be the sole direct child of the registered parent and pass production
postcommit authorization validation.

## Protocol pair and semantic contract

Both publication files are regular, non-symlink files and are an exact
canonical pair:

```text
edfad6cbe68c1ac1459640f71e7c2664d6702cc519804ed6414ea7e8761526f8  42,237 bytes  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v4-protocol.json
a5b6450e657bd3332f67d03d8f7cb5bdfb47629bbceb93f2a2fac4d3c693e8c0     312 bytes  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v4-protocol.md
```

Independent canonical serialization, exact nested-schema checks, removal of
only `semantic_sha256`, and rehashing reproduce semantic SHA-256
`9cf24ba0455163d8b08877f98ff211eb4729f5dd8f37570ebc1631d7c7580647`.
The Markdown exactly renders that identity and the JSON byte hash above.
The protocol is exactly schema `cbf2026-qualified-closure-protocol-v1`, kind
`development`, version `v4`, conditions `dynamic_primary` then
`fixed_fim_ablation`, with `smoke_schedule=null` and `no_retry=true`.

## Registered parent and four-artifact topology

The registered and observed documentation-closure parent is:

```text
HEAD  14a27b7cedb704aeaa49945bc21dab1df04dfef4
tree  b67b21e40d72944c33f2973cada7ffaec7ab767a
base  edb9bd00ee00a98510dfe106bf3ed4ef9fede281
root  /private/tmp/cbf2026-diagnostic
```

The parent changes exactly the implementation report, operative recovery
plan, and independent implementation review over the six-file module-launch
repair. It contains none of the four future lifecycle paths and has no child
reachable from current refs. Tracked state is clean. Untracked state is
exactly the 512 frozen `build-diagnostic/` paths plus the protocol JSON and
Markdown; there is no other relevant untracked path.

The only permitted next commit has one parent equal to `14a27b7...` and adds
exactly these four paths, with no modification, deletion, rename, or extra
path:

```text
docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v4-protocol.json
docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v4-protocol.md
docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v4-preflight.md
docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v4-authorization.json
```

The protocol pair must not be committed separately. Amend, merge, later
descendant, dirty tracked state, relevant untracked state, or a live/HEAD byte
mismatch must fail closed.

## Source, document, configuration, and build bindings

Every registered tracked identity matches both live bytes and its exact Git
blob at `14a27b7...`. The independent comparisons include:

```text
80a68310cc798d517d5ee391a41dea274b03dafa68c86e506cb55aca9ad2fb19   43,086  implementation report
2cbb370bc6ace83d0e1e2cc6d2681707239d7534cc690172e4caeb845e013d95   49,030  implementation review
d04ae28198f7992543572d3fe54be54894d6d964d9bea66c80fe50be951996bf   94,650  campaign runner
4467f4f2b725adf883bd3fa5bf0ec47f715cc49d01c5c9f37f1fecf433e08f89  104,631  campaign analyzer
8aa059beeea209aef7a47ad4f538f4fb7bf3aec0087b2e3e0e8fac4efa9ed801   56,494  campaign registrar
c1e2965c61ff71b7384bbf9ee02e80d41c842faa77734e66dc444be3a02803c3  108,247  evidence schema
2b354a7a629baa4a8f937365079764bfeb0d8667f8c675a5b2814721ff26ae9b   27,558  measurement generator
4a47920e715c274c4e41a5813334f2de5dd36cfac45a0e462d261356646b4aea   49,283  estimator replay
e0eeb0bd2d54ac49160aa862734a0b188c1da7587d005e7357e48a1c69b3b60f   57,353  estimator analyzer
0920ca21fc57e51d44b0df3c00bc32bf4d9fe5bbc7f6fa8bb826def1b88184ac    4,783  base configuration
6a198e17b8c3bbf6777caaa325fa593dd24ee9ce3981f90769075112be1cacb3    1,330  dynamic-primary configuration
6ad20e1e32fff5e8950f8de31864d77a238f6d418abafe3a50cdb292b8c0bc66    1,325  fixed-FIM ablation configuration
```

The implementation report and review reproduce the operative plan identity
`f3cb2c0c9bc047e1873ca27c2ce56cd5ab5732ade0264c3ce8f90c80b7979fbb`
(178,976 bytes / 3,243 lines), and that plan is the matching parent blob.
Primary selects `dynamic-lower-index`; ablation selects `fixed-cbf-only`.

Both identity-bearing allowed-untracked files are regular exact members of
the 512-path allowlist:

```text
02f1c30575a04194fe2d459469a36ba0e099c72d2c4979dc29bf2627fef6b63f  1,616,232  build-diagnostic/Swarm
034eab2688f36fa330838513ab597e6ef83283c85cda6310b2aa639a24c9a264     19,242  build-diagnostic/CMakeCache.txt
```

`Swarm` is executable and the cache contains `ENABLE_GUROBI:BOOL=ON`.
Fresh command identities reproduce `otool -L` SHA-256
`1cdf696769ab1ba14df66af7d6c47a9a0ed8ffda1356f52e59389f0124832492`
(621 bytes) and `conda list --explicit` SHA-256
`361248f9642ee3c46023f5d5a7253e6c03c7a8cde71eae525bd71d4465aeb3c0`
(7,040 bytes).

## Schedule, universes, thresholds, argv, roots, and capacity

The trusted schedule is ten ordered missions with campaign ID
`development-v4`, trajectory seeds `2026080101..2026080110`, range-noise
seeds `2026081101..2026081110`, 1,000 frames, 500 s horizons, and the two
ordered conditions above. Independent reconstruction yields canonical
SHA-256 `8b491445758bc1d73175931cd32214f7605a149fc5ee7666ed5f140703e73213`
over 2,041 bytes and matches the runner and analyzer envelopes.

The frozen universes are 140 initialization rows, 139,860 estimator rows per
condition, 279,720 estimator rows total, 10,000 controller frames, 2,320,000
allocated endpoints, 1,190,000 reconstructed rows, and 10 missions.
All statistical, safety, rate/input/residual, and 50 m finite-error thresholds
are unchanged. Supervision remains 3,600 s wallclock, 300 s line stall, and
5 s termination grace. Disk gates remain 8 GB launch, 6 GB stop, 2 GB reusable
cache, and 25 MB per compact analysis bundle.

The registered commands exactly use repository-cwd module entry points:

```text
conda run -n cbf_env python -m scripts.diagnostics.run_qualified_closure_campaign --kind development --version v4 --protocol docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v4-protocol.json --authorization docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v4-authorization.json --binary build-diagnostic/Swarm --base-config config/config.json --primary-config config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json --trajectory-seeds 2026080101:2026080110 --range-noise-seeds 2026081101:2026081110 --frames 1000 --output-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v4
conda run -n cbf_env python -m scripts.diagnostics.analyze_qualified_closure_campaign --kind development --version v4 --protocol docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v4-protocol.json --authorization docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v4-authorization.json --input-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v4 --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json --output-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v4
```

No top-level executable token uses a `.py` entry point. Registrar, runner,
and analyzer reject development v1-v3; only development-v4 is current, while
confirmatory remains v1. Both v4 roots are absent, non-symbolic, outside the
repository, and have no symbolic ancestor. At audit time `/private/tmp` had
15,820,259,328 free bytes, above the 8,000,000,000-byte launch floor.

## Consumed-version preservation and authorization boundary

Development-v1 remains the immutable three-file failed-preflight commit
`0e4c439...`, with authorization/result/review and both roots absent.
Development-v2 remains the exact four-artifact child `ae7f68c...`; its failed
postcommit-validation archive is byte-identical at SHA-256
`815988e3149d6adcffb61d323e242905ae3e89a145eb37560abf1b47de9fb235`,
and its result/review and both roots are absent. Development-v3 remains the
exact four-artifact child `16c0aca...`; its launch-failure archive is
byte-identical at SHA-256
`54f78e86c4b1dc9eff1a031a8ed2121caff305afc1892eddc28212fc492a2818`,
and its result/review and both roots are absent. No historical artifact or
root changed. Versions v1-v3 are consumed, immutable, and non-executable.

The DRA worktree is clean on main at
`764f6600696c642bb78590e62ae9368e7330ed2c`, recording the v3 failure and v4
recovery without modifying submission records.

An independent 142-check recomputation covers canonical pairing, exact schema,
semantic hash, schedules, universes, thresholds, all live and parent-blob
identities, allowed-untracked state, build/dependency identities, argv,
topology, roots, capacity, historical preservation, and DRA state. It returns
C0/I0/M0. No registrar, runner, analyzer, Swarm, authorization, campaign, or
execution-root command ran during this preflight.

The later authorization must use the exact ten-field production schema; bind
protocol SHA-256
`edfad6cbe68c1ac1459640f71e7c2664d6702cc519804ed6414ea7e8761526f8`,
implementation identity `14a27b7cedb704aeaa49945bc21dab1df04dfef4`,
this file's final SHA-256, kind `development`, version `v4`, canonical date,
and the actual user-origin text plus its UTF-8 SHA-256. Creating that JSON
alone still does not authorize execution; the exact committed four-artifact
direct-child state must pass the production gate first.
