# Qualified-closure development-v5 independent preflight

Date: 2026-08-02 (Asia/Shanghai)

Worktree: `/private/tmp/cbf2026-diagnostic`

Branch: `codex/cbf2026-diagnostic`

## Decision

Critical: `0`; Important: `0`; Minor: `0`.

Ready only to create the exact actual-user-text authorization and commit
exactly the four development-v5 lifecycle artifacts as one add-only direct
child: **Yes**.

Execution authorized now: **No**.

This preflight creates no authorization, supplies no user-origin approval,
and claims no execution root. Development-v5 can become execution-eligible
only after a separate authorization binds the actual user text, the exact
protocol, the registered parent, and this file's final SHA-256; all four
artifacts must then be the sole direct child of the registered parent and pass
production postcommit authorization validation.

## Protocol pair and semantic contract

Both publication files are regular non-symlink files and form the exact
canonical production pair:

```text
a821e6d2abbc9a85a088892fb33413387e8233b6d67646ab346e64d5698f8123  47,677 bytes  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v5-protocol.json
91d7cc6a01c53b7ac2627b9318dd75d7328eb401cf70c25869916715823269cf     312 bytes  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v5-protocol.md
```

Strict duplicate-key rejection, finite strict JSON parsing, exact top-level
and nested schema-v2 validation, canonical single-line serialization, removal
of only `semantic_sha256`, and independent rehashing reproduce semantic
SHA-256
`95a479b19dcf979f3df683d5dbc865cf327c9fd60cd88d2a112ea6bff1e94b81`.
The Markdown is the registrar's exact rendering of that semantic identity and
the JSON byte hash above. The protocol is exactly schema
`cbf2026-qualified-closure-protocol-v2`, kind `development`, version `v5`,
conditions `dynamic_primary` then `fixed_fim_ablation`, with
`smoke_schedule=null` and `no_retry=true`.

## Registered parent and exact-four topology

The registered and observed documentation-closure parent is:

```text
HEAD    55a9a783a94a1d59a60db99552a6a4421acca25b
parent  6ac6e088ee5d2c6dbed9ed24c128a3225252de6f
tree    1e7dacbeadf96fd00353e198daed4110ef7304e9
root    /private/tmp/cbf2026-diagnostic
```

The parent is a single-parent commit that changes exactly the implementation
report and independent implementation review over the six-file v5 integration
commit. It contains none of the four future lifecycle paths and has no child
reachable from current refs. Tracked state is clean. Before this preflight was
created, untracked state was exactly the 512 frozen `build-diagnostic/` paths
plus the protocol JSON and Markdown; no other relevant untracked path existed.

The only permitted next commit has one parent equal to `55a9a78...` and adds
exactly these four paths, with no modification, deletion, rename, merge, or
extra path:

```text
A  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v5-protocol.json
A  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v5-protocol.md
A  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v5-preflight.md
A  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v5-authorization.json
```

The protocol pair and preflight must not be committed separately. An amend,
merge, later descendant, dirty tracked path, relevant untracked path, or
live/HEAD byte mismatch must fail closed.

## Source, review, configuration, tooling, and build identities

All 15 registered tracked identities match both their live bytes and their
exact Git blobs at `55a9a78...`; both identity-bearing allowed-untracked files
match their registered live bytes. The complete comparison is:

```text
c0300d8a60e08f1af8c4ee7b0f912a54a2bd51e575a6dcb68ee915d881284706   49,900  implementation report
020b7a9598f53f1abb197c1d298c0379492d2ec9affc0a44902feacdcaba408a   54,689  implementation review
8b6496aac6f2a8000ad04cd5281dad6c986df4a2d5523053a8f77328bf4fa8e8  109,484  campaign runner source
04666b242ce3a5399f4d03f92ae723597e89982ca0302b8d7285caea88500f4f  115,933  evidence schema
0920ca21fc57e51d44b0df3c00bc32bf4d9fe5bbc7f6fa8bb826def1b88184ac    4,783  base configuration
6a198e17b8c3bbf6777caaa325fa593dd24ee9ce3981f90769075112be1cacb3    1,330  dynamic-primary configuration
6ad20e1e32fff5e8950f8de31864d77a238f6d418abafe3a50cdb292b8c0bc66    1,325  fixed-FIM ablation configuration
7fb8597cd89a41315cbd86ca87c0bc6e3ecb22ea3dcab8ee6278aca44743fe24    3,703  frozen initial family
2acb118223fdb226c00c104e9f0a6fe1a469b17be512412eb5e9e2c7266b304f  115,411  campaign analyzer
e0eeb0bd2d54ac49160aa862734a0b188c1da7587d005e7357e48a1c69b3b60f   57,353  estimator analyzer
2b354a7a629baa4a8f937365079764bfeb0d8667f8c675a5b2814721ff26ae9b   27,558  measurement generator
678df2a373e729844b1e6be5d21959e9c04d5ccf509c2c8d6041e94c9b5bc303   41,088  initial-state auditor
dd12bb8ad34ebf0f849194a73e25aaaecc980f08cf368cb355a24a1793665d74   67,525  campaign registrar
4a47920e715c274c4e41a5813334f2de5dd36cfac45a0e462d261356646b4aea   49,283  estimator replay
8b6496aac6f2a8000ad04cd5281dad6c986df4a2d5523053a8f77328bf4fa8e8  109,484  campaign runner tooling
02f1c30575a04194fe2d459469a36ba0e099c72d2c4979dc29bf2627fef6b63f 1,616,232  build-diagnostic/Swarm
034eab2688f36fa330838513ab597e6ef83283c85cda6310b2aa639a24c9a264    19,242  build-diagnostic/CMakeCache.txt
```

`Swarm` is executable and the CMake cache contains
`ENABLE_GUROBI:BOOL=ON`. Fresh commands reproduce the registered `otool -L`
identity `1cdf696769ab1ba14df66af7d6c47a9a0ed8ffda1356f52e59389f0124832492`
(621 bytes) and `conda list --explicit` identity
`361248f9642ee3c46023f5d5a7253e6c03c7a8cde71eae525bd71d4465aeb3c0`
(7,040 bytes).

## Frozen initial-family and admission contract

The registered family is exactly the literal regular non-symbolic path
`config/diagnostics/qualified_initial_family_v1.json`; neither the leaf nor
any ancestor is symbolic. Its file SHA-256 is `7fb8597c...` as above and its
independently recomputed semantic SHA-256 is
`4f8070127c86149de8ebde10197e5c7a639814e03dac7d56ab17befc6d5bdb8b`.
Strict duplicate-safe loading and complete production-formula recomputation
admit all 10/10 registered seeds and all 100/100 audit seeds, with no clamp,
resampling, or rejected seed.

The protocol's `initial_state` has exactly these nine fields:

```text
admission
audit_trajectory_seeds
family_schema_version
family_semantic_sha256
frozen_summary
missions
namespace
perturbation_policy
registered_trajectory_seeds
```

Its namespace is `cbf2026-v5-initial`; its registered seeds are exactly
`2026080201..2026080210`, its audit universe is exactly
`2026080201..2026080300`, and its perturbation policy is
`clamp=false, resample=false`. Independent reconstruction matches the entire
nine-field declaration and these ten per-mission position identities:

```text
2026080201  161fb8a9104b7b5b0a3a20cd5cf0e9c896db98e74ee2262088751edddaa72e88
2026080202  387a836180aa8790ea18b4d84be1a4fa24196242f8843df28bf54ed878e00cde
2026080203  d3083a94c5996019602ef387d94f6443671d052a8468331e8210c0cfbbdc7b1b
2026080204  55ef0edc2cef176a8473af58e547b719bc482bf84c421270be9fee2f76b29b2d
2026080205  a82cd0c8b45d978c9a7735718c4b3db1e6a2f23afb216a43cf61ab8b6e5c143e
2026080206  00131761a0d985ec212b6ac28d9844d163da33653ab86a8cdeb0480ea3089d2b
2026080207  781c60a106c8a8cadaf12aa0dde68408844f8c1efc3465ab7b2d0209ae0af745
2026080208  e7ad816b6be74a614681dfb588dcd9a3c51e0679892d65b746099df36e6534b5
2026080209  1280dd3e0e62d9e5edc733c639da541f89062e72771ee345f81bee5b47d60597
2026080210  84b14e41c336be7ccd09f4d9d1e55f4daaf309abc6079bd5d712655789f8db55
```

The full audit minimum barrier is `35.77296640879953 m` at seed
`2026080205`, the minimum local max-min QP margin is
`0.7658252531927233 m/s` at seed `2026080207`, the maximum uncertainty-rate
bound is `6.7773637849535655 m/s`, and the minimum pair distance is
`48.51563216720877 m`. These are admission/precondition checks only. They are
not mission-wide localization, safety, containment, availability, ablation,
or performance evidence.

## Schedule, universes, thresholds, and argv

The trusted schedule is ten ordered `development-v5` missions with trajectory
seeds `2026080201..2026080210`, range-noise seeds
`2026081201..2026081210`, 1,000 frames, 500 s horizons, and ordered conditions
`dynamic_primary`, `fixed_fim_ablation`. Its independently reconstructed
canonical identity is SHA-256
`20ed1dd4d91fa9ea3d5028bae490d418c7cb9bc6da622d35707da3429876fe69`
over 2,981 bytes and matches both runner and analyzer envelopes. Each protocol
mission binds its exact registered-seed position SHA-256.

The frozen universes are 140 initialization rows, 139,860 estimator rows per
condition, 279,720 estimator rows total, 10,000 controller frames, 2,320,000
allocated endpoints, 1,190,000 reconstructed rows, and 10 missions. All
thresholds match the production frozen dictionary: containment 0.98,
minimum-depth containment 0.95, joint available-and-contained 0.93, fresh
retention 0.98, availability 0.95, controller-certificate availability 0.99,
mission success 0.95, maximum finite error 50 m, rate tolerance `1e-9`, input
tolerance `1e-7`, and residual tolerance `-1e-7`. Supervision is 3,600 s
wallclock, 300 s complete-line stall, and 5 s termination grace.

The registered commands are exactly:

```text
conda run -n cbf_env python -m scripts.diagnostics.run_qualified_closure_campaign --kind development --version v5 --protocol docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v5-protocol.json --authorization docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v5-authorization.json --binary build-diagnostic/Swarm --base-config config/config.json --primary-config config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json --initial-family config/diagnostics/qualified_initial_family_v1.json --trajectory-seeds 2026080201:2026080210 --range-noise-seeds 2026081201:2026081210 --frames 1000 --output-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v5
conda run -n cbf_env python -m scripts.diagnostics.analyze_qualified_closure_campaign --kind development --version v5 --protocol docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v5-protocol.json --authorization docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v5-authorization.json --input-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v5 --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json --output-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v5
```

The runner contains exactly one `--initial-family` binding; the analyzer
contains none. Both use repository-cwd module entry points rather than `.py`
top-level tokens.

## Historical preservation, roots, capacity, and DRA state

Development v1--v3 raw and analysis roots remain absent, non-symbolic, and
without symbolic ancestors. Development-v4 remains exact immutable terminal
evidence:

```text
raw       28 files  12,618,979 bytes  tree 9b915ad84aeda2b22aafbd259bf61bbc9515e3b082559bf3e5f217e14af9b5ac  manifest 12f71bb00720636ca610ad8d9d380dde7e92147e7bdf57589bb8915c1bf72f71
analysis   3 files       8,639 bytes  tree 155b6b9372f91bb7b4b2476581b4270e7a372c680cd8b2ea1847ef9c5766729a  manifest 890174908e5df83e367edd33cb0d83960635cc94ac57fd79fafd553aef40e6f0
```

The prospective development-v5 roots are absent, non-symbolic, outside the
repository, and have no symbolic ancestor:

```text
/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v5
/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v5
```

Disk gates remain 8,000,000,000 bytes at launch, 6,000,000,000 bytes at hard
stop, 2,000,000,000 bytes for reusable cache, and 25,000,000 bytes per compact
analysis bundle. At audit time `/private/tmp` had 15,653,421,056 free bytes,
above the launch floor. No root was claimed or created.

The DRA worktree is clean on `main` at
`2c69a0f8be2852c344e462e0a097948817903f5a`, tree
`ed3b1a20cf189b81265904109e834cca426ba585`, with parent
`85563db33a181c50c0289e252298583fca63270a`. It records the v5 lifecycle
integration across the five expected DRA status/meta paths, tracks
`origin/main`, and is 53 commits ahead / 0 behind. This preflight did not
modify or push DRA.

## Fresh verification and authorization boundary

The fresh combined suite
`qualified_initial_state + qualified_closure_evidence + runner + analyzer + registrar`
passed 206/206 in `84.648 s`; its real complete 100-seed audit took
`11.563 s`. The four production modules passed `py_compile`, repository and
staged `git diff --check` passed, and the protocol pair has no trailing
whitespace. Production exact-schema, derived-contract, canonical-pair, v4
predecessor, and registered-argv validators also passed.

No authorization exists. This file and the protocol pair are not
authorization, and no registrar, runner, analyzer, Swarm, measurement
producer, replay, campaign, output-root, paper, submission, or push command
ran during this preflight.

The separately created authorization must use the exact ten-field production
schema and bind protocol SHA-256
`a821e6d2abbc9a85a088892fb33413387e8233b6d67646ab346e64d5698f8123`,
implementation identity `55a9a783a94a1d59a60db99552a6a4421acca25b`,
this preflight's final SHA-256, kind `development`, version `v5`, a canonical
ISO date, and the actual user-origin authorization text plus its UTF-8
SHA-256. Test fixtures, historical text, inferred approval, and this
preflight's verdict are not user authorization. Creating that JSON alone
still does not authorize execution: the sole exact-four add-only direct child
must first pass production postcommit authorization validation.
