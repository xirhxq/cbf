# Qualified-closure development-v3 independent preflight

Date: 2026-08-02 (Asia/Shanghai)

Worktree: `/private/tmp/cbf2026-diagnostic`

Branch: `codex/cbf2026-diagnostic`

## Decision

Critical: `0`; Important: `0`; Minor: `0`.

Ready only to create the exact bound authorization and commit exactly the four
development-v3 lifecycle artifacts as one add-only direct child: **Yes**.

Execution authorized now: **No**.

This preflight neither creates nor supplies user authorization. The separate
authorization must bind the actual user-origin approval, the exact protocol,
the registered implementation/documentation parent, and this file's final
SHA-256. Development-v3 can become execution-eligible only after all four
artifacts are committed as the sole direct child of the registered parent and
the production postcommit authorization validator independently passes.

## Reviewed protocol pair

The exact registered publication files are regular non-symlink files:

```text
8359bc7beffb676208a50f770a43817dbd71fc551dd8687790caecaef49f7c2b  42,233 bytes  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.json
21f59658bfa7ad051f936e497ff45e583fefa2fa75b0e166db7127ca3d62cab2     312 bytes  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.md
```

The production canonical-pair, exact-schema, semantic-rehash,
frozen-derived-contract, and registered-argv checks passed. The Markdown is
the registrar's exact rendering of the JSON identity. The protocol semantic
SHA-256 is
`54160bf022e983d385862ad15e831c1f5d4e65460be666ee8cf0de0f81b60fac`.
It is exactly kind `development`, version `v3`, conditions
`dynamic_primary` and `fixed_fim_ablation`, and `no_retry=true`.

## Registered parent and four-artifact topology

The registered and observed documentation-closure parent is:

```text
HEAD  70affe063e358a22c2139984429e741ff5a03023
tree  ad0989d9e6483f757b25358e9c9ba2161138af72
root  /private/tmp/cbf2026-diagnostic
```

Its sole parent is code-fix commit
`64f64d00a1fd6b7ab01760e340eca883be58f338`. The parent changes exactly the
final implementation report, recovery plan, and independent implementation
review over that code fix. The protocol correctly binds the future-facing
documentation closure `70affe0...`, not `64f64d0...` directly.

Tracked state is clean. At preflight start, untracked state consisted exactly
of the protocol pair plus the 512 `build-diagnostic/` paths frozen in
`repository.allowed_untracked_paths`; no other relevant path existed. The
parent tree contains none of the four future lifecycle paths. The only
permitted next commit must have exactly one parent equal to `70affe0...` and
exactly these four add-only entries, with no other path:

```text
A  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.json
A  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.md
A  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-preflight.md
A  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-authorization.json
```

The protocol pair must not be committed separately. An amend, merge, second
descendant, extra path, modification, deletion, rename, dirty tracked path,
relevant untracked path, or live/HEAD byte mismatch must fail closed. The
production full repository-state check is intentionally deferred until the
four-file commit exists; the uncommitted protocol/preflight state cannot and
does not authorize execution.

## Implementation, review, code, configuration, and build bindings

All 13 registered tracked identities matched both their live bytes and exact
Git blobs at `70affe0...`. The key document and production identities are:

```text
1858044715147522398c7a43b42418a6e5ce7ac29a00b6ec081382e5e5f67672   37,754  implementation report
433479c5a6c72d3cce77849aefde746c32786411d10875cb1605a915c276d61a   43,291  implementation review
ffcf158ec6ed89fd313d8ae6479e1339f23b570fda502a637239c2bc4864feb1  163,543  recovery plan
ce671d089f736bd01c9adabfbef965f82fd7e4ba2cd46b49cb04873730b946fe   94,647  campaign runner
7d724212e95c082c2d0a106d14ea9945f25a3e153c6feef5c4a4750b9174b6ea  104,628  campaign analyzer
e94057319898d56c788d203add27c7a8ebc3883001eac9e324c831acf73d22dd   56,482  campaign registrar
c1e2965c61ff71b7384bbf9ee02e80d41c842faa77734e66dc444be3a02803c3  108,247  evidence schema
2b354a7a629baa4a8f937365079764bfeb0d8667f8c675a5b2814721ff26ae9b   27,558  measurement generator
4a47920e715c274c4e41a5813334f2de5dd36cfac45a0e462d261356646b4aea   49,283  estimator replay
e0eeb0bd2d54ac49160aa862734a0b188c1da7587d005e7357e48a1c69b3b60f   57,353  estimator analyzer
```

The report embeds the exact recovery-plan hash. Both identity-bearing
allowed-untracked files are regular live files and exact members of the frozen
allowlist:

```text
02f1c30575a04194fe2d459469a36ba0e099c72d2c4979dc29bf2627fef6b63f  1,616,232  build-diagnostic/Swarm
034eab2688f36fa330838513ab597e6ef83283c85cda6310b2aa639a24c9a264     19,242  build-diagnostic/CMakeCache.txt
```

The binary is executable. The cache contains `ENABLE_GUROBI:BOOL=ON`.
Fresh recomputation matched the registered `otool -L` identity
`1cdf696769ab1ba14df66af7d6c47a9a0ed8ffda1356f52e59389f0124832492`
(621 bytes) and `conda list --explicit` identity
`361248f9642ee3c46023f5d5a7253e6c03c7a8cde71eae525bd71d4465aeb3c0`
(7,040 bytes). Base, primary, and ablation configuration identities match;
the primary selects `dynamic-lower-index` and the ablation selects
`fixed-cbf-only`.

## Schedule, universes, thresholds, argv, roots, and capacity

The full trusted schedule envelope has ten ordered missions, campaign ID
`development-v3`, trajectory seeds `2026080101..2026080110`, range-noise
seeds `2026081101..2026081110`, 1,000 frames, 500 s horizons, and the exact
ordered conditions `dynamic_primary`, `fixed_fim_ablation`. Its independently
derived canonical identity is SHA-256
`2b9effe4f8cdc79a911815f650e41ffffb831ba6e3037ad4b60fa43f46e4c48c`
over 2,041 bytes. It equals both the runner development schedule and the
analyzer's independently derived schedule envelope.

The frozen universes are:

```text
initialization              140
estimator per condition  139,860
estimator total          279,720
controller                10,000
allocated endpoints     2,320,000
reconstructed rows      1,190,000
missions                     10
```

All thresholds equal the production frozen dictionary: containment 0.98,
minimum-depth containment 0.95, joint available-and-contained 0.93, fresh
retention 0.98, availability 0.95, controller-certificate availability 0.99,
mission success 0.95, maximum finite error 50 m, rate tolerance `1e-9`, input
tolerance `1e-7`, residual tolerance `-1e-7`, start/stop space 8/6 GB,
reusable-cache cap 2 GB, and compact-output cap 25 MB. Supervision is 3,600 s
wallclock, 300 s complete-line stall, and 5 s termination grace.

The registered runner argv is exactly:

```text
conda run -n cbf_env python scripts/diagnostics/run_qualified_closure_campaign.py --kind development --version v3 --protocol docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.json --authorization docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-authorization.json --binary build-diagnostic/Swarm --base-config config/config.json --primary-config config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json --trajectory-seeds 2026080101:2026080110 --range-noise-seeds 2026081101:2026081110 --frames 1000 --output-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v3
```

The registered analyzer argv is exactly:

```text
conda run -n cbf_env python scripts/diagnostics/analyze_qualified_closure_campaign.py --kind development --version v3 --protocol docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.json --authorization docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-authorization.json --input-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v3 --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json --output-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v3
```

Both v3 roots are absent, are not symlinks, and have no symbolic ancestor:

```text
/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v3
/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v3
```

At preflight, `/private/tmp` had 15,971,381,248 free bytes, exceeding the
registered 8,000,000,000-byte launch floor. No root was claimed or created.

## Failed-v1 and failed-v2 preservation

The three failed-v1 artifacts remain byte-identical to their exact blobs in
`0e4c439...`. Its authorization, result/review, raw root, and analysis root
remain absent. The four development-v2 lifecycle artifacts remain
byte-identical to `ae7f68c...`, and the postcommit failure archive remains
byte-identical to `02f0f13...` at SHA-256
`815988e3149d6adcffb61d323e242905ae3e89a145eb37560abf1b47de9fb235`.
Both v2 roots and its result/review remain absent. The unrelated consumed
two-range v2 development root remains present and its paired analysis root
remains absent. No historical artifact or root was modified.

Development-v1 and development-v2 remain immutable, consumed,
non-executable evidence. They cannot be retried, overwritten, amended,
reinterpreted, or consumed as a later scientific result.

## Independent verification and authorization boundary

The focused registrar suite passed 39/39. The independent Standards/integrity
axis and Specification/conformance axis each return C0/I0/M0. No protocol was
rewritten, no authorization was created, no repository path was staged or
committed, no execution root was created, and no campaign command was run.

The separately created authorization must use the exact ten-field production
schema, bind protocol SHA-256
`8359bc7beffb676208a50f770a43817dbd71fc551dd8687790caecaef49f7c2b`,
implementation identity `70affe063e358a22c2139984429e741ff5a03023`, this
preflight's final SHA-256, kind `development`, version `v3`, canonical date,
and the actual user-origin authorization text plus its UTF-8 SHA-256. Test
fixtures, historical authorization text, and inferred approval are not
production authorization. Creating that JSON alone still does not authorize
execution; the exact committed four-artifact direct-child state must pass the
production gate first.
