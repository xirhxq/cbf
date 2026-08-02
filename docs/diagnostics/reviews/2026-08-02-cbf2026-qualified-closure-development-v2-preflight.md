# Qualified-closure development-v2 independent preflight

Date: 2026-08-02 (Asia/Shanghai)

Worktree: `/private/tmp/cbf2026-diagnostic`

Branch: `codex/cbf2026-diagnostic`

## Decision

Critical: `0`; Important: `0`; Minor: `0`.

Ready to create the exact bound authorization and commit exactly the four
development-v2 lifecycle artifacts as one add-only direct child: **Yes**.

Execution authorized now: **No**.

Development-v2 may become eligible only after the authorization below is
created without changing any reviewed byte, all four files are committed as
the sole direct child of the registered parent, and that committed state is
independently revalidated by the production authorization gate. This preflight
does not authorize a campaign, create an execution root, permit a retry, or
permit a separate protocol-pair commit.

## Reviewed protocol pair

The reviewed files are regular non-symlink files at the exact registered
publication paths:

```text
fd138abf50c95db5b3d43c602b91508e6c13521dbcabb3354c87bd32b9fc7538  42,233 bytes  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.json
1102a7a1663678762c126066685e835991b3449a8147c8eb8d7f6c62fc460da0     312 bytes  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.md
```

Independent calls to the production canonical-pair, exact-schema, semantic
rehash, frozen-derived-contract, and registered-argv validators all passed.
The protocol semantic SHA-256 is
`5d68ef523fa8e8a73113b36e010554f6202f213cb1ccd40fc8452aff1c41dacc`.
The Markdown is the exact renderer output for these JSON bytes and binds the
same JSON SHA-256.

The protocol is exactly kind `development`, version `v2`, conditions
`dynamic_primary` and `fixed_fim_ablation`, and `no_retry=true`. Its frozen
authorization declaration requires protocol SHA-256, implementation identity,
preflight SHA-256, and user authorization.

## Repository parent and future commit topology

The registered and observed implementation/documentation parent is exactly:

```text
HEAD  857de7ee8dd563fdd5fa15b6860ed54da6758b37
tree  d6a8d34b50a23177fa54bb426093f5591f01d2b5
root  /private/tmp/cbf2026-diagnostic
```

`857de7ee...` has sole parent `eafa68c3631c590b9f1070bcc8d313ca4f3b7705`
and is the clean report/plan/review documentation-closure commit required by
the recovery plan. The protocol correctly binds `857de7ee...`, not the earlier
code-fix commit `eafa68c...`.

At preflight start the tracked worktree was clean. The observed untracked state
was exactly the 512 `build-diagnostic/` paths captured in the protocol plus the
two new protocol files; no other relevant path existed. The future preflight
and authorization paths were absent. The parent commit contains none of the
four lifecycle paths.

The only permitted next repository commit must have exactly one parent equal
to `857de7ee...` and an exact no-renames name-status diff consisting of four
`A` entries and no other path:

```text
A  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.json
A  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.md
A  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-preflight.md
A  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-authorization.json
```

The protocol pair must not be committed separately. No amend, second
descendant, merge commit, extra path, modification, deletion, rename, dirty
tracked/relevant path, or live/HEAD blob mismatch is acceptable. After the
four-file commit, every live artifact byte must equal its exact HEAD blob and
every registered source/report/tool/config identity must still equal the
registered parent blob.

## Implementation, report, plan, code, and build identities

Every registered live file was independently hashed and sized. Each tracked
identity also matched the exact blob at `857de7ee...`:

```text
e54e4a0cc3df1b33c26b7490d1793b1bdbe422ab57c403f30b975184d48bd943  31,753  implementation report
c7e7f184805df737c80a4eab5702b9aeacca7e5e80038feab8304bb0d05e89e3  35,487  implementation review
e4cb6569e532f3dcb9358b6fd796420e0060d4eaf9430244e12cb492049acfda 152,241  recovery plan
13338d80859d9f1db3855d6fcfe988639bf08bbe50c166f5a8473de330beb26e  94,647  campaign runner/source
c1e2965c61ff71b7384bbf9ee02e80d41c842faa77734e66dc444be3a02803c3 108,247  evidence schema
174baa3dc5140a6df5f63de9a8296ea9181e582c2f8efd20c4c9cc70295e28f9 103,773  campaign analyzer
83c2b95243203f494d5f416600d2d02828da44855d16a7f636d51a01f6859631  55,998  campaign registrar
2b354a7a629baa4a8f937365079764bfeb0d8667f8c675a5b2814721ff26ae9b  27,558  measurement generator
4a47920e715c274c4e41a5813334f2de5dd36cfac45a0e462d261356646b4aea  49,283  estimator replay
e0eeb0bd2d54ac49160aa862734a0b188c1da7587d005e7357e48a1c69b3b60f  57,353  estimator analyzer
```

The implementation report embeds the exact recovery-plan SHA-256 above. The
binary and configuration bindings also matched exactly:

```text
02f1c30575a04194fe2d459469a36ba0e099c72d2c4979dc29bf2627fef6b63f 1,616,232  build-diagnostic/Swarm
0920ca21fc57e51d44b0df3c00bc32bf4d9fe5bbc7f6fa8bb826def1b88184ac     4,783  config/config.json
6a198e17b8c3bbf6777caaa325fa593dd24ee9ce3981f90769075112be1cacb3     1,330  primary config
6ad20e1e32fff5e8950f8de31864d77a238f6d418abafe3a50cdb292b8c0bc66     1,325  fixed-FIM ablation config
034eab2688f36fa330838513ab597e6ef83283c85cda6310b2aa639a24c9a264    19,242  CMake cache
```

The CMake cache contains exactly `ENABLE_GUROBI:BOOL=ON`. Fresh recomputation
matched the registered `otool -L` identity
`1cdf696769ab1ba14df66af7d6c47a9a0ed8ffda1356f52e59389f0124832492`
(621 bytes) and the registered `conda list --explicit` identity
`361248f9642ee3c46023f5d5a7253e6c03c7a8cde71eae525bd71d4465aeb3c0`
(7,040 bytes).

## Schedule, universes, thresholds, argv, roots, and capacity

The exact ten mission pairs are trajectory seeds
`2026080101..2026080110` paired positionally with range-noise seeds
`2026081101..2026081110`. Each mission has 1,000 frames and a 500 s horizon.
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

Every threshold equals the production frozen dictionary: containment 0.98,
minimum-depth containment 0.95, joint available-and-contained 0.93, fresh
retention 0.98, availability 0.95, controller-certificate availability 0.99,
mission success 0.95, maximum finite error 50 m, rate tolerance `1e-9`, input
tolerance `1e-7`, residual tolerance `-1e-7`, start/stop space 8/6 GB, reusable
cache cap 2 GB, and compact-output cap 25 MB. Supervision remains 3,600 s
wallclock, 300 s complete-line stall, and 5 s termination grace.

The serialized runner and analyzer arrays match the registrar's independent
production derivation token-for-token. They use only version `v2`, the exact
protocol/authorization names, the registered binary/configs and seed ranges,
and the exact roots:

```text
/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v2
/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v2
```

Both roots are absent, non-symlink paths with no symbolic ancestor. At the
space check, `/private/tmp` had 16,050,257,920 available bytes, exceeding both
8 GiB and the registered 8,000,000,000-byte launch floor. No root was created.

## Failed-v1 preservation

The immutable failed-v1 artifacts remain byte-identical to their exact blobs
in commit `0e4c439ae35f8490b27019aeb26b1c46ba9ab3f7`:

```text
9a5e9d2503d083bdbb239468a88cf5ceed2cc283978e8beed5badc020ae74343  41,979 bytes  v1 protocol JSON
19cf657d787bfe76bfa2aecf35ce06baf9fd2f57ff2deeab12e894429ccebffc     312 bytes  v1 protocol Markdown
7ef7fbbf09d1ef95ecad482282a75398158fb97008fdb8a08745efeb51523515   9,274 bytes  failed v1 preflight
```

That commit adds exactly those three files. The v1 authorization and both v1
execution roots remain absent. Failed v1 is consumed and cannot be retried,
overwritten, repaired, reinterpreted, or used for execution.

## Independent verification

The registrar lifecycle suite passed 38/38. All three lifecycle production
scripts passed `py_compile`, and `git diff --check` returned zero. The
independent Standards/integrity axis and Specification/conformance axis each
returned C0/I0/M0. No campaign command was run and no v2 result exists.

## Authorization construction boundary

Because every check above passed, the separately created authorization may
contain only the exact ten production fields, with `authorized=true`, kind
`development`, version `v2`, the protocol SHA-256 above, implementation
identity `857de7ee...`, this preflight's exact SHA-256, date `2026-08-02`, and
the researcher's original authorization text `批准`. The UTF-8 SHA-256 of that
exact text is:

```text
8cbe697b157364a5b13646285b38409dc53ec5287deeb7913493e65b275cd14d
```

English unit-test fixture text is not production authorization. Creation of
the bound JSON does not itself authorize execution; only the exact committed
four-artifact direct-child state can pass the production gate.
