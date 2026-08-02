# Qualified-closure development-v2 post-commit validation failure

Date: 2026-08-02 (Asia/Shanghai)

Worktree: `/private/tmp/cbf2026-diagnostic`

Branch: `codex/cbf2026-diagnostic`

## Decision

Critical: `1`; Important: `0`; Minor: `0`.

Development-v2 execution: **NOT AUTHORIZED**.

The exact four-artifact commit topology and the protocol, preflight, and user
authorization byte bindings were correct, but the mandatory production
post-commit validator failed before any campaign root was claimed. This is an
authorization-lifecycle implementation failure, not an empirical result.

## Frozen v2 identities

The documentation parent and sole four-artifact child are:

```text
implementation/documentation parent  857de7ee8dd563fdd5fa15b6860ed54da6758b37
four-artifact authorization child     ae7f68c
```

The child adds exactly the following four paths and no other path:

```text
A  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.json
A  docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v2-protocol.md
A  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-authorization.json
A  docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v2-preflight.md
```

Their live and committed identities at validation were:

```text
fd138abf50c95db5b3d43c602b91508e6c13521dbcabb3354c87bd32b9fc7538  protocol JSON
1102a7a1663678762c126066685e835991b3449a8147c8eb8d7f6c62fc460da0  protocol Markdown
c790cd2252e9c73209f7b3d19994233f1997695173e8ca9d6e4ceb2fba07019d  independent preflight
aa0756887dbdf0d24f6d81bc845986c98713964844cbf4434cd65389cc2d9179  authorization JSON
```

The authorization binds the exact user-origin text `批准`, dated
`2026-08-02`; its UTF-8 SHA-256 is
`8cbe697b157364a5b13646285b38409dc53ec5287deeb7913493e65b275cd14d`.

## Reproduced blocker

Immediately after the exact child commit, the production call
`validate_authorization_binding(...)` failed with:

```text
ValueError: registered Git state is unavailable: fatal: path
'build-diagnostic/Swarm' exists on disk, but not in
'857de7ee8dd563fdd5fa15b6860ed54da6758b37'
```

The validator first accepted the direct-child ancestry and exact four-file
diff. It then merged all `bindings`, `review_artifacts`, and `tooling`
identities and attempted `git show <implementation>:<path>` for every path
inside the worktree. That rule is correct for tracked source, configuration,
tool, report, and review files, but it is impossible for the deliberately
untracked `build-diagnostic/Swarm` binary and CMake cache. Those files were
already frozen by exact path, byte count, SHA-256, dependency identity, and the
protocol's exact `allowed_untracked_paths` set. The implementation therefore
confused a registered allowed-untracked build binding with a tracked Git-blob
binding.

## No-campaign and no-retry disposition

Both registered roots remained absent after the failure:

```text
/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v2
/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v2
```

No runner, analyzer, mission, or empirical computation was launched. The v2
protocol, preflight, authorization, and four-artifact commit are immutable
failed-gate evidence. They must not be retried, overwritten, amended,
reinterpreted, or used for execution.

Recovery requires a red/green regression with a genuinely untracked registered
binary, a fix that preserves Git-blob checks for tracked identities while
checking allowed-untracked identities against their exact live and registered
status/hash/size bindings, independent review, and a new non-colliding
development-v3 protocol and root namespace. Confirmatory remains closed.
