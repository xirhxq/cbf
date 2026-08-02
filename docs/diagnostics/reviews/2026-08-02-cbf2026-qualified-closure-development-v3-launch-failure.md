# Qualified-closure development-v3 launch failure

Date: 2026-08-02 (Asia/Shanghai)

Worktree: `/private/tmp/cbf2026-diagnostic`

Branch: `codex/cbf2026-diagnostic`

## Decision

Critical: `1`; Important: `0`; Minor: `0`.

Development-v3 is consumed and **must not be retried**. Confirmatory remains
closed.

The exact registered runner invocation failed before it created or claimed
the raw root. This is a CLI/package-resolution implementation failure, not a
mission, safety, estimator, controller, or performance result.

## Authorized state before invocation

The production postcommit authorization gate had passed at exact source HEAD
`16c0aca90f578259b16a75659433ad12841ae10e`, the sole direct child of
documentation parent `70affe063e358a22c2139984429e741ff5a03023`. The
child added exactly the development-v3 protocol JSON, protocol Markdown,
independent C0/I0/M0 preflight, and ten-field authorization. The protocol
bound this exact runner argv:

```text
conda run -n cbf_env python scripts/diagnostics/run_qualified_closure_campaign.py --kind development --version v3 --protocol docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v3-protocol.json --authorization docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-closure-development-v3-authorization.json --binary build-diagnostic/Swarm --base-config config/config.json --primary-config config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json --ablation-config config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v1.json --trajectory-seeds 2026080101:2026080110 --range-noise-seeds 2026081101:2026081110 --frames 1000 --output-root /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v3
```

Immediately before launch, production authorization validation passed, the
serialized argv matched token-for-token, source and DRA worktrees were clean
within their declared scopes, both v3 roots were absent, and `/private/tmp`
had about 15 GB free.

## Exact failure

The sole runner invocation exited nonzero with:

```text
qualified campaign failed: No module named 'scripts'
```

The frozen argv uses direct-script mode:
`python scripts/diagnostics/run_qualified_closure_campaign.py`. Python then
places the script directory rather than the repository root at the front of
the module search path. A runtime absolute import of
`scripts.diagnostics...` therefore fails. The authorization lifecycle and
registered argv checks all agreed on the same unusable command, so their
agreement did not establish real package-launch viability.

## No-data boundary

After the failure, both registered roots remained absent:

```text
/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v3
/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v3
```

No mission directory, measurement stream, Swarm process, estimator replay,
controller evidence, terminal raw manifest, analysis root, compact result, or
empirical observation was created. Disk space remained about 15 GB. Nothing
was deleted or retried.

## Required recovery

The v3 protocol, authorization, sole invocation, and this failure record are
immutable audit evidence. Recovery requires a real subprocess RED that uses
the frozen cwd and command shape, a complete audit of registered and child
Python command vectors, an invariant-preserving package-resolution fix, fresh
review, and a non-colliding development-v4 protocol/root namespace. Changing
the command and rerunning v3 is forbidden.
