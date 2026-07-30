# Predictive WNLS Stage 1 v2 smoke failure record

## Immutable outcome

The Stage 1 v2 smoke registration is retired after both of its predeclared
smoke commands failed before evidence-root allocation.
Each command was invoked exactly once.
Neither command may be retried,
and none of the v2 smoke or registered target paths may be reused by a
replacement protocol,
even though all four remained absent.

The registered replay and registered analyzer were not invoked.
The paper-edit gate remains closed.

## Frozen protocol and approval

The machine-readable v2 protocol was committed by:

```text
5e5dd36d1234659266d7990c44a6e765061a4f6f
docs(diagnostics): freeze predictive WNLS stage1 protocol
```

The independent v2 preflight approval was committed by:

```text
47426f54c9999992b3cc7315c29d2e9c53467eb4
docs(diagnostics): approve predictive WNLS preflight
```

The frozen protocol files had the independently recorded SHA-256 identities:

```text
9513cfc06caa80333dc6c6d368c7ce6de79ea05450bc561ef7275429eaf6a5b1  docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.json
34f44ea467faf9e54dc26804b323c120d55cbe560a27e451140c358002f72581  docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.md
```

Immediately before execution,
the repository was at the exact approved preflight commit.
Tracked files were clean,
and `build-diagnostic/` was the only untracked path.
The protocol-bound implementation diff check passed.

## Exactly-once smoke invocations

Smoke A used the exact frozen command below once:

```text
conda run -n cbf_env python scripts/diagnostics/replay_predictive_wnls_recovery.py --data-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json --input-manifest-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json --protocol-json docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.json --output-root /private/tmp/cbf2026-predictive-wnls-smoke-a --run-seeds 20260727 --max-frames 2
```

Smoke A exited with status `1`.
It failed at replay-module import before allocation:

```text
ModuleNotFoundError: No module named 'scripts'
```

Smoke B then used its exact distinct-root frozen command once:

```text
conda run -n cbf_env python scripts/diagnostics/replay_predictive_wnls_recovery.py --data-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json --input-manifest-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json --protocol-json docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.json --output-root /private/tmp/cbf2026-predictive-wnls-smoke-b --run-seeds 20260727 --max-frames 2
```

Smoke B also exited with status `1`
and the same pre-allocation `ModuleNotFoundError`.
Neither target was retried.

The failure mechanism is that direct execution of
`scripts/diagnostics/replay_predictive_wnls_recovery.py`
does not make the repository root importable,
while the script imports `scripts.diagnostics.*`.
This is an invocation/interface defect in the frozen direct CLI,
not an estimator result.

## Evidence and disk audit

Immediately before the smoke pair,
`/private/tmp` had `47,851,160 KiB`
(`48,999,587,840` bytes) available,
well above the frozen 8 GB launch floor.
The final read-only probe reported `47,852,152 KiB`
(`49,000,603,648` bytes) available.

The post-failure audit found all exact v2 targets absent:

```text
/private/tmp/cbf2026-predictive-wnls-smoke-a
/private/tmp/cbf2026-predictive-wnls-smoke-b
/private/tmp/cbf2026-predictive-wnls-development/stage1-v2
/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v2
```

No terminal manifest,
raw process stream,
compressed process stream,
quarantine entry,
partial evidence,
or compact analysis root was created.
Allocated evidence bytes were therefore zero for every v2 target.
No row count or process SHA-256 exists to compare.
The registered replay and analyzer commands remained unrun.

## Retirement and required remediation

This record permanently retires the v2 command tokens and all four v2 target
paths.
Target absence does not restore authorization:
the exactly-once v2 smoke opportunities were consumed by the two failed
invocations.
The v2 protocol and preflight approval cannot authorize a retry or a corrected
command.

Any replacement must:

1. implement and test secure direct-CLI startup for both replay and analyzer,
   including execution from the frozen repository working directory without
   relying on an ambient `PYTHONPATH`;
2. use a distinct v3 protocol schema version,
   protocol identifier,
   registration token,
   smoke roots,
   registered replay root,
   and registered analyzer root;
3. regenerate the complete machine-readable and Markdown protocol from the
   corrected committed implementation;
4. independently review and commit a new preflight approval that explicitly
   exercises the exact direct CLI in validation-only or other non-evidence
   mode; and
5. authorize new exactly-once smoke and registered invocations only after that
   v3 approval.

No code,
plan,
v2 protocol,
or evidence directory is changed by this failure record.
