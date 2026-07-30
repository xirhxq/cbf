# CBF2026 Predictive WNLS Stage 1 Protocol

This protocol freezes a paired, single-trajectory development replay.
It is not confirmatory paper evidence, and the paper gate is `CLOSED`.

- Protocol schema: `cbf2026-predictive-wnls-stage1-protocol-v4`
- Protocol ID: `cbf2026-predictive-wnls-stage1-v4`
- Implementation parent: `68a9a0c1c85abc33ce85b9b846d43b139a1d5ebf`
- Registered retry allowed: `false`
- Exact output roots, exclusive creation, and terminal manifests: required

## Frozen sources

| Name | Absolute path | SHA-256 |
|---|---|---|
| `analyzer_source` | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/analyze_predictive_wnls_recovery.py` | `bbc4e6e0940580f48222f641cba0f705c8b8f419c585a18999cb1cdc9e4146a8` |
| `baseline_process` | `/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/restart/localization-calibration/20260729T112437.413270Z_efb8e7137776415c833f11204cf519d5/calibration.jsonl.gz` | `c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003` |
| `diagnostic_integrity_source` | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/run_diagnostic.py` | `ccde94fa739649cb9b94a8304ef3867ae946f28b0e3a812d3feb3f30bb76fd23` |
| `estimator_source` | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/predictive_wnls.py` | `3decd0d1eabfb0e54f56852d2e7c91726579ddfee806835502b1da6333724ea7` |
| `input_manifest` | `/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json` | `6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb` |
| `legacy_solver_source` | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/replay_localization_calibration.py` | `0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8` |
| `replay_source` | `/private/tmp/cbf2026-diagnostic/scripts/diagnostics/replay_predictive_wnls_recovery.py` | `10922bf369822a6898053b7d326bc4937a7da3e0b44a898eaeea4d18fc752661` |
| `truth_data` | `/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json` | `3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527` |

## Frozen cohort and gates

- Range-noise seeds: `20260727` through `20260746` (20 exact seeds)
- Variants: `prediction_expiry`, `fresh_reference_qualification`, `predictive_multistart`
- Maximum published/fresh error gate: strictly below `50 m`
- Fresh-or-predicted availability gate: at least `0.95`
- Maximum public prediction age: `2` frames
- Paper gate: `CLOSED`

## Exact commands

### `smoke_a`

```text
conda run -n cbf_env python scripts/diagnostics/replay_predictive_wnls_recovery.py --data-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json --input-manifest-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json --protocol-json docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol-v4.json --output-root /private/tmp/cbf2026-predictive-wnls-smoke-v4-a --run-seeds 20260727 --max-frames 2
```

### `smoke_b`

```text
conda run -n cbf_env python scripts/diagnostics/replay_predictive_wnls_recovery.py --data-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json --input-manifest-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json --protocol-json docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol-v4.json --output-root /private/tmp/cbf2026-predictive-wnls-smoke-v4-b --run-seeds 20260727 --max-frames 2
```

### `registered_replay`

```text
conda run -n cbf_env python scripts/diagnostics/replay_predictive_wnls_recovery.py --data-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json --input-manifest-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json --protocol-json docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol-v4.json --output-root /private/tmp/cbf2026-predictive-wnls-development/stage1-v4 --run-seeds 20260727,20260728,20260729,20260730,20260731,20260732,20260733,20260734,20260735,20260736,20260737,20260738,20260739,20260740,20260741,20260742,20260743,20260744,20260745,20260746
```

### `registered_analyzer`

```text
conda run -n cbf_env python scripts/diagnostics/analyze_predictive_wnls_recovery.py --baseline-process-path /private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/restart/localization-calibration/20260729T112437.413270Z_efb8e7137776415c833f11204cf519d5/calibration.jsonl.gz --development-manifest-path /private/tmp/cbf2026-predictive-wnls-development/stage1-v4/manifest.json --protocol-json docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol-v4.json --expected-baseline-sha256 c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003 --output-root /private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v4
```

## Machine-readable contract

The companion JSON file is authoritative for every constant, schema,
ablation, invocation, command token, disk cap, gate, and lifecycle rule.
