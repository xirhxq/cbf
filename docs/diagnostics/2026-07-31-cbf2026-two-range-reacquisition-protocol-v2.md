# CBF2026 Two-Range Reacquisition Protocol

This protocol freezes deterministic smoke validation and one
externally authorized, no-retry registered development replay.
The paper gate remains `CLOSED`.

- Protocol schema: `cbf2026-two-range-reacquisition-protocol-v2`
- Registration schema: `cbf2026-two-range-reacquisition-registration-v2`
- Protocol ID: `cbf2026-two-range-reacquisition-v2`
- Implementation parent: `526e1418138426e54bff5a41e3847b6d2a9f8203`
- JSON SHA-256: `9a0985e00e1b6bd801f7b46c3811c2e135c29f86f75b5045e7aff1ad71206bb2`

## Exact commands

### `smoke_a`

```text
conda run -n cbf_env python scripts/diagnostics/replay_two_range_reacquisition.py --protocol-path docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json --data-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json --input-manifest-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json --output-root /private/tmp/cbf2026-two-range-reacquisition-smoke-v2-a --run-seeds --max-frames 0 --invocation-name smoke_a
```

### `smoke_b`

```text
conda run -n cbf_env python scripts/diagnostics/replay_two_range_reacquisition.py --protocol-path docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json --data-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json --input-manifest-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json --output-root /private/tmp/cbf2026-two-range-reacquisition-smoke-v2-b --run-seeds --max-frames 0 --invocation-name smoke_b
```

### `smoke_analyzer_a`

```text
conda run -n cbf_env python scripts/diagnostics/analyze_two_range_reacquisition.py --protocol-path docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json --raw-root /private/tmp/cbf2026-two-range-reacquisition-smoke-v2-a --output-root /private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v2-a --invocation-name smoke_analyzer_a
```

### `smoke_analyzer_b`

```text
conda run -n cbf_env python scripts/diagnostics/analyze_two_range_reacquisition.py --protocol-path docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json --raw-root /private/tmp/cbf2026-two-range-reacquisition-smoke-v2-b --output-root /private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v2-b --invocation-name smoke_analyzer_b
```

### `registered_replay`

```text
conda run -n cbf_env python scripts/diagnostics/replay_two_range_reacquisition.py --protocol-path docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json --data-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json --input-manifest-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json --output-root /private/tmp/cbf2026-two-range-reacquisition-development/v2 --run-seeds 20260727 20260728 20260729 20260730 20260731 20260732 20260733 20260734 20260735 20260736 20260737 20260738 20260739 20260740 20260741 20260742 20260743 20260744 20260745 20260746 --max-frames 500 --invocation-name registered_replay --authorization-json docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-registered-v2-authorization.json
```

### `registered_analyzer`

```text
conda run -n cbf_env python scripts/diagnostics/analyze_two_range_reacquisition.py --protocol-path docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json --raw-root /private/tmp/cbf2026-two-range-reacquisition-development/v2 --output-root /private/tmp/cbf2026-two-range-reacquisition-analysis/v2 --invocation-name registered_analyzer --authorization-json docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-registered-v2-authorization.json
```
