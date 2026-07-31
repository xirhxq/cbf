# CBF2026 Two-Range Reacquisition Protocol

This protocol freezes deterministic smoke validation and one
externally authorized, no-retry registered development replay.
The paper gate remains `CLOSED`.

- Protocol schema: `cbf2026-two-range-reacquisition-protocol-v1`
- Registration schema: `cbf2026-two-range-reacquisition-registration-v1`
- Protocol ID: `cbf2026-two-range-reacquisition-v1`
- Implementation parent: `5a14eb66ffeb541540183041ce2966c5dfd6d949`
- JSON SHA-256: `8e01df56623bb2e759fb37108ddc68fbc41e301b8305ade7322e212ae6e3ea16`

## Exact commands

### `smoke_a`

```text
conda run -n cbf_env python scripts/diagnostics/replay_two_range_reacquisition.py --protocol-path docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json --data-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json --input-manifest-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json --output-root /private/tmp/cbf2026-two-range-reacquisition-smoke-v1-a --run-seeds --max-frames 0 --invocation-name smoke_a
```

### `smoke_b`

```text
conda run -n cbf_env python scripts/diagnostics/replay_two_range_reacquisition.py --protocol-path docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json --data-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json --input-manifest-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json --output-root /private/tmp/cbf2026-two-range-reacquisition-smoke-v1-b --run-seeds --max-frames 0 --invocation-name smoke_b
```

### `smoke_analyzer_a`

```text
conda run -n cbf_env python scripts/diagnostics/analyze_two_range_reacquisition.py --protocol-path docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json --raw-root /private/tmp/cbf2026-two-range-reacquisition-smoke-v1-a --output-root /private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-a --invocation-name smoke_analyzer_a
```

### `smoke_analyzer_b`

```text
conda run -n cbf_env python scripts/diagnostics/analyze_two_range_reacquisition.py --protocol-path docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json --raw-root /private/tmp/cbf2026-two-range-reacquisition-smoke-v1-b --output-root /private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-b --invocation-name smoke_analyzer_b
```

### `registered_replay`

```text
conda run -n cbf_env python scripts/diagnostics/replay_two_range_reacquisition.py --protocol-path docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json --data-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/2026-07-28_14-27-53_R_seed_20260727_250s/data.json --input-manifest-path /private/tmp/cbf2026-results/mc-first-mechanism-250s/R/20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json --output-root /private/tmp/cbf2026-two-range-reacquisition-development/v1 --run-seeds 20260727 20260728 20260729 20260730 20260731 20260732 20260733 20260734 20260735 20260736 20260737 20260738 20260739 20260740 20260741 20260742 20260743 20260744 20260745 20260746 --max-frames 500 --invocation-name registered_replay --authorization-json docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-registered-authorization.json
```

### `registered_analyzer`

```text
conda run -n cbf_env python scripts/diagnostics/analyze_two_range_reacquisition.py --protocol-path docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json --raw-root /private/tmp/cbf2026-two-range-reacquisition-development/v1 --output-root /private/tmp/cbf2026-two-range-reacquisition-analysis/v1 --invocation-name registered_analyzer --authorization-json docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-registered-authorization.json
```
