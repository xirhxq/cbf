# Predictive WNLS Stage 0 fixtures

The immutable `cbf2026-predictive-wnls-stage0-fixtures-v2` fixtures replay complete deterministic 14-UAV prefixes. Runtime uses only configured data, measurement presence/noisy ranges, communicated estimates, and preceding applied commands. Truth and legacy baseline comparisons are confined to separate offline subtrees and are evaluated only after runtime execution.

Frozen source hashes: truth data `3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527`; input manifest `6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb`; baseline process `c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003`; unchanged legacy replay `0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8`. The extractor checks each input before and after reading and fails closed when the fixture root exists.

| Fixture | SHA-256 | Baseline target | New target/result |
|---|---|---|---|
| `frame44_recovery.json` | `b7ad93f09a3b487a5d581bc9a8f6240c56377ed7d300bcc10109ebfc018bb8a7` | UAV 14, seed 20260736/frame 44: failed, stale, 12.630872 m | accepted fresh from prediction start, 1.607815 m |
| `frame177_cascade.json` | `ddccd65d147b366ce79fd0b742f89180c67b2932276058fe8a1e206f2f2deb59` | UAV 14, seed 20260730/frame 177: converged, 168.901697 m | reference-unavailable and unpublished |
| `reacquisition.json` | `25e5eebbb27ae5ee649194495fe70c32178affc72fa022eb044583fd53bcd590` | synthetic, no baseline row | two ranges: rejected/unavailable; three non-collinear ranges: accepted fresh, 0 m offline error |
| `manifest.json` | `fed7f1a14b64d3f08dba2e97749f93fad715c2a1c9c3f2765d8ebd0f4b24d7b1` | fixture identity manifest | binds schema, sources, legacy source, and fixture hashes |

Frame 44 is a genuine fresh measurement update, not a prediction-only availability change. Its selected candidate is the command-prediction start and meets the frozen 5 m check. Frame 177 makes a different change: it does not relabel the bad branch as fresh. Across its 2,492 rows, maximum fresh offline error is 16.826126 m and no used UAV anchor is nonfresh. Its counts are 2,345 fresh, 56 predicted, and 91 unavailable, so availability loss remains explicit rather than being treated as an error improvement.

Every row retains complete candidate diagnostics. The frame-44 prefix contains 211 innovation-gate and 22 non-converged candidate rejections. The frame-177 prefix contains 1,284 innovation-gate, 207 non-converged, 4 excessive reduced-cost, and 93 insufficient-three-reference candidate rejections. The synthetic fixture contains two insufficient-three-reference rejections in its two-range frame. The per-candidate source, proposal trace, gate diagnostics, and rejection reason remain in the executed fixture rows.

These are frozen mechanism regressions, not a generalization claim or a replacement for bounded-input confirmation.
