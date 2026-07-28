# Diagnostic Source Snapshots

This directory preserves compact source bundles that bind diagnostic evidence
to the exact working-tree state used to produce it.

`2026-07-27-evidence-source.tar.gz` captures the CBF2026 diagnostic source
state after Tasks 1--4 and before the post-run provenance-runner repair.
It excludes Git metadata, build products, hidden SDD coordination files, and
the snapshot directory itself.

The archive is evidence, not a substitute for integrating the reviewed source
changes into Git.
Its SHA-256 is recorded in the compact gate reports and central provenance
index.
The five historical smoke/full manifests remain byte-for-byte unchanged.
Non-destructive `provenance-amendment.json` sidecars now bind their artifacts
to this archive, and
`docs/diagnostics/2026-07-27-evidence-provenance.json` compactly enumerates the
same bindings.
Byte-for-byte copies of the approved Task 5 and Task 6 independent reviews
are available under `docs/diagnostics/reviews/`.
The archive, index, reports, review copies, and reviewed source are still
uncommitted; durable preservation remains pending user-authorized Git
integration.
