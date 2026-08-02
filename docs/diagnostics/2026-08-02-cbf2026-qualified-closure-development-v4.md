# Qualified-closure development-v4 terminal report

## Verdict

`FAIL — INITIAL THEOREM-CERTIFICATE RESET INFEASIBLE; 0/10 MISSIONS COMPLETED; NO RETRY`

Development-v4 is terminal and consumed. The registered runner and analyzer
were each invoked exactly once. Neither command may be rerun, resumed,
partitioned, overwritten, or reinterpreted. Confirmatory and every paper
evidence/claim gate remain closed.

This is not a large-localization-error result. No estimator frame, controller
interval, localization error, or ablation row was produced. The first Swarm
process failed closed during its frame-zero theorem-certificate reset, and the
runner synthesized the frozen missing-universe accounting for all remaining
work.

## Registered identities

The execution source was branch `codex/cbf2026-diagnostic`, upstream `none`,
at exact four-add-only authorization child
`1478f68a9576fb55a4bca65c607a7eaf76ff7a6d`, tree
`254cb41f878672c88b18ef52485df8e6d5854be0`. Its sole parent is the registered
implementation/documentation identity
`14a27b7cedb704aeaa49945bc21dab1df04dfef4`. The child adds exactly the four
development-v4 lifecycle artifacts and no other path.

```text
edfad6cbe68c1ac1459640f71e7c2664d6702cc519804ed6414ea7e8761526f8  42,237 bytes  protocol JSON
a5b6450e657bd3332f67d03d8f7cb5bdfb47629bbceb93f2a2fac4d3c693e8c0     312 bytes  protocol Markdown
5cc7b24591c5a0f7dfdeaaebe3160c0db3e0b1901d746f6d694fbdd2efb6a375  10,163 bytes  independent preflight
0089f4278130c6f31b3da7c8419b5fd4b8e996e2b8777bc0a9444032708d6ad7     565 bytes  authorization JSON
```

The independent preflight was C0/I0/M0. The authorization binds the actual
user text `批准`, its UTF-8 SHA-256
`8cbe697b157364a5b13646285b38409dc53ec5287deeb7913493e65b275cd14d`,
the protocol/preflight hashes above, and implementation identity `14a27b7...`.
Production postcommit validation passed before root allocation. Protocol
semantic SHA-256 is
`9cf24ba0455163d8b08877f98ff211eb4729f5dd8f37570ebc1631d7c7580647`.

The live binary and configs matched the registered identities:

```text
02f1c30575a04194fe2d459469a36ba0e099c72d2c4979dc29bf2627fef6b63f  build-diagnostic/Swarm
0920ca21fc57e51d44b0df3c00bc32bf4d9fe5bbc7f6fa8bb826def1b88184ac  config/config.json
6a198e17b8c3bbf6777caaa325fa593dd24ee9ce3981f90769075112be1cacb3  primary config
6ad20e1e32fff5e8950f8de31864d77a238f6d418abafe3a50cdb292b8c0bc66  fixed-FIM ablation config
```

`ENABLE_GUROBI:BOOL=ON`; `Swarm` was executable. The source porcelain was
exactly the preserved `?? build-diagnostic/`, and the separately committed
DRA authorization checkpoint was clean on `main` at
`cbbb199b6b590cda92ab0ba318bb54737463ad3e` before launch.

## Once-only execution

The runner used the protocol-serialized module argv with development version
v4, trajectory seeds `2026080101..2026080110`, range-noise seeds
`2026081101..2026081110`, 1,000 frames per mission, and raw root
`/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v4`. It was
invoked once and returned nonzero. The terminal raw manifest is:

```text
status                         failed
reason                         schema_error
mission_count                  10
completed_mission_count        0
manifest SHA-256               12f71bb00720636ca610ad8d9d380dde7e92147e7bdf57589bb8915c1bf72f71
raw regular files              28
raw logical bytes              12,618,979
raw per-file tree commitment   9b915ad84aeda2b22aafbd259bf61bbc9515e3b082559bf3e5f217e14af9b5ac
```

Mission-01 launched Swarm once. Its ordered stream contained exactly the 14
frame-zero initialization records before Swarm returned `1`; the supervisor
manifest records `valid_rows=14`, `missing_rows=0`, `status=failed`, and
`reason=schema_error`. Its stderr is exactly:

```text
[SIMULATION_ERROR] theorem certificate reset rejected: post-reset bounded local hard QP is infeasible for UAV 3 (infeasible)
```

The mission-01 manifest is terminal `failed/schema_error`. Missions 02--10
were not launched; the runner produced terminal failed manifests and exact
synthetic missing-universe streams. Consequently all ten mission manifests
are terminal failed and the raw campaign manifest accounts for the complete
registered schedule.

Because that raw manifest was terminal, the analyzer then used the
protocol-serialized module argv exactly once. It returned nonzero because the
scientific verdict is FAIL, while successfully publishing a terminal completed
analysis bundle at
`/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v4`:

```text
analysis manifest SHA-256      890174908e5df83e367edd33cb0d83960635cc94ac57fd79fafd553aef40e6f0
analysis JSON SHA-256          831bf600d818a2a6e3d1ef84530efe1377d0a49fae7ba93c4970aa8e948f20db
analysis Markdown SHA-256      85ee391655f720336a1a485ce678cbbc2be88efffdeb960e7b0a67ea4ae723a0
semantic analysis SHA-256      623f6e4d21f0463bdcf4aa89a53a4bbe0d324b2b996fb42e7e0e375cd5edff33
analysis regular files         3
analysis logical bytes         8,639
analysis tree commitment       155b6b9372f91bb7b4b2476581b4270e7a372c680cd8b2ea1847ef9c5766729a
```

## Independently analyzable gate outcome

The analyzer includes the registered ten missions in the scientific
denominator and reports `passed=false`. The decisive accounting is:

```text
successful missions                         0 / 10       FAIL
complete keys/manifests                     0 / 1        FAIL
initialization omissions                  140 / 140      FAIL
fresh/available estimator tuples            0 / 139,860  FAIL
joint available-and-contained tuples        0 / 139,860  FAIL
controller certificate intervals            0 / 10,000   FAIL
missing/duplicate allocated endpoint rows    2,320,000 / 2,320,000  FAIL
primary localization errors                  0 samples
fixed-FIM ablation errors                    0 samples
```

Zero-count safety/error gates do not constitute positive evidence. For
example, zero true-distance violations, zero input-limit violations, and zero
finite errors above 50 m all arise with no controller/error sample. They may
not be reported as safety or localization success. The only positive integrity
gate of substance is that primary/ablation use the same immutable measurement
declaration; execution ended before either estimator condition produced rows.

## Disk and immutability

The launch check observed about 15.45 million KiB free, above the registered
8 GB floor. The terminal raw and analysis bundles together use far below the
2 GB cache cap; about 15 GB remained free after both calls. No disk stop was
triggered. Both versioned roots now exist and are immutable. They must not be
deleted, modified, resumed, or reused.

## Decision and next gate

Development-v4 is a valid fail-closed terminal result but not useful empirical
performance evidence. It closes the v4 execution gate as FAIL and keeps
confirmatory, paper numerical edits, new claims, and submission closed.

The smallest responsible next task is an evidence-grounded audit of the
frame-zero reset witness for UAV 3. A new version may be considered only after
that audit distinguishes an implementation/sign/lifecycle defect from genuine
bounded distributed-QP infeasibility or excessive reset conservatism, supplies
RED/GREEN regression evidence, and receives independent C0/I0 review. No v5
protocol, authorization, root, or run is created by this report.
