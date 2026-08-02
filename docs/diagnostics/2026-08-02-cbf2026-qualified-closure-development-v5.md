# Qualified-closure development-v5 terminal report

## Verdict

`FAIL — CONTINUOUS-FLOW LOCAL HARD QP INFEASIBLE AFTER ONE INTERVAL; ANALYSIS PREFIX AUDIT ALSO FAILED; NO RETRY`

Development-v5 is terminal and consumed. The registered runner and analyzer
were each invoked exactly once. Neither command may be rerun, resumed,
partitioned, overwritten, or reinterpreted. Confirmatory execution and every
paper numerical-evidence gate remain closed.

This result is not evidence of a large localization error, unsafe separation,
or a dynamic-reference switch. Mission-01 completed one controller interval
for the dynamic primary condition, then failed closed while rebuilding the
continuous theorem-certificate flow at frame 1. The fixed-FIM condition and
missions 02--10 were not launched. The registered analyzer subsequently found
an independent retained-prefix accounting defect and published only a terminal
failed analysis manifest.

## Registered identities

The execution source was branch `codex/cbf2026-diagnostic`, upstream `none`, at
the exact four-add-only authorization child
`9f9c8352cdd3dd32cf6c907b0e8b7f592559ce46`, tree
`1dbf14f4b39b13177eaf6439a80d3928f0d825e5`.
Its sole parent is the registered implementation/documentation identity
`55a9a783a94a1d59a60db99552a6a4421acca25b`. The child adds exactly the
development-v5 protocol JSON, protocol Markdown, independent preflight, and
authorization JSON, with no fifth path.

```text
a821e6d2abbc9a85a088892fb33413387e8233b6d67646ab346e64d5698f8123  47,677 bytes  protocol JSON
91d7cc6a01c53b7ac2627b9318dd75d7328eb401cf70c25869916715823269cf     312 bytes  protocol Markdown
fe2426d850a66f7ed086b771d7ba58b3651b473262e8977f29d437c6a029c380  13,900 bytes  independent preflight
0e92fef88049807e00e925fd52844645c78f6e9d6fb342968640ae5408853bd1     565 bytes  authorization JSON
```

The independent preflight was C0/I0/M0. The authorization binds the actual
post-preflight user text `批准`, its UTF-8 SHA-256
`8cbe697b157364a5b13646285b38409dc53ec5287deeb7913493e65b275cd14d`,
the protocol and preflight hashes above, and implementation identity
`55a9a78...`. Production postcommit authorization and exact-root validation
passed before launch. The protocol semantic SHA-256 is
`95a479b19dcf979f3df683d5dbc865cf327c9fd60cd88d2a112ea6bff1e94b81`.

The live registered binary and configurations matched these identities:

```text
02f1c30575a04194fe2d459469a36ba0e099c72d2c4979dc29bf2627fef6b63f  build-diagnostic/Swarm
0920ca21fc57e51d44b0df3c00bc32bf4d9fe5bbc7f6fa8bb826def1b88184ac  config/config.json
6a198e17b8c3bbf6777caaa325fa593dd24ee9ce3981f90769075112be1cacb3  primary config
6ad20e1e32fff5e8950f8de31864d77a238f6d418abafe3a50cdb292b8c0bc66  fixed-FIM ablation config
7fb8597cd89a41315cbd86ca87c0bc6e3ecb22ea3dcab8ee6278aca44743fe24  frozen initial-state family
```

The frozen family used registered trajectory seeds `2026080201..2026080210`.
Its independent 100-seed audit admitted 100/100 proposed states, with minimum
frame-zero barrier `35.77296640879953 m` and minimum frame-zero max-min local-QP
margin `0.7658252531927233 m/s`. Those values were an initial-state
precondition only and were not a recursive-feasibility guarantee.

## Once-only runner result

The runner used the protocol-serialized module argv with development version
v5, range-noise seeds `2026081201..2026081210`, 1,000 frames per mission, and
raw root
`/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v5`. It was
invoked once and returned nonzero. The immutable raw bundle contains 28 regular
files, no symbolic member, and 12,764,382 logical bytes. Its sorted
`relative-path<TAB>bytes<TAB>sha256<LF>` tree commitment is:

```text
7ead4049e0d13f63458ba8d1a522f055ffed56357ffa1719e1bfc1da38a66870
```

The terminal raw manifest is 1,516 bytes and hashes to
`a3970bcf588e938f8487a794b00d1bbbe8bbd181c0d5a919cc0297b258e64a32`:

```text
status                         failed
reason                         child_nonzero_exit
mission_count                  10
completed_mission_count        0
schedule SHA-256               c5a06a8cae9aca984fccfd94bcf1621bffd30ce33672d59b581ddb4f2ff1fe77
authorization SHA-256          0e92fef88049807e00e925fd52844645c78f6e9d6fb342968640ae5408853bd1
```

Mission-01 retained 250 valid ordered rows: 14 initialization rows, one
accepted reset, 232 endpoint rows, one complete frame-0 controller interval,
one incomplete frame-1 controller interval, and one failed mission terminal.
The supervisor recorded `valid_rows=250`, `missing_rows=0`, return code `1`,
and `reason=child_nonzero_exit`. Swarm stderr ends with:

```text
[SIMULATION_ERROR] continuous certificate flow produced an infeasible hard QP
```

The frame-0 reset contained exactly 119 fixed hard edges: 28 fixed
localization edges and 91 all-pairs collision edges. All 14 local hard QPs
were `optimal`. Their reconstructed max-min margins were positive; the
smallest was `0.8626134268990517 m/s` for UAV 13. Nevertheless, the actual
task-QP commands lay on the hard boundary: the minimum applied hard-row
residual over the 14 local problems was approximately
`7.63e-13 m/s`. The largest applied planar component was
`24.18197080343183 m/s`, below the registered `25 m/s` component limit.

After `0.5 s`, frame 1 failed before any new task QP was attempted. The abort
row marks all 14 nodes `not-attempted`; therefore the generic runtime message
does not itself identify the infeasible owner. The runner synthesized the
exact frozen missing-key complement for the incomplete primary condition, the
unlaunched ablation condition, and missions 02--10, then terminalized every
mission and the campaign without retry.

## Independent one-step reconstruction

The retained initialization positions and frame-0 applied commands determine
the frame-1 SingleIntegrate2D positions exactly. Recomputing the production
dynamic-lower-index FIM certificates, design radii, rate bounds, fixed hard
rows, and bounded max-min local LP at that one-step state gives:

```text
t = 0.0 s   minimum barrier 35.9554065 m; minimum local margin +0.8626134 m/s (UAV 13)
t = 0.5 s   minimum barrier 37.2493144 m; UAV 4 margin -0.01730845 m/s
t = 0.5 s                                      UAV 12 margin -0.04495827 m/s
```

All robust hard barriers therefore remained positive. The failure was a loss
of bounded local-QP feasibility, not a negative-barrier event. The tight
conflicts were collision edges `(4,10)` with `(4,13)` and `(6,12)` with
`(12,13)`. The 14 dynamic FIM reference sets were unchanged across the first
interval, so no graph-switch reset caused this counterexample.

This also confirms the graph separation required by the manuscript. The FIM
certificate graph dynamically includes in-range bases and lower local-index
predecessors. The hard localization registry remains the fixed formation
reference graph. The continuous local QPs use that fixed localization
registry plus all collision edges; dynamic FIM candidate references are not
silently promoted into distance-maintenance CBF edges.

The root cause is a contract gap: frozen-family admission checks only the
materialized frame-zero state. It does not prove that the task-QP command
selected at frame zero maps the system into another state with a nonempty
bounded local hard-QP feasible set. Positive robust barriers and frame-zero
QP feasibility do not imply recursive feasibility of the distributed local
problems.

An in-memory diagnostic sweep at the retained one-step state changed only the
linear class-K coefficient in the row formula. It found worst margins
`-0.04496`, `+0.08161`, `+1.22076`, and `+2.48648 m/s` for coefficients
`0.100`, `0.105`, `0.150`, and `0.200`, respectively. This is a hypothesis
about a standard legal tuning direction, not campaign evidence: a changed
coefficient also changes the frame-zero task-QP command, so no long-horizon
claim follows from this static sweep.

## Once-only analyzer result

Because the raw manifest was terminal, the analyzer used the exact
protocol-serialized module argv once with output root
`/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v5`.
It returned nonzero with:

```text
qualified analysis failed: synthetic missing key universe differs
```

The analysis root contains only a 176-byte terminal failed manifest, SHA-256
`82457d6a4838b2396b16fa36d99b15b6ccd0ab16823cfbf486be51613e862a63`,
with `status=failed` and `reason=analysis_exception:ValueError`. The one-file
tree commitment is:

```text
e42ce3e1cd83c60366b2901e3adec58a4282cfc70df7f9b13609a16f600020f5
```

The analyzer defect is independent of the controller failure. The runner's
missing-stream producer counts a controller interval and its 119 reconstructed
edge keys as observed only when `runtime.complete is true`. The analyzer's
retained-prefix reconstruction counted the incomplete frame-1 abort row as an
observed controller interval and also invented its 119 reconstructed keys.
It consequently expected `missing_controller` to advance from frame 1 to
frame 2, while the correctly synthesized stream still contained frame 1.

The minimal correction is to apply the same `complete is true` predicate in
the analyzer before adding controller or reconstructed observed keys, with a
real failed-prefix regression test. This can repair future lifecycle analysis
and explain v5, but cannot convert the consumed v5 campaign into a qualifying
result or authorize a second v5 analyzer call.

## Disk, immutability, and scientific boundary

The launch gate observed about 15.28 million KiB free, above the registered
8 GB floor. The raw and analysis roots occupy about 20 MB together; no disk
stop occurred. Both versioned roots now exist and are immutable. They must not
be deleted, modified, resumed, or reused.

Development-v5 supplies a valid continuous-flow recursive-feasibility
counterexample and a separate analyzer regression witness. It supplies no
long-horizon localization-error distribution, no fixed-FIM comparison, no
mission-performance estimate, and no positive implementation-level safety
evidence. Zero later-frame violations would be missing data, not success.

The next responsible lifecycle is v6, not a v5 retry. Before registration it
must: (1) fix the analyzer predicate through RED/GREEN tests; (2) choose and
freeze a scientifically legal recursive-feasibility mitigation, with an
explicit one-step viability gate rather than a weaker frame-zero threshold;
(3) independently review the implementation and protocol; and (4) receive a
new exact post-preflight user authorization. Until a terminal v6 passes, the
paper must retain conditional stacked-controller theory and must not promote
the distributed local-QP implementation to a proved safety guarantee.
