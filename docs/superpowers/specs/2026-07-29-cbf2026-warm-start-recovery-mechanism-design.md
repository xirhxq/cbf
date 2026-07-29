# CBF2026 Warm-Start Recovery Mechanism Design

## Status and purpose

This design specifies a two-gate investigation of a replay-orchestration
failure mechanism observed after the completed localization failure audit.
The first gate is a read-only retrospective classification of the immutable
corrected Stage 2 bundle.
The second gate is a prospectively registered,
paired intervention that changes only how a UAV without a finite prior
estimate is initialized on a later frame.

The investigation remains an offline estimator replay.
It does not change the controller,
the production CBF,
the fixed distance-maintenance references,
the distributed dynamic localization DAG,
the WNLS objective,
the FIM,
or the coefficient-3 epsilon radius.
It cannot establish controller safety,
mission success,
or multi-trajectory generality.

## Observed post-hoc signal

The completed failure-mechanism audit grouped all non-upstream invalid
attempts into `invalid_input_or_numeric`.
Read-only source tracing and limited row inspection subsequently identified a
more specific candidate chain:

```text
frame-zero WNLS nonconvergence
  -> result contains estimate=null
  -> the next-frame warm start reads that null value
  -> deployment fallback is not selected because the estimate key exists
  -> WNLS input validation fails
  -> the UAV never acquires a finite estimate
```

The aggregate counts are exactly consistent with full-trajectory persistence:

| Graph case | Frame-zero WNLS failures | Later invalid-input rows | Product |
| --- | ---: | ---: | ---: |
| Dynamic DAG | 31 | 15,469 | \(31\times499\) |
| Fixed references | 29 | 14,471 | \(29\times499\) |

These values were inspected before this design was frozen.
They are therefore hypothesis-generating,
not confirmatory evidence.
The first gate must classify the complete immutable stream before any
intervention result is generated.

## Research questions

The primary mechanism question is:

> Does deterministic deployment-position restart for a UAV that has never
> acquired a finite estimate remove persistent malformed-initialization
> failures under otherwise unchanged replay conditions?

The hierarchical secondary question is:

> If direct malformed-initialization failures decline,
> does the number of downstream `invalid upstream UAV reference` attempts also
> decline?

The investigation also checks whether newly recovered estimates worsen
conditional calibration.
It does not ask whether the dynamic graph is superior to the fixed graph.

## Alternatives considered

### Directly patch and rerun

This is fast,
but it combines diagnosis and intervention.
An improvement would be harder to attribute because the compound
`non-finite or malformed WNLS input` label covers several validator branches.
This option is rejected.

### Read-only classification followed by one minimal intervention

The existing raw stream is first classified using temporal predecessor state.
Only if the missing-prior-estimate mechanism is dominant is a new replay
launched.
The intervention changes one initialization decision and preserves every
scientific input.
This option is selected.

### Availability-aware optional-reference reselection

Dropping or replacing an unavailable optional localization reference could
reduce propagation,
but it changes the graph policy and the effective information set.
It cannot isolate the empty warm-start mechanism.
This option is deferred to a separate design if restart recovery does not
reduce upstream unavailability.

## Immutable baseline

The sole baseline evidence bundle is:

```text
/private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288
```

Its manifest,
summary,
summary Markdown,
and compressed process stream must be verified against their recorded hashes
before either gate.
They must also be hashed again after each read.
The investigation must not edit,
copy into an intervention bundle,
relabel,
or overwrite this baseline.

The conceptual paired cells are:

```text
dynamic DAG x immutable baseline
dynamic DAG x deployment restart
fixed references x immutable baseline
fixed references x deployment restart
```

Only the two deployment-restart cells are newly replayed.
The baseline cells come from the completed immutable bundle.
This avoids duplicate raw evidence and prevents selection between multiple
baseline reruns.

## Gate 1: read-only temporal classification

### Component boundary

Create a standalone streaming analyzer:

```text
scripts/diagnostics/analyze_initialization_persistence.py
```

It consumes the immutable bundle and writes no files unless an external
output directory is explicitly supplied.
It must not import or call the WNLS solver.
It processes `calibration.jsonl.gz` one raw line at a time,
verifies the compressed and decompressed hashes,
and retains only the immediately preceding state for each
`(seed, graph_case, robot_id)`.

### Frozen categories

Every attempt with
`attempt_failure_reason == "non-finite or malformed WNLS input"` is assigned
to exactly one of:

| Category | Predicate |
| --- | --- |
| `prior_self_estimate_missing_or_malformed` | The preceding row for the same seed, graph case, and UAV has no finite two-dimensional estimate |
| `recorded_measurement_missing_or_nonfinite` | The current row contains a missing or non-finite recorded noisy range |
| `unresolved_compound_validator` | Neither of the above predicates is supported by the recorded fields |

The first matching predicate wins in the order shown.
The analyzer reports overlap counters so that a row satisfying more than one
predicate is visible rather than silently hidden.
It does not infer unrecorded covariance shapes or claim to identify an
internal validator branch that the bundle cannot reconstruct.

The output includes counts by graph case,
seed,
frame,
squad-local depth,
preceding attempt status and reason,
and compact example keys.
It also reports exact reconciliation to the
`invalid_input_or_numeric` totals from the completed failure audit.

### Gate

The deployment-restart intervention proceeds only if,
separately in both graph cases:

1. every baseline `invalid_input_or_numeric` row is classified once;
2. at least 95% are
   `prior_self_estimate_missing_or_malformed`;
3. no source-integrity,
   ordering,
   reconciliation,
   non-finite JSON,
   or disk-limit error occurs; and
4. an independent review finds no Critical or Important issue.

If the 95% dominance gate fails,
the intervention is not launched.
The unresolved categories become the subject of a new design.

Gate 1 is explicitly retrospective and descriptive.
It supports mechanism prioritization,
not a causal paper claim.

## Gate 2: deployment-restart intervention

### Initialization policy

The baseline policy is identified explicitly as
`strict_previous_v1`.
The intervention policy is
`deployment_restart_when_no_finite_prior_v1`.

For each UAV,
the intervention initial estimate is:

```text
frame zero                         -> configured deployment position
finite valid preceding estimate   -> preceding estimate
no finite valid preceding estimate -> configured deployment position
```

The deployment position is a fixed initial guess from the existing
configuration.
It is not the current truth position and does not inject future or oracle
information.

The restart changes only the initial estimate supplied after reference-input
validation succeeds.
An unavailable required reference still short-circuits the attempt exactly as
before.
The intervention does not drop,
replace,
or fabricate any base or UAV reference.

### Frozen scientific inputs

The intervention must reuse:

- the exact baseline truth trajectory;
- the exact configuration;
- the same 20 range-noise seeds;
- the same stable measurement-seed derivation;
- the same ranging sigma;
- the same dynamic and fixed graph-case definitions;
- the same lower-index,
  same-squad DAG restriction;
- the same fixed reference membership;
- the same WNLS objective,
  residual Jacobian,
  weights,
  damping,
  tolerances,
  iteration limit,
  and acceptance rules;
- the same final localization FIM; and
- the same
  \(\epsilon=3\sqrt{\lambda_{\max}(\Phi^{-1})}\)
  definition.

The two fixed CBF distance-maintenance references remain required by the
paper's controller semantics.
This offline replay does not alter or execute the controller.

### Provenance

The replay must record:

- a new implementation identifier;
- the unchanged estimator-contract identifier;
- the initialization-policy identifier;
- the immutable baseline bundle path and hashes;
- the intervention source commit and source archive hash;
- configuration and truth hashes;
- the exact seed list;
- disk-limit constants;
- start,
  minimum-live,
  and end free-space probes; and
- terminal completion or failure status.

The intervention is a new evidence bundle.
It must never be presented as a corrected version of the immutable baseline.

## Outcomes and analysis

### Statistical unit

The statistical unit is one range-noise seed.
Rows,
frames,
UAVs,
and DAG edges are correlated repeated observations and must not be treated as
independent samples.

All 20 registered seeds are run once.
Seeds are not added,
removed,
or rerun based on observed results.

### Primary outcome

For the dynamic DAG case,
the primary outcome is the paired seed-level difference in the fraction of
primary rows classified as `invalid_input_or_numeric`:

\[
\Delta_{\mathrm{direct},s}
=
p^{\mathrm{restart}}_{\mathrm{direct},s}
-
p^{\mathrm{baseline}}_{\mathrm{direct},s}.
\]

Report every seed-level value,
the aggregate percentage-point difference,
the median paired difference,
and a two-sided percentile bootstrap 95% interval over the 20 paired seeds
using 10,000 resamples and fixed seed 20260729.

The primary mechanism gate passes only if:

1. the bootstrap interval lies entirely below zero; and
2. the aggregate dynamic direct-invalid count falls by at least 90%.

Both criteria are frozen before the intervention run.

### Hierarchical secondary outcome

Only if the primary gate passes,
evaluate the paired seed-level change in
`upstream_unavailable`.
Report the same effect-size summaries and interval.

Evidence that restart interrupted the observed cascade requires the interval
for the dynamic upstream-unavailable difference to lie entirely below zero.
If direct invalids decline but upstream unavailability does not,
the design supports recovery of the local initialization mechanism only,
not a cascade explanation.

The fixed-reference case is a prespecified descriptive comparator.
It is not a second confirmatory primary test.

### Calibration safeguards

For both graph cases,
report:

- converged-attempt count;
- WNLS nonconvergence count;
- conditional coefficient-3 epsilon containment;
- conditional \(q>5.991\) and \(q>9\) rates;
- error-to-epsilon-ratio quantiles;
- maximum consecutive direct-invalid,
  upstream-unavailable,
  and WNLS-nonconvergence streaks; and
- restart attempts,
  restart convergences,
  and restart failures by reason.

The intervention is not advanced to multi-trajectory validation if,
in the dynamic case:

- conditional \(q>9\) rises by more than 2 percentage points; or
- conditional coefficient-3 epsilon containment falls by more than
  2 percentage points.

These safeguards prevent “more estimates returned” from being reported as
“better localization” when the recovered estimates are poorly calibrated.
They do not validate the radius.

### Claim boundary

A passing single-trajectory intervention may support only:

> Under the preserved trajectory and registered paired noise seeds,
> deployment restart recovered a replay initialization failure mechanism and,
> if the hierarchical secondary gate also passes,
> reduced its observed downstream unavailability.

It does not support:

- graph superiority;
- controller,
  collision-avoidance,
  connectivity,
  or mission guarantees;
- coefficient-3 epsilon sufficiency;
- production estimator robustness; or
- generalization across trajectories or geometries.

Any paper claim upgrade requires a separately registered multi-trajectory
stage with at least five independently generated trajectories and the
trajectory,
not the row,
as the external-validity level.

## Output and disk contract

Gate 1 writes at most:

```text
initialization-persistence.json
initialization-persistence.md
```

Its run allocation is capped at 10,000,000 bytes.

Gate 2 uses the established experiment limits:

- at least 8,000,000,000 available bytes before launch;
- immediate controlled stop below 6,000,000,000 available bytes;
- at most 2,000,000,000 allocated bytes in the experiment root; and
- at most 250,000,000 allocated bytes for the intervention bundle.

The paired comparison writes at most:

```text
warm-start-recovery-comparison.json
warm-start-recovery-comparison.md
```

Its run allocation is capped at 10,000,000 bytes.

Free-space checks occur before allocation,
at least every 10,000 streamed rows,
before every persistent write,
and after atomic publication.
Partial output is staged in a collision-resistant sibling directory and
atomically renamed only after all checks pass.

If launch space is below 8 GB,
only the separately authorized exact rebuildable cache
`/Users/xirhxq/.cache/codex-runtimes`
may be measured and removed.
No evidence,
Git object,
user document,
or unrelated cache may be deleted.
If that cleanup does not restore the launch threshold,
the run does not start.

## Error handling

Both gates fail closed on:

- source hash mutation;
- duplicate,
  missing,
  or out-of-order process keys;
- unknown graph cases or seed sets;
- non-finite JSON values;
- incomplete category reconciliation;
- missing baseline provenance;
- output-path overlap with an input bundle;
- output or disk-cap violation; or
- an unexpected exception.

A controlled failure preserves the immutable input,
closes gzip output safely,
records the failure reason in the incomplete manifest when possible,
and does not register a completed evidence directory.
Production evidence is never automatically retried.

## Testing and review

Implementation follows test-first development.
Required tests include:

1. a synthetic frame-zero WNLS failure followed by 499 missing-prior rows;
2. classifier reconciliation,
   overlap,
   unresolved,
   ordering,
   source-mutation,
   and non-finite-input failures;
3. baseline policy preserving the existing null warm-start behavior;
4. restart policy selecting deployment only when no finite prior exists;
5. both policies selecting the previous estimate when it is finite;
6. upstream-unavailable reference input still short-circuiting before WNLS;
7. identical active and fixed reference membership across policies;
8. identical measurement noise for every paired key;
9. exact preservation of WNLS,
   FIM,
   epsilon,
   and graph-selection constants;
10. paired-analysis denominator,
    seed,
    bootstrap,
    hierarchy,
    and safeguard checks;
11. output-overlap,
    atomic-publication,
    8 GB launch,
    6 GB live,
    2 GB root,
    250 MB replay,
    and 10 MB analysis caps; and
12. a compact end-to-end fixture covering both graph cases and both policies.

Before any production run:

- the implementation and complete test suite must pass;
- the intervention run protocol must be committed;
- source identity and inputs must be frozen;
- an independent code and protocol review must report no Critical or
  Important issue; and
- disk preflight must pass.

After the one production run,
an independent evidence review must reproduce hashes,
counters,
paired outcomes,
gates,
and claim boundaries before any manuscript change.

## Documentation and repository scope

The diagnostic work remains on
`codex/cbf2026-diagnostic`.
The existing untracked `build-diagnostic/` directory is neither staged nor
deleted.

Scoped commits may include:

- this design;
- the later implementation plan and run registration;
- diagnostic source and tests;
- compact reviewed reports; and
- the exact CBF2026 DRA records.

The DRA stays on `main`.
Only CBF2026-specific status,
open-question,
source,
timeline,
and experiment-log files may be changed.
Unrelated paper records and existing dirty work are preserved.

The paper is not changed by Gate 1.
It is changed after Gate 2 only if the intervention,
paired analysis,
and independent evidence review justify a narrowly scoped methods or
limitations correction.
