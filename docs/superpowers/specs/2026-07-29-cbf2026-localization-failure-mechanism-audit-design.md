# CBF2026 Localization Failure-Mechanism Audit Design

## Status and purpose

This design specifies one post-hoc exploratory audit of the already completed,
objective-correct Stage 2 localization-calibration bundle.
It does not register a new estimator experiment,
does not rerun WNLS or Monte Carlo,
and cannot validate the coefficient-3 FIM-derived epsilon radius.

The existing evidence has integrity PASS and scientific adequacy FAIL.
Its primary-row results are:

| Quantity | Dynamic DAG | Fixed references |
| --- | ---: | ---: |
| Rows | 139,720 | 139,720 |
| Contained | 62,516 (44.7438%) | 41,913 (29.9979%) |
| Invalid attempt | 66,367 (47.5000%) | 57,385 (41.0714%) |
| Failed attempt | 6,311 (4.5169%) | 184 (0.1317%) |

Read-only exploratory inspection has already observed that:

- 50,898 dynamic and 42,914 fixed primary rows have
  `attempt_failure_reason="invalid upstream UAV reference"`;
- those counts explain 7,984 of the 8,982 excess dynamic invalid attempts;
- among converged primary attempts,
  4,526/67,042 dynamic rows and 40,238/82,151 fixed rows lie outside the
  recorded epsilon radius.

Because these values were inspected before this design was frozen,
this audit is explicitly post hoc.
Its tables and classification rules are frozen to prevent further selective
reporting,
but its output remains descriptive and hypothesis-generating.

## Research question

For the one preserved trajectory and its 20 paired range-noise seeds,
how much of the failed overall containment is associated with:

1. an unavailable upstream UAV estimate;
2. another invalid input or numeric condition;
3. WNLS nonconvergence;
4. a converged estimate outside the recorded epsilon radius; or
5. a converged estimate inside the recorded epsilon radius?

The audit also asks where those categories occur by squad-local depth and
time,
whether frame-zero failures persist through the DAG,
and whether covariance calibration remains inadequate after conditioning on
a converged attempt.

It does not ask whether the dynamic graph is superior,
whether a different estimator would work,
or whether the CBF controller is safe.

## Alternatives considered

### Extend the frozen replay summary

Adding fields to `_SummarySamples` or `_summary()` would change the source
identity and output contract of the registered replay.
It could not explain the existing immutable JSONL without creating a new
bundle.
This option is rejected.

### Add a standalone streaming analyzer

A new analyzer reads the completed gzip stream once,
verifies its manifest and hashes,
keeps bounded counters and current-frame predecessor state,
and writes a compact derived report outside the evidence bundle.
It never invokes the estimator or controller.
This option is selected.

### Run counterfactual estimators or new trajectories now

Oracle-reference,
alternative-initialization,
shared-covariance,
and multi-trajectory runs could improve causal identification.
They would also introduce new method choices and multiple-comparison paths
before the present failure mechanism is localized.
They are deferred to a separately designed and registered round.

## Input and immutability contract

The sole production input is:

```text
/private/tmp/cbf2026-localization-calibration-corrected/stage2/localization-calibration/20260728T185008.982732Z_ca76d378475447068e08f0921ba87288
```

The analyzer reads:

- `manifest.json`;
- `summary.json`; and
- `summary.md`; and
- `calibration.jsonl.gz`.

It must verify:

- `termination_reason == "completed"`;
- estimator contract
  `variable_weight_nls_full_residual_jacobian_v1`;
- the recorded compressed and decompressed process SHA-256 values;
- the recorded `summary.json` and `summary.md` SHA-256 values;
- `process_rows == expected_process_rows == 280000`;
- exactly 279,440 primary keys,
  consisting of 20 seeds,
  two graph cases,
  499 frames,
  and 14 UAVs;
- every expected `(seed, graph_case, frame_index, robot_id)` key appears once;
  and
- the overall attempt-status counts reconcile to the existing summary.

Key completeness and uniqueness are checked against the replay's canonical
stream order
`frame -> seed -> graph case -> robot_id`.
The analyzer advances one expected-key cursor and never retains a
280,000-element key set.
For each graph case,
primary-row retained and attempt-status totals are compared separately with
`graph_cases[case].overall`,
while frame-zero retained and attempt-status totals are compared separately
with `graph_cases[case].initialization_frame`.

Any mismatch is an input-integrity failure.
The analyzer must stop without repairing,
rewriting,
or relabeling the source bundle.

Before and after analysis,
the source gzip,
manifest,
summary JSON,
and summary Markdown hashes must remain unchanged.

## Architecture

Create a separate module:

```text
scripts/diagnostics/analyze_localization_failures.py
```

Its public library interface is:

```python
def analyze_localization_failures(
    bundle_dir: Path,
    *,
    verify_hashes: bool = True,
    output_dir: Path | None = None,
    max_examples_per_bucket: int = 5,
) -> dict:
    """Stream and diagnose one completed localization-calibration bundle."""
```

The CLI is:

```bash
conda run -n cbf_env python -m scripts.diagnostics.analyze_localization_failures \
  --bundle-dir <completed-bundle> \
  --output-dir <external-derived-output-directory>
```

With no output directory,
the analyzer prints strict JSON to standard output and writes nothing.
With an output directory,
it writes only:

```text
failure-mechanisms.json
failure-mechanisms.md
```

The output directory must not be the source bundle,
an ancestor of the source bundle,
or a descendant of the source bundle.
The derived output is analysis,
not registered evidence.

The gzip is processed one raw line at a time.
The analyzer updates the decompressed SHA-256 from the exact line bytes before
strict JSON parsing.
It does not call `gzip.decompress`,
`readlines`,
`json.load` on the JSONL,
pandas,
or an external sort.

Memory is bounded by:

- fixed counters;
- 20 seed-level aggregates;
- two graph cases;
- seven squad-local depths;
- five fixed time bins;
- current-frame predecessor state for at most 14 UAVs; and
- at most `max_examples_per_bucket` compact example keys.

No raw-row list or decompressed copy is retained.

## Frozen row classification

Classification uses `attempt_status`,
never retained `status`.
A stale retained estimate is not counted as a successful current attempt.

Every primary row belongs to exactly one of:

| Class | Predicate |
| --- | --- |
| `contained` | `attempt_status=="converged"` and `containment==true` |
| `converged_outside_radius` | `attempt_status=="converged"` and `containment==false` |
| `upstream_unavailable` | `attempt_status=="invalid"` and reason is `invalid upstream UAV reference` |
| `invalid_input_or_numeric` | `attempt_status=="invalid"` and any other reason |
| `wnls_nonconvergence` | `attempt_status=="failed"` and reason is `maximum WNLS iterations exceeded` |
| `other_failed` | `attempt_status=="failed"` and any other reason |

Unknown reason strings are always emitted verbatim in a nested counter.
They are not silently mapped to a known cause.
At most 32 distinct raw reason labels are retained.
Additional labels increment `reason_label_overflow_count`,
with at most five verbatim overflow examples.

The six category counts must sum to the full primary denominator for each
case,
depth,
time bin,
and seed.

## Upstream lineage

The recorded DAG only references strictly lower squad-local UAV indices,
and rows are emitted in that order within each
`(seed, graph_case, frame_index)` block.

For an `upstream_unavailable` row,
the analyzer inspects measurement records whose:

```text
kind == "uav"
estimated_reference_available == false
```

It links each unavailable UAV reference to the already seen same-frame
predecessor row.
The report distinguishes:

- `upstream_unavailable_rows`,
  counting affected observer rows; and
- `unavailable_uav_reference_edges`,
  counting individual unavailable UAV measurement edges.

The compact link records:

- predecessor key;
- predecessor attempt class;
- predecessor retained status; and
- predecessor attempt reason.

If the predecessor is absent,
the analyzer records `not_observed`;
it does not infer a cause.

Recursive lineage may report propagation depth from a directly observed
non-upstream root failure.
This is a replay-order mechanism trace,
not a causal proof about the physical system.

## Frozen descriptive metrics

### Primary failure budget

For each graph case,
report the count and rate of all six mutually exclusive classes using all
139,720 primary rows as denominator.

Also report the dynamic-minus-fixed count and percentage-point difference for
every class.
No class may be omitted because its direction is unfavorable.

The contribution of upstream unavailability to excess invalidity is:

\[
D_{\mathrm{upstream}} =
\frac{N^{\mathrm{dyn}}_{\mathrm{upstream}}
      -N^{\mathrm{fix}}_{\mathrm{upstream}}}
     {N^{\mathrm{dyn}}_{\mathrm{invalid}}
      -N^{\mathrm{fix}}_{\mathrm{invalid}}}.
\]

If the denominator is non-positive,
report `not_estimable`.
Report the 20 seed-specific values and a descriptive percentile bootstrap
interval over paired seeds only,
using 10,000 resamples and RNG seed `20260729`.
Each resample draws 20 seed indices with replacement,
sums the selected dynamic/fixed upstream and invalid counts,
then recomputes the ratio from those four aggregate counts.
If a resample denominator is non-positive,
that resample is recorded as non-estimable.
The report prints estimable and non-estimable resample counts;
it reports percentile bounds only when at least 9,500 of 10,000 resamples are
estimable.
Seed-specific rows always print their four counts and use `null` when their
own denominator is non-positive.
Do not report a p-value.

Because the reason counts were inspected before design freeze,
\(D_{\mathrm{upstream}}\) is descriptive,
not a confirmatory primary endpoint.

### Conditional estimator calibration

Among rows with `attempt_status=="converged"`,
report:

- epsilon-radius containment;
- error-to-epsilon ratio counts in fixed bins
  `[0,0.5)`,
  `[0.5,1)`,
  `[1,2)`,
  `[2,5)`,
  and `[5,infinity)`,
  together with finite count,
  mean,
  and maximum;
- two-dimensional normalized squared error
  \(q=e^\mathsf{T}P^{-1}e\);
- normalized-squared-error counts in fixed bins
  `[0,2.295748929)`,
  `[2.295748929,5.991464547)`,
  `[5.991464547,9]`,
  and `(9,infinity)`,
  together with finite count,
  mean,
  and maximum;
- the fraction with \(q>5.991464547\);
  and
- the fraction with \(q>9\).

The \(q\) statistics diagnose covariance calibration.
They are not substituted for the registered circular epsilon-containment
metric.

If covariance is absent,
non-finite,
non-symmetric within `1e-12` relative tolerance,
or not positive definite,
the row is counted in a separate conditional-covariance-invalid bucket and
excluded from the \(q\) denominator.

### Initialization and persistence

Frame zero is reported separately for each graph case and depth:

- the six-class failure budget;
- upstream-unavailable observer-row count and unavailable UAV-reference-edge
  count; and
- the number of max-iteration failures.

For each `(seed, graph_case, robot_id)`,
report:

- first primary frame with a converged attempt;
- number of primary frames before first convergence;
- longest consecutive upstream-unavailable streak;
- longest consecutive WNLS-nonconvergence streak; and
- whether no primary convergence occurs.

These are persistence descriptions.
They cannot distinguish poor initialization from geometry or noise because
the frozen rows do not store the attempt's initial and terminal iterates.

### Geometry association

For converged attempts only,
report conditional containment and \(q>9\) rate by:

- squad-local depth 1--7;
- active-reference count,
  defined exactly as
  `len(active_references.base_ids)+len(active_references.uav_ids)`,
  in bins `0`,
  `1`,
  `2`,
  `3`,
  `4`,
  `5`,
  `6`,
  `7`,
  `8`,
  `9`,
  and `10_or_more`;
- FIM condition-number bins
  `[1,10)`,
  `[10,30)`,
  `[30,100)`,
  and `[100,infinity)`; and
- FIM minimum-eigenvalue bins
  `(0,0.05)`,
  `[0.05,0.2)`,
  `[0.2,1)`,
  and `[1,infinity)`.

All bins are printed,
including empty bins.
These associations do not identify geometry as a cause because reference
count,
depth,
and trajectory position co-vary.

## Fixed hierarchy and time bins

Every failure-budget metric is reported:

1. overall;
2. by squad-local depth 1--7;
3. by frames 1--100,
   101--200,
   201--300,
   301--400,
   and 401--499; and
4. in a complete dynamic-only 7-by-5 depth/time table.

The two squads are pooled at equal local depth.
No squad,
individual UAV,
seed,
edge identity,
or favorable post-convergence subset is promoted to a separate claim.

Seed-level rows remain available in the JSON for audit,
but the Markdown report prints all seeds only for
\(D_{\mathrm{upstream}}\) and overall conditional containment.

## Operational success and stopping rules

The audit is operationally successful only if:

1. source hashes and estimator contract match;
2. all 280,000 rows and 279,440 primary keys are unique and complete;
3. graph-case status totals match the frozen summary;
4. every category partition reconciles to its denominator;
5. every depth and time-bin denominator reconciles;
6. the output remains below 10 MB allocated size; and
7. source hashes remain unchanged after the run.

If any condition fails,
the analyzer records or raises an integrity failure and stops.

Run the production analyzer once on the corrected Stage 2 bundle.
Do not rerun it with changed bins,
taxonomy,
examples,
or thresholds.
Fixing a software defect requires a regression test,
an explicit defect record,
and a new derived-analysis run ID;
the failed output remains preserved.

The resulting report may rank the observed classes by contribution.
That ranking determines which mechanism deserves a separately designed
investigation;
it does not trigger an estimator,
graph,
or radius change in this audit.

## Disk and process limits

Persistent-output launch requires at least 8,000,000,000 available bytes.
The analyzer checks a 6,000,000,000-byte live floor during streaming and
before final writes.
The complete derived-analysis root is capped at 2,000,000,000 allocated
bytes,
and one run is capped at 10,000,000 allocated bytes.

The expected input read is approximately 47.6 MB compressed and 414.5 MB
decompressed in the gzip stream.
No decompressed bytes are written to disk.

If free space is below the launch threshold,
do not start the production analysis.
Only explicitly identified,
rebuildable caches may be removed.
Never delete,
move,
or overwrite experiment evidence,
user documents,
or Git history.

## Error handling

Use a dedicated `InputIntegrityError` for:

- missing or non-object manifest/summary;
- incomplete termination;
- hash mismatch;
- estimator-contract mismatch;
- malformed or non-strict JSONL;
- duplicate or missing process keys;
- row-count mismatch; and
- aggregate-count mismatch.

Use a dedicated `AnalysisLimitError` for:

- launch-space failure;
- live-space failure;
- output nesting violation;
- output-run cap; and
- output-root cap.

An unsuccessful output run may write an incomplete diagnostic record outside
the source bundle,
but it must never write a file that claims `status="completed"`.

## Output schema

The JSON top level is:

```text
schema
status
source
integrity
protocol
initialization
cases
comparisons
limitations
```

The schema identifier is:

```text
cbf2026-localization-failure-analysis-v1
```

`source` binds the input paths,
small-file hashes,
compressed/decompressed process hashes,
estimator contract,
and exact run settings.

`protocol` records all category predicates,
bin edges,
bootstrap settings,
and denominators.

`limitations` always states:

- post-hoc exploratory mechanism audit;
- one preserved trajectory with 20 paired noise seeds;
- offline estimator outside the controller;
- no shared-ancestor cross-covariance model;
- no causal estimator or graph comparison;
- no radius,
  mission-probability,
  robustness,
  or safety validation.

## Tests

Create:

```text
tests/test_analyze_localization_failures.py
```

Tests use small generated completed bundles and real gzip/JSON parsing.
They must not invoke WNLS,
the replay,
the controller,
or Monte Carlo.

Each production behavior is implemented test-first:

1. accept a complete strict bundle and reject hash,
   row-count,
   key,
   contract,
   or aggregate mismatches;
2. classify attempt rather than retained status and reconcile all categories;
3. link unavailable predecessors in stream order and surface
   `not_observed`;
4. compute hand-derived epsilon containment,
   error-ratio,
   and two-dimensional normalized-squared-error fixtures;
5. produce the fixed depth,
   time,
   reference-count,
   and FIM-bin denominators;
6. compute the fixed ratio and normalized-squared-error histogram bins with
   hand-derived boundary fixtures;
7. bound retained state,
   raw reason labels,
   and example lists independently of input row count;
8. preserve source hashes and reject nested output;
9. enforce the 8 GB launch,
   6 GB live,
   2 GB root,
   and 10 MB run caps; and
10. render deterministic strict JSON and Markdown without copying raw rows.

Every test names the production mutation it catches,
uses hand-derived literal expectations,
and exercises the real analyzer rather than mocks of its classification
logic.

## Non-goals

This audit must not:

- change or invoke `solve_wnls`;
- change FIM or epsilon calculation;
- change initialization,
  damping,
  iteration limits,
  noise,
  graph membership,
  or fixed references;
- rerun replay,
  controller,
  or Monte Carlo;
- compare estimators;
- write into an existing evidence bundle;
- delete old or corrected evidence;
- update the paper with a stronger claim; or
- claim that one observed association is the unique cause.

The next method change,
if justified,
requires a separate design,
tests,
registration,
and evidence round.
