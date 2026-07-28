# CBF2026 Minimal Diagnostic Experiment Design

Date: 2026-07-27

## Purpose

The first revision stage is a diagnostic gate rather than a new Monte Carlo campaign.
Its purpose is to determine whether the archived CBF2026 implementation can support the manuscript's safety, localization-connectivity, bounded-input, and real-time claims before theory or large experiments are revised.

The diagnostic must distinguish:

1. historical behavior produced by the archived configuration;
2. behavior when the safety and constraint-checking switches match the manuscript claims;
3. failures caused specifically by uncertainty tightening rather than by the search controller or solver infrastructure.

No second-order or HOCBF contribution will be imported from the later research branches.
Those branches support other papers and are outside this revision's scope.

## Repository Isolation

The authoritative code starting point is commit `47e7b3d`, referenced by both `cbf2026` and `cbf2026-baseline`.

The diagnostic work is isolated in:

- branch: `codex/cbf2026-diagnostic`
- worktree: `/private/tmp/cbf2026-diagnostic`

The main checkout remains on `cbf-research-2026`.
Its saved work remains in `stash@{0}` and must not be applied, dropped, or rewritten during this stage.

The paper repository will not be edited during the diagnostic gate.
A separate paper worktree will be created only after the evidence and theoretical correction path are selected.

## Disk-Space Policy

All experiment commands must check free space on the data volume before building and before starting each run.

- Hard floor: 6 GB available.
- Start threshold: 8 GB available.
- A command must abort before generating new data if the start threshold is not met.
- A running batch must stop before launching its next case if free space is at or below the hard floor.

Before creating any output directory, the runner checks the nearest existing
ancestor against the 8 GB start threshold.
The output root must resolve outside the project root.
An equal or nested path is rejected before disk probing or allocation, which
prevents run artifacts from entering the source snapshot or contaminating its
Git dirty-state semantics.
Each invocation atomically allocates a collision-resistant run root at
`<output-root>/<case>/<runner-run-id>/`.
The materialized configuration, runner logs, manifest, source snapshot, and the
simulator's timestamped `data.json` child all remain inside that run root, so a
same-case rerun cannot replace earlier provenance.
Before simulator launch, the runner creates a standard-library `tar.gz` source
snapshot containing the tracked and untracked source/configuration/scripts/tests
needed to reconstruct the working tree.
The archive excludes Git metadata, build products, raw output, hidden SDD
coordination, and `docs/diagnostics/source-snapshots` to prevent recursion.
Every manifest distinguishes the base commit from working-tree dirtiness and
binds the archive by SHA-256, path, format, and a versioned exclusion-policy
label.
Once a run root has been allocated, every terminal path publishes a manifest
when the bundle remains writable.
Configuration materialization, source archiving, hashing, and log setup errors
retain intended paths plus any identities already available, use nullable
fields for unavailable hashes/solver metadata, and record
`termination_reason=runner_setup_error`.
`Popen` failure records `simulator_launch_error`.
Any live-monitor exception first terminates the child, waits no more than five
seconds, escalates to kill when needed, and records `runner_monitor_error`;
the final free-space value is `null` if its probe also fails.
The Python API returns these terminal manifests; the CLI prints them and exits
`2`.
Smoke-test data are retained only until their diagnostics are extracted.
When a smoke run has no evidential value, only its manifest, standard output, standard error, and compact diagnostic summary are retained.
No raw result, source evidence, Git object, or stash may be deleted automatically.

## Preconditions and Test-First Corrections

Before interpreting any simulation, regression tests will expose the following archived implementation issues:

1. the distributed QP nominal-control vector is not initialized;
2. nested class-\(\mathcal K\) parameters are read using literal keys such as `alpha/coe`;
3. a failed solve can return a zero vector that is consumed before the solver status is handled;
4. the collision-avoidance implementation uses one minimum barrier per UAV rather than an explicit barrier for every UAV pair;
5. the optimizer has no control-input bounds although the manuscript assumes bounded inputs.

Only experiment-enabling defects will be corrected before the first diagnostic runs:

- deterministic initialization;
- zero initialization of nominal control;
- correct nested configuration parsing;
- solver status checked before applying a result;
- complete logging of solver status, solve time, hard-constraint coefficients, constants, results, and barrier values.

Pairwise collision-barrier semantics, input bounds, and time-varying uncertainty treatment are not silently changed at this stage.
They are research decisions and must remain visible in the diagnostic results.

Every correction begins with a failing focused test.
The unmodified historical behavior is preserved as a separate reference result.

## Experiment Matrix

All cases use the same specified initial positions, the same deterministic seed, the single-integrator model, the distributed controller, Gurobi, and the archived control period of 0.5 s.

Each case first runs for 20 simulated seconds.
A case proceeds to the full 500 s horizon only if it produces valid JSON, finite states and controls, and interpretable solver diagnostics.

### H0: Historical replay

Use the archived configuration without changing its safety or violation-check switches.
This case establishes what the repository actually produced when safety was disabled.
It is a provenance control and must not be reported as safety validation.

### C1: Claim-aligned configuration

Enable collision safety and constraint checking while preserving uncertainty tightening.
This case tests the manuscript-facing configuration under the archived controller semantics.

### U0: Uncertainty ablation

Keep collision safety and constraint checking enabled but disable uncertainty tightening in both collision and fixed localization-connectivity barriers.
Comparison with C1 isolates whether infeasibility or excessive control demand is introduced primarily by cascading uncertainty margins.

No randomized multi-run experiment is permitted during this gate.

## Recorded and Derived Metrics

The compact diagnostic report must include:

- Git commit, branch, configuration hash, seed, solver, start time, end time, and termination reason;
- coverage trajectory and completion time, if completed;
- solver status counts and per-QP solve-time distribution;
- non-finite state, control, covariance, and uncertainty counts;
- hard-CBF linear residual \(a_k^\top u + b_k\), including minimum and negative count;
- minimum nominal and uncertainty-tightened localization margin for every required reference link;
- minimum nominal and uncertainty-tightened separation for every UAV pair;
- control \(\ell_2\) and \(\ell_\infty\) norms;
- per-step minimum \(\ell_\infty\) control bound required to make all logged hard half-space constraints feasible;
- uncertainty radius range and one-step growth \(\Delta\epsilon_i/\Delta t\);
- agreement or disagreement between configured and instantiated class-\(\mathcal K\) parameters.

The current simulator has no distinct truth and estimated state.
Therefore RMSE, NEES, ANEES, estimator consistency, or error-bound coverage must be reported as unavailable rather than inferred from nominal trajectories.

## Gate Criteria

The diagnostic gate fails if any claim-aligned run exhibits:

- invalid or non-finite output;
- a non-optimal solver status whose control is nevertheless applied;
- hard-CBF residual below the numerical tolerance;
- violation of a required localization link or pairwise safety condition;
- a required control bound incompatible with a defensible UAV velocity limit;
- unbounded or discontinuous uncertainty growth that the current CBF derivative does not cover.

If the gate fails, the next work item is theory-and-implementation repair, not Monte Carlo expansion.

If the gate passes, the next experiment stage begins with five deterministic seeds.
Twenty-run Monte Carlo and Gazebo reruns are considered only after the five-seed gate and after an estimator validation design exists.

## Theory Decision After Diagnosis

The expected theory repair is deliberately conditional on the measured failure mode:

1. Replace the inverse-FIM-as-true-covariance claim with either an estimator-supplied certified error bound or a calibrated covariance surrogate with an explicit inflation assumption.
2. Include uncertainty-rate effects through an explicit \(\dot\epsilon\) term or a sampled-data one-step growth reserve.
3. Define the bounded admissible control set in the QP.
4. Express joint hard-constraint feasibility through a computable feasibility margin over that admissible set and state the invariance result conditionally on nonempty feasible control sets.
5. Replace the single minimum collision barrier with explicit pairwise constraints if the diagnostic confirms the semantic mismatch.

The single-integrator model will be presented as a reference-command layer.
The revision will not claim nonlinear UAV invariance unless a separate derivation and appropriate validation are added without conflicting with the second-order paper.

## Paper Update Boundary

No manuscript claim, number, table, or figure will be changed from a diagnostic run alone.

After theory and implementation agree, the paper revision must:

- remove unsupported 99.7% two-dimensional confidence statements;
- separate CRLB, estimator covariance, and certified error-bound concepts;
- state whether localization uses only reference neighbors or all neighbors;
- include bounded-input and joint-feasibility assumptions;
- report safety as enabled in every safety-validation configuration;
- distinguish numerical truth-level tests from estimator-in-the-loop and Gazebo evidence;
- report unavailable metrics honestly until they are generated by an estimator-aware experiment.

## Deliverables

The diagnostic stage produces:

1. focused regression tests;
2. three versioned diagnostic configurations;
3. a disk-guarded, collision-resistant per-run bundle runner;
4. one source-bound manifest, source snapshot, and compact metric summary per
   retained run;
5. a cross-case diagnostic report;
6. a recommendation selecting the smallest defensible theory and experiment revision.

No Git commit, push, paper edit, or large experiment batch is authorized by this design.
