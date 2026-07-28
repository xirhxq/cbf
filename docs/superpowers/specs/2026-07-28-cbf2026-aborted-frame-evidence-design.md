# CBF2026 Aborted-Frame Evidence Classification Design

## Problem

The retained `RGP` run aborts during the distributed optimization loop at
frame 3 after UAV 11 reports `inf_or_unbounded`.  `Swarm::run` optimizes robots
sequentially, catches the exception, logs the current state, and does not call
`stepTimeForward`.  Thus:

- UAVs 1--10 have fresh optimal solutions that were never applied;
- UAV 11 has a fresh failed solution;
- UAVs 12--14 retain stale optimal records from frame 2 and were not
  attempted; and
- no frame-3 command was applied to the state.

The current analyzer labels every per-robot `success/optimal` record as
applied.  It therefore overstates solver attempts, applied controls, hard-row
residuals, and pairwise-row coverage on an aborted frame.

## Decision

Correct the analyzer without changing the simulator, raw bundle, manifest, or
controller.

For a distributed frame containing an explicit failed/non-optimal robot
record:

1. preserve state, geometry, covariance, uncertainty, and margin measurements
   at that runtime;
2. classify robot records through the first failure in logged robot order as
   fresh attempts;
3. classify later robot records as not attempted/stale;
4. classify the entire frame as unapplied, including fresh optimal records
   before the failure; and
5. exclude the aborted frame from applied control norms, applied hard
   residuals, applied feasibility bounds, control-mismatch propagation, and
   applied pairwise-row coverage.

Expose explicit applied-frame, partial-frame, fresh-attempt, and not-attempted
counts so the classification is auditable.  Keep the assumption narrow:
inference from record order is valid for the current sequential
`execution-mode=distributed` loop.  Do not silently apply it to centralized
execution.

## RGP acceptance truth

Reanalysis of the preserved raw data must report:

- four logged states but three applied/completed control frames;
- 52 fresh optimal solver attempts, one fresh failed attempt, and three
  not-attempted/stale records;
- 42 applied optimal controls;
- 630 applied hard rows with no residual below \(-10^{-7}\);
- 546 applied directed pairwise rows (three complete frames);
- frame 3 as an aborted/unapplied partial frame with 10 fresh optimal,
  one failed, and three not attempted; and
- 143 fresh attempted pairwise rows in the partial frame, without promoting
  them to applied coverage.

The fixed information-set, covariance, and state-margin measurements remain
valid for four logged states.  The local infeasibility certificate itself is
unchanged.

## Evidence boundary

This is an evidence-classification fix, not a controller fix and not a new
trajectory.  It does not make RGP pass, alter its stop decision, or establish
joint feasibility.  Future simulator work should log explicit per-iteration
attempt/applied markers, but that is outside this minimal recovery task.
