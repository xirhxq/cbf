# CBF2026 MC-First Numerical Revision Design

- Date: 2026-07-28
- Status: approved conversational design, written before implementation
- Code baseline: `b9d381d4c1a0ed31e025eeb0f00dfae4037e4d4a`
- Branch/worktree: `codex/cbf2026-diagnostic` in `/private/tmp/cbf2026-diagnostic`

## Goal and claim boundary

The first numerical revision gate will test whether a minimally corrected
velocity-command controller can satisfy localization-connectivity and collision
constraints under time-varying FIM-derived uncertainty while respecting
explicit command limits.

The continuous-time theorem remains conditional on valid uncertainty/rate
bounds and QP feasibility. Monte Carlo experiments will support those
conditions over a declared operating distribution; they will not be described
as a proof of global feasibility.

This iteration will not add a hybrid theorem or controller reserve for positive
uncertainty jumps, and it will not add a reserve for the difference between a
received previous neighbor command and the newly applied command. Both effects
will be measured in the diagnostic output so that their empirical scale and
relationship to any practical-tolerance violation remain visible.

## Controller architecture

Each control iteration uses a consistent two-phase uncertainty snapshot:

1. Exchange positions, held velocity commands, covariances, and uncertainty
   rates from the previous iteration.
2. Every UAV recomputes its covariance from that same snapshot and computes
   the positive backward-difference uncertainty rate
   \[
   \nu_i^k
   =
   \max\left(0,\frac{\epsilon_i^k-\epsilon_i^{k-1}}{\Delta t}\right).
   \]
   The first valid covariance sample has rate zero.
3. Exchange the updated covariance and uncertainty rate.
4. Construct all hard and soft CBF rows, solve, and advance the
   single-integrator state.

The fixed communication row uses the received zero-order-held neighbor
velocity and subtracts \(k(\nu_i+\nu_j)\). Collision constraints support two
configurable modes:

- `minimum`: one analytical row for the currently smallest robust pair;
- `pairwise`: one analytical row for every other UAV.

Hard communication and collision rows use the linear class-\(\mathcal K\)
function \(\alpha(h)=0.1h\) in every new experiment. The distance/yaw task CBF
and the implemented linear slack penalty are held unchanged in this first gate
to avoid mixing a task-controller redesign with the safety diagnosis.

## Explicit input limits

The bounded cases include the following hard linear rows inside the QP:

\[
-25\le u_{i,x}\le25,\qquad
-25\le u_{i,y}\le25,
\]

\[
-0.35\le\omega_i\le0.35.
\]

The planar values are the researcher-selected exploratory component limits.
The yaw-rate value is taken from the existing full-simulator bridge
configuration and is an exploratory numerical limit, not a hardware
certificate. No post-solve clipping is allowed.

## Diagnostic cases

All cases use the same initial state, seed, solver, time step, linear hard-CBF
class-\(\mathcal K\), and uncertainty-enabled geometry.

| Case | Positive uncertainty rate | Input limits | Collision mode | Purpose |
| --- | --- | --- | --- | --- |
| `C0` | off | off | minimum | corrected code-path control |
| `R` | on | off | minimum | isolate uncertainty-rate compensation |
| `RB` | on | on | minimum | test bounded-input feasibility |
| `RBP` | on | on | pairwise | test final explicit-pair semantics |

The first execution is a 20 s smoke for `C0`, `R`, `RB`, and `RBP`, in that
order. No 250 s, 500 s, or Monte Carlo batch starts during this gate.

## Measurements

Every run must preserve enough evidence to derive:

- solver status, hard-row residuals, row counts, and solve time;
- nominal and uncertainty-tightened localization/collision margins;
- applied \(L_2\) and \(L_\infty\) commands, saturation, and bound violations;
- per-frame minimum required \(L_\infty\) bound and the headroom to 25 m/s;
- minimum observed \(\lambda_{\min}(\Phi_i)\), maximum \(\epsilon_i\), and
  maximum positive one-step uncertainty rate;
- positive uncertainty jumps and optional-anchor removal counts when the
  available log fields permit them;
- projected previous-command mismatch
  \[
  \left|n_{ij}^{T}(u_j^k-u_j^{k-1})\right|
  \]
  reconstructed from consecutive frames;
- pairwise collision-row coverage;
- coverage fraction and loop/solver runtime.

Metrics that cannot be supported by the current logs must be marked
`unavailable`; they must not be synthesized from unrelated quantities.

## Smoke gate

A case is technically invalid and stops later cases if any of the following
occurs:

- simulator or runner failure;
- non-finite state, control, covariance, uncertainty, or rate;
- non-optimal control is applied;
- a logged hard-QP residual is below `-1e-7`;
- an explicit input bound is exceeded by more than `1e-7`;
- the expected pairwise row coverage is absent in `RBP`;
- disk or cache guard termination.

Geometric constraint results use a predeclared practical reporting tolerance:

- any tightened margin below `-0.5 m` fails;
- two consecutive negative frames fail;
- an isolated tightened margin in `[-0.5 m, 0)` is recorded as a practical
  tolerance event rather than a continuous-time guarantee.

The smoke report must show strict minima as well as tolerance classification.

## Disk and evidence policy

- Require at least `8,000,000,000` available bytes before each run.
- Terminate a running case below `6,000,000,000` available bytes.
- Cap the new diagnostic output root at `2,000,000,000` allocated bytes.
- Cap each run bundle at `250,000,000` allocated bytes.
- Preserve materialized configuration, hashes, source snapshot, manifest,
  compact summary, and raw trajectory until the researcher makes a separate
  cleanup decision.
- Do not duplicate raw arrays in DRA; record commands, hashes, results, and
  decision boundaries.

## Next decision after smoke

- If `R` fails technically or has a tightened margin below `-0.5 m`, stop and
  diagnose before adding bounded/pairwise conclusions.
- If `RB` is infeasible, do not interpret `RBP` as a pairwise-method failure;
  first resolve bounded-input feasibility.
- If all four cases are valid, use the paired results to decide which cases
  advance to 250 s and then to the pre-registered geometry distribution.

