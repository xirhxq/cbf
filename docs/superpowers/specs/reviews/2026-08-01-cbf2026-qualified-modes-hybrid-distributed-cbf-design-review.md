# Independent Review: CBF2026 Qualified Modes and Hybrid Distributed CBF

Date: 2026-08-01

Reviewed specification:
docs/superpowers/specs/2026-08-01-cbf2026-qualified-modes-hybrid-distributed-cbf-design.md

Initial specification commit:
54ab657c2fb26dd9b74b1f1d166099076be9d3e5

Final reviewed specification:

- size: 43492 bytes;
- lines: 1037; and
- SHA-256:
  bcaa312fa8bb80dbb880dee0391035b3f2dcf9044ecdc7d3f46336ab4f74bc36.

Review method: independent read-only adversarial theory, evidence, denominator,
and implementation-contract review by the mirror-theory subagent. The reviewer
did not modify files or run experiments.

## Review scope

The review checked that the specification:

1. preserves the offline-estimator boundary, dynamic lower-index FIM DAG, two
   fixed CBF references, original diagonal FIM, coefficient-three radius,
   decentralized local QPs, and planar component bounds;
2. separates graph well-foundedness, global mode qualification, local FIM
   information, radius calibration, and hybrid distributed CBF invariance;
3. gives valid cold-start, temporal-history, allocated-row, flow-rate, and
   hybrid-reset propositions;
4. prevents truth, source-order, innovation, runtime-status, and
   process-launch selection from creating false uniqueness or incomplete
   denominators;
5. keeps the failed registered v2 roots immutable; and
6. defines falsifiable development, confirmatory, paper, DRA, disk, and
   completion gates.

## Round 1

Verdict: FAIL.

Counts:

- Critical: 3;
- Important: 3; and
- Minor: 0.

The Critical findings were:

1. the instantaneous radius-rate expression depended on the unknown current
   command and therefore could not be treated as a constant in an independent
   linear local QP;
2. the initial mode definitions applied innovation before clustering and
   required a representative only after mode admissibility, creating a
   circular order that could hide a false mode; and
3. controller failures could be omitted by admitting only selected easy flow
   intervals.

The Important findings were:

1. the reset transaction did not explicitly include the complete transitive
   descendant closure or a no-Zeno premise;
2. private-history age was finite but not frozen as an exact policy parameter;
   and
3. the ideal continuous-time proposition was not sufficiently separated from
   the 2 Hz snapshot implementation.

## Round 1 resolutions

The specification was revised to:

- derive a command-independent
  \(\bar\nu_i\) before QP solution from the componentwise speed bound,
  current geometry, predecessor \(L_j^P\) values, and the lower-index DAG;
- retain the realized \(\nu_i^{\mathrm{inst}}\) only for independent analyzer
  verification;
- use planar \(v_i=[u_{ix},u_{iy}]^{\mathsf T}\) in all endpoint rows;
- freeze the order
  local eligibility, clustering, representative selection, mode
  qualification, unique-mode publication;
- reject the complete frame when a provisional cluster is nonseparable;
- define complete estimator, initialization, and controller interval
  universes;
- add controller-certificate availability and strict mission-success
  definitions;
- make every reset a two-phase, versioned, transitive-descendant topological
  transaction;
- add a locally finite/no-Zeno reset premise and post-reset envelope premise;
- freeze
  \(K_{\mathrm{priv,max}}=H-1\), with explicit expiry and age-stratified
  outputs; and
- state that the continuous-time allocated proposition is ideal while the
  2 Hz implementation provides empirical residual evidence only.

## Round 2

Verdict: not yet PASS.

Counts:

- Critical: 0;
- Important: 1; and
- Minor: 0.

The reviewer confirmed that the command-independent \(\bar\nu_i\) recursion is
mathematically valid:

\[
\|\dot n_{ij}\|_2
\le
\frac{V_i^{\max}+V_j^{\max}}{s_{ij}},
\]

\[
|\dot w_{ij}|
\le
2\|\dot n_{ij}\|_2\|P_j\|_2+\|\dot P_j\|_2,
\]

\[
\|\dot P_i\|_2
\le
\|P_i\|_2^2\|\dot\Phi_i\|_2,
\]

and the bases-first lower-index DAG supplies a finite recursion with
\(L_j^P=\bar\nu_j=0\) for a bUAV.

The remaining Important finding was that universes and mission-success
denominators still referred to started missions. A registered process that
failed before becoming started could therefore disappear.

## Round 2 resolution

The specification was revised so that:

- every declared development mission contributes its complete frozen tuple,
  initialization, and controller-interval schedules;
- never-started and launch-failed development missions contribute missing
  estimator output, certificate-unavailable intervals, and an unsuccessful
  mission;
- the 60 confirmatory seeds define the primary mission universe before launch;
- every confirmatory seed contributes its complete frozen schedules regardless
  of launch or runtime status; and
- the confirmatory primary-mission success denominator is exactly 60.

## Final review

Counts:

- Critical: 0;
- Important: 0; and
- Minor: 0.

Final verdict: PASS.

The reviewer found no remaining theoretical, denominator, lifecycle,
distributed-realization, or evidence-integrity blocker in the written
specification.

This PASS approves the written design for researcher review. It is not
implementation evidence, a Monte Carlo result, a paper-readiness verdict, or
authorization to reinterpret the failed v2 run.
