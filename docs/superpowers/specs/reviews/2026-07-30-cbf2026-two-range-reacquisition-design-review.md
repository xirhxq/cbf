# CBF2026 Two-Range Reacquisition Design Review

Date: 2026-07-30

Reviewed design:
`docs/superpowers/specs/2026-07-30-cbf2026-two-range-reacquisition-design.md`

Final decision:
theory `C0 / I0 / M0`;
evidence design `C0 / I0 / M0`.

## Review scope

The theory review checked:

- the distinction between local two-range FIM rank and global mirror
  ambiguity;
- whether the private ZOH state is used only for disclosed discrete branch
  identity;
- `fresh`,
  conditional covariance,
  and epsilon semantics;
- exact-once private-state recursion;
- branch identity,
  threshold boundaries,
  and fail-closed behavior; and
- the permitted paper/theory claims.

The evidence review checked:

- resistance to unregistered full-grid result tuning;
- immutable Stage 1 evidence and comparator identity;
- exact raw/schema/provenance reconstruction;
- deterministic branch generation and selection;
- the two-reference cost rule;
- the exact registered key grid and no-retry lifecycle;
- the paired both-fresh p95 definition; and
- unit,
  adversarial,
  producer,
  analyzer,
  and integration coverage.

## Material findings and resolutions

The initial theory review found one Critical and two Important issues.
The private seed could still have influenced finite-iteration WNLS through
its start,
`fresh` and covariance wording hid the historical branch information,
and incoming versus outgoing private-state recursion was ambiguous.
The design now:

- permits continuous WNLS branches only from the canonical negative and
  positive circle starts;
- forbids the private,
  algebraic,
  or other non-circle starts and representatives on the new path;
- discloses the private state as historical information for discrete branch
  identity;
- limits covariance and epsilon to local modeled quantities conditional on
  correct branch selection; and
- records separate incoming branch-selection prior and outgoing private
  state fields with exact source-frame,
  propagated-frame,
  and age relations.

The initial evidence review found one Critical,
six Important,
and three Minor issues.
The design now:

- permits only unit/adversarial tests,
  synthetic fixtures,
  and one frozen mechanism key before registration;
- prohibits unregistered full-grid or equivalent partitioned execution;
- requires a new authorized round after any failed registered run;
- rejects separated circle starts whose WNLS results merge;
- records every considered accepted/rejected/unavailable attempt;
- freezes distinct protocol,
  raw,
  analysis,
  and registration schema identifiers;
- freezes the exact `140000`-row key grid;
- freezes the \(N=2\) cost rule as `cost <= 9`;
- freezes exact paired-p95 join,
  cohort,
  interpolation,
  and equality rules;
- distinguishes smoke stream hashes from timestamped manifests;
- adds numerical threshold and adversarial schema tests; and
- binds the authoritative Stage 1 v4 replay,
  analysis,
  and legacy baseline identities.

## Final boundary

The review approves the design for user review only.
It does not authorize implementation,
an unregistered replay,
the registered full-grid replay,
Stage 2 trajectory generation,
or paper-claim changes.
A separate test-first implementation plan is required after the user
approves the design.
