# Radius notation and environment-fix review

Verdict: **CLEAN**

Scope was limited to the five pending documentation edits.
No raw experiment bundle was rehashed; this review relies on the preceding
immutable-bundle evidence audit.

The exact registered definition remains
\(\epsilon=3\sqrt{\lambda_{\max}(P)}\), and the edited prose no longer
mislabels that radius as \(3\epsilon\).
The diff changes only that terminology, the corrected report's own byte
count and SHA-256 reference, and an environment clarification.
It does not change a registered command, bundle path, statistic, threshold,
adequacy condition, or verdict.

The environment clarification is consistent with the linked worktree:
the frozen commands use conda environment `cbf_env` with Python 3.11.12
and NumPy 1.24.4, while `.agents/AGENTS.md` is absent from the linked
worktree.
The original checkout's `.agents/AGENTS.md` is external to Git, ignored,
and specifies `.venv`, so documenting it as inapplicable to the frozen
linked-worktree execution is accurate.

Reviewed file identities:

| File | Bytes | SHA-256 |
|---|---:|---|
| `docs/diagnostics/2026-07-29-corrected-wnls-localization-calibration.md` | 20,571 | `dd410d5607b7e4ed24a785d2c8780bcbfa4051b46cacc573faf26e681b21f00d` |
| `docs/diagnostics/README.md` | 557 | `874b4ab823541425117ad6be0c9aba0e70c3a637b7b5a061129ce3c6d7fdc482` |
| `docs/diagnostics/reviews/2026-07-29-corrected-wnls-calibration-evidence-review.md` | 12,330 | `fdea3aefc5ff7413e6e4dc0dbba79b78a4eb68eec71da14f1d0dd8a3b2ba8f66` |
| `docs/superpowers/plans/2026-07-28-cbf2026-dynamic-localization-calibration-implementation.md` | 31,660 | `6afc7a19df778dba471bfe4bb169f3408786a1fb86dfe193f514b4dd8aba8a5c` |
| `docs/superpowers/plans/2026-07-29-cbf2026-corrected-wnls-calibration-run.md` | 23,436 | `c8052eaeaa832c2e375dda07dc125df100ebda423b44883037ff5ffb145d8dd1` |

`git diff --check` passes.
