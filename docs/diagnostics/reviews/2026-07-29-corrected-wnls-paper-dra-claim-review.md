# Independent cross-repository claim review for corrected WNLS evidence

## Verdict

**CLEAN / READY at the final reviewed cross-repository snapshot.**

Paper commit `62f414b84d243a53fa49136838630a4663a86ab2` and the
DRA commit `c65bdf06979989689ed02dc6fa74a064cce2fe1e` accurately
reproduce the committed corrected-WNLS evidence, preserve the failed
scientific-adequacy decision, and do not overstate the experiment.

Issues by severity:

- Critical: 0.
- Important: 0.
- Minor: 0.

This is a claim/evidence review, not a new estimator, bundle, or raw-row
review.  The numerical ground truth for this review is the independently
audited evidence committed at
`7653938eb2e370e79d01ba98fe8d67f6a8aa3495`.

## Frozen evidence

The committed evidence files match the requested immutable identities:

| File | Bytes | SHA-256 |
| --- | ---: | --- |
| `docs/diagnostics/2026-07-29-corrected-wnls-localization-calibration.md` | 20,223 | `92e71a8642b67a88e8aceddd8b70052afbce3549c66d9f3b660b5b68c5b0d3e2` |
| `docs/diagnostics/reviews/2026-07-29-corrected-wnls-calibration-evidence-review.md` | 11,744 | `1aa9b2d279f56f28227461133de1d22db05dc70b09ce405e557aa93e756e76cb` |

The working copies and the blobs at commit `7653938` have the same hashes.
The evidence review records `Evidence/report integrity: PASS` and
`Scientific adequacy: FAIL`; those two decisions remain separate everywhere
reviewed below.

The provenance statements are also exact.  Commit
`f6e995d6dc4711f0d2869636beb70f061e73063a` freezes the corrected executable
source and is an ancestor of registration commit
`2b4b5ac266d05f3f3ebb28b8129d8ec9876fd588`.
Only the registration plan and tracked mathematical review differ between
those commits.  The old Stage 1 and Stage 2 bundles remain present under
`/private/tmp/cbf2026-localization-calibration`; the old Stage 2 is preserved
at the exact path identified by the evidence report and is excluded from
objective-correct estimator adequacy.

## Numerical and unit audit

Every corrected number introduced into the paper and DRA agrees with the
committed report and its independent raw-row review:

| Quantity | Dynamic DAG | Fixed references |
| --- | ---: | ---: |
| Primary tuples per graph case | 139,720 | 139,720 |
| Primary containment | 62,516 / 139,720 = 44.7438% | 41,913 / 139,720 = 29.9979% |
| Seed-bootstrap 95% CI | [34.2591%, 54.5785%] | [27.4098%, 32.5809%] |
| Minimum squad-local-depth containment | 2,923 / 19,960 = 14.6443% | 100 / 19,960 = 0.5010% |
| Invalid-attempt rate | 66,367 / 139,720 = 47.5000% | 57,385 / 139,720 = 41.0714% |
| Failed-attempt rate | 6,311 / 139,720 = 4.5169% | 184 / 139,720 = 0.1317% |

The denominators use every primary per-UAV/per-frame tuple in the
corresponding graph case, exactly as preregistered.  Frame zero is retained
in the 280,000-row integrity total but excluded from the 279,440 primary
rows, leaving 139,720 primary rows per graph case.  The confidence intervals
are correctly labeled as seed-level bootstrap intervals, not intervals from
treating robot-frame tuples as independent.

The paper and DRA correctly state that one of five preregistered conditions
passes (complete rows with no silent drops) and four fail:

1. 44.7438% is below the 98% dynamic aggregate threshold;
2. 14.6443% is below the 95% every-depth threshold;
3. 47.5000% is worse than 41.0714% for invalid attempts; and
4. 4.5169% is worse than 0.1317% for failed attempts.

The overall scientific-adequacy decision is therefore **FAIL**, while the
row/report integrity decision remains **PASS**.

## Paper review

Reviewed repository:
`/Users/xirhxq/Documents/Clones/cbf/papers/CBF2026`, tracked HEAD
`62f414b84d243a53fa49136838630a4663a86ab2`.

The paper commit changes only `main.tex`; its diff is confined to the
corrected calibration discussion and its conclusion summary.  It writes the
WNLS objective with
position-dependent variance, explicitly says that the corrected
implementation differentiates the whitened residuals of the full
position-dependent objective, applies the `1e-9` residual--Jacobian
stationarity gate, and keeps the localization FIM separate from the
estimator Gauss--Newton matrix.  This matches the corrected estimator
contract.

The result table uses the exact counts, units, confidence intervals, and
denominators above.  The interpretation is appropriately limited:

- the estimator is an offline sidecar and never enters the controller or
  simulated dynamics;
- the dynamic/fixed comparison is a paired reference-graph ablation, not an
  estimator comparison;
- all 20 noise seeds reuse one preserved 500-frame truth trajectory and are
  not independent mission trajectories;
- the localization FIM omits shared-ancestor cross-correlations;
- higher aggregate dynamic containment in this replay is explicitly not
  claimed as dynamic-graph superiority;
- the failed run does not validate the coefficient-3 radius; and
- no mission-level probability or closed-loop safety guarantee is claimed.

Reviewed paper files:

| File | Bytes | SHA-256 |
| --- | ---: | --- |
| `main.tex` | 95,964 | `66d20272fabb08d943c9d53d84d1577b423c13b4f3809f0525b4b3fd0f716faa` |
| existing working-tree `main.pdf` | 6,189,611 | `6c369c183e67f8c711836be4fac2943bb236e37fc6e2670c5525bc3a307f37b0` |

An isolated copy was built with the repository `build.sh`
(`pdflatex`, `bibtex`, `pdflatex`, `pdflatex`).
It produced a 16-page letter-size PDF, 6,213,900 bytes, SHA-256
`d0007978abcd2a65334c99fbb55c16344a76016782a07a9d6f56d7e6313590c7`.
The final log contains no unresolved citation/reference, LaTeX warning,
missing-file message, or overfull box.  The existing working-tree
`main.pdf` had the same hash before and after this isolated build and was not
overwritten.

## DRA review

Reviewed repository:
`/Users/xirhxq/Documents/Clones/doctoral-research-agent`, tracked HEAD
`c65bdf06979989689ed02dc6fa74a064cce2fe1e`.

Only these six CBF2026 files are in scope:

| File | Bytes | SHA-256 |
| --- | ---: | --- |
| `meta-log/2026-07-28-cbf2026-dynamic-localization-calibration-results.md` | 8,619 | `883423e68bbd3599f26de4d1cef6faaeed9ff7aa0b41fca874a42eef649c4d73` |
| `meta-log/2026-07-28-cbf2026-dynamic-localization-dag-calibration-design.md` | 4,304 | `8b615f42e0d8c5f1c873c320aa680f6b60a4c456140db467d1e50e33c925f9d9` |
| `papers/cbf2026/open-questions.md` | 8,184 | `c21576196619cc09cf0605169b253e632c1f239461aaefd081a675019f1b66c8` |
| `papers/cbf2026/status.md` | 13,274 | `47c3807edd98a83180fd5c18b0431b837f3f5ccecc174c50eb65ff3e2ad6409b` |
| `papers/cbf2026/theory/2026-07-28-rate-aware-robust-cbf-closure.md` | 32,342 | `4eb939a3bb03d9659cdbd9fed9749515ed474c3ef3edafceb1b7631cdd605527` |
| `papers/cbf2026/timeline.md` | 18,088 | `fab729447aa3f8fccb416a64180f8ba141b6f0082af4fcbfb14c062c2b030837` |

Commit `c65bdf0` contains exactly these six paths and no others.
The SHA-256 of every blob read directly from that commit matches the reviewed
hash in the table.

These files reproduce the evidence numbers and hashes, identify the omitted
position-dependent-weight derivative in the old solver, describe the
corrected full-objective WNLS evidence, preserve the old bundles while
excluding the old Stage 2 from objective-correct adequacy, and retain every
interpretation boundary listed in the paper review.

The bounded change since the preceding review only replaces the
pending-paper wording with the actual paper commit, source hash, isolated
build hash, 16-page result, clean build status, and unchanged user-PDF
status.  Those values match the final paper repository exactly.  No
scientific number, denominator, confidence interval, adequacy decision, or
interpretation boundary changed.

The timeline remains append-only relative to DRA HEAD: its diff still adds
only the final row at line 29 and changes no historical row.  Relative to the
preceding reviewed timeline hash
`bb759c3a3577e2bab512bc29a04a40baee7476009c1941c4f70218ce5678eb36`,
only that already-appended final row was updated, replacing the honest
pending-paper placeholder with commit `62f414b`, its reviewed build result,
and the unchanged boundary statements.

## Worktree boundaries and checks

At the final reviewed DRA HEAD, the only remaining worktree dirt consists of
eight modified
`papers/podsearch2026/*` paths, nine untracked `papers/podsearch2026/*`
entries, and three untracked JFR plan/spec documents under
`docs/superpowers/`.  They were inherited, unrelated, excluded from this
review, and untouched by the CBF2026 commit and this review.  No CBF2026 DRA
path remains dirty.  The paper repository's pre-existing modified
`main.pdf` and untracked `AGENTS.md`, `rebuttal.pdf`, and `rebuttal.tex` were
also untouched.

Fresh `git diff --check` completed with no output in the diagnostic, paper,
and DRA repositories.
