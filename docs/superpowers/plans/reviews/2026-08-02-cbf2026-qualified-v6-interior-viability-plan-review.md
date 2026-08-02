# Independent plan review — qualified-v6 interior viability

## Verdict

**NON-PASS — Critical: 0, Important: 4, Minor: 0 (C0/I4/M0).**

Task 1 must not begin.  The committed plan needs a new planning commit and a
fresh independent review; only C0/I0/M0 may advance to implementation.

## Planning identity

| Item | Observed value |
|---|---|
| planning commit / `HEAD` | `424b730cc1e5da1e1455b4efb5e223da03a50e34` |
| tree | `e8e53bf27808a351e4ae1658101f8f12efde99d5` |
| parent | `081e2fe812ddf3dd972fb972d59553d1e39e54f0` |
| branch | `codex/cbf2026-diagnostic` |
| pre-review status | `?? build-diagnostic/` |
| committed plan SHA-256 | `9d666fdf2e5e5e4986be455578ec9827718840469bf66f670b3d814e4826e3f5` |

The plan digest was recomputed from the committed blob with:

```bash
git show HEAD:docs/superpowers/plans/2026-08-02-cbf2026-qualified-v6-interior-viability.md | shasum -a 256
```

## Important findings

### I1 — Task 1 names the wrong immutable v5 raw manifest

Plan line 218 selects `campaign.manifest.json` for the raw v5 root, but the
immutable root contains `manifest.json`, matching the committed v5 terminal
report.  `_sha256(manifest)` therefore fails before Task 1 can produce its two
identity records.  The plan must correct the command; the consumed root must
remain untouched.

### I2 — The Task 2 witness assertion contradicts three-plane enumeration

Plan lines 321--332 require radius `2` and witness `(0,0)` for the symmetric
rows.  At `rho=2`, the optimal set is `u_x=0`, `u_y in [-1,1]`.  Under the
required three-plane enumeration in lines 358--365, the optimal vertices have
`u_y=-1` or `u_y=1`; `(0,0,2)` has only two active planes and is never
enumerated.  Ordinary lexicographic selection gives `(0,-1)`, not `(0,0)`.
The stated RED test and GREEN algorithm cannot both pass.

### I3 — An interrupted formal gate can be retried for the same identity

Global Constraint line 31 requires failure or interruption to be terminal.
Task 6 line 770 publishes the artifact only after execution, while Task 8 lines
926--954 use final-output absence as the launch precondition.  There is no
nonreplaceable pre-launch identity claim.  If the producer is interrupted
before publication, the artifact remains absent and the exact same committed
identity can be launched again, violating once-only/no-retry semantics.

### I4 — Task 9 validates the exact-four committed child before creating it

Plan lines 1046--1048 require the production authorization/exact-four-child
validation, but lines 1050--1064 commit those four artifacts only afterward.
The current production validator requires a committed HEAD distinct from the
registered implementation and verifies that it is the direct exact-four
add-only child.  Thus Step 6 cannot pass before Step 7, and the plan has no
postcommit validation step before root allocation.

## Scope and disposition

The local-only controller intent, fixed v1/v5 trajectory seed universe, v5-root
immutability, fresh authorization boundary, and runner/analyzer once-only intent
were reviewed, but the four findings above prevent end-to-end closure.  This
review does not modify or authorize changes to the plan, code, tests,
configuration, `build-diagnostic/`, v5 roots, or Git history.

`git diff --check` is clean for the worktree changes after creating this review.
