# Variable-weight WNLS mathematical review

## Review scope and provenance

This was a read-only mathematical and implementation review.
No production source, tests, evidence bundle, or paper content was edited by
the reviewer.
Stage 1 and Stage 2 were not executed.

The original final-branch review found that the replay estimator recomputed
the position-dependent variance

\[
v_j(p)=\sigma_r^2+n_j(p)^T P_j n_j(p)
\]

but used only the frozen-weight range terms \(J_d^TWr\) and \(J_d^TWJ_d\).
Consequently, its convergence test was not the stationarity condition of the
stated variable-weight objective.

The review covered these committed ranges:

- Initial correction review: `2496791..ceabaa4`
- Fix-wave re-review: `ceabaa4..f6e995d`

The exact full final source commit reviewed was:

```text
f6e995d6dc4711f0d2869636beb70f061e73063a
```

## Mathematical contract checked

For each reference \(j\), the reviewed objective defines

\[
a_j=p-r_j,\qquad
\rho_j=\lVert a_j\rVert,\qquad
n_j=\frac{a_j}{\rho_j},
\]

\[
e_j=\rho_j-\hat d_j,\qquad
v_j=\sigma_r^2+n_j^T P_jn_j,\qquad
q_j=\frac{e_j}{\sqrt{v_j}},
\]

with

\[
F(p)=\sum_j q_j(p)^2.
\]

For \(\rho_j>0\),

\[
\frac{\partial n_j}{\partial p}
=\frac{I-n_jn_j^T}{\rho_j},
\qquad
\nabla_p v_j
=\frac{2}{\rho_j}(I-n_jn_j^T)P_jn_j.
\]

Writing

\[
t_j=(I-n_jn_j^T)P_jn_j,
\]

the complete whitened-residual Jacobian row is

\[
J_{q,j}
=\frac{n_j^T}{\sqrt{v_j}}
-\frac{e_jt_j^T}{\rho_jv_j^{3/2}}.
\]

The sign and factor conventions were checked.
For the half-objective \(F/2\), the exact gradient is

\[
g=J_q^Tq,
\]

and the Gauss--Newton matrix is

\[
H_{\mathrm{GN}}=J_q^TJ_q.
\]

The damped step must therefore satisfy

\[
(J_q^TJ_q+\lambda I)\Delta p=-J_q^Tq.
\]

Trial acceptance compares the same \(F=q^Tq\), and convergence uses the exact
half-gradient norm \(\lVert J_q^Tq\rVert_2\).
An undefined direction with
\(\rho_j\leq\texttt{RANGE_EPSILON}\) is rejected rather than replaced by a
fabricated direction.

The final localization information matrix was separately checked to remain

\[
\Phi
=\sum_j\frac{n_jn_j^T}{v_j}
=J_{\mathrm{range}}^TWJ_{\mathrm{range}},
\]

with

\[
\Sigma=\Phi^{-1},
\qquad
\epsilon=3\sqrt{\lambda_{\max}(\Sigma)}.
\]

The final FIM is intentionally not the residual-Jacobian optimizer matrix
\(J_q^TJ_q\).

The audit contract was also checked:

- stable estimator identifier
  `variable_weight_nls_full_residual_jacobian_v1`;
- exact `stationarity_norm` on solver results;
- separate `attempt_stationarity_norm` in process rows;
- finite failed-attempt stationarity preserved when a prior state is retained
  as stale;
- strict JSON conversion of undefined or non-finite audit values to `null`;
- fixed-size compact summary storage remains free of retained process
  dictionaries and stationarity sample arrays.

## Initial correction review: `2496791..ceabaa4`

The analytic Jacobian, sign, factors, variable-weight Gauss--Newton terms,
exact-objective acceptance, zero-range rejection, and directional-FIM
implementation were correct.

The anisotropic finite-difference check genuinely exposed the original
defect:

```text
old frozen-weight half-gradient: [-0.32987900, 0.46985274]
exact/finite-difference half-gradient: [-0.42214845, 0.40844966]
maximum absolute discrepancy: approximately 0.09226945
```

The old-surrogate stationary-point regression used a point where the
frozen-weight gradient norm was approximately \(2.10\times10^{-16}\), while
the exact objective half-gradient norm was approximately \(1.27\times10^{-2}\).

The reviewer personally observed:

- five focused mathematical/solver tests: passed;
- complete replay-calibration suite: 34 tests passed;
- `python -m py_compile` for the replay module and its tests: passed;
- `git diff --check 2496791..ceabaa4`: passed.

Three Important issues were reported:

1. An accepted stationary trial on the final allowed iteration could still be
   returned as failed because step-size or cost-decrease conditions vetoed
   exact stationarity.
2. Existing FIM tests used line-of-sight-aligned covariance, for which
   \((I-nn^T)Pn=0\), and therefore did not protect directional FIM from
   accidental replacement by \(J_q^TJ_q\).
3. Tests did not cover a finite nonstationary failed attempt retained as stale
   through solver, process-row, and shared-settings audit evidence.

The initial assessment was `NEEDS FIXES`.

## Fix-wave re-review: `ceabaa4..f6e995d`

All three Important issues were resolved:

1. A cost-decreasing accepted trial now converges immediately whenever its
   exact half-gradient satisfies tolerance, including on the final allowed
   iteration.
2. An off-axis anisotropic regression has nonzero tangent covariance and
   nonzero residuals, checks the returned covariance against the directional
   FIM, and verifies that it differs from the residual-Jacobian Gauss--Newton
   covariance.
3. Finite failed-attempt stationarity is tested through actual stale retention
   and serialized process rows; the manifest and summary shared-settings
   estimator identifiers are explicitly asserted.

The reviewer confirmed that the production fix wave changed only the obsolete
accepted-trial step/cost veto.
The registered iteration count, damping, tolerances, ranging sigma, seeds,
graph cases, bootstrap constants, epsilon multiplier, FIM definition,
adequacy gates, strict-JSON behavior, and compact-memory design did not drift.

The reviewer personally observed:

- five targeted fix-wave tests: passed;
- complete replay-calibration suite: 38 tests passed;
- `python -m py_compile` for the replay module and its tests: passed;
- `git diff --check ceabaa4..f6e995d`: passed.

## Final verdict

No remaining mathematical, convergence, FIM-separation, stale-audit,
validation, strict-JSON, compact-memory, or parameter-drift issue was found in
the reviewed Task 6 final source.

```text
CLEAN / READY
```

This verdict covers source commit
`f6e995d6dc4711f0d2869636beb70f061e73063a`.
It does not certify any Stage 1 or Stage 2 run, because neither evidence stage
was executed during this read-only review.
