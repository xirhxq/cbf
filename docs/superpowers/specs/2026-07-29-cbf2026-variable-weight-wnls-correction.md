# Variable-weight WNLS correction

## Scope

The localization replay estimator uses a position-dependent propagated range
variance. Its objective is therefore the nonlinear least-squares objective

\[
F(p)=\sum_j q_j(p)^2,\qquad
q_j(p)=\frac{\lVert p-r_j\rVert-\hat d_j}
{\sqrt{\sigma_r^2+n_j(p)^T P_j n_j(p)}}.
\]

This correction applies only to the replay estimator and its audit evidence.
It does not change the production localization graph, simulation seeds,
ranging sigma, graph cases, bootstrap constants, iteration or damping policy,
tolerances, epsilon multiplier, final adequacy gates, or paper claims.

## Complete residual Jacobian

For one reference, define

\[
a_j=p-r_j,\quad
\rho_j=\lVert a_j\rVert,\quad
n_j=\frac{a_j}{\rho_j},\quad
e_j=\rho_j-\hat d_j,\quad
v_j=\sigma_r^2+n_j^T P_j n_j.
\]

For \(\rho_j>0\),

\[
\frac{\partial n_j}{\partial p}
=\frac{I-n_jn_j^T}{\rho_j},\qquad
\nabla_p v_j
=\frac{2}{\rho_j}(I-n_jn_j^T)P_jn_j.
\]

Writing

\[
t_j=(I-n_jn_j^T)P_jn_j,
\]

the complete row Jacobian of \(q_j=e_j/\sqrt{v_j}\) is

\[
J_{q,j}
=\frac{n_j^T}{\sqrt{v_j}}
-\frac{e_j t_j^T}{\rho_j v_j^{3/2}}.
\]

The damped Gauss--Newton step uses

\[
(J_q^TJ_q+\lambda I)\Delta p=-J_q^Tq.
\]

Trial acceptance compares the same \(F(p)=q^Tq\), and convergence uses the
exact objective half-gradient norm

\[
\lVert J_q^Tq\rVert_2\leq 10^{-9}.
\]

A range \(\rho_j\leq\texttt{RANGE_EPSILON}\) has no defined direction or
derivative and is rejected. No substitute direction is fabricated.

## Final localization FIM

The estimator Gauss--Newton matrix is not the localization FIM. After a
stationary estimate is obtained, covariance and radius continue to use

\[
\Phi=\sum_j\frac{n_jn_j^T}{v_j},\qquad
\Sigma=\Phi^{-1},\qquad
\epsilon=3\sqrt{\lambda_{\max}(\Sigma)}.
\]

The implementation keeps this calculation in a separate helper so that
\(J_q^TJ_q\) cannot silently replace \(\Phi\).

## Audit contract

The stable estimator contract identifier is
`variable_weight_nls_full_residual_jacobian_v1`. It is recorded in the
terminal manifest, summary, and shared settings block.

Every solver result records `stationarity_norm`. Each process row separately
records `attempt_stationarity_norm`, including a failed current attempt when a
prior state is retained as stale. Values that cannot be defined before a valid
linearization remain JSON `null`. Output remains strict JSON, and the
fixed-size compact summary store does not retain process dictionaries or add a
stationarity sample array.

## Evidence status

The existing Stage 2 bundle remains reproducible evidence of the previous
implementation. It must not be deleted, overwritten, or relabeled. Because
that implementation omitted the position-dependent weight derivative, the old
bundle is excluded from objective-correct estimator adequacy.

A later task must independently review and commit this correction,
preregister a corrected run, and then execute Stage 1 and Stage 2 once. This
correction task does not execute either evidence stage.
