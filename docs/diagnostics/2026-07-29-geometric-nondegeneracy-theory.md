# Conditional geometric nondegeneracy of the modeled localization FIM

## Scope

This note isolates what the localization model proves from what must be
measured.  It concerns the range-only Fisher information matrix (FIM) used by
the offline estimator on a finite lower-index localization DAG.  It does not
turn an inverse FIM into a deterministic bound on estimator error, and it does
not establish an unconditional closed-loop safety result.

For UAV \(i\), let \(\mathcal R_i(t)\) be its complete active reference set at
time \(t\).  The set always contains the two predeclared fixed references and
may also contain visible bases and strictly lower squad-local-index UAVs.
For each admitted reference \(j\), define

\[
g_{ij}
=
\frac{\hat p_i-\hat p_j}
{\lVert\hat p_i-\hat p_j\rVert},
\qquad
v_{ij}
=
\sigma_r^2+g_{ij}^{\mathsf T}P_jg_{ij}.
\]

All directions below are assumed finite and defined, so observer and reference
positions are distinct.  The three matrices that must not be conflated are

\[
\begin{aligned}
P_i &:= \Phi_i^{-1},
&&\text{the modeled inverse-FIM matrix},\\
C_i &:= \operatorname{Cov}(p_i-\hat p_i),
&&\text{the true estimator-error covariance, when it exists},\\
G_i &:= \sum_{j\in\mathcal R_i(t)}
g_{ij}g_{ij}^{\mathsf T},
&&\text{the unweighted active-set geometry matrix}.
\end{aligned}
\]

In particular, \(P_i=C_i\) is not assumed.  Such an identification would need
an explicit calibration and dependence model that is absent here.  The
implemented coefficient-3 radius

\[
\epsilon_i=3\sqrt{\lambda_{\max}(P_i)}
\]

is therefore a model-derived radius.  Whether it contains
\(\lVert p_i-\hat p_i\rVert\) is an empirical calibration question.

## Active-set geometry theorem

The modeled range FIM is

\[
\Phi_i
=
\sum_{j\in\mathcal R_i(t)}
\frac{g_{ij}g_{ij}^{\mathsf T}}{v_{ij}}.
\]

Assume, for one UAV at one frame, that

\[
G_i\succeq\gamma_i I,\qquad \gamma_i>0,
\]

and that every effective variance is finite, positive, and satisfies

\[
0<v_{ij}\leq\bar v_i<\infty.
\]

Then

\[
\Phi_i\succeq\frac{\gamma_i}{\bar v_i}I.
\]

To prove the result, take an arbitrary \(x\in\mathbb R^2\).  Positivity and the
upper variance bound give \(v_{ij}^{-1}\geq\bar v_i^{-1}\), hence

\[
\begin{aligned}
x^{\mathsf T}\Phi_i x
&=
\sum_{j\in\mathcal R_i(t)}
\frac{(g_{ij}^{\mathsf T}x)^2}{v_{ij}}\\
&\geq
\frac{1}{\bar v_i}
\sum_{j\in\mathcal R_i(t)}
(g_{ij}^{\mathsf T}x)^2\\
&=
\frac{1}{\bar v_i}x^{\mathsf T}G_i x\\
&\geq
\frac{\gamma_i}{\bar v_i}\lVert x\rVert^2.
\end{aligned}
\]

Because this holds for every \(x\), the claimed Loewner-order inequality
follows.  In particular, \(\Phi_i\) is positive definite and invertible.
Inverting the positive-definite inequality reverses the Loewner order:

\[
P_i=\Phi_i^{-1}
\preceq
\frac{\bar v_i}{\gamma_i}I.
\]

This is a model-internal upper bound on \(P_i\).  It is neither an upper bound
on \(C_i\) nor a deterministic bound on the realized error
\(p_i-\hat p_i\).

The theorem makes the complete dynamic active set the primary geometry.
If extra admitted references are added while the existing directions and
weights are held fixed, then

\[
\Phi_i^{\mathrm{dynamic}}
=
\Phi_i^{\mathrm{fixed}}
+
\sum_{\ell\in\mathcal R_i^{\mathrm{extra}}(t)}
\frac{g_{i\ell}g_{i\ell}^{\mathsf T}}{v_{i\ell}}
\succeq
\Phi_i^{\mathrm{fixed}}.
\]

Thus added positive-semidefinite terms cannot reduce the modeled minimum FIM
eigenvalue in that same-frame, fixed-term comparison.  This does not compare
two complete estimator replays in which estimates, upstream covariances,
directions, weights, or WNLS branches differ.  Nor does it show that
correlated references improve true-error calibration.

## Fixed-pair geometry is only a sufficient special case

Let \(j(i)\) and \(k(i)\) be the two predeclared fixed references.  With

\[
\alpha_{ij}=v_{ij}^{-1}>0,\qquad
\alpha_{ik}=v_{ik}^{-1}>0,
\]

their two-dimensional modeled information contribution is

\[
\Phi_i^{\mathrm{fixed}}
=
\alpha_{ij}g_{ij}g_{ij}^{\mathsf T}
+
\alpha_{ik}g_{ik}g_{ik}^{\mathsf T}.
\]

Choose an orthonormal coordinate system whose first axis is \(g_{ij}\), and
write

\[
g_{ik}
=
\begin{bmatrix}
\cos\theta_i\\
\sin\theta_i
\end{bmatrix}.
\]

In these coordinates,

\[
\det\!\left(\Phi_i^{\mathrm{fixed}}\right)
=
\alpha_{ij}\alpha_{ik}\sin^2\theta_i
=
\alpha_{ij}\alpha_{ik}
\left(1-(g_{ij}^{\mathsf T}g_{ik})^2\right).
\]

Consequently,

\[
\lvert g_{ij}^{\mathsf T}g_{ik}\rvert<1
\quad\Longrightarrow\quad
\det\!\left(\Phi_i^{\mathrm{fixed}}\right)>0.
\]

Since the matrix is positive semidefinite, a positive determinant makes it
positive definite.  A fixed pair with a strict angle-to-collinearity margin
therefore supplies one sufficient mechanism for \(G_i\succeq\gamma_i I\).
It is not the necessary mechanism: additional dynamic reference directions
can make the complete \(G_i\) positive definite even when the fixed pair is
nearly collinear.

Nominal triangular-ladder targets may explain why a fixed-reference triple is
designed to be noncollinear.  Let the nominal side lengths be

\[
\begin{aligned}
a_i^\star&=\lVert p_i^\star-p_{j(i)}^\star\rVert,\\
b_i^\star&=\lVert p_i^\star-p_{k(i)}^\star\rVert,\\
c_i^\star&=\lVert p_{j(i)}^\star-p_{k(i)}^\star\rVert.
\end{aligned}
\]

The nominal included angle satisfies

\[
\cos\theta_i^\star
=
\frac{(a_i^\star)^2+(b_i^\star)^2-(c_i^\star)^2}
{2a_i^\star b_i^\star}.
\]

If \(\lvert\cos\theta_i^\star\rvert<1\), and if each estimated vertex remains
within a stated perturbation radius \(\delta_i\) of its target, every side
length changes by at most \(2\delta_i\).  Define the three nominal strict
triangle-inequality slacks

\[
\begin{aligned}
s_{i,1}^\star
&=a_i^\star+b_i^\star-c_i^\star,\\
s_{i,2}^\star
&=a_i^\star+c_i^\star-b_i^\star,\\
s_{i,3}^\star
&=b_i^\star+c_i^\star-a_i^\star.
\end{aligned}
\]

For a finite positive-area nominal triangle, every slack is strictly
positive.  In the worst uniform three-vertex perturbation, the two positive
side terms in a triangle inequality can each decrease by \(2\delta_i\), while
the negative side term can increase by \(2\delta_i\).  The perturbed slack is
therefore at least \(s_{i,r}^\star-6\delta_i\).  The uniform admissible
tracking-deviation supremum is

\[
\delta_i^\star
=
\frac{1}{6}
\min\left\{
s_{i,1}^\star,
s_{i,2}^\star,
s_{i,3}^\star
\right\}.
\]

The sufficient certificate is strict:

\[
\max_{\ell\in\{i,j(i),k(i)\}}
\lVert \hat p_\ell-p_\ell^\star\rVert
<\delta_i^\star.
\]

Equality is not certified because the worst-case lower bound on one perturbed
triangle slack is then zero.  This uniform perturbation result is sufficient,
not necessary; an observed triangle can remain noncollinear even when the
uniform certificate fails.

The current controller does not make that condition invariant.  Its task CBF
is a soft objective and supplies no hard bound on the target-tracking
deviation \(\delta_i\).  The localization-connectivity CBF supplies upper
bounds on observer-to-reference range, but those upper bounds alone do not
bound the reference-to-reference separation, triangle area, or included angle
away from zero.  Collinear triples can satisfy both observer-to-reference
upper bounds.  Therefore neither the soft tracking objective nor the range
upper bound guarantees the fixed-pair determinant condition.  Fixed-pair
angles and areas remain explanatory diagnostics; complete dynamic active-set
geometry is the registered primary quantity.

## Finiteness on a finite lower-index DAG

Consider one frame.  Bases are roots with finite configured modeled
covariance.  Every UAV-to-UAV reference edge points from a strictly lower
squad-local index to a higher squad-local index.  Increasing squad-local index
therefore gives a topological order for each finite squad.

Assume:

1. root covariances and \(\sigma_r^2\) are finite, with
   \(\sigma_r^2>0\);
2. every UAV has its required fixed-reference inputs available;
3. every admitted reference position and every returned UAV estimate needed
   to form a direction is finite;
4. each complete active set satisfies
   \(G_i\succeq\gamma_i I\) for some finite \(\gamma_i>0\); and
5. the active set is finite and every effective variance is positive.

The proof is induction over the finite topological order.  At a root, the
configured modeled covariance is finite by assumption.  Now take a UAV \(i\)
and suppose every lower-index UAV predecessor \(j\) used by \(i\) has a finite
modeled covariance \(P_j\).  A finite unit direction then gives

\[
0<v_{ij}
=
\sigma_r^2+g_{ij}^{\mathsf T}P_jg_{ij}
<\infty.
\]

The finite active set has a finite maximum

\[
\bar v_i=\max_{j\in\mathcal R_i(t)}v_{ij}.
\]

The active-set theorem yields

\[
\Phi_i\succeq\frac{\gamma_i}{\bar v_i}I\succ0,
\]

so \(P_i=\Phi_i^{-1}\) exists and is finite.  This closes the induction step.
There are finitely many lower-index vertices, so induction establishes finite
modeled covariance for every UAV in the DAG at that frame.

This result excludes singular modeled-FIM blow-up under the listed
assumptions.  It does not prove estimator convergence, resolve the
two-range mirror ambiguity, make shared upstream errors independent, or
establish finiteness of \(C_i\).  It is also not uniform in arbitrary DAG
depth.

## Finite-depth model recursion

A conservative model-only corollary can make the depth dependence explicit.
Suppose bases have zero configured modeled covariance, every complete active
geometry matrix satisfies the uniform condition

\[
G_i\succeq\gamma I,\qquad \gamma>0,
\]

and every predecessor at inductive level \(d-1\) satisfies
\(P_j\preceq B_{d-1}I\).  Define

\[
B_0=0,\qquad
B_d=\frac{\sigma_r^2+B_{d-1}}{\gamma}.
\]

For every parent direction,

\[
v_{ij}
=
\sigma_r^2+g_{ij}^{\mathsf T}P_jg_{ij}
\leq
\sigma_r^2+B_{d-1}.
\]

Applying the active-set theorem with
\(\bar v_i=\sigma_r^2+B_{d-1}\) gives

\[
P_i
\preceq
\frac{\sigma_r^2+B_{d-1}}{\gamma}I
=
B_d I,
\]

and hence the model-derived radius obeys

\[
\epsilon_i\leq3\sqrt{B_d}.
\]

The recurrence is a finite-depth bookkeeping bound, not a stability
contraction.  In particular, it need not contract and can grow rapidly with
depth.  It proves only that every \(B_d\) is finite for a fixed finite \(d\)
when \(\sigma_r^2\) and \(\gamma\) are finite and positive.  It says nothing
about arbitrarily deep graphs.  It is not a bound on \(C_i\), on a realized
true error, or on the probability that the coefficient-3 radius contains that
error.  The empirical report uses `squad_local_index`, rather than calling it
a hop count, because dynamic base references can shorten graph distance.

## Relation to the robust CBF premise

The robust distance-CBF argument retains the explicit premise

\[
\lVert p_i-\hat p_i\rVert\leq\epsilon_i.
\]

The geometry theorem proves that the modeled \(\epsilon_i\) is finite when its
assumptions hold.  It does not prove this true-error containment premise.
The controller uses the modeled radius to robustify formation maintenance,
while maintained formation geometry is intended to support a nonsingular
localization model.  The logically valid structure is therefore:

1. conditional active-set geometry implies a finite modeled inverse FIM;
2. conditional true-error containment supports the existing robust CBF
   result; and
3. post-hoc Monte Carlo evidence measures whether the tested trajectory and
   seeds exhibit the required geometry, finite-horizon error behavior,
   estimator availability, and radius calibration.

These are two conditional implications plus empirical consistency evidence,
not one unconditional end-to-end theorem.

## Claim boundary

The analytic result supports positive definiteness and finite modeled
covariances on a finite lower-index DAG under explicit geometry, variance,
availability, and finiteness assumptions.  The registered exploratory
analysis may describe only the tested one-trajectory, 20-seed configuration.
Neither this note nor that analysis supports a deterministic true-error bound,
arbitrary-depth stability, unconditional controller safety, fixed-pair
nondegeneracy at every frame, estimator-in-the-loop mission performance,
graph superiority, or cross-trajectory generality.
