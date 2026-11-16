"""Cros et al., IEEE L-CSS 2023, DOI 10.1109/LCSYS.2023.3285390.

Complete one-distance SCI (Eq. 12), minimizing trace of the reported bound.
No empirical reference covariance inflation. Requires consistent input bounds
and independent fresh range noise in the linearized observation model.
"""
import numpy as np
from scipy.optimize import brentq


def sci_update_matrices(P, reference_variance, direction, range_variance):
    """Return gain, full covariance bound, omega; 0 omega means no fusion.

    A precisely known reference (b=0) is an ordinary independent range update.
    Root tolerance below is numerical accuracy, not a statistical tuning factor.
    """
    u = np.asarray(direction, dtype=float)
    P = np.asarray(P, dtype=float)
    b = float(reference_variance)
    r = float(range_variance)
    if b < 0 or r <= 0 or not np.all(np.isfinite(P)):
        raise ValueError('Invalid covariance or measurement variance')
    v = P @ u
    a = float(u @ v)
    t = float(np.trace(P))
    m = float(v @ v)
    if a <= 0 or t <= 0:
        raise ValueError('Own covariance must be positive definite')
    if b == 0:
        K = v / (a + r)
        F = np.eye(len(u)) - np.outer(K, u)
        posterior = F @ P @ F.T + r * np.outer(K, K)
        return K, (posterior + posterior.T) / 2, None
    # d trace(P_SCI)/d omega at zero = trace(P) - ||P u||^2 / b.
    # The trace objective is convex (Cros Lemma 2), so this is global.
    if m <= t * b:
        return np.zeros_like(u), P.copy(), 0.0
    # Scale covariances together for numerical conditioning; omega unchanged.
    an, bn, rn, mn = a / t, b / t, r / t, m / (t * t)
    def derivative_numerator(w):
        D = w * an + (1 - w) * (bn + w * rn)
        Dprime = an - bn + rn - 2 * rn * w
        return D * D - mn * (D - w * (1 - w) * Dprime)
    w = float(brentq(derivative_numerator, 0., 1., xtol=1e-12, rtol=1e-12))
    if not 0 < w < 1:
        raise ArithmeticError('SCI optimum at singular boundary')
    D = w * a + (1 - w) * (b + w * r)
    K = w * v / D
    # Joseph equivalent of Eq. 12a. Independent range noise is not inflated.
    F = np.eye(len(u)) - np.outer(K, u)
    posterior = F @ P @ F.T / (1 - w) + (b / w + r) * np.outer(K, K)
    return K, (posterior + posterior.T) / 2, w
