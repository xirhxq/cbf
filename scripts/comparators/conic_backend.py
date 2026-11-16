"""Unchanged original solve; one fixed precision refinement only after rejection.

This is numerical work on the same SOCP, not a second scientific trajectory.
Every candidate must pass the original physical and KKT admission thresholds.
"""
import time
import conic_backend_core as core

ADMISSION_POLICY = core.ADMISSION_POLICY
REFINED_SETTINGS = dict(core.NUMERICAL_SETTINGS, tol_gap_abs=1e-12,
    tol_gap_rel=1e-14, tol_feas=1e-14, static_regularization_constant=1e-12,
    dynamic_regularization_eps=1e-16, dynamic_regularization_delta=1e-10,
    iterative_refinement_reltol=1e-15, iterative_refinement_abstol=1e-15,
    iterative_refinement_max_iter=30)

def solve_conic(problem):
    started=time.perf_counter()
    original=core.solve_conic(problem)
    if original['status']=='success':
        # No modifications to the actual accepted candidate or its diagnostics.
        return original
    original_settings=core.NUMERICAL_SETTINGS
    try:
        core.NUMERICAL_SETTINGS=REFINED_SETTINGS
        refined=core.solve_conic(problem)
    finally:
        core.NUMERICAL_SETTINGS=original_settings
    refined['initial_rejected_candidate']=original
    refined['numerical_attempts']=2
    refined['numerical_refinement_policy']='one-predeclared-tighter-solve-after-original-rejection'
    refined['wall_s']=time.perf_counter()-started
    return refined
