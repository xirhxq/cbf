"""Exact max--min reserve-feasibility budget for a 2-D acceleration box.

The epigraph linear program is

    maximize    t
    subject to  t <= c_e.T @ a + d_e,  for every residual e
                -half_box <= a_x, a_y <= half_box.

The solver enumerates every full-rank vertex of this three-variable
polyhedron.  This includes box corners, residual/box-edge intersections, and
interior intersections of three residual planes.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class ResidualConstraint:
    """Affine residual ``cx * ax + cy * ay + offset``."""

    name: str
    cx: float
    cy: float
    offset: float


@dataclass(frozen=True)
class GammaStarSolution:
    """Primal solution and ordered residual-plus-box dual certificate."""

    gamma: float
    acceleration: tuple[float, float]
    active_residuals: tuple[str, ...]
    active_box_constraints: tuple[str, ...]
    dual_multiplier_names: tuple[str, ...]
    dual_multipliers: np.ndarray
    primal_residual: float
    stationarity_residual: float
    complementarity_residual: float
    duality_gap: float


_BOX_ROWS = (
    ("a_x lower", (-1.0, 0.0, 0.0)),
    ("a_x upper", (1.0, 0.0, 0.0)),
    ("a_y lower", (0.0, -1.0, 0.0)),
    ("a_y upper", (0.0, 1.0, 0.0)),
)


def _validate_inputs(
    constraints: Sequence[ResidualConstraint],
    half_box: float,
) -> None:
    if not math.isfinite(half_box) or half_box < 0.0:
        raise ValueError("half_box must be finite and non-negative")
    for constraint in constraints:
        values = (constraint.cx, constraint.cy, constraint.offset)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"constraint {constraint.name!r} must be finite")


def _linear_program(
    constraints: Sequence[ResidualConstraint],
    half_box: float,
) -> tuple[np.ndarray, np.ndarray, tuple[str, ...], tuple[str, ...]]:
    residual_names = tuple(constraint.name for constraint in constraints)
    box_names = tuple(name for name, _ in _BOX_ROWS)
    rows = [
        (-constraint.cx, -constraint.cy, 1.0)
        for constraint in constraints
    ]
    bounds = [constraint.offset for constraint in constraints]
    rows.extend(row for _, row in _BOX_ROWS)
    bounds.extend([half_box] * len(_BOX_ROWS))
    return (
        np.asarray(rows, dtype=float),
        np.asarray(bounds, dtype=float),
        residual_names,
        box_names,
    )


def _best_vertex(
    matrix: np.ndarray,
    bounds: np.ndarray,
    constraints: Sequence[ResidualConstraint],
    half_box: float,
) -> np.ndarray:
    best: np.ndarray | None = None
    for indices in combinations(range(matrix.shape[0]), 3):
        active_matrix = matrix[list(indices)]
        determinant = float(np.linalg.det(active_matrix))
        singularity_bound = (
            64.0
            * np.finfo(float).eps
            * float(np.prod(np.linalg.norm(active_matrix, axis=1)))
        )
        if not math.isfinite(determinant) or abs(determinant) <= singularity_bound:
            continue
        candidate = np.linalg.solve(active_matrix, bounds[list(indices)])
        # Linear solves can return a coordinate a few ulps inside a box face
        # that was one of the defining equalities.  Preserve that exact active
        # face before recomputing the attained epigraph value; otherwise the
        # subsequent KKT pass can lose a genuine box multiplier.
        active = set(indices)
        box_offset = len(constraints)
        if box_offset in active:
            candidate[0] = -half_box
        elif box_offset + 1 in active:
            candidate[0] = half_box
        else:
            candidate[0] = min(half_box, max(-half_box, candidate[0]))
        if box_offset + 2 in active:
            candidate[1] = -half_box
        elif box_offset + 3 in active:
            candidate[1] = half_box
        else:
            candidate[1] = min(half_box, max(-half_box, candidate[1]))
        achieved = min(
            constraint.cx * candidate[0]
            + constraint.cy * candidate[1]
            + constraint.offset
            for constraint in constraints
        )
        candidate[2] = achieved
        terms = matrix * candidate[np.newaxis, :]
        allowance = 64.0 * np.finfo(float).eps * (
            np.sum(np.abs(terms), axis=1) + np.abs(bounds)
        ) + np.finfo(float).smallest_subnormal
        if np.any(matrix @ candidate - bounds > allowance):
            continue
        if best is None or achieved > best[2]:
            best = candidate.copy()
    if best is None:
        raise RuntimeError("epigraph LP has no numerically valid vertex")
    return best


def _dual_certificate(
    matrix: np.ndarray,
    bounds: np.ndarray,
    point: np.ndarray,
) -> np.ndarray:
    objective = np.array([0.0, 0.0, 1.0], dtype=float)
    terms = matrix * point[np.newaxis, :]
    active_allowance = 64.0 * np.finfo(float).eps * (
        np.sum(np.abs(terms), axis=1) + np.abs(bounds)
    ) + np.finfo(float).smallest_subnormal
    active = np.flatnonzero(np.abs(matrix @ point - bounds) <= active_allowance)
    for indices_tuple in combinations(active.tolist(), 3):
        indices = list(indices_tuple)
        active_transpose = matrix[indices].T
        determinant = float(np.linalg.det(active_transpose))
        singularity_bound = (
            64.0
            * np.finfo(float).eps
            * float(np.prod(np.linalg.norm(active_transpose, axis=0)))
        )
        if not math.isfinite(determinant) or abs(determinant) <= singularity_bound:
            continue
        selected = np.linalg.solve(active_transpose, objective)
        multiplier_allowance = 64.0 * np.finfo(float).eps * max(
            1.0, float(np.max(np.abs(selected)))
        )
        if np.min(selected) < -multiplier_allowance:
            continue
        dual = np.zeros(matrix.shape[0], dtype=float)
        dual[indices] = np.maximum(selected, 0.0)
        stationarity = matrix.T @ dual - objective
        # A componentwise relative test is invalid here: a perfectly ordinary
        # backward-stable solve can form one stationarity component by nearly
        # cancelling small terms.  Certify the solve with the standard
        # normwise backward-error scale for A.T @ lambda = objective instead.
        stationarity_bound = 64.0 * np.finfo(float).eps * (
            float(np.linalg.norm(active_transpose, ord=np.inf))
            * float(np.linalg.norm(dual[indices], ord=np.inf))
            + float(np.linalg.norm(objective, ord=np.inf))
        ) + np.finfo(float).smallest_subnormal
        if float(np.linalg.norm(stationarity, ord=np.inf)) <= stationarity_bound:
            return dual
    raise RuntimeError("failed to derive a non-negative dual certificate")


def solve_gamma_star(
    constraints: Sequence[ResidualConstraint],
    half_box: float,
) -> GammaStarSolution:
    """Solve the joint reserve-feasibility budget by vertex enumeration.

    The dual vector is self-described by ``dual_multiplier_names``. Residual
    multipliers come first, followed by lower/upper rows for x and then y.
    """

    constraints = tuple(constraints)
    _validate_inputs(constraints, half_box)
    if not constraints:
        return GammaStarSolution(
            gamma=math.inf,
            acceleration=(0.0, 0.0),
            active_residuals=(),
            active_box_constraints=(),
            dual_multiplier_names=(),
            dual_multipliers=np.zeros(0, dtype=float),
            primal_residual=0.0,
            stationarity_residual=0.0,
            complementarity_residual=0.0,
            duality_gap=0.0,
        )

    matrix, bounds, residual_names, box_names = _linear_program(
        constraints,
        half_box,
    )
    point = _best_vertex(matrix, bounds, constraints, half_box)
    dual = _dual_certificate(matrix, bounds, point)
    slacks = matrix @ point - bounds
    objective = np.array([0.0, 0.0, 1.0], dtype=float)
    residual_count = len(residual_names)
    terms = matrix * point[np.newaxis, :]
    active_allowance = 64.0 * np.finfo(float).eps * (
        np.sum(np.abs(terms), axis=1) + np.abs(bounds)
    ) + np.finfo(float).smallest_subnormal
    active_indices = np.flatnonzero(np.abs(slacks) <= active_allowance)
    active_residuals = tuple(
        residual_names[index]
        for index in active_indices
        if index < residual_count
    )
    active_box_constraints = tuple(
        box_names[index - residual_count]
        for index in active_indices
        if index >= residual_count
    )

    return GammaStarSolution(
        gamma=float(point[2]),
        acceleration=(float(point[0]), float(point[1])),
        active_residuals=active_residuals,
        active_box_constraints=active_box_constraints,
        dual_multiplier_names=residual_names + box_names,
        dual_multipliers=dual,
        primal_residual=float(max(np.max(slacks), 0.0)),
        stationarity_residual=float(np.max(np.abs(matrix.T @ dual - objective))),
        complementarity_residual=float(np.max(np.abs(dual * slacks))),
        duality_gap=float(bounds @ dual - point[2]),
    )
