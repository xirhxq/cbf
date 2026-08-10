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
    tolerance: float,
) -> None:
    if not math.isfinite(half_box) or half_box < 0.0:
        raise ValueError("half_box must be finite and non-negative")
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("tolerance must be finite and positive")
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
    tolerance: float,
) -> np.ndarray:
    best: np.ndarray | None = None
    for indices in combinations(range(matrix.shape[0]), 3):
        active_matrix = matrix[list(indices)]
        if np.linalg.matrix_rank(active_matrix, tol=tolerance) < 3:
            continue
        candidate = np.linalg.solve(active_matrix, bounds[list(indices)])
        if np.max(matrix @ candidate - bounds) > tolerance:
            continue
        if best is None or candidate[2] > best[2] + tolerance:
            best = candidate
    if best is None:
        raise RuntimeError("epigraph LP has no numerically valid vertex")
    return best


def _dual_certificate(
    matrix: np.ndarray,
    bounds: np.ndarray,
    point: np.ndarray,
    tolerance: float,
) -> np.ndarray:
    objective = np.array([0.0, 0.0, 1.0], dtype=float)
    active = np.flatnonzero(np.abs(matrix @ point - bounds) <= 10.0 * tolerance)
    for indices_tuple in combinations(active.tolist(), 3):
        indices = list(indices_tuple)
        active_transpose = matrix[indices].T
        if np.linalg.matrix_rank(active_transpose, tol=tolerance) < 3:
            continue
        selected = np.linalg.solve(active_transpose, objective)
        if np.min(selected) < -10.0 * tolerance:
            continue
        dual = np.zeros(matrix.shape[0], dtype=float)
        dual[indices] = np.maximum(selected, 0.0)
        if np.max(np.abs(matrix.T @ dual - objective)) <= 10.0 * tolerance:
            return dual
    raise RuntimeError("failed to derive a non-negative dual certificate")


def solve_gamma_star(
    constraints: Sequence[ResidualConstraint],
    half_box: float,
    tolerance: float = 1.0e-9,
) -> GammaStarSolution:
    """Solve the joint reserve-feasibility budget exactly up to tolerance.

    The dual vector is self-described by ``dual_multiplier_names``. Residual
    multipliers come first, followed by lower/upper rows for x and then y.
    """

    constraints = tuple(constraints)
    _validate_inputs(constraints, half_box, tolerance)
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
    point = _best_vertex(matrix, bounds, tolerance)
    dual = _dual_certificate(matrix, bounds, point, tolerance)
    slacks = matrix @ point - bounds
    objective = np.array([0.0, 0.0, 1.0], dtype=float)
    residual_count = len(residual_names)
    active_indices = np.flatnonzero(np.abs(slacks) <= 10.0 * tolerance)
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
