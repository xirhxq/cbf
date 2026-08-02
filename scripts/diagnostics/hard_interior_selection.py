"""Independent reconstruction of the frozen planar hard-interior policy."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
import math
from typing import Mapping

import numpy as np


@dataclass(frozen=True)
class PlanarChebyshevAudit:
    radius_mps: float
    witness: tuple[float, float]
    tight_hard_row_indices: tuple[int, ...]


def _continuous(value: object, *, positive: bool = False) -> float:
    if type(value) is not float:
        raise ValueError("hard-interior continuous value is not canonical")
    number = value
    if not math.isfinite(number) or (positive and number <= 0.0):
        raise ValueError("hard-interior numeric value is not finite and positive")
    return number


def _uint64(value: object) -> int:
    if type(value) is not int or not 0 <= value <= (1 << 64) - 1:
        raise ValueError("hard-interior uint64 is not canonical")
    return value


def _int32(value: object) -> int:
    if type(value) is not int or not -(1 << 31) <= value <= (1 << 31) - 1:
        raise ValueError("hard-interior int32 is not canonical")
    return value


def _canonical_problem(problem: Mapping) -> tuple[float, list[tuple[float, float, float]]]:
    if not isinstance(problem, Mapping) or set(problem) != {
        "owner", "control_size", "planar_component_max", "yaw_rate_max",
        "snapshot_version", "allocation_version", "bounds", "rows",
        "hard_problem_id",
    }:
        raise ValueError("hard-interior problem schema differs")
    if _int32(problem["owner"]) <= 0 or _int32(problem["control_size"]) != 3:
        raise ValueError("hard-interior problem owner/control size differs")
    component_max = _continuous(problem["planar_component_max"], positive=True)
    yaw_max = _continuous(problem["yaw_rate_max"], positive=True)
    if _uint64(problem["snapshot_version"]) == 0 or _uint64(problem["allocation_version"]) == 0:
        raise ValueError("hard-interior problem version differs")
    if not isinstance(problem["hard_problem_id"], str) or not problem["hard_problem_id"]:
        raise ValueError("hard-interior problem identity differs")
    bounds = problem["bounds"]
    expected_bounds = (
        (0, 1.0, component_max), (0, -1.0, component_max),
        (1, 1.0, component_max), (1, -1.0, component_max),
        (2, 1.0, yaw_max), (2, -1.0, yaw_max),
    )
    if not isinstance(bounds, list) or len(bounds) != len(expected_bounds):
        raise ValueError("hard-interior bounds differ")
    for bound, expected in zip(bounds, expected_bounds):
        if not isinstance(bound, Mapping) or set(bound) != {
            "control_index", "coefficient", "limit"
        }:
            raise ValueError("hard-interior bound schema differs")
        observed = (
            _int32(bound["control_index"]), _continuous(bound["coefficient"]),
            _continuous(bound["limit"]),
        )
        if observed != expected:
            raise ValueError("hard-interior bound differs")
    rows = problem["rows"]
    if not isinstance(rows, list):
        raise ValueError("hard-interior rows differ")
    planar_rows: list[tuple[float, float, float]] = []
    for row in rows:
        if not isinstance(row, Mapping) or set(row) != {
            "edge", "owner", "name", "coefficients", "constant",
            "post_reset_barrier", "snapshot_version", "allocation_version",
        }:
            raise ValueError("hard-interior row schema differs")
        edge = row["edge"]
        if not isinstance(edge, Mapping) or set(edge) != {"kind", "low", "high", "base_id"}:
            raise ValueError("hard-interior edge schema differs")
        if edge["kind"] not in {"localization", "collision"}:
            raise ValueError("hard-interior edge kind differs")
        try:
            for field in ("low", "high", "base_id"):
                _int32(edge[field])
        except ValueError:
            raise ValueError("hard-interior edge fields differ")
        if (_int32(row["owner"]) != problem["owner"]
                or not isinstance(row["name"], str) or not row["name"]):
            raise ValueError("hard-interior row owner/name differs")
        coefficients = row["coefficients"]
        if not isinstance(coefficients, list) or len(coefficients) != 3:
            raise ValueError("hard-interior row coefficients differ")
        coefficient = tuple(_continuous(item) for item in coefficients)
        if coefficient[2] != 0.0:
            raise ValueError("hard-interior row has yaw coefficient")
        if (_uint64(row["snapshot_version"]) != problem["snapshot_version"]
                or _uint64(row["allocation_version"]) != problem["allocation_version"]):
            raise ValueError("hard-interior row version differs")
        planar_rows.append((coefficient[0], coefficient[1], _continuous(row["constant"])))
        _continuous(row["post_reset_barrier"])
    return component_max, planar_rows


def solve_planar_hard_row_chebyshev(
    problem: Mapping, *, tolerance_mps: float = 1e-9
) -> PlanarChebyshevAudit:
    """Solve the serialized local LP by exhaustive, independent 3-plane enumeration."""
    tolerance = _continuous(tolerance_mps)
    if tolerance < 0.0:
        raise ValueError("hard-interior tolerance is negative")
    component_max, rows = _canonical_problem(problem)
    planes = [(-ax, -ay, 1.0, constant) for ax, ay, constant in rows]
    planes.extend((ax, ay, 0.0, component_max) for ax, ay in (
        (1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)
    ))
    candidates: list[np.ndarray] = []
    for selected in combinations(planes, 3):
        matrix = np.asarray([plane[:3] for plane in selected], dtype=float)
        if np.linalg.matrix_rank(matrix, tol=1e-12) != 3:
            continue
        candidate = np.linalg.solve(matrix, np.asarray([plane[3] for plane in selected]))
        if not np.all(np.isfinite(candidate)):
            continue
        if all(float(np.dot(plane[:3], candidate) - plane[3]) <= tolerance for plane in planes):
            candidates.append(candidate)
    if not candidates:
        raise ValueError("hard-interior problem has no finite feasible vertex")
    raw_maximum = max(float(candidate[2]) for candidate in candidates)
    tolerance_band = [candidate for candidate in candidates if raw_maximum - float(candidate[2]) <= tolerance]
    selected = min(tolerance_band, key=lambda candidate: (
        float(candidate[0]), float(candidate[1]), float(candidate[2])
    ))
    witness = (float(selected[0]), float(selected[1]))
    radius = float(selected[2])
    tight = tuple(
        index for index, (ax, ay, constant) in enumerate(rows)
        if abs(ax * witness[0] + ay * witness[1] + constant - radius) <= tolerance
    )
    return PlanarChebyshevAudit(radius, witness, tight)


def frozen_interior_floor(
    radius_mps: float, *, fraction: float = 0.10, cap_mps: float = 0.10,
    tolerance_mps: float = 1e-9,
) -> float:
    radius = _continuous(radius_mps)
    fraction_value = _continuous(fraction)
    cap = _continuous(cap_mps)
    tolerance = _continuous(tolerance_mps)
    if min(fraction_value, cap, tolerance) < 0.0:
        raise ValueError("hard-interior floor argument is negative")
    if radius <= tolerance:
        return 0.0
    return min(cap, fraction_value * max(0.0, radius - tolerance))
