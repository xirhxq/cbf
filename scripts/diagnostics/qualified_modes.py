"""Pure local solver-result projection and global numerical-mode clustering."""

import hashlib
import json
import math
from dataclasses import dataclass
from itertools import combinations
from numbers import Real
from typing import Mapping, Sequence

import numpy as np


MODE_TOLERANCE_M = 0.001
MODE_SENSITIVITY_M = (0.0005, 0.001, 0.002)


@dataclass(frozen=True)
class LocalCandidate:
    attempt_id: str
    estimate: tuple[float, float]
    objective_cost: float
    payload: Mapping[str, object]


@dataclass(frozen=True)
class NumericalMode:
    member_ids: tuple[str, ...]
    members: tuple[LocalCandidate, ...]
    diameter_m: float


@dataclass(frozen=True)
class ModeClustering:
    tolerance_m: float
    separable: bool
    reason: str
    modes: tuple[NumericalMode, ...]


@dataclass(frozen=True)
class QualifiedStart:
    """One source-independent WNLS start retained for offline qualification."""

    kind: str
    estimate: tuple[float, float]
    reference_keys: tuple[tuple[str, int], ...]
    branch: str | None = None


def project_local_candidate(
    attempt_id: str,
    result: Mapping,
) -> LocalCandidate | None:
    """Project one complete finite solver result into an immutable record."""
    if not isinstance(attempt_id, str) or not isinstance(result, Mapping):
        return None
    estimate = _finite_point(result.get("estimate"))
    cost = _finite_scalar(result.get("cost"))
    if estimate is None or cost is None or cost < 0.0:
        return None
    return LocalCandidate(attempt_id, estimate, cost, dict(result))


def cluster_candidates(
    candidates: Sequence[LocalCandidate],
    tolerance_m: float,
) -> ModeClustering:
    """Cluster starts that converge to the same numerical solution."""
    ordered = tuple(sorted(candidates, key=lambda candidate: candidate.attempt_id))
    visited: set[str] = set()
    modes: list[NumericalMode] = []
    for candidate in ordered:
        if candidate.attempt_id in visited:
            continue
        component: list[LocalCandidate] = []
        pending = [candidate]
        visited.add(candidate.attempt_id)
        while pending:
            current = pending.pop()
            component.append(current)
            for neighbor in ordered:
                if (
                    neighbor.attempt_id not in visited
                    and _distance(current, neighbor) <= tolerance_m
                ):
                    visited.add(neighbor.attempt_id)
                    pending.append(neighbor)
        members = tuple(sorted(component, key=lambda member: member.attempt_id))
        diameter = max(
            (_distance(first, second) for first in members for second in members),
            default=0.0,
        )
        if diameter > tolerance_m:
            return ModeClustering(
                tolerance_m,
                False,
                "nonseparable_chain",
                (),
            )
        modes.append(
            NumericalMode(
                tuple(member.attempt_id for member in members),
                members,
                diameter,
            )
        )
    return ModeClustering(tolerance_m, True, "separable", tuple(modes))


def _distance(a: LocalCandidate, b: LocalCandidate) -> float:
    return math.hypot(
        a.estimate[0] - b.estimate[0],
        a.estimate[1] - b.estimate[1],
    )


def select_representative(mode: NumericalMode) -> LocalCandidate:
    """Return the frozen, within-mode-only deterministic representative."""
    return min(
        mode.members,
        key=lambda candidate: (
            candidate.objective_cost,
            candidate.estimate[0].hex(),
            candidate.estimate[1].hex(),
            candidate.attempt_id,
        ),
    )


def canonical_mode_id(mode: NumericalMode) -> str:
    """Hash only sorted final coordinates, never start metadata or labels."""
    coordinates = sorted(
        (member.estimate[0].hex(), member.estimate[1].hex())
        for member in mode.members
    )
    encoded = json.dumps(coordinates, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def sensitivity_cluster_counts(
    candidates: Sequence[LocalCandidate],
) -> dict[str, int]:
    """Return the mode count at each frozen numerical sensitivity threshold."""
    return {
        tolerance.hex(): len(cluster_candidates(candidates, tolerance).modes)
        for tolerance in MODE_SENSITIVITY_M
    }


def stable_attempt_id(start: QualifiedStart) -> str:
    """Encode a start identity without index, record ordering, or source label."""
    payload = {
        "branch": start.branch,
        "estimate": [start.estimate[0].hex(), start.estimate[1].hex()],
        "kind": start.kind,
        "reference_keys": [list(key) for key in sorted(start.reference_keys)],
    }
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def enumerate_qualified_starts(
    references: Sequence[Mapping],
    *,
    live_seed: object,
    private_seed: object,
) -> tuple[QualifiedStart, ...]:
    """Enumerate all deterministic analytic and valid history starts."""
    canonical_references = _canonical_references(references)
    if canonical_references is None or len(canonical_references) < 2:
        return ()
    tentative: list[QualifiedStart] = []
    reference_keys = tuple(reference[0] for reference in canonical_references)
    if len(canonical_references) >= 3:
        algebraic = _algebraic_start(canonical_references)
        if algebraic is not None:
            tentative.append(
                QualifiedStart("algebraic", algebraic, reference_keys)
            )
        for first, second in combinations(canonical_references, 2):
            pair_keys = tuple(sorted((first[0], second[0])))
            for branch, estimate in _circle_branches(first, second):
                tentative.append(
                    QualifiedStart("circle", estimate, pair_keys, branch)
                )
        for kind, seed in (("live", live_seed), ("private", private_seed)):
            estimate = _seed_estimate(seed)
            if estimate is not None:
                tentative.append(QualifiedStart(kind, estimate, reference_keys))
    else:
        first, second = canonical_references
        pair_keys = tuple(sorted((first[0], second[0])))
        for branch, estimate in _circle_branches(first, second):
            tentative.append(QualifiedStart("circle", estimate, pair_keys, branch))
    unique: list[QualifiedStart] = []
    for start in sorted(tentative, key=stable_attempt_id):
        if all(_point_distance(start.estimate, kept.estimate) > 1e-9 for kept in unique):
            unique.append(start)
    return tuple(unique)


def _finite_scalar(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    scalar = float(value)
    return scalar if math.isfinite(scalar) else None


def _finite_point(value: object) -> tuple[float, float] | None:
    try:
        point = tuple(float(item) for item in value)
    except (TypeError, ValueError, OverflowError):
        return None
    if len(point) != 2 or not all(math.isfinite(item) for item in point):
        return None
    return point


def _reference_key(value: object) -> tuple[str, int] | None:
    if (
        not isinstance(value, (tuple, list))
        or len(value) != 2
        or not isinstance(value[0], str)
        or isinstance(value[1], bool)
        or not isinstance(value[1], int)
    ):
        return None
    return value[0], value[1]


def _canonical_references(
    references: Sequence[Mapping],
) -> tuple[tuple[tuple[str, int], tuple[float, float], float], ...] | None:
    if not isinstance(references, Sequence) or isinstance(references, (str, bytes)):
        return None
    canonical = []
    for reference in references:
        if not isinstance(reference, Mapping):
            return None
        key = _reference_key(reference.get("key"))
        position = _finite_point(reference.get("position"))
        radius = _finite_scalar(reference.get("range"))
        if key is None or position is None or radius is None or radius < 0.0:
            return None
        canonical.append((key, position, radius))
    if len({reference[0] for reference in canonical}) != len(canonical):
        return None
    return tuple(sorted(canonical, key=lambda reference: reference[0]))


def _circle_branches(
    first: tuple[tuple[str, int], tuple[float, float], float],
    second: tuple[tuple[str, int], tuple[float, float], float],
) -> tuple[tuple[str, tuple[float, float]], ...]:
    _, first_position, first_radius = first
    _, second_position, second_radius = second
    dx = second_position[0] - first_position[0]
    dy = second_position[1] - first_position[1]
    baseline = math.hypot(dx, dy)
    if (
        baseline == 0.0
        or baseline > first_radius + second_radius
        or baseline < abs(first_radius - second_radius)
    ):
        return ()
    along = (first_radius ** 2 - second_radius ** 2 + baseline ** 2) / (2 * baseline)
    height_squared = first_radius ** 2 - along ** 2
    if height_squared < -1e-9:
        return ()
    height = math.sqrt(max(0.0, height_squared))
    unit_x, unit_y = dx / baseline, dy / baseline
    midpoint = (first_position[0] + along * unit_x, first_position[1] + along * unit_y)
    negative = (midpoint[0] + height * unit_y, midpoint[1] - height * unit_x)
    positive = (midpoint[0] - height * unit_y, midpoint[1] + height * unit_x)
    return (("negative", negative), ("positive", positive))


def _algebraic_start(
    references: Sequence[tuple[tuple[str, int], tuple[float, float], float]],
) -> tuple[float, float] | None:
    origin = references[0]
    coefficients = []
    values = []
    for _, position, radius in references[1:]:
        coefficients.append([
            2.0 * (position[0] - origin[1][0]),
            2.0 * (position[1] - origin[1][1]),
        ])
        values.append(
            position[0] ** 2 + position[1] ** 2 - radius ** 2
            - (origin[1][0] ** 2 + origin[1][1] ** 2 - origin[2] ** 2)
        )
    try:
        candidate, _, rank, _ = np.linalg.lstsq(
            np.asarray(coefficients),
            np.asarray(values),
            rcond=None,
        )
    except np.linalg.LinAlgError:
        return None
    return _finite_point(candidate) if rank >= 2 else None


def _seed_estimate(seed: object) -> tuple[float, float] | None:
    if not isinstance(seed, Mapping):
        return None
    estimate = _finite_point(seed.get("estimate"))
    try:
        covariance = np.asarray(seed.get("modeled_covariance"), dtype=float)
    except (TypeError, ValueError, OverflowError):
        return None
    if (
        estimate is None
        or covariance.shape != (2, 2)
        or not np.isfinite(covariance).all()
        or not np.allclose(covariance, covariance.T, rtol=1e-12, atol=1e-12)
    ):
        return None
    try:
        return estimate if np.all(np.linalg.eigvalsh(covariance) > 0.0) else None
    except np.linalg.LinAlgError:
        return None


def _point_distance(first: tuple[float, float], second: tuple[float, float]) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])
