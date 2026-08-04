"""Pure local solver-result projection and global numerical-mode clustering."""

import hashlib
import json
import math
from dataclasses import dataclass
from itertools import combinations
from numbers import Real
from typing import Mapping, Sequence

import numpy as np

from scripts.diagnostics.estimator_lifecycle import canonical_private_state


MODE_TOLERANCE_M = 0.001
MODE_SENSITIVITY_M = (0.0005, 0.001, 0.002)
CONTRACT_TOLERANCE = 1e-12
INNOVATION_LIMIT = 11.829007011943707
RELATIVE_SPECTRAL_THRESHOLD = 1e-12
FORBIDDEN_RUNTIME_KEYS = frozenset({
    "truth_position",
    "future_estimate",
    "analyzer_label",
    "realized_error",
})
DEPLOYMENT_CONTRACT_FIELDS = frozenset({
    "anchor_ids",
    "anchor_coordinates",
    "deployment_vertices",
    "unit_normal",
    "offset",
    "ocean_side",
    "margin_m",
    "domain_version",
})


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
class DeploymentContract:
    anchor_ids: tuple[int, int]
    anchor_coordinates: tuple[tuple[float, float], tuple[float, float]]
    deployment_vertices: tuple[tuple[float, float], ...]
    unit_normal: tuple[float, float]
    offset: float
    ocean_side: int
    margin_m: float
    domain_version: str


@dataclass(frozen=True)
class ModeQualification:
    mode_id: str
    admissible: bool
    reason: str
    score: float | None


@dataclass(frozen=True)
class PublicationDecision:
    status: str
    reason: str
    mode_id: str | None
    representative: LocalCandidate | None


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
    if not isinstance(mode, NumericalMode) or not mode.members:
        raise ValueError("mode must contain at least one candidate")
    canonical_members = []
    for member in mode.members:
        canonical = _canonical_local_candidate(member)
        if canonical is None:
            raise ValueError("mode contains a malformed candidate")
        canonical_members.append(canonical)
    return min(
        canonical_members,
        key=lambda candidate: (
            candidate.objective_cost,
            candidate.estimate[0].hex(),
            candidate.estimate[1].hex(),
            candidate.attempt_id,
        ),
    )


def canonical_mode_id(mode: NumericalMode) -> str:
    """Hash only sorted final coordinates, never start metadata or labels."""
    if not isinstance(mode, NumericalMode) or not mode.members:
        raise ValueError("mode must contain at least one candidate")
    canonical_members = []
    for member in mode.members:
        canonical = _canonical_local_candidate(member)
        if canonical is None:
            raise ValueError("mode contains a malformed candidate")
        canonical_members.append(canonical)
    coordinates = sorted(
        (member.estimate[0].hex(), member.estimate[1].hex())
        for member in canonical_members
    )
    encoded = json.dumps(coordinates, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def qualify_deployment_mode(
    mode: NumericalMode,
    representative: LocalCandidate,
    domain: DeploymentContract,
) -> ModeQualification:
    """Qualify one representative against a frozen oriented half-plane."""
    _validated_deployment_contract(domain)
    _validate_mode_representative(mode, representative)
    signed = domain.ocean_side * (
        domain.unit_normal[0] * representative.estimate[0]
        + domain.unit_normal[1] * representative.estimate[1]
        + domain.offset
    )
    if not math.isfinite(signed):
        raise ValueError("candidate signed distance must be finite")
    admissible = signed >= domain.margin_m
    return ModeQualification(
        canonical_mode_id(mode),
        admissible,
        "deployment_side" if admissible else "outside_deployment_side",
        signed,
    )


def qualify_history_mode(
    mode: NumericalMode,
    representative: LocalCandidate,
    propagated_private_prior: Mapping,
    innovation_limit: float,
) -> ModeQualification:
    """Qualify one representative using only a propagated private branch prior."""
    _validate_mode_representative(mode, representative)
    limit = _finite_scalar(innovation_limit)
    if limit is None or limit != INNOVATION_LIMIT:
        raise ValueError("innovation limit does not match the frozen threshold")
    prior = canonical_private_state(propagated_private_prior)
    if prior is None:
        raise ValueError("propagated private prior is invalid")
    candidate_covariance = _candidate_covariance(representative)
    prior_covariance = _canonical_history_covariance(
        prior["modeled_covariance"]
    )
    candidate_estimate = np.asarray(representative.estimate, dtype=float)
    prior_estimate = np.asarray(prior["estimate"], dtype=float)
    if candidate_covariance is None or prior_covariance is None:
        raise ValueError("history covariance is invalid")
    covariance_sum = _canonical_history_covariance(
        candidate_covariance + prior_covariance
    )
    if covariance_sum is None:
        raise ValueError("history covariance sum is invalid")
    innovation = candidate_estimate - prior_estimate
    try:
        score = float(innovation @ np.linalg.solve(covariance_sum, innovation))
    except np.linalg.LinAlgError as error:
        raise ValueError("history covariance solve failed") from error
    if not math.isfinite(score) or score < 0.0:
        raise ValueError("history innovation score must be finite and nonnegative")
    admissible = score <= limit
    return ModeQualification(
        canonical_mode_id(mode),
        admissible,
        (
            "history_innovation_within_limit"
            if admissible
            else "history_innovation_exceeds_limit"
        ),
        score,
    )


def _candidate_covariance(candidate: LocalCandidate) -> np.ndarray | None:
    has_covariance = "covariance" in candidate.payload
    has_modeled = "modeled_covariance" in candidate.payload
    if has_covariance == has_modeled:
        return None
    value = (
        candidate.payload["covariance"]
        if has_covariance
        else candidate.payload["modeled_covariance"]
    )
    return _canonical_history_covariance(value)


def _canonical_history_covariance(value: object) -> np.ndarray | None:
    try:
        covariance = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return None
    if (
        covariance.shape != (2, 2)
        or not np.isfinite(covariance).all()
        or not np.allclose(
            covariance,
            covariance.T,
            rtol=CONTRACT_TOLERANCE,
            atol=CONTRACT_TOLERANCE,
        )
    ):
        return None
    canonical = 0.5 * (covariance + covariance.T)
    try:
        eigenvalues = np.linalg.eigvalsh(canonical)
    except np.linalg.LinAlgError:
        return None
    if (
        not np.isfinite(eigenvalues).all()
        or eigenvalues[-1] <= 0.0
        or eigenvalues[0] <= RELATIVE_SPECTRAL_THRESHOLD * eigenvalues[-1]
    ):
        return None
    return canonical


def qualify_all(
    modes: Sequence[NumericalMode],
    representatives: Sequence[LocalCandidate],
    qualifier_kind: str,
    qualifier_payload: Mapping,
) -> tuple[ModeQualification, ...]:
    """Apply one exact-schema runtime qualifier to every discovered mode."""
    if (
        not isinstance(modes, Sequence)
        or isinstance(modes, (str, bytes))
        or not isinstance(representatives, Sequence)
        or isinstance(representatives, (str, bytes))
    ):
        raise ValueError("modes and representatives must be sequences")
    mode_tuple = tuple(modes)
    representative_tuple = tuple(representatives)
    if len(mode_tuple) != len(representative_tuple):
        raise ValueError("one representative is required per mode")
    if not isinstance(qualifier_payload, Mapping):
        raise ValueError("qualifier payload must be a mapping")
    if _contains_forbidden_runtime_key(qualifier_payload):
        raise ValueError("forbidden runtime qualifier field")
    for mode, representative in zip(
        mode_tuple,
        representative_tuple,
        strict=True,
    ):
        _validate_mode_representative(mode, representative)
    if qualifier_kind == "deployment":
        if set(qualifier_payload) != {"domain"}:
            raise ValueError("deployment qualifier payload schema is invalid")
        domain_value = qualifier_payload["domain"]
        if isinstance(domain_value, DeploymentContract):
            domain = domain_value
        elif isinstance(domain_value, Mapping):
            if set(domain_value) != DEPLOYMENT_CONTRACT_FIELDS:
                raise ValueError("deployment contract mapping schema is invalid")
            try:
                domain = DeploymentContract(**dict(domain_value))
            except TypeError as error:
                raise ValueError("deployment contract mapping is invalid") from error
        else:
            raise ValueError("deployment domain must be a contract or mapping")
        _validated_deployment_contract(domain)
        return tuple(
            qualify_deployment_mode(mode, representative, domain)
            for mode, representative in zip(
                mode_tuple,
                representative_tuple,
                strict=True,
            )
        )
    if qualifier_kind == "history":
        if set(qualifier_payload) != {
            "propagated_private_prior",
            "innovation_limit",
        }:
            raise ValueError("history qualifier payload schema is invalid")
        return tuple(
            qualify_history_mode(
                mode,
                representative,
                qualifier_payload["propagated_private_prior"],
                qualifier_payload["innovation_limit"],
            )
            for mode, representative in zip(
                mode_tuple,
                representative_tuple,
                strict=True,
            )
        )
    raise ValueError("unsupported qualifier kind")


def _contains_forbidden_runtime_key(value: object) -> bool:
    return not _runtime_payload_is_safe(value, set())


def _runtime_payload_is_safe(value: object, active_ids: set[int]) -> bool:
    if value is None or isinstance(value, (str, bool)):
        return True
    if isinstance(value, Real):
        return _finite_scalar(value) is not None
    if isinstance(value, np.generic):
        return _runtime_payload_is_safe(value.item(), active_ids)
    if isinstance(value, DeploymentContract):
        nested = (
            value.anchor_ids,
            value.anchor_coordinates,
            value.deployment_vertices,
            value.unit_normal,
            value.offset,
            value.ocean_side,
            value.margin_m,
            value.domain_version,
        )
        return _runtime_container_is_safe(value, nested, active_ids)
    if isinstance(value, Mapping):
        identifier = id(value)
        if identifier in active_ids:
            return False
        active_ids.add(identifier)
        try:
            return all(
                isinstance(key, str)
                and key not in FORBIDDEN_RUNTIME_KEYS
                and _runtime_payload_is_safe(nested, active_ids)
                for key, nested in value.items()
            )
        finally:
            active_ids.remove(identifier)
    if isinstance(value, (tuple, list)):
        return _runtime_container_is_safe(value, value, active_ids)
    if isinstance(value, np.ndarray):
        identifier = id(value)
        if identifier in active_ids:
            return False
        active_ids.add(identifier)
        try:
            return _runtime_payload_is_safe(value.tolist(), active_ids)
        finally:
            active_ids.remove(identifier)
    return False


def _runtime_container_is_safe(
    container: object,
    values: object,
    active_ids: set[int],
) -> bool:
    identifier = id(container)
    if identifier in active_ids:
        return False
    active_ids.add(identifier)
    try:
        return all(_runtime_payload_is_safe(item, active_ids) for item in values)
    finally:
        active_ids.remove(identifier)


def publish_unique_mode(
    clustering: ModeClustering,
    qualifications: Sequence[ModeQualification],
) -> PublicationDecision:
    """Publish only the representative of exactly one admissible mode."""
    if not isinstance(clustering, ModeClustering):
        return _unavailable_publication("malformed_clustering_input")
    tolerance = _finite_scalar(clustering.tolerance_m)
    if (
        tolerance is None
        or tolerance <= 0.0
        or not isinstance(clustering.reason, str)
        or not clustering.reason
        or not isinstance(clustering.modes, tuple)
    ):
        return _unavailable_publication("malformed_clustering_input")
    if clustering.separable is not True:
        reason = (
            clustering.reason
            if isinstance(clustering.reason, str) and clustering.reason
            else "nonseparable_clustering"
        )
        return _unavailable_publication(reason)
    modes = clustering.modes
    if not isinstance(modes, tuple) or not modes:
        return _unavailable_publication("no_admissible_mode")
    try:
        mode_ids_list = []
        all_member_ids_list = []
        for mode in modes:
            _validate_mode_representative(mode, select_representative(mode))
            mode_ids_list.append(canonical_mode_id(mode))
            all_member_ids_list.extend(
                member.attempt_id for member in mode.members
            )
            actual_diameter = max(
                (
                    _distance(first, second)
                    for first in mode.members
                    for second in mode.members
                ),
                default=0.0,
            )
            if (
                actual_diameter > tolerance
                or abs(actual_diameter - mode.diameter_m) > CONTRACT_TOLERANCE
            ):
                raise ValueError("mode diameter is inconsistent")
        mode_ids = tuple(mode_ids_list)
        all_member_ids = tuple(all_member_ids_list)
    except (TypeError, ValueError):
        return _unavailable_publication("malformed_clustering_input")
    if (
        len(set(mode_ids)) != len(mode_ids)
        or len(set(all_member_ids)) != len(all_member_ids)
    ):
        return _unavailable_publication("malformed_clustering_input")
    if (
        not isinstance(qualifications, Sequence)
        or isinstance(qualifications, (str, bytes))
    ):
        return _unavailable_publication("malformed_qualification_input")
    qualification_tuple = tuple(qualifications)
    if len(qualification_tuple) != len(modes):
        return _unavailable_publication("malformed_qualification_input")
    if any(not _valid_mode_qualification(item) for item in qualification_tuple):
        return _unavailable_publication("malformed_qualification_input")
    qualification_ids = tuple(item.mode_id for item in qualification_tuple)
    if (
        len(set(qualification_ids)) != len(qualification_ids)
        or set(qualification_ids) != set(mode_ids)
    ):
        return _unavailable_publication("malformed_qualification_input")
    admissible = tuple(item for item in qualification_tuple if item.admissible)
    if not admissible:
        return _unavailable_publication("no_admissible_mode")
    if len(admissible) != 1:
        return _unavailable_publication("multiple_admissible_modes")
    selected_id = admissible[0].mode_id
    selected_mode = modes[mode_ids.index(selected_id)]
    return PublicationDecision(
        "fresh",
        "unique_admissible_mode",
        selected_id,
        select_representative(selected_mode),
    )


def _unavailable_publication(reason: str) -> PublicationDecision:
    return PublicationDecision("unavailable", reason, None, None)


def _valid_mode_qualification(value: object) -> bool:
    if (
        not isinstance(value, ModeQualification)
        or not isinstance(value.mode_id, str)
        or not value.mode_id
        or not isinstance(value.admissible, bool)
        or not isinstance(value.reason, str)
        or not value.reason
    ):
        return False
    return value.score is None or _finite_scalar(value.score) is not None


def _validated_deployment_contract(domain: object) -> DeploymentContract:
    if not isinstance(domain, DeploymentContract):
        raise ValueError("domain must be a DeploymentContract")
    ids = domain.anchor_ids
    if (
        not isinstance(ids, (tuple, list))
        or len(ids) != 2
        or any(
            isinstance(identifier, bool)
            or not isinstance(identifier, int)
            or identifier < 0
            for identifier in ids
        )
        or ids[0] == ids[1]
    ):
        raise ValueError("anchor IDs must be distinct nonnegative integers")
    coordinates = domain.anchor_coordinates
    if not isinstance(coordinates, (tuple, list)) or len(coordinates) != 2:
        raise ValueError("exactly two anchor coordinates are required")
    first = _strict_finite_point(coordinates[0])
    second = _strict_finite_point(coordinates[1])
    if first is None or second is None or first == second:
        raise ValueError("anchor coordinates must be finite and distinct")
    dx = second[0] - first[0]
    dy = second[1] - first[1]
    baseline = math.hypot(dx, dy)
    if not math.isfinite(baseline) or baseline <= 0.0:
        raise ValueError("anchor baseline must be finite and nonzero")
    expected_normal = (dy / baseline, -dx / baseline)
    unit_normal = _strict_finite_point(domain.unit_normal)
    if (
        unit_normal is None
        or abs(math.hypot(*unit_normal) - 1.0) > CONTRACT_TOLERANCE
        or any(
            abs(actual - expected) > CONTRACT_TOLERANCE
            for actual, expected in zip(unit_normal, expected_normal, strict=True)
        )
    ):
        raise ValueError("unit normal is inconsistent with ordered anchors")
    offset = _finite_scalar(domain.offset)
    expected_offset = -(unit_normal[0] * first[0] + unit_normal[1] * first[1])
    if offset is None or abs(offset - expected_offset) > CONTRACT_TOLERANCE:
        raise ValueError("offset is inconsistent with the anchor line")
    if domain.domain_version != "ocean-side-v1":
        raise ValueError("unsupported deployment-domain version")
    if (
        isinstance(domain.ocean_side, bool)
        or not isinstance(domain.ocean_side, int)
        or domain.ocean_side not in (-1, 1)
    ):
        raise ValueError("ocean_side must be exactly -1 or 1")
    margin = _finite_scalar(domain.margin_m)
    if margin is None or margin <= 0.0:
        raise ValueError("margin_m must be finite and strictly positive")
    vertices = domain.deployment_vertices
    if (
        not isinstance(vertices, (tuple, list))
        or not vertices
    ):
        raise ValueError("at least one deployment vertex is required")
    for raw_vertex in vertices:
        vertex = _strict_finite_point(raw_vertex)
        if vertex is None:
            raise ValueError("deployment vertices must be finite two-vectors")
        signed = domain.ocean_side * (
            unit_normal[0] * vertex[0]
            + unit_normal[1] * vertex[1]
            + offset
        )
        if not math.isfinite(signed) or signed < margin:
            raise ValueError("deployment region crosses the qualified half-plane")
    return domain


def _validate_mode_representative(
    mode: object,
    representative: object,
) -> None:
    if not isinstance(mode, NumericalMode) or not mode.members:
        raise ValueError("mode must contain at least one candidate")
    if not isinstance(representative, LocalCandidate):
        raise ValueError("representative must be a LocalCandidate")
    if any(not isinstance(member, LocalCandidate) for member in mode.members):
        raise ValueError("mode contains a malformed candidate")
    if len(mode.member_ids) != len(mode.members):
        raise ValueError("mode member IDs are incomplete")
    actual_ids = tuple(member.attempt_id for member in mode.members)
    if (
        any(not _valid_local_candidate(member) for member in mode.members)
        or len(set(actual_ids)) != len(actual_ids)
        or tuple(mode.member_ids) != tuple(sorted(actual_ids))
        or _finite_scalar(mode.diameter_m) is None
        or mode.diameter_m < 0.0
    ):
        raise ValueError("mode is malformed")
    selected = select_representative(mode)
    if (
        representative.attempt_id != selected.attempt_id
        or representative.estimate != selected.estimate
        or representative.objective_cost != selected.objective_cost
        or not _payloads_equal(representative.payload, selected.payload)
    ):
        raise ValueError("representative is inconsistent with its mode")


def _payloads_equal(first: object, second: object) -> bool:
    if first is second:
        return True
    if isinstance(first, Mapping) or isinstance(second, Mapping):
        if not isinstance(first, Mapping) or not isinstance(second, Mapping):
            return False
        if tuple(first.keys()) != tuple(second.keys()):
            return False
        return all(_payloads_equal(first[key], second[key]) for key in first)
    if isinstance(first, np.ndarray) or isinstance(second, np.ndarray):
        try:
            return bool(np.array_equal(np.asarray(first), np.asarray(second)))
        except (TypeError, ValueError):
            return False
    sequence_types = (tuple, list)
    if isinstance(first, sequence_types) or isinstance(second, sequence_types):
        if not isinstance(first, sequence_types) or not isinstance(second, sequence_types):
            return False
        return len(first) == len(second) and all(
            _payloads_equal(left, right)
            for left, right in zip(first, second, strict=True)
        )
    try:
        equal = first == second
    except (TypeError, ValueError):
        return False
    return isinstance(equal, (bool, np.bool_)) and bool(equal)


def _valid_local_candidate(candidate: object) -> bool:
    return bool(
        isinstance(candidate, LocalCandidate)
        and isinstance(candidate.attempt_id, str)
        and candidate.attempt_id
        and _strict_finite_point(candidate.estimate) is not None
        and (cost := _finite_scalar(candidate.objective_cost)) is not None
        and cost >= 0.0
        and isinstance(candidate.payload, Mapping)
        and not _contains_forbidden_runtime_key(candidate.payload)
    )


def _canonical_local_candidate(candidate: object) -> LocalCandidate | None:
    if not _valid_local_candidate(candidate):
        return None
    estimate = _strict_finite_point(candidate.estimate)
    cost = _finite_scalar(candidate.objective_cost)
    if estimate is None or cost is None:
        return None
    return LocalCandidate(
        candidate.attempt_id,
        estimate,
        cost,
        candidate.payload,
    )


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
        analytic: list[QualifiedStart] = []
        for branch, estimate in _circle_branches(first, second):
            analytic.append(QualifiedStart("circle", estimate, pair_keys, branch))
        # Recursive-prior repair: with exactly two references the two circle
        # branches are ambiguous; include the live/private prior seeds so the
        # previous-frame estimate can participate in mode selection.
        for kind, seed in (("live", live_seed), ("private", private_seed)):
            estimate = _seed_estimate(seed)
            if estimate is not None:
                analytic.append(QualifiedStart(kind, estimate, reference_keys))
        unique: list[QualifiedStart] = []
        for start in sorted(analytic, key=stable_attempt_id):
            if all(
                _point_distance(start.estimate, kept.estimate) > 1e-9
                for kept in unique
            ):
                unique.append(start)
        return tuple(unique)
    unique: list[QualifiedStart] = []
    for start in sorted(tentative, key=stable_attempt_id):
        if all(_point_distance(start.estimate, kept.estimate) > 1e-9 for kept in unique):
            unique.append(start)
    return tuple(unique)


def _finite_scalar(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    try:
        scalar = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return scalar if math.isfinite(scalar) else None


def _finite_point(value: object) -> tuple[float, float] | None:
    try:
        point = tuple(float(item) for item in value)
    except (TypeError, ValueError, OverflowError):
        return None
    if len(point) != 2 or not all(math.isfinite(item) for item in point):
        return None
    return point


def _strict_finite_point(value: object) -> tuple[float, float] | None:
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        return None
    coordinates = tuple(_finite_scalar(item) for item in value)
    if any(item is None for item in coordinates):
        return None
    return coordinates


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
    if negative == positive:
        return (("negative", negative),)
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
    canonical = 0.5 * (covariance + covariance.T)
    try:
        eigenvalues = np.linalg.eigvalsh(canonical)
    except np.linalg.LinAlgError:
        return None
    return estimate if np.all(np.isfinite(eigenvalues)) and np.all(eigenvalues > 0.0) else None


def _point_distance(first: tuple[float, float], second: tuple[float, float]) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])
