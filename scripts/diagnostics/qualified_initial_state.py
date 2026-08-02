"""Deterministic admission for the CBF2026 v5 initial-state family.

This module intentionally has no campaign-launch behavior.  It materializes
one declared seed, reconstructs the production FIM/radius and hard-row
formulas, and either returns the exact audit or rejects that same seed.
"""

from __future__ import annotations

import copy
import hashlib
import itertools
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping, Sequence

import numpy as np


EXPECTED_TEMPLATE = (
    (-1405.6519845832192, -181.57557542683088),
    (-1405.651984583215, 181.57557542682423),
    (-1370.5, 0.0),
    (-1486.517149260088, 62.21755876564317),
    (-1383.3900347553965, -123.69189439925512),
    (-1476.6099652446042, -123.69189439925204),
    (-1454.3480154167842, -181.5755754268258),
    (-1383.3900347553965, 123.691894399255),
    (-1373.482850739912, 62.2175587656418),
    (-1476.6099652446028, 123.69189439925789),
    (-1373.4828507399125, -62.21755876564655),
    (-1486.5171492600884, -62.217558765639424),
    (-1489.5, 1.1732292571851443e-12),
    (-1454.348015416783, 181.57557542682727),
)
EXPECTED_REGISTERED_SEEDS = tuple(range(2026080201, 2026080211))
EXPECTED_AUDIT_SEEDS = tuple(range(2026080201, 2026080301))


@dataclass(frozen=True, order=True)
class EdgeId:
    kind: str
    low: int
    high: int
    base_id: int = -1

    def token(self) -> tuple[str, int, int, int]:
        return (self.kind, self.low, self.high, self.base_id)

    def reference_token(self) -> tuple[str, int]:
        if self.kind != "localization":
            raise ValueError("collision edge has no fixed reference token")
        return (
            ("base", self.base_id)
            if self.base_id >= 0
            else ("uav", self.low)
        )


@dataclass(frozen=True)
class NodeCertificate:
    robot_id: int
    reference_ids: tuple[tuple[str, int], ...]
    covariance: tuple[tuple[float, float], tuple[float, float]]
    covariance_rate_bound: float
    epsilon: float
    bar_nu: float


@dataclass(frozen=True)
class BarrierValue:
    edge: EdgeId
    value: float


@dataclass(frozen=True)
class EndpointRow:
    edge: EdgeId
    owner: int
    coefficient: tuple[float, float]
    constant: float


@dataclass(frozen=True)
class LocalQpAdmission:
    robot_id: int
    margin: float
    witness: tuple[float, float]
    tight_rows: tuple[EndpointRow, ...]


@dataclass(frozen=True)
class InitialStateAudit:
    seed: int | None
    positions: tuple[tuple[float, float], ...]
    positions_sha256: str
    certificates: tuple[NodeCertificate, ...]
    barriers: tuple[BarrierValue, ...]
    endpoint_rows: tuple[EndpointRow, ...]
    local_qps: tuple[LocalQpAdmission, ...]
    accepted: bool
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class BarrierSeedMetric:
    value: float
    seed: int
    edge: EdgeId


@dataclass(frozen=True)
class QpSeedMetric:
    value: float
    seed: int
    robot_id: int
    witness: tuple[float, float]
    active_edges: tuple[EdgeId, ...]


@dataclass(frozen=True)
class NodeSeedMetric:
    value: float
    seed: int
    robot_id: int


@dataclass(frozen=True)
class PairSeedMetric:
    value: float
    seed: int
    edge: tuple[int, int]


@dataclass(frozen=True)
class SeedSetSummary:
    proposed_count: int
    accepted_count: int
    rejected_seeds: tuple[int, ...]
    minimum_barrier: BarrierSeedMetric
    minimum_qp_margin: QpSeedMetric
    maximum_bar_nu: NodeSeedMetric
    minimum_pair_distance: PairSeedMetric


@dataclass(frozen=True)
class SeedSetAudit:
    audits: tuple[InitialStateAudit, ...]
    summary: SeedSetSummary

    def subset(self, seeds: Iterable[int]) -> "SeedSetAudit":
        requested = _strict_seed_tuple(seeds)
        by_seed = {audit.seed: audit for audit in self.audits}
        if any(seed not in by_seed for seed in requested):
            raise ValueError("subset seed is absent from the audited universe")
        selected = tuple(by_seed[seed] for seed in requested)
        return SeedSetAudit(selected, _summarize_seed_audits(selected))


@dataclass(frozen=True)
class FrozenInitialFamilyAudit:
    registered: SeedSetAudit
    audit: SeedSetAudit


class InitialStateAdmissionError(ValueError):
    def __init__(self, audit: InitialStateAudit):
        self.audit = audit
        super().__init__(
            "initial state is not admitted: " + ", ".join(audit.reasons)
        )


def canonical_json_bytes(value) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def family_semantic_sha256(family: Mapping) -> str:
    if not isinstance(family, Mapping):
        raise ValueError("initial family must be an object")
    semantic = {key: value for key, value in family.items() if key != "semantic_sha256"}
    return hashlib.sha256(canonical_json_bytes(semantic)).hexdigest()


def canonical_positions_sha256(
    positions: Sequence[Sequence[float]],
) -> str:
    normalized = _normalize_positions(positions)
    return hashlib.sha256(canonical_json_bytes(normalized)).hexdigest()


def _keys(value, expected: set[str], label: str) -> None:
    if not isinstance(value, Mapping) or set(value) != expected:
        raise ValueError(f"{label} has an invalid exact schema")


def _number(value, expected: float, label: str) -> None:
    if (
        type(value) not in (int, float)
        or not math.isfinite(float(value))
        or value != expected
    ):
        raise ValueError(f"{label} differs from the frozen numeric contract")


def _integer(value, expected: int, label: str) -> None:
    if type(value) is not int or value != expected:
        raise ValueError(f"{label} differs from the frozen integer contract")


def _boolean(value, expected: bool, label: str) -> None:
    if type(value) is not bool or value is not expected:
        raise ValueError(f"{label} differs from the frozen Boolean contract")


def _exact_numeric_matrix(value, expected, label: str) -> None:
    if type(value) is not list or len(value) != len(expected):
        raise ValueError(f"{label} has an invalid row universe")
    for row_index, (actual_row, expected_row) in enumerate(zip(value, expected)):
        if type(actual_row) is not list or len(actual_row) != len(expected_row):
            raise ValueError(f"{label}[{row_index}] has an invalid coordinate shape")
        for column, (actual, target) in enumerate(zip(actual_row, expected_row)):
            _number(actual, target, f"{label}[{row_index}][{column}]")


def validate_qualified_initial_family(family: Mapping) -> dict:
    """Return a deep copy only when the complete v1 family is exact."""
    if not isinstance(family, Mapping):
        raise ValueError("initial family must be an object")
    _keys(
        family,
        {
            "schema_version",
            "namespace",
            "template_positions_m",
            "perturbation",
            "deployment",
            "schedule",
            "production_contract",
            "admission",
            "frozen_summary",
            "semantic_sha256",
        },
        "initial family",
    )
    semantic_sha = family["semantic_sha256"]
    if (
        type(semantic_sha) is not str
        or len(semantic_sha) != 64
        or any(character not in "0123456789abcdef" for character in semantic_sha)
        or semantic_sha != family_semantic_sha256(family)
    ):
        raise ValueError("initial family semantic SHA-256 does not match")
    if family["schema_version"] != "cbf2026-qualified-initial-family-v1":
        raise ValueError("initial family schema version is not frozen")
    if family["namespace"] != "cbf2026-v5-initial":
        raise ValueError("initial family namespace is not frozen")
    _exact_numeric_matrix(
        family["template_positions_m"], EXPECTED_TEMPLATE, "template_positions_m"
    )

    perturbation = family["perturbation"]
    _keys(
        perturbation,
        {"method", "coordinate_radius_m", "clamp", "resample"},
        "perturbation",
    )
    if perturbation["method"] != "sha256-first8-uint64-be-affine-v1":
        raise ValueError("perturbation method is not frozen")
    _number(perturbation["coordinate_radius_m"], 0.1, "coordinate radius")
    _boolean(perturbation["clamp"], False, "perturbation clamp")
    _boolean(perturbation["resample"], False, "perturbation resample")

    deployment = family["deployment"]
    _keys(
        deployment,
        {"polygon_vertices_m", "world_boundary_m"},
        "deployment",
    )
    _exact_numeric_matrix(
        deployment["polygon_vertices_m"],
        ((-1490.0, -200.0), (-1490.0, 200.0),
         (-1370.0, 200.0), (-1370.0, -200.0)),
        "deployment polygon",
    )
    _exact_numeric_matrix(
        deployment["world_boundary_m"],
        ((-1500.0, -1500.0), (1500.0, -1500.0),
         (1500.0, 1500.0), (-1500.0, 1500.0)),
        "world boundary",
    )

    schedule = family["schedule"]
    _keys(
        schedule,
        {
            "registered_trajectory_seeds",
            "audit_seed_first",
            "audit_seed_last",
            "audit_seed_count",
        },
        "schedule",
    )
    if (
        type(schedule["registered_trajectory_seeds"]) is not list
        or any(type(seed) is not int for seed in schedule["registered_trajectory_seeds"])
        or tuple(schedule["registered_trajectory_seeds"])
        != EXPECTED_REGISTERED_SEEDS
    ):
        raise ValueError("registered trajectory seed sequence is not frozen")
    _integer(schedule["audit_seed_first"], EXPECTED_AUDIT_SEEDS[0], "audit first seed")
    _integer(schedule["audit_seed_last"], EXPECTED_AUDIT_SEEDS[-1], "audit last seed")
    _integer(schedule["audit_seed_count"], len(EXPECTED_AUDIT_SEEDS), "audit seed count")

    contract = family["production_contract"]
    _keys(
        contract,
        {
            "base_positions_m",
            "formation_base_ids",
            "squad_count",
            "uavs_per_squad",
            "ranging_sigma_m",
            "reference_max_range_m",
            "safe_distance_m",
            "class_k_coefficient",
            "uav_endpoint_allocation",
            "base_endpoint_allocation",
            "planar_component_max_mps",
        },
        "production contract",
    )
    _exact_numeric_matrix(
        contract["base_positions_m"],
        ((-1550.0, -300.0), (-1550.0, 0.0), (-1550.0, 300.0)),
        "base positions",
    )
    if contract["formation_base_ids"] != [[1, 0], [1, 2]] or any(
        type(value) is not int
        for row in contract["formation_base_ids"]
        for value in row
    ):
        raise ValueError("formation base IDs are not frozen")
    _integer(contract["squad_count"], 2, "squad count")
    _integer(contract["uavs_per_squad"], 7, "UAVs per squad")
    for key, expected in (
        ("ranging_sigma_m", 0.5),
        ("reference_max_range_m", 850.0),
        ("safe_distance_m", 10.0),
        ("class_k_coefficient", 0.1),
        ("uav_endpoint_allocation", 0.5),
        ("base_endpoint_allocation", 1.0),
        ("planar_component_max_mps", 25.0),
    ):
        _number(contract[key], expected, f"production contract {key}")

    admission = family["admission"]
    _keys(
        admission,
        {
            "minimum_barrier_m",
            "minimum_qp_margin_mps",
            "fixed_localization_edge_count",
            "collision_edge_count",
            "barrier_count",
            "endpoint_row_count",
            "local_qp_count",
        },
        "admission",
    )
    _number(admission["minimum_barrier_m"], 35.0, "minimum barrier")
    _number(admission["minimum_qp_margin_mps"], 0.7, "minimum QP margin")
    for key, expected in (
        ("fixed_localization_edge_count", 28),
        ("collision_edge_count", 91),
        ("barrier_count", 119),
        ("endpoint_row_count", 232),
        ("local_qp_count", 14),
    ):
        _integer(admission[key], expected, f"admission {key}")

    _validate_frozen_summary(family["frozen_summary"])
    return copy.deepcopy(dict(family))


def _validate_frozen_summary(summary) -> None:
    _keys(
        summary,
        {
            "registered",
            "audit",
            "representative_seed",
            "representative_positions_sha256",
        },
        "frozen summary",
    )
    expected = {
        "registered": {
            "proposed_count": 10,
            "accepted_count": 10,
            "minimum_barrier_m": 35.77296640879953,
            "minimum_barrier_seed": 2026080205,
            "minimum_barrier_edge": ["collision", 2, 14, -1],
            "minimum_qp_margin_mps": 0.7658252531927233,
            "minimum_qp_seed": 2026080207,
            "minimum_qp_uav": 13,
            "maximum_bar_nu_mps": 6.768799189292255,
            "maximum_bar_nu_seed": 2026080204,
            "maximum_bar_nu_uav": 14,
            "minimum_pair_distance_m": 48.53866222462101,
            "minimum_pair_distance_seed": 2026080205,
            "minimum_pair_distance_edge": [2, 14],
        },
        "audit": {
            "proposed_count": 100,
            "accepted_count": 100,
            "minimum_barrier_m": 35.77296640879953,
            "minimum_barrier_seed": 2026080205,
            "minimum_barrier_edge": ["collision", 2, 14, -1],
            "minimum_qp_margin_mps": 0.7658252531927233,
            "minimum_qp_seed": 2026080207,
            "minimum_qp_uav": 13,
            "maximum_bar_nu_mps": 6.7773637849535655,
            "maximum_bar_nu_seed": 2026080253,
            "maximum_bar_nu_uav": 14,
            "minimum_pair_distance_m": 48.51563216720877,
            "minimum_pair_distance_seed": 2026080246,
            "minimum_pair_distance_edge": [1, 7],
        },
    }
    for label in ("registered", "audit"):
        actual = summary[label]
        target = expected[label]
        _keys(actual, set(target), f"frozen summary {label}")
        for key, value in target.items():
            observed = actual[key]
            if type(value) is int:
                _integer(observed, value, f"frozen summary {label}.{key}")
            elif type(value) is float:
                _number(observed, value, f"frozen summary {label}.{key}")
            else:
                if type(observed) is not list or len(observed) != len(value):
                    raise ValueError(
                        f"frozen summary {label}.{key} is not frozen"
                    )
                for index, (actual_item, expected_item) in enumerate(
                    zip(observed, value)
                ):
                    item_label = f"frozen summary {label}.{key}[{index}]"
                    if type(expected_item) is int:
                        _integer(actual_item, expected_item, item_label)
                    elif type(actual_item) is not str or actual_item != expected_item:
                        raise ValueError(f"{item_label} is not frozen")
    _integer(summary["representative_seed"], 2026080201, "representative seed")
    if summary["representative_positions_sha256"] != (
        "161fb8a9104b7b5b0a3a20cd5cf0e9c896db98e74ee2262088751edddaa72e88"
    ):
        raise ValueError("representative position SHA-256 is not frozen")


def load_qualified_initial_family(path: Path) -> dict:
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ValueError("initial family must be a regular file")

    def reject_duplicate_keys(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result

    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=reject_duplicate_keys,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError("initial family is not valid UTF-8 JSON") from error
    return validate_qualified_initial_family(value)


def _normalize_positions(positions) -> tuple[tuple[float, float], ...]:
    if type(positions) not in (list, tuple) or len(positions) != 14:
        raise ValueError("initial state requires exactly 14 positions")
    normalized = []
    for robot_index, position in enumerate(positions, start=1):
        if type(position) not in (list, tuple) or len(position) != 2:
            raise ValueError(f"UAV {robot_index} position must have two coordinates")
        if any(
            type(value) not in (int, float) or not math.isfinite(float(value))
            for value in position
        ):
            raise ValueError(f"UAV {robot_index} position is not finite numeric data")
        normalized.append((float(position[0]), float(position[1])))
    return tuple(normalized)


def _strict_seed_tuple(seeds: Iterable[int]) -> tuple[int, ...]:
    result = tuple(seeds)
    if not result or any(type(seed) is not int for seed in result):
        raise ValueError("seed universe must be nonempty exact integers")
    if len(set(result)) != len(result):
        raise ValueError("seed universe contains duplicates")
    return result


def _materialize_seed_positions(family: Mapping, seed: int) -> tuple[tuple[float, float], ...]:
    if type(seed) is not int:
        raise ValueError("trajectory seed must be an exact integer")
    namespace = family["namespace"]
    radius = float(family["perturbation"]["coordinate_radius_m"])
    result = []
    for robot_id, nominal in enumerate(family["template_positions_m"], start=1):
        coordinates = []
        for axis in range(2):
            payload = f"{namespace}|{seed}|{robot_id}|{axis}".encode("ascii")
            integer = int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")
            unit = integer / (2**64 - 1)
            coordinates.append(float(nominal[axis]) + radius * (2.0 * unit - 1.0))
        result.append(tuple(coordinates))
    return tuple(result)


def materialize_seed_positions(
    family: Mapping,
    seed: int,
) -> tuple[tuple[float, float], ...]:
    checked = validate_qualified_initial_family(family)
    if type(seed) is not int:
        raise ValueError("trajectory seed must be an exact integer")
    if seed not in checked["schedule"]["registered_trajectory_seeds"]:
        raise ValueError("trajectory seed is not a registered trajectory seed")
    return _materialize_seed_positions(checked, seed)


def _local_index(robot_id: int) -> int:
    return (robot_id - 1) % 7 + 1


def _squad(robot_id: int) -> int:
    return (robot_id - 1) // 7


def _fixed_reference_ids(family: Mapping, robot_id: int) -> tuple[tuple[str, int], ...]:
    if type(robot_id) is not int or robot_id < 1 or robot_id > 14:
        raise ValueError("robot ID is outside the 14-UAV universe")
    local = _local_index(robot_id)
    base_ids = family["production_contract"]["formation_base_ids"][_squad(robot_id)]
    maximum_base_index = 2 - local
    references = [
        ("base", base_id)
        for index, base_id in enumerate(base_ids)
        if index <= maximum_base_index
    ]
    first = 1 + 7 * _squad(robot_id)
    references.extend(
        ("uav", other_id)
        for other_id in range(max(first, robot_id - 2), robot_id)
    )
    return tuple(references)


def fixed_localization_edges(family: Mapping) -> tuple[EdgeId, ...]:
    checked = validate_qualified_initial_family(family)
    edges = set()
    for robot_id in range(1, 15):
        for kind, reference_id in _fixed_reference_ids(checked, robot_id):
            edges.add(
                EdgeId("localization", robot_id, robot_id, reference_id)
                if kind == "base"
                else EdgeId(
                    "localization",
                    min(robot_id, reference_id),
                    max(robot_id, reference_id),
                    -1,
                )
            )
    return tuple(sorted(edges, key=lambda edge: (edge.low, edge.high, edge.base_id)))


def collision_edges(family: Mapping) -> tuple[EdgeId, ...]:
    validate_qualified_initial_family(family)
    return tuple(
        EdgeId("collision", low, high, -1)
        for low in range(1, 15)
        for high in range(low + 1, 15)
    )


def _dynamic_fim_reference_ids(
    family: Mapping,
    positions: tuple[tuple[float, float], ...],
    robot_id: int,
) -> tuple[tuple[str, int], ...]:
    references = set(_fixed_reference_ids(family, robot_id))
    point = np.asarray(positions[robot_id - 1], dtype=float)
    maximum_range = float(
        family["production_contract"]["reference_max_range_m"]
    )
    for base_id, base in enumerate(family["production_contract"]["base_positions_m"]):
        if np.linalg.norm(point - np.asarray(base, dtype=float)) <= maximum_range:
            references.add(("base", base_id))
    first = 1 + 7 * _squad(robot_id)
    for other_id in range(first, robot_id):
        if np.linalg.norm(point - np.asarray(positions[other_id - 1])) <= maximum_range:
            references.add(("uav", other_id))
    return tuple(
        sorted(
            references,
            key=lambda item: (
                0 if item[0] == "base" else 1,
                item[1] if item[0] == "base" else _local_index(item[1]),
            ),
        )
    )


def dynamic_fim_reference_ids(
    family: Mapping,
    positions: Sequence[Sequence[float]],
    robot_id: int,
) -> tuple[tuple[str, int], ...]:
    checked = validate_qualified_initial_family(family)
    return _dynamic_fim_reference_ids(checked, _normalize_positions(positions), robot_id)


def _compute_certificates(
    family: Mapping,
    positions: tuple[tuple[float, float], ...],
) -> tuple[NodeCertificate, ...]:
    contract = family["production_contract"]
    bases = tuple(np.asarray(base, dtype=float) for base in contract["base_positions_m"])
    sigma = float(contract["ranging_sigma_m"])
    component_max = float(contract["planar_component_max_mps"])
    speed_bound = math.sqrt(2.0) * component_max
    internal = {}
    public = {}
    for robot_id in (*range(1, 8), *range(8, 15)):
        point = np.asarray(positions[robot_id - 1], dtype=float)
        reference_ids = _dynamic_fim_reference_ids(family, positions, robot_id)
        information = np.zeros((2, 2), dtype=float)
        information_rate_bound = 0.0
        for kind, reference_id in reference_ids:
            if kind == "base":
                reference_position = bases[reference_id]
                predecessor_covariance = np.zeros((2, 2), dtype=float)
                predecessor_covariance_rate = 0.0
                predecessor_speed = 0.0
            else:
                predecessor = internal[reference_id]
                reference_position = np.asarray(positions[reference_id - 1], dtype=float)
                predecessor_covariance = predecessor["covariance"]
                predecessor_covariance_rate = predecessor["covariance_rate"]
                predecessor_speed = speed_bound
            displacement = point - reference_position
            distance = float(np.linalg.norm(displacement))
            if not math.isfinite(distance) or distance <= 1e-8:
                raise ValueError(f"UAV {robot_id} has a singular active reference")
            direction = displacement / distance
            predecessor_norm = float(
                np.max(np.abs(np.linalg.eigvalsh(predecessor_covariance)))
            )
            effective_variance = float(
                direction @ predecessor_covariance @ direction + sigma * sigma
            )
            beta = (speed_bound + predecessor_speed) / distance
            effective_variance_rate = (
                2.0 * beta * predecessor_norm + predecessor_covariance_rate
            )
            information += np.outer(direction, direction) / effective_variance
            information_rate_bound += (
                effective_variance_rate / effective_variance**2
                + 2.0 * beta / effective_variance
            )
        information_eigenvalues = np.linalg.eigvalsh(information)
        if (
            information_eigenvalues[-1] <= 0.0
            or information_eigenvalues[0]
            <= 1e-12 * information_eigenvalues[-1]
        ):
            raise ValueError(f"UAV {robot_id} FIM is not positive definite")
        covariance = np.linalg.inv(information)
        covariance = 0.5 * (covariance + covariance.T)
        covariance_eigenvalues = np.linalg.eigvalsh(covariance)
        covariance_norm = float(np.max(np.abs(covariance_eigenvalues)))
        covariance_rate = covariance_norm**2 * information_rate_bound
        maximum_covariance = float(covariance_eigenvalues[-1])
        epsilon = 3.0 * math.sqrt(maximum_covariance)
        bar_nu = 3.0 * covariance_rate / (2.0 * math.sqrt(maximum_covariance))
        internal[robot_id] = {
            "covariance": covariance,
            "covariance_rate": covariance_rate,
        }
        public[robot_id] = NodeCertificate(
            robot_id=robot_id,
            reference_ids=reference_ids,
            covariance=(
                (float(covariance[0, 0]), float(covariance[0, 1])),
                (float(covariance[1, 0]), float(covariance[1, 1])),
            ),
            covariance_rate_bound=float(covariance_rate),
            epsilon=float(epsilon),
            bar_nu=float(bar_nu),
        )
    return tuple(public[robot_id] for robot_id in range(1, 15))


def _barriers_and_rows(
    family: Mapping,
    positions: tuple[tuple[float, float], ...],
    certificates: tuple[NodeCertificate, ...],
) -> tuple[tuple[BarrierValue, ...], tuple[EndpointRow, ...]]:
    contract = family["production_contract"]
    bases = tuple(np.asarray(base, dtype=float) for base in contract["base_positions_m"])
    maximum_range = float(contract["reference_max_range_m"])
    safe_distance = float(contract["safe_distance_m"])
    class_k = float(contract["class_k_coefficient"])
    uav_allocation = float(contract["uav_endpoint_allocation"])
    base_allocation = float(contract["base_endpoint_allocation"])
    edges = (*_fixed_localization_edges_unchecked(family), *_collision_edges_unchecked())
    barriers = []
    rows = []
    for edge in edges:
        first_position = np.asarray(positions[edge.low - 1], dtype=float)
        first = certificates[edge.low - 1]
        if edge.kind == "localization" and edge.base_id >= 0:
            second_position = bases[edge.base_id]
            second_epsilon = 0.0
            second_bar_nu = 0.0
            threshold = maximum_range
        else:
            second_position = np.asarray(positions[edge.high - 1], dtype=float)
            second = certificates[edge.high - 1]
            second_epsilon = second.epsilon
            second_bar_nu = second.bar_nu
            threshold = maximum_range if edge.kind == "localization" else safe_distance
        displacement = first_position - second_position
        separation = float(np.linalg.norm(displacement))
        if not math.isfinite(separation) or separation <= 1e-8:
            raise ValueError("hard edge has singular geometry")
        normal = displacement / separation
        if edge.kind == "localization":
            barrier = threshold - separation - first.epsilon - second_epsilon
            alpha = class_k * barrier
            rows.append(
                EndpointRow(
                    edge, edge.low,
                    (-float(normal[0]), -float(normal[1])),
                    -first.bar_nu
                    + (base_allocation if edge.base_id >= 0 else uav_allocation)
                    * alpha,
                )
            )
            if edge.base_id < 0:
                rows.append(
                    EndpointRow(
                        edge, edge.high,
                        (float(normal[0]), float(normal[1])),
                        -second_bar_nu + uav_allocation * alpha,
                    )
                )
        else:
            barrier = separation - threshold - first.epsilon - second_epsilon
            alpha = class_k * barrier
            rows.extend(
                (
                    EndpointRow(
                        edge, edge.low,
                        (float(normal[0]), float(normal[1])),
                        -first.bar_nu + uav_allocation * alpha,
                    ),
                    EndpointRow(
                        edge, edge.high,
                        (-float(normal[0]), -float(normal[1])),
                        -second_bar_nu + uav_allocation * alpha,
                    ),
                )
            )
        barriers.append(BarrierValue(edge, float(barrier)))
    return tuple(barriers), tuple(rows)


def _fixed_localization_edges_unchecked(family: Mapping) -> tuple[EdgeId, ...]:
    edges = set()
    for robot_id in range(1, 15):
        for kind, reference_id in _fixed_reference_ids(family, robot_id):
            if kind == "base":
                edges.add(EdgeId("localization", robot_id, robot_id, reference_id))
            else:
                edges.add(
                    EdgeId(
                        "localization",
                        min(robot_id, reference_id),
                        max(robot_id, reference_id),
                        -1,
                    )
                )
    return tuple(sorted(edges, key=lambda edge: (edge.low, edge.high, edge.base_id)))


def _collision_edges_unchecked() -> tuple[EdgeId, ...]:
    return tuple(
        EdgeId("collision", low, high, -1)
        for low in range(1, 15)
        for high in range(low + 1, 15)
    )


def _solve_max_min_lp(
    robot_id: int,
    rows: tuple[EndpointRow, ...],
    component_max: float,
) -> LocalQpAdmission:
    # A row is a^T u + c >= t.  Every plane below is stored as A x <= b
    # for x=(u_x,u_y,t).  Enumerating triples is exact for this 3-D LP.
    planes = [
        (
            np.asarray((-row.coefficient[0], -row.coefficient[1], 1.0)),
            row.constant,
        )
        for row in rows
    ]
    planes.extend(
        (
            (np.asarray((1.0, 0.0, 0.0)), component_max),
            (np.asarray((-1.0, 0.0, 0.0)), component_max),
            (np.asarray((0.0, 1.0, 0.0)), component_max),
            (np.asarray((0.0, -1.0, 0.0)), component_max),
        )
    )
    best = None
    for active in itertools.combinations(range(len(planes)), 3):
        matrix = np.asarray([planes[index][0] for index in active])
        target = np.asarray([planes[index][1] for index in active])
        try:
            candidate = np.linalg.solve(matrix, target)
        except np.linalg.LinAlgError:
            continue
        if all(plane @ candidate <= bound + 1e-8 for plane, bound in planes):
            if best is None or candidate[2] > best[2]:
                best = candidate
    if best is None:
        raise ValueError(f"UAV {robot_id} max-min LP has no finite vertex")
    residuals = [
        (
            float(np.asarray(row.coefficient) @ best[:2] + row.constant),
            row,
        )
        for row in rows
    ]
    residuals.sort(
        key=lambda item: (item[0], item[1].edge.token(), item[1].owner)
    )
    margin = float(best[2])
    tight = tuple(row for residual, row in residuals if abs(residual - margin) <= 1e-8)
    return LocalQpAdmission(
        robot_id,
        margin,
        (float(best[0]), float(best[1])),
        tight,
    )


def _point_in_frozen_rectangles(family: Mapping, point: tuple[float, float]) -> bool:
    polygon = family["deployment"]["polygon_vertices_m"]
    world = family["deployment"]["world_boundary_m"]
    return (
        min(vertex[0] for vertex in polygon) <= point[0] <= max(vertex[0] for vertex in polygon)
        and min(vertex[1] for vertex in polygon) <= point[1] <= max(vertex[1] for vertex in polygon)
        and min(vertex[0] for vertex in world) <= point[0] <= max(vertex[0] for vertex in world)
        and min(vertex[1] for vertex in world) <= point[1] <= max(vertex[1] for vertex in world)
    )


def _audit_positions(
    family: Mapping,
    positions: tuple[tuple[float, float], ...],
    *,
    seed: int | None,
) -> InitialStateAudit:
    certificates = _compute_certificates(family, positions)
    barriers, rows = _barriers_and_rows(family, positions, certificates)
    rows_by_robot = tuple(
        tuple(row for row in rows if row.owner == robot_id)
        for robot_id in range(1, 15)
    )
    component_max = float(
        family["production_contract"]["planar_component_max_mps"]
    )
    local_qps = tuple(
        _solve_max_min_lp(robot_id, rows_by_robot[robot_id - 1], component_max)
        for robot_id in range(1, 15)
    )
    admission = family["admission"]
    reasons = []
    for robot_id, point in enumerate(positions, start=1):
        if not _point_in_frozen_rectangles(family, point):
            reasons.append(f"deployment:{robot_id}")
    if len(_fixed_localization_edges_unchecked(family)) != admission["fixed_localization_edge_count"]:
        reasons.append("fixed_localization_edge_count")
    if len(_collision_edges_unchecked()) != admission["collision_edge_count"]:
        reasons.append("collision_edge_count")
    if len(barriers) != admission["barrier_count"]:
        reasons.append("barrier_count")
    if len(rows) != admission["endpoint_row_count"]:
        reasons.append("endpoint_row_count")
    if len(local_qps) != admission["local_qp_count"]:
        reasons.append("local_qp_count")
    for barrier in barriers:
        if barrier.value < admission["minimum_barrier_m"]:
            reasons.append(
                f"barrier:{barrier.edge.kind}:{barrier.edge.low}:"
                f"{barrier.edge.high}:{barrier.edge.base_id}:{barrier.value:.17g}"
            )
    for qp in local_qps:
        if qp.margin < admission["minimum_qp_margin_mps"]:
            reasons.append(f"qp_margin:{qp.robot_id}:{qp.margin:.17g}")
    return InitialStateAudit(
        seed=seed,
        positions=positions,
        positions_sha256=canonical_positions_sha256(positions),
        certificates=certificates,
        barriers=barriers,
        endpoint_rows=rows,
        local_qps=local_qps,
        accepted=not reasons,
        reasons=tuple(reasons),
    )


def audit_positions(
    family: Mapping,
    positions: Sequence[Sequence[float]],
) -> InitialStateAudit:
    checked = validate_qualified_initial_family(family)
    return _audit_positions(checked, _normalize_positions(positions), seed=None)


def audit_seed(
    family: Mapping,
    seed: int,
) -> InitialStateAudit:
    checked = validate_qualified_initial_family(family)
    if type(seed) is not int:
        raise ValueError("trajectory seed must be an exact integer")
    if seed not in checked["schedule"]["registered_trajectory_seeds"]:
        raise ValueError("trajectory seed is not a registered trajectory seed")
    return _audit_positions(
        checked, _materialize_seed_positions(checked, seed), seed=seed
    )


def require_admitted_seed(
    family: Mapping,
    seed: int,
) -> InitialStateAudit:
    audit = audit_seed(family, seed)
    if not audit.accepted:
        raise InitialStateAdmissionError(audit)
    return audit


def _minimum_pair_distance(
    positions: tuple[tuple[float, float], ...],
) -> tuple[float, tuple[int, int]]:
    return min(
        (
            math.dist(positions[first], positions[second]),
            (first + 1, second + 1),
        )
        for first in range(14)
        for second in range(first + 1, 14)
    )


def _summarize_seed_audits(
    audits: tuple[InitialStateAudit, ...],
) -> SeedSetSummary:
    if not audits or any(audit.seed is None for audit in audits):
        raise ValueError("seed-set summary requires seeded audits")
    minimum_barrier = None
    minimum_qp = None
    maximum_bar_nu = None
    minimum_distance = None
    for audit in audits:
        seed = int(audit.seed)
        barrier = min(audit.barriers, key=lambda item: item.value)
        barrier_metric = BarrierSeedMetric(barrier.value, seed, barrier.edge)
        if minimum_barrier is None or barrier_metric.value < minimum_barrier.value:
            minimum_barrier = barrier_metric
        qp = min(audit.local_qps, key=lambda item: item.margin)
        qp_metric = QpSeedMetric(
            qp.margin,
            seed,
            qp.robot_id,
            qp.witness,
            tuple(row.edge for row in qp.tight_rows),
        )
        if minimum_qp is None or qp_metric.value < minimum_qp.value:
            minimum_qp = qp_metric
        node = max(audit.certificates, key=lambda item: item.bar_nu)
        node_metric = NodeSeedMetric(node.bar_nu, seed, node.robot_id)
        if maximum_bar_nu is None or node_metric.value > maximum_bar_nu.value:
            maximum_bar_nu = node_metric
        distance, edge = _minimum_pair_distance(audit.positions)
        distance_metric = PairSeedMetric(distance, seed, edge)
        if minimum_distance is None or distance_metric.value < minimum_distance.value:
            minimum_distance = distance_metric
    return SeedSetSummary(
        proposed_count=len(audits),
        accepted_count=sum(audit.accepted for audit in audits),
        rejected_seeds=tuple(int(audit.seed) for audit in audits if not audit.accepted),
        minimum_barrier=minimum_barrier,
        minimum_qp_margin=minimum_qp,
        maximum_bar_nu=maximum_bar_nu,
        minimum_pair_distance=minimum_distance,
    )


def _audit_seed_set(
    family: Mapping,
    seeds: Iterable[int],
) -> SeedSetAudit:
    checked = validate_qualified_initial_family(family)
    requested = _strict_seed_tuple(seeds)
    audits = tuple(
        _audit_positions(
            checked,
            _materialize_seed_positions(checked, seed),
            seed=seed,
        )
        for seed in requested
    )
    return SeedSetAudit(audits, _summarize_seed_audits(audits))


def _assert_summary_matches_frozen(
    summary: SeedSetSummary,
    frozen: Mapping,
    label: str,
) -> None:
    exact_pairs = {
        "proposed_count": summary.proposed_count,
        "accepted_count": summary.accepted_count,
        "minimum_barrier_seed": summary.minimum_barrier.seed,
        "minimum_qp_seed": summary.minimum_qp_margin.seed,
        "minimum_qp_uav": summary.minimum_qp_margin.robot_id,
        "maximum_bar_nu_seed": summary.maximum_bar_nu.seed,
        "maximum_bar_nu_uav": summary.maximum_bar_nu.robot_id,
        "minimum_pair_distance_seed": summary.minimum_pair_distance.seed,
    }
    for key, actual in exact_pairs.items():
        if actual != frozen[key]:
            raise ValueError(f"{label} frozen summary differs at {key}")
    sequence_pairs = {
        "minimum_barrier_edge": list(summary.minimum_barrier.edge.token()),
        "minimum_pair_distance_edge": list(summary.minimum_pair_distance.edge),
    }
    for key, actual in sequence_pairs.items():
        if actual != frozen[key]:
            raise ValueError(f"{label} frozen summary differs at {key}")
    numeric_pairs = {
        "minimum_barrier_m": summary.minimum_barrier.value,
        "minimum_qp_margin_mps": summary.minimum_qp_margin.value,
        "maximum_bar_nu_mps": summary.maximum_bar_nu.value,
        "minimum_pair_distance_m": summary.minimum_pair_distance.value,
    }
    for key, actual in numeric_pairs.items():
        if not math.isclose(
            actual, frozen[key], rel_tol=1e-12, abs_tol=1e-12
        ):
            raise ValueError(f"{label} frozen summary differs at {key}")
    if summary.rejected_seeds:
        raise ValueError(f"{label} frozen seed universe contains a rejection")


def audit_frozen_initial_family(
    family: Mapping,
) -> FrozenInitialFamilyAudit:
    """Recompute and bind both immutable v5 seed universes in full."""
    checked = validate_qualified_initial_family(family)
    schedule = checked["schedule"]
    audit_seeds = tuple(
        range(schedule["audit_seed_first"], schedule["audit_seed_last"] + 1)
    )
    if len(audit_seeds) != schedule["audit_seed_count"]:
        raise ValueError("frozen audit seed universe is inconsistent")
    if audit_seeds != EXPECTED_AUDIT_SEEDS:
        raise ValueError("frozen audit seed universe was replaced")
    audit = _audit_seed_set(checked, audit_seeds)
    registered_seeds = tuple(schedule["registered_trajectory_seeds"])
    if registered_seeds != EXPECTED_REGISTERED_SEEDS:
        raise ValueError("registered seed universe was replaced")
    registered = audit.subset(registered_seeds)
    frozen = checked["frozen_summary"]
    _assert_summary_matches_frozen(
        registered.summary, frozen["registered"], "registered"
    )
    _assert_summary_matches_frozen(audit.summary, frozen["audit"], "audit")
    representative = require_admitted_seed(
        checked, frozen["representative_seed"]
    )
    if (
        representative.positions_sha256
        != frozen["representative_positions_sha256"]
    ):
        raise ValueError("representative position SHA-256 differs")
    return FrozenInitialFamilyAudit(registered=registered, audit=audit)
