from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from itertools import combinations
import math
import struct
from typing import Callable, Sequence

import numpy as np


class LazyKeySequence(Sequence[tuple[object, ...]]):
    def __init__(
        self,
        length: int,
        key_at: Callable[[int], tuple[object, ...]],
    ) -> None:
        self._length = length
        self._key_at = key_at

    def __len__(self) -> int:
        return self._length

    def __getitem__(self, index: int | slice):
        if isinstance(index, slice):
            return tuple(self[position] for position in range(*index.indices(self._length)))
        if index < 0:
            index += self._length
        if index < 0 or index >= self._length:
            raise IndexError(index)
        return self._key_at(index)


@dataclass(frozen=True)
class FrozenMissionSchedule:
    campaign_id: str
    trajectory_seed: int
    range_noise_seed: int
    frames: int
    robots: tuple[int, ...]
    estimator_conditions: tuple[str, ...]
    controller_condition: str


@dataclass(frozen=True, order=True)
class CanonicalEdgeId:
    kind: str
    low: int
    high: int
    base_id: int


def _paper_localization_edges() -> tuple[CanonicalEdgeId, ...]:
    edges: set[CanonicalEdgeId] = {
        CanonicalEdgeId("localization", 1, 1, 0),
        CanonicalEdgeId("localization", 1, 1, 1),
        CanonicalEdgeId("localization", 2, 2, 1),
        CanonicalEdgeId("localization", 8, 8, 1),
        CanonicalEdgeId("localization", 8, 8, 2),
        CanonicalEdgeId("localization", 9, 9, 1),
    }
    for squad in range(2):
        first_uav = 1 + 7 * squad
        for local_index in range(2, 8):
            owner = first_uav + local_index - 1
            for reference_index in range(max(1, local_index - 2), local_index):
                reference = first_uav + reference_index - 1
                edges.add(
                    CanonicalEdgeId(
                        "localization",
                        min(owner, reference),
                        max(owner, reference),
                        -1,
                    )
                )
    return tuple(sorted(edges, key=lambda edge: (edge.low, edge.high, edge.base_id)))


_LOCALIZATION_EDGES = _paper_localization_edges()
_COLLISION_EDGES = tuple(
    CanonicalEdgeId("collision", low, high, -1)
    for low in range(1, 15)
    for high in range(low + 1, 15)
)
_PAPER_EDGES = _LOCALIZATION_EDGES + _COLLISION_EDGES
_PAPER_ENDPOINTS = tuple(
    (edge, owner)
    for edge in _PAPER_EDGES
    for owner in ((edge.low,) if edge.base_id >= 0 else (edge.low, edge.high))
)
_PAPER_BASE_POSITIONS = {
    0: (-1550.0, -300.0),
    1: (-1550.0, 0.0),
    2: (-1550.0, 300.0),
}


@dataclass(frozen=True)
class MissionResult:
    key: tuple[str, int, int]
    success: bool
    reason: str


@dataclass(frozen=True)
class MissingMissionRows:
    initialization: Sequence[tuple[object, ...]]
    estimator: Sequence[tuple[object, ...]]
    controller: Sequence[tuple[object, ...]]
    endpoint: Sequence[tuple[object, ...]]
    reconstructed: Sequence[tuple[object, ...]]
    reset: Sequence[tuple[object, ...]]
    mission: MissionResult


@dataclass(frozen=True)
class ControllerReconstruction:
    nodes: dict[int, dict[str, object]]
    local_residuals: dict[tuple[CanonicalEdgeId, int], float]
    full_residuals: dict[CanonicalEdgeId, float]
    integrity_errors: tuple[str, ...]


@dataclass(frozen=True)
class EvidenceRatio:
    numerator: int
    denominator: int

    @property
    def value(self) -> float | None:
        return None if self.denominator == 0 else self.numerator / self.denominator


@dataclass(frozen=True)
class EvidenceAudit:
    initialization_availability: EvidenceRatio
    conditional_containment: dict[tuple[str, int], EvidenceRatio]
    joint_containment: dict[tuple[str, int], EvidenceRatio]
    controller_availability: EvidenceRatio
    mission_success: EvidenceRatio
    integrity_errors: tuple[str, ...]


_EVIDENCE_IDENTITY_FIELDS = {
    "record_type",
    "schema_version",
    "campaign_id",
    "condition",
    "trajectory_seed",
    "range_noise_seed",
    "frame_index",
}


def _valid_evidence_identity(record: object, record_type: str) -> bool:
    return (
        isinstance(record, dict)
        and _EVIDENCE_IDENTITY_FIELDS <= set(record)
        and record.get("record_type") == record_type
        and record.get("schema_version") == "cbf2026-qualified-evidence-v1"
        and _nonempty_string(record.get("campaign_id"))
        and _nonempty_string(record.get("condition"))
        and _uint64(record.get("trajectory_seed"))
        and _uint64(record.get("range_noise_seed"))
        and _uint64(record.get("frame_index"))
    )


def validate_initialization_schema(record: object) -> bool:
    try:
        if not _valid_evidence_identity(record, "initialization"):
            return False
        required = _EVIDENCE_IDENTITY_FIELDS | {
            "robot_id", "runtime", "analyzer_only"
        }
        if set(record) != required or record["frame_index"] != 0:
            return False
        runtime = record["runtime"]
        analyzer = record["analyzer_only"]
        return (
            _integer(record["robot_id"])
            and record["robot_id"] > 0
            and isinstance(runtime, dict)
            and set(runtime) == {"local_index"}
            and _integer(runtime["local_index"])
            and runtime["local_index"] > 0
            and isinstance(analyzer, dict)
            and set(analyzer) == {"truth_position"}
            and _vector(analyzer["truth_position"], 2)
        )
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def validate_estimator_tuple_schema(record: object) -> bool:
    try:
        if not _valid_evidence_identity(record, "estimator_tuple"):
            return False
        required = _EVIDENCE_IDENTITY_FIELDS | {
            "robot_id", "depth", "output_status", "estimate", "radius"
        }
        if set(record) != required:
            return False
        status = record["output_status"]
        available = status in {"fresh", "predicted"}
        return (
            _integer(record["robot_id"])
            and record["robot_id"] > 0
            and _integer(record["depth"])
            and record["depth"] > 0
            and status in {"fresh", "predicted", "unavailable"}
            and (
                (
                    available
                    and _vector(record["estimate"], 2)
                    and _finite_number(record["radius"])
                    and record["radius"] >= 0.0
                )
                or (
                    not available
                    and record["estimate"] is None
                    and record["radius"] is None
                )
            )
        )
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def validate_mission_terminal_schema(record: object) -> bool:
    try:
        if not _valid_evidence_identity(record, "mission_terminal"):
            return False
        if set(record) != _EVIDENCE_IDENTITY_FIELDS | {"runtime"}:
            return False
        runtime = record["runtime"]
        return (
            isinstance(runtime, dict)
            and set(runtime) == {
                "success",
                "reason",
                "process_outcome",
                "declared_frames",
                "completed_intervals",
            }
            and type(runtime["success"]) is bool
            and _nonempty_string(runtime["reason"])
            and runtime["process_outcome"]
                in {
                    "completed",
                    "bootstrap_failure",
                    "loop_failure",
                    "terminated_early",
                }
            and _integer(runtime["declared_frames"])
            and runtime["declared_frames"] > 0
            and _integer(runtime["completed_intervals"])
            and 0 <= runtime["completed_intervals"] <= runtime["declared_frames"]
            and record["frame_index"] == runtime["declared_frames"]
        )
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def _valid_controller_truth_boundary(record: object) -> bool:
    try:
        if not _valid_evidence_identity(record, "controller_interval"):
            return False
        if set(record) != _EVIDENCE_IDENTITY_FIELDS | {
            "runtime", "analyzer_only"
        }:
            return False
        analyzer = record["analyzer_only"]
        if not isinstance(analyzer, dict) or set(analyzer) != {"truth"}:
            return False
        truth = analyzer["truth"]
        if not isinstance(truth, list) or not truth:
            return False
        robot_ids = []
        for item in truth:
            if (
                not isinstance(item, dict)
                or set(item) != {"robot_id", "position"}
                or not _integer(item["robot_id"])
                or item["robot_id"] <= 0
                or not _vector(item["position"], 2)
            ):
                return False
            robot_ids.append(item["robot_id"])
        return sorted(robot_ids) == list(range(1, 15))
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def audit_reset_primitives(
    record: dict[str, object],
    expected_robot_ids: tuple[int, ...],
    expected_edge_count: int,
) -> tuple[str, ...]:
    errors: list[str] = []
    if not _valid_evidence_identity(record, "reset") or set(record) != (
        _EVIDENCE_IDENTITY_FIELDS | {"runtime"}
    ):
        return ("reset_schema",)
    runtime = record.get("runtime")
    runtime_fields = {
        "stage",
        "trigger",
        "predecessor_version",
        "proposed_version",
        "changed_nodes",
        "descendant_closure",
        "nodes",
        "active_sets",
        "pre_endpoint_states",
        "post_endpoint_states",
        "hard_edges",
        "hard_problems",
        "local_hard_qps",
        "guard_decision",
        "outcome",
        "reason",
    }
    if not isinstance(runtime, dict) or set(runtime) != runtime_fields:
        return ("reset_schema",)
    try:
        stages = {
            "proposal_started",
            "candidate_built",
            "hard_edges_built",
            "hard_qp_preflight",
            "hard_qp_verified",
            "lifecycle_rejected",
            "committed",
        }
        stage = runtime["stage"]
        if stage not in stages:
            errors.append("reset_stage")
        stage_rank = {
            "proposal_started": 0,
            "candidate_built": 1,
            "hard_edges_built": 2,
            "hard_qp_preflight": 3,
            "hard_qp_verified": 4,
            "lifecycle_rejected": 5,
            "committed": 6,
        }.get(stage, -1)
        if (
            not isinstance(runtime["trigger"], list)
            or not runtime["trigger"]
            or any(
                cause not in {
                    "active-reference-change",
                    "estimate-interface-reset",
                    "certificate-discontinuity",
                    "hard-edge-snapshot-change",
                }
                for cause in runtime["trigger"]
            )
            or not _nonempty_string(runtime["reason"])
        ):
            errors.append("reset_schema")
        predecessor_version = runtime["predecessor_version"]
        proposed_version = runtime["proposed_version"]
        if (
            not _uint64(predecessor_version)
            or not _uint64(proposed_version)
            or proposed_version != predecessor_version + 1
        ):
            errors.append("reset_version_successor")

        expected_ids = set(expected_robot_ids)
        accepted_claim = runtime.get("guard_decision") == "accepted"
        if runtime.get("guard_decision") not in {"accepted", "rejected"}:
            errors.append("reset_guard_decision")
        if runtime.get("outcome") not in {"commit", "abort"}:
            errors.append("reset_outcome")
        if (stage == "committed") != accepted_claim:
            errors.append("reset_stage_outcome")

        active_sets = runtime["active_sets"]
        if not isinstance(active_sets, list):
            return ("reset_active_sets",)
        active_by_node = {
            item["node_id"]: item for item in active_sets if isinstance(item, dict)
        }
        active_sets_valid = (
            len(active_by_node) == len(active_sets)
            and all(
                set(item) == {
                    "node_id",
                    "pre_active_references",
                    "post_active_references",
                }
                and _integer(item["node_id"])
                and isinstance(item["pre_active_references"], list)
                and isinstance(item["post_active_references"], list)
                for item in active_sets
            )
        )
        if not active_sets_valid or (accepted_claim and set(active_by_node) != expected_ids):
            errors.append("reset_active_sets")

        children: dict[int, set[int]] = {robot_id: set() for robot_id in expected_ids}
        for robot_id, item in active_by_node.items():
            post_references = item.get("post_active_references")
            if not isinstance(post_references, list):
                errors.append(f"reset_post_active_set:{robot_id}")
                continue
            for reference_id in post_references:
                if not _integer(reference_id) or reference_id == 0:
                    errors.append(f"reset_reference_identity:{robot_id}")
                elif reference_id > 0:
                    if reference_id not in expected_ids:
                        errors.append(f"reset_unknown_reference:{robot_id}")
                    else:
                        children[reference_id].add(robot_id)

        changed_nodes = runtime["changed_nodes"]
        if (
            not isinstance(changed_nodes, list)
            or not changed_nodes
            or any(node not in expected_ids for node in changed_nodes)
        ):
            errors.append("reset_changed_nodes")
            changed_nodes = []
        closure: set[int] = set(changed_nodes)
        frontier = list(changed_nodes)
        while frontier:
            parent = frontier.pop()
            for child in children.get(parent, ()):
                if child not in closure:
                    closure.add(child)
                    frontier.append(child)
        if stage_rank >= 1:
            if (
                not isinstance(runtime["descendant_closure"], list)
                or sorted(runtime["descendant_closure"]) != sorted(closure)
            ):
                errors.append("reset_descendant_closure")
        elif runtime["descendant_closure"] != []:
            errors.append("reset_descendant_closure")

        def endpoint_map(field: str) -> dict[int, dict[str, object]]:
            states = runtime[field]
            if not isinstance(states, list):
                return {}
            result: dict[int, dict[str, object]] = {}
            required = {
                "robot_id",
                "estimate",
                "covariance",
                "covariance_rate_bound",
                "epsilon",
                "bar_nu",
                "snapshot_version",
                "allocation_version",
            }
            for state in states:
                if (
                    not isinstance(state, dict)
                    or set(state) != required
                    or not _integer(state["robot_id"])
                    or not _vector(state["estimate"], 2)
                    or not _matrix2(state["covariance"])
                    or not all(
                        _finite_number(state[name]) and state[name] >= 0.0
                        for name in (
                            "covariance_rate_bound",
                            "epsilon",
                            "bar_nu",
                        )
                    )
                    or not _integer(state["snapshot_version"])
                    or not _integer(state["allocation_version"])
                    or state["allocation_version"] <= 0
                ):
                    errors.append(f"reset_endpoint_schema:{field}")
                    continue
                covariance = np.asarray(state["covariance"], dtype=float)
                if (
                    not np.allclose(covariance, covariance.T, rtol=0.0, atol=1e-12)
                    or float(np.linalg.eigvalsh(covariance)[0]) < -1e-12
                ):
                    errors.append(f"reset_endpoint_covariance:{state['robot_id']}")
                if state["robot_id"] in result:
                    errors.append(f"reset_duplicate_endpoint:{field}:{state['robot_id']}")
                result[state["robot_id"]] = state
            return result

        pre_endpoints = endpoint_map("pre_endpoint_states")
        post_endpoints = endpoint_map("post_endpoint_states")
        if accepted_claim and (
            set(pre_endpoints) != expected_ids or set(post_endpoints) != expected_ids
        ):
            errors.append("reset_endpoint_universe")
        for state in pre_endpoints.values():
            if state.get("snapshot_version") != predecessor_version:
                errors.append(f"reset_pre_snapshot_version:{state.get('robot_id')}")
        for state in post_endpoints.values():
            if state.get("snapshot_version") != proposed_version:
                errors.append(f"reset_post_snapshot_version:{state.get('robot_id')}")

        node_records = runtime["nodes"]
        if not isinstance(node_records, list):
            return ("reset_schema",)
        node_by_id = {
            item["node_id"]: item
            for item in node_records
            if isinstance(item, dict) and _integer(item.get("node_id"))
        }
        expected_node_records = closure if stage_rank >= 1 else set()
        if (
            len(node_by_id) != len(node_records)
            or set(node_by_id) != expected_node_records
        ):
            errors.append("reset_node_closure")
        topological_pairs = []
        for item in node_records:
            if (
                isinstance(item, dict)
                and _integer(item.get("node_id"))
                and _integer(item.get("topological_index"))
            ):
                topological_pairs.append(
                    (item["topological_index"], item["node_id"])
                )
        expected_topological_indices = list(
            range(1, len(node_records) + 1)
        )
        if (
            sorted(index for index, _node_id in topological_pairs)
                != expected_topological_indices
            or [
                node_id for _index, node_id in sorted(topological_pairs)
            ] != list(runtime["descendant_closure"])
        ):
            errors.append("reset_topological_index:closure_order")
        topological_index_by_node = {
            node_id: index for index, node_id in topological_pairs
        }
        for node_id, active in active_by_node.items():
            node_index = topological_index_by_node.get(node_id)
            if node_index is None:
                continue
            for reference_id in active.get("post_active_references", []):
                reference_index = topological_index_by_node.get(reference_id)
                if (
                    reference_index is not None
                    and reference_index >= node_index
                ):
                    errors.append(
                        f"reset_topological_reference:{node_id}:{reference_id}"
                    )
        for node_id, item in node_by_id.items():
            if set(item) != {
                "node_id",
                "topological_index",
                "snapshot_version",
                "delta_p",
                "delta_epsilon",
                "pre_active_references",
                "post_active_references",
                "proposed_snapshot",
            }:
                errors.append(f"reset_node_schema:{node_id}")
                continue
            if node_id not in pre_endpoints or node_id not in post_endpoints:
                errors.append(f"reset_node_endpoint:{node_id}")
                continue
            pre = pre_endpoints[node_id]
            post = post_endpoints[node_id]
            delta_p = (
                np.asarray(post["estimate"], dtype=float)
                - np.asarray(pre["estimate"], dtype=float)
            )
            delta_epsilon = float(post["epsilon"] - pre["epsilon"])
            if not np.allclose(
                np.asarray(item["delta_p"], dtype=float),
                delta_p,
                rtol=0.0,
                atol=1e-12,
            ):
                errors.append(f"reset_delta_p:{node_id}")
            if not math.isclose(
                float(item["delta_epsilon"]),
                delta_epsilon,
                rel_tol=0.0,
                abs_tol=1e-12,
            ):
                errors.append(f"reset_delta_epsilon:{node_id}")
            if item.get("snapshot_version") != proposed_version:
                errors.append(f"reset_node_snapshot_version:{node_id}")
            proposed_snapshot = item.get("proposed_snapshot")
            required_snapshot_fields = {
                "robot_id",
                "estimate",
                "covariance",
                "covariance_rate_bound",
                "epsilon",
                "bar_nu",
                "snapshot_version",
                "allocation_version",
            }
            proposed_snapshot_valid = (
                isinstance(proposed_snapshot, dict)
                and set(proposed_snapshot) == required_snapshot_fields
                and proposed_snapshot.get("robot_id") == node_id
                and _vector(proposed_snapshot.get("estimate"), 2)
                and _matrix2(proposed_snapshot.get("covariance"))
                and all(
                    _finite_number(proposed_snapshot.get(field))
                    and proposed_snapshot[field] >= 0.0
                    for field in (
                        "covariance_rate_bound", "epsilon", "bar_nu"
                    )
                )
                and proposed_snapshot.get("snapshot_version")
                    == proposed_version
                and proposed_snapshot.get("allocation_version")
                    == post["allocation_version"]
            )
            if proposed_snapshot_valid:
                proposed_covariance = np.asarray(
                    proposed_snapshot["covariance"], dtype=float
                )
                proposed_snapshot_valid = (
                    np.allclose(
                        proposed_covariance,
                        proposed_covariance.T,
                        rtol=0.0,
                        atol=1e-12,
                    )
                    and float(np.linalg.eigvalsh(proposed_covariance)[0])
                        >= -1e-12
                    and proposed_snapshot == post
                )
            if not proposed_snapshot_valid:
                errors.append(f"reset_proposed_snapshot:{node_id}")
            active = active_by_node.get(node_id, {})
            if item.get("pre_active_references") != active.get(
                "pre_active_references"
            ):
                errors.append(f"reset_pre_active_projection:{node_id}")
            if item.get("post_active_references") != active.get(
                "post_active_references"
            ):
                errors.append(f"reset_post_active_projection:{node_id}")

        hard_edges = runtime["hard_edges"]
        if not isinstance(hard_edges, list):
            return ("reset_schema",)
        if accepted_claim and len(hard_edges) != expected_edge_count:
            errors.append("reset_edge_cardinality")
        seen_edges: set[CanonicalEdgeId] = set()
        all_post_barriers_nonnegative = True
        expected_problem_rows: dict[
            tuple[int, CanonicalEdgeId], dict[str, object]
        ] = {}
        for item in hard_edges:
            if not isinstance(item, dict) or set(item) != {
                "edge",
                "threshold",
                "base_position",
                "b_minus",
                "b_plus",
                "class_k_coefficient",
                "class_k_power",
                "endpoint_rows",
            }:
                errors.append("reset_edge_schema")
                continue
            edge_raw = item["edge"]
            edge = CanonicalEdgeId(
                edge_raw["kind"],
                edge_raw["low"],
                edge_raw["high"],
                edge_raw["base_id"],
            )
            if edge in seen_edges:
                errors.append(f"reset_duplicate_edge:{edge}")
            seen_edges.add(edge)
            if expected_edge_count == len(_PAPER_EDGES) and edge not in set(_PAPER_EDGES):
                errors.append("reset_edge_universe")

            def barrier(
                endpoints: dict[int, dict[str, object]],
            ) -> float:
                first = endpoints[edge.low]
                first_position = np.asarray(first["estimate"], dtype=float)
                if edge.base_id >= 0:
                    second_position = np.asarray(item["base_position"], dtype=float)
                    second_epsilon = 0.0
                else:
                    second = endpoints[edge.high]
                    second_position = np.asarray(second["estimate"], dtype=float)
                    second_epsilon = float(second["epsilon"])
                separation = float(np.linalg.norm(first_position - second_position))
                uncertainty = float(first["epsilon"]) + second_epsilon
                if edge.kind == "localization":
                    return float(item["threshold"]) - separation - uncertainty
                return separation - float(item["threshold"]) - uncertainty

            if edge.low not in pre_endpoints or edge.low not in post_endpoints or (
                edge.base_id < 0
                and (edge.high not in pre_endpoints or edge.high not in post_endpoints)
            ):
                errors.append(f"reset_edge_endpoint:{edge}")
                continue
            b_minus = barrier(pre_endpoints)
            b_plus = barrier(post_endpoints)
            if not math.isclose(
                float(item["b_minus"]), b_minus, rel_tol=1e-9, abs_tol=1e-9
            ):
                errors.append(f"reset_b_minus:{edge}")
            if not math.isclose(
                float(item["b_plus"]), b_plus, rel_tol=1e-9, abs_tol=1e-9
            ):
                errors.append(f"reset_b_plus:{edge}")
            if b_plus < 0.0:
                all_post_barriers_nonnegative = False
                errors.append(f"reset_negative_b_plus:{edge}")
            endpoint_rows = item["endpoint_rows"]
            if not isinstance(endpoint_rows, list):
                errors.append(f"reset_endpoint_rows:{edge}")
                continue
            expected_owners = {edge.low} if edge.base_id >= 0 else {edge.low, edge.high}
            if (
                len(endpoint_rows) != len(expected_owners)
                or {row.get("owner") for row in endpoint_rows} != expected_owners
            ):
                errors.append(f"reset_endpoint_rows:{edge}")
            for endpoint_row in endpoint_rows:
                if not isinstance(endpoint_row, dict) or set(endpoint_row) != {
                    "owner",
                    "coefficient",
                    "constant",
                    "allocation",
                    "snapshot_version",
                    "allocation_version",
                }:
                    errors.append(f"reset_endpoint_row_schema:{edge}")
                    continue
                if endpoint_row.get("snapshot_version") != proposed_version:
                    errors.append(f"reset_edge_snapshot_version:{edge}")
                owner = endpoint_row["owner"]
                first = post_endpoints[edge.low]
                first_position = np.asarray(first["estimate"], dtype=float)
                if edge.base_id >= 0:
                    second_position = np.asarray(item["base_position"], dtype=float)
                    epsilon_high = 0.0
                    bar_high = 0.0
                else:
                    second = post_endpoints[edge.high]
                    second_position = np.asarray(second["estimate"], dtype=float)
                    epsilon_high = float(second["epsilon"])
                    bar_high = float(second["bar_nu"])
                displacement = first_position - second_position
                separation = float(np.linalg.norm(displacement))
                if separation <= 0.0:
                    errors.append(f"reset_edge_separation:{edge}")
                    continue
                normal = displacement / separation
                if edge.kind == "localization":
                    coefficient = -normal if owner == edge.low else normal
                else:
                    coefficient = normal if owner == edge.low else -normal
                allocation = 1.0 if edge.base_id >= 0 else 0.5
                alpha = float(item["class_k_coefficient"]) * b_plus ** int(
                    item["class_k_power"]
                )
                owner_bar_nu = float(first["bar_nu"]) if owner == edge.low else bar_high
                constant = -owner_bar_nu + allocation * alpha
                comparisons = (
                    math.isclose(endpoint_row["allocation"], allocation, rel_tol=0.0, abs_tol=1e-12),
                    np.allclose(endpoint_row["coefficient"], coefficient, rtol=0.0, atol=1e-9),
                    math.isclose(endpoint_row["constant"], constant, rel_tol=1e-8, abs_tol=1e-8),
                    endpoint_row["allocation_version"] == 1,
                )
                if not all(comparisons):
                    errors.append(f"reset_endpoint_row:{edge}:{owner}")
                other_id = edge.high if owner == edge.low else edge.low
                name = (
                    f"fixedCommCBF(base-{edge.base_id})"
                    if edge.kind == "localization" and edge.base_id >= 0
                    else f"fixedCommCBF(#{other_id})"
                    if edge.kind == "localization"
                    else f"safetyCBF(#{other_id})"
                )
                expected_problem_rows[(owner, edge)] = {
                    "coefficients": [float(coefficient[0]), float(coefficient[1]), 0.0],
                    "constant": constant,
                    "post_reset_barrier": b_plus,
                    "snapshot_version": proposed_version,
                    "allocation_version": 1,
                    "name": name,
                }

        if (
            accepted_claim
            and expected_edge_count == len(_PAPER_EDGES)
            and seen_edges != set(_PAPER_EDGES)
        ):
            errors.append("reset_edge_universe")

        local_qps = runtime["local_hard_qps"]
        if not isinstance(local_qps, list):
            return ("reset_schema",)
        qps_by_node = {
            item["node_id"]: item
            for item in local_qps
            if isinstance(item, dict) and _integer(item.get("node_id"))
        }
        all_qps_valid = set(qps_by_node) == expected_ids and len(qps_by_node) == len(local_qps)
        if accepted_claim and not all_qps_valid:
            errors.append("reset_qp_universe")
        checked_problem_by_node: dict[int, dict[str, object]] = {}
        hard_problems = runtime["hard_problems"]
        if not isinstance(hard_problems, list):
            return ("reset_schema",)
        for problem in hard_problems:
            owner = problem.get("owner") if isinstance(problem, dict) else None
            if not _integer(owner) or not _valid_hard_problem(problem, owner):
                errors.append("reset_checked_problem_schema")
                continue
            if not _exact_hard_problem_structure(
                problem, owner, proposed_version, 1
            ):
                errors.append(f"reset_hard_problem_structure:{owner}:checked")
            if owner in checked_problem_by_node:
                errors.append(f"reset_duplicate_checked_problem:{owner}")
            checked_problem_by_node[owner] = problem
        if accepted_claim and set(checked_problem_by_node) != expected_ids:
            errors.append("reset_checked_problem_universe")

        for robot_id, qp in qps_by_node.items():
            if not isinstance(qp, dict) or set(qp) != {
                "node_id",
                "problem",
                "feasible",
                "status",
                "minimum_residual",
                "hard_problem_id",
                "solution",
            }:
                errors.append(f"reset_qp_schema:{robot_id}")
                all_qps_valid = False
                continue
            problem = qp["problem"]
            if not _valid_hard_problem(problem, robot_id):
                errors.append(f"reset_hard_problem_schema:{robot_id}")
                all_qps_valid = False
                continue
            if not _exact_hard_problem_structure(
                problem, robot_id, proposed_version, 1
            ):
                errors.append(f"reset_hard_problem_structure:{robot_id}:qp")
                all_qps_valid = False
            canonical_hash = _canonical_hard_problem_hash(problem)
            if problem["hard_problem_id"] != canonical_hash:
                errors.append(f"reset_hard_problem_hash:{robot_id}")
                all_qps_valid = False
            if qp["hard_problem_id"] != canonical_hash:
                errors.append(f"reset_qp_hash:{robot_id}")
                all_qps_valid = False
            expected_rows = {
                edge: fields
                for (owner, edge), fields in expected_problem_rows.items()
                if owner == robot_id
            }
            observed_rows: dict[CanonicalEdgeId, dict[str, object]] = {}
            for problem_row in problem["rows"]:
                edge_raw = problem_row["edge"]
                edge = CanonicalEdgeId(
                    edge_raw["kind"], edge_raw["low"], edge_raw["high"], edge_raw["base_id"]
                )
                observed_rows[edge] = problem_row
            if set(observed_rows) != set(expected_rows):
                errors.append(f"reset_problem_row_universe:{robot_id}")
                all_qps_valid = False
            else:
                for edge, expected in expected_rows.items():
                    actual = observed_rows[edge]
                    if (
                        actual["name"] != expected["name"]
                        or not np.allclose(actual["coefficients"], expected["coefficients"], rtol=0.0, atol=1e-9)
                        or not math.isclose(actual["constant"], expected["constant"], rel_tol=1e-8, abs_tol=1e-8)
                        or not math.isclose(actual["post_reset_barrier"], expected["post_reset_barrier"], rel_tol=1e-8, abs_tol=1e-8)
                        or actual["snapshot_version"] != proposed_version
                        or actual["allocation_version"] != 1
                    ):
                        errors.append(f"reset_problem_row:{robot_id}:{edge}")
                        all_qps_valid = False
            if robot_id in checked_problem_by_node and checked_problem_by_node[robot_id] != problem:
                errors.append(f"reset_checked_problem_projection:{robot_id}")
                all_qps_valid = False

            unchecked = qp["status"] == "unchecked"
            if unchecked:
                if (
                    stage != "hard_qp_preflight"
                    or qp["solution"] is not None
                    or qp["minimum_residual"] is not None
                    or qp["feasible"] is not False
                ):
                    errors.append(f"reset_qp_stage:{robot_id}")
                    all_qps_valid = False
                continue
            solution_valid = _vector(qp["solution"], 3)
            residuals = _problem_residuals(problem, qp["solution"]) if solution_valid else []
            witness_feasible = bool(residuals) and min(residuals) >= -1e-7
            independently_feasible = witness_feasible or _hard_problem_is_feasible(problem)
            if solution_valid and not witness_feasible:
                errors.append(f"reset_qp_solution:{robot_id}")
                all_qps_valid = False
            if solution_valid:
                expected_minimum = min(residuals)
                if (
                    not _finite_number(qp["minimum_residual"])
                    or not math.isclose(qp["minimum_residual"], expected_minimum, rel_tol=1e-9, abs_tol=1e-9)
                ):
                    errors.append(f"reset_qp_minimum:{robot_id}")
                    all_qps_valid = False
            elif qp["minimum_residual"] is not None:
                errors.append(f"reset_qp_minimum:{robot_id}")
                all_qps_valid = False
            derived_feasible = independently_feasible and witness_feasible
            if type(qp["feasible"]) is not bool or qp["feasible"] != derived_feasible:
                errors.append(f"reset_qp_comparison:{robot_id}")
                all_qps_valid = False
            if (qp["status"] == "optimal") != derived_feasible:
                errors.append(f"reset_qp_status:{robot_id}")
                all_qps_valid = False

        derived_accept = (
            proposed_version == predecessor_version + 1
            and sorted(runtime["descendant_closure"]) == sorted(closure)
            and len(seen_edges) == expected_edge_count
            and all_post_barriers_nonnegative
            and all_qps_valid
            and set(pre_endpoints) == expected_ids
            and set(post_endpoints) == expected_ids
            and set(checked_problem_by_node) == expected_ids
        )
        if accepted_claim:
            if not derived_accept:
                errors.append("reset_guard_decision")
            if runtime.get("outcome") != "commit":
                errors.append("reset_outcome")
        else:
            if runtime.get("outcome") != "abort":
                errors.append("reset_outcome")
            if stage == "committed":
                errors.append("reset_stage_outcome")
    except Exception:
        errors.append("reset_schema")
    return tuple(errors)


def audit_evidence_denominators(
    records: Sequence[dict[str, object]],
    schedules: Sequence[FrozenMissionSchedule],
    registered_depths: dict[str, dict[int, int]],
) -> EvidenceAudit:
    errors: list[str] = []
    if (
        not isinstance(records, Sequence)
        or isinstance(records, (str, bytes, dict))
    ):
        errors.append("malformed_record:record_sequence")
        records = ()
    else:
        valid_records = []
        for record in records:
            if isinstance(record, dict):
                valid_records.append(record)
            else:
                errors.append("malformed_record:non_object")
        records = tuple(valid_records)
    initialization_expected: set[tuple[object, ...]] = set()
    controller_expected: set[tuple[object, ...]] = set()
    mission_expected: set[tuple[object, ...]] = set()
    estimator_expected: dict[
        tuple[str, int], set[tuple[object, ...]]
    ] = {}

    for schedule in schedules:
        mission_key = (
            schedule.campaign_id,
            schedule.trajectory_seed,
            schedule.range_noise_seed,
        )
        if mission_key in mission_expected:
            errors.append(f"duplicate_schedule:{mission_key}")
        mission_expected.add(mission_key)
        for robot_id in schedule.robots:
            initialization_expected.add((*mission_key, 0, robot_id))
        for interval in range(schedule.frames):
            controller_expected.add(
                (
                    schedule.campaign_id,
                    schedule.controller_condition,
                    schedule.trajectory_seed,
                    schedule.range_noise_seed,
                    interval,
                )
            )
        for condition in schedule.estimator_conditions:
            condition_depths = registered_depths.get(condition, {})
            for robot_id in schedule.robots:
                if robot_id not in condition_depths:
                    errors.append(f"missing_registered_depth:{condition}:{robot_id}")
                    continue
                depth = condition_depths[robot_id]
                universe = estimator_expected.setdefault((condition, depth), set())
                for frame in range(1, schedule.frames):
                    universe.add(
                        (
                            schedule.campaign_id,
                            condition,
                            schedule.trajectory_seed,
                            schedule.range_noise_seed,
                            frame,
                            robot_id,
                        )
                    )

    initialization_observed: set[tuple[object, ...]] = set()
    controller_observed: set[tuple[object, ...]] = set()
    controller_records: dict[tuple[object, ...], dict[str, object]] = {}
    endpoint_records: dict[tuple[object, ...], list[dict[str, object]]] = {}
    reset_records: dict[
        tuple[object, ...], list[dict[str, object]]
    ] = {}
    mission_observed: set[tuple[object, ...]] = set()
    mission_terminal_records: dict[
        tuple[object, ...], dict[str, object]
    ] = {}
    estimator_observed: dict[
        tuple[str, int], dict[tuple[object, ...], bool]
    ] = {}

    truth_by_key: dict[tuple[object, ...], list[float]] = {}
    for record in records:
        if not _valid_controller_truth_boundary(record):
            continue
        truth = record["analyzer_only"]["truth"]
        for item in truth:
            key = (
                record["campaign_id"],
                record["trajectory_seed"],
                record["range_noise_seed"],
                record["frame_index"],
                item["robot_id"],
            )
            if key in truth_by_key:
                errors.append(f"duplicate_analyzer_truth:{key}")
            truth_by_key[key] = item["position"]

    for record in records:
        record_type = record.get("record_type")
        try:
            if record_type == "initialization":
                if not validate_initialization_schema(record):
                    errors.append("malformed_record:initialization")
                    continue
                key = (
                    record["campaign_id"],
                    record["trajectory_seed"],
                    record["range_noise_seed"],
                    record["frame_index"],
                    record["robot_id"],
                )
                if key in initialization_observed:
                    errors.append(f"duplicate_initialization:{key}")
                initialization_observed.add(key)
            elif record_type == "estimator_tuple":
                if not validate_estimator_tuple_schema(record):
                    errors.append("malformed_record:estimator_tuple")
                    continue
                key = (
                    record["campaign_id"],
                    record["condition"],
                    record["trajectory_seed"],
                    record["range_noise_seed"],
                    record["frame_index"],
                    record["robot_id"],
                )
                depth_key = (record["condition"], record["depth"])
                observed = estimator_observed.setdefault(depth_key, {})
                if key in observed:
                    errors.append(f"duplicate_estimator:{key}")
                estimate = record.get("estimate")
                truth = truth_by_key.get(
                    (
                        record["campaign_id"],
                        record["trajectory_seed"],
                        record["range_noise_seed"],
                        record["frame_index"],
                        record["robot_id"],
                    )
                )
                radius = record.get("radius")
                available = (
                    _vector(estimate, 2)
                    and _vector(truth, 2)
                    and _finite_number(radius)
                    and radius >= 0.0
                )
                contained = False
                if available:
                    contained = math.dist(estimate, truth) <= radius
                observed[key] = contained if available else None
            elif record_type == "controller_interval":
                key = (
                    record["campaign_id"],
                    record["condition"],
                    record["trajectory_seed"],
                    record["range_noise_seed"],
                    record["frame_index"],
                )
                if key in controller_observed:
                    errors.append(f"duplicate_controller:{key}")
                controller_observed.add(key)
                controller_records[key] = record
            elif record_type == "endpoint_row":
                key = (
                    record["campaign_id"],
                    record["condition"],
                    record["trajectory_seed"],
                    record["range_noise_seed"],
                    record["frame_index"],
                )
                endpoint_records.setdefault(key, []).append(record)
            elif record_type == "reset":
                key = (
                    record["campaign_id"],
                    record["condition"],
                    record["trajectory_seed"],
                    record["range_noise_seed"],
                    record["frame_index"],
                )
                reset_records.setdefault(key, []).append(record)
            elif record_type == "mission_terminal":
                if not validate_mission_terminal_schema(record):
                    errors.append("malformed_record:mission_terminal")
                    continue
                key = (
                    record["campaign_id"],
                    record["trajectory_seed"],
                    record["range_noise_seed"],
                )
                if key in mission_observed:
                    errors.append(f"duplicate_mission_terminal:{key}")
                mission_observed.add(key)
                mission_terminal_records[key] = record
        except (KeyError, TypeError, ValueError, OverflowError):
            errors.append(f"malformed_record:{record_type}")

    unknown_initialization = initialization_observed - initialization_expected
    if unknown_initialization:
        errors.append("unexpected_initialization_key")
    unknown_controller = controller_observed - controller_expected
    if unknown_controller:
        errors.append("unexpected_controller_key")
    unknown_mission = mission_observed - mission_expected
    if unknown_mission:
        errors.append("unexpected_mission_key")

    conditional: dict[tuple[str, int], EvidenceRatio] = {}
    joint: dict[tuple[str, int], EvidenceRatio] = {}
    for depth_key, expected in estimator_expected.items():
        if not expected:
            errors.append(
                f"zero_depth_denominator:{depth_key[0]}:{depth_key[1]}"
            )
        observed = estimator_observed.get(depth_key, {})
        unexpected = set(observed) - expected
        if unexpected:
            errors.append(f"unexpected_estimator_key:{depth_key}")
        finite_values = [
            contained
            for key, contained in observed.items()
            if key in expected and contained is not None
        ]
        contained_count = sum(contained is True for contained in finite_values)
        conditional[depth_key] = EvidenceRatio(
            contained_count, len(finite_values)
        )
        joint[depth_key] = EvidenceRatio(contained_count, len(expected))

    initialization_hits = len(
        initialization_observed & initialization_expected
    )
    valid_controller: set[tuple[object, ...]] = set()
    for reset_key in set(reset_records) - controller_expected:
        errors.append(f"unexpected_reset:{reset_key}")
    for schedule in schedules:
        current_snapshot_version = 0
        version_chain_known = True
        for frame_index in range(schedule.frames):
            key = (
                schedule.campaign_id,
                schedule.controller_condition,
                schedule.trajectory_seed,
                schedule.range_noise_seed,
                frame_index,
            )
            controller = controller_records.get(key)
            if controller is None:
                version_chain_known = False
                continue
            if (
                not validate_controller_primitive_schema(controller)
                or controller["runtime"]["complete"] is not True
            ):
                errors.append(f"incomplete_controller:{key}")
                version_chain_known = False
                continue
            runtime = controller["runtime"]
            snapshot_version = runtime["snapshot_version"]
            allocation_version = runtime["allocation_version"]
            reset_marker = runtime["reset"]
            matching_resets = reset_records.get(key, [])
            transition_valid = version_chain_known
            if not version_chain_known:
                errors.append(f"reset_version_chain_unknown:{key}")
            if allocation_version != 1:
                errors.append(f"allocation_version:{key}")
                transition_valid = False
            if len(matching_resets) > 1:
                errors.append(f"reset_cardinality:{key}")
                transition_valid = False

            reset_runtime = None
            reset_accepted = False
            if len(matching_resets) == 1:
                reset_errors = audit_reset_primitives(
                    matching_resets[0], tuple(range(1, 15)), 119
                )
                if reset_errors:
                    errors.append(f"invalid_reset:{key}")
                    transition_valid = False
                else:
                    reset_runtime = matching_resets[0]["runtime"]
                    reset_accepted = (
                        reset_runtime["guard_decision"] == "accepted"
                    )

            version_changed = snapshot_version != current_snapshot_version
            if version_changed:
                if (
                    not version_chain_known
                    or snapshot_version != current_snapshot_version + 1
                    or reset_runtime is None
                    or not reset_accepted
                    or reset_runtime["predecessor_version"]
                        != current_snapshot_version
                    or reset_runtime["proposed_version"] != snapshot_version
                ):
                    errors.append(f"reset_version_transition:{key}")
                    transition_valid = False
            elif reset_runtime is not None:
                if (
                    reset_accepted
                    or reset_runtime["predecessor_version"]
                        != current_snapshot_version
                    or reset_runtime["proposed_version"]
                        != current_snapshot_version + 1
                ):
                    errors.append(f"reset_version_transition:{key}")
                    transition_valid = False

            expected_attempted = reset_runtime is not None
            expected_status = (
                "accepted" if reset_accepted else
                "rejected" if expected_attempted else
                "not-attempted"
            )
            if (
                reset_marker["attempted"] != expected_attempted
                or reset_marker["guard_status"] != expected_status
                or reset_marker["committed"] != reset_accepted
            ):
                errors.append(f"reset_projection:{key}")
                transition_valid = False

            if transition_valid:
                current_snapshot_version = snapshot_version
            else:
                version_chain_known = False
                continue
            reconstruction = reconstruct_controller_primitives(
                controller,
                endpoint_records.get(key, ()),
                expected_endpoint_count=232,
                expected_reconstructed_count=119,
            )
            if reconstruction.integrity_errors:
                errors.append(f"invalid_controller:{key}")
                continue
            valid_controller.add(key)
    controller_hits = len(valid_controller)
    successful_missions = 0
    for schedule in schedules:
        mission_key = (
            schedule.campaign_id,
            schedule.trajectory_seed,
            schedule.range_noise_seed,
        )
        required_controller = {
            (
                schedule.campaign_id,
                schedule.controller_condition,
                schedule.trajectory_seed,
                schedule.range_noise_seed,
                interval,
            )
            for interval in range(schedule.frames)
        }
        terminal = mission_terminal_records.get(mission_key)
        runtime = terminal.get("runtime", {}) if terminal is not None else {}
        terminal_success = (
            terminal is not None
            and terminal.get("condition") == schedule.controller_condition
            and terminal.get("frame_index") == schedule.frames
            and runtime.get("success") is True
            and runtime.get("reason") == "completed"
            and runtime.get("process_outcome") == "completed"
            and runtime.get("declared_frames") == schedule.frames
            and runtime.get("completed_intervals") == schedule.frames
        )
        if terminal_success and required_controller <= valid_controller:
            successful_missions += 1

    return EvidenceAudit(
        EvidenceRatio(initialization_hits, len(initialization_expected)),
        conditional,
        joint,
        EvidenceRatio(controller_hits, len(controller_expected)),
        EvidenceRatio(successful_missions, len(mission_expected)),
        tuple(errors),
    )


def _reconstruct_controller_primitives_impl(
    controller: dict[str, object],
    endpoint_rows: Sequence[dict[str, object]],
    expected_endpoint_count: int | None = None,
    expected_reconstructed_count: int | None = None,
    schema_expected_node_count: int = 14,
) -> ControllerReconstruction:
    errors: list[str] = []
    if not _validate_controller_primitive_schema(
        controller, schema_expected_node_count
    ):
        return ControllerReconstruction({}, {}, {}, ("controller_schema",))

    runtime = controller["runtime"]
    snapshot_version = runtime["snapshot_version"]
    allocation_version = runtime["allocation_version"]
    raw_nodes = sorted(
        runtime["nodes"], key=lambda node: (node["local_index"], node["robot_id"])
    )
    raw_node_by_id = {node["robot_id"]: node for node in raw_nodes}
    full_commands = {
        node["robot_id"]: np.asarray(node["applied_command"], dtype=float)
        for node in raw_nodes
    }
    commands = {
        robot_id: command[:2] for robot_id, command in full_commands.items()
    }
    for robot_id, command in full_commands.items():
        for component, limit in ((0, 25.0), (1, 25.0), (2, 0.35)):
            if abs(float(command[component])) > limit + 1e-7:
                errors.append(f"input_bound:{robot_id}:{component}")
    reconstructed: dict[int, dict[str, object]] = {}
    covariance_derivatives: dict[int, np.ndarray] = {}

    for node in raw_nodes:
        robot_id = node["robot_id"]
        component_bound = float(node["node_component_bound"])
        expected_local_index = (robot_id - 1) % 7 + 1
        if node["local_index"] != expected_local_index:
            errors.append(f"node_local_index:{robot_id}")
        if not math.isclose(component_bound, 25.0, rel_tol=0.0, abs_tol=0.0):
            errors.append(f"node_component_bound:{robot_id}")
        information = np.zeros((2, 2), dtype=float)
        information_derivative = np.zeros((2, 2), dtype=float)
        information_rate_bound = 0.0
        derivatives = []
        seen_references: set[int] = set()
        node_inputs_valid = True
        reference_order = [
            (
                0,
                -reference["canonical_reference_id"],
            )
            if reference["canonical_reference_id"] < 0
            else (
                1,
                reference["predecessor_local_index"],
                reference["canonical_reference_id"],
            )
            for reference in node["references"]
        ]
        if reference_order != sorted(reference_order):
            errors.append(f"reference_order:{robot_id}")

        for reference in node["references"]:
            reference_id = reference["canonical_reference_id"]
            if reference_id in seen_references:
                errors.append(f"duplicate_reference:{robot_id}:{reference_id}")
            seen_references.add(reference_id)
            published_direction = np.asarray(
                reference["direction"], dtype=float
            )
            published_distance = float(reference["distance"])
            if reference_id < 0:
                base_id = -reference_id - 1
                reference_position = _PAPER_BASE_POSITIONS.get(base_id)
            else:
                predecessor_node = raw_node_by_id.get(reference_id)
                reference_position = (
                    predecessor_node["interface_estimate"]
                    if predecessor_node is not None else None
                )
            if reference_position is None:
                errors.append(f"reference_geometry:{robot_id}:{reference_id}")
                node_inputs_valid = False
                continue
            displacement = (
                np.asarray(node["interface_estimate"], dtype=float)
                - np.asarray(reference_position, dtype=float)
            )
            distance = float(np.linalg.norm(displacement))
            if not math.isfinite(distance) or distance <= 0.0:
                errors.append(f"reference_geometry:{robot_id}:{reference_id}")
                node_inputs_valid = False
                continue
            q = displacement / distance
            if (
                not np.allclose(
                    published_direction, q, rtol=0.0, atol=1e-10
                )
                or not math.isclose(
                    published_distance, distance, rel_tol=1e-10, abs_tol=1e-10
                )
            ):
                errors.append(f"reference_geometry:{robot_id}:{reference_id}")
            ranging_variance = float(reference["ranging_variance"])
            predecessor_covariance = np.asarray(
                reference["predecessor_covariance"], dtype=float
            )
            predecessor_rate_bound = float(
                reference["predecessor_covariance_rate_bound"]
            )
            predecessor_speed_bound = float(reference["predecessor_speed_bound"])
            covariance_symmetric = np.allclose(
                predecessor_covariance,
                predecessor_covariance.T,
                rtol=0.0,
                atol=1e-12,
            )
            covariance_psd = covariance_symmetric and (
                float(np.linalg.eigvalsh(predecessor_covariance)[0]) >= -1e-12
            )
            if not covariance_symmetric:
                errors.append(
                    f"reference_covariance_symmetry:{robot_id}:{reference_id}"
                )
                node_inputs_valid = False
            if not covariance_psd:
                errors.append(f"reference_covariance_psd:{robot_id}:{reference_id}")
                node_inputs_valid = False
            if not math.isclose(float(np.linalg.norm(q)), 1.0, abs_tol=1e-12):
                errors.append(f"nonunit_direction:{robot_id}:{reference_id}")
                node_inputs_valid = False
            if reference["predecessor_snapshot_version"] != snapshot_version:
                errors.append(f"reference_version:{robot_id}:{reference_id}")

            if reference_id < 0:
                if (
                    reference_id not in {-1, -2, -3}
                    or reference["reference_kind"] != "base"
                    or reference["predecessor_local_index"] != 0
                    or not np.array_equal(
                        predecessor_covariance, np.zeros((2, 2), dtype=float)
                    )
                    or predecessor_rate_bound != 0.0
                    or predecessor_speed_bound != 0.0
                ):
                    errors.append(f"base_reference_state:{robot_id}:{reference_id}")
                    node_inputs_valid = False
                predecessor_velocity = np.zeros(2, dtype=float)
                predecessor_derivative = np.zeros((2, 2), dtype=float)
            else:
                predecessor_local_index = (reference_id - 1) % 7 + 1
                same_squad = (reference_id - 1) // 7 == (robot_id - 1) // 7
                expected_speed_bound = math.sqrt(2.0) * component_bound
                if (
                    reference["reference_kind"] != "uav"
                    or reference["predecessor_local_index"]
                        != predecessor_local_index
                    or not same_squad
                    or predecessor_local_index >= expected_local_index
                    or not math.isclose(
                        predecessor_speed_bound,
                        expected_speed_bound,
                        rel_tol=0.0,
                        abs_tol=1e-12,
                    )
                ):
                    errors.append(f"uav_reference_identity:{robot_id}:{reference_id}")
                    node_inputs_valid = False
                if reference_id not in reconstructed:
                    errors.append(f"nontopological_reference:{robot_id}:{reference_id}")
                    predecessor_derivative = np.zeros((2, 2), dtype=float)
                else:
                    predecessor_derivative = covariance_derivatives[reference_id]
                    expected_predecessor = np.asarray(
                        reconstructed[reference_id]["covariance"], dtype=float
                    )
                    if not np.allclose(
                        predecessor_covariance,
                        expected_predecessor,
                        rtol=0.0,
                        atol=1e-10,
                    ):
                        errors.append(
                            f"predecessor_covariance:{robot_id}:{reference_id}"
                        )
                    if not math.isclose(
                        predecessor_rate_bound,
                        float(reconstructed[reference_id]["L_P"]),
                        rel_tol=0.0,
                        abs_tol=1e-9,
                    ):
                        errors.append(
                            f"predecessor_rate_bound:{robot_id}:{reference_id}"
                        )
                predecessor_velocity = commands.get(reference_id, np.zeros(2))

            effective_variance = float(
                q @ predecessor_covariance @ q + ranging_variance
            )
            if not math.isfinite(effective_variance) or effective_variance <= 0.0:
                errors.append(f"effective_variance:{robot_id}:{reference_id}")
                node_inputs_valid = False
                continue
            beta = (
                math.sqrt(2.0) * component_bound + predecessor_speed_bound
            ) / distance
            effective_variance_rate_bound = (
                2.0
                * beta
                * float(np.linalg.norm(predecessor_covariance, ord=2))
                + predecessor_rate_bound
            )
            information += np.outer(q, q) / effective_variance
            information_rate_bound += (
                effective_variance_rate_bound / effective_variance**2
                + 2.0 * beta / effective_variance
            )

            relative_velocity = commands[robot_id] - predecessor_velocity
            qdot = (
                (np.eye(2) - np.outer(q, q)) @ relative_velocity / distance
            )
            wdot = float(
                2.0 * qdot @ predecessor_covariance @ q
                + q @ predecessor_derivative @ q
            )
            information_derivative += (
                -wdot / effective_variance**2 * np.outer(q, q)
                + (np.outer(qdot, q) + np.outer(q, qdot))
                / effective_variance
            )
            derivatives.append(
                {
                    "reference_id": reference_id,
                    "qdot": qdot.tolist(),
                    "wdot": wdot,
                    "effective_variance": effective_variance,
                }
            )

        if not node_inputs_valid:
            continue

        eigenvalues = np.linalg.eigvalsh(information)
        if eigenvalues[0] <= 1e-12 * eigenvalues[-1]:
            errors.append(f"singular_information:{robot_id}")
            continue
        covariance = np.linalg.inv(information)
        covariance = 0.5 * (covariance + covariance.T)
        covariance_derivative = -covariance @ information_derivative @ covariance
        covariance_derivative = 0.5 * (
            covariance_derivative + covariance_derivative.T
        )
        covariance_eigenvalues, covariance_eigenvectors = np.linalg.eigh(covariance)
        lambda_max = float(covariance_eigenvalues[-1])
        epsilon = 3.0 * math.sqrt(lambda_max)
        covariance_norm = float(np.linalg.norm(covariance, ord=2))
        covariance_rate_bound = covariance_norm**2 * information_rate_bound
        bar_nu = 3.0 * covariance_rate_bound / (2.0 * math.sqrt(lambda_max))
        gap = float(covariance_eigenvalues[-1] - covariance_eigenvalues[0])
        if gap <= 1e-12 * max(1.0, abs(lambda_max)):
            directional_lambda = float(
                np.linalg.eigvalsh(covariance_derivative)[-1]
            )
        else:
            eigenvector = covariance_eigenvectors[:, -1]
            directional_lambda = float(
                eigenvector @ covariance_derivative @ eigenvector
            )
        dplus_epsilon = (
            3.0 * directional_lambda / (2.0 * math.sqrt(lambda_max))
        )
        nu_inst = (
            3.0
            * covariance_norm**2
            * float(np.linalg.norm(information_derivative, ord=2))
            / (2.0 * math.sqrt(lambda_max))
        )
        if dplus_epsilon > nu_inst + 1e-9:
            errors.append(f"dini_exceeds_realized:{robot_id}")
        if nu_inst > bar_nu + 1e-9:
            errors.append(f"realized_exceeds_bound:{robot_id}")

        result = {
            "information": information.tolist(),
            "information_derivative": information_derivative.tolist(),
            "covariance": covariance.tolist(),
            "covariance_derivative": covariance_derivative.tolist(),
            "epsilon": epsilon,
            "L_P": covariance_rate_bound,
            "dplus_epsilon": dplus_epsilon,
            "nu_inst": nu_inst,
            "bar_nu": bar_nu,
            "reference_derivatives": derivatives,
        }
        reconstructed[robot_id] = result
        covariance_derivatives[robot_id] = covariance_derivative

        comparison_fields = {
            "information": information,
            "covariance": covariance,
            "covariance_derivative": covariance_derivative,
            "epsilon": epsilon,
            "covariance_rate_bound": covariance_rate_bound,
            "dplus_epsilon": dplus_epsilon,
            "nu_inst": nu_inst,
            "bar_nu": bar_nu,
        }
        for field, expected in comparison_fields.items():
            actual = node[field]
            if not np.allclose(
                np.asarray(actual, dtype=float),
                np.asarray(expected, dtype=float),
                rtol=1e-8,
                atol=1e-9,
            ):
                errors.append(f"serialized_mismatch:{robot_id}:{field}")

        normal_problem = node["normal_problem"]
        hard_only_problem = node["hard_only_problem"]
        normal_hash = _canonical_hard_problem_hash(normal_problem)
        hard_only_hash = _canonical_hard_problem_hash(hard_only_problem)
        if normal_problem != hard_only_problem:
            errors.append(f"hard_problem_projection:{robot_id}")
        if (
            node["committed_hard_problem_id"] != normal_hash
            or node["consumed_hard_problem_id"] != normal_hash
            or hard_only_hash != normal_hash
        ):
            errors.append(f"hard_problem_identity:{robot_id}")
        errors.extend(
            _audit_problem_and_solution(
                robot_id,
                "normal",
                normal_problem,
                node["normal_qp"],
                snapshot_version,
                allocation_version,
                full_commands[robot_id],
            )
        )
        errors.extend(
            _audit_problem_and_solution(
                robot_id,
                "hard_only",
                hard_only_problem,
                node["hard_only_qp"],
                snapshot_version,
                allocation_version,
                full_commands[robot_id],
            )
        )

    if expected_endpoint_count is None:
        expected_endpoint_count = 232 if len(raw_nodes) == 14 else len(endpoint_rows)
    if expected_reconstructed_count is None:
        expected_reconstructed_count = 119 if len(raw_nodes) == 14 else len(
            {(
                row.get("edge", {}).get("kind"),
                row.get("edge", {}).get("low"),
                row.get("edge", {}).get("high"),
                row.get("edge", {}).get("base_id"),
            ) for row in endpoint_rows}
        )
    if runtime["expected_endpoint_row_count"] != expected_endpoint_count:
        errors.append("serialized_expected_endpoint_count")
    if runtime["expected_reconstructed_row_count"] != expected_reconstructed_count:
        errors.append("serialized_expected_reconstructed_count")
    if runtime["observed_endpoint_row_count"] != len(endpoint_rows):
        errors.append("serialized_observed_endpoint_count")

    raw_node_by_id = {node["robot_id"]: node for node in raw_nodes}
    local_residuals: dict[tuple[CanonicalEdgeId, int], float] = {}
    grouped_rows: dict[
        CanonicalEdgeId, list[tuple[int, float, np.ndarray, float]]
    ] = {}
    observed_endpoints: list[tuple[CanonicalEdgeId, int]] = []
    expected_problem_rows: dict[
        tuple[int, CanonicalEdgeId], dict[str, object]
    ] = {}
    for row in endpoint_rows:
        if not validate_endpoint_primitive_schema(row):
            errors.append("endpoint_schema")
            continue
        if (
            row["snapshot_version"] != snapshot_version
            or row["allocation_version"] != allocation_version
        ):
            errors.append(f"endpoint_version:{row['owner']}")
        edge_raw = row["edge"]
        edge = CanonicalEdgeId(
            edge_raw["kind"],
            edge_raw["low"],
            edge_raw["high"],
            edge_raw["base_id"],
        )
        owner = row["owner"]
        observed_endpoints.append((edge, owner))
        if expected_endpoint_count == len(_PAPER_ENDPOINTS):
            expected_threshold = 850.0 if edge.kind == "localization" else 10.0
            expected_base_position = (
                _PAPER_BASE_POSITIONS[edge.base_id]
                if edge.base_id >= 0 else (0.0, 0.0)
            )
            if (
                edge_raw["threshold"] != expected_threshold
                or edge_raw["class_k_coefficient"] <= 0.0
                or edge_raw["class_k_power"] != 1
                or not np.array_equal(
                    np.asarray(edge_raw["base_position"], dtype=float),
                    np.asarray(expected_base_position, dtype=float),
                )
            ):
                errors.append(f"edge_authority:{edge}")
        command = commands.get(owner)
        if command is None:
            errors.append(f"endpoint_owner_command:{owner}")
            continue
        published = np.asarray(row["owner_applied_command"][:2], dtype=float)
        if not np.allclose(command, published, rtol=0.0, atol=1e-12):
            errors.append(f"endpoint_command_mismatch:{owner}")
        if edge.low not in reconstructed:
            errors.append(f"endpoint_low_node:{edge}")
            continue
        low_position = np.asarray(
            raw_node_by_id[edge.low]["interface_estimate"], dtype=float
        )
        if edge.base_id >= 0:
            second_position = np.asarray(edge_raw["base_position"], dtype=float)
            epsilon_high = 0.0
            bar_high = 0.0
        else:
            if edge.high not in reconstructed:
                errors.append(f"endpoint_high_node:{edge}")
                continue
            second_position = np.asarray(
                raw_node_by_id[edge.high]["interface_estimate"], dtype=float
            )
            epsilon_high = float(reconstructed[edge.high]["epsilon"])
            bar_high = float(reconstructed[edge.high]["bar_nu"])
        displacement = low_position - second_position
        separation = float(np.linalg.norm(displacement))
        normal = displacement / separation
        epsilon_low = float(reconstructed[edge.low]["epsilon"])
        if edge.kind == "localization":
            barrier = float(edge_raw["threshold"]) - separation - epsilon_low - epsilon_high
        else:
            barrier = separation - float(edge_raw["threshold"]) - epsilon_low - epsilon_high
        alpha_value = float(edge_raw["class_k_coefficient"]) * barrier ** int(
            edge_raw["class_k_power"]
        )
        expected_allocation = 1.0 if edge.base_id >= 0 else 0.5
        if edge.kind == "localization":
            expected_coefficient = -normal if owner == edge.low else normal
        else:
            expected_coefficient = normal if owner == edge.low else -normal
        owner_bar_nu = (
            float(reconstructed[edge.low]["bar_nu"])
            if owner == edge.low
            else bar_high
        )
        expected_constant = -owner_bar_nu + expected_allocation * alpha_value
        other_id = edge.high if owner == edge.low else edge.low
        if edge.kind == "localization":
            expected_name = (
                f"fixedCommCBF(base-{edge.base_id})"
                if edge.base_id >= 0
                else f"fixedCommCBF(#{other_id})"
            )
        else:
            expected_name = f"safetyCBF(#{other_id})"
        expected_problem_rows[(owner, edge)] = {
            "coefficients": [
                float(expected_coefficient[0]),
                float(expected_coefficient[1]),
                0.0,
            ],
            "constant": expected_constant,
            "post_reset_barrier": barrier,
            "snapshot_version": snapshot_version,
            "allocation_version": allocation_version,
            "name": expected_name,
        }
        if not math.isclose(
            float(row["allocation"]),
            expected_allocation,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            errors.append(f"endpoint_allocation:{edge}:{owner}")
        if not np.allclose(
            np.asarray(row["coefficient"], dtype=float),
            expected_coefficient,
            rtol=0.0,
            atol=1e-9,
        ):
            errors.append(f"endpoint_coefficient:{edge}:{owner}")
        if not math.isclose(
            float(row["constant"]),
            expected_constant,
            rel_tol=1e-8,
            abs_tol=1e-8,
        ):
            errors.append(f"endpoint_constant:{edge}:{owner}")
        comparisons = {
            "normal": normal,
            "separation": separation,
            "tightened_barrier": barrier,
            "alpha_value": alpha_value,
        }
        for field, expected in comparisons.items():
            if not np.allclose(
                np.asarray(edge_raw[field], dtype=float),
                np.asarray(expected, dtype=float),
                rtol=1e-8,
                atol=1e-8,
            ):
                errors.append(f"serialized_edge_mismatch:{edge}:{field}")
        residual = float(expected_constant + expected_coefficient @ command)
        local_residuals[(edge, owner)] = residual
        if residual < -1e-7:
            errors.append(f"unsafe_local_residual:{edge}:{owner}")
        grouped_rows.setdefault(edge, []).append(
            (owner, expected_allocation, expected_coefficient, expected_constant)
        )
        if row["qp_status"] != "optimal":
            errors.append(f"endpoint_qp_status:{edge}:{owner}")
        if row["hard_problem_id"] != raw_node_by_id[owner][
            "committed_hard_problem_id"
        ]:
            errors.append(f"endpoint_problem_identity:{edge}:{owner}")

    for node in raw_nodes:
        robot_id = node["robot_id"]
        expected_for_owner = {
            edge: fields
            for (owner, edge), fields in expected_problem_rows.items()
            if owner == robot_id
        }
        for label in ("normal_problem", "hard_only_problem"):
            problem = node[label]
            observed_for_owner: dict[CanonicalEdgeId, dict[str, object]] = {}
            for problem_row in problem["rows"]:
                edge_raw = problem_row["edge"]
                edge = CanonicalEdgeId(
                    edge_raw["kind"],
                    edge_raw["low"],
                    edge_raw["high"],
                    edge_raw["base_id"],
                )
                if edge in observed_for_owner:
                    errors.append(f"hard_problem_duplicate_row:{robot_id}:{label}")
                observed_for_owner[edge] = problem_row
            if set(observed_for_owner) != set(expected_for_owner):
                errors.append(f"hard_problem_row_universe:{robot_id}:{label}")
                continue
            for edge, expected in expected_for_owner.items():
                actual = observed_for_owner[edge]
                if (
                    actual["name"] != expected["name"]
                    or actual["snapshot_version"]
                        != expected["snapshot_version"]
                    or actual["allocation_version"]
                        != expected["allocation_version"]
                    or not np.allclose(
                        np.asarray(actual["coefficients"], dtype=float),
                        np.asarray(expected["coefficients"], dtype=float),
                        rtol=0.0,
                        atol=1e-9,
                    )
                    or not math.isclose(
                        float(actual["constant"]),
                        float(expected["constant"]),
                        rel_tol=1e-8,
                        abs_tol=1e-8,
                    )
                    or not math.isclose(
                        float(actual["post_reset_barrier"]),
                        float(expected["post_reset_barrier"]),
                        rel_tol=1e-8,
                        abs_tol=1e-8,
                    )
                ):
                    errors.append(
                        f"hard_problem_row:{robot_id}:{label}:{edge}"
                    )

    full_residuals: dict[CanonicalEdgeId, float] = {}
    for edge, rows in grouped_rows.items():
        expected_owners = {edge.low} if edge.base_id >= 0 else {edge.low, edge.high}
        actual_owners = {row[0] for row in rows}
        if actual_owners != expected_owners or len(rows) != len(expected_owners):
            errors.append(f"endpoint_closure:{edge}")
            continue
        if not math.isclose(
            sum(row[1] for row in rows),
            1.0,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            errors.append(f"allocation_sum:{edge}")
        full_residuals[edge] = sum(
            local_residuals[(edge, row[0])] for row in rows
        )
        if full_residuals[edge] < -1e-7:
            errors.append(f"unsafe_full_residual:{edge}")

    if len(endpoint_rows) != expected_endpoint_count:
        errors.append("endpoint_cardinality")
    if len(grouped_rows) != expected_reconstructed_count:
        errors.append("reconstructed_cardinality")
    if (
        expected_endpoint_count == len(_PAPER_ENDPOINTS)
        and expected_reconstructed_count == len(_PAPER_EDGES)
        and Counter(observed_endpoints) != Counter(_PAPER_ENDPOINTS)
    ):
        errors.append("endpoint_universe")
    component_maxima = {
        "vx": max(abs(float(command[0])) for command in full_commands.values()),
        "vy": max(abs(float(command[1])) for command in full_commands.values()),
        "yaw_rate": max(
            abs(float(command[2])) for command in full_commands.values()
        ),
    }
    if any(
        not math.isclose(
            float(runtime["component_maxima"][field]),
            value,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
        for field, value in component_maxima.items()
    ):
        errors.append("serialized_component_maxima")
    if local_residuals:
        local_minimum = min(local_residuals.values())
        if not math.isclose(
            float(runtime["local_residual_minimum"]),
            local_minimum,
            rel_tol=1e-9,
            abs_tol=1e-9,
        ):
            errors.append("serialized_local_residual_minimum")
    if full_residuals:
        full_minimum = min(full_residuals.values())
        if not math.isclose(
            float(runtime["reconstructed_residual_minimum"]),
            full_minimum,
            rel_tol=1e-9,
            abs_tol=1e-9,
        ):
            errors.append("serialized_full_residual_minimum")
    if runtime["complete_finite_snapshot"] is not True:
        errors.append("complete_finite_snapshot")
    return ControllerReconstruction(
        reconstructed,
        local_residuals,
        full_residuals,
        tuple(errors),
    )


def reconstruct_controller_primitives(
    controller: dict[str, object],
    endpoint_rows: Sequence[dict[str, object]],
    expected_endpoint_count: int | None = None,
    expected_reconstructed_count: int | None = None,
) -> ControllerReconstruction:
    if (
        not isinstance(endpoint_rows, Sequence)
        or isinstance(endpoint_rows, (str, bytes, dict))
    ):
        return ControllerReconstruction(
            {}, {}, {}, ("endpoint_record_sequence",)
        )
    try:
        return _reconstruct_controller_primitives_impl(
            controller,
            endpoint_rows,
            expected_endpoint_count,
            expected_reconstructed_count,
        )
    except Exception:
        return ControllerReconstruction(
            {}, {}, {}, ("controller_internal_error",)
        )


def _finite_number(value: object) -> bool:
    if type(value) is int:
        return -(1 << 63) <= value <= (1 << 63) - 1
    return type(value) is float and math.isfinite(value)


def _integer(value: object) -> bool:
    return type(value) is int


def _int32(value: object) -> bool:
    return _integer(value) and -(1 << 31) <= value <= (1 << 31) - 1


def _uint64(value: object) -> bool:
    return _integer(value) and 0 <= value <= (1 << 64) - 1


def _vector(value: object, length: int) -> bool:
    return (
        isinstance(value, list)
        and len(value) == length
        and all(_finite_number(component) for component in value)
    )


def _matrix2(value: object) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 2
        and all(_vector(row, 2) for row in value)
    )


def _nonempty_string(value: object) -> bool:
    return isinstance(value, str) and bool(value)


def _valid_reference(reference: object, snapshot_version: int) -> bool:
    if not isinstance(reference, dict):
        return False
    required = {
        "canonical_reference_id",
        "reference_kind",
        "direction",
        "distance",
        "ranging_variance",
        "predecessor_local_index",
        "predecessor_snapshot_version",
        "predecessor_covariance",
        "predecessor_covariance_rate_bound",
        "predecessor_speed_bound",
    }
    if set(reference) != required:
        return False
    reference_id = reference["canonical_reference_id"]
    kind = reference["reference_kind"]
    if not _integer(reference_id) or kind not in {"base", "uav"}:
        return False
    if (kind == "base") != (reference_id < 0):
        return False
    local_index = reference["predecessor_local_index"]
    if not _integer(local_index) or local_index < 0:
        return False
    if kind == "base" and local_index != 0:
        return False
    return (
        _vector(reference["direction"], 2)
        and _finite_number(reference["distance"])
        and reference["distance"] > 0.0
        and _finite_number(reference["ranging_variance"])
        and reference["ranging_variance"] > 0.0
        and reference["predecessor_snapshot_version"] == snapshot_version
        and _matrix2(reference["predecessor_covariance"])
        and _finite_number(reference["predecessor_covariance_rate_bound"])
        and reference["predecessor_covariance_rate_bound"] >= 0.0
        and _finite_number(reference["predecessor_speed_bound"])
        and reference["predecessor_speed_bound"] >= 0.0
    )


def _valid_qp(qp: object) -> bool:
    return (
        isinstance(qp, dict)
        and set(qp)
            == {"status", "minimum_residual", "hard_problem_id", "solution"}
        and _nonempty_string(qp["status"])
        and _finite_number(qp["minimum_residual"])
        and _nonempty_string(qp["hard_problem_id"])
        and _vector(qp["solution"], 3)
    )


def _valid_hard_problem(problem: object, owner: int) -> bool:
    if not isinstance(problem, dict) or set(problem) != {
        "owner",
        "control_size",
        "planar_component_max",
        "yaw_rate_max",
        "snapshot_version",
        "allocation_version",
        "bounds",
        "rows",
        "hard_problem_id",
    }:
        return False
    if (
        problem["owner"] != owner
        or not _int32(owner)
        or owner <= 0
        or not _int32(problem["control_size"])
        or problem["control_size"] != 3
        or not _finite_number(problem["planar_component_max"])
        or not _finite_number(problem["yaw_rate_max"])
        or not _uint64(problem["snapshot_version"])
        or problem["snapshot_version"] == 0
        or not _uint64(problem["allocation_version"])
        or problem["allocation_version"] == 0
        or not _nonempty_string(problem["hard_problem_id"])
    ):
        return False
    bounds = problem["bounds"]
    rows = problem["rows"]
    if not isinstance(bounds, list) or not isinstance(rows, list):
        return False
    for bound in bounds:
        if (
            not isinstance(bound, dict)
            or set(bound) != {"control_index", "coefficient", "limit"}
            or not _int32(bound["control_index"])
            or not 0 <= bound["control_index"] < problem["control_size"]
            or not _finite_number(bound["coefficient"])
            or not _finite_number(bound["limit"])
        ):
            return False
    for row in rows:
        edge = row.get("edge") if isinstance(row, dict) else None
        if (
            not isinstance(row, dict)
            or set(row) != {
                "edge",
                "owner",
                "name",
                "coefficients",
                "constant",
                "post_reset_barrier",
                "snapshot_version",
                "allocation_version",
            }
            or not isinstance(edge, dict)
            or set(edge) != {"kind", "low", "high", "base_id"}
            or edge["kind"] not in {"localization", "collision"}
            or not all(_int32(edge[field]) for field in ("low", "high", "base_id"))
            or not 1 <= edge["low"] <= 14
            or not 1 <= edge["high"] <= 14
            or edge["base_id"] not in {-1, 0, 1, 2}
            or row["owner"] != owner
            or not _nonempty_string(row["name"])
            or not _vector(row["coefficients"], problem["control_size"])
            or not _finite_number(row["constant"])
            or not _finite_number(row["post_reset_barrier"])
            or not _uint64(row["snapshot_version"])
            or not _uint64(row["allocation_version"])
        ):
            return False
    return True


def _canonical_hard_problem_hash(problem: dict[str, object]) -> str:
    digest = 1469598103934665603

    def append(raw: bytes) -> None:
        nonlocal digest
        for byte in raw:
            digest ^= byte
            digest = (digest * 1099511628211) & ((1 << 64) - 1)

    def integer(value: int) -> None:
        append(struct.pack("<i", value))

    def uint64(value: int) -> None:
        append(struct.pack("<Q", value))

    def floating(value: float) -> None:
        append(struct.pack("<d", float(value)))

    integer(problem["owner"])
    integer(problem["control_size"])
    floating(problem["planar_component_max"])
    floating(problem["yaw_rate_max"])
    uint64(problem["snapshot_version"])
    uint64(problem["allocation_version"])
    uint64(len(problem["bounds"]))
    for bound in problem["bounds"]:
        integer(bound["control_index"])
        floating(bound["coefficient"])
        floating(bound["limit"])
    uint64(len(problem["rows"]))
    for row in problem["rows"]:
        edge = row["edge"]
        integer(0 if edge["kind"] == "localization" else 1)
        integer(edge["low"])
        integer(edge["high"])
        integer(edge["base_id"])
        integer(row["owner"])
        name = row["name"].encode("utf-8")
        uint64(len(name))
        append(name)
        uint64(len(row["coefficients"]))
        for coefficient in row["coefficients"]:
            floating(coefficient)
        floating(row["constant"])
        floating(row["post_reset_barrier"])
        uint64(row["snapshot_version"])
        uint64(row["allocation_version"])
    return f"{digest:016x}"


def _problem_residuals(
    problem: dict[str, object], solution: Sequence[float]
) -> list[float]:
    vector = np.asarray(solution, dtype=float)
    residuals = [
        float(bound["coefficient"] * vector[bound["control_index"]] + bound["limit"])
        for bound in problem["bounds"]
    ]
    residuals.extend(
        float(row["constant"] + np.asarray(row["coefficients"], dtype=float) @ vector)
        for row in problem["rows"]
    )
    return residuals


def _hard_problem_is_feasible(problem: dict[str, object]) -> bool:
    planes: list[tuple[np.ndarray, float]] = []
    for bound in problem["bounds"]:
        coefficients = np.zeros(problem["control_size"], dtype=float)
        coefficients[bound["control_index"]] = bound["coefficient"]
        planes.append((coefficients, -float(bound["limit"])))
    for row in problem["rows"]:
        planes.append(
            (
                np.asarray(row["coefficients"], dtype=float),
                -float(row["constant"]),
            )
        )
    for selected in combinations(planes, problem["control_size"]):
        matrix = np.vstack([plane[0] for plane in selected])
        if np.linalg.matrix_rank(matrix, tol=1e-12) != problem["control_size"]:
            continue
        candidate = np.linalg.solve(
            matrix, np.asarray([plane[1] for plane in selected], dtype=float)
        )
        if all(float(coefficients @ candidate) >= rhs - 1e-9 for coefficients, rhs in planes):
            return True
    return False


def _exact_hard_problem_structure(
    problem: dict[str, object],
    robot_id: int,
    snapshot_version: int,
    allocation_version: int,
) -> bool:
    required_bounds = (
        (0, 1.0, 25.0),
        (0, -1.0, 25.0),
        (1, 1.0, 25.0),
        (1, -1.0, 25.0),
        (2, 1.0, 0.35),
        (2, -1.0, 0.35),
    )
    observed_bounds = tuple(
        (bound["control_index"], bound["coefficient"], bound["limit"])
        for bound in problem["bounds"]
    )
    return (
        problem["owner"] == robot_id
        and problem["control_size"] == 3
        and problem["planar_component_max"] == 25.0
        and problem["yaw_rate_max"] == 0.35
        and problem["snapshot_version"] == snapshot_version
        and problem["allocation_version"] == allocation_version
        and observed_bounds == required_bounds
    )


def _audit_problem_and_solution(
    robot_id: int,
    label: str,
    problem: dict[str, object],
    qp: dict[str, object],
    snapshot_version: int,
    allocation_version: int,
    applied_command: np.ndarray,
) -> list[str]:
    errors: list[str] = []
    qp_label = f"{label}_qp"
    if not _exact_hard_problem_structure(
        problem, robot_id, snapshot_version, allocation_version
    ):
        errors.append(f"hard_problem_structure:{robot_id}:{label}")
    canonical_hash = _canonical_hard_problem_hash(problem)
    if problem["hard_problem_id"] != canonical_hash:
        errors.append(f"hard_problem_hash:{robot_id}:{label}")
    solution = np.asarray(qp["solution"], dtype=float)
    residuals = _problem_residuals(problem, solution)
    solution_valid = bool(residuals) and min(residuals) >= -1e-7
    if label == "normal" and not np.allclose(
        solution, applied_command, rtol=0.0, atol=1e-12
    ):
        errors.append(f"normal_solution:{robot_id}")
        solution_valid = False
    if label == "hard_only" and not solution_valid:
        errors.append(f"hard_only_solution:{robot_id}")
    minimum = min(residuals) if residuals else math.inf
    if not math.isclose(
        float(qp["minimum_residual"]), minimum, rel_tol=1e-9, abs_tol=1e-9
    ):
        errors.append(f"qp_minimum_residual:{robot_id}:{label}")
    independently_feasible = solution_valid or _hard_problem_is_feasible(problem)
    expected_status = "optimal" if independently_feasible and solution_valid else "infeasible"
    if qp["status"] != expected_status:
        errors.append(f"qp_status:{robot_id}:{qp_label}")
    if qp["hard_problem_id"] != canonical_hash:
        errors.append(f"qp_problem_identity:{robot_id}:{qp_label}")
    return errors


def _valid_node(node: object, snapshot_version: int, allocation_version: int) -> bool:
    if not isinstance(node, dict):
        return False
    required = {
        "robot_id",
        "local_index",
        "snapshot_version",
        "allocation_version",
        "interface_estimate",
        "node_component_bound",
        "references",
        "information",
        "covariance",
        "covariance_derivative",
        "covariance_rate_bound",
        "epsilon",
        "dplus_epsilon",
        "nu_inst",
        "bar_nu",
        "applied_command",
        "normal_qp",
        "hard_only_qp",
        "normal_problem",
        "hard_only_problem",
        "committed_hard_problem_id",
        "consumed_hard_problem_id",
    }
    if set(node) != required:
        return False
    references = node["references"]
    finite_scalars = (
        "node_component_bound",
        "covariance_rate_bound",
        "epsilon",
        "dplus_epsilon",
        "nu_inst",
        "bar_nu",
    )
    return (
        _integer(node["robot_id"])
        and node["robot_id"] > 0
        and _integer(node["local_index"])
        and node["local_index"] > 0
        and node["snapshot_version"] == snapshot_version
        and node["allocation_version"] == allocation_version
        and _vector(node["interface_estimate"], 2)
        and all(_finite_number(node[field]) for field in finite_scalars)
        and node["node_component_bound"] > 0.0
        and isinstance(references, list)
        and len(references) >= 2
        and all(_valid_reference(reference, snapshot_version) for reference in references)
        and _matrix2(node["information"])
        and _matrix2(node["covariance"])
        and _matrix2(node["covariance_derivative"])
        and _vector(node["applied_command"], 3)
        and _valid_qp(node["normal_qp"])
        and _valid_qp(node["hard_only_qp"])
        and _valid_hard_problem(node["normal_problem"], node["robot_id"])
        and _valid_hard_problem(node["hard_only_problem"], node["robot_id"])
        and _nonempty_string(node["committed_hard_problem_id"])
        and _nonempty_string(node["consumed_hard_problem_id"])
    )


def _validate_controller_primitive_schema(
    row: object,
    frozen_expected_node_count: int,
) -> bool:
    try:
        if not isinstance(row, dict):
            return False
        required = {
            "record_type",
            "schema_version",
            "campaign_id",
            "condition",
            "trajectory_seed",
            "range_noise_seed",
            "frame_index",
            "runtime",
            "analyzer_only",
        }
        if set(row) != required or row["record_type"] != "controller_interval":
            return False
        if row["schema_version"] != "cbf2026-qualified-evidence-v1":
            return False
        if not all(
            _nonempty_string(row[field]) for field in ("campaign_id", "condition")
        ):
            return False
        if not all(
            _integer(row[field])
            for field in ("trajectory_seed", "range_noise_seed", "frame_index")
        ) or row["frame_index"] < 0:
            return False
        runtime = row["runtime"]
        runtime_fields = {
            "snapshot_version",
            "allocation_version",
            "nodes",
            "expected_node_count",
            "expected_endpoint_row_count",
            "expected_reconstructed_row_count",
            "observed_endpoint_row_count",
            "complete_finite_snapshot",
            "reset",
            "component_maxima",
            "local_residual_minimum",
            "reconstructed_residual_minimum",
            "abort_reason",
            "complete",
        }
        if not isinstance(runtime, dict) or set(runtime) != runtime_fields:
            return False
        snapshot_version = runtime["snapshot_version"]
        allocation_version = runtime["allocation_version"]
        if not _uint64(snapshot_version) or snapshot_version <= 0:
            return False
        if not _uint64(allocation_version) or allocation_version <= 0:
            return False
        nodes = runtime["nodes"]
        expected_nodes = runtime["expected_node_count"]
        if (
            not isinstance(nodes, list)
            or not _integer(expected_nodes)
            or expected_nodes != frozen_expected_node_count
            or len(nodes) != expected_nodes
            or not all(
                _valid_node(node, snapshot_version, allocation_version)
                for node in nodes
            )
        ):
            return False
        if sorted(node["robot_id"] for node in nodes) != list(
            range(1, expected_nodes + 1)
        ):
            return False
        for field in (
            "expected_endpoint_row_count",
            "expected_reconstructed_row_count",
            "observed_endpoint_row_count",
        ):
            if not _integer(runtime[field]) or runtime[field] <= 0:
                return False
        reset = runtime["reset"]
        if (
            not isinstance(reset, dict)
            or set(reset) != {"attempted", "guard_status", "committed"}
            or type(reset["attempted"]) is not bool
            or type(reset["committed"]) is not bool
            or reset["guard_status"] not in {"accepted", "rejected", "not-attempted"}
        ):
            return False
        maxima = runtime["component_maxima"]
        if (
            not isinstance(maxima, dict)
            or set(maxima) != {"vx", "vy", "yaw_rate"}
            or not all(_finite_number(value) for value in maxima.values())
        ):
            return False
        if not _finite_number(runtime["local_residual_minimum"]):
            return False
        if not _finite_number(runtime["reconstructed_residual_minimum"]):
            return False
        if not isinstance(runtime["abort_reason"], str):
            return False
        if type(runtime["complete"]) is not bool:
            return False
        if type(runtime["complete_finite_snapshot"]) is not bool:
            return False
        analyzer = row["analyzer_only"]
        if not isinstance(analyzer, dict) or set(analyzer) != {"truth"}:
            return False
        truth = analyzer["truth"]
        return (
            isinstance(truth, list)
            and len(truth) == expected_nodes
            and all(
                isinstance(item, dict)
                and set(item) == {"robot_id", "position"}
                and _integer(item["robot_id"])
                and _vector(item["position"], 2)
                for item in truth
            )
            and sorted(item["robot_id"] for item in truth)
                == list(range(1, expected_nodes + 1))
        )
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def validate_controller_primitive_schema(row: object) -> bool:
    return _validate_controller_primitive_schema(row, 14)


def validate_endpoint_primitive_schema(row: object) -> bool:
    try:
        if not isinstance(row, dict):
            return False
        required = {
            "record_type",
            "schema_version",
            "campaign_id",
            "condition",
            "trajectory_seed",
            "range_noise_seed",
            "frame_index",
            "edge",
            "owner",
            "allocation",
            "coefficient",
            "constant",
            "snapshot_version",
            "allocation_version",
            "owner_applied_command",
            "qp_status",
            "hard_problem_id",
        }
        if set(row) != required or row["record_type"] != "endpoint_row":
            return False
        if row["schema_version"] != "cbf2026-qualified-evidence-v1":
            return False
        if not all(
            _nonempty_string(row[field]) for field in ("campaign_id", "condition")
        ):
            return False
        for field in (
            "trajectory_seed",
            "range_noise_seed",
            "frame_index",
            "owner",
            "snapshot_version",
            "allocation_version",
        ):
            if not _integer(row[field]):
                return False
        if row["frame_index"] < 0 or row["owner"] <= 0:
            return False
        if row["snapshot_version"] <= 0 or row["allocation_version"] <= 0:
            return False
        edge = row["edge"]
        edge_fields = {
            "kind",
            "low",
            "high",
            "base_id",
            "threshold",
            "base_position",
            "normal",
            "separation",
            "tightened_barrier",
            "class_k_coefficient",
            "class_k_power",
            "alpha_value",
        }
        if not isinstance(edge, dict) or set(edge) != edge_fields:
            return False
        if edge["kind"] not in {"localization", "collision"}:
            return False
        if not all(_integer(edge[field]) for field in ("low", "high", "base_id")):
            return False
        if not _integer(edge["class_k_power"]):
            return False
        if not _vector(edge["base_position"], 2) or not _vector(edge["normal"], 2):
            return False
        if not all(
            _finite_number(edge[field])
            for field in (
                "threshold",
                "separation",
                "tightened_barrier",
                "class_k_coefficient",
                "alpha_value",
            )
        ):
            return False
        return (
            _finite_number(row["allocation"])
            and _vector(row["coefficient"], 2)
            and _finite_number(row["constant"])
            and _vector(row["owner_applied_command"], 3)
            and _nonempty_string(row["qp_status"])
            and _nonempty_string(row["hard_problem_id"])
        )
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def synthesize_missing_mission(
    schedule: FrozenMissionSchedule,
    reason: str,
) -> MissingMissionRows:
    if schedule.robots != tuple(range(1, 15)):
        raise ValueError("qualified controller evidence requires the frozen 14-UAV universe")
    if schedule.frames <= 0:
        raise ValueError("frozen mission schedule must contain at least one frame")
    if len(set(schedule.estimator_conditions)) != len(schedule.estimator_conditions):
        raise ValueError("estimator conditions must be unique")
    if schedule.controller_condition not in schedule.estimator_conditions:
        raise ValueError("controller condition must be an estimator condition")
    robot_count = len(schedule.robots)
    estimator_frame_count = max(0, schedule.frames - 1)
    estimator_count = (
        len(schedule.estimator_conditions)
        * estimator_frame_count
        * robot_count
    )

    initialization = LazyKeySequence(
        robot_count,
        lambda index: (
            schedule.campaign_id,
            schedule.trajectory_seed,
            schedule.range_noise_seed,
            0,
            schedule.robots[index],
        ),
    )

    def estimator_key(index: int) -> tuple[object, ...]:
        per_condition = estimator_frame_count * robot_count
        condition_index, condition_offset = divmod(index, per_condition)
        frame_offset, robot_index = divmod(condition_offset, robot_count)
        return (
            schedule.campaign_id,
            schedule.estimator_conditions[condition_index],
            schedule.trajectory_seed,
            schedule.range_noise_seed,
            frame_offset + 1,
            schedule.robots[robot_index],
        )

    estimator = LazyKeySequence(estimator_count, estimator_key)
    controller = LazyKeySequence(
        schedule.frames,
        lambda interval: (
            schedule.campaign_id,
            schedule.controller_condition,
            schedule.trajectory_seed,
            schedule.range_noise_seed,
            interval,
        ),
    )

    endpoint_per_frame = len(_PAPER_ENDPOINTS)

    def endpoint_key(index: int) -> tuple[object, ...]:
        frame, endpoint_index = divmod(index, endpoint_per_frame)
        edge, owner = _PAPER_ENDPOINTS[endpoint_index]
        return (
            schedule.campaign_id,
            schedule.controller_condition,
            schedule.trajectory_seed,
            schedule.range_noise_seed,
            frame,
            edge,
            owner,
        )

    endpoint = LazyKeySequence(
        schedule.frames * endpoint_per_frame,
        endpoint_key,
    )

    edge_per_frame = len(_PAPER_EDGES)

    def reconstructed_key(index: int) -> tuple[object, ...]:
        frame, edge_index = divmod(index, edge_per_frame)
        return (
            schedule.campaign_id,
            schedule.controller_condition,
            schedule.trajectory_seed,
            schedule.range_noise_seed,
            frame,
            _PAPER_EDGES[edge_index],
        )

    reconstructed = LazyKeySequence(
        schedule.frames * edge_per_frame,
        reconstructed_key,
    )
    return MissingMissionRows(
        initialization=initialization,
        estimator=estimator,
        controller=controller,
        endpoint=endpoint,
        reconstructed=reconstructed,
        reset=(),
        mission=MissionResult(
            key=(
                schedule.campaign_id,
                schedule.trajectory_seed,
                schedule.range_noise_seed,
            ),
            success=False,
            reason=reason,
        ),
    )
