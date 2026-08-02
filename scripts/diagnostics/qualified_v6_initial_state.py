"""Strict, pure reconstruction for the frozen v6 one-step input universe.

This module declares and reconstructs mathematical state only.  It neither
launches a binary nor evaluates a gate verdict; the later exact-binary gate
consumes its reconstructed barriers and local planar radii.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

from scripts.diagnostics import qualified_initial_state as v1
from scripts.diagnostics.hard_interior_selection import (
    solve_planar_hard_row_chebyshev,
)


EXPECTED_V2_PATH = (
    Path(__file__).resolve().parents[2]
    / "config/diagnostics/qualified_initial_family_v2.json"
)
V1_NAMESPACE = "cbf2026-v5-initial"
EXPECTED_AUDIT_SEEDS = tuple(range(2026080201, 2026080301))
EXPECTED_REGISTERED_SEEDS = tuple(range(2026080201, 2026080211))


@dataclass(frozen=True)
class V6OneStepState:
    """One single-integrator update and its independently recomputed state.

    Chronology is fixed: validate the current geometry, apply the supplied
    planar command exactly once, then recompute certificates, hard rows, and
    planar Chebyshev radii at that next geometry.
    """

    chronology: tuple[str, ...]
    current_positions_m: tuple[tuple[float, float], ...]
    applied_planar_commands_mps: tuple[tuple[float, float], ...]
    next_positions_m: tuple[tuple[float, float], ...]
    next_certificates: tuple[v1.NodeCertificate, ...]
    next_barriers: tuple[v1.BarrierValue, ...]
    next_endpoint_rows: tuple[v1.EndpointRow, ...]
    next_local_radii_mps: tuple[float, ...]


def _canonical_json_bytes(value: object) -> bytes:
    return json.dumps(
        value, ensure_ascii=False, allow_nan=False, sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def v6_family_semantic_sha256(family: Mapping) -> str:
    if not isinstance(family, Mapping):
        raise ValueError("v6 initial family must be an object")
    semantic = {
        key: value for key, value in family.items()
        if key != "semantic_sha256"
    }
    return hashlib.sha256(_canonical_json_bytes(semantic)).hexdigest()


def _keys(value: object, expected: set[str], label: str) -> Mapping:
    if not isinstance(value, Mapping) or set(value) != expected:
        raise ValueError(f"{label} has an invalid exact schema")
    return value


def _float(value: object, expected: float, label: str) -> None:
    if type(value) is not float or not math.isfinite(value) or value != expected:
        raise ValueError(f"{label} differs from the frozen contract")


def _integer(value: object, expected: int, label: str) -> None:
    if type(value) is not int or value != expected:
        raise ValueError(f"{label} differs from the frozen contract")


def _boolean(value: object, expected: bool, label: str) -> None:
    if type(value) is not bool or value is not expected:
        raise ValueError(f"{label} differs from the frozen contract")


def _legacy_v1_family(family: Mapping) -> dict:
    """Build the exact v1 mathematical contract used as the frozen source."""
    legacy = {
        key: copy.deepcopy(family[key])
        for key in (
            "template_positions_m", "perturbation", "deployment", "schedule",
            "production_contract", "admission", "frozen_summary",
        )
    }
    legacy["schema_version"] = "cbf2026-qualified-initial-family-v1"
    legacy["namespace"] = V1_NAMESPACE
    legacy["semantic_sha256"] = v1.family_semantic_sha256(legacy)
    return v1.validate_qualified_initial_family(legacy)


def validate_qualified_v6_initial_family(family: Mapping) -> dict:
    """Return a deep copy only for the one exact v6 static declaration."""
    _keys(
        family,
        {
            "schema_version", "namespace", "template_positions_m",
            "perturbation", "deployment", "schedule", "production_contract",
            "admission", "frozen_summary", "controller_policy",
            "one_step_gate", "semantic_sha256",
        },
        "v6 initial family",
    )
    if family["schema_version"] != "cbf2026-qualified-initial-family-v2":
        raise ValueError("v6 initial family schema version is not frozen")
    if family["namespace"] != "cbf2026-v6-initial":
        raise ValueError("v6 initial family namespace is not frozen")
    semantic_sha = family["semantic_sha256"]
    if (
        type(semantic_sha) is not str or len(semantic_sha) != 64
        or any(character not in "0123456789abcdef" for character in semantic_sha)
        or semantic_sha != v6_family_semantic_sha256(family)
    ):
        raise ValueError("v6 initial family semantic SHA-256 does not match")

    # The legacy projection proves every v1 field, template coordinate, graph
    # cardinality, and static summary is still exactly the consumed contract.
    legacy = _legacy_v1_family(family)
    if legacy["production_contract"]["class_k_coefficient"] != 0.1:
        raise ValueError("v6 class-K coefficient is not frozen")

    policy = _keys(
        family["controller_policy"],
        {
            "class_k_coefficient", "class_k_power", "mode", "fraction",
            "cap_mps", "planar_component_max_mps", "yaw_enters_radius",
        },
        "controller policy",
    )
    _float(policy["class_k_coefficient"], 0.1, "controller class-K coefficient")
    _integer(policy["class_k_power"], 1, "controller class-K power")
    if policy["mode"] != "planar-chebyshev-fraction-cap-v1":
        raise ValueError("controller policy mode is not frozen")
    _float(policy["fraction"], 0.1, "controller policy fraction")
    _float(policy["cap_mps"], 0.1, "controller policy cap")
    _float(policy["planar_component_max_mps"], 25.0, "controller component bound")
    _boolean(policy["yaw_enters_radius"], False, "controller yaw policy")
    if policy["class_k_coefficient"] != legacy["production_contract"]["class_k_coefficient"]:
        raise ValueError("v6 class-K declarations disagree")
    if policy["planar_component_max_mps"] != legacy["production_contract"]["planar_component_max_mps"]:
        raise ValueError("v6 component bounds disagree")

    gate = _keys(
        family["one_step_gate"],
        {
            "dt_s", "minimum_next_barrier_m", "barrier_comparison",
            "minimum_next_local_radius_mps", "clamp", "resample", "retry",
        },
        "one-step gate declaration",
    )
    _float(gate["dt_s"], 0.5, "one-step dt")
    _float(gate["minimum_next_barrier_m"], 0.0, "one-step barrier threshold")
    if gate["barrier_comparison"] != "strictly-greater":
        raise ValueError("one-step barrier comparison is not frozen")
    _float(gate["minimum_next_local_radius_mps"], 0.05, "one-step radius threshold")
    _boolean(gate["clamp"], False, "one-step clamp")
    _boolean(gate["resample"], False, "one-step resample")
    _boolean(gate["retry"], False, "one-step retry")
    return copy.deepcopy(dict(family))


def _reject_duplicate_keys(pairs: list[tuple[str, object]]) -> dict:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def load_qualified_v6_initial_family(path: Path) -> dict:
    """Load only the canonical regular v2 declaration, never a substitute."""
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ValueError("v6 initial family must be a regular file")
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError("v6 initial family is not valid UTF-8 JSON") from error
    checked = validate_qualified_v6_initial_family(value)
    if path.resolve() != EXPECTED_V2_PATH.resolve():
        raise ValueError("v6 initial family provenance is not canonical")
    return checked


def materialize_v6_seed_positions(
    family: Mapping, seed: int,
) -> tuple[tuple[float, float], ...]:
    """Materialize exactly one declared audit seed with the preserved v1 salt."""
    checked = validate_qualified_v6_initial_family(family)
    if type(seed) is not int or seed not in EXPECTED_AUDIT_SEEDS:
        raise ValueError("trajectory seed is outside the frozen audit universe")
    legacy = _legacy_v1_family(checked)
    return v1._materialize_seed_positions(legacy, seed)


def audit_frozen_v6_initial_family(family: Mapping) -> v1.FrozenInitialFamilyAudit:
    """Recompute the unchanged 100-seed mathematical initial universe."""
    checked = validate_qualified_v6_initial_family(family)
    schedule = checked["schedule"]
    if tuple(schedule["registered_trajectory_seeds"]) != EXPECTED_REGISTERED_SEEDS:
        raise ValueError("registered seed universe was replaced")
    audit_seeds = tuple(range(schedule["audit_seed_first"], schedule["audit_seed_last"] + 1))
    if audit_seeds != EXPECTED_AUDIT_SEEDS or schedule["audit_seed_count"] != len(audit_seeds):
        raise ValueError("audit seed universe was replaced")
    return v1.audit_frozen_initial_family(_legacy_v1_family(checked))


def _normalize_commands(
    commands: Sequence[Sequence[float]], component_max_mps: float,
) -> tuple[tuple[float, float], ...]:
    if type(commands) not in (tuple, list) or len(commands) != 14:
        raise ValueError("one-step reconstruction requires 14 planar commands")
    normalized = []
    for robot_id, command in enumerate(commands, start=1):
        if type(command) not in (tuple, list) or len(command) != 2:
            raise ValueError(f"UAV {robot_id} command must have two components")
        if any(type(value) not in (int, float) or isinstance(value, bool) or not math.isfinite(float(value)) for value in command):
            raise ValueError(f"UAV {robot_id} command is not finite numeric data")
        planar = (float(command[0]), float(command[1]))
        if any(abs(value) > component_max_mps for value in planar):
            raise ValueError(f"UAV {robot_id} command exceeds the component bound")
        normalized.append(planar)
    return tuple(normalized)


def _local_hard_problem(
    robot_id: int, rows: tuple[v1.EndpointRow, ...], component_max_mps: float,
) -> dict:
    return {
        "owner": robot_id,
        "control_size": 3,
        "planar_component_max": float(component_max_mps),
        "yaw_rate_max": 0.35,
        "snapshot_version": 1,
        "allocation_version": 1,
        "bounds": [
            {"control_index": 0, "coefficient": 1.0, "limit": float(component_max_mps)},
            {"control_index": 0, "coefficient": -1.0, "limit": float(component_max_mps)},
            {"control_index": 1, "coefficient": 1.0, "limit": float(component_max_mps)},
            {"control_index": 1, "coefficient": -1.0, "limit": float(component_max_mps)},
            {"control_index": 2, "coefficient": 1.0, "limit": 0.35},
            {"control_index": 2, "coefficient": -1.0, "limit": 0.35},
        ],
        "rows": [
            {
                "edge": {
                    "kind": row.edge.kind, "low": row.edge.low,
                    "high": row.edge.high, "base_id": row.edge.base_id,
                },
                "owner": row.owner,
                "name": f"v6-frozen-{index}",
                "coefficients": [float(row.coefficient[0]), float(row.coefficient[1]), 0.0],
                "constant": float(row.constant),
                "post_reset_barrier": 0.0,
                "snapshot_version": 1,
                "allocation_version": 1,
            }
            for index, row in enumerate(rows)
        ],
        "hard_problem_id": f"v6-frozen-next-uav-{robot_id}",
    }


def reconstruct_v6_one_step_state(
    family: Mapping,
    current_positions_m: Sequence[Sequence[float]],
    applied_planar_commands_mps: Sequence[Sequence[float]],
) -> V6OneStepState:
    """Purely reconstruct one state update; no verdict or execution occurs."""
    checked = validate_qualified_v6_initial_family(family)
    legacy = _legacy_v1_family(checked)
    positions = v1._normalize_positions(current_positions_m)
    component_max = checked["controller_policy"]["planar_component_max_mps"]
    commands = _normalize_commands(applied_planar_commands_mps, component_max)
    # Current-state reconstruction intentionally precedes the deterministic
    # SingleIntegrate2D update, even though its values are not a gate verdict.
    v1._audit_positions(legacy, positions, seed=None)
    dt_s = checked["one_step_gate"]["dt_s"]
    next_positions = tuple(
        (point[0] + dt_s * command[0], point[1] + dt_s * command[1])
        for point, command in zip(positions, commands)
    )
    certificates = v1._compute_certificates(legacy, next_positions)
    barriers, rows = v1._barriers_and_rows(legacy, next_positions, certificates)
    local_rows = tuple(
        tuple(row for row in rows if row.owner == robot_id)
        for robot_id in range(1, 15)
    )
    radii = tuple(
        solve_planar_hard_row_chebyshev(
            _local_hard_problem(robot_id, local_rows[robot_id - 1], component_max)
        ).radius_mps
        for robot_id in range(1, 15)
    )
    return V6OneStepState(
        chronology=(
            "validate-current-state", "apply-single-integrator",
            "recompute-next-certificates", "recompute-next-hard-rows",
            "solve-next-planar-radii",
        ),
        current_positions_m=positions,
        applied_planar_commands_mps=commands,
        next_positions_m=next_positions,
        next_certificates=certificates,
        next_barriers=barriers,
        next_endpoint_rows=rows,
        next_local_radii_mps=radii,
    )
