"""Exact-binary producer for the qualified-v6 one-step development gate.

The public entry point always consumes the frozen ordered 100-seed universe.
It creates an immutable claim before any child launch and publishes a separate
terminal artifact without replacement.  The artifact is only a one-step
development-admission result; it is not a long-horizon or recursive-feasibility
claim.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import stat
import subprocess
import sys
import tempfile
from contextlib import ExitStack
from pathlib import Path
from typing import Mapping, Protocol, Sequence

from scripts.diagnostics.qualified_closure_evidence import (
    audit_reset_primitives,
    reconstruct_controller_primitives,
    validate_controller_primitive_schema,
    validate_endpoint_primitive_schema,
    validate_initialization_schema,
    validate_mission_terminal_schema,
)
from scripts.diagnostics.qualified_config import validate_qualified_config
from scripts.diagnostics.hard_interior_selection import (
    frozen_interior_floor,
    solve_planar_hard_row_chebyshev,
)
from scripts.diagnostics import qualified_initial_state as v1_initial
from scripts.diagnostics import qualified_v6_initial_state as v6_initial
from scripts.diagnostics.qualified_v6_initial_state import (
    EXPECTED_AUDIT_SEEDS,
    EXPECTED_REGISTERED_SEEDS,
    materialize_v6_seed_positions,
    validate_qualified_v6_initial_family,
)


SCHEMA_VERSION = "cbf2026-qualified-v6-one-step-viability-v1"
CLAIM_SCHEMA_VERSION = f"{SCHEMA_VERSION}-claim"
FROZEN_SEEDS = tuple(range(2026080201, 2026080301))
REGISTERED_SEEDS = tuple(range(2026080201, 2026080211))
DT_S = 0.5
MINIMUM_NEXT_BARRIER_M = 0.0
MINIMUM_NEXT_LOCAL_RADIUS_MPS = 0.05
COMPONENT_MAX_MPS = 25.0
NUMERIC_TOLERANCE = 1e-7
CHILD_TIMEOUT_S = 30.0
CAMPAIGN_ID = "qualified-v6-one-step-development-gate-v1"
CONDITION = "dynamic_primary"


class OneStepOperations(Protocol):
    def launch_seed(self, *, seed: int, config_path: Path) -> Mapping: ...


class ChildLaunchFailure(RuntimeError):
    """Caught, terminal child execution failure."""

    def __init__(self, reason: str, detail: str = ""):
        self.reason = reason
        self.detail = detail
        super().__init__(reason if not detail else f"{reason}: {detail}")


def _canonical_json_bytes(value: object) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _reject_duplicate_keys(pairs: list[tuple[str, object]]) -> dict:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _read_json_object(path: Path, label: str) -> dict:
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"{label} must be a regular file")
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError(f"{label} is not valid UTF-8 JSON") from error
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object")
    return value


def _read_bound_json(path: Path, label: str) -> tuple[dict, dict]:
    """Parse and identify one immutable byte buffer from one file descriptor."""
    path = Path(path)
    if path.is_symlink():
        raise ValueError(f"{label} must be a regular file")
    flags = os.O_RDONLY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise ValueError(f"{label} must be a regular file") from error
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise ValueError(f"{label} must be a regular file")
        chunks: list[bytes] = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        after = os.fstat(descriptor)
        if (
            before.st_dev,
            before.st_ino,
            before.st_size,
            before.st_mtime_ns,
        ) != (
            after.st_dev,
            after.st_ino,
            after.st_size,
            after.st_mtime_ns,
        ):
            raise ValueError(f"{label} changed during its bound read")
    finally:
        os.close(descriptor)
    raw = b"".join(chunks)
    try:
        value = json.loads(
            raw.decode("utf-8"), object_pairs_hook=_reject_duplicate_keys
        )
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ValueError(f"{label} is not valid UTF-8 JSON") from error
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object")
    return value, {
        "path": str(path.resolve()),
        "bytes": len(raw),
        "sha256": hashlib.sha256(raw).hexdigest(),
    }


def _regular_file_identity(path: Path, label: str) -> dict:
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"{label} must be a regular file")
    resolved = path.resolve()
    return {
        "path": str(resolved),
        "bytes": resolved.stat().st_size,
        "sha256": _sha256(resolved),
    }


def _repository_identity(project_root: Path) -> dict:
    root = Path(project_root).resolve()

    def git(*arguments: str) -> str:
        completed = subprocess.run(
            ["git", *arguments],
            cwd=root,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        if completed.returncode != 0:
            raise ValueError(
                "Git identity unavailable: " + completed.stderr.strip()
            )
        return completed.stdout.strip()

    if Path(git("rev-parse", "--show-toplevel")).resolve() != root:
        raise ValueError("project root must equal the Git worktree root")
    return {
        "root": str(root),
        "head": git("rev-parse", "HEAD"),
        "tree": git("rev-parse", "HEAD^{tree}"),
    }


def _collect_bound_identities(
    *,
    binary: Path,
    base_config: Path,
    primary_config: Path,
    initial_family: Path,
    project_root: Path,
) -> dict:
    return {
        "repository": _repository_identity(project_root),
        "implementation": _regular_file_identity(
            Path(__file__), "one-step implementation"
        ),
        "binary": _regular_file_identity(binary, "Swarm binary"),
        "base_config": _regular_file_identity(base_config, "base config"),
        "primary_config": _regular_file_identity(
            primary_config, "primary config"
        ),
        "initial_family": _regular_file_identity(
            initial_family, "initial family"
        ),
    }


def _require_publication_paths_absent(claim: Path, output: Path) -> None:
    claim = Path(claim)
    output = Path(output)
    if claim.absolute() == output.absolute():
        raise ValueError("claim and output must be distinct paths")
    for path, label in ((claim, "claim"), (output, "output")):
        if path.exists() or path.is_symlink():
            raise FileExistsError(f"{label} path must be absent: {path}")
        parent = path.parent
        if parent.is_symlink() or not parent.is_dir():
            raise ValueError(f"{label} parent must be an existing directory")


def _write_json_no_replace(path: Path, payload: Mapping) -> None:
    """Write the final path directly with O_EXCL, then fsync file and parent."""
    path = Path(path)
    encoded = _canonical_json_bytes(payload) + b"\n"
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(path, flags, 0o600)
    try:
        offset = 0
        while offset != len(encoded):
            written = os.write(descriptor, encoded[offset:])
            if written <= 0:
                raise OSError("short no-replace JSON write")
            offset += written
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    parent_flags = os.O_RDONLY
    if hasattr(os, "O_DIRECTORY"):
        parent_flags |= os.O_DIRECTORY
    parent_descriptor = os.open(path.parent, parent_flags)
    try:
        os.fsync(parent_descriptor)
    finally:
        os.close(parent_descriptor)


def _deep_merge(base: Mapping, overlay: Mapping) -> dict:
    merged = copy.deepcopy(dict(base))
    for key, value in overlay.items():
        if isinstance(value, Mapping) and isinstance(merged.get(key), Mapping):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _materialize_seed_config(
    *,
    base: Mapping,
    primary: Mapping,
    family: Mapping,
    seed: int,
    directory: Path,
) -> tuple[Path, tuple[tuple[float, float], ...]]:
    positions = materialize_v6_seed_positions(family, seed)
    config = _deep_merge(base, primary)
    config.setdefault("initial", {})["position"] = {
        "method": "specified",
        "positions": [list(position) for position in positions],
    }
    execute = config.setdefault("execute", {})
    execute["random-seed"] = seed
    execute["time-total"] = 0.5
    execute["time-step"] = 0.5
    execute["execution-mode"] = "distributed"
    config["output_path"] = str(directory)
    config["run_suffix"] = f"_qualified-v6-one-step-{seed}"
    config["evidence-stream"] = {
        "enabled": True,
        "schema-version": "cbf2026-qualified-evidence-v1",
        "campaign-id": CAMPAIGN_ID,
        "trajectory-seed": seed,
        "range-noise-seed": 0,
        "condition": CONDITION,
    }
    for forbidden in ("campaign_root", "campaign-root", "campaign_root_path"):
        config.pop(forbidden, None)
    path = directory / "config.json"
    _write_json_no_replace(path, config)
    return path, positions


def _strict_number(value: object, label: str) -> float:
    if type(value) not in (int, float) or isinstance(value, bool):
        raise ValueError(f"{label} must be finite numeric data")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite numeric data")
    return number


def _evaluate_predicate(
    *,
    commands: Sequence[Sequence[float]],
    residual_floor_pairs: Sequence[tuple[float, float]],
    next_barriers: Sequence[float],
    next_radii: Sequence[float],
) -> tuple[bool, tuple[str, ...]]:
    reasons: list[str] = []
    if any(
        abs(_strict_number(component, "applied command"))
        > COMPONENT_MAX_MPS + NUMERIC_TOLERANCE
        for command in commands
        for component in command[:2]
    ):
        reasons.append("component_bound_violation")
    if any(
        residual < floor - NUMERIC_TOLERANCE
        for residual, floor in residual_floor_pairs
    ):
        reasons.append("applied_residual_below_floor")
    if any(barrier <= MINIMUM_NEXT_BARRIER_M for barrier in next_barriers):
        reasons.append("nonpositive_next_barrier")
    if any(radius < MINIMUM_NEXT_LOCAL_RADIUS_MPS for radius in next_radii):
        reasons.append("small_next_local_radius")
    return not reasons, tuple(reasons)


def _reconstruct_next_metrics(
    family: Mapping,
    positions: tuple[tuple[float, float], ...],
    commands: Sequence[Sequence[float]],
):
    """Use Task 5's exact chronology without imposing its zero-tolerance box."""
    checked = validate_qualified_v6_initial_family(family)
    legacy = v6_initial._legacy_v1_family(checked)
    current_positions = v1_initial._normalize_positions(positions)
    current_audit = v1_initial._audit_positions(
        legacy, current_positions, seed=None
    )
    if not current_audit.accepted:
        raise v1_initial.InitialStateAdmissionError(current_audit)
    next_positions = tuple(
        (
            point[0] + DT_S * float(command[0]),
            point[1] + DT_S * float(command[1]),
        )
        for point, command in zip(current_positions, commands)
    )
    certificates = v1_initial._compute_certificates(legacy, next_positions)
    barriers, rows = v1_initial._barriers_and_rows(
        legacy, next_positions, certificates
    )
    local_rows = tuple(
        tuple(row for row in rows if row.owner == robot_id)
        for robot_id in range(1, 15)
    )
    radii = tuple(
        solve_planar_hard_row_chebyshev(
            v6_initial._local_hard_problem(
                robot_id,
                local_rows[robot_id - 1],
                checked["controller_policy"]["planar_component_max_mps"],
            )
        ).radius_mps
        for robot_id in range(1, 15)
    )
    return current_audit, barriers, radii


def _validate_operation_result(
    result: Mapping,
    *,
    seed: int,
    config_path: Path,
    expected_positions: tuple[tuple[float, float], ...],
    family: Mapping,
) -> dict:
    if not isinstance(result, Mapping) or set(result) != {
        "trajectory_seed",
        "config_sha256",
        "attempt_count",
        "retry_count",
        "frame_zero_records",
    }:
        raise ValueError("seed operation result has an invalid exact schema")
    if result["trajectory_seed"] != seed:
        raise ValueError("seed operation returned the wrong seed")
    observed_config_sha = _sha256(config_path)
    if result["config_sha256"] != observed_config_sha:
        raise ValueError("seed operation returned an altered config hash")
    if result["attempt_count"] != 1 or result["retry_count"] != 0:
        raise ValueError("seed operation attempted a retry")
    records = result["frame_zero_records"]
    if not isinstance(records, list) or len(records) != 1:
        raise ValueError("seed operation must return exactly one frame-zero record")
    frame = records[0]
    if not isinstance(frame, Mapping) or set(frame) != {
        "trajectory_seed",
        "frame_index",
        "complete",
        "current_positions_m",
        "nodes",
    }:
        raise ValueError("frame-zero record has an invalid exact schema")
    if (
        frame["trajectory_seed"] != seed
        or frame["frame_index"] != 0
        or frame["complete"] is not True
    ):
        raise ValueError("frame-zero identity is missing, wrong, or incomplete")
    positions = tuple(
        tuple(_strict_number(value, "current position") for value in position)
        for position in frame["current_positions_m"]
    )
    if positions != expected_positions:
        raise ValueError("frame-zero positions differ from the materialized seed")
    nodes = frame["nodes"]
    if not isinstance(nodes, list) or len(nodes) != 14:
        raise ValueError("frame-zero must contain exactly 14 nodes")
    ordered = sorted(nodes, key=lambda node: node.get("robot_id", -1))
    if [node.get("robot_id") for node in ordered] != list(range(1, 15)):
        raise ValueError("frame-zero node identity differs")
    commands: list[tuple[float, float, float]] = []
    residual_floor_pairs: list[tuple[float, float]] = []
    for node in ordered:
        if not isinstance(node, Mapping) or set(node) != {
            "robot_id",
            "applied_command",
            "normal_problem",
            "hard_interior_selection",
        }:
            raise ValueError("frame-zero node has an invalid exact schema")
        command_raw = node["applied_command"]
        if not isinstance(command_raw, list) or len(command_raw) != 3:
            raise ValueError("applied command must have exactly three components")
        command = tuple(
            _strict_number(value, "applied command") for value in command_raw
        )
        commands.append(command)
        problem = node["normal_problem"]
        policy = node["hard_interior_selection"]
        if not isinstance(problem, Mapping) or not isinstance(problem.get("rows"), list):
            raise ValueError("normal hard problem rows are missing")
        if not isinstance(policy, Mapping) or set(policy) != {
            "mode",
            "fraction",
            "cap_mps",
            "feasibility_tolerance_mps",
            "planar_chebyshev_radius_mps",
            "enforced_floor_mps",
            "minimum_original_hard_residual_mps",
        }:
            raise ValueError("hard-interior selection evidence is malformed")
        if policy["mode"] != "planar-chebyshev-fraction-cap-v1":
            raise ValueError("hard-interior selection mode differs")
        fraction = _strict_number(policy["fraction"], "interior fraction")
        cap = _strict_number(policy["cap_mps"], "interior cap")
        tolerance = _strict_number(
            policy["feasibility_tolerance_mps"], "interior tolerance"
        )
        if (fraction, cap, tolerance) != (0.1, 0.1, 1e-9):
            raise ValueError("hard-interior selection constants differ")
        independent_interior = solve_planar_hard_row_chebyshev(
            problem, tolerance_mps=tolerance
        )
        expected_floor = frozen_interior_floor(
            independent_interior.radius_mps,
            fraction=fraction,
            cap_mps=cap,
            tolerance_mps=tolerance,
        )
        published_radius = _strict_number(
            policy["planar_chebyshev_radius_mps"], "published interior radius"
        )
        floor = _strict_number(policy["enforced_floor_mps"], "enforced floor")
        if not math.isclose(
            published_radius,
            independent_interior.radius_mps,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError("published interior radius differs from reconstruction")
        if not math.isclose(
            floor, expected_floor, rel_tol=0.0, abs_tol=1e-12
        ):
            raise ValueError("published enforced floor differs from reconstruction")
        if not problem["rows"]:
            raise ValueError("normal hard problem contains no hard rows")
        node_residuals: list[float] = []
        for row in problem["rows"]:
            if not isinstance(row, Mapping) or not {
                "coefficients", "constant"
            } <= set(row):
                raise ValueError("normal hard row is malformed")
            coefficients = row["coefficients"]
            if not isinstance(coefficients, list) or len(coefficients) != 3:
                raise ValueError("normal hard row coefficient count differs")
            residual = _strict_number(row["constant"], "hard row constant") + sum(
                _strict_number(coefficient, "hard row coefficient") * component
                for coefficient, component in zip(coefficients, command)
            )
            node_residuals.append(residual)
            residual_floor_pairs.append((residual, floor))
        published_minimum = _strict_number(
            policy["minimum_original_hard_residual_mps"],
            "published minimum original hard residual",
        )
        if not math.isclose(
            published_minimum,
            min(node_residuals),
            rel_tol=1e-9,
            abs_tol=1e-9,
        ):
            raise ValueError(
                "published minimum original hard residual differs from reconstruction"
            )

    current_audit, next_barriers, next_radii = _reconstruct_next_metrics(
        family, positions, commands
    )
    if len(next_barriers) != 119 or len(next_radii) != 14:
        raise ValueError("independent one-step reconstruction cardinality differs")
    barriers = tuple(float(item.value) for item in next_barriers)
    radii = tuple(float(value) for value in next_radii)
    passed, reasons = _evaluate_predicate(
        commands=commands,
        residual_floor_pairs=residual_floor_pairs,
        next_barriers=barriers,
        next_radii=radii,
    )
    return {
        "seed": seed,
        "launched": True,
        "status": "passed" if passed else "failed",
        "passed": passed,
        "reasons": list(reasons),
        "config_sha256": observed_config_sha,
        "positions_sha256": current_audit.positions_sha256,
        "minimum_applied_original_residual_mps": min(
            residual for residual, _floor in residual_floor_pairs
        ),
        "minimum_enforced_floor_mps": min(
            floor for _residual, floor in residual_floor_pairs
        ),
        "maximum_planar_component_mps": max(
            abs(component) for command in commands for component in command[:2]
        ),
        "minimum_next_barrier_m": min(barriers),
        "minimum_next_local_radius_mps": min(radii),
        "barrier_count": len(barriers),
        "local_radius_count": len(radii),
    }


def _summary(seed_results: Sequence[Mapping], seeds: Sequence[int]) -> dict:
    selected = [row for row in seed_results if row["seed"] in seeds]
    launched = [row for row in selected if row.get("launched") is True]
    passed = [row for row in launched if row.get("passed") is True]
    finite_barriers = [
        row["minimum_next_barrier_m"]
        for row in launched
        if "minimum_next_barrier_m" in row
    ]
    finite_radii = [
        row["minimum_next_local_radius_mps"]
        for row in launched
        if "minimum_next_local_radius_mps" in row
    ]
    return {
        "proposed_count": len(selected),
        "launched_count": len(launched),
        "passed_count": len(passed),
        "minimum_next_barrier_m": min(finite_barriers) if finite_barriers else None,
        "minimum_next_local_radius_mps": min(finite_radii) if finite_radii else None,
    }


def _temporary_parent(claim: Path) -> Path:
    temporary_root = Path("/private/tmp").resolve()
    parent = Path(claim).parent.resolve()
    try:
        parent.relative_to(temporary_root)
    except ValueError:
        return temporary_root
    return parent


def _audit_seed_sequence(
    *,
    binary: Path,
    base_config: Path,
    primary_config: Path,
    initial_family: Path,
    claim: Path,
    output: Path,
    project_root: Path,
    seed_universe: Sequence[int],
    operations: OneStepOperations | None = None,
) -> dict:
    claim = Path(claim)
    output = Path(output)
    project_root = Path(project_root).resolve()
    seeds = tuple(seed_universe)
    if (
        not seeds
        or any(type(seed) is not int or seed not in FROZEN_SEEDS for seed in seeds)
        or len(set(seeds)) != len(seeds)
        or tuple(sorted(seeds)) != seeds
    ):
        raise ValueError("seed universe is not an ordered unique frozen subset")
    _require_publication_paths_absent(claim, output)
    base, base_buffer_identity = _read_bound_json(base_config, "base config")
    primary, primary_buffer_identity = _read_bound_json(
        primary_config, "primary config"
    )
    if not validate_qualified_config(primary):
        raise ValueError("primary config fails exact qualified-v2 validation")
    if primary["position_covariance"]["reference-selection"] != "dynamic-lower-index":
        raise ValueError("one-step gate requires the dynamic primary overlay")
    family_value, family_buffer_identity = _read_bound_json(
        initial_family, "initial family"
    )
    family = validate_qualified_v6_initial_family(family_value)
    if EXPECTED_AUDIT_SEEDS != FROZEN_SEEDS:
        raise ValueError("Task 5 audit universe differs from the gate")
    if EXPECTED_REGISTERED_SEEDS != REGISTERED_SEEDS:
        raise ValueError("Task 5 registered universe differs from the gate")
    observed_identities = _collect_bound_identities(
        binary=binary,
        base_config=base_config,
        primary_config=primary_config,
        initial_family=initial_family,
        project_root=project_root,
    )
    identities = copy.deepcopy(observed_identities)
    identities["base_config"] = base_buffer_identity
    identities["primary_config"] = primary_buffer_identity
    identities["initial_family"] = family_buffer_identity
    if observed_identities != identities:
        raise ValueError("input changed before claim publication")
    canonical_inputs = {
        "base_config": project_root / "config/config.json",
        "primary_config": project_root
        / "config/diagnostics/qualified_mode_hybrid_dcbf_development_v2.json",
        "initial_family": project_root
        / "config/diagnostics/qualified_initial_family_v2.json",
    }
    for label, canonical_path in canonical_inputs.items():
        canonical = _regular_file_identity(canonical_path, f"canonical {label}")
        if (
            identities[label]["bytes"],
            identities[label]["sha256"],
        ) != (canonical["bytes"], canonical["sha256"]):
            raise ValueError(f"{label} differs from the exact canonical bytes")
    claim_payload = {
        "schema_version": CLAIM_SCHEMA_VERSION,
        "claimed": True,
        "source": identities["repository"],
        "bindings": {key: identities[key] for key in (
            "implementation", "binary", "base_config", "primary_config", "initial_family"
        )},
        "seed_universe": list(seeds),
        "gate": {
            "dt_s": 0.5,
            "minimum_next_barrier_m": 0.0,
            "barrier_comparison": "strictly-greater",
            "minimum_next_local_radius_mps": 0.05,
            "component_max_mps": 25.0,
            "numeric_tolerance": 1e-7,
            "clamp": False,
            "resample": False,
            "retry": False,
        },
        "boundary": (
            "one-step development admission only; not long-horizon safety "
            "or recursive feasibility"
        ),
    }
    _write_json_no_replace(claim, claim_payload)
    claimed_sha256 = _sha256(claim)
    launch_count = 0
    retry_count = 0
    results: list[dict] = []
    failure_reason: str | None = None
    real_operations = operations or SubprocessOneStepOperations(
        binary=binary, project_root=project_root
    )

    with ExitStack() as temporary_material:
        try:
            if _collect_bound_identities(
                binary=binary,
                base_config=base_config,
                primary_config=primary_config,
                initial_family=initial_family,
                project_root=project_root,
            ) != identities:
                raise ValueError("bound identity changed after claim publication")
            for seed in seeds:
                temporary = Path(
                    temporary_material.enter_context(
                        tempfile.TemporaryDirectory(
                            prefix=f"qualified-v6-one-step-{seed}-",
                            dir=_temporary_parent(claim),
                        )
                    )
                )
                config_path, positions = _materialize_seed_config(
                    base=base,
                    primary=primary,
                    family=family,
                    seed=seed,
                    directory=temporary,
                )
                launch_count += 1
                operation_result = real_operations.launch_seed(
                    seed=seed, config_path=config_path
                )
                if isinstance(operation_result, Mapping):
                    observed_retry = operation_result.get("retry_count", 0)
                    if type(observed_retry) is int and observed_retry > 0:
                        retry_count += observed_retry
                results.append(
                    _validate_operation_result(
                        operation_result,
                        seed=seed,
                        config_path=config_path,
                        expected_positions=positions,
                        family=family,
                    )
                )
                if _sha256(claim) != claimed_sha256:
                    raise ValueError("immutable claim identity changed")
                if _collect_bound_identities(
                    binary=binary,
                    base_config=base_config,
                    primary_config=primary_config,
                    initial_family=initial_family,
                    project_root=project_root,
                ) != identities:
                    raise ValueError("bound identity changed during execution")
        except Exception as error:
            failure_reason = (
                error.reason if isinstance(error, ChildLaunchFailure)
                else f"{type(error).__name__}: {error}"
            )

        completed_seeds = {row["seed"] for row in results}
        for seed in seeds:
            if seed not in completed_seeds:
                results.append({
                    "seed": seed,
                    "launched": seed in seeds[:launch_count],
                    "status": "failed" if seed in seeds[:launch_count] else "not_launched",
                    "passed": False,
                    "reasons": [failure_reason or "terminal_failure"],
                })
        results.sort(key=lambda row: row["seed"])
        all_passed = failure_reason is None and all(row["passed"] for row in results)
        audit_summary = _summary(results, seeds)
        registered_summary = _summary(results, tuple(
            seed for seed in seeds if seed in REGISTERED_SEEDS
        ))
        observed_claim_sha = _sha256(claim)
        if observed_claim_sha != claimed_sha256:
            raise ValueError("immutable claim identity changed before terminal publication")
        terminal_identities = _collect_bound_identities(
            binary=binary,
            base_config=base_config,
            primary_config=primary_config,
            initial_family=initial_family,
            project_root=project_root,
        )
        if terminal_identities != identities:
            raise ValueError("bound identity changed before terminal publication")
        output_payload = {
            "schema_version": SCHEMA_VERSION,
            "terminal": True,
            "status": "completed" if all_passed else "failed",
            "passed": all_passed,
            "reason": "completed" if all_passed else (failure_reason or "predicate_failed"),
            "claim_sha256": claimed_sha256,
            "claimed_identity_sha256": claimed_sha256,
            "source": identities["repository"],
            "terminal_source": terminal_identities["repository"],
            "bindings": {key: identities[key] for key in (
                "implementation", "binary", "base_config", "primary_config", "initial_family"
            )},
            "seed_universe": list(seeds),
            "seed_results": results,
            "audit_summary": audit_summary,
            "registered_summary": registered_summary,
            "launch_count": launch_count,
            "retry_count": retry_count,
            "boundary": claim_payload["boundary"],
        }
        _write_json_no_replace(output, output_payload)
        if _sha256(claim) != output_payload["claim_sha256"]:
            raise ValueError("terminal output does not cross-bind the exact claim")
        observed_output = _read_json_object(output, "terminal output")
        if observed_output != output_payload:
            raise ValueError("terminal output publication is partial or altered")
    return output_payload


def audit_one_step_universe(
    *,
    binary: Path,
    base_config: Path,
    primary_config: Path,
    initial_family: Path,
    claim: Path,
    output: Path,
    project_root: Path,
    operations: OneStepOperations | None = None,
) -> dict:
    return _audit_seed_sequence(
        binary=binary,
        base_config=base_config,
        primary_config=primary_config,
        initial_family=initial_family,
        claim=claim,
        output=output,
        project_root=project_root,
        seed_universe=FROZEN_SEEDS,
        operations=operations,
    )


class SubprocessOneStepOperations:
    """Concrete exact-binary boundary: one subprocess invocation per call."""

    def __init__(self, *, binary: Path, project_root: Path, timeout_s: float = CHILD_TIMEOUT_S):
        self.binary = Path(binary)
        self.project_root = Path(project_root)
        self.timeout_s = timeout_s

    def launch_seed(self, *, seed: int, config_path: Path) -> Mapping:
        try:
            completed = subprocess.run(
                [str(self.binary), str(config_path)],
                cwd=self.project_root,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=self.timeout_s,
                check=False,
            )
        except subprocess.TimeoutExpired as error:
            raise ChildLaunchFailure("child_timeout", str(error)) from error
        if completed.returncode != 0:
            raise ChildLaunchFailure(
                "child_nonzero_exit",
                f"returncode={completed.returncode}; stderr={completed.stderr[-4096:]}",
            )
        records = []
        for line_number, line in enumerate(completed.stdout.splitlines(), start=1):
            if not line:
                continue
            try:
                row = json.loads(line, object_pairs_hook=_reject_duplicate_keys)
            except (json.JSONDecodeError, ValueError) as error:
                raise ChildLaunchFailure(
                    "malformed_child_evidence", f"line={line_number}"
                ) from error
            if not isinstance(row, dict):
                raise ChildLaunchFailure(
                    "malformed_child_evidence", f"line={line_number}"
                )
            records.append(row)
        allowed_types = {
            "initialization",
            "reset",
            "endpoint_row",
            "controller_interval",
            "mission_terminal",
        }
        if any(row.get("record_type") not in allowed_types for row in records):
            raise ChildLaunchFailure("extra_or_unknown_frame_zero_evidence")
        for row in records:
            if (
                row.get("campaign_id") != CAMPAIGN_ID
                or row.get("condition") != CONDITION
                or row.get("trajectory_seed") != seed
                or row.get("range_noise_seed") != 0
            ):
                raise ChildLaunchFailure("wrong_frame_zero_evidence_identity")
        initializations = [
            row for row in records if row.get("record_type") == "initialization"
        ]
        resets = [row for row in records if row.get("record_type") == "reset"]
        terminals = [
            row for row in records if row.get("record_type") == "mission_terminal"
        ]
        if (
            len(initializations) != 14
            or sorted(row.get("robot_id") for row in initializations)
            != list(range(1, 15))
            or not all(validate_initialization_schema(row) for row in initializations)
            or len(resets) != 1
            or audit_reset_primitives(resets[0], tuple(range(1, 15)), 119)
            or len(terminals) != 1
            or not validate_mission_terminal_schema(terminals[0])
            or terminals[0].get("frame_index") != 1
            or terminals[0].get("runtime", {}).get("success") is not True
            or terminals[0].get("runtime", {}).get("declared_frames") != 1
            or terminals[0].get("runtime", {}).get("completed_intervals") != 1
        ):
            raise ChildLaunchFailure("missing_or_malformed_frame_zero_envelope")
        controllers = [
            row for row in records if row.get("record_type") == "controller_interval"
        ]
        if len(controllers) != 1:
            raise ChildLaunchFailure("multiple_or_missing_frame_zero")
        controller = controllers[0]
        if (
            controller.get("trajectory_seed") != seed
            or controller.get("frame_index") != 0
            or controller.get("campaign_id") != CAMPAIGN_ID
            or controller.get("condition") != CONDITION
            or controller.get("range_noise_seed") != 0
            or controller.get("runtime", {}).get("complete") is not True
            or not validate_controller_primitive_schema(controller)
        ):
            raise ChildLaunchFailure("wrong_or_malformed_frame_zero")
        endpoints = [
            row for row in records if row.get("record_type") == "endpoint_row"
        ]
        if len(endpoints) != 232 or not all(
            row.get("trajectory_seed") == seed
            and row.get("frame_index") == 0
            and row.get("campaign_id") == CAMPAIGN_ID
            and row.get("condition") == CONDITION
            and row.get("range_noise_seed") == 0
            and validate_endpoint_primitive_schema(row)
            for row in endpoints
        ):
            raise ChildLaunchFailure("wrong_or_malformed_frame_zero_endpoints")
        reconstruction = reconstruct_controller_primitives(
            controller, endpoints, 232, 119
        )
        if reconstruction.integrity_errors:
            raise ChildLaunchFailure(
                "frame_zero_reconstruction_failed",
                ",".join(reconstruction.integrity_errors[:16]),
            )
        truth = sorted(
            controller["analyzer_only"]["truth"],
            key=lambda row: row["robot_id"],
        )
        nodes = []
        for node in sorted(
            controller["runtime"]["nodes"], key=lambda row: row["robot_id"]
        ):
            policy = node.get("hard_interior_selection")
            if not isinstance(policy, dict):
                raise ChildLaunchFailure("missing_hard_interior_evidence")
            nodes.append({
                "robot_id": node["robot_id"],
                "applied_command": node["applied_command"],
                "normal_problem": copy.deepcopy(node["normal_problem"]),
                "hard_interior_selection": copy.deepcopy(policy),
            })
        return {
            "trajectory_seed": seed,
            "config_sha256": _sha256(config_path),
            "attempt_count": 1,
            "retry_count": 0,
            "frame_zero_records": [{
                "trajectory_seed": seed,
                "frame_index": 0,
                "complete": True,
                "current_positions_m": [row["position"] for row in truth],
                "nodes": nodes,
            }],
        }


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the immutable qualified-v6 100-seed one-step gate"
    )
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--base-config", type=Path, required=True)
    parser.add_argument("--primary-config", type=Path, required=True)
    parser.add_argument("--initial-family", type=Path, required=True)
    parser.add_argument("--claim", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--project-root", type=Path, default=Path.cwd())
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = _argument_parser().parse_args(argv)
    try:
        result = audit_one_step_universe(
            binary=arguments.binary,
            base_config=arguments.base_config,
            primary_config=arguments.primary_config,
            initial_family=arguments.initial_family,
            claim=arguments.claim,
            output=arguments.output,
            project_root=arguments.project_root,
        )
    except Exception as error:
        print(f"qualified-v6 one-step gate failed: {error}", file=sys.stderr)
        return 2
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
