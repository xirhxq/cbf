"""Exact-output Stage-1 replay for the bounded predictive WNLS estimator."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
import os
import re
import stat
import tempfile
from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from scripts.diagnostics.predictive_wnls import (
    ATTEMPT_STATUSES,
    CANDIDATE_DEDUP_M,
    FRAME_DT_SECONDS,
    INITIAL_WNLS_DAMPING as INITIAL_DAMPING,
    INNOVATION_REFERENCE_QUANTILE as INNOVATION_Q_MAX,
    MAX_PUBLIC_PREDICTION_AGE as MAX_PREDICTION_AGE_FRAMES,
    MAX_WNLS_DAMPING as MAX_DAMPING,
    MAX_WNLS_PROPOSALS as MAX_PROPOSALS_PER_CANDIDATE,
    MIN_WNLS_DAMPING as MIN_DAMPING,
    OUTPUT_STATUSES,
    RELATIVE_SPECTRAL_THRESHOLD,
    RELATIVE_TIE_TOLERANCE,
    REPRESENTABLE_STEP_SCALE as REPRESENTABLE_STEP_RELATIVE_THRESHOLD,
    STATIONARITY_SCALE as SCALE_AWARE_STATIONARITY,
    WNLS_DAMPING_FACTOR as DAMPING_FACTOR,
    qualify_active_references,
    reference_is_eligible,
    solve_predictive_multistart,
)
from scripts.diagnostics.replay_localization_calibration import (
    RESTART_BEFORE_FIRST_FINITE_POLICY,
    _frames,
    _initial_positions,
    _truth_positions,
    _valid_prior_result,
    active_references,
    fixed_references,
    select_initial_estimate,
    solve_later_frame,
    solve_wnls,
    stable_measurement_seed,
)
from scripts.diagnostics.run_diagnostic import (
    HARD_FLOOR_BYTES,
    START_BYTES,
    DiskSpaceError,
    _nearest_existing_ancestor,
    _sha256,
    allocated_bytes,
    available_bytes,
    require_start_space,
)


DEVELOPMENT_VARIANTS = (
    "prediction_expiry",
    "fresh_reference_qualification",
    "predictive_multistart",
)
PROTOCOL_SCHEMA_ID = "cbf2026-predictive-wnls-stage1-protocol-v1"
PROTOCOL_ID = "cbf2026-predictive-wnls-stage1-v2"
RAW_SCHEMA_ID = "cbf2026-predictive-wnls-development-rows-v2"
RAW_PROCESS_NAME = "predictive-wnls-development.jsonl.gz"
TERMINAL_MANIFEST_NAME = "manifest.json"
RAW_BUNDLE_CAP_BYTES = 2_000_000_000
MOTION_SIGMA_M_PER_FRAME = 0.5
LEGACY_SOLVER_SHA256 = (
    "0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8"
)

CANDIDATE_FIELDS = (
    "source",
    "initial_estimate",
    "status",
    "estimate",
    "covariance",
    "cost",
    "accepted",
    "rejection_reason",
    "q_innov",
    "gate_diagnostics",
    "proposal_trace",
)
PROPOSAL_TRACE_FIELDS = (
    "proposal",
    "damping",
    "cost",
    "stationarity_norm",
    "raw_step_norm",
    "trial_cost",
    "invalid_trial_reason",
    "accepted",
)
GATE_DIAGNOSTIC_FIELDS = (
    "innovation_gate",
    "q_innov",
    "gate_outcome",
    "valid",
    "failure_reason",
    "reduced_whitened_cost",
)
REFERENCE_EVIDENCE_FIELDS = (
    "reference_kind",
    "reference_id",
    "role",
    "measurement_present",
    "noisy_range",
    "noise_seed",
    "current_freshness",
    "eligible",
    "used",
    "exclusion_reason",
    "base_anchor_provenance",
)
ROW_FIELDS = (
    "variant",
    "seed",
    "frame_index",
    "robot_id",
    "squad_local_index",
    "applied_command_source_frame",
    "applied_command",
    "legacy_numeric_status",
    "legacy_initial_estimate_source",
    "attempt_status",
    "attempt_failure_reason",
    "output_status",
    "prediction_age",
    "estimate",
    "fresh_modeled_covariance",
    "fresh_epsilon",
    "aged_modeled_covariance",
    "aged_modeled_radius",
    "private_reacquisition_seed",
    "attempt_base_anchor_provenance",
    "base_anchor_provenance",
    "mandatory_references",
    "optional_candidates",
    "active_references",
    "reference_evidence",
    "excluded_references",
    "reference_violations",
    "candidates",
    "selected_candidate_source",
    "offline_truth_position",
    "offline_error_norm",
    "offline_fresh_containment",
    "offline_aged_radius_containment",
    "offline_fresh_q_error",
    "offline_aged_q_error",
)

BINDING_DESIGN = {
    "path": (
        "docs/superpowers/specs/"
        "2026-07-30-cbf2026-bounded-predictive-wnls-recovery-design.md"
    ),
    "commit": "a2ae1f0",
    "sha256": "f3820b8c912ed1a019334f0e7e273a1bcf3d3b1f4ec32cb059f5aaef597c152d",
}
EXPERIMENT_CONTRACT = {
    "stage": 1,
    "evidence_class": "paired_single_trajectory_development_only",
    "variants": list(DEVELOPMENT_VARIANTS),
    "frame_dt_seconds": 0.5,
    "ranging_sigma_m": 0.5,
    "measurement_seed_contract": "cbf2026-range-v1",
    "execution_order": [
        "variant",
        "seed",
        "frame_index",
        "ascending_global_robot_id",
    ],
    "baseline_graph_case": "dynamic_dag_wnls",
    "baseline_initialization_policy": RESTART_BEFORE_FIRST_FINITE_POLICY,
}
ESTIMATOR_CONSTANTS = {
    "max_prediction_age_frames": MAX_PREDICTION_AGE_FRAMES,
    "motion_sigma_m_per_frame": MOTION_SIGMA_M_PER_FRAME,
    "innovation_q_max": INNOVATION_Q_MAX,
    "reacquisition_reduced_cost_max": 9.0,
    "catastrophic_error_m": 50.0,
    "max_proposals_per_candidate": MAX_PROPOSALS_PER_CANDIDATE,
    "initial_damping": INITIAL_DAMPING,
    "minimum_damping": MIN_DAMPING,
    "maximum_damping": MAX_DAMPING,
    "damping_factor": DAMPING_FACTOR,
    "scale_aware_stationarity": SCALE_AWARE_STATIONARITY,
    "relative_spectral_threshold": RELATIVE_SPECTRAL_THRESHOLD,
    "representable_step_relative_threshold": (
        REPRESENTABLE_STEP_RELATIVE_THRESHOLD
    ),
    "candidate_dedup_m": CANDIDATE_DEDUP_M,
    "relative_tie_tolerance": RELATIVE_TIE_TOLERANCE,
}
STATUS_CONTRACT = {
    "output_statuses": list(OUTPUT_STATUSES),
    "attempt_statuses": list(ATTEMPT_STATUSES),
}
ABLATION_CONTRACTS = {
    "prediction_expiry": {
        "numeric_solver": "legacy_solve_wnls_v3_exact",
        "initialization_policy": RESTART_BEFORE_FIRST_FINITE_POLICY,
        "reference_policy": (
            "legacy_dynamic_dag_include_all_visible_with_retained_numeric_state"
        ),
        "online_acceptance": "legacy_converged_only",
        "publication_policy": "command_prediction_age_1_2_then_unavailable",
        "fresh_reference_qualification": False,
        "two_base_provenance_gate": False,
        "reacquisition_gate": False,
        "normalized_innovation_gate": False,
        "multistart": False,
        "paper_evidence_eligible": False,
    },
    "fresh_reference_qualification": {
        "numeric_solver": "legacy_solve_wnls_v3_exact",
        "initialization_policy": RESTART_BEFORE_FIRST_FINITE_POLICY,
        "reference_policy": (
            "sensor_present_mandatory_plus_all_visible_current_fresh_lower_index"
        ),
        "online_acceptance": (
            "legacy_converged_plus_fim_provenance_and_reacquisition"
        ),
        "publication_policy": "command_prediction_age_1_2_then_unavailable",
        "fresh_reference_qualification": True,
        "two_base_provenance_gate": True,
        "reacquisition_gate": True,
        "normalized_innovation_gate": False,
        "multistart": False,
        "paper_evidence_eligible": False,
    },
    "predictive_multistart": {
        "numeric_solver": "finite_budget_scale_aware_wnls_v1",
        "initialization_policy": (
            "prediction_or_private_seed_then_algebraic_and_two_circle"
        ),
        "reference_policy": (
            "sensor_present_mandatory_plus_all_visible_current_fresh_lower_index"
        ),
        "online_acceptance": (
            "finite_solver_fim_provenance_and_innovation_or_reacquisition"
        ),
        "publication_policy": "command_prediction_age_1_2_then_unavailable",
        "fresh_reference_qualification": True,
        "two_base_provenance_gate": True,
        "reacquisition_gate": True,
        "normalized_innovation_gate": True,
        "multistart": True,
        "paper_evidence_eligible": False,
    },
}
RAW_SCHEMA_DECLARATION = {
    "id": RAW_SCHEMA_ID,
    "process_name": RAW_PROCESS_NAME,
    "row_fields": list(ROW_FIELDS),
    "candidate_fields": list(CANDIDATE_FIELDS),
    "proposal_trace_fields": list(PROPOSAL_TRACE_FIELDS),
    "gate_diagnostic_fields": list(GATE_DIAGNOSTIC_FIELDS),
    "reference_evidence_fields": list(REFERENCE_EVIDENCE_FIELDS),
}
ANALYSIS_SCHEMA = {
    "id": "cbf2026-predictive-wnls-development-analysis-v2",
    "json_name": "predictive-wnls-development.json",
    "markdown_name": "predictive-wnls-development.md",
}
GATES = {
    "maximum_published_error_m_strictly_below": 50.0,
    "maximum_fresh_error_m_strictly_below": 50.0,
    "paired_both_fresh_p95_must_not_worsen": True,
    "fresh_availability_max_drop_fraction": 0.02,
    "fresh_or_predicted_min_fraction": 0.95,
    "maximum_prediction_age_frames": 2,
    "qualification_anchor_violations_allowed": 0,
    "current_frame_provenance_violations_allowed": 0,
    "ascending_dag_violations_allowed": 0,
}
DISK_CONTRACT = {
    "launch_minimum_free_bytes": START_BYTES,
    "live_minimum_free_bytes": HARD_FLOOR_BYTES,
    "raw_bundle_max_allocated_bytes": RAW_BUNDLE_CAP_BYTES,
    "compact_bundle_max_allocated_bytes": 10_000_000,
}
EVIDENCE_LIFECYCLE = {
    "registered_retry_allowed": False,
    "exact_output_root_required": True,
    "preexisting_target_allowed": False,
    "nested_timestamp_directory_allowed": False,
    "terminal_manifest_required_on_success_and_failure": True,
    "paper_gate": "CLOSED",
}
PROTOCOL_TOP_LEVEL_FIELDS = {
    "schema_id",
    "protocol_id",
    "implementation_parent_commit",
    "binding_design",
    "sources",
    "experiment",
    "estimator_constants",
    "status_contract",
    "ablation_contracts",
    "raw_schema",
    "analysis_schema",
    "gates",
    "disk_contract",
    "invocations",
    "evidence_lifecycle",
    "commands",
}
REQUIRED_SOURCES = {
    "truth_data",
    "input_manifest",
    "replay_source",
    "estimator_source",
    "legacy_solver_source",
    "diagnostic_integrity_source",
    "baseline_process",
}
OPTIONAL_SOURCES = {"analyzer_source"}
STAGING_NAMES = ("finalizing", "completed", "failed")


def _native_json(value):
    if isinstance(value, np.ndarray):
        return _native_json(value.tolist())
    if isinstance(value, np.generic):
        return _native_json(value.item())
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("non-finite JSON evidence")
        return value
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, Mapping):
        result = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise ValueError("JSON evidence keys must be strings")
            result[key] = _native_json(item)
        return result
    if isinstance(value, (list, tuple)):
        return [_native_json(item) for item in value]
    raise TypeError(f"unsupported JSON evidence value: {type(value).__name__}")


def _strict_json_bytes(value, *, indent=None) -> bytes:
    return json.dumps(
        _native_json(value),
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":") if indent is None else None,
        indent=indent,
    ).encode("utf-8")


def _strict_load(path: Path) -> dict:
    def reject_constant(value: str):
        raise ValueError(f"non-finite JSON constant {value}")

    with path.open(encoding="utf-8") as source:
        value = json.load(source, parse_constant=reject_constant)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _absolute(path: Path) -> Path:
    path = Path(path)
    if not path.is_absolute() or ".." in path.parts:
        raise ValueError(f"path must be normalized and absolute: {path}")
    return path


def _lstat_components(path: Path, *, leaf_required: bool) -> None:
    path = _absolute(path)
    chain = list(reversed(path.parents)) + [path]
    for index, component in enumerate(chain):
        try:
            metadata = component.lstat()
        except FileNotFoundError:
            if leaf_required or index < len(chain) - 1:
                raise ValueError(f"missing trusted path component: {component}") from None
            return
        if stat.S_ISLNK(metadata.st_mode):
            raise ValueError(f"symbolic-link path component is forbidden: {component}")
    if leaf_required and not stat.S_ISREG(path.lstat().st_mode):
        raise ValueError(f"trusted leaf must be a regular file: {path}")


def _file_identity(path: Path) -> dict:
    _lstat_components(path, leaf_required=True)
    metadata = path.lstat()
    return {
        "path": str(path),
        "sha256": _sha256(path),
        "device": metadata.st_dev,
        "inode": metadata.st_ino,
        "size": metadata.st_size,
        "mtime_ns": metadata.st_mtime_ns,
    }


def _same_or_nested(first: Path, second: Path) -> bool:
    return first == second or first in second.parents or second in first.parents


def _stage_path(output_root: Path, state: str) -> Path:
    return output_root.parent / f".{output_root.name}.manifest.{state}"


def _validate_stage_absence(output_root: Path) -> None:
    for state in STAGING_NAMES:
        stage = _stage_path(output_root, state)
        if stage.exists() or stage.is_symlink():
            raise FileExistsError(f"pre-existing terminal staging path: {stage}")


def _validate_paths(
    project_root: Path,
    protocol_path: Path,
    source_paths: list[Path],
    output_root: Path,
    *,
    output_allocated: bool,
) -> None:
    _lstat_components(project_root, leaf_required=False)
    _lstat_components(protocol_path, leaf_required=True)
    for source in source_paths:
        _lstat_components(source, leaf_required=True)
    output_root = _absolute(output_root)
    _lstat_components(_nearest_existing_ancestor(output_root), leaf_required=False)
    if output_allocated:
        _lstat_components(output_root, leaf_required=False)
        if output_root.is_symlink() or not output_root.is_dir():
            raise ValueError("allocated output_root must be a non-symlink directory")
    elif output_root.exists() or output_root.is_symlink():
        raise FileExistsError(f"registered output_root already exists: {output_root}")
    protected = [project_root, protocol_path, protocol_path.parent]
    for source in source_paths:
        protected.extend((source, source.parent))
    if any(_same_or_nested(output_root, path) for path in protected):
        raise ValueError("output_root overlaps protected code or evidence")
    if not output_allocated:
        _validate_stage_absence(output_root)


def _create_exact_root(output_root: Path) -> None:
    ancestor = _nearest_existing_ancestor(output_root)
    _lstat_components(ancestor, leaf_required=False)
    pending = []
    cursor = output_root.parent
    while cursor != ancestor:
        pending.append(cursor)
        cursor = cursor.parent
    for directory in reversed(pending):
        directory.mkdir()
        if directory.is_symlink() or not directory.is_dir():
            raise ValueError(f"unsafe output parent: {directory}")
    output_root.mkdir(exist_ok=False)


def _validate_protocol_shape(
    protocol: dict,
    *,
    output_root: Path,
    run_seeds: tuple[int, ...],
    max_frames: int | None,
) -> str:
    if set(protocol) != PROTOCOL_TOP_LEVEL_FIELDS:
        raise ValueError("protocol top-level fields differ from exact contract")
    exact = {
        "schema_id": PROTOCOL_SCHEMA_ID,
        "protocol_id": PROTOCOL_ID,
        "binding_design": BINDING_DESIGN,
        "estimator_constants": ESTIMATOR_CONSTANTS,
        "status_contract": STATUS_CONTRACT,
        "ablation_contracts": ABLATION_CONTRACTS,
        "raw_schema": RAW_SCHEMA_DECLARATION,
        "analysis_schema": ANALYSIS_SCHEMA,
        "gates": GATES,
        "disk_contract": DISK_CONTRACT,
        "evidence_lifecycle": EVIDENCE_LIFECYCLE,
    }
    for field, expected in exact.items():
        if protocol.get(field) != expected:
            raise ValueError(f"protocol {field} differs from exact contract")
    commit = protocol.get("implementation_parent_commit")
    if not isinstance(commit, str) or re.fullmatch(r"[0-9a-f]{40}", commit) is None:
        raise ValueError("implementation_parent_commit must be 40 lowercase hex")
    experiment = protocol.get("experiment")
    if not isinstance(experiment, dict):
        raise ValueError("protocol experiment must be an object")
    seeds = experiment.get("range_noise_seeds")
    frames = experiment.get("max_frames")
    expected_experiment = {
        **EXPERIMENT_CONTRACT,
        "range_noise_seeds": seeds,
        "max_frames": frames,
    }
    if experiment != expected_experiment:
        raise ValueError("protocol experiment differs from exact contract")
    if (
        not isinstance(seeds, list)
        or not seeds
        or any(isinstance(seed, bool) or not isinstance(seed, int) for seed in seeds)
        or len(seeds) != len(set(seeds))
    ):
        raise ValueError("protocol seed cohort is invalid")
    if frames is not None and (
        isinstance(frames, bool) or not isinstance(frames, int) or frames <= 0
    ):
        raise ValueError("protocol max_frames is invalid")
    invocations = protocol.get("invocations")
    commands = protocol.get("commands")
    if (
        not isinstance(invocations, dict)
        or not invocations
        or not isinstance(commands, dict)
        or set(commands) != set(invocations)
        or any(
            not isinstance(command, list)
            or not command
            or any(not isinstance(token, str) or not token for token in command)
            for command in commands.values()
        )
    ):
        raise ValueError("protocol invocations/commands are invalid")
    selected = None
    replay_invocations = {}
    analyzer_invocations = []
    declared_output_roots = set()
    for name, declaration in invocations.items():
        if not isinstance(name, str) or not isinstance(declaration, dict):
            raise ValueError("protocol invocation declaration is invalid")
        replay_fields = {
            "kind",
            "output_root",
            "range_noise_seeds",
            "max_frames",
        }
        analyzer_fields = {
            "kind",
            "development_manifest_path",
            "output_root",
            "expected_baseline_sha256",
        }
        declaration_fields = set(declaration)
        if declaration_fields == replay_fields:
            if declaration["kind"] not in {
                "unregistered_smoke",
                "registered_exactly_once",
            }:
                raise ValueError("replay invocation kind is invalid")
            declared_root = _absolute(Path(declaration["output_root"]))
            if str(declared_root) != declaration["output_root"]:
                raise ValueError("replay output_root is not normalized")
            declared_seeds = declaration["range_noise_seeds"]
            declared_frames = declaration["max_frames"]
            if (
                not isinstance(declared_seeds, list)
                or not declared_seeds
                or any(
                    isinstance(seed, bool) or not isinstance(seed, int)
                    for seed in declared_seeds
                )
                or len(declared_seeds) != len(set(declared_seeds))
                or (
                    declared_frames is not None
                    and (
                        isinstance(declared_frames, bool)
                        or not isinstance(declared_frames, int)
                        or declared_frames <= 0
                    )
                )
            ):
                raise ValueError("replay invocation arguments are invalid")
            replay_invocations[name] = declaration
            if declaration["output_root"] == str(output_root):
                if selected is not None:
                    raise ValueError("output_root is declared more than once")
                selected = name
                if (
                    declared_seeds != list(run_seeds)
                    or declared_frames != max_frames
                ):
                    raise ValueError(
                        "runtime arguments differ from declared invocation"
                    )
        elif declaration_fields == analyzer_fields:
            if declaration["kind"] != "registered_exactly_once":
                raise ValueError("analyzer invocation kind is invalid")
            declared_root = _absolute(Path(declaration["output_root"]))
            development_manifest = _absolute(
                Path(declaration["development_manifest_path"])
            )
            baseline_hash = declaration["expected_baseline_sha256"]
            if (
                str(declared_root) != declaration["output_root"]
                or str(development_manifest)
                != declaration["development_manifest_path"]
                or not isinstance(baseline_hash, str)
                or re.fullmatch(r"[0-9a-f]{64}", baseline_hash) is None
            ):
                raise ValueError("analyzer invocation is invalid")
            analyzer_invocations.append(declaration)
        else:
            raise ValueError("invocation fields differ from exact contract")
        if declaration["output_root"] in declared_output_roots:
            raise ValueError("output_root is declared more than once")
        declared_output_roots.add(declaration["output_root"])
    if selected is None:
        raise ValueError("runtime output_root is not declared by protocol")
    registered_manifests = {
        str(Path(declaration["output_root"]) / TERMINAL_MANIFEST_NAME)
        for declaration in replay_invocations.values()
        if declaration["kind"] == "registered_exactly_once"
    }
    expected_baseline = protocol.get("sources", {}).get(
        "baseline_process", {}
    ).get("sha256")
    for declaration in analyzer_invocations:
        if (
            declaration["development_manifest_path"] not in registered_manifests
            or declaration["expected_baseline_sha256"] != expected_baseline
        ):
            raise ValueError("analyzer invocation is not bound to registered replay")
    if list(run_seeds) != seeds or max_frames != frames:
        # A generated smoke protocol declares its own exact experiment cohort.
        # The production registrar may instead include three invocations and
        # sets experiment to the registered cohort; those invocations are
        # validated independently by their exact declaration.
        kinds = {
            item.get("kind")
            for item in replay_invocations.values()
        }
        if kinds == {"unregistered_smoke"}:
            raise ValueError("smoke runtime differs from experiment declaration")
    return selected


def _verify_protocol_and_sources(
    *,
    protocol_path: Path,
    expected_protocol_identity: dict | None,
    data_path: Path,
    input_manifest_path: Path,
    output_root: Path,
    run_seeds: tuple[int, ...],
    max_frames: int | None,
    output_allocated: bool = False,
) -> tuple[dict, dict, dict, str]:
    protocol_identity = _file_identity(protocol_path)
    if (
        expected_protocol_identity is not None
        and protocol_identity != expected_protocol_identity
    ):
        raise ValueError("protocol identity changed after initial read")
    protocol = _strict_load(protocol_path)
    invocation = _validate_protocol_shape(
        protocol,
        output_root=output_root,
        run_seeds=run_seeds,
        max_frames=max_frames,
    )
    sources = protocol.get("sources")
    if not isinstance(sources, dict) or not (
        REQUIRED_SOURCES <= set(sources) <= REQUIRED_SOURCES | OPTIONAL_SOURCES
    ):
        raise ValueError("protocol sources differ from required exact set")
    project_root = Path(__file__).resolve().parents[2]
    actual_paths = {
        "truth_data": data_path,
        "input_manifest": input_manifest_path,
        "replay_source": Path(__file__).resolve(),
        "estimator_source": project_root / "scripts/diagnostics/predictive_wnls.py",
        "legacy_solver_source": (
            project_root / "scripts/diagnostics/replay_localization_calibration.py"
        ),
        "diagnostic_integrity_source": (
            project_root / "scripts/diagnostics/run_diagnostic.py"
        ),
    }
    identities = {}
    for name, declaration in sources.items():
        if (
            not isinstance(declaration, dict)
            or set(declaration) != {"path", "sha256"}
            or not isinstance(declaration["path"], str)
            or not isinstance(declaration["sha256"], str)
            or re.fullmatch(r"[0-9a-f]{64}", declaration["sha256"]) is None
        ):
            raise ValueError(f"protocol source declaration is invalid: {name}")
        declared_path = _absolute(Path(declaration["path"]))
        if name in actual_paths and declared_path != _absolute(actual_paths[name]):
            raise ValueError(f"protocol source path differs from runtime: {name}")
        identity = _file_identity(declared_path)
        if identity["sha256"] != declaration["sha256"]:
            raise ValueError(f"protocol source hash mismatch: {name}")
        identities[name] = identity
    if identities["legacy_solver_source"]["sha256"] != LEGACY_SOLVER_SHA256:
        raise ValueError("legacy solver source differs from frozen identity")
    _validate_paths(
        project_root,
        protocol_path,
        [Path(item["path"]) for item in identities.values()],
        output_root,
        output_allocated=output_allocated,
    )
    return protocol, protocol_identity, identities, invocation


def _preflight_frames(data: dict) -> tuple[list[dict], dict[int, dict[int, list[float]]]]:
    config = data.get("config")
    if not isinstance(config, dict):
        raise ValueError("truth replay must contain config")
    try:
        number = int(config["num"])
        parts = int(config["formation"]["parts"])
    except (KeyError, TypeError, ValueError):
        raise ValueError("config must provide positive num and formation.parts") from None
    if number <= 0 or parts <= 0:
        raise ValueError("config num/parts must be positive")
    frames = _frames(data)
    expected_ids = list(range(1, number + 1))
    commands: dict[int, dict[int, list[float]]] = {}
    for frame_index, frame in enumerate(frames):
        for field in ("frame_index", "id"):
            if field in frame and frame[field] != frame_index:
                raise ValueError(f"frame {field} linkage mismatch")
        if (
            "previous_frame_index" in frame
            and frame["previous_frame_index"] != frame_index - 1
        ):
            raise ValueError("previous frame linkage mismatch")
        robots = frame.get("robots")
        if not isinstance(robots, list):
            raise ValueError("every frame must contain a robots list")
        ids = [robot.get("id") if isinstance(robot, dict) else None for robot in robots]
        if ids != expected_ids:
            raise ValueError("frame UAV order/membership differs from exact IDs")
        _truth_positions(frame, set(expected_ids))
        if frame_index == len(frames) - 1:
            continue
        frame_commands = {}
        for robot in robots:
            opt = robot.get("opt")
            if not isinstance(opt, dict) or opt.get("status") != "success":
                raise ValueError("predecessor optimizer status must be success")
            result = opt.get("result")
            if not isinstance(result, dict) or "vx" not in result or "vy" not in result:
                raise ValueError("predecessor applied result must provide vx/vy")
            try:
                command = np.asarray([result["vx"], result["vy"]], dtype=float)
            except (TypeError, ValueError, OverflowError):
                raise ValueError("applied command must be a finite planar vector") from None
            if command.shape != (2,) or not np.isfinite(command).all():
                raise ValueError("applied command must be a finite planar vector")
            frame_commands[int(robot["id"])] = command.tolist()
        commands[frame_index] = frame_commands
    return frames, commands


def _squad_local_index(robot_id: int, number: int, parts: int) -> int:
    squad_size = math.ceil(number / parts)
    return (robot_id - 1) % squad_size + 1


def _sensor_records(
    config: dict,
    observer_id: int,
    truth: dict[int, np.ndarray],
    seed: int,
    frame_index: int,
    sigma: float,
) -> tuple[dict, list[tuple[str, int]], dict[tuple[str, int], dict]]:
    mandatory = fixed_references(config, observer_id)
    active = active_references(config, observer_id, truth)
    mandatory_keys = {
        ("base", identifier) for identifier in mandatory["base_ids"]
    } | {("uav", identifier) for identifier in mandatory["uav_ids"]}
    active_keys = {
        ("base", identifier) for identifier in active["base_ids"]
    } | {("uav", identifier) for identifier in active["uav_ids"]}
    optional = sorted(
        active_keys - mandatory_keys,
        key=lambda key: (0 if key[0] == "base" else 1, key[1]),
    )
    records = {}
    for kind, identifier in sorted(
        active_keys,
        key=lambda key: (0 if key[0] == "base" else 1, key[1]),
    ):
        reference = (
            np.asarray(config["bases"][identifier], dtype=float)
            if kind == "base"
            else truth[identifier]
        )
        noise_seed = stable_measurement_seed(
            seed, frame_index, observer_id, kind, identifier
        )
        noisy_range = float(
            np.linalg.norm(truth[observer_id] - reference)
            + np.random.default_rng(noise_seed).normal(0.0, sigma)
        )
        records[(kind, identifier)] = {
            "present": True,
            "noisy_range": noisy_range,
            "noise_seed": noise_seed,
        }
    return mandatory, optional, records


def _canonical_active_records(qualification: dict) -> list[dict]:
    by_key = {}
    for record in qualification.get("active_records", []):
        key = tuple(record.get("key", ()))
        if len(key) == 2 and key not in by_key:
            by_key[key] = record
    return [
        by_key[key]
        for key in sorted(by_key, key=lambda item: (0 if item[0] == "base" else 1, item[1]))
    ]


def _numeric_reference_arrays(
    config: dict,
    active_records: list[dict],
    uav_states: dict[int, dict],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[tuple[str, int]]] | None:
    positions = []
    covariances = []
    measurements = []
    keys = []
    for record in active_records:
        key = tuple(record["key"])
        if key[0] == "base":
            position = np.asarray(config["bases"][key[1]], dtype=float)
            covariance = np.zeros((2, 2), dtype=float)
        else:
            valid = _valid_prior_result(uav_states.get(key[1]))
            if valid is None:
                return None
            position, covariance = valid[0], valid[1]
        positions.append(position)
        covariances.append(covariance)
        measurements.append(record["noisy_range"])
        keys.append(key)
    if len(keys) < 2:
        return None
    return (
        np.asarray(positions),
        np.asarray(covariances),
        np.asarray(measurements),
        keys,
    )


def _public_reference_arrays(
    config: dict,
    active_records: list[dict],
    public_outputs: dict[int, dict],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[tuple[str, int]]] | None:
    states = {}
    for identifier, output in public_outputs.items():
        if reference_is_eligible(output):
            states[identifier] = {
                "status": "converged",
                "estimate": output["estimate"],
                "covariance": output["modeled_covariance"],
                "epsilon": output["epsilon"],
                "phi_min_eigenvalue": 1.0,
                "phi_condition": 1.0,
            }
    return _numeric_reference_arrays(config, active_records, states)


def _retain_legacy_numeric(previous: dict | None, attempt: dict) -> dict:
    valid = _valid_prior_result(previous)
    if attempt.get("status") == "converged" or valid is None:
        return attempt
    estimate, covariance, epsilon, phi_minimum, phi_condition = valid
    return {
        "status": "stale",
        "estimate": estimate.tolist(),
        "covariance": covariance.tolist(),
        "epsilon": epsilon,
        "phi_min_eigenvalue": phi_minimum,
        "phi_condition": phi_condition,
        "iterations": attempt.get("iterations", 0),
        "cost": attempt.get("cost"),
        "stationarity_norm": attempt.get("stationarity_norm"),
        "failure_reason": attempt.get("failure_reason"),
        "failure": attempt,
    }


def _unavailable(reason: str) -> dict:
    return {
        "output_status": "unavailable",
        "estimate": None,
        "modeled_covariance": None,
        "epsilon": None,
        "prediction_age": None,
        "aged_modeled_radius": None,
        "base_anchor_provenance": [],
        "reason": reason,
    }


def _finite_public_state(output: object):
    if not isinstance(output, dict) or output.get("output_status") not in {
        "fresh",
        "predicted",
    }:
        return None
    try:
        estimate = np.asarray(output["estimate"], dtype=float)
        covariance = np.asarray(output["modeled_covariance"], dtype=float)
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    if (
        estimate.shape != (2,)
        or covariance.shape != (2, 2)
        or not np.isfinite(estimate).all()
        or not np.isfinite(covariance).all()
    ):
        return None
    return estimate, covariance


def _finite_private_state(private: object):
    if not isinstance(private, dict):
        return None
    try:
        estimate = np.asarray(private["estimate"], dtype=float)
        covariance = np.asarray(private["modeled_covariance"], dtype=float)
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    if (
        estimate.shape != (2,)
        or covariance.shape != (2, 2)
        or not np.isfinite(estimate).all()
        or not np.isfinite(covariance).all()
    ):
        return None
    return estimate, covariance


def _propagate_public(
    previous_output: dict | None,
    previous_private: dict | None,
    command: list[float] | None,
) -> dict:
    if command is None:
        return {
            "public_prediction": _unavailable("no_predecessor_command"),
            "private_reacquisition_seed": None,
        }
    command_array = np.asarray(command, dtype=float)
    previous_public = _finite_public_state(previous_output)
    previous_is_unavailable = (
        isinstance(previous_output, dict)
        and previous_output.get("output_status") == "unavailable"
    )
    source = _finite_private_state(previous_private) if previous_is_unavailable else previous_public
    if source is None:
        return {
            "public_prediction": _unavailable("no_finite_prior"),
            "private_reacquisition_seed": None,
        }
    estimate, covariance = source
    next_estimate = estimate + FRAME_DT_SECONDS * command_array
    next_covariance = covariance + MOTION_SIGMA_M_PER_FRAME**2 * np.eye(2)
    private = {
        "estimate": next_estimate.tolist(),
        "modeled_covariance": next_covariance.tolist(),
    }
    if previous_is_unavailable:
        return {
            "public_prediction": _unavailable("previously_unavailable"),
            "private_reacquisition_seed": private,
        }
    age = 1 if previous_output["output_status"] == "fresh" else int(
        previous_output["prediction_age"]
    ) + 1
    if age > MAX_PREDICTION_AGE_FRAMES:
        return {
            "public_prediction": _unavailable("prediction_expired"),
            "private_reacquisition_seed": private,
        }
    return {
        "public_prediction": {
            "output_status": "predicted",
            "estimate": next_estimate.tolist(),
            "modeled_covariance": next_covariance.tolist(),
            "epsilon": None,
            "prediction_age": age,
            "aged_modeled_radius": 3.0 * math.sqrt(
                float(np.max(np.linalg.eigvalsh(next_covariance)))
            ),
            "base_anchor_provenance": [],
        },
        "private_reacquisition_seed": private,
    }


def _legacy_gate(
    result: dict,
    *,
    variant: str,
    has_live_prediction: bool,
    active_count: int,
    provenance: tuple[int, ...],
) -> tuple[bool, str, dict]:
    diagnostics = {
        "innovation_gate": "not_applied_legacy_solver",
        "q_innov": None,
        "gate_outcome": "invalid",
        "valid": result.get("status") == "converged",
        "failure_reason": result.get("failure_reason"),
        "reduced_whitened_cost": None,
    }
    if result.get("status") != "converged":
        return False, result.get("failure_reason") or "legacy_not_converged", diagnostics
    if variant == "prediction_expiry":
        diagnostics["gate_outcome"] = "accepted"
        return True, "accepted", diagnostics
    if len(set(provenance)) < 2:
        diagnostics["gate_outcome"] = "rejected"
        return False, "insufficient_base_anchor_provenance", diagnostics
    try:
        covariance = np.asarray(result["covariance"], dtype=float)
        eigenvalues = np.linalg.eigvalsh(covariance)
    except (KeyError, TypeError, ValueError, np.linalg.LinAlgError):
        diagnostics["failure_reason"] = "invalid_legacy_covariance"
        return False, "invalid_legacy_covariance", diagnostics
    if (
        covariance.shape != (2, 2)
        or not np.isfinite(covariance).all()
        or eigenvalues[0] <= RELATIVE_SPECTRAL_THRESHOLD * eigenvalues[-1]
    ):
        diagnostics["failure_reason"] = "legacy_fim_not_positive_definite"
        return False, "legacy_fim_not_positive_definite", diagnostics
    if not has_live_prediction:
        if active_count < 3:
            diagnostics["gate_outcome"] = "rejected"
            return False, "reacquisition_requires_three_active_references", diagnostics
        try:
            reduced = float(result["cost"]) / max(1, active_count - 2)
        except (KeyError, TypeError, ValueError, OverflowError):
            diagnostics["failure_reason"] = "invalid_legacy_cost"
            return False, "invalid_legacy_cost", diagnostics
        diagnostics["reduced_whitened_cost"] = reduced
        if not math.isfinite(reduced) or reduced > 9.0:
            diagnostics["gate_outcome"] = "rejected"
            return False, "reacquisition_reduced_cost_exceeds_nine", diagnostics
    diagnostics["gate_outcome"] = "accepted"
    return True, "accepted", diagnostics


def _legacy_attempt(
    result: dict,
    *,
    source: str,
    initial: np.ndarray,
    variant: str,
    live_prediction: dict | None,
    active_count: int,
    provenance: tuple[int, ...],
) -> dict:
    accepted, reason, diagnostics = _legacy_gate(
        result,
        variant=variant,
        has_live_prediction=(
            isinstance(live_prediction, dict)
            and live_prediction.get("output_status") == "predicted"
        ),
        active_count=active_count,
        provenance=provenance,
    )
    status = result.get("status")
    candidate = {
        "source": source,
        "initial_estimate": initial.tolist(),
        "status": status,
        "estimate": result.get("estimate"),
        "covariance": result.get("covariance"),
        "cost": result.get("cost"),
        "accepted": accepted,
        "rejection_reason": None if accepted else reason,
        "q_innov": None,
        "gate_diagnostics": diagnostics,
        "proposal_trace": [],
    }
    if accepted:
        return {
            "attempt_status": "accepted",
            "candidate": {
                "estimate": result["estimate"],
                "modeled_covariance": result["covariance"],
                "epsilon": result["epsilon"],
                "base_anchor_provenance": list(provenance),
            },
            "candidates": [candidate],
            "selected_candidate": candidate,
            "failure_reason": None,
        }
    attempt_status = (
        "failed"
        if status == "failed"
        else "invalid"
        if status == "invalid"
        else "rejected"
    )
    return {
        "attempt_status": attempt_status,
        "candidate": None,
        "candidates": [candidate],
        "selected_candidate": None,
        "failure_reason": reason,
    }


def _finalize_public(
    attempt: dict,
    prior: dict,
) -> tuple[dict, dict | None]:
    if attempt.get("attempt_status") == "accepted":
        candidate = attempt["candidate"]
        output = {
            "attempt_status": "accepted",
            "output_status": "fresh",
            "estimate": candidate["estimate"],
            "modeled_covariance": candidate["modeled_covariance"],
            "epsilon": candidate["epsilon"],
            "prediction_age": 0,
            "aged_modeled_radius": None,
            "base_anchor_provenance": candidate["base_anchor_provenance"],
        }
        private = {
            "estimate": list(candidate["estimate"]),
            "modeled_covariance": candidate["modeled_covariance"],
        }
        return output, private
    prediction = dict(prior["public_prediction"])
    prediction["attempt_status"] = attempt.get("attempt_status", "invalid")
    return prediction, prior.get("private_reacquisition_seed")


def _compact_gate(diagnostics: object) -> list:
    diagnostics = diagnostics if isinstance(diagnostics, dict) else {}
    return [diagnostics.get(field) for field in GATE_DIAGNOSTIC_FIELDS]


def _compact_candidates(candidates: object) -> list[list]:
    result = []
    for candidate in candidates if isinstance(candidates, list) else []:
        if not isinstance(candidate, dict):
            continue
        nested = candidate.get("result") if isinstance(candidate.get("result"), dict) else candidate
        trace = nested.get("proposal_trace", candidate.get("proposal_trace", []))
        compact_trace = [
            [proposal.get(field) for field in PROPOSAL_TRACE_FIELDS]
            for proposal in trace
            if isinstance(proposal, dict)
        ]
        result.append(
            [
                candidate.get("source"),
                candidate.get("initial_estimate"),
                candidate.get("status", nested.get("status")),
                candidate.get("estimate", nested.get("estimate")),
                candidate.get("covariance", nested.get("covariance")),
                candidate.get("cost", nested.get("cost")),
                candidate.get("accepted"),
                candidate.get("rejection_reason"),
                candidate.get("q_innov"),
                _compact_gate(candidate.get("gate_diagnostics")),
                compact_trace,
            ]
        )
    return result


def _reference_evidence(
    *,
    mandatory: dict,
    optional: list[tuple[str, int]],
    records: dict[tuple[str, int], dict],
    qualification: dict,
    current_public: dict[int, dict],
) -> list[list]:
    mandatory_keys = {
        ("base", identifier) for identifier in mandatory["base_ids"]
    } | {("uav", identifier) for identifier in mandatory["uav_ids"]}
    all_keys = sorted(
        mandatory_keys | set(optional),
        key=lambda key: (0 if key[0] == "base" else 1, key[1]),
    )
    used = {tuple(record["key"]) for record in _canonical_active_records(qualification)}
    excluded = {
        tuple(item["key"]): item["reason"]
        for item in qualification.get("excluded", [])
        if isinstance(item, dict) and "key" in item
    }
    for key in qualification.get("missing_mandatory", []):
        excluded[tuple(key)] = (
            "measurement_not_present"
            if not records.get(tuple(key), {}).get("present")
            else "not_current_frame_fresh"
        )
    evidence = []
    for key in all_keys:
        record = records.get(key, {"present": False, "noisy_range": None, "noise_seed": None})
        if key[0] == "base":
            freshness = "fresh"
            eligible = bool(record.get("present"))
            provenance = [key[1]]
        else:
            output = current_public.get(key[1])
            freshness = (
                output.get("output_status")
                if isinstance(output, dict)
                else "missing"
            )
            eligible = bool(record.get("present")) and reference_is_eligible(output)
            provenance = (
                list(output.get("base_anchor_provenance", []))
                if isinstance(output, dict)
                else []
            )
        evidence.append(
            [
                key[0],
                key[1],
                "mandatory" if key in mandatory_keys else "optional",
                bool(record.get("present")),
                record.get("noisy_range"),
                record.get("noise_seed"),
                freshness,
                eligible,
                key in used,
                excluded.get(key),
                provenance,
            ]
        )
    return evidence


def _offline_metrics(output: dict, truth: np.ndarray) -> dict:
    result = {
        "offline_truth_position": truth.tolist(),
        "offline_error_norm": None,
        "offline_fresh_containment": None,
        "offline_aged_radius_containment": None,
        "offline_fresh_q_error": None,
        "offline_aged_q_error": None,
    }
    state = _finite_public_state(output)
    if state is None:
        return result
    estimate, covariance = state
    error = truth - estimate
    result["offline_error_norm"] = float(np.linalg.norm(error))
    try:
        q_error = float(error @ np.linalg.solve(covariance, error))
    except np.linalg.LinAlgError:
        q_error = None
    if output["output_status"] == "fresh":
        result["offline_fresh_containment"] = bool(
            result["offline_error_norm"] <= float(output["epsilon"])
        )
        result["offline_fresh_q_error"] = q_error
    else:
        result["offline_aged_radius_containment"] = bool(
            result["offline_error_norm"] <= float(output["aged_modeled_radius"])
        )
        result["offline_aged_q_error"] = q_error
    return result


def _write_row(compressed, line: bytes, digest) -> None:
    compressed.write(line)
    digest.update(line)


def _safe_probe(function, path: Path):
    try:
        return function(path)
    except BaseException:
        return None


def _write_stage(path: Path, payload: bytes) -> None:
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, "wb") as target:
        target.write(payload)
        target.flush()
        os.fsync(target.fileno())


def _cleanup_stage(path: Path) -> None:
    path.unlink()


def _link_stage(stage: Path, target: Path) -> None:
    os.link(stage, target)


def _unlink_noexcept(path: Path) -> None:
    try:
        path.unlink()
    except BaseException:
        pass


def _external_stage(payload: bytes) -> Path:
    descriptor, name = tempfile.mkstemp(prefix="cbf2026-terminal-", suffix=".json")
    path = Path(name)
    try:
        with os.fdopen(descriptor, "wb") as target:
            target.write(payload)
            target.flush()
            os.fsync(target.fileno())
    except BaseException:
        _unlink_noexcept(path)
        raise
    return path


def _publish_failed(output_root: Path, manifest: dict) -> None:
    target = output_root / TERMINAL_MANIFEST_NAME
    if target.exists() or target.is_symlink():
        return
    payload = _strict_json_bytes(manifest, indent=2) + b"\n"
    stage = _external_stage(payload)
    try:
        os.link(stage, target)
    finally:
        _unlink_noexcept(stage)
        for state in STAGING_NAMES:
            _unlink_noexcept(_stage_path(output_root, state))


def _final_identity_and_disk_probe(
    *,
    protocol_path: Path,
    protocol_identity: dict,
    data_path: Path,
    input_manifest_path: Path,
    output_root: Path,
    run_seeds: tuple[int, ...],
    max_frames: int | None,
) -> tuple[dict, dict, str]:
    _, observed_protocol, identities, invocation = _verify_protocol_and_sources(
        protocol_path=protocol_path,
        expected_protocol_identity=protocol_identity,
        data_path=data_path,
        input_manifest_path=input_manifest_path,
        output_root=output_root,
        run_seeds=run_seeds,
        max_frames=max_frames,
        output_allocated=True,
    )
    if available_bytes(output_root) < HARD_FLOOR_BYTES:
        raise DiskSpaceError("available bytes below live floor")
    if allocated_bytes(output_root) > RAW_BUNDLE_CAP_BYTES:
        raise DiskSpaceError("raw bundle exceeds allocated-byte cap")
    return observed_protocol, identities, invocation


def replay_predictive_recovery(
    *,
    data_path: Path,
    input_manifest_path: Path,
    protocol_path: Path,
    output_root: Path,
    run_seeds: tuple[int, ...],
    max_frames: int | None = None,
) -> dict:
    """Create exactly one protocol-authorized, disk-guarded Stage-1 bundle."""
    data_path = _absolute(Path(data_path))
    input_manifest_path = _absolute(Path(input_manifest_path))
    protocol_path = _absolute(Path(protocol_path))
    output_root = _absolute(Path(output_root))
    if (
        not run_seeds
        or any(isinstance(seed, bool) or not isinstance(seed, int) for seed in run_seeds)
        or len(set(run_seeds)) != len(run_seeds)
    ):
        raise ValueError("run_seeds must be unique non-empty integers")
    protocol, protocol_identity, source_identities, invocation = (
        _verify_protocol_and_sources(
            protocol_path=protocol_path,
            expected_protocol_identity=None,
            data_path=data_path,
            input_manifest_path=input_manifest_path,
            output_root=output_root,
            run_seeds=run_seeds,
            max_frames=max_frames,
        )
    )
    data = _strict_load(data_path)
    input_manifest = _strict_load(input_manifest_path)
    if input_manifest.get("termination_reason") != "completed":
        raise ValueError("input manifest is not completed")
    # Second identity pass closes the check/read race before any allocation.
    protocol, protocol_identity, source_identities, invocation = (
        _verify_protocol_and_sources(
            protocol_path=protocol_path,
            expected_protocol_identity=protocol_identity,
            data_path=data_path,
            input_manifest_path=input_manifest_path,
            output_root=output_root,
            run_seeds=run_seeds,
            max_frames=max_frames,
        )
    )
    all_frames, applied_commands = _preflight_frames(data)
    frames = all_frames if max_frames is None else all_frames[:max_frames]
    config = data["config"]
    number = int(config["num"])
    parts = int(config["formation"]["parts"])
    expected_ids = set(range(1, number + 1))
    deployment = _initial_positions(config, expected_ids)
    sigma = float(config["position_covariance"]["ranging_sigma"])
    if sigma != EXPERIMENT_CONTRACT["ranging_sigma_m"]:
        raise ValueError("ranging sigma differs from exact protocol")
    free_before = require_start_space(_nearest_existing_ancestor(output_root))
    _create_exact_root(output_root)
    started_at = datetime.now(timezone.utc).isoformat()
    process_path = output_root / RAW_PROCESS_NAME
    rows_written = 0
    decompressed_digest = hashlib.sha256()

    def terminal(status: str, error: BaseException | None = None) -> dict:
        value = {
            "status": status,
            "schema_id": RAW_SCHEMA_ID,
            "raw_schema": RAW_SCHEMA_DECLARATION,
            "protocol_identity": protocol_identity,
            "protocol_parent": str(protocol_path.parent),
            "selected_invocation": invocation,
            "source_identities": source_identities,
            "output_root": str(output_root),
            "run_seeds": list(run_seeds),
            "max_frames": max_frames,
            "started_at": started_at,
            "ended_at": datetime.now(timezone.utc).isoformat(),
            "rows_written": rows_written,
            "expected_rows": (
                len(DEVELOPMENT_VARIANTS) * len(run_seeds) * len(frames) * number
            ),
            "free_bytes_before": free_before,
            "free_bytes_after": _safe_probe(available_bytes, output_root),
            "allocated_bytes": _safe_probe(allocated_bytes, output_root),
            "raw_bundle_cap_bytes": RAW_BUNDLE_CAP_BYTES,
            "compressed_process_sha256": (
                _sha256(process_path) if process_path.exists() else None
            ),
            "decompressed_process_sha256": (
                decompressed_digest.hexdigest() if process_path.exists() else None
            ),
        }
        if error is not None:
            value["error"] = {
                "type": type(error).__name__,
                "message": str(error),
            }
        return value

    try:
        if available_bytes(output_root) < HARD_FLOOR_BYTES:
            raise DiskSpaceError("available bytes below live floor")
        states: dict[tuple[str, int], dict[int, dict]] = {}
        with process_path.open("xb") as raw, gzip.GzipFile(
            filename="", fileobj=raw, mode="wb", mtime=0
        ) as compressed:
            for variant in DEVELOPMENT_VARIANTS:
                for seed in run_seeds:
                    variant_state = states.setdefault((variant, seed), {})
                    for frame_index, frame in enumerate(frames):
                        truth = _truth_positions(frame, expected_ids)
                        current_public: dict[int, dict] = {}
                        current_numeric: dict[int, dict] = {}
                        current_numeric_provenance: dict[int, tuple[int, ...]] = {}
                        for robot_id in range(1, number + 1):
                            previous = variant_state.get(robot_id, {})
                            command = (
                                None
                                if frame_index == 0
                                else applied_commands[frame_index - 1][robot_id]
                            )
                            command_source = None if frame_index == 0 else frame_index - 1
                            prior = _propagate_public(
                                previous.get("public_output"),
                                previous.get("private_seed"),
                                command,
                            )
                            mandatory, optional, records = _sensor_records(
                                config,
                                robot_id,
                                truth,
                                seed,
                                frame_index,
                                sigma,
                            )
                            qualification = qualify_active_references(
                                mandatory=mandatory,
                                optional_keys=optional,
                                measurement_records=records,
                                uav_outputs=current_public,
                                variant=variant,
                            )
                            active_records = _canonical_active_records(qualification)
                            if variant == "prediction_expiry":
                                roots = {
                                    int(record["key"][1])
                                    for record in active_records
                                    if record["key"][0] == "base"
                                }
                                for record in active_records:
                                    key = tuple(record["key"])
                                    if key[0] == "uav":
                                        roots.update(
                                            current_numeric_provenance.get(
                                                key[1], ()
                                            )
                                        )
                                attempted_provenance = tuple(sorted(roots))
                            else:
                                attempted_provenance = tuple(
                                    qualification.get(
                                        "base_anchor_provenance", ()
                                    )
                                )
                            legacy_previous = previous.get("legacy_numeric_state")
                            ever_finite = bool(previous.get("ever_finite", False))
                            legacy_initial, legacy_initial_source = select_initial_estimate(
                                RESTART_BEFORE_FIRST_FINITE_POLICY,
                                frame_index=frame_index,
                                deployment=deployment[robot_id],
                                previous_result=legacy_previous,
                                ever_acquired_finite=ever_finite,
                            )
                            if variant == "prediction_expiry":
                                arrays = _numeric_reference_arrays(
                                    config, active_records, current_numeric
                                )
                            else:
                                arrays = (
                                    _public_reference_arrays(
                                        config, active_records, current_public
                                    )
                                    if qualification.get("status") == "ok"
                                    else None
                                )
                            legacy_result = None
                            if arrays is None:
                                legacy_attempt = {
                                    "status": "invalid",
                                    "estimate": None,
                                    "covariance": None,
                                    "epsilon": None,
                                    "phi_min_eigenvalue": None,
                                    "phi_condition": None,
                                    "iterations": 0,
                                    "cost": None,
                                    "stationarity_norm": None,
                                    "failure_reason": "reference_unavailable",
                                }
                                legacy_result = _retain_legacy_numeric(
                                    legacy_previous, legacy_attempt
                                )
                                attempt = {
                                    "attempt_status": "reference_unavailable",
                                    "candidate": None,
                                    "candidates": [],
                                    "selected_candidate": None,
                                    "failure_reason": "reference_unavailable",
                                }
                            else:
                                positions, covariances, measurements, keys = arrays
                                if variant == "predictive_multistart":
                                    attempt = solve_predictive_multistart(
                                        reference_positions=positions,
                                        reference_covariances=covariances,
                                        measurements=measurements,
                                        reference_keys=keys,
                                        live_prediction=(
                                            prior["public_prediction"]
                                            if prior["public_prediction"].get(
                                                "output_status"
                                            )
                                            == "predicted"
                                            else None
                                        ),
                                        private_seed=prior.get(
                                            "private_reacquisition_seed"
                                        ),
                                        ranging_sigma=sigma,
                                        base_anchor_provenance=attempted_provenance,
                                    )
                                else:
                                    # Both cumulative ablations reserve the
                                    # exact legacy initialization channel.
                                    # The current private seed is still aged
                                    # and serialized, but only the complete
                                    # multistart variant may consume it.
                                    single_initial = np.asarray(
                                        legacy_initial, dtype=float
                                    )
                                    source = legacy_initial_source
                                    if frame_index == 0:
                                        legacy_result = solve_wnls(
                                            positions,
                                            covariances,
                                            measurements,
                                            single_initial,
                                            sigma,
                                        )
                                    else:
                                        legacy_result = solve_later_frame(
                                            legacy_previous,
                                            positions,
                                            covariances,
                                            measurements,
                                            single_initial,
                                            sigma,
                                        )
                                    numeric_attempt = (
                                        legacy_result.get("failure")
                                        if legacy_result.get("status") == "stale"
                                        else legacy_result
                                    )
                                    attempt = _legacy_attempt(
                                        numeric_attempt,
                                        source=source,
                                        initial=single_initial,
                                        variant=variant,
                                        live_prediction=prior["public_prediction"],
                                        active_count=len(positions),
                                        provenance=attempted_provenance,
                                    )
                            if variant == "prediction_expiry" and legacy_result is not None:
                                current_numeric[robot_id] = legacy_result
                            elif legacy_result is not None:
                                current_numeric[robot_id] = legacy_result
                            if legacy_result is not None:
                                if legacy_result.get("status") == "converged":
                                    numeric_provenance = attempted_provenance
                                elif legacy_result.get("status") == "stale":
                                    numeric_provenance = tuple(
                                        previous.get(
                                            "legacy_numeric_provenance", ()
                                        )
                                    )
                                else:
                                    numeric_provenance = ()
                                current_numeric_provenance[robot_id] = (
                                    numeric_provenance
                                )
                            else:
                                numeric_provenance = ()
                            output, current_private = _finalize_public(attempt, prior)
                            current_public[robot_id] = output
                            if legacy_result is not None and _valid_prior_result(legacy_result):
                                ever_finite = True
                            variant_state[robot_id] = {
                                "legacy_numeric_state": legacy_result,
                                "legacy_numeric_provenance": numeric_provenance,
                                "public_output": output,
                                "private_seed": current_private,
                                "ever_finite": ever_finite,
                            }
                            reference_evidence = _reference_evidence(
                                mandatory=mandatory,
                                optional=optional,
                                records=records,
                                qualification=qualification,
                                current_public={
                                    key: value
                                    for key, value in current_public.items()
                                    if key != robot_id
                                },
                            )
                            fresh_covariance = (
                                output.get("modeled_covariance")
                                if output["output_status"] == "fresh"
                                else None
                            )
                            aged_covariance = (
                                output.get("modeled_covariance")
                                if output["output_status"] == "predicted"
                                else None
                            )
                            row = {
                                "variant": variant,
                                "seed": seed,
                                "frame_index": frame_index,
                                "robot_id": robot_id,
                                "squad_local_index": _squad_local_index(
                                    robot_id, number, parts
                                ),
                                "applied_command_source_frame": command_source,
                                "applied_command": command,
                                "legacy_numeric_status": (
                                    legacy_result.get("status")
                                    if isinstance(legacy_result, dict)
                                    else None
                                ),
                                "legacy_initial_estimate_source": (
                                    legacy_initial_source
                                    if variant != "predictive_multistart"
                                    else None
                                ),
                                "attempt_status": attempt["attempt_status"],
                                "attempt_failure_reason": attempt.get(
                                    "failure_reason"
                                ),
                                "output_status": output["output_status"],
                                "prediction_age": output.get("prediction_age"),
                                "estimate": output.get("estimate"),
                                "fresh_modeled_covariance": fresh_covariance,
                                "fresh_epsilon": (
                                    output.get("epsilon")
                                    if output["output_status"] == "fresh"
                                    else None
                                ),
                                "aged_modeled_covariance": aged_covariance,
                                "aged_modeled_radius": (
                                    output.get("aged_modeled_radius")
                                    if output["output_status"] == "predicted"
                                    else None
                                ),
                                "private_reacquisition_seed": current_private,
                                "attempt_base_anchor_provenance": list(
                                    attempted_provenance
                                ),
                                "base_anchor_provenance": output.get(
                                    "base_anchor_provenance", []
                                ),
                                "mandatory_references": mandatory,
                                "optional_candidates": [
                                    [kind, identifier]
                                    for kind, identifier in optional
                                ],
                                "active_references": [
                                    list(record["key"])
                                    for record in active_records
                                ],
                                "reference_evidence": reference_evidence,
                                "excluded_references": qualification.get(
                                    "excluded", []
                                ),
                                "reference_violations": qualification.get(
                                    "violations", []
                                ),
                                "candidates": _compact_candidates(
                                    attempt.get("candidates")
                                ),
                                "selected_candidate_source": (
                                    attempt.get("selected_candidate") or {}
                                ).get("source"),
                                **_offline_metrics(output, truth[robot_id]),
                            }
                            if set(row) != set(ROW_FIELDS):
                                raise RuntimeError("internal raw-row schema drift")
                            line = _strict_json_bytes(row) + b"\n"
                            _write_row(compressed, line, decompressed_digest)
                            rows_written += 1
                        compressed.flush()
                        if available_bytes(output_root) < HARD_FLOOR_BYTES:
                            raise DiskSpaceError("available bytes below live floor")
                        if allocated_bytes(output_root) > RAW_BUNDLE_CAP_BYTES:
                            raise DiskSpaceError(
                                "raw bundle exceeds allocated-byte cap"
                            )
        finalizing_stage = _stage_path(output_root, "finalizing")
        _write_stage(
            finalizing_stage,
            _strict_json_bytes(terminal("finalizing"), indent=2) + b"\n",
        )
        observed_protocol, observed_sources, observed_invocation = (
            _final_identity_and_disk_probe(
                protocol_path=protocol_path,
                protocol_identity=protocol_identity,
                data_path=data_path,
                input_manifest_path=input_manifest_path,
                output_root=output_root,
                run_seeds=run_seeds,
                max_frames=max_frames,
            )
        )
        protocol_identity = observed_protocol
        source_identities = observed_sources
        invocation = observed_invocation
        _cleanup_stage(finalizing_stage)
        completed = terminal("completed")
        completed_stage = _external_stage(
            _strict_json_bytes(completed, indent=2) + b"\n"
        )
        try:
            _link_stage(
                completed_stage,
                output_root / TERMINAL_MANIFEST_NAME,
            )
        except BaseException:
            _unlink_noexcept(completed_stage)
            raise
        # Publication above is the final throwing boundary.  Cleanup is
        # best-effort and external to the exact output bundle.
        _unlink_noexcept(completed_stage)
        return completed
    except BaseException as error:
        _publish_failed(output_root, terminal("failed", error))
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-path", type=Path, required=True)
    parser.add_argument("--input-manifest-path", type=Path, required=True)
    parser.add_argument("--protocol-json", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--run-seeds", required=True)
    parser.add_argument("--max-frames", type=int)
    arguments = parser.parse_args(argv)
    replay_predictive_recovery(
        data_path=arguments.data_path,
        input_manifest_path=arguments.input_manifest_path,
        protocol_path=arguments.protocol_json,
        output_root=arguments.output_root,
        run_seeds=tuple(
            int(value) for value in arguments.run_seeds.split(",") if value
        ),
        max_frames=arguments.max_frames,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
