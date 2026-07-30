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
    allocated_bytes,
    available_bytes,
    require_start_space,
)


DEVELOPMENT_VARIANTS = (
    "prediction_expiry",
    "fresh_reference_qualification",
    "predictive_multistart",
)
PROTOCOL_SCHEMA_ID = "cbf2026-predictive-wnls-stage1-protocol-v2"
PROTOCOL_ID = "cbf2026-predictive-wnls-stage1-v2"
HERMETIC_PROTOCOL_SCHEMA_ID = (
    "cbf2026-predictive-wnls-stage1-hermetic-protocol-v1"
)
HERMETIC_PROTOCOL_ID = "cbf2026-predictive-wnls-stage1-hermetic-v1"
RAW_SCHEMA_ID = "cbf2026-predictive-wnls-development-rows-v2"
RAW_PROCESS_NAME = "predictive-wnls-development.jsonl.gz"
TERMINAL_MANIFEST_NAME = "manifest.json"
RAW_BUNDLE_CAP_BYTES = 2_000_000_000
MOTION_SIGMA_M_PER_FRAME = 0.5
LEGACY_SOLVER_SHA256 = (
    "0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8"
)
PRODUCTION_RANGE_NOISE_SEEDS = tuple(range(20260727, 20260747))
PRODUCTION_PROTOCOL_TOKEN = (
    "docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.json"
)
PRODUCTION_TRUTH_DATA_PATH = (
    "/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/"
    "20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/"
    "2026-07-28_14-27-53_R_seed_20260727_250s/data.json"
)
PRODUCTION_TRUTH_DATA_SHA256 = (
    "3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527"
)
PRODUCTION_INPUT_MANIFEST_PATH = (
    "/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/"
    "20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/manifest.json"
)
PRODUCTION_INPUT_MANIFEST_SHA256 = (
    "6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb"
)
PRODUCTION_BASELINE_PROCESS_PATH = (
    "/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/"
    "20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/restart/"
    "localization-calibration/"
    "20260729T112437.413270Z_efb8e7137776415c833f11204cf519d5/"
    "calibration.jsonl.gz"
)
PRODUCTION_BASELINE_PROCESS_SHA256 = (
    "c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003"
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
MULTISTART_CANDIDATE_RECORD_FIELDS = (
    "source",
    "initial_estimate",
    "result",
    "accepted",
    "rejection_reason",
    "gate_diagnostics",
    "status",
    "estimate",
    "covariance",
    "cost",
    "q_innov",
)
SOLVER_RESULT_FIELDS = (
    "status",
    "estimate",
    "covariance",
    "epsilon",
    "phi_min_eigenvalue",
    "phi_condition",
    "fim_valid",
    "proposal_count",
    "iterations",
    "cost",
    "stationarity_norm",
    "failure_reason",
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
REFERENCE_FRESHNESS_FIELDS = (
    "reference_kind",
    "reference_id",
    "current_freshness",
)
MANDATORY_REFERENCE_FIELDS = ("base_ids", "uav_ids")
REFERENCE_KEY_FIELDS = ("reference_kind", "reference_id")
EXCLUSION_FIELDS = ("reference_kind", "reference_id", "reason")
VIOLATION_FIELDS = ("reference_kind", "reference_id", "reason")
PRIVATE_STATE_FIELDS = ("estimate", "modeled_covariance")
PUBLIC_STATE_FIELDS = (
    "output_status",
    "prediction_age",
    "estimate",
    "fresh_modeled_covariance",
    "fresh_epsilon",
    "aged_modeled_covariance",
    "aged_modeled_radius",
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
    "reference_freshness",
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
    "legacy_candidate_record_fields": list(CANDIDATE_FIELDS),
    "multistart_candidate_record_fields": list(
        MULTISTART_CANDIDATE_RECORD_FIELDS
    ),
    "solver_result_fields": list(SOLVER_RESULT_FIELDS),
    "proposal_trace_fields": list(PROPOSAL_TRACE_FIELDS),
    "gate_diagnostic_fields": list(GATE_DIAGNOSTIC_FIELDS),
    "reference_evidence_fields": list(REFERENCE_EVIDENCE_FIELDS),
    "reference_freshness_fields": list(REFERENCE_FRESHNESS_FIELDS),
    "mandatory_reference_fields": list(MANDATORY_REFERENCE_FIELDS),
    "reference_key_fields": list(REFERENCE_KEY_FIELDS),
    "exclusion_fields": list(EXCLUSION_FIELDS),
    "violation_fields": list(VIOLATION_FIELDS),
    "private_state_fields": list(PRIVATE_STATE_FIELDS),
    "public_state_fields": list(PUBLIC_STATE_FIELDS),
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
    "analyzer_source",
}
HERMETIC_REQUIRED_SOURCES = REQUIRED_SOURCES - {"analyzer_source"}
STAGING_NAMES = ("finalizing", "completed", "failed")


def canonical_replay_argv(
    *,
    data_path: Path,
    input_manifest_path: Path,
    protocol_token: str,
    output_root: Path,
    run_seeds: tuple[int, ...],
    max_frames: int | None,
) -> list[str]:
    """Return the token-for-token replay command bound by a protocol."""
    command = [
        "conda",
        "run",
        "-n",
        "cbf_env",
        "python",
        "scripts/diagnostics/replay_predictive_wnls_recovery.py",
        "--data-path",
        str(data_path),
        "--input-manifest-path",
        str(input_manifest_path),
        "--protocol-json",
        protocol_token,
        "--output-root",
        str(output_root),
        "--run-seeds",
        ",".join(str(seed) for seed in run_seeds),
    ]
    if max_frames is not None:
        command.extend(("--max-frames", str(max_frames)))
    return command


def canonical_analyzer_argv(
    *,
    baseline_process_path: Path,
    development_manifest_path: Path,
    protocol_token: str,
    expected_baseline_sha256: str,
    output_root: Path,
) -> list[str]:
    """Return the token-for-token analyzer command bound by production v2."""
    return [
        "conda",
        "run",
        "-n",
        "cbf_env",
        "python",
        "scripts/diagnostics/analyze_predictive_wnls_recovery.py",
        "--baseline-process-path",
        str(baseline_process_path),
        "--development-manifest-path",
        str(development_manifest_path),
        "--protocol-json",
        protocol_token,
        "--expected-baseline-sha256",
        expected_baseline_sha256,
        "--output-root",
        str(output_root),
    ]


def production_invocation_contract() -> dict[str, dict]:
    """Return the exact four-invocation registered Stage-1 contract."""
    registered_root = Path(
        "/private/tmp/cbf2026-predictive-wnls-development/stage1-v2"
    )
    return {
        "smoke_a": {
            "kind": "unregistered_smoke",
            "output_root": "/private/tmp/cbf2026-predictive-wnls-smoke-a",
            "range_noise_seeds": [20260727],
            "max_frames": 2,
        },
        "smoke_b": {
            "kind": "unregistered_smoke",
            "output_root": "/private/tmp/cbf2026-predictive-wnls-smoke-b",
            "range_noise_seeds": [20260727],
            "max_frames": 2,
        },
        "registered_replay": {
            "kind": "registered_exactly_once",
            "output_root": str(registered_root),
            "range_noise_seeds": list(PRODUCTION_RANGE_NOISE_SEEDS),
            "max_frames": None,
        },
        "registered_analyzer": {
            "kind": "registered_exactly_once",
            "development_manifest_path": str(
                registered_root / TERMINAL_MANIFEST_NAME
            ),
            "output_root": (
                "/private/tmp/cbf2026-predictive-wnls-development-analysis/"
                "stage1-v2"
            ),
            "expected_baseline_sha256": PRODUCTION_BASELINE_PROCESS_SHA256,
        },
    }


def production_command_contract(sources: Mapping) -> dict[str, list[str]]:
    """Return canonical argv arrays for Task 6's production registrar."""
    invocations = production_invocation_contract()
    data_path = Path(sources["truth_data"]["path"])
    input_manifest_path = Path(sources["input_manifest"]["path"])
    commands = {}
    for name in ("smoke_a", "smoke_b", "registered_replay"):
        declaration = invocations[name]
        commands[name] = canonical_replay_argv(
            data_path=data_path,
            input_manifest_path=input_manifest_path,
            protocol_token=PRODUCTION_PROTOCOL_TOKEN,
            output_root=Path(declaration["output_root"]),
            run_seeds=tuple(declaration["range_noise_seeds"]),
            max_frames=declaration["max_frames"],
        )
    analyzer = invocations["registered_analyzer"]
    commands["registered_analyzer"] = canonical_analyzer_argv(
        baseline_process_path=Path(sources["baseline_process"]["path"]),
        development_manifest_path=Path(analyzer["development_manifest_path"]),
        protocol_token=PRODUCTION_PROTOCOL_TOKEN,
        expected_baseline_sha256=analyzer["expected_baseline_sha256"],
        output_root=Path(analyzer["output_root"]),
    )
    return commands


def _native_json(value):
    if isinstance(value, np.ndarray):
        return _native_json(value.tolist())
    if isinstance(value, np.generic):
        converted = value.item()
        if isinstance(converted, np.generic):
            raise TypeError(
                f"unsupported NumPy scalar evidence type: {type(value).__name__}"
            )
        return _native_json(converted)
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


def _parse_json_object(payload: bytes, path: Path) -> dict:
    def reject_constant(value: str):
        raise ValueError(f"non-finite JSON constant {value}")

    def finite_float(value: str) -> float:
        parsed = float(value)
        if not math.isfinite(parsed):
            raise ValueError(f"non-finite JSON float {value}")
        return parsed

    try:
        value = json.loads(
            payload.decode("utf-8"),
            parse_constant=reject_constant,
            parse_float=finite_float,
        )
    except UnicodeDecodeError as error:
        raise ValueError(f"{path} is not UTF-8 JSON") from error
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    _native_json(value)
    return value


def _read_trusted_bytes(
    path: Path,
    *,
    expected_identity: dict | None = None,
    expected_sha256: str | None = None,
    capture_payload: bool = True,
) -> tuple[bytes | None, dict]:
    """Read and identify the same no-follow descriptor bytes atomically."""
    path = _absolute(path)
    _lstat_components(path, leaf_required=True)
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise ValueError(f"trusted leaf must be a regular file: {path}")
        chunks = [] if capture_payload else None
        digest = hashlib.sha256()
        bytes_read = 0
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
            bytes_read += len(chunk)
            if chunks is not None:
                chunks.append(chunk)
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    state_before = (
        before.st_dev,
        before.st_ino,
        before.st_size,
        before.st_mtime_ns,
    )
    state_after = (
        after.st_dev,
        after.st_ino,
        after.st_size,
        after.st_mtime_ns,
    )
    if state_before != state_after:
        raise ValueError(f"trusted file changed while reading: {path}")
    payload = None if chunks is None else b"".join(chunks)
    if bytes_read != before.st_size:
        raise ValueError(f"trusted file size changed while reading: {path}")
    _lstat_components(path, leaf_required=True)
    path_state = path.lstat()
    if (
        path_state.st_dev,
        path_state.st_ino,
        path_state.st_size,
        path_state.st_mtime_ns,
    ) != state_after:
        raise ValueError(f"trusted path identity changed while reading: {path}")
    identity = {
        "path": str(path),
        "sha256": digest.hexdigest(),
        "device": after.st_dev,
        "inode": after.st_ino,
        "size": after.st_size,
        "mtime_ns": after.st_mtime_ns,
    }
    if expected_sha256 is not None and identity["sha256"] != expected_sha256:
        raise ValueError(f"trusted file hash mismatch: {path}")
    if expected_identity is not None and identity != expected_identity:
        raise ValueError(f"trusted file identity changed: {path}")
    return payload, identity


def _strict_load(path: Path) -> dict:
    payload, _ = _read_trusted_bytes(_absolute(Path(path)))
    if payload is None:
        raise RuntimeError("internal trusted JSON payload was not captured")
    return _parse_json_object(payload, Path(path))


def _absolute(path: Path) -> Path:
    path = Path(path)
    if not path.is_absolute() or ".." in path.parts:
        raise ValueError(f"path must be normalized and absolute: {path}")
    return path


def _runtime_absolute(path: Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return _absolute(path)
    if ".." in path.parts:
        raise ValueError(f"runtime path must not contain parent traversal: {path}")
    return _absolute(Path.cwd() / path)


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
    _, identity = _read_trusted_bytes(path, capture_payload=False)
    return identity


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


def _directory_identity(metadata: os.stat_result, path: Path) -> dict:
    return {
        "path": str(path),
        "device": metadata.st_dev,
        "inode": metadata.st_ino,
    }


def _create_exact_root(
    output_root: Path,
    *,
    identity_sink: dict | None = None,
) -> dict:
    """Create every missing component through no-follow directory descriptors."""
    output_root = _absolute(output_root)
    if identity_sink is not None:
        identity_sink.clear()
    components = output_root.parts[1:]
    if not components:
        raise ValueError("output_root cannot be filesystem root")
    directory_flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    parent_fd = os.open("/", directory_flags)
    try:
        for component in components[:-1]:
            try:
                next_fd = os.open(component, directory_flags, dir_fd=parent_fd)
            except FileNotFoundError:
                os.mkdir(component, dir_fd=parent_fd)
                next_fd = os.open(component, directory_flags, dir_fd=parent_fd)
            os.close(parent_fd)
            parent_fd = next_fd
        os.mkdir(components[-1], dir_fd=parent_fd)
        root_fd = os.open(components[-1], directory_flags, dir_fd=parent_fd)
        try:
            metadata = os.fstat(root_fd)
            linked = os.stat(
                components[-1],
                dir_fd=parent_fd,
                follow_symlinks=False,
            )
            if (
                not stat.S_ISDIR(metadata.st_mode)
                or (metadata.st_dev, metadata.st_ino)
                != (linked.st_dev, linked.st_ino)
            ):
                raise ValueError("exclusive output root identity is invalid")
            identity = _directory_identity(metadata, output_root)
            if identity_sink is not None:
                identity_sink["root_identity"] = identity
            return identity
        finally:
            os.close(root_fd)
    finally:
        os.close(parent_fd)


def _assert_directory_identity(output_root: Path, identity: dict) -> None:
    try:
        _lstat_components(output_root, leaf_required=False)
        metadata = output_root.lstat()
    except (FileNotFoundError, OSError) as error:
        raise ValueError("allocated output_root path disappeared") from error
    if (
        not stat.S_ISDIR(metadata.st_mode)
        or _directory_identity(metadata, output_root) != identity
    ):
        raise ValueError("allocated output_root identity changed")


def _validate_protocol_shape(
    protocol: dict,
    *,
    protocol_path: Path,
    output_root: Path,
    run_seeds: tuple[int, ...],
    max_frames: int | None,
) -> str:
    if set(protocol) != PROTOCOL_TOP_LEVEL_FIELDS:
        raise ValueError("protocol top-level fields differ from exact contract")
    schema_id = protocol.get("schema_id")
    if schema_id not in {PROTOCOL_SCHEMA_ID, HERMETIC_PROTOCOL_SCHEMA_ID}:
        raise ValueError("protocol schema_id differs from exact contract")
    exact = {
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
    sources = protocol.get("sources")
    if not isinstance(sources, dict):
        raise ValueError("protocol sources must be an object")
    for name, declaration in sources.items():
        if (
            not isinstance(name, str)
            or not isinstance(declaration, dict)
            or set(declaration) != {"path", "sha256"}
            or not isinstance(declaration["path"], str)
            or not isinstance(declaration["sha256"], str)
            or re.fullmatch(r"[0-9a-f]{64}", declaration["sha256"]) is None
        ):
            raise ValueError(f"protocol source declaration is invalid: {name}")
    experiment = protocol.get("experiment")
    if not isinstance(experiment, dict):
        raise ValueError("protocol experiment must be an object")
    invocations = protocol.get("invocations")
    commands = protocol.get("commands")
    if not isinstance(invocations, dict) or not isinstance(commands, dict):
        raise ValueError("protocol invocations/commands are invalid")
    if schema_id == PROTOCOL_SCHEMA_ID:
        if protocol.get("protocol_id") != PROTOCOL_ID:
            raise ValueError("production protocol_id differs from exact contract")
        expected_protocol_path = (
            Path(__file__).resolve().parents[2] / PRODUCTION_PROTOCOL_TOKEN
        )
        if protocol_path != expected_protocol_path:
            raise ValueError("production protocol path differs from exact contract")
        expected_experiment = {
            **EXPERIMENT_CONTRACT,
            "range_noise_seeds": list(PRODUCTION_RANGE_NOISE_SEEDS),
            "max_frames": None,
        }
        if experiment != expected_experiment:
            raise ValueError("production experiment differs from exact contract")
        if set(sources) != REQUIRED_SOURCES:
            raise ValueError("production sources differ from exact contract")
        project_root = Path(__file__).resolve().parents[2]
        expected_paths = {
            "truth_data": PRODUCTION_TRUTH_DATA_PATH,
            "input_manifest": PRODUCTION_INPUT_MANIFEST_PATH,
            "baseline_process": PRODUCTION_BASELINE_PROCESS_PATH,
            "replay_source": str(Path(__file__).resolve()),
            "estimator_source": str(
                project_root / "scripts/diagnostics/predictive_wnls.py"
            ),
            "analyzer_source": str(
                project_root
                / "scripts/diagnostics/analyze_predictive_wnls_recovery.py"
            ),
            "legacy_solver_source": str(
                project_root
                / "scripts/diagnostics/replay_localization_calibration.py"
            ),
            "diagnostic_integrity_source": str(
                project_root / "scripts/diagnostics/run_diagnostic.py"
            ),
        }
        expected_hashes = {
            "truth_data": PRODUCTION_TRUTH_DATA_SHA256,
            "input_manifest": PRODUCTION_INPUT_MANIFEST_SHA256,
            "baseline_process": PRODUCTION_BASELINE_PROCESS_SHA256,
            "legacy_solver_source": LEGACY_SOLVER_SHA256,
        }
        for name, path in expected_paths.items():
            if sources[name]["path"] != path:
                raise ValueError(f"production source path differs: {name}")
        for name, digest in expected_hashes.items():
            if sources[name]["sha256"] != digest:
                raise ValueError(f"production source hash differs: {name}")
        expected_invocations = production_invocation_contract()
        if invocations != expected_invocations:
            raise ValueError("production invocations differ from exact contract")
        if commands != production_command_contract(sources):
            raise ValueError("production command argv differs from exact contract")
        replay_names = ("smoke_a", "smoke_b", "registered_replay")
    else:
        if protocol.get("protocol_id") != HERMETIC_PROTOCOL_ID:
            raise ValueError("hermetic protocol_id differs from exact contract")
        if set(sources) != HERMETIC_REQUIRED_SOURCES:
            raise ValueError("hermetic sources differ from exact contract")
        seeds = experiment.get("range_noise_seeds")
        frames = experiment.get("max_frames")
        expected_experiment = {
            **EXPERIMENT_CONTRACT,
            "evidence_class": "hermetic_non_evidence_only",
            "range_noise_seeds": seeds,
            "max_frames": frames,
        }
        if (
            experiment != expected_experiment
            or not isinstance(seeds, list)
            or not seeds
            or any(
                isinstance(seed, bool) or not isinstance(seed, int)
                for seed in seeds
            )
            or len(seeds) != len(set(seeds))
            or (
                frames is not None
                and (
                    isinstance(frames, bool)
                    or not isinstance(frames, int)
                    or frames <= 0
                )
            )
        ):
            raise ValueError("hermetic experiment differs from exact contract")
        expected_invocations = {
            "hermetic_replay": {
                "kind": "hermetic_non_evidence",
                "output_root": str(output_root),
                "range_noise_seeds": list(run_seeds),
                "max_frames": max_frames,
            }
        }
        if invocations != expected_invocations:
            raise ValueError("hermetic invocation differs from exact contract")
        expected_command = canonical_replay_argv(
            data_path=Path(sources["truth_data"]["path"]),
            input_manifest_path=Path(sources["input_manifest"]["path"]),
            protocol_token=str(protocol_path),
            output_root=output_root,
            run_seeds=run_seeds,
            max_frames=max_frames,
        )
        if commands != {"hermetic_replay": expected_command}:
            raise ValueError("hermetic command argv differs from exact contract")
        if seeds != list(run_seeds) or frames != max_frames:
            raise ValueError("hermetic runtime differs from exact experiment")
        replay_names = ("hermetic_replay",)
    selected = [
        name
        for name in replay_names
        if invocations[name]["output_root"] == str(output_root)
    ]
    if len(selected) != 1:
        raise ValueError("runtime output_root is not one exact replay invocation")
    declaration = invocations[selected[0]]
    if (
        declaration["range_noise_seeds"] != list(run_seeds)
        or declaration["max_frames"] != max_frames
    ):
        raise ValueError("runtime arguments differ from declared invocation")
    return selected[0]


def _verify_protocol_and_sources(
    *,
    protocol_path: Path,
    expected_protocol_identity: dict | None,
    expected_source_identities: dict | None = None,
    data_path: Path,
    input_manifest_path: Path,
    output_root: Path,
    run_seeds: tuple[int, ...],
    max_frames: int | None,
    output_allocated: bool = False,
) -> tuple[dict, dict, dict, str, dict[str, bytes]]:
    protocol_payload, protocol_identity = _read_trusted_bytes(
        protocol_path,
        expected_identity=expected_protocol_identity,
    )
    if protocol_payload is None:
        raise RuntimeError("internal protocol payload was not captured")
    protocol = _parse_json_object(protocol_payload, protocol_path)
    invocation = _validate_protocol_shape(
        protocol,
        protocol_path=protocol_path,
        output_root=output_root,
        run_seeds=run_seeds,
        max_frames=max_frames,
    )
    sources = protocol.get("sources")
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
        "analyzer_source": (
            project_root
            / "scripts/diagnostics/analyze_predictive_wnls_recovery.py"
        ),
    }
    identities = {}
    payloads = {}
    if expected_source_identities is not None and (
        set(expected_source_identities) != set(sources)
    ):
        raise ValueError("source identity set changed after initial read")
    for name, declaration in sources.items():
        declared_path = _absolute(Path(declaration["path"]))
        if name in actual_paths and declared_path != _absolute(actual_paths[name]):
            raise ValueError(f"protocol source path differs from runtime: {name}")
        payload, identity = _read_trusted_bytes(
            declared_path,
            expected_identity=(
                None
                if expected_source_identities is None
                else expected_source_identities[name]
            ),
            expected_sha256=declaration["sha256"],
            capture_payload=name in {"truth_data", "input_manifest"},
        )
        identities[name] = identity
        if payload is not None:
            payloads[name] = payload
    if identities["legacy_solver_source"]["sha256"] != LEGACY_SOLVER_SHA256:
        raise ValueError("legacy solver source differs from frozen identity")
    _validate_paths(
        project_root,
        protocol_path,
        [Path(item["path"]) for item in identities.values()],
        output_root,
        output_allocated=output_allocated,
    )
    return protocol, protocol_identity, identities, invocation, payloads


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
) -> tuple[
    dict,
    list[tuple[str, int]],
    dict[tuple[str, int], dict],
    dict[tuple[str, int], int],
]:
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
    noise_seeds = {}
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
        }
        noise_seeds[(kind, identifier)] = noise_seed
    return mandatory, optional, records, noise_seeds


def _valid_measurement_record(record: object) -> bool:
    if not isinstance(record, Mapping) or record.get("present") is not True:
        return False
    value = record.get("noisy_range")
    if isinstance(value, bool) or not isinstance(value, (int, float, np.number)):
        return False
    try:
        converted = float(value)
    except (TypeError, ValueError, OverflowError):
        return False
    return math.isfinite(converted) and converted >= 0.0


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
    required = {"innovation_gate", "q_innov", "gate_outcome"}
    if (
        not isinstance(diagnostics, dict)
        or not required <= set(diagnostics)
        or not set(diagnostics) <= set(GATE_DIAGNOSTIC_FIELDS)
    ):
        raise ValueError("candidate gate diagnostics differ from exact schema")
    return [diagnostics.get(field) for field in GATE_DIAGNOSTIC_FIELDS]


def _compact_candidates(candidates: object) -> list[list]:
    if not isinstance(candidates, list):
        raise ValueError("candidate evidence must be a list")
    result = []
    for candidate in candidates:
        if not isinstance(candidate, dict):
            raise ValueError("candidate evidence entry must be an object")
        if "result" in candidate:
            if set(candidate) != set(MULTISTART_CANDIDATE_RECORD_FIELDS):
                raise ValueError("multistart candidate differs from exact schema")
            nested = candidate["result"]
            if not isinstance(nested, dict) or set(nested) != set(
                SOLVER_RESULT_FIELDS
            ):
                raise ValueError("solver result differs from exact schema")
            for field in ("status", "estimate", "covariance", "cost"):
                if candidate[field] != nested[field]:
                    raise ValueError("candidate summary differs from solver result")
            trace = nested["proposal_trace"]
        else:
            if set(candidate) != set(CANDIDATE_FIELDS):
                raise ValueError("legacy candidate differs from exact schema")
            nested = candidate
            trace = candidate["proposal_trace"]
        if not isinstance(trace, list):
            raise ValueError("proposal trace must be a list")
        compact_trace = []
        for proposal in trace:
            if not isinstance(proposal, dict) or set(proposal) != set(
                PROPOSAL_TRACE_FIELDS
            ):
                raise ValueError("proposal trace entry differs from exact schema")
            compact_trace.append(
                [proposal[field] for field in PROPOSAL_TRACE_FIELDS]
            )
        result.append(
            [
                candidate["source"],
                candidate["initial_estimate"],
                candidate["status"],
                candidate["estimate"],
                candidate["covariance"],
                candidate["cost"],
                candidate["accepted"],
                candidate["rejection_reason"],
                candidate["q_innov"],
                _compact_gate(candidate["gate_diagnostics"]),
                compact_trace,
            ]
        )
    return result


def _reference_evidence(
    *,
    mandatory: dict,
    optional: list[tuple[str, int]],
    records: dict[tuple[str, int], dict],
    noise_seeds: dict[tuple[str, int], int],
    qualification: dict,
    current_public: dict[int, dict],
    solver_used_keys: tuple[tuple[str, int], ...],
    solver_exclusion_reason: str | None = None,
) -> list[list]:
    mandatory_keys = {
        ("base", identifier) for identifier in mandatory["base_ids"]
    } | {("uav", identifier) for identifier in mandatory["uav_ids"]}
    all_keys = sorted(
        mandatory_keys | set(optional),
        key=lambda key: (0 if key[0] == "base" else 1, key[1]),
    )
    used = set(solver_used_keys)
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
    qualification_failed = qualification.get("status") != "ok"
    for key in all_keys:
        record = records.get(key, {"present": False, "noisy_range": None})
        valid_measurement = _valid_measurement_record(record)
        if key[0] == "base":
            freshness = "fresh"
            eligible = valid_measurement
            provenance = [key[1]]
        else:
            output = current_public.get(key[1])
            freshness = (
                output.get("output_status")
                if isinstance(output, dict)
                else "missing"
            )
            eligible = valid_measurement and reference_is_eligible(output)
            provenance = (
                list(output.get("base_anchor_provenance", []))
                if isinstance(output, dict)
                else []
            )
        reason = excluded.get(key)
        if key not in used and reason is None:
            if not valid_measurement:
                reason = "measurement_not_present"
            elif qualification_failed:
                reason = "not_evaluated_due_to_missing_mandatory"
            elif solver_exclusion_reason is not None:
                reason = solver_exclusion_reason
        evidence.append(
            [
                key[0],
                key[1],
                "mandatory" if key in mandatory_keys else "optional",
                bool(record.get("present")),
                record.get("noisy_range"),
                noise_seeds.get(key),
                freshness,
                eligible,
                key in used,
                reason,
                provenance,
            ]
        )
    return evidence


def _reference_freshness(evidence: list[list]) -> list[list]:
    decoded = [
        dict(zip(REFERENCE_EVIDENCE_FIELDS, item))
        for item in evidence
    ]
    return [
        [
            item["reference_kind"],
            item["reference_id"],
            item["current_freshness"],
        ]
        for item in decoded
    ]


def _normalize_reason_records(
    records: object,
    *,
    fields: tuple[str, ...],
) -> list[list]:
    if not isinstance(records, list):
        raise ValueError("reference reason records must be a list")
    normalized = []
    for record in records:
        if (
            not isinstance(record, dict)
            or set(record) != {"key", "reason"}
            or not isinstance(record["key"], (list, tuple))
            or len(record["key"]) != 2
        ):
            raise ValueError("reference reason record differs from exact schema")
        kind, identifier = record["key"]
        normalized.append([kind, identifier, record["reason"]])
    if len(fields) != 3:
        raise RuntimeError("internal reason-field declaration is invalid")
    return normalized


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


def _write_stage(path: Path, payload: bytes) -> dict:
    flags = (
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptor = os.open(path, flags, 0o600)
    with os.fdopen(descriptor, "wb") as target:
        target.write(payload)
        target.flush()
        os.fsync(target.fileno())
        metadata = os.fstat(target.fileno())
    return {
        "path": str(path),
        "device": metadata.st_dev,
        "inode": metadata.st_ino,
        "size": metadata.st_size,
        "sha256": hashlib.sha256(payload).hexdigest(),
    }


def _owned_stage_matches(stage: dict) -> bool:
    try:
        _, identity = _read_trusted_bytes(Path(stage["path"]))
    except (OSError, ValueError):
        return False
    return all(
        identity[field] == stage[field]
        for field in ("device", "inode", "size", "sha256")
    )


def _cleanup_stage(stage: dict) -> None:
    if not _owned_stage_matches(stage):
        raise ValueError("refusing to remove an unowned staging path")
    Path(stage["path"]).unlink()


def _link_stage(stage: dict, target: Path) -> None:
    if not _owned_stage_matches(stage):
        raise ValueError("refusing to publish an unowned staging path")
    if Path(stage["path"]).parent.stat().st_dev != target.parent.stat().st_dev:
        raise ValueError("terminal stage is not on the target filesystem")
    os.link(stage["path"], target, follow_symlinks=False)


def _target_matches_stage(target: Path, stage: dict) -> bool:
    try:
        _, identity = _read_trusted_bytes(target, capture_payload=False)
    except (OSError, ValueError):
        return False
    return (
        identity["device"] == stage["device"]
        and identity["inode"] == stage["inode"]
        and identity["size"] == stage["size"]
        and identity["sha256"] == stage["sha256"]
    )


def _fsync_output_directory(output_root: Path) -> None:
    flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptor = os.open(output_root, flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _fsync_raw_output(process_path: Path) -> None:
    descriptor = os.open(
        process_path,
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0),
    )
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _required_terminal_metrics(output_root: Path) -> dict:
    return {
        "free_bytes_after": available_bytes(output_root),
        "allocated_bytes": allocated_bytes(output_root),
    }


def _failure_terminal_metrics(output_root: Path) -> dict:
    values = {}
    for name, function in (
        ("free_bytes_after", available_bytes),
        ("allocated_bytes", allocated_bytes),
    ):
        try:
            values[name] = function(output_root)
        except Exception:
            values[name] = None
    return values


def _publish_failed(output_root: Path, manifest: dict) -> None:
    target = output_root / TERMINAL_MANIFEST_NAME
    if target.exists() or target.is_symlink():
        raise FileExistsError("terminal manifest publication conflict")
    payload = _strict_json_bytes(manifest, indent=2) + b"\n"
    stage = _write_stage(_stage_path(output_root, "failed"), payload)
    try:
        _link_stage(stage, target)
        _fsync_output_directory(output_root)
    finally:
        try:
            _cleanup_stage(stage)
        except Exception:
            pass


def _rollback_owned_target(target: Path, stage: dict) -> None:
    if _target_matches_stage(target, stage):
        target.unlink()
        _fsync_output_directory(target.parent)


def _final_identity_and_disk_probe(
    *,
    protocol_path: Path,
    protocol_identity: dict,
    source_identities: dict,
    data_path: Path,
    input_manifest_path: Path,
    output_root: Path,
    run_seeds: tuple[int, ...],
    max_frames: int | None,
) -> tuple[dict, dict, str, dict]:
    _, observed_protocol, identities, invocation, _ = _verify_protocol_and_sources(
        protocol_path=protocol_path,
        expected_protocol_identity=protocol_identity,
        expected_source_identities=source_identities,
        data_path=data_path,
        input_manifest_path=input_manifest_path,
        output_root=output_root,
        run_seeds=run_seeds,
        max_frames=max_frames,
        output_allocated=True,
    )
    metrics = _required_terminal_metrics(output_root)
    if metrics["free_bytes_after"] < HARD_FLOOR_BYTES:
        raise DiskSpaceError("available bytes below live floor")
    if metrics["allocated_bytes"] > RAW_BUNDLE_CAP_BYTES:
        raise DiskSpaceError("raw bundle exceeds allocated-byte cap")
    return observed_protocol, identities, invocation, metrics


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
    data_path = _runtime_absolute(Path(data_path))
    input_manifest_path = _runtime_absolute(Path(input_manifest_path))
    protocol_path = _runtime_absolute(Path(protocol_path))
    output_root = _runtime_absolute(Path(output_root))
    if (
        not run_seeds
        or any(isinstance(seed, bool) or not isinstance(seed, int) for seed in run_seeds)
        or len(set(run_seeds)) != len(run_seeds)
    ):
        raise ValueError("run_seeds must be unique non-empty integers")
    protocol, protocol_identity, source_identities, invocation, source_payloads = (
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
    data = _parse_json_object(source_payloads["truth_data"], data_path)
    input_manifest = _parse_json_object(
        source_payloads["input_manifest"], input_manifest_path
    )
    if input_manifest.get("termination_reason") != "completed":
        raise ValueError("input manifest is not completed")
    # Pin every full descriptor identity immediately after parsing the exact
    # bytes read during the first verification pass.
    protocol, protocol_identity, source_identities, invocation, _ = (
        _verify_protocol_and_sources(
            protocol_path=protocol_path,
            expected_protocol_identity=protocol_identity,
            expected_source_identities=source_identities,
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
    started_at = datetime.now(timezone.utc).isoformat()
    process_path = output_root / RAW_PROCESS_NAME
    rows_written = 0
    decompressed_digest = hashlib.sha256()
    root_identity = None

    def terminal(
        status: str,
        *,
        metrics: dict,
        process_hash: str | None,
        error: BaseException | None = None,
    ) -> dict:
        value = {
            "status": status,
            "schema_id": RAW_SCHEMA_ID,
            "raw_schema": RAW_SCHEMA_DECLARATION,
            "protocol_schema_id": protocol["schema_id"],
            "protocol_id": protocol["protocol_id"],
            "evidence_class": protocol["experiment"]["evidence_class"],
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
            "free_bytes_after": metrics["free_bytes_after"],
            "allocated_bytes": metrics["allocated_bytes"],
            "raw_bundle_cap_bytes": RAW_BUNDLE_CAP_BYTES,
            "compressed_process_sha256": process_hash,
            "decompressed_process_sha256": (
                decompressed_digest.hexdigest() if process_hash is not None else None
            ),
        }
        if error is not None:
            value["error"] = {
                "type": type(error).__name__,
                "message": str(error),
            }
        return value

    creation_state: dict = {}
    try:
        root_identity = _create_exact_root(
            output_root,
            identity_sink=creation_state,
        )
    except BaseException as error:
        try:
            root_identity = creation_state.get("root_identity")
            if root_identity is not None:
                _assert_directory_identity(output_root, root_identity)
                failure = terminal(
                    "failed",
                    metrics=_failure_terminal_metrics(output_root),
                    process_hash=None,
                    error=error,
                )
                _publish_failed(output_root, failure)
        except Exception:
            pass
        raise

    try:
        _assert_directory_identity(output_root, root_identity)
        if available_bytes(output_root) < HARD_FLOOR_BYTES:
            raise DiskSpaceError("available bytes below live floor")
        states: dict[tuple[str, int], dict[int, dict]] = {}
        _assert_directory_identity(output_root, root_identity)
        descriptor = os.open(
            process_path,
            os.O_WRONLY
            | os.O_CREAT
            | os.O_EXCL
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NOFOLLOW", 0),
            0o600,
        )
        with os.fdopen(descriptor, "wb") as raw, gzip.GzipFile(
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
                            mandatory, optional, records, noise_seeds = _sensor_records(
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
                            solver_used_keys: tuple[tuple[str, int], ...] = ()
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
                                solver_used_keys = tuple(keys)
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
                            if legacy_result is not None:
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
                                noise_seeds=noise_seeds,
                                qualification=qualification,
                                current_public={
                                    key: value
                                    for key, value in current_public.items()
                                    if key != robot_id
                                },
                                solver_used_keys=solver_used_keys,
                                solver_exclusion_reason=(
                                    "not_supplied_due_to_reference_state_unavailable"
                                    if arrays is None
                                    else None
                                ),
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
                                "mandatory_references": {
                                    field: list(mandatory[field])
                                    for field in MANDATORY_REFERENCE_FIELDS
                                },
                                "optional_candidates": [
                                    [kind, identifier]
                                    for kind, identifier in optional
                                ],
                                "active_references": [
                                    list(record["key"])
                                    for record in active_records
                                ],
                                "reference_evidence": reference_evidence,
                                "reference_freshness": _reference_freshness(
                                    reference_evidence
                                ),
                                "excluded_references": _normalize_reason_records(
                                    qualification.get("excluded", []),
                                    fields=EXCLUSION_FIELDS,
                                ),
                                "reference_violations": _normalize_reason_records(
                                    qualification.get("violations", []),
                                    fields=VIOLATION_FIELDS,
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
        _fsync_raw_output(process_path)
        _assert_directory_identity(output_root, root_identity)
        process_hash = _file_identity(process_path)["sha256"]
        finalizing_metrics = _required_terminal_metrics(output_root)
        if finalizing_metrics["free_bytes_after"] < HARD_FLOOR_BYTES:
            raise DiskSpaceError("available bytes below live floor")
        if finalizing_metrics["allocated_bytes"] > RAW_BUNDLE_CAP_BYTES:
            raise DiskSpaceError("raw bundle exceeds allocated-byte cap")
        finalizing_stage = _write_stage(
            _stage_path(output_root, "finalizing"),
            _strict_json_bytes(
                terminal(
                    "finalizing",
                    metrics=finalizing_metrics,
                    process_hash=process_hash,
                ),
                indent=2,
            )
            + b"\n",
        )
        (
            observed_protocol,
            observed_sources,
            observed_invocation,
            completed_metrics,
        ) = (
            _final_identity_and_disk_probe(
                protocol_path=protocol_path,
                protocol_identity=protocol_identity,
                source_identities=source_identities,
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
        finalizing_stage = None
        completed = terminal(
            "completed",
            metrics=completed_metrics,
            process_hash=process_hash,
        )
        completed_stage = _write_stage(
            _stage_path(output_root, "completed"),
            _strict_json_bytes(completed, indent=2) + b"\n"
        )
        target = output_root / TERMINAL_MANIFEST_NAME
        _assert_directory_identity(output_root, root_identity)
        try:
            _link_stage(completed_stage, target)
        except BaseException:
            if _target_matches_stage(target, completed_stage):
                try:
                    _cleanup_stage(completed_stage)
                    _fsync_output_directory(output_root)
                except BaseException:
                    _rollback_owned_target(target, completed_stage)
                    raise
                return completed
            try:
                _cleanup_stage(completed_stage)
            except Exception:
                pass
            raise
        try:
            _cleanup_stage(completed_stage)
            _fsync_output_directory(output_root)
        except BaseException:
            _rollback_owned_target(target, completed_stage)
            try:
                _cleanup_stage(completed_stage)
            except Exception:
                pass
            raise
        return completed
    except BaseException as error:
        for stage in (locals().get("finalizing_stage"), locals().get("completed_stage")):
            if isinstance(stage, dict):
                try:
                    _cleanup_stage(stage)
                except Exception:
                    pass
        try:
            _assert_directory_identity(output_root, root_identity)
            failed_process_hash = (
                _file_identity(process_path)["sha256"]
                if process_path.exists()
                else None
            )
            failure = terminal(
                "failed",
                metrics=_failure_terminal_metrics(output_root),
                process_hash=failed_process_hash,
                error=error,
            )
            _publish_failed(output_root, failure)
        except Exception:
            pass
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
