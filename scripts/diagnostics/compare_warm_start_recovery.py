"""Compare strict and pre-acquisition-restart localization replay policies."""

from __future__ import annotations

import argparse
import ctypes
import errno
import gzip
import hashlib
import json
import math
import os
import sys
import tempfile
from itertools import zip_longest
from pathlib import Path
from typing import Callable

import numpy as np
import scripts.diagnostics.analyze_localization_failures as failure_analyzer
import scripts.diagnostics.replay_localization_calibration as replay_module
from scripts.diagnostics.analyze_localization_failures import (
    AnalysisLimitError,
    InputIntegrityError,
    analyze_localization_failures,
)
from scripts.diagnostics.replay_localization_calibration import (
    ESTIMATOR_CONTRACT_ID,
    RESTART_BEFORE_FIRST_FINITE_POLICY,
    STRICT_PREVIOUS_POLICY,
)
from scripts.diagnostics.run_warm_start_recovery import PARENT_SCHEMA_ID
from scripts.diagnostics.run_diagnostic import (
    HARD_FLOOR_BYTES,
    OUTPUT_ROOT_CAP_BYTES,
    START_BYTES,
    DiskSpaceError,
    allocated_bytes,
    available_bytes,
    require_start_space,
)


SCHEMA_ID = "cbf2026-warm-start-recovery-comparison-v1"
BOOTSTRAP_RESAMPLES = 10_000
BOOTSTRAP_SEED = 20260729
PROSPECTIVE_FIELDS = frozenset(
    {
        "initialization_policy",
        "initial_estimate_source",
        "ever_acquired_finite_before_attempt",
    }
)
_BUNDLE_FILES = (
    "manifest.json",
    "summary.json",
    "summary.md",
    "calibration.jsonl.gz",
)
_PROCESS_NAME = "calibration.jsonl.gz"
_SUMMARY_NAME = "summary.json"
_SUMMARY_MARKDOWN_NAME = "summary.md"
_OUTPUT_JSON_NAME = "warm-start-recovery-comparison.json"
_OUTPUT_MARKDOWN_NAME = "warm-start-recovery-comparison.md"
_ANALYSIS_OUTPUT_CAP_BYTES = 10_000_000
_LIVE_CHECK_INTERVAL_ROWS = 10_000
_GRAPH_CASES = ("dynamic_dag_wnls", "fixed_refs_wnls")
_FROZEN_SEEDS = tuple(range(20260727, 20260747))
_EXACT_DIRECT_REASON = "non-finite or malformed WNLS input"
_UPSTREAM_REASON = "invalid upstream UAV reference"
_WNLS_REASON = "maximum WNLS iterations exceeded"
_INITIAL_SOURCE_LABELS = {
    "deployment_frame_zero",
    "previous_finite",
    "strict_previous_missing",
    "deployment_restart_before_first_finite",
}
_PRE_WNLS_SHORT_CIRCUIT_REASONS = {
    "invalid upstream UAV reference",
    "invalid reference truth",
    "fewer than two active references",
}
_MISSING = object()


def _validate_json_tree(value: object, description: str) -> None:
    value_type = type(value)
    if value_type is float:
        if not math.isfinite(value):
            raise InputIntegrityError(
                f"{description} contains a non-finite number"
            )
        return
    if value_type in (type(None), bool, int, str):
        return
    if value_type is list:
        for index, item in enumerate(value):
            _validate_json_tree(item, f"{description}[{index}]")
        return
    if value_type is dict:
        for key, item in value.items():
            if type(key) is not str:
                raise InputIntegrityError(
                    f"{description} contains a non-string object key"
                )
            _validate_json_tree(item, f"{description}.{key}")
        return
    raise InputIntegrityError(
        f"{description} contains a non-JSON value of type "
        f"{value_type.__name__}"
    )


def _json_type_exact_equal(first: object, second: object) -> bool:
    if type(first) is not type(second):
        return False
    if type(first) is dict:
        return (
            set(first) == set(second)
            and all(
                _json_type_exact_equal(first[key], second[key])
                for key in first
            )
        )
    if type(first) is list:
        return len(first) == len(second) and all(
            _json_type_exact_equal(left, right)
            for left, right in zip(first, second)
        )
    return first == second


def _strict_object(path: Path, description: str) -> tuple[dict, bytes]:
    try:
        raw = path.read_bytes()
        value = json.loads(
            raw,
            parse_constant=lambda token: (_ for _ in ()).throw(
                ValueError(f"non-finite JSON token: {token}")
            ),
        )
    except (OSError, ValueError) as error:
        raise InputIntegrityError(f"invalid {description}: {error}") from error
    if not isinstance(value, dict):
        raise InputIntegrityError(f"{description} must be a JSON object")
    _validate_json_tree(value, description)
    return value, raw


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as source:
            for chunk in iter(lambda: source.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as error:
        raise InputIntegrityError(f"cannot hash {path.name}: {error}") from error
    return digest.hexdigest()


def _expected_hash(value: object, description: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise InputIntegrityError(f"{description} must be a lowercase SHA-256")
    return value


def _verify_external_hash(
    path: Path,
    expected: object,
    description: str,
) -> str:
    expected_hash = _expected_hash(expected, f"expected {description} hash")
    observed = _sha256(path)
    if observed != expected_hash:
        raise InputIntegrityError(
            f"{description} does not match its external trust root"
        )
    return expected_hash


def _regular_file(path: Path, description: str) -> None:
    if path.is_symlink() or not path.is_file():
        raise InputIntegrityError(f"{description} must be a regular file")


def _bundle_hashes(bundle_dir: Path) -> dict[str, str]:
    hashes = {}
    for name in _BUNDLE_FILES:
        path = bundle_dir / name
        _regular_file(path, f"{bundle_dir.name} {name}")
        hashes[name] = _sha256(path)
    return hashes


def _verify_declared_bundle_hashes(
    bundle_dir: Path,
    manifest: dict,
    hashes: dict[str, str],
    *,
    verify_hashes: bool,
) -> None:
    if not verify_hashes:
        return
    declared = {
        "summary.json": manifest.get("summary_json_sha256"),
        "summary.md": manifest.get("summary_markdown_sha256"),
        "calibration.jsonl.gz": manifest.get("compressed_process_sha256"),
    }
    for name, raw_expected in declared.items():
        expected = _expected_hash(raw_expected, f"{bundle_dir.name} {name} hash")
        if hashes[name] != expected:
            raise InputIntegrityError(
                f"{bundle_dir.name} {name} hash does not match manifest"
            )


def _load_bundle(
    bundle_dir: Path,
    *,
    verify_hashes: bool,
) -> dict:
    bundle_dir = Path(bundle_dir)
    if bundle_dir.is_symlink() or not bundle_dir.is_dir():
        raise InputIntegrityError(f"{bundle_dir} must be a real bundle directory")
    hashes = _bundle_hashes(bundle_dir)
    manifest, manifest_raw = _strict_object(
        bundle_dir / "manifest.json", f"{bundle_dir.name} manifest"
    )
    summary, _ = _strict_object(
        bundle_dir / _SUMMARY_NAME, f"{bundle_dir.name} summary"
    )
    if manifest.get("termination_reason") != "completed":
        raise InputIntegrityError(f"{bundle_dir.name} manifest is not completed")
    if manifest.get("estimator_contract") != ESTIMATOR_CONTRACT_ID:
        raise InputIntegrityError(
            f"{bundle_dir.name} estimator contract does not match"
        )
    if not _json_type_exact_equal(
        manifest.get("settings"), summary.get("settings")
    ):
        raise InputIntegrityError(
            f"{bundle_dir.name} manifest and summary settings differ"
        )
    _verify_declared_bundle_hashes(
        bundle_dir, manifest, hashes, verify_hashes=verify_hashes
    )
    stream_manifest = dict(manifest)
    stream_manifest["_verify_hashes"] = verify_hashes
    return {
        "dir": bundle_dir,
        "manifest": manifest,
        "manifest_raw": manifest_raw,
        "summary": summary,
        "hashes": hashes,
        "stream_manifest": stream_manifest,
    }


def _compact_key(row: dict) -> tuple[int, int, str, int]:
    try:
        return (
            row["frame_index"],
            row["seed"],
            row["graph_case"],
            row["robot_id"],
        )
    except KeyError as error:
        raise InputIntegrityError(
            f"row is missing compact key field {error.args[0]}"
        ) from error


def _normalized(row: dict) -> dict:
    return {
        key: value
        for key, value in row.items()
        if key not in PROSPECTIVE_FIELDS
    }


def _verified_rows(
    bundle: dict,
    *,
    live_guard: Callable[[], None] | None = None,
):
    for row in failure_analyzer._iter_verified_rows(
        bundle["dir"],
        bundle["stream_manifest"],
        bundle["summary"],
        live_guard=live_guard,
    ):
        _validate_json_tree(
            row,
            f"{bundle['dir'].name} process row {_compact_key(row)!r}",
        )
        yield row


def _verify_bundle_unchanged(bundle: dict) -> None:
    current = _bundle_hashes(bundle["dir"])
    if current != bundle["hashes"]:
        raise InputIntegrityError(f"{bundle['dir'].name} bundle changed during analysis")


def _strict_anchor(
    baseline: dict,
    strict: dict,
    *,
    live_guard: Callable[[], None] | None = None,
) -> dict:
    rows_compared = 0
    pairs = zip_longest(
        _verified_rows(baseline, live_guard=live_guard),
        _verified_rows(strict, live_guard=live_guard),
        fillvalue=_MISSING,
    )
    for baseline_row, strict_row in pairs:
        if baseline_row is _MISSING or strict_row is _MISSING:
            raise InputIntegrityError(
                "strict and immutable streams have different cardinality"
            )
        baseline_key = _compact_key(baseline_row)
        strict_key = _compact_key(strict_row)
        if baseline_key != strict_key:
            raise InputIntegrityError(
                "strict and immutable row key mismatch at "
                f"{baseline_key!r}/{strict_key!r}"
            )
        if not _json_type_exact_equal(
            _normalized(baseline_row), _normalized(strict_row)
        ):
            raise InputIntegrityError(
                f"strict scientific row drift at key {baseline_key!r}"
            )
        rows_compared += 1
    _verify_bundle_unchanged(baseline)
    _verify_bundle_unchanged(strict)
    return {
        "rows_compared": rows_compared,
        "normalized_rows_equal": True,
        "immutable_hashes_unchanged": True,
    }


def _path_record(value: object, description: str) -> tuple[Path, str]:
    if not isinstance(value, dict):
        raise InputIntegrityError(f"{description} record must be an object")
    raw_path = value.get("path")
    if not isinstance(raw_path, str) or not raw_path:
        raise InputIntegrityError(f"{description} path must be a non-empty string")
    path = Path(raw_path)
    _regular_file(path, description)
    expected = _expected_hash(value.get("sha256"), f"{description} hash")
    if _sha256(path) != expected:
        raise InputIntegrityError(f"{description} hash does not match")
    return path, expected


def _parent_inputs(parent: dict) -> dict[str, tuple[Path, str]]:
    source_snapshot = parent.get("source_snapshot_path")
    if not isinstance(source_snapshot, str) or not source_snapshot:
        raise InputIntegrityError("parent source snapshot path is invalid")
    snapshot_path = Path(source_snapshot)
    _regular_file(snapshot_path, "source snapshot")
    snapshot_hash = _expected_hash(
        parent.get("source_snapshot_sha256"), "source snapshot hash"
    )
    if _sha256(snapshot_path) != snapshot_hash:
        raise InputIntegrityError("source snapshot hash does not match")
    replay_path, replay_hash = _path_record(
        parent.get("replay_implementation"), "replay implementation"
    )
    data_path, data_hash = _path_record(parent.get("input_data"), "input data")
    input_manifest_path, input_manifest_hash = _path_record(
        parent.get("input_manifest"), "input manifest"
    )
    return {
        "source_snapshot": (snapshot_path, snapshot_hash),
        "replay_implementation": (replay_path, replay_hash),
        "input_data": (data_path, data_hash),
        "input_manifest": (input_manifest_path, input_manifest_hash),
    }


def _verify_parent_inputs_unchanged(
    inputs: dict[str, tuple[Path, str]],
) -> None:
    for description, (path, expected) in inputs.items():
        if _sha256(path) != expected:
            raise InputIntegrityError(f"{description} changed during analysis")


def _require_policy(child: dict, expected: str, label: str) -> None:
    manifest = child["manifest"]
    if manifest.get("initialization_policy") != expected:
        raise InputIntegrityError(f"{label} child policy is not frozen")
    settings = manifest.get("settings")
    if not isinstance(settings, dict) or settings.get("initialization_policy") != expected:
        raise InputIntegrityError(f"{label} child settings policy is not frozen")


def _require_dimensions(
    parent: dict,
    baseline: dict,
    strict: dict,
    restart: dict,
) -> list[int]:
    seeds = parent.get("seeds")
    if not _json_type_exact_equal(seeds, list(_FROZEN_SEEDS)):
        raise InputIntegrityError(
            "parent seeds must equal the frozen 20260727..20260746 list"
        )
    if type(parent.get("max_frames")) is not int or parent["max_frames"] <= 0:
        raise InputIntegrityError("parent max_frames must be a positive integer")
    for label, bundle in (
        ("immutable", baseline),
        ("strict", strict),
        ("restart", restart),
    ):
        settings = bundle["manifest"].get("settings")
        if not isinstance(settings, dict):
            raise InputIntegrityError(f"{label} settings must be an object")
        if not _json_type_exact_equal(settings.get("run_seeds"), seeds):
            raise InputIntegrityError(f"{label} seeds do not match parent")
        if not _json_type_exact_equal(
            settings.get("graph_cases"), list(_GRAPH_CASES)
        ):
            raise InputIntegrityError(
                f"{label} graph cases do not match the frozen order"
            )
        if (
            type(settings.get("effective_frame_count")) is not int
            or settings["effective_frame_count"] != parent["max_frames"]
        ):
            raise InputIntegrityError(
                f"{label} frame count does not match parent max_frames"
            )
    strict_settings = {
        key: value
        for key, value in strict["manifest"]["settings"].items()
        if key != "initialization_policy"
    }
    restart_settings = {
        key: value
        for key, value in restart["manifest"]["settings"].items()
        if key != "initialization_policy"
    }
    if not _json_type_exact_equal(strict_settings, restart_settings):
        raise InputIntegrityError(
            "strict and restart settings differ beyond initialization policy"
        )
    return seeds


def _require_child_sources(
    parent: dict,
    strict: dict,
    restart: dict,
) -> None:
    for label, bundle in (("strict", strict), ("restart", restart)):
        manifest = bundle["manifest"]
        for field in ("input_data", "input_manifest"):
            if not _json_type_exact_equal(
                manifest.get(field), parent.get(field)
            ):
                raise InputIntegrityError(
                    f"{label} {field} does not match paired parent"
                )
        settings = manifest.get("settings")
        implementation = (
            settings.get("implementation")
            if isinstance(settings, dict)
            else None
        )
        parent_implementation = parent.get("replay_implementation")
        if (
            not isinstance(implementation, dict)
            or not isinstance(parent_implementation, dict)
            or {
                "path": implementation.get("path"),
                "sha256": implementation.get("sha256"),
            }
            != parent_implementation
        ):
            raise InputIntegrityError(
                f"{label} replay implementation does not match parent"
            )
    if not _json_type_exact_equal(
        strict["manifest"].get("input_data"),
        restart["manifest"].get("input_data"),
    ):
        raise InputIntegrityError("child input data records differ")
    if not _json_type_exact_equal(
        strict["manifest"].get("input_manifest"),
        restart["manifest"].get("input_manifest"),
    ):
        raise InputIntegrityError("child input manifest records differ")


def _measurement_noise_identity(row: dict) -> list[dict]:
    measurements = row.get("measurements")
    if not isinstance(measurements, list):
        raise InputIntegrityError(
            f"paired measurements must be a list at key {_compact_key(row)!r}"
        )
    identity = []
    for measurement in measurements:
        if type(measurement) is not dict:
            raise InputIntegrityError(
                f"paired measurement must be an object at key {_compact_key(row)!r}"
            )
        required = {
            "kind",
            "id",
            "true_range",
            "noise",
            "noisy_range",
            "estimated_reference_available",
        }
        if not required <= set(measurement):
            raise InputIntegrityError(
                f"paired measurement fields are incomplete at key "
                f"{_compact_key(row)!r}"
            )
        kind = measurement["kind"]
        reference_id = measurement["id"]
        true_range = measurement["true_range"]
        noise = measurement["noise"]
        noisy_range = measurement["noisy_range"]
        available = measurement["estimated_reference_available"]
        if kind not in {"base", "uav"} or type(kind) is not str:
            raise InputIntegrityError(
                f"paired measurement kind is invalid at key "
                f"{_compact_key(row)!r}"
            )
        if type(reference_id) is not int or reference_id < 0:
            raise InputIntegrityError(
                f"paired measurement ID is invalid at key "
                f"{_compact_key(row)!r}"
            )
        if type(noise) is not float or not math.isfinite(noise):
            raise InputIntegrityError(
                f"paired measurement noise is invalid at key "
                f"{_compact_key(row)!r}"
            )
        if type(available) is not bool:
            raise InputIntegrityError(
                f"paired measurement availability is invalid at key "
                f"{_compact_key(row)!r}"
            )
        if true_range is None:
            if noisy_range is not None:
                raise InputIntegrityError(
                    f"paired measurement range semantics are invalid at key "
                    f"{_compact_key(row)!r}"
                )
        elif (
            type(true_range) is not float
            or not math.isfinite(true_range)
            or type(noisy_range) is not float
            or not math.isfinite(noisy_range)
            or noisy_range != true_range + noise
        ):
            raise InputIntegrityError(
                f"paired measurement range/noise values are invalid at key "
                f"{_compact_key(row)!r}"
            )
        if (
            kind == "base"
            and available is not (true_range is not None)
        ):
            raise InputIntegrityError(
                f"base availability is invalid at key {_compact_key(row)!r}"
            )
        identity.append(
            {
                "kind": kind,
                "id": reference_id,
                "true_range": true_range,
                "noise": noise,
                "noisy_range": noisy_range,
            }
        )
    return identity


def _reconcile_replay_row_inputs(row: dict) -> dict:
    key = _compact_key(row)
    if type(row.get("squad_local_index")) is not int:
        raise InputIntegrityError(
            f"paired squad-local index is invalid at key {key!r}"
        )
    references = row.get("active_references")
    if type(references) is not dict or set(references) != {
        "base_ids",
        "uav_ids",
    }:
        raise InputIntegrityError(
            f"paired active references are invalid at key {key!r}"
        )
    for family in ("base_ids", "uav_ids"):
        identifiers = references[family]
        if type(identifiers) is not list or any(
            type(identifier) is not int or identifier < 0
            for identifier in identifiers
        ):
            raise InputIntegrityError(
                f"paired {family} are invalid at key {key!r}"
            )
        if identifiers != sorted(identifiers) or len(identifiers) != len(
            set(identifiers)
        ):
            raise InputIntegrityError(
                f"paired {family} must be sorted and unique at key {key!r}"
            )
    truth = row.get("truth_position")
    if (
        type(truth) is not list
        or len(truth) != 2
        or any(
            type(value) is not float or not math.isfinite(value)
            for value in truth
        )
    ):
        raise InputIntegrityError(
            f"paired truth position is invalid at key {key!r}"
        )
    measurements = row.get("measurements")
    _measurement_noise_identity(row)
    expected_edges = [
        ("base", reference_id)
        for reference_id in references["base_ids"]
    ] + [
        ("uav", reference_id)
        for reference_id in references["uav_ids"]
    ]
    observed_edges = [
        (measurement["kind"], measurement["id"])
        for measurement in measurements
    ]
    if not _json_type_exact_equal(observed_edges, expected_edges):
        raise InputIntegrityError(
            f"measurement order/IDs do not match active references at key "
            f"{key!r}"
        )
    pre_wnls_reason = None
    for measurement in measurements:
        if measurement["true_range"] is None:
            pre_wnls_reason = "invalid reference truth"
            break
        if measurement["estimated_reference_available"] is False:
            pre_wnls_reason = "invalid upstream UAV reference"
            break
    if pre_wnls_reason is None and len(measurements) < 2:
        pre_wnls_reason = "fewer than two active references"
    if pre_wnls_reason is not None:
        if (
            row.get("attempt_status") != "invalid"
            or row.get("attempt_failure_reason") != pre_wnls_reason
        ):
            raise InputIntegrityError(
                f"pre-WNLS outcome does not reconcile at key {key!r}"
            )
        return {
            "wnls_invoked": False,
            "short_circuit_reason": pre_wnls_reason,
        }
    if (
        row.get("attempt_status") == "invalid"
        and row.get("attempt_failure_reason")
        in _PRE_WNLS_SHORT_CIRCUIT_REASONS
    ):
        raise InputIntegrityError(
            f"invoked WNLS row reports a pre-WNLS reason at key {key!r}"
        )
    return {"wnls_invoked": True, "short_circuit_reason": None}


def _paired_inputs_equal(strict_row: dict, restart_row: dict) -> bool:
    fields = (
        "seed",
        "graph_case",
        "frame_index",
        "robot_id",
        "squad_local_index",
        "active_references",
        "truth_position",
    )
    return (
        all(
            _json_type_exact_equal(
                strict_row.get(field), restart_row.get(field)
            )
            for field in fields
        )
        and _json_type_exact_equal(
            _measurement_noise_identity(strict_row),
            _measurement_noise_identity(restart_row),
        )
    )


def _empty_seed_counts(seeds: list[int]) -> dict[str, dict[int, dict[str, int]]]:
    return {
        graph_case: {
            seed: {
                "denominator": 0,
                "exact_direct": 0,
                "upstream_unavailable": 0,
                "invalid_input_or_numeric": 0,
                "wnls_nonconvergence": 0,
            }
            for seed in seeds
        }
        for graph_case in _GRAPH_CASES
    }


def _record_primary_count(target: dict[str, int], row: dict) -> None:
    if row.get("primary_statistics") is not True:
        return
    target["denominator"] += 1
    attempt_status = row.get("attempt_status")
    reason = row.get("attempt_failure_reason")
    target["exact_direct"] += int(
        attempt_status == "invalid" and reason == _EXACT_DIRECT_REASON
    )
    target["upstream_unavailable"] += int(
        attempt_status == "invalid" and reason == _UPSTREAM_REASON
    )
    target["invalid_input_or_numeric"] += int(
        attempt_status == "invalid" and reason != _UPSTREAM_REASON
    )
    target["wnls_nonconvergence"] += int(
        attempt_status == "failed" and reason == _WNLS_REASON
    )


def _update_streaks(
    state: dict[tuple[str, int, int, str], int],
    maxima: dict[str, dict[str, int]],
    row: dict,
) -> None:
    if row.get("primary_statistics") is not True:
        return
    graph_case = row["graph_case"]
    events = {
        "exact_direct": (
            row.get("attempt_status") == "invalid"
            and row.get("attempt_failure_reason") == _EXACT_DIRECT_REASON
        ),
        "upstream_unavailable": (
            row.get("attempt_status") == "invalid"
            and row.get("attempt_failure_reason") == _UPSTREAM_REASON
        ),
        "wnls_nonconvergence": (
            row.get("attempt_status") == "failed"
            and row.get("attempt_failure_reason") == _WNLS_REASON
        ),
    }
    for event, occurred in events.items():
        key = (graph_case, row["seed"], row["robot_id"], event)
        current = state.get(key, 0) + 1 if occurred else 0
        state[key] = current
        maxima[graph_case][event] = max(
            maxima[graph_case][event], current
        )


def _validate_policy_row(
    row: dict,
    *,
    policy: str,
    acquisition_state: dict[tuple[int, str, int], tuple[bool, bool]],
) -> None:
    key = _compact_key(row)
    if row.get("initialization_policy") != policy:
        raise InputIntegrityError(f"row policy mismatch at key {key!r}")
    source = row.get("initial_estimate_source")
    if source not in _INITIAL_SOURCE_LABELS:
        raise InputIntegrityError(
            f"row initialization source is invalid at key {key!r}"
        )
    ever_value = row.get("ever_acquired_finite_before_attempt")
    if type(ever_value) is not bool:
        raise InputIntegrityError(
            f"row ever-finite provenance is not boolean at key {key!r}"
        )
    state_key = (row["seed"], row["graph_case"], row["robot_id"])
    ever_expected, previous_finite = acquisition_state.get(
        state_key, (False, False)
    )
    if ever_value != ever_expected:
        raise InputIntegrityError(
            f"row ever-finite provenance does not reconcile at key {key!r}"
        )
    if row["frame_index"] == 0:
        expected_source = "deployment_frame_zero"
    elif previous_finite:
        expected_source = "previous_finite"
    elif policy == RESTART_BEFORE_FIRST_FINITE_POLICY and not ever_expected:
        expected_source = "deployment_restart_before_first_finite"
    else:
        expected_source = "strict_previous_missing"
    if source != expected_source:
        raise InputIntegrityError(
            f"row initialization source does not reconcile at key {key!r}"
        )
    finite_recorded = row.get("finite")
    if type(finite_recorded) is not bool:
        raise InputIntegrityError(
            f"row finite provenance is not boolean at key {key!r}"
        )
    finite_recomputed = replay_module._valid_prior_result(row) is not None
    if finite_recorded is not finite_recomputed:
        raise InputIntegrityError(
            f"row finite provenance does not match retained state at key "
            f"{key!r}"
        )
    covariance_spd = row.get("covariance_spd")
    if type(covariance_spd) is not bool or covariance_spd is not finite_recomputed:
        raise InputIntegrityError(
            f"row covariance-SPD provenance does not match retained state at "
            f"key {key!r}"
        )
    acquisition_state[state_key] = (
        ever_expected or finite_recomputed,
        finite_recomputed,
    )


def _provenance_case() -> dict:
    return {
        "total_rows": 0,
        "source_counts": {
            source: 0 for source in sorted(_INITIAL_SOURCE_LABELS)
        },
        "restart_source_selections": 0,
        "restart_valid_input_attempts": 0,
        "restart_convergences": 0,
        "restart_short_circuit_reasons": {},
        "restart_attempt_failure_reasons": {},
    }


def _provenance_record() -> dict:
    return {
        "by_graph_case": {
            graph_case: _provenance_case()
            for graph_case in _GRAPH_CASES
        }
    }


def _increment_reason(target: dict[str, int], reason: object, key: tuple) -> None:
    label = "<none>" if reason is None else reason
    if type(label) is not str:
        raise InputIntegrityError(
            f"restart reason is invalid at key {key!r}"
        )
    target[label] = target.get(label, 0) + 1


def _record_restart_provenance(
    record: dict,
    row: dict,
    execution: dict,
) -> None:
    case = record["by_graph_case"][row["graph_case"]]
    case["total_rows"] += 1
    source = row["initial_estimate_source"]
    case["source_counts"][source] += 1
    if source != "deployment_restart_before_first_finite":
        return
    case["restart_source_selections"] += 1
    reason = row.get("attempt_failure_reason")
    if execution["wnls_invoked"] is False:
        _increment_reason(
            case["restart_short_circuit_reasons"],
            execution["short_circuit_reason"],
            _compact_key(row),
        )
        return
    case["restart_valid_input_attempts"] += 1
    if row.get("attempt_status") == "converged":
        case["restart_convergences"] += 1
    else:
        _increment_reason(
            case["restart_attempt_failure_reasons"],
            reason,
            _compact_key(row),
        )


def _merge_reason_counts(
    target: dict[str, int],
    source: dict[str, int],
) -> None:
    for reason, count in source.items():
        target[reason] = target.get(reason, 0) + count


def _finalize_provenance(record: dict) -> dict:
    aggregate = _provenance_case()
    for graph_case in _GRAPH_CASES:
        case = record["by_graph_case"][graph_case]
        source_total = sum(case["source_counts"].values())
        selection_total = (
            case["restart_valid_input_attempts"]
            + sum(case["restart_short_circuit_reasons"].values())
        )
        attempted_total = (
            case["restart_convergences"]
            + sum(case["restart_attempt_failure_reasons"].values())
        )
        if (
            source_total != case["total_rows"]
            or selection_total != case["restart_source_selections"]
            or attempted_total != case["restart_valid_input_attempts"]
        ):
            raise InputIntegrityError(
                f"restart provenance does not reconcile for {graph_case}"
            )
        case["source_counts_total"] = source_total
        case["reconciled"] = True
        aggregate["total_rows"] += case["total_rows"]
        for source, count in case["source_counts"].items():
            aggregate["source_counts"][source] += count
        for field in (
            "restart_source_selections",
            "restart_valid_input_attempts",
            "restart_convergences",
        ):
            aggregate[field] += case[field]
        _merge_reason_counts(
            aggregate["restart_short_circuit_reasons"],
            case["restart_short_circuit_reasons"],
        )
        _merge_reason_counts(
            aggregate["restart_attempt_failure_reasons"],
            case["restart_attempt_failure_reasons"],
        )
    aggregate["source_counts_total"] = sum(
        aggregate["source_counts"].values()
    )
    aggregate["reconciled"] = all(
        record["by_graph_case"][case]["reconciled"]
        for case in _GRAPH_CASES
    )
    record["aggregate"] = aggregate
    return record


def _empty_streak_maxima() -> dict[str, dict[str, int]]:
    return {
        graph_case: {
            "exact_direct": 0,
            "upstream_unavailable": 0,
            "wnls_nonconvergence": 0,
        }
        for graph_case in _GRAPH_CASES
    }


def _stream_paired_rows(
    strict: dict,
    restart: dict,
    seeds: list[int],
    *,
    live_guard: Callable[[], None] | None,
) -> dict:
    policy_counts = {
        "strict": _empty_seed_counts(seeds),
        "restart": _empty_seed_counts(seeds),
    }
    ratio_samples = {
        policy: {graph_case: [] for graph_case in _GRAPH_CASES}
        for policy in ("strict", "restart")
    }
    streak_state = {"strict": {}, "restart": {}}
    streak_maxima = {
        "strict": _empty_streak_maxima(),
        "restart": _empty_streak_maxima(),
    }
    acquisition = {"strict": {}, "restart": {}}
    provenance = _provenance_record()
    rows_compared = 0
    pairs = zip_longest(
        _verified_rows(strict, live_guard=live_guard),
        _verified_rows(restart, live_guard=live_guard),
        fillvalue=_MISSING,
    )
    for strict_row, restart_row in pairs:
        if strict_row is _MISSING or restart_row is _MISSING:
            raise InputIntegrityError(
                "strict and restart streams have different cardinality"
            )
        strict_key = _compact_key(strict_row)
        restart_key = _compact_key(restart_row)
        if strict_key != restart_key:
            raise InputIntegrityError(
                "strict and restart row key mismatch at "
                f"{strict_key!r}/{restart_key!r}"
            )
        _reconcile_replay_row_inputs(strict_row)
        restart_execution = _reconcile_replay_row_inputs(restart_row)
        if not _paired_inputs_equal(strict_row, restart_row):
            raise InputIntegrityError(
                f"paired trajectory or noise inputs differ at key {strict_key!r}"
            )
        _validate_policy_row(
            strict_row,
            policy=STRICT_PREVIOUS_POLICY,
            acquisition_state=acquisition["strict"],
        )
        _validate_policy_row(
            restart_row,
            policy=RESTART_BEFORE_FIRST_FINITE_POLICY,
            acquisition_state=acquisition["restart"],
        )
        _record_restart_provenance(
            provenance,
            restart_row,
            restart_execution,
        )
        for policy, row in (
            ("strict", strict_row),
            ("restart", restart_row),
        ):
            graph_case = row["graph_case"]
            seed = row["seed"]
            if graph_case not in policy_counts[policy] or seed not in (
                policy_counts[policy][graph_case]
            ):
                raise InputIntegrityError(
                    f"paired row dimensions are unexpected at key {_compact_key(row)!r}"
                )
            _record_primary_count(
                policy_counts[policy][graph_case][seed], row
            )
            _update_streaks(
                streak_state[policy], streak_maxima[policy], row
            )
            if (
                row.get("primary_statistics") is True
                and row.get("attempt_status") == "converged"
            ):
                ratio = row.get("error_to_epsilon_ratio")
                if (
                    type(ratio) in (int, float)
                    and math.isfinite(ratio)
                    and ratio >= 0.0
                ):
                    ratio_samples[policy][graph_case].append(float(ratio))
        rows_compared += 1
    _verify_bundle_unchanged(strict)
    _verify_bundle_unchanged(restart)
    _finalize_provenance(provenance)
    return {
        "rows_compared": rows_compared,
        "counts": policy_counts,
        "ratio_samples": ratio_samples,
        "streak_maxima": streak_maxima,
        "restart_provenance": provenance,
    }


def _bootstrap_interval(differences: list[float]) -> dict:
    if not differences:
        raise InputIntegrityError("paired outcome has no seed differences")
    values = np.asarray(differences, dtype=float)
    if not np.isfinite(values).all():
        raise InputIntegrityError("paired seed differences must be finite")
    rng = np.random.default_rng(BOOTSTRAP_SEED)
    indices = rng.integers(
        0,
        len(values),
        size=(BOOTSTRAP_RESAMPLES, 20),
    )
    means = values[indices].mean(axis=1)
    return {
        "resamples": BOOTSTRAP_RESAMPLES,
        "rng_seed": BOOTSTRAP_SEED,
        "draws_per_resample": 20,
        "lower": float(np.percentile(means, 2.5)),
        "upper": float(np.percentile(means, 97.5)),
    }


def _paired_outcome(
    counts: dict,
    graph_case: str,
    event: str,
    seeds: list[int],
) -> dict:
    records = []
    for seed in seeds:
        strict = counts["strict"][graph_case][seed]
        restart = counts["restart"][graph_case][seed]
        if strict["denominator"] != restart["denominator"]:
            raise InputIntegrityError(
                f"policy denominators differ for {graph_case}, seed {seed}"
            )
        denominator = strict["denominator"]
        if denominator <= 0:
            raise InputIntegrityError(
                f"primary denominator is empty for {graph_case}, seed {seed}"
            )
        strict_fraction = strict[event] / denominator
        restart_fraction = restart[event] / denominator
        records.append(
            {
                "seed": seed,
                "denominator": denominator,
                "strict_count": strict[event],
                "restart_count": restart[event],
                "strict_fraction": strict_fraction,
                "restart_fraction": restart_fraction,
                "paired_difference": restart_fraction - strict_fraction,
            }
        )
    strict_count = sum(record["strict_count"] for record in records)
    restart_count = sum(record["restart_count"] for record in records)
    strict_denominator = sum(record["denominator"] for record in records)
    restart_denominator = strict_denominator
    strict_fraction = strict_count / strict_denominator
    restart_fraction = restart_count / restart_denominator
    difference = restart_fraction - strict_fraction
    relative_difference = (
        None if strict_fraction == 0.0 else difference / strict_fraction
    )
    count_reduction = (
        None
        if strict_count == 0
        else (strict_count - restart_count) / strict_count
    )
    differences = [record["paired_difference"] for record in records]
    return {
        "seed_records": records,
        "aggregate": {
            "strict": {
                "count": strict_count,
                "denominator": strict_denominator,
                "fraction": strict_fraction,
            },
            "restart": {
                "count": restart_count,
                "denominator": restart_denominator,
                "fraction": restart_fraction,
            },
            "percentage_point_difference": 100.0 * difference,
            "relative_difference": relative_difference,
            "count_reduction": count_reduction,
        },
        "median_paired_difference": float(np.median(differences)),
        "bootstrap_95_percentile": _bootstrap_interval(differences),
    }


def _quantiles(values: list[float]) -> dict:
    if not values:
        return {
            "finite_count": 0,
            "median": None,
            "p90": None,
            "p95": None,
            "p99": None,
            "maximum": None,
        }
    array = np.asarray(values, dtype=float)
    quantiles = np.percentile(array, [50.0, 90.0, 95.0, 99.0])
    return {
        "finite_count": len(values),
        "median": float(quantiles[0]),
        "p90": float(quantiles[1]),
        "p95": float(quantiles[2]),
        "p99": float(quantiles[3]),
        "maximum": float(np.max(array)),
    }


def _calibration_record(
    analysis: dict,
    graph_case: str,
    ratio_samples: list[float],
    streaks: dict[str, int],
) -> dict:
    case = analysis["cases"][graph_case]
    calibration = case["calibration"]
    q = calibration["q"]
    return {
        "converged_attempts": calibration["converged_denominator"],
        "wnls_nonconvergence_count": case["failure_budget"]["counts"][
            "wnls_nonconvergence"
        ],
        "epsilon_contained_count": calibration["epsilon_contained"],
        "epsilon_containment_rate": calibration[
            "epsilon_containment_rate"
        ],
        "q_finite_count": q["finite_count"],
        "q_above_5_991464547_count": q["above_5_991464547"],
        "q_above_5_991464547_rate": q[
            "above_5_991464547_rate"
        ],
        "q_above_9_count": q["above_9"],
        "q_above_9_rate": q["above_9_rate"],
        "error_to_epsilon_ratio_quantiles": _quantiles(ratio_samples),
        "maximum_consecutive": dict(streaks),
    }


def _calibration_safeguards(
    strict_analysis: dict,
    restart_analysis: dict,
    paired: dict,
    *,
    primary_gate_passed: bool,
) -> dict:
    result = {}
    for graph_case in _GRAPH_CASES:
        strict_record = _calibration_record(
            strict_analysis,
            graph_case,
            paired["ratio_samples"]["strict"][graph_case],
            paired["streak_maxima"]["strict"][graph_case],
        )
        restart_record = _calibration_record(
            restart_analysis,
            graph_case,
            paired["ratio_samples"]["restart"][graph_case],
            paired["streak_maxima"]["restart"][graph_case],
        )
        strict_q = strict_record["q_above_9_rate"]
        restart_q = restart_record["q_above_9_rate"]
        strict_containment = strict_record["epsilon_containment_rate"]
        restart_containment = restart_record["epsilon_containment_rate"]
        q_change = (
            None
            if strict_q is None or restart_q is None
            else 100.0 * (restart_q - strict_q)
        )
        containment_change = (
            None
            if strict_containment is None or restart_containment is None
            else 100.0 * (restart_containment - strict_containment)
        )
        q_safe = q_change is not None and q_change <= 2.0
        containment_safe = (
            containment_change is not None and containment_change >= -2.0
        )
        record = {
            "strict": strict_record,
            "restart": restart_record,
            "q_above_9_percentage_point_change": q_change,
            "epsilon_containment_percentage_point_change": (
                containment_change
            ),
            "q_above_9_increase_at_most_2pp": q_safe,
            "epsilon_containment_drop_at_most_2pp": containment_safe,
            "safeguards_passed": q_safe and containment_safe,
        }
        if graph_case == "dynamic_dag_wnls":
            record["advance_to_multi_trajectory"] = (
                primary_gate_passed and q_safe and containment_safe
            )
        else:
            record["role"] = "descriptive_only"
        result[graph_case] = record
    return result


def _markdown(report: dict) -> bytes:
    gate = report["gates"]["dynamic_exact_direct"]
    lines = [
        "# Warm-start recovery comparison",
        "",
        f"- Status: {report['status']}",
        (
            "- Strict anchor rows: "
            f"{report['strict_anchor']['rows_compared']}"
        ),
        f"- Dynamic exact-direct gate: {gate['passed']}",
        (
            "- Aggregate reduction criterion: "
            f"{gate['aggregate_reduction_at_least_0_90']}"
        ),
        (
            "- Bootstrap upper below zero: "
            f"{gate['bootstrap_upper_below_zero']}"
        ),
        (
            "- Dynamic calibration safeguards: "
            f"{report['calibration_safeguards']['dynamic_dag_wnls']['safeguards_passed']}"
        ),
        "",
        "The statistical unit is one paired range-noise seed.",
        "The fixed-reference result is descriptive only.",
        "No controller, safety, radius, or cross-trajectory claim is supported.",
        "",
    ]
    return "\n".join(lines).encode("utf-8")


def _json_bytes(report: dict) -> bytes:
    return (
        json.dumps(
            report,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")


def _nearest_existing_ancestor(path: Path) -> Path:
    candidate = path
    while not candidate.exists():
        if candidate.parent == candidate:
            raise AnalysisLimitError(
                f"no existing ancestor for output path {path}"
            )
        candidate = candidate.parent
    return candidate


def _paths_overlap(first: Path, second: Path) -> bool:
    first = first.resolve()
    second = second.resolve()
    return (
        first == second
        or first in second.parents
        or second in first.parents
    )


def _rename_no_replace(source: Path, destination: Path) -> None:
    """Atomically rename a directory while refusing to replace a destination."""
    libc = ctypes.CDLL(None, use_errno=True)
    source_bytes = os.fsencode(source)
    destination_bytes = os.fsencode(destination)
    if sys.platform == "darwin":
        rename = libc.renamex_np
        rename.argtypes = [
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_uint,
        ]
        rename.restype = ctypes.c_int
        result = rename(source_bytes, destination_bytes, 0x00000004)
    elif sys.platform.startswith("linux"):
        try:
            rename = libc.renameat2
        except AttributeError as error:
            raise AnalysisLimitError(
                "atomic no-replace rename is unavailable"
            ) from error
        rename.argtypes = [
            ctypes.c_int,
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.c_char_p,
            ctypes.c_uint,
        ]
        rename.restype = ctypes.c_int
        result = rename(-100, source_bytes, -100, destination_bytes, 1)
    else:
        raise AnalysisLimitError(
            "atomic no-replace rename is unsupported on this platform"
        )
    if result == 0:
        return
    error_number = ctypes.get_errno()
    if error_number in (errno.EEXIST, errno.ENOTEMPTY):
        raise FileExistsError(
            error_number,
            os.strerror(error_number),
            str(destination),
        )
    raise OSError(
        error_number,
        os.strerror(error_number),
        str(destination),
    )


class _OutputTransaction:
    def __init__(
        self,
        output_dir: Path | None,
        protected_paths: list[Path],
    ) -> None:
        self.output_dir = None if output_dir is None else Path(output_dir)
        self.staging_dir: Path | None = None
        self.free_probes: list[int] = []
        self.protected_paths = protected_paths
        self.owns_output_parent = False
        self.owns_staging = False
        self.owns_published_output = False

    def __enter__(self):
        if self.output_dir is None:
            return self
        output = self.output_dir
        if output.is_symlink():
            raise AnalysisLimitError("output directory must not be a symlink")
        for protected in self.protected_paths:
            if _paths_overlap(output, protected):
                raise AnalysisLimitError(
                    "output directory overlaps a protected input"
                )
        if output.exists():
            raise FileExistsError("output directory already exists")
        try:
            nearest = _nearest_existing_ancestor(output.parent)
            require_start_space(nearest)
        except DiskSpaceError as error:
            raise AnalysisLimitError(str(error)) from error
        try:
            if not output.parent.exists():
                if (
                    not output.parent.parent.is_dir()
                    or output.parent.parent.is_symlink()
                ):
                    raise AnalysisLimitError(
                        "only the exact output parent may be created"
                    )
                try:
                    output.parent.mkdir()
                    self.owns_output_parent = True
                except FileExistsError:
                    self.owns_output_parent = False
            if output.parent.is_symlink() or not output.parent.is_dir():
                raise AnalysisLimitError(
                    "output parent must be a real directory"
                )
            if output.exists():
                raise FileExistsError("output directory already exists")
            self._probe(output.parent)
            staging = tempfile.mkdtemp(
                prefix=f"{output.name}.incomplete-",
                dir=output.parent,
            )
            self.staging_dir = Path(staging)
            self.owns_staging = True
            self._probe(self.staging_dir)
            return self
        except Exception:
            self._cleanup_owned_staging()
            self._cleanup_owned_parent_if_empty()
            raise

    def _probe(self, path: Path) -> None:
        free = available_bytes(path)
        self.free_probes.append(free)
        if free < HARD_FLOOR_BYTES:
            raise AnalysisLimitError(
                f"available={free} below live threshold={HARD_FLOOR_BYTES}"
            )
        root = (
            self.output_dir.parent
            if self.output_dir is not None
            else path
        )
        if allocated_bytes(root) > OUTPUT_ROOT_CAP_BYTES:
            raise AnalysisLimitError(
                "analysis output root exceeds allocation cap"
            )
        if (
            self.staging_dir is not None
            and self.staging_dir.exists()
            and allocated_bytes(self.staging_dir)
            > _ANALYSIS_OUTPUT_CAP_BYTES
        ):
            raise AnalysisLimitError(
                "comparison output exceeds 10 MB allocation cap"
            )

    def live_guard(self) -> None:
        if self.staging_dir is not None:
            self._probe(self.staging_dir)

    def publish(self, report: dict, reverify: Callable[[], None]) -> None:
        if self.output_dir is None or self.staging_dir is None:
            return
        json_bytes = _json_bytes(report)
        markdown_bytes = _markdown(report)
        if len(json_bytes) + len(markdown_bytes) > _ANALYSIS_OUTPUT_CAP_BYTES:
            raise AnalysisLimitError(
                "serialized comparison exceeds 10 MB cap"
            )
        for name, payload in (
            (_OUTPUT_JSON_NAME, json_bytes),
            (_OUTPUT_MARKDOWN_NAME, markdown_bytes),
        ):
            self._probe(self.staging_dir)
            path = self.staging_dir / name
            with path.open("xb") as destination:
                destination.write(payload)
            self._probe(self.staging_dir)
        if {path.name for path in self.staging_dir.iterdir()} != {
            _OUTPUT_JSON_NAME,
            _OUTPUT_MARKDOWN_NAME,
        }:
            raise AnalysisLimitError(
                "staging output contains unexpected files"
            )
        reverify()
        self._probe(self.staging_dir)
        _rename_no_replace(self.staging_dir, self.output_dir)
        self.owns_published_output = True
        self.owns_staging = False
        self.staging_dir = None
        self._probe(self.output_dir)

    def _cleanup_dir(self, directory: Path) -> None:
        if not directory.exists() or directory.is_symlink():
            return
        for path in directory.iterdir():
            if path.is_file() and not path.is_symlink():
                path.unlink()
            elif path.is_dir() and not path.is_symlink():
                self._cleanup_dir(path)
        directory.rmdir()

    def _cleanup_owned_staging(self) -> None:
        if (
            self.owns_staging
            and self.staging_dir is not None
            and self.staging_dir.exists()
        ):
            self._cleanup_dir(self.staging_dir)
        self.owns_staging = False
        self.staging_dir = None

    def _cleanup_owned_parent_if_empty(self) -> None:
        if (
            self.owns_output_parent
            and self.output_dir is not None
            and self.output_dir.parent.exists()
        ):
            try:
                self.output_dir.parent.rmdir()
            except OSError:
                pass
        self.owns_output_parent = False

    def __exit__(self, exc_type, exc_value, traceback):
        self._cleanup_owned_staging()
        if (
            exc_type is not None
            and self.owns_published_output
            and self.output_dir is not None
            and self.output_dir.exists()
        ):
            self._cleanup_dir(self.output_dir)
            self.owns_published_output = False
        if exc_type is not None:
            self._cleanup_owned_parent_if_empty()
        return False


def _run_calibration_analysis(
    bundle: dict,
    transaction: _OutputTransaction,
    label: str,
) -> dict:
    if transaction.staging_dir is None:
        return analyze_localization_failures(
            bundle["dir"],
            verify_hashes=True,
            max_examples_per_bucket=0,
        )
    output_dir = transaction.staging_dir / f".{label}-calibration-analysis"
    incomplete_dir = output_dir.with_name(f"{output_dir.name}.incomplete")
    try:
        report = analyze_localization_failures(
            bundle["dir"],
            verify_hashes=True,
            output_dir=output_dir,
            max_examples_per_bucket=0,
        )
        transaction.live_guard()
        if {path.name for path in output_dir.iterdir()} != {
            "failure-mechanisms.json",
            "failure-mechanisms.md",
        }:
            raise AnalysisLimitError(
                f"{label} calibration scratch output is unexpected"
            )
        return report
    finally:
        for scratch in (output_dir, incomplete_dir):
            if scratch.exists() and not scratch.is_symlink():
                transaction._cleanup_dir(scratch)
        transaction.live_guard()


def compare_warm_start_recovery(
    paired_bundle_dir: Path,
    immutable_baseline_dir: Path,
    *,
    expected_paired_parent_manifest_sha256: str,
    expected_baseline_manifest_sha256: str,
    expected_comparator_source_sha256: str,
    expected_failure_analyzer_source_sha256: str,
    output_dir: Path | None = None,
    verify_hashes: bool = True,
) -> dict:
    """Validate the strict anchor, pair seeds, and evaluate frozen gates."""
    if verify_hashes is not True:
        raise ValueError("strict comparison requires verify_hashes=True")
    paired_bundle_dir = Path(paired_bundle_dir)
    immutable_baseline_dir = Path(immutable_baseline_dir)
    if paired_bundle_dir.is_symlink() or not paired_bundle_dir.is_dir():
        raise InputIntegrityError("paired bundle must be a real directory")
    parent_path = paired_bundle_dir / "manifest.json"
    _regular_file(parent_path, "paired parent manifest")
    baseline_manifest_path = immutable_baseline_dir / "manifest.json"
    _regular_file(baseline_manifest_path, "immutable baseline manifest")
    comparator_source_path = Path(__file__).resolve()
    analyzer_source_path = Path(failure_analyzer.__file__).resolve()
    external_trust_roots = {
        "paired_parent_manifest_sha256": _verify_external_hash(
            parent_path,
            expected_paired_parent_manifest_sha256,
            "paired parent manifest",
        ),
        "baseline_manifest_sha256": _verify_external_hash(
            baseline_manifest_path,
            expected_baseline_manifest_sha256,
            "immutable baseline manifest",
        ),
        "comparator_source_sha256": _verify_external_hash(
            comparator_source_path,
            expected_comparator_source_sha256,
            "comparator source",
        ),
        "failure_analyzer_source_sha256": _verify_external_hash(
            analyzer_source_path,
            expected_failure_analyzer_source_sha256,
            "failure analyzer source",
        ),
    }
    parent, parent_raw = _strict_object(parent_path, "paired parent manifest")
    if parent.get("schema") != PARENT_SCHEMA_ID:
        raise InputIntegrityError("paired parent schema does not match")
    if parent.get("termination_reason") != "completed":
        raise InputIntegrityError("paired parent is not completed")
    if parent.get("estimator_contract") != ESTIMATOR_CONTRACT_ID:
        raise InputIntegrityError("paired parent estimator contract does not match")
    if not _json_type_exact_equal(
        parent.get("policies"),
        [
            STRICT_PREVIOUS_POLICY,
            RESTART_BEFORE_FIRST_FINITE_POLICY,
        ],
    ):
        raise InputIntegrityError("paired parent policies do not match")
    if Path(parent.get("immutable_baseline_dir", "")).resolve() != (
        immutable_baseline_dir.resolve()
    ):
        raise InputIntegrityError("parent immutable baseline path does not match")

    parent_inputs = _parent_inputs(parent)
    baseline = _load_bundle(
        immutable_baseline_dir, verify_hashes=verify_hashes
    )
    for field in ("input_data", "input_manifest"):
        if not _json_type_exact_equal(
            parent.get(field), baseline["manifest"].get(field)
        ):
            raise InputIntegrityError(
                f"parent {field} does not match anchored baseline record"
            )
    declared_baseline_hashes = parent.get("immutable_baseline_hashes")
    if not _json_type_exact_equal(
        declared_baseline_hashes, baseline["hashes"]
    ):
        raise InputIntegrityError("parent immutable baseline hashes do not match")

    children = parent.get("children")
    if not isinstance(children, dict) or set(children) != {"strict", "restart"}:
        raise InputIntegrityError("parent must contain exactly strict and restart children")
    loaded_children = {}
    for label in ("strict", "restart"):
        child_record = children[label]
        if not isinstance(child_record, dict):
            raise InputIntegrityError(f"{label} child record must be an object")
        output_dir_value = child_record.get("output_dir")
        if not isinstance(output_dir_value, str) or not output_dir_value:
            raise InputIntegrityError(f"{label} child output directory is invalid")
        child = _load_bundle(Path(output_dir_value), verify_hashes=verify_hashes)
        if not _json_type_exact_equal(child["manifest"], child_record):
            raise InputIntegrityError(
                f"parent {label} child record does not match child manifest"
            )
        loaded_children[label] = child
    strict = loaded_children["strict"]
    restart = loaded_children["restart"]
    _require_policy(strict, STRICT_PREVIOUS_POLICY, "strict")
    _require_policy(
        restart, RESTART_BEFORE_FIRST_FINITE_POLICY, "restart"
    )
    seeds = _require_dimensions(parent, baseline, strict, restart)
    _require_child_sources(parent, strict, restart)

    def reverify_all() -> None:
        _verify_external_hash(
            parent_path,
            external_trust_roots["paired_parent_manifest_sha256"],
            "paired parent manifest",
        )
        _verify_external_hash(
            baseline_manifest_path,
            external_trust_roots["baseline_manifest_sha256"],
            "immutable baseline manifest",
        )
        _verify_external_hash(
            comparator_source_path,
            external_trust_roots["comparator_source_sha256"],
            "comparator source",
        )
        _verify_external_hash(
            analyzer_source_path,
            external_trust_roots["failure_analyzer_source_sha256"],
            "failure analyzer source",
        )
        _verify_bundle_unchanged(baseline)
        _verify_bundle_unchanged(strict)
        _verify_bundle_unchanged(restart)
        _verify_parent_inputs_unchanged(parent_inputs)
        try:
            current_parent = parent_path.read_bytes()
        except OSError as error:
            raise InputIntegrityError(
                f"cannot re-read paired parent manifest: {error}"
            ) from error
        if hashlib.sha256(current_parent).digest() != hashlib.sha256(
            parent_raw
        ).digest():
            raise InputIntegrityError(
                "paired parent manifest changed during analysis"
            )

    protected = [
        paired_bundle_dir,
        immutable_baseline_dir,
        strict["dir"],
        restart["dir"],
        *[path for path, _ in parent_inputs.values()],
    ]
    with _OutputTransaction(
        None if output_dir is None else Path(output_dir),
        protected,
    ) as transaction:
        live_guard = (
            transaction.live_guard if output_dir is not None else None
        )
        anchor = _strict_anchor(
            baseline,
            strict,
            live_guard=live_guard,
        )
        paired = _stream_paired_rows(
            strict,
            restart,
            seeds,
            live_guard=live_guard,
        )
        if paired["rows_compared"] != anchor["rows_compared"]:
            raise InputIntegrityError(
                "paired child cardinality differs from strict anchor"
            )

        exact_dynamic = _paired_outcome(
            paired["counts"],
            "dynamic_dag_wnls",
            "exact_direct",
            seeds,
        )
        exact_interval = exact_dynamic["bootstrap_95_percentile"]
        dynamic_reduction = exact_dynamic["aggregate"]["count_reduction"]
        dynamic_gate = {
            "bootstrap_upper_below_zero": exact_interval["upper"] < 0.0,
            "aggregate_reduction_at_least_0_90": (
                dynamic_reduction is not None
                and dynamic_reduction >= 0.90
            ),
        }
        dynamic_gate["passed"] = all(dynamic_gate.values())

        broader_dynamic = _paired_outcome(
            paired["counts"],
            "dynamic_dag_wnls",
            "invalid_input_or_numeric",
            seeds,
        )
        upstream_inference = None
        upstream_gate = None
        if dynamic_gate["passed"]:
            upstream_inference = _paired_outcome(
                paired["counts"],
                "dynamic_dag_wnls",
                "upstream_unavailable",
                seeds,
            )
            upstream_gate = {
                "bootstrap_upper_below_zero": (
                    upstream_inference["bootstrap_95_percentile"]["upper"]
                    < 0.0
                )
            }
            upstream_gate["cascade_interruption_supported"] = (
                upstream_gate["bootstrap_upper_below_zero"]
            )

        fixed_exact = _paired_outcome(
            paired["counts"],
            "fixed_refs_wnls",
            "exact_direct",
            seeds,
        )
        fixed_broader = _paired_outcome(
            paired["counts"],
            "fixed_refs_wnls",
            "invalid_input_or_numeric",
            seeds,
        )
        fixed_upstream = _paired_outcome(
            paired["counts"],
            "fixed_refs_wnls",
            "upstream_unavailable",
            seeds,
        )

        strict_analysis = _run_calibration_analysis(
            strict, transaction, "strict"
        )
        restart_analysis = _run_calibration_analysis(
            restart, transaction, "restart"
        )
        if output_dir is not None:
            transaction.live_guard()
        reverify_all()

        safeguards = _calibration_safeguards(
            strict_analysis,
            restart_analysis,
            paired,
            primary_gate_passed=dynamic_gate["passed"],
        )
        allowed_claim = None
        if dynamic_gate["passed"]:
            allowed_claim = (
                "Under the preserved trajectory and the exact frozen paired "
                "range-noise seeds, deployment restart reduced the registered "
                "local initialization failure event."
            )
            if (
                upstream_gate is not None
                and upstream_gate["cascade_interruption_supported"]
            ):
                allowed_claim += (
                    " The hierarchical paired result also reduced observed "
                    "downstream unavailability on this trajectory."
                )
        report = {
            "schema": SCHEMA_ID,
            "status": "completed",
            "source": {
                "paired_bundle_dir": str(paired_bundle_dir.resolve()),
                "paired_parent_manifest_sha256": hashlib.sha256(
                    parent_raw
                ).hexdigest(),
                "immutable_baseline_dir": str(
                    immutable_baseline_dir.resolve()
                ),
                "immutable_baseline_hashes": dict(baseline["hashes"]),
                "strict_child_dir": str(strict["dir"].resolve()),
                "strict_child_hashes": dict(strict["hashes"]),
                "restart_child_dir": str(restart["dir"].resolve()),
                "restart_child_hashes": dict(restart["hashes"]),
                "source_snapshot_sha256": parent_inputs[
                    "source_snapshot"
                ][1],
                "replay_implementation_sha256": parent_inputs[
                    "replay_implementation"
                ][1],
                "input_data_sha256": parent_inputs["input_data"][1],
                "input_manifest_sha256": parent_inputs[
                    "input_manifest"
                ][1],
                "external_trust_roots": dict(external_trust_roots),
            },
            "protocol": {
                "statistical_unit": "paired range-noise seed",
                "seed_count": len(seeds),
                "seeds": list(seeds),
                "bootstrap_resamples": BOOTSTRAP_RESAMPLES,
                "bootstrap_seed": BOOTSTRAP_SEED,
                "bootstrap_draws_per_resample": 20,
                "primary_event": {
                    "primary_statistics": True,
                    "attempt_status": "invalid",
                    "attempt_failure_reason": _EXACT_DIRECT_REASON,
                },
                "primary_denominator": (
                    "every primary_statistics row, including converged, "
                    "failed, and invalid attempts"
                ),
            },
            "strict_anchor": anchor,
            "paired_integrity": {
                "rows_compared": paired["rows_compared"],
                "exact_keys_and_order": True,
                "paired_inputs_equal": True,
                "child_hashes_unchanged": True,
            },
            "comparisons": {
                "dynamic_dag_wnls": {
                    "role": "confirmatory_primary",
                    "exact_direct": exact_dynamic,
                    "broader_invalid_input_or_numeric": broader_dynamic,
                    "upstream_unavailable_inference": upstream_inference,
                },
                "fixed_refs_wnls": {
                    "role": "descriptive_only",
                    "exact_direct": fixed_exact,
                    "broader_invalid_input_or_numeric": fixed_broader,
                    "upstream_unavailable": fixed_upstream,
                },
            },
            "gates": {
                "dynamic_exact_direct": dynamic_gate,
                "dynamic_upstream_secondary": upstream_gate,
            },
            "calibration_safeguards": safeguards,
            "restart_provenance": paired["restart_provenance"],
            "claim_boundary": {
                "allowed": allowed_claim,
                "not_supported": [
                    "graph superiority",
                    "controller, collision-avoidance, connectivity, or mission guarantees",
                    "coefficient-3 epsilon sufficiency",
                    "production estimator robustness",
                    "generalization across trajectories or geometries",
                ],
            },
        }
        transaction.publish(report, reverify_all)
        return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--paired-bundle-dir", required=True, type=Path)
    parser.add_argument("--immutable-baseline-dir", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument(
        "--expected-paired-parent-manifest-sha256", required=True
    )
    parser.add_argument(
        "--expected-baseline-manifest-sha256", required=True
    )
    parser.add_argument("--expected-comparator-source-sha256", required=True)
    parser.add_argument(
        "--expected-failure-analyzer-source-sha256", required=True
    )
    arguments = parser.parse_args(argv)
    try:
        report = compare_warm_start_recovery(
            arguments.paired_bundle_dir,
            arguments.immutable_baseline_dir,
            output_dir=arguments.output_dir,
            expected_paired_parent_manifest_sha256=(
                arguments.expected_paired_parent_manifest_sha256
            ),
            expected_baseline_manifest_sha256=(
                arguments.expected_baseline_manifest_sha256
            ),
            expected_comparator_source_sha256=(
                arguments.expected_comparator_source_sha256
            ),
            expected_failure_analyzer_source_sha256=(
                arguments.expected_failure_analyzer_source_sha256
            ),
        )
    except (
        AnalysisLimitError,
        InputIntegrityError,
        FileExistsError,
        OSError,
        ValueError,
    ) as error:
        parser.error(str(error))
    print(json.dumps(report, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
