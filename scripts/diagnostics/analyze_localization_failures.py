"""Read-only input verification for localization-failure analysis."""

from __future__ import annotations

import gzip
import hashlib
import json
import math
import random
from collections.abc import Iterator
from pathlib import Path


SCHEMA_ID = "cbf2026-localization-failure-analysis-v1"
ESTIMATOR_CONTRACT_ID = "variable_weight_nls_full_residual_jacobian_v1"
_HASH_CHUNK_SIZE = 8192
_PROCESS_NAME = "calibration.jsonl.gz"
_SUMMARY_NAME = "summary.json"
_SUMMARY_MARKDOWN_NAME = "summary.md"
_REQUIRED_ROW_FIELDS = {
    "frame_index",
    "seed",
    "graph_case",
    "robot_id",
    "primary_statistics",
    "attempt_status",
    "status",
}
_COUNT_FIELDS = ("attempt_status_counts", "status_counts")
_ATTEMPT_STATUSES = {"converged", "invalid", "failed"}
_RETAINED_STATUSES = {"converged", "stale", "invalid", "failed"}
_COUNT_STATUSES = ("converged", "stale", "invalid", "failed")
_UPSTREAM_UNAVAILABLE_REASON = "invalid upstream UAV reference"
_WNLS_NONCONVERGENCE_REASON = "maximum WNLS iterations exceeded"
_FAILURE_CLASSES = (
    "contained",
    "converged_outside_radius",
    "upstream_unavailable",
    "invalid_input_or_numeric",
    "wnls_nonconvergence",
    "other_failed",
)
_PRIMARY_ATTEMPT_STATUSES = ("converged", "invalid", "failed")
_MAX_REASON_LABELS = 32
_MAX_REASON_OVERFLOW_EXAMPLES = 5
_TIME_BINS = ("1-100", "101-200", "201-300", "301-400", "401-499")
_REFERENCE_BINS = tuple(str(value) for value in range(10)) + ("10_or_more",)
_CONDITION_BINS = ("[1,10)", "[10,30)", "[30,100)", "[100,infinity)")
_MIN_EIGEN_BINS = ("(0,0.05)", "[0.05,0.2)", "[0.2,1)", "[1,infinity)")


class InputIntegrityError(RuntimeError):
    pass


class AnalysisLimitError(RuntimeError):
    pass


def _normalized_squared_error(error_vector: object, covariance: object) -> float:
    """Return e^T P^-1 e after strict finite, symmetric 2-D PD validation."""
    if not isinstance(error_vector, (list, tuple)) or len(error_vector) != 2:
        raise ValueError("error vector must contain two values")
    if (
        not isinstance(covariance, (list, tuple))
        or len(covariance) != 2
        or any(not isinstance(row, (list, tuple)) or len(row) != 2 for row in covariance)
    ):
        raise ValueError("covariance must be a 2 by 2 matrix")
    ex, ey = error_vector
    a, b = covariance[0]
    c, d = covariance[1]
    values = (ex, ey, a, b, c, d)
    if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
        raise ValueError("error vector and covariance must be finite")
    max_abs = max(abs(value) for value in (a, b, c, d))
    if abs(b - c) > 1e-12 * max(1.0, max_abs):
        raise ValueError("covariance must be symmetric")
    determinant = a * d - b * c
    if a <= 0.0 or determinant <= 0.0:
        raise ValueError("covariance must be positive definite")
    return (d * ex * ex - (b + c) * ex * ey + a * ey * ey) / determinant


def _ratio_bin(value: float) -> str:
    if value < 0.5:
        return "[0,0.5)"
    if value < 1.0:
        return "[0.5,1)"
    if value < 2.0:
        return "[1,2)"
    if value < 5.0:
        return "[2,5)"
    return "[5,infinity)"


def _q_bin(value: float) -> str:
    if value < 2.295748929:
        return "[0,2.295748929)"
    if value < 5.991464547:
        return "[2.295748929,5.991464547)"
    if value <= 9.0:
        return "[5.991464547,9]"
    return "(9,infinity)"


def _paired_seed_bootstrap(
    seed_counts: list[dict], *, resamples: int = 10000, rng_seed: int = 20260729
) -> dict:
    """Bootstrap D_upstream by resampling paired seed count records."""
    if not seed_counts or type(resamples) is not int or resamples <= 0:
        raise ValueError("seed counts and a positive resample count are required")
    required = ("dyn_up", "fix_up", "dyn_invalid", "fix_invalid")
    for record in seed_counts:
        if not isinstance(record, dict) or any(
            type(record.get(key)) is not int or record[key] < 0 for key in required
        ):
            raise ValueError("seed counts must be non-negative integer records")

    def ratio(records: list[dict]) -> float | None:
        numerator = sum(record["dyn_up"] - record["fix_up"] for record in records)
        denominator = sum(
            record["dyn_invalid"] - record["fix_invalid"] for record in records
        )
        return None if denominator <= 0 else numerator / denominator

    seed_specific = [ratio([record]) for record in seed_counts]
    rng = random.Random(rng_seed)
    estimable: list[float] = []
    non_estimable = 0
    for _ in range(resamples):
        sample = [seed_counts[rng.randrange(len(seed_counts))] for _ in seed_counts]
        value = ratio(sample)
        if value is None:
            non_estimable += 1
        else:
            estimable.append(value)
    interval = None
    if len(estimable) >= math.ceil(0.95 * resamples):
        values = sorted(estimable)
        interval = {
            "lower": values[int(0.025 * (len(values) - 1))],
            "upper": values[int(0.975 * (len(values) - 1))],
        }
    return {
        "point_estimate": ratio(seed_counts),
        "seed_specific": seed_specific,
        "estimable_resamples": len(estimable),
        "non_estimable_resamples": non_estimable,
        "percentile_interval": interval,
    }


def _time_bin(frame_index: int) -> str | None:
    return _TIME_BINS[(frame_index - 1) // 100] if 1 <= frame_index <= 499 else None


def _reference_bin(value: int) -> str:
    return str(value) if value < 10 else "10_or_more"


def _condition_bin(value: float) -> str | None:
    if value < 1:
        return None
    if value < 10:
        return "[1,10)"
    if value < 30:
        return "[10,30)"
    if value < 100:
        return "[30,100)"
    return "[100,infinity)"


def _min_eigen_bin(value: float) -> str | None:
    if value <= 0:
        return None
    if value < 0.05:
        return "(0,0.05)"
    if value < 0.2:
        return "[0.05,0.2)"
    if value < 1:
        return "[0.2,1)"
    return "[1,infinity)"


def _budget() -> dict:
    return {"denominator": 0, "counts": {key: 0 for key in _FAILURE_CLASSES}}


def _empty_task3_case() -> dict:
    return {
        "initialization": {"by_depth": {str(i): _budget() for i in range(1, 8)}},
        "strata": {
            "depth": {str(i): _budget() for i in range(1, 8)},
            "time": {key: _budget() for key in _TIME_BINS},
            "reference_count": {key: {"denominator": 0} for key in _REFERENCE_BINS},
            "phi_condition": {key: {"denominator": 0} for key in _CONDITION_BINS},
            "phi_min": {key: {"denominator": 0} for key in _MIN_EIGEN_BINS},
        },
        "calibration": {
            "epsilon_containment_denominator": 0,
            "epsilon_contained": 0,
            "conditional_covariance_invalid": 0,
            "q": {"finite_count": 0, "sum": 0.0, "max": None,
                  "bins": {key: 0 for key in ("[0,2.295748929)", "[2.295748929,5.991464547)", "[5.991464547,9]", "(9,infinity)")}},
            "ratio": {"finite_count": 0, "sum": 0.0, "max": None,
                      "bins": {key: 0 for key in ("[0,0.5)", "[0.5,1)", "[1,2)", "[2,5)", "[5,infinity)")}},
        },
    }


def _record_budget(budget: dict, attempt_class: str) -> None:
    budget["denominator"] += 1
    budget["counts"][attempt_class] += 1


def _record_calibration(case: dict, row: dict) -> None:
    if row["attempt_status"] != "converged":
        return
    calibration = case["calibration"]
    calibration["epsilon_containment_denominator"] += 1
    if row.get("containment") is True:
        calibration["epsilon_contained"] += 1
    ratio = row.get("error_to_epsilon_ratio")
    if type(ratio) in (int, float) and math.isfinite(ratio) and ratio >= 0:
        target = calibration["ratio"]
        target["finite_count"] += 1
        target["sum"] += ratio
        target["max"] = ratio if target["max"] is None else max(target["max"], ratio)
        target["bins"][_ratio_bin(ratio)] += 1
    try:
        q = _normalized_squared_error(row.get("error_vector"), row.get("covariance"))
    except ValueError:
        calibration["conditional_covariance_invalid"] += 1
        return
    target = calibration["q"]
    target["finite_count"] += 1
    target["sum"] += q
    target["max"] = q if target["max"] is None else max(target["max"], q)
    target["bins"][_q_bin(q)] += 1


def _attempt_class(row: dict) -> str:
    """Classify the current solver attempt, never the retained state."""
    attempt_status = row.get("attempt_status")
    reason = row.get("attempt_failure_reason")
    if attempt_status == "converged":
        containment = row.get("containment")
        if containment is True:
            return "contained"
        if containment is False:
            return "converged_outside_radius"
        raise InputIntegrityError("converged attempt containment must be boolean")
    if attempt_status == "invalid":
        return (
            "upstream_unavailable"
            if reason == _UPSTREAM_UNAVAILABLE_REASON
            else "invalid_input_or_numeric"
        )
    if attempt_status == "failed":
        return (
            "wnls_nonconvergence"
            if reason == _WNLS_NONCONVERGENCE_REASON
            else "other_failed"
        )
    raise InputIntegrityError("row attempt status is not recognized")


def _empty_failure_budget(graph_cases: list[str]) -> dict[str, dict]:
    predecessor_classes = {
        attempt_class: 0 for attempt_class in (*_FAILURE_CLASSES, "not_observed")
    }
    return {
        graph_case: {
            "failure_budget": {
                "denominator": 0,
                "counts": {attempt_class: 0 for attempt_class in _FAILURE_CLASSES},
            },
            "attempt_status_counts": {
                attempt_status: 0 for attempt_status in _PRIMARY_ATTEMPT_STATUSES
            },
            "reason_labels": {
                "counts": {},
                "overflow_count": 0,
                "overflow_examples": [],
            },
            "lineage": {
                "upstream_unavailable_rows": 0,
                "unavailable_uav_reference_edges": 0,
                "predecessor_attempt_classes": predecessor_classes.copy(),
                "propagation_depth_counts": {"not_observed": 0},
            },
        }
        for graph_case in graph_cases
    }


def _record_reason_label(case: dict, reason: object) -> None:
    if reason is None:
        return
    if not isinstance(reason, str) or not reason:
        raise InputIntegrityError("attempt failure reason must be a string or null")
    labels = case["reason_labels"]
    counts = labels["counts"]
    if reason in counts:
        counts[reason] += 1
        return
    if len(counts) < _MAX_REASON_LABELS:
        counts[reason] = 1
        return
    labels["overflow_count"] += 1
    examples = labels["overflow_examples"]
    if len(examples) < _MAX_REASON_OVERFLOW_EXAMPLES:
        examples.append(reason)


def _unavailable_uav_reference_ids(row: dict) -> list[int]:
    measurements = row.get("measurements")
    if not isinstance(measurements, list):
        raise InputIntegrityError("upstream-unavailable row measurements must be a list")
    unavailable_ids = []
    for measurement in measurements:
        if not isinstance(measurement, dict):
            raise InputIntegrityError("measurement record must be an object")
        if (
            measurement.get("kind") == "uav"
            and measurement.get("estimated_reference_available") is False
        ):
            reference_id = measurement.get("id")
            if type(reference_id) is not int or reference_id < 0:
                raise InputIntegrityError("unavailable UAV reference ID must be a non-negative integer")
            unavailable_ids.append(reference_id)
    return unavailable_ids


def _lineage_depth(
    attempt_class: str,
    unavailable_ids: list[int],
    predecessors: dict[int, dict],
) -> int | None:
    if attempt_class != "upstream_unavailable":
        return 0
    observed_depths = [
        predecessor["propagation_depth"]
        for reference_id in unavailable_ids
        if (predecessor := predecessors.get(reference_id)) is not None
        and predecessor["propagation_depth"] is not None
    ]
    return None if not observed_depths else 1 + max(observed_depths)


def _record_lineage(
    case: dict,
    row: dict,
    attempt_class: str,
    predecessors: dict[int, dict],
) -> int | None:
    unavailable_ids = (
        _unavailable_uav_reference_ids(row)
        if attempt_class == "upstream_unavailable"
        else []
    )
    propagation_depth = _lineage_depth(attempt_class, unavailable_ids, predecessors)
    if attempt_class != "upstream_unavailable":
        return propagation_depth

    lineage = case["lineage"]
    lineage["upstream_unavailable_rows"] += 1
    for reference_id in unavailable_ids:
        lineage["unavailable_uav_reference_edges"] += 1
        predecessor = predecessors.get(reference_id)
        predecessor_class = (
            "not_observed" if predecessor is None else predecessor["attempt_class"]
        )
        lineage["predecessor_attempt_classes"][predecessor_class] += 1
    depth_key = "not_observed" if propagation_depth is None else str(propagation_depth)
    lineage["propagation_depth_counts"][depth_key] = (
        lineage["propagation_depth_counts"].get(depth_key, 0) + 1
    )
    return propagation_depth


def _strict_json_line(raw: bytes) -> dict:
    value = json.loads(
        raw,
        parse_constant=lambda token: (_ for _ in ()).throw(
            ValueError(f"non-finite JSON token: {token}")
        ),
    )
    if not isinstance(value, dict):
        raise InputIntegrityError("every process line must be a JSON object")
    return value


def _require_mapping(value: object, description: str) -> dict:
    if not isinstance(value, dict):
        raise InputIntegrityError(f"{description} must be an object")
    return value


def _require_string(value: object, description: str) -> str:
    if not isinstance(value, str) or not value:
        raise InputIntegrityError(f"{description} must be a non-empty string")
    return value


def _require_nonnegative_int(value: object, description: str) -> int:
    if type(value) is not int or value < 0:
        raise InputIntegrityError(f"{description} must be a non-negative integer")
    return value


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(_HASH_CHUNK_SIZE), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_object(path: Path, description: str) -> tuple[dict, bytes]:
    try:
        raw = path.read_bytes()
        value = _strict_json_line(raw)
    except (OSError, ValueError) as error:
        raise InputIntegrityError(f"invalid {description}: {error}") from error
    return value, raw


def _verify_hash(path: Path, expected: object, description: str) -> None:
    expected_hash = _require_string(expected, f"{description} hash")
    try:
        observed_hash = _sha256_file(path)
    except OSError as error:
        raise InputIntegrityError(f"cannot read {description}: {error}") from error
    if observed_hash != expected_hash:
        raise InputIntegrityError(f"{description} hash does not match manifest")


def _settings_dimensions(settings_value: object) -> tuple[int, list[int], list[str], int]:
    settings = _require_mapping(settings_value, "settings")
    frame_count = _require_nonnegative_int(
        settings.get("effective_frame_count"), "effective frame count"
    )
    if frame_count <= 0:
        raise InputIntegrityError("effective frame count must be positive")
    seeds_value = settings.get("run_seeds")
    graph_cases_value = settings.get("graph_cases")
    if not isinstance(seeds_value, list) or not seeds_value:
        raise InputIntegrityError("run seeds must be a non-empty list")
    if not isinstance(graph_cases_value, list) or not graph_cases_value:
        raise InputIntegrityError("graph cases must be a non-empty list")
    seeds: list[int] = []
    for seed in seeds_value:
        if type(seed) is not int:
            raise InputIntegrityError("every run seed must be an integer")
        seeds.append(seed)
    graph_cases: list[str] = []
    for graph_case in graph_cases_value:
        graph_cases.append(_require_string(graph_case, "graph case"))
    if len(set(graph_cases)) != len(graph_cases):
        raise InputIntegrityError("graph cases must not repeat")
    return frame_count, seeds, graph_cases, frame_count * len(seeds) * len(graph_cases)


def _stream_dimensions(summary: dict) -> tuple[int, int, list[int], list[str], int]:
    expected_rows = _require_nonnegative_int(
        summary.get("expected_process_rows"), "expected process row count"
    )
    process_rows = _require_nonnegative_int(summary.get("process_rows"), "process row count")
    if expected_rows != process_rows:
        raise InputIntegrityError("expected process rows and process rows must match")
    frame_count, seeds, graph_cases, dimension = _settings_dimensions(summary.get("settings"))
    if expected_rows % dimension:
        raise InputIntegrityError("expected process row count is not divisible by key dimensions")
    robot_count = expected_rows // dimension
    if robot_count <= 0:
        raise InputIntegrityError("expected process row count produces a non-positive robot count")
    cases = _require_mapping(summary.get("graph_cases"), "summary graph cases")
    if set(cases) != set(graph_cases):
        raise InputIntegrityError("summary graph cases do not match settings")
    return expected_rows, frame_count, seeds, graph_cases, robot_count


def _expected_keys(
    frame_count: int, seeds: list[int], graph_cases: list[str], robot_count: int
) -> Iterator[tuple[int, int, str, int]]:
    for frame_index in range(frame_count):
        for seed in seeds:
            for graph_case in graph_cases:
                for robot_id in range(1, robot_count + 1):
                    yield frame_index, seed, graph_case, robot_id


def _validate_row(row: dict) -> tuple[int, int, str, int, bool, str, str]:
    missing = _REQUIRED_ROW_FIELDS - set(row)
    if missing:
        raise InputIntegrityError(f"process row is missing fields: {sorted(missing)!r}")
    frame_index = _require_nonnegative_int(row["frame_index"], "row frame index")
    seed = _require_nonnegative_int(row["seed"], "row seed")
    graph_case = _require_string(row["graph_case"], "row graph case")
    robot_id = _require_nonnegative_int(row["robot_id"], "row robot id")
    if type(row["primary_statistics"]) is not bool:
        raise InputIntegrityError("row primary statistics must be boolean")
    if row["primary_statistics"] != (frame_index != 0):
        raise InputIntegrityError("row primary statistics disagrees with frame index")
    attempt_status = _require_string(row["attempt_status"], "row attempt status")
    status = _require_string(row["status"], "row status")
    if attempt_status not in _ATTEMPT_STATUSES:
        raise InputIntegrityError("row attempt status is not recognized")
    if status not in _RETAINED_STATUSES:
        raise InputIntegrityError("row status is not recognized")
    return frame_index, seed, graph_case, robot_id, row["primary_statistics"], attempt_status, status


def _empty_case_counts(graph_cases: list[str]) -> dict[str, dict[str, dict[str, dict[str, int]]]]:
    def empty_status_counts() -> dict[str, int]:
        return {status: 0 for status in _COUNT_STATUSES}

    return {
        graph_case: {
            "overall": {
                "attempt_status_counts": empty_status_counts(),
                "status_counts": empty_status_counts(),
            },
            "initialization_frame": {
                "attempt_status_counts": empty_status_counts(),
                "status_counts": empty_status_counts(),
            },
        }
        for graph_case in graph_cases
    }


def _increment(counter: dict[str, int], value: str) -> None:
    counter[value] = counter.get(value, 0) + 1


def _validate_summary_counts(summary: dict, observed: dict[str, dict[str, dict[str, dict[str, int]]]]) -> None:
    cases = _require_mapping(summary.get("graph_cases"), "summary graph cases")
    for graph_case, observed_case in observed.items():
        declared_case = _require_mapping(cases.get(graph_case), f"summary graph case {graph_case}")
        for bucket in ("overall", "initialization_frame"):
            declared_bucket = _require_mapping(
                declared_case.get(bucket), f"summary {graph_case} {bucket}"
            )
            for count_field in _COUNT_FIELDS:
                if count_field not in declared_bucket:
                    raise InputIntegrityError(
                        f"summary {graph_case} {bucket} is missing {count_field}"
                    )
                declared_counts = _require_mapping(
                    declared_bucket[count_field], f"summary {graph_case} {bucket} {count_field}"
                )
                if set(declared_counts) != set(_COUNT_STATUSES):
                    raise InputIntegrityError(
                        f"summary {graph_case} {bucket} {count_field} has unrecognized statuses"
                    )
                for status, count in declared_counts.items():
                    _require_string(status, f"summary {graph_case} {bucket} status")
                    _require_nonnegative_int(
                        count, f"summary {graph_case} {bucket} {count_field} count"
                    )
                if declared_counts != observed_case[bucket][count_field]:
                    raise InputIntegrityError(
                        f"summary {graph_case} {bucket} {count_field} does not match process stream"
                    )


def _iter_verified_rows(bundle_dir: Path, manifest: dict, summary: dict) -> Iterator[dict]:
    process_path = bundle_dir / _PROCESS_NAME
    verify_hashes = bool(manifest.get("_verify_hashes", True))
    if verify_hashes:
        _verify_hash(process_path, manifest.get("compressed_process_sha256"), "compressed process")
        expected_decompressed_hash = _require_string(
            manifest.get("decompressed_process_sha256"), "decompressed process hash"
        )
    else:
        expected_decompressed_hash = ""

    expected_rows, frame_count, seeds, graph_cases, robot_count = _stream_dimensions(summary)
    cursor = _expected_keys(frame_count, seeds, graph_cases, robot_count)
    observed_counts = _empty_case_counts(graph_cases)
    digest = hashlib.sha256()
    observed_rows = 0
    try:
        with gzip.open(process_path, "rb") as stream:
            for raw in stream:
                digest.update(raw)
                try:
                    row = _strict_json_line(raw)
                except ValueError as error:
                    raise InputIntegrityError(f"invalid process JSON line: {error}") from error
                (
                    frame_index,
                    seed,
                    graph_case,
                    robot_id,
                    primary_statistics,
                    attempt_status,
                    status,
                ) = _validate_row(row)
                try:
                    expected_key = next(cursor)
                except StopIteration as error:
                    raise InputIntegrityError("process stream has more rows than expected") from error
                actual_key = (frame_index, seed, graph_case, robot_id)
                if actual_key != expected_key:
                    raise InputIntegrityError(
                        "row key does not match canonical expected key "
                        f"{expected_key!r}: got {actual_key!r}"
                    )
                bucket = "overall" if primary_statistics else "initialization_frame"
                _increment(observed_counts[graph_case][bucket]["attempt_status_counts"], attempt_status)
                _increment(observed_counts[graph_case][bucket]["status_counts"], status)
                observed_rows += 1
                yield row
    except InputIntegrityError:
        raise
    except (OSError, EOFError, ValueError) as error:
        raise InputIntegrityError(f"invalid or truncated process stream: {error}") from error

    try:
        next(cursor)
    except StopIteration:
        pass
    else:
        raise InputIntegrityError("process stream ended before its canonical expected key")
    if observed_rows != expected_rows:
        raise InputIntegrityError("observed process row count does not match expected row count")
    if verify_hashes and digest.hexdigest() != expected_decompressed_hash:
        raise InputIntegrityError("decompressed process hash does not match manifest")
    _validate_summary_counts(summary, observed_counts)


def _verify_unchanged_inputs(
    bundle_dir: Path,
    manifest_raw: bytes,
    manifest: dict,
) -> None:
    try:
        current_manifest = (bundle_dir / "manifest.json").read_bytes()
    except OSError as error:
        raise InputIntegrityError(f"cannot re-read manifest: {error}") from error
    if hashlib.sha256(current_manifest).digest() != hashlib.sha256(manifest_raw).digest():
        raise InputIntegrityError("manifest changed while process stream was read")
    _verify_hash(bundle_dir / _PROCESS_NAME, manifest.get("compressed_process_sha256"), "compressed process")
    _verify_hash(bundle_dir / _SUMMARY_NAME, manifest.get("summary_json_sha256"), "summary JSON")
    _verify_hash(
        bundle_dir / _SUMMARY_MARKDOWN_NAME,
        manifest.get("summary_markdown_sha256"),
        "summary Markdown",
    )


def analyze_localization_failures(
    bundle_dir: Path,
    *,
    verify_hashes: bool = True,
    output_dir: Path | None = None,
    max_examples_per_bucket: int = 5,
) -> dict:
    del output_dir, max_examples_per_bucket
    bundle_dir = Path(bundle_dir)
    manifest, manifest_raw = _read_object(bundle_dir / "manifest.json", "manifest")
    if manifest.get("termination_reason") != "completed":
        raise InputIntegrityError("manifest termination reason must be completed")
    if manifest.get("estimator_contract") != ESTIMATOR_CONTRACT_ID:
        raise InputIntegrityError("manifest estimator contract does not match")
    manifest_settings = _require_mapping(manifest.get("settings"), "manifest settings")
    summary, _ = _read_object(bundle_dir / _SUMMARY_NAME, "summary JSON")
    if manifest_settings != _require_mapping(summary.get("settings"), "summary settings"):
        raise InputIntegrityError("manifest and summary settings do not match")
    if verify_hashes:
        _verify_hash(bundle_dir / _SUMMARY_NAME, manifest.get("summary_json_sha256"), "summary JSON")
        _verify_hash(
            bundle_dir / _SUMMARY_MARKDOWN_NAME,
            manifest.get("summary_markdown_sha256"),
            "summary Markdown",
        )
    manifest["_verify_hashes"] = verify_hashes
    observed_rows = 0
    primary_rows = 0
    _, _, _, graph_cases, _ = _stream_dimensions(summary)
    cases = _empty_failure_budget(graph_cases)
    task3_cases = {graph_case: _empty_task3_case() for graph_case in graph_cases}
    persistence: dict[tuple[int, str, int], dict] = {}
    seed_counts = {seed: {"dyn_up": 0, "fix_up": 0, "dyn_invalid": 0, "fix_invalid": 0}
                   for seed in manifest_settings["run_seeds"]}
    predecessor_group: tuple[int, str, int] | None = None
    predecessors: dict[int, dict] = {}
    for row in _iter_verified_rows(bundle_dir, manifest, summary):
        observed_rows += 1
        group = (row["seed"], row["graph_case"], row["frame_index"])
        if group != predecessor_group:
            predecessor_group = group
            predecessors = {}
        if row["primary_statistics"]:
            primary_rows += 1
            attempt_class = _attempt_class(row)
            case = cases[row["graph_case"]]
            case["failure_budget"]["denominator"] += 1
            case["failure_budget"]["counts"][attempt_class] += 1
            case["attempt_status_counts"][row["attempt_status"]] += 1
            _record_reason_label(case, row.get("attempt_failure_reason"))
            propagation_depth = _record_lineage(case, row, attempt_class, predecessors)
            predecessors[row["robot_id"]] = {
                "robot_id": row["robot_id"],
                "attempt_class": attempt_class,
                "attempt_status": row["attempt_status"],
                "retained_status": row["status"],
                "attempt_failure_reason": row.get("attempt_failure_reason"),
                "propagation_depth": propagation_depth,
            }
            task3 = task3_cases[row["graph_case"]]
            depth = row.get("squad_local_index", row["robot_id"])
            if type(depth) is int and 1 <= depth <= 7:
                _record_budget(task3["strata"]["depth"][str(depth)], attempt_class)
            time_key = _time_bin(row["frame_index"])
            if time_key is not None:
                _record_budget(task3["strata"]["time"][time_key], attempt_class)
            if row["attempt_status"] == "converged":
                active = row.get("active_references", {})
                if isinstance(active, dict):
                    base = active.get("base_ids", [])
                    uav = active.get("uav_ids", [])
                    if isinstance(base, list) and isinstance(uav, list):
                        task3["strata"]["reference_count"][_reference_bin(len(base) + len(uav))]["denominator"] += 1
                for field, bins, bucket_fn in (("phi_condition", "phi_condition", _condition_bin), ("phi_min", "phi_min", _min_eigen_bin)):
                    value = row.get(field)
                    if type(value) in (int, float) and math.isfinite(value):
                        bucket = bucket_fn(value)
                        if bucket is not None:
                            task3["strata"][bins][bucket]["denominator"] += 1
                _record_calibration(task3, row)
            key = (row["seed"], row["graph_case"], row["robot_id"])
            state = persistence.setdefault(key, {"first_primary_converged_frame": None,
                "primary_frames_before_first_convergence": 0, "upstream_current": 0,
                "upstream_longest": 0, "wnls_current": 0, "wnls_longest": 0})
            if row["attempt_status"] == "converged" and state["first_primary_converged_frame"] is None:
                state["first_primary_converged_frame"] = row["frame_index"]
            if state["first_primary_converged_frame"] is None:
                state["primary_frames_before_first_convergence"] += 1
            state["upstream_current"] = state["upstream_current"] + 1 if attempt_class == "upstream_unavailable" else 0
            state["wnls_current"] = state["wnls_current"] + 1 if attempt_class == "wnls_nonconvergence" else 0
            state["upstream_longest"] = max(state["upstream_longest"], state["upstream_current"])
            state["wnls_longest"] = max(state["wnls_longest"], state["wnls_current"])
            record = seed_counts[row["seed"]]
            prefix = "dyn" if row["graph_case"] == "dynamic_dag_wnls" else "fix"
            if attempt_class == "upstream_unavailable": record[f"{prefix}_up"] += 1
            if row["attempt_status"] == "invalid": record[f"{prefix}_invalid"] += 1
        else:
            attempt_class = _attempt_class(row)
            depth = row.get("squad_local_index", row["robot_id"])
            if type(depth) is int and 1 <= depth <= 7:
                _record_budget(task3_cases[row["graph_case"]]["initialization"]["by_depth"][str(depth)], attempt_class)
    if verify_hashes:
        _verify_unchanged_inputs(bundle_dir, manifest_raw, manifest)
    for graph_case in graph_cases:
        cases[graph_case].update(task3_cases[graph_case])
    for state in persistence.values():
        state["never_primary_converged"] = state["first_primary_converged_frame"] is None
        state["longest_upstream_unavailable_streak"] = state.pop("upstream_longest")
        state["longest_wnls_nonconvergence_streak"] = state.pop("wnls_longest")
        state.pop("upstream_current")
        state.pop("wnls_current")
    comparisons = {"dynamic_minus_fixed": {key: {
        "count_difference": cases["dynamic_dag_wnls"]["failure_budget"]["counts"][key] - cases["fixed_refs_wnls"]["failure_budget"]["counts"][key],
        "percentage_point_difference": (
            100 * cases["dynamic_dag_wnls"]["failure_budget"]["counts"][key] / cases["dynamic_dag_wnls"]["failure_budget"]["denominator"]
            - 100 * cases["fixed_refs_wnls"]["failure_budget"]["counts"][key] / cases["fixed_refs_wnls"]["failure_budget"]["denominator"]
        ) if cases["dynamic_dag_wnls"]["failure_budget"]["denominator"] and cases["fixed_refs_wnls"]["failure_budget"]["denominator"] else None
    } for key in _FAILURE_CLASSES}}
    return {
        "schema": SCHEMA_ID,
        "status": "completed",
        "integrity": {
            "observed_rows": observed_rows,
            "primary_rows": primary_rows,
            "hashes_match": verify_hashes,
        },
        "cases": cases,
        "initialization": {"persistence": persistence},
        "comparisons": comparisons,
        "bootstrap": _paired_seed_bootstrap(list(seed_counts.values())),
    }
