"""Read-only input verification for localization-failure analysis."""

from __future__ import annotations

import gzip
import hashlib
import json
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


class InputIntegrityError(RuntimeError):
    pass


class AnalysisLimitError(RuntimeError):
    pass


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
    return {
        graph_case: {
            "overall": {"attempt_status_counts": {}, "status_counts": {}},
            "initialization_frame": {"attempt_status_counts": {}, "status_counts": {}},
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
    for row in _iter_verified_rows(bundle_dir, manifest, summary):
        observed_rows += 1
        if row["primary_statistics"]:
            primary_rows += 1
    if verify_hashes:
        _verify_unchanged_inputs(bundle_dir, manifest_raw, manifest)
    return {
        "schema": SCHEMA_ID,
        "status": "completed",
        "integrity": {
            "observed_rows": observed_rows,
            "primary_rows": primary_rows,
            "hashes_match": verify_hashes,
        },
    }
