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
_ROW_KEYS = {
    "frame",
    "seed",
    "graph_case",
    "robot_id",
    "primary",
    "initialization_frame",
}
_COUNT_KEYS = {"attempts", "retained"}


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


def _read_object(path: Path, description: str) -> dict:
    try:
        value = _strict_json_line(path.read_bytes())
    except (OSError, ValueError) as error:
        raise InputIntegrityError(f"invalid {description}: {error}") from error
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


def _verify_file_hash(bundle_dir: Path, record: object, description: str) -> Path:
    details = _require_mapping(record, description)
    path = bundle_dir / _require_string(details.get("path"), f"{description} path")
    expected_hash = _require_string(details.get("sha256"), f"{description} hash")
    try:
        observed_hash = _sha256_file(path)
    except OSError as error:
        raise InputIntegrityError(f"cannot read {description}: {error}") from error
    if observed_hash != expected_hash:
        raise InputIntegrityError(f"{description} hash does not match manifest")
    return path


def _stream_dimensions(summary: dict) -> tuple[int, list[int], list[str], int]:
    expected_rows = _require_nonnegative_int(summary.get("expected_rows"), "expected row count")
    effective_frame_count = _require_nonnegative_int(
        summary.get("effective_frame_count"), "effective frame count"
    )
    if effective_frame_count == 0:
        raise InputIntegrityError("effective frame count must be positive")
    run_seeds_value = summary.get("run_seeds")
    graph_cases_value = summary.get("graph_cases")
    if not isinstance(run_seeds_value, list) or not run_seeds_value:
        raise InputIntegrityError("run seeds must be a non-empty list")
    if not isinstance(graph_cases_value, dict) or not graph_cases_value:
        raise InputIntegrityError("graph cases must be a non-empty object")
    run_seeds = []
    for seed in run_seeds_value:
        if type(seed) is not int:
            raise InputIntegrityError("every run seed must be an integer")
        run_seeds.append(seed)
    graph_cases = list(graph_cases_value)
    if any(not isinstance(graph_case, str) or not graph_case for graph_case in graph_cases):
        raise InputIntegrityError("every graph case must be a non-empty string")
    dimension = effective_frame_count * len(run_seeds) * len(graph_cases)
    if expected_rows % dimension:
        raise InputIntegrityError("expected row count is not divisible by key dimensions")
    robot_count = expected_rows // dimension
    if robot_count <= 0:
        raise InputIntegrityError("expected row count produces a non-positive robot count")
    return expected_rows, run_seeds, graph_cases, robot_count


def _row_counts(row: dict, field: str) -> tuple[int, int]:
    counts = _require_mapping(row.get(field), f"row {field}")
    if set(counts) != _COUNT_KEYS:
        raise InputIntegrityError(f"row {field} must contain attempts and retained only")
    attempts = _require_nonnegative_int(counts["attempts"], f"row {field} attempts")
    retained = _require_nonnegative_int(counts["retained"], f"row {field} retained")
    if retained > attempts:
        raise InputIntegrityError(f"row {field} retained count exceeds attempts")
    return attempts, retained


def _expected_keys(
    effective_frame_count: int,
    run_seeds: list[int],
    graph_cases: list[str],
    robot_count: int,
) -> Iterator[tuple[int, int, str, int]]:
    for frame in range(effective_frame_count):
        for seed in run_seeds:
            for graph_case in graph_cases:
                for robot_id in range(1, robot_count + 1):
                    yield frame, seed, graph_case, robot_id


def _validate_summary_counts(summary: dict, counts: dict[str, dict[str, dict[str, int]]]) -> None:
    cases = _require_mapping(summary.get("graph_cases"), "graph cases")
    for graph_case, observed in counts.items():
        case_summary = _require_mapping(cases.get(graph_case), f"summary graph case {graph_case}")
        for block in ("overall", "initialization_frame"):
            declared = _require_mapping(case_summary.get(block), f"summary {graph_case} {block}")
            if set(declared) != _COUNT_KEYS:
                raise InputIntegrityError(
                    f"summary {graph_case} {block} must contain attempts and retained only"
                )
            for count in ("attempts", "retained"):
                expected = _require_nonnegative_int(
                    declared[count], f"summary {graph_case} {block} {count}"
                )
                if observed[block][count] != expected:
                    raise InputIntegrityError(
                        f"summary {graph_case} {block} {count} does not match process stream"
                    )


def _iter_verified_rows(bundle_dir: Path, manifest: dict, summary: dict) -> Iterator[dict]:
    process = _require_mapping(manifest.get("process"), "process record")
    process_path = bundle_dir / _require_string(process.get("path"), "process path")
    verify_hashes = bool(manifest.get("_verify_hashes", True))
    if verify_hashes:
        expected_compressed_hash = _require_string(
            process.get("compressed_sha256"), "compressed process hash"
        )
        try:
            observed_compressed_hash = _sha256_file(process_path)
        except OSError as error:
            raise InputIntegrityError(f"cannot read compressed process stream: {error}") from error
        if observed_compressed_hash != expected_compressed_hash:
            raise InputIntegrityError("compressed process hash does not match manifest")
        expected_decompressed_hash = _require_string(
            process.get("decompressed_sha256"), "decompressed process hash"
        )
    else:
        expected_decompressed_hash = ""

    expected_rows, run_seeds, graph_cases, robot_count = _stream_dimensions(summary)
    cursor = _expected_keys(
        _require_nonnegative_int(summary["effective_frame_count"], "effective frame count"),
        run_seeds,
        graph_cases,
        robot_count,
    )
    digest = hashlib.sha256()
    counts = {
        graph_case: {
            "overall": {"attempts": 0, "retained": 0},
            "initialization_frame": {"attempts": 0, "retained": 0},
        }
        for graph_case in graph_cases
    }
    observed_rows = 0
    try:
        with gzip.open(process_path, "rb") as stream:
            for raw in stream:
                digest.update(raw)
                try:
                    row = _strict_json_line(raw)
                except ValueError as error:
                    raise InputIntegrityError(f"invalid process JSON line: {error}") from error
                if set(row) != _ROW_KEYS:
                    raise InputIntegrityError("process row has unexpected fields")
                try:
                    expected_key = next(cursor)
                except StopIteration as error:
                    raise InputIntegrityError("process stream has more rows than expected") from error
                actual_key = (
                    row["frame"],
                    row["seed"],
                    row["graph_case"],
                    row["robot_id"],
                )
                if actual_key != expected_key:
                    raise InputIntegrityError(
                        "row key does not match canonical expected key "
                        f"{expected_key!r}: got {actual_key!r}"
                    )
                primary_attempts, primary_retained = _row_counts(row, "primary")
                initialization_attempts, initialization_retained = _row_counts(
                    row, "initialization_frame"
                )
                graph_counts = counts[row["graph_case"]]
                graph_counts["overall"]["attempts"] += primary_attempts
                graph_counts["overall"]["retained"] += primary_retained
                graph_counts["initialization_frame"]["attempts"] += initialization_attempts
                graph_counts["initialization_frame"]["retained"] += initialization_retained
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
    _validate_summary_counts(summary, counts)


def analyze_localization_failures(
    bundle_dir: Path,
    *,
    verify_hashes: bool = True,
    output_dir: Path | None = None,
    max_examples_per_bucket: int = 5,
) -> dict:
    del output_dir, max_examples_per_bucket
    bundle_dir = Path(bundle_dir)
    manifest = _read_object(bundle_dir / "manifest.json", "manifest")
    if manifest.get("status") != "completed":
        raise InputIntegrityError("manifest status must be completed")
    if manifest.get("termination_reason") != "completed":
        raise InputIntegrityError("manifest termination reason must be completed")
    if manifest.get("estimator_contract") != ESTIMATOR_CONTRACT_ID:
        raise InputIntegrityError("manifest estimator contract does not match")
    if verify_hashes:
        _verify_file_hash(bundle_dir, manifest.get("summary"), "summary JSON")
        _verify_file_hash(bundle_dir, manifest.get("summary_markdown"), "summary Markdown")
    summary_record = _require_mapping(manifest.get("summary"), "summary record")
    summary = _read_object(
        bundle_dir / _require_string(summary_record.get("path"), "summary path"), "summary JSON"
    )
    manifest["_verify_hashes"] = verify_hashes
    observed_rows = 0
    for _ in _iter_verified_rows(bundle_dir, manifest, summary):
        observed_rows += 1
    return {
        "schema": SCHEMA_ID,
        "status": "completed",
        "integrity": {
            "observed_rows": observed_rows,
            "primary_rows": 0,
            "hashes_match": verify_hashes,
        },
    }
