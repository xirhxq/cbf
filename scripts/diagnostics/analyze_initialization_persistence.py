"""Read-only Gate 1 analysis for WNLS initialization persistence."""

from __future__ import annotations

import math
import hashlib
import json
from collections.abc import Callable
from pathlib import Path

import numpy as np

from scripts.diagnostics.analyze_localization_failures import (
    AnalysisLimitError,
    InputIntegrityError,
    _iter_verified_rows,
    _read_object,
    _stream_dimensions,
    _verify_unchanged_inputs,
)
from scripts.diagnostics.run_diagnostic import (
    HARD_FLOOR_BYTES,
    OUTPUT_ROOT_CAP_BYTES,
    DiskSpaceError,
    allocated_bytes,
    available_bytes,
    require_start_space,
)


SCHEMA_ID = "cbf2026-initialization-persistence-v1"
EXACT_REASON = "non-finite or malformed WNLS input"
_PROCESS_NAME = "calibration.jsonl.gz"
_SUMMARY_NAME = "summary.json"
_SUMMARY_MARKDOWN_NAME = "summary.md"
_OUTPUT_JSON_NAME = "initialization-persistence.json"
_OUTPUT_MARKDOWN_NAME = "initialization-persistence.md"
_INCOMPLETE_OUTPUT_CAP_BYTES = 10_000_000
_LIVE_CHECK_INTERVAL_ROWS = 10_000
_DOMINANCE_GATE = 0.95
_CATEGORY_NAMES = (
    "exclusive_prior_self_estimate_missing_or_malformed",
    "prior_missing_and_recorded_measurement_invalid",
    "recorded_measurement_missing_or_nonfinite",
    "unresolved_compound_validator",
)
_MAX_REASON_LABELS = 32
_MAX_SEED_LABELS = 64
_MAX_FRAME_LABELS = 500


def _finite_estimate(row: dict | None) -> bool:
    if not isinstance(row, dict):
        return False
    try:
        estimate = np.asarray(row.get("estimate"), dtype=float)
    except (TypeError, ValueError, OverflowError):
        return False
    return estimate.shape == (2,) and np.isfinite(estimate).all()


def _recorded_measurements_finite(row: dict) -> bool:
    records = row.get("measurements")
    if not isinstance(records, list) or not records:
        return False
    values = [record.get("noisy_range") for record in records if isinstance(record, dict)]
    return len(values) == len(records) and all(
        type(value) in (int, float) and math.isfinite(value)
        for value in values
    )


def classify_exact_reason(row: dict, preceding_row: dict | None) -> str:
    """Return one frozen mutually exclusive Gate 1 category."""
    prior_missing = not _finite_estimate(preceding_row)
    measurements_finite = _recorded_measurements_finite(row)
    if prior_missing and measurements_finite:
        return "exclusive_prior_self_estimate_missing_or_malformed"
    if prior_missing and not measurements_finite:
        return "prior_missing_and_recorded_measurement_invalid"
    if not measurements_finite:
        return "recorded_measurement_missing_or_nonfinite"
    return "unresolved_compound_validator"


def _empty_categories() -> dict[str, int]:
    return {category: 0 for category in _CATEGORY_NAMES}


def _empty_case() -> dict:
    return {
        "exact_reason_rows": 0,
        "categories": _empty_categories(),
        "dominance_fraction": None,
        "gate_passed": False,
        "by_seed": {},
        "by_frame": {},
        "by_depth": {},
        "preceding_attempt_reasons": {},
        "examples": {category: [] for category in _CATEGORY_NAMES},
        "other_non_upstream_invalid_rows": 0,
    }


def _increment_category(target: dict[str, dict[str, int]], key: str, category: str) -> None:
    target.setdefault(key, _empty_categories())[category] += 1


def _bounded_counter_key(counter: dict[str, dict[str, int]], key: str, limit: int) -> str:
    return key if key in counter or len(counter) < limit else "<other>"


def _depth_key(row: dict) -> str:
    depth = row.get("squad_local_index", row["robot_id"])
    return str(depth) if type(depth) is int and 1 <= depth <= 7 else "unstratified"


def _reason_label(value: object) -> str:
    return value if isinstance(value, str) and value else "<missing-or-malformed>"


def _increment_bounded_reason(counter: dict[str, int], value: object) -> None:
    label = _reason_label(value)
    if label not in counter and len(counter) >= _MAX_REASON_LABELS:
        label = "<other>"
    counter[label] = counter.get(label, 0) + 1


def _nearest_existing_ancestor(path: Path) -> Path:
    candidate = path
    while not candidate.exists():
        if candidate.parent == candidate:
            raise ValueError(f"no existing ancestor for output path {path}")
        candidate = candidate.parent
    return candidate


def _validate_output_path(bundle_dir: Path, output_dir: Path) -> tuple[Path, Path]:
    incomplete_lexical = output_dir.with_name(f"{output_dir.name}.incomplete")
    if output_dir.is_symlink() or incomplete_lexical.is_symlink():
        raise AnalysisLimitError("output directory and its incomplete sibling must not be symlinks")
    bundle = bundle_dir.resolve()
    output = output_dir.resolve()
    if output == bundle or bundle in output.parents or output in bundle.parents:
        raise AnalysisLimitError("output directory must be separate from the source bundle")
    incomplete = output.with_name(f"{output.name}.incomplete")
    if output.exists() or incomplete.exists():
        raise FileExistsError("output directory or its incomplete sibling already exists")
    if not output.parent.exists():
        raise ValueError("output directory parent must already exist")
    return output, incomplete


def _require_live_space(path: Path) -> None:
    free = available_bytes(path)
    if free < HARD_FLOOR_BYTES:
        raise AnalysisLimitError(f"available={free} below live threshold={HARD_FLOOR_BYTES}")


def _check_output_limits(output_dir: Path, incomplete_dir: Path) -> None:
    output_root_bytes = allocated_bytes(output_dir.parent)
    if output_root_bytes > OUTPUT_ROOT_CAP_BYTES:
        raise AnalysisLimitError(
            f"output root allocated={output_root_bytes} exceeds cap={OUTPUT_ROOT_CAP_BYTES}"
        )
    incomplete_bytes = allocated_bytes(incomplete_dir)
    if incomplete_bytes > _INCOMPLETE_OUTPUT_CAP_BYTES:
        raise AnalysisLimitError(
            f"incomplete output allocated={incomplete_bytes} exceeds cap={_INCOMPLETE_OUTPUT_CAP_BYTES}"
        )


def _render_markdown(report: dict) -> bytes:
    lines = [
        "# Initialization persistence",
        "",
        f"- Status: {report['status']}",
        f"- Gate passed: {report['gate_passed']}",
        "",
    ]
    for graph_case in sorted(report["cases"]):
        case = report["cases"][graph_case]
        lines.extend([
            f"## {graph_case}",
            "",
            f"- Exact-reason rows: {case['exact_reason_rows']}",
            f"- Exclusive-prior dominance: {case['dominance_fraction']}",
            f"- Gate passed: {case['gate_passed']}",
        ])
        lines.extend(f"- {category}: {case['categories'][category]}" for category in _CATEGORY_NAMES)
        lines.append("")
    return "\n".join(lines).encode("utf-8")


def _write_output(report: dict, output_dir: Path, incomplete_dir: Path) -> None:
    json_path = incomplete_dir / _OUTPUT_JSON_NAME
    markdown_path = incomplete_dir / _OUTPUT_MARKDOWN_NAME
    try:
        _require_live_space(incomplete_dir)
        _check_output_limits(output_dir, incomplete_dir)
        json_path.write_bytes(
            (json.dumps(report, sort_keys=True, indent=2, allow_nan=False) + "\n").encode("utf-8")
        )
        _require_live_space(incomplete_dir)
        _check_output_limits(output_dir, incomplete_dir)
        markdown_path.write_bytes(_render_markdown(report))
        _require_live_space(incomplete_dir)
        _check_output_limits(output_dir, incomplete_dir)
        incomplete_dir.replace(output_dir)
    except Exception:
        json_path.unlink(missing_ok=True)
        markdown_path.unlink(missing_ok=True)
        raise


def _source_record(bundle_dir: Path, manifest_raw: bytes, manifest: dict) -> dict:
    return {
        "bundle_dir": str(bundle_dir.resolve()),
        "hashes": {
            "manifest_sha256": hashlib.sha256(manifest_raw).hexdigest(),
            "summary_json_sha256": manifest["summary_json_sha256"],
            "summary_markdown_sha256": manifest["summary_markdown_sha256"],
            "compressed_process_sha256": manifest["compressed_process_sha256"],
            "decompressed_process_sha256": manifest["decompressed_process_sha256"],
        },
    }


def analyze_initialization_persistence(
    bundle_dir: Path,
    *,
    verify_hashes: bool = True,
    output_dir: Path | None = None,
    max_examples_per_category: int = 5,
) -> dict:
    """Stream, verify, classify, and optionally publish one compact report."""
    if type(max_examples_per_category) is not int or not 0 <= max_examples_per_category <= 20:
        raise ValueError("max_examples_per_category must be between 0 and 20")
    bundle_dir = Path(bundle_dir)
    output_path: Path | None = None
    incomplete_path: Path | None = None
    live_guard: Callable[[], None] | None = None
    if output_dir is not None:
        output_path, incomplete_path = _validate_output_path(bundle_dir, Path(output_dir))
        try:
            require_start_space(_nearest_existing_ancestor(output_path))
        except DiskSpaceError as error:
            raise AnalysisLimitError(str(error)) from error
        incomplete_path.mkdir()
        _require_live_space(incomplete_path)
        live_guard = lambda: _require_live_space(incomplete_path)

    manifest, manifest_raw = _read_object(bundle_dir / "manifest.json", "manifest")
    if manifest.get("termination_reason") != "completed":
        raise InputIntegrityError("manifest termination reason must be completed")
    if manifest.get("estimator_contract") != "variable_weight_nls_full_residual_jacobian_v1":
        raise InputIntegrityError("manifest estimator contract does not match")
    summary, _ = _read_object(bundle_dir / _SUMMARY_NAME, "summary JSON")
    if manifest.get("settings") != summary.get("settings"):
        raise InputIntegrityError("manifest and summary settings do not match")
    if verify_hashes:
        manifest_for_stream = {**manifest, "_verify_hashes": True}
        # The verified row reader validates the compressed stream before yielding rows.
    else:
        manifest_for_stream = {**manifest, "_verify_hashes": False}

    _, _, _, graph_cases, _ = _stream_dimensions(summary)
    cases = {graph_case: _empty_case() for graph_case in graph_cases}
    preceding: dict[tuple[int, str, int], dict] = {}
    observed_rows = 0
    non_upstream_invalid_rows = 0
    other_non_upstream_invalid_rows = 0
    for row in _iter_verified_rows(
        bundle_dir, manifest_for_stream, summary, live_guard=live_guard
    ):
        observed_rows += 1
        key = (row["seed"], row["graph_case"], row["robot_id"])
        preceding_row = preceding.get(key)
        case = cases[row["graph_case"]]
        reason = row.get("attempt_failure_reason")
        if row["attempt_status"] == "invalid" and reason != "invalid upstream UAV reference":
            non_upstream_invalid_rows += 1
            if reason != EXACT_REASON:
                other_non_upstream_invalid_rows += 1
                case["other_non_upstream_invalid_rows"] += 1
        if row["attempt_status"] == "invalid" and reason == EXACT_REASON:
            category = classify_exact_reason(row, preceding_row)
            case["exact_reason_rows"] += 1
            case["categories"][category] += 1
            _increment_category(
                case["by_seed"],
                _bounded_counter_key(case["by_seed"], str(row["seed"]), _MAX_SEED_LABELS),
                category,
            )
            _increment_category(
                case["by_frame"],
                _bounded_counter_key(case["by_frame"], str(row["frame_index"]), _MAX_FRAME_LABELS),
                category,
            )
            _increment_category(case["by_depth"], _depth_key(row), category)
            _increment_bounded_reason(
                case["preceding_attempt_reasons"],
                None if preceding_row is None else preceding_row.get("attempt_failure_reason"),
            )
            examples = case["examples"][category]
            if len(examples) < max_examples_per_category:
                examples.append({
                    "seed": row["seed"],
                    "frame_index": row["frame_index"],
                    "robot_id": row["robot_id"],
                    "preceding_attempt_failure_reason": (
                        None if preceding_row is None else preceding_row.get("attempt_failure_reason")
                    ),
                })
        preceding[key] = row

    _verify_unchanged_inputs(bundle_dir, manifest_raw, manifest)
    for case in cases.values():
        total = case["exact_reason_rows"]
        exclusive = case["categories"]["exclusive_prior_self_estimate_missing_or_malformed"]
        case["dominance_fraction"] = None if not total else exclusive / total
        case["gate_passed"] = (
            total > 0
            and case["dominance_fraction"] >= _DOMINANCE_GATE
            and case["categories"]["prior_missing_and_recorded_measurement_invalid"] == 0
        )
        del case["other_non_upstream_invalid_rows"]

    exact_rows = sum(case["exact_reason_rows"] for case in cases.values())
    report = {
        "schema": SCHEMA_ID,
        "status": "completed",
        "source": _source_record(bundle_dir, manifest_raw, manifest),
        "protocol": {"exact_reason": EXACT_REASON, "dominance_gate": _DOMINANCE_GATE},
        "integrity": {
            "observed_rows": observed_rows,
            "invalid_reason_reconciliation": (
                exact_rows + other_non_upstream_invalid_rows == non_upstream_invalid_rows
            ),
            "source_hashes_unchanged": True,
        },
        "cases": cases,
        "gate_passed": all(case["gate_passed"] for case in cases.values()),
    }
    if output_path is not None and incomplete_path is not None:
        _write_output(report, output_path, incomplete_path)
    return report
