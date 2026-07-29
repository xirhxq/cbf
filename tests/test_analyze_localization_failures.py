"""Acceptance tests for localization-failure analysis input verification."""

from __future__ import annotations

import gzip
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from scripts.diagnostics.analyze_localization_failures import (
    InputIntegrityError,
    analyze_localization_failures,
)


PROCESS_NAME = "calibration.jsonl.gz"
SUMMARY_NAME = "summary.json"
SUMMARY_MARKDOWN_NAME = "summary.md"
GRAPH_CASES = ("dynamic_dag_wnls", "fixed_refs_wnls")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(8192), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _settings() -> dict:
    return {
        "run_seeds": [17],
        "graph_cases": list(GRAPH_CASES),
        "effective_frame_count": 1,
    }


def _bucket_counts(rows: int) -> dict:
    return {
        "attempt_status_counts": {"converged": rows} if rows else {},
        "status_counts": {"converged": rows} if rows else {},
    }


def _write_completed_bundle(bundle: Path, effective_frame_count: int = 1) -> None:
    bundle.mkdir()
    rows = [
        {
            "frame_index": frame_index,
            "seed": 17,
            "graph_case": graph_case,
            "robot_id": robot_id,
            "primary_statistics": frame_index != 0,
            "attempt_status": "converged",
            "status": "converged",
        }
        for frame_index in range(effective_frame_count)
        for graph_case in GRAPH_CASES
        for robot_id in (1, 2)
    ]
    lines = [
        json.dumps(row, sort_keys=True, separators=(",", ":")).encode("utf-8") + b"\n"
        for row in rows
    ]
    process_path = bundle / PROCESS_NAME
    with process_path.open("wb") as raw:
        with gzip.GzipFile(fileobj=raw, mode="wb", mtime=0) as compressed:
            for line in lines:
                compressed.write(line)

    summary = {
        "expected_process_rows": 4 * effective_frame_count,
        "process_rows": 4 * effective_frame_count,
        "settings": {**_settings(), "effective_frame_count": effective_frame_count},
        "graph_cases": {
            graph_case: {
                "overall": _bucket_counts(2 * (effective_frame_count - 1)),
                "initialization_frame": _bucket_counts(2),
            }
            for graph_case in GRAPH_CASES
        },
    }
    summary_path = bundle / SUMMARY_NAME
    summary_path.write_text(json.dumps(summary, sort_keys=True) + "\n", encoding="utf-8")
    summary_markdown_path = bundle / SUMMARY_MARKDOWN_NAME
    summary_markdown_path.write_text("# Completed localization diagnostic bundle\n", encoding="utf-8")
    manifest = {
        "termination_reason": "completed",
        "estimator_contract": "variable_weight_nls_full_residual_jacobian_v1",
        "settings": {**_settings(), "effective_frame_count": effective_frame_count},
        "compressed_process_sha256": _sha256(process_path),
        "decompressed_process_sha256": hashlib.sha256(b"".join(lines)).hexdigest(),
        "summary_json_sha256": _sha256(summary_path),
        "summary_markdown_sha256": _sha256(summary_markdown_path),
    }
    _write_manifest(bundle, manifest)


def _read_manifest(bundle: Path) -> dict:
    return json.loads((bundle / "manifest.json").read_text(encoding="utf-8"))


def _write_manifest(bundle: Path, manifest: dict) -> None:
    (bundle / "manifest.json").write_text(
        json.dumps(manifest, sort_keys=True) + "\n", encoding="utf-8"
    )


def _process_lines(bundle: Path) -> list[bytes]:
    with gzip.open(bundle / PROCESS_NAME, "rb") as stream:
        return list(stream)


def _rewrite_process(bundle: Path, lines: list[bytes]) -> None:
    process_path = bundle / PROCESS_NAME
    with process_path.open("wb") as raw:
        with gzip.GzipFile(fileobj=raw, mode="wb", mtime=0) as compressed:
            for line in lines:
                compressed.write(line)
    manifest = _read_manifest(bundle)
    manifest["compressed_process_sha256"] = _sha256(process_path)
    manifest["decompressed_process_sha256"] = hashlib.sha256(b"".join(lines)).hexdigest()
    _write_manifest(bundle, manifest)


def _read_summary(bundle: Path) -> dict:
    return json.loads((bundle / SUMMARY_NAME).read_text(encoding="utf-8"))


def _write_summary(bundle: Path, summary: dict) -> None:
    summary_path = bundle / SUMMARY_NAME
    summary_path.write_text(json.dumps(summary, sort_keys=True) + "\n", encoding="utf-8")
    manifest = _read_manifest(bundle)
    manifest["summary_json_sha256"] = _sha256(summary_path)
    _write_manifest(bundle, manifest)


class LocalizationFailureInputTests(unittest.TestCase):
    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.bundle = Path(self._temporary_directory.name) / "bundle"
        _write_completed_bundle(self.bundle)

    def tearDown(self) -> None:
        self._temporary_directory.cleanup()

    def test_accepts_complete_bundle_and_reports_exact_source_identity(self) -> None:
        report = analyze_localization_failures(self.bundle)
        self.assertEqual(report["schema"], "cbf2026-localization-failure-analysis-v1")
        self.assertEqual(report["status"], "completed")
        self.assertEqual(report["integrity"]["observed_rows"], 4)
        self.assertEqual(report["integrity"]["primary_rows"], 0)
        self.assertTrue(report["integrity"]["hashes_match"])

    def test_reports_primary_rows_from_the_verified_stream(self) -> None:
        two_frame_bundle = Path(self._temporary_directory.name) / "two-frame-bundle"
        _write_completed_bundle(two_frame_bundle, effective_frame_count=2)
        self.assertEqual(
            analyze_localization_failures(two_frame_bundle)["integrity"]["primary_rows"], 4
        )

    def test_accepts_extra_summary_statistics(self) -> None:
        summary = _read_summary(self.bundle)
        summary["graph_cases"]["dynamic_dag_wnls"]["overall"]["mean_iterations"] = 3.5
        _write_summary(self.bundle, summary)
        self.assertEqual(analyze_localization_failures(self.bundle)["integrity"]["observed_rows"], 4)

    def test_rejects_a_corrupted_compressed_byte(self) -> None:
        process_path = self.bundle / PROCESS_NAME
        content = bytearray(process_path.read_bytes())
        content[len(content) // 2] ^= 1
        process_path.write_bytes(bytes(content))
        self._assert_integrity_error()

    def test_rejects_a_wrong_decompressed_process_hash(self) -> None:
        manifest = _read_manifest(self.bundle)
        manifest["decompressed_process_sha256"] = "0" * 64
        _write_manifest(self.bundle, manifest)
        self._assert_integrity_error()

    def test_rejects_summary_json_changed_after_manifest_creation(self) -> None:
        (self.bundle / SUMMARY_NAME).write_text("{}\n", encoding="utf-8")
        self._assert_integrity_error()

    def test_rejects_summary_markdown_changed_after_manifest_creation(self) -> None:
        (self.bundle / SUMMARY_MARKDOWN_NAME).write_text("changed\n", encoding="utf-8")
        self._assert_integrity_error()

    def test_rejects_wrong_estimator_contract(self) -> None:
        manifest = _read_manifest(self.bundle)
        manifest["estimator_contract"] = "other"
        _write_manifest(self.bundle, manifest)
        self._assert_integrity_error()

    def test_rejects_nonterminal_termination_reason(self) -> None:
        manifest = _read_manifest(self.bundle)
        manifest["termination_reason"] = "interrupted"
        _write_manifest(self.bundle, manifest)
        self._assert_integrity_error()

    def test_rejects_a_row_outside_the_canonical_key_order(self) -> None:
        lines = _process_lines(self.bundle)
        row = json.loads(lines[0])
        row["robot_id"] = 2
        lines[0] = json.dumps(row, sort_keys=True, separators=(",", ":")).encode() + b"\n"
        _rewrite_process(self.bundle, lines)
        self._assert_integrity_error()

    def test_rejects_a_duplicate_at_the_canonical_expected_key_comparison(self) -> None:
        lines = _process_lines(self.bundle)
        lines[1] = lines[0]
        _rewrite_process(self.bundle, lines)
        with self.assertRaisesRegex(InputIntegrityError, "canonical expected key"):
            analyze_localization_failures(self.bundle)

    def test_rejects_a_boolean_key_where_an_integer_is_required(self) -> None:
        lines = _process_lines(self.bundle)
        row = json.loads(lines[0])
        row["frame_index"] = True
        lines[0] = json.dumps(row, sort_keys=True, separators=(",", ":")).encode() + b"\n"
        _rewrite_process(self.bundle, lines)
        self._assert_integrity_error()

    def test_rejects_primary_statistics_that_disagrees_with_frame_index(self) -> None:
        lines = _process_lines(self.bundle)
        row = json.loads(lines[0])
        row["primary_statistics"] = True
        lines[0] = json.dumps(row, sort_keys=True, separators=(",", ":")).encode() + b"\n"
        _rewrite_process(self.bundle, lines)
        summary = _read_summary(self.bundle)
        summary["graph_cases"]["dynamic_dag_wnls"]["overall"] = _bucket_counts(1)
        summary["graph_cases"]["dynamic_dag_wnls"]["initialization_frame"] = _bucket_counts(1)
        _write_summary(self.bundle, summary)
        self._assert_integrity_error()

    def test_rejects_an_arbitrary_attempt_or_retained_status(self) -> None:
        lines = _process_lines(self.bundle)
        row = json.loads(lines[0])
        row["attempt_status"] = "arbitrary"
        row["status"] = "arbitrary"
        lines[0] = json.dumps(row, sort_keys=True, separators=(",", ":")).encode() + b"\n"
        _rewrite_process(self.bundle, lines)
        summary = _read_summary(self.bundle)
        summary["graph_cases"]["dynamic_dag_wnls"]["initialization_frame"] = {
            "attempt_status_counts": {"arbitrary": 1, "converged": 1},
            "status_counts": {"arbitrary": 1, "converged": 1},
        }
        _write_summary(self.bundle, summary)
        self._assert_integrity_error()

    def test_rejects_wrong_expected_process_row_count(self) -> None:
        summary = _read_summary(self.bundle)
        summary["expected_process_rows"] = 6
        _write_summary(self.bundle, summary)
        self._assert_integrity_error()

    def test_rejects_wrong_primary_overall_attempt_counts(self) -> None:
        self._mutate_summary_count("overall", "attempt_status_counts", "attempted")

    def test_rejects_wrong_primary_overall_status_counts(self) -> None:
        self._mutate_summary_count("overall", "status_counts", "retained")

    def test_rejects_wrong_initialization_attempt_counts(self) -> None:
        self._mutate_summary_count("initialization_frame", "attempt_status_counts", "attempted")

    def test_rejects_wrong_initialization_status_counts(self) -> None:
        self._mutate_summary_count("initialization_frame", "status_counts", "retained")

    def test_rejects_nonfinite_json_tokens(self) -> None:
        lines = _process_lines(self.bundle)
        lines[0] = lines[0].replace(b'"frame_index":0', b'"frame_index":NaN', 1)
        _rewrite_process(self.bundle, lines)
        self._assert_integrity_error()

    def test_rejects_expected_rows_not_divisible_by_key_dimensions(self) -> None:
        summary = _read_summary(self.bundle)
        summary["expected_process_rows"] = 3
        _write_summary(self.bundle, summary)
        self._assert_integrity_error()

    def test_rejects_expected_rows_that_produce_zero_robots(self) -> None:
        summary = _read_summary(self.bundle)
        summary["expected_process_rows"] = 0
        summary["process_rows"] = 0
        _write_summary(self.bundle, summary)
        self._assert_integrity_error()

    def _mutate_summary_count(self, block: str, counter: str, value: str) -> None:
        summary = _read_summary(self.bundle)
        counts = summary["graph_cases"]["dynamic_dag_wnls"][block][counter]
        counts[value] = counts.get(value, 0) + 1
        _write_summary(self.bundle, summary)
        self._assert_integrity_error()

    def _assert_integrity_error(self) -> None:
        with self.assertRaises(InputIntegrityError):
            analyze_localization_failures(self.bundle)
