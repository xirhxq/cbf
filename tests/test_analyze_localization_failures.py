"""Acceptance tests for localization-failure analysis input verification."""

from __future__ import annotations

import gzip
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from scripts.diagnostics.analyze_localization_failures import (
    ESTIMATOR_CONTRACT_ID,
    InputIntegrityError,
    analyze_localization_failures,
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(8192), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_completed_bundle(bundle: Path) -> None:
    bundle.mkdir()
    rows = [
        {
            "frame": 0,
            "seed": 17,
            "graph_case": graph_case,
            "robot_id": robot_id,
            "primary": {"attempts": 1, "retained": 1},
            "initialization_frame": {"attempts": 1, "retained": 1},
        }
        for graph_case in ("dynamic_dag_wnls", "fixed_refs_wnls")
        for robot_id in (1, 2)
    ]
    lines = [
        json.dumps(row, sort_keys=True, separators=(",", ":")).encode("utf-8") + b"\n"
        for row in rows
    ]
    process_path = bundle / "process.jsonl.gz"
    with process_path.open("wb") as raw:
        with gzip.GzipFile(fileobj=raw, mode="wb", mtime=0) as compressed:
            for line in lines:
                compressed.write(line)

    summary = {
        "expected_rows": 4,
        "effective_frame_count": 1,
        "run_seeds": [17],
        "graph_cases": {
            graph_case: {
                "overall": {"attempts": 2, "retained": 2},
                "initialization_frame": {"attempts": 2, "retained": 2},
            }
            for graph_case in ("dynamic_dag_wnls", "fixed_refs_wnls")
        },
    }
    summary_path = bundle / "summary.json"
    summary_path.write_text(json.dumps(summary, sort_keys=True) + "\n", encoding="utf-8")
    summary_md_path = bundle / "summary.md"
    summary_md_path.write_text("# Completed localization diagnostic bundle\n", encoding="utf-8")
    decompressed = hashlib.sha256(b"".join(lines)).hexdigest()
    manifest = {
        "status": "completed",
        "estimator_contract": "variable_weight_nls_full_residual_jacobian_v1",
        "termination_reason": "completed",
        "process": {
            "path": "process.jsonl.gz",
            "compressed_sha256": _sha256(process_path),
            "decompressed_sha256": decompressed,
        },
        "summary": {"path": "summary.json", "sha256": _sha256(summary_path)},
        "summary_markdown": {"path": "summary.md", "sha256": _sha256(summary_md_path)},
    }
    (bundle / "manifest.json").write_text(
        json.dumps(manifest, sort_keys=True) + "\n", encoding="utf-8"
    )


def _read_manifest(bundle: Path) -> dict:
    return json.loads((bundle / "manifest.json").read_text(encoding="utf-8"))


def _write_manifest(bundle: Path, manifest: dict) -> None:
    (bundle / "manifest.json").write_text(
        json.dumps(manifest, sort_keys=True) + "\n", encoding="utf-8"
    )


def _refresh_process_hashes(bundle: Path) -> None:
    process_path = bundle / "process.jsonl.gz"
    with gzip.open(process_path, "rb") as stream:
        decompressed = hashlib.sha256(stream.read()).hexdigest()
    manifest = _read_manifest(bundle)
    manifest["process"]["compressed_sha256"] = _sha256(process_path)
    manifest["process"]["decompressed_sha256"] = decompressed
    _write_manifest(bundle, manifest)


def _rewrite_process(bundle: Path, lines: list[bytes]) -> None:
    process_path = bundle / "process.jsonl.gz"
    with process_path.open("wb") as raw:
        with gzip.GzipFile(fileobj=raw, mode="wb", mtime=0) as compressed:
            for line in lines:
                compressed.write(line)
    _refresh_process_hashes(bundle)


def _process_lines(bundle: Path) -> list[bytes]:
    with gzip.open(bundle / "process.jsonl.gz", "rb") as stream:
        return list(stream)


def _refresh_summary_hash(bundle: Path) -> None:
    manifest = _read_manifest(bundle)
    manifest["summary"]["sha256"] = _sha256(bundle / "summary.json")
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

    def test_rejects_a_corrupted_compressed_byte(self) -> None:
        process_path = self.bundle / "process.jsonl.gz"
        content = bytearray(process_path.read_bytes())
        content[len(content) // 2] ^= 1
        process_path.write_bytes(bytes(content))
        with self.assertRaises(InputIntegrityError):
            analyze_localization_failures(self.bundle)

    def test_rejects_a_wrong_decompressed_process_hash(self) -> None:
        manifest = _read_manifest(self.bundle)
        manifest["process"]["decompressed_sha256"] = "0" * 64
        _write_manifest(self.bundle, manifest)
        with self.assertRaises(InputIntegrityError):
            analyze_localization_failures(self.bundle)

    def test_rejects_summary_json_changed_after_manifest_creation(self) -> None:
        (self.bundle / "summary.json").write_text("{}\n", encoding="utf-8")
        with self.assertRaises(InputIntegrityError):
            analyze_localization_failures(self.bundle)

    def test_rejects_summary_markdown_changed_after_manifest_creation(self) -> None:
        (self.bundle / "summary.md").write_text("changed\n", encoding="utf-8")
        with self.assertRaises(InputIntegrityError):
            analyze_localization_failures(self.bundle)

    def test_rejects_wrong_estimator_contract(self) -> None:
        manifest = _read_manifest(self.bundle)
        manifest["estimator_contract"] = "other"
        _write_manifest(self.bundle, manifest)
        with self.assertRaises(InputIntegrityError):
            analyze_localization_failures(self.bundle)

    def test_rejects_nonterminal_termination_reason(self) -> None:
        manifest = _read_manifest(self.bundle)
        manifest["termination_reason"] = "interrupted"
        _write_manifest(self.bundle, manifest)
        with self.assertRaises(InputIntegrityError):
            analyze_localization_failures(self.bundle)

    def test_rejects_a_row_outside_the_canonical_key_order(self) -> None:
        lines = _process_lines(self.bundle)
        row = json.loads(lines[0])
        row["robot_id"] = 2
        lines[0] = json.dumps(row, sort_keys=True, separators=(",", ":")).encode() + b"\n"
        _rewrite_process(self.bundle, lines)
        with self.assertRaises(InputIntegrityError):
            analyze_localization_failures(self.bundle)

    def test_rejects_a_duplicate_at_the_canonical_expected_key_comparison(self) -> None:
        lines = _process_lines(self.bundle)
        lines[1] = lines[0]
        _rewrite_process(self.bundle, lines)
        with self.assertRaisesRegex(InputIntegrityError, "canonical expected key"):
            analyze_localization_failures(self.bundle)

    def test_rejects_expected_rows_that_do_not_match_the_stream(self) -> None:
        summary = json.loads((self.bundle / "summary.json").read_text(encoding="utf-8"))
        summary["expected_rows"] = 6
        (self.bundle / "summary.json").write_text(json.dumps(summary) + "\n", encoding="utf-8")
        _refresh_summary_hash(self.bundle)
        with self.assertRaises(InputIntegrityError):
            analyze_localization_failures(self.bundle)

    def test_rejects_wrong_primary_overall_attempt_count(self) -> None:
        self._mutate_summary_count("dynamic_dag_wnls", "overall", "attempts")

    def test_rejects_wrong_primary_overall_retained_count(self) -> None:
        self._mutate_summary_count("dynamic_dag_wnls", "overall", "retained")

    def test_rejects_wrong_initialization_frame_attempt_count(self) -> None:
        self._mutate_summary_count("dynamic_dag_wnls", "initialization_frame", "attempts")

    def test_rejects_wrong_initialization_frame_retained_count(self) -> None:
        self._mutate_summary_count("dynamic_dag_wnls", "initialization_frame", "retained")

    def test_rejects_nonfinite_json_tokens(self) -> None:
        lines = _process_lines(self.bundle)
        lines[0] = lines[0].replace(b'"attempts":1', b'"attempts":NaN', 1)
        _rewrite_process(self.bundle, lines)
        with self.assertRaises(InputIntegrityError):
            analyze_localization_failures(self.bundle)

    def test_rejects_expected_rows_not_divisible_by_key_dimensions(self) -> None:
        summary = json.loads((self.bundle / "summary.json").read_text(encoding="utf-8"))
        summary["expected_rows"] = 3
        (self.bundle / "summary.json").write_text(json.dumps(summary) + "\n", encoding="utf-8")
        _refresh_summary_hash(self.bundle)
        with self.assertRaises(InputIntegrityError):
            analyze_localization_failures(self.bundle)

    def test_rejects_expected_rows_that_produce_zero_robots(self) -> None:
        summary = json.loads((self.bundle / "summary.json").read_text(encoding="utf-8"))
        summary["expected_rows"] = 0
        (self.bundle / "summary.json").write_text(json.dumps(summary) + "\n", encoding="utf-8")
        _refresh_summary_hash(self.bundle)
        with self.assertRaises(InputIntegrityError):
            analyze_localization_failures(self.bundle)

    def _mutate_summary_count(self, graph_case: str, block: str, count: str) -> None:
        summary = json.loads((self.bundle / "summary.json").read_text(encoding="utf-8"))
        summary["graph_cases"][graph_case][block][count] += 1
        (self.bundle / "summary.json").write_text(json.dumps(summary) + "\n", encoding="utf-8")
        _refresh_summary_hash(self.bundle)
        with self.assertRaises(InputIntegrityError):
            analyze_localization_failures(self.bundle)
