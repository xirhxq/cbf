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
    _attempt_class,
    _empty_failure_budget,
    _normalized_squared_error,
    _paired_seed_bootstrap,
    _q_bin,
    _ratio_bin,
    _record_lineage,
    analyze_localization_failures,
)


PROCESS_NAME = "calibration.jsonl.gz"
SUMMARY_NAME = "summary.json"
SUMMARY_MARKDOWN_NAME = "summary.md"
GRAPH_CASES = ("dynamic_dag_wnls", "fixed_refs_wnls")
STATUS_KEYS = ("converged", "stale", "invalid", "failed")


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
    counts = {status: 0 for status in STATUS_KEYS}
    counts["converged"] = rows
    return {
        "attempt_status_counts": counts.copy(),
        "status_counts": counts.copy(),
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
            "containment": True,
            "attempt_failure_reason": None,
            "measurements": [],
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


def _real_schema_row(
    frame_index: int,
    graph_case: str,
    robot_id: int,
    *,
    attempt_status: str = "converged",
    status: str = "converged",
    containment: bool = True,
    attempt_failure_reason: str | None = None,
    measurements: list[dict] | None = None,
) -> dict:
    return {
        "frame_index": frame_index,
        "seed": 17,
        "graph_case": graph_case,
        "robot_id": robot_id,
        "squad_local_index": robot_id,
        "primary_statistics": frame_index != 0,
        "attempt_status": attempt_status,
        "status": status,
        "containment": containment,
        "attempt_failure_reason": attempt_failure_reason,
        "measurements": [] if measurements is None else measurements,
    }


def _write_real_schema_bundle(bundle: Path, rows: list[dict], robot_count: int) -> None:
    """Write a self-consistent canonical bundle without using analyzer helpers."""
    bundle.mkdir()
    lines = [
        json.dumps(row, sort_keys=True, separators=(",", ":")).encode("utf-8") + b"\n"
        for row in rows
    ]
    process_path = bundle / PROCESS_NAME
    with process_path.open("wb") as raw:
        with gzip.GzipFile(fileobj=raw, mode="wb", mtime=0) as compressed:
            for line in lines:
                compressed.write(line)

    def status_counts(selected: list[dict], key: str) -> dict[str, int]:
        counts = {status: 0 for status in STATUS_KEYS}
        for row in selected:
            counts[row[key]] += 1
        return counts

    frame_count = max(row["frame_index"] for row in rows) + 1
    graph_cases = ["dynamic_dag_wnls", "fixed_refs_wnls"]
    summary_cases = {}
    for graph_case in graph_cases:
        case_rows = [row for row in rows if row["graph_case"] == graph_case]
        primary_rows = [row for row in case_rows if row["primary_statistics"]]
        initialization_rows = [row for row in case_rows if not row["primary_statistics"]]
        summary_cases[graph_case] = {
            "overall": {
                "attempt_status_counts": status_counts(primary_rows, "attempt_status"),
                "status_counts": status_counts(primary_rows, "status"),
            },
            "initialization_frame": {
                "attempt_status_counts": status_counts(initialization_rows, "attempt_status"),
                "status_counts": status_counts(initialization_rows, "status"),
            },
        }
    settings = {
        "run_seeds": [17],
        "graph_cases": graph_cases,
        "effective_frame_count": frame_count,
    }
    summary = {
        "expected_process_rows": len(rows),
        "process_rows": len(rows),
        "settings": settings,
        "graph_cases": summary_cases,
    }
    summary_path = bundle / SUMMARY_NAME
    summary_path.write_text(json.dumps(summary, sort_keys=True) + "\n", encoding="utf-8")
    summary_markdown_path = bundle / SUMMARY_MARKDOWN_NAME
    summary_markdown_path.write_text("# Completed localization diagnostic bundle\n", encoding="utf-8")
    _write_manifest(
        bundle,
        {
            "termination_reason": "completed",
            "estimator_contract": "variable_weight_nls_full_residual_jacobian_v1",
            "settings": settings,
            "compressed_process_sha256": _sha256(process_path),
            "decompressed_process_sha256": hashlib.sha256(b"".join(lines)).hexdigest(),
            "summary_json_sha256": _sha256(summary_path),
            "summary_markdown_sha256": _sha256(summary_markdown_path),
        },
    )


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

    def test_accepts_zero_valued_counts_for_all_frozen_statuses(self) -> None:
        summary = _read_summary(self.bundle)
        expected_counts = {status: 0 for status in STATUS_KEYS}
        expected_counts["converged"] = 2
        self.assertEqual(
            summary["graph_cases"]["dynamic_dag_wnls"]["initialization_frame"]
            ["attempt_status_counts"],
            expected_counts,
        )
        self.assertEqual(analyze_localization_failures(self.bundle)["integrity"]["observed_rows"], 4)

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

    def test_rejects_an_unknown_summary_status_key(self) -> None:
        summary = _read_summary(self.bundle)
        summary["graph_cases"]["dynamic_dag_wnls"]["initialization_frame"][
            "attempt_status_counts"
        ]["unknown"] = 0
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


class FailureBudgetTests(unittest.TestCase):
    def test_classifies_current_attempt_not_retained_state(self) -> None:
        """Breaks if a stale retained result hides a failed current attempt."""
        rows = [
            {"attempt_status": "converged", "containment": True},
            {"attempt_status": "converged", "containment": False},
            {
                "attempt_status": "invalid",
                "attempt_failure_reason": "invalid upstream UAV reference",
                "status": "invalid",
            },
            {
                "attempt_status": "invalid",
                "attempt_failure_reason": "non-finite or malformed WNLS input",
                "status": "invalid",
            },
            {
                "attempt_status": "failed",
                "attempt_failure_reason": "maximum WNLS iterations exceeded",
                "status": "stale",
            },
            {
                "attempt_status": "failed",
                "attempt_failure_reason": "new future failure",
                "status": "failed",
            },
        ]

        self.assertEqual(
            [_attempt_class(row) for row in rows],
            [
                "contained",
                "converged_outside_radius",
                "upstream_unavailable",
                "invalid_input_or_numeric",
                "wnls_nonconvergence",
                "other_failed",
            ],
        )

    def test_reconciles_the_primary_attempt_partition(self) -> None:
        """Breaks if a primary attempt is omitted or counted in two budget classes."""
        classes = (
            ("converged", "converged", True, None),
            ("converged", "converged", False, None),
            ("invalid", "invalid", False, "invalid upstream UAV reference"),
            ("invalid", "invalid", False, "non-finite or malformed WNLS input"),
            ("failed", "stale", False, "maximum WNLS iterations exceeded"),
            ("failed", "failed", False, "new future failure"),
        )
        rows = []
        for frame_index in range(2):
            for graph_case in GRAPH_CASES:
                for robot_id in range(1, 13):
                    if frame_index == 0:
                        rows.append(_real_schema_row(frame_index, graph_case, robot_id))
                        continue
                    attempt, status, containment, reason = classes[(robot_id - 1) // 2]
                    rows.append(
                        _real_schema_row(
                            frame_index,
                            graph_case,
                            robot_id,
                            attempt_status=attempt,
                            status=status,
                            containment=containment,
                            attempt_failure_reason=reason,
                        )
                    )

        with tempfile.TemporaryDirectory() as temporary:
            bundle = Path(temporary) / "bundle"
            _write_real_schema_bundle(bundle, rows, robot_count=12)
            report = analyze_localization_failures(bundle)

        case = report["cases"]["dynamic_dag_wnls"]
        self.assertEqual(sum(case["failure_budget"]["counts"].values()), 12)
        self.assertEqual(case["failure_budget"]["denominator"], 12)
        self.assertEqual(
            case["attempt_status_counts"],
            {"converged": 4, "invalid": 4, "failed": 4},
        )

    def test_bounds_raw_reason_labels_without_dropping_overflow_evidence(self) -> None:
        """Breaks if unbounded future failure reasons grow with process rows."""
        rows = []
        for frame_index in range(2):
            for graph_case in GRAPH_CASES:
                for robot_id in range(1, 34):
                    if frame_index == 0:
                        rows.append(_real_schema_row(frame_index, graph_case, robot_id))
                    else:
                        rows.append(
                            _real_schema_row(
                                frame_index,
                                graph_case,
                                robot_id,
                                attempt_status="failed",
                                status="failed",
                                containment=False,
                                attempt_failure_reason=f"future failure {robot_id}",
                            )
                        )

        with tempfile.TemporaryDirectory() as temporary:
            bundle = Path(temporary) / "bundle"
            _write_real_schema_bundle(bundle, rows, robot_count=33)
            report = analyze_localization_failures(bundle)

        reason_labels = report["cases"]["dynamic_dag_wnls"]["reason_labels"]
        self.assertEqual(len(reason_labels["counts"]), 32)
        self.assertEqual(reason_labels["overflow_count"], 1)
        self.assertEqual(reason_labels["overflow_examples"], ["future failure 33"])


class PredecessorLineageTests(unittest.TestCase):
    def test_counts_two_unavailable_edges_for_one_observer_at_the_maximum_parent_depth(self) -> None:
        """Breaks if one observer's two unavailable parents are collapsed or under-depth."""
        case = _empty_failure_budget(["dynamic_dag_wnls"])["dynamic_dag_wnls"]
        row = {
            "primary_statistics": True,
            "measurements": [
                {
                    "kind": "uav",
                    "id": 1,
                    "estimated_reference_available": False,
                },
                {
                    "kind": "uav",
                    "id": 2,
                    "estimated_reference_available": False,
                },
            ],
        }
        predecessors = {
            1: {
                "robot_id": 1,
                "attempt_class": "wnls_nonconvergence",
                "attempt_status": "failed",
                "retained_status": "stale",
                "attempt_failure_reason": "maximum WNLS iterations exceeded",
                "propagation_depth": 0,
            },
            2: {
                "robot_id": 2,
                "attempt_class": "upstream_unavailable",
                "attempt_status": "invalid",
                "retained_status": "invalid",
                "attempt_failure_reason": "invalid upstream UAV reference",
                "propagation_depth": 1,
            },
        }

        observer_depth = _record_lineage(
            case, row, "upstream_unavailable", predecessors
        )

        lineage = case["lineage"]
        self.assertEqual(lineage["upstream_unavailable_rows"], 1)
        self.assertEqual(lineage["unavailable_uav_reference_edges"], 2)
        self.assertEqual(
            lineage["predecessor_attempt_classes"]["wnls_nonconvergence"], 1
        )
        self.assertEqual(
            lineage["predecessor_attempt_classes"]["upstream_unavailable"], 1
        )
        self.assertEqual(observer_depth, 2)
        self.assertEqual(lineage["propagation_depth_counts"], {"not_observed": 0, "2": 1})

    def test_links_same_frame_unavailable_uav_to_the_lower_index_attempt(self) -> None:
        """Breaks if upstream lineage is inferred from retained state or another frame."""
        rows = []
        for frame_index in range(2):
            for graph_case in GRAPH_CASES:
                rows.append(
                    _real_schema_row(
                        frame_index,
                        graph_case,
                        1,
                        attempt_status="failed" if frame_index else "converged",
                        status="stale" if frame_index else "converged",
                        containment=False if frame_index else True,
                        attempt_failure_reason=(
                            "maximum WNLS iterations exceeded" if frame_index else None
                        ),
                    )
                )
                rows.append(
                    _real_schema_row(
                        frame_index,
                        graph_case,
                        2,
                        attempt_status="invalid" if frame_index else "converged",
                        status="invalid" if frame_index else "converged",
                        containment=False if frame_index else True,
                        attempt_failure_reason=(
                            "invalid upstream UAV reference" if frame_index else None
                        ),
                        measurements=(
                            [
                                {
                                    "kind": "uav",
                                    "id": 1,
                                    "estimated_reference_available": False,
                                }
                            ]
                            if frame_index
                            else []
                        ),
                    )
                )

        with tempfile.TemporaryDirectory() as temporary:
            bundle = Path(temporary) / "bundle"
            _write_real_schema_bundle(bundle, rows, robot_count=2)
            report = analyze_localization_failures(bundle)

        lineage = report["cases"]["dynamic_dag_wnls"]["lineage"]
        self.assertEqual(lineage["upstream_unavailable_rows"], 1)
        self.assertEqual(lineage["unavailable_uav_reference_edges"], 1)
        self.assertEqual(
            lineage["predecessor_attempt_classes"]["wnls_nonconvergence"], 1
        )

    def test_counts_only_observed_same_frame_chain_depth(self) -> None:
        """Breaks if a two-hop unavailable chain is reported as a direct failure."""
        rows = []
        for frame_index in range(2):
            for graph_case in GRAPH_CASES:
                rows.append(
                    _real_schema_row(
                        frame_index,
                        graph_case,
                        1,
                        attempt_status="failed" if frame_index else "converged",
                        status="stale" if frame_index else "converged",
                        containment=False if frame_index else True,
                        attempt_failure_reason=(
                            "maximum WNLS iterations exceeded" if frame_index else None
                        ),
                    )
                )
                for robot_id, parent_id in ((2, 1), (3, 2)):
                    rows.append(
                        _real_schema_row(
                            frame_index,
                            graph_case,
                            robot_id,
                            attempt_status="invalid" if frame_index else "converged",
                            status="invalid" if frame_index else "converged",
                            containment=False if frame_index else True,
                            attempt_failure_reason=(
                                "invalid upstream UAV reference" if frame_index else None
                            ),
                            measurements=(
                                [
                                    {
                                        "kind": "uav",
                                        "id": parent_id,
                                        "estimated_reference_available": False,
                                    }
                                ]
                                if frame_index
                                else []
                            ),
                        )
                    )

        with tempfile.TemporaryDirectory() as temporary:
            bundle = Path(temporary) / "bundle"
            _write_real_schema_bundle(bundle, rows, robot_count=3)
            report = analyze_localization_failures(bundle)

        self.assertEqual(
            report["cases"]["dynamic_dag_wnls"]["lineage"]["propagation_depth_counts"],
            {"not_observed": 0, "1": 1, "2": 1},
        )

    def test_marks_a_referenced_predecessor_absent_from_the_same_frame_as_not_observed(self) -> None:
        """Breaks if a nonexistent predecessor is assigned an inferred failure cause."""
        rows = []
        for frame_index in range(2):
            for graph_case in GRAPH_CASES:
                rows.append(_real_schema_row(frame_index, graph_case, 1))
                rows.append(
                    _real_schema_row(
                        frame_index,
                        graph_case,
                        2,
                        attempt_status="invalid" if frame_index else "converged",
                        status="invalid" if frame_index else "converged",
                        containment=False if frame_index else True,
                        attempt_failure_reason=(
                            "invalid upstream UAV reference" if frame_index else None
                        ),
                        measurements=(
                            [
                                {
                                    "kind": "uav",
                                    "id": 0,
                                    "estimated_reference_available": False,
                                }
                            ]
                            if frame_index
                            else []
                        ),
                    )
                )

        with tempfile.TemporaryDirectory() as temporary:
            bundle = Path(temporary) / "bundle"
            _write_real_schema_bundle(bundle, rows, robot_count=2)
            report = analyze_localization_failures(bundle)

        lineage = report["cases"]["dynamic_dag_wnls"]["lineage"]
        self.assertEqual(lineage["predecessor_attempt_classes"]["not_observed"], 1)


class ConditionalCalibrationTests(unittest.TestCase):
    def test_hand_derived_nees_and_fixed_bins(self) -> None:
        """Breaks if covariance inversion or half-open endpoint assignment changes."""
        self.assertEqual(
            _normalized_squared_error([2.0, 1.0], [[4.0, 0.0], [0.0, 1.0]]),
            2.0,
        )


class PairedBootstrapTests(unittest.TestCase):
    def test_recomputes_paired_aggregate_counts_and_marks_non_estimable_resamples(self) -> None:
        """Breaks if bootstrap averages seed ratios instead of resampling summed paired counts."""
        seed_counts = [
            {"dyn_up": 8, "fix_up": 2, "dyn_invalid": 10, "fix_invalid": 4},
            {"dyn_up": 1, "fix_up": 3, "dyn_invalid": 2, "fix_invalid": 5},
        ]
        report = _paired_seed_bootstrap(seed_counts, resamples=20, rng_seed=20260729)
        self.assertEqual(report["point_estimate"], 4 / 3)
        self.assertEqual(report["seed_specific"], [1.0, None])
        self.assertGreater(report["non_estimable_resamples"], 0)
        self.assertIsNone(report["percentile_interval"])
        self.assertEqual(
            [_ratio_bin(value) for value in (0.0, 0.5, 1.0, 2.0, 5.0)],
            ["[0,0.5)", "[0.5,1)", "[1,2)", "[2,5)", "[5,infinity)"],
        )
        self.assertEqual(
            [_q_bin(value) for value in (0.0, 2.295748929, 5.991464547, 9.0, 9.000000001)],
            ["[0,2.295748929)", "[2.295748929,5.991464547)", "[5.991464547,9]", "[5.991464547,9]", "(9,infinity)"],
        )
