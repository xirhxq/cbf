"""Acceptance tests for initialization-persistence analysis."""

from __future__ import annotations

import gzip
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import scripts.diagnostics.analyze_initialization_persistence as analyzer
from scripts.diagnostics.analyze_initialization_persistence import (
    EXACT_REASON,
    InputIntegrityError,
    analyze_initialization_persistence,
    classify_exact_reason,
)
from scripts.diagnostics.run_diagnostic import allocated_bytes


PROCESS_NAME = "calibration.jsonl.gz"
SUMMARY_NAME = "summary.json"
SUMMARY_MARKDOWN_NAME = "summary.md"
GRAPH_CASES = ("dynamic_dag_wnls", "fixed_refs_wnls")
STATUS_KEYS = ("converged", "stale", "invalid", "failed")


def malformed_row(*ranges: float | None) -> dict:
    return {
        "attempt_status": "invalid",
        "attempt_failure_reason": EXACT_REASON,
        "measurements": [{"noisy_range": value} for value in ranges],
    }


class ExactReasonClassificationTests(unittest.TestCase):
    def test_missing_prior_with_finite_measurements_is_exclusive_prior(self) -> None:
        """Breaks if missing priors are not isolated from finite measurements."""
        previous = {"estimate": None}
        self.assertEqual(
            classify_exact_reason(malformed_row(1.0, 2.0), previous),
            "exclusive_prior_self_estimate_missing_or_malformed",
        )

    def test_missing_prior_and_bad_measurement_is_compound(self) -> None:
        """Breaks if the compound event is incorrectly counted as exclusive prior."""
        previous = {"estimate": None}
        self.assertEqual(
            classify_exact_reason(malformed_row(1.0, None), previous),
            "prior_missing_and_recorded_measurement_invalid",
        )

    def test_finite_prior_and_bad_measurement_is_measurement_category(self) -> None:
        """Breaks if invalid recorded measurements are not distinguished from priors."""
        previous = {"estimate": [3.0, 4.0]}
        self.assertEqual(
            classify_exact_reason(malformed_row(None), previous),
            "recorded_measurement_missing_or_nonfinite",
        )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(8192), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _real_schema_row(
    frame_index: int,
    graph_case: str,
    *,
    attempt_status: str = "converged",
    status: str = "converged",
    attempt_failure_reason: str | None = None,
    measurements: list[dict] | None = None,
    estimate: list[float] | None = None,
) -> dict:
    return {
        "frame_index": frame_index,
        "seed": 17,
        "graph_case": graph_case,
        "robot_id": 1,
        "squad_local_index": 1,
        "primary_statistics": frame_index != 0,
        "attempt_status": attempt_status,
        "status": status,
        "containment": attempt_status == "converged",
        "attempt_failure_reason": attempt_failure_reason,
        "measurements": [] if measurements is None else measurements,
        "estimate": estimate,
    }


def _write_manifest(bundle: Path, manifest: dict) -> None:
    (bundle / "manifest.json").write_text(
        json.dumps(manifest, sort_keys=True) + "\n", encoding="utf-8"
    )


class InitializationPersistenceAnalyzerTests(unittest.TestCase):
    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self._temporary_directory.name)

    def tearDown(self) -> None:
        self._temporary_directory.cleanup()

    def test_499_frame_persistence_reconciles_by_seed_case_and_depth(self) -> None:
        """Breaks if prior state is not persisted across every primary frame."""
        rows = self.persistence_rows(primary_frames=499)
        report = self.analyze_fixture(rows)
        dynamic = report["cases"]["dynamic_dag_wnls"]
        self.assertEqual(dynamic["exact_reason_rows"], 499)
        self.assertEqual(
            dynamic["categories"][
                "exclusive_prior_self_estimate_missing_or_malformed"
            ],
            499,
        )

    def test_gate_requires_95_percent_exclusive_prior_in_each_case(self) -> None:
        """Breaks if a 94 percent exclusive-prior case opens the Gate."""
        report = self.analyze_fixture(self.rows_with_dominance(0.94))
        self.assertFalse(report["gate_passed"])

    def test_gate_rejects_any_compound_prior_measurement_row(self) -> None:
        """Breaks if one compound event is permitted through the Gate."""
        report = self.analyze_fixture(self.rows_with_one_compound_event())
        self.assertEqual(
            report["cases"]["dynamic_dag_wnls"]["categories"][
                "prior_missing_and_recorded_measurement_invalid"
            ],
            1,
        )
        self.assertFalse(report["gate_passed"])

    def test_exact_reason_plus_other_invalid_reasons_reconciles_broad_class(self) -> None:
        """Breaks if exact and non-exact invalid reasons lose the invalid partition."""
        bundle = self.write_fixture(self.rows_with_two_invalid_reasons())
        report = analyze_initialization_persistence(
            bundle,
            expected_broad_invalid_totals={
                "dynamic_dag_wnls": 2,
                "fixed_refs_wnls": 2,
            },
        )
        integrity = report["integrity"]
        self.assertTrue(integrity["invalid_reason_reconciliation"])

    def test_completed_audit_reconciliation_is_per_case_and_closes_gate(self) -> None:
        """Breaks if a case can pass without matching its completed-audit broad total."""
        bundle = self.write_fixture(self.rows_with_two_invalid_reasons())
        report = analyze_initialization_persistence(
            bundle,
            expected_broad_invalid_totals={
                "dynamic_dag_wnls": 2,
                "fixed_refs_wnls": 1,
            },
        )
        self.assertTrue(
            report["cases"]["dynamic_dag_wnls"]["invalid_reason_reconciliation"]
        )
        self.assertFalse(
            report["cases"]["fixed_refs_wnls"]["invalid_reason_reconciliation"]
        )
        self.assertFalse(report["integrity"]["invalid_reason_reconciliation"])
        self.assertFalse(report["gate_passed"])

    def test_rejects_any_graph_case_set_other_than_the_two_frozen_cases(self) -> None:
        """Breaks if a self-consistent one-case bundle can satisfy a two-case Gate."""
        rows = [
            row
            for row in self.persistence_rows(primary_frames=2)
            if row["graph_case"] == "dynamic_dag_wnls"
        ]
        bundle = self.write_fixture(rows, graph_cases=("dynamic_dag_wnls",))
        with self.assertRaises(InputIntegrityError):
            analyze_initialization_persistence(bundle)

    def test_source_hash_mutation_and_out_of_order_keys_fail_closed(self) -> None:
        """Breaks if a rehashed but noncanonical stream is accepted."""
        bundle = self.write_fixture(self.persistence_rows(primary_frames=2))
        self.rewrite_first_process_key(bundle, frame_index=1)
        with self.assertRaises(InputIntegrityError):
            analyze_initialization_persistence(bundle)

    def test_external_output_is_atomic_and_below_10_mb(self) -> None:
        """Breaks if publication leaks siblings or exceeds its bounded output budget."""
        bundle = self.write_fixture(self.persistence_rows(primary_frames=2))
        output = self.root / "analysis" / "run"
        analyze_initialization_persistence(bundle, output_dir=output)
        self.assertEqual(
            {path.name for path in output.iterdir()},
            {
                "initialization-persistence.json",
                "initialization-persistence.md",
            },
        )
        self.assertLess(allocated_bytes(output), 10_000_000)

    def test_publication_does_not_overwrite_directory_created_by_racer(self) -> None:
        """Breaks if final publication can replace a concurrently created directory."""
        bundle = self.write_fixture(self.persistence_rows(primary_frames=2))
        output = self.root / "analysis" / "run"
        output.parent.mkdir()
        verify_inputs = analyzer._verify_unchanged_inputs
        verification_count = 0

        def create_racing_directory(*args: object) -> None:
            nonlocal verification_count
            verify_inputs(*args)
            verification_count += 1
            if verification_count == 2:
                output.mkdir()

        with patch.object(
            analyzer,
            "_verify_unchanged_inputs",
            side_effect=create_racing_directory,
        ):
            with self.assertRaises(FileExistsError):
                analyze_initialization_persistence(bundle, output_dir=output)
        self.assertTrue(output.is_dir())
        self.assertEqual(list(output.iterdir()), [])
        self.assertEqual(list(output.with_name("run.incomplete").iterdir()), [])

    def test_module_cli_executes_analysis_with_frozen_trust_root(self) -> None:
        """Breaks if python -m merely imports definitions without running Gate 1."""
        bundle = self.write_fixture(self.persistence_rows(primary_frames=2))
        completed = self.run_cli(bundle, expected_hashes=self.source_hashes(bundle))
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertTrue(completed.stdout.strip(), "CLI emitted no report")
        self.assertTrue(json.loads(completed.stdout)["gate_passed"])

    def test_cli_rejects_self_consistent_bundle_replaced_before_start(self) -> None:
        """Breaks if a replaced manifest and artifact can self-certify at startup."""
        bundle = self.write_fixture(self.persistence_rows(primary_frames=2))
        expected_hashes = self.source_hashes(bundle)
        summary_path = bundle / SUMMARY_NAME
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        summary["substituted_before_start"] = True
        summary_path.write_text(json.dumps(summary, sort_keys=True) + "\n", encoding="utf-8")
        manifest = json.loads((bundle / "manifest.json").read_text(encoding="utf-8"))
        manifest["summary_json_sha256"] = _sha256(summary_path)
        _write_manifest(bundle, manifest)

        completed = self.run_cli(bundle, expected_hashes=expected_hashes)

        self.assertNotEqual(completed.returncode, 0)
        self.assertFalse(completed.stdout.strip())

    def test_output_rejects_source_mutation_during_publication(self) -> None:
        """Breaks if a completed report survives a source change while it is rendered."""
        bundle = self.write_fixture(self.persistence_rows(primary_frames=2))
        output = self.root / "analysis" / "run"
        output.parent.mkdir()
        render_markdown = analyzer._render_markdown

        def mutate_source(report: dict) -> bytes:
            (bundle / SUMMARY_MARKDOWN_NAME).write_text("mutated\n", encoding="utf-8")
            return render_markdown(report)

        with patch.object(analyzer, "_render_markdown", side_effect=mutate_source):
            with self.assertRaises(InputIntegrityError):
                analyze_initialization_persistence(bundle, output_dir=output)
        self.assertFalse(output.exists())
        self.assertEqual(
            list(output.with_name("run.incomplete").iterdir()),
            [],
        )

    def analyze_fixture(self, rows: list[dict], robot_count: int = 1) -> dict:
        """Write a canonical, rehashed real-schema bundle and run the public API."""
        return analyze_initialization_persistence(
            self.write_fixture(rows, robot_count=robot_count)
        )

    def write_fixture(
        self,
        rows: list[dict],
        robot_count: int = 1,
        graph_cases: tuple[str, ...] = GRAPH_CASES,
    ) -> Path:
        if {row["robot_id"] for row in rows} != set(range(1, robot_count + 1)):
            raise ValueError("fixture rows must cover exactly the declared robot IDs")
        bundle = self.root / f"bundle-{len(list(self.root.glob('bundle-*')))}"
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
            "graph_cases": list(graph_cases),
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
        return bundle

    def source_hashes(self, bundle: Path) -> dict[str, str]:
        return {
            "manifest": _sha256(bundle / "manifest.json"),
            "summary": _sha256(bundle / SUMMARY_NAME),
            "process": _sha256(bundle / PROCESS_NAME),
        }

    def run_cli(
        self,
        bundle: Path,
        *,
        expected_hashes: dict[str, str],
    ) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                sys.executable,
                "-m",
                "scripts.diagnostics.analyze_initialization_persistence",
                "--bundle-dir",
                str(bundle),
                "--expected-manifest-sha256",
                expected_hashes["manifest"],
                "--expected-summary-json-sha256",
                expected_hashes["summary"],
                "--expected-compressed-process-sha256",
                expected_hashes["process"],
                "--expected-dynamic-broad-invalid-total",
                "2",
                "--expected-fixed-broad-invalid-total",
                "2",
            ],
            cwd=Path(__file__).resolve().parents[1],
            capture_output=True,
            text=True,
            check=False,
        )

    def rewrite_first_process_key(self, bundle: Path, *, frame_index: int) -> None:
        process_path = bundle / PROCESS_NAME
        with gzip.open(process_path, "rb") as stream:
            lines = list(stream)
        first_row = json.loads(lines[0])
        first_row["frame_index"] = frame_index
        lines[0] = json.dumps(first_row, sort_keys=True, separators=(",", ":")).encode() + b"\n"
        with process_path.open("wb") as raw:
            with gzip.GzipFile(fileobj=raw, mode="wb", mtime=0) as compressed:
                for line in lines:
                    compressed.write(line)
        manifest = json.loads((bundle / "manifest.json").read_text(encoding="utf-8"))
        manifest["compressed_process_sha256"] = _sha256(process_path)
        manifest["decompressed_process_sha256"] = hashlib.sha256(b"".join(lines)).hexdigest()
        _write_manifest(bundle, manifest)

    def persistence_rows(self, *, primary_frames: int) -> list[dict]:
        rows = []
        for frame_index in range(primary_frames + 1):
            for graph_case in GRAPH_CASES:
                if frame_index == 0:
                    rows.append(_real_schema_row(frame_index, graph_case, estimate=None))
                else:
                    rows.append(
                        _real_schema_row(
                            frame_index,
                            graph_case,
                            attempt_status="invalid",
                            status="invalid",
                            attempt_failure_reason=EXACT_REASON,
                            measurements=[{"noisy_range": 1.0}],
                            estimate=None,
                        )
                    )
        return rows

    def rows_with_dominance(self, dominance: float) -> list[dict]:
        total = 100
        exclusive = round(total * dominance)
        rows = self.persistence_rows(primary_frames=total)
        for row in rows:
            if (
                row["graph_case"] == "dynamic_dag_wnls"
                and row["frame_index"] > exclusive
            ):
                row["measurements"] = [{"noisy_range": None}]
        return rows

    def rows_with_one_compound_event(self) -> list[dict]:
        rows = self.persistence_rows(primary_frames=2)
        for row in rows:
            if row["graph_case"] == "dynamic_dag_wnls" and row["frame_index"] == 2:
                row["measurements"] = [{"noisy_range": None}]
        return rows

    def rows_with_two_invalid_reasons(self) -> list[dict]:
        rows = self.persistence_rows(primary_frames=2)
        for row in rows:
            if row["frame_index"] == 2:
                row["attempt_failure_reason"] = "another invalid reason"
        return rows
