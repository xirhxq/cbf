"""Acceptance tests for strict-anchored warm-start recovery comparisons."""

from __future__ import annotations

import gzip
import hashlib
import importlib
import json
import tempfile
import unittest
from collections.abc import Callable
from pathlib import Path
from unittest.mock import patch

from scripts.diagnostics.replay_localization_calibration import (
    RESTART_BEFORE_FIRST_FINITE_POLICY,
    STRICT_PREVIOUS_POLICY,
)


PROCESS_NAME = "calibration.jsonl.gz"
SUMMARY_NAME = "summary.json"
SUMMARY_MARKDOWN_NAME = "summary.md"
GRAPH_CASES = ("dynamic_dag_wnls", "fixed_refs_wnls")
STATUS_KEYS = ("converged", "stale", "invalid", "failed")
PROSPECTIVE_FIELDS = {
    "initialization_policy",
    "initial_estimate_source",
    "ever_acquired_finite_before_attempt",
}


def normalized(row: dict) -> dict:
    return {
        key: value
        for key, value in row.items()
        if key not in PROSPECTIVE_FIELDS
    }


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_comparator():
    try:
        return importlib.import_module(
            "scripts.diagnostics.compare_warm_start_recovery"
        )
    except ModuleNotFoundError:
        return None


class ComparisonFixture(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.fixture_count = 0

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _rows(
        self,
        policy: str | None,
        *,
        seeds: tuple[int, ...] = (17,),
    ) -> list[dict]:
        rows = []
        for frame_index in range(2):
            for seed in seeds:
                for graph_case in GRAPH_CASES:
                    for robot_id in (1, 2):
                        row = {
                            "frame_index": frame_index,
                            "seed": seed,
                            "graph_case": graph_case,
                            "robot_id": robot_id,
                            "squad_local_index": robot_id,
                            "primary_statistics": frame_index != 0,
                            "active_references": [0, 1],
                            "measurements": [],
                            "truth_position": [1.0, 1.0],
                            "status": "converged",
                            "estimate": [1.0, 1.0],
                            "attempt_status": "converged",
                            "attempt_failure_reason": None,
                            "covariance": [[1.0, 0.0], [0.0, 1.0]],
                            "epsilon": 3.0,
                            "finite": True,
                            "covariance_spd": True,
                            "error_vector": [1.0, 0.0],
                            "error_norm": 1.0,
                            "error_to_epsilon_ratio": 1.0 / 3.0,
                            "state_containment": True,
                            "containment": True,
                            "phi_min_eigenvalue": 1.0,
                            "phi_condition": 1.0,
                            "iterations": 1,
                            "cost": 0.0,
                            "stationarity_norm": 0.0,
                            "failure_reason": None,
                            "failure": None,
                            "transition": {
                                "changed": False,
                                "epsilon_change": None,
                            },
                        }
                        if policy is not None:
                            row.update(
                                {
                                    "initialization_policy": policy,
                                    "initial_estimate_source": (
                                        "deployment_frame_zero"
                                        if frame_index == 0
                                        else "previous_finite"
                                    ),
                                    "ever_acquired_finite_before_attempt": (
                                        frame_index != 0
                                    ),
                                }
                            )
                        rows.append(row)
        return rows

    def _write_bundle(
        self,
        bundle: Path,
        rows: list[dict],
        policy: str | None,
        *,
        source_paths: dict[str, Path],
    ) -> dict:
        bundle.mkdir()
        lines = [
            json.dumps(
                row,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            ).encode("utf-8")
            + b"\n"
            for row in rows
        ]
        process_path = bundle / PROCESS_NAME
        with process_path.open("wb") as raw:
            with gzip.GzipFile(fileobj=raw, mode="wb", mtime=0) as compressed:
                for line in lines:
                    compressed.write(line)

        def counts(selected: list[dict], field: str) -> dict[str, int]:
            result = {status: 0 for status in STATUS_KEYS}
            for row in selected:
                result[row[field]] += 1
            return result

        settings = {
            "estimator_contract": (
                "variable_weight_nls_full_residual_jacobian_v1"
            ),
            "run_seeds": sorted({row["seed"] for row in rows}),
            "graph_cases": list(GRAPH_CASES),
            "effective_frame_count": 2,
            "max_frames": 2,
            "implementation": {
                "path": str(source_paths["replay"]),
                "sha256": _sha256(source_paths["replay"]),
            },
        }
        if policy is not None:
            settings["initialization_policy"] = policy
        cases = {}
        for graph_case in GRAPH_CASES:
            case_rows = [
                row for row in rows if row["graph_case"] == graph_case
            ]
            primary = [row for row in case_rows if row["primary_statistics"]]
            initialization = [
                row for row in case_rows if not row["primary_statistics"]
            ]
            cases[graph_case] = {
                "overall": {
                    "attempt_status_counts": counts(primary, "attempt_status"),
                    "status_counts": counts(primary, "status"),
                },
                "initialization_frame": {
                    "attempt_status_counts": counts(
                        initialization, "attempt_status"
                    ),
                    "status_counts": counts(initialization, "status"),
                },
            }
        summary = {
            "estimator_contract": (
                "variable_weight_nls_full_residual_jacobian_v1"
            ),
            "settings": settings,
            "graph_cases": cases,
            "expected_process_rows": len(rows),
            "process_rows": len(rows),
        }
        summary_path = bundle / SUMMARY_NAME
        summary_path.write_text(
            json.dumps(summary, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        markdown_path = bundle / SUMMARY_MARKDOWN_NAME
        markdown_path.write_text("# Fixture\n", encoding="utf-8")
        manifest = {
            "termination_reason": "completed",
            "estimator_contract": (
                "variable_weight_nls_full_residual_jacobian_v1"
            ),
            "initialization_policy": policy,
            "output_dir": str(bundle),
            "input_data": {
                "path": str(source_paths["data"]),
                "sha256": _sha256(source_paths["data"]),
            },
            "input_manifest": {
                "path": str(source_paths["input_manifest"]),
                "sha256": _sha256(source_paths["input_manifest"]),
            },
            "settings": settings,
            "compressed_process_sha256": _sha256(process_path),
            "decompressed_process_sha256": hashlib.sha256(
                b"".join(lines)
            ).hexdigest(),
            "summary_json_sha256": _sha256(summary_path),
            "summary_markdown_sha256": _sha256(markdown_path),
        }
        (bundle / "manifest.json").write_text(
            json.dumps(manifest, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        return manifest

    def compare_fixture(
        self,
        strict_mutation: Callable[[dict], None] | None = None,
        key_mutation: str | None = None,
        *,
        seeds: tuple[int, ...] = (17,),
        outcome_setup: (
            Callable[[list[dict], list[dict], list[dict]], None] | None
        ) = None,
        restart_provenance_mutation: Callable[[list[dict]], None] | None = None,
        output: bool = False,
    ) -> dict:
        module = _load_comparator()
        if module is None:
            self.fail("warm-start recovery comparator is not implemented")

        fixture_root = self.root / f"fixture-{self.fixture_count}"
        self.fixture_count += 1
        fixture_root.mkdir()
        data_path = fixture_root / "data.json"
        input_manifest_path = fixture_root / "input-manifest.json"
        replay_path = fixture_root / "replay.py"
        snapshot_path = fixture_root / "source-snapshot.tar.gz"
        data_path.write_text('{"trajectory":"fixture"}\n', encoding="utf-8")
        input_manifest_path.write_text(
            '{"termination_reason":"completed"}\n', encoding="utf-8"
        )
        replay_path.write_text("IMPLEMENTATION = 1\n", encoding="utf-8")
        snapshot_path.write_bytes(b"fixture snapshot")
        sources = {
            "data": data_path,
            "input_manifest": input_manifest_path,
            "replay": replay_path,
        }

        baseline_rows = self._rows(None, seeds=seeds)
        strict_rows = self._rows(STRICT_PREVIOUS_POLICY, seeds=seeds)
        restart_rows = self._rows(
            RESTART_BEFORE_FIRST_FINITE_POLICY, seeds=seeds
        )
        if outcome_setup is not None:
            outcome_setup(baseline_rows, strict_rows, restart_rows)
        if restart_provenance_mutation is not None:
            restart_provenance_mutation(restart_rows)
        if strict_mutation is not None:
            strict_mutation(strict_rows[0])
        if key_mutation == "missing":
            strict_rows.pop()
        elif key_mutation == "duplicate":
            strict_rows[1] = dict(strict_rows[0])
        elif key_mutation == "reordered":
            strict_rows[0], strict_rows[1] = strict_rows[1], strict_rows[0]

        baseline_dir = fixture_root / "baseline"
        strict_dir = fixture_root / "strict"
        restart_dir = fixture_root / "restart"
        baseline_manifest = self._write_bundle(
            baseline_dir, baseline_rows, None, source_paths=sources
        )
        strict_manifest = self._write_bundle(
            strict_dir,
            strict_rows,
            STRICT_PREVIOUS_POLICY,
            source_paths=sources,
        )
        restart_manifest = self._write_bundle(
            restart_dir,
            restart_rows,
            RESTART_BEFORE_FIRST_FINITE_POLICY,
            source_paths=sources,
        )

        parent_dir = fixture_root / "paired"
        parent_dir.mkdir()
        parent = {
            "schema": "cbf2026-warm-start-recovery-parent-v1",
            "termination_reason": "completed",
            "estimator_contract": (
                "variable_weight_nls_full_residual_jacobian_v1"
            ),
            "source_commit": "fixture-commit",
            "source_branch": "codex/fixture",
            "tracked_worktree_status": "",
            "source_snapshot_path": str(snapshot_path),
            "source_snapshot_sha256": _sha256(snapshot_path),
            "replay_implementation": {
                "path": str(replay_path),
                "sha256": _sha256(replay_path),
            },
            "immutable_baseline_dir": str(baseline_dir),
            "immutable_baseline_hashes": {
                name: _sha256(baseline_dir / name)
                for name in (
                    "manifest.json",
                    SUMMARY_NAME,
                    SUMMARY_MARKDOWN_NAME,
                    PROCESS_NAME,
                )
            },
            "input_data": {
                "path": str(data_path),
                "sha256": _sha256(data_path),
            },
            "input_manifest": {
                "path": str(input_manifest_path),
                "sha256": _sha256(input_manifest_path),
            },
            "seeds": list(seeds),
            "max_frames": 2,
            "policies": [
                STRICT_PREVIOUS_POLICY,
                RESTART_BEFORE_FIRST_FINITE_POLICY,
            ],
            "children": {
                "strict": strict_manifest,
                "restart": restart_manifest,
            },
        }
        (parent_dir / "manifest.json").write_text(
            json.dumps(parent, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        comparison_output = fixture_root / "comparison" if output else None
        self.last_output_dir = comparison_output
        try:
            return module.compare_warm_start_recovery(
                parent_dir,
                baseline_dir,
                output_dir=comparison_output,
            )
        except NotImplementedError as error:
            self.fail(str(error))

    @staticmethod
    def _set_invalid(row: dict, reason: str) -> None:
        row.update(
            {
                "status": "invalid",
                "estimate": None,
                "attempt_status": "invalid",
                "attempt_failure_reason": reason,
                "covariance": None,
                "epsilon": None,
                "finite": False,
                "covariance_spd": False,
                "error_vector": None,
                "error_norm": None,
                "error_to_epsilon_ratio": None,
                "state_containment": None,
                "containment": False,
                "phi_min_eigenvalue": None,
                "phi_condition": None,
                "failure_reason": reason,
                "failure": {"status": "invalid", "failure_reason": reason},
            }
        )

    @staticmethod
    def _primary(
        rows: list[dict],
        graph_case: str,
        *,
        seed: int | None = None,
    ) -> list[dict]:
        return [
            row
            for row in rows
            if row["primary_statistics"]
            and row["graph_case"] == graph_case
            and (seed is None or row["seed"] == seed)
        ]


class StrictAnchorTests(ComparisonFixture):
    def test_strict_rows_equal_immutable_rows_after_normalization(self):
        report = self.compare_fixture()
        self.assertTrue(report["strict_anchor"]["normalized_rows_equal"])
        self.assertEqual(report["strict_anchor"]["rows_compared"], 8)

    def test_any_scientific_field_drift_stops_before_policy_statistics(self):
        def mutate(row):
            row["epsilon"] = 999.0

        module = _load_comparator()
        if module is None:
            self.fail("warm-start recovery comparator is not implemented")
        with self.assertRaises(module.InputIntegrityError):
            self.compare_fixture(strict_mutation=mutate)

    def test_missing_duplicate_or_reordered_key_stops(self):
        module = _load_comparator()
        if module is None:
            self.fail("warm-start recovery comparator is not implemented")
        for mutation in ("missing", "duplicate", "reordered"):
            with self.subTest(mutation=mutation):
                with self.assertRaises(module.InputIntegrityError):
                    self.compare_fixture(key_mutation=mutation)


class PairedOutcomeTests(ComparisonFixture):
    EXACT_REASON = "non-finite or malformed WNLS input"

    def test_primary_denominator_includes_every_row_and_pairs_seed_fractions(self):
        """Breaks if rows replace seeds as units or failures shrink denominators."""

        def setup(baseline, strict, restart):
            for source in (baseline, strict):
                dynamic_17 = self._primary(
                    source, "dynamic_dag_wnls", seed=17
                )
                dynamic_18 = self._primary(
                    source, "dynamic_dag_wnls", seed=18
                )
                self._set_invalid(dynamic_17[0], self.EXACT_REASON)
                for row in dynamic_18:
                    self._set_invalid(row, self.EXACT_REASON)
            restart_18 = self._primary(
                restart, "dynamic_dag_wnls", seed=18
            )
            self._set_invalid(restart_18[0], self.EXACT_REASON)

        report = self.compare_fixture(
            seeds=(17, 18),
            outcome_setup=setup,
        )
        self.assertIn("comparisons", report)
        outcome = report["comparisons"]["dynamic_dag_wnls"][
            "exact_direct"
        ]

        self.assertEqual(outcome["aggregate"]["strict"]["denominator"], 4)
        self.assertEqual(outcome["aggregate"]["strict"]["count"], 3)
        self.assertEqual(outcome["aggregate"]["restart"]["denominator"], 4)
        self.assertEqual(outcome["aggregate"]["restart"]["count"], 1)
        self.assertEqual(len(outcome["seed_records"]), 2)
        self.assertEqual(
            [record["strict_fraction"] for record in outcome["seed_records"]],
            [0.5, 1.0],
        )
        self.assertEqual(
            [record["paired_difference"] for record in outcome["seed_records"]],
            [-0.5, -0.5],
        )

    def test_dynamic_gate_requires_interval_and_ninety_percent_reduction(self):
        """Breaks if either preregistered primary criterion is optional."""

        def passing(baseline, strict, restart):
            for source in (baseline, strict):
                for row in self._primary(source, "dynamic_dag_wnls"):
                    self._set_invalid(row, self.EXACT_REASON)

        passed = self.compare_fixture(outcome_setup=passing)
        self.assertIn("gates", passed)
        self.assertTrue(passed["gates"]["dynamic_exact_direct"]["passed"])

        def insufficient_reduction(baseline, strict, restart):
            passing(baseline, strict, restart)
            self._set_invalid(
                self._primary(restart, "dynamic_dag_wnls")[0],
                self.EXACT_REASON,
            )

        count_failed = self.compare_fixture(
            outcome_setup=insufficient_reduction
        )
        count_gate = count_failed["gates"]["dynamic_exact_direct"]
        self.assertTrue(count_gate["bootstrap_upper_below_zero"])
        self.assertFalse(count_gate["aggregate_reduction_at_least_0_90"])
        self.assertFalse(count_gate["passed"])

        sparse_seeds = tuple(range(1, 21))

        def interval_touches_zero(baseline, strict, restart):
            for source in (baseline, strict):
                for row in self._primary(
                    source, "dynamic_dag_wnls", seed=1
                ):
                    self._set_invalid(row, self.EXACT_REASON)

        interval_failed = self.compare_fixture(
            seeds=sparse_seeds,
            outcome_setup=interval_touches_zero,
        )
        interval_gate = interval_failed["gates"]["dynamic_exact_direct"]
        self.assertFalse(interval_gate["bootstrap_upper_below_zero"])
        self.assertTrue(interval_gate["aggregate_reduction_at_least_0_90"])
        self.assertFalse(interval_gate["passed"])

    def test_failed_primary_omits_upstream_inference_and_fixed_is_descriptive(self):
        """Breaks if hierarchy or the fixed comparator becomes confirmatory."""

        def setup(baseline, strict, restart):
            for source in (baseline, strict):
                self._set_invalid(
                    self._primary(source, "dynamic_dag_wnls")[0],
                    self.EXACT_REASON,
                )
                self._set_invalid(
                    self._primary(source, "fixed_refs_wnls")[0],
                    self.EXACT_REASON,
                )
            self._set_invalid(
                self._primary(restart, "dynamic_dag_wnls")[0],
                self.EXACT_REASON,
            )

        report = self.compare_fixture(outcome_setup=setup)

        self.assertIn("gates", report)
        self.assertIn("comparisons", report)
        self.assertFalse(report["gates"]["dynamic_exact_direct"]["passed"])
        self.assertIsNone(
            report["comparisons"]["dynamic_dag_wnls"][
                "upstream_unavailable_inference"
            ]
        )
        self.assertEqual(
            report["comparisons"]["fixed_refs_wnls"]["role"],
            "descriptive_only",
        )
        self.assertNotIn(
            "passed", report["comparisons"]["fixed_refs_wnls"]["exact_direct"]
        )


class SafeguardAndProvenanceTests(ComparisonFixture):
    EXACT_REASON = "non-finite or malformed WNLS input"

    def test_calibration_safeguards_use_only_aggregate_converged_attempts(self):
        """Breaks if invalid rows enter q/containment denominators or rows infer."""

        def setup(_baseline, _strict, restart):
            dynamic = self._primary(restart, "dynamic_dag_wnls")
            dynamic[0].update(
                {
                    "containment": False,
                    "state_containment": False,
                    "error_vector": [4.0, 0.0],
                    "error_norm": 4.0,
                    "error_to_epsilon_ratio": 4.0 / 3.0,
                }
            )
            self._set_invalid(dynamic[1], self.EXACT_REASON)
            dynamic[1]["error_vector"] = [100.0, 0.0]
            dynamic[1]["covariance"] = [[1.0, 0.0], [0.0, 1.0]]
            dynamic[1]["containment"] = False

        report = self.compare_fixture(outcome_setup=setup)
        self.assertIn("calibration_safeguards", report)
        dynamic = report["calibration_safeguards"]["dynamic_dag_wnls"]

        self.assertEqual(dynamic["strict"]["converged_attempts"], 2)
        self.assertEqual(dynamic["strict"]["q_above_9_count"], 0)
        self.assertEqual(dynamic["restart"]["converged_attempts"], 1)
        self.assertEqual(dynamic["restart"]["q_finite_count"], 1)
        self.assertEqual(dynamic["restart"]["q_above_9_count"], 1)
        self.assertEqual(dynamic["restart"]["q_above_9_rate"], 1.0)
        self.assertEqual(
            dynamic["restart"]["epsilon_containment_rate"], 0.0
        )
        self.assertFalse(dynamic["advance_to_multi_trajectory"])

    def test_restart_provenance_reconciles_attempts_outcomes_and_sources(self):
        """Breaks if restart counts are inferred without row-level provenance."""

        def lose_frame_zero(baseline, strict, restart):
            for source in (baseline, strict, restart):
                row = next(
                    row
                    for row in source
                    if row["frame_index"] == 0
                    and row["graph_case"] == "dynamic_dag_wnls"
                    and row["robot_id"] == 1
                )
                self._set_invalid(row, "synthetic frame-zero loss")
            strict_row = self._primary(strict, "dynamic_dag_wnls")[0]
            strict_row["initial_estimate_source"] = "strict_previous_missing"
            strict_row["ever_acquired_finite_before_attempt"] = False

        def restart_once(rows):
            row = self._primary(rows, "dynamic_dag_wnls")[0]
            row["initial_estimate_source"] = (
                "deployment_restart_before_first_finite"
            )
            row["ever_acquired_finite_before_attempt"] = False

        report = self.compare_fixture(
            outcome_setup=lose_frame_zero,
            restart_provenance_mutation=restart_once
        )
        self.assertIn("restart_provenance", report)
        provenance = report["restart_provenance"]

        self.assertEqual(provenance["total_rows"], 8)
        self.assertEqual(provenance["source_counts_total"], 8)
        self.assertEqual(provenance["restart_attempts"], 1)
        self.assertEqual(provenance["restart_convergences"], 1)
        self.assertEqual(provenance["restart_failure_reasons"], {})
        self.assertTrue(provenance["reconciled"])

        def inconsistent(rows):
            restart_once(rows)
            row = self._primary(rows, "dynamic_dag_wnls")[0]
            row["ever_acquired_finite_before_attempt"] = True

        module = _load_comparator()
        with self.assertRaises(module.InputIntegrityError):
            self.compare_fixture(
                outcome_setup=lose_frame_zero,
                restart_provenance_mutation=inconsistent
            )

    def test_output_is_exactly_two_atomic_files_and_cap_failure_is_unpublished(self):
        """Breaks if output is non-atomic, unbounded, or contains extra files."""
        report = self.compare_fixture(output=True)
        self.assertEqual(report["status"], "completed")
        self.assertEqual(
            {path.name for path in self.last_output_dir.iterdir()},
            {
                "warm-start-recovery-comparison.json",
                "warm-start-recovery-comparison.md",
            },
        )

        module = _load_comparator()
        with patch.object(module, "_ANALYSIS_OUTPUT_CAP_BYTES", 1):
            with self.assertRaises(module.AnalysisLimitError):
                self.compare_fixture(output=True)
        self.assertFalse(self.last_output_dir.exists())
        self.assertEqual(
            list(self.last_output_dir.parent.glob("comparison.incomplete-*")),
            [],
        )
