"""Tests for the geometric-stability diagnostic primitives."""

from __future__ import annotations

import hashlib
import gzip
import json
import math
import tempfile
import unittest
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

from scripts.diagnostics.analyze_geometric_stability import (
    InputIntegrityError,
    analyze_geometric_stability,
    fixed_pair_metrics,
    geometry_metrics,
    load_trajectory,
    opposite_baseline_side,
    shared_uav_ancestor_metrics,
)
from scripts.diagnostics.replay_localization_calibration import (
    RESTART_BEFORE_FIRST_FINITE_POLICY,
    STRICT_PREVIOUS_POLICY,
    fixed_references,
)


PROCESS_NAME = "calibration.jsonl.gz"
GRAPH_CASES = ("dynamic_dag_wnls", "fixed_refs_wnls")
STATUS_KEYS = ("converged", "stale", "invalid", "failed")
ESTIMATOR_CONTRACT = "variable_weight_nls_full_residual_jacobian_v1"


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


@dataclass(frozen=True)
class GeometricFixture:
    comparison: Path
    comparison_sha256: str
    parent_sha256: str

    def call_args(self, output_dir: Path | None = None) -> dict:
        return {
            "comparison_path": self.comparison,
            "expected_comparison_sha256": self.comparison_sha256,
            "expected_parent_manifest_sha256": self.parent_sha256,
            "output_dir": output_dir,
        }


def trajectory_fixture() -> dict:
    return {
        "config": {
            "bases": [[-1.0, 0.0], [0.0, 1.0]],
            "num": 1,
            "execute": {"time-step": 0.5},
            "formation": {"parts": 1, "bases-id": [[0, 1]]},
        },
        "state": [
            {
                "robots": [
                    {
                        "id": 1,
                        "state": {"x": 0.0, "y": 0.0},
                        "cvt": {"center": [1.0, 1.0]},
                    }
                ]
            },
            {
                "robots": [
                    {
                        "id": 1,
                        "state": {"x": 0.1, "y": 0.2},
                        "cvt": {"center": [1.1, 1.2]},
                    }
                ]
            },
        ],
    }


class GeometricAnalysisFixture(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.fixture_index = 0

    def tearDown(self) -> None:
        self.temporary.cleanup()

    @staticmethod
    def _trajectory(
        *,
        robot_count: int = 2,
        frame_count: int = 2,
    ) -> dict:
        states = []
        for frame_index in range(frame_count):
            robots = []
            for robot_id in range(1, robot_count + 1):
                x = float(robot_id + 0.1 * frame_index)
                y = float((robot_id % 2) + 0.2 * frame_index)
                robots.append(
                    {
                        "id": robot_id,
                        "state": {"x": x, "y": y},
                        "cvt": {"center": [x + 0.25, y - 0.25]},
                    }
                )
            states.append({"robots": robots})
        squad_count = 2 if robot_count >= 4 else 1
        return {
            "config": {
                "bases": [[-3.0, -2.0], [3.0, 4.0]],
                "num": robot_count,
                "execute": {"time-step": 0.5},
                "formation": {
                    "parts": squad_count,
                    "bases-id": [[0, 1] for _ in range(squad_count)],
                },
                "cbfs": {
                    "without-slack": {
                        "comm-fixed": {
                            "min-neighbour-id-offset": -2,
                            "max-neighbour-id-offset": 0,
                            "max-range": 1000.0,
                        }
                    }
                },
            },
            "state": states,
        }

    @staticmethod
    def _status_counts(rows: list[dict], field: str) -> dict[str, int]:
        counts = {status: 0 for status in STATUS_KEYS}
        for row in rows:
            counts[row[field]] += 1
        return counts

    def _rows(
        self,
        policy: str,
        trajectory: dict,
        *,
        seeds: tuple[int, ...] = (101,),
    ) -> list[dict]:
        rows = []
        robot_count = trajectory["config"]["num"]
        for frame_index, frame in enumerate(trajectory["state"]):
            truth_by_id = {
                robot["id"]: [
                    float(robot["state"]["x"]),
                    float(robot["state"]["y"]),
                ]
                for robot in frame["robots"]
            }
            for seed in seeds:
                for graph_case in GRAPH_CASES:
                    for robot_id in range(1, robot_count + 1):
                        truth = truth_by_id[robot_id]
                        references = fixed_references(
                            trajectory["config"], robot_id
                        )
                        measurements = []
                        for base_id in references["base_ids"]:
                            base = trajectory["config"]["bases"][base_id]
                            true_range = float(
                                np.linalg.norm(
                                    np.asarray(truth) - np.asarray(base)
                                )
                            )
                            noise = float(0.01 * (seed % 7 + base_id + 1))
                            measurements.append(
                                {
                                    "kind": "base",
                                    "id": base_id,
                                    "true_range": true_range,
                                    "noise": noise,
                                    "noisy_range": true_range + noise,
                                    "estimated_reference_available": True,
                                }
                            )
                        for reference_id in references["uav_ids"]:
                            reference_truth = truth_by_id[reference_id]
                            true_range = float(
                                np.linalg.norm(
                                    np.asarray(truth)
                                    - np.asarray(reference_truth)
                                )
                            )
                            noise = float(
                                0.01 * (seed % 7 + reference_id + 3)
                            )
                            measurements.append(
                                {
                                    "kind": "uav",
                                    "id": reference_id,
                                    "true_range": true_range,
                                    "noise": noise,
                                    "noisy_range": true_range + noise,
                                    "estimated_reference_available": True,
                                }
                            )
                        estimate = [truth[0] + 0.1, truth[1]]
                        squad_size = math.ceil(
                            robot_count
                            / trajectory["config"]["formation"]["parts"]
                        )
                        rows.append(
                            {
                                "frame_index": frame_index,
                                "seed": seed,
                                "graph_case": graph_case,
                                "robot_id": robot_id,
                                "squad_local_index": (
                                    (robot_id - 1) % squad_size + 1
                                ),
                                "primary_statistics": frame_index != 0,
                                "active_references": {
                                    "base_ids": references["base_ids"],
                                    "uav_ids": references["uav_ids"],
                                },
                                "measurements": measurements,
                                "truth_position": list(truth),
                                "status": "converged",
                                "estimate": estimate,
                                "attempt_status": "converged",
                                "attempt_failure_reason": None,
                                "covariance": [[1.0, 0.0], [0.0, 1.0]],
                                "epsilon": 3.0,
                                "finite": True,
                                "covariance_spd": True,
                                "error_vector": [0.1, 0.0],
                                "error_norm": 0.1,
                                "error_to_epsilon_ratio": 1.0 / 30.0,
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
        return rows

    def _write_bundle(
        self,
        directory: Path,
        rows: list[dict],
        policy: str,
        data_path: Path,
    ) -> dict:
        directory.mkdir()
        lines = [
            json.dumps(
                row,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            ).encode()
            + b"\n"
            for row in rows
        ]
        process_path = directory / PROCESS_NAME
        with process_path.open("wb") as raw:
            with gzip.GzipFile(fileobj=raw, mode="wb", mtime=0) as compressed:
                for line in lines:
                    compressed.write(line)
        settings = {
            "estimator_contract": ESTIMATOR_CONTRACT,
            "run_seeds": sorted({row["seed"] for row in rows}),
            "graph_cases": list(GRAPH_CASES),
            "effective_frame_count": len(
                {row["frame_index"] for row in rows}
            ),
            "max_frames": len({row["frame_index"] for row in rows}),
            "initialization_policy": policy,
            "implementation": {
                "path": str(data_path),
                "sha256": file_sha256(data_path),
            },
        }
        cases = {}
        for graph_case in GRAPH_CASES:
            case_rows = [
                row for row in rows if row["graph_case"] == graph_case
            ]
            primary = [row for row in case_rows if row["primary_statistics"]]
            initialization = [
                row
                for row in case_rows
                if not row["primary_statistics"]
            ]
            cases[graph_case] = {
                "overall": {
                    "attempt_status_counts": self._status_counts(
                        primary, "attempt_status"
                    ),
                    "status_counts": self._status_counts(primary, "status"),
                },
                "initialization_frame": {
                    "attempt_status_counts": self._status_counts(
                        initialization, "attempt_status"
                    ),
                    "status_counts": self._status_counts(
                        initialization, "status"
                    ),
                },
            }
        summary = {
            "estimator_contract": ESTIMATOR_CONTRACT,
            "settings": settings,
            "graph_cases": cases,
            "expected_process_rows": len(rows),
            "process_rows": len(rows),
        }
        summary_path = directory / "summary.json"
        summary_path.write_text(
            json.dumps(summary, sort_keys=True, allow_nan=False) + "\n"
        )
        markdown_path = directory / "summary.md"
        markdown_path.write_text("# fixture\n")
        manifest = {
            "termination_reason": "completed",
            "estimator_contract": ESTIMATOR_CONTRACT,
            "initialization_policy": policy,
            "output_dir": str(directory),
            "input_data": {
                "path": str(data_path),
                "sha256": file_sha256(data_path),
            },
            "settings": settings,
            "compressed_process_sha256": file_sha256(process_path),
            "decompressed_process_sha256": hashlib.sha256(
                b"".join(lines)
            ).hexdigest(),
            "summary_json_sha256": file_sha256(summary_path),
            "summary_markdown_sha256": file_sha256(markdown_path),
        }
        (directory / "manifest.json").write_text(
            json.dumps(manifest, sort_keys=True, allow_nan=False) + "\n"
        )
        return manifest

    @staticmethod
    def _bundle_hashes(directory: Path) -> dict[str, str]:
        return {
            name: file_sha256(directory / name)
            for name in (
                "manifest.json",
                "summary.json",
                "summary.md",
                PROCESS_NAME,
            )
        }

    def write_complete_fixture(
        self,
        *,
        mutation: Callable[[list[dict]], None] | None = None,
        trajectory: dict | None = None,
        seeds: tuple[int, ...] = (101,),
    ) -> GeometricFixture:
        fixture_root = self.root / f"fixture-{self.fixture_index}"
        self.fixture_index += 1
        fixture_root.mkdir()
        trajectory = (
            self._trajectory() if trajectory is None else trajectory
        )
        data_path = fixture_root / "data.json"
        data_path.write_text(
            json.dumps(trajectory, sort_keys=True, allow_nan=False) + "\n"
        )
        strict_rows = self._rows(
            STRICT_PREVIOUS_POLICY, trajectory, seeds=seeds
        )
        restart_rows = self._rows(
            RESTART_BEFORE_FIRST_FINITE_POLICY,
            trajectory,
            seeds=seeds,
        )
        if mutation is not None:
            mutation(restart_rows)
        strict_dir = fixture_root / "strict"
        restart_dir = fixture_root / "restart"
        self._write_bundle(
            strict_dir,
            strict_rows,
            STRICT_PREVIOUS_POLICY,
            data_path,
        )
        self._write_bundle(
            restart_dir,
            restart_rows,
            RESTART_BEFORE_FIRST_FINITE_POLICY,
            data_path,
        )
        parent_dir = fixture_root / "paired"
        parent_dir.mkdir()
        parent_path = parent_dir / "manifest.json"
        parent_path.write_text(
            json.dumps(
                {
                    "schema": "cbf2026-warm-start-recovery-parent-v1",
                    "termination_reason": "completed",
                },
                sort_keys=True,
            )
            + "\n"
        )
        parent_sha256 = file_sha256(parent_path)
        comparison_path = fixture_root / "comparison.json"
        comparison = {
            "schema": "cbf2026-warm-start-recovery-comparison-v1",
            "status": "completed",
            "source": {
                "paired_bundle_dir": str(parent_dir),
                "paired_parent_manifest_sha256": parent_sha256,
                "strict_child_dir": str(strict_dir),
                "strict_child_hashes": self._bundle_hashes(strict_dir),
                "restart_child_dir": str(restart_dir),
                "restart_child_hashes": self._bundle_hashes(restart_dir),
                "input_data_sha256": file_sha256(data_path),
            },
        }
        comparison_path.write_text(
            json.dumps(comparison, sort_keys=True, allow_nan=False) + "\n"
        )
        return GeometricFixture(
            comparison=comparison_path,
            comparison_sha256=file_sha256(comparison_path),
            parent_sha256=parent_sha256,
        )

    @staticmethod
    def mutate_restart_key(rows: list[dict]) -> None:
        rows[-1]["robot_id"] = 1

    @staticmethod
    def mutate_restart_noise(rows: list[dict]) -> None:
        measurement = rows[-1]["measurements"][0]
        measurement["noise"] += 0.5
        measurement["noisy_range"] += 0.5

    @staticmethod
    def mutate_restart_reference(rows: list[dict]) -> None:
        rows[-1]["active_references"]["uav_ids"][0] = 7
        rows[-1]["measurements"][-1]["id"] = 7

    @staticmethod
    def mutate_restart_truth(rows: list[dict]) -> None:
        rows[-1]["truth_position"][0] += 1.0


class EvidenceIntegrityTests(GeometricAnalysisFixture):
    def test_exact_trust_roots_and_every_pair_are_verified(self):
        fixture = self.write_complete_fixture()
        report = analyze_geometric_stability(**fixture.call_args())
        self.assertEqual(report["integrity"]["paired_rows"], 8)
        self.assertTrue(report["integrity"]["paired_inputs_equal"])
        self.assertTrue(report["integrity"]["source_hashes_unchanged"])

    def test_wrong_comparison_or_parent_hash_fails_before_streaming(self):
        fixture = self.write_complete_fixture()
        with self.assertRaises(InputIntegrityError):
            analyze_geometric_stability(
                fixture.comparison,
                expected_comparison_sha256="0" * 64,
                expected_parent_manifest_sha256=fixture.parent_sha256,
            )
        with self.assertRaises(InputIntegrityError):
            analyze_geometric_stability(
                fixture.comparison,
                expected_comparison_sha256=fixture.comparison_sha256,
                expected_parent_manifest_sha256="f" * 64,
            )

    def test_key_noise_reference_or_truth_mutation_fails_closed(self):
        for mutation in (
            self.mutate_restart_key,
            self.mutate_restart_noise,
            self.mutate_restart_reference,
            self.mutate_restart_truth,
        ):
            fixture = self.write_complete_fixture(mutation=mutation)
            with self.assertRaises(InputIntegrityError):
                analyze_geometric_stability(**fixture.call_args())


class ScientificAggregationTests(GeometricAnalysisFixture):
    @staticmethod
    def _dynamic_primary(rows: list[dict]) -> list[dict]:
        return [
            row
            for row in rows
            if row["graph_case"] == "dynamic_dag_wnls"
            and row["primary_statistics"]
        ]

    @staticmethod
    def _make_invalid(row: dict) -> None:
        row.update(
            {
                "status": "invalid",
                "attempt_status": "invalid",
                "attempt_failure_reason": (
                    "non-finite or malformed WNLS input"
                ),
                "estimate": None,
                "covariance": None,
                "epsilon": None,
                "finite": False,
                "covariance_spd": False,
                "error_vector": None,
                "error_norm": None,
                "error_to_epsilon_ratio": None,
                "state_containment": False,
                "containment": False,
                "phi_min_eigenvalue": None,
                "phi_condition": None,
            }
        )

    def two_seed_fixture(self) -> GeometricFixture:
        return self.write_complete_fixture(seeds=(101, 102))

    def one_invalid_restart_row_fixture(self) -> GeometricFixture:
        def mutation(rows: list[dict]) -> None:
            self._make_invalid(self._dynamic_primary(rows)[-1])

        return self.write_complete_fixture(mutation=mutation)

    def seven_depth_two_squad_fixture(self) -> GeometricFixture:
        return self.write_complete_fixture(
            trajectory=self._trajectory(robot_count=14),
        )

    def q_fixture(
        self,
        *,
        error: list[float],
        covariance: np.ndarray,
    ) -> GeometricFixture:
        def mutation(rows: list[dict]) -> None:
            row = self._dynamic_primary(rows)[0]
            row["estimate"] = [
                row["truth_position"][0] + float(error[0]),
                row["truth_position"][1] + float(error[1]),
            ]
            row["error_vector"] = [float(error[0]), float(error[1])]
            row["error_norm"] = float(np.linalg.norm(error))
            row["covariance"] = np.asarray(covariance).tolist()
            row["state_containment"] = False
            row["containment"] = False

        return self.write_complete_fixture(mutation=mutation)

    def invalid_covariance_fixture(self) -> GeometricFixture:
        def mutation(rows: list[dict]) -> None:
            row = self._dynamic_primary(rows)[0]
            row["covariance"] = [[1.0, 0.5], [0.0, 1.0]]
            row["finite"] = False
            row["covariance_spd"] = False

        return self.write_complete_fixture(mutation=mutation)

    def analyze(self, fixture: GeometricFixture) -> dict:
        return analyze_geometric_stability(**fixture.call_args())

    def test_true_geometry_is_counted_once_per_trajectory_tuple(self):
        report = self.analyze(self.two_seed_fixture())
        geometry = report["geometry"]["true_dynamic_all_primary"]["overall"]
        self.assertEqual(geometry["trajectory_tuple_count"], 2)
        self.assertEqual(geometry["range_seed_repetitions_verified"], 2)

    def test_estimated_geometry_uses_only_finite_restart_rows(self):
        report = self.analyze(self.one_invalid_restart_row_fixture())
        estimated = report["geometry"]["estimated_dynamic_finite"]["overall"]
        self.assertEqual(estimated["denominator"], 1)
        self.assertEqual(
            report["availability"]["overall"]["primary_attempts"],
            2,
        )

    def test_depth_time_squad_and_seed_strata_reconcile(self):
        report = self.analyze(self.seven_depth_two_squad_fixture())
        error = report["absolute_error"]
        self.assertEqual(
            sum(
                item["denominator"]
                for item in error["by_depth"].values()
            ),
            error["overall"]["denominator"],
        )
        self.assertEqual(
            sum(
                item["denominator"]
                for item in error["by_time_bin"].values()
            ),
            error["overall"]["denominator"],
        )

    def test_q_is_recomputed_from_error_and_covariance(self):
        report = self.analyze(
            self.q_fixture(
                error=[3.1, 0.0],
                covariance=np.eye(2),
            )
        )
        self.assertEqual(report["calibration"]["overall"]["q_above_9"], 1)

    def test_error_and_containment_denominators_remain_separate(self):
        report = self.analyze(self.invalid_covariance_fixture())
        calibration = report["calibration"]["overall"]
        self.assertGreater(
            calibration["converged_error_denominator"],
            calibration["finite_q_denominator"],
        )


class AvailabilityTests(GeometricAnalysisFixture):
    @staticmethod
    def _unavailable(row: dict, attempt_status: str) -> None:
        reason = {
            "invalid": "non-finite or malformed WNLS input",
            "failed": "maximum WNLS iterations exceeded",
        }[attempt_status]
        row.update(
            {
                "status": attempt_status,
                "attempt_status": attempt_status,
                "attempt_failure_reason": reason,
                "estimate": None,
                "covariance": None,
                "epsilon": None,
                "finite": False,
                "covariance_spd": False,
                "error_vector": None,
                "error_norm": None,
                "error_to_epsilon_ratio": None,
                "state_containment": False,
                "containment": False,
                "phi_min_eigenvalue": None,
                "phi_condition": None,
            }
        )

    @staticmethod
    def _reconcile_restart_provenance(rows: list[dict]) -> None:
        state: dict[tuple[int, str, int], tuple[bool, bool]] = {}
        for row in rows:
            key = (row["seed"], row["graph_case"], row["robot_id"])
            ever, previous_finite = state.get(key, (False, False))
            row["ever_acquired_finite_before_attempt"] = ever
            if row["frame_index"] == 0:
                source = "deployment_frame_zero"
            elif previous_finite:
                source = "previous_finite"
            elif not ever:
                source = "deployment_restart_before_first_finite"
            else:
                source = "strict_previous_missing"
            row["initial_estimate_source"] = source
            finite = row["finite"]
            state[key] = (ever or finite, finite)

    def status_series(self, statuses: list[str]) -> GeometricFixture:
        trajectory = self._trajectory(
            robot_count=2,
            frame_count=len(statuses) + 1,
        )

        def mutation(rows: list[dict]) -> None:
            selected = [
                row
                for row in rows
                if row["graph_case"] == "dynamic_dag_wnls"
                and row["robot_id"] == 2
                and row["primary_statistics"]
            ]
            for row, status in zip(selected, statuses):
                if status != "converged":
                    self._unavailable(row, status)
            self._reconcile_restart_provenance(rows)

        return self.write_complete_fixture(
            mutation=mutation,
            trajectory=trajectory,
        )

    def first_finite_and_never_finite_fixture(self) -> GeometricFixture:
        trajectory = self._trajectory(robot_count=1, frame_count=3)

        def mutation(rows: list[dict]) -> None:
            for row in rows:
                if (
                    row["graph_case"] == "dynamic_dag_wnls"
                    and row["seed"] == 102
                ):
                    self._unavailable(row, "invalid")
            self._reconcile_restart_provenance(rows)

        return self.write_complete_fixture(
            mutation=mutation,
            trajectory=trajectory,
            seeds=(101, 102),
        )

    def mixed_status_fixture(self) -> GeometricFixture:
        return self.status_series(["converged", "invalid", "failed"])

    def analyze(self, fixture: GeometricFixture) -> dict:
        return analyze_geometric_stability(**fixture.call_args())

    def test_longest_unavailable_streak_uses_seed_robot_series(self):
        report = self.analyze(
            self.status_series(
                ["converged", "invalid", "invalid", "converged"]
            )
        )
        self.assertEqual(
            report["availability"]["overall"][
                "maximum_consecutive_unavailable_frames"
            ],
            2,
        )

    def test_first_finite_lag_is_stratified_without_dropping_never_finite_series(
        self,
    ):
        report = self.analyze(
            self.first_finite_and_never_finite_fixture()
        )
        lineage = report["availability"]["first_finite_acquisition"]
        self.assertEqual(lineage["acquired_series"], 1)
        self.assertEqual(lineage["never_acquired_series"], 1)

    def test_failure_reasons_reconcile_to_all_primary_attempts(self):
        report = self.analyze(self.mixed_status_fixture())
        overall = report["availability"]["overall"]
        self.assertEqual(
            sum(overall["attempt_status_counts"].values()),
            overall["primary_attempts"],
        )


class DependencyAndBranchTests(GeometricAnalysisFixture):
    def dependency_and_branch_fixture(self) -> GeometricFixture:
        return self.write_complete_fixture()

    def analyze(self, fixture: GeometricFixture) -> dict:
        return analyze_geometric_stability(**fixture.call_args())

    def test_shared_uav_ancestor_counts_overlap_not_known_bases(self):
        lineage = {
            2: frozenset({1, 2}),
            3: frozenset({1, 3}),
            4: frozenset({4}),
        }
        result = shared_uav_ancestor_metrics([2, 3, 4], lineage)
        self.assertEqual(result["reference_pair_count"], 3)
        self.assertEqual(result["pairs_with_shared_uav_ancestor"], 1)
        self.assertEqual(result["shared_uav_ancestor_instances"], 1)

    def test_opposite_side_detects_mirror_proxy(self):
        references = [[-1.0, 0.0], [1.0, 0.0]]
        self.assertTrue(
            opposite_baseline_side(
                estimate=[0.0, -1.0],
                truth=[0.0, 1.0],
                reference_positions=references,
            )
        )
        self.assertFalse(
            opposite_baseline_side(
                estimate=[0.0, 0.5],
                truth=[0.0, 1.0],
                reference_positions=references,
            )
        )
        self.assertIsNone(
            opposite_baseline_side(
                estimate=[0.0, 0.0],
                truth=[0.0, 1.0],
                reference_positions=references,
            )
        )

    def test_calibration_is_stratified_by_uav_refs_ancestry_and_side(self):
        report = self.analyze(self.dependency_and_branch_fixture())
        mechanisms = report["calibration"]["mechanism_diagnostics"]
        self.assertIn("by_uav_reference_count", mechanisms)
        self.assertIn(
            "by_shared_uav_ancestor_pair_count", mechanisms
        )
        self.assertIn(
            "two_reference_opposite_baseline_side", mechanisms
        )


class GeometryPrimitiveTests(unittest.TestCase):
    def test_orthogonal_pair_has_identity_geometry(self):
        result = geometry_metrics([0.0, 0.0], [[1.0, 0.0], [0.0, 1.0]])
        self.assertAlmostEqual(result["lambda_min"], 1.0)
        self.assertAlmostEqual(result["lambda_max"], 1.0)
        self.assertAlmostEqual(result["normalized_lambda_min"], 1.0)
        self.assertTrue(result["positive_definite"])

    def test_collinear_pair_is_not_positive_definite(self):
        result = geometry_metrics([0.0, 0.0], [[1.0, 0.0], [-2.0, 0.0]])
        self.assertAlmostEqual(result["lambda_min"], 0.0)
        self.assertFalse(result["positive_definite"])

    def test_adding_reference_cannot_reduce_geometry_eigenvalue(self):
        pair = geometry_metrics([0.0, 0.0], [[1.0, 0.0], [2.0, 0.1]])
        active = geometry_metrics(
            [0.0, 0.0],
            [[1.0, 0.0], [2.0, 0.1], [0.0, 1.0]],
        )
        self.assertGreaterEqual(active["lambda_min"], pair["lambda_min"])

    def test_extreme_finite_coordinates_produce_finite_geometry(self):
        result = geometry_metrics([0.0, 0.0], [[1e308, 0.0], [0.0, 1e308]])
        self.assertAlmostEqual(result["lambda_min"], 1.0)
        self.assertAlmostEqual(result["lambda_max"], 1.0)
        self.assertTrue(result["finite"])

    def test_fixed_pair_extreme_finite_coordinates_fail_closed(self):
        with self.assertRaises(InputIntegrityError):
            fixed_pair_metrics([0.0, 0.0], [[1e308, 0.0], [0.0, 1e308]])

    def test_fixed_pair_reports_smaller_angle_to_collinearity(self):
        result = fixed_pair_metrics(
            [0.0, 0.0],
            [[1.0, 0.0], [-math.sqrt(3.0) / 2.0, 0.5]],
        )
        self.assertAlmostEqual(
            result["noncollinearity_angle_rad"],
            math.pi / 6.0,
        )


class TrajectoryContractTests(unittest.TestCase):
    def write_trajectory(self, payload: dict) -> tuple[Path, str]:
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as source:
            json.dump(payload, source)
        path = Path(source.name)
        self.addCleanup(path.unlink, missing_ok=True)
        return path, hashlib.sha256(path.read_bytes()).hexdigest()

    @staticmethod
    def remove_target(payload: dict) -> dict:
        del payload["state"][0]["robots"][0]["cvt"]
        return payload

    @staticmethod
    def duplicate_robot_id(payload: dict) -> dict:
        duplicate = payload["state"][0]["robots"][0].copy()
        payload["state"][0]["robots"].append(duplicate)
        return payload

    @staticmethod
    def insert_nonfinite_state(payload: dict) -> dict:
        payload["state"][0]["robots"][0]["state"]["x"] = float("inf")
        return payload

    def test_loader_returns_truth_targets_and_time_step(self):
        path, digest = self.write_trajectory(trajectory_fixture())
        loaded = load_trajectory(path, expected_sha256=digest)
        self.assertEqual(loaded["time_step"], 0.5)
        self.assertEqual(loaded["truth"][1][1], [0.1, 0.2])
        self.assertEqual(loaded["targets"][1][1], [1.1, 1.2])

    def test_wrong_hash_missing_target_duplicate_id_and_nonfinite_fail_closed(self):
        path, digest = self.write_trajectory(trajectory_fixture())
        with self.assertRaises(InputIntegrityError):
            load_trajectory(path, expected_sha256="0" * 64)
        for mutation in (
            self.remove_target,
            self.duplicate_robot_id,
            self.insert_nonfinite_state,
        ):
            bad_path, bad_digest = self.write_trajectory(mutation(trajectory_fixture()))
            with self.assertRaises(InputIntegrityError):
                load_trajectory(bad_path, expected_sha256=bad_digest)
