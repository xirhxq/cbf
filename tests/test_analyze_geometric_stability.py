"""Tests for the geometric-stability diagnostic primitives."""

from __future__ import annotations

import hashlib
import gzip
import json
import math
import stat
import subprocess
import sys
import tempfile
import unittest
from dataclasses import dataclass
from pathlib import Path
from typing import Callable
from unittest.mock import patch

import numpy as np

import scripts.diagnostics.analyze_geometric_stability as analyzer_module
from scripts.diagnostics.analyze_geometric_stability import (
    OUTPUT_JSON_NAME,
    OUTPUT_MARKDOWN_NAME,
    AnalysisLimitError,
    InputIntegrityError,
    analyze_geometric_stability,
    fixed_pair_metrics,
    fixed_pair_tracking_margin,
    geometry_metrics,
    load_trajectory,
    nominal_fixed_pair_metrics,
    opposite_baseline_side,
    shared_uav_ancestor_metrics,
)
from scripts.diagnostics import run_diagnostic
from scripts.diagnostics.replay_localization_calibration import (
    RESTART_BEFORE_FIRST_FINITE_POLICY,
    STRICT_PREVIOUS_POLICY,
    active_references,
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
        self.root = Path(self.temporary.name).resolve()
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
                "bases": [
                    [-3.0, -2.0],
                    [3.0, 4.0],
                    [2000.0, 2000.0],
                ],
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
                        if graph_case == "dynamic_dag_wnls":
                            references = active_references(
                                trajectory["config"],
                                robot_id,
                                {
                                    identifier: np.asarray(position)
                                    for identifier, position in truth_by_id.items()
                                },
                            )
                        else:
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
                        error_vector = [
                            truth[0] - estimate[0],
                            truth[1] - estimate[1],
                        ]
                        error_norm = float(np.linalg.norm(error_vector))
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
                                "error_vector": error_vector,
                                "error_norm": error_norm,
                                "error_to_epsilon_ratio": (
                                    error_norm / 3.0
                                ),
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

    @staticmethod
    def reconcile_restart_provenance(rows: list[dict]) -> None:
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

    def write_complete_fixture(
        self,
        *,
        mutation: Callable[[list[dict]], None] | None = None,
        paired_mutation: Callable[[list[dict]], None] | None = None,
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
        if paired_mutation is not None:
            paired_mutation(strict_rows)
            paired_mutation(restart_rows)
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

    @staticmethod
    def omit_dynamic_addition(rows: list[dict]) -> None:
        row = next(
            item
            for item in rows
            if item["graph_case"] == "dynamic_dag_wnls"
            and item["frame_index"] == 1
            and item["robot_id"] == 2
        )
        row["active_references"]["base_ids"].remove(1)
        row["measurements"] = [
            measurement
            for measurement in row["measurements"]
            if not (
                measurement["kind"] == "base"
                and measurement["id"] == 1
            )
        ]

    @staticmethod
    def add_ineligible_dynamic_reference(rows: list[dict]) -> None:
        row = next(
            item
            for item in rows
            if item["graph_case"] == "dynamic_dag_wnls"
            and item["frame_index"] == 1
            and item["robot_id"] == 2
        )
        truth = np.asarray(row["truth_position"])
        reference = np.asarray([2000.0, 2000.0])
        true_range = float(np.linalg.norm(truth - reference))
        row["active_references"]["base_ids"].append(2)
        row["measurements"].insert(
            2,
            {
                "kind": "base",
                "id": 2,
                "true_range": true_range,
                "noise": 0.125,
                "noisy_range": true_range + 0.125,
                "estimated_reference_available": True,
            },
        )


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

    def test_dynamic_topology_is_reconstructed_not_trusted_from_rows(self):
        complete = self.write_complete_fixture()
        report = analyze_geometric_stability(**complete.call_args())
        dynamic = report["geometry"]["true_dynamic_all_primary"]["overall"]
        self.assertGreater(
            dynamic["metrics"]["reference_count"]["maximum"],
            2,
        )
        for mutation in (
            self.omit_dynamic_addition,
            self.add_ineligible_dynamic_reference,
        ):
            fixture = self.write_complete_fixture(
                paired_mutation=mutation
            )
            with self.assertRaises(InputIntegrityError):
                analyze_geometric_stability(**fixture.call_args())

    def test_missing_uav_target_fails_closed_before_aggregation(self):
        trajectory = self._trajectory()
        del trajectory["state"][1]["robots"][0]["cvt"]["center"]
        fixture = self.write_complete_fixture(trajectory=trajectory)

        with self.assertRaisesRegex(
            InputIntegrityError,
            "finite 2D point",
        ):
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
                "state_containment": None,
                "containment": False,
                "phi_min_eigenvalue": None,
                "phi_condition": None,
            }
        )

    def two_seed_fixture(self) -> GeometricFixture:
        return self.write_complete_fixture(seeds=(101, 102))

    def base_and_uav_target_triangle_fixture(self) -> GeometricFixture:
        trajectory = self._trajectory()
        trajectory["config"]["bases"][0] = [0.0, 0.0]
        trajectory["config"]["bases"][1] = [0.0, 5.0]
        first = {
            robot["id"]: robot
            for robot in trajectory["state"][0]["robots"]
        }
        first[1]["cvt"]["center"] = [8.0, 7.0]
        first[2]["cvt"]["center"] = [9.0, 6.0]
        primary = {
            robot["id"]: robot
            for robot in trajectory["state"][1]["robots"]
        }
        primary[1]["cvt"]["center"] = [3.0, 4.0]
        primary[2]["cvt"]["center"] = [3.0, 0.0]
        return self.write_complete_fixture(trajectory=trajectory)

    def one_invalid_restart_row_fixture(self) -> GeometricFixture:
        def mutation(rows: list[dict]) -> None:
            selected = [
                row
                for row in rows
                if row["graph_case"] == "dynamic_dag_wnls"
                and row["robot_id"] == 2
            ]
            for row in selected:
                self._make_invalid(row)
            self.reconcile_restart_provenance(rows)

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
                row["truth_position"][0] - float(error[0]),
                row["truth_position"][1] - float(error[1]),
            ]
            row["error_vector"] = [
                row["truth_position"][index] - row["estimate"][index]
                for index in range(2)
            ]
            row["error_norm"] = float(
                np.linalg.norm(row["error_vector"])
            )
            row["error_to_epsilon_ratio"] = (
                row["error_norm"] / row["epsilon"]
            )
            row["covariance"] = np.asarray(covariance).tolist()
            row["state_containment"] = False
            row["containment"] = False

        return self.write_complete_fixture(mutation=mutation)

    def error_semantic_drift_fixture(
        self,
        mutation: Callable[[dict], None],
    ) -> GeometricFixture:
        def mutate_rows(rows: list[dict]) -> None:
            mutation(self._dynamic_primary(rows)[0])

        return self.write_complete_fixture(mutation=mutate_rows)

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
        nominal = report["geometry"][
            "nominal_fixed_pair_all_primary"
        ]["overall"]
        self.assertEqual(nominal["trajectory_tuple_count"], 2)
        self.assertEqual(nominal["range_seed_repetitions_verified"], 2)

    def test_true_trajectory_metrics_are_not_assigned_to_first_seed(self):
        report = self.analyze(self.two_seed_fixture())
        for name in (
            "nominal_fixed_pair_all_primary",
            "true_dynamic_all_primary",
            "true_fixed_pair_all_primary",
        ):
            family = report["geometry"][name]
            self.assertNotIn("by_seed", family)
            self.assertFalse(family["seed_stratification"]["applicable"])
            self.assertEqual(
                family["seed_stratification"]["reason"],
                "trajectory_level_quantity",
            )
        tracking = report["geometry"]["target_tracking"][
            "true_position"
        ]
        self.assertNotIn("by_seed", tracking)
        self.assertFalse(tracking["seed_stratification"]["applicable"])

    def test_nominal_base_and_uav_targets_drive_hand_derived_margin(self):
        report = self.analyze(self.base_and_uav_target_triangle_fixture())
        nominal = report["geometry"][
            "nominal_fixed_pair_all_primary"
        ]["by_depth"]["2"]

        self.assertAlmostEqual(
            nominal["metrics"][
                "admissible_tracking_deviation_supremum"
            ]["maximum"],
            1.0 / 3.0,
        )
        self.assertAlmostEqual(
            nominal["metrics"]["included_angle_rad"]["maximum"],
            math.pi / 2.0,
        )

    def test_estimated_tracking_margin_uses_explicit_finite_denominator(self):
        report = self.analyze(self.one_invalid_restart_row_fixture())
        margin = report["geometry"][
            "estimated_fixed_pair_tracking_margin_finite"
        ]["overall"]

        self.assertEqual(margin["denominator"], 1)
        self.assertEqual(
            margin["strictly_within_admissible_count"]
            + margin["not_strictly_within_admissible_count"],
            1,
        )
        self.assertEqual(
            margin["inapplicability_reasons"][
                "restart_row_not_finite"
            ],
            1,
        )

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

    def test_stored_error_and_containment_semantic_drift_fails_closed(self):
        mutations = (
            lambda row: row.update({"error_vector": [9.0, 0.0]}),
            lambda row: row.update({"error_norm": 9.0}),
            lambda row: row.update({"error_to_epsilon_ratio": 9.0}),
            lambda row: row.update({"state_containment": False}),
            lambda row: row.update({"containment": False}),
        )
        for mutation in mutations:
            fixture = self.error_semantic_drift_fixture(mutation)
            with self.assertRaises(InputIntegrityError):
                self.analyze(fixture)

    def test_modeled_fim_radius_stats_use_valid_rows_and_frozen_strata(self):
        report = self.analyze(self.one_invalid_restart_row_fixture())
        modeled = report["geometry"]["modeled_fim_valid"]
        self.assertEqual(modeled["overall"]["denominator"], 1)
        for metric in (
            "phi_min_eigenvalue",
            "phi_condition",
            "epsilon",
        ):
            self.assertEqual(
                modeled["overall"]["metrics"][metric]["denominator"],
                1,
            )
        self.assertEqual(
            sum(
                bucket["denominator"]
                for bucket in modeled["by_depth"].values()
            ),
            modeled["overall"]["denominator"],
        )
        self.assertEqual(
            sum(
                bucket["denominator"]
                for bucket in modeled["by_time_bin"].values()
            ),
            modeled["overall"]["denominator"],
        )
        self.assertEqual(
            modeled["overall"]["inapplicability_reasons"][
                "restart_row_not_finite"
            ],
            1,
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
                "state_containment": None,
                "containment": False,
                "phi_min_eigenvalue": None,
                "phi_condition": None,
            }
        )

    @staticmethod
    def _reconcile_restart_provenance(rows: list[dict]) -> None:
        GeometricAnalysisFixture.reconcile_restart_provenance(rows)

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


class PublicationTests(GeometricAnalysisFixture):
    def test_output_is_atomic_exact_and_below_ten_megabytes(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"

        analyze_geometric_stability(**fixture.call_args(output_dir=output))

        self.assertEqual(
            {path.name for path in output.iterdir()},
            {OUTPUT_JSON_NAME, OUTPUT_MARKDOWN_NAME},
        )
        self.assertLess(run_diagnostic.allocated_bytes(output), 10_000_000)

    def test_existing_output_or_nested_source_output_is_rejected(self):
        fixture = self.write_complete_fixture()
        for output in (
            fixture.comparison.parent / "restart" / "analysis",
            self.root / "exists",
        ):
            output.mkdir(parents=True)
            with self.assertRaises((InputIntegrityError, AnalysisLimitError)):
                analyze_geometric_stability(**fixture.call_args(output_dir=output))

    def test_nonexistent_output_nested_in_source_is_rejected(self):
        fixture = self.write_complete_fixture()
        output = fixture.comparison.parent / "restart" / "new" / "run"

        with self.assertRaisesRegex(AnalysisLimitError, "protected input"):
            analyze_geometric_stability(**fixture.call_args(output_dir=output))

        self.assertFalse(output.parent.exists())

    def test_live_floor_failure_leaves_no_published_output(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        with self.assertRaises(AnalysisLimitError):
            analyze_geometric_stability(
                **fixture.call_args(output_dir=output),
                live_guard=lambda: (_ for _ in ()).throw(
                    AnalysisLimitError("below live floor")
                ),
            )
        self.assertFalse(output.exists())

    def test_markdown_states_exploratory_claim_boundary(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        analyze_geometric_stability(**fixture.call_args(output_dir=output))
        text = (output / OUTPUT_MARKDOWN_NAME).read_text(encoding="utf-8")
        self.assertIn("post-hoc exploratory", text)
        self.assertIn("not a deterministic true-error bound", text)
        self.assertIn("one truth trajectory", text)

    def test_json_and_markdown_render_compact_nominal_perturbation_families(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"

        analyze_geometric_stability(**fixture.call_args(output_dir=output))

        json_text = (output / OUTPUT_JSON_NAME).read_text(
            encoding="utf-8"
        )
        report = json.loads(json_text)
        markdown = (output / OUTPUT_MARKDOWN_NAME).read_text(
            encoding="utf-8"
        )
        self.assertIn(
            "nominal_fixed_pair_all_primary",
            report["geometry"],
        )
        self.assertIn(
            "estimated_fixed_pair_tracking_margin_finite",
            report["geometry"],
        )
        self.assertIn("Nominal target perturbation", markdown)
        self.assertIn("nominal_fixed_pair_all_primary", markdown)
        self.assertIn(
            "estimated_fixed_pair_tracking_margin_finite",
            markdown,
        )
        for raw_field in (
            '"measurements"',
            '"truth_position"',
            '"calibration.jsonl.gz"',
        ):
            self.assertNotIn(raw_field, json_text)
            self.assertNotIn(raw_field, markdown)

    def test_json_is_deterministic_complete_and_matches_returned_report(self):
        fixture = self.write_complete_fixture()
        first = self.root / "analysis" / "first"
        second = self.root / "analysis" / "second"

        report = analyze_geometric_stability(**fixture.call_args(output_dir=first))
        analyze_geometric_stability(**fixture.call_args(output_dir=second))

        first_bytes = (first / OUTPUT_JSON_NAME).read_bytes()
        self.assertEqual(first_bytes, (second / OUTPUT_JSON_NAME).read_bytes())
        self.assertEqual(json.loads(first_bytes), report)
        self.assertEqual(report["status"], "completed")
        self.assertEqual(report["protocol"]["scope"], "post-hoc exploratory")
        self.assertEqual(report["protocol"]["truth_trajectory_count"], 1)
        self.assertEqual(report["protocol"]["time_bins_seconds"], [0, 50, 100, 150, 200, 250])
        self.assertEqual(
            report["claim_boundary"][0],
            "not a deterministic true-error bound",
        )
        self.assertTrue(first_bytes.endswith(b"\n"))

    def test_cli_publishes_with_registered_arguments(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        result = subprocess.run(
            [
                sys.executable,
                "-m",
                "scripts.diagnostics.analyze_geometric_stability",
                "--comparison",
                str(fixture.comparison),
                "--expected-comparison-sha256",
                fixture.comparison_sha256,
                "--expected-parent-manifest-sha256",
                fixture.parent_sha256,
                "--output-dir",
                str(output),
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue((output / OUTPUT_JSON_NAME).is_file())
        self.assertEqual(result.stdout, "")

    def test_cli_success_does_not_write_stdout_after_publication(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        arguments = [
            "--comparison",
            str(fixture.comparison),
            "--expected-comparison-sha256",
            fixture.comparison_sha256,
            "--expected-parent-manifest-sha256",
            fixture.parent_sha256,
            "--output-dir",
            str(output),
        ]

        with patch(
            "builtins.print",
            side_effect=BrokenPipeError("stdout is closed"),
        ):
            exit_code = analyzer_module.main(arguments)

        self.assertEqual(exit_code, 0)
        self.assertTrue((output / OUTPUT_JSON_NAME).is_file())

    def test_cli_integrity_failure_is_nonzero_without_output(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        result = subprocess.run(
            [
                sys.executable,
                "-m",
                "scripts.diagnostics.analyze_geometric_stability",
                "--comparison",
                str(fixture.comparison),
                "--expected-comparison-sha256",
                "0" * 64,
                "--expected-parent-manifest-sha256",
                fixture.parent_sha256,
                "--output-dir",
                str(output),
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(output.exists())


class PublicationIntegrityTests(GeometricAnalysisFixture):
    @staticmethod
    def _staging_entries(output: Path) -> set[str] | None:
        parent = output.parent
        if not parent.is_dir() or parent.is_symlink():
            return None
        staging = list(parent.glob(f"{output.name}.incomplete-*"))
        if len(staging) != 1:
            return None
        return {item.name for item in staging[0].iterdir()}

    def _assert_descriptors_closed(
        self,
        descriptors: list[int] | set[int],
        *,
        fstat: Callable[[int], object],
        close: Callable[[int], None],
    ) -> None:
        leaked: list[int] = []
        for descriptor in set(descriptors):
            try:
                fstat(descriptor)
            except OSError as error:
                self.assertEqual(error.errno, analyzer_module.errno.EBADF)
            else:
                leaked.append(descriptor)
        for descriptor in leaked:
            close(descriptor)
        self.assertEqual(leaked, [])

    def test_start_gate_precedes_all_source_access(self):
        output = self.root / "analysis" / "run"
        missing_comparison = self.root / "missing-comparison.json"
        with patch.object(
            analyzer_module,
            "_available_bytes_for_guard",
            return_value=analyzer_module.START_BYTES - 1,
        ):
            with self.assertRaisesRegex(AnalysisLimitError, "below start"):
                analyze_geometric_stability(
                    missing_comparison,
                    expected_comparison_sha256="0" * 64,
                    expected_parent_manifest_sha256="1" * 64,
                    output_dir=output,
                )
        self.assertFalse(output.parent.exists())

    def test_combined_entry_guard_runs_builtin_before_failing_caller(self):
        output = self.root / "analysis" / "run"
        events: list[str] = []

        def start_gate(parent_fd: int | None, path: Path) -> int:
            del parent_fd, path
            events.append("builtin")
            return analyzer_module.START_BYTES + 1

        def caller_guard() -> None:
            events.append("caller")
            raise AnalysisLimitError("caller stopped analysis")

        with patch.object(
            analyzer_module,
            "_available_bytes_for_guard",
            side_effect=start_gate,
        ):
            with self.assertRaisesRegex(AnalysisLimitError, "caller stopped"):
                analyze_geometric_stability(
                    self.root / "missing-comparison.json",
                    expected_comparison_sha256="0" * 64,
                    expected_parent_manifest_sha256="1" * 64,
                    output_dir=output,
                    live_guard=caller_guard,
                )

        self.assertEqual(events, ["builtin", "caller"])

    def test_caller_guard_surrounds_both_writes_and_publication(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        observed: list[frozenset[str] | None] = []

        def guard() -> None:
            entries = self._staging_entries(output)
            observed.append(None if entries is None else frozenset(entries))

        analyze_geometric_stability(
            **fixture.call_args(output_dir=output),
            live_guard=guard,
        )

        self.assertEqual(observed[0], None)
        self.assertIn(frozenset(), observed)
        self.assertIn(frozenset({OUTPUT_JSON_NAME}), observed)
        completed = frozenset({OUTPUT_JSON_NAME, OUTPUT_MARKDOWN_NAME})
        self.assertGreaterEqual(observed.count(completed), 2)

    def test_no_fallible_resource_probe_occurs_after_rename(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        real_probe = analyzer_module._available_bytes_for_guard
        probe_calls = 0

        def probe(parent_fd, fallback_path: Path) -> int:
            nonlocal probe_calls
            probe_calls += 1
            if output.exists():
                raise AnalysisLimitError("probe occurred after commit")
            return real_probe(parent_fd, fallback_path)

        with patch.object(
            analyzer_module,
            "_available_bytes_for_guard",
            side_effect=probe,
        ):
            report = analyze_geometric_stability(
                **fixture.call_args(output_dir=output)
            )

        self.assertGreater(probe_calls, 0)
        self.assertEqual(report["status"], "completed")
        self.assertTrue((output / OUTPUT_JSON_NAME).is_file())

    def test_python_close_hook_cannot_interrupt_postcommit_teardown(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        real_open = analyzer_module.os.open
        real_close = analyzer_module.os.close
        real_fstat = analyzer_module.os.fstat
        opened: list[int] = []
        interrupted = False

        def tracked_open(*args, **kwargs):
            descriptor = real_open(*args, **kwargs)
            opened.append(descriptor)
            return descriptor

        def interrupted_close(descriptor: int) -> None:
            nonlocal interrupted
            metadata = real_fstat(descriptor)
            if (
                not interrupted
                and output.exists()
                and stat.S_ISDIR(metadata.st_mode)
                and analyzer_module._directory_identity(metadata)
                != analyzer_module._directory_identity(output.stat())
            ):
                interrupted = True
                raise KeyboardInterrupt("injected post-commit close")
            real_close(descriptor)

        with patch.object(
            analyzer_module.os,
            "open",
            side_effect=tracked_open,
        ), patch.object(
            analyzer_module.os,
            "close",
            side_effect=interrupted_close,
        ):
            report = analyze_geometric_stability(
                **fixture.call_args(output_dir=output)
            )

        self.assertFalse(interrupted)
        self._assert_descriptors_closed(
            opened,
            fstat=real_fstat,
            close=real_close,
        )
        self.assertEqual(report["status"], "completed")
        self.assertTrue((output / OUTPUT_JSON_NAME).is_file())

    def test_final_guard_staging_name_replacement_cannot_publish(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        complete_calls = 0
        replaced = False

        def guard() -> None:
            nonlocal complete_calls, replaced
            entries = self._staging_entries(output)
            if entries == {OUTPUT_JSON_NAME, OUTPUT_MARKDOWN_NAME}:
                complete_calls += 1
                if complete_calls == 2:
                    staging = next(
                        output.parent.glob(f"{output.name}.incomplete-*")
                    )
                    staging.rename(staging.with_name("displaced-staging"))
                    staging.mkdir()
                    replaced = True

        with self.assertRaisesRegex(AnalysisLimitError, "staging.*changed"):
            analyze_geometric_stability(
                **fixture.call_args(output_dir=output),
                live_guard=guard,
            )

        self.assertTrue(replaced)
        self.assertFalse(output.exists())

    def test_final_guard_extra_overcap_file_cannot_publish(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        complete_calls = 0
        injected = False

        def guard() -> None:
            nonlocal complete_calls, injected
            entries = self._staging_entries(output)
            if entries == {OUTPUT_JSON_NAME, OUTPUT_MARKDOWN_NAME}:
                complete_calls += 1
                if complete_calls == 2:
                    staging = next(
                        output.parent.glob(f"{output.name}.incomplete-*")
                    )
                    (staging / "over-cap.bin").write_bytes(
                        b"x" * (analyzer_module.OUTPUT_CAP_BYTES + 1)
                    )
                    injected = True

        with self.assertRaisesRegex(AnalysisLimitError, "unexpected|10 MB"):
            analyze_geometric_stability(
                **fixture.call_args(output_dir=output),
                live_guard=guard,
            )

        self.assertTrue(injected)
        self.assertFalse(output.exists())

    def test_final_guard_source_mutation_cannot_publish(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        complete_calls = 0
        mutated = False

        def guard() -> None:
            nonlocal complete_calls, mutated
            entries = self._staging_entries(output)
            if entries == {OUTPUT_JSON_NAME, OUTPUT_MARKDOWN_NAME}:
                complete_calls += 1
                if complete_calls == 2:
                    fixture.comparison.write_text("{}\n", encoding="utf-8")
                    mutated = True

        with self.assertRaises(InputIntegrityError):
            analyze_geometric_stability(
                **fixture.call_args(output_dir=output),
                live_guard=guard,
            )

        self.assertTrue(mutated)
        self.assertFalse(output.exists())

    def test_final_closure_rejects_mutated_artifact_type_or_bytes(self):
        mutation_kinds = (
            "small_content",
            "same_length_content",
            "symlink",
            "directory",
        )
        for artifact_name in (OUTPUT_JSON_NAME, OUTPUT_MARKDOWN_NAME):
            for mutation_kind in mutation_kinds:
                with self.subTest(
                    artifact=artifact_name,
                    mutation=mutation_kind,
                ):
                    fixture = self.write_complete_fixture()
                    output = (
                        self.root
                        / f"artifact-{artifact_name}-{mutation_kind}"
                        / "run"
                    )
                    external = (
                        self.root
                        / f"external-{artifact_name}-{mutation_kind}.txt"
                    )
                    external.write_bytes(b"must survive\n")
                    complete_calls = 0
                    mutated = False

                    def guard() -> None:
                        nonlocal complete_calls, mutated
                        entries = self._staging_entries(output)
                        if entries == {
                            OUTPUT_JSON_NAME,
                            OUTPUT_MARKDOWN_NAME,
                        }:
                            complete_calls += 1
                            if complete_calls != 2:
                                return
                            staging = next(
                                output.parent.glob(
                                    f"{output.name}.incomplete-*"
                                )
                            )
                            artifact = staging / artifact_name
                            if mutation_kind == "small_content":
                                artifact.write_bytes(b"forged\n")
                            elif mutation_kind == "same_length_content":
                                original = artifact.read_bytes()
                                replacement = (
                                    b"X" if original[:1] != b"X" else b"Y"
                                )
                                artifact.write_bytes(
                                    replacement + original[1:]
                                )
                            elif mutation_kind == "symlink":
                                artifact.unlink()
                                artifact.symlink_to(external)
                            else:
                                artifact.unlink()
                                artifact.mkdir()
                            mutated = True

                    with self.assertRaisesRegex(
                        AnalysisLimitError,
                        "artifact",
                    ):
                        analyze_geometric_stability(
                            **fixture.call_args(output_dir=output),
                            live_guard=guard,
                        )

                    self.assertTrue(mutated)
                    self.assertFalse(output.exists())
                    self.assertEqual(external.read_bytes(), b"must survive\n")

    def test_inter_artifact_replacement_cannot_publish_forged_json(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        real_read = analyzer_module.os.read
        injected = False

        def replace_json_while_markdown_is_verified(
            descriptor: int,
            size: int,
        ) -> bytes:
            nonlocal injected
            markdown = output / OUTPUT_MARKDOWN_NAME
            if (
                not injected
                and markdown.exists()
                and analyzer_module._directory_identity(
                    analyzer_module.os.fstat(descriptor)
                )
                == analyzer_module._directory_identity(markdown.stat())
            ):
                json_path = output / OUTPUT_JSON_NAME
                original = json_path.read_bytes()
                replacement = b"X" if original[:1] != b"X" else b"Y"
                forged = self.root / "forged-json-replacement"
                forged.write_bytes(replacement + original[1:])
                forged.replace(json_path)
                injected = True
            return real_read(descriptor, size)

        with patch.object(
            analyzer_module.os,
            "read",
            side_effect=replace_json_while_markdown_is_verified,
        ):
            with self.assertRaisesRegex(AnalysisLimitError, "artifact"):
                analyze_geometric_stability(
                    **fixture.call_args(output_dir=output)
                )

        self.assertTrue(injected)
        self.assertFalse(output.exists())

    def test_destination_identity_mismatch_preserves_victim_and_cleans_owned(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        owned_displaced = output.parent / "owned-staging-at-rename"
        real_rename = analyzer_module._rename_no_replace_at
        replaced = False

        def replace_then_rename(
            parent_fd: int,
            source_name: str,
            destination_name: str,
        ) -> None:
            nonlocal replaced
            staging = output.parent / source_name
            staging.rename(owned_displaced)
            staging.mkdir()
            (staging / "victim.txt").write_text(
                "must survive\n",
                encoding="utf-8",
            )
            replaced = True
            real_rename(parent_fd, source_name, destination_name)

        with patch.object(
            analyzer_module,
            "_rename_no_replace_at",
            side_effect=replace_then_rename,
        ):
            with self.assertRaisesRegex(
                AnalysisLimitError,
                "destination.*changed",
            ):
                analyze_geometric_stability(
                    **fixture.call_args(output_dir=output)
                )

        self.assertTrue(replaced)
        self.assertEqual(
            (output / "victim.txt").read_text(encoding="utf-8"),
            "must survive\n",
        )
        self.assertFalse(owned_displaced.exists())
        self.assertEqual(
            list(output.parent.glob(f"{output.name}.incomplete-*")),
            [],
        )

    def test_parent_replacement_aborts_and_cleans_pinned_staging(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        moved_parent = self.root / "moved-analysis"
        attacker_parent = self.root / "attacker-analysis"
        attacker_parent.mkdir()
        replaced = False

        def guard() -> None:
            nonlocal replaced
            entries = self._staging_entries(output)
            if (
                not replaced
                and entries == {OUTPUT_JSON_NAME, OUTPUT_MARKDOWN_NAME}
            ):
                output.parent.rename(moved_parent)
                output.parent.symlink_to(attacker_parent, target_is_directory=True)
                replaced = True

        with self.assertRaisesRegex(AnalysisLimitError, "output parent changed"):
            analyze_geometric_stability(
                **fixture.call_args(output_dir=output),
                live_guard=guard,
            )

        self.assertTrue(replaced)
        self.assertFalse(output.exists())
        self.assertEqual(list(moved_parent.iterdir()), [])

    def test_ancestor_replacement_aborts_before_publication(self):
        fixture = self.write_complete_fixture()
        ancestor = self.root / "publication-root"
        ancestor.mkdir()
        output = ancestor / "analysis" / "run"
        moved_ancestor = self.root / "moved-publication-root"
        attacker_ancestor = self.root / "attacker-publication-root"
        attacker_ancestor.mkdir()
        complete_calls = 0
        replaced = False

        def guard() -> None:
            nonlocal complete_calls, replaced
            entries = self._staging_entries(output)
            if entries == {OUTPUT_JSON_NAME, OUTPUT_MARKDOWN_NAME}:
                complete_calls += 1
                if complete_calls == 2:
                    ancestor.rename(moved_ancestor)
                    ancestor.symlink_to(
                        attacker_ancestor,
                        target_is_directory=True,
                    )
                    replaced = True

        with self.assertRaisesRegex(AnalysisLimitError, "output path changed"):
            analyze_geometric_stability(
                **fixture.call_args(output_dir=output),
                live_guard=guard,
            )

        self.assertTrue(replaced)
        self.assertFalse(output.exists())

    def test_start_gate_uses_pinned_existing_filesystem_descriptor(self):
        output = self.root / "analysis" / "run"
        observed_fds: list[int | None] = []

        def available(parent_fd, fallback_path: Path) -> int:
            del fallback_path
            observed_fds.append(parent_fd)
            return analyzer_module.START_BYTES - 1

        with patch.object(
            analyzer_module,
            "_available_bytes_for_guard",
            side_effect=available,
        ):
            with self.assertRaisesRegex(AnalysisLimitError, "start threshold"):
                analyze_geometric_stability(
                    self.root / "missing-comparison.json",
                    expected_comparison_sha256="0" * 64,
                    expected_parent_manifest_sha256="1" * 64,
                    output_dir=output,
                )

        self.assertTrue(observed_fds)
        self.assertIsInstance(observed_fds[0], int)

    def test_missing_parent_live_probe_uses_pinned_preflight_descriptor(self):
        fixture = self.write_complete_fixture()
        output = self.root / "new-analysis" / "run"
        observed_fds: list[int | None] = []

        def available(parent_fd, fallback_path: Path) -> int:
            del fallback_path
            observed_fds.append(parent_fd)
            return analyzer_module.START_BYTES + 1

        with patch.object(
            analyzer_module,
            "_available_bytes_for_guard",
            side_effect=available,
        ):
            analyze_geometric_stability(
                **fixture.call_args(output_dir=output)
            )

        self.assertGreaterEqual(len(observed_fds), 2)
        self.assertTrue(all(isinstance(item, int) for item in observed_fds))

    def test_preflight_source_failure_closes_every_opened_descriptor(self):
        output = self.root / "analysis" / "run"
        real_open = analyzer_module.os.open
        real_close = analyzer_module.os.close
        real_fstat = analyzer_module.os.fstat
        opened: list[int] = []

        def tracked_open(*args, **kwargs):
            descriptor = real_open(*args, **kwargs)
            opened.append(descriptor)
            return descriptor

        with patch.object(
            analyzer_module.os,
            "open",
            side_effect=tracked_open,
        ):
            with self.assertRaises(InputIntegrityError):
                analyze_geometric_stability(
                    self.root / "missing-comparison.json",
                    expected_comparison_sha256="0" * 64,
                    expected_parent_manifest_sha256="1" * 64,
                    output_dir=output,
                )

        self.assertTrue(opened)
        self._assert_descriptors_closed(
            opened,
            fstat=real_fstat,
            close=real_close,
        )

    def test_path_chain_fstat_failure_closes_new_child_descriptor(self):
        output = self.root / "analysis" / "run"
        real_open = analyzer_module.os.open
        real_close = analyzer_module.os.close
        real_fstat = analyzer_module.os.fstat
        opened: list[int] = []
        injected = False

        def tracked_open(*args, **kwargs):
            descriptor = real_open(*args, **kwargs)
            opened.append(descriptor)
            return descriptor

        def failing_fstat(descriptor: int):
            nonlocal injected
            if (
                not injected
                and len(opened) >= 2
                and descriptor == opened[-1]
            ):
                injected = True
                raise OSError("injected path-chain fstat failure")
            return real_fstat(descriptor)

        with patch.object(
            analyzer_module.os,
            "open",
            side_effect=tracked_open,
        ), patch.object(
            analyzer_module.os,
            "fstat",
            side_effect=failing_fstat,
        ):
            with self.assertRaisesRegex(
                OSError,
                "path-chain fstat failure",
            ):
                analyze_geometric_stability(
                    self.root / "missing-comparison.json",
                    expected_comparison_sha256="0" * 64,
                    expected_parent_manifest_sha256="1" * 64,
                    output_dir=output,
                )

        self.assertTrue(injected)
        self._assert_descriptors_closed(
            opened,
            fstat=real_fstat,
            close=real_close,
        )

    def test_staging_identity_is_pinned_before_name_stat_replacement(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        real_stat = analyzer_module.os.stat
        staging_victim: Path | None = None
        owned_displaced = output.parent / "owned-staging-displaced"
        injected = False

        def replace_before_name_stat(path, *args, **kwargs):
            nonlocal injected, staging_victim
            if (
                not injected
                and isinstance(path, str)
                and path.startswith(f"{output.name}.incomplete-")
                and kwargs.get("dir_fd") is not None
            ):
                staging_victim = output.parent / path
                staging_victim.rename(owned_displaced)
                staging_victim.mkdir()
                (staging_victim / "victim.txt").write_text(
                    "must survive\n",
                    encoding="utf-8",
                )
                injected = True
            return real_stat(path, *args, **kwargs)

        with patch.object(
            analyzer_module.os,
            "stat",
            side_effect=replace_before_name_stat,
        ):
            with self.assertRaisesRegex(
                AnalysisLimitError,
                "staging.*changed|unexpected artifacts",
            ):
                analyze_geometric_stability(
                    **fixture.call_args(output_dir=output)
                )

        self.assertTrue(injected)
        self.assertIsNotNone(staging_victim)
        self.assertEqual(
            (staging_victim / "victim.txt").read_text(encoding="utf-8"),
            "must survive\n",
        )
        self.assertFalse(owned_displaced.exists())
        self.assertFalse(output.exists())

    def test_staging_create_and_pin_bypasses_former_open_callback_gap(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        real_open_directory = (
            analyzer_module._PublicationTransaction._open_directory
        )
        displaced = output.parent / "former-gap-owned"
        callback_reached = False

        def replace_from_former_gap(
            path: Path,
            *,
            dir_fd: int | None = None,
        ) -> int:
            nonlocal callback_reached
            if path.name.startswith(f"{output.name}.incomplete-"):
                callback_reached = True
                staging = output.parent / path.name
                staging.rename(displaced)
                staging.mkdir()
                (staging / "victim.txt").write_text(
                    "must survive\n",
                    encoding="utf-8",
                )
            return real_open_directory(path, dir_fd=dir_fd)

        failure: AnalysisLimitError | None = None
        report: dict | None = None
        with patch.object(
            analyzer_module._PublicationTransaction,
            "_open_directory",
            side_effect=replace_from_former_gap,
        ):
            try:
                report = analyze_geometric_stability(
                    **fixture.call_args(output_dir=output)
                )
            except AnalysisLimitError as error:
                failure = error

        self.assertIsNone(failure)
        self.assertFalse(callback_reached)
        self.assertFalse(displaced.exists())
        self.assertIsNotNone(report)
        self.assertEqual(report["status"], "completed")
        self.assertTrue((output / OUTPUT_JSON_NAME).is_file())

    def test_replacement_after_create_and_pin_preserves_unowned_entry(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        primitive = getattr(
            analyzer_module,
            "_create_and_pin_directory_at",
            None,
        )
        displaced = output.parent / "post-pin-owned"
        victim: Path | None = None
        replaced = False

        def replace_after_pin(
            parent_fd: int,
            candidate: str,
        ) -> tuple[int, tuple[int, int]]:
            nonlocal replaced, victim
            self.assertIsNotNone(primitive)
            descriptor, identity = primitive(parent_fd, candidate)
            victim = output.parent / candidate
            victim.rename(displaced)
            victim.mkdir()
            (victim / "victim.txt").write_text(
                "must survive\n",
                encoding="utf-8",
            )
            replaced = True
            return descriptor, identity

        with patch.object(
            analyzer_module,
            "_create_and_pin_directory_at",
            side_effect=replace_after_pin,
            create=primitive is None,
        ):
            with self.assertRaisesRegex(
                AnalysisLimitError,
                "staging.*changed",
            ):
                analyze_geometric_stability(
                    **fixture.call_args(output_dir=output)
                )

        self.assertTrue(replaced)
        self.assertIsNotNone(victim)
        self.assertEqual(
            (victim / "victim.txt").read_text(encoding="utf-8"),
            "must survive\n",
        )
        self.assertFalse(displaced.exists())
        self.assertFalse(output.exists())

    def test_staging_stat_failure_removes_created_owned_directory(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        real_stat = analyzer_module.os.stat
        injected = False

        def failing_stat(path, *args, **kwargs):
            nonlocal injected
            if (
                not injected
                and isinstance(path, str)
                and path.startswith(f"{output.name}.incomplete-")
                and kwargs.get("dir_fd") is not None
            ):
                injected = True
                raise OSError("injected staging stat failure")
            return real_stat(path, *args, **kwargs)

        with patch.object(
            analyzer_module.os,
            "stat",
            side_effect=failing_stat,
        ):
            with self.assertRaisesRegex(OSError, "staging stat failure"):
                analyze_geometric_stability(
                    **fixture.call_args(output_dir=output)
                )

        self.assertTrue(injected)
        self.assertFalse(output.exists())
        self.assertFalse(output.parent.exists())

    def test_ancestor_symlink_is_rejected_without_target_artifacts(self):
        fixture = self.write_complete_fixture()
        target = self.root / "publication-target"
        target.mkdir()
        ancestor_link = self.root / "publication-link"
        ancestor_link.symlink_to(target, target_is_directory=True)
        output = ancestor_link / "run"

        with self.assertRaisesRegex(AnalysisLimitError, "symlink"):
            analyze_geometric_stability(
                **fixture.call_args(output_dir=output)
            )

        self.assertFalse((target / output.name).exists())
        self.assertEqual(
            list(target.glob(f"{output.name}.incomplete-*")),
            [],
        )

    def test_base_exception_during_entry_guard_cleans_staging_and_fds(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        real_open = analyzer_module.os.open
        real_close = analyzer_module.os.close
        real_fstat = analyzer_module.os.fstat
        opened: list[int] = []
        interrupted = False

        def tracked_open(*args, **kwargs):
            descriptor = real_open(*args, **kwargs)
            opened.append(descriptor)
            return descriptor

        def guard() -> None:
            nonlocal interrupted
            if not interrupted and self._staging_entries(output) == set():
                interrupted = True
                raise KeyboardInterrupt("injected entry interruption")

        with patch.object(
            analyzer_module.os,
            "open",
            side_effect=tracked_open,
        ):
            with self.assertRaisesRegex(
                KeyboardInterrupt,
                "entry interruption",
            ):
                analyze_geometric_stability(
                    **fixture.call_args(output_dir=output),
                    live_guard=guard,
                )

        self.assertTrue(interrupted)
        self._assert_descriptors_closed(
            opened,
            fstat=real_fstat,
            close=real_close,
        )
        self.assertFalse(output.exists())
        self.assertFalse(output.parent.exists())

    def test_repeated_close_base_exceptions_cannot_leak_artifact_fds(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        real_open = analyzer_module.os.open
        real_close = analyzer_module.os.close
        real_fstat = analyzer_module.os.fstat
        real_library = analyzer_module.ctypes.PyDLL(None, use_errno=True)
        real_raw_close = (
            getattr(real_library, "__close_nocancel")
            if analyzer_module.sys.platform == "darwin"
            else real_library.close
        )
        real_raw_close.argtypes = [analyzer_module.ctypes.c_int]
        real_raw_close.restype = analyzer_module.ctypes.c_int
        artifact_fds: list[int] = []
        injected: list[type[BaseException]] = []

        def tracked_open(path, flags, *args, **kwargs):
            descriptor = real_open(path, flags, *args, **kwargs)
            if (
                isinstance(path, str)
                and path in (OUTPUT_JSON_NAME, OUTPUT_MARKDOWN_NAME)
                and flags & analyzer_module.os.O_ACCMODE
                == analyzer_module.os.O_RDONLY
            ):
                artifact_fds.append(descriptor)
            return descriptor

        class InterruptedRawClose:
            argtypes = None
            restype = None

            def __call__(self, descriptor: int) -> int:
                result = real_raw_close(descriptor)
                if descriptor in artifact_fds and len(injected) < 2:
                    exception_type = (
                        KeyboardInterrupt if not injected else SystemExit
                    )
                    injected.append(exception_type)
                    raise exception_type(
                        "injected post-syscall artifact close"
                    )
                return result

        interrupted_raw_close = InterruptedRawClose()

        class LibraryProxy:
            def __getattr__(self, name: str):
                if name in ("__close_nocancel", "close"):
                    return interrupted_raw_close
                return getattr(real_library, name)

        with patch.object(
            analyzer_module.os,
            "open",
            side_effect=tracked_open,
        ), patch.object(
            analyzer_module.ctypes,
            "PyDLL",
            return_value=LibraryProxy(),
        ):
            report = analyze_geometric_stability(
                **fixture.call_args(output_dir=output)
            )

        self.assertEqual(len(artifact_fds), 2)
        self.assertEqual(injected, [KeyboardInterrupt, SystemExit])
        leaked: list[int] = []
        for descriptor in artifact_fds:
            try:
                real_fstat(descriptor)
            except OSError as error:
                self.assertEqual(error.errno, analyzer_module.errno.EBADF)
            else:
                leaked.append(descriptor)
        for descriptor in leaked:
            real_close(descriptor)
        self.assertEqual(leaked, [])
        self.assertEqual(report["status"], "completed")
        self.assertTrue((output / OUTPUT_JSON_NAME).is_file())

    def test_base_exception_at_final_guard_cleans_staging(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        complete_calls = 0

        def guard() -> None:
            nonlocal complete_calls
            if self._staging_entries(output) == {
                OUTPUT_JSON_NAME,
                OUTPUT_MARKDOWN_NAME,
            }:
                complete_calls += 1
                if complete_calls == 2:
                    raise SystemExit("injected final interruption")

        with self.assertRaisesRegex(SystemExit, "final interruption"):
            analyze_geometric_stability(
                **fixture.call_args(output_dir=output),
                live_guard=guard,
            )

        self.assertFalse(output.exists())
        self.assertFalse(output.parent.exists())

    def test_staging_pin_failure_preserves_unknown_created_entry(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"

        with patch.object(
            analyzer_module,
            "_PIN_FSTAT",
            side_effect=OSError("injected staging pin failure"),
        ):
            with self.assertRaisesRegex(OSError, "staging pin failure"):
                analyze_geometric_stability(
                    **fixture.call_args(output_dir=output)
                )

        self.assertFalse(output.exists())
        preserved = list(
            output.parent.glob(f"{output.name}.incomplete-*")
        )
        self.assertEqual(len(preserved), 1)
        self.assertEqual(list(preserved[0].iterdir()), [])

    def test_write_phase_failure_cleans_staging(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"

        def guard() -> None:
            if self._staging_entries(output) == {OUTPUT_JSON_NAME}:
                raise AnalysisLimitError("injected write-phase failure")

        with self.assertRaisesRegex(AnalysisLimitError, "write-phase"):
            analyze_geometric_stability(
                **fixture.call_args(output_dir=output),
                live_guard=guard,
            )

        self.assertFalse(output.exists())
        self.assertFalse(output.parent.exists())

    def test_direct_write_failure_cleans_staging(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        with patch.object(
            analyzer_module,
            "_write_all",
            side_effect=OSError("injected write failure"),
        ):
            with self.assertRaisesRegex(OSError, "injected write failure"):
                analyze_geometric_stability(
                    **fixture.call_args(output_dir=output)
                )
        self.assertFalse(output.exists())
        self.assertFalse(output.parent.exists())

    def test_direct_rename_failure_cleans_staging(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        with patch.object(
            analyzer_module,
            "_rename_no_replace_at",
            side_effect=OSError("injected rename failure"),
        ):
            with self.assertRaisesRegex(OSError, "injected rename failure"):
                analyze_geometric_stability(
                    **fixture.call_args(output_dir=output)
                )
        self.assertFalse(output.exists())
        self.assertFalse(output.parent.exists())

    def test_transient_cleanup_failure_is_retried_without_masking_original(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        real_clear = analyzer_module._clear_directory_fd
        cleanup_attempts = 0

        def flaky_clear(directory_fd: int) -> None:
            nonlocal cleanup_attempts
            cleanup_attempts += 1
            if cleanup_attempts == 1:
                raise OSError("transient cleanup interference")
            real_clear(directory_fd)

        def guard() -> None:
            if self._staging_entries(output) == {OUTPUT_JSON_NAME}:
                raise AnalysisLimitError("original publication failure")

        with patch.object(
            analyzer_module,
            "_clear_directory_fd",
            side_effect=flaky_clear,
        ):
            with self.assertRaisesRegex(
                AnalysisLimitError,
                "original publication failure",
            ):
                analyze_geometric_stability(
                    **fixture.call_args(output_dir=output),
                    live_guard=guard,
                )

        self.assertEqual(cleanup_attempts, 2)
        self.assertFalse(output.exists())
        self.assertFalse(output.parent.exists())

    def test_cleanup_removes_unexpected_entries_without_masking_failure(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        external_target = self.root / "must-survive.txt"
        external_target.write_text("survives\n", encoding="utf-8")
        injected = False

        def guard() -> None:
            nonlocal injected
            entries = self._staging_entries(output)
            if (
                not injected
                and entries == {OUTPUT_JSON_NAME, OUTPUT_MARKDOWN_NAME}
            ):
                staging = next(
                    output.parent.glob(f"{output.name}.incomplete-*")
                )
                (staging / "unexpected-link").symlink_to(external_target)
                nested = staging / "unexpected-dir"
                nested.mkdir()
                (nested / "nested-link").symlink_to(external_target)
                injected = True
                raise AnalysisLimitError("original publication failure")

        with self.assertRaisesRegex(AnalysisLimitError, "original publication"):
            analyze_geometric_stability(
                **fixture.call_args(output_dir=output),
                live_guard=guard,
            )

        self.assertTrue(injected)
        self.assertEqual(external_target.read_text(encoding="utf-8"), "survives\n")
        self.assertFalse(output.exists())
        self.assertFalse(output.parent.exists())

    def test_ten_megabyte_cap_fails_without_publication(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        with patch.object(analyzer_module, "OUTPUT_CAP_BYTES", 1):
            with self.assertRaisesRegex(AnalysisLimitError, "10 MB"):
                analyze_geometric_stability(
                    **fixture.call_args(output_dir=output)
                )
        self.assertFalse(output.exists())

    def test_source_drift_after_rendering_aborts_publication(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        mutated = False

        def guard() -> None:
            nonlocal mutated
            if (
                not mutated
                and self._staging_entries(output)
                == {OUTPUT_JSON_NAME, OUTPUT_MARKDOWN_NAME}
            ):
                fixture.comparison.write_text("{}\n", encoding="utf-8")
                mutated = True

        with self.assertRaises(InputIntegrityError):
            analyze_geometric_stability(
                **fixture.call_args(output_dir=output),
                live_guard=guard,
            )
        self.assertTrue(mutated)
        self.assertFalse(output.exists())

    def test_no_replace_collision_preserves_racer_output(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        raced = False

        def guard() -> None:
            nonlocal raced
            if (
                not raced
                and self._staging_entries(output)
                == {OUTPUT_JSON_NAME, OUTPUT_MARKDOWN_NAME}
            ):
                output.mkdir()
                (output / "racer.txt").write_text("racer\n", encoding="utf-8")
                raced = True

        with self.assertRaisesRegex(AnalysisLimitError, "already exists"):
            analyze_geometric_stability(
                **fixture.call_args(output_dir=output),
                live_guard=guard,
            )

        self.assertTrue(raced)
        self.assertEqual(
            (output / "racer.txt").read_text(encoding="utf-8"),
            "racer\n",
        )
        self.assertEqual(
            list(output.parent.glob(f"{output.name}.incomplete-*")),
            [],
        )

    def test_source_commit_drift_aborts_before_publication(self):
        fixture = self.write_complete_fixture()
        output = self.root / "analysis" / "run"
        commits = ["1" * 40, "2" * 40]
        with patch.object(
            analyzer_module,
            "_source_commit",
            side_effect=commits,
        ):
            with self.assertRaisesRegex(InputIntegrityError, "source commit"):
                analyze_geometric_stability(
                    **fixture.call_args(output_dir=output)
                )
        self.assertFalse(output.exists())

    def test_live_guard_runs_once_at_exact_ten_thousand_pair_boundary(self):
        trajectory = self._trajectory(robot_count=1, frame_count=5_000)
        trajectory["config"]["execute"]["time-step"] = 0.05
        trajectory["config"]["bases"][1] = [3.0, 10.0]
        fixture = self.write_complete_fixture(trajectory=trajectory)
        output = self.root / "analysis" / "run"
        processed = 0
        boundary_calls = 0
        original_record = analyzer_module._ScientificAccumulator.record

        def record(accumulator, row):
            nonlocal processed
            result = original_record(accumulator, row)
            processed += 1
            return result

        def guard() -> None:
            nonlocal boundary_calls
            if processed == 10_000:
                boundary_calls += 1

        with patch.object(
            analyzer_module._ScientificAccumulator,
            "record",
            new=record,
        ):
            analyze_geometric_stability(
                **fixture.call_args(),
                live_guard=guard,
            )

        self.assertEqual(processed, 10_000)
        self.assertEqual(boundary_calls, 1)

    def test_builtin_live_floor_stops_at_ten_thousand_pair_boundary(self):
        trajectory = self._trajectory(robot_count=1, frame_count=5_000)
        trajectory["config"]["execute"]["time-step"] = 0.05
        trajectory["config"]["bases"][1] = [3.0, 10.0]
        fixture = self.write_complete_fixture(trajectory=trajectory)
        output = self.root / "analysis" / "run"
        processed = 0
        original_record = analyzer_module._ScientificAccumulator.record
        real_available = analyzer_module.available_bytes

        def record(accumulator, row):
            nonlocal processed
            result = original_record(accumulator, row)
            processed += 1
            return result

        def available(parent_fd, fallback_path: Path) -> int:
            del parent_fd, fallback_path
            if processed == 10_000:
                return analyzer_module.HARD_FLOOR_BYTES - 1
            return analyzer_module.START_BYTES + 1

        with patch.object(
            analyzer_module._ScientificAccumulator,
            "record",
            new=record,
        ), patch.object(
            analyzer_module,
            "_available_bytes_for_guard",
            side_effect=available,
            create=True,
        ):
            with self.assertRaisesRegex(AnalysisLimitError, "live threshold"):
                analyze_geometric_stability(
                    **fixture.call_args(output_dir=output)
                )

        self.assertEqual(processed, 10_000)
        self.assertFalse(output.exists())


class RawFilesystemPrimitiveTests(unittest.TestCase):
    class FakeCall:
        def __init__(self, result: int = 0) -> None:
            self.result = result
            self.calls: list[tuple] = []
            self.argtypes = None
            self.restype = None

        def __call__(self, *arguments):
            self.calls.append(arguments)
            return self.result

    def test_create_and_pin_defines_mkdirat_and_openat_signatures(self):
        primitive = getattr(
            analyzer_module,
            "_create_and_pin_directory_at",
            None,
        )
        self.assertIsNotNone(primitive)
        mkdirat = self.FakeCall()
        openat = self.FakeCall(result=73)
        library = type(
            "Library",
            (),
            {"mkdirat": mkdirat, "openat": openat},
        )()
        metadata = type(
            "Metadata",
            (),
            {
                "st_mode": stat.S_IFDIR | 0o700,
                "st_dev": 12,
                "st_ino": 34,
            },
        )()

        with patch.object(
            analyzer_module.sys,
            "platform",
            "linux",
        ), patch.object(
            analyzer_module.ctypes,
            "PyDLL",
            return_value=library,
        ), patch.object(
            analyzer_module,
            "_PIN_FSTAT",
            return_value=metadata,
            create=True,
        ):
            descriptor, identity = primitive(51, "stage-name")

        self.assertEqual(mkdirat.argtypes, [
            analyzer_module.ctypes.c_int,
            analyzer_module.ctypes.c_char_p,
            analyzer_module.ctypes.c_uint,
        ])
        self.assertIs(mkdirat.restype, analyzer_module.ctypes.c_int)
        self.assertEqual(openat.argtypes, [
            analyzer_module.ctypes.c_int,
            analyzer_module.ctypes.c_char_p,
            analyzer_module.ctypes.c_int,
        ])
        self.assertIs(openat.restype, analyzer_module.ctypes.c_int)
        self.assertEqual(mkdirat.calls, [(51, b"stage-name", 0o700)])
        self.assertEqual(
            openat.calls,
            [(
                51,
                b"stage-name",
                analyzer_module.os.O_RDONLY
                | analyzer_module.os.O_DIRECTORY
                | analyzer_module.os.O_NOFOLLOW
                | analyzer_module.os.O_CLOEXEC,
            )],
        )
        self.assertEqual((descriptor, identity), (73, (12, 34)))

    def test_create_and_pin_translates_mkdirat_and_openat_errno(self):
        primitive = getattr(
            analyzer_module,
            "_create_and_pin_directory_at",
            None,
        )
        self.assertIsNotNone(primitive)
        cases = (
            (
                self.FakeCall(result=-1),
                self.FakeCall(),
                analyzer_module.errno.EEXIST,
                FileExistsError,
            ),
            (
                self.FakeCall(),
                self.FakeCall(result=-1),
                analyzer_module.errno.ELOOP,
                OSError,
            ),
        )
        for mkdirat, openat, error_number, exception_type in cases:
            with self.subTest(error_number=error_number):
                library = type(
                    "Library",
                    (),
                    {"mkdirat": mkdirat, "openat": openat},
                )()
                with patch.object(
                    analyzer_module.sys,
                    "platform",
                    "linux",
                ), patch.object(
                    analyzer_module.ctypes,
                    "PyDLL",
                    return_value=library,
                ), patch.object(
                    analyzer_module.ctypes,
                    "get_errno",
                    return_value=error_number,
                ):
                    with self.assertRaises(exception_type) as caught:
                        primitive(52, "stage-name")
                self.assertEqual(caught.exception.errno, error_number)

    def test_raw_close_defines_signature_and_never_retries_ambiguous_error(self):
        close_no_fail = getattr(
            analyzer_module,
            "_close_fd_no_fail",
            None,
        )
        self.assertIsNotNone(close_no_fail)
        for error_number in (
            analyzer_module.errno.EINTR,
            analyzer_module.errno.EBADF,
        ):
            with self.subTest(error_number=error_number):
                close = self.FakeCall(result=-1)
                library = type("Library", (), {"close": close})()
                with patch.object(
                    analyzer_module.sys,
                    "platform",
                    "linux",
                ), patch.object(
                    analyzer_module.ctypes,
                    "PyDLL",
                    return_value=library,
                ), patch.object(
                    analyzer_module.ctypes,
                    "get_errno",
                    return_value=error_number,
                ):
                    close_no_fail(74)

                self.assertEqual(
                    close.argtypes,
                    [analyzer_module.ctypes.c_int],
                )
                self.assertIs(close.restype, analyzer_module.ctypes.c_int)
                self.assertEqual(close.calls, [(74,)])

    def test_darwin_raw_close_falls_back_when_nocancel_symbol_is_absent(self):
        close = self.FakeCall()
        library = type("Library", (), {"close": close})()

        with patch.object(
            analyzer_module.sys,
            "platform",
            "darwin",
        ), patch.object(
            analyzer_module.ctypes,
            "PyDLL",
            return_value=library,
        ):
            analyzer_module._close_fd_no_fail(75)

        self.assertEqual(close.calls, [(75,)])


class RenameNoReplaceTests(unittest.TestCase):
    class FakeRename:
        def __init__(self, result: int = 0) -> None:
            self.result = result
            self.calls: list[tuple] = []
            self.argtypes = None
            self.restype = None

        def __call__(self, *arguments):
            self.calls.append(arguments)
            return self.result

    def test_darwin_uses_descriptor_relative_rename_excl(self):
        rename = self.FakeRename()
        library = type("Library", (), {"renameatx_np": rename})()

        with patch.object(analyzer_module.sys, "platform", "darwin"), patch.object(
            analyzer_module.ctypes,
            "CDLL",
            return_value=library,
        ):
            analyzer_module._rename_no_replace_at(41, "stage", "final")

        self.assertEqual(
            rename.calls,
            [(41, b"stage", 41, b"final", 0x00000004)],
        )

    def test_linux_uses_descriptor_relative_rename_noreplace(self):
        rename = self.FakeRename()
        library = type("Library", (), {"renameat2": rename})()

        with patch.object(analyzer_module.sys, "platform", "linux"), patch.object(
            analyzer_module.ctypes,
            "CDLL",
            return_value=library,
        ):
            analyzer_module._rename_no_replace_at(42, "stage", "final")

        self.assertEqual(
            rename.calls,
            [(42, b"stage", 42, b"final", 1)],
        )

    def test_missing_platform_symbol_fails_closed(self):
        for platform in ("darwin", "linux"):
            with self.subTest(platform=platform), patch.object(
                analyzer_module.sys,
                "platform",
                platform,
            ), patch.object(
                analyzer_module.ctypes,
                "CDLL",
                return_value=object(),
            ):
                with self.assertRaises(AnalysisLimitError):
                    analyzer_module._rename_no_replace_at(
                        43,
                        "stage",
                        "final",
                    )

    def test_errno_is_translated_without_losing_collision_semantics(self):
        for error_number, expected_exception in (
            (analyzer_module.errno.EEXIST, AnalysisLimitError),
            (analyzer_module.errno.ENOTEMPTY, AnalysisLimitError),
            (analyzer_module.errno.EPERM, OSError),
        ):
            with self.subTest(error_number=error_number):
                rename = self.FakeRename(result=-1)
                library = type("Library", (), {"renameatx_np": rename})()
                with patch.object(
                    analyzer_module.sys,
                    "platform",
                    "darwin",
                ), patch.object(
                    analyzer_module.ctypes,
                    "CDLL",
                    return_value=library,
                ), patch.object(
                    analyzer_module.ctypes,
                    "get_errno",
                    return_value=error_number,
                ):
                    with self.assertRaises(expected_exception) as caught:
                        analyzer_module._rename_no_replace_at(
                            44,
                            "stage",
                            "final",
                        )
                if error_number == analyzer_module.errno.EPERM:
                    self.assertEqual(caught.exception.errno, error_number)

    def test_unsupported_platform_fails_closed(self):
        with patch.object(
            analyzer_module.sys,
            "platform",
            "win32",
        ), patch.object(
            analyzer_module.ctypes,
            "CDLL",
            return_value=object(),
        ):
            with self.assertRaisesRegex(AnalysisLimitError, "unsupported"):
                analyzer_module._rename_no_replace_at(
                    45,
                    "stage",
                    "final",
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


class NominalPerturbationPrimitiveTests(unittest.TestCase):
    def test_three_four_five_target_triangle_has_one_third_supremum(self):
        result = nominal_fixed_pair_metrics(
            [0.0, 0.0],
            [[3.0, 0.0], [0.0, 4.0]],
        )

        self.assertAlmostEqual(result["included_angle_rad"], math.pi / 2.0)
        self.assertAlmostEqual(result["observer_reference_1_length"], 3.0)
        self.assertAlmostEqual(result["observer_reference_2_length"], 4.0)
        self.assertAlmostEqual(result["reference_pair_length"], 5.0)
        self.assertAlmostEqual(result["minimum_triangle_slack"], 2.0)
        self.assertAlmostEqual(
            result["admissible_tracking_deviation_supremum"],
            1.0 / 3.0,
        )

    def test_equality_at_supremum_is_not_strictly_certified(self):
        result = fixed_pair_tracking_margin(
            observer_estimate=[1.0 / 3.0, 0.0],
            reference_estimates=[[3.0, 0.0], [0.0, 4.0]],
            observer_target=[0.0, 0.0],
            reference_targets=[[3.0, 0.0], [0.0, 4.0]],
        )

        self.assertAlmostEqual(
            result["maximum_vertex_target_deviation"],
            1.0 / 3.0,
        )
        self.assertAlmostEqual(result["tracking_deviation_margin"], 0.0)
        self.assertFalse(
            result["strictly_within_admissible_tracking_deviation"]
        )

    def test_collinear_nominal_triangle_fails_closed(self):
        with self.assertRaisesRegex(
            InputIntegrityError,
            "strictly nondegenerate",
        ):
            nominal_fixed_pair_metrics(
                [0.0, 0.0],
                [[1.0, 0.0], [2.0, 0.0]],
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
