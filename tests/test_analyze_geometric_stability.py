"""Tests for the geometric-stability diagnostic primitives."""

from __future__ import annotations

import hashlib
import json
import math
import tempfile
import unittest
from pathlib import Path

from scripts.diagnostics.analyze_geometric_stability import (
    InputIntegrityError,
    fixed_pair_metrics,
    geometry_metrics,
    load_trajectory,
)


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
