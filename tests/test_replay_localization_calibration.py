"""Stage-0 contracts for the offline localization calibration core."""

import copy
import gzip
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

import scripts.diagnostics.replay_localization_calibration as replay_module
from scripts.diagnostics.replay_localization_calibration import (
    active_references,
    fixed_references,
    fim_radius,
    solve_later_frame,
    solve_wnls,
    stable_measurement_seed,
    replay_calibration,
)


def production_like_config():
    """Return a compact 14-UAV version of the production formation settings."""
    return {
        "num": 14,
        "formation": {"parts": 2, "bases-id": [[1, 0], [1, 2]]},
        "bases": [[-20.0, 0.0], [0.0, 0.0], [20.0, 0.0]],
        "cbfs": {
            "without-slack": {
                "comm-fixed": {
                    "max-range": 10.0,
                    "min-neighbour-id-offset": -2,
                    "max-neighbour-id-offset": 0,
                }
            }
        },
    }


class StableMeasurementSeedTests(unittest.TestCase):
    def test_seed_is_deterministic_and_changes_for_every_edge_key_component(self):
        """Breaks if any edge-key component is omitted from the stable seed."""
        key = (20260727, 5, 14, "uav", 12)
        seed = stable_measurement_seed(*key)

        self.assertEqual(seed, stable_measurement_seed(*key))
        self.assertEqual(
            len(
                {
                    seed,
                    stable_measurement_seed(20260728, 5, 14, "uav", 12),
                    stable_measurement_seed(20260727, 6, 14, "uav", 12),
                    stable_measurement_seed(20260727, 5, 13, "uav", 12),
                    stable_measurement_seed(20260727, 5, 14, "base", 12),
                    stable_measurement_seed(20260727, 5, 14, "uav", 11),
                }
            ),
            6,
        )

    def test_common_edge_noise_is_bitwise_paired_across_graph_cases(self):
        """Breaks if graph case or iteration order affects an edge's noise."""
        seed = stable_measurement_seed(20260727, 5, 14, "uav", 12)
        fixed_noise = np.random.default_rng(seed).normal(0.0, 0.5)
        dynamic_noise = np.random.default_rng(seed).normal(0.0, 0.5)

        self.assertEqual(
            np.asarray(fixed_noise).tobytes(), np.asarray(dynamic_noise).tobytes()
        )


class LocalizationGraphTests(unittest.TestCase):
    def setUp(self):
        self.config = production_like_config()
        self.positions = {
            robot_id: np.asarray([float(robot_id), 0.0])
            for robot_id in range(1, 15)
        }

    def test_fixed_references_match_assigned_bases_and_previous_two_squad_uavs(self):
        """Breaks if fixed production references use the wrong part or offset rule."""
        self.assertEqual(
            fixed_references(self.config, 1), {"base_ids": [0, 1], "uav_ids": []}
        )
        self.assertEqual(
            fixed_references(self.config, 3), {"base_ids": [], "uav_ids": [1, 2]}
        )
        self.assertEqual(
            fixed_references(self.config, 10), {"base_ids": [], "uav_ids": [8, 9]}
        )

    def test_active_references_are_fixed_union_all_visible_eligible_references(self):
        """Breaks if a visible base or lower-index same-squad UAV is omitted."""
        positions = copy.deepcopy(self.positions)
        positions[7] = np.asarray([0.0, 0.0])
        positions[1] = np.asarray([1.0, 0.0])
        positions[2] = np.asarray([2.0, 0.0])
        positions[3] = np.asarray([30.0, 0.0])
        positions[4] = np.asarray([4.0, 0.0])
        positions[5] = np.asarray([5.0, 0.0])
        positions[6] = np.asarray([6.0, 0.0])

        self.assertEqual(
            active_references(self.config, 7, positions),
            {"base_ids": [1], "uav_ids": [1, 2, 4, 5, 6]},
        )

    def test_assigned_references_persist_beyond_range_but_unassigned_references_do_not(self):
        """Breaks if fixed CBF references are accidentally range-gated."""
        config = production_like_config()
        config["cbfs"]["without-slack"]["comm-fixed"]["max-range"] = 1.0
        positions = copy.deepcopy(self.positions)
        positions[2] = np.asarray([100.0, 0.0])
        positions[1] = np.asarray([200.0, 0.0])
        config["bases"] = [[1000.0, 0.0], [2000.0, 0.0], [3000.0, 0.0]]

        self.assertEqual(
            active_references(config, 2, positions),
            {"base_ids": [1], "uav_ids": [1]},
        )

        positions[7] = np.asarray([0.0, 0.0])
        positions[1] = np.asarray([100.0, 0.0])
        positions[5] = np.asarray([200.0, 0.0])
        positions[6] = np.asarray([300.0, 0.0])
        self.assertEqual(
            active_references(config, 7, positions),
            {"base_ids": [], "uav_ids": [5, 6]},
        )

        positions[1] = np.asarray([0.5, 0.0])
        self.assertEqual(active_references(config, 7, positions)["uav_ids"], [1, 5, 6])

    def test_every_observer_has_only_lower_index_same_squad_uav_references(self):
        """Breaks if any of the 14 DAG nodes admits a cyclic or cross-squad edge."""
        squad_size = 7
        for observer_id in range(1, 15):
            observer_part = (observer_id - 1) // squad_size
            observer_local = (observer_id - 1) % squad_size
            references = active_references(
                self.config, observer_id, self.positions
            )
            for reference_id in references["uav_ids"]:
                with self.subTest(
                    observer_id=observer_id, reference_id=reference_id
                ):
                    self.assertEqual(
                        (reference_id - 1) // squad_size, observer_part
                    )
                    self.assertLess(
                        (reference_id - 1) % squad_size, observer_local
                    )


class WnlsAndFimTests(unittest.TestCase):
    def setUp(self):
        self.references = np.asarray([[0.0, 0.0], [2.0, 0.0]])
        self.reference_covariances = np.zeros((2, 2, 2))
        self.measurements = np.asarray([np.sqrt(2.0), np.sqrt(2.0)])

    def test_zero_noise_two_reference_solution_recovers_the_positive_branch(self):
        """Breaks if the WNLS residual or update has the wrong sign."""
        result = solve_wnls(
            self.references,
            self.reference_covariances,
            self.measurements,
            np.asarray([1.0, 0.8]),
            0.5,
        )

        self.assertEqual(result["status"], "converged")
        np.testing.assert_allclose(result["estimate"], [1.0, 1.0], atol=1e-8)

    def test_initial_estimate_selects_the_continuous_range_mirror_branch(self):
        """Breaks if WNLS jumps across the two-range mirror ambiguity."""
        positive = solve_wnls(
            self.references,
            self.reference_covariances,
            self.measurements,
            np.asarray([1.0, 0.8]),
            0.5,
        )
        negative = solve_wnls(
            self.references,
            self.reference_covariances,
            self.measurements,
            np.asarray([1.0, -0.8]),
            0.5,
        )

        self.assertGreater(positive["estimate"][1], 0.0)
        self.assertLess(negative["estimate"][1], 0.0)

    def test_repeatedly_rejected_steps_do_not_report_false_convergence(self):
        """Breaks if damping-induced tiny rejected steps satisfy convergence."""
        references = np.asarray(
            [[2.8, 1.4], [-2.5, -1.8], [-5.2, -9.2]], dtype=float
        )
        covariances = np.zeros((3, 2, 2), dtype=float)
        covariances[:, 0, 0] = [17.5, 11.0, 15.1]
        covariances[:, 1, 1] = [9.4, 6.5, 0.6]

        result = solve_wnls(
            references,
            covariances,
            np.asarray([7.5, 0.7, 2.5]),
            np.asarray([9.3, 3.2]),
            0.5,
        )

        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["iterations"], 50)
        self.assertIsNone(result["covariance"])
        self.assertIsNone(result["epsilon"])

    def test_accepted_tiny_damped_step_requires_stationarity(self):
        """Breaks if an accepted high-damping step bypasses the gradient gate."""
        references = np.asarray(
            [
                [-0.12056416482955547, 2.7187170092356103],
                [5.872335644685272, -0.477801917195226],
                [-0.14520206798959212, 0.5954405813630369],
            ]
        )
        covariances = np.asarray(
            [
                [
                    [7.220460239885509, 8.011804237242888],
                    [8.011804237242888, 12.373768616266027],
                ],
                [
                    [0.29551573062296704, -1.4138111914715052],
                    [-1.4138111914715052, 6.844642038867566],
                ],
                [
                    [12.463450157301342, 0.377288367010776],
                    [0.377288367010776, 2.189387646352776],
                ],
            ]
        )
        measurements = np.asarray(
            [4.612572322975238, 11.18865210126454, 4.947081336786854]
        )

        result = solve_wnls(
            references,
            covariances,
            measurements,
            np.asarray([5.144692260836605, -1.1997999821356011]),
            0.5,
        )

        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["iterations"], 50)
        self.assertIsNone(result["covariance"])
        self.assertIsNone(result["epsilon"])

    def test_third_non_collinear_reference_is_accepted_without_api_change(self):
        """Breaks if the estimator assumes exactly two measurements."""
        references = np.vstack((self.references, np.asarray([[0.0, 2.0]])))
        measurements = np.linalg.norm(references - np.asarray([1.0, 1.0]), axis=1)
        result = solve_wnls(
            references,
            np.zeros((3, 2, 2)),
            measurements,
            np.asarray([1.0, 0.8]),
            0.5,
        )

        self.assertEqual(result["status"], "converged")
        np.testing.assert_allclose(result["estimate"], [1.0, 1.0], atol=1e-8)

    def test_fim_covariance_and_radius_match_the_hand_computation(self):
        """Breaks if reference covariance, FIM inversion, or epsilon scaling changes."""
        estimate = np.asarray([1.0, 1.0])
        directions = (estimate - self.references) / np.sqrt(2.0)
        expected_phi = directions.T @ (4.0 * directions)
        expected_covariance = np.linalg.inv(expected_phi)
        expected_epsilon = 3.0 * np.sqrt(np.linalg.eigvalsh(expected_covariance).max())

        result = fim_radius(
            estimate, self.references, self.reference_covariances, 0.5
        )

        self.assertEqual(result["status"], "converged")
        np.testing.assert_allclose(result["covariance"], expected_covariance, atol=1e-12)
        self.assertAlmostEqual(result["epsilon"], expected_epsilon, places=12)
        self.assertAlmostEqual(result["phi_min_eigenvalue"], 4.0, places=12)
        self.assertAlmostEqual(result["phi_condition"], 1.0, places=12)

    def test_fim_weights_match_nonzero_propagated_covariance_hand_case(self):
        """Breaks if projected reference covariance is omitted from range weights."""
        references = np.asarray([[0.0, 1.0], [1.0, 0.0]])
        covariances = np.asarray(
            [
                [[0.75, 0.0], [0.0, 9.0]],
                [[9.0, 0.0], [0.0, 1.75]],
            ]
        )

        result = fim_radius(
            np.asarray([1.0, 1.0]), references, covariances, 0.5
        )

        self.assertEqual(result["status"], "converged")
        np.testing.assert_allclose(
            result["covariance"], [[1.0, 0.0], [0.0, 2.0]], atol=1e-12
        )
        self.assertAlmostEqual(result["epsilon"], 3.0 * np.sqrt(2.0), places=12)
        self.assertAlmostEqual(result["phi_min_eigenvalue"], 0.5, places=12)
        self.assertAlmostEqual(result["phi_condition"], 2.0, places=12)

    def test_relative_spectral_threshold_accepts_above_and_rejects_below(self):
        """Breaks if the 1e-12 relative FIM condition gate drifts."""
        references = np.asarray([[-1.0, 0.0], [0.0, -1.0]])
        above = np.zeros((2, 2, 2))
        above[1, 1, 1] = 1.0 / (8.0e-12) - 0.25
        below = np.zeros((2, 2, 2))
        below[1, 1, 1] = 1.0 / (2.0e-12) - 0.25

        accepted = fim_radius(
            np.asarray([0.0, 0.0]), references, above, 0.5
        )
        rejected = fim_radius(
            np.asarray([0.0, 0.0]), references, below, 0.5
        )

        self.assertEqual(accepted["status"], "converged")
        self.assertAlmostEqual(
            accepted["phi_min_eigenvalue"], 8.0e-12, delta=1.0e-22
        )
        self.assertEqual(rejected["status"], "invalid")
        self.assertIsNone(rejected["covariance"])

    def test_collinear_fim_is_invalid_without_fabricated_covariance(self):
        """Breaks if a singular final FIM is regularized or reported as valid."""
        result = solve_wnls(
            self.references,
            self.reference_covariances,
            np.asarray([1.0, 1.0]),
            np.asarray([1.0, 0.0]),
            0.5,
        )

        self.assertEqual(result["status"], "invalid")
        self.assertIsNone(result["covariance"])
        self.assertIsNone(result["epsilon"])

    def test_non_finite_measurement_is_invalid_without_fabricated_covariance(self):
        """Breaks if invalid range data reaches the linear solver."""
        result = solve_wnls(
            self.references,
            self.reference_covariances,
            np.asarray([np.nan, np.sqrt(2.0)]),
            np.asarray([1.0, 0.8]),
            0.5,
        )

        self.assertEqual(result["status"], "invalid")
        self.assertIsNone(result["estimate"])
        self.assertIsNone(result["covariance"])

    def test_malformed_numeric_inputs_return_invalid_instead_of_raising(self):
        """Breaks if numeric conversion or malformed shapes escape the status API."""
        cases = {
            "nonnumeric reference": (
                [["bad", 0.0], [2.0, 0.0]],
                self.reference_covariances,
                self.measurements,
                [1.0, 0.8],
                0.5,
            ),
            "malformed reference shape": (
                [0.0, 2.0],
                self.reference_covariances,
                self.measurements,
                [1.0, 0.8],
                0.5,
            ),
            "nonnumeric covariance": (
                self.references,
                [[["bad", 0.0], [0.0, 0.0]], [[0.0, 0.0], [0.0, 0.0]]],
                self.measurements,
                [1.0, 0.8],
                0.5,
            ),
            "malformed covariance shape": (
                self.references,
                np.zeros((2, 2)),
                self.measurements,
                [1.0, 0.8],
                0.5,
            ),
            "nonnumeric measurements": (
                self.references,
                self.reference_covariances,
                ["bad", 1.0],
                [1.0, 0.8],
                0.5,
            ),
            "nonnumeric initial estimate": (
                self.references,
                self.reference_covariances,
                self.measurements,
                ["bad", 0.8],
                0.5,
            ),
            "nonnumeric sigma": (
                self.references,
                self.reference_covariances,
                self.measurements,
                [1.0, 0.8],
                "bad",
            ),
            "malformed sigma shape": (
                self.references,
                self.reference_covariances,
                self.measurements,
                [1.0, 0.8],
                np.asarray([0.5]),
            ),
            "nonfinite reference": (
                [[np.inf, 0.0], [2.0, 0.0]],
                self.reference_covariances,
                self.measurements,
                [1.0, 0.8],
                0.5,
            ),
            "nonfinite covariance": (
                self.references,
                np.asarray(
                    [[[np.nan, 0.0], [0.0, 0.0]], [[0.0, 0.0], [0.0, 0.0]]]
                ),
                self.measurements,
                [1.0, 0.8],
                0.5,
            ),
            "nonfinite initial estimate": (
                self.references,
                self.reference_covariances,
                self.measurements,
                [1.0, np.inf],
                0.5,
            ),
            "nonfinite sigma": (
                self.references,
                self.reference_covariances,
                self.measurements,
                [1.0, 0.8],
                np.nan,
            ),
        }

        for label, arguments in cases.items():
            with self.subTest(label):
                result = solve_wnls(*arguments)
                self.assertEqual(result["status"], "invalid")
                self.assertIsNone(result["covariance"])

    def test_fim_malformed_numeric_input_returns_invalid_instead_of_raising(self):
        """Breaks if the shared FIM validator leaks numeric conversion errors."""
        result = fim_radius(
            ["bad", 1.0],
            self.references,
            self.reference_covariances,
            0.5,
        )

        self.assertEqual(result["status"], "invalid")
        self.assertIsNone(result["covariance"])

    def test_asymmetric_or_non_psd_reference_covariance_is_invalid(self):
        """Breaks if propagated covariance assumptions are not enforced."""
        asymmetric = np.zeros((2, 2, 2))
        asymmetric[0] = [[1.0, 0.5], [0.0, 1.0]]
        non_psd = np.zeros((2, 2, 2))
        non_psd[0] = [[-1.0, 0.0], [0.0, 1.0]]

        for label, covariances in (
            ("asymmetric", asymmetric),
            ("non-PSD", non_psd),
        ):
            with self.subTest(label):
                result = solve_wnls(
                    self.references,
                    covariances,
                    self.measurements,
                    np.asarray([1.0, 0.8]),
                    0.5,
                )
                self.assertEqual(result["status"], "invalid")
                self.assertIsNone(result["covariance"])

    def test_later_frame_failure_holds_previous_finite_result_as_stale(self):
        """Breaks if a later failure discards usable upstream DAG state."""
        previous = {
            "status": "converged",
            "estimate": [1.0, 1.0],
            "covariance": [[0.25, 0.0], [0.0, 0.25]],
            "epsilon": 1.5,
            "phi_min_eigenvalue": 4.0,
            "phi_condition": 1.0,
            "iterations": 3,
            "cost": 0.0,
            "failure_reason": None,
        }
        for status in ("converged", "stale"):
            with self.subTest(status=status):
                previous["status"] = status
                result = solve_later_frame(
                    previous,
                    self.references,
                    self.reference_covariances,
                    np.asarray([np.nan, np.sqrt(2.0)]),
                    np.asarray([1.0, 1.0]),
                    0.5,
                )

                self.assertEqual(result["status"], "stale")
                self.assertEqual(result["estimate"], previous["estimate"])
                self.assertEqual(result["covariance"], previous["covariance"])
                self.assertEqual(result["failure"]["status"], "invalid")

    def test_malformed_or_missing_prior_does_not_become_stale(self):
        """Breaks if stale fallback accepts an invalid prior calibration state."""
        valid_prior = {
            "status": "converged",
            "estimate": [1.0, 1.0],
            "covariance": [[0.25, 0.0], [0.0, 0.25]],
            "epsilon": 1.5,
            "phi_min_eigenvalue": 4.0,
            "phi_condition": 1.0,
            "iterations": 3,
            "cost": 0.0,
            "failure_reason": None,
        }
        malformed_priors = {
            "no prior": None,
            "invalid status": {**valid_prior, "status": "invalid"},
            "wrong estimate shape": {**valid_prior, "estimate": [1.0]},
            "nonnumeric covariance": {**valid_prior, "covariance": [["bad"]]},
            "non-SPD covariance": {
                **valid_prior,
                "covariance": [[1.0, 0.0], [0.0, 0.0]],
            },
            "asymmetric covariance": {
                **valid_prior,
                "covariance": [[1.0, 0.5], [0.0, 1.0]],
            },
            "nonfinite epsilon": {**valid_prior, "epsilon": np.nan},
            "invalid FIM minimum": {
                **valid_prior,
                "phi_min_eigenvalue": 0.0,
            },
            "invalid FIM condition": {
                **valid_prior,
                "phi_condition": np.inf,
            },
        }

        for label, previous in malformed_priors.items():
            with self.subTest(label):
                result = solve_later_frame(
                    previous,
                    self.references,
                    self.reference_covariances,
                    np.asarray([np.nan, np.sqrt(2.0)]),
                    np.asarray([1.0, 1.0]),
                    0.5,
                )
                self.assertEqual(result["status"], "invalid")
                self.assertNotIn("failure", result)


class ReplayEvidenceBundleTests(unittest.TestCase):
    def _fixture(self, directory: Path) -> tuple[Path, Path]:
        """Create a three-frame two-squad truth replay with non-collinear bases."""
        config = {
            "num": 4,
            "formation": {"parts": 2, "bases-id": [[0, 1], [1, 2]]},
            "bases": [[0.0, 0.0], [4.0, 0.0], [0.0, 4.0]],
            "initial": {"position": {"positions": [[1.0, 1.0], [2.0, 1.0], [1.0, 2.0], [2.0, 2.0]]}},
            "execute": {"random-seed": 9},
            "position_covariance": {"ranging_sigma": 0.5},
            "cbfs": {"without-slack": {"comm-fixed": {
                "max-range": 20.0,
                "min-neighbour-id-offset": -2,
                "max-neighbour-id-offset": 0,
            }}},
        }
        frames = []
        for frame_index in range(3):
            frames.append({"robots": [
                {"id": robot_id, "state": {"x": x + 0.1 * frame_index, "y": y}}
                for robot_id, (x, y) in enumerate(config["initial"]["position"]["positions"], 1)
            ]})
        data_path = directory / "data.json"
        manifest_path = directory / "input-manifest.json"
        data_path.write_text(json.dumps({"config": config, "state": frames}))
        manifest_path.write_text(json.dumps({"base_commit": "fixture-commit", "config_sha256": "fixture-config"}))
        return data_path, manifest_path

    def test_replay_writes_complete_deterministic_paired_evidence_bundle(self):
        """Breaks if replay drops rows, changes paired noise, or copies truth evidence."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            project_root = root / "project"
            output_root = root / "output"
            project_root.mkdir()
            data_path, manifest_path = self._fixture(root)

            first = replay_calibration(
                data_path, manifest_path, output_root, [17], project_root
            )
            second = replay_calibration(
                data_path, manifest_path, output_root, [17], project_root
            )

            self.assertEqual(first["termination_reason"], "completed")
            self.assertEqual(first["input_data"]["path"], str(data_path.resolve()))
            self.assertEqual(
                first["input_data"]["sha256"],
                hashlib.sha256(data_path.read_bytes()).hexdigest(),
            )
            self.assertFalse((Path(first["output_dir"]) / "data.json").exists())
            self.assertFalse((Path(first["output_dir"]) / "source-snapshot.tar.gz").exists())
            self.assertEqual(first["settings"]["run_seeds"], [17])
            self.assertEqual(first["settings"]["ranging_sigma"], 0.5)
            self.assertEqual(first["settings"]["graph_cases"], ["dynamic_dag_wnls", "fixed_refs_wnls"])
            self.assertEqual(first["settings"]["max_frames"], None)
            self.assertIn("implementation", first["settings"])
            self.assertIn("lm_fim", first["settings"])
            with gzip.open(Path(first["output_dir"]) / "calibration.jsonl.gz", "rt") as source:
                first_rows = [json.loads(line) for line in source]
            with gzip.open(Path(second["output_dir"]) / "calibration.jsonl.gz", "rt") as source:
                second_rows = [json.loads(line) for line in source]
            self.assertEqual(first_rows, second_rows)
            self.assertEqual(len(first_rows), 1 * 2 * 4 * 3)
            self.assertTrue(all("active_references" in row for row in first_rows))
            self.assertTrue(all("measurements" in row for row in first_rows))
            self.assertTrue(all("transition" in row for row in first_rows))
            self.assertTrue(all("attempt_status" in row for row in first_rows))
            self.assertTrue(
                all(
                    isinstance(row["containment"], bool)
                    for row in first_rows
                    if row["primary_statistics"]
                )
            )
            self.assertTrue(all(not row["primary_statistics"] for row in first_rows if row["frame_index"] == 0))
            by_edge = {}
            for row in first_rows:
                for measurement in row["measurements"]:
                    key = (row["seed"], row["frame_index"], row["robot_id"], measurement["kind"], measurement["id"])
                    by_edge.setdefault(key, set()).add(measurement["noisy_range"])
            self.assertTrue(all(len(values) == 1 for values in by_edge.values()))
            self.assertEqual(
                json.loads((Path(first["output_dir"]) / "summary.json").read_text()),
                json.loads((Path(second["output_dir"]) / "summary.json").read_text()),
            )
            decompressed = gzip.decompress(
                (Path(first["output_dir"]) / "calibration.jsonl.gz").read_bytes()
            )
            self.assertEqual(
                first["decompressed_process_sha256"],
                hashlib.sha256(decompressed).hexdigest(),
            )
            self.assertEqual(
                first["compressed_process_sha256"],
                hashlib.sha256(
                    (Path(first["output_dir"]) / "calibration.jsonl.gz").read_bytes()
                ).hexdigest(),
            )
            self.assertEqual(
                first["decompressed_process_sha256"],
                second["decompressed_process_sha256"],
            )
            summary = json.loads((Path(first["output_dir"]) / "summary.json").read_text())
            self.assertEqual(summary["settings"], first["settings"])
            self.assertEqual(
                summary["graph_cases"]["dynamic_dag_wnls"]["initialization_frame"]["robot_frame_count"],
                4,
            )

    def test_attempt_failures_reduce_containment_and_no_worse_rates(self):
        """Breaks if stale retained state hides current failures or inflates coverage."""
        samples = replay_module._SummarySamples(6, [17])
        template = {
            "seed": 17,
            "graph_case": "dynamic_dag_wnls",
            "squad_local_index": 1,
            "primary_statistics": True,
            "status": "stale",
            "attempt_status": "invalid",
            "containment": False,
            "error_to_epsilon_ratio": None,
            "phi_min_eigenvalue": None,
            "phi_condition": None,
            "epsilon": None,
            "transition": {"changed": False, "epsilon_change": None},
        }
        samples.add({**template, "graph_case": "dynamic_dag_wnls", "status": "converged", "attempt_status": "converged", "containment": True})
        samples.add(template)
        samples.add({**template, "attempt_status": "failed"})
        for attempt_status in ("converged", "invalid", "converged"):
            samples.add({**template, "graph_case": "fixed_refs_wnls", "status": attempt_status, "attempt_status": attempt_status, "containment": attempt_status == "converged"})

        summary = replay_module._summary(samples, 6, {"run_seeds": [17]})
        dynamic = summary["graph_cases"]["dynamic_dag_wnls"]["overall"]

        self.assertEqual(dynamic["containment_denominator"], 3)
        self.assertEqual(dynamic["containment_rate"], 1 / 3)
        self.assertEqual(dynamic["attempt_status_counts"]["invalid"], 1)
        self.assertEqual(dynamic["attempt_status_counts"]["failed"], 1)
        self.assertFalse(summary["adequacy"]["aggregate_containment_at_least_0_98"])
        self.assertFalse(summary["adequacy"]["dynamic_failure_rate_no_worse"])
        self.assertEqual(
            summary["graph_cases"]["dynamic_dag_wnls"]["containment_bootstrap_95"]["lower"],
            1 / 3,
        )

    def test_reference_input_records_all_edges_before_upstream_validation(self):
        """Breaks if one missing estimate truncates later physical edge evidence."""
        with tempfile.TemporaryDirectory() as temporary:
            data_path, _ = self._fixture(Path(temporary))
            config = json.loads(data_path.read_text())["config"]
            truth = {index: np.asarray(position) for index, position in enumerate(config["initial"]["position"]["positions"], 1)}
            references = {"base_ids": [0, 1], "uav_ids": [1, 2]}

            *_, records, error = replay_module._reference_inputs(
                references, config, truth, {}, 17, 1, 3, 0.5
            )

        self.assertEqual(error, "invalid upstream UAV reference")
        self.assertEqual([(record["kind"], record["id"]) for record in records], [("base", 0), ("base", 1), ("uav", 1), ("uav", 2)])
        self.assertTrue(all(record["true_range"] is not None for record in records))
        self.assertTrue(all(record["noisy_range"] is not None for record in records))

    def test_compact_summary_storage_scales_to_registered_row_count(self):
        """Breaks if the 280k-row replay retains nested process dictionaries."""
        samples = replay_module._SummarySamples(280_000, list(range(20)))

        self.assertLess(samples.nbytes, 32_000_000)
        self.assertFalse(hasattr(samples, "rows"))

    def test_duplicate_seeds_and_non_preregistered_sigma_are_rejected(self):
        """Breaks if summary keys overwrite duplicate seeds or input sigma silently drifts."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            project_root = root / "project"
            project_root.mkdir()
            data_path, manifest_path = self._fixture(root)
            with self.assertRaisesRegex(ValueError, "duplicate"):
                replay_calibration(data_path, manifest_path, root / "duplicate", [17, 17], project_root)
            data = json.loads(data_path.read_text())
            data["config"]["position_covariance"]["ranging_sigma"] = 0.6
            data_path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, "preregistered"):
                replay_calibration(data_path, manifest_path, root / "sigma", [17], project_root)

    def test_process_control_exceptions_propagate_instead_of_becoming_setup_error(self):
        """Breaks if process-control interrupts are caught as ordinary replay failures."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            project_root = root / "project"
            project_root.mkdir()
            data_path, manifest_path = self._fixture(root)
            for exception in (KeyboardInterrupt(), SystemExit(3)):
                with self.subTest(exception=type(exception).__name__), patch.object(
                    replay_module,
                    "_truth_positions",
                    side_effect=exception,
                ), self.assertRaises(type(exception)):
                    replay_calibration(
                        data_path,
                        manifest_path,
                        root / type(exception).__name__,
                        [17],
                        project_root,
                    )

    def test_live_disk_floor_writes_terminal_manifest(self):
        """Breaks if a post-frame disk guard exits without an inspectable manifest."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            project_root = root / "project"
            output_root = root / "output"
            project_root.mkdir()
            data_path, manifest_path = self._fixture(root)
            with patch(
                "scripts.diagnostics.replay_localization_calibration.available_bytes",
                side_effect=[
                    9_000_000_000,
                    7_000_000_000,
                    5_999_999_999,
                    7_000_000_000,
                ],
            ):
                result = replay_calibration(
                    data_path, manifest_path, output_root, [17], project_root
                )

            self.assertEqual(result["termination_reason"], "disk_hard_floor")
            self.assertTrue((Path(result["output_dir"]) / "manifest.json").exists())

    def test_cache_caps_write_terminal_manifests(self):
        """Breaks if either retained-evidence cap ends replay without a manifest."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            project_root = root / "project"
            project_root.mkdir()
            data_path, manifest_path = self._fixture(root)
            for expected, allocation in (
                ("cache_root_cap", lambda _: 2_000_000_001),
                ("cache_run_cap", lambda path: 250_000_001 if path.name.count("_") else 0),
            ):
                with self.subTest(expected=expected), patch(
                    "scripts.diagnostics.replay_localization_calibration.allocated_bytes",
                    side_effect=allocation,
                ):
                    result = replay_calibration(
                        data_path, manifest_path, root / expected, [17], project_root
                    )
                self.assertEqual(result["termination_reason"], expected)
                self.assertTrue((Path(result["output_dir"]) / "manifest.json").exists())

    def test_preexisting_root_cap_stops_before_process_rows(self):
        """Breaks if an already oversized evidence root begins replay processing."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            project_root = root / "project"
            output_root = root / "output"
            project_root.mkdir()
            output_root.mkdir()
            data_path, manifest_path = self._fixture(root)

            def allocation(path):
                return 2_000_000_001 if Path(path) == output_root else 0

            with patch.object(replay_module, "allocated_bytes", side_effect=allocation):
                result = replay_calibration(data_path, manifest_path, output_root, [17], project_root)

            self.assertEqual(result["termination_reason"], "cache_root_cap")
            self.assertFalse((Path(result["output_dir"]) / "calibration.jsonl.gz").exists())
            self.assertTrue((Path(result["output_dir"]) / "manifest.json").exists())

    def test_summary_induced_cap_cannot_leave_completed_manifest(self):
        """Breaks if retained summary output can cross the cap after the last frame."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            project_root = root / "project"
            output_root = root / "output"
            project_root.mkdir()
            data_path, manifest_path = self._fixture(root)
            run_probes = 0

            def allocation(path):
                nonlocal run_probes
                if Path(path).name.count("_"):
                    run_probes += 1
                    return 250_000_001 if run_probes >= 5 else 0
                return 0

            with patch.object(replay_module, "allocated_bytes", side_effect=allocation):
                result = replay_calibration(data_path, manifest_path, output_root, [17], project_root)

            self.assertEqual(result["termination_reason"], "cache_run_cap")
            self.assertTrue((Path(result["output_dir"]) / "summary.json").exists())
            stored = json.loads((Path(result["output_dir"]) / "manifest.json").read_text())
            self.assertEqual(stored["termination_reason"], "cache_run_cap")


if __name__ == "__main__":
    unittest.main()
