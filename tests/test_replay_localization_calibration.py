"""Stage-0 contracts for the offline localization calibration core."""

import copy
import unittest

import numpy as np

from scripts.diagnostics.replay_localization_calibration import (
    active_references,
    fixed_references,
    fim_radius,
    solve_later_frame,
    solve_wnls,
    stable_measurement_seed,
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

    def test_active_uav_references_are_strictly_lower_index_and_same_squad(self):
        """Breaks if a cyclic, cross-squad, or higher-index UAV edge is admitted."""
        references = active_references(self.config, 10, self.positions)

        self.assertEqual(references["uav_ids"], [8, 9])
        self.assertTrue(all(8 <= robot_id < 10 for robot_id in references["uav_ids"]))


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

    def test_later_frame_failure_holds_previous_finite_result_as_stale(self):
        """Breaks if a later failure discards usable upstream DAG state."""
        previous = {
            "status": "converged",
            "estimate": [1.0, 1.0],
            "covariance": [[0.25, 0.0], [0.0, 0.25]],
            "epsilon": 1.5,
        }
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


if __name__ == "__main__":
    unittest.main()
