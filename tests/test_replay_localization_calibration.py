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


if __name__ == "__main__":
    unittest.main()
