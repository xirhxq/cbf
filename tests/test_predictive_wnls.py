import math
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from scripts.diagnostics.predictive_wnls import (
    algebraic_multilateration_candidate,
    best_conditioned_pair,
    candidate_acceptance,
    finalize_attempt,
    initial_candidates,
    make_unavailable_output,
    merge_base_anchor_provenance,
    normalized_innovation,
    output_is_fresh,
    propagate_estimator_prior,
    qualify_active_references,
    reference_is_eligible,
    scale_aware_stationary,
    select_candidate_result,
    solve_finite_budget_wnls,
    solve_predictive_multistart,
    two_circle_candidates,
)


def make_test_output(
    status="fresh",
    estimate=(10.0, 20.0),
    covariance=((1.0, 0.0), (0.0, 4.0)),
    prediction_age=0,
    provenance=(0, 1),
):
    return {
        "output_status": status,
        "estimate": list(estimate),
        "modeled_covariance": [list(row) for row in covariance],
        "epsilon": 6.0 if status == "fresh" else None,
        "prediction_age": prediction_age,
        "aged_modeled_radius": None if status == "fresh" else 6.0,
        "base_anchor_provenance": list(provenance),
    }


def make_converged_candidate_result(
    *,
    estimate=(1.0, 1.0),
    covariance=((1.0, 0.0), (0.0, 1.0)),
    cost=0.0,
    proposal_trace=(),
):
    trace = [dict(row) for row in proposal_trace]
    return {
        "status": "converged",
        "estimate": list(estimate),
        "covariance": [list(row) for row in covariance],
        "epsilon": 3.0,
        "phi_min_eigenvalue": 1.0,
        "phi_condition": 1.0,
        "fim_valid": True,
        "proposal_count": len(trace),
        "iterations": len(trace),
        "cost": cost,
        "stationarity_norm": 0.0,
        "failure_reason": None,
        "proposal_trace": trace,
    }


class PredictiveLifecycleTests(unittest.TestCase):
    def test_command_prediction_ages_covariance_and_public_status(self):
        bundle = propagate_estimator_prior(
            make_test_output(),
            None,
            [4.0, -2.0],
        )
        prediction = bundle["public_prediction"]
        self.assertEqual(prediction["output_status"], "predicted")
        self.assertEqual(prediction["prediction_age"], 1)
        np.testing.assert_allclose(prediction["estimate"], [12.0, 19.0])
        np.testing.assert_allclose(
            prediction["modeled_covariance"],
            [[1.25, 0.0], [0.0, 4.25]],
        )
        self.assertIsNone(prediction["epsilon"])

    def test_age_three_is_publicly_unavailable_but_keeps_private_seed(self):
        age_two = make_test_output(
            status="predicted",
            prediction_age=2,
            provenance=(),
        )
        bundle = propagate_estimator_prior(age_two, None, [2.0, 0.0])
        public = bundle["public_prediction"]
        private = bundle["private_reacquisition_seed"]
        self.assertEqual(public["output_status"], "unavailable")
        self.assertIsNone(public["estimate"])
        self.assertEqual(public["base_anchor_provenance"], [])
        np.testing.assert_allclose(private["estimate"], [11.0, 20.0])

    def test_unavailable_public_state_can_continue_private_seed_only(self):
        bundle = propagate_estimator_prior(
            make_unavailable_output("expired"),
            {
                "estimate": [5.0, 6.0],
                "modeled_covariance": [[2.0, 0.0], [0.0, 2.0]],
            },
            [2.0, 4.0],
        )
        self.assertEqual(
            bundle["public_prediction"]["output_status"],
            "unavailable",
        )
        np.testing.assert_allclose(
            bundle["private_reacquisition_seed"]["estimate"],
            [6.0, 8.0],
        )

    def test_reference_eligibility_requires_fresh_finite_two_base_roots(self):
        self.assertTrue(reference_is_eligible(make_test_output()))
        self.assertFalse(
            reference_is_eligible(make_test_output(provenance=(0, 0)))
        )
        self.assertFalse(
            reference_is_eligible(
                make_test_output(status="predicted", provenance=())
            )
        )

    def test_invalid_velocity_fails_closed(self):
        for velocity in ([math.nan, 0.0], [1.0], "bad"):
            with self.subTest(velocity=velocity):
                with self.assertRaises(ValueError):
                    propagate_estimator_prior(
                        make_test_output(),
                        None,
                        velocity,
                    )

    def test_malformed_fresh_prior_fails_closed_without_private_seed(self):
        malformed = make_test_output()
        malformed["epsilon"] = None

        bundle = propagate_estimator_prior(malformed, None, [1.0, 0.0])

        self.assertEqual(
            bundle["public_prediction"]["output_status"],
            "unavailable",
        )
        self.assertIsNone(bundle["private_reacquisition_seed"])

    def test_noncanonical_predicted_prior_fails_closed(self):
        malformed = make_test_output(
            status="predicted",
            prediction_age=1,
            provenance=(0, 1),
        )
        malformed["epsilon"] = 7.0

        bundle = propagate_estimator_prior(malformed, None, [1.0, 0.0])

        self.assertEqual(
            bundle["public_prediction"]["output_status"],
            "unavailable",
        )
        self.assertIsNone(bundle["private_reacquisition_seed"])

    def test_noninteger_public_age_fails_closed(self):
        for status, age, provenance in (
            ("fresh", 0.0, (0, 1)),
            ("predicted", 1.0, ()),
        ):
            with self.subTest(status=status):
                malformed = make_test_output(
                    status=status,
                    prediction_age=age,
                    provenance=provenance,
                )

                bundle = propagate_estimator_prior(
                    malformed,
                    None,
                    [1.0, 0.0],
                )

                self.assertEqual(
                    bundle["public_prediction"]["output_status"],
                    "unavailable",
                )
                self.assertIsNone(bundle["private_reacquisition_seed"])


class FinalizeAttemptTests(unittest.TestCase):
    def setUp(self):
        self.prediction = make_test_output(
            status="predicted",
            prediction_age=1,
            provenance=(),
        )
        self.prior_bundle = {
            "public_prediction": self.prediction,
            "private_reacquisition_seed": {
                "estimate": [10.0, 20.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 4.0]],
            },
        }

    def test_accepted_candidate_with_two_valid_roots_publishes_fresh(self):
        result = finalize_attempt(
            {
                "attempt_status": "accepted",
                "candidate": make_test_output(provenance=(0, 2)),
            },
            self.prior_bundle,
            frame_index=4,
        )

        self.assertEqual(result.get("attempt_status"), "accepted")
        self.assertEqual(result["output_status"], "fresh")
        self.assertEqual(result["prediction_age"], 0)
        self.assertEqual(result["base_anchor_provenance"], [0, 2])
        self.assertTrue(output_is_fresh(result))

    def test_accepted_candidate_requires_distinct_nonnegative_integer_roots(self):
        for provenance in ((0,), (0, 0), (-1, 0), (False, 1), (0, "1")):
            with self.subTest(provenance=provenance):
                result = finalize_attempt(
                    {
                        "attempt_status": "accepted",
                        "candidate": make_test_output(provenance=provenance),
                    },
                    self.prior_bundle,
                    frame_index=4,
                )

                self.assertEqual(result.get("attempt_status"), "invalid")
                self.assertEqual(result["output_status"], "predicted")


class ReferenceQualificationTests(unittest.TestCase):
    def test_malformed_mandatory_specifications_are_invalid(self):
        cases = (
            None,
            {},
            {"base_ids": [0]},
            {"base_ids": [0], "uav_ids": [], "other": []},
            {"base_ids": "bad", "uav_ids": []},
            {"base_ids": [0, "bad"], "uav_ids": []},
            {"base_ids": [False], "uav_ids": []},
            {"base_ids": [-1], "uav_ids": []},
        )
        for mandatory in cases:
            with self.subTest(mandatory=mandatory):
                result = qualify_active_references(
                    mandatory=mandatory,
                    optional_keys=[],
                    measurement_records={
                        ("base", 0): {
                            "present": True,
                            "noisy_range": 10.0,
                        },
                    },
                    uav_outputs={},
                    variant="predictive_multistart",
                )
                self.assertEqual(result["status"], "invalid")

    def test_malformed_optional_keys_and_unknown_variant_are_invalid(self):
        for optional_keys, variant in (
            ("bad", "predictive_multistart"),
            ([("base", False)], "predictive_multistart"),
            ([("truth", 0)], "predictive_multistart"),
            ([], "unknown"),
        ):
            with self.subTest(optional_keys=optional_keys, variant=variant):
                result = qualify_active_references(
                    mandatory={"base_ids": [], "uav_ids": []},
                    optional_keys=optional_keys,
                    measurement_records={},
                    uav_outputs={},
                    variant=variant,
                )
                self.assertEqual(result["status"], "invalid")

    def test_malformed_mandatory_scalar_records_are_unavailable(self):
        for noisy_range in (True, "10.0", math.nan, math.inf, -1.0):
            with self.subTest(noisy_range=noisy_range):
                result = qualify_active_references(
                    mandatory={"base_ids": [0], "uav_ids": []},
                    optional_keys=[],
                    measurement_records={
                        ("base", 0): {
                            "present": True,
                            "noisy_range": noisy_range,
                        },
                    },
                    uav_outputs={},
                    variant="predictive_multistart",
                )
                self.assertEqual(result["status"], "reference_unavailable")
                self.assertEqual(result["missing_mandatory"], [("base", 0)])

    def test_malformed_mandatory_uav_output_cannot_qualify(self):
        malformed_outputs = (
            None,
            {},
            {"output_status": "fresh"},
            make_test_output(provenance=(0, 0)),
        )
        for malformed in malformed_outputs:
            with self.subTest(malformed=malformed):
                result = qualify_active_references(
                    mandatory={"base_ids": [], "uav_ids": [1]},
                    optional_keys=[],
                    measurement_records={
                        ("uav", 1): {
                            "present": True,
                            "noisy_range": 10.0,
                        },
                    },
                    uav_outputs={1: malformed},
                    variant="predictive_multistart",
                )
                self.assertEqual(result["status"], "reference_unavailable")
                self.assertEqual(result["missing_mandatory"], [("uav", 1)])

    def test_missing_mandatory_measurement_is_reference_unavailable(self):
        result = qualify_active_references(
            mandatory={"base_ids": [0], "uav_ids": [1]},
            optional_keys=[("uav", 2)],
            measurement_records={
                ("base", 0): {"present": True, "noisy_range": 10.0},
                ("uav", 2): {"present": True, "noisy_range": 12.0},
            },
            uav_outputs={
                1: make_test_output(),
                2: make_test_output(),
            },
            variant="predictive_multistart",
        )
        self.assertEqual(result["status"], "reference_unavailable")
        self.assertEqual(result["missing_mandatory"], [("uav", 1)])

    def test_qualification_filters_predicted_optional_without_truth(self):
        result = qualify_active_references(
            mandatory={"base_ids": [0], "uav_ids": [1]},
            optional_keys=[("uav", 2), ("uav", 3)],
            measurement_records={
                ("base", 0): {"present": True, "noisy_range": 10.0},
                ("uav", 1): {"present": True, "noisy_range": 11.0},
                ("uav", 2): {"present": True, "noisy_range": 12.0},
                ("uav", 3): {"present": True, "noisy_range": 13.0},
            },
            uav_outputs={
                1: make_test_output(),
                2: make_test_output(status="predicted", provenance=()),
                3: make_test_output(provenance=(0, 2)),
            },
            variant="predictive_multistart",
        )
        self.assertEqual(
            result["active_keys"],
            [("base", 0), ("uav", 1), ("uav", 3)],
        )
        self.assertEqual(
            result["excluded"],
            [{"key": ("uav", 2), "reason": "not_current_frame_fresh"}],
        )

    def test_prediction_expiry_ablation_records_stale_anchor_violation(self):
        result = qualify_active_references(
            mandatory={"base_ids": [0], "uav_ids": [1]},
            optional_keys=[("uav", 2)],
            measurement_records={
                ("base", 0): {"present": True, "noisy_range": 10.0},
                ("uav", 1): {"present": True, "noisy_range": 11.0},
                ("uav", 2): {"present": True, "noisy_range": 12.0},
            },
            uav_outputs={
                1: make_test_output(),
                2: make_test_output(status="predicted", provenance=()),
            },
            variant="prediction_expiry",
        )
        self.assertIn(("uav", 2), result["active_keys"])
        self.assertEqual(
            result["violations"],
            [{"key": ("uav", 2), "reason": "stale_or_predicted_anchor_used"}],
        )

    def test_merge_provenance_uses_direct_bases_and_fresh_uav_roots(self):
        self.assertEqual(
            merge_base_anchor_provenance([3, 1], {7: make_test_output(provenance=(0, 2))}),
            (0, 1, 2, 3),
        )


class CandidateGeometryTests(unittest.TestCase):
    def test_cosine_feasibility_uses_exact_one_e_minus_twelve_allowance(self):
        inside_cosine = -1.0 - 0.5e-12
        outside_cosine = -1.0 - 2.0e-12
        inside_baseline = math.sqrt(2.0 - 2.0 * inside_cosine)
        outside_baseline = math.sqrt(2.0 - 2.0 * outside_cosine)

        inside = best_conditioned_pair(
            None,
            [[0.0, 0.0], [inside_baseline, 0.0]],
            [1.0, 1.0],
            [("base", 0), ("base", 1)],
        )
        outside = best_conditioned_pair(
            None,
            [[0.0, 0.0], [outside_baseline, 0.0]],
            [1.0, 1.0],
            [("base", 0), ("base", 1)],
        )

        self.assertEqual(inside, (0, 1))
        self.assertIsNone(outside)

    def test_tied_pair_is_selected_and_returned_in_lexical_key_order(self):
        height = math.sqrt(3.0)
        pair = best_conditioned_pair(
            None,
            [[0.0, 0.0], [2.0, 0.0], [1.0, height]],
            [2.0, 2.0, 2.0],
            [("base", 1), ("base", 0), ("base", 2)],
        )
        self.assertEqual(pair, (1, 0))

    def test_two_circle_branches_have_fixed_orientation_order(self):
        candidates = two_circle_candidates([0, 0], 5, [6, 0], 5)
        np.testing.assert_allclose(candidates[0], [3, -4])
        np.testing.assert_allclose(candidates[1], [3, 4])

    def test_reacquisition_pair_uses_cosine_law_without_observer_center(self):
        pair = best_conditioned_pair(
            None,
            [[0.0, 0.0], [6.0, 0.0], [0.0, 8.0]],
            [5.0, 5.0, 5.0],
            [("base", 0), ("base", 1), ("base", 2)],
        )
        self.assertEqual(pair, (0, 1))

    def test_initial_candidates_are_unique_ordered_and_capped_at_four(self):
        candidates = initial_candidates(
            live_prediction={
                "estimate": [3.0, 3.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
            },
            private_seed=None,
            reference_positions=[[0.0, 0.0], [6.0, 0.0], [0.0, 8.0]],
            measured_ranges=[5.0, 5.0, 6.0],
            reference_keys=[("base", 0), ("base", 1), ("base", 2)],
        )
        self.assertEqual(len(candidates), 4)
        self.assertEqual(
            [candidate["source"] for candidate in candidates],
            ["prediction", "algebraic", "circle_negative", "circle_positive"],
        )

    def test_prediction_wins_deduplication_with_exact_geometry(self):
        candidates = initial_candidates(
            live_prediction={
                "estimate": [3.0, 4.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
            },
            private_seed=None,
            reference_positions=[[0.0, 0.0], [6.0, 0.0], [0.0, 8.0]],
            measured_ranges=[5.0, 5.0, 5.0],
            reference_keys=[("base", 0), ("base", 1), ("base", 2)],
        )
        self.assertEqual(
            [candidate["source"] for candidate in candidates],
            ["prediction", "circle_negative"],
        )

    def test_tangential_circle_branches_are_deduplicated(self):
        candidates = initial_candidates(
            live_prediction=None,
            private_seed={
                "estimate": [10.0, 10.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
            },
            reference_positions=[[0.0, 0.0], [2.0, 0.0]],
            measured_ranges=[1.0, 1.0],
            reference_keys=[("base", 0), ("base", 1)],
        )
        self.assertEqual(
            [candidate["source"] for candidate in candidates],
            ["private_reacquisition_seed", "circle_negative"],
        )

    def test_exact_dedup_threshold_is_inclusive_and_candidate_cap_is_four(self):
        common = {
            "private_seed": None,
            "reference_positions": [
                [0.0, 0.0],
                [3.0, 0.0],
                [0.0, 4.0],
            ],
            "measured_ranges": [0.0, 3.0, 4.0],
            "reference_keys": [
                ("base", 0),
                ("base", 1),
                ("base", 2),
            ],
        }
        at_threshold = initial_candidates(
            live_prediction={
                "estimate": [1e-9, 0.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
            },
            **common,
        )
        above_threshold = initial_candidates(
            live_prediction={
                "estimate": [math.nextafter(1e-9, math.inf), 0.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
            },
            **common,
        )

        self.assertNotIn(
            "algebraic",
            [candidate["source"] for candidate in at_threshold],
        )
        self.assertIn(
            "algebraic",
            [candidate["source"] for candidate in above_threshold],
        )
        self.assertLessEqual(len(above_threshold), 4)


class FinalizeAttemptRemainderTests(unittest.TestCase):
    def setUp(self):
        self.prediction = make_test_output(
            status="predicted",
            prediction_age=1,
            provenance=(),
        )
        self.prior_bundle = {
            "public_prediction": self.prediction,
            "private_reacquisition_seed": {
                "estimate": [10.0, 20.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 4.0]],
            },
        }

    def test_nonaccepted_attempt_canonicalizes_finite_prediction(self):
        noncanonical = dict(self.prediction)
        noncanonical["epsilon"] = 99.0
        noncanonical["aged_modeled_radius"] = -1.0
        noncanonical["base_anchor_provenance"] = [0, 1]

        result = finalize_attempt(
            {"attempt_status": "rejected"},
            {"public_prediction": noncanonical},
            frame_index=4,
        )

        self.assertEqual(result.get("attempt_status"), "rejected")
        self.assertEqual(result["output_status"], "predicted")
        self.assertIsNone(result["epsilon"])
        self.assertEqual(result["base_anchor_provenance"], [])
        self.assertEqual(result["aged_modeled_radius"], 6.0)

    def test_nonaccepted_attempt_without_prediction_publishes_unavailable(self):
        result = finalize_attempt(
            {"attempt_status": "reference_unavailable"},
            {"public_prediction": make_unavailable_output("expired")},
            frame_index=4,
        )

        self.assertEqual(result.get("attempt_status"), "reference_unavailable")
        self.assertEqual(result["output_status"], "unavailable")
        self.assertIsNone(result["estimate"])

    def test_malformed_prediction_fails_closed(self):
        malformed = dict(self.prediction)
        malformed["modeled_covariance"] = [[1.0, 2.0], [0.0, 1.0]]

        result = finalize_attempt(
            {"attempt_status": "failed"},
            {"public_prediction": malformed},
            frame_index=4,
        )

        self.assertEqual(result.get("attempt_status"), "failed")
        self.assertEqual(result["output_status"], "unavailable")

    def test_unknown_attempt_status_is_reported_as_invalid(self):
        for status in ("mystery", None, ["failed"], np.asarray(["failed"])):
            with self.subTest(status=status):
                result = finalize_attempt(
                    {"attempt_status": status},
                    self.prior_bundle,
                    frame_index=4,
                )

                self.assertEqual(result.get("attempt_status"), "invalid")
                self.assertEqual(result["output_status"], "predicted")


class FiniteBudgetWnlsTests(unittest.TestCase):
    def test_scale_aware_stationarity_accepts_expected_solution(self):
        self.assertTrue(
            scale_aware_stationary(
                [1.5e-6, 0.0],
                [1.0],
            )
        )
        self.assertFalse(
            scale_aware_stationary(
                [2.1e-6, 0.0],
                [1.0],
            )
        )

    def test_scale_aware_stationarity_rejects_malformed_or_empty_vectors(self):
        malformed = (
            ([], [1.0]),
            ([0.0], [1.0]),
            ([0.0, 0.0, 0.0], [1.0]),
            ([[0.0, 0.0]], [1.0]),
            ([0.0, 0.0], []),
            ([0.0, 0.0], [[1.0]]),
            ([math.nan, 0.0], [1.0]),
            ([0.0, 0.0], [math.inf]),
        )
        for gradient, residual in malformed:
            with self.subTest(gradient=gradient, residual=residual):
                try:
                    stationary = scale_aware_stationary(gradient, residual)
                except Exception as error:  # pragma: no cover - should fail closed
                    self.fail(f"stationarity helper raised {error!r}")
                self.assertFalse(stationary)

    def test_stationarity_is_checked_before_first_damped_solve(self):
        stationary_terms = (
            np.eye(2),
            np.ones(3),
            np.zeros(3),
            np.eye(2),
            np.zeros(2),
            0.0,
        )
        with mock.patch(
            "scripts.diagnostics.predictive_wnls._linearized_terms",
            return_value=stationary_terms,
        ), mock.patch(
            "scripts.diagnostics.predictive_wnls.np.linalg.solve",
            side_effect=AssertionError("stationary candidate must not solve"),
        ):
            result = solve_finite_budget_wnls(
                [[0.0, 0.0], [6.0, 0.0], [0.0, 8.0]],
                [np.eye(2), np.eye(2), np.eye(2)],
                [5.0, 5.0, 5.0],
                [3.0, 4.0],
                0.5,
            )
        self.assertEqual(result["status"], "converged")
        self.assertEqual(result["proposal_trace"], [])

    def test_rejected_proposals_increase_damping_without_overflow(self):
        with mock.patch(
            "scripts.diagnostics.predictive_wnls._linearized_terms",
            return_value=(
                np.eye(2),
                np.ones(2),
                np.ones(2),
                np.eye(2),
                np.array([1e6, 0.0]),
                2.0,
            ),
        ):
            result = solve_finite_budget_wnls(
                [[0.0, 0.0], [2.0, 0.0]],
                [np.eye(2), np.eye(2)],
                [1.0, 1.0],
                [1.0, 1.0],
                0.5,
            )
        damping = [row["damping"] for row in result["proposal_trace"]]
        self.assertEqual(damping[:2], [1e-3, 1e-2])
        self.assertEqual(damping[-1], 1e15)
        self.assertLessEqual(damping[-1], 1e15)
        self.assertEqual(result["failure_reason"], "maximum_damping_exceeded")

    def test_accepted_proposals_reach_and_hold_minimum_damping(self):
        def decreasing_terms(estimate, *_args):
            x_value = float(estimate[0])
            gradient = np.array([-1.0, 0.0]) if x_value < 13.5 else np.zeros(2)
            return (
                np.eye(2),
                np.ones(3),
                np.ones(3),
                np.eye(2),
                gradient,
                100.0 - x_value,
            )

        with mock.patch(
            "scripts.diagnostics.predictive_wnls._linearized_terms",
            side_effect=decreasing_terms,
        ), mock.patch(
            "scripts.diagnostics.predictive_wnls.fim_radius",
            return_value=make_converged_candidate_result(),
        ):
            result = solve_finite_budget_wnls(
                [[0.0, 0.0], [6.0, 0.0], [0.0, 8.0]],
                [np.eye(2), np.eye(2), np.eye(2)],
                [5.0, 5.0, 5.0],
                [0.0, 1.0],
                0.5,
            )
        damping = [row["damping"] for row in result["proposal_trace"]]
        self.assertEqual(result["status"], "converged")
        self.assertEqual(damping[-2:], [1e-15, 1e-15])

    def test_accepted_fiftieth_proposal_can_converge_immediately(self):
        def decreasing_terms(estimate, *_args):
            x_value = float(estimate[0])
            gradient = np.array([-1.0, 0.0]) if x_value < 49.5 else np.zeros(2)
            return (
                np.eye(2),
                np.ones(3),
                np.ones(3),
                np.eye(2),
                gradient,
                100.0 - x_value,
            )

        with mock.patch(
            "scripts.diagnostics.predictive_wnls._linearized_terms",
            side_effect=decreasing_terms,
        ), mock.patch(
            "scripts.diagnostics.predictive_wnls.fim_radius",
            return_value=make_converged_candidate_result(),
        ):
            result = solve_finite_budget_wnls(
                [[0.0, 0.0], [6.0, 0.0], [0.0, 8.0]],
                [np.eye(2), np.eye(2), np.eye(2)],
                [5.0, 5.0, 5.0],
                [0.0, 1.0],
                0.5,
            )
        self.assertEqual(result["status"], "converged")
        self.assertEqual(len(result["proposal_trace"]), 50)
        self.assertTrue(result["proposal_trace"][-1]["accepted"])

    def test_tiny_nonstationary_step_fails_with_explicit_reason(self):
        tiny_terms = (
            np.eye(2),
            np.ones(2),
            np.ones(2),
            1e20 * np.eye(2),
            np.array([1e-3, 0.0]),
            2.0,
        )
        with mock.patch(
            "scripts.diagnostics.predictive_wnls._linearized_terms",
            return_value=tiny_terms,
        ):
            result = solve_finite_budget_wnls(
                [[0.0, 0.0], [2.0, 0.0]],
                [np.eye(2), np.eye(2)],
                [1.0, 1.0],
                [1.0, 1.0],
                0.5,
            )
        self.assertEqual(
            result["failure_reason"],
            "no_representable_improving_step",
        )

    def test_trace_records_every_proposal(self):
        result = solve_finite_budget_wnls(
            [[0.0, 0.0], [6.0, 0.0], [0.0, 8.0]],
            [np.eye(2), np.eye(2), np.eye(2)],
            [5.0, 5.0, 5.0],
            [3.1, 4.1],
            0.5,
        )
        self.assertGreaterEqual(len(result["proposal_trace"]), 1)
        self.assertEqual(
            set(result["proposal_trace"][0]),
            {
                "proposal",
                "damping",
                "cost",
                "stationarity_norm",
                "raw_step_norm",
                "trial_cost",
                "invalid_trial_reason",
                "accepted",
            },
        )


class CandidateAcceptanceTests(unittest.TestCase):
    def test_mahalanobis_gate_uses_sum_of_prediction_and_range_covariance(self):
        result = normalized_innovation(
            [4.0, 0.0],
            [[1.0, 0.0], [0.0, 1.0]],
            {
                "estimate": [0.0, 0.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
            },
        )
        self.assertAlmostEqual(result["q_innov"], 8.0)
        self.assertTrue(result["valid"])

    def test_candidate_above_11_829007011943707_is_rejected(self):
        candidate = make_converged_candidate_result(estimate=(5.0, 0.0))
        accepted, reason, diagnostics = candidate_acceptance(
            candidate,
            live_prediction={
                "estimate": [0.0, 0.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
            },
            active_reference_count=3,
            base_anchor_provenance=[0, 1],
        )
        self.assertFalse(accepted)
        self.assertEqual(reason, "innovation_q_exceeds_reference_quantile")
        self.assertAlmostEqual(diagnostics["q_innov"], 12.5)

    def test_reacquisition_requires_three_refs_and_reduced_cost_at_most_nine(self):
        candidate = make_converged_candidate_result(cost=9.0)
        accepted, _, _ = candidate_acceptance(
            candidate,
            live_prediction=None,
            active_reference_count=3,
            base_anchor_provenance=[0, 1],
        )
        self.assertTrue(accepted)
        too_large = dict(candidate, cost=math.nextafter(9.0, math.inf))
        self.assertFalse(
            candidate_acceptance(
                too_large,
                live_prediction=None,
                active_reference_count=3,
                base_anchor_provenance=[0, 1],
            )[0]
        )
        self.assertFalse(
            candidate_acceptance(
                candidate,
                live_prediction=None,
                active_reference_count=2,
                base_anchor_provenance=[0, 1],
            )[0]
        )

    def test_multistart_ties_use_q_then_fixed_source_order(self):
        selected = select_candidate_result(
            [
                {
                    "source": "algebraic",
                    "accepted": True,
                    "cost": 1.0,
                    "q_innov": 2.0,
                },
                {
                    "source": "circle_negative",
                    "accepted": True,
                    "cost": 1.0 + 5e-13,
                    "q_innov": 1.0,
                },
            ],
            has_live_prediction=True,
        )
        self.assertEqual(selected["source"], "circle_negative")

    def test_fabricated_bare_fim_valid_flag_is_invalid(self):
        accepted, reason, diagnostics = candidate_acceptance(
            {
                "status": "converged",
                "estimate": [1.0, 1.0],
                "covariance": [[1.0, 0.0], [0.0, 1.0]],
                "cost": 0.0,
                "fim_valid": True,
            },
            live_prediction=None,
            active_reference_count=3,
            base_anchor_provenance=[0, 1],
        )
        self.assertFalse(accepted)
        self.assertEqual(reason, "invalid_candidate_output")
        self.assertEqual(diagnostics["gate_outcome"], "invalid")

    def test_nonfinite_proposal_trace_field_is_invalid(self):
        trace = ({
            "proposal": 0,
            "damping": 1e-3,
            "cost": math.nan,
            "stationarity_norm": 1.0,
            "raw_step_norm": 1.0,
            "trial_cost": 0.5,
            "invalid_trial_reason": None,
            "accepted": True,
        },)
        candidate = make_converged_candidate_result(proposal_trace=trace)
        accepted, reason, diagnostics = candidate_acceptance(
            candidate,
            live_prediction=None,
            active_reference_count=3,
            base_anchor_provenance=[0, 1],
        )
        self.assertFalse(accepted)
        self.assertEqual(reason, "invalid_candidate_output")
        self.assertEqual(diagnostics["gate_outcome"], "invalid")

    def test_reacquisition_cost_tie_uses_fixed_source_order(self):
        selected = select_candidate_result(
            [
                {
                    "source": "circle_positive",
                    "accepted": True,
                    "cost": 1.0,
                    "q_innov": None,
                },
                {
                    "source": "private_reacquisition_seed",
                    "accepted": True,
                    "cost": 1.0 + 5e-13,
                    "q_innov": None,
                },
            ],
            has_live_prediction=False,
        )
        self.assertEqual(selected["source"], "private_reacquisition_seed")


class PredictiveMultistartBoundaryTests(unittest.TestCase):
    def multistart_kwargs(self, live_prediction):
        return {
            "reference_positions": [
                [0.0, 0.0],
                [6.0, 0.0],
                [0.0, 8.0],
            ],
            "reference_covariances": [
                np.eye(2),
                np.eye(2),
                np.eye(2),
            ],
            "measurements": [5.0, 5.0, 5.0],
            "reference_keys": [
                ("base", 0),
                ("base", 1),
                ("base", 2),
            ],
            "live_prediction": live_prediction,
            "private_seed": None,
            "ranging_sigma": 0.5,
            "base_anchor_provenance": [0, 1],
        }

    def test_noncanonical_live_public_predictions_fail_before_candidates(self):
        valid = make_test_output(
            status="predicted",
            estimate=(3.0, 4.0),
            prediction_age=1,
            provenance=(),
        )
        missing_status = dict(valid)
        missing_status.pop("output_status")
        wrong_status = dict(valid, output_status="fresh")
        age_zero = dict(valid, prediction_age=0)
        age_three = dict(valid, prediction_age=3)
        nonnull_epsilon = dict(valid, epsilon=3.0)
        nonempty_provenance = dict(valid, base_anchor_provenance=[0, 1])
        bad_radius = dict(valid, aged_modeled_radius=6.5)
        bad_covariance = dict(
            valid,
            modeled_covariance=[[1.0, 2.0], [0.0, 1.0]],
        )
        for malformed in (
            missing_status,
            wrong_status,
            age_zero,
            age_three,
            nonnull_epsilon,
            nonempty_provenance,
            bad_radius,
            bad_covariance,
        ):
            with self.subTest(malformed=malformed):
                result = solve_predictive_multistart(
                    **self.multistart_kwargs(malformed)
                )
                self.assertEqual(result["attempt_status"], "invalid")
                self.assertEqual(result["candidates"], [])
                self.assertEqual(result["failure_reason"], "invalid_live_prediction")

    def test_successful_multistart_candidate_publishes_fresh_without_downgrade(self):
        live_prediction = make_test_output(
            status="predicted",
            estimate=(3.0, 4.0),
            prediction_age=1,
            provenance=(),
        )
        attempt = solve_predictive_multistart(
            **self.multistart_kwargs(live_prediction)
        )
        output = finalize_attempt(
            attempt,
            {"public_prediction": live_prediction},
            frame_index=4,
        )
        self.assertEqual(attempt["attempt_status"], "accepted")
        self.assertEqual(output["attempt_status"], "accepted")
        self.assertEqual(output["output_status"], "fresh")
        self.assertTrue(output_is_fresh(output))

    def test_malformed_fim_diagnostics_aggregate_as_invalid(self):
        live_prediction = make_test_output(
            status="predicted",
            estimate=(3.0, 4.0),
            prediction_age=1,
            provenance=(),
        )
        malformed = make_converged_candidate_result(estimate=(3.0, 4.0))
        malformed["phi_min_eigenvalue"] = math.nan
        with mock.patch(
            "scripts.diagnostics.predictive_wnls.solve_finite_budget_wnls",
            return_value=malformed,
        ):
            result = solve_predictive_multistart(
                **self.multistart_kwargs(live_prediction)
            )
        self.assertEqual(result["attempt_status"], "invalid")
        self.assertTrue(result["candidates"])
        self.assertTrue(
            all(not row["accepted"] for row in result["candidates"])
        )

    def test_complete_candidates_above_innovation_gate_aggregate_as_rejected(self):
        live_prediction = make_test_output(
            status="predicted",
            estimate=(3.0, 4.0),
            prediction_age=1,
            provenance=(),
        )
        rejected = make_converged_candidate_result(estimate=(30.0, 40.0))
        with mock.patch(
            "scripts.diagnostics.predictive_wnls.solve_finite_budget_wnls",
            return_value=rejected,
        ):
            result = solve_predictive_multistart(
                **self.multistart_kwargs(live_prediction)
            )
        self.assertEqual(result["attempt_status"], "rejected")

    def test_invalid_innovation_covariance_aggregates_as_invalid(self):
        live_prediction = make_test_output(
            status="predicted",
            estimate=(3.0, 4.0),
            prediction_age=1,
            provenance=(),
        )
        converged = make_converged_candidate_result(estimate=(3.0, 4.0))
        with mock.patch(
            "scripts.diagnostics.predictive_wnls.solve_finite_budget_wnls",
            return_value=converged,
        ), mock.patch(
            "scripts.diagnostics.predictive_wnls.normalized_innovation",
            return_value={
                "valid": False,
                "q_innov": None,
                "failure_reason": "innovation_covariance_not_positive_definite",
            },
        ):
            result = solve_predictive_multistart(
                **self.multistart_kwargs(live_prediction)
            )
        self.assertEqual(result["attempt_status"], "invalid")
