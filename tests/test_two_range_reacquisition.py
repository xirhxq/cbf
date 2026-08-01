import copy
import math
import unittest
from unittest import mock

import numpy as np

import scripts.diagnostics.two_range_reacquisition as two_range
from scripts.diagnostics.predictive_wnls import make_unavailable_output
from scripts.diagnostics.two_range_reacquisition import (
    advance_two_range_prior,
    finalize_two_range_lifecycle,
    propagate_private_state,
    reset_private_state,
)


def make_complete_converged_result(*, estimate=(1.0, 1.0), cost=0.0):
    return {
        "status": "converged",
        "estimate": list(estimate),
        "covariance": [[1.0, 0.0], [0.0, 1.0]],
        "epsilon": 3.0,
        "phi_min_eigenvalue": 1.0,
        "phi_condition": 1.0,
        "fim_valid": True,
        "proposal_count": 0,
        "iterations": 0,
        "cost": cost,
        "stationarity_norm": 0.0,
        "failure_reason": None,
        "proposal_trace": [],
    }


def valid_two_range_kwargs() -> dict:
    return {
        "robot_id": 12,
        "reference_positions": [[0.0, 0.0], [2.0, 0.0]],
        "reference_covariances": [
            [[0.1, 0.0], [0.0, 0.1]],
            [[0.1, 0.0], [0.0, 0.1]],
        ],
        "measurements": [math.sqrt(2.0), math.sqrt(2.0)],
        "reference_keys": [("uav", 10), ("uav", 11)],
        "private_prior": reset_private_state(
            {
                "estimate": [1.0, 1.75],
                "modeled_covariance": [[0.2, 0.0], [0.0, 0.2]],
            },
            frame_index=20,
        ),
        "ranging_sigma": 0.5,
        "base_anchor_provenance": [0, 1],
    }


class TaggedPrivateStateTests(unittest.TestCase):
    def test_reset_uses_current_fresh_candidate_and_age_zero(self):
        candidate = {
            "estimate": [1.0, -2.0],
            "modeled_covariance": [[2.0, 0.1], [0.1, 1.0]],
        }
        state = reset_private_state(candidate, frame_index=17)
        self.assertEqual(state["status"], "available")
        self.assertEqual(state["source_fresh_frame"], 17)
        self.assertEqual(state["propagated_to_frame"], 17)
        self.assertEqual(state["age_frames"], 0)
        self.assertEqual(state["estimate"], [1.0, -2.0])

    def test_reset_delegates_to_versioned_qualified_private_schema(self):
        state = reset_private_state(
            {
                "estimate": [1.0, -2.0],
                "modeled_covariance": [[2.0, 0.1], [0.1, 1.0]],
            },
            frame_index=17,
        )

        self.assertIn("history_version", state)
        self.assertEqual(state["history_version"], 0)
        self.assertIsNone(state["last_command_frame"])
        self.assertIsNone(state["last_held_velocity"])
        self.assertIsNotNone(two_range.canonical_private_state(state))

    def test_propagation_is_exactly_one_transition(self):
        state = reset_private_state(
            {
                "estimate": [1.0, 2.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 2.0]],
            },
            frame_index=8,
        )
        propagated = propagate_private_state(
            state,
            [4.0, -2.0],
            next_frame_index=9,
        )
        self.assertEqual(propagated["estimate"], [3.0, 1.0])
        self.assertEqual(
            propagated["modeled_covariance"],
            [[1.25, 0.0], [0.0, 2.25]],
        )
        self.assertEqual(propagated["source_fresh_frame"], 8)
        self.assertEqual(propagated["propagated_to_frame"], 9)
        self.assertEqual(propagated["age_frames"], 1)

    def test_same_frame_or_skipped_frame_propagation_rejects(self):
        state = reset_private_state(
            {
                "estimate": [0.0, 0.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
            },
            frame_index=4,
        )
        self.assertIsNone(
            propagate_private_state(state, [0.0, 0.0], next_frame_index=4)
        )
        self.assertIsNone(
            propagate_private_state(state, [0.0, 0.0], next_frame_index=6)
        )

    def test_private_state_has_no_public_prediction_age_expiry(self):
        state = {
            "status": "available",
            "estimate": [10.0, 20.0],
            "modeled_covariance": [[25.0, 0.0], [0.0, 25.0]],
            "source_fresh_frame": 0,
            "propagated_to_frame": 99,
            "age_frames": 99,
        }
        propagated = propagate_private_state(
            state,
            [0.0, 0.0],
            next_frame_index=100,
        )
        self.assertEqual(propagated["age_frames"], 100)
        self.assertEqual(propagated["propagated_to_frame"], 100)

    def test_expired_public_prediction_does_not_delete_private_state(self):
        bundle = advance_two_range_prior(
            previous_public={
                "output_status": "predicted",
                "prediction_age": 2,
                "estimate": [99.0, 99.0],
                "modeled_covariance": [[9.0, 0.0], [0.0, 9.0]],
                "epsilon": None,
                "aged_modeled_radius": 9.0,
                "base_anchor_provenance": [],
            },
            previous_private={
                "status": "available",
                "estimate": [1.0, 2.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
                "source_fresh_frame": 0,
                "propagated_to_frame": 2,
                "age_frames": 2,
            },
            held_velocity=[2.0, 0.0],
            next_frame_index=3,
        )
        self.assertEqual(
            bundle["public_prediction"]["output_status"], "unavailable"
        )
        self.assertEqual(
            bundle["branch_selection_prior"]["estimate"], [2.0, 2.0]
        )
        self.assertEqual(
            bundle["branch_selection_prior"]["propagated_to_frame"], 3
        )

    def test_live_public_prediction_never_uses_private_state_as_source(self):
        bundle = advance_two_range_prior(
            previous_public={
                "output_status": "fresh",
                "prediction_age": 0,
                "estimate": [100.0, 100.0],
                "modeled_covariance": [[4.0, 0.0], [0.0, 4.0]],
                "epsilon": 6.0,
                "aged_modeled_radius": None,
                "base_anchor_provenance": [0, 1],
            },
            previous_private={
                "status": "available",
                "estimate": [1.0, 2.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
                "source_fresh_frame": 0,
                "propagated_to_frame": 0,
                "age_frames": 0,
            },
            held_velocity=[2.0, 0.0],
            next_frame_index=1,
        )
        self.assertEqual(
            bundle["public_prediction"]["estimate"], [101.0, 100.0]
        )
        self.assertEqual(
            bundle["branch_selection_prior"]["estimate"], [2.0, 2.0]
        )

    def test_frame_zero_has_no_incoming_public_or_private_prior(self):
        bundle = advance_two_range_prior(
            previous_public={
                "output_status": "fresh",
                "prediction_age": 0,
                "estimate": [10.0, 20.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
                "epsilon": 3.0,
                "aged_modeled_radius": None,
                "base_anchor_provenance": [0, 1],
            },
            previous_private={
                "status": "available",
                "estimate": [1.0, 2.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
                "source_fresh_frame": 0,
                "propagated_to_frame": 0,
                "age_frames": 0,
            },
            held_velocity=[2.0, 0.0],
            next_frame_index=0,
        )
        self.assertEqual(
            bundle["public_prediction"]["output_status"], "unavailable"
        )
        self.assertIsNone(bundle["branch_selection_prior"])

    def test_accepted_attempt_resets_both_public_and_private_state(self):
        prior = {
            "public_prediction": make_unavailable_output("expired"),
            "branch_selection_prior": {
                "status": "available",
                "estimate": [9.0, 9.0],
                "modeled_covariance": [[5.0, 0.0], [0.0, 5.0]],
                "source_fresh_frame": 1,
                "propagated_to_frame": 7,
                "age_frames": 6,
            },
        }
        result = finalize_two_range_lifecycle(
            {
                "attempt_status": "accepted",
                "candidate": {
                    "estimate": [1.0, 2.0],
                    "modeled_covariance": [[1.0, 0.0], [0.0, 4.0]],
                    "base_anchor_provenance": [0, 1],
                },
            },
            prior,
            frame_index=7,
        )
        self.assertEqual(result["public_output"]["output_status"], "fresh")
        self.assertEqual(result["next_private_state"]["estimate"], [1.0, 2.0])
        self.assertEqual(
            result["next_private_state"]["source_fresh_frame"], 7
        )
        self.assertEqual(result["next_private_state"]["age_frames"], 0)

    def test_rejected_attempt_preserves_already_propagated_private_state(self):
        incoming = {
            "status": "available",
            "estimate": [9.0, 9.0],
            "modeled_covariance": [[5.0, 0.0], [0.0, 5.0]],
            "source_fresh_frame": 1,
            "propagated_to_frame": 7,
            "age_frames": 6,
        }
        result = finalize_two_range_lifecycle(
            {
                "attempt_status": "rejected",
                "candidate": None,
            },
            {
                "public_prediction": make_unavailable_output("expired"),
                "branch_selection_prior": incoming,
            },
            frame_index=7,
        )
        self.assertEqual(
            result["public_output"]["output_status"], "unavailable"
        )
        self.assertEqual(result["next_private_state"], incoming)


class TwoRangeBranchSelectorTests(unittest.TestCase):
    def test_branch_gate_uses_frozen_innovation_threshold(self):
        q_star = 11.829007011943707

        self.assertTrue(two_range.branch_gate_passes(q_star))
        self.assertTrue(
            two_range.branch_gate_passes(np.nextafter(q_star, -np.inf)),
        )
        self.assertFalse(
            two_range.branch_gate_passes(np.nextafter(q_star, np.inf)),
        )
        self.assertFalse(two_range.branch_gate_passes(float("nan")))
        self.assertFalse(two_range.branch_gate_passes(float("inf")))

    def test_validate_solver_branches_rejects_merged_results(self):
        result = make_complete_converged_result()

        valid, reason = two_range.validate_solver_branches(
            [
                {"solver_result": copy.deepcopy(result)},
                {"solver_result": copy.deepcopy(result)},
            ],
        )

        self.assertFalse(valid)
        self.assertEqual(reason, "two_range_solver_branches_merged")

    def test_two_range_selects_only_the_positive_circle_branch(self):
        attempt = two_range.solve_two_range_reacquisition(
            **valid_two_range_kwargs(),
        )

        self.assertEqual(
            [row["branch_id"] for row in attempt["branches"]],
            ["circle_negative", "circle_positive"],
        )
        self.assertEqual(attempt["selected_branch_id"], "circle_positive")
        self.assertTrue(
            math.isclose(
                attempt["branches"][0]["q_branch"],
                13.750000000000005,
                rel_tol=1e-12,
                abs_tol=1e-12,
            ),
        )
        self.assertTrue(
            math.isclose(
                attempt["branches"][1]["q_branch"],
                1.0227272727272725,
                rel_tol=1e-12,
                abs_tol=1e-12,
            ),
        )
        self.assertFalse(attempt["branches"][0]["passes_branch_gate"])
        self.assertTrue(attempt["branches"][1]["passes_branch_gate"])
        self.assertTrue(attempt["prior_used_for_branch_selection"])
        self.assertFalse(attempt["prior_used_in_fim"])
        self.assertFalse(attempt["prior_used_for_continuous_update"])

    def test_two_range_rejects_when_no_branch_passes(self):
        with mock.patch.object(
            two_range,
            "solve_finite_budget_wnls",
            side_effect=[
                make_complete_converged_result(estimate=(1.0, -1.0)),
                make_complete_converged_result(estimate=(1.0, 1.0)),
            ],
        ), mock.patch.object(
            two_range,
            "normalized_innovation",
            side_effect=[
                {"valid": True, "q_innov": 12.0, "failure_reason": None},
                {"valid": True, "q_innov": 13.0, "failure_reason": None},
            ],
        ):
            attempt = two_range.solve_two_range_reacquisition(
                **valid_two_range_kwargs(),
            )

        self.assertEqual(attempt["failure_reason"], "two_range_no_branch_passes")
        self.assertTrue(attempt["prior_used_for_branch_selection"])

    def test_two_range_rejects_when_multiple_branches_pass(self):
        with mock.patch.object(
            two_range,
            "solve_finite_budget_wnls",
            side_effect=[
                make_complete_converged_result(estimate=(1.0, -1.0)),
                make_complete_converged_result(estimate=(1.0, 1.0)),
            ],
        ), mock.patch.object(
            two_range,
            "normalized_innovation",
            side_effect=[
                {"valid": True, "q_innov": 1.0, "failure_reason": None},
                {"valid": True, "q_innov": 2.0, "failure_reason": None},
            ],
        ):
            attempt = two_range.solve_two_range_reacquisition(
                **valid_two_range_kwargs(),
            )

        self.assertEqual(
            attempt["failure_reason"],
            "two_range_multiple_branches_pass",
        )
        self.assertTrue(attempt["prior_used_for_branch_selection"])

    def test_two_range_rejects_tangent_circle_starts(self):
        kwargs = valid_two_range_kwargs()
        kwargs["measurements"] = [1.0, 1.0]

        attempt = two_range.solve_two_range_reacquisition(**kwargs)

        self.assertEqual(
            attempt["failure_reason"],
            "two_range_circle_starts_not_distinct",
        )

    def test_two_range_rejects_disjoint_circle_geometry(self):
        kwargs = valid_two_range_kwargs()
        kwargs["measurements"] = [0.5, 0.5]

        attempt = two_range.solve_two_range_reacquisition(**kwargs)

        self.assertEqual(
            attempt["failure_reason"],
            "two_range_circle_geometry_invalid",
        )

    def test_two_range_rejects_contained_circle_geometry(self):
        kwargs = valid_two_range_kwargs()
        kwargs["measurements"] = [3.0, 0.5]

        attempt = two_range.solve_two_range_reacquisition(**kwargs)

        self.assertEqual(
            attempt["failure_reason"],
            "two_range_circle_geometry_invalid",
        )

    def test_two_range_rejects_coincident_circle_centers(self):
        kwargs = valid_two_range_kwargs()
        kwargs["reference_positions"] = [[0.0, 0.0], [0.0, 0.0]]

        attempt = two_range.solve_two_range_reacquisition(**kwargs)

        self.assertEqual(
            attempt["failure_reason"],
            "two_range_circle_geometry_invalid",
        )

    def test_two_range_rejects_zero_range_input(self):
        kwargs = valid_two_range_kwargs()
        kwargs["measurements"] = [0.0, math.sqrt(2.0)]

        attempt = two_range.solve_two_range_reacquisition(**kwargs)

        self.assertEqual(attempt["failure_reason"], "two_range_input_invalid")

    def test_two_range_rejects_invalid_ranging_sigma_as_input_invalid(self):
        invalid_values = (
            True,
            False,
            float("nan"),
            float("inf"),
            float("-inf"),
            0.0,
            -0.5,
            [0.5],
            np.asarray(0.5),
            "0.5",
        )
        for ranging_sigma in invalid_values:
            with self.subTest(ranging_sigma=repr(ranging_sigma)):
                kwargs = valid_two_range_kwargs()
                kwargs["ranging_sigma"] = ranging_sigma

                attempt = two_range.solve_two_range_reacquisition(**kwargs)

                self.assertEqual(attempt["attempt_status"], "rejected")
                self.assertEqual(
                    attempt["failure_reason"],
                    "two_range_input_invalid",
                )
                self.assertEqual(attempt["branches"], [])
                self.assertIsNone(attempt["candidate"])
                self.assertFalse(attempt["prior_used_for_branch_selection"])

    def test_two_range_rejects_invalid_private_covariance(self):
        kwargs = valid_two_range_kwargs()
        kwargs["private_prior"]["modeled_covariance"] = [
            [1.0, 1.0],
            [1.0, 1.0],
        ]

        attempt = two_range.solve_two_range_reacquisition(**kwargs)

        self.assertEqual(
            attempt["failure_reason"],
            "two_range_private_prior_invalid",
        )

    def test_two_range_preserves_unscored_branches_when_innovation_is_nonfinite(self):
        with mock.patch.object(
            two_range,
            "solve_finite_budget_wnls",
            side_effect=[
                make_complete_converged_result(estimate=(1.0, -1.0)),
                make_complete_converged_result(estimate=(1.0, 1.0)),
            ],
        ), mock.patch.object(
            two_range,
            "normalized_innovation",
            side_effect=[
                {
                    "valid": False,
                    "q_innov": None,
                    "failure_reason": "non-finite_normalized_innovation",
                },
                {"valid": True, "q_innov": 1.0, "failure_reason": None},
            ],
        ):
            attempt = two_range.solve_two_range_reacquisition(
                **valid_two_range_kwargs(),
            )

        self.assertEqual(
            attempt["failure_reason"],
            "non-finite_normalized_innovation",
        )
        self.assertEqual(
            [branch["q_branch"] for branch in attempt["branches"]],
            [None, None],
        )
        self.assertEqual(
            [branch["passes_branch_gate"] for branch in attempt["branches"]],
            [None, None],
        )
        self.assertFalse(attempt["prior_used_for_branch_selection"])

    def test_two_range_rejects_one_failed_solver_branch(self):
        failed = make_complete_converged_result(estimate=(1.0, -1.0))
        failed["status"] = "invalid"
        failed["failure_reason"] = "unsolvable"
        with mock.patch.object(
            two_range,
            "solve_finite_budget_wnls",
            side_effect=[
                failed,
                make_complete_converged_result(estimate=(1.0, 1.0)),
            ],
        ):
            attempt = two_range.solve_two_range_reacquisition(
                **valid_two_range_kwargs(),
            )

        self.assertEqual(
            attempt["failure_reason"],
            "two_range_branch_solver_invalid",
        )

    def test_two_range_rejects_non_spd_solver_branch(self):
        non_spd = make_complete_converged_result(estimate=(1.0, -1.0))
        non_spd["covariance"] = [[1.0, 0.0], [0.0, 0.0]]
        with mock.patch.object(
            two_range,
            "solve_finite_budget_wnls",
            side_effect=[
                non_spd,
                make_complete_converged_result(estimate=(1.0, 1.0)),
            ],
        ):
            attempt = two_range.solve_two_range_reacquisition(
                **valid_two_range_kwargs(),
            )

        self.assertEqual(
            attempt["failure_reason"],
            "two_range_branch_solver_invalid",
        )

    def test_two_range_rejects_merged_solver_results(self):
        same_result = make_complete_converged_result(estimate=(1.0, 1.0))
        with mock.patch.object(
            two_range,
            "solve_finite_budget_wnls",
            side_effect=[same_result, same_result],
        ):
            attempt = two_range.solve_two_range_reacquisition(
                **valid_two_range_kwargs(),
            )

        self.assertEqual(
            attempt["failure_reason"],
            "two_range_solver_branches_merged",
        )

    def test_two_range_rejects_same_robot_reference_key(self):
        kwargs = valid_two_range_kwargs()
        kwargs["reference_keys"] = [("uav", 12), ("uav", 11)]

        attempt = two_range.solve_two_range_reacquisition(**kwargs)

        self.assertEqual(
            attempt["failure_reason"],
            "two_range_reference_keys_invalid",
        )

    def test_two_range_rejects_future_robot_reference_key(self):
        kwargs = valid_two_range_kwargs()
        kwargs["reference_keys"] = [("uav", 13), ("uav", 11)]

        attempt = two_range.solve_two_range_reacquisition(**kwargs)

        self.assertEqual(
            attempt["failure_reason"],
            "two_range_reference_keys_invalid",
        )

    def test_two_range_canonicalizes_swapped_reference_order(self):
        sorted_attempt = two_range.solve_two_range_reacquisition(
            **valid_two_range_kwargs(),
        )
        swapped = valid_two_range_kwargs()
        for field in (
            "reference_positions",
            "reference_covariances",
            "measurements",
            "reference_keys",
        ):
            swapped[field] = list(reversed(swapped[field]))
        swapped_attempt = two_range.solve_two_range_reacquisition(**swapped)

        self.assertEqual(
            [branch["branch_id"] for branch in swapped_attempt["branches"]],
            ["circle_negative", "circle_positive"],
        )
        self.assertEqual(
            swapped_attempt["selected_branch_id"],
            sorted_attempt["selected_branch_id"],
        )
        self.assertEqual(
            swapped_attempt["candidate"]["estimate"],
            sorted_attempt["candidate"]["estimate"],
        )
        self.assertEqual(
            swapped_attempt["candidate"]["modeled_covariance"],
            sorted_attempt["candidate"]["modeled_covariance"],
        )
        self.assertEqual(
            swapped_attempt["candidate"]["epsilon"],
            sorted_attempt["candidate"]["epsilon"],
        )
        self.assertEqual(
            swapped_attempt["candidate"]["base_anchor_provenance"],
            sorted_attempt["candidate"]["base_anchor_provenance"],
        )

    def test_two_range_rejects_invalid_base_anchor_provenance(self):
        kwargs = valid_two_range_kwargs()
        kwargs["base_anchor_provenance"] = [0, 0]

        attempt = two_range.solve_two_range_reacquisition(**kwargs)

        self.assertEqual(
            attempt["failure_reason"],
            "two_range_base_anchor_provenance_invalid",
        )

    def test_two_range_selects_negative_circle_for_negative_private_prior(self):
        kwargs = valid_two_range_kwargs()
        kwargs["private_prior"] = reset_private_state(
            {
                "estimate": [1.0, -1.75],
                "modeled_covariance": [[0.2, 0.0], [0.0, 0.2]],
            },
            frame_index=20,
        )

        attempt = two_range.solve_two_range_reacquisition(**kwargs)

        self.assertEqual(attempt["selected_branch_id"], "circle_negative")
        self.assertTrue(attempt["prior_used_for_branch_selection"])
        self.assertFalse(attempt["prior_used_in_fim"])
        self.assertFalse(attempt["prior_used_for_continuous_update"])

    def test_two_range_solver_starts_are_raw_circle_intersections(self):
        original_solver = two_range.solve_finite_budget_wnls
        with mock.patch.object(
            two_range,
            "solve_finite_budget_wnls",
            wraps=original_solver,
        ) as solver:
            two_range.solve_two_range_reacquisition(
                **valid_two_range_kwargs(),
            )

        starts = [
            np.asarray(call.args[3], dtype=float)
            for call in solver.call_args_list
        ]
        np.testing.assert_allclose(starts, [[1.0, -1.0], [1.0, 1.0]])
        for start in starts:
            self.assertFalse(np.array_equal(start, [1.0, 1.75]))

    def test_two_range_branch_records_do_not_expose_other_candidate_sources(self):
        attempt = two_range.solve_two_range_reacquisition(
            **valid_two_range_kwargs(),
        )

        names = [branch["branch_id"] for branch in attempt["branches"]]
        for name in names:
            self.assertNotIn("private", name)
            self.assertNotIn("algebraic", name)
            self.assertNotIn("prediction", name)

    def test_two_range_rejects_nearly_collinear_fim(self):
        kwargs = valid_two_range_kwargs()
        kwargs["measurements"] = [1.0000000000001, 1.0000000000001]

        attempt = two_range.solve_two_range_reacquisition(**kwargs)

        self.assertEqual(
            attempt["failure_reason"],
            "two_range_branch_solver_invalid",
        )

    def test_two_range_rejects_malformed_reference_positions(self):
        kwargs = valid_two_range_kwargs()
        kwargs["reference_positions"] = [[0.0, 0.0]]

        attempt = two_range.solve_two_range_reacquisition(**kwargs)

        self.assertEqual(attempt["failure_reason"], "two_range_input_invalid")

    def test_two_range_rejects_invalid_robot_id(self):
        kwargs = valid_two_range_kwargs()
        kwargs["robot_id"] = 0

        attempt = two_range.solve_two_range_reacquisition(**kwargs)

        self.assertEqual(attempt["failure_reason"], "two_range_robot_id_invalid")

    def test_two_range_rejects_invalid_reference_covariance(self):
        kwargs = valid_two_range_kwargs()
        kwargs["reference_covariances"][0] = [[1.0, 1.0], [1.0, 1.0]]

        attempt = two_range.solve_two_range_reacquisition(**kwargs)

        self.assertEqual(
            attempt["failure_reason"],
            "two_range_reference_covariance_invalid",
        )

    def test_two_range_rejects_wrong_reference_covariance_count_as_input_invalid(self):
        kwargs = valid_two_range_kwargs()
        kwargs["reference_covariances"] = []

        attempt = two_range.solve_two_range_reacquisition(**kwargs)

        self.assertEqual(attempt["failure_reason"], "two_range_input_invalid")

    def test_two_range_rejects_wrong_reference_key_count_as_input_invalid(self):
        kwargs = valid_two_range_kwargs()
        kwargs["reference_keys"] = []

        attempt = two_range.solve_two_range_reacquisition(**kwargs)

        self.assertEqual(attempt["failure_reason"], "two_range_input_invalid")

    def test_two_range_rejects_failed_branch_prediction_covariance_solve(self):
        with mock.patch.object(
            two_range,
            "solve_finite_budget_wnls",
            side_effect=[
                make_complete_converged_result(estimate=(1.0, -1.0)),
                make_complete_converged_result(estimate=(1.0, 1.0)),
            ],
        ), mock.patch.object(
            two_range,
            "normalized_innovation",
            side_effect=[
                {
                    "valid": False,
                    "q_innov": None,
                    "failure_reason": "innovation_covariance_invalid",
                },
                {"valid": True, "q_innov": 1.0, "failure_reason": None},
            ],
        ):
            attempt = two_range.solve_two_range_reacquisition(
                **valid_two_range_kwargs(),
            )

        self.assertEqual(
            attempt["failure_reason"],
            "innovation_covariance_invalid",
        )

    def test_two_range_rejects_uncoercible_input(self):
        kwargs = valid_two_range_kwargs()
        kwargs["reference_positions"] = "bad"

        attempt = two_range.solve_two_range_reacquisition(**kwargs)

        self.assertEqual(attempt["failure_reason"], "two_range_input_invalid")
