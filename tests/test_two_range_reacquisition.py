import unittest

from scripts.diagnostics.predictive_wnls import make_unavailable_output
from scripts.diagnostics.two_range_reacquisition import (
    advance_two_range_prior,
    finalize_two_range_lifecycle,
    propagate_private_state,
    reset_private_state,
)


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
