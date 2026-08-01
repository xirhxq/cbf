import json
import math
import unittest

from scripts.diagnostics.estimator_lifecycle import (
    PriorBundle,
    advance_qualified_prior,
    canonical_private_state,
    finalize_qualified_lifecycle,
    propagate_private_state,
    reset_private_state,
)
from scripts.diagnostics.qualified_modes import LocalCandidate, PublicationDecision


class PrivateHistoryTests(unittest.TestCase):
    def fresh_candidate(self):
        return {
            "estimate": [1.0, 2.0],
            "modeled_covariance": [[1.0, 0.0], [0.0, 2.0]],
        }

    def test_private_prior_propagates_one_held_transition(self):
        state = reset_private_state(
            self.fresh_candidate(),
            frame_index=0,
            history_version=0,
        )

        propagated = propagate_private_state(
            state,
            [4.0, -2.0],
            next_frame_index=1,
            applied_command_frame=0,
            history_version=1,
            mission_horizon_frames=10,
        )

        self.assertIsNotNone(propagated)
        self.assertEqual(propagated["estimate"], [3.0, 1.0])
        self.assertEqual(
            propagated["modeled_covariance"],
            [[1.25, 0.0], [0.0, 2.25]],
        )
        self.assertEqual(propagated["age_frames"], 1)
        self.assertEqual(propagated["last_command_frame"], 0)
        self.assertEqual(propagated["history_version"], 1)

    def test_private_age_h_minus_one_is_present_and_h_is_absent(self):
        state = reset_private_state(
            self.fresh_candidate(),
            frame_index=0,
            history_version=0,
        )
        for frame in range(1, 10):
            state = propagate_private_state(
                state,
                [0.0, 0.0],
                next_frame_index=frame,
                applied_command_frame=frame - 1,
                history_version=frame,
                mission_horizon_frames=10,
            )
        self.assertEqual(state["age_frames"], 9)
        self.assertIsNone(propagate_private_state(
            state,
            [0.0, 0.0],
            next_frame_index=10,
            applied_command_frame=9,
            history_version=10,
            mission_horizon_frames=10,
        ))

    def test_private_propagation_rejects_frame_command_version_and_horizon_errors(self):
        state = reset_private_state(
            self.fresh_candidate(),
            frame_index=4,
            history_version=8,
        )
        invalid_calls = (
            {"next_frame_index": 4, "applied_command_frame": 3,
             "history_version": 9, "mission_horizon_frames": 20},
            {"next_frame_index": 6, "applied_command_frame": 5,
             "history_version": 9, "mission_horizon_frames": 20},
            {"next_frame_index": 5, "applied_command_frame": 3,
             "history_version": 9, "mission_horizon_frames": 20},
            {"next_frame_index": 5, "applied_command_frame": 5,
             "history_version": 9, "mission_horizon_frames": 20},
            {"next_frame_index": 5, "applied_command_frame": 4,
             "history_version": 8, "mission_horizon_frames": 20},
            {"next_frame_index": 5, "applied_command_frame": 4,
             "history_version": 7, "mission_horizon_frames": 20},
            {"next_frame_index": 5, "applied_command_frame": 4,
             "history_version": 9, "mission_horizon_frames": 0},
        )
        for kwargs in invalid_calls:
            with self.subTest(kwargs=kwargs):
                self.assertIsNone(
                    propagate_private_state(state, [0.0, 0.0], **kwargs)
                )
        self.assertIsNone(propagate_private_state(
            state,
            [math.nan, 0.0],
            next_frame_index=5,
            applied_command_frame=4,
            history_version=9,
            mission_horizon_frames=20,
        ))

    def test_private_state_requires_exact_schema_and_canonical_spd(self):
        valid = reset_private_state(
            {
                "estimate": [1.0, 2.0],
                "modeled_covariance": [
                    [2.0, 0.1],
                    [0.10000000000000002, 1.0],
                ],
            },
            frame_index=0,
            history_version=0,
        )
        self.assertIsNotNone(canonical_private_state(valid))
        malformed = (
            dict(valid, unknown=True),
            {key: value for key, value in valid.items() if key != "age_frames"},
            dict(valid, status="predicted"),
            dict(valid, estimate=[math.inf, 0.0]),
            dict(valid, modeled_covariance=[[1.0, 1e-6], [0.0, 1.0]]),
            dict(valid, modeled_covariance=[[1.0, 0.0], [0.0, 1e-13]]),
            dict(valid, source_fresh_frame=1),
            dict(valid, last_command_frame=0),
            dict(valid, last_held_velocity=[0.0, 0.0]),
        )
        for state in malformed:
            with self.subTest(state=state):
                self.assertIsNone(canonical_private_state(state))

    def test_qualified_propagation_requires_explicit_chronology_and_horizon(self):
        state = reset_private_state(
            self.fresh_candidate(),
            frame_index=0,
            history_version=0,
        )
        with self.assertRaises(TypeError):
            propagate_private_state(
                state,
                [0.0, 0.0],
                next_frame_index=1,
            )


class QualifiedLifecycleTests(unittest.TestCase):
    def candidate(self, xy=(5.0, 6.0), covariance=None):
        if covariance is None:
            covariance = [[1.0, 0.0], [0.0, 2.0]]
        return LocalCandidate(
            "fresh-candidate",
            tuple(xy),
            0.0,
            {
                "covariance": covariance,
                "base_anchor_provenance": [0, 2],
            },
        )

    def fresh_decision(self, candidate=None):
        candidate = self.candidate() if candidate is None else candidate
        return PublicationDecision(
            "fresh",
            "unique_admissible_mode",
            "mode-1",
            candidate,
        )

    def unavailable_decision(self, representative=None):
        return PublicationDecision(
            "unavailable",
            "multiple_admissible_modes",
            None,
            representative,
        )

    def initial_fresh(self, *, horizon=10):
        incoming = advance_qualified_prior(
            None,
            None,
            [0.0, 0.0],
            next_frame_index=0,
            applied_command_frame=None,
            mission_horizon_frames=horizon,
            history_version=0,
        )
        return finalize_qualified_lifecycle(
            self.fresh_decision(),
            incoming,
            frame_index=0,
            mission_horizon_frames=horizon,
        )

    def test_frame_zero_has_no_incoming_prior_and_fresh_resets_history(self):
        incoming = advance_qualified_prior(
            {"output_status": "fresh"},
            {"status": "available"},
            [0.0, 0.0],
            next_frame_index=0,
            applied_command_frame=None,
            mission_horizon_frames=10,
            history_version=0,
        )
        self.assertIsNone(incoming.public_prediction)
        self.assertIsNone(incoming.private_prior)

        finalized = finalize_qualified_lifecycle(
            self.fresh_decision(),
            incoming,
            frame_index=0,
            mission_horizon_frames=10,
        )

        self.assertEqual(finalized["public_output"]["output_status"], "fresh")
        self.assertEqual(finalized["next_private_state"]["age_frames"], 0)
        self.assertEqual(finalized["next_private_state"]["history_version"], 1)
        self.assertIsNone(finalized["next_private_state"]["last_command_frame"])

    def test_rejection_preserves_already_propagated_private_state_byte_for_byte(self):
        initial = self.initial_fresh()
        prior = advance_qualified_prior(
            initial["public_output"],
            initial["next_private_state"],
            [2.0, -4.0],
            next_frame_index=1,
            applied_command_frame=0,
            mission_horizon_frames=10,
            history_version=2,
        )
        before = json.dumps(prior.private_prior, sort_keys=True, separators=(",", ":"))
        rejected = self.candidate(xy=(999.0, 999.0))

        result = finalize_qualified_lifecycle(
            self.unavailable_decision(rejected),
            prior,
            frame_index=1,
            mission_horizon_frames=10,
        )

        after = json.dumps(result["next_private_state"], sort_keys=True, separators=(",", ":"))
        self.assertIs(result["next_private_state"], prior.private_prior)
        self.assertEqual(after, before)
        self.assertEqual(result["public_output"]["output_status"], "predicted")
        self.assertNotEqual(result["next_private_state"]["estimate"], [999.0, 999.0])

    def test_fresh_at_later_frame_resets_age_and_advances_history_version(self):
        initial = self.initial_fresh()
        prior = advance_qualified_prior(
            initial["public_output"],
            initial["next_private_state"],
            [2.0, 0.0],
            next_frame_index=1,
            applied_command_frame=0,
            mission_horizon_frames=10,
            history_version=2,
        )

        result = finalize_qualified_lifecycle(
            self.fresh_decision(self.candidate(xy=(7.0, 8.0))),
            prior,
            frame_index=1,
            mission_horizon_frames=10,
        )

        state = result["next_private_state"]
        self.assertEqual(state["estimate"], [7.0, 8.0])
        self.assertEqual(state["source_fresh_frame"], 1)
        self.assertEqual(state["age_frames"], 0)
        self.assertEqual(state["last_command_frame"], 0)
        self.assertEqual(state["last_held_velocity"], [2.0, 0.0])
        self.assertEqual(state["history_version"], 3)

    def test_public_is_predicted_through_age_two_then_unavailable(self):
        initial = self.initial_fresh()
        public = initial["public_output"]
        private = initial["next_private_state"]
        outputs = []
        for frame in range(1, 4):
            prior = advance_qualified_prior(
                public,
                private,
                [2.0, 0.0],
                next_frame_index=frame,
                applied_command_frame=frame - 1,
                mission_horizon_frames=10,
                history_version=frame + 1,
            )
            finalized = finalize_qualified_lifecycle(
                self.unavailable_decision(),
                prior,
                frame_index=frame,
                mission_horizon_frames=10,
            )
            public = finalized["public_output"]
            private = finalized["next_private_state"]
            outputs.append((public, private))

        self.assertEqual(outputs[0][0]["prediction_age"], 1)
        self.assertEqual(outputs[1][0]["prediction_age"], 2)
        self.assertEqual(outputs[2][0]["output_status"], "unavailable")
        self.assertEqual(outputs[2][1]["age_frames"], 3)
        self.assertIsNotNone(outputs[2][1])

    def test_malformed_bundle_version_and_invalid_horizon_drop_private_state(self):
        initial = self.initial_fresh()
        mismatched = PriorBundle(
            initial["public_output"],
            initial["next_private_state"],
            0,
        )
        result = finalize_qualified_lifecycle(
            self.unavailable_decision(),
            mismatched,
            frame_index=0,
            mission_horizon_frames=10,
        )
        invalid_horizon = finalize_qualified_lifecycle(
            self.unavailable_decision(),
            PriorBundle(None, initial["next_private_state"], 1),
            frame_index=0,
            mission_horizon_frames=0,
        )

        self.assertIsNone(result["next_private_state"])
        self.assertIsNone(invalid_horizon["next_private_state"])

    def test_fresh_decision_cannot_replace_version_mismatched_private_bundle(self):
        initial = self.initial_fresh()
        mismatched = PriorBundle(
            initial["public_output"],
            initial["next_private_state"],
            0,
        )

        result = finalize_qualified_lifecycle(
            self.fresh_decision(),
            mismatched,
            frame_index=0,
            mission_horizon_frames=10,
        )

        self.assertEqual(result["public_output"]["output_status"], "unavailable")
        self.assertIsNone(result["next_private_state"])
        self.assertEqual(result["history_version"], 0)

    def test_malformed_public_inputs_canonicalize_unavailable_and_keep_private(self):
        initial = self.initial_fresh()
        prior = advance_qualified_prior(
            initial["public_output"],
            initial["next_private_state"],
            [0.0, 0.0],
            next_frame_index=1,
            applied_command_frame=0,
            mission_horizon_frames=10,
            history_version=2,
        )
        predicted = dict(prior.public_prediction)
        missing = dict(predicted)
        missing.pop("epsilon")
        malformed_inputs = {
            "missing_field": missing,
            "unknown_field": dict(predicted, unknown=True),
            "noncanonical_predicted_field": dict(predicted, epsilon=3.0),
            "recursive_forbidden_field": dict(
                predicted,
                diagnostics={"nested": [{"truth_position": [9.0, 9.0]}]},
            ),
            "stale_unavailable_localization": {
                "output_status": "unavailable",
                "estimate": [9.0, 9.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
                "epsilon": None,
                "prediction_age": None,
                "aged_modeled_radius": None,
                "base_anchor_provenance": [],
                "reason": "stale",
                "diagnostics": {"truth_position": [9.0, 9.0]},
            },
        }
        expected_keys = {
            "output_status",
            "estimate",
            "modeled_covariance",
            "epsilon",
            "prediction_age",
            "aged_modeled_radius",
            "base_anchor_provenance",
            "reason",
        }

        for name, public_input in malformed_inputs.items():
            with self.subTest(name=name):
                result = finalize_qualified_lifecycle(
                    self.unavailable_decision(),
                    PriorBundle(public_input, prior.private_prior, 2),
                    frame_index=1,
                    mission_horizon_frames=10,
                )

                public = result["public_output"]
                self.assertEqual(public["output_status"], "unavailable")
                self.assertEqual(set(public), expected_keys)
                self.assertIsNone(public["estimate"])
                self.assertIsNone(public["modeled_covariance"])
                self.assertNotIn("truth_position", json.dumps(public))
                self.assertIs(result["next_private_state"], prior.private_prior)
