import math
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from scripts.diagnostics.predictive_wnls import (
    finalize_attempt,
    make_unavailable_output,
    output_is_fresh,
    propagate_estimator_prior,
    reference_is_eligible,
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
