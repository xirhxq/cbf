import math
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from scripts.diagnostics.predictive_wnls import (
    make_unavailable_output,
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
