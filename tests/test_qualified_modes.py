import unittest
from dataclasses import asdict, replace
from hashlib import sha256
import json
import math
from itertools import combinations

import numpy as np

from scripts.diagnostics.qualified_modes import (
    DeploymentContract,
    MODE_TOLERANCE_M,
    LocalCandidate,
    ModeClustering,
    ModeQualification,
    NumericalMode,
    QualifiedStart,
    canonical_mode_id,
    cluster_candidates,
    enumerate_qualified_starts,
    project_local_candidate,
    publish_unique_mode,
    qualify_deployment_mode,
    qualify_all,
    qualify_history_mode,
    select_representative,
    sensitivity_cluster_counts,
    stable_attempt_id,
)


class ModeClusteringTests(unittest.TestCase):
    def candidate(self, key, xy, cost=1.0):
        return LocalCandidate(key, tuple(xy), cost, {"key": key})

    def test_submillimetre_starts_form_one_mode(self):
        result = cluster_candidates([
            self.candidate("a", (0.0, 0.0)),
            self.candidate("b", (0.0008, 0.0)),
        ], 0.001)
        self.assertTrue(result.separable)
        self.assertEqual(len(result.modes), 1)

    def test_registered_mirror_pair_forms_two_modes(self):
        result = cluster_candidates([
            self.candidate("ocean", (-1490.1017, -120.0)),
            self.candidate("land", (-1609.8983, -120.0)),
        ], MODE_TOLERANCE_M)
        self.assertTrue(result.separable)
        self.assertEqual(len(result.modes), 2)

    def test_chained_component_over_diameter_rejects_complete_frame(self):
        result = cluster_candidates([
            self.candidate("a", (0.0, 0.0)),
            self.candidate("b", (0.0008, 0.0)),
            self.candidate("c", (0.0016, 0.0)),
        ], 0.001)
        self.assertFalse(result.separable)
        self.assertEqual(result.reason, "nonseparable_chain")
        self.assertEqual(result.modes, ())

    def test_clustering_is_order_and_source_label_invariant(self):
        first = [self.candidate("stable-a", (0, 0)),
                 self.candidate("stable-b", (2, 0))]
        second = [
            replace(self.candidate("stable-b", (2, 0)),
                    payload={"source_label": "renamed-y"}),
            replace(self.candidate("stable-a", (0, 0)),
                    payload={"source_label": "renamed-x"}),
        ]
        self.assertEqual(
            tuple(mode.member_ids for mode in cluster_candidates(first, .001).modes),
            tuple(mode.member_ids for mode in cluster_candidates(second, .001).modes),
        )

    def test_projection_rejects_nonfinite_result_and_preserves_solver_payload(self):
        accepted = project_local_candidate("circle:negative", {
            "estimate": [3.0, 4.0], "cost": 1.25, "status": "converged",
        })

        self.assertEqual(accepted.attempt_id, "circle:negative")
        self.assertEqual(accepted.estimate, (3.0, 4.0))
        self.assertEqual(accepted.objective_cost, 1.25)
        self.assertEqual(accepted.payload["status"], "converged")
        self.assertIsNone(project_local_candidate("bad", {
            "estimate": [float("nan"), 0.0], "cost": 1.0,
        }))

    def test_representative_and_mode_id_ignore_source_labels(self):
        candidates = [
            self.candidate("z", (1.0, 2.0), 1.0),
            self.candidate("a", (1.0, 1.0), 1.0),
        ]
        mode = NumericalMode(("a", "z"), tuple(candidates), 0.0)
        expected_payload = json.dumps(
            sorted([(1.0.hex(), 1.0.hex()), (1.0.hex(), 2.0.hex())]),
            separators=(",", ":"),
        ).encode("utf-8")

        self.assertEqual(select_representative(mode).attempt_id, "a")
        self.assertEqual(canonical_mode_id(mode), sha256(expected_payload).hexdigest())
        renamed = NumericalMode(
            ("z", "a"),
            tuple(reversed([
                replace(candidate, payload={"source_label": "changed"})
                for candidate in candidates
            ])),
            0.0,
        )
        self.assertEqual(canonical_mode_id(mode), canonical_mode_id(renamed))

    def test_integer_coordinates_canonicalize_for_representative_and_mode_id(self):
        integer_candidate = LocalCandidate("integer", (-1490, 0), 1.0, {})
        integer_mode = NumericalMode(("integer",), (integer_candidate,), 0.0)
        float_candidate = LocalCandidate("float", (-1490.0, 0.0), 1.0, {})
        float_mode = NumericalMode(("float",), (float_candidate,), 0.0)

        try:
            representative = select_representative(integer_mode)
            integer_mode_id = canonical_mode_id(integer_mode)
        except Exception as error:
            outcome = type(error).__name__
            representative = None
            integer_mode_id = None
        else:
            outcome = "accepted"

        self.assertEqual(outcome, "accepted")
        self.assertTrue(all(type(value) is float for value in representative.estimate))
        self.assertEqual(integer_mode_id, canonical_mode_id(float_mode))

    def test_representative_and_mode_id_reject_malformed_coordinates_with_value_error(self):
        malformed_estimates = {
            "numeric_strings": ("-1490", "0"),
            "boolean": (True, 0.0),
            "nonfinite": (math.nan, 0.0),
            "oversized_integer": (10 ** 10000, 0.0),
        }
        for name, estimate in malformed_estimates.items():
            candidate = LocalCandidate("bad", estimate, 1.0, {})
            mode = NumericalMode(("bad",), (candidate,), 0.0)
            outcomes = []
            for operation in (select_representative, canonical_mode_id):
                try:
                    operation(mode)
                except Exception as error:
                    outcomes.append(type(error).__name__)
                else:
                    outcomes.append("accepted")
            with self.subTest(name=name):
                self.assertEqual(outcomes, ["ValueError", "ValueError"])

    def test_representative_precedence_is_complete_within_a_valid_cluster(self):
        cases = (
            (
                "cost",
                [self.candidate("higher-cost", (0.0, 0.0), 2.0),
                 self.candidate("lower-cost", (0.0001, 0.0), 1.0)],
                "lower-cost",
            ),
            (
                "x_hex",
                [self.candidate("smaller-x", (0.0, 0.0002)),
                 self.candidate("larger-x", (0.0001, 0.0))],
                "smaller-x",
            ),
            (
                "y_hex",
                [self.candidate("smaller-y", (0.0, 0.0)),
                 self.candidate("larger-y", (0.0, 0.0001))],
                "smaller-y",
            ),
            (
                "attempt_id",
                [self.candidate("a", (0.0, 0.0)),
                 self.candidate("z", (0.0, 0.0))],
                "a",
            ),
        )
        for name, candidates, expected in cases:
            with self.subTest(name=name):
                mode = cluster_candidates(candidates, MODE_TOLERANCE_M).modes[0]
                mutated = tuple(reversed([
                    replace(candidate, payload={"source_label": "renamed"})
                    for candidate in mode.members
                ]))
                self.assertEqual(select_representative(mode).attempt_id, expected)
                self.assertEqual(
                    select_representative(
                        NumericalMode(mode.member_ids, mutated, mode.diameter_m)
                    ).attempt_id,
                    expected,
                )

    def test_failed_innovation_diagnostic_cannot_hide_a_locally_eligible_mode(self):
        candidates = [
            LocalCandidate("near", (0.0, 0.0), 2.0, {
                "innovation": {"valid": True, "q_innov": 0.0},
            }),
            LocalCandidate("mirror", (120.0, 0.0), 1.0, {
                "innovation": {
                    "valid": False,
                    "failure_reason": "innovation_q_exceeds_reference_quantile",
                },
            }),
        ]

        result = cluster_candidates(candidates, MODE_TOLERANCE_M)

        self.assertTrue(result.separable)
        self.assertEqual(len(result.modes), 2)
        self.assertEqual(
            select_representative(result.modes[0]).attempt_id,
            "mirror",
        )

    def test_sensitivity_counts_use_every_frozen_tolerance(self):
        candidates = [
            self.candidate("a", (0.0, 0.0)),
            self.candidate("b", (0.0008, 0.0)),
        ]

        self.assertEqual(sensitivity_cluster_counts(candidates), {
            "0x1.0624dd2f1a9fcp-11": 2,
            "0x1.0624dd2f1a9fcp-10": 1,
            "0x1.0624dd2f1a9fcp-9": 1,
        })


class DeploymentQualificationTests(unittest.TestCase):
    def domain(self, **changes):
        domain = DeploymentContract(
            anchor_ids=(0, 2),
            anchor_coordinates=((-1550.0, -300.0), (-1550.0, 300.0)),
            deployment_vertices=(
                (-1491.0, -201.0),
                (-1369.0, -201.0),
                (-1369.0, 201.0),
                (-1491.0, 201.0),
            ),
            unit_normal=(1.0, 0.0),
            offset=1550.0,
            ocean_side=1,
            margin_m=1.0,
            domain_version="ocean-side-v1",
        )
        return replace(domain, **changes)

    def mode(self, key, xy, cost=1.0):
        return cluster_candidates(
            [LocalCandidate(key, tuple(xy), cost, {"key": key})],
            MODE_TOLERANCE_M,
        ).modes[0]

    def clustering(self, *modes):
        candidates = [member for mode in modes for member in mode.members]
        return cluster_candidates(candidates, MODE_TOLERANCE_M)

    def test_ocean_side_domain_selects_one_mirror_mode(self):
        domain = self.domain()
        ocean = self.mode("ocean", (-1490.0, 0.0), cost=100.0)
        land = self.mode("land", (-1610.0, 0.0), cost=1.0)
        decisions = tuple(
            qualify_deployment_mode(mode, select_representative(mode), domain)
            for mode in (ocean, land)
        )

        publication = publish_unique_mode(
            self.clustering(ocean, land),
            decisions,
        )

        self.assertEqual(publication.status, "fresh")
        self.assertEqual(publication.mode_id, canonical_mode_id(ocean))

    def test_cross_line_or_invalid_domain_fails_closed(self):
        invalid = DeploymentContract(
            anchor_ids=(0, 2),
            anchor_coordinates=((0.0, 0.0), (1.0, 0.0)),
            deployment_vertices=((-1.0, -1.0), (1.0, 1.0)),
            unit_normal=(0.0, 0.0),
            offset=0.0,
            ocean_side=1,
            margin_m=1.0,
            domain_version="ocean-side-v1",
        )

        with self.assertRaises(ValueError):
            mode = self.mode("invalid", (0.0, 0.0))
            qualify_deployment_mode(mode, select_representative(mode), invalid)

    def test_margin_is_inclusive_and_one_ulp_below_is_rejected(self):
        equality = self.mode("equality", (-1549.0, 4.0))
        below = self.mode(
            "below",
            (math.nextafter(-1549.0, -math.inf), 4.0),
        )

        equality_result = qualify_deployment_mode(
            equality,
            select_representative(equality),
            self.domain(),
        )
        below_result = qualify_deployment_mode(
            below,
            select_representative(below),
            self.domain(),
        )

        self.assertTrue(equality_result.admissible)
        self.assertEqual(equality_result.score, 1.0)
        self.assertFalse(below_result.admissible)
        self.assertLess(below_result.score, 1.0)

    def test_invalid_deployment_contract_variants_raise(self):
        invalid_contracts = {
            "reversed_normal": self.domain(unit_normal=(-1.0, 0.0)),
            "unnormalized_normal": self.domain(unit_normal=(2.0, 0.0)),
            "duplicate_anchor_ids": self.domain(anchor_ids=(0, 0)),
            "negative_anchor_id": self.domain(anchor_ids=(0, -1)),
            "duplicate_anchor_coordinates": self.domain(
                anchor_coordinates=((-1550.0, -300.0), (-1550.0, -300.0)),
            ),
            "bad_offset": self.domain(offset=1549.0),
            "bad_version": self.domain(domain_version="ocean-side-v2"),
            "bad_ocean_side": self.domain(ocean_side=0),
            "zero_margin": self.domain(margin_m=0.0),
            "nonfinite_margin": self.domain(margin_m=math.nan),
            "nonfinite_anchor": self.domain(
                anchor_coordinates=((math.inf, -300.0), (-1550.0, 300.0)),
            ),
            "nonfinite_normal": self.domain(unit_normal=(math.nan, 0.0)),
            "nonfinite_vertex": self.domain(
                deployment_vertices=((math.nan, 0.0),),
            ),
            "no_vertices": self.domain(deployment_vertices=()),
            "crossing_region": self.domain(
                deployment_vertices=((-1491.0, 0.0), (-1550.0, 0.0)),
            ),
        }
        candidate = self.mode("candidate", (-1490.0, 0.0))
        representative = select_representative(candidate)

        for name, domain in invalid_contracts.items():
            with self.subTest(name=name), self.assertRaises(ValueError):
                qualify_deployment_mode(candidate, representative, domain)

    def test_deployment_vectors_reject_numeric_strings_with_value_error(self):
        invalid_contracts = {
            "anchor_coordinates": self.domain(
                anchor_coordinates=(("-1550", "-300"), ("-1550", "300")),
            ),
            "unit_normal": self.domain(unit_normal=("1", "0")),
            "deployment_vertices": self.domain(
                deployment_vertices=(("-1491", "0"),),
            ),
        }
        mode = self.mode("candidate", (-1490.0, 0.0))
        representative = select_representative(mode)

        for name, domain in invalid_contracts.items():
            with self.subTest(name=name):
                try:
                    qualify_deployment_mode(mode, representative, domain)
                except Exception as error:
                    outcome = type(error).__name__
                else:
                    outcome = "no_exception"
                self.assertEqual(outcome, "ValueError")

    def test_candidate_and_representative_must_be_finite_and_consistent(self):
        good = LocalCandidate("good", (-1490.0, 0.0), 1.0, {})
        other = LocalCandidate("other", (-1480.0, 0.0), 1.0, {})
        mode = NumericalMode(("good",), (good,), 0.0)
        malformed = NumericalMode(
            ("bad",),
            (LocalCandidate("bad", (math.nan, 0.0), 1.0, {}),),
            0.0,
        )

        with self.assertRaises(ValueError):
            qualify_deployment_mode(mode, other, self.domain())
        with self.assertRaises(ValueError):
            qualify_deployment_mode(
                malformed,
                malformed.members[0],
                self.domain(),
            )

    def test_deployment_canonicalizes_integers_and_rejects_malformed_coordinates(self):
        integer_candidate = LocalCandidate("integer", (-1490, 0), 1.0, {})
        integer_mode = NumericalMode(("integer",), (integer_candidate,), 0.0)
        try:
            integer_result = qualify_deployment_mode(
                integer_mode,
                integer_candidate,
                self.domain(),
            )
        except Exception as error:
            integer_outcome = type(error).__name__
        else:
            integer_outcome = "admissible" if integer_result.admissible else "rejected"
        self.assertEqual(integer_outcome, "admissible")

        malformed_estimates = {
            "numeric_strings": ("-1490", "0"),
            "boolean": (True, 0.0),
            "nonfinite": (math.nan, 0.0),
            "oversized_integer": (10 ** 10000, 0.0),
        }
        for name, estimate in malformed_estimates.items():
            candidate = LocalCandidate("bad", estimate, 1.0, {})
            mode = NumericalMode(("bad",), (candidate,), 0.0)
            try:
                qualify_deployment_mode(mode, candidate, self.domain())
            except Exception as error:
                outcome = type(error).__name__
            else:
                outcome = "accepted"
            with self.subTest(name=name):
                self.assertEqual(outcome, "ValueError")

    def test_publication_refuses_nonseparable_zero_and_multiple_modes(self):
        ocean = self.mode("ocean", (-1490.0, 0.0))
        land = self.mode("land", (-1610.0, 0.0))
        clustering = self.clustering(ocean, land)
        reject_all = tuple(
            qualify_deployment_mode(mode, select_representative(mode), self.domain())
            for mode in (land,)
        )
        zero = publish_unique_mode(
            self.clustering(land),
            reject_all,
        )
        multiple = publish_unique_mode(
            clustering,
            tuple(
                replace(
                    qualify_deployment_mode(
                        mode,
                        select_representative(mode),
                        self.domain(),
                    ),
                    admissible=True,
                )
                for mode in clustering.modes
            ),
        )
        nonseparable = publish_unique_mode(
            replace(clustering, separable=False, reason="nonseparable_chain", modes=()),
            (),
        )

        self.assertEqual(zero.reason, "no_admissible_mode")
        self.assertEqual(multiple.reason, "multiple_admissible_modes")
        self.assertEqual(nonseparable.reason, "nonseparable_chain")
        self.assertTrue(all(
            decision.status == "unavailable"
            for decision in (zero, multiple, nonseparable)
        ))

    def test_malformed_publication_inputs_fail_closed(self):
        mode = self.mode("candidate", (-1490.0, 0.0))
        clustering = self.clustering(mode)
        qualification = qualify_deployment_mode(
            mode,
            select_representative(mode),
            self.domain(),
        )
        malformed_cases = (
            (),
            (qualification, qualification),
            (replace(qualification, mode_id="unknown"),),
            (replace(qualification, mode_id=""),),
        )
        for qualifications in malformed_cases:
            with self.subTest(qualifications=qualifications):
                decision = publish_unique_mode(clustering, qualifications)
                self.assertEqual(decision.status, "unavailable")
                self.assertEqual(decision.reason, "malformed_qualification_input")
        valid_qualification = replace(qualification, admissible=True)
        malformed_clustering = replace(clustering, tolerance_m=math.nan)
        decision = publish_unique_mode(
            malformed_clustering,
            (valid_qualification,),
        )
        self.assertEqual(decision.status, "unavailable")
        self.assertEqual(decision.reason, "malformed_clustering_input")

    def test_publication_canonicalizes_integers_and_fails_closed_on_bad_coordinates(self):
        integer_candidate = LocalCandidate("integer", (-1490, 0), 1.0, {})
        integer_mode = NumericalMode(("integer",), (integer_candidate,), 0.0)
        float_mode = NumericalMode(
            ("float",),
            (LocalCandidate("float", (-1490.0, 0.0), 1.0, {}),),
            0.0,
        )
        integer_clustering = ModeClustering(
            MODE_TOLERANCE_M,
            True,
            "separable",
            (integer_mode,),
        )
        qualification = ModeQualification(
            canonical_mode_id(float_mode),
            True,
            "deployment_side",
            60.0,
        )
        try:
            published = publish_unique_mode(integer_clustering, (qualification,))
        except Exception as error:
            integer_outcome = type(error).__name__
            published = None
        else:
            integer_outcome = published.status

        self.assertEqual(integer_outcome, "fresh")
        self.assertTrue(all(
            type(value) is float for value in published.representative.estimate
        ))

        malformed_estimates = {
            "numeric_strings": ("-1490", "0"),
            "boolean": (True, 0.0),
            "nonfinite": (math.nan, 0.0),
            "oversized_integer": (10 ** 10000, 0.0),
        }
        for name, estimate in malformed_estimates.items():
            candidate = LocalCandidate("bad", estimate, 1.0, {})
            mode = NumericalMode(("bad",), (candidate,), 0.0)
            clustering = ModeClustering(
                MODE_TOLERANCE_M,
                True,
                "separable",
                (mode,),
            )
            try:
                decision = publish_unique_mode(clustering, (qualification,))
            except Exception as error:
                outcome = type(error).__name__
            else:
                outcome = decision.status
            with self.subTest(name=name):
                self.assertEqual(outcome, "unavailable")


class HistoryQualificationTests(unittest.TestCase):
    def mode(self, key, xy, covariance=None):
        if covariance is None:
            covariance = [[0.5, 0.0], [0.0, 0.5]]
        candidate = LocalCandidate(
            key,
            tuple(xy),
            1.0,
            {"covariance": covariance},
        )
        return cluster_candidates([candidate], MODE_TOLERANCE_M).modes[0]

    def prior(self, xy=(0.0, 0.0), covariance=None):
        if covariance is None:
            covariance = [[0.5, 0.0], [0.0, 0.5]]
        return {
            "status": "available",
            "estimate": list(xy),
            "modeled_covariance": covariance,
            "source_fresh_frame": 0,
            "propagated_to_frame": 1,
            "age_frames": 1,
            "last_command_frame": 0,
            "last_held_velocity": [0.0, 0.0],
            "history_version": 1,
        }

    def test_unique_low_innovation_mode_publishes_fresh(self):
        near = self.mode("near", (1.0, 0.0))
        far = self.mode("far", (20.0, 0.0))
        clustering = cluster_candidates(
            [near.members[0], far.members[0]],
            MODE_TOLERANCE_M,
        )
        qualifications = tuple(
            qualify_history_mode(
                mode,
                select_representative(mode),
                self.prior(),
                11.829007011943707,
            )
            for mode in clustering.modes
        )

        publication = publish_unique_mode(clustering, qualifications)

        self.assertEqual(publication.status, "fresh")
        self.assertEqual(publication.mode_id, canonical_mode_id(near))
        self.assertEqual(publication.representative.attempt_id, "near")

    def test_history_canonicalizes_integers_and_rejects_malformed_coordinates(self):
        integer_candidate = LocalCandidate(
            "integer",
            (1, 0),
            1.0,
            {"covariance": [[0.5, 0.0], [0.0, 0.5]]},
        )
        integer_mode = NumericalMode(("integer",), (integer_candidate,), 0.0)
        try:
            integer_result = qualify_history_mode(
                integer_mode,
                integer_candidate,
                self.prior(),
                11.829007011943707,
            )
        except Exception as error:
            integer_outcome = type(error).__name__
        else:
            integer_outcome = "admissible" if integer_result.admissible else "rejected"
        self.assertEqual(integer_outcome, "admissible")

        malformed_estimates = {
            "numeric_strings": ("1", "0"),
            "boolean": (True, 0.0),
            "nonfinite": (math.nan, 0.0),
            "oversized_integer": (10 ** 10000, 0.0),
        }
        for name, estimate in malformed_estimates.items():
            candidate = replace(integer_candidate, attempt_id="bad", estimate=estimate)
            mode = NumericalMode(("bad",), (candidate,), 0.0)
            try:
                qualify_history_mode(
                    mode,
                    candidate,
                    self.prior(),
                    11.829007011943707,
                )
            except Exception as error:
                outcome = type(error).__name__
            else:
                outcome = "accepted"
            with self.subTest(name=name):
                self.assertEqual(outcome, "ValueError")

    def test_zero_or_multiple_history_modes_do_not_publish(self):
        near_a = self.mode("near-a", (1.0, 0.0))
        near_b = self.mode("near-b", (-1.0, 0.0))
        far = self.mode("far", (20.0, 0.0))
        zero_clustering = cluster_candidates(
            [far.members[0]],
            MODE_TOLERANCE_M,
        )
        multiple_clustering = cluster_candidates(
            [near_a.members[0], near_b.members[0]],
            MODE_TOLERANCE_M,
        )

        zero = publish_unique_mode(
            zero_clustering,
            tuple(
                qualify_history_mode(
                    mode,
                    select_representative(mode),
                    self.prior(),
                    11.829007011943707,
                )
                for mode in zero_clustering.modes
            ),
        )
        multiple = publish_unique_mode(
            multiple_clustering,
            tuple(
                qualify_history_mode(
                    mode,
                    select_representative(mode),
                    self.prior(),
                    11.829007011943707,
                )
                for mode in multiple_clustering.modes
            ),
        )

        self.assertEqual(zero.reason, "no_admissible_mode")
        self.assertEqual(multiple.reason, "multiple_admissible_modes")
        self.assertEqual(zero.status, "unavailable")
        self.assertEqual(multiple.status, "unavailable")

    def test_history_threshold_is_inclusive_at_exact_float_boundary(self):
        threshold = 11.829007011943707
        cases = (
            (math.nextafter(threshold, -math.inf), True),
            (threshold, True),
            (math.nextafter(threshold, math.inf), False),
        )
        for q_value, expected in cases:
            with self.subTest(q_value=q_value):
                half_variance = 0.5 / q_value
                covariance = [[half_variance, 0.0], [0.0, 1.0]]
                mode = self.mode("candidate", (1.0, 0.0), covariance)
                result = qualify_history_mode(
                    mode,
                    select_representative(mode),
                    self.prior(covariance=covariance),
                    threshold,
                )
                self.assertEqual(result.score, q_value)
                self.assertEqual(result.admissible, expected)

    def test_history_rejects_noncanonical_covariance_and_limit(self):
        mode = self.mode("candidate", (1.0, 0.0))
        representative = select_representative(mode)
        malformed_covariances = (
            [[1.0, 1e-6], [0.0, 1.0]],
            [[1.0, 0.0], [0.0, 1e-13]],
            [[1.0, 0.0], [0.0, 0.0]],
            [[math.nan, 0.0], [0.0, 1.0]],
        )
        for covariance in malformed_covariances:
            with self.subTest(source="prior", covariance=covariance):
                with self.assertRaises(ValueError):
                    qualify_history_mode(
                        mode,
                        representative,
                        self.prior(covariance=covariance),
                        11.829007011943707,
                    )
            with self.subTest(source="candidate", covariance=covariance):
                malformed_mode = self.mode("bad", (1.0, 0.0), covariance)
                with self.assertRaises(ValueError):
                    qualify_history_mode(
                        malformed_mode,
                        select_representative(malformed_mode),
                        self.prior(),
                        11.829007011943707,
                    )
        with self.assertRaises(ValueError):
            qualify_history_mode(
                mode,
                representative,
                self.prior(),
                math.nextafter(11.829007011943707, math.inf),
            )

    def test_representative_payload_must_match_mode_member(self):
        mode = self.mode("candidate", (20.0, 0.0))
        representative = replace(
            select_representative(mode),
            payload={"covariance": [[100.0, 0.0], [0.0, 100.0]]},
        )

        with self.assertRaises(ValueError):
            qualify_history_mode(
                mode,
                representative,
                self.prior(),
                11.829007011943707,
            )

    def test_incorrect_self_consistent_prior_is_analyzer_side_wrong_unique(self):
        false_mode = self.mode("false", (100.0, 0.0))
        true_mode = self.mode("true", (0.0, 0.0))
        clustering = cluster_candidates(
            [false_mode.members[0], true_mode.members[0]],
            MODE_TOLERANCE_M,
        )
        qualifications = tuple(
            qualify_history_mode(
                mode,
                select_representative(mode),
                self.prior(xy=(100.0, 0.0)),
                11.829007011943707,
            )
            for mode in clustering.modes
        )

        runtime = publish_unique_mode(clustering, qualifications)
        truth_mode_id = canonical_mode_id(true_mode)
        analyzer_label = (
            "wrong_unique"
            if runtime.status == "fresh" and runtime.mode_id != truth_mode_id
            else "not_wrong_unique"
        )

        self.assertEqual(runtime.status, "fresh")
        self.assertEqual(runtime.reason, "unique_admissible_mode")
        self.assertEqual(runtime.mode_id, canonical_mode_id(false_mode))
        self.assertEqual(analyzer_label, "wrong_unique")


class QualifierPayloadSchemaTests(unittest.TestCase):
    def setUp(self):
        candidate = LocalCandidate(
            "candidate",
            (-1490.0, 0.0),
            1.0,
            {"covariance": [[1.0, 0.0], [0.0, 1.0]]},
        )
        self.mode = cluster_candidates(
            [candidate],
            MODE_TOLERANCE_M,
        ).modes[0]
        self.representative = select_representative(self.mode)
        self.domain = DeploymentContract(
            anchor_ids=(0, 2),
            anchor_coordinates=((-1550.0, -300.0), (-1550.0, 300.0)),
            deployment_vertices=((-1491.0, 0.0), (-1369.0, 0.0)),
            unit_normal=(1.0, 0.0),
            offset=1550.0,
            ocean_side=1,
            margin_m=1.0,
            domain_version="ocean-side-v1",
        )
        self.prior = {
            "status": "available",
            "estimate": [-1490.0, 0.0],
            "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
            "source_fresh_frame": 0,
            "propagated_to_frame": 1,
            "age_frames": 1,
            "last_command_frame": 0,
            "last_held_velocity": [0.0, 0.0],
            "history_version": 1,
        }

    def test_exact_deployment_and_history_payload_schemas_are_accepted(self):
        deployment = qualify_all(
            (self.mode,),
            (self.representative,),
            "deployment",
            {"domain": asdict(self.domain)},
        )
        history = qualify_all(
            (self.mode,),
            (self.representative,),
            "history",
            {
                "propagated_private_prior": self.prior,
                "innovation_limit": 11.829007011943707,
            },
        )

        self.assertTrue(deployment[0].admissible)
        self.assertTrue(history[0].admissible)

    def test_unknown_or_forbidden_runtime_payload_fields_raise(self):
        forbidden = (
            "truth_position",
            "future_estimate",
            "analyzer_label",
            "realized_error",
        )
        for key in forbidden:
            with self.subTest(kind="deployment", key=key):
                with self.assertRaises(ValueError):
                    qualify_all(
                        (self.mode,),
                        (self.representative,),
                        "deployment",
                        {"domain": self.domain, key: [0.0, 0.0]},
                    )
            with self.subTest(kind="history", key=key):
                with self.assertRaises(ValueError):
                    qualify_all(
                        (self.mode,),
                        (self.representative,),
                        "history",
                        {
                            "propagated_private_prior": self.prior,
                            "innovation_limit": 11.829007011943707,
                            key: [0.0, 0.0],
                        },
                    )
        nested_domain = asdict(self.domain)
        nested_domain["truth_position"] = [0.0, 0.0]
        with self.assertRaises(ValueError):
            qualify_all(
                (self.mode,),
                (self.representative,),
                "deployment",
                {"domain": nested_domain},
            )
        with self.assertRaises(ValueError):
            qualify_all(
                (self.mode,),
                (self.representative,),
                "deployment",
                {"domain": self.domain, "unknown": True},
            )

    def test_direct_qualifiers_reject_forbidden_candidate_payload_fields(self):
        for key in (
            "truth_position",
            "future_estimate",
            "analyzer_label",
            "realized_error",
        ):
            candidate = replace(
                self.representative,
                payload={
                    "covariance": [[1.0, 0.0], [0.0, 1.0]],
                    key: [0.0, 0.0],
                },
            )
            mode = NumericalMode(
                (candidate.attempt_id,),
                (candidate,),
                0.0,
            )
            with self.subTest(kind="deployment", key=key):
                with self.assertRaises(ValueError):
                    qualify_deployment_mode(mode, candidate, self.domain)
            with self.subTest(kind="history", key=key):
                with self.assertRaises(ValueError):
                    qualify_history_mode(
                        mode,
                        candidate,
                        self.prior,
                        11.829007011943707,
                    )

    def test_nested_object_arrays_and_opaque_payloads_fail_closed(self):
        malformed_payloads = {
            "nested_forbidden_object_array": {
                "covariance": [[1.0, 0.0], [0.0, 1.0]],
                "metadata": np.array(
                    [{"nested": {"truth_position": [9.0, 9.0]}}],
                    dtype=object,
                ),
            },
            "unsupported_opaque_container": {
                "covariance": [[1.0, 0.0], [0.0, 1.0]],
                "metadata": object(),
            },
        }
        for name, payload in malformed_payloads.items():
            candidate = replace(self.representative, payload=payload)
            mode = NumericalMode(
                (candidate.attempt_id,),
                (candidate,),
                0.0,
            )
            with self.subTest(name=name), self.assertRaises(ValueError):
                qualify_all(
                    (mode,),
                    (candidate,),
                    "deployment",
                    {"domain": self.domain},
                )

    def test_mismatched_modes_representatives_and_unknown_kind_raise(self):
        with self.assertRaises(ValueError):
            qualify_all(
                (self.mode,),
                (),
                "deployment",
                {"domain": self.domain},
            )
        with self.assertRaises(ValueError):
            qualify_all(
                (self.mode,),
                (self.representative,),
                "unsupported",
                {},
            )


class QualifiedStartEnumerationTests(unittest.TestCase):
    def reference(self, key, position, radius):
        return {"key": key, "position": position, "range": radius}

    def expected_circle_roots(self, first, second):
        first_key, first_position, first_radius = first
        second_key, second_position, second_radius = second
        dx = second_position[0] - first_position[0]
        dy = second_position[1] - first_position[1]
        baseline = math.hypot(dx, dy)
        if (
            baseline == 0.0
            or baseline > first_radius + second_radius
            or baseline < abs(first_radius - second_radius)
        ):
            return ()
        along = (
            first_radius ** 2 - second_radius ** 2 + baseline ** 2
        ) / (2.0 * baseline)
        height_squared = first_radius ** 2 - along ** 2
        if height_squared < -1e-9:
            return ()
        height = math.sqrt(max(0.0, height_squared))
        unit_x = dx / baseline
        unit_y = dy / baseline
        midpoint = (
            first_position[0] + along * unit_x,
            first_position[1] + along * unit_y,
        )
        negative = (
            midpoint[0] + height * unit_y,
            midpoint[1] - height * unit_x,
        )
        positive = (
            midpoint[0] - height * unit_y,
            midpoint[1] + height * unit_x,
        )
        return (("negative", negative),) if negative == positive else (
            ("negative", negative), ("positive", positive),
        )

    def expected_attempt_id(self, kind, estimate, reference_keys, branch=None):
        return json.dumps({
            "branch": branch,
            "estimate": [estimate[0].hex(), estimate[1].hex()],
            "kind": kind,
            "reference_keys": [list(key) for key in sorted(reference_keys)],
        }, sort_keys=True, separators=(",", ":"))

    def expected_multistart_ids(self, references, live_seed=None, private_seed=None):
        canonical = sorted(
            [(reference["key"], tuple(reference["position"]), reference["range"])
             for reference in references],
        )
        reference_keys = tuple(reference[0] for reference in canonical)
        origin = canonical[0]
        coefficients = []
        values = []
        for _, position, radius in canonical[1:]:
            coefficients.append([
                2.0 * (position[0] - origin[1][0]),
                2.0 * (position[1] - origin[1][1]),
            ])
            values.append(
                position[0] ** 2 + position[1] ** 2 - radius ** 2
                - (origin[1][0] ** 2 + origin[1][1] ** 2 - origin[2] ** 2)
            )
        algebraic, _, rank, _ = np.linalg.lstsq(
            np.asarray(coefficients), np.asarray(values), rcond=None,
        )
        starts = []
        if rank >= 2:
            starts.append(("algebraic", tuple(algebraic), reference_keys, None))
        for first, second in combinations(canonical, 2):
            pair_keys = tuple(sorted((first[0], second[0])))
            starts.extend(
                ("circle", estimate, pair_keys, branch)
                for branch, estimate in self.expected_circle_roots(first, second)
            )
        for kind, seed in (("live", live_seed), ("private", private_seed)):
            if seed is not None:
                starts.append((kind, tuple(seed), reference_keys, None))
        starts.sort(key=lambda start: self.expected_attempt_id(*start))
        unique = []
        for start in starts:
            if all(math.dist(start[1], prior[1]) > 1e-9 for prior in unique):
                unique.append(start)
        return tuple(self.expected_attempt_id(*start) for start in unique)

    def test_two_references_return_only_distinct_circle_branches(self):
        starts = enumerate_qualified_starts([
            self.reference(("base", 0), (0.0, 0.0), 5.0),
            self.reference(("base", 1), (6.0, 0.0), 5.0),
        ], live_seed={"estimate": (99.0, 99.0)}, private_seed={"estimate": (98.0, 98.0)})

        self.assertEqual({start.kind for start in starts}, {"circle"})
        self.assertEqual({start.branch for start in starts}, {"negative", "positive"})
        self.assertEqual(
            {start.estimate for start in starts},
            {(3.0, -4.0), (3.0, 4.0)},
        )
        self.assertEqual(
            tuple(stable_attempt_id(start) for start in starts),
            tuple(sorted(stable_attempt_id(start) for start in starts)),
        )

    def test_two_reference_degeneracies_have_only_distinct_analytic_roots(self):
        cases = {
            "tangent": ((0.0, 0.0), 1.0, (2.0, 0.0), 1.0, 1),
            "disjoint": ((0.0, 0.0), 1.0, (3.0, 0.0), 1.0, 0),
            "contained": ((0.0, 0.0), 5.0, (1.0, 0.0), 1.0, 0),
            "coincident": ((0.0, 0.0), 1.0, (0.0, 0.0), 1.0, 0),
        }
        for name, (first, first_radius, second, second_radius, expected_count) in cases.items():
            with self.subTest(name=name):
                starts = enumerate_qualified_starts([
                    self.reference(("base", 0), first, first_radius),
                    self.reference(("base", 1), second, second_radius),
                ], live_seed={"estimate": (9.0, 9.0)}, private_seed={"estimate": (8.0, 8.0)})
                self.assertEqual(len(starts), expected_count)
                self.assertTrue(all(start.kind == "circle" for start in starts))

    def test_two_reference_near_tangent_roots_are_not_nanometre_deduplicated(self):
        radius = 1e-6
        starts = enumerate_qualified_starts([
            self.reference(("base", 0), (0.0, 0.0), radius),
            self.reference(
                ("base", 1),
                (math.nextafter(2.0 * radius, 0.0), 0.0),
                radius,
            ),
        ], live_seed=None, private_seed=None)

        self.assertEqual(len(starts), 2)
        self.assertNotEqual(starts[0].estimate, starts[1].estimate)
        self.assertLess(
            math.dist(starts[0].estimate, starts[1].estimate),
            1e-9,
        )

    def test_three_reference_starts_include_all_pairs_algebraic_and_valid_seeds(self):
        references = [
            self.reference(("base", 2), (0.0, 8.0), 5.5),
            self.reference(("base", 0), (0.0, 0.0), 5.2),
            self.reference(("base", 1), (6.0, 0.0), 4.8),
        ]
        starts = enumerate_qualified_starts(
            references,
            live_seed={"estimate": (7.0, 8.0), "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]]},
            private_seed={"estimate": (8.0, 8.0), "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]]},
        )

        self.assertIn("algebraic", {start.kind for start in starts})
        self.assertIn("live", {start.kind for start in starts})
        self.assertIn("private", {start.kind for start in starts})
        circle_pairs = {start.reference_keys for start in starts if start.kind == "circle"}
        self.assertEqual(circle_pairs, {
            (("base", 0), ("base", 1)),
            (("base", 0), ("base", 2)),
            (("base", 1), ("base", 2)),
        })
        self.assertEqual(
            {stable_attempt_id(start) for start in starts},
            {stable_attempt_id(start) for start in enumerate_qualified_starts(
                list(reversed(references)),
                live_seed={"estimate": (7.0, 8.0), "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]]},
                private_seed={"estimate": (8.0, 8.0), "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]]},
            )},
        )

    def test_four_reference_enumeration_keeps_every_applicable_pair(self):
        starts = enumerate_qualified_starts([
            self.reference(("base", 0), (0.0, 0.0), 5.2),
            self.reference(("base", 1), (6.0, 0.0), 4.8),
            self.reference(("base", 2), (0.0, 8.0), 5.5),
            self.reference(("base", 3), (6.0, 8.0), 5.4),
        ], live_seed=None, private_seed=None)

        self.assertEqual(
            {start.reference_keys for start in starts if start.kind == "circle"},
            {
                (("base", 0), ("base", 1)),
                (("base", 0), ("base", 2)),
                (("base", 0), ("base", 3)),
                (("base", 1), ("base", 2)),
                (("base", 1), ("base", 3)),
                (("base", 2), ("base", 3)),
            },
        )

    def test_three_and_four_reference_starts_have_all_branches_and_ordered_ids(self):
        cases = (
            (
                [
                    self.reference(("base", 2), (0.0, 8.0), 5.5),
                    self.reference(("base", 0), (0.0, 0.0), 5.2),
                    self.reference(("base", 1), (6.0, 0.0), 4.8),
                ],
                (7.0, 8.0),
                (8.0, 8.0),
            ),
            (
                [
                    self.reference(("base", 0), (0.0, 0.0), 5.2),
                    self.reference(("base", 1), (6.0, 0.0), 4.8),
                    self.reference(("base", 2), (0.0, 8.0), 5.5),
                    self.reference(("base", 3), (6.0, 8.0), 5.4),
                ],
                None,
                None,
            ),
        )
        for references, live, private in cases:
            with self.subTest(reference_count=len(references)):
                covariance = [[1.0, 0.0], [0.0, 1.0]]
                starts = enumerate_qualified_starts(
                    references,
                    live_seed=None if live is None else {
                        "estimate": live, "modeled_covariance": covariance,
                    },
                    private_seed=None if private is None else {
                        "estimate": private, "modeled_covariance": covariance,
                    },
                )
                expected_ids = self.expected_multistart_ids(
                    references,
                    live_seed=live,
                    private_seed=private,
                )
                self.assertEqual(
                    tuple(stable_attempt_id(start) for start in starts),
                    expected_ids,
                )
                for pair in combinations(sorted(reference["key"] for reference in references), 2):
                    pair_starts = [
                        start for start in starts
                        if start.kind == "circle" and start.reference_keys == pair
                    ]
                    self.assertEqual(
                        tuple(start.branch for start in pair_starts),
                        ("negative", "positive"),
                    )
                permuted = [dict(reference, source_label="renamed")
                            for reference in reversed(references)]
                permuted_starts = enumerate_qualified_starts(
                    permuted,
                    live_seed=None if live is None else {
                        "estimate": live, "modeled_covariance": covariance,
                    },
                    private_seed=None if private is None else {
                        "estimate": private, "modeled_covariance": covariance,
                    },
                )
                self.assertEqual(
                    tuple(stable_attempt_id(start) for start in permuted_starts),
                    expected_ids,
                )

    def test_multistart_deduplication_has_an_exact_nanometre_boundary(self):
        references = [
            self.reference(("base", 0), (-1.0, 0.0), 1.0),
            self.reference(("base", 1), (1.0, 0.0), 1.0),
            self.reference(("base", 2), (0.0, 4.0), 4.0),
        ]
        pair = (("base", 0), (-1.0, 0.0), 1.0), (("base", 1), (1.0, 0.0), 1.0)
        _, root = self.expected_circle_roots(*pair)[0]
        baseline = enumerate_qualified_starts(references, live_seed=None, private_seed=None)
        covariance = [[1.0, 0.0], [0.0, 1.0]]
        within = enumerate_qualified_starts(
            references,
            live_seed={
                "estimate": (root[0] + 1e-9, root[1]),
                "modeled_covariance": covariance,
            },
            private_seed=None,
        )
        outside = enumerate_qualified_starts(
            references,
            live_seed={
                "estimate": (root[0] + math.nextafter(1e-9, math.inf), root[1]),
                "modeled_covariance": covariance,
            },
            private_seed=None,
        )
        self.assertEqual(len(within), len(baseline))
        self.assertEqual(len(outside), len(baseline) + 1)

    def test_seed_covariance_canonicalization_accepts_roundoff_and_rejects_invalid(self):
        references = [
            self.reference(("base", 0), (0.0, 0.0), 5.2),
            self.reference(("base", 1), (6.0, 0.0), 4.8),
            self.reference(("base", 2), (0.0, 8.0), 5.5),
        ]
        valid = {
            "estimate": (7.0, 8.0),
            "modeled_covariance": [[2.0, 0.1], [0.10000000000000002, 1.0]],
        }
        invalid_covariances = (
            [[1e-13, 1e-12], [0.0, 1e-13]],
            [[1.0, 0.0], [0.0, 0.0]],
            [[math.nan, 0.0], [0.0, 1.0]],
            None,
        )
        for kind in ("live", "private"):
            with self.subTest(kind=kind, case="roundoff"):
                starts = enumerate_qualified_starts(
                    references,
                    live_seed=valid if kind == "live" else None,
                    private_seed=valid if kind == "private" else None,
                )
                self.assertIn(kind, {start.kind for start in starts})
            for covariance in invalid_covariances:
                with self.subTest(kind=kind, covariance=covariance):
                    invalid = {"estimate": (7.0, 8.0)}
                    if covariance is not None:
                        invalid["modeled_covariance"] = covariance
                    starts = enumerate_qualified_starts(
                        references,
                        live_seed=invalid if kind == "live" else None,
                        private_seed=invalid if kind == "private" else None,
                    )
                    self.assertNotIn(kind, {start.kind for start in starts})
