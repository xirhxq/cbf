import unittest
from dataclasses import replace
from hashlib import sha256
import json

from scripts.diagnostics.qualified_modes import (
    MODE_TOLERANCE_M,
    LocalCandidate,
    NumericalMode,
    QualifiedStart,
    canonical_mode_id,
    cluster_candidates,
    enumerate_qualified_starts,
    project_local_candidate,
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


class QualifiedStartEnumerationTests(unittest.TestCase):
    def reference(self, key, position, radius):
        return {"key": key, "position": position, "range": radius}

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
