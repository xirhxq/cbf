import unittest
from dataclasses import replace
from hashlib import sha256
import json
import math
from itertools import combinations

import numpy as np

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
