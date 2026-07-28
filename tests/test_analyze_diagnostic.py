import json
import math
import tempfile
import unittest
from pathlib import Path

from scripts.diagnostics.analyze_diagnostic import analyze_run, minimum_linf_bound


UNAVAILABLE = "unavailable_no_distinct_truth_estimate_or_input_bounds"


def mc_first_two_frame_fixture():
    return {
        "para": {"gridWorld": {"xNum": 1, "yNum": 1}},
        "config": {
            "cbfs": {
                "uncertainty-rate": {
                    "mode": "backward-difference-positive"
                },
                "input-limits": {
                    "on": True,
                    "planar-component-max": 25.0,
                    "yaw-rate-max": 0.35,
                },
                "without-slack": {
                    "method": "all",
                    "safety": {
                        "mode": "pairwise",
                        "safe-distance": 2.0,
                        "consider-uncertainty": True,
                        "alpha": {"coe": 0.1, "pow": 1},
                    },
                    "comm-fixed": {
                        "alpha": {"coe": 0.1, "pow": 1}
                    },
                },
            }
        },
        "state": [
            {
                "runtime": 0.0,
                "formation": [],
                "update": [],
                "robots": [
                    {
                        "id": 1,
                        "state": {"x": 3.0, "y": 4.0},
                        "uncertainty": 1.0,
                        "uncertainty_rate": 0.0,
                        "opt": {
                            "status": "success",
                            "solver_info": {
                                "status": "optimal",
                                "solve_time_ms": 1.0,
                            },
                            "result": {
                                "vx": 25.0,
                                "vy": 0.0,
                                "yawRateRad": 0.35,
                            },
                            "input_limits": {
                                "enabled": True,
                                "planar_component_max": 25.0,
                                "yaw_rate_max": 0.35,
                                "bound_row_count": 6,
                                "saturation_tolerance": 1e-7,
                                "saturated": {
                                    "vx": True,
                                    "vy": False,
                                    "yawRateRad": True,
                                    "any": True,
                                },
                            },
                            "cbfNoSlack": [
                                {
                                    "name": "safetyCBF(#2)",
                                    "coe": {
                                        "vx": 0.0,
                                        "vy": 0.0,
                                        "yawRateRad": 0.0,
                                    },
                                    "const": 1.0,
                                    "residual": 1.0,
                                    "alpha_coe": 0.1,
                                    "alpha_pow": 1,
                                },
                                {
                                    "name": "diagnosticRequiredVx",
                                    "coe": {
                                        "vx": 1.0,
                                        "vy": 0.0,
                                        "yawRateRad": 0.0,
                                    },
                                    "const": -2.0,
                                    "residual": 23.0,
                                    "alpha_coe": 0.1,
                                    "alpha_pow": 1,
                                },
                            ],
                        },
                    },
                    {
                        "id": 2,
                        "state": {"x": 0.0, "y": 0.0},
                        "uncertainty": 2.0,
                        "uncertainty_rate": 0.0,
                        "opt": {
                            "status": "success",
                            "solver_info": {
                                "status": "optimal",
                                "solve_time_ms": 1.0,
                            },
                            "result": {
                                "vx": 2.0,
                                "vy": -1.0,
                                "yawRateRad": 0.0,
                            },
                            "input_limits": {
                                "enabled": True,
                                "planar_component_max": 25.0,
                                "yaw_rate_max": 0.35,
                                "bound_row_count": 6,
                                "saturation_tolerance": 1e-7,
                                "saturated": {
                                    "vx": False,
                                    "vy": False,
                                    "yawRateRad": False,
                                    "any": False,
                                },
                            },
                            "cbfNoSlack": [
                                {
                                    "name": "safetyCBF(#1)",
                                    "coe": {
                                        "vx": 0.0,
                                        "vy": 0.0,
                                        "yawRateRad": 0.0,
                                    },
                                    "const": 1.0,
                                    "residual": 1.0,
                                    "alpha_coe": 0.1,
                                    "alpha_pow": 1,
                                },
                                {
                                    "name": "diagnosticRequiredVx",
                                    "coe": {
                                        "vx": 1.0,
                                        "vy": 0.0,
                                        "yawRateRad": 0.0,
                                    },
                                    "const": -2.0,
                                    "residual": 0.0,
                                    "alpha_coe": 0.1,
                                    "alpha_pow": 1,
                                },
                            ],
                        },
                    },
                ],
            },
            {
                "runtime": 0.5,
                "formation": [],
                "update": [],
                "robots": [
                    {
                        "id": 1,
                        "state": {"x": 3.0, "y": 4.0},
                        "uncertainty": 1.75,
                        "uncertainty_rate": 1.5,
                        "opt": {
                            "status": "success",
                            "solver_info": {
                                "status": "optimal",
                                "solve_time_ms": 1.0,
                            },
                            "result": {
                                "vx": 24.0,
                                "vy": 0.1,
                                "yawRateRad": 0.0,
                            },
                            "input_limits": {
                                "enabled": True,
                                "planar_component_max": 25.0,
                                "yaw_rate_max": 0.35,
                                "bound_row_count": 6,
                                "saturation_tolerance": 1e-7,
                                "saturated": {
                                    "vx": False,
                                    "vy": False,
                                    "yawRateRad": False,
                                    "any": False,
                                },
                            },
                            "cbfNoSlack": [
                                {
                                    "name": "safetyCBF(#2)",
                                    "coe": {
                                        "vx": 0.0,
                                        "vy": 0.0,
                                        "yawRateRad": 0.0,
                                    },
                                    "const": 1.0,
                                    "residual": 1.0,
                                    "alpha_coe": 0.1,
                                    "alpha_pow": 1,
                                },
                                {
                                    "name": "diagnosticRequiredVx",
                                    "coe": {
                                        "vx": 1.0,
                                        "vy": 0.0,
                                        "yawRateRad": 0.0,
                                    },
                                    "const": -2.0,
                                    "residual": 22.0,
                                    "alpha_coe": 0.1,
                                    "alpha_pow": 1,
                                },
                            ],
                        },
                    },
                    {
                        "id": 2,
                        "state": {"x": 0.0, "y": 0.0},
                        "uncertainty": 1.5,
                        "uncertainty_rate": 0.0,
                        "opt": {
                            "status": "success",
                            "solver_info": {
                                "status": "optimal",
                                "solve_time_ms": 1.0,
                            },
                            "result": {
                                "vx": 5.0,
                                "vy": 3.0,
                                "yawRateRad": 0.0,
                            },
                            "input_limits": {
                                "enabled": True,
                                "planar_component_max": 25.0,
                                "yaw_rate_max": 0.35,
                                "bound_row_count": 6,
                                "saturation_tolerance": 1e-7,
                                "saturated": {
                                    "vx": False,
                                    "vy": False,
                                    "yawRateRad": False,
                                    "any": False,
                                },
                            },
                            "cbfNoSlack": [
                                {
                                    "name": "safetyCBF(#1)",
                                    "coe": {
                                        "vx": 0.0,
                                        "vy": 0.0,
                                        "yawRateRad": 0.0,
                                    },
                                    "const": 1.0,
                                    "residual": 1.0,
                                    "alpha_coe": 0.1,
                                    "alpha_pow": 1,
                                },
                                {
                                    "name": "diagnosticRequiredVx",
                                    "coe": {
                                        "vx": 1.0,
                                        "vy": 0.0,
                                        "yawRateRad": 0.0,
                                    },
                                    "const": -2.0,
                                    "residual": 3.0,
                                    "alpha_coe": 0.1,
                                    "alpha_pow": 1,
                                },
                            ],
                        },
                    },
                ],
            },
        ],
    }


class FeasibilityBoundTests(unittest.TestCase):
    def test_single_axis_lower_bound(self):
        constraints = [{"coe": {"vx": 1.0, "vy": 0.0, "yawRateRad": 0.0}, "rhs": 2.0}]
        self.assertAlmostEqual(minimum_linf_bound(constraints, ["vx", "vy", "yawRateRad"]), 2.0)

    def test_diagonal_constraint(self):
        constraints = [{"coe": {"vx": 1.0, "vy": 1.0, "yawRateRad": 0.0}, "rhs": 2.0}]
        self.assertAlmostEqual(minimum_linf_bound(constraints, ["vx", "vy", "yawRateRad"]), 1.0)

    def test_conflicting_constraints_are_infeasible(self):
        constraints = [
            {"coe": {"vx": 1.0}, "rhs": 1.0},
            {"coe": {"vx": -1.0}, "rhs": 1.0},
        ]
        self.assertTrue(math.isinf(minimum_linf_bound(constraints, ["vx"])))

    def test_no_hard_constraints_need_no_control_bound(self):
        self.assertEqual(minimum_linf_bound([], ["vx", "vy", "yawRateRad"]), 0.0)


class RunAnalysisTests(unittest.TestCase):
    def analyze_fixture(self, data):
        with tempfile.TemporaryDirectory() as temporary_directory:
            run_dir = Path(temporary_directory)
            data_path = run_dir / "data.json"
            manifest_path = run_dir / "manifest.json"
            data_path.write_text(json.dumps(data))
            manifest_path.write_text(json.dumps({"case": "C1"}))
            return analyze_run(data_path, manifest_path)

    def test_covariance_information_set_transitions_and_reference_mismatches_are_reported(self):
        data = {
            "para": {"gridWorld": {"xNum": 1, "yNum": 1}},
            "config": {},
            "state": [
                {
                    "runtime": 0.0,
                    "formation": [
                        {"id": 1, "anchorIds": [], "baseIds": [0]},
                        {"id": 2, "anchorIds": [1], "baseIds": []},
                    ],
                    "covariance_formation": [
                        {"id": 1, "anchorIds": [], "baseIds": [0]},
                        {"id": 2, "anchorIds": [1], "baseIds": []},
                    ],
                    "update": [],
                    "robots": [],
                },
                {
                    "runtime": 0.5,
                    "formation": [
                        {"id": 1, "anchorIds": [], "baseIds": [0]},
                        {"id": 2, "anchorIds": [1], "baseIds": []},
                    ],
                    "covariance_formation": [
                        {"id": 1, "anchorIds": [], "baseIds": [0]},
                        {"id": 2, "anchorIds": [1, 3], "baseIds": []},
                    ],
                    "update": [],
                    "robots": [],
                },
            ],
        }

        summary = self.analyze_fixture(data)
        audit = summary["covariance_information_set"]
        self.assertEqual(audit["status"], "available")
        self.assertEqual(audit["robot_frame_record_count"], 4)
        self.assertEqual(audit["transition_count"], 1)
        self.assertEqual(audit["required_reference_mismatch_count"], 1)
        self.assertEqual(audit["transitions"][0]["robot_id"], 2)
        self.assertEqual(audit["transitions"][0]["frame_index"], 1)

    def test_mc_first_two_frame_metrics_match_hand_derived_literals(self):
        summary = self.analyze_fixture(mc_first_two_frame_fixture())

        self.assertEqual(
            summary["uncertainty_jumps"],
            {
                "positive_count": 1,
                "maximum": 0.75,
                "source": "consecutive_logged_scalar_uncertainty",
            },
        )
        self.assertEqual(
            summary["uncertainty"]["logged_rate"],
            {
                "status": "available",
                "count": 4,
                "max": 1.5,
                "missing_count": 0,
                "nonfinite_count": 0,
                "source": "robot.uncertainty_rate",
            },
        )

        mismatch = summary["control_mismatch"]
        self.assertEqual(mismatch["status"], "available")
        self.assertEqual(mismatch["count"], 2)
        self.assertAlmostEqual(mismatch["maximum"], 5.0)
        self.assertEqual(
            [
                (
                    record["frame_index"],
                    record["owner_id"],
                    record["neighbor_id"],
                    record["projected_mismatch"],
                )
                for record in mismatch["records"]
            ],
            [(1, 1, 2, 5.0), (1, 2, 1, 0.52)],
        )

        coverage = summary["pairwise_row_coverage"]
        self.assertEqual(coverage["status"], "available")
        self.assertEqual(coverage["mode"], "pairwise")
        self.assertEqual(coverage["expected_count"], 4)
        self.assertEqual(coverage["observed_count"], 4)
        self.assertEqual(coverage["missing_count"], 0)
        self.assertEqual(coverage["unexpected_count"], 0)
        self.assertEqual(coverage["missing_rows"], [])
        self.assertEqual(coverage["unexpected_rows"], [])
        self.assertTrue(coverage["complete"])

        practical = summary["practical_tolerance"]["collision"]
        self.assertEqual(
            summary["practical_tolerance"]["negative_tolerance_m"],
            -0.5,
        )
        self.assertAlmostEqual(practical["strict_nominal_min"], 3.0)
        self.assertAlmostEqual(practical["strict_tightened_min"], -0.25)
        self.assertEqual(practical["below_negative_tolerance_count"], 0)
        self.assertEqual(practical["negative_observation_count"], 1)
        self.assertEqual(practical["isolated_tolerance_event_count"], 1)
        self.assertFalse(practical["has_consecutive_negative_frames"])
        self.assertEqual(practical["classification"], "practical_tolerance_event")

        limits = summary["input_limits"]
        self.assertEqual(limits["status"], "enabled")
        self.assertEqual(limits["planar_component_max"], 25.0)
        self.assertEqual(limits["yaw_rate_max"], 0.35)
        self.assertEqual(
            limits["observed_saturation_counts"],
            {
                "vx": 1,
                "vy": 0,
                "yawRateRad": 1,
                "planar_any": 1,
                "any": 1,
            },
        )
        self.assertEqual(
            limits["logged_saturation_counts"],
            {
                "vx": 1,
                "vy": 0,
                "yawRateRad": 1,
                "any": 1,
            },
        )
        self.assertEqual(
            limits["violation_counts"],
            {"vx": 0, "vy": 0, "yawRateRad": 0, "total": 0},
        )
        self.assertAlmostEqual(limits["minimum_required_linf"], 2.0)
        self.assertAlmostEqual(
            limits["bounded_feasibility_headroom"],
            23.0,
        )
        self.assertEqual(limits["applicable_optimal_record_count"], 4)
        self.assertEqual(limits["validated_record_count"], 4)
        self.assertEqual(limits["invalid_record_count"], 0)
        self.assertNotIn("saturation", summary["unavailable_metrics"])
        self.assertEqual(summary["finite_value_failures"]["count"], 0)

    def test_control_mismatch_includes_fixed_communication_neighbor_rows(self):
        data = mc_first_two_frame_fixture()
        for frame in data["state"]:
            frame["robots"][0]["opt"]["cbfNoSlack"][0]["name"] = (
                "fixedCommCBF(#2)"
            )

        mismatch = self.analyze_fixture(data)["control_mismatch"]

        self.assertEqual(mismatch["count"], 2)
        self.assertAlmostEqual(mismatch["maximum"], 5.0)
        owner_record = next(
            record for record in mismatch["records"]
            if record["owner_id"] == 1
        )
        self.assertEqual(owner_record["neighbor_id"], 2)
        self.assertAlmostEqual(owner_record["projected_mismatch"], 5.0)
        self.assertEqual(owner_record["sources"], ["fixed_communication"])
        self.assertEqual(owner_record["source_rows"], ["fixedCommCBF(#2)"])

    def test_control_mismatch_deduplicates_safety_and_communication_rows(self):
        data = mc_first_two_frame_fixture()
        for frame in data["state"]:
            row = frame["robots"][0]["opt"]["cbfNoSlack"][0]
            frame["robots"][0]["opt"]["cbfNoSlack"].append(
                {
                    **row,
                    "name": "fixedCommCBF(#2)",
                }
            )

        mismatch = self.analyze_fixture(data)["control_mismatch"]

        self.assertEqual(mismatch["count"], 2)
        owner_record = next(
            record for record in mismatch["records"]
            if record["owner_id"] == 1
        )
        self.assertEqual(
            owner_record["sources"],
            ["fixed_communication", "safety"],
        )
        self.assertEqual(
            owner_record["source_rows"],
            ["fixedCommCBF(#2)", "safetyCBF(#2)"],
        )

    def test_input_limit_violations_use_the_declared_one_e_minus_seven_tolerance(self):
        data = mc_first_two_frame_fixture()
        result = data["state"][1]["robots"][1]["opt"]["result"]
        result["vx"] = 25.0000002
        result["vy"] = -25.0000002
        result["yawRateRad"] = 0.3500002
        data["state"][1]["robots"][1]["opt"]["input_limits"]["saturated"] = {
            "vx": True,
            "vy": True,
            "yawRateRad": True,
            "any": True,
        }

        limits = self.analyze_fixture(data)["input_limits"]

        self.assertEqual(
            limits["violation_counts"],
            {"vx": 1, "vy": 1, "yawRateRad": 1, "total": 3},
        )

    def test_bounded_input_evidence_rejects_missing_or_inconsistent_opt_metadata(self):
        cases = (
            ("missing", "missing"),
            ("disabled", False),
            ("wrong_row_count", 5),
            ("wrong_planar_limit", 24.0),
            ("wrong_tolerance", 1e-6),
        )
        for name, bad_value in cases:
            with self.subTest(name=name):
                data = mc_first_two_frame_fixture()
                opt = data["state"][0]["robots"][0]["opt"]
                if name == "missing":
                    opt.pop("input_limits")
                elif name == "disabled":
                    opt["input_limits"]["enabled"] = bad_value
                elif name == "wrong_row_count":
                    opt["input_limits"]["bound_row_count"] = bad_value
                elif name == "wrong_planar_limit":
                    opt["input_limits"]["planar_component_max"] = bad_value
                else:
                    opt["input_limits"]["saturation_tolerance"] = bad_value

                summary = self.analyze_fixture(data)
                limits = summary["input_limits"]

                self.assertEqual(limits["status"], "invalid")
                self.assertEqual(limits["applicable_optimal_record_count"], 4)
                self.assertEqual(limits["validated_record_count"], 3)
                self.assertEqual(limits["invalid_record_count"], 1)
                self.assertIsNone(limits["observed_saturation_counts"])
                self.assertIsNone(limits["logged_saturation_counts"])
                self.assertIsNone(limits["violation_counts"])
                self.assertIsNone(limits["minimum_required_linf"])
                self.assertIsNone(limits["bounded_feasibility_headroom"])
                self.assertIn("saturation", summary["unavailable_metrics"])

    def test_all_nonoptimal_frames_have_unavailable_bound_and_saturation_evidence(self):
        data = mc_first_two_frame_fixture()
        for frame in data["state"]:
            for robot in frame["robots"]:
                robot["opt"]["status"] = "failed"
                robot["opt"]["solver_info"]["status"] = "infeasible"

        summary = self.analyze_fixture(data)
        limits = summary["input_limits"]

        self.assertEqual(
            summary["minimum_linf_bound"]["per_frame"],
            [None, None],
        )
        self.assertIsNone(summary["minimum_linf_bound"]["maximum"])
        self.assertEqual(limits["status"], "unavailable")
        self.assertEqual(limits["applicable_optimal_record_count"], 0)
        self.assertEqual(limits["validated_record_count"], 0)
        self.assertEqual(limits["invalid_record_count"], 0)
        self.assertIsNone(limits["observed_saturation_counts"])
        self.assertIsNone(limits["logged_saturation_counts"])
        self.assertIsNone(limits["violation_counts"])
        self.assertIsNone(limits["minimum_required_linf"])
        self.assertIsNone(limits["bounded_feasibility_headroom"])
        self.assertIn("saturation", summary["unavailable_metrics"])

    def test_exact_negative_half_meter_is_an_event_not_a_below_tolerance_failure(self):
        data = mc_first_two_frame_fixture()
        data["state"][1]["robots"][0]["uncertainty"] = 2.0

        collision = self.analyze_fixture(data)["practical_tolerance"]["collision"]

        self.assertAlmostEqual(collision["strict_tightened_min"], -0.5)
        self.assertEqual(collision["below_negative_tolerance_count"], 0)
        self.assertEqual(collision["classification"], "practical_tolerance_event")

    def test_consecutive_negative_detection_is_tracked_for_the_same_physical_pair(self):
        data = mc_first_two_frame_fixture()
        data["state"][0]["robots"][0]["uncertainty"] = 1.1

        collision = self.analyze_fixture(data)["practical_tolerance"]["collision"]

        self.assertTrue(collision["has_consecutive_negative_frames"])
        self.assertEqual(collision["consecutive_negative_keys"], ["robots:1-2"])
        self.assertEqual(collision["classification"], "failed_consecutive_negative")

    def test_nonfinite_direct_logged_uncertainty_rate_is_an_input_failure(self):
        data = mc_first_two_frame_fixture()
        data["state"][1]["robots"][0]["uncertainty_rate"] = float("nan")

        summary = self.analyze_fixture(data)

        self.assertEqual(
            summary["uncertainty"]["logged_rate"],
            {
                "status": "available_with_nonfinite_values",
                "count": 3,
                "max": 0.0,
                "missing_count": 0,
                "nonfinite_count": 1,
                "source": "robot.uncertainty_rate",
            },
        )
        self.assertIn(
            "state[1].robots[0].uncertainty_rate",
            summary["finite_value_failures"]["paths"],
        )

    def test_uncertainty_jump_does_not_bridge_a_missing_robot_frame(self):
        data = mc_first_two_frame_fixture()
        data["state"].insert(
            1,
            {
                "runtime": 0.25,
                "formation": [],
                "update": [],
                "robots": [],
            },
        )

        jumps = self.analyze_fixture(data)["uncertainty_jumps"]

        self.assertEqual(jumps["positive_count"], 0)
        self.assertIsNone(jumps["maximum"])

    def test_failed_qp_placeholder_is_not_reported_as_applied_control_evidence(self):
        data = {
            "para": {"gridWorld": {"xNum": 1, "yNum": 1}},
            "config": {},
            "state": [
                {
                    "runtime": 0.0,
                    "formation": [],
                    "update": [],
                    "robots": [
                        {
                            "id": 1,
                            "state": {"x": 0.0, "y": 0.0},
                            "uncertainty": 0.0,
                            "opt": {
                                "status": "success",
                                "solver_info": {"status": "optimal", "solve_time_ms": 1.0},
                                "result": {"vx": 1.0},
                                "cbfNoSlack": [
                                    {"name": "safetyCBF", "coe": {"vx": 1.0}, "const": -1.0, "residual": 0.0}
                                ],
                            },
                        },
                        {
                            "id": 2,
                            "state": {"x": 10.0, "y": 0.0},
                            "uncertainty": 0.0,
                            "opt": {
                                "status": "failed",
                                "solver_info": {"status": "infeasible", "solve_time_ms": 2.0},
                                "result": {"vx": 0.0},
                                "cbfNoSlack": [
                                    {"name": "safetyCBF", "coe": {"vx": 1.0}, "const": -2.0}
                                ],
                            },
                        },
                    ],
                }
            ],
        }

        summary = self.analyze_fixture(data)

        self.assertEqual(summary["hard_constraints"]["count"], 1)
        self.assertEqual(summary["hard_constraints"]["negative_count"], 0)
        self.assertAlmostEqual(summary["hard_constraints"]["residual_min"], 0.0)
        self.assertEqual(summary["hard_constraints"]["logged_residual_missing_count"], 0)
        self.assertEqual(summary["control_norms"]["l2"]["count"], 1)
        self.assertEqual(summary["minimum_linf_bound"]["per_frame"], [1.0])
        self.assertEqual(summary["unapplied_solver_records"]["count"], 1)
        self.assertEqual(summary["unapplied_solver_records"]["failed_solver_record_count"], 1)
        self.assertEqual(summary["unapplied_solver_records"]["unconfirmed_record_count"], 0)
        self.assertEqual(summary["unapplied_solver_records"]["status_counts"], {"infeasible": 1})
        self.assertEqual(summary["unapplied_solver_records"]["hard_constraint_count"], 1)
        self.assertEqual(summary["unapplied_solver_records"]["result_placeholder_count"], 1)
        self.assertEqual(summary["unapplied_solver_records"]["minimum_linf_bound"]["per_frame"], [2.0])
        self.assertEqual(
            summary["unapplied_solver_records"]["interpretation"],
            (
                "unconfirmed records are excluded from applied evidence; "
                "solver failures are caught and logged without a state step, "
                "so their result placeholders were not applied"
            ),
        )

    def test_min_mode_surfaces_identity_alpha_and_contained_comm_auto_configuration(self):
        data = {
            "para": {"gridWorld": {"xNum": 1, "yNum": 1}},
            "config": {
                "cbfs": {
                    "without-slack": {
                        "method": "min",
                        "comm-fixed": {"alpha": {"coe": 0.1, "pow": 3}},
                        "comm-auto": {"alpha": {"coe": 0.2, "pow": 1}},
                        "safety": {"alpha": {"coe": 0.3, "pow": 3}},
                    }
                }
            },
            "state": [
                {
                    "runtime": 0.0,
                    "formation": [],
                    "update": [],
                    "robots": [
                        {
                            "id": 1,
                            "state": {"x": 0.0, "y": 0.0},
                            "uncertainty": 0.0,
                            "opt": {
                                "status": "success",
                                "solver_info": {"status": "optimal"},
                                "result": {"vx": 0.0},
                                "cbfNoSlack": [
                                    {
                                        "name": "min(fixedCommCBF(base-0),commCBF,safetyCBF)",
                                        "coe": {"vx": 1.0},
                                        "const": 0.0,
                                        "residual": 0.0,
                                        "alpha_coe": 1.0,
                                        "alpha_pow": 1,
                                    }
                                ],
                            },
                        }
                    ],
                }
            ],
        }

        summary = self.analyze_fixture(data)

        audit = summary["class_k_parameters"]
        self.assertEqual(audit["not_directly_comparable_count"], 1)
        self.assertEqual(audit["unmapped_count"], 0)
        aggregate = audit["records"][0]
        self.assertEqual(aggregate["source"], "MultiCBF.identity_alpha")
        self.assertEqual(
            aggregate["comparison"],
            "intentional_semantic_difference_not_directly_comparable",
        )
        self.assertEqual(
            aggregate["instantiated"],
            {"coefficient": 1.0, "power": 1},
        )
        contained = {item["name"]: item for item in aggregate["contained_configured_parameters"]}
        self.assertEqual(
            contained["commCBF"]["config_source"],
            "config.cbfs.without-slack.comm-auto.alpha",
        )
        self.assertEqual(
            contained["commCBF"]["configured"],
            {"coefficient": 0.2, "power": 1},
        )

    def test_covariance_uncertainty_inconsistency_is_reported_for_each_configured_mapping(self):
        cases = [
            ("max_eigenvalue", {"cov_xx": 4.0, "cov_xy": 0.0, "cov_yx": 0.0, "cov_yy": 1.0}, 5.0, 6.0, 1.0),
            ("std_avg", {"cov_xx": 4.0, "cov_xy": 0.0, "cov_yx": 0.0, "cov_yy": 9.0}, 2.0, 2.5, 0.5),
        ]
        for uncertainty_type, covariance, logged, derived, error in cases:
            with self.subTest(uncertainty_type=uncertainty_type):
                data = {
                    "para": {"gridWorld": {"xNum": 1, "yNum": 1}},
                    "config": {
                        "position_covariance": {
                            "enable": True,
                            "uncertainty-type": uncertainty_type,
                        }
                    },
                    "state": [
                        {
                            "runtime": 0.0,
                            "formation": [],
                            "update": [],
                            "robots": [
                                {
                                    "id": 1,
                                    "state": {"x": 0.0, "y": 0.0},
                                    "position_covariance": covariance,
                                    "uncertainty": logged,
                                    "opt": {
                                        "status": "success",
                                        "solver_info": {"status": "optimal"},
                                        "result": {"vx": 0.0},
                                        "cbfNoSlack": [],
                                    },
                                }
                            ],
                        }
                    ],
                }

                summary = self.analyze_fixture(data)

                consistency = summary["covariance_uncertainty_consistency"]
                self.assertEqual(consistency["status"], "available")
                self.assertEqual(consistency["configured_type"], uncertainty_type)
                self.assertEqual(consistency["checked_count"], 1)
                self.assertEqual(consistency["mismatch_count"], 1)
                self.assertAlmostEqual(consistency["max_absolute_error"], error)
                self.assertAlmostEqual(consistency["records"][0]["derived_uncertainty"], derived)

    def test_analyzer_reports_an_infeasible_frame_without_writing_nonstandard_json(self):
        data = {
            "para": {"gridWorld": {"xNum": 1, "yNum": 1}},
            "config": {},
            "state": [
                {
                    "runtime": 0.0,
                    "formation": [],
                    "update": [],
                    "robots": [
                        {
                            "id": 1,
                            "state": {"x": 0.0, "y": 0.0},
                            "uncertainty": 0.0,
                            "opt": {
                                "status": "success",
                                "solver_info": {"status": "optimal"},
                                "result": {"vx": 0.0},
                                "cbfNoSlack": [
                                    {"coe": {"vx": 1.0}, "const": -1.0},
                                    {"coe": {"vx": -1.0}, "const": -1.0},
                                ],
                            },
                        }
                    ],
                }
            ],
        }
        with tempfile.TemporaryDirectory() as temporary_directory:
            run_dir = Path(temporary_directory)
            data_path = run_dir / "data.json"
            manifest_path = run_dir / "manifest.json"
            data_path.write_text(json.dumps(data))
            manifest_path.write_text(json.dumps({"case": "C1"}))

            summary = analyze_run(data_path, manifest_path)

            self.assertEqual(summary["minimum_linf_bound"]["per_frame"], [None])
            self.assertEqual(summary["minimum_linf_bound"]["infeasible_frame_count"], 1)
            self.assertEqual(json.loads((run_dir / "diagnostic-summary.json").read_text())["minimum_linf_bound"]["per_frame"], [None])

    def test_analyzer_uses_negative_const_as_half_space_rhs_and_writes_summary(self):
        data = {
            "para": {"gridWorld": {"xNum": 2, "yNum": 2}},
            "config": {
                "bases": [[0.0, 0.0]],
                "cbfs": {
                    "without-slack": {
                        "comm-fixed": {"max-range": 10.0, "k": 1.0, "consider-uncertainty": True},
                        "safety": {"safe-distance": 2.0, "k": 1.0, "consider-uncertainty": True},
                    }
                },
            },
            "state": [
                {
                    "runtime": 0.0,
                    "formation": [
                        {"id": 1, "anchorIds": [], "baseIds": [0]},
                        {"id": 2, "anchorIds": [], "baseIds": []},
                    ],
                    "update": [[0, 0]],
                    "robots": [
                        {
                            "id": 1,
                            "state": {"x": 3.0, "y": 4.0},
                            "position_covariance": {"cov_xx": 1.0, "cov_xy": 0.0, "cov_yx": 0.0, "cov_yy": 1.0},
                            "uncertainty": 1.0,
                            "opt": {
                                "status": "success",
                                "solver_info": {"status": "optimal", "solve_time_ms": 2.0},
                                "result": {"vx": 2.0, "vy": 0.0, "yawRateRad": 0.0},
                                "cbfNoSlack": [
                                    {
                                        "name": "fixedCommCBF(base-0)",
                                        "coe": {"vx": 1.0, "vy": 0.0, "yawRateRad": 0.0},
                                        "const": -2.0,
                                        "residual": 0.0,
                                    }
                                ],
                            },
                        },
                        {
                            "id": 2,
                            "state": {"x": 3.0, "y": 8.0},
                            "position_covariance": {"cov_xx": 1.0, "cov_xy": 0.0, "cov_yx": 0.0, "cov_yy": 1.0},
                            "uncertainty": 0.0,
                            "opt": {
                                "status": "success",
                                "solver_info": {"status": "optimal", "solve_time_ms": 4.0},
                                "result": {"vx": 0.0, "vy": 0.0, "yawRateRad": 0.0},
                                "cbfNoSlack": [],
                            },
                        },
                    ],
                },
                {
                    "runtime": 1.0,
                    "formation": [
                        {"id": 1, "anchorIds": [], "baseIds": [0]},
                        {"id": 2, "anchorIds": [], "baseIds": []},
                    ],
                    "update": [[1, 0]],
                    "robots": [
                        {
                            "id": 1,
                            "state": {"x": 3.0, "y": 4.0},
                            "position_covariance": {"cov_xx": 4.0, "cov_xy": 0.0, "cov_yx": 0.0, "cov_yy": 4.0},
                            "uncertainty": 2.0,
                            "opt": {
                                "status": "success",
                                "solver_info": {"status": "optimal", "solve_time_ms": 6.0},
                                "result": {"vx": 1.0, "vy": 0.0, "yawRateRad": 0.0},
                                "cbfNoSlack": [
                                    {
                                        "name": "fixedCommCBF(base-0)",
                                        "coe": {"vx": 1.0, "vy": 0.0, "yawRateRad": 0.0},
                                        "const": -2.0,
                                        "residual": -1.0,
                                    }
                                ],
                            },
                        },
                        {
                            "id": 2,
                            "state": {"x": 3.0, "y": 8.0},
                            "position_covariance": {"cov_xx": 1.0, "cov_xy": 0.0, "cov_yx": 0.0, "cov_yy": 1.0},
                            "uncertainty": 0.0,
                            "opt": {
                                "status": "success",
                                "solver_info": {"status": "optimal", "solve_time_ms": 8.0},
                                "result": {"vx": 0.0, "vy": 0.0, "yawRateRad": 0.0},
                                "cbfNoSlack": [],
                            },
                        },
                    ],
                },
                {
                    "runtime": 2.0,
                    "formation": [],
                    "update": [],
                    "robots": [],
                },
            ],
        }
        manifest = {"case": "C1", "seed": 20260727, "termination_reason": "completed"}

        with tempfile.TemporaryDirectory() as temporary_directory:
            run_dir = Path(temporary_directory)
            data_path = run_dir / "data.json"
            manifest_path = run_dir / "manifest.json"
            data_path.write_text(json.dumps(data))
            manifest_path.write_text(json.dumps(manifest))

            summary = analyze_run(data_path, manifest_path)

            self.assertEqual(summary["solver_status_counts"], {"optimal": 4})
            self.assertEqual(summary["hard_constraints"]["count"], 2)
            self.assertEqual(summary["hard_constraints"]["negative_count"], 1)
            self.assertAlmostEqual(summary["hard_constraints"]["residual_min"], -1.0)
            self.assertEqual(summary["minimum_linf_bound"]["per_frame"], [2.0, 2.0, 0.0])
            self.assertAlmostEqual(summary["minimum_linf_bound"]["maximum"], 2.0)
            self.assertAlmostEqual(summary["localization_margins"]["required_reference"]["nominal_min"], 5.0)
            self.assertAlmostEqual(summary["localization_margins"]["required_reference"]["tightened_min"], 3.0)
            self.assertAlmostEqual(summary["collision_margins"]["all_pairs"]["nominal_min"], 2.0)
            self.assertAlmostEqual(summary["collision_margins"]["all_pairs"]["tightened_min"], 0.0)
            self.assertAlmostEqual(summary["coverage"]["fraction"], 0.5)
            self.assertAlmostEqual(summary["uncertainty"]["max"], 2.0)
            self.assertAlmostEqual(summary["uncertainty"]["max_positive_one_step_rate"], 1.0)
            self.assertEqual(set(summary["unavailable_metrics"].values()), {UNAVAILABLE})
            self.assertTrue((run_dir / "diagnostic-summary.json").is_file())
            self.assertTrue((run_dir / "diagnostic-summary.md").is_file())
            self.assertEqual(
                (run_dir / "diagnostic-summary.md").read_text().count("```"),
                2,
            )


if __name__ == "__main__":
    unittest.main()
