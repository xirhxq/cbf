import json
import math
import tempfile
import unittest
from pathlib import Path

from scripts.diagnostics.analyze_diagnostic import analyze_run, minimum_linf_bound


UNAVAILABLE = "unavailable_no_distinct_truth_estimate_or_input_bounds"


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
