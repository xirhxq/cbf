import csv
import json
import math
import pathlib
import sys
import tempfile
import unittest


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from bridge_experiments.run_full_simulator_bridge import build_row_config, write_row_configs
from bridge_experiments.extract_full_simulator_bridge import extract_bridge_metrics, write_summary
from bridge_experiments.plot_full_simulator_bridge_maps import (
    _denial_rectangle_bounds,
    _row_display_label,
    _robot_trajectories,
    write_bridge_search_map_figure,
)
from bridge_experiments.plot_hocbf_feasibility import (
    feasible_acceleration_polygon,
    terminal_hocbf_panels,
    write_hocbf_feasibility_figure,
)


def make_bridge_frame(runtime: float, detected: bool = False) -> dict:
    return {
        "runtime": runtime,
        "bridge": {
            "search": {
                "coverage_ratio": 0.2 + runtime * 0.01,
                "belief_entropy": 3.0 - runtime * 0.1,
                "belief_at_target": 0.05 + runtime * 0.01,
                "detected": detected,
                "detection_time_s": runtime if detected else -1.0,
            },
            "topology": {
                "relay_active": runtime > 0.0,
                "accepted_switches": 1 if runtime > 0.0 else 0,
                "rejected_candidates": 1,
                "min_robust_margin": 15.0,
                "min_fim_eigenvalue": 0.2,
            },
        },
        "robots": [
            {
                "id": 1,
                "uncertainty": 4.0,
                "opt": {
                    "status": "success",
                    "result": {"ax": 0.5, "ay": 0.0, "yawRateRad": 0.1},
                    "slacks": [0.0],
                    "hocbfNoSlack": [
                        {"hocbf": 0.3, "h": 0.6, "hdot": -0.2, "psi1": 0.4}
                    ],
                    "solver_info": {"status": "optimal"},
                },
                "cbfNoSlack": {
                    "secondOrderSafetyCBF(#2)": 20.0,
                    "secondOrderFixedCommCBF(#2)": 15.0,
                },
            }
        ],
    }


def make_bridge_map_data() -> dict:
    return {
        "config": {
            "world": {
                "boundary": [[0.0, 0.0], [200.0, 0.0], [200.0, 200.0], [0.0, 200.0]],
                "spacing": 100.0,
            },
            "bridge": {
                "topology": {
                    "denial-center": [100.0, 120.0],
                    "denial-half-size": [20.0, 30.0],
                }
            },
            "bases": [[0.0, 0.0]],
            "cbfs": {
                "without-slack": {
                    "comm-fixed": {
                        "on": True,
                        "max-range": 10.0,
                    }
                }
            },
        },
        "bridge": {
            "metadata": {
                "row": "R1",
                "area_width_m": 200.0,
                "area_height_m": 200.0,
                "horizon_s": 2.0,
                "control_sample_time_s": 1.0,
                "report_cadence_s": 1.0,
                "target": {"x": 150.0, "y": 150.0, "radius": 30.0},
            },
            "final_search_map": [
                [{"searched": True, "belief": 0.1}, {"searched": False, "belief": 0.3}],
                [{"searched": True, "belief": 0.2}, {"searched": True, "belief": 0.4}],
            ],
        },
        "state": [
            {
                "runtime": 0.0,
                "robots": [
                    {
                        "id": 1,
                        "state": {"x": 0.0, "y": 0.0},
                        "uncertainty": 0.0,
                        "opt": {"status": "success", "solver_info": {"status": "optimal"}},
                    },
                    {
                        "id": 2,
                        "state": {"x": 5.0, "y": 0.0},
                        "uncertainty": 0.0,
                        "opt": {"status": "failed", "solver_info": {"status": "primal_infeasible"}},
                    },
                ],
                "formation": [
                    {"id": 1, "anchorIds": [], "baseIds": []},
                    {"id": 2, "anchorIds": [1], "baseIds": []},
                ],
            },
            {
                "runtime": 1.0,
                "robots": [
                    {
                        "id": 1,
                        "state": {"x": 0.0, "y": 0.0},
                        "uncertainty": 0.0,
                        "opt": {"status": "success", "solver_info": {"status": "optimal"}},
                    },
                    {
                        "id": 2,
                        "state": {"x": 12.0, "y": 0.0},
                        "uncertainty": 0.5,
                        "opt": {"status": "success", "solver_info": {"status": "optimal"}},
                    },
                ],
                "formation": [
                    {"id": 1, "anchorIds": [], "baseIds": []},
                    {"id": 2, "anchorIds": [1], "baseIds": []},
                ],
            },
        ],
    }


class FullSimulatorBridgeTest(unittest.TestCase):
    def test_build_row_config_preserves_large_scale_metadata(self):
        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        row = build_row_config(base, "R4")

        self.assertEqual(row["bridge"]["row"], "R4")
        self.assertEqual(row["bridge"]["search-policy"], "active")
        self.assertEqual(row["bridge"]["topology-policy"], "adaptive-relay")
        self.assertEqual(row["bridge"]["safety-filter"], "second-order-hocbf")
        self.assertEqual(row["model"], "DoubleIntegrate2D")
        self.assertEqual(row["execute"]["time-total"], 400.0)
        self.assertEqual(row["execute"]["time-step"], 0.5)
        self.assertEqual(row["bridge"]["report-cadence"], 1.0)
        self.assertEqual(row["world"]["boundary"][2], [3000.0, 3000.0])
        self.assertEqual(row["cbfs"]["high-order"]["acceleration-bound"], 8.0)
        self.assertGreater(row["cbfs"]["high-order"]["sampled-data-reserve"], 0.0)
        self.assertEqual(row["bridge"]["nominal"]["max-speed"], 8.0)
        self.assertLess(row["bridge"]["nominal"]["max-acceleration"], row["cbfs"]["high-order"]["acceleration-bound"])

    def test_write_row_configs_creates_four_rows(self):
        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = write_row_configs(base, pathlib.Path(tmpdir))
            self.assertEqual(sorted(paths), ["R1", "R2", "R3", "R4"])
            self.assertTrue(paths["R1"].exists())
            r1 = json.loads(paths["R1"].read_text())
            self.assertEqual(r1["bridge"]["search-policy"], "coverage")
            self.assertEqual(r1["bridge"]["topology-policy"], "fixed")
            self.assertFalse(r1["cbfs"]["high-order"]["enabled"])
            self.assertEqual(r1["cbfs"]["high-order"].get("sampled-data-reserve", 0.0), 0.0)

    def test_row_configs_preserve_physical_range_and_tightening_margin(self):
        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())

        r1 = build_row_config(base, "R1")
        r4 = build_row_config(base, "R4")

        self.assertEqual(r1["cbfs"]["without-slack"]["comm-fixed"]["max-range"], 1200.0)
        self.assertEqual(r4["cbfs"]["without-slack"]["comm-fixed"]["max-range"], 1200.0)
        self.assertGreater(r1["cbfs"]["without-slack"]["comm-fixed"]["range-tightening-margin"], 0.0)
        self.assertEqual(
            r1["cbfs"]["without-slack"]["comm-fixed"]["range-tightening-margin"],
            r4["cbfs"]["without-slack"]["comm-fixed"]["range-tightening-margin"],
        )

    def test_second_order_row_does_not_inherit_first_order_safety_tightening(self):
        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())

        r2 = build_row_config(base, "R2")
        r4 = build_row_config(base, "R4")

        self.assertEqual(r2["cbfs"]["without-slack"]["safety"]["safe-distance-tightening-margin"], 5.0)
        self.assertEqual(r4["cbfs"]["without-slack"]["safety"]["safe-distance-tightening-margin"], 0.0)

    def test_single_integrator_rows_do_not_set_velocity_state(self):
        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        r1 = build_row_config(base, "R1")
        r4 = build_row_config(base, "R4")

        self.assertEqual(r1["model"], "SingleIntegrate2D")
        self.assertNotIn("velocity", r1["initial"])
        self.assertEqual(r4["model"], "DoubleIntegrate2D")
        self.assertIn("velocity", r4["initial"])


class FullSimulatorBridgeExtractionTest(unittest.TestCase):
    def test_extract_bridge_metrics_reports_nominal_guard_fields(self):
        data = {
            "config": {"cbfs": {"high-order": {"acceleration-bound": 8.0}}},
            "bridge": {"metadata": {"row": "R4_GUARD", "horizon_s": 1.0, "control_sample_time_s": 0.5}},
            "state": [
                {
                    "runtime": 0.0,
                    "bridge": {
                        "nominal": {
                            "relay_support_guard": {
                                "enabled": True,
                                "active": True,
                                "robust_margin": -2.0,
                            },
                            "support_chain_guard": {
                                "enabled": True,
                                "active_count": 2,
                                "min_margin_before": -4.0,
                                "min_margin_after": 12.0,
                            },
                            "predictive_active_gate": {
                                "enabled": True,
                                "active_count": 1,
                                "max_penalty": 12.5,
                                "min_selected_robust_margin": 30.0,
                                "links": [
                                    {
                                        "exposure_active": True,
                                        "selected_exposure_utility": 0.25,
                                        "max_exposure_utility": 0.40,
                                        "service_schedule_due": True,
                                        "searched_cells": 3.0,
                                        "required_searched_cells": 5.0,
                                        "service_schedule_deficit": 2.0,
                                    }
                                ],
                            }
                        }
                    },
                    "robots": [
                        {
                            "id": 1,
                            "opt": {
                                "status": "success",
                                "result": {"ax": 0.0, "ay": 0.0},
                                "nominalGuard": {
                                    "enabled": True,
                                    "active": True,
                                    "feasible": True,
                                    "projection_norm": 1.25,
                                    "margin_before": -0.5,
                                    "margin_after": 0.0,
                                },
                            },
                        }
                    ],
                }
            ],
        }

        metrics = extract_bridge_metrics(data)

        self.assertAlmostEqual(metrics["nominal_guard_active_ratio"], 1.0)
        self.assertAlmostEqual(metrics["nominal_guard_feasible_ratio"], 1.0)
        self.assertAlmostEqual(metrics["max_nominal_guard_projection_norm"], 1.25)
        self.assertAlmostEqual(metrics["min_nominal_guard_margin_before"], -0.5)
        self.assertAlmostEqual(metrics["min_nominal_guard_margin_after"], 0.0)
        self.assertAlmostEqual(metrics["terminal_nominal_guard_feasible_ratio"], 1.0)
        self.assertAlmostEqual(metrics["relay_support_guard_active_ratio"], 1.0)
        self.assertAlmostEqual(metrics["min_relay_support_margin"], -2.0)
        self.assertAlmostEqual(metrics["terminal_relay_support_margin"], -2.0)
        self.assertAlmostEqual(metrics["support_chain_guard_active_ratio"], 1.0)
        self.assertAlmostEqual(metrics["max_support_chain_guard_active_count"], 2.0)
        self.assertAlmostEqual(metrics["min_support_chain_margin_before"], -4.0)
        self.assertAlmostEqual(metrics["min_support_chain_margin_after"], 12.0)
        self.assertAlmostEqual(metrics["predictive_gate_active_ratio"], 1.0)
        self.assertAlmostEqual(metrics["max_predictive_gate_active_count"], 1.0)
        self.assertAlmostEqual(metrics["max_predictive_gate_penalty"], 12.5)
        self.assertAlmostEqual(metrics["min_predictive_gate_selected_robust_margin"], 30.0)
        self.assertAlmostEqual(metrics["terminal_predictive_gate_active_count"], 1.0)
        self.assertAlmostEqual(metrics["terminal_predictive_gate_selected_robust_margin"], 30.0)
        self.assertAlmostEqual(metrics["exposure_gate_active_ratio"], 1.0)
        self.assertAlmostEqual(metrics["max_exposure_gate_active_count"], 1.0)
        self.assertAlmostEqual(metrics["max_selected_exposure_utility"], 0.25)
        self.assertAlmostEqual(metrics["max_exposure_utility"], 0.40)
        self.assertAlmostEqual(metrics["service_schedule_due_ratio"], 1.0)
        self.assertAlmostEqual(metrics["max_service_schedule_deficit"], 2.0)
        self.assertAlmostEqual(metrics["max_required_searched_cells"], 5.0)

    def test_extract_bridge_metrics_reports_hocbf_control_authority_margin(self):
        data = {
            "config": {
                "cbfs": {
                    "high-order": {
                        "acceleration-bound": 8.0,
                    }
                }
            },
            "bridge": {
                "metadata": {
                    "row": "R4",
                    "horizon_s": 1.0,
                    "control_sample_time_s": 0.5,
                }
            },
            "state": [
                {
                    "runtime": 0.0,
                    "robots": [
                        {
                            "id": 1,
                            "opt": {
                                "status": "success",
                                "solver_info": {"status": "optimal"},
                                "hocbfNoSlack": [
                                    {
                                        "name": "impossible",
                                        "coe": {"ax": 1.0, "ay": 0.0},
                                        "const": -9.0,
                                        "hocbf": 0.0,
                                        "h": 1.0,
                                        "hdot": -0.5,
                                        "psi1": 0.5,
                                    }
                                ],
                            },
                        }
                    ],
                }
            ],
        }

        metrics = extract_bridge_metrics(data)

        self.assertAlmostEqual(metrics["min_control_authority_margin"], -1.0)
        self.assertEqual(metrics["joint_hocbf_feasible_ratio"], 0.0)
        self.assertAlmostEqual(metrics["terminal_min_control_authority_margin"], -1.0)
        self.assertEqual(metrics["terminal_joint_hocbf_feasible_ratio"], 0.0)
        self.assertEqual(metrics["terminal_hocbf_infeasible_robot_count"], 1.0)

    def test_extract_bridge_metrics_reports_joint_hocbf_feasibility(self):
        data = {
            "config": {
                "cbfs": {
                    "high-order": {
                        "acceleration-bound": 8.0,
                    }
                }
            },
            "bridge": {
                "metadata": {
                    "row": "R4",
                    "horizon_s": 1.0,
                    "control_sample_time_s": 0.5,
                }
            },
            "state": [
                {
                    "runtime": 0.0,
                    "robots": [
                        {
                            "id": 1,
                            "opt": {
                                "status": "success",
                                "solver_info": {"status": "optimal"},
                                "hocbfNoSlack": [
                                    {
                                        "name": "push-right",
                                        "coe": {"ax": 1.0, "ay": 0.0},
                                        "const": -7.0,
                                        "hocbf": 0.0,
                                        "h": 1.0,
                                        "hdot": 0.0,
                                        "psi1": 1.0,
                                    },
                                    {
                                        "name": "allow-left",
                                        "coe": {"ax": -1.0, "ay": 0.0},
                                        "const": 8.0,
                                        "hocbf": 0.0,
                                        "h": 1.0,
                                        "hdot": 0.0,
                                        "psi1": 1.0,
                                    },
                                ],
                            },
                        },
                        {
                            "id": 2,
                            "opt": {
                                "status": "success",
                                "solver_info": {"status": "optimal"},
                                "hocbfNoSlack": [
                                    {
                                        "name": "push-right",
                                        "coe": {"ax": 1.0, "ay": 0.0},
                                        "const": -7.0,
                                        "hocbf": 0.0,
                                        "h": 1.0,
                                        "hdot": 0.0,
                                        "psi1": 1.0,
                                    },
                                    {
                                        "name": "push-left-impossible-together",
                                        "coe": {"ax": -1.0, "ay": 0.0},
                                        "const": -9.0,
                                        "hocbf": 0.0,
                                        "h": 1.0,
                                        "hdot": 0.0,
                                        "psi1": 1.0,
                                    },
                                ],
                            },
                        },
                    ],
                }
            ],
        }

        metrics = extract_bridge_metrics(data)

        self.assertAlmostEqual(metrics["min_control_authority_margin"], -1.0)
        self.assertEqual(metrics["joint_hocbf_feasible_ratio"], 0.5)
        self.assertAlmostEqual(metrics["terminal_min_control_authority_margin"], -1.0)
        self.assertEqual(metrics["terminal_joint_hocbf_feasible_ratio"], 0.5)
        self.assertEqual(metrics["terminal_hocbf_infeasible_robot_count"], 1.0)

    def test_extract_bridge_metrics_collects_search_topology_and_hocbf(self):
        data = {
            "config": {
                "cbfs": {
                    "high-order": {
                        "sampled-data-reserve": 2.0,
                    }
                }
            },
            "bridge": {
                "metadata": {
                    "row": "R4",
                    "area_width_m": 3000.0,
                    "area_height_m": 3000.0,
                    "horizon_s": 1.0,
                    "control_sample_time_s": 0.5,
                    "report_cadence_s": 1.0,
                }
            },
            "state": [make_bridge_frame(0.0), make_bridge_frame(1.0, detected=True)],
        }

        metrics = extract_bridge_metrics(data)

        self.assertEqual(metrics["row"], "R4")
        self.assertEqual(metrics["area_width_m"], 3000.0)
        self.assertEqual(metrics["horizon_s"], 1.0)
        self.assertEqual(metrics["final_runtime_s"], 1.0)
        self.assertEqual(metrics["completed_horizon"], 1.0)
        self.assertEqual(metrics["detection_time_s"], 1.0)
        self.assertGreater(metrics["final_coverage"], 0.0)
        self.assertEqual(metrics["qp_success_ratio"], 1.0)
        self.assertEqual(metrics["accepted_switches"], 1)
        self.assertEqual(metrics["rejected_candidates"], 1)
        self.assertEqual(metrics["sampled_data_reserve"], 2.0)
        self.assertAlmostEqual(metrics["min_hocbf"], 0.3)
        self.assertAlmostEqual(metrics["min_hocbf_h"], 0.6)
        self.assertAlmostEqual(metrics["min_hocbf_hdot"], -0.2)
        self.assertAlmostEqual(metrics["min_psi1"], 0.4)

    def test_extract_bridge_metrics_reports_certified_graph_and_fail_safe_ratios(self):
        data = {
            "bridge": {
                "metadata": {
                    "row": "LD_COMPLETION_FIXED",
                    "area_width_m": 3000.0,
                    "area_height_m": 3000.0,
                    "horizon_s": 3.0,
                    "control_sample_time_s": 1.0,
                    "report_cadence_s": 1.0,
                }
            },
            "state": [
                {
                    "runtime": 0.0,
                    "bridge": {
                        "search": {"coverage_ratio": 0.1},
                        "topology": {
                            "relay_active": False,
                            "accepted_switches": 0,
                            "rejected_candidates": 1,
                            "certified": True,
                            "fail_safe": False,
                            "min_robust_margin": 4.0,
                            "min_fim_eigenvalue": 0.7,
                        },
                    },
                    "robots": [],
                },
                {
                    "runtime": 1.0,
                    "bridge": {
                        "search": {"coverage_ratio": 1.0},
                        "topology": {
                            "relay_active": False,
                            "accepted_switches": 0,
                            "rejected_candidates": 2,
                            "certified": False,
                            "fail_safe": True,
                            "min_robust_margin": -8.0,
                            "min_fim_eigenvalue": 0.0,
                        },
                    },
                    "robots": [],
                },
            ],
        }

        metrics = extract_bridge_metrics(data)

        self.assertEqual(metrics["certified_graph_ratio"], 0.5)
        self.assertEqual(metrics["fail_safe_ratio"], 0.5)
        self.assertEqual(metrics["fail_safe_steps"], 1.0)
        self.assertEqual(metrics["certified_graph_steps"], 1.0)
        self.assertEqual(metrics["coverage_completed"], 1.0)
        self.assertEqual(metrics["coverage_completion_time_s"], 1.0)

    def test_extract_bridge_metrics_collects_validation_scale_metrics(self):
        data = make_bridge_map_data()
        data["bridge"]["metadata"]["horizon_s"] = 3.0
        metrics = extract_bridge_metrics(data)

        self.assertEqual(metrics["frames"], 2.0)
        self.assertEqual(metrics["final_runtime_s"], 1.0)
        self.assertEqual(metrics["completed_horizon"], 0.0)
        self.assertEqual(metrics["solver_failures"], 1.0)
        self.assertAlmostEqual(metrics["max_comm_excess"], 2.5)
        self.assertAlmostEqual(metrics["min_physical_comm_margin"], -2.5)

    def test_write_summary_accepts_all_extracted_metric_fields(self):
        data = {
            "bridge": {
                "metadata": {
                    "row": "R4",
                    "area_width_m": 3000.0,
                    "area_height_m": 3000.0,
                    "horizon_s": 400.0,
                    "control_sample_time_s": 0.5,
                    "report_cadence_s": 1.0,
                }
            },
            "state": [make_bridge_frame(0.0)],
        }
        metrics = extract_bridge_metrics(data)

        with tempfile.TemporaryDirectory() as tmpdir:
            output_csv = pathlib.Path(tmpdir) / "summary.csv"
            write_summary([metrics], output_csv)

            text = output_csv.read_text()
            self.assertIn("min_cbf", text)
            self.assertIn("completed_horizon", text)
            self.assertIn("max_comm_excess", text)
            self.assertIn("min_physical_comm_margin", text)
            self.assertIn("min_hocbf_h", text)
            self.assertIn("min_hocbf_hdot", text)
            self.assertIn("min_control_authority_margin", text)
            self.assertIn("joint_hocbf_feasible_ratio", text)
            self.assertIn("terminal_min_control_authority_margin", text)
            self.assertIn("terminal_joint_hocbf_feasible_ratio", text)
            self.assertIn("terminal_hocbf_infeasible_robot_count", text)
            self.assertIn("R4", text)


class FullSimulatorBridgeMapFigureTest(unittest.TestCase):
    def test_denial_rectangle_bounds_uses_bridge_topology_config(self):
        bounds = _denial_rectangle_bounds(make_bridge_map_data())

        self.assertEqual(bounds, (80.0, 90.0, 40.0, 60.0))

    def test_robot_trajectories_collects_positions_by_robot_id(self):
        trajectories = _robot_trajectories(make_bridge_map_data())

        self.assertEqual(trajectories[1], [(0.0, 0.0), (0.0, 0.0)])
        self.assertEqual(trajectories[2], [(5.0, 0.0), (12.0, 0.0)])

    def test_row_display_label_uses_short_publication_names(self):
        self.assertEqual(_row_display_label("LD_COMPLETION_FIXED"), "Fixed topology")
        self.assertEqual(_row_display_label("LD_COMPLETION_RELAY"), "Certified relay")
        self.assertEqual(_row_display_label("AS_COMPLETION_ACTIVE"), "Active search")
        self.assertEqual(_row_display_label("AS_COMPLETION_FALLBACK"), "Coverage fallback")

    def test_write_bridge_search_map_figure_creates_publication_exports(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            output_prefix = pathlib.Path(tmpdir) / "bridge_search_maps"
            outputs = write_bridge_search_map_figure([make_bridge_map_data()], output_prefix)

            for path in outputs:
                self.assertTrue(path.exists(), path)
                self.assertGreater(path.stat().st_size, 0)


def make_hocbf_item(name: str, ax: float, ay: float, rhs: float) -> dict:
    return {
        "name": name,
        "coe": {"ax": ax, "ay": ay, "yawRateRad": 0.0},
        "const": -rhs,
        "hocbf": 0.0,
        "h": 1.0,
        "hdot": 0.0,
        "psi1": 1.0,
    }


def make_hocbf_feasibility_data(row: str, runtime: float, robots: list[dict]) -> dict:
    return {
        "config": {
            "cbfs": {
                "high-order": {
                    "enabled": True,
                    "acceleration-bound": 8.0,
                }
            }
        },
        "bridge": {
            "metadata": {
                "row": row,
                "horizon_s": 1.0,
                "control_sample_time_s": 0.5,
            }
        },
        "state": [
            {
                "runtime": runtime,
                "robots": robots,
            }
        ],
    }


class FullSimulatorBridgeHOCBFFeasibilityFigureTest(unittest.TestCase):
    def test_feasible_acceleration_polygon_clips_box_by_halfspaces(self):
        polygon = feasible_acceleration_polygon(
            [
                (1.0, 0.0, -2.0),
                (-1.0, 0.0, -2.0),
                (0.0, 1.0, -2.0),
                (0.0, -1.0, -2.0),
            ],
            acceleration_bound=8.0,
        )

        self.assertGreaterEqual(len(polygon), 4)
        self.assertTrue(all(-8.0 <= x <= 8.0 and -8.0 <= y <= 8.0 for x, y in polygon))
        self.assertTrue(all(-2.000001 <= x <= 2.000001 for x, _ in polygon))
        self.assertTrue(all(-2.000001 <= y <= 2.000001 for _, y in polygon))

    def test_feasible_acceleration_polygon_returns_empty_when_halfspaces_conflict(self):
        polygon = feasible_acceleration_polygon(
            [
                (1.0, 0.0, 7.0),
                (-1.0, 0.0, 9.0),
            ],
            acceleration_bound=8.0,
        )

        self.assertEqual(polygon, [])

    def test_terminal_hocbf_panels_selects_completed_and_infeasible_robots(self):
        completed = make_hocbf_feasibility_data(
            "LD_HOCBF",
            399.5,
            [
                {
                    "id": 1,
                    "opt": {
                        "status": "success",
                        "hocbfNoSlack": [
                            make_hocbf_item("right", 1.0, 0.0, -2.0),
                            make_hocbf_item("left", -1.0, 0.0, -2.0),
                            make_hocbf_item("up", 0.0, 1.0, -2.0),
                            make_hocbf_item("down", 0.0, -1.0, -2.0),
                        ],
                    },
                }
            ],
        )
        early = make_hocbf_feasibility_data(
            "LD_HOCBF",
            157.5,
            [
                {
                    "id": 4,
                    "opt": {
                        "status": None,
                        "hocbfNoSlack": [
                            make_hocbf_item("push-right", 1.0, 0.0, 7.0),
                            make_hocbf_item("push-left", -1.0, 0.0, 9.0),
                        ],
                    },
                }
            ],
        )

        panels = terminal_hocbf_panels(completed, early)

        self.assertEqual([panel.label for panel in panels[:2]], ["completed", "early termination"])
        self.assertEqual(panels[0].robot_id, 1)
        self.assertTrue(panels[0].feasible)
        self.assertGreater(len(panels[0].polygon), 0)
        self.assertEqual(panels[1].robot_id, 4)
        self.assertFalse(panels[1].feasible)
        self.assertEqual(panels[1].polygon, [])

    def test_write_hocbf_feasibility_figure_exports_publication_formats(self):
        completed = make_hocbf_feasibility_data(
            "LD_HOCBF",
            399.5,
            [
                {
                    "id": 1,
                    "opt": {
                        "status": "success",
                        "hocbfNoSlack": [
                            make_hocbf_item("right", 1.0, 0.0, -2.0),
                            make_hocbf_item("left", -1.0, 0.0, -2.0),
                            make_hocbf_item("up", 0.0, 1.0, -2.0),
                            make_hocbf_item("down", 0.0, -1.0, -2.0),
                        ],
                    },
                }
            ],
        )
        early = make_hocbf_feasibility_data(
            "LD_HOCBF",
            157.5,
            [
                {
                    "id": 4,
                    "opt": {
                        "status": None,
                        "hocbfNoSlack": [
                            make_hocbf_item("push-right", 1.0, 0.0, 7.0),
                            make_hocbf_item("push-left", -1.0, 0.0, 9.0),
                        ],
                    },
                }
            ],
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)
            trials_csv = tmp / "trials.csv"
            trials_csv.write_text(
                "row,trial,completed_horizon,terminal_joint_hocbf_feasible_ratio,terminal_hocbf_infeasible_robot_count\n"
                "LD_HOCBF,0,1,1.0,0\n"
                "LD_HOCBF,3,0,0.75,1\n",
                encoding="utf-8",
            )
            outputs = write_hocbf_feasibility_figure(
                completed,
                early,
                trials_csv,
                tmp / "hocbf_feasibility_mechanism",
            )

            self.assertEqual({path.suffix for path in outputs}, {".png", ".svg", ".pdf"})
            for path in outputs:
                self.assertTrue(path.exists(), path)
                self.assertGreater(path.stat().st_size, 0, path)


class FullSimulatorBridgeMonteCarloTest(unittest.TestCase):
    def test_build_trial_config_can_enable_nominal_guard(self):
        from bridge_experiments.run_full_simulator_bridge_mc import build_trial_config

        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        config = build_trial_config(
            base,
            row="R4",
            row_label="R4_GUARD",
            trial=0,
            seed=20260617,
            enable_nominal_guard=True,
        )

        guard = config["bridge"]["nominal"]["guard"]
        self.assertTrue(guard["enabled"])
        self.assertEqual(guard["mode"], "hocbf-feasible-projection")
        self.assertIn("_bridge_mc_seed20260617_trial000_r4_guard", config["run_suffix"])

    def test_trial_config_randomizes_target_prior_and_denial_zone_deterministically(self):
        from bridge_experiments.run_full_simulator_bridge_mc import build_trial_config

        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        config1 = build_trial_config(base, row="R4", trial=2, seed=111)
        config2 = build_trial_config(base, row="R4", trial=2, seed=111)
        config3 = build_trial_config(base, row="R4", trial=2, seed=112)

        self.assertEqual(config1, config2)
        self.assertNotEqual(config1["bridge"]["target"], base["bridge"]["target"])
        self.assertNotEqual(config1["bridge"]["target"], config3["bridge"]["target"])
        self.assertNotEqual(
            config1["bridge"]["topology"]["denial-center"],
            base["bridge"]["topology"]["denial-center"],
        )
        self.assertEqual(config1["bridge"]["row"], "R4")
        self.assertEqual(config1["execute"]["random-seed"], 111)
        self.assertIn("_bridge_mc_seed111_trial002_r4", config1["run_suffix"])
        self.assertEqual(config1["cbfs"]["high-order"]["acceleration-bound"], 8.0)
        self.assertGreater(config1["cbfs"]["high-order"]["sampled-data-reserve"], 0.0)

        target = config1["bridge"]["target"]
        prior = config1["bridge"]["search"]["target-prior-center"]
        prior_offset = math.hypot(float(target["x"]) - float(prior["x"]), float(target["y"]) - float(prior["y"]))
        self.assertGreater(prior_offset, 0.0)
        self.assertLessEqual(prior_offset, 360.0)
        self.assertGreaterEqual(target["x"], 250.0)
        self.assertLessEqual(target["x"], 2750.0)
        self.assertGreaterEqual(target["y"], 250.0)
        self.assertLessEqual(target["y"], 2750.0)

    def test_aggregate_bridge_mc_metrics_groups_trials_by_row(self):
        from bridge_experiments.run_full_simulator_bridge_mc import aggregate_bridge_mc_metrics

        rows = [
            {
                "row": "R4",
                "trial": 0,
                "seed": 100,
                "frames": 800.0,
                "final_runtime_s": 400.0,
                "completed_horizon": 1.0,
                "final_coverage": 0.40,
                "coverage_completed": 1.0,
                "coverage_completion_time_s": 300.0,
                "detected": 1.0,
                "detection_time_s": 100.0,
                "max_comm_excess": 0.0,
                "solver_failures": 0.0,
                "min_hocbf": -1e-9,
                "min_hocbf_h": 0.3,
                "min_hocbf_hdot": -0.1,
                "min_psi1": -0.5,
                "min_control_authority_margin": 0.3,
                "joint_hocbf_feasible_ratio": 1.0,
                "terminal_min_control_authority_margin": 0.5,
                "terminal_joint_hocbf_feasible_ratio": 1.0,
                "terminal_hocbf_infeasible_robot_count": 0.0,
                "max_acceleration": 7.5,
                "min_physical_comm_margin": 2.0,
                "certified_graph_ratio": 1.0,
                "certified_graph_steps": 400.0,
                "fail_safe_ratio": 0.0,
                "fail_safe_steps": 0.0,
                "predictive_gate_active_ratio": 0.25,
                "max_predictive_gate_active_count": 1.0,
                "max_predictive_gate_penalty": 4.0,
                "min_predictive_gate_selected_robust_margin": -10.0,
                "terminal_predictive_gate_active_count": 0.0,
                "exposure_gate_active_ratio": 0.25,
                "max_exposure_gate_active_count": 1.0,
                "max_selected_exposure_utility": 0.20,
                "max_exposure_utility": 0.35,
            },
            {
                "row": "R4",
                "trial": 1,
                "seed": 101,
                "frames": 380.0,
                "final_runtime_s": 190.0,
                "completed_horizon": 0.0,
                "final_coverage": 0.50,
                "coverage_completed": 0.0,
                "coverage_completion_time_s": math.nan,
                "detected": 0.0,
                "detection_time_s": math.nan,
                "max_comm_excess": 0.2,
                "solver_failures": 1.0,
                "min_hocbf": -2e-9,
                "min_hocbf_h": -0.2,
                "min_hocbf_hdot": -0.4,
                "min_psi1": -0.7,
                "min_control_authority_margin": -1.2,
                "joint_hocbf_feasible_ratio": 0.0,
                "terminal_min_control_authority_margin": -1.2,
                "terminal_joint_hocbf_feasible_ratio": 0.0,
                "terminal_hocbf_infeasible_robot_count": 1.0,
                "max_acceleration": 7.9,
                "min_physical_comm_margin": -0.2,
                "certified_graph_ratio": 0.25,
                "certified_graph_steps": 95.0,
                "fail_safe_ratio": 0.75,
                "fail_safe_steps": 285.0,
                "predictive_gate_active_ratio": 0.75,
                "max_predictive_gate_active_count": 3.0,
                "max_predictive_gate_penalty": 8.0,
                "min_predictive_gate_selected_robust_margin": -20.0,
                "terminal_predictive_gate_active_count": 2.0,
                "exposure_gate_active_ratio": 0.75,
                "max_exposure_gate_active_count": 2.0,
                "max_selected_exposure_utility": 0.30,
                "max_exposure_utility": 0.45,
            },
            {
                "row": "R3",
                "trial": 0,
                "seed": 100,
                "frames": 400.0,
                "final_runtime_s": 400.0,
                "completed_horizon": 1.0,
                "final_coverage": 0.55,
                "coverage_completed": 1.0,
                "coverage_completion_time_s": 250.0,
                "detected": 1.0,
                "detection_time_s": 95.0,
                "max_comm_excess": 0.0,
                "solver_failures": 0.0,
                "min_hocbf": math.nan,
                "min_hocbf_h": math.nan,
                "min_hocbf_hdot": math.nan,
                "min_psi1": math.nan,
                "min_control_authority_margin": math.nan,
                "joint_hocbf_feasible_ratio": math.nan,
                "terminal_min_control_authority_margin": math.nan,
                "terminal_joint_hocbf_feasible_ratio": math.nan,
                "terminal_hocbf_infeasible_robot_count": math.nan,
                "max_acceleration": 0.0,
                "min_physical_comm_margin": 5.0,
                "certified_graph_ratio": 1.0,
                "certified_graph_steps": 400.0,
                "fail_safe_ratio": 0.0,
                "fail_safe_steps": 0.0,
            },
        ]

        aggregate = aggregate_bridge_mc_metrics(rows)
        by_row = {row["row"]: row for row in aggregate}

        self.assertEqual(by_row["R4"]["trial_count"], 2)
        self.assertAlmostEqual(by_row["R4"]["completion_rate"], 0.5)
        self.assertAlmostEqual(by_row["R4"]["coverage_completion_rate"], 0.5)
        self.assertAlmostEqual(by_row["R4"]["mean_coverage_completion_time_s"], 300.0)
        self.assertAlmostEqual(by_row["R4"]["coverage_completion_time_s_max"], 300.0)
        self.assertAlmostEqual(by_row["R4"]["detection_rate"], 0.5)
        self.assertAlmostEqual(by_row["R4"]["mean_detection_time_s"], 100.0)
        self.assertAlmostEqual(by_row["R4"]["mean_final_coverage"], 0.45)
        self.assertAlmostEqual(by_row["R4"]["max_comm_excess_max"], 0.2)
        self.assertAlmostEqual(by_row["R4"]["min_physical_comm_margin_min"], -0.2)
        self.assertEqual(by_row["R4"]["solver_failures_max"], 1.0)
        self.assertAlmostEqual(by_row["R4"]["min_hocbf_min"], -2e-9)
        self.assertAlmostEqual(by_row["R4"]["min_hocbf_h_min"], -0.2)
        self.assertAlmostEqual(by_row["R4"]["min_hocbf_hdot_min"], -0.4)
        self.assertAlmostEqual(by_row["R4"]["min_control_authority_margin_min"], -1.2)
        self.assertAlmostEqual(by_row["R4"]["joint_hocbf_feasible_ratio_min"], 0.0)
        self.assertAlmostEqual(by_row["R4"]["terminal_min_control_authority_margin_min"], -1.2)
        self.assertAlmostEqual(by_row["R4"]["terminal_joint_hocbf_feasible_ratio_min"], 0.0)
        self.assertAlmostEqual(by_row["R4"]["terminal_hocbf_infeasible_robot_count_max"], 1.0)
        self.assertAlmostEqual(by_row["R4"]["max_acceleration_max"], 7.9)
        self.assertAlmostEqual(by_row["R4"]["certified_graph_ratio_mean"], 0.625)
        self.assertAlmostEqual(by_row["R4"]["certified_graph_ratio_min"], 0.25)
        self.assertAlmostEqual(by_row["R4"]["fail_safe_ratio_mean"], 0.375)
        self.assertAlmostEqual(by_row["R4"]["fail_safe_ratio_max"], 0.75)
        self.assertAlmostEqual(by_row["R4"]["fail_safe_steps_max"], 285.0)
        self.assertAlmostEqual(by_row["R4"]["predictive_gate_active_ratio_mean"], 0.5)
        self.assertAlmostEqual(by_row["R4"]["predictive_gate_active_ratio_max"], 0.75)
        self.assertAlmostEqual(by_row["R4"]["max_predictive_gate_active_count_max"], 3.0)
        self.assertAlmostEqual(by_row["R4"]["max_predictive_gate_penalty_max"], 8.0)
        self.assertAlmostEqual(by_row["R4"]["min_predictive_gate_selected_robust_margin_min"], -20.0)
        self.assertAlmostEqual(by_row["R4"]["terminal_predictive_gate_active_count_max"], 2.0)
        self.assertAlmostEqual(by_row["R4"]["exposure_gate_active_ratio_mean"], 0.5)
        self.assertAlmostEqual(by_row["R4"]["exposure_gate_active_ratio_max"], 0.75)
        self.assertAlmostEqual(by_row["R4"]["max_exposure_gate_active_count_max"], 2.0)
        self.assertAlmostEqual(by_row["R4"]["max_selected_exposure_utility_max"], 0.30)
        self.assertAlmostEqual(by_row["R4"]["max_exposure_utility_max"], 0.45)
        self.assertEqual(by_row["R3"]["trial_count"], 1)

    def test_write_bridge_mc_artifacts_creates_trials_summary_and_figures(self):
        from bridge_experiments.run_full_simulator_bridge_mc import write_bridge_mc_artifacts

        rows = [
            {
                "row": "R2",
                "trial": 0,
                "seed": 100,
                "target_x": 2400.0,
                "target_y": 1800.0,
                "prior_x": 2300.0,
                "prior_y": 1900.0,
                "denial_center_x": 1600.0,
                "denial_center_y": 1500.0,
                "denial_half_x": 400.0,
                "denial_half_y": 650.0,
                "frames": 400.0,
                "final_runtime_s": 400.0,
                "completed_horizon": 1.0,
                "final_coverage": 0.44,
                "coverage_completed": 1.0,
                "coverage_completion_time_s": 314.0,
                "detected": 1.0,
                "detection_time_s": 112.0,
                "max_comm_excess": 0.0,
                "solver_failures": 0.0,
                "min_hocbf": math.nan,
                "min_hocbf_h": math.nan,
                "min_hocbf_hdot": math.nan,
                "min_psi1": math.nan,
                "min_control_authority_margin": math.nan,
                "joint_hocbf_feasible_ratio": math.nan,
                "terminal_min_control_authority_margin": math.nan,
                "terminal_joint_hocbf_feasible_ratio": math.nan,
                "terminal_hocbf_infeasible_robot_count": math.nan,
                "max_acceleration": 0.0,
                "min_physical_comm_margin": 1.0,
                "certified_graph_ratio": 1.0,
                "certified_graph_steps": 400.0,
                "fail_safe_ratio": 0.0,
                "fail_safe_steps": 0.0,
                "sampled_data_reserve": 2.0,
            }
        ]

        with tempfile.TemporaryDirectory() as tmpdir:
            paths = write_bridge_mc_artifacts(rows, pathlib.Path(tmpdir), "bridge_mc")

            self.assertIn("trial", paths["trials_csv"].read_text())
            self.assertIn("min_physical_comm_margin", paths["trials_csv"].read_text())
            self.assertIn("sampled_data_reserve", paths["trials_csv"].read_text())
            self.assertIn("coverage_completed", paths["trials_csv"].read_text())
            self.assertIn("coverage_completion_rate", paths["aggregate_csv"].read_text())
            self.assertIn("coverage_completion_time_s", paths["trials_csv"].read_text())
            self.assertIn("mean_coverage_completion_time_s", paths["aggregate_csv"].read_text())
            self.assertIn("min_physical_comm_margin_min", paths["aggregate_csv"].read_text())
            self.assertIn("min_hocbf_h", paths["trials_csv"].read_text())
            self.assertIn("min_hocbf_h_min", paths["aggregate_csv"].read_text())
            self.assertIn("min_hocbf_hdot_min", paths["aggregate_csv"].read_text())
            self.assertIn("min_control_authority_margin", paths["trials_csv"].read_text())
            self.assertIn("min_control_authority_margin_min", paths["aggregate_csv"].read_text())
            self.assertIn("joint_hocbf_feasible_ratio", paths["trials_csv"].read_text())
            self.assertIn("joint_hocbf_feasible_ratio_min", paths["aggregate_csv"].read_text())
            self.assertIn("terminal_joint_hocbf_feasible_ratio", paths["trials_csv"].read_text())
            self.assertIn("terminal_joint_hocbf_feasible_ratio_min", paths["aggregate_csv"].read_text())
            self.assertIn("terminal_hocbf_infeasible_robot_count", paths["trials_csv"].read_text())
            self.assertIn("terminal_hocbf_infeasible_robot_count_max", paths["aggregate_csv"].read_text())
            self.assertIn("certified_graph_ratio", paths["trials_csv"].read_text())
            self.assertIn("certified_graph_ratio_mean", paths["aggregate_csv"].read_text())
            self.assertIn("fail_safe_ratio", paths["trials_csv"].read_text())
            self.assertIn("fail_safe_ratio_max", paths["aggregate_csv"].read_text())
            self.assertIn("detection_rate", paths["aggregate_csv"].read_text())
            for key in ("figure_png", "figure_svg", "figure_pdf"):
                self.assertTrue(paths[key].exists(), key)
                self.assertGreater(paths[key].stat().st_size, 0)


class FullSimulatorBridgePaperSuiteTest(unittest.TestCase):
    def write_minimal_valid_suite_data(self, path: pathlib.Path, row: str) -> None:
        data = {
            "config": {
                "world": {
                    "boundary": [[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]],
                },
                "execute": {"time-step": 1.0},
                "model": "SingleIntegrate2D",
                "bases": [],
                "cbfs": {
                    "without-slack": {
                        "comm-fixed": {"on": False},
                        "safety": {"on": False},
                    }
                },
            },
            "bridge": {
                "metadata": {
                    "row": row,
                    "area_width_m": 100.0,
                    "area_height_m": 100.0,
                    "horizon_s": 1.0,
                    "control_sample_time_s": 1.0,
                    "report_cadence_s": 1.0,
                }
            },
            "state": [
                {
                    "runtime": 0.0,
                    "robots": [
                        {
                            "id": 1,
                            "state": {"x": 10.0, "y": 10.0, "battery": 4100.0, "yawRad": 0.0},
                            "uncertainty": 0.0,
                            "opt": {
                                "status": "success",
                                "solver_info": {"status": "optimal"},
                                "result": {"vx": 0.0, "vy": 0.0, "yawRateRad": 0.0},
                                "slacks": [],
                            },
                            "cbfNoSlack": {},
                        }
                    ],
                    "formation": [{"id": 1, "anchorIds": [], "baseIds": []}],
                }
            ],
        }
        path.write_text(json.dumps(data), encoding="utf-8")

    def make_suite_metric_row(self, row: str, source_data: pathlib.Path | None = None) -> dict:
        return {
            "row": row,
            "trial": 0,
            "seed": 20260617,
            "source_data": str(source_data) if source_data else "",
            "frames": 400.0,
            "final_runtime_s": 400.0,
            "completed_horizon": 1.0,
            "final_coverage": 0.42,
            "coverage_completed": 0.0,
            "coverage_completion_time_s": math.nan,
            "detected": 1.0,
            "detection_time_s": 120.0,
            "qp_success_ratio": 1.0,
            "solver_failures": 0.0,
            "max_comm_excess": 0.0,
            "max_slack": 0.0,
            "min_cbf": 1.0,
            "min_collision_margin": 1.0,
            "min_localization_margin": 1.0,
            "min_hocbf": math.nan,
            "min_psi1": math.nan,
            "max_acceleration": 0.0,
            "max_yaw_rate": 0.0,
            "relay_active_ratio": 0.5,
            "min_robust_margin": 1.0,
            "min_fim_eigenvalue": 0.1,
        }

    def test_link_denied_suite_specs_and_dry_run_generation(self):
        from link_denied_experiments import run_full_simulator_link_denied_bridge as link_denied_suite

        specs = link_denied_suite.default_link_denied_suite_specs(include_hocbf=True)

        self.assertEqual([spec.label for spec in specs], ["LD_FIXED", "LD_RELAY", "LD_HOCBF"])
        self.assertEqual([spec.source_row for spec in specs], ["R2", "R3", "R4"])

        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)
            rows = link_denied_suite.run_link_denied_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "paper",
                trials=1,
                base_seed=20260617,
                data_dir=tmp / "data",
                include_hocbf=True,
                dry_run=True,
            )

            self.assertEqual(rows, [])
            self.assertTrue((tmp / "configs" / "ld_fixed_seed20260617_trial000.json").exists())
            self.assertTrue((tmp / "configs" / "ld_relay_seed20260617_trial000.json").exists())
            self.assertTrue((tmp / "configs" / "ld_hocbf_seed20260617_trial000.json").exists())

    def test_link_denied_suite_can_include_guard_row(self):
        from link_denied_experiments import run_full_simulator_link_denied_bridge as link_denied_suite

        specs = link_denied_suite.default_link_denied_suite_specs(include_hocbf=True, include_guard=True)

        self.assertEqual([spec.label for spec in specs], ["LD_FIXED", "LD_RELAY", "LD_HOCBF", "LD_HOCBF_GUARD"])
        self.assertEqual([spec.source_row for spec in specs], ["R2", "R3", "R4", "R4"])
        self.assertEqual([spec.enable_nominal_guard for spec in specs], [False, False, False, True])

        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)
            rows = link_denied_suite.run_link_denied_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "paper",
                trials=1,
                base_seed=20260617,
                data_dir=tmp / "data",
                include_hocbf=True,
                include_guard=True,
                dry_run=True,
            )

            self.assertEqual(rows, [])
            guard_path = tmp / "configs" / "ld_hocbf_guard_seed20260617_trial000.json"
            self.assertTrue(guard_path.exists())
            guard_config = json.loads(guard_path.read_text())
            self.assertTrue(guard_config["bridge"]["nominal"]["guard"]["enabled"])

    def test_link_denied_suite_can_include_predictive_reserve_row(self):
        from link_denied_experiments import run_full_simulator_link_denied_bridge as link_denied_suite

        specs = link_denied_suite.default_link_denied_suite_specs(include_hocbf=True, include_predictive_reserve=True)

        self.assertEqual([spec.label for spec in specs], ["LD_FIXED", "LD_RELAY", "LD_HOCBF", "LD_HOCBF_PRED"])
        self.assertEqual([spec.source_row for spec in specs], ["R2", "R3", "R4", "R4"])
        self.assertEqual([spec.enable_nominal_guard for spec in specs], [False, False, False, True])
        self.assertEqual(specs[-1].topology_policy, "adaptive-relay-reserve")
        self.assertGreater(specs[-1].robust_switch_margin, 0.0)

        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)
            rows = link_denied_suite.run_link_denied_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "paper",
                trials=1,
                base_seed=20260617,
                data_dir=tmp / "data",
                include_hocbf=True,
                include_predictive_reserve=True,
                dry_run=True,
            )

            self.assertEqual(rows, [])
            pred_path = tmp / "configs" / "ld_hocbf_pred_seed20260617_trial000.json"
            self.assertTrue(pred_path.exists())
            pred_config = json.loads(pred_path.read_text())
            self.assertEqual(pred_config["bridge"]["topology-policy"], "adaptive-relay-reserve")
            self.assertEqual(pred_config["bridge"]["topology"]["robust-switch-margin"], specs[-1].robust_switch_margin)
            self.assertTrue(pred_config["bridge"]["nominal"]["guard"]["enabled"])
            relay_guard = pred_config["bridge"]["nominal"]["relay-support-guard"]
            self.assertTrue(relay_guard["enabled"])
            self.assertEqual(relay_guard["robust-margin"], specs[-1].robust_switch_margin)
            chain_guard = pred_config["bridge"]["nominal"]["support-chain-guard"]
            self.assertTrue(chain_guard["enabled"])
            self.assertEqual(chain_guard["robust-margin"], specs[-1].robust_switch_margin)

    def test_link_denied_suite_can_include_predictive_edge_reserve_row(self):
        from link_denied_experiments import run_full_simulator_link_denied_bridge as link_denied_suite

        specs = link_denied_suite.default_link_denied_suite_specs(
            include_hocbf=True,
            include_predictive_reserve=True,
            include_predictive_edge_reserve=True,
        )

        self.assertEqual(
            [spec.label for spec in specs],
            ["LD_FIXED", "LD_RELAY", "LD_HOCBF", "LD_HOCBF_PRED", "LD_HOCBF_PRED_EDGE"],
        )
        self.assertFalse(specs[-2].enable_edge_reserve_tightening)
        self.assertTrue(specs[-1].enable_edge_reserve_tightening)

        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        base_tightening = base["cbfs"]["without-slack"]["comm-fixed"]["range-tightening-margin"]
        topology = base["bridge"]["topology"]
        expected_uncertainty = topology["uncertainty-multiplier"] * math.sqrt(
            topology["sigma0"] ** 2 / topology["denied-quality"] + topology["reference-variance"]
        )
        expected_tightening = specs[-1].robust_switch_margin + expected_uncertainty

        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)
            rows = link_denied_suite.run_link_denied_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "paper",
                trials=1,
                base_seed=20260617,
                data_dir=tmp / "data",
                include_hocbf=True,
                include_predictive_reserve=True,
                include_predictive_edge_reserve=True,
                dry_run=True,
            )

            self.assertEqual(rows, [])
            pred_path = tmp / "configs" / "ld_hocbf_pred_seed20260617_trial000.json"
            edge_path = tmp / "configs" / "ld_hocbf_pred_edge_seed20260617_trial000.json"
            self.assertTrue(pred_path.exists())
            self.assertTrue(edge_path.exists())
            pred_config = json.loads(pred_path.read_text())
            edge_config = json.loads(edge_path.read_text())
            self.assertEqual(
                pred_config["cbfs"]["without-slack"]["comm-fixed"]["range-tightening-margin"],
                base_tightening,
            )
            self.assertAlmostEqual(
                edge_config["cbfs"]["without-slack"]["comm-fixed"]["range-tightening-margin"],
                expected_tightening,
            )

    def test_link_denied_suite_can_include_predictive_state_reserve_row(self):
        from link_denied_experiments import run_full_simulator_link_denied_bridge as link_denied_suite

        state_only_specs = link_denied_suite.default_link_denied_suite_specs(
            include_hocbf=True,
            include_predictive_state_reserve=True,
        )
        self.assertEqual(
            [spec.label for spec in state_only_specs],
            ["LD_FIXED", "LD_RELAY", "LD_HOCBF", "LD_HOCBF_PRED_SD"],
        )

        specs = link_denied_suite.default_link_denied_suite_specs(
            include_hocbf=True,
            include_predictive_reserve=True,
            include_predictive_edge_reserve=True,
            include_predictive_state_reserve=True,
        )

        self.assertEqual(
            [spec.label for spec in specs],
            ["LD_FIXED", "LD_RELAY", "LD_HOCBF", "LD_HOCBF_PRED", "LD_HOCBF_PRED_EDGE", "LD_HOCBF_PRED_SD"],
        )
        self.assertFalse(specs[-3].enable_state_dependent_edge_reserve)
        self.assertFalse(specs[-2].enable_state_dependent_edge_reserve)
        self.assertTrue(specs[-1].enable_state_dependent_edge_reserve)

        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        base_tightening = base["cbfs"]["without-slack"]["comm-fixed"]["range-tightening-margin"]

        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)
            rows = link_denied_suite.run_link_denied_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "paper",
                trials=1,
                base_seed=20260617,
                data_dir=tmp / "data",
                include_hocbf=True,
                include_predictive_reserve=True,
                include_predictive_edge_reserve=True,
                include_predictive_state_reserve=True,
                dry_run=True,
            )

            self.assertEqual(rows, [])
            pred_path = tmp / "configs" / "ld_hocbf_pred_seed20260617_trial000.json"
            edge_path = tmp / "configs" / "ld_hocbf_pred_edge_seed20260617_trial000.json"
            state_path = tmp / "configs" / "ld_hocbf_pred_sd_seed20260617_trial000.json"
            self.assertTrue(pred_path.exists())
            self.assertTrue(edge_path.exists())
            self.assertTrue(state_path.exists())

            pred_config = json.loads(pred_path.read_text())
            edge_config = json.loads(edge_path.read_text())
            state_config = json.loads(state_path.read_text())
            self.assertNotIn(
                "state-dependent-reserve",
                pred_config["cbfs"]["without-slack"]["comm-fixed"],
            )
            self.assertNotIn(
                "state-dependent-reserve",
                edge_config["cbfs"]["without-slack"]["comm-fixed"],
            )
            self.assertEqual(
                state_config["cbfs"]["without-slack"]["comm-fixed"]["range-tightening-margin"],
                base_tightening,
            )
            state_reserve = state_config["cbfs"]["without-slack"]["comm-fixed"]["state-dependent-reserve"]
            self.assertTrue(state_reserve["enabled"])
            self.assertEqual(state_reserve["velocity-gain"], 1.0)
            self.assertEqual(state_reserve["sample-time"], state_config["execute"]["time-step"])
            self.assertEqual(state_reserve["acceleration-gain"], 1.0)
            self.assertEqual(
                state_reserve["neighbor-acceleration-bound"],
                state_config["cbfs"]["high-order"]["acceleration-bound"],
            )
            self.assertEqual(state_reserve["max-reserve"], 120.0)

    def test_link_denied_suite_can_include_predictive_all_edge_reserve_row(self):
        from link_denied_experiments import run_full_simulator_link_denied_bridge as link_denied_suite

        specs = link_denied_suite.default_link_denied_suite_specs(
            include_hocbf=True,
            include_predictive_reserve=True,
            include_predictive_state_reserve=True,
            include_predictive_all_edge_reserve=True,
        )

        self.assertEqual(specs[-1].label, "LD_HOCBF_PRED_AE")
        self.assertEqual(specs[-1].support_chain_guard_scope, "all-active-edges")

        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)
            rows = link_denied_suite.run_link_denied_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "paper",
                trials=1,
                base_seed=20260617,
                data_dir=tmp / "data",
                include_hocbf=True,
                include_predictive_reserve=True,
                include_predictive_state_reserve=True,
                include_predictive_all_edge_reserve=True,
                dry_run=True,
            )

            self.assertEqual(rows, [])
            config_path = tmp / "configs" / "ld_hocbf_pred_ae_seed20260617_trial000.json"
            self.assertTrue(config_path.exists())
            config = json.loads(config_path.read_text())
            guard = config["bridge"]["nominal"]["support-chain-guard"]
            self.assertTrue(guard["enabled"])
            self.assertEqual(guard["scope"], "all-active-edges")
            self.assertIn("state-dependent-reserve", config["cbfs"]["without-slack"]["comm-fixed"])

    def test_link_denied_suite_can_generate_first_order_completion_stress_rows(self):
        from link_denied_experiments import run_full_simulator_link_denied_bridge as link_denied_suite

        specs = link_denied_suite.default_link_denied_completion_suite_specs()

        self.assertEqual([spec.label for spec in specs], ["LD_COMPLETION_FIXED", "LD_COMPLETION_RELAY"])
        self.assertEqual([spec.source_row for spec in specs], ["R1", "R3"])

        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)
            rows = link_denied_suite.run_link_denied_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "paper",
                trials=1,
                base_seed=20260617,
                data_dir=tmp / "data",
                completion_stress=True,
                dry_run=True,
            )

            self.assertEqual(rows, [])
            fixed_path = tmp / "configs" / "ld_completion_fixed_seed20260617_trial000.json"
            relay_path = tmp / "configs" / "ld_completion_relay_seed20260617_trial000.json"
            self.assertTrue(fixed_path.exists())
            self.assertTrue(relay_path.exists())
            fixed_config = json.loads(fixed_path.read_text())
            relay_config = json.loads(relay_path.read_text())

        for config in (fixed_config, relay_config):
            self.assertEqual(config["model"], "SingleIntegrate2D")
            self.assertEqual(config["bridge"]["search-policy"], "coverage")
            self.assertGreaterEqual(config["bridge"]["nominal"]["max-speed"], 12.0)
            self.assertTrue(config["bridge"]["topology"]["certified-only"])
            self.assertTrue(config["bridge"]["topology"]["fail-safe-hold"])
            self.assertEqual(config["execute"]["time-total"], 400.0)
            self.assertEqual(config["world"]["boundary"][2], [3000.0, 3000.0])
            front_sector = config["searching"]["front-sector"]
            self.assertGreaterEqual(front_sector["outer-radius"], 600.0)
            self.assertGreaterEqual(front_sector["half-angle-deg"], 55.0)
            topology = config["bridge"]["topology"]
            expected_uncertainty = topology["uncertainty-multiplier"] * math.sqrt(
                topology["sigma0"] ** 2 / topology["denied-quality"] + topology["reference-variance"]
            )
            self.assertGreaterEqual(
                config["cbfs"]["without-slack"]["comm-fixed"]["range-tightening-margin"],
                expected_uncertainty,
            )
            self.assertEqual(config["bridge"]["search"]["target-prior-strength"], 0.0)
            self.assertEqual(config["bridge"]["search"]["goal-boundary-margin"], 0.0)
            support_guard = config["bridge"]["nominal"]["support-chain-guard"]
            self.assertTrue(support_guard["enabled"])
            self.assertEqual(support_guard["scope"], "all-active-edges")
            self.assertEqual(support_guard["robust-margin"], 0.0)

        self.assertEqual(fixed_config["bridge"]["topology-policy"], "fixed")
        self.assertEqual(relay_config["bridge"]["topology-policy"], "adaptive-chain")
        self.assertEqual(relay_config["bridge"]["topology"]["max-range"], fixed_config["bridge"]["topology"]["max-range"])

    def test_link_denied_completion_stress_can_include_predictive_reserve_row(self):
        from link_denied_experiments import run_full_simulator_link_denied_bridge as link_denied_suite

        specs = link_denied_suite.default_link_denied_completion_suite_specs(
            include_predictive_reserve=True,
            predictive_reserve_margin=40.0,
        )

        self.assertEqual(
            [spec.label for spec in specs],
            ["LD_COMPLETION_FIXED", "LD_COMPLETION_RELAY", "LD_COMPLETION_RELAY_RESERVE"],
        )
        self.assertEqual(specs[-1].topology_policy, "adaptive-relay-reserve")
        self.assertEqual(specs[-1].robust_switch_margin, 40.0)

        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)
            rows = link_denied_suite.run_link_denied_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "paper",
                trials=1,
                base_seed=20260617,
                data_dir=tmp / "data",
                include_predictive_reserve=True,
                predictive_reserve_margin=40.0,
                completion_stress=True,
                dry_run=True,
            )

            self.assertEqual(rows, [])
            reserve_path = tmp / "configs" / "ld_completion_relay_reserve_seed20260617_trial000.json"
            self.assertTrue(reserve_path.exists())
            reserve_config = json.loads(reserve_path.read_text())

        self.assertEqual(reserve_config["model"], "SingleIntegrate2D")
        self.assertEqual(reserve_config["bridge"]["search-policy"], "coverage")
        self.assertEqual(reserve_config["bridge"]["topology-policy"], "adaptive-relay-reserve")
        self.assertEqual(reserve_config["bridge"]["topology"]["robust-switch-margin"], 40.0)
        relay_guard = reserve_config["bridge"]["nominal"]["relay-support-guard"]
        self.assertTrue(relay_guard["enabled"])
        self.assertEqual(relay_guard["robust-margin"], 40.0)
        chain_guard = reserve_config["bridge"]["nominal"]["support-chain-guard"]
        self.assertTrue(chain_guard["enabled"])
        self.assertEqual(chain_guard["scope"], "first-anchor")
        self.assertEqual(chain_guard["robust-margin"], 40.0)

    def test_link_denied_completion_stress_can_generate_reserve_margin_frontier(self):
        from link_denied_experiments import run_full_simulator_link_denied_bridge as link_denied_suite

        specs = link_denied_suite.default_link_denied_completion_suite_specs(
            predictive_reserve_margins=[0.0, 5.0, 12.5],
        )

        self.assertEqual(
            [spec.label for spec in specs],
            [
                "LD_COMPLETION_FIXED",
                "LD_COMPLETION_RELAY",
                "LD_COMPLETION_RELAY_RESERVE_M0",
                "LD_COMPLETION_RELAY_RESERVE_M5",
                "LD_COMPLETION_RELAY_RESERVE_M12P5",
            ],
        )
        self.assertEqual([spec.robust_switch_margin for spec in specs[-3:]], [0.0, 5.0, 12.5])

        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)
            rows = link_denied_suite.run_link_denied_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "paper",
                trials=1,
                base_seed=20260617,
                data_dir=tmp / "data",
                completion_reserve_margins=[0.0, 5.0],
                completion_stress=True,
                dry_run=True,
            )

            self.assertEqual(rows, [])
            reserve_path = tmp / "configs" / "ld_completion_relay_reserve_m5_seed20260617_trial000.json"
            self.assertTrue(reserve_path.exists())
            reserve_config = json.loads(reserve_path.read_text())

        self.assertEqual(reserve_config["bridge"]["topology-policy"], "adaptive-relay-reserve")
        self.assertEqual(reserve_config["bridge"]["topology"]["robust-switch-margin"], 5.0)
        self.assertEqual(reserve_config["bridge"]["nominal"]["relay-support-guard"]["robust-margin"], 5.0)
        self.assertEqual(reserve_config["bridge"]["nominal"]["support-chain-guard"]["robust-margin"], 5.0)

    def test_link_denied_completion_stress_can_include_task_aware_reserve_row(self):
        from link_denied_experiments import run_full_simulator_link_denied_bridge as link_denied_suite

        specs = link_denied_suite.default_link_denied_completion_suite_specs(
            include_task_aware_reserve=True,
            predictive_reserve_margin=25.0,
        )

        self.assertEqual(
            [spec.label for spec in specs],
            [
                "LD_COMPLETION_FIXED",
                "LD_COMPLETION_RELAY",
                "LD_COMPLETION_CHAIN_TASK_RESERVE",
            ],
        )
        self.assertEqual(specs[-1].topology_policy, "adaptive-chain")
        self.assertEqual(specs[-1].robust_switch_margin, 25.0)
        self.assertTrue(specs[-1].enable_task_aware_reserve)

        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)
            rows = link_denied_suite.run_link_denied_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "paper",
                trials=1,
                base_seed=20260617,
                data_dir=tmp / "data",
                include_task_aware_reserve=True,
                predictive_reserve_margin=25.0,
                completion_stress=True,
                dry_run=True,
            )

            self.assertEqual(rows, [])
            reserve_path = tmp / "configs" / "ld_completion_chain_task_reserve_seed20260617_trial000.json"
            self.assertTrue(reserve_path.exists())
            reserve_config = json.loads(reserve_path.read_text())

        self.assertEqual(reserve_config["bridge"]["topology-policy"], "adaptive-chain")
        self.assertEqual(reserve_config["bridge"]["topology"]["robust-switch-margin"], 25.0)
        nominal = reserve_config["bridge"]["nominal"]
        self.assertNotIn("relay-support-guard", nominal)
        chain_guard = nominal["support-chain-guard"]
        self.assertTrue(chain_guard["enabled"])
        self.assertEqual(chain_guard["scope"], "first-anchor")
        self.assertEqual(chain_guard["robust-margin"], 25.0)

    def test_active_search_suite_specs_and_dry_run_generation(self):
        from active_search_experiments import run_full_simulator_active_bridge as active_suite

        specs = active_suite.default_active_search_suite_specs(include_hocbf=True)

        self.assertEqual([spec.label for spec in specs], ["AS_COVERAGE", "AS_ACTIVE", "AS_RELAY", "AS_HOCBF"])
        self.assertEqual([spec.source_row for spec in specs], ["R1", "R2", "R3", "R4"])

        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)
            rows = active_suite.run_active_search_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "paper",
                trials=1,
                base_seed=20260617,
                data_dir=tmp / "data",
                include_hocbf=True,
                dry_run=True,
            )

            self.assertEqual(rows, [])
            self.assertTrue((tmp / "configs" / "as_coverage_seed20260617_trial000.json").exists())
            self.assertTrue((tmp / "configs" / "as_active_seed20260617_trial000.json").exists())
            self.assertTrue((tmp / "configs" / "as_relay_seed20260617_trial000.json").exists())
            self.assertTrue((tmp / "configs" / "as_hocbf_seed20260617_trial000.json").exists())

    def test_active_search_suite_can_generate_first_order_completion_fallback_rows(self):
        from active_search_experiments import run_full_simulator_active_bridge as active_suite

        specs = active_suite.default_active_completion_suite_specs()

        self.assertEqual([spec.label for spec in specs], ["AS_COMPLETION_ACTIVE", "AS_COMPLETION_FALLBACK"])
        self.assertEqual([spec.source_row for spec in specs], ["R2", "R2"])
        self.assertEqual([spec.search_policy for spec in specs], ["active", "coverage"])

        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)
            rows = active_suite.run_active_search_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "paper",
                trials=1,
                base_seed=20260617,
                data_dir=tmp / "data",
                completion_stress=True,
                dry_run=True,
            )

            self.assertEqual(rows, [])
            active_path = tmp / "configs" / "as_completion_active_seed20260617_trial000.json"
            fallback_path = tmp / "configs" / "as_completion_fallback_seed20260617_trial000.json"
            self.assertTrue(active_path.exists())
            self.assertTrue(fallback_path.exists())
            active_config = json.loads(active_path.read_text())
            fallback_config = json.loads(fallback_path.read_text())

        for config in (active_config, fallback_config):
            self.assertEqual(config["model"], "SingleIntegrate2D")
            self.assertEqual(config["execute"]["time-total"], 400.0)
            self.assertEqual(config["execute"]["time-step"], 1.0)
            self.assertEqual(config["world"]["boundary"][2], [3000.0, 3000.0])
            self.assertFalse(config["cbfs"]["high-order"]["enabled"])
            self.assertEqual(config["cbfs"]["high-order"]["sampled-data-reserve"], 0.0)
            self.assertGreaterEqual(config["bridge"]["nominal"]["max-speed"], 12.0)
            self.assertEqual(config["bridge"]["safety-filter"], "first-order-cbf")
            front_sector = config["searching"]["front-sector"]
            self.assertEqual(front_sector["outer-radius"], 450.0)
            self.assertEqual(front_sector["half-angle-deg"], 60.0)
            search = config["bridge"]["search"]
            self.assertEqual(search["coverage-completion-threshold"], 1.0)
            self.assertEqual(search["goal-boundary-margin"], 0.0)

        self.assertEqual(active_config["bridge"]["search-policy"], "active")
        self.assertGreater(active_config["bridge"]["search"]["target-prior-strength"], 0.0)
        self.assertGreater(active_config["bridge"]["search"]["belief-weight"], 0.0)
        self.assertEqual(fallback_config["bridge"]["search-policy"], "coverage")
        self.assertEqual(fallback_config["bridge"]["search"]["fallback-from-policy"], "active")

    def test_active_search_suite_can_include_guard_row(self):
        from active_search_experiments import run_full_simulator_active_bridge as active_suite

        specs = active_suite.default_active_search_suite_specs(include_hocbf=True, include_guard=True)

        self.assertEqual(
            [spec.label for spec in specs],
            ["AS_COVERAGE", "AS_ACTIVE", "AS_RELAY", "AS_HOCBF", "AS_HOCBF_GUARD"],
        )
        self.assertEqual([spec.source_row for spec in specs], ["R1", "R2", "R3", "R4", "R4"])
        self.assertEqual([spec.enable_nominal_guard for spec in specs], [False, False, False, False, True])

        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)
            rows = active_suite.run_active_search_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "paper",
                trials=1,
                base_seed=20260617,
                data_dir=tmp / "data",
                include_hocbf=True,
                include_guard=True,
                dry_run=True,
            )

            self.assertEqual(rows, [])
            guard_path = tmp / "configs" / "as_hocbf_guard_seed20260617_trial000.json"
            self.assertTrue(guard_path.exists())
            guard_config = json.loads(guard_path.read_text())
            self.assertTrue(guard_config["bridge"]["nominal"]["guard"]["enabled"])

    def test_active_search_suite_can_include_predictive_gate_row(self):
        from active_search_experiments import run_full_simulator_active_bridge as active_suite

        specs = active_suite.default_active_search_suite_specs(
            include_hocbf=True,
            include_guard=True,
            include_predictive_gate=True,
        )

        self.assertEqual(
            [spec.label for spec in specs],
            ["AS_COVERAGE", "AS_ACTIVE", "AS_RELAY", "AS_HOCBF", "AS_HOCBF_GUARD", "AS_HOCBF_PRED"],
        )
        self.assertEqual(specs[-1].source_row, "R4")
        self.assertTrue(specs[-1].enable_nominal_guard)
        self.assertEqual(specs[-1].search_policy, "active-predictive")

        base = json.loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)
            rows = active_suite.run_active_search_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "paper",
                trials=1,
                base_seed=20260617,
                data_dir=tmp / "data",
                include_hocbf=True,
                include_guard=True,
                include_predictive_gate=True,
                dry_run=True,
            )

            self.assertEqual(rows, [])
            pred_path = tmp / "configs" / "as_hocbf_pred_seed20260617_trial000.json"
            self.assertTrue(pred_path.exists())
            pred_config = json.loads(pred_path.read_text())
            self.assertEqual(pred_config["bridge"]["search-policy"], "active-predictive")
            self.assertTrue(pred_config["bridge"]["nominal"]["guard"]["enabled"])
            search = pred_config["bridge"]["search"]
            self.assertGreater(search["predictive-feasibility-weight"], 0.0)
            self.assertGreater(search["predictive-robust-margin"], 0.0)
            self.assertGreaterEqual(search["predictive-horizon"], 2)
            self.assertGreater(search["predictive-step-m"], 0.0)

    def test_paper_suite_artifacts_use_paper_specific_prefixes(self):
        from active_search_experiments import run_full_simulator_active_bridge as active_suite
        from link_denied_experiments import run_full_simulator_link_denied_bridge as link_denied_suite

        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = pathlib.Path(tmpdir)
            link_data = output_dir / "ld_relay_data.json"
            active_data = output_dir / "as_active_data.json"
            self.write_minimal_valid_suite_data(link_data, "LD_RELAY")
            self.write_minimal_valid_suite_data(active_data, "AS_ACTIVE")

            link_paths = link_denied_suite.write_link_denied_bridge_artifacts(
                [self.make_suite_metric_row("LD_RELAY", link_data)],
                output_dir,
            )
            active_paths = active_suite.write_active_search_bridge_artifacts(
                [self.make_suite_metric_row("AS_ACTIVE", active_data)],
                output_dir,
            )

            self.assertTrue(link_paths["trials_csv"].name.startswith("link_denied_full_bridge"))
            self.assertTrue(link_paths["aggregate_csv"].name.startswith("link_denied_full_bridge"))
            self.assertTrue(link_paths["validation_csv"].name.startswith("link_denied_full_bridge"))
            self.assertTrue(active_paths["trials_csv"].name.startswith("active_search_full_bridge"))
            self.assertTrue(active_paths["aggregate_csv"].name.startswith("active_search_full_bridge"))
            self.assertTrue(active_paths["validation_csv"].name.startswith("active_search_full_bridge"))
            for path in list(link_paths.values()) + list(active_paths.values()):
                self.assertTrue(path.exists(), path)
                self.assertGreater(path.stat().st_size, 0, path)
            self.assertIn("passed", link_paths["validation_csv"].read_text())
            self.assertIn("1", active_paths["validation_csv"].read_text())

    def test_completion_stress_validation_requires_search_completion(self):
        from link_denied_experiments import run_full_simulator_link_denied_bridge as link_denied_suite

        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = pathlib.Path(tmpdir)
            data_path = output_dir / "ld_fixed_data.json"
            self.write_minimal_valid_suite_data(data_path, "LD_COMPLETION_FIXED")
            row = self.make_suite_metric_row("LD_COMPLETION_FIXED", data_path)
            row["coverage_completed"] = 0.0

            paths = link_denied_suite.write_link_denied_bridge_artifacts(
                [row],
                output_dir,
                require_coverage_completion=True,
            )

            with paths["validation_csv"].open(newline="", encoding="utf-8") as handle:
                validation_row = next(csv.DictReader(handle))
            self.assertEqual(float(validation_row["passed"]), 0.0)
            self.assertEqual(float(validation_row["bridge_coverage_completed"]), 0.0)
            self.assertGreater(float(validation_row["error_count"]), 0.0)

    def test_active_completion_validation_accepts_detection_as_search_completion(self):
        from active_search_experiments import run_full_simulator_active_bridge as active_suite

        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = pathlib.Path(tmpdir)
            detected_data = output_dir / "as_detected_data.json"
            incomplete_data = output_dir / "as_incomplete_data.json"
            self.write_minimal_valid_suite_data(detected_data, "AS_COMPLETION_ACTIVE")
            self.write_minimal_valid_suite_data(incomplete_data, "AS_COMPLETION_FALLBACK")
            detected_row = self.make_suite_metric_row("AS_COMPLETION_ACTIVE", detected_data)
            detected_row["coverage_completed"] = 0.0
            detected_row["detected"] = 1.0
            incomplete_row = self.make_suite_metric_row("AS_COMPLETION_FALLBACK", incomplete_data)
            incomplete_row["coverage_completed"] = 0.0
            incomplete_row["detected"] = 0.0

            paths = active_suite.write_active_search_bridge_artifacts(
                [detected_row, incomplete_row],
                output_dir,
                require_search_completion=True,
            )

            with paths["validation_csv"].open(newline="", encoding="utf-8") as handle:
                validation_rows = list(csv.DictReader(handle))
            by_row = {row["row"]: row for row in validation_rows}
            self.assertEqual(float(by_row["AS_COMPLETION_ACTIVE"]["passed"]), 1.0)
            self.assertEqual(float(by_row["AS_COMPLETION_FALLBACK"]["passed"]), 0.0)
            self.assertGreater(float(by_row["AS_COMPLETION_FALLBACK"]["error_count"]), 0.0)


if __name__ == "__main__":
    unittest.main()
