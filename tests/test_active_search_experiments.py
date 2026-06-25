import pathlib
import sys
import tempfile
import unittest


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from active_search_experiments.simulator import (
    ActiveSearchConfig,
    choose_active_target,
    choose_feasibility_aware_target,
    choose_predictive_feasibility_target,
    filter_step_by_localization,
    maritime_scale_metadata,
    run_active_completion_stress,
    run_active_fallback_stress,
    run_active_detection_monte_carlo,
    run_active_detection_sensitivity,
    run_active_detection_sensor_sensitivity,
    run_active_search_suite,
    simulate_active_search,
    simulate_active_detection,
    update_belief_with_detection,
    write_active_detection_artifacts,
    write_active_detection_monte_carlo_artifacts,
    write_active_detection_sensitivity_artifacts,
    write_active_detection_sensor_sensitivity_artifacts,
    write_active_completion_stress_artifacts,
    write_active_fallback_stress_artifacts,
    write_active_search_artifacts,
)
from active_search_experiments.run_full_simulator_active_bridge import run_active_search_bridge_suite
from active_search_experiments.run_full_simulator_active_bridge import default_active_search_suite_specs


class ActiveSearchExperimentsTest(unittest.TestCase):
    def test_full_bridge_suite_can_include_task_aware_hocbf_reserve(self):
        specs = default_active_search_suite_specs(
            include_hocbf=True,
            include_predictive_gate=True,
            include_task_aware_reserve=True,
            predictive_reserve_margin=25.0,
        )

        self.assertEqual(specs[-1].label, "AS_HOCBF_TASK_RESERVE")
        self.assertEqual(specs[-1].source_row, "R4")
        self.assertEqual(specs[-1].search_policy, "active-predictive")
        self.assertEqual(specs[-1].topology_policy, "adaptive-chain")
        self.assertTrue(specs[-1].enable_nominal_guard)
        self.assertTrue(specs[-1].enable_predictive_gate)
        self.assertTrue(specs[-1].enable_task_aware_reserve)

        base = __import__("json").loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)

            run_active_search_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "out",
                trials=1,
                base_seed=20261217,
                data_dir=tmp / "data",
                include_hocbf=True,
                include_predictive_gate=True,
                include_task_aware_reserve=True,
                predictive_reserve_margin=25.0,
                dry_run=True,
            )

            configs = sorted((tmp / "configs").glob("as_hocbf_task_reserve_*.json"))
            self.assertEqual(len(configs), 1)
            config = __import__("json").loads(configs[0].read_text())

        self.assertEqual(config["model"], "DoubleIntegrate2D")
        self.assertEqual(config["bridge"]["safety-filter"], "second-order-hocbf")
        self.assertEqual(config["bridge"]["search-policy"], "active-predictive")
        self.assertEqual(config["bridge"]["topology-policy"], "adaptive-chain")
        self.assertEqual(config["bridge"]["topology"]["robust-switch-margin"], 25.0)
        self.assertNotIn("relay-support-guard", config["bridge"]["nominal"])
        support_guard = config["bridge"]["nominal"]["support-chain-guard"]
        self.assertTrue(support_guard["enabled"])
        self.assertEqual(support_guard["scope"], "first-anchor")
        self.assertEqual(support_guard["robust-margin"], 25.0)

    def test_full_bridge_suite_can_include_all_edge_predictive_hocbf_gate(self):
        base = __import__("json").loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)

            run_active_search_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "out",
                trials=1,
                base_seed=20261217,
                data_dir=tmp / "data",
                include_hocbf=True,
                include_guard=True,
                include_predictive_gate=True,
                include_predictive_all_edge_gate=True,
                dry_run=True,
            )

            configs = sorted((tmp / "configs").glob("as_hocbf_pred_ae_*.json"))
            self.assertEqual(len(configs), 1)
            config = __import__("json").loads(configs[0].read_text())

        self.assertEqual(config["bridge"]["search-policy"], "active-predictive")
        self.assertEqual(config["bridge"]["topology-policy"], "adaptive-relay-reserve")
        self.assertTrue(config["bridge"]["nominal"]["relay-support-guard"]["enabled"])
        self.assertTrue(config["bridge"]["nominal"]["support-chain-guard"]["enabled"])
        self.assertEqual(config["bridge"]["nominal"]["support-chain-guard"]["scope"], "all-active-edges")
        self.assertTrue(config["cbfs"]["without-slack"]["comm-fixed"]["state-dependent-reserve"]["enabled"])

    def test_full_bridge_suite_exposes_predictive_gate_tuning(self):
        base = __import__("json").loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)

            run_active_search_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "out",
                trials=1,
                base_seed=20261217,
                data_dir=tmp / "data",
                include_hocbf=True,
                include_predictive_gate=True,
                include_predictive_all_edge_gate=True,
                predictive_gate_weight=0.01,
                predictive_gate_robust_margin=15.0,
                predictive_gate_horizon=5,
                predictive_gate_step_m=80.0,
                dry_run=True,
            )

            configs = sorted((tmp / "configs").glob("as_hocbf_pred_ae_*.json"))
            self.assertEqual(len(configs), 1)
            config = __import__("json").loads(configs[0].read_text())

        search = config["bridge"]["search"]
        self.assertEqual(config["output_path"], str(tmp / "data"))
        self.assertEqual(search["predictive-feasibility-weight"], 0.01)
        self.assertEqual(search["predictive-robust-margin"], 15.0)
        self.assertEqual(search["predictive-horizon"], 5)
        self.assertEqual(search["predictive-step-m"], 80.0)

    def test_full_bridge_suite_can_include_exposure_all_edge_predictive_hocbf_gate(self):
        base = __import__("json").loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)

            run_active_search_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "out",
                trials=1,
                base_seed=20261217,
                data_dir=tmp / "data",
                include_hocbf=True,
                include_predictive_gate=True,
                include_predictive_all_edge_gate=True,
                include_exposure_all_edge_gate=True,
                exposure_weight=160.0,
                dry_run=True,
            )

            configs = sorted((tmp / "configs").glob("as_hocbf_pred_expose_ae_*.json"))
            self.assertEqual(len(configs), 1)
            config = __import__("json").loads(configs[0].read_text())

        self.assertEqual(config["bridge"]["search-policy"], "active-predictive-exposure")
        self.assertEqual(config["bridge"]["topology-policy"], "adaptive-relay-reserve")
        self.assertTrue(config["bridge"]["nominal"]["support-chain-guard"]["enabled"])
        self.assertEqual(config["bridge"]["nominal"]["support-chain-guard"]["scope"], "all-active-edges")
        self.assertTrue(config["cbfs"]["without-slack"]["comm-fixed"]["state-dependent-reserve"]["enabled"])
        search = config["bridge"]["search"]
        self.assertEqual(search["exposure-weight"], 160.0)
        self.assertEqual(search["exposure-radius-m"], 260.0)
        self.assertEqual(search["exposure-half-angle-deg"], 35.0)
        self.assertTrue(search["exposure-unsearched-only"])
        self.assertEqual(search["exposure-lookahead-steps"], 1)
        self.assertEqual(search["exposure-lookahead-discount"], 1.0)

    def test_full_bridge_suite_can_include_horizon_exposure_all_edge_predictive_hocbf_gate(self):
        base = __import__("json").loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        specs = default_active_search_suite_specs(
            include_hocbf=True,
            include_predictive_gate=True,
            include_predictive_all_edge_gate=True,
            include_horizon_exposure_all_edge_gate=True,
        )
        self.assertEqual(specs[-1].label, "AS_HOCBF_PRED_EXPOSE_HORIZON_AE")
        self.assertEqual(specs[-1].search_policy, "active-predictive-exposure")
        self.assertTrue(specs[-1].enable_exposure_gate)

        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)

            run_active_search_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "out",
                trials=1,
                base_seed=20261217,
                data_dir=tmp / "data",
                include_hocbf=True,
                include_predictive_gate=True,
                include_predictive_all_edge_gate=True,
                include_horizon_exposure_all_edge_gate=True,
                exposure_weight=160.0,
                exposure_lookahead_steps=4,
                exposure_lookahead_step_m=120.0,
                exposure_lookahead_discount=0.7,
                dry_run=True,
            )

            configs = sorted((tmp / "configs").glob("as_hocbf_pred_expose_horizon_ae_*.json"))
            self.assertEqual(len(configs), 1)
            config = __import__("json").loads(configs[0].read_text())

        self.assertEqual(config["bridge"]["search-policy"], "active-predictive-exposure")
        self.assertEqual(config["bridge"]["topology-policy"], "adaptive-relay-reserve")
        self.assertTrue(config["bridge"]["nominal"]["support-chain-guard"]["enabled"])
        self.assertEqual(config["bridge"]["nominal"]["support-chain-guard"]["scope"], "all-active-edges")
        self.assertTrue(config["cbfs"]["without-slack"]["comm-fixed"]["state-dependent-reserve"]["enabled"])
        search = config["bridge"]["search"]
        self.assertEqual(search["exposure-weight"], 160.0)
        self.assertEqual(search["exposure-lookahead-steps"], 4)
        self.assertEqual(search["exposure-lookahead-step-m"], 120.0)
        self.assertEqual(search["exposure-lookahead-discount"], 0.7)
        self.assertEqual(search["exposure-service-gate-min-cells"], 0.0)
        self.assertEqual(search["exposure-service-gate-ratio"], 0.0)

    def test_full_bridge_suite_can_include_service_exposure_all_edge_predictive_hocbf_gate(self):
        base = __import__("json").loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        specs = default_active_search_suite_specs(
            include_hocbf=True,
            include_predictive_gate=True,
            include_predictive_all_edge_gate=True,
            include_service_exposure_all_edge_gate=True,
        )
        self.assertEqual(specs[-1].label, "AS_HOCBF_PRED_EXPOSE_SERVICE_AE")
        self.assertEqual(specs[-1].search_policy, "active-predictive-exposure")
        self.assertTrue(specs[-1].enable_exposure_gate)

        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)

            run_active_search_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "out",
                trials=1,
                base_seed=20261217,
                data_dir=tmp / "data",
                include_hocbf=True,
                include_predictive_gate=True,
                include_predictive_all_edge_gate=True,
                include_service_exposure_all_edge_gate=True,
                exposure_weight=160.0,
                exposure_lookahead_steps=4,
                exposure_lookahead_step_m=120.0,
                exposure_lookahead_discount=0.7,
                exposure_service_gate_min_cells=5.0,
                exposure_service_gate_ratio=0.65,
                dry_run=True,
            )

            configs = sorted((tmp / "configs").glob("as_hocbf_pred_expose_service_ae_*.json"))
            self.assertEqual(len(configs), 1)
            config = __import__("json").loads(configs[0].read_text())

        self.assertEqual(config["bridge"]["search-policy"], "active-predictive-exposure")
        self.assertEqual(config["bridge"]["topology-policy"], "adaptive-relay-reserve")
        self.assertTrue(config["bridge"]["nominal"]["support-chain-guard"]["enabled"])
        self.assertEqual(config["bridge"]["nominal"]["support-chain-guard"]["scope"], "all-active-edges")
        self.assertTrue(config["cbfs"]["without-slack"]["comm-fixed"]["state-dependent-reserve"]["enabled"])
        search = config["bridge"]["search"]
        self.assertEqual(search["exposure-weight"], 160.0)
        self.assertEqual(search["exposure-lookahead-steps"], 4)
        self.assertEqual(search["exposure-lookahead-step-m"], 120.0)
        self.assertEqual(search["exposure-lookahead-discount"], 0.7)
        self.assertEqual(search["exposure-service-gate-min-cells"], 5.0)
        self.assertEqual(search["exposure-service-gate-ratio"], 0.65)

    def test_full_bridge_suite_can_include_scheduled_service_exposure_all_edge_predictive_hocbf_gate(self):
        base = __import__("json").loads((PROJECT_ROOT / "config" / "bridge_full_simulator_base.json").read_text())
        specs = default_active_search_suite_specs(
            include_hocbf=True,
            include_predictive_gate=True,
            include_predictive_all_edge_gate=True,
            include_scheduled_service_exposure_all_edge_gate=True,
        )
        self.assertEqual(specs[-1].label, "AS_HOCBF_PRED_EXPOSE_SCHED_AE")
        self.assertEqual(specs[-1].search_policy, "active-predictive-exposure")
        self.assertTrue(specs[-1].enable_exposure_gate)

        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = pathlib.Path(tmpdir)

            run_active_search_bridge_suite(
                base=base,
                config_dir=tmp / "configs",
                binary=PROJECT_ROOT / "build-codex" / "Swarm",
                output_dir=tmp / "out",
                trials=1,
                base_seed=20261217,
                data_dir=tmp / "data",
                include_hocbf=True,
                include_predictive_gate=True,
                include_predictive_all_edge_gate=True,
                include_scheduled_service_exposure_all_edge_gate=True,
                exposure_weight=160.0,
                exposure_lookahead_steps=4,
                exposure_lookahead_step_m=120.0,
                exposure_lookahead_discount=0.7,
                exposure_service_schedule_rate_cells_per_s=0.25,
                exposure_service_schedule_slack_cells=12.0,
                dry_run=True,
            )

            configs = sorted((tmp / "configs").glob("as_hocbf_pred_expose_sched_ae_*.json"))
            self.assertEqual(len(configs), 1)
            config = __import__("json").loads(configs[0].read_text())

        self.assertEqual(config["bridge"]["search-policy"], "active-predictive-exposure")
        self.assertEqual(config["bridge"]["topology-policy"], "adaptive-relay-reserve")
        self.assertTrue(config["bridge"]["nominal"]["support-chain-guard"]["enabled"])
        self.assertEqual(config["bridge"]["nominal"]["support-chain-guard"]["scope"], "all-active-edges")
        self.assertTrue(config["cbfs"]["without-slack"]["comm-fixed"]["state-dependent-reserve"]["enabled"])
        search = config["bridge"]["search"]
        self.assertEqual(search["exposure-weight"], 160.0)
        self.assertEqual(search["exposure-lookahead-steps"], 4)
        self.assertEqual(search["exposure-lookahead-step-m"], 120.0)
        self.assertEqual(search["exposure-lookahead-discount"], 0.7)
        self.assertEqual(search["exposure-service-gate-min-cells"], 0.0)
        self.assertEqual(search["exposure-service-gate-ratio"], 0.0)
        self.assertEqual(search["exposure-service-schedule-rate-cells-per-s"], 0.25)
        self.assertEqual(search["exposure-service-schedule-slack-cells"], 12.0)

    def test_maritime_scale_metadata_matches_cbf2026_style_domain(self):
        metadata = maritime_scale_metadata(
            ActiveSearchConfig(width=30, height=30, steps=400, cell_size_m=100.0, dt_s=1.0)
        )

        self.assertEqual(metadata["width_m"], 3000.0)
        self.assertEqual(metadata["height_m"], 3000.0)
        self.assertEqual(metadata["duration_s"], 400.0)

    def test_active_target_prefers_uncleared_high_belief_cell(self):
        config = ActiveSearchConfig(width=12, height=6, robots=1, steps=1, seed=3)
        clarity = [[1.0 for _ in range(config.width)] for _ in range(config.height)]
        belief = [[0.01 for _ in range(config.width)] for _ in range(config.height)]
        clarity[4][9] = 0.0
        belief[4][9] = 0.9
        clarity[1][2] = 0.0
        belief[1][2] = 0.2

        target = choose_active_target((1.0, 1.0), clarity, belief, config)

        self.assertEqual(target, (9, 4))

    def test_feasibility_aware_target_keeps_localization_reserve(self):
        config = ActiveSearchConfig(
            width=12,
            height=7,
            robots=1,
            steps=1,
            max_step=3.0,
            d_loc=5.0,
            travel_weight=0.0,
            feasibility_weight=8.0,
            feasibility_reserve=0.5,
        )
        clarity = [[1.0 for _ in range(config.width)] for _ in range(config.height)]
        belief = [[0.01 for _ in range(config.width)] for _ in range(config.height)]
        clarity[3][11] = 0.0
        belief[3][11] = 1.0
        clarity[4][4] = 0.0
        belief[4][4] = 0.35

        greedy_target = choose_active_target((4.5, 3.0), clarity, belief, config)
        feasible_target = choose_feasibility_aware_target(
            (4.5, 3.0),
            anchor=(0.0, 3.0),
            clarity=clarity,
            belief=belief,
            config=config,
        )

        self.assertEqual(greedy_target, (11, 3))
        self.assertEqual(feasible_target, (4, 4))

    def test_predictive_feasibility_target_penalizes_future_projection(self):
        config = ActiveSearchConfig(
            width=12,
            height=7,
            robots=1,
            steps=1,
            max_step=2.0,
            d_loc=5.0,
            travel_weight=0.0,
            feasibility_weight=0.0,
            predictive_feasibility_weight=12.0,
            predictive_feasibility_reserve=0.4,
            predictive_horizon=4,
        )
        clarity = [[1.0 for _ in range(config.width)] for _ in range(config.height)]
        belief = [[0.01 for _ in range(config.width)] for _ in range(config.height)]
        clarity[3][11] = 0.0
        belief[3][11] = 1.0
        clarity[4][4] = 0.0
        belief[4][4] = 0.35

        target = choose_predictive_feasibility_target(
            (4.5, 3.0),
            anchor=(0.0, 3.0),
            clarity=clarity,
            belief=belief,
            config=config,
        )

        self.assertEqual(target, (4, 4))

    def test_localization_filter_keeps_robot_inside_anchor_range(self):
        config = ActiveSearchConfig(width=20, height=8, d_loc=5.0, max_step=3.0)
        next_pos, margin = filter_step_by_localization(
            current=(4.0, 2.0),
            desired=(9.0, 2.0),
            anchor=(0.0, 2.0),
            config=config,
        )

        self.assertAlmostEqual(next_pos[0], 5.0)
        self.assertAlmostEqual(next_pos[1], 2.0)
        self.assertGreaterEqual(margin, 0.0)

    def test_suite_distinguishes_unfiltered_and_safety_filtered_active_search(self):
        results = run_active_search_suite(
            ActiveSearchConfig(width=32, height=16, robots=4, steps=24, seed=12)
        )

        by_method = {row["method"]: row for row in results}

        self.assertIn("active_unfiltered", by_method)
        self.assertIn("active_cbf", by_method)
        self.assertIn("active_fallback_cbf", by_method)
        self.assertLess(by_method["active_unfiltered"]["min_loc_margin"], 0.0)
        self.assertGreaterEqual(by_method["active_cbf"]["min_loc_margin"], 0.0)
        self.assertGreaterEqual(by_method["active_cbf"]["min_pair_margin"], 0.0)
        self.assertGreater(by_method["active_cbf"]["final_coverage"], 0.0)
        self.assertGreater(by_method["active_cbf"]["belief_reduction"], 0.0)
        self.assertGreaterEqual(
            by_method["active_fallback_cbf"]["final_coverage"],
            by_method["active_cbf"]["final_coverage"],
        )
        self.assertGreaterEqual(by_method["active_fallback_cbf"]["min_loc_margin"], 0.0)
        self.assertGreaterEqual(by_method["active_fallback_cbf"]["min_pair_margin"], 0.0)

    def test_artifact_writer_creates_summary_and_figures(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = write_active_search_artifacts(
                pathlib.Path(tmpdir),
                ActiveSearchConfig(width=16, height=8, robots=2, steps=5, seed=5),
            )

            self.assertTrue(paths["summary_csv"].exists())
            self.assertTrue(paths["progress_png"].exists())
            self.assertTrue(paths["map_png"].exists())
            self.assertIn("active_cbf", paths["summary_csv"].read_text())

    def test_fallback_stress_exercises_maritime_scale_completion_switch(self):
        config = ActiveSearchConfig(
            width=30,
            height=30,
            robots=4,
            steps=400,
            seed=31,
            max_step=0.45,
            d_loc=18.0,
            sensor_radius=2.5,
            clarity_weight=0.02,
            belief_weight=60.0,
            travel_weight=0.0,
            fallback_period=1,
            cell_size_m=100.0,
            dt_s=1.0,
            prior_center=(3.0, 15.0),
        )

        result = run_active_fallback_stress(config)
        by_method = {row["method"]: row for row in result["summary"]}

        self.assertEqual(result["metadata"]["width_m"], 3000.0)
        self.assertEqual(result["metadata"]["height_m"], 3000.0)
        self.assertEqual(result["metadata"]["duration_s"], 400.0)
        self.assertGreater(
            by_method["active_fallback_cbf"]["final_coverage"],
            by_method["active_cbf"]["final_coverage"],
        )
        self.assertEqual(by_method["active_fallback_cbf"]["final_coverage"], 1.0)
        self.assertGreater(by_method["active_fallback_cbf"]["fallback_switches"], 0)
        self.assertGreaterEqual(by_method["active_fallback_cbf"]["min_loc_margin"], 0.0)
        self.assertGreaterEqual(by_method["active_fallback_cbf"]["min_pair_margin"], 0.0)

        with tempfile.TemporaryDirectory() as tmpdir:
            paths = write_active_fallback_stress_artifacts(pathlib.Path(tmpdir), config)
            self.assertTrue(paths["summary_csv"].exists())
            self.assertTrue(paths["progress_png"].exists())
            summary = paths["summary_csv"].read_text()
            self.assertIn("fallback_switches", summary)
            self.assertIn("active_fallback_cbf", summary)

    def test_safe_coverage_fallback_completes_reachable_large_maritime_grid(self):
        config = ActiveSearchConfig(
            width=30,
            height=30,
            robots=4,
            steps=400,
            seed=31,
            max_step=0.45,
            d_loc=18.0,
            sensor_radius=2.5,
            cell_size_m=100.0,
            dt_s=1.0,
        )

        result = simulate_active_search("coverage_cbf", config)

        self.assertEqual(result["final_coverage"], 1.0)
        self.assertGreaterEqual(result["min_loc_margin"], 0.0)
        self.assertGreaterEqual(result["min_pair_margin"], 0.0)

    def test_completion_stress_artifacts_report_safe_coverage_finish_time(self):
        config = ActiveSearchConfig(
            width=30,
            height=30,
            robots=4,
            steps=400,
            seed=31,
            max_step=0.45,
            d_loc=18.0,
            sensor_radius=2.5,
            cell_size_m=100.0,
            dt_s=1.0,
        )

        result = run_active_completion_stress(config)

        self.assertEqual(result["metadata"]["width_m"], 3000.0)
        self.assertEqual(result["summary"]["method"], "coverage_cbf")
        self.assertEqual(result["summary"]["final_coverage"], 1.0)
        self.assertLessEqual(result["summary"]["completion_time_s"], 400.0)

        with tempfile.TemporaryDirectory() as tmpdir:
            paths = write_active_completion_stress_artifacts(pathlib.Path(tmpdir), config)
            self.assertTrue(paths["summary_csv"].exists())
            self.assertTrue(paths["progress_png"].exists())
            self.assertTrue(paths["map_png"].exists())
            summary = paths["summary_csv"].read_text()
            self.assertIn("completion_time_s", summary)
            self.assertIn("coverage_cbf", summary)

    def test_bayesian_negative_detection_reduces_belief_near_observation(self):
        config = ActiveSearchConfig(width=12, height=6, robots=1, sensor_radius=2.5)
        belief = [[1.0 / (config.width * config.height) for _ in range(config.width)] for _ in range(config.height)]
        updated = update_belief_with_detection(
            belief,
            observer=(6.0, 3.0),
            target=(10, 4),
            detected=False,
            config=config,
        )

        self.assertLess(updated[3][6], belief[3][6])
        self.assertGreater(updated[4][10], belief[4][10])
        self.assertAlmostEqual(sum(sum(row) for row in updated), 1.0, places=6)

    def test_detection_scenario_shows_active_search_finds_target_earlier_under_safety(self):
        config = ActiveSearchConfig(width=32, height=16, robots=4, steps=28, seed=21, d_loc=10.0)
        coverage = simulate_active_detection("coverage_cbf", config)
        active = simulate_active_detection("active_cbf", config)

        self.assertIsNotNone(active["detection_step"])
        self.assertLessEqual(active["detection_step"], coverage["detection_step"])
        self.assertGreater(active["belief_at_target_final"], coverage["belief_at_target_final"])
        self.assertGreaterEqual(active["min_loc_margin"], 0.0)
        self.assertGreaterEqual(active["min_pair_margin"], 0.0)

    def test_detection_artifact_writer_creates_summary_and_figures(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = write_active_detection_artifacts(
                pathlib.Path(tmpdir),
                ActiveSearchConfig(width=20, height=10, robots=3, steps=8, seed=8),
            )

            self.assertTrue(paths["summary_csv"].exists())
            self.assertTrue(paths["progress_png"].exists())
            self.assertTrue(paths["map_png"].exists())
            self.assertIn("detection_step", paths["summary_csv"].read_text())

    def test_detection_sensitivity_reports_active_advantage_under_range_changes(self):
        config = ActiveSearchConfig(
            width=30,
            height=30,
            robots=4,
            steps=400,
            seed=23,
            max_step=0.45,
            sensor_radius=2.5,
            cell_size_m=100.0,
            dt_s=1.0,
        )

        rows = run_active_detection_sensitivity(config, d_locs=[9.0, 10.0])

        self.assertEqual([row["d_loc"] for row in rows], [9.0, 10.0])
        for row in rows:
            self.assertGreater(row["coverage_detection_step"], row["active_detection_step"])
            self.assertGreater(row["detection_gain_s"], 0.0)
            self.assertGreaterEqual(row["active_min_loc_margin"], 0.0)

    def test_detection_sensitivity_artifact_writer_creates_summary_and_figure(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = write_active_detection_sensitivity_artifacts(
                pathlib.Path(tmpdir),
                ActiveSearchConfig(width=18, height=18, robots=4, steps=80, seed=23, max_step=0.45),
                d_locs=[8.0, 9.0],
            )

            self.assertTrue(paths["summary_csv"].exists())
            self.assertTrue(paths["progress_png"].exists())
            self.assertIn("detection_gain_s", paths["summary_csv"].read_text())

    def test_detection_sensor_sensitivity_uses_large_scale_long_horizon_scenario(self):
        config = ActiveSearchConfig(
            width=30,
            height=30,
            robots=4,
            steps=400,
            seed=23,
            max_step=0.45,
            d_loc=10.0,
            cell_size_m=100.0,
            dt_s=1.0,
        )

        rows = run_active_detection_sensor_sensitivity(config, sensor_radii=[2.0, 2.5, 3.0])

        self.assertEqual([row["sensor_radius_m"] for row in rows], [200.0, 250.0, 300.0])
        for row in rows:
            self.assertGreater(row["coverage_detection_step"], row["active_detection_step"])
            self.assertGreater(row["detection_gain_s"], 0.0)
            self.assertGreaterEqual(row["active_min_loc_margin"], 0.0)
            self.assertGreaterEqual(row["active_min_pair_margin"], 0.0)

    def test_detection_sensor_sensitivity_artifact_writer_creates_summary_and_figure(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = write_active_detection_sensor_sensitivity_artifacts(
                pathlib.Path(tmpdir),
                ActiveSearchConfig(width=18, height=18, robots=4, steps=80, seed=23, max_step=0.45),
                sensor_radii=[2.0, 2.4],
            )

            self.assertTrue(paths["summary_csv"].exists())
            self.assertTrue(paths["progress_png"].exists())
            self.assertIn("sensor_radius_m", paths["summary_csv"].read_text())

    def test_detection_monte_carlo_reports_large_scale_success_statistics(self):
        config = ActiveSearchConfig(
            width=30,
            height=30,
            robots=4,
            steps=400,
            seed=23,
            max_step=0.45,
            d_loc=10.0,
            sensor_radius=2.5,
            cell_size_m=100.0,
            dt_s=1.0,
        )

        result = run_active_detection_monte_carlo(config, trials=4)

        self.assertEqual(result["metadata"]["width_m"], 3000.0)
        self.assertEqual(result["metadata"]["duration_s"], 400.0)
        self.assertEqual(len(result["trials"]), 8)
        by_method = {row["method"]: row for row in result["summary"]}
        self.assertEqual(by_method["coverage_cbf"]["trial_count"], 4)
        self.assertEqual(by_method["active_cbf"]["trial_count"], 4)
        for row in result["summary"]:
            self.assertGreaterEqual(row["success_rate"], 0.0)
            self.assertLessEqual(row["success_rate"], 1.0)
            self.assertLessEqual(row["success_rate_ci_low"], row["success_rate"])
            self.assertGreaterEqual(row["success_rate_ci_high"], row["success_rate"])
            self.assertGreaterEqual(row["success_rate_ci_low"], 0.0)
            self.assertLessEqual(row["success_rate_ci_high"], 1.0)
            self.assertGreaterEqual(row["mean_censored_detection_time_ci_half_width_s"], 0.0)
            self.assertGreaterEqual(row["mean_min_loc_margin"], 0.0)

    def test_detection_monte_carlo_accepts_feasibility_aware_active_method(self):
        config = ActiveSearchConfig(
            width=18,
            height=18,
            robots=4,
            steps=80,
            seed=23,
            max_step=0.45,
            d_loc=10.0,
            sensor_radius=2.5,
            cell_size_m=100.0,
            dt_s=1.0,
        )

        result = run_active_detection_monte_carlo(
            config,
            trials=2,
            methods=("coverage_cbf", "active_cbf", "active_feasible_cbf", "active_predictive_cbf"),
        )

        by_method = {row["method"]: row for row in result["summary"]}
        self.assertEqual(by_method["active_feasible_cbf"]["trial_count"], 2)
        self.assertEqual(by_method["active_predictive_cbf"]["trial_count"], 2)
        self.assertGreaterEqual(by_method["active_feasible_cbf"]["mean_min_loc_margin"], 0.0)
        self.assertGreaterEqual(by_method["active_predictive_cbf"]["mean_min_loc_margin"], 0.0)
        self.assertEqual(len(result["trials"]), 8)

    def test_detection_monte_carlo_artifact_writer_creates_summary_trials_and_figure(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = write_active_detection_monte_carlo_artifacts(
                pathlib.Path(tmpdir),
                ActiveSearchConfig(width=18, height=18, robots=4, steps=80, seed=23, max_step=0.45),
                trials=3,
            )

            self.assertTrue(paths["summary_csv"].exists())
            self.assertTrue(paths["trials_csv"].exists())
            self.assertTrue(paths["progress_png"].exists())
            self.assertIn("success_rate", paths["summary_csv"].read_text())
            self.assertIn("success_rate_ci_low", paths["summary_csv"].read_text())
            self.assertIn("mean_censored_detection_time_ci_half_width_s", paths["summary_csv"].read_text())
            self.assertIn("target_x", paths["trials_csv"].read_text())


if __name__ == "__main__":
    unittest.main()
