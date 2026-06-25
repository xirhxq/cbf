import pathlib
import sys
import tempfile
import unittest


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from link_denied_experiments.simulator import (
    LinkDeniedConfig,
    LinkState,
    _plot_link_monte_carlo,
    effective_range_variance,
    link_denied_scale_metadata,
    run_link_denied_completion_stress,
    run_link_denied_monte_carlo,
    run_link_denied_severity_sensitivity,
    run_link_denied_range_sensitivity,
    run_link_denied_suite,
    run_link_denied_topology_ablation,
    select_adaptive_topology,
    simulate_link_denied_2d,
    write_link_denied_2d_artifacts,
    write_link_denied_completion_stress_artifacts,
    write_link_denied_monte_carlo_artifacts,
    write_link_denied_severity_sensitivity_artifacts,
    write_link_denied_range_sensitivity_artifacts,
    write_link_denied_topology_ablation_artifacts,
    write_link_denied_artifacts,
)


class LinkDeniedExperimentsTest(unittest.TestCase):
    def test_link_denied_scale_metadata_matches_cbf2026_style_domain(self):
        metadata = link_denied_scale_metadata(
            LinkDeniedConfig(width=30, height=30, steps=400, cell_size_m=100.0, dt_s=1.0)
        )

        self.assertEqual(metadata["width_m"], 3000.0)
        self.assertEqual(metadata["height_m"], 3000.0)
        self.assertEqual(metadata["duration_s"], 400.0)

    def test_effective_variance_increases_with_denial_terms(self):
        healthy = effective_range_variance(
            LinkState(available=True, quality=1.0, dropout_age=0.0, delay=0.0, reference_variance=0.1)
        )
        degraded = effective_range_variance(
            LinkState(available=True, quality=0.2, dropout_age=3.0, delay=1.0, reference_variance=0.8)
        )

        self.assertGreater(degraded, healthy)

    def test_adaptive_topology_uses_relay_to_avoid_denied_direct_link(self):
        config = LinkDeniedConfig(width=30, height=12, d_loc=9.0)
        positions = {
            "beacon": (0.0, 6.0),
            "relay": (8.0, 6.0),
            "searcher": (15.0, 6.0),
        }
        links = {
            ("beacon", "searcher"): LinkState(available=True, quality=0.1, dropout_age=4.0, delay=1.0, reference_variance=0.0),
            ("beacon", "relay"): LinkState(available=True, quality=0.9, dropout_age=0.0, delay=0.0, reference_variance=0.0),
            ("relay", "searcher"): LinkState(available=True, quality=0.8, dropout_age=0.0, delay=0.0, reference_variance=0.2),
        }

        topology = select_adaptive_topology(positions, links, config)

        self.assertEqual(topology.roles["relay"], "relay")
        self.assertEqual(topology.parent["searcher"], "relay")
        self.assertGreaterEqual(topology.min_loc_margin, 0.0)

    def test_suite_shows_adaptive_relay_improves_over_fixed_topology(self):
        results = run_link_denied_suite(LinkDeniedConfig(width=36, height=16, steps=24, seed=14))
        by_method = {row["method"]: row for row in results}

        for row in results:
            self.assertLessEqual(row["fail_safe_ratio"], 1.0)
        self.assertGreaterEqual(by_method["no_denial_fixed"]["certified_ratio"], 0.9)
        self.assertLessEqual(by_method["no_denial_fixed"]["fail_safe_ratio"], 0.1)
        self.assertLess(by_method["fixed_denied"]["min_loc_margin"], 0.0)
        self.assertGreater(
            by_method["adaptive_relay"]["final_coverage"],
            by_method["fixed_denied"]["final_coverage"],
        )
        self.assertGreaterEqual(by_method["adaptive_relay"]["certified_ratio"], 0.9)
        self.assertGreater(by_method["adaptive_relay"]["relay_steps"], 0)

    def test_artifact_writer_creates_summary_and_figures(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = write_link_denied_artifacts(
                pathlib.Path(tmpdir),
                LinkDeniedConfig(width=18, height=8, steps=5, seed=13),
            )

            self.assertTrue(paths["summary_csv"].exists())
            self.assertTrue(paths["progress_png"].exists())
            self.assertTrue(paths["map_png"].exists())
            self.assertIn("adaptive_relay", paths["summary_csv"].read_text())

    def test_2d_denial_zone_requires_relay_for_certified_search_progress(self):
        config = LinkDeniedConfig(width=42, height=24, steps=30, seed=9)
        fixed = simulate_link_denied_2d("fixed_denied", config)
        adaptive = simulate_link_denied_2d("adaptive_relay", config)

        self.assertGreaterEqual(len(adaptive["searcher_names"]), 2)
        self.assertGreater(adaptive["relay_steps"], 0)
        self.assertGreater(adaptive["accepted_switches"], 0)
        self.assertGreater(adaptive["rejected_switches"], 0)
        self.assertGreater(adaptive["final_coverage"], fixed["final_coverage"])
        self.assertGreaterEqual(adaptive["certified_ratio"], 0.9)
        self.assertLess(fixed["certified_ratio"], 0.8)
        self.assertGreaterEqual(adaptive["min_loc_margin"], 0.0)
        self.assertIn("hold", set(fixed["fail_safe_commands"]))

    def test_predictive_relay_switches_before_direct_margin_fails(self):
        config = LinkDeniedConfig(
            width=42,
            height=24,
            steps=30,
            seed=9,
            predictive_margin_reserve=0.35,
        )
        reactive = simulate_link_denied_2d("adaptive_relay", config)
        predictive = simulate_link_denied_2d("adaptive_relay_predictive", config)

        self.assertGreater(predictive["relay_steps"], reactive["relay_steps"])
        self.assertGreaterEqual(predictive["certified_ratio"], reactive["certified_ratio"])
        self.assertGreaterEqual(predictive["min_loc_margin"], 0.0)

    def test_2d_artifact_writer_creates_summary_switch_log_and_map(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = write_link_denied_2d_artifacts(
                pathlib.Path(tmpdir),
                LinkDeniedConfig(width=24, height=14, steps=8, seed=11),
            )

            self.assertTrue(paths["summary_csv"].exists())
            self.assertTrue(paths["switch_log_csv"].exists())
            self.assertTrue(paths["map_png"].exists())
            self.assertIn("adaptive_relay", paths["summary_csv"].read_text())
            self.assertIn("rejected", paths["switch_log_csv"].read_text())

    def test_range_sensitivity_reports_relay_certification_advantage(self):
        config = LinkDeniedConfig(
            width=30,
            height=30,
            steps=400,
            seed=17,
            max_step=0.08,
            sensor_radius=2.5,
            cell_size_m=100.0,
            dt_s=1.0,
        )

        rows = run_link_denied_range_sensitivity(config, d_locs=[9.0, 10.0])

        self.assertEqual([row["d_loc"] for row in rows], [9.0, 10.0])
        for row in rows:
            self.assertGreater(row["relay_certified_ratio"], row["fixed_certified_ratio"])
            self.assertGreater(row["relay_final_coverage"], row["fixed_final_coverage"])
            self.assertGreaterEqual(row["relay_min_loc_margin"], 0.0)

    def test_range_sensitivity_artifact_writer_creates_summary_and_figure(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = write_link_denied_range_sensitivity_artifacts(
                pathlib.Path(tmpdir),
                LinkDeniedConfig(width=18, height=18, steps=60, seed=17, max_step=0.08),
                d_locs=[8.5, 9.0],
            )

            self.assertTrue(paths["summary_csv"].exists())
            self.assertTrue(paths["progress_png"].exists())
            self.assertIn("relay_certified_ratio", paths["summary_csv"].read_text())

    def test_severity_sensitivity_reports_denial_quality_effect(self):
        config = LinkDeniedConfig(
            width=30,
            height=30,
            steps=400,
            seed=17,
            d_loc=10.0,
            max_step=0.08,
            sensor_radius=2.5,
            cell_size_m=100.0,
            dt_s=1.0,
        )

        rows = run_link_denied_severity_sensitivity(config, denied_qualities=[0.28, 0.18, 0.10])

        self.assertEqual([row["denied_quality"] for row in rows], [0.28, 0.18, 0.10])
        self.assertGreater(rows[0]["fixed_certified_ratio"], rows[-1]["fixed_certified_ratio"])
        for row in rows:
            self.assertGreater(row["relay_certified_ratio"], row["fixed_certified_ratio"])
            self.assertGreater(row["relay_final_coverage"], row["fixed_final_coverage"])
            self.assertGreaterEqual(row["relay_min_loc_margin"], 0.0)

    def test_severity_sensitivity_artifact_writer_creates_summary_and_figure(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = write_link_denied_severity_sensitivity_artifacts(
                pathlib.Path(tmpdir),
                LinkDeniedConfig(width=18, height=18, steps=60, seed=17, d_loc=10.0, max_step=0.08),
                denied_qualities=[0.26, 0.18],
            )

            self.assertTrue(paths["summary_csv"].exists())
            self.assertTrue(paths["progress_png"].exists())
            self.assertIn("denied_quality", paths["summary_csv"].read_text())

    def test_monte_carlo_reports_large_scale_denial_zone_statistics(self):
        config = LinkDeniedConfig(
            width=30,
            height=30,
            steps=400,
            seed=17,
            d_loc=10.0,
            max_step=0.08,
            sensor_radius=2.5,
            cell_size_m=100.0,
            dt_s=1.0,
        )

        result = run_link_denied_monte_carlo(
            config,
            trials=4,
            methods=("fixed_denied", "adaptive_relay", "adaptive_relay_predictive"),
        )

        self.assertEqual(result["metadata"]["width_m"], 3000.0)
        self.assertEqual(result["metadata"]["duration_s"], 400.0)
        self.assertEqual(len(result["trials"]), 12)
        by_method = {row["method"]: row for row in result["summary"]}
        self.assertEqual(by_method["fixed_denied"]["trial_count"], 4)
        self.assertEqual(by_method["adaptive_relay"]["trial_count"], 4)
        self.assertEqual(by_method["adaptive_relay_predictive"]["trial_count"], 4)
        self.assertGreater(
            by_method["adaptive_relay"]["mean_certified_ratio"],
            by_method["fixed_denied"]["mean_certified_ratio"],
        )
        self.assertGreater(
            by_method["adaptive_relay"]["mean_final_coverage"],
            by_method["fixed_denied"]["mean_final_coverage"],
        )
        self.assertGreaterEqual(by_method["adaptive_relay"]["success_rate"], 0.0)
        for row in result["summary"]:
            self.assertLessEqual(row["success_rate_ci_low"], row["success_rate"])
            self.assertGreaterEqual(row["success_rate_ci_high"], row["success_rate"])
            self.assertGreaterEqual(row["success_rate_ci_low"], 0.0)
            self.assertLessEqual(row["success_rate_ci_high"], 1.0)
            self.assertGreaterEqual(row["mean_final_coverage_ci_half_width"], 0.0)
            self.assertGreaterEqual(row["mean_certified_ratio_ci_half_width"], 0.0)

    def test_monte_carlo_artifact_writer_creates_summary_trials_and_figure(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = write_link_denied_monte_carlo_artifacts(
                pathlib.Path(tmpdir),
                LinkDeniedConfig(width=18, height=18, steps=60, seed=17, d_loc=10.0, max_step=0.08),
                trials=3,
            )

            self.assertTrue(paths["summary_csv"].exists())
            self.assertTrue(paths["trials_csv"].exists())
            self.assertTrue(paths["progress_png"].exists())
            self.assertIn("mean_certified_ratio", paths["summary_csv"].read_text())
            self.assertIn("success_rate_ci_low", paths["summary_csv"].read_text())
            self.assertIn("mean_certified_ratio_ci_half_width", paths["summary_csv"].read_text())
            self.assertIn("denial_center_x", paths["trials_csv"].read_text())

    def test_monte_carlo_plot_clamps_boundary_wilson_error_bars(self):
        rows = [
            {
                "method": "failed",
                "mean_certified_ratio": 0.4,
                "mean_certified_ratio_ci_half_width": 0.0,
                "mean_final_coverage": 0.1,
                "mean_final_coverage_ci_half_width": 0.0,
                "success_rate": 0.0,
                "success_rate_ci_low": 0.0,
                "success_rate_ci_high": 0.074103,
            },
            {
                "method": "perfect",
                "mean_certified_ratio": 1.0,
                "mean_certified_ratio_ci_half_width": 0.0,
                "mean_final_coverage": 0.3,
                "mean_final_coverage_ci_half_width": 0.0,
                "success_rate": 1.0,
                "success_rate_ci_low": 0.925897,
                "success_rate_ci_high": 0.9999999999999999,
            },
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            output_path = pathlib.Path(tmpdir) / "monte_carlo.png"

            _plot_link_monte_carlo(rows, output_path)

            self.assertTrue(output_path.exists())

    def test_topology_only_ablation_isolates_relay_certification_gain(self):
        config = LinkDeniedConfig(
            width=30,
            height=30,
            steps=400,
            seed=17,
            d_loc=10.0,
            max_step=0.08,
            sensor_radius=2.5,
            cell_size_m=100.0,
            dt_s=1.0,
        )

        result = run_link_denied_topology_ablation(config, trials=4)

        self.assertEqual(result["metadata"]["width_m"], 3000.0)
        self.assertEqual(result["metadata"]["duration_s"], 400.0)
        by_method = {row["method"]: row for row in result["summary"]}
        self.assertLess(
            by_method["fixed_direct"]["mean_certified_ratio"],
            by_method["adaptive_topology"]["mean_certified_ratio"],
        )
        self.assertGreater(
            by_method["adaptive_topology"]["mean_recovery_ratio"],
            0.0,
        )
        self.assertGreaterEqual(
            by_method["adaptive_topology"]["mean_min_margin"],
            by_method["fixed_direct"]["mean_min_margin"],
        )

    def test_topology_ablation_artifact_writer_creates_summary_trials_and_figure(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = write_link_denied_topology_ablation_artifacts(
                pathlib.Path(tmpdir),
                LinkDeniedConfig(width=18, height=18, steps=60, seed=17, d_loc=10.0, max_step=0.08),
                trials=3,
            )

            self.assertTrue(paths["summary_csv"].exists())
            self.assertTrue(paths["trials_csv"].exists())
            self.assertTrue(paths["progress_png"].exists())
            self.assertIn("mean_recovery_ratio", paths["summary_csv"].read_text())
            self.assertIn("direct_certified_ratio", paths["trials_csv"].read_text())

    def test_completion_stress_reports_relay_completion_and_fixed_fail_safe(self):
        config = LinkDeniedConfig(
            width=30,
            height=30,
            steps=400,
            seed=41,
            d_loc=20.0,
            max_step=0.45,
            sensor_radius=2.5,
            cell_size_m=100.0,
            dt_s=1.0,
        )

        result = run_link_denied_completion_stress(config)
        by_method = {row["method"]: row for row in result["summary"]}

        self.assertEqual(result["metadata"]["width_m"], 3000.0)
        self.assertEqual(result["metadata"]["height_m"], 3000.0)
        self.assertEqual(result["metadata"]["duration_s"], 400.0)
        self.assertLess(by_method["fixed_denied"]["final_coverage"], 1.0)
        self.assertGreater(by_method["fixed_denied"]["fail_safe_ratio"], 0.0)
        self.assertEqual(by_method["adaptive_relay"]["final_coverage"], 1.0)
        self.assertLessEqual(by_method["adaptive_relay"]["completion_time_s"], 400.0)
        self.assertEqual(by_method["adaptive_relay"]["fail_safe_ratio"], 0.0)
        self.assertGreater(by_method["adaptive_relay"]["relay_steps"], 0)

        with tempfile.TemporaryDirectory() as tmpdir:
            paths = write_link_denied_completion_stress_artifacts(pathlib.Path(tmpdir), config)
            self.assertTrue(paths["summary_csv"].exists())
            self.assertTrue(paths["progress_png"].exists())
            self.assertTrue(paths["map_png"].exists())
            summary = paths["summary_csv"].read_text()
            self.assertIn("completion_time_s", summary)
            self.assertIn("adaptive_relay", summary)


if __name__ == "__main__":
    unittest.main()
