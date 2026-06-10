import pathlib
import sys
import unittest


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts" / "analysis"))

from run_repro_benchmark import BenchmarkRun, parse_output_dir, summarize_runs


class RunReproBenchmarkTest(unittest.TestCase):
    def test_parse_output_dir_uses_last_marker(self):
        output = """first line
[OUTPUT_DIR] /tmp/old
Data saved in /tmp/new/data.json
[OUTPUT_DIR] /tmp/new
"""

        self.assertEqual(parse_output_dir(output), pathlib.Path("/tmp/new"))

    def test_summarize_runs_reports_timing_rationality_and_state_reproducibility(self):
        runs = [
            BenchmarkRun(
                index=1,
                elapsed_seconds=0.2,
                output_dir=pathlib.Path("/tmp/run1"),
                data_json=pathlib.Path("/tmp/run1/data.json"),
                rationality_passed=True,
                state_checksum="abc123",
                metrics={
                    "solver_failures": 0.0,
                    "max_comm_excess": 0.0,
                },
            ),
            BenchmarkRun(
                index=2,
                elapsed_seconds=0.4,
                output_dir=pathlib.Path("/tmp/run2"),
                data_json=pathlib.Path("/tmp/run2/data.json"),
                rationality_passed=True,
                state_checksum="abc123",
                metrics={
                    "solver_failures": 0.0,
                    "max_comm_excess": 0.0,
                },
            ),
        ]

        summary = summarize_runs(runs)

        self.assertEqual(summary["runs"], 2)
        self.assertTrue(summary["all_rational"])
        self.assertTrue(summary["state_reproducible"])
        self.assertAlmostEqual(summary["elapsed_seconds"]["mean"], 0.3)
        self.assertEqual(summary["elapsed_seconds"]["min"], 0.2)
        self.assertEqual(summary["elapsed_seconds"]["max"], 0.4)
        self.assertEqual(summary["metrics"]["solver_failures"]["max"], 0.0)
        self.assertEqual(summary["metrics"]["max_comm_excess"]["max"], 0.0)


if __name__ == "__main__":
    unittest.main()
