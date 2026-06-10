import copy
import pathlib
import sys
import unittest


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts" / "analysis"))

from make_short_benchmark_config import build_config


class MakeShortBenchmarkConfigTest(unittest.TestCase):
    def test_build_config_applies_short_run_overrides_without_mutating_base(self):
        base_config = {
            "optimiser": "Gurobi",
            "execute": {
                "execution-mode": "centralized",
                "time-total": 500,
                "time-step": 0.5,
            },
            "output_path": "/old/output",
            "run_suffix": "_old",
            "world": {
                "spacing": 10.0,
            },
        }
        original = copy.deepcopy(base_config)

        config = build_config(
            base_config,
            optimiser="OSQP",
            execution_mode="distributed",
            time_total=5.0,
            random_seed=20260611,
            output_path="/private/tmp/cbf_benchmark_verify",
            run_suffix="_fixed_seed",
        )

        self.assertEqual(config["optimiser"], "OSQP")
        self.assertEqual(config["execute"]["execution-mode"], "distributed")
        self.assertEqual(config["execute"]["time-total"], 5.0)
        self.assertEqual(config["execute"]["random-seed"], 20260611)
        self.assertEqual(config["execute"]["time-step"], 0.5)
        self.assertEqual(config["output_path"], "/private/tmp/cbf_benchmark_verify")
        self.assertEqual(config["run_suffix"], "_fixed_seed")
        self.assertEqual(config["world"], {"spacing": 10.0})
        self.assertEqual(base_config, original)


if __name__ == "__main__":
    unittest.main()
