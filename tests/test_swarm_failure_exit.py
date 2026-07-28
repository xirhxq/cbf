import json
import os
import re
import subprocess
import tempfile
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SWARM_BINARY = Path(
    os.environ.get(
        "CBF_SWARM_BINARY",
        PROJECT_ROOT / "build-diagnostic" / "Swarm",
    )
)


def single_robot_config(output_root: Path, position: list[float]) -> dict:
    return {
        "world": {
            "boundary": [
                [-10.0, -10.0],
                [10.0, -10.0],
                [10.0, 10.0],
                [-10.0, 10.0],
            ],
            "charge": [],
            "spacing": 1.0,
        },
        "num": 1,
        "dim": 4,
        "formation": {
            "parts": 1,
            "bases-id": [[0]],
        },
        "bases": [[0.0, 0.0]],
        "initial": {
            "position": {
                "method": "specified",
                "positions": [position],
            },
            "battery": {
                "min": 4000.0,
                "max": 4000.0,
            },
            "yawDeg": 0.0,
        },
        "model": "SingleIntegrate2D",
        "model-params": {
            "discharge-rate": 0.5,
        },
        "uncertainty": {
            "method": "const",
            "const": {
                "epsilon": 0.0,
            },
        },
        "position_covariance": {
            "enable": False,
        },
        "searching": {
            "method": "downward",
            "downward": {
                "radius": 1.0,
            },
        },
        "optimiser": "Gurobi",
        "cbfs": {
            "objective-function": {
                "k_delta": 1.0,
            },
            "uncertainty-rate": {
                "mode": "off",
            },
            "input-limits": {
                "on": False,
                "planar-component-max": 25.0,
                "yaw-rate-max": 0.35,
            },
            "with-slack": {
                "cvt": {
                    "on": False,
                },
                "cvt-yaw": {
                    "on": False,
                },
                "target-yaw": {
                    "on": False,
                },
            },
            "without-slack": {
                "method": "all",
                "energy": {
                    "on": False,
                },
                "safety": {
                    "on": False,
                },
                "comm-fixed": {
                    "on": False,
                    "max-range": 100.0,
                    "k": 1.0,
                    "min-neighbour-id-offset": -2,
                    "max-neighbour-id-offset": 0,
                    "compensate-velocity": False,
                    "consider-uncertainty": False,
                    "alpha": {
                        "coe": 0.1,
                        "pow": 1,
                    },
                },
                "comm-auto": {
                    "on": False,
                },
            },
        },
        "execute": {
            "execution-mode": "distributed",
            "time-total": 0.1,
            "time-step": 0.1,
            "random-seed": 7,
            "check-constraint-violation": False,
        },
        "debug": {
            "opt-cbc": False,
        },
        "output_path": str(output_root),
        "run_suffix": "-failure-exit-fixture",
    }


def run_fixture(output_root: Path, position: list[float]) -> subprocess.CompletedProcess:
    config_path = output_root / "config.json"
    config_path.write_text(
        json.dumps(single_robot_config(output_root, position)),
        encoding="utf-8",
    )
    return subprocess.run(
        [str(SWARM_BINARY), str(config_path)],
        cwd=PROJECT_ROOT,
        text=True,
        capture_output=True,
        timeout=10,
        check=False,
    )


def reported_output_directory(stdout: str) -> Path:
    match = re.search(r"^\[OUTPUT_DIR\] (.+)$", stdout, re.MULTILINE)
    if match is None:
        raise AssertionError(f"missing [OUTPUT_DIR] line in stdout:\n{stdout}")
    return Path(match.group(1))


class SwarmFailureExitIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not SWARM_BINARY.is_file():
            raise unittest.SkipTest(f"Swarm binary not found: {SWARM_BINARY}")

    def test_internal_loop_failure_is_nonzero_and_retains_partial_data(self):
        with tempfile.TemporaryDirectory(
            prefix="cbf-swarm-failure-exit-",
            dir="/private/tmp",
        ) as temporary_directory:
            output_root = Path(temporary_directory)
            result = run_fixture(output_root, [100.0, 0.0])

            self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn(
                "[SIMULATION_ERROR] Robot is outside the world",
                result.stderr,
            )

            output_directory = reported_output_directory(result.stdout)
            self.assertEqual(output_directory.parent, output_root)
            data_path = output_directory / "data.json"
            self.assertTrue(data_path.is_file())

            partial_data = json.loads(data_path.read_text(encoding="utf-8"))
            self.assertEqual(len(partial_data["state"]), 1)
            self.assertEqual(partial_data["state"][0]["runtime"], 0.0)

    def test_successful_single_step_run_remains_zero(self):
        with tempfile.TemporaryDirectory(
            prefix="cbf-swarm-success-exit-",
            dir="/private/tmp",
        ) as temporary_directory:
            output_root = Path(temporary_directory)
            result = run_fixture(output_root, [0.0, 0.0])

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertNotIn("[SIMULATION_ERROR]", result.stderr)

            output_directory = reported_output_directory(result.stdout)
            data_path = output_directory / "data.json"
            normal_data = json.loads(data_path.read_text(encoding="utf-8"))
            self.assertEqual(len(normal_data["state"]), 1)


if __name__ == "__main__":
    unittest.main()
