import math
import pathlib
import sys
import unittest


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts" / "analysis"))

from validate_rationality import RationalityOptions, validate_data


def make_robot(robot_id, x, y, *, status="success", solver_status="optimal", uncertainty=0.0):
    return {
        "id": robot_id,
        "state": {
            "x": x,
            "y": y,
            "battery": 4100.0,
            "yawRad": 0.0,
        },
        "uncertainty": uncertainty,
        "position_covariance": {
            "cov_xx": 0.0,
            "cov_xy": 0.0,
            "cov_yx": 0.0,
            "cov_yy": 0.0,
        },
        "opt": {
            "status": status,
            "result": {
                "vx": 1.0,
                "vy": 0.0,
                "yawRateRad": 0.0,
            },
            "slacks": [0.0, 0.0],
            "solver_info": {
                "status": solver_status,
                "objective_value": 0.0,
            },
        },
        "cbfNoSlack": {
            "fixedCommCBF(#1)": 1.0,
        },
        "cbfSlack": {
            "cvtCBF": -1.0,
        },
    }


def make_data(robot1_frame1=None, robot2_frame1=None, *, formations=None):
    formations = formations or [
        {"id": 1, "anchorIds": [], "baseIds": []},
        {"id": 2, "anchorIds": [1], "baseIds": []},
    ]
    return {
        "config": {
            "num": 2,
            "model": "SingleIntegrate2D",
            "world": {
                "boundary": [[0.0, 0.0], [20.0, 0.0], [20.0, 20.0], [0.0, 20.0]],
                "charge": [],
                "spacing": 1.0,
            },
            "bases": [],
            "cbfs": {
                "without-slack": {
                    "comm-fixed": {
                        "on": True,
                        "max-range": 10.0,
                    },
                    "safety": {
                        "on": True,
                        "safe-distance": 1.0,
                    },
                }
            },
            "execute": {
                "time-step": 1.0,
            },
        },
        "state": [
            {
                "runtime": 0.0,
                "robots": [make_robot(1, 1.0, 1.0), make_robot(2, 4.0, 1.0)],
                "formation": formations,
            },
            {
                "runtime": 1.0,
                "robots": [
                    robot1_frame1 or make_robot(1, 2.0, 1.0),
                    robot2_frame1 or make_robot(2, 5.0, 1.0),
                ],
                "formation": formations,
            },
        ],
    }


def make_double_robot(robot_id, x, y, vx, vy, *, ax=0.5, ay=-0.25):
    return {
        "id": robot_id,
        "state": {
            "x": x,
            "y": y,
            "vx": vx,
            "vy": vy,
            "battery": 4100.0,
            "yawRad": 0.0,
        },
        "uncertainty": 0.0,
        "opt": {
            "status": "success",
            "result": {
                "ax": ax,
                "ay": ay,
                "yawRateRad": 0.0,
            },
            "slacks": [],
            "solver_info": {
                "status": "optimal",
                "objective_value": 0.0,
            },
        },
        "cbfNoSlack": {},
        "cbfSlack": {},
    }


def make_double_integrator_data(next_robot=None):
    return {
        "config": {
            "num": 1,
            "model": "DoubleIntegrate2D",
            "world": {
                "boundary": [[0.0, 0.0], [20.0, 0.0], [20.0, 20.0], [0.0, 20.0]],
                "charge": [],
                "spacing": 1.0,
            },
            "bases": [],
            "cbfs": {
                "without-slack": {
                    "comm-fixed": {"on": False},
                    "safety": {"on": False},
                }
            },
            "execute": {
                "time-step": 1.0,
            },
        },
        "state": [
            {
                "runtime": 0.0,
                "robots": [make_double_robot(1, 1.0, 1.0, 2.0, 0.0)],
                "formation": [{"id": 1, "anchorIds": [], "baseIds": []}],
            },
            {
                "runtime": 1.0,
                "robots": [
                    next_robot or make_double_robot(1, 3.0, 1.0, 2.5, -0.25),
                ],
                "formation": [{"id": 1, "anchorIds": [], "baseIds": []}],
            },
        ],
    }


class ValidateRationalityTest(unittest.TestCase):
    def test_valid_distributed_run_passes(self):
        result = validate_data(make_data(), RationalityOptions())

        self.assertTrue(result.passed, result.format_report())
        self.assertEqual(result.error_count, 0)
        self.assertGreaterEqual(result.metrics["final_coverage"], 0.0)

    def test_rejects_non_finite_values(self):
        data = make_data(robot1_frame1=make_robot(1, math.nan, 1.0))

        result = validate_data(data, RationalityOptions())

        self.assertFalse(result.passed)
        self.assertIn("non-finite", result.format_report())

    def test_rejects_robot_outside_world_boundary(self):
        data = make_data(robot1_frame1=make_robot(1, 30.0, 1.0))

        result = validate_data(data, RationalityOptions())

        self.assertFalse(result.passed)
        self.assertIn("outside world boundary", result.format_report())

    def test_rejects_failed_solver_status(self):
        data = make_data(robot1_frame1=make_robot(1, 2.0, 1.0, status="failed", solver_status="primal_infeasible"))

        result = validate_data(data, RationalityOptions())

        self.assertFalse(result.passed)
        self.assertIn("solver", result.format_report())

    def test_rejects_communication_constraint_violation(self):
        data = make_data(robot2_frame1=make_robot(2, 15.0, 1.0, uncertainty=0.5))

        result = validate_data(data, RationalityOptions())

        self.assertFalse(result.passed)
        self.assertIn("communication constraint", result.format_report())

    def test_valid_double_integrator_run_passes(self):
        result = validate_data(make_double_integrator_data(), RationalityOptions())

        self.assertTrue(result.passed, result.format_report())

    def test_rejects_double_integrator_velocity_mismatch(self):
        data = make_double_integrator_data(
            next_robot=make_double_robot(1, 3.0, 1.0, 3.0, -0.25)
        )

        result = validate_data(data, RationalityOptions())

        self.assertFalse(result.passed)
        self.assertIn("velocity propagation", result.format_report())


if __name__ == "__main__":
    unittest.main()
