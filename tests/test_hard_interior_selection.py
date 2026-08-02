import copy
import unittest


class HardInteriorSelectionTests(unittest.TestCase):
    def fixture_problem(self):
        return {
            "owner": 1,
            "control_size": 3,
            "planar_component_max": 25.0,
            "yaw_rate_max": 0.35,
            "snapshot_version": 1,
            "allocation_version": 1,
            "bounds": [
                {"control_index": 0, "coefficient": 1.0, "limit": 25.0},
                {"control_index": 0, "coefficient": -1.0, "limit": 25.0},
                {"control_index": 1, "coefficient": 1.0, "limit": 25.0},
                {"control_index": 1, "coefficient": -1.0, "limit": 25.0},
                {"control_index": 2, "coefficient": 1.0, "limit": 0.35},
                {"control_index": 2, "coefficient": -1.0, "limit": 0.35},
            ],
            "rows": [
                {
                    "edge": {"kind": "localization", "low": 1, "high": 1, "base_id": 0},
                    "owner": 1,
                    "name": "fixedCommCBF(base-0)",
                    "coefficients": [1.0, 1e-11, 0.0],
                    "constant": 1.0,
                    "post_reset_barrier": 1.0,
                    "snapshot_version": 1,
                    "allocation_version": 1,
                }
            ],
            "hard_problem_id": "registered-cpp-near-tie",
        }

    def test_python_chebyshev_matches_registered_cpp_fixture(self):
        from scripts.diagnostics.hard_interior_selection import (
            solve_planar_hard_row_chebyshev,
        )

        audit = solve_planar_hard_row_chebyshev(self.fixture_problem())

        self.assertAlmostEqual(audit.radius_mps, 26.0 - 25.0e-11, places=12)
        self.assertEqual(audit.witness, (25.0, -25.0))
        self.assertEqual(audit.tight_hard_row_indices, (0,))

    def test_floor_uses_selected_witness_radius_not_raw_maximum(self):
        from scripts.diagnostics.hard_interior_selection import (
            frozen_interior_floor,
            solve_planar_hard_row_chebyshev,
        )

        audit = solve_planar_hard_row_chebyshev(self.fixture_problem())

        self.assertAlmostEqual(
            frozen_interior_floor(audit.radius_mps), 0.1, places=12
        )

    def test_rejects_noncanonical_policy_problem_data(self):
        from scripts.diagnostics.hard_interior_selection import (
            solve_planar_hard_row_chebyshev,
        )

        for mutate in (
            lambda problem: problem["rows"][0]["coefficients"].__setitem__(2, 0.1),
            lambda problem: problem["bounds"].pop(),
            lambda problem: problem.__setitem__("planar_component_max", True),
            lambda problem: problem.__setitem__("extra", 1.0),
        ):
            with self.subTest(mutate=mutate):
                problem = self.fixture_problem()
                mutate(problem)
                with self.assertRaises(ValueError):
                    solve_planar_hard_row_chebyshev(problem)

    def test_rejects_integer_continuous_tokens_and_noninteger_discrete_tokens(self):
        from scripts.diagnostics.hard_interior_selection import (
            solve_planar_hard_row_chebyshev,
        )

        mutations = (
            lambda problem: problem.__setitem__("planar_component_max", 25),
            lambda problem: problem["bounds"][0].__setitem__("coefficient", 1),
            lambda problem: problem["rows"][0].__setitem__("constant", 1),
            lambda problem: problem.__setitem__("control_size", 3.0),
            lambda problem: problem.__setitem__("owner", True),
            lambda problem: problem["rows"][0].__setitem__("owner", True),
        )
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                problem = self.fixture_problem()
                mutate(problem)
                with self.assertRaises(ValueError):
                    solve_planar_hard_row_chebyshev(problem)
