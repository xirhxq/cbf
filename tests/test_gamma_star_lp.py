import unittest

from scripts.analysis.gamma_star_lp import ResidualConstraint, solve_gamma_star


class GammaStarLPRegressionTest(unittest.TestCase):
    def test_normwise_dual_certificate_and_rounded_box_face(self) -> None:
        cancellation = [
            ResidualConstraint("s2", 0.855605633249312, -0.5176282453190161, 549.83121758651),
            ResidualConstraint("s3", -0.0005031145766689188, -0.9999998734378533, 552.1338252656878),
            ResidualConstraint("c2", -0.855605633249312, 0.5176282453190161, 290.16878241349),
            ResidualConstraint("s1", 0.5028820379848217, -0.8643550519735698, 969.9425289534347),
            ResidualConstraint("c3", 0.0005031145766689188, 0.9999998734378533, 287.86617473431215),
        ]
        box_face = [
            ResidualConstraint("s1", -0.026192013138682767, -0.9996569303754879, 542.5358375359449),
            ResidualConstraint("s4", -0.8608025344865874, 0.5089390893038848, 529.6900316849122),
            ResidualConstraint("c1", 0.026192013138682767, 0.9996569303754879, 297.4641624640551),
            ResidualConstraint("s3", -0.8879898030665115, -0.45986314230420566, 565.5088660327011),
            ResidualConstraint("c0", -0.6651219621762592, 0.7467347423488496, 299.2935648598574),
        ]

        first = solve_gamma_star(cancellation, half_box=2.0)
        second = solve_gamma_star(box_face, half_box=2.0)

        self.assertLessEqual(first.stationarity_residual, 1.0e-15)
        self.assertIn("a_y upper", second.active_box_constraints)
        self.assertLessEqual(second.stationarity_residual, 1.0e-15)


if __name__ == "__main__":
    unittest.main()
