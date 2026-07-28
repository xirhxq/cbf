import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class CMakeConfigurationTests(unittest.TestCase):
    def test_no_solver_build_does_not_register_gurobi_process_fixture(self):
        cmake = shutil.which("cmake")
        ctest = shutil.which("ctest")
        if cmake is None or ctest is None:
            self.skipTest("cmake and ctest are required")

        with tempfile.TemporaryDirectory(
            prefix="cbf-no-solver-config-",
            dir=os.environ.get("CBF_TEST_TEMP_ROOT"),
        ) as build_directory:
            configure = subprocess.run(
                [
                    cmake,
                    "-S",
                    str(PROJECT_ROOT),
                    "-B",
                    build_directory,
                    "-DBUILD_TESTING=ON",
                    "-DENABLE_GUROBI=OFF",
                    "-DENABLE_HIGHS=OFF",
                    "-DENABLE_OSQP=OFF",
                ],
                text=True,
                capture_output=True,
                timeout=30,
                check=False,
            )
            self.assertEqual(
                configure.returncode,
                0,
                configure.stdout + configure.stderr,
            )

            listed_tests = subprocess.run(
                [ctest, "--test-dir", build_directory, "-N"],
                text=True,
                capture_output=True,
                timeout=10,
                check=False,
            )
            self.assertEqual(
                listed_tests.returncode,
                0,
                listed_tests.stdout + listed_tests.stderr,
            )
            self.assertNotIn(
                "testSwarmFailureExit",
                listed_tests.stdout + listed_tests.stderr,
            )


if __name__ == "__main__":
    unittest.main()
