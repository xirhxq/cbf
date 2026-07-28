import hashlib
import io
import json
import subprocess
import tarfile
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch

from scripts.diagnostics.run_diagnostic import (
    DiskSpaceError,
    HARD_FLOOR_BYTES,
    OUTPUT_ROOT_CAP_BYTES,
    RUN_CAP_BYTES,
    START_BYTES,
    allocated_bytes,
    deep_merge,
    main,
    materialize_config,
    require_start_space,
    run_diagnostic,
)


class TerminatedProcess:
    def __init__(self):
        self.returncode = None
        self.terminated = False
        self.killed = False
        self.wait_timeouts = []

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminated = True

    def wait(self, timeout=None):
        self.wait_timeouts.append(timeout)
        self.returncode = -15
        return self.returncode

    def kill(self):
        self.killed = True
        self.returncode = -9


class TimeoutProcess(TerminatedProcess):
    def wait(self, timeout=None):
        self.wait_timeouts.append(timeout)
        if timeout is not None and not self.killed:
            raise subprocess.TimeoutExpired(cmd="Swarm", timeout=timeout)
        self.returncode = -9
        return self.returncode


class CompletedAfterOnePollProcess(TerminatedProcess):
    def __init__(self):
        super().__init__()
        self.poll_count = 0

    def poll(self):
        self.poll_count += 1
        return None if self.poll_count == 1 else 0


class NeverCompletingProcess(TerminatedProcess):
    def poll(self):
        return self.returncode


class ImmediatelyCompletedProcess(TerminatedProcess):
    def __init__(self, returncode):
        super().__init__()
        self.returncode = returncode


def write_minimal_project(project_root: Path) -> Path:
    base_config = {
        "initial": {"position": {"method": "random"}},
        "execute": {
            "time-total": 500,
            "random-seed": 1,
            "check-constraint-violation": False,
        },
        "cbfs": {
            "uncertainty-rate": {"mode": "off"},
            "input-limits": {
                "on": False,
                "planar-component-max": 25.0,
                "yaw-rate-max": 0.35,
            },
            "without-slack": {
                "method": "all",
                "safety": {
                    "on": False,
                    "mode": "minimum",
                    "consider-uncertainty": True,
                    "alpha": {"coe": 0.1, "pow": 1},
                },
                "comm-fixed": {
                    "on": True,
                    "consider-uncertainty": True,
                    "alpha": {"coe": 0.1, "pow": 1},
                },
            }
        },
        "execute": {
            "execution-mode": "distributed",
            "time-total": 500,
            "random-seed": 1,
            "check-constraint-violation": False,
        },
        "optimiser": "Gurobi",
    }
    case_patches = {
        "h0_historical.json": {
            "case": "H0",
            "overrides": {
                "initial": {"position": {"method": "specified"}},
                "execute": {"check-constraint-violation": False},
            },
        },
        "c1_claim_aligned.json": {
            "case": "C1",
            "overrides": {
                "initial": {"position": {"method": "specified"}},
                "execute": {"check-constraint-violation": True},
                "cbfs": {
                    "without-slack": {
                        "safety": {"on": True, "consider-uncertainty": True},
                        "comm-fixed": {"on": True, "consider-uncertainty": True},
                    }
                },
            },
        },
        "u0_uncertainty_ablation.json": {
            "case": "U0",
            "overrides": {
                "initial": {"position": {"method": "specified"}},
                "execute": {"check-constraint-violation": True},
                "cbfs": {
                    "without-slack": {
                        "safety": {"on": True, "consider-uncertainty": False},
                        "comm-fixed": {"on": True, "consider-uncertainty": False},
                    }
                },
            },
        },
        "c0_corrected.json": {
            "case": "C0",
            "overrides": {
                "initial": {"position": {"method": "specified"}},
                "cbfs": {
                    "uncertainty-rate": {"mode": "off"},
                    "input-limits": {"on": False},
                    "without-slack": {
                        "method": "all",
                        "safety": {
                            "on": True,
                            "mode": "minimum",
                            "consider-uncertainty": True,
                            "alpha": {"coe": 0.1, "pow": 1},
                        },
                        "comm-fixed": {
                            "on": True,
                            "consider-uncertainty": True,
                            "alpha": {"coe": 0.1, "pow": 1},
                        },
                    },
                },
                "execute": {
                    "execution-mode": "distributed",
                    "check-constraint-violation": True,
                },
            },
        },
        "r_rate_aware.json": {
            "case": "R",
            "overrides": {
                "initial": {"position": {"method": "specified"}},
                "cbfs": {
                    "uncertainty-rate": {
                        "mode": "backward-difference-positive"
                    },
                    "input-limits": {"on": False},
                    "without-slack": {
                        "method": "all",
                        "safety": {
                            "on": True,
                            "mode": "minimum",
                            "consider-uncertainty": True,
                            "alpha": {"coe": 0.1, "pow": 1},
                        },
                        "comm-fixed": {
                            "on": True,
                            "consider-uncertainty": True,
                            "alpha": {"coe": 0.1, "pow": 1},
                        },
                    },
                },
                "execute": {
                    "execution-mode": "distributed",
                    "check-constraint-violation": True,
                },
            },
        },
        "rb_bounded.json": {
            "case": "RB",
            "overrides": {
                "initial": {"position": {"method": "specified"}},
                "cbfs": {
                    "uncertainty-rate": {
                        "mode": "backward-difference-positive"
                    },
                    "input-limits": {"on": True},
                    "without-slack": {
                        "method": "all",
                        "safety": {
                            "on": True,
                            "mode": "minimum",
                            "consider-uncertainty": True,
                            "alpha": {"coe": 0.1, "pow": 1},
                        },
                        "comm-fixed": {
                            "on": True,
                            "consider-uncertainty": True,
                            "alpha": {"coe": 0.1, "pow": 1},
                        },
                    },
                },
                "execute": {
                    "execution-mode": "distributed",
                    "check-constraint-violation": True,
                },
            },
        },
        "rbp_pairwise.json": {
            "case": "RBP",
            "overrides": {
                "initial": {"position": {"method": "specified"}},
                "cbfs": {
                    "uncertainty-rate": {
                        "mode": "backward-difference-positive"
                    },
                    "input-limits": {"on": True},
                    "without-slack": {
                        "method": "all",
                        "safety": {
                            "on": True,
                            "mode": "pairwise",
                            "consider-uncertainty": True,
                            "alpha": {"coe": 0.1, "pow": 1},
                        },
                        "comm-fixed": {
                            "on": True,
                            "consider-uncertainty": True,
                            "alpha": {"coe": 0.1, "pow": 1},
                        },
                    },
                },
                "execute": {
                    "execution-mode": "distributed",
                    "check-constraint-violation": True,
                },
            },
        },
    }
    (project_root / "config" / "diagnostics").mkdir(parents=True)
    (project_root / "config" / "config.json").write_text(json.dumps(base_config))
    for filename, case_patch in case_patches.items():
        (project_root / "config" / "diagnostics" / filename).write_text(
            json.dumps(case_patch)
        )

    (project_root / "scripts" / "diagnostics").mkdir(parents=True)
    (project_root / "scripts" / "diagnostics" / "__init__.py").write_text(
        '"""Required package marker."""\n'
    )
    (project_root / "scripts" / "diagnostics" / "run_diagnostic.py").write_text(
        '"""Runner source."""\n'
    )
    (project_root / "include").mkdir()
    (project_root / "include" / "untracked-source.hpp").write_text("// source\n")
    (project_root / "tests").mkdir()
    (project_root / "tests" / "test_runner.py").write_text("# test\n")

    (project_root / ".git").mkdir()
    (project_root / ".git" / "index").write_bytes(b"git metadata")
    (project_root / "build-diagnostic").mkdir()
    binary_path = project_root / "build-diagnostic" / "Swarm"
    binary_path.write_bytes(b"diagnostic binary")
    (project_root / ".superpowers" / "sdd").mkdir(parents=True)
    (project_root / ".superpowers" / "sdd" / "private.md").write_text("private\n")
    (project_root / "data-records").mkdir()
    (project_root / "data-records" / "raw.json").write_text("{}\n")
    snapshot_root = project_root / "docs" / "diagnostics" / "source-snapshots"
    snapshot_root.mkdir(parents=True)
    (snapshot_root / "old-source.tar.gz").write_bytes(b"old snapshot")
    return binary_path


def fake_git_output(_project_root: Path, *arguments: str) -> str:
    if arguments == ("rev-parse", "HEAD"):
        return "deadbeef"
    if arguments == ("rev-parse", "--abbrev-ref", "HEAD"):
        return "codex/test"
    if arguments == ("status", "--porcelain", "--untracked-files=all"):
        return "?? include/untracked-source.hpp"
    return "unknown"


def completed_process_factory(log_prefix: str = "run"):
    invocation = 0

    def create_process(arguments, **kwargs):
        nonlocal invocation
        invocation += 1
        kwargs["stdout"].write(f"{log_prefix}-stdout-{invocation}\n")
        kwargs["stderr"].write(f"{log_prefix}-stderr-{invocation}\n")
        config = json.loads(Path(arguments[1]).read_text())
        snapshot_path = Path(config["output_path"]) / "source-snapshot.tar.gz"
        if not snapshot_path.is_file():
            raise AssertionError("source snapshot must exist before simulator launch")
        data_root = Path(config["output_path"]) / f"simulator-child-{invocation}"
        data_root.mkdir()
        (data_root / "data.json").write_text("{}\n")
        return CompletedAfterOnePollProcess()

    return create_process


class DiagnosticRunnerTests(unittest.TestCase):
    def test_deep_merge_preserves_unmodified_nested_values(self):
        base = {
            "execute": {"time-step": 0.5, "time-total": 500},
            "cbfs": {"safety": {"on": False}},
        }
        patch_value = {"execute": {"time-total": 20}}
        self.assertEqual(
            deep_merge(base, patch_value),
            {
                "execute": {"time-step": 0.5, "time-total": 20},
                "cbfs": {"safety": {"on": False}},
            },
        )

    @patch(
        "scripts.diagnostics.run_diagnostic.available_bytes",
        return_value=7_999_999_999,
    )
    def test_start_space_below_eight_gb_aborts(self, _available):
        with self.assertRaises(DiskSpaceError):
            require_start_space(Path("/private/tmp"), 8_000_000_000)

    @patch(
        "scripts.diagnostics.run_diagnostic.available_bytes",
        return_value=9_000_000_000,
    )
    def test_default_start_space_allows_nine_gb(self, _available):
        self.assertEqual(
            require_start_space(Path("/private/tmp")),
            9_000_000_000,
        )

    def test_allocated_bytes_counts_blocks_recursively_without_following_symlinks(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            allocation_root = temporary_path / "allocation-root"
            allocation_root.mkdir()
            payload = allocation_root / "payload.bin"
            payload.write_bytes(b"x" * 8192)
            external = temporary_path / "external.bin"
            external.write_bytes(b"y" * 16384)
            external_link = allocation_root / "external-link"
            external_link.symlink_to(external)

            expected = sum(
                path.lstat().st_blocks * 512
                for path in (allocation_root, payload, external_link)
            )
            self.assertEqual(allocated_bytes(allocation_root), expected)

    def test_failed_start_guard_creates_no_output_directories(self):
        project_root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as temporary_directory:
            existing_ancestor = Path(temporary_directory)
            output_root = existing_ancestor / "new-parent" / "output"
            with patch(
                "scripts.diagnostics.run_diagnostic.available_bytes",
                return_value=START_BYTES - 1,
            ) as mocked_available:
                with self.assertRaises(DiskSpaceError):
                    run_diagnostic(
                        case="H0",
                        horizon_s=20,
                        seed=7,
                        binary_path=existing_ancestor / "unused-Swarm",
                        output_root=output_root,
                        project_root=project_root,
                    )

            self.assertFalse(output_root.exists())
            self.assertFalse(output_root.parent.exists())
            mocked_available.assert_called_once_with(existing_ancestor)

    def test_output_root_equal_to_project_root_is_rejected_before_allocation(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            project_root = Path(temporary_directory) / "project"
            project_root.mkdir()
            binary_path = write_minimal_project(project_root)

            with (
                patch(
                    "scripts.diagnostics.run_diagnostic.available_bytes",
                    return_value=START_BYTES,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic.subprocess.Popen",
                    return_value=CompletedAfterOnePollProcess(),
                ) as mocked_popen,
                patch(
                    "scripts.diagnostics.run_diagnostic._git_output",
                    side_effect=fake_git_output,
                ),
                patch("scripts.diagnostics.run_diagnostic.time.sleep"),
            ):
                with self.assertRaisesRegex(
                    ValueError,
                    "output_root must resolve outside project_root",
                ):
                    run_diagnostic(
                        case="H0",
                        horizon_s=20,
                        seed=7,
                        binary_path=binary_path,
                        output_root=project_root,
                        project_root=project_root,
                    )

            self.assertFalse((project_root / "H0").exists())
            mocked_popen.assert_not_called()

    def test_nested_unignored_output_root_is_rejected_before_allocation(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            project_root = Path(temporary_directory) / "project"
            project_root.mkdir()
            binary_path = write_minimal_project(project_root)
            output_root = project_root / "unignored-results"

            with (
                patch(
                    "scripts.diagnostics.run_diagnostic.available_bytes",
                    return_value=START_BYTES,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic.subprocess.Popen",
                    return_value=CompletedAfterOnePollProcess(),
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic._git_output",
                    side_effect=fake_git_output,
                ),
                patch("scripts.diagnostics.run_diagnostic.time.sleep"),
            ):
                with self.assertRaisesRegex(
                    ValueError,
                    "output_root must resolve outside project_root",
                ):
                    run_diagnostic(
                        case="H0",
                        horizon_s=20,
                        seed=7,
                        binary_path=binary_path,
                        output_root=output_root,
                        project_root=project_root,
                    )

            self.assertFalse(output_root.exists())

    def test_final_output_root_symlink_is_rejected_before_allocation(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            project_root = temporary_path / "project"
            project_root.mkdir()
            binary_path = write_minimal_project(project_root)
            real_output_root = temporary_path / "real-results"
            real_output_root.mkdir()
            output_root = temporary_path / "results-link"
            output_root.symlink_to(real_output_root, target_is_directory=True)

            with (
                patch(
                    "scripts.diagnostics.run_diagnostic.available_bytes",
                    return_value=START_BYTES,
                ) as mocked_available,
                patch(
                    "scripts.diagnostics.run_diagnostic.subprocess.Popen",
                    side_effect=completed_process_factory("symlink-output"),
                ) as mocked_popen,
                patch(
                    "scripts.diagnostics.run_diagnostic._git_output",
                    side_effect=fake_git_output,
                ),
                patch("scripts.diagnostics.run_diagnostic.time.sleep"),
            ):
                with self.assertRaisesRegex(
                    ValueError,
                    "output_root must not be a symbolic link",
                ):
                    run_diagnostic(
                        case="H0",
                        horizon_s=20,
                        seed=7,
                        binary_path=binary_path,
                        output_root=output_root,
                        project_root=project_root,
                    )

            self.assertFalse((real_output_root / "H0").exists())
            mocked_available.assert_not_called()
            mocked_popen.assert_not_called()

    def test_each_case_materializes_intended_safety_and_uncertainty_truth(self):
        project_root = Path(__file__).resolve().parents[1]
        expected_truth = {
            "H0": (
                False,
                True,
                True,
                False,
                "off",
                {"coe": 0.1, "pow": 3},
                {"coe": 0.1, "pow": 3},
            ),
            "C1": (
                True,
                True,
                True,
                True,
                "off",
                {"coe": 0.1, "pow": 3},
                {"coe": 0.1, "pow": 3},
            ),
            "U0": (
                True,
                False,
                False,
                True,
                "off",
                {"coe": 0.1, "pow": 3},
                {"coe": 0.1, "pow": 3},
            ),
        }
        patch_names = {
            "H0": "h0_historical.json",
            "C1": "c1_claim_aligned.json",
            "U0": "u0_uncertainty_ablation.json",
        }
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            for case, expected in expected_truth.items():
                with self.subTest(case=case):
                    config = materialize_config(
                        project_root / "config" / "config.json",
                        project_root
                        / "config"
                        / "diagnostics"
                        / patch_names[case],
                        temporary_path / case / "config.materialized.json",
                        horizon_s=20,
                        seed=7,
                    )
                    safety = config["cbfs"]["without-slack"]["safety"]
                    communication = config["cbfs"]["without-slack"]["comm-fixed"]
                    self.assertEqual(
                        (
                            safety["on"],
                            safety["consider-uncertainty"],
                            communication["consider-uncertainty"],
                            config["execute"]["check-constraint-violation"],
                            config["cbfs"]["uncertainty-rate"]["mode"],
                            safety["alpha"],
                            communication["alpha"],
                        ),
                        expected,
                    )

    def test_mc_first_cases_materialize_exact_distinguishing_truth(self):
        project_root = Path(__file__).resolve().parents[1]
        expected = {
            "C0": ("off", False, "minimum"),
            "R": ("backward-difference-positive", False, "minimum"),
            "RB": ("backward-difference-positive", True, "minimum"),
            "RBP": ("backward-difference-positive", True, "pairwise"),
        }
        patch_names = {
            "C0": "c0_corrected.json",
            "R": "r_rate_aware.json",
            "RB": "rb_bounded.json",
            "RBP": "rbp_pairwise.json",
        }
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            for case, expected_truth in expected.items():
                with self.subTest(case=case):
                    config = materialize_config(
                        project_root / "config" / "config.json",
                        project_root
                        / "config"
                        / "diagnostics"
                        / patch_names[case],
                        temporary_path / case / "config.materialized.json",
                        horizon_s=20,
                        seed=7,
                    )
                    without_slack = config["cbfs"]["without-slack"]
                    actual_truth = (
                        config["cbfs"]["uncertainty-rate"]["mode"],
                        config["cbfs"]["input-limits"]["on"],
                        without_slack["safety"]["mode"],
                    )
                    self.assertEqual(actual_truth, expected_truth)
                    self.assertEqual(config["execute"]["execution-mode"], "distributed")
                    self.assertEqual(without_slack["method"], "all")
                    self.assertEqual(
                        without_slack["safety"]["alpha"],
                        {"coe": 0.1, "pow": 1},
                    )
                    self.assertEqual(
                        without_slack["comm-fixed"]["alpha"],
                        {"coe": 0.1, "pow": 1},
                    )

    def test_rg_materializes_only_the_fixed_geometry_differently_from_r(self):
        project_root = Path(__file__).resolve().parents[1]
        expected_positions = [
            [-1490.0, -120.0],
            [-1487.320508, -164.641016],
            [-1450.0, -140.0],
            [-1447.320508, -184.641016],
            [-1410.0, -160.0],
            [-1407.320508, -204.641016],
            [-1370.0, -180.0],
            [-1490.0, 120.0],
            [-1487.320508, 164.641016],
            [-1450.0, 140.0],
            [-1447.320508, 184.641016],
            [-1410.0, 160.0],
            [-1407.320508, 204.641016],
            [-1370.0, 180.0],
        ]
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            r_config = materialize_config(
                project_root / "config" / "config.json",
                project_root / "config" / "diagnostics" / "r_rate_aware.json",
                temporary_path / "R" / "config.materialized.json",
                horizon_s=20,
                seed=7,
            )
            rg_config = materialize_config(
                project_root / "config" / "config.json",
                project_root / "config" / "diagnostics" / "rg_fixed_geometry.json",
                temporary_path / "RG" / "config.materialized.json",
                horizon_s=20,
                seed=7,
            )

        self.assertEqual(rg_config["initial"]["position"]["method"], "specified")
        self.assertEqual(rg_config["initial"]["position"]["positions"], expected_positions)
        for config in (r_config, rg_config):
            config["initial"]["position"].pop("positions")
            config.pop("output_path")
            config.pop("run_suffix")
        self.assertEqual(rg_config, r_config)

    def test_run_diagnostic_maps_rg_to_the_fixed_geometry_overlay(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            project_root = temporary_path / "project"
            project_root.mkdir()
            binary_path = project_root / "Swarm"
            binary_path.write_bytes(b"diagnostic binary")
            output_root = temporary_path / "results"
            with (
                patch(
                    "scripts.diagnostics.run_diagnostic.available_bytes",
                    return_value=START_BYTES,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic._git_output",
                    side_effect=fake_git_output,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic.materialize_config",
                    side_effect=RuntimeError("stop after mapping"),
                ) as mocked_materialize,
                patch(
                    "scripts.diagnostics.run_diagnostic.subprocess.Popen",
                ) as mocked_popen,
            ):
                manifest = run_diagnostic(
                    case="RG",
                    horizon_s=20,
                    seed=7,
                    binary_path=binary_path,
                    output_root=output_root,
                    project_root=project_root,
                )

        self.assertEqual(manifest["termination_reason"], "runner_setup_error")
        mocked_materialize.assert_called_once()
        self.assertEqual(
            mocked_materialize.call_args.args[1],
            project_root / "config" / "diagnostics" / "rg_fixed_geometry.json",
        )
        mocked_popen.assert_not_called()

    def test_rg_fixed_geometry_reflects_each_squad_across_the_x_axis(self):
        project_root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as temporary_directory:
            config = materialize_config(
                project_root / "config" / "config.json",
                project_root / "config" / "diagnostics" / "rg_fixed_geometry.json",
                Path(temporary_directory) / "RG" / "config.materialized.json",
                horizon_s=20,
                seed=7,
            )

        positions = config["initial"]["position"]["positions"]
        self.assertEqual(len(positions), 14)
        for lower, upper in zip(positions[:7], positions[7:]):
            self.assertEqual(lower[0], upper[0])
            self.assertEqual(lower[1], -upper[1])

    def test_rg_fixed_geometry_has_nonzero_area_for_every_fixed_reference_triple(self):
        project_root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as temporary_directory:
            config = materialize_config(
                project_root / "config" / "config.json",
                project_root / "config" / "diagnostics" / "rg_fixed_geometry.json",
                Path(temporary_directory) / "RG" / "config.materialized.json",
                horizon_s=20,
                seed=7,
            )

        positions = config["initial"]["position"]["positions"]
        for squad in (positions[:7], positions[7:]):
            for first, second, third in zip(squad, squad[1:], squad[2:]):
                signed_area_twice = (
                    (second[0] - first[0]) * (third[1] - first[1])
                    - (second[1] - first[1]) * (third[0] - first[0])
                )
                self.assertNotEqual(signed_area_twice, 0.0)

    def test_cli_accepts_every_mc_first_case(self):
        for case in ("C0", "R", "RG", "RB", "RBP"):
            with self.subTest(case=case):
                completed_manifest = {
                    "termination_reason": "completed",
                    "output_dir": f"/private/tmp/results/{case}/run-id",
                }
                with (
                    patch(
                        "scripts.diagnostics.run_diagnostic.run_diagnostic",
                        return_value=completed_manifest,
                    ) as mocked_run,
                    redirect_stdout(io.StringIO()),
                ):
                    returncode = main(
                        [
                            "--case",
                            case,
                            "--horizon",
                            "20",
                            "--seed",
                            "7",
                            "--binary",
                            "/private/tmp/Swarm",
                            "--output-root",
                            "/private/tmp/results",
                        ]
                    )

                self.assertEqual(returncode, 0)
                self.assertEqual(mocked_run.call_args.kwargs["case"], case)

    def test_two_same_case_runs_preserve_distinct_complete_bundles(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            project_root = temporary_path / "project"
            project_root.mkdir()
            binary_path = write_minimal_project(project_root)
            output_root = temporary_path / "results"
            popen_factory = completed_process_factory()

            with (
                patch(
                    "scripts.diagnostics.run_diagnostic.available_bytes",
                    return_value=START_BYTES,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic.subprocess.Popen",
                    side_effect=popen_factory,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic._git_output",
                    side_effect=fake_git_output,
                ),
                patch("scripts.diagnostics.run_diagnostic.time.sleep"),
            ):
                first_manifest = run_diagnostic(
                    case="C1",
                    horizon_s=20,
                    seed=7,
                    binary_path=binary_path,
                    output_root=output_root,
                    project_root=project_root,
                )
                first_root = Path(first_manifest["output_dir"])
                first_contents = {
                    name: (first_root / name).read_bytes()
                    for name in (
                        "config.materialized.json",
                        "stdout.log",
                        "stderr.log",
                        "manifest.json",
                        "source-snapshot.tar.gz",
                    )
                }

                second_manifest = run_diagnostic(
                    case="C1",
                    horizon_s=20,
                    seed=8,
                    binary_path=binary_path,
                    output_root=output_root,
                    project_root=project_root,
                )

            second_root = Path(second_manifest["output_dir"])
            self.assertNotEqual(first_root, second_root)
            self.assertEqual(first_root.parent, output_root / "C1")
            self.assertEqual(second_root.parent, output_root / "C1")
            self.assertEqual(
                {
                    path.name
                    for path in (output_root / "C1").iterdir()
                    if path.is_dir()
                },
                {first_root.name, second_root.name},
            )
            for name, original_content in first_contents.items():
                self.assertEqual((first_root / name).read_bytes(), original_content)
                self.assertTrue((second_root / name).is_file())
            self.assertEqual(
                json.loads(
                    (first_root / "config.materialized.json").read_text()
                )["output_path"],
                str(first_root),
            )
            self.assertEqual(
                json.loads(
                    (second_root / "config.materialized.json").read_text()
                )["output_path"],
                str(second_root),
            )
            self.assertTrue((first_root / "simulator-child-1" / "data.json").is_file())
            self.assertTrue((second_root / "simulator-child-2" / "data.json").is_file())

    def test_source_snapshot_hash_and_policy_bind_manifest(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            project_root = temporary_path / "project"
            project_root.mkdir()
            binary_path = write_minimal_project(project_root)
            output_root = temporary_path / "data-output"

            with (
                patch(
                    "scripts.diagnostics.run_diagnostic.available_bytes",
                    return_value=START_BYTES,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic.subprocess.Popen",
                    side_effect=completed_process_factory("snapshot"),
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic._git_output",
                    side_effect=fake_git_output,
                ),
                patch("scripts.diagnostics.run_diagnostic.time.sleep"),
            ):
                manifest = run_diagnostic(
                    case="U0",
                    horizon_s=20,
                    seed=7,
                    binary_path=binary_path,
                    output_root=output_root,
                    project_root=project_root,
                )

            run_root = Path(manifest["output_dir"])
            snapshot_path = run_root / manifest["source_snapshot_path"]
            self.assertEqual(manifest["base_commit"], "deadbeef")
            self.assertTrue(manifest["working_tree_dirty"])
            self.assertEqual(manifest["source_snapshot_format"], "tar.gz")
            self.assertEqual(
                manifest["source_snapshot_policy"],
                "cbf2026-source-snapshot-v1",
            )
            self.assertEqual(
                manifest["source_snapshot_sha256"],
                hashlib.sha256(snapshot_path.read_bytes()).hexdigest(),
            )
            with tarfile.open(snapshot_path, "r:gz") as archive:
                names = set(archive.getnames())
            self.assertIn("scripts/diagnostics/__init__.py", names)
            self.assertIn("scripts/diagnostics/run_diagnostic.py", names)
            self.assertIn("include/untracked-source.hpp", names)
            self.assertIn("tests/test_runner.py", names)
            excluded_prefixes = (
                ".git/",
                ".superpowers/",
                "build-diagnostic/",
                "data-records/",
                "docs/diagnostics/source-snapshots/",
            )
            for name in names:
                self.assertFalse(name.startswith(excluded_prefixes), name)

    def test_working_tree_state_is_captured_before_output_creation(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            project_root = temporary_path / "project"
            project_root.mkdir()
            binary_path = write_minimal_project(project_root)
            output_root = temporary_path / "results"

            def pre_output_git_state(_project_root, *arguments):
                self.assertFalse(output_root.exists())
                if arguments == ("rev-parse", "HEAD"):
                    return "deadbeef"
                if arguments == ("rev-parse", "--abbrev-ref", "HEAD"):
                    return "codex/test"
                if arguments == ("status", "--porcelain", "--untracked-files=all"):
                    return ""
                return "unknown"

            with (
                patch(
                    "scripts.diagnostics.run_diagnostic.available_bytes",
                    return_value=START_BYTES,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic.subprocess.Popen",
                    side_effect=completed_process_factory("clean-source"),
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic._git_output",
                    side_effect=pre_output_git_state,
                ),
                patch("scripts.diagnostics.run_diagnostic.time.sleep"),
            ):
                manifest = run_diagnostic(
                    case="H0",
                    horizon_s=20,
                    seed=7,
                    binary_path=binary_path,
                    output_root=output_root,
                    project_root=project_root,
                )

            self.assertFalse(manifest["working_tree_dirty"])

    def test_popen_failure_writes_complete_terminal_manifest(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            project_root = temporary_path / "project"
            project_root.mkdir()
            binary_path = write_minimal_project(project_root)
            output_root = temporary_path / "results"
            launch_error = PermissionError("simulator is not executable")

            with (
                patch(
                    "scripts.diagnostics.run_diagnostic.available_bytes",
                    side_effect=[START_BYTES, START_BYTES - 1],
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic.subprocess.Popen",
                    side_effect=launch_error,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic._git_output",
                    side_effect=fake_git_output,
                ),
            ):
                try:
                    manifest = run_diagnostic(
                        case="C1",
                        horizon_s=20,
                        seed=7,
                        binary_path=binary_path,
                        output_root=output_root,
                        project_root=project_root,
                    )
                except PermissionError as error:
                    self.fail(f"launch error escaped without a manifest: {error}")

            run_roots = list((output_root / "C1").iterdir())
            self.assertEqual(len(run_roots), 1)
            run_root = run_roots[0]
            self.assertEqual(manifest["output_dir"], str(run_root))
            self.assertEqual(
                {
                    path.name
                    for path in run_root.iterdir()
                    if path.is_file()
                },
                {
                    "config.materialized.json",
                    "manifest.json",
                    "source-snapshot.tar.gz",
                    "stderr.log",
                    "stdout.log",
                },
            )
            self.assertEqual(manifest["termination_reason"], "simulator_launch_error")
            self.assertIsNone(manifest["returncode"])
            self.assertEqual(
                manifest["error"],
                {
                    "type": "PermissionError",
                    "message": "simulator is not executable",
                },
            )
            self.assertEqual(manifest["binary_path"], str(binary_path))
            self.assertEqual(
                manifest["binary_sha256"],
                hashlib.sha256(binary_path.read_bytes()).hexdigest(),
            )
            config_path = run_root / "config.materialized.json"
            snapshot_path = run_root / "source-snapshot.tar.gz"
            self.assertEqual(
                manifest["config_sha256"],
                hashlib.sha256(config_path.read_bytes()).hexdigest(),
            )
            self.assertEqual(
                manifest["source_snapshot_sha256"],
                hashlib.sha256(snapshot_path.read_bytes()).hexdigest(),
            )
            self.assertEqual(
                json.loads((run_root / "manifest.json").read_text()),
                manifest,
            )
            self.assertEqual((run_root / "stdout.log").read_text(), "")
            self.assertEqual((run_root / "stderr.log").read_text(), "")

    def test_post_allocation_snapshot_failure_writes_setup_manifest(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            project_root = temporary_path / "project"
            project_root.mkdir()
            binary_path = write_minimal_project(project_root)
            output_root = temporary_path / "results"
            setup_error = OSError("snapshot creation failed")

            with (
                patch(
                    "scripts.diagnostics.run_diagnostic.available_bytes",
                    side_effect=[START_BYTES, START_BYTES - 1],
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic._create_source_snapshot",
                    side_effect=setup_error,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic.subprocess.Popen",
                ) as mocked_popen,
                patch(
                    "scripts.diagnostics.run_diagnostic._git_output",
                    side_effect=fake_git_output,
                ),
            ):
                try:
                    manifest = run_diagnostic(
                        case="U0",
                        horizon_s=20,
                        seed=7,
                        binary_path=binary_path,
                        output_root=output_root,
                        project_root=project_root,
                    )
                except OSError as error:
                    self.fail(f"setup error escaped without a manifest: {error}")

            mocked_popen.assert_not_called()
            run_roots = list((output_root / "U0").iterdir())
            self.assertEqual(len(run_roots), 1)
            run_root = run_roots[0]
            self.assertEqual(manifest["termination_reason"], "runner_setup_error")
            self.assertIsNone(manifest["returncode"])
            self.assertEqual(
                manifest["error"],
                {
                    "type": "OSError",
                    "message": "snapshot creation failed",
                },
            )
            self.assertEqual(manifest["config_path"], "config.materialized.json")
            self.assertEqual(
                manifest["config_sha256"],
                hashlib.sha256(
                    (run_root / "config.materialized.json").read_bytes()
                ).hexdigest(),
            )
            self.assertEqual(manifest["binary_path"], str(binary_path))
            self.assertEqual(
                manifest["binary_sha256"],
                hashlib.sha256(binary_path.read_bytes()).hexdigest(),
            )
            self.assertEqual(
                manifest["source_snapshot_path"],
                "source-snapshot.tar.gz",
            )
            self.assertIsNone(manifest["source_snapshot_sha256"])
            self.assertEqual(
                json.loads((run_root / "manifest.json").read_text()),
                manifest,
            )
            self.assertTrue((run_root / "stdout.log").is_file())
            self.assertTrue((run_root / "stderr.log").is_file())

    def test_snapshot_that_exceeds_run_cap_stops_before_launch_with_terminal_manifest(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            project_root = temporary_path / "project"
            project_root.mkdir()
            binary_path = write_minimal_project(project_root)
            output_root = temporary_path / "results"

            def over_cap_after_snapshot(path):
                if path == output_root:
                    return 100_000_000
                return RUN_CAP_BYTES + 512

            with (
                patch(
                    "scripts.diagnostics.run_diagnostic.available_bytes",
                    return_value=START_BYTES,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic.allocated_bytes",
                    side_effect=over_cap_after_snapshot,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic.subprocess.Popen",
                ) as mocked_popen,
                patch(
                    "scripts.diagnostics.run_diagnostic._git_output",
                    side_effect=fake_git_output,
                ),
            ):
                manifest = run_diagnostic(
                    case="H0",
                    horizon_s=20,
                    seed=7,
                    binary_path=binary_path,
                    output_root=output_root,
                    project_root=project_root,
                )

            mocked_popen.assert_not_called()
            self.assertEqual(manifest["termination_reason"], "cache_run_cap")
            self.assertIsNone(manifest["returncode"])
            self.assertEqual(
                manifest["output_root_allocated_bytes"],
                100_000_000,
            )
            self.assertEqual(
                manifest["run_allocated_bytes"],
                RUN_CAP_BYTES + 512,
            )
            self.assertEqual(
                manifest["output_root_cap_bytes"],
                OUTPUT_ROOT_CAP_BYTES,
            )
            self.assertEqual(manifest["run_cap_bytes"], RUN_CAP_BYTES)
            self.assertEqual(
                json.loads(
                    (Path(manifest["output_dir"]) / "manifest.json").read_text()
                ),
                manifest,
            )

    def test_live_output_root_cap_stops_child_and_writes_terminal_manifest(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            project_root = temporary_path / "project"
            project_root.mkdir()
            binary_path = write_minimal_project(project_root)
            output_root = temporary_path / "results"
            process = NeverCompletingProcess()
            root_probe_count = 0

            def root_breaches_on_live_probe(path):
                nonlocal root_probe_count
                if path == output_root:
                    root_probe_count += 1
                    if root_probe_count == 1:
                        return 100_000_000
                    return OUTPUT_ROOT_CAP_BYTES + 512
                return 50_000_000

            with (
                patch(
                    "scripts.diagnostics.run_diagnostic.available_bytes",
                    return_value=START_BYTES,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic.allocated_bytes",
                    side_effect=root_breaches_on_live_probe,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic.subprocess.Popen",
                    return_value=process,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic._git_output",
                    side_effect=fake_git_output,
                ),
            ):
                manifest = run_diagnostic(
                    case="H0",
                    horizon_s=20,
                    seed=7,
                    binary_path=binary_path,
                    output_root=output_root,
                    project_root=project_root,
                )

            self.assertTrue(process.terminated)
            self.assertFalse(process.killed)
            self.assertEqual(manifest["termination_reason"], "cache_root_cap")
            self.assertEqual(manifest["returncode"], -15)
            self.assertEqual(
                manifest["output_root_allocated_bytes"],
                OUTPUT_ROOT_CAP_BYTES + 512,
            )
            self.assertEqual(manifest["run_allocated_bytes"], 50_000_000)
            self.assertEqual(
                manifest["output_root_cap_bytes"],
                OUTPUT_ROOT_CAP_BYTES,
            )
            self.assertEqual(manifest["run_cap_bytes"], RUN_CAP_BYTES)
            self.assertEqual(
                json.loads(
                    (Path(manifest["output_dir"]) / "manifest.json").read_text()
                ),
                manifest,
            )

    def test_live_run_cap_stops_child_and_writes_terminal_manifest(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            project_root = temporary_path / "project"
            project_root.mkdir()
            binary_path = write_minimal_project(project_root)
            output_root = temporary_path / "results"
            process = NeverCompletingProcess()
            run_probe_count = 0

            def run_breaches_on_live_probe(path):
                nonlocal run_probe_count
                if path == output_root:
                    return 100_000_000
                run_probe_count += 1
                if run_probe_count == 1:
                    return 50_000_000
                return RUN_CAP_BYTES + 512

            with (
                patch(
                    "scripts.diagnostics.run_diagnostic.available_bytes",
                    return_value=START_BYTES,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic.allocated_bytes",
                    side_effect=run_breaches_on_live_probe,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic.subprocess.Popen",
                    return_value=process,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic._git_output",
                    side_effect=fake_git_output,
                ),
            ):
                manifest = run_diagnostic(
                    case="H0",
                    horizon_s=20,
                    seed=7,
                    binary_path=binary_path,
                    output_root=output_root,
                    project_root=project_root,
                )

            self.assertTrue(process.terminated)
            self.assertFalse(process.killed)
            self.assertEqual(manifest["termination_reason"], "cache_run_cap")
            self.assertEqual(manifest["returncode"], -15)
            self.assertEqual(
                manifest["output_root_allocated_bytes"],
                100_000_000,
            )
            self.assertEqual(
                manifest["run_allocated_bytes"],
                RUN_CAP_BYTES + 512,
            )
            self.assertEqual(
                manifest["output_root_cap_bytes"],
                OUTPUT_ROOT_CAP_BYTES,
            )
            self.assertEqual(manifest["run_cap_bytes"], RUN_CAP_BYTES)
            self.assertEqual(
                json.loads(
                    (Path(manifest["output_dir"]) / "manifest.json").read_text()
                ),
                manifest,
            )

    def test_monitor_probe_error_kills_child_and_writes_terminal_manifest(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            project_root = temporary_path / "project"
            project_root.mkdir()
            binary_path = write_minimal_project(project_root)
            output_root = temporary_path / "results"
            process = TimeoutProcess()

            with (
                patch(
                    "scripts.diagnostics.run_diagnostic.available_bytes",
                    side_effect=[
                        START_BYTES,
                        OSError("live disk probe failed"),
                        OSError("final disk probe failed"),
                    ],
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic.subprocess.Popen",
                    return_value=process,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic._git_output",
                    side_effect=fake_git_output,
                ),
            ):
                try:
                    manifest = run_diagnostic(
                        case="C1",
                        horizon_s=20,
                        seed=7,
                        binary_path=binary_path,
                        output_root=output_root,
                        project_root=project_root,
                    )
                except OSError as error:
                    self.fail(f"monitor error escaped with a live child: {error}")

            self.assertTrue(process.terminated)
            self.assertTrue(process.killed)
            self.assertEqual(process.wait_timeouts, [5, None])
            self.assertEqual(process.returncode, -9)
            self.assertEqual(manifest["termination_reason"], "runner_monitor_error")
            self.assertEqual(manifest["returncode"], -9)
            self.assertEqual(
                manifest["error"],
                {
                    "type": "OSError",
                    "message": "live disk probe failed",
                },
            )
            self.assertIsNone(manifest["free_bytes_after"])
            run_root = Path(manifest["output_dir"])
            self.assertEqual(
                json.loads((run_root / "manifest.json").read_text()),
                manifest,
            )

    def test_simulator_nonzero_and_signal_write_terminal_manifests(self):
        cases = [
            (7, "simulator_nonzero_exit"),
            (-9, "simulator_signal"),
        ]
        for child_returncode, expected_reason in cases:
            with self.subTest(child_returncode=child_returncode):
                with tempfile.TemporaryDirectory() as temporary_directory:
                    temporary_path = Path(temporary_directory)
                    project_root = temporary_path / "project"
                    project_root.mkdir()
                    binary_path = write_minimal_project(project_root)
                    output_root = temporary_path / "results"
                    process = ImmediatelyCompletedProcess(child_returncode)

                    with (
                        patch(
                            "scripts.diagnostics.run_diagnostic.available_bytes",
                            return_value=START_BYTES,
                        ),
                        patch(
                            "scripts.diagnostics.run_diagnostic.allocated_bytes",
                            return_value=50_000_000,
                        ),
                        patch(
                            "scripts.diagnostics.run_diagnostic.subprocess.Popen",
                            return_value=process,
                        ),
                        patch(
                            "scripts.diagnostics.run_diagnostic._git_output",
                            side_effect=fake_git_output,
                        ),
                    ):
                        manifest = run_diagnostic(
                            case="H0",
                            horizon_s=20,
                            seed=7,
                            binary_path=binary_path,
                            output_root=output_root,
                            project_root=project_root,
                        )

                    self.assertEqual(
                        manifest["termination_reason"],
                        expected_reason,
                    )
                    self.assertEqual(manifest["returncode"], child_returncode)
                    self.assertEqual(
                        json.loads(
                            (
                                Path(manifest["output_dir"]) / "manifest.json"
                            ).read_text()
                        ),
                        manifest,
                    )

    def test_cli_returns_nonzero_after_complete_runner_failure_manifest(self):
        for termination_reason in (
            "runner_setup_error",
            "simulator_launch_error",
            "runner_monitor_error",
            "simulator_nonzero_exit",
            "simulator_signal",
            "disk_hard_floor",
            "cache_root_cap",
            "cache_run_cap",
        ):
            with self.subTest(termination_reason=termination_reason):
                failure_manifest = {
                    "termination_reason": termination_reason,
                    "output_dir": "/private/tmp/results/C1/run-id",
                }
                with (
                    patch(
                        "scripts.diagnostics.run_diagnostic.run_diagnostic",
                        return_value=failure_manifest,
                    ),
                    redirect_stdout(io.StringIO()) as standard_output,
                ):
                    returncode = main(
                        [
                            "--case",
                            "C1",
                            "--horizon",
                            "20",
                            "--seed",
                            "7",
                            "--binary",
                            "/private/tmp/Swarm",
                            "--output-root",
                            "/private/tmp/results",
                        ]
                    )

                self.assertEqual(returncode, 2)
                self.assertEqual(
                    json.loads(standard_output.getvalue()),
                    failure_manifest,
                )

    def test_timeout_after_hard_floor_terminate_escalates_to_kill(self):
        project_root = Path(__file__).resolve().parents[1]
        process = TimeoutProcess()
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            binary_path = temporary_path / "Swarm"
            binary_path.write_bytes(b"diagnostic binary")
            output_root = temporary_path / "output"

            with (
                patch(
                    "scripts.diagnostics.run_diagnostic.available_bytes",
                    side_effect=[
                        START_BYTES,
                        HARD_FLOOR_BYTES - 1,
                        HARD_FLOOR_BYTES - 2,
                    ],
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic.subprocess.Popen",
                    return_value=process,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic._git_output",
                    return_value="test-value",
                ),
            ):
                manifest = run_diagnostic(
                    case="H0",
                    horizon_s=20,
                    seed=7,
                    binary_path=binary_path,
                    output_root=output_root,
                    project_root=project_root,
                )

            self.assertTrue(process.terminated)
            self.assertTrue(process.killed)
            self.assertEqual(process.wait_timeouts, [5, None])
            self.assertEqual(manifest["termination_reason"], "disk_hard_floor")
            self.assertEqual(manifest["returncode"], -9)
            self.assertEqual(manifest["free_bytes_before"], START_BYTES)
            self.assertEqual(manifest["free_bytes_after"], HARD_FLOOR_BYTES - 2)
            self.assertEqual(
                json.loads(
                    (Path(manifest["output_dir"]) / "manifest.json").read_text()
                ),
                manifest,
            )

    def test_seven_gb_during_execution_is_above_live_hard_floor(self):
        project_root = Path(__file__).resolve().parents[1]
        process = CompletedAfterOnePollProcess()
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            binary_path = temporary_path / "Swarm"
            binary_path.write_bytes(b"diagnostic binary")
            output_root = temporary_path / "output"

            with (
                patch(
                    "scripts.diagnostics.run_diagnostic.available_bytes",
                    side_effect=[
                        11_000_000_000,
                        7_000_000_000,
                        7_000_000_000,
                    ],
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic.subprocess.Popen",
                    return_value=process,
                ),
                patch(
                    "scripts.diagnostics.run_diagnostic._git_output",
                    return_value="test-value",
                ),
                patch("scripts.diagnostics.run_diagnostic.time.sleep"),
            ):
                manifest = run_diagnostic(
                    case="H0",
                    horizon_s=20,
                    seed=7,
                    binary_path=binary_path,
                    output_root=output_root,
                    project_root=project_root,
                )

        self.assertFalse(process.terminated)
        self.assertEqual(manifest["termination_reason"], "completed")
        self.assertEqual(manifest["free_bytes_after"], 7_000_000_000)
