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
    START_BYTES,
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


def write_minimal_project(project_root: Path) -> Path:
    base_config = {
        "initial": {"position": {"method": "random"}},
        "execute": {
            "time-total": 500,
            "random-seed": 1,
            "check-constraint-violation": False,
        },
        "cbfs": {
            "without-slack": {
                "safety": {"on": False, "consider-uncertainty": True},
                "comm-fixed": {"on": True, "consider-uncertainty": True},
            }
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

    def test_each_case_materializes_intended_safety_and_uncertainty_truth(self):
        project_root = Path(__file__).resolve().parents[1]
        expected_truth = {
            "H0": (False, True, True, False),
            "C1": (True, True, True, True),
            "U0": (True, False, False, True),
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
                        ),
                        expected,
                    )

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

    def test_cli_returns_nonzero_after_complete_runner_failure_manifest(self):
        for termination_reason in (
            "runner_setup_error",
            "simulator_launch_error",
            "runner_monitor_error",
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
