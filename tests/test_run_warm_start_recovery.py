"""Contracts for the supervised strict-versus-restart replay parent."""

import gzip
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts.diagnostics.replay_localization_calibration import (
    RESTART_BEFORE_FIRST_FINITE_POLICY,
    STRICT_PREVIOUS_POLICY,
)
from scripts.diagnostics.run_diagnostic import DiskSpaceError
from scripts.diagnostics.run_warm_start_recovery import (
    run_warm_start_recovery,
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class WarmStartRecoveryRunnerTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.project_root = self.root / "project"
        self.project_root.mkdir()
        (self.project_root / "tracked-source.py").write_text("VALUE = 1\n")

        self.data_path = self.root / "data.json"
        self.data_path.write_text(
            json.dumps(
                {
                    "config": {
                        "num": 1,
                        "formation": {"parts": 1},
                        "position_covariance": {"ranging_sigma": 0.5},
                    },
                    "state": [
                        {
                            "robots": [
                                {
                                    "id": 1,
                                    "state": {"x": 1.0, "y": 2.0},
                                }
                            ]
                        }
                    ],
                }
            )
        )
        self.input_manifest_path = self.root / "trajectory-manifest.json"
        self.input_manifest_path.write_text(
            json.dumps(
                {
                    "termination_reason": "completed",
                    "base_commit": "trajectory-commit",
                    "config_sha256": "fixture-config",
                }
            )
        )

        self.immutable_baseline_dir = self.root / "baseline"
        self.immutable_baseline_dir.mkdir()
        summary_path = self.immutable_baseline_dir / "summary.json"
        markdown_path = self.immutable_baseline_dir / "summary.md"
        process_path = self.immutable_baseline_dir / "calibration.jsonl.gz"
        summary_path.write_text('{"status":"fixture"}\n')
        markdown_path.write_text("# Fixture baseline\n")
        with process_path.open("wb") as raw:
            with gzip.GzipFile(fileobj=raw, mode="wb", mtime=0) as compressed:
                compressed.write(b'{"row":1}\n')
        baseline_manifest = {
            "termination_reason": "completed",
            "compressed_process_sha256": _sha256(process_path),
            "decompressed_process_sha256": hashlib.sha256(
                gzip.decompress(process_path.read_bytes())
            ).hexdigest(),
            "summary_json_sha256": _sha256(summary_path),
            "summary_markdown_sha256": _sha256(markdown_path),
        }
        (self.immutable_baseline_dir / "manifest.json").write_text(
            json.dumps(baseline_manifest)
        )
        self.output_root = self.root / "output"

    def tearDown(self):
        self.temporary.cleanup()

    def run_fixture(
        self,
        child_reasons: tuple[str, str] = ("completed", "completed"),
    ):
        def git_output(_project_root, *arguments):
            values = {
                ("rev-parse", "HEAD"): "deadbeef",
                ("branch", "--show-current"): "codex/fixture",
                ("status", "--short", "--untracked-files=no"): "",
            }
            return values[arguments]

        child_manifests = [
            {
                "termination_reason": reason,
                "output_dir": str(self.root / f"child-{index}"),
            }
            for index, reason in enumerate(child_reasons)
        ]
        with patch(
            "scripts.diagnostics.run_warm_start_recovery._git_output",
            side_effect=git_output,
        ), patch(
            "scripts.diagnostics.run_warm_start_recovery.replay_calibration",
            side_effect=child_manifests,
        ) as replay:
            manifest = run_warm_start_recovery(
                self.data_path,
                self.input_manifest_path,
                self.immutable_baseline_dir,
                self.output_root,
                [20260727, 20260728],
                self.project_root,
                1,
            )
        return manifest, replay.call_args_list

    def test_one_parent_runs_both_policies_with_same_inputs_and_seeds(self):
        """Breaks if policies are omitted, reordered, retried, or given different inputs."""
        manifest, child_calls = self.run_fixture()

        self.assertEqual(manifest["termination_reason"], "completed")
        self.assertEqual(len(child_calls), 2)
        self.assertEqual(
            [call.kwargs["initialization_policy"] for call in child_calls],
            [
                STRICT_PREVIOUS_POLICY,
                RESTART_BEFORE_FIRST_FINITE_POLICY,
            ],
        )
        self.assertEqual(child_calls[0].args[0], child_calls[1].args[0])
        self.assertEqual(child_calls[0].args[1], child_calls[1].args[1])
        self.assertEqual(child_calls[0].args[3], child_calls[1].args[3])
        self.assertEqual(child_calls[0].args[5], child_calls[1].args[5])

    def test_parent_records_commit_snapshot_and_baseline_hashes(self):
        """Breaks if the parent cannot identify its executable or external anchor."""
        manifest, _ = self.run_fixture()

        self.assertEqual(manifest["source_commit"], "deadbeef")
        self.assertEqual(len(manifest["source_snapshot_sha256"]), 64)
        self.assertEqual(
            set(manifest["immutable_baseline_hashes"]),
            {
                "manifest.json",
                "summary.json",
                "summary.md",
                "calibration.jsonl.gz",
            },
        )
        self.assertTrue(
            (Path(manifest["output_dir"]) / "source-snapshot.tar.gz").is_file()
        )

    def test_child_failure_prevents_completed_parent_registration(self):
        """Breaks if one failed arm is published as a completed paired run."""
        manifest, child_calls = self.run_fixture(
            child_reasons=("completed", "runner_setup_error")
        )

        self.assertEqual(len(child_calls), 2)
        self.assertEqual(manifest["termination_reason"], "child_failure")
        stored = json.loads(
            (Path(manifest["output_dir"]) / "manifest.json").read_text()
        )
        self.assertEqual(stored["termination_reason"], "child_failure")

    def test_child_exception_is_child_failure_without_retry(self):
        """Breaks if an attempted child exception is mislabeled or automatically retried."""
        def git_output(_project_root, *arguments):
            values = {
                ("rev-parse", "HEAD"): "deadbeef",
                ("branch", "--show-current"): "codex/fixture",
                ("status", "--short", "--untracked-files=no"): "",
            }
            return values[arguments]

        with patch(
            "scripts.diagnostics.run_warm_start_recovery._git_output",
            side_effect=git_output,
        ), patch(
            "scripts.diagnostics.run_warm_start_recovery.replay_calibration",
            side_effect=RuntimeError("synthetic child crash"),
        ) as replay:
            manifest = run_warm_start_recovery(
                self.data_path,
                self.input_manifest_path,
                self.immutable_baseline_dir,
                self.output_root,
                [20260727],
                self.project_root,
                1,
            )

        self.assertEqual(manifest["termination_reason"], "child_failure")
        self.assertEqual(replay.call_count, 1)

    def test_parent_enforces_8gb_6gb_2gb_and_250mb_limits(self):
        """Breaks if the parent launches without the registered disk reserves."""
        with patch(
            "scripts.diagnostics.run_warm_start_recovery.available_bytes",
            return_value=7_999_999_999,
        ):
            with self.assertRaises(DiskSpaceError):
                self.run_fixture()

        for expected, allocation in (
            ("cache_root_cap", lambda path: 2_000_000_001),
            (
                "cache_run_cap",
                lambda path: (
                    250_000_001
                    if Path(path).name.startswith("20")
                    else 0
                ),
            ),
        ):
            with self.subTest(expected=expected), patch(
                "scripts.diagnostics.run_warm_start_recovery.allocated_bytes",
                side_effect=allocation,
            ):
                manifest, _ = self.run_fixture()
            self.assertEqual(manifest["termination_reason"], expected)

        with patch(
            "scripts.diagnostics.run_warm_start_recovery.available_bytes",
            side_effect=[
                9_000_000_000,
                5_999_999_999,
                9_000_000_000,
                9_000_000_000,
            ],
        ):
            manifest, child_calls = self.run_fixture()
        self.assertEqual(manifest["termination_reason"], "disk_hard_floor")
        self.assertEqual(child_calls, [])

    def test_baseline_hash_mismatch_stops_before_parent_allocation(self):
        """Breaks if a mutated immutable baseline can anchor a new intervention."""
        (self.immutable_baseline_dir / "summary.md").write_text("# Mutated\n")

        with self.assertRaisesRegex(ValueError, "immutable baseline"):
            self.run_fixture()

        self.assertFalse(self.output_root.exists())

    def test_dirty_tracked_tree_stops_before_parent_allocation(self):
        """Breaks if a replay can start from an uncommitted executable snapshot."""
        def dirty_git(_project_root, *arguments):
            values = {
                ("rev-parse", "HEAD"): "deadbeef",
                ("branch", "--show-current"): "codex/fixture",
                ("status", "--short", "--untracked-files=no"): " M source.py",
            }
            return values[arguments]

        with patch(
            "scripts.diagnostics.run_warm_start_recovery._git_output",
            side_effect=dirty_git,
        ), patch(
            "scripts.diagnostics.run_warm_start_recovery.replay_calibration"
        ), self.assertRaisesRegex(ValueError, "tracked working tree"):
            run_warm_start_recovery(
                self.data_path,
                self.input_manifest_path,
                self.immutable_baseline_dir,
                self.output_root,
                [20260727],
                self.project_root,
                1,
            )

        self.assertFalse(self.output_root.exists())


if __name__ == "__main__":
    unittest.main()
