"""Contracts for the supervised strict-versus-restart replay parent."""

import gzip
import hashlib
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import scripts.diagnostics.replay_localization_calibration as replay_module
import scripts.diagnostics.run_diagnostic as run_diagnostic_module
import scripts.diagnostics.run_warm_start_recovery as supervisor_module
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

        self.input_bundle = self.root / "input-bundle"
        self.input_bundle.mkdir()
        self.data_path = self.input_bundle / "data.json"
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
        self.input_manifest_path = self.input_bundle / "trajectory-manifest.json"
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
        self.available_patcher = patch.object(
            supervisor_module,
            "available_bytes",
            return_value=9_000_000_000,
        )
        self.available_patcher.start()

    def tearDown(self):
        self.available_patcher.stop()
        self.temporary.cleanup()

    def expected_trust_roots(self) -> dict[str, str]:
        return {
            "expected_data_sha256": _sha256(self.data_path),
            "expected_input_manifest_sha256": _sha256(
                self.input_manifest_path
            ),
            "expected_baseline_manifest_sha256": _sha256(
                self.immutable_baseline_dir / "manifest.json"
            ),
            "expected_supervisor_source_sha256": _sha256(
                Path(supervisor_module.__file__)
            ),
            "expected_replay_source_sha256": _sha256(
                Path(replay_module.__file__)
            ),
            "expected_run_diagnostic_source_sha256": _sha256(
                Path(run_diagnostic_module.__file__)
            ),
        }

    def run_fixture(
        self,
        child_reasons: tuple[str, str] = ("completed", "completed"),
    ):
        def git_output(_project_root, *arguments):
            values = {
                ("rev-parse", "HEAD"): "deadbeef",
                ("branch", "--show-current"): "codex/fixture",
                ("status", "--short"): "?? build-diagnostic/",
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
                **self.expected_trust_roots(),
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
        self.assertEqual(
            manifest["tracked_worktree_status"], "?? build-diagnostic/"
        )
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
                ("status", "--short"): "?? build-diagnostic/",
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
                **self.expected_trust_roots(),
            )

        self.assertEqual(manifest["termination_reason"], "child_failure")
        self.assertEqual(replay.call_count, 1)

    def test_child_live_probe_enforces_parent_cap_and_records_minimum_free(self):
        """Breaks if aggregate limits are invisible while a blocking child runs."""
        inside_child = False

        def git_output(_project_root, *arguments):
            values = {
                ("rev-parse", "HEAD"): "deadbeef",
                ("branch", "--show-current"): "codex/fixture",
                ("status", "--short"): "?? build-diagnostic/",
            }
            return values[arguments]

        def available(_path):
            return 6_250_000_000 if inside_child else 9_000_000_000

        def allocation(path):
            if inside_child and Path(path).name.startswith("20"):
                return 250_000_001
            return 0

        def child(*_arguments, **kwargs):
            nonlocal inside_child
            inside_child = True
            try:
                reason = kwargs["supervisor_probe"]()
            finally:
                inside_child = False
            return {
                "termination_reason": reason or "completed",
                "output_dir": str(self.root / "child"),
            }

        with patch(
            "scripts.diagnostics.run_warm_start_recovery._git_output",
            side_effect=git_output,
        ), patch(
            "scripts.diagnostics.run_warm_start_recovery.available_bytes",
            side_effect=available,
        ), patch(
            "scripts.diagnostics.run_warm_start_recovery.allocated_bytes",
            side_effect=allocation,
        ), patch(
            "scripts.diagnostics.run_warm_start_recovery.replay_calibration",
            side_effect=child,
        ) as replay:
            manifest = run_warm_start_recovery(
                self.data_path,
                self.input_manifest_path,
                self.immutable_baseline_dir,
                self.output_root,
                [20260727],
                self.project_root,
                1,
                **self.expected_trust_roots(),
            )

        self.assertEqual(replay.call_count, 2)
        self.assertEqual(manifest["termination_reason"], "child_failure")
        self.assertEqual(
            manifest["children"]["strict"]["termination_reason"],
            "cache_run_cap",
        )
        self.assertEqual(manifest["minimum_live_free_bytes"], 6_250_000_000)

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

    def test_final_probe_failure_cannot_leave_completed_manifest(self):
        """Breaks if a failing final probe follows publication of completed status."""
        def git_output(_project_root, *arguments):
            values = {
                ("rev-parse", "HEAD"): "deadbeef",
                ("branch", "--show-current"): "codex/fixture",
                ("status", "--short"): "?? build-diagnostic/",
            }
            return values[arguments]

        def available(_path):
            if list(self.output_root.rglob("manifest.json")) or list(
                self.output_root.rglob(".manifest*.tmp")
            ):
                raise OSError("synthetic final disk probe failure")
            return 9_000_000_000

        children = [
            {"termination_reason": "completed", "output_dir": "strict"},
            {"termination_reason": "completed", "output_dir": "restart"},
        ]
        with patch(
            "scripts.diagnostics.run_warm_start_recovery._git_output",
            side_effect=git_output,
        ), patch(
            "scripts.diagnostics.run_warm_start_recovery.available_bytes",
            side_effect=available,
        ), patch(
            "scripts.diagnostics.run_warm_start_recovery.replay_calibration",
            side_effect=children,
        ):
            try:
                run_warm_start_recovery(
                    self.data_path,
                    self.input_manifest_path,
                    self.immutable_baseline_dir,
                    self.output_root,
                    [20260727],
                    self.project_root,
                    1,
                    **self.expected_trust_roots(),
                )
            except OSError:
                pass

        published = list(self.output_root.rglob("manifest.json"))
        self.assertTrue(
            not published
            or all(
                json.loads(path.read_text())["termination_reason"] != "completed"
                for path in published
            )
        )

    def test_output_overlap_with_frozen_inputs_stops_before_allocation(self):
        """Breaks if parent output can be allocated inside or around frozen inputs."""
        forbidden_roots = (
            self.immutable_baseline_dir / "new-output",
            self.input_bundle / "new-output",
            self.root,
        )
        for index, forbidden in enumerate(forbidden_roots):
            with self.subTest(forbidden=forbidden), patch(
                "scripts.diagnostics.run_warm_start_recovery.replay_calibration"
            ) as replay, self.assertRaisesRegex(ValueError, "overlap"):
                run_warm_start_recovery(
                    self.data_path,
                    self.input_manifest_path,
                    self.immutable_baseline_dir,
                    forbidden,
                    [20260727 + index],
                    self.project_root,
                    1,
                    **self.expected_trust_roots(),
                )
            replay.assert_not_called()
            self.assertFalse((forbidden / "warm-start-recovery").exists())

    def test_baseline_hash_mismatch_stops_before_parent_allocation(self):
        """Breaks if a mutated immutable baseline can anchor a new intervention."""
        (self.immutable_baseline_dir / "summary.md").write_text("# Mutated\n")

        with self.assertRaisesRegex(ValueError, "immutable baseline"):
            self.run_fixture()

        self.assertFalse(self.output_root.exists())

    def test_external_trust_root_mismatch_stops_before_allocation(self):
        """Breaks if self-consistent replacement inputs can certify themselves."""
        for key in self.expected_trust_roots():
            with self.subTest(key=key):
                trust_roots = self.expected_trust_roots()
                trust_roots[key] = "0" * 64
                output_root = self.root / f"trust-mismatch-{key}"

                with patch(
                    "scripts.diagnostics.run_warm_start_recovery.replay_calibration"
                ) as replay, self.assertRaisesRegex(ValueError, "external"):
                    run_warm_start_recovery(
                        self.data_path,
                        self.input_manifest_path,
                        self.immutable_baseline_dir,
                        output_root,
                        [20260727],
                        self.project_root,
                        1,
                        **trust_roots,
                    )

                replay.assert_not_called()
                self.assertFalse(output_root.exists())

    def test_cli_requires_every_external_trust_root(self):
        """Breaks if production CLI can derive any trust root from its inputs."""
        with self.assertRaises(SystemExit) as caught:
            supervisor_module.main(
                [
                    "--data",
                    str(self.data_path),
                    "--input-manifest",
                    str(self.input_manifest_path),
                    "--immutable-baseline",
                    str(self.immutable_baseline_dir),
                    "--output-root",
                    str(self.output_root),
                    "--seed",
                    "20260727",
                    "--max-frames",
                    "1",
                ]
            )

        self.assertEqual(caught.exception.code, 2)
        self.assertFalse(self.output_root.exists())

    def test_finalizing_data_or_snapshot_mutation_cannot_publish_completed(self):
        """Breaks if final publication omits a unified input/snapshot recheck."""
        original_data = self.data_path.read_bytes()
        for target in ("data", "snapshot"):
            with self.subTest(target=target):
                output_root = self.root / f"mutation-{target}"
                original_stage = supervisor_module._stage_manifest
                mutated = False

                def stage(path, manifest):
                    nonlocal mutated
                    original_stage(path, manifest)
                    if (
                        not mutated
                        and manifest.get("termination_reason") == "completed"
                    ):
                        mutated = True
                        if target == "data":
                            self.data_path.write_text('{"mutated":true}\n')
                        else:
                            snapshot = (
                                Path(manifest["source_snapshot_path"])
                            )
                            snapshot.write_bytes(
                                snapshot.read_bytes() + b"mutation"
                            )

                def git_output(_project_root, *arguments):
                    values = {
                        ("rev-parse", "HEAD"): "deadbeef",
                        ("branch", "--show-current"): "codex/fixture",
                        ("status", "--short"): "?? build-diagnostic/",
                    }
                    return values[arguments]

                children = [
                    {
                        "termination_reason": "completed",
                        "output_dir": "strict",
                    },
                    {
                        "termination_reason": "completed",
                        "output_dir": "restart",
                    },
                ]
                with patch.object(
                    supervisor_module, "_stage_manifest", side_effect=stage
                ), patch.object(
                    supervisor_module, "_git_output", side_effect=git_output
                ), patch.object(
                    supervisor_module,
                    "replay_calibration",
                    side_effect=children,
                ):
                    manifest = run_warm_start_recovery(
                        self.data_path,
                        self.input_manifest_path,
                        self.immutable_baseline_dir,
                        output_root,
                        [20260727],
                        self.project_root,
                        1,
                        **self.expected_trust_roots(),
                    )

                self.assertTrue(mutated)
                self.assertNotEqual(
                    manifest["termination_reason"], "completed"
                )
                stored = json.loads(
                    (Path(manifest["output_dir"]) / "manifest.json").read_text()
                )
                self.assertNotEqual(
                    stored["termination_reason"], "completed"
                )
                if target == "data":
                    self.data_path.write_bytes(original_data)

    def test_finalizing_manifest_module_helper_or_baseline_mutation_is_closed(self):
        """Breaks if any non-data trust category is absent from final closure."""
        static_targets = {
            "input-manifest": self.input_manifest_path,
            "baseline-manifest": (
                self.immutable_baseline_dir / "manifest.json"
            ),
            "baseline-artifact": (
                self.immutable_baseline_dir / "summary.json"
            ),
            "supervisor": Path(supervisor_module.__file__),
            "replay": Path(replay_module.__file__),
            "helper": Path(run_diagnostic_module.__file__),
        }
        original_stage = supervisor_module._stage_manifest
        original_sha256 = supervisor_module._sha256

        for label, static_target in static_targets.items():
            with self.subTest(label=label):
                output_root = self.root / f"final-root-{label}"
                trust_roots = self.expected_trust_roots()
                final_target = None

                def stage(path, manifest):
                    nonlocal final_target
                    original_stage(path, manifest)
                    if manifest.get("termination_reason") == "completed":
                        final_target = static_target.resolve()

                def sha256(path):
                    observed = original_sha256(path)
                    if (
                        final_target is not None
                        and Path(path).resolve() == final_target
                    ):
                        return "0" * 64
                    return observed

                def git_output(_project_root, *arguments):
                    values = {
                        ("rev-parse", "HEAD"): "deadbeef",
                        ("branch", "--show-current"): "codex/fixture",
                        ("status", "--short"): "?? build-diagnostic/",
                    }
                    return values[arguments]

                children = [
                    {
                        "termination_reason": "completed",
                        "output_dir": "strict",
                    },
                    {
                        "termination_reason": "completed",
                        "output_dir": "restart",
                    },
                ]
                with patch.object(
                    supervisor_module, "_stage_manifest", side_effect=stage
                ), patch.object(
                    supervisor_module, "_sha256", side_effect=sha256
                ), patch.object(
                    supervisor_module, "_git_output", side_effect=git_output
                ), patch.object(
                    supervisor_module,
                    "replay_calibration",
                    side_effect=children,
                ):
                    manifest = run_warm_start_recovery(
                        self.data_path,
                        self.input_manifest_path,
                        self.immutable_baseline_dir,
                        output_root,
                        [20260727],
                        self.project_root,
                        1,
                        **trust_roots,
                    )

                self.assertIsNotNone(final_target)
                self.assertNotEqual(
                    manifest["termination_reason"], "completed"
                )
                stored = json.loads(
                    (Path(manifest["output_dir"]) / "manifest.json").read_text()
                )
                self.assertNotEqual(
                    stored["termination_reason"], "completed"
                )

    def test_finalizing_git_mutation_cannot_publish_completed(self):
        """Breaks if final publication omits full Git-state revalidation."""
        mutations = {
            "commit": ("rev-parse", "HEAD"),
            "branch": ("branch", "--show-current"),
            "status": ("status", "--short"),
        }
        for label, mutated_arguments in mutations.items():
            with self.subTest(label=label):
                original_stage = supervisor_module._stage_manifest
                finalizing = False

                def stage(path, manifest):
                    nonlocal finalizing
                    original_stage(path, manifest)
                    if manifest.get("termination_reason") == "completed":
                        finalizing = True

                def git_output(_project_root, *arguments):
                    values = {
                        ("rev-parse", "HEAD"): "deadbeef",
                        ("branch", "--show-current"): "codex/fixture",
                        ("status", "--short"): "?? build-diagnostic/",
                    }
                    if finalizing and arguments == mutated_arguments:
                        return {
                            "commit": "changed-commit",
                            "branch": "changed-branch",
                            "status": "?? unexpected-source.py",
                        }[label]
                    return values[arguments]

                children = [
                    {
                        "termination_reason": "completed",
                        "output_dir": "strict",
                    },
                    {
                        "termination_reason": "completed",
                        "output_dir": "restart",
                    },
                ]
                with patch.object(
                    supervisor_module, "_stage_manifest", side_effect=stage
                ), patch.object(
                    supervisor_module, "_git_output", side_effect=git_output
                ), patch.object(
                    supervisor_module,
                    "replay_calibration",
                    side_effect=children,
                ):
                    manifest = run_warm_start_recovery(
                        self.data_path,
                        self.input_manifest_path,
                        self.immutable_baseline_dir,
                        self.root / f"git-mutation-{label}",
                        [20260727],
                        self.project_root,
                        1,
                        **self.expected_trust_roots(),
                    )

                self.assertTrue(finalizing)
                self.assertNotEqual(
                    manifest["termination_reason"], "completed"
                )

    def test_dirty_tracked_tree_stops_before_parent_allocation(self):
        """Breaks if tracked or non-allowlisted untracked source is accepted."""
        for status in (" M source.py", "?? unexpected-source.py"):
            with self.subTest(status=status):
                def dirty_git(_project_root, *arguments):
                    values = {
                        ("rev-parse", "HEAD"): "deadbeef",
                        ("branch", "--show-current"): "codex/fixture",
                        ("status", "--short"): status,
                    }
                    return values[arguments]

                with patch(
                    "scripts.diagnostics.run_warm_start_recovery._git_output",
                    side_effect=dirty_git,
                ), patch(
                    "scripts.diagnostics.run_warm_start_recovery.replay_calibration"
                ), self.assertRaisesRegex(ValueError, "working tree"):
                    run_warm_start_recovery(
                        self.data_path,
                        self.input_manifest_path,
                        self.immutable_baseline_dir,
                        self.output_root,
                        [20260727],
                        self.project_root,
                        1,
                        **self.expected_trust_roots(),
                    )

                self.assertFalse(self.output_root.exists())

    def test_real_four_cell_parent_uses_one_8gb_gate_then_6gb_live_floor(self):
        """Breaks if supervised children reapply 8 GB or four-cell pairing is mocked."""
        project_root = self.root / "real-project"
        project_root.mkdir()
        (project_root / "source.txt").write_text("tracked\n")
        for arguments in (
            ("init", "-q"),
            ("config", "user.email", "fixture@example.com"),
            ("config", "user.name", "Fixture"),
            ("add", "source.txt"),
            ("commit", "-q", "-m", "fixture"),
        ):
            subprocess.run(
                ["git", *arguments],
                cwd=project_root,
                check=True,
                capture_output=True,
            )

        positions = [[1.0, 1.0], [2.0, 1.0], [1.0, 2.0], [2.0, 2.0]]
        config = {
            "num": 4,
            "formation": {"parts": 2, "bases-id": [[0, 1], [1, 2]]},
            "bases": [[0.0, 0.0], [4.0, 0.0], [0.0, 4.0]],
            "initial": {"position": {"positions": positions}},
            "execute": {"random-seed": 9},
            "position_covariance": {"ranging_sigma": 0.5},
            "cbfs": {
                "without-slack": {
                    "comm-fixed": {
                        "max-range": 20.0,
                        "min-neighbour-id-offset": -2,
                        "max-neighbour-id-offset": 0,
                    }
                }
            },
        }
        frames = [
            {
                "robots": [
                    {
                        "id": robot_id,
                        "state": {"x": x + 0.1 * frame, "y": y},
                    }
                    for robot_id, (x, y) in enumerate(positions, 1)
                ]
            }
            for frame in range(2)
        ]
        data_path = self.root / "real-input" / "data.json"
        data_path.parent.mkdir()
        data_path.write_text(json.dumps({"config": config, "state": frames}))
        input_manifest_path = data_path.parent / "manifest.json"
        input_manifest_path.write_text(
            json.dumps(
                {
                    "termination_reason": "completed",
                    "base_commit": "trajectory",
                    "config_sha256": "fixture-config",
                }
            )
        )
        output_root = self.root / "real-output"
        trust_roots = {
            "expected_data_sha256": _sha256(data_path),
            "expected_input_manifest_sha256": _sha256(input_manifest_path),
            "expected_baseline_manifest_sha256": _sha256(
                self.immutable_baseline_dir / "manifest.json"
            ),
            "expected_supervisor_source_sha256": _sha256(
                Path(supervisor_module.__file__)
            ),
            "expected_replay_source_sha256": _sha256(
                Path(replay_module.__file__)
            ),
            "expected_run_diagnostic_source_sha256": _sha256(
                Path(run_diagnostic_module.__file__)
            ),
        }
        parent_probes = 0

        def parent_available(_path):
            nonlocal parent_probes
            parent_probes += 1
            return 8_000_000_000 if parent_probes == 1 else 7_000_000_000

        with patch.object(
            supervisor_module,
            "available_bytes",
            side_effect=parent_available,
        ), patch.object(
            replay_module, "available_bytes", return_value=7_000_000_000
        ), patch.object(
            replay_module,
            "require_start_space",
            side_effect=AssertionError("nested 8 GB gate"),
        ):
            manifest = run_warm_start_recovery(
                data_path,
                input_manifest_path,
                self.immutable_baseline_dir,
                output_root,
                [20260727],
                project_root,
                2,
                **trust_roots,
            )

        self.assertEqual(manifest["termination_reason"], "completed")
        self.assertEqual(manifest["free_bytes_before"], 8_000_000_000)
        self.assertEqual(manifest["minimum_live_free_bytes"], 7_000_000_000)
        self.assertEqual(set(manifest["children"]), {"strict", "restart"})
        rows_by_policy = {}
        for label, policy in (
            ("strict", STRICT_PREVIOUS_POLICY),
            ("restart", RESTART_BEFORE_FIRST_FINITE_POLICY),
        ):
            child = manifest["children"][label]
            self.assertEqual(child["termination_reason"], "completed")
            with gzip.open(
                Path(child["output_dir"]) / "calibration.jsonl.gz", "rt"
            ) as source:
                rows = [json.loads(line) for line in source]
            self.assertEqual(
                {row["graph_case"] for row in rows},
                {"dynamic_dag_wnls", "fixed_refs_wnls"},
            )
            self.assertTrue(
                all(row["initialization_policy"] == policy for row in rows)
            )
            self.assertTrue(
                all(
                    "initial_estimate_source" in row
                    and "ever_acquired_finite_before_attempt" in row
                    for row in rows
                )
            )
            rows_by_policy[policy] = rows

        strict_by_key = {
            (
                row["seed"],
                row["graph_case"],
                row["frame_index"],
                row["robot_id"],
            ): row
            for row in rows_by_policy[STRICT_PREVIOUS_POLICY]
        }
        restart_by_key = {
            (
                row["seed"],
                row["graph_case"],
                row["frame_index"],
                row["robot_id"],
            ): row
            for row in rows_by_policy[RESTART_BEFORE_FIRST_FINITE_POLICY]
        }
        self.assertEqual(set(strict_by_key), set(restart_by_key))
        for key in strict_by_key:
            strict = strict_by_key[key]
            restart = restart_by_key[key]
            self.assertEqual(
                strict["active_references"], restart["active_references"]
            )
            strict_noise = [
                {
                    field: measurement[field]
                    for field in (
                        "kind",
                        "id",
                        "true_range",
                        "noise",
                        "noisy_range",
                    )
                }
                for measurement in strict["measurements"]
            ]
            restart_noise = [
                {
                    field: measurement[field]
                    for field in (
                        "kind",
                        "id",
                        "true_range",
                        "noise",
                        "noisy_range",
                    )
                }
                for measurement in restart["measurements"]
            ]
            self.assertEqual(strict_noise, restart_noise)


if __name__ == "__main__":
    unittest.main()
