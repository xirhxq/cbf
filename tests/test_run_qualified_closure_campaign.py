import os
import gzip
import hashlib
import json
import signal
import shutil
import subprocess
import sys
import tempfile
import unittest
import argparse
import copy
import dataclasses
from pathlib import Path
from unittest import mock

from scripts.diagnostics.run_qualified_closure_campaign import (
    DEVELOPMENT_ANALYSIS_ROOT,
    DEVELOPMENT_RAW_ROOT,
    STOP_FREE_BYTES,
    claim_campaign_root,
    development_schedule,
    execute_campaign,
    ProductionOperations,
    _build_condition_replay_row,
    _extract_swarm_inputs,
    _resolve_condition_references,
    _synthesize_and_publish_failed_mission,
    _prepare_replay_namespace,
    _validate_raw_replay_row,
    _validate_runtime_measurement_row,
    _SwarmStreamState,
    _runtime_runner_argv,
    _schedule_from_arguments,
    _observed_failed_mission_keys,
    _serialize_frozen_key,
    materialize_primary_config,
    orchestrate_estimator_conditions,
    project_raw_estimator_tuple,
    supervise_child_to_gzip,
    validate_new_campaign_root,
    write_schedule_no_replace,
)
from scripts.diagnostics.qualified_closure_evidence import (
    validate_estimator_tuple_schema,
)


class RunnerRootTests(unittest.TestCase):
    """Catch accidental root reuse or allocation in the source tree."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(
            prefix="qualified-runner-root-"
        )
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.project = self.base / "repo"
        self.project.mkdir()

    def test_existing_root_is_rejected_without_modification(self):
        root = self.base / "existing"
        root.mkdir()
        marker = root / "owned"
        marker.write_text("preserve")

        with self.assertRaisesRegex(FileExistsError, "must be absent"):
            claim_campaign_root(
                root,
                self.project,
                available_bytes_fn=lambda _path: 9_000_000_000,
            )

        self.assertEqual(marker.read_text(), "preserve")

    def test_symlinked_root_is_rejected(self):
        target = self.base / "target"
        target.mkdir()
        root = self.base / "root-link"
        root.symlink_to(target, target_is_directory=True)

        with self.assertRaisesRegex(ValueError, "symbolic link"):
            validate_new_campaign_root(
                root,
                self.project,
                available_bytes_fn=lambda _path: 9_000_000_000,
            )

    def test_root_inside_repository_is_rejected(self):
        root = self.project / "evidence" / "v1"

        with self.assertRaisesRegex(ValueError, "outside"):
            validate_new_campaign_root(
                root,
                self.project,
                available_bytes_fn=lambda _path: 9_000_000_000,
            )

        self.assertFalse(root.exists())

    def test_less_than_eight_gb_is_rejected_without_allocation(self):
        root = self.base / "new" / "v1"

        with self.assertRaisesRegex(RuntimeError, "8 GB"):
            claim_campaign_root(
                root,
                self.project,
                available_bytes_fn=lambda _path: 7_999_999_999,
            )

        self.assertFalse(root.exists())

    def test_identity_collection_finishes_before_atomic_claim(self):
        root = self.base / "identity-first" / "v1"
        observations = []

        claimed = claim_campaign_root(
            root,
            self.project,
            available_bytes_fn=lambda _path: 9_000_000_000,
            identity_loader=lambda: observations.append(root.exists()),
        )

        self.assertEqual(observations, [False])
        self.assertEqual(claimed, root)
        self.assertTrue(root.is_dir())


class RunnerDiskPolicyTests(unittest.TestCase):
    """Catch removal of the 6 GB hard floor and 2 GB cache ceiling."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(
            prefix="qualified-runner-disk-"
        )
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def paths(self):
        return {
            "stream_path": self.root / "rows.jsonl.gz",
            "stderr_path": self.root / "child.stderr.log",
            "manifest_path": self.root / "manifest.json",
        }

    def child(self):
        return [
            os.environ.get("PYTHON", os.sys.executable),
            "-c",
            (
                "import json,time; "
                "print(json.dumps({'key': 0}), flush=True); "
                "time.sleep(10)"
            ),
        ]

    def test_crossing_six_gb_stops_child_and_publishes_failure_manifest(self):
        free = iter((7_000_000_000, 5_999_999_999))

        result = supervise_child_to_gzip(
            self.child(),
            expected_keys=(0, 1),
            key_from_row=lambda row: row["key"],
            available_bytes_fn=lambda _path: next(free, 5_999_999_999),
            poll_interval_s=0.01,
            terminate_grace_s=0.05,
            **self.paths(),
        )

        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["reason"], "disk_hard_floor")
        self.assertTrue((self.root / "manifest.json").is_file())

    def test_reusable_cache_cap_is_independent_of_raw_evidence_size(self):
        cache = self.root / "cache"
        cache.mkdir()
        allocations = {cache: 2_000_000_001}

        result = supervise_child_to_gzip(
            self.child(),
            cache_root=cache,
            expected_keys=(0,),
            key_from_row=lambda row: row["key"],
            available_bytes_fn=lambda _path: 7_000_000_000,
            allocated_bytes_fn=lambda path: allocations.get(Path(path), 0),
            poll_interval_s=0.01,
            terminate_grace_s=0.05,
            **self.paths(),
        )

        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["reason"], "cache_cap")
        self.assertTrue((self.root / "manifest.json").is_file())

    def test_production_measurement_generation_stops_with_paired_prefix(self):
        from tests.test_generate_qualified_measurements import (
            read_rows,
            write_truth,
        )

        truth = self.root / "truth.jsonl.gz"
        write_truth(truth)
        stage = self.root / "mission-stage"
        stage.mkdir()
        free = iter((STOP_FREE_BYTES + 1, STOP_FREE_BYTES - 1))
        operations = ProductionOperations()
        operations.available_bytes_fn = lambda _path: next(
            free, STOP_FREE_BYTES - 1
        )
        result = operations.generate_measurements(
            {
                "range_noise_seed": 2026081101,
            },
            stage,
            {
                "truth_path": truth,
                "truth_manifest": {
                    "terminal": True,
                    "status": "completed",
                    "sha256": hashlib.sha256(
                        truth.read_bytes()
                    ).hexdigest(),
                },
                "config_sha256": "a" * 64,
                "edge_schedule": {
                    "dynamic_primary": [
                        (0, 1, ("base", 0)),
                        (0, 2, ("uav", 1)),
                    ],
                    "fixed_fim_ablation": [
                        (0, 1, ("base", 0)),
                    ],
                },
            },
        )

        bundle = stage / "measurements"
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["reason"], "disk_hard_floor")
        self.assertTrue(result["terminal"])
        self.assertEqual(len(read_rows(bundle / "runtime.jsonl.gz")), 1)
        self.assertEqual(len(read_rows(bundle / "audit.jsonl.gz")), 1)
        self.assertEqual(
            json.loads((bundle / "manifest.json").read_text())["status"],
            "failed",
        )
        self.assertFalse(any(stage.glob(".measurements.*.tmp")))


class RunnerScheduleTests(unittest.TestCase):
    """Catch implicit/colliding seeds and accidental ablation launches."""

    def test_development_roots_and_all_ten_seed_pairs_are_exact(self):
        schedule = development_schedule()

        self.assertEqual(
            DEVELOPMENT_RAW_ROOT,
            Path("/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v4"),
        )
        self.assertEqual(
            DEVELOPMENT_ANALYSIS_ROOT,
            Path("/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v4"),
        )
        self.assertEqual(len(schedule), 10)
        self.assertEqual(
            [mission["trajectory_seed"] for mission in schedule],
            list(range(2026080101, 2026080111)),
        )
        self.assertEqual(
            [mission["range_noise_seed"] for mission in schedule],
            list(range(2026081101, 2026081111)),
        )
        self.assertTrue(all(
            mission["campaign_id"] == "development-v4"
            and mission["frames"] == 1000
            and mission["horizon_s"] == 500.0
            and mission["conditions"]
                == ["dynamic_primary", "fixed_fim_ablation"]
            for mission in schedule
        ))

    def test_development_v4_schedule_is_required_while_confirmatory_remains_v1(self):
        development = argparse.Namespace(
            kind="development", version="v4", smoke_id=None,
            trajectory_seeds="2026080101:2026080110",
            range_noise_seeds="2026081101:2026081110", frames=1000,
        )
        self.assertTrue(all(
            mission["campaign_id"] == "development-v4"
            for mission in _schedule_from_arguments(development)
        ))
        for version in ("v1", "v2", "v3"):
            with self.subTest(version=version):
                development.version = version
                with self.assertRaisesRegex(ValueError, "development.*v4"):
                    _schedule_from_arguments(development)

        confirmatory = argparse.Namespace(
            kind="confirmatory", version="v1", smoke_id=None,
            trajectory_seeds="2026082001:2026082060",
            range_noise_seeds="2026083001:2026083060", frames=1000,
        )
        self.assertTrue(all(
            mission["campaign_id"] == "confirmatory-v1"
            for mission in _schedule_from_arguments(confirmatory)
        ))
        confirmatory.version = "v2"
        with self.assertRaisesRegex(ValueError, "confirmatory.*v1"):
            _schedule_from_arguments(confirmatory)

    def test_full_schedule_is_no_replace_and_serialized_before_launch(self):
        with tempfile.TemporaryDirectory(
            prefix="qualified-schedule-"
        ) as directory:
            path = Path(directory) / "schedule.json"
            schedule = development_schedule()

            write_schedule_no_replace(path, schedule)

            self.assertEqual(json.loads(path.read_text()), {
                "schema_version": "cbf2026-qualified-campaign-schedule-v1",
                "mission_count": 10,
                "missions": schedule,
            })
            with self.assertRaises(FileExistsError):
                write_schedule_no_replace(path, schedule)

    def test_confirmatory_smoke_registration_uses_singleton_schedule_and_exact_root(self):
        with tempfile.TemporaryDirectory(prefix="qualified-smoke-registration-") as directory:
            root = Path(directory)
            protocol_path = root / "protocol.json"
            authorization_path = root / "authorization.json"
            smoke_raw = root / "smoke-a-raw"
            protocol_path.write_text(json.dumps({
                "kind": "confirmatory", "version": "v1", "no_retry": True,
                "roots": {"smoke_a_raw": str(smoke_raw.resolve())},
                "smoke_schedule": {
                    "trajectory_seed": 2026089001,
                    "range_noise_seed": 2026089101,
                    "frames": 20,
                },
            }))
            authorization_path.write_text("{}")
            arguments = argparse.Namespace(
                kind="confirmatory-smoke", version=None, smoke_id="a",
                protocol=protocol_path, authorization=authorization_path,
                binary=root / "Swarm", base_config=root / "base.json",
                primary_config=root / "primary.json",
                ablation_config=root / "ablation.json",
                output_root=smoke_raw,
                trajectory_seeds="2026089001:2026089001",
                range_noise_seeds="2026089101:2026089101", frames=20,
            )
            protocol = json.loads(protocol_path.read_text())
            protocol["smoke_argv"] = {
                "a": {"runner": _runtime_runner_argv(arguments)}
            }
            protocol_path.write_text(json.dumps(protocol))
            with mock.patch(
                "scripts.diagnostics.register_qualified_closure_campaign.validate_authorization_binding",
                return_value={"implementation_identity": "fixture"},
            ):
                registration = ProductionOperations().validate_registration(arguments)
            self.assertEqual(registration["implementation_identity"], "fixture")

            arguments.output_root = root / "unregistered-root"
            with mock.patch(
                "scripts.diagnostics.register_qualified_closure_campaign.validate_authorization_binding",
                return_value={"implementation_identity": "fixture"},
            ):
                with self.assertRaisesRegex(ValueError, "output root"):
                    ProductionOperations().validate_registration(arguments)

    def test_runtime_runner_argv_and_binding_paths_are_exact(self):
        with tempfile.TemporaryDirectory(prefix="qualified-runtime-binding-") as directory:
            root = Path(directory)
            paths = {}
            for name in ("Swarm", "base.json", "primary.json", "ablation.json"):
                path = root / name
                path.write_text("same bytes\n")
                paths[name] = path
            arguments = argparse.Namespace(
                kind="development", version="v4", smoke_id=None,
                protocol=root / "fixture-protocol.json",
                authorization=root / "fixture-authorization.json",
                binary=paths["Swarm"], base_config=paths["base.json"],
                primary_config=paths["primary.json"],
                ablation_config=paths["ablation.json"],
                trajectory_seeds="2026080101:2026080110",
                range_noise_seeds="2026081101:2026081110", frames=1000,
                output_root=root / "raw",
            )
            protocol = {
                "runner_argv": _runtime_runner_argv(arguments),
                "bindings": {},
            }
            for label, path in (
                ("binary", arguments.binary),
                ("base_config", arguments.base_config),
                ("primary_config", arguments.primary_config),
                ("ablation_config", arguments.ablation_config),
            ):
                protocol["bindings"][label] = {
                    "path": str(path.resolve()),
                    "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "bytes": path.stat().st_size,
                }
            operations = ProductionOperations()
            operations.protocol = protocol
            operations.collect_identities(arguments)

            substitute = root / "same-content-alternate.json"
            substitute.write_text("same bytes\n")
            arguments.ablation_config = substitute
            with self.assertRaisesRegex(ValueError, "identity differs"):
                operations.collect_identities(arguments)
            with self.assertRaisesRegex(ValueError, "runner argv differs"):
                from scripts.diagnostics.run_qualified_closure_campaign import (
                    _validate_runtime_runner_argv,
                )
                _validate_runtime_runner_argv(protocol, arguments)

    def test_materialized_primary_deep_merges_and_sets_only_mission_identity(self):
        with tempfile.TemporaryDirectory(
            prefix="qualified-materialized-"
        ) as directory:
            root = Path(directory)
            project = Path(__file__).resolve().parents[1]
            base = project / "config" / "config.json"
            overlay = (
                project
                / "config"
                / "diagnostics"
                / "qualified_mode_hybrid_dcbf_development_v1.json"
            )
            output = root / "mission" / "config.materialized.json"
            mission = {
                "campaign_id": "development-v4",
                "mission_id": "mission-01",
                "trajectory_seed": 2026080101,
                "range_noise_seed": 2026081101,
                "frames": 1000,
                "horizon_s": 500.0,
                "conditions": ["dynamic_primary", "fixed_fim_ablation"],
            }

            materialized = materialize_primary_config(
                base, overlay, output, mission
            )

            self.assertEqual(materialized["world"]["spacing"], 10.0)
            self.assertEqual(
                materialized["initial"]["position"]["method"],
                "random-in-polygon",
            )
            self.assertEqual(materialized["execute"]["time-step"], 0.5)
            self.assertEqual(materialized["execute"]["time-total"], 500.0)
            self.assertEqual(
                materialized["execute"]["execution-mode"], "distributed"
            )
            self.assertEqual(
                materialized["execute"]["random-seed"], 2026080101
            )
            self.assertEqual(
                materialized["position_covariance"]["reference-selection"],
                "dynamic-lower-index",
            )
            self.assertEqual(materialized["output_path"], str(output.parent))
            self.assertEqual(materialized["run_suffix"], "_mission-01")
            self.assertEqual(materialized["evidence-stream"], {
                "enabled": True,
                "schema-version": "cbf2026-qualified-evidence-v1",
                "campaign-id": "development-v4",
                "trajectory-seed": 2026080101,
                "range-noise-seed": 2026081101,
                "condition": "dynamic_primary",
            })
            self.assertEqual(json.loads(output.read_text()), materialized)


class RunnerSupervisorTests(unittest.TestCase):
    """Catch child-lifecycle paths that could leave a nonterminal bundle."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(
            prefix="qualified-supervisor-"
        )
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def run_child(self, source, **overrides):
        argv = overrides.pop("argv", [os.sys.executable, "-c", source])
        options = {
            "stream_path": self.root / "rows.jsonl.gz",
            "synthetic_path": self.root / "missing.jsonl.gz",
            "stderr_path": self.root / "child.stderr.log",
            "manifest_path": self.root / "manifest.json",
            "expected_keys": (0, 1, 2),
            "key_from_row": lambda row: row["key"],
            "available_bytes_fn": lambda _path: 7_000_000_000,
            "poll_interval_s": 0.005,
            "terminate_grace_s": 0.02,
            "wallclock_timeout_s": 2.0,
            "line_stall_timeout_s": 1.0,
        }
        options.update(overrides)
        return supervise_child_to_gzip(
            argv,
            **options,
        )

    def rows(self, name):
        with gzip.open(self.root / name, "rt", encoding="utf-8") as source:
            return [json.loads(line) for line in source]

    def assert_failure_bundle(self, result, reason, prefix_keys):
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["reason"], reason)
        self.assertTrue(result["terminal"])
        self.assertEqual(
            [row["key"] for row in self.rows("rows.jsonl.gz")],
            prefix_keys,
        )
        synthetic = self.rows("missing.jsonl.gz")
        missing = [row["key"] for row in synthetic if row["record_type"] == "missing"]
        self.assertEqual(missing, [key for key in (0, 1, 2) if key not in prefix_keys])
        self.assertEqual(synthetic[-1]["record_type"], "mission")
        self.assertFalse(synthetic[-1]["success"])
        self.assertEqual(json.loads((self.root / "manifest.json").read_text()), result)

    def test_normal_child_streams_exact_complete_json_objects(self):
        result = self.run_child(
            "import json; [print(json.dumps({'key': i}), flush=True) for i in range(3)]"
        )

        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["valid_rows"], 3)
        self.assertEqual([row["key"] for row in self.rows("rows.jsonl.gz")], [0, 1, 2])
        self.assertEqual(self.rows("missing.jsonl.gz"), [])

    def test_malformed_third_row_retains_two_row_prefix(self):
        result = self.run_child(
            "import json; print(json.dumps({'key': 0}), flush=True); "
            "print(json.dumps({'key': 1}), flush=True); print('[]', flush=True)"
        )

        self.assert_failure_bundle(result, "malformed_json", [0, 1])

    def test_nonzero_exit_after_two_rows_is_terminal(self):
        result = self.run_child(
            "import json,sys; print(json.dumps({'key': 0}), flush=True); "
            "print(json.dumps({'key': 1}), flush=True); sys.exit(7)"
        )

        self.assert_failure_bundle(result, "child_nonzero_exit", [0, 1])

    def test_signal_after_two_rows_is_terminal(self):
        result = self.run_child(
            "import json,os,signal; print(json.dumps({'key': 0}), flush=True); "
            "print(json.dumps({'key': 1}), flush=True); "
            "os.kill(os.getpid(), signal.SIGTERM)"
        )

        self.assert_failure_bundle(result, "child_signal", [0, 1])

    def test_silent_child_hits_wallclock_deadline(self):
        result = self.run_child(
            "import time; time.sleep(1)",
            wallclock_timeout_s=0.05,
            line_stall_timeout_s=1.0,
        )

        self.assert_failure_bundle(result, "wallclock_timeout", [])

    def test_partial_line_hits_complete_line_stall_deadline(self):
        result = self.run_child(
            "import sys,time; sys.stdout.write('{\"key\": 0'); "
            "sys.stdout.flush(); time.sleep(1)",
            wallclock_timeout_s=1.0,
            line_stall_timeout_s=0.05,
        )

        self.assert_failure_bundle(result, "line_stall_timeout", [])

    def test_launch_failure_still_publishes_complete_terminal_bundle(self):
        result = self.run_child("", argv=[str(self.root / "missing-child")])

        self.assert_failure_bundle(result, "child_launch_error", [])

    def test_duplicate_and_unexpected_keys_have_distinct_terminal_reasons(self):
        duplicate = self.run_child(
            "import json; print(json.dumps({'key': 0}), flush=True); "
            "print(json.dumps({'key': 0}), flush=True)"
        )
        self.assert_failure_bundle(duplicate, "duplicate_key", [0])

        self.temporary.cleanup()
        self.temporary = tempfile.TemporaryDirectory(
            prefix="qualified-supervisor-unexpected-"
        )
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        unexpected = self.run_child(
            "import json; print(json.dumps({'key': 0}), flush=True); "
            "print(json.dumps({'key': 9}), flush=True)"
        )
        self.assert_failure_bundle(unexpected, "unexpected_key", [0])

    def test_exact_row_validator_rejects_schema_before_prefix_write(self):
        result = self.run_child(
            "import json; print(json.dumps({'key': 0}), flush=True)",
            row_validator=lambda row: row.get("schema") == "exact-v1",
        )
        self.assert_failure_bundle(result, "schema_error", [])

    def test_oversized_partial_line_is_bounded_and_terminal(self):
        result = self.run_child(
            "import sys,time; sys.stdout.write('{' + 'x' * 1000); "
            "sys.stdout.flush(); time.sleep(1)",
            max_line_bytes=128,
        )
        self.assert_failure_bundle(result, "line_too_large", [])


class RunnerProducerOrchestrationTests(unittest.TestCase):
    """Catch measurement substitution, truth leakage, or shared replay state."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(
            prefix="qualified-producers-"
        )
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.measurements = self.root / "runtime-measurements.jsonl.gz"
        self.measurements.write_bytes(b"immutable-runtime-measurements")
        import hashlib
        self.measurement_sha = hashlib.sha256(
            self.measurements.read_bytes()
        ).hexdigest()
        self.commands = self.root / "held-commands.jsonl.gz"
        self.commands.write_bytes(b"immutable-held-commands")
        self.configs = {
            "dynamic_primary": self.root / "primary.json",
            "fixed_fim_ablation": self.root / "ablation.json",
        }
        for path in self.configs.values():
            path.write_text("{}")
        self.outputs = {
            condition: self.root / f"{condition}.jsonl.gz"
            for condition in self.configs
        }

    def manifest(self, sha=None):
        return {
            "terminal": True,
            "status": "completed",
            "sha256": sha or self.measurement_sha,
            "measurement_stream_id": "stream-v1",
        }

    def replay_row(self, frame, robot):
        from tests.test_replay_qualified_estimator import (
            build_registered_qualified_row,
            registered_deployment_domain,
        )

        arguments = {
            "frame_index": frame,
            "applied_command_frame": frame - 1,
            "history_version": frame,
        }
        if frame == 0:
            arguments.update({
                "applied_command_frame": None,
                "qualifier_kind": "deployment",
                "qualifier_payload": {
                    "domain": registered_deployment_domain()
                },
            })
        row = build_registered_qualified_row(**arguments)
        local = robot if robot <= 7 else robot - 7
        row["robot_id"] = robot
        row["squad_local_index"] = local
        row["schedule_id"] = (
            f"frame-{frame}:robot-{robot}:squad-local-{local}"
        )
        return row

    def write_replay_rows(self, stage, condition, rows):
        path = stage / f"replay-{condition}.raw.jsonl.gz"
        with path.open("xb") as raw:
            with gzip.GzipFile(
                filename="", mode="wb", fileobj=raw, mtime=0
            ) as sink:
                for row in rows:
                    sink.write(
                        json.dumps(row, separators=(",", ":")).encode()
                        + b"\n"
                    )
        return path

    def assert_exact_failed_complement(self, stage, mission, reason):
        from scripts.diagnostics.analyze_qualified_closure_campaign import (
            _validate_synthesized_missing_stream,
        )
        from scripts.diagnostics.qualified_closure_evidence import (
            FrozenMissionSchedule,
            synthesize_missing_mission,
        )

        observed = _observed_failed_mission_keys(stage, mission)
        frozen = FrozenMissionSchedule(
            mission["campaign_id"], mission["trajectory_seed"],
            mission["range_noise_seed"], mission["frames"],
            tuple(range(1, 15)), tuple(mission["conditions"]),
            "dynamic_primary",
        )
        universe = synthesize_missing_mission(frozen, reason)
        missing = {
            kind: []
            for kind in (
                "initialization", "estimator", "controller", "endpoint",
                "reconstructed", "reset",
            )
        }
        with gzip.open(
            stage / "synthetic-missing.jsonl.gz", "rt", encoding="utf-8"
        ) as source:
            for row in map(json.loads, source):
                if row["record_type"] == "missing_mission":
                    continue
                kind = row["record_type"].removeprefix("missing_")
                missing[kind].append(json.dumps(
                    row["key"], sort_keys=True, separators=(",", ":")
                ))

        for kind in missing:
            observed_keys = {
                json.dumps(
                    _serialize_frozen_key(key), sort_keys=True,
                    separators=(",", ":"),
                )
                for key in observed[kind]
            }
            missing_keys = set(missing[kind])
            frozen_keys = {
                json.dumps(
                    _serialize_frozen_key(key), sort_keys=True,
                    separators=(",", ":"),
                )
                for key in getattr(universe, kind)
            }
            self.assertEqual(len(missing[kind]), len(missing_keys))
            self.assertFalse(observed_keys & missing_keys)
            self.assertEqual(observed_keys | missing_keys, frozen_keys)

        _validate_synthesized_missing_stream(
            stage / "synthetic-missing.jsonl.gz",
            mission,
            reason,
            mission_root=stage,
        )

    def assert_replay_prefix_rejected(self, label, frames, keys):
        from scripts.diagnostics.analyze_qualified_closure_campaign import (
            _analyzer_observed_failed_mission_keys,
        )

        validators = (
            _observed_failed_mission_keys,
            _analyzer_observed_failed_mission_keys,
        )
        for validator in validators:
            with self.subTest(validator=validator.__name__):
                stage = self.root / f"{label}-{validator.__name__}"
                stage.mkdir()
                mission = {
                    "campaign_id": "development-v4",
                    "mission_id": "mission-01",
                    "trajectory_seed": 2026080101,
                    "range_noise_seed": 2026081101,
                    "frames": frames,
                    "conditions": ["dynamic_primary"],
                }
                self.write_replay_rows(
                    stage,
                    "dynamic_primary",
                    [self.replay_row(frame, robot) for frame, robot in keys],
                )
                with self.assertRaisesRegex(
                    ValueError, "frozen prefix order"
                ):
                    validator(stage, mission)

    def test_retained_replay_rejects_late_first_row(self):
        self.assert_replay_prefix_rejected(
            "late-first-row", 181, [(180, 12)]
        )

    def test_retained_replay_rejects_middle_gap(self):
        self.assert_replay_prefix_rejected(
            "middle-gap", 2, [(0, 1), (0, 3)]
        )

    def test_retained_replay_rejects_cross_frame_gap(self):
        self.assert_replay_prefix_rejected(
            "cross-frame-gap", 3, [(0, 1), (1, 1)]
        )

    def test_retained_replay_accepts_exact_partial_prefix(self):
        from scripts.diagnostics.analyze_qualified_closure_campaign import (
            _analyzer_observed_failed_mission_keys,
        )

        valid_stage = self.root / "valid-partial-prefix"
        valid_stage.mkdir()
        valid_mission = {
            "campaign_id": "development-v4",
            "mission_id": "mission-01",
            "trajectory_seed": 2026080101,
            "range_noise_seed": 2026081101,
            "frames": 2,
            "conditions": ["dynamic_primary"],
        }
        valid_keys = [(0, 1), (0, 2), (0, 3)]
        self.write_replay_rows(
            valid_stage,
            "dynamic_primary",
            [self.replay_row(frame, robot) for frame, robot in valid_keys],
        )
        for validator in (
            _observed_failed_mission_keys,
            _analyzer_observed_failed_mission_keys,
        ):
            observed = validator(valid_stage, valid_mission)
            self.assertEqual(observed["estimator"], set())
            self.assertEqual(observed["initialization"], set())

    def test_declared_unsuccessful_swarm_has_exact_complement(self):
        stage = self.root / "declared-unsuccessful"
        stage.mkdir()
        mission = {
            "campaign_id": "development-v4",
            "mission_id": "mission-01",
            "trajectory_seed": 2026080101,
            "range_noise_seed": 2026081101,
            "frames": 1,
            "conditions": ["dynamic_primary", "fixed_fim_ablation"],
        }
        rows = [
            {
                "record_type": "initialization",
                "schema_version": "cbf2026-qualified-evidence-v1",
                "campaign_id": mission["campaign_id"],
                "condition": "dynamic_primary",
                "trajectory_seed": mission["trajectory_seed"],
                "range_noise_seed": mission["range_noise_seed"],
                "frame_index": 0,
                "robot_id": robot,
                "runtime": {"local_index": (robot - 1) % 7 + 1},
                "analyzer_only": {"truth_position": [float(robot), 0.0]},
            }
            for robot in range(1, 15)
        ]
        rows.append({
            "record_type": "mission_terminal",
            "schema_version": "cbf2026-qualified-evidence-v1",
            "campaign_id": mission["campaign_id"],
            "condition": "dynamic_primary",
            "trajectory_seed": mission["trajectory_seed"],
            "range_noise_seed": mission["range_noise_seed"],
            "frame_index": 1,
            "runtime": {
                "success": False,
                "reason": "bootstrap_failure:singular_fim",
                "process_outcome": "bootstrap_failure",
                "declared_frames": 1,
                "completed_intervals": 0,
            },
        })
        with gzip.open(
            stage / "swarm.jsonl.gz", "wt", encoding="utf-8"
        ) as sink:
            for row in rows:
                sink.write(json.dumps(row, separators=(",", ":")) + "\n")

        ProductionOperations().synthesize_failed_mission(
            mission, stage, "mission_declared_unsuccessful"
        )

        self.assert_exact_failed_complement(
            stage, mission, "mission_declared_unsuccessful"
        )

    def test_partial_replay_has_exact_complement(self):
        stage = self.root / "partial-replay-complement"
        stage.mkdir()
        mission = {
            "campaign_id": "development-v4",
            "mission_id": "mission-01",
            "trajectory_seed": 2026080101,
            "range_noise_seed": 2026081101,
            "frames": 2,
            "conditions": ["dynamic_primary", "fixed_fim_ablation"],
        }
        self.write_replay_rows(
            stage,
            "dynamic_primary",
            [
                self.replay_row(0, robot) for robot in range(1, 15)
            ] + [
                self.replay_row(1, robot) for robot in range(1, 4)
            ],
        )

        ProductionOperations().synthesize_failed_mission(
            mission, stage, "replay_line_stall_timeout"
        )

        self.assert_exact_failed_complement(
            stage, mission, "replay_line_stall_timeout"
        )

    def test_initialization_key_requires_cpp_and_both_condition_audits(self):
        from scripts.diagnostics.analyze_qualified_closure_campaign import (
            _analyzer_observed_failed_mission_keys,
        )

        stage = self.root / "composite-initialization-key"
        stage.mkdir()
        mission = {
            "campaign_id": "development-v4",
            "mission_id": "mission-01",
            "trajectory_seed": 2026080101,
            "range_noise_seed": 2026081101,
            "frames": 1,
            "conditions": ["dynamic_primary", "fixed_fim_ablation"],
        }
        initialization = {
            "record_type": "initialization",
            "schema_version": "cbf2026-qualified-evidence-v1",
            "campaign_id": mission["campaign_id"],
            "condition": "dynamic_primary",
            "trajectory_seed": mission["trajectory_seed"],
            "range_noise_seed": mission["range_noise_seed"],
            "frame_index": 0,
            "robot_id": 1,
            "runtime": {"local_index": 1},
            "analyzer_only": {"truth_position": [1.0, 0.0]},
        }
        with gzip.open(
            stage / "swarm.jsonl.gz", "wt", encoding="utf-8"
        ) as sink:
            sink.write(json.dumps(initialization) + "\n")
        self.write_replay_rows(
            stage, "dynamic_primary", [self.replay_row(0, 1)]
        )
        validators = (
            _observed_failed_mission_keys,
            _analyzer_observed_failed_mission_keys,
        )
        for validator in validators:
            with self.subTest(
                phase="one-condition", validator=validator.__name__
            ):
                observed = validator(stage, mission)
                self.assertEqual(observed["initialization"], set())
                self.assertEqual(observed["estimator"], set())

        self.write_replay_rows(
            stage, "fixed_fim_ablation", [self.replay_row(0, 1)]
        )
        expected = {
            ("development-v4", 2026080101, 2026081101, 0, 1)
        }
        for validator in validators:
            with self.subTest(
                phase="both-conditions", validator=validator.__name__
            ):
                observed = validator(stage, mission)
                self.assertEqual(observed["initialization"], expected)
                self.assertEqual(observed["estimator"], set())

    def test_both_conditions_receive_same_measurement_identity_and_no_audit_path(self):
        calls = []

        def producer(**kwargs):
            calls.append(kwargs)
            return {"history": [kwargs["condition"]]}

        results = orchestrate_estimator_conditions(
            measurement_path=self.measurements,
            measurement_manifest=self.manifest(),
            command_history_path=self.commands,
            config_paths=self.configs,
            output_paths=self.outputs,
            producer=producer,
        )

        self.assertEqual(set(results), set(self.configs))
        self.assertEqual(
            {call["measurement_sha256"] for call in calls},
            {self.measurement_sha},
        )
        serialized_calls = json.dumps(calls, default=str).lower()
        self.assertNotIn("truth", serialized_calls)
        self.assertNotIn("noiseless", serialized_calls)
        self.assertNotIn("sampled_noise", serialized_calls)
        self.assertNotIn("audit", serialized_calls)

    def test_conditions_never_share_public_or_private_lifecycle_identity(self):
        state_ids = []

        def producer(**kwargs):
            state = kwargs["lifecycle_state"]
            state_ids.append((id(state), id(state["public"]), id(state["private"])))
            state["public"]["condition"] = kwargs["condition"]
            state["private"]["history"] = [kwargs["condition"]]
            return state

        results = orchestrate_estimator_conditions(
            measurement_path=self.measurements,
            measurement_manifest=self.manifest(),
            command_history_path=self.commands,
            config_paths=self.configs,
            output_paths=self.outputs,
            producer=producer,
        )

        self.assertEqual(len({item[0] for item in state_ids}), 2)
        self.assertEqual(len({item[1] for item in state_ids}), 2)
        self.assertEqual(len({item[2] for item in state_ids}), 2)
        results["dynamic_primary"]["public"]["mutated"] = True
        self.assertNotIn("mutated", results["fixed_fim_ablation"]["public"])

    def test_measurement_substitution_fails_before_any_producer_runs(self):
        calls = []
        with self.assertRaisesRegex(ValueError, "measurement SHA-256"):
            orchestrate_estimator_conditions(
                measurement_path=self.measurements,
                measurement_manifest=self.manifest("0" * 64),
                command_history_path=self.commands,
                config_paths=self.configs,
                output_paths=self.outputs,
                producer=lambda **kwargs: calls.append(kwargs),
            )
        self.assertEqual(calls, [])

    def test_swarm_inputs_freeze_distinct_dynamic_and_fixed_reference_policies(self):
        evidence = self.root / "swarm.jsonl.gz"
        row = {
            "record_type": "controller_interval", "frame_index": 0,
            "runtime": {"nodes": [
                {"robot_id": robot, "applied_command": [0.0, 0.0, 0.0]}
                for robot in range(1, 15)
            ]},
        }
        with evidence.open("xb") as raw:
            with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as sink:
                sink.write(json.dumps(row).encode() + b"\n")

        bundle = _extract_swarm_inputs(evidence, self.root / "swarm-inputs")

        self.assertEqual(bundle["edge_schedule"], {
            "dynamic_primary": "controller_references",
            "fixed_fim_ablation": "fixed_paper_localization_references",
        })

    def test_uav_references_resolve_only_from_each_condition_publication(self):
        raw = [{
            "key": ("uav", 1),
            "position": [999.0, 999.0],
            "range": 4.0,
            "covariance": [[99.0, 0.0], [0.0, 99.0]],
            "ranging_sigma": 0.5,
            "base_anchor_provenance": [99],
        }]

        def condition_state(position, covariance, provenance):
            return {1: {"public": {
                "output_status": "fresh",
                "estimate": list(position),
                "modeled_covariance": covariance,
                "epsilon": 3.0,
                "prediction_age": 0,
                "aged_modeled_radius": None,
                "base_anchor_provenance": list(provenance),
                "mode_id": "mode-1",
                "reason": "unique_admissible_mode",
            }, "private": None, "history": 0}}

        dynamic = condition_state([1.0, 2.0], [[1.0, 0.0], [0.0, 1.0]], [0, 1])
        fixed = condition_state([8.0, 9.0], [[2.0, 0.0], [0.0, 2.0]], [1, 2])
        fixed_before = _resolve_condition_references(raw, fixed, owner_id=2)

        dynamic[1]["public"]["estimate"] = [30.0, 40.0]
        dynamic_result = _resolve_condition_references(raw, dynamic, owner_id=2)
        fixed_after = _resolve_condition_references(raw, fixed, owner_id=2)

        self.assertEqual(fixed_after, fixed_before)
        self.assertEqual(fixed_after[0]["position"], [8.0, 9.0])
        self.assertEqual(dynamic_result[0]["position"], [30.0, 40.0])
        self.assertNotEqual(dynamic_result, fixed_after)

        dynamic[1]["public"]["base_anchor_provenance"] = [0]
        self.assertEqual(
            _resolve_condition_references(raw, dynamic, owner_id=2), []
        )

    def test_hidden_deployment_then_history_rows_validate_independently(self):
        from scripts.diagnostics.analyze_qualified_estimator import (
            validate_and_recompute_qualified_row,
        )
        from tests.test_replay_qualified_estimator import (
            registered_deployment_domain,
            registered_two_circle_inputs,
        )
        _fixture, references, solver = registered_two_circle_inputs()
        references = [
            dict(
                reference,
                key=("base", index),
                base_anchor_provenance=[index],
            )
            for index, reference in enumerate(references)
        ]
        state = {12: {"public": None, "private": None, "history": 0}}

        deployment = _build_condition_replay_row(
            frame=0, robot_id=12, raw_references=references,
            condition_state=state, held_velocity=[0.0, 0.0],
            deployment_domain=registered_deployment_domain(),
            innovation_limit=11.829007011943707, solver=solver,
            mission_horizon_frames=3,
        )
        history = _build_condition_replay_row(
            frame=1, robot_id=12, raw_references=references,
            condition_state=state, held_velocity=[0.0, 0.0],
            deployment_domain=registered_deployment_domain(),
            innovation_limit=11.829007011943707, solver=solver,
            mission_horizon_frames=3,
        )

        self.assertEqual(deployment["qualification_kind"], "deployment")
        self.assertEqual(history["qualification_kind"], "history")
        self.assertEqual(
            history["audit_bundle"]["transition_inputs"]["previous_private"],
            deployment["audit_bundle"]["lifecycle"]["next_private_state"],
        )
        self.assertTrue(validate_and_recompute_qualified_row(deployment)["valid"])
        self.assertTrue(validate_and_recompute_qualified_row(history)["valid"])

    def test_production_failed_mission_serializes_canonical_edge_keys(self):
        stage = self.root / "failed-stage"
        stage.mkdir()
        mission = {
            "campaign_id": "development-v4",
            "trajectory_seed": 2026080101,
            "range_noise_seed": 2026081101,
            "frames": 1,
            "conditions": ["dynamic_primary", "fixed_fim_ablation"],
        }

        ProductionOperations().synthesize_failed_mission(
            mission, stage, "injected_failure"
        )

        with gzip.open(
            stage / "synthetic-missing.jsonl.gz", "rt", encoding="utf-8"
        ) as source:
            rows = [json.loads(line) for line in source]
        endpoint = next(
            row for row in rows if row["record_type"] == "missing_endpoint"
        )
        reconstructed = next(
            row for row in rows
            if row["record_type"] == "missing_reconstructed"
        )
        self.assertEqual(endpoint["key"][5], {
            "kind": "localization", "low": 1, "high": 1, "base_id": 0,
        })
        self.assertEqual(reconstructed["key"][5], endpoint["key"][5])
        self.assertEqual(rows[-1], {
            "record_type": "missing_mission", "reason": "injected_failure",
        })

    def test_failed_publication_preserves_prefix_and_adds_exact_missing_universe(self):
        from scripts.diagnostics.analyze_qualified_closure_campaign import (
            _validate_synthesized_missing_stream,
        )
        root = self.root / "published-failure"
        root.mkdir()
        stage = root / ".mission-01.stage"
        stage.mkdir()
        initialization = {
            "record_type": "initialization",
            "schema_version": "cbf2026-qualified-evidence-v1",
            "campaign_id": "development-v4",
            "condition": "dynamic_primary",
            "trajectory_seed": 2026080101,
            "range_noise_seed": 2026081101,
            "frame_index": 0,
            "robot_id": 1,
            "runtime": {"local_index": 1},
            "analyzer_only": {"truth_position": [-1450.0, -300.0]},
        }
        with gzip.open(stage / "swarm.jsonl.gz", "wt", encoding="utf-8") as sink:
            sink.write(json.dumps(initialization) + "\n")
        for condition in (
            "dynamic_primary", "fixed_fim_ablation"
        ):
            self.write_replay_rows(
                stage, condition, [self.replay_row(0, 1)]
            )
        (stage / "swarm.stderr.log").write_text("producer diagnostic\n")
        (stage / "swarm.supervisor.manifest.json").write_text(json.dumps({
            "terminal": True, "status": "failed", "reason": "malformed_json",
        }))
        mission = {
            "campaign_id": "development-v4", "mission_id": "mission-01",
            "trajectory_seed": 2026080101, "range_noise_seed": 2026081101,
            "frames": 1,
            "conditions": ["dynamic_primary", "fixed_fim_ablation"],
        }

        _synthesize_and_publish_failed_mission(
            ProductionOperations(), mission, stage, root, "malformed_json"
        )

        published = root / "mission-01"
        with gzip.open(
            published / "swarm.jsonl.gz", "rt", encoding="utf-8"
        ) as source:
            self.assertEqual(json.loads(source.readline())["record_type"],
                             "initialization")
        _validate_synthesized_missing_stream(
            published / "synthetic-missing.jsonl.gz",
            mission,
            "malformed_json",
            mission_root=published,
        )
        with gzip.open(
            published / "synthetic-missing.jsonl.gz", "rt", encoding="utf-8"
        ) as source:
            missing_initialization = {
                tuple(row["key"])
                for row in map(json.loads, source)
                if row["record_type"] == "missing_initialization"
            }
        observed_initialization = {
            ("development-v4", 2026080101, 2026081101, 0, 1)
        }
        frozen_initialization = {
            ("development-v4", 2026080101, 2026081101, 0, robot)
            for robot in range(1, 15)
        }
        self.assertFalse(observed_initialization & missing_initialization)
        self.assertEqual(
            observed_initialization | missing_initialization,
            frozen_initialization,
        )
        manifest = json.loads((published / "manifest.json").read_text())
        self.assertIn("swarm.jsonl.gz", manifest["member_identities"])
        self.assertIn(
            "swarm.supervisor.manifest.json", manifest["member_identities"]
        )

    def test_swarm_stream_rejects_schema_valid_endpoint_out_of_canonical_order(self):
        from scripts.diagnostics.qualified_closure_evidence import _PAPER_ENDPOINTS
        mission = {
            "campaign_id": "development-v4", "trajectory_seed": 101,
            "range_noise_seed": 201, "frames": 1,
        }
        state = _SwarmStreamState(mission)
        state.initialization = 14
        expected_edge, expected_owner = _PAPER_ENDPOINTS[0]
        wrong_edge, wrong_owner = _PAPER_ENDPOINTS[1]
        self.assertNotEqual((expected_edge, expected_owner), (wrong_edge, wrong_owner))
        row = {
            "record_type": "endpoint_row", "frame_index": 0,
            "edge": dataclasses.asdict(wrong_edge), "owner": wrong_owner,
            "campaign_id": "development-v4", "condition": "dynamic_primary",
            "trajectory_seed": 101, "range_noise_seed": 201,
        }
        with mock.patch(
            "scripts.diagnostics.qualified_closure_evidence.validate_endpoint_primitive_schema",
            return_value=True,
        ):
            self.assertFalse(state(row))
        self.assertEqual(state.reason, "endpoint_order_mismatch")

    def test_schema_valid_unsuccessful_swarm_terminal_is_preserved(self):
        mission = {
            "campaign_id": "development-v4", "trajectory_seed": 101,
            "range_noise_seed": 201, "frames": 1000,
        }
        state = _SwarmStreamState(mission)
        terminal = {
            "record_type": "mission_terminal", "frame_index": 1000,
            "campaign_id": "development-v4", "condition": "dynamic_primary",
            "trajectory_seed": 101, "range_noise_seed": 201,
            "runtime": {
                "success": False, "reason": "bootstrap_failure:singular_fim",
                "process_outcome": "bootstrap_failure", "declared_frames": 1000,
                "completed_intervals": 0,
            },
        }
        with mock.patch(
            "scripts.diagnostics.qualified_closure_evidence.validate_mission_terminal_schema",
            return_value=True,
        ):
            self.assertTrue(state(terminal))
        self.assertTrue(state.complete)
        self.assertFalse(state.success)
        self.assertEqual(state.reason, "bootstrap_failure:singular_fim")

    def test_replay_namespace_contains_only_approved_producer_inputs(self):
        stage = self.root / "mission-stage"
        stage.mkdir()
        truth = stage / "controller-truth.jsonl.gz"
        audit = stage / "audit.jsonl.gz"
        measurements = stage / "runtime.jsonl.gz"
        commands = stage / "commands.jsonl.gz"
        config = stage / "config.json"
        for path in (truth, audit, measurements, commands, config):
            path.write_bytes(b"{}\n")
        manifest = {
            "terminal": True, "status": "completed",
            "runtime_sha256": hashlib.sha256(measurements.read_bytes()).hexdigest(),
            "measurement_stream_id": "b" * 64,
            "config_sha256": hashlib.sha256(config.read_bytes()).hexdigest(),
            "row_count": 10,
        }

        namespace = _prepare_replay_namespace(
            stage, "dynamic_primary", measurements, manifest, commands, config
        )
        self.addCleanup(shutil.rmtree, namespace)

        self.assertNotEqual(namespace.parent, stage)
        self.assertNotIn(stage, namespace.parents)
        self.assertEqual(
            sorted(path.name for path in namespace.iterdir()),
            ["commands.jsonl.gz", "commands.manifest.json", "config.json",
             "measurements.jsonl.gz", "measurements.manifest.json"],
        )
        serialized = " ".join(path.name for path in namespace.iterdir()).lower()
        self.assertNotIn("truth", serialized)
        self.assertNotIn("audit", serialized)

    def test_production_replay_uses_public_replay_module_and_records_full_argv(self):
        mission_stage = self.root / "production-replay"
        mission_stage.mkdir()
        (mission_stage / "swarm-inputs").mkdir()
        commands = mission_stage / "swarm-inputs" / "commands.jsonl.gz"
        with gzip.open(commands, "wt", encoding="utf-8") as sink:
            sink.write('{"frame_index":0,"commands":[]}\n')
        config = mission_stage / "materialized-primary.json"
        config.write_text(json.dumps({
            "position_covariance": {
                "reference-selection": "dynamic-lower-index"
            }
        }))
        measurements_path = mission_stage / "runtime.jsonl.gz"
        with gzip.open(measurements_path, "wt", encoding="utf-8") as sink:
            sink.write("{}\n")
        measurements = {
            "terminal": True,
            "status": "completed",
            "runtime_path": measurements_path,
            "runtime_sha256": hashlib.sha256(
                measurements_path.read_bytes()
            ).hexdigest(),
            "sha256": hashlib.sha256(
                measurements_path.read_bytes()
            ).hexdigest(),
            "measurement_stream_id": "a" * 64,
            "config_sha256": "b" * 64,
            "row_count": 1,
        }
        mission = {
            "campaign_id": "development-v4",
            "trajectory_seed": 2026080101,
            "range_noise_seed": 2026081101,
            "frames": 2,
            "horizon_s": 1.0,
        }
        observed = []

        def supervise(argv, **kwargs):
            observed.append((
                list(argv),
                Path(kwargs["cwd"]),
                list(kwargs["expected_keys"]),
            ))
            with gzip.open(
                kwargs["stream_path"], "wt", encoding="utf-8"
            ) as sink:
                sink.write("{}\n")
            return {
                "terminal": True, "status": "completed",
                "reason": "completed", "valid_rows": 1,
            }

        operations = ProductionOperations()
        operations.protocol = {"bindings": {}}
        with mock.patch(
            "scripts.diagnostics.run_qualified_closure_campaign."
            "supervise_child_to_gzip",
            side_effect=supervise,
        ):
            result = operations.run_replay(
                mission, mission_stage, "dynamic_primary", measurements, {}
            )

        expected_script = str(
            Path(__file__).resolve().parents[1]
            / "scripts/diagnostics/replay_qualified_estimator.py"
        )
        expected_argv = [
            sys.executable, expected_script,
            "--condition", "dynamic_primary",
            "--measurements", "measurements.jsonl.gz",
            "--measurement-sha256", measurements["sha256"],
            "--measurement-manifest", "measurements.manifest.json",
            "--commands", "commands.jsonl.gz",
            "--commands-manifest", "commands.manifest.json",
            "--config", "config.json",
            "--frames", "2",
        ]
        self.assertEqual(observed[0][0], expected_argv)
        self.assertNotIn("__replay-producer", observed[0][0])
        self.assertEqual(result["producer_argv"], expected_argv)
        self.assertNotIn(mission_stage, observed[0][1].parents)
        self.assertEqual(len(observed[0][2]), 28)
        self.assertEqual(observed[0][2][0], (0, 1))
        self.assertEqual(observed[0][2][-1], (1, 14))

    def test_runtime_measurement_validator_rejects_bool_and_forged_provenance(self):
        row = {
            "schema_version": "cbf2026-qualified-measurements-v1",
            "record_type": "runtime_measurement",
            "frame_index": 0,
            "owner_id": 1,
            "reference_key": ["base", 0],
            "role_tags": ["dynamic_primary"],
            "reference_position": [-1550.0, -300.0],
            "reference_covariance": [[0.0, 0.0], [0.0, 0.0]],
            "base_anchor_provenance": [0],
            "noisy_range": 100.0,
            "ranging_sigma": 0.5,
            "config_sha256": "c" * 64,
            "measurement_stream_id": "b" * 64,
        }
        manifest = {
            "config_sha256": "c" * 64,
            "measurement_stream_id": "b" * 64,
        }
        _validate_runtime_measurement_row(
            row, condition="dynamic_primary", manifest=manifest
        )
        for field, value in (
            ("reference_position", [False, -300.0]),
            ("base_anchor_provenance", [False]),
            ("base_anchor_provenance", [100, 101]),
        ):
            with self.subTest(field=field, value=value):
                tampered = copy.deepcopy(row)
                tampered[field] = value
                with self.assertRaises(ValueError):
                    _validate_runtime_measurement_row(
                        tampered,
                        condition="dynamic_primary",
                        manifest=manifest,
                    )

        single_anchor_uav = copy.deepcopy(row)
        single_anchor_uav.update({
            "owner_id": 2,
            "reference_key": ["uav", 1],
            "base_anchor_provenance": [0],
        })
        _validate_runtime_measurement_row(
            single_anchor_uav,
            condition="dynamic_primary",
            manifest=manifest,
        )

    def test_campaign_replay_validator_rejects_bool_diameter_and_forged_provenance(self):
        from tests.test_replay_qualified_estimator import (
            fresh_registered_qualified_row,
        )
        row = fresh_registered_qualified_row()
        _validate_raw_replay_row(row)

        diameter = copy.deepcopy(row)
        diameter["audit_bundle"]["clustering"]["modes"][0][
            "diameter_m"
        ] = False
        with self.assertRaisesRegex(ValueError, "diameter"):
            _validate_raw_replay_row(diameter)

        provenance = copy.deepcopy(row)
        provenance["runtime_inputs"]["references"][0][
            "base_anchor_provenance"
        ] = [100, 101]
        with self.assertRaisesRegex(ValueError, "provenance"):
            _validate_raw_replay_row(provenance)

    def test_validated_raw_row_projects_to_exact_estimator_tuple(self):
        from tests.test_replay_qualified_estimator import (
            fresh_registered_qualified_row,
        )
        raw = fresh_registered_qualified_row()

        projected = project_raw_estimator_tuple(
            raw,
            campaign_id="development-v4",
            condition="dynamic_primary",
            trajectory_seed=2026080101,
            range_noise_seed=2026081101,
            depth=1,
        )

        self.assertTrue(validate_estimator_tuple_schema(projected))
        self.assertEqual(projected["output_status"], raw["public_status"])


class RunnerCliTests(unittest.TestCase):
    def development_arguments(self, root):
        return argparse.Namespace(
            kind="development", version="v4", smoke_id=None,
            protocol=root / "absent-protocol.json",
            authorization=root / "absent-authorization.json",
            binary=root / "absent-Swarm",
            base_config=root / "absent-base.json",
            primary_config=root / "absent-primary.json",
            ablation_config=root / "absent-ablation.json",
            trajectory_seeds="2026080101:2026080110",
            range_noise_seeds="2026081101:2026081110",
            frames=1000,
            output_root=root / "raw" / "v4",
        )

    def test_runtime_development_argv_uses_v4_module_entrypoint_exactly(self):
        root = Path("/private/tmp/qualified-runtime-argv")
        arguments = self.development_arguments(root)

        self.assertEqual(_runtime_runner_argv(arguments), [
            "conda", "run", "-n", "cbf_env", "python", "-m",
            "scripts.diagnostics.run_qualified_closure_campaign",
            "--kind", "development", "--version", "v4",
            "--protocol", str(root / "absent-protocol.json"),
            "--authorization", str(root / "absent-authorization.json"),
            "--binary", str(root / "absent-Swarm"),
            "--base-config", str(root / "absent-base.json"),
            "--primary-config", str(root / "absent-primary.json"),
            "--ablation-config", str(root / "absent-ablation.json"),
            "--trajectory-seeds", "2026080101:2026080110",
            "--range-noise-seeds", "2026081101:2026081110",
            "--frames", "1000",
            "--output-root", str(root / "raw" / "v4"),
        ])

    def test_frozen_runner_reaches_registration_without_claiming_root(self):
        repository = Path(__file__).resolve().parents[1]
        environment = dict(os.environ)
        environment.pop("PYTHONPATH", None)
        with tempfile.TemporaryDirectory(
            prefix="qualified-runner-module-cli-"
        ) as directory:
            root = Path(directory)
            arguments = self.development_arguments(root)
            result = subprocess.run(
                _runtime_runner_argv(arguments),
                cwd=repository,
                env=environment,
                text=True,
                capture_output=True,
                check=False,
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("registered protocol is unreadable", result.stderr)
            self.assertNotIn("No module named 'scripts'", result.stderr)
            self.assertFalse(arguments.output_root.exists())

    def test_help_exposes_exact_campaign_options(self):
        script = Path(__file__).resolve().parents[1] / "scripts" / "diagnostics" / "run_qualified_closure_campaign.py"
        result = subprocess.run(
            [os.sys.executable, str(script), "--help"],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        for option in (
            "--kind", "--version", "--smoke-id", "--protocol",
            "--authorization", "--binary", "--base-config",
            "--primary-config", "--ablation-config", "--trajectory-seeds",
            "--range-noise-seeds", "--frames", "--output-root",
        ):
            self.assertIn(option, result.stdout)


class CampaignCoordinatorTests(unittest.TestCase):
    class FakeOperations:
        def __init__(self, fail_mission=None, fail_phase=None,
                     fail_reason="fake_failure"):
            self.events = []
            self.fail_mission = fail_mission
            self.fail_phase = fail_phase
            self.fail_reason = fail_reason

        def validate_registration(self, arguments):
            self.events.append("validated")
            return {"protocol_sha256": "p" * 64, "authorization_sha256": "a" * 64}

        def collect_identities(self, arguments):
            self.events.append("identities")
            return {"binary_sha256": "b" * 64, "base_sha256": "c" * 64,
                    "primary_sha256": "d" * 64, "ablation_sha256": "e" * 64}

        def schedule_frozen(self, schedule):
            self.events.append(("schedule", len(schedule)))

        def run_swarm(self, mission, mission_stage, config_path):
            self.events.append(("swarm", mission["mission_id"]))
            evidence = mission_stage / "swarm.jsonl.gz"
            with evidence.open("xb") as raw:
                with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as sink:
                    sink.write(b'{"record_type":"validated-prefix"}\n')
            stderr = mission_stage / "swarm.stderr.log"
            stderr.write_text("preserved diagnostic\n")
            missing = mission_stage / "swarm.missing.jsonl.gz"
            with gzip.open(missing, "wt", encoding="utf-8") as sink:
                sink.write('{"record_type":"mission","success":false}\n')
            supervisor = mission_stage / "swarm.supervisor.manifest.json"
            supervisor.write_text(json.dumps({
                "terminal": True,
                "status": "failed" if mission["mission_id"] == self.fail_mission
                    else "completed",
                "reason": self.fail_reason if mission["mission_id"] == self.fail_mission
                    else "completed",
            }))
            commands = mission_stage / "commands.jsonl.gz"
            commands.write_bytes(b"commands")
            truth = mission_stage / "truth.jsonl.gz"
            truth.write_bytes(b"truth")
            return {
                "terminal": True,
                "status": "failed" if mission["mission_id"] == self.fail_mission else "completed",
                "reason": self.fail_reason if mission["mission_id"] == self.fail_mission else "completed",
                "evidence_path": evidence,
                "commands_path": commands,
                "truth_path": truth,
            }

        def generate_measurements(self, mission, mission_stage, swarm):
            self.events.append(("measurements", mission["mission_id"]))
            if self.fail_phase == "measurements":
                raise RuntimeError("injected measurements failure")
            bundle = mission_stage / "measurements"
            bundle.mkdir()
            runtime = bundle / "runtime.jsonl.gz"
            runtime.write_bytes(b"shared")
            if self.fail_phase == "measurements-terminal":
                return {
                    "terminal": True,
                    "status": "failed",
                    "reason": "disk_hard_floor",
                    "runtime_path": runtime,
                    "sha256": "m" * 64,
                }
            return {"terminal": True, "status": "completed",
                    "runtime_path": runtime, "sha256": "m" * 64}

        def run_replay(self, mission, mission_stage, condition, measurements, state):
            self.events.append(("replay", mission["mission_id"], condition,
                                id(state["public"]), measurements["sha256"]))
            if self.fail_phase == f"replay-{condition}":
                raise RuntimeError("injected replay failure")
            state["public"]["condition"] = condition
            output = mission_stage / f"{condition}.jsonl.gz"
            output.write_bytes(condition.encode())
            return {"terminal": True, "status": "completed",
                    "sha256": condition, "path": output}

        def synthesize_failed_mission(self, mission, mission_stage, reason):
            self.events.append(("synthesize", mission["mission_id"], reason))
            (mission_stage / "synthetic.jsonl.gz").write_bytes(b"synthetic")

    def arguments(self, output_root):
        import argparse
        return argparse.Namespace(
            kind="development", version="v4", smoke_id=None,
            protocol=Path("protocol.json"), authorization=Path("authorization.json"),
            binary=Path("Swarm"), base_config=Path("config.json"),
            primary_config=Path("primary.json"), ablation_config=Path("ablation.json"),
            trajectory_seeds="2026080101:2026080110",
            range_noise_seeds="2026081101:2026081110", frames=1000,
            output_root=output_root,
        )

    def test_ten_missions_run_sequentially_with_shared_measurements_and_isolated_replays(self):
        with tempfile.TemporaryDirectory(prefix="qualified-campaign-flow-") as directory:
            output = Path(directory) / "raw" / "v1"
            operations = self.FakeOperations()
            manifest = execute_campaign(self.arguments(output), operations)

            self.assertEqual(manifest["status"], "completed", manifest)
            self.assertEqual(operations.events[:3], ["validated", "identities", ("schedule", 10)])
            swarms = [event for event in operations.events if event[0] == "swarm"]
            self.assertEqual(len(swarms), 10)
            replays = [event for event in operations.events if event[0] == "replay"]
            self.assertEqual(len(replays), 20)
            for mission_number in range(1, 11):
                mission = f"mission-{mission_number:02d}"
                paired = [event for event in replays if event[1] == mission]
                self.assertEqual({event[2] for event in paired}, {
                    "dynamic_primary", "fixed_fim_ablation"
                })
                self.assertEqual({event[4] for event in paired}, {"m" * 64})
                self.assertEqual(len({event[3] for event in paired}), 2)
            self.assertTrue((output / "manifest.json").is_file())
            campaign_manifest = json.loads((output / "manifest.json").read_text())
            self.assertEqual(
                campaign_manifest["schedule_sha256"],
                hashlib.sha256((output / "schedule.json").read_bytes()).hexdigest(),
            )
            self.assertEqual(
                set(campaign_manifest["mission_manifest_identities"]),
                {f"mission-{number:02d}" for number in range(1, 11)},
            )
            self.assertEqual(len(list(output.glob("mission-*"))), 10)

    def test_failure_synthesizes_remaining_schedule_and_returns_terminal_failure(self):
        with tempfile.TemporaryDirectory(prefix="qualified-campaign-failure-") as directory:
            output = Path(directory) / "raw" / "v1"
            operations = self.FakeOperations(fail_mission="mission-03")
            manifest = execute_campaign(self.arguments(output), operations)

            self.assertEqual(manifest["status"], "failed")
            self.assertEqual(manifest["reason"], "fake_failure")
            self.assertTrue((output / "manifest.json").is_file())
            synthetic = [event for event in operations.events if event[0] == "synthesize"]
            self.assertEqual([event[1] for event in synthetic], [
                f"mission-{number:02d}" for number in range(3, 11)
            ])

    def test_terminal_measurement_disk_stop_preserves_exact_failure_reason(self):
        with tempfile.TemporaryDirectory(
            prefix="qualified-campaign-measurement-disk-stop-"
        ) as directory:
            output = Path(directory) / "raw" / "v1"
            operations = self.FakeOperations(
                fail_phase="measurements-terminal"
            )

            manifest = execute_campaign(self.arguments(output), operations)

            self.assertEqual(manifest["status"], "failed")
            self.assertEqual(manifest["reason"], "disk_hard_floor")
            for mission_number in range(1, 11):
                mission_manifest = json.loads((
                    output / f"mission-{mission_number:02d}" / "manifest.json"
                ).read_text())
                self.assertEqual(mission_manifest["reason"], "disk_hard_floor")

    def test_all_swarm_failure_classes_preserve_validated_prefix_and_supervisor_bundle(self):
        for reason in (
            "malformed_json", "child_nonzero_exit", "line_stall_timeout",
            "mission_declared_unsuccessful",
        ):
            with self.subTest(reason=reason), tempfile.TemporaryDirectory(
                prefix="qualified-campaign-prefix-"
            ) as directory:
                output = Path(directory) / "raw" / "v1"
                operations = self.FakeOperations(
                    fail_mission="mission-01", fail_reason=reason
                )

                manifest = execute_campaign(self.arguments(output), operations)

                self.assertEqual(manifest["status"], "failed")
                mission_root = output / "mission-01"
                with gzip.open(
                    mission_root / "swarm.jsonl.gz", "rt", encoding="utf-8"
                ) as source:
                    self.assertEqual(
                        json.loads(source.readline())["record_type"],
                        "validated-prefix",
                    )
                for name in (
                    "swarm.stderr.log", "swarm.supervisor.manifest.json",
                    "swarm.missing.jsonl.gz", "synthetic.jsonl.gz",
                ):
                    self.assertTrue((mission_root / name).is_file(), name)
                mission_manifest = json.loads(
                    (mission_root / "manifest.json").read_text()
                )
                self.assertEqual(mission_manifest["reason"], reason)
                for relative, identity in mission_manifest[
                    "member_identities"
                ].items():
                    member = mission_root / relative
                    self.assertEqual(identity, {
                        "sha256": hashlib.sha256(member.read_bytes()).hexdigest(),
                        "bytes": member.stat().st_size,
                    })

    def test_every_post_claim_phase_failure_still_terminalizes_full_schedule(self):
        import scripts.diagnostics.run_qualified_closure_campaign as runner

        for phase in (
            "campaign-setup",
            "measurements", "replay-dynamic_primary",
            "replay-fixed_fim_ablation", "mission-publication",
        ):
            with self.subTest(phase=phase), tempfile.TemporaryDirectory(
                prefix="qualified-campaign-post-claim-failure-"
            ) as directory:
                output = Path(directory) / "raw" / "v1"
                operations = self.FakeOperations(
                    fail_phase=None if phase == "mission-publication" else phase
                )
                original = runner._publish_json_no_replace
                injected = {"done": False}

                def publish(path, payload):
                    path = Path(path)
                    if (
                        phase == "campaign-setup"
                        and not injected["done"]
                        and path.name == "schedule.json"
                    ):
                        injected["done"] = True
                        raise RuntimeError("injected campaign setup failure")
                    if (
                        phase == "mission-publication"
                        and not injected["done"]
                        and path.name == "manifest.json"
                        and path.parent.name.startswith(".mission-01.")
                    ):
                        injected["done"] = True
                        raise RuntimeError("injected mission publication failure")
                    return original(path, payload)

                with mock.patch.object(runner, "_publish_json_no_replace", publish):
                    manifest = execute_campaign(self.arguments(output), operations)

                self.assertEqual(manifest["status"], "failed")
                self.assertTrue(manifest["terminal"])
                self.assertTrue((output / "manifest.json").is_file())
                self.assertEqual(
                    sorted(path.name for path in output.glob("mission-*")),
                    [f"mission-{number:02d}" for number in range(1, 11)],
                )
                self.assertFalse(any(output.glob(".mission-*.tmp")))
                synthetic = [
                    event[1] for event in operations.events
                    if event[0] == "synthesize"
                ]
                self.assertEqual(synthetic, [
                    f"mission-{number:02d}" for number in range(1, 11)
                ])

    def test_rename_failure_after_completed_manifest_republishes_failed_bundle(self):
        import scripts.diagnostics.run_qualified_closure_campaign as runner

        with tempfile.TemporaryDirectory(
            prefix="qualified-campaign-rename-failure-"
        ) as directory:
            output = Path(directory) / "raw" / "v1"
            operations = self.FakeOperations()
            original = runner._rename_directory_no_replace
            injected = {"done": False}

            def rename(stage, target):
                stage = Path(stage)
                target = Path(target)
                manifest_path = stage / "manifest.json"
                if (
                    not injected["done"]
                    and target.name == "mission-01"
                    and manifest_path.is_file()
                    and json.loads(manifest_path.read_text())["status"]
                        == "completed"
                ):
                    injected["done"] = True
                    raise RuntimeError("injected rename failure")
                return original(stage, target)

            with mock.patch.object(
                runner, "_rename_directory_no_replace", rename
            ):
                manifest = execute_campaign(self.arguments(output), operations)

            self.assertTrue(injected["done"])
            self.assertEqual(manifest["status"], "failed")
            self.assertEqual(
                manifest["reason"],
                "mission_publication_exception:RuntimeError",
            )
            self.assertEqual(
                sorted(path.name for path in output.glob("mission-*")),
                [f"mission-{number:02d}" for number in range(1, 11)],
            )
            for mission_number in range(1, 11):
                mission_root = output / f"mission-{mission_number:02d}"
                mission_manifest = json.loads(
                    (mission_root / "manifest.json").read_text()
                )
                self.assertTrue(mission_manifest["terminal"])
                self.assertEqual(mission_manifest["status"], "failed")
                self.assertEqual(
                    mission_manifest["reason"],
                    "mission_publication_exception:RuntimeError",
                )
                expected_members = {
                    str(path.relative_to(mission_root))
                    for path in mission_root.rglob("*")
                    if path.is_file()
                    and not (
                        path.name == "manifest.json"
                        and path.parent == mission_root
                    )
                }
                self.assertEqual(
                    set(mission_manifest["member_identities"]),
                    expected_members,
                )
                for relative, identity in mission_manifest[
                    "member_identities"
                ].items():
                    member = mission_root / relative
                    self.assertEqual(identity, {
                        "sha256": hashlib.sha256(
                            member.read_bytes()
                        ).hexdigest(),
                        "bytes": member.stat().st_size,
                    })

    def test_swarm_failed_bundle_rename_retry_does_not_resynthesize(self):
        import scripts.diagnostics.run_qualified_closure_campaign as runner

        with tempfile.TemporaryDirectory(
            prefix="qualified-campaign-failed-rename-retry-"
        ) as directory:
            output = Path(directory) / "raw" / "v1"
            operations = self.FakeOperations(fail_mission="mission-01")
            original = runner._rename_directory_no_replace
            injected = {"done": False}

            def rename(stage, target):
                stage = Path(stage)
                target = Path(target)
                manifest_path = stage / "manifest.json"
                if (
                    not injected["done"]
                    and target.name == "mission-01"
                    and manifest_path.is_file()
                    and json.loads(manifest_path.read_text())["status"]
                        == "failed"
                ):
                    injected["done"] = True
                    raise RuntimeError("injected failed-bundle rename failure")
                return original(stage, target)

            with mock.patch.object(
                runner, "_rename_directory_no_replace", rename
            ):
                manifest = execute_campaign(self.arguments(output), operations)

            self.assertTrue(injected["done"])
            self.assertEqual(manifest["status"], "failed")
            self.assertEqual(manifest["reason"], "fake_failure")
            self.assertEqual(
                [
                    event[1] for event in operations.events
                    if event[0] == "synthesize"
                ],
                [f"mission-{number:02d}" for number in range(1, 11)],
            )
            for mission_number in range(1, 11):
                mission_manifest = json.loads((
                    output / f"mission-{mission_number:02d}" / "manifest.json"
                ).read_text())
                self.assertEqual(mission_manifest["status"], "failed")
                self.assertEqual(mission_manifest["reason"], "fake_failure")

    def test_partial_uncommitted_synthetic_is_removed_before_rebuild(self):
        class PartialSyntheticOperations(self.FakeOperations):
            def __init__(self):
                super().__init__(fail_mission="mission-01")
                self.partial_injected = False

            def synthesize_failed_mission(
                self, mission, mission_stage, reason
            ):
                self.events.append(("synthesize", mission["mission_id"], reason))
                synthetic = mission_stage / "synthetic-missing.jsonl.gz"
                if not self.partial_injected:
                    self.partial_injected = True
                    with synthetic.open("xb") as sink:
                        sink.write(b"partial")
                    raise RuntimeError("injected interrupted synthesis")
                with synthetic.open("xb") as sink:
                    sink.write(b"complete")

        with tempfile.TemporaryDirectory(
            prefix="qualified-campaign-partial-synthetic-"
        ) as directory:
            output = Path(directory) / "raw" / "v1"
            operations = PartialSyntheticOperations()

            manifest = execute_campaign(self.arguments(output), operations)

            self.assertTrue(operations.partial_injected)
            self.assertEqual(manifest["status"], "failed")
            self.assertEqual(
                manifest["reason"], "swarm_exception:RuntimeError"
            )
            self.assertEqual(
                (output / "mission-01" / "synthetic-missing.jsonl.gz")
                    .read_bytes(),
                b"complete",
            )
            self.assertEqual(
                [
                    event[1] for event in operations.events
                    if event[0] == "synthesize"
                ],
                ["mission-01", *(
                    f"mission-{number:02d}" for number in range(1, 11)
                )],
            )


class UnscheduledValidatedStreamTests(unittest.TestCase):
    def test_variable_validated_stream_does_not_require_a_fixed_key_iterator(self):
        with tempfile.TemporaryDirectory(prefix="qualified-variable-stream-") as directory:
            root = Path(directory)
            child = root / "child.py"
            child.write_text("print('{\\\"kind\\\":\\\"reset\\\"}')\nprint('{\\\"kind\\\":\\\"controller\\\"}')\n")
            manifest = supervise_child_to_gzip(
                [sys.executable, str(child)],
                stream_path=root / "rows.jsonl.gz",
                stderr_path=root / "stderr.log",
                manifest_path=root / "manifest.json",
                expected_keys=None,
                row_validator=lambda row: row.get("kind") in {"reset", "controller"},
                available_bytes_fn=lambda _path: STOP_FREE_BYTES + 1,
            )
            self.assertEqual(manifest["status"], "completed", manifest)
            self.assertEqual(manifest["valid_rows"], 2)


class RunnerCliEndToEndTests(unittest.TestCase):
    def test_cli_rejects_arbitrary_operations_injection_before_root_claim(self):
        with tempfile.TemporaryDirectory(prefix="qualified-runner-cli-") as directory:
            root = Path(directory)
            module = root / "qualified_fake_operations.py"
            module.write_text('''
from pathlib import Path

class Operations:
    def validate_registration(self, arguments):
        return {"protocol_sha256": "p" * 64, "authorization_sha256": "a" * 64}
    def collect_identities(self, arguments):
        return {"binary_sha256": "b" * 64, "base_sha256": "c" * 64,
                "primary_sha256": "d" * 64, "ablation_sha256": "e" * 64}
    def schedule_frozen(self, schedule):
        if len(schedule) != 10: raise RuntimeError("schedule")
    def run_swarm(self, mission, stage, config):
        evidence = stage / "swarm.jsonl.gz"; evidence.write_bytes(b"swarm")
        command = stage / "commands.jsonl.gz"; command.write_bytes(b"commands")
        truth = stage / "truth.jsonl.gz"; truth.write_bytes(b"truth")
        return {"terminal": True, "status": "completed", "reason": "completed",
                "evidence_path": evidence, "commands_path": command, "truth_path": truth}
    def generate_measurements(self, mission, stage, swarm):
        bundle = stage / "measurements"; bundle.mkdir()
        runtime = bundle / "runtime.jsonl.gz"; runtime.write_bytes(b"shared")
        return {"terminal": True, "status": "completed", "runtime_path": runtime,
                "sha256": "m" * 64}
    def run_replay(self, mission, stage, condition, measurements, state):
        output = stage / (condition + ".jsonl.gz"); output.write_bytes(condition.encode())
        return {"terminal": True, "status": "completed", "path": output,
                "sha256": condition}
    def synthesize_failed_mission(self, mission, stage, reason):
        (stage / "synthetic.jsonl.gz").write_bytes(b"synthetic")
''')
            output = root / "raw" / "v1"
            script = Path(__file__).resolve().parents[1] / "scripts" / "diagnostics" / "run_qualified_closure_campaign.py"
            environment = {
                **os.environ,
                "PYTHONPATH": os.pathsep.join([str(root), str(Path(__file__).resolve().parents[1])]),
                "CBF2026_QUALIFIED_OPERATIONS": "qualified_fake_operations:Operations",
            }
            result = subprocess.run([
                sys.executable, str(script), "--kind", "development", "--version", "v1",
                "--protocol", str(root / "protocol.json"), "--authorization", str(root / "authorization.json"),
                "--binary", str(root / "Swarm"), "--base-config", str(root / "base.json"),
                "--primary-config", str(root / "primary.json"), "--ablation-config", str(root / "ablation.json"),
                "--trajectory-seeds", "2026080101:2026080110",
                "--range-noise-seeds", "2026081101:2026081110", "--frames", "1000",
                "--output-root", str(output),
            ], cwd=Path(__file__).resolve().parents[1], env=environment,
               text=True, capture_output=True, check=False)

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("registered protocol", result.stderr)
            self.assertFalse(output.exists())

if __name__ == "__main__":
    unittest.main()
