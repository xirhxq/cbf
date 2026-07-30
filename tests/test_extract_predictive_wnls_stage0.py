"""Contracts for immutable predictive-WNLS Stage 0 fixture extraction."""

import hashlib
import copy
import gzip
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import scripts.diagnostics.extract_predictive_wnls_stage0 as stage0
from scripts.diagnostics.extract_predictive_wnls_stage0 import (
    STAGE0_SCHEMA_ID,
    _result_row,
    _strict_json_bytes,
    extract_stage0_fixtures,
)


FIXTURE_ROOT = Path("tests/fixtures/cbf2026_predictive_wnls")
TRUTH_PATH = Path(
    "/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/"
    "20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/"
    "2026-07-28_14-27-53_R_seed_20260727_250s/data.json"
)


class Stage0ExtractorContractTests(unittest.TestCase):
    def _relocated_legacy_sources(self, root):
        root = root.resolve()
        extractor = root / "extract_predictive_wnls_stage0.py"
        legacy = root / "replay_localization_calibration.py"
        extractor.write_text("# relocated extractor source\n")
        legacy.write_bytes(
            Path(stage0.legacy_replay_module.__file__).resolve().read_bytes()
        )
        return extractor, legacy

    def test_legacy_source_verification_relocates_with_imported_module(self):
        with tempfile.TemporaryDirectory() as temporary:
            extractor, legacy = self._relocated_legacy_sources(
                Path(temporary)
            )
            with mock.patch.object(
                stage0,
                "__file__",
                str(extractor),
            ), mock.patch.object(
                stage0.legacy_replay_module,
                "__file__",
                str(legacy),
            ):
                before = stage0._verified_legacy_before()
                identity = stage0._verify_legacy_after(before)
            self.assertEqual(identity, {
                "path": str(legacy),
                "sha256": stage0.LEGACY_REPLAY_SHA256,
            })

    def test_legacy_source_rejects_imported_module_path_mismatch(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve()
            extractor, legacy = self._relocated_legacy_sources(root)
            separate = root / "separate" / legacy.name
            separate.parent.mkdir()
            separate.write_bytes(legacy.read_bytes())
            with mock.patch.object(
                stage0,
                "__file__",
                str(extractor),
            ), mock.patch.object(
                stage0.legacy_replay_module,
                "__file__",
                str(separate),
            ):
                with self.assertRaisesRegex(
                    ValueError,
                    "imported legacy replay module path mismatch",
                ):
                    stage0._verified_legacy_before()

    def _registered_json(self, root):
        root = root.resolve()
        source = root / "registered.json"
        source.write_text('{"value":1}')
        return source, {
            "truth_data": {
                "path": source,
                "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            },
        }

    def _extractor_sources(self, root):
        root = root.resolve()
        truth = root / "truth-source.json"
        manifest = root / "input-manifest-source.json"
        baseline = root / "baseline-source.jsonl.gz"
        truth.write_text(json.dumps({"config": {}, "state": []}))
        manifest.write_text("{}")
        with gzip.open(baseline, "wt") as destination:
            destination.write("{}\n")
        paths = {
            "truth_data": truth,
            "input_manifest": manifest,
            "baseline_process": baseline,
        }
        registry = {
            name: {
                "path": path,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
            for name, path in paths.items()
        }
        return paths, registry

    def test_registered_json_requires_exact_absolute_path_and_hash(self):
        with tempfile.TemporaryDirectory() as temporary:
            source, registry = self._registered_json(Path(temporary))
            with mock.patch.object(
                stage0,
                "FROZEN_SOURCE_REGISTRY",
                registry,
                create=True,
            ):
                value, identity = stage0._read_registered_json(
                    "truth_data",
                    source,
                )
            self.assertEqual(value, {"value": 1})
            self.assertEqual(identity["path"], str(source))
            self.assertEqual(identity["sha256"], registry["truth_data"]["sha256"])

    def test_registered_json_rejects_wrong_relative_and_symlink_spellings(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, registry = self._registered_json(root)
            wrong = root / "wrong" / source.name
            wrong.parent.mkdir()
            wrong.write_bytes(source.read_bytes())
            file_alias = root / "file-alias.json"
            file_alias.symlink_to(source)
            parent_alias = root / "parent-alias"
            parent_alias.symlink_to(source.parent, target_is_directory=True)
            relative = Path(os.path.relpath(source, Path.cwd()))
            with mock.patch.object(
                stage0,
                "FROZEN_SOURCE_REGISTRY",
                registry,
                create=True,
            ):
                for candidate in (
                    wrong,
                    relative,
                    file_alias,
                    parent_alias / source.name,
                ):
                    with self.subTest(candidate=candidate):
                        with self.assertRaises(ValueError):
                            stage0._read_registered_json(
                                "truth_data",
                                candidate,
                            )

    def test_registered_json_rejects_same_hash_path_replacement_during_read(self):
        with tempfile.TemporaryDirectory() as temporary:
            source, registry = self._registered_json(Path(temporary))
            original_load = stage0._strict_load

            def replace_after_read(path):
                value = original_load(path)
                replacement = path.with_name("replacement.json")
                replacement.write_bytes(path.read_bytes())
                replacement.replace(path)
                return value

            with mock.patch.object(
                stage0,
                "FROZEN_SOURCE_REGISTRY",
                registry,
                create=True,
            ), mock.patch.object(
                stage0,
                "_strict_load",
                side_effect=replace_after_read,
            ):
                with self.assertRaisesRegex(ValueError, "changed while read"):
                    stage0._read_registered_json("truth_data", source)

    def test_predecessor_command_index_requires_exact_unique_successful_ids(self):
        valid = {
            "robots": [
                {
                    "id": identifier,
                    "opt": {
                        "status": "success",
                        "result": {"vx": identifier + 0.25, "vy": -identifier},
                    },
                }
                for identifier in (1, 2, 3)
            ],
        }
        self.assertEqual(
            stage0._validated_predecessor_commands(valid, {1, 2, 3}),
            {
                1: [1.25, -1.0],
                2: [2.25, -2.0],
                3: [3.25, -3.0],
            },
        )
        mutations = {}
        duplicate = copy.deepcopy(valid)
        duplicate["robots"][2]["id"] = 2
        mutations["duplicate"] = duplicate
        missing = copy.deepcopy(valid)
        missing["robots"].pop()
        mutations["missing"] = missing
        extra = copy.deepcopy(valid)
        extra["robots"].append(copy.deepcopy(valid["robots"][2]))
        extra["robots"][-1]["id"] = 4
        mutations["extra"] = extra
        bad_status = copy.deepcopy(valid)
        bad_status["robots"][0]["opt"]["status"] = "failed"
        mutations["bad_status"] = bad_status
        malformed = copy.deepcopy(valid)
        malformed["robots"][0]["opt"]["result"]["vx"] = float("nan")
        mutations["malformed_command"] = malformed
        for name, frame in mutations.items():
            with self.subTest(name=name):
                with self.assertRaises(ValueError):
                    stage0._validated_predecessor_commands(
                        frame,
                        {1, 2, 3},
                    )

    def test_extractor_manifest_binds_exact_registered_paths_and_hashes(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            paths, registry = self._extractor_sources(root)
            extractor, legacy = self._relocated_legacy_sources(root)
            output = root.resolve() / "output"
            with mock.patch.object(
                stage0,
                "FROZEN_SOURCE_REGISTRY",
                registry,
            ), mock.patch.object(
                stage0,
                "__file__",
                str(extractor),
            ), mock.patch.object(
                stage0.legacy_replay_module,
                "__file__",
                str(legacy),
            ), mock.patch.object(
                stage0,
                "_runtime_prefix",
                return_value=(
                    {"kind": "historical_prefix"},
                    {"truth_positions": []},
                ),
            ), mock.patch.object(
                stage0,
                "_comparison_row",
                return_value={},
            ):
                result = extract_stage0_fixtures(
                    truth_data_path=paths["truth_data"],
                    input_manifest_path=paths["input_manifest"],
                    baseline_process_path=paths["baseline_process"],
                    output_dir=output,
                )
            self.assertEqual(
                result["sources"],
                {
                    name: {
                        "path": str(entry["path"]),
                        "sha256": entry["sha256"],
                    }
                    for name, entry in registry.items()
                } | {
                    "legacy_replay": {
                        "path": str(legacy),
                        "sha256": stage0.LEGACY_REPLAY_SHA256,
                    },
                },
            )

    def test_extractor_rejects_legacy_source_mutation_during_generation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            paths, registry = self._extractor_sources(root)
            extractor, legacy = self._relocated_legacy_sources(root)

            def mutate_legacy(*_args, **_kwargs):
                legacy.write_text("# changed during extraction\n")
                return (
                    {"kind": "historical_prefix"},
                    {"truth_positions": []},
                )

            with mock.patch.object(
                stage0,
                "FROZEN_SOURCE_REGISTRY",
                registry,
            ), mock.patch.object(
                stage0,
                "__file__",
                str(extractor),
            ), mock.patch.object(
                stage0.legacy_replay_module,
                "__file__",
                str(legacy),
            ), mock.patch.object(
                stage0,
                "_runtime_prefix",
                side_effect=mutate_legacy,
            ), mock.patch.object(
                stage0,
                "_comparison_row",
                return_value={},
            ):
                with self.assertRaisesRegex(
                    ValueError,
                    "legacy.*changed|changed.*legacy",
                ):
                    extract_stage0_fixtures(
                        truth_data_path=paths["truth_data"],
                        input_manifest_path=paths["input_manifest"],
                        baseline_process_path=paths["baseline_process"],
                        output_dir=root.resolve() / "output",
                    )

    def test_reacquisition_online_rows_precede_offline_annotation(self):
        fixture = stage0._reacquisition_fixture()
        runtime_rows = stage0._run_reacquisition(fixture["runtime"])
        self.assertTrue(runtime_rows)
        self.assertTrue(
            all("offline_error_norm" not in row for row in runtime_rows)
        )
        annotated = stage0._annotate_offline_rows(
            runtime_rows,
            fixture["offline"],
        )
        self.assertEqual(
            [row["offline_error_norm"] for row in annotated],
            [None, 0.0],
        )

    def test_real_estimator_boundaries_receive_runtime_only_values(self):
        forbidden = ("truth", "true_range", "offline", "baseline")
        calls = []

        def assert_clean(value):
            if isinstance(value, dict):
                for key, item in value.items():
                    self.assertFalse(
                        any(token in str(key).lower() for token in forbidden),
                        key,
                    )
                    assert_clean(item)
            elif isinstance(value, (list, tuple)):
                for item in value:
                    assert_clean(item)

        def guarded(original, name):
            def invoke(*args, **kwargs):
                assert_clean(args)
                assert_clean(kwargs)
                calls.append(name)
                return original(*args, **kwargs)
            return invoke

        fixture_path = FIXTURE_ROOT / "reacquisition.json"
        with mock.patch.object(
            stage0,
            "qualify_active_references",
            side_effect=guarded(
                stage0.qualify_active_references,
                "qualification",
            ),
        ), mock.patch.object(
            stage0,
            "solve_predictive_multistart",
            side_effect=guarded(
                stage0.solve_predictive_multistart,
                "estimator",
            ),
        ), mock.patch.object(
            stage0,
            "finalize_attempt",
            side_effect=guarded(stage0.finalize_attempt, "finalizer"),
        ):
            stage0.run_stage0_fixture(fixture_path)
        self.assertIn("qualification", calls)
        self.assertIn("estimator", calls)
        self.assertIn("finalizer", calls)

    def test_stale_baseline_preserves_disjoint_output_and_failed_attempt(self):
        stale_row = {
            "graph_case": "dynamic_dag_wnls",
            "seed": 7,
            "frame_index": 44,
            "robot_id": 14,
            "status": "stale",
            "estimate": [10.0, 20.0],
            "covariance": [[2.0, 0.0], [0.0, 3.0]],
            "epsilon": 5.0,
            "error_norm": 12.5,
            "failure": {
                "status": "failed",
                "estimate": None,
                "covariance": None,
                "epsilon": None,
                "iterations": 50,
                "cost": 406.5,
                "stationarity_norm": 8.3e-7,
                "failure_reason": "maximum WNLS iterations exceeded",
            },
        }
        comparison = stage0._comparison_row(
            [stale_row],
            seed=7,
            frame_index=44,
            robot_id=14,
        )
        self.assertEqual(comparison["baseline_output"]["status"], "stale")
        self.assertEqual(
            comparison["baseline_output"]["estimate"],
            [10.0, 20.0],
        )
        self.assertEqual(comparison["baseline_attempt"]["status"], "failed")
        self.assertEqual(comparison["baseline_attempt"]["iterations"], 50)
        self.assertIsNone(comparison["baseline_attempt"]["estimate"])

    def test_committed_historical_prefixes_are_complete_and_truth_free_at_runtime(self):
        for filename, seed, last_frame in (
            ("frame44_recovery.json", 20260736, 44),
            ("frame177_cascade.json", 20260730, 177),
        ):
            with self.subTest(filename=filename):
                fixture = json.loads((FIXTURE_ROOT / filename).read_text())
                runtime = fixture["runtime"]
                self.assertEqual(fixture["schema_id"], STAGE0_SCHEMA_ID)
                self.assertEqual(runtime["seed"], seed)
                self.assertEqual(
                    [frame["frame_index"] for frame in runtime["frames"]],
                    list(range(last_frame + 1)),
                )
                self.assertTrue(all(
                    [entry["robot_id"] for entry in frame["uavs"]] == list(range(1, 15))
                    for frame in runtime["frames"]
                ))
                self.assertNotIn("truth", _strict_json_bytes(runtime).decode("utf-8"))
                self.assertNotIn("true_range", _strict_json_bytes(runtime).decode("utf-8"))
                self.assertNotIn("offline_error", _strict_json_bytes(runtime).decode("utf-8"))
                self.assertNotIn("baseline", _strict_json_bytes(runtime).decode("utf-8"))
                self.assertIn("truth_positions", fixture["offline"])
                self.assertIn("baseline_output", fixture["offline"])
                self.assertIn("baseline_attempt", fixture["offline"])

    def test_first_transition_uses_preceding_applied_command(self):
        fixture = json.loads((FIXTURE_ROOT / "frame44_recovery.json").read_text())
        truth = json.loads(TRUTH_PATH.read_text())
        for robot_id in range(1, 15):
            expected = truth["state"][0]["robots"][robot_id - 1]["opt"]["result"]
            recorded = fixture["runtime"]["frames"][1]["uavs"][robot_id - 1]["transition_velocity"]
            self.assertEqual(recorded, [expected["vx"], expected["vy"]])

    def test_manifest_matches_every_committed_fixture_hash(self):
        manifest = json.loads((FIXTURE_ROOT / "manifest.json").read_text())
        self.assertEqual(manifest["schema_id"], STAGE0_SCHEMA_ID)
        self.assertEqual(
            manifest["sources"],
            {
                name: {
                    "path": str(entry["path"]),
                    "sha256": entry["sha256"],
                }
                for name, entry in stage0.FROZEN_SOURCE_REGISTRY.items()
            } | {
                "legacy_replay": {
                    "path": str(
                        Path(stage0.legacy_replay_module.__file__).resolve()
                    ),
                    "sha256": stage0.LEGACY_REPLAY_SHA256,
                },
            },
        )
        for filename, expected_hash in manifest["fixtures"].items():
            self.assertEqual(
                hashlib.sha256((FIXTURE_ROOT / filename).read_bytes()).hexdigest(),
                expected_hash,
            )

    def test_committed_baseline_comparisons_split_output_from_attempt(self):
        frame44 = json.loads(
            (FIXTURE_ROOT / "frame44_recovery.json").read_text()
        )
        offline = frame44["offline"]
        self.assertEqual(offline["baseline_output"]["status"], "stale")
        self.assertEqual(offline["baseline_attempt"]["status"], "failed")
        self.assertEqual(offline["baseline_attempt"]["iterations"], 50)
        self.assertIsNone(offline["baseline_attempt"]["estimate"])

    def test_malformed_historical_runtime_fails_as_invalid_fixture(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "malformed.json"
            path.write_text(json.dumps({
                "schema_id": STAGE0_SCHEMA_ID,
                "runtime": {"kind": "historical_prefix"},
                "offline": {"truth_positions": []},
            }))
            with self.assertRaisesRegex(
                ValueError,
                "malformed historical runtime",
            ):
                stage0.run_stage0_fixture(path)

    def test_used_uav_reference_records_its_current_fresh_status(self):
        row = _result_row(
            frame_index=0,
            robot_id=2,
            output={"attempt_status": "accepted", "output_status": "fresh", "estimate": [0.0, 0.0]},
            attempt={"candidates": [], "selected_candidate": None},
            qualification={
                "active_keys": [("uav", 1)],
                "active_records": [{"key": ("uav", 1), "present": True, "noisy_range": 1.0}],
                "excluded": [],
            },
            uav_outputs={1: {"output_status": "fresh"}},
        )
        self.assertEqual(row["reference_freshness"][0]["output_status"], "fresh")

    def test_direct_script_invocation_resolves_repository_imports(self):
        completed = subprocess.run(
            [sys.executable, "scripts/diagnostics/extract_predictive_wnls_stage0.py", "--help"],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_deterministic_json_bytes_have_a_stable_hash(self):
        payload = {"z": [3, 2, 1], "a": {"x": 1}}
        first = _strict_json_bytes(payload)
        second = _strict_json_bytes(payload)
        self.assertEqual(first, second)
        self.assertEqual(
            hashlib.sha256(first).hexdigest(),
            hashlib.sha256(second).hexdigest(),
        )

    def test_existing_output_directory_fails_closed_before_writing(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            truth = root / "truth.json"
            manifest = root / "manifest.json"
            baseline = root / "baseline.jsonl"
            output = root / "already-exists"
            truth.write_text(json.dumps({"config": {}, "state": []}))
            manifest.write_text("{}")
            baseline.write_text("")
            output.mkdir()

            with self.assertRaises(FileExistsError):
                extract_stage0_fixtures(
                    truth_data_path=truth,
                    input_manifest_path=manifest,
                    baseline_process_path=baseline,
                    output_dir=output,
                )

    def test_absent_parent_directory_is_created_for_a_new_fixture_root(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            paths, registry = self._extractor_sources(root)
            output = root.resolve() / "missing-parent" / "fixtures"
            with mock.patch.object(
                stage0,
                "FROZEN_SOURCE_REGISTRY",
                registry,
            ), mock.patch(
                "scripts.diagnostics.extract_predictive_wnls_stage0._runtime_prefix",
                return_value=({"kind": "historical_prefix"}, {"truth_positions": []}),
            ), mock.patch(
                "scripts.diagnostics.extract_predictive_wnls_stage0._comparison_row",
                return_value={},
            ):
                extract_stage0_fixtures(
                    truth_data_path=paths["truth_data"],
                    input_manifest_path=paths["input_manifest"],
                    baseline_process_path=paths["baseline_process"],
                    output_dir=output,
                )
            self.assertTrue((output / "manifest.json").is_file())


if __name__ == "__main__":
    unittest.main()
