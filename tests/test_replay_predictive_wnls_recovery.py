"""Adversarial contracts for the exact Stage-1 predictive replay."""

import copy
import gzip
import hashlib
import json
import math
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

import scripts.diagnostics.predictive_wnls as estimator
import scripts.diagnostics.replay_predictive_wnls_recovery as replay
from scripts.diagnostics.run_diagnostic import DiskSpaceError


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def open_fd_count() -> int:
    return len(os.listdir("/dev/fd"))


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, allow_nan=False, sort_keys=True))


def tiny_data() -> dict:
    config = {
        "num": 2,
        "formation": {"parts": 1, "bases-id": [[0, 1]]},
        "bases": [[0.0, 0.0], [10.0, 0.0], [0.0, 10.0]],
        "initial": {"position": {"positions": [[3.0, 4.0], [5.0, 4.0]]}},
        "position_covariance": {"ranging_sigma": 0.5},
        "cbfs": {"without-slack": {"comm-fixed": {
            "max-range": 100.0,
            "min-neighbour-id-offset": -2,
            "max-neighbour-id-offset": 0,
        }}},
    }

    def robot(identifier, x, command, nominal):
        return {
            "id": identifier,
            "state": {"x": x, "y": 4.0},
            "opt": {
                "status": "success",
                "result": {"vx": command[0], "vy": command[1]},
                "nominal": {"vx": nominal[0], "vy": nominal[1]},
            },
        }

    return {
        "config": config,
        "state": [
            {
                "frame_index": 0,
                "robots": [
                    robot(1, 3.0, (4.0, 0.0), (99.0, 99.0)),
                    robot(2, 5.0, (2.0, 0.0), (88.0, 88.0)),
                ],
            },
            {
                "frame_index": 1,
                "robots": [
                    robot(1, 5.0, (8.0, 0.0), (77.0, 77.0)),
                    robot(2, 6.0, (6.0, 0.0), (66.0, 66.0)),
                ],
            },
        ],
    }


class ReplayHarness(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name).resolve()
        self.input_root = self.root / "input-bundle"
        self.protocol_root = self.root / "protocol-bundle"
        self.output_parent = self.root / "outputs"
        self.input_root.mkdir()
        self.protocol_root.mkdir()
        self.data_path = self.input_root / "data.json"
        self.input_manifest_path = self.input_root / "manifest.json"
        self.baseline_path = self.input_root / "baseline.jsonl.gz"
        self.protocol_path = self.protocol_root / "protocol.json"
        write_json(self.data_path, tiny_data())
        write_json(self.input_manifest_path, {"termination_reason": "completed"})
        self.baseline_marker = b"BASELINE_ROWS_MUST_NOT_BE_COPIED\n"
        self.baseline_path.write_bytes(self.baseline_marker)
        self.default_output = self.output_parent / "bundle"
        self.write_protocol(self.default_output, (11, 12), 2)

    def tearDown(self):
        self.temporary.cleanup()

    def source_entries(self) -> dict:
        project = Path(replay.__file__).resolve().parents[2]
        paths = {
            "truth_data": self.data_path,
            "input_manifest": self.input_manifest_path,
            "replay_source": Path(replay.__file__).resolve(),
            "estimator_source": project / "scripts/diagnostics/predictive_wnls.py",
            "legacy_solver_source": project / "scripts/diagnostics/replay_localization_calibration.py",
            "diagnostic_integrity_source": project / "scripts/diagnostics/run_diagnostic.py",
            "baseline_process": self.baseline_path,
        }
        return {
            name: {"path": str(path), "sha256": sha256(path)}
            for name, path in paths.items()
        }

    def protocol(self, output_root: Path, seeds: tuple[int, ...], max_frames: int | None):
        command = [
            "conda",
            "run",
            "-n",
            "cbf_env",
            "python",
            "scripts/diagnostics/replay_predictive_wnls_recovery.py",
            "--data-path",
            str(self.data_path),
            "--input-manifest-path",
            str(self.input_manifest_path),
            "--protocol-json",
            str(self.protocol_path),
            "--output-root",
            str(output_root),
            "--run-seeds",
            ",".join(str(seed) for seed in seeds),
        ]
        if max_frames is not None:
            command.extend(["--max-frames", str(max_frames)])
        return {
            "schema_id": "cbf2026-predictive-wnls-stage1-hermetic-protocol-v1",
            "protocol_id": "cbf2026-predictive-wnls-stage1-hermetic-v1",
            "implementation_parent_commit": "0" * 40,
            "binding_design": copy.deepcopy(replay.BINDING_DESIGN),
            "sources": self.source_entries(),
            "experiment": {
                **copy.deepcopy(replay.EXPERIMENT_CONTRACT),
                "evidence_class": "hermetic_non_evidence_only",
                "range_noise_seeds": list(seeds),
                "max_frames": max_frames,
            },
            "estimator_constants": copy.deepcopy(replay.ESTIMATOR_CONSTANTS),
            "status_contract": copy.deepcopy(replay.STATUS_CONTRACT),
            "ablation_contracts": copy.deepcopy(replay.ABLATION_CONTRACTS),
            "raw_schema": copy.deepcopy(replay.RAW_SCHEMA_DECLARATION),
            "analysis_schema": copy.deepcopy(replay.ANALYSIS_SCHEMA),
            "gates": copy.deepcopy(replay.GATES),
            "disk_contract": copy.deepcopy(replay.DISK_CONTRACT),
            "invocations": {
                "hermetic_replay": {
                    "kind": "hermetic_non_evidence",
                    "output_root": str(output_root),
                    "range_noise_seeds": list(seeds),
                    "max_frames": max_frames,
                }
            },
            "evidence_lifecycle": copy.deepcopy(replay.EVIDENCE_LIFECYCLE),
            "commands": {"hermetic_replay": command},
        }

    def write_protocol(
        self,
        output_root: Path,
        seeds: tuple[int, ...] = (11, 12),
        max_frames: int | None = 2,
        mutate=None,
    ) -> None:
        value = self.protocol(output_root, seeds, max_frames)
        if mutate is not None:
            mutate(value)
        write_json(self.protocol_path, value)

    def execute(
        self,
        output_root: Path | None = None,
        seeds: tuple[int, ...] = (11, 12),
        max_frames: int | None = 2,
    ):
        return replay.replay_predictive_recovery(
            data_path=self.data_path,
            input_manifest_path=self.input_manifest_path,
            protocol_path=self.protocol_path,
            output_root=output_root or self.default_output,
            run_seeds=seeds,
            max_frames=max_frames,
        )

    def rows(self, output_root: Path | None = None):
        path = (output_root or self.default_output) / replay.RAW_PROCESS_NAME
        with gzip.open(path, "rt") as source:
            return [json.loads(line) for line in source]


class ProtocolAndPreflightTests(ReplayHarness):
    def test_direct_path_help_runs_from_repository_root(self):
        project = Path(replay.__file__).resolve().parents[2]
        result = subprocess.run(
            [
                sys.executable,
                "scripts/diagnostics/replay_predictive_wnls_recovery.py",
                "--help",
            ],
            cwd=project,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("usage:", result.stdout)

    def test_direct_path_help_rejects_preceding_shadow_scripts_package(self):
        project = Path(replay.__file__).resolve().parents[2]
        shadow_root = self.root / "shadow"
        shadow_scripts = shadow_root / "scripts"
        shadow_scripts.mkdir(parents=True)
        marker = self.root / "shadow-imported"
        (shadow_scripts / "__init__.py").write_text(
            "from pathlib import Path\n"
            f"Path({str(marker)!r}).write_text('imported')\n"
        )
        environment = os.environ.copy()
        python_path = [str(shadow_root)]
        if environment.get("PYTHONPATH"):
            python_path.append(environment["PYTHONPATH"])
        environment["PYTHONPATH"] = os.pathsep.join(python_path)
        result = subprocess.run(
            [
                sys.executable,
                "scripts/diagnostics/replay_predictive_wnls_recovery.py",
                "--help",
            ],
            cwd=project,
            env=environment,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(marker.exists())
        self.assertIn("usage:", result.stdout)

    def test_production_v3_contract_is_exact_and_exports_canonical_argv(self):
        self.assertEqual(
            replay.PROTOCOL_SCHEMA_ID,
            "cbf2026-predictive-wnls-stage1-protocol-v3",
        )
        self.assertEqual(
            replay.PROTOCOL_ID,
            "cbf2026-predictive-wnls-stage1-v3",
        )
        self.assertEqual(
            replay.PRODUCTION_PROTOCOL_TOKEN,
            (
                "docs/diagnostics/"
                "2026-07-30-predictive-wnls-stage1-protocol-v3.json"
            ),
        )
        self.assertEqual(
            replay.RAW_SCHEMA_ID,
            "cbf2026-predictive-wnls-development-rows-v2",
        )
        self.assertEqual(
            replay.ANALYSIS_SCHEMA["id"],
            "cbf2026-predictive-wnls-development-analysis-v2",
        )
        self.assertEqual(
            replay.HERMETIC_PROTOCOL_SCHEMA_ID,
            "cbf2026-predictive-wnls-stage1-hermetic-protocol-v1",
        )
        self.assertEqual(
            replay.HERMETIC_PROTOCOL_ID,
            "cbf2026-predictive-wnls-stage1-hermetic-v1",
        )
        self.assertEqual(
            replay.PRODUCTION_RANGE_NOISE_SEEDS,
            tuple(range(20260727, 20260747)),
        )
        invocations = replay.production_invocation_contract()
        self.assertEqual(
            tuple(invocations),
            ("smoke_a", "smoke_b", "registered_replay", "registered_analyzer"),
        )
        self.assertEqual(
            invocations["smoke_a"],
            {
                "kind": "unregistered_smoke",
                "output_root": (
                    "/private/tmp/cbf2026-predictive-wnls-smoke-v3-a"
                ),
                "range_noise_seeds": [20260727],
                "max_frames": 2,
            },
        )
        self.assertEqual(
            invocations["smoke_b"],
            {
                "kind": "unregistered_smoke",
                "output_root": (
                    "/private/tmp/cbf2026-predictive-wnls-smoke-v3-b"
                ),
                "range_noise_seeds": [20260727],
                "max_frames": 2,
            },
        )
        self.assertEqual(
            invocations["registered_replay"]["output_root"],
            (
                "/private/tmp/cbf2026-predictive-wnls-development/"
                "stage1-v3"
            ),
        )
        self.assertEqual(
            invocations["registered_replay"]["range_noise_seeds"],
            list(range(20260727, 20260747)),
        )
        self.assertIsNone(invocations["registered_replay"]["max_frames"])
        self.assertEqual(
            invocations["registered_analyzer"]["development_manifest_path"],
            (
                "/private/tmp/cbf2026-predictive-wnls-development/"
                "stage1-v3/manifest.json"
            ),
        )
        self.assertEqual(
            invocations["registered_analyzer"]["output_root"],
            (
                "/private/tmp/"
                "cbf2026-predictive-wnls-development-analysis/stage1-v3"
            ),
        )
        production_declarations = json.dumps(
            {
                "schema_id": replay.PROTOCOL_SCHEMA_ID,
                "protocol_id": replay.PROTOCOL_ID,
                "protocol_token": replay.PRODUCTION_PROTOCOL_TOKEN,
                "invocations": invocations,
                "commands": replay.production_command_contract(
                    {
                        "truth_data": {"path": "/input/data.json"},
                        "input_manifest": {
                            "path": "/input/manifest.json"
                        },
                        "baseline_process": {
                            "path": "/input/baseline.jsonl.gz"
                        },
                    }
                ),
            },
            sort_keys=True,
        )
        for retired in (
            "docs/diagnostics/2026-07-30-predictive-wnls-stage1-protocol.json",
            "/private/tmp/cbf2026-predictive-wnls-smoke-a",
            "/private/tmp/cbf2026-predictive-wnls-smoke-b",
            "/private/tmp/cbf2026-predictive-wnls-development/stage1-v2",
            (
                "/private/tmp/"
                "cbf2026-predictive-wnls-development-analysis/stage1-v2"
            ),
        ):
            with self.subTest(retired=retired):
                self.assertNotIn(retired, production_declarations)
        command = replay.canonical_replay_argv(
            data_path=Path("/input/data.json"),
            input_manifest_path=Path("/input/manifest.json"),
            protocol_token="docs/protocol.json",
            output_root=Path("/output"),
            run_seeds=(20260727,),
            max_frames=2,
        )
        self.assertEqual(
            command,
            [
                "conda", "run", "-n", "cbf_env", "python",
                "scripts/diagnostics/replay_predictive_wnls_recovery.py",
                "--data-path", "/input/data.json",
                "--input-manifest-path", "/input/manifest.json",
                "--protocol-json", "docs/protocol.json",
                "--output-root", "/output",
                "--run-seeds", "20260727",
                "--max-frames", "2",
            ],
        )

    def test_production_protocol_rejects_cohort_invocation_argv_and_analyzer_drift(self):
        project = Path(replay.__file__).resolve().parents[2]
        sources = {
            "truth_data": {
                "path": replay.PRODUCTION_TRUTH_DATA_PATH,
                "sha256": replay.PRODUCTION_TRUTH_DATA_SHA256,
            },
            "input_manifest": {
                "path": replay.PRODUCTION_INPUT_MANIFEST_PATH,
                "sha256": replay.PRODUCTION_INPUT_MANIFEST_SHA256,
            },
            "baseline_process": {
                "path": replay.PRODUCTION_BASELINE_PROCESS_PATH,
                "sha256": replay.PRODUCTION_BASELINE_PROCESS_SHA256,
            },
            "replay_source": {
                "path": str(Path(replay.__file__).resolve()),
                "sha256": "1" * 64,
            },
            "estimator_source": {
                "path": str(project / "scripts/diagnostics/predictive_wnls.py"),
                "sha256": "2" * 64,
            },
            "analyzer_source": {
                "path": str(
                    project
                    / "scripts/diagnostics/analyze_predictive_wnls_recovery.py"
                ),
                "sha256": "3" * 64,
            },
            "legacy_solver_source": {
                "path": str(
                    project
                    / "scripts/diagnostics/replay_localization_calibration.py"
                ),
                "sha256": replay.LEGACY_SOLVER_SHA256,
            },
            "diagnostic_integrity_source": {
                "path": str(project / "scripts/diagnostics/run_diagnostic.py"),
                "sha256": "4" * 64,
            },
        }
        protocol = {
            "schema_id": replay.PROTOCOL_SCHEMA_ID,
            "protocol_id": replay.PROTOCOL_ID,
            "implementation_parent_commit": "0" * 40,
            "binding_design": copy.deepcopy(replay.BINDING_DESIGN),
            "sources": sources,
            "experiment": {
                **copy.deepcopy(replay.EXPERIMENT_CONTRACT),
                "range_noise_seeds": list(range(20260727, 20260747)),
                "max_frames": None,
            },
            "estimator_constants": copy.deepcopy(replay.ESTIMATOR_CONSTANTS),
            "status_contract": copy.deepcopy(replay.STATUS_CONTRACT),
            "ablation_contracts": copy.deepcopy(replay.ABLATION_CONTRACTS),
            "raw_schema": copy.deepcopy(replay.RAW_SCHEMA_DECLARATION),
            "analysis_schema": copy.deepcopy(replay.ANALYSIS_SCHEMA),
            "gates": copy.deepcopy(replay.GATES),
            "disk_contract": copy.deepcopy(replay.DISK_CONTRACT),
            "invocations": replay.production_invocation_contract(),
            "evidence_lifecycle": copy.deepcopy(replay.EVIDENCE_LIFECYCLE),
            "commands": replay.production_command_contract(sources),
        }
        protocol_path = project / replay.PRODUCTION_PROTOCOL_TOKEN
        selected = replay._validate_protocol_shape(
            protocol,
            protocol_path=protocol_path,
            output_root=Path(
                "/private/tmp/cbf2026-predictive-wnls-smoke-v3-a"
            ),
            run_seeds=(20260727,),
            max_frames=2,
        )
        self.assertEqual(selected, "smoke_a")
        mutations = {
            "cohort": lambda p: p["experiment"]["range_noise_seeds"].pop(),
            "frames": lambda p: p["experiment"].update(max_frames=2),
            "name": lambda p: p["invocations"].update(
                renamed=p["invocations"].pop("smoke_b")
            ),
            "count": lambda p: p["invocations"].pop("registered_analyzer"),
            "argv": lambda p: p["commands"]["smoke_a"].__setitem__(0, "wrong"),
            "analyzer": lambda p: p["sources"]["analyzer_source"].update(
                path="/tmp/unrelated.py"
            ),
        }
        for name, mutation in mutations.items():
            with self.subTest(name=name):
                changed = copy.deepcopy(protocol)
                mutation(changed)
                with self.assertRaises(ValueError):
                    replay._validate_protocol_shape(
                        changed,
                        protocol_path=protocol_path,
                        output_root=Path(
                            "/private/tmp/"
                            "cbf2026-predictive-wnls-smoke-v3-a"
                        ),
                        run_seeds=(20260727,),
                        max_frames=2,
                    )

    def test_git_parent_provenance_accepts_ancestor_blobs_and_rejects_false_hashes(self):
        project = Path(replay.__file__).resolve().parents[2]
        parent = subprocess.run(
            ["git", "-C", str(project), "rev-parse", "HEAD^"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        relative = "scripts/diagnostics/predictive_wnls.py"
        blob = subprocess.run(
            ["git", "-C", str(project), "show", f"{parent}:{relative}"],
            check=True,
            capture_output=True,
        ).stdout
        sources = {
            "estimator_source": {
                "path": str(project / relative),
                "sha256": hashlib.sha256(blob).hexdigest(),
            }
        }
        observed = replay.verify_implementation_parent_provenance(
            project_root=project,
            implementation_parent_commit=parent,
            sources=sources,
            source_names=("estimator_source",),
        )
        self.assertEqual(observed["commit"], parent)
        self.assertEqual(observed["source_names"], ["estimator_source"])

        false_sources = copy.deepcopy(sources)
        false_sources["estimator_source"]["sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "Git blob hash"):
            replay.verify_implementation_parent_provenance(
                project_root=project,
                implementation_parent_commit=parent,
                sources=false_sources,
                source_names=("estimator_source",),
            )
        with self.assertRaisesRegex(ValueError, "Git commit"):
            replay.verify_implementation_parent_provenance(
                project_root=project,
                implementation_parent_commit="0" * 40,
                sources=sources,
                source_names=("estimator_source",),
            )

    def test_protocol_is_authority_for_every_bound_contract(self):
        mutations = {
            "schema": lambda p: p.update(schema_id="wrong"),
            "raw": lambda p: p["raw_schema"].update(id="wrong"),
            "variants": lambda p: p["experiment"]["variants"].reverse(),
            "runtime": lambda p: p["experiment"].update(frame_dt_seconds=1.0),
            "gate": lambda p: p["estimator_constants"].update(innovation_q_max=1.0),
            "disk": lambda p: p["disk_contract"].update(launch_minimum_free_bytes=1),
            "retry": lambda p: p["evidence_lifecycle"].update(registered_retry_allowed=True),
            "paper": lambda p: p["evidence_lifecycle"].update(paper_gate="OPEN"),
            "baseline": lambda p: p["sources"].pop("baseline_process"),
            "legacy": lambda p: p["sources"].pop("legacy_solver_source"),
            "integrity": lambda p: p["sources"].pop("diagnostic_integrity_source"),
            "command": lambda p: p["commands"].update(hermetic_replay=[]),
            "argv": lambda p: p["commands"]["hermetic_replay"].__setitem__(
                0, "wrong"
            ),
            "extra_invocation": lambda p: p["invocations"].update(extra={}),
        }
        for name, mutation in mutations.items():
            with self.subTest(name=name):
                target = self.output_parent / name
                self.write_protocol(target, mutate=mutation)
                with self.assertRaises(ValueError):
                    self.execute(target)
                self.assertFalse(target.exists())

        target = self.output_parent / "undeclared"
        self.write_protocol(self.default_output)
        with self.assertRaises(ValueError):
            self.execute(target)
        self.assertFalse(target.exists())
        with self.assertRaises(ValueError):
            self.execute(self.default_output, seeds=(11,))
        self.assertFalse(self.default_output.exists())

    def test_strict_json_rejects_nonfinite_numpy_values_and_raw_failure_is_terminal(self):
        for value in (math.nan, math.inf, -math.inf, np.float64(math.nan)):
            with self.subTest(value=value), self.assertRaises(ValueError):
                replay._strict_json_bytes({"value": value})
        with self.assertRaises(TypeError):
            replay._strict_json_bytes({"value": np.longdouble("1.25")})

        overflow = self.root / "overflow.json"
        overflow.write_text('{"value": 1e400}')
        with self.assertRaises(ValueError):
            replay._strict_load(overflow)

        original = replay._compact_candidates

        def corrupt(value):
            compact = original(value)
            return compact or [["bad", None, "invalid", None, None, math.nan,
                                False, "bad", None, [None] * len(replay.GATE_DIAGNOSTIC_FIELDS), []]]

        with mock.patch.object(replay, "_compact_candidates", side_effect=corrupt):
            with self.assertRaises(ValueError):
                self.execute()
        terminal = json.loads((self.default_output / replay.TERMINAL_MANIFEST_NAME).read_text())
        self.assertEqual(terminal["status"], "failed")
        self.assertEqual(terminal["error"]["type"], "ValueError")

    def test_preflight_rejects_frame_order_membership_status_and_nonfinite_command(self):
        mutations = {
            "robot_order": lambda d: d["state"][0]["robots"].reverse(),
            "membership": lambda d: d["state"][0]["robots"].pop(),
            "frame_link": lambda d: d["state"][1].update(frame_index=9),
            "opt_status": lambda d: d["state"][0]["robots"][0]["opt"].update(status="failed"),
            "missing_vx": lambda d: d["state"][0]["robots"][0]["opt"]["result"].pop("vx"),
            "nonfinite_vy": lambda d: d["state"][0]["robots"][0]["opt"]["result"].update(vy=math.nan),
        }
        for name, mutation in mutations.items():
            with self.subTest(name=name):
                data = tiny_data()
                mutation(data)
                # Nonfinite JSON is intentionally emitted directly so the input
                # parser/preflight, not this test helper, owns rejection.
                self.data_path.write_text(json.dumps(data))
                target = self.output_parent / name
                self.write_protocol(target)
                with self.assertRaises(ValueError):
                    self.execute(target)
                self.assertFalse(target.exists())
        self.assertEqual(replay._squad_local_index(8, 14, 2), 1)

    def test_order_and_predecessor_result_are_exact(self):
        manifest = self.execute()
        rows = self.rows()
        self.assertEqual(manifest["rows_written"], 24)
        self.assertEqual(
            [(r["variant"], r["seed"], r["frame_index"], r["robot_id"]) for r in rows],
            [(variant, seed, frame, robot)
             for variant in replay.DEVELOPMENT_VARIANTS
             for seed in (11, 12) for frame in (0, 1) for robot in (1, 2)],
        )
        row = next(r for r in rows if r["variant"] == "prediction_expiry"
                   and r["seed"] == 11 and r["frame_index"] == 1 and r["robot_id"] == 1)
        self.assertEqual(row["applied_command_source_frame"], 0)
        self.assertEqual(row["applied_command"], [4.0, 0.0])
        self.assertNotEqual(row["applied_command"], [99.0, 99.0])


class EvidenceAndAblationTests(ReplayHarness):
    def test_truth_stops_at_sensor_boundary_and_noise_seed_is_stable(self):
        calls = []
        original_qualify = replay.qualify_active_references
        original_multi = replay.solve_predictive_multistart

        def qualify(**kwargs):
            calls.append(kwargs)
            self.assertNotIn("truth", kwargs)
            self.assertNotIn("true_range", repr(kwargs))
            return original_qualify(**kwargs)

        def multi(**kwargs):
            self.assertNotIn("truth", kwargs)
            self.assertNotIn("true_range", repr(kwargs))
            return original_multi(**kwargs)

        with mock.patch.object(replay, "qualify_active_references", side_effect=qualify), \
             mock.patch.object(replay, "solve_predictive_multistart", side_effect=multi):
            self.execute()
        self.assertTrue(calls)
        first = calls[0]["measurement_records"]
        key = sorted(first)[0]
        record = first[key]
        self.assertEqual(set(record), {"present", "noisy_range"})
        first_row = self.rows()[0]
        decoded = [
            dict(zip(replay.REFERENCE_EVIDENCE_FIELDS, item))
            for item in first_row["reference_evidence"]
        ]
        evidence = next(
            item
            for item in decoded
            if (item["reference_kind"], item["reference_id"]) == key
        )
        self.assertEqual(
            evidence["noise_seed"],
            replay.stable_measurement_seed(11, 0, 1, key[0], key[1]),
        )

    def test_qualification_and_expiry_are_distinguished_by_real_unavailable_state(self):
        failed = {
            "status": "failed", "estimate": None, "covariance": None,
            "epsilon": None, "phi_min_eigenvalue": None, "phi_condition": None,
            "iterations": 50, "cost": 1.0, "stationarity_norm": 1.0,
            "failure_reason": "maximum WNLS iterations exceeded",
        }
        with mock.patch.object(replay, "solve_wnls", return_value=failed):
            self.execute()
        rows = self.rows()
        expiry = next(r for r in rows if r["variant"] == "prediction_expiry"
                      and r["seed"] == 11 and r["frame_index"] == 0 and r["robot_id"] == 2)
        qualified = next(r for r in rows if r["variant"] == "fresh_reference_qualification"
                         and r["seed"] == 11 and r["frame_index"] == 0 and r["robot_id"] == 2)
        expiry_refs = [dict(zip(replay.REFERENCE_EVIDENCE_FIELDS, item))
                       for item in expiry["reference_evidence"]]
        qualified_refs = [dict(zip(replay.REFERENCE_EVIDENCE_FIELDS, item))
                          for item in qualified["reference_evidence"]]
        expiry_uav = next(item for item in expiry_refs if item["reference_kind"] == "uav")
        qualified_uav = next(item for item in qualified_refs if item["reference_kind"] == "uav")
        self.assertFalse(expiry_uav["used"])
        self.assertFalse(expiry_uav["eligible"])
        self.assertEqual(
            expiry_uav["exclusion_reason"],
            "not_supplied_due_to_reference_state_unavailable",
        )
        self.assertTrue(expiry["reference_violations"])
        self.assertFalse(qualified_uav["used"])
        self.assertEqual(qualified_uav["exclusion_reason"], "not_current_frame_fresh")
        self.assertFalse(qualified["reference_violations"])

    def test_early_variants_use_legacy_single_start_and_multistart_is_unique(self):
        legacy = mock.Mock(wraps=replay.solve_wnls)
        multi = mock.Mock(wraps=replay.solve_predictive_multistart)
        with mock.patch.object(replay, "solve_wnls", legacy), \
             mock.patch.object(replay, "solve_predictive_multistart", multi):
            self.execute()
        self.assertGreater(legacy.call_count, 0)
        self.assertGreater(multi.call_count, 0)
        rows = self.rows()
        for row in rows:
            sources = [
                dict(zip(replay.CANDIDATE_FIELDS, candidate))["source"]
                for candidate in row["candidates"]
            ]
            if row["variant"] != "predictive_multistart":
                self.assertLessEqual(len(sources), 1)
            elif sources:
                self.assertNotIn("deployment", sources)

        for row in rows:
            if row["variant"] != "predictive_multistart":
                self.assertNotIn(
                    row["legacy_initial_estimate_source"],
                    {"prediction", "private_reacquisition_seed"},
                )

    def test_private_state_attempt_provenance_and_complete_reference_schema(self):
        self.execute()
        manifest = json.loads((self.default_output / replay.TERMINAL_MANIFEST_NAME).read_text())
        self.assertEqual(manifest["raw_schema"], replay.RAW_SCHEMA_DECLARATION)
        for declaration in (
            "reference_freshness_fields",
            "mandatory_reference_fields",
            "reference_key_fields",
            "exclusion_fields",
            "violation_fields",
            "private_state_fields",
            "public_state_fields",
        ):
            self.assertIn(declaration, manifest["raw_schema"])
        for row in self.rows():
            self.assertIn("attempt_base_anchor_provenance", row)
            self.assertIn("base_anchor_provenance", row)
            self.assertIn("private_reacquisition_seed", row)
            self.assertIn("reference_freshness", row)
            for freshness in row["reference_freshness"]:
                self.assertEqual(
                    len(freshness),
                    len(replay.REFERENCE_FRESHNESS_FIELDS),
                )
            self.assertEqual(
                tuple(row["mandatory_references"]),
                tuple(replay.MANDATORY_REFERENCE_FIELDS),
            )
            if row["private_reacquisition_seed"] is not None:
                self.assertEqual(
                    tuple(row["private_reacquisition_seed"]),
                    tuple(replay.PRIVATE_STATE_FIELDS),
                )
            for key in row["optional_candidates"] + row["active_references"]:
                self.assertEqual(len(key), len(replay.REFERENCE_KEY_FIELDS))
            for excluded in row["excluded_references"]:
                self.assertEqual(len(excluded), len(replay.EXCLUSION_FIELDS))
            for violation in row["reference_violations"]:
                self.assertEqual(len(violation), len(replay.VIOLATION_FIELDS))
            for evidence in row["reference_evidence"]:
                self.assertIsInstance(evidence, list)
                self.assertEqual(len(evidence), len(replay.REFERENCE_EVIDENCE_FIELDS))
            for candidate in row["candidates"]:
                self.assertEqual(len(candidate), len(replay.CANDIDATE_FIELDS))
                decoded = dict(zip(replay.CANDIDATE_FIELDS, candidate))
                self.assertEqual(
                    len(decoded["gate_diagnostics"]),
                    len(replay.GATE_DIAGNOSTIC_FIELDS),
                )
                for proposal in decoded["proposal_trace"]:
                    self.assertEqual(len(proposal), len(replay.PROPOSAL_TRACE_FIELDS))
        fresh = next((r for r in self.rows() if r["output_status"] == "fresh"), None)
        self.assertIsNotNone(fresh)
        self.assertEqual(
            fresh["private_reacquisition_seed"]["estimate"],
            fresh["estimate"],
        )
        nonfresh_with_attempt_roots = next(
            (
                row
                for row in self.rows()
                if row["output_status"] != "fresh"
                and row["attempt_base_anchor_provenance"]
            ),
            None,
        )
        self.assertIsNotNone(nonfresh_with_attempt_roots)
        self.assertEqual(nonfresh_with_attempt_roots["base_anchor_provenance"], [])

    def test_partial_reference_failure_marks_no_reference_used_and_all_reasons(self):
        mandatory = {"base_ids": [0], "uav_ids": []}
        optional = [("base", 1), ("base", 2)]
        runtime_records = {
            ("base", 0): {"present": False, "noisy_range": None},
            ("base", 1): {"present": True, "noisy_range": 5.0},
            ("base", 2): {"present": True, "noisy_range": -1.0},
        }
        qualification = estimator.qualify_active_references(
            mandatory=mandatory,
            optional_keys=optional,
            measurement_records=runtime_records,
            uav_outputs={},
            variant="fresh_reference_qualification",
        )
        evidence = replay._reference_evidence(
            mandatory=mandatory,
            optional=optional,
            records=runtime_records,
            noise_seeds={("base", 0): 10, ("base", 1): 11, ("base", 2): 12},
            qualification=qualification,
            current_public={},
            solver_used_keys=(),
        )
        decoded = [
            dict(zip(replay.REFERENCE_EVIDENCE_FIELDS, item))
            for item in evidence
        ]
        self.assertTrue(all(item["used"] is False for item in decoded))
        base_one = next(item for item in decoded if item["reference_id"] == 1)
        self.assertTrue(base_one["eligible"])
        self.assertEqual(
            base_one["exclusion_reason"],
            "not_evaluated_due_to_missing_mandatory",
        )
        base_two = next(item for item in decoded if item["reference_id"] == 2)
        self.assertTrue(base_two["measurement_present"])
        self.assertFalse(base_two["eligible"])
        self.assertEqual(
            base_two["exclusion_reason"],
            "measurement_invalid_or_absent",
        )

    def test_invalid_mandatory_measurement_uses_measurement_reason(self):
        mandatory = {"base_ids": [0], "uav_ids": []}
        records = {("base", 0): {"present": True, "noisy_range": -1.0}}
        qualification = estimator.qualify_active_references(
            mandatory=mandatory,
            optional_keys=[],
            measurement_records=records,
            uav_outputs={},
            variant="fresh_reference_qualification",
        )
        evidence = replay._reference_evidence(
            mandatory=mandatory,
            optional=[],
            records=records,
            noise_seeds={("base", 0): 10},
            qualification=qualification,
            current_public={},
            solver_used_keys=(),
        )
        decoded = dict(zip(replay.REFERENCE_EVIDENCE_FIELDS, evidence[0]))
        self.assertTrue(decoded["measurement_present"])
        self.assertFalse(decoded["eligible"])
        self.assertEqual(decoded["current_freshness"], "fresh")
        self.assertEqual(
            decoded["exclusion_reason"],
            "measurement_invalid_or_absent",
        )

    def test_compact_invalid_and_rejected_candidate_traces_are_complete(self):
        candidates = [
            {
                "source": "prediction",
                "initial_estimate": [1.0, 2.0],
                "status": "failed",
                "estimate": [1.0, 2.0],
                "covariance": None,
                "cost": 4.0,
                "accepted": False,
                "rejection_reason": "maximum_proposals_exhausted",
                "q_innov": None,
                "gate_diagnostics": {
                    "innovation_gate": "applied",
                    "q_innov": None,
                    "gate_outcome": "invalid",
                    "valid": False,
                    "failure_reason": "maximum_proposals_exhausted",
                    "reduced_whitened_cost": None,
                },
                "proposal_trace": [{
                    "proposal": 0,
                    "damping": 1e-3,
                    "cost": 4.0,
                    "stationarity_norm": 2.0,
                    "raw_step_norm": 1.0,
                    "trial_cost": None,
                    "invalid_trial_reason": "invalid_trial_terms",
                    "accepted": False,
                }],
            },
            {
                "source": "algebraic",
                "initial_estimate": [2.0, 3.0],
                "status": "converged",
                "estimate": [2.0, 3.0],
                "covariance": [[1.0, 0.0], [0.0, 1.0]],
                "cost": 1.0,
                "accepted": False,
                "rejection_reason": "innovation_q_exceeds_reference_quantile",
                "q_innov": 12.0,
                "gate_diagnostics": {
                    "innovation_gate": "applied",
                    "q_innov": 12.0,
                    "gate_outcome": "rejected",
                    "valid": True,
                    "failure_reason": None,
                    "reduced_whitened_cost": None,
                },
                "proposal_trace": [],
            },
        ]
        compact = replay._compact_candidates(candidates)
        self.assertEqual(len(compact), 2)
        for candidate in compact:
            decoded = dict(zip(replay.CANDIDATE_FIELDS, candidate))
            self.assertEqual(
                len(decoded["gate_diagnostics"]),
                len(replay.GATE_DIAGNOSTIC_FIELDS),
            )
        first = dict(zip(replay.CANDIDATE_FIELDS, compact[0]))
        self.assertEqual(
            dict(zip(replay.PROPOSAL_TRACE_FIELDS, first["proposal_trace"][0]))[
                "invalid_trial_reason"
            ],
            "invalid_trial_terms",
        )
        malformed = (
            [None],
            [{"source": "prediction"}],
            [{**candidates[0], "unexpected": True}],
            [{**candidates[0], "gate_diagnostics": {}}],
            [{**candidates[0], "proposal_trace": [None]}],
            [{**candidates[0], "proposal_trace": [
                {**candidates[0]["proposal_trace"][0], "unexpected": True}
            ]}],
        )
        for value in malformed:
            with self.subTest(value=value), self.assertRaises(ValueError):
                replay._compact_candidates(value)

    def test_offline_fresh_aged_and_unavailable_null_semantics(self):
        fresh = {
            "output_status": "fresh", "estimate": [3.0, 4.0],
            "modeled_covariance": [[1.0, 0.0], [0.0, 4.0]], "epsilon": 6.0,
        }
        predicted = {
            "output_status": "predicted", "estimate": [3.0, 4.0],
            "modeled_covariance": [[1.0, 0.0], [0.0, 4.0]],
            "aged_modeled_radius": 6.0,
        }
        unavailable = {"output_status": "unavailable", "estimate": None}
        fresh_metrics = replay._offline_metrics(fresh, np.array([4.0, 6.0]))
        aged_metrics = replay._offline_metrics(predicted, np.array([4.0, 6.0]))
        null_metrics = replay._offline_metrics(unavailable, np.array([4.0, 6.0]))
        self.assertEqual(fresh_metrics["offline_error_norm"], math.sqrt(5.0))
        self.assertTrue(fresh_metrics["offline_fresh_containment"])
        self.assertEqual(fresh_metrics["offline_fresh_q_error"], 2.0)
        self.assertIsNone(fresh_metrics["offline_aged_q_error"])
        self.assertTrue(aged_metrics["offline_aged_radius_containment"])
        self.assertEqual(aged_metrics["offline_aged_q_error"], 2.0)
        self.assertIsNone(aged_metrics["offline_fresh_q_error"])
        self.assertTrue(all(value is None for key, value in null_metrics.items()
                            if key != "offline_truth_position"))


class PathDiskAndTerminalTests(ReplayHarness):
    def test_symlink_ancestors_protocol_overlap_and_source_symlinks_fail_preallocation(self):
        real = self.root / "real-parent"
        real.mkdir()
        linked = self.root / "linked-parent"
        linked.symlink_to(real, target_is_directory=True)
        target = linked / "bundle"
        self.write_protocol(target)
        with self.assertRaises(ValueError):
            self.execute(target)
        self.assertFalse((real / "bundle").exists())

        overlap = self.protocol_root / "bundle"
        self.write_protocol(overlap)
        with self.assertRaises(ValueError):
            self.execute(overlap)
        self.assertFalse(overlap.exists())

        alias = self.input_root / "data-alias.json"
        alias.symlink_to(self.data_path)
        target = self.output_parent / "source-alias"
        self.write_protocol(target)
        protocol = json.loads(self.protocol_path.read_text())
        protocol["sources"]["truth_data"]["path"] = str(alias)
        write_json(self.protocol_path, protocol)
        with self.assertRaises(ValueError):
            self.execute(target)
        self.assertFalse(target.exists())

    def test_start_floor_and_cap_failures_publish_only_when_allocated(self):
        with mock.patch.object(replay, "require_start_space", side_effect=DiskSpaceError("start")):
            with self.assertRaises(DiskSpaceError):
                self.execute()
        self.assertFalse(self.default_output.exists())

        target = self.output_parent / "floor"
        self.write_protocol(target)
        with mock.patch.object(
            replay,
            "_available_bytes_fd",
            return_value=replay.HARD_FLOOR_BYTES - 1,
        ):
            with self.assertRaises(DiskSpaceError):
                self.execute(target)
        self.assertEqual(json.loads((target / replay.TERMINAL_MANIFEST_NAME).read_text())["status"], "failed")

        target = self.output_parent / "cap"
        self.write_protocol(target)
        with mock.patch.object(
            replay,
            "_allocated_bytes_fd",
            return_value=replay.RAW_BUNDLE_CAP_BYTES + 1,
        ):
            with self.assertRaises(DiskSpaceError):
                self.execute(target)
        self.assertEqual(json.loads((target / replay.TERMINAL_MANIFEST_NAME).read_text())["status"], "failed")

    def test_preexisting_target_and_staging_files_fail_without_overwrite(self):
        self.default_output.mkdir(parents=True)
        for content in (None, b"x"):
            if content is not None:
                (self.default_output / "keep").write_bytes(content)
            with self.assertRaises(FileExistsError):
                self.execute()

        for stage_name in replay.STAGING_NAMES:
            target = self.output_parent / stage_name
            self.write_protocol(target)
            stage = replay._stage_path(target, "finalizing")
            stage.parent.mkdir(exist_ok=True)
            stage.write_bytes(b"occupied")
            with self.assertRaises(FileExistsError):
                self.execute(target)
            self.assertEqual(stage.read_bytes(), b"occupied")
            self.assertFalse(target.exists())

    def test_finalizing_probe_link_and_cleanup_faults_never_leave_completed_or_staging(self):
        fault_points = {
            "finalizing_write": ("_write_stage", RuntimeError("write")),
            "probe": ("_final_identity_and_disk_probe", RuntimeError("probe")),
            "link": ("_link_stage", RuntimeError("link")),
            "cleanup": ("_cleanup_stage", RuntimeError("cleanup")),
        }
        for name, (attribute, error) in fault_points.items():
            with self.subTest(name=name):
                target = self.output_parent / name
                self.write_protocol(target)
                original = getattr(replay, attribute)

                calls = 0

                def inject_once(*args, **kwargs):
                    nonlocal calls
                    calls += 1
                    if calls == 1:
                        raise error
                    return original(*args, **kwargs)

                with mock.patch.object(replay, attribute, side_effect=inject_once):
                    with self.assertRaises(RuntimeError):
                        self.execute(target)
                terminal = json.loads((target / replay.TERMINAL_MANIFEST_NAME).read_text())
                self.assertEqual(terminal["status"], "failed")
                self.assertFalse(any(target.parent.glob(f".{target.name}.manifest.*")))
                setattr(replay, attribute, original)

    def test_real_link_then_exception_is_reconciled_as_committed(self):
        original = replay._link_stage

        for error_type in (RuntimeError, KeyboardInterrupt, SystemExit):
            with self.subTest(error_type=error_type):
                target_root = self.output_parent / f"linked-{error_type.__name__}"
                self.write_protocol(target_root)

                def link_then_raise(transaction, stage):
                    original(transaction, stage)
                    raise error_type("delivered after link")

                with mock.patch.object(
                    replay, "_link_stage", side_effect=link_then_raise
                ):
                    manifest = self.execute(target_root)
                self.assertEqual(manifest["status"], "completed")
                terminal = json.loads(
                    (target_root / replay.TERMINAL_MANIFEST_NAME).read_text()
                )
                self.assertEqual(terminal["status"], "completed")

    def test_completed_stage_cleanup_failure_rolls_back_before_commit(self):
        original = replay._cleanup_stage
        calls = 0

        def fail_completed_cleanup(transaction, stage):
            nonlocal calls
            calls += 1
            if calls == 2:
                raise RuntimeError("completed cleanup")
            return original(transaction, stage)

        with mock.patch.object(
            replay, "_cleanup_stage", side_effect=fail_completed_cleanup
        ):
            with self.assertRaisesRegex(RuntimeError, "completed cleanup"):
                self.execute()
        terminal = json.loads(
            (self.default_output / replay.TERMINAL_MANIFEST_NAME).read_text()
        )
        self.assertEqual(terminal["status"], "failed")
        self.assertFalse(any(
            self.output_parent.glob(f".{self.default_output.name}.manifest.*")
        ))

    def test_success_probe_interrupts_publish_failed_and_are_rethrown(self):
        for error_type in (KeyboardInterrupt, SystemExit):
            with self.subTest(error_type=error_type):
                target = self.output_parent / error_type.__name__
                self.write_protocol(target)
                with mock.patch.object(
                    replay,
                    "_required_terminal_metrics",
                    side_effect=error_type("probe"),
                    create=True,
                ):
                    with self.assertRaises(error_type):
                        self.execute(target)
                terminal = json.loads(
                    (target / replay.TERMINAL_MANIFEST_NAME).read_text()
                )
                self.assertEqual(terminal["status"], "failed")

    def test_precommit_fsync_failures_roll_back_owned_target_and_publish_failed(self):
        fault_points = ("_fsync_raw_output", "_fsync_output_directory")
        for attribute in fault_points:
            with self.subTest(attribute=attribute):
                target = self.output_parent / attribute
                self.write_protocol(target)
                with mock.patch.object(
                    replay,
                    attribute,
                    side_effect=OSError("fsync"),
                    create=True,
                ):
                    with self.assertRaises(OSError):
                        self.execute(target)
                terminal = json.loads(
                    (target / replay.TERMINAL_MANIFEST_NAME).read_text()
                )
                self.assertEqual(terminal["status"], "failed")

    def test_foreign_sibling_stage_survives_failure(self):
        foreign = replay._stage_path(self.default_output, "completed")
        original = replay._write_row
        injected = False

        def create_foreign_then_fail(*args):
            nonlocal injected
            if not injected:
                injected = True
                foreign.write_bytes(b"foreign")
                raise RuntimeError("row")
            return original(*args)

        with mock.patch.object(
            replay, "_write_row", side_effect=create_foreign_then_fail
        ):
            with self.assertRaises(RuntimeError):
                self.execute()
        self.assertEqual(foreign.read_bytes(), b"foreign")
        terminal = json.loads(
            (self.default_output / replay.TERMINAL_MANIFEST_NAME).read_text()
        )
        self.assertEqual(terminal["status"], "failed")

    def test_failed_publication_ordinary_error_does_not_mask_original(self):
        with mock.patch.object(
            replay, "_write_row", side_effect=RuntimeError("original")
        ), mock.patch.object(
            replay, "_publish_failed", side_effect=OSError("terminal")
        ):
            with self.assertRaisesRegex(RuntimeError, "original") as caught:
                self.execute()
        notes = getattr(caught.exception, "__notes__", [])
        self.assertTrue(
            any("terminal" in note and "publication" in note for note in notes)
        )
        emergency = list(
            self.default_output.glob("manifest.failed.emergency.*.json")
        )
        self.assertEqual(len(emergency), 1)
        record = json.loads(emergency[0].read_text())
        self.assertEqual(record["status"], "failed")
        self.assertEqual(record["terminal_publication_error"]["type"], "OSError")

    def test_exception_immediately_after_root_creation_is_terminal(self):
        original = replay._create_exact_root

        def create_then_raise(output_root, *, identity_sink=None):
            original(output_root, identity_sink=identity_sink)
            raise RuntimeError("after allocation")

        with mock.patch.object(
            replay, "_create_exact_root", side_effect=create_then_raise
        ):
            with self.assertRaises(RuntimeError):
                self.execute()
        terminal = json.loads(
            (self.default_output / replay.TERMINAL_MANIFEST_NAME).read_text()
        )
        self.assertEqual(terminal["status"], "failed")

    def test_post_mkdir_leaf_open_failure_removes_unexported_root(self):
        real_mkdir = os.mkdir
        real_open = os.open
        leaf_created = False
        failed = False

        def track_leaf_mkdir(path, mode=0o777, *, dir_fd=None):
            nonlocal leaf_created
            result = real_mkdir(path, mode, dir_fd=dir_fd)
            if path == self.default_output.name and dir_fd is not None:
                leaf_created = True
            return result

        def fail_first_leaf_open(path, flags, mode=0o777, *, dir_fd=None):
            nonlocal failed
            if (
                leaf_created
                and not failed
                and path == self.default_output.name
                and dir_fd is not None
            ):
                failed = True
                raise OSError("post-mkdir leaf open")
            return real_open(path, flags, mode, dir_fd=dir_fd)

        with mock.patch("os.mkdir", side_effect=track_leaf_mkdir), mock.patch(
            "os.open", side_effect=fail_first_leaf_open
        ):
            with self.assertRaisesRegex(OSError, "post-mkdir leaf open"):
                self.execute()
        self.assertTrue(failed)
        self.assertFalse(self.default_output.exists())

    def test_post_mkdir_leaf_fstat_and_stat_failures_remove_unexported_root(self):
        for fault in ("fstat", "stat"):
            with self.subTest(fault=fault):
                target = self.output_parent / f"post-mkdir-{fault}"
                self.write_protocol(target)
                real_mkdir = os.mkdir
                real_open = os.open
                real_fstat = os.fstat
                real_stat = os.stat
                leaf_created = False
                leaf_descriptor = None
                failed = False

                def track_leaf_mkdir(path, mode=0o777, *, dir_fd=None):
                    nonlocal leaf_created
                    result = real_mkdir(path, mode, dir_fd=dir_fd)
                    if path == target.name and dir_fd is not None:
                        leaf_created = True
                    return result

                def track_leaf_open(path, flags, mode=0o777, *, dir_fd=None):
                    nonlocal leaf_descriptor
                    descriptor = real_open(path, flags, mode, dir_fd=dir_fd)
                    if leaf_created and path == target.name and dir_fd is not None:
                        leaf_descriptor = descriptor
                    return descriptor

                def fail_leaf_fstat(descriptor):
                    nonlocal failed
                    if (
                        fault == "fstat"
                        and descriptor == leaf_descriptor
                        and not failed
                    ):
                        failed = True
                        raise OSError("post-mkdir leaf fstat")
                    return real_fstat(descriptor)

                def fail_leaf_stat(
                    path,
                    *,
                    dir_fd=None,
                    follow_symlinks=True,
                ):
                    nonlocal failed
                    if (
                        fault == "stat"
                        and leaf_created
                        and path == target.name
                        and dir_fd is not None
                        and not failed
                    ):
                        failed = True
                        raise OSError("post-mkdir leaf stat")
                    return real_stat(
                        path,
                        dir_fd=dir_fd,
                        follow_symlinks=follow_symlinks,
                    )

                with mock.patch(
                    "os.mkdir", side_effect=track_leaf_mkdir
                ), mock.patch("os.open", side_effect=track_leaf_open), mock.patch(
                    "os.fstat", side_effect=fail_leaf_fstat
                ), mock.patch(
                    "os.stat", side_effect=fail_leaf_stat
                ):
                    with self.assertRaisesRegex(
                        OSError, f"post-mkdir leaf {fault}"
                    ):
                        self.execute(target)
                self.assertTrue(failed)
                self.assertFalse(target.exists())

    def test_concurrent_foreign_root_is_never_claimed_by_failure_publication(self):
        self.output_parent.mkdir()
        real_mkdir = os.mkdir
        injected = False

        def create_foreign_then_conflict(path, mode=0o777, *, dir_fd=None):
            nonlocal injected
            if path == self.default_output.name and dir_fd is not None and not injected:
                injected = True
                real_mkdir(path, mode, dir_fd=dir_fd)
                (self.default_output / "foreign-owner").write_text("foreign")
                raise FileExistsError("concurrent foreign root")
            return real_mkdir(path, mode, dir_fd=dir_fd)

        with mock.patch("os.mkdir", side_effect=create_foreign_then_conflict):
            with self.assertRaises(FileExistsError):
                self.execute()
        self.assertTrue(injected)
        self.assertEqual(
            (self.default_output / "foreign-owner").read_text(), "foreign"
        )
        self.assertFalse(
            (self.default_output / replay.TERMINAL_MANIFEST_NAME).exists()
        )

    def test_parent_swap_during_mkdir_cannot_redirect_output(self):
        self.output_parent.mkdir()
        moved_parent = self.root / "moved-output-parent"
        real_mkdir = os.mkdir
        swapped = False

        def swap_then_mkdir(path, mode=0o777, *, dir_fd=None):
            nonlocal swapped
            if path == self.default_output.name and dir_fd is not None and not swapped:
                swapped = True
                os.rename(self.output_parent, moved_parent)
                real_mkdir(self.output_parent)
                real_mkdir(self.default_output)
                (self.default_output / "foreign-owner").write_text("foreign")
            return real_mkdir(path, mode, dir_fd=dir_fd)

        with mock.patch("os.mkdir", side_effect=swap_then_mkdir):
            with self.assertRaises(ValueError):
                self.execute()
        self.assertTrue(swapped)
        self.assertEqual(
            (self.default_output / "foreign-owner").read_text(), "foreign"
        )
        self.assertFalse(
            (self.default_output / replay.TERMINAL_MANIFEST_NAME).exists()
        )

    def test_root_swap_at_raw_open_never_writes_foreign_replacement(self):
        moved = self.output_parent / "owned-moved-raw"
        real_open = os.open
        swapped = False

        def swap_at_raw_open(path, flags, mode=0o777, *, dir_fd=None):
            nonlocal swapped
            if Path(path).name == replay.RAW_PROCESS_NAME and not swapped:
                swapped = True
                os.rename(self.default_output, moved)
                self.default_output.mkdir()
                (self.default_output / "foreign-owner").write_text("foreign")
            return real_open(path, flags, mode, dir_fd=dir_fd)

        with mock.patch("os.open", side_effect=swap_at_raw_open):
            with self.assertRaises(ValueError):
                self.execute()
        self.assertTrue(swapped)
        self.assertEqual(
            (self.default_output / "foreign-owner").read_text(), "foreign"
        )
        self.assertFalse((self.default_output / replay.RAW_PROCESS_NAME).exists())
        self.assertFalse(
            (self.default_output / replay.TERMINAL_MANIFEST_NAME).exists()
        )
        self.assertTrue((moved / replay.RAW_PROCESS_NAME).exists())
        self.assertEqual(
            json.loads((moved / replay.TERMINAL_MANIFEST_NAME).read_text())[
                "status"
            ],
            "failed",
        )

    def test_root_swap_at_terminal_link_rolls_back_owned_completion(self):
        moved = self.output_parent / "owned-moved-link"
        real_link = os.link
        swapped = False

        def swap_at_manifest_link(
            source,
            destination,
            *,
            src_dir_fd=None,
            dst_dir_fd=None,
            follow_symlinks=True,
        ):
            nonlocal swapped
            if Path(destination).name == replay.TERMINAL_MANIFEST_NAME and not swapped:
                swapped = True
                os.rename(self.default_output, moved)
                self.default_output.mkdir()
                (self.default_output / "foreign-owner").write_text("foreign")
            return real_link(
                source,
                destination,
                src_dir_fd=src_dir_fd,
                dst_dir_fd=dst_dir_fd,
                follow_symlinks=follow_symlinks,
            )

        with mock.patch("os.link", side_effect=swap_at_manifest_link):
            with self.assertRaises(ValueError):
                self.execute()
        self.assertTrue(swapped)
        self.assertEqual(
            (self.default_output / "foreign-owner").read_text(), "foreign"
        )
        self.assertFalse(
            (self.default_output / replay.TERMINAL_MANIFEST_NAME).exists()
        )
        self.assertTrue((moved / replay.RAW_PROCESS_NAME).exists())
        self.assertEqual(
            json.loads((moved / replay.TERMINAL_MANIFEST_NAME).read_text())[
                "status"
            ],
            "failed",
        )

    def test_root_swap_at_failed_link_never_writes_foreign_replacement(self):
        moved = self.output_parent / "owned-moved-failed-link"
        real_link = os.link
        swapped = False

        def swap_at_failed_manifest_link(
            source,
            destination,
            *,
            src_dir_fd=None,
            dst_dir_fd=None,
            follow_symlinks=True,
        ):
            nonlocal swapped
            if (
                Path(source).name.endswith(".failed")
                and Path(destination).name == replay.TERMINAL_MANIFEST_NAME
                and not swapped
            ):
                swapped = True
                os.rename(self.default_output, moved)
                self.default_output.mkdir()
                (self.default_output / "foreign-owner").write_text("foreign")
            return real_link(
                source,
                destination,
                src_dir_fd=src_dir_fd,
                dst_dir_fd=dst_dir_fd,
                follow_symlinks=follow_symlinks,
            )

        with mock.patch.object(
            replay, "_write_row", side_effect=RuntimeError("row")
        ), mock.patch("os.link", side_effect=swap_at_failed_manifest_link):
            with self.assertRaisesRegex(RuntimeError, "row"):
                self.execute()
        self.assertTrue(swapped)
        self.assertEqual(
            (self.default_output / "foreign-owner").read_text(), "foreign"
        )
        self.assertFalse(
            (self.default_output / replay.TERMINAL_MANIFEST_NAME).exists()
        )
        self.assertEqual(
            json.loads((moved / replay.TERMINAL_MANIFEST_NAME).read_text())[
                "status"
            ],
            "failed",
        )

    def test_stage_cleanup_swap_preserves_foreign_and_owned_inodes(self):
        root = self.output_parent / "direct-cleanup"
        transaction = replay._create_exact_root(root)
        close_transaction = getattr(
            replay, "_close_output_transaction", lambda unused: None
        )
        try:
            stage = replay._write_stage(
                transaction,
                "completed",
                b"owned-stage",
            )
            stage_path = replay._stage_path(root, "completed")
            preserved = stage_path.with_name(stage_path.name + ".owned-preserved")
            real_rename = os.rename
            real_open = os.open
            injected = False

            def replace_before_quarantine(
                source,
                destination,
                *,
                src_dir_fd=None,
                dst_dir_fd=None,
            ):
                nonlocal injected
                if source == stage["name"] and not injected:
                    injected = True
                    real_rename(
                        source,
                        preserved.name,
                        src_dir_fd=src_dir_fd,
                        dst_dir_fd=src_dir_fd,
                    )
                    descriptor = real_open(
                        source,
                        os.O_WRONLY | os.O_CREAT | os.O_EXCL,
                        0o600,
                        dir_fd=src_dir_fd,
                    )
                    try:
                        os.write(descriptor, b"foreign-stage")
                    finally:
                        os.close(descriptor)
                return real_rename(
                    source,
                    destination,
                    src_dir_fd=src_dir_fd,
                    dst_dir_fd=dst_dir_fd,
                )

            with mock.patch("os.rename", side_effect=replace_before_quarantine):
                with self.assertRaises(ValueError):
                    replay._cleanup_stage(transaction, stage)
            self.assertTrue(injected)
            self.assertEqual(stage_path.read_bytes(), b"foreign-stage")
            self.assertEqual(preserved.read_bytes(), b"owned-stage")
        finally:
            close_transaction(transaction)

    def test_stage_link_swap_never_publishes_foreign_stage_bytes(self):
        root = self.output_parent / "direct-link"
        transaction = replay._create_exact_root(root)
        close_transaction = getattr(
            replay, "_close_output_transaction", lambda unused: None
        )
        try:
            stage = replay._write_stage(
                transaction,
                "completed",
                b"owned-stage",
            )
            stage_path = replay._stage_path(root, "completed")
            preserved = stage_path.with_name(stage_path.name + ".owned-preserved")
            real_link = os.link
            real_rename = os.rename
            real_open = os.open
            injected = False

            def replace_before_link(
                source,
                destination,
                *,
                src_dir_fd=None,
                dst_dir_fd=None,
                follow_symlinks=True,
            ):
                nonlocal injected
                if source == stage["name"] and not injected:
                    injected = True
                    real_rename(
                        source,
                        preserved.name,
                        src_dir_fd=src_dir_fd,
                        dst_dir_fd=src_dir_fd,
                    )
                    descriptor = real_open(
                        source,
                        os.O_WRONLY | os.O_CREAT | os.O_EXCL,
                        0o600,
                        dir_fd=src_dir_fd,
                    )
                    try:
                        os.write(descriptor, b"foreign-stage")
                    finally:
                        os.close(descriptor)
                return real_link(
                    source,
                    destination,
                    src_dir_fd=src_dir_fd,
                    dst_dir_fd=dst_dir_fd,
                    follow_symlinks=follow_symlinks,
                )

            with mock.patch("os.link", side_effect=replace_before_link):
                with self.assertRaises(ValueError):
                    replay._link_stage(transaction, stage)
            self.assertTrue(injected)
            self.assertFalse(
                (root / replay.TERMINAL_MANIFEST_NAME).exists()
            )
            self.assertEqual(stage_path.read_bytes(), b"foreign-stage")
            self.assertEqual(preserved.read_bytes(), b"owned-stage")
        finally:
            close_transaction(transaction)

    def test_stage_adoption_baseexception_closes_fd_and_removes_entry(self):
        class ExplodingSet(set):
            def add(self, value):
                super().add(value)
                raise KeyboardInterrupt("stage adoption")

        root = self.output_parent / "stage-adoption"
        baseline = open_fd_count()
        transaction = replay._create_exact_root(root)
        transaction["resource_fds"] = ExplodingSet()
        try:
            with self.assertRaisesRegex(KeyboardInterrupt, "stage adoption"):
                replay._write_stage(transaction, "completed", b"stage")
            self.assertFalse(
                replay._stage_path(root, "completed").exists()
            )
        finally:
            replay._close_output_transaction(transaction)
        self.assertEqual(open_fd_count(), baseline)

    def test_raw_adoption_baseexception_closes_fd_and_removes_entry(self):
        class ExplodingSet(set):
            def add(self, value):
                super().add(value)
                raise KeyboardInterrupt("raw adoption")

        root = self.output_parent / "raw-adoption"
        baseline = open_fd_count()
        transaction = replay._create_exact_root(root)
        transaction["resource_fds"] = ExplodingSet()
        try:
            with self.assertRaisesRegex(KeyboardInterrupt, "raw adoption"):
                replay._open_raw_output(transaction)
            self.assertFalse((root / replay.RAW_PROCESS_NAME).exists())
        finally:
            replay._close_output_transaction(transaction)
        self.assertEqual(open_fd_count(), baseline)

    def test_stage_write_baseexception_rolls_back_entry_and_descriptor(self):
        root = self.output_parent / "stage-write-failure"
        transaction = replay._create_exact_root(root)
        baseline = open_fd_count()
        try:
            with mock.patch(
                "os.fsync", side_effect=KeyboardInterrupt("stage fsync")
            ):
                with self.assertRaisesRegex(KeyboardInterrupt, "stage fsync"):
                    replay._write_stage(
                        transaction,
                        "finalizing",
                        b"partial-stage",
                    )
            self.assertFalse(
                replay._stage_path(root, "finalizing").exists()
            )
            self.assertEqual(open_fd_count(), baseline)
            self.assertEqual(transaction["resource_fds"], set())
        finally:
            replay._close_output_transaction(transaction)

    def test_stage_rollback_fault_is_noted_and_retried_without_orphan(self):
        root = self.output_parent / "stage-rollback-fault"
        transaction = replay._create_exact_root(root)
        real_rename = os.rename
        rename_calls = 0

        def fail_first_quarantine_rename(*args, **kwargs):
            nonlocal rename_calls
            rename_calls += 1
            if rename_calls == 1:
                raise OSError("quarantine fault")
            return real_rename(*args, **kwargs)

        try:
            with mock.patch(
                "os.fsync", side_effect=RuntimeError("stage write")
            ), mock.patch(
                "os.rename", side_effect=fail_first_quarantine_rename
            ):
                with self.assertRaisesRegex(RuntimeError, "stage write") as caught:
                    replay._write_stage(
                        transaction,
                        "failed",
                        b"partial-stage",
                    )
            self.assertTrue(
                any(
                    "stage rollback failed" in note
                    and "quarantine fault" in note
                    for note in getattr(caught.exception, "__notes__", [])
                )
            )
            self.assertGreaterEqual(rename_calls, 2)
            self.assertFalse(replay._stage_path(root, "failed").exists())
        finally:
            replay._close_output_transaction(transaction)

    def test_stage_rollback_resumes_after_quarantine_unlink_fault(self):
        root = self.output_parent / "stage-rollback-unlink-fault"
        transaction = replay._create_exact_root(root)
        baseline = open_fd_count()
        stage_path = replay._stage_path(root, "finalizing")
        rollback_prefix = f".{stage_path.name}.rollback."
        real_unlink = os.unlink
        unlink_faulted = False

        def fail_first_rollback_unlink(path, *, dir_fd=None):
            nonlocal unlink_faulted
            if str(path).startswith(rollback_prefix) and not unlink_faulted:
                unlink_faulted = True
                raise OSError("transient quarantine unlink fault")
            return real_unlink(path, dir_fd=dir_fd)

        try:
            with mock.patch(
                "os.fsync", side_effect=RuntimeError("stage fsync")
            ), mock.patch(
                "os.unlink", side_effect=fail_first_rollback_unlink
            ):
                with self.assertRaisesRegex(RuntimeError, "stage fsync") as caught:
                    replay._write_stage(
                        transaction,
                        "finalizing",
                        b"partial-stage",
                    )
            self.assertTrue(unlink_faulted)
            self.assertFalse(stage_path.exists())
            self.assertEqual(
                [
                    entry.name
                    for entry in root.parent.iterdir()
                    if entry.name.startswith(rollback_prefix)
                ],
                [],
            )
            self.assertEqual(open_fd_count(), baseline)
            self.assertEqual(transaction["resource_fds"], set())
            self.assertTrue(
                any(
                    "transient quarantine unlink fault" in note
                    for note in getattr(caught.exception, "__notes__", [])
                )
            )
        finally:
            replay._close_output_transaction(transaction)

    def test_raw_adoption_rollback_resumes_after_quarantine_unlink_fault(self):
        class ExplodingSet(set):
            def add(self, value):
                super().add(value)
                raise KeyboardInterrupt("raw adoption")

        root = self.output_parent / "raw-rollback-unlink-fault"
        transaction = replay._create_exact_root(root)
        transaction["resource_fds"] = ExplodingSet()
        baseline = open_fd_count()
        rollback_prefix = f".{replay.RAW_PROCESS_NAME}.rollback."
        real_unlink = os.unlink
        unlink_faulted = False

        def fail_first_rollback_unlink(path, *, dir_fd=None):
            nonlocal unlink_faulted
            if str(path).startswith(rollback_prefix) and not unlink_faulted:
                unlink_faulted = True
                raise OSError("transient quarantine unlink fault")
            return real_unlink(path, dir_fd=dir_fd)

        try:
            with mock.patch(
                "os.unlink", side_effect=fail_first_rollback_unlink
            ):
                with self.assertRaisesRegex(
                    KeyboardInterrupt, "raw adoption"
                ) as caught:
                    replay._open_raw_output(transaction)
            self.assertTrue(unlink_faulted)
            self.assertFalse((root / replay.RAW_PROCESS_NAME).exists())
            self.assertEqual(
                [
                    entry.name
                    for entry in root.iterdir()
                    if entry.name.startswith(rollback_prefix)
                ],
                [],
            )
            self.assertEqual(open_fd_count(), baseline)
            self.assertEqual(transaction["resource_fds"], set())
            self.assertTrue(
                any(
                    "transient quarantine unlink fault" in note
                    for note in getattr(caught.exception, "__notes__", [])
                )
            )
        finally:
            replay._close_output_transaction(transaction)

    def test_every_descriptor_relative_mkdir_fsyncs_containing_directory(self):
        root = self.output_parent / "mkdir-a" / "mkdir-b" / "bundle"
        events = []
        real_mkdir = os.mkdir
        real_fsync = os.fsync

        def record_mkdir(path, mode=0o777, *, dir_fd=None):
            result = real_mkdir(path, mode, dir_fd=dir_fd)
            events.append(("mkdir", path, dir_fd))
            return result

        def record_fsync(descriptor):
            events.append(("fsync", descriptor))
            return real_fsync(descriptor)

        with mock.patch("os.mkdir", side_effect=record_mkdir), mock.patch(
            "os.fsync", side_effect=record_fsync
        ):
            transaction = replay._create_exact_root(root)
        try:
            mkdir_indexes = [
                index
                for index, event in enumerate(events)
                if event[0] == "mkdir"
            ]
            self.assertGreaterEqual(len(mkdir_indexes), 3)
            for index in mkdir_indexes:
                self.assertLess(index + 1, len(events))
                self.assertEqual(
                    events[index + 1],
                    ("fsync", events[index][2]),
                )
        finally:
            replay._close_output_transaction(transaction)

    def test_close_transaction_suppresses_fault_and_closes_every_descriptor(self):
        root = self.output_parent / "close-all"
        baseline = open_fd_count()
        transaction = replay._create_exact_root(root)
        extras = {
            os.open("/dev/null", os.O_RDONLY),
            os.open("/dev/null", os.O_RDONLY),
        }
        transaction["resource_fds"].update(extras)
        tracked = {
            transaction["root_fd"],
            transaction["parent_fd"],
            *extras,
        }
        real_close = os.close
        faulted = False

        def close_then_interrupt(descriptor):
            nonlocal faulted
            real_close(descriptor)
            if descriptor in tracked and not faulted:
                faulted = True
                raise KeyboardInterrupt("close boundary")

        with mock.patch("os.close", side_effect=close_then_interrupt):
            replay._close_output_transaction(transaction)
        self.assertTrue(faulted)
        for descriptor in tracked:
            with self.assertRaises(OSError):
                os.fstat(descriptor)
        self.assertEqual(open_fd_count(), baseline)
        self.assertTrue(transaction["close_faults"])

    def test_post_commit_boundary_exception_reconciles_to_completed(self):
        delivered = False

        def raise_after_commit():
            nonlocal delivered
            delivered = True
            raise RuntimeError("post-commit boundary")

        with mock.patch.object(
            replay,
            "_post_commit_boundary",
            side_effect=raise_after_commit,
            create=True,
        ):
            manifest = self.execute()
        self.assertTrue(delivered)
        self.assertEqual(manifest["status"], "completed")
        self.assertEqual(
            json.loads(
                (self.default_output / replay.TERMINAL_MANIFEST_NAME).read_text()
            )["status"],
            "completed",
        )

    def test_success_has_no_staging_and_completed_publication_is_last_throwing_boundary(self):
        manifest = self.execute()
        self.assertEqual(manifest["status"], "completed")
        self.assertEqual(
            manifest["protocol_schema_id"],
            "cbf2026-predictive-wnls-stage1-hermetic-protocol-v1",
        )
        self.assertEqual(manifest["evidence_class"], "hermetic_non_evidence_only")
        self.assertFalse(any(self.output_parent.glob(f".{self.default_output.name}.manifest.*")))
        self.assertEqual(
            json.loads((self.default_output / replay.TERMINAL_MANIFEST_NAME).read_text())["status"],
            "completed",
        )


class DeterminismAndMutationTests(ReplayHarness):
    def test_raw_gzip_bytes_and_both_hashes_are_independently_deterministic(self):
        first = self.default_output
        second = self.output_parent / "second"
        first_manifest = self.execute(first)
        self.write_protocol(second)
        second_manifest = self.execute(second)
        first_raw = first / replay.RAW_PROCESS_NAME
        second_raw = second / replay.RAW_PROCESS_NAME
        self.assertEqual(first_raw.read_bytes(), second_raw.read_bytes())
        self.assertEqual(sha256(first_raw), first_manifest["compressed_process_sha256"])
        self.assertEqual(sha256(second_raw), second_manifest["compressed_process_sha256"])
        with gzip.open(first_raw, "rb") as source:
            first_decompressed = source.read()
        with gzip.open(second_raw, "rb") as source:
            second_decompressed = source.read()
        self.assertEqual(first_decompressed, second_decompressed)
        self.assertEqual(
            hashlib.sha256(first_decompressed).hexdigest(),
            first_manifest["decompressed_process_sha256"],
        )
        self.assertNotIn(self.baseline_marker, first_decompressed)

    def test_raw_append_after_first_hash_cannot_publish_completed(self):
        original = replay._final_identity_and_disk_probe
        appended = False

        def append_during_final_probe(*args, **kwargs):
            nonlocal appended
            if not appended:
                appended = True
                with (self.default_output / replay.RAW_PROCESS_NAME).open(
                    "ab"
                ) as target:
                    target.write(b"post-hash-drift")
                    target.flush()
                    os.fsync(target.fileno())
            return original(*args, **kwargs)

        with mock.patch.object(
            replay,
            "_final_identity_and_disk_probe",
            side_effect=append_during_final_probe,
        ):
            with self.assertRaises(ValueError):
                self.execute()
        self.assertTrue(appended)
        terminal = json.loads(
            (self.default_output / replay.TERMINAL_MANIFEST_NAME).read_text()
        )
        self.assertEqual(terminal["status"], "failed")
        self.assertNotEqual(terminal["status"], "completed")

    def test_concrete_source_mutation_after_read_cannot_publish_completed(self):
        original = replay._write_row
        mutated = False

        def mutate_then_write(*args):
            nonlocal mutated
            if not mutated:
                mutated = True
                self.baseline_path.write_bytes(b"changed")
            return original(*args)

        with mock.patch.object(replay, "_write_row", side_effect=mutate_then_write):
            with self.assertRaises(ValueError):
                self.execute()
        terminal = json.loads((self.default_output / replay.TERMINAL_MANIFEST_NAME).read_text())
        self.assertEqual(terminal["status"], "failed")
        self.assertNotEqual(terminal["status"], "completed")

    def test_same_byte_source_inode_replacement_is_rejected(self):
        original = replay._write_row
        replaced = False

        def replace_then_write(*args):
            nonlocal replaced
            if not replaced:
                replaced = True
                replacement = self.input_root / "same-bytes.json"
                replacement.write_bytes(self.data_path.read_bytes())
                os.replace(replacement, self.data_path)
            return original(*args)

        with mock.patch.object(replay, "_write_row", side_effect=replace_then_write):
            with self.assertRaises(ValueError):
                self.execute()
        terminal = json.loads(
            (self.default_output / replay.TERMINAL_MANIFEST_NAME).read_text()
        )
        self.assertEqual(terminal["status"], "failed")

    def test_transient_different_bytes_cannot_be_parsed_under_registered_identity(self):
        original_load = replay._strict_load
        original_read = os.read
        truth_inode = self.data_path.stat().st_ino
        swapped = False

        def transient(path):
            nonlocal swapped
            if Path(path) == self.data_path and not swapped:
                swapped = True
                original_bytes = self.data_path.read_bytes()
                changed = tiny_data()
                changed["state"][0]["robots"][0]["state"]["x"] = 303.0
                write_json(self.data_path, changed)
                try:
                    return original_load(path)
                finally:
                    self.data_path.write_bytes(original_bytes)
            return original_load(path)

        def transient_during_fd_read(descriptor, size):
            nonlocal swapped
            if not swapped and os.fstat(descriptor).st_ino == truth_inode:
                swapped = True
                saved = self.input_root / "registered-data.json"
                os.rename(self.data_path, saved)
                changed = tiny_data()
                changed["state"][0]["robots"][0]["state"]["x"] = 303.0
                write_json(self.data_path, changed)
                try:
                    return original_read(descriptor, size)
                finally:
                    self.data_path.unlink()
                    os.rename(saved, self.data_path)
            return original_read(descriptor, size)

        with mock.patch.object(
            replay, "_strict_load", side_effect=transient
        ), mock.patch.object(
            replay.os, "read", side_effect=transient_during_fd_read
        ):
            self.execute()
        self.assertTrue(swapped)
        first = next(
            row
            for row in self.rows()
            if row["variant"] == "prediction_expiry"
            and row["seed"] == 11
            and row["frame_index"] == 0
            and row["robot_id"] == 1
        )
        self.assertEqual(first["offline_truth_position"], [3.0, 4.0])

    def test_protocol_identity_is_observed_and_protocol_mutation_fails_completion(self):
        original = replay._write_row
        mutated = False

        def mutate_then_write(*args):
            nonlocal mutated
            if not mutated:
                mutated = True
                protocol = json.loads(self.protocol_path.read_text())
                protocol["comment"] = "post-read mutation"
                write_json(self.protocol_path, protocol)
            return original(*args)

        with mock.patch.object(replay, "_write_row", side_effect=mutate_then_write):
            with self.assertRaises(ValueError):
                self.execute()
        terminal = json.loads((self.default_output / replay.TERMINAL_MANIFEST_NAME).read_text())
        self.assertEqual(terminal["status"], "failed")
        self.assertIn("protocol_identity", terminal)
        self.assertEqual(terminal["protocol_identity"]["path"], str(self.protocol_path))
