"""Adversarial contracts for the exact Stage-1 predictive replay."""

import copy
import gzip
import hashlib
import json
import math
import os
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
        return {
            "schema_id": replay.PROTOCOL_SCHEMA_ID,
            "protocol_id": replay.PROTOCOL_ID,
            "implementation_parent_commit": "0" * 40,
            "binding_design": copy.deepcopy(replay.BINDING_DESIGN),
            "sources": self.source_entries(),
            "experiment": {
                **copy.deepcopy(replay.EXPERIMENT_CONTRACT),
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
                "test_smoke": {
                    "kind": "unregistered_smoke",
                    "output_root": str(output_root),
                    "range_noise_seeds": list(seeds),
                    "max_frames": max_frames,
                }
            },
            "evidence_lifecycle": copy.deepcopy(replay.EVIDENCE_LIFECYCLE),
            "commands": {"test_smoke": ["replay", "test_smoke"]},
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
    def test_protocol_accepts_bound_analyzer_invocation_without_selecting_it(self):
        analyzer_root = self.output_parent / "analysis"
        registered_root = self.output_parent / "registered"

        def add_analyzer(protocol):
            protocol["invocations"]["registered_replay"] = {
                "kind": "registered_exactly_once",
                "output_root": str(registered_root),
                "range_noise_seeds": [11, 12],
                "max_frames": 2,
            }
            protocol["invocations"]["registered_analyzer"] = {
                "kind": "registered_exactly_once",
                "development_manifest_path": str(
                    registered_root / replay.TERMINAL_MANIFEST_NAME
                ),
                "output_root": str(analyzer_root),
                "expected_baseline_sha256": sha256(self.baseline_path),
            }
            protocol["commands"]["registered_replay"] = [
                "replay",
                "registered_replay",
            ]
            protocol["commands"]["registered_analyzer"] = [
                "analyze",
                "registered_analyzer",
            ]

        self.write_protocol(self.default_output, mutate=add_analyzer)
        manifest = self.execute()
        self.assertEqual(manifest["status"], "completed")
        self.assertEqual(manifest["selected_invocation"], "test_smoke")
        self.assertFalse(analyzer_root.exists())

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
            "command": lambda p: p["commands"].update(test_smoke=[]),
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
        self.assertEqual(
            record["noise_seed"],
            replay.stable_measurement_seed(11, 0, 1, key[0], key[1]),
        )
        self.assertEqual(set(record), {"present", "noisy_range", "noise_seed"})

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
        self.assertTrue(expiry_uav["used"])
        self.assertFalse(expiry_uav["eligible"])
        self.assertEqual(expiry_uav["exclusion_reason"], None)
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
        for row in self.rows():
            self.assertIn("attempt_base_anchor_provenance", row)
            self.assertIn("base_anchor_provenance", row)
            self.assertIn("private_reacquisition_seed", row)
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
        with mock.patch.object(replay, "available_bytes", return_value=replay.HARD_FLOOR_BYTES - 1):
            with self.assertRaises(DiskSpaceError):
                self.execute(target)
        self.assertEqual(json.loads((target / replay.TERMINAL_MANIFEST_NAME).read_text())["status"], "failed")

        target = self.output_parent / "cap"
        self.write_protocol(target)
        with mock.patch.object(replay, "allocated_bytes", return_value=replay.RAW_BUNDLE_CAP_BYTES + 1):
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
                if attribute == "_cleanup_stage":
                    injected = mock.Mock(side_effect=[error, None, None])
                else:
                    injected = mock.Mock(side_effect=error)
                with mock.patch.object(replay, attribute, injected):
                    with self.assertRaises(RuntimeError):
                        self.execute(target)
                terminal = json.loads((target / replay.TERMINAL_MANIFEST_NAME).read_text())
                self.assertEqual(terminal["status"], "failed")
                self.assertFalse(any(target.parent.glob(f".{target.name}.manifest.*")))
                setattr(replay, attribute, original)

    def test_success_has_no_staging_and_completed_publication_is_last_throwing_boundary(self):
        manifest = self.execute()
        self.assertEqual(manifest["status"], "completed")
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
