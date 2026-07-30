"""Hermetic contracts for the exact Stage-1 predictive replay."""

import gzip
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import scripts.diagnostics.replay_predictive_wnls_recovery as replay


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _strict_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, allow_nan=False, sort_keys=True))


def _data() -> dict:
    """Two UAVs whose frame-zero applied commands differ from nominal ones."""
    config = {
        "num": 2,
        "formation": {"parts": 1, "bases-id": [[0, 1]]},
        "bases": [[0.0, 0.0], [10.0, 0.0]],
        "initial": {"position": {"positions": [[2.0, 3.0], [4.0, 3.0]]}},
        "position_covariance": {"ranging_sigma": 0.5},
        "cbfs": {"without-slack": {"comm-fixed": {
            "max-range": 100.0, "min-neighbour-id-offset": -2,
            "max-neighbour-id-offset": 0,
        }}},
    }
    def robot(identifier, position, applied, nominal):
        return {
            "id": identifier,
            "state": {"x": position[0], "y": position[1]},
            "opt": {"result": {"vx": applied[0], "vy": applied[1]},
                    "nominal": {"vx": nominal[0], "vy": nominal[1]}},
        }
    return {
        "config": config,
        "state": [
            {"robots": [robot(1, (2.0, 3.0), (4.0, 0.0), (99.0, 99.0)),
                        robot(2, (4.0, 3.0), (2.0, 0.0), (88.0, 88.0))]},
            {"robots": [robot(1, (4.0, 3.0), (8.0, 0.0), (77.0, 77.0)),
                        robot(2, (5.0, 3.0), (6.0, 0.0), (66.0, 66.0))]},
        ],
    }


class PredictiveReplayTests(unittest.TestCase):
    """All tests use actual JSON/gzip files and never need production evidence."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.input_root = self.root / "input"
        self.input_root.mkdir()
        self.output_parent = self.root / "output"
        self.data_path = self.input_root / "data.json"
        self.input_manifest_path = self.input_root / "input-manifest.json"
        _strict_json(self.data_path, _data())
        _strict_json(self.input_manifest_path, {"termination_reason": "completed"})
        self.protocol_path = self.input_root / "protocol.json"
        self._write_protocol()

    def tearDown(self):
        self.temporary.cleanup()

    def _write_protocol(self):
        project = Path(replay.__file__).resolve().parents[2]
        replay_source = project / "scripts/diagnostics/replay_predictive_wnls_recovery.py"
        estimator_source = project / "scripts/diagnostics/predictive_wnls.py"
        _strict_json(self.protocol_path, {
            "schema_id": "cbf2026-predictive-wnls-stage1-protocol-v1",
            "sources": {
                "truth_data": {"path": str(self.data_path), "sha256": _sha256(self.data_path)},
                "input_manifest": {"path": str(self.input_manifest_path), "sha256": _sha256(self.input_manifest_path)},
                "replay_source": {"path": str(replay_source), "sha256": _sha256(replay_source)},
                "estimator_source": {"path": str(estimator_source), "sha256": _sha256(estimator_source)},
            },
            "raw_schema": {
                "id": replay.RAW_SCHEMA_ID,
                "candidate_fields": list(replay.CANDIDATE_FIELDS),
                "proposal_trace_fields": list(replay.PROPOSAL_TRACE_FIELDS),
            },
        })

    def _run(self, name="bundle", **kwargs):
        return replay.replay_predictive_recovery(
            data_path=self.data_path,
            input_manifest_path=self.input_manifest_path,
            protocol_path=self.protocol_path,
            output_root=self.output_parent / name,
            run_seeds=(11, 12), max_frames=2, **kwargs,
        )

    def _rows(self, name="bundle"):
        with gzip.open(self.output_parent / name / replay.RAW_PROCESS_NAME, "rt") as source:
            return [json.loads(line) for line in source]

    def test_rows_are_paired_ordered_and_use_previous_applied_command(self):
        """Breaks if a row is missing, loop order drifts, or nominal/truth is used."""
        manifest = self._run()
        rows = self._rows()
        self.assertEqual(len(rows), 3 * 2 * 2 * 2)
        self.assertEqual(manifest["status"], "completed")
        self.assertEqual(
            [(row["variant"], row["seed"], row["frame_index"], row["robot_id"])
             for row in rows],
            [(variant, seed, frame, robot)
             for variant in replay.DEVELOPMENT_VARIANTS for seed in (11, 12)
             for frame in (0, 1) for robot in (1, 2)],
        )
        next_row = next(row for row in rows if row["frame_index"] == 1 and row["robot_id"] == 1)
        self.assertEqual(next_row["applied_command_source_frame"], 0)
        self.assertEqual(next_row["applied_command"], [4.0, 0.0])

    def test_sensor_truth_does_not_reach_runtime_and_noise_is_stable(self):
        """Breaks if estimator callbacks see truth or edge noise depends on run order."""
        captured = []
        original = replay.qualify_active_references
        def checked(**kwargs):
            captured.append(kwargs["measurement_records"])
            self.assertNotIn("truth", repr(kwargs))
            self.assertNotIn("true_range", repr(kwargs))
            return original(**kwargs)
        with mock.patch.object(replay, "qualify_active_references", side_effect=checked):
            self._run("first")
        self._run("second")
        self.assertTrue(captured)
        with gzip.open(self.output_parent / "first" / replay.RAW_PROCESS_NAME, "rb") as first, \
             gzip.open(self.output_parent / "second" / replay.RAW_PROCESS_NAME, "rb") as second:
            self.assertEqual(first.read(), second.read())

    def test_qualification_and_ablation_keep_reference_evidence_separate(self):
        """Breaks if a qualified variant consumes an ineligible UAV anchor."""
        with mock.patch.object(replay, "reference_is_eligible", return_value=False):
            self._run()
        rows = self._rows()
        qualified = [r for r in rows if r["variant"] != "prediction_expiry"]
        self.assertTrue(all(not r["reference_violations"] for r in qualified))
        for row in rows:
            self.assertIn(row["output_status"], ("fresh", "predicted", "unavailable"))
            self.assertIn(row["attempt_status"], replay.ATTEMPT_STATUSES)
            self.assertIn("private_reacquisition_seed", row)
            self.assertIsInstance(row["base_anchor_provenance"], list)
        expiry = [r for r in rows if r["variant"] == "prediction_expiry"]
        self.assertTrue(any(r["reference_violations"] for r in expiry))

    def test_compact_evidence_and_offline_semantics_are_strict(self):
        """Breaks if trace arrays, radius containment, or q-error nulls drift."""
        self._run()
        for row in self._rows():
            for candidate in row["candidates"]:
                self.assertIsInstance(candidate, list)
                self.assertEqual(len(candidate), len(replay.CANDIDATE_FIELDS))
                candidate_values = dict(zip(replay.CANDIDATE_FIELDS, candidate))
                for proposal in candidate_values["proposal_trace"]:
                    self.assertIsInstance(proposal, list)
                    self.assertEqual(len(proposal), len(replay.PROPOSAL_TRACE_FIELDS))
            if row["output_status"] == "fresh":
                self.assertIsInstance(row["offline_fresh_containment"], bool)
                self.assertIsNone(row["offline_aged_radius_containment"])
            elif row["output_status"] == "predicted":
                self.assertIsNone(row["offline_fresh_containment"])
                self.assertIsInstance(row["offline_aged_radius_containment"], bool)
            else:
                self.assertIsNone(row["offline_fresh_q_error"])
                self.assertIsNone(row["offline_aged_q_error"])

    def test_integrity_overlap_target_and_failures_are_terminal(self):
        """Breaks if trust checks allocate output or errors leave an ambiguous run."""
        protocol = json.loads(self.protocol_path.read_text())
        protocol["sources"]["truth_data"]["sha256"] = "0" * 64
        _strict_json(self.protocol_path, protocol)
        with self.assertRaises(ValueError):
            self._run("already")
        self.assertFalse((self.output_parent / "already").exists())
        self._write_protocol()
        self.output_parent.mkdir(exist_ok=True)
        (self.output_parent / "already").mkdir()
        with self.assertRaises(FileExistsError):
            self._run("already")
        with self.assertRaises(ValueError):
            replay.replay_predictive_recovery(
                data_path=self.data_path, input_manifest_path=self.input_manifest_path,
                protocol_path=self.protocol_path, output_root=self.data_path.parent,
                run_seeds=(11,), max_frames=2,
            )
        for name, exception in (("ordinary", RuntimeError("boom")),
                                ("interrupted", KeyboardInterrupt()),
                                ("exited", SystemExit(3))):
            with self.subTest(exception=type(exception).__name__), \
                 mock.patch.object(replay, "_write_row", side_effect=exception):
                with self.assertRaises(type(exception)):
                    self._run(name)
            failed = json.loads((self.output_parent / name / replay.TERMINAL_MANIFEST_NAME).read_text())
            self.assertEqual(failed["status"], "failed")
            self.assertEqual(failed["error"]["type"], type(exception).__name__)

    def test_rehashing_deterministic_gzip_and_final_drift_probe(self):
        """Breaks if identity drift can produce a completed exact bundle."""
        first = self._run("a")
        second = self._run("b")
        self.assertEqual(first["compressed_process_sha256"], second["compressed_process_sha256"])
        self.assertEqual(first["decompressed_process_sha256"], second["decompressed_process_sha256"])
        with mock.patch.object(replay, "_verify_bound_identities", side_effect=[None, None, ValueError("drift")]):
            with self.assertRaises(ValueError):
                self._run("drift")
        failed = json.loads((self.output_parent / "drift" / replay.TERMINAL_MANIFEST_NAME).read_text())
        self.assertEqual(failed["status"], "failed")
