"""Contracts for immutable predictive-WNLS Stage 0 fixture extraction."""

import hashlib
import gzip
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

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
                self.assertIn("baseline_target_comparison", fixture["offline"])

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
        for filename, expected_hash in manifest["fixtures"].items():
            self.assertEqual(
                hashlib.sha256((FIXTURE_ROOT / filename).read_bytes()).hexdigest(),
                expected_hash,
            )

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
            offline={"truth_positions": [{"frame_index": 0, "positions": {"2": [0.0, 0.0]}}]},
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
            truth = root / "truth.json"
            manifest = root / "manifest.json"
            baseline = root / "baseline.jsonl.gz"
            output = root / "missing-parent" / "fixtures"
            truth.write_text(json.dumps({"config": {}, "state": []}))
            manifest.write_text("{}")
            with gzip.open(baseline, "wt"):
                pass
            with mock.patch(
                "scripts.diagnostics.extract_predictive_wnls_stage0._runtime_prefix",
                return_value=({"kind": "historical_prefix"}, {"truth_positions": []}),
            ), mock.patch(
                "scripts.diagnostics.extract_predictive_wnls_stage0._comparison_row",
                return_value={},
            ):
                extract_stage0_fixtures(
                    truth_data_path=truth,
                    input_manifest_path=manifest,
                    baseline_process_path=baseline,
                    output_dir=output,
                )
            self.assertTrue((output / "manifest.json").is_file())


if __name__ == "__main__":
    unittest.main()
