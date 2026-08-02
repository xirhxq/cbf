import gzip
import json
import tempfile
import unittest
import subprocess
import sys
from pathlib import Path

import scripts.diagnostics.generate_qualified_measurements as measurement_module
from scripts.diagnostics.generate_qualified_measurements import (
    generate_measurement_bundle,
)


def write_truth(path: Path):
    rows = [
        {
            "frame_index": 0,
            "owner_id": 1,
            "reference_key": ["base", 0],
            "role_tags": ["dynamic_primary", "fixed_fim_ablation"],
            "endpoint_truth_position": [0.0, 0.0],
            "reference_truth_position": [3.0, 4.0],
            "reference_position": [3.0, 4.0],
            "reference_covariance": [[0.0, 0.0], [0.0, 0.0]],
            "base_anchor_provenance": [0],
        },
        {
            "frame_index": 0,
            "owner_id": 2,
            "reference_key": ["uav", 1],
            "role_tags": ["dynamic_primary"],
            "endpoint_truth_position": [0.0, 0.0],
            "reference_truth_position": [0.0, 12.0],
            "reference_position": [0.0, 12.0],
            "reference_covariance": [[1.0, 0.0], [0.0, 1.0]],
            "base_anchor_provenance": [0, 2],
        },
    ]
    with path.open("xb") as raw:
        with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as sink:
            for row in rows:
                sink.write(json.dumps(row, sort_keys=True).encode() + b"\n")
    return rows


def read_rows(path: Path):
    with gzip.open(path, "rt", encoding="utf-8") as source:
        return [json.loads(line) for line in source]


def write_controller_truth(path: Path):
    truth = [
        {"robot_id": robot_id, "position": [float(robot_id), 0.0]}
        for robot_id in range(1, 15)
    ]
    nodes = []
    for robot_id in range(1, 15):
        if robot_id == 1:
            reference_id, reference_kind = -1, "base"
            direction, distance = [1.0, 0.0], 1551.0
            covariance = [[0.0, 0.0], [0.0, 0.0]]
        else:
            reference_id, reference_kind = robot_id - 1, "uav"
            direction, distance = [1.0, 0.0], 1.0
            covariance = [[1.0, 0.0], [0.0, 1.0]]
        nodes.append({
            "robot_id": robot_id,
            "interface_estimate": [float(robot_id), 0.0],
            "covariance": [[1.0, 0.0], [0.0, 1.0]],
            "references": [{
                "canonical_reference_id": reference_id,
                "reference_kind": reference_kind,
                "direction": direction,
                "distance": distance,
                "predecessor_covariance": covariance,
            }],
        })
    row = {
        "record_type": "controller_interval",
        "schema_version": "cbf2026-qualified-evidence-v1",
        "campaign_id": "development-v1",
        "condition": "dynamic_primary",
        "trajectory_seed": 2026080101,
        "range_noise_seed": 2026081101,
        "frame_index": 0,
        "runtime": {"nodes": nodes},
        "analyzer_only": {"truth": truth},
    }
    with path.open("xb") as raw:
        with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as sink:
            sink.write(json.dumps(row, sort_keys=True).encode() + b"\n")
    required = [
        (0, robot_id, ("base", 0) if robot_id == 1 else ("uav", robot_id - 1))
        for robot_id in range(1, 15)
    ]
    return {"dynamic_primary": required, "fixed_fim_ablation": required}


class QualifiedMeasurementTests(unittest.TestCase):
    """Catch noise regeneration or analyzer-only leakage into runtime input."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(
            prefix="qualified-measurements-"
        )
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.truth = self.root / "truth.jsonl.gz"
        write_truth(self.truth)
        self.required = {
            "dynamic_primary": [(0, 1, ("base", 0)), (0, 2, ("uav", 1))],
            "fixed_fim_ablation": [(0, 1, ("base", 0))],
        }

    def generate(self, directory: str, seed=2026081101):
        target = self.root / directory
        manifest = generate_measurement_bundle(
            self.truth,
            target,
            range_noise_seed=seed,
            ranging_sigma=0.5,
            config_sha256="a" * 64,
            required_edges_by_condition=self.required,
            truth_manifest={
                "terminal": True, "status": "completed",
                "sha256": __import__("hashlib").sha256(
                    self.truth.read_bytes()
                ).hexdigest(),
            },
        )
        runtime = target / "runtime.jsonl.gz"
        audit = target / "audit.jsonl.gz"
        result = {
            "runtime_manifest": json.loads(
                (target / "runtime.manifest.json").read_text()
            ),
            "audit_manifest": json.loads(
                (target / "audit.manifest.json").read_text()
            ),
            "fixed_edges_subset_of_union": True,
            "bundle_manifest": manifest,
        }
        return result, runtime, audit

    def test_runtime_and_audit_streams_are_physically_separate_and_truth_isolated(self):
        result, runtime_path, audit_path = self.generate("first")

        runtime = read_rows(runtime_path)
        audit = read_rows(audit_path)
        self.assertEqual(len(runtime), 2)
        self.assertEqual(len(audit), 2)
        self.assertNotEqual(runtime_path, audit_path)
        forbidden = ("truth", "noiseless", "sampled_noise", "audit")
        self.assertTrue(all(
            not any(fragment in key.lower() for fragment in forbidden)
            for row in runtime
            for key in row
        ))
        self.assertEqual(runtime[0]["ranging_sigma"], 0.5)
        self.assertEqual(runtime[0]["config_sha256"], "a" * 64)
        self.assertAlmostEqual(
            runtime[0]["noisy_range"],
            audit[0]["noiseless_range"] + audit[0]["sampled_noise"],
        )
        self.assertEqual(
            result["runtime_manifest"]["measurement_stream_id"],
            result["audit_manifest"]["measurement_stream_id"],
        )
        self.assertNotEqual(
            result["runtime_manifest"]["sha256"],
            result["audit_manifest"]["sha256"],
        )

    def test_same_seed_regenerates_identical_bytes_and_different_seed_changes_ranges(self):
        _first, runtime_a, audit_a = self.generate("same-a", seed=2026081101)
        _second, runtime_b, audit_b = self.generate("same-b", seed=2026081101)
        _third, runtime_c, _audit_c = self.generate("different", seed=2026081102)

        self.assertEqual(runtime_a.read_bytes(), runtime_b.read_bytes())
        self.assertEqual(audit_a.read_bytes(), audit_b.read_bytes())
        self.assertNotEqual(
            [row["noisy_range"] for row in read_rows(runtime_a)],
            [row["noisy_range"] for row in read_rows(runtime_c)],
        )

    def test_union_contains_every_condition_edge_exactly_once(self):
        result, runtime, _audit = self.generate("union")

        keys = [
            (row["frame_index"], row["owner_id"], tuple(row["reference_key"]))
            for row in read_rows(runtime)
        ]
        self.assertEqual(keys, [(0, 1, ("base", 0)), (0, 2, ("uav", 1))])
        self.assertEqual(len(keys), len(set(keys)))
        self.assertTrue(result["fixed_edges_subset_of_union"])

    def test_no_replace_and_invalid_union_publish_nothing_new(self):
        _result, runtime, audit = self.generate("immutable")
        before = (runtime.read_bytes(), audit.read_bytes())
        with self.assertRaises(FileExistsError):
            generate_measurement_bundle(
                self.truth,
                runtime.parent,
                range_noise_seed=2026081101,
                ranging_sigma=0.5,
                config_sha256="a" * 64,
                required_edges_by_condition=self.required,
                truth_manifest={"terminal": True, "status": "completed",
                                "sha256": __import__("hashlib").sha256(self.truth.read_bytes()).hexdigest()},
            )
        self.assertEqual((runtime.read_bytes(), audit.read_bytes()), before)

        invalid_dir = self.root / "invalid"
        with self.assertRaisesRegex(ValueError, "requested edge"):
            generate_measurement_bundle(
                self.truth,
                invalid_dir,
                range_noise_seed=2026081101,
                ranging_sigma=0.5,
                config_sha256="a" * 64,
                required_edges_by_condition={
                    **self.required,
                    "fixed_fim_ablation": [(0, 7, ("uav", 6))],
                },
                truth_manifest={"terminal": True, "status": "completed",
                                "sha256": __import__("hashlib").sha256(self.truth.read_bytes()).hexdigest()},
            )
        self.assertFalse(invalid_dir.exists())

    def test_nontransactional_legacy_publication_api_is_not_exposed(self):
        self.assertFalse(hasattr(
            measurement_module, "generate_qualified_measurements"
        ))
        self.assertFalse(hasattr(
            measurement_module, "_generate_qualified_measurements_legacy"
        ))

    def test_cli_help_exposes_internal_measurement_contract(self):
        script = Path(__file__).resolve().parents[1] / "scripts" / "diagnostics" / "generate_qualified_measurements.py"
        result = subprocess.run(
            [sys.executable, str(script), "--help"],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        for option in (
            "--truth", "--output-root", "--range-noise-seed",
            "--ranging-sigma", "--config-sha256", "--edge-schedule",
        ):
            self.assertIn(option, result.stdout)


class TransactionalMeasurementBundleTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(
            prefix="qualified-measurement-transaction-"
        )
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.truth = self.root / "truth.jsonl.gz"
        write_truth(self.truth)
        import hashlib
        self.truth_sha = hashlib.sha256(self.truth.read_bytes()).hexdigest()
        self.required = {
            "dynamic_primary": [(0, 1, ("base", 0)), (0, 2, ("uav", 1))],
            "fixed_fim_ablation": [(0, 1, ("base", 0))],
        }

    def call(self, output, **overrides):
        options = {
            "range_noise_seed": 2026081101,
            "ranging_sigma": 0.5,
            "config_sha256": "a" * 64,
            "required_edges_by_condition": self.required,
            "truth_manifest": {
                "terminal": True,
                "status": "completed",
                "sha256": self.truth_sha,
            },
        }
        options.update(overrides)
        return generate_measurement_bundle(self.truth, output, **options)

    def test_truth_descriptor_is_opened_exactly_once_and_hash_bound(self):
        opens = []

        def opener(path):
            opens.append(Path(path))
            return Path(path).open("rb")

        output = self.root / "bundle"
        manifest = self.call(output, truth_opener=opener)

        self.assertEqual(opens, [self.truth])
        self.assertEqual(manifest["truth_sha256"], self.truth_sha)
        self.assertEqual(
            sorted(path.name for path in output.iterdir()),
            ["audit.jsonl.gz", "audit.manifest.json", "manifest.json",
             "runtime.jsonl.gz", "runtime.manifest.json"],
        )

    def test_failure_before_directory_publish_exposes_no_partial_bundle(self):
        output = self.root / "never-visible"
        with self.assertRaisesRegex(RuntimeError, "injected publication"):
            self.call(
                output,
                publish_hook=lambda stage: (_ for _ in ()).throw(
                    RuntimeError("injected publication failure")
                ),
            )
        self.assertFalse(output.exists())
        self.assertEqual(
            [path for path in self.root.iterdir() if path.name.startswith(".never-visible")],
            [],
        )

    def test_truth_hash_tamper_discards_entire_staged_bundle(self):
        output = self.root / "tampered"
        with self.assertRaisesRegex(ValueError, "truth SHA-256"):
            self.call(
                output,
                truth_manifest={
                    "terminal": True,
                    "status": "completed",
                    "sha256": "0" * 64,
                },
            )
        self.assertFalse(output.exists())

    def test_real_controller_truth_shape_is_expanded_without_runtime_truth_leakage(self):
        controller = self.root / "controller.jsonl.gz"
        required = write_controller_truth(controller)
        import hashlib
        output = self.root / "controller-bundle"
        manifest = generate_measurement_bundle(
            controller,
            output,
            range_noise_seed=2026081101,
            ranging_sigma=0.5,
            config_sha256="a" * 64,
            required_edges_by_condition=required,
            truth_manifest={
                "terminal": True,
                "status": "completed",
                "sha256": hashlib.sha256(controller.read_bytes()).hexdigest(),
            },
        )

        runtime = read_rows(output / "runtime.jsonl.gz")
        self.assertEqual(manifest["row_count"], 14)
        self.assertEqual(len(runtime), 14)
        self.assertTrue(all("truth" not in json.dumps(row).lower() for row in runtime))
        self.assertEqual(runtime[1]["base_anchor_provenance"], [0])

    def test_frozen_controller_selector_does_not_materialize_repeated_edge_keys(self):
        controller = self.root / "controller-selector.jsonl.gz"
        write_controller_truth(controller)
        import hashlib
        output = self.root / "selector-bundle"
        manifest = generate_measurement_bundle(
            controller, output,
            range_noise_seed=2026081101, ranging_sigma=0.5,
            config_sha256="a" * 64,
            required_edges_by_condition={
                "dynamic_primary": "controller_references",
                "fixed_fim_ablation": "controller_references",
            },
            truth_manifest={"terminal": True, "status": "completed",
                            "sha256": hashlib.sha256(controller.read_bytes()).hexdigest()},
        )
        self.assertEqual(manifest["row_count"], 14)

    def test_dynamic_and_fixed_paper_policies_produce_distinct_tagged_sets(self):
        controller = self.root / "controller-distinct.jsonl.gz"
        write_controller_truth(controller)
        import hashlib
        output = self.root / "distinct-bundle"

        generate_measurement_bundle(
            controller, output,
            range_noise_seed=2026081101, ranging_sigma=0.5,
            config_sha256="a" * 64,
            required_edges_by_condition={
                "dynamic_primary": "controller_references",
                "fixed_fim_ablation": "fixed_paper_localization_references",
            },
            truth_manifest={"terminal": True, "status": "completed",
                            "sha256": hashlib.sha256(controller.read_bytes()).hexdigest()},
        )

        rows = read_rows(output / "runtime.jsonl.gz")
        keys = lambda role: {
            (row["frame_index"], row["owner_id"], tuple(row["reference_key"]))
            for row in rows if role in row["role_tags"]
        }
        dynamic = keys("dynamic_primary")
        fixed = keys("fixed_fim_ablation")
        union = {
            (row["frame_index"], row["owner_id"], tuple(row["reference_key"]))
            for row in rows
        }
        self.assertEqual(len(dynamic), 14)
        self.assertEqual(len(fixed), 28)
        self.assertNotEqual(dynamic, fixed)
        self.assertLess(fixed, union)
        self.assertTrue(dynamic & fixed)
        self.assertTrue(dynamic - fixed)
        self.assertTrue(fixed - dynamic)


if __name__ == "__main__":
    unittest.main()
