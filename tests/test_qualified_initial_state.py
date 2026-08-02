import copy
import json
import math
import tempfile
import time
import unittest
from pathlib import Path

from scripts.diagnostics.qualified_initial_state import (
    InitialStateAdmissionError,
    audit_positions,
    audit_seed,
    audit_frozen_initial_family,
    canonical_positions_sha256,
    collision_edges,
    dynamic_fim_reference_ids,
    family_semantic_sha256,
    fixed_localization_edges,
    load_qualified_initial_family,
    materialize_seed_positions,
    require_admitted_seed,
    validate_qualified_initial_family,
)


ROOT = Path(__file__).resolve().parents[1]
FAMILY_PATH = (
    ROOT / "config/diagnostics/qualified_initial_family_v1.json"
)

V4_FRAME_ZERO_POSITIONS = (
    (-1480.938929244014, 100.81607030759574),
    (-1409.4895671667527, 126.14818646893141),
    (-1477.8409425625919, -19.310751104565554),
    (-1420.7295251204407, -127.39160357880249),
    (-1432.551879404855, -104.59083790133123),
    (-1455.5630513751946, 44.38678429337485),
    (-1390.8482692570806, 47.9956218360925),
    (-1487.6316255982983, 4.030018318265107),
    (-1387.617852898455, 31.86759980632951),
    (-1424.706481167289, 187.98778418730916),
    (-1389.0487318716064, 69.09065915183174),
    (-1472.7289553564817, 7.507218479307014),
    (-1451.9053753366904, 157.7585197329898),
    (-1453.9507620164127, -189.51934349355219),
)


class QualifiedInitialFamilyContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.family = load_qualified_initial_family(FAMILY_PATH)
        cls.raw = json.loads(FAMILY_PATH.read_text(encoding="utf-8"))

    def test_family_is_strict_canonical_and_has_no_retry_semantics(self):
        self.assertEqual(self.family["namespace"], "cbf2026-v5-initial")
        self.assertEqual(
            self.family["schedule"]["registered_trajectory_seeds"],
            list(range(2026080201, 2026080211)),
        )
        self.assertEqual(
            self.family["schedule"]["audit_seed_first"], 2026080201
        )
        self.assertEqual(
            self.family["schedule"]["audit_seed_last"], 2026080300
        )
        self.assertEqual(self.family["schedule"]["audit_seed_count"], 100)
        self.assertEqual(
            self.family["perturbation"],
            {
                "method": "sha256-first8-uint64-be-affine-v1",
                "coordinate_radius_m": 0.1,
                "clamp": False,
                "resample": False,
            },
        )
        self.assertEqual(
            family_semantic_sha256(self.raw), self.raw["semantic_sha256"]
        )
        exported = {
            name.lower()
            for name in __import__(
                "scripts.diagnostics.qualified_initial_state",
                fromlist=["*"],
            ).__dict__
            if not name.startswith("_")
        }
        self.assertFalse(any("resample" in name or "retry" in name for name in exported))

    def test_sha256_perturbation_and_position_hash_are_exact(self):
        positions = materialize_seed_positions(
            self.family, 2026080201
        )

        self.assertEqual(
            positions[0],
            (-1405.6658757438072, -181.63620300472508),
        )
        self.assertEqual(
            positions[-1],
            (-1454.354970132772, 181.66691776537996),
        )
        self.assertEqual(
            canonical_positions_sha256(positions),
            "161fb8a9104b7b5b0a3a20cd5cf0e9c896db98e74ee2262088751edddaa72e88",
        )
        template = self.family["template_positions_m"]
        for actual, nominal in zip(positions, template):
            self.assertLessEqual(abs(actual[0] - nominal[0]), 0.1)
            self.assertLessEqual(abs(actual[1] - nominal[1]), 0.1)
        self.assertEqual(
            positions,
            materialize_seed_positions(
                self.family, 2026080201
            ),
        )
        with self.assertRaisesRegex(ValueError, "not a registered trajectory seed"):
            materialize_seed_positions(
                self.family, 2026080211
            )

    def test_loader_rejects_duplicate_json_keys_before_semantic_validation(self):
        source = FAMILY_PATH.read_text(encoding="utf-8")
        duplicate = source.replace(
            '  "namespace": "cbf2026-v5-initial",',
            '  "namespace": "tampered-first",\n'
            '  "namespace": "cbf2026-v5-initial",',
            1,
        )
        self.assertNotEqual(source, duplicate)
        with tempfile.TemporaryDirectory(
            prefix="qualified-initial-duplicate-"
        ) as directory:
            path = Path(directory) / "duplicate.json"
            path.write_text(duplicate, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "duplicate JSON key"):
                load_qualified_initial_family(path)

    def test_schema_and_every_frozen_contract_tamper_are_rejected(self):
        mutations = []

        def mutate(path, value, *, delete=False):
            candidate = copy.deepcopy(self.raw)
            target = candidate
            for key in path[:-1]:
                target = target[key]
            if delete:
                del target[path[-1]]
            else:
                target[path[-1]] = value
            candidate["semantic_sha256"] = family_semantic_sha256(candidate)
            mutations.append(candidate)

        mutate(("namespace",), "cbf2026-v5-initial-tampered")
        mutate(("template_positions_m", 0, 0), -1405.0)
        mutate(("perturbation", "coordinate_radius_m"), 0.2)
        mutate(("perturbation", "coordinate_radius_m"), True)
        mutate(("perturbation", "clamp"), 0)
        mutate(("admission", "minimum_barrier_m"), 34.0)
        mutate(("admission", "minimum_qp_margin_mps"), True)
        mutate(("schedule", "registered_trajectory_seeds", 0), 2026080999)
        mutate(("schedule", "audit_seed_count"), True)
        mutate(
            ("frozen_summary", "registered", "minimum_barrier_m"),
            35.8,
        )
        mutate(
            ("frozen_summary", "audit", "minimum_pair_distance_edge"),
            [True, 7],
        )
        mutate(
            ("production_contract", "planar_component_max_mps"), True
        )
        mutate(("schema_version",), None, delete=True)
        extra = copy.deepcopy(self.raw)
        extra["unexpected"] = True
        extra["semantic_sha256"] = family_semantic_sha256(extra)
        mutations.append(extra)

        for candidate in mutations:
            with self.subTest(candidate=candidate):
                with self.assertRaises(ValueError):
                    validate_qualified_initial_family(candidate)

        bad_hash = copy.deepcopy(self.raw)
        bad_hash["semantic_sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "semantic SHA-256"):
            validate_qualified_initial_family(bad_hash)


class QualifiedInitialGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.family = load_qualified_initial_family(FAMILY_PATH)
        start = time.perf_counter()
        cls.frozen = audit_frozen_initial_family(cls.family)
        cls.hundred = cls.frozen.audit
        cls.hundred_seconds = time.perf_counter() - start
        cls.registered = cls.frozen.registered

    def test_fixed_cbf_edges_and_dynamic_fim_references_are_separate(self):
        fixed = fixed_localization_edges(self.family)
        collisions = collision_edges(self.family)
        positions = materialize_seed_positions(
            self.family, 2026080201
        )

        self.assertEqual(len(fixed), 28)
        self.assertEqual(len(collisions), 91)
        self.assertEqual(
            [
                edge.reference_token()
                for edge in fixed
                if edge.low == 1 and edge.high == 1
            ],
            [("base", 0), ("base", 1)],
        )
        self.assertEqual(
            dynamic_fim_reference_ids(self.family, positions, 1),
            (("base", 0), ("base", 1), ("base", 2)),
        )
        self.assertEqual(
            dynamic_fim_reference_ids(self.family, positions, 3),
            (
                ("base", 0),
                ("base", 1),
                ("base", 2),
                ("uav", 1),
                ("uav", 2),
            ),
        )

    def test_v4_frame_zero_reproduces_uav3_failure_and_is_rejected(self):
        audit = audit_positions(self.family, V4_FRAME_ZERO_POSITIONS)
        uav3 = audit.certificates[2]
        qp3 = audit.local_qps[2]

        self.assertAlmostEqual(uav3.epsilon, 1.387872443304371, places=12)
        self.assertAlmostEqual(uav3.bar_nu, 5.154384235458222, places=12)
        self.assertAlmostEqual(qp3.margin, -2.243905861169388, places=12)
        self.assertEqual(
            [row.edge.token() for row in qp3.tight_rows],
            [
                ("collision", 3, 5, -1),
                ("collision", 3, 14, -1),
                ("collision", 3, 8, -1),
            ],
        )
        self.assertFalse(audit.accepted)
        self.assertTrue(any(reason.startswith("qp_margin:") for reason in audit.reasons))
        with self.assertRaises(TypeError):
            require_admitted_seed(
                self.family, 2026080201, positions=V4_FRAME_ZERO_POSITIONS
            )

    def test_registered_ten_are_exactly_ten_of_ten_with_frozen_worst_cases(self):
        summary = self.registered.summary

        self.assertEqual(summary.proposed_count, 10)
        self.assertEqual(summary.accepted_count, 10)
        self.assertEqual(summary.rejected_seeds, ())
        self.assertAlmostEqual(summary.minimum_barrier.value, 35.77296640879953)
        self.assertEqual(summary.minimum_barrier.seed, 2026080205)
        self.assertEqual(
            summary.minimum_barrier.edge.token(),
            ("collision", 2, 14, -1),
        )
        self.assertAlmostEqual(summary.minimum_qp_margin.value, 0.7658252531927233)
        self.assertEqual(summary.minimum_qp_margin.seed, 2026080207)
        self.assertEqual(summary.minimum_qp_margin.robot_id, 13)
        self.assertEqual(
            [edge.token() for edge in summary.minimum_qp_margin.active_edges],
            [
                ("collision", 4, 13, -1),
                ("collision", 12, 13, -1),
            ],
        )
        self.assertAlmostEqual(summary.maximum_bar_nu.value, 6.768799189292255)
        self.assertEqual(summary.maximum_bar_nu.seed, 2026080204)
        self.assertEqual(summary.maximum_bar_nu.robot_id, 14)

    def test_audit_universe_is_100_of_100_and_has_frozen_worst_cases(self):
        summary = self.hundred.summary

        self.assertEqual(summary.proposed_count, 100)
        self.assertEqual(summary.accepted_count, 100)
        self.assertEqual(summary.rejected_seeds, ())
        self.assertAlmostEqual(summary.minimum_barrier.value, 35.77296640879953)
        self.assertEqual(summary.minimum_barrier.seed, 2026080205)
        self.assertAlmostEqual(summary.minimum_qp_margin.value, 0.7658252531927233)
        self.assertEqual(summary.minimum_qp_margin.seed, 2026080207)
        self.assertEqual(summary.minimum_qp_margin.robot_id, 13)
        self.assertAlmostEqual(summary.maximum_bar_nu.value, 6.7773637849535655)
        self.assertEqual(summary.maximum_bar_nu.seed, 2026080253)
        self.assertEqual(summary.maximum_bar_nu.robot_id, 14)
        print(f"qualified-initial-state 100-seed audit: {self.hundred_seconds:.3f}s")

    def test_conflict_geometry_is_rejected_without_seed_substitution(self):
        rejected = audit_positions(self.family, V4_FRAME_ZERO_POSITIONS)

        self.assertIsNone(rejected.seed)
        self.assertFalse(rejected.accepted)
        self.assertEqual(rejected.positions, V4_FRAME_ZERO_POSITIONS)
        self.assertNotEqual(
            rejected.positions,
            materialize_seed_positions(self.family, 2026080201),
        )
        seed_two_positions = materialize_seed_positions(
            self.family, 2026080202
        )
        with self.assertRaises(TypeError):
            audit_seed(
                self.family, 2026080201, positions=seed_two_positions
            )
        with self.assertRaises(ValueError):
            audit_seed(self.family, 2026080999)


if __name__ == "__main__":
    unittest.main()
