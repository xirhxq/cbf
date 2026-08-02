import copy
import hashlib
import json
import struct
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from scripts.diagnostics import qualified_v6_initial_state as v6
from scripts.diagnostics.qualified_initial_state import (
    _materialize_seed_positions,
    InitialStateAdmissionError,
    load_qualified_initial_family,
)
from scripts.diagnostics.hard_interior_selection import frozen_interior_floor
from scripts.diagnostics.qualified_v6_initial_state import (
    audit_frozen_v6_initial_family,
    load_qualified_v6_initial_family,
    materialize_v6_seed_positions,
    reconstruct_v6_one_step_state,
    validate_qualified_v6_initial_family,
    v6_family_semantic_sha256,
)


ROOT = Path(__file__).resolve().parents[1]
V1_PATH = ROOT / "config/diagnostics/qualified_initial_family_v1.json"
V2_PATH = ROOT / "config/diagnostics/qualified_initial_family_v2.json"
V3_PATH = ROOT / "config/diagnostics/qualified_initial_family_v3.json"
V2_PRIMARY_PATH = ROOT / "config/diagnostics/qualified_mode_hybrid_dcbf_development_v2.json"
V2_ABLATION_PATH = ROOT / "config/diagnostics/qualified_mode_hybrid_dcbf_fixed_fim_ablation_v2.json"
HISTORICAL_V2_SHA256 = {
    V2_PRIMARY_PATH: "ad71ca5d3e7580022b7af4d8f21767aff74dba9c0edb4d347c3c7f174614382d",
    V2_ABLATION_PATH: "13d5a3f2dcf41ce99579c342c64008c8e915b9ff7c91bb028eb98737a958372c",
    V2_PATH: "21d04b79e9e81ba867e28826ad43615120a4889d16e082d602e933a6a73177ef",
}

# Immutable retained development-v5 Mission-01 frame-zero evidence:
# /private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v5/
# mission-01/swarm.jsonl.gz, SHA-256 below.  The fixture is copied here so this
# mathematical regression never launches or depends on the historical binary.
V5_COUNTEREXAMPLE_PROVENANCE = {
    "campaign_id": "development-v5",
    "condition": "dynamic_primary",
    "trajectory_seed": 2026080201,
    "range_noise_seed": 2026081201,
    "frame_index": 0,
    "schema_version": "cbf2026-qualified-evidence-v1",
    "source_sha256": "4fb1ccfd3fbeb8168006f6d86d5a12350ab0dab43bbcab105aea1542d3c1be65",
}
V5_FRAME_ZERO_PLANAR_COMMANDS_BY_ROBOT = (
    (1, (-1.1684117246282746, 2.4153004576023847)),
    (2, (-0.6329247324648044, -1.6283379264469815)),
    (3, (-4.011248209808141, -0.9719565315104042)),
    (4, (-3.551592436649411, 0.3603419279486495)),
    (5, (12.847260069533, -3.392015743447832)),
    (6, (-11.361581193522856, -3.0610384105729906)),
    (7, (-3.501930991539261, -4.372419114666091)),
    (8, (-3.514287953519581, 0.7314340743815393)),
    (9, (-3.6250519541692654, 0.5136528899624722)),
    (10, (-10.983935873377126, 4.982867013103082)),
    (11, (-2.743104460071617, 0.34624720143649057)),
    (12, (-3.9203214897186136, -0.40654497881470775)),
    (13, (-7.3908383141473095, 0.0026764753358534676)),
    (14, (-4.952201068461996, 24.18197080343183)),
)


class QualifiedV6InitialFamilyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.v1 = load_qualified_initial_family(V1_PATH)
        cls.v2 = load_qualified_v6_initial_family(V2_PATH)
        cls.raw = json.loads(V2_PATH.read_text(encoding="utf-8"))

    def v3_candidate(self):
        candidate = copy.deepcopy(self.raw)
        candidate["schema_version"] = "cbf2026-qualified-initial-family-v3"
        candidate["controller_policy"] = {
            "schema_version": "hard-interior-v3",
            "class_k_coefficient": 0.1,
            "class_k_power": 1,
            "mode": "planar-chebyshev-fraction-cap-v2",
            "fraction": 0.131,
            "cap_mps": 0.1,
            "feasibility_tolerance_mps": 1e-9,
            "planar_component_max_mps": 25.0,
            "yaw_enters_radius": False,
        }
        candidate["semantic_sha256"] = v6_family_semantic_sha256(candidate)
        return candidate

    def test_v3_family_accepts_only_declared_additions_and_canonical_hash(self):
        candidate = self.v3_candidate()
        try:
            checked = validate_qualified_v6_initial_family(candidate)
        except ValueError as error:
            self.fail(f"exact v3 family was rejected: {error}")
        self.assertEqual(checked, candidate)
        self.assertEqual(candidate["namespace"], self.v2["namespace"])
        for key in candidate:
            if key not in {"schema_version", "controller_policy", "semantic_sha256"}:
                self.assertEqual(candidate[key], self.v2[key], key)
        self.assertEqual(
            candidate["semantic_sha256"], v6_family_semantic_sha256(candidate)
        )

    def test_v3_family_rejects_policy_marker_mode_fraction_and_token_mutations(self):
        mutations = (
            (("schema_version",), ["cbf2026-qualified-initial-family-v3"]),
            (("schema_version",), {"value": "cbf2026-qualified-initial-family-v3"}),
            (("schema_version",), True),
            (("controller_policy", "schema_version"), "hard-interior-v2"),
            (("controller_policy", "schema_version"), ["hard-interior-v3"]),
            (("controller_policy", "schema_version"), {"value": "hard-interior-v3"}),
            (("controller_policy", "schema_version"), True),
            (("controller_policy", "mode"), "planar-chebyshev-fraction-cap-v1"),
            (("controller_policy", "fraction"), 0.13),
            (("controller_policy", "fraction"), 0.132),
            (("controller_policy", "fraction"), 131),
            (("controller_policy", "fraction"), True),
            (("controller_policy", "cap_mps"), 0),
            (("controller_policy", "feasibility_tolerance_mps"), 0),
        )
        for path, value in mutations:
            with self.subTest(path=path, value=value):
                candidate = self.v3_candidate()
                target = candidate
                for key in path[:-1]:
                    target = target[key]
                target[path[-1]] = value
                candidate["semantic_sha256"] = v6_family_semantic_sha256(candidate)
                with self.assertRaises(ValueError):
                    validate_qualified_v6_initial_family(candidate)

        extra = self.v3_candidate()
        extra["controller_policy"]["extra"] = True
        extra["semantic_sha256"] = v6_family_semantic_sha256(extra)
        with self.assertRaises(ValueError):
            validate_qualified_v6_initial_family(extra)
        missing = self.v3_candidate()
        del missing["controller_policy"]["feasibility_tolerance_mps"]
        missing["semantic_sha256"] = v6_family_semantic_sha256(missing)
        with self.assertRaises(ValueError):
            validate_qualified_v6_initial_family(missing)

    def test_v3_positions_are_ieee_identical_and_historical_v2_bytes_are_frozen(self):
        self.assertTrue(V3_PATH.exists())
        if not V3_PATH.exists():
            return
        v3_family = load_qualified_v6_initial_family(V3_PATH)
        for path, expected in HISTORICAL_V2_SHA256.items():
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), expected)
        for seed in range(2026080201, 2026080301):
            with self.subTest(seed=seed):
                v1_positions = _materialize_seed_positions(self.v1, seed)
                v2_positions = materialize_v6_seed_positions(self.v2, seed)
                v3_positions = materialize_v6_seed_positions(v3_family, seed)
                self.assertEqual(
                    tuple(struct.pack(">dd", *point) for point in v3_positions),
                    tuple(struct.pack(">dd", *point) for point in v2_positions),
                )
                self.assertEqual(v3_positions, v1_positions)

    def test_registered_v2_policy_saturates_all_1400_frozen_current_problems(self):
        candidate = self.v3_candidate()
        try:
            frozen = audit_frozen_v6_initial_family(candidate)
        except ValueError as error:
            self.fail(f"v3 frozen-universe reconstruction was rejected: {error}")
        radii = [
            qp.margin
            for seed_audit in frozen.audit.audits
            for qp in seed_audit.local_qps
        ]
        self.assertEqual(len(radii), 1400)
        self.assertAlmostEqual(min(radii), 0.7658252531927233, places=12)
        registered_floors = [
            frozen_interior_floor(
                radius,
                fraction=0.131,
                cap_mps=0.1,
                tolerance_mps=1e-9,
            )
            for radius in radii
        ]
        historical_floors = [
            frozen_interior_floor(
                radius,
                fraction=0.1,
                cap_mps=0.1,
                tolerance_mps=1e-9,
            )
            for radius in radii
        ]
        self.assertTrue(all(floor == 0.1 for floor in registered_floors))
        self.assertTrue(all(floor <= radius for floor, radius in zip(registered_floors, radii)))
        self.assertAlmostEqual(min(historical_floors), 0.07658252521927234, places=14)

    def test_audit_positions_are_bitwise_identical_to_v1(self):
        for seed in range(2026080201, 2026080301):
            with self.subTest(seed=seed):
                self.assertEqual(
                    materialize_v6_seed_positions(self.v2, seed),
                    _materialize_seed_positions(self.v1, seed),
                )

    def test_registered_prefix_is_unchanged_without_selection(self):
        self.assertEqual(
            self.v2["schedule"]["registered_trajectory_seeds"],
            list(range(2026080201, 2026080211)),
        )
        self.assertFalse(self.v2["perturbation"]["clamp"])
        self.assertFalse(self.v2["perturbation"]["resample"])
        self.assertFalse(self.v2["one_step_gate"]["clamp"])
        self.assertFalse(self.v2["one_step_gate"]["resample"])
        self.assertFalse(self.v2["one_step_gate"]["retry"])

    def test_v2_repeats_every_v1_static_value_except_its_declared_additions(self):
        static_v2 = {
            key: value for key, value in self.v2.items()
            if key not in {"schema_version", "namespace", "semantic_sha256", "controller_policy", "one_step_gate"}
        }
        static_v1 = {
            key: value for key, value in self.v1.items()
            if key not in {"schema_version", "namespace", "semantic_sha256"}
        }
        self.assertEqual(static_v2, static_v1)

    def test_schema_freezes_all_v6_contract_mutations(self):
        mutations = []

        def mutate(path, value):
            candidate = copy.deepcopy(self.raw)
            target = candidate
            for key in path[:-1]:
                target = target[key]
            target[path[-1]] = value
            candidate["semantic_sha256"] = v6_family_semantic_sha256(candidate)
            mutations.append(candidate)

        mutate(("template_positions_m", 0, 0), -1405.0)
        mutate(("perturbation", "method"), "different")
        mutate(("perturbation", "coordinate_radius_m"), 0.2)
        mutate(("perturbation", "clamp"), True)
        mutate(("perturbation", "resample"), True)
        mutate(("schedule", "registered_trajectory_seeds", 0), 2026080211)
        mutate(("schedule", "audit_seed_first"), 2026080202)
        mutate(("schedule", "audit_seed_last"), 2026080299)
        mutate(("schedule", "audit_seed_count"), 99)
        mutate(("production_contract", "class_k_coefficient"), 0.2)
        mutate(("admission", "fixed_localization_edge_count"), 27)
        mutate(("admission", "collision_edge_count"), 90)
        mutate(("admission", "barrier_count"), 118)
        mutate(("admission", "endpoint_row_count"), 231)
        mutate(("admission", "local_qp_count"), 13)
        mutate(("production_contract", "planar_component_max_mps"), 24.0)
        mutate(("controller_policy", "class_k_coefficient"), 0.2)
        mutate(("controller_policy", "class_k_power"), 2)
        mutate(("controller_policy", "mode"), "other")
        mutate(("controller_policy", "fraction"), 0.2)
        mutate(("controller_policy", "cap_mps"), 0.2)
        mutate(("controller_policy", "planar_component_max_mps"), 24.0)
        mutate(("controller_policy", "yaw_enters_radius"), True)
        mutate(("one_step_gate", "dt_s"), 0.25)
        mutate(("one_step_gate", "barrier_comparison"), "greater-or-equal")
        mutate(("one_step_gate", "minimum_next_barrier_m"), 0.1)
        mutate(("one_step_gate", "minimum_next_local_radius_mps"), 0.04)
        mutate(("one_step_gate", "clamp"), True)
        mutate(("one_step_gate", "resample"), True)
        mutate(("one_step_gate", "retry"), True)
        for candidate in mutations:
            with self.subTest(candidate=candidate):
                with self.assertRaises(ValueError):
                    validate_qualified_v6_initial_family(candidate)

    def test_v1_static_projection_requires_exact_recursive_json_token_types(self):
        mutations = (
            (("template_positions_m", 2, 1), 0),
            (("production_contract", "planar_component_max_mps"), 25),
            (("perturbation", "clamp"), 0),
            (("production_contract", "formation_base_ids", 0, 0), True),
            (("schedule", "audit_seed_count"), 100.0),
            (("template_positions_m", 0), tuple(self.raw["template_positions_m"][0])),
            (("frozen_summary", "audit", "minimum_barrier_edge"),
             tuple(self.raw["frozen_summary"]["audit"]["minimum_barrier_edge"])),
        )
        for path, value in mutations:
            with self.subTest(path=path, value=value):
                candidate = copy.deepcopy(self.raw)
                target = candidate
                for key in path[:-1]:
                    target = target[key]
                target[path[-1]] = value
                candidate["semantic_sha256"] = v6_family_semantic_sha256(candidate)
                with self.assertRaises(ValueError):
                    validate_qualified_v6_initial_family(candidate)

    def test_v1_static_projection_rejects_signed_zero_bit_substitution(self):
        candidate = copy.deepcopy(self.raw)
        candidate["template_positions_m"][2][1] = -0.0
        candidate["semantic_sha256"] = v6_family_semantic_sha256(candidate)
        with self.assertRaises(ValueError):
            validate_qualified_v6_initial_family(candidate)

    def test_v1_static_oracle_uses_only_the_hash_checked_byte_buffer(self):
        contradictory_second_read = copy.deepcopy(self.v1)
        contradictory_second_read["template_positions_m"][2][1] = 1.0
        with mock.patch.object(
            v6.v1,
            "load_qualified_initial_family",
            return_value=contradictory_second_read,
        ):
            checked = validate_qualified_v6_initial_family(self.raw)
        self.assertEqual(checked, self.raw)

    def test_duplicate_keys_and_alternate_identity_fail_closed(self):
        source = V2_PATH.read_text(encoding="utf-8")
        duplicate = source.replace(
            '  "namespace": "cbf2026-v6-initial",',
            '  "namespace": "wrong-first",\n'
            '  "namespace": "cbf2026-v6-initial",',
            1,
        )
        with tempfile.TemporaryDirectory(prefix="qualified-v6-") as directory:
            temporary = Path(directory) / "family.json"
            temporary.write_text(duplicate, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "duplicate JSON key"):
                load_qualified_v6_initial_family(temporary)
            temporary.write_text(source, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "provenance"):
                load_qualified_v6_initial_family(temporary)

    def test_family_declares_later_gate_without_a_passing_claim(self):
        self.assertEqual(
            self.raw["semantic_sha256"], v6_family_semantic_sha256(self.raw)
        )
        self.assertNotIn("passed", self.raw["one_step_gate"])
        self.assertNotIn("status", self.raw["one_step_gate"])
        self.assertNotIn("result", self.raw["one_step_gate"])

    def test_frozen_audit_reconstructs_the_unchanged_universe(self):
        frozen = audit_frozen_v6_initial_family(self.v2)
        self.assertEqual(frozen.audit.summary.proposed_count, 100)
        self.assertEqual(frozen.audit.summary.accepted_count, 100)
        self.assertEqual(frozen.registered.summary.proposed_count, 10)
        self.assertEqual(frozen.registered.summary.accepted_count, 10)


class OneStepReconstructionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.family = load_qualified_v6_initial_family(V2_PATH)
        cls.positions = materialize_v6_seed_positions(cls.family, 2026080201)

    def test_reconstruction_applies_single_integrator_before_next_certificates(self):
        commands = tuple((0.0, 0.0) for _ in range(14))
        state = reconstruct_v6_one_step_state(self.family, self.positions, commands)
        self.assertEqual(state.current_positions_m, self.positions)
        self.assertTrue(state.current_audit.accepted)
        self.assertEqual(state.applied_planar_commands_mps, commands)
        self.assertEqual(state.next_positions_m, self.positions)
        self.assertEqual(len(state.next_barriers), 119)
        self.assertEqual(len(state.next_local_radii_mps), 14)
        self.assertFalse(hasattr(state, "passed"))
        self.assertEqual(
            state.chronology,
            (
                "validate-current-state",
                "apply-single-integrator",
                "recompute-next-certificates",
                "recompute-next-hard-rows",
                "solve-next-planar-radii",
            ),
        )

    def test_reconstruction_rejects_command_bound_and_producer_boolean(self):
        commands = tuple((0.0, 0.0) for _ in range(14))
        with self.assertRaises(ValueError):
            reconstruct_v6_one_step_state(
                self.family, self.positions, ((25.1, 0.0), *commands[1:])
            )
        with self.assertRaises(TypeError):
            reconstruct_v6_one_step_state(
                self.family, self.positions, commands, producer_passed=True
            )

    def test_reconstruction_rejects_unadmitted_current_state_before_update(self):
        shifted = list(self.positions)
        shifted[2] = (shifted[2][0] + 2.0, shifted[2][1])
        commands = tuple((0.0, 0.0) for _ in range(14))
        with self.assertRaises(InitialStateAdmissionError) as caught:
            reconstruct_v6_one_step_state(self.family, shifted, commands)
        self.assertIn("deployment:3", caught.exception.audit.reasons)

    def test_historical_v5_frame_zero_commands_reproduce_one_step_failure(self):
        self.assertEqual(
            tuple(robot_id for robot_id, _ in V5_FRAME_ZERO_PLANAR_COMMANDS_BY_ROBOT),
            tuple(range(1, 15)),
        )
        self.assertEqual(V5_COUNTEREXAMPLE_PROVENANCE["campaign_id"], "development-v5")
        commands = tuple(
            command for _, command in V5_FRAME_ZERO_PLANAR_COMMANDS_BY_ROBOT
        )
        state = reconstruct_v6_one_step_state(self.family, self.positions, commands)
        minimum_barrier = min(state.next_barriers, key=lambda item: item.value)
        self.assertAlmostEqual(minimum_barrier.value, 37.24931436014771, places=12)
        self.assertEqual(minimum_barrier.edge.token(), ("collision", 1, 7, -1))
        self.assertAlmostEqual(
            state.next_local_radii_mps[3], -0.017308452413388856, places=12
        )
        self.assertAlmostEqual(
            state.next_local_radii_mps[11], -0.044958268439302146, places=12
        )
        self.assertFalse(hasattr(state, "passed"))


if __name__ == "__main__":
    unittest.main()
