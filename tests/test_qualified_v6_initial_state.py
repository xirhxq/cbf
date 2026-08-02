import copy
import json
import tempfile
import unittest
from pathlib import Path

from scripts.diagnostics.qualified_initial_state import (
    _materialize_seed_positions,
    load_qualified_initial_family,
)
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


class QualifiedV6InitialFamilyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.v1 = load_qualified_initial_family(V1_PATH)
        cls.v2 = load_qualified_v6_initial_family(V2_PATH)
        cls.raw = json.loads(V2_PATH.read_text(encoding="utf-8"))

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


if __name__ == "__main__":
    unittest.main()
