from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import shutil
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import scripts.diagnostics.audit_qualified_v6_one_step_viability as producer
from scripts.diagnostics import qualified_initial_state as v1_initial
from scripts.diagnostics import qualified_v6_initial_state as v6_initial
from scripts.diagnostics.audit_qualified_v6_one_step_viability import (
    ChildLaunchFailure,
    SubprocessOneStepOperations,
    _audit_seed_sequence,
    _evaluate_predicate,
    audit_one_step_universe,
)
from scripts.diagnostics.hard_interior_selection import (
    frozen_interior_floor,
    solve_planar_hard_row_chebyshev,
)


ROOT = Path(__file__).resolve().parents[1]
FROZEN_SEEDS = list(range(2026080201, 2026080301))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def assert_terminal_predicate_consistent(
    case: unittest.TestCase, result: dict
) -> None:
    """Check terminal semantics from frozen thresholds, never a producer boolean."""
    launched = [row for row in result["seed_results"] if row["launched"]]
    case.assertEqual(result["launch_count"], len(launched))
    for row in launched:
        if "maximum_planar_component_mps" not in row:
            continue
        expected_reasons = []
        if row["maximum_planar_component_mps"] > 25.0 + 1e-7:
            expected_reasons.append("component_bound_violation")
        if (
            row["minimum_applied_original_residual_mps"]
            < row["minimum_enforced_floor_mps"] - 1e-7
        ):
            expected_reasons.append("applied_residual_below_floor")
        if row["minimum_next_barrier_m"] <= 0.0:
            expected_reasons.append("nonpositive_next_barrier")
        if row["minimum_next_local_radius_mps"] < 0.05:
            expected_reasons.append("small_next_local_radius")
        case.assertEqual(row["reasons"], expected_reasons)
        case.assertEqual(row["passed"], not expected_reasons)
        case.assertEqual(row["status"], "passed" if not expected_reasons else "failed")
    expected_predicate_pass = (
        result["launch_count"] == len(result["seed_universe"])
        and all(row["passed"] for row in result["seed_results"])
    )
    case.assertEqual(result["predicate_passed"], expected_predicate_pass)
    if result["qualifying"]:
        case.assertEqual(result["passed"], expected_predicate_pass)
        case.assertEqual(
            result["status"], "completed" if expected_predicate_pass else "failed"
        )
        case.assertEqual(
            result["reason"],
            "completed" if expected_predicate_pass else "predicate_failed",
        )
    else:
        case.assertFalse(result["passed"])
        case.assertEqual(result["status"], "test_only")
        case.assertEqual(result["reason"], "injected_operations_nonqualifying")


def _edge_dict(edge) -> dict:
    return {
        "kind": edge.kind,
        "low": edge.low,
        "high": edge.high,
        "base_id": edge.base_id,
    }


def _certificate_dict(certificate) -> dict:
    return {
        "robot_id": certificate.robot_id,
        "reference_ids": [
            {"kind": kind, "id": reference_id}
            for kind, reference_id in certificate.reference_ids
        ],
        "covariance": [list(row) for row in certificate.covariance],
        "covariance_rate_bound": certificate.covariance_rate_bound,
        "epsilon": certificate.epsilon,
        "bar_nu": certificate.bar_nu,
    }


def assert_retained_evidence_recomputes(
    case: unittest.TestCase, seed_result: dict, family: dict
) -> None:
    """Independent Task 4/5 round trip; no producer reconstruction helper."""
    retained = seed_result["recomputation_evidence"]
    positions = tuple(tuple(point) for point in retained["current_positions_m"])
    robots = retained["robots"]
    case.assertEqual([robot["robot_id"] for robot in robots], list(range(1, 15)))
    case.assertEqual(
        seed_result["positions_sha256"],
        v1_initial.canonical_positions_sha256(positions),
    )
    commands = tuple(tuple(robot["applied_command"]) for robot in robots)
    all_pairs = []
    for robot in robots:
        policy = robot["hard_interior_selection"]
        problem = robot["normal_problem"]
        audit = solve_planar_hard_row_chebyshev(
            problem, tolerance_mps=policy["feasibility_tolerance_mps"]
        )
        expected_floor = frozen_interior_floor(
            audit.radius_mps,
            fraction=policy["fraction"],
            cap_mps=policy["cap_mps"],
            tolerance_mps=policy["feasibility_tolerance_mps"],
        )
        case.assertAlmostEqual(
            policy["planar_chebyshev_radius_mps"], audit.radius_mps, delta=1e-12
        )
        case.assertAlmostEqual(policy["enforced_floor_mps"], expected_floor, delta=1e-12)
        residuals = [
            row["constant"]
            + sum(
                coefficient * component
                for coefficient, component in zip(
                    row["coefficients"], robot["applied_command"]
                )
            )
            for row in problem["rows"]
        ]
        case.assertAlmostEqual(
            policy["minimum_original_hard_residual_mps"],
            min(residuals),
            delta=1e-9,
        )
        all_pairs.extend((residual, expected_floor) for residual in residuals)
    next_positions = tuple(
        (point[0] + 0.5 * command[0], point[1] + 0.5 * command[1])
        for point, command in zip(positions, commands)
    )
    case.assertEqual(retained["next_positions_m"], [list(point) for point in next_positions])
    checked = v6_initial.validate_qualified_v6_initial_family(family)
    legacy = v6_initial._legacy_v1_family(checked)
    certificates = v1_initial._compute_certificates(legacy, next_positions)
    case.assertEqual(
        retained["next_dynamic_fim_certificates"],
        [_certificate_dict(certificate) for certificate in certificates],
    )
    barriers, rows = v1_initial._barriers_and_rows(
        legacy, next_positions, certificates
    )
    case.assertEqual(
        retained["next_barriers"],
        [{"edge": _edge_dict(barrier.edge), "value_m": barrier.value} for barrier in barriers],
    )
    local_rows = tuple(
        tuple(row for row in rows if row.owner == robot_id)
        for robot_id in range(1, 15)
    )
    expected_problems = [
        v6_initial._local_hard_problem(
            robot_id,
            local_rows[robot_id - 1],
            checked["controller_policy"]["planar_component_max_mps"],
        )
        for robot_id in range(1, 15)
    ]
    expected_radii = [
        solve_planar_hard_row_chebyshev(problem).radius_mps
        for problem in expected_problems
    ]
    case.assertEqual(
        [entry["problem"] for entry in retained["next_local_problems"]],
        expected_problems,
    )
    case.assertEqual(
        [entry["radius_mps"] for entry in retained["next_local_problems"]],
        expected_radii,
    )
    expected_reasons = []
    if any(abs(component) > 25.0 + 1e-7 for command in commands for component in command[:2]):
        expected_reasons.append("component_bound_violation")
    if any(residual < floor - 1e-7 for residual, floor in all_pairs):
        expected_reasons.append("applied_residual_below_floor")
    if any(barrier.value <= 0.0 for barrier in barriers):
        expected_reasons.append("nonpositive_next_barrier")
    if any(radius < 0.05 for radius in expected_radii):
        expected_reasons.append("small_next_local_radius")
    case.assertEqual(seed_result["reasons"], expected_reasons)
    case.assertEqual(seed_result["passed"], not expected_reasons)


def _normal_problem(robot_id: int) -> dict:
    return {
        "owner": robot_id,
        "control_size": 3,
        "planar_component_max": 25.0,
        "yaw_rate_max": 0.35,
        "snapshot_version": 1,
        "allocation_version": 1,
        "bounds": [
            {"control_index": 0, "coefficient": 1.0, "limit": 25.0},
            {"control_index": 0, "coefficient": -1.0, "limit": 25.0},
            {"control_index": 1, "coefficient": 1.0, "limit": 25.0},
            {"control_index": 1, "coefficient": -1.0, "limit": 25.0},
            {"control_index": 2, "coefficient": 1.0, "limit": 0.35},
            {"control_index": 2, "coefficient": -1.0, "limit": 0.35},
        ],
        "rows": [
            {
                "edge": {
                    "kind": "collision",
                    "low": 1,
                    "high": 2,
                    "base_id": -1,
                },
                "owner": robot_id,
                "name": "unit-hard-row",
                "coefficients": [1.0, 0.0, 0.0],
                "constant": 1.0,
                "post_reset_barrier": 1.0,
                "snapshot_version": 1,
                "allocation_version": 1,
            }
        ],
        "hard_problem_id": f"unit-uav-{robot_id}",
    }


def _policy() -> dict:
    return {
        "schema_version": "hard-interior-v3",
        "mode": "planar-chebyshev-fraction-cap-v2",
        "fraction": 0.131,
        "cap_mps": 0.1,
        "feasibility_tolerance_mps": 1e-9,
        "planar_chebyshev_radius_mps": 26.0,
        "enforced_floor_mps": 0.1,
        "minimum_original_hard_residual_mps": 1.0,
    }


def _fast_reconstruction(*, barriers=None, radii=None) -> dict:
    barriers = barriers or tuple(
        SimpleNamespace(
            value=1.0,
            edge=SimpleNamespace(kind="collision", low=1, high=2, base_id=-1),
        )
        for _ in range(119)
    )
    barriers = tuple(
        barrier
        if hasattr(barrier, "edge")
        else SimpleNamespace(
            value=barrier.value,
            edge=SimpleNamespace(kind="collision", low=1, high=2, base_id=-1),
        )
        for barrier in barriers
    )
    radii = radii or (0.05,) * 14
    return {
        "current_audit": SimpleNamespace(positions_sha256="0" * 64),
        "next_positions_m": ((0.0, 0.0),) * 14,
        "next_certificates": tuple(
            SimpleNamespace(
                robot_id=robot_id,
                reference_ids=(("base", 0), ("base", 1)),
                covariance=((1.0, 0.0), (0.0, 1.0)),
                covariance_rate_bound=1.0,
                epsilon=1.0,
                bar_nu=1.0,
            )
            for robot_id in range(1, 15)
        ),
        "next_barriers": barriers,
        "next_local_problems": tuple(
            _normal_problem(robot_id) for robot_id in range(1, 15)
        ),
        "next_radii_mps": radii,
    }


class FakeOperations:
    def __init__(
        self,
        *,
        interrupt: bool = False,
        error: Exception | None = None,
        mutate_result=None,
        during_launch=None,
    ):
        self.interrupt = interrupt
        self.error = error
        self.mutate_result = mutate_result
        self.during_launch = during_launch
        self.launched_seeds: list[int] = []
        self.config_paths: list[Path] = []
        self.configs: list[dict] = []
        self.claim: Path | None = None
        self.claim_bytes: bytes | None = None

    def launch_seed(self, *, seed: int, config_path: Path):
        self.launched_seeds.append(seed)
        self.config_paths.append(config_path)
        if self.claim is not None:
            if not self.claim.is_file():
                raise AssertionError("claim must exist before launch")
            observed_claim = self.claim.read_bytes()
            if self.claim_bytes is None:
                self.claim_bytes = observed_claim
            elif observed_claim != self.claim_bytes:
                raise AssertionError("claim changed between launches")
        if self.interrupt:
            raise KeyboardInterrupt
        if self.error is not None:
            raise self.error
        config = json.loads(config_path.read_text(encoding="utf-8"))
        self.configs.append(config)
        if self.during_launch is not None:
            self.during_launch()
        nodes = [
            {
                "robot_id": robot_id,
                "applied_command": [0.0, 0.0, 0.0],
                "normal_problem": _normal_problem(robot_id),
                "hard_interior_selection": _policy(),
            }
            for robot_id in range(1, 15)
        ]
        result = {
            "trajectory_seed": seed,
            "config_sha256": _sha256(config_path),
            "attempt_count": 1,
            "retry_count": 0,
            "frame_zero_records": [
                {
                    "trajectory_seed": seed,
                    "frame_index": 0,
                    "complete": True,
                    "current_positions_m": config["initial"]["position"]["positions"],
                    "nodes": nodes,
                }
            ],
        }
        if self.mutate_result is not None:
            self.mutate_result(result)
        return result


class QualifiedV6OneStepOrchestrationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(
            prefix="qualified-v6-one-step-unit-", dir="/private/tmp"
        )
        self.root = Path(self.temporary.name)
        self.binary = self.root / "Swarm"
        self.binary.write_bytes(b"fake exact binary\n")
        self.binary.chmod(0o700)
        self.base = self.root / "base.json"
        self.primary = self.root / "primary.json"
        self.family = self.root / "family.json"
        shutil.copyfile(ROOT / "config/config.json", self.base)
        shutil.copyfile(
            ROOT
            / "config/diagnostics/qualified_mode_hybrid_dcbf_development_v3.json",
            self.primary,
        )
        shutil.copyfile(
            ROOT / "config/diagnostics/qualified_initial_family_v3.json",
            self.family,
        )
        self.claim = self.root / "claim.json"
        self.output = self.root / "output.json"

    def tearDown(self):
        self.temporary.cleanup()

    def arguments(self, *, operations=None):
        if operations is not None:
            operations.claim = self.claim
        return {
            "binary": self.binary,
            "base_config": self.base,
            "primary_config": self.primary,
            "initial_family": self.family,
            "claim": self.claim,
            "output": self.output,
            "project_root": ROOT,
            "operations": operations,
        }

    def audit_one(self, operations, *, claim=None, output=None):
        claim = claim or self.claim
        output = output or self.output
        operations.claim = claim
        arguments = self.arguments(operations=operations)
        arguments.update(claim=claim, output=output)
        return _audit_seed_sequence(
            **arguments,
            seed_universe=(FROZEN_SEEDS[0],),
        )

    def qualifying_one(self, operations):
        operations.claim = self.claim
        with mock.patch.object(
            producer, "SubprocessOneStepOperations", return_value=operations
        ):
            return _audit_seed_sequence(
                binary=ROOT / "build-diagnostic/Swarm",
                base_config=ROOT / "config/config.json",
                primary_config=ROOT
                / "config/diagnostics/qualified_mode_hybrid_dcbf_development_v3.json",
                initial_family=ROOT
                / "config/diagnostics/qualified_initial_family_v3.json",
                claim=self.claim,
                output=self.output,
                project_root=ROOT,
                seed_universe=(FROZEN_SEEDS[0],),
                operations=None,
            )

    def test_repository_tree_is_resolved_from_the_frozen_head(self):
        frozen_head = "a" * 40
        frozen_tree = "b" * 40
        stale_tree = "c" * 40

        def fake_git(arguments, **_kwargs):
            command = tuple(arguments)
            outputs = {
                ("git", "rev-parse", "--show-toplevel"): str(ROOT),
                ("git", "rev-parse", "HEAD"): frozen_head,
                ("git", "rev-parse", "HEAD^{tree}"): stale_tree,
                ("git", "rev-parse", f"{frozen_head}^{{tree}}"): frozen_tree,
            }
            if command not in outputs:
                raise AssertionError(f"unexpected Git command: {command}")
            return SimpleNamespace(
                returncode=0,
                stdout=outputs[command] + "\n",
                stderr="",
            )

        with mock.patch.object(producer.subprocess, "run", side_effect=fake_git):
            identity = producer._repository_identity(ROOT)

        self.assertEqual(identity["head"], frozen_head)
        self.assertEqual(identity["tree"], frozen_tree)

    def test_qualifying_rejects_alternate_producer_path_before_claim(self):
        copied_producer = self.root / "copied_producer.py"
        shutil.copyfile(Path(producer.__file__), copied_producer)
        operations = FakeOperations()
        with mock.patch.object(producer, "__file__", str(copied_producer)):
            with self.assertRaisesRegex(
                ValueError, "implementation path is not canonical"
            ):
                self.qualifying_one(operations)
        self.assertEqual(operations.launched_seeds, [])
        self.assertFalse(self.claim.exists())
        self.assertFalse(self.output.exists())

    def test_qualifying_rejects_live_producer_bytes_not_in_recorded_head(self):
        original_reader = producer._read_stable_regular_file
        canonical_producer = Path(producer.__file__).resolve()

        def dirty_live_reader(path, label):
            raw, identity = original_reader(path, label)
            if Path(path).resolve() == canonical_producer:
                raw = raw + b"\n# uncommitted producer mutation\n"
                identity = {
                    "path": str(canonical_producer),
                    "bytes": len(raw),
                    "sha256": hashlib.sha256(raw).hexdigest(),
                }
            return raw, identity

        operations = FakeOperations()
        with mock.patch.object(
            producer,
            "_read_stable_regular_file",
            side_effect=dirty_live_reader,
        ):
            with self.assertRaisesRegex(
                ValueError, "implementation differs from recorded HEAD blob"
            ):
                self.qualifying_one(operations)
        self.assertEqual(operations.launched_seeds, [])
        self.assertFalse(self.claim.exists())
        self.assertFalse(self.output.exists())

    def test_claim_write_return_identity_mismatch_is_fail_stop_before_launch(self):
        original_publish = producer._write_json_no_replace

        def wrong_return_identity(path, payload):
            identity = original_publish(path, payload)
            if Path(path) == self.claim:
                identity = dict(identity)
                identity["sha256"] = "0" * 64
            return identity

        operations = FakeOperations()
        with mock.patch.object(
            producer,
            "_write_json_no_replace",
            side_effect=wrong_return_identity,
        ):
            with self.assertRaisesRegex(
                ValueError, "published claim identity"
            ):
                self.audit_one(operations)
        self.assertEqual(operations.launched_seeds, [])
        self.assertTrue(self.claim.is_file())
        self.assertFalse(self.output.exists())

    def test_claim_post_write_return_mutation_is_fail_stop_before_launch(self):
        original_publish = producer._write_json_no_replace

        def mutate_after_return(path, payload):
            identity = original_publish(path, payload)
            if Path(path) == self.claim:
                Path(path).write_bytes(Path(path).read_bytes() + b" ")
            return identity

        operations = FakeOperations()
        with mock.patch.object(
            producer,
            "_write_json_no_replace",
            side_effect=mutate_after_return,
        ):
            with self.assertRaisesRegex(
                ValueError, "published claim identity"
            ):
                self.audit_one(operations)
        self.assertEqual(operations.launched_seeds, [])
        self.assertTrue(self.claim.is_file())
        self.assertFalse(self.output.exists())

    def test_terminal_output_post_return_byte_mutation_is_fail_stop(self):
        original_publish = producer._write_json_no_replace

        def mutate_terminal_bytes(path, payload):
            identity = original_publish(path, payload)
            if Path(path) == self.output:
                parsed = json.loads(Path(path).read_text(encoding="utf-8"))
                Path(path).write_text(
                    json.dumps(parsed, indent=2) + "\n", encoding="utf-8"
                )
            return identity

        operations = FakeOperations()
        with mock.patch.object(
            producer,
            "_write_json_no_replace",
            side_effect=mutate_terminal_bytes,
        ):
            with self.assertRaisesRegex(
                ValueError, "published terminal output identity"
            ):
                self.audit_one(operations)
        self.assertEqual(operations.launched_seeds, [FROZEN_SEEDS[0]])
        self.assertTrue(self.claim.is_file())
        self.assertTrue(self.output.is_file())

    def test_injected_operations_are_irreversibly_nonqualifying(self):
        result = self.audit_one(FakeOperations())
        claim = json.loads(self.claim.read_text(encoding="utf-8"))
        self.assertEqual(
            claim.get("execution_provenance"), "injected-operations-test-only"
        )
        self.assertIs(claim.get("qualifying"), False)
        self.assertEqual(
            result.get("execution_provenance"), "injected-operations-test-only"
        )
        self.assertIs(result.get("qualifying"), False)
        self.assertTrue(result["seed_results"][0]["passed"])
        self.assertFalse(result["passed"])
        self.assertEqual(result["status"], "test_only")
        self.assertEqual(result["reason"], "injected_operations_nonqualifying")

    def test_seed_record_retains_durable_recomputation_evidence(self):
        result = self.audit_one(FakeOperations())
        retained = result["seed_results"][0].get("recomputation_evidence")
        self.assertIsInstance(retained, dict)
        self.assertEqual(
            set(retained),
            {
                "current_positions_m",
                "robots",
                "next_positions_m",
                "next_dynamic_fim_certificates",
                "next_barriers",
                "next_local_problems",
            },
        )
        self.assertEqual(len(retained["current_positions_m"]), 14)
        self.assertEqual(len(retained["robots"]), 14)
        self.assertEqual(len(retained["next_positions_m"]), 14)
        self.assertEqual(len(retained["next_dynamic_fim_certificates"]), 14)
        self.assertEqual(len(retained["next_barriers"]), 119)
        self.assertEqual(len(retained["next_local_problems"]), 14)

    def test_materialized_config_mutation_is_fail_stop_claim_only(self):
        operations = FakeOperations()

        def mutate_config():
            path = operations.config_paths[-1]
            path.write_bytes(path.read_bytes() + b" ")

        operations.during_launch = mutate_config
        with self.assertRaisesRegex(ValueError, "materialized config identity"):
            self.audit_one(operations)
        self.assertTrue(self.claim.is_file())
        self.assertFalse(self.output.exists())

    def test_claim_output_normalized_alias_fails_before_claim_or_launch(self):
        nested = self.root / "nested"
        nested.mkdir()
        operations = FakeOperations()
        output_alias = nested / ".." / self.claim.name
        with self.assertRaisesRegex(ValueError, "distinct normalized targets"):
            self.audit_one(operations, output=output_alias)
        self.assertEqual(operations.launched_seeds, [])
        self.assertFalse(self.claim.exists())

    def test_integer_alias_for_continuous_command_is_terminally_malformed(self):
        def integer_command(result):
            result["frame_zero_records"][0]["nodes"][0]["applied_command"][0] = 0

        result = self.audit_one(FakeOperations(mutate_result=integer_command))
        observed = result["seed_results"][0]
        self.assertFalse(observed["passed"])
        self.assertIn("must be an exact finite float", observed["reasons"][0])

    def test_returned_config_identity_mismatch_is_fail_stop_claim_only(self):
        operation = FakeOperations(
            mutate_result=lambda result: result.__setitem__(
                "config_sha256", "0" * 64
            )
        )
        with self.assertRaisesRegex(ValueError, "materialized config identity"):
            self.audit_one(operation)
        self.assertTrue(self.claim.is_file())
        self.assertFalse(self.output.exists())

    def test_execution_provenance_tamper_is_claim_only(self):
        def tamper_claim():
            claim = json.loads(self.claim.read_text(encoding="utf-8"))
            claim["execution_provenance"] = "exact-binary-subprocess"
            claim["qualifying"] = True
            self.claim.write_text(json.dumps(claim), encoding="utf-8")

        operation = FakeOperations(during_launch=tamper_claim)
        with self.assertRaisesRegex(ValueError, "claim identity changed"):
            self.audit_one(operation)
        self.assertTrue(self.claim.is_file())
        self.assertFalse(self.output.exists())

    def test_qualifying_mode_rejects_each_same_byte_alternate_path(self):
        canonical = {
            "binary": ROOT / "build-diagnostic/Swarm",
            "base_config": ROOT / "config/config.json",
            "primary_config": ROOT
            / "config/diagnostics/qualified_mode_hybrid_dcbf_development_v3.json",
            "initial_family": ROOT
            / "config/diagnostics/qualified_initial_family_v3.json",
        }
        self.binary.write_bytes(canonical["binary"].read_bytes())
        self.binary.chmod(0o700)
        alternates = {
            "binary": self.binary,
            "base_config": self.base,
            "primary_config": self.primary,
            "initial_family": self.family,
        }
        for label, alternate in alternates.items():
            with self.subTest(label=label):
                claim = self.root / f"canonical-{label}-claim.json"
                output = self.root / f"canonical-{label}-output.json"
                arguments = dict(canonical)
                arguments.update(
                    claim=claim,
                    output=output,
                    project_root=ROOT,
                    seed_universe=(FROZEN_SEEDS[0],),
                    operations=None,
                )
                arguments[label] = alternate
                with self.assertRaisesRegex(
                    ValueError, f"{label} path is not canonical"
                ):
                    _audit_seed_sequence(**arguments)
                self.assertFalse(claim.exists())
                self.assertFalse(output.exists())

    def test_old_v2_primary_and_family_paths_cannot_qualify_gate_v2(self):
        self.assertEqual(
            producer.SCHEMA_VERSION,
            "cbf2026-qualified-v6-one-step-viability-v2",
        )
        self.assertEqual(
            producer.CAMPAIGN_ID,
            "qualified-v6-one-step-development-gate-v2",
        )
        with self.assertRaisesRegex(ValueError, "primary_config path is not canonical"):
            producer._require_qualifying_canonical_paths(
                binary=ROOT / "build-diagnostic/Swarm",
                base_config=ROOT / "config/config.json",
                primary_config=ROOT
                / "config/diagnostics/qualified_mode_hybrid_dcbf_development_v2.json",
                initial_family=ROOT
                / "config/diagnostics/qualified_initial_family_v3.json",
                project_root=ROOT,
            )
        with self.assertRaisesRegex(ValueError, "initial_family path is not canonical"):
            producer._require_qualifying_canonical_paths(
                binary=ROOT / "build-diagnostic/Swarm",
                base_config=ROOT / "config/config.json",
                primary_config=ROOT
                / "config/diagnostics/qualified_mode_hybrid_dcbf_development_v3.json",
                initial_family=ROOT
                / "config/diagnostics/qualified_initial_family_v2.json",
                project_root=ROOT,
            )

    def test_integer_float_boolean_and_nonfinite_aliases_fail_end_to_end(self):
        mutations = {
            "operation_seed_float": lambda result: result.__setitem__(
                "trajectory_seed", float(result["trajectory_seed"])
            ),
            "attempt_bool": lambda result: result.__setitem__("attempt_count", True),
            "retry_float": lambda result: result.__setitem__("retry_count", 0.0),
            "frame_index_bool": lambda result: result["frame_zero_records"][0].__setitem__(
                "frame_index", False
            ),
            "robot_id_float": lambda result: result["frame_zero_records"][0]["nodes"][0].__setitem__(
                "robot_id", 1.0
            ),
            "position_nan": lambda result: result["frame_zero_records"][0]["current_positions_m"][0].__setitem__(
                0, float("nan")
            ),
            "command_int": lambda result: result["frame_zero_records"][0]["nodes"][0]["applied_command"].__setitem__(
                0, 0
            ),
            "command_inf": lambda result: result["frame_zero_records"][0]["nodes"][0]["applied_command"].__setitem__(
                0, float("inf")
            ),
            "policy_int": lambda result: result["frame_zero_records"][0]["nodes"][0]["hard_interior_selection"].__setitem__(
                "enforced_floor_mps", 0
            ),
            "problem_int": lambda result: result["frame_zero_records"][0]["nodes"][0]["normal_problem"]["rows"][0].__setitem__(
                "constant", 1
            ),
        }
        for label, mutation in mutations.items():
            with self.subTest(label=label):
                claim = self.root / f"strict-{label}-claim.json"
                output = self.root / f"strict-{label}-output.json"
                result = self.audit_one(
                    FakeOperations(mutate_result=mutation),
                    claim=claim,
                    output=output,
                )
                self.assertEqual(result["status"], "test_only")
                self.assertFalse(result["predicate_passed"])
                self.assertFalse(result["seed_results"][0]["passed"])
                self.assertTrue(output.is_file())

    def test_retained_payload_round_trips_and_rejects_mutation(self):
        result = self.audit_one(FakeOperations())
        family = json.loads(self.family.read_text(encoding="utf-8"))
        seed_result = result["seed_results"][0]
        assert_retained_evidence_recomputes(self, seed_result, family)

        def mutate_current(row):
            row["recomputation_evidence"]["current_positions_m"][0][0] += 1.0

        def mutate_command(row):
            row["recomputation_evidence"]["robots"][0]["applied_command"][0] += 1.0

        def mutate_problem(row):
            row["recomputation_evidence"]["robots"][0]["normal_problem"]["rows"][0]["constant"] += 1.0

        def mutate_next_position(row):
            row["recomputation_evidence"]["next_positions_m"][0][0] += 1.0

        def mutate_certificate(row):
            row["recomputation_evidence"]["next_dynamic_fim_certificates"][0]["epsilon"] += 1.0

        def mutate_barrier(row):
            row["recomputation_evidence"]["next_barriers"][0]["value_m"] += 1.0

        def mutate_next_problem(row):
            row["recomputation_evidence"]["next_local_problems"][0]["problem"]["rows"][0]["constant"] += 1.0

        def mutate_radius(row):
            row["recomputation_evidence"]["next_local_problems"][0]["radius_mps"] += 1.0

        for label, mutation in (
            ("current", mutate_current),
            ("command", mutate_command),
            ("normal_problem", mutate_problem),
            ("next_position", mutate_next_position),
            ("certificate", mutate_certificate),
            ("barrier", mutate_barrier),
            ("next_problem", mutate_next_problem),
            ("radius", mutate_radius),
        ):
            with self.subTest(label=label):
                changed = copy.deepcopy(seed_result)
                mutation(changed)
                with self.assertRaises((AssertionError, ValueError)):
                    assert_retained_evidence_recomputes(self, changed, family)

    def test_gate_launches_each_frozen_seed_exactly_once(self):
        operations = FakeOperations()
        result = audit_one_step_universe(**self.arguments(operations=operations))
        self.assertFalse(result["passed"], result)
        self.assertTrue(result["predicate_passed"], result)
        self.assertEqual(operations.launched_seeds, FROZEN_SEEDS)
        self.assertEqual(len(set(operations.launched_seeds)), 100)
        self.assertEqual(result["launch_count"], 100)
        self.assertEqual(result["retry_count"], 0)
        self.assertEqual(result["audit_summary"]["passed_count"], 100)
        self.assertEqual(result["registered_summary"]["passed_count"], 10)
        self.assertGreater(result["audit_summary"]["minimum_next_barrier_m"], 0.0)
        self.assertGreaterEqual(
            result["audit_summary"]["minimum_next_local_radius_mps"], 0.05
        )
        self.assertEqual(result["source"], result["terminal_source"])
        assert_terminal_predicate_consistent(self, result)
        self.assertTrue(self.claim.is_file())
        self.assertEqual(json.loads(self.output.read_text(encoding="utf-8")), result)
        self.assertEqual(result["claim_sha256"], _sha256(self.claim))
        claim = json.loads(self.claim.read_text(encoding="utf-8"))
        self.assertEqual(claim["source"]["head"], result["source"]["head"])
        self.assertEqual(claim["source"]["tree"], result["source"]["tree"])
        self.assertEqual(
            set(claim["bindings"]),
            {"implementation", "binary", "base_config", "primary_config", "initial_family"},
        )
        self.assertIn("not long-horizon safety", result["boundary"])
        self.assertIn("not long-horizon safety", claim["boundary"])
        self.assertTrue(all(not path.exists() for path in operations.config_paths))
        self.assertEqual(len({path.parent for path in operations.config_paths}), 100)
        for seed, config in zip(FROZEN_SEEDS, operations.configs):
            self.assertEqual(config["execute"]["random-seed"], seed)
            self.assertEqual(config["execute"]["time-total"], 0.5)
            self.assertEqual(config["execute"]["time-step"], 0.5)
            self.assertEqual(
                config["evidence-stream"],
                {
                    "enabled": True,
                    "schema-version": "cbf2026-qualified-evidence-v1",
                    "campaign-id": producer.CAMPAIGN_ID,
                    "trajectory-seed": seed,
                    "range-noise-seed": 0,
                    "condition": producer.CONDITION,
                },
            )
            self.assertNotIn("campaign_root", config)

    def test_existing_claim_or_output_is_never_overwritten(self):
        self.claim.write_text("occupied", encoding="utf-8")
        with self.assertRaises(FileExistsError):
            audit_one_step_universe(**self.arguments(operations=FakeOperations()))
        self.assertEqual(self.claim.read_text(encoding="utf-8"), "occupied")
        self.claim.unlink()
        self.output.write_text("occupied", encoding="utf-8")
        with self.assertRaises(FileExistsError):
            audit_one_step_universe(**self.arguments(operations=FakeOperations()))
        self.assertEqual(self.output.read_text(encoding="utf-8"), "occupied")

    def test_hard_interruption_consumes_identity_before_first_launch(self):
        operations = FakeOperations(interrupt=True)
        with self.assertRaises(KeyboardInterrupt):
            audit_one_step_universe(**self.arguments(operations=operations))
        self.assertEqual(operations.launched_seeds, [FROZEN_SEEDS[0]])
        self.assertTrue(self.claim.is_file())
        self.assertFalse(self.output.exists())
        with self.assertRaises(FileExistsError):
            audit_one_step_universe(**self.arguments(operations=FakeOperations()))

    def test_caught_child_nonzero_and_timeout_publish_terminal_failures(self):
        for label, error in (
            ("nonzero", ChildLaunchFailure("child_nonzero_exit")),
            ("timeout", TimeoutError("bounded timeout")),
        ):
            with self.subTest(label=label):
                claim = self.root / f"{label}-claim.json"
                output = self.root / f"{label}-output.json"
                arguments = self.arguments(operations=FakeOperations(error=error))
                arguments["claim"] = claim
                arguments["output"] = output
                arguments["operations"].claim = claim
                result = audit_one_step_universe(**arguments)
                self.assertEqual(result["status"], "test_only")
                self.assertFalse(result["passed"])
                self.assertEqual(result["launch_count"], 1)
                self.assertEqual(result["claim_sha256"], _sha256(claim))
                self.assertTrue(output.is_file())

    def test_malformed_multiple_wrong_identity_and_retry_fail_closed(self):
        mutations = {
            "missing": lambda result: result.__setitem__("frame_zero_records", []),
            "multiple": lambda result: result["frame_zero_records"].append(
                json.loads(json.dumps(result["frame_zero_records"][0]))
            ),
            "wrong_seed": lambda result: result.__setitem__(
                "trajectory_seed", result["trajectory_seed"] + 1
            ),
            "wrong_frame_seed": lambda result: result["frame_zero_records"][0].__setitem__(
                "trajectory_seed", result["trajectory_seed"] + 1
            ),
            "retry": lambda result: result.update({"attempt_count": 2, "retry_count": 1}),
            "incomplete": lambda result: result["frame_zero_records"][0].__setitem__(
                "complete", False
            ),
        }
        for label, mutation in mutations.items():
            with self.subTest(label=label):
                claim = self.root / f"malformed-{label}-claim.json"
                output = self.root / f"malformed-{label}-output.json"
                operation = FakeOperations(mutate_result=mutation)
                arguments = self.arguments(operations=operation)
                arguments.update(claim=claim, output=output)
                operation.claim = claim
                result = audit_one_step_universe(**arguments)
                self.assertEqual(result["status"], "test_only")
                self.assertFalse(result["passed"])
                self.assertEqual(result["launch_count"], 1)
                self.assertEqual(result["retry_count"], 1 if label == "retry" else 0)

    def test_forged_floor_and_residual_below_mu_terminally_fail(self):
        def forge(result):
            policy = result["frame_zero_records"][0]["nodes"][0][
                "hard_interior_selection"
            ]
            policy["enforced_floor_mps"] = 0.0
            policy["minimum_original_hard_residual_mps"] = 0.0

        operation = FakeOperations(mutate_result=forge)
        result = audit_one_step_universe(**self.arguments(operations=operation))
        self.assertEqual(result["status"], "test_only")
        self.assertFalse(result["passed"])
        self.assertEqual(result["launch_count"], 1)

    def test_component_overflow_terminally_fails(self):
        def overflow(result):
            node = result["frame_zero_records"][0]["nodes"][0]
            node["applied_command"][0] = 25.0 + 2e-7
            node["hard_interior_selection"][
                "minimum_original_hard_residual_mps"
            ] = 26.0 + 2e-7

        operation = FakeOperations(mutate_result=overflow)
        reconstruction = _fast_reconstruction()
        with mock.patch.object(
            producer, "_reconstruct_next_metrics", return_value=reconstruction
        ):
            result = audit_one_step_universe(
                **self.arguments(operations=operation)
            )
        self.assertEqual(result["status"], "test_only")
        self.assertFalse(result["passed"])
        self.assertEqual(result["launch_count"], 100)
        assert_terminal_predicate_consistent(self, result)

    def test_actual_residual_below_mu_terminally_fails(self):
        def below_floor(result):
            node = result["frame_zero_records"][0]["nodes"][0]
            node["normal_problem"]["rows"][0]["constant"] = 0.0999998
            policy = node["hard_interior_selection"]
            policy["planar_chebyshev_radius_mps"] = 25.0999998
            policy["minimum_original_hard_residual_mps"] = 0.0999998

        operation = FakeOperations(mutate_result=below_floor)
        reconstruction = _fast_reconstruction()
        with mock.patch.object(
            producer, "_reconstruct_next_metrics", return_value=reconstruction
        ):
            result = audit_one_step_universe(
                **self.arguments(operations=operation)
            )
        self.assertEqual(result["status"], "test_only")
        self.assertFalse(result["passed"])
        self.assertIn(
            "applied_residual_below_floor",
            result["seed_results"][0]["reasons"],
        )
        self.assertEqual(result["launch_count"], 100)
        assert_terminal_predicate_consistent(self, result)

    def test_nonpositive_barrier_and_small_radius_terminally_fail(self):
        cases = {
            "barrier": (
                tuple(
                    SimpleNamespace(value=0.0 if index == 0 else 1.0)
                    for index in range(119)
                ),
                (0.05,) * 14,
                "nonpositive_next_barrier",
            ),
            "radius": (
                tuple(SimpleNamespace(value=1.0) for _ in range(119)),
                (0.049999,) + (0.05,) * 13,
                "small_next_local_radius",
            ),
        }
        for label, (barriers, radii, expected_reason) in cases.items():
            with self.subTest(label=label):
                claim = self.root / f"predicate-{label}-claim.json"
                output = self.root / f"predicate-{label}-output.json"
                operation = FakeOperations()
                arguments = self.arguments(operations=operation)
                arguments.update(claim=claim, output=output)
                operation.claim = claim
                with mock.patch.object(
                    producer,
                    "_reconstruct_next_metrics",
                    return_value=_fast_reconstruction(
                        barriers=barriers, radii=radii
                    ),
                ):
                    result = audit_one_step_universe(**arguments)
                self.assertEqual(result["status"], "test_only")
                self.assertIn(expected_reason, result["seed_results"][0]["reasons"])
                self.assertEqual(result["launch_count"], 100)
                assert_terminal_predicate_consistent(self, result)

    def test_component_within_numeric_tolerance_is_accepted(self):
        def boundary(result):
            node = result["frame_zero_records"][0]["nodes"][0]
            node["applied_command"][0] = 25.0 + 0.5e-7
            node["hard_interior_selection"][
                "minimum_original_hard_residual_mps"
            ] = 26.0 + 0.5e-7

        operation = FakeOperations(mutate_result=boundary)
        reconstruction = _fast_reconstruction()
        with mock.patch.object(
            producer, "_reconstruct_next_metrics", return_value=reconstruction
        ):
            result = audit_one_step_universe(
                **self.arguments(operations=operation)
            )
        self.assertFalse(result["passed"], result)
        self.assertTrue(result["predicate_passed"], result)
        self.assertEqual(result["launch_count"], 100)
        assert_terminal_predicate_consistent(self, result)

    def test_source_identity_mutation_fails_before_child_launch(self):
        original = producer._repository_identity(ROOT)
        for field in ("head", "tree"):
            with self.subTest(field=field):
                changed = dict(original)
                changed[field] = "0" * 40
                claim = self.root / f"source-{field}-claim.json"
                output = self.root / f"source-{field}-output.json"
                operation = FakeOperations()
                arguments = self.arguments(operations=operation)
                arguments.update(claim=claim, output=output)
                operation.claim = claim
                with mock.patch.object(
                    producer,
                    "_repository_identity",
                    side_effect=[original, changed, changed],
                ), self.assertRaisesRegex(ValueError, "bound identity changed"):
                    audit_one_step_universe(**arguments)
                self.assertEqual(operation.launched_seeds, [])
                self.assertTrue(claim.is_file())
                self.assertFalse(output.exists())

    def test_implementation_identity_mutation_fails_before_child_launch(self):
        original_collector = producer._collect_bound_identities
        calls = 0

        def changed_implementation(**arguments):
            nonlocal calls
            calls += 1
            identities = original_collector(**arguments)
            if calls > 1:
                identities["implementation"]["sha256"] = "0" * 64
            return identities

        operation = FakeOperations()
        with mock.patch.object(
            producer,
            "_collect_bound_identities",
            side_effect=changed_implementation,
        ), self.assertRaisesRegex(ValueError, "bound identity changed"):
            audit_one_step_universe(**self.arguments(operations=operation))
        self.assertEqual(operation.launched_seeds, [])
        self.assertTrue(self.claim.is_file())
        self.assertFalse(self.output.exists())

    def test_claim_and_output_fsync_file_and_parent(self):
        operation = FakeOperations(error=ChildLaunchFailure("expected"))
        calls = []
        original_fsync = producer.os.fsync

        def recording_fsync(descriptor):
            calls.append(descriptor)
            return original_fsync(descriptor)

        with mock.patch.object(producer.os, "fsync", side_effect=recording_fsync):
            result = audit_one_step_universe(
                **self.arguments(operations=operation)
            )
        self.assertEqual(result["status"], "test_only")
        self.assertGreaterEqual(len(calls), 6)

    def test_bound_file_identity_mutations_terminally_fail(self):
        for label, path in (
            ("binary", self.binary),
            ("base", self.base),
            ("primary", self.primary),
            ("family", self.family),
        ):
            with self.subTest(label=label):
                original = path.read_bytes()
                claim = self.root / f"identity-{label}-claim.json"
                output = self.root / f"identity-{label}-output.json"
                operation = FakeOperations(
                    during_launch=lambda target=path: target.write_bytes(
                        target.read_bytes() + b" "
                    )
                )
                arguments = self.arguments(operations=operation)
                arguments.update(claim=claim, output=output)
                operation.claim = claim
                try:
                    with self.assertRaisesRegex(
                        ValueError, "bound identity changed"
                    ):
                        audit_one_step_universe(**arguments)
                finally:
                    path.write_bytes(original)
                self.assertTrue(claim.is_file())
                self.assertFalse(output.exists())

    def test_claim_mutation_is_detected_and_cross_bound_terminally(self):
        def mutate_claim():
            with self.claim.open("ab") as destination:
                destination.write(b" ")

        operation = FakeOperations(during_launch=mutate_claim)
        with self.assertRaisesRegex(ValueError, "claim identity changed"):
            audit_one_step_universe(**self.arguments(operations=operation))
        self.assertTrue(self.claim.is_file())
        self.assertFalse(self.output.exists())

    def test_predicate_failure_cannot_hide_identity_mutation(self):
        original = self.binary.read_bytes()

        def predicate_and_identity_failure(result):
            node = result["frame_zero_records"][0]["nodes"][0]
            node["applied_command"][0] = 25.0 + 2e-7
            node["hard_interior_selection"][
                "minimum_original_hard_residual_mps"
            ] = 26.0 + 2e-7
            self.binary.write_bytes(original + b"changed")

        operation = FakeOperations(mutate_result=predicate_and_identity_failure)
        audit = SimpleNamespace(positions_sha256="0" * 64)
        reconstruction = (
            audit,
            tuple(SimpleNamespace(value=1.0) for _ in range(119)),
            (0.05,) * 14,
        )
        try:
            with mock.patch.object(
                producer, "_reconstruct_next_metrics", return_value=reconstruction
            ), self.assertRaisesRegex(ValueError, "bound identity changed"):
                audit_one_step_universe(
                    **self.arguments(operations=operation)
                )
        finally:
            self.binary.write_bytes(original)
        self.assertTrue(self.claim.is_file())
        self.assertFalse(self.output.exists())

    def test_materialization_and_claim_share_the_same_input_bytes(self):
        original_reader = producer._read_bound_json

        def mutate_after_parse(path, label):
            value, identity = original_reader(path, label)
            if Path(path) == self.base:
                self.base.write_bytes(self.base.read_bytes() + b" ")
            return value, identity

        operation = FakeOperations()
        with mock.patch.object(
            producer, "_read_bound_json", side_effect=mutate_after_parse
        ), self.assertRaisesRegex(ValueError, "input changed before claim"):
            audit_one_step_universe(
                **self.arguments(operations=operation)
            )
        self.assertEqual(operation.launched_seeds, [])
        self.assertFalse(self.claim.exists())
        self.assertFalse(self.output.exists())

    def test_partial_terminal_publication_is_preserved_and_blocks_relaunch(self):
        original_publish = producer._write_json_no_replace

        def partial_output(path, payload):
            if Path(path) == self.output:
                with Path(path).open("xb") as destination:
                    destination.write(b'{"terminal":')
                    destination.flush()
                    os.fsync(destination.fileno())
                raise OSError("injected partial terminal publication")
            return original_publish(path, payload)

        fast_reconstruction = _fast_reconstruction()
        with mock.patch.object(
            producer, "_reconstruct_next_metrics", return_value=fast_reconstruction
        ), mock.patch.object(
            producer, "_write_json_no_replace", side_effect=partial_output
        ):
            with self.assertRaisesRegex(OSError, "partial terminal"):
                audit_one_step_universe(
                    **self.arguments(operations=FakeOperations())
                )
        self.assertEqual(self.output.read_bytes(), b'{"terminal":')
        with self.assertRaises(FileExistsError):
            audit_one_step_universe(**self.arguments(operations=FakeOperations()))


class QualifiedV6OneStepPredicateTests(unittest.TestCase):
    def predicate(self, *, component=25.0, residual=0.1, floor=0.1, barrier=1.0, radius=0.05):
        return _evaluate_predicate(
            commands=[(component, 0.0, 0.0)],
            residual_floor_pairs=[(residual, floor)],
            next_barriers=[barrier],
            next_radii=[radius],
        )

    def test_strict_barrier_and_radius_boundaries(self):
        self.assertFalse(self.predicate(barrier=0.0)[0])
        self.assertFalse(self.predicate(barrier=-1e-12)[0])
        self.assertFalse(self.predicate(radius=0.049999)[0])
        self.assertTrue(self.predicate(radius=0.05)[0])

    def test_component_and_residual_tolerances_are_exact(self):
        self.assertTrue(self.predicate(component=25.0 + 1e-7)[0])
        self.assertFalse(self.predicate(component=25.0 + 1.000001e-7)[0])
        self.assertTrue(self.predicate(residual=0.1 - 1e-7)[0])
        self.assertFalse(self.predicate(residual=0.1 - 1.000001e-7)[0])

    def test_predicate_rejects_noncanonical_or_nonfinite_inputs(self):
        for label, arguments in (
            ("component_int", {"component": 25}),
            ("residual_nan", {"residual": float("nan")}),
            ("floor_inf", {"floor": float("inf")}),
            ("barrier_nan", {"barrier": float("nan")}),
            ("radius_inf", {"radius": float("inf")}),
        ):
            with self.subTest(label=label), self.assertRaisesRegex(
                ValueError, "exact finite float"
            ):
                self.predicate(**arguments)


class QualifiedV6OneStepSubprocessOperationsTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(
            prefix="qualified-v6-subprocess-unit-", dir="/private/tmp"
        )
        self.root = Path(self.temporary.name)
        self.config = self.root / "config.json"
        self.config.write_text("{}\n", encoding="utf-8")
        self.operations = SubprocessOneStepOperations(
            binary=self.root / "Swarm", project_root=ROOT
        )

    def tearDown(self):
        self.temporary.cleanup()

    def test_nonzero_timeout_and_malformed_stdout_are_terminal_child_failures(self):
        cases = {
            "nonzero": subprocess_completed(1, "", "failure"),
            "malformed": subprocess_completed(0, "not-json\n", ""),
            "multiple": subprocess_completed(
                0,
                '{"record_type":"controller_interval"}\n'
                '{"record_type":"controller_interval"}\n',
                "",
            ),
        }
        for label, completed in cases.items():
            with self.subTest(label=label), mock.patch.object(
                producer.subprocess, "run", return_value=completed
            ) as launched:
                with self.assertRaises(ChildLaunchFailure):
                    self.operations.launch_seed(
                        seed=FROZEN_SEEDS[0], config_path=self.config
                    )
                self.assertEqual(launched.call_count, 1)
        with mock.patch.object(
            producer.subprocess,
            "run",
            side_effect=producer.subprocess.TimeoutExpired(["Swarm"], 30.0),
        ) as launched:
            with self.assertRaisesRegex(ChildLaunchFailure, "child_timeout"):
                self.operations.launch_seed(
                    seed=FROZEN_SEEDS[0], config_path=self.config
                )
            self.assertEqual(launched.call_count, 1)


def subprocess_completed(returncode: int, stdout: str, stderr: str):
    return producer.subprocess.CompletedProcess(
        args=["Swarm"], returncode=returncode, stdout=stdout, stderr=stderr
    )


class QualifiedV6OneStepRealBinaryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        binary = os.environ.get("CBF_SWARM_BINARY")
        if not binary:
            raise unittest.SkipTest("CBF_SWARM_BINARY is required for the declared real test")
        cls.binary = (ROOT / binary).resolve() if not Path(binary).is_absolute() else Path(binary)
        if not cls.binary.is_file():
            raise unittest.SkipTest(f"Swarm binary not found: {cls.binary}")

    def test_exact_binary_launches_one_declared_seed_once_in_private_temp(self):
        production_paths = (
            ROOT
            / "docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2-claim.json",
            ROOT
            / "docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-v2.json",
            ROOT
            / "docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v6-protocol.json",
            Path("/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v6"),
            Path("/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v6"),
        )
        self.assertTrue(all(not path.exists() for path in production_paths))

        with tempfile.TemporaryDirectory(
            prefix="qualified-v6-one-step-real-", dir="/private/tmp"
        ) as temporary:
            temporary_path = Path(temporary)
            claim = temporary_path / "claim.json"
            output = temporary_path / "output.json"
            result = _audit_seed_sequence(
                binary=self.binary,
                base_config=ROOT / "config/config.json",
                primary_config=ROOT
                / "config/diagnostics/qualified_mode_hybrid_dcbf_development_v3.json",
                initial_family=ROOT
                / "config/diagnostics/qualified_initial_family_v3.json",
                claim=claim,
                output=output,
                project_root=ROOT,
                seed_universe=(FROZEN_SEEDS[0],),
                operations=None,
            )
            self.assertTrue(result["terminal"], result)
            self.assertEqual(result["launch_count"], 1)
            self.assertEqual(result["retry_count"], 0)
            assert_terminal_predicate_consistent(self, result)
            observed = result["seed_results"][0]
            self.assertEqual(observed["barrier_count"], 119)
            self.assertEqual(observed["local_radius_count"], 14)
            self.assertEqual(sorted(path.name for path in temporary_path.iterdir()), ["claim.json", "output.json"])
        self.assertFalse(Path(temporary).exists())
        self.assertTrue(all(not path.exists() for path in production_paths))


if __name__ == "__main__":
    unittest.main()
