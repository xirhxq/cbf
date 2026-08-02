from __future__ import annotations

import hashlib
import json
import os
import shutil
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import scripts.diagnostics.audit_qualified_v6_one_step_viability as producer
from scripts.diagnostics.audit_qualified_v6_one_step_viability import (
    ChildLaunchFailure,
    SubprocessOneStepOperations,
    _audit_seed_sequence,
    _evaluate_predicate,
    audit_one_step_universe,
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
    expected_pass = (
        result["launch_count"] == len(result["seed_universe"])
        and all(row["passed"] for row in result["seed_results"])
    )
    case.assertEqual(result["passed"], expected_pass)
    case.assertEqual(result["status"], "completed" if expected_pass else "failed")
    case.assertEqual(
        result["reason"], "completed" if expected_pass else "predicate_failed"
    )


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
        "mode": "planar-chebyshev-fraction-cap-v1",
        "fraction": 0.1,
        "cap_mps": 0.1,
        "feasibility_tolerance_mps": 1e-9,
        "planar_chebyshev_radius_mps": 26.0,
        "enforced_floor_mps": 0.1,
        "minimum_original_hard_residual_mps": 1.0,
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
            / "config/diagnostics/qualified_mode_hybrid_dcbf_development_v2.json",
            self.primary,
        )
        shutil.copyfile(
            ROOT / "config/diagnostics/qualified_initial_family_v2.json",
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

    def test_gate_launches_each_frozen_seed_exactly_once(self):
        operations = FakeOperations()
        result = audit_one_step_universe(**self.arguments(operations=operations))
        self.assertTrue(result["passed"], result)
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
                self.assertEqual(result["status"], "failed")
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
            "wrong_hash": lambda result: result.__setitem__("config_sha256", "0" * 64),
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
                self.assertEqual(result["status"], "failed")
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
        self.assertEqual(result["status"], "failed")
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
        audit = SimpleNamespace(positions_sha256="0" * 64)
        reconstruction = (
            audit,
            tuple(SimpleNamespace(value=1.0) for _ in range(119)),
            (0.05,) * 14,
        )
        with mock.patch.object(
            producer, "_reconstruct_next_metrics", return_value=reconstruction
        ):
            result = audit_one_step_universe(
                **self.arguments(operations=operation)
            )
        self.assertEqual(result["status"], "failed")
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
        audit = SimpleNamespace(positions_sha256="0" * 64)
        reconstruction = (
            audit,
            tuple(SimpleNamespace(value=1.0) for _ in range(119)),
            (0.05,) * 14,
        )
        with mock.patch.object(
            producer, "_reconstruct_next_metrics", return_value=reconstruction
        ):
            result = audit_one_step_universe(
                **self.arguments(operations=operation)
            )
        self.assertEqual(result["status"], "failed")
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
                audit = SimpleNamespace(positions_sha256="0" * 64)
                with mock.patch.object(
                    producer,
                    "_reconstruct_next_metrics",
                    return_value=(audit, barriers, radii),
                ):
                    result = audit_one_step_universe(**arguments)
                self.assertEqual(result["status"], "failed")
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
        audit = SimpleNamespace(positions_sha256="0" * 64)
        reconstruction = (
            audit,
            tuple(SimpleNamespace(value=1.0) for _ in range(119)),
            (0.05,) * 14,
        )
        with mock.patch.object(
            producer, "_reconstruct_next_metrics", return_value=reconstruction
        ):
            result = audit_one_step_universe(
                **self.arguments(operations=operation)
            )
        self.assertTrue(result["passed"], result)
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
        self.assertEqual(result["status"], "failed")
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

        fast_state = SimpleNamespace(
            positions_sha256="0" * 64,
        )
        fast_reconstruction = (
            fast_state,
            tuple(SimpleNamespace(value=1.0) for _ in range(119)),
            (0.05,) * 14,
        )
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
            / "docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability-claim.json",
            ROOT
            / "docs/diagnostics/reviews/2026-08-02-cbf2026-qualified-v6-one-step-viability.json",
            ROOT
            / "docs/diagnostics/2026-08-02-cbf2026-qualified-closure-development-v6-protocol.json",
            Path("/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v6"),
            Path("/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v6"),
        )
        self.assertTrue(all(not path.exists() for path in production_paths))

        class CountingOperations(SubprocessOneStepOperations):
            def __init__(self, **arguments):
                super().__init__(**arguments)
                self.calls = []
                self.config_paths = []

            def launch_seed(self, *, seed, config_path):
                self.calls.append(seed)
                self.config_paths.append(config_path)
                return super().launch_seed(seed=seed, config_path=config_path)

        with tempfile.TemporaryDirectory(
            prefix="qualified-v6-one-step-real-", dir="/private/tmp"
        ) as temporary:
            temporary_path = Path(temporary)
            claim = temporary_path / "claim.json"
            output = temporary_path / "output.json"
            operations = CountingOperations(
                binary=self.binary, project_root=ROOT
            )
            result = _audit_seed_sequence(
                binary=self.binary,
                base_config=ROOT / "config/config.json",
                primary_config=ROOT
                / "config/diagnostics/qualified_mode_hybrid_dcbf_development_v2.json",
                initial_family=ROOT
                / "config/diagnostics/qualified_initial_family_v2.json",
                claim=claim,
                output=output,
                project_root=ROOT,
                seed_universe=(FROZEN_SEEDS[0],),
                operations=operations,
            )
            self.assertTrue(result["terminal"], result)
            self.assertEqual(result["launch_count"], 1)
            self.assertEqual(result["retry_count"], 0)
            self.assertEqual(operations.calls, [FROZEN_SEEDS[0]])
            assert_terminal_predicate_consistent(self, result)
            observed = result["seed_results"][0]
            self.assertEqual(observed["barrier_count"], 119)
            self.assertEqual(observed["local_radius_count"], 14)
            self.assertTrue(all(not path.exists() for path in operations.config_paths))
            self.assertEqual(sorted(path.name for path in temporary_path.iterdir()), ["claim.json", "output.json"])
        self.assertFalse(Path(temporary).exists())
        self.assertTrue(all(not path.exists() for path in production_paths))


if __name__ == "__main__":
    unittest.main()
