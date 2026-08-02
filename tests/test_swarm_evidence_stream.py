import copy
import json
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

import scripts.diagnostics.qualified_closure_evidence as evidence_schema

from scripts.diagnostics.qualified_closure_evidence import (
    FrozenMissionSchedule,
    audit_evidence_denominators,
    audit_reset_primitives,
    reconstruct_controller_primitives,
    validate_controller_primitive_schema,
    validate_endpoint_primitive_schema,
    validate_initialization_schema,
    validate_mission_terminal_schema,
)


ROOT = Path(__file__).resolve().parents[1]
SWARM_BINARY = Path(
    os.environ.get("CBF_SWARM_BINARY", ROOT / "build-diagnostic" / "Swarm")
)
NO_HOOK_SWARM_BINARY = Path(
    os.environ.get(
        "CBF_NO_HOOK_SWARM_BINARY",
        ROOT / "build-diagnostic" / "SwarmNoEvidenceTestHooks",
    )
)
PRIMARY = ROOT / "config/diagnostics/qualified_mode_hybrid_dcbf_development_v1.json"
PRIMARY_V2 = ROOT / "config/diagnostics/qualified_mode_hybrid_dcbf_development_v2.json"


def merge_overlay(materialized, overlay):
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(materialized.get(key), dict):
            merge_overlay(materialized[key], value)
        else:
            materialized[key] = value


def evidence_fixture(output_root: Path, *, v2=False) -> dict:
    config = json.loads((ROOT / "config/config.json").read_text(encoding="utf-8"))
    merge_overlay(config, json.loads((PRIMARY_V2 if v2 else PRIMARY).read_text(encoding="utf-8")))
    config["initial"]["position"] = {
        "method": "specified",
        "positions": [
            [-1450.0, -300.0], [-1250.0, -300.0],
            [-1050.0, -300.0], [-850.0, -300.0],
            [-1450.0, -100.0], [-1250.0, -100.0],
            [-1050.0, -100.0], [-850.0, -100.0],
            [-1450.0, 100.0], [-1250.0, 100.0],
            [-1050.0, 100.0], [-850.0, 100.0],
            [-1450.0, 300.0], [-1250.0, 300.0],
        ],
    }
    if not v2:
        config["cbfs"]["without-slack"]["comm-fixed"]["alpha"]["coe"] = 10.0
        config["cbfs"]["without-slack"]["safety"]["alpha"]["coe"] = 10.0
    config["execute"]["time-total"] = 1.0
    config["execute"]["random-seed"] = 7
    config["execute"]["check-constraint-violation"] = False
    config["output_path"] = str(output_root)
    config["evidence-stream"] = {
        "enabled": True,
        "schema-version": "cbf2026-qualified-evidence-v1",
        "campaign-id": "two-frame-fixture",
        "trajectory-seed": 101,
        "range-noise-seed": 201,
        "condition": "dynamic_primary",
    }
    if v2:
        config["initial"]["position"] = {
            "method": "specified",
            "positions": json.loads(
                (ROOT / "config/diagnostics/qualified_initial_family_v1.json").read_text(
                    encoding="utf-8"
                )
            )["template_positions_m"],
        }
    return config


class SwarmEvidenceStdoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not SWARM_BINARY.is_file():
            raise unittest.SkipTest(f"Swarm binary not found: {SWARM_BINARY}")

    def test_two_frame_real_swarm_stdout_is_jsonl_only(self):
        with tempfile.TemporaryDirectory(prefix="cbf-evidence-") as temporary:
            root = Path(temporary)
            config_path = root / "config.json"
            config_path.write_text(
                json.dumps(evidence_fixture(root)), encoding="utf-8"
            )
            result = subprocess.run(
                [str(SWARM_BINARY), str(config_path)],
                cwd=ROOT,
                text=True,
                capture_output=True,
                timeout=30,
                check=False,
            )

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            lines = [line for line in result.stdout.splitlines() if line]
            records = []
            for line in lines:
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError:
                    self.fail(f"non-JSON stdout line: {line!r}")
            counts = {
                kind: sum(row.get("record_type") == kind for row in records)
                for kind in {
                    "initialization",
                    "reset",
                    "endpoint_row",
                    "controller_interval",
                    "mission_terminal",
                }
            }
            self.assertEqual(counts["initialization"], 14)
            self.assertEqual(counts["reset"], 1)
            self.assertEqual(counts["endpoint_row"], 232 * 2)
            self.assertEqual(counts["controller_interval"], 2)
            self.assertEqual(counts["mission_terminal"], 1)
            self.assertEqual(records[-1]["record_type"], "mission_terminal")
            controllers = [
                row for row in records if row.get("record_type") == "controller_interval"
            ]
            endpoints = [
                row for row in records if row.get("record_type") == "endpoint_row"
            ]
            resets = [row for row in records if row.get("record_type") == "reset"]
            initializations = [
                row for row in records if row.get("record_type") == "initialization"
            ]
            terminal = records[-1]
            self.assertTrue(all(map(validate_initialization_schema, initializations)))
            self.assertTrue(validate_mission_terminal_schema(terminal))
            self.assertTrue(
                all(set(row["runtime"]) == {"local_index"} for row in initializations)
            )
            self.assertEqual(resets[0]["runtime"].get("stage"), "committed")
            self.assertEqual(len(resets[0]["runtime"]["hard_problems"]), 14)
            self.assertTrue(
                all(
                    len(qp["solution"]) == 3
                    for qp in resets[0]["runtime"]["local_hard_qps"]
                )
            )
            self.assertEqual(
                audit_reset_primitives(resets[0], tuple(range(1, 15)), 119),
                (),
            )
            self.assertTrue(all(map(validate_controller_primitive_schema, controllers)))
            self.assertTrue(all(map(validate_endpoint_primitive_schema, endpoints)))
            for controller in controllers:
                self.assertEqual(
                    len(controller["runtime"]["nodes"]), 14
                )
                for node in controller["runtime"]["nodes"]:
                    self.assertIn("normal_problem", node)
                    self.assertIn("hard_only_problem", node)
                    self.assertEqual(len(node["normal_qp"]["solution"]), 3)
                    self.assertEqual(len(node["hard_only_qp"]["solution"]), 3)
                    self.assertNotIn("hard_interior_selection", node)
            for controller in controllers:
                frame_rows = [
                    row
                    for row in endpoints
                    if row["frame_index"] == controller["frame_index"]
                ]
                reconstruction = reconstruct_controller_primitives(
                    controller, frame_rows
                )
                self.assertEqual(reconstruction.integrity_errors, ())
                self.assertEqual(len(reconstruction.nodes), 14)
                self.assertEqual(len(reconstruction.local_residuals), 232)
                self.assertEqual(len(reconstruction.full_residuals), 119)
            schedule = FrozenMissionSchedule(
                campaign_id="two-frame-fixture",
                trajectory_seed=101,
                range_noise_seed=201,
                frames=2,
                robots=tuple(range(1, 15)),
                estimator_conditions=("dynamic_primary", "fixed_fim_ablation"),
                controller_condition="dynamic_primary",
            )
            depths = {
                condition: {
                    robot_id: (robot_id - 1) % 7 + 1
                    for robot_id in schedule.robots
                }
                for condition in schedule.estimator_conditions
            }
            audit = audit_evidence_denominators(records, [schedule], depths)
            self.assertEqual(audit.controller_availability.numerator, 2)
            self.assertEqual(audit.controller_availability.denominator, 2)
            self.assertEqual(audit.mission_success.numerator, 1)
            self.assertEqual(audit.mission_success.denominator, 1)

            missing_reset_records = [
                row for row in records if row.get("record_type") != "reset"
            ]
            missing_reset_audit = audit_evidence_denominators(
                missing_reset_records, [schedule], depths
            )
            self.assertEqual(missing_reset_audit.controller_availability.numerator, 0)
            self.assertEqual(missing_reset_audit.mission_success.numerator, 0)

            hidden_reset_records = copy.deepcopy(missing_reset_records)
            hidden_frame_zero = next(
                row
                for row in hidden_reset_records
                if row.get("record_type") == "controller_interval"
                and row["frame_index"] == 0
            )
            hidden_frame_zero["runtime"]["reset"] = {
                "attempted": False,
                "guard_status": "not-attempted",
                "committed": False,
            }
            hidden_reset_audit = audit_evidence_denominators(
                hidden_reset_records, [schedule], depths
            )
            self.assertEqual(
                hidden_reset_audit.controller_availability.numerator, 0
            )
            self.assertEqual(hidden_reset_audit.mission_success.numerator, 0)

            forged_reset_records = copy.deepcopy(records)
            forged_reset_record = next(
                row for row in forged_reset_records if row.get("record_type") == "reset"
            )
            forged_reset_record["runtime"]["local_hard_qps"][0]["problem"][
                "bounds"
            ][0]["limit"] = 24.0
            forged_reset_audit = audit_evidence_denominators(
                forged_reset_records, [schedule], depths
            )
            self.assertEqual(forged_reset_audit.controller_availability.numerator, 0)
            self.assertEqual(forged_reset_audit.mission_success.numerator, 0)

            incomplete_records = copy.deepcopy(records)
            incomplete_controller = next(
                row
                for row in incomplete_records
                if row.get("record_type") == "controller_interval"
                and row["frame_index"] == 1
            )
            incomplete_controller["runtime"]["complete"] = False
            incomplete_audit = audit_evidence_denominators(
                incomplete_records, [schedule], depths
            )
            self.assertEqual(incomplete_audit.controller_availability.numerator, 1)
            self.assertEqual(incomplete_audit.controller_availability.denominator, 2)
            self.assertEqual(incomplete_audit.mission_success.numerator, 0)

            false_terminal_records = copy.deepcopy(records)
            next(
                row
                for row in false_terminal_records
                if row.get("record_type") == "mission_terminal"
            )["runtime"]["success"] = False
            false_terminal_audit = audit_evidence_denominators(
                false_terminal_records, [schedule], depths
            )
            self.assertEqual(false_terminal_audit.mission_success.numerator, 0)

            missing_terminal_records = [
                row
                for row in records
                if row.get("record_type") != "mission_terminal"
            ]
            missing_terminal_audit = audit_evidence_denominators(
                missing_terminal_records, [schedule], depths
            )
            self.assertEqual(missing_terminal_audit.mission_success.denominator, 1)
            self.assertEqual(missing_terminal_audit.mission_success.numerator, 0)

            controller0 = next(row for row in controllers if row["frame_index"] == 0)
            endpoint0 = [row for row in endpoints if row["frame_index"] == 0]

            missing_version = copy.deepcopy(endpoint0)
            del missing_version[0]["snapshot_version"]
            self.assertIn(
                "endpoint_schema",
                reconstruct_controller_primitives(
                    controller0,
                    missing_version,
                    expected_endpoint_count=232,
                    expected_reconstructed_count=119,
                ).integrity_errors,
            )

            missing_edge = endpoint0[:-1]
            missing_edge_errors = reconstruct_controller_primitives(
                controller0,
                missing_edge,
                expected_endpoint_count=232,
                expected_reconstructed_count=119,
            ).integrity_errors
            self.assertIn("endpoint_cardinality", missing_edge_errors)
            self.assertTrue(
                any(error.startswith("endpoint_closure:") for error in missing_edge_errors)
            )

            wrong_exact_edge = copy.deepcopy(endpoint0)
            replaced = next(
                row
                for row in wrong_exact_edge
                if row["edge"]["kind"] == "localization"
                and row["edge"]["base_id"] == 0
                and row["edge"]["low"] == 1
            )
            replaced["edge"]["base_id"] = 2
            self.assertIn(
                "endpoint_universe",
                reconstruct_controller_primitives(
                    controller0, wrong_exact_edge, 232, 119
                ).integrity_errors,
            )

            missing_input = copy.deepcopy(controller0)
            missing_input["runtime"]["nodes"][0]["applied_command"].pop()
            self.assertEqual(
                reconstruct_controller_primitives(
                    missing_input, endpoint0, 232, 119
                ).integrity_errors,
                ("controller_schema",),
            )

            bad_qp = copy.deepcopy(controller0)
            bad_qp["runtime"]["nodes"][0]["normal_qp"]["status"] = "infeasible"
            self.assertTrue(
                any(
                    error.startswith("qp_status:1:normal_qp")
                    for error in reconstruct_controller_primitives(
                        bad_qp, endpoint0, 232, 119
                    ).integrity_errors
                )
            )

            for field, replacement, prefix in (
                ("coefficient", [0.0, 0.0], "endpoint_coefficient:"),
                ("constant", endpoint0[0]["constant"] + 1.0, "endpoint_constant:"),
                ("allocation", 0.25, "endpoint_allocation:"),
            ):
                tampered_rows = copy.deepcopy(endpoint0)
                tampered_rows[0][field] = replacement
                self.assertTrue(
                    any(
                        error.startswith(prefix)
                        for error in reconstruct_controller_primitives(
                            controller0, tampered_rows, 232, 119
                        ).integrity_errors
                    ),
                    field,
                )

            bad_expected = copy.deepcopy(controller0)
            bad_expected["runtime"]["expected_endpoint_row_count"] = 999
            self.assertIn(
                "serialized_expected_endpoint_count",
                reconstruct_controller_primitives(
                    bad_expected, endpoint0, 232, 119
                ).integrity_errors,
            )

            unsafe_geometry = copy.deepcopy(controller0)
            node1 = next(
                node for node in unsafe_geometry["runtime"]["nodes"]
                if node["robot_id"] == 1
            )
            node2 = next(
                node for node in unsafe_geometry["runtime"]["nodes"]
                if node["robot_id"] == 2
            )
            node2["interface_estimate"] = [
                node1["interface_estimate"][0] + 0.1,
                node1["interface_estimate"][1],
            ]
            unsafe_errors = reconstruct_controller_primitives(
                unsafe_geometry, endpoint0, 232, 119
            ).integrity_errors
            self.assertTrue(
                any(error.startswith("unsafe_local_residual:") for error in unsafe_errors)
            )
            self.assertTrue(
                any(error.startswith("unsafe_full_residual:") for error in unsafe_errors)
            )

            over_bound = copy.deepcopy(controller0)
            over_bound_node = over_bound["runtime"]["nodes"][0]
            over_bound_node["applied_command"][0] = 25.1
            over_bound_node["normal_qp"]["solution"][0] = 25.1
            self.assertTrue(
                any(
                    error.startswith("input_bound:")
                    for error in reconstruct_controller_primitives(
                        over_bound, endpoint0, 232, 119
                    ).integrity_errors
                )
            )

            forged_summaries = copy.deepcopy(controller0)
            forged_summaries["runtime"]["component_maxima"]["vx"] = 0.0
            forged_summaries["runtime"]["local_residual_minimum"] += 1.0
            forged_summaries["runtime"]["reconstructed_residual_minimum"] += 1.0
            forged_summaries["runtime"]["complete_finite_snapshot"] = False
            forged_errors = reconstruct_controller_primitives(
                forged_summaries, endpoint0, 232, 119
            ).integrity_errors
            self.assertIn("serialized_component_maxima", forged_errors)
            self.assertIn("serialized_local_residual_minimum", forged_errors)
            self.assertIn("serialized_full_residual_minimum", forged_errors)
            self.assertIn("complete_finite_snapshot", forged_errors)

            forged_problem = copy.deepcopy(controller0)
            forged_problem["runtime"]["nodes"][0]["normal_problem"][
                "bounds"
            ][0]["limit"] = 24.0
            self.assertTrue(
                any(
                    error.startswith("hard_problem_hash:")
                    for error in reconstruct_controller_primitives(
                        forged_problem, endpoint0, 232, 119
                    ).integrity_errors
                )
            )

            try:
                malformed_endpoint_result = reconstruct_controller_primitives(
                    controller0, 7, 232, 119
                )
            except Exception:
                malformed_endpoint_result = None
            self.assertIsNotNone(malformed_endpoint_result)
            self.assertTrue(malformed_endpoint_result.integrity_errors)

            forged_hard_solution = copy.deepcopy(controller0)
            forged_hard_solution["runtime"]["nodes"][0]["hard_only_qp"][
                "solution"
            ][0] = 100.0
            forged_hard_solution["runtime"]["nodes"][0]["hard_only_qp"][
                "minimum_residual"
            ] = 1.0
            hard_solution_errors = reconstruct_controller_primitives(
                forged_hard_solution, endpoint0, 232, 119
            ).integrity_errors
            self.assertTrue(
                any(
                    error.startswith("hard_only_solution:")
                    for error in hard_solution_errors
                )
            )

            forged_qp_minimum = copy.deepcopy(controller0)
            forged_qp_minimum["runtime"]["nodes"][0]["normal_qp"][
                "minimum_residual"
            ] += 1.0
            self.assertTrue(
                any(
                    error.startswith("qp_minimum_residual:")
                    for error in reconstruct_controller_primitives(
                        forged_qp_minimum, endpoint0, 232, 119
                    ).integrity_errors
                )
            )

            swapped_base_geometry = copy.deepcopy(controller0)
            base_references = swapped_base_geometry["runtime"]["nodes"][0][
                "references"
            ]
            self.assertLess(base_references[0]["canonical_reference_id"], 0)
            self.assertLess(base_references[1]["canonical_reference_id"], 0)
            for field in ("direction", "distance", "ranging_variance"):
                base_references[0][field], base_references[1][field] = (
                    base_references[1][field],
                    base_references[0][field],
                )
            geometry_errors = reconstruct_controller_primitives(
                swapped_base_geometry, endpoint0, 232, 119
            ).integrity_errors
            self.assertTrue(
                any(
                    error.startswith("reference_geometry:")
                    for error in geometry_errors
                )
            )

            bad_reset = copy.deepcopy(resets[0])
            bad_reset["runtime"]["descendant_closure"].pop()
            self.assertIn(
                "reset_descendant_closure",
                audit_reset_primitives(bad_reset, tuple(range(1, 15)), 119),
            )
            forged_reset_problem = copy.deepcopy(resets[0])
            forged_reset_problem["runtime"]["local_hard_qps"][0]["problem"][
                "bounds"
            ][0]["limit"] = 24.0
            self.assertTrue(
                any(
                    error.startswith("reset_hard_problem_hash:")
                    for error in audit_reset_primitives(
                        forged_reset_problem, tuple(range(1, 15)), 119
                    )
                )
            )

            forged_reset_solution = copy.deepcopy(resets[0])
            forged_reset_solution["runtime"]["local_hard_qps"][0][
                "solution"
            ][0] = 100.0
            forged_reset_solution["runtime"]["local_hard_qps"][0][
                "minimum_residual"
            ] = 1.0
            self.assertTrue(
                any(
                    error.startswith("reset_qp_solution:")
                    for error in audit_reset_primitives(
                        forged_reset_solution, tuple(range(1, 15)), 119
                    )
                )
            )
            self.assertIn("seconds elapsed", result.stderr)
            self.assertIn("[OUTPUT_DIR]", result.stderr)
            self.assertEqual(
                sorted(path.name for path in root.iterdir()),
                ["config.json"],
                "evidence mode must skip legacy output files",
            )

    def test_marker_declared_v2_real_stream_has_exact_policy_for_every_node(self):
        with tempfile.TemporaryDirectory(prefix="cbf-evidence-v2-") as temporary:
            root = Path(temporary)
            config_path = root / "config.json"
            config_path.write_text(
                json.dumps(evidence_fixture(root, v2=True)), encoding="utf-8"
            )
            result = subprocess.run(
                [str(SWARM_BINARY), str(config_path)], cwd=ROOT, text=True,
                capture_output=True, timeout=30, check=False,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            records = [json.loads(line) for line in result.stdout.splitlines() if line]
            controllers = [
                row for row in records
                if row.get("record_type") == "controller_interval"
            ]
            endpoints = [
                row for row in records if row.get("record_type") == "endpoint_row"
            ]
            self.assertTrue(controllers)
            for controller in controllers:
                self.assertTrue(
                    validate_controller_primitive_schema(controller),
                    {
                        "node": evidence_schema._valid_node(
                            controller["runtime"]["nodes"][0], 1, 1
                        ),
                        "problem": evidence_schema._valid_hard_problem(
                            controller["runtime"]["nodes"][0]["normal_problem"], 1
                        ),
                        "policy": evidence_schema._valid_hard_interior_selection(
                            controller["runtime"]["nodes"][0]["hard_interior_selection"]
                        ),
                    },
                )
                nodes = controller["runtime"]["nodes"]
                self.assertEqual(len(nodes), 14)
                self.assertTrue(all("hard_interior_selection" in node for node in nodes))
                self.assertTrue(all(
                    set(node["hard_interior_selection"]) == {
                        "mode", "fraction", "cap_mps",
                        "feasibility_tolerance_mps",
                        "planar_chebyshev_radius_mps",
                        "enforced_floor_mps",
                        "minimum_original_hard_residual_mps",
                    }
                    for node in nodes
                ))
            controller0 = controllers[0]
            endpoint0 = [
                row for row in endpoints
                if row["frame_index"] == controller0["frame_index"]
            ]
            for field, replacement in (
                ("fraction", 0.2), ("cap_mps", 0.2),
                ("feasibility_tolerance_mps", 1e-8),
                ("planar_chebyshev_radius_mps", 0.0),
                ("enforced_floor_mps", 0.0),
                ("minimum_original_hard_residual_mps", 0.0),
            ):
                forged = copy.deepcopy(controller0)
                forged["runtime"]["nodes"][0]["hard_interior_selection"][field] = replacement
                self.assertTrue(any(
                    error.startswith("interior_")
                    for error in reconstruct_controller_primitives(
                        forged, endpoint0, 232, 119
                    ).integrity_errors
                ), field)
            missing = copy.deepcopy(controller0)
            del missing["runtime"]["nodes"][0]["hard_interior_selection"]
            self.assertFalse(validate_controller_primitive_schema(missing))
            integral_tokens = copy.deepcopy(controller0)
            integral_policy = integral_tokens["runtime"]["nodes"][0][
                "hard_interior_selection"
            ]
            integral_policy["planar_chebyshev_radius_mps"] = int(
                integral_policy["planar_chebyshev_radius_mps"]
            )
            integral_policy["minimum_original_hard_residual_mps"] = int(
                integral_policy["minimum_original_hard_residual_mps"]
            )
            self.assertFalse(validate_controller_primitive_schema(integral_tokens))
            self.assertTrue(
                reconstruct_controller_primitives(
                    integral_tokens, endpoint0, 232, 119
                ).integrity_errors
            )
            hard_only_integer = copy.deepcopy(controller0)
            hard_only_integer["runtime"]["nodes"][0]["hard_only_problem"][
                "planar_component_max"
            ] = 25
            self.assertFalse(validate_controller_primitive_schema(hard_only_integer))
            self.assertTrue(
                reconstruct_controller_primitives(
                    hard_only_integer, endpoint0, 232, 119
                ).integrity_errors
            )
            from scripts.diagnostics.analyze_qualified_closure_campaign import (
                _controller_frame_metrics,
            )
            with self.assertRaises(ValueError):
                _controller_frame_metrics(hard_only_integer, endpoint0)
            yaw = copy.deepcopy(controller0)
            yaw["runtime"]["nodes"][0]["normal_problem"]["rows"][0][
                "coefficients"
            ][2] = 0.1
            self.assertTrue(any(
                error.startswith("interior_policy:")
                for error in reconstruct_controller_primitives(
                    yaw, endpoint0, 232, 119
                ).integrity_errors
            ))

    def test_truth_robot_universe_is_exact_and_denominator_fail_closed(self):
        with tempfile.TemporaryDirectory(
            prefix="cbf-evidence-truth-universe-"
        ) as temporary:
            root = Path(temporary)
            config_path = root / "config.json"
            config_path.write_text(
                json.dumps(evidence_fixture(root)), encoding="utf-8"
            )
            result = subprocess.run(
                [str(SWARM_BINARY), str(config_path)],
                cwd=ROOT,
                text=True,
                capture_output=True,
                timeout=30,
                check=False,
            )

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            records = [
                json.loads(line)
                for line in result.stdout.splitlines()
                if line
            ]
            schedule = FrozenMissionSchedule(
                campaign_id="two-frame-fixture",
                trajectory_seed=101,
                range_noise_seed=201,
                frames=2,
                robots=tuple(range(1, 15)),
                estimator_conditions=("dynamic_primary", "fixed_fim_ablation"),
                controller_condition="dynamic_primary",
            )
            depths = {
                condition: {
                    robot_id: (robot_id - 1) % 7 + 1
                    for robot_id in schedule.robots
                }
                for condition in schedule.estimator_conditions
            }

            for mutation in ("duplicate", "out_of_range"):
                with self.subTest(truth_robot_id=mutation):
                    forged_truth_records = copy.deepcopy(records)
                    forged_truth_controller = next(
                        row
                        for row in forged_truth_records
                        if row.get("record_type") == "controller_interval"
                        and row["frame_index"] == 1
                    )
                    truth = forged_truth_controller["analyzer_only"]["truth"]
                    expected_nodes = forged_truth_controller["runtime"][
                        "expected_node_count"
                    ]
                    truth[-1]["robot_id"] = (
                        truth[-2]["robot_id"]
                        if mutation == "duplicate"
                        else expected_nodes + 1
                    )

                    self.assertFalse(
                        validate_controller_primitive_schema(
                            forged_truth_controller
                        )
                    )
                    forged_truth_audit = audit_evidence_denominators(
                        forged_truth_records, [schedule], depths
                    )
                    self.assertEqual(
                        forged_truth_audit.controller_availability.numerator,
                        1,
                    )
                    self.assertEqual(
                        forged_truth_audit.mission_success.numerator, 0
                    )

    def test_failed_interval_emits_abort_without_stale_command_or_completion(self):
        with tempfile.TemporaryDirectory(prefix="cbf-evidence-abort-") as temporary:
            root = Path(temporary)
            config = evidence_fixture(root)
            config["searching"]["method"] = 7
            config_path = root / "config.json"
            config_path.write_text(json.dumps(config), encoding="utf-8")

            result = subprocess.run(
                [str(SWARM_BINARY), str(config_path)],
                cwd=ROOT,
                text=True,
                capture_output=True,
                timeout=30,
                check=False,
            )

            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            records = [json.loads(line) for line in result.stdout.splitlines() if line]
            controllers = [
                row for row in records if row.get("record_type") == "controller_interval"
            ]
            self.assertEqual(len(controllers), 1)
            abort = controllers[0]
            self.assertFalse(abort["runtime"]["complete"])
            self.assertNotEqual(abort["runtime"]["abort_reason"], "")
            self.assertEqual(abort["runtime"]["observed_endpoint_row_count"], 0)
            self.assertTrue(
                all(node["applied_command"] is None for node in abort["runtime"]["nodes"])
            )
            terminals = [
                row for row in records if row.get("record_type") == "mission_terminal"
            ]
            self.assertEqual(len(terminals), 1)
            self.assertFalse(terminals[0]["runtime"]["success"])
            self.assertEqual(terminals[0]["runtime"]["completed_intervals"], 0)
            self.assertIn("[SIMULATION_ERROR]", result.stderr)

    def test_mid_emission_failure_retains_prefix_and_emits_incomplete_marker(self):
        with tempfile.TemporaryDirectory(prefix="cbf-evidence-mid-failure-") as temporary:
            root = Path(temporary)
            config = evidence_fixture(root)
            config["execute"]["time-total"] = 0.5
            config_path = root / "config.json"
            config_path.write_text(json.dumps(config), encoding="utf-8")
            environment = os.environ.copy()
            environment["CBF_EVIDENCE_TEST_FAIL_AFTER_ENDPOINT_ROWS"] = "5"

            result = subprocess.run(
                [str(SWARM_BINARY), str(config_path)],
                cwd=ROOT,
                text=True,
                capture_output=True,
                timeout=30,
                check=False,
                env=environment,
            )

            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            records = [json.loads(line) for line in result.stdout.splitlines() if line]
            self.assertEqual(
                sum(row.get("record_type") == "endpoint_row" for row in records), 5
            )
            aborts = [
                row for row in records if row.get("record_type") == "controller_interval"
            ]
            self.assertEqual(len(aborts), 1)
            self.assertFalse(aborts[0]["runtime"]["complete"])
            self.assertEqual(aborts[0]["runtime"]["observed_endpoint_row_count"], 5)
            self.assertFalse(records[-1]["runtime"]["success"])

    def test_failure_hooks_are_inert_outside_explicit_test_campaign(self):
        with tempfile.TemporaryDirectory(prefix="cbf-evidence-hook-scope-") as temporary:
            root = Path(temporary)
            config = evidence_fixture(root)
            config["execute"]["time-total"] = 0.5
            config["evidence-stream"]["campaign-id"] = "development-v1"
            config_path = root / "config.json"
            config_path.write_text(json.dumps(config), encoding="utf-8")
            environment = os.environ.copy()
            environment["CBF_EVIDENCE_TEST_FAIL_AFTER_ENDPOINT_ROWS"] = "5"
            environment["CBF_EVIDENCE_TEST_FAIL_BOOTSTRAP"] = "1"

            result = subprocess.run(
                [str(SWARM_BINARY), str(config_path)],
                cwd=ROOT,
                text=True,
                capture_output=True,
                timeout=30,
                check=False,
                env=environment,
            )

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            records = [json.loads(line) for line in result.stdout.splitlines() if line]
            self.assertEqual(
                sum(row.get("record_type") == "endpoint_row" for row in records),
                232,
            )
            self.assertTrue(records[-1]["runtime"]["success"])

    def test_failure_hooks_are_absent_from_no_hook_build(self):
        self.assertTrue(
            NO_HOOK_SWARM_BINARY.is_file(),
            f"no-hook Swarm binary not found: {NO_HOOK_SWARM_BINARY}",
        )
        with tempfile.TemporaryDirectory(prefix="cbf-evidence-no-hook-") as temporary:
            root = Path(temporary)
            config = evidence_fixture(root)
            config["execute"]["time-total"] = 0.5
            config_path = root / "config.json"
            config_path.write_text(json.dumps(config), encoding="utf-8")
            environment = os.environ.copy()
            environment["CBF_EVIDENCE_TEST_FAIL_AFTER_ENDPOINT_ROWS"] = "5"
            environment["CBF_EVIDENCE_TEST_FAIL_BOOTSTRAP"] = "1"

            result = subprocess.run(
                [str(NO_HOOK_SWARM_BINARY), str(config_path)],
                cwd=ROOT,
                text=True,
                capture_output=True,
                timeout=30,
                check=False,
                env=environment,
            )

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            records = [json.loads(line) for line in result.stdout.splitlines() if line]
            self.assertEqual(
                sum(row.get("record_type") == "endpoint_row" for row in records),
                232,
            )
            self.assertTrue(records[-1]["runtime"]["success"])

    def test_bootstrap_failure_emits_stage_faithful_reset_abort_and_terminal(self):
        with tempfile.TemporaryDirectory(prefix="cbf-evidence-bootstrap-failure-") as temporary:
            root = Path(temporary)
            config_path = root / "config.json"
            config_path.write_text(
                json.dumps(evidence_fixture(root)), encoding="utf-8"
            )
            environment = os.environ.copy()
            environment["CBF_EVIDENCE_TEST_FAIL_BOOTSTRAP"] = "1"

            result = subprocess.run(
                [str(SWARM_BINARY), str(config_path)],
                cwd=ROOT,
                text=True,
                capture_output=True,
                timeout=30,
                check=False,
                env=environment,
            )

            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            records = [json.loads(line) for line in result.stdout.splitlines() if line]
            self.assertEqual(
                sum(row.get("record_type") == "initialization" for row in records), 14
            )
            reset = next(row for row in records if row.get("record_type") == "reset")
            self.assertEqual(reset["runtime"]["stage"], "proposal_started")
            self.assertEqual(
                audit_reset_primitives(reset, tuple(range(1, 15)), 119), ()
            )
            abort = next(
                row for row in records if row.get("record_type") == "controller_interval"
            )
            self.assertFalse(abort["runtime"]["complete"])
            self.assertEqual(records[-1]["record_type"], "mission_terminal")
            self.assertFalse(records[-1]["runtime"]["success"])


if __name__ == "__main__":
    unittest.main()
