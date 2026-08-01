import copy
import unittest

from scripts.diagnostics.qualified_closure_evidence import (
    CanonicalEdgeId,
    FrozenMissionSchedule,
    _reconstruct_controller_primitives_impl,
    audit_reset_primitives,
    audit_evidence_denominators,
    reconstruct_controller_primitives,
    synthesize_missing_mission,
    validate_controller_primitive_schema,
    validate_endpoint_primitive_schema,
    validate_estimator_tuple_schema,
    validate_initialization_schema,
    validate_mission_terminal_schema,
)


def bounded_hard_problem(owner, row, hard_problem_id):
    return {
        "owner": owner,
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
        "rows": [row],
        "hard_problem_id": hard_problem_id,
    }


def localization_problem_row(constant=77.89339828220179):
    return {
        "edge": {
            "kind": "localization",
            "low": 1,
            "high": 1,
            "base_id": 0,
        },
        "owner": 1,
        "name": "fixedCommCBF(base-0)",
        "coefficients": [-1.0, 0.0, 0.0],
        "constant": constant,
        "post_reset_barrier": 88.5,
        "snapshot_version": 1,
        "allocation_version": 1,
    }


def collision_hard_problem(owner):
    if owner == 1:
        coefficients = [-1.0, 0.0, 0.0]
        name = "safetyCBF(#2)"
        hard_problem_id = "096db1cc7b14a73f"
    else:
        coefficients = [1.0, 0.0, 0.0]
        name = "safetyCBF(#1)"
        hard_problem_id = "f7f4e8093c330224"
    row = {
        "edge": {
            "kind": "collision",
            "low": 1,
            "high": 2,
            "base_id": -1,
        },
        "owner": owner,
        "name": name,
        "coefficients": coefficients,
        "constant": 0.6,
        "post_reset_barrier": 1.8,
        "snapshot_version": 1,
        "allocation_version": 1,
    }
    return bounded_hard_problem(owner, row, hard_problem_id)


class EvidenceUniverseTests(unittest.TestCase):
    def schedule(self, frames=1000):
        return FrozenMissionSchedule(
            campaign_id="development-v1",
            trajectory_seed=101,
            range_noise_seed=201,
            frames=frames,
            robots=tuple(range(1, 15)),
            estimator_conditions=("dynamic_primary", "fixed_fim_ablation"),
            controller_condition="dynamic_primary",
        )

    def test_launch_failed_mission_contributes_every_frozen_key(self):
        schedule = self.schedule()

        rows = synthesize_missing_mission(schedule, reason="launch_failed")

        self.assertEqual(len(rows.estimator), 2 * 999 * 14)
        self.assertEqual(len(rows.initialization), 14)
        self.assertEqual(len(rows.controller), 1000)
        self.assertFalse(rows.mission.success)

    def test_frozen_universe_keys_are_ordered_complete_and_lazy(self):
        rows = synthesize_missing_mission(self.schedule(frames=2), "launch_failed")

        self.assertEqual(
            rows.initialization[0],
            ("development-v1", 101, 201, 0, 1),
        )
        self.assertEqual(
            rows.initialization[-1],
            ("development-v1", 101, 201, 0, 14),
        )
        self.assertEqual(
            rows.estimator[0],
            ("development-v1", "dynamic_primary", 101, 201, 1, 1),
        )
        self.assertEqual(
            rows.estimator[-1],
            ("development-v1", "fixed_fim_ablation", 101, 201, 1, 14),
        )
        self.assertEqual(
            rows.controller[-1],
            ("development-v1", "dynamic_primary", 101, 201, 1),
        )
        self.assertEqual(len(rows.endpoint), 232 * 2)
        self.assertEqual(len(rows.reconstructed), 119 * 2)
        self.assertEqual(
            rows.endpoint[0],
            (
                "development-v1",
                "dynamic_primary",
                101,
                201,
                0,
                CanonicalEdgeId("localization", 1, 1, 0),
                1,
            ),
        )
        self.assertEqual(
            rows.endpoint[-1],
            (
                "development-v1",
                "dynamic_primary",
                101,
                201,
                1,
                CanonicalEdgeId("collision", 13, 14, -1),
                14,
            ),
        )
        self.assertEqual(len(rows.reset), 0)
        self.assertFalse(hasattr(rows.endpoint, "append"))

    def test_frozen_endpoint_universe_matches_registered_edges_and_owner_counts(self):
        rows = synthesize_missing_mission(self.schedule(frames=1), "launch_failed")
        endpoints = [(key[-2], key[-1]) for key in rows.endpoint]
        localization_edges = {edge for edge, _owner in endpoints if edge.kind == "localization"}
        expected_base_edges = {
            CanonicalEdgeId("localization", 1, 1, 0),
            CanonicalEdgeId("localization", 1, 1, 1),
            CanonicalEdgeId("localization", 2, 2, 1),
            CanonicalEdgeId("localization", 8, 8, 1),
            CanonicalEdgeId("localization", 8, 8, 2),
            CanonicalEdgeId("localization", 9, 9, 1),
        }
        expected_uav_edges = {
            CanonicalEdgeId("localization", 1, 2, -1),
            CanonicalEdgeId("localization", 1, 3, -1),
            CanonicalEdgeId("localization", 2, 3, -1),
            CanonicalEdgeId("localization", 2, 4, -1),
            CanonicalEdgeId("localization", 3, 4, -1),
            CanonicalEdgeId("localization", 3, 5, -1),
            CanonicalEdgeId("localization", 4, 5, -1),
            CanonicalEdgeId("localization", 4, 6, -1),
            CanonicalEdgeId("localization", 5, 6, -1),
            CanonicalEdgeId("localization", 5, 7, -1),
            CanonicalEdgeId("localization", 6, 7, -1),
            CanonicalEdgeId("localization", 8, 9, -1),
            CanonicalEdgeId("localization", 8, 10, -1),
            CanonicalEdgeId("localization", 9, 10, -1),
            CanonicalEdgeId("localization", 9, 11, -1),
            CanonicalEdgeId("localization", 10, 11, -1),
            CanonicalEdgeId("localization", 10, 12, -1),
            CanonicalEdgeId("localization", 11, 12, -1),
            CanonicalEdgeId("localization", 11, 13, -1),
            CanonicalEdgeId("localization", 12, 13, -1),
            CanonicalEdgeId("localization", 12, 14, -1),
            CanonicalEdgeId("localization", 13, 14, -1),
        }
        self.assertEqual(localization_edges, expected_base_edges | expected_uav_edges)
        owner_counts = [sum(owner == robot_id for _edge, owner in endpoints) for robot_id in range(1, 15)]
        self.assertEqual(owner_counts, [17, 17, 17, 17, 17, 16, 15] * 2)


class ControllerPrimitiveSchemaTests(unittest.TestCase):
    def test_reduced_controller_universe_is_rejected_but_endpoint_is_complete(self):
        reference = {
            "canonical_reference_id": -1,
            "reference_kind": "base",
            "direction": [1.0, 0.0],
            "distance": 10.0,
            "ranging_variance": 0.25,
            "predecessor_local_index": 0,
            "predecessor_snapshot_version": 1,
            "predecessor_covariance": [[0.0, 0.0], [0.0, 0.0]],
            "predecessor_covariance_rate_bound": 0.0,
            "predecessor_speed_bound": 0.0,
        }
        node = {
            "robot_id": 1,
            "local_index": 1,
            "snapshot_version": 1,
            "allocation_version": 1,
            "interface_estimate": [-1450.0, -300.0],
            "node_component_bound": 25.0,
            "references": [
                reference,
                {
                    **reference,
                    "canonical_reference_id": -2,
                    "direction": [0.0, 1.0],
                },
            ],
            "information": [[4.0, 0.0], [0.0, 4.0]],
            "covariance": [[0.25, 0.0], [0.0, 0.25]],
            "covariance_derivative": [[0.01, 0.0], [0.0, 0.01]],
            "covariance_rate_bound": 0.1,
            "epsilon": 1.5,
            "dplus_epsilon": 0.01,
            "nu_inst": 0.02,
            "bar_nu": 0.1,
            "applied_command": [1.0, 2.0, 0.1],
            "normal_qp": {
                "status": "optimal",
                "minimum_residual": 0.5,
                "hard_problem_id": "problem-1",
                "solution": [1.0, 2.0, 0.1],
            },
            "hard_only_qp": {
                "status": "optimal",
                "minimum_residual": 0.5,
                "hard_problem_id": "problem-1",
                "solution": [0.0, 0.0, 0.0],
            },
            "normal_problem": bounded_hard_problem(
                1, localization_problem_row(8984.9), "problem-1"
            ),
            "hard_only_problem": bounded_hard_problem(
                1, localization_problem_row(8984.9), "problem-1"
            ),
            "committed_hard_problem_id": "problem-1",
            "consumed_hard_problem_id": "problem-1",
        }
        controller = {
            "record_type": "controller_interval",
            "schema_version": "cbf2026-qualified-evidence-v1",
            "campaign_id": "development-v1",
            "condition": "dynamic_primary",
            "trajectory_seed": 101,
            "range_noise_seed": 201,
            "frame_index": 0,
            "runtime": {
                "snapshot_version": 1,
                "allocation_version": 1,
                "nodes": [node],
                "expected_node_count": 1,
                "expected_endpoint_row_count": 1,
                "expected_reconstructed_row_count": 1,
                "observed_endpoint_row_count": 1,
                "complete_finite_snapshot": True,
                "reset": {
                    "attempted": True,
                    "guard_status": "accepted",
                    "committed": True,
                },
                "component_maxima": {"vx": 1.0, "vy": 2.0, "yaw_rate": 0.1},
                "local_residual_minimum": 0.5,
                "reconstructed_residual_minimum": 0.5,
                "abort_reason": "",
                "complete": True,
            },
            "analyzer_only": {
                "truth": [{"robot_id": 1, "position": [-1450.0, -300.0]}]
            },
        }
        endpoint = {
            "record_type": "endpoint_row",
            "schema_version": "cbf2026-qualified-evidence-v1",
            "campaign_id": "development-v1",
            "condition": "dynamic_primary",
            "trajectory_seed": 101,
            "range_noise_seed": 201,
            "frame_index": 0,
            "edge": {
                "kind": "localization",
                "low": 1,
                "high": 1,
                "base_id": 0,
                "threshold": 1000.0,
                "base_position": [-1550.0, -300.0],
                "normal": [1.0, 0.0],
                "separation": 100.0,
                "tightened_barrier": 898.5,
                "class_k_coefficient": 10.0,
                "class_k_power": 1,
                "alpha_value": 8985.0,
            },
            "owner": 1,
            "allocation": 1.0,
            "coefficient": [-1.0, 0.0],
            "constant": 8984.9,
            "snapshot_version": 1,
            "allocation_version": 1,
            "owner_applied_command": [1.0, 2.0, 0.1],
            "qp_status": "optimal",
            "hard_problem_id": "problem-1",
        }

        self.assertFalse(validate_controller_primitive_schema(controller))
        self.assertTrue(validate_endpoint_primitive_schema(endpoint))


class ExactLifecycleSchemaTests(unittest.TestCase):
    def initialization(self):
        return {
            "record_type": "initialization",
            "schema_version": "cbf2026-qualified-evidence-v1",
            "campaign_id": "development-v1",
            "condition": "dynamic_primary",
            "trajectory_seed": 101,
            "range_noise_seed": 201,
            "frame_index": 0,
            "robot_id": 1,
            "runtime": {"local_index": 1},
            "analyzer_only": {"truth_position": [-1450.0, -300.0]},
        }

    def estimator(self):
        return {
            "record_type": "estimator_tuple",
            "schema_version": "cbf2026-qualified-evidence-v1",
            "campaign_id": "development-v1",
            "condition": "dynamic_primary",
            "trajectory_seed": 101,
            "range_noise_seed": 201,
            "frame_index": 1,
            "robot_id": 1,
            "depth": 1,
            "output_status": "fresh",
            "estimate": [-1449.5, -300.0],
            "radius": 1.5,
        }

    def terminal(self):
        return {
            "record_type": "mission_terminal",
            "schema_version": "cbf2026-qualified-evidence-v1",
            "campaign_id": "development-v1",
            "condition": "dynamic_primary",
            "trajectory_seed": 101,
            "range_noise_seed": 201,
            "frame_index": 2,
            "runtime": {
                "success": True,
                "reason": "completed",
                "process_outcome": "completed",
                "declared_frames": 2,
                "completed_intervals": 2,
            },
        }

    def test_initialization_schema_rejects_truth_leak_version_nonfinite_and_truncation(self):
        valid = self.initialization()
        self.assertTrue(validate_initialization_schema(valid))

        leaked = self.initialization()
        leaked["runtime"]["interface_position"] = [-1450.0, -300.0]
        self.assertFalse(validate_initialization_schema(leaked))

        wrong_version = self.initialization()
        wrong_version["schema_version"] = "forged"
        self.assertFalse(validate_initialization_schema(wrong_version))

        nonfinite = self.initialization()
        nonfinite["analyzer_only"]["truth_position"][0] = float("nan")
        self.assertFalse(validate_initialization_schema(nonfinite))

        truncated = self.initialization()
        del truncated["runtime"]
        self.assertFalse(validate_initialization_schema(truncated))

    def test_estimator_and_terminal_schemas_are_exact_and_finite(self):
        self.assertTrue(validate_estimator_tuple_schema(self.estimator()))
        self.assertTrue(validate_mission_terminal_schema(self.terminal()))

        nonfinite = self.estimator()
        nonfinite["radius"] = float("inf")
        self.assertFalse(validate_estimator_tuple_schema(nonfinite))

        forged = self.terminal()
        forged["runtime"]["extra"] = True
        self.assertFalse(validate_mission_terminal_schema(forged))


class IndependentReconstructionTests(unittest.TestCase):
    def test_raw_primitives_override_corrupted_serialized_derivatives(self):
        references = [
            {
                "canonical_reference_id": -1,
                "reference_kind": "base",
                "direction": [0.7071067811865475, 0.7071067811865475],
                "distance": 212.13203435596427,
                "ranging_variance": 0.25,
                "predecessor_local_index": 0,
                "predecessor_snapshot_version": 1,
                "predecessor_covariance": [[0.0, 0.0], [0.0, 0.0]],
                "predecessor_covariance_rate_bound": 0.0,
                "predecessor_speed_bound": 0.0,
            },
            {
                "canonical_reference_id": -2,
                "reference_kind": "base",
                "direction": [0.7071067811865475, -0.7071067811865475],
                "distance": 212.13203435596427,
                "ranging_variance": 0.25,
                "predecessor_local_index": 0,
                "predecessor_snapshot_version": 1,
                "predecessor_covariance": [[0.0, 0.0], [0.0, 0.0]],
                "predecessor_covariance_rate_bound": 0.0,
                "predecessor_speed_bound": 0.0,
            },
        ]
        problem_row = localization_problem_row(785.8679656440357)
        problem_row["coefficients"] = [
            -0.7071067811865475, -0.7071067811865475, 0.0
        ]
        problem_row["post_reset_barrier"] = 786.3679656440357
        node = {
            "robot_id": 1,
            "local_index": 1,
            "snapshot_version": 1,
            "allocation_version": 1,
            "interface_estimate": [-1400.0, -150.0],
            "node_component_bound": 25.0,
            "references": references,
            "information": [[999.0, 0.0], [0.0, 999.0]],
            "covariance": [[999.0, 0.0], [0.0, 999.0]],
            "covariance_derivative": [[999.0, 0.0], [0.0, 999.0]],
            "covariance_rate_bound": 999.0,
            "epsilon": 999.0,
            "dplus_epsilon": 999.0,
            "nu_inst": 999.0,
            "bar_nu": 999.0,
            "applied_command": [1.0, 2.0, 0.1],
            "normal_qp": {
                "status": "optimal",
                "minimum_residual": 0.25,
                "hard_problem_id": "3df88f9e7027386a",
                "solution": [1.0, 2.0, 0.1],
            },
            "hard_only_qp": {
                "status": "optimal",
                "minimum_residual": 0.35,
                "hard_problem_id": "3df88f9e7027386a",
                "solution": [0.0, 0.0, 0.0],
            },
            "normal_problem": bounded_hard_problem(
                1, copy.deepcopy(problem_row), "3df88f9e7027386a"
            ),
            "hard_only_problem": bounded_hard_problem(
                1, copy.deepcopy(problem_row), "3df88f9e7027386a"
            ),
            "committed_hard_problem_id": "3df88f9e7027386a",
            "consumed_hard_problem_id": "3df88f9e7027386a",
        }
        controller = {
            "record_type": "controller_interval",
            "schema_version": "cbf2026-qualified-evidence-v1",
            "campaign_id": "development-v1",
            "condition": "dynamic_primary",
            "trajectory_seed": 101,
            "range_noise_seed": 201,
            "frame_index": 0,
            "runtime": {
                "snapshot_version": 1,
                "allocation_version": 1,
                "nodes": [node],
                "expected_node_count": 1,
                "expected_endpoint_row_count": 1,
                "expected_reconstructed_row_count": 1,
                "observed_endpoint_row_count": 1,
                "complete_finite_snapshot": True,
                "reset": {
                    "attempted": True,
                    "guard_status": "accepted",
                    "committed": True,
                },
                "component_maxima": {"vx": 1.0, "vy": 2.0, "yaw_rate": 0.1},
                "local_residual_minimum": 783.7466453004761,
                "reconstructed_residual_minimum": 783.7466453004761,
                "abort_reason": "",
                "complete": True,
            },
            "analyzer_only": {
                "truth": [{"robot_id": 1, "position": [-1400.0, -150.0]}]
            },
        }
        endpoint = {
            "record_type": "endpoint_row",
            "schema_version": "cbf2026-qualified-evidence-v1",
            "campaign_id": "development-v1",
            "condition": "dynamic_primary",
            "trajectory_seed": 101,
            "range_noise_seed": 201,
            "frame_index": 0,
            "edge": {
                "kind": "localization",
                "low": 1,
                "high": 1,
                "base_id": 0,
                "threshold": 1000.0,
                "base_position": [-1550.0, -300.0],
                "normal": [0.7071067811865475, 0.7071067811865475],
                "separation": 212.13203435596427,
                "tightened_barrier": 786.3679656440357,
                "class_k_coefficient": 1.0,
                "class_k_power": 1,
                "alpha_value": 786.3679656440357,
            },
            "owner": 1,
            "allocation": 1.0,
            "coefficient": [-0.7071067811865475, -0.7071067811865475],
            "constant": 785.8679656440357,
            "snapshot_version": 1,
            "allocation_version": 1,
            "owner_applied_command": [1.0, 2.0, 0.1],
            "qp_status": "optimal",
            "hard_problem_id": "3df88f9e7027386a",
        }

        def reconstruct_fixture(candidate):
            return _reconstruct_controller_primitives_impl(
                candidate,
                [endpoint],
                schema_expected_node_count=1,
            )

        result = reconstruct_fixture(controller)
        reconstructed = result.nodes[1]

        self.assertAlmostEqual(reconstructed["information"][0][0], 4.0)
        self.assertAlmostEqual(reconstructed["information"][1][1], 4.0)
        self.assertAlmostEqual(reconstructed["information"][0][1], 0.0)
        self.assertAlmostEqual(reconstructed["covariance"][0][0], 0.25)
        self.assertAlmostEqual(reconstructed["covariance"][1][1], 0.25)
        self.assertAlmostEqual(reconstructed["epsilon"], 1.5)
        self.assertAlmostEqual(
            reconstructed["covariance_derivative"][0][0], -1.0 / 600.0
        )
        self.assertAlmostEqual(reconstructed["dplus_epsilon"], 0.005)
        self.assertAlmostEqual(reconstructed["nu_inst"], 0.005)
        self.assertAlmostEqual(reconstructed["bar_nu"], 0.5)
        self.assertAlmostEqual(
            next(iter(result.local_residuals.values())),
            783.7466453004761,
        )
        self.assertAlmostEqual(
            next(iter(result.full_residuals.values())),
            783.7466453004761,
        )

        non_psd = copy.deepcopy(controller)
        non_psd["runtime"]["nodes"][0]["references"][0][
            "predecessor_covariance"
        ] = [[-0.1, 0.0], [0.0, 0.0]]
        self.assertTrue(
            any(
                error.startswith("reference_covariance_psd:")
                for error in reconstruct_fixture(non_psd).integrity_errors
            )
        )

        cancelled_variance = copy.deepcopy(controller)
        cancelled_variance["runtime"]["nodes"][0]["references"][0][
            "predecessor_covariance"
        ] = [[-0.6, 0.0], [0.0, 0.0]]
        try:
            cancelled = reconstruct_fixture(cancelled_variance).integrity_errors
        except (ArithmeticError, ValueError):
            cancelled = ()
        self.assertTrue(
            any(
                error.startswith("effective_variance:")
                for error in cancelled
            )
        )

        reordered = copy.deepcopy(controller)
        reordered["runtime"]["nodes"][0]["references"].reverse()
        self.assertIn(
            "reference_order:1",
            reconstruct_fixture(reordered).integrity_errors,
        )


class EvidenceDenominatorTests(unittest.TestCase):
    def schedule(self, frames=3):
        return FrozenMissionSchedule(
            campaign_id="development-v1",
            trajectory_seed=101,
            range_noise_seed=201,
            frames=frames,
            robots=tuple(range(1, 15)),
            estimator_conditions=("dynamic_primary", "fixed_fim_ablation"),
            controller_condition="dynamic_primary",
        )

    def test_invalid_truth_excludes_containment_but_frozen_denominators_remain(self):
        schedule = self.schedule()
        records = []
        for robot_id in schedule.robots:
            records.append(
                {
                    "record_type": "initialization",
                    "schema_version": "cbf2026-qualified-evidence-v1",
                    "campaign_id": schedule.campaign_id,
                    "condition": schedule.controller_condition,
                    "trajectory_seed": schedule.trajectory_seed,
                    "range_noise_seed": schedule.range_noise_seed,
                    "frame_index": 0,
                    "robot_id": robot_id,
                    "runtime": {"local_index": (robot_id - 1) % 7 + 1},
                    "analyzer_only": {"truth_position": [0.0, 0.0]},
                }
            )
        records.extend(
            [
                {
                    "record_type": "estimator_tuple",
                    "schema_version": "cbf2026-qualified-evidence-v1",
                    "campaign_id": schedule.campaign_id,
                    "condition": "dynamic_primary",
                    "trajectory_seed": schedule.trajectory_seed,
                    "range_noise_seed": schedule.range_noise_seed,
                    "frame_index": 1,
                    "robot_id": 1,
                    "depth": 1,
                    "output_status": "fresh",
                    "estimate": [0.5, 0.0],
                    "radius": 1.0,
                },
                {
                    "record_type": "estimator_tuple",
                    "schema_version": "cbf2026-qualified-evidence-v1",
                    "campaign_id": schedule.campaign_id,
                    "condition": "dynamic_primary",
                    "trajectory_seed": schedule.trajectory_seed,
                    "range_noise_seed": schedule.range_noise_seed,
                    "frame_index": 2,
                    "robot_id": 1,
                    "depth": 1,
                    "output_status": "fresh",
                    "estimate": [2.0, 0.0],
                    "radius": 1.0,
                },
                {
                    "record_type": "estimator_tuple",
                    "schema_version": "cbf2026-qualified-evidence-v1",
                    "campaign_id": schedule.campaign_id,
                    "condition": "dynamic_primary",
                    "trajectory_seed": schedule.trajectory_seed,
                    "range_noise_seed": schedule.range_noise_seed,
                    "frame_index": 1,
                    "robot_id": 2,
                    "depth": 1,
                    "output_status": "unavailable",
                    "estimate": None,
                    "radius": None,
                },
                {
                    "record_type": "controller_interval",
                    "schema_version": "cbf2026-qualified-evidence-v1",
                    "campaign_id": schedule.campaign_id,
                    "condition": "dynamic_primary",
                    "trajectory_seed": schedule.trajectory_seed,
                    "range_noise_seed": schedule.range_noise_seed,
                    "frame_index": 0,
                    "runtime": {"complete": True},
                    "analyzer_only": {
                        "truth": [
                            {"robot_id": 1, "position": [0.0, 0.0]},
                            {"robot_id": 2, "position": [0.0, 0.0]},
                        ]
                    },
                },
                {
                    "record_type": "controller_interval",
                    "schema_version": "cbf2026-qualified-evidence-v1",
                    "campaign_id": schedule.campaign_id,
                    "condition": "dynamic_primary",
                    "trajectory_seed": schedule.trajectory_seed,
                    "range_noise_seed": schedule.range_noise_seed,
                    "frame_index": 1,
                    "runtime": {"complete": True},
                    "analyzer_only": {
                        "truth": [
                            {"robot_id": 1, "position": [0.0, 0.0]},
                            {"robot_id": 2, "position": [0.0, 0.0]},
                        ]
                    },
                },
                {
                    "record_type": "controller_interval",
                    "schema_version": "cbf2026-qualified-evidence-v1",
                    "campaign_id": schedule.campaign_id,
                    "condition": "dynamic_primary",
                    "trajectory_seed": schedule.trajectory_seed,
                    "range_noise_seed": schedule.range_noise_seed,
                    "frame_index": 2,
                    "runtime": {"complete": True},
                    "analyzer_only": {
                        "truth": [
                            {"robot_id": 1, "position": [0.0, 0.0]},
                            {"robot_id": 2, "position": [0.0, 0.0]},
                        ]
                    },
                },
                {
                    "record_type": "mission_terminal",
                    "schema_version": "cbf2026-qualified-evidence-v1",
                    "campaign_id": schedule.campaign_id,
                    "condition": schedule.controller_condition,
                    "trajectory_seed": schedule.trajectory_seed,
                    "range_noise_seed": schedule.range_noise_seed,
                    "frame_index": 3,
                    "runtime": {
                        "success": True,
                        "reason": "completed",
                        "process_outcome": "completed",
                        "declared_frames": 3,
                        "completed_intervals": 3,
                    },
                },
            ]
        )
        depths = {
            condition: {robot_id: 1 for robot_id in schedule.robots}
            for condition in schedule.estimator_conditions
        }

        audit = audit_evidence_denominators(records, [schedule], depths)

        self.assertEqual(audit.initialization_availability.numerator, 14)
        self.assertEqual(audit.initialization_availability.denominator, 14)
        self.assertEqual(
            audit.conditional_containment[("dynamic_primary", 1)].numerator, 0
        )
        self.assertEqual(
            audit.conditional_containment[("dynamic_primary", 1)].denominator, 0
        )
        self.assertEqual(
            audit.joint_containment[("dynamic_primary", 1)].denominator, 28
        )
        self.assertEqual(
            audit.joint_containment[("fixed_fim_ablation", 1)].denominator, 28
        )
        self.assertEqual(audit.controller_availability.numerator, 0)
        self.assertEqual(audit.controller_availability.denominator, 3)
        self.assertEqual(audit.mission_success.numerator, 0)
        self.assertEqual(audit.mission_success.denominator, 1)

    def test_zero_registered_depth_denominator_is_explicit_not_divided(self):
        schedule = self.schedule(frames=1)
        depths = {
            "dynamic_primary": {robot_id: 1 for robot_id in schedule.robots},
            "fixed_fim_ablation": {robot_id: 1 for robot_id in schedule.robots},
        }

        audit = audit_evidence_denominators([], [schedule], depths)

        self.assertEqual(
            audit.joint_containment[("dynamic_primary", 1)].denominator, 0
        )
        self.assertIsNone(
            audit.joint_containment[("dynamic_primary", 1)].value
        )
        self.assertIn(
            "zero_depth_denominator:dynamic_primary:1",
            audit.integrity_errors,
        )

    def test_malformed_initialization_and_estimator_rows_do_not_enter_numerators(self):
        schedule = self.schedule(frames=2)
        depths = {
            condition: {robot_id: 1 for robot_id in schedule.robots}
            for condition in schedule.estimator_conditions
        }
        initialization = [
            {
                "record_type": "initialization",
                "schema_version": "cbf2026-qualified-evidence-v1",
                "campaign_id": schedule.campaign_id,
                "condition": schedule.controller_condition,
                "trajectory_seed": schedule.trajectory_seed,
                "range_noise_seed": schedule.range_noise_seed,
                "frame_index": 0,
                "robot_id": robot_id,
                "runtime": {"local_index": (robot_id - 1) % 7 + 1},
                "analyzer_only": {"truth_position": [0.0, 0.0]},
            }
            for robot_id in schedule.robots
        ]
        initialization[0]["schema_version"] = "forged"
        estimator = {
            "record_type": "estimator_tuple",
            "schema_version": "forged",
            "campaign_id": schedule.campaign_id,
            "condition": schedule.controller_condition,
            "trajectory_seed": schedule.trajectory_seed,
            "range_noise_seed": schedule.range_noise_seed,
            "frame_index": 1,
            "robot_id": 1,
            "depth": 1,
            "output_status": "fresh",
            "estimate": [0.0, 0.0],
            "radius": 1.0,
        }
        truth = {
            "record_type": "controller_interval",
            "schema_version": "cbf2026-qualified-evidence-v1",
            "campaign_id": schedule.campaign_id,
            "condition": schedule.controller_condition,
            "trajectory_seed": schedule.trajectory_seed,
            "range_noise_seed": schedule.range_noise_seed,
            "frame_index": 1,
            "runtime": {"complete": False},
            "analyzer_only": {"truth": [{"robot_id": 1, "position": [0.0, 0.0]}]},
        }

        audit = audit_evidence_denominators(
            [*initialization, estimator, truth], [schedule], depths
        )

        self.assertEqual(audit.initialization_availability.numerator, 13)
        self.assertEqual(
            audit.conditional_containment[("dynamic_primary", 1)].numerator, 0
        )
        self.assertTrue(
            any(error.startswith("malformed_record:") for error in audit.integrity_errors)
        )

    def test_malformed_controller_cannot_supply_analyzer_truth(self):
        schedule = self.schedule(frames=2)
        depths = {
            condition: {robot_id: 1 for robot_id in schedule.robots}
            for condition in schedule.estimator_conditions
        }
        estimator = {
            "record_type": "estimator_tuple",
            "schema_version": "cbf2026-qualified-evidence-v1",
            "campaign_id": schedule.campaign_id,
            "condition": schedule.controller_condition,
            "trajectory_seed": schedule.trajectory_seed,
            "range_noise_seed": schedule.range_noise_seed,
            "frame_index": 1,
            "robot_id": 1,
            "depth": 1,
            "output_status": "fresh",
            "estimate": [0.0, 0.0],
            "radius": 1.0,
        }
        malformed_truth = {
            "record_type": "controller_interval",
            "campaign_id": schedule.campaign_id,
            "condition": schedule.controller_condition,
            "trajectory_seed": schedule.trajectory_seed,
            "range_noise_seed": schedule.range_noise_seed,
            "frame_index": 1,
            "runtime": {"complete": False},
            "analyzer_only": {
                "truth": [{"robot_id": 1, "position": [0.0, 0.0]}]
            },
        }

        audit = audit_evidence_denominators(
            [estimator, malformed_truth], [schedule], depths
        )

        self.assertEqual(
            audit.conditional_containment[("dynamic_primary", 1)].numerator, 0
        )
        self.assertEqual(
            audit.conditional_containment[("dynamic_primary", 1)].denominator, 0
        )

    def test_non_object_records_fail_closed_without_escaping(self):
        schedule = self.schedule(frames=2)
        depths = {
            condition: {robot_id: 1 for robot_id in schedule.robots}
            for condition in schedule.estimator_conditions
        }

        try:
            audit = audit_evidence_denominators(
                [None, [], 7, "not-a-record"], [schedule], depths
            )
        except Exception:
            audit = None

        self.assertIsNotNone(audit)
        self.assertEqual(audit.controller_availability.numerator, 0)
        self.assertEqual(audit.mission_success.numerator, 0)
        self.assertTrue(
            any(
                error.startswith("malformed_record:")
                for error in audit.integrity_errors
            )
        )

    def test_missing_launch_initialization_estimator_and_terminal_rows_stay_in_universe(self):
        schedule = self.schedule()
        depths = {
            condition: {robot_id: 1 for robot_id in schedule.robots}
            for condition in schedule.estimator_conditions
        }
        initialization = [
            {
                "record_type": "initialization",
                "schema_version": "cbf2026-qualified-evidence-v1",
                "campaign_id": schedule.campaign_id,
                "condition": schedule.controller_condition,
                "trajectory_seed": schedule.trajectory_seed,
                "range_noise_seed": schedule.range_noise_seed,
                "frame_index": 0,
                "robot_id": robot_id,
                "runtime": {"local_index": (robot_id - 1) % 7 + 1},
                "analyzer_only": {"truth_position": [0.0, 0.0]},
            }
            for robot_id in schedule.robots
        ]
        estimator = {
            "record_type": "estimator_tuple",
            "schema_version": "cbf2026-qualified-evidence-v1",
            "campaign_id": schedule.campaign_id,
            "condition": "dynamic_primary",
            "trajectory_seed": schedule.trajectory_seed,
            "range_noise_seed": schedule.range_noise_seed,
            "frame_index": 1,
            "robot_id": 1,
            "depth": 1,
            "output_status": "fresh",
            "estimate": [0.0, 0.0],
            "radius": 1.0,
        }

        baseline = audit_evidence_denominators(
            [*initialization, estimator], [schedule], depths
        )
        missing_initialization = audit_evidence_denominators(
            [*initialization[1:], estimator], [schedule], depths
        )
        missing_estimator = audit_evidence_denominators(
            initialization, [schedule], depths
        )

        self.assertEqual(baseline.initialization_availability.denominator, 14)
        self.assertEqual(missing_initialization.initialization_availability.numerator, 13)
        self.assertEqual(
            baseline.joint_containment[("dynamic_primary", 1)].denominator,
            28,
        )
        self.assertEqual(
            missing_estimator.joint_containment[("dynamic_primary", 1)].denominator,
            28,
        )
        self.assertEqual(
            missing_estimator.joint_containment[("dynamic_primary", 1)].numerator,
            0,
        )
        self.assertEqual(baseline.mission_success.denominator, 1)
        self.assertEqual(baseline.mission_success.numerator, 0)

        duplicate_initialization = audit_evidence_denominators(
            [*initialization, initialization[0]], [schedule], depths
        )
        self.assertTrue(
            any(
                error.startswith("duplicate_initialization:")
                for error in duplicate_initialization.integrity_errors
            )
        )

        launch_failure = synthesize_missing_mission(schedule, "launch_failed")
        self.assertEqual(len(launch_failure.initialization), 14)
        self.assertEqual(len(launch_failure.estimator), 2 * 2 * 14)
        self.assertEqual(len(launch_failure.controller), 3)
        self.assertEqual(len(launch_failure.reset), 0)
        self.assertFalse(launch_failure.mission.success)


class ResetReconstructionTests(unittest.TestCase):
    @staticmethod
    def endpoint(robot_id, estimate, epsilon, version):
        return {
            "robot_id": robot_id,
            "estimate": estimate,
            "covariance": [[0.25, 0.0], [0.0, 0.25]],
            "covariance_rate_bound": 0.1,
            "epsilon": epsilon,
            "bar_nu": 0.3,
            "snapshot_version": version,
            "allocation_version": 1,
        }

    def valid_reset(self):
        pre = [
            self.endpoint(1, [0.0, 0.0], 1.0, 0),
            self.endpoint(2, [5.0, 0.0], 1.0, 0),
        ]
        post = [
            self.endpoint(1, [0.1, 0.0], 1.1, 1),
            self.endpoint(2, [5.1, 0.0], 1.1, 1),
        ]
        return {
            "record_type": "reset",
            "schema_version": "cbf2026-qualified-evidence-v1",
            "campaign_id": "development-v1",
            "condition": "dynamic_primary",
            "trajectory_seed": 101,
            "range_noise_seed": 201,
            "frame_index": 0,
            "runtime": {
                "stage": "committed",
                "trigger": ["active-reference-change"],
                "predecessor_version": 0,
                "proposed_version": 1,
                "changed_nodes": [1],
                "descendant_closure": [1, 2],
                "active_sets": [
                    {
                        "node_id": 1,
                        "pre_active_references": [-1, -2],
                        "post_active_references": [-1, -2],
                    },
                    {
                        "node_id": 2,
                        "pre_active_references": [-1, 1],
                        "post_active_references": [-1, 1],
                    },
                ],
                "nodes": [
                    {
                        "node_id": 1,
                        "topological_index": 1,
                        "snapshot_version": 1,
                        "delta_p": [0.1, 0.0],
                        "delta_epsilon": 0.1,
                        "pre_active_references": [-1, -2],
                        "post_active_references": [-1, -2],
                        "proposed_snapshot": post[0],
                    },
                    {
                        "node_id": 2,
                        "topological_index": 2,
                        "snapshot_version": 1,
                        "delta_p": [0.1, 0.0],
                        "delta_epsilon": 0.1,
                        "pre_active_references": [-1, 1],
                        "post_active_references": [-1, 1],
                        "proposed_snapshot": post[1],
                    },
                ],
                "pre_endpoint_states": pre,
                "post_endpoint_states": post,
                "hard_edges": [
                    {
                        "edge": {
                            "kind": "collision",
                            "low": 1,
                            "high": 2,
                            "base_id": -1,
                        },
                        "threshold": 1.0,
                        "base_position": [0.0, 0.0],
                        "b_minus": 2.0,
                        "b_plus": 1.8,
                        "class_k_coefficient": 1.0,
                        "class_k_power": 1,
                        "endpoint_rows": [
                            {
                                "owner": 1,
                                "coefficient": [-1.0, 0.0],
                                "constant": 0.6,
                                "allocation": 0.5,
                                "snapshot_version": 1,
                                "allocation_version": 1,
                            },
                            {
                                "owner": 2,
                                "coefficient": [1.0, 0.0],
                                "constant": 0.6,
                                "allocation": 0.5,
                                "snapshot_version": 1,
                                "allocation_version": 1,
                            },
                        ],
                    }
                ],
                "hard_problems": [
                    collision_hard_problem(1),
                    collision_hard_problem(2),
                ],
                "local_hard_qps": [
                    {
                        "node_id": 1,
                        "problem": collision_hard_problem(1),
                        "feasible": True,
                        "status": "optimal",
                        "minimum_residual": 0.35,
                        "hard_problem_id": "096db1cc7b14a73f",
                        "solution": [0.0, 0.0, 0.0],
                    },
                    {
                        "node_id": 2,
                        "problem": collision_hard_problem(2),
                        "feasible": True,
                        "status": "optimal",
                        "minimum_residual": 0.35,
                        "hard_problem_id": "f7f4e8093c330224",
                        "solution": [0.0, 0.0, 0.0],
                    },
                ],
                "guard_decision": "accepted",
                "outcome": "commit",
                "reason": "accepted",
            },
        }

    def test_reset_delta_barrier_and_descendant_closure_are_independent(self):
        valid = self.valid_reset()
        self.assertEqual(audit_reset_primitives(valid, (1, 2), 1), ())

        tampered = self.valid_reset()
        tampered["runtime"]["descendant_closure"] = [1]
        self.assertIn(
            "reset_descendant_closure",
            audit_reset_primitives(tampered, (1, 2), 1),
        )

        tampered_barrier = self.valid_reset()
        tampered_barrier["runtime"]["hard_edges"][0]["b_plus"] += 1.0
        self.assertTrue(
            any(
                error.startswith("reset_b_plus:")
                for error in audit_reset_primitives(
                    tampered_barrier, (1, 2), 1
                )
            )
        )

        tampered_delta = self.valid_reset()
        tampered_delta["runtime"]["nodes"][0]["delta_p"] = [0.2, 0.0]
        self.assertIn(
            "reset_delta_p:1",
            audit_reset_primitives(tampered_delta, (1, 2), 1),
        )

    def test_reset_rejects_consistently_relaxed_hard_qp_bounds(self):
        relaxed = self.valid_reset()
        for problem in (
            relaxed["runtime"]["hard_problems"][0],
            relaxed["runtime"]["local_hard_qps"][0]["problem"],
        ):
            problem["planar_component_max"] = 26.0
            problem["bounds"][0]["limit"] = 26.0
            problem["bounds"][1]["limit"] = 26.0
            problem["hard_problem_id"] = "f50413463f554932"
        relaxed["runtime"]["local_hard_qps"][0][
            "hard_problem_id"
        ] = "f50413463f554932"

        errors = audit_reset_primitives(relaxed, (1, 2), 1)

        self.assertTrue(
            any(
                error.startswith("reset_hard_problem_structure:1")
                for error in errors
            )
        )

    def test_reset_malformed_indices_and_hash_overflow_fail_closed(self):
        bad_index = self.valid_reset()
        bad_index["runtime"]["local_hard_qps"][0]["problem"]["bounds"][0][
            "control_index"
        ] = 99
        try:
            index_errors = audit_reset_primitives(bad_index, (1, 2), 1)
        except Exception:
            index_errors = ()
        self.assertTrue(index_errors)

        huge_version = self.valid_reset()
        for problem in (
            huge_version["runtime"]["hard_problems"][0],
            huge_version["runtime"]["local_hard_qps"][0]["problem"],
        ):
            problem["snapshot_version"] = 1 << 80
        try:
            overflow_errors = audit_reset_primitives(
                huge_version, (1, 2), 1
            )
        except Exception:
            overflow_errors = ()
        self.assertTrue(overflow_errors)

    def test_reset_node_order_and_proposed_snapshot_are_exact(self):
        duplicate_order = self.valid_reset()
        duplicate_order["runtime"]["nodes"][1]["topological_index"] = 1
        self.assertTrue(
            any(
                error.startswith("reset_topological_index:")
                for error in audit_reset_primitives(
                    duplicate_order, (1, 2), 1
                )
            )
        )

        reversed_dependencies = self.valid_reset()
        reversed_dependencies["runtime"]["descendant_closure"] = [2, 1]
        reversed_dependencies["runtime"]["nodes"][0][
            "topological_index"
        ] = 2
        reversed_dependencies["runtime"]["nodes"][1][
            "topological_index"
        ] = 1
        self.assertIn(
            "reset_topological_reference:2:1",
            audit_reset_primitives(reversed_dependencies, (1, 2), 1),
        )

        mismatched_snapshot = self.valid_reset()
        mismatched_snapshot["runtime"]["nodes"][0][
            "proposed_snapshot"
        ] = copy.deepcopy(
            mismatched_snapshot["runtime"]["nodes"][0]["proposed_snapshot"]
        )
        mismatched_snapshot["runtime"]["nodes"][0]["proposed_snapshot"][
            "estimate"
        ] = [0.2, 0.0]
        self.assertIn(
            "reset_proposed_snapshot:1",
            audit_reset_primitives(mismatched_snapshot, (1, 2), 1),
        )

        nonfinite_snapshot = self.valid_reset()
        nonfinite_snapshot["runtime"]["nodes"][0][
            "proposed_snapshot"
        ] = copy.deepcopy(
            nonfinite_snapshot["runtime"]["nodes"][0]["proposed_snapshot"]
        )
        nonfinite_snapshot["runtime"]["nodes"][0]["proposed_snapshot"][
            "covariance"
        ][0][0] = float("nan")
        self.assertIn(
            "reset_proposed_snapshot:1",
            audit_reset_primitives(nonfinite_snapshot, (1, 2), 1),
        )


if __name__ == "__main__":
    unittest.main()
