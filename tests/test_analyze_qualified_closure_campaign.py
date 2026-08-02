import copy
import json
import tempfile
import unittest
import subprocess
import sys
import gzip
import hashlib
import argparse
import sqlite3
import io
import os
from collections import Counter
from contextlib import chdir, contextmanager, redirect_stderr
from pathlib import Path
from unittest import mock

import scripts.diagnostics.register_qualified_closure_campaign as registrar
import scripts.diagnostics.analyze_qualified_closure_campaign as analyzer_module
import scripts.diagnostics.qualified_initial_state as initial_state_module

from scripts.diagnostics.analyze_qualified_closure_campaign import (
    COMPACT_CAP_BYTES,
    analyze_qualified_closure_campaign,
    publish_analysis_bundle,
    stream_validated_replay_rows,
    validate_measurement_pair_streams,
    validate_terminal_mission_manifest,
    validate_analysis_registration_contract,
    _cross_mode_key_mismatch_count,
    validate_analysis_output_root,
    semantic_analysis_sha256,
    _controller_frame_metrics,
    _verify_controller_interior_evidence,
    _derive_mission_success,
    _publish_claimed_analysis_bundle,
    _terminalize_claimed_analysis_failure,
    _runtime_analyzer_argv,
    _canonical_json_identity,
    _account_initialization_audit,
    _preflight_raw_campaign,
    _producer_input_violations,
    _report_from_streamed_counts,
    _validate_synthesized_missing_stream,
    _validate_producer_namespace_identities,
    _validate_replay_reference_provenance,
    _wrong_mode_from_truth,
    _load_development_initial_state_contract,
    _validate_completed_mission_initial_configs,
    _validate_swarm_initial_truth,
    _validate_measurement_config_link,
    _registered_schedule_envelope,
    analyze_campaign_from_arguments,
    main as analyzer_main,
)
from tests.test_replay_qualified_estimator import build_registered_qualified_row
from scripts.diagnostics.register_qualified_closure_campaign import (
    FROZEN_THRESHOLDS,
    build_qualified_closure_protocol,
    publish_protocol,
)
from tests.test_generate_qualified_measurements import write_truth, read_rows
from scripts.diagnostics.generate_qualified_measurements import generate_measurement_bundle
from scripts.diagnostics.run_qualified_closure_campaign import (
    ProductionOperations,
    write_schedule_no_replace,
)
from scripts.diagnostics.qualified_initial_state import (
    audit_frozen_initial_family,
    load_qualified_initial_family,
)


def frozen_initial_family_fixture():
    repository = Path(__file__).resolve().parents[1]
    path = repository / "config/diagnostics/qualified_initial_family_v1.json"
    family = load_qualified_initial_family(path)
    audited = audit_frozen_initial_family(family)
    initial_state = {
        "family_schema_version": family["schema_version"],
        "namespace": family["namespace"],
        "family_semantic_sha256": family["semantic_sha256"],
        "registered_trajectory_seeds": [
            audit.seed for audit in audited.registered.audits
        ],
        "audit_trajectory_seeds": [
            audit.seed for audit in audited.audit.audits
        ],
        "missions": [
            {
                "trajectory_seed": audit.seed,
                "positions_sha256": audit.positions_sha256,
            }
            for audit in audited.registered.audits
        ],
        "frozen_summary": family["frozen_summary"],
        "admission": family["admission"],
        "perturbation_policy": {
            "clamp": family["perturbation"]["clamp"],
            "resample": family["perturbation"]["resample"],
        },
    }
    identity = {
        "path": str(path.resolve()),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "bytes": path.stat().st_size,
    }
    return path, family, audited, initial_state, identity


def write_development_registration(root, raw, analysis):
    project = root / "protocol-project"
    project.mkdir()
    files = {}
    for name in (
        "source.py", "Swarm", "base.json", "primary.json", "ablation.json",
        "dependencies.txt", "schema.json",
    ):
        path = project / name
        path.write_text("{}\n")
        files[name] = path
    files["initial_family"] = (
        Path(__file__).resolve().parents[1]
        / "config/diagnostics/qualified_initial_family_v1.json"
    )
    files["dependencies.txt"].write_text("ENABLE_GUROBI:BOOL=ON\n")
    files["primary.json"].write_text(json.dumps({
        "position_covariance": {"reference-selection": "dynamic-lower-index"}
    }))
    files["ablation.json"].write_text(json.dumps({
        "position_covariance": {"reference-selection": "fixed-cbf-only"}
    }))
    protocol = build_qualified_closure_protocol(
        kind="development", version="v5", project_root=project,
        trajectory_seeds=list(range(2026080201, 2026080211)),
        range_noise_seeds=list(range(2026081201, 2026081211)), frames=1000,
        roots={"raw": raw, "analysis": analysis},
        bindings={
            "source": files["source.py"], "binary": files["Swarm"],
            "base_config": files["base.json"],
            "primary_config": files["primary.json"],
            "ablation_config": files["ablation.json"],
            "dependencies": files["dependencies.txt"],
            "schema": files["schema.json"],
            "initial_family": files["initial_family"],
        },
        thresholds=FROZEN_THRESHOLDS, dirty_relevant_paths=[],
    )
    protocol_path = root / "fixture-protocol.json"
    markdown_path = protocol_path.with_suffix(".md")
    repository = {
        "root": str(project.resolve()),
        "head": "a" * 40,
        "tree": "b" * 40,
        "dirty_tracked_paths": [],
        "dirty_relevant_paths": [],
        "allowed_untracked_paths": [],
    }
    dependency = {
        "argv": ["ldd", str(files["Swarm"])],
        "returncode": 0,
        "sha256": "c" * 64,
        "bytes": 1,
    }
    conda = {
        "argv": ["conda", "list", "--explicit"],
        "sha256": "d" * 64,
        "bytes": 1,
    }
    protocol.update({
        "repository": repository,
        "review_artifacts": {
            "implementation_report": registrar._file_identity(files["source.py"]),
            "implementation_review": registrar._file_identity(files["schema.json"]),
        },
        "build": {
            "cmake_cache": registrar._file_identity(files["dependencies.txt"]),
            "gurobi_enabled": True,
            "binary_dependencies": dependency,
            "conda_explicit": conda,
        },
        "tooling": {
            name: registrar._file_identity(files["source.py"])
            for name in (
                "run_qualified_closure_campaign.py",
                "generate_qualified_measurements.py",
                "analyze_qualified_closure_campaign.py",
                "register_qualified_closure_campaign.py",
                "replay_qualified_estimator.py",
                "analyze_qualified_estimator.py",
                "qualified_initial_state.py",
            )
        },
        "publication": {
            "json_path": str(protocol_path),
            "markdown_path": str(markdown_path),
        },
        "supervision": dict(registrar.FROZEN_SUPERVISION),
    })
    commands = registrar._registered_argv(protocol)
    protocol["runner_argv"] = commands["runner"]
    protocol["analyzer_argv"] = commands["analyzer"]
    protocol["semantic_sha256"] = registrar._semantic_sha256(protocol)
    with development_registration_patches(protocol):
        publish_protocol(protocol, protocol_path, markdown_path)
    authorization_path = root / "reviews" / "fixture-authorization.json"
    authorization_path.parent.mkdir()
    authorization_path.write_text(json.dumps({
        "schema_version": "cbf2026-qualified-authorization-v1",
        "authorized": True, "kind": "development", "version": "v5",
        "protocol_sha256": hashlib.sha256(protocol_path.read_bytes()).hexdigest(),
        "implementation_identity": repository["head"],
    }))
    return protocol, protocol_path, authorization_path


@contextmanager
def development_registration_patches(protocol):
    with (
        mock.patch.dict(
            registrar.FROZEN_EXECUTION_ROOTS["development"],
            protocol["roots"],
            clear=True,
        ),
        mock.patch.object(
            registrar, "_repository_identity",
            return_value=copy.deepcopy(protocol["repository"]),
        ),
        mock.patch.object(
            registrar, "_dependency_identity",
            return_value=copy.deepcopy(protocol["build"]["binary_dependencies"]),
        ),
        mock.patch.object(
            registrar, "_command_identity",
            return_value=copy.deepcopy(protocol["build"]["conda_explicit"]),
        ),
        mock.patch.object(
            registrar, "validate_authorization_binding",
            return_value={
                "implementation_identity": protocol["repository"]["head"]
            },
        ),
        mock.patch.object(
            registrar, "verify_development_predecessor_state",
            return_value=None,
        ),
    ):
        yield


def write_raw_campaign_manifest(
    raw, missions, protocol_path, authorization_path, *, status, completed
):
    schedule_path = raw / "schedule.json"
    identities = {}
    for mission in missions:
        path = raw / mission["mission_id"] / "manifest.json"
        identities[mission["mission_id"]] = {
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "bytes": path.stat().st_size,
        }
    (raw / "manifest.json").write_text(json.dumps({
        "schema_version": "cbf2026-qualified-campaign-manifest-v1",
        "terminal": True,
        "status": status,
        "reason": "completed" if status == "completed" else "launch_failure",
        "mission_count": len(missions),
        "completed_mission_count": completed,
        "protocol_sha256": hashlib.sha256(protocol_path.read_bytes()).hexdigest(),
        "authorization_sha256": hashlib.sha256(
            authorization_path.read_bytes()
        ).hexdigest(),
        "schedule_sha256": hashlib.sha256(schedule_path.read_bytes()).hexdigest(),
        "schedule_bytes": schedule_path.stat().st_size,
        "mission_manifest_identities": identities,
    }))


def passing_campaign():
    estimator = []
    for condition, depth in (
        ("dynamic_primary", 1),
        ("fixed_fim_ablation", 2),
    ):
        for mission in ("mission-01", "mission-02"):
            estimator.append({
                "mission_id": mission,
                "condition": condition,
                "depth": depth,
                "output_status": "fresh",
                "fresh_expected": True,
                "bounded_prediction": False,
                "contained": True,
                "error_m": 1.0,
                "source_order_valid": True,
                "admissible_mode_count": 1,
                "wrong_mode": False,
                "private_age": 0,
            })
    return {
        "manifests": [
            {"terminal": True, "status": "completed"},
            {"terminal": True, "status": "completed"},
        ],
        "expected_key_count": 20,
        "observed_key_count": 20,
        "expected_primary_tuple_count": 2,
        "runtime_truth_read_count": 0,
        "initialization_rows": [
            {"accounted": True}, {"accounted": True},
            {"accounted": True}, {"accounted": True},
        ],
        "registered_depths": {
            "dynamic_primary": [1],
            "fixed_fim_ablation": [2],
        },
        "estimator_rows": estimator,
        "hard_rows": {"missing": 0, "duplicate": 0},
        "controller_rows": [
            {
                "certificate_available": True,
                "nu_inst": 0.1,
                "bar_nu": 0.2,
                "qp_status": "optimal",
                "local_residual_min": 0.0,
                "reconstructed_residual_min": 0.0,
                "input_limit_excess": 0.0,
                "true_localization_violation": False,
                "true_collision_violation": False,
                "allocation_stress": "passed",
                "deadlock": False,
            },
            {
                "certificate_available": True,
                "nu_inst": 0.2,
                "bar_nu": 0.2,
                "qp_status": "optimal",
                "local_residual_min": 0.1,
                "reconstructed_residual_min": 0.1,
                "input_limit_excess": 0.0,
                "true_localization_violation": False,
                "true_collision_violation": False,
                "allocation_stress": "passed",
                "deadlock": False,
            },
        ],
        "reset_rows": [{"accepted_violation": False, "reason": "none"}],
        "missions": [
            {"mission_id": "mission-01", "terminal": True, "success": True},
            {"mission_id": "mission-02", "terminal": True, "success": True},
        ],
        "measurement_sha256_by_condition": {
            "dynamic_primary": "a" * 64,
            "fixed_fim_ablation": "a" * 64,
        },
        "measurement_regenerated_by_analyzer": False,
    }


class CampaignAnalyzerGateTests(unittest.TestCase):
    def failed(self, campaign):
        report = analyze_qualified_closure_campaign(campaign)
        return {
            name
            for name, gate in report["gates"].items()
            if isinstance(gate, dict) and gate.get("passed") is False
        }

    def test_two_mission_synthetic_campaign_passes_every_gate_and_emits_metrics(self):
        report = analyze_qualified_closure_campaign(passing_campaign())

        self.assertTrue(report["passed"])
        self.assertTrue(report["gates"])
        for gate in report["gates"].values():
            if "passed" in gate:
                self.assertTrue({
                    "numerator", "denominator", "comparison", "threshold",
                    "passed", "pass_fail",
                } <= set(gate))
        self.assertEqual(report["errors_m"]["max"], 1.0)
        self.assertIn("p95", report["errors_m"])
        self.assertIn("p99", report["errors_m"])
        self.assertEqual(report["private_age_strata"]["0"], 4)
        self.assertEqual(report["incomplete_missions"], [])

    def test_controller_metrics_come_from_independent_primitive_reconstruction(self):
        from scripts.diagnostics.qualified_closure_evidence import (
            CanonicalEdgeId,
            ControllerReconstruction,
        )
        reconstruction = ControllerReconstruction(
            nodes={1: {"nu_inst": 0.1, "bar_nu": 0.2}},
            local_residuals={(CanonicalEdgeId("localization", 1, 1, 0), 1): 0.25},
            full_residuals={CanonicalEdgeId("localization", 1, 1, 0): 0.5},
            integrity_errors=(),
        )
        controller = {"runtime": {
            "local_residual_minimum": -999.0,
            "reconstructed_residual_minimum": -999.0,
            "component_maxima": {"vx": 999.0, "vy": 999.0, "yaw_rate": 999.0},
            "nodes": [{"robot_id": 1, "applied_command": [1.0, -2.0, 0.1]}],
        }}
        with mock.patch(
            "scripts.diagnostics.qualified_closure_evidence.reconstruct_controller_primitives",
            return_value=reconstruction,
        ):
            metrics = _controller_frame_metrics(controller, [{}] * 232)

        self.assertEqual(metrics["local_residual_minimum"], 0.25)
        self.assertEqual(metrics["reconstructed_residual_minimum"], 0.5)
        self.assertEqual(
            metrics["component_maxima"], {"vx": 1.0, "vy": 2.0, "yaw_rate": 0.1}
        )
        self.assertEqual(metrics["nodes"][1]["nu_inst"], 0.1)

    def test_analyzer_rejects_forged_or_missing_interior_policy(self):
        from tests.test_qualified_closure_evidence import (
            bounded_hard_problem,
            interior_policy,
            localization_problem_row,
        )

        problem = bounded_hard_problem(
            1, localization_problem_row(2.0), "analyzer-fixture"
        )
        command = [0.0, 0.0, 0.0]
        # The hash is irrelevant to this direct analyzer reconstruction.
        node = {
            "normal_problem": problem,
            "applied_command": command,
            "hard_interior_selection": interior_policy(problem, command),
        }
        _verify_controller_interior_evidence([node])
        for field, value in (
            ("planar_chebyshev_radius_mps", 0.0),
            ("enforced_floor_mps", 0.0),
            ("minimum_original_hard_residual_mps", 0.0),
        ):
            forged = copy.deepcopy(node)
            forged["hard_interior_selection"][field] = value
            with self.assertRaisesRegex(ValueError, "interior"):
                _verify_controller_interior_evidence([forged])
        integral_tokens = copy.deepcopy(node)
        integral_tokens["hard_interior_selection"][
            "planar_chebyshev_radius_mps"
        ] = 27
        integral_tokens["hard_interior_selection"][
            "minimum_original_hard_residual_mps"
        ] = 2
        with self.assertRaisesRegex(ValueError, "interior"):
            _verify_controller_interior_evidence([integral_tokens])
        with self.assertRaisesRegex(ValueError, "provenance"):
            _verify_controller_interior_evidence([node, {"normal_problem": problem}])

    def test_analyzer_reconstructs_registered_v3_policy_and_rejects_cross_version(self):
        from tests.test_qualified_closure_evidence import (
            bounded_hard_problem,
            interior_policy,
            localization_problem_row,
        )

        problem = bounded_hard_problem(
            1, localization_problem_row(2.0), "analyzer-v3-fixture"
        )
        command = [0.0, 0.0, 0.0]
        node = {
            "normal_problem": problem,
            "applied_command": command,
            "hard_interior_selection": interior_policy(
                problem,
                command,
                schema_version="hard-interior-v3",
                mode="planar-chebyshev-fraction-cap-v2",
                fraction=0.131,
            ),
        }
        _verify_controller_interior_evidence([node])
        for field, value in (
            ("schema_version", "hard-interior-v2"),
            ("mode", "planar-chebyshev-fraction-cap-v1"),
            ("fraction", 0.13),
            ("fraction", 0.132),
            ("fraction", True),
        ):
            with self.subTest(field=field, value=value):
                mutated = copy.deepcopy(node)
                mutated["hard_interior_selection"][field] = value
                with self.assertRaisesRegex(ValueError, "interior"):
                    _verify_controller_interior_evidence([mutated])

    def test_analyzer_rejects_mixed_controller_policy_identities(self):
        from tests.test_qualified_closure_evidence import (
            bounded_hard_problem,
            interior_policy,
            localization_problem_row,
        )

        problem = bounded_hard_problem(
            1, localization_problem_row(2.0), "analyzer-mixed-policy"
        )
        command = [0.0, 0.0, 0.0]

        def node(policy=None):
            result = {
                "normal_problem": copy.deepcopy(problem),
                "applied_command": list(command),
            }
            if policy is not None:
                result["hard_interior_selection"] = copy.deepcopy(policy)
            return result

        historical_v2 = interior_policy(problem, command)
        marked_v2 = interior_policy(
            problem, command, schema_version="hard-interior-v2"
        )
        registered_v3 = interior_policy(
            problem,
            command,
            schema_version="hard-interior-v3",
            mode="planar-chebyshev-fraction-cap-v2",
            fraction=0.131,
        )

        for policies in (
            [None, None],
            [historical_v2, historical_v2],
            [marked_v2, marked_v2],
            [registered_v3, registered_v3],
        ):
            with self.subTest(
                identity=None if policies[0] is None
                else policies[0].get("schema_version", "historical-v2")
            ):
                _verify_controller_interior_evidence(
                    [node(policy) for policy in policies]
                )

        with self.assertRaisesRegex(ValueError, "provenance"):
            _verify_controller_interior_evidence(
                [node(historical_v2), node(registered_v3)]
            )

    def test_wrong_mode_compares_publication_to_all_physical_modes(self):
        row = {
            "published_mode_id": "far-mode",
            "audit_bundle": {
                "representatives": [
                    {"attempt_id": "near", "estimate": [0.0, 0.0]},
                    {"attempt_id": "far", "estimate": [10.0, 0.0]},
                ],
                "clustering": {"modes": [
                    {"mode_id": "near-mode", "member_ids": ["near"]},
                    {"mode_id": "far-mode", "member_ids": ["far"]},
                ]},
                "qualifications": [
                    {"admissible": False}, {"admissible": True},
                ],
            },
        }

        self.assertTrue(_wrong_mode_from_truth(row, (0.1, 0.0)))

    def test_launch_failure_fails_completeness_and_mission_fraction(self):
        campaign = passing_campaign()
        campaign["manifests"][0]["status"] = "failed"
        campaign["missions"][0]["success"] = False
        self.assertEqual(
            self.failed(campaign),
            {"complete_keys_and_manifests", "successful_mission_fraction"},
        )

    def test_wrong_unique_mode_fails_only_wrong_mode_gate(self):
        campaign = passing_campaign()
        campaign["estimator_rows"][0]["wrong_mode"] = True
        self.assertEqual(
            self.failed(campaign), {"zero_wrong_mode_fresh_publication"}
        )

    def test_missing_registered_depth_fails_depth_gate(self):
        campaign = passing_campaign()
        campaign["registered_depths"]["dynamic_primary"].append(3)
        self.assertEqual(
            self.failed(campaign), {"every_registered_depth_containment"}
        )

    def test_catastrophic_error_fails_fifty_meter_gate(self):
        campaign = passing_campaign()
        campaign["estimator_rows"][0]["error_m"] = 50.0001
        self.assertEqual(
            self.failed(campaign), {"zero_finite_error_above_50m"}
        )

    def test_ablation_errors_are_reported_without_entering_primary_denominator(self):
        campaign = passing_campaign()
        ablation = next(
            row for row in campaign["estimator_rows"]
            if row["condition"] == "fixed_fim_ablation"
        )
        ablation["error_m"] = 500.0
        ablation["contained"] = False

        report = analyze_qualified_closure_campaign(campaign)

        self.assertTrue(report["passed"])
        self.assertEqual(report["ablation"]["finite_error_above_50m"], 1)
        self.assertEqual(
            report["gates"]["zero_finite_error_above_50m"]["denominator"], 2
        )

    def test_fresh_retention_uses_entire_frozen_primary_tuple_universe(self):
        campaign = passing_campaign()
        primary = [
            row for row in campaign["estimator_rows"]
            if row["condition"] == "dynamic_primary"
        ]
        primary[1].update({
            "output_status": "predicted",
            "fresh_expected": False,
            "bounded_prediction": True,
        })

        gate = analyze_qualified_closure_campaign(campaign)["gates"][
            "fresh_retention"
        ]

        self.assertEqual(gate["numerator"], 1)
        self.assertEqual(gate["denominator"], 2)
        self.assertFalse(gate["passed"])

    def test_missing_primary_row_cannot_shrink_synthetic_frozen_denominator(self):
        campaign = passing_campaign()
        campaign["estimator_rows"].remove(next(
            row for row in campaign["estimator_rows"]
            if row["condition"] == "dynamic_primary"
            and row["mission_id"] == "mission-02"
        ))

        gate = analyze_qualified_closure_campaign(campaign)["gates"][
            "fresh_retention"
        ]

        self.assertEqual(gate["numerator"], 1)
        self.assertEqual(gate["denominator"], 2)
        self.assertFalse(gate["passed"])

    def test_production_sql_fresh_retention_uses_frozen_primary_universe(self):
        database = sqlite3.connect(":memory:")
        self.addCleanup(database.close)
        database.execute(
            "CREATE TABLE estimator (condition TEXT, mission TEXT, "
            "frame INTEGER, robot INTEGER, depth INTEGER, status TEXT, "
            "error REAL, contained INTEGER, bounded INTEGER, "
            "admissible INTEGER, wrong INTEGER, "
            "private_age INTEGER, source_order_valid INTEGER, "
            "PRIMARY KEY(condition,mission,frame,robot))"
        )
        database.execute(
            "CREATE TABLE initialization (mission TEXT, robot INTEGER, "
            "PRIMARY KEY(mission,robot))"
        )
        database.execute(
            "CREATE TABLE initialization_audit (condition TEXT, "
            "mission TEXT, robot INTEGER, public_status TEXT, "
            "private_present INTEGER, "
            "PRIMARY KEY(condition,mission,robot))"
        )
        database.executemany(
            "INSERT INTO initialization VALUES (?,?)",
            [("mission-01", robot) for robot in range(1, 15)],
        )
        database.executemany(
            "INSERT INTO initialization_audit VALUES (?,?,?,?,?)",
            [
                (condition, "mission-01", robot, "fresh", 1)
                for condition in (
                    "dynamic_primary", "fixed_fim_ablation"
                )
                for robot in range(1, 15)
            ],
        )
        rows = []
        for condition in ("dynamic_primary", "fixed_fim_ablation"):
            for robot, status in (
                (1, "fresh"),
                (2, "predicted"),
            ):
                rows.append((
                    condition, "mission-01", 1, robot, 1, status,
                    1.0, 1, int(status == "predicted"),
                    1, 0, 0, 1,
                ))
        database.executemany(
            "INSERT INTO estimator VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)",
            rows,
        )
        counters = Counter({
            "initialization": 14,
            "initialization_audit": 28,
            "controller": 1,
            "endpoint": 1,
            "estimator": 4,
            "controller_available": 1,
            "missions_success": 1,
        })
        report = _report_from_streamed_counts(
            database,
            {"universes": {
                "initialization": 14,
                "controller": 1,
                "endpoint": 1,
                "estimator_total": 4,
                "estimator_per_condition": 2,
                "reconstructed": 1,
            }},
            1,
            counters,
            1,
            [],
            ["a" * 64],
            Counter(),
        )

        gate = report["gates"]["fresh_retention"]
        self.assertEqual(gate["numerator"], 1)
        self.assertEqual(gate["denominator"], 2)
        self.assertFalse(gate["passed"])

    def test_missing_edge_fails_hard_row_gate(self):
        campaign = passing_campaign()
        campaign["hard_rows"]["missing"] = 1
        self.assertEqual(
            self.failed(campaign), {"zero_missing_duplicate_hard_rows"}
        )

    def test_nu_excess_fails_rate_bound_gate(self):
        campaign = passing_campaign()
        campaign["controller_rows"][0]["nu_inst"] = 0.200000002
        self.assertEqual(self.failed(campaign), {"zero_nu_bound_excess"})

    def test_negative_residual_fails_residual_gate(self):
        campaign = passing_campaign()
        campaign["controller_rows"][0]["local_residual_min"] = -1.0001e-7
        self.assertEqual(
            self.failed(campaign), {"zero_negative_hard_residual"}
        )

    def test_infeasible_qp_fails_primary_qp_gate(self):
        campaign = passing_campaign()
        campaign["controller_rows"][0]["qp_status"] = "infeasible"
        self.assertEqual(
            self.failed(campaign), {"zero_primary_local_hard_qp_infeasibility"}
        )

    def test_input_violation_fails_input_gate(self):
        campaign = passing_campaign()
        campaign["controller_rows"][0]["input_limit_excess"] = 1.0001e-7
        self.assertEqual(
            self.failed(campaign), {"zero_input_limit_violations"}
        )

    def test_reset_violation_fails_reset_gate(self):
        campaign = passing_campaign()
        campaign["reset_rows"][0]["accepted_violation"] = True
        self.assertEqual(
            self.failed(campaign), {"zero_accepted_reset_violations"}
        )

    def test_true_distance_violation_fails_distance_gate(self):
        campaign = passing_campaign()
        campaign["controller_rows"][0]["true_collision_violation"] = True
        self.assertEqual(
            self.failed(campaign), {"zero_true_distance_violations"}
        )

    def test_incomplete_mission_fails_success_fraction_and_is_reported(self):
        campaign = passing_campaign()
        campaign["missions"][0]["success"] = False
        report = analyze_qualified_closure_campaign(campaign)
        self.assertEqual(
            self.failed(campaign), {"successful_mission_fraction"}
        )
        self.assertEqual(report["incomplete_missions"], ["mission-01"])

    def test_measurement_substitution_or_regeneration_is_rejected(self):
        substituted = passing_campaign()
        substituted["measurement_sha256_by_condition"][
            "fixed_fim_ablation"
        ] = "b" * 64
        self.assertEqual(
            self.failed(substituted), {"identical_immutable_measurements"}
        )
        regenerated = passing_campaign()
        regenerated["measurement_regenerated_by_analyzer"] = True
        self.assertEqual(
            self.failed(regenerated), {"identical_immutable_measurements"}
        )


class CompactAnalysisWriterTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(
            prefix="qualified-analysis-"
        )
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)

    def test_just_below_twenty_five_mb_publishes_all_files_no_replace(self):
        root = self.base / "below"
        manifest = publish_analysis_bundle(
            root,
            {"schema_version": "analysis-v1", "passed": True},
            "# PASS\n",
            allocated_size_fn=lambda _paths: COMPACT_CAP_BYTES - 1,
        )

        self.assertEqual(manifest["status"], "completed")
        self.assertEqual(
            sorted(path.name for path in root.iterdir()),
            ["analysis.json", "analysis.md", "manifest.json"],
        )
        with self.assertRaises(FileExistsError):
            publish_analysis_bundle(
                root, {}, "", allocated_size_fn=lambda _paths: 0
            )

    def test_above_twenty_five_mb_publishes_failure_manifest_only(self):
        root = self.base / "above"
        manifest = publish_analysis_bundle(
            root,
            {"schema_version": "analysis-v1", "passed": True},
            "# too large\n",
            allocated_size_fn=lambda _paths: COMPACT_CAP_BYTES + 1,
        )

        self.assertEqual(manifest["status"], "failed")
        self.assertEqual(manifest["reason"], "compact_bundle_cap")
        self.assertEqual(
            [path.name for path in root.iterdir()], ["manifest.json"]
        )

    def test_injected_failure_before_directory_commit_exposes_no_analysis_root(self):
        root = self.base / "transaction-failure"
        with self.assertRaisesRegex(RuntimeError, "injected"):
            publish_analysis_bundle(
                root, {"schema_version": "analysis-v1"}, "# report\n",
                publish_hook=lambda _stage: (_ for _ in ()).throw(RuntimeError("injected")),
            )
        self.assertFalse(root.exists())

    def test_claimed_success_exposes_no_member_before_directory_exchange(self):
        root = self.base / "claimed-success"
        root.mkdir()
        observed = []

        def fail_exchange(stage, target):
            observed.append((
                sorted(path.name for path in Path(stage).iterdir()),
                sorted(path.name for path in Path(target).iterdir()),
            ))
            raise RuntimeError("injected exchange failure")

        with mock.patch(
            "scripts.diagnostics.analyze_qualified_closure_campaign."
            "_exchange_claimed_analysis_root",
            side_effect=fail_exchange,
        ):
            with self.assertRaisesRegex(RuntimeError, "exchange failure"):
                _publish_claimed_analysis_bundle(
                    root, {"passed": True}, "# PASS\n"
                )
        self.assertEqual(observed, [(
            ["analysis.json", "analysis.md", "manifest.json"], []
        )])
        self.assertEqual(list(root.iterdir()), [])

    def test_claimed_failure_preserves_old_bundle_until_directory_exchange(self):
        root = self.base / "claimed-failure"
        root.mkdir()
        marker = root / "owned-partial"
        marker.write_text("preserve until commit")

        with mock.patch(
            "scripts.diagnostics.analyze_qualified_closure_campaign."
            "_exchange_claimed_analysis_root",
            side_effect=RuntimeError("injected exchange failure"),
        ):
            with self.assertRaisesRegex(RuntimeError, "exchange failure"):
                _terminalize_claimed_analysis_failure(
                    root, ValueError("analysis failed")
                )
        self.assertEqual(marker.read_text(), "preserve until commit")
        self.assertEqual([path.name for path in root.iterdir()], ["owned-partial"])


class RawReplayStreamingTests(unittest.TestCase):
    def write(self, path, row):
        with path.open("xb") as raw:
            with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as sink:
                sink.write(json.dumps(row, separators=(",", ":")).encode() + b"\n")

    def test_independent_recomputation_precedes_projection_and_detects_tamper(self):
        with tempfile.TemporaryDirectory(prefix="qualified-raw-recompute-") as directory:
            root = Path(directory)
            valid = root / "valid.jsonl.gz"
            row = build_registered_qualified_row()
            self.write(valid, row)
            projected = []
            count = stream_validated_replay_rows(
                valid, hashlib.sha256(valid.read_bytes()).hexdigest(),
                on_valid_row=projected.append,
            )
            self.assertEqual(count, 1)
            self.assertEqual(projected[0]["frame_index"], row["frame_index"])

            tampered = root / "tampered.jsonl.gz"
            row["published_mode_id"] = "forged-mode"
            self.write(tampered, row)
            with self.assertRaises(ValueError):
                stream_validated_replay_rows(
                    tampered, hashlib.sha256(tampered.read_bytes()).hexdigest(),
                    on_valid_row=lambda _row: None,
                )

    def test_frame_zero_audit_reconstructs_full_base_case_without_post_metrics(self):
        from tests.test_replay_qualified_estimator import (
            fresh_registered_qualified_row,
        )

        with tempfile.TemporaryDirectory(
            prefix="qualified-initialization-audit-"
        ) as directory:
            root = Path(directory)
            valid = root / "frame-zero.jsonl.gz"
            row = fresh_registered_qualified_row()
            self.write(valid, row)
            reconstructed = []

            self.assertEqual(
                stream_validated_replay_rows(
                    valid,
                    hashlib.sha256(valid.read_bytes()).hexdigest(),
                    on_valid_row=reconstructed.append,
                ),
                1,
            )
            self.assertEqual(
                reconstructed[0]["qualification_kind"], "deployment"
            )
            self.assertIn(
                "qualifier_payload",
                reconstructed[0]["audit_bundle"]["qualifier_context"],
            )
            self.assertTrue(
                reconstructed[0]["audit_bundle"]["clustering"]["modes"]
            )
            self.assertTrue(reconstructed[0]["qualifications"])
            self.assertIn(
                "public_output",
                reconstructed[0]["audit_bundle"]["lifecycle"],
            )
            self.assertIn(
                "next_private_state",
                reconstructed[0]["audit_bundle"]["lifecycle"],
            )

            database = sqlite3.connect(":memory:")
            self.addCleanup(database.close)
            database.execute(
                "CREATE TABLE initialization_audit (condition TEXT, "
                "mission TEXT, robot INTEGER, public_status TEXT, "
                "private_present INTEGER, "
                "PRIMARY KEY(condition,mission,robot))"
            )
            database.execute(
                "CREATE TABLE estimator (condition TEXT, mission TEXT, "
                "frame INTEGER, robot INTEGER)"
            )
            _account_initialization_audit(
                database,
                "mission-01",
                "dynamic_primary",
                reconstructed[0],
            )
            self.assertEqual(
                database.execute(
                    "SELECT condition,mission,robot FROM initialization_audit"
                ).fetchall(),
                [("dynamic_primary", "mission-01", row["robot_id"])],
            )
            self.assertEqual(
                database.execute("SELECT COUNT(*) FROM estimator").fetchone()[0],
                0,
            )

            mutations = {
                "deployment-domain": lambda item: item["audit_bundle"][
                    "qualifier_context"
                ]["qualifier_payload"]["domain"].__setitem__(
                    "margin_m", False
                ),
                "mode-enumeration": lambda item: item["audit_bundle"][
                    "clustering"
                ]["modes"][0].__setitem__("mode_id", "forged-mode"),
                "qualification": lambda item: item["qualifications"][0].__setitem__(
                    "admissible", not item["qualifications"][0]["admissible"]
                ),
                "publication": lambda item: item.__setitem__(
                    "published_mode_id", "forged-mode"
                ),
                "private-state": lambda item: item["audit_bundle"][
                    "lifecycle"
                ]["next_private_state"].__setitem__("history_version", False),
            }
            for label, mutate in mutations.items():
                with self.subTest(label=label):
                    tampered = copy.deepcopy(row)
                    mutate(tampered)
                    path = root / f"{label}.jsonl.gz"
                    self.write(path, tampered)
                    with self.assertRaises(ValueError):
                        stream_validated_replay_rows(
                            path,
                            hashlib.sha256(path.read_bytes()).hexdigest(),
                            on_valid_row=lambda _row: None,
                        )

    def test_cross_mode_source_order_requires_identical_mission_frame_robot_keys(self):
        database = sqlite3.connect(":memory:")
        self.addCleanup(database.close)
        database.execute(
            "CREATE TABLE estimator (condition TEXT, mission TEXT, frame INTEGER, robot INTEGER)"
        )
        database.executemany(
            "INSERT INTO estimator VALUES (?,?,?,?)",
            [
                ("dynamic_primary", "mission-01", 1, 1),
                ("dynamic_primary", "mission-01", 1, 2),
                ("fixed_fim_ablation", "mission-01", 1, 1),
                ("fixed_fim_ablation", "mission-01", 1, 3),
            ],
        )

        self.assertEqual(_cross_mode_key_mismatch_count(database), 2)

    def test_same_keys_with_reordered_sources_fail_condition_local_verdict(self):
        database = sqlite3.connect(":memory:")
        self.addCleanup(database.close)
        database.execute(
            "CREATE TABLE measurement (mission TEXT, condition TEXT, frame INTEGER, "
            "owner INTEGER, ordinal INTEGER, kind TEXT, reference_id INTEGER, "
            "noisy_range REAL, ranging_sigma REAL, position_json TEXT, "
            "covariance_json TEXT, provenance_json TEXT)"
        )
        for ordinal, reference_id in enumerate((0, 1)):
            database.execute(
                "INSERT INTO measurement VALUES (?,?,?,?,?,?,?,?,?,?,?,?)",
                (
                    "mission-01", "dynamic_primary", 1, 3, ordinal,
                    "base", reference_id, 10.0 + ordinal, 0.5,
                    json.dumps([float(reference_id), 0.0]),
                    json.dumps([[0.0, 0.0], [0.0, 0.0]]),
                    json.dumps([reference_id]),
                ),
            )
        references = [
            {
                "key": ["base", reference_id],
                "position": [float(reference_id), 0.0],
                "range": 10.0 + reference_id,
                "covariance": [[0.0, 0.0], [0.0, 0.0]],
                "ranging_sigma": 0.5,
                "base_anchor_provenance": [reference_id],
            }
            for reference_id in (0, 1)
        ]
        row = {
            "frame_index": 1,
            "robot_id": 3,
            "runtime_inputs": {"references": references},
        }
        self.assertTrue(_validate_replay_reference_provenance(
            database, "mission-01", "dynamic_primary", row, {}
        ))
        row["runtime_inputs"]["references"] = list(reversed(references))
        with self.assertRaisesRegex(ValueError, "condition-local inputs"):
            _validate_replay_reference_provenance(
                database, "mission-01", "dynamic_primary", row, {}
            )


class RawMeasurementJoinTests(unittest.TestCase):
    def bundle(self, root):
        truth = root / "truth.jsonl.gz"
        write_truth(truth)
        output = root / "measurements"
        manifest = generate_measurement_bundle(
            truth, output, range_noise_seed=2026081101,
            ranging_sigma=0.5, config_sha256="a" * 64,
            required_edges_by_condition={
                "dynamic_primary": [(0, 1, ("base", 0)), (0, 2, ("uav", 1))],
                "fixed_fim_ablation": [(0, 1, ("base", 0))],
            },
            truth_manifest={"terminal": True, "status": "completed",
                            "sha256": hashlib.sha256(truth.read_bytes()).hexdigest()},
        )
        return output, manifest

    def test_exact_runtime_audit_schema_and_semantic_join_are_derived(self):
        with tempfile.TemporaryDirectory(prefix="qualified-measurement-join-") as directory:
            output, manifest = self.bundle(Path(directory))
            observed = []
            summary = validate_measurement_pair_streams(
                output / "runtime.jsonl.gz", output / "audit.jsonl.gz",
                runtime_sha256=manifest["runtime_sha256"],
                audit_sha256=manifest["audit_sha256"],
                on_pair=lambda runtime, audit: observed.append((runtime, audit)),
            )
            self.assertEqual(summary["row_count"], 2)
            self.assertEqual(len(observed), 2)
            self.assertTrue(all("truth" not in json.dumps(runtime).lower()
                                for runtime, _audit in observed))

    def test_rehashed_audit_semantic_tamper_is_rejected(self):
        with tempfile.TemporaryDirectory(prefix="qualified-measurement-tamper-") as directory:
            output, manifest = self.bundle(Path(directory))
            audit_path = output / "audit.jsonl.gz"
            rows = read_rows(audit_path)
            rows[0]["noiseless_range"] += 1.0
            audit_path.unlink()
            with audit_path.open("xb") as raw:
                with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as sink:
                    for row in rows:
                        sink.write(json.dumps(row, sort_keys=True).encode() + b"\n")
            with self.assertRaises(ValueError):
                validate_measurement_pair_streams(
                    output / "runtime.jsonl.gz", audit_path,
                    runtime_sha256=manifest["runtime_sha256"],
                    audit_sha256=hashlib.sha256(audit_path.read_bytes()).hexdigest(),
                )

    def test_balanced_noiseless_and_noise_tamper_is_independently_rejected(self):
        with tempfile.TemporaryDirectory(prefix="qualified-balanced-tamper-") as directory:
            output, manifest = self.bundle(Path(directory))
            audit_path = output / "audit.jsonl.gz"
            rows = read_rows(audit_path)
            rows[0]["noiseless_range"] += 1.0
            rows[0]["sampled_noise"] -= 1.0
            audit_path.unlink()
            with audit_path.open("xb") as raw:
                with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as sink:
                    for row in rows:
                        sink.write(json.dumps(row, sort_keys=True).encode() + b"\n")

            with self.assertRaisesRegex(ValueError, "noiseless range"):
                validate_measurement_pair_streams(
                    output / "runtime.jsonl.gz", audit_path,
                    runtime_sha256=manifest["runtime_sha256"],
                    audit_sha256=hashlib.sha256(audit_path.read_bytes()).hexdigest(),
                )

    def test_runtime_stream_identity_is_bound_to_frozen_bundle_manifest(self):
        with tempfile.TemporaryDirectory(prefix="qualified-measurement-identity-") as directory:
            output, manifest = self.bundle(Path(directory))
            runtime_path = output / "runtime.jsonl.gz"
            rows = read_rows(runtime_path)
            rows[0]["config_sha256"] = "f" * 64
            runtime_path.unlink()
            with runtime_path.open("xb") as raw:
                with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as sink:
                    for row in rows:
                        sink.write(json.dumps(row, sort_keys=True).encode() + b"\n")

            with self.assertRaisesRegex(ValueError, "stream identity"):
                validate_measurement_pair_streams(
                    runtime_path,
                    output / "audit.jsonl.gz",
                    runtime_sha256=hashlib.sha256(runtime_path.read_bytes()).hexdigest(),
                    audit_sha256=manifest["audit_sha256"],
                    config_sha256=manifest["config_sha256"],
                    measurement_stream_id=manifest["measurement_stream_id"],
                    expected_row_count=manifest["row_count"],
                    range_noise_seed=2026081101,
                )


class RawManifestSchemaTests(unittest.TestCase):
    def test_derived_gate_labels_cannot_be_injected_into_raw_manifest(self):
        base = {
            "schema_version": "cbf2026-qualified-mission-manifest-v1",
            "terminal": True, "status": "failed", "reason": "launch_failure",
            "mission": {"mission_id": "mission-01"},
            "member_identities": {},
        }
        self.assertTrue(validate_terminal_mission_manifest(base))
        forged = {**base, "gates": {"successful_mission_fraction": {"passed": True}}}
        self.assertFalse(validate_terminal_mission_manifest(forged))

    def test_failed_mission_stream_matches_exact_lazy_synthetic_universe(self):
        with tempfile.TemporaryDirectory(prefix="qualified-missing-stream-") as directory:
            root = Path(directory)
            mission = {
                "campaign_id": "development-v4",
                "mission_id": "mission-01",
                "trajectory_seed": 2026080101,
                "range_noise_seed": 2026081101,
                "frames": 1,
                "conditions": ["dynamic_primary", "fixed_fim_ablation"],
            }
            ProductionOperations().synthesize_failed_mission(
                mission, root, "launch_failure"
            )
            _validate_synthesized_missing_stream(
                root / "synthetic-missing.jsonl.gz", mission, "launch_failure"
            )

    def test_incomplete_controller_abort_remains_in_synthetic_missing_universe(self):
        from tests.test_run_qualified_closure_campaign import (
            rejected_verified_reset_fixture,
        )

        with tempfile.TemporaryDirectory(
            prefix="qualified-incomplete-controller-missing-"
        ) as directory:
            root = Path(directory)
            mission = {
                "campaign_id": "development-v5",
                "mission_id": "mission-01",
                "trajectory_seed": 2026080101,
                "range_noise_seed": 2026081101,
                "frames": 1,
                "conditions": ["dynamic_primary", "fixed_fim_ablation"],
            }
            identity = {
                "schema_version": "cbf2026-qualified-evidence-v1",
                "campaign_id": mission["campaign_id"],
                "condition": "dynamic_primary",
                "trajectory_seed": mission["trajectory_seed"],
                "range_noise_seed": mission["range_noise_seed"],
            }
            rows = [
                {
                    **identity,
                    "record_type": "initialization",
                    "frame_index": 0,
                    "robot_id": robot_id,
                    "runtime": {"local_index": robot_id},
                    "analyzer_only": {
                        "truth_position": [float(robot_id), 0.0]
                    },
                }
                for robot_id in range(1, 15)
            ]
            reset = rejected_verified_reset_fixture(identity)
            rows.extend((
                reset,
                {
                    **identity,
                    "record_type": "controller_interval",
                    "frame_index": 0,
                    "runtime": {
                        "snapshot_version": 0,
                        "allocation_version": 1,
                        "nodes": [
                            {
                                "robot_id": robot_id,
                                "snapshot_version": 0,
                                "optimization_status": "not-attempted",
                                "applied_command": None,
                                "committed_hard_problem_id": "unavailable",
                                "consumed_hard_problem_id": "unavailable",
                            }
                            for robot_id in range(1, 15)
                        ],
                        "expected_node_count": 14,
                        "expected_endpoint_row_count": 232,
                        "expected_reconstructed_row_count": 119,
                        "observed_endpoint_row_count": 0,
                        "abort_reason": (
                            "theorem certificate reset rejected: "
                            + reset["runtime"]["reason"]
                        ),
                        "complete": False,
                    },
                    "analyzer_only": {},
                },
                {
                    **identity,
                    "record_type": "mission_terminal",
                    "frame_index": mission["frames"],
                    "runtime": {
                        "success": False,
                        "reason": "bootstrap_failure",
                        "process_outcome": "bootstrap_failure",
                        "declared_frames": mission["frames"],
                        "completed_intervals": 0,
                    },
                },
            ))
            with gzip.open(
                root / "swarm.jsonl.gz", "wt", encoding="utf-8"
            ) as sink:
                for row in rows:
                    sink.write(json.dumps(row, separators=(",", ":")) + "\n")

            ProductionOperations().synthesize_failed_mission(
                mission, root, "mission_declared_unsuccessful"
            )

            _validate_synthesized_missing_stream(
                root / "synthetic-missing.jsonl.gz",
                mission,
                "mission_declared_unsuccessful",
                mission_root=root,
            )

    def test_campaign_terminal_counts_are_exact_and_status_consistent(self):
        with tempfile.TemporaryDirectory(prefix="qualified-campaign-state-") as directory:
            root = Path(directory)
            protocol_path = root / "protocol.json"
            authorization_path = root / "authorization.json"
            protocol_path.write_text("{}\n")
            authorization_path.write_text("{}\n")
            registered = [{
                "mission_id": "mission-01",
                "trajectory_seed": 2026080101,
                "range_noise_seed": 2026081101,
                "frames": 1000,
            }]
            mission = {
                "campaign_id": "development-v4",
                **registered[0],
                "horizon_s": 500.0,
                "conditions": ["dynamic_primary", "fixed_fim_ablation"],
            }
            (root / "mission-01").mkdir()
            mission_manifest = root / "mission-01" / "manifest.json"
            mission_manifest.write_text(json.dumps({
                "schema_version": "cbf2026-qualified-mission-manifest-v1",
                "terminal": True,
                "status": "failed",
                "reason": "launch_failure",
                "mission": mission,
                "member_identities": {},
            }))
            schedule_path = root / "schedule.json"
            write_schedule_no_replace(schedule_path, [mission])
            campaign = {
                "schema_version": "cbf2026-qualified-campaign-manifest-v1",
                "terminal": True,
                "status": "failed",
                "reason": "launch_failure",
                "mission_count": 1,
                "completed_mission_count": 0,
                "protocol_sha256": hashlib.sha256(
                    protocol_path.read_bytes()
                ).hexdigest(),
                "authorization_sha256": hashlib.sha256(
                    authorization_path.read_bytes()
                ).hexdigest(),
                "schedule_sha256": hashlib.sha256(
                    schedule_path.read_bytes()
                ).hexdigest(),
                "schedule_bytes": schedule_path.stat().st_size,
                "mission_manifest_identities": {
                    "mission-01": {
                        "sha256": hashlib.sha256(
                            mission_manifest.read_bytes()
                        ).hexdigest(),
                        "bytes": mission_manifest.stat().st_size,
                    }
                },
            }
            campaign_path = root / "manifest.json"
            campaign_path.write_text(json.dumps(campaign))
            _preflight_raw_campaign(
                root,
                [mission],
                protocol_sha256=campaign["protocol_sha256"],
                authorization_sha256=campaign["authorization_sha256"],
            )

            mission_manifest_doc = json.loads(mission_manifest.read_text())

            def publish_consistent_schedule(candidate):
                schedule_path.write_text(json.dumps({
                    "schema_version": "cbf2026-qualified-campaign-schedule-v1",
                    "mission_count": 1,
                    "missions": [candidate],
                }))
                mission_manifest.write_text(json.dumps({
                    **mission_manifest_doc,
                    "mission": candidate,
                }))
                rewritten = {
                    **campaign,
                    "schedule_sha256": hashlib.sha256(
                        schedule_path.read_bytes()
                    ).hexdigest(),
                    "schedule_bytes": schedule_path.stat().st_size,
                    "mission_manifest_identities": {
                        "mission-01": {
                            "sha256": hashlib.sha256(
                                mission_manifest.read_bytes()
                            ).hexdigest(),
                            "bytes": mission_manifest.stat().st_size,
                        }
                    },
                }
                campaign_path.write_text(json.dumps(rewritten))

            consistent_tampers = (
                {**mission, "campaign_id": "development-v2"},
                {**mission, "horizon_s": 123.0},
                {
                    **mission,
                    "conditions": ["fixed_fim_ablation", "dynamic_primary"],
                },
            )
            for candidate in consistent_tampers:
                with self.subTest(candidate=candidate):
                    publish_consistent_schedule(candidate)
                    with self.assertRaisesRegex(
                        ValueError, "raw schedule differs from registered protocol"
                    ):
                        _preflight_raw_campaign(
                            root,
                            [mission],
                            protocol_sha256=campaign["protocol_sha256"],
                            authorization_sha256=campaign["authorization_sha256"],
                        )

            publish_consistent_schedule(mission)

            for field, value in (
                ("completed_mission_count", False),
                ("reason", "completed"),
            ):
                with self.subTest(field=field):
                    tampered = {**campaign, field: value}
                    campaign_path.write_text(json.dumps(tampered))
                    with self.assertRaisesRegex(
                        ValueError, "campaign manifest identity"
                    ):
                        _preflight_raw_campaign(
                            root,
                            [mission],
                            protocol_sha256=campaign["protocol_sha256"],
                            authorization_sha256=campaign["authorization_sha256"],
                        )


class ProducerNamespaceIdentityTests(unittest.TestCase):
    def identities(self):
        runtime_sha = "a" * 64
        command_sha = "b" * 64
        config_sha = "c" * 64
        stream_id = "d" * 64
        measurement_manifest = {
            "measurement_stream_id": stream_id,
            "config_sha256": config_sha,
            "row_count": 17,
        }
        measurement = {"sha256": runtime_sha, "bytes": 101}
        command = {"sha256": command_sha, "bytes": 202}
        config = {"sha256": config_sha, "bytes": 303}
        declared = {
            "measurements/runtime.jsonl.gz": measurement,
            "swarm-inputs/commands.jsonl.gz": command,
            "materialized-primary.json": config,
            "materialized-dynamic_primary.json": config,
        }
        producer = {
            "measurements.jsonl.gz": measurement,
            "commands.jsonl.gz": command,
            "config.json": config,
            "measurements.manifest.json": _canonical_json_identity({
                "schema_version": "cbf2026-qualified-producer-measurements-v1",
                "terminal": True,
                "status": "completed",
                "sha256": runtime_sha,
                "measurement_stream_id": stream_id,
                "config_sha256": config_sha,
                "row_count": 17,
            }),
            "commands.manifest.json": _canonical_json_identity({
                "schema_version": "cbf2026-qualified-producer-commands-v1",
                "terminal": True,
                "status": "completed",
                "sha256": command_sha,
            }),
        }
        replay = {"config_sha256": config_sha}
        return producer, declared, replay, measurement_manifest, runtime_sha

    def validate(self, producer, declared, replay, manifest, runtime_sha):
        _validate_producer_namespace_identities(
            producer,
            declared_members=declared,
            replay=replay,
            condition="dynamic_primary",
            measurement_manifest=manifest,
            runtime_sha=runtime_sha,
            mission_id="mission-01",
        )

    def test_all_five_members_are_bound_to_mission_and_canonical_manifests(self):
        values = self.identities()
        self.validate(*values)

        for member in ("config.json", "measurements.manifest.json",
                       "commands.manifest.json"):
            with self.subTest(member=member):
                tampered = copy.deepcopy(values[0])
                tampered[member]["sha256"] = "e" * 64
                with self.assertRaisesRegex(ValueError, "producer namespace"):
                    self.validate(tampered, *values[1:])

        self_rewritten = copy.deepcopy(values)
        self_rewritten[0]["config.json"] = {"sha256": "e" * 64, "bytes": 303}
        self_rewritten[2]["config_sha256"] = "e" * 64
        with self.assertRaisesRegex(ValueError, "producer namespace"):
            self.validate(*self_rewritten)

    def test_producer_argv_binds_entrypoint_condition_hash_frames_and_paths(self):
        replay_script = Path(__file__).resolve().parents[1] / (
            "scripts/diagnostics/replay_qualified_estimator.py"
        )
        measurement_sha = "a" * 64
        allowed = {
            "measurements.jsonl.gz", "measurements.manifest.json",
            "commands.jsonl.gz", "commands.manifest.json", "config.json",
        }
        argv = [
            sys.executable, str(replay_script),
            "--condition", "dynamic_primary",
            "--measurements", "measurements.jsonl.gz",
            "--measurement-sha256", measurement_sha,
            "--measurement-manifest", "measurements.manifest.json",
            "--commands", "commands.jsonl.gz",
            "--commands-manifest", "commands.manifest.json",
            "--config", "config.json",
            "--frames", "1000",
        ]
        kwargs = {
            "condition": "dynamic_primary",
            "measurement_sha256": measurement_sha,
            "frames": 1000,
        }
        self.assertEqual(_producer_input_violations(argv, allowed, **kwargs), 0)
        for flag, value in (
            ("--condition", "fixed_fim_ablation"),
            ("--measurement-sha256", "b" * 64),
            ("--frames", "999"),
            ("--config", "alternate.json"),
        ):
            with self.subTest(flag=flag):
                tampered = list(argv)
                tampered[tampered.index(flag) + 1] = value
                self.assertEqual(
                    _producer_input_violations(tampered, allowed, **kwargs), 1
                )

class DevelopmentV5InitialStateAnalyzerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        (
            cls.family_path,
            cls.family,
            cls.audited,
            cls.initial_state,
            cls.family_identity,
        ) = frozen_initial_family_fixture()
        cls.by_seed = {
            audit.seed: audit for audit in cls.audited.registered.audits
        }

    def protocol(self):
        missions = [
            {
                "mission_id": f"mission-{index:02d}",
                "trajectory_seed": audit.seed,
                "range_noise_seed": 2026081200 + index,
                "frames": 1000,
                "initial_positions_sha256": audit.positions_sha256,
            }
            for index, audit in enumerate(
                self.audited.registered.audits, start=1
            )
        ]
        return {
            "kind": "development",
            "version": "v5",
            "bindings": {"initial_family": copy.deepcopy(self.family_identity)},
            "initial_state": copy.deepcopy(self.initial_state),
            "schedule": {"missions": missions},
        }

    def test_v5_schedule_envelope_propagates_exact_initial_positions_hash(self):
        audit = self.by_seed[2026080201]
        mission = {
            "mission_id": "mission-01",
            "trajectory_seed": audit.seed,
            "range_noise_seed": 2026081201,
            "frames": 1000,
            "initial_positions_sha256": audit.positions_sha256,
        }
        self.assertEqual(
            _registered_schedule_envelope(
                [mission], campaign_id="development-v5"
            ),
            [{
                "campaign_id": "development-v5",
                **mission,
                "horizon_s": 500.0,
                "conditions": ["dynamic_primary", "fixed_fim_ablation"],
            }],
        )
        for tampered in (
            {key: value for key, value in mission.items()
             if key != "initial_positions_sha256"},
            {**mission, "initial_positions_sha256": "not-a-sha"},
        ):
            with self.subTest(tampered=tampered):
                with self.assertRaisesRegex(ValueError, "initial|schedule"):
                    _registered_schedule_envelope(
                        [tampered], campaign_id="development-v5"
                    )

    def test_family_reaudit_binds_all_100_seeds_and_rejects_seed_hash_substitution(self):
        contract = _load_development_initial_state_contract(self.protocol())
        self.assertEqual(set(contract), set(range(2026080201, 2026080211)))
        self.assertEqual(
            contract[2026080201].positions_sha256,
            self.by_seed[2026080201].positions_sha256,
        )

        tampered = self.protocol()
        tampered["initial_state"]["missions"][0]["positions_sha256"] = (
            self.by_seed[2026080202].positions_sha256
        )
        with self.assertRaisesRegex(ValueError, "initial state|position"):
            _load_development_initial_state_contract(tampered)

        tampered = self.protocol()
        tampered["initial_state"]["frozen_summary"]["audit"][
            "minimum_pair_distance_m"
        ] += 1.0
        with self.assertRaisesRegex(ValueError, "initial state"):
            _load_development_initial_state_contract(tampered)

        with tempfile.TemporaryDirectory(
            prefix="qualified-v5-family-identity-"
        ) as directory:
            root = Path(directory)
            duplicate = root / "duplicate-family.json"
            duplicate.write_text(
                self.family_path.read_text().replace(
                    "{",
                    '{"schema_version":"cbf2026-qualified-initial-family-v1",',
                    1,
                )
            )
            tampered = self.protocol()
            tampered["bindings"]["initial_family"] = {
                "path": str(duplicate.resolve()),
                "sha256": hashlib.sha256(duplicate.read_bytes()).hexdigest(),
                "bytes": duplicate.stat().st_size,
            }
            with self.assertRaisesRegex(ValueError, "duplicate JSON key"):
                _load_development_initial_state_contract(tampered)

    def test_content_identity_cache_still_reloads_and_revalidates_family(self):
        analyzer_module._INITIAL_FAMILY_AUDIT_CACHE.clear()
        with (
            mock.patch.object(
                initial_state_module,
                "load_qualified_initial_family",
                wraps=initial_state_module.load_qualified_initial_family,
            ) as loader,
            mock.patch.object(
                initial_state_module,
                "audit_frozen_initial_family",
                wraps=initial_state_module.audit_frozen_initial_family,
            ) as auditor,
        ):
            _load_development_initial_state_contract(self.protocol())
            _load_development_initial_state_contract(self.protocol())
        self.assertEqual(loader.call_count, 2)
        self.assertEqual(auditor.call_count, 1)

    def test_bad_initial_family_fails_before_analysis_output_root_is_claimed(self):
        with tempfile.TemporaryDirectory(
            prefix="qualified-v5-family-pre-root-"
        ) as directory:
            root = Path(directory)
            protocol = self.protocol()
            protocol["initial_state"]["missions"][0][
                "positions_sha256"
            ] = self.by_seed[2026080202].positions_sha256
            protocol_path = root / "protocol.json"
            protocol_path.write_text(json.dumps(protocol))
            arguments = argparse.Namespace(
                kind="development", version="v5", smoke_id=None,
                protocol=protocol_path,
                authorization=root / "authorization.json",
                input_root=root / "raw-v5",
                ablation_config=root / "ablation.json",
                output_root=root / "analysis-v5",
            )
            contract = {
                "claimed_root": "raw",
                "registered_schedule": [],
            }
            with (
                mock.patch(
                    "scripts.diagnostics.analyze_qualified_closure_campaign."
                    "validate_analysis_registration_contract",
                    return_value=contract,
                ),
                mock.patch.object(
                    registrar,
                    "validate_authorization_binding",
                    return_value={"implementation_identity": "a" * 40},
                ),
            ):
                with self.assertRaisesRegex(ValueError, "initial state|position"):
                    analyze_campaign_from_arguments(arguments)
            self.assertFalse(arguments.output_root.exists())

    def materialized_fixture(self, root, seed=2026080201):
        audit = self.by_seed[seed]
        exact_position = {
            "method": "specified",
            "positions": [list(position) for position in audit.positions],
        }
        documents = {
            "materialized-primary.json": {
                "initial": {"position": copy.deepcopy(exact_position)},
                "condition": "dynamic_primary",
            },
            "materialized-dynamic_primary.json": {
                "initial": {"position": copy.deepcopy(exact_position)},
                "condition": "dynamic_primary",
            },
            "materialized-fixed_fim_ablation.json": {
                "initial": {"position": copy.deepcopy(exact_position)},
                "condition": "fixed_fim_ablation",
            },
        }
        identities = {}
        for name, document in documents.items():
            path = root / name
            path.write_text(json.dumps(document, sort_keys=True))
            identities[name] = {
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "bytes": path.stat().st_size,
            }
        manifest = {
            "member_identities": identities,
            "swarm": {"config_sha256": identities[
                "materialized-primary.json"
            ]["sha256"]},
        }
        mission = {
            "mission_id": "mission-01",
            "trajectory_seed": seed,
            "initial_positions_sha256": audit.positions_sha256,
        }
        return audit, documents, manifest, mission

    def test_completed_mission_requires_exact_specified_materialized_initial_state(self):
        with tempfile.TemporaryDirectory(
            prefix="qualified-v5-materialized-initial-"
        ) as directory:
            root = Path(directory)
            audit, _, manifest, mission = self.materialized_fixture(root)
            primary_identity = _validate_completed_mission_initial_configs(
                root, manifest, mission, audit
            )
            self.assertEqual(
                primary_identity,
                manifest["member_identities"]["materialized-primary.json"],
            )

            path = root / "materialized-fixed_fim_ablation.json"
            tampered = json.loads(path.read_text())
            tampered["initial"]["position"]["method"] = "random-in-polygon"
            path.write_text(json.dumps(tampered, sort_keys=True))
            manifest["member_identities"][path.name] = {
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "bytes": path.stat().st_size,
            }
            with self.assertRaisesRegex(ValueError, "materialized initial"):
                _validate_completed_mission_initial_configs(
                    root, manifest, mission, audit
                )

        with tempfile.TemporaryDirectory(
            prefix="qualified-v5-rehashed-initial-"
        ) as directory:
            root = Path(directory)
            audit, documents, manifest, mission = self.materialized_fixture(root)
            for name, document in documents.items():
                document["initial"]["position"]["positions"][0][0] += 1e-9
                path = root / name
                path.write_text(json.dumps(document, sort_keys=True))
                manifest["member_identities"][name] = {
                    "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "bytes": path.stat().st_size,
                }
            manifest["swarm"]["config_sha256"] = manifest[
                "member_identities"
            ]["materialized-primary.json"]["sha256"]
            with self.assertRaisesRegex(ValueError, "materialized initial"):
                _validate_completed_mission_initial_configs(
                    root, manifest, mission, audit
                )

    def test_materialized_config_parses_the_same_bytes_that_are_authenticated(self):
        with tempfile.TemporaryDirectory(
            prefix="qualified-v5-materialized-buffer-"
        ) as directory:
            root = Path(directory)
            audit, _, manifest, mission = self.materialized_fixture(root)
            target = root / "materialized-fixed_fim_ablation.json"
            authenticated = target.read_bytes()
            replacement = authenticated.replace(
                b'"specified"', b'"forbidden"', 1
            )
            self.assertEqual(len(replacement), len(authenticated))
            original_open = Path.open
            state = {"wrapped": False, "swapped": False}

            class SwapAfterAuthenticatedRead:
                def __init__(self, stream):
                    self.stream = stream

                def __enter__(self):
                    self.stream.__enter__()
                    return self

                def __exit__(self, *arguments):
                    return self.stream.__exit__(*arguments)

                def read(self, *arguments):
                    payload = self.stream.read(*arguments)
                    if payload and not state["swapped"]:
                        state["swapped"] = True
                        with original_open(target, "wb") as output:
                            output.write(replacement)
                    return payload

                def __getattr__(self, name):
                    return getattr(self.stream, name)

            def swapping_open(path, mode="r", *arguments, **keywords):
                stream = original_open(path, mode, *arguments, **keywords)
                if (
                    Path(path) == target
                    and mode == "rb"
                    and not state["wrapped"]
                ):
                    state["wrapped"] = True
                    return SwapAfterAuthenticatedRead(stream)
                return stream

            with mock.patch.object(Path, "open", new=swapping_open):
                primary_identity = _validate_completed_mission_initial_configs(
                    root, manifest, mission, audit
                )

            self.assertTrue(state["swapped"])
            self.assertEqual(
                primary_identity,
                manifest["member_identities"]["materialized-primary.json"],
            )

    def test_initialization_and_frame_zero_truth_are_exact_frozen_coordinates(self):
        audit = self.by_seed[2026080201]
        initialization = {
            "record_type": "initialization",
            "robot_id": 1,
            "analyzer_only": {"truth_position": list(audit.positions[0])},
        }
        _validate_swarm_initial_truth(initialization, audit)
        tampered = copy.deepcopy(initialization)
        tampered["analyzer_only"]["truth_position"][0] += 1e-9
        with self.assertRaisesRegex(ValueError, "initialization truth"):
            _validate_swarm_initial_truth(tampered, audit)

        controller = {
            "record_type": "controller_interval",
            "frame_index": 0,
            "analyzer_only": {"truth": [
                {"robot_id": robot_id, "position": list(position)}
                for robot_id, position in enumerate(audit.positions, start=1)
            ]},
        }
        _validate_swarm_initial_truth(controller, audit)
        tampered = copy.deepcopy(controller)
        tampered["analyzer_only"]["truth"][1]["position"] = list(
            audit.positions[0]
        )
        with self.assertRaisesRegex(ValueError, "frame-zero truth"):
            _validate_swarm_initial_truth(tampered, audit)

    def test_measurement_config_hash_links_primary_member_swarm_and_runtime(self):
        with tempfile.TemporaryDirectory(
            prefix="qualified-v5-measurement-config-link-"
        ) as directory:
            root = Path(directory)
            audit, _, manifest, mission = self.materialized_fixture(root)
            primary = _validate_completed_mission_initial_configs(
                root, manifest, mission, audit
            )
            measurement = {"config_sha256": primary["sha256"]}
            _validate_measurement_config_link(
                measurement, manifest, primary, mission["mission_id"]
            )
            for target in (measurement, manifest["swarm"]):
                with self.subTest(target=target):
                    changed = copy.deepcopy(target)
                    changed["config_sha256"] = "e" * 64
                    with self.assertRaisesRegex(ValueError, "config SHA"):
                        _validate_measurement_config_link(
                            changed if target is measurement else measurement,
                            {
                                **manifest,
                                "swarm": changed if target is manifest["swarm"]
                                else manifest["swarm"],
                            },
                            primary,
                            mission["mission_id"],
                        )


class AnalyzerCliTests(unittest.TestCase):
    def development_arguments(self, root):
        return argparse.Namespace(
            kind="development", version="v5", smoke_id=None,
            protocol=root / "absent-protocol.json",
            authorization=root / "absent-authorization.json",
            input_root=root / "absent-raw" / "v5",
            ablation_config=root / "absent-ablation.json",
            output_root=root / "analysis" / "v5",
        )

    def test_runtime_development_analyzer_argv_uses_v5_module_entrypoint_exactly(self):
        root = Path("/private/tmp/qualified-runtime-analyzer-argv")
        arguments = self.development_arguments(root)

        self.assertEqual(_runtime_analyzer_argv(arguments), [
            "conda", "run", "-n", "cbf_env", "python", "-m",
            "scripts.diagnostics.analyze_qualified_closure_campaign",
            "--kind", "development", "--version", "v5",
            "--protocol", str(root / "absent-protocol.json"),
            "--authorization", str(root / "absent-authorization.json"),
            "--input-root", str(root / "absent-raw" / "v5"),
            "--ablation-config", str(root / "absent-ablation.json"),
            "--output-root", str(root / "analysis" / "v5"),
        ])

    def test_frozen_analyzer_reaches_registration_without_claiming_root(self):
        repository = Path(__file__).resolve().parents[1]
        environment = dict(os.environ)
        environment.pop("PYTHONPATH", None)
        with tempfile.TemporaryDirectory(
            prefix="qualified-analyzer-module-cli-"
        ) as directory:
            root = Path(directory)
            arguments = self.development_arguments(root)
            result = subprocess.run(
                _runtime_analyzer_argv(arguments),
                cwd=repository,
                env=environment,
                text=True,
                capture_output=True,
                check=False,
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("registered protocol must be a regular file", result.stderr)
            self.assertNotIn("No module named 'scripts'", result.stderr)
            self.assertFalse(arguments.output_root.exists())

    def test_runtime_analyzer_requires_version_exact_argv_and_bound_ablation_path(self):
        with tempfile.TemporaryDirectory(prefix="qualified-analysis-binding-") as directory:
            root = Path(directory)
            raw, output = root / "raw", root / "analysis"
            ablation = root / "ablation.json"
            ablation.write_text("{}\n")
            arguments = argparse.Namespace(
                kind="development", version="v5", smoke_id=None,
                protocol=root / "fixture-protocol.json",
                authorization=root / "fixture-authorization.json",
                input_root=raw, output_root=output,
                ablation_config=ablation,
            )
            protocol = {
                "kind": "development", "version": "v5",
                "roots": {
                    "raw": str(raw.resolve()),
                    "analysis": str(output.resolve()),
                },
                "schedule": {"missions": [{
                    "mission_id": "mission-01",
                    "trajectory_seed": 2026080201,
                    "range_noise_seed": 2026081201,
                    "frames": 1000,
                    "initial_positions_sha256": "a" * 64,
                }]},
                "bindings": {"ablation_config": {
                    "path": str(ablation.resolve()),
                    "sha256": hashlib.sha256(ablation.read_bytes()).hexdigest(),
                    "bytes": ablation.stat().st_size,
                }},
                "analyzer_argv": _runtime_analyzer_argv(arguments),
            }
            with mock.patch(
                "scripts.diagnostics.analyze_qualified_closure_campaign."
                "_load_development_initial_state_contract",
                return_value={},
            ):
                contract = validate_analysis_registration_contract(protocol, arguments)
            self.assertEqual(contract["registered_schedule"], [{
                "campaign_id": "development-v5",
                "mission_id": "mission-01",
                "trajectory_seed": 2026080201,
                "range_noise_seed": 2026081201,
                "frames": 1000,
                "initial_positions_sha256": "a" * 64,
                "horizon_s": 500.0,
                "conditions": ["dynamic_primary", "fixed_fim_ablation"],
            }])

            substitute = root / "same-content-ablation.json"
            substitute.write_text("{}\n")
            arguments.ablation_config = substitute
            with self.assertRaisesRegex(ValueError, "ablation config identity"):
                validate_analysis_registration_contract(protocol, arguments)
            arguments.ablation_config = ablation
            arguments.version = None
            with self.assertRaisesRegex(ValueError, "version"):
                validate_analysis_registration_contract(protocol, arguments)
            arguments.version = "v5"
            protocol["analyzer_argv"] = [*protocol["analyzer_argv"], "--extra"]
            with self.assertRaisesRegex(ValueError, "analyzer argv"):
                validate_analysis_registration_contract(protocol, arguments)

            protocol["analyzer_argv"] = _runtime_analyzer_argv(arguments)
            for version in ("v1", "v2", "v3", "v4"):
                with self.subTest(version=version):
                    arguments.version = version
                    protocol["version"] = version
                    with self.assertRaisesRegex(ValueError, "development.*v5"):
                        validate_analysis_registration_contract(protocol, arguments)

    def test_mission_success_is_derived_from_completion_and_local_evidence(self):
        mission = {"frames": 1000}
        terminal = {
            "success": True,
            "reason": "completed",
            "process_outcome": "completed",
            "declared_frames": 1000,
            "completed_intervals": 1000,
        }
        self.assertTrue(_derive_mission_success(
            terminal, mission, local_violations=0
        ))
        for field, value in (
            ("completed_intervals", 999),
            ("process_outcome", "loop_failure"),
        ):
            with self.subTest(field=field):
                tampered = {**terminal, field: value}
                self.assertFalse(_derive_mission_success(
                    tampered, mission, local_violations=0
                ))
        self.assertFalse(_derive_mission_success(
            terminal, mission, local_violations=1
        ))
        declared_false = {**terminal, "success": False}
        self.assertTrue(_derive_mission_success(
            declared_false, mission, local_violations=0
        ))

    def test_semantic_analysis_hash_excludes_only_declared_transport_identity(self):
        first = {
            "schema_version": "cbf2026-qualified-closure-analysis-v1",
            "passed": True,
            "gates": {"complete": {"passed": True}},
            "raw_campaign_manifest_sha256": "a" * 64,
        }
        second = {**first, "raw_campaign_manifest_sha256": "b" * 64}
        self.assertEqual(
            semantic_analysis_sha256(first), semantic_analysis_sha256(second)
        )
        second["gates"] = {"complete": {"passed": False}}
        self.assertNotEqual(
            semantic_analysis_sha256(first), semantic_analysis_sha256(second)
        )

    def test_unavailable_controller_fails_availability_without_nu_claim(self):
        campaign = passing_campaign()
        campaign["controller_rows"][0].update({
            "certificate_available": False,
            "nu_inst": 1000.0,
            "bar_nu": 0.0,
        })

        report = analyze_qualified_closure_campaign(campaign)

        self.assertEqual(
            report["gates"]["controller_certificate_availability"]["pass_fail"],
            "FAIL",
        )
        self.assertEqual(
            report["gates"]["zero_nu_bound_excess"]["pass_fail"], "PASS"
        )

    def test_analysis_requires_eight_gb_before_root_claim(self):
        with tempfile.TemporaryDirectory(prefix="qualified-analysis-space-") as directory:
            output = Path(directory) / "analysis" / "v1"
            with self.assertRaisesRegex(RuntimeError, "8 GB"):
                validate_analysis_output_root(
                    output, available_bytes_fn=lambda _path: 7_999_999_999
                )
            self.assertFalse(output.exists())
            validate_analysis_output_root(
                output, available_bytes_fn=lambda _path: 8_000_000_000
            )
            self.assertFalse(output.exists())

    def test_help_exposes_exact_analysis_options(self):
        script = Path(__file__).resolve().parents[1] / "scripts" / "diagnostics" / "analyze_qualified_closure_campaign.py"
        result = subprocess.run(
            [sys.executable, str(script), "--help"],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        for option in (
            "--kind", "--version", "--smoke-id", "--protocol",
            "--authorization", "--input-root", "--ablation-config",
            "--output-root",
        ):
            self.assertIn(option, result.stdout)

    def test_confirmatory_smoke_uses_only_its_registered_roots_and_universe(self):
        with tempfile.TemporaryDirectory(prefix="qualified-smoke-analysis-") as directory:
            root = Path(directory)
            protocol = {
                "kind": "confirmatory", "version": "v1",
                "roots": {
                    "smoke_a_raw": str((root / "raw-a").resolve()),
                    "smoke_a_analysis": str((root / "analysis-a").resolve()),
                    "smoke_b_raw": str((root / "raw-b").resolve()),
                    "smoke_b_analysis": str((root / "analysis-b").resolve()),
                },
                "smoke_schedule": {
                    "trajectory_seed": 2026089001,
                    "range_noise_seed": 2026089101,
                    "frames": 20,
                    "universes": {"mission": 1, "controller": 20},
                    "included_in_scientific_denominator": False,
                },
                "universes": {"mission": 60, "controller": 60000},
            }
            arguments = argparse.Namespace(
                kind="confirmatory-smoke", version="v1", smoke_id="a",
                protocol=root / "protocol.json",
                authorization=root / "authorization.json",
                ablation_config=None,
                input_root=root / "raw-a", output_root=root / "analysis-a",
            )
            protocol["smoke_argv"] = {
                "a": {"analyzer": _runtime_analyzer_argv(arguments)}
            }

            contract = validate_analysis_registration_contract(protocol, arguments)

            self.assertEqual(contract["claimed_root"], "smoke_a_raw")
            self.assertEqual(contract["analysis_protocol"]["universes"], {
                "mission": 1, "controller": 20,
            })
            self.assertEqual(contract["registered_missions"], [{
                "mission_id": "mission-01", "trajectory_seed": 2026089001,
                "range_noise_seed": 2026089101, "frames": 20,
            }])
            self.assertEqual(contract["registered_schedule"], [{
                "campaign_id": "confirmatory-smoke-a",
                "mission_id": "mission-01", "trajectory_seed": 2026089001,
                "range_noise_seed": 2026089101, "frames": 20,
                "horizon_s": 10.0,
                "conditions": ["dynamic_primary", "fixed_fim_ablation"],
            }])
            arguments.input_root = root / "raw-b"
            with self.assertRaisesRegex(ValueError, "input root"):
                validate_analysis_registration_contract(protocol, arguments)

    def test_confirmatory_contract_derives_full_v1_schedule_envelope(self):
        with tempfile.TemporaryDirectory(prefix="qualified-confirmatory-analysis-") as directory:
            root = Path(directory)
            arguments = argparse.Namespace(
                kind="confirmatory", version="v1", smoke_id=None,
                protocol=root / "protocol.json",
                authorization=root / "authorization.json",
                ablation_config=None,
                input_root=root / "raw", output_root=root / "analysis",
            )
            protocol = {
                "kind": "confirmatory",
                "version": "v1",
                "roots": {
                    "raw": str(arguments.input_root.resolve()),
                    "analysis": str(arguments.output_root.resolve()),
                },
                "schedule": {"missions": [{
                    "mission_id": "mission-01",
                    "trajectory_seed": 2026082001,
                    "range_noise_seed": 2026083001,
                    "frames": 1000,
                }]},
                "analyzer_argv": _runtime_analyzer_argv(arguments),
            }

            contract = validate_analysis_registration_contract(protocol, arguments)

            self.assertEqual(contract["registered_schedule"], [{
                "campaign_id": "confirmatory-v1",
                "mission_id": "mission-01",
                "trajectory_seed": 2026082001,
                "range_noise_seed": 2026083001,
                "frames": 1000,
                "horizon_s": 500.0,
                "conditions": ["dynamic_primary", "fixed_fim_ablation"],
            }])

    def test_terminal_raw_member_tamper_terminalizes_claimed_analysis_root(self):
        with tempfile.TemporaryDirectory(prefix="qualified-analyzer-cli-") as directory:
            root = Path(directory).resolve()
            raw = root / "raw"
            analysis = root / "analysis"
            protocol, protocol_path, authorization_path = (
                write_development_registration(root, raw, analysis)
            )
            missions = []
            for row in protocol["schedule"]["missions"]:
                missions.append({
                    "campaign_id": "development-v5", **row,
                    "horizon_s": 500.0,
                    "conditions": ["dynamic_primary", "fixed_fim_ablation"],
                })
            mission_root = raw / "mission-01"
            mission_root.mkdir(parents=True)
            member = mission_root / "swarm.jsonl.gz"
            member.write_bytes(b"tampered")
            mission = missions[0]
            (mission_root / "manifest.json").write_text(json.dumps({
                "schema_version": "cbf2026-qualified-mission-manifest-v1",
                "terminal": True, "status": "completed", "mission": mission,
                "swarm": {}, "measurements": {}, "replays": {},
                "member_identities": {"swarm.jsonl.gz": {"sha256": "0" * 64, "bytes": 8}},
            }))
            for mission in missions[1:]:
                other = raw / mission["mission_id"]
                other.mkdir()
                (other / "manifest.json").write_text(json.dumps({
                    "schema_version": "cbf2026-qualified-mission-manifest-v1",
                    "terminal": True, "status": "failed", "reason": "not_started",
                    "mission": mission, "member_identities": {},
                }))
            write_schedule_no_replace(raw / "schedule.json", missions)
            write_raw_campaign_manifest(
                raw, missions, protocol_path, authorization_path,
                status="failed", completed=0,
            )
            stderr = io.StringIO()
            ablation_token = protocol["analyzer_argv"][
                protocol["analyzer_argv"].index("--ablation-config") + 1
            ]
            with (
                development_registration_patches(protocol),
                chdir(Path(protocol["bindings"]["ablation_config"]["path"]).parent),
                redirect_stderr(stderr),
            ):
                returncode = analyzer_main([
                    "--kind", "development",
                    "--version", "v5",
                    "--protocol", str(protocol_path),
                    "--authorization", str(authorization_path),
                    "--ablation-config", ablation_token,
                    "--input-root", str(raw),
                    "--output-root", str(analysis),
                ])
            self.assertNotEqual(returncode, 0)
            self.assertIn("member identity differs", stderr.getvalue())
            failure = json.loads((analysis / "manifest.json").read_text())
            self.assertEqual(failure["status"], "failed")
            self.assertTrue(failure["terminal"])

    def test_complete_terminal_failure_bundle_is_streamed_to_terminal_analysis(self):
        with tempfile.TemporaryDirectory(prefix="qualified-analyzer-terminal-") as directory:
            root = Path(directory).resolve()
            raw, analysis = root / "raw", root / "analysis"
            protocol, protocol_path, authorization_path = (
                write_development_registration(root, raw, analysis)
            )
            raw.mkdir()
            missions = []
            for row in protocol["schedule"]["missions"]:
                mission = {
                    "campaign_id": "development-v5", **row, "horizon_s": 500.0,
                    "conditions": ["dynamic_primary", "fixed_fim_ablation"],
                }
                missions.append(mission)
                mission_root = raw / mission["mission_id"]
                mission_root.mkdir()
                synthetic = mission_root / "synthetic-missing.jsonl.gz"
                with gzip.open(synthetic, "wt", encoding="utf-8") as sink:
                    sink.write(json.dumps({
                        "record_type": "test-placeholder",
                        "reason": "launch_failure",
                    }) + "\n")
                (mission_root / "manifest.json").write_text(json.dumps({
                    "schema_version": "cbf2026-qualified-mission-manifest-v1",
                    "terminal": True, "status": "failed", "reason": "launch_failure",
                    "mission": mission,
                    "member_identities": {"synthetic-missing.jsonl.gz": {
                        "sha256": hashlib.sha256(synthetic.read_bytes()).hexdigest(),
                        "bytes": synthetic.stat().st_size,
                    }},
                }))
            write_schedule_no_replace(raw / "schedule.json", missions)
            write_raw_campaign_manifest(
                raw, missions, protocol_path, authorization_path,
                status="failed", completed=0,
            )
            stderr = io.StringIO()
            validated_missing = []

            def record_missing(path, mission, reason, *, mission_root):
                self.assertEqual(Path(mission_root), Path(path).parent)
                validated_missing.append((Path(path).name, mission["mission_id"], reason))

            ablation_token = protocol["analyzer_argv"][
                protocol["analyzer_argv"].index("--ablation-config") + 1
            ]
            with (
                development_registration_patches(protocol),
                chdir(Path(protocol["bindings"]["ablation_config"]["path"]).parent),
                mock.patch(
                    "scripts.diagnostics.analyze_qualified_closure_campaign."
                    "_validate_synthesized_missing_stream",
                    side_effect=record_missing,
                ),
                redirect_stderr(stderr),
            ):
                returncode = analyzer_main([
                    "--kind", "development",
                    "--version", "v5",
                    "--protocol", str(protocol_path),
                    "--authorization", str(authorization_path),
                    "--ablation-config", ablation_token,
                    "--input-root", str(raw),
                    "--output-root", str(analysis),
                ])
            self.assertEqual(returncode, 1, stderr.getvalue())
            self.assertEqual(json.loads((analysis / "manifest.json").read_text())["status"], "completed")
            report = json.loads((analysis / "analysis.json").read_text())
            self.assertFalse(report["passed"])
            self.assertEqual(len(report["incomplete_missions"]), 10)
            self.assertEqual(
                validated_missing,
                [
                    ("synthetic-missing.jsonl.gz", f"mission-{index:02d}",
                     "launch_failure")
                    for index in range(1, 11)
                ],
            )


if __name__ == "__main__":
    unittest.main()
