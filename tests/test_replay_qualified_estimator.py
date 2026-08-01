import copy
import json
import math
from pathlib import Path
import unittest

import numpy as np

from scripts.diagnostics.predictive_wnls import (
    canonical_qualified_solver,
    solve_finite_budget_wnls,
)
from scripts.diagnostics.replay_qualified_estimator import (
    RAW_SCHEMA_ID,
    build_qualified_replay_row,
    validate_qualified_replay_row,
)


FIXTURE_PATH = Path(
    "tests/fixtures/cbf2026_two_range_reacquisition/"
    "mechanism_20260727_180_12.json"
)


def registered_two_circle_inputs():
    fixture = json.loads(FIXTURE_PATH.read_text())
    references = [
        {
            "key": ("uav", 10),
            "position": [-1550.0, -300.0],
            "range": 189.8066398119302,
            "covariance": [[1.0, 0.0], [0.0, 1.0]],
            "ranging_sigma": 0.5,
            "base_anchor_provenance": [0, 1, 2],
        },
        {
            "key": ("uav", 11),
            "position": [-1550.0, 300.0],
            "range": 424.14308667822655,
            "covariance": [[1.0, 0.0], [0.0, 1.0]],
            "ranging_sigma": 0.5,
            "base_anchor_provenance": [0, 1, 2],
        },
    ]
    def solver_from_start(start, canonical_references):
        return solve_finite_budget_wnls(
            [record["position"] for record in canonical_references],
            [record["covariance"] for record in canonical_references],
            [record["range"] for record in canonical_references],
            start.estimate,
            canonical_references[0]["ranging_sigma"],
        )

    return fixture, references, canonical_qualified_solver(solver_from_start)


def registered_deployment_domain():
    return {
        "anchor_ids": [0, 2],
        "anchor_coordinates": [[-1550.0, -300.0], [-1550.0, 300.0]],
        "deployment_vertices": [[-1491.0, -200.0], [-1491.0, 200.0]],
        "unit_normal": [1.0, 0.0],
        "offset": 1550.0,
        "ocean_side": 1,
        "margin_m": 1.0,
        "domain_version": "ocean-side-v1",
    }


def valid_serialized_proposal_row(index=0):
    """Return one proposal row in the frozen serialized field order."""
    return {
        "accepted": False,
        "cost": 1.0,
        "damping": 0.001,
        "invalid_trial_reason": "invalid_trial_terms",
        "proposal": index,
        "raw_step_norm": 1.0,
        "stationarity_norm": 1.0,
        "trial_cost": None,
    }


def valid_failed_serialized_solver_result():
    """Return a bounded failed result in the frozen serialized field order."""
    return {
        "cost": 0.0,
        "covariance": None,
        "epsilon": None,
        "estimate": [0.0, 0.0],
        "failure_reason": "synthetic_failure",
        "fim_valid": False,
        "iterations": 0,
        "phi_condition": None,
        "phi_min_eigenvalue": None,
        "proposal_count": 0,
        "proposal_trace": [],
        "stationarity_norm": 0.0,
        "status": "failed",
    }


def malformed_serialized_solver_evidence_cases():
    """Build fresh malformed records shared by producer/analyzer probes."""
    formal = valid_failed_serialized_solver_result()
    formal.update({
        "epsilon": False,
        "fim_valid": 1,
        "proposal_count": True,
        "iterations": -7,
        "cost": "bad",
        "stationarity_norm": {},
        "failure_reason": 7,
    })
    too_many = valid_failed_serialized_solver_result()
    too_many["proposal_trace"] = [
        valid_serialized_proposal_row(index) for index in range(51)
    ]
    too_many["proposal_count"] = 51
    too_many["iterations"] = 51
    boolean_count = valid_failed_serialized_solver_result()
    boolean_count["proposal_count"] = True
    negative_iteration = valid_failed_serialized_solver_result()
    negative_iteration["iterations"] = -1
    nonstring_reason = valid_failed_serialized_solver_result()
    nonstring_reason["failure_reason"] = 17
    empty_reason = valid_failed_serialized_solver_result()
    empty_reason["failure_reason"] = ""
    unknown_status = valid_failed_serialized_solver_result()
    unknown_status["status"] = "mystery"
    nonfinite = valid_failed_serialized_solver_result()
    nonfinite["cost"] = float("inf")
    overflow = valid_failed_serialized_solver_result()
    overflow["cost"] = 10**10000
    forbidden = valid_failed_serialized_solver_result()
    forbidden["cost"] = {"truth_position": [0.0, 0.0]}
    deep = valid_failed_serialized_solver_result()
    nested = 0.0
    for _ in range(16):
        nested = [nested]
    deep["cost"] = nested
    oversized = valid_failed_serialized_solver_result()
    oversized["cost"] = [0.0] * 5000
    noncontiguous = valid_failed_serialized_solver_result()
    noncontiguous["proposal_trace"] = [valid_serialized_proposal_row(1)]
    noncontiguous["proposal_count"] = 1
    noncontiguous["iterations"] = 1
    count_mismatch = valid_failed_serialized_solver_result()
    count_mismatch["proposal_trace"] = [valid_serialized_proposal_row(0)]
    nonboolean_accepted = valid_failed_serialized_solver_result()
    proposal = valid_serialized_proposal_row(0)
    proposal["accepted"] = 1
    nonboolean_accepted["proposal_trace"] = [proposal]
    nonboolean_accepted["proposal_count"] = 1
    nonboolean_accepted["iterations"] = 1
    recursive = valid_failed_serialized_solver_result()
    cycle = []
    cycle.append(cycle)
    recursive["cost"] = cycle
    return (
        ("formal-reviewer-record", formal),
        ("51-row-trace", too_many),
        ("bool-as-int", boolean_count),
        ("negative-iteration", negative_iteration),
        ("non-string-reason", nonstring_reason),
        ("empty-reason", empty_reason),
        ("unknown-status", unknown_status),
        ("nonfinite-scalar", nonfinite),
        ("overflow-scalar", overflow),
        ("forbidden", forbidden),
        ("excessive-depth", deep),
        ("excessive-size", oversized),
        ("noncontiguous-proposals", noncontiguous),
        ("count-mismatch", count_mismatch),
        ("nonboolean-accepted", nonboolean_accepted),
        ("recursive-cycle", recursive),
    )


def build_registered_qualified_row(
    *,
    frame_index=180,
    applied_command_frame=179,
    history_version=180,
    qualifier_kind="unavailable",
    qualifier_payload=None,
    previous_public=None,
    previous_private=None,
    held_velocity=(0.0, 0.0),
    solver_from_start=None,
):
    fixture, references, solver = registered_two_circle_inputs()
    if solver_from_start is not None:
        solver = canonical_qualified_solver(
            lambda start, references: solver_from_start(start)
        )
    if qualifier_payload is None:
        qualifier_payload = {"reason": "qualifier_information_absent"}
    return build_qualified_replay_row(
        frame_index=frame_index,
        robot_id=fixture["key"]["robot_id"],
        squad_local_index=fixture["key"]["squad_local_index"],
        schedule_id=(
            f"frame-{frame_index}:robot-{fixture['key']['robot_id']}:"
            f"squad-local-{fixture['key']['squad_local_index']}"
        ),
        references=references,
        solver_from_start=solver,
        live_seed=None,
        private_seed=None,
        qualifier_kind=qualifier_kind,
        qualifier_payload=qualifier_payload,
        previous_public=previous_public,
        previous_private=previous_private,
        held_velocity=list(held_velocity),
        applied_command_frame=applied_command_frame,
        history_version=history_version,
        mission_horizon_frames=500,
        active_reference_count=2,
        base_anchor_provenance=fixture["measurements"][
            "base_anchor_provenance"
        ],
    )


def fresh_registered_qualified_row():
    return build_registered_qualified_row(
        frame_index=0,
        applied_command_frame=None,
        history_version=0,
        qualifier_kind="deployment",
        qualifier_payload={"domain": registered_deployment_domain()},
    )


def aged_registered_qualified_row():
    fresh = fresh_registered_qualified_row()
    lifecycle = fresh["audit_bundle"]["lifecycle"]
    return build_registered_qualified_row(
        frame_index=1,
        applied_command_frame=0,
        history_version=2,
        previous_public=lifecycle["public_output"],
        previous_private=lifecycle["next_private_state"],
    )


def rewrite_nonempty_provenance(value, replacement):
    rewritten = 0
    if isinstance(value, dict):
        for key, nested in value.items():
            if key == "base_anchor_provenance" and nested:
                value[key] = list(replacement)
                rewritten += 1
            else:
                rewritten += rewrite_nonempty_provenance(
                    nested,
                    replacement,
                )
    elif isinstance(value, list):
        for nested in value:
            rewritten += rewrite_nonempty_provenance(nested, replacement)
    return rewritten


def nested_lists(depth):
    value = "leaf"
    for _ in range(depth):
        value = [value]
    return value


class QualifiedReplayTests(unittest.TestCase):
    def build_registered_row(self):
        return build_registered_qualified_row()

    def test_two_circle_ambiguity_serializes_both_modes(self):
        row = self.build_registered_row()

        self.assertIn("qualified-mode-hybrid-dcbf", RAW_SCHEMA_ID)
        self.assertEqual(row["mode_count"], 2)
        representatives = row["audit_bundle"]["representatives"]
        self.assertEqual(len(representatives), 2)
        self.assertAlmostEqual(
            math.dist(
                representatives[0]["estimate"],
                representatives[1]["estimate"],
            ),
            119.7962964736439,
            places=6,
        )
        self.assertNotEqual(row["public_status"], "fresh")

    def test_row_serializes_runtime_inputs_attempts_and_sensitivity(self):
        row = self.build_registered_row()

        self.assertEqual(
            [
                reference["key"]
                for reference in row.get("runtime_inputs", {}).get(
                    "references",
                    (),
                )
            ],
            [["uav", 10], ["uav", 11]],
        )
        self.assertEqual(len(row["audit_bundle"]["starts"]), 2)
        self.assertEqual(len(row["audit_bundle"]["solver_attempts"]), 2)
        self.assertEqual(row["local_candidate_count"], 2)
        self.assertEqual(row["sensitivity_count_0_5_mm"], 2)
        self.assertEqual(row["sensitivity_count_1_mm"], 2)
        self.assertEqual(row["sensitivity_count_2_mm"], 2)
        self.assertIsNone(row["public_age"])
        self.assertIsNone(row["private_age"])

    def test_fabricated_solver_geometry_is_retained_but_not_eligible(self):
        fabricated = {
            "status": "converged",
            "estimate": [0.0, 0.0],
            "covariance": [[1.0, 0.0], [0.0, 1.0]],
            "epsilon": 3.0,
            "phi_min_eigenvalue": 1.0,
            "phi_condition": 1.0,
            "fim_valid": True,
            "proposal_count": 0,
            "iterations": 0,
            "cost": 0.0,
            "stationarity_norm": 0.0,
            "failure_reason": None,
            "proposal_trace": [],
        }

        row = build_registered_qualified_row(
            solver_from_start=lambda start: dict(fabricated),
        )

        attempts = row["audit_bundle"]["solver_attempts"]
        self.assertEqual(len(attempts), 2)
        self.assertEqual(row["local_candidate_count"], 0)
        self.assertEqual(row["mode_count"], 0)
        self.assertTrue(all(
            not attempt["local_eligible"]
            and attempt["local_eligibility_reason"]
            == "solver_geometry_mismatch"
            for attempt in attempts
        ))

    def test_recursive_provenance_must_be_derived_from_references(self):
        fixture, references, solver = registered_two_circle_inputs()

        with self.assertRaisesRegex(ValueError, "base anchor provenance"):
            build_qualified_replay_row(
                frame_index=0,
                robot_id=fixture["key"]["robot_id"],
                squad_local_index=fixture["key"]["squad_local_index"],
                schedule_id=(
                    f"frame-0:robot-{fixture['key']['robot_id']}:"
                    f"squad-local-{fixture['key']['squad_local_index']}"
                ),
                references=references,
                solver_from_start=solver,
                live_seed=None,
                private_seed=None,
                qualifier_kind="deployment",
                qualifier_payload={
                    "domain": registered_deployment_domain(),
                },
                previous_public=None,
                previous_private=None,
                held_velocity=[0.0, 0.0],
                applied_command_frame=None,
                history_version=0,
                mission_horizon_frames=500,
                active_reference_count=2,
                base_anchor_provenance=[100, 101],
            )

    def test_deployment_qualifier_publishes_only_the_ocean_mode(self):
        row = build_registered_qualified_row(
            frame_index=0,
            applied_command_frame=None,
            history_version=0,
            qualifier_kind="deployment",
            qualifier_payload={"domain": registered_deployment_domain()},
        )

        self.assertEqual(row["admissible_mode_count"], 1)
        self.assertEqual(row["public_status"], "fresh")
        self.assertEqual(row["public_age"], 0)
        self.assertEqual(row["private_age"], 0)
        self.assertIsNotNone(row["published_mode_id"])
        selected = next(
            representative
            for representative in row["audit_bundle"]["representatives"]
            if representative["attempt_id"]
            == row["audit_bundle"]["decision"][
                "representative_attempt_id"
            ]
        )
        self.assertGreater(selected["estimate"][0], -1550.0)

    def test_row_is_strict_json_and_producer_validator_rejects_unknown_fields(self):
        row = self.build_registered_row()

        encoded = json.dumps(row, allow_nan=False, separators=(",", ":"))
        self.assertIn(RAW_SCHEMA_ID, encoded)
        malformed = dict(row)
        malformed["unknown"] = True
        with self.assertRaisesRegex(ValueError, "exact field order"):
            validate_qualified_replay_row(malformed)
        nested = copy.deepcopy(row)
        nested["audit_bundle"]["solver_attempts"][0]["solver_result"][
            "unknown"
        ] = True
        with self.assertRaisesRegex(ValueError, "solver result evidence"):
            validate_qualified_replay_row(nested)
        boolean_diameter = copy.deepcopy(row)
        boolean_diameter["audit_bundle"]["clustering"]["modes"][0][
            "diameter_m"
        ] = False
        with self.assertRaisesRegex(ValueError, "cluster diameter"):
            validate_qualified_replay_row(boolean_diameter)
        false_provenance = copy.deepcopy(row)
        false_provenance["runtime_inputs"]["references"][0][
            "base_anchor_provenance"
        ] = [100, 101]
        with self.assertRaisesRegex(ValueError, "base anchor provenance"):
            validate_qualified_replay_row(false_provenance)

    def test_producer_rejects_recursive_current_provenance_tampering(self):
        cases = (
            (
                "local candidate",
                lambda row: row["audit_bundle"]["local_candidates"][0][
                    "payload"
                ].update(base_anchor_provenance=[100, 101]),
            ),
            (
                "representative",
                lambda row: row["audit_bundle"]["representatives"][0][
                    "payload"
                ].update(base_anchor_provenance=[100, 101]),
            ),
        )
        for name, mutation in cases:
            with self.subTest(name=name):
                row = self.build_registered_row()
                mutation(row)
                with self.assertRaisesRegex(
                    ValueError,
                    "base anchor provenance",
                ):
                    validate_qualified_replay_row(row)

        fresh = build_registered_qualified_row(
            frame_index=0,
            applied_command_frame=None,
            history_version=0,
            qualifier_kind="deployment",
            qualifier_payload={"domain": registered_deployment_domain()},
        )
        fresh["audit_bundle"]["lifecycle"]["public_output"][
            "base_anchor_provenance"
        ] = [100, 101]
        with self.assertRaisesRegex(ValueError, "base anchor provenance"):
            validate_qualified_replay_row(fresh)

    def test_producer_rejects_boolean_numeric_nested_state_evidence(self):
        fresh = build_registered_qualified_row(
            frame_index=0,
            applied_command_frame=None,
            history_version=0,
            qualifier_kind="deployment",
            qualifier_payload={"domain": registered_deployment_domain()},
        )
        fresh["audit_bundle"]["lifecycle"]["public_output"][
            "epsilon"
        ] = False
        with self.assertRaisesRegex(ValueError, "public state"):
            validate_qualified_replay_row(fresh)

        fresh = build_registered_qualified_row(
            frame_index=0,
            applied_command_frame=None,
            history_version=0,
            qualifier_kind="deployment",
            qualifier_payload={"domain": registered_deployment_domain()},
        )
        lifecycle = fresh["audit_bundle"]["lifecycle"]
        aged = build_registered_qualified_row(
            frame_index=1,
            applied_command_frame=0,
            history_version=2,
            previous_public=lifecycle["public_output"],
            previous_private=lifecycle["next_private_state"],
        )
        aged["audit_bundle"]["transition_inputs"]["previous_private"][
            "age_frames"
        ] = False
        with self.assertRaisesRegex(ValueError, "private state"):
            validate_qualified_replay_row(aged)

    def test_producer_rejects_boolean_numeric_nested_audit_fields(self):
        cases = (
            (
                "runtime covariance",
                lambda row: row["audit_bundle"]["runtime_inputs"][
                    "references"
                ][0]["covariance"][0].__setitem__(0, False),
            ),
            (
                "start estimate",
                lambda row: row["audit_bundle"]["starts"][0][
                    "estimate"
                ].__setitem__(0, False),
            ),
            (
                "local diagnostic",
                lambda row: row["audit_bundle"]["solver_attempts"][0][
                    "local_diagnostics"
                ].update(recomputed_objective=False),
            ),
            (
                "transition frame",
                lambda row: row["audit_bundle"]["transition_inputs"].update(
                    frame_index=False
                ),
            ),
            (
                "prior history version",
                lambda row: row["audit_bundle"]["prior_bundle"].update(
                    history_version=False
                ),
            ),
            (
                "lifecycle history version",
                lambda row: row["audit_bundle"]["lifecycle"].update(
                    history_version=False
                ),
            ),
        )
        for name, mutation in cases:
            with self.subTest(name=name):
                row = self.build_registered_row()
                mutation(row)
                with self.assertRaises(ValueError):
                    validate_qualified_replay_row(row)

        deployment = build_registered_qualified_row(
            frame_index=0,
            applied_command_frame=None,
            history_version=0,
            qualifier_kind="deployment",
            qualifier_payload={"domain": registered_deployment_domain()},
        )
        deployment["audit_bundle"]["qualifier_context"][
            "qualifier_payload"
        ]["domain"]["margin_m"] = False
        with self.assertRaisesRegex(ValueError, "qualifier payload"):
            validate_qualified_replay_row(deployment)

        history = build_registered_qualified_row(
            qualifier_kind="history",
            qualifier_payload={"innovation_limit": 11.829007011943707},
        )
        history["audit_bundle"]["qualifier_context"]["qualifier_payload"][
            "innovation_limit"
        ] = False
        with self.assertRaisesRegex(ValueError, "qualifier payload"):
            validate_qualified_replay_row(history)

    def test_producer_rejects_boolean_lifecycle_numeric_coercion(self):
        cases = (
            (
                "fresh public estimate",
                fresh_registered_qualified_row,
                lambda row: row["audit_bundle"]["lifecycle"][
                    "public_output"
                ]["estimate"].__setitem__(0, False),
            ),
            (
                "fresh public covariance",
                fresh_registered_qualified_row,
                lambda row: row["audit_bundle"]["lifecycle"][
                    "public_output"
                ].update(
                    modeled_covariance=[[True, False], [False, True]]
                ),
            ),
            (
                "previous public estimate",
                aged_registered_qualified_row,
                lambda row: row["audit_bundle"]["transition_inputs"][
                    "previous_public"
                ]["estimate"].__setitem__(0, False),
            ),
            (
                "previous private estimate",
                aged_registered_qualified_row,
                lambda row: row["audit_bundle"]["transition_inputs"][
                    "previous_private"
                ]["estimate"].__setitem__(0, False),
            ),
            (
                "previous private covariance",
                aged_registered_qualified_row,
                lambda row: row["audit_bundle"]["transition_inputs"][
                    "previous_private"
                ].update(
                    modeled_covariance=[[True, False], [False, True]]
                ),
            ),
            (
                "transition held velocity",
                aged_registered_qualified_row,
                lambda row: row["audit_bundle"]["transition_inputs"].update(
                    held_velocity=[False, 0.0]
                ),
            ),
        )
        for name, factory, mutation in cases:
            with self.subTest(name=name):
                row = factory()
                mutation(row)
                with self.assertRaises(ValueError):
                    validate_qualified_replay_row(row)

    def test_producer_rejects_mission_inconsistent_provenance_rewrite(self):
        row = self.build_registered_row()
        rewritten = rewrite_nonempty_provenance(row, [100, 101])
        self.assertGreaterEqual(rewritten, 10)
        with self.assertRaisesRegex(ValueError, "mission base authority"):
            validate_qualified_replay_row(row)

        aged = aged_registered_qualified_row()
        aged["audit_bundle"]["transition_inputs"]["previous_public"][
            "base_anchor_provenance"
        ] = [100, 101]
        with self.assertRaisesRegex(ValueError, "mission base authority"):
            validate_qualified_replay_row(aged)

        deployment = fresh_registered_qualified_row()
        deployment["audit_bundle"]["qualifier_context"][
            "qualifier_payload"
        ]["domain"]["anchor_ids"] = [0, 1]
        with self.assertRaisesRegex(ValueError, "deployment anchor authority"):
            validate_qualified_replay_row(deployment)

    def test_producer_bounds_forbidden_field_traversal(self):
        row = self.build_registered_row()
        row["audit_bundle"]["qualifier_context"]["qualifier_payload"][
            "reason"
        ] = nested_lists(1500)
        with self.assertRaisesRegex(ValueError, "bounded"):
            validate_qualified_replay_row(row)

        row = self.build_registered_row()
        row["audit_bundle"]["qualifier_context"]["qualifier_payload"][
            "reason"
        ] = [0.0] * 5000
        with self.assertRaisesRegex(ValueError, "bounded"):
            validate_qualified_replay_row(row)

    def test_producer_validator_rejects_every_malformed_solver_evidence_case(self):
        for name, malformed_result in malformed_serialized_solver_evidence_cases():
            with self.subTest(name=name):
                row = self.build_registered_row()
                row["audit_bundle"]["solver_attempts"][0][
                    "solver_result"
                ] = malformed_result
                with self.assertRaisesRegex(ValueError, "solver result evidence"):
                    validate_qualified_replay_row(row)

    def test_producer_validator_rejects_reordered_protocol_mappings(self):
        row = self.build_registered_row()
        reversed_row = dict(reversed(tuple(row.items())))
        with self.assertRaisesRegex(ValueError, "exact field order"):
            validate_qualified_replay_row(reversed_row)

        row = self.build_registered_row()
        row["audit_bundle"] = dict(reversed(tuple(
            row["audit_bundle"].items()
        )))
        with self.assertRaisesRegex(ValueError, "audit bundle.*exact schema"):
            validate_qualified_replay_row(row)

        row = self.build_registered_row()
        result = row["audit_bundle"]["solver_attempts"][0]["solver_result"]
        row["audit_bundle"]["solver_attempts"][0]["solver_result"] = dict(
            reversed(tuple(result.items()))
        )
        with self.assertRaisesRegex(ValueError, "solver result evidence.*order"):
            validate_qualified_replay_row(row)

    def test_producer_rejects_dealiased_reordered_top_projections(self):
        runtime_row = json.loads(json.dumps(self.build_registered_row()))
        runtime_row["runtime_inputs"] = dict(reversed(tuple(
            runtime_row["runtime_inputs"].items()
        )))
        with self.assertRaisesRegex(ValueError, "runtime input projection"):
            validate_qualified_replay_row(runtime_row)

        bool_covariance = json.loads(json.dumps(self.build_registered_row()))
        bool_covariance["runtime_inputs"]["references"][0]["covariance"][0][1] = False
        with self.assertRaisesRegex(ValueError, "runtime input projection"):
            validate_qualified_replay_row(bool_covariance)

        bool_provenance = json.loads(json.dumps(self.build_registered_row()))
        bool_provenance["runtime_inputs"]["base_anchor_provenance"] = [
            False,
            True,
            2,
        ]
        with self.assertRaisesRegex(ValueError, "runtime input projection"):
            validate_qualified_replay_row(bool_provenance)

        qualification_row = json.loads(json.dumps(self.build_registered_row()))
        qualification = qualification_row["qualifications"][0]
        qualification_row["qualifications"][0] = dict(reversed(tuple(
            qualification.items()
        )))
        with self.assertRaisesRegex(ValueError, "qualification record"):
            validate_qualified_replay_row(qualification_row)


if __name__ == "__main__":
    unittest.main()
