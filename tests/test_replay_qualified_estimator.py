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
        with self.assertRaisesRegex(ValueError, "solver result.*exact schema"):
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


if __name__ == "__main__":
    unittest.main()
