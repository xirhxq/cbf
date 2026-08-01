import copy
import unittest

from scripts.diagnostics.analyze_qualified_estimator import (
    validate_and_recompute_qualified_row,
)
import tests.test_replay_qualified_estimator as replay_test


def valid_registered_row():
    return replay_test.build_registered_qualified_row()


def fresh_registered_row():
    return replay_test.build_registered_qualified_row(
        frame_index=0,
        applied_command_frame=None,
        history_version=0,
        qualifier_kind="deployment",
        qualifier_payload={
            "domain": replay_test.registered_deployment_domain(),
        },
    )


def aged_private_row():
    fresh = fresh_registered_row()
    lifecycle = fresh["audit_bundle"]["lifecycle"]
    return replay_test.build_registered_qualified_row(
        frame_index=1,
        applied_command_frame=0,
        history_version=2,
        previous_public=lifecycle["public_output"],
        previous_private=lifecycle["next_private_state"],
    )


def history_qualified_row():
    fresh = fresh_registered_row()
    lifecycle = fresh["audit_bundle"]["lifecycle"]
    return replay_test.build_registered_qualified_row(
        frame_index=1,
        applied_command_frame=0,
        history_version=2,
        qualifier_kind="history",
        qualifier_payload={
            "innovation_limit": 11.829007011943707,
        },
        previous_public=lifecycle["public_output"],
        previous_private=lifecycle["next_private_state"],
    )


class QualifiedAnalyzerTests(unittest.TestCase):
    def test_independent_analyzer_reconstructs_registered_row(self):
        result = validate_and_recompute_qualified_row(valid_registered_row())

        self.assertTrue(result.get("valid", False))
        self.assertEqual(result.get("mode_count"), 2)
        self.assertEqual(result.get("admissible_mode_count"), 0)
        self.assertEqual(result.get("public_status"), "unavailable")

    def test_independent_analyzer_reconstructs_history_qualified_publication(self):
        result = validate_and_recompute_qualified_row(history_qualified_row())

        self.assertTrue(result["valid"])
        self.assertEqual(result["mode_count"], 2)
        self.assertEqual(result["admissible_mode_count"], 1)
        self.assertIsNotNone(result["published_mode_id"])
        self.assertEqual(result["public_status"], "fresh")

    def assert_tamper_rejected(self, mutation, message, *, row=None):
        tampered = copy.deepcopy(valid_registered_row() if row is None else row)
        mutation(tampered)
        with self.assertRaisesRegex(ValueError, message):
            validate_and_recompute_qualified_row(tampered)

    def test_reference_and_start_order_tampering_is_rejected(self):
        self.assert_tamper_rejected(
            lambda row: row["runtime_inputs"]["references"].reverse(),
            "reference order inconsistent with stable order",
        )
        self.assert_tamper_rejected(
            lambda row: row["audit_bundle"]["starts"].reverse(),
            "stable start order mismatch",
        )
        self.assert_tamper_rejected(
            lambda row: row["runtime_inputs"]["references"][0].update(
                unknown=True
            ),
            "runtime reference schema mismatch",
        )
        self.assert_tamper_rejected(
            lambda row: row["runtime_inputs"].update(
                base_anchor_provenance=[100, 101]
            ),
            "base anchor provenance mismatch",
        )
        self.assert_tamper_rejected(
            lambda row: row["runtime_inputs"]["references"][0].update(
                base_anchor_provenance=[100, 101]
            ),
            "base anchor provenance mismatch",
        )

    def test_negative_schedule_index_is_rejected_even_when_identity_matches(self):
        def make_negative(row):
            row["robot_id"] = -7
            row["schedule_id"] = (
                f"frame-{row['frame_index']}:robot--7:"
                f"squad-local-{row['squad_local_index']}"
            )

        self.assert_tamper_rejected(
            make_negative,
            "qualified schedule index mismatch",
        )

    def test_boolean_substitutions_for_numeric_derived_fields_are_rejected(self):
        fresh = fresh_registered_row()
        cases = (
            (lambda row: row.update(public_age=False), "public_age"),
            (lambda row: row.update(private_age=False), "private_age"),
            (lambda row: row.update(history_version=True), "history_version"),
            (lambda row: row.update(mode_count=True), "mode_count"),
            (
                lambda row: row["audit_bundle"]["clustering"]["modes"][0]
                .update(diameter_m=False),
                "cluster diameter",
            ),
        )
        for mutation, message in cases:
            with self.subTest(message=message):
                self.assert_tamper_rejected(
                    mutation,
                    message,
                    row=fresh,
                )

    def test_analyzer_recomputes_fabricated_solver_geometry(self):
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
        row = replay_test.build_registered_qualified_row(
            solver_from_start=lambda start: dict(fabricated),
        )

        result = validate_and_recompute_qualified_row(row)
        self.assertTrue(result["valid"])
        self.assertEqual(result["mode_count"], 0)

    def test_solver_attempt_or_locally_eligible_mode_omission_is_rejected(self):
        self.assert_tamper_rejected(
            lambda row: row["audit_bundle"]["solver_attempts"].pop(),
            "solver attempt completeness mismatch",
        )
        self.assert_tamper_rejected(
            lambda row: row["audit_bundle"]["local_candidates"].pop(),
            "local candidate completeness mismatch",
        )

    def test_cluster_identity_membership_and_diameter_tampering_is_rejected(self):
        self.assert_tamper_rejected(
            lambda row: row["audit_bundle"]["clustering"]["modes"][0].update(
                mode_id="0" * 64
            ),
            "mode ID mismatch",
        )
        self.assert_tamper_rejected(
            lambda row: row["audit_bundle"]["clustering"]["modes"][0][
                "member_ids"
            ].clear(),
            "mode membership mismatch",
        )
        self.assert_tamper_rejected(
            lambda row: row["audit_bundle"]["clustering"]["modes"][0].update(
                diameter_m=0.25
            ),
            "cluster diameter mismatch",
        )

    def test_qualification_and_admissible_count_tampering_is_rejected(self):
        self.assert_tamper_rejected(
            lambda row: row["audit_bundle"]["qualifications"][0].update(
                admissible=True
            ),
            "audit qualification mismatch",
        )
        self.assert_tamper_rejected(
            lambda row: row.update(admissible_mode_count=1),
            "admissible mode count mismatch",
        )

    def test_primary_and_each_sensitivity_count_tampering_is_rejected(self):
        self.assert_tamper_rejected(
            lambda row: row.update(primary_tolerance_m=0.002),
            "primary tolerance mismatch",
        )
        cases = (
            ("sensitivity_count_0_5_mm", "0.5 mm"),
            ("sensitivity_count_1_mm", "1 mm"),
            ("sensitivity_count_2_mm", "2 mm"),
        )
        for field, label in cases:
            with self.subTest(field=field):
                self.assert_tamper_rejected(
                    lambda row, field=field: row.update(
                        {field: row[field] + 1}
                    ),
                    f"sensitivity {label} count mismatch",
                )

    def test_predecision_prior_and_transition_tampering_is_rejected(self):
        self.assert_tamper_rejected(
            lambda row: row["audit_bundle"]["prior_bundle"].update(
                history_version=999
            ),
            "pre-decision prior bundle mismatch",
        )
        aged = aged_private_row()
        self.assert_tamper_rejected(
            lambda row: row["audit_bundle"]["transition_inputs"].update(
                held_velocity=[1.0, 0.0]
            ),
            "pre-decision prior bundle mismatch",
            row=aged,
        )
        self.assert_tamper_rejected(
            lambda row: row["audit_bundle"]["transition_inputs"][
                "previous_private"
            ].update(age_frames=1),
            "previous private state schema mismatch",
            row=aged,
        )
        self.assert_tamper_rejected(
            lambda row: row["audit_bundle"]["transition_inputs"].update(
                history_version=3
            ),
            "pre-decision prior bundle mismatch",
            row=aged,
        )

    def test_publication_status_private_age_and_lifecycle_tampering_is_rejected(self):
        fresh = fresh_registered_row()
        self.assert_tamper_rejected(
            lambda row: row.update(published_mode_id="0" * 64),
            "published mode ID mismatch",
            row=fresh,
        )
        self.assert_tamper_rejected(
            lambda row: row.update(public_status="predicted"),
            "public status mismatch",
            row=fresh,
        )
        self.assert_tamper_rejected(
            lambda row: row["audit_bundle"]["lifecycle"][
                "public_output"
            ].update(reason="tampered"),
            "qualified lifecycle mismatch",
            row=fresh,
        )
        aged = aged_private_row()
        self.assert_tamper_rejected(
            lambda row: row.update(private_age=99),
            "private age mismatch",
            row=aged,
        )
        self.assert_tamper_rejected(
            lambda row: row.update(history_version=99),
            "lifecycle history version mismatch",
            row=aged,
        )

    def test_forbidden_runtime_and_old_v2_protocol_substitution_are_rejected(self):
        self.assert_tamper_rejected(
            lambda row: row["audit_bundle"]["qualifier_context"][
                "qualifier_payload"
            ].update(nested={"truth_position": [0.0, 0.0]}),
            "forbidden qualified runtime field",
        )
        self.assert_tamper_rejected(
            lambda row: row.update(
                schema_id="cbf2026-two-range-reacquisition-raw-v2"
            ),
            "schema identity mismatch",
        )
        self.assert_tamper_rejected(
            lambda row: row.update(method_id="two-range-reacquisition-v2"),
            "method identity mismatch",
        )

    def test_analyzer_rejects_every_malformed_solver_evidence_case(self):
        for name, malformed_result in (
            replay_test.malformed_serialized_solver_evidence_cases()
        ):
            with self.subTest(name=name):
                row = valid_registered_row()
                row["audit_bundle"]["solver_attempts"][0][
                    "solver_result"
                ] = malformed_result
                with self.assertRaisesRegex(ValueError, "solver result evidence"):
                    validate_and_recompute_qualified_row(row)

    def test_analyzer_rejects_reordered_protocol_mappings(self):
        row = valid_registered_row()
        with self.assertRaisesRegex(ValueError, "qualified row.*exact schema"):
            validate_and_recompute_qualified_row(
                dict(reversed(tuple(row.items())))
            )

        row = valid_registered_row()
        row["audit_bundle"] = dict(reversed(tuple(
            row["audit_bundle"].items()
        )))
        with self.assertRaisesRegex(ValueError, "qualified audit.*exact schema"):
            validate_and_recompute_qualified_row(row)

        row = valid_registered_row()
        result = row["audit_bundle"]["solver_attempts"][0]["solver_result"]
        row["audit_bundle"]["solver_attempts"][0]["solver_result"] = dict(
            reversed(tuple(result.items()))
        )
        with self.assertRaisesRegex(ValueError, "solver result evidence.*order"):
            validate_and_recompute_qualified_row(row)

    def test_analyzer_reconstructs_only_the_exact_invalid_evidence_sentinel(self):
        row = replay_test.build_registered_qualified_row(
            solver_from_start=lambda start: {},
        )

        result = validate_and_recompute_qualified_row(row)
        self.assertTrue(result["valid"])
        self.assertEqual(result["mode_count"], 0)
        self.assertTrue(all(
            attempt["local_eligibility_reason"]
            == "invalid_solver_result_evidence"
            for attempt in row["audit_bundle"]["solver_attempts"]
        ))

        tampered = copy.deepcopy(row)
        tampered["audit_bundle"]["solver_attempts"][0]["solver_result"][
            "estimate"
        ] = [0.0, 0.0]
        with self.assertRaisesRegex(ValueError, "local eligibility outcome"):
            validate_and_recompute_qualified_row(tampered)


if __name__ == "__main__":
    unittest.main()
