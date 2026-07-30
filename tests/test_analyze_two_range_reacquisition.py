"""Contracts for the independent two-range reacquisition analyzer."""

from __future__ import annotations

import copy
import gzip
import hashlib
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from scripts.diagnostics import analyze_two_range_reacquisition as analyzer
from scripts.diagnostics import replay_two_range_reacquisition as replay


FIXTURE_ROOT = Path("tests/fixtures/cbf2026_two_range_reacquisition")


def valid_smoke_row(case_id: str = "select_positive") -> dict:
    fixture = json.loads(
        (FIXTURE_ROOT / "mechanism_20260727_180_12.json").read_bytes()
    )
    return replay.produce_smoke_row(
        case_id=case_id,
        mechanism_fixture=fixture,
    )


def mechanism_fixture() -> dict:
    return json.loads(
        (FIXTURE_ROOT / "mechanism_20260727_180_12.json").read_bytes()
    )


def mechanism_public(fixture: dict) -> dict[int, dict]:
    return {
        record["reference_key"][1]: replay._internal_public(
            record["public_output"]
        )
        for record in fixture["current_reference_outputs"]
    }


def fixed_topology_config() -> dict:
    return {
        "num": 14,
        "formation": {
            "parts": 2,
            "bases-id": [[1, 2, 3], [1, 2, 3]],
        },
        "cbfs": {
            "without-slack": {
                "comm-fixed": {
                    "min-neighbour-id-offset": -2,
                    "max-neighbour-id-offset": 0,
                }
            }
        },
    }


def registered_new_row() -> dict:
    row = valid_smoke_row("mechanism_20260727_180_12")
    row.update(
        {
            "invocation_name": "registered_replay",
            "smoke_case_id": None,
            "smoke_case_kind": None,
            "smoke_case_input": None,
            "smoke_case_expected": None,
        }
    )
    replay._validate_row(row)
    return row


def comparison_rows(
    *,
    baseline_fresh=True,
    baseline_error=1.0,
    new_error=None,
):
    new = registered_new_row()
    if new_error is not None:
        new["offline_error_norm"] = new_error
    key = (new["seed"], new["frame_index"], new["robot_id"])
    baseline = {
        "graph_case": "dynamic_dag_wnls",
        "seed": key[0],
        "frame_index": key[1],
        "robot_id": key[2],
        "attempt_status": "converged" if baseline_fresh else "failed",
        "status": "converged" if baseline_fresh else "stale",
        "error_norm": baseline_error,
        "finite": True,
    }
    v4 = {
        "variant": "predictive_multistart",
        "seed": key[0],
        "frame_index": key[1],
        "robot_id": key[2],
        "squad_local_index": new["squad_local_index"],
        "attempt_status": "accepted",
        "output_status": "fresh",
        "prediction_age": 0,
        "offline_error_norm": baseline_error,
        "offline_fresh_q_error": baseline_error * baseline_error,
    }
    return [baseline], [v4], [new]


def compact_identity(path, *, seed=1, domain="file_bytes"):
    return {
        "path": path,
        "device": 1,
        "inode": seed,
        "size": 100,
        "mtime_ns": seed,
        "sha256": f"{seed:064x}",
        "hash_domain": domain,
    }


def canonical_registered_result(result):
    paths = {
        "protocol": (
            "/tmp/docs/diagnostics/"
            "2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json"
        ),
        "authorization": (
            "/tmp/docs/diagnostics/reviews/"
            "2026-07-30-cbf2026-two-range-reacquisition-"
            "registered-authorization.json"
        ),
        "raw_manifest": "/tmp/raw/manifest.json",
        "raw_compressed_process": (
            f"/tmp/raw/{replay.RAW_PROCESS_NAME}"
        ),
        "raw_decompressed_process": (
            f"/tmp/raw/{replay.RAW_PROCESS_NAME}"
        ),
        "v4_manifest": "/tmp/v4/manifest.json",
        "v4_compressed_process": (
            "/tmp/v4/predictive-wnls-development.jsonl.gz"
        ),
        "v4_decompressed_process": (
            "/tmp/v4/predictive-wnls-development.jsonl.gz"
        ),
        "v4_analysis_manifest": "/tmp/v4-analysis/manifest.json",
        "v4_analysis_json": (
            "/tmp/v4-analysis/predictive-wnls-development.json"
        ),
        "v4_analysis_markdown": (
            "/tmp/v4-analysis/predictive-wnls-development.md"
        ),
        "legacy_baseline_process": "/tmp/legacy/calibration.jsonl.gz",
        "legacy_baseline_protocol_json": (
            "/tmp/docs/diagnostics/"
            "2026-07-30-predictive-wnls-stage1-protocol-v4.json"
        ),
        "truth_data": "/tmp/truth/data.json",
    }
    identities = {}
    for index, name in enumerate(analyzer.IDENTITY_FIELDS, 1):
        if name in {"mechanism_fixture", "synthetic_case_source"}:
            identities[name] = None
            continue
        pair_seed = {
            "raw_decompressed_process": analyzer.IDENTITY_FIELDS.index(
                "raw_compressed_process"
            )
            + 1,
            "v4_decompressed_process": analyzer.IDENTITY_FIELDS.index(
                "v4_compressed_process"
            )
            + 1,
        }.get(name, index)
        identities[name] = compact_identity(
            paths[name],
            seed=pair_seed,
            domain=(
                "decompressed_jsonl_bytes"
                if name.endswith("decompressed_process")
                else "file_bytes"
            ),
        )
    result["identities"] = identities
    result["budgets"].update(
        {
            "expected_rows": 140000,
            "observed_rows": 140000,
            "unique_rows": 140000,
        }
    )
    result["status_counts"] = {
        "attempt_accepted": 140000,
        "attempt_rejected": 0,
        "attempt_failed": 0,
        "attempt_invalid": 0,
        "attempt_reference_unavailable": 0,
        "output_fresh": 140000,
        "output_predicted": 0,
        "output_unavailable": 0,
    }
    result["decision"] = (
        "pass"
        if all(
            record["passed"]
            for record in (
                *result["scientific_gates"],
                *result["integrity_gates"],
            )
        )
        else "fail"
    )
    result["semantic_payload_sha256"] = analyzer._semantic_sha256(result)
    return result


def validate_row(row: dict, *, expected_key=None, **overrides):
    if expected_key is None:
        expected_key = (
            row["method"],
            row["seed"],
            row["frame_index"],
            row["robot_id"],
        )
    arguments = {
        "expected_key": expected_key,
        "protocol": (
            {"truth_config": fixed_topology_config()}
            if row["frame_index"] is not None
            else {}
        ),
        "truth_position": row["offline_truth_position"],
        "current_public": {},
        "previous_private": None,
        "held_command": row["applied_command"],
    }
    arguments.update(overrides)
    return analyzer.validate_and_reconstruct_row(row, **arguments)


class ExactSchemaTests(unittest.TestCase):
    def test_strict_json_rejects_nonfinite_constants(self):
        for constant in (b"NaN", b"Infinity", b"-Infinity"):
            with self.subTest(constant=constant):
                with self.assertRaisesRegex(ValueError, "invalid strict JSON"):
                    analyzer._strict_json_object(
                        b'{"value":' + constant + b"}",
                        Path("/tmp/nonfinite.json"),
                    )
        with self.assertRaisesRegex(ValueError, "invalid strict JSON"):
            analyzer._strict_json_object(
                b'{"value":1e400}',
                Path("/tmp/overflow.json"),
            )

    def test_missing_field_rejects(self):
        row = valid_smoke_row()
        row.pop("offline_fresh_q_error")
        with self.assertRaises(ValueError):
            validate_row(row)

    def test_wrong_method_rejects(self):
        row = valid_smoke_row("q_equal_threshold")
        row["method"] = "forged_method"
        with self.assertRaises(ValueError):
            validate_row(row)

    def test_extra_field_rejects(self):
        row = valid_smoke_row("q_equal_threshold")
        row["extra"] = None
        with self.assertRaises(ValueError):
            validate_row(row)

    def test_reordered_serialized_fields_rejects(self):
        original = valid_smoke_row("q_equal_threshold")
        fields = list(original)
        fields[0], fields[1] = fields[1], fields[0]
        row = {field: original[field] for field in fields}
        with self.assertRaises(ValueError):
            validate_row(row)

    def test_noncanonical_branch_order_rejects(self):
        row = valid_smoke_row()
        branch = row["branches"][0]
        fields = list(branch)
        fields[0], fields[1] = fields[1], fields[0]
        row["branches"][0] = {field: branch[field] for field in fields}
        with self.assertRaises(ValueError):
            validate_row(row)

    def test_seed_outside_grid_rejects(self):
        row = valid_smoke_row("mechanism_20260727_180_12")
        row["seed"] = 20260726
        with self.assertRaises(ValueError):
            validate_row(row)

    def test_frame_outside_grid_rejects(self):
        row = valid_smoke_row("mechanism_20260727_180_12")
        row["frame_index"] = 500
        with self.assertRaises(ValueError):
            validate_row(row)

    def test_robot_outside_grid_rejects(self):
        row = valid_smoke_row("mechanism_20260727_180_12")
        row["robot_id"] = 15
        with self.assertRaises(ValueError):
            validate_row(row)

    def test_expected_stream_key_mismatch_rejects(self):
        row = valid_smoke_row("q_equal_threshold")
        with self.assertRaises(ValueError):
            validate_row(
                row,
                expected_key=(row["method"], None, None, 1),
            )


class AnalysisSchemaDeclarationTests(unittest.TestCase):
    def test_top_level_field_order_is_frozen(self):
        self.assertEqual(
            analyzer.ANALYSIS_FIELDS,
            (
                "schema_id",
                "protocol_id",
                "invocation_name",
                "decision",
                "semantic_payload_sha256",
                "identities",
                "budgets",
                "status_counts",
                "selector_accounting",
                "baseline_fresh_transitions",
                "v4_descriptive_comparison",
                "paired_comparison",
                "scientific_gates",
                "integrity_gates",
                "tails",
                "limitations",
            ),
        )


class PrivateStateReconstructionTests(unittest.TestCase):
    def setUp(self):
        self.fixture = mechanism_fixture()
        self.row = replay.produce_smoke_row(
            case_id="mechanism_20260727_180_12",
            mechanism_fixture=self.fixture,
        )

    def assert_private_tamper_rejects(self, field, value):
        row = copy.deepcopy(self.row)
        row[field] = value
        with self.assertRaises(ValueError):
            validate_row(
                row,
                current_public=mechanism_public(self.fixture),
                previous_private=self.fixture["preceding_private_state"],
                held_command=self.fixture["held_command"],
            )

    def test_valid_one_frame_private_state_recursion_reconstructs(self):
        reconstructed = validate_row(
            self.row,
            current_public=mechanism_public(self.fixture),
            previous_private=self.fixture["preceding_private_state"],
            held_command=self.fixture["held_command"],
        )
        self.assertEqual(
            reconstructed["private_state"]["source_fresh_frame"],
            self.row["next_private_state_source_fresh_frame"],
        )
        self.assertEqual(reconstructed["private_state"]["age_frames"], 0)

    def test_prior_estimate_tamper_rejects(self):
        estimate = copy.deepcopy(self.row["branch_selection_prior_estimate"])
        estimate[0] += 1.0
        self.assert_private_tamper_rejects(
            "branch_selection_prior_estimate", estimate
        )

    def test_prior_covariance_tamper_rejects(self):
        covariance = copy.deepcopy(
            self.row["branch_selection_prior_covariance"]
        )
        covariance[0][0] += 1.0
        self.assert_private_tamper_rejects(
            "branch_selection_prior_covariance", covariance
        )

    def test_prior_source_frame_tamper_rejects(self):
        self.assert_private_tamper_rejects(
            "branch_selection_prior_source_fresh_frame", 176
        )

    def test_prior_propagated_frame_tamper_rejects(self):
        self.assert_private_tamper_rejects(
            "branch_selection_prior_propagated_to_frame", 179
        )

    def test_prior_age_tamper_rejects(self):
        self.assert_private_tamper_rejects(
            "branch_selection_prior_age_frames", 2
        )

    def test_outgoing_state_tamper_rejects(self):
        estimate = copy.deepcopy(self.row["next_private_state_estimate"])
        estimate[1] += 1.0
        self.assert_private_tamper_rejects(
            "next_private_state_estimate", estimate
        )

    def test_held_command_tamper_rejects(self):
        held = copy.deepcopy(self.fixture["held_command"])
        held["command"][0] += 1.0
        with self.assertRaises(ValueError):
            validate_row(
                self.row,
                current_public=mechanism_public(self.fixture),
                previous_private=self.fixture["preceding_private_state"],
                held_command=held,
            )

    def test_command_source_frame_tamper_rejects(self):
        held = copy.deepcopy(self.fixture["held_command"])
        held["source_frame"] -= 1
        with self.assertRaises(ValueError):
            validate_row(
                self.row,
                current_public=mechanism_public(self.fixture),
                previous_private=self.fixture["preceding_private_state"],
                held_command=held,
            )

    def test_previous_frame_private_state_is_propagated_independently(self):
        incoming = self.fixture["preceding_private_state"]
        velocity = np.asarray(self.fixture["held_command"]["command"])
        previous = {
            **incoming,
            "estimate": (
                np.asarray(incoming["estimate"]) - 0.5 * velocity
            ).tolist(),
            "modeled_covariance": (
                np.asarray(incoming["modeled_covariance"])
                - 0.25 * np.eye(2)
            ).tolist(),
            "propagated_to_frame": 179,
            "age_frames": 2,
        }
        reconstructed = validate_row(
            self.row,
            current_public=mechanism_public(self.fixture),
            previous_private=previous,
            held_command=self.fixture["held_command"],
        )
        self.assertEqual(reconstructed["private_state"]["age_frames"], 0)


class BranchReconstructionTests(unittest.TestCase):
    def setUp(self):
        self.fixture = mechanism_fixture()
        self.row = replay.produce_smoke_row(
            case_id="mechanism_20260727_180_12",
            mechanism_fixture=self.fixture,
        )
        self.public = mechanism_public(self.fixture)

    def reconstruct(self, row=None, **overrides):
        return validate_row(
            self.row if row is None else row,
            current_public=self.public,
            previous_private=self.fixture["preceding_private_state"],
            held_command=self.fixture["held_command"],
            **overrides,
        )

    def assert_tamper_rejects(self, mutate):
        row = copy.deepcopy(self.row)
        mutate(row)
        with self.assertRaises(ValueError):
            self.reconstruct(row)

    def test_valid_considered_attempt_reconstructs_two_fixed_uav_branches(self):
        reconstructed = self.reconstruct()
        self.assertEqual(
            reconstructed["branch_reconstruction"]["reference_keys"],
            (("uav", 10), ("uav", 11)),
        )
        self.assertEqual(
            reconstructed["branch_reconstruction"]["selected_branch_id"],
            self.row["selected_branch_id"],
        )

    def test_predicted_output_from_new_selector_rejects(self):
        row = valid_smoke_row("none_pass")
        row.update(
            {
                "output_status": "predicted",
                "prediction_age": 1,
                "estimate": [1.0, 1.0],
                "aged_modeled_covariance": [
                    [1.0, 0.0],
                    [0.0, 1.0],
                ],
                "aged_modeled_radius": 3.0,
            }
        )
        with self.assertRaises(ValueError):
            validate_row(row)

    def test_truth_dependent_rebinding_rejects(self):
        row = copy.deepcopy(self.row)
        original_truth = copy.deepcopy(row["offline_truth_position"])
        row["offline_truth_position"][0] += 1.0
        residual = (
            np.asarray(row["offline_truth_position"])
            - np.asarray(row["estimate"])
        )
        covariance = np.asarray(row["fresh_modeled_covariance"])
        row["offline_error_norm"] = float(np.linalg.norm(residual))
        row["offline_fresh_q_error"] = float(
            residual @ np.linalg.solve(covariance, residual)
        )
        row["offline_fresh_containment"] = (
            row["offline_error_norm"] <= row["fresh_epsilon"]
        )
        with self.assertRaises(ValueError):
            self.reconstruct(row, truth_position=original_truth)

    def test_forged_q_branch_rejects(self):
        self.assert_tamper_rejects(
            lambda row: row["branches"][0].__setitem__(
                "q_branch", row["branches"][0]["q_branch"] + 1.0
            )
        )

    def test_swapped_branch_labels_rejects(self):
        def mutate(row):
            row["branches"][0]["branch_id"] = "circle_positive"
            row["branches"][1]["branch_id"] = "circle_negative"

        self.assert_tamper_rejects(mutate)

    def test_hidden_branch_rejects(self):
        self.assert_tamper_rejects(lambda row: row["branches"].pop(0))

    def test_private_prior_as_reference_rejects(self):
        self.assert_tamper_rejects(
            lambda row: row["active_references"][0].__setitem__(
                "reference_kind", "private_prior"
            )
        )

    def test_private_prior_as_wnls_start_rejects(self):
        self.assert_tamper_rejects(
            lambda row: row["branches"][0].__setitem__(
                "circle_start",
                copy.deepcopy(row["branch_selection_prior_estimate"]),
            )
        )

    def test_algebraic_start_substitution_rejects(self):
        self.assert_tamper_rejects(
            lambda row: row["branches"][0].__setitem__(
                "circle_start", [0.0, 0.0]
            )
        )

    def test_distinct_circle_results_cannot_collapse(self):
        self.assert_tamper_rejects(
            lambda row: row["branches"][0].__setitem__(
                "solver_result",
                copy.deepcopy(row["branches"][1]["solver_result"]),
            )
        )

    def test_selected_result_cannot_rebind_other_branch(self):
        def mutate(row):
            row["selected_branch_id"] = "circle_positive"
            row["branches"][0]["passes_branch_gate"] = False
            row["branches"][1]["passes_branch_gate"] = True

        self.assert_tamper_rejects(mutate)

    def test_zero_pass_publication_rejects(self):
        def mutate(row):
            row["selected_branch_id"] = None
            for branch in row["branches"]:
                branch["passes_branch_gate"] = False

        self.assert_tamper_rejects(mutate)

    def test_multiple_pass_publication_rejects(self):
        def mutate(row):
            row["selected_branch_id"] = None
            for branch in row["branches"]:
                branch["passes_branch_gate"] = True

        self.assert_tamper_rejects(mutate)

    def test_fim_covariance_epsilon_substitution_rejects(self):
        def mutate(row):
            selected = row["branches"][1]["solver_result"]
            selected["covariance"] = [[2.0, 0.0], [0.0, 2.0]]
            selected["epsilon"] = 3.0 * (2.0**0.5)
            selected["phi_min_eigenvalue"] = 0.5
            selected["phi_condition"] = 1.0
            row["fresh_modeled_covariance"] = copy.deepcopy(
                selected["covariance"]
            )
            row["fresh_epsilon"] = selected["epsilon"]
            row["next_private_state_covariance"] = copy.deepcopy(
                selected["covariance"]
            )
            residual = (
                np.asarray(row["offline_truth_position"])
                - np.asarray(row["estimate"])
            )
            row["offline_fresh_q_error"] = float(
                residual
                @ np.linalg.solve(
                    np.asarray(selected["covariance"]), residual
                )
            )

        self.assert_tamper_rejects(mutate)

    def test_missing_fixed_reference_rejects(self):
        def mutate(row):
            row["mandatory_references"]["uav_ids"].pop(0)
            row["active_references"].pop(0)
            row["reference_evidence"].pop(0)
            row["reference_freshness"].pop(0)

        self.assert_tamper_rejects(mutate)

    def test_optional_reference_substitution_rejects(self):
        def mutate(row):
            row["optional_candidates"] = [
                {"reference_kind": "uav", "reference_id": 9}
            ]

        self.assert_tamper_rejects(mutate)

    def test_consistent_alternate_fixed_reference_set_rejects(self):
        row = copy.deepcopy(self.row)
        remap = {10: 9, 11: 10}
        row["mandatory_references"]["uav_ids"] = [9, 10]
        for collection in (
            row["active_references"],
            row["reference_evidence"],
            row["reference_freshness"],
        ):
            for record in collection:
                record["reference_id"] = remap[record["reference_id"]]
        substituted_public = {
            9: self.public[10],
            10: self.public[11],
        }
        with self.assertRaisesRegex(ValueError, "reference set"):
            validate_row(
                row,
                current_public=substituted_public,
                previous_private=self.fixture["preceding_private_state"],
                held_command=self.fixture["held_command"],
            )

    def test_wrong_squad_local_index_rejects(self):
        row = registered_new_row()
        row["squad_local_index"] = 4
        with self.assertRaisesRegex(ValueError, "squad"):
            self.reconstruct(row)

    def test_reference_freshness_projection_mutation_rejects(self):
        row = registered_new_row()
        row["reference_freshness"][0]["current_freshness"] = "predicted"
        with self.assertRaisesRegex(ValueError, "freshness"):
            self.reconstruct(row)

    def test_measurement_noise_seed_mutation_rejects(self):
        row = registered_new_row()
        row["reference_evidence"][0]["noise_seed"] += 1
        with self.assertRaisesRegex(ValueError, "noise seed"):
            self.reconstruct(row)


class LinearPercentileTests(unittest.TestCase):
    def test_one_element_returns_the_element(self):
        self.assertEqual(analyzer.linear_percentile([4.0], 0.95), 4.0)

    def test_odd_cardinality_uses_linear_index(self):
        self.assertEqual(
            analyzer.linear_percentile([20.0, 0.0, 10.0], 0.5),
            10.0,
        )

    def test_even_cardinality_interpolates(self):
        self.assertEqual(
            analyzer.linear_percentile([0.0, 10.0, 20.0, 30.0], 0.5),
            15.0,
        )

    def test_empty_returns_none(self):
        self.assertIsNone(analyzer.linear_percentile([], 0.95))

    def test_nonfinite_returns_none(self):
        for value in (float("nan"), float("inf"), -float("inf")):
            with self.subTest(value=value):
                self.assertIsNone(
                    analyzer.linear_percentile([0.0, value], 0.95)
                )

    def test_matches_numpy_linear_percentile(self):
        values = [7.0, 1.0, 9.0, 3.0, 20.0, 11.0]
        self.assertEqual(
            analyzer.linear_percentile(values, 0.95),
            float(np.percentile(values, 95, method="linear")),
        )


class AggregateGateTests(unittest.TestCase):
    def test_duplicate_missing_extra_and_out_of_order_new_stream_reject(self):
        baseline, v4, new = comparison_rows()
        first = new[0]
        second = copy.deepcopy(first)
        second["seed"] += 1
        replay._validate_row(second)
        mutations = {
            "duplicate": [first, copy.deepcopy(first)],
            "missing": [],
            "extra": [first, second],
            "out_of_order": [second, first],
        }
        messages = {
            "duplicate": "duplicate",
            "missing": "exact keys differ",
            "extra": "exact keys differ",
            "out_of_order": "out of order",
        }
        for name, changed in mutations.items():
            with self.subTest(name=name):
                with self.assertRaisesRegex(ValueError, messages[name]):
                    analyzer.aggregate_two_range_reacquisition(
                        baseline_rows=baseline,
                        v4_rows=v4,
                        new_rows=changed,
                        truth_data={"config": fixed_topology_config()},
                        protocol={
                            "protocol_id": "test-protocol",
                            "gates": analyzer.GATES,
                        },
                    )

    def test_empty_paired_both_fresh_cohort_fails_gate(self):
        baseline, v4, new = comparison_rows(baseline_fresh=False)
        result = analyzer.aggregate_two_range_reacquisition(
            baseline_rows=baseline,
            v4_rows=v4,
            new_rows=new,
            truth_data={},
            protocol={"protocol_id": "test-protocol", "gates": analyzer.GATES},
        )
        self.assertEqual(
            result["paired_comparison"],
            {
                "cohort_size": 0,
                "baseline_p95_m": None,
                "new_p95_m": None,
                "new_minus_baseline_p95_m": None,
            },
        )
        paired_gate = result["scientific_gates"][2]
        self.assertFalse(paired_gate["passed"])

    def test_zero_paired_cohort_is_canonical_and_validates(self):
        baseline, v4, new = comparison_rows(baseline_fresh=False)
        result = analyzer.aggregate_two_range_reacquisition(
            baseline_rows=baseline,
            v4_rows=v4,
            new_rows=new,
            truth_data={"config": fixed_topology_config()},
            protocol={
                "protocol_id": "test-protocol",
                "gates": analyzer.GATES,
            },
        )
        result = canonical_registered_result(result)
        analyzer._validate_analysis_result(result)
        self.assertEqual(result["paired_comparison"]["cohort_size"], 0)
        self.assertFalse(result["scientific_gates"][2]["passed"])

    def test_compact_compressed_decompressed_descriptor_mismatch_rejects(self):
        baseline, v4, new = comparison_rows(baseline_fresh=False)
        result = analyzer.aggregate_two_range_reacquisition(
            baseline_rows=baseline,
            v4_rows=v4,
            new_rows=new,
            truth_data={"config": fixed_topology_config()},
            protocol={
                "protocol_id": "test-protocol",
                "gates": analyzer.GATES,
            },
        )
        result = canonical_registered_result(result)
        result["identities"]["raw_decompressed_process"]["inode"] += 1
        with self.assertRaisesRegex(ValueError, "descriptor differs"):
            analyzer._validate_analysis_result(result)

    def aggregate(self, *, baseline_error=1.0, new_error=None):
        baseline, v4, new = comparison_rows(
            baseline_error=baseline_error,
            new_error=new_error,
        )
        return analyzer.aggregate_two_range_reacquisition(
            baseline_rows=baseline,
            v4_rows=v4,
            new_rows=new,
            truth_data={},
            protocol={"protocol_id": "test-protocol", "gates": analyzer.GATES},
        )

    def test_one_row_equality_paired_cohort_passes(self):
        result = self.aggregate(baseline_error=2.0, new_error=2.0)
        self.assertEqual(result["paired_comparison"]["cohort_size"], 1)
        self.assertEqual(
            result["paired_comparison"]["new_minus_baseline_p95_m"], 0.0
        )
        self.assertTrue(result["scientific_gates"][2]["passed"])

    def test_paired_improvement_passes(self):
        result = self.aggregate(baseline_error=2.0, new_error=1.0)
        self.assertEqual(
            result["paired_comparison"]["new_minus_baseline_p95_m"], -1.0
        )
        self.assertTrue(result["scientific_gates"][2]["passed"])

    def test_paired_worsening_fails(self):
        result = self.aggregate(baseline_error=1.0, new_error=2.0)
        self.assertEqual(
            result["paired_comparison"]["new_minus_baseline_p95_m"], 1.0
        )
        self.assertFalse(result["scientific_gates"][2]["passed"])

    def test_maximum_error_gate_is_strict_at_fifty(self):
        for value, passed in (
            (np.nextafter(50.0, -np.inf).item(), True),
            (50.0, False),
            (np.nextafter(50.0, np.inf).item(), False),
        ):
            with self.subTest(value=value):
                result = self.aggregate(new_error=value)
                self.assertIs(
                    result["scientific_gates"][0]["passed"], passed
                )
                self.assertIs(
                    result["scientific_gates"][1]["passed"], passed
                )

    def test_exact_transition_sums_and_tail_orders(self):
        result = self.aggregate()
        transitions = result["baseline_fresh_transitions"]
        self.assertEqual(
            transitions["baseline_fresh_total"],
            transitions["new_fresh"]
            + transitions["new_predicted"]
            + transitions["new_unavailable"],
        )
        self.assertEqual(len(result["tails"]), 1092)
        self.assertEqual(
            len(result["v4_descriptive_comparison"]["tails"]), 92
        )
        self.assertEqual(
            [
                (record["metric"], record["stratifier"])
                for record in result["tails"][:7]
            ],
            [("offline_error_norm", "depth")] * 7,
        )
        self.assertNotIn(
            "private_age",
            {
                record["stratifier"]
                for record in result["v4_descriptive_comparison"]["tails"]
            },
        )

    def test_availability_integer_boundaries_are_exact(self):
        paired = {
            "cohort_size": 1,
            "baseline_p95_m": 1.0,
            "new_p95_m": 1.0,
            "new_minus_baseline_p95_m": 0.0,
        }
        template = {
            "output_status": "fresh",
            "prediction_age": 0,
            "offline_error_norm": 1.0,
            "reference_evidence": [],
            "reference_violations": [],
            "active_references": [],
            "robot_id": 1,
        }
        for fresh_count, passed in ((124646, False), (124647, True)):
            rows = [template] * fresh_count
            rows += [
                {**template, "output_status": "unavailable"}
            ] * (140000 - fresh_count)
            records = analyzer._scientific_gate_records(
                new_rows=rows,
                paired=paired,
                baseline_fresh_total=127190,
                baseline_fresh_new_fresh=fresh_count,
                expected_rows=140000,
            )
            with self.subTest(fresh_count=fresh_count):
                self.assertIs(records[3]["passed"], passed)
        for available_count, passed in ((132999, False), (133000, True)):
            rows = [template] * available_count
            rows += [
                {**template, "output_status": "unavailable"}
            ] * (140000 - available_count)
            records = analyzer._scientific_gate_records(
                new_rows=rows,
                paired=paired,
                baseline_fresh_total=127190,
                baseline_fresh_new_fresh=127190,
                expected_rows=140000,
            )
            with self.subTest(available_count=available_count):
                self.assertIs(records[4]["passed"], passed)

    def test_nonbaseline_fresh_rows_cannot_compensate_retention_loss(self):
        paired = {
            "cohort_size": 1,
            "baseline_p95_m": 1.0,
            "new_p95_m": 1.0,
            "new_minus_baseline_p95_m": 0.0,
        }
        template = {
            "output_status": "fresh",
            "prediction_age": 0,
            "offline_error_norm": 1.0,
            "reference_evidence": [],
            "reference_violations": [],
            "active_references": [],
            "robot_id": 1,
        }
        rows = [template] * 124647
        records = analyzer._scientific_gate_records(
            new_rows=rows,
            paired=paired,
            baseline_fresh_total=127190,
            baseline_fresh_new_fresh=124646,
            expected_rows=140000,
        )
        retention = records[3]
        self.assertEqual(retention["numerator"], 124646)
        self.assertEqual(retention["denominator"], 127190)
        self.assertFalse(retention["passed"])

    def test_prediction_age_two_passes_and_three_fails(self):
        paired = {
            "cohort_size": 1,
            "baseline_p95_m": 1.0,
            "new_p95_m": 1.0,
            "new_minus_baseline_p95_m": 0.0,
        }
        template = {
            "output_status": "predicted",
            "offline_error_norm": 1.0,
            "reference_evidence": [],
            "reference_violations": [],
            "active_references": [],
            "robot_id": 1,
        }
        for age, passed in ((2, True), (3, False)):
            records = analyzer._scientific_gate_records(
                new_rows=[{**template, "prediction_age": age}],
                paired=paired,
                baseline_fresh_total=1,
                baseline_fresh_new_fresh=1,
                expected_rows=1,
            )
            with self.subTest(age=age):
                self.assertIs(records[5]["passed"], passed)

    def test_three_scientific_integrity_counts_require_zero(self):
        paired = {
            "cohort_size": 1,
            "baseline_p95_m": 1.0,
            "new_p95_m": 1.0,
            "new_minus_baseline_p95_m": 0.0,
        }
        base = {
            "output_status": "fresh",
            "prediction_age": 0,
            "offline_error_norm": 1.0,
            "reference_evidence": [],
            "reference_violations": [],
            "active_references": [],
            "robot_id": 2,
        }
        mutations = (
            {
                "reference_evidence": [
                    {
                        "used": True,
                        "current_freshness": "predicted",
                    }
                ]
            },
            {
                "reference_violations": [
                    {"reason": "provenance_invalid"}
                ]
            },
            {
                "active_references": [
                    {"reference_kind": "uav", "reference_id": 2}
                ]
            },
        )
        exact = analyzer._scientific_gate_records(
            new_rows=[base],
            paired=paired,
            baseline_fresh_total=1,
            baseline_fresh_new_fresh=1,
            expected_rows=1,
        )
        self.assertTrue(all(record["passed"] for record in exact[6:9]))
        for offset, mutation in enumerate(mutations):
            records = analyzer._scientific_gate_records(
                new_rows=[{**base, **mutation}],
                paired=paired,
                baseline_fresh_total=1,
                baseline_fresh_new_fresh=1,
                expected_rows=1,
            )
            with self.subTest(gate=offset):
                self.assertFalse(records[6 + offset]["passed"])

    def test_all_integrity_gates_require_exactly_zero(self):
        for gate_id in analyzer.INTEGRITY_GATE_IDS:
            zero = {item: 0 for item in analyzer.INTEGRITY_GATE_IDS}
            records = analyzer._integrity_gate_records(
                zero, denominator=140000
            )
            record = records[analyzer.INTEGRITY_GATE_IDS.index(gate_id)]
            self.assertTrue(record["passed"])
            one = dict(zero)
            one[gate_id] = 1
            records = analyzer._integrity_gate_records(
                one, denominator=140000
            )
            record = records[analyzer.INTEGRITY_GATE_IDS.index(gate_id)]
            self.assertFalse(record["passed"])
            self.assertEqual(record["operator"], "equal")
            self.assertEqual(record["threshold"], 0)

    def test_aggregate_integrity_counts_freshness_projection_mutation(self):
        baseline, v4, new = comparison_rows()
        new[0]["reference_freshness"][0][
            "current_freshness"
        ] = "predicted"
        result = analyzer.aggregate_two_range_reacquisition(
            baseline_rows=baseline,
            v4_rows=v4,
            new_rows=new,
            truth_data={"config": fixed_topology_config()},
            protocol={
                "protocol_id": "test-protocol",
                "gates": analyzer.GATES,
            },
        )
        record = result["integrity_gates"][
            analyzer.INTEGRITY_GATE_IDS.index(
                "preserved_contract_violation"
            )
        ]
        self.assertEqual(record["numerator"], 1)
        self.assertFalse(record["passed"])
        self.assertEqual(result["decision"], "fail")

    def test_legal_aggregate_derives_all_fourteen_zero_integrity_counts(self):
        baseline, v4, new = comparison_rows()
        result = analyzer.aggregate_two_range_reacquisition(
            baseline_rows=baseline,
            v4_rows=v4,
            new_rows=new,
            truth_data={"config": fixed_topology_config()},
            protocol={
                "protocol_id": "test-protocol",
                "gates": analyzer.GATES,
            },
        )
        self.assertEqual(
            tuple(
                record["gate_id"]
                for record in result["integrity_gates"]
            ),
            analyzer.INTEGRITY_GATE_IDS,
        )
        self.assertEqual(
            [record["numerator"] for record in result["integrity_gates"]],
            [0] * len(analyzer.INTEGRITY_GATE_IDS),
        )

    def test_aggregate_integrity_counts_noise_seed_mutation(self):
        baseline, v4, new = comparison_rows()
        new[0]["reference_evidence"][0]["noise_seed"] += 1
        result = analyzer.aggregate_two_range_reacquisition(
            baseline_rows=baseline,
            v4_rows=v4,
            new_rows=new,
            truth_data={"config": fixed_topology_config()},
            protocol={
                "protocol_id": "test-protocol",
                "gates": analyzer.GATES,
            },
        )
        record = result["integrity_gates"][
            analyzer.INTEGRITY_GATE_IDS.index(
                "preserved_contract_violation"
            )
        ]
        self.assertEqual(record["numerator"], 1)
        self.assertFalse(record["passed"])

    def test_aggregate_integrity_counts_fixed_reference_substitution(self):
        baseline, v4, new = comparison_rows()
        row = new[0]
        remap = {10: 9, 11: 10}
        row["mandatory_references"]["uav_ids"] = [9, 10]
        for collection in (
            row["active_references"],
            row["reference_evidence"],
            row["reference_freshness"],
        ):
            for record in collection:
                record["reference_id"] = remap[record["reference_id"]]
        result = analyzer.aggregate_two_range_reacquisition(
            baseline_rows=baseline,
            v4_rows=v4,
            new_rows=new,
            truth_data={"config": fixed_topology_config()},
            protocol={
                "protocol_id": "test-protocol",
                "gates": analyzer.GATES,
            },
        )
        record = result["integrity_gates"][
            analyzer.INTEGRITY_GATE_IDS.index(
                "selector_reference_set_violation"
            )
        ]
        self.assertEqual(record["numerator"], 1)
        self.assertFalse(record["passed"])

    def test_outage_lineage_separates_root_and_downstream_unavailable(self):
        root = {
            "selector_considered": True,
            "attempt_status": "rejected",
            "attempt_failure_reason": "two_range_no_branch_passes",
            "branches": [
                {"passes_branch_gate": False},
                {"passes_branch_gate": False},
            ],
            "output_status": "unavailable",
            "offline_fresh_containment": None,
            "seed": 20260727,
            "frame_index": 0,
            "robot_id": 12,
        }
        downstream = {
            **root,
            "selector_considered": False,
            "attempt_status": "reference_unavailable",
            "attempt_failure_reason": "fixed_reference_unavailable",
            "branches": [],
            "frame_index": 1,
        }
        accounting = analyzer._selector_accounting([root, downstream])
        self.assertEqual(accounting["root_rejections"], 1)
        self.assertEqual(accounting["downstream_unavailable"], 1)
        self.assertEqual(accounting["outage_episode_count"], 1)
        self.assertEqual(accounting["outage_episode_lengths"], [2])


class InvocationSplitTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name).resolve()
        self.protocol_path = self.root / "protocol.json"
        self.raw_a = self.root / "raw-a"
        self.raw_b = self.root / "raw-b"
        self.analysis_a = self.root / "analysis-a"
        self.analysis_b = self.root / "analysis-b"
        self.protocol = {
            "protocol_id": "hermetic-two-range-analysis-v1",
            "ranging_sigma": 0.5,
            "disk_contract": {
                "launch_minimum_free_bytes": 0,
                "live_minimum_free_bytes": 0,
                "raw_bundle_max_allocated_bytes": 100_000_000,
                "compact_bundle_max_allocated_bytes": 10_000_000,
            },
            "invocations": {
                "smoke_a": {"output_root": str(self.raw_a)},
                "smoke_b": {"output_root": str(self.raw_b)},
                "smoke_analyzer_a": {
                    "invocation_name": "smoke_analyzer_a",
                    "input_root": str(self.raw_a),
                    "output_root": str(self.analysis_a),
                    "expected_rows": 18,
                    "authorization_required": False,
                    "retry_allowed": False,
                },
                "smoke_analyzer_b": {
                    "invocation_name": "smoke_analyzer_b",
                    "input_root": str(self.raw_b),
                    "output_root": str(self.analysis_b),
                    "expected_rows": 18,
                    "authorization_required": False,
                    "retry_allowed": False,
                },
            },
        }
        self.protocol_path.write_text(json.dumps(self.protocol))

    def tearDown(self):
        self.temporary.cleanup()

    def produce(self, invocation):
        root = self.raw_a if invocation == "smoke_a" else self.raw_b
        return replay.replay_two_range_reacquisition(
            protocol_path=self.protocol_path,
            data_path=FIXTURE_ROOT / "mechanism_20260727_180_12.json",
            input_manifest_path=FIXTURE_ROOT / "manifest.json",
            output_root=root,
            run_seeds=(),
            max_frames=0,
            invocation_name=invocation,
        )

    def assert_failed_analysis(self, output):
        terminal = json.loads(
            (output / analyzer.ANALYZER_MANIFEST_NAME).read_bytes()
        )
        self.assertEqual(terminal["status"], "failed")
        self.assertTrue(
            all(value is None for value in terminal["output_identities"].values())
        )
        self.assertFalse((output / analyzer.OUTPUT_JSON_NAME).exists())
        self.assertFalse((output / analyzer.OUTPUT_MARKDOWN_NAME).exists())
        self.assertEqual(
            sorted(path.name for path in output.iterdir()),
            [analyzer.ANALYZER_MANIFEST_NAME],
        )

    def registered_authorization_fixture(self):
        text = "I authorize this exact registered diagnostic replay."
        values = {
            "schema_id": replay.REGISTERED_AUTHORIZATION_SCHEMA_ID,
            "protocol_id": self.protocol["protocol_id"],
            "protocol_sha256": "1" * 64,
            "protocol_commit": "2" * 40,
            "preflight_commit": "3" * 40,
            "smoke_commit": "4" * 40,
            "smoke_a_compressed_sha256": "5" * 64,
            "smoke_a_decompressed_sha256": "6" * 64,
            "smoke_b_compressed_sha256": "7" * 64,
            "smoke_b_decompressed_sha256": "8" * 64,
            "smoke_analyzer_a_json_sha256": "9" * 64,
            "smoke_analyzer_a_markdown_sha256": "a" * 64,
            "smoke_analyzer_b_json_sha256": "b" * 64,
            "smoke_analyzer_b_markdown_sha256": "c" * 64,
            "smoke_semantic_payload_sha256": "d" * 64,
            "user_authorization_date": "2026-07-31",
            "user_authorization_text": text,
            "user_authorization_text_sha256": hashlib.sha256(
                text.encode()
            ).hexdigest(),
            "registered_replay_root": str(self.root / "registered-raw"),
            "registered_analyzer_root": str(
                self.root / "registered-analysis"
            ),
            "registered_retry_allowed": False,
        }
        return {
            field: values[field]
            for field in replay.REGISTERED_AUTHORIZATION_FIELDS
        }

    def invoke_authorization_validator(self, authorization):
        authorization_path = self.root / "authorization.json"
        authorization_path.write_text(json.dumps(authorization))
        _, authorization_identity = analyzer._pinned_file_identity(
            authorization_path
        )
        protocol_payload, protocol_identity = analyzer._pinned_file_identity(
            self.protocol_path
        )
        raw_manifest = {
            "authorization_identity": {
                field: authorization_identity[field]
                for field in replay.FILE_IDENTITY_FIELDS
            },
            "source_identities": {},
        }
        return analyzer._validate_registered_authorization(
            authorization_path=authorization_path,
            protocol_path=self.protocol_path,
            protocol_payload=protocol_payload,
            protocol=self.protocol,
            protocol_identity=protocol_identity,
            raw_root=self.root / "registered-raw",
            output_root=self.root / "registered-analysis",
            raw_manifest=raw_manifest,
        )

    def test_smoke_a_accepts_only_its_exact_eighteen_row_root(self):
        self.produce("smoke_a")
        completed = analyzer.analyze_two_range_reacquisition(
            protocol_path=self.protocol_path,
            raw_root=self.raw_a,
            output_root=self.analysis_a,
            invocation_name="smoke_analyzer_a",
        )
        self.assertEqual(completed, self.analysis_a)
        result = json.loads(
            (completed / analyzer.OUTPUT_JSON_NAME).read_bytes()
        )
        self.assertEqual(result["decision"], "smoke_pass")
        self.assertEqual(result["budgets"]["expected_rows"], 18)
        self.assertEqual(result["scientific_gates"], [])
        self.assertEqual(result["tails"], [])
        terminal = json.loads(
            (completed / analyzer.ANALYZER_MANIFEST_NAME).read_bytes()
        )
        analyzer._validate_analysis_manifest(
            terminal,
            expected_protocol_id=self.protocol["protocol_id"],
            expected_disk_contract=self.protocol["disk_contract"],
        )
        self.assertEqual(terminal["status"], "completed")
        self.assertTrue(
            all(
                identity is not None
                for identity in terminal["output_identities"].values()
            )
        )

    def test_smoke_b_accepts_only_its_exact_eighteen_row_root(self):
        self.produce("smoke_b")
        completed = analyzer.analyze_two_range_reacquisition(
            protocol_path=self.protocol_path,
            raw_root=self.raw_b,
            output_root=self.analysis_b,
            invocation_name="smoke_analyzer_b",
        )
        result = json.loads(
            (completed / analyzer.OUTPUT_JSON_NAME).read_bytes()
        )
        self.assertEqual(result["invocation_name"], "smoke_analyzer_b")
        self.assertEqual(result["decision"], "smoke_pass")

    def test_cross_invocation_raw_root_substitution_rejects(self):
        self.produce("smoke_a")
        with self.assertRaises(ValueError):
            analyzer.analyze_two_range_reacquisition(
                protocol_path=self.protocol_path,
                raw_root=self.raw_a,
                output_root=self.analysis_b,
                invocation_name="smoke_analyzer_b",
            )
        self.assertFalse(self.analysis_b.exists())

    def test_protocol_symlink_is_rejected_before_output_allocation(self):
        self.produce("smoke_a")
        alias = self.root / "protocol-alias.json"
        alias.symlink_to(self.protocol_path)
        with self.assertRaisesRegex(ValueError, "symbolic-link"):
            analyzer.analyze_two_range_reacquisition(
                protocol_path=alias,
                raw_root=self.raw_a,
                output_root=self.analysis_a,
                invocation_name="smoke_analyzer_a",
            )
        self.assertFalse(self.analysis_a.exists())

    def test_raw_root_symlink_is_rejected_before_output_allocation(self):
        self.produce("smoke_a")
        alias = self.root / "raw-alias"
        alias.symlink_to(self.raw_a, target_is_directory=True)
        self.protocol["invocations"]["smoke_analyzer_a"][
            "input_root"
        ] = str(alias)
        self.protocol_path.write_text(json.dumps(self.protocol))
        with self.assertRaisesRegex(ValueError, "symbolic-link"):
            analyzer.analyze_two_range_reacquisition(
                protocol_path=self.protocol_path,
                raw_root=alias,
                output_root=self.analysis_a,
                invocation_name="smoke_analyzer_a",
            )
        self.assertFalse(self.analysis_a.exists())

    def test_output_parent_symlink_is_rejected(self):
        real_parent = self.root / "real-output-parent"
        real_parent.mkdir()
        alias_parent = self.root / "output-parent-alias"
        alias_parent.symlink_to(real_parent, target_is_directory=True)
        output = alias_parent / "analysis"
        self.protocol["invocations"]["smoke_analyzer_a"][
            "output_root"
        ] = str(output)
        self.protocol_path.write_text(json.dumps(self.protocol))
        self.produce("smoke_a")
        with self.assertRaisesRegex(
            (ValueError, OSError), "symbolic-link|Too many levels"
        ):
            analyzer.analyze_two_range_reacquisition(
                protocol_path=self.protocol_path,
                raw_root=self.raw_a,
                output_root=output,
                invocation_name="smoke_analyzer_a",
            )
        self.assertFalse(real_parent.joinpath("analysis").exists())

    def test_authorization_symlink_is_rejected_without_following(self):
        authorization_path = self.root / "authorization-real.json"
        authorization_path.write_text(
            json.dumps(self.registered_authorization_fixture())
        )
        alias = self.root / "authorization-alias.json"
        alias.symlink_to(authorization_path)
        protocol_payload, protocol_identity = analyzer._pinned_file_identity(
            self.protocol_path
        )
        with self.assertRaisesRegex(ValueError, "symbolic-link"):
            analyzer._validate_registered_authorization(
                authorization_path=alias,
                protocol_path=self.protocol_path,
                protocol_payload=protocol_payload,
                protocol=self.protocol,
                protocol_identity=protocol_identity,
                raw_root=self.root / "registered-raw",
                output_root=self.root / "registered-analysis",
                raw_manifest={
                    "authorization_identity": {},
                    "source_identities": {},
                },
            )

    def test_registered_analyzer_rejects_absent_authorization(self):
        with self.assertRaisesRegex(ValueError, "requires authorization"):
            analyzer.analyze_two_range_reacquisition(
                protocol_path=self.protocol_path,
                raw_root=self.root / "registered-raw",
                output_root=self.root / "registered-analysis",
                invocation_name="registered_analyzer",
            )
        self.assertFalse((self.root / "registered-analysis").exists())

    def test_registered_analyzer_rejects_mismatched_authorization(self):
        authorization = self.registered_authorization_fixture()
        authorization["registered_analyzer_root"] = str(
            self.root / "substituted-analysis"
        )
        with self.assertRaisesRegex(ValueError, "exact binding"):
            self.invoke_authorization_validator(authorization)

    def test_registered_analyzer_rejects_dirty_or_uncommitted_state(self):
        authorization = self.registered_authorization_fixture()
        authorization["protocol_sha256"] = analyzer._pinned_file_identity(
            self.protocol_path
        )[1]["sha256"]
        with mock.patch.object(
            replay,
            "_validate_committed_registered_state",
            side_effect=ValueError("authorization-related tracked paths are dirty"),
        ):
            with self.assertRaisesRegex(ValueError, "dirty"):
                self.invoke_authorization_validator(authorization)

    def test_two_smokes_share_semantic_payload_but_not_invocation(self):
        self.produce("smoke_a")
        self.produce("smoke_b")
        analyzer.analyze_two_range_reacquisition(
            protocol_path=self.protocol_path,
            raw_root=self.raw_a,
            output_root=self.analysis_a,
            invocation_name="smoke_analyzer_a",
        )
        analyzer.analyze_two_range_reacquisition(
            protocol_path=self.protocol_path,
            raw_root=self.raw_b,
            output_root=self.analysis_b,
            invocation_name="smoke_analyzer_b",
        )
        left = json.loads(
            (self.analysis_a / analyzer.OUTPUT_JSON_NAME).read_bytes()
        )
        right = json.loads(
            (self.analysis_b / analyzer.OUTPUT_JSON_NAME).read_bytes()
        )
        self.assertNotEqual(
            left["invocation_name"], right["invocation_name"]
        )
        self.assertEqual(
            left["semantic_payload_sha256"],
            right["semantic_payload_sha256"],
        )

    def test_preexisting_output_root_retains_failed_forensic_manifest(self):
        self.produce("smoke_a")
        self.analysis_a.mkdir()
        with self.assertRaises(FileExistsError):
            analyzer.analyze_two_range_reacquisition(
                protocol_path=self.protocol_path,
                raw_root=self.raw_a,
                output_root=self.analysis_a,
                invocation_name="smoke_analyzer_a",
            )
        sibling = analyzer._analysis_preallocation_failure_path(
            self.analysis_a
        )
        terminal = json.loads(sibling.read_bytes())
        self.assertEqual(terminal["status"], "failed")
        self.assertTrue(
            all(value is None for value in terminal["output_identities"].values())
        )

    def test_compact_cap_overflow_retains_failed_manifest(self):
        self.produce("smoke_a")
        with mock.patch.object(
            analyzer, "COMPACT_OUTPUT_CAP_BYTES", 1
        ):
            with self.assertRaises(analyzer.DiskSpaceError):
                analyzer.analyze_two_range_reacquisition(
                    protocol_path=self.protocol_path,
                    raw_root=self.raw_a,
                    output_root=self.analysis_a,
                    invocation_name="smoke_analyzer_a",
                )
        self.assert_failed_analysis(self.analysis_a)

    def test_json_write_failure_retains_failed_manifest(self):
        self.produce("smoke_a")
        real_stage = analyzer._stage_output

        def fail_json_write(*args, **kwargs):
            if kwargs["final_name"] == analyzer.OUTPUT_JSON_NAME:
                with mock.patch.object(
                    analyzer,
                    "_write_all",
                    side_effect=OSError("injected JSON write failure"),
                ):
                    return real_stage(*args, **kwargs)
            return real_stage(*args, **kwargs)

        with mock.patch.object(
            analyzer,
            "_stage_output",
            side_effect=fail_json_write,
        ):
            with self.assertRaisesRegex(OSError, "JSON write"):
                analyzer.analyze_two_range_reacquisition(
                    protocol_path=self.protocol_path,
                    raw_root=self.raw_a,
                    output_root=self.analysis_a,
                    invocation_name="smoke_analyzer_a",
                )
        self.assert_failed_analysis(self.analysis_a)

    def test_json_fsync_failure_retains_failed_manifest(self):
        self.produce("smoke_a")
        real_stage = analyzer._stage_output

        def fail_fsync(*args, **kwargs):
            if kwargs["final_name"] == analyzer.OUTPUT_JSON_NAME:
                with mock.patch.object(
                    analyzer.os,
                    "fsync",
                    side_effect=OSError("injected JSON fsync failure"),
                ):
                    return real_stage(*args, **kwargs)
            return real_stage(*args, **kwargs)

        with mock.patch.object(
            analyzer, "_stage_output", side_effect=fail_fsync
        ):
            with self.assertRaisesRegex(OSError, "JSON fsync"):
                analyzer.analyze_two_range_reacquisition(
                    protocol_path=self.protocol_path,
                    raw_root=self.raw_a,
                    output_root=self.analysis_a,
                    invocation_name="smoke_analyzer_a",
                )
        self.assert_failed_analysis(self.analysis_a)

    def test_markdown_write_failure_retains_no_hidden_stage(self):
        self.produce("smoke_a")
        real_stage = analyzer._stage_output

        def fail_markdown_write(*args, **kwargs):
            if kwargs["final_name"] == analyzer.OUTPUT_MARKDOWN_NAME:
                with mock.patch.object(
                    analyzer,
                    "_write_all",
                    side_effect=OSError("injected Markdown write failure"),
                ):
                    return real_stage(*args, **kwargs)
            return real_stage(*args, **kwargs)

        with mock.patch.object(
            analyzer,
            "_stage_output",
            side_effect=fail_markdown_write,
        ):
            with self.assertRaisesRegex(OSError, "Markdown write"):
                analyzer.analyze_two_range_reacquisition(
                    protocol_path=self.protocol_path,
                    raw_root=self.raw_a,
                    output_root=self.analysis_a,
                    invocation_name="smoke_analyzer_a",
                )
        self.assert_failed_analysis(self.analysis_a)

    def test_markdown_fsync_failure_retains_no_hidden_stage(self):
        self.produce("smoke_a")
        real_stage = analyzer._stage_output

        def fail_markdown_fsync(*args, **kwargs):
            if kwargs["final_name"] == analyzer.OUTPUT_MARKDOWN_NAME:
                with mock.patch.object(
                    analyzer.os,
                    "fsync",
                    side_effect=OSError("injected Markdown fsync failure"),
                ):
                    return real_stage(*args, **kwargs)
            return real_stage(*args, **kwargs)

        with mock.patch.object(
            analyzer,
            "_stage_output",
            side_effect=fail_markdown_fsync,
        ):
            with self.assertRaisesRegex(OSError, "Markdown fsync"):
                analyzer.analyze_two_range_reacquisition(
                    protocol_path=self.protocol_path,
                    raw_root=self.raw_a,
                    output_root=self.analysis_a,
                    invocation_name="smoke_analyzer_a",
                )
        self.assert_failed_analysis(self.analysis_a)

    def test_stage_name_swap_cannot_publish_completed_manifest(self):
        self.produce("smoke_a")
        real_stage = analyzer._stage_output
        swapped = False

        def swap_json_stage(*args, **kwargs):
            nonlocal swapped
            stage = real_stage(*args, **kwargs)
            if (
                not swapped
                and kwargs["final_name"] == analyzer.OUTPUT_JSON_NAME
            ):
                swapped = True
                name = stage["name"]
                transaction = args[0]
                os.unlink(name, dir_fd=transaction["root_fd"])
                descriptor = os.open(
                    name,
                    os.O_WRONLY | os.O_CREAT | os.O_EXCL,
                    0o600,
                    dir_fd=transaction["root_fd"],
                )
                try:
                    os.write(descriptor, b"substituted stage\n")
                finally:
                    os.close(descriptor)
            return stage

        with mock.patch.object(
            analyzer,
            "_stage_output",
            side_effect=swap_json_stage,
        ):
            with self.assertRaisesRegex(ValueError, "stage|identity"):
                analyzer.analyze_two_range_reacquisition(
                    protocol_path=self.protocol_path,
                    raw_root=self.raw_a,
                    output_root=self.analysis_a,
                    invocation_name="smoke_analyzer_a",
                )
        self.assert_failed_analysis(self.analysis_a)

    def test_final_link_swap_cannot_publish_completed_manifest(self):
        self.produce("smoke_a")
        real_publish = analyzer._publish_staged_output
        swapped = False

        def swap_json_link(transaction, stage):
            nonlocal swapped
            identity = real_publish(transaction, stage)
            if (
                not swapped
                and stage["final_name"] == analyzer.OUTPUT_JSON_NAME
            ):
                swapped = True
                os.unlink(
                    stage["final_name"],
                    dir_fd=transaction["root_fd"],
                )
                descriptor = os.open(
                    stage["final_name"],
                    os.O_WRONLY | os.O_CREAT | os.O_EXCL,
                    0o600,
                    dir_fd=transaction["root_fd"],
                )
                try:
                    os.write(descriptor, b"substituted final link\n")
                finally:
                    os.close(descriptor)
            return identity

        with mock.patch.object(
            analyzer,
            "_publish_staged_output",
            side_effect=swap_json_link,
        ):
            with self.assertRaisesRegex(
                ValueError, "directory-entry identity"
            ):
                analyzer.analyze_two_range_reacquisition(
                    protocol_path=self.protocol_path,
                    raw_root=self.raw_a,
                    output_root=self.analysis_a,
                    invocation_name="smoke_analyzer_a",
                )
        self.assert_failed_analysis(self.analysis_a)

    def test_analyzer_crash_after_root_creation_retains_failed_manifest(self):
        self.produce("smoke_a")
        with mock.patch.object(
            analyzer,
            "_stage_output",
            side_effect=RuntimeError("injected analyzer crash"),
        ):
            with self.assertRaisesRegex(RuntimeError, "analyzer crash"):
                analyzer.analyze_two_range_reacquisition(
                    protocol_path=self.protocol_path,
                    raw_root=self.raw_a,
                    output_root=self.analysis_a,
                    invocation_name="smoke_analyzer_a",
                )
        self.assert_failed_analysis(self.analysis_a)

    def test_failed_lifecycle_is_retained_when_retry_is_refused(self):
        self.produce("smoke_a")
        with mock.patch.object(
            analyzer,
            "_stage_output",
            side_effect=RuntimeError("injected retained failure"),
        ):
            with self.assertRaisesRegex(RuntimeError, "retained failure"):
                analyzer.analyze_two_range_reacquisition(
                    protocol_path=self.protocol_path,
                    raw_root=self.raw_a,
                    output_root=self.analysis_a,
                    invocation_name="smoke_analyzer_a",
                )
        retained = (
            self.analysis_a / analyzer.ANALYZER_MANIFEST_NAME
        ).read_bytes()
        with self.assertRaises(FileExistsError):
            analyzer.analyze_two_range_reacquisition(
                protocol_path=self.protocol_path,
                raw_root=self.raw_a,
                output_root=self.analysis_a,
                invocation_name="smoke_analyzer_a",
            )
        self.assertEqual(
            (
                self.analysis_a / analyzer.ANALYZER_MANIFEST_NAME
            ).read_bytes(),
            retained,
        )
        sibling = analyzer._analysis_preallocation_failure_path(
            self.analysis_a
        )
        self.assertEqual(
            json.loads(sibling.read_bytes())["status"],
            "failed",
        )

    def test_raw_identity_mismatch_retains_observed_failed_manifest(self):
        self.produce("smoke_a")
        original = analyzer._reverify_inputs
        mutated = False

        def mutate_then_reverify(expected):
            nonlocal mutated
            if not mutated:
                mutated = True
                process = self.raw_a / replay.RAW_PROCESS_NAME
                process.write_bytes(process.read_bytes() + b"x")
            return original(expected)

        with mock.patch.object(
            analyzer,
            "_reverify_inputs",
            side_effect=mutate_then_reverify,
        ):
            with self.assertRaisesRegex(ValueError, "raw process|identity"):
                analyzer.analyze_two_range_reacquisition(
                    protocol_path=self.protocol_path,
                    raw_root=self.raw_a,
                    output_root=self.analysis_a,
                    invocation_name="smoke_analyzer_a",
                )
        self.assert_failed_analysis(self.analysis_a)
        terminal = json.loads(
            (self.analysis_a / analyzer.ANALYZER_MANIFEST_NAME).read_bytes()
        )
        self.assertEqual(
            terminal["source_identities"]["raw_compressed_process"]["size"],
            (self.raw_a / replay.RAW_PROCESS_NAME).stat().st_size,
        )

    def test_protocol_drift_before_completion_retains_observed_failure(self):
        self.produce("smoke_a")
        original = analyzer._reverify_inputs
        mutated = False

        def mutate_then_reverify(expected):
            nonlocal mutated
            if not mutated:
                mutated = True
                self.protocol_path.write_bytes(
                    self.protocol_path.read_bytes() + b"\n"
                )
            return original(expected)

        with mock.patch.object(
            analyzer,
            "_reverify_inputs",
            side_effect=mutate_then_reverify,
        ):
            with self.assertRaisesRegex(ValueError, "protocol|identity"):
                analyzer.analyze_two_range_reacquisition(
                    protocol_path=self.protocol_path,
                    raw_root=self.raw_a,
                    output_root=self.analysis_a,
                    invocation_name="smoke_analyzer_a",
                )
        self.assert_failed_analysis(self.analysis_a)
        terminal = json.loads(
            (self.analysis_a / analyzer.ANALYZER_MANIFEST_NAME).read_bytes()
        )
        self.assertEqual(
            terminal["protocol_identity"]["size"],
            self.protocol_path.stat().st_size,
        )

    def test_authorization_final_reverification_rejects_drift(self):
        authorization_path = self.root / "authorization-drift.json"
        authorization_path.write_bytes(b"pinned authorization\n")
        _, expected = analyzer._pinned_file_identity(authorization_path)
        authorization_path.write_bytes(b"mutated authorization\n")
        with self.assertRaisesRegex(
            analyzer._SourceIdentityMismatch,
            "authorization",
        ) as captured:
            analyzer._reverify_inputs({"authorization": expected})
        self.assertEqual(
            captured.exception.observed["authorization"]["sha256"],
            hashlib.sha256(b"mutated authorization\n").hexdigest(),
        )

    def test_comparator_reverification_failures_retain_failed_lifecycle(self):
        self.produce("smoke_a")
        self.produce("smoke_b")
        cases = (
            (
                "v4_compressed_process",
                self.raw_a,
                self.analysis_a,
                "smoke_analyzer_a",
            ),
            (
                "legacy_baseline_process",
                self.raw_b,
                self.analysis_b,
                "smoke_analyzer_b",
            ),
        )
        for label, raw_root, output_root, invocation in cases:
            with self.subTest(label=label):
                def reject(expected, source_label=label):
                    raise analyzer._SourceIdentityMismatch(
                        f"{source_label} identity changed",
                        expected,
                    )

                with mock.patch.object(
                    analyzer,
                    "_reverify_inputs",
                    side_effect=reject,
                ):
                    with self.assertRaisesRegex(
                        analyzer._SourceIdentityMismatch,
                        label,
                    ):
                        analyzer.analyze_two_range_reacquisition(
                            protocol_path=self.protocol_path,
                            raw_root=raw_root,
                            output_root=output_root,
                            invocation_name=invocation,
                        )
                self.assert_failed_analysis(output_root)

    def _assert_comparator_reverification_rejects(self, source_name):
        self.produce("smoke_a")
        process_path = self.raw_a / replay.RAW_PROCESS_NAME
        _, compressed, decompressed = analyzer._pinned_raw_rows(
            process_path
        )
        comparator_path = self.root / f"{source_name}.json"
        comparator_path.write_bytes(b"pinned comparator\n")
        _, comparator = analyzer._pinned_file_identity(comparator_path)
        expected = {
            "raw_compressed_process": compressed,
            "raw_decompressed_process": decompressed,
            source_name: comparator,
        }
        comparator_path.write_bytes(b"mutated comparator\n")
        with self.assertRaisesRegex(
            analyzer._SourceIdentityMismatch,
            source_name,
        ) as captured:
            analyzer._reverify_inputs(expected)
        self.assertEqual(
            captured.exception.observed[source_name]["size"],
            comparator_path.stat().st_size,
        )

    def test_v4_comparator_identity_mismatch_rejects(self):
        self._assert_comparator_reverification_rejects(
            "v4_compressed_process"
        )

    def test_baseline_identity_mismatch_rejects(self):
        self._assert_comparator_reverification_rejects(
            "legacy_baseline_process"
        )

    def test_corrupt_gzip_reverification_records_observed_prefix_digest(self):
        process_path = self.root / "corruptible.jsonl.gz"
        with gzip.GzipFile(
            filename=process_path, mode="wb", mtime=0
        ) as stream:
            stream.write(b'{"row":1}\n')
        _, expected_compressed, expected_decompressed = (
            analyzer._pinned_raw_rows(process_path)
        )
        process_path.write_bytes(b"not a gzip stream")
        with self.assertRaisesRegex(
            analyzer._SourceIdentityMismatch,
            "cannot be reverified",
        ) as captured:
            analyzer._reverify_inputs(
                {
                    "raw_compressed_process": expected_compressed,
                    "raw_decompressed_process": expected_decompressed,
                }
            )
        observed = captured.exception.observed
        self.assertNotEqual(
            observed["raw_decompressed_process"]["sha256"],
            expected_decompressed["sha256"],
        )
        self.assertEqual(
            observed["raw_decompressed_process"]["sha256"],
            hashlib.sha256(b"").hexdigest(),
        )


def manifest_identity(path, *, domain="file_bytes", seed=1):
    return {
        "path": path,
        "device": 1,
        "inode": seed,
        "size": 100,
        "allocated_bytes": 4096,
        "mtime_ns": seed,
        "sha256": f"{seed:064x}",
        "hash_domain": domain,
    }


def valid_analysis_manifest(
    invocation="smoke_analyzer_a",
    *,
    status="creating",
):
    paths = {
        "raw_manifest": "/tmp/raw/manifest.json",
        "raw_compressed_process": (
            f"/tmp/raw/{replay.RAW_PROCESS_NAME}"
        ),
        "raw_decompressed_process": (
            f"/tmp/raw/{replay.RAW_PROCESS_NAME}"
        ),
        "mechanism_fixture": (
            "/tmp/mechanism_20260727_180_12.json"
        ),
        "synthetic_case_source": (
            "/tmp/scripts/diagnostics/"
            "replay_two_range_reacquisition.py"
        ),
        "v4_manifest": "/tmp/v4/manifest.json",
        "v4_compressed_process": (
            "/tmp/v4/predictive-wnls-development.jsonl.gz"
        ),
        "v4_decompressed_process": (
            "/tmp/v4/predictive-wnls-development.jsonl.gz"
        ),
        "v4_analysis_manifest": "/tmp/v4-analysis/manifest.json",
        "v4_analysis_json": (
            "/tmp/v4-analysis/predictive-wnls-development.json"
        ),
        "v4_analysis_markdown": (
            "/tmp/v4-analysis/predictive-wnls-development.md"
        ),
        "legacy_baseline_process": "/tmp/legacy/calibration.jsonl.gz",
        "legacy_baseline_protocol_json": (
            "/tmp/docs/diagnostics/"
            "2026-07-30-predictive-wnls-stage1-protocol-v4.json"
        ),
        "truth_data": "/tmp/truth/data.json",
    }
    sources = {}
    for index, name in enumerate(
        analyzer.ANALYSIS_SOURCE_MEMBER_NAMES[invocation], 10
    ):
        domain = (
            "decompressed_jsonl_bytes"
            if name.endswith("decompressed_process")
            else "file_bytes"
        )
        descriptor_seed = (
            sources[name.replace("decompressed", "compressed")]["inode"]
            if name.endswith("decompressed_process")
            else index
        )
        sources[name] = manifest_identity(
            paths[name],
            domain=domain,
            seed=descriptor_seed,
        )
    terminal = status in {"completed", "failed"}
    outputs = {
        name: (
            manifest_identity(
                (
                    "/tmp/analysis/"
                    + (
                        analyzer.OUTPUT_JSON_NAME
                        if name == "analysis_json"
                        else analyzer.OUTPUT_MARKDOWN_NAME
                    )
                ),
                seed=50 + index,
            )
            if status == "completed"
            else None
        )
        for index, name in enumerate(analyzer.ANALYSIS_OUTPUT_MEMBER_NAMES)
    }
    expected_rows = 140000 if invocation == "registered_analyzer" else 18
    error = (
        {"type": "ValueError", "message": "injected"}
        if status == "failed"
        else None
    )
    protocol_path = (
        "/tmp/docs/diagnostics/"
        "2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json"
        if invocation == "registered_analyzer"
        else "/tmp/protocol.json"
    )
    authorization_path = (
        "/tmp/docs/diagnostics/reviews/"
        "2026-07-30-cbf2026-two-range-reacquisition-"
        "registered-authorization.json"
    )
    return analyzer._analysis_manifest(
        protocol_id="protocol-v1",
        invocation_name=invocation,
        status=status,
        output_root=Path("/tmp/analysis"),
        protocol_identity=manifest_identity(protocol_path, seed=2),
        authorization_identity=(
            manifest_identity(authorization_path, seed=3)
            if invocation == "registered_analyzer"
            else None
        ),
        source_identities=sources,
        output_identities=outputs,
        expected_rows=expected_rows,
        observed_rows=expected_rows if status == "completed" else 0,
        disk_contract={
            "launch_minimum_free_bytes": 0,
            "live_minimum_free_bytes": 0,
            "raw_bundle_max_allocated_bytes": 100_000_000,
            "compact_bundle_max_allocated_bytes": 10_000_000,
        },
        started_at="2026-07-31T00:00:00+00:00",
        completed_at=(
            "2026-07-31T00:01:00+00:00" if terminal else None
        ),
        error=error,
    )


class AnalysisManifestTests(unittest.TestCase):
    def test_creating_manifest_has_exact_full_field_order(self):
        manifest = valid_analysis_manifest()
        analyzer._validate_analysis_manifest(manifest)
        self.assertEqual(tuple(manifest), analyzer.ANALYSIS_MANIFEST_FIELDS)

    def test_missing_extra_and_reordered_manifest_fields_reject(self):
        exact = valid_analysis_manifest()
        missing = copy.deepcopy(exact)
        missing.pop("error")
        extra = copy.deepcopy(exact)
        extra["extra"] = None
        fields = list(exact)
        fields[0], fields[1] = fields[1], fields[0]
        reordered = {field: exact[field] for field in fields}
        for changed in (missing, extra, reordered):
            with self.assertRaises(ValueError):
                analyzer._validate_analysis_manifest(changed)

    def test_wrong_schema_protocol_method_and_invocation_reject(self):
        mutations = (
            ("schema_id", "wrong"),
            ("protocol_id", ""),
            ("method", "wrong"),
            ("invocation_name", "wrong"),
        )
        for field, value in mutations:
            changed = valid_analysis_manifest()
            changed[field] = value
            with self.subTest(field=field):
                with self.assertRaises(ValueError):
                    analyzer._validate_analysis_manifest(changed)
        with self.assertRaises(ValueError):
            analyzer._validate_analysis_manifest(
                valid_analysis_manifest(),
                expected_protocol_id="different",
            )

    def test_source_member_mutations_reject_for_every_invocation(self):
        for invocation in analyzer.ANALYZER_INVOCATIONS:
            exact = valid_analysis_manifest(invocation)
            names = list(exact["source_identities"])
            missing = copy.deepcopy(exact)
            missing["source_identities"].pop(names[0])
            extra = copy.deepcopy(exact)
            extra["source_identities"]["extra"] = copy.deepcopy(
                extra["source_identities"][names[0]]
            )
            reordered = copy.deepcopy(exact)
            reordered["source_identities"] = {
                name: reordered["source_identities"][name]
                for name in reversed(names)
            }
            substituted = copy.deepcopy(exact)
            substituted["source_identities"][names[0]] = copy.deepcopy(
                substituted["source_identities"][names[1]]
            )
            for changed in (missing, extra, reordered, substituted):
                with self.subTest(invocation=invocation):
                    with self.assertRaises(ValueError):
                        analyzer._validate_analysis_manifest(changed)

    def test_wrong_source_hash_domain_rejects(self):
        manifest = valid_analysis_manifest()
        manifest["source_identities"]["raw_decompressed_process"][
            "hash_domain"
        ] = "file_bytes"
        with self.assertRaises(ValueError):
            analyzer._validate_analysis_manifest(manifest)

    def test_same_domain_source_role_swap_rejects(self):
        manifest = valid_analysis_manifest("registered_analyzer")
        sources = manifest["source_identities"]
        left = copy.deepcopy(sources["legacy_baseline_protocol_json"])
        sources["legacy_baseline_protocol_json"] = copy.deepcopy(
            sources["truth_data"]
        )
        sources["truth_data"] = left
        with self.assertRaisesRegex(ValueError, "substituted"):
            analyzer._validate_analysis_manifest(manifest)

    def test_same_basename_manifest_role_swap_rejects(self):
        manifest = valid_analysis_manifest("registered_analyzer")
        sources = manifest["source_identities"]
        left = copy.deepcopy(sources["raw_manifest"])
        sources["raw_manifest"] = copy.deepcopy(sources["v4_manifest"])
        sources["v4_manifest"] = left
        with self.assertRaisesRegex(ValueError, "substituted"):
            analyzer._validate_analysis_manifest(manifest)

    def test_registered_protocol_and_authorization_role_substitution_rejects(
        self,
    ):
        protocol_substitution = valid_analysis_manifest(
            "registered_analyzer"
        )
        protocol_substitution["protocol_identity"]["path"] = (
            "/tmp/data.json"
        )
        authorization_substitution = valid_analysis_manifest(
            "registered_analyzer"
        )
        authorization_substitution["authorization_identity"]["path"] = (
            "/tmp/protocol.json"
        )
        for changed in (
            protocol_substitution,
            authorization_substitution,
        ):
            with self.assertRaisesRegex(ValueError, "substituted"):
                analyzer._validate_analysis_manifest(changed)

    def test_authorization_null_rules_reject(self):
        smoke = valid_analysis_manifest()
        smoke["authorization_identity"] = manifest_identity(
            "/tmp/authorization.json", seed=8
        )
        registered = valid_analysis_manifest("registered_analyzer")
        registered["authorization_identity"] = None
        for changed in (smoke, registered):
            with self.assertRaises(ValueError):
                analyzer._validate_analysis_manifest(changed)

    def test_output_identity_state_rules_reject(self):
        running = valid_analysis_manifest(status="running")
        running["output_identities"]["analysis_json"] = manifest_identity(
            f"/tmp/analysis/{analyzer.OUTPUT_JSON_NAME}",
            seed=70,
        )
        completed = valid_analysis_manifest(status="completed")
        completed["output_identities"]["analysis_json"] = None
        for changed in (running, completed):
            with self.assertRaises(ValueError):
                analyzer._validate_analysis_manifest(changed)

    def test_completed_output_identity_path_substitution_rejects(self):
        completed = valid_analysis_manifest(status="completed")
        completed["output_identities"]["analysis_json"]["path"] = (
            "/tmp/substituted.json"
        )
        with self.assertRaisesRegex(ValueError, "output.*substituted"):
            analyzer._validate_analysis_manifest(completed)

    def test_row_count_overflow_and_incomplete_completion_reject(self):
        overflow = valid_analysis_manifest()
        overflow["observed_rows"] = 19
        incomplete = valid_analysis_manifest(status="completed")
        incomplete["observed_rows"] = 17
        for changed in (overflow, incomplete):
            with self.assertRaises(ValueError):
                analyzer._validate_analysis_manifest(changed)

    def test_disk_timestamp_and_error_rule_mutations_reject(self):
        disk = valid_analysis_manifest()
        disk["disk_contract"]["compact_bundle_max_allocated_bytes"] = 1
        timestamp = valid_analysis_manifest()
        timestamp["completed_at"] = "2026-07-31T00:00:00+00:00"
        completed_error = valid_analysis_manifest(status="completed")
        completed_error["error"] = {
            "type": "ValueError",
            "message": "forged",
        }
        failed_null = valid_analysis_manifest(status="failed")
        failed_null["error"] = None
        with self.assertRaises(ValueError):
            analyzer._validate_analysis_manifest(
                disk,
                expected_disk_contract=valid_analysis_manifest()[
                    "disk_contract"
                ],
            )
        for changed in (timestamp, completed_error, failed_null):
            with self.assertRaises(ValueError):
                analyzer._validate_analysis_manifest(changed)
