import copy
import gzip
import hashlib
import json
import math
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

import scripts.diagnostics.replay_two_range_reacquisition as replay
from scripts.diagnostics.two_range_reacquisition import reset_private_state


FIXTURE_ROOT = Path(
    "tests/fixtures/cbf2026_two_range_reacquisition"
)


class OrderedStrictJsonTests(unittest.TestCase):
    def test_declared_non_alphabetical_order_is_preserved(self):
        payload = replay.ordered_strict_json_bytes(
            {"z": 1, "a": [2, 3], "m": "μ"},
            ("z", "a", "m"),
        )
        self.assertEqual(payload, '{"z":1,"a":[2,3],"m":"μ"}'.encode())

    def test_writer_rejects_numpy_nonfinite_and_unordered_objects(self):
        bad_values = (
            {"x": np.float64(1.0)},
            {"x": np.asarray([1.0])},
            {"x": math.nan},
            {"x": math.inf},
            {"x": {1}},
            {"x": (item for item in [1])},
            {1: "bad"},
        )
        for value in bad_values:
            with self.subTest(value=type(next(iter(value.values())))):
                with self.assertRaises(ValueError):
                    replay.ordered_strict_json_bytes(value, tuple(value))
        with self.assertRaises(ValueError):
            replay.ordered_strict_json_bytes({"a": 1, "b": 2}, ("b", "a"))
        self.assertEqual(
            replay.ordered_strict_json_bytes({"x": (1, 2)}, ("x",)),
            b'{"x":[1,2]}',
        )


def fresh_output():
    return {
        "output_status": "fresh",
        "prediction_age": 0,
        "estimate": [1.0, 2.0],
        "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
        "epsilon": 3.0,
        "aged_modeled_radius": None,
        "base_anchor_provenance": [0, 1],
    }


class RoutingTests(unittest.TestCase):
    def _config_and_truth(self):
        config = {
            "num": 7,
            "number": 7,
            "parts": 1,
            "formation": {"parts": 1, "bases-id": [[0, 1]]},
            "bases": [[1000.0, 1000.0], [1200.0, 1200.0]],
            "cbfs": {
                "without-slack": {
                    "comm-fixed": {
                        "min-neighbour-id-offset": -2,
                        "max-neighbour-id-offset": 0,
                        "max-range": 10.0,
                    },
                },
            },
        }
        truth = {
            identifier: np.asarray([100.0 * identifier, 100.0])
            for identifier in range(1, 8)
        }
        truth[4] = np.asarray([1.0, 0.0])
        truth[5] = np.asarray([0.0, 1.0])
        truth[6] = np.asarray([0.0, 0.0])
        return config, truth

    def _accepted_attempt(self):
        result = copy.deepcopy(
            replay.CANDIDATE_TEMPLATES[
                "canonical_spd_zero_residual_v1"
            ],
        )
        return {
            "attempt_status": "accepted",
            "failure_reason": None,
            "candidate": {
                "estimate": [1.0, 1.0],
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
                "epsilon": 3.0,
                "base_anchor_provenance": [0, 1],
            },
            "candidates": [],
            "selected_candidate": {
                "branch_id": "circle_positive",
            },
            "branches": [
                {
                    "branch_id": "circle_negative",
                    "circle_start": [1.0, -1.0],
                    "solver_result": copy.deepcopy(result),
                    "q_branch": 100.0,
                    "passes_branch_gate": False,
                },
                {
                    "branch_id": "circle_positive",
                    "circle_start": [1.0, 1.0],
                    "solver_result": copy.deepcopy(result),
                    "q_branch": 1.0,
                    "passes_branch_gate": True,
                },
            ],
            "selected_branch_id": "circle_positive",
            "prior_used_for_branch_selection": True,
        }

    def test_non_robot_12_canonical_two_uav_case_is_considered(self):
        considered = replay.selector_consideration(
            robot_id=6,
            live_prediction=None,
            mandatory={"base_ids": [], "uav_ids": [3, 5]},
            optional_keys=[],
            qualification={
                "status": "ok",
                "active_keys": [("uav", 3), ("uav", 5)],
                "active_records": [
                    {"key": ("uav", 3), "present": True, "noisy_range": 4.0},
                    {"key": ("uav", 5), "present": True, "noisy_range": 5.0},
                ],
                "missing_mandatory": [],
                "excluded": [],
                "violations": [],
                "base_anchor_provenance": (0, 1),
                "fixed_outputs": {3: fresh_output(), 5: fresh_output()},
            },
        )
        self.assertEqual(considered, (True, "considered"))

    def test_non_robot_12_full_producer_calls_only_two_range_solver(self):
        config, truth = self._config_and_truth()
        current_public = {
            4: fresh_output(),
            5: fresh_output(),
        }
        previous_state = {
            "public_output": replay.make_unavailable_output("expired"),
            "private_state": reset_private_state(
                {
                    "estimate": [1.0, 1.5],
                    "modeled_covariance": [[0.2, 0.0], [0.0, 0.2]],
                },
                frame_index=19,
            ),
        }
        with mock.patch.object(
            replay,
            "solve_two_range_reacquisition",
            return_value=self._accepted_attempt(),
        ) as two_range, mock.patch.object(
            replay,
            "solve_predictive_multistart",
        ) as existing:
            row, _ = replay.produce_method_row(
                seed=20260727,
                frame_index=20,
                robot_id=6,
                config=config,
                truth_positions=truth,
                current_public=current_public,
                previous_state=previous_state,
                applied_command=[0.0, 0.0],
                ranging_sigma=0.5,
            )
        two_range.assert_called_once()
        existing.assert_not_called()
        self.assertEqual(row["attempt_path"], "two_range_reacquisition")
        self.assertEqual(
            row["active_references"],
            [
                {"reference_kind": "uav", "reference_id": 4},
                {"reference_kind": "uav", "reference_id": 5},
            ],
        )

    def test_mechanism_fixture_calls_real_two_range_route(self):
        fixture = json.loads(
            (FIXTURE_ROOT / "mechanism_20260727_180_12.json").read_bytes()
        )
        with mock.patch.object(
            replay,
            "solve_two_range_reacquisition",
            wraps=replay.solve_two_range_reacquisition,
        ) as two_range, mock.patch.object(
            replay,
            "solve_predictive_multistart",
        ) as existing:
            row = replay.produce_smoke_row(
                case_id="mechanism_20260727_180_12",
                mechanism_fixture=fixture,
            )
        two_range.assert_called_once()
        existing.assert_not_called()
        self.assertTrue(row["selector_considered"])
        self.assertEqual(
            row["active_references"],
            [
                {"reference_kind": "uav", "reference_id": 10},
                {"reference_kind": "uav", "reference_id": 11},
            ],
        )

    def test_live_prediction_full_producer_calls_only_existing_solver(self):
        config, truth = self._config_and_truth()
        current_public = {4: fresh_output(), 5: fresh_output()}
        previous_state = {
            "public_output": fresh_output(),
            "private_state": reset_private_state(
                {
                    "estimate": [1.0, 1.5],
                    "modeled_covariance": [[0.2, 0.0], [0.0, 0.2]],
                },
                frame_index=19,
            ),
        }
        frozen_attempt = {
            "attempt_status": "invalid",
            "status": "invalid",
            "candidates": [],
            "selected_candidate": None,
            "candidate": None,
            "failure_reason": "no_valid_initial_candidates",
        }
        with mock.patch.object(
            replay,
            "solve_two_range_reacquisition",
        ) as two_range, mock.patch.object(
            replay,
            "solve_predictive_multistart",
            return_value=frozen_attempt,
        ) as existing:
            row, _ = replay.produce_method_row(
                seed=20260727,
                frame_index=20,
                robot_id=6,
                config=config,
                truth_positions=truth,
                current_public=current_public,
                previous_state=previous_state,
                applied_command=[0.0, 0.0],
                ranging_sigma=0.5,
            )
        existing.assert_called_once()
        two_range.assert_not_called()
        self.assertEqual(row["attempt_path"], "existing_predictive_multistart")
        self.assertEqual(
            row["selector_consideration_reason"],
            "live_public_prediction",
        )

    def test_ineligible_structures_route_to_frozen_existing_path(self):
        base = {
            "status": "ok",
            "active_keys": [("uav", 3), ("uav", 5)],
            "active_records": [
                {"key": ("uav", 3), "present": True, "noisy_range": 4.0},
                {"key": ("uav", 5), "present": True, "noisy_range": 5.0},
            ],
            "base_anchor_provenance": (0, 1),
            "fixed_outputs": {3: fresh_output(), 5: fresh_output()},
        }
        cases = (
            ("live_public_prediction", {"live_prediction": fresh_output()}),
            (
                "active_reference_count_not_two",
                {"qualification": {**base, "active_keys": [("uav", 3)]}},
            ),
            (
                "active_base_present",
                {"qualification": {
                    **base,
                    "active_keys": [("base", 0), ("uav", 3)],
                }},
            ),
            ("active_optional_present", {"optional_keys": [("uav", 4)]}),
            (
                "reference_not_strictly_lower_index",
                {
                    "mandatory": {"base_ids": [], "uav_ids": [3, 6]},
                    "qualification": {
                        **base,
                        "active_keys": [("uav", 3), ("uav", 6)],
                        "fixed_outputs": {
                            3: fresh_output(),
                            6: fresh_output(),
                        },
                    },
                },
            ),
            (
                "range_invalid",
                {"qualification": {
                    **base,
                    "active_records": [
                        {
                            "key": ("uav", 3),
                            "present": True,
                            "noisy_range": -1.0,
                        },
                        base["active_records"][1],
                    ],
                }},
            ),
            (
                "provenance_invalid",
                {"qualification": {
                    **base,
                    "base_anchor_provenance": (0,),
                }},
            ),
        )
        for expected, overrides in cases:
            kwargs = {
                "robot_id": 6,
                "live_prediction": None,
                "mandatory": {"base_ids": [], "uav_ids": [3, 5]},
                "optional_keys": [],
                "qualification": copy.deepcopy(base),
            }
            kwargs.update(overrides)
            with self.subTest(expected=expected):
                self.assertEqual(
                    replay.selector_consideration(**kwargs),
                    (False, expected),
                )


class SmokeRowTests(unittest.TestCase):
    def test_select_positive_flattens_prior_and_next_state(self):
        fixture = json.loads(
            (FIXTURE_ROOT / "mechanism_20260727_180_12.json").read_bytes()
        )
        row = replay.produce_smoke_row(
            case_id="select_positive",
            mechanism_fixture=fixture,
        )
        self.assertEqual(tuple(row), replay.ROW_FIELDS)
        self.assertEqual(row["invocation_name"], "smoke_validation")
        self.assertEqual(row["smoke_case_kind"], "selector")
        self.assertEqual(row["attempt_path"], "two_range_reacquisition")
        self.assertEqual(row["attempt_status"], "accepted")
        self.assertEqual(row["selected_branch_id"], "circle_positive")
        self.assertEqual(row["branch_selection_prior_status"], "available")
        self.assertEqual(
            row["branch_selection_prior_source_fresh_frame"], 20
        )
        self.assertEqual(
            row["next_private_state_source_fresh_frame"], 20
        )
        self.assertEqual(row["next_private_state_age_frames"], 0)
        self.assertEqual(len(row["branches"]), 2)

    def test_exact_ordered_smoke_matrix_executes_all_18_cases(self):
        fixture = json.loads(
            (FIXTURE_ROOT / "mechanism_20260727_180_12.json").read_bytes()
        )
        rows = [
            replay.produce_smoke_row(
                case_id=case_id,
                mechanism_fixture=fixture,
            )
            for case_id in replay.SMOKE_CASE_IDS
        ]
        self.assertEqual(
            [row["smoke_case_id"] for row in rows],
            list(replay.SMOKE_CASE_IDS),
        )
        self.assertTrue(all(tuple(row) == replay.ROW_FIELDS for row in rows))
        self.assertTrue(
            all(row["invocation_name"] == "smoke_validation" for row in rows)
        )


class RowAndKeyMutationTests(unittest.TestCase):
    def setUp(self):
        fixture = json.loads(
            (FIXTURE_ROOT / "mechanism_20260727_180_12.json").read_bytes()
        )
        self.fixture = fixture
        self.mechanism = replay.produce_smoke_row(
            case_id="mechanism_20260727_180_12",
            mechanism_fixture=fixture,
        )
        self.accepted = replay.produce_smoke_row(
            case_id="select_positive",
            mechanism_fixture=fixture,
        )
        self.component = replay.produce_smoke_row(
            case_id="q_equal_threshold",
            mechanism_fixture=fixture,
        )
        self.rejected = replay.produce_smoke_row(
            case_id="none_pass",
            mechanism_fixture=fixture,
        )
        self.candidate = replay.produce_smoke_row(
            case_id="cost_equal_nine",
            mechanism_fixture=fixture,
        )["existing_candidates"][0]

    def test_six_reviewed_strict_row_bypasses_reject(self):
        wrong_mechanism_seed = copy.deepcopy(self.mechanism)
        wrong_mechanism_seed["seed"] += 1

        accepted_component = copy.deepcopy(self.component)
        accepted_component["attempt_status"] = "accepted"

        accepted_unavailable = copy.deepcopy(self.accepted)
        accepted_unavailable.update({
            "output_status": "unavailable",
            "prediction_age": None,
            "estimate": None,
            "fresh_modeled_covariance": None,
            "fresh_epsilon": None,
        })

        rejected_private_mutation = copy.deepcopy(self.rejected)
        rejected_private_mutation["next_private_state_estimate"][0] += 1.0

        illegal_solver_status = copy.deepcopy(self.accepted)
        illegal_solver_status["branches"][0]["solver_result"][
            "status"
        ] = "arbitrary"

        reversed_reference_evidence = copy.deepcopy(self.mechanism)
        reversed_reference_evidence["reference_evidence"].reverse()

        for name, row in (
            ("wrong_mechanism_seed", wrong_mechanism_seed),
            ("accepted_component", accepted_component),
            ("accepted_unavailable", accepted_unavailable),
            ("rejected_private_mutation", rejected_private_mutation),
            ("illegal_solver_status", illegal_solver_status),
            ("reversed_reference_evidence", reversed_reference_evidence),
        ):
            with self.subTest(name=name):
                with self.assertRaises(ValueError):
                    replay._validate_row(row)

    def test_nested_solver_and_gate_discriminant_bypasses_reject(self):
        failed_accepted_branch = copy.deepcopy(self.accepted)
        failed_accepted_branch["branches"][0]["solver_result"][
            "status"
        ] = "failed"

        incomplete_converged_branch = copy.deepcopy(self.accepted)
        incomplete_converged_branch["branches"][0]["solver_result"][
            "covariance"
        ] = None

        mismatched_trace_count = copy.deepcopy(self.accepted)
        mismatched_trace_count["branches"][0]["solver_result"][
            "proposal_count"
        ] = 1

        arbitrary_gate_outcome = replay.produce_smoke_row(
            case_id="cost_equal_nine",
            mechanism_fixture=self.fixture,
        )
        arbitrary_gate_outcome["existing_candidates"][0][
            "gate_diagnostics"
        ]["gate_outcome"] = "arbitrary"

        arbitrary_candidate_source = replay.produce_smoke_row(
            case_id="cost_equal_nine",
            mechanism_fixture=self.fixture,
        )
        arbitrary_candidate_source["existing_candidates"][0][
            "source"
        ] = "arbitrary"

        accepted_rejected_gate = replay.produce_smoke_row(
            case_id="cost_equal_nine",
            mechanism_fixture=self.fixture,
        )
        accepted_rejected_gate["existing_candidates"][0][
            "gate_diagnostics"
        ]["gate_outcome"] = "rejected"

        for name, row in (
            ("failed_accepted_branch", failed_accepted_branch),
            ("incomplete_converged_branch", incomplete_converged_branch),
            ("mismatched_trace_count", mismatched_trace_count),
            ("arbitrary_gate_outcome", arbitrary_gate_outcome),
            ("arbitrary_candidate_source", arbitrary_candidate_source),
            ("accepted_rejected_gate", accepted_rejected_gate),
        ):
            with self.subTest(name=name):
                with self.assertRaises(ValueError):
                    replay._validate_row(row)

    def test_cross_path_candidate_branch_and_cardinality_substitutions_reject(self):
        mutations = []
        with_existing = copy.deepcopy(self.accepted)
        with_existing["existing_candidates"] = [copy.deepcopy(self.candidate)]
        mutations.append(with_existing)
        existing = copy.deepcopy(self.accepted)
        branch = copy.deepcopy(existing["branches"][0])
        existing.update({
            "attempt_path": "existing_predictive_multistart",
            "selector_considered": False,
            "selector_consideration_reason": "live_public_prediction",
            "branches": [branch],
            "selected_branch_id": None,
            "prior_used_for_branch_selection": False,
        })
        mutations.append(existing)
        one_branch = copy.deepcopy(self.accepted)
        one_branch["branches"] = one_branch["branches"][:1]
        mutations.append(one_branch)
        zero_post = copy.deepcopy(self.accepted)
        zero_post.update({
            "attempt_status": "rejected",
            "attempt_failure_reason": "two_range_no_branch_passes",
            "branches": [],
            "selected_branch_id": None,
            "prior_used_for_branch_selection": False,
        })
        mutations.append(zero_post)
        two_pre = copy.deepcopy(self.accepted)
        two_pre.update({
            "attempt_status": "rejected",
            "attempt_failure_reason": "two_range_circle_geometry_invalid",
            "selected_branch_id": None,
            "prior_used_for_branch_selection": False,
        })
        for branch_record in two_pre["branches"]:
            branch_record["q_branch"] = None
            branch_record["passes_branch_gate"] = None
        mutations.append(two_pre)
        unavailable_selected = copy.deepcopy(self.accepted)
        unavailable_selected.update({
            "attempt_path": "reference_unavailable",
            "attempt_status": "reference_unavailable",
            "attempt_failure_reason": "reference_unavailable",
            "branches": [],
            "selected_branch_id": "circle_positive",
            "prior_used_for_branch_selection": False,
        })
        mutations.append(unavailable_selected)
        for index, row in enumerate(mutations):
            with self.subTest(index=index):
                with self.assertRaises(ValueError):
                    replay._validate_row(row)

    def test_old_positional_nested_encodings_reject(self):
        nested_fields = (
            "branches",
            "existing_candidates",
            "optional_candidates",
            "active_references",
            "reference_evidence",
            "reference_freshness",
            "excluded_references",
            "reference_violations",
        )
        for field in nested_fields:
            row = copy.deepcopy(self.accepted)
            row[field] = [["old", "positional"]]
            if field == "branches":
                row["selected_branch_id"] = None
                row["prior_used_for_branch_selection"] = False
            with self.subTest(field=field):
                with self.assertRaises((ValueError, TypeError)):
                    replay._validate_row(row)

    def test_registered_key_iterator_has_exact_140000_ordered_keys(self):
        keys = replay.iter_registered_keys()
        first = next(keys)
        count = 1
        last = first
        for last in keys:
            count += 1
        self.assertEqual(
            first,
            (replay.METHOD_ID, 20260727, 0, 1),
        )
        self.assertEqual(
            last,
            (replay.METHOD_ID, 20260746, 499, 14),
        )
        self.assertEqual(count, 140000)

    def test_duplicate_missing_extra_and_out_of_order_keys_reject(self):
        exact = [("m", 1), ("m", 2), ("m", 3)]
        self.assertEqual(replay.validate_key_sequence(exact, exact), 3)
        mutations = (
            [("m", 1), ("m", 1), ("m", 3)],
            [("m", 1), ("m", 2)],
            [("m", 1), ("m", 2), ("m", 3), ("m", 4)],
            [("m", 2), ("m", 1), ("m", 3)],
        )
        for observed in mutations:
            with self.subTest(observed=observed):
                with self.assertRaises(ValueError):
                    replay.validate_key_sequence(observed, exact)

    def test_gate_diagnostic_discriminants_and_partial_canonicalization(self):
        partial = replay._gate_diagnostics({
            "innovation_gate": "not_applicable_reacquisition",
            "q_innov": None,
            "gate_outcome": "rejected",
        })
        self.assertEqual(tuple(partial), replay.GATE_DIAGNOSTIC_FIELDS)
        self.assertIsNone(partial["valid"])
        self.assertIsNone(partial["failure_reason"])
        self.assertIsNone(partial["reduced_whitened_cost"])
        applied = replay._gate_diagnostics({
            "innovation_gate": "applied",
            "q_innov": 1.0,
            "gate_outcome": "accepted",
            "valid": True,
            "failure_reason": None,
        })
        self.assertEqual(applied["q_innov"], 1.0)
        reacquisition = replay._gate_diagnostics({
            "innovation_gate": "not_applicable_reacquisition",
            "q_innov": None,
            "gate_outcome": "accepted",
            "reduced_whitened_cost": 4.0,
        })
        self.assertEqual(reacquisition["reduced_whitened_cost"], 4.0)

    def test_gate_diagnostic_missing_half_present_and_unknown_keys_reject(self):
        bad = (
            {
                "innovation_gate": "applied",
                "q_innov": 1.0,
            },
            {
                "innovation_gate": "applied",
                "q_innov": 1.0,
                "gate_outcome": "accepted",
                "valid": True,
            },
            {
                "innovation_gate": "applied",
                "q_innov": 1.0,
                "gate_outcome": "accepted",
                "unknown": None,
            },
            {
                "innovation_gate": "not_applicable_reacquisition",
                "q_innov": None,
                "gate_outcome": "accepted",
                "valid": True,
                "failure_reason": None,
            },
        )
        for source in bad:
            with self.subTest(source=source):
                with self.assertRaises(ValueError):
                    replay._gate_diagnostics(source)

    def test_declared_row_contract_rejects_wrong_method_registered_smoke_and_unavailable_payload(self):
        wrong_method = copy.deepcopy(self.accepted)
        wrong_method["method"] = "wrong_method"
        registered_smoke = copy.deepcopy(self.accepted)
        registered_smoke["invocation_name"] = "registered_replay"
        unavailable_payload = replay.produce_smoke_row(
            case_id="q_equal_threshold",
            mechanism_fixture=json.loads(
                (FIXTURE_ROOT / "mechanism_20260727_180_12.json").read_bytes()
            ),
        )
        unavailable_payload["fresh_modeled_covariance"] = [
            [1.0, 0.0],
            [0.0, 1.0],
        ]
        unavailable_payload["fresh_epsilon"] = 3.0
        for name, row in (
            ("wrong_method", wrong_method),
            ("registered_smoke", registered_smoke),
            ("unavailable_payload", unavailable_payload),
        ):
            with self.subTest(name=name):
                with self.assertRaises(ValueError):
                    replay._validate_row(row)

    def test_every_declared_smoke_row_form_validates_then_null_mutations_reject(self):
        fixture = json.loads(
            (FIXTURE_ROOT / "mechanism_20260727_180_12.json").read_bytes()
        )
        rows = [
            replay.produce_smoke_row(
                case_id=case_id,
                mechanism_fixture=fixture,
            )
            for case_id in replay.SMOKE_CASE_IDS
        ]
        for row in rows:
            replay._validate_row(row)
        fresh = next(row for row in rows if row["output_status"] == "fresh")
        fresh_with_aged = copy.deepcopy(fresh)
        fresh_with_aged["aged_modeled_covariance"] = [[1.0, 0.0], [0.0, 1.0]]
        fresh_with_aged["aged_modeled_radius"] = 3.0
        absent_prior = next(
            row for row in rows
            if row["branch_selection_prior_status"] == "absent"
        )
        absent_prior_payload = copy.deepcopy(absent_prior)
        absent_prior_payload["branch_selection_prior_estimate"] = [0.0, 0.0]
        for row in (fresh_with_aged, absent_prior_payload):
            with self.assertRaises(ValueError):
                replay._validate_row(row)


class ProducerLifecycleTests(unittest.TestCase):
    class RegisteredStop(RuntimeError):
        pass

    def _protocol(
        self,
        root,
        *,
        launch_floor=0,
        live_floor=0,
        raw_cap=100_000_000,
    ):
        protocol = root / "protocol.json"
        protocol.write_text(
            json.dumps({
                "protocol_id": "hermetic-two-range-smoke-v1",
                "disk_contract": {
                    "launch_minimum_free_bytes": launch_floor,
                    "live_minimum_free_bytes": live_floor,
                    "raw_bundle_max_allocated_bytes": raw_cap,
                },
            }),
            encoding="utf-8",
        )
        return protocol

    def _run(self, protocol, output):
        return replay.replay_two_range_reacquisition(
            protocol_path=protocol,
            data_path=FIXTURE_ROOT / "mechanism_20260727_180_12.json",
            input_manifest_path=FIXTURE_ROOT / "manifest.json",
            output_root=output,
            run_seeds=(),
            max_frames=0,
            invocation_name="smoke_a",
        )

    def _registered_records(self, root):
        protocol_path = root / "protocol.json"
        authorization_path = root / "authorization.json"
        output_root = root / "registered-root"

        def identity(path, inode):
            return {
                "path": str(path),
                "device": 1,
                "inode": inode,
                "size": 1,
                "mtime_ns": 1,
                "sha256": f"{inode:064x}",
            }

        declared_sources = {
            name: identity(root / f"{name}.source", index + 10)
            for index, name in enumerate(
                replay.REGISTERED_PROTOCOL_SOURCE_NAMES,
            )
        }
        raw_sources = {
            name: declared_sources[name]
            for name in replay.RAW_SOURCE_MEMBER_NAMES["registered_replay"]
        }
        protocol_payload = b'{"registered":"protocol"}\n'
        protocol_identity = identity(protocol_path, 2)
        protocol_identity["size"] = len(protocol_payload)
        protocol_identity["sha256"] = hashlib.sha256(
            protocol_payload,
        ).hexdigest()
        protocol = {
            "schema_id": replay.REGISTERED_PROTOCOL_SCHEMA_ID,
            "protocol_id": replay.REGISTERED_PROTOCOL_ID,
            "implementation_parent_commit": "a" * 40,
            "sources": declared_sources,
            "method_contract": {
                "synthetic_declaration_sha256": (
                    replay.SYNTHETIC_DECLARATION_SHA256
                ),
            },
            "disk_contract": copy.deepcopy(
                replay.REGISTERED_DISK_CONTRACT,
            ),
            "ranging_sigma": 0.5,
        }
        authorization_payload = b'{"registered":"authorization"}\n'
        authorization_identity = identity(authorization_path, 3)
        authorization_identity["size"] = len(authorization_payload)
        authorization_identity["sha256"] = hashlib.sha256(
            authorization_payload,
        ).hexdigest()
        authorization = {
            field: None
            for field in replay.REGISTERED_AUTHORIZATION_FIELDS
        }
        authorization.update({
            "schema_id": replay.REGISTERED_AUTHORIZATION_SCHEMA_ID,
            "protocol_id": replay.REGISTERED_PROTOCOL_ID,
            "protocol_sha256": protocol_identity["sha256"],
            "protocol_commit": "a" * 40,
            "preflight_commit": "b" * 40,
            "smoke_commit": "c" * 40,
            "user_authorization_date": "2026-07-31",
            "user_authorization_text": "authorized test record",
            "registered_replay_root": str(output_root),
            "registered_analyzer_root": replay.REGISTERED_ANALYZER_ROOT,
            "registered_retry_allowed": False,
        })
        for field in replay.REGISTERED_AUTHORIZATION_FIELDS:
            if field.endswith("_sha256") and authorization[field] is None:
                authorization[field] = "0" * 64
        authorization["user_authorization_text_sha256"] = hashlib.sha256(
            authorization["user_authorization_text"].encode(),
        ).hexdigest()
        return {
            "protocol_path": protocol_path,
            "authorization_path": authorization_path,
            "output_root": output_root,
            "protocol": protocol,
            "protocol_payload": protocol_payload,
            "protocol_identity": protocol_identity,
            "authorization": authorization,
            "authorization_payload": authorization_payload,
            "authorization_identity": authorization_identity,
            "declared_sources": declared_sources,
            "raw_sources": raw_sources,
        }

    def _assert_forensic_not_completed(self, output):
        normalized = Path(str(output).replace("/var/", "/private/var/", 1))
        candidates = []
        if normalized.is_dir():
            candidates.extend(normalized.glob("manifest*.json"))
            manifest = normalized / "manifest.json"
            if manifest.exists():
                candidates.append(manifest)
        sibling = replay._preallocation_failure_path(normalized)
        if sibling.exists():
            candidates.append(sibling)
        records = [
            json.loads(path.read_bytes())
            for path in dict.fromkeys(candidates)
        ]
        self.assertTrue(records, "failure retained no forensic manifest")
        self.assertTrue(all(record["status"] != "completed" for record in records))
        self.assertTrue(any(record["status"] == "failed" for record in records))

    def test_two_smoke_roots_have_identical_process_hashes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(str(directory).replace("/var/", "/private/var/", 1))
            protocol = self._protocol(root)
            outputs = []
            for invocation in ("smoke_a", "smoke_b"):
                outputs.append(
                    replay.replay_two_range_reacquisition(
                        protocol_path=protocol,
                        data_path=FIXTURE_ROOT
                        / "mechanism_20260727_180_12.json",
                        input_manifest_path=FIXTURE_ROOT / "manifest.json",
                        output_root=root / invocation,
                        run_seeds=(),
                        max_frames=0,
                        invocation_name=invocation,
                    )
                )
            manifests = [
                json.loads((path / "manifest.json").read_bytes())
                for path in outputs
            ]
            self.assertEqual(
                manifests[0]["process_identity"]["compressed_sha256"],
                manifests[1]["process_identity"]["compressed_sha256"],
            )
            self.assertEqual(
                manifests[0]["process_identity"]["decompressed_sha256"],
                manifests[1]["process_identity"]["decompressed_sha256"],
            )
            self.assertEqual(manifests[0]["observed_rows"], 18)
            with gzip.open(outputs[0] / replay.RAW_PROCESS_NAME, "rb") as stream:
                rows = [json.loads(line) for line in stream]
            self.assertEqual(
                [row["smoke_case_id"] for row in rows],
                list(replay.SMOKE_CASE_IDS),
            )
            self.assertEqual(
                manifests[0]["synthetic_declaration_sha256"],
                replay.SYNTHETIC_DECLARATION_SHA256,
            )

    def test_registered_source_less_protocol_and_arbitrary_authorization_fail_before_root(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            protocol = root / "protocol.json"
            protocol.write_text(
                json.dumps({
                    "schema_id": "cbf2026-two-range-reacquisition-protocol-v1",
                    "protocol_id": "arbitrary",
                    "disk_contract": {
                        "raw_bundle_max_allocated_bytes": 100_000_000,
                        "hard_floor_bytes": 0,
                    },
                }),
                encoding="utf-8",
            )
            authorization = root / "authorization.json"
            authorization.write_text("{}", encoding="utf-8")
            output = root / "registered"
            with self.assertRaises(ValueError):
                replay.replay_two_range_reacquisition(
                    protocol_path=protocol,
                    data_path=FIXTURE_ROOT
                    / "mechanism_20260727_180_12.json",
                    input_manifest_path=FIXTURE_ROOT / "manifest.json",
                    output_root=output,
                    run_seeds=tuple(range(20260727, 20260747)),
                    max_frames=500,
                    invocation_name="registered_replay",
                    authorization_json=authorization,
                )
            self.assertFalse(output.exists())

    def test_fixture_bytes_must_match_committed_fixture_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fixture = json.loads(
                (FIXTURE_ROOT / "mechanism_20260727_180_12.json").read_bytes()
            )
            fixture["expected_mechanism"]["old_candidate_count"] = 2
            altered = root / "mechanism_20260727_180_12.json"
            altered.write_text(
                json.dumps(fixture, separators=(",", ":")) + "\n",
                encoding="utf-8",
            )
            manifest = root / "manifest.json"
            manifest.write_bytes((FIXTURE_ROOT / "manifest.json").read_bytes())
            with self.assertRaises(ValueError):
                replay._load_fixture(
                    fixture_path=altered,
                    manifest_path=manifest,
                )

    def test_fixture_parse_uses_pinned_bytes_without_second_path_open(self):
        fixture_path = (
            FIXTURE_ROOT / "mechanism_20260727_180_12.json"
        ).resolve()
        manifest_path = (FIXTURE_ROOT / "manifest.json").resolve()
        real_strict_load = replay._strict_load

        def reject_fixture_reopen(path):
            if Path(path) == fixture_path:
                raise AssertionError("fixture path was reopened for parse")
            return real_strict_load(path)

        with mock.patch.object(
            replay,
            "_strict_load",
            side_effect=reject_fixture_reopen,
        ):
            fixture = replay._load_fixture(
                fixture_path=fixture_path,
                manifest_path=manifest_path,
            )
        self.assertEqual(fixture["fixture_id"], replay.MECHANISM_FIXTURE_ID)

    def test_fixture_manifest_rejects_arbitrary_design_commit(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(str(directory).replace("/var/", "/private/var/", 1))
            fixture = root / "mechanism_20260727_180_12.json"
            fixture.write_bytes(
                (FIXTURE_ROOT / fixture.name).read_bytes(),
            )
            manifest_record = json.loads(
                (FIXTURE_ROOT / "manifest.json").read_bytes(),
            )
            manifest_record["approved_design_commit"] = "f" * 40
            manifest = root / "manifest.json"
            manifest.write_text(
                json.dumps(manifest_record, separators=(",", ":")) + "\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "design commit|frozen"):
                replay._load_fixture(
                    fixture_path=fixture,
                    manifest_path=manifest,
                )

    def test_registered_disk_contract_is_exact_and_uses_two_floors(self):
        exact = {
            "launch_minimum_free_bytes": 8_000_000_000,
            "live_minimum_free_bytes": 6_000_000_000,
            "raw_bundle_max_allocated_bytes": 2_000_000_000,
            "compact_bundle_max_allocated_bytes": 10_000_000,
        }
        limits = getattr(
            replay,
            "_disk_limits",
            lambda contract, registered: (
                contract.get("hard_floor_bytes"),
                contract.get("hard_floor_bytes"),
                contract.get("raw_bundle_max_allocated_bytes"),
            ),
        )
        self.assertEqual(
            limits(exact, registered=True),
            (8_000_000_000, 6_000_000_000, 2_000_000_000),
        )
        mutations = (
            {**exact, "launch_minimum_free_bytes": 7_999_999_999},
            {**exact, "live_minimum_free_bytes": 5_999_999_999},
            {
                "hard_floor_bytes": 0,
                "raw_bundle_max_allocated_bytes": 2_000_000_000,
            },
        )
        for contract in mutations:
            with self.subTest(contract=contract):
                with self.assertRaises(ValueError):
                    limits(contract, registered=True)

    def test_launch_and_live_floors_reach_separate_boundaries(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(str(directory).replace("/var/", "/private/var/", 1))
            protocol = self._protocol(
                root,
                launch_floor=100,
                live_floor=300,
            )
            output = root / "two-stage-floors"
            launch_space = types.SimpleNamespace(
                f_bavail=2,
                f_frsize=100,
            )
            live_space = types.SimpleNamespace(
                f_bavail=10,
                f_frsize=100,
            )
            real_check = replay._check_live_resource_limits
            with mock.patch.object(
                replay.os,
                "statvfs",
                return_value=launch_space,
            ), mock.patch.object(
                replay.os,
                "fstatvfs",
                return_value=live_space,
            ), mock.patch.object(
                replay,
                "_check_live_resource_limits",
                wraps=real_check,
            ) as live_check:
                self._run(protocol, output)
            self.assertTrue(live_check.called)
            self.assertTrue(
                all(
                    call.kwargs["live_floor"] == 300
                    for call in live_check.call_args_list
                ),
            )

    def test_structurally_self_consistent_uncommitted_authorization_rejects(self):
        verifier = getattr(
            replay,
            "_validate_committed_registered_state",
            lambda **kwargs: None,
        )
        protocol_payload = b'{"protocol":"self-consistent"}\n'
        authorization_payload = b'{"authorization":"self-consistent"}\n'
        protocol_identity = {
            "path": "/repo/protocol.json",
            "device": 1,
            "inode": 2,
            "size": len(protocol_payload),
            "mtime_ns": 3,
            "sha256": hashlib.sha256(protocol_payload).hexdigest(),
        }
        authorization_identity = {
            "path": "/repo/authorization.json",
            "device": 1,
            "inode": 4,
            "size": len(authorization_payload),
            "mtime_ns": 5,
            "sha256": hashlib.sha256(authorization_payload).hexdigest(),
        }
        source_payload = b"print('pinned source')\n"
        source_identity = {
            "path": "/repo/source.py",
            "device": 1,
            "inode": 6,
            "size": len(source_payload),
            "mtime_ns": 7,
            "sha256": hashlib.sha256(source_payload).hexdigest(),
        }
        protocol = {"implementation_parent_commit": "a" * 40}
        authorization = {
            "protocol_commit": "b" * 40,
            "preflight_commit": "c" * 40,
            "smoke_commit": "d" * 40,
        }
        arguments = {
            "project_root": Path("/repo"),
            "protocol_path": Path(protocol_identity["path"]),
            "protocol_payload": protocol_payload,
            "protocol": protocol,
            "protocol_identity": protocol_identity,
            "authorization_path": Path(authorization_identity["path"]),
            "authorization_payload": authorization_payload,
            "authorization": authorization,
            "authorization_identity": authorization_identity,
            "sources": {"replay_source": source_identity},
        }

        def git_run(command, **kwargs):
            if "rev-parse" in command and command[-1] == "HEAD":
                return types.SimpleNamespace(
                    returncode=0,
                    stdout=("e" * 40 + "\n").encode(),
                )
            if "status" in command:
                return types.SimpleNamespace(returncode=0, stdout=b"")
            raise AssertionError(command)

        with mock.patch.object(
            replay,
            "_git_resolve_commit",
            create=True,
            side_effect=lambda project, commit: commit,
        ), mock.patch.object(replay.subprocess, "run", side_effect=git_run):
            cases = {
                "protocol": lambda project, commit, path: b"forged protocol",
                "authorization": lambda project, commit, path: (
                    protocol_payload
                    if Path(path) == Path(protocol_identity["path"])
                    else b"forged authorization"
                ),
                "source": lambda project, commit, path: {
                    Path(protocol_identity["path"]): protocol_payload,
                    Path(authorization_identity["path"]): authorization_payload,
                    Path(source_identity["path"]): b"forged source",
                }[Path(path)],
            }
            for name, blob_side_effect in cases.items():
                with self.subTest(name=name), mock.patch.object(
                    replay,
                    "_git_blob_at",
                    side_effect=blob_side_effect,
                ):
                    with self.assertRaisesRegex(
                        ValueError,
                        "blob|committed|source",
                    ):
                        verifier(**arguments)

    def test_dirty_authorization_related_path_rejects_before_allocation(self):
        protocol_payload = b'{"protocol":"committed"}\n'
        authorization_payload = b'{"authorization":"committed"}\n'
        source_payload = b"print('committed source')\n"
        protocol_identity = {
            "path": "/repo/protocol.json",
            "device": 1,
            "inode": 2,
            "size": len(protocol_payload),
            "mtime_ns": 3,
            "sha256": hashlib.sha256(protocol_payload).hexdigest(),
        }
        authorization_identity = {
            "path": "/repo/authorization.json",
            "device": 1,
            "inode": 4,
            "size": len(authorization_payload),
            "mtime_ns": 5,
            "sha256": hashlib.sha256(authorization_payload).hexdigest(),
        }
        source_identity = {
            "path": "/repo/source.py",
            "device": 1,
            "inode": 6,
            "size": len(source_payload),
            "mtime_ns": 7,
            "sha256": hashlib.sha256(source_payload).hexdigest(),
        }
        arguments = {
            "project_root": Path("/repo"),
            "protocol_path": Path(protocol_identity["path"]),
            "protocol_payload": protocol_payload,
            "protocol": {"implementation_parent_commit": "a" * 40},
            "protocol_identity": protocol_identity,
            "authorization_path": Path(authorization_identity["path"]),
            "authorization_payload": authorization_payload,
            "authorization": {
                "protocol_commit": "b" * 40,
                "preflight_commit": "c" * 40,
                "smoke_commit": "d" * 40,
            },
            "authorization_identity": authorization_identity,
            "sources": {"replay_source": source_identity},
        }

        def git_run(command, **kwargs):
            if "rev-parse" in command:
                return types.SimpleNamespace(
                    returncode=0,
                    stdout=("e" * 40 + "\n").encode(),
                )
            if "status" in command:
                return types.SimpleNamespace(
                    returncode=0,
                    stdout=b" M docs/diagnostics/reviews/authorization.json\n",
                )
            raise AssertionError(command)

        def git_blob(project, commit, path):
            return {
                Path(protocol_identity["path"]): protocol_payload,
                Path(authorization_identity["path"]): authorization_payload,
                Path(source_identity["path"]): source_payload,
            }[Path(path)]

        allocation = mock.Mock(side_effect=AssertionError("allocated root"))
        with mock.patch.object(
            replay,
            "_git_resolve_commit",
            side_effect=lambda project, commit: commit,
        ), mock.patch.object(
            replay,
            "_git_blob_at",
            side_effect=git_blob,
        ), mock.patch.object(
            replay.subprocess,
            "run",
            side_effect=git_run,
        ), mock.patch.object(
            replay,
            "_create_exact_root",
            allocation,
        ):
            with self.assertRaisesRegex(ValueError, "dirty"):
                replay._validate_committed_registered_state(**arguments)
        allocation.assert_not_called()

    def test_protocol_and_authorization_parse_their_single_pinned_read(self):
        identity = {
            "path": "/repo/pinned.json",
            "device": 1,
            "inode": 2,
            "size": 17,
            "mtime_ns": 3,
            "sha256": "4" * 64,
        }
        for label in ("protocol", "authorization"):
            path = Path(f"/repo/{label}.json")
            payload = json.dumps({label: "pinned"}).encode()
            observed = {**identity, "path": str(path), "size": len(payload)}
            observed["sha256"] = hashlib.sha256(payload).hexdigest()
            allocation = mock.Mock(side_effect=AssertionError("allocated root"))
            with self.subTest(label=f"{label}_single_read"), mock.patch.object(
                replay,
                "_read_trusted_bytes",
                return_value=(payload, observed),
            ) as pinned_read, mock.patch.object(
                replay,
                "_parse_json_object",
                wraps=replay._parse_json_object,
            ) as parser, mock.patch.object(
                replay,
                "_create_exact_root",
                allocation,
            ):
                record, returned_payload, returned_identity = (
                    replay._read_pinned_json(path)
                )
                self.assertEqual(record, {label: "pinned"})
                self.assertEqual(returned_payload, payload)
                self.assertEqual(returned_identity, observed)
                pinned_read.assert_called_once_with(path)
                parser.assert_called_once_with(payload, path)
                allocation.assert_not_called()

            drift_allocation = mock.Mock(
                side_effect=AssertionError("allocated root"),
            )
            with self.subTest(label=f"{label}_replacement"), mock.patch.object(
                replay,
                "_read_trusted_bytes",
                side_effect=ValueError(f"{label} identity changed during read"),
            ) as pinned_read, mock.patch.object(
                replay,
                "_parse_json_object",
                wraps=replay._parse_json_object,
            ) as parser, mock.patch.object(
                replay,
                "_create_exact_root",
                drift_allocation,
            ):
                with self.assertRaisesRegex(ValueError, "identity changed"):
                    replay._read_pinned_json(path)
                pinned_read.assert_called_once_with(path)
                parser.assert_not_called()
                drift_allocation.assert_not_called()

    def test_registered_replay_never_loads_mechanism_fixture(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(str(directory).replace("/var/", "/private/var/", 1))
            records = self._registered_records(root)
            self.assertNotIn(
                "mechanism_fixture",
                replay.RAW_SOURCE_MEMBER_NAMES["registered_replay"],
            )
            self.assertNotIn(
                "mechanism_fixture_manifest",
                replay.RAW_SOURCE_MEMBER_NAMES["registered_replay"],
            )
            events = []
            transaction = {
                "root_fd": 10,
                "parent_fd": 11,
                "resource_fds": set(),
            }
            pinned_records = {
                records["protocol_path"]: (
                    records["protocol"],
                    records["protocol_payload"],
                    records["protocol_identity"],
                ),
                records["authorization_path"]: (
                    records["authorization"],
                    records["authorization_payload"],
                    records["authorization_identity"],
                ),
            }
            identity_by_path = {
                Path(identity["path"]): identity
                for identity in records["declared_sources"].values()
            }

            def trusted_read(path, *, capture_payload=True):
                self.assertFalse(capture_payload)
                return None, identity_by_path[Path(path)]

            def allocate(path):
                events.append("allocate")
                return transaction

            def stop_registered_rows(**kwargs):
                events.append("registered_rows")
                raise self.RegisteredStop("controlled registered stop")

            filesystem = types.SimpleNamespace(
                f_bavail=100_000_000,
                f_frsize=4096,
            )
            with mock.patch.object(
                replay,
                "REGISTERED_PROTOCOL_RELATIVE_PATH",
                str(records["protocol_path"]),
            ), mock.patch.object(
                replay,
                "REGISTERED_AUTHORIZATION_RELATIVE_PATH",
                str(records["authorization_path"]),
            ), mock.patch.object(
                replay,
                "REGISTERED_REPLAY_ROOT",
                str(records["output_root"]),
            ), mock.patch.object(
                replay,
                "_read_pinned_json",
                side_effect=lambda path: pinned_records[Path(path)],
            ), mock.patch.object(
                replay,
                "_source_snapshots",
                return_value=(
                    records["raw_sources"],
                    {"truth_data": {}},
                ),
            ), mock.patch.object(
                replay,
                "_read_trusted_bytes",
                side_effect=trusted_read,
            ), mock.patch.object(
                replay,
                "_validate_committed_registered_state",
            ), mock.patch.object(
                replay.os,
                "statvfs",
                return_value=filesystem,
            ), mock.patch.object(
                replay.os,
                "fstatvfs",
                return_value=filesystem,
            ), mock.patch.object(
                replay,
                "_create_exact_root",
                side_effect=allocate,
            ), mock.patch.object(
                replay,
                "_publish_manifest",
            ), mock.patch.object(
                replay,
                "_open_process",
                return_value=12,
            ), mock.patch.object(
                replay,
                "_load_fixture",
                side_effect=AssertionError(
                    "registered replay loaded mechanism fixture",
                ),
            ) as fixture_loader, mock.patch.object(
                replay,
                "_registered_rows",
                side_effect=stop_registered_rows,
            ), mock.patch.object(
                replay,
                "_close_output_transaction",
            ):
                with self.assertRaises(self.RegisteredStop):
                    replay.replay_two_range_reacquisition(
                        protocol_path=records["protocol_path"],
                        data_path=root / "truth.json",
                        input_manifest_path=root / "manifest.json",
                        output_root=records["output_root"],
                        run_seeds=tuple(range(20260727, 20260747)),
                        max_frames=500,
                        invocation_name="registered_replay",
                        authorization_json=records["authorization_path"],
                    )
            fixture_loader.assert_not_called()
            self.assertEqual(events, ["allocate", "registered_rows"])

    def test_smoke_fixture_validation_precedes_root_allocation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(str(directory).replace("/var/", "/private/var/", 1))
            protocol = self._protocol(root)
            allocation = mock.Mock(side_effect=AssertionError("allocated root"))
            with mock.patch.object(
                replay,
                "_load_fixture",
                side_effect=ValueError("fixture binding drift"),
            ) as fixture_loader, mock.patch.object(
                replay,
                "_create_exact_root",
                allocation,
            ):
                with self.assertRaisesRegex(ValueError, "fixture binding drift"):
                    self._run(protocol, root / "smoke-root")
            fixture_loader.assert_called_once_with()
            allocation.assert_not_called()

    def test_authorized_smoke_hashes_require_existing_pinned_evidence(self):
        verifier = replay._validate_smoke_evidence_binding
        with tempfile.TemporaryDirectory() as directory:
            root = Path(str(directory).replace("/var/", "/private/var/", 1))
            protocol = {
                "invocations": {
                    name: {"output_root": str(root / name)}
                    for name in (
                        "smoke_a", "smoke_b",
                        "smoke_analyzer_a", "smoke_analyzer_b",
                    )
                },
            }
            authorization = {
                field: "0" * 64
                for field in (
                    "smoke_a_compressed_sha256",
                    "smoke_a_decompressed_sha256",
                    "smoke_b_compressed_sha256",
                    "smoke_b_decompressed_sha256",
                    "smoke_analyzer_a_json_sha256",
                    "smoke_analyzer_a_markdown_sha256",
                    "smoke_analyzer_b_json_sha256",
                    "smoke_analyzer_b_markdown_sha256",
                    "smoke_semantic_payload_sha256",
                )
            }
            with self.assertRaises((FileNotFoundError, ValueError)):
                verifier(protocol, authorization)

    def test_live_floor_is_rechecked_inside_row_loop(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            protocol = self._protocol(root, live_floor=1)
            output = root / "live-loop"
            high = types.SimpleNamespace(f_bavail=1000, f_frsize=4096)
            low = types.SimpleNamespace(f_bavail=0, f_frsize=4096)
            with mock.patch.object(
                replay.os,
                "fstatvfs",
                side_effect=[high, low],
            ):
                with self.assertRaisesRegex(OSError, "live floor"):
                    self._run(protocol, output)
            self._assert_forensic_not_completed(output)

    def test_raw_cap_is_rechecked_inside_row_loop(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw_cap = 100_000_000
            protocol = self._protocol(root, raw_cap=raw_cap)
            output = root / "cap-loop"
            with mock.patch.object(
                replay,
                "_allocated_bytes_live",
                create=True,
                return_value=raw_cap + 1,
            ):
                with self.assertRaisesRegex(OSError, "allocated-byte cap"):
                    self._run(protocol, output)
            self._assert_forensic_not_completed(output)

    def test_post_completed_rename_failure_retracts_visible_completed_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            protocol = self._protocol(root)
            output = root / "post-rename"
            real_publish = replay._publish_manifest

            def publish(transaction, manifest):
                if manifest["status"] == "completed":
                    real_fsync = replay.os.fsync

                    def fail_root_fsync(descriptor):
                        if descriptor == transaction["root_fd"]:
                            raise OSError("injected post-rename fsync failure")
                        return real_fsync(descriptor)

                    with mock.patch.object(
                        replay.os,
                        "fsync",
                        side_effect=fail_root_fsync,
                    ):
                        return real_publish(transaction, manifest)
                if manifest["status"] == "failed":
                    raise OSError("injected failed replacement failure")
                return real_publish(transaction, manifest)

            with mock.patch.object(
                replay,
                "_publish_manifest",
                side_effect=publish,
            ):
                with self.assertRaisesRegex(OSError, "post-rename"):
                    self._run(protocol, output)
            visible = output / "manifest.json"
            if visible.exists():
                self.assertNotEqual(
                    json.loads(visible.read_bytes())["status"],
                    "completed",
                )
            self._assert_forensic_not_completed(output)

    def test_start_space_failure_retains_failed_forensic_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            protocol = self._protocol(root, launch_floor=1)
            output = root / "start-space"
            zero = types.SimpleNamespace(f_bavail=0, f_frsize=4096)
            with mock.patch.object(replay.os, "statvfs", return_value=zero):
                with self.assertRaisesRegex(OSError, "start floor"):
                    self._run(protocol, output)
            self._assert_forensic_not_completed(output)

    def test_live_floor_failure_retains_failed_forensic_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            protocol = self._protocol(root, live_floor=1)
            output = root / "live-floor"
            zero = types.SimpleNamespace(f_bavail=0, f_frsize=4096)
            with mock.patch.object(replay.os, "fstatvfs", return_value=zero):
                with self.assertRaisesRegex(OSError, "live floor"):
                    self._run(protocol, output)
            self._assert_forensic_not_completed(output)

    def test_raw_cap_failure_retains_failed_forensic_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            protocol = self._protocol(root, raw_cap=0)
            output = root / "raw-cap"
            with self.assertRaisesRegex(OSError, "allocated-byte cap"):
                self._run(protocol, output)
            self._assert_forensic_not_completed(output)

    def test_preexisting_root_retains_failed_forensic_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            protocol = self._protocol(root)
            output = root / "preexisting"
            output.mkdir()
            with self.assertRaises(FileExistsError):
                self._run(protocol, output)
            self._assert_forensic_not_completed(output)

    def test_symlink_component_retains_failed_forensic_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            protocol = self._protocol(root)
            real = root / "real"
            real.mkdir()
            link = root / "link"
            link.symlink_to(real, target_is_directory=True)
            output = link / "symlink-output"
            with self.assertRaises(OSError):
                self._run(protocol, output)
            self._assert_forensic_not_completed(output)

    def test_write_failure_retains_failed_forensic_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            protocol = self._protocol(root)
            output = root / "write-failure"
            with mock.patch.object(
                replay.gzip.GzipFile,
                "write",
                side_effect=OSError("injected write failure"),
            ):
                with self.assertRaisesRegex(OSError, "injected write"):
                    self._run(protocol, output)
            self._assert_forensic_not_completed(output)

    def test_fsync_failure_retains_emergency_forensic_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            protocol = self._protocol(root)
            output = root / "fsync-failure"
            with mock.patch.object(
                replay.os,
                "fsync",
                side_effect=OSError("injected fsync failure"),
            ):
                with self.assertRaisesRegex(OSError, "injected fsync"):
                    self._run(protocol, output)
            self._assert_forensic_not_completed(output)

    def test_final_identity_mismatch_retains_failed_forensic_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            protocol = self._protocol(root)
            output = root / "identity-mismatch"
            with mock.patch.object(
                replay,
                "_verify_process_link",
                side_effect=ValueError("injected identity mismatch"),
            ):
                with self.assertRaisesRegex(ValueError, "identity mismatch"):
                    self._run(protocol, output)
            self._assert_forensic_not_completed(output)

    def test_terminal_manifest_failure_retains_failed_forensic_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            protocol = self._protocol(root)
            output = root / "terminal-failure"
            real_publish = replay._publish_manifest
            calls = 0

            def fail_completed(transaction, manifest):
                nonlocal calls
                calls += 1
                if calls == 2:
                    raise OSError("injected terminal manifest failure")
                return real_publish(transaction, manifest)

            with mock.patch.object(
                replay, "_publish_manifest", side_effect=fail_completed,
            ):
                with self.assertRaisesRegex(OSError, "terminal manifest"):
                    self._run(protocol, output)
            self._assert_forensic_not_completed(output)


if __name__ == "__main__":
    unittest.main()
