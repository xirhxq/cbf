"""One-method raw replay producer for two-range branch reacquisition."""

from __future__ import annotations

import argparse
import copy
import ctypes
import errno
import gzip
import hashlib
import importlib.machinery
import importlib.util
import json
import math
import os
import secrets
import stat
import subprocess
import sys
from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

if __package__ in (None, ""):
    _SCRIPT_REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(_SCRIPT_REPOSITORY_ROOT))
    _SCRIPT_PACKAGE_SPEC = importlib.machinery.PathFinder.find_spec(
        "scripts",
        [str(_SCRIPT_REPOSITORY_ROOT)],
    )
    if (
        _SCRIPT_PACKAGE_SPEC is None
        or _SCRIPT_PACKAGE_SPEC.submodule_search_locations is None
    ):
        raise ImportError("implementation-root scripts package is unavailable")
    sys.modules["scripts"] = importlib.util.module_from_spec(
        _SCRIPT_PACKAGE_SPEC
    )

from scripts.diagnostics.predictive_wnls import (
    ATTEMPT_STATUSES,
    INNOVATION_REFERENCE_QUANTILE,
    OUTPUT_STATUSES,
    _complete_converged_solver_result,
    candidate_acceptance,
    make_unavailable_output,
    qualify_active_references,
    reference_is_eligible,
    solve_predictive_multistart,
)
from scripts.diagnostics.replay_localization_calibration import (
    _truth_positions,
    fixed_references,
)
from scripts.diagnostics.replay_predictive_wnls_recovery import (
    _canonical_active_records,
    _close_output_transaction,
    _create_exact_root,
    _file_identity,
    _assert_registered_root,
    _lstat_components,
    _offline_metrics,
    _preflight_frames,
    _parse_json_object,
    _public_reference_arrays,
    _read_trusted_bytes,
    _sensor_records,
    _squad_local_index,
    _strict_load,
)
from scripts.diagnostics.two_range_reacquisition import (
    BRANCH_IDS,
    METHOD_ID,
    advance_two_range_prior,
    branch_gate_passes,
    finalize_two_range_lifecycle,
    solve_two_range_reacquisition,
    validate_solver_branches,
)


METHODS = (METHOD_ID,)
RAW_SCHEMA_ID = "cbf2026-two-range-reacquisition-raw-v1"
RAW_PROCESS_NAME = "two-range-reacquisition.jsonl.gz"
PRODUCER_INVOCATIONS = (
    "unit_fixture",
    "smoke_a",
    "smoke_b",
    "registered_replay",
)
ROW_INVOCATION_NAMES = (
    "unit_fixture",
    "smoke_validation",
    "registered_replay",
)
SMOKE_CASE_IDS = (
    "mechanism_20260727_180_12",
    "select_negative",
    "select_positive",
    "q_equal_threshold",
    "q_below_threshold",
    "q_above_threshold",
    "none_pass",
    "multiple_pass",
    "tangent",
    "disjoint",
    "contained",
    "coincident",
    "zero_range",
    "nearly_collinear",
    "merged_solver_branches",
    "invalid_private_covariance",
    "cost_equal_nine",
    "cost_above_nine",
)
CONSIDERATION_REASONS = (
    "considered",
    "live_public_prediction",
    "qualification_not_ok",
    "active_reference_count_not_two",
    "active_reference_not_mandatory_uav",
    "active_base_present",
    "active_optional_present",
    "fixed_reference_missing",
    "reference_not_current_fresh",
    "reference_not_strictly_lower_index",
    "range_invalid",
    "provenance_invalid",
)
SYNTHETIC_CASE_FIELDS = ("case_id", "case_kind", "input", "expected")
COMMON_SELECTOR_INPUT = {
    "selector_base_id": "symmetric_two_range_v1",
    "robot_id": 12,
    "reference_positions": [[0.0, 0.0], [2.0, 0.0]],
    "reference_covariances": [
        [[0.1, 0.0], [0.0, 0.1]],
        [[0.1, 0.0], [0.0, 0.1]],
    ],
    "measurements": [math.sqrt(2.0), math.sqrt(2.0)],
    "reference_keys": [["uav", 10], ["uav", 11]],
    "private_prior_estimate": [1.0, 1.75],
    "private_prior_covariance": [[0.2, 0.0], [0.0, 0.2]],
    "private_prior_source_fresh_frame": 20,
    "private_prior_propagated_to_frame": 20,
    "private_prior_age_frames": 0,
    "ranging_sigma": 0.5,
    "base_anchor_provenance": [0, 1],
}
CANDIDATE_TEMPLATES = {
    "canonical_spd_zero_residual_v1": {
        "status": "converged",
        "estimate": [1.0, 1.0],
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
    },
}
_COMMON_POSITIVE_RESULT = {
    "status": "converged",
    "estimate": [1.0, 1.0000000000000002],
    "covariance": [
        [0.3500000000000001, 8.05917096752434e-19],
        [8.05917096752434e-19, 0.34999999999999987],
    ],
    "epsilon": 1.774823934929885,
    "phi_min_eigenvalue": 2.8571428571428563,
    "phi_condition": 1.0000000000000007,
    "fim_valid": True,
    "proposal_count": 0,
    "iterations": 0,
    "cost": 0.0,
    "stationarity_norm": 0.0,
    "failure_reason": None,
    "proposal_trace": [],
}
SMOKE_SOLVER_OVERRIDES = {
    "both_positive_common_solution_v1": (
        copy.deepcopy(_COMMON_POSITIVE_RESULT),
        copy.deepcopy(_COMMON_POSITIVE_RESULT),
    ),
}
SMOKE_BRANCH_PAIR_RECORDS = {
    "both_positive_common_solution_v1": (
        {
            "branch_id": "circle_negative",
            "circle_start": [1.0, -1.0000000000000002],
            "solver_result": copy.deepcopy(_COMMON_POSITIVE_RESULT),
            "q_branch": None,
            "passes_branch_gate": None,
        },
        {
            "branch_id": "circle_positive",
            "circle_start": [1.0, 1.0000000000000002],
            "solver_result": copy.deepcopy(_COMMON_POSITIVE_RESULT),
            "q_branch": None,
            "passes_branch_gate": None,
        },
    ),
}


def frozen_candidate_gate_record(
    *,
    cost: float,
    accepted: bool,
    reason: str,
) -> dict:
    result = {
        **CANDIDATE_TEMPLATES["canonical_spd_zero_residual_v1"],
        "cost": cost,
    }
    diagnostics = {
        "innovation_gate": "not_applicable_reacquisition",
        "q_innov": None,
        "gate_outcome": "accepted" if accepted else "rejected",
        "valid": None,
        "failure_reason": None,
        "reduced_whitened_cost": cost,
    }
    return {
        "source": "circle_positive",
        "initial_estimate": [1.0, 1.0],
        "result": result,
        "accepted": accepted,
        "rejection_reason": None if accepted else reason,
        "gate_diagnostics": diagnostics,
        "status": result["status"],
        "estimate": result["estimate"],
        "covariance": result["covariance"],
        "cost": result["cost"],
        "q_innov": None,
    }


CANDIDATE_GATE_RECORDS = {
    "cost_equal_nine": frozen_candidate_gate_record(
        cost=9.0, accepted=True, reason="accepted",
    ),
    "cost_above_nine": frozen_candidate_gate_record(
        cost=np.nextafter(9.0, np.inf).item(),
        accepted=False,
        reason="reacquisition_reduced_cost_exceeds_nine",
    ),
}


def _selector_case(
    case_id: str,
    overrides: dict,
    attempt_status: str,
    failure_reason: str | None,
    selected_branch_id: str | None,
    score_outcome: str,
    prior_used: bool,
) -> dict:
    return {
        "case_id": case_id,
        "case_kind": "selector",
        "input": {
            "selector_base_id": "symmetric_two_range_v1",
            "overrides": overrides,
            "solver_override_id": None,
        },
        "expected": {
            "attempt_status": attempt_status,
            "failure_reason": failure_reason,
            "selected_branch_id": selected_branch_id,
            "score_outcome": score_outcome,
            "prior_used_for_branch_selection": prior_used,
        },
    }


SYNTHETIC_CASES = (
    _selector_case(
        "select_negative", {"private_prior_estimate": [1.0, -1.75]},
        "accepted", None, "circle_negative", "exactly_one", True,
    ),
    _selector_case(
        "select_positive", {}, "accepted", None, "circle_positive",
        "exactly_one", True,
    ),
    *(
        {
            "case_id": case_id,
            "case_kind": "branch_gate",
            "input": {"q_source": q_source},
            "expected": {"passes": passes},
        }
        for case_id, q_source, passes in (
            ("q_equal_threshold", "threshold", True),
            ("q_below_threshold", "nextafter_down", True),
            ("q_above_threshold", "nextafter_up", False),
        )
    ),
    _selector_case(
        "none_pass", {"private_prior_estimate": [10.0, 0.0]},
        "rejected", "two_range_no_branch_passes", None, "none", True,
    ),
    _selector_case(
        "multiple_pass", {"private_prior_estimate": [1.0, 0.0]},
        "rejected", "two_range_multiple_branches_pass", None, "multiple", True,
    ),
    *(
        _selector_case(
            case_id, overrides, "rejected", reason, None,
            "not_evaluated", False,
        )
        for case_id, overrides, reason in (
            ("tangent", {"measurements": [1.0, 1.0]},
             "two_range_circle_starts_not_distinct"),
            ("disjoint", {"measurements": [0.5, 0.5]},
             "two_range_circle_geometry_invalid"),
            ("contained", {"measurements": [5.0, 1.0]},
             "two_range_circle_geometry_invalid"),
            ("coincident", {
                "reference_positions": [[0.0, 0.0], [0.0, 0.0]],
                "measurements": [1.0, 1.0],
            }, "two_range_circle_geometry_invalid"),
            ("zero_range", {"measurements": [0.0, 2.0]},
             "two_range_input_invalid"),
            ("nearly_collinear", {
                "measurements": [
                    math.hypot(1.0, 1e-7),
                    math.hypot(1.0, 1e-7),
                ],
            }, "two_range_branch_solver_invalid"),
        )
    ),
    {
        "case_id": "merged_solver_branches",
        "case_kind": "branch_pair_gate",
        "input": {"solver_override_id": "both_positive_common_solution_v1"},
        "expected": {
            "valid": False,
            "reason": "two_range_solver_branches_merged",
        },
    },
    _selector_case(
        "invalid_private_covariance",
        {"private_prior_covariance": [[1.0, 2.0], [2.0, 1.0]]},
        "rejected", "two_range_private_prior_invalid", None,
        "not_evaluated", False,
    ),
    *(
        {
            "case_id": case_id,
            "case_kind": "candidate_gate",
            "input": {
                "candidate_template_id": "canonical_spd_zero_residual_v1",
                "cost_source": cost_source,
            },
            "expected": {"accepted": accepted, "reason": reason},
        }
        for case_id, cost_source, accepted, reason in (
            ("cost_equal_nine", "nine", True, "accepted"),
            (
                "cost_above_nine",
                "nextafter_nine_up",
                False,
                "reacquisition_reduced_cost_exceeds_nine",
            ),
        )
    ),
)
SELECTOR_SMOKE_INPUT_FIELDS = (
    "selector_base_id", "overrides", "solver_override_id",
)
SELECTOR_SMOKE_EXPECTED_FIELDS = (
    "attempt_status", "failure_reason", "selected_branch_id",
    "score_outcome", "prior_used_for_branch_selection",
)
BRANCH_GATE_SMOKE_INPUT_FIELDS = ("q_source",)
BRANCH_GATE_SMOKE_EXPECTED_FIELDS = ("passes",)
BRANCH_PAIR_GATE_SMOKE_INPUT_FIELDS = ("solver_override_id",)
BRANCH_PAIR_GATE_SMOKE_EXPECTED_FIELDS = ("valid", "reason")
CANDIDATE_GATE_SMOKE_INPUT_FIELDS = ("candidate_template_id", "cost_source")
CANDIDATE_GATE_SMOKE_EXPECTED_FIELDS = ("accepted", "reason")
MECHANISM_SMOKE_INPUT_FIELDS = ("fixture_id", "fixture_sha256")
MECHANISM_SMOKE_EXPECTED_FIELDS = (
    "selector_considered", "active_reference_count",
    "active_reference_keys", "old_failure_reason",
    "two_branches_required", "favorable_new_outcome_required",
)
MECHANISM_SMOKE_EXPECTED = {
    "selector_considered": True,
    "active_reference_count": 2,
    "active_reference_keys": [["uav", 10], ["uav", 11]],
    "old_failure_reason": "reacquisition_requires_three_active_references",
    "two_branches_required": True,
    "favorable_new_outcome_required": False,
}
ROW_FIELDS = (
    "method", "invocation_name", "smoke_case_id", "smoke_case_kind",
    "smoke_case_input", "smoke_case_expected", "seed", "frame_index",
    "robot_id", "squad_local_index", "applied_command_source_frame",
    "applied_command", "attempt_path", "selector_considered",
    "selector_consideration_reason", "attempt_status",
    "attempt_failure_reason", "output_status", "prediction_age", "estimate",
    "fresh_modeled_covariance", "fresh_epsilon", "aged_modeled_covariance",
    "aged_modeled_radius", "branch_selection_prior_status",
    "branch_selection_prior_estimate", "branch_selection_prior_covariance",
    "branch_selection_prior_source_fresh_frame",
    "branch_selection_prior_propagated_to_frame",
    "branch_selection_prior_age_frames", "prior_used_for_branch_selection",
    "prior_used_in_fim", "prior_used_for_continuous_update", "branches",
    "selected_branch_id", "next_private_state_status",
    "next_private_state_estimate", "next_private_state_covariance",
    "next_private_state_source_fresh_frame",
    "next_private_state_propagated_to_frame",
    "next_private_state_age_frames", "existing_candidates",
    "existing_selected_candidate_source", "attempt_base_anchor_provenance",
    "base_anchor_provenance", "mandatory_references",
    "optional_candidates", "active_references", "reference_evidence",
    "reference_freshness", "excluded_references", "reference_violations",
    "offline_truth_position", "offline_error_norm",
    "offline_fresh_containment", "offline_aged_radius_containment",
    "offline_fresh_q_error", "offline_aged_q_error",
)
BRANCH_FIELDS = (
    "branch_id", "circle_start", "solver_result", "q_branch",
    "passes_branch_gate",
)
SOLVER_RESULT_FIELDS = (
    "status", "estimate", "covariance", "epsilon",
    "phi_min_eigenvalue", "phi_condition", "fim_valid", "proposal_count",
    "iterations", "cost", "stationarity_norm", "failure_reason",
    "proposal_trace",
)
PROPOSAL_TRACE_FIELDS = (
    "proposal", "damping", "cost", "stationarity_norm", "raw_step_norm",
    "trial_cost", "invalid_trial_reason", "accepted",
)
EXISTING_CANDIDATE_FIELDS = (
    "source", "initial_estimate", "result", "accepted", "rejection_reason",
    "gate_diagnostics", "status", "estimate", "covariance", "cost",
    "q_innov",
)
GATE_DIAGNOSTIC_FIELDS = (
    "innovation_gate", "q_innov", "gate_outcome", "valid",
    "failure_reason", "reduced_whitened_cost",
)
REFERENCE_EVIDENCE_FIELDS = (
    "reference_kind", "reference_id", "role", "measurement_present",
    "noisy_range", "noise_seed", "current_freshness", "eligible", "used",
    "exclusion_reason", "base_anchor_provenance",
)
REFERENCE_FRESHNESS_FIELDS = (
    "reference_kind", "reference_id", "current_freshness",
)
MANDATORY_REFERENCE_FIELDS = ("base_ids", "uav_ids")
REFERENCE_KEY_FIELDS = ("reference_kind", "reference_id")
EXCLUSION_FIELDS = ("reference_kind", "reference_id", "reason")
VIOLATION_FIELDS = ("reference_kind", "reference_id", "reason")
PRIVATE_PRIOR_FIELDS = (
    "branch_selection_prior_status", "branch_selection_prior_estimate",
    "branch_selection_prior_covariance",
    "branch_selection_prior_source_fresh_frame",
    "branch_selection_prior_propagated_to_frame",
    "branch_selection_prior_age_frames",
)
NEXT_PRIVATE_STATE_FIELDS = (
    "next_private_state_status", "next_private_state_estimate",
    "next_private_state_covariance",
    "next_private_state_source_fresh_frame",
    "next_private_state_propagated_to_frame",
    "next_private_state_age_frames",
)
RAW_MANIFEST_FIELDS = (
    "schema_id", "protocol_id", "invocation_name", "status", "method",
    "output_root", "protocol_identity", "source_identities",
    "authorization_identity", "process_identity",
    "synthetic_declaration_sha256", "expected_rows",
    "observed_rows", "key_contract", "disk_contract", "started_at",
    "completed_at", "error",
)
FIXTURE_MANIFEST_FIELDS = (
    "schema_id", "fixture_id", "fixture_file", "fixture_size",
    "fixture_sha256", "source_sha256", "extractor_identity",
    "approved_design_commit",
)
FIXTURE_MANIFEST_SCHEMA_ID = (
    "cbf2026-two-range-reacquisition-fixture-manifest-v1"
)
FIXTURE_SCHEMA_ID = "cbf2026-two-range-reacquisition-fixture-v1"
MECHANISM_FIXTURE_ID = "mechanism_20260727_180_12"
MECHANISM_FIXTURE_KEY = (20260727, 180, 12)
MECHANISM_FIXTURE_SQUAD_LOCAL_INDEX = 5
MECHANISM_FIXTURE_SHA256 = (
    "9febff2393017b7b0fbd1a02dd76a13d1d70639f2b176df236931fe29a8601c7"
)
APPROVED_DESIGN_COMMIT = "20a61aad96af35ee7e16434fab0a5edaaea38ef0"
REGISTERED_PROTOCOL_SCHEMA_ID = (
    "cbf2026-two-range-reacquisition-protocol-v1"
)
REGISTERED_AUTHORIZATION_SCHEMA_ID = (
    "cbf2026-two-range-reacquisition-registration-v1"
)
REGISTERED_AUTHORIZATION_FIELDS = (
    "schema_id", "protocol_id", "protocol_sha256", "protocol_commit",
    "preflight_commit", "smoke_commit", "smoke_a_compressed_sha256",
    "smoke_a_decompressed_sha256", "smoke_b_compressed_sha256",
    "smoke_b_decompressed_sha256", "smoke_analyzer_a_json_sha256",
    "smoke_analyzer_a_markdown_sha256", "smoke_analyzer_b_json_sha256",
    "smoke_analyzer_b_markdown_sha256", "smoke_semantic_payload_sha256",
    "user_authorization_date", "user_authorization_text",
    "user_authorization_text_sha256", "registered_replay_root",
    "registered_analyzer_root", "registered_retry_allowed",
)
REGISTERED_PROTOCOL_ID = "cbf2026-two-range-reacquisition-v1"
REGISTERED_REPLAY_ROOT = (
    "/private/tmp/cbf2026-two-range-reacquisition-development/v1"
)
REGISTERED_ANALYZER_ROOT = (
    "/private/tmp/cbf2026-two-range-reacquisition-analysis/v1"
)
REGISTERED_PROTOCOL_RELATIVE_PATH = (
    "docs/diagnostics/"
    "2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json"
)
REGISTERED_AUTHORIZATION_RELATIVE_PATH = (
    "docs/diagnostics/reviews/"
    "2026-07-30-cbf2026-two-range-reacquisition-"
    "registered-authorization.json"
)
REGISTERED_PREFLIGHT_REVIEW_RELATIVE_PATH = (
    "docs/diagnostics/reviews/"
    "2026-07-30-cbf2026-two-range-reacquisition-protocol-v1-review.md"
)
REGISTERED_SMOKE_REPORT_RELATIVE_PATH = (
    "docs/diagnostics/"
    "2026-07-30-cbf2026-two-range-reacquisition-smoke.md"
)
REGISTERED_PROTOCOL_SOURCE_NAMES = (
    "implementation_plan", "two_range_reacquisition_source",
    "predictive_wnls_source", "fixture_extractor_source", "replay_source",
    "analyzer_source", "registrar_source", "mechanism_fixture",
    "mechanism_fixture_manifest", "truth_data", "input_manifest",
)
SOLVER_STATUSES = ("converged", "invalid", "failed")
REGISTERED_DISK_CONTRACT = {
    "launch_minimum_free_bytes": 8_000_000_000,
    "live_minimum_free_bytes": 6_000_000_000,
    "raw_bundle_max_allocated_bytes": 2_000_000_000,
    "compact_bundle_max_allocated_bytes": 10_000_000,
}
FILE_IDENTITY_FIELDS = (
    "path", "device", "inode", "size", "mtime_ns", "sha256",
)
PROCESS_IDENTITY_FIELDS = (
    "path", "device", "inode", "size", "allocated_bytes", "mtime_ns",
    "compressed_sha256", "decompressed_sha256",
)
RAW_KEY_CONTRACT_FIELDS = (
    "method", "seeds", "frames", "robots", "smoke_case_ids", "order",
)
RAW_SOURCE_MEMBER_NAMES = {
    invocation: (
        "two_range_reacquisition_source", "predictive_wnls_source",
        "replay_source", "mechanism_fixture", "mechanism_fixture_manifest",
    )
    for invocation in ("unit_fixture", "smoke_a", "smoke_b")
}
RAW_SOURCE_MEMBER_NAMES["registered_replay"] = (
    "two_range_reacquisition_source", "predictive_wnls_source",
    "replay_source", "truth_data", "input_manifest",
)
_SOURCE_SUFFIXES = {
    "two_range_reacquisition_source": (
        "scripts/diagnostics/two_range_reacquisition.py"
    ),
    "predictive_wnls_source": "scripts/diagnostics/predictive_wnls.py",
    "replay_source": (
        "scripts/diagnostics/replay_two_range_reacquisition.py"
    ),
    "mechanism_fixture": (
        "tests/fixtures/cbf2026_two_range_reacquisition/"
        "mechanism_20260727_180_12.json"
    ),
    "mechanism_fixture_manifest": (
        "tests/fixtures/cbf2026_two_range_reacquisition/manifest.json"
    ),
    "input_manifest": "manifest.json",
}
ROW_SCALAR_CONTRACTS = {
    "method": f"literal:{METHOD_ID}",
    "invocation_name": "enum:ROW_INVOCATION_NAMES",
    "smoke_case_id": "nullable:enum:SMOKE_CASE_IDS",
    "smoke_case_kind": (
        "nullable:enum:mechanism_fixture,selector,"
        "branch_gate,branch_pair_gate,candidate_gate"
    ),
    "seed": "nullable:int:20260727:20260746",
    "frame_index": "nullable:int:0:499",
    "robot_id": "nullable:int:1:14",
    "squad_local_index": "nullable:int:1:7",
    "applied_command_source_frame": "nullable:int:0:498",
    "attempt_path": (
        "enum:reference_unavailable,"
        "existing_predictive_multistart,two_range_reacquisition,"
        "smoke_branch_gate,smoke_branch_pair_gate,"
        "smoke_candidate_gate"
    ),
    "selector_considered": "bool",
    "selector_consideration_reason": (
        "enum:CONSIDERATION_REASONS|smoke_component_case"
    ),
    "attempt_status": "enum:ATTEMPT_STATUSES",
    "attempt_failure_reason": "nullable:str",
    "output_status": "enum:OUTPUT_STATUSES",
    "prediction_age": "nullable:int:0:2",
    "fresh_epsilon": "nullable:finite_nonnegative",
    "aged_modeled_radius": "nullable:finite_nonnegative",
    "branch_selection_prior_status": "enum:available,absent",
    "branch_selection_prior_source_fresh_frame": "nullable:int:0:499",
    "branch_selection_prior_propagated_to_frame": "nullable:int:0:499",
    "branch_selection_prior_age_frames": "nullable:int:0:499",
    "prior_used_for_branch_selection": "bool",
    "prior_used_in_fim": "literal:false",
    "prior_used_for_continuous_update": "literal:false",
    "selected_branch_id": "nullable:enum:BRANCH_IDS",
    "next_private_state_status": "enum:available,absent",
    "next_private_state_source_fresh_frame": "nullable:int:0:499",
    "next_private_state_propagated_to_frame": "nullable:int:0:499",
    "next_private_state_age_frames": "nullable:int:0:499",
    "existing_selected_candidate_source": "nullable:str",
    "offline_error_norm": "nullable:finite_nonnegative",
    "offline_fresh_containment": "nullable:bool",
    "offline_aged_radius_containment": "nullable:bool",
    "offline_fresh_q_error": "nullable:finite_nonnegative",
    "offline_aged_q_error": "nullable:finite_nonnegative",
}
ROW_ARRAY_CONTRACTS = {
    "smoke_case_input": (
        "nullable:exact_case_object:"
        "SELECTOR_SMOKE_INPUT_FIELDS|BRANCH_GATE_SMOKE_INPUT_FIELDS|"
        "BRANCH_PAIR_GATE_SMOKE_INPUT_FIELDS|"
        "CANDIDATE_GATE_SMOKE_INPUT_FIELDS|MECHANISM_SMOKE_INPUT_FIELDS"
    ),
    "smoke_case_expected": (
        "nullable:exact_case_object:"
        "SELECTOR_SMOKE_EXPECTED_FIELDS|"
        "BRANCH_GATE_SMOKE_EXPECTED_FIELDS|"
        "BRANCH_PAIR_GATE_SMOKE_EXPECTED_FIELDS|"
        "CANDIDATE_GATE_SMOKE_EXPECTED_FIELDS|"
        "MECHANISM_SMOKE_EXPECTED_FIELDS"
    ),
    "applied_command": "nullable:finite_vec2",
    "estimate": "nullable:finite_vec2",
    "fresh_modeled_covariance": "nullable:finite_spd_2x2",
    "aged_modeled_covariance": "nullable:finite_spd_2x2",
    "branch_selection_prior_estimate": "nullable:finite_vec2",
    "branch_selection_prior_covariance": "nullable:finite_spd_2x2",
    "branches": "list:strict_object:BRANCH_FIELDS",
    "next_private_state_estimate": "nullable:finite_vec2",
    "next_private_state_covariance": "nullable:finite_spd_2x2",
    "existing_candidates": "list:strict_object:EXISTING_CANDIDATE_FIELDS",
    "attempt_base_anchor_provenance": "sorted_unique_list:int_nonnegative",
    "base_anchor_provenance": "sorted_unique_list:int_nonnegative",
    "mandatory_references": "strict_object:MANDATORY_REFERENCE_FIELDS",
    "optional_candidates": "list:strict_object:REFERENCE_KEY_FIELDS",
    "active_references": "list:strict_object:REFERENCE_KEY_FIELDS",
    "reference_evidence": "list:strict_object:REFERENCE_EVIDENCE_FIELDS",
    "reference_freshness": "list:strict_object:REFERENCE_FRESHNESS_FIELDS",
    "excluded_references": "list:strict_object:EXCLUSION_FIELDS",
    "reference_violations": "list:strict_object:VIOLATION_FIELDS",
    "offline_truth_position": "nullable:finite_vec2",
}
BRANCH_FIELD_CONTRACTS = {
    "branch_id": "enum:BRANCH_IDS",
    "circle_start": "finite_vec2",
    "solver_result": "strict_object:SOLVER_RESULT_FIELDS",
    "q_branch": "nullable:finite_nonnegative",
    "passes_branch_gate": "nullable:bool",
}
if set(ROW_SCALAR_CONTRACTS).isdisjoint(ROW_ARRAY_CONTRACTS) is False:
    raise RuntimeError("row scalar/array contracts overlap")
if set(ROW_SCALAR_CONTRACTS) | set(ROW_ARRAY_CONTRACTS) != set(ROW_FIELDS):
    raise RuntimeError("row contracts do not cover exact row fields")
if set(BRANCH_FIELD_CONTRACTS) != set(BRANCH_FIELDS):
    raise RuntimeError("branch contracts do not cover exact branch fields")
if tuple(case["case_id"] for case in SYNTHETIC_CASES) != SMOKE_CASE_IDS[1:]:
    raise RuntimeError("synthetic smoke case IDs differ from declaration")
if any(tuple(case) != SYNTHETIC_CASE_FIELDS for case in SYNTHETIC_CASES):
    raise RuntimeError("synthetic smoke case fields differ from declaration")

SYNTHETIC_DECLARATION_FIELDS = (
    "common_selector_input", "candidate_templates",
    "smoke_solver_overrides", "smoke_branch_pair_records",
    "candidate_gate_records", "synthetic_cases",
)
SYNTHETIC_DECLARATION = {
    "common_selector_input": COMMON_SELECTOR_INPUT,
    "candidate_templates": CANDIDATE_TEMPLATES,
    "smoke_solver_overrides": SMOKE_SOLVER_OVERRIDES,
    "smoke_branch_pair_records": SMOKE_BRANCH_PAIR_RECORDS,
    "candidate_gate_records": CANDIDATE_GATE_RECORDS,
    "synthetic_cases": SYNTHETIC_CASES,
}
def _native_json_value(value: object) -> object:
    if isinstance(value, (np.ndarray, np.generic)):
        raise ValueError("value is not native strict JSON")
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, int) and not isinstance(value, bool):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("JSON numbers must be finite")
        return value
    if isinstance(value, (list, tuple)):
        return [_native_json_value(item) for item in value]
    if isinstance(value, Mapping):
        if any(not isinstance(key, str) for key in value):
            raise ValueError("JSON object keys must be strings")
        return {
            key: _native_json_value(item)
            for key, item in value.items()
        }
    raise ValueError("value is not native strict JSON")


def ordered_strict_json_bytes(
    value: Mapping,
    fields: tuple[str, ...],
) -> bytes:
    if tuple(value) != fields:
        raise ValueError("object differs from declared field order")
    native = _native_json_value(value)
    return json.dumps(
        native,
        allow_nan=False,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=False,
    ).encode("utf-8")


SYNTHETIC_DECLARATION_BYTES = ordered_strict_json_bytes(
    SYNTHETIC_DECLARATION, SYNTHETIC_DECLARATION_FIELDS,
)
SYNTHETIC_DECLARATION_SHA256 = hashlib.sha256(
    SYNTHETIC_DECLARATION_BYTES,
).hexdigest()


def selector_consideration(
    *,
    robot_id: int,
    live_prediction: object,
    mandatory: Mapping,
    optional_keys: object,
    qualification: Mapping,
) -> tuple[bool, str]:
    if live_prediction is not None:
        return False, "live_public_prediction"
    if not isinstance(qualification, Mapping) or qualification.get("status") != "ok":
        return False, "qualification_not_ok"
    active_keys = [
        tuple(key) for key in qualification.get("active_keys", ())
        if isinstance(key, (list, tuple)) and len(key) == 2
    ]
    if len(active_keys) != 2:
        return False, "active_reference_count_not_two"
    if any(key[0] == "base" for key in active_keys):
        return False, "active_base_present"
    if optional_keys not in ([], ()):
        return False, "active_optional_present"
    if (
        not isinstance(mandatory, Mapping)
        or mandatory.get("base_ids") != []
        or not isinstance(mandatory.get("uav_ids"), list)
    ):
        return False, "active_reference_not_mandatory_uav"
    fixed_keys = [("uav", int(identifier)) for identifier in mandatory["uav_ids"]]
    if active_keys != fixed_keys or any(key[0] != "uav" for key in active_keys):
        return False, "active_reference_not_mandatory_uav"
    fixed_outputs = qualification.get("fixed_outputs")
    if not isinstance(fixed_outputs, Mapping) or any(
        identifier not in fixed_outputs for _, identifier in fixed_keys
    ):
        return False, "fixed_reference_missing"
    if any(
        not reference_is_eligible(fixed_outputs[identifier])
        or fixed_outputs[identifier].get("output_status") != "fresh"
        or fixed_outputs[identifier].get("prediction_age") != 0
        for _, identifier in fixed_keys
    ):
        return False, "reference_not_current_fresh"
    if any(identifier >= robot_id for _, identifier in fixed_keys):
        return False, "reference_not_strictly_lower_index"
    records = qualification.get("active_records")
    if not isinstance(records, list) or len(records) != 2 or any(
        not isinstance(record, Mapping)
        or isinstance(record.get("noisy_range"), bool)
        or not isinstance(record.get("noisy_range"), (int, float, np.number))
        or not math.isfinite(float(record["noisy_range"]))
        or float(record["noisy_range"]) <= 0.0
        for record in records
    ):
        return False, "range_invalid"
    provenance = qualification.get("base_anchor_provenance")
    if (
        not isinstance(provenance, (list, tuple))
        or any(
            isinstance(value, bool)
            or not isinstance(value, (int, np.integer))
            or value < 0
            for value in provenance
        )
        or len(set(int(value) for value in provenance)) < 2
    ):
        return False, "provenance_invalid"
    return True, "considered"


def _ordered_source(source: object, fields: tuple[str, ...]) -> dict:
    if not isinstance(source, Mapping) or set(source) != set(fields):
        raise ValueError("nested object differs from exact schema")
    return {
        field: _builder_json_value(source[field])
        for field in fields
    }


def _builder_json_value(value: object) -> object:
    if isinstance(value, np.ndarray):
        return [_builder_json_value(item) for item in value.tolist()]
    if isinstance(value, np.generic):
        return _builder_json_value(value.item())
    if isinstance(value, Mapping):
        if any(not isinstance(key, str) for key in value):
            raise ValueError("builder object keys must be strings")
        return {
            key: _builder_json_value(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_builder_json_value(item) for item in value]
    return _native_json_value(value)


def _solver_result(source: object) -> dict:
    result = _ordered_source(source, SOLVER_RESULT_FIELDS)
    if result["status"] != "converged":
        for field in (
            "covariance",
            "epsilon",
            "phi_min_eigenvalue",
            "phi_condition",
        ):
            result[field] = None
    traces = result["proposal_trace"]
    if not isinstance(traces, list):
        raise ValueError("proposal trace must be a list")
    result["proposal_trace"] = [
        _ordered_source(trace, PROPOSAL_TRACE_FIELDS)
        for trace in traces
    ]
    return result


def _branch(source: object) -> dict:
    branch = _ordered_source(source, BRANCH_FIELDS)
    branch["solver_result"] = _solver_result(branch["solver_result"])
    return branch


def _gate_diagnostics(source: object) -> dict:
    required = {"innovation_gate", "q_innov", "gate_outcome"}
    allowed = set(GATE_DIAGNOSTIC_FIELDS)
    if (
        not isinstance(source, Mapping)
        or not required.issubset(source)
        or not set(source).issubset(allowed)
    ):
        raise ValueError("gate diagnostics differ from discriminated schema")
    if ("valid" in source) != ("failure_reason" in source):
        raise ValueError("gate valid/failure_reason must appear together")
    result = {
        field: _builder_json_value(source.get(field))
        for field in GATE_DIAGNOSTIC_FIELDS
    }
    innovation_gate = result["innovation_gate"]
    gate_outcome = result["gate_outcome"]
    if innovation_gate not in {
        None, "applied", "not_applicable_reacquisition",
    } or gate_outcome not in {"invalid", "rejected", "accepted"}:
        raise ValueError("gate diagnostics contain an unfrozen enum")
    for field in ("q_innov", "reduced_whitened_cost"):
        value = result[field]
        if value is not None and not _finite_number(
            value,
            nonnegative=True,
        ):
            raise ValueError("gate diagnostic numeric field is invalid")
    if result["q_innov"] is not None and result["innovation_gate"] != "applied":
        raise ValueError("innovation score appears without innovation gate")
    if (
        (result["valid"] is not None or result["failure_reason"] is not None)
        and result["innovation_gate"] != "applied"
    ):
        raise ValueError("innovation result appears outside evaluation")
    if (
        result["reduced_whitened_cost"] is not None
        and result["innovation_gate"] != "not_applicable_reacquisition"
    ):
        raise ValueError("reacquisition cost appears outside reacquisition")
    if innovation_gate is None and (
        gate_outcome != "invalid"
        or any(
            result[field] is not None
            for field in (
                "q_innov", "valid", "failure_reason",
                "reduced_whitened_cost",
            )
        )
    ):
        raise ValueError("unevaluated gate diagnostics differ")
    if innovation_gate == "applied":
        if "valid" not in source or result["reduced_whitened_cost"] is not None:
            raise ValueError("innovation evaluation is incomplete")
        valid = result["valid"]
        if not isinstance(valid, bool):
            raise ValueError("innovation validity is not Boolean")
        if valid:
            if (
                result["q_innov"] is None
                or result["failure_reason"] is not None
                or gate_outcome not in {"accepted", "rejected"}
            ):
                raise ValueError("valid innovation gate null semantics differ")
        elif (
            result["q_innov"] is not None
            or not isinstance(result["failure_reason"], str)
            or not result["failure_reason"]
            or gate_outcome != "invalid"
        ):
            raise ValueError("invalid innovation gate null semantics differ")
    elif innovation_gate == "not_applicable_reacquisition" and (
        result["q_innov"] is not None
        or result["valid"] is not None
        or result["failure_reason"] is not None
        or gate_outcome not in {"accepted", "rejected"}
        or (
            gate_outcome == "accepted"
            and result["reduced_whitened_cost"] is None
        )
    ):
        raise ValueError("reacquisition gate null semantics differ")
    return result


def _existing_candidate(source: object) -> dict:
    candidate = _ordered_source(source, EXISTING_CANDIDATE_FIELDS)
    candidate["result"] = _solver_result(candidate["result"])
    candidate["gate_diagnostics"] = _gate_diagnostics(
        candidate["gate_diagnostics"],
    )
    for field in ("status", "estimate", "covariance", "cost"):
        if candidate[field] != candidate["result"][field]:
            raise ValueError("candidate summary differs from solver result")
    return candidate


def _private_fields(prefix: str, state: object) -> dict:
    stem = (
        "branch_selection_prior"
        if prefix == "branch_selection_prior"
        else "next_private_state"
    )
    if state is None:
        return {
            f"{stem}_status": "absent",
            f"{stem}_estimate": None,
            f"{stem}_covariance": None,
            f"{stem}_source_fresh_frame": None,
            f"{stem}_propagated_to_frame": None,
            f"{stem}_age_frames": None,
        }
    if not isinstance(state, Mapping) or state.get("status") != "available":
        raise ValueError("private state must be tagged available or absent")
    return {
        f"{stem}_status": "available",
        f"{stem}_estimate": _builder_json_value(state["estimate"]),
        f"{stem}_covariance": _builder_json_value(
            state["modeled_covariance"],
        ),
        f"{stem}_source_fresh_frame": int(state["source_fresh_frame"]),
        f"{stem}_propagated_to_frame": int(state["propagated_to_frame"]),
        f"{stem}_age_frames": int(state["age_frames"]),
    }


def _reference_key(key: object) -> dict:
    if not isinstance(key, (list, tuple)) or len(key) != 2:
        raise ValueError("reference key must be a pair")
    return {"reference_kind": str(key[0]), "reference_id": int(key[1])}


def _base_row() -> dict:
    return {field: None for field in ROW_FIELDS}


def _assemble_row(
    *,
    invocation_name: str,
    smoke_case_id: str | None,
    smoke_case_kind: str | None,
    smoke_case_input: object,
    smoke_case_expected: object,
    seed: int | None,
    frame_index: int | None,
    robot_id: int | None,
    squad_local_index: int | None,
    command_source: int | None,
    command: object,
    attempt_path: str,
    considered: bool,
    consideration_reason: str,
    attempt: Mapping,
    prior: object,
    lifecycle: Mapping,
    mandatory: Mapping,
    optional: list,
    active_keys: list,
    provenance: object,
    truth: object = None,
    reference_evidence: list | None = None,
    reference_freshness: list | None = None,
    excluded: list | None = None,
    violations: list | None = None,
) -> dict:
    output = lifecycle["public_output"]
    row = _base_row()
    row.update({
        "method": METHOD_ID,
        "invocation_name": invocation_name,
        "smoke_case_id": smoke_case_id,
        "smoke_case_kind": smoke_case_kind,
        "smoke_case_input": _builder_json_value(smoke_case_input),
        "smoke_case_expected": _builder_json_value(smoke_case_expected),
        "seed": seed,
        "frame_index": frame_index,
        "robot_id": robot_id,
        "squad_local_index": squad_local_index,
        "applied_command_source_frame": command_source,
        "applied_command": _builder_json_value(command),
        "attempt_path": attempt_path,
        "selector_considered": considered,
        "selector_consideration_reason": consideration_reason,
        "attempt_status": attempt["attempt_status"],
        "attempt_failure_reason": attempt.get("failure_reason"),
        "output_status": output["output_status"],
        "prediction_age": output.get("prediction_age"),
        "estimate": _builder_json_value(output.get("estimate")),
        "fresh_modeled_covariance": (
            _builder_json_value(output.get("modeled_covariance"))
            if output["output_status"] == "fresh" else None
        ),
        "fresh_epsilon": (
            output.get("epsilon") if output["output_status"] == "fresh"
            else None
        ),
        "aged_modeled_covariance": (
            _builder_json_value(output.get("modeled_covariance"))
            if output["output_status"] == "predicted" else None
        ),
        "aged_modeled_radius": (
            output.get("aged_modeled_radius")
            if output["output_status"] == "predicted" else None
        ),
        **_private_fields("branch_selection_prior", prior),
        "prior_used_for_branch_selection": bool(
            attempt.get("prior_used_for_branch_selection", False)
        ),
        "prior_used_in_fim": False,
        "prior_used_for_continuous_update": False,
        "branches": [_branch(item) for item in attempt.get("branches", [])],
        "selected_branch_id": attempt.get("selected_branch_id"),
        **_private_fields(
            "next_private_state", lifecycle.get("next_private_state"),
        ),
        "existing_candidates": [
            _existing_candidate(item)
            for item in attempt.get("candidates", [])
        ],
        "existing_selected_candidate_source": (
            (attempt.get("selected_candidate") or {}).get("source")
        ),
        "attempt_base_anchor_provenance": _builder_json_value(provenance),
        "base_anchor_provenance": _builder_json_value(
            output.get("base_anchor_provenance", []),
        ),
        "mandatory_references": {
            "base_ids": list(mandatory.get("base_ids", [])),
            "uav_ids": list(mandatory.get("uav_ids", [])),
        },
        "optional_candidates": [_reference_key(key) for key in optional],
        "active_references": [_reference_key(key) for key in active_keys],
        "reference_evidence": [] if reference_evidence is None else reference_evidence,
        "reference_freshness": (
            [] if reference_freshness is None else reference_freshness
        ),
        "excluded_references": [] if excluded is None else excluded,
        "reference_violations": [] if violations is None else violations,
        "offline_truth_position": _builder_json_value(truth),
        "offline_error_norm": None,
        "offline_fresh_containment": None,
        "offline_aged_radius_containment": None,
        "offline_fresh_q_error": None,
        "offline_aged_q_error": None,
    })
    if truth is not None:
        row.update(_offline_metrics(output, np.asarray(truth, dtype=float)))
    if tuple(row) != ROW_FIELDS:
        row = {field: row[field] for field in ROW_FIELDS}
    _validate_row(row)
    return row


def _strict_int(
    value: object,
    *,
    minimum: int = 0,
    maximum: int | None = None,
) -> bool:
    return (
        isinstance(value, int)
        and not isinstance(value, bool)
        and value >= minimum
        and (maximum is None or value <= maximum)
    )


def _finite_number(value: object, *, nonnegative: bool = False) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
        and (not nonnegative or float(value) >= 0.0)
    )


def _finite_vec2(value: object) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 2
        and all(_finite_number(item) for item in value)
    )


def _finite_spd_2x2(value: object) -> bool:
    if (
        not isinstance(value, list)
        or len(value) != 2
        or any(not isinstance(row, list) or len(row) != 2 for row in value)
        or any(not _finite_number(item) for row in value for item in row)
    ):
        return False
    covariance = np.asarray(value, dtype=float)
    return bool(
        np.array_equal(covariance, covariance.T)
        and np.all(np.linalg.eigvalsh(covariance) > 0.0)
    )


def _sorted_unique_nonnegative_ints(value: object) -> bool:
    return (
        isinstance(value, list)
        and all(_strict_int(item) for item in value)
        and value == sorted(set(value))
    )


def _lower_hex(value: object, length: int) -> bool:
    return (
        isinstance(value, str)
        and len(value) == length
        and all(character in "0123456789abcdef" for character in value)
    )


def _canonical_iso_date(value: object) -> bool:
    if not isinstance(value, str):
        return False
    try:
        return datetime.strptime(value, "%Y-%m-%d").strftime("%Y-%m-%d") == value
    except ValueError:
        return False


def _disk_limits(
    disk_contract: Mapping,
    registered: bool,
) -> tuple[int, int, int]:
    if not isinstance(disk_contract, Mapping):
        raise ValueError("disk contract must be an object")
    if "hard_floor_bytes" in disk_contract:
        raise ValueError("test-only hard_floor_bytes is forbidden")
    required = (
        "launch_minimum_free_bytes",
        "live_minimum_free_bytes",
        "raw_bundle_max_allocated_bytes",
    )
    allowed = (*required, "compact_bundle_max_allocated_bytes")
    if tuple(disk_contract) not in {required, allowed}:
        raise ValueError("disk contract keys differ from frozen order")
    if any(
        not _strict_int(disk_contract[field])
        for field in tuple(disk_contract)
    ):
        raise ValueError("disk contract values must be nonnegative integers")
    if registered and disk_contract != REGISTERED_DISK_CONTRACT:
        raise ValueError("registered disk contract differs from frozen values")
    return tuple(disk_contract[field] for field in required)


def _validate_solver_result(result: object) -> None:
    if not isinstance(result, Mapping) or tuple(result) != SOLVER_RESULT_FIELDS:
        raise ValueError("solver result differs from exact schema")
    if result["status"] not in SOLVER_STATUSES:
        raise ValueError("solver status is not frozen")
    nullable_vec = result["estimate"]
    nullable_cov = result["covariance"]
    if nullable_vec is not None and not _finite_vec2(nullable_vec):
        raise ValueError("solver estimate is invalid")
    if nullable_cov is not None and not _finite_spd_2x2(nullable_cov):
        raise ValueError("solver covariance is invalid")
    for field in (
        "epsilon", "phi_min_eigenvalue", "phi_condition", "cost",
        "stationarity_norm",
    ):
        value = result[field]
        if value is not None and not _finite_number(value, nonnegative=True):
            raise ValueError(f"solver {field} is invalid")
    for field in ("proposal_count", "iterations"):
        if not _strict_int(result[field], maximum=50):
            raise ValueError(f"solver {field} is invalid")
    if not isinstance(result["fim_valid"], bool):
        raise ValueError("solver fim_valid is invalid")
    if (
        result["failure_reason"] is not None
        and not isinstance(result["failure_reason"], str)
    ):
        raise ValueError("solver failure reason is invalid")
    traces = result["proposal_trace"]
    if (
        not isinstance(traces, list)
        or result["proposal_count"] != result["iterations"]
        or result["proposal_count"] != len(traces)
    ):
        raise ValueError("solver proposal trace is not a list")
    for index, trace in enumerate(traces):
        if not isinstance(trace, Mapping) or tuple(trace) != PROPOSAL_TRACE_FIELDS:
            raise ValueError("proposal trace differs from exact schema")
        if not _strict_int(trace["proposal"]) or trace["proposal"] != index:
            raise ValueError("proposal trace index is invalid")
        for field in (
            "damping", "cost", "stationarity_norm", "raw_step_norm",
            "trial_cost",
        ):
            value = trace[field]
            if value is not None and not _finite_number(
                value, nonnegative=True,
            ):
                raise ValueError("proposal trace numeric field is invalid")
        if (
            trace["invalid_trial_reason"] is not None
            and (
                not isinstance(trace["invalid_trial_reason"], str)
                or not trace["invalid_trial_reason"]
            )
        ) or not isinstance(trace["accepted"], bool):
            raise ValueError("proposal trace discriminant is invalid")
        if (
            trace["damping"] is None
            or trace["cost"] is None
            or trace["stationarity_norm"] is None
        ):
            raise ValueError("proposal trace is incomplete")
        if trace["accepted"]:
            if (
                trace["raw_step_norm"] is None
                or trace["trial_cost"] is None
                or trace["invalid_trial_reason"] is not None
                or trace["trial_cost"] >= trace["cost"]
            ):
                raise ValueError("accepted proposal trace is inconsistent")
        elif (
            trace["trial_cost"] is None
            and trace["invalid_trial_reason"] is None
        ) or (
            trace["trial_cost"] is not None
            and trace["invalid_trial_reason"] is not None
        ) or (
            trace["trial_cost"] is not None
            and trace["trial_cost"] < trace["cost"]
        ):
            raise ValueError("rejected proposal trace is inconsistent")
    if result["status"] == "converged":
        if not _complete_converged_solver_result(result):
            raise ValueError("converged solver result is incomplete")
    elif (
        not isinstance(result["failure_reason"], str)
        or not result["failure_reason"]
        or result["fim_valid"] is not False
        or result["covariance"] is not None
        or result["epsilon"] is not None
        or result["phi_min_eigenvalue"] is not None
        or result["phi_condition"] is not None
    ):
        raise ValueError("nonconverged solver result null semantics differ")


def _validate_reference_key(record: object) -> tuple[int, int]:
    if (
        not isinstance(record, Mapping)
        or tuple(record) != REFERENCE_KEY_FIELDS
        or record["reference_kind"] not in {"base", "uav"}
        or not _strict_int(record["reference_id"])
    ):
        raise ValueError("reference key is invalid")
    return (
        0 if record["reference_kind"] == "base" else 1,
        record["reference_id"],
    )


def _validate_row(row: Mapping) -> None:
    if tuple(row) != ROW_FIELDS:
        raise ValueError("row differs from exact field order")
    if (
        row["method"] != METHOD_ID
        or row["invocation_name"] not in ROW_INVOCATION_NAMES
        or row["attempt_path"] not in {
            "reference_unavailable", "existing_predictive_multistart",
            "two_range_reacquisition", "smoke_branch_gate",
            "smoke_branch_pair_gate", "smoke_candidate_gate",
        }
        or not isinstance(row["selector_considered"], bool)
        or row["selector_consideration_reason"]
        not in {*CONSIDERATION_REASONS, "smoke_component_case"}
        or row["attempt_status"] not in ATTEMPT_STATUSES
        or row["output_status"] not in OUTPUT_STATUSES
        or not isinstance(row["prior_used_for_branch_selection"], bool)
        or row["prior_used_in_fim"] is not False
        or row["prior_used_for_continuous_update"] is not False
    ):
        raise ValueError("row scalar discriminant is invalid")
    if (
        row["attempt_failure_reason"] is not None
        and not isinstance(row["attempt_failure_reason"], str)
    ):
        raise ValueError("attempt failure reason is invalid")
    nullable_ints = {
        "seed": (20260727, 20260746),
        "frame_index": (0, 499),
        "robot_id": (1, 14),
        "squad_local_index": (1, 7),
        "applied_command_source_frame": (0, 498),
        "branch_selection_prior_source_fresh_frame": (0, 499),
        "branch_selection_prior_propagated_to_frame": (0, 499),
        "branch_selection_prior_age_frames": (0, 499),
        "next_private_state_source_fresh_frame": (0, 499),
        "next_private_state_propagated_to_frame": (0, 499),
        "next_private_state_age_frames": (0, 499),
    }
    for field, bounds in nullable_ints.items():
        value = row[field]
        if value is not None and not _strict_int(
            value, minimum=bounds[0], maximum=bounds[1],
        ):
            raise ValueError(f"{field} is invalid")
    if (
        row["prediction_age"] is not None
        and not _strict_int(row["prediction_age"], maximum=2)
    ):
        raise ValueError("prediction age is invalid")
    for field in ("fresh_epsilon", "aged_modeled_radius"):
        value = row[field]
        if value is not None and not _finite_number(value, nonnegative=True):
            raise ValueError(f"{field} is invalid")
    for field in (
        "applied_command", "estimate", "branch_selection_prior_estimate",
        "next_private_state_estimate", "offline_truth_position",
    ):
        value = row[field]
        if value is not None and not _finite_vec2(value):
            raise ValueError(f"{field} is invalid")
    for field in (
        "fresh_modeled_covariance", "aged_modeled_covariance",
        "branch_selection_prior_covariance",
        "next_private_state_covariance",
    ):
        value = row[field]
        if value is not None and not _finite_spd_2x2(value):
            raise ValueError(f"{field} is invalid")
    for field in (
        "offline_error_norm", "offline_fresh_q_error",
        "offline_aged_q_error",
    ):
        value = row[field]
        if value is not None and not _finite_number(value, nonnegative=True):
            raise ValueError(f"{field} is invalid")
    for field in (
        "offline_fresh_containment", "offline_aged_radius_containment",
    ):
        if row[field] is not None and not isinstance(row[field], bool):
            raise ValueError(f"{field} is invalid")
    for field in (
        "attempt_base_anchor_provenance", "base_anchor_provenance",
    ):
        if not _sorted_unique_nonnegative_ints(row[field]):
            raise ValueError(f"{field} is not sorted unique")

    invocation = row["invocation_name"]
    smoke_values = (
        row["smoke_case_id"], row["smoke_case_kind"],
        row["smoke_case_input"], row["smoke_case_expected"],
    )
    production_values = (
        row["seed"], row["frame_index"], row["robot_id"],
        row["squad_local_index"], row["offline_truth_position"],
    )
    if invocation == "registered_replay":
        if any(value is not None for value in smoke_values) or any(
            value is None for value in production_values
        ):
            raise ValueError("registered row null contract differs")
        if row["frame_index"] == 0:
            if (
                row["applied_command_source_frame"] is not None
                or row["applied_command"] is not None
            ):
                raise ValueError("frame-zero command must be null")
        elif (
            row["applied_command_source_frame"] != row["frame_index"] - 1
            or row["applied_command"] is None
        ):
            raise ValueError("registered command contract differs")
    else:
        if any(value is None for value in smoke_values):
            raise ValueError("smoke declaration is incomplete")
        if row["smoke_case_id"] not in SMOKE_CASE_IDS:
            raise ValueError("smoke case ID is not declared")
        if row["smoke_case_id"] == MECHANISM_FIXTURE_ID:
            if row["smoke_case_kind"] != "mechanism_fixture":
                raise ValueError("mechanism smoke kind differs")
            if row["smoke_case_input"] != {
                "fixture_id": MECHANISM_FIXTURE_ID,
                "fixture_sha256": MECHANISM_FIXTURE_SHA256,
            } or row["smoke_case_expected"] != MECHANISM_SMOKE_EXPECTED:
                raise ValueError("mechanism smoke declaration differs")
            if any(value is None for value in production_values):
                raise ValueError("mechanism smoke production key is incomplete")
            if (
                (row["seed"], row["frame_index"], row["robot_id"])
                != MECHANISM_FIXTURE_KEY
                or row["squad_local_index"]
                != MECHANISM_FIXTURE_SQUAD_LOCAL_INDEX
            ):
                raise ValueError("mechanism smoke key differs from fixture")
        else:
            case = next(
                item for item in SYNTHETIC_CASES
                if item["case_id"] == row["smoke_case_id"]
            )
            if (
                row["smoke_case_kind"] != case["case_kind"]
                or row["smoke_case_input"] != case["input"]
                or row["smoke_case_expected"] != case["expected"]
                or any(value is not None for value in production_values)
                or row["applied_command_source_frame"] is not None
                or row["applied_command"] is not None
            ):
                raise ValueError("synthetic smoke declaration differs")
        if invocation == "unit_fixture" and row["smoke_case_id"] != MECHANISM_FIXTURE_ID:
            raise ValueError("unit fixture invocation carries synthetic case")

    status = row["output_status"]
    if (row["attempt_status"] == "accepted") != (status == "fresh"):
        raise ValueError("accepted attempt and fresh publication differ")
    if status == "fresh":
        if (
            row["prediction_age"] != 0
            or row["estimate"] is None
            or row["fresh_modeled_covariance"] is None
            or row["fresh_epsilon"] is None
            or row["aged_modeled_covariance"] is not None
            or row["aged_modeled_radius"] is not None
        ):
            raise ValueError("fresh output null contract differs")
    elif status == "predicted":
        if (
            row["prediction_age"] not in {1, 2}
            or row["estimate"] is None
            or row["fresh_modeled_covariance"] is not None
            or row["fresh_epsilon"] is not None
            or row["aged_modeled_covariance"] is None
            or row["aged_modeled_radius"] is None
        ):
            raise ValueError("predicted output null contract differs")
    elif any(
        row[field] is not None
        for field in (
            "prediction_age", "estimate", "fresh_modeled_covariance",
            "fresh_epsilon", "aged_modeled_covariance",
            "aged_modeled_radius",
        )
    ):
        raise ValueError("unavailable output carries numeric payload")

    if invocation == "smoke_validation" and row["smoke_case_id"] != MECHANISM_FIXTURE_ID:
        if any(
            row[field] is not None
            for field in (
                "offline_truth_position", "offline_error_norm",
                "offline_fresh_containment",
                "offline_aged_radius_containment",
                "offline_fresh_q_error", "offline_aged_q_error",
            )
        ):
            raise ValueError("synthetic row carries offline evidence")
    elif row["offline_truth_position"] is not None:
        required_metrics = {
            "fresh": (
                "offline_error_norm", "offline_fresh_containment",
                "offline_fresh_q_error",
            ),
            "predicted": (
                "offline_error_norm", "offline_aged_radius_containment",
                "offline_aged_q_error",
            ),
            "unavailable": (),
        }[status]
        for field in (
            "offline_error_norm", "offline_fresh_containment",
            "offline_aged_radius_containment", "offline_fresh_q_error",
            "offline_aged_q_error",
        ):
            if (field in required_metrics) != (row[field] is not None):
                raise ValueError("offline metric null contract differs")

    if row["attempt_path"] == "two_range_reacquisition":
        if (
            row["selector_considered"] is not True
            or row["existing_candidates"]
            or row["existing_selected_candidate_source"] is not None
        ):
            raise ValueError("two-range row carries existing candidates")
        branches = row["branches"]
        if len(branches) not in (0, 2):
            raise ValueError("two-range row must carry zero or two branches")
        pre_solver_failures = {
            "two_range_robot_id_invalid",
            "two_range_input_invalid",
            "two_range_reference_covariance_invalid",
            "two_range_reference_keys_invalid",
            "two_range_private_prior_invalid",
            "two_range_base_anchor_provenance_invalid",
            "two_range_circle_geometry_invalid",
            "two_range_circle_starts_not_distinct",
        }
        failure = row["attempt_failure_reason"]
        if row["attempt_status"] == "accepted" and len(branches) != 2:
            raise ValueError("accepted two-range row lacks two branches")
        if failure in pre_solver_failures and branches:
            raise ValueError("pre-solver rejection carries solved branches")
        if (
            row["attempt_status"] != "accepted"
            and failure is not None
            and failure not in pre_solver_failures
            and not branches
        ):
            raise ValueError("post-solver rejection lacks branch evidence")
    elif row["attempt_path"] == "existing_predictive_multistart":
        if (
            row["selector_considered"] is not False
            or row["branches"]
            or row["selected_branch_id"] is not None
            or row["prior_used_for_branch_selection"] is not False
        ):
            raise ValueError("existing row carries two-range branches")
    elif row["attempt_path"] == "reference_unavailable":
        if (
            row["attempt_status"] != "reference_unavailable"
            or row["branches"]
            or row["existing_candidates"]
            or row["selected_branch_id"]
            or row["existing_selected_candidate_source"] is not None
            or row["prior_used_for_branch_selection"] is not False
        ):
            raise ValueError("unavailable row carries candidate evidence")
    component_paths = {
        "branch_gate": "smoke_branch_gate",
        "branch_pair_gate": "smoke_branch_pair_gate",
        "candidate_gate": "smoke_candidate_gate",
    }
    if row["smoke_case_kind"] in component_paths:
        expected_path = component_paths[row["smoke_case_kind"]]
        empty_fields = (
            "attempt_base_anchor_provenance", "base_anchor_provenance",
            "optional_candidates", "active_references",
            "reference_evidence", "reference_freshness",
            "excluded_references", "reference_violations",
        )
        component_shape_valid = (
            row["invocation_name"] == "smoke_validation"
            and row["attempt_path"] == expected_path
            and row["selector_considered"] is False
            and row["selector_consideration_reason"] == "smoke_component_case"
            and row["attempt_status"] == "invalid"
            and row["attempt_failure_reason"] is None
            and row["output_status"] == "unavailable"
            and row["branch_selection_prior_status"] == "absent"
            and row["next_private_state_status"] == "absent"
            and row["prior_used_for_branch_selection"] is False
            and row["selected_branch_id"] is None
            and row["existing_selected_candidate_source"] is None
            and row["mandatory_references"] == {
                "base_ids": [], "uav_ids": [],
            }
            and all(row[field] == [] for field in empty_fields)
        )
        cardinality_valid = {
            "branch_gate": (
                len(row["branches"]) == 0
                and len(row["existing_candidates"]) == 0
            ),
            "branch_pair_gate": (
                len(row["branches"]) == 2
                and len(row["existing_candidates"]) == 0
            ),
            "candidate_gate": (
                len(row["branches"]) == 0
                and len(row["existing_candidates"]) == 1
            ),
        }[row["smoke_case_kind"]]
        if not component_shape_valid or not cardinality_valid:
            raise ValueError("component smoke encoding differs from frozen form")
    elif row["attempt_path"].startswith("smoke_"):
        raise ValueError("component attempt path differs from smoke kind")
    if row["branches"] and [
        branch.get("branch_id")
        for branch in row["branches"]
        if isinstance(branch, Mapping)
    ] != list(BRANCH_IDS):
        raise ValueError("branches differ from frozen order")
    for branch in row["branches"]:
        if _branch(branch) != branch:
            raise ValueError("branch is not canonical")
        if (
            branch["branch_id"] not in BRANCH_IDS
            or not _finite_vec2(branch["circle_start"])
        ):
            raise ValueError("branch scalar contract differs")
        _validate_solver_result(branch["solver_result"])
        if (
            row["attempt_status"] == "accepted"
            or branch["q_branch"] is not None
            or branch["passes_branch_gate"] is not None
        ) and branch["solver_result"]["status"] != "converged":
            raise ValueError("scored or accepted branch did not converge")
        if branch["q_branch"] is not None and not _finite_number(
            branch["q_branch"], nonnegative=True,
        ):
            raise ValueError("branch score is invalid")
        if (
            branch["passes_branch_gate"] is not None
            and not isinstance(branch["passes_branch_gate"], bool)
        ):
            raise ValueError("branch gate result is invalid")
    for candidate in row["existing_candidates"]:
        if _existing_candidate(candidate) != candidate:
            raise ValueError("existing candidate is not canonical")
        _validate_solver_result(candidate["result"])
        if (
            candidate["source"] not in {
                "prediction", "private_reacquisition_seed", "algebraic",
                "circle_negative", "circle_positive",
            }
            or not _finite_vec2(candidate["initial_estimate"])
        ):
            raise ValueError("candidate initial estimate is invalid")
        if not isinstance(candidate["accepted"], bool):
            raise ValueError("candidate acceptance flag is invalid")
        if (
            candidate["rejection_reason"] is not None
            and not isinstance(candidate["rejection_reason"], str)
        ):
            raise ValueError("candidate rejection reason is invalid")
        diagnostics = candidate["gate_diagnostics"]
        if (
            candidate["q_innov"] != diagnostics["q_innov"]
            or (
                candidate["accepted"]
                != (diagnostics["gate_outcome"] == "accepted")
            )
            or (
                candidate["accepted"]
                and (
                    candidate["rejection_reason"] is not None
                    or candidate["result"]["status"] != "converged"
                )
            )
            or (
                not candidate["accepted"]
                and (
                    not isinstance(candidate["rejection_reason"], str)
                    or not candidate["rejection_reason"]
                )
            )
        ):
            raise ValueError("candidate gate summary is inconsistent")
    nested_fields = (
        ("mandatory_references", MANDATORY_REFERENCE_FIELDS),
        *(
            (name, fields)
            for name, fields in (
                ("optional_candidates", REFERENCE_KEY_FIELDS),
                ("active_references", REFERENCE_KEY_FIELDS),
                ("reference_evidence", REFERENCE_EVIDENCE_FIELDS),
                ("reference_freshness", REFERENCE_FRESHNESS_FIELDS),
                ("excluded_references", EXCLUSION_FIELDS),
                ("reference_violations", VIOLATION_FIELDS),
            )
        ),
    )
    if tuple(row["mandatory_references"]) != MANDATORY_REFERENCE_FIELDS:
        raise ValueError("mandatory references are not canonical")
    if not all(
        _sorted_unique_nonnegative_ints(row["mandatory_references"][field])
        for field in MANDATORY_REFERENCE_FIELDS
    ):
        raise ValueError("mandatory reference IDs are not sorted unique")
    for name, fields in nested_fields[1:]:
        if not isinstance(row[name], list):
            raise ValueError(f"{name} must be a list")
        if any(
            not isinstance(item, Mapping) or tuple(item) != fields
            for item in row[name]
        ):
            raise ValueError(f"{name} contains a noncanonical object")
    for name in ("optional_candidates", "active_references"):
        keys = [_validate_reference_key(item) for item in row[name]]
        if keys != sorted(set(keys)):
            raise ValueError(f"{name} is not canonical sorted unique")
    ordered_reference_fields = (
        "reference_evidence", "reference_freshness",
        "excluded_references", "reference_violations",
    )
    for name in ordered_reference_fields:
        keys = [
            (
                0 if item["reference_kind"] == "base" else 1,
                item["reference_id"],
            )
            for item in row[name]
        ]
        if keys != sorted(keys) or len(keys) != len(set(keys)):
            raise ValueError(f"{name} differs from canonical reference order")
    if (
        row["reference_evidence"]
        and row["reference_freshness"]
        and [
            (item["reference_kind"], item["reference_id"])
            for item in row["reference_evidence"]
        ]
        != [
            (item["reference_kind"], item["reference_id"])
            for item in row["reference_freshness"]
        ]
    ):
        raise ValueError("reference evidence and freshness order differ")
    for record in row["reference_evidence"]:
        if (
            record["reference_kind"] not in {"base", "uav"}
            or not _strict_int(record["reference_id"])
            or record["role"] not in {"mandatory", "optional"}
            or not isinstance(record["measurement_present"], bool)
            or (
                record["noisy_range"] is not None
                and not _finite_number(
                    record["noisy_range"], nonnegative=True,
                )
            )
            or (
                record["noise_seed"] is not None
                and not _strict_int(record["noise_seed"])
            )
            or record["current_freshness"]
            not in {"fresh", "predicted", "unavailable", "missing"}
            or not isinstance(record["eligible"], bool)
            or not isinstance(record["used"], bool)
            or (
                record["exclusion_reason"] is not None
                and not isinstance(record["exclusion_reason"], str)
            )
            or not _sorted_unique_nonnegative_ints(
                record["base_anchor_provenance"],
            )
        ):
            raise ValueError("reference evidence is invalid")
    for record in row["reference_freshness"]:
        if (
            record["reference_kind"] not in {"base", "uav"}
            or not _strict_int(record["reference_id"])
            or record["current_freshness"]
            not in {"fresh", "predicted", "unavailable", "missing"}
        ):
            raise ValueError("reference freshness is invalid")
    for name in ("excluded_references", "reference_violations"):
        for record in row[name]:
            if (
                record["reference_kind"] not in {"base", "uav"}
                or not _strict_int(record["reference_id"])
                or not isinstance(record["reason"], str)
            ):
                raise ValueError(f"{name} is invalid")
    prior_available = row["branch_selection_prior_status"] == "available"
    if row["branch_selection_prior_status"] not in {"available", "absent"}:
        raise ValueError("branch-selection prior status is invalid")
    prior_fields_present = [
        row[field] is not None for field in PRIVATE_PRIOR_FIELDS[1:]
    ]
    if (
        prior_available and not all(prior_fields_present)
    ) or (
        not prior_available and any(prior_fields_present)
    ):
        raise ValueError("branch-selection prior null semantics differ")
    next_available = row["next_private_state_status"] == "available"
    if row["next_private_state_status"] not in {"available", "absent"}:
        raise ValueError("next-private-state status is invalid")
    next_fields_present = [
        row[field] is not None for field in NEXT_PRIVATE_STATE_FIELDS[1:]
    ]
    if (
        next_available and not all(next_fields_present)
    ) or (
        not next_available and any(next_fields_present)
    ):
        raise ValueError("next-private-state null semantics differ")
    if prior_available and (
        row["branch_selection_prior_source_fresh_frame"]
        + row["branch_selection_prior_age_frames"]
        != row["branch_selection_prior_propagated_to_frame"]
    ):
        raise ValueError("branch-selection prior frame arithmetic differs")
    if next_available and (
        row["next_private_state_source_fresh_frame"]
        + row["next_private_state_age_frames"]
        != row["next_private_state_propagated_to_frame"]
    ):
        raise ValueError("next-private-state frame arithmetic differs")
    if row["attempt_status"] == "rejected":
        prior_projection = (
            row["branch_selection_prior_estimate"],
            row["branch_selection_prior_covariance"],
            row["branch_selection_prior_source_fresh_frame"],
            row["branch_selection_prior_propagated_to_frame"],
            row["branch_selection_prior_age_frames"],
        )
        next_projection = (
            row["next_private_state_estimate"],
            row["next_private_state_covariance"],
            row["next_private_state_source_fresh_frame"],
            row["next_private_state_propagated_to_frame"],
            row["next_private_state_age_frames"],
        )
        if (
            row["branch_selection_prior_status"]
            != row["next_private_state_status"]
            or prior_projection != next_projection
        ):
            raise ValueError(
                "rejected attempt changed outgoing private state",
            )
    passing = [
        branch for branch in row["branches"]
        if branch["passes_branch_gate"] is True
    ]
    if row["prior_used_for_branch_selection"]:
        if len(row["branches"]) != 2 or any(
            branch["q_branch"] is None
            or not isinstance(branch["passes_branch_gate"], bool)
            for branch in row["branches"]
        ):
            raise ValueError("scored branches are incomplete")
    elif any(
        branch["q_branch"] is not None
        or branch["passes_branch_gate"] is not None
        for branch in row["branches"]
    ):
        raise ValueError("unscored branch carries a score")
    if row["selected_branch_id"] is not None and (
        len(passing) != 1
        or passing[0]["branch_id"] != row["selected_branch_id"]
    ):
        raise ValueError("selected branch is not uniquely passing")
    if row["selected_branch_id"] is None and len(passing) == 1 and (
        row["attempt_path"] == "two_range_reacquisition"
        and row["attempt_status"] == "accepted"
    ):
        raise ValueError("accepted branch was not published")
    selected_candidate = row["existing_selected_candidate_source"]
    if selected_candidate is not None:
        accepted = [
            candidate for candidate in row["existing_candidates"]
            if candidate["accepted"] is True
        ]
        if (
            row["attempt_path"] != "existing_predictive_multistart"
            or row["attempt_status"] != "accepted"
            or row["output_status"] != "fresh"
            or len(accepted) != 1
            or accepted[0]["source"] != selected_candidate
        ):
            raise ValueError("candidate publication is not accepted/fresh")
    ordered_strict_json_bytes(row, ROW_FIELDS)


def iter_registered_keys():
    for seed in range(20260727, 20260747):
        for frame_index in range(500):
            for robot_id in range(1, 15):
                yield METHOD_ID, seed, frame_index, robot_id


def validate_key_sequence(observed, expected) -> int:
    expected_iterator = iter(expected)
    count = 0
    for key in observed:
        try:
            wanted = next(expected_iterator)
        except StopIteration:
            raise ValueError("extra raw key") from None
        if tuple(key) != tuple(wanted):
            raise ValueError("raw key is missing, duplicate, or out of order")
        count += 1
    try:
        next(expected_iterator)
    except StopIteration:
        return count
    raise ValueError("raw keys are missing")


def _selector_input(case: Mapping) -> dict:
    values = copy.deepcopy(COMMON_SELECTOR_INPUT)
    values.update(copy.deepcopy(case["input"]["overrides"]))
    return values


def _private_from_selector_input(values: Mapping) -> dict:
    return {
        "status": "available",
        "estimate": values["private_prior_estimate"],
        "modeled_covariance": values["private_prior_covariance"],
        "source_fresh_frame": values["private_prior_source_fresh_frame"],
        "propagated_to_frame": values[
            "private_prior_propagated_to_frame"
        ],
        "age_frames": values["private_prior_age_frames"],
    }


def _selector_smoke_row(case: Mapping) -> dict:
    values = _selector_input(case)
    attempt = solve_two_range_reacquisition(
        robot_id=values["robot_id"],
        reference_positions=values["reference_positions"],
        reference_covariances=values["reference_covariances"],
        measurements=values["measurements"],
        reference_keys=values["reference_keys"],
        private_prior=_private_from_selector_input(values),
        ranging_sigma=values["ranging_sigma"],
        base_anchor_provenance=values["base_anchor_provenance"],
    )
    prior = (
        None
        if attempt.get("failure_reason") == "two_range_private_prior_invalid"
        else _private_from_selector_input(values)
    )
    lifecycle = finalize_two_range_lifecycle(
        attempt,
        {
            "public_prediction": make_unavailable_output(
                "smoke_no_live_prediction",
            ),
            "branch_selection_prior": prior,
        },
        frame_index=20,
    )
    return _assemble_row(
        invocation_name="smoke_validation",
        smoke_case_id=case["case_id"],
        smoke_case_kind=case["case_kind"],
        smoke_case_input=case["input"],
        smoke_case_expected=case["expected"],
        seed=None,
        frame_index=None,
        robot_id=None,
        squad_local_index=None,
        command_source=None,
        command=None,
        attempt_path="two_range_reacquisition",
        considered=True,
        consideration_reason="considered",
        attempt=attempt,
        prior=prior,
        lifecycle=lifecycle,
        mandatory={"base_ids": [], "uav_ids": [10, 11]},
        optional=[],
        active_keys=[("uav", 10), ("uav", 11)],
        provenance=values["base_anchor_provenance"],
    )


def _component_lifecycle() -> dict:
    return {
        "public_output": make_unavailable_output("smoke_component_case"),
        "next_private_state": None,
    }


def _component_smoke_row(case: Mapping) -> dict:
    kind = case["case_kind"]
    attempt = {
        "attempt_status": "invalid",
        "failure_reason": None,
        "branches": [],
        "selected_branch_id": None,
        "candidates": [],
        "selected_candidate": None,
        "prior_used_for_branch_selection": False,
    }
    if kind == "branch_gate":
        source = case["input"]["q_source"]
        value = {
            "threshold": INNOVATION_REFERENCE_QUANTILE,
            "nextafter_down": np.nextafter(
                INNOVATION_REFERENCE_QUANTILE, -np.inf,
            ).item(),
            "nextafter_up": np.nextafter(
                INNOVATION_REFERENCE_QUANTILE, np.inf,
            ).item(),
        }[source]
        if branch_gate_passes(value) is not case["expected"]["passes"]:
            raise RuntimeError("branch-gate smoke declaration mismatch")
        path = "smoke_branch_gate"
    elif kind == "branch_pair_gate":
        override_id = case["input"]["solver_override_id"]
        branches = [
            copy.deepcopy(item)
            for item in SMOKE_BRANCH_PAIR_RECORDS[override_id]
        ]
        observed = validate_solver_branches(branches)
        if observed != (
            case["expected"]["valid"], case["expected"]["reason"],
        ):
            raise RuntimeError("branch-pair smoke declaration mismatch")
        attempt["branches"] = branches
        path = "smoke_branch_pair_gate"
    elif kind == "candidate_gate":
        template = copy.deepcopy(
            CANDIDATE_TEMPLATES[
                case["input"]["candidate_template_id"]
            ],
        )
        cost = (
            9.0
            if case["input"]["cost_source"] == "nine"
            else np.nextafter(9.0, np.inf).item()
        )
        template["cost"] = cost
        accepted, reason, diagnostics = candidate_acceptance(
            template,
            live_prediction=None,
            active_reference_count=2,
            base_anchor_provenance=[0, 1],
            allow_two_reference_reacquisition=True,
        )
        if (accepted, reason) != (
            case["expected"]["accepted"], case["expected"]["reason"],
        ):
            raise RuntimeError("candidate-gate smoke declaration mismatch")
        candidate = frozen_candidate_gate_record(
            cost=cost, accepted=accepted, reason=reason,
        )
        candidate["gate_diagnostics"] = diagnostics
        attempt["candidates"] = [candidate]
        path = "smoke_candidate_gate"
    else:
        raise ValueError("unknown component smoke case kind")
    return _assemble_row(
        invocation_name="smoke_validation",
        smoke_case_id=case["case_id"],
        smoke_case_kind=kind,
        smoke_case_input=case["input"],
        smoke_case_expected=case["expected"],
        seed=None,
        frame_index=None,
        robot_id=None,
        squad_local_index=None,
        command_source=None,
        command=None,
        attempt_path=path,
        considered=False,
        consideration_reason="smoke_component_case",
        attempt=attempt,
        prior=None,
        lifecycle=_component_lifecycle(),
        mandatory={"base_ids": [], "uav_ids": []},
        optional=[],
        active_keys=[],
        provenance=[],
    )


def _internal_public(output: Mapping) -> dict:
    status = output["output_status"]
    return {
        "output_status": status,
        "prediction_age": output.get("prediction_age"),
        "estimate": output.get("estimate"),
        "modeled_covariance": (
            output.get("fresh_modeled_covariance")
            if status == "fresh"
            else output.get("aged_modeled_covariance")
        ),
        "epsilon": (
            output.get("fresh_epsilon") if status == "fresh" else None
        ),
        "aged_modeled_radius": output.get("aged_modeled_radius"),
        "base_anchor_provenance": output.get(
            "base_anchor_provenance", [],
        ),
    }


def _mechanism_smoke_row(fixture: Mapping) -> dict:
    if fixture.get("fixture_id") != SMOKE_CASE_IDS[0]:
        raise ValueError("mechanism fixture ID differs from approved case")
    key = fixture["key"]
    references = {
        int(record["reference_key"][1]): _internal_public(
            record["public_output"],
        )
        for record in fixture["current_reference_outputs"]
    }
    keys = [tuple(key) for key in fixture["measurements"]["reference_keys"]]
    positions = [references[identifier]["estimate"] for _, identifier in keys]
    covariances = [
        references[identifier]["modeled_covariance"]
        for _, identifier in keys
    ]
    mandatory = fixture["mandatory_references"]
    qualification = {
        "status": "ok",
        "active_keys": keys,
        "active_records": [
            {
                "key": key_pair,
                "present": True,
                "noisy_range": noisy_range,
            }
            for key_pair, noisy_range in zip(
                keys, fixture["measurements"]["ranges"], strict=True,
            )
        ],
        "base_anchor_provenance": tuple(
            fixture["measurements"]["base_anchor_provenance"],
        ),
        "fixed_outputs": references,
    }
    considered, consideration_reason = selector_consideration(
        robot_id=key["robot_id"],
        live_prediction=None,
        mandatory=mandatory,
        optional_keys=fixture["optional_references"],
        qualification=qualification,
    )
    if not considered:
        raise ValueError(
            "approved mechanism fixture is not structurally considered: "
            f"{consideration_reason}"
        )
    prior = copy.deepcopy(fixture["preceding_private_state"])
    attempt = solve_two_range_reacquisition(
        robot_id=key["robot_id"],
        reference_positions=positions,
        reference_covariances=covariances,
        measurements=fixture["measurements"]["ranges"],
        reference_keys=keys,
        private_prior=prior,
        ranging_sigma=fixture["measurements"]["ranging_sigma"],
        base_anchor_provenance=fixture["measurements"][
            "base_anchor_provenance"
        ],
    )
    lifecycle = finalize_two_range_lifecycle(
        attempt,
        {
            "public_prediction": make_unavailable_output(
                "prediction_expired",
            ),
            "branch_selection_prior": prior,
        },
        frame_index=key["frame_index"],
    )
    canonical_fixture = json.dumps(
        fixture, allow_nan=False, ensure_ascii=False,
        separators=(",", ":"), sort_keys=False,
    ).encode("utf-8") + b"\n"
    smoke_input = {
        "fixture_id": fixture["fixture_id"],
        "fixture_sha256": hashlib.sha256(canonical_fixture).hexdigest(),
    }
    evidence = []
    freshness = []
    for key_pair, noisy_range, noise_seed in zip(
        keys,
        fixture["measurements"]["ranges"],
        fixture["measurements"]["noise_seeds"],
        strict=True,
    ):
        output = references[key_pair[1]]
        evidence.append({
            "reference_kind": key_pair[0],
            "reference_id": key_pair[1],
            "role": "mandatory",
            "measurement_present": True,
            "noisy_range": noisy_range,
            "noise_seed": noise_seed,
            "current_freshness": "fresh",
            "eligible": True,
            "used": True,
            "exclusion_reason": None,
            "base_anchor_provenance": output["base_anchor_provenance"],
        })
        freshness.append({
            "reference_kind": key_pair[0],
            "reference_id": key_pair[1],
            "current_freshness": "fresh",
        })
    return _assemble_row(
        invocation_name="smoke_validation",
        smoke_case_id=fixture["fixture_id"],
        smoke_case_kind="mechanism_fixture",
        smoke_case_input=smoke_input,
        smoke_case_expected=MECHANISM_SMOKE_EXPECTED,
        seed=key["seed"],
        frame_index=key["frame_index"],
        robot_id=key["robot_id"],
        squad_local_index=key["squad_local_index"],
        command_source=fixture["held_command"]["source_frame"],
        command=fixture["held_command"]["command"],
        attempt_path="two_range_reacquisition",
        considered=True,
        consideration_reason="considered",
        attempt=attempt,
        prior=prior,
        lifecycle=lifecycle,
        mandatory=mandatory,
        optional=[],
        active_keys=keys,
        provenance=fixture["measurements"]["base_anchor_provenance"],
        truth=fixture["key"]["truth_position"],
        reference_evidence=evidence,
        reference_freshness=freshness,
    )


def produce_smoke_row(*, case_id: str, mechanism_fixture: Mapping) -> dict:
    if case_id not in SMOKE_CASE_IDS:
        raise ValueError("smoke case ID is not declared")
    if case_id == SMOKE_CASE_IDS[0]:
        return _mechanism_smoke_row(mechanism_fixture)
    case = next(
        record for record in SYNTHETIC_CASES
        if record["case_id"] == case_id
    )
    if case["case_kind"] == "selector":
        return _selector_smoke_row(case)
    return _component_smoke_row(case)


def _freshness(output: object) -> str:
    if not isinstance(output, Mapping):
        return "missing"
    status = output.get("output_status")
    return status if status in {"fresh", "predicted", "unavailable"} else "missing"


def _mapping_reference_evidence(
    *,
    mandatory: Mapping,
    optional: list,
    records: Mapping,
    noise_seeds: Mapping,
    qualification: Mapping,
    current_public: Mapping,
    used_keys: tuple[tuple[str, int], ...],
) -> list[dict]:
    mandatory_keys = [
        *(("base", identifier) for identifier in mandatory["base_ids"]),
        *(("uav", identifier) for identifier in mandatory["uav_ids"]),
    ]
    all_keys = sorted(
        set(mandatory_keys) | set(optional),
        key=lambda key: (0 if key[0] == "base" else 1, key[1]),
    )
    active = {tuple(key) for key in qualification.get("active_keys", [])}
    excluded = {
        tuple(record["key"]): record["reason"]
        for record in qualification.get("excluded", [])
    }
    missing = {
        tuple(key) for key in qualification.get("missing_mandatory", [])
    }
    evidence = []
    for key in all_keys:
        record = records.get(key)
        present = (
            isinstance(record, Mapping) and record.get("present") is True
        )
        output = current_public.get(key[1]) if key[0] == "uav" else None
        current_freshness = "fresh" if key[0] == "base" else _freshness(output)
        eligible = (
            present
            and (
                key[0] == "base"
                or (
                    reference_is_eligible(output)
                    and current_freshness == "fresh"
                )
            )
        )
        reason = excluded.get(key)
        if key in missing:
            reason = (
                "measurement_not_present"
                if not present else "not_current_frame_fresh"
            )
        provenance = (
            [key[1]]
            if key[0] == "base"
            else list(
                output.get("base_anchor_provenance", [])
                if isinstance(output, Mapping) else []
            )
        )
        evidence.append({
            "reference_kind": key[0],
            "reference_id": key[1],
            "role": "mandatory" if key in mandatory_keys else "optional",
            "measurement_present": present,
            "noisy_range": (
                _builder_json_value(record.get("noisy_range"))
                if present else None
            ),
            "noise_seed": noise_seeds.get(key),
            "current_freshness": current_freshness,
            "eligible": bool(eligible),
            "used": key in used_keys,
            "exclusion_reason": reason,
            "base_anchor_provenance": sorted(set(provenance)),
        })
    return evidence


def _reason_mappings(records: object) -> list[dict]:
    if not isinstance(records, list):
        raise ValueError("reason records must be a list")
    normalized = []
    for source in records:
        if (
            not isinstance(source, Mapping)
            or set(source) != {"key", "reason"}
        ):
            raise ValueError("reason record differs from exact source schema")
        key = source["key"]
        normalized.append({
            "reference_kind": key[0],
            "reference_id": key[1],
            "reason": source["reason"],
        })
    return normalized


def produce_method_row(
    *,
    seed: int,
    frame_index: int,
    robot_id: int,
    config: Mapping,
    truth_positions: Mapping[int, np.ndarray],
    current_public: dict[int, dict],
    previous_state: Mapping | None,
    applied_command: object,
    ranging_sigma: float,
) -> tuple[dict, dict]:
    previous = previous_state if isinstance(previous_state, Mapping) else {}
    prior_bundle = advance_two_range_prior(
        previous.get("public_output"),
        previous.get("private_state"),
        applied_command,
        next_frame_index=frame_index,
    )
    mandatory, optional, records, noise_seeds = _sensor_records(
        dict(config),
        robot_id,
        dict(truth_positions),
        seed,
        frame_index,
        ranging_sigma,
    )
    qualification = qualify_active_references(
        mandatory=mandatory,
        optional_keys=optional,
        measurement_records=records,
        uav_outputs=current_public,
        variant="predictive_multistart",
    )
    qualification = {
        **qualification,
        "fixed_outputs": {
            identifier: current_public[identifier]
            for identifier in mandatory["uav_ids"]
            if identifier in current_public
        },
    }
    active_records = _canonical_active_records(qualification)
    arrays = (
        _public_reference_arrays(
            dict(config), active_records, current_public,
        )
        if qualification.get("status") == "ok" else None
    )
    live_prediction = (
        prior_bundle["public_prediction"]
        if prior_bundle["public_prediction"].get("output_status")
        == "predicted"
        else None
    )
    considered, reason = selector_consideration(
        robot_id=robot_id,
        live_prediction=live_prediction,
        mandatory=mandatory,
        optional_keys=optional,
        qualification=qualification,
    )
    used_keys: tuple[tuple[str, int], ...] = ()
    if arrays is None:
        attempt_path = "reference_unavailable"
        considered = False
        reason = (
            reason if reason != "considered" else "qualification_not_ok"
        )
        attempt = {
            "attempt_status": "reference_unavailable",
            "failure_reason": "reference_unavailable",
            "candidate": None,
            "candidates": [],
            "selected_candidate": None,
            "branches": [],
            "selected_branch_id": None,
            "prior_used_for_branch_selection": False,
        }
    else:
        positions, covariances, measurements, keys = arrays
        used_keys = tuple(keys)
        provenance = tuple(qualification["base_anchor_provenance"])
        if considered:
            attempt_path = "two_range_reacquisition"
            attempt = solve_two_range_reacquisition(
                robot_id=robot_id,
                reference_positions=positions,
                reference_covariances=covariances,
                measurements=measurements,
                reference_keys=keys,
                private_prior=prior_bundle["branch_selection_prior"],
                ranging_sigma=ranging_sigma,
                base_anchor_provenance=provenance,
            )
        else:
            attempt_path = "existing_predictive_multistart"
            private = prior_bundle.get("branch_selection_prior")
            private_projection = (
                {
                    "estimate": private["estimate"],
                    "modeled_covariance": private["modeled_covariance"],
                }
                if isinstance(private, Mapping) else None
            )
            attempt = solve_predictive_multistart(
                reference_positions=positions,
                reference_covariances=covariances,
                measurements=measurements,
                reference_keys=keys,
                live_prediction=live_prediction,
                private_seed=private_projection,
                ranging_sigma=ranging_sigma,
                base_anchor_provenance=provenance,
            )
            attempt = {
                **attempt,
                "branches": [],
                "selected_branch_id": None,
                "prior_used_for_branch_selection": False,
                "prior_used_in_fim": False,
                "prior_used_for_continuous_update": False,
            }
    lifecycle = finalize_two_range_lifecycle(
        attempt, prior_bundle, frame_index=frame_index,
    )
    evidence = _mapping_reference_evidence(
        mandatory=mandatory,
        optional=optional,
        records=records,
        noise_seeds=noise_seeds,
        qualification=qualification,
        current_public=current_public,
        used_keys=used_keys,
    )
    row = _assemble_row(
        invocation_name="registered_replay",
        smoke_case_id=None,
        smoke_case_kind=None,
        smoke_case_input=None,
        smoke_case_expected=None,
        seed=seed,
        frame_index=frame_index,
        robot_id=robot_id,
        squad_local_index=_squad_local_index(
            robot_id, int(config["number"]), int(config["parts"]),
        ),
        command_source=None if frame_index == 0 else frame_index - 1,
        command=applied_command,
        attempt_path=attempt_path,
        considered=considered,
        consideration_reason=reason,
        attempt=attempt,
        prior=prior_bundle.get("branch_selection_prior"),
        lifecycle=lifecycle,
        mandatory=mandatory,
        optional=optional,
        active_keys=list(qualification.get("active_keys", [])),
        provenance=list(qualification.get("base_anchor_provenance", ())),
        truth=truth_positions[robot_id],
        reference_evidence=evidence,
        reference_freshness=[
            {
                "reference_kind": record["reference_kind"],
                "reference_id": record["reference_id"],
                "current_freshness": record["current_freshness"],
            }
            for record in evidence
        ],
        excluded=_reason_mappings(qualification.get("excluded", [])),
        violations=_reason_mappings(qualification.get("violations", [])),
    )
    next_state = {
        "public_output": lifecycle["public_output"],
        "private_state": lifecycle["next_private_state"],
    }
    return row, next_state


def _canonical_file_identity(identity: Mapping) -> dict:
    return {
        field: identity[field]
        for field in FILE_IDENTITY_FIELDS
    }


def _read_pinned_json(path: Path) -> tuple[dict, bytes, dict]:
    payload, identity = _read_trusted_bytes(Path(path))
    if payload is None:
        raise RuntimeError("trusted JSON payload was not captured")
    return (
        _parse_json_object(payload, Path(path)),
        payload,
        _canonical_file_identity(identity),
    )


def _source_snapshots(
    *,
    invocation_name: str,
    data_path: Path,
    input_manifest_path: Path,
) -> tuple[dict, dict[str, dict]]:
    project = Path(__file__).resolve().parents[2]
    fixture_root = project / "tests/fixtures/cbf2026_two_range_reacquisition"
    paths = {
        "two_range_reacquisition_source": (
            project / "scripts/diagnostics/two_range_reacquisition.py"
        ),
        "predictive_wnls_source": (
            project / "scripts/diagnostics/predictive_wnls.py"
        ),
        "replay_source": Path(__file__).resolve(),
        "mechanism_fixture": (
            fixture_root / "mechanism_20260727_180_12.json"
        ),
        "mechanism_fixture_manifest": fixture_root / "manifest.json",
        "truth_data": Path(data_path),
        "input_manifest": Path(input_manifest_path),
    }
    identities = {}
    json_payloads = {}
    for name in RAW_SOURCE_MEMBER_NAMES[invocation_name]:
        capture = name in {"truth_data", "input_manifest"}
        payload, identity = _read_trusted_bytes(
            paths[name],
            capture_payload=capture,
        )
        identities[name] = _canonical_file_identity(identity)
        if capture:
            if payload is None:
                raise RuntimeError("source JSON payload was not captured")
            json_payloads[name] = _parse_json_object(payload, paths[name])
    return identities, json_payloads


def _git_resolve_commit(project_root: Path, commit: str) -> str:
    if not _lower_hex(commit, 40):
        raise ValueError("declared Git commit is not a full lowercase OID")
    result = subprocess.run(
        [
            "git", "-C", str(project_root), "rev-parse", "--verify",
            f"{commit}^{{commit}}",
        ],
        capture_output=True,
        check=False,
    )
    resolved = result.stdout.decode("ascii", errors="replace").strip()
    if result.returncode != 0 or resolved != commit:
        raise ValueError("declared Git commit does not resolve exactly")
    return resolved


def _git_blob_at(
    project_root: Path,
    commit: str,
    path: Path,
) -> bytes:
    try:
        relative = Path(path).resolve().relative_to(Path(project_root).resolve())
    except ValueError:
        raise ValueError("Git-bound path is outside the repository") from None
    result = subprocess.run(
        [
            "git", "-C", str(project_root), "show",
            f"{commit}:{relative.as_posix()}",
        ],
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        raise ValueError(f"Git blob is absent: {relative.as_posix()}")
    return result.stdout


def _pinned_gzip_hashes(
    path: Path,
    *,
    identity_sink: dict | None = None,
) -> tuple[str, str]:
    path = Path(path)
    _lstat_components(path, leaf_required=True)
    flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptor = os.open(path, flags)
    try:
        before = os.fstat(descriptor)
        compressed_digest = hashlib.sha256()
        offset = 0
        while offset < before.st_size:
            chunk = os.pread(
                descriptor,
                min(1024 * 1024, before.st_size - offset),
                offset,
            )
            if not chunk:
                raise ValueError("smoke process short read")
            compressed_digest.update(chunk)
            offset += len(chunk)
        os.lseek(descriptor, 0, os.SEEK_SET)
        decompressed_digest = hashlib.sha256()
        with os.fdopen(descriptor, "rb", closefd=False) as raw:
            with gzip.GzipFile(fileobj=raw, mode="rb") as stream:
                for line in stream:
                    decompressed_digest.update(line)
        compressed_sha256 = compressed_digest.hexdigest()
        decompressed_sha256 = decompressed_digest.hexdigest()
        after = os.fstat(descriptor)
        linked = os.stat(path, follow_symlinks=False)
        expected = (
            before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns,
        )
        if (
            (
                after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns,
            )
            != expected
            or (
                linked.st_dev, linked.st_ino, linked.st_size,
                linked.st_mtime_ns,
            )
            != expected
        ):
            raise ValueError("smoke process identity changed while hashing")
        if identity_sink is not None:
            identity_sink.clear()
            identity_sink.update({
                "path": str(path),
                "device": before.st_dev,
                "inode": before.st_ino,
                "size": before.st_size,
                "allocated_bytes": before.st_blocks * 512,
                "mtime_ns": before.st_mtime_ns,
                "compressed_sha256": compressed_sha256,
                "decompressed_sha256": decompressed_sha256,
            })
        return compressed_sha256, decompressed_sha256
    finally:
        os.close(descriptor)


def _revalidate_smoke_raw_record(record: Mapping) -> None:
    invocation = record["invocation"]
    manifest_path = record["manifest_path"]
    manifest_payload = record["manifest_payload"]
    manifest_identity = record["manifest_identity"]
    try:
        payload, observed_manifest = _read_trusted_bytes(
            manifest_path,
            expected_sha256=manifest_identity["sha256"],
        )
    except ValueError as error:
        raise ValueError(
            f"{invocation} manifest identity changed",
        ) from error
    if (
        payload != manifest_payload
        or _canonical_file_identity(observed_manifest) != manifest_identity
    ):
        raise ValueError(f"{invocation} manifest identity changed")
    for label, expected in (
        ("protocol", record["protocol_identity"]),
        *(
            (f"source {name}", identity)
            for name, identity in record["source_identities"].items()
        ),
    ):
        _, observed = _read_trusted_bytes(
            Path(expected["path"]),
            expected_sha256=expected["sha256"],
            capture_payload=False,
        )
        if _canonical_file_identity(observed) != expected:
            raise ValueError(f"{invocation} {label} identity changed")
    observed_process = {}
    _pinned_gzip_hashes(
        record["process_path"],
        identity_sink=observed_process,
    )
    if observed_process != record["process_identity"]:
        raise ValueError(f"{invocation} process identity changed")


def _validate_smoke_evidence_binding(
    protocol: Mapping,
    authorization: Mapping,
    *,
    expected_protocol_identity: Mapping | None = None,
) -> None:
    invocations = protocol.get("invocations")
    if not isinstance(invocations, Mapping):
        raise ValueError("protocol lacks smoke invocation bindings")
    declared_sources = protocol.get("sources")
    if not isinstance(declared_sources, Mapping):
        raise ValueError("protocol lacks registered source bindings")
    raw_records = []
    raw_fields = {
        "smoke_a": (
            "smoke_a_compressed_sha256",
            "smoke_a_decompressed_sha256",
        ),
        "smoke_b": (
            "smoke_b_compressed_sha256",
            "smoke_b_decompressed_sha256",
        ),
    }
    for invocation, fields in raw_fields.items():
        declaration = invocations.get(invocation)
        if not isinstance(declaration, Mapping):
            raise ValueError(f"protocol lacks {invocation}")
        root = Path(declaration["output_root"])
        manifest_path = root / "manifest.json"
        manifest, manifest_payload, manifest_identity = _read_pinned_json(
            manifest_path,
        )
        try:
            _validate_manifest(manifest)
        except ValueError as error:
            raise ValueError(
                f"{invocation} raw manifest differs from exact contract",
            ) from error
        process = manifest.get("process_identity")
        if not isinstance(process, Mapping):
            raise ValueError(f"{invocation} process identity is absent")
        process_path = root / RAW_PROCESS_NAME
        if (
            manifest["protocol_id"] != protocol.get("protocol_id")
            or manifest["output_root"] != str(root)
            or process["path"] != str(process_path)
        ):
            raise ValueError(f"{invocation} process path or protocol differs")
        protocol_identity = manifest["protocol_identity"]
        if (
            expected_protocol_identity is not None
            and protocol_identity != expected_protocol_identity
        ):
            raise ValueError(f"{invocation} protocol identity differs")
        if protocol_identity["sha256"] != authorization["protocol_sha256"]:
            raise ValueError(f"{invocation} protocol identity differs")
        _, observed_protocol = _read_trusted_bytes(
            Path(protocol_identity["path"]),
            expected_sha256=protocol_identity["sha256"],
            capture_payload=False,
        )
        if _canonical_file_identity(observed_protocol) != protocol_identity:
            raise ValueError(f"{invocation} protocol identity differs")
        source_identities = manifest["source_identities"]
        for name, source_identity in source_identities.items():
            if declared_sources.get(name) != source_identity:
                raise ValueError(f"{invocation} source identity differs")
            _, observed_source = _read_trusted_bytes(
                Path(source_identity["path"]),
                expected_sha256=source_identity["sha256"],
                capture_payload=False,
            )
            if (
                _canonical_file_identity(observed_source)
                != source_identity
            ):
                raise ValueError(f"{invocation} source identity differs")
        observed_process = {}
        observed_hashes = _pinned_gzip_hashes(
            process_path,
            identity_sink=observed_process,
        )
        if observed_process != process:
            raise ValueError(f"{invocation} process identity differs")
        if (
            manifest.get("status") != "completed"
            or manifest.get("invocation_name") != invocation
            or manifest.get("expected_rows") != 18
            or manifest.get("observed_rows") != 18
            or process.get("compressed_sha256") != authorization[fields[0]]
            or process.get("decompressed_sha256") != authorization[fields[1]]
            or observed_hashes
            != (
                authorization[fields[0]],
                authorization[fields[1]],
            )
        ):
            raise ValueError(f"{invocation} evidence differs from authorization")
        record = {
            "invocation": invocation,
            "manifest_path": manifest_path,
            "manifest_payload": manifest_payload,
            "manifest_identity": manifest_identity,
            "protocol_identity": protocol_identity,
            "source_identities": source_identities,
            "process_path": process_path,
            "process_identity": process,
        }
        _revalidate_smoke_raw_record(record)
        raw_records.append(record)
    analyzer_fields = {
        "smoke_analyzer_a": (
            "smoke_analyzer_a_json_sha256",
            "smoke_analyzer_a_markdown_sha256",
        ),
        "smoke_analyzer_b": (
            "smoke_analyzer_b_json_sha256",
            "smoke_analyzer_b_markdown_sha256",
        ),
    }
    semantic_hashes = []
    for invocation, fields in analyzer_fields.items():
        declaration = invocations.get(invocation)
        if not isinstance(declaration, Mapping):
            raise ValueError(f"protocol lacks {invocation}")
        root = Path(declaration["output_root"])
        manifest, _, _ = _read_pinned_json(root / "manifest.json")
        identities = manifest.get("output_identities")
        if manifest.get("status") != "completed" or not isinstance(
            identities, Mapping,
        ):
            raise ValueError(f"{invocation} manifest is not completed")
        artifacts = list(identities.values())
        json_identity = next(
            (
                item for item in artifacts
                if isinstance(item, Mapping)
                and str(item.get("path", "")).endswith(".json")
                and not str(item.get("path", "")).endswith("manifest.json")
            ),
            None,
        )
        markdown_identity = next(
            (
                item for item in artifacts
                if isinstance(item, Mapping)
                and str(item.get("path", "")).endswith(".md")
            ),
            None,
        )
        if (
            json_identity is None
            or markdown_identity is None
            or json_identity.get("sha256") != authorization[fields[0]]
            or markdown_identity.get("sha256") != authorization[fields[1]]
        ):
            raise ValueError(f"{invocation} artifacts differ from authorization")
        result, _, observed = _read_pinned_json(Path(json_identity["path"]))
        if observed["sha256"] != json_identity["sha256"]:
            raise ValueError(f"{invocation} JSON identity changed")
        _, markdown_observed = _read_trusted_bytes(
            Path(markdown_identity["path"]),
            expected_sha256=markdown_identity["sha256"],
            capture_payload=False,
        )
        if markdown_observed["sha256"] != authorization[fields[1]]:
            raise ValueError(f"{invocation} Markdown identity changed")
        semantic_hashes.append(result.get("semantic_payload_sha256"))
    if (
        semantic_hashes
        != [authorization["smoke_semantic_payload_sha256"]] * 2
    ):
        raise ValueError("smoke semantic payload differs from authorization")
    for record in raw_records:
        _revalidate_smoke_raw_record(record)


def _validate_committed_registered_state(
    *,
    project_root: Path,
    protocol_path: Path,
    protocol_payload: bytes,
    protocol: Mapping,
    protocol_identity: Mapping,
    authorization_path: Path,
    authorization_payload: bytes,
    authorization: Mapping,
    authorization_identity: Mapping,
    sources: Mapping,
) -> None:
    commits = {
        name: _git_resolve_commit(project_root, authorization[name])
        for name in ("protocol_commit", "preflight_commit", "smoke_commit")
    }
    implementation_commit = _git_resolve_commit(
        project_root,
        protocol.get("implementation_parent_commit"),
    )
    if _git_blob_at(
        project_root, commits["protocol_commit"], protocol_path,
    ) != protocol_payload:
        raise ValueError("protocol pinned bytes are not the declared Git blob")
    head = subprocess.run(
        ["git", "-C", str(project_root), "rev-parse", "--verify", "HEAD"],
        capture_output=True,
        check=False,
    )
    if head.returncode != 0:
        raise ValueError("repository HEAD cannot be resolved")
    head_commit = head.stdout.decode("ascii", errors="replace").strip()
    if _git_blob_at(
        project_root, head_commit, authorization_path,
    ) != authorization_payload:
        raise ValueError("authorization pinned bytes are not committed at HEAD")
    tracked_paths = [
        protocol_path,
        authorization_path,
        Path(project_root) / REGISTERED_PREFLIGHT_REVIEW_RELATIVE_PATH,
        Path(project_root) / REGISTERED_SMOKE_REPORT_RELATIVE_PATH,
        *(
            Path(identity["path"])
            for identity in sources.values()
        ),
    ]
    relative_paths = []
    for path in tracked_paths:
        try:
            relative_paths.append(
                str(Path(path).resolve().relative_to(Path(project_root).resolve())),
            )
        except ValueError:
            continue
    dirty = subprocess.run(
        [
            "git", "-C", str(project_root), "status", "--porcelain",
            "--untracked-files=no", "--", *relative_paths,
        ],
        capture_output=True,
        check=False,
    )
    if dirty.returncode != 0 or dirty.stdout:
        raise ValueError("authorization-related tracked paths are dirty")
    for name, identity in sources.items():
        path = Path(identity["path"])
        try:
            path.resolve().relative_to(Path(project_root).resolve())
        except ValueError:
            continue
        blob = _git_blob_at(project_root, implementation_commit, path)
        if hashlib.sha256(blob).hexdigest() != identity["sha256"]:
            raise ValueError(f"source Git blob differs: {name}")
    preflight_path = (
        Path(project_root) / REGISTERED_PREFLIGHT_REVIEW_RELATIVE_PATH
    )
    smoke_path = Path(project_root) / REGISTERED_SMOKE_REPORT_RELATIVE_PATH
    preflight_payload, _ = _read_trusted_bytes(preflight_path)
    smoke_payload, _ = _read_trusted_bytes(smoke_path)
    if (
        preflight_payload is None
        or _git_blob_at(
            project_root, commits["preflight_commit"], preflight_path,
        )
        != preflight_payload
    ):
        raise ValueError("preflight approval is not the declared Git blob")
    if (
        smoke_payload is None
        or _git_blob_at(
            project_root, commits["smoke_commit"], smoke_path,
        )
        != smoke_payload
    ):
        raise ValueError("smoke report is not the declared Git blob")
    for field in (
        "smoke_a_compressed_sha256", "smoke_a_decompressed_sha256",
        "smoke_b_compressed_sha256", "smoke_b_decompressed_sha256",
        "smoke_analyzer_a_json_sha256",
        "smoke_analyzer_a_markdown_sha256",
        "smoke_analyzer_b_json_sha256",
        "smoke_analyzer_b_markdown_sha256",
        "smoke_semantic_payload_sha256",
    ):
        if authorization[field].encode("ascii") not in smoke_payload:
            raise ValueError(f"smoke report does not bind {field}")
    _validate_smoke_evidence_binding(
        protocol,
        authorization,
        expected_protocol_identity=protocol_identity,
    )


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _write_all(descriptor: int, payload: bytes) -> None:
    view = memoryview(payload)
    while view:
        written = os.write(descriptor, view)
        if written <= 0:
            raise OSError("short evidence write")
        view = view[written:]


def _manifest_inode(metadata: os.stat_result) -> tuple[int, int]:
    return metadata.st_dev, metadata.st_ino


def _rename_noreplace(
    source: str,
    destination: str,
    *,
    src_dir_fd: int,
    dst_dir_fd: int,
) -> None:
    try:
        libc = ctypes.CDLL(None, use_errno=True)
    except BaseException as error:
        raise OSError(
            errno.ENOSYS,
            "atomic no-replace rename libc is unavailable",
        ) from error
    if sys.platform == "darwin":
        symbol_name = "renameatx_np"
        flags = 0x4
    elif sys.platform.startswith("linux"):
        symbol_name = "renameat2"
        flags = 0x1
    else:
        raise OSError(
            errno.ENOSYS,
            f"atomic no-replace rename is unsupported on {sys.platform}",
        )
    try:
        primitive = getattr(libc, symbol_name)
    except AttributeError as error:
        raise OSError(
            errno.ENOSYS,
            f"libc symbol {symbol_name} is unavailable",
        ) from error
    primitive.argtypes = [
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    ]
    primitive.restype = ctypes.c_int
    ctypes.set_errno(0)
    result = primitive(
        src_dir_fd,
        os.fsencode(source),
        dst_dir_fd,
        os.fsencode(destination),
        flags,
    )
    if result != 0:
        error_number = ctypes.get_errno() or errno.EIO
        raise OSError(
            error_number,
            f"{os.strerror(error_number)}: "
            f"{source!r} -> {destination!r}",
        )


def _fsync_manifest_removal(
    transaction: Mapping,
    quarantine: str,
) -> None:
    failures = []
    for label, descriptor in (
        ("root", transaction["root_fd"]),
        ("parent", transaction["parent_fd"]),
    ):
        try:
            os.fsync(descriptor)
        except BaseException as error:
            failures.append((label, error))
    if failures:
        transaction["manifest_retraction_indeterminate"] = True
        durability_error = OSError(
            "manifest retraction durability is indeterminate; "
            f"forensic quarantine: {quarantine}",
        )
        for label, error in failures:
            durability_error.add_note(
                f"{label} fsync failed: "
                f"{type(error).__name__}: {error}",
            )
        raise durability_error from failures[0][1]


def _remove_manifest_identity(
    transaction: Mapping,
    name: str,
    expected_identity: tuple[int, int],
) -> tuple[bool, str | None]:
    try:
        descriptor = os.open(
            name,
            os.O_RDONLY
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=transaction["root_fd"],
        )
    except FileNotFoundError:
        return False, None
    try:
        held = os.fstat(descriptor)
        if (
            not stat.S_ISREG(held.st_mode)
            or _manifest_inode(held) != expected_identity
        ):
            return False, None
        quarantine = f".{name}.quarantine.{secrets.token_hex(16)}"
        try:
            _rename_noreplace(
                name,
                quarantine,
                src_dir_fd=transaction["root_fd"],
                dst_dir_fd=transaction["root_fd"],
            )
        except BaseException:
            transaction["manifest_publication_blocked"] = True
            raise
        try:
            moved = os.stat(
                quarantine,
                dir_fd=transaction["root_fd"],
                follow_symlinks=False,
            )
            held_after = os.fstat(descriptor)
        except BaseException as identity_error:
            try:
                _fsync_manifest_removal(transaction, quarantine)
            except BaseException as sync_error:
                identity_error.add_note(
                    "manifest quarantine identity fsync failed: "
                    f"{type(sync_error).__name__}: {sync_error}",
                )
            identity_error.add_note(
                f"forensic quarantine preserved: {quarantine}",
            )
            raise
        owned = (
            stat.S_ISREG(moved.st_mode)
            and stat.S_ISREG(held_after.st_mode)
            and _manifest_inode(moved) == expected_identity
            and _manifest_inode(held_after) == expected_identity
        )
        if owned:
            _fsync_manifest_removal(transaction, quarantine)
            return True, quarantine
        try:
            os.link(
                quarantine,
                name,
                src_dir_fd=transaction["root_fd"],
                dst_dir_fd=transaction["root_fd"],
                follow_symlinks=False,
            )
        except FileExistsError as restore_error:
            conflict = RuntimeError(
                "foreign manifest preserved in quarantine: "
                f"{quarantine}",
            )
            try:
                _fsync_manifest_removal(transaction, quarantine)
            except BaseException as sync_error:
                conflict.add_note(
                    "foreign manifest preservation fsync failed: "
                    f"{type(sync_error).__name__}: {sync_error}",
                )
            raise conflict from restore_error
        except BaseException as restore_error:
            try:
                _fsync_manifest_removal(transaction, quarantine)
            except BaseException as sync_error:
                restore_error.add_note(
                    "manifest quarantine restore fsync failed: "
                    f"{type(sync_error).__name__}: {sync_error}",
                )
            restore_error.add_note(
                f"foreign manifest preserved in quarantine: {quarantine}",
            )
            raise
        _fsync_manifest_removal(transaction, quarantine)
        return False, quarantine
    finally:
        os.close(descriptor)


def _publish_manifest(
    transaction: Mapping,
    manifest: Mapping,
) -> tuple[int, int]:
    _validate_manifest(manifest)
    payload = ordered_strict_json_bytes(manifest, RAW_MANIFEST_FIELDS) + b"\n"
    name = f".manifest.{os.getpid()}.{secrets.token_hex(8)}"
    descriptor = os.open(
        name,
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0),
        0o600,
        dir_fd=transaction["root_fd"],
    )
    staged_identity = None
    try:
        _write_all(descriptor, payload)
        os.fsync(descriptor)
        metadata = os.fstat(descriptor)
        staged_identity = (metadata.st_dev, metadata.st_ino)
    finally:
        os.close(descriptor)
    renamed = False
    try:
        os.rename(
            name,
            "manifest.json",
            src_dir_fd=transaction["root_fd"],
            dst_dir_fd=transaction["root_fd"],
        )
        renamed = True
        os.fsync(transaction["root_fd"])
        os.fsync(transaction["parent_fd"])
    except BaseException as error:
        target = "manifest.json" if renamed else name
        try:
            removed, quarantine = _remove_manifest_identity(
                transaction,
                target,
                staged_identity,
            )
            if quarantine is not None:
                error.add_note(
                    f"manifest forensic quarantine: {quarantine}",
                )
            if not removed:
                transaction["manifest_publication_blocked"] = True
                error.add_note(
                    "manifest cleanup preserved a non-owned entry: "
                    f"{target}",
                )
        except BaseException as cleanup_error:
            transaction["manifest_publication_blocked"] = True
            error.add_note(
                "manifest cleanup failed: "
                f"{type(cleanup_error).__name__}: {cleanup_error}",
            )
        raise
    if staged_identity is None:
        raise RuntimeError("published manifest identity was not captured")
    return staged_identity


def _retract_manifest_identity(
    transaction: Mapping,
    expected_identity: tuple[int, int],
) -> tuple[bool, str | None]:
    return _remove_manifest_identity(
        transaction,
        "manifest.json",
        expected_identity,
    )


def _emergency_manifest(
    transaction: Mapping,
    manifest: Mapping,
    error: BaseException,
) -> None:
    record = {
        **manifest,
        "status": "failed",
        "completed_at": _utc_now(),
        "error": {
            "type": type(error).__name__,
            "message": str(error),
        },
    }
    payload = ordered_strict_json_bytes(record, RAW_MANIFEST_FIELDS) + b"\n"
    name = f"manifest.failed.emergency.{os.getpid()}.json"
    try:
        descriptor = os.open(
            name,
            os.O_WRONLY
            | os.O_CREAT
            | os.O_EXCL
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NOFOLLOW", 0),
            0o600,
            dir_fd=transaction["root_fd"],
        )
        try:
            view = memoryview(payload)
            while view:
                written = os.write(descriptor, view)
                if written <= 0:
                    break
                view = view[written:]
        finally:
            os.close(descriptor)
    except BaseException as emergency_error:
        error.add_note(
            "emergency manifest failed: "
            f"{type(emergency_error).__name__}: {emergency_error}"
        )


def _preallocation_failure_path(output_root: Path) -> Path:
    digest = hashlib.sha256(str(output_root).encode("utf-8")).hexdigest()[:16]
    current = Path("/")
    safe_ancestor = current
    for component in output_root.parts[1:-1]:
        candidate = current / component
        try:
            metadata = candidate.lstat()
        except FileNotFoundError:
            break
        if candidate.is_symlink() or not candidate.is_dir():
            break
        safe_ancestor = candidate
        current = candidate
    return safe_ancestor / (
        f".{output_root.name}.{digest}.manifest.failed.json"
    )


def _publish_preallocation_failure(
    output_root: Path,
    manifest: Mapping,
) -> Path:
    _validate_manifest(manifest)
    target = _preallocation_failure_path(output_root)
    payload = ordered_strict_json_bytes(manifest, RAW_MANIFEST_FIELDS) + b"\n"
    descriptor = os.open(
        target,
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0),
        0o600,
    )
    try:
        _write_all(descriptor, payload)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return target


def _open_process(transaction: Mapping) -> int:
    descriptor = os.open(
        RAW_PROCESS_NAME,
        os.O_RDWR
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0),
        0o600,
        dir_fd=transaction["root_fd"],
    )
    transaction["resource_fds"].add(descriptor)
    return descriptor


def _process_identity(
    descriptor: int,
    *,
    output_root: Path,
    decompressed_sha256: str,
) -> dict:
    metadata = os.fstat(descriptor)
    digest = hashlib.sha256()
    offset = 0
    while offset < metadata.st_size:
        chunk = os.pread(descriptor, min(1024 * 1024, metadata.st_size - offset), offset)
        if not chunk:
            raise ValueError("raw process short read during final identity")
        digest.update(chunk)
        offset += len(chunk)
    return {
        "path": str(output_root / RAW_PROCESS_NAME),
        "device": metadata.st_dev,
        "inode": metadata.st_ino,
        "size": metadata.st_size,
        "allocated_bytes": metadata.st_blocks * 512,
        "mtime_ns": metadata.st_mtime_ns,
        "compressed_sha256": digest.hexdigest(),
        "decompressed_sha256": decompressed_sha256,
    }


def _allocated_bytes_live(descriptor: int) -> int:
    return os.fstat(descriptor).st_blocks * 512


def _check_live_resource_limits(
    transaction: Mapping,
    descriptor: int,
    *,
    live_floor: int,
    raw_cap: int,
) -> None:
    filesystem = os.fstatvfs(transaction["root_fd"])
    if filesystem.f_bavail * filesystem.f_frsize < live_floor:
        raise OSError("available bytes below live floor")
    if _allocated_bytes_live(descriptor) > raw_cap:
        raise OSError("raw bundle exceeds allocated-byte cap")


def _verify_process_link(
    transaction: Mapping,
    process_identity: Mapping,
) -> None:
    linked = os.stat(
        RAW_PROCESS_NAME,
        dir_fd=transaction["root_fd"],
        follow_symlinks=False,
    )
    if (
        linked.st_dev != process_identity["device"]
        or linked.st_ino != process_identity["inode"]
        or linked.st_size != process_identity["size"]
        or linked.st_mtime_ns != process_identity["mtime_ns"]
    ):
        raise ValueError("final raw-process identity mismatch")


def _key_contract(invocation_name: str) -> dict:
    production = invocation_name == "registered_replay"
    return {
        "method": METHOD_ID,
        "seeds": list(range(20260727, 20260747)) if production else [],
        "frames": [0, 499] if production else [],
        "robots": [1, 14] if production else [],
        "smoke_case_ids": (
            [] if production
            else (
                [SMOKE_CASE_IDS[0]]
                if invocation_name == "unit_fixture"
                else list(SMOKE_CASE_IDS)
            )
        ),
        "order": (
            "method,seed,frame_index,ascending_global_robot_id"
            if production else "SMOKE_CASE_IDS"
        ),
    }


def _manifest(
    *,
    protocol_id: str,
    invocation_name: str,
    status: str,
    output_root: Path,
    protocol_identity: object,
    sources: Mapping,
    authorization_identity: object,
    process_identity: object,
    expected_rows: int,
    observed_rows: int,
    disk_contract: Mapping,
    started_at: str,
    completed_at: str | None,
    error: object,
) -> dict:
    return {
        "schema_id": RAW_SCHEMA_ID,
        "protocol_id": protocol_id,
        "invocation_name": invocation_name,
        "status": status,
        "method": METHOD_ID,
        "output_root": str(output_root),
        "protocol_identity": protocol_identity,
        "source_identities": sources,
        "authorization_identity": authorization_identity,
        "process_identity": process_identity,
        "synthetic_declaration_sha256": SYNTHETIC_DECLARATION_SHA256,
        "expected_rows": expected_rows,
        "observed_rows": observed_rows,
        "key_contract": _key_contract(invocation_name),
        "disk_contract": _builder_json_value(disk_contract),
        "started_at": started_at,
        "completed_at": completed_at,
        "error": error,
    }


def _validate_manifest(manifest: Mapping) -> None:
    if tuple(manifest) != RAW_MANIFEST_FIELDS:
        raise ValueError("raw manifest differs from exact field order")
    invocation = manifest["invocation_name"]
    if (
        manifest["schema_id"] != RAW_SCHEMA_ID
        or invocation not in PRODUCER_INVOCATIONS
        or manifest["status"] not in {
            "creating", "running", "completed", "failed",
        }
        or manifest["method"] != METHOD_ID
        or manifest["synthetic_declaration_sha256"]
        != SYNTHETIC_DECLARATION_SHA256
        or not isinstance(manifest["protocol_id"], str)
        or not isinstance(manifest["output_root"], str)
    ):
        raise ValueError("raw manifest scalar discriminant is invalid")
    sources = manifest["source_identities"]
    if (
        not isinstance(sources, Mapping)
        or tuple(sources) != RAW_SOURCE_MEMBER_NAMES[invocation]
        or any(
            not isinstance(identity, Mapping)
            or tuple(identity) != FILE_IDENTITY_FIELDS
            for identity in sources.values()
        )
    ):
        raise ValueError("raw manifest sources differ from exact contract")
    for name, identity in sources.items():
        suffix = _SOURCE_SUFFIXES.get(name)
        if suffix is not None and not identity["path"].endswith(suffix):
            raise ValueError("raw manifest source member is substituted")
    protocol_identity = manifest["protocol_identity"]
    if invocation == "unit_fixture":
        if protocol_identity is not None:
            raise ValueError("unit fixture must not carry protocol identity")
    elif (
        not isinstance(protocol_identity, Mapping)
        or tuple(protocol_identity) != FILE_IDENTITY_FIELDS
    ):
        raise ValueError("invocation requires canonical protocol identity")
    authorization = manifest["authorization_identity"]
    if invocation == "registered_replay":
        if (
            not isinstance(authorization, Mapping)
            or tuple(authorization) != FILE_IDENTITY_FIELDS
        ):
            raise ValueError("registered replay requires authorization identity")
    elif authorization is not None:
        raise ValueError("non-production invocation carries authorization")
    process = manifest["process_identity"]
    if process is not None and (
        not isinstance(process, Mapping)
        or tuple(process) != PROCESS_IDENTITY_FIELDS
    ):
        raise ValueError("process identity differs from exact contract")
    if manifest["status"] == "completed" and process is None:
        raise ValueError("completed manifest lacks process identity")
    for field in ("expected_rows", "observed_rows"):
        value = manifest[field]
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError("row count must be a nonnegative integer")
    key_contract = manifest["key_contract"]
    if (
        not isinstance(key_contract, Mapping)
        or tuple(key_contract) != RAW_KEY_CONTRACT_FIELDS
        or key_contract != _key_contract(invocation)
    ):
        raise ValueError("manifest key contract differs from invocation")
    if not isinstance(manifest["disk_contract"], Mapping):
        raise ValueError("manifest disk contract must be an object")
    for field in ("started_at", "completed_at"):
        value = manifest[field]
        if value is None and field == "completed_at":
            continue
        if not isinstance(value, str):
            raise ValueError("manifest timestamp is invalid")
        parsed = datetime.fromisoformat(value)
        if parsed.tzinfo != timezone.utc:
            raise ValueError("manifest timestamp must be canonical UTC")
    terminal = manifest["status"] in {"completed", "failed"}
    if terminal != (manifest["completed_at"] is not None):
        raise ValueError("terminal timestamp differs from status")
    error = manifest["error"]
    if manifest["status"] == "completed":
        if error is not None:
            raise ValueError("completed manifest carries an error")
    elif manifest["status"] == "failed":
        if (
            not isinstance(error, Mapping)
            or tuple(error) != ("type", "message")
            or not all(isinstance(value, str) for value in error.values())
        ):
            raise ValueError("failed manifest lacks canonical error")
    elif error is not None:
        raise ValueError("nonterminal manifest carries an error")


def _load_fixture(
    *,
    fixture_path: Path | None = None,
    manifest_path: Path | None = None,
) -> dict:
    project = Path(__file__).resolve().parents[2]
    fixture_root = project / "tests/fixtures/cbf2026_two_range_reacquisition"
    fixture_path = (
        fixture_root / "mechanism_20260727_180_12.json"
        if fixture_path is None else Path(fixture_path)
    )
    manifest_path = (
        fixture_root / "manifest.json"
        if manifest_path is None else Path(manifest_path)
    )
    manifest = _strict_load(manifest_path)
    if (
        tuple(manifest) != FIXTURE_MANIFEST_FIELDS
        or manifest["schema_id"] != FIXTURE_MANIFEST_SCHEMA_ID
        or manifest["fixture_id"] != MECHANISM_FIXTURE_ID
        or manifest["fixture_file"] != fixture_path.name
        or manifest["fixture_sha256"] != MECHANISM_FIXTURE_SHA256
        or not _strict_int(manifest["fixture_size"])
        or manifest["approved_design_commit"] != APPROVED_DESIGN_COMMIT
    ):
        raise ValueError("fixture manifest differs from frozen contract")
    source_hashes = manifest["source_sha256"]
    expected_source_hashes = {
        "v4_manifest": (
            "123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223"
        ),
        "v4_compressed_process": (
            "9710d1a5792e682bb272d0bb3305162180d1e0b9a0636647d6e071d1782a699b"
        ),
        "v4_decompressed_process": (
            "7047989804fd03bd4cadffac8212349b7183857623ac201d14ef2fc3e30ee2e1"
        ),
        "truth_data": (
            "3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527"
        ),
    }
    if source_hashes != expected_source_hashes:
        raise ValueError("fixture manifest source hashes differ")
    extractor = manifest["extractor_identity"]
    extractor_path = project / "scripts/diagnostics/extract_two_range_reacquisition_fixture.py"
    if (
        not isinstance(extractor, Mapping)
        or tuple(extractor) != ("path", "sha256")
        or extractor["path"]
        != "scripts/diagnostics/extract_two_range_reacquisition_fixture.py"
        or extractor["sha256"] != _file_identity(extractor_path)["sha256"]
    ):
        raise ValueError("fixture manifest extractor identity differs")
    payload, identity = _read_trusted_bytes(
        fixture_path,
        expected_sha256=manifest["fixture_sha256"],
    )
    if payload is None:
        raise RuntimeError("fixture payload was not captured")
    if identity["size"] != manifest["fixture_size"]:
        raise ValueError("fixture size differs from committed manifest")
    fixture = _parse_json_object(payload, fixture_path)
    if (
        tuple(fixture)
        != (
            "schema_id", "fixture_id", "source_identities", "key",
            "mandatory_references", "optional_references",
            "current_reference_outputs", "measurements",
            "preceding_public_output", "preceding_private_state",
            "held_command", "expected_mechanism",
        )
        or fixture["schema_id"] != FIXTURE_SCHEMA_ID
        or fixture["fixture_id"] != MECHANISM_FIXTURE_ID
    ):
        raise ValueError("fixture differs from exact schema")
    return fixture


def _registered_rows(
    *,
    data: Mapping,
    run_seeds: tuple[int, ...],
    max_frames: int,
    ranging_sigma: float,
):
    frames, commands = _preflight_frames(data)
    if len(frames) < max_frames:
        raise ValueError("truth data has fewer than 500 frames")
    config = data["config"]
    producer_config = {
        **config,
        "number": int(config["num"]),
        "parts": int(config["formation"]["parts"]),
    }
    expected_ids = set(range(1, producer_config["number"] + 1))
    for seed in run_seeds:
        method_state: dict[int, dict] = {}
        for frame_index, frame in enumerate(frames[:max_frames]):
            truth = _truth_positions(frame, expected_ids)
            current_public: dict[int, dict] = {}
            for robot_id in range(1, producer_config["number"] + 1):
                row, next_state = produce_method_row(
                    seed=seed,
                    frame_index=frame_index,
                    robot_id=robot_id,
                    config=producer_config,
                    truth_positions=truth,
                    current_public=current_public,
                    previous_state=method_state.get(robot_id),
                    applied_command=(
                        None if frame_index == 0
                        else commands[frame_index - 1][robot_id]
                    ),
                    ranging_sigma=ranging_sigma,
                )
                method_state[robot_id] = next_state
                current_public[robot_id] = next_state["public_output"]
                yield row


def replay_two_range_reacquisition(
    *,
    protocol_path: Path,
    data_path: Path,
    input_manifest_path: Path,
    output_root: Path,
    run_seeds: tuple[int, ...],
    max_frames: int,
    invocation_name: str,
    authorization_json: Path | None = None,
) -> Path:
    if invocation_name not in PRODUCER_INVOCATIONS:
        raise ValueError("invocation name is not registered")
    def runtime_path(value: Path) -> Path:
        path = Path(value)
        if not path.is_absolute():
            path = Path.cwd() / path
        if path.parts[:2] == ("/", "var"):
            path = Path("/private").joinpath(*path.parts[1:])
        if ".." in path.parts:
            raise ValueError("runtime path must be normalized")
        return path

    protocol_path = runtime_path(protocol_path)
    data_path = runtime_path(data_path)
    input_manifest_path = runtime_path(input_manifest_path)
    authorization_json = (
        None if authorization_json is None
        else runtime_path(authorization_json)
    )
    output_root = runtime_path(output_root)
    if invocation_name == "registered_replay":
        if (
            run_seeds != tuple(range(20260727, 20260747))
            or max_frames != 500
        ):
            raise ValueError("registered replay requires the exact 140000-key grid")
        if authorization_json is None:
            raise ValueError("registered replay requires authorization")
        project = Path(__file__).resolve().parents[2]
        if (
            protocol_path != project / REGISTERED_PROTOCOL_RELATIVE_PATH
            or authorization_json
            != project / REGISTERED_AUTHORIZATION_RELATIVE_PATH
            or str(output_root) != REGISTERED_REPLAY_ROOT
        ):
            raise ValueError(
                "registered replay requires exact committed paths and root",
            )
    elif run_seeds or max_frames not in (0, 1):
        raise ValueError("non-production invocation rejects trajectory grids")
    protocol, protocol_payload, observed_protocol_identity = (
        _read_pinned_json(protocol_path)
    )
    protocol_id = protocol.get("protocol_id")
    if not isinstance(protocol_id, str):
        raise ValueError("protocol must declare protocol_id")
    disk_contract = protocol.get("disk_contract")
    if not isinstance(disk_contract, Mapping):
        raise ValueError("protocol must declare disk_contract")
    registered_ranging_sigma = None
    if invocation_name == "registered_replay":
        experiment = protocol.get("experiment")
        if not isinstance(experiment, Mapping):
            raise ValueError(
                "registered protocol experiment.ranging_sigma_m "
                "differs from exact binding",
            )
        registered_ranging_sigma = experiment.get("ranging_sigma_m")
        if (
            type(registered_ranging_sigma) is not float
            or not math.isfinite(registered_ranging_sigma)
            or registered_ranging_sigma != 0.5
        ):
            raise ValueError(
                "registered protocol experiment.ranging_sigma_m "
                "differs from exact binding",
            )
    protocol_identity = (
        None if invocation_name == "unit_fixture"
        else observed_protocol_identity
    )
    sources, source_payloads = _source_snapshots(
        invocation_name=invocation_name,
        data_path=data_path,
        input_manifest_path=input_manifest_path,
    )
    declared_sources = protocol.get("sources")
    method_contract = protocol.get("method_contract")
    if invocation_name == "registered_replay" and (
        protocol.get("schema_id") != REGISTERED_PROTOCOL_SCHEMA_ID
        or protocol_id != REGISTERED_PROTOCOL_ID
        or declared_sources is None
        or not isinstance(method_contract, Mapping)
        or method_contract.get("synthetic_declaration_sha256")
        != SYNTHETIC_DECLARATION_SHA256
    ):
        raise ValueError(
            "registered protocol lacks frozen schema, sources, or declaration",
        )
    if declared_sources is not None:
        expected_declared_names = (
            REGISTERED_PROTOCOL_SOURCE_NAMES
            if invocation_name == "registered_replay"
            else tuple(sources)
        )
        if (
            not isinstance(declared_sources, Mapping)
            or tuple(declared_sources) != expected_declared_names
        ):
            raise ValueError("protocol source members differ from contract")
        if invocation_name == "registered_replay":
            for name, declaration in declared_sources.items():
                if (
                    not isinstance(declaration, Mapping)
                    or tuple(declaration) != FILE_IDENTITY_FIELDS
                ):
                    raise ValueError(
                        f"protocol source identity differs: {name}",
                    )
                observed_identity = sources.get(name)
                if observed_identity is None:
                    _, observed = _read_trusted_bytes(
                        Path(declaration["path"]),
                        capture_payload=False,
                    )
                    observed_identity = _canonical_file_identity(observed)
                if declaration != observed_identity:
                    raise ValueError(
                        f"protocol source identity differs: {name}",
                    )
        for name, identity in sources.items():
            declaration = declared_sources[name]
            if not isinstance(declaration, Mapping):
                raise ValueError(f"protocol source is not an object: {name}")
            if invocation_name == "registered_replay":
                matches = (
                    tuple(declaration) == FILE_IDENTITY_FIELDS
                    and declaration == identity
                )
            else:
                matches = (
                    tuple(declaration) == ("path", "sha256")
                    and declaration["path"] == identity["path"]
                    and declaration["sha256"] == identity["sha256"]
                )
            if not matches:
                raise ValueError(
                    f"protocol source identity differs: {name}"
                )
    authorization_identity = None
    authorization_payload = None
    authorization = None
    if authorization_json is not None:
        (
            authorization,
            authorization_payload,
            authorization_identity,
        ) = _read_pinned_json(
            authorization_json,
        )
        authorization_text = authorization.get("user_authorization_text")
        sha_fields = (
            "protocol_sha256", "smoke_a_compressed_sha256",
            "smoke_a_decompressed_sha256", "smoke_b_compressed_sha256",
            "smoke_b_decompressed_sha256",
            "smoke_analyzer_a_json_sha256",
            "smoke_analyzer_a_markdown_sha256",
            "smoke_analyzer_b_json_sha256",
            "smoke_analyzer_b_markdown_sha256",
            "smoke_semantic_payload_sha256",
            "user_authorization_text_sha256",
        )
        commit_fields = (
            "protocol_commit", "preflight_commit", "smoke_commit",
        )
        if invocation_name == "registered_replay" and (
            tuple(authorization) != REGISTERED_AUTHORIZATION_FIELDS
            or authorization["schema_id"]
            != REGISTERED_AUTHORIZATION_SCHEMA_ID
            or authorization["protocol_id"] != protocol_id
            or authorization["protocol_sha256"]
            != protocol_identity["sha256"]
            or any(
                not _lower_hex(authorization[field], 64)
                for field in sha_fields
            )
            or any(
                not _lower_hex(authorization[field], 40)
                for field in commit_fields
            )
            or not isinstance(authorization_text, str)
            or not authorization_text
            or hashlib.sha256(
                authorization_text.encode("utf-8"),
            ).hexdigest()
            != authorization["user_authorization_text_sha256"]
            or not _canonical_iso_date(
                authorization["user_authorization_date"],
            )
            or authorization["registered_replay_root"] != str(output_root)
            or authorization["registered_analyzer_root"]
            != REGISTERED_ANALYZER_ROOT
            or authorization["registered_retry_allowed"] is not False
        ):
            raise ValueError(
                "registered authorization differs from exact binding",
            )
        if invocation_name == "registered_replay":
            _validate_committed_registered_state(
                project_root=Path(__file__).resolve().parents[2],
                protocol_path=protocol_path,
                protocol_payload=protocol_payload,
                protocol=protocol,
                protocol_identity=protocol_identity,
                authorization_path=authorization_json,
                authorization_payload=authorization_payload,
                authorization=authorization,
                authorization_identity=authorization_identity,
                sources=declared_sources,
            )
    expected_rows = (
        140000 if invocation_name == "registered_replay"
        else 1 if invocation_name == "unit_fixture" else 18
    )
    launch_floor, live_floor, raw_cap = _disk_limits(
        disk_contract,
        registered=invocation_name == "registered_replay",
    )
    started_at = _utc_now()
    raw_descriptor = None
    observed = 0
    manifest = _manifest(
        protocol_id=protocol_id,
        invocation_name=invocation_name,
        status="creating",
        output_root=output_root,
        protocol_identity=protocol_identity,
        sources=sources,
        authorization_identity=authorization_identity,
        process_identity=None,
        expected_rows=expected_rows,
        observed_rows=0,
        disk_contract=disk_contract,
        started_at=started_at,
        completed_at=None,
        error=None,
    )
    transaction = None
    fixture = None
    completed_manifest_identity = None
    try:
        if invocation_name != "registered_replay":
            fixture = _load_fixture()
        ancestor = output_root.parent
        while not ancestor.exists():
            ancestor = ancestor.parent
        filesystem = os.statvfs(ancestor)
        free_at_start = filesystem.f_bavail * filesystem.f_frsize
        if free_at_start < launch_floor:
            raise OSError("available bytes below start floor")
        transaction = _create_exact_root(output_root)
    except BaseException as error:
        failed = {
            **manifest,
            "status": "failed",
            "completed_at": _utc_now(),
            "error": {
                "type": type(error).__name__,
                "message": str(error),
            },
        }
        _publish_preallocation_failure(output_root, failed)
        raise
    try:
        _publish_manifest(transaction, manifest)
        current_free = os.fstatvfs(transaction["root_fd"])
        if current_free.f_bavail * current_free.f_frsize < live_floor:
            raise OSError("available bytes below live floor")
        raw_descriptor = _open_process(transaction)
        if invocation_name == "unit_fixture":
            unit_row = produce_smoke_row(
                case_id=SMOKE_CASE_IDS[0],
                mechanism_fixture=fixture,
            )
            unit_row["invocation_name"] = "unit_fixture"
            unit_row = {field: unit_row[field] for field in ROW_FIELDS}
            _validate_row(unit_row)
            rows = (unit_row,)
        elif invocation_name in {"smoke_a", "smoke_b"}:
            rows = (
                produce_smoke_row(
                    case_id=case_id,
                    mechanism_fixture=fixture,
                )
                for case_id in SMOKE_CASE_IDS
            )
        else:
            rows = _registered_rows(
                data=source_payloads["truth_data"],
                run_seeds=run_seeds,
                max_frames=max_frames,
                ranging_sigma=registered_ranging_sigma,
            )
        decompressed_digest = hashlib.sha256()
        expected_keys = (
            iter(
                (
                    METHOD_ID, seed, frame_index, robot_id,
                )
                for seed in range(20260727, 20260747)
                for frame_index in range(500)
                for robot_id in range(1, 15)
            )
            if invocation_name == "registered_replay" else None
        )
        expected_smoke_ids = (
            iter(
                (SMOKE_CASE_IDS[0],)
                if invocation_name == "unit_fixture"
                else SMOKE_CASE_IDS
            )
            if invocation_name != "registered_replay" else None
        )
        with os.fdopen(os.dup(raw_descriptor), "wb") as raw:
            with gzip.GzipFile(
                filename="", fileobj=raw, mode="wb", mtime=0,
            ) as compressed:
                for row in rows:
                    if expected_keys is not None:
                        try:
                            expected_key = next(expected_keys)
                        except StopIteration:
                            raise ValueError("extra registered raw key") from None
                        observed_key = (
                            row["method"], row["seed"],
                            row["frame_index"], row["robot_id"],
                        )
                        if observed_key != expected_key:
                            raise ValueError(
                                "registered raw key is missing, duplicate, "
                                "or out of order"
                            )
                    elif expected_smoke_ids is not None:
                        try:
                            expected_case_id = next(expected_smoke_ids)
                        except StopIteration:
                            raise ValueError("extra smoke raw key") from None
                        if row["smoke_case_id"] != expected_case_id:
                            raise ValueError(
                                "smoke raw key is missing, duplicate, "
                                "or out of order"
                            )
                    line = ordered_strict_json_bytes(row, ROW_FIELDS) + b"\n"
                    decompressed_digest.update(line)
                    compressed.write(line)
                    compressed.flush()
                    observed += 1
                    _check_live_resource_limits(
                        transaction,
                        raw_descriptor,
                        live_floor=live_floor,
                        raw_cap=raw_cap,
                    )
        if expected_keys is not None:
            try:
                next(expected_keys)
            except StopIteration:
                pass
            else:
                raise ValueError("registered raw keys are missing")
        if expected_smoke_ids is not None:
            try:
                next(expected_smoke_ids)
            except StopIteration:
                pass
            else:
                raise ValueError("smoke raw keys are missing")
        os.fsync(raw_descriptor)
        if observed != expected_rows:
            raise ValueError("observed row count differs from exact contract")
        process_identity = _process_identity(
            raw_descriptor,
            output_root=output_root,
            decompressed_sha256=decompressed_digest.hexdigest(),
        )
        _verify_process_link(transaction, process_identity)
        manifest = _manifest(
            protocol_id=protocol_id,
            invocation_name=invocation_name,
            status="running",
            output_root=output_root,
            protocol_identity=protocol_identity,
            sources=sources,
            authorization_identity=authorization_identity,
            process_identity=process_identity,
            expected_rows=expected_rows,
            observed_rows=observed,
            disk_contract=disk_contract,
            started_at=started_at,
            completed_at=None,
            error=None,
        )
        if process_identity["allocated_bytes"] > raw_cap:
            raise OSError("raw bundle exceeds allocated-byte cap")
        completed = _manifest(
            protocol_id=protocol_id,
            invocation_name=invocation_name,
            status="completed",
            output_root=output_root,
            protocol_identity=protocol_identity,
            sources=sources,
            authorization_identity=authorization_identity,
            process_identity=process_identity,
            expected_rows=expected_rows,
            observed_rows=observed,
            disk_contract=disk_contract,
            started_at=started_at,
            completed_at=_utc_now(),
            error=None,
        )
        _assert_registered_root(transaction)
        completed_manifest_identity = _publish_manifest(
            transaction,
            completed,
        )
        _assert_registered_root(transaction)
        return output_root
    except BaseException as error:
        failed_publication_allowed = True
        if completed_manifest_identity is not None:
            try:
                removed, quarantine = _retract_manifest_identity(
                    transaction,
                    completed_manifest_identity,
                )
                failed_publication_allowed = removed
                if quarantine is not None:
                    error.add_note(
                        f"completed manifest forensic quarantine: "
                        f"{quarantine}",
                    )
                if not removed:
                    error.add_note(
                        "completed manifest retraction preserved a "
                        "non-owned entry",
                    )
            except BaseException as cleanup_error:
                failed_publication_allowed = False
                error.add_note(
                    "completed manifest retraction failed: "
                    f"{type(cleanup_error).__name__}: {cleanup_error}",
                )
        if (
            transaction.get("manifest_retraction_indeterminate", False)
            or transaction.get("manifest_publication_blocked", False)
            or not failed_publication_allowed
        ):
            raise
        failed = {
            **manifest,
            "status": "failed",
            "observed_rows": observed,
            "completed_at": _utc_now(),
            "error": {
                "type": type(error).__name__,
                "message": str(error),
            },
        }
        try:
            _publish_manifest(transaction, failed)
        except BaseException:
            _emergency_manifest(transaction, failed, error)
        raise
    finally:
        _close_output_transaction(transaction)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol-path", type=Path, required=True)
    parser.add_argument("--data-path", type=Path, required=True)
    parser.add_argument("--input-manifest-path", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument(
        "--run-seeds", type=int, nargs="*", default=(),
    )
    parser.add_argument("--max-frames", type=int, required=True)
    parser.add_argument(
        "--invocation-name", choices=PRODUCER_INVOCATIONS, required=True,
    )
    parser.add_argument("--authorization-json", type=Path)
    arguments = parser.parse_args(argv)
    replay_two_range_reacquisition(
        protocol_path=arguments.protocol_path,
        data_path=arguments.data_path,
        input_manifest_path=arguments.input_manifest_path,
        output_root=arguments.output_root,
        run_seeds=tuple(arguments.run_seeds),
        max_frames=arguments.max_frames,
        invocation_name=arguments.invocation_name,
        authorization_json=arguments.authorization_json,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
