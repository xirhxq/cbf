"""Freeze the exact two-range reacquisition experiment protocol."""

from __future__ import annotations

import argparse
import copy
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
from pathlib import Path

if __package__ in {None, ""}:
    _SCRIPT_REPOSITORY_ROOT = Path(__file__).resolve().parents[2]

    def _is_script_repository_path(entry: object) -> bool:
        if not isinstance(entry, str):
            return False
        candidate = Path.cwd() if entry == "" else Path(entry)
        try:
            return candidate.resolve() == _SCRIPT_REPOSITORY_ROOT
        except (OSError, RuntimeError):
            return False

    sys.path[:] = [
        entry
        for entry in sys.path
        if not _is_script_repository_path(entry)
    ]
    sys.path.insert(0, str(_SCRIPT_REPOSITORY_ROOT))
    for _module_name in tuple(sys.modules):
        if _module_name == "scripts" or _module_name.startswith("scripts."):
            del sys.modules[_module_name]
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

from scripts.diagnostics import analyze_two_range_reacquisition as analyzer
from scripts.diagnostics import predictive_wnls as estimator
from scripts.diagnostics import replay_two_range_reacquisition as replay
from scripts.diagnostics import two_range_reacquisition as method


PROTOCOL_SCHEMA_ID = "cbf2026-two-range-reacquisition-protocol-v2"
REGISTRATION_SCHEMA_ID = "cbf2026-two-range-reacquisition-registration-v2"
PROTOCOL_ID = "cbf2026-two-range-reacquisition-v2"
RAW_SCHEMA_ID = replay.RAW_SCHEMA_ID
ANALYSIS_SCHEMA_ID = analyzer.ANALYSIS_SCHEMA_ID
METHOD_ID = method.METHOD_ID

assert replay.REGISTERED_PROTOCOL_SCHEMA_ID == PROTOCOL_SCHEMA_ID
assert replay.REGISTERED_AUTHORIZATION_SCHEMA_ID == REGISTRATION_SCHEMA_ID
assert replay.REGISTERED_PROTOCOL_ID == PROTOCOL_ID

PROTOCOL_FIELDS = (
    "schema_id",
    "registration_schema_id",
    "protocol_id",
    "implementation_parent_commit",
    "binding_design",
    "sources",
    "comparators",
    "experiment",
    "method_contract",
    "estimator_constants",
    "status_contract",
    "raw_schema",
    "analysis_schema",
    "gates",
    "disk_contract",
    "invocations",
    "evidence_lifecycle",
    "authorization",
    "commands",
)

SOURCE_MEMBER_NAMES = (
    "implementation_plan",
    "two_range_reacquisition_source",
    "predictive_wnls_source",
    "fixture_extractor_source",
    "replay_source",
    "analyzer_source",
    "registrar_source",
    "mechanism_fixture",
    "mechanism_fixture_manifest",
    "truth_data",
    "input_manifest",
)

INVOCATION_MEMBER_NAMES = (
    "smoke_a",
    "smoke_b",
    "smoke_analyzer_a",
    "smoke_analyzer_b",
    "registered_replay",
    "registered_analyzer",
)

PROTOCOL_SECTION_FIELDS = {
    "binding_design": (
        "path",
        "commit",
        "sha256",
        "review_path",
        "review_sha256",
    ),
    "source_record": (
        "path",
        "device",
        "inode",
        "size",
        "mtime_ns",
        "sha256",
    ),
    "sources": SOURCE_MEMBER_NAMES,
    "comparators": (
        "v4_replay_root",
        "v4_replay_manifest",
        "v4_compressed_process",
        "v4_decompressed_process",
        "v4_analysis_root",
        "v4_analysis_manifest",
        "v4_analysis_json",
        "v4_analysis_markdown",
        "legacy_baseline_process",
        "legacy_baseline_protocol_json_sha256",
    ),
    "experiment": (
        "method",
        "seeds",
        "frames",
        "robots",
        "expected_rows",
        "key_order",
        "ranging_sigma_m",
        "frame_dt_seconds",
        "measurement_seed_contract",
        "evidence_class",
    ),
    "method_contract": (
        "structural_conditions",
        "branch_ids",
        "continuous_starts",
        "private_prior_allowed_roles",
        "private_prior_forbidden_roles",
        "branch_score_rule",
        "publication_rule",
        "failure_rule",
        "synthetic_declaration",
        "synthetic_declaration_sha256",
    ),
    "estimator_constants": (
        "maximum_public_prediction_age",
        "innovation_reference_quantile",
        "candidate_dedup_m",
        "motion_covariance_per_frame",
        "reacquisition_reduced_cost_max",
        "maximum_error_m",
    ),
    "status_contract": (
        "attempt_statuses",
        "output_statuses",
        "private_statuses",
        "prior_used_semantics",
    ),
    "raw_schema": (
        "schema_id",
        "row_fields",
        "row_scalar_contracts",
        "row_array_contracts",
        "nested_field_orders",
        "branch_field_contracts",
        "manifest_fields",
        "null_rules",
    ),
    "analysis_schema": (
        "schema_id",
        "analysis_fields",
        "nested_field_orders",
        "value_contracts",
        "list_order_and_cardinality",
        "semantic_payload_fields",
        "manifest_fields",
        "manifest_nested_field_orders",
        "manifest_source_member_names",
        "manifest_null_rules",
    ),
    "gates": (
        "scientific_gate_order",
        "scientific_gate_records",
        "integrity_gate_order",
        "integrity_gate_records",
        "aggregate_decision_rule",
    ),
    "disk_contract": (
        "launch_minimum_free_bytes",
        "live_minimum_free_bytes",
        "raw_bundle_max_allocated_bytes",
        "compact_bundle_max_allocated_bytes",
    ),
    "invocation_record": (
        "invocation_name",
        "input_root",
        "output_root",
        "expected_rows",
        "authorization_required",
        "retry_allowed",
    ),
    "invocations": INVOCATION_MEMBER_NAMES,
    "evidence_lifecycle": (
        "preexisting_target_allowed",
        "no_follow",
        "descriptor_pinning",
        "transactional_publication",
        "fsync_required",
        "terminal_manifest_required",
        "failure_retained",
        "paper_gate",
    ),
    "authorization": (
        "implementation_plan_approved",
        "protocol_preflight_required",
        "deterministic_smoke_review_required",
        "registered_full_grid_authorization",
        "authorization_record_schema",
        "authorization_record_path",
    ),
}

COMMAND_FIELDS = INVOCATION_MEMBER_NAMES

BINDING_DESIGN = {
    "path": (
        "docs/superpowers/specs/"
        "2026-07-30-cbf2026-two-range-reacquisition-design.md"
    ),
    "commit": "20a61aad96af35ee7e16434fab0a5edaaea38ef0",
    "sha256": (
        "d28d6bfe297a6e37b0a878dc716bd5b91f26c5b11568c3f6fc42bd07760a266b"
    ),
    "review_path": (
        "docs/superpowers/specs/reviews/"
        "2026-07-30-cbf2026-two-range-reacquisition-design-review.md"
    ),
    "review_sha256": (
        "6d857be965de5b83bb05ce06381b79108fdc4abbccf6b96fe4ec456d7a7df570"
    ),
}

EXPECTED_COMPARATOR_BINDINGS = {
    "v4_replay_root": (
        "/private/tmp/cbf2026-predictive-wnls-development/stage1-v4"
    ),
    "v4_manifest_sha256": (
        "123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223"
    ),
    "v4_compressed_sha256": (
        "9710d1a5792e682bb272d0bb3305162180d1e0b9a0636647d6e071d1782a699b"
    ),
    "v4_decompressed_sha256": (
        "7047989804fd03bd4cadffac8212349b7183857623ac201d14ef2fc3e30ee2e1"
    ),
    "v4_analysis_root": (
        "/private/tmp/cbf2026-predictive-wnls-development-analysis/stage1-v4"
    ),
    "v4_analysis_manifest_sha256": (
        "e50352825665bb94c0e3b32c7922a081f093100daf0ed8e5c8ce9e9f32bd9238"
    ),
    "v4_analysis_json_sha256": (
        "8a0a417f2d3b139fa248bef8157a3d4871fa04042aaad82ce5cd31f470a6ff1e"
    ),
    "v4_analysis_markdown_sha256": (
        "986dcf72f72ab6dc3d424d0de6cdbf410cf271d174d16caef32945acc7d4e13b"
    ),
    "legacy_baseline_process_sha256": (
        "c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003"
    ),
    "legacy_baseline_protocol_json_sha256": (
        "09b236978e2e21bac226dc3d00c36a2ee5cdf5fea0e616f2c1d875b029d4e4e0"
    ),
}
_BOUND_COMPARATOR_ITEMS = tuple(EXPECTED_COMPARATOR_BINDINGS.items())

ROOTS = {
    "smoke_a": "/private/tmp/cbf2026-two-range-reacquisition-smoke-v2-a",
    "smoke_b": "/private/tmp/cbf2026-two-range-reacquisition-smoke-v2-b",
    "smoke_analyzer_a": (
        "/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v2-a"
    ),
    "smoke_analyzer_b": (
        "/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v2-b"
    ),
    "registered_replay": (
        "/private/tmp/cbf2026-two-range-reacquisition-development/v2"
    ),
    "registered_analyzer": (
        "/private/tmp/cbf2026-two-range-reacquisition-analysis/v2"
    ),
}
_BOUND_ROOT_ITEMS = tuple(ROOTS.items())

RETIRED_ROOTS = (
    "/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-a",
    "/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-b",
    "/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-a",
    "/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-b",
    "/private/tmp/cbf2026-two-range-reacquisition-development/v1",
    "/private/tmp/cbf2026-two-range-reacquisition-analysis/v1",
)
_BOUND_RETIRED_ROOTS = RETIRED_ROOTS

GENERATED_PROTOCOL_PATHS = (
    (
        "docs/diagnostics/"
        "2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.md"
    ),
    (
        "docs/diagnostics/"
        "2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json"
    ),
)

PRESERVED_TRAJECTORY_ROOT = (
    "/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/"
    "20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725"
)


def _assert_bound_interfaces() -> None:
    if replay.SMOKE_CASE_IDS != (
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
    ):
        raise ValueError("producer smoke case declaration changed")


_assert_bound_interfaces()


def production_invocation_contract() -> dict:
    """Return the exact smoke and registered invocation records."""
    _assert_bound_interfaces()
    fixture_root = "tests/fixtures/cbf2026_two_range_reacquisition"
    return {
        "smoke_a": {
            "invocation_name": "smoke_a",
            "input_root": fixture_root,
            "output_root": ROOTS["smoke_a"],
            "expected_rows": 18,
            "authorization_required": False,
            "retry_allowed": False,
        },
        "smoke_b": {
            "invocation_name": "smoke_b",
            "input_root": fixture_root,
            "output_root": ROOTS["smoke_b"],
            "expected_rows": 18,
            "authorization_required": False,
            "retry_allowed": False,
        },
        "smoke_analyzer_a": {
            "invocation_name": "smoke_analyzer_a",
            "input_root": ROOTS["smoke_a"],
            "output_root": ROOTS["smoke_analyzer_a"],
            "expected_rows": 18,
            "authorization_required": False,
            "retry_allowed": False,
        },
        "smoke_analyzer_b": {
            "invocation_name": "smoke_analyzer_b",
            "input_root": ROOTS["smoke_b"],
            "output_root": ROOTS["smoke_analyzer_b"],
            "expected_rows": 18,
            "authorization_required": False,
            "retry_allowed": False,
        },
        "registered_replay": {
            "invocation_name": "registered_replay",
            "input_root": PRESERVED_TRAJECTORY_ROOT,
            "output_root": ROOTS["registered_replay"],
            "expected_rows": 140000,
            "authorization_required": True,
            "retry_allowed": False,
        },
        "registered_analyzer": {
            "invocation_name": "registered_analyzer",
            "input_root": ROOTS["registered_replay"],
            "output_root": ROOTS["registered_analyzer"],
            "expected_rows": 140000,
            "authorization_required": True,
            "retry_allowed": False,
        },
    }


PROTOCOL_RELATIVE_PATH = (
    "docs/diagnostics/"
    "2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json"
)
AUTHORIZATION_RELATIVE_PATH = (
    "docs/diagnostics/reviews/"
    "2026-07-31-cbf2026-two-range-reacquisition-"
    "registered-v2-authorization.json"
)


def production_command_contract(sources: Mapping) -> dict[str, list[str]]:
    """Return exact argv arrays without a shell or implicit CLI defaults."""
    _assert_bound_interfaces()
    if not isinstance(sources, Mapping):
        raise ValueError("sources must be an object")
    source_paths = {}
    for name in ("truth_data", "input_manifest"):
        declaration = sources.get(name)
        if (
            not isinstance(declaration, Mapping)
            or not isinstance(declaration.get("path"), str)
        ):
            raise ValueError(f"{name} source path is absent")
        path = Path(declaration["path"])
        if not path.is_absolute() or ".." in path.parts:
            raise ValueError(f"{name} source path must be exact and absolute")
        source_paths[name] = str(path)
    replay_prefix = [
        "conda",
        "run",
        "-n",
        "cbf_env",
        "python",
        "scripts/diagnostics/replay_two_range_reacquisition.py",
        "--protocol-path",
        PROTOCOL_RELATIVE_PATH,
        "--data-path",
        source_paths["truth_data"],
        "--input-manifest-path",
        source_paths["input_manifest"],
    ]
    analyzer_prefix = [
        "conda",
        "run",
        "-n",
        "cbf_env",
        "python",
        "scripts/diagnostics/analyze_two_range_reacquisition.py",
        "--protocol-path",
        PROTOCOL_RELATIVE_PATH,
    ]
    commands = {}
    for name in ("smoke_a", "smoke_b"):
        commands[name] = replay_prefix + [
            "--output-root",
            ROOTS[name],
            "--run-seeds",
            "--max-frames",
            "0",
            "--invocation-name",
            name,
        ]
    for name, raw_name in (
        ("smoke_analyzer_a", "smoke_a"),
        ("smoke_analyzer_b", "smoke_b"),
    ):
        commands[name] = analyzer_prefix + [
            "--raw-root",
            ROOTS[raw_name],
            "--output-root",
            ROOTS[name],
            "--invocation-name",
            name,
        ]
    commands["registered_replay"] = replay_prefix + [
        "--output-root",
        ROOTS["registered_replay"],
        "--run-seeds",
        *(str(seed) for seed in range(20260727, 20260747)),
        "--max-frames",
        "500",
        "--invocation-name",
        "registered_replay",
        "--authorization-json",
        AUTHORIZATION_RELATIVE_PATH,
    ]
    commands["registered_analyzer"] = analyzer_prefix + [
        "--raw-root",
        ROOTS["registered_replay"],
        "--output-root",
        ROOTS["registered_analyzer"],
        "--invocation-name",
        "registered_analyzer",
        "--authorization-json",
        AUTHORIZATION_RELATIVE_PATH,
    ]
    return {
        name: commands[name]
        for name in COMMAND_FIELDS
    }


def _literal_contract(value: str) -> str:
    replacements = {
        name: ",".join(str(item) for item in declaration)
        for name, declaration in (
            ("ROW_INVOCATION_NAMES", replay.ROW_INVOCATION_NAMES),
            ("SMOKE_CASE_IDS", replay.SMOKE_CASE_IDS),
            ("CONSIDERATION_REASONS", replay.CONSIDERATION_REASONS),
            ("ATTEMPT_STATUSES", replay.ATTEMPT_STATUSES),
            ("OUTPUT_STATUSES", replay.OUTPUT_STATUSES),
            ("BRANCH_IDS", method.BRANCH_IDS),
            ("BRANCH_FIELDS", replay.BRANCH_FIELDS),
            ("SOLVER_RESULT_FIELDS", replay.SOLVER_RESULT_FIELDS),
            ("PROPOSAL_TRACE_FIELDS", replay.PROPOSAL_TRACE_FIELDS),
            ("EXISTING_CANDIDATE_FIELDS", replay.EXISTING_CANDIDATE_FIELDS),
            ("GATE_DIAGNOSTIC_FIELDS", replay.GATE_DIAGNOSTIC_FIELDS),
            ("REFERENCE_EVIDENCE_FIELDS", replay.REFERENCE_EVIDENCE_FIELDS),
            (
                "REFERENCE_FRESHNESS_FIELDS",
                replay.REFERENCE_FRESHNESS_FIELDS,
            ),
            ("MANDATORY_REFERENCE_FIELDS", replay.MANDATORY_REFERENCE_FIELDS),
            ("REFERENCE_KEY_FIELDS", replay.REFERENCE_KEY_FIELDS),
            ("EXCLUSION_FIELDS", replay.EXCLUSION_FIELDS),
            ("VIOLATION_FIELDS", replay.VIOLATION_FIELDS),
            ("SELECTOR_SMOKE_INPUT_FIELDS", replay.SELECTOR_SMOKE_INPUT_FIELDS),
            (
                "SELECTOR_SMOKE_EXPECTED_FIELDS",
                replay.SELECTOR_SMOKE_EXPECTED_FIELDS,
            ),
            (
                "BRANCH_GATE_SMOKE_INPUT_FIELDS",
                replay.BRANCH_GATE_SMOKE_INPUT_FIELDS,
            ),
            (
                "BRANCH_GATE_SMOKE_EXPECTED_FIELDS",
                replay.BRANCH_GATE_SMOKE_EXPECTED_FIELDS,
            ),
            (
                "BRANCH_PAIR_GATE_SMOKE_INPUT_FIELDS",
                replay.BRANCH_PAIR_GATE_SMOKE_INPUT_FIELDS,
            ),
            (
                "BRANCH_PAIR_GATE_SMOKE_EXPECTED_FIELDS",
                replay.BRANCH_PAIR_GATE_SMOKE_EXPECTED_FIELDS,
            ),
            (
                "CANDIDATE_GATE_SMOKE_INPUT_FIELDS",
                replay.CANDIDATE_GATE_SMOKE_INPUT_FIELDS,
            ),
            (
                "CANDIDATE_GATE_SMOKE_EXPECTED_FIELDS",
                replay.CANDIDATE_GATE_SMOKE_EXPECTED_FIELDS,
            ),
            (
                "MECHANISM_SMOKE_INPUT_FIELDS",
                replay.MECHANISM_SMOKE_INPUT_FIELDS,
            ),
            (
                "MECHANISM_SMOKE_EXPECTED_FIELDS",
                replay.MECHANISM_SMOKE_EXPECTED_FIELDS,
            ),
        )
    }
    for name in sorted(replacements, key=len, reverse=True):
        value = value.replace(name, replacements[name])
    return value


def _expanded_contract_map(contract: Mapping) -> dict:
    return {
        str(name): _literal_contract(str(value))
        for name, value in contract.items()
    }


def _raw_schema_contract() -> dict:
    nested = {
        "branch": list(replay.BRANCH_FIELDS),
        "solver_result": list(replay.SOLVER_RESULT_FIELDS),
        "proposal_trace": list(replay.PROPOSAL_TRACE_FIELDS),
        "existing_candidate": list(replay.EXISTING_CANDIDATE_FIELDS),
        "gate_diagnostics": list(replay.GATE_DIAGNOSTIC_FIELDS),
        "reference_evidence": list(replay.REFERENCE_EVIDENCE_FIELDS),
        "reference_freshness": list(replay.REFERENCE_FRESHNESS_FIELDS),
        "mandatory_references": list(replay.MANDATORY_REFERENCE_FIELDS),
        "reference_key": list(replay.REFERENCE_KEY_FIELDS),
        "exclusion": list(replay.EXCLUSION_FIELDS),
        "violation": list(replay.VIOLATION_FIELDS),
        "private_prior": list(replay.PRIVATE_PRIOR_FIELDS),
        "next_private_state": list(replay.NEXT_PRIVATE_STATE_FIELDS),
        "synthetic_case": list(replay.SYNTHETIC_CASE_FIELDS),
        "selector_smoke_input": list(replay.SELECTOR_SMOKE_INPUT_FIELDS),
        "selector_smoke_expected": list(
            replay.SELECTOR_SMOKE_EXPECTED_FIELDS
        ),
        "branch_gate_smoke_input": list(
            replay.BRANCH_GATE_SMOKE_INPUT_FIELDS
        ),
        "branch_gate_smoke_expected": list(
            replay.BRANCH_GATE_SMOKE_EXPECTED_FIELDS
        ),
        "branch_pair_gate_smoke_input": list(
            replay.BRANCH_PAIR_GATE_SMOKE_INPUT_FIELDS
        ),
        "branch_pair_gate_smoke_expected": list(
            replay.BRANCH_PAIR_GATE_SMOKE_EXPECTED_FIELDS
        ),
        "candidate_gate_smoke_input": list(
            replay.CANDIDATE_GATE_SMOKE_INPUT_FIELDS
        ),
        "candidate_gate_smoke_expected": list(
            replay.CANDIDATE_GATE_SMOKE_EXPECTED_FIELDS
        ),
        "mechanism_smoke_input": list(replay.MECHANISM_SMOKE_INPUT_FIELDS),
        "mechanism_smoke_expected": list(
            replay.MECHANISM_SMOKE_EXPECTED_FIELDS
        ),
        "source_record": list(replay.FILE_IDENTITY_FIELDS),
        "process_identity": list(replay.PROCESS_IDENTITY_FIELDS),
        "key_contract": list(replay.RAW_KEY_CONTRACT_FIELDS),
        "disk_contract": list(PROTOCOL_SECTION_FIELDS["disk_contract"]),
        "error": ["type", "message"],
    }
    null_rules = {
        "smoke_case_fields": (
            "non_null_exactly_for_smoke_rows_and_null_for_registered_rows"
        ),
        "registered_key_fields": (
            "non_null_exactly_for_registered_rows_and_fixture_mechanism_row"
        ),
        "fresh_publication_fields": (
            "fresh_covariance_and_epsilon_non_null_exactly_when_output_fresh"
        ),
        "aged_publication_fields": (
            "aged_covariance_and_radius_non_null_exactly_when_output_predicted"
        ),
        "private_state_fields": (
            "state_payload_non_null_exactly_when_corresponding_status_available"
        ),
        "branch_score_fields": (
            "q_and_pass_non_null_exactly_after_runtime_branch_scoring"
        ),
        "manifest_authorization_identity": (
            "non_null_exactly_for_registered_replay"
        ),
        "manifest_terminal_fields": (
            "completed_at_non_null_exactly_when_terminal_and_error_non_null_"
            "exactly_when_failed"
        ),
    }
    return {
        "schema_id": RAW_SCHEMA_ID,
        "row_fields": list(replay.ROW_FIELDS),
        "row_scalar_contracts": _expanded_contract_map(
            replay.ROW_SCALAR_CONTRACTS
        ),
        "row_array_contracts": _expanded_contract_map(
            replay.ROW_ARRAY_CONTRACTS
        ),
        "nested_field_orders": nested,
        "branch_field_contracts": _expanded_contract_map(
            replay.BRANCH_FIELD_CONTRACTS
        ),
        "manifest_fields": list(replay.RAW_MANIFEST_FIELDS),
        "null_rules": null_rules,
    }


def _analysis_schema_contract() -> dict:
    nested = {
        "identities": list(analyzer.IDENTITY_FIELDS),
        "identity_record": list(analyzer.ANALYSIS_IDENTITY_RECORD_FIELDS),
        "budgets": list(analyzer.BUDGET_FIELDS),
        "status_counts": list(analyzer.STATUS_COUNT_FIELDS),
        "selector_accounting": list(analyzer.SELECTOR_ACCOUNTING_FIELDS),
        "baseline_fresh_transitions": list(
            analyzer.BASELINE_FRESH_TRANSITION_FIELDS
        ),
        "v4_descriptive_comparison": list(
            analyzer.V4_DESCRIPTIVE_COMPARISON_FIELDS
        ),
        "v4_fresh_transitions": list(analyzer.V4_FRESH_TRANSITION_FIELDS),
        "v4_paired_comparison": list(
            analyzer.V4_PAIRED_COMPARISON_FIELDS
        ),
        "paired_comparison": list(analyzer.PAIRED_COMPARISON_FIELDS),
        "gate_record": list(analyzer.GATE_RECORD_FIELDS),
        "tail_record": list(analyzer.TAIL_RECORD_FIELDS),
        "source_projection": list(analyzer.SOURCE_PROJECTION_FIELDS),
    }
    value_contracts = {
        "schema_id": f"literal:{ANALYSIS_SCHEMA_ID}",
        "protocol_id": f"literal:{PROTOCOL_ID}",
        "invocation_name": (
            "enum:smoke_analyzer_a,smoke_analyzer_b,registered_analyzer"
        ),
        "decision": (
            "registered_enum:pass,fail;"
            "smoke_enum:smoke_pass,smoke_fail"
        ),
        "semantic_payload_sha256": "lowercase_sha256",
        "identities": "strict_object:literal_nested_field_order",
        "budgets": "strict_object:non_boolean_nonnegative_integers",
        "status_counts": "strict_object:non_boolean_nonnegative_integers",
        "selector_accounting": (
            "strict_object:exact_counts_and_ordered_outage_lengths"
        ),
        "baseline_fresh_transitions": (
            "strict_object:non_boolean_nonnegative_integers"
        ),
        "v4_descriptive_comparison": (
            "strict_object:descriptive_only_not_gate_substitute"
        ),
        "paired_comparison": (
            "strict_object:exact_intersection_linear_p95"
        ),
        "scientific_gates": (
            "registered_ordered_list:exactly_nine_gate_records;"
            "smoke_ordered_list:empty"
        ),
        "integrity_gates": (
            "ordered_list:exactly_fourteen_gate_records"
        ),
        "tails": "canonical_ordered_list:tail_records",
        "limitations": "literal_ordered_list:six_disclosures",
    }
    list_contract = {
        "scientific_gate_ids": list(analyzer.GATES),
        "registered_scientific_gate_count": 9,
        "smoke_scientific_gate_count": 0,
        "integrity_gate_ids": list(analyzer.INTEGRITY_GATE_IDS),
        "integrity_gate_count": 14,
        "tail_metrics": list(analyzer.TAIL_METRICS),
        "tail_stratifiers": list(analyzer.TAIL_STRATIFIERS),
        "v4_tail_stratifiers": list(analyzer.V4_TAIL_STRATIFIERS),
        "tail_populations": list(analyzer.TAIL_POPULATIONS),
        "time_bins": [list(interval) for interval in analyzer.TIME_BINS],
        "limitations": list(analyzer.LIMITATIONS),
        "smoke_expected_rows": 18,
        "registered_expected_rows": 140000,
    }
    manifest_nested = {
        "identity_record": list(analyzer.ANALYSIS_MANIFEST_IDENTITY_FIELDS),
        "source_member_names": {
            name: list(members)
            for name, members in analyzer.ANALYSIS_SOURCE_MEMBER_NAMES.items()
        },
        "output_member_names": list(analyzer.ANALYSIS_OUTPUT_MEMBER_NAMES),
        "disk_contract": list(PROTOCOL_SECTION_FIELDS["disk_contract"]),
        "error": list(analyzer.ANALYSIS_ERROR_FIELDS),
    }
    manifest_nulls = {
        "authorization_identity": (
            "non_null_exactly_for_registered_analyzer"
        ),
        "output_identities": (
            "non_null_exactly_for_each_completed_output_and_null_on_failure"
        ),
        "completed_at": "non_null_exactly_when_terminal",
        "error": "non_null_exactly_when_failed",
    }
    return {
        "schema_id": ANALYSIS_SCHEMA_ID,
        "analysis_fields": list(analyzer.ANALYSIS_FIELDS),
        "nested_field_orders": nested,
        "value_contracts": value_contracts,
        "list_order_and_cardinality": list_contract,
        "semantic_payload_fields": list(analyzer.SMOKE_SEMANTIC_FIELDS),
        "manifest_fields": list(analyzer.ANALYSIS_MANIFEST_FIELDS),
        "manifest_nested_field_orders": manifest_nested,
        "manifest_source_member_names": {
            name: list(members)
            for name, members in analyzer.ANALYSIS_SOURCE_MEMBER_NAMES.items()
        },
        "manifest_null_rules": manifest_nulls,
    }


def _gate_contract() -> dict:
    scientific = {
        "maximum_published_error_m_strictly_below": {
            "operator": "strictly_below",
            "threshold": 50.0,
        },
        "maximum_fresh_error_m_strictly_below": {
            "operator": "strictly_below",
            "threshold": 50.0,
        },
        "paired_both_fresh_p95_must_not_worsen": {
            "operator": "less_than_or_equal",
            "threshold": 0.0,
        },
        "fresh_availability_max_drop_fraction": {
            "operator": "less_than_or_equal",
            "threshold": 0.02,
        },
        "fresh_or_predicted_min_fraction": {
            "operator": "greater_than_or_equal",
            "threshold": 0.95,
        },
        "maximum_prediction_age_frames": {
            "operator": "less_than_or_equal",
            "threshold": 2,
        },
        "qualification_anchor_violations_allowed": {
            "operator": "equal",
            "threshold": 0,
        },
        "current_frame_provenance_violations_allowed": {
            "operator": "equal",
            "threshold": 0,
        },
        "ascending_dag_violations_allowed": {
            "operator": "equal",
            "threshold": 0,
        },
    }
    integrity = {
        gate_id: {"operator": "equal", "threshold": 0}
        for gate_id in analyzer.INTEGRITY_GATE_IDS
    }
    return {
        "scientific_gate_order": list(scientific),
        "scientific_gate_records": scientific,
        "integrity_gate_order": list(integrity),
        "integrity_gate_records": integrity,
        "aggregate_decision_rule": (
            "PASS_if_and_only_if_every_scientific_and_integrity_gate_passes"
        ),
    }


def _build_protocol(
    *,
    head: str,
    sources: Mapping,
    comparators: Mapping,
) -> dict:
    """Assemble the canonical protocol before strict validation."""
    _assert_bound_interfaces()
    experiment = {
        "method": METHOD_ID,
        "seeds": list(range(20260727, 20260747)),
        "frames": list(range(500)),
        "robots": list(range(1, 15)),
        "expected_rows": 140000,
        "key_order": ["method", "seed", "frame_index", "robot_id"],
        "ranging_sigma_m": 0.5,
        "frame_dt_seconds": estimator.FRAME_DT_SECONDS,
        "measurement_seed_contract": "cbf2026-range-v1",
        "evidence_class": "paired_single_trajectory_development_only",
    }
    method_contract = {
        "structural_conditions": [
            "live_public_prediction_absent",
            "qualification_status_ok",
            "exactly_two_active_references",
            "active_references_are_current_fresh_mandatory_uavs",
            "active_references_have_strictly_lower_robot_ids",
            "active_base_references_absent",
            "active_optional_references_absent",
            "fixed_references_present",
            "ranges_and_provenance_valid",
        ],
        "branch_ids": list(method.BRANCH_IDS),
        "continuous_starts": list(method.BRANCH_IDS),
        "private_prior_allowed_roles": [
            "discrete_branch_identity_selection"
        ],
        "private_prior_forbidden_roles": [
            "active_reference",
            "fim_term",
            "continuous_wnls_start",
            "continuous_update_input",
            "covariance_source",
            "publication_representative",
        ],
        "branch_score_rule": (
            "runtime_normalized_innovation_q_branch_less_than_or_equal_to_"
            "innovation_reference_quantile"
        ),
        "publication_rule": (
            "publish_only_exactly_one_passing_branch_bound_to_its_current_"
            "range_result"
        ),
        "failure_rule": (
            "zero_or_multiple_passing_branches_publish_unavailable_without_"
            "prediction"
        ),
        "synthetic_declaration": copy.deepcopy(
            replay.SYNTHETIC_DECLARATION
        ),
        "synthetic_declaration_sha256": (
            replay.SYNTHETIC_DECLARATION_SHA256
        ),
    }
    estimator_constants = {
        "maximum_public_prediction_age": estimator.MAX_PUBLIC_PREDICTION_AGE,
        "innovation_reference_quantile": (
            estimator.INNOVATION_REFERENCE_QUANTILE
        ),
        "candidate_dedup_m": estimator.CANDIDATE_DEDUP_M,
        "motion_covariance_per_frame": 0.25,
        "reacquisition_reduced_cost_max": 9.0,
        "maximum_error_m": 50.0,
    }
    status_contract = {
        "attempt_statuses": list(estimator.ATTEMPT_STATUSES),
        "output_statuses": list(estimator.OUTPUT_STATUSES),
        "private_statuses": ["available", "absent"],
        "prior_used_semantics": {
            "branch_selection": (
                "true_exactly_when_runtime_branch_scores_are_evaluated"
            ),
            "fim": False,
            "continuous_update": False,
            "public_covariance": False,
            "publication_representative": False,
        },
    }
    disk_contract = {
        "launch_minimum_free_bytes": 8_000_000_000,
        "live_minimum_free_bytes": 6_000_000_000,
        "raw_bundle_max_allocated_bytes": 2_000_000_000,
        "compact_bundle_max_allocated_bytes": 10_000_000,
    }
    lifecycle = {
        "preexisting_target_allowed": False,
        "no_follow": True,
        "descriptor_pinning": True,
        "transactional_publication": True,
        "fsync_required": True,
        "terminal_manifest_required": True,
        "failure_retained": True,
        "paper_gate": "CLOSED",
    }
    authorization = {
        "implementation_plan_approved": True,
        "protocol_preflight_required": True,
        "deterministic_smoke_review_required": True,
        "registered_full_grid_authorization": "pending_external_record",
        "authorization_record_schema": REGISTRATION_SCHEMA_ID,
        "authorization_record_path": AUTHORIZATION_RELATIVE_PATH,
    }
    source_copy = copy.deepcopy(dict(sources))
    return {
        "schema_id": PROTOCOL_SCHEMA_ID,
        "registration_schema_id": REGISTRATION_SCHEMA_ID,
        "protocol_id": PROTOCOL_ID,
        "implementation_parent_commit": head,
        "binding_design": copy.deepcopy(BINDING_DESIGN),
        "sources": source_copy,
        "comparators": copy.deepcopy(dict(comparators)),
        "experiment": experiment,
        "method_contract": method_contract,
        "estimator_constants": estimator_constants,
        "status_contract": status_contract,
        "raw_schema": _raw_schema_contract(),
        "analysis_schema": _analysis_schema_contract(),
        "gates": _gate_contract(),
        "disk_contract": disk_contract,
        "invocations": production_invocation_contract(),
        "evidence_lifecycle": lifecycle,
        "authorization": authorization,
        "commands": production_command_contract(source_copy),
    }


def _require_object_order(
    value: object,
    fields: tuple[str, ...],
    *,
    label: str,
) -> Mapping:
    if not isinstance(value, Mapping) or tuple(value) != fields:
        raise ValueError(f"{label} differs from strict field order")
    return value


def _canonical_hex(value: object, length: int) -> bool:
    return (
        isinstance(value, str)
        and len(value) == length
        and all(character in "0123456789abcdef" for character in value)
    )


def _strict_integer(
    value: object,
    *,
    minimum: int = 0,
    maximum: int | None = None,
) -> bool:
    return (
        type(value) is int
        and value >= minimum
        and (maximum is None or value <= maximum)
    )


def _finite_number(value: object, *, positive: bool = False) -> bool:
    return (
        type(value) in {int, float}
        and math.isfinite(float(value))
        and (not positive or float(value) > 0.0)
    )


def _canonical_path(
    value: object,
    *,
    absolute: bool,
) -> bool:
    if not isinstance(value, str) or not value:
        return False
    path = Path(value)
    return (
        path.is_absolute() is absolute
        and ".." not in path.parts
        and str(path) == value
    )


def _validate_source_record(record: Mapping, *, label: str) -> None:
    if not _canonical_path(record["path"], absolute=True):
        raise ValueError(f"{label} path must be exact and absolute")
    for field in ("device", "inode", "size", "mtime_ns"):
        if not _strict_integer(record[field]):
            raise ValueError(
                f"{label} {field} must be a non-Boolean integer"
            )
    if not _canonical_hex(record["sha256"], 64):
        raise ValueError(f"{label} SHA-256 must be canonical lowercase hex")


def _validate_value_types(protocol: Mapping) -> None:
    if not _canonical_hex(protocol["implementation_parent_commit"], 40):
        raise ValueError("implementation parent OID must be canonical")
    for field in ("schema_id", "registration_schema_id", "protocol_id"):
        if not isinstance(protocol[field], str) or not protocol[field]:
            raise ValueError(f"{field} must be a non-empty string")
    design = protocol["binding_design"]
    for field in ("path", "review_path"):
        if not _canonical_path(design[field], absolute=False):
            raise ValueError(f"binding design {field} must be exact relative")
    if not _canonical_hex(design["commit"], 40):
        raise ValueError("binding design OID must be canonical")
    for field in ("sha256", "review_sha256"):
        if not _canonical_hex(design[field], 64):
            raise ValueError(f"binding design {field} SHA-256 is invalid")
    for name, record in protocol["sources"].items():
        _validate_source_record(record, label=f"{name} source")
    comparators = protocol["comparators"]
    for field in ("v4_replay_root", "v4_analysis_root"):
        if not _canonical_path(comparators[field], absolute=True):
            raise ValueError(f"{field} must be an exact absolute root")
    for name in (
        "v4_replay_manifest",
        "v4_compressed_process",
        "v4_decompressed_process",
        "v4_analysis_manifest",
        "v4_analysis_json",
        "v4_analysis_markdown",
        "legacy_baseline_process",
    ):
        _validate_source_record(
            comparators[name],
            label=f"{name} comparator",
        )
    if not _canonical_hex(
        comparators["legacy_baseline_protocol_json_sha256"],
        64,
    ):
        raise ValueError("legacy protocol SHA-256 must be canonical")
    experiment = protocol["experiment"]
    if not isinstance(experiment["method"], str):
        raise ValueError("experiment method must be a string")
    for field in ("seeds", "frames", "robots"):
        if (
            not isinstance(experiment[field], list)
            or any(not _strict_integer(value) for value in experiment[field])
        ):
            raise ValueError(
                f"experiment {field} must contain non-Boolean integers"
            )
    if not _strict_integer(experiment["expected_rows"], minimum=1):
        raise ValueError(
            "expected rows must be a positive non-Boolean integer"
        )
    if (
        not isinstance(experiment["key_order"], list)
        or not experiment["key_order"]
        or any(type(value) is not str or not value for value in experiment["key_order"])
    ):
        raise ValueError("key order must be a non-empty string list")
    for field in ("ranging_sigma_m", "frame_dt_seconds"):
        if not _finite_number(experiment[field], positive=True):
            raise ValueError(f"experiment {field} must be finite and positive")
    for field in ("measurement_seed_contract", "evidence_class"):
        if not isinstance(experiment[field], str) or not experiment[field]:
            raise ValueError(f"experiment {field} must be a string")
    method_contract = protocol["method_contract"]
    for field in (
        "structural_conditions",
        "branch_ids",
        "continuous_starts",
        "private_prior_allowed_roles",
        "private_prior_forbidden_roles",
    ):
        if (
            not isinstance(method_contract[field], list)
            or not method_contract[field]
            or any(
                type(item) is not str or not item
                for item in method_contract[field]
            )
        ):
            raise ValueError(f"method {field} must be a non-empty string list")
    for field in ("branch_score_rule", "publication_rule", "failure_rule"):
        if not isinstance(method_contract[field], str) or not method_contract[field]:
            raise ValueError(f"method {field} must be a string")
    if not isinstance(method_contract["synthetic_declaration"], Mapping):
        raise ValueError("synthetic declaration must be an object")
    if not _canonical_hex(
        method_contract["synthetic_declaration_sha256"],
        64,
    ):
        raise ValueError("synthetic declaration SHA-256 must be canonical")
    constants = protocol["estimator_constants"]
    for field, value in constants.items():
        if not _finite_number(value):
            raise ValueError(f"estimator constant {field} must be finite")
    if not _strict_integer(constants["maximum_public_prediction_age"]):
        raise ValueError(
            "maximum prediction age must be a non-Boolean integer"
        )
    status = protocol["status_contract"]
    for field in ("attempt_statuses", "output_statuses", "private_statuses"):
        if (
            not isinstance(status[field], list)
            or not status[field]
            or any(type(item) is not str or not item for item in status[field])
        ):
            raise ValueError(f"{field} must be a non-empty string list")
    if not isinstance(status["prior_used_semantics"], Mapping):
        raise ValueError("prior-used semantics must be an object")
    for section in ("raw_schema", "analysis_schema"):
        for field, value in protocol[section].items():
            if value is None:
                raise ValueError(f"{section} {field} cannot be null")
    gates = protocol["gates"]
    if (
        not isinstance(gates["scientific_gate_order"], list)
        or not isinstance(gates["scientific_gate_records"], Mapping)
        or not isinstance(gates["integrity_gate_order"], list)
        or not isinstance(gates["integrity_gate_records"], Mapping)
        or not isinstance(gates["aggregate_decision_rule"], str)
    ):
        raise ValueError("gate declarations have invalid types")
    for field, value in protocol["disk_contract"].items():
        if not _strict_integer(value):
            raise ValueError(f"{field} must be a non-Boolean integer")
    for name, record in protocol["invocations"].items():
        if record["invocation_name"] != name:
            raise ValueError(f"{name} invocation name differs")
        if not _canonical_path(
            record["input_root"],
            absolute=record["input_root"].startswith("/"),
        ):
            raise ValueError(f"{name} input root is not canonical")
        if not _canonical_path(record["output_root"], absolute=True):
            raise ValueError(f"{name} output root must be absolute")
        if not _strict_integer(record["expected_rows"], minimum=1):
            raise ValueError(
                f"{name} expected rows must be a non-Boolean integer"
            )
        for field in ("authorization_required", "retry_allowed"):
            if type(record[field]) is not bool:
                raise ValueError(f"{name} {field} must be Boolean")
    for field, value in protocol["evidence_lifecycle"].items():
        if field == "paper_gate":
            if not isinstance(value, str) or not value:
                raise ValueError("paper gate must be a string")
        elif type(value) is not bool:
            raise ValueError(f"{field} must be Boolean")
    authorization = protocol["authorization"]
    for field in (
        "implementation_plan_approved",
        "protocol_preflight_required",
        "deterministic_smoke_review_required",
    ):
        if type(authorization[field]) is not bool:
            raise ValueError(f"{field} must be Boolean")
    for field in (
        "registered_full_grid_authorization",
        "authorization_record_schema",
        "authorization_record_path",
    ):
        if not isinstance(authorization[field], str) or not authorization[field]:
            raise ValueError(f"{field} must be a string")
    shell_symbols = ("*", "?", "[", "]", "{", "}", ";", "|", "&", ">", "<", "$", "`")
    for name, command in protocol["commands"].items():
        if (
            not isinstance(command, list)
            or not command
            or any(type(token) is not str or not token for token in command)
        ):
            raise ValueError(f"{name} command must be a non-empty string argv")
        if any(
            any(symbol in token for symbol in shell_symbols)
            or "\n" in token
            or "\r" in token
            for token in command
        ):
            raise ValueError(f"{name} command contains shell syntax")


def _validate_cardinality(protocol: Mapping) -> None:
    experiment = protocol["experiment"]
    if (
        len(experiment["seeds"]) != 20
        or len(experiment["frames"]) != 500
        or len(experiment["robots"]) != 14
        or experiment["expected_rows"]
        != (
            len(experiment["seeds"])
            * len(experiment["frames"])
            * len(experiment["robots"])
        )
    ):
        raise ValueError("registered grid cardinality differs")
    method_contract = protocol["method_contract"]
    if (
        len(method_contract["branch_ids"]) != 2
        or len(method_contract["continuous_starts"]) != 2
    ):
        raise ValueError("method branch cardinality differs")
    gates = protocol["gates"]
    if (
        len(gates["scientific_gate_order"]) != 9
        or len(gates["scientific_gate_records"]) != 9
    ):
        raise ValueError("scientific gate cardinality differs")
    if (
        len(gates["integrity_gate_order"]) != 14
        or len(gates["integrity_gate_records"]) != 14
    ):
        raise ValueError("integrity gate cardinality differs")
    if len(replay.SMOKE_CASE_IDS) != 18:
        raise ValueError("producer smoke cardinality differs")
    for name in ("smoke_a", "smoke_b", "smoke_analyzer_a", "smoke_analyzer_b"):
        if protocol["invocations"][name]["expected_rows"] != 18:
            raise ValueError(f"{name} cardinality differs")
    for name in ("registered_replay", "registered_analyzer"):
        if protocol["invocations"][name]["expected_rows"] != 140000:
            raise ValueError(f"{name} cardinality differs")
    analysis_lists = protocol["analysis_schema"][
        "list_order_and_cardinality"
    ]
    if (
        analysis_lists.get("registered_scientific_gate_count") != 9
        or analysis_lists.get("smoke_scientific_gate_count") != 0
        or analysis_lists.get("integrity_gate_count") != 14
        or analysis_lists.get("smoke_expected_rows") != 18
        or analysis_lists.get("registered_expected_rows") != 140000
    ):
        raise ValueError("analysis schema cardinality differs")


def _validate_schema_ids(protocol: Mapping) -> None:
    expected = (
        (
            protocol["schema_id"],
            "cbf2026-two-range-reacquisition-protocol-v2",
        ),
        (
            protocol["registration_schema_id"],
            "cbf2026-two-range-reacquisition-registration-v2",
        ),
        (
            protocol["raw_schema"]["schema_id"],
            "cbf2026-two-range-reacquisition-raw-v1",
        ),
        (
            protocol["analysis_schema"]["schema_id"],
            "cbf2026-two-range-reacquisition-analysis-v1",
        ),
    )
    if any(observed != required for observed, required in expected):
        raise ValueError("protocol schema declaration differs")
    if protocol["protocol_id"] != "cbf2026-two-range-reacquisition-v2":
        raise ValueError("protocol schema stable ID differs")
    if (
        protocol["experiment"]["method"]
        != "two_range_private_branch_reacquisition"
    ):
        raise ValueError("protocol schema method ID differs")


def _validate_thresholds(protocol: Mapping) -> None:
    expected_constants = {
        "maximum_public_prediction_age": 2,
        "innovation_reference_quantile": 11.829007011943707,
        "candidate_dedup_m": 1e-9,
        "motion_covariance_per_frame": 0.25,
        "reacquisition_reduced_cost_max": 9.0,
        "maximum_error_m": 50.0,
    }
    if protocol["estimator_constants"] != expected_constants:
        raise ValueError("estimator threshold declaration differs")
    expected_scientific = {
        "maximum_published_error_m_strictly_below": {
            "operator": "strictly_below",
            "threshold": 50.0,
        },
        "maximum_fresh_error_m_strictly_below": {
            "operator": "strictly_below",
            "threshold": 50.0,
        },
        "paired_both_fresh_p95_must_not_worsen": {
            "operator": "less_than_or_equal",
            "threshold": 0.0,
        },
        "fresh_availability_max_drop_fraction": {
            "operator": "less_than_or_equal",
            "threshold": 0.02,
        },
        "fresh_or_predicted_min_fraction": {
            "operator": "greater_than_or_equal",
            "threshold": 0.95,
        },
        "maximum_prediction_age_frames": {
            "operator": "less_than_or_equal",
            "threshold": 2,
        },
        "qualification_anchor_violations_allowed": {
            "operator": "equal",
            "threshold": 0,
        },
        "current_frame_provenance_violations_allowed": {
            "operator": "equal",
            "threshold": 0,
        },
        "ascending_dag_violations_allowed": {
            "operator": "equal",
            "threshold": 0,
        },
    }
    gates = protocol["gates"]
    if (
        gates["scientific_gate_order"] != list(expected_scientific)
        or gates["scientific_gate_records"] != expected_scientific
        or any(
            record != {"operator": "equal", "threshold": 0}
            for record in gates["integrity_gate_records"].values()
        )
    ):
        raise ValueError("scientific or integrity threshold differs")


def _validate_grid(protocol: Mapping) -> None:
    expected = {
        "method": "two_range_private_branch_reacquisition",
        "seeds": list(range(20260727, 20260747)),
        "frames": list(range(500)),
        "robots": list(range(1, 15)),
        "expected_rows": 140000,
        "key_order": ["method", "seed", "frame_index", "robot_id"],
        "ranging_sigma_m": 0.5,
        "frame_dt_seconds": 0.5,
        "measurement_seed_contract": "cbf2026-range-v1",
        "evidence_class": "paired_single_trajectory_development_only",
    }
    if protocol["experiment"] != expected:
        raise ValueError("registered experiment grid differs")


def _validate_method_contract(protocol: Mapping) -> None:
    contract = protocol["method_contract"]
    expected_values = {
        "structural_conditions": [
            "live_public_prediction_absent",
            "qualification_status_ok",
            "exactly_two_active_references",
            "active_references_are_current_fresh_mandatory_uavs",
            "active_references_have_strictly_lower_robot_ids",
            "active_base_references_absent",
            "active_optional_references_absent",
            "fixed_references_present",
            "ranges_and_provenance_valid",
        ],
        "branch_ids": ["circle_negative", "circle_positive"],
        "continuous_starts": ["circle_negative", "circle_positive"],
        "private_prior_allowed_roles": [
            "discrete_branch_identity_selection"
        ],
        "private_prior_forbidden_roles": [
            "active_reference",
            "fim_term",
            "continuous_wnls_start",
            "continuous_update_input",
            "covariance_source",
            "publication_representative",
        ],
        "branch_score_rule": (
            "runtime_normalized_innovation_q_branch_less_than_or_equal_to_"
            "innovation_reference_quantile"
        ),
        "publication_rule": (
            "publish_only_exactly_one_passing_branch_bound_to_its_current_"
            "range_result"
        ),
        "failure_rule": (
            "zero_or_multiple_passing_branches_publish_unavailable_without_"
            "prediction"
        ),
    }
    if any(contract[field] != expected for field, expected in expected_values.items()):
        raise ValueError("method or smoke case contract differs")
    declaration = contract["synthetic_declaration"]
    if (
        not isinstance(declaration, Mapping)
        or tuple(declaration) != replay.SYNTHETIC_DECLARATION_FIELDS
        or declaration != replay.SYNTHETIC_DECLARATION
        or contract["synthetic_declaration_sha256"]
        != replay.SYNTHETIC_DECLARATION_SHA256
    ):
        raise ValueError("serialized smoke case declaration differs")


def _validate_comparator_contract(protocol: Mapping) -> None:
    _assert_comparator_binding_declaration()
    comparators = protocol["comparators"]
    expected = EXPECTED_COMPARATOR_BINDINGS
    replay_root = expected["v4_replay_root"]
    analysis_root = expected["v4_analysis_root"]
    if (
        comparators["v4_replay_root"] != replay_root
        or comparators["v4_analysis_root"] != analysis_root
    ):
        raise ValueError("comparator root differs from exact binding")
    expected_paths = {
        "v4_replay_manifest": f"{replay_root}/manifest.json",
        "v4_compressed_process": (
            f"{replay_root}/predictive-wnls-development.jsonl.gz"
        ),
        "v4_decompressed_process": (
            f"{replay_root}/predictive-wnls-development.jsonl.gz"
        ),
        "v4_analysis_manifest": f"{analysis_root}/manifest.json",
        "v4_analysis_json": (
            f"{analysis_root}/predictive-wnls-development.json"
        ),
        "v4_analysis_markdown": (
            f"{analysis_root}/predictive-wnls-development.md"
        ),
    }
    expected_hashes = {
        "v4_replay_manifest": expected["v4_manifest_sha256"],
        "v4_compressed_process": expected["v4_compressed_sha256"],
        "v4_decompressed_process": expected["v4_decompressed_sha256"],
        "v4_analysis_manifest": expected["v4_analysis_manifest_sha256"],
        "v4_analysis_json": expected["v4_analysis_json_sha256"],
        "v4_analysis_markdown": expected["v4_analysis_markdown_sha256"],
        "legacy_baseline_process": expected[
            "legacy_baseline_process_sha256"
        ],
    }
    for name, digest in expected_hashes.items():
        record = comparators[name]
        if (
            record["sha256"] != digest
            or (
                name in expected_paths
                and record["path"] != expected_paths[name]
            )
        ):
            raise ValueError(f"{name} comparator path or hash differs")
    compressed = comparators["v4_compressed_process"]
    decompressed = comparators["v4_decompressed_process"]
    if any(
        compressed[field] != decompressed[field]
        for field in ("path", "device", "inode", "size", "mtime_ns")
    ):
        raise ValueError("compressed/decompressed comparator identity differs")
    if (
        comparators["legacy_baseline_protocol_json_sha256"]
        != expected["legacy_baseline_protocol_json_sha256"]
    ):
        raise ValueError("legacy comparator protocol hash differs")


def _validate_authorization_contract(protocol: Mapping) -> None:
    expected = {
        "implementation_plan_approved": True,
        "protocol_preflight_required": True,
        "deterministic_smoke_review_required": True,
        "registered_full_grid_authorization": "pending_external_record",
        "authorization_record_schema": REGISTRATION_SCHEMA_ID,
        "authorization_record_path": AUTHORIZATION_RELATIVE_PATH,
    }
    if protocol["authorization"] != expected:
        raise ValueError("authorization record contract differs")


def _type_aware_deep_equal(observed: object, expected: object) -> bool:
    if isinstance(expected, Mapping):
        return (
            isinstance(observed, Mapping)
            and tuple(observed) == tuple(expected)
            and all(
                _type_aware_deep_equal(observed[key], expected[key])
                for key in expected
            )
        )
    if isinstance(expected, list):
        return (
            type(observed) is list
            and len(observed) == len(expected)
            and all(
                _type_aware_deep_equal(item, expected_item)
                for item, expected_item in zip(observed, expected)
            )
        )
    if isinstance(expected, tuple):
        return (
            type(observed) is tuple
            and len(observed) == len(expected)
            and all(
                _type_aware_deep_equal(item, expected_item)
                for item, expected_item in zip(observed, expected)
            )
        )
    return type(observed) is type(expected) and observed == expected


def _validate_exact_static_contract(protocol: Mapping) -> None:
    expected = _build_protocol(
        head=protocol["implementation_parent_commit"],
        sources=protocol["sources"],
        comparators=protocol["comparators"],
    )
    for section in (
        "binding_design",
        "experiment",
        "method_contract",
        "estimator_constants",
        "status_contract",
        "raw_schema",
        "analysis_schema",
        "gates",
        "disk_contract",
        "invocations",
        "evidence_lifecycle",
        "authorization",
        "commands",
    ):
        if not _type_aware_deep_equal(
            protocol[section],
            expected[section],
        ):
            raise ValueError(f"exact protocol {section} contract differs")


def _validate_protocol(protocol: object) -> None:
    """Reject any undeclared or reordered protocol object."""
    protocol = _require_object_order(
        protocol,
        PROTOCOL_FIELDS,
        label="protocol top-level order",
    )
    for section in (
        "binding_design",
        "sources",
        "comparators",
        "experiment",
        "method_contract",
        "estimator_constants",
        "status_contract",
        "raw_schema",
        "analysis_schema",
        "gates",
        "disk_contract",
        "invocations",
        "evidence_lifecycle",
        "authorization",
    ):
        _require_object_order(
            protocol[section],
            PROTOCOL_SECTION_FIELDS[section],
            label=f"{section} section order",
        )
    sources = protocol["sources"]
    for name in SOURCE_MEMBER_NAMES:
        _require_object_order(
            sources[name],
            PROTOCOL_SECTION_FIELDS["source_record"],
            label=f"{name} source record order",
        )
    comparators = protocol["comparators"]
    for name in (
        "v4_replay_manifest",
        "v4_compressed_process",
        "v4_decompressed_process",
        "v4_analysis_manifest",
        "v4_analysis_json",
        "v4_analysis_markdown",
        "legacy_baseline_process",
    ):
        _require_object_order(
            comparators[name],
            PROTOCOL_SECTION_FIELDS["source_record"],
            label=f"{name} comparator record order",
        )
    invocations = protocol["invocations"]
    for name in INVOCATION_MEMBER_NAMES:
        _require_object_order(
            invocations[name],
            PROTOCOL_SECTION_FIELDS["invocation_record"],
            label=f"{name} invocation record order",
        )
    _require_object_order(
        protocol["commands"],
        COMMAND_FIELDS,
        label="commands section order",
    )
    _validate_value_types(protocol)
    _validate_cardinality(protocol)
    _validate_schema_ids(protocol)
    _validate_thresholds(protocol)
    _validate_grid(protocol)
    _validate_method_contract(protocol)
    _validate_comparator_contract(protocol)
    _validate_authorization_contract(protocol)
    _validate_exact_static_contract(protocol)


REPOSITORY_SOURCE_PATHS = {
    "implementation_plan": Path(
        "docs/superpowers/plans/"
        "2026-07-31-cbf2026-two-range-smoke-v2-recovery.md"
    ),
    "two_range_reacquisition_source": Path(
        "scripts/diagnostics/two_range_reacquisition.py"
    ),
    "predictive_wnls_source": Path(
        "scripts/diagnostics/predictive_wnls.py"
    ),
    "fixture_extractor_source": Path(
        "scripts/diagnostics/extract_two_range_reacquisition_fixture.py"
    ),
    "replay_source": Path(
        "scripts/diagnostics/replay_two_range_reacquisition.py"
    ),
    "analyzer_source": Path(
        "scripts/diagnostics/analyze_two_range_reacquisition.py"
    ),
    "registrar_source": Path(
        "scripts/diagnostics/register_two_range_reacquisition.py"
    ),
    "mechanism_fixture": Path(
        "tests/fixtures/cbf2026_two_range_reacquisition/"
        "mechanism_20260727_180_12.json"
    ),
    "mechanism_fixture_manifest": Path(
        "tests/fixtures/cbf2026_two_range_reacquisition/manifest.json"
    ),
}


def _absolute(path: Path, *, base: Path | None = None) -> Path:
    path = Path(path)
    if ".." in path.parts:
        raise ValueError(f"path contains parent traversal: {path}")
    if not path.is_absolute():
        path = (Path.cwd() if base is None else base) / path
    if not path.is_absolute():
        raise ValueError(f"path is not absolute: {path}")
    return path


def _lstat_path(path: Path, *, leaf: str) -> os.stat_result | None:
    path = _absolute(path)
    chain = list(reversed(path.parents)) + [path]
    for index, component in enumerate(chain):
        is_leaf = index == len(chain) - 1
        try:
            metadata = component.lstat()
        except FileNotFoundError:
            if is_leaf and leaf == "absent":
                return None
            raise ValueError(
                f"missing trusted path component: {component}"
            ) from None
        if stat.S_ISLNK(metadata.st_mode):
            raise ValueError(
                f"symbolic-link path component is forbidden: {component}"
            )
        if is_leaf:
            if leaf == "absent":
                raise FileExistsError(f"trusted path already exists: {path}")
            if leaf == "file" and not stat.S_ISREG(metadata.st_mode):
                raise ValueError(f"trusted leaf must be a regular file: {path}")
            if leaf == "directory" and not stat.S_ISDIR(metadata.st_mode):
                raise ValueError(f"trusted leaf must be a directory: {path}")
            return metadata
    raise RuntimeError("unreachable path validation state")


def _leaf_identity_state(metadata: os.stat_result) -> tuple[int, int, int, int]:
    return (
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_size,
        metadata.st_mtime_ns,
    )


def _parent_identity_state(metadata: os.stat_result) -> tuple[int, int]:
    return metadata.st_dev, metadata.st_ino


def _open_bound_source_leaf(
    path: Path,
) -> tuple[int, int, tuple[int, int], tuple[int, int, int, int]]:
    parent = path.parent
    observed_parent = _lstat_path(parent, leaf="directory")
    assert observed_parent is not None
    parent_flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    parent_descriptor = os.open(parent, parent_flags)
    descriptor = None
    try:
        parent_state = _parent_identity_state(observed_parent)
        if _parent_identity_state(os.fstat(parent_descriptor)) != parent_state:
            raise ValueError(f"source parent identity changed: {parent}")
        try:
            observed_leaf = os.stat(
                path.name,
                dir_fd=parent_descriptor,
                follow_symlinks=False,
            )
        except FileNotFoundError:
            raise ValueError(
                f"source identity changed before open: {path}"
            ) from None
        if stat.S_ISLNK(observed_leaf.st_mode):
            raise ValueError(
                f"symbolic-link path component is forbidden: {path}"
            )
        if not stat.S_ISREG(observed_leaf.st_mode):
            raise ValueError(f"trusted leaf must be a regular file: {path}")
        leaf_state = _leaf_identity_state(observed_leaf)
        flags = (
            os.O_RDONLY
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NOFOLLOW", 0)
        )
        descriptor = os.open(
            path.name,
            flags,
            dir_fd=parent_descriptor,
        )
        opened_leaf = os.fstat(descriptor)
        if (
            not stat.S_ISREG(opened_leaf.st_mode)
            or _leaf_identity_state(opened_leaf) != leaf_state
        ):
            raise ValueError(f"source identity changed before open: {path}")
        return parent_descriptor, descriptor, parent_state, leaf_state
    except BaseException:
        if descriptor is not None:
            os.close(descriptor)
        os.close(parent_descriptor)
        raise


def _verify_bound_source_leaf(
    path: Path,
    *,
    parent_descriptor: int,
    parent_state: tuple[int, int],
    leaf_state: tuple[int, int, int, int],
    label: str,
) -> None:
    try:
        descriptor_leaf = os.stat(
            path.name,
            dir_fd=parent_descriptor,
            follow_symlinks=False,
        )
    except FileNotFoundError:
        raise ValueError(f"{label} identity changed: {path}") from None
    observed_parent = _lstat_path(path.parent, leaf="directory")
    observed_leaf = _lstat_path(path, leaf="file")
    assert observed_parent is not None
    assert observed_leaf is not None
    if (
        _parent_identity_state(os.fstat(parent_descriptor)) != parent_state
        or _parent_identity_state(observed_parent) != parent_state
        or _leaf_identity_state(descriptor_leaf) != leaf_state
        or _leaf_identity_state(observed_leaf) != leaf_state
    ):
        raise ValueError(f"{label} identity changed: {path}")


def _read_bound_source(
    path: Path,
    *,
    expected_sha256: str | None = None,
    expected_identity: Mapping | None = None,
    capture_payload: bool = True,
) -> tuple[bytes | None, dict]:
    path = _absolute(path)
    (
        parent_descriptor,
        descriptor,
        parent_state,
        leaf_state,
    ) = _open_bound_source_leaf(path)
    chunks = [] if capture_payload else None
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise ValueError(f"trusted leaf must be a regular file: {path}")
        digest = hashlib.sha256()
        bytes_read = 0
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
            bytes_read += len(chunk)
            if chunks is not None:
                chunks.append(chunk)
        after = os.fstat(descriptor)
        _verify_bound_source_leaf(
            path,
            parent_descriptor=parent_descriptor,
            parent_state=parent_state,
            leaf_state=leaf_state,
            label="source",
        )
    finally:
        os.close(descriptor)
        os.close(parent_descriptor)
    before_state = (
        before.st_dev,
        before.st_ino,
        before.st_size,
        before.st_mtime_ns,
    )
    after_state = (
        after.st_dev,
        after.st_ino,
        after.st_size,
        after.st_mtime_ns,
    )
    if before_state != after_state or bytes_read != after.st_size:
        raise ValueError(f"source changed while reading: {path}")
    identity = {
        "path": str(path),
        "device": after.st_dev,
        "inode": after.st_ino,
        "size": after.st_size,
        "mtime_ns": after.st_mtime_ns,
        "sha256": digest.hexdigest(),
    }
    if expected_sha256 is not None and identity["sha256"] != expected_sha256:
        raise ValueError(f"source hash changed or differs: {path}")
    if expected_identity is not None and identity != dict(expected_identity):
        raise ValueError(f"source identity changed: {path}")
    return None if chunks is None else b"".join(chunks), identity


_GZIP_HASH_VALIDATION_CACHE: set[tuple[object, ...]] = set()


def _read_bound_gzip_source(
    path: Path,
    *,
    expected_compressed_sha256: str,
    expected_decompressed_sha256: str,
) -> tuple[dict, dict]:
    """Verify both gzip hash domains through one no-follow descriptor."""
    path = _absolute(path)
    (
        parent_descriptor,
        descriptor,
        parent_state,
        leaf_state,
    ) = _open_bound_source_leaf(path)
    cache_key = None
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise ValueError(f"trusted leaf must be a regular file: {path}")
        compressed_digest = hashlib.sha256()
        bytes_read = 0
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            compressed_digest.update(chunk)
            bytes_read += len(chunk)
        if bytes_read != before.st_size:
            raise ValueError(f"compressed comparator short read: {path}")
        compressed_sha256 = compressed_digest.hexdigest()
        if compressed_sha256 != expected_compressed_sha256:
            raise ValueError(f"compressed comparator hash differs: {path}")
        cache_key = (
            str(path),
            before.st_dev,
            before.st_ino,
            before.st_size,
            before.st_mtime_ns,
            compressed_sha256,
            expected_decompressed_sha256,
        )
        if cache_key not in _GZIP_HASH_VALIDATION_CACHE:
            os.lseek(descriptor, 0, os.SEEK_SET)
            decompressed_digest = hashlib.sha256()
            with os.fdopen(descriptor, "rb", closefd=False) as raw:
                with gzip.GzipFile(fileobj=raw, mode="rb") as stream:
                    while True:
                        chunk = stream.read(1024 * 1024)
                        if not chunk:
                            break
                        decompressed_digest.update(chunk)
            if (
                decompressed_digest.hexdigest()
                != expected_decompressed_sha256
            ):
                raise ValueError(
                    f"decompressed comparator hash differs: {path}"
                )
        after = os.fstat(descriptor)
        _verify_bound_source_leaf(
            path,
            parent_descriptor=parent_descriptor,
            parent_state=parent_state,
            leaf_state=leaf_state,
            label="gzip comparator",
        )
    finally:
        os.close(descriptor)
        os.close(parent_descriptor)
    expected_state = (
        before.st_dev,
        before.st_ino,
        before.st_size,
        before.st_mtime_ns,
    )
    after_state = (
        after.st_dev,
        after.st_ino,
        after.st_size,
        after.st_mtime_ns,
    )
    if after_state != expected_state:
        raise ValueError(f"gzip comparator identity changed: {path}")
    assert cache_key is not None
    _GZIP_HASH_VALIDATION_CACHE.add(cache_key)
    compressed_identity = {
        "path": str(path),
        "device": before.st_dev,
        "inode": before.st_ino,
        "size": before.st_size,
        "mtime_ns": before.st_mtime_ns,
        "sha256": expected_compressed_sha256,
    }
    decompressed_identity = {
        **compressed_identity,
        "sha256": expected_decompressed_sha256,
    }
    return compressed_identity, decompressed_identity


def _strict_json_object(payload: bytes, *, path: Path) -> dict:
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f"duplicate JSON member in {path}: {key}")
            result[key] = value
        return result

    try:
        value = json.loads(
            payload.decode("utf-8"),
            object_pairs_hook=pairs,
            parse_constant=lambda token: (_ for _ in ()).throw(
                ValueError(f"non-finite JSON token in {path}: {token}")
            ),
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid strict JSON: {path}") from error
    if not isinstance(value, dict):
        raise ValueError(f"trusted JSON is not an object: {path}")
    return value


def _canonical_declared_identity(value: object, *, label: str) -> dict:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} identity is absent")
    try:
        return {
            field: value[field]
            for field in PROTOCOL_SECTION_FIELDS["source_record"]
        }
    except KeyError as error:
        raise ValueError(f"{label} identity is incomplete") from error


def _git(
    repository_root: Path,
    arguments: list[str],
) -> subprocess.CompletedProcess:
    completed = subprocess.run(
        ["git", "-C", str(repository_root), *arguments],
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        raise ValueError(
            f"Git command failed ({' '.join(arguments)}): "
            f"{completed.stderr.strip()}"
        )
    return completed


def _repository_head(repository_root: Path) -> str:
    top = _git(
        repository_root,
        ["rev-parse", "--show-toplevel"],
    ).stdout.strip()
    if Path(top) != repository_root:
        raise ValueError("repository root is not the Git top level")
    branch_result = subprocess.run(
        [
            "git",
            "-C",
            str(repository_root),
            "symbolic-ref",
            "--quiet",
            "--short",
            "HEAD",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    if branch_result.returncode == 1:
        raise ValueError("detached implementation parent is forbidden")
    if branch_result.returncode != 0:
        raise ValueError(
            "unable to resolve implementation branch: "
            f"{branch_result.stderr.strip()}"
        )
    branch = branch_result.stdout.strip()
    if not branch:
        raise ValueError("detached implementation parent is forbidden")
    head = _git(
        repository_root,
        ["rev-parse", "--verify", "HEAD"],
    ).stdout.strip()
    if not _canonical_hex(head, 40):
        raise ValueError("implementation parent OID is invalid")
    return head


def _assert_implementation_parent_ancestry(
    repository_root: Path,
    *,
    head: str,
) -> None:
    design_commit = BINDING_DESIGN["commit"]
    completed = subprocess.run(
        [
            "git",
            "-C",
            str(repository_root),
            "merge-base",
            "--is-ancestor",
            design_commit,
            head,
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode == 1:
        raise ValueError(
            "implementation parent is not a descendant of the "
            "approved design ancestor"
        )
    if completed.returncode != 0:
        raise ValueError(
            "unable to verify approved design ancestor: "
            f"{completed.stderr.strip()}"
        )


def _assert_non_circular_protocol_parent(
    repository_root: Path,
    *,
    head: str,
) -> None:
    for relative in GENERATED_PROTOCOL_PATHS:
        tracked = _git(
            repository_root,
            ["ls-tree", "--name-only", head, "--", relative],
        ).stdout.splitlines()
        if relative in tracked:
            raise ValueError(
                "circular implementation parent already contains "
                f"generated protocol: {relative}"
            )


def _assert_comparator_binding_declaration() -> None:
    if (
        tuple(EXPECTED_COMPARATOR_BINDINGS.items())
        != _BOUND_COMPARATOR_ITEMS
    ):
        raise ValueError(
            "comparator binding declaration has unresolved, extra, "
            "renamed, or changed members"
        )


def _resolve_comparators() -> tuple[dict, dict[str, dict]]:
    _assert_comparator_binding_declaration()
    expected = EXPECTED_COMPARATOR_BINDINGS
    replay_root = Path(expected["v4_replay_root"])
    analysis_root = Path(expected["v4_analysis_root"])
    replay_manifest_path = replay_root / "manifest.json"
    replay_payload, replay_manifest_identity = _read_bound_source(
        replay_manifest_path,
        expected_sha256=expected["v4_manifest_sha256"],
    )
    replay_manifest = _strict_json_object(
        replay_payload,
        path=replay_manifest_path,
    )
    if (
        replay_manifest.get("output_root") != str(replay_root)
        or replay_manifest.get("status") != "completed"
        or replay_manifest.get("compressed_process_sha256")
        != expected["v4_compressed_sha256"]
        or replay_manifest.get("decompressed_process_sha256")
        != expected["v4_decompressed_sha256"]
    ):
        raise ValueError("v4 replay comparator manifest differs")
    process_path = replay_root / "predictive-wnls-development.jsonl.gz"
    compressed_identity, decompressed_identity = _read_bound_gzip_source(
        process_path,
        expected_compressed_sha256=expected["v4_compressed_sha256"],
        expected_decompressed_sha256=expected["v4_decompressed_sha256"],
    )
    analysis_paths = {
        "v4_analysis_manifest": analysis_root / "manifest.json",
        "v4_analysis_json": (
            analysis_root / "predictive-wnls-development.json"
        ),
        "v4_analysis_markdown": (
            analysis_root / "predictive-wnls-development.md"
        ),
    }
    analysis_hashes = {
        "v4_analysis_manifest": expected["v4_analysis_manifest_sha256"],
        "v4_analysis_json": expected["v4_analysis_json_sha256"],
        "v4_analysis_markdown": expected["v4_analysis_markdown_sha256"],
    }
    analysis_records = {}
    analysis_payloads = {}
    for name, path in analysis_paths.items():
        payload, identity = _read_bound_source(
            path,
            expected_sha256=analysis_hashes[name],
            capture_payload=name == "v4_analysis_manifest",
        )
        analysis_payloads[name] = payload
        analysis_records[name] = identity
    analysis_manifest = _strict_json_object(
        analysis_payloads["v4_analysis_manifest"],
        path=analysis_paths["v4_analysis_manifest"],
    )
    if (
        analysis_manifest.get("output_root") != str(analysis_root)
        or analysis_manifest.get("status") != "completed"
    ):
        raise ValueError("v4 analysis comparator manifest differs")
    baseline_declaration = _canonical_declared_identity(
        replay_manifest.get("source_identities", {}).get("baseline_process"),
        label="legacy baseline process",
    )
    _, baseline_identity = _read_bound_source(
        Path(baseline_declaration["path"]),
        expected_sha256=expected["legacy_baseline_process_sha256"],
        expected_identity=baseline_declaration,
        capture_payload=False,
    )
    protocol_declaration = _canonical_declared_identity(
        replay_manifest.get("protocol_identity"),
        label="legacy baseline protocol",
    )
    _read_bound_source(
        Path(protocol_declaration["path"]),
        expected_sha256=expected[
            "legacy_baseline_protocol_json_sha256"
        ],
        expected_identity=protocol_declaration,
        capture_payload=False,
    )
    external_sources = {}
    source_identities = replay_manifest.get("source_identities")
    if not isinstance(source_identities, Mapping):
        raise ValueError("v4 source identities are absent")
    for name in ("truth_data", "input_manifest"):
        declaration = _canonical_declared_identity(
            source_identities.get(name),
            label=name,
        )
        _, identity = _read_bound_source(
            Path(declaration["path"]),
            expected_sha256=declaration["sha256"],
            expected_identity=declaration,
            capture_payload=False,
        )
        external_sources[name] = identity
    comparators = {
        "v4_replay_root": str(replay_root),
        "v4_replay_manifest": replay_manifest_identity,
        "v4_compressed_process": compressed_identity,
        "v4_decompressed_process": decompressed_identity,
        "v4_analysis_root": str(analysis_root),
        "v4_analysis_manifest": analysis_records["v4_analysis_manifest"],
        "v4_analysis_json": analysis_records["v4_analysis_json"],
        "v4_analysis_markdown": analysis_records[
            "v4_analysis_markdown"
        ],
        "legacy_baseline_process": baseline_identity,
        "legacy_baseline_protocol_json_sha256": expected[
            "legacy_baseline_protocol_json_sha256"
        ],
    }
    _validate_comparator_contract(
        {
            "comparators": comparators,
        }
    )
    return comparators, external_sources


def _source_contract(
    repository_root: Path,
    external_sources: Mapping[str, Mapping],
) -> dict:
    sources = {}
    for name, relative in REPOSITORY_SOURCE_PATHS.items():
        _, identity = _read_bound_source(
            repository_root / relative,
            capture_payload=False,
        )
        sources[name] = identity
    for name in ("truth_data", "input_manifest"):
        sources[name] = copy.deepcopy(dict(external_sources[name]))
    return sources


def _verify_repository_sources(
    repository_root: Path,
    *,
    head: str,
    sources: Mapping[str, Mapping],
) -> None:
    relative_paths = [
        relative.as_posix()
        for relative in REPOSITORY_SOURCE_PATHS.values()
    ]
    dirty = _git(
        repository_root,
        [
            "status",
            "--porcelain=v1",
            "--untracked-files=all",
            "--",
            *relative_paths,
        ],
    ).stdout
    if dirty:
        raise ValueError(
            "dirty required source cannot be registered: "
            + dirty.strip()
        )
    for name, relative in REPOSITORY_SOURCE_PATHS.items():
        tracked = _git(
            repository_root,
            ["ls-files", "--error-unmatch", "--", relative.as_posix()],
        ).stdout.strip()
        if tracked != relative.as_posix():
            raise ValueError(f"required source is not tracked: {relative}")
        completed = subprocess.run(
            [
                "git",
                "-C",
                str(repository_root),
                "show",
                f"{head}:{relative.as_posix()}",
            ],
            capture_output=True,
            check=False,
        )
        if completed.returncode != 0:
            raise ValueError(f"required source blob is absent: {relative}")
        if (
            hashlib.sha256(completed.stdout).hexdigest()
            != sources[name]["sha256"]
        ):
            raise ValueError(
                f"dirty required source differs from HEAD: {relative}"
            )


def _validate_binding_files(repository_root: Path) -> None:
    for path_field, hash_field in (
        ("path", "sha256"),
        ("review_path", "review_sha256"),
    ):
        path = repository_root / BINDING_DESIGN[path_field]
        _read_bound_source(
            path,
            expected_sha256=BINDING_DESIGN[hash_field],
            capture_payload=False,
        )


def _assert_registered_roots_absent() -> None:
    if tuple(ROOTS.items()) != _BOUND_ROOT_ITEMS:
        raise ValueError(
            "current v2 root contract changed"
        )
    if RETIRED_ROOTS != _BOUND_RETIRED_ROOTS:
        raise ValueError("retired v1 root contract changed")
    guarded_roots = (
        *ROOTS.items(),
        *((f"retired_v1_{index}", root)
          for index, root in enumerate(RETIRED_ROOTS)),
    )
    for name, root in guarded_roots:
        path = Path(root)
        if path.is_symlink():
            raise ValueError(
                f"{name} registered output root is a symbolic-link: {root}"
            )
        if path.exists():
            raise FileExistsError(
                f"{name} registered output root already exists: {root}"
            )


def _strict_json_bytes(value: object) -> bytes:
    return (
        json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=False,
            indent=2,
            sort_keys=False,
        )
        + "\n"
    ).encode("utf-8")


def _markdown_bytes(protocol: Mapping, json_payload: bytes) -> bytes:
    lines = [
        "# CBF2026 Two-Range Reacquisition Protocol",
        "",
        "This protocol freezes deterministic smoke validation and one",
        "externally authorized, no-retry registered development replay.",
        "The paper gate remains `CLOSED`.",
        "",
        f"- Protocol schema: `{protocol['schema_id']}`",
        f"- Registration schema: `{protocol['registration_schema_id']}`",
        f"- Protocol ID: `{protocol['protocol_id']}`",
        (
            "- Implementation parent: "
            f"`{protocol['implementation_parent_commit']}`"
        ),
        f"- JSON SHA-256: `{hashlib.sha256(json_payload).hexdigest()}`",
        "",
        "## Exact commands",
        "",
    ]
    for name, argv in protocol["commands"].items():
        lines.extend(
            (
                f"### `{name}`",
                "",
                "```text",
                " ".join(argv),
                "```",
                "",
            )
        )
    return "\n".join(lines).encode("utf-8")


def _validate_markdown_json_pair(
    protocol: Mapping,
    *,
    markdown_payload: bytes,
    json_payload: bytes,
) -> None:
    parsed = _strict_json_object(
        json_payload,
        path=Path("<generated-protocol-json>"),
    )
    if _strict_json_bytes(parsed) != json_payload:
        raise ValueError("generated protocol JSON is not canonical")
    for field in (
        "schema_id",
        "registration_schema_id",
        "protocol_id",
        "implementation_parent_commit",
    ):
        if parsed.get(field) != protocol[field]:
            raise ValueError(
                f"generated protocol JSON changed semantic field: {field}"
            )
    digest = hashlib.sha256(json_payload).hexdigest().encode("ascii")
    binding = b"- JSON SHA-256: `" + digest + b"`"
    if markdown_payload.count(binding) != 1:
        raise ValueError(
            "generated protocol Markdown/JSON semantic binding differs"
        )


def _deterministic_protocol_bytes(
    protocol: Mapping,
) -> tuple[bytes, bytes]:
    json_payload = _strict_json_bytes(protocol)
    markdown_payload = _markdown_bytes(protocol, json_payload)
    repeated_json = _strict_json_bytes(protocol)
    repeated_markdown = _markdown_bytes(protocol, repeated_json)
    if (
        repeated_json != json_payload
        or repeated_markdown != markdown_payload
    ):
        raise ValueError("protocol dry runs are not byte deterministic")
    _validate_markdown_json_pair(
        protocol,
        markdown_payload=markdown_payload,
        json_payload=json_payload,
    )
    return markdown_payload, json_payload


def _directory_flags() -> int:
    return (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )


def _open_output_parent(path: Path) -> tuple[int, tuple[int, int]]:
    metadata = _lstat_path(path.parent, leaf="directory")
    assert metadata is not None
    descriptor = os.open(path.parent, _directory_flags())
    try:
        observed = os.fstat(descriptor)
        if (
            not stat.S_ISDIR(observed.st_mode)
            or (observed.st_dev, observed.st_ino)
            != (metadata.st_dev, metadata.st_ino)
        ):
            raise ValueError(f"output parent identity changed: {path.parent}")
    except BaseException as primary_error:
        close_faults = _close_descriptors((descriptor,))
        _add_cleanup_notes(
            primary_error,
            [
                ("output parent descriptor close", fault)
                for fault in close_faults
            ],
        )
        raise
    return descriptor, (observed.st_dev, observed.st_ino)


def _write_all(descriptor: int, payload: bytes) -> None:
    offset = 0
    while offset < len(payload):
        written = os.write(descriptor, payload[offset:])
        if written <= 0:
            raise OSError("short write while freezing protocol")
        offset += written


def _path_matches_descriptor(
    *,
    parent_descriptor: int,
    name: str,
    descriptor: int,
) -> bool:
    owned = os.fstat(descriptor)
    observed = os.stat(
        name,
        dir_fd=parent_descriptor,
        follow_symlinks=False,
    )
    return (
        stat.S_ISREG(observed.st_mode)
        and (observed.st_dev, observed.st_ino)
        == (owned.st_dev, owned.st_ino)
    )


def _descriptor_matches_payload(descriptor: int, payload: bytes) -> bool:
    before = os.fstat(descriptor)
    if not stat.S_ISREG(before.st_mode):
        return False
    os.lseek(descriptor, 0, os.SEEK_SET)
    captured = bytearray()
    while True:
        chunk = os.read(descriptor, 1024 * 1024)
        if not chunk:
            break
        captured.extend(chunk)
    after = os.fstat(descriptor)
    stable_metadata = (
        before.st_dev,
        before.st_ino,
        before.st_size,
        before.st_mtime_ns,
        before.st_ctime_ns,
    ) == (
        after.st_dev,
        after.st_ino,
        after.st_size,
        after.st_mtime_ns,
        after.st_ctime_ns,
    )
    return (
        stable_metadata
        and after.st_size == len(payload)
        and hashlib.sha256(captured).digest()
        == hashlib.sha256(payload).digest()
    )


def _rollback_owned_entry(
    *,
    parent_descriptor: int,
    name: str,
    descriptor: int,
) -> list[BaseException]:
    faults = []
    settled = False
    quarantine_name = None
    restore_linked = False
    for _ in range(3):
        try:
            if not settled and quarantine_name is None:
                try:
                    _path_matches_descriptor(
                        parent_descriptor=parent_descriptor,
                        name=name,
                        descriptor=descriptor,
                    )
                except FileNotFoundError:
                    settled = True
                else:
                    quarantine_name = (
                        ".cbf2026-protocol-rollback-"
                        f"{secrets.token_hex(16)}"
                    )
                    reserve_flags = (
                        os.O_WRONLY
                        | os.O_CREAT
                        | os.O_EXCL
                        | getattr(os, "O_CLOEXEC", 0)
                        | getattr(os, "O_NOFOLLOW", 0)
                    )
                    reserve_descriptor = os.open(
                        quarantine_name,
                        reserve_flags,
                        0o600,
                        dir_fd=parent_descriptor,
                    )
                    os.close(reserve_descriptor)
                    try:
                        os.rename(
                            name,
                            quarantine_name,
                            src_dir_fd=parent_descriptor,
                            dst_dir_fd=parent_descriptor,
                        )
                    except BaseException:
                        os.unlink(
                            quarantine_name,
                            dir_fd=parent_descriptor,
                        )
                        quarantine_name = None
                        raise
            if not settled and quarantine_name is not None:
                observed = os.stat(
                    quarantine_name,
                    dir_fd=parent_descriptor,
                    follow_symlinks=False,
                )
                owned = os.fstat(descriptor)
                is_owned = (
                    stat.S_ISREG(observed.st_mode)
                    and (observed.st_dev, observed.st_ino)
                    == (owned.st_dev, owned.st_ino)
                )
                if is_owned:
                    os.unlink(
                        quarantine_name,
                        dir_fd=parent_descriptor,
                    )
                else:
                    if not restore_linked:
                        try:
                            os.link(
                                quarantine_name,
                                name,
                                src_dir_fd=parent_descriptor,
                                dst_dir_fd=parent_descriptor,
                                follow_symlinks=False,
                            )
                        except FileExistsError:
                            raise RuntimeError(
                                "foreign protocol target preserved in "
                                f"quarantine: {quarantine_name}"
                            ) from None
                        restore_linked = True
                    os.unlink(
                        quarantine_name,
                        dir_fd=parent_descriptor,
                    )
                quarantine_name = None
                settled = True
            os.fsync(parent_descriptor)
            return faults
        except BaseException as error:
            faults.append(error)
    return faults


def _close_descriptors(descriptors: tuple[int | None, ...]) -> list[BaseException]:
    faults = []
    for descriptor in descriptors:
        if descriptor is None:
            continue
        for _ in range(3):
            try:
                os.close(descriptor)
                break
            except BaseException as error:
                faults.append(error)
                try:
                    os.fstat(descriptor)
                except OSError:
                    break
    return faults


def _add_cleanup_notes(
    primary_error: BaseException,
    faults: list[tuple[str, BaseException]],
) -> None:
    for context, fault in faults:
        primary_error.add_note(
            "protocol cleanup fault "
            f"[{context}]: {type(fault).__name__}: {fault}"
        )


def _write_paired_outputs(
    *,
    output_markdown: Path,
    output_json: Path,
    markdown_payload: bytes,
    json_payload: bytes,
    final_probe,
) -> None:
    if output_markdown == output_json:
        raise ValueError("Markdown and JSON targets must differ")
    _lstat_path(output_markdown, leaf="absent")
    _lstat_path(output_json, leaf="absent")
    markdown_parent = json_parent = None
    markdown_descriptor = json_descriptor = None
    markdown_identity = json_identity = None
    flags = (
        os.O_RDWR
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    try:
        markdown_parent, markdown_identity = _open_output_parent(
            output_markdown
        )
        json_parent, json_identity = _open_output_parent(output_json)
        json_descriptor = os.open(
            output_json.name,
            flags,
            0o644,
            dir_fd=json_parent,
        )
        markdown_descriptor = os.open(
            output_markdown.name,
            flags,
            0o644,
            dir_fd=markdown_parent,
        )
        _write_all(json_descriptor, json_payload)
        _write_all(markdown_descriptor, markdown_payload)
        os.fsync(json_descriptor)
        os.fsync(markdown_descriptor)
        os.fsync(json_parent)
        if markdown_parent != json_parent:
            os.fsync(markdown_parent)
        final_probe()
        for path, parent, identity, descriptor in (
            (
                output_markdown,
                markdown_parent,
                markdown_identity,
                markdown_descriptor,
            ),
            (output_json, json_parent, json_identity, json_descriptor),
        ):
            parent_path = _lstat_path(path.parent, leaf="directory")
            assert parent_path is not None
            if (parent_path.st_dev, parent_path.st_ino) != identity:
                raise ValueError(f"output parent path changed: {path.parent}")
            if not _path_matches_descriptor(
                parent_descriptor=parent,
                name=path.name,
                descriptor=descriptor,
            ):
                raise ValueError(f"protocol target identity changed: {path}")
            expected_payload = (
                markdown_payload
                if path == output_markdown
                else json_payload
            )
            if not _descriptor_matches_payload(
                descriptor,
                expected_payload,
            ):
                raise ValueError(f"protocol target content changed: {path}")
    except BaseException as primary_error:
        cleanup_faults = []
        if markdown_parent is not None and markdown_descriptor is not None:
            cleanup_faults.extend(
                ("markdown target", fault)
                for fault in _rollback_owned_entry(
                    parent_descriptor=markdown_parent,
                    name=output_markdown.name,
                    descriptor=markdown_descriptor,
                )
            )
        if json_parent is not None and json_descriptor is not None:
            cleanup_faults.extend(
                ("JSON target", fault)
                for fault in _rollback_owned_entry(
                    parent_descriptor=json_parent,
                    name=output_json.name,
                    descriptor=json_descriptor,
                )
            )
        close_faults = _close_descriptors(
            (
                markdown_descriptor,
                json_descriptor,
                markdown_parent,
                json_parent,
            )
        )
        cleanup_faults.extend(
            ("descriptor close", fault) for fault in close_faults
        )
        markdown_descriptor = json_descriptor = None
        markdown_parent = json_parent = None
        _add_cleanup_notes(primary_error, cleanup_faults)
        raise
    finally:
        _close_descriptors(
            (
                markdown_descriptor,
                json_descriptor,
                markdown_parent,
                json_parent,
            )
        )


def _reverify_bound_state(
    *,
    repository_root: Path,
    head: str,
    sources: Mapping[str, Mapping],
    comparators: Mapping,
) -> None:
    if _repository_head(repository_root) != head:
        raise ValueError("implementation parent changed during registration")
    _validate_binding_files(repository_root)
    for name, relative in REPOSITORY_SOURCE_PATHS.items():
        _read_bound_source(
            repository_root / relative,
            expected_sha256=sources[name]["sha256"],
            expected_identity=sources[name],
            capture_payload=False,
        )
    observed_comparators, observed_external = _resolve_comparators()
    if observed_comparators != comparators:
        raise ValueError("comparator identity changed during registration")
    for name in ("truth_data", "input_manifest"):
        if observed_external[name] != sources[name]:
            raise ValueError(f"{name} source identity changed")
    _verify_repository_sources(
        repository_root,
        head=head,
        sources=sources,
    )
    _assert_registered_roots_absent()


def register_two_range_protocol(
    *,
    repository_root: Path,
    output_markdown: Path,
    output_json: Path,
) -> tuple[Path, Path]:
    """Bind sources/evidence and publish deterministic paired protocol files."""
    repository_root = _absolute(repository_root)
    output_markdown = _absolute(output_markdown, base=repository_root)
    output_json = _absolute(output_json, base=repository_root)
    head = _repository_head(repository_root)
    _assert_implementation_parent_ancestry(
        repository_root,
        head=head,
    )
    _assert_non_circular_protocol_parent(
        repository_root,
        head=head,
    )
    _validate_binding_files(repository_root)
    _assert_registered_roots_absent()
    comparators, external_sources = _resolve_comparators()
    sources = _source_contract(repository_root, external_sources)
    _verify_repository_sources(
        repository_root,
        head=head,
        sources=sources,
    )
    protocol = _build_protocol(
        head=head,
        sources=sources,
        comparators=comparators,
    )
    _validate_protocol(protocol)
    markdown_payload, json_payload = _deterministic_protocol_bytes(protocol)

    def final_probe() -> None:
        _reverify_bound_state(
            repository_root=repository_root,
            head=head,
            sources=sources,
            comparators=comparators,
        )

    _write_paired_outputs(
        output_markdown=output_markdown,
        output_json=output_json,
        markdown_payload=markdown_payload,
        json_payload=json_payload,
        final_probe=final_probe,
    )
    return output_markdown, output_json


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Freeze the exact two-range reacquisition protocol."
    )
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--output-markdown", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    register_two_range_protocol(
        repository_root=arguments.repository_root,
        output_markdown=arguments.output_markdown,
        output_json=arguments.output_json,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
