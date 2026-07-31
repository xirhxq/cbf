"""Independent analyzer for two-range branch reacquisition evidence."""

from __future__ import annotations

from collections.abc import Mapping
from collections.abc import Iterable
import math
from collections import Counter, defaultdict
import argparse
from contextvars import ContextVar
from functools import wraps
import gzip
import hashlib
import inspect
import json
import os
import secrets
import stat
import sys
import threading
from typing import NamedTuple
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.diagnostics import replay_two_range_reacquisition as replay
from scripts.diagnostics.replay_localization_calibration import (
    fixed_references,
    stable_measurement_seed,
)
from scripts.diagnostics.run_diagnostic import DiskSpaceError
from scripts.diagnostics.replay_predictive_wnls_recovery import (
    _assert_registered_root,
)
from scripts.diagnostics.two_range_reacquisition import (
    BRANCH_IDS,
    branch_gate_passes,
    canonical_private_state,
    propagate_private_state,
    reset_private_state,
    solve_two_range_reacquisition,
)

ANALYSIS_SCHEMA_ID = "cbf2026-two-range-reacquisition-analysis-v1"
OUTPUT_JSON_NAME = "two-range-reacquisition.json"
OUTPUT_MARKDOWN_NAME = "two-range-reacquisition.md"
ANALYZER_MANIFEST_NAME = "manifest.json"
COMPACT_OUTPUT_CAP_BYTES = 10_000_000
ERROR_MESSAGE_MAX_UTF8_BYTES = 4096
ANALYZER_INVOCATIONS = (
    "smoke_analyzer_a",
    "smoke_analyzer_b",
    "registered_analyzer",
)
ANALYSIS_FIELDS = (
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
)
IDENTITY_FIELDS = (
    "protocol",
    "authorization",
    "mechanism_fixture",
    "synthetic_case_source",
    "raw_manifest",
    "raw_compressed_process",
    "raw_decompressed_process",
    "v4_manifest",
    "v4_compressed_process",
    "v4_decompressed_process",
    "v4_analysis_manifest",
    "v4_analysis_json",
    "v4_analysis_markdown",
    "legacy_baseline_process",
    "legacy_baseline_protocol_json",
    "truth_data",
)
ANALYSIS_IDENTITY_RECORD_FIELDS = (
    "path",
    "device",
    "inode",
    "size",
    "mtime_ns",
    "sha256",
    "hash_domain",
)
IDENTITY_PATH_SUFFIXES = {
    "protocol": tuple(
        Path(replay.REGISTERED_PROTOCOL_RELATIVE_PATH).parts
    ),
    "authorization": tuple(
        Path(replay.REGISTERED_AUTHORIZATION_RELATIVE_PATH).parts
    ),
    "raw_manifest": (ANALYZER_MANIFEST_NAME,),
    "raw_compressed_process": (replay.RAW_PROCESS_NAME,),
    "raw_decompressed_process": (replay.RAW_PROCESS_NAME,),
    "mechanism_fixture": ("mechanism_20260727_180_12.json",),
    "synthetic_case_source": (
        "scripts",
        "diagnostics",
        "replay_two_range_reacquisition.py",
    ),
    "v4_manifest": (ANALYZER_MANIFEST_NAME,),
    "v4_compressed_process": (
        "predictive-wnls-development.jsonl.gz",
    ),
    "v4_decompressed_process": (
        "predictive-wnls-development.jsonl.gz",
    ),
    "v4_analysis_manifest": (ANALYZER_MANIFEST_NAME,),
    "v4_analysis_json": ("predictive-wnls-development.json",),
    "v4_analysis_markdown": ("predictive-wnls-development.md",),
    "legacy_baseline_process": ("calibration.jsonl.gz",),
    "legacy_baseline_protocol_json": (
        "docs",
        "diagnostics",
        "2026-07-30-predictive-wnls-stage1-protocol-v4.json",
    ),
    "truth_data": ("data.json",),
}
BUDGET_FIELDS = (
    "expected_rows",
    "observed_rows",
    "unique_rows",
    "raw_allocated_bytes",
    "compact_allocated_bytes",
)
STATUS_COUNT_FIELDS = (
    "attempt_accepted",
    "attempt_rejected",
    "attempt_failed",
    "attempt_invalid",
    "attempt_reference_unavailable",
    "output_fresh",
    "output_predicted",
    "output_unavailable",
)
SELECTOR_ACCOUNTING_FIELDS = (
    "considered",
    "accepted",
    "rejected",
    "unavailable",
    "pre_score_rejections",
    "post_score_rejections",
    "score_exactly_one",
    "score_none",
    "score_multiple",
    "score_not_evaluated",
    "root_rejections",
    "downstream_unavailable",
    "outage_episode_count",
    "outage_episode_lengths",
    "fresh_contained",
    "fresh_not_contained",
)
BASELINE_FRESH_TRANSITION_FIELDS = (
    "baseline_fresh_total",
    "new_fresh",
    "new_predicted",
    "new_unavailable",
)
V4_DESCRIPTIVE_COMPARISON_FIELDS = (
    "fresh_transitions",
    "paired_both_fresh",
    "tails",
)
V4_FRESH_TRANSITION_FIELDS = (
    "v4_fresh_total",
    "new_fresh",
    "new_predicted",
    "new_unavailable",
)
V4_PAIRED_COMPARISON_FIELDS = (
    "cohort_size",
    "v4_p95_m",
    "new_p95_m",
    "new_minus_v4_p95_m",
)
PAIRED_COMPARISON_FIELDS = (
    "cohort_size",
    "baseline_p95_m",
    "new_p95_m",
    "new_minus_baseline_p95_m",
)
GATE_RECORD_FIELDS = (
    "gate_id",
    "operator",
    "threshold",
    "numerator",
    "denominator",
    "value",
    "passed",
)
TAIL_RECORD_FIELDS = (
    "metric",
    "stratifier",
    "stratum",
    "population",
    "count",
    "minimum",
    "p50",
    "p95",
    "maximum",
)
TAIL_METRICS = (
    "offline_error_norm",
    "offline_fresh_q_error",
)
TAIL_STRATIFIERS = (
    "depth",
    "seed",
    "robot",
    "time_bin",
    "private_age",
)
V4_TAIL_STRATIFIERS = (
    "depth",
    "seed",
    "robot",
    "time_bin",
)
TIME_BINS = (
    (0, 99),
    (100, 199),
    (200, 299),
    (300, 399),
    (400, 499),
)
TAIL_POPULATIONS = (
    "two_range_accepted_fresh",
    "v4_fresh",
)
SMOKE_SEMANTIC_FIELDS = (
    "schema_id",
    "protocol_id",
    "decision",
    "expected_rows",
    "observed_rows",
    "unique_rows",
    "status_counts",
    "selector_accounting",
    "baseline_fresh_transitions",
    "v4_descriptive_comparison",
    "paired_comparison",
    "scientific_gates",
    "integrity_gates",
    "tails",
    "limitations",
)
SOURCE_PROJECTION_FIELDS = (
    "invocation_name",
    "expected_rows",
    "observed_rows",
    "unique_rows",
    "status_counts",
    "selector_accounting",
    "baseline_fresh_transitions",
    "v4_fresh_transitions",
    "paired_comparison",
    "v4_paired_comparison",
    "maximum_published_error_m",
    "maximum_fresh_error_m",
    "maximum_prediction_age_frames",
    "integrity_counts",
    "scientific_gates",
    "integrity_gates",
)
RAW_ORIGIN_IDENTITY_NAMES = (
    "raw_manifest",
    "raw_compressed_process",
    "raw_decompressed_process",
)
ANALYSIS_MANIFEST_FIELDS = (
    "schema_id",
    "protocol_id",
    "invocation_name",
    "status",
    "method",
    "output_root",
    "protocol_identity",
    "authorization_identity",
    "source_identities",
    "output_identities",
    "expected_rows",
    "observed_rows",
    "disk_contract",
    "started_at",
    "completed_at",
    "error",
)
ANALYSIS_MANIFEST_IDENTITY_FIELDS = (
    "path",
    "device",
    "inode",
    "size",
    "allocated_bytes",
    "mtime_ns",
    "sha256",
    "hash_domain",
)
ANALYSIS_OUTPUT_MEMBER_NAMES = (
    "analysis_json",
    "analysis_markdown",
)
ANALYSIS_SOURCE_MEMBER_NAMES = {
    "smoke_analyzer_a": (
        "raw_manifest",
        "raw_compressed_process",
        "raw_decompressed_process",
        "mechanism_fixture",
        "synthetic_case_source",
    ),
    "smoke_analyzer_b": (
        "raw_manifest",
        "raw_compressed_process",
        "raw_decompressed_process",
        "mechanism_fixture",
        "synthetic_case_source",
    ),
    "registered_analyzer": (
        "raw_manifest",
        "raw_compressed_process",
        "raw_decompressed_process",
        "v4_manifest",
        "v4_compressed_process",
        "v4_decompressed_process",
        "v4_analysis_manifest",
        "v4_analysis_json",
        "v4_analysis_markdown",
        "legacy_baseline_process",
        "legacy_baseline_protocol_json",
        "truth_data",
    ),
}
ANALYSIS_ERROR_FIELDS = ("type", "message")
INTEGRITY_GATE_IDS = (
    "nonfresh_anchor_use",
    "selector_reference_set_violation",
    "missing_fixed_reference_publication",
    "private_prior_role_violation",
    "noncircle_continuous_start",
    "noncircle_publication_representative",
    "nonruntime_branch_score",
    "branch_selection_reconstruction_mismatch",
    "nonunique_passing_branch_publication",
    "selected_result_binding_mismatch",
    "private_state_recursion_mismatch",
    "predicted_selector_output",
    "exact_denominator_violation",
    "preserved_contract_violation",
)
GATES = {
    "maximum_published_error_m_strictly_below": 50.0,
    "maximum_fresh_error_m_strictly_below": 50.0,
    "paired_both_fresh_p95_must_not_worsen": True,
    "fresh_availability_max_drop_fraction": 0.02,
    "fresh_or_predicted_min_fraction": 0.95,
    "maximum_prediction_age_frames": 2,
    "qualification_anchor_violations_allowed": 0,
    "current_frame_provenance_violations_allowed": 0,
    "ascending_dag_violations_allowed": 0,
}
LIMITATIONS = (
    "single_preserved_truth_trajectory",
    "estimator_outside_controller",
    "diagonal_marginal_covariance_fim_approximation",
    "shared_ancestor_cross_covariance_unmodeled",
    "three_sigma_radius_is_conditional_modeled_surrogate",
    "source_has_243_of_7000_component_bound_violations",
)

def _build_raw_origin_binding_lifecycle():
    """Create process-local origin operations without exporting mint power."""
    mint_authority = object()
    process_capability = object()
    binding_lock = threading.RLock()
    binding_registry = {}
    active_analysis_bindings = ContextVar(
        "two_range_raw_origin_bindings",
        default=None,
    )

    class RawOriginRecord(NamedTuple):
        projection_bytes: bytes
        projection_sha256: str
        invocation_name: str
        protocol_id: str
        protocol_identity_commitment: str | None
        raw_identity_commitment: str | None
        capability: bytes
        aggregation_nonce: bytes
        result_object: Mapping
        result_commitment: str
        process_capability: object

    class RawOriginBinding:
        """Opaque, frozen, process-local proof of raw aggregation origin."""

        __slots__ = (
            "__capability",
            "__projection_bytes",
            "__projection_sha256",
            "__invocation_name",
            "__protocol_id",
            "__protocol_identity_commitment",
            "__raw_identity_commitment",
            "__aggregation_nonce",
            "__result_commitment",
        )

        def __init__(
            self,
            authority: object,
            *,
            capability: bytes,
            projection_bytes: bytes,
            projection_sha256: str,
            invocation_name: str,
            protocol_id: str,
            protocol_identity_commitment: str | None,
            raw_identity_commitment: str | None,
            aggregation_nonce: bytes,
            result_commitment: str,
        ) -> None:
            if authority is not mint_authority:
                raise TypeError("raw-origin bindings are minted internally")
            values = {
                "capability": capability,
                "projection_bytes": projection_bytes,
                "projection_sha256": projection_sha256,
                "invocation_name": invocation_name,
                "protocol_id": protocol_id,
                "protocol_identity_commitment": (
                    protocol_identity_commitment
                ),
                "raw_identity_commitment": raw_identity_commitment,
                "aggregation_nonce": aggregation_nonce,
                "result_commitment": result_commitment,
            }
            for name, value in values.items():
                object.__setattr__(
                    self,
                    f"_RawOriginBinding__{name}",
                    value,
                )

        def __setattr__(self, name: str, value: object) -> None:
            del name, value
            raise AttributeError("raw-origin binding is frozen")

        def __getattribute__(self, name: str) -> object:
            if name.startswith("_RawOriginBinding__"):
                raise AttributeError("raw-origin binding is opaque")
            return object.__getattribute__(self, name)

        def __repr__(self) -> str:
            return "<opaque raw-origin binding>"

        def __copy__(self):
            raise TypeError("raw-origin binding cannot be copied")

        def __deepcopy__(self, memo):
            del memo
            raise TypeError("raw-origin binding cannot be copied")

        def __reduce__(self):
            raise TypeError("raw-origin binding cannot be serialized")

        def __reduce_ex__(self, protocol):
            del protocol
            raise TypeError("raw-origin binding cannot be serialized")

    def binding_attribute(binding: RawOriginBinding, name: str) -> object:
        return object.__getattribute__(
            binding, f"_RawOriginBinding__{name}"
        )

    def canonical_result_commitment(result: Mapping) -> str:
        if (
            not isinstance(result, Mapping)
            or tuple(result) != ANALYSIS_FIELDS
        ):
            raise ValueError(
                "raw-origin aggregation result is not canonical"
            )
        payload = replay.ordered_strict_json_bytes(
            result,
            ANALYSIS_FIELDS,
        )
        return hashlib.sha256(payload).hexdigest()

    def mint(
        projection_bytes: bytes,
        *,
        result: Mapping,
        invocation_name: str,
        protocol_id: str,
        protocol_identity_commitment: str | None,
        raw_identity_commitment: str | None,
    ) -> RawOriginBinding:
        if (
            not isinstance(projection_bytes, bytes)
            or invocation_name not in ANALYZER_INVOCATIONS
            or not isinstance(protocol_id, str)
            or not protocol_id
            or any(
                commitment is not None
                and (
                    not isinstance(commitment, str)
                    or len(commitment) != 64
                    or any(
                        character not in "0123456789abcdef"
                        for character in commitment
                    )
                )
                for commitment in (
                    protocol_identity_commitment,
                    raw_identity_commitment,
                )
            )
        ):
            raise ValueError(
                "raw-origin binding context is not canonical"
            )
        immutable_projection = bytes(projection_bytes)
        projection_sha256 = hashlib.sha256(
            immutable_projection
        ).hexdigest()
        result_commitment = canonical_result_commitment(result)
        capability = secrets.token_bytes(32)
        aggregation_nonce = secrets.token_bytes(32)
        binding = RawOriginBinding(
            mint_authority,
            capability=capability,
            projection_bytes=immutable_projection,
            projection_sha256=projection_sha256,
            invocation_name=invocation_name,
            protocol_id=protocol_id,
            protocol_identity_commitment=(
                protocol_identity_commitment
            ),
            raw_identity_commitment=raw_identity_commitment,
            aggregation_nonce=aggregation_nonce,
            result_commitment=result_commitment,
        )
        record = RawOriginRecord(
            projection_bytes=immutable_projection,
            projection_sha256=projection_sha256,
            invocation_name=invocation_name,
            protocol_id=protocol_id,
            protocol_identity_commitment=(
                protocol_identity_commitment
            ),
            raw_identity_commitment=raw_identity_commitment,
            capability=capability,
            aggregation_nonce=aggregation_nonce,
            result_object=result,
            result_commitment=result_commitment,
            process_capability=process_capability,
        )
        active = active_analysis_bindings.get()
        if active is not None:
            active.append(binding)
        with binding_lock:
            binding_registry[binding] = record
        return binding

    def revoke(binding: object) -> None:
        if type(binding) is not RawOriginBinding:
            return
        with binding_lock:
            binding_registry.pop(binding, None)

    def refresh(binding: object, result: Mapping) -> None:
        if type(binding) is not RawOriginBinding:
            raise ValueError(
                "currently minted raw-origin binding is required"
            )
        with binding_lock:
            record = binding_registry.get(binding)
            if record is None:
                raise ValueError(
                    "raw-origin binding is absent, revoked, or already "
                    "consumed"
                )
            if record.result_object is not result:
                raise ValueError(
                    "raw-origin aggregation result binding differs"
                )
            commitment = canonical_result_commitment(result)
            object.__setattr__(
                binding,
                "_RawOriginBinding__result_commitment",
                commitment,
            )
            binding_registry[binding] = record._replace(
                result_commitment=commitment
            )

    def claim(
        binding: object,
        *,
        result: Mapping,
        invocation_name: str,
        protocol_id: str,
        protocol_identity_commitment: str | None,
        raw_identity_commitment: str | None,
    ) -> bytes:
        if type(binding) is not RawOriginBinding:
            raise ValueError(
                "currently minted raw-origin binding is required for "
                "raw-derived source projection"
            )
        with binding_lock:
            record = binding_registry.pop(binding, None)
        if record is None:
            raise ValueError(
                "raw-origin binding is absent, revoked, or already "
                "consumed"
            )
        binding_values = (
            binding_attribute(binding, "projection_bytes"),
            binding_attribute(binding, "projection_sha256"),
            binding_attribute(binding, "invocation_name"),
            binding_attribute(binding, "protocol_id"),
            binding_attribute(
                binding, "protocol_identity_commitment"
            ),
            binding_attribute(binding, "raw_identity_commitment"),
            binding_attribute(binding, "capability"),
            binding_attribute(binding, "aggregation_nonce"),
            binding_attribute(binding, "result_commitment"),
        )
        record_values = (
            record.projection_bytes,
            record.projection_sha256,
            record.invocation_name,
            record.protocol_id,
            record.protocol_identity_commitment,
            record.raw_identity_commitment,
            record.capability,
            record.aggregation_nonce,
            record.result_commitment,
        )
        if (
            record.process_capability is not process_capability
            or binding_values != record_values
            or record.result_object is not result
            or canonical_result_commitment(result)
            != record.result_commitment
            or hashlib.sha256(record.projection_bytes).hexdigest()
            != record.projection_sha256
            or record.invocation_name != invocation_name
            or record.protocol_id != protocol_id
            or record.protocol_identity_commitment
            != protocol_identity_commitment
            or record.raw_identity_commitment
            != raw_identity_commitment
        ):
            raise ValueError(
                "raw-origin aggregation result or binding differs from "
                "raw-derived source projection capability or context"
            )
        return record.projection_bytes

    def cardinality() -> int:
        with binding_lock:
            return len(binding_registry)

    def with_minter(function):
        @wraps(function)
        def wrapped(*args, **kwargs):
            if "_raw_origin_minter" in kwargs:
                raise TypeError("raw-origin minter is closure-injected")
            return function(
                *args,
                _raw_origin_minter=mint,
                **kwargs,
            )

        wrapped.__signature__ = inspect.signature(function).replace(
            parameters=tuple(
                parameter
                for name, parameter in inspect.signature(
                    function
                ).parameters.items()
                if name != "_raw_origin_minter"
            )
        )
        return wrapped

    def with_analysis_scope(function):
        @wraps(function)
        def wrapped(*args, **kwargs):
            if "_refresh_raw_origin_binding" in kwargs:
                raise TypeError(
                    "raw-origin refresh is closure-injected"
                )
            active = []
            token = active_analysis_bindings.set(active)
            try:
                return function(
                    *args,
                    _refresh_raw_origin_binding=refresh,
                    **kwargs,
                )
            finally:
                for binding in active:
                    revoke(binding)
                active_analysis_bindings.reset(token)

        wrapped.__signature__ = inspect.signature(function).replace(
            parameters=tuple(
                parameter
                for name, parameter in inspect.signature(
                    function
                ).parameters.items()
                if name != "_refresh_raw_origin_binding"
            )
        )
        return wrapped

    return with_minter, with_analysis_scope, revoke, claim, cardinality


(
    _with_raw_origin_minter,
    _with_raw_origin_analysis_scope,
    _revoke_raw_origin_binding,
    _claim_raw_origin_binding,
    _raw_origin_binding_cardinality,
) = _build_raw_origin_binding_lifecycle()
del _build_raw_origin_binding_lifecycle


def linear_percentile(
    values: Iterable[float],
    fraction: float,
) -> float | None:
    """Return the frozen linear percentile used by compact evidence."""
    data = np.sort(np.asarray(tuple(values), dtype=float))
    if data.size == 0 or not np.isfinite(data).all():
        return None
    h = fraction * (data.size - 1)
    lower = int(math.floor(h))
    upper = int(math.ceil(h))
    weight = h - lower
    return float(data[lower] + weight * (data[upper] - data[lower]))


def _row_key(row: Mapping) -> tuple[int, int, int]:
    key = (row.get("seed"), row.get("frame_index"), row.get("robot_id"))
    if any(
        isinstance(value, bool) or not isinstance(value, int)
        for value in key
    ):
        raise ValueError("comparison key is not a strict integer triple")
    return key


def _exact_index(
    rows: Iterable[Mapping],
    *,
    name: str,
    require_key_order: bool = True,
) -> tuple[list[tuple[int, int, int]], dict[tuple[int, int, int], Mapping]]:
    keys = []
    indexed = {}
    previous = None
    for row in rows:
        if not isinstance(row, Mapping):
            raise ValueError(f"{name} row is not an object")
        key = _row_key(row)
        if key in indexed:
            raise ValueError(f"{name} contains a duplicate key")
        if require_key_order and previous is not None and key <= previous:
            raise ValueError(f"{name} keys are out of order")
        keys.append(key)
        indexed[key] = row
        previous = key
    return (keys if require_key_order else sorted(keys)), indexed


def _paired_record(
    left: Mapping,
    right: Mapping,
    keys: Iterable[tuple[int, int, int]],
    *,
    left_status_field: str,
    left_fresh_value: str,
    left_error_field: str,
    fields: tuple[str, ...],
    left_label: str,
) -> dict:
    cohort = [
        key
        for key in keys
        if left[key][left_status_field] == left_fresh_value
        and right[key]["output_status"] == "fresh"
    ]
    left_p95 = linear_percentile(
        (left[key][left_error_field] for key in cohort), 0.95
    )
    new_p95 = linear_percentile(
        (right[key]["offline_error_norm"] for key in cohort), 0.95
    )
    difference = (
        None
        if left_p95 is None or new_p95 is None
        else float(new_p95 - left_p95)
    )
    values = {
        "cohort_size": len(cohort),
        f"{left_label}_p95_m": left_p95,
        "new_p95_m": new_p95,
        f"new_minus_{left_label}_p95_m": difference,
    }
    return {field: values[field] for field in fields}


def _transition_record(
    source: Mapping,
    new: Mapping,
    keys: Iterable[tuple[int, int, int]],
    *,
    source_status_field: str,
    source_fresh_value: str,
    fields: tuple[str, ...],
    total_field: str,
) -> dict:
    cohort = [
        key
        for key in keys
        if source[key][source_status_field] == source_fresh_value
    ]
    counts = Counter(new[key]["output_status"] for key in cohort)
    values = {
        total_field: len(cohort),
        "new_fresh": counts["fresh"],
        "new_predicted": counts["predicted"],
        "new_unavailable": counts["unavailable"],
    }
    return {field: values[field] for field in fields}


def _tail_summary(
    values: Iterable[object],
    *,
    metric: str,
    stratifier: str,
    stratum: object,
    population: str,
) -> dict:
    data = tuple(float(value) for value in values if value is not None)
    if data:
        statistics = {
            "minimum": float(min(data)),
            "p50": linear_percentile(data, 0.5),
            "p95": linear_percentile(data, 0.95),
            "maximum": float(max(data)),
        }
    else:
        statistics = {
            "minimum": None,
            "p50": None,
            "p95": None,
            "maximum": None,
        }
    values_by_field = {
        "metric": metric,
        "stratifier": stratifier,
        "stratum": stratum,
        "population": population,
        "count": len(data),
        **statistics,
    }
    return {
        field: values_by_field[field]
        for field in TAIL_RECORD_FIELDS
    }


def _tail_records(
    rows: Iterable[Mapping],
    *,
    v4: bool,
) -> list[dict]:
    rows = tuple(rows)
    population = "v4_fresh" if v4 else "two_range_accepted_fresh"
    eligible = [
        row
        for row in rows
        if row["output_status"] == "fresh"
        and (
            v4
            or (
                row.get("selector_considered") is True
                and row.get("attempt_status") == "accepted"
            )
        )
    ]
    stratifiers = V4_TAIL_STRATIFIERS if v4 else TAIL_STRATIFIERS
    strata = {
        "depth": tuple(range(1, 8)),
        "seed": tuple(range(20260727, 20260747)),
        "robot": tuple(range(1, 15)),
        "time_bin": tuple(f"{start}-{end}" for start, end in TIME_BINS),
        "private_age": tuple(range(500)),
    }

    def matches(row: Mapping, stratifier: str, stratum: object) -> bool:
        if stratifier == "depth":
            return row.get("squad_local_index") == stratum
        if stratifier == "seed":
            return row.get("seed") == stratum
        if stratifier == "robot":
            return row.get("robot_id") == stratum
        if stratifier == "private_age":
            return row.get("branch_selection_prior_age_frames") == stratum
        start, end = next(
            interval
            for interval in TIME_BINS
            if f"{interval[0]}-{interval[1]}" == stratum
        )
        return start <= row.get("frame_index", -1) <= end

    return [
        _tail_summary(
            (
                row.get(metric)
                for row in eligible
                if matches(row, stratifier, stratum)
            ),
            metric=metric,
            stratifier=stratifier,
            stratum=stratum,
            population=population,
        )
        for metric in TAIL_METRICS
        for stratifier in stratifiers
        for stratum in strata[stratifier]
    ]


def _gate_record(
    gate_id: str,
    operator: str,
    threshold: object,
    numerator: int | None,
    denominator: int | None,
    value: float | int | None,
    passed: bool,
) -> dict:
    values = {
        "gate_id": gate_id,
        "operator": operator,
        "threshold": threshold,
        "numerator": numerator,
        "denominator": denominator,
        "value": value,
        "passed": bool(passed),
    }
    return {field: values[field] for field in GATE_RECORD_FIELDS}


def _scientific_gate_records(
    *,
    new_rows: list[Mapping],
    paired: Mapping,
    baseline_fresh_total: int,
    baseline_fresh_new_fresh: int,
    expected_rows: int,
) -> list[dict]:
    published_errors = [
        row["offline_error_norm"]
        for row in new_rows
        if row["output_status"] in {"fresh", "predicted"}
    ]
    fresh_errors = [
        row["offline_error_norm"]
        for row in new_rows
        if row["output_status"] == "fresh"
    ]
    maximum_published = (
        None if not published_errors else float(max(published_errors))
    )
    maximum_fresh = None if not fresh_errors else float(max(fresh_errors))
    fresh = sum(row["output_status"] == "fresh" for row in new_rows)
    available = sum(
        row["output_status"] in {"fresh", "predicted"}
        for row in new_rows
    )
    predicted_ages = [
        row["prediction_age"]
        for row in new_rows
        if row["output_status"] == "predicted"
    ]
    maximum_age = max(predicted_ages, default=0)
    anchor_violations = sum(
        record["used"] is True
        and record["current_freshness"] != "fresh"
        for row in new_rows
        for record in row.get("reference_evidence", ())
    )
    provenance_violations = sum(
        "provenance" in record["reason"]
        for row in new_rows
        for record in row.get("reference_violations", ())
    )
    dag_violations = sum(
        record["reference_kind"] == "uav"
        and record["reference_id"] >= row["robot_id"]
        for row in new_rows
        for record in row.get("active_references", ())
    )
    paired_difference = paired["new_minus_baseline_p95_m"]
    drop_fraction = (
        None
        if baseline_fresh_total == 0
        else max(
            0.0,
            (
                baseline_fresh_total - baseline_fresh_new_fresh
            )
            / baseline_fresh_total,
        )
    )
    availability_fraction = (
        None if expected_rows == 0 else available / expected_rows
    )
    return [
        _gate_record(
            "maximum_published_error_m_strictly_below",
            "strictly_below",
            50.0,
            int(
                maximum_published is not None
                and maximum_published >= 50.0
            ),
            len(published_errors),
            maximum_published,
            maximum_published is not None and maximum_published < 50.0,
        ),
        _gate_record(
            "maximum_fresh_error_m_strictly_below",
            "strictly_below",
            50.0,
            int(
                maximum_fresh is not None
                and maximum_fresh >= 50.0
            ),
            len(fresh_errors),
            maximum_fresh,
            maximum_fresh is not None and maximum_fresh < 50.0,
        ),
        _gate_record(
            "paired_both_fresh_p95_must_not_worsen",
            "less_than_or_equal",
            0.0,
            paired["cohort_size"],
            paired["cohort_size"],
            paired_difference,
            paired["cohort_size"] > 0
            and paired_difference is not None
            and paired_difference <= 0.0,
        ),
        _gate_record(
            "fresh_availability_max_drop_fraction",
            "less_than_or_equal",
            0.02,
            baseline_fresh_new_fresh,
            baseline_fresh_total,
            drop_fraction,
            drop_fraction is not None and drop_fraction <= 0.02,
        ),
        _gate_record(
            "fresh_or_predicted_min_fraction",
            "greater_than_or_equal",
            0.95,
            available,
            expected_rows,
            availability_fraction,
            availability_fraction is not None
            and availability_fraction >= 0.95,
        ),
        _gate_record(
            "maximum_prediction_age_frames",
            "less_than_or_equal",
            2,
            len(predicted_ages),
            len(predicted_ages),
            maximum_age,
            maximum_age <= 2,
        ),
        _gate_record(
            "qualification_anchor_violations_allowed",
            "equal",
            0,
            anchor_violations,
            expected_rows,
            anchor_violations,
            anchor_violations == 0,
        ),
        _gate_record(
            "current_frame_provenance_violations_allowed",
            "equal",
            0,
            provenance_violations,
            expected_rows,
            provenance_violations,
            provenance_violations == 0,
        ),
        _gate_record(
            "ascending_dag_violations_allowed",
            "equal",
            0,
            dag_violations,
            expected_rows,
            dag_violations,
            dag_violations == 0,
        ),
    ]


def _integrity_gate_records(
    counts: Mapping[str, int],
    *,
    denominator: int,
) -> list[dict]:
    return [
        _gate_record(
            gate_id,
            "equal",
            0,
            int(counts.get(gate_id, 0)),
            denominator,
            int(counts.get(gate_id, 0)),
            counts.get(gate_id, 0) == 0,
        )
        for gate_id in INTEGRITY_GATE_IDS
    ]


def _integrity_counts_from_rows(
    rows: list[Mapping],
    *,
    truth_data: Mapping,
    expected_rows: int,
    branch_representatives: Iterable[object] | None = None,
) -> dict[str, int]:
    counts = {gate_id: 0 for gate_id in INTEGRITY_GATE_IDS}
    expected_representatives = tuple(
        (None for _ in rows)
        if branch_representatives is None
        else branch_representatives
    )
    if len(expected_representatives) != len(rows):
        raise ValueError(
            "independent branch representatives differ from row count"
        )
    config = (
        truth_data.get("config")
        if isinstance(truth_data, Mapping)
        else None
    )
    for row, expected_branches in zip(
        rows, expected_representatives, strict=True
    ):
        considered = row["selector_considered"] is True
        evidence_records = row["reference_evidence"]
        if any(
            record["used"] is True
            and record["current_freshness"] != "fresh"
            for record in evidence_records
        ):
            counts["nonfresh_anchor_use"] += 1
        freshness_projection = [
            {
                "reference_kind": record["reference_kind"],
                "reference_id": record["reference_id"],
                "current_freshness": record["current_freshness"],
            }
            for record in row["reference_evidence"]
        ]
        if row["reference_freshness"] != freshness_projection:
            counts["preserved_contract_violation"] += 1
        elif row["frame_index"] is not None and any(
            record["noise_seed"]
            != stable_measurement_seed(
                row["seed"],
                row["frame_index"],
                row["robot_id"],
                record["reference_kind"],
                record["reference_id"],
            )
            for record in row["reference_evidence"]
        ):
            counts["preserved_contract_violation"] += 1
        if (
            row["prior_used_in_fim"] is True
            or row["prior_used_for_continuous_update"] is True
            or any(
                record["reference_kind"] == "private_prior"
                for record in row["active_references"]
            )
        ):
            counts["private_prior_role_violation"] += 1
        branches = row["branches"]
        observed_branches = tuple(
            (
                branch["branch_id"],
                tuple(branch["circle_start"]),
            )
            for branch in branches
        )
        if considered and (
            tuple(branch["branch_id"] for branch in branches)
            != BRANCH_IDS
            or expected_branches is None
            or observed_branches != expected_branches
        ):
            counts["noncircle_continuous_start"] += 1
        scored = [
            branch
            for branch in branches
            if branch["q_branch"] is not None
        ]
        if any(
            branch["passes_branch_gate"]
            is not branch_gate_passes(branch["q_branch"])
            for branch in scored
        ):
            counts["nonruntime_branch_score"] += 1
        passing = [
            branch
            for branch in scored
            if branch["passes_branch_gate"] is True
        ]
        selected = next(
            (
                branch
                for branch in branches
                if branch["branch_id"] == row["selected_branch_id"]
            ),
            None,
        )
        if considered and (
            (
                len(passing) == 1
                and row["selected_branch_id"]
                != passing[0]["branch_id"]
            )
            or (
                len(passing) != 1
                and row["selected_branch_id"] is not None
            )
        ):
            counts[
                "branch_selection_reconstruction_mismatch"
            ] += 1
        if (
            considered
            and row["output_status"] == "fresh"
            and len(passing) != 1
        ):
            counts["nonunique_passing_branch_publication"] += 1
        selected_binding_mismatch = False
        if considered and row["attempt_status"] == "accepted":
            if selected is None:
                selected_binding_mismatch = True
            else:
                result = selected["solver_result"]
                selected_binding_mismatch = (
                    row["output_status"] != "fresh"
                    or row["estimate"] != result["estimate"]
                    or row["fresh_modeled_covariance"]
                    != result["covariance"]
                    or row["fresh_epsilon"] != result["epsilon"]
                )
        if selected_binding_mismatch:
            counts["selected_result_binding_mismatch"] += 1
            counts["noncircle_publication_representative"] += 1
        if considered and row["output_status"] == "predicted":
            counts["predicted_selector_output"] += 1
        prior = _private_state_from_row(row, "branch_selection_prior")
        outgoing = _private_state_from_row(row, "next_private_state")
        recursion_mismatch = False
        if row["attempt_status"] == "accepted":
            recursion_mismatch = (
                outgoing is None
                or outgoing["source_fresh_frame"] != row["frame_index"]
                or outgoing["propagated_to_frame"] != row["frame_index"]
                or outgoing["age_frames"] != 0
                or outgoing["estimate"] != row["estimate"]
                or outgoing["modeled_covariance"]
                != row["fresh_modeled_covariance"]
            )
        elif outgoing != prior:
            recursion_mismatch = True
        if recursion_mismatch:
            counts["private_state_recursion_mismatch"] += 1
        if isinstance(config, Mapping):
            expected_mandatory = fixed_references(
                dict(config), row["robot_id"]
            )
            formation = config.get("formation")
            parts = (
                formation.get("parts")
                if isinstance(formation, Mapping)
                else None
            )
            number = config.get("num")
            if (
                isinstance(parts, int)
                and not isinstance(parts, bool)
                and parts > 0
                and isinstance(number, int)
                and not isinstance(number, bool)
                and number > 0
            ):
                squad_size = math.ceil(number / parts)
                expected_local_index = (
                    (row["robot_id"] - 1) % squad_size + 1
                )
                if row["squad_local_index"] != expected_local_index:
                    counts["preserved_contract_violation"] += 1
            if considered:
                expected_keys = [
                    *(
                        ("base", identifier)
                        for identifier in expected_mandatory["base_ids"]
                    ),
                    *(
                        ("uav", identifier)
                        for identifier in expected_mandatory["uav_ids"]
                    ),
                ]
                active_keys = [
                    (
                        record["reference_kind"],
                        record["reference_id"],
                    )
                    for record in row["active_references"]
                ]
                if (
                    row["mandatory_references"] != expected_mandatory
                    or
                    active_keys != expected_keys
                    or row["optional_candidates"] != []
                ):
                    counts[
                        "selector_reference_set_violation"
                    ] += 1
                evidence = {
                    (
                        record["reference_kind"],
                        record["reference_id"],
                    ): record
                    for record in row["reference_evidence"]
                }
                if any(
                    key not in evidence
                    or evidence[key]["used"] is not True
                    or evidence[key]["current_freshness"] != "fresh"
                    for key in expected_keys
                ):
                    counts[
                        "missing_fixed_reference_publication"
                    ] += 1
    if len(rows) != expected_rows or len(
        {
            (row["seed"], row["frame_index"], row["robot_id"])
            for row in rows
        }
    ) != expected_rows:
        counts["exact_denominator_violation"] += 1
    return counts


def _selector_accounting(rows: list[Mapping]) -> dict:
    considered = [row for row in rows if row["selector_considered"] is True]
    score_counts = Counter()
    pre_solver_reasons = {
        "two_range_robot_id_invalid",
        "two_range_input_invalid",
        "two_range_reference_covariance_invalid",
        "two_range_reference_keys_invalid",
        "two_range_private_prior_invalid",
        "two_range_base_anchor_provenance_invalid",
        "two_range_circle_geometry_invalid",
        "two_range_circle_starts_not_distinct",
    }
    for row in considered:
        scored = [
            branch["passes_branch_gate"]
            for branch in row["branches"]
            if branch["passes_branch_gate"] is not None
        ]
        if not scored:
            outcome = "not_evaluated"
        else:
            passes = sum(value is True for value in scored)
            outcome = "exactly_one" if passes == 1 else "none" if passes == 0 else "multiple"
        score_counts[outcome] += 1
    outages = []
    grouped = defaultdict(list)
    for row in rows:
        if all(
            isinstance(row.get(field), int)
            and not isinstance(row.get(field), bool)
            for field in ("seed", "frame_index", "robot_id")
        ):
            grouped[(row["seed"], row["robot_id"])].append(row)
    for group in grouped.values():
        current = 0
        previous_frame = None
        for row in sorted(group, key=lambda item: item["frame_index"]):
            unavailable = row["output_status"] == "unavailable"
            consecutive = (
                previous_frame is not None
                and row["frame_index"] == previous_frame + 1
            )
            if unavailable and (current == 0 or consecutive):
                current += 1
            elif unavailable:
                outages.append(current)
                current = 1
            elif current:
                outages.append(current)
                current = 0
            previous_frame = row["frame_index"]
        if current:
            outages.append(current)
    values = {
        "considered": len(considered),
        "accepted": sum(row["attempt_status"] == "accepted" for row in considered),
        "rejected": sum(row["attempt_status"] == "rejected" for row in considered),
        "unavailable": sum(
            row["attempt_status"] == "reference_unavailable"
            for row in considered
        ),
        "pre_score_rejections": sum(
            row["attempt_failure_reason"] in pre_solver_reasons
            for row in considered
        ),
        "post_score_rejections": sum(
            row["attempt_status"] == "rejected"
            and row["attempt_failure_reason"] not in pre_solver_reasons
            for row in considered
        ),
        "score_exactly_one": score_counts["exactly_one"],
        "score_none": score_counts["none"],
        "score_multiple": score_counts["multiple"],
        "score_not_evaluated": score_counts["not_evaluated"],
        "root_rejections": sum(
            row["attempt_status"] != "accepted"
            for row in considered
        ),
        "downstream_unavailable": sum(
            row["selector_considered"] is False
            and row["output_status"] == "unavailable"
            for row in rows
        ),
        "outage_episode_count": len(outages),
        "outage_episode_lengths": sorted(outages),
        "fresh_contained": sum(
            row["selector_considered"] is True
            and row["output_status"] == "fresh"
            and row["offline_fresh_containment"] is True
            for row in rows
        ),
        "fresh_not_contained": sum(
            row["selector_considered"] is True
            and row["output_status"] == "fresh"
            and row["offline_fresh_containment"] is False
            for row in rows
        ),
    }
    return {field: values[field] for field in SELECTOR_ACCOUNTING_FIELDS}


@_with_raw_origin_minter
def _aggregate_two_range_reacquisition_with_projection(
    *,
    baseline_rows: Iterable[Mapping],
    v4_rows: Iterable[Mapping],
    new_rows: Iterable[Mapping],
    truth_data: Mapping,
    protocol: Mapping,
    branch_representatives: Iterable[object] | None = None,
    protocol_identity_commitment: str | None = None,
    raw_identity_commitment: str | None = None,
    _raw_origin_minter,
) -> tuple[dict, object]:
    """Aggregate raw evidence and retain its immutable source projection."""
    baseline_rows = [
        row
        for row in baseline_rows
        if row.get("graph_case", "dynamic_dag_wnls")
        == "dynamic_dag_wnls"
    ]
    v4_rows = [
        row
        for row in v4_rows
        if row.get("variant", "predictive_multistart")
        == "predictive_multistart"
    ]
    new_rows = list(new_rows)
    for row in new_rows:
        _validate_exact_row_schema(row)
    baseline_keys, baseline = _exact_index(
        baseline_rows,
        name="legacy baseline",
        require_key_order=False,
    )
    v4_keys, v4 = _exact_index(v4_rows, name="v4 comparator")
    new_keys, new = _exact_index(new_rows, name="new method")
    if baseline_keys != new_keys or v4_keys != new_keys:
        raise ValueError("baseline/v4/new exact keys differ")
    expected_rows = len(new_keys)
    declared_expected = (
        protocol.get("experiment", {}).get("expected_rows")
        if isinstance(protocol.get("experiment"), Mapping)
        else None
    )
    if declared_expected is not None and declared_expected != expected_rows:
        raise ValueError("observed rows differ from protocol denominator")
    if expected_rows == 140000:
        observed = (
            (replay.METHOD_ID, *key)
            for key in new_keys
        )
        replay.validate_key_sequence(observed, replay.iter_registered_keys())

    paired = _paired_record(
        baseline,
        new,
        new_keys,
        left_status_field="status",
        left_fresh_value="converged",
        left_error_field="error_norm",
        fields=PAIRED_COMPARISON_FIELDS,
        left_label="baseline",
    )
    v4_paired = _paired_record(
        v4,
        new,
        new_keys,
        left_status_field="output_status",
        left_fresh_value="fresh",
        left_error_field="offline_error_norm",
        fields=V4_PAIRED_COMPARISON_FIELDS,
        left_label="v4",
    )
    baseline_transitions = _transition_record(
        baseline,
        new,
        new_keys,
        source_status_field="status",
        source_fresh_value="converged",
        fields=BASELINE_FRESH_TRANSITION_FIELDS,
        total_field="baseline_fresh_total",
    )
    v4_transitions = _transition_record(
        v4,
        new,
        new_keys,
        source_status_field="output_status",
        source_fresh_value="fresh",
        fields=V4_FRESH_TRANSITION_FIELDS,
        total_field="v4_fresh_total",
    )
    scientific = _scientific_gate_records(
        new_rows=new_rows,
        paired=paired,
        baseline_fresh_total=baseline_transitions[
            "baseline_fresh_total"
        ],
        baseline_fresh_new_fresh=baseline_transitions["new_fresh"],
        expected_rows=expected_rows,
    )
    integrity_counts = _integrity_counts_from_rows(
        new_rows,
        truth_data=truth_data,
        expected_rows=expected_rows,
        branch_representatives=branch_representatives,
    )
    integrity = _integrity_gate_records(
        integrity_counts, denominator=expected_rows
    )
    attempts = Counter(row["attempt_status"] for row in new_rows)
    outputs = Counter(row["output_status"] for row in new_rows)
    status_values = {
        "attempt_accepted": attempts["accepted"],
        "attempt_rejected": attempts["rejected"],
        "attempt_failed": attempts["failed"],
        "attempt_invalid": attempts["invalid"],
        "attempt_reference_unavailable": attempts[
            "reference_unavailable"
        ],
        "output_fresh": outputs["fresh"],
        "output_predicted": outputs["predicted"],
        "output_unavailable": outputs["unavailable"],
    }
    status_counts = {
        field: status_values[field] for field in STATUS_COUNT_FIELDS
    }
    selector_accounting = _selector_accounting(new_rows)
    tails = _tail_records(new_rows, v4=False)
    published_errors = [
        row["offline_error_norm"]
        for row in new_rows
        if row["output_status"] in {"fresh", "predicted"}
    ]
    fresh_errors = [
        row["offline_error_norm"]
        for row in new_rows
        if row["output_status"] == "fresh"
    ]
    maximum_published_error = (
        None
        if not published_errors
        else float(max(published_errors))
    )
    maximum_fresh_error = (
        None if not fresh_errors else float(max(fresh_errors))
    )
    maximum_prediction_age = max(
        (
            row["prediction_age"]
            for row in new_rows
            if row["output_status"] == "predicted"
        ),
        default=0,
    )
    v4_description = {
        "fresh_transitions": v4_transitions,
        "paired_both_fresh": v4_paired,
        "tails": _tail_records(v4_rows, v4=True),
    }
    decision = (
        "pass"
        if all(record["passed"] for record in (*scientific, *integrity))
        else "fail"
    )
    result = {
        "schema_id": ANALYSIS_SCHEMA_ID,
        "protocol_id": protocol.get("protocol_id", ""),
        "invocation_name": "registered_analyzer",
        "decision": decision,
        "semantic_payload_sha256": "",
        "identities": {field: None for field in IDENTITY_FIELDS},
        "budgets": {
            "expected_rows": expected_rows,
            "observed_rows": expected_rows,
            "unique_rows": len(new),
            "raw_allocated_bytes": 0,
            "compact_allocated_bytes": 0,
        },
        "status_counts": status_counts,
        "selector_accounting": selector_accounting,
        "baseline_fresh_transitions": baseline_transitions,
        "v4_descriptive_comparison": v4_description,
        "paired_comparison": paired,
        "scientific_gates": scientific,
        "integrity_gates": integrity,
        "tails": tails,
        "limitations": list(LIMITATIONS),
    }
    projection_values = {
        "invocation_name": "registered_analyzer",
        "expected_rows": expected_rows,
        "observed_rows": len(new_rows),
        "unique_rows": len(new),
        "status_counts": status_counts,
        "selector_accounting": selector_accounting,
        "baseline_fresh_transitions": baseline_transitions,
        "v4_fresh_transitions": v4_transitions,
        "paired_comparison": paired,
        "v4_paired_comparison": v4_paired,
        "maximum_published_error_m": maximum_published_error,
        "maximum_fresh_error_m": maximum_fresh_error,
        "maximum_prediction_age_frames": maximum_prediction_age,
        "integrity_counts": {
            gate_id: integrity_counts[gate_id]
            for gate_id in INTEGRITY_GATE_IDS
        },
        "scientific_gates": scientific,
        "integrity_gates": integrity,
    }
    source_projection = replay.ordered_strict_json_bytes(
        {
            field: projection_values[field]
            for field in SOURCE_PROJECTION_FIELDS
        },
        SOURCE_PROJECTION_FIELDS,
    )
    source_binding = _raw_origin_minter(
        source_projection,
        result=result,
        invocation_name="registered_analyzer",
        protocol_id=result["protocol_id"],
        protocol_identity_commitment=protocol_identity_commitment,
        raw_identity_commitment=raw_identity_commitment,
    )
    return result, source_binding


def aggregate_two_range_reacquisition(
    *,
    baseline_rows: Iterable[Mapping],
    v4_rows: Iterable[Mapping],
    new_rows: Iterable[Mapping],
    truth_data: Mapping,
    protocol: Mapping,
    branch_representatives: Iterable[object] | None = None,
) -> dict:
    """Join exact baseline/v4/new keys and compute frozen compact metrics."""
    result, source_binding = _aggregate_two_range_reacquisition_with_projection(
        baseline_rows=baseline_rows,
        v4_rows=v4_rows,
        new_rows=new_rows,
        truth_data=truth_data,
        protocol=protocol,
        branch_representatives=branch_representatives,
    )
    try:
        return result
    finally:
        _revoke_raw_origin_binding(source_binding)


def _validate_exact_row_schema(row: Mapping) -> None:
    if (
        not isinstance(row, Mapping)
        or tuple(row) != replay.ROW_FIELDS
        or set(row) != set(replay.ROW_FIELDS)
    ):
        raise ValueError("row differs from exact field schema")
    nested_lists = (
        ("branches", replay.BRANCH_FIELDS),
        ("existing_candidates", replay.EXISTING_CANDIDATE_FIELDS),
        ("optional_candidates", replay.REFERENCE_KEY_FIELDS),
        ("active_references", replay.REFERENCE_KEY_FIELDS),
        ("reference_evidence", replay.REFERENCE_EVIDENCE_FIELDS),
        ("reference_freshness", replay.REFERENCE_FRESHNESS_FIELDS),
        ("excluded_references", replay.EXCLUSION_FIELDS),
        ("reference_violations", replay.VIOLATION_FIELDS),
    )
    for name, fields in nested_lists:
        value = row[name]
        if not isinstance(value, list) or any(
            not isinstance(item, Mapping) or tuple(item) != fields
            for item in value
        ):
            raise ValueError(f"{name} differs from exact nested schema")
    if (
        not isinstance(row["mandatory_references"], Mapping)
        or tuple(row["mandatory_references"])
        != replay.MANDATORY_REFERENCE_FIELDS
    ):
        raise ValueError("mandatory references differ from exact schema")
    for branch in row["branches"]:
        result = branch["solver_result"]
        if (
            not isinstance(result, Mapping)
            or tuple(result) != replay.SOLVER_RESULT_FIELDS
            or not isinstance(result["proposal_trace"], list)
            or any(
                not isinstance(proposal, Mapping)
                or tuple(proposal) != replay.PROPOSAL_TRACE_FIELDS
                for proposal in result["proposal_trace"]
            )
        ):
            raise ValueError("branch solver result differs from exact schema")
    replay._validate_row(row)


def _private_state_from_row(row: Mapping, prefix: str) -> dict | None:
    if row[f"{prefix}_status"] == "absent":
        return None
    return {
        "status": "available",
        "estimate": row[f"{prefix}_estimate"],
        "modeled_covariance": row[f"{prefix}_covariance"],
        "source_fresh_frame": row[f"{prefix}_source_fresh_frame"],
        "propagated_to_frame": row[f"{prefix}_propagated_to_frame"],
        "age_frames": row[f"{prefix}_age_frames"],
    }


def _public_output_from_row(row: Mapping) -> dict:
    status = row["output_status"]
    return {
        "output_status": status,
        "prediction_age": row["prediction_age"],
        "estimate": row["estimate"],
        "modeled_covariance": (
            row["fresh_modeled_covariance"]
            if status == "fresh"
            else row["aged_modeled_covariance"]
            if status == "predicted"
            else None
        ),
        "epsilon": row["fresh_epsilon"] if status == "fresh" else None,
        "aged_modeled_radius": (
            row["aged_modeled_radius"] if status == "predicted" else None
        ),
        "base_anchor_provenance": row["base_anchor_provenance"],
    }


def _synthetic_private_prior(row: Mapping) -> dict | None:
    if (
        row["invocation_name"] != "smoke_validation"
        or row["smoke_case_kind"] != "selector"
    ):
        return None
    values = dict(replay.COMMON_SELECTOR_INPUT)
    values.update(row["smoke_case_input"]["overrides"])
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


def _reconstruct_private_state(
    row: Mapping,
    *,
    previous_private: object,
    held_command: object,
) -> dict | None:
    frame_index = row["frame_index"]
    command = held_command
    command_source = row["applied_command_source_frame"]
    if isinstance(held_command, Mapping):
        if tuple(held_command) != ("source_frame", "command"):
            raise ValueError("held command differs from exact schema")
        command_source = held_command["source_frame"]
        command = held_command["command"]
    if command_source != row["applied_command_source_frame"]:
        raise ValueError("held command source frame differs")
    if command != row["applied_command"]:
        raise ValueError("held command differs from raw row")

    expected_prior = None
    canonical_previous = canonical_private_state(previous_private)
    if frame_index is None:
        expected_prior = canonical_private_state(_synthetic_private_prior(row))
    elif canonical_previous is not None:
        if canonical_previous["propagated_to_frame"] == frame_index:
            expected_prior = canonical_previous
        else:
            expected_prior = propagate_private_state(
                canonical_previous,
                command,
                next_frame_index=frame_index,
            )
    elif frame_index != 0:
        expected_prior = None
    serialized_prior = canonical_private_state(
        _private_state_from_row(row, "branch_selection_prior")
    )
    if expected_prior != serialized_prior:
        raise ValueError("branch-selection private recursion mismatch")

    if row["attempt_status"] == "accepted":
        expected_outgoing = reset_private_state(
            _public_output_from_row(row),
            frame_index=(
                frame_index
                if frame_index is not None
                else expected_prior["propagated_to_frame"]
            ),
        )
    else:
        expected_outgoing = expected_prior
    serialized_outgoing = canonical_private_state(
        _private_state_from_row(row, "next_private_state")
    )
    if expected_outgoing != serialized_outgoing:
        raise ValueError("outgoing private recursion mismatch")
    return expected_outgoing


def _ranging_sigma(protocol: Mapping) -> float:
    for value in (
        protocol.get("ranging_sigma_m"),
        protocol.get("ranging_sigma"),
        (
            protocol.get("experiment", {}).get("ranging_sigma_m")
            if isinstance(protocol.get("experiment"), Mapping)
            else None
        ),
    ):
        if value is not None:
            return float(value)
    return 0.5


def _reference_public_state(
    current_public: Mapping,
    identifier: int,
) -> tuple[list, list, list]:
    state = current_public.get(identifier)
    if not isinstance(state, Mapping):
        raise ValueError("fixed current public reference is missing")
    if (
        state.get("output_status") != "fresh"
        or state.get("prediction_age") != 0
    ):
        raise ValueError("fixed reference is not current-frame fresh")
    estimate = state.get("estimate")
    covariance = state.get("modeled_covariance")
    if covariance is None:
        covariance = state.get("fresh_modeled_covariance")
    provenance = state.get("base_anchor_provenance")
    if (
        not isinstance(estimate, list)
        or not isinstance(covariance, list)
        or not isinstance(provenance, list)
    ):
        raise ValueError("fixed reference public state is incomplete")
    return estimate, covariance, provenance


def _expected_topology(row: Mapping, protocol: Mapping) -> dict | None:
    if row["frame_index"] is None:
        return None
    config = protocol.get("truth_config")
    if not isinstance(config, Mapping):
        config = protocol.get("_truth_config")
    if isinstance(config, Mapping):
        expected_mandatory = fixed_references(
            dict(config), row["robot_id"]
        )
        number = config.get("num")
        formation = config.get("formation")
        parts = (
            formation.get("parts")
            if isinstance(formation, Mapping)
            else None
        )
        if (
            isinstance(number, bool)
            or not isinstance(number, int)
            or isinstance(parts, bool)
            or not isinstance(parts, int)
            or number <= 0
            or parts <= 0
        ):
            raise ValueError("truth config squad topology is invalid")
        squad_size = math.ceil(number / parts)
        expected_local_index = (
            (row["robot_id"] - 1) % squad_size + 1
        )
    else:
        expected_mandatory = protocol.get(
            "_expected_mandatory_references"
        )
        expected_local_index = protocol.get(
            "_expected_squad_local_index"
        )
        if row["invocation_name"] == "registered_replay":
            raise ValueError("registered truth topology is absent")
    if (
        not isinstance(expected_mandatory, Mapping)
        or tuple(expected_mandatory)
        != replay.MANDATORY_REFERENCE_FIELDS
        or not isinstance(expected_local_index, int)
        or isinstance(expected_local_index, bool)
    ):
        raise ValueError("independent reference topology is absent")
    return {
        "mandatory_references": {
            field: list(expected_mandatory[field])
            for field in replay.MANDATORY_REFERENCE_FIELDS
        },
        "squad_local_index": expected_local_index,
    }


def _validate_topology_fields(row: Mapping, protocol: Mapping) -> dict | None:
    expected = _expected_topology(row, protocol)
    if expected is None:
        return None
    if row["squad_local_index"] != expected["squad_local_index"]:
        raise ValueError("squad local index differs from truth config")
    if row["mandatory_references"] != expected["mandatory_references"]:
        raise ValueError(
            "considered selector reference set differs from fixed topology"
        )
    return expected


def _validate_reference_runtime_contract(
    row: Mapping,
    *,
    current_public: Mapping,
) -> None:
    expected_freshness = []
    for record in row["reference_evidence"]:
        kind = record["reference_kind"]
        identifier = record["reference_id"]
        if row["frame_index"] is None:
            current_freshness = record["current_freshness"]
        elif kind == "base":
            current_freshness = "fresh"
        else:
            output = current_public.get(identifier)
            status = (
                output.get("output_status")
                if isinstance(output, Mapping)
                else None
            )
            current_freshness = (
                status
                if status in {"fresh", "predicted", "unavailable"}
                else "missing"
            )
        if record["current_freshness"] != current_freshness:
            raise ValueError(
                "reference freshness differs from current public state"
            )
        expected_freshness.append(
            {
                "reference_kind": kind,
                "reference_id": identifier,
                "current_freshness": current_freshness,
            }
        )
        if row["frame_index"] is not None:
            expected_seed = stable_measurement_seed(
                row["seed"],
                row["frame_index"],
                row["robot_id"],
                kind,
                identifier,
            )
            if record["noise_seed"] != expected_seed:
                raise ValueError(
                    "measurement noise seed differs from frozen derivation"
                )
    if row["reference_freshness"] != expected_freshness:
        raise ValueError(
            "reference freshness projection differs from evidence"
        )


def _branch_runtime_inputs(
    row: Mapping,
    *,
    protocol: Mapping,
    current_public: Mapping,
) -> dict:
    if (
        row["invocation_name"] == "smoke_validation"
        and row["smoke_case_kind"] == "selector"
    ):
        values = dict(replay.COMMON_SELECTOR_INPUT)
        values.update(row["smoke_case_input"]["overrides"])
        return {
            "robot_id": values["robot_id"],
            "reference_positions": values["reference_positions"],
            "reference_covariances": values["reference_covariances"],
            "measurements": values["measurements"],
            "reference_keys": values["reference_keys"],
            "private_prior": _private_state_from_row(
                row, "branch_selection_prior"
            ),
            "ranging_sigma": values["ranging_sigma"],
            "base_anchor_provenance": values["base_anchor_provenance"],
        }

    active_keys = tuple(
        (record["reference_kind"], record["reference_id"])
        for record in row["active_references"]
    )
    topology = _validate_topology_fields(row, protocol)
    mandatory = topology["mandatory_references"]
    fixed_keys = tuple(
        ("uav", identifier) for identifier in mandatory["uav_ids"]
    )
    if (
        mandatory["base_ids"] != []
        or row["optional_candidates"] != []
        or len(fixed_keys) != 2
        or active_keys != fixed_keys
        or any(kind != "uav" for kind, _ in fixed_keys)
        or any(identifier >= row["robot_id"] for _, identifier in fixed_keys)
    ):
        raise ValueError("considered selector reference set differs")
    evidence = {
        (record["reference_kind"], record["reference_id"]): record
        for record in row["reference_evidence"]
    }
    positions = []
    covariances = []
    measurements = []
    provenance = set()
    for key in fixed_keys:
        estimate, covariance, roots = _reference_public_state(
            current_public, key[1]
        )
        record = evidence.get(key)
        if (
            record is None
            or record["used"] is not True
            or record["eligible"] is not True
            or record["current_freshness"] != "fresh"
            or record["measurement_present"] is not True
            or record["noisy_range"] is None
        ):
            raise ValueError("fixed reference evidence differs from runtime")
        if record["base_anchor_provenance"] != roots:
            raise ValueError("fixed reference provenance differs")
        positions.append(estimate)
        covariances.append(covariance)
        measurements.append(record["noisy_range"])
        provenance.update(roots)
    provenance = sorted(provenance)
    if provenance != row["attempt_base_anchor_provenance"]:
        raise ValueError("recursive base provenance differs")
    return {
        "robot_id": row["robot_id"],
        "reference_positions": positions,
        "reference_covariances": covariances,
        "measurements": measurements,
        "reference_keys": [list(key) for key in fixed_keys],
        "private_prior": _private_state_from_row(
            row, "branch_selection_prior"
        ),
        "ranging_sigma": _ranging_sigma(protocol),
        "base_anchor_provenance": provenance,
    }


def _reconstruct_branches(
    row: Mapping,
    *,
    protocol: Mapping,
    current_public: Mapping,
) -> dict | None:
    if row["selector_considered"] is not True:
        return None
    if row["attempt_path"] != "two_range_reacquisition":
        raise ValueError("considered selector did not use two-range path")
    if row["output_status"] == "predicted":
        raise ValueError("two-range selector may not publish predicted output")
    inputs = _branch_runtime_inputs(
        row,
        protocol=protocol,
        current_public=current_public,
    )
    expected = solve_two_range_reacquisition(**inputs)
    comparisons = {
        "attempt_status": expected["attempt_status"],
        "attempt_failure_reason": expected["failure_reason"],
        "selected_branch_id": expected["selected_branch_id"],
        "prior_used_for_branch_selection": expected[
            "prior_used_for_branch_selection"
        ],
        "prior_used_in_fim": expected.get("prior_used_in_fim", False),
        "prior_used_for_continuous_update": expected.get(
            "prior_used_for_continuous_update", False
        ),
    }
    for field, value in comparisons.items():
        if row[field] != value:
            raise ValueError(f"reconstructed {field} differs")
    expected_branches = [
        replay._branch(branch) for branch in expected.get("branches", [])
    ]
    if row["branches"] != expected_branches:
        raise ValueError("reconstructed branch traces or scores differ")
    if row["attempt_status"] == "accepted":
        selected = next(
            branch
            for branch in expected_branches
            if branch["branch_id"] == expected["selected_branch_id"]
        )
        result = selected["solver_result"]
        if (
            result["cost"] > 9.0
            or row["output_status"] != "fresh"
            or row["estimate"] != result["estimate"]
            or row["fresh_modeled_covariance"] != result["covariance"]
            or row["fresh_epsilon"] != result["epsilon"]
            or row["base_anchor_provenance"]
            != sorted(inputs["base_anchor_provenance"])
        ):
            raise ValueError(
                "selected branch does not bind the fresh publication"
            )
    elif row["output_status"] == "fresh":
        raise ValueError("nonaccepted selector published a fresh result")
    return {
        "reference_keys": tuple(
            tuple(key) for key in inputs["reference_keys"]
        ),
        "selected_branch_id": expected["selected_branch_id"],
        "branches": expected_branches,
    }


def _validate_offline_metrics(
    row: Mapping,
    truth_position: object,
) -> None:
    offline_fields = (
        "offline_error_norm",
        "offline_fresh_containment",
        "offline_aged_radius_containment",
        "offline_fresh_q_error",
        "offline_aged_q_error",
    )
    if truth_position is None:
        if row["offline_truth_position"] is not None or any(
            row[field] is not None for field in offline_fields
        ):
            raise ValueError("row carries truth-dependent offline evidence")
        return
    truth = np.asarray(truth_position, dtype=float)
    if (
        truth.shape != (2,)
        or not np.isfinite(truth).all()
        or row["offline_truth_position"] != truth.tolist()
    ):
        raise ValueError("offline truth differs from protocol-bound truth")
    expected = {field: None for field in offline_fields}
    if row["output_status"] != "unavailable":
        estimate = np.asarray(row["estimate"], dtype=float)
        residual = truth - estimate
        error = float(np.linalg.norm(residual))
        expected["offline_error_norm"] = error
        if row["output_status"] == "fresh":
            covariance = np.asarray(
                row["fresh_modeled_covariance"], dtype=float
            )
            expected["offline_fresh_containment"] = bool(
                error <= row["fresh_epsilon"]
            )
            expected["offline_fresh_q_error"] = float(
                residual @ np.linalg.solve(covariance, residual)
            )
        else:
            covariance = np.asarray(
                row["aged_modeled_covariance"], dtype=float
            )
            expected["offline_aged_radius_containment"] = bool(
                error <= row["aged_modeled_radius"]
            )
            expected["offline_aged_q_error"] = float(
                residual @ np.linalg.solve(covariance, residual)
            )
    if any(row[field] != expected[field] for field in offline_fields):
        raise ValueError("offline metrics differ from independent reconstruction")


def validate_and_reconstruct_row(
    row: Mapping,
    *,
    expected_key: tuple[str, int, int, int],
    protocol: Mapping,
    truth_position: object,
    current_public: dict[int, dict],
    previous_private: dict | None,
    held_command: object,
) -> dict:
    """Validate one exact raw row before independently reconstructing it."""
    _validate_exact_row_schema(row)
    observed_key = (
        row["method"],
        row["seed"],
        row["frame_index"],
        row["robot_id"],
    )
    if observed_key != expected_key:
        raise ValueError("row key differs from expected stream key")
    _validate_topology_fields(row, protocol)
    _validate_reference_runtime_contract(
        row, current_public=current_public
    )
    next_private = _reconstruct_private_state(
        row,
        previous_private=previous_private,
        held_command=held_command,
    )
    branch_reconstruction = _reconstruct_branches(
        row,
        protocol=protocol,
        current_public=current_public,
    )
    _validate_offline_metrics(row, truth_position)
    return {
        "row": dict(row),
        "public_output": _public_output_from_row(row),
        "private_state": next_private,
        "branch_reconstruction": branch_reconstruction,
    }


def _strict_json_object(payload: bytes, path: Path) -> dict:
    def object_pairs(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key in {path}")
            result[key] = value
        return result

    def reject_constant(value):
        raise ValueError(f"non-finite JSON constant: {value}")

    def finite_float(value):
        parsed = float(value)
        if not math.isfinite(parsed):
            raise ValueError(f"non-finite JSON float: {value}")
        return parsed

    try:
        value = json.loads(
            payload,
            object_pairs_hook=object_pairs,
            parse_constant=reject_constant,
            parse_float=finite_float,
        )
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
        raise ValueError(f"invalid strict JSON: {path}") from error
    if not isinstance(value, dict):
        raise ValueError(f"JSON artifact is not an object: {path}")
    return value


def _manifest_identity(identity: Mapping, *, digest: str, domain: str) -> dict:
    allocated_bytes = identity.get("allocated_bytes")
    if allocated_bytes is None:
        metadata = os.stat(identity["path"], follow_symlinks=False)
        observed = (
            metadata.st_dev,
            metadata.st_ino,
            metadata.st_size,
            metadata.st_mtime_ns,
        )
        declared = (
            identity["device"],
            identity["inode"],
            identity["size"],
            identity["mtime_ns"],
        )
        if observed != declared:
            raise ValueError("file identity changed before allocation audit")
        allocated_bytes = metadata.st_blocks * 512
    values = {
        "path": identity["path"],
        "device": identity["device"],
        "inode": identity["inode"],
        "size": identity["size"],
        "allocated_bytes": allocated_bytes,
        "mtime_ns": identity["mtime_ns"],
        "sha256": digest,
        "hash_domain": domain,
    }
    return {
        field: values[field]
        for field in ANALYSIS_MANIFEST_IDENTITY_FIELDS
    }


def _result_identity(identity: Mapping) -> dict:
    return {
        field: identity[field]
        for field in ANALYSIS_IDENTITY_RECORD_FIELDS
    }


def _raw_identity_commitment(identities: Mapping) -> str:
    if not isinstance(identities, Mapping):
        raise ValueError("raw-origin identity set is absent")
    canonical = []
    for name in RAW_ORIGIN_IDENTITY_NAMES:
        identity = identities.get(name)
        if (
            not isinstance(identity, Mapping)
            or tuple(identity) != ANALYSIS_MANIFEST_IDENTITY_FIELDS
        ):
            raise ValueError("raw-origin identity set differs")
        canonical.append(
            [
                name,
                [
                    identity[field]
                    for field in ANALYSIS_MANIFEST_IDENTITY_FIELDS
                ],
            ]
        )
    payload = json.dumps(
        canonical,
        ensure_ascii=True,
        allow_nan=False,
        separators=(",", ":"),
    ).encode("ascii")
    return hashlib.sha256(payload).hexdigest()


def _pinned_file_identity(
    path: Path,
    *,
    domain: str = "file_bytes",
) -> tuple[bytes, dict]:
    payload, observed = replay._read_trusted_bytes(Path(path))
    if payload is None:
        raise RuntimeError("pinned payload was not captured")
    identity = _manifest_identity(
        observed,
        digest=observed["sha256"],
        domain=domain,
    )
    return payload, identity


def _pinned_raw_rows(
    path: Path,
) -> tuple[list[dict], dict, dict]:
    path = Path(path)
    replay._lstat_components(path, leaf_required=True)
    descriptor = os.open(
        path,
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0),
    )
    try:
        before = os.fstat(descriptor)
        compressed = hashlib.sha256()
        offset = 0
        while offset < before.st_size:
            chunk = os.pread(
                descriptor,
                min(1024 * 1024, before.st_size - offset),
                offset,
            )
            if not chunk:
                raise ValueError("raw process short read")
            compressed.update(chunk)
            offset += len(chunk)
        os.lseek(descriptor, 0, os.SEEK_SET)
        decompressed = hashlib.sha256()
        rows = []
        with os.fdopen(os.dup(descriptor), "rb") as raw:
            with gzip.GzipFile(fileobj=raw, mode="rb") as stream:
                for line_number, line in enumerate(stream, 1):
                    decompressed.update(line)
                    if not line.endswith(b"\n"):
                        raise ValueError(
                            f"raw JSONL line {line_number} lacks newline"
                        )
                    rows.append(_strict_json_object(line, path))
        after = os.fstat(descriptor)
        linked = os.stat(path, follow_symlinks=False)
        expected = (
            before.st_dev,
            before.st_ino,
            before.st_size,
            before.st_mtime_ns,
        )
        if (
            (
                after.st_dev,
                after.st_ino,
                after.st_size,
                after.st_mtime_ns,
            )
            != expected
            or (
                linked.st_dev,
                linked.st_ino,
                linked.st_size,
                linked.st_mtime_ns,
            )
            != expected
        ):
            raise ValueError("raw process identity changed while reading")
        base = {
            "path": str(path),
            "device": before.st_dev,
            "inode": before.st_ino,
            "size": before.st_size,
            "allocated_bytes": before.st_blocks * 512,
            "mtime_ns": before.st_mtime_ns,
        }
        compressed_identity = _manifest_identity(
            base,
            digest=compressed.hexdigest(),
            domain="file_bytes",
        )
        decompressed_identity = _manifest_identity(
            base,
            digest=decompressed.hexdigest(),
            domain="decompressed_jsonl_bytes",
        )
        return rows, compressed_identity, decompressed_identity
    finally:
        os.close(descriptor)


def _observed_raw_failure_identities(
    path: Path,
) -> tuple[dict, dict, BaseException | None]:
    """Capture the exact file and the actually readable gzip prefix."""
    path = Path(path)
    replay._lstat_components(path, leaf_required=True)
    descriptor = os.open(
        path,
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0),
    )
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
                raise ValueError("raw process short read during observation")
            compressed_digest.update(chunk)
            offset += len(chunk)
        os.lseek(descriptor, 0, os.SEEK_SET)
        decompressed_digest = hashlib.sha256()
        decompression_error = None
        with os.fdopen(os.dup(descriptor), "rb") as raw:
            with gzip.GzipFile(fileobj=raw, mode="rb") as stream:
                while True:
                    try:
                        chunk = stream.read(1024 * 1024)
                    except BaseException as error:
                        decompression_error = error
                        break
                    if not chunk:
                        break
                    decompressed_digest.update(chunk)
        after = os.fstat(descriptor)
        linked = os.stat(path, follow_symlinks=False)
        expected = (
            before.st_dev,
            before.st_ino,
            before.st_size,
            before.st_mtime_ns,
        )
        if (
            (
                after.st_dev,
                after.st_ino,
                after.st_size,
                after.st_mtime_ns,
            )
            != expected
            or (
                linked.st_dev,
                linked.st_ino,
                linked.st_size,
                linked.st_mtime_ns,
            )
            != expected
        ):
            raise ValueError(
                "raw process identity changed during failure observation"
            )
        base = {
            "path": str(path),
            "device": before.st_dev,
            "inode": before.st_ino,
            "size": before.st_size,
            "allocated_bytes": before.st_blocks * 512,
            "mtime_ns": before.st_mtime_ns,
        }
        return (
            _manifest_identity(
                base,
                digest=compressed_digest.hexdigest(),
                domain="file_bytes",
            ),
            _manifest_identity(
                base,
                digest=decompressed_digest.hexdigest(),
                domain="decompressed_jsonl_bytes",
            ),
            decompression_error,
        )
    finally:
        os.close(descriptor)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _canonical_error_message(
    detail: object,
    *,
    contexts: Iterable[object] = (),
) -> str:
    detail_raw = str(detail).encode(
        "utf-8", errors="backslashreplace"
    )
    context_values = tuple(str(context) for context in contexts)
    context_raw = (
        b""
        if not context_values
        else (
            "; context="
            + " | ".join(context_values)
        ).encode("utf-8", errors="backslashreplace")
    )
    combined = detail_raw + context_raw
    if len(combined) <= ERROR_MESSAGE_MAX_UTF8_BYTES:
        return combined.decode("utf-8")
    digest = hashlib.sha256(detail_raw).hexdigest()
    suffix = (
        " ... [detail_truncated=true "
        f"detail_utf8_bytes={len(detail_raw)} "
        f"detail_sha256={digest}]"
    ).encode("ascii")
    context_budget = max(
        0,
        ERROR_MESSAGE_MAX_UTF8_BYTES - len(suffix),
    )
    bounded_context = context_raw[:context_budget]
    if len(bounded_context) < len(context_raw):
        context_digest = hashlib.sha256(context_raw).hexdigest()
        context_suffix = (
            " ... [context_truncated=true "
            f"context_utf8_bytes={len(context_raw)} "
            f"context_sha256={context_digest}]"
        ).encode("ascii")
        context_budget = max(
            0,
            ERROR_MESSAGE_MAX_UTF8_BYTES
            - len(suffix)
            - len(context_suffix),
        )
        bounded_context = (
            context_raw[:context_budget] + context_suffix
        )
    prefix_budget = max(
        0,
        ERROR_MESSAGE_MAX_UTF8_BYTES
        - len(bounded_context)
        - len(suffix),
    )
    prefix = detail_raw[:prefix_budget].decode(
        "utf-8", errors="ignore"
    )
    return (
        prefix
        + bounded_context.decode("utf-8", errors="ignore")
        + suffix.decode("ascii")
    )


def _analysis_manifest(
    *,
    protocol_id: str,
    invocation_name: str,
    status: str,
    output_root: Path,
    protocol_identity: Mapping,
    authorization_identity: Mapping | None,
    source_identities: Mapping,
    output_identities: Mapping,
    expected_rows: int,
    observed_rows: int,
    disk_contract: Mapping,
    started_at: str,
    completed_at: str | None,
    error: Mapping | None,
) -> dict:
    return {
        "schema_id": ANALYSIS_SCHEMA_ID,
        "protocol_id": protocol_id,
        "invocation_name": invocation_name,
        "status": status,
        "method": replay.METHOD_ID,
        "output_root": str(output_root),
        "protocol_identity": dict(protocol_identity),
        "authorization_identity": (
            None
            if authorization_identity is None
            else dict(authorization_identity)
        ),
        "source_identities": {
            name: dict(source_identities[name])
            for name in ANALYSIS_SOURCE_MEMBER_NAMES[invocation_name]
        },
        "output_identities": {
            name: (
                None
                if output_identities[name] is None
                else dict(output_identities[name])
            )
            for name in ANALYSIS_OUTPUT_MEMBER_NAMES
        },
        "expected_rows": expected_rows,
        "observed_rows": observed_rows,
        "disk_contract": dict(disk_contract),
        "started_at": started_at,
        "completed_at": completed_at,
        "error": None if error is None else dict(error),
    }


def _valid_nonnegative_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _production_compact_cap(disk_contract: object) -> int:
    if (
        not isinstance(disk_contract, Mapping)
        or disk_contract.get("compact_bundle_max_allocated_bytes")
        != COMPACT_OUTPUT_CAP_BYTES
    ):
        raise ValueError(
            "compact allocated-byte cap differs from frozen production cap"
        )
    return COMPACT_OUTPUT_CAP_BYTES


def _lower_hex(value: object, length: int) -> bool:
    return (
        isinstance(value, str)
        and len(value) == length
        and all(character in "0123456789abcdef" for character in value)
    )


def _validate_manifest_identity(
    identity: object,
    *,
    domain: str,
) -> None:
    if (
        not isinstance(identity, Mapping)
        or tuple(identity) != ANALYSIS_MANIFEST_IDENTITY_FIELDS
        or not isinstance(identity["path"], str)
        or not Path(identity["path"]).is_absolute()
        or any(
            not _valid_nonnegative_int(identity[field])
            for field in (
                "device",
                "inode",
                "size",
                "allocated_bytes",
                "mtime_ns",
            )
        )
        or not _lower_hex(identity["sha256"], 64)
        or identity["hash_domain"] != domain
    ):
        raise ValueError("analysis identity differs from exact contract")


def _identity_path_has_role(identity: Mapping, name: str) -> bool:
    suffix = IDENTITY_PATH_SUFFIXES[name]
    parts = Path(identity["path"]).parts
    return len(parts) >= len(suffix) and tuple(parts[-len(suffix):]) == suffix


def _validate_analysis_manifest(
    manifest: Mapping,
    *,
    expected_protocol_id: str | None = None,
    expected_disk_contract: Mapping | None = None,
) -> None:
    if (
        not isinstance(manifest, Mapping)
        or tuple(manifest) != ANALYSIS_MANIFEST_FIELDS
    ):
        raise ValueError("analysis manifest differs from exact field order")
    invocation = manifest["invocation_name"]
    if (
        manifest["schema_id"] != ANALYSIS_SCHEMA_ID
        or invocation not in ANALYZER_INVOCATIONS
        or manifest["status"]
        not in {"creating", "running", "completed", "failed"}
        or manifest["method"] != replay.METHOD_ID
        or not isinstance(manifest["protocol_id"], str)
        or not manifest["protocol_id"]
        or not isinstance(manifest["output_root"], str)
        or not Path(manifest["output_root"]).is_absolute()
    ):
        raise ValueError("analysis manifest scalar discriminant differs")
    if (
        expected_protocol_id is not None
        and manifest["protocol_id"] != expected_protocol_id
    ):
        raise ValueError("analysis manifest protocol ID differs")
    _validate_manifest_identity(
        manifest["protocol_identity"], domain="file_bytes"
    )
    authorization = manifest["authorization_identity"]
    if invocation == "registered_analyzer":
        _validate_manifest_identity(authorization, domain="file_bytes")
        if (
            not _identity_path_has_role(
                manifest["protocol_identity"], "protocol"
            )
            or not _identity_path_has_role(
                authorization, "authorization"
            )
        ):
            raise ValueError("analysis registered identity is substituted")
    elif authorization is not None:
        raise ValueError("smoke analysis carries authorization identity")
    sources = manifest["source_identities"]
    if (
        not isinstance(sources, Mapping)
        or tuple(sources) != ANALYSIS_SOURCE_MEMBER_NAMES[invocation]
    ):
        raise ValueError("analysis source members differ from invocation")
    for name, identity in sources.items():
        expected_domain = (
            "decompressed_jsonl_bytes"
            if name.endswith("decompressed_process")
            else "file_bytes"
        )
        _validate_manifest_identity(identity, domain=expected_domain)
    for name, identity in sources.items():
        if not _identity_path_has_role(identity, name):
            raise ValueError("analysis source member is substituted")
    for manifest_name, sibling_names in (
        (
            "raw_manifest",
            ("raw_compressed_process", "raw_decompressed_process"),
        ),
        (
            "v4_manifest",
            ("v4_compressed_process", "v4_decompressed_process"),
        ),
        (
            "v4_analysis_manifest",
            ("v4_analysis_json", "v4_analysis_markdown"),
        ),
    ):
        if manifest_name not in sources:
            continue
        parent = Path(sources[manifest_name]["path"]).parent
        if any(
            Path(sources[name]["path"]).parent != parent
            for name in sibling_names
        ):
            raise ValueError("analysis source member is substituted")
    for compressed, decompressed in (
        ("raw_compressed_process", "raw_decompressed_process"),
        ("v4_compressed_process", "v4_decompressed_process"),
    ):
        if compressed not in sources:
            continue
        left = sources[compressed]
        right = sources[decompressed]
        if any(
            left[field] != right[field]
            for field in (
                "path",
                "device",
                "inode",
                "size",
                "allocated_bytes",
                "mtime_ns",
            )
        ):
            raise ValueError("compressed/decompressed descriptor differs")
    outputs = manifest["output_identities"]
    if (
        not isinstance(outputs, Mapping)
        or tuple(outputs) != ANALYSIS_OUTPUT_MEMBER_NAMES
    ):
        raise ValueError("analysis output members differ")
    completed = manifest["status"] == "completed"
    output_paths = {
        "analysis_json": str(
            Path(manifest["output_root"]) / OUTPUT_JSON_NAME
        ),
        "analysis_markdown": str(
            Path(manifest["output_root"]) / OUTPUT_MARKDOWN_NAME
        ),
    }
    for name, identity in outputs.items():
        if completed:
            _validate_manifest_identity(identity, domain="file_bytes")
            if identity["path"] != output_paths[name]:
                raise ValueError("analysis output identity is substituted")
        elif identity is not None:
            raise ValueError("preterminal/failed analysis carries output identity")
    expected = 140000 if invocation == "registered_analyzer" else 18
    if (
        manifest["expected_rows"] != expected
        or not _valid_nonnegative_int(manifest["observed_rows"])
        or manifest["observed_rows"] > expected
        or (completed and manifest["observed_rows"] != expected)
    ):
        raise ValueError("analysis manifest row counts differ")
    disk_contract = manifest["disk_contract"]
    if (
        not isinstance(disk_contract, Mapping)
        or tuple(disk_contract)
        != (
            "launch_minimum_free_bytes",
            "live_minimum_free_bytes",
            "raw_bundle_max_allocated_bytes",
            "compact_bundle_max_allocated_bytes",
        )
        or any(
            not _valid_nonnegative_int(value)
            for value in disk_contract.values()
        )
        or (
            expected_disk_contract is not None
            and disk_contract != expected_disk_contract
        )
    ):
        raise ValueError("analysis manifest disk contract differs")
    _production_compact_cap(disk_contract)
    for field in ("started_at", "completed_at"):
        value = manifest[field]
        if field == "completed_at" and value is None:
            continue
        if not isinstance(value, str):
            raise ValueError("analysis manifest timestamp differs")
        parsed = datetime.fromisoformat(value)
        if parsed.tzinfo != timezone.utc or parsed.isoformat() != value:
            raise ValueError("analysis manifest timestamp is not UTC")
    terminal = manifest["status"] in {"completed", "failed"}
    if terminal != (manifest["completed_at"] is not None):
        raise ValueError("analysis terminal timestamp null rule differs")
    error = manifest["error"]
    if manifest["status"] == "failed":
        if (
            not isinstance(error, Mapping)
            or tuple(error) != ANALYSIS_ERROR_FIELDS
            or not all(isinstance(value, str) for value in error.values())
        ):
            raise ValueError("failed analysis lacks canonical error")
    elif error is not None:
        raise ValueError("nonfailed analysis carries error")


def _write_all(descriptor: int, payload: bytes) -> None:
    view = memoryview(payload)
    while view:
        written = os.write(descriptor, view)
        if written <= 0:
            raise OSError("short analysis write")
        view = view[written:]


class _OutputRootIdentityMismatch(ValueError):
    pass


def _assert_output_root_path(
    transaction: Mapping,
    boundary: str,
) -> None:
    try:
        _assert_registered_root(transaction)
    except BaseException as error:
        raise _OutputRootIdentityMismatch(
            f"output root path identity differs at {boundary}"
        ) from error


def _publish_analysis_manifest(
    transaction: Mapping,
    manifest: Mapping,
    *,
    require_path_binding: bool = True,
) -> None:
    _validate_analysis_manifest(manifest)
    payload = (
        replay.ordered_strict_json_bytes(
            manifest, ANALYSIS_MANIFEST_FIELDS
        )
        + b"\n"
    )
    stage = None
    status = manifest["status"]
    try:
        stage = _stage_output(
            transaction,
            final_name=ANALYZER_MANIFEST_NAME,
            payload=payload,
        )
        if require_path_binding:
            _assert_output_root_path(
                transaction, f"{status}_before"
            )
        _publish_staged_output(transaction, stage)
        _verify_staged_output(transaction, stage)
        os.fsync(transaction["parent_fd"])
        if require_path_binding:
            _assert_output_root_path(
                transaction, f"{status}_after"
            )
    except BaseException as error:
        if stage is not None:
            _cleanup_staged_output(transaction, stage, error)
        raise
    else:
        _close_staged_output(transaction, stage)


def _analysis_preallocation_failure_path(output_root: Path) -> Path:
    output_root = Path(output_root)
    digest = hashlib.sha256(
        str(output_root).encode("utf-8")
    ).hexdigest()[:16]
    return output_root.parent / (
        f".{output_root.name}.{digest}.analysis.manifest.failed.json"
    )


def _publish_analysis_preallocation_failure(
    output_root: Path,
    manifest: Mapping,
) -> Path:
    _validate_analysis_manifest(manifest)
    target = _analysis_preallocation_failure_path(output_root)
    replay._lstat_components(target.parent, leaf_required=False)
    payload = (
        replay.ordered_strict_json_bytes(
            manifest, ANALYSIS_MANIFEST_FIELDS
        )
        + b"\n"
    )
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
    allocated = target.stat().st_blocks * 512
    compact_cap = _production_compact_cap(manifest["disk_contract"])
    if allocated > compact_cap:
        target.unlink()
        raise DiskSpaceError(
            "preallocation failure manifest exceeds compact cap"
        )
    return target


def _audit_failed_bundle_cap(
    transaction: Mapping,
    disk_contract: Mapping,
) -> int:
    total = 0
    for name in os.listdir(transaction["root_fd"]):
        metadata = os.stat(
            name,
            dir_fd=transaction["root_fd"],
            follow_symlinks=False,
        )
        total += metadata.st_blocks * 512
    compact_cap = _production_compact_cap(disk_contract)
    if total > compact_cap:
        raise DiskSpaceError(
            "failed forensic bundle exceeds compact allocated-byte cap"
        )
    return total


def _semantic_projection(result: Mapping) -> dict:
    values = {
        "schema_id": result["schema_id"],
        "protocol_id": result["protocol_id"],
        "decision": result["decision"],
        "expected_rows": result["budgets"]["expected_rows"],
        "observed_rows": result["budgets"]["observed_rows"],
        "unique_rows": result["budgets"]["unique_rows"],
        "status_counts": result["status_counts"],
        "selector_accounting": result["selector_accounting"],
        "baseline_fresh_transitions": result[
            "baseline_fresh_transitions"
        ],
        "v4_descriptive_comparison": result[
            "v4_descriptive_comparison"
        ],
        "paired_comparison": result["paired_comparison"],
        "scientific_gates": result["scientific_gates"],
        "integrity_gates": result["integrity_gates"],
        "tails": result["tails"],
        "limitations": result["limitations"],
    }
    return {field: values[field] for field in SMOKE_SEMANTIC_FIELDS}


def _semantic_sha256(result: Mapping) -> str:
    payload = replay.ordered_strict_json_bytes(
        _semantic_projection(result), SMOKE_SEMANTIC_FIELDS
    )
    return hashlib.sha256(payload).hexdigest()


def _validate_result_identity(
    identity: object,
    *,
    domain: str,
) -> None:
    if (
        not isinstance(identity, Mapping)
        or tuple(identity) != ANALYSIS_IDENTITY_RECORD_FIELDS
        or not isinstance(identity["path"], str)
        or not Path(identity["path"]).is_absolute()
        or any(
            not _valid_nonnegative_int(identity[field])
            for field in ("device", "inode", "size", "mtime_ns")
        )
        or not _lower_hex(identity["sha256"], 64)
        or identity["hash_domain"] != domain
    ):
        raise ValueError("compact identity differs from exact contract")


def _validate_gate(record: object, gate_id: str) -> None:
    if (
        not isinstance(record, Mapping)
        or tuple(record) != GATE_RECORD_FIELDS
        or record["gate_id"] != gate_id
        or not isinstance(record["operator"], str)
        or not isinstance(record["passed"], bool)
    ):
        raise ValueError("gate record differs from exact schema")
    numerator = record["numerator"]
    denominator = record["denominator"]
    if (numerator is None) != (denominator is None) or (
        numerator is not None
        and (
            not _valid_nonnegative_int(numerator)
            or not _valid_nonnegative_int(denominator)
        )
    ):
        raise ValueError("gate numerator/denominator differ")
    for field in ("threshold", "value"):
        value = record[field]
        if value is not None and (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
        ):
            raise ValueError("gate numeric value differs")


def _gate_comparison(
    operator: str,
    value: int | float | None,
    threshold: int | float,
) -> bool:
    if value is None:
        return False
    if operator == "strictly_below":
        return value < threshold
    if operator == "less_than_or_equal":
        return value <= threshold
    if operator == "greater_than_or_equal":
        return value >= threshold
    if operator == "equal":
        return value == threshold
    raise ValueError("gate operator is not canonical")


def _expected_scientific_gates_from_sources(
    result: Mapping,
) -> list[dict]:
    budgets = result["budgets"]
    status = result["status_counts"]
    accounting = result["selector_accounting"]
    transitions = result["baseline_fresh_transitions"]
    paired = result["paired_comparison"]
    integrity = {
        record["gate_id"]: record["numerator"]
        for record in result["integrity_gates"]
    }
    expected_rows = budgets["expected_rows"]
    if (
        accounting["accepted"] > status["attempt_accepted"]
        or accounting["accepted"] > status["output_fresh"]
    ):
        raise ValueError("selector accounting exceeds global gate source")
    fresh_count = status["output_fresh"]
    maximum_fresh = result["scientific_gates"][1]["value"]
    predicted = status["output_predicted"]
    if integrity["predicted_selector_output"] > predicted:
        raise ValueError("selector predictions exceed global predictions")
    published_count = fresh_count + predicted
    maximum_published = result["scientific_gates"][0]["value"]
    maximum_age = result["scientific_gates"][5]["value"]
    paired_difference = paired["new_minus_baseline_p95_m"]
    drop_fraction = (
        None
        if transitions["baseline_fresh_total"] == 0
        else max(
            0.0,
            (
                transitions["baseline_fresh_total"]
                - transitions["new_fresh"]
            )
            / transitions["baseline_fresh_total"],
        )
    )
    available = status["output_fresh"] + predicted
    availability_fraction = (
        None if expected_rows == 0 else available / expected_rows
    )
    sources = (
        (
            "maximum_published_error_m_strictly_below",
            "strictly_below",
            50.0,
            int(
                maximum_published is not None
                and maximum_published >= 50.0
            ),
            published_count,
            maximum_published,
        ),
        (
            "maximum_fresh_error_m_strictly_below",
            "strictly_below",
            50.0,
            int(
                maximum_fresh is not None
                and maximum_fresh >= 50.0
            ),
            fresh_count,
            maximum_fresh,
        ),
        (
            "paired_both_fresh_p95_must_not_worsen",
            "less_than_or_equal",
            0.0,
            paired["cohort_size"],
            paired["cohort_size"],
            paired_difference,
        ),
        (
            "fresh_availability_max_drop_fraction",
            "less_than_or_equal",
            0.02,
            transitions["new_fresh"],
            transitions["baseline_fresh_total"],
            drop_fraction,
        ),
        (
            "fresh_or_predicted_min_fraction",
            "greater_than_or_equal",
            0.95,
            available,
            expected_rows,
            availability_fraction,
        ),
        (
            "maximum_prediction_age_frames",
            "less_than_or_equal",
            2,
            predicted,
            predicted,
            maximum_age,
        ),
        (
            "qualification_anchor_violations_allowed",
            "equal",
            0,
            integrity["nonfresh_anchor_use"],
            expected_rows,
            integrity["nonfresh_anchor_use"],
        ),
        (
            "current_frame_provenance_violations_allowed",
            "equal",
            0,
            integrity["preserved_contract_violation"],
            expected_rows,
            integrity["preserved_contract_violation"],
        ),
        (
            "ascending_dag_violations_allowed",
            "equal",
            0,
            integrity["selector_reference_set_violation"],
            expected_rows,
            integrity["selector_reference_set_violation"],
        ),
    )
    return [
        _gate_record(
            gate_id,
            operator,
            threshold,
            numerator,
            denominator,
            value,
            _gate_comparison(operator, value, threshold),
        )
        for (
            gate_id,
            operator,
            threshold,
            numerator,
            denominator,
            value,
        ) in sources
    ]


def _validate_gate_arithmetic(record: Mapping) -> None:
    gate_id = record["gate_id"]
    numerator = record["numerator"]
    denominator = record["denominator"]
    threshold = record["threshold"]
    value = record["value"]
    if (
        numerator is None
        or denominator is None
        or threshold is None
        or numerator > denominator
    ):
        raise ValueError("gate arithmetic is incomplete")
    equal_count_gates = {
        *INTEGRITY_GATE_IDS,
        "qualification_anchor_violations_allowed",
        "current_frame_provenance_violations_allowed",
        "ascending_dag_violations_allowed",
    }
    if gate_id in equal_count_gates:
        expected_value = numerator
    elif gate_id == "fresh_availability_max_drop_fraction":
        expected_value = (
            None
            if denominator == 0
            else max(0.0, (denominator - numerator) / denominator)
        )
    elif gate_id == "fresh_or_predicted_min_fraction":
        expected_value = (
            None if denominator == 0 else numerator / denominator
        )
    elif gate_id == "paired_both_fresh_p95_must_not_worsen":
        if numerator != denominator or (
            denominator == 0 and value is not None
        ) or (
            denominator > 0 and value is None
        ):
            raise ValueError("gate arithmetic differs for paired cohort")
        expected_value = value
    elif gate_id == "maximum_prediction_age_frames":
        if (
            numerator != denominator
            or isinstance(value, bool)
            or not isinstance(value, int)
            or value < 0
        ):
            raise ValueError("gate arithmetic differs for prediction age")
        expected_value = value
    elif gate_id in {
        "maximum_published_error_m_strictly_below",
        "maximum_fresh_error_m_strictly_below",
    }:
        if (
            (denominator == 0 and (numerator != 0 or value is not None))
            or (denominator > 0 and value is None)
            or (
                denominator > 0
                and ((numerator == 0) != (value < threshold))
            )
        ):
            raise ValueError("gate arithmetic differs for maximum error")
        expected_value = value
    else:
        raise ValueError("gate arithmetic has unknown gate ID")
    if value != expected_value:
        raise ValueError("gate arithmetic value differs")
    expected_passed = _gate_comparison(
        record["operator"], expected_value, threshold
    )
    if record["passed"] is not expected_passed:
        raise ValueError("gate result differs from arithmetic")


def _validate_tail(record: object, *, v4: bool) -> None:
    allowed_stratifiers = (
        V4_TAIL_STRATIFIERS if v4 else TAIL_STRATIFIERS
    )
    expected_population = "v4_fresh" if v4 else "two_range_accepted_fresh"
    if (
        not isinstance(record, Mapping)
        or tuple(record) != TAIL_RECORD_FIELDS
        or record["metric"] not in TAIL_METRICS
        or record["stratifier"] not in allowed_stratifiers
        or record["population"] != expected_population
        or not _valid_nonnegative_int(record["count"])
    ):
        raise ValueError("tail record differs from exact schema")
    stratum = record["stratum"]
    if record["stratifier"] == "time_bin":
        if stratum not in {f"{start}-{end}" for start, end in TIME_BINS}:
            raise ValueError("tail time-bin stratum differs")
    elif not _valid_nonnegative_int(stratum):
        raise ValueError("tail integer stratum differs")
    statistics = [
        record[field] for field in ("minimum", "p50", "p95", "maximum")
    ]
    if record["count"] == 0:
        if any(value is not None for value in statistics):
            raise ValueError("empty tail carries statistics")
    elif (
        any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or value < 0
            for value in statistics
        )
        or statistics != sorted(statistics)
    ):
        raise ValueError("positive tail statistics differ")


def _expected_tail_keys(*, v4: bool) -> tuple[tuple[object, ...], ...]:
    strata = {
        "depth": tuple(range(1, 8)),
        "seed": tuple(range(20260727, 20260747)),
        "robot": tuple(range(1, 15)),
        "time_bin": tuple(
            f"{start}-{end}" for start, end in TIME_BINS
        ),
        "private_age": tuple(range(500)),
    }
    stratifiers = V4_TAIL_STRATIFIERS if v4 else TAIL_STRATIFIERS
    return tuple(
        (metric, stratifier, stratum)
        for metric in TAIL_METRICS
        for stratifier in stratifiers
        for stratum in strata[stratifier]
    )


def _validate_ordered_tails(tails: object, *, v4: bool) -> None:
    expected = _expected_tail_keys(v4=v4)
    if (
        not isinstance(tails, list)
        or len(tails) != len(expected)
        or tuple(
            (
                record.get("metric"),
                record.get("stratifier"),
                record.get("stratum"),
            )
            if isinstance(record, Mapping)
            else (None, None, None)
            for record in tails
        )
        != expected
    ):
        raise ValueError("tail order/cardinality differs")
    for record in tails:
        _validate_tail(record, v4=v4)


def _validate_tail_partitions(
    tails: list[Mapping],
    *,
    expected_count: int,
    v4: bool,
) -> None:
    stratifiers = V4_TAIL_STRATIFIERS if v4 else TAIL_STRATIFIERS
    for metric in TAIL_METRICS:
        global_extrema = None
        for stratifier in stratifiers:
            partition = [
                record
                for record in tails
                if record["metric"] == metric
                and record["stratifier"] == stratifier
            ]
            count = sum(record["count"] for record in partition)
            minima = [
                record["minimum"]
                for record in partition
                if record["minimum"] is not None
            ]
            maxima = [
                record["maximum"]
                for record in partition
                if record["maximum"] is not None
            ]
            extrema = (
                (None, None)
                if not minima
                else (min(minima), max(maxima))
            )
            if (
                count != expected_count
                or (count == 0) != (extrema == (None, None))
            ):
                raise ValueError(
                    "tail partition count differs from population"
                )
            if global_extrema is None:
                global_extrema = extrema
            elif extrema != global_extrema:
                raise ValueError(
                    "tail partition source global extrema differ"
                )


def _decode_source_projection(
    result: Mapping,
    source_binding: object,
    *,
    invocation_name: str,
    protocol_id: str,
    protocol_identity_commitment: str | None,
    raw_identity_commitment: str | None,
) -> dict:
    source_projection = _claim_raw_origin_binding(
        source_binding,
        result=result,
        invocation_name=invocation_name,
        protocol_id=protocol_id,
        protocol_identity_commitment=protocol_identity_commitment,
        raw_identity_commitment=raw_identity_commitment,
    )
    projection = _strict_json_object(
        source_projection,
        Path("/raw-derived-source-projection"),
    )
    if (
        tuple(projection) != SOURCE_PROJECTION_FIELDS
        or replay.ordered_strict_json_bytes(
            projection, SOURCE_PROJECTION_FIELDS
        )
        != source_projection
        or projection["invocation_name"] not in ANALYZER_INVOCATIONS
        or any(
            not _valid_nonnegative_int(projection[field])
            for field in (
                "expected_rows",
                "observed_rows",
                "unique_rows",
                "maximum_prediction_age_frames",
            )
        )
        or not isinstance(projection["integrity_counts"], Mapping)
        or tuple(projection["integrity_counts"]) != INTEGRITY_GATE_IDS
        or any(
            not _valid_nonnegative_int(value)
            for value in projection["integrity_counts"].values()
        )
    ):
        raise ValueError(
            "raw-derived source projection differs from canonical schema"
        )
    for field in (
        "maximum_published_error_m",
        "maximum_fresh_error_m",
    ):
        value = projection[field]
        if value is not None and (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or value < 0
        ):
            raise ValueError(
                "raw-derived source projection maximum differs"
            )
    return projection


def _validate_source_projection_binding(
    result: Mapping,
    source_binding: object,
    *,
    expected_invocation_name: str | None = None,
    expected_protocol_id: str | None = None,
    protocol_identity_commitment: str | None = None,
    raw_identity_commitment: str | None = None,
) -> None:
    invocation_name = (
        result["invocation_name"]
        if expected_invocation_name is None
        else expected_invocation_name
    )
    protocol_id = (
        result["protocol_id"]
        if expected_protocol_id is None
        else expected_protocol_id
    )
    if (
        result["invocation_name"] != invocation_name
        or result["protocol_id"] != protocol_id
    ):
        _revoke_raw_origin_binding(source_binding)
        raise ValueError(
            "compact invocation/protocol differs from raw-origin context"
        )
    registered = result["invocation_name"] == "registered_analyzer"
    if registered and (
        protocol_identity_commitment is None
        or raw_identity_commitment is None
    ):
        _revoke_raw_origin_binding(source_binding)
        raise ValueError(
            "registered raw-origin binding requires pinned protocol and "
            "raw identity commitments"
        )
    projection = _decode_source_projection(
        result,
        source_binding,
        invocation_name=invocation_name,
        protocol_id=protocol_id,
        protocol_identity_commitment=protocol_identity_commitment,
        raw_identity_commitment=raw_identity_commitment,
    )
    v4 = result["v4_descriptive_comparison"]
    observed_compact = {
        "invocation_name": result["invocation_name"],
        "expected_rows": result["budgets"]["expected_rows"],
        "observed_rows": result["budgets"]["observed_rows"],
        "unique_rows": result["budgets"]["unique_rows"],
        "status_counts": result["status_counts"],
        "selector_accounting": result["selector_accounting"],
        "baseline_fresh_transitions": result[
            "baseline_fresh_transitions"
        ],
        "v4_fresh_transitions": (
            v4["fresh_transitions"] if registered else None
        ),
        "paired_comparison": result["paired_comparison"],
        "v4_paired_comparison": (
            v4["paired_both_fresh"] if registered else None
        ),
        "maximum_published_error_m": (
            result["scientific_gates"][0]["value"]
            if registered
            else projection["maximum_published_error_m"]
        ),
        "maximum_fresh_error_m": (
            result["scientific_gates"][1]["value"]
            if registered
            else projection["maximum_fresh_error_m"]
        ),
        "maximum_prediction_age_frames": (
            result["scientific_gates"][5]["value"]
            if registered
            else projection["maximum_prediction_age_frames"]
        ),
        "integrity_counts": {
            record["gate_id"]: record["numerator"]
            for record in result["integrity_gates"]
        },
        "scientific_gates": result["scientific_gates"],
        "integrity_gates": result["integrity_gates"],
    }
    for field in SOURCE_PROJECTION_FIELDS:
        if observed_compact[field] != projection[field]:
            raise ValueError(
                "compact result differs from raw-derived source "
                f"projection at {field}"
            )


def _validate_analysis_result(
    result: Mapping,
    *,
    source_projection: object = None,
    expected_invocation_name: str | None = None,
    expected_protocol_id: str | None = None,
    protocol_identity_commitment: str | None = None,
    raw_identity_commitment: str | None = None,
) -> None:
    if (
        not isinstance(result, Mapping)
        or tuple(result) != ANALYSIS_FIELDS
        or result["schema_id"] != ANALYSIS_SCHEMA_ID
        or result["invocation_name"] not in ANALYZER_INVOCATIONS
        or not isinstance(result["protocol_id"], str)
        or not result["protocol_id"]
        or not _lower_hex(result["semantic_payload_sha256"], 64)
    ):
        raise ValueError("compact result differs from exact top-level schema")
    invocation = result["invocation_name"]
    registered = invocation == "registered_analyzer"
    if result["decision"] not in (
        {"pass", "fail"} if registered else {"smoke_pass", "smoke_fail"}
    ):
        raise ValueError("compact decision differs from invocation")
    identities = result["identities"]
    if not isinstance(identities, Mapping) or tuple(identities) != IDENTITY_FIELDS:
        raise ValueError("compact identities differ from exact order")
    smoke_nonnull = {
        "protocol",
        "mechanism_fixture",
        "synthetic_case_source",
        "raw_manifest",
        "raw_compressed_process",
        "raw_decompressed_process",
    }
    registered_null = {"mechanism_fixture", "synthetic_case_source"}
    for name, identity in identities.items():
        required = (
            name not in registered_null
            if registered
            else name in smoke_nonnull
        )
        if not required:
            if identity is not None:
                raise ValueError("inapplicable compact identity is non-null")
            continue
        domain = (
            "decompressed_jsonl_bytes"
            if name.endswith("decompressed_process")
            else "file_bytes"
        )
        _validate_result_identity(identity, domain=domain)
        if (
            name not in {"protocol", "authorization"}
            or registered
        ) and not _identity_path_has_role(identity, name):
            raise ValueError("compact identity role is substituted")
    for compressed, decompressed in (
        ("raw_compressed_process", "raw_decompressed_process"),
        ("v4_compressed_process", "v4_decompressed_process"),
    ):
        left = identities[compressed]
        right = identities[decompressed]
        if left is None:
            continue
        if any(
            left[field] != right[field]
            for field in ("path", "device", "inode", "size", "mtime_ns")
        ):
            raise ValueError("compact compressed/decompressed descriptor differs")
    for manifest_name, sibling_names in (
        (
            "raw_manifest",
            ("raw_compressed_process", "raw_decompressed_process"),
        ),
        (
            "v4_manifest",
            ("v4_compressed_process", "v4_decompressed_process"),
        ),
        (
            "v4_analysis_manifest",
            ("v4_analysis_json", "v4_analysis_markdown"),
        ),
    ):
        manifest_identity = identities[manifest_name]
        if manifest_identity is None:
            continue
        parent = Path(manifest_identity["path"]).parent
        if any(
            Path(identities[name]["path"]).parent != parent
            for name in sibling_names
        ):
            raise ValueError("compact identity role is substituted")
    budgets = result["budgets"]
    if (
        not isinstance(budgets, Mapping)
        or tuple(budgets) != BUDGET_FIELDS
        or any(
            not _valid_nonnegative_int(value)
            for value in budgets.values()
        )
        or budgets["expected_rows"] != (140000 if registered else 18)
        or budgets["observed_rows"] != budgets["expected_rows"]
        or budgets["unique_rows"] != budgets["expected_rows"]
        or budgets["compact_allocated_bytes"] > COMPACT_OUTPUT_CAP_BYTES
    ):
        raise ValueError("compact budgets differ")
    status = result["status_counts"]
    if (
        not isinstance(status, Mapping)
        or tuple(status) != STATUS_COUNT_FIELDS
        or any(not _valid_nonnegative_int(value) for value in status.values())
        or sum(status[field] for field in STATUS_COUNT_FIELDS[:5])
        != budgets["observed_rows"]
        or sum(status[field] for field in STATUS_COUNT_FIELDS[5:])
        != budgets["observed_rows"]
    ):
        raise ValueError("compact status counts differ")
    accounting = result["selector_accounting"]
    if (
        not isinstance(accounting, Mapping)
        or tuple(accounting) != SELECTOR_ACCOUNTING_FIELDS
        or any(
            not _valid_nonnegative_int(accounting[field])
            for field in SELECTOR_ACCOUNTING_FIELDS
            if field != "outage_episode_lengths"
        )
        or not isinstance(accounting["outage_episode_lengths"], list)
        or any(
            not _valid_nonnegative_int(value) or value == 0
            for value in accounting["outage_episode_lengths"]
        )
        or accounting["outage_episode_lengths"]
        != sorted(accounting["outage_episode_lengths"])
        or accounting["outage_episode_count"]
        != len(accounting["outage_episode_lengths"])
        or sum(
            accounting[field]
            for field in (
                "score_exactly_one",
                "score_none",
                "score_multiple",
                "score_not_evaluated",
            )
        )
        != accounting["considered"]
    ):
        raise ValueError("selector accounting differs")
    if (
        accounting["accepted"] > status["attempt_accepted"]
        or accounting["accepted"] > status["output_fresh"]
        or accounting["rejected"] > status["attempt_rejected"]
        or accounting["unavailable"]
        > status["attempt_reference_unavailable"]
        or accounting["root_rejections"]
        > (
            status["attempt_rejected"]
            + status["attempt_failed"]
            + status["attempt_invalid"]
            + status["attempt_reference_unavailable"]
        )
        or accounting["downstream_unavailable"]
        > status["output_unavailable"]
    ):
        raise ValueError(
            "selector accounting exceeds global status populations"
        )
    if registered:
        transitions = result["baseline_fresh_transitions"]
        if (
            not isinstance(transitions, Mapping)
            or tuple(transitions) != BASELINE_FRESH_TRANSITION_FIELDS
            or any(
                not _valid_nonnegative_int(value)
                for value in transitions.values()
            )
            or transitions["baseline_fresh_total"]
            != transitions["new_fresh"]
            + transitions["new_predicted"]
            + transitions["new_unavailable"]
        ):
            raise ValueError("baseline transition record differs")
        if any(
            transitions[field] > status[status_field]
            for field, status_field in (
                ("new_fresh", "output_fresh"),
                ("new_predicted", "output_predicted"),
                ("new_unavailable", "output_unavailable"),
            )
        ):
            raise ValueError(
                "baseline transition subcount exceeds global population"
            )
        v4 = result["v4_descriptive_comparison"]
        if (
            not isinstance(v4, Mapping)
            or tuple(v4) != V4_DESCRIPTIVE_COMPARISON_FIELDS
        ):
            raise ValueError("v4 descriptive comparison differs")
        fresh_transitions = v4["fresh_transitions"]
        if (
            not isinstance(fresh_transitions, Mapping)
            or tuple(fresh_transitions) != V4_FRESH_TRANSITION_FIELDS
            or any(
                not _valid_nonnegative_int(value)
                for value in fresh_transitions.values()
            )
            or fresh_transitions["v4_fresh_total"]
            != fresh_transitions["new_fresh"]
            + fresh_transitions["new_predicted"]
            + fresh_transitions["new_unavailable"]
        ):
            raise ValueError("v4 fresh transitions differ")
        if any(
            fresh_transitions[field] > status[status_field]
            for field, status_field in (
                ("new_fresh", "output_fresh"),
                ("new_predicted", "output_predicted"),
                ("new_unavailable", "output_unavailable"),
            )
        ):
            raise ValueError(
                "v4 transition subcount exceeds global population"
            )
        for field, fields in (
            ("paired_comparison", PAIRED_COMPARISON_FIELDS),
            ("paired_both_fresh", V4_PAIRED_COMPARISON_FIELDS),
        ):
            record = (
                result[field]
                if field == "paired_comparison"
                else v4[field]
            )
            if not isinstance(record, Mapping) or tuple(record) != fields:
                raise ValueError("paired comparison differs")
            cohort = record["cohort_size"]
            if not _valid_nonnegative_int(cohort):
                raise ValueError("paired cohort differs")
            numeric = [record[name] for name in fields[1:]]
            if cohort == 0:
                if any(value is not None for value in numeric):
                    raise ValueError("empty paired cohort carries statistics")
            elif any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                for value in numeric
            ):
                raise ValueError("paired statistics differ")
            elif (
                record[fields[1]] < 0
                or record[fields[2]] < 0
            ):
                raise ValueError("paired p95 statistic is negative")
            elif (
                record[fields[2]] - record[fields[1]]
                != record[fields[3]]
            ):
                raise ValueError("paired signed difference differs")
        if (
            result["paired_comparison"]["cohort_size"]
            > transitions["baseline_fresh_total"]
            or result["paired_comparison"]["cohort_size"]
            > status["output_fresh"]
            or v4["paired_both_fresh"]["cohort_size"]
            > fresh_transitions["v4_fresh_total"]
            or v4["paired_both_fresh"]["cohort_size"]
            > status["output_fresh"]
        ):
            raise ValueError(
                "paired cohort exceeds baseline/v4 or new-fresh "
                "source population"
            )
        _validate_ordered_tails(v4["tails"], v4=True)
        _validate_tail_partitions(
            v4["tails"],
            expected_count=fresh_transitions["v4_fresh_total"],
            v4=True,
        )
        if (
            accounting["fresh_contained"]
            + accounting["fresh_not_contained"]
            != accounting["accepted"]
        ):
            raise ValueError("fresh containment denominator differs")
    elif (
        result["baseline_fresh_transitions"] is not None
        or result["v4_descriptive_comparison"] is not None
        or result["paired_comparison"] is not None
        or result["scientific_gates"] != []
        or result["tails"] != []
        or accounting["fresh_contained"] != 0
        or accounting["fresh_not_contained"] != 0
    ):
        raise ValueError("smoke result carries registered analysis fields")
    scientific = result["scientific_gates"]
    if registered:
        if (
            not isinstance(scientific, list)
            or len(scientific) != len(GATES)
            or any(not isinstance(record, Mapping) for record in scientific)
            or tuple(record["gate_id"] for record in scientific)
            != tuple(GATES)
        ):
            raise ValueError("scientific gate order differs")
        gate_contracts = (
            ("strictly_below", 50.0),
            ("strictly_below", 50.0),
            ("less_than_or_equal", 0.0),
            ("less_than_or_equal", 0.02),
            ("greater_than_or_equal", 0.95),
            ("less_than_or_equal", 2),
            ("equal", 0),
            ("equal", 0),
            ("equal", 0),
        )
        for record, gate_id, contract in zip(
            scientific, GATES, gate_contracts, strict=True
        ):
            _validate_gate(record, gate_id)
            if (record["operator"], record["threshold"]) != contract:
                raise ValueError("scientific gate contract differs")
            _validate_gate_arithmetic(record)
    integrity = result["integrity_gates"]
    if (
        not isinstance(integrity, list)
        or len(integrity) != len(INTEGRITY_GATE_IDS)
        or any(not isinstance(record, Mapping) for record in integrity)
        or tuple(record["gate_id"] for record in integrity)
        != INTEGRITY_GATE_IDS
    ):
        raise ValueError("integrity gate order differs")
    for record, gate_id in zip(
        integrity, INTEGRITY_GATE_IDS, strict=True
    ):
        _validate_gate(record, gate_id)
        if record["operator"] != "equal" or record["threshold"] != 0:
            raise ValueError("integrity gate contract differs")
        if record["denominator"] != budgets["expected_rows"]:
            raise ValueError(
                "integrity gate denominator differs from source rows"
            )
        _validate_gate_arithmetic(record)
    integrity_by_id = {
        record["gate_id"]: record for record in integrity
    }
    expected_denominator_violation = int(
        budgets["observed_rows"] != budgets["expected_rows"]
        or budgets["unique_rows"] != budgets["expected_rows"]
    )
    if (
        integrity_by_id["exact_denominator_violation"]["numerator"]
        != expected_denominator_violation
    ):
        raise ValueError(
            "exact denominator violation differs from compact budgets"
        )
    if (
        integrity_by_id["predicted_selector_output"]["numerator"]
        > status["output_predicted"]
    ):
        raise ValueError(
            "selector predictions exceed global predicted population"
        )
    tails = result["tails"]
    if registered:
        _validate_ordered_tails(tails, v4=False)
        _validate_tail_partitions(
            tails,
            expected_count=accounting["accepted"],
            v4=False,
        )
        expected_scientific = _expected_scientific_gates_from_sources(
            result
        )
        if scientific != expected_scientific:
            raise ValueError(
                "scientific gate differs from compact source summary"
            )
    expected_decision = (
        ("pass" if all(record["passed"] for record in (*scientific, *integrity))
         else "fail")
        if registered
        else (
            "smoke_pass"
            if all(record["passed"] for record in integrity)
            else "smoke_fail"
        )
    )
    if result["decision"] != expected_decision:
        raise ValueError("decision differs from gate conjunction")
    if result["limitations"] != list(LIMITATIONS):
        raise ValueError("limitations differ from frozen order")
    if result["semantic_payload_sha256"] != _semantic_sha256(result):
        raise ValueError("semantic payload digest differs")
    if source_projection is not None:
        _validate_source_projection_binding(
            result,
            source_projection,
            expected_invocation_name=expected_invocation_name,
            expected_protocol_id=expected_protocol_id,
            protocol_identity_commitment=protocol_identity_commitment,
            raw_identity_commitment=raw_identity_commitment,
        )
    elif registered:
        raise ValueError(
            "registered compact requires a currently minted raw-origin "
            "binding for its raw-derived source projection"
        )


_validate_analysis = _validate_analysis_result


@_with_raw_origin_minter
def _smoke_result_with_projection(
    *,
    invocation_name: str,
    protocol_id: str,
    rows: list[Mapping],
    identities: Mapping,
    raw_allocated_bytes: int,
    protocol_identity_commitment: str | None = None,
    raw_identity_commitment: str | None = None,
    _raw_origin_minter,
) -> tuple[dict, object]:
    attempts = Counter(row["attempt_status"] for row in rows)
    outputs = Counter(row["output_status"] for row in rows)
    status_values = {
        "attempt_accepted": attempts["accepted"],
        "attempt_rejected": attempts["rejected"],
        "attempt_failed": attempts["failed"],
        "attempt_invalid": attempts["invalid"],
        "attempt_reference_unavailable": attempts[
            "reference_unavailable"
        ],
        "output_fresh": outputs["fresh"],
        "output_predicted": outputs["predicted"],
        "output_unavailable": outputs["unavailable"],
    }
    accounting = _selector_accounting(rows)
    accounting["fresh_contained"] = 0
    accounting["fresh_not_contained"] = 0
    integrity_counts = {
        gate_id: 0 for gate_id in INTEGRITY_GATE_IDS
    }
    integrity = _integrity_gate_records(integrity_counts, denominator=18)
    values = {
        "schema_id": ANALYSIS_SCHEMA_ID,
        "protocol_id": protocol_id,
        "invocation_name": invocation_name,
        "decision": (
            "smoke_pass"
            if all(record["passed"] for record in integrity)
            else "smoke_fail"
        ),
        "semantic_payload_sha256": "",
        "identities": {
            field: identities.get(field)
            for field in IDENTITY_FIELDS
        },
        "budgets": {
            "expected_rows": 18,
            "observed_rows": len(rows),
            "unique_rows": len({row["smoke_case_id"] for row in rows}),
            "raw_allocated_bytes": raw_allocated_bytes,
            "compact_allocated_bytes": 0,
        },
        "status_counts": {
            field: status_values[field] for field in STATUS_COUNT_FIELDS
        },
        "selector_accounting": accounting,
        "baseline_fresh_transitions": None,
        "v4_descriptive_comparison": None,
        "paired_comparison": None,
        "scientific_gates": [],
        "integrity_gates": integrity,
        "tails": [],
        "limitations": list(LIMITATIONS),
    }
    result = {field: values[field] for field in ANALYSIS_FIELDS}
    result["semantic_payload_sha256"] = _semantic_sha256(result)
    published_errors = [
        row["offline_error_norm"]
        for row in rows
        if row["output_status"] in {"fresh", "predicted"}
        and row["offline_error_norm"] is not None
    ]
    fresh_errors = [
        row["offline_error_norm"]
        for row in rows
        if row["output_status"] == "fresh"
        and row["offline_error_norm"] is not None
    ]
    projection_values = {
        "invocation_name": invocation_name,
        "expected_rows": 18,
        "observed_rows": len(rows),
        "unique_rows": len({row["smoke_case_id"] for row in rows}),
        "status_counts": {
            field: status_values[field] for field in STATUS_COUNT_FIELDS
        },
        "selector_accounting": accounting,
        "baseline_fresh_transitions": None,
        "v4_fresh_transitions": None,
        "paired_comparison": None,
        "v4_paired_comparison": None,
        "maximum_published_error_m": (
            None
            if not published_errors
            else float(max(published_errors))
        ),
        "maximum_fresh_error_m": (
            None if not fresh_errors else float(max(fresh_errors))
        ),
        "maximum_prediction_age_frames": max(
            (
                row["prediction_age"]
                for row in rows
                if row["output_status"] == "predicted"
            ),
            default=0,
        ),
        "integrity_counts": integrity_counts,
        "scientific_gates": [],
        "integrity_gates": integrity,
    }
    source_projection = replay.ordered_strict_json_bytes(
        {
            field: projection_values[field]
            for field in SOURCE_PROJECTION_FIELDS
        },
        SOURCE_PROJECTION_FIELDS,
    )
    source_binding = _raw_origin_minter(
        source_projection,
        result=result,
        invocation_name=invocation_name,
        protocol_id=protocol_id,
        protocol_identity_commitment=protocol_identity_commitment,
        raw_identity_commitment=raw_identity_commitment,
    )
    return result, source_binding


def _smoke_result(
    *,
    invocation_name: str,
    protocol_id: str,
    rows: list[Mapping],
    identities: Mapping,
    raw_allocated_bytes: int,
) -> dict:
    result, source_binding = _smoke_result_with_projection(
        invocation_name=invocation_name,
        protocol_id=protocol_id,
        rows=rows,
        identities=identities,
        raw_allocated_bytes=raw_allocated_bytes,
    )
    try:
        return result
    finally:
        _revoke_raw_origin_binding(source_binding)


del _with_raw_origin_minter


def _markdown(result: Mapping) -> str:
    return (
        "# Two-range reacquisition analysis\n\n"
        f"- Invocation: `{result['invocation_name']}`\n"
        f"- Decision: `{result['decision']}`\n"
        f"- Rows: {result['budgets']['observed_rows']}/"
        f"{result['budgets']['expected_rows']}\n"
        f"- Semantic payload: `{result['semantic_payload_sha256']}`\n"
    )


def _stage_output(
    transaction: Mapping,
    *,
    final_name: str,
    payload: bytes,
) -> dict:
    temporary = (
        f".{final_name}.{os.getpid()}.{secrets.token_hex(8)}.staged"
    )
    descriptor = None
    try:
        descriptor = os.open(
            temporary,
            os.O_RDWR
            | os.O_CREAT
            | os.O_EXCL
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NOFOLLOW", 0),
            0o600,
            dir_fd=transaction["root_fd"],
        )
        _write_all(descriptor, payload)
        os.fsync(descriptor)
        metadata = os.fstat(descriptor)
        linked = os.stat(
            temporary,
            dir_fd=transaction["root_fd"],
            follow_symlinks=False,
        )
        if (
            not stat.S_ISREG(metadata.st_mode)
            or (metadata.st_dev, metadata.st_ino)
            != (linked.st_dev, linked.st_ino)
        ):
            raise ValueError("staged output identity differs after write")
        identity = {
            "path": str(Path(transaction["output_root"]) / final_name),
            "device": metadata.st_dev,
            "inode": metadata.st_ino,
            "size": metadata.st_size,
            "allocated_bytes": metadata.st_blocks * 512,
            "mtime_ns": metadata.st_mtime_ns,
            "sha256": hashlib.sha256(payload).hexdigest(),
            "hash_domain": "file_bytes",
        }
        stage = {
            "name": temporary,
            "final_name": final_name,
            "fd": descriptor,
            "identity": identity,
            "entry_name": temporary,
            "published": False,
            "closed": False,
        }
        transaction["resource_fds"].add(descriptor)
        return stage
    except BaseException as error:
        if descriptor is not None:
            try:
                os.close(descriptor)
            except BaseException as cleanup_error:
                error.add_note(
                    "staged descriptor cleanup failed: "
                    f"{cleanup_error}"
                )
        try:
            os.unlink(temporary, dir_fd=transaction["root_fd"])
        except FileNotFoundError:
            pass
        except BaseException as cleanup_error:
            error.add_note(
                "hidden staged entry cleanup failed: "
                f"{cleanup_error}"
            )
        raise


def _stage_descriptor_digest(descriptor: int, size: int) -> str:
    digest = hashlib.sha256()
    offset = 0
    while offset < size:
        chunk = os.pread(
            descriptor, min(1024 * 1024, size - offset), offset
        )
        if not chunk:
            raise ValueError("staged output short read")
        digest.update(chunk)
        offset += len(chunk)
    return digest.hexdigest()


def _verify_staged_output(transaction: Mapping, stage: Mapping) -> dict:
    if stage.get("closed") or not isinstance(stage.get("fd"), int):
        raise ValueError("staged output descriptor is not retained")
    metadata = os.fstat(stage["fd"])
    identity = stage["identity"]
    if (
        not stat.S_ISREG(metadata.st_mode)
        or any(
            metadata_value != identity[field]
            for metadata_value, field in (
                (metadata.st_dev, "device"),
                (metadata.st_ino, "inode"),
                (metadata.st_size, "size"),
                (metadata.st_blocks * 512, "allocated_bytes"),
                (metadata.st_mtime_ns, "mtime_ns"),
            )
        )
        or _stage_descriptor_digest(stage["fd"], metadata.st_size)
        != identity["sha256"]
    ):
        raise ValueError("retained staged output identity changed")
    entry_name = stage.get("entry_name")
    if not isinstance(entry_name, str):
        raise ValueError("staged output has no linked directory entry")
    linked = os.stat(
        entry_name,
        dir_fd=transaction["root_fd"],
        follow_symlinks=False,
    )
    if (
        not stat.S_ISREG(linked.st_mode)
        or any(
            linked_value != identity[field]
            for linked_value, field in (
                (linked.st_dev, "device"),
                (linked.st_ino, "inode"),
                (linked.st_size, "size"),
                (linked.st_blocks * 512, "allocated_bytes"),
                (linked.st_mtime_ns, "mtime_ns"),
            )
        )
    ):
        raise ValueError("staged output directory-entry identity changed")
    return dict(identity)


def _close_staged_output(transaction: Mapping, stage: dict) -> None:
    if stage.get("closed"):
        return
    descriptor = stage.get("fd")
    if isinstance(descriptor, int):
        os.close(descriptor)
        transaction["resource_fds"].discard(descriptor)
    stage["fd"] = None
    stage["closed"] = True


def _discard_staged_output(transaction: Mapping, stage: dict) -> None:
    _verify_staged_output(transaction, stage)
    os.unlink(stage["entry_name"], dir_fd=transaction["root_fd"])
    stage["entry_name"] = None
    os.fsync(transaction["root_fd"])
    _close_staged_output(transaction, stage)


def _publish_staged_output(transaction: Mapping, stage: dict) -> dict:
    _verify_staged_output(transaction, stage)
    os.rename(
        stage["entry_name"],
        stage["final_name"],
        src_dir_fd=transaction["root_fd"],
        dst_dir_fd=transaction["root_fd"],
    )
    stage["entry_name"] = stage["final_name"]
    stage["published"] = True
    os.fsync(transaction["root_fd"])
    return _verify_staged_output(transaction, stage)


def _cleanup_staged_output(
    transaction: Mapping,
    stage: dict,
    error: BaseException,
) -> None:
    entry_name = stage.get("entry_name")
    if isinstance(entry_name, str):
        try:
            os.unlink(entry_name, dir_fd=transaction["root_fd"])
        except FileNotFoundError:
            pass
        except BaseException as cleanup_error:
            error.add_note(
                f"staged entry cleanup failed for {entry_name}: "
                f"{cleanup_error}"
            )
        stage["entry_name"] = None
    try:
        _close_staged_output(transaction, stage)
    except BaseException as cleanup_error:
        error.add_note(
            "staged descriptor close failed: "
            f"{cleanup_error}"
        )


def _smoke_context(row: Mapping, fixture: Mapping) -> tuple[dict, object, object]:
    if row["smoke_case_id"] == replay.MECHANISM_FIXTURE_ID:
        current_public = {
            int(record["reference_key"][1]): replay._internal_public(
                record["public_output"]
            )
            for record in fixture["current_reference_outputs"]
        }
        return (
            current_public,
            fixture["preceding_private_state"],
            fixture["held_command"],
        )
    return {}, None, row["applied_command"]


class _SourceIdentityMismatch(ValueError):
    def __init__(self, message: str, observed: Mapping):
        super().__init__(message)
        self.observed = dict(observed)


def _reverify_inputs(expected: Mapping[str, Mapping]) -> None:
    observed = {}
    paired_process_names = []
    for compressed_name in (
        name for name in expected if name.endswith("compressed_process")
    ):
        decompressed_name = compressed_name.replace(
            "compressed_process", "decompressed_process"
        )
        if decompressed_name not in expected:
            continue
        paired_process_names.extend(
            (compressed_name, decompressed_name)
        )
        process_path = Path(expected[compressed_name]["path"])
        try:
            _, compressed, decompressed = _pinned_raw_rows(process_path)
        except BaseException as error:
            (
                compressed,
                decompressed,
                decompression_error,
            ) = _observed_raw_failure_identities(process_path)
            observed.update(
                {
                    compressed_name: compressed,
                    decompressed_name: decompressed,
                }
            )
            label = (
                "raw process"
                if compressed_name == "raw_compressed_process"
                else compressed_name
            )
            suffix = (
                ""
                if decompression_error is None
                else (
                    "; observed decompressed prefix ended with "
                    f"{type(decompression_error).__name__}: "
                    f"{decompression_error}"
                )
            )
            raise _SourceIdentityMismatch(
                f"{label} cannot be reverified: {error}{suffix}",
                observed,
            ) from error
        observed[compressed_name] = compressed
        observed[decompressed_name] = decompressed
    for name, identity in expected.items():
        if name in paired_process_names:
            continue
        _, current = _pinned_file_identity(Path(identity["path"]))
        observed[name] = current
    mismatches = [
        name for name in expected if observed[name] != expected[name]
    ]
    if mismatches:
        raise _SourceIdentityMismatch(
            "analysis input identity changed: " + ", ".join(mismatches),
            observed,
        )


def _validate_smoke_rows(
    rows: list[Mapping],
    *,
    protocol: Mapping,
) -> None:
    if (
        len(rows) != 18
        or tuple(row.get("smoke_case_id") for row in rows)
        != replay.SMOKE_CASE_IDS
    ):
        raise ValueError("smoke rows differ from exact ordered case grid")
    fixture = replay._load_fixture()
    for row in rows:
        validation_protocol = protocol
        if row["smoke_case_id"] == replay.MECHANISM_FIXTURE_ID:
            validation_protocol = {
                **protocol,
                "_expected_mandatory_references": fixture[
                    "mandatory_references"
                ],
                "_expected_squad_local_index": fixture["key"][
                    "squad_local_index"
                ],
            }
        current_public, previous_private, held_command = _smoke_context(
            row, fixture
        )
        validate_and_reconstruct_row(
            row,
            expected_key=(
                row["method"],
                row["seed"],
                row["frame_index"],
                row["robot_id"],
            ),
            protocol=validation_protocol,
            truth_position=row["offline_truth_position"],
            current_public=current_public,
            previous_private=previous_private,
            held_command=held_command,
        )


def _matches_declared_file_identity(
    declared: object,
    observed: Mapping,
) -> bool:
    return (
        isinstance(declared, Mapping)
        and all(field in declared for field in replay.FILE_IDENTITY_FIELDS)
        and all(
            declared[field] == observed[field]
            for field in replay.FILE_IDENTITY_FIELDS
        )
    )


def _validate_registered_authorization(
    *,
    authorization_path: Path,
    protocol_path: Path,
    protocol_payload: bytes,
    protocol: Mapping,
    protocol_identity: Mapping,
    raw_root: Path,
    output_root: Path,
    raw_manifest: Mapping,
) -> tuple[dict, bytes, dict]:
    authorization_payload, authorization_identity = _pinned_file_identity(
        authorization_path
    )
    authorization = _strict_json_object(
        authorization_payload, authorization_path
    )
    authorization_text = authorization.get("user_authorization_text")
    sha_fields = (
        "protocol_sha256",
        "smoke_a_compressed_sha256",
        "smoke_a_decompressed_sha256",
        "smoke_b_compressed_sha256",
        "smoke_b_decompressed_sha256",
        "smoke_analyzer_a_json_sha256",
        "smoke_analyzer_a_markdown_sha256",
        "smoke_analyzer_b_json_sha256",
        "smoke_analyzer_b_markdown_sha256",
        "smoke_semantic_payload_sha256",
        "user_authorization_text_sha256",
    )
    commit_fields = ("protocol_commit", "preflight_commit", "smoke_commit")
    if (
        tuple(authorization) != replay.REGISTERED_AUTHORIZATION_FIELDS
        or authorization["schema_id"]
        != replay.REGISTERED_AUTHORIZATION_SCHEMA_ID
        or authorization["protocol_id"] != protocol["protocol_id"]
        or authorization["protocol_sha256"] != protocol_identity["sha256"]
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
            authorization_text.encode("utf-8")
        ).hexdigest()
        != authorization["user_authorization_text_sha256"]
        or not replay._canonical_iso_date(
            authorization["user_authorization_date"]
        )
        or authorization["registered_replay_root"] != str(raw_root)
        or authorization["registered_analyzer_root"] != str(output_root)
        or authorization["registered_retry_allowed"] is not False
        or not _matches_declared_file_identity(
            raw_manifest["authorization_identity"],
            authorization_identity,
        )
    ):
        raise ValueError(
            "registered authorization differs from exact binding"
        )
    replay._validate_committed_registered_state(
        project_root=Path(__file__).resolve().parents[2],
        protocol_path=protocol_path,
        protocol_payload=protocol_payload,
        protocol=protocol,
        protocol_identity={
            field: protocol_identity[field]
            for field in replay.FILE_IDENTITY_FIELDS
        },
        authorization_path=authorization_path,
        authorization_payload=authorization_payload,
        authorization=authorization,
        authorization_identity={
            field: authorization_identity[field]
            for field in replay.FILE_IDENTITY_FIELDS
        },
        sources=raw_manifest["source_identities"],
    )
    return authorization, authorization_payload, authorization_identity


def _pin_declared_file(
    declared: object,
    *,
    label: str,
) -> tuple[bytes, dict]:
    if not isinstance(declared, Mapping) or not isinstance(
        declared.get("path"), str
    ):
        raise ValueError(f"{label} declaration is absent")
    payload, observed = _pinned_file_identity(Path(declared["path"]))
    if not _matches_declared_file_identity(declared, observed):
        raise ValueError(f"{label} identity differs")
    return payload, observed


def _registered_source_data(
    *,
    protocol: Mapping,
    raw_manifest: Mapping,
) -> tuple[dict, dict, list[dict], list[dict], Mapping]:
    comparators = protocol.get("comparators")
    sources = protocol.get("sources")
    if not isinstance(comparators, Mapping) or not isinstance(
        sources, Mapping
    ):
        raise ValueError("registered comparator/source declarations absent")

    v4_manifest_payload, v4_manifest_identity = _pin_declared_file(
        comparators.get("v4_replay_manifest"),
        label="v4 manifest",
    )
    v4_manifest = _strict_json_object(
        v4_manifest_payload,
        Path(v4_manifest_identity["path"]),
    )
    v4_process_declaration = comparators.get("v4_compressed_process")
    if not isinstance(v4_process_declaration, Mapping):
        raise ValueError("v4 process declaration is absent")
    v4_rows, v4_compressed, v4_decompressed = _pinned_raw_rows(
        Path(v4_process_declaration["path"])
    )
    for name, declared, observed in (
        (
            "v4 compressed process",
            v4_process_declaration,
            v4_compressed,
        ),
        (
            "v4 decompressed process",
            comparators.get("v4_decompressed_process"),
            v4_decompressed,
        ),
    ):
        if not _matches_declared_file_identity(declared, observed):
            raise ValueError(f"{name} identity differs")
    if (
        v4_manifest.get("status") != "completed"
        or v4_manifest.get("compressed_process_sha256")
        != v4_compressed["sha256"]
        or v4_manifest.get("decompressed_process_sha256")
        != v4_decompressed["sha256"]
    ):
        raise ValueError("v4 manifest does not bind the comparator process")

    direct_files = {}
    for source_name, declaration_name in (
        ("v4_analysis_manifest", "v4_analysis_manifest"),
        ("v4_analysis_json", "v4_analysis_json"),
        ("v4_analysis_markdown", "v4_analysis_markdown"),
        ("legacy_baseline_process", "legacy_baseline_process"),
    ):
        payload, identity = _pin_declared_file(
            comparators.get(declaration_name),
            label=source_name,
        )
        direct_files[source_name] = (payload, identity)
    baseline_rows, baseline_compressed, _ = _pinned_raw_rows(
        Path(direct_files["legacy_baseline_process"][1]["path"])
    )
    if (
        baseline_compressed["sha256"]
        != direct_files["legacy_baseline_process"][1]["sha256"]
    ):
        raise ValueError("legacy baseline process identity differs")

    baseline_protocol_declaration = v4_manifest.get("protocol_identity")
    baseline_protocol_payload, baseline_protocol_identity = (
        _pin_declared_file(
            baseline_protocol_declaration,
            label="legacy baseline protocol",
        )
    )
    del baseline_protocol_payload
    if (
        baseline_protocol_identity["sha256"]
        != comparators.get("legacy_baseline_protocol_json_sha256")
    ):
        raise ValueError("legacy baseline protocol digest differs")

    truth_payload, truth_identity = _pin_declared_file(
        sources.get("truth_data"), label="truth data"
    )
    truth_data = _strict_json_object(
        truth_payload, Path(truth_identity["path"])
    )
    raw_truth = raw_manifest["source_identities"].get("truth_data")
    if not _matches_declared_file_identity(raw_truth, truth_identity):
        raise ValueError("raw replay truth identity differs")

    source_identities = {
        "v4_manifest": v4_manifest_identity,
        "v4_compressed_process": v4_compressed,
        "v4_decompressed_process": v4_decompressed,
        "v4_analysis_manifest": direct_files[
            "v4_analysis_manifest"
        ][1],
        "v4_analysis_json": direct_files["v4_analysis_json"][1],
        "v4_analysis_markdown": direct_files[
            "v4_analysis_markdown"
        ][1],
        "legacy_baseline_process": direct_files[
            "legacy_baseline_process"
        ][1],
        "legacy_baseline_protocol_json": baseline_protocol_identity,
        "truth_data": truth_identity,
    }
    return (
        source_identities,
        truth_data,
        baseline_rows,
        v4_rows,
        json.loads(direct_files["v4_analysis_json"][0]),
    )


def _validate_registered_rows(
    rows: list[Mapping],
    *,
    protocol: Mapping,
    truth_data: Mapping,
) -> tuple[object, ...]:
    if len(rows) != 140000:
        raise ValueError("registered raw row denominator differs")
    frames, commands = replay._preflight_frames(dict(truth_data))
    if len(frames) < 500:
        raise ValueError("truth data has fewer than 500 frames")
    expected_ids = set(range(1, 15))
    private_states: dict[int, dict] = {}
    current_public: dict[int, dict] = {}
    previous_seed = None
    previous_frame = None
    validation_protocol = {
        **protocol,
        "_truth_config": truth_data["config"],
    }
    branch_representatives = []
    for row, expected_key in zip(
        rows, replay.iter_registered_keys(), strict=True
    ):
        _, seed, frame_index, robot_id = expected_key
        if seed != previous_seed:
            private_states = {}
            current_public = {}
            previous_frame = None
        if frame_index != previous_frame:
            current_public = {}
        truth = replay._truth_positions(
            frames[frame_index], expected_ids
        )[robot_id]
        held_command = (
            None
            if frame_index == 0
            else commands[frame_index - 1][robot_id]
        )
        reconstructed = validate_and_reconstruct_row(
            row,
            expected_key=expected_key,
            protocol=validation_protocol,
            truth_position=truth,
            current_public=current_public,
            previous_private=private_states.get(robot_id),
            held_command={
                "source_frame": (
                    None if frame_index == 0 else frame_index - 1
                ),
                "command": held_command,
            },
        )
        private_state = reconstructed["private_state"]
        if private_state is None:
            private_states.pop(robot_id, None)
        else:
            private_states[robot_id] = private_state
        current_public[robot_id] = reconstructed["public_output"]
        branch_reconstruction = reconstructed["branch_reconstruction"]
        branch_representatives.append(
            None
            if branch_reconstruction is None
            else tuple(
                (
                    branch["branch_id"],
                    tuple(branch["circle_start"]),
                )
                for branch in branch_reconstruction["branches"]
            )
        )
        previous_seed = seed
        previous_frame = frame_index
    return tuple(branch_representatives)


def _runtime_absolute_no_resolve(path: Path, *, label: str) -> Path:
    candidate = Path(path)
    if not candidate.is_absolute():
        candidate = Path.cwd() / candidate
    if ".." in candidate.parts:
        raise ValueError(f"{label} must not contain parent traversal")
    return candidate


@_with_raw_origin_analysis_scope
def analyze_two_range_reacquisition(
    *,
    protocol_path: Path,
    raw_root: Path,
    output_root: Path,
    invocation_name: str,
    authorization_json: Path | None = None,
    _refresh_raw_origin_binding,
) -> Path:
    """Validate one exact raw invocation and publish compact evidence."""
    if invocation_name not in ANALYZER_INVOCATIONS:
        raise ValueError("analyzer invocation is not registered")
    protocol_path = _runtime_absolute_no_resolve(
        protocol_path, label="protocol_path"
    )
    raw_root = _runtime_absolute_no_resolve(raw_root, label="raw_root")
    output_root = _runtime_absolute_no_resolve(
        output_root, label="output_root"
    )
    if invocation_name == "registered_analyzer" and authorization_json is None:
        raise ValueError("registered analyzer requires authorization")
    if invocation_name != "registered_analyzer" and authorization_json is not None:
        raise ValueError("smoke analyzer rejects authorization")
    protocol_payload, protocol_identity = _pinned_file_identity(protocol_path)
    protocol = _strict_json_object(protocol_payload, protocol_path)
    protocol_id = protocol.get("protocol_id")
    disk_contract = protocol.get("disk_contract")
    _production_compact_cap(disk_contract)
    invocations = protocol.get("invocations")
    declaration = (
        invocations.get(invocation_name)
        if isinstance(invocations, Mapping)
        else None
    )
    if (
        not isinstance(protocol_id, str)
        or not protocol_id
        or not isinstance(disk_contract, Mapping)
        or not isinstance(declaration, Mapping)
        or declaration.get("invocation_name") != invocation_name
        or Path(declaration.get("input_root", "")) != raw_root
        or Path(declaration.get("output_root", "")) != output_root
        or declaration.get("expected_rows")
        != (140000 if invocation_name == "registered_analyzer" else 18)
    ):
        raise ValueError("protocol invocation binding differs")
    expected_raw_invocation = {
        "smoke_analyzer_a": "smoke_a",
        "smoke_analyzer_b": "smoke_b",
        "registered_analyzer": "registered_replay",
    }[invocation_name]
    manifest_path = raw_root / ANALYZER_MANIFEST_NAME
    raw_manifest_payload, raw_manifest_identity = _pinned_file_identity(
        manifest_path
    )
    raw_manifest = _strict_json_object(
        raw_manifest_payload, manifest_path
    )
    replay._validate_manifest(raw_manifest)
    if (
        raw_manifest["status"] != "completed"
        or raw_manifest["invocation_name"] != expected_raw_invocation
        or Path(raw_manifest["output_root"]) != raw_root
        or raw_manifest["expected_rows"]
        != declaration["expected_rows"]
        or raw_manifest["observed_rows"]
        != declaration["expected_rows"]
        or raw_manifest["protocol_id"] != protocol_id
    ):
        raise ValueError("raw manifest differs from analyzer invocation")
    raw_protocol = raw_manifest["protocol_identity"]
    if raw_protocol is None or any(
        raw_protocol[field] != protocol_identity[field]
        for field in replay.FILE_IDENTITY_FIELDS
    ):
        raise ValueError("raw protocol identity differs")
    process_path = raw_root / replay.RAW_PROCESS_NAME
    rows, raw_compressed, raw_decompressed = _pinned_raw_rows(process_path)
    declared_process = raw_manifest["process_identity"]
    if (
        declared_process["path"] != str(process_path)
        or any(
            declared_process[field] != raw_compressed[field]
            for field in ("device", "inode", "size", "mtime_ns")
        )
        or declared_process["compressed_sha256"]
        != raw_compressed["sha256"]
        or declared_process["decompressed_sha256"]
        != raw_decompressed["sha256"]
    ):
        raise ValueError("raw process identity differs")
    authorization_identity = None
    source_identities = {
        "raw_manifest": raw_manifest_identity,
        "raw_compressed_process": raw_compressed,
        "raw_decompressed_process": raw_decompressed,
    }
    raw_identity_commitment = _raw_identity_commitment(
        source_identities
    )
    protocol_identity_commitment = protocol_identity["sha256"]
    source_binding = None
    result_identities = {field: None for field in IDENTITY_FIELDS}
    result_identities.update(
        {
            "protocol": _result_identity(protocol_identity),
            "raw_manifest": _result_identity(raw_manifest_identity),
            "raw_compressed_process": _result_identity(raw_compressed),
            "raw_decompressed_process": _result_identity(raw_decompressed),
        }
    )
    expected_rows = declaration["expected_rows"]
    if invocation_name == "registered_analyzer":
        if (
            protocol_id != replay.REGISTERED_PROTOCOL_ID
            or disk_contract != replay.REGISTERED_DISK_CONTRACT
        ):
            raise ValueError("registered protocol binding differs")
        _, _, authorization_identity = _validate_registered_authorization(
            authorization_path=_runtime_absolute_no_resolve(
                authorization_json, label="authorization_json"
            ),
            protocol_path=protocol_path,
            protocol_payload=protocol_payload,
            protocol=protocol,
            protocol_identity=protocol_identity,
            raw_root=raw_root,
            output_root=output_root,
            raw_manifest=raw_manifest,
        )
        (
            registered_sources,
            truth_data,
            baseline_rows,
            v4_rows,
            _,
        ) = _registered_source_data(
            protocol=protocol,
            raw_manifest=raw_manifest,
        )
        source_identities.update(registered_sources)
        branch_representatives = _validate_registered_rows(
            rows, protocol=protocol, truth_data=truth_data
        )
        result, source_binding = (
            _aggregate_two_range_reacquisition_with_projection(
                baseline_rows=baseline_rows,
                v4_rows=v4_rows,
                new_rows=rows,
                truth_data=truth_data,
                protocol=protocol,
                branch_representatives=branch_representatives,
                protocol_identity_commitment=(
                    protocol_identity_commitment
                ),
                raw_identity_commitment=raw_identity_commitment,
            )
        )
        result_identities["authorization"] = _result_identity(
            authorization_identity
        )
        for name in ANALYSIS_SOURCE_MEMBER_NAMES[
            "registered_analyzer"
        ]:
            identity_name = name
            if name in source_identities:
                result_identities[identity_name] = _result_identity(
                    source_identities[name]
                )
        result["identities"] = {
            field: result_identities[field] for field in IDENTITY_FIELDS
        }
        result["budgets"]["raw_allocated_bytes"] = sum(
            identity["allocated_bytes"]
            for name, identity in source_identities.items()
            if not name.endswith("decompressed_process")
        )
        result["semantic_payload_sha256"] = _semantic_sha256(result)
    else:
        _validate_smoke_rows(rows, protocol=protocol)
        raw_sources = raw_manifest["source_identities"]
        mechanism_payload, mechanism_identity = _pinned_file_identity(
            Path(raw_sources["mechanism_fixture"]["path"])
        )
        del mechanism_payload
        synthetic_payload, synthetic_identity = _pinned_file_identity(
            Path(raw_sources["replay_source"]["path"])
        )
        del synthetic_payload
        for name, observed, declared in (
            (
                "mechanism_fixture",
                mechanism_identity,
                raw_sources["mechanism_fixture"],
            ),
            (
                "synthetic_case_source",
                synthetic_identity,
                raw_sources["replay_source"],
            ),
        ):
            if not _matches_declared_file_identity(declared, observed):
                raise ValueError(f"{name} identity differs")
        source_identities.update(
            {
                "mechanism_fixture": mechanism_identity,
                "synthetic_case_source": synthetic_identity,
            }
        )
        result_identities.update(
            {
                "mechanism_fixture": _result_identity(mechanism_identity),
                "synthetic_case_source": _result_identity(
                    synthetic_identity
                ),
            }
        )
        raw_allocated = sum(
            identity["allocated_bytes"]
            for identity in (
                raw_manifest_identity,
                raw_compressed,
            )
        )
        result, source_binding = _smoke_result_with_projection(
            invocation_name=invocation_name,
            protocol_id=protocol_id,
            rows=rows,
            identities=result_identities,
            raw_allocated_bytes=raw_allocated,
            protocol_identity_commitment=protocol_identity_commitment,
            raw_identity_commitment=raw_identity_commitment,
        )
    started_at = _utc_now()
    null_outputs = {
        name: None for name in ANALYSIS_OUTPUT_MEMBER_NAMES
    }
    manifest = _analysis_manifest(
        protocol_id=protocol_id,
        invocation_name=invocation_name,
        status="creating",
        output_root=output_root,
        protocol_identity=protocol_identity,
        authorization_identity=authorization_identity,
        source_identities=source_identities,
        output_identities=null_outputs,
        expected_rows=expected_rows,
        observed_rows=0,
        disk_contract=disk_contract,
        started_at=started_at,
        completed_at=None,
        error=None,
    )
    try:
        transaction = replay._create_exact_root(output_root)
    except BaseException as error:
        failed = _analysis_manifest(
            protocol_id=protocol_id,
            invocation_name=invocation_name,
            status="failed",
            output_root=output_root,
            protocol_identity=protocol_identity,
            authorization_identity=authorization_identity,
            source_identities=source_identities,
            output_identities=null_outputs,
            expected_rows=expected_rows,
            observed_rows=len(rows),
            disk_contract=disk_contract,
            started_at=started_at,
            completed_at=_utc_now(),
            error={
                "type": type(error).__name__,
                "message": _canonical_error_message(error),
            },
        )
        try:
            _publish_analysis_preallocation_failure(output_root, failed)
        finally:
            _revoke_raw_origin_binding(source_binding)
        raise
    transaction["output_root"] = str(output_root)
    stages = []
    try:
        _publish_analysis_manifest(transaction, manifest)
        json_payload = (
            replay.ordered_strict_json_bytes(result, ANALYSIS_FIELDS) + b"\n"
        )
        markdown_payload = _markdown(result).encode("utf-8")
        json_stage = _stage_output(
            transaction,
            final_name=OUTPUT_JSON_NAME,
            payload=json_payload,
        )
        stages.append(json_stage)
        markdown_stage = _stage_output(
            transaction,
            final_name=OUTPUT_MARKDOWN_NAME,
            payload=markdown_payload,
        )
        stages.append(markdown_stage)
        manifest_metadata = os.stat(
            ANALYZER_MANIFEST_NAME,
            dir_fd=transaction["root_fd"],
            follow_symlinks=False,
        )
        compact_cap = _production_compact_cap(disk_contract)
        for _ in range(3):
            compact_allocated = (
                json_stage["identity"]["allocated_bytes"]
                + markdown_stage["identity"]["allocated_bytes"]
                + manifest_metadata.st_blocks * 512
            )
            if compact_allocated > compact_cap:
                raise DiskSpaceError(
                    "compact bundle exceeds allocated-byte cap"
                )
            if (
                result["budgets"]["compact_allocated_bytes"]
                == compact_allocated
            ):
                break
            result["budgets"][
                "compact_allocated_bytes"
            ] = compact_allocated
            result["semantic_payload_sha256"] = _semantic_sha256(result)
            _discard_staged_output(transaction, json_stage)
            stages.remove(json_stage)
            json_payload = (
                replay.ordered_strict_json_bytes(result, ANALYSIS_FIELDS)
                + b"\n"
            )
            json_stage = _stage_output(
                transaction,
                final_name=OUTPUT_JSON_NAME,
                payload=json_payload,
            )
            stages.insert(0, json_stage)
        else:
            raise DiskSpaceError(
                "compact JSON allocated-byte budget did not stabilize"
            )
        _refresh_raw_origin_binding(source_binding, result)
        _validate_analysis_result(
            result,
            source_projection=source_binding,
            expected_invocation_name=invocation_name,
            expected_protocol_id=protocol_id,
            protocol_identity_commitment=(
                protocol_identity_commitment
            ),
            raw_identity_commitment=raw_identity_commitment,
        )
        json_identity = _publish_staged_output(
            transaction, json_stage
        )
        markdown_identity = _publish_staged_output(
            transaction, markdown_stage
        )
        output_identities = {
            "analysis_json": json_identity,
            "analysis_markdown": markdown_identity,
        }
        verification_inputs = {"protocol": protocol_identity}
        if authorization_identity is not None:
            verification_inputs["authorization"] = authorization_identity
        verification_inputs.update(source_identities)
        _reverify_inputs(verification_inputs)
        for stage in stages:
            _verify_staged_output(transaction, stage)
        final_manifest_metadata = os.stat(
            ANALYZER_MANIFEST_NAME,
            dir_fd=transaction["root_fd"],
            follow_symlinks=False,
        )
        final_compact_allocated = (
            json_identity["allocated_bytes"]
            + markdown_identity["allocated_bytes"]
            + final_manifest_metadata.st_blocks * 512
        )
        if (
            final_compact_allocated
            != result["budgets"]["compact_allocated_bytes"]
            or final_compact_allocated > compact_cap
        ):
            raise DiskSpaceError(
                "final compact bundle allocated-byte contract differs"
            )
        completed = _analysis_manifest(
            protocol_id=protocol_id,
            invocation_name=invocation_name,
            status="completed",
            output_root=output_root,
            protocol_identity=protocol_identity,
            authorization_identity=authorization_identity,
            source_identities=source_identities,
            output_identities=output_identities,
            expected_rows=expected_rows,
            observed_rows=expected_rows,
            disk_contract=disk_contract,
            started_at=started_at,
            completed_at=_utc_now(),
            error=None,
        )
        _publish_analysis_manifest(transaction, completed)
        for stage in stages:
            _verify_staged_output(transaction, stage)
        completed_manifest_metadata = os.stat(
            ANALYZER_MANIFEST_NAME,
            dir_fd=transaction["root_fd"],
            follow_symlinks=False,
        )
        completed_compact_allocated = (
            json_identity["allocated_bytes"]
            + markdown_identity["allocated_bytes"]
            + completed_manifest_metadata.st_blocks * 512
        )
        if (
            completed_compact_allocated
            != result["budgets"]["compact_allocated_bytes"]
            or completed_compact_allocated > compact_cap
        ):
            raise DiskSpaceError(
                "completed compact bundle allocated-byte contract differs"
            )
        for stage in stages:
            _close_staged_output(transaction, stage)
        _assert_output_root_path(transaction, "return")
        return output_root
    except BaseException as error:
        for stage in stages:
            _cleanup_staged_output(transaction, stage, error)
        failed_protocol_identity = protocol_identity
        failed_authorization_identity = authorization_identity
        failed_sources = source_identities
        if isinstance(error, _SourceIdentityMismatch):
            failed_protocol_identity = error.observed.get(
                "protocol", protocol_identity
            )
            if authorization_identity is not None:
                failed_authorization_identity = error.observed.get(
                    "authorization", authorization_identity
                )
            failed_sources = {
                name: error.observed.get(name, source_identities[name])
                for name in ANALYSIS_SOURCE_MEMBER_NAMES[invocation_name]
            }
        failed = _analysis_manifest(
            protocol_id=protocol_id,
            invocation_name=invocation_name,
            status="failed",
            output_root=output_root,
            protocol_identity=failed_protocol_identity,
            authorization_identity=failed_authorization_identity,
            source_identities=failed_sources,
            output_identities=null_outputs,
            expected_rows=expected_rows,
            observed_rows=min(len(rows), expected_rows),
            disk_contract=disk_contract,
            started_at=started_at,
            completed_at=_utc_now(),
            error={
                "type": type(error).__name__,
                "message": _canonical_error_message(error),
            },
        )
        try:
            _publish_analysis_manifest(transaction, failed)
            _audit_failed_bundle_cap(transaction, disk_contract)
        except _OutputRootIdentityMismatch as root_error:
            failed["error"]["message"] = _canonical_error_message(
                error,
                contexts=(root_error,),
            )
            try:
                _publish_analysis_manifest(
                    transaction,
                    failed,
                    require_path_binding=False,
                )
                _audit_failed_bundle_cap(transaction, disk_contract)
            except BaseException as terminal_error:
                error.add_note(
                    "detached failed analysis manifest could not be "
                    f"published: {terminal_error}"
                )
        except BaseException as terminal_error:
            error.add_note(
                "failed analysis manifest could not be published: "
                f"{terminal_error}"
            )
        raise
    finally:
        _revoke_raw_origin_binding(source_binding)
        replay._close_output_transaction(transaction)


del _with_raw_origin_analysis_scope


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol-path", type=Path, required=True)
    parser.add_argument("--raw-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument(
        "--invocation-name",
        choices=ANALYZER_INVOCATIONS,
        required=True,
    )
    parser.add_argument("--authorization-json", type=Path)
    arguments = parser.parse_args(argv)
    analyze_two_range_reacquisition(
        protocol_path=arguments.protocol_path,
        raw_root=arguments.raw_root,
        output_root=arguments.output_root,
        invocation_name=arguments.invocation_name,
        authorization_json=arguments.authorization_json,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
