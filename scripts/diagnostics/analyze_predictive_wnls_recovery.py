"""Compact exact-key analysis for predictive-WNLS Stage-1 evidence."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import importlib.machinery
import importlib.util
import json
import math
import os
import sys
from collections import Counter, defaultdict
from collections.abc import Iterable, Mapping
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

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
        entry for entry in sys.path if not _is_script_repository_path(entry)
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

from scripts.diagnostics import replay_predictive_wnls_recovery as replay
from scripts.diagnostics.run_diagnostic import (
    HARD_FLOOR_BYTES,
    START_BYTES,
    DiskSpaceError,
    _nearest_existing_ancestor,
    require_start_space,
)


ANALYSIS_SCHEMA_ID = "cbf2026-predictive-wnls-development-analysis-v2"
OUTPUT_JSON_NAME = "predictive-wnls-development.json"
OUTPUT_MARKDOWN_NAME = "predictive-wnls-development.md"
ANALYZER_MANIFEST_NAME = "manifest.json"
COMPACT_OUTPUT_CAP_BYTES = 10_000_000
ANALYZER_TEST_SCHEMA_ID = "cbf2026-predictive-wnls-analyzer-test-v1"
Q95 = 5.991464547107979
Q3 = 9.0
E_CAT_M = 50.0

_BASELINE_OUTPUTS = ("fresh", "legacy_published", "unavailable")
_MECHANISM_KEYS = {
    "frame44_recovery": (20260736, 44, 14),
    "legacy_999m_stale": (20260736, 138, 14),
    "legacy_168m_fresh": (20260730, 177, 14),
}


def _finite_number(value: object) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _distribution(values: Iterable[float]) -> dict:
    data = [float(value) for value in values]
    if not data:
        return {
            "denominator": 0,
            "p50": None,
            "p95": None,
            "p99": None,
            "maximum": None,
        }
    array = np.asarray(data, dtype=float)
    if not np.all(np.isfinite(array)):
        raise ValueError("non-finite value entered a distribution")
    return {
        "denominator": len(data),
        "p50": float(np.percentile(array, 50)),
        "p95": float(np.percentile(array, 95)),
        "p99": float(np.percentile(array, 99)),
        "maximum": float(np.max(array)),
    }


def _rate(numerator: int, denominator: int) -> dict:
    return {
        "numerator": int(numerator),
        "denominator": int(denominator),
        "rate": None if denominator == 0 else numerator / denominator,
    }


def _baseline_key(row: Mapping) -> tuple[int, int, int]:
    values = (row.get("seed"), row.get("frame_index"), row.get("robot_id"))
    if any(isinstance(value, bool) or not isinstance(value, int) for value in values):
        raise ValueError("baseline exact key must contain integers")
    return values


def _development_key(row: Mapping) -> tuple[int, int, int]:
    return _baseline_key(row)


def _map_baseline_row(row: Mapping) -> dict:
    if row.get("graph_case") != "dynamic_dag_wnls":
        raise ValueError("only dynamic_dag_wnls rows may be mapped")
    key = _baseline_key(row)
    status = row.get("status")
    attempt = row.get("attempt_status")
    error = row.get("error_norm")
    if attempt == "converged":
        mapped_attempt = "accepted"
    elif attempt in {"failed", "invalid"}:
        mapped_attempt = attempt
    else:
        raise ValueError(f"invalid baseline attempt status at {key}")
    finite_error = _finite_number(error)
    finite_flag = row.get("finite")
    if status == "converged" and finite_error and finite_flag is True:
        output = "fresh"
    elif status == "stale" and finite_error and finite_flag is True:
        output = "legacy_published"
    elif (
        status in {"failed", "invalid"}
        and error is None
        and finite_flag is False
    ):
        output = "unavailable"
    else:
        raise ValueError(f"invalid baseline publication mapping at {key}")
    return {
        "key": key,
        "attempt_status": mapped_attempt,
        "output_status": output,
        "error": float(error) if finite_error else None,
    }


def _validate_row_schema(row: Mapping) -> None:
    if not isinstance(row, dict) or set(row) != set(replay.ROW_FIELDS):
        raise ValueError("development row differs from exact 36-field schema")
    variant = row["variant"]
    if variant not in replay.DEVELOPMENT_VARIANTS:
        raise ValueError("unknown development variant")
    _development_key(row)
    if row["attempt_status"] not in replay.ATTEMPT_STATUSES:
        raise ValueError("unknown attempt status")
    status = row["output_status"]
    if status not in replay.OUTPUT_STATUSES:
        raise ValueError("unknown output status")
    if (row["attempt_status"] == "accepted") != (status == "fresh"):
        raise ValueError("accepted attempt and fresh output must be equivalent")
    expected_local_index = (row["robot_id"] - 1) % 7 + 1
    if (
        isinstance(row["squad_local_index"], bool)
        or not isinstance(row["squad_local_index"], int)
        or row["squad_local_index"] != expected_local_index
    ):
        raise ValueError("squad-local index differs from paper formation mapping")
    error = row["offline_error_norm"]
    truth = np.asarray(row["offline_truth_position"], dtype=float)
    if truth.shape != (2,) or not np.all(np.isfinite(truth)):
        raise ValueError("offline truth must be finite 2-D")
    if status == "fresh":
        if (
            row["prediction_age"] != 0
            or not _finite_number(error)
            or not isinstance(row["offline_fresh_containment"], bool)
            or row["offline_aged_radius_containment"] is not None
            or not _finite_number(row["offline_fresh_q_error"])
            or row["offline_aged_q_error"] is not None
        ):
            raise ValueError("fresh row has inconsistent offline fields")
        estimate = np.asarray(row["estimate"], dtype=float)
        covariance = np.asarray(row["fresh_modeled_covariance"], dtype=float)
        epsilon = row["fresh_epsilon"]
        if (
            estimate.shape != (2,)
            or not np.all(np.isfinite(estimate))
            or covariance.shape != (2, 2)
            or not np.all(np.isfinite(covariance))
            or not np.allclose(
                covariance, covariance.T, rtol=0.0, atol=1e-12
            )
            or float(np.min(np.linalg.eigvalsh(covariance))) <= 0.0
            or not _finite_number(epsilon)
            or epsilon <= 0
            or row["aged_modeled_covariance"] is not None
            or row["aged_modeled_radius"] is not None
        ):
            raise ValueError("fresh public state differs from exact schema")
        residual = truth - estimate
        computed_error = float(np.linalg.norm(residual))
        expected_radius = 3.0 * math.sqrt(
            float(np.max(np.linalg.eigvalsh(covariance)))
        )
        try:
            computed_q = float(residual @ np.linalg.solve(covariance, residual))
        except np.linalg.LinAlgError as error_value:
            raise ValueError("fresh covariance is singular") from error_value
        if (
            not math.isclose(error, computed_error, rel_tol=1e-12, abs_tol=1e-12)
            or not math.isclose(
                epsilon, expected_radius, rel_tol=1e-12, abs_tol=1e-12
            )
            or row["offline_fresh_containment"] != (computed_error <= epsilon)
            or not math.isclose(
                row["offline_fresh_q_error"],
                computed_q,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        ):
            raise ValueError("fresh offline diagnostic drift")
    elif status == "predicted":
        if (
            row["prediction_age"] not in {1, 2}
            or not _finite_number(error)
            or not isinstance(row["offline_aged_radius_containment"], bool)
            or row["offline_fresh_containment"] is not None
            or not _finite_number(row["offline_aged_q_error"])
            or row["offline_fresh_q_error"] is not None
        ):
            raise ValueError("predicted row has inconsistent offline fields")
        estimate = np.asarray(row["estimate"], dtype=float)
        covariance = np.asarray(row["aged_modeled_covariance"], dtype=float)
        radius = row["aged_modeled_radius"]
        if (
            estimate.shape != (2,)
            or not np.all(np.isfinite(estimate))
            or covariance.shape != (2, 2)
            or not np.all(np.isfinite(covariance))
            or not np.allclose(
                covariance, covariance.T, rtol=0.0, atol=1e-12
            )
            or float(np.min(np.linalg.eigvalsh(covariance))) <= 0.0
            or not _finite_number(radius)
            or radius <= 0
            or row["fresh_modeled_covariance"] is not None
            or row["fresh_epsilon"] is not None
        ):
            raise ValueError("predicted public state differs from exact schema")
        residual = truth - estimate
        computed_error = float(np.linalg.norm(residual))
        expected_radius = 3.0 * math.sqrt(
            float(np.max(np.linalg.eigvalsh(covariance)))
        )
        try:
            computed_q = float(residual @ np.linalg.solve(covariance, residual))
        except np.linalg.LinAlgError as error_value:
            raise ValueError("aged covariance is singular") from error_value
        if (
            not math.isclose(error, computed_error, rel_tol=1e-12, abs_tol=1e-12)
            or not math.isclose(
                radius, expected_radius, rel_tol=1e-12, abs_tol=1e-12
            )
            or row["offline_aged_radius_containment"] != (computed_error <= radius)
            or not math.isclose(
                row["offline_aged_q_error"],
                computed_q,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        ):
            raise ValueError("aged offline diagnostic drift")
    elif (
        row["prediction_age"] is not None
        or error is not None
        or row["offline_fresh_containment"] is not None
        or row["offline_aged_radius_containment"] is not None
        or row["offline_fresh_q_error"] is not None
        or row["offline_aged_q_error"] is not None
        or row["estimate"] is not None
        or row["fresh_modeled_covariance"] is not None
        or row["fresh_epsilon"] is not None
        or row["aged_modeled_covariance"] is not None
        or row["aged_modeled_radius"] is not None
    ):
        raise ValueError("unavailable row has inconsistent offline fields")


def audit_input_limits(truth_data: Mapping, *, component_limit: float) -> dict:
    """Audit each physical command once from the protocol-bound truth input."""
    if not _finite_number(component_limit) or component_limit <= 0:
        raise ValueError("component limit must be finite and positive")
    states = truth_data.get("state")
    if not isinstance(states, list) or not states:
        raise ValueError("truth data must contain non-empty state rows")
    commands: dict[tuple[int, int], tuple[float, float]] = {}
    positions: dict[tuple[int, int], tuple[float, float]] = {}
    for frame, state in enumerate(states):
        if not isinstance(state, dict):
            raise ValueError("truth state must be an object")
        declared_frame = state.get("frame_index")
        if declared_frame is not None and declared_frame != frame:
            raise ValueError("declared truth frame differs from state index")
        robots = state.get("robots")
        if not isinstance(robots, list) or not robots:
            raise ValueError("truth state robots must be non-empty")
        seen = set()
        for robot in robots:
            if not isinstance(robot, dict):
                raise ValueError("truth robot must be an object")
            robot_id = robot.get("id")
            if (
                isinstance(robot_id, bool)
                or not isinstance(robot_id, int)
                or robot_id <= 0
                or robot_id in seen
            ):
                raise ValueError("truth robot ids must be unique positive integers")
            seen.add(robot_id)
            result = (robot.get("opt") or {}).get("result")
            if not isinstance(result, dict):
                raise ValueError("truth row lacks applied opt.result")
            vx, vy = result.get("vx"), result.get("vy")
            if not _finite_number(vx) or not _finite_number(vy):
                raise ValueError("truth applied command must be finite")
            commands[(frame, robot_id)] = (float(vx), float(vy))
            state_position = robot.get("state")
            if not isinstance(state_position, dict):
                raise ValueError("truth robot lacks a state position")
            x, y = state_position.get("x"), state_position.get("y")
            if not _finite_number(x) or not _finite_number(y):
                raise ValueError("truth state position must be finite")
            positions[(frame, robot_id)] = (float(x), float(y))
    components = [max(abs(vx), abs(vy)) for vx, vy in commands.values()]
    norms = [math.hypot(vx, vy) for vx, vy in commands.values()]
    tolerance = 1e-7
    violations = sum(value > component_limit + tolerance for value in components)
    return {
        "source": "protocol_bound_truth_state_opt_result",
        "unique_physical_rows": len(commands),
        "component_limit": float(component_limit),
        "tolerance": tolerance,
        "component_bound_violations": violations,
        "violation_rate": violations / len(commands),
        "maximum_applied_component": max(components),
        "maximum_planar_norm": max(norms),
        "maximum_displacement": 0.5 * max(norms),
        "_commands": commands,
        "_truth_positions": positions,
        "_expected_predecessor_keys": {
            key for key in commands if key[0] > 0
        },
    }


def _candidate_records(
    row: Mapping,
) -> Iterable[tuple[str, bool, float | None, float | None]]:
    truth = np.asarray(row["offline_truth_position"], dtype=float)
    if truth.shape != (2,) or not np.all(np.isfinite(truth)):
        raise ValueError("offline truth position must be finite 2-D")
    candidates = row["candidates"]
    if not isinstance(candidates, list):
        raise ValueError("candidate records must be a list")
    for candidate in candidates:
        if not isinstance(candidate, list) or len(candidate) != len(
            replay.CANDIDATE_FIELDS
        ):
            raise ValueError("candidate differs from compact exact schema")
        accepted = candidate[6]
        estimate = candidate[3]
        q_innov = candidate[8]
        if not isinstance(accepted, bool):
            raise ValueError("candidate accepted flag must be boolean")
        gate = candidate[9]
        trace = candidate[10]
        if (
            not isinstance(candidate[0], str)
            or not isinstance(gate, list)
            or len(gate) != len(replay.GATE_DIAGNOSTIC_FIELDS)
            or not isinstance(trace, list)
            or any(
                not isinstance(proposal, list)
                or len(proposal) != len(replay.PROPOSAL_TRACE_FIELDS)
                for proposal in trace
            )
        ):
            raise ValueError("candidate compact diagnostics differ from exact schema")
        error = None
        if estimate is not None:
            estimate_array = np.asarray(estimate, dtype=float)
            if estimate_array.shape != (2,) or not np.all(np.isfinite(estimate_array)):
                raise ValueError("candidate estimate must be finite 2-D")
            error = float(np.linalg.norm(truth - estimate_array))
        if q_innov is not None and not _finite_number(q_innov):
            raise ValueError("candidate q innovation must be finite or null")
        yield (
            candidate[0],
            accepted,
            None if q_innov is None else float(q_innov),
            error,
        )


def _new_stratum() -> dict:
    return {
        "rows": 0,
        "attempt_counts": Counter(),
        "output_counts": Counter(),
        "fresh_errors": [],
        "published_errors": [],
    }


def _finish_strata(values: Mapping[str, Mapping[str, dict]]) -> dict:
    result = {}
    for family, groups in values.items():
        result[family] = {}
        for label, state in sorted(groups.items()):
            result[family][label] = {
                "rows": state["rows"],
                "attempt_counts": {
                    name: state["attempt_counts"].get(name, 0)
                    for name in replay.ATTEMPT_STATUSES
                },
                "output_counts": {
                    name: state["output_counts"].get(name, 0)
                    for name in replay.OUTPUT_STATUSES
                },
                "fresh_errors": _distribution(state["fresh_errors"]),
                "all_published_errors": _distribution(state["published_errors"]),
            }
    return result


def _new_paired_state() -> dict:
    return {
        "published_transitions": Counter(),
        "published_changes": [],
        "attrition_errors": [],
        "fresh_transitions": Counter(),
        "both_fresh_changes": [],
        "both_fresh_old_errors": [],
        "both_fresh_new_errors": [],
        "fresh_predicted_errors": [],
        "fresh_unavailable_errors": [],
    }


def _update_paired(state: dict, old: Mapping, new: Mapping) -> None:
    new_status = new["output_status"]
    if old["output_status"] != "unavailable":
        state["published_transitions"][new_status] += 1
        if new_status in {"fresh", "predicted"}:
            state["published_changes"].append(
                new["offline_error_norm"] - old["error"]
            )
        else:
            state["attrition_errors"].append(old["error"])
    if old["output_status"] == "fresh":
        state["fresh_transitions"][new_status] += 1
        if new_status == "fresh":
            state["both_fresh_old_errors"].append(old["error"])
            state["both_fresh_new_errors"].append(new["offline_error_norm"])
            state["both_fresh_changes"].append(
                new["offline_error_norm"] - old["error"]
            )
        elif new_status == "predicted":
            state["fresh_predicted_errors"].append(old["error"])
        else:
            state["fresh_unavailable_errors"].append(old["error"])


def _finish_paired(state: Mapping) -> dict:
    published_transitions = state["published_transitions"]
    published_changes = state["published_changes"]
    attrition_errors = state["attrition_errors"]
    fresh_transitions = state["fresh_transitions"]
    both_fresh_changes = state["both_fresh_changes"]
    both_fresh_old_errors = state["both_fresh_old_errors"]
    both_fresh_new_errors = state["both_fresh_new_errors"]
    fresh_predicted_errors = state["fresh_predicted_errors"]
    fresh_unavailable_errors = state["fresh_unavailable_errors"]
    excluded = fresh_predicted_errors + fresh_unavailable_errors
    return {
        "baseline_published": {
            "cohort_denominator": sum(published_transitions.values()),
            "transition_counts": {
                name: published_transitions.get(name, 0)
                for name in replay.OUTPUT_STATUSES
            },
            "both_published_error_change": _distribution(published_changes),
            "newly_unavailable": {
                "count": len(attrition_errors),
                "rate": (
                    len(attrition_errors) / sum(published_transitions.values())
                    if published_transitions
                    else None
                ),
                "baseline_errors": _distribution(attrition_errors),
                "catastrophic_count": sum(value >= E_CAT_M for value in attrition_errors),
            },
        },
        "baseline_fresh": {
            "cohort_denominator": sum(fresh_transitions.values()),
            "transition_counts": {
                name: fresh_transitions.get(name, 0)
                for name in replay.OUTPUT_STATUSES
            },
            "both_fresh_error_change": _distribution(both_fresh_changes),
            "both_fresh_baseline_errors": _distribution(both_fresh_old_errors),
            "both_fresh_new_errors": _distribution(both_fresh_new_errors),
            "downgraded_to_predicted": {
                "count": len(fresh_predicted_errors),
                "baseline_errors": _distribution(fresh_predicted_errors),
                "catastrophic_count": sum(
                    value >= E_CAT_M for value in fresh_predicted_errors
                ),
            },
            "newly_unavailable": {
                "count": len(fresh_unavailable_errors),
                "baseline_errors": _distribution(fresh_unavailable_errors),
                "catastrophic_count": sum(
                    value >= E_CAT_M for value in fresh_unavailable_errors
                ),
            },
            "excluded_from_both_fresh": {
                "count": len(excluded),
                "baseline_errors": _distribution(excluded),
                "catastrophic_count": sum(value >= E_CAT_M for value in excluded),
            },
        },
    }


def resolve_mechanism_records(
    baseline: Mapping[tuple[int, int, int], Mapping],
    variants: Mapping[str, Mapping[tuple[int, int, int], Mapping]],
) -> dict:
    """Resolve predeclared adverse records by exact key."""
    records = {}
    for name, key in _MECHANISM_KEYS.items():
        old = baseline.get(key)
        records[name] = {
            "key": {
                "seed": key[0],
                "frame_index": key[1],
                "robot_id": key[2],
            },
            "baseline": None
            if old is None
            else {
                "attempt_status": old.get("attempt_status"),
                "output_status": old.get("output_status", old.get("status")),
                "error": old.get("error", old.get("error_norm")),
            },
            "variants": {
                variant: (
                    None
                    if rows.get(key) is None
                    else {
                        "attempt_status": rows[key]["attempt_status"],
                        "output_status": rows[key]["output_status"],
                        "error": rows[key]["offline_error_norm"],
                        "prediction_age": rows[key]["prediction_age"],
                    }
                )
                for variant, rows in variants.items()
            },
        }
    return records


def _validate_production_disk_metrics(
    manifest: Mapping, *, actual_allocated_bytes: int
) -> None:
    values = {
        name: manifest.get(name)
        for name in (
            "free_bytes_before",
            "free_bytes_after",
            "allocated_bytes",
            "raw_bundle_cap_bytes",
        )
    }
    if any(
        isinstance(value, bool) or not isinstance(value, int) or value < 0
        for value in (*values.values(), actual_allocated_bytes)
    ):
        raise ValueError("production disk metrics must be nonnegative integers")
    if (
        values["raw_bundle_cap_bytes"] != replay.RAW_BUNDLE_CAP_BYTES
        or values["allocated_bytes"] != actual_allocated_bytes
        or values["allocated_bytes"] > values["raw_bundle_cap_bytes"]
        or values["free_bytes_before"] < START_BYTES
        or values["free_bytes_after"] < HARD_FLOOR_BYTES
    ):
        raise ValueError("production replay disk metrics do not reconcile")


def _finish_variant(state: dict) -> dict:
    rows = state["rows"]
    output_counts = {
        name: state["output_counts"].get(name, 0)
        for name in replay.OUTPUT_STATUSES
    }
    attempt_counts = {
        name: state["attempt_counts"].get(name, 0)
        for name in replay.ATTEMPT_STATUSES
    }
    if sum(output_counts.values()) != rows or sum(attempt_counts.values()) != rows:
        raise ValueError("variant attempt/output counts do not reconcile")
    fresh_errors = state["fresh_errors"]
    published_errors = state["published_errors"]
    fresh_q = state["fresh_q"]
    aged_q = state["aged_q"]
    result = {
        "budgets": {"attempted": rows, "outputs": rows},
        "attempt_counts": attempt_counts,
        "output_counts": output_counts,
        "errors": {
            "fresh": _distribution(fresh_errors),
            "all_published": _distribution(published_errors),
        },
        "paired": _finish_paired(state["paired"]),
        "catastrophic": {
            "fresh": _rate(
                sum(value >= E_CAT_M for value in fresh_errors), len(fresh_errors)
            ),
            "all_published": _rate(
                sum(value >= E_CAT_M for value in published_errors),
                len(published_errors),
            ),
        },
        "availability": {
            "fresh": _rate(output_counts["fresh"], rows),
            "fresh_or_predicted": _rate(
                output_counts["fresh"] + output_counts["predicted"], rows
            ),
        },
        "prediction": {
            "maximum_age": max(state["prediction_ages"], default=None),
            "age_above_two_count": sum(
                value > 2 for value in state["prediction_ages"]
            ),
            "maximum_unavailable_streak": state["maximum_unavailable_streak"],
        },
        "references": {
            "active_set_composition": dict(sorted(state["active_composition"].items())),
            "freshness_counts": dict(sorted(state["reference_freshness"].items())),
        },
        "integrity": {
            "reference_violation_count": state["integrity"][
                "reference_violation_count"
            ],
            "reference_violation_reasons": dict(
                sorted(state["integrity"]["reference_violation_reasons"].items())
            ),
            "nonfresh_anchor_use_count": state["integrity"][
                "nonfresh_anchor_use_count"
            ],
            "insufficient_provenance_count": state["integrity"][
                "insufficient_provenance_count"
            ],
            "provenance_mismatch_count": state["integrity"][
                "provenance_mismatch_count"
            ],
            "ascending_dag_violation_count": state["integrity"][
                "ascending_dag_violation_count"
            ],
        },
        "calibration": {
            "fresh_containment": _rate(
                sum(state["fresh_containment"]), len(state["fresh_containment"])
            ),
            "fresh_epsilon": _distribution(state["fresh_epsilon"]),
            "fresh_q_error": {
                **_distribution(fresh_q),
                "above_5_991464547107979": sum(value > Q95 for value in fresh_q),
                "above_9": sum(value > Q3 for value in fresh_q),
            },
            "aged_containment": _rate(
                sum(state["aged_containment"]), len(state["aged_containment"])
            ),
            "aged_radius": _distribution(state["aged_radius"]),
            "aged_q_error": {
                **_distribution(aged_q),
                "above_5_991464547107979": sum(value > Q95 for value in aged_q),
                "above_9": sum(value > Q3 for value in aged_q),
            },
            "fresh_fim": {
                "minimum_eigenvalue": _distribution(state["fim_min_eigenvalue"]),
                "condition_number": _distribution(state["fim_condition"]),
            },
            "online_q_innovation": {
                "accepted": _distribution(state["accepted_q"]),
                "rejected": _distribution(state["rejected_q"]),
            },
        },
        "rejected_candidate_offline_errors": _distribution(
            state["rejected_candidate_errors"]
        ),
        "strata": _finish_strata(state["strata"]),
    }
    return result


def _gate_records(summary: dict, baseline: Mapping, gates: Mapping) -> dict:
    complete = summary["variants"]["predictive_multistart"]
    paired_fresh = complete["paired"]["baseline_fresh"]
    both_fresh_changes = paired_fresh["both_fresh_error_change"]
    baseline_p95 = paired_fresh["both_fresh_baseline_errors"]["p95"]
    complete_p95 = paired_fresh["both_fresh_new_errors"]["p95"]
    records = {
        "maximum_published_error_m_strictly_below": {
            **complete["catastrophic"]["all_published"],
            "observed": complete["errors"]["all_published"]["maximum"],
            "threshold": gates["maximum_published_error_m_strictly_below"],
        },
        "maximum_fresh_error_m_strictly_below": {
            **complete["catastrophic"]["fresh"],
            "observed": complete["errors"]["fresh"]["maximum"],
            "threshold": gates["maximum_fresh_error_m_strictly_below"],
        },
        "paired_both_fresh_p95_must_not_worsen": {
            "numerator": both_fresh_changes["denominator"],
            "denominator": both_fresh_changes["denominator"],
            "observed_new_p95": complete_p95,
            "baseline_p95": baseline_p95,
        },
        "fresh_availability_max_drop_fraction": {
            "numerator": paired_fresh["transition_counts"]["fresh"],
            "denominator": paired_fresh["cohort_denominator"],
            "threshold": 1.0 - gates["fresh_availability_max_drop_fraction"],
        },
        "fresh_or_predicted_min_fraction": {
            **complete["availability"]["fresh_or_predicted"],
            "threshold": gates["fresh_or_predicted_min_fraction"],
        },
        "maximum_prediction_age_frames": {
            "numerator": complete["prediction"]["age_above_two_count"],
            "denominator": complete["output_counts"]["predicted"],
            "observed": complete["prediction"]["maximum_age"],
            "threshold": gates["maximum_prediction_age_frames"],
        },
        "qualification_anchor_violations_allowed": {
            "numerator": complete["integrity"]["nonfresh_anchor_use_count"],
            "denominator": complete["budgets"]["outputs"],
            "threshold": gates["qualification_anchor_violations_allowed"],
        },
        "current_frame_provenance_violations_allowed": {
            "numerator": complete["integrity"][
                "insufficient_provenance_count"
            ]
            + complete["integrity"]["provenance_mismatch_count"],
            "denominator": complete["budgets"]["outputs"],
            "threshold": gates["current_frame_provenance_violations_allowed"],
        },
        "ascending_dag_violations_allowed": {
            "numerator": complete["integrity"][
                "ascending_dag_violation_count"
            ],
            "denominator": complete["budgets"]["outputs"],
            "threshold": gates["ascending_dag_violations_allowed"],
        },
    }
    for name, record in records.items():
        if name.startswith("maximum_") and name.endswith("_strictly_below"):
            record["passed"] = (
                record["observed"] is not None
                and record["observed"] < record["threshold"]
            )
        elif name == "paired_both_fresh_p95_must_not_worsen":
            record["passed"] = (
                record["denominator"] > 0
                and record["observed_new_p95"] is not None
                and record["baseline_p95"] is not None
                and record["observed_new_p95"] <= record["baseline_p95"]
            )
        elif name in {
            "fresh_availability_max_drop_fraction",
            "fresh_or_predicted_min_fraction",
        }:
            record["passed"] = (
                record["denominator"] > 0
                and record["numerator"] / record["denominator"] >= record["threshold"]
            )
        elif name == "maximum_prediction_age_frames":
            record["passed"] = (
                record["observed"] is None
                or record["observed"] <= record["threshold"]
            )
        else:
            record["passed"] = record["numerator"] <= record["threshold"]
    if set(records) != set(gates):
        raise ValueError("analysis gates differ from predeclared protocol")
    return records


def aggregate_predictive_recovery(
    *,
    baseline_rows: Iterable[Mapping],
    development_rows: Iterable[Mapping],
    truth_data: Mapping,
    protocol: Mapping,
) -> dict:
    """Aggregate validated rows without treating repeated frames as trajectories."""
    variants = protocol.get("experiment", {}).get("variants")
    if variants != list(replay.DEVELOPMENT_VARIANTS):
        raise ValueError("variant contract differs from exact protocol")
    if protocol.get("gates") != replay.GATES:
        raise ValueError("gate contract differs from exact protocol")
    if protocol.get("estimator_constants") != replay.ESTIMATOR_CONSTANTS:
        raise ValueError("estimator constants differ from exact protocol")
    if protocol.get("ablation_contracts") != replay.ABLATION_CONTRACTS:
        raise ValueError("ablation contracts differ from exact protocol")
    input_audit = audit_input_limits(truth_data, component_limit=25.0)
    truth_commands = input_audit.pop("_commands")
    truth_positions = input_audit.pop("_truth_positions")
    expected_predecessor_keys = input_audit.pop("_expected_predecessor_keys")
    observed_predecessor_keys = set()
    baseline = {}
    baseline_counts = Counter()
    baseline_attempts = Counter()
    previous_baseline_order = None
    for row in baseline_rows:
        if row.get("graph_case") != "dynamic_dag_wnls":
            continue
        mapped = _map_baseline_row(row)
        key = mapped["key"]
        order = (key[1], key[0], key[2])
        if previous_baseline_order is not None and order <= previous_baseline_order:
            raise ValueError("baseline dynamic rows are duplicate or out of order")
        previous_baseline_order = order
        if key in baseline:
            raise ValueError("duplicate baseline exact key")
        baseline[key] = mapped
        baseline_counts[mapped["output_status"]] += 1
        baseline_attempts[mapped["attempt_status"]] += 1
    if not baseline:
        raise ValueError("baseline dynamic cohort is empty")
    expected_keys = sorted(baseline)
    states = {}
    for variant in variants:
        states[variant] = {
            "rows": 0,
            "attempt_counts": Counter(),
            "output_counts": Counter(),
            "fresh_errors": [],
            "published_errors": [],
            "fresh_containment": [],
            "fresh_epsilon": [],
            "aged_containment": [],
            "aged_radius": [],
            "fresh_q": [],
            "aged_q": [],
            "fim_min_eigenvalue": [],
            "fim_condition": [],
            "accepted_q": [],
            "rejected_q": [],
            "rejected_candidate_errors": [],
            "prediction_ages": [],
            "paired": _new_paired_state(),
            "mechanisms": {},
            "active_composition": Counter(),
            "reference_freshness": Counter(),
            "seen_base_ids": set(),
            "integrity": {
                "reference_violation_count": 0,
                "reference_violation_reasons": Counter(),
                "nonfresh_anchor_use_count": 0,
                "insufficient_provenance_count": 0,
                "provenance_mismatch_count": 0,
                "ascending_dag_violation_count": 0,
            },
            "strata": {
                name: defaultdict(_new_stratum)
                for name in ("depth", "squad", "time", "seed")
            },
            "streak": defaultdict(int),
            "maximum_unavailable_streak": 0,
        }
    expected_variant_index = 0
    key_index = 0
    for row in development_rows:
        _validate_row_schema(row)
        variant = row["variant"]
        if variant != variants[expected_variant_index]:
            raise ValueError("development variants are missing, extra, or out of order")
        key = _development_key(row)
        truth_position = truth_positions.get(
            (row["frame_index"], row["robot_id"])
        )
        if (
            truth_position is None
            or not np.allclose(
                np.asarray(row["offline_truth_position"], dtype=float),
                np.asarray(truth_position, dtype=float),
                rtol=0.0,
                atol=1e-12,
            )
        ):
            raise ValueError("offline truth differs from protocol-bound trajectory")
        if key_index >= len(expected_keys) or key != expected_keys[key_index]:
            raise ValueError("development exact keys are missing, extra, or out of order")
        key_index += 1
        if key_index == len(expected_keys):
            expected_variant_index += 1
            key_index = 0
            if expected_variant_index == len(variants):
                expected_variant_index = len(variants) - 1
                key_index = len(expected_keys)
        state = states[variant]
        compact_output = {
            "attempt_status": row["attempt_status"],
            "output_status": row["output_status"],
            "offline_error_norm": row["offline_error_norm"],
            "prediction_age": row["prediction_age"],
        }
        _update_paired(state["paired"], baseline[key], compact_output)
        if key in _MECHANISM_KEYS.values():
            state["mechanisms"][key] = compact_output
        state["rows"] += 1
        state["attempt_counts"][row["attempt_status"]] += 1
        status = row["output_status"]
        state["output_counts"][status] += 1
        error = row["offline_error_norm"]
        if status == "fresh":
            state["fresh_errors"].append(error)
            state["published_errors"].append(error)
            state["fresh_containment"].append(row["offline_fresh_containment"])
            state["fresh_epsilon"].append(row["fresh_epsilon"])
            state["fresh_q"].append(row["offline_fresh_q_error"])
            covariance = np.asarray(row["fresh_modeled_covariance"], dtype=float)
            if covariance.shape != (2, 2) or not np.all(np.isfinite(covariance)):
                raise ValueError("fresh covariance must be finite 2x2")
            try:
                fim = np.linalg.inv(covariance)
                eigenvalues = np.linalg.eigvalsh(fim)
            except np.linalg.LinAlgError as error_value:
                raise ValueError("fresh covariance must be invertible") from error_value
            if np.min(eigenvalues) <= 0:
                raise ValueError("fresh FIM must be positive definite")
            state["fim_min_eigenvalue"].append(float(np.min(eigenvalues)))
            state["fim_condition"].append(float(np.max(eigenvalues) / np.min(eigenvalues)))
        elif status == "predicted":
            state["published_errors"].append(error)
            state["aged_containment"].append(row["offline_aged_radius_containment"])
            state["aged_radius"].append(row["aged_modeled_radius"])
            state["aged_q"].append(row["offline_aged_q_error"])
            state["prediction_ages"].append(row["prediction_age"])
        streak_key = (row["seed"], row["robot_id"])
        if status == "unavailable":
            state["streak"][streak_key] += 1
            state["maximum_unavailable_streak"] = max(
                state["maximum_unavailable_streak"], state["streak"][streak_key]
            )
        else:
            state["streak"][streak_key] = 0
        expected_command = (
            None
            if row["frame_index"] == 0
            else truth_commands.get((row["frame_index"] - 1, row["robot_id"]))
        )
        if row["frame_index"] == 0:
            if (
                row["applied_command_source_frame"] is not None
                or row["applied_command"] is not None
            ):
                raise ValueError("frame-zero prediction command must be null")
        elif (
            row["applied_command_source_frame"] != row["frame_index"] - 1
            or expected_command is None
            or row["applied_command"] != list(expected_command)
        ):
            raise ValueError("applied command does not match predecessor opt.result")
        if row["frame_index"] > 0:
            observed_predecessor_keys.add((row["frame_index"], row["robot_id"]))
        active = row["active_references"]
        if not isinstance(active, list):
            raise ValueError("active references must be a list")
        kinds = Counter()
        active_keys = []
        for item in active:
            if (
                not isinstance(item, list)
                or len(item) != 2
                or item[0] not in {"base", "uav"}
                or isinstance(item[1], bool)
                or not isinstance(item[1], int)
            ):
                raise ValueError("active reference differs from exact key schema")
            kinds[item[0]] += 1
            active_keys.append((item[0], item[1]))
            if item[0] == "uav" and item[1] >= row["robot_id"]:
                state["integrity"]["ascending_dag_violation_count"] += 1
                raise ValueError("nonascending DAG reference use")
        if len(active_keys) != len(set(active_keys)):
            raise ValueError("active reference keys must be unique")
        state["active_composition"][
            f"base={kinds['base']},uav={kinds['uav']}"
        ] += 1
        qualified = protocol["ablation_contracts"][variant][
            "fresh_reference_qualification"
        ]
        evidence = row["reference_evidence"]
        if not isinstance(evidence, list):
            raise ValueError("reference evidence must be a list")
        decoded_evidence = []
        for compact_item in evidence:
            if (
                not isinstance(compact_item, list)
                or len(compact_item) != len(replay.REFERENCE_EVIDENCE_FIELDS)
            ):
                raise ValueError("reference evidence differs from exact schema")
            item = dict(zip(replay.REFERENCE_EVIDENCE_FIELDS, compact_item))
            decoded_evidence.append(item)
            item_provenance = item["base_anchor_provenance"]
            if (
                item["reference_kind"] not in {"base", "uav"}
                or isinstance(item["reference_id"], bool)
                or not isinstance(item["reference_id"], int)
                or not isinstance(item["used"], bool)
                or not isinstance(item_provenance, list)
                or any(
                    isinstance(value, bool) or not isinstance(value, int)
                    for value in item_provenance
                )
                or item_provenance != sorted(set(item_provenance))
            ):
                raise ValueError("reference evidence value differs from exact schema")
            state["reference_freshness"][str(item["current_freshness"])] += 1
            if item["reference_kind"] == "base":
                state["seen_base_ids"].add(item["reference_id"])
        evidence_keys = [
            (item["reference_kind"], item["reference_id"])
            for item in decoded_evidence
        ]
        if (
            len(evidence_keys) != len(set(evidence_keys))
            or evidence_keys
            != sorted(evidence_keys, key=lambda key: (key[0] == "uav", key[1]))
            or not set(active_keys).issubset(evidence_keys)
        ):
            raise ValueError("reference evidence keys/order differ from active contract")
        freshness = row["reference_freshness"]
        expected_freshness = [
            [
                item["reference_kind"],
                item["reference_id"],
                item["current_freshness"],
            ]
            for item in decoded_evidence
        ]
        if freshness != expected_freshness:
            raise ValueError("reference freshness differs from compact evidence")
        violations = row["reference_violations"]
        if not isinstance(violations, list):
            raise ValueError("reference violations must be a list")
        decoded_violations = []
        for compact_violation in violations:
            if (
                not isinstance(compact_violation, list)
                or len(compact_violation) != len(replay.VIOLATION_FIELDS)
            ):
                raise ValueError("reference violation differs from exact schema")
            violation = dict(zip(replay.VIOLATION_FIELDS, compact_violation))
            if not isinstance(violation["reason"], str):
                raise ValueError("reference violation reason must be a string")
            decoded_violations.append(violation)
            state["integrity"]["reference_violation_count"] += 1
            state["integrity"]["reference_violation_reasons"][
                violation["reason"]
            ] += 1
        evidence_by_key = {
            (item["reference_kind"], item["reference_id"]): item
            for item in decoded_evidence
        }
        expected_diagnostic_violations = [
            {
                "reference_kind": kind,
                "reference_id": identifier,
                "reason": "stale_or_predicted_anchor_used",
            }
            for kind, identifier in active_keys
            if kind == "uav"
            and evidence_by_key[(kind, identifier)]["current_freshness"] != "fresh"
        ]
        if variant == "prediction_expiry":
            if decoded_violations != expected_diagnostic_violations:
                raise ValueError(
                    "prediction-expiry violations differ from derived stale anchors"
                )
        elif decoded_violations:
            raise ValueError("qualified variant contains reference violation")
        state["integrity"]["nonfresh_anchor_use_count"] += len(
            expected_diagnostic_violations
        )
        if qualified and expected_diagnostic_violations:
            raise ValueError("qualification-enabled variant used aged UAV anchor")
        provenance = row["base_anchor_provenance"]
        attempted_provenance = row["attempt_base_anchor_provenance"]
        if (
            not isinstance(provenance, list)
            or not isinstance(attempted_provenance, list)
            or any(
                isinstance(value, bool) or not isinstance(value, int)
                for value in provenance + attempted_provenance
            )
            or provenance != sorted(set(provenance))
            or attempted_provenance != sorted(set(attempted_provenance))
        ):
            raise ValueError("base-anchor provenance differs from exact schema")
        used = {
            (item["reference_kind"], item["reference_id"])
            for item in decoded_evidence
            if item["used"] is True
        }
        if row["attempt_status"] == "reference_unavailable":
            if used:
                raise ValueError("reference-unavailable attempt supplied references")
        elif used != set(active_keys):
            raise ValueError("solver-used references differ from active references")
        derived_roots = sorted(
            {
                root
                for key in active_keys
                for root in evidence_by_key[key]["base_anchor_provenance"]
            }
        )
        attempted_mismatch = attempted_provenance != derived_roots
        if attempted_mismatch:
            state["integrity"]["provenance_mismatch_count"] += 1
            diagnostic_exception = (
                variant == "prediction_expiry"
                and bool(expected_diagnostic_violations)
                and set(attempted_provenance).issubset(state["seen_base_ids"])
            )
            if not diagnostic_exception:
                raise ValueError(
                    "attempt provenance differs from current active chain"
                )
        if status == "fresh" and len(provenance) < 2:
            state["integrity"]["insufficient_provenance_count"] += 1
            if qualified:
                raise ValueError("fresh output lacks two-base current provenance")
        if status == "fresh":
            provenance_mismatch = (
                used != set(active_keys)
                or provenance != attempted_provenance
            )
            if provenance_mismatch:
                state["integrity"]["provenance_mismatch_count"] += 1
                if qualified:
                    raise ValueError(
                        "fresh publication provenance differs from current used chain"
                    )
        elif provenance:
            raise ValueError("nonfresh publication must not expose provenance")
        candidate_records = list(_candidate_records(row))
        selected_source = row["selected_candidate_source"]
        accepted_candidates = [
            source
            for source, accepted, _, _ in candidate_records
            if accepted
        ]
        if row["attempt_status"] == "accepted":
            if (
                not isinstance(selected_source, str)
                or not selected_source
                or selected_source not in accepted_candidates
            ):
                raise ValueError(
                    "accepted attempt lacks its selected accepted candidate"
                )
        elif selected_source is not None or accepted_candidates:
            raise ValueError(
                "nonaccepted attempt exposes selected or accepted candidate"
            )
        for _, accepted, q_innov, candidate_error in candidate_records:
            if q_innov is not None:
                state["accepted_q" if accepted else "rejected_q"].append(q_innov)
            if not accepted and candidate_error is not None:
                state["rejected_candidate_errors"].append(candidate_error)
        depth = str(row["squad_local_index"])
        squad = str((row["robot_id"] - 1) // 7 + 1)
        time_bin = str(int(row["frame_index"] * 0.5 // 50))
        labels = {
            "depth": depth,
            "squad": squad,
            "time": time_bin,
            "seed": str(row["seed"]),
        }
        for family, label in labels.items():
            stratum = state["strata"][family][label]
            stratum["rows"] += 1
            stratum["attempt_counts"][row["attempt_status"]] += 1
            stratum["output_counts"][status] += 1
            if status == "fresh":
                stratum["fresh_errors"].append(error)
            if status in {"fresh", "predicted"}:
                stratum["published_errors"].append(error)
    expected_total = len(variants) * len(expected_keys)
    if sum(state["rows"] for state in states.values()) != expected_total:
        raise ValueError("development exact key set is incomplete")
    result = {
        "schema_id": ANALYSIS_SCHEMA_ID,
        "evidence_class": protocol["experiment"].get(
            "evidence_class", "hermetic_non_evidence_only"
        ),
        "paper_gate": "CLOSED",
        "baseline": {
            "budgets": {
                "attempted": len(baseline),
                "outputs": len(baseline),
            },
            "attempt_counts": {
                name: baseline_attempts.get(name, 0)
                for name in ("accepted", "failed", "invalid")
            },
            "output_counts": {
                name: baseline_counts.get(name, 0) for name in _BASELINE_OUTPUTS
            },
            "fresh_errors": _distribution(
                item["error"]
                for item in baseline.values()
                if item["output_status"] == "fresh"
            ),
            "all_published_errors": _distribution(
                item["error"]
                for item in baseline.values()
                if item["output_status"] != "unavailable"
            ),
        },
        "variants": {
            variant: _finish_variant(states[variant])
            for variant in variants
        },
        "input_limits": input_audit,
        "prediction_transition_audit": {
            "unique_predecessor_rows": len(observed_predecessor_keys),
            "expected_unique_predecessor_rows": len(expected_predecessor_keys),
            "coverage_complete": observed_predecessor_keys
            == expected_predecessor_keys,
            "coverage_fraction": (
                len(observed_predecessor_keys) / len(expected_predecessor_keys)
                if expected_predecessor_keys
                else None
            ),
        },
        "mechanism_records": resolve_mechanism_records(
            baseline,
            {variant: state["mechanisms"] for variant, state in states.items()},
        ),
    }
    result["gates"] = _gate_records(result, baseline, protocol["gates"])
    result["all_estimator_gates_passed"] = all(
        gate["passed"] for gate in result["gates"].values()
    )
    return result


def _iter_gzip_json_rows(
    path: Path,
    *,
    expected_identity: Mapping,
    decompressed_digest=None,
) -> Iterable[dict]:
    descriptor = os.open(
        path,
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0),
    )
    expected_state = tuple(
        expected_identity[name]
        for name in ("device", "inode", "size", "mtime_ns")
    )
    try:
        before = os.fstat(descriptor)
        if (
            before.st_dev,
            before.st_ino,
            before.st_size,
            before.st_mtime_ns,
        ) != expected_state:
            raise ValueError("gzip process identity changed before streaming")
        with os.fdopen(os.dup(descriptor), "rb") as compressed_file:
            with gzip.GzipFile(fileobj=compressed_file, mode="rb") as stream:
                for line_number, line in enumerate(stream, 1):
                    if decompressed_digest is not None:
                        decompressed_digest.update(line)
                    try:
                        row = json.loads(
                            line,
                            parse_constant=lambda value: (_ for _ in ()).throw(
                                ValueError(f"non-finite constant {value}")
                            ),
                            parse_float=lambda value: (
                                float(value)
                                if math.isfinite(float(value))
                                else (_ for _ in ()).throw(
                                    ValueError(f"non-finite float {value}")
                                )
                            ),
                        )
                    except Exception as error:
                        raise ValueError(f"invalid JSON row {line_number}") from error
                    if not isinstance(row, dict):
                        raise ValueError(f"JSON row {line_number} is not an object")
                    replay._native_json(row)
                    yield row
        after = os.fstat(descriptor)
        if (
            after.st_dev,
            after.st_ino,
            after.st_size,
            after.st_mtime_ns,
        ) != expected_state:
            raise ValueError("gzip process identity changed while streaming")
    finally:
        os.close(descriptor)


def _hash_decompressed(path: Path, expected_identity: Mapping) -> str:
    digest = hashlib.sha256()
    for _ in _iter_gzip_json_rows(
        path,
        expected_identity=expected_identity,
        decompressed_digest=digest,
    ):
        pass
    return digest.hexdigest()


def _ordered_baseline_rows(rows: Iterable[Mapping]) -> Iterable[Mapping]:
    previous = None
    for row in rows:
        graph_case = row.get("graph_case")
        seed = row.get("seed")
        frame = row.get("frame_index")
        robot = row.get("robot_id")
        if (
            not isinstance(graph_case, str)
            or any(
                isinstance(value, bool) or not isinstance(value, int)
                for value in (frame, seed, robot)
            )
        ):
            raise ValueError("baseline native order fields are invalid")
        order = (frame, seed, graph_case, robot)
        if previous is not None and order <= previous:
            raise ValueError("baseline native frame/seed/graph/robot order drift")
        previous = order
        yield row


def _authorize_inputs(
    *,
    baseline_process_path: Path,
    development_manifest_path: Path,
    protocol_path: Path,
    expected_baseline_sha256: str,
    output_root: Path,
) -> dict:
    protocol_payload, protocol_identity = replay._read_trusted_bytes(protocol_path)
    protocol = replay._parse_json_object(protocol_payload, protocol_path)
    sources = protocol.get("sources")
    if not isinstance(sources, dict):
        raise ValueError("protocol sources must be an object")
    if protocol.get("schema_id") == replay.PROTOCOL_SCHEMA_ID:
        raw_invocation = replay.production_invocation_contract()["registered_replay"]
        replay._validate_protocol_shape(
            protocol,
            protocol_path=protocol_path,
            output_root=Path(raw_invocation["output_root"]),
            run_seeds=tuple(raw_invocation["range_noise_seeds"]),
            max_frames=raw_invocation["max_frames"],
        )
        declaration = protocol["invocations"]["registered_analyzer"]
        if (
            Path(declaration["development_manifest_path"])
            != development_manifest_path
            or Path(declaration["output_root"]) != output_root
            or declaration["expected_baseline_sha256"] != expected_baseline_sha256
            or Path(sources["baseline_process"]["path"]) != baseline_process_path
        ):
            raise ValueError("runtime analyzer arguments differ from registered invocation")
        expected_command = replay.canonical_analyzer_argv(
            baseline_process_path=baseline_process_path,
            development_manifest_path=development_manifest_path,
            protocol_token=replay.PRODUCTION_PROTOCOL_TOKEN,
            expected_baseline_sha256=expected_baseline_sha256,
            output_root=output_root,
        )
        if protocol["commands"]["registered_analyzer"] != expected_command:
            raise ValueError("registered analyzer command differs from runtime")
        replay.verify_implementation_parent_provenance(
            project_root=Path(__file__).resolve().parents[2],
            implementation_parent_commit=protocol["implementation_parent_commit"],
            sources=sources,
        )
        production = True
    elif protocol.get("schema_id") == ANALYZER_TEST_SCHEMA_ID:
        exact = {
            "schema_id",
            "sources",
            "experiment",
            "ablation_contracts",
            "estimator_constants",
            "gates",
            "raw_schema",
            "analysis_schema",
            "disk_contract",
            "invocations",
        }
        if set(protocol) != exact or set(sources) != {
            "truth_data",
            "baseline_process",
        }:
            raise ValueError("analyzer test protocol differs from exact non-evidence schema")
        if (
            protocol["raw_schema"] != replay.RAW_SCHEMA_DECLARATION
            or protocol["analysis_schema"] != replay.ANALYSIS_SCHEMA
            or protocol["disk_contract"] != replay.DISK_CONTRACT
            or protocol["experiment"].get("evidence_class")
            != "hermetic_non_evidence_only"
        ):
            raise ValueError("analyzer test protocol contract drift")
        if protocol["invocations"] != {
            "analyzer_test": {
                "kind": "analyzer_test_only",
                "development_manifest_path": str(development_manifest_path),
                "output_root": str(output_root),
                "expected_baseline_sha256": expected_baseline_sha256,
            }
        }:
            raise ValueError("analyzer test invocation differs from runtime")
        if Path(sources["baseline_process"]["path"]) != baseline_process_path:
            raise ValueError("test baseline path differs from runtime")
        production = False
    else:
        raise ValueError("protocol does not authorize analyzer execution")
    if (
        not isinstance(expected_baseline_sha256, str)
        or len(expected_baseline_sha256) != 64
        or sources["baseline_process"]["sha256"] != expected_baseline_sha256
    ):
        raise ValueError("baseline hash differs from exact protocol")
    identities = {}
    payloads = {}
    for name, declaration in sources.items():
        if (
            not isinstance(declaration, dict)
            or set(declaration) != {"path", "sha256"}
        ):
            raise ValueError("protocol source declaration differs from exact schema")
        payload, identity = replay._read_trusted_bytes(
            replay._absolute(Path(declaration["path"])),
            expected_sha256=declaration["sha256"],
            capture_payload=name == "truth_data",
        )
        identities[name] = identity
        if payload is not None:
            payloads[name] = payload
    manifest_payload, manifest_identity = replay._read_trusted_bytes(
        development_manifest_path
    )
    manifest = replay._parse_json_object(manifest_payload, development_manifest_path)
    test_manifest_fields = {
        "status",
        "schema_id",
        "raw_schema",
        "rows_written",
        "expected_rows",
        "compressed_process_sha256",
        "decompressed_process_sha256",
    }
    if (
        manifest.get("status") != "completed"
        or manifest.get("schema_id") != replay.RAW_SCHEMA_ID
        or manifest.get("raw_schema") != replay.RAW_SCHEMA_DECLARATION
        or manifest.get("rows_written") != manifest.get("expected_rows")
    ):
        raise ValueError("development replay manifest is not complete and exact")
    production_manifest_fields = {
        "status",
        "schema_id",
        "raw_schema",
        "protocol_schema_id",
        "protocol_id",
        "evidence_class",
        "protocol_identity",
        "protocol_parent",
        "selected_invocation",
        "source_identities",
        "output_root",
        "run_seeds",
        "max_frames",
        "started_at",
        "ended_at",
        "rows_written",
        "expected_rows",
        "free_bytes_before",
        "free_bytes_after",
        "allocated_bytes",
        "raw_bundle_cap_bytes",
        "compressed_process_sha256",
        "decompressed_process_sha256",
    }
    if production:
        if set(manifest) != production_manifest_fields:
            raise ValueError("completed replay manifest differs from exact schema")
        expected_rows = (
            len(replay.DEVELOPMENT_VARIANTS)
            * len(replay.PRODUCTION_RANGE_NOISE_SEEDS)
            * 500
            * 14
        )
        if (
            manifest["selected_invocation"] != "registered_replay"
            or manifest["protocol_schema_id"] != protocol["schema_id"]
            or manifest["protocol_id"] != protocol["protocol_id"]
            or manifest["evidence_class"]
            != protocol["experiment"]["evidence_class"]
            or manifest["protocol_identity"] != protocol_identity
            or manifest["protocol_parent"] != str(protocol_path.parent)
            or manifest["source_identities"] != identities
            or manifest["run_seeds"]
            != list(replay.PRODUCTION_RANGE_NOISE_SEEDS)
            or manifest["max_frames"] is not None
            or manifest["rows_written"] != expected_rows
            or manifest["expected_rows"] != expected_rows
            or manifest["raw_bundle_cap_bytes"] != replay.RAW_BUNDLE_CAP_BYTES
            or Path(manifest["output_root"])
            != Path(raw_invocation["output_root"])
            or Path(manifest["output_root"])
            != development_manifest_path.parent
        ):
            raise ValueError("completed replay manifest provenance drift")
    elif set(manifest) != test_manifest_fields:
        raise ValueError("test replay manifest differs from exact schema")
    raw_path = development_manifest_path.parent / replay.RAW_PROCESS_NAME
    raw_payload, raw_identity = replay._read_trusted_bytes(
        raw_path,
        expected_sha256=manifest["compressed_process_sha256"],
        capture_payload=False,
    )
    decompressed_digest = _hash_decompressed(raw_path, raw_identity)
    if decompressed_digest != manifest.get("decompressed_process_sha256"):
        raise ValueError("development decompressed process hash mismatch")
    if production:
        replay._lstat_components(raw_path, leaf_required=True)
        raw_metadata = raw_path.lstat()
        if (
            raw_metadata.st_dev,
            raw_metadata.st_ino,
            raw_metadata.st_size,
            raw_metadata.st_mtime_ns,
        ) != (
            raw_identity["device"],
            raw_identity["inode"],
            raw_identity["size"],
            raw_identity["mtime_ns"],
        ):
            raise ValueError("production raw path changed before disk audit")
        _validate_production_disk_metrics(
            manifest,
            actual_allocated_bytes=raw_metadata.st_blocks * 512,
        )
    replay._validate_paths(
        Path(__file__).resolve().parents[2],
        protocol_path,
        [
            *[Path(item["path"]) for item in identities.values()],
            development_manifest_path,
            raw_path,
        ],
        output_root,
        output_allocated=False,
    )
    return {
        "protocol": protocol,
        "protocol_identity": protocol_identity,
        "source_identities": identities,
        "source_payloads": payloads,
        "manifest": manifest,
        "manifest_identity": manifest_identity,
        "raw_path": raw_path,
        "raw_identity": raw_identity,
        "production": production,
    }


def _open_compact_file(transaction: dict, name: str) -> int:
    descriptor = os.open(
        name,
        os.O_RDWR
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0),
        0o600,
        dir_fd=transaction["root_fd"],
    )
    return replay._adopt_or_close(
        transaction,
        descriptor,
        rollback=lambda error: replay._rollback_created_entry(
            directory_fd=transaction["root_fd"],
            name=name,
            descriptor=descriptor,
            original_error=error,
            label=f"compact {name}",
        ),
    )


def _write_descriptor(descriptor: int, payload: bytes) -> dict:
    offset = 0
    while offset < len(payload):
        written = os.write(descriptor, payload[offset:])
        if written <= 0:
            raise OSError("compact write made no progress")
        offset += written
    os.fsync(descriptor)
    identity = replay._fd_identity(descriptor)
    os.lseek(descriptor, 0, os.SEEK_SET)
    digest = hashlib.sha256()
    while True:
        chunk = os.read(descriptor, 65536)
        if not chunk:
            break
        digest.update(chunk)
    if digest.hexdigest() != hashlib.sha256(payload).hexdigest():
        raise ValueError("compact output bytes changed after write")
    return {**identity, "sha256": digest.hexdigest()}


def _verify_compact_file(
    transaction: Mapping,
    name: str,
    descriptor: int,
    expected: Mapping,
) -> None:
    if not replay._same_file_identity(replay._fd_identity(descriptor), expected):
        raise ValueError("compact output descriptor identity changed")
    linked = replay._read_entry_identity(transaction["root_fd"], name)
    if not replay._same_pinned_identity(linked, expected):
        raise ValueError("compact output path differs from retained descriptor")
    os.lseek(descriptor, 0, os.SEEK_SET)
    digest = hashlib.sha256()
    while True:
        chunk = os.read(descriptor, 65536)
        if not chunk:
            break
        digest.update(chunk)
    if digest.hexdigest() != expected["sha256"]:
        raise ValueError("compact output hash changed")


def _allocated_sum(descriptors: Iterable[int]) -> int:
    return sum(os.fstat(descriptor).st_blocks * 512 for descriptor in descriptors)


def _validate_production_summary(summary: Mapping) -> None:
    for name, record in summary["mechanism_records"].items():
        if record["baseline"] is None or any(
            value is None for value in record["variants"].values()
        ):
            raise ValueError(f"missing exact mechanism record: {name}")
    limits = summary["input_limits"]
    if (
        limits["unique_physical_rows"] != 7000
        or limits["component_bound_violations"] != 243
    ):
        raise ValueError("production input-limit audit differs from 243/7000")
    transition = summary["prediction_transition_audit"]
    if (
        transition["unique_predecessor_rows"] != 6986
        or transition["expected_unique_predecessor_rows"] != 6986
        or transition["coverage_complete"] is not True
    ):
        raise ValueError(
            "production prediction-transition coverage differs from 6986/6986"
        )


def _rewrite_stage(stage: dict, payload: bytes) -> None:
    descriptor = stage["fd"]
    os.ftruncate(descriptor, 0)
    os.lseek(descriptor, 0, os.SEEK_SET)
    offset = 0
    while offset < len(payload):
        written = os.write(descriptor, payload[offset:])
        if written <= 0:
            raise OSError("terminal manifest rewrite made no progress")
        offset += written
    os.fsync(descriptor)
    stage.update(replay._fd_identity(descriptor))


def _markdown(summary: Mapping) -> str:
    def rounded(value):
        return "n/a" if value is None else f"{float(value):.6g}"

    def distribution(value):
        return " / ".join(
            (
                rounded(value["p50"]),
                rounded(value["p95"]),
                rounded(value["p99"]),
                rounded(value["maximum"]),
                str(value["denominator"]),
            )
        )

    def counts(value, names):
        return " / ".join(f"{name}={value.get(name, 0)}" for name in names)

    def compact_mapping(value):
        if not value:
            return "none"
        return ", ".join(f"{name}={count}" for name, count in value.items())

    attempt_names = tuple(replay.ATTEMPT_STATUSES)
    output_names = tuple(replay.OUTPUT_STATUSES)
    lines = [
        "# Predictive WNLS Stage 1 development analysis",
        "",
        "This is paired single-trajectory development evidence; the paper gate remains CLOSED.",
        "",
        "All distribution cells use `p50 / p95 / p99 / max / n`.",
        "",
        "## Baseline budgets and tails",
        "",
        "| Attempted / outputs | Attempt counts | Output counts | "
        "Fresh errors (p50 / p95 / p99 / max / n) | "
        "All-published errors (p50 / p95 / p99 / max / n) |",
        "|---:|---|---|---|---|",
    ]
    baseline = summary["baseline"]
    lines.append(
        f"| {baseline['budgets']['attempted']} / "
        f"{baseline['budgets']['outputs']} | "
        f"{counts(baseline['attempt_counts'], ('accepted', 'failed', 'invalid'))} | "
        f"{counts(baseline['output_counts'], ('fresh', 'legacy_published', 'unavailable'))} | "
        f"{distribution(baseline['fresh_errors'])} | "
        f"{distribution(baseline['all_published_errors'])} |"
    )
    lines.extend(
        [
            "",
            "## Attempt counts and output tails",
            "",
            "| Variant | Attempted / outputs | Attempt counts | Output counts | "
            "Fresh errors (p50 / p95 / p99 / max / n) | "
            "All-published errors (p50 / p95 / p99 / max / n) |",
            "|---|---:|---|---|---|---|",
        ]
    )
    for variant in replay.DEVELOPMENT_VARIANTS:
        item = summary["variants"][variant]
        lines.append(
            f"| `{variant}` | {item['budgets']['attempted']} / "
            f"{item['budgets']['outputs']} | "
            f"{counts(item['attempt_counts'], attempt_names)} | "
            f"{counts(item['output_counts'], output_names)} | "
            f"{distribution(item['errors']['fresh'])} | "
            f"{distribution(item['errors']['all_published'])} |"
        )
    lines.extend(
        [
            "",
            "## Paired cohort audits",
            "",
            "| Variant | Baseline-published transitions | "
            "Both-published error change (p50 / p95 / p99 / max / n) | "
            "Baseline-published attrition | Attrition baseline errors "
            "(p50 / p95 / p99 / max / n) |",
            "|---|---|---|---:|---|",
        ]
    )
    for variant in replay.DEVELOPMENT_VARIANTS:
        published = summary["variants"][variant]["paired"][
            "baseline_published"
        ]
        attrition = published["newly_unavailable"]
        lines.append(
            f"| `{variant}` | "
            f"{counts(published['transition_counts'], output_names)} "
            f"(n={published['cohort_denominator']}) | "
            f"{distribution(published['both_published_error_change'])} | "
            f"{attrition['count']}/{published['cohort_denominator']} "
            f"(catastrophic={attrition['catastrophic_count']}) | "
            f"{distribution(attrition['baseline_errors'])} |"
        )
    lines.extend(
        [
            "",
            "| Variant | Baseline-fresh transitions | "
            "Both-fresh change (p50 / p95 / p99 / max / n) | "
            "Both-fresh baseline / new errors | "
            "Excluded baseline-error tails (predicted / unavailable / all) |",
            "|---|---|---|---|---|",
        ]
    )
    for variant in replay.DEVELOPMENT_VARIANTS:
        fresh = summary["variants"][variant]["paired"]["baseline_fresh"]
        downgraded = fresh["downgraded_to_predicted"]
        unavailable = fresh["newly_unavailable"]
        excluded = fresh["excluded_from_both_fresh"]
        lines.append(
            f"| `{variant}` | "
            f"{counts(fresh['transition_counts'], output_names)} "
            f"(n={fresh['cohort_denominator']}) | "
            f"{distribution(fresh['both_fresh_error_change'])} | "
            f"{distribution(fresh['both_fresh_baseline_errors'])} / "
            f"{distribution(fresh['both_fresh_new_errors'])} | "
            f"{distribution(downgraded['baseline_errors'])} / "
            f"{distribution(unavailable['baseline_errors'])} / "
            f"{distribution(excluded['baseline_errors'])}; "
            f"counts={downgraded['count']}/{unavailable['count']}/"
            f"{excluded['count']}; catastrophic="
            f"{downgraded['catastrophic_count']}/"
            f"{unavailable['catastrophic_count']}/"
            f"{excluded['catastrophic_count']} |"
        )
    lines.extend(
        [
            "",
            "## Catastrophic errors and availability",
            "",
            "| Variant | Fresh catastrophic | All-published catastrophic | "
            "Fresh availability | Fresh-or-predicted availability |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for variant in replay.DEVELOPMENT_VARIANTS:
        item = summary["variants"][variant]
        catastrophic = item["catastrophic"]
        availability = item["availability"]
        lines.append(
            f"| `{variant}` | "
            f"{catastrophic['fresh']['numerator']}/"
            f"{catastrophic['fresh']['denominator']} | "
            f"{catastrophic['all_published']['numerator']}/"
            f"{catastrophic['all_published']['denominator']} | "
            f"{availability['fresh']['numerator']}/"
            f"{availability['fresh']['denominator']} | "
            f"{availability['fresh_or_predicted']['numerator']}/"
            f"{availability['fresh_or_predicted']['denominator']} |"
        )
    lines.extend(
        [
            "",
            "## Prediction lifecycle and integrity",
            "",
            "| Variant | Maximum prediction age | Age > 2 | "
            "Maximum unavailable streak | Reference violations | "
            "Nonfresh / insufficient / provenance / DAG violations |",
            "|---|---:|---:|---:|---|---|",
        ]
    )
    for variant in replay.DEVELOPMENT_VARIANTS:
        item = summary["variants"][variant]
        prediction = item["prediction"]
        integrity = item["integrity"]
        lines.append(
            f"| `{variant}` | {rounded(prediction['maximum_age'])} | "
            f"{prediction['age_above_two_count']}/"
            f"{item['output_counts']['predicted']} | "
            f"{prediction['maximum_unavailable_streak']} | "
            f"{integrity['reference_violation_count']}/"
            f"{item['budgets']['outputs']} "
            f"({compact_mapping(integrity['reference_violation_reasons'])}) | "
            f"{integrity['nonfresh_anchor_use_count']} / "
            f"{integrity['insufficient_provenance_count']} / "
            f"{integrity['provenance_mismatch_count']} / "
            f"{integrity['ascending_dag_violation_count']} "
            f"(n={item['budgets']['outputs']}) |"
        )
    lines.extend(
        [
            "",
            "## Reference summaries",
            "",
            "| Variant | Active-set composition | Reference freshness counts |",
            "|---|---|---|",
        ]
    )
    for variant in replay.DEVELOPMENT_VARIANTS:
        references = summary["variants"][variant]["references"]
        lines.append(
            f"| `{variant}` | "
            f"{compact_mapping(references['active_set_composition'])} | "
            f"{compact_mapping(references['freshness_counts'])} |"
        )
    lines.extend(
        [
            "",
            "## FIM, epsilon, radius, and q diagnostics",
            "",
            "Calibration denominators are reported in every containment ratio "
            "and distribution cell.",
            "",
            "| Variant | Fresh containment | Fresh epsilon | Fresh q "
            "(>5.991 / >9) | Aged containment | Aged radius | "
            "Aged q (>5.991 / >9) |",
            "|---|---:|---|---|---:|---|---|",
        ]
    )
    for variant in replay.DEVELOPMENT_VARIANTS:
        calibration = summary["variants"][variant]["calibration"]
        fresh_q = calibration["fresh_q_error"]
        aged_q = calibration["aged_q_error"]
        lines.append(
            f"| `{variant}` | "
            f"{calibration['fresh_containment']['numerator']}/"
            f"{calibration['fresh_containment']['denominator']} | "
            f"{distribution(calibration['fresh_epsilon'])} | "
            f"{distribution(fresh_q)} "
            f"({fresh_q['above_5_991464547107979']} / "
            f"{fresh_q['above_9']}) | "
            f"{calibration['aged_containment']['numerator']}/"
            f"{calibration['aged_containment']['denominator']} | "
            f"{distribution(calibration['aged_radius'])} | "
            f"{distribution(aged_q)} "
            f"({aged_q['above_5_991464547107979']} / "
            f"{aged_q['above_9']}) |"
        )
    lines.extend(
        [
            "",
            "| Variant | FIM minimum eigenvalue | FIM condition number | "
            "q innovation accepted | q innovation rejected | "
            "Rejected candidate offline errors |",
            "|---|---|---|---|---|---|",
        ]
    )
    for variant in replay.DEVELOPMENT_VARIANTS:
        item = summary["variants"][variant]
        calibration = item["calibration"]
        lines.append(
            f"| `{variant}` | "
            f"{distribution(calibration['fresh_fim']['minimum_eigenvalue'])} | "
            f"{distribution(calibration['fresh_fim']['condition_number'])} | "
            f"{distribution(calibration['online_q_innovation']['accepted'])} | "
            f"{distribution(calibration['online_q_innovation']['rejected'])} | "
            f"{distribution(item['rejected_candidate_offline_errors'])} |"
        )
    lines.extend(
        [
            "",
            "## Stratified diagnostics",
            "",
            "| Variant | Family | Group | Rows | Attempt counts | Output counts | "
            "Fresh errors | All-published errors |",
            "|---|---|---|---:|---|---|---|---|",
        ]
    )
    for variant in replay.DEVELOPMENT_VARIANTS:
        strata = summary["variants"][variant]["strata"]
        for family, groups in strata.items():
            for group, item in groups.items():
                lines.append(
                    f"| `{variant}` | `{family}` | `{group}` | "
                    f"{item['rows']} | "
                    f"{counts(item['attempt_counts'], attempt_names)} | "
                    f"{counts(item['output_counts'], output_names)} | "
                    f"{distribution(item['fresh_errors'])} | "
                    f"{distribution(item['all_published_errors'])} |"
                )
    transition = summary["prediction_transition_audit"]
    limits = summary["input_limits"]
    lines.extend(
        [
            "",
            "## Applied-input and transition audits",
            "",
            f"Prediction-transition coverage: "
            f"{transition['unique_predecessor_rows']}/"
            f"{transition['expected_unique_predecessor_rows']} unique predecessor "
            f"rows (complete={transition['coverage_complete']}, "
            f"fraction={rounded(transition['coverage_fraction'])}).",
            "",
            f"Input-limit audit: {limits['component_bound_violations']}/"
            f"{limits['unique_physical_rows']} unique physical rows exceeded "
            f"{rounded(limits['component_limit'])} m/s per component; "
            f"maximum component {rounded(limits['maximum_applied_component'])} m/s, "
            f"planar norm {rounded(limits['maximum_planar_norm'])} m/s, and "
            f"0.5 s displacement {rounded(limits['maximum_displacement'])} m.",
            "",
            "## Exact mechanism records",
            "",
            "| Mechanism | Key | Baseline error | Complete status / error |",
            "|---|---|---:|---|",
        ]
    )
    for name, record in summary["mechanism_records"].items():
        key = record["key"]
        baseline = record["baseline"]
        complete = record["variants"]["predictive_multistart"]
        complete_text = (
            "n/a"
            if complete is None
            else f"{complete['output_status']} / {rounded(complete['error'])}"
        )
        lines.append(
            f"| `{name}` | ({key['seed']}, {key['frame_index']}, "
            f"{key['robot_id']}) | "
            f"{'n/a' if baseline is None else rounded(baseline['error'])} | "
            f"{complete_text} |"
        )
    lines.extend(
        [
            "",
            "## Gates",
            "",
            "| Gate | Passed | Numerator | Denominator |",
            "|---|---:|---:|---:|",
        ]
    )
    for name, gate in summary["gates"].items():
        lines.append(
            f"| `{name}` | {gate['passed']} | {gate['numerator']} | "
            f"{gate['denominator']} |"
        )
    return "\n".join(lines) + "\n"


def _reverify_inputs(
    authorization: Mapping,
    *,
    protocol_path: Path,
    development_manifest_path: Path,
) -> None:
    replay._read_trusted_bytes(
        protocol_path, expected_identity=authorization["protocol_identity"]
    )
    for identity in authorization["source_identities"].values():
        replay._read_trusted_bytes(
            Path(identity["path"]),
            expected_identity=identity,
            capture_payload=False,
        )
    replay._read_trusted_bytes(
        development_manifest_path,
        expected_identity=authorization["manifest_identity"],
        capture_payload=False,
    )
    replay._read_trusted_bytes(
        authorization["raw_path"],
        expected_identity=authorization["raw_identity"],
        capture_payload=False,
    )


def _reconcile_completed(
    *,
    transaction: Mapping,
    stage: Mapping,
    authorization: Mapping,
    protocol_path: Path,
    development_manifest_path: Path,
    json_fd: int,
    json_identity: Mapping,
    markdown_fd: int,
    markdown_identity: Mapping,
) -> bool:
    """Accept an already durable, still-valid completed transaction."""
    if not replay._target_matches_stage(transaction, stage):
        return False
    if stage.get("cleanup_started") and not stage.get("source_cleaned"):
        try:
            replay._rollback_owned_target(transaction, stage)
        except Exception:
            pass
        return False
    try:
        _reverify_inputs(
            authorization,
            protocol_path=protocol_path,
            development_manifest_path=development_manifest_path,
        )
        _verify_compact_file(
            transaction, OUTPUT_JSON_NAME, json_fd, json_identity
        )
        _verify_compact_file(
            transaction,
            OUTPUT_MARKDOWN_NAME,
            markdown_fd,
            markdown_identity,
        )
        if not stage.get("source_cleaned"):
            replay._cleanup_stage(transaction, stage)
        replay._assert_registered_root(transaction)
        replay._fsync_output_directory(transaction)
        return replay._target_matches_stage(transaction, stage)
    except Exception:
        return False


def analyze_predictive_recovery(
    *,
    baseline_process_path: Path,
    development_manifest_path: Path,
    protocol_path: Path,
    expected_baseline_sha256: str,
    output_root: Path,
) -> dict:
    """Aggregate exact-key paired tails, status transitions, and calibration."""
    baseline_process_path = replay._runtime_absolute(Path(baseline_process_path))
    development_manifest_path = replay._runtime_absolute(
        Path(development_manifest_path)
    )
    protocol_path = replay._runtime_absolute(Path(protocol_path))
    output_root = replay._runtime_absolute(Path(output_root))
    authorization = _authorize_inputs(
        baseline_process_path=baseline_process_path,
        development_manifest_path=development_manifest_path,
        protocol_path=protocol_path,
        expected_baseline_sha256=expected_baseline_sha256,
        output_root=output_root,
    )
    free_before = require_start_space(_nearest_existing_ancestor(output_root))
    started_at = datetime.now(timezone.utc).isoformat()
    transaction = None
    json_fd = None
    markdown_fd = None
    json_identity = None
    markdown_identity = None
    finalizing_stage = None
    completed_stage = None
    completed = None

    def terminal(status: str, *, allocated: int | None, error=None) -> dict:
        value = {
            "status": status,
            "schema_id": ANALYSIS_SCHEMA_ID,
            "protocol_identity": authorization["protocol_identity"],
            "source_identities": authorization["source_identities"],
            "development_manifest_identity": authorization["manifest_identity"],
            "raw_process_identity": authorization["raw_identity"],
            "output_root": str(output_root),
            "started_at": started_at,
            "ended_at": datetime.now(timezone.utc).isoformat(),
            "free_bytes_before": free_before,
            "free_bytes_after": (
                replay._available_bytes_fd(transaction)
                if transaction is not None
                else None
            ),
            "allocated_bytes": allocated,
            "compact_output_cap_bytes": COMPACT_OUTPUT_CAP_BYTES,
            "outputs": {
                name: identity
                for name, identity in (
                    (OUTPUT_JSON_NAME, json_identity),
                    (OUTPUT_MARKDOWN_NAME, markdown_identity),
                )
                if identity is not None
            },
        }
        if error is not None:
            value["error"] = {"type": type(error).__name__, "message": str(error)}
        return value

    creation_state = {}
    try:
        transaction = replay._create_exact_root(
            output_root, identity_sink=creation_state
        )
    except BaseException as error:
        transaction = creation_state.get("transaction")
        if transaction is not None:
            failure = terminal("failed", allocated=0, error=error)
            replay._publish_failure_or_note(transaction, failure, error)
            replay._close_output_transaction(transaction)
        raise
    try:
        replay._assert_registered_root(transaction)
        if replay._available_bytes_fd(transaction) < HARD_FLOOR_BYTES:
            raise DiskSpaceError("available bytes below live floor")
        baseline_identity = authorization["source_identities"]["baseline_process"]
        baseline_rows = _ordered_baseline_rows(
            _iter_gzip_json_rows(
                Path(baseline_identity["path"]),
                expected_identity=baseline_identity,
            )
        )
        development_digest = hashlib.sha256()
        development_rows = _iter_gzip_json_rows(
            authorization["raw_path"],
            expected_identity=authorization["raw_identity"],
            decompressed_digest=development_digest,
        )
        truth = replay._parse_json_object(
            authorization["source_payloads"]["truth_data"],
            Path(authorization["source_identities"]["truth_data"]["path"]),
        )
        summary = aggregate_predictive_recovery(
            baseline_rows=baseline_rows,
            development_rows=development_rows,
            truth_data=truth,
            protocol=authorization["protocol"],
        )
        if (
            development_digest.hexdigest()
            != authorization["manifest"]["decompressed_process_sha256"]
            or sum(
                item["budgets"]["attempted"]
                for item in summary["variants"].values()
            )
            != authorization["manifest"]["rows_written"]
        ):
            raise ValueError("development process count/hash changed during analysis")
        if authorization["production"]:
            _validate_production_summary(summary)
        json_payload = replay._strict_json_bytes(summary, indent=2) + b"\n"
        markdown_payload = _markdown(summary).encode("utf-8")
        if len(json_payload) + len(markdown_payload) >= COMPACT_OUTPUT_CAP_BYTES:
            raise DiskSpaceError("compact payload exceeds output cap")
        json_fd = _open_compact_file(transaction, OUTPUT_JSON_NAME)
        json_identity = _write_descriptor(json_fd, json_payload)
        markdown_fd = _open_compact_file(transaction, OUTPUT_MARKDOWN_NAME)
        markdown_identity = _write_descriptor(markdown_fd, markdown_payload)
        allocated = _allocated_sum((json_fd, markdown_fd))
        if allocated >= COMPACT_OUTPUT_CAP_BYTES:
            raise DiskSpaceError("compact outputs exceed allocated-byte cap")
        finalizing = terminal("finalizing", allocated=allocated)
        finalizing["outputs"] = {
            OUTPUT_JSON_NAME: json_identity,
            OUTPUT_MARKDOWN_NAME: markdown_identity,
        }
        finalizing_stage = replay._write_stage(
            transaction,
            "finalizing",
            replay._strict_json_bytes(finalizing, indent=2) + b"\n",
        )
        if (
            allocated + os.fstat(finalizing_stage["fd"]).st_blocks * 512
            > COMPACT_OUTPUT_CAP_BYTES
        ):
            raise DiskSpaceError("compact finalizing probe exceeds output cap")
        _reverify_inputs(
            authorization,
            protocol_path=protocol_path,
            development_manifest_path=development_manifest_path,
        )
        _verify_compact_file(
            transaction, OUTPUT_JSON_NAME, json_fd, json_identity
        )
        _verify_compact_file(
            transaction, OUTPUT_MARKDOWN_NAME, markdown_fd, markdown_identity
        )
        replay._cleanup_stage(transaction, finalizing_stage)
        finalizing_stage = None
        completed = terminal("completed", allocated=allocated)
        completed["outputs"] = {
            OUTPUT_JSON_NAME: json_identity,
            OUTPUT_MARKDOWN_NAME: markdown_identity,
        }
        completed_stage = replay._write_stage(
            transaction,
            "completed",
            replay._strict_json_bytes(completed, indent=2) + b"\n",
        )
        total_allocated = allocated + os.fstat(completed_stage["fd"]).st_blocks * 512
        if total_allocated > COMPACT_OUTPUT_CAP_BYTES:
            raise DiskSpaceError("compact bundle including manifest exceeds cap")
        completed["allocated_bytes"] = total_allocated
        _rewrite_stage(
            completed_stage,
            replay._strict_json_bytes(completed, indent=2) + b"\n",
        )
        final_total = allocated + os.fstat(completed_stage["fd"]).st_blocks * 512
        if final_total != total_allocated:
            completed["allocated_bytes"] = final_total
            _rewrite_stage(
                completed_stage,
                replay._strict_json_bytes(completed, indent=2) + b"\n",
            )
            final_total = (
                allocated + os.fstat(completed_stage["fd"]).st_blocks * 512
            )
        if final_total > COMPACT_OUTPUT_CAP_BYTES:
            raise DiskSpaceError("compact bundle including manifest exceeds cap")
        _reverify_inputs(
            authorization,
            protocol_path=protocol_path,
            development_manifest_path=development_manifest_path,
        )
        _verify_compact_file(
            transaction, OUTPUT_JSON_NAME, json_fd, json_identity
        )
        _verify_compact_file(
            transaction, OUTPUT_MARKDOWN_NAME, markdown_fd, markdown_identity
        )
        replay._assert_registered_root(transaction)
        replay._link_stage(transaction, completed_stage)
        completed_stage["cleanup_started"] = True
        replay._cleanup_stage(transaction, completed_stage)
        replay._fsync_output_directory(transaction)
        if not replay._target_matches_stage(transaction, completed_stage):
            raise ValueError("completed analyzer manifest changed before commit")
        _verify_compact_file(
            transaction, OUTPUT_JSON_NAME, json_fd, json_identity
        )
        _verify_compact_file(
            transaction, OUTPUT_MARKDOWN_NAME, markdown_fd, markdown_identity
        )
        replay._post_commit_boundary()
        return completed
    except BaseException as error:
        if (
            isinstance(completed, dict)
            and isinstance(completed_stage, dict)
            and isinstance(json_fd, int)
            and isinstance(json_identity, dict)
            and isinstance(markdown_fd, int)
            and isinstance(markdown_identity, dict)
            and _reconcile_completed(
                transaction=transaction,
                stage=completed_stage,
                authorization=authorization,
                protocol_path=protocol_path,
                development_manifest_path=development_manifest_path,
                json_fd=json_fd,
                json_identity=json_identity,
                markdown_fd=markdown_fd,
                markdown_identity=markdown_identity,
            )
        ):
            return completed
        for stage in (completed_stage, finalizing_stage):
            if isinstance(stage, dict):
                if stage is completed_stage:
                    try:
                        replay._rollback_owned_target(transaction, stage)
                    except Exception:
                        pass
                try:
                    replay._cleanup_stage(transaction, stage)
                except Exception:
                    pass
        allocated = _allocated_sum(
            descriptor
            for descriptor in (json_fd, markdown_fd)
            if isinstance(descriptor, int)
        )
        failure = terminal("failed", allocated=allocated, error=error)
        replay._publish_failure_or_note(transaction, failure, error)
        raise
    finally:
        replay._close_output_transaction(transaction)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-process-path", type=Path, required=True)
    parser.add_argument("--development-manifest-path", type=Path, required=True)
    parser.add_argument("--protocol-json", type=Path, required=True)
    parser.add_argument("--expected-baseline-sha256", required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    arguments = parser.parse_args(argv)
    analyze_predictive_recovery(
        baseline_process_path=arguments.baseline_process_path,
        development_manifest_path=arguments.development_manifest_path,
        protocol_path=arguments.protocol_json,
        expected_baseline_sha256=arguments.expected_baseline_sha256,
        output_root=arguments.output_root,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
