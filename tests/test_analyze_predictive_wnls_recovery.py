"""Contracts for the exact Stage-1 predictive-recovery analyzer."""

from __future__ import annotations

import copy
import gzip
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from scripts.diagnostics import analyze_predictive_wnls_recovery as analyzer
from scripts.diagnostics import predictive_wnls
from scripts.diagnostics import replay_predictive_wnls_recovery as replay
from scripts.diagnostics.run_diagnostic import START_BYTES, DiskSpaceError


def baseline_row(
    seed: int,
    frame: int,
    robot: int,
    *,
    attempt: str = "converged",
    status: str = "converged",
    error: float | None = 1.0,
) -> dict:
    return {
        "graph_case": "dynamic_dag_wnls",
        "seed": seed,
        "frame_index": frame,
        "robot_id": robot,
        "squad_local_index": robot,
        "attempt_status": attempt,
        "status": status,
        "error_norm": error,
        "finite": error is not None,
    }


def development_row(
    variant: str,
    seed: int,
    frame: int,
    robot: int,
    *,
    output: str = "fresh",
    attempt: str = "accepted",
    error: float | None = 1.0,
    prediction_age: int | None = 0,
) -> dict:
    values = {field: None for field in replay.ROW_FIELDS}
    values.update(
        {
            "variant": variant,
            "seed": seed,
            "frame_index": frame,
            "robot_id": robot,
            "squad_local_index": robot,
            "applied_command_source_frame": None if frame == 0 else frame - 1,
            "applied_command": None if frame == 0 else [1.0, 0.0],
            "legacy_numeric_status": (
                None if variant == "predictive_multistart" else "converged"
            ),
            "legacy_initial_estimate_source": "test",
            "attempt_status": attempt,
            "attempt_failure_reason": None,
            "output_status": output,
            "prediction_age": prediction_age,
            "estimate": None if output == "unavailable" else [error or 0.0, 0.0],
            "fresh_modeled_covariance": (
                [[1.0, 0.0], [0.0, 1.0]] if output == "fresh" else None
            ),
            "fresh_epsilon": 3.0 if output == "fresh" else None,
            "aged_modeled_covariance": (
                [[2.0, 0.0], [0.0, 2.0]] if output == "predicted" else None
            ),
            "aged_modeled_radius": (
                3.0 * (2.0**0.5) if output == "predicted" else None
            ),
            "private_reacquisition_seed": None,
            "attempt_base_anchor_provenance": [0, 1],
            "base_anchor_provenance": [0, 1] if output == "fresh" else [],
            "mandatory_references": {"base_ids": [0, 1], "uav_ids": []},
            "optional_candidates": [],
            "active_references": [["base", 0], ["base", 1]],
            "reference_evidence": [
                [
                    "base",
                    base_id,
                    "mandatory",
                    True,
                    1.0,
                    seed + base_id,
                    "fresh",
                    True,
                    attempt != "reference_unavailable",
                    None,
                    [base_id],
                ]
                for base_id in (0, 1)
            ],
            "reference_freshness": [
                ["base", base_id, "fresh"] for base_id in (0, 1)
            ],
            "excluded_references": [],
            "reference_violations": [],
            "candidates": (
                [
                    [
                        "test",
                        [0.0, 0.0],
                        "converged",
                        [error or 0.0, 0.0],
                        [[1.0, 0.0], [0.0, 1.0]],
                        0.0,
                        True,
                        None,
                        (
                            None
                            if variant == "predictive_multistart"
                            else 1.0
                        ),
                        (
                            [
                                "not_applicable_reacquisition",
                                None,
                                "accepted",
                                True,
                                None,
                                0.0,
                            ]
                            if variant == "predictive_multistart"
                            else [
                                "normalized",
                                1.0,
                                "accepted",
                                True,
                                None,
                                0.0,
                            ]
                        ),
                        [],
                        *(
                            [
                                3.0,
                                1.0,
                                1.0,
                                True,
                                0,
                                0,
                                0.0,
                                None,
                            ]
                            if variant == "predictive_multistart"
                            else []
                        ),
                    ]
                ]
                if attempt == "accepted"
                else []
            ),
            "selected_candidate_source": "test" if attempt == "accepted" else None,
            "offline_truth_position": [0.0, 0.0],
            "offline_error_norm": error if output != "unavailable" else None,
            "offline_fresh_containment": (
                error <= 3.0 if output == "fresh" and error is not None else None
            ),
            "offline_aged_radius_containment": (
                error <= 3.0 * (2.0**0.5)
                if output == "predicted" and error is not None
                else None
            ),
            "offline_fresh_q_error": (
                error * error if output == "fresh" and error is not None else None
            ),
            "offline_aged_q_error": (
                error * error / 2.0
                if output == "predicted" and error is not None
                else None
            ),
        }
    )
    return values


def truth_data() -> dict:
    def robot(identifier: int, vx: float) -> dict:
        return {
            "id": identifier,
            "state": {"x": 0.0, "y": 0.0},
            "opt": {"result": {"vx": vx, "vy": 0.0}},
        }

    return {
        "config": sensor_config(2, separate_squads=True),
        "state": [
            {"frame_index": 0, "robots": [robot(1, 1.0), robot(2, 1.0)]},
            {"frame_index": 1, "robots": [robot(1, 30.0), robot(2, 1.0)]},
            {"frame_index": 2, "robots": [robot(1, 1.0), robot(2, 1.0)]},
        ]
    }


def sensor_config(number: int, *, separate_squads: bool) -> dict:
    parts = number if separate_squads else 1
    return {
        "num": number,
        "bases": [
            [100.0, 0.0],
            [0.0, 100.0],
            [-100.0, 0.0],
        ],
        "formation": {
            "parts": parts,
            "bases-id": [[0, 1, 2] for _ in range(parts)],
        },
        "cbfs": {
            "without-slack": {
                "comm-fixed": {
                    "min-neighbour-id-offset": -2,
                    "max-neighbour-id-offset": 0,
                    "max-range": 1000.0,
                }
            }
        },
        "position_covariance": {"ranging_sigma": 0.5},
    }


def materialize_sensor_evidence(rows: list[dict], truth: dict) -> None:
    """Populate exact producer sensor-boundary evidence for literal rows."""
    config = truth["config"]
    frames = {
        frame["frame_index"]: {
            robot["id"]: [
                float(robot["state"]["x"]),
                float(robot["state"]["y"]),
            ]
            for robot in frame["robots"]
        }
        for frame in truth["state"]
    }
    current_group = None
    current_public = {}
    current_numeric_available = {}
    previous_numeric_available = {}
    for row in rows:
        group = (row["variant"], row["seed"], row["frame_index"])
        if group != current_group:
            current_group = group
            current_public = {}
            current_numeric_available = {}
        mandatory, optional, records, noise_seeds = replay._sensor_records(
            config,
            row["robot_id"],
            {
                identifier: np.asarray(position, dtype=float)
                for identifier, position in frames[row["frame_index"]].items()
            },
            row["seed"],
            row["frame_index"],
            config["position_covariance"]["ranging_sigma"],
        )
        qualification = replay.qualify_active_references(
            mandatory=mandatory,
            optional_keys=optional,
            measurement_records=records,
            uav_outputs=current_public,
            variant=row["variant"],
        )
        active_records = replay._canonical_active_records(qualification)
        active_keys = tuple(tuple(record["key"]) for record in active_records)
        if row["variant"] == "prediction_expiry":
            arrays_available = (
                len(active_keys) >= 2
                and all(
                    kind == "base"
                    or current_numeric_available.get(identifier, False)
                    for kind, identifier in active_keys
                )
            )
        else:
            arrays_available = (
                qualification["status"] == "ok"
                and len(active_keys) >= 2
            )
        solver_used = (
            active_keys
            if arrays_available
            else ()
        )
        evidence = replay._reference_evidence(
            mandatory=mandatory,
            optional=optional,
            records=records,
            noise_seeds=noise_seeds,
            qualification=qualification,
            current_public=current_public,
            solver_used_keys=solver_used,
            solver_exclusion_reason=(
                None
                if solver_used
                else "not_supplied_due_to_reference_state_unavailable"
            ),
        )
        row["mandatory_references"] = {
            field: list(mandatory[field])
            for field in replay.MANDATORY_REFERENCE_FIELDS
        }
        row["optional_candidates"] = [list(key) for key in optional]
        row["active_references"] = [list(key) for key in active_keys]
        row["reference_evidence"] = evidence
        row["reference_freshness"] = replay._reference_freshness(evidence)
        row["excluded_references"] = replay._normalize_reason_records(
            qualification["excluded"],
            fields=replay.EXCLUSION_FIELDS,
        )
        row["reference_violations"] = replay._normalize_reason_records(
            qualification["violations"],
            fields=replay.VIOLATION_FIELDS,
        )
        row["attempt_base_anchor_provenance"] = list(
            qualification["base_anchor_provenance"]
        )
        if row["output_status"] == "fresh":
            row["base_anchor_provenance"] = list(
                row["attempt_base_anchor_provenance"]
            )
        numeric_state_key = (
            row["variant"],
            row["seed"],
            row["robot_id"],
        )
        if row["legacy_numeric_status"] == "converged":
            numeric_available = True
        elif row["legacy_numeric_status"] == "stale":
            numeric_available = previous_numeric_available.get(
                numeric_state_key, False
            )
        else:
            numeric_available = False
        current_numeric_available[row["robot_id"]] = numeric_available
        previous_numeric_available[numeric_state_key] = numeric_available
        current_public[row["robot_id"]] = analyzer._public_output_from_row(row)


def protocol() -> dict:
    return {
        "experiment": {
            "variants": list(replay.DEVELOPMENT_VARIANTS),
            "frame_dt_seconds": 0.5,
        },
        "ablation_contracts": copy.deepcopy(replay.ABLATION_CONTRACTS),
        "estimator_constants": copy.deepcopy(replay.ESTIMATOR_CONSTANTS),
        "gates": copy.deepcopy(replay.GATES),
    }


def paired_fixture() -> tuple[list[dict], list[dict]]:
    baseline = [
        baseline_row(11, 0, 1, error=1.0),
        baseline_row(11, 0, 2, status="stale", attempt="failed", error=60.0),
        baseline_row(
            11, 1, 1, status="failed", attempt="invalid", error=None
        ),
        baseline_row(11, 1, 2, error=4.0),
        baseline_row(11, 2, 1, error=6.0),
    ]
    rows = []
    for variant in replay.DEVELOPMENT_VARIANTS:
        rows.extend(
            [
                development_row(variant, 11, 0, 1, error=0.5),
                development_row(
                    variant,
                    11,
                    0,
                    2,
                    output="predicted",
                    attempt="rejected",
                    error=3.0,
                    prediction_age=1,
                ),
                development_row(
                    variant,
                    11,
                    1,
                    1,
                    output="unavailable",
                    attempt="invalid",
                    error=None,
                    prediction_age=None,
                ),
                development_row(
                    variant,
                    11,
                    1,
                    2,
                    output=(
                        "unavailable"
                        if variant == "predictive_multistart"
                        else "fresh"
                    ),
                    attempt=(
                        "invalid"
                        if variant == "predictive_multistart"
                        else "accepted"
                    ),
                    error=None if variant == "predictive_multistart" else 2.0,
                    prediction_age=(
                        None if variant == "predictive_multistart" else 0
                    ),
                ),
                development_row(
                    variant,
                    11,
                    2,
                    1,
                    output="predicted",
                    attempt="rejected",
                    error=5.0,
                    prediction_age=2,
                ),
            ]
        )
    numeric_available = {}
    for row in rows:
        if row["variant"] == "predictive_multistart":
            continue
        state_key = (row["variant"], row["seed"], row["robot_id"])
        attempt_status = row["attempt_status"]
        if attempt_status == "accepted":
            candidate_status = "converged"
            legacy_status = "converged"
        elif attempt_status == "rejected":
            candidate_status = "invalid"
            legacy_status = (
                "stale"
                if numeric_available.get(state_key, False)
                else "invalid"
            )
        else:
            candidate_status = attempt_status
            legacy_status = (
                "stale"
                if numeric_available.get(state_key, False)
                else candidate_status
            )
        set_legacy_candidate_contract(
            row,
            candidate_status=candidate_status,
            attempt_status=attempt_status,
            legacy_status=legacy_status,
            rejection_reason=(
                None
                if attempt_status == "accepted"
                else f"test_{attempt_status}"
            ),
        )
        numeric_available[state_key] = legacy_status in {
            "converged",
            "stale",
        }
    rejected = rows[-4]
    rejected["candidates"] = [
        [
            "algebraic",
            [0.0, 0.0],
            "converged",
            [9.0, 0.0],
            [[1.0, 0.0], [0.0, 1.0]],
            10.0,
            False,
            "reacquisition_reduced_cost_exceeds_nine",
            None,
            [
                "not_applicable_reacquisition",
                None,
                "rejected",
                None,
                None,
                10.0,
            ],
            [],
            3.0,
            1.0,
            1.0,
            True,
            0,
            0,
            0.0,
            None,
        ]
    ]
    accepted = rows[-5]
    accepted["candidates"] = [
        [
            "prediction",
            [0.0, 0.0],
            "converged",
            [0.5, 0.0],
            [[1.0, 0.0], [0.0, 1.0]],
            0.1,
            True,
            None,
            None,
            [
                "not_applicable_reacquisition",
                None,
                "accepted",
                None,
                None,
                0.1,
            ],
            [],
            3.0,
            1.0,
            1.0,
            True,
            0,
            0,
            0.0,
            None,
        ]
    ]
    accepted["selected_candidate_source"] = "prediction"
    for row in rows:
        if (
            row["variant"] == "predictive_multistart"
            and not row["candidates"]
            and row["attempt_status"] != "reference_unavailable"
        ):
            row["attempt_status"] = "invalid"
            row["attempt_failure_reason"] = "no_valid_initial_candidates"
            row["selected_candidate_source"] = None
        if (
            row["variant"] == "prediction_expiry"
            and row["attempt_status"] == "rejected"
        ):
            make_unavailable(row)
            set_legacy_candidate_contract(
                row,
                candidate_status="invalid",
                attempt_status="invalid",
                legacy_status=(
                    "stale"
                    if row["frame_index"] > 0 and row["robot_id"] == 1
                    else "invalid"
                ),
                rejection_reason="invalid_geometry",
            )
        if row["frame_index"] == 2 and row["robot_id"] == 1:
            row["applied_command"] = [30.0, 0.0]
    materialize_nonaccepted_public_lifecycle(rows)
    materialize_legacy_initial_sources(rows)
    materialize_sensor_evidence(rows, truth_data())
    materialize_legacy_gate_contract(rows)
    return baseline, rows


def semantic_fixture(
    keys: list[tuple[int, int, int]],
) -> tuple[list[dict], list[dict], dict]:
    """Build a literal zero-truth cohort for reference-integrity tests."""
    baseline = [
        baseline_row(seed, frame, robot)
        for seed, frame, robot in sorted(
            keys, key=lambda key: (key[1], key[0], key[2])
        )
    ]
    rows = [
        development_row(variant, seed, frame, robot)
        for variant in replay.DEVELOPMENT_VARIANTS
        for seed, frame, robot in sorted(keys)
    ]
    maximum_frame = max(frame for _, frame, _ in keys)
    robots = sorted({robot for _, _, robot in keys})
    states = []
    for frame in range(maximum_frame + 1):
        states.append(
            {
                "frame_index": frame,
                "robots": [
                    {
                        "id": robot,
                        "state": {"x": 10.0 * robot, "y": 0.0},
                        "opt": {"result": {"vx": 1.0, "vy": 0.0}},
                    }
                    for robot in robots
                ],
            }
        )
    truth = {
        "config": sensor_config(max(robots), separate_squads=False),
        "state": states,
    }
    for row in rows:
        truth_x = 10.0 * row["robot_id"]
        row["offline_truth_position"] = [truth_x, 0.0]
        if row["output_status"] != "unavailable":
            row["estimate"] = [
                truth_x + float(row["offline_error_norm"]),
                0.0,
            ]
            if (
                row["variant"] == "predictive_multistart"
                and row["attempt_status"] == "accepted"
                and len(row["candidates"]) == 1
            ):
                row["candidates"][0][3] = copy.deepcopy(row["estimate"])
                row["candidates"][0][4] = copy.deepcopy(
                    row["fresh_modeled_covariance"]
                )
    materialize_legacy_initial_sources(rows)
    materialize_nonaccepted_public_lifecycle(rows)
    materialize_sensor_evidence(rows, truth)
    materialize_legacy_gate_contract(rows)
    return baseline, rows, truth


def row_at(
    rows: list[dict],
    variant: str,
    seed: int,
    frame: int,
    robot: int,
) -> dict:
    return next(
        row
        for row in rows
        if (
            row["variant"],
            row["seed"],
            row["frame_index"],
            row["robot_id"],
        )
        == (variant, seed, frame, robot)
    )


def add_uav_evidence(
    target: dict,
    source: dict,
    *,
    active: bool = True,
    serialized_eligible: bool | None = None,
    serialized_freshness: str | None = None,
) -> None:
    """Attach a complete compact UAV reference using explicit public fields."""
    identifier = source["robot_id"]
    key = ["uav", identifier]
    declared_mandatory = identifier in target["mandatory_references"]["uav_ids"]
    if not declared_mandatory and key not in target["optional_candidates"]:
        target["optional_candidates"].append(key)
        target["optional_candidates"].sort(
            key=lambda item: (item[0] == "uav", item[1])
        )
    if active:
        if key not in target["active_references"]:
            target["active_references"].append(key)
            target["active_references"].sort(
                key=lambda item: (item[0] == "uav", item[1])
            )
    elif key in target["active_references"]:
        target["active_references"].remove(key)
    freshness = (
        source["output_status"]
        if serialized_freshness is None
        else serialized_freshness
    )
    eligible = source["output_status"] == "fresh"
    if serialized_eligible is not None:
        eligible = serialized_eligible
    evidence = next(
        (
            item
            for item in target["reference_evidence"]
            if item[:2] == key
        ),
        None,
    )
    if evidence is None:
        evidence = [
            "uav",
            identifier,
            "mandatory" if declared_mandatory else "optional",
            True,
            1.0,
            1000 + identifier,
            freshness,
            eligible,
            active,
            None,
            list(source["base_anchor_provenance"]),
        ]
        target["reference_evidence"].append(evidence)
    else:
        evidence[6] = freshness
        evidence[7] = eligible
        evidence[8] = active
        evidence[9] = None
        evidence[10] = list(source["base_anchor_provenance"])
    target["reference_evidence"].sort(
        key=lambda item: (item[0] == "uav", item[1])
    )
    target["reference_freshness"] = [
        [item[0], item[1], item[6]]
        for item in target["reference_evidence"]
    ]
    violation = ["uav", identifier, "stale_or_predicted_anchor_used"]
    target["reference_violations"] = [
        item
        for item in target["reference_violations"]
        if item[:2] != key
    ]
    if active and source["output_status"] != "fresh":
        target["reference_violations"].append(violation)
    target["reference_violations"].sort(
        key=lambda item: (item[0] == "uav", item[1], item[2])
    )


def make_predicted(row: dict, *, error: float = 1.0) -> None:
    truth = row["offline_truth_position"]
    row.update(
        {
            "attempt_status": "rejected",
            "output_status": "predicted",
            "prediction_age": 1,
            "estimate": [float(truth[0]) + error, float(truth[1])],
            "fresh_modeled_covariance": None,
            "fresh_epsilon": None,
            "aged_modeled_covariance": [[2.0, 0.0], [0.0, 2.0]],
            "aged_modeled_radius": 3.0 * (2.0**0.5),
            "base_anchor_provenance": [],
            "candidates": [],
            "selected_candidate_source": None,
            "offline_error_norm": error,
            "offline_fresh_containment": None,
            "offline_aged_radius_containment": True,
            "offline_fresh_q_error": None,
            "offline_aged_q_error": error * error / 2.0,
        }
    )
    if row["variant"] == "prediction_expiry":
        set_legacy_candidate_contract(
            row,
            candidate_status="invalid",
            attempt_status="invalid",
            legacy_status="invalid",
            rejection_reason="invalid_geometry",
        )
    elif row["variant"] != "predictive_multistart":
        set_legacy_candidate_contract(
            row,
            candidate_status="converged",
            attempt_status="rejected",
            legacy_status="converged",
            rejection_reason="reacquisition_reduced_cost_exceeds_nine",
        )


def make_unavailable(row: dict) -> None:
    row.update(
        {
            "output_status": "unavailable",
            "prediction_age": None,
            "estimate": None,
            "fresh_modeled_covariance": None,
            "fresh_epsilon": None,
            "aged_modeled_covariance": None,
            "aged_modeled_radius": None,
            "base_anchor_provenance": [],
            "offline_error_norm": None,
            "offline_fresh_containment": None,
            "offline_aged_radius_containment": None,
            "offline_fresh_q_error": None,
            "offline_aged_q_error": None,
        }
    )


def materialize_nonaccepted_public_lifecycle(rows: list[dict]) -> None:
    """Apply the producer's exact prior-public publication policy to fixtures."""
    previous_public = {}
    for row in rows:
        state_key = (row["variant"], row["seed"], row["robot_id"])
        prior = previous_public.get(state_key)
        previous_output = (
            prior[1]
            if prior and prior[0] == row["frame_index"] - 1
            else None
        )
        propagated = replay._propagate_public(
            previous_output,
            None,
            row["applied_command"],
        )["public_prediction"]
        if row["attempt_status"] != "accepted":
            if propagated["output_status"] == "predicted":
                covariance = np.asarray(
                    propagated["modeled_covariance"], dtype=float
                )
                estimate = np.asarray(
                    propagated["estimate"], dtype=float
                )
                truth = np.asarray(
                    row["offline_truth_position"], dtype=float
                )
                residual = truth - estimate
                error = float(np.linalg.norm(residual))
                q_error = float(
                    residual @ np.linalg.solve(covariance, residual)
                )
                row.update(
                    {
                        "output_status": "predicted",
                        "prediction_age": propagated["prediction_age"],
                        "estimate": propagated["estimate"],
                        "fresh_modeled_covariance": None,
                        "fresh_epsilon": None,
                        "aged_modeled_covariance": propagated[
                            "modeled_covariance"
                        ],
                        "aged_modeled_radius": propagated[
                            "aged_modeled_radius"
                        ],
                        "base_anchor_provenance": [],
                        "offline_error_norm": error,
                        "offline_fresh_containment": None,
                        "offline_aged_radius_containment": (
                            error <= propagated["aged_modeled_radius"]
                        ),
                        "offline_fresh_q_error": None,
                        "offline_aged_q_error": q_error,
                    }
                )
            else:
                make_unavailable(row)
        previous_public[state_key] = (
            row["frame_index"],
            analyzer._public_output_from_row(row),
        )


def set_legacy_candidate_contract(
    row: dict,
    *,
    candidate_status: str,
    attempt_status: str,
    legacy_status: str,
    source: str | None = None,
    rejection_reason: str | None = None,
) -> None:
    if source is None:
        source = row["legacy_initial_estimate_source"]
    accepted = attempt_status == "accepted"
    row["legacy_numeric_status"] = legacy_status
    row["legacy_initial_estimate_source"] = source
    row["attempt_status"] = attempt_status
    row["attempt_failure_reason"] = rejection_reason
    row["candidates"] = [
        [
            source,
            [0.0, 0.0],
            candidate_status,
            (
                copy.deepcopy(row["estimate"])
                if accepted
                else [1.0, 0.0]
                if candidate_status == "converged"
                else None
            ),
            (
                copy.deepcopy(row["fresh_modeled_covariance"])
                if accepted
                else [[1.0, 0.0], [0.0, 1.0]]
                if candidate_status == "converged"
                else None
            ),
            (
                10.0
                if candidate_status == "converged"
                and attempt_status == "rejected"
                else 1.0
                if candidate_status == "converged"
                else None
            ),
            accepted,
            rejection_reason,
            None,
            [
                "not_applied_legacy_solver",
                None,
                (
                    "accepted"
                    if accepted
                    else "rejected"
                    if candidate_status == "converged"
                    else "invalid"
                ),
                candidate_status == "converged",
                None if candidate_status == "converged" else rejection_reason,
                (
                    10.0
                    if candidate_status == "converged"
                    and attempt_status == "rejected"
                    else None
                ),
            ],
            [],
        ]
    ]
    row["selected_candidate_source"] = source if accepted else None


def materialize_legacy_initial_sources(rows: list[dict]) -> None:
    previous_available = {}
    ever_finite = {}
    for row in rows:
        state_key = (row["variant"], row["seed"], row["robot_id"])
        if row["variant"] == "predictive_multistart":
            row["legacy_initial_estimate_source"] = None
        else:
            if row["frame_index"] == 0:
                expected_source = "deployment_frame_zero"
            elif previous_available.get(state_key, False):
                expected_source = "previous_finite"
            elif not ever_finite.get(state_key, False):
                expected_source = "deployment_restart_before_first_finite"
            else:
                expected_source = "strict_previous_missing"
            row["legacy_initial_estimate_source"] = expected_source
            if len(row["candidates"]) == 1:
                candidate = row["candidates"][0]
                candidate[0] = expected_source
                candidate[8] = None
                candidate[9] = [
                    "not_applied_legacy_solver",
                    None,
                    (
                        "accepted"
                        if candidate[6]
                        else "rejected"
                        if candidate[2] == "converged"
                        else "invalid"
                    ),
                    candidate[2] == "converged",
                    None if candidate[2] == "converged" else candidate[7],
                    None,
                ]
                if candidate[6]:
                    candidate[3] = copy.deepcopy(row["estimate"])
                    candidate[4] = copy.deepcopy(
                        row["fresh_modeled_covariance"]
                    )
                    row["selected_candidate_source"] = expected_source
        current_available = row["legacy_numeric_status"] in {
            "converged",
            "stale",
        }
        previous_available[state_key] = current_available
        ever_finite[state_key] = (
            ever_finite.get(state_key, False) or current_available
        )


def materialize_legacy_gate_contract(rows: list[dict]) -> None:
    previous_public = {}
    for row in rows:
        state_key = (row["variant"], row["seed"], row["robot_id"])
        prior = previous_public.get(state_key)
        live_prediction = None
        if prior and prior[0] == row["frame_index"] - 1:
            propagated = replay._propagate_public(
                prior[1],
                None,
                row["applied_command"],
            )["public_prediction"]
            if propagated["output_status"] == "predicted":
                live_prediction = propagated
        has_live_prediction = live_prediction is not None
        if row["variant"] == "predictive_multistart":
            for candidate in row["candidates"]:
                compact = dict(
                    zip(
                        replay.MULTISTART_COMPACT_CANDIDATE_FIELDS,
                        candidate,
                    )
                )
                result = {
                    field: compact[field]
                    for field in replay.SOLVER_RESULT_FIELDS
                    if field != "proposal_trace"
                }
                result["proposal_trace"] = [
                    dict(zip(replay.PROPOSAL_TRACE_FIELDS, proposal))
                    for proposal in compact["proposal_trace"]
                ]
                accepted, reason, diagnostics = (
                    predictive_wnls.candidate_acceptance(
                        result,
                        live_prediction=live_prediction,
                        active_reference_count=len(
                            row["active_references"]
                        ),
                        base_anchor_provenance=row[
                            "attempt_base_anchor_provenance"
                        ],
                    )
                )
                candidate[6] = accepted
                candidate[7] = None if accepted else reason
                candidate[8] = diagnostics.get("q_innov")
                candidate[9] = [
                    diagnostics.get(field)
                    for field in replay.GATE_DIAGNOSTIC_FIELDS
                ]
        if (
            row["variant"] != "predictive_multistart"
            and len(row["candidates"]) == 1
        ):
            candidate = row["candidates"][0]
            accepted, reason, diagnostics = replay._legacy_gate(
                {
                    "status": candidate[2],
                    "estimate": candidate[3],
                    "covariance": candidate[4],
                    "cost": candidate[5],
                    "failure_reason": candidate[9][4],
                },
                variant=row["variant"],
                has_live_prediction=has_live_prediction,
                active_count=len(row["active_references"]),
                provenance=tuple(row["attempt_base_anchor_provenance"]),
            )
            candidate[6] = accepted
            candidate[7] = None if accepted else reason
            candidate[9] = [
                diagnostics.get(field)
                for field in replay.GATE_DIAGNOSTIC_FIELDS
            ]
            row["attempt_status"] = (
                "accepted"
                if accepted
                else {
                    "converged": "rejected",
                    "failed": "failed",
                    "invalid": "invalid",
                }[candidate[2]]
            )
            row["attempt_failure_reason"] = None if accepted else reason
            row["selected_candidate_source"] = (
                candidate[0] if accepted else None
            )
        previous_public[state_key] = (
            row["frame_index"],
            analyzer._public_output_from_row(row),
        )


def set_reference_unavailable_contract(
    row: dict,
    *,
    legacy_status: str,
) -> None:
    row["legacy_numeric_status"] = legacy_status
    row["attempt_status"] = "reference_unavailable"
    row["attempt_failure_reason"] = "reference_unavailable"
    row["candidates"] = []
    row["selected_candidate_source"] = None


def coordinate_legacy_rejection(
    row: dict,
    *,
    reason: str,
    reduced_cost: float | None = None,
) -> None:
    make_unavailable(row)
    candidate = row["candidates"][0]
    if reduced_cost is not None:
        candidate[5] = reduced_cost
    candidate[6] = False
    candidate[7] = reason
    candidate[9] = [
        "not_applied_legacy_solver",
        None,
        "rejected",
        True,
        None,
        reduced_cost,
    ]
    row["legacy_numeric_status"] = "converged"
    row["attempt_status"] = "rejected"
    row["attempt_failure_reason"] = reason
    row["selected_candidate_source"] = None


def legacy_numeric_contract_fixture():
    keys = [
        (11, 0, 1),
        (11, 0, 2),
        (11, 1, 1),
        (11, 1, 2),
        (12, 0, 1),
        (12, 0, 2),
    ]
    baseline, rows, truth = semantic_fixture(keys)
    variant = "fresh_reference_qualification"

    prior_source = row_at(rows, variant, 11, 0, 1)
    prior_target = row_at(rows, variant, 11, 0, 2)
    set_legacy_candidate_contract(
        prior_source,
        candidate_status="converged",
        attempt_status="accepted",
        legacy_status="converged",
    )
    set_legacy_candidate_contract(
        prior_target,
        candidate_status="converged",
        attempt_status="accepted",
        legacy_status="converged",
    )

    stale_source = row_at(rows, variant, 11, 1, 1)
    make_predicted(stale_source)
    set_legacy_candidate_contract(
        stale_source,
        candidate_status="invalid",
        attempt_status="invalid",
        legacy_status="stale",
        rejection_reason="invalid_geometry",
    )
    stale_target = row_at(rows, variant, 11, 1, 2)
    make_predicted(stale_target)
    set_reference_unavailable_contract(stale_target, legacy_status="stale")

    invalid_source = row_at(rows, variant, 12, 0, 1)
    make_unavailable(invalid_source)
    set_legacy_candidate_contract(
        invalid_source,
        candidate_status="invalid",
        attempt_status="invalid",
        legacy_status="invalid",
        rejection_reason="invalid_geometry",
    )
    invalid_target = row_at(rows, variant, 12, 0, 2)
    make_unavailable(invalid_target)
    set_reference_unavailable_contract(invalid_target, legacy_status="invalid")

    materialize_nonaccepted_public_lifecycle(rows)
    materialize_sensor_evidence(rows, truth)
    materialize_legacy_gate_contract(rows)
    return baseline, rows, truth


def prediction_expiry_stale_fixture(
    robot_count: int,
    *,
    predicted_ids: tuple[int, ...],
):
    keys = [
        (11, frame, robot)
        for frame in (0, 1)
        for robot in range(1, robot_count + 1)
    ]
    baseline, rows, truth = semantic_fixture(keys)
    for robot_id in predicted_ids:
        source = row_at(
            rows, "prediction_expiry", 11, 1, robot_id
        )
        make_predicted(source)
        source["legacy_numeric_status"] = "stale"
    materialize_nonaccepted_public_lifecycle(rows)
    materialize_sensor_evidence(rows, truth)
    materialize_legacy_gate_contract(rows)
    return baseline, rows, truth


class PureAggregationTests(unittest.TestCase):
    def test_status_mapping_exact_cohorts_and_denominators(self):
        """Breaks if stale or unavailable baseline rows enter fresh errors."""
        baseline, rows = paired_fixture()
        result = analyzer.aggregate_predictive_recovery(
            baseline_rows=baseline,
            development_rows=rows,
            truth_data=truth_data(),
            protocol=protocol(),
        )
        self.assertEqual(result["baseline"]["output_counts"], {
            "fresh": 3,
            "legacy_published": 1,
            "unavailable": 1,
        })
        complete = result["variants"]["predictive_multistart"]
        self.assertEqual(
            complete["output_counts"],
            {"fresh": 1, "predicted": 2, "unavailable": 2},
        )
        self.assertEqual(complete["budgets"], {"attempted": 5, "outputs": 5})
        self.assertEqual(
            sum(complete["attempt_counts"].values()), complete["budgets"]["attempted"]
        )
        self.assertEqual(
            complete["errors"]["fresh"]["denominator"], 1
        )
        self.assertEqual(
            complete["errors"]["all_published"]["denominator"], 3
        )
        self.assertEqual(
            result["prediction_transition_audit"]["unique_predecessor_rows"], 3
        )
        self.assertEqual(
            set(complete["errors"]["all_published"]),
            {"denominator", "p50", "p95", "p99", "maximum"},
        )
        self.assertEqual(complete["errors"]["all_published"]["p50"], 1.0)
        self.assertEqual(complete["errors"]["all_published"]["maximum"], 16.0)

    def test_pair_audits_separate_attrition_downgrade_and_improvement(self):
        """Breaks if excluded rows are disguised as paired improvements."""
        baseline, rows = paired_fixture()
        result = analyzer.aggregate_predictive_recovery(
            baseline_rows=baseline,
            development_rows=rows,
            truth_data=truth_data(),
            protocol=protocol(),
        )
        paired = result["variants"]["predictive_multistart"]["paired"]
        published = paired["baseline_published"]
        self.assertEqual(
            published["transition_counts"],
            {"fresh": 1, "predicted": 1, "unavailable": 2},
        )
        self.assertEqual(published["newly_unavailable"]["count"], 2)
        self.assertEqual(
            published["newly_unavailable"]["baseline_errors"]["maximum"],
            60.0,
        )
        self.assertEqual(
            published["both_published_error_change"]["denominator"], 2
        )
        fresh = paired["baseline_fresh"]
        self.assertEqual(
            fresh["transition_counts"],
            {"fresh": 1, "predicted": 1, "unavailable": 1},
        )
        self.assertEqual(fresh["both_fresh_error_change"]["denominator"], 1)
        self.assertEqual(fresh["downgraded_to_predicted"]["count"], 1)
        self.assertEqual(fresh["newly_unavailable"]["count"], 1)
        self.assertEqual(fresh["excluded_from_both_fresh"]["count"], 2)

    def test_calibration_rejected_candidates_catastrophic_and_strata(self):
        """Breaks if diagnostic families share denominators or strata disappear."""
        baseline, rows = paired_fixture()
        result = analyzer.aggregate_predictive_recovery(
            baseline_rows=baseline,
            development_rows=rows,
            truth_data=truth_data(),
            protocol=protocol(),
        )
        complete = result["variants"]["predictive_multistart"]
        self.assertEqual(
            {
                key: complete["catastrophic"]["all_published"][key]
                for key in ("numerator", "denominator")
            },
            {"numerator": 0, "denominator": 3},
        )
        calibration = complete["calibration"]
        self.assertEqual(calibration["fresh_containment"]["denominator"], 1)
        self.assertEqual(calibration["fresh_epsilon"]["denominator"], 1)
        self.assertEqual(calibration["fresh_q_error"]["denominator"], 1)
        self.assertEqual(calibration["fresh_q_error"]["above_5_991464547107979"], 0)
        self.assertEqual(calibration["fresh_q_error"]["above_9"], 0)
        self.assertEqual(calibration["aged_containment"]["denominator"], 2)
        self.assertEqual(calibration["aged_radius"]["denominator"], 2)
        self.assertEqual(calibration["aged_q_error"]["above_5_991464547107979"], 1)
        self.assertEqual(calibration["aged_q_error"]["above_9"], 1)
        self.assertEqual(calibration["online_q_innovation"]["accepted"]["denominator"], 0)
        self.assertEqual(calibration["online_q_innovation"]["rejected"]["denominator"], 0)
        rejected = complete["rejected_candidate_offline_errors"]
        self.assertEqual(rejected["denominator"], 1)
        self.assertEqual(rejected["maximum"], 9.0)
        self.assertEqual(
            set(complete["strata"]),
            {"depth", "squad", "time", "seed"},
        )
        self.assertEqual(complete["strata"]["seed"]["11"]["rows"], 5)
        self.assertEqual(complete["strata"]["squad"]["1"]["rows"], 5)
        self.assertEqual(complete["strata"]["time"]["0"]["rows"], 5)
        self.assertEqual(complete["strata"]["depth"]["1"]["rows"], 3)

    def test_truth_input_limit_audit_deduplicates_variants_and_noise_seeds(self):
        """Breaks if repeated raw rows inflate the physical command denominator."""
        audit = analyzer.audit_input_limits(truth_data(), component_limit=25.0)
        self.assertEqual(audit["unique_physical_rows"], 6)
        self.assertEqual(audit["component_bound_violations"], 1)
        self.assertEqual(audit["maximum_applied_component"], 30.0)
        self.assertEqual(audit["maximum_planar_norm"], 30.0)
        self.assertEqual(audit["maximum_displacement"], 15.0)

    def test_production_shaped_input_audit_is_exactly_243_of_7000(self):
        """Breaks if Stage 1 audits predecessor rows or repeated noise rows."""
        states = []
        remaining = 243
        for frame in range(500):
            robots = []
            for robot_id in range(1, 15):
                violating = remaining > 0
                if violating:
                    remaining -= 1
                robots.append(
                    {
                        "id": robot_id,
                        "state": {"x": 0.0, "y": 0.0},
                        "opt": {
                            "result": {
                                "vx": 25.5 if violating else 25.0,
                                "vy": 0.0,
                            }
                        },
                    }
                )
            states.append({"frame_index": frame, "robots": robots})
        audit = analyzer.audit_input_limits(
            {"state": states}, component_limit=25.0
        )
        self.assertEqual(
            (
                audit["component_bound_violations"],
                audit["unique_physical_rows"],
            ),
            (243, 7000),
        )

    def test_production_summary_requires_complete_6986_predecessor_coverage(self):
        """Breaks if production can publish with missing prediction transitions."""
        summary = {
            "mechanism_records": {
                "record": {
                    "baseline": {"error": 1.0},
                    "variants": {
                        variant: {"error": 1.0}
                        for variant in replay.DEVELOPMENT_VARIANTS
                    },
                }
            },
            "input_limits": {
                "unique_physical_rows": 7000,
                "component_bound_violations": 243,
            },
            "prediction_transition_audit": {
                "unique_predecessor_rows": 6986,
                "expected_unique_predecessor_rows": 6986,
                "coverage_complete": True,
            },
        }
        analyzer._validate_production_summary(summary)
        mutations = (
            ("unique_predecessor_rows", 6985),
            ("expected_unique_predecessor_rows", 6985),
            ("coverage_complete", False),
        )
        for field, value in mutations:
            with self.subTest(field=field):
                changed = copy.deepcopy(summary)
                changed["prediction_transition_audit"][field] = value
                with self.assertRaises(ValueError):
                    analyzer._validate_production_summary(changed)

    def test_production_raw_disk_metrics_reconcile_to_pinned_leaf(self):
        """Breaks if replay disk claims are trusted without raw-leaf reconciliation."""
        metrics = {
            "free_bytes_before": START_BYTES,
            "free_bytes_after": replay.HARD_FLOOR_BYTES + 1,
            "allocated_bytes": 4096,
            "raw_bundle_cap_bytes": replay.RAW_BUNDLE_CAP_BYTES,
        }
        analyzer._validate_production_disk_metrics(
            metrics, actual_allocated_bytes=4096
        )
        mutations = (
            ("free_bytes_before", START_BYTES - 1),
            ("free_bytes_after", replay.HARD_FLOOR_BYTES - 1),
            ("allocated_bytes", 8192),
            ("allocated_bytes", True),
            ("raw_bundle_cap_bytes", replay.RAW_BUNDLE_CAP_BYTES - 1),
        )
        for field, value in mutations:
            with self.subTest(field=field, value=value):
                changed = dict(metrics)
                changed[field] = value
                with self.assertRaises(ValueError):
                    analyzer._validate_production_disk_metrics(
                        changed, actual_allocated_bytes=4096
                    )

    def test_exact_key_contract_rejects_duplicate_missing_extra_and_order_drift(self):
        """Breaks if positional pairing can silently change a denominator."""
        baseline, rows = paired_fixture()
        mutations = [
            rows + [copy.deepcopy(rows[0])],
            rows[:-1],
            rows + [development_row("predictive_multistart", 99, 0, 1)],
            [rows[1], rows[0], *rows[2:]],
        ]
        wrong_schema = copy.deepcopy(rows)
        wrong_schema[0].pop("offline_fresh_q_error")
        mutations.append(wrong_schema)
        semantic_drift = copy.deepcopy(rows)
        semantic_drift[0]["offline_error_norm"] = 0.75
        mutations.append(semantic_drift)
        epsilon_drift = copy.deepcopy(rows)
        epsilon_drift[0]["fresh_epsilon"] = 4.0
        mutations.append(epsilon_drift)
        covariance_drift = copy.deepcopy(rows)
        covariance_drift[0]["fresh_modeled_covariance"] = [
            [1.0, 0.1],
            [0.0, 1.0],
        ]
        mutations.append(covariance_drift)
        truth_drift = copy.deepcopy(rows)
        truth_drift[0]["offline_truth_position"] = [1.0, 0.0]
        truth_drift[0]["estimate"] = [1.5, 0.0]
        mutations.append(truth_drift)
        compact_drift = copy.deepcopy(rows)
        compact_drift[-5]["candidates"][0][9] = ["truncated"]
        mutations.append(compact_drift)
        attempt_output_drift = copy.deepcopy(rows)
        attempt_output_drift[0]["attempt_status"] = "failed"
        attempt_output_drift[0]["selected_candidate_source"] = None
        attempt_output_drift[0]["candidates"][0][6] = False
        mutations.append(attempt_output_drift)
        accepted_predicted_drift = copy.deepcopy(rows)
        accepted_predicted_drift[1]["attempt_status"] = "accepted"
        mutations.append(accepted_predicted_drift)
        selected_source_drift = copy.deepcopy(rows)
        selected_source_drift[0]["selected_candidate_source"] = "not_a_candidate"
        mutations.append(selected_source_drift)
        accepted_candidate_drift = copy.deepcopy(rows)
        accepted_candidate_drift[0]["candidates"][0][6] = False
        mutations.append(accepted_candidate_drift)
        squad_local_drift = copy.deepcopy(rows)
        squad_local_drift[0]["squad_local_index"] = 7
        mutations.append(squad_local_drift)
        squad_local_bool = copy.deepcopy(rows)
        squad_local_bool[0]["squad_local_index"] = True
        mutations.append(squad_local_bool)
        squad_local_string = copy.deepcopy(rows)
        squad_local_string[0]["squad_local_index"] = "1"
        mutations.append(squad_local_string)
        for changed in mutations:
            with self.subTest(rows=len(changed)):
                with self.assertRaises(ValueError):
                    analyzer.aggregate_predictive_recovery(
                        baseline_rows=baseline,
                        development_rows=changed,
                        truth_data=truth_data(),
                        protocol=protocol(),
                    )

    def test_baseline_filter_and_native_order_are_enforced(self):
        """Breaks if fixed-graph rows enter pairs or dynamic rows reorder."""
        baseline, rows = paired_fixture()
        fixed = dict(baseline[0])
        fixed["graph_case"] = "fixed_refs_wnls"
        result = analyzer.aggregate_predictive_recovery(
            baseline_rows=[fixed, *baseline],
            development_rows=rows,
            truth_data=truth_data(),
            protocol=protocol(),
        )
        self.assertEqual(result["baseline"]["budgets"]["attempted"], 5)
        reordered = [baseline[2], baseline[0], baseline[1], *baseline[3:]]
        with self.assertRaisesRegex(ValueError, "out of order"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=reordered,
                development_rows=rows,
                truth_data=truth_data(),
                protocol=protocol(),
            )

    def test_protocol_integrity_violations_fail_closed(self):
        """Breaks if aged anchors, age three, or cycles can enter evidence."""
        baseline, rows = paired_fixture()
        mutations = []
        age = copy.deepcopy(rows)
        age[-1]["prediction_age"] = 3
        mutations.append(age)
        dag = copy.deepcopy(rows)
        dag[-1]["active_references"].append(["uav", dag[-1]["robot_id"]])
        mutations.append(dag)
        anchor = copy.deepcopy(rows)
        anchor[-1]["reference_evidence"] = [
            [
                "uav",
                1,
                "optional",
                True,
                1.0,
                2,
                "predicted",
                True,
                True,
                None,
                [0, 1],
            ]
        ]
        mutations.append(anchor)
        provenance = copy.deepcopy(rows)
        provenance[-1]["reference_violations"] = [
            ["uav", 1, "provenance"]
        ]
        mutations.append(provenance)
        current_chain = copy.deepcopy(rows)
        current_chain[5]["base_anchor_provenance"] = [2, 3]
        mutations.append(current_chain)
        attempt_chain = copy.deepcopy(rows)
        attempt_chain[6]["attempt_base_anchor_provenance"] = [99]
        mutations.append(attempt_chain)
        timing = copy.deepcopy(rows)
        timing[-1]["applied_command"] = [1.0, 0.0]
        mutations.append(timing)
        for changed in mutations:
            with self.subTest():
                with self.assertRaises(ValueError):
                    analyzer.aggregate_predictive_recovery(
                        baseline_rows=baseline,
                        development_rows=changed,
                        truth_data=truth_data(),
                        protocol=protocol(),
                    )

    def test_public_covariances_must_be_exact_canonical_matrices(self):
        """Breaks if tolerance-valid but noncanonical public state is accepted."""
        baseline, rows = paired_fixture()
        fresh = copy.deepcopy(rows)
        fresh[0]["fresh_modeled_covariance"] = [
            [1.0, 5.0e-14],
            [0.0, 1.0],
        ]
        predicted = copy.deepcopy(rows)
        predicted_target = row_at(
            predicted, "predictive_multistart", 11, 1, 1
        )
        predicted_target["aged_modeled_covariance"] = [
            [2.0, 5.0e-14],
            [0.0, 2.0],
        ]
        for changed in (fresh, predicted):
            with self.subTest():
                with self.assertRaisesRegex(ValueError, "canonical"):
                    analyzer.aggregate_predictive_recovery(
                        baseline_rows=baseline,
                        development_rows=changed,
                        truth_data=truth_data(),
                        protocol=protocol(),
                    )

    def test_uav_evidence_is_reconstructed_from_lower_index_public_output(self):
        """Breaks if serialized freshness or eligibility is trusted as truth."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1), (11, 0, 2)])
        expiry_source = row_at(rows, "prediction_expiry", 11, 0, 1)
        expiry_target = row_at(rows, "prediction_expiry", 11, 0, 2)
        add_uav_evidence(
            expiry_target,
            expiry_source,
            serialized_eligible=False,
            serialized_freshness="predicted",
        )
        expiry_target["reference_violations"] = [
            ["uav", 1, "stale_or_predicted_anchor_used"]
        ]
        with self.assertRaisesRegex(ValueError, "reconstructed"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_serialized_eligible_cannot_override_reconstructed_ineligibility(self):
        """Breaks if an ineligible predicted output can be marked eligible."""
        baseline, rows, truth = prediction_expiry_stale_fixture(
            2, predicted_ids=(1,)
        )
        source = row_at(rows, "prediction_expiry", 11, 1, 1)
        target = row_at(rows, "prediction_expiry", 11, 1, 2)
        add_uav_evidence(target, source, serialized_eligible=True)
        with self.assertRaisesRegex(ValueError, "eligible"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_reference_state_resets_at_frame_boundary(self):
        """Breaks if a prior frame's UAV output leaks into the current group."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1), (11, 1, 2)])
        source = row_at(rows, "prediction_expiry", 11, 0, 1)
        target = row_at(rows, "prediction_expiry", 11, 1, 2)
        add_uav_evidence(target, source)
        with self.assertRaisesRegex(ValueError, "same group"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_reference_state_resets_at_seed_boundary(self):
        """Breaks if a prior seed's UAV output leaks into the current group."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1), (12, 0, 2)])
        source = row_at(rows, "prediction_expiry", 11, 0, 1)
        target = row_at(rows, "prediction_expiry", 12, 0, 2)
        add_uav_evidence(target, source)
        with self.assertRaisesRegex(ValueError, "same group"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_nonactive_same_or_future_uav_evidence_fails_closed(self):
        """Breaks if same/future UAVs bypass the DAG check as optional evidence."""
        for reference_id in (2, 3):
            with self.subTest(reference_id=reference_id):
                baseline, rows, truth = semantic_fixture(
                    [(11, 0, 1), (11, 0, 2), (11, 0, 3)]
                )
                target = row_at(rows, "prediction_expiry", 11, 0, 2)
                source = row_at(rows, "prediction_expiry", 11, 0, reference_id)
                add_uav_evidence(target, source, active=False)
                with self.assertRaisesRegex(
                    ValueError, "sensor boundary|lower-index"
                ):
                    analyzer.aggregate_predictive_recovery(
                        baseline_rows=baseline,
                        development_rows=rows,
                        truth_data=truth,
                        protocol=protocol(),
                    )

    def test_reference_evidence_must_exactly_match_declared_candidates(self):
        """Breaks if duplicate, missing, or undeclared evidence is accepted."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1), (11, 0, 2)])
        source = row_at(rows, "prediction_expiry", 11, 0, 1)
        target = row_at(rows, "prediction_expiry", 11, 0, 2)
        add_uav_evidence(target, source, active=False)
        mutations = []
        duplicate = copy.deepcopy(rows)
        duplicate_target = row_at(duplicate, "prediction_expiry", 11, 0, 2)
        duplicate_target["reference_evidence"].append(
            copy.deepcopy(duplicate_target["reference_evidence"][-1])
        )
        duplicate_target["reference_freshness"].append(
            copy.deepcopy(duplicate_target["reference_freshness"][-1])
        )
        mutations.append(duplicate)
        missing = copy.deepcopy(rows)
        missing_target = row_at(missing, "prediction_expiry", 11, 0, 2)
        missing_target["reference_evidence"].pop()
        missing_target["reference_freshness"].pop()
        mutations.append(missing)
        extra = copy.deepcopy(rows)
        extra_target = row_at(extra, "prediction_expiry", 11, 0, 2)
        extra_target["optional_candidates"] = []
        mutations.append(extra)
        for changed in mutations:
            with self.subTest():
                with self.assertRaises(ValueError):
                    analyzer.aggregate_predictive_recovery(
                        baseline_rows=baseline,
                        development_rows=changed,
                        truth_data=truth,
                        protocol=protocol(),
                    )

    def test_reconstructed_evidence_rejects_measurement_and_provenance_tampering(self):
        """Breaks if active measurements or lower-output provenance are unaudited."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1), (11, 0, 2)])
        source = row_at(rows, "prediction_expiry", 11, 0, 1)
        target = row_at(rows, "prediction_expiry", 11, 0, 2)
        add_uav_evidence(target, source)
        measurement = copy.deepcopy(rows)
        measurement_target = row_at(
            measurement, "prediction_expiry", 11, 0, 2
        )
        measurement_evidence = measurement_target["reference_evidence"][-1]
        measurement_evidence[3] = False
        measurement_evidence[4] = None
        measurement_evidence[5] = None
        measurement_evidence[7] = False
        measurement_target["reference_violations"] = [
            ["uav", 1, "stale_or_predicted_anchor_used"]
        ]
        provenance = copy.deepcopy(rows)
        provenance_target = row_at(
            provenance, "prediction_expiry", 11, 0, 2
        )
        provenance_target["reference_evidence"][-1][10] = [99]
        for changed in (measurement, provenance):
            with self.subTest():
                with self.assertRaises(ValueError):
                    analyzer.aggregate_predictive_recovery(
                        baseline_rows=baseline,
                        development_rows=changed,
                        truth_data=truth,
                        protocol=protocol(),
                    )

    def test_sensor_boundary_rejects_coordinated_reference_deletion_and_relabel(self):
        """Breaks if declarations can redefine the protocol-bound sensor graph."""
        keys = [(11, 0, robot) for robot in range(1, 5)]
        baseline, rows, truth = semantic_fixture(keys)
        deletion = copy.deepcopy(rows)
        deletion_target = row_at(
            deletion, "predictive_multistart", 11, 0, 4
        )
        deletion_target["optional_candidates"].remove(["uav", 1])
        deletion_target["active_references"].remove(["uav", 1])
        deletion_target["reference_evidence"] = [
            item
            for item in deletion_target["reference_evidence"]
            if item[:2] != ["uav", 1]
        ]
        deletion_target["reference_freshness"] = [
            item
            for item in deletion_target["reference_freshness"]
            if item[:2] != ["uav", 1]
        ]

        relabel = copy.deepcopy(rows)
        relabel_target = row_at(
            relabel, "predictive_multistart", 11, 0, 4
        )
        relabel_target["optional_candidates"].remove(["uav", 1])
        relabel_target["mandatory_references"]["uav_ids"].append(1)
        relabel_target["mandatory_references"]["uav_ids"].sort()
        next(
            item
            for item in relabel_target["reference_evidence"]
            if item[:2] == ["uav", 1]
        )[2] = "mandatory"

        for changed in (deletion, relabel):
            with self.subTest():
                with self.assertRaisesRegex(ValueError, "sensor boundary"):
                    analyzer.aggregate_predictive_recovery(
                        baseline_rows=baseline,
                        development_rows=changed,
                        truth_data=truth,
                        protocol=protocol(),
                    )

    def test_sensor_boundary_rejects_coordinated_measurement_and_exclusion_tamper(self):
        """Breaks if mutually consistent serialized sensor lies are accepted."""
        keys = [(11, 0, robot) for robot in range(1, 5)]
        baseline, rows, truth = semantic_fixture(keys)
        measurement = copy.deepcopy(rows)
        measurement_target = row_at(
            measurement, "predictive_multistart", 11, 0, 4
        )
        evidence = next(
            item
            for item in measurement_target["reference_evidence"]
            if item[:2] == ["uav", 1]
        )
        evidence[3:10] = [
            False,
            None,
            None,
            "fresh",
            False,
            False,
            "measurement_not_present",
        ]
        measurement_target["active_references"].remove(["uav", 1])
        measurement_target["excluded_references"].append(
            ["uav", 1, "measurement_not_present"]
        )
        measurement_target["excluded_references"].sort(
            key=lambda item: (item[0] == "uav", item[1], item[2])
        )

        exclusion = copy.deepcopy(rows)
        exclusion_target = row_at(
            exclusion, "predictive_multistart", 11, 0, 4
        )
        exclusion_evidence = next(
            item
            for item in exclusion_target["reference_evidence"]
            if item[:2] == ["uav", 1]
        )
        exclusion_evidence[9] = "not_current_frame_fresh"
        exclusion_target["excluded_references"].append(
            ["uav", 1, "not_current_frame_fresh"]
        )

        for changed in (measurement, exclusion):
            with self.subTest():
                with self.assertRaisesRegex(ValueError, "sensor boundary"):
                    analyzer.aggregate_predictive_recovery(
                        baseline_rows=baseline,
                        development_rows=changed,
                        truth_data=truth,
                        protocol=protocol(),
                    )

    def test_prediction_expiry_attempted_provenance_is_reconstructed_exactly(self):
        """Breaks if historical base ids can excuse current provenance drift."""
        baseline, rows, truth = prediction_expiry_stale_fixture(
            4, predicted_ids=(3,)
        )
        source = row_at(rows, "prediction_expiry", 11, 1, 3)
        target = row_at(rows, "prediction_expiry", 11, 1, 4)
        add_uav_evidence(target, source)
        target["attempt_base_anchor_provenance"] = [0]

        with self.assertRaisesRegex(ValueError, "numeric provenance"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_prediction_expiry_accepted_fresh_provenance_matches_attempt(self):
        """Breaks if the diagnostic ablation may forge published provenance."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1)])
        materialize_sensor_evidence(rows, truth)
        materialize_legacy_gate_contract(rows)
        target = row_at(rows, "prediction_expiry", 11, 0, 1)
        target["base_anchor_provenance"] = [0, 2]

        with self.assertRaisesRegex(ValueError, "publication provenance"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_multistart_accepted_candidate_requires_finite_converged_payload(self):
        """Breaks if an accepted null solver result can publish fresh evidence."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1)])
        target = row_at(rows, "predictive_multistart", 11, 0, 1)
        candidate = target["candidates"][0]
        candidate[2:6] = ["failed", None, None, None]

        with self.assertRaisesRegex(
            ValueError, "multistart candidate gate"
        ):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_multistart_selected_candidate_matches_frozen_tie_break(self):
        """Breaks if a higher-cost accepted start can claim selection."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1)])
        target = row_at(rows, "predictive_multistart", 11, 0, 1)
        target["candidates"][0][5] = 2.0
        target["candidates"][0][9][5] = 2.0
        lower_cost = copy.deepcopy(target["candidates"][0])
        lower_cost[0] = "algebraic"
        lower_cost[5] = 1.0
        lower_cost[9][5] = 1.0
        target["candidates"].append(lower_cost)

        with self.assertRaisesRegex(ValueError, "frozen candidate selection"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_multistart_selected_candidate_binds_fresh_publication(self):
        """Breaks if fresh evidence can differ from its selected solver result."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1)])
        target = row_at(rows, "predictive_multistart", 11, 0, 1)
        target["candidates"][0][3] = [
            target["estimate"][0] + 5.0,
            target["estimate"][1],
        ]

        with self.assertRaisesRegex(ValueError, "fresh publication"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_multistart_live_gate_recomputes_normalized_innovation(self):
        """Breaks if candidate and gate may coordinate a forged live q value."""
        baseline, rows, truth = semantic_fixture(
            [(11, 0, 1), (11, 1, 1)]
        )
        target = row_at(rows, "predictive_multistart", 11, 1, 1)
        candidate = target["candidates"][0]
        forged_q = candidate[8] + 1.0
        candidate[8] = forged_q
        candidate[9][1] = forged_q

        with self.assertRaisesRegex(ValueError, "multistart candidate gate"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_multistart_real_reacquisition_compact_gate_is_accepted(self):
        """Binds analyzer semantics to producer-to-compact reacquisition bytes."""
        attempt = predictive_wnls.solve_predictive_multistart(
            reference_positions=np.asarray(
                [[0.0, 0.0], [6.0, 0.0], [0.0, 8.0]]
            ),
            reference_covariances=np.zeros((3, 2, 2)),
            measurements=np.asarray([5.0, 5.0, 5.0]),
            reference_keys=(
                ("base", 0),
                ("base", 1),
                ("base", 2),
            ),
            live_prediction=None,
            private_seed=None,
            ranging_sigma=0.5,
            base_anchor_provenance=(0, 1, 2),
        )
        compact = replay._compact_candidates(attempt["candidates"])
        selected = attempt["selected_candidate"]
        selected_compact = next(
            candidate
            for candidate in compact
            if candidate[0] == selected["source"]
        )
        self.assertIsNone(selected_compact[9][3])
        row = {
            "attempt_base_anchor_provenance": [0, 1, 2],
            "candidates": compact,
            "selected_candidate_source": selected["source"],
            "attempt_status": "accepted",
            "attempt_failure_reason": None,
            "estimate": selected["estimate"],
            "fresh_modeled_covariance": selected["covariance"],
        }

        analyzer._validate_multistart_selection(
            row,
            live_prediction=None,
            active_reference_count=3,
        )

    def test_multistart_cannot_suppress_the_true_selected_candidate(self):
        """Breaks if a valid lower-cost candidate can be relabelled rejected."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1)])
        target = row_at(rows, "predictive_multistart", 11, 0, 1)
        target["candidates"][0][5] = 2.0
        target["candidates"][0][9][5] = 2.0
        suppressed = copy.deepcopy(target["candidates"][0])
        suppressed[0] = "algebraic"
        suppressed[5] = 1.0
        suppressed[6] = False
        suppressed[7] = "forged_rejection"
        suppressed[9][5] = 1.0
        target["candidates"].append(suppressed)

        with self.assertRaisesRegex(ValueError, "candidate gate"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_nonaccepted_publication_equals_reconstructed_propagation(self):
        """Breaks if nonaccepted rows can rewrite availability or prediction."""
        baseline, rows, truth = semantic_fixture(
            [(11, 0, 1), (11, 1, 1)]
        )
        mutations = []
        unavailable = copy.deepcopy(rows)
        unavailable_target = row_at(
            unavailable, "predictive_multistart", 11, 1, 1
        )
        make_unavailable(unavailable_target)
        unavailable_target["attempt_status"] = "invalid"
        unavailable_target["attempt_failure_reason"] = (
            "no_valid_initial_candidates"
        )
        unavailable_target["candidates"] = []
        unavailable_target["selected_candidate_source"] = None
        mutations.append(unavailable)

        substituted = copy.deepcopy(rows)
        substituted_target = row_at(
            substituted, "predictive_multistart", 11, 1, 1
        )
        make_predicted(substituted_target, error=2.0)
        substituted_target["attempt_status"] = "invalid"
        substituted_target["attempt_failure_reason"] = (
            "no_valid_initial_candidates"
        )
        substituted_target["candidates"] = []
        substituted_target["selected_candidate_source"] = None
        mutations.append(substituted)

        for changed in mutations:
            with self.subTest(
                status=row_at(
                    changed, "predictive_multistart", 11, 1, 1
                )["output_status"]
            ):
                with self.assertRaisesRegex(
                    ValueError, "propagated public"
                ):
                    analyzer.aggregate_predictive_recovery(
                        baseline_rows=baseline,
                        development_rows=changed,
                        truth_data=truth,
                        protocol=protocol(),
                    )

    def test_unavailable_arrays_derive_stale_or_invalid_numeric_status(self):
        """Breaks if unavailable attempts can forge retained numeric state."""
        baseline, rows, truth = legacy_numeric_contract_fixture()
        mutations = [
            (11, 1, 2, "invalid"),
            (12, 0, 2, "converged"),
        ]
        for seed, frame, robot, forged_status in mutations:
            with self.subTest(
                seed=seed,
                frame=frame,
                forged_status=forged_status,
            ):
                changed = copy.deepcopy(rows)
                row_at(
                    changed,
                    "fresh_reference_qualification",
                    seed,
                    frame,
                    robot,
                )["legacy_numeric_status"] = forged_status
                with self.assertRaisesRegex(
                    ValueError, "legacy numeric|reference-unavailable"
                ):
                    analyzer.aggregate_predictive_recovery(
                        baseline_rows=baseline,
                        development_rows=changed,
                        truth_data=truth,
                        protocol=protocol(),
                    )

    def test_converged_candidate_cannot_forge_invalid_numeric_status(self):
        """Breaks if a converged legacy result can erase numeric availability."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1)])
        target = row_at(
            rows, "fresh_reference_qualification", 11, 0, 1
        )
        target["legacy_numeric_status"] = "invalid"
        with self.assertRaisesRegex(ValueError, "legacy numeric"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_nonconverged_candidate_cannot_forge_converged_numeric_status(self):
        """Breaks if a failed legacy result can invent finite numeric state."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1)])
        target = row_at(
            rows, "fresh_reference_qualification", 11, 0, 1
        )
        make_unavailable(target)
        set_legacy_candidate_contract(
            target,
            candidate_status="invalid",
            attempt_status="invalid",
            legacy_status="converged",
            rejection_reason="invalid_geometry",
        )
        materialize_sensor_evidence(rows, truth)
        with self.assertRaisesRegex(ValueError, "legacy numeric"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_multistart_cannot_serialize_legacy_numeric_status(self):
        """Breaks if multistart rows can inject legacy numeric state."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1)])
        row_at(
            rows, "predictive_multistart", 11, 0, 1
        )["legacy_numeric_status"] = "converged"
        with self.assertRaisesRegex(ValueError, "legacy numeric"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_legacy_candidate_source_and_failure_must_match_attempt(self):
        """Breaks if a legacy candidate can be rebound to forged attempt fields."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1)])
        source_mismatch = copy.deepcopy(rows)
        row_at(
            source_mismatch,
            "fresh_reference_qualification",
            11,
            0,
            1,
        )["legacy_initial_estimate_source"] = "forged_source"

        failure_mismatch = copy.deepcopy(rows)
        failure_target = row_at(
            failure_mismatch,
            "fresh_reference_qualification",
            11,
            0,
            1,
        )
        make_unavailable(failure_target)
        set_legacy_candidate_contract(
            failure_target,
            candidate_status="converged",
            attempt_status="rejected",
            legacy_status="converged",
            rejection_reason="innovation_gate",
        )
        failure_target["attempt_failure_reason"] = "forged_failure"
        materialize_sensor_evidence(failure_mismatch, truth)

        for field, changed in (
            ("source", source_mismatch),
            ("failure", failure_mismatch),
        ):
            with self.subTest(field=field):
                with self.assertRaisesRegex(
                    ValueError,
                    "initial estimate source|failure reason|gate",
                ):
                    analyzer.aggregate_predictive_recovery(
                        baseline_rows=baseline,
                        development_rows=changed,
                        truth_data=truth,
                        protocol=protocol(),
                    )

    def test_null_numeric_payload_cannot_be_relabelled_as_converged(self):
        """Breaks if coordinated status edits can invent a finite candidate."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1)])
        target = row_at(
            rows, "fresh_reference_qualification", 11, 0, 1
        )
        make_unavailable(target)
        set_legacy_candidate_contract(
            target,
            candidate_status="invalid",
            attempt_status="invalid",
            legacy_status="invalid",
            rejection_reason="invalid_geometry",
        )
        candidate = target["candidates"][0]
        candidate[2] = "converged"
        candidate[5] = 1.0
        candidate[7] = "innovation_gate"
        candidate[9][2] = "rejected"
        candidate[9][3] = True
        candidate[9][4] = None
        target["attempt_status"] = "rejected"
        target["attempt_failure_reason"] = "innovation_gate"
        target["legacy_numeric_status"] = "converged"
        materialize_sensor_evidence(rows, truth)

        with self.assertRaisesRegex(ValueError, "finite|converged"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_coordinated_legacy_source_relabel_is_rejected(self):
        """Breaks if matching source strings can replace initialization policy."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1)])
        target = row_at(
            rows, "fresh_reference_qualification", 11, 0, 1
        )
        target["legacy_initial_estimate_source"] = "arbitrary_source"
        target["candidates"][0][0] = "arbitrary_source"
        target["selected_candidate_source"] = "arbitrary_source"

        with self.assertRaisesRegex(ValueError, "initial estimate source"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_converged_legacy_candidate_requires_finite_spd_gate_payload(self):
        """Breaks if status alone authenticates a converged numeric result."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1)])
        mutations = []
        singular = copy.deepcopy(rows)
        row_at(
            singular, "fresh_reference_qualification", 11, 0, 1
        )["candidates"][0][4] = [[1.0, 0.0], [0.0, 0.0]]
        mutations.append(("singular_covariance", singular))
        nonfinite = copy.deepcopy(rows)
        row_at(
            nonfinite, "fresh_reference_qualification", 11, 0, 1
        )["candidates"][0][4] = [[float("inf"), 0.0], [0.0, 1.0]]
        mutations.append(("nonfinite_covariance", nonfinite))
        negative_cost = copy.deepcopy(rows)
        row_at(
            negative_cost, "fresh_reference_qualification", 11, 0, 1
        )["candidates"][0][5] = -1.0
        mutations.append(("negative_cost", negative_cost))
        nonfinite_cost = copy.deepcopy(rows)
        row_at(
            nonfinite_cost, "fresh_reference_qualification", 11, 0, 1
        )["candidates"][0][5] = float("inf")
        mutations.append(("nonfinite_cost", nonfinite_cost))
        gate = copy.deepcopy(rows)
        gate_candidate = row_at(
            gate, "fresh_reference_qualification", 11, 0, 1
        )["candidates"][0]
        gate_candidate[9][0] = "normalized"
        gate_candidate[9][2] = "invalid"
        gate_candidate[9][3] = False
        mutations.append(("gate", gate))

        for field, changed in mutations:
            with self.subTest(field=field):
                with self.assertRaisesRegex(
                    ValueError, "candidate|covariance|cost|gate"
                ):
                    analyzer.aggregate_predictive_recovery(
                        baseline_rows=baseline,
                        development_rows=changed,
                        truth_data=truth,
                        protocol=protocol(),
                    )

    def test_nonconverged_legacy_candidate_rejects_finite_or_false_gate_payload(self):
        """Breaks if invalid status can carry contradictory numeric evidence."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1)])
        target = row_at(
            rows, "fresh_reference_qualification", 11, 0, 1
        )
        make_unavailable(target)
        set_legacy_candidate_contract(
            target,
            candidate_status="invalid",
            attempt_status="invalid",
            legacy_status="invalid",
            rejection_reason="invalid_geometry",
        )
        materialize_sensor_evidence(rows, truth)

        mutations = []
        finite_payload = copy.deepcopy(rows)
        finite_candidate = row_at(
            finite_payload,
            "fresh_reference_qualification",
            11,
            0,
            1,
        )["candidates"][0]
        finite_candidate[3] = [1.0, 0.0]
        finite_candidate[4] = [[1.0, 0.0], [0.0, 1.0]]
        mutations.append(("finite_payload", finite_payload))
        nonfinite_cost = copy.deepcopy(rows)
        row_at(
            nonfinite_cost,
            "fresh_reference_qualification",
            11,
            0,
            1,
        )["candidates"][0][5] = float("inf")
        mutations.append(("nonfinite_cost", nonfinite_cost))
        gate = copy.deepcopy(rows)
        gate_candidate = row_at(
            gate, "fresh_reference_qualification", 11, 0, 1
        )["candidates"][0]
        gate_candidate[9][2] = "accepted"
        gate_candidate[9][3] = True
        mutations.append(("gate", gate))
        gate_failure = copy.deepcopy(rows)
        row_at(
            gate_failure,
            "fresh_reference_qualification",
            11,
            0,
            1,
        )["candidates"][0][9][4] = "different_failure"
        mutations.append(("gate_failure", gate_failure))

        for field, changed in mutations:
            with self.subTest(field=field):
                with self.assertRaisesRegex(
                    ValueError, "candidate|cost|gate|failure"
                ):
                    analyzer.aggregate_predictive_recovery(
                        baseline_rows=baseline,
                        development_rows=changed,
                        truth_data=truth,
                        protocol=protocol(),
                    )

    def test_rejected_converged_legacy_candidate_remains_numeric_finite(self):
        """Breaks if gate rejection is mistaken for numeric solver failure."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1)])
        target = row_at(
            rows, "fresh_reference_qualification", 11, 0, 1
        )
        make_unavailable(target)
        set_legacy_candidate_contract(
            target,
            candidate_status="converged",
            attempt_status="rejected",
            legacy_status="converged",
            rejection_reason="reacquisition_reduced_cost_exceeds_nine",
        )
        materialize_sensor_evidence(rows, truth)

        result = analyzer.aggregate_predictive_recovery(
            baseline_rows=baseline,
            development_rows=rows,
            truth_data=truth,
            protocol=protocol(),
        )
        self.assertEqual(
            result["variants"]["fresh_reference_qualification"][
                "attempt_counts"
            ]["rejected"],
            1,
        )

    def test_prediction_expiry_converged_candidate_cannot_fake_rejection(self):
        """Breaks if expiry bypass semantics are replaced by serialized gates."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1)])
        target = row_at(rows, "prediction_expiry", 11, 0, 1)
        coordinate_legacy_rejection(
            target,
            reason="reacquisition_reduced_cost_exceeds_nine",
            reduced_cost=10.0,
        )
        materialize_sensor_evidence(rows, truth)

        with self.assertRaisesRegex(ValueError, "gate|attempt"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_qualified_gate_reconstructs_provenance_and_reacquisition_context(self):
        """Breaks if a context-consistent-looking rejection reason is trusted."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1)])
        target = row_at(
            rows, "fresh_reference_qualification", 11, 0, 1
        )
        coordinate_legacy_rejection(
            target,
            reason="insufficient_base_anchor_provenance",
        )
        materialize_sensor_evidence(rows, truth)

        with self.assertRaisesRegex(ValueError, "gate|reason"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_legacy_gate_rejects_failure_and_reduced_cost_rewrites(self):
        """Breaks if unused serialized gate fields can be rewritten freely."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1)])
        target = row_at(
            rows, "fresh_reference_qualification", 11, 0, 1
        )
        coordinate_legacy_rejection(
            target,
            reason="reacquisition_reduced_cost_exceeds_nine",
            reduced_cost=10.0,
        )
        materialize_sensor_evidence(rows, truth)

        failure = copy.deepcopy(rows)
        row_at(
            failure, "fresh_reference_qualification", 11, 0, 1
        )["candidates"][0][9][4] = "rewritten_gate_failure"
        reduced = copy.deepcopy(rows)
        row_at(
            reduced, "fresh_reference_qualification", 11, 0, 1
        )["candidates"][0][9][5] = 999.0
        for field, changed in (("failure", failure), ("reduced", reduced)):
            with self.subTest(field=field):
                with self.assertRaisesRegex(ValueError, "gate|payload"):
                    analyzer.aggregate_predictive_recovery(
                        baseline_rows=baseline,
                        development_rows=changed,
                        truth_data=truth,
                        protocol=protocol(),
                    )

    def test_legacy_gate_uses_previous_public_prediction_lifecycle(self):
        """Breaks if reacquisition checks ignore a live propagated prediction."""
        baseline, rows, truth = semantic_fixture(
            [(11, 0, 1), (11, 1, 1)]
        )
        target = row_at(
            rows, "fresh_reference_qualification", 11, 1, 1
        )
        coordinate_legacy_rejection(
            target,
            reason="reacquisition_requires_three_active_references",
        )
        materialize_sensor_evidence(rows, truth)

        with self.assertRaisesRegex(ValueError, "gate|attempt"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_accepted_legacy_candidate_is_bound_to_public_state(self):
        """Breaks if valid candidate/public substitutions can diverge silently."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1)])
        mutations = []

        candidate_estimate = copy.deepcopy(rows)
        row_at(
            candidate_estimate,
            "fresh_reference_qualification",
            11,
            0,
            1,
        )["candidates"][0][3] = [12.0, 0.0]
        mutations.append(("candidate_estimate", candidate_estimate))

        candidate_covariance = copy.deepcopy(rows)
        row_at(
            candidate_covariance,
            "fresh_reference_qualification",
            11,
            0,
            1,
        )["candidates"][0][4] = [[2.0, 0.0], [0.0, 2.0]]
        mutations.append(("candidate_covariance", candidate_covariance))

        public_estimate = copy.deepcopy(rows)
        estimate_target = row_at(
            public_estimate,
            "fresh_reference_qualification",
            11,
            0,
            1,
        )
        estimate_target["estimate"] = [12.0, 0.0]
        estimate_target["offline_error_norm"] = 2.0
        estimate_target["offline_fresh_q_error"] = 4.0
        mutations.append(("public_estimate", public_estimate))

        public_covariance = copy.deepcopy(rows)
        covariance_target = row_at(
            public_covariance,
            "fresh_reference_qualification",
            11,
            0,
            1,
        )
        covariance_target["fresh_modeled_covariance"] = [
            [2.0, 0.0],
            [0.0, 2.0],
        ]
        covariance_target["fresh_epsilon"] = 3.0 * (2.0**0.5)
        covariance_target["offline_fresh_q_error"] = 0.5
        mutations.append(("public_covariance", public_covariance))

        for field, changed in mutations:
            with self.subTest(field=field):
                with self.assertRaisesRegex(
                    ValueError, "candidate|public|estimate|covariance"
                ):
                    analyzer.aggregate_predictive_recovery(
                        baseline_rows=baseline,
                        development_rows=changed,
                        truth_data=truth,
                        protocol=protocol(),
                    )

    def test_reference_state_resets_at_variant_boundary(self):
        """Breaks if a prior variant's current-frame output can be reused."""
        baseline, rows, truth = prediction_expiry_stale_fixture(
            2, predicted_ids=(1,)
        )
        expiry_source = row_at(rows, "prediction_expiry", 11, 1, 1)
        target = row_at(
            rows, "fresh_reference_qualification", 11, 1, 2
        )
        add_uav_evidence(
            target,
            expiry_source,
            serialized_eligible=False,
            serialized_freshness="predicted",
        )

        with self.assertRaisesRegex(ValueError, "reconstructed"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_reference_violations_reject_duplicate_missing_and_extra_facts(self):
        """Breaks if violation records do not exactly match reconstructed facts."""
        baseline, rows, truth = semantic_fixture([(11, 0, 1), (11, 0, 2)])
        source = row_at(rows, "prediction_expiry", 11, 0, 1)
        make_predicted(source)
        target = row_at(rows, "prediction_expiry", 11, 0, 2)
        add_uav_evidence(target, source)
        duplicate = copy.deepcopy(rows)
        duplicate_target = row_at(
            duplicate, "prediction_expiry", 11, 0, 2
        )
        duplicate_target["reference_violations"].append(
            copy.deepcopy(duplicate_target["reference_violations"][0])
        )
        missing = copy.deepcopy(rows)
        row_at(
            missing, "prediction_expiry", 11, 0, 2
        )["reference_violations"] = []
        extra = copy.deepcopy(rows)
        row_at(
            extra, "prediction_expiry", 11, 0, 2
        )["reference_violations"].append(
            ["base", 0, "stale_or_predicted_anchor_used"]
        )
        for changed in (duplicate, missing, extra):
            with self.subTest():
                with self.assertRaises(ValueError):
                    analyzer.aggregate_predictive_recovery(
                        baseline_rows=baseline,
                        development_rows=changed,
                        truth_data=truth,
                        protocol=protocol(),
                    )

    def test_prediction_expiry_violations_use_reconstructed_active_uavs(self):
        """Breaks if violation facts follow serialized eligible or noncanonical order."""
        baseline, rows, truth = prediction_expiry_stale_fixture(
            4, predicted_ids=(1, 2)
        )
        accepted = analyzer.aggregate_predictive_recovery(
            baseline_rows=baseline,
            development_rows=rows,
            truth_data=truth,
            protocol=protocol(),
        )
        self.assertEqual(
            accepted["variants"]["prediction_expiry"]["integrity"][
                "reference_violation_count"
            ],
            5,
        )
        reversed_rows = copy.deepcopy(rows)
        reversed_target = row_at(
            reversed_rows, "prediction_expiry", 11, 1, 4
        )
        reversed_target["reference_violations"].reverse()
        with self.assertRaisesRegex(ValueError, "canonical"):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=reversed_rows,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_prediction_expiry_retains_diagnostic_reference_violations(self):
        """Breaks if the sole fresh-qualification ablation is rejected or hidden."""
        baseline, rows, truth = prediction_expiry_stale_fixture(
            2, predicted_ids=(1,)
        )
        result = analyzer.aggregate_predictive_recovery(
            baseline_rows=baseline,
            development_rows=rows,
            truth_data=truth,
            protocol=protocol(),
        )
        integrity = result["variants"]["prediction_expiry"]["integrity"]
        self.assertEqual(integrity["reference_violation_count"], 1)
        self.assertEqual(
            integrity["reference_violation_reasons"],
            {"stale_or_predicted_anchor_used": 1},
        )
        qualified = copy.deepcopy(rows)
        qualified_source = row_at(
            qualified, "fresh_reference_qualification", 11, 0, 1
        )
        make_predicted(qualified_source)
        qualified_target = row_at(
            qualified, "fresh_reference_qualification", 11, 0, 2
        )
        add_uav_evidence(qualified_target, qualified_source)
        with self.assertRaises(ValueError):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=qualified,
                truth_data=truth,
                protocol=protocol(),
            )
        arbitrary = copy.deepcopy(rows)
        arbitrary_target = row_at(
            arbitrary, "prediction_expiry", 11, 1, 2
        )
        arbitrary_target["reference_violations"][0][2] = "arbitrary_reason"
        with self.assertRaises(ValueError):
            analyzer.aggregate_predictive_recovery(
                baseline_rows=baseline,
                development_rows=arbitrary,
                truth_data=truth,
                protocol=protocol(),
            )

    def test_mechanisms_are_resolved_by_exact_key_not_maximum(self):
        """Breaks if the historical mechanisms are found by approximate tails."""
        baseline = {
            (20260736, 138, 14): baseline_row(
                20260736,
                138,
                14,
                attempt="failed",
                status="stale",
                error=999.3318962079554,
            ),
            (20260730, 177, 14): baseline_row(
                20260730, 177, 14, error=168.90169712504604
            ),
            (20260736, 44, 14): baseline_row(
                20260736, 44, 14, attempt="failed", status="stale", error=12.0
            ),
            (1, 1, 1): baseline_row(1, 1, 1, error=2000.0),
        }
        variants = {
            name: {
                key: development_row(name, *key, error=1.0)
                for key in baseline
            }
            for name in replay.DEVELOPMENT_VARIANTS
        }
        records = analyzer.resolve_mechanism_records(baseline, variants)
        self.assertEqual(
            records["legacy_999m_stale"]["key"],
            {"seed": 20260736, "frame_index": 138, "robot_id": 14},
        )
        self.assertEqual(
            records["legacy_168m_fresh"]["key"],
            {"seed": 20260730, "frame_index": 177, "robot_id": 14},
        )
        self.assertEqual(
            records["frame44_recovery"]["key"]["frame_index"], 44
        )

    def test_markdown_exposes_every_diagnostic_family_with_denominators(self):
        """Breaks if compact JSON evidence is hidden from the human report."""
        baseline, rows = paired_fixture()
        result = analyzer.aggregate_predictive_recovery(
            baseline_rows=baseline,
            development_rows=rows,
            truth_data=truth_data(),
            protocol=protocol(),
        )
        markdown = analyzer._markdown(result)
        for label in (
            "Baseline budgets and tails",
            "Attempt counts and output tails",
            "Both-published error change",
            "Excluded baseline-error tails",
            "Catastrophic errors and availability",
            "Prediction lifecycle and integrity",
            "Reference summaries",
            "FIM, epsilon, radius, and q diagnostics",
            "Rejected candidate offline errors",
            "Stratified diagnostics",
            "Prediction-transition coverage: 3/4",
            "p50 / p95 / p99 / max / n",
        ):
            with self.subTest(label=label):
                self.assertIn(label, markdown)


class DirectCliBootstrapTests(unittest.TestCase):
    def test_direct_script_help_runs_from_repository_root(self):
        """Breaks if direct registered-path execution cannot import the package."""
        implementation_root = Path(analyzer.__file__).resolve().parents[2]
        environment = os.environ.copy()
        environment.pop("PYTHONPATH", None)
        completed = subprocess.run(
            [
                sys.executable,
                "scripts/diagnostics/analyze_predictive_wnls_recovery.py",
                "--help",
            ],
            cwd=implementation_root,
            env=environment,
            capture_output=True,
            text=True,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("--baseline-process-path", completed.stdout)

    def test_direct_script_cannot_import_preceding_shadow_scripts_package(self):
        """Breaks if PYTHONPATH can replace the registered implementation."""
        implementation_root = Path(analyzer.__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            shadow_root = root / "shadow-pythonpath"
            shadow_diagnostics = shadow_root / "scripts" / "diagnostics"
            shadow_diagnostics.mkdir(parents=True)
            (shadow_root / "scripts" / "__init__.py").write_text("")
            (shadow_diagnostics / "__init__.py").write_text("")
            marker = root / "shadow-imported"
            (
                shadow_diagnostics / "replay_predictive_wnls_recovery.py"
            ).write_text(
                "import os\n"
                "from pathlib import Path\n"
                "Path(os.environ['CBF2026_SHADOW_MARKER']).write_text('shadow')\n"
                "raise RuntimeError('shadow replay imported')\n"
            )
            environment = os.environ.copy()
            environment["CBF2026_SHADOW_MARKER"] = str(marker)
            environment["PYTHONPATH"] = os.pathsep.join(
                (str(shadow_root), str(implementation_root))
            )
            completed = subprocess.run(
                [
                    sys.executable,
                    "scripts/diagnostics/analyze_predictive_wnls_recovery.py",
                    "--help",
                ],
                cwd=implementation_root,
                env=environment,
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("--baseline-process-path", completed.stdout)
            self.assertFalse(marker.exists())


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


class AnalyzerLifecycleTests(unittest.TestCase):
    V3_RAW_MANIFEST = Path(
        "/private/tmp/cbf2026-predictive-wnls-development/"
        "stage1-v3/manifest.json"
    )
    V3_FAILED_ANALYZER_MANIFEST = Path(
        "/private/tmp/cbf2026-predictive-wnls-development-analysis/"
        "stage1-v3/manifest.json"
    )

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name).resolve()
        self.output = self.root.parent / f"{self.root.name}-analysis"

    def tearDown(self):
        if self.output.is_symlink():
            self.output.unlink()
        elif self.output.exists():
            for path in self.output.iterdir():
                path.unlink()
            self.output.rmdir()
        self.temporary.cleanup()

    def build_bundle(self, rows=None):
        baseline, default_rows = paired_fixture()
        rows = default_rows if rows is None else rows
        baseline_path = self.root / "baseline.jsonl.gz"
        raw_root = self.root / "raw"
        raw_root.mkdir()
        raw_path = raw_root / replay.RAW_PROCESS_NAME
        manifest_path = raw_root / replay.TERMINAL_MANIFEST_NAME
        protocol_path = self.root / "protocol.json"
        truth_path = self.root / "truth.json"
        with gzip.open(baseline_path, "wt") as target:
            for row in baseline:
                target.write(json.dumps(row) + "\n")
        decompressed = b"".join(
            (json.dumps(row) + "\n").encode() for row in rows
        )
        with raw_path.open("wb") as raw_file:
            with gzip.GzipFile(
                filename="", fileobj=raw_file, mode="wb", mtime=0
            ) as target:
                target.write(decompressed)
        truth_path.write_text(json.dumps(truth_data()))
        protocol_value = {
            "schema_id": analyzer.ANALYZER_TEST_SCHEMA_ID,
            "sources": {
                "truth_data": {
                    "path": str(truth_path),
                    "sha256": sha256(truth_path),
                },
                "baseline_process": {
                    "path": str(baseline_path),
                    "sha256": sha256(baseline_path),
                },
            },
            "experiment": {
                **protocol()["experiment"],
                "evidence_class": "hermetic_non_evidence_only",
            },
            "ablation_contracts": protocol()["ablation_contracts"],
            "estimator_constants": protocol()["estimator_constants"],
            "gates": protocol()["gates"],
            "raw_schema": copy.deepcopy(replay.RAW_SCHEMA_DECLARATION),
            "analysis_schema": copy.deepcopy(replay.ANALYSIS_SCHEMA),
            "disk_contract": copy.deepcopy(replay.DISK_CONTRACT),
            "invocations": {
                "analyzer_test": {
                    "kind": "analyzer_test_only",
                    "development_manifest_path": str(manifest_path),
                    "output_root": str(self.output),
                    "expected_baseline_sha256": sha256(baseline_path),
                }
            },
        }
        protocol_path.write_text(json.dumps(protocol_value, sort_keys=True))
        manifest = {
            "status": "completed",
            "schema_id": replay.RAW_SCHEMA_ID,
            "raw_schema": replay.RAW_SCHEMA_DECLARATION,
            "rows_written": len(rows),
            "expected_rows": len(rows),
            "compressed_process_sha256": sha256(raw_path),
            "decompressed_process_sha256": hashlib.sha256(decompressed).hexdigest(),
        }
        manifest_path.write_text(json.dumps(manifest))
        return {
            "baseline": baseline_path,
            "raw": raw_path,
            "manifest": manifest_path,
            "protocol": protocol_path,
            "truth": truth_path,
            "manifest_value": manifest,
        }

    def execute(self, bundle):
        return analyzer.analyze_predictive_recovery(
            baseline_process_path=bundle["baseline"],
            development_manifest_path=bundle["manifest"],
            protocol_path=bundle["protocol"],
            expected_baseline_sha256=sha256(bundle["baseline"]),
            output_root=self.output,
        )

    def test_v4_analyzer_rejects_retired_v3_protocol_before_rebinding(self):
        """Breaks if the retired v3 protocol can authorize the v4 analyzer."""
        bundle = self.build_bundle()
        protocol_value = json.loads(bundle["protocol"].read_text())
        protocol_value["schema_id"] = (
            "cbf2026-predictive-wnls-stage1-protocol-v3"
        )
        protocol_value["protocol_id"] = "cbf2026-predictive-wnls-stage1-v3"
        bundle["protocol"].write_text(
            json.dumps(protocol_value, sort_keys=True)
        )
        with self.assertRaisesRegex(
            ValueError, "protocol does not authorize analyzer execution"
        ):
            self.execute(bundle)
        self.assertFalse(self.output.exists())

    def test_v4_analyzer_rejects_retired_raw_v2_forensic_manifest(self):
        """Breaks if raw-schema-v2 evidence can be rebound to v4 analysis."""
        bundle = self.build_bundle()
        retired_schema = copy.deepcopy(replay.RAW_SCHEMA_DECLARATION)
        retired_schema["id"] = (
            "cbf2026-predictive-wnls-development-rows-v2"
        )
        manifest = json.loads(bundle["manifest"].read_text())
        manifest["schema_id"] = retired_schema["id"]
        manifest["raw_schema"] = retired_schema
        bundle["manifest"].write_text(json.dumps(manifest, sort_keys=True))
        with self.assertRaisesRegex(
            ValueError, "development replay manifest is not complete and exact"
        ):
            self.execute(bundle)
        self.assertFalse(self.output.exists())

    def _bind_actual_forensic_manifest(self, bundle, manifest_path):
        protocol_value = json.loads(bundle["protocol"].read_text())
        protocol_value["invocations"]["analyzer_test"][
            "development_manifest_path"
        ] = str(manifest_path)
        bundle["protocol"].write_text(
            json.dumps(protocol_value, sort_keys=True)
        )
        bundle["manifest"] = manifest_path

    @unittest.skipUnless(
        V3_RAW_MANIFEST.exists(),
        "preserved v3 raw manifest is unavailable",
    )
    def test_actual_v3_raw_forensic_bundle_cannot_rebind_to_v4_analyzer(self):
        """Breaks if the preserved successful v3 raw root can be rebound."""
        bundle = self.build_bundle()
        self._bind_actual_forensic_manifest(bundle, self.V3_RAW_MANIFEST)
        with self.assertRaisesRegex(
            ValueError, "development replay manifest is not complete and exact"
        ):
            self.execute(bundle)
        self.assertFalse(self.output.exists())

    @unittest.skipUnless(
        V3_FAILED_ANALYZER_MANIFEST.exists(),
        "preserved v3 failed analyzer manifest is unavailable",
    )
    def test_actual_v3_failed_analyzer_manifest_cannot_rebind_as_raw(self):
        """Breaks if the preserved failed v3 terminal can authorize analysis."""
        bundle = self.build_bundle()
        self._bind_actual_forensic_manifest(
            bundle, self.V3_FAILED_ANALYZER_MANIFEST
        )
        with self.assertRaisesRegex(
            ValueError, "development replay manifest is not complete and exact"
        ):
            self.execute(bundle)
        self.assertFalse(self.output.exists())

    def test_v4_analysis_schema_is_pinned(self):
        """Breaks if compact outputs can silently retain the v2 schema."""
        self.assertEqual(
            analyzer.ANALYSIS_SCHEMA_ID,
            "cbf2026-predictive-wnls-development-analysis-v3",
        )
        self.assertEqual(
            replay.ANALYSIS_SCHEMA["id"],
            analyzer.ANALYSIS_SCHEMA_ID,
        )

    def test_exclusive_root_success_cap_and_terminal_manifest(self):
        """Breaks if compact evidence can overwrite or publish without a terminal."""
        bundle = self.build_bundle()
        completed = self.execute(bundle)
        self.assertEqual(completed["status"], "completed")
        self.assertEqual(
            set(path.name for path in self.output.iterdir()),
            {
                analyzer.OUTPUT_JSON_NAME,
                analyzer.OUTPUT_MARKDOWN_NAME,
                analyzer.ANALYZER_MANIFEST_NAME,
            },
        )
        self.assertLessEqual(
            completed["allocated_bytes"], analyzer.COMPACT_OUTPUT_CAP_BYTES
        )
        terminal = json.loads(
            (self.output / analyzer.ANALYZER_MANIFEST_NAME).read_text()
        )
        self.assertEqual(terminal["status"], "completed")
        markdown = (self.output / analyzer.OUTPUT_MARKDOWN_NAME).read_text()
        for label in (
            "Baseline-published attrition",
            "Baseline-fresh transitions",
            "Calibration denominators",
            "Input-limit audit: 1/6",
            "frame44_recovery",
            "legacy_999m_stale",
            "legacy_168m_fresh",
        ):
            self.assertIn(label, markdown)
        with self.assertRaises(FileExistsError):
            self.execute(bundle)

    def test_failure_after_allocation_writes_failed_terminal(self):
        """Breaks if an allocated failed analysis erases failure evidence."""
        _, rows = paired_fixture()
        rows[-1]["prediction_age"] = 3
        bundle = self.build_bundle(rows)
        with self.assertRaises(ValueError):
            self.execute(bundle)
        terminal = json.loads(
            (self.output / analyzer.ANALYZER_MANIFEST_NAME).read_text()
        )
        self.assertEqual(terminal["status"], "failed")

    def test_authority_hash_row_count_and_decompressed_hash_fail_preallocation(self):
        """Breaks if manifest/source drift can allocate an evidence root."""
        mutations = (
            ("expected_rows", 1),
            ("compressed_process_sha256", "0" * 64),
            ("decompressed_process_sha256", "0" * 64),
        )
        for field, value in mutations:
            with self.subTest(field=field):
                bundle = self.build_bundle()
                manifest = json.loads(bundle["manifest"].read_text())
                manifest[field] = value
                bundle["manifest"].write_text(json.dumps(manifest))
                with self.assertRaises(ValueError):
                    self.execute(bundle)
                self.assertFalse(self.output.exists())
                bundle["raw"].parent.joinpath(replay.TERMINAL_MANIFEST_NAME).unlink()
                bundle["raw"].unlink()
                bundle["raw"].parent.rmdir()

    def test_compact_cap_failure_retains_failed_terminal(self):
        """Breaks if the JSON/Markdown/manifest aggregate can exceed 10 MB."""
        bundle = self.build_bundle()
        with mock.patch.object(analyzer, "COMPACT_OUTPUT_CAP_BYTES", 1):
            with self.assertRaises(DiskSpaceError):
                self.execute(bundle)
        terminal = json.loads(
            (self.output / analyzer.ANALYZER_MANIFEST_NAME).read_text()
        )
        self.assertEqual(terminal["status"], "failed")

    def test_symlink_and_preexisting_targets_fail_closed(self):
        """Breaks if exact-root allocation follows or reuses a target."""
        bundle = self.build_bundle()
        self.output.symlink_to(self.root)
        with self.assertRaises((FileExistsError, ValueError)):
            self.execute(bundle)
        self.assertTrue(self.output.is_symlink())

    def test_raw_identity_drift_before_completed_becomes_failed_terminal(self):
        """Breaks if final source revalidation is omitted."""
        bundle = self.build_bundle()
        original = analyzer._reverify_inputs
        mutated = False

        def mutate_then_verify(*args, **kwargs):
            nonlocal mutated
            if not mutated:
                mutated = True
                bundle["raw"].write_bytes(bundle["raw"].read_bytes() + b"x")
            return original(*args, **kwargs)

        with mock.patch.object(
            analyzer, "_reverify_inputs", side_effect=mutate_then_verify
        ):
            with self.assertRaises(ValueError):
                self.execute(bundle)
        terminal = json.loads(
            (self.output / analyzer.ANALYZER_MANIFEST_NAME).read_text()
        )
        self.assertEqual(terminal["status"], "failed")

    def test_durable_completed_manifest_wins_post_commit_interrupt(self):
        """Breaks if an asynchronous delivery fault rewrites valid success."""
        bundle = self.build_bundle()
        with mock.patch.object(
            replay, "_post_commit_boundary", side_effect=KeyboardInterrupt
        ):
            completed = self.execute(bundle)
        self.assertEqual(completed["status"], "completed")
        terminal = json.loads(
            (self.output / analyzer.ANALYZER_MANIFEST_NAME).read_text()
        )
        self.assertEqual(terminal["status"], "completed")
        self.assertFalse(
            any(
                path.name.startswith("manifest.failed")
                for path in self.output.iterdir()
            )
        )

    def test_link_then_interrupt_reconcile_cleans_retained_stage(self):
        """Breaks if completed reconciliation leaks its parent staging entry."""
        bundle = self.build_bundle()
        original = replay._link_stage

        def link_then_interrupt(transaction, stage):
            original(transaction, stage)
            if stage["name"].endswith(".completed"):
                raise KeyboardInterrupt

        with mock.patch.object(replay, "_link_stage", side_effect=link_then_interrupt):
            completed = self.execute(bundle)
        self.assertEqual(completed["status"], "completed")
        self.assertFalse(
            any(
                self.output.parent.glob(
                    f".{self.output.name}.manifest.completed*"
                )
            )
        )


if __name__ == "__main__":
    unittest.main()
