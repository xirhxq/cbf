"""Geometry primitives for the immutable geometric-stability diagnostic."""

from __future__ import annotations

import argparse
import ctypes
import errno
import hashlib
import json
import math
import os
import stat
import subprocess
import sys
import uuid
from itertools import zip_longest
from pathlib import Path
from typing import Callable

import numpy as np

import scripts.diagnostics.compare_warm_start_recovery as comparison_module
from scripts.diagnostics.analyze_localization_failures import (
    _normalized_squared_error,
)
from scripts.diagnostics.compare_warm_start_recovery import InputIntegrityError
from scripts.diagnostics.replay_localization_calibration import (
    RESTART_BEFORE_FIRST_FINITE_POLICY,
    STRICT_PREVIOUS_POLICY,
    active_references,
    fixed_references,
)
from scripts.diagnostics.run_diagnostic import (
    HARD_FLOOR_BYTES as _RUN_HARD_FLOOR_BYTES,
    START_BYTES as _RUN_START_BYTES,
    available_bytes,
)


SCHEMA_ID = "cbf2026-geometric-stability-v1"
TIME_BIN_SECONDS = 50.0
OUTPUT_JSON_NAME = "geometric-stability.json"
OUTPUT_MARKDOWN_NAME = "geometric-stability.md"
COMPARISON_SCHEMA_ID = "cbf2026-warm-start-recovery-comparison-v1"
TIME_BIN_LABELS = (
    "0_to_lt_50_s",
    "50_to_lt_100_s",
    "100_to_lt_150_s",
    "150_to_lt_200_s",
    "200_to_250_s",
)
_PERCENTILES = (50.0, 90.0, 95.0, 99.0)
_DYNAMIC_CASE = "dynamic_dag_wnls"
_MISSING = object()
START_BYTES = 8_000_000_000
HARD_FLOOR_BYTES = 6_000_000_000
OUTPUT_CAP_BYTES = 10_000_000
LIVE_CHECK_INTERVAL_ROWS = 10_000
_PIN_FSTAT = os.fstat

if (START_BYTES, HARD_FLOOR_BYTES) != (
    _RUN_START_BYTES,
    _RUN_HARD_FLOOR_BYTES,
):
    raise RuntimeError("geometric-stability disk thresholds must match the runner")


class AnalysisLimitError(RuntimeError):
    """Raised when a geometric-stability analysis exceeds its safe limits."""


def _point(value: object, name: str) -> np.ndarray:
    try:
        point = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError) as error:
        raise InputIntegrityError(f"{name} is not a numeric point") from error
    if point.shape != (2,) or not np.isfinite(point).all():
        raise InputIntegrityError(f"{name} is not a finite 2D point")
    return point


def _references(value: object, count: int | None = None) -> np.ndarray:
    try:
        points = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError) as error:
        raise InputIntegrityError("geometry references are not numeric") from error
    if points.ndim != 2 or points.shape[1:] != (2,) or (
        count is not None and points.shape[0] != count
    ) or (count is None and points.shape[0] < 2):
        requirement = "exactly two" if count == 2 else "at least two"
        raise InputIntegrityError(f"geometry requires {requirement} 2D references")
    if not np.isfinite(points).all():
        raise InputIntegrityError("geometry contains a coincident or non-finite reference")
    return points


def _directions(center: np.ndarray, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    differences = center[None, :] - points
    ranges = np.hypot(differences[:, 0], differences[:, 1])
    if (
        not np.isfinite(differences).all()
        or not np.isfinite(ranges).all()
        or np.any(ranges <= 0.0)
    ):
        raise InputIntegrityError("geometry contains a coincident or non-finite reference")
    directions = differences / ranges[:, None]
    if not np.isfinite(directions).all():
        raise InputIntegrityError("geometry directions are not finite")
    return differences, directions


def geometry_metrics(observer: object, references: object) -> dict:
    """Return count, eigenvalues, normalized minimum, and finite/SPD flags."""
    center = _point(observer, "observer")
    points = _references(references)
    _, directions = _directions(center, points)
    matrix = directions.T @ directions
    eigenvalues = np.linalg.eigvalsh(matrix)
    minimum, maximum = map(float, eigenvalues)
    if (
        not np.isfinite(matrix).all()
        or not np.isfinite(eigenvalues).all()
        or maximum <= 0.0
    ):
        raise InputIntegrityError("geometry matrix or eigenvalues are not finite")
    normalized_minimum = minimum / maximum
    if not math.isfinite(normalized_minimum):
        raise InputIntegrityError("normalized geometry eigenvalue is not finite")
    return {
        "reference_count": int(points.shape[0]),
        "lambda_min": minimum,
        "lambda_max": maximum,
        "normalized_lambda_min": normalized_minimum,
        "finite": True,
        "positive_definite": minimum > 0.0,
    }


def fixed_pair_metrics(observer: object, references: object) -> dict:
    """Return angular and area metrics for exactly two fixed references."""
    center = _point(observer, "observer")
    points = _references(references, count=2)
    differences, directions = _directions(center, points)
    cosine = float(np.clip(np.dot(directions[0], directions[1]), -1.0, 1.0))
    included_angle = float(np.arccos(cosine))
    with np.errstate(over="ignore", invalid="ignore"):
        twice_triangle_area = abs(float(np.linalg.det(differences)))
    metrics = {
        "included_angle_rad": included_angle,
        "noncollinearity_angle_rad": min(included_angle, np.pi - included_angle),
        "absolute_cosine": abs(cosine),
        "twice_triangle_area": twice_triangle_area,
    }
    if not all(math.isfinite(value) for value in metrics.values()):
        raise InputIntegrityError("fixed-pair geometry metrics are not finite")
    return metrics


def _finite_distance(first: np.ndarray, second: np.ndarray, name: str) -> float:
    difference = first - second
    with np.errstate(over="ignore", invalid="ignore"):
        distance = float(np.hypot(difference[0], difference[1]))
    if not math.isfinite(distance):
        raise InputIntegrityError(f"{name} is not finite")
    return distance


def nominal_fixed_pair_metrics(
    observer_target: object,
    reference_targets: object,
) -> dict:
    """Return nominal fixed-pair geometry and strict perturbation supremum."""
    observer = _point(observer_target, "observer target")
    references = _references(reference_targets, count=2)
    first_length = _finite_distance(
        observer,
        references[0],
        "first nominal observer-reference side",
    )
    second_length = _finite_distance(
        observer,
        references[1],
        "second nominal observer-reference side",
    )
    reference_length = _finite_distance(
        references[0],
        references[1],
        "nominal reference-pair side",
    )
    pair = fixed_pair_metrics(observer, references)
    slacks = (
        first_length + second_length - reference_length,
        first_length + reference_length - second_length,
        second_length + reference_length - first_length,
    )
    minimum_slack = min(slacks)
    if (
        first_length <= 0.0
        or second_length <= 0.0
        or reference_length <= 0.0
        or pair["twice_triangle_area"] <= 0.0
        or not all(math.isfinite(value) and value > 0.0 for value in slacks)
    ):
        raise InputIntegrityError(
            "nominal fixed-reference target triangle is not strictly nondegenerate"
        )
    supremum = minimum_slack / 6.0
    if not math.isfinite(supremum) or supremum <= 0.0:
        raise InputIntegrityError(
            "nominal tracking-deviation supremum is not finite and positive"
        )
    return {
        **geometry_metrics(observer, references),
        **pair,
        "observer_reference_1_length": first_length,
        "observer_reference_2_length": second_length,
        "reference_pair_length": reference_length,
        "minimum_triangle_slack": minimum_slack,
        "admissible_tracking_deviation_supremum": supremum,
    }


def fixed_pair_tracking_margin(
    *,
    observer_estimate: object,
    reference_estimates: object,
    observer_target: object,
    reference_targets: object,
) -> dict:
    """Measure the strict three-vertex target-tracking perturbation margin."""
    observer = _point(observer_estimate, "observer estimate")
    estimates = _references(reference_estimates, count=2)
    target = _point(observer_target, "observer target")
    targets = _references(reference_targets, count=2)
    nominal = nominal_fixed_pair_metrics(target, targets)
    deviations = (
        _finite_distance(observer, target, "observer target deviation"),
        _finite_distance(
            estimates[0],
            targets[0],
            "first reference target deviation",
        ),
        _finite_distance(
            estimates[1],
            targets[1],
            "second reference target deviation",
        ),
    )
    maximum_deviation = max(deviations)
    supremum = nominal["admissible_tracking_deviation_supremum"]
    margin = supremum - maximum_deviation
    if not math.isfinite(margin):
        raise InputIntegrityError(
            "fixed-pair target-tracking margin is not finite"
        )
    return {
        "admissible_tracking_deviation_supremum": supremum,
        "maximum_vertex_target_deviation": maximum_deviation,
        "tracking_deviation_margin": margin,
        "strictly_within_admissible_tracking_deviation": (
            maximum_deviation < supremum
        ),
    }


def shared_uav_ancestor_metrics(
    active_uav_references: list[int],
    lineage_by_robot: dict[int, frozenset[int]],
) -> dict:
    """Count UAV-reference pairs with shared uncertain UAV ancestry."""
    if (
        not isinstance(active_uav_references, list)
        or any(type(reference) is not int for reference in active_uav_references)
        or len(active_uav_references) != len(set(active_uav_references))
    ):
        raise InputIntegrityError(
            "active UAV references must be a unique integer list"
        )
    pair_count = 0
    pairs_with_shared = 0
    shared_instances = 0
    for left_index, left in enumerate(active_uav_references):
        if left not in lineage_by_robot:
            raise InputIntegrityError(
                f"missing UAV-reference lineage for robot {left}"
            )
        left_lineage = lineage_by_robot[left]
        if not isinstance(left_lineage, frozenset) or any(
            type(ancestor) is not int for ancestor in left_lineage
        ):
            raise InputIntegrityError("UAV-reference lineage is invalid")
        for right in active_uav_references[left_index + 1 :]:
            if right not in lineage_by_robot:
                raise InputIntegrityError(
                    f"missing UAV-reference lineage for robot {right}"
                )
            right_lineage = lineage_by_robot[right]
            if not isinstance(right_lineage, frozenset) or any(
                type(ancestor) is not int for ancestor in right_lineage
            ):
                raise InputIntegrityError("UAV-reference lineage is invalid")
            shared = left_lineage & right_lineage
            pair_count += 1
            pairs_with_shared += int(bool(shared))
            shared_instances += len(shared)
    return {
        "reference_pair_count": pair_count,
        "pairs_with_shared_uav_ancestor": pairs_with_shared,
        "shared_uav_ancestor_instances": shared_instances,
    }


def opposite_baseline_side(
    estimate: object,
    truth: object,
    reference_positions: object,
    *,
    tolerance_metres: float = 1e-6,
) -> bool | None:
    """Return branch-side mismatch for exactly two references, or None near baseline."""
    if (
        type(tolerance_metres) not in (int, float)
        or not math.isfinite(float(tolerance_metres))
        or tolerance_metres < 0.0
    ):
        raise InputIntegrityError(
            "baseline-side tolerance must be finite and non-negative"
        )
    estimated_point = _point(estimate, "estimate")
    truth_point = _point(truth, "truth")
    references = _references(reference_positions, count=2)
    baseline = references[1] - references[0]
    length = float(np.hypot(baseline[0], baseline[1]))
    if not math.isfinite(length) or length <= 0.0:
        raise InputIntegrityError(
            "baseline-side diagnostic requires distinct references"
        )

    def signed_distance(point: np.ndarray) -> float:
        offset = point - references[0]
        return float(
            (baseline[0] * offset[1] - baseline[1] * offset[0])
            / length
        )

    estimate_distance = signed_distance(estimated_point)
    truth_distance = signed_distance(truth_point)
    if (
        abs(estimate_distance) <= tolerance_metres
        or abs(truth_distance) <= tolerance_metres
    ):
        return None
    return bool(estimate_distance * truth_distance < 0.0)


def load_trajectory(path: Path, *, expected_sha256: str) -> dict:
    """Verify and load the immutable configuration and trajectory data."""
    source = Path(path)
    before = _sha256(source)
    if before != expected_sha256:
        raise InputIntegrityError("trajectory does not match its external trust root")
    try:
        payload = json.loads(
            source.read_bytes(),
            parse_constant=lambda token: (_ for _ in ()).throw(
                ValueError(f"non-standard JSON constant: {token}")
            ),
        )
    except (OSError, ValueError, UnicodeDecodeError) as error:
        raise InputIntegrityError(f"invalid trajectory JSON: {error}") from error
    if not isinstance(payload, dict):
        raise InputIntegrityError("trajectory must be a JSON object")
    config = _mapping(payload.get("config"), "config")
    frames = payload.get("state")
    if not isinstance(frames, list):
        raise InputIntegrityError("state must be a list of frames")
    time_step = _finite_positive(
        _mapping(config.get("execute"), "config.execute").get("time-step"),
        "config.execute.time-step",
    )
    num_robots = config.get("num")
    if type(num_robots) is not int or num_robots <= 0:
        raise InputIntegrityError("config.num must be a positive integer")
    expected_ids = set(range(1, num_robots + 1))
    truth: dict[int, dict[int, list[float]]] = {}
    targets: dict[int, dict[int, list[float]]] = {}
    for frame_index, raw_frame in enumerate(frames):
        frame = _mapping(raw_frame, f"state[{frame_index}]")
        robots = frame.get("robots")
        if not isinstance(robots, list):
            raise InputIntegrityError(f"state[{frame_index}].robots must be a list")
        frame_truth: dict[int, list[float]] = {}
        frame_targets: dict[int, list[float]] = {}
        for robot_index, raw_robot in enumerate(robots):
            robot = _mapping(raw_robot, f"state[{frame_index}].robots[{robot_index}]")
            robot_id = robot.get("id")
            if type(robot_id) is not int or robot_id not in expected_ids or robot_id in frame_truth:
                raise InputIntegrityError("each frame must contain each configured robot exactly once")
            state = _mapping(robot.get("state"), "robot.state")
            position = _point([state.get("x"), state.get("y")], "robot.state")
            cvt = _mapping(robot.get("cvt"), "robot.cvt")
            target = _point(cvt.get("center"), "robot.cvt.center")
            frame_truth[robot_id] = position.tolist()
            frame_targets[robot_id] = target.tolist()
        if set(frame_truth) != expected_ids:
            raise InputIntegrityError("each frame must contain each configured robot exactly once")
        truth[frame_index] = frame_truth
        targets[frame_index] = frame_targets
    if _sha256(source) != before:
        raise InputIntegrityError("trajectory changed while it was read")
    return {
        "path": str(source.resolve()),
        "sha256": expected_sha256,
        "config": config,
        "frame_count": len(frames),
        "time_step": time_step,
        "truth": truth,
        "targets": targets,
    }


class _ScientificAccumulator:
    def __init__(self, trajectory: dict, seeds: list[int]) -> None:
        self.trajectory = trajectory
        self.config = trajectory["config"]
        self.seeds = tuple(seeds)
        self.squad_size = math.ceil(
            self.config["num"] / self.config["formation"]["parts"]
        )
        self.estimated_positions: dict[
            tuple[int, int], dict[int, np.ndarray]
        ] = {}
        self.lineage_by_context: dict[
            tuple[int, int, int], dict[int, frozenset[int]]
        ] = {}
        self.true_signatures: dict[tuple[int, int], tuple] = {}
        self.true_seed_repetitions: dict[
            tuple[int, int], set[int]
        ] = {}
        self.geometry_samples = {
            "nominal_fixed_pair_all_primary": _geometry_family(),
            "true_dynamic_all_primary": _geometry_family(),
            "true_fixed_pair_all_primary": _geometry_family(),
            "estimated_dynamic_finite": _geometry_family(),
            "estimated_fixed_pair_finite": _geometry_family(),
        }
        self.fixed_pair_tracking_margin = _geometry_family()
        self.modeled_fim = _geometry_family()
        self.tracking_true = _stratified_samples()
        self.tracking_estimated = _stratified_samples()
        self.absolute_error = _stratified_samples()
        self.calibration = _stratified_calibration()
        self.mechanisms = {
            "by_uav_reference_count": {},
            "by_shared_uav_ancestor_pair_count": {},
            "two_reference_opposite_baseline_side": {},
        }
        self.availability = _stratified_availability()
        self.series: dict[tuple[int, int], dict] = {}

    def record(self, row: dict) -> None:
        if row["graph_case"] != _DYNAMIC_CASE:
            return
        frame_index = row["frame_index"]
        seed = row["seed"]
        robot_id = row["robot_id"]
        squad = (robot_id - 1) // self.squad_size + 1
        depth = (robot_id - 1) % self.squad_size + 1
        if row["squad_local_index"] != depth:
            raise InputIntegrityError(
                f"squad-local depth does not match config at "
                f"{comparison_module._compact_key(row)!r}"
            )
        time_bin = _time_bin(
            frame_index * self.trajectory["time_step"]
        )
        strata = {
            "depth": str(depth),
            "time_bin": time_bin,
            "squad": str(squad),
            "seed": str(seed),
        }
        context = (frame_index, seed)
        estimated = self.estimated_positions.setdefault(context, {})
        observer_estimate = _optional_finite_point(row.get("estimate"))
        truth_by_robot = self.trajectory["truth"][frame_index]
        immutable_truth = _point(
            truth_by_robot[robot_id],
            "immutable trajectory truth",
        )
        error_state = _reconcile_error_state(
            row,
            immutable_truth,
            observer_estimate,
        )
        expected_active = active_references(
            self.config,
            robot_id,
            {
                identifier: np.asarray(position, dtype=float)
                for identifier, position in truth_by_robot.items()
            },
        )
        if row["active_references"] != expected_active:
            raise InputIntegrityError(
                f"recorded dynamic references do not match frozen "
                f"config and truth at "
                f"{comparison_module._compact_key(row)!r}"
            )
        active_uav_references = expected_active["uav_ids"]
        lineage = self.lineage_by_context.setdefault(
            (frame_index, seed, squad), {}
        )
        for reference_id in active_uav_references:
            if reference_id not in lineage:
                raise InputIntegrityError(
                    f"active UAV lineage is unavailable at "
                    f"{comparison_module._compact_key(row)!r}"
                )
        dependency = shared_uav_ancestor_metrics(
            active_uav_references, lineage
        )
        lineage[robot_id] = frozenset(
            {robot_id}
            | {
                ancestor
                for reference_id in active_uav_references
                for ancestor in lineage[reference_id]
            }
        )
        self._record_availability(row, strata, depth, squad)

        if row.get("primary_statistics") is True:
            truth = immutable_truth
            active = _active_reference_positions(
                expected_active,
                self.config,
                truth_by_robot,
            )
            fixed_ids = fixed_references(self.config, robot_id)
            _require_fixed_subset(
                fixed_ids,
                expected_active,
                comparison_module._compact_key(row),
            )
            fixed_truth = _active_reference_positions(
                fixed_ids,
                self.config,
                self.trajectory["truth"][frame_index],
            )
            targets_by_robot = self.trajectory["targets"][frame_index]
            observer_target = _point(
                targets_by_robot[robot_id],
                "observer trajectory target",
            )
            fixed_targets = _fixed_reference_targets(
                fixed_ids,
                self.config,
                targets_by_robot,
            )
            nominal_metrics = nominal_fixed_pair_metrics(
                observer_target,
                fixed_targets,
            )
            signature = (
                tuple(row["truth_position"]),
                tuple(expected_active["base_ids"]),
                tuple(expected_active["uav_ids"]),
                tuple(tuple(point) for point in active.tolist()),
                tuple(observer_target.tolist()),
                tuple(tuple(point) for point in fixed_targets.tolist()),
            )
            tuple_key = (frame_index, robot_id)
            previous_signature = self.true_signatures.setdefault(
                tuple_key, signature
            )
            if signature != previous_signature:
                raise InputIntegrityError(
                    "true geometry differs across range-noise seeds at "
                    f"{tuple_key!r}"
                )
            repetitions = self.true_seed_repetitions.setdefault(
                tuple_key, set()
            )
            if seed in repetitions:
                raise InputIntegrityError(
                    f"duplicate true-geometry seed repetition at {tuple_key!r}"
                )
            repetitions.add(seed)
            if len(repetitions) == 1:
                _record_geometry(
                    self.geometry_samples[
                        "nominal_fixed_pair_all_primary"
                    ],
                    nominal_metrics,
                    strata,
                )
                _record_geometry(
                    self.geometry_samples[
                        "true_dynamic_all_primary"
                    ],
                    geometry_metrics(truth, active),
                    strata,
                )
                _record_geometry(
                    self.geometry_samples[
                        "true_fixed_pair_all_primary"
                    ],
                    {
                        **geometry_metrics(truth, fixed_truth),
                        **fixed_pair_metrics(truth, fixed_truth),
                    },
                    strata,
                )
                _record_sample(
                    self.tracking_true,
                    _finite_distance(
                        truth,
                        observer_target,
                        "true-position target deviation",
                    ),
                    strata,
                )

            estimated_active, active_reason = _estimated_references(
                expected_active,
                self.config,
                estimated,
            )
            estimated_fixed, fixed_reason = _estimated_references(
                fixed_ids,
                self.config,
                estimated,
            )
            side_label = _baseline_side_label(
                observer_estimate,
                truth,
                estimated_active,
                active_reason,
            )
            if row.get("finite") is True and observer_estimate is not None:
                modeled_metrics = _modeled_fim_metrics(row)
                _record_geometry(
                    self.modeled_fim,
                    modeled_metrics,
                    strata,
                )
                if active_reason is None:
                    _record_geometry(
                        self.geometry_samples[
                            "estimated_dynamic_finite"
                        ],
                        geometry_metrics(
                            observer_estimate, estimated_active
                        ),
                        strata,
                    )
                else:
                    _record_inapplicable(
                        self.geometry_samples[
                            "estimated_dynamic_finite"
                        ],
                        active_reason,
                        strata,
                    )
                if fixed_reason is None:
                    _record_geometry(
                        self.geometry_samples[
                            "estimated_fixed_pair_finite"
                        ],
                        {
                            **geometry_metrics(
                                observer_estimate, estimated_fixed
                            ),
                            **fixed_pair_metrics(
                                observer_estimate, estimated_fixed
                            ),
                        },
                        strata,
                    )
                    perturbation = fixed_pair_tracking_margin(
                        observer_estimate=observer_estimate,
                        reference_estimates=estimated_fixed,
                        observer_target=observer_target,
                        reference_targets=fixed_targets,
                    )
                    _record_geometry(
                        self.fixed_pair_tracking_margin,
                        {
                            key: (
                                int(value)
                                if type(value) is bool
                                else value
                            )
                            for key, value in perturbation.items()
                        },
                        strata,
                    )
                else:
                    _record_inapplicable(
                        self.geometry_samples[
                            "estimated_fixed_pair_finite"
                        ],
                        fixed_reason,
                        strata,
                    )
                    _record_inapplicable(
                        self.fixed_pair_tracking_margin,
                        fixed_reason,
                        strata,
                    )
                _record_sample(
                    self.tracking_estimated,
                    _finite_distance(
                        observer_estimate,
                        observer_target,
                        "estimated-position target deviation",
                    ),
                    strata,
                )
            else:
                reason = (
                    "restart_row_not_finite"
                    if row.get("finite") is False
                    else "estimate_not_finite"
                )
                _record_inapplicable(
                    self.geometry_samples[
                        "estimated_dynamic_finite"
                    ],
                    reason,
                    strata,
                )
                _record_inapplicable(
                    self.geometry_samples[
                        "estimated_fixed_pair_finite"
                    ],
                    reason,
                    strata,
                )
                _record_inapplicable(
                    self.fixed_pair_tracking_margin,
                    reason,
                    strata,
                )
                _record_sample_inapplicable(
                    self.tracking_estimated, reason, strata
                )
                _record_inapplicable(
                    self.modeled_fim,
                    reason,
                    strata,
                )
            self._record_error_and_calibration(
                row,
                strata,
                uav_reference_count=len(active_uav_references),
                shared_pair_count=dependency[
                    "pairs_with_shared_uav_ancestor"
                ],
                baseline_side_label=side_label,
                error_state=error_state,
            )

        if row.get("finite") is True:
            if observer_estimate is None:
                raise InputIntegrityError(
                    f"finite row has no finite estimate at "
                    f"{comparison_module._compact_key(row)!r}"
                )
            estimated[robot_id] = observer_estimate

    def _record_availability(
        self,
        row: dict,
        strata: dict[str, str],
        depth: int,
        squad: int,
    ) -> None:
        key = (row["seed"], row["robot_id"])
        series = self.series.setdefault(
            key,
            {
                "seed": row["seed"],
                "robot_id": row["robot_id"],
                "depth": depth,
                "squad": squad,
                "first_finite_frame": None,
                "current_unavailable": 0,
                "current_nonfinite": 0,
                "maximum_unavailable": 0,
                "maximum_nonfinite": 0,
            },
        )
        finite = row.get("finite") is True
        if finite and series["first_finite_frame"] is None:
            series["first_finite_frame"] = row["frame_index"]
        if row.get("primary_statistics") is not True:
            return
        attempt_status = row.get("attempt_status")
        if attempt_status not in {"converged", "invalid", "failed"}:
            raise InputIntegrityError(
                f"unknown attempt status at "
                f"{comparison_module._compact_key(row)!r}"
            )
        reason_value = row.get("attempt_failure_reason")
        if reason_value is None and attempt_status == "converged":
            reason = "converged"
        elif isinstance(reason_value, str) and reason_value:
            reason = reason_value
        else:
            reason = "unspecified"
        for bucket in _selected_buckets(self.availability, strata):
            bucket["primary_attempts"] += 1
            bucket["attempt_status_counts"][attempt_status] += 1
            reason_counts = bucket["attempt_failure_reason_counts"]
            reason_counts[reason] = reason_counts.get(reason, 0) + 1
            bucket["converged_attempts"] += int(
                attempt_status == "converged"
            )
            bucket["finite_retained_rows"] += int(finite)
            bucket["stale_retained_rows"] += int(
                row.get("status") == "stale"
            )
            bucket["invalid_attempts"] += int(
                attempt_status == "invalid"
            )
            bucket["failed_attempts"] += int(
                attempt_status == "failed"
            )
            if series["first_finite_frame"] is None:
                bucket["pre_acquisition_attempts"] += 1
            else:
                lag = row["frame_index"] - series["first_finite_frame"]
                lag_counts = bucket["frames_since_first_finite"]
                label = str(lag)
                lag_counts[label] = lag_counts.get(label, 0) + 1
        unavailable = attempt_status != "converged"
        nonfinite = not finite
        series["current_unavailable"] = (
            series["current_unavailable"] + 1 if unavailable else 0
        )
        series["current_nonfinite"] = (
            series["current_nonfinite"] + 1 if nonfinite else 0
        )
        series["maximum_unavailable"] = max(
            series["maximum_unavailable"],
            series["current_unavailable"],
        )
        series["maximum_nonfinite"] = max(
            series["maximum_nonfinite"],
            series["current_nonfinite"],
        )

    def _record_error_and_calibration(
        self,
        row: dict,
        strata: dict[str, str],
        *,
        uav_reference_count: int,
        shared_pair_count: int,
        baseline_side_label: str,
        error_state: dict,
    ) -> None:
        error_norm = error_state["error_norm"]
        error_finite = error_norm is not None
        if error_finite:
            _record_sample(
                self.absolute_error, float(error_norm), strata
            )
        else:
            _record_sample_inapplicable(
                self.absolute_error, "error_norm_not_finite", strata
            )
        if (
            row.get("attempt_status") != "converged"
            or not error_finite
        ):
            _record_calibration_inapplicable(
                self.calibration,
                "attempt_not_converged_or_error_missing",
                strata,
            )
            return
        mechanism_buckets = [
            _mechanism_bucket(
                self.mechanisms["by_uav_reference_count"],
                str(uav_reference_count),
            ),
            _mechanism_bucket(
                self.mechanisms[
                    "by_shared_uav_ancestor_pair_count"
                ],
                str(shared_pair_count),
            ),
            _mechanism_bucket(
                self.mechanisms[
                    "two_reference_opposite_baseline_side"
                ],
                baseline_side_label,
            ),
        ]
        for bucket in _selected_buckets(self.calibration, strata):
            bucket["converged_error_denominator"] += 1
            bucket["contained"] += int(
                error_state["state_containment"] is True
            )
        for bucket in mechanism_buckets:
            bucket["converged_error_denominator"] += 1
            bucket["contained"] += int(
                error_state["state_containment"] is True
            )
        try:
            q_value = _normalized_squared_error(
                error_state["error_vector"],
                row.get("covariance"),
            )
        except ValueError:
            _record_calibration_inapplicable(
                self.calibration,
                "invalid_covariance_or_error_vector",
                strata,
            )
            return
        for bucket in _selected_buckets(self.calibration, strata):
            bucket["finite_q_denominator"] += 1
            bucket["q_above_9"] += int(q_value > 9.0)
            bucket["q_samples"].append(float(q_value))
        for bucket in mechanism_buckets:
            bucket["finite_q_denominator"] += 1
            bucket["q_above_9"] += int(q_value > 9.0)

    def finish(self) -> dict:
        expected_seeds = set(self.seeds)
        if not self.true_seed_repetitions:
            raise InputIntegrityError("no primary dynamic rows were observed")
        for tuple_key, observed in self.true_seed_repetitions.items():
            if observed != expected_seeds:
                raise InputIntegrityError(
                    f"true geometry seed repetitions differ at {tuple_key!r}"
                )
        geometry = {
            name: _finish_geometry_family(family)
            for name, family in self.geometry_samples.items()
        }
        geometry["modeled_fim_valid"] = _finish_geometry_family(
            self.modeled_fim
        )
        geometry[
            "estimated_fixed_pair_tracking_margin_finite"
        ] = _finish_tracking_margin_family(
            self.fixed_pair_tracking_margin
        )
        true_tuple_count = len(self.true_seed_repetitions)
        for name in (
            "nominal_fixed_pair_all_primary",
            "true_dynamic_all_primary",
            "true_fixed_pair_all_primary",
        ):
            geometry[name]["overall"]["trajectory_tuple_count"] = (
                true_tuple_count
            )
            geometry[name]["overall"][
                "range_seed_repetitions_verified"
            ] = len(self.seeds)
            geometry[name].pop("by_seed", None)
            geometry[name]["seed_stratification"] = {
                "applicable": False,
                "reason": "trajectory_level_quantity",
            }
        true_tracking = _finish_stratified_samples(
            self.tracking_true
        )
        true_tracking.pop("by_seed", None)
        true_tracking["seed_stratification"] = {
            "applicable": False,
            "reason": "trajectory_level_quantity",
        }
        geometry["target_tracking"] = {
            "true_position": true_tracking,
            "estimated_position_finite": _finish_stratified_samples(
                self.tracking_estimated
            ),
        }
        availability = _finish_availability(
            self.availability,
            self.series,
            self.seeds,
            int(self.config["num"]),
        )
        return {
            "geometry": geometry,
            "absolute_error": _finish_stratified_samples(
                self.absolute_error
            ),
            "availability": availability,
            "calibration": {
                **_finish_stratified_calibration(self.calibration),
                "mechanism_diagnostics": {
                    name: {
                        label: dict(bucket)
                        for label, bucket in sorted(groups.items())
                    }
                    for name, groups in self.mechanisms.items()
                },
            },
        }


def _empty_samples() -> dict:
    return {"samples": [], "inapplicability_reasons": {}}


def _stratified_samples() -> dict:
    return {
        "overall": _empty_samples(),
        "by_depth": {},
        "by_time_bin": {
            label: _empty_samples() for label in TIME_BIN_LABELS
        },
        "by_squad": {},
        "by_seed": {},
    }


def _geometry_family() -> dict:
    return {
        "overall": {"samples": {}, "inapplicability_reasons": {}},
        "by_depth": {},
        "by_time_bin": {
            label: {"samples": {}, "inapplicability_reasons": {}}
            for label in TIME_BIN_LABELS
        },
        "by_squad": {},
        "by_seed": {},
    }


def _empty_availability() -> dict:
    return {
        "primary_attempts": 0,
        "attempt_status_counts": {
            "converged": 0,
            "invalid": 0,
            "failed": 0,
        },
        "attempt_failure_reason_counts": {},
        "converged_attempts": 0,
        "finite_retained_rows": 0,
        "stale_retained_rows": 0,
        "invalid_attempts": 0,
        "failed_attempts": 0,
        "pre_acquisition_attempts": 0,
        "frames_since_first_finite": {},
    }


def _stratified_availability() -> dict:
    return {
        "overall": _empty_availability(),
        "by_depth": {},
        "by_time_bin": {
            label: _empty_availability() for label in TIME_BIN_LABELS
        },
        "by_squad": {},
        "by_seed": {},
    }


def _empty_calibration() -> dict:
    return {
        "converged_error_denominator": 0,
        "contained": 0,
        "finite_q_denominator": 0,
        "q_above_9": 0,
        "q_samples": [],
        "inapplicability_reasons": {},
    }


def _stratified_calibration() -> dict:
    return {
        "overall": _empty_calibration(),
        "by_depth": {},
        "by_time_bin": {
            label: _empty_calibration() for label in TIME_BIN_LABELS
        },
        "by_squad": {},
        "by_seed": {},
    }


def _selected_buckets(family: dict, strata: dict[str, str]) -> list[dict]:
    selected = [family["overall"]]
    for family_name, key in (
        ("by_depth", "depth"),
        ("by_time_bin", "time_bin"),
        ("by_squad", "squad"),
        ("by_seed", "seed"),
    ):
        group = family[family_name]
        label = strata[key]
        if label not in group:
            exemplar = family["overall"]
            if "attempt_status_counts" in exemplar:
                group[label] = _empty_availability()
            elif "q_samples" in exemplar:
                group[label] = _empty_calibration()
            elif "samples" in exemplar and isinstance(
                exemplar["samples"], list
            ):
                group[label] = _empty_samples()
            else:
                group[label] = {
                    "samples": {},
                    "inapplicability_reasons": {},
                }
        selected.append(group[label])
    return selected


def _record_sample(
    family: dict, value: float, strata: dict[str, str]
) -> None:
    for bucket in _selected_buckets(family, strata):
        bucket["samples"].append(value)


def _record_sample_inapplicable(
    family: dict, reason: str, strata: dict[str, str]
) -> None:
    for bucket in _selected_buckets(family, strata):
        reasons = bucket["inapplicability_reasons"]
        reasons[reason] = reasons.get(reason, 0) + 1


def _record_geometry(
    family: dict, metrics: dict, strata: dict[str, str]
) -> None:
    numeric_metrics = {
        key: float(value)
        for key, value in metrics.items()
        if type(value) in (int, float) and type(value) is not bool
    }
    for bucket in _selected_buckets(family, strata):
        for name, value in numeric_metrics.items():
            bucket["samples"].setdefault(name, []).append(value)


def _record_inapplicable(
    family: dict, reason: str, strata: dict[str, str]
) -> None:
    for bucket in _selected_buckets(family, strata):
        reasons = bucket["inapplicability_reasons"]
        reasons[reason] = reasons.get(reason, 0) + 1


def _record_calibration_inapplicable(
    family: dict, reason: str, strata: dict[str, str]
) -> None:
    for bucket in _selected_buckets(family, strata):
        reasons = bucket["inapplicability_reasons"]
        reasons[reason] = reasons.get(reason, 0) + 1


def _sample_summary(samples: list[float]) -> dict:
    if not samples:
        return {
            "denominator": 0,
            "median": None,
            "p90": None,
            "p95": None,
            "p99": None,
            "maximum": None,
        }
    percentiles = np.percentile(np.asarray(samples), _PERCENTILES)
    return {
        "denominator": len(samples),
        "median": float(percentiles[0]),
        "p90": float(percentiles[1]),
        "p95": float(percentiles[2]),
        "p99": float(percentiles[3]),
        "maximum": float(np.max(samples)),
    }


def _finish_sample_bucket(bucket: dict) -> dict:
    return {
        **_sample_summary(bucket["samples"]),
        "inapplicability_reasons": dict(
            sorted(bucket["inapplicability_reasons"].items())
        ),
    }


def _finish_stratified_samples(family: dict) -> dict:
    return {
        "overall": _finish_sample_bucket(family["overall"]),
        **{
            name: {
                label: _finish_sample_bucket(bucket)
                for label, bucket in sorted(group.items())
            }
            for name, group in family.items()
            if name != "overall"
        },
    }


def _finish_geometry_bucket(bucket: dict) -> dict:
    metrics = {
        name: _sample_summary(samples)
        for name, samples in sorted(bucket["samples"].items())
    }
    denominators = {
        item["denominator"] for item in metrics.values()
    }
    denominator = 0 if not denominators else max(denominators)
    return {
        "denominator": denominator,
        "metrics": metrics,
        "inapplicability_reasons": dict(
            sorted(bucket["inapplicability_reasons"].items())
        ),
    }


def _finish_geometry_family(family: dict) -> dict:
    return {
        "overall": _finish_geometry_bucket(family["overall"]),
        **{
            name: {
                label: _finish_geometry_bucket(bucket)
                for label, bucket in sorted(group.items())
            }
            for name, group in family.items()
            if name != "overall"
        },
    }


def _finish_tracking_margin_bucket(bucket: dict) -> dict:
    result = _finish_geometry_bucket(bucket)
    indicators = bucket["samples"].get(
        "strictly_within_admissible_tracking_deviation",
        [],
    )
    if any(value not in (0.0, 1.0) for value in indicators):
        raise InputIntegrityError(
            "strict tracking-deviation indicators are invalid"
        )
    result["metrics"].pop(
        "strictly_within_admissible_tracking_deviation",
        None,
    )
    satisfied = int(sum(indicators))
    result["strictly_within_admissible_count"] = satisfied
    result["not_strictly_within_admissible_count"] = (
        len(indicators) - satisfied
    )
    return result


def _finish_tracking_margin_family(family: dict) -> dict:
    return {
        "overall": _finish_tracking_margin_bucket(family["overall"]),
        **{
            name: {
                label: _finish_tracking_margin_bucket(bucket)
                for label, bucket in sorted(group.items())
            }
            for name, group in family.items()
            if name != "overall"
        },
    }


def _finish_calibration_bucket(bucket: dict) -> dict:
    q_summary = _sample_summary(bucket["q_samples"])
    return {
        "converged_error_denominator": bucket[
            "converged_error_denominator"
        ],
        "contained": bucket["contained"],
        "finite_q_denominator": bucket["finite_q_denominator"],
        "q_above_9": bucket["q_above_9"],
        "q": q_summary,
        "inapplicability_reasons": dict(
            sorted(bucket["inapplicability_reasons"].items())
        ),
    }


def _finish_stratified_calibration(family: dict) -> dict:
    return {
        "overall": _finish_calibration_bucket(family["overall"]),
        **{
            name: {
                label: _finish_calibration_bucket(bucket)
                for label, bucket in sorted(group.items())
            }
            for name, group in family.items()
            if name != "overall"
        },
    }


def _finish_availability_bucket(bucket: dict) -> dict:
    return {
        key: (
            dict(sorted(value.items()))
            if isinstance(value, dict)
            else value
        )
        for key, value in bucket.items()
    }


def _finish_availability(
    family: dict,
    series: dict[tuple[int, int], dict],
    seeds: tuple[int, ...],
    robot_count: int,
) -> dict:
    expected_series = {
        (seed, robot_id)
        for seed in seeds
        for robot_id in range(1, robot_count + 1)
    }
    if set(series) != expected_series:
        raise InputIntegrityError(
            "availability seed-robot series do not match expected dimensions"
        )
    acquired = [
        item for item in series.values()
        if item["first_finite_frame"] is not None
    ]
    never = [
        item for item in series.values()
        if item["first_finite_frame"] is None
    ]
    frame_counts: dict[str, int] = {}
    for item in acquired:
        label = str(item["first_finite_frame"])
        frame_counts[label] = frame_counts.get(label, 0) + 1

    def lineage_group(key_name: str) -> dict[str, dict[str, int]]:
        labels = sorted(
            {str(item[key_name]) for item in series.values()},
            key=lambda label: int(label),
        )
        return {
            label: {
                "acquired_series": sum(
                    item["first_finite_frame"] is not None
                    and str(item[key_name]) == label
                    for item in series.values()
                ),
                "never_acquired_series": sum(
                    item["first_finite_frame"] is None
                    and str(item[key_name]) == label
                    for item in series.values()
                ),
            }
            for label in labels
        }

    result = {
        "overall": _finish_availability_bucket(family["overall"]),
        **{
            name: {
                label: _finish_availability_bucket(bucket)
                for label, bucket in sorted(group.items())
            }
            for name, group in family.items()
            if name != "overall"
        },
        "first_finite_acquisition": {
            "total_series": len(series),
            "acquired_series": len(acquired),
            "never_acquired_series": len(never),
            "first_finite_frame_counts": dict(sorted(frame_counts.items())),
            "by_depth": lineage_group("depth"),
            "by_squad": lineage_group("squad"),
            "by_seed": lineage_group("seed"),
        },
    }
    result["overall"][
        "maximum_consecutive_unavailable_frames"
    ] = max(item["maximum_unavailable"] for item in series.values())
    result["overall"][
        "maximum_consecutive_nonfinite_retained_frames"
    ] = max(item["maximum_nonfinite"] for item in series.values())
    return result


def _mechanism_bucket(target: dict, label: str) -> dict:
    return target.setdefault(
        label,
        {
            "converged_error_denominator": 0,
            "contained": 0,
            "finite_q_denominator": 0,
            "q_above_9": 0,
        },
    )


def _modeled_fim_metrics(row: dict) -> dict[str, float]:
    values = {
        "phi_min_eigenvalue": row.get("phi_min_eigenvalue"),
        "phi_condition": row.get("phi_condition"),
        "epsilon": row.get("epsilon"),
    }
    if any(
        type(value) not in (int, float)
        or not math.isfinite(float(value))
        for value in values.values()
    ):
        raise InputIntegrityError(
            f"finite row has invalid modeled FIM metrics at "
            f"{comparison_module._compact_key(row)!r}"
        )
    metrics = {
        name: float(value) for name, value in values.items()
    }
    if (
        metrics["phi_min_eigenvalue"] <= 0.0
        or metrics["phi_condition"] < 1.0
        or metrics["epsilon"] <= 0.0
    ):
        raise InputIntegrityError(
            f"finite row has out-of-domain modeled FIM metrics at "
            f"{comparison_module._compact_key(row)!r}"
        )
    return metrics


def _reconcile_error_state(
    row: dict,
    truth: np.ndarray,
    estimate: np.ndarray | None,
) -> dict:
    key = comparison_module._compact_key(row)
    raw_estimate = row.get("estimate")
    if raw_estimate is not None and estimate is None:
        raise InputIntegrityError(
            f"retained estimate is malformed at key {key!r}"
        )
    if estimate is None:
        error_vector = None
        error_norm = None
    else:
        vector = truth - estimate
        if not np.isfinite(vector).all():
            raise InputIntegrityError(
                f"recomputed error is not finite at key {key!r}"
            )
        error_vector = vector.tolist()
        error_norm = float(np.linalg.norm(vector))
        if not math.isfinite(error_norm):
            raise InputIntegrityError(
                f"recomputed error norm is not finite at key {key!r}"
            )

    epsilon = row.get("epsilon")
    if epsilon is not None and (
        type(epsilon) not in (int, float)
        or not math.isfinite(float(epsilon))
    ):
        raise InputIntegrityError(
            f"retained epsilon is invalid at key {key!r}"
        )
    if error_norm is None or epsilon is None:
        error_to_epsilon_ratio = None
        state_containment = None
    else:
        epsilon_value = float(epsilon)
        error_to_epsilon_ratio = (
            None
            if epsilon_value <= 0.0
            else error_norm / epsilon_value
        )
        state_containment = error_norm <= epsilon_value
    containment = (
        row.get("attempt_status") == "converged"
        and state_containment is True
    )
    expected = {
        "error_vector": error_vector,
        "error_norm": error_norm,
        "error_to_epsilon_ratio": error_to_epsilon_ratio,
        "state_containment": state_containment,
        "containment": containment,
    }
    for field, value in expected.items():
        if not _stored_error_value_equal(row.get(field), value):
            raise InputIntegrityError(
                f"stored {field} does not match immutable truth and "
                f"retained estimate at key {key!r}"
            )
    return {
        "error_vector": error_vector,
        "error_norm": error_norm,
        "state_containment": state_containment,
        "containment": containment,
    }


def _stored_error_value_equal(stored: object, expected: object) -> bool:
    if expected is None or type(expected) is bool:
        return stored is expected
    if isinstance(expected, list):
        return (
            isinstance(stored, list)
            and len(stored) == len(expected)
            and all(
                type(observed) is float and observed == wanted
                for observed, wanted in zip(stored, expected)
            )
        )
    return type(stored) is float and stored == expected


def _baseline_side_label(
    estimate: np.ndarray | None,
    truth: np.ndarray,
    estimated_references: np.ndarray | None,
    reference_reason: str | None,
) -> str:
    if reference_reason is not None or estimate is None:
        return "inapplicable_estimated_positions_unavailable"
    if estimated_references is None or len(estimated_references) != 2:
        return "inapplicable_not_exactly_two_references"
    mismatch = opposite_baseline_side(
        estimate,
        truth,
        estimated_references,
    )
    if mismatch is None:
        return "inapplicable_near_baseline"
    return "true" if mismatch else "false"


def _optional_finite_point(value: object) -> np.ndarray | None:
    try:
        point = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return None
    if point.shape != (2,) or not np.isfinite(point).all():
        return None
    return point


def _active_reference_positions(
    references: dict,
    config: dict,
    truth_by_robot: dict[int, list[float]],
) -> np.ndarray:
    positions = [
        _point(config["bases"][base_id], f"base {base_id}")
        for base_id in references["base_ids"]
    ]
    positions.extend(
        _point(
            truth_by_robot[reference_id],
            f"reference UAV {reference_id}",
        )
        for reference_id in references["uav_ids"]
    )
    if len(positions) < 2:
        raise InputIntegrityError("geometry requires at least two references")
    return np.asarray(positions)


def _fixed_reference_targets(
    references: dict,
    config: dict,
    targets_by_robot: dict[int, list[float]],
) -> np.ndarray:
    if (
        not isinstance(references, dict)
        or set(references) != {"base_ids", "uav_ids"}
        or not isinstance(references["base_ids"], list)
        or not isinstance(references["uav_ids"], list)
    ):
        raise InputIntegrityError("fixed-reference target semantics are unknown")
    positions = [
        _point(config["bases"][base_id], f"base target {base_id}")
        for base_id in references["base_ids"]
    ]
    for reference_id in references["uav_ids"]:
        if reference_id not in targets_by_robot:
            raise InputIntegrityError(
                f"target is missing for fixed UAV reference {reference_id}"
            )
        positions.append(
            _point(
                targets_by_robot[reference_id],
                f"fixed UAV reference target {reference_id}",
            )
        )
    return _references(positions, count=2)


def _estimated_references(
    references: dict,
    config: dict,
    estimated_by_robot: dict[int, np.ndarray],
) -> tuple[np.ndarray | None, str | None]:
    positions = [
        _point(config["bases"][base_id], f"base {base_id}")
        for base_id in references["base_ids"]
    ]
    for reference_id in references["uav_ids"]:
        if reference_id not in estimated_by_robot:
            return None, "estimated_uav_reference_unavailable"
        positions.append(estimated_by_robot[reference_id])
    if len(positions) < 2:
        return None, "fewer_than_two_estimated_references"
    return np.asarray(positions), None


def _require_fixed_subset(
    fixed: dict, active: dict, key: tuple[int, int, str, int]
) -> None:
    for family in ("base_ids", "uav_ids"):
        if not set(fixed[family]).issubset(active[family]):
            raise InputIntegrityError(
                f"active references omit fixed topology at key {key!r}"
            )


def _time_bin(seconds: float) -> str:
    if not math.isfinite(seconds) or seconds < 0.0 or seconds > 250.0:
        raise InputIntegrityError("row time is outside frozen time bins")
    index = min(int(seconds // TIME_BIN_SECONDS), 4)
    return TIME_BIN_LABELS[index]


def analyze_geometric_stability(
    comparison_path: Path,
    *,
    expected_comparison_sha256: str,
    expected_parent_manifest_sha256: str,
    output_dir: Path | None = None,
    live_guard: Callable[[], None] | None = None,
) -> dict:
    """Verify paired evidence, aggregate geometry/error/availability, publish optionally."""
    publication = _PublicationTransaction(output_dir, [])
    active_guard = _combined_live_guard(
        live_guard,
        publication.resource_guard if output_dir is not None else None,
    )
    publication.set_live_guard(active_guard)
    try:
        if active_guard is not None:
            active_guard()
        analyzer_path = Path(__file__).resolve()
        analyzer_hash = _sha256(analyzer_path)
        source_commit = _source_commit(analyzer_path)
        source_path = Path(comparison_path)
        comparison_hash = _verify_hash(
            source_path,
            expected_comparison_sha256,
            "comparison",
        )
        comparison = _strict_object(source_path, "comparison")
        if comparison.get("schema") != COMPARISON_SCHEMA_ID:
            raise InputIntegrityError("comparison schema does not match")
        if comparison.get("status") != "completed":
            raise InputIntegrityError("comparison is not completed")
        source = _mapping(comparison.get("source"), "comparison.source")

        parent_dir = _real_directory(
            source.get("paired_bundle_dir"),
            "paired parent bundle",
        )
        parent_path = parent_dir / "manifest.json"
        parent_hash = _verify_hash(
            parent_path,
            expected_parent_manifest_sha256,
            "paired parent manifest",
        )
        if source.get("paired_parent_manifest_sha256") != parent_hash:
            raise InputIntegrityError(
                "comparison parent-manifest hash does not match trust root"
            )

        strict = _load_child(source, "strict")
        restart = _load_child(source, "restart")
        _require_policy(strict, STRICT_PREVIOUS_POLICY, "strict")
        _require_policy(
            restart,
            RESTART_BEFORE_FIRST_FINITE_POLICY,
            "restart",
        )
        strict_input = _mapping(
            strict["manifest"].get("input_data"),
            "strict input_data",
        )
        restart_input = _mapping(
            restart["manifest"].get("input_data"),
            "restart input_data",
        )
        if strict_input != restart_input:
            raise InputIntegrityError(
                "strict and restart trajectory records differ"
            )
        if strict_input.get("sha256") != source.get("input_data_sha256"):
            raise InputIntegrityError(
                "trajectory hash does not match comparison source"
            )
        trajectory_path = strict_input.get("path")
        trajectory_hash = strict_input.get("sha256")
        if not isinstance(trajectory_path, str) or not isinstance(
            trajectory_hash, str
        ):
            raise InputIntegrityError("trajectory record is invalid")
        publication.protected = [
            source_path,
            parent_path,
            strict["dir"],
            restart["dir"],
            Path(trajectory_path),
            analyzer_path,
        ]
    except BaseException:
        publication.abort_preparation()
        raise
    with publication:
        trajectory = load_trajectory(
            Path(trajectory_path),
            expected_sha256=trajectory_hash,
        )
        raw_seeds = strict["summary"].get("settings", {}).get("run_seeds")
        if not isinstance(raw_seeds, list) or any(
            type(seed) is not int for seed in raw_seeds
        ):
            raise InputIntegrityError("strict run seeds are invalid")
        scientific = _ScientificAccumulator(trajectory, raw_seeds)

        strict_state: dict[tuple[int, str, int], tuple[bool, bool]] = {}
        restart_state: dict[tuple[int, str, int], tuple[bool, bool]] = {}
        paired_rows = 0
        pairs = zip_longest(
            comparison_module._verified_rows(strict, live_guard=None),
            comparison_module._verified_rows(restart, live_guard=None),
            fillvalue=_MISSING,
        )
        for strict_row, restart_row in pairs:
            if strict_row is _MISSING or restart_row is _MISSING:
                raise InputIntegrityError(
                    "strict and restart streams have different cardinality"
                )
            strict_key = comparison_module._compact_key(strict_row)
            restart_key = comparison_module._compact_key(restart_row)
            if strict_key != restart_key:
                raise InputIntegrityError(
                    f"paired row key mismatch at {strict_key!r}/{restart_key!r}"
                )
            if not comparison_module._paired_inputs_equal(
                strict_row, restart_row
            ):
                raise InputIntegrityError(
                    f"paired external inputs differ at key {strict_key!r}"
                )
            comparison_module._reconcile_replay_row_inputs(strict_row)
            comparison_module._reconcile_replay_row_inputs(restart_row)
            comparison_module._validate_policy_row(
                strict_row,
                policy=STRICT_PREVIOUS_POLICY,
                acquisition_state=strict_state,
            )
            comparison_module._validate_policy_row(
                restart_row,
                policy=RESTART_BEFORE_FIRST_FINITE_POLICY,
                acquisition_state=restart_state,
            )
            frame_index, _, _, robot_id = strict_key
            try:
                expected_truth = trajectory["truth"][frame_index][robot_id]
            except KeyError as error:
                raise InputIntegrityError(
                    f"row key {strict_key!r} is outside the trajectory"
                ) from error
            if strict_row.get("truth_position") != expected_truth:
                raise InputIntegrityError(
                    f"row truth does not match trajectory at {strict_key!r}"
                )
            scientific.record(restart_row)
            paired_rows += 1
            if (
                active_guard is not None
                and paired_rows % LIVE_CHECK_INTERVAL_ROWS == 0
            ):
                active_guard()

        expected_rows = _expected_paired_rows(strict, restart, trajectory)
        if paired_rows != expected_rows:
            raise InputIntegrityError(
                "paired row cardinality does not match trajectory dimensions"
            )
        scientific_report = scientific.finish()
        report = {
            "schema": SCHEMA_ID,
            "status": "completed",
            "source": {
                "comparison_path": str(source_path.resolve()),
                "comparison_sha256": comparison_hash,
                "parent_manifest_sha256": parent_hash,
                "strict_bundle": str(strict["dir"].resolve()),
                "restart_bundle": str(restart["dir"].resolve()),
                "input_data_path": trajectory["path"],
                "input_data_sha256": trajectory_hash,
                "source_commit": source_commit,
                "analyzer_path": str(analyzer_path),
                "analyzer_sha256": analyzer_hash,
                "analyzer_revision_binding": (
                    "source_commit identifies repository HEAD at analysis "
                    "start; analyzer_sha256 identifies the working-tree "
                    "source bytes frozen after disk preflight"
                ),
            },
            "protocol": {
                "scope": "post-hoc exploratory",
                "truth_trajectory_count": 1,
                "range_noise_seed_count": len(raw_seeds),
                "time_bins_seconds": [0, 50, 100, 150, 200, 250],
                "primary_geometry": "dynamic active-set geometry matrix",
                "statistical_unit": (
                    "range-noise seed nested in one truth trajectory"
                ),
            },
            "integrity": {
                "paired_rows": paired_rows,
                "paired_inputs_equal": True,
                "source_hashes_unchanged": True,
            },
            **scientific_report,
            "claim_boundary": [
                "not a deterministic true-error bound",
                "not an arbitrary-depth stability result",
                "not an unconditional controller-safety result",
                "not a cross-trajectory generality result",
            ],
        }
        _validate_json_tree(report, "geometric-stability report")

        def reverify_sources() -> None:
            comparison_module._verify_bundle_unchanged(strict)
            comparison_module._verify_bundle_unchanged(restart)
            _verify_hash(source_path, comparison_hash, "comparison")
            _verify_hash(parent_path, parent_hash, "paired parent manifest")
            _verify_hash(Path(trajectory["path"]), trajectory_hash, "trajectory")
            if _sha256(analyzer_path) != analyzer_hash:
                raise InputIntegrityError("analyzer source changed during analysis")
            if _source_commit(analyzer_path) != source_commit:
                raise InputIntegrityError("analyzer source commit changed")

        if output_dir is None:
            reverify_sources()
        publication.publish(report, reverify_sources)
        return report


def _combined_live_guard(
    external: Callable[[], None] | None,
    resource: Callable[[], None] | None,
) -> Callable[[], None] | None:
    if external is None:
        return resource
    if resource is None:
        return external

    def guard() -> None:
        resource()
        external()

    return guard


def _validate_json_tree(value: object, description: str) -> None:
    value_type = type(value)
    if value_type is float:
        if not math.isfinite(value):
            raise InputIntegrityError(
                f"{description} contains a non-finite number"
            )
        return
    if value_type in (type(None), bool, int, str):
        return
    if value_type is list:
        for index, item in enumerate(value):
            _validate_json_tree(item, f"{description}[{index}]")
        return
    if value_type is dict:
        for key, item in value.items():
            if type(key) is not str:
                raise InputIntegrityError(
                    f"{description} contains a non-string object key"
                )
            _validate_json_tree(item, f"{description}.{key}")
        return
    raise InputIntegrityError(
        f"{description} contains a non-JSON value of type "
        f"{value_type.__name__}"
    )


def _json_bytes(report: dict) -> bytes:
    _validate_json_tree(report, "geometric-stability report")
    return (
        json.dumps(report, sort_keys=True, indent=2, allow_nan=False) + "\n"
    ).encode("utf-8")


def _markdown_bytes(report: dict) -> bytes:
    source = report["source"]
    geometry = report["geometry"]
    lines = [
        "# Geometric stability audit",
        "",
        "## Scope and sources",
        "",
        "This is a post-hoc exploratory analysis of one truth trajectory and "
        "the registered range-noise seeds.",
        "",
        "```json",
        json.dumps(source, sort_keys=True, indent=2, allow_nan=False),
        "```",
        "",
        "## Theory and empirical separation",
        "",
        "The dynamic active-set geometry matrix is the primary geometry "
        "quantity. Its FIM interpretation is model-internal; the empirical "
        "sections below separately report observed errors and calibration.",
        "",
        "## Geometry and FIM validity",
        "",
        "True active geometry and fixed-pair explanatory geometry are kept "
        "separate from estimated geometry. Estimated-geometry and modeled-FIM "
        "denominators are shown in the report data below.",
        "",
        "```json",
        json.dumps(
            {
                "true_dynamic_all_primary": geometry[
                    "true_dynamic_all_primary"
                ],
                "true_fixed_pair_all_primary": geometry[
                    "true_fixed_pair_all_primary"
                ],
                "estimated_dynamic_finite": geometry[
                    "estimated_dynamic_finite"
                ],
                "estimated_fixed_pair_finite": geometry[
                    "estimated_fixed_pair_finite"
                ],
                "modeled_fim_valid": geometry["modeled_fim_valid"],
            },
            sort_keys=True,
            indent=2,
            allow_nan=False,
        ),
        "```",
        "",
        "## Nominal target perturbation",
        "",
        "Nominal fixed-pair target geometry and its strict three-vertex "
        "tracking-deviation supremum are explanatory. Equality at the "
        "supremum is not certified. The complete dynamic active-set "
        "geometry remains primary.",
        "",
        "```json",
        json.dumps(
            {
                "nominal_fixed_pair_all_primary": geometry[
                    "nominal_fixed_pair_all_primary"
                ],
                "estimated_fixed_pair_tracking_margin_finite": geometry[
                    "estimated_fixed_pair_tracking_margin_finite"
                ],
            },
            sort_keys=True,
            indent=2,
            allow_nan=False,
        ),
        "```",
        "",
        "## Absolute error and availability",
        "",
        "Absolute-error profiles are in metres. Availability includes the "
        "longest unavailable and non-finite retained outages.",
        "",
        "```json",
        json.dumps(
            {
                "absolute_error_metres": report["absolute_error"],
                "availability": report["availability"],
            },
            sort_keys=True,
            indent=2,
            allow_nan=False,
        ),
        "```",
        "",
        "## Calibration diagnostics",
        "",
        "Epsilon containment and q>9 are calibration diagnostics, not "
        "deterministic bounds. Their strata include UAV-reference count, "
        "shared UAV ancestry, and the two-reference opposite-baseline-side "
        "proxy.",
        "",
        "```json",
        json.dumps(report["calibration"], sort_keys=True, indent=2, allow_nan=False),
        "```",
        "",
        "## Claim boundary",
        "",
        *[f"- {claim}" for claim in report["claim_boundary"]],
        "- one truth trajectory; no cross-trajectory generality is asserted.",
        "",
    ]
    return "\n".join(lines).encode("utf-8")


def _absolute_lexical_path(path: Path) -> Path:
    raw_path = os.fspath(path)
    if not os.path.isabs(raw_path):
        raw_path = os.path.join(os.getcwd(), raw_path)
    normalized = os.path.normpath(raw_path)
    if not os.path.isabs(normalized):
        raise AnalysisLimitError("output path must be absolute")
    return Path(normalized)


def _paths_overlap(first: Path, second: Path) -> bool:
    first = first.resolve()
    second = second.resolve()
    return (
        first == second
        or first in second.parents
        or second in first.parents
    )


def _close_fd_no_fail(descriptor: int) -> None:
    """Issue one raw close without exposing Python's patchable close hook."""
    try:
        libc = ctypes.PyDLL(None, use_errno=True)
        if sys.platform == "darwin":
            try:
                close = libc.__close_nocancel
            except AttributeError:
                close = libc.close
        elif sys.platform.startswith("linux"):
            close = libc.close
        else:
            return
        close.argtypes = [ctypes.c_int]
        close.restype = ctypes.c_int
        ctypes.set_errno(0)
        result = close(descriptor)
        if result == 0:
            return
        error_number = ctypes.get_errno()
        if error_number in (errno.EBADF, errno.EINTR):
            return
        # close(2) may already have released the descriptor for every other
        # reported error. Retrying could close a concurrently reused number.
    except BaseException:
        # Descriptor teardown must not replace a precommit failure or escape
        # after the authoritative publication transition.
        return


def _create_and_pin_directory_at(
    parent_fd: int,
    candidate: str,
) -> tuple[int, tuple[int, int]]:
    """Create and pin one unpredictable child without a Python open hook."""
    if sys.platform == "darwin":
        mode_type = ctypes.c_ushort
    elif sys.platform.startswith("linux"):
        mode_type = ctypes.c_uint
    else:
        raise AnalysisLimitError(
            "descriptor-relative staging creation is unsupported"
        )
    try:
        libc = ctypes.PyDLL(None, use_errno=True)
        mkdirat = libc.mkdirat
        openat = libc.openat
    except AttributeError as error:
        raise AnalysisLimitError(
            "descriptor-relative staging creation is unavailable"
        ) from error
    mkdirat.argtypes = [
        ctypes.c_int,
        ctypes.c_char_p,
        mode_type,
    ]
    mkdirat.restype = ctypes.c_int
    openat.argtypes = [
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
    ]
    openat.restype = ctypes.c_int
    candidate_bytes = os.fsencode(candidate)
    ctypes.set_errno(0)
    if mkdirat(parent_fd, candidate_bytes, 0o700) != 0:
        error_number = ctypes.get_errno()
        if error_number == errno.EEXIST:
            raise FileExistsError(
                error_number,
                os.strerror(error_number),
                candidate,
            )
        raise OSError(
            error_number,
            os.strerror(error_number),
            candidate,
        )
    descriptor = openat(
        parent_fd,
        candidate_bytes,
        os.O_RDONLY
        | os.O_DIRECTORY
        | os.O_NOFOLLOW
        | os.O_CLOEXEC,
    )
    if descriptor < 0:
        error_number = ctypes.get_errno()
        raise OSError(
            error_number,
            os.strerror(error_number),
            candidate,
        )
    try:
        metadata = _PIN_FSTAT(descriptor)
        if not stat.S_ISDIR(metadata.st_mode):
            raise AnalysisLimitError(
                "created staging entry is not a directory"
            )
        return descriptor, _directory_identity(metadata)
    except BaseException:
        _close_fd_no_fail(descriptor)
        raise


def _rename_no_replace_at(
    parent_fd: int,
    source_name: str,
    destination_name: str,
) -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    source_bytes = os.fsencode(source_name)
    destination_bytes = os.fsencode(destination_name)
    if sys.platform == "darwin":
        try:
            rename = libc.renameatx_np
        except AttributeError as error:
            raise AnalysisLimitError(
                "descriptor-relative no-replace rename is unavailable"
            ) from error
        rename.argtypes = [
            ctypes.c_int,
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.c_char_p,
            ctypes.c_uint,
        ]
        rename.restype = ctypes.c_int
        result = rename(
            parent_fd,
            source_bytes,
            parent_fd,
            destination_bytes,
            0x00000004,
        )
    elif sys.platform.startswith("linux"):
        try:
            rename = libc.renameat2
        except AttributeError as error:
            raise AnalysisLimitError(
                "atomic no-replace rename is unavailable"
            ) from error
        rename.argtypes = [
            ctypes.c_int,
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.c_char_p,
            ctypes.c_uint,
        ]
        rename.restype = ctypes.c_int
        result = rename(
            parent_fd,
            source_bytes,
            parent_fd,
            destination_bytes,
            1,
        )
    else:
        raise AnalysisLimitError("atomic no-replace rename is unsupported")
    if result == 0:
        return
    error_number = ctypes.get_errno()
    if error_number in (errno.EEXIST, errno.ENOTEMPTY):
        raise AnalysisLimitError("output directory already exists")
    raise OSError(
        error_number,
        os.strerror(error_number),
        destination_name,
    )


def _directory_identity(metadata: os.stat_result) -> tuple[int, int]:
    return metadata.st_dev, metadata.st_ino


def _available_bytes_for_guard(
    parent_fd: int | None,
    fallback_path: Path,
) -> int:
    if parent_fd is None:
        return available_bytes(fallback_path)
    filesystem = os.fstatvfs(parent_fd)
    return filesystem.f_bavail * filesystem.f_frsize


def _allocated_directory_fd(directory_fd: int) -> int:
    total = os.fstat(directory_fd).st_blocks * 512
    for name in os.listdir(directory_fd):
        metadata = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
        total += metadata.st_blocks * 512
        if stat.S_ISDIR(metadata.st_mode):
            child_fd = os.open(
                name,
                os.O_RDONLY
                | os.O_DIRECTORY
                | os.O_NOFOLLOW
                | os.O_CLOEXEC,
                dir_fd=directory_fd,
            )
            try:
                total += _allocated_directory_fd(child_fd) - (
                    metadata.st_blocks * 512
                )
            finally:
                os.close(child_fd)
    return total


def _write_all(file_fd: int, payload: bytes) -> None:
    view = memoryview(payload)
    while view:
        written = os.write(file_fd, view)
        if written <= 0:
            raise OSError("persistent output write made no progress")
        view = view[written:]


def _clear_directory_fd(directory_fd: int) -> None:
    for name in os.listdir(directory_fd):
        metadata = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
        if stat.S_ISDIR(metadata.st_mode):
            child_fd = os.open(
                name,
                os.O_RDONLY
                | os.O_DIRECTORY
                | os.O_NOFOLLOW
                | os.O_CLOEXEC,
                dir_fd=directory_fd,
            )
            try:
                _clear_directory_fd(child_fd)
            finally:
                os.close(child_fd)
            os.rmdir(name, dir_fd=directory_fd)
        else:
            os.unlink(name, dir_fd=directory_fd)


class _PublicationTransaction:
    """Publish through pinned directory descriptors without following links."""

    def __init__(self, output_dir: Path | None, protected: list[Path]) -> None:
        self.output_dir = (
            None
            if output_dir is None
            else _absolute_lexical_path(Path(output_dir))
        )
        self.protected = protected
        self.output_name = (
            None if self.output_dir is None else self.output_dir.name
        )
        self.parent_path = (
            None if self.output_dir is None else self.output_dir.parent
        )
        self.canonical_parent: Path | None = None
        self.preflight_path = (
            None if self.parent_path is None else Path(os.sep)
        )
        self.parent_fd: int | None = None
        self.parent_identity: tuple[int, int] | None = None
        self.parent_container_fd: int | None = None
        self.parent_name: str | None = None
        self.chain_fds: list[int] = []
        self.chain_records: list[
            tuple[int, str, tuple[int, int], int]
        ] = []
        self.preflight_fd: int | None = None
        self.staging_fd: int | None = None
        self.artifact_fds: dict[str, int] = {}
        self.staging_name: str | None = None
        self.staging_identity: tuple[int, int] | None = None
        self.staging_created = False
        self.owns_parent = False
        self.published = False
        self.start_checked = False
        self._live_guard: Callable[[], None] | None = None

    def set_live_guard(self, guard: Callable[[], None] | None) -> None:
        self._live_guard = guard

    def resource_guard(self) -> None:
        if self.output_dir is None or self.preflight_path is None:
            return
        if not self.start_checked:
            self._pin_existing_path_chain()
            free = _available_bytes_for_guard(
                self.preflight_fd,
                self.preflight_path,
            )
            if free < START_BYTES:
                raise AnalysisLimitError(
                    f"available={free} below start threshold={START_BYTES}"
                )
            self.start_checked = True
            return
        probe_fd = (
            self.parent_fd
            if self.parent_fd is not None
            else self.preflight_fd
        )
        free = _available_bytes_for_guard(probe_fd, self.preflight_path)
        if free < HARD_FLOOR_BYTES:
            raise AnalysisLimitError(
                f"available={free} below live threshold={HARD_FLOOR_BYTES}"
            )
        if (
            self.staging_fd is not None
            and _allocated_directory_fd(self.staging_fd) > OUTPUT_CAP_BYTES
        ):
            raise AnalysisLimitError(
                "analysis output exceeds 10 MB allocation cap"
            )

    def _pin_existing_path_chain(self) -> None:
        if self.output_dir is None or self.parent_path is None:
            return
        if self.chain_fds:
            return
        self.canonical_parent = self.parent_path
        parts = self.canonical_parent.parts
        if not parts or parts[0] != os.sep:
            raise AnalysisLimitError("output path must be absolute")
        root_fd = self._open_directory(Path(os.sep))
        self.chain_fds.append(root_fd)
        current_fd = root_fd
        self.preflight_fd = root_fd
        for index, component in enumerate(parts[1:]):
            is_parent = index == len(parts[1:]) - 1
            try:
                child_fd = self._open_directory(
                    Path(component),
                    dir_fd=current_fd,
                )
            except FileNotFoundError:
                if not is_parent:
                    raise AnalysisLimitError(
                        "only the exact output parent may be created"
                    )
                self.parent_container_fd = current_fd
                self.parent_name = component
                break
            except OSError as error:
                try:
                    metadata = os.stat(
                        component,
                        dir_fd=current_fd,
                        follow_symlinks=False,
                    )
                except OSError:
                    raise AnalysisLimitError(
                        "output path component cannot be pinned"
                    ) from error
                if stat.S_ISLNK(metadata.st_mode):
                    raise AnalysisLimitError(
                        "output path contains a symlink component"
                    ) from error
                raise AnalysisLimitError(
                    "output path component is not a directory"
                ) from error
            self.chain_fds.append(child_fd)
            metadata = os.fstat(child_fd)
            identity = _directory_identity(metadata)
            self.chain_records.append(
                (current_fd, component, identity, child_fd)
            )
            current_fd = child_fd
            self.preflight_fd = child_fd
            if is_parent:
                self.parent_fd = child_fd
                self.parent_identity = identity
                self.parent_container_fd = self.chain_records[-1][0]
                self.parent_name = component

    def _verify_path_chain(self) -> None:
        if (
            self.parent_path is None
            or self.canonical_parent is None
            or not self.chain_fds
        ):
            raise AnalysisLimitError("output path chain is not pinned")
        if self.parent_identity is not None:
            self._verify_parent_identity()
        if self.parent_path != self.canonical_parent:
            raise AnalysisLimitError("output path changed before publication")
        for container_fd, name, identity, child_fd in self.chain_records:
            try:
                metadata = os.stat(
                    name,
                    dir_fd=container_fd,
                    follow_symlinks=False,
                )
            except FileNotFoundError as error:
                raise AnalysisLimitError(
                    "output path changed before publication"
                ) from error
            if (
                not stat.S_ISDIR(metadata.st_mode)
                or _directory_identity(metadata) != identity
                or _directory_identity(os.fstat(child_fd)) != identity
            ):
                raise AnalysisLimitError(
                    "output path changed before publication"
                )

    def _guard(self) -> None:
        if self._live_guard is not None:
            self._live_guard()

    @staticmethod
    def _open_directory(path: Path, *, dir_fd: int | None = None) -> int:
        return os.open(
            path if dir_fd is None else path.name,
            os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
            dir_fd=dir_fd,
        )

    def __enter__(self) -> _PublicationTransaction:
        if self.output_dir is None:
            return self
        output = self.output_dir
        try:
            if output.is_symlink():
                raise AnalysisLimitError("output directory must not be a symlink")
            if output.exists():
                raise AnalysisLimitError("output directory already exists")
            if any(_paths_overlap(output, path) for path in self.protected):
                raise AnalysisLimitError(
                    "output directory overlaps a protected input"
                )
            self._verify_path_chain()
            if self.parent_fd is None:
                if self.parent_container_fd is None or not self.parent_name:
                    raise AnalysisLimitError("output parent is not pinned")
                self._guard()
                os.mkdir(
                    self.parent_name,
                    mode=0o700,
                    dir_fd=self.parent_container_fd,
                )
                self.owns_parent = True
                parent_metadata = os.stat(
                    self.parent_name,
                    dir_fd=self.parent_container_fd,
                    follow_symlinks=False,
                )
                self.parent_identity = _directory_identity(parent_metadata)
                self.parent_fd = self._open_directory(
                    Path(self.parent_name),
                    dir_fd=self.parent_container_fd,
                )
                self.chain_fds.append(self.parent_fd)
                if (
                    _directory_identity(os.fstat(self.parent_fd))
                    != self.parent_identity
                ):
                    raise AnalysisLimitError(
                        "output parent changed while it was opened"
                    )
                self.chain_records.append(
                    (
                        self.parent_container_fd,
                        self.parent_name,
                        self.parent_identity,
                        self.parent_fd,
                    )
                )
                self._guard()
            self._verify_path_chain()
            if self._entry_exists(self.output_name):
                raise AnalysisLimitError("output directory already exists")
            self._guard()
            for _ in range(100):
                candidate = (
                    f"{self.output_name}.incomplete-{uuid.uuid4().hex}"
                )
                self.staging_name = candidate
                self.staging_created = True
                try:
                    descriptor, identity = _create_and_pin_directory_at(
                        self.parent_fd,
                        candidate,
                    )
                except FileExistsError:
                    self.staging_name = None
                    self.staging_created = False
                    continue
                self.staging_fd = descriptor
                self.staging_identity = identity
                name_metadata = os.stat(
                    candidate,
                    dir_fd=self.parent_fd,
                    follow_symlinks=False,
                )
                if (
                    not stat.S_ISDIR(name_metadata.st_mode)
                    or _directory_identity(name_metadata)
                    != self.staging_identity
                ):
                    raise AnalysisLimitError(
                        "staging directory changed while it was opened"
                    )
                break
            if self.staging_name is None:
                raise AnalysisLimitError("could not allocate staging directory")
            self._guard()
            return self
        except BaseException:
            self._cleanup_with_retry(suppress_errors=True)
            self._close_descriptors()
            raise

    def _entry_exists(self, name: str | None) -> bool:
        if self.parent_fd is None or name is None:
            return False
        try:
            os.stat(name, dir_fd=self.parent_fd, follow_symlinks=False)
        except FileNotFoundError:
            return False
        return True

    def _verify_parent_identity(self) -> None:
        if (
            self.parent_container_fd is None
            or self.parent_name is None
            or self.parent_identity is None
        ):
            raise AnalysisLimitError("output parent is not pinned")
        try:
            metadata = os.stat(
                self.parent_name,
                dir_fd=self.parent_container_fd,
                follow_symlinks=False,
            )
        except FileNotFoundError as error:
            raise AnalysisLimitError("output parent changed before publication") from error
        if (
            not stat.S_ISDIR(metadata.st_mode)
            or _directory_identity(metadata) != self.parent_identity
        ):
            raise AnalysisLimitError("output parent changed before publication")

    def _verify_staging_identity(self) -> None:
        if (
            self.parent_fd is None
            or self.staging_fd is None
            or self.staging_name is None
            or self.staging_identity is None
        ):
            raise AnalysisLimitError("staging directory is not pinned")
        try:
            metadata = os.stat(
                self.staging_name,
                dir_fd=self.parent_fd,
                follow_symlinks=False,
            )
        except FileNotFoundError as error:
            raise AnalysisLimitError(
                "staging directory changed before publication"
            ) from error
        if (
            not stat.S_ISDIR(metadata.st_mode)
            or _directory_identity(metadata) != self.staging_identity
            or _directory_identity(os.fstat(self.staging_fd))
            != self.staging_identity
        ):
            raise AnalysisLimitError(
                "staging directory changed before publication"
            )

    def _destination_matches_staging(self) -> bool:
        if (
            self.parent_fd is None
            or self.output_name is None
            or self.staging_fd is None
        ):
            return False
        try:
            destination = os.stat(
                self.output_name,
                dir_fd=self.parent_fd,
                follow_symlinks=False,
            )
        except FileNotFoundError:
            return False
        return (
            stat.S_ISDIR(destination.st_mode)
            and _directory_identity(destination)
            == _directory_identity(os.fstat(self.staging_fd))
        )

    def _pin_artifacts(self, expected: dict[str, bytes]) -> None:
        if self.staging_fd is None:
            raise AnalysisLimitError("staging directory is not pinned")
        if set(os.listdir(self.staging_fd)) != set(expected):
            raise AnalysisLimitError(
                "staging output contains unexpected artifacts"
            )
        for name in expected:
            try:
                descriptor = os.open(
                    name,
                    os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC,
                    dir_fd=self.staging_fd,
                )
            except OSError as error:
                raise AnalysisLimitError(
                    f"artifact {name} cannot be opened without following links"
                ) from error
            self.artifact_fds[name] = descriptor
        self._verify_artifact_contents(expected)
        self._verify_artifact_name_identities(expected)

    def _verify_artifact_contents(
        self,
        expected: dict[str, bytes],
    ) -> None:
        if set(self.artifact_fds) != set(expected):
            raise AnalysisLimitError("both output artifacts must be pinned")
        for name, expected_bytes in expected.items():
            descriptor = self.artifact_fds[name]
            metadata = os.fstat(descriptor)
            if not stat.S_ISREG(metadata.st_mode):
                raise AnalysisLimitError(
                    f"artifact {name} is not a regular file"
                )
            if metadata.st_size != len(expected_bytes):
                raise AnalysisLimitError(
                    f"artifact {name} has an unexpected byte length"
                )
            os.lseek(descriptor, 0, os.SEEK_SET)
            chunks: list[bytes] = []
            remaining = len(expected_bytes)
            while remaining:
                chunk = os.read(
                    descriptor,
                    min(remaining, 1024 * 1024),
                )
                if not chunk:
                    break
                chunks.append(chunk)
                remaining -= len(chunk)
            if (
                remaining
                or os.read(descriptor, 1)
                or b"".join(chunks) != expected_bytes
            ):
                raise AnalysisLimitError(
                    f"artifact {name} bytes do not match generated output"
                )

    def _verify_artifact_name_identities(
        self,
        expected: dict[str, bytes],
    ) -> None:
        if self.staging_fd is None:
            raise AnalysisLimitError("staging directory is not pinned")
        if set(os.listdir(self.staging_fd)) != set(expected):
            raise AnalysisLimitError(
                "staging output contains unexpected artifacts"
            )
        for name, expected_bytes in expected.items():
            try:
                name_metadata = os.stat(
                    name,
                    dir_fd=self.staging_fd,
                    follow_symlinks=False,
                )
            except OSError as error:
                raise AnalysisLimitError(
                    f"artifact {name} changed before publication"
                ) from error
            descriptor_metadata = os.fstat(self.artifact_fds[name])
            if (
                not stat.S_ISREG(name_metadata.st_mode)
                or name_metadata.st_size != len(expected_bytes)
                or _directory_identity(name_metadata)
                != _directory_identity(descriptor_metadata)
            ):
                raise AnalysisLimitError(
                    f"artifact {name} changed before publication"
                )

    def publish(self, report: dict, reverify_sources: Callable[[], None]) -> None:
        if (
            self.output_dir is None
            or self.parent_fd is None
            or self.staging_fd is None
            or self.staging_name is None
            or self.output_name is None
        ):
            return
        json_bytes = _json_bytes(report)
        markdown_bytes = _markdown_bytes(report)
        expected_artifacts = {
            OUTPUT_JSON_NAME: json_bytes,
            OUTPUT_MARKDOWN_NAME: markdown_bytes,
        }
        if len(json_bytes) + len(markdown_bytes) > OUTPUT_CAP_BYTES:
            raise AnalysisLimitError("serialized analysis exceeds 10 MB cap")
        for name, payload in (
            (OUTPUT_JSON_NAME, json_bytes),
            (OUTPUT_MARKDOWN_NAME, markdown_bytes),
        ):
            self._guard()
            file_fd = os.open(
                name,
                os.O_WRONLY
                | os.O_CREAT
                | os.O_EXCL
                | os.O_NOFOLLOW
                | os.O_CLOEXEC,
                0o600,
                dir_fd=self.staging_fd,
            )
            try:
                _write_all(file_fd, payload)
                os.fsync(file_fd)
            finally:
                os.close(file_fd)
            self._guard()
        # This is the final caller-visible checkpoint. Everything below is
        # descriptor-relative validation and the commit closure.
        self._guard()
        reverify_sources()
        self._pin_artifacts(expected_artifacts)
        if len(json_bytes) + len(markdown_bytes) > OUTPUT_CAP_BYTES:
            raise AnalysisLimitError("serialized analysis exceeds 10 MB cap")
        if _allocated_directory_fd(self.staging_fd) > OUTPUT_CAP_BYTES:
            raise AnalysisLimitError(
                "analysis output exceeds 10 MB allocation cap"
            )
        self._verify_staging_identity()
        os.fsync(self.staging_fd)
        self._verify_path_chain()
        _rename_no_replace_at(
            self.parent_fd,
            self.staging_name,
            self.output_name,
        )
        if not self._destination_matches_staging():
            raise AnalysisLimitError(
                "publication destination changed at rename boundary"
            )
        self.staging_name = self.output_name
        self._verify_artifact_contents(expected_artifacts)
        self._verify_artifact_name_identities(expected_artifacts)
        # Both registered names still identify the simultaneously pinned,
        # byte-verified regular files. No caller callback occurs between this
        # final closure and the authoritative commit transition.
        self.published = True

    def _cleanup(self) -> None:
        staging_name = self._find_staging_name()
        staging_identity = self.staging_identity
        if (
            self.staging_fd is None
            and staging_name is not None
            and staging_identity is not None
        ):
            try:
                candidate_fd = self._open_directory(
                    Path(staging_name),
                    dir_fd=self.parent_fd,
                )
                candidate_identity = _directory_identity(
                    os.fstat(candidate_fd)
                )
                if candidate_identity != staging_identity:
                    _close_fd_no_fail(candidate_fd)
                    staging_name = None
                else:
                    self.staging_fd = candidate_fd
            except OSError:
                self.staging_fd = None
        if self.staging_fd is not None:
            descriptor_identity = _directory_identity(
                os.fstat(self.staging_fd)
            )
            if (
                staging_identity is not None
                and descriptor_identity == staging_identity
            ):
                _clear_directory_fd(self.staging_fd)
            os.close(self.staging_fd)
            self.staging_fd = None
        if (
            staging_name is not None
            and staging_identity is not None
            and self.parent_fd is not None
        ):
            try:
                current = os.stat(
                    staging_name,
                    dir_fd=self.parent_fd,
                    follow_symlinks=False,
                )
            except FileNotFoundError:
                current = None
            if (
                current is not None
                and stat.S_ISDIR(current.st_mode)
                and _directory_identity(current) == staging_identity
            ):
                os.rmdir(staging_name, dir_fd=self.parent_fd)
        self.staging_name = None
        self.staging_identity = None
        self.staging_created = False
        if self.owns_parent and self._parent_name_still_matches():
            if (
                self.parent_container_fd is not None
                and self.parent_name is not None
            ):
                os.rmdir(self.parent_name, dir_fd=self.parent_container_fd)
        self.owns_parent = False

    def _find_staging_name(self) -> str | None:
        if self.parent_fd is None:
            return None
        identity = self.staging_identity
        if identity is None:
            return None
        if self.staging_fd is not None:
            descriptor_identity = _directory_identity(
                os.fstat(self.staging_fd)
            )
            if identity is not None and descriptor_identity != identity:
                return None
            identity = descriptor_identity
            self.staging_identity = identity
        for name in os.listdir(self.parent_fd):
            try:
                metadata = os.stat(
                    name,
                    dir_fd=self.parent_fd,
                    follow_symlinks=False,
                )
            except FileNotFoundError:
                continue
            if (
                stat.S_ISDIR(metadata.st_mode)
                and _directory_identity(metadata) == identity
            ):
                return name
        return None

    def _parent_name_still_matches(self) -> bool:
        if (
            self.parent_container_fd is None
            or self.parent_name is None
            or self.parent_identity is None
        ):
            return False
        try:
            metadata = os.stat(
                self.parent_name,
                dir_fd=self.parent_container_fd,
                follow_symlinks=False,
            )
        except FileNotFoundError:
            return False
        return (
            stat.S_ISDIR(metadata.st_mode)
            and _directory_identity(metadata) == self.parent_identity
        )

    def _close_descriptors(self) -> None:
        for descriptor in self.artifact_fds.values():
            _close_fd_no_fail(descriptor)
        self.artifact_fds.clear()
        if self.staging_fd is not None:
            _close_fd_no_fail(self.staging_fd)
            self.staging_fd = None
        for descriptor in reversed(self.chain_fds):
            _close_fd_no_fail(descriptor)
        self.chain_fds.clear()
        self.chain_records.clear()
        self.parent_fd = None
        self.parent_container_fd = None
        self.preflight_fd = None

    def abort_preparation(self) -> None:
        self._cleanup_with_retry(suppress_errors=True)
        self._close_descriptors()

    def _cleanup_with_retry(self, *, suppress_errors: bool) -> None:
        last_error: BaseException | None = None
        for _ in range(2):
            try:
                self._cleanup()
                return
            except BaseException as error:
                last_error = error
        if not suppress_errors and last_error is not None:
            raise last_error

    def __exit__(self, exc_type, exc_value, traceback) -> bool:
        if exc_type is not None or not self.published:
            try:
                self._cleanup_with_retry(suppress_errors=exc_type is not None)
            except BaseException:
                self._close_descriptors()
                raise
        self._close_descriptors()
        return False


def _source_commit(analyzer_path: Path) -> str:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=analyzer_path.parents[2],
        check=False,
        capture_output=True,
        text=True,
    )
    commit = result.stdout.strip()
    if result.returncode != 0 or len(commit) != 40:
        raise InputIntegrityError("cannot resolve analyzer source commit")
    return commit


def _verify_hash(path: Path, expected: object, description: str) -> str:
    if (
        not isinstance(expected, str)
        or len(expected) != 64
        or any(character not in "0123456789abcdef" for character in expected)
    ):
        raise InputIntegrityError(
            f"expected {description} hash must be a lowercase SHA-256"
        )
    if _sha256(path) != expected:
        raise InputIntegrityError(
            f"{description} does not match its external trust root"
        )
    return expected


def _strict_object(path: Path, description: str) -> dict:
    if path.is_symlink() or not path.is_file():
        raise InputIntegrityError(f"{description} must be a regular file")
    try:
        value = json.loads(
            path.read_bytes(),
            parse_constant=lambda token: (_ for _ in ()).throw(
                ValueError(f"non-standard JSON constant: {token}")
            ),
        )
    except (OSError, ValueError, UnicodeDecodeError) as error:
        raise InputIntegrityError(f"invalid {description}: {error}") from error
    return _mapping(value, description)


def _real_directory(value: object, description: str) -> Path:
    if not isinstance(value, str) or not value:
        raise InputIntegrityError(f"{description} path is invalid")
    path = Path(value)
    if path.is_symlink() or not path.is_dir():
        raise InputIntegrityError(f"{description} must be a real directory")
    return path


def _load_child(source: dict, label: str) -> dict:
    directory = _real_directory(
        source.get(f"{label}_child_dir"),
        f"{label} child bundle",
    )
    declared_hashes = source.get(f"{label}_child_hashes")
    if not isinstance(declared_hashes, dict):
        raise InputIntegrityError(f"{label} child hashes must be an object")
    child = comparison_module._load_bundle(directory, verify_hashes=True)
    if child["hashes"] != declared_hashes:
        raise InputIntegrityError(
            f"{label} child hashes do not match comparison source"
        )
    return child


def _require_policy(bundle: dict, expected: str, label: str) -> None:
    if bundle["manifest"].get("initialization_policy") != expected:
        raise InputIntegrityError(f"{label} initialization policy does not match")
    settings = bundle["manifest"].get("settings")
    if not isinstance(settings, dict) or settings.get(
        "initialization_policy"
    ) != expected:
        raise InputIntegrityError(
            f"{label} settings initialization policy does not match"
        )


def _expected_paired_rows(strict: dict, restart: dict, trajectory: dict) -> int:
    strict_settings = strict["summary"].get("settings")
    restart_settings = restart["summary"].get("settings")
    if not isinstance(strict_settings, dict) or not isinstance(
        restart_settings, dict
    ):
        raise InputIntegrityError("child settings are invalid")
    dimensions = (
        "run_seeds",
        "graph_cases",
        "effective_frame_count",
    )
    if any(
        strict_settings.get(field) != restart_settings.get(field)
        for field in dimensions
    ):
        raise InputIntegrityError("strict and restart dimensions differ")
    if strict_settings.get("effective_frame_count") != trajectory["frame_count"]:
        raise InputIntegrityError("bundle frame count does not match trajectory")
    seeds = strict_settings.get("run_seeds")
    graph_cases = strict_settings.get("graph_cases")
    if not isinstance(seeds, list) or not isinstance(graph_cases, list):
        raise InputIntegrityError("bundle seed or graph dimensions are invalid")
    expected = (
        trajectory["frame_count"]
        * len(seeds)
        * len(graph_cases)
        * int(trajectory["config"]["num"])
    )
    if strict["summary"].get("expected_process_rows") != expected:
        raise InputIntegrityError(
            "bundle row count does not match trajectory cardinality"
        )
    return expected


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as source:
            for chunk in iter(lambda: source.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as error:
        raise InputIntegrityError(f"cannot hash trajectory: {error}") from error
    return digest.hexdigest()


def _mapping(value: object, name: str) -> dict:
    if not isinstance(value, dict):
        raise InputIntegrityError(f"{name} must be an object")
    return value


def _finite_positive(value: object, name: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(float(value)) or value <= 0:
        raise InputIntegrityError(f"{name} must be finite and positive")
    return float(value)


def main(argv: list[str] | None = None) -> int:
    """Run the registered geometric-stability publication command."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--comparison", required=True, type=Path)
    parser.add_argument("--expected-comparison-sha256", required=True)
    parser.add_argument("--expected-parent-manifest-sha256", required=True)
    parser.add_argument("--output-dir", required=True, type=Path)
    arguments = parser.parse_args(argv)
    try:
        analyze_geometric_stability(
            arguments.comparison,
            expected_comparison_sha256=(
                arguments.expected_comparison_sha256
            ),
            expected_parent_manifest_sha256=(
                arguments.expected_parent_manifest_sha256
            ),
            output_dir=arguments.output_dir,
        )
    except (
        AnalysisLimitError,
        InputIntegrityError,
        OSError,
        ValueError,
    ) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
