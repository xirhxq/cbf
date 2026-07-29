"""Geometry primitives for the immutable geometric-stability diagnostic."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np

from scripts.diagnostics.compare_warm_start_recovery import InputIntegrityError


SCHEMA_ID = "cbf2026-geometric-stability-v1"
TIME_BIN_SECONDS = 50.0
OUTPUT_JSON_NAME = "geometric-stability.json"
OUTPUT_MARKDOWN_NAME = "geometric-stability.md"


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
    ranges = np.linalg.norm(differences, axis=1)
    if np.any(ranges <= 0.0):
        raise InputIntegrityError("geometry contains a coincident or non-finite reference")
    return differences, differences / ranges[:, None]


def geometry_metrics(observer: object, references: object) -> dict:
    """Return count, eigenvalues, normalized minimum, and finite/SPD flags."""
    center = _point(observer, "observer")
    points = _references(references)
    _, directions = _directions(center, points)
    eigenvalues = np.linalg.eigvalsh(directions.T @ directions)
    minimum, maximum = map(float, eigenvalues)
    return {
        "reference_count": int(points.shape[0]),
        "lambda_min": minimum,
        "lambda_max": maximum,
        "normalized_lambda_min": minimum / maximum,
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
    return {
        "included_angle_rad": included_angle,
        "noncollinearity_angle_rad": min(included_angle, np.pi - included_angle),
        "absolute_cosine": abs(cosine),
        "twice_triangle_area": abs(float(np.linalg.det(differences))),
    }


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
