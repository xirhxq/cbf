#!/usr/bin/env python3
"""Validate whether a Swarm data.json run is behaviorally rational."""

import argparse
import json
import math
import sys
from pathlib import Path
try:
    from typing import Any, Iterable
except ImportError:  # pragma: no cover - typing is present in supported Python versions
    Any = object
    Iterable = object


class RationalityOptions:
    def __init__(
        self,
        boundary_tolerance=1.0e-6,
        communication_tolerance=1.0e-6,
        communication_tolerance_ratio=0.0,
        movement_tolerance=1.0e-5,
        yaw_tolerance=1.0e-5,
        battery_min=3700.0,
        battery_max=4200.0,
        battery_tolerance=1.0e-6,
        max_slack=1.0e6,
        allowed_solver_statuses=("optimal", "optimal_inaccurate"),
    ):
        self.boundary_tolerance = boundary_tolerance
        self.communication_tolerance = communication_tolerance
        self.communication_tolerance_ratio = communication_tolerance_ratio
        self.movement_tolerance = movement_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.battery_min = battery_min
        self.battery_max = battery_max
        self.battery_tolerance = battery_tolerance
        self.max_slack = max_slack
        self.allowed_solver_statuses = allowed_solver_statuses


class RationalityResult:
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.metrics = {}

    @property
    def passed(self):
        return not self.errors

    @property
    def error_count(self):
        return len(self.errors)

    def add_error(self, message):
        self.errors.append(message)

    def add_warning(self, message):
        self.warnings.append(message)

    def format_report(self):
        status = "PASS" if self.passed else "FAIL"
        lines = [f"Rationality check: {status}"]
        if self.metrics:
            lines.append("Metrics:")
            for name in sorted(self.metrics):
                lines.append(f"  {name}: {self.metrics[name]:.6g}")
        if self.errors:
            lines.append("Errors:")
            for error in self.errors:
                lines.append(f"  - {error}")
        if self.warnings:
            lines.append("Warnings:")
            for warning in self.warnings:
                lines.append(f"  - {warning}")
        return "\n".join(lines)


def iter_numbers(value: Any, path: str = "$") -> Iterable[tuple[str, float]]:
    if isinstance(value, bool):
        return
    if isinstance(value, (int, float)):
        yield path, float(value)
        return
    if isinstance(value, dict):
        for key, child in value.items():
            yield from iter_numbers(child, f"{path}.{key}")
        return
    if isinstance(value, list):
        for index, child in enumerate(value):
            yield from iter_numbers(child, f"{path}[{index}]")


def point_in_polygon(x: float, y: float, polygon: list[list[float]], tolerance: float) -> bool:
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]

        dx = x2 - x1
        dy = y2 - y1
        cross = dx * (y - y1) - dy * (x - x1)
        if abs(cross) <= tolerance:
            dot = (x - x1) * dx + (y - y1) * dy
            length_sq = dx * dx + dy * dy
            if -tolerance <= dot <= length_sq + tolerance:
                return True

        intersects = (y1 > y) != (y2 > y)
        if intersects:
            x_at_y = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x <= x_at_y + tolerance:
                inside = not inside
    return inside


def angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(b - a), math.cos(b - a))


def robot_map(frame: dict[str, Any]) -> dict[int, dict[str, Any]]:
    return {int(robot["id"]): robot for robot in frame.get("robots", [])}


def formation_map(frame: dict[str, Any]) -> dict[int, dict[str, Any]]:
    return {int(item.get("id", index + 1)): item for index, item in enumerate(frame.get("formation", []))}


def get_xy(robot: dict[str, Any]) -> tuple[float, float]:
    state = robot["state"]
    return float(state["x"]), float(state["y"])


def get_uncertainty(robot: dict[str, Any]) -> float:
    return float(robot.get("uncertainty", 0.0))


def norm2(x: float, y: float) -> float:
    return math.sqrt(x * x + y * y)


def validate_data(data: dict[str, Any], options: RationalityOptions | None = None) -> RationalityResult:
    options = options or RationalityOptions()
    result = RationalityResult()

    for path, value in iter_numbers(data):
        if not math.isfinite(value):
            result.add_error(f"non-finite numeric value at {path}: {value}")

    config = data.get("config", {})
    frames = data.get("state", [])
    if not frames:
        result.add_error("run has no state frames")
        return result

    boundary = config.get("world", {}).get("boundary", [])
    if len(boundary) < 3:
        result.add_error("world boundary has fewer than three points")
    comm_config = config.get("cbfs", {}).get("without-slack", {}).get("comm-fixed", {})
    safety_config = config.get("cbfs", {}).get("without-slack", {}).get("safety", {})
    time_step = float(config.get("execute", {}).get("time-step", 0.0))
    model_name = config.get("model", "")
    bases = config.get("bases", [])

    max_speed = 0.0
    max_yaw_rate = 0.0
    max_slack = 0.0
    max_comm_excess = 0.0
    min_pair_distance = math.inf
    solver_failures = 0
    seen_updates: set[tuple[int, int]] = set()

    for frame_index, frame in enumerate(frames):
        robots = robot_map(frame)
        formations = formation_map(frame)

        for robot_id, robot in robots.items():
            x, y = get_xy(robot)
            if boundary and not point_in_polygon(x, y, boundary, options.boundary_tolerance):
                result.add_error(
                    f"frame {frame_index} robot {robot_id} outside world boundary at ({x:.6g}, {y:.6g})"
                )

            battery = float(robot.get("state", {}).get("battery", options.battery_min))
            if battery < options.battery_min - options.battery_tolerance or battery > options.battery_max + options.battery_tolerance:
                result.add_error(
                    f"frame {frame_index} robot {robot_id} battery {battery:.6g} outside "
                    f"[{options.battery_min:.6g}, {options.battery_max:.6g}]"
                )

            opt = robot.get("opt", {})
            if opt:
                status = opt.get("status")
                solver_info = opt.get("solver_info", {})
                solver_status = solver_info.get("status")
                if status is not None and status != "success":
                    solver_failures += 1
                    result.add_error(f"frame {frame_index} robot {robot_id} optimization status is {status}")
                if solver_status is not None and solver_status not in options.allowed_solver_statuses:
                    solver_failures += 1
                    result.add_error(f"frame {frame_index} robot {robot_id} solver status is {solver_status}")

                for slack in opt.get("slacks", []):
                    slack_value = float(slack)
                    max_slack = max(max_slack, abs(slack_value))
                    if abs(slack_value) > options.max_slack:
                        result.add_error(
                            f"frame {frame_index} robot {robot_id} slack {slack_value:.6g} exceeds "
                            f"{options.max_slack:.6g}"
                        )

        if comm_config.get("on", False):
            max_range = float(comm_config.get("max-range", math.inf))
            tolerance = max(options.communication_tolerance, abs(max_range) * options.communication_tolerance_ratio)
            for robot_id, formation in formations.items():
                if robot_id not in robots:
                    continue
                x1, y1 = get_xy(robots[robot_id])
                uncertainty1 = get_uncertainty(robots[robot_id])
                for anchor_id in formation.get("anchorIds", []):
                    other_id = int(anchor_id)
                    if other_id not in robots:
                        result.add_warning(f"frame {frame_index} robot {robot_id} anchor {other_id} missing from robot list")
                        continue
                    x2, y2 = get_xy(robots[other_id])
                    total = norm2(x1 - x2, y1 - y2) + uncertainty1 + get_uncertainty(robots[other_id])
                    excess = total - max_range
                    max_comm_excess = max(max_comm_excess, excess)
                    if excess > tolerance:
                        result.add_error(
                            f"frame {frame_index} robot {robot_id}->{other_id} communication constraint "
                            f"violated: distance+uncertainty={total:.6g}, max={max_range:.6g}, tolerance={tolerance:.6g}"
                        )
                for base_id in formation.get("baseIds", []):
                    base_index = int(base_id)
                    if base_index < 0 or base_index >= len(bases):
                        result.add_warning(f"frame {frame_index} robot {robot_id} base {base_index} missing from config")
                        continue
                    bx, by = bases[base_index]
                    total = norm2(x1 - float(bx), y1 - float(by)) + uncertainty1
                    excess = total - max_range
                    max_comm_excess = max(max_comm_excess, excess)
                    if excess > tolerance:
                        result.add_error(
                            f"frame {frame_index} robot {robot_id}->base {base_index} communication constraint "
                            f"violated: distance+uncertainty={total:.6g}, max={max_range:.6g}, tolerance={tolerance:.6g}"
                        )

        if safety_config.get("on", False):
            safe_distance = float(safety_config.get("safe-distance", 0.0))
            consider_uncertainty = bool(safety_config.get("consider-uncertainty", True))
            ids = sorted(robots)
            for i, robot_id in enumerate(ids):
                x1, y1 = get_xy(robots[robot_id])
                for other_id in ids[i + 1 :]:
                    x2, y2 = get_xy(robots[other_id])
                    distance = norm2(x1 - x2, y1 - y2)
                    min_pair_distance = min(min_pair_distance, distance)
                    required = safe_distance
                    if consider_uncertainty:
                        required += get_uncertainty(robots[robot_id]) + get_uncertainty(robots[other_id])
                    if distance + options.communication_tolerance < required:
                        result.add_error(
                            f"frame {frame_index} robots {robot_id}-{other_id} safety distance violated: "
                            f"distance={distance:.6g}, required={required:.6g}"
                        )

        for update in frame.get("update", []):
            if isinstance(update, list) and len(update) == 2:
                seen_updates.add((int(update[0]), int(update[1])))

    for frame_index in range(len(frames) - 1):
        current = frames[frame_index]
        next_frame = frames[frame_index + 1]
        dt = float(next_frame.get("runtime", 0.0)) - float(current.get("runtime", 0.0))
        if dt <= 0:
            result.add_error(f"frame {frame_index}->{frame_index + 1} runtime is not strictly increasing")
            continue
        expected_dt = time_step or dt
        if abs(dt - expected_dt) > max(options.movement_tolerance, expected_dt * 1.0e-6):
            result.add_warning(
                f"frame {frame_index}->{frame_index + 1} dt {dt:.6g} differs from configured time-step {expected_dt:.6g}"
            )

        current_robots = robot_map(current)
        next_robots = robot_map(next_frame)
        for robot_id, current_robot in current_robots.items():
            if robot_id not in next_robots:
                result.add_warning(f"frame {frame_index}->{frame_index + 1} robot {robot_id} missing in next frame")
                continue
            opt = current_robot.get("opt", {})
            control = opt.get("result", {})
            x1, y1 = get_xy(current_robot)
            x2, y2 = get_xy(next_robots[robot_id])
            displacement = norm2(x2 - x1, y2 - y1)
            if model_name == "SingleIntegrate2D" and control:
                speed = norm2(float(control.get("vx", 0.0)), float(control.get("vy", 0.0)))
                max_speed = max(max_speed, speed)
                allowed_displacement = speed * dt + options.movement_tolerance
                if displacement > allowed_displacement:
                    result.add_error(
                        f"frame {frame_index}->{frame_index + 1} robot {robot_id} displacement {displacement:.6g} "
                        f"exceeds control-implied limit {allowed_displacement:.6g}"
                    )

                yaw1 = float(current_robot.get("state", {}).get("yawRad", 0.0))
                yaw2 = float(next_robots[robot_id].get("state", {}).get("yawRad", yaw1))
                yaw_rate = abs(float(control.get("yawRateRad", 0.0)))
                max_yaw_rate = max(max_yaw_rate, yaw_rate)
                allowed_yaw_delta = yaw_rate * dt + options.yaw_tolerance
                actual_yaw_delta = abs(angle_delta(yaw1, yaw2))
                if actual_yaw_delta > allowed_yaw_delta:
                    result.add_error(
                        f"frame {frame_index}->{frame_index + 1} robot {robot_id} yaw change {actual_yaw_delta:.6g} "
                        f"exceeds control-implied limit {allowed_yaw_delta:.6g}"
                    )

    valid_grid = data.get("para", {}).get("gridWorld", {}).get("valid")
    valid_count = 0
    if isinstance(valid_grid, list):
        valid_count = sum(1 for row in valid_grid for value in row if value)
    final_coverage = (len(seen_updates) / valid_count) if valid_count else 0.0
    if final_coverage < -options.movement_tolerance or final_coverage > 1.0 + options.movement_tolerance:
        result.add_error(f"derived coverage {final_coverage:.6g} outside [0, 1]")

    result.metrics.update(
        {
            "frames": float(len(frames)),
            "robots": float(max((len(frame.get("robots", [])) for frame in frames), default=0)),
            "solver_failures": float(solver_failures),
            "max_speed": float(max_speed),
            "max_yaw_rate": float(max_yaw_rate),
            "max_slack": float(max_slack),
            "max_comm_excess": float(max_comm_excess),
            "min_pair_distance": 0.0 if math.isinf(min_pair_distance) else float(min_pair_distance),
            "final_coverage": float(final_coverage),
        }
    )

    return result


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def parse_args(argv):
    defaults = RationalityOptions()
    parser = argparse.ArgumentParser(description="Validate rational behavior in a CBF Swarm data.json file")
    parser.add_argument("data_json", type=Path, help="Path to data.json produced by Swarm")
    parser.add_argument("--comm-tolerance", type=float, default=defaults.communication_tolerance)
    parser.add_argument("--comm-tolerance-ratio", type=float, default=defaults.communication_tolerance_ratio)
    parser.add_argument("--movement-tolerance", type=float, default=defaults.movement_tolerance)
    parser.add_argument("--yaw-tolerance", type=float, default=defaults.yaw_tolerance)
    parser.add_argument("--max-slack", type=float, default=defaults.max_slack)
    parser.add_argument("--json", action="store_true", help="Print machine-readable summary")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    options = RationalityOptions(
        communication_tolerance=args.comm_tolerance,
        communication_tolerance_ratio=args.comm_tolerance_ratio,
        movement_tolerance=args.movement_tolerance,
        yaw_tolerance=args.yaw_tolerance,
        max_slack=args.max_slack,
    )
    result = validate_data(load_json(args.data_json), options)
    if args.json:
        print(json.dumps({
            "passed": result.passed,
            "errors": result.errors,
            "warnings": result.warnings,
            "metrics": result.metrics,
        }, indent=2, sort_keys=True))
    else:
        print(result.format_report())
    return 0 if result.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
