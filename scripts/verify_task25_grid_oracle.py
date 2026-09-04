#!/usr/bin/env python3
"""Independent scalar verifier for Task25 DAG/lifting Grid oracle output."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


RESERVE = 0.05 + 5.0 * math.sqrt(2.0)


def in_sector(pose: tuple[float, float], yaw: float,
              cell: tuple[float, float]) -> bool:
    dx, dy = cell[0] - pose[0], cell[1] - pose[1]
    distance = math.hypot(dx, dy)
    if distance <= RESERVE or distance > 400.0 - RESERVE:
        return False
    error = abs(math.atan2(math.sin(math.atan2(dy, dx) - yaw),
                           math.cos(math.atan2(dy, dx) - yaw)))
    return error + math.asin(RESERVE / distance) <= math.pi / 3.0 + 1e-10


def initial_mask(nx: int, ny: int) -> list[bool]:
    center = nx * 5.0
    poses = [(center - 120.0 - 10.0 * i, 10.0 + 20.0 * i)
             for i in range(7)]
    poses += [(center + 120.0 + 10.0 * i, 10.0 + 20.0 * i)
              for i in range(7)]
    return [any(in_sector(pose, math.pi / 2.0,
                          (5.0 + 10.0 * x, 5.0 + 10.0 * y))
                for pose in poses)
            for x in range(nx) for y in range(ny)]


def rotate60(v: tuple[float, float], sign: float) -> tuple[float, float]:
    c, s = 0.5, sign * math.sqrt(3.0) / 2.0
    return c * v[0] - s * v[1], s * v[0] + c * v[1]


def lift(summary: dict, fronts: dict[str, list[float]]) -> dict[int, tuple[float, float]]:
    width = float(summary["width_m"])
    fixed = {100: (0.4 * width, -50.0), 101: (0.5 * width, -50.0),
             102: (0.6 * width, -50.0)}
    units = {unit["id"]: unit for unit in summary["coverage_units"]}
    out: dict[int, tuple[float, float]] = {}
    for member_text, role in summary["member_roles"].items():
        member = int(member_text)
        unit = units[role["coverage_unit"]]
        anchors = [fixed[int(anchor)] for anchor in unit["base_anchors"]]
        base = (sum(p[0] for p in anchors) / len(anchors),
                sum(p[1] for p in anchors) / len(anchors))
        front = tuple(map(float, fronts[unit["id"]]))
        v = (front[0] - base[0], front[1] - base[1])
        triangular = float(role["triangular_fraction"])
        tri = rotate60(v, -1.0 if triangular < 0.0 else 1.0)
        axial = float(role["axial_fraction"])
        out[member] = (base[0] + axial * v[0] + abs(triangular) * tri[0],
                       base[1] + axial * v[1] + abs(triangular) * tri[1])
    return out


def geometry(summary: dict, targets: dict[int, tuple[float, float]]) -> tuple[float, float, float]:
    width = float(summary["width_m"])
    fixed = {100: (0.4 * width, -50.0), 101: (0.5 * width, -50.0),
             102: (0.6 * width, -50.0)}
    all_points = list(fixed.items()) + list(targets.items())
    minimum_sep = min(math.dist(a[1], b[1])
                      for i, a in enumerate(all_points)
                      for b in all_points[i + 1:])
    maximum_ref = 0.0
    bearings: dict[int, list[tuple[float, float]]] = {}
    for edge in summary["reference_edges"]:
        owner, reference = int(edge["owner"]), int(edge["reference"])
        p, q = targets[owner], targets.get(reference, fixed.get(reference))
        assert q is not None
        dx, dy = q[0] - p[0], q[1] - p[1]
        distance = math.hypot(dx, dy)
        maximum_ref = max(maximum_ref, distance)
        bearings.setdefault(owner, []).append((dx / distance, dy / distance))
    minimum_fim = math.inf
    for owner in range(1, 15):
        xx = sum(v[0] * v[0] for v in bearings[owner])
        xy = sum(v[0] * v[1] for v in bearings[owner])
        yy = sum(v[1] * v[1] for v in bearings[owner])
        eigmin = 0.5 * ((xx + yy) - math.hypot(xx - yy, 2.0 * xy))
        minimum_fim = min(minimum_fim, eigmin)
    return maximum_ref, minimum_sep, minimum_fim


def close(a: float, b: float, tolerance: float = 1e-7) -> bool:
    return abs(a - b) <= tolerance * max(1.0, abs(a), abs(b))


def verify(directory: Path) -> dict:
    summary = json.loads((directory / "summary.json").read_text())
    nx, ny = round(summary["width_m"] / 10.0), round(summary["height_m"] / 10.0)
    initial = initial_mask(nx, ny)
    errors: list[str] = []
    serviceable = compatible = virtual = rows = 0
    with (directory / "witnesses.jsonl").open() as stream:
        for line in stream:
            row = json.loads(line)
            rows += 1
            x, y = map(int, row["cell_id"].split(":"))
            index = x * ny + y
            if bool(row["initial_certified"]) != initial[index]:
                errors.append(f"initial mask mismatch {row['cell_id']}")
            if initial[index]:
                if row["serviceable"] is not None:
                    errors.append(f"initial cell has service claim {row['cell_id']}")
                continue
            if not row["serviceable"]:
                continue
            serviceable += 1
            compatible += int(bool(row["nominal_reference_compatible"]))
            virtual += int(not bool(row["nominal_reference_compatible"]))
            targets = lift(summary, row["fronts"])
            member = int(row["responsible_member"])
            pose = tuple(map(float, row["service_pose"]))
            if math.dist(targets[member], pose) > 1e-7:
                errors.append(f"lifting mismatch {row['cell_id']}")
            cell = (5.0 + 10.0 * x, 5.0 + 10.0 * y)
            if not in_sector(pose, float(row["service_yaw_rad"]), cell):
                errors.append(f"service mismatch {row['cell_id']}")
            ref, sep, fim = geometry(summary, targets)
            if not close(ref, float(row["maximum_reference_edge_m"])):
                errors.append(f"reference mismatch {row['cell_id']}")
            if not close(sep, float(row["minimum_target_separation_m"])):
                errors.append(f"separation mismatch {row['cell_id']}")
            if not close(fim, float(row["minimum_nominal_fim_proxy"]), 1e-6):
                errors.append(f"fim mismatch {row['cell_id']}")
            if (ref < 850.0) != bool(row["nominal_reference_compatible"]):
                errors.append(f"compatibility mismatch {row['cell_id']}")
            if len(errors) >= 50:
                break
    initial_count = sum(initial)
    union = initial_count + serviceable
    checks = {
        "rows": rows == nx * ny,
        "initial_count": initial_count == summary["initial_certified_count"],
        "serviceable_count": serviceable == summary["serviceable_task_cells"],
        "compatible_count": compatible == summary["nominal_reference_compatible_task_cells"],
        "virtual_count": virtual == summary["safety_layer_required_task_cells"],
        "union_count": union == summary["joint_gate_count"] == nx * ny,
    }
    errors.extend(name for name, valid in checks.items() if not valid)
    path = directory / "witnesses.jsonl"
    return {
        "protocol": "task25-grid-oracle-independent-scalar-v1",
        "mode": summary["mode"], "valid": not errors,
        "grid_cells": nx * ny, "initial_certified_count": initial_count,
        "serviceable_task_cells": serviceable, "joint_gate_count": union,
        "nominal_reference_compatible_task_cells": compatible,
        "virtual_dependent_task_cells": virtual, "checks": checks,
        "witness_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "errors": errors,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("oracle_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = verify(args.oracle_dir)
    encoded = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(encoded)
    print(encoded, end="")
    return 0 if result["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
