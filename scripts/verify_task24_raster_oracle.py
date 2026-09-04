#!/usr/bin/env python3
"""Independent scalar verifier for Task 24 raster-oracle artifacts.

This intentionally does not call the C++ implementation.  It reconstructs
the frozen formulas, the formal initial certified mask, pass/corridor
assignment, and every recorded service witness with scalar trigonometry.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


RESERVE = 0.05 + 5.0 * math.sqrt(2.0)


def passes(height: float, spacing: float) -> list[float]:
    out: list[float] = []
    p = 5.0 + 0.5 * spacing
    while p <= 5.0 + (height - 10.0) + 0.5 * spacing + 1e-9:
        out.append(min(p, height - 5.0))
        p += spacing
    return list(dict.fromkeys(out))


def in_sector(pose: tuple[float, float], yaw: float,
              cell: tuple[float, float]) -> bool:
    dx, dy = cell[0] - pose[0], cell[1] - pose[1]
    distance = math.hypot(dx, dy)
    if distance <= RESERVE or distance > 400.0 - RESERVE:
        return False
    bearing = math.atan2(dy, dx)
    error = abs(math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw)))
    return error + math.asin(RESERVE / distance) <= math.pi / 3.0 + 1e-11


def rotate60(v: tuple[float, float], sign: float) -> tuple[float, float]:
    c, s = 0.5, sign * math.sqrt(3.0) / 2.0
    return c * v[0] - s * v[1], s * v[0] + c * v[1]


def lift_member(mode: str, member: int, front: tuple[float, float]) -> tuple[float, float]:
    if mode == "h0-dual-ladder":
        roles_a = [(0.25, 0.0), (0.25, -0.25), (0.50, 0.0),
                   (0.50, -0.25), (0.75, 0.0), (0.75, -0.25), (1.0, 0.0)]
        roles_b = [(0.25, 0.0), (0.25, 0.25), (0.50, 0.0),
                   (0.50, 0.25), (0.75, 0.0), (0.75, 0.25), (1.0, 0.0)]
        role = roles_a[member - 1] if member <= 7 else roles_b[member - 8]
        base = (1500.0, -50.0)
        v = (front[0] - base[0], front[1] - base[1])
        tri = rotate60(v, -1.0 if role[1] < 0.0 else 1.0)
        return (base[0] + role[0] * v[0] + abs(role[1]) * tri[0],
                base[1] + role[0] * v[1] + abs(role[1]) * tri[1])
    if mode == "pinball-5-4-3-2":
        member0 = member - 1
        row_counts = (5, 4, 3, 2)
        offset = 0
        for row, count in enumerate(row_counts, start=1):
            if member0 < offset + count:
                slot = member0 - offset
                alpha = row / 4.0
                return ((slot + row / 2.0) * 900.0 + alpha * (front[0] - 2250.0),
                        -50.0 + alpha * (front[1] + 50.0))
            offset += count
        raise AssertionError(member)
    center = (1000.0, -50.0)
    vx, vy = front[0] - center[0], front[1] - center[1]
    length = math.hypot(vx, vy)
    h = length / 13.0
    tx, ty = vx / length, vy / length
    nx, ny = -ty, tx
    d = math.sqrt(3.0) * h / 2.0
    if member == 1:
        axial, sign = 1.0, -1.0
    elif member == 2:
        axial, sign = 1.0, 1.0
    else:
        axial, sign = float(member - 1), 1.0 if member % 2 == 0 else -1.0
    return (center[0] + axial * h * tx + sign * d * nx,
            center[1] + axial * h * ty + sign * d * ny)


def mode_geometry(mode: str) -> tuple[int, int, list[tuple[float, float]], float]:
    if mode == "h0-dual-ladder":
        launch = [(1380, 10), (1370, 30), (1360, 50), (1350, 70),
                  (1340, 90), (1330, 110), (1320, 130), (1620, 10),
                  (1630, 30), (1640, 50), (1650, 70), (1660, 90),
                  (1670, 110), (1680, 130)]
        return 300, 300, launch, math.pi / 2.0
    nx, ny = (450, 200) if mode == "pinball-5-4-3-2" else (200, 450)
    spacing = 650.0 if mode == "pinball-5-4-3-2" else 560.0
    first = passes(ny * 10.0, spacing)[0]
    front = (nx * 5.0, first)
    return nx, ny, [lift_member(mode, member, front) for member in range(1, 15)], 0.0


def initial_mask(mode: str) -> tuple[int, int, list[bool]]:
    nx, ny, poses, yaw = mode_geometry(mode)
    mask = []
    for x in range(nx):
        for y in range(ny):
            cell = (5.0 + 10.0 * x, 5.0 + 10.0 * y)
            mask.append(any(in_sector(pose, yaw, cell) for pose in poses))
    return nx, ny, mask


def corridor_cut(mask: list[bool], nx: int, ny: int) -> float:
    weights = [sum(not mask[x * ny + y] for y in range(ny)) for x in range(nx)]
    desired = sum(weights) / 2.0
    cumulative = 0
    index = 0
    while index + 1 < nx and cumulative + weights[index] < desired:
        cumulative += weights[index]
        index += 1
    return 10.0 * (index + 1)


def verify(directory: Path) -> dict:
    summary = json.loads((directory / "summary.json").read_text())
    mode = summary["mode"]
    nx, ny, initial = initial_mask(mode)
    spacing = float(summary["adopted_spacing_m"])
    pass_values = passes(ny * 10.0, spacing)
    cut = corridor_cut(initial, nx, ny) if mode == "h0-dual-ladder" else None
    witnessed: dict[str, dict] = {}
    errors: list[str] = []
    with (directory / "witnesses.jsonl").open() as stream:
        for line_number, line in enumerate(stream, start=1):
            witness = json.loads(line)
            cell_id = witness["cell_id"]
            if cell_id in witnessed:
                errors.append(f"duplicate witness {cell_id}")
                continue
            witnessed[cell_id] = witness
            x, y = map(int, cell_id.split(":"))
            index = x * ny + y
            cell = (5.0 + 10.0 * x, 5.0 + 10.0 * y)
            if initial[index]:
                errors.append(f"witness duplicates initial mask {cell_id}")
            expected_band = min(range(len(pass_values)),
                                key=lambda i: (abs(cell[1] - pass_values[i]), i))
            if witness["band"] != expected_band:
                errors.append(f"band mismatch {cell_id}")
            if cut is not None:
                expected_unit = "A" if cell[0] < cut - 1e-9 else "B"
                if witness["coverage_unit"] != expected_unit:
                    errors.append(f"unit mismatch {cell_id}")
            front = tuple(map(float, witness["front"]))
            pose = lift_member(mode, int(witness["responsible_member"]), front)
            recorded_pose = tuple(map(float, witness["service_pose"]))
            if math.hypot(pose[0] - recorded_pose[0], pose[1] - recorded_pose[1]) > 1e-7:
                errors.append(f"lift mismatch {cell_id}")
            if not in_sector(pose, float(witness["yaw_rad"]), cell):
                errors.append(f"service mismatch {cell_id}")
            if len(errors) >= 50:
                break
    final_mask = list((directory / "final-mask.bin").read_bytes())
    if len(final_mask) != nx * ny or any(value != 1 for value in final_mask):
        errors.append("final mask is not exactly all 90000 cells")
    initial_count = sum(initial)
    if initial_count != summary["initial_certified_count"]:
        errors.append(f"initial count {initial_count} != summary")
    if len(witnessed) != summary["route_service_count"]:
        errors.append(f"witness count {len(witnessed)} != summary")
    union_count = initial_count + len(witnessed)
    if union_count != nx * ny:
        errors.append(f"independent union {union_count} != {nx * ny}")
    return {
        "protocol": "task24-raster-oracle-independent-python-v1",
        "mode": mode,
        "valid": not errors,
        "grid_cells": nx * ny,
        "initial_certified_count": initial_count,
        "route_witness_count": len(witnessed),
        "union_count": union_count,
        "witness_sha256": hashlib.sha256(
            (directory / "witnesses.jsonl").read_bytes()).hexdigest(),
        "final_mask_sha256": hashlib.sha256(
            (directory / "final-mask.bin").read_bytes()).hexdigest(),
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
