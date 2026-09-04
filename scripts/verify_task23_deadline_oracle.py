#!/usr/bin/env python3
"""Independent scalar verification of Task 23 deadline-oracle assets.

This intentionally reimplements lifting, sensing, reference/separation and
2-D FIM arithmetic instead of importing or binding the C++ implementation.
"""

import argparse
import json
import math
from pathlib import Path


RESERVE = 0.05 + 5.0 * math.sqrt(2.0)
MAX_RANGE = 400.0 - RESERVE


def rotate60(value, sign):
    x, y = value
    return (0.5 * x - sign * math.sqrt(3.0) * 0.5 * y,
            sign * math.sqrt(3.0) * 0.5 * x + 0.5 * y)


def norm(delta):
    return math.hypot(delta[0], delta[1])


def sub(first, second):
    return (first[0] - second[0], first[1] - second[1])


def lift(summary, active_unit, front):
    fixed = {int(key): tuple(value)
             for key, value in summary["fixed_positions"].items()}
    fronts = {key: tuple(value)
              for key, value in summary["initial_fronts"].items()}
    for unit in summary["coverage_units"]:
        if unit["id"] == active_unit:
            fronts[unit["id"]] = front
    targets = {}
    for unit in summary["coverage_units"]:
        anchors = [fixed[item] for item in unit["anchors"]]
        base = (sum(p[0] for p in anchors) / len(anchors),
                sum(p[1] for p in anchors) / len(anchors))
        displacement = sub(fronts[unit["id"]], base)
        for member in unit["members"]:
            role = summary["member_roles"][str(member)]
            sign = -1.0 if role["triangular"] < 0.0 else 1.0
            triangular = rotate60(displacement, sign)
            weight = abs(role["triangular"])
            targets[member] = (
                base[0] + role["axial"] * displacement[0] +
                weight * triangular[0],
                base[1] + role["axial"] * displacement[1] +
                weight * triangular[1])
    return fixed, targets


def sector_margin(pose, yaw, center):
    delta = sub(center, pose)
    distance = norm(delta)
    if not (distance > RESERVE and distance <= MAX_RANGE):
        return None
    error = abs(math.remainder(math.atan2(delta[1], delta[0]) - yaw,
                               2.0 * math.pi))
    angular = math.pi / 3.0 - error - math.asin(RESERVE / distance)
    if angular < 0.0:
        return None
    return min(distance - RESERVE, MAX_RANGE - distance,
               distance * angular), angular


def geometry(summary, fixed, targets):
    positions = dict(fixed)
    positions.update(targets)
    maximum_reference = max(
        norm(sub(positions[owner], positions[reference]))
        for reference, owner in summary["reference_edges"])
    minimum_separation = math.inf
    for mobile in range(1, 15):
        for other in range(mobile + 1, 15):
            minimum_separation = min(minimum_separation,
                                     norm(sub(targets[mobile], targets[other])))
        for fixed_position in fixed.values():
            minimum_separation = min(minimum_separation,
                                     norm(sub(targets[mobile], fixed_position)))
    minimum_fim = math.inf
    for mobile in range(1, 15):
        xx = xy = yy = 0.0
        for reference, owner in summary["reference_edges"]:
            if owner != mobile:
                continue
            delta = sub(positions[reference], targets[mobile])
            distance = norm(delta)
            if distance <= 1.0e-12:
                continue
            ux, uy = delta[0] / distance, delta[1] / distance
            xx += ux * ux
            xy += ux * uy
            yy += uy * uy
        eigen_min = 0.5 * (xx + yy -
                           math.sqrt((xx - yy) ** 2 + 4.0 * xy * xy))
        minimum_fim = min(minimum_fim, eigen_min)
    return maximum_reference, minimum_separation, minimum_fim


def decode_mask(value, count):
    if not value:
        return [False] * count
    if len(value) != (count + 3) // 4:
        raise ValueError("packed initial mask length mismatch")
    result = []
    for index in range(count):
        result.append(bool(int(value[index // 4], 16) & (1 << (index % 4))))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("evidence_dir", type=Path)
    parser.add_argument("--tolerance", type=float, default=1.0e-7)
    args = parser.parse_args()
    summary = json.loads((args.evidence_dir / "summary.json").read_text())
    witness_rows = [json.loads(line) for line in
                    (args.evidence_dir / "witnesses.jsonl").read_text().splitlines()
                    if line.strip()]
    queue_rows = [json.loads(line) for line in
                  (args.evidence_dir / "deadline-queues.jsonl").read_text().splitlines()
                  if line.strip()]
    grid = summary["grid"]
    cell_count = grid["cells"]
    initial = decode_mask(summary["initial_certified_bits_hex"], cell_count)
    expected = {f"{x}:{y}" for x in range(grid["x"])
                for y in range(grid["y"])
                if not initial[x * grid["y"] + y]}
    seen = set()
    maximum_reference = 0.0
    minimum_separation = math.inf
    minimum_fim = math.inf
    failures = []
    unit_ids = sorted(item["id"] for item in summary["coverage_units"])
    for row in witness_rows:
        cell_id = row["cell_id"]
        if cell_id in seen:
            failures.append(f"duplicate:{cell_id}")
            continue
        seen.add(cell_id)
        x_index = int(cell_id.split(":")[0])
        owner_index = min(len(unit_ids) - 1,
                          x_index * len(unit_ids) // grid["x"])
        if row["coverage_unit"] != unit_ids[owner_index]:
            failures.append(f"owner:{cell_id}")
        fixed, targets = lift(summary, row["coverage_unit"],
                              tuple(row["canonical_front"]))
        yaw = math.atan2(row["route_tangent"][1],
                         row["route_tangent"][0])
        margin = sector_margin(targets[row["service_member"]], yaw,
                               tuple(row["cell_center"]))
        if margin is None:
            failures.append(f"sector:{cell_id}")
            continue
        ref, separation, fim = geometry(summary, fixed, targets)
        maximum_reference = max(maximum_reference, ref)
        minimum_separation = min(minimum_separation, separation)
        minimum_fim = min(minimum_fim, fim)
        values = ((margin[0], row["sensing_margin_m"], "sensing"),
                  (margin[1], row["yaw_margin_rad"], "yaw"),
                  (ref, row["maximum_target_reference_m"], "reference"),
                  (separation, row["minimum_target_separation_m"], "separation"),
                  (fim, row["nominal_fim_proxy"], "fim"))
        for actual, recorded, label in values:
            if abs(actual - recorded) > args.tolerance * max(1.0, abs(actual)):
                failures.append(f"{label}:{cell_id}")
        expected_tier = ("nominal-compatible"
                         if ref < 850.0 and separation > 10.0
                         else "virtual-dependent")
        if row["tier"] != expected_tier:
            failures.append(f"tier:{cell_id}")
    queue_by_unit = {}
    for row in queue_rows:
        queue_by_unit.setdefault(row["coverage_unit"], []).append(row)
    for unit, rows in queue_by_unit.items():
        for index, row in enumerate(rows):
            if row["queue_index"] != index:
                failures.append(f"queue-index:{unit}:{index}")
        actual = [(row["canonical_s"], row["cell_id"]) for row in rows]
        if actual != sorted(actual):
            failures.append(f"queue-order:{unit}")
    if {row["cell_id"] for row in queue_rows} != seen:
        failures.append("queue-witness-bijection")
    missing = sorted(expected - seen)
    extra = sorted(seen - expected)
    result = {
        "protocol": "task23-deadline-oracle-python-independent-v1",
        "valid": not failures and not missing and not extra,
        "grid_cells": cell_count,
        "initial_certified_count_rebuilt": sum(initial),
        "witness_count_rebuilt": len(seen),
        "joint_count_rebuilt": sum(initial) + len(seen),
        "missing_count": len(missing),
        "extra_count": len(extra),
        "failure_count": len(failures),
        "failure_examples": failures[:20],
        "missing_examples": missing[:20],
        "extra_examples": extra[:20],
        "maximum_target_reference_m_rebuilt": maximum_reference,
        "minimum_target_separation_m_rebuilt": minimum_separation,
        "minimum_nominal_fim_proxy_rebuilt": minimum_fim,
        "boundary": "validates every published canonical witness and the full "
                    "initial-certified union; it does not independently optimize "
                    "which canonical point is chosen inside a service interval"
    }
    (args.evidence_dir / "independent-verification.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    raise SystemExit(0 if result["valid"] else 1)


if __name__ == "__main__":
    main()
