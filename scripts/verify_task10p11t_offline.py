#!/usr/bin/env python3
"""Independent read-only verifier for the Task 10.11t pair audit."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

PAIR_BASE = "reference:2->4"
FIRST_OWNER = 2
SECOND_OWNER = 4
TOL = 1.0e-7


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def pair_rows(rows: list[dict]) -> tuple[dict, dict]:
    selected = [
        row for row in rows
        if row["id"].split(":owner:", 1)[0] == PAIR_BASE
    ]
    if len(selected) != 2:
        raise ValueError("selected pair must have exactly two rows")
    selected.sort(key=lambda row: row["owner"])
    first, second = selected
    if (
        first["owner"] != FIRST_OWNER
        or second["owner"] != SECOND_OWNER
        or first["peer"] != SECOND_OWNER
        or second["peer"] != FIRST_OWNER
        or any(
            abs(float(left) + float(right)) > 1.0e-10
            for left, right in zip(
                first["control_coefficient"],
                second["control_coefficient"], strict=True
            )
        )
    ):
        raise ValueError("selected pair rows are incoherent")
    return first, second


def dot(left: list[float], right: list[float]) -> float:
    return float(left[0]) * float(right[0]) + float(left[1]) * float(right[1])


def support_bound(rows: list[dict], target: dict, owner: int) -> float:
    constraints: list[tuple[list[float], float]] = [
        ([float(value) for value in row["control_coefficient"]],
         float(row["constant"]))
        for row in rows
        if row["owner"] == owner and row["id"] != target["id"]
    ]
    constraints.extend([
        ([1.0, 0.0], 4.0), ([-1.0, 0.0], 4.0),
        ([0.0, 1.0], 4.0), ([0.0, -1.0], 4.0),
    ])
    vertices: list[list[float]] = []
    for index, (first_a, first_c) in enumerate(constraints):
        for second_a, second_c in constraints[index + 1:]:
            determinant = first_a[0] * second_a[1] - first_a[1] * second_a[0]
            if abs(determinant) < 1.0e-12:
                continue
            point = [
                (-first_c * second_a[1] + first_a[1] * second_c) /
                    determinant,
                (-first_a[0] * second_c + first_c * second_a[0]) /
                    determinant,
            ]
            if all(dot(a, point) + constant >= -TOL
                   for a, constant in constraints):
                vertices.append(point)
    if not vertices:
        raise RuntimeError(f"no feasible support vertices for owner {owner}")
    target_a = [float(value) for value in target["control_coefficient"]]
    return max(dot(target_a, point) + float(target["constant"])
               for point in vertices)


def residuals(
    rows: list[dict], owner: int, control: list[float],
    transfer: float, first: dict, second: dict
) -> list[tuple[str, float]]:
    result: list[tuple[str, float]] = []
    for row in rows:
        if row["owner"] != owner:
            continue
        constant = float(row["constant"])
        if row["id"] == first["id"]:
            constant += transfer
        elif row["id"] == second["id"]:
            constant -= transfer
        coefficient = [float(value) for value in row["control_coefficient"]]
        result.append((row["id"], dot(coefficient, control) + constant))
    return result


def verify(snapshot_path: Path, result_path: Path) -> dict:
    snapshot = json.loads(snapshot_path.read_text())
    audit = json.loads(result_path.read_text())
    rows = snapshot["actual_rows"]
    first, second = pair_rows(rows)

    first_support = support_bound(rows, first, FIRST_OWNER)
    second_support = support_bound(rows, second, SECOND_OWNER)
    lower = -first_support
    upper = second_support
    transfer = float(audit["selected_transfer_mps2"])
    first_control = [float(value) for value in
        audit["independent_distributed_replay"]["first_control"]]
    second_control = [float(value) for value in
        audit["independent_distributed_replay"]["second_control"]]
    compiled = (
        residuals(rows, FIRST_OWNER, first_control, transfer, first, second)
        + residuals(rows, SECOND_OWNER, second_control, transfer, first, second)
    )
    limiting_id, minimum_residual = min(compiled, key=lambda item: item[1])
    first_pair = dict(compiled)[first["id"]]
    second_pair = dict(compiled)[second["id"]]
    reserve = float(first["coefficient_uncertainty_reserve"])
    local_sum = first_pair + second_pair
    full_pair = local_sum + reserve
    central = 2.0 * (float(first["constant"]) + reserve)
    bounded_half_width = 0.5 * abs(central)
    bounded_fraction_feasible = (
        max(lower, -bounded_half_width)
        <= min(upper, bounded_half_width) + TOL
    )
    assertions = {
        "snapshot_preflight_complete": snapshot["preflight"]["complete"] is True,
        "owner_count_14": snapshot["preflight"]["owner_count"] == 14,
        "row_count_matches": len(rows) == audit["canonical_row_count"],
        "transfer_interval_matches": (
            math.isclose(
                lower,
                audit["burden_transfer_intervals_mps2"]["shared"]["lower"],
                abs_tol=TOL,
            )
            and math.isclose(
                upper,
                audit["burden_transfer_intervals_mps2"]["shared"]["upper"],
                abs_tol=TOL,
            )
        ),
        "selected_transfer_in_interval": lower - TOL <= transfer <= upper + TOL,
        "all_local_rows_satisfied": minimum_residual >= -TOL,
        "transfer_cancels_in_pair_sum": math.isclose(
            full_pair, local_sum + reserve, abs_tol=1.0e-12
        ),
        "full_pair_row_implied": full_pair >= -TOL,
        "bounded_fraction_result_matches": (
            bounded_fraction_feasible is audit["bounded_fraction_feasible"]
        ),
        "no_trajectory_performed": audit["trajectory_run_performed"] is False,
        "no_production_controller_modified": (
            audit["production_controller_modified"] is False
        ),
    }
    return {
        "protocol": "task10p11t-independent-offline-verifier-v1",
        "passed": all(assertions.values()),
        "assertions": assertions,
        "snapshot_sha256": sha256(snapshot_path),
        "audit_sha256": sha256(result_path),
        "independent_transfer_interval_mps2": [lower, upper],
        "selected_transfer_mps2": transfer,
        "minimum_recomputed_local_residual_mps2": minimum_residual,
        "limiting_row_id": limiting_id,
        "dynamic_local_pair_sum_residual_mps2": local_sum,
        "once_reserve_full_pair_residual_mps2": full_pair,
        "bounded_fraction_feasible": bounded_fraction_feasible,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("audit", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    result = verify(args.snapshot, args.audit)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
