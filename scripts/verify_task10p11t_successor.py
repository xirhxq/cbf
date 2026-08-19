#!/usr/bin/env python3
"""Independent read-only verifier for the Task 10.11t one-step audit."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from verify_task10p11t_offline import (
    FIRST_OWNER,
    SECOND_OWNER,
    TOL,
    pair_rows,
    residuals,
    sha256,
    support_bound,
)


def controls(step: dict) -> dict[int, list[float]]:
    return {
        int(owner): [float(value) for value in control]
        for owner, control in step["controls"].items()
    }


def close_vector(left: list[float], right: list[float]) -> bool:
    return all(
        math.isclose(float(a), float(b), abs_tol=TOL)
        for a, b in zip(left, right, strict=True)
    )


def verify(
    snapshot_path: Path,
    current_audit_path: Path,
    successor_audit_path: Path,
) -> dict:
    snapshot = json.loads(snapshot_path.read_text())
    current_audit = json.loads(current_audit_path.read_text())
    audit = json.loads(successor_audit_path.read_text())
    current = audit["current"]
    successor = audit["successor"]
    current_controls = controls(current)
    successor_controls = controls(successor)
    rows = audit["successor_canonical_rows"]
    first, second = pair_rows(rows)

    lower = -support_bound(rows, first, FIRST_OWNER)
    upper = support_bound(rows, second, SECOND_OWNER)
    transfer = float(successor["pair"]["selected_transfer_mps2"])
    compiled: list[tuple[int, str, float]] = []
    for owner, control in successor_controls.items():
        compiled.extend(
            (owner, row_id, value)
            for row_id, value in residuals(
                rows, owner, control, transfer, first, second
            )
        )
    limiting_owner, limiting_id, minimum_residual = min(
        compiled, key=lambda item: item[2]
    )
    by_id = {(owner, row_id): value for owner, row_id, value in compiled}
    first_pair = by_id[(FIRST_OWNER, first["id"])]
    second_pair = by_id[(SECOND_OWNER, second["id"])]
    local_sum = first_pair + second_pair
    reserve = float(first["coefficient_uncertainty_reserve"])
    full_pair = local_sum + reserve

    reported_interval = successor["pair"][
        "burden_transfer_intervals_mps2"
    ]["shared"]
    prior_first = current_audit["independent_distributed_replay"][
        "first_control"
    ]
    prior_second = current_audit["independent_distributed_replay"][
        "second_control"
    ]
    assertions = {
        "frozen_snapshot_complete": snapshot["preflight"]["complete"] is True,
        "frozen_acceleration_box_unchanged": math.isclose(
            float(snapshot["canonical_request"]["acceleration_half_box"]),
            4.0,
            abs_tol=1.0e-12,
        ),
        "protocol_exact_zoh_one_step": (
            audit["protocol"] ==
            "task10p11t-exact-zoh-one-step-successor-v1"
            and audit["prediction"] == "exact_zoh_no_measurement_one_step"
            and math.isclose(float(audit["applied_dt_s"]), 0.1,
                             abs_tol=1.0e-12)
        ),
        "current_fourteen_owner_step_feasible": (
            current["feasible"] is True
            and current["owner_count"] == 14
            and set(current_controls) == set(range(1, 15))
        ),
        "current_delta_matches_coverage_first_audit": math.isclose(
            float(current["pair"]["selected_transfer_mps2"]),
            float(current_audit["selected_transfer_mps2"]),
            abs_tol=TOL,
        ),
        "current_pair_controls_match_coverage_first_audit": (
            close_vector(current_controls[FIRST_OWNER], prior_first)
            and close_vector(current_controls[SECOND_OWNER], prior_second)
        ),
        "successor_rebuilt_and_feasible": (
            audit["successor_performed"] is True
            and audit["successor_feasible"] is True
            and successor["feasible"] is True
        ),
        "successor_fourteen_owner_controls_complete": (
            successor["owner_count"] == 14
            and set(successor_controls) == set(range(1, 15))
        ),
        "all_successor_rows_present": (
            len(rows) == audit["successor_canonical_row_count"]
            and len(rows) == audit["current_canonical_row_count"]
        ),
        "successor_transfer_interval_matches_independent_vertices": (
            math.isclose(lower, float(reported_interval["lower"]), abs_tol=TOL)
            and math.isclose(upper, float(reported_interval["upper"]),
                             abs_tol=TOL)
        ),
        "successor_transfer_in_interval": lower - TOL <= transfer <= upper + TOL,
        "all_successor_local_rows_satisfied": minimum_residual >= -TOL,
        "reported_minimum_residual_matches": math.isclose(
            minimum_residual,
            float(successor["minimum_local_residual_mps2"]),
            abs_tol=TOL,
        ),
        "full_pair_row_implied_with_one_reserve": full_pair >= -TOL,
        "fixed_topology_and_no_measurement_update": (
            audit["fixed_topology_unchanged"] is True
            and audit["measurement_update_performed"] is False
        ),
        "successor_objective_boundary_explicit": (
            audit["current_nominal_objective_source"] ==
            "snapshot_nominal_controls"
            and audit["successor_nominal_objective_source"] ==
            "current_applied_controls_frozen_for_feasibility_audit"
            and audit["coverage_nominal_recomputed_at_successor"] is False
        ),
        "no_trajectory_or_recursive_claim": (
            audit["trajectory_run_performed"] is False
            and audit["production_controller_modified"] is False
            and audit["recursive_feasibility_claimed"] is False
        ),
    }
    return {
        "protocol": "task10p11t-independent-successor-verifier-v1",
        "passed": all(assertions.values()),
        "assertions": assertions,
        "snapshot_sha256": sha256(snapshot_path),
        "current_audit_sha256": sha256(current_audit_path),
        "successor_audit_sha256": sha256(successor_audit_path),
        "independent_successor_transfer_interval_mps2": [lower, upper],
        "selected_successor_transfer_mps2": transfer,
        "minimum_recomputed_successor_local_residual_mps2": minimum_residual,
        "limiting_owner": limiting_owner,
        "limiting_row_id": limiting_id,
        "successor_dynamic_local_pair_sum_residual_mps2": local_sum,
        "successor_once_reserve_full_pair_residual_mps2": full_pair,
        "claim_boundary": (
            "one_step_no_measurement_successor_feasibility_"
            "not_recursive_feasibility"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("current_audit", type=Path)
    parser.add_argument("successor_audit", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    result = verify(
        args.snapshot, args.current_audit, args.successor_audit
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
