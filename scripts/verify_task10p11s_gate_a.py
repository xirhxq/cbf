#!/usr/bin/env python3
"""Independent, standard-library LP verification for Task 10.11s Gate A."""

from __future__ import annotations

import hashlib
import itertools
import json
import math
import platform
import sys
from pathlib import Path

def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def solve_3x3(matrix: list[list[float]], rhs: list[float]) -> list[float] | None:
    augmented = [row[:] + [value] for row, value in zip(matrix, rhs)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            return None
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        scale = augmented[column][column]
        augmented[column] = [value / scale for value in augmented[column]]
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                value - factor * pivot_value
                for value, pivot_value in zip(augmented[row], augmented[column])
            ]
    return [augmented[row][3] for row in range(3)]


def solve_gamma(rows: list[dict], half_box: float) -> float:
    # Maximise gamma in (ux, uy, gamma). The optimum is a vertex formed by
    # three active row/box half-spaces, so enumerate those vertices directly.
    constraints: list[tuple[list[float], float]] = []
    for row in rows:
        normal = row["normal"]
        constraints.append(([-float(normal[0]), -float(normal[1]), 1.0], float(row["constant"])))
    constraints.extend(
        [
            ([1.0, 0.0, 0.0], half_box),
            ([-1.0, 0.0, 0.0], half_box),
            ([0.0, 1.0, 0.0], half_box),
            ([0.0, -1.0, 0.0], half_box),
        ]
    )

    optimum = -math.inf
    for active in itertools.combinations(constraints, 3):
        candidate = solve_3x3(
            [constraint[0] for constraint in active],
            [constraint[1] for constraint in active],
        )
        if candidate is None:
            continue
        if all(
            sum(coefficient * value for coefficient, value in zip(lhs, candidate))
            <= rhs + 1e-9
            for lhs, rhs in constraints
        ):
            optimum = max(optimum, candidate[2])
    if not math.isfinite(optimum):
        raise RuntimeError("independent vertex enumeration found no feasible LP vertex")
    return optimum


def main() -> int:
    if len(sys.argv) != 4:
        print("usage: verify_task10p11s_gate_a.py INPUT_JSON GATE_A_JSON OUTPUT_JSON")
        return 2

    input_path = Path(sys.argv[1])
    gate_path = Path(sys.argv[2])
    output_path = Path(sys.argv[3])
    output_path.parent.mkdir(parents=True, exist_ok=True)

    verification: dict = {
        "protocol": "task10p11s-independent-vertex-lp-v1",
        "python_version": platform.python_version(),
        "solver": "standard-library enumeration of 3-active-constraint LP vertices",
        "input_sha256": sha256(input_path),
        "gate_a_sha256": sha256(gate_path),
    }
    try:
        evidence = json.loads(input_path.read_text())
        gate = json.loads(gate_path.read_text())
        hard = evidence["timeline"][-1]["hard_polytope"]
        rows = hard["rows"]
        half_box = float(hard["input_half_box_mps2"])
        local_gamma = solve_gamma(rows, half_box)

        selected_id = "reference:2->4:owner:2"
        selected = next(row for row in rows if row["id"] == selected_id)
        reserve = float(selected["coefficient_reserve_mps2"])
        full_constant = 2.0 * float(selected["constant"]) + reserve
        peer_box_support = half_box * sum(abs(float(v)) for v in selected["normal"])
        relaxed_row = dict(selected)
        relaxed_row["id"] = selected_id + ":independent-relaxation"
        relaxed_row["constant"] = full_constant + peer_box_support
        relaxed_rows = [row for row in rows if row["id"] != selected_id]
        relaxed_rows.append(relaxed_row)
        relaxed_gamma = solve_gamma(relaxed_rows, half_box)

        if abs(local_gamma - float(gate["limiting_gamma_mps2"])) > 1e-9:
            raise AssertionError("local gamma differs from Gate A output")
        if int(gate.get("limiting_owner", -1)) != int(hard["owner"]):
            raise AssertionError("limiting owner differs from frozen hard-polytope owner")
        if gate.get("snapshot_digest") != verification["input_sha256"]:
            raise AssertionError("Gate A snapshot digest differs from independent input hash")
        if local_gamma >= 0.0 or relaxed_gamma < 0.0:
            raise AssertionError("same-half/relaxation signs do not match the stop claim")
        if gate["reason"] != "frozen_snapshot_incomplete_for_full_28d":
            raise AssertionError("Gate A did not retain the completeness stop")
        expected_fields = {
            "snapshot_complete": False,
            "same_half_decidable": True,
            "same_half_feasible": False,
            "full_pair_relaxation_decidable": True,
            "full_pair_relaxation_feasible": True,
            "full_pair_status": "relaxed_full_pair_feasible_full_28d_undetermined",
            "full_pair_complete_28d_decidable": False,
            "successor_audit_authorized": False,
        }
        for field, expected in expected_fields.items():
            if gate.get(field) != expected:
                raise AssertionError(
                    f"Gate A field {field!r} differs from independent conclusion"
                )
        if gate.get("provenance", {}).get("independent_verifier", {}).get(
            "result"
        ) != "pending":
            raise AssertionError("Gate A provenance did not retain verifier pending state")
        if gate["gate_b_authorized"] or gate["gate_b_run_performed"]:
            raise AssertionError("Gate B was authorized or run")
        if gate["long_horizon_rerun_performed"]:
            raise AssertionError("long-horizon rerun was recorded")

        verification.update(
            {
                "result": "passed",
                "owner": int(hard["owner"]),
                "independent_local_gamma_mps2": local_gamma,
                "independent_relaxed_full_pair_gamma_mps2": relaxed_gamma,
                "same_half_feasible": False,
                "full_pair_relaxation_feasible": True,
                "full_28d_determined": False,
                "gate_b_authorized": False,
            }
        )
        output_path.write_text(json.dumps(verification, indent=2) + "\n")
        print(json.dumps(verification, indent=2))
        return 0
    except Exception as error:  # retain verifier failure evidence
        verification.update({"result": "failed", "error": str(error)})
        output_path.write_text(json.dumps(verification, indent=2) + "\n")
        print(json.dumps(verification, indent=2), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
