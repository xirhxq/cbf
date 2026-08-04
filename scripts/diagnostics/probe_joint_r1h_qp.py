"""Probe a joint neighborhood QP for an R1H infeasible frame (Form A).

Two-velocity decomposition used by the joint resolver:
    coe·u_self - coe·u_other >= -const - coe·v_other_current
for every hard row shared between two robots in the joint set.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import linprog


def _other_of(name: str) -> int | None:
    if name.startswith("safetyCBF(#") or name.startswith("fixedCommCBF(#"):
        return int(name.split("#")[1].rstrip(")"))
    return None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("data_path", type=Path)
    parser.add_argument("robot_id", type=int)
    parser.add_argument(
        "--limit", type=float, default=50.0,
    )
    parser.add_argument(
        "--partners", type=str, default="",
        help="comma-separated partner ids; default: most violated at u=0",
    )
    arguments = parser.parse_args(argv)

    frames = json.loads(arguments.data_path.read_text())["state"]
    frame = frames[-1]
    robots = {r["id"]: r for r in frame["robots"]}
    failed_id = arguments.robot_id

    def current_velocity(robot_id: int) -> np.ndarray:
        result = robots[robot_id]["opt"].get("result", {})
        return np.asarray(
            [result.get("vx", 0.0), result.get("vy", 0.0)], dtype=float
        )

    if arguments.partners:
        partners = [int(x) for x in arguments.partners.split(",")]
    else:
        rows = robots[failed_id]["opt"]["cbfNoSlack"]
        violated = [
            (row["const"], _other_of(row["name"]))
            for row in rows
            if _other_of(row["name"]) is not None
            and row["const"] < -1e-9
        ]
        violated.sort()
        partners = []
        for _, other in violated:
            if other not in partners and len(partners) < 3:
                partners.append(other)
    joint = [failed_id] + partners
    print("failed", failed_id, "partners", partners)

    slot = {robot_id: 2 * i for i, robot_id in enumerate(joint)}
    n_vars = 2 * len(joint) + 1  # planar + margin
    inequalities = []
    bounds = []

    def add_margin_row(coefficients: np.ndarray, constant: float) -> None:
        # m <= coeff·u - constant  =>  -coeff·u + m <= -constant
        inequalities.append(list(-coefficients) + [1.0])
        bounds.append(-constant)

    def add_plain(coefficients: np.ndarray, bound: float) -> None:
        inequalities.append(list(coefficients) + [0.0])
        bounds.append(bound)

    labels = []
    for robot_id in joint:
        for row in robots[robot_id]["opt"]["cbfNoSlack"]:
            coe = np.zeros(n_vars - 1)
            self_coe = np.asarray(
                [row["coe"]["vx"], row["coe"]["vy"]], dtype=float
            )
            other_id = _other_of(row["name"])
            if other_id is not None and other_id in slot:
                coe[slot[robot_id]:slot[robot_id] + 2] = self_coe
                coe[slot[other_id]:slot[other_id] + 2] = -self_coe
                rhs = (
                    -row["const"]
                    - float(self_coe @ current_velocity(other_id))
                )
            else:
                coe[slot[robot_id]:slot[robot_id] + 2] = self_coe
                rhs = -row["const"]
            add_margin_row(coe, rhs)
            labels.append(f"robot{robot_id}:{row['name']}")
    for variable in range(n_vars - 1):
        upper = np.zeros(n_vars - 1)
        upper[variable] = 1.0
        add_plain(upper, arguments.limit)
        lower = np.zeros(n_vars - 1)
        lower[variable] = -1.0
        add_plain(lower, arguments.limit)

    result = linprog(
        c=[0.0] * (n_vars - 1) + [-1.0],
        A_ub=np.asarray(inequalities, dtype=float),
        b_ub=np.asarray(bounds, dtype=float),
        bounds=[(None, None)] * n_vars,
        method="highs",
    )
    if result.status != 0:
        print("LP failed:", result.message)
        return 1
    print("joint max-min margin m = %.6f m/s" % result.x[-1])
    for i, robot_id in enumerate(joint):
        print(
            "  robot %d v=(%.2f, %.2f)"
            % (robot_id, result.x[2 * i], result.x[2 * i + 1])
        )
    margins = []
    for label, coe_row, bound in zip(labels, inequalities, bounds):
        value = float(np.asarray(coe_row[: n_vars - 1]) @ result.x[: n_vars - 1]) - bound
        margins.append((value, label))
    margins.sort()
    print("tightest margin rows:")
    for value, label in margins[:5]:
        print("  %-32s %.4f" % (label, value))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
