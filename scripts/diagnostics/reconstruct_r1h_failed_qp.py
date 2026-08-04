"""Reconstruct the hard no-slack QP of a failed R1H frame as a Chebyshev LP.

Consumes an R1H/R1H-EI ``data.json`` that terminated with a failed solver
record, takes the last logged frame and the given robot id, rebuilds the hard
rows plus the planar input limits, and reports the maximum worst-case margin
``m`` (negative = infeasible) and the tightest rows.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import linprog


PLANAR_LIMIT_MPS = 50.0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data_path", type=Path)
    parser.add_argument("robot_id", type=int)
    arguments = parser.parse_args(argv)

    frames = json.loads(arguments.data_path.read_text())["state"]
    frame = frames[-1]
    robot = next(r for r in frame["robots"] if r["id"] == arguments.robot_id)
    opt = robot["opt"]
    rows = opt.get("cbfNoSlack", [])
    print(
        "frame t=%.1f robot %d status=%s constraints=%s vars=%s"
        % (
            frame.get("runtime"),
            arguments.robot_id,
            opt.get("status"),
            opt.get("solver_info", {}).get("constraints_count"),
            opt.get("solver_info", {}).get("vars_count"),
        )
    )
    print("hard rows:", len(rows))

    inequality_matrix = []
    inequality_bounds = []
    for row in rows:
        coe = row["coe"]
        inequality_matrix.append(
            [-coe["vx"], -coe["vy"], 1.0]
        )
        inequality_bounds.append(row["const"])
    for variable in (0, 1):
        upper = [0.0, 0.0, 0.0]
        upper[variable] = 1.0
        inequality_matrix.append(upper)
        inequality_bounds.append(PLANAR_LIMIT_MPS)
        lower = [0.0, 0.0, 0.0]
        lower[variable] = -1.0
        inequality_matrix.append(lower)
        inequality_bounds.append(PLANAR_LIMIT_MPS)

    result = linprog(
        c=[0.0, 0.0, -1.0],
        A_ub=np.asarray(inequality_matrix, dtype=float),
        b_ub=np.asarray(inequality_bounds, dtype=float),
        bounds=[(None, None)] * 3,
        method="highs",
    )
    if result.status != 0:
        print("LP solve failed:", result.message)
        return 1
    margin = float(result.x[2])
    print("max-min margin m = %.6f m/s" % margin)
    print("witness u* = (%.4f, %.4f)" % (result.x[0], result.x[1]))
    evaluated = [
        (
            row["name"],
            row["coe"]["vx"] * result.x[0]
            + row["coe"]["vy"] * result.x[1]
            + row["const"],
        )
        for row in rows
    ]
    evaluated.sort(key=lambda entry: entry[1])
    print("tightest rows at u*:")
    for name, value in evaluated[:6]:
        print("  %-22s %.4f m/s" % (name, value))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
