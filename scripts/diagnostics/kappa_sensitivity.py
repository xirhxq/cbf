"""Containment sensitivity of the EKF to the anchor-covariance scale kappa,
replayed on the 20 estimator-in-the-loop runs (same closed-loop trajectories)."""

from __future__ import annotations

import glob
import json
from pathlib import Path

import numpy as np

from scripts.diagnostics.ekf_estimator_service import (
    EKFInLoopService,
    build_ekf_raw_references,
)
from scripts.diagnostics.replay_r1h_estimator import _load_materialized_bases


ROOT = Path("/private/tmp/cbf2026-mc-ei/jr_j5_full")
SIGMA0 = 0.5
RANGE0 = 850.0
SEEDS = [f"202608{s}" for s in range(1501, 1521)]


def containment_for_kappa(kappa: float) -> dict:
    pooled_fails = 0
    pooled_total = 0
    per_run = []
    for seed in SEEDS:
        run = ROOT / seed
        data = json.loads((run / "config.materialized.json").read_text())
        bases = _load_materialized_bases({"config": data})
        deployment = {
            index + 1: [float(x) for x in position]
            for index, position in enumerate(data["initial"]["position"]["positions"])
        }
        data_path = glob.glob(str(run / "*" / "data.json"))[0]
        sim = json.loads(Path(data_path).read_text())["state"]
        service = EKFInLoopService(
            deployment_positions=deployment,
            bases=bases,
            anchor_covariance_scale=kappa,
        )
        rng = np.random.default_rng(int(seed))
        fails = 0
        total = 0
        for frame_i, frame in enumerate(sim):
            truth = {
                r["id"]: np.asarray([r["state"]["x"], r["state"]["y"]])
                for r in frame["robots"]
            }
            frame_like = {
                "robots": [
                    {
                        "id": entry["id"],
                        "state": {"x": entry["state"]["x"], "y": entry["state"]["y"]},
                    }
                    for entry in frame["robots"]
                ],
                "covariance_formation": frame["covariance_formation"],
                "formation": frame["formation"],
            }
            if frame_i == 0:
                held = {e["id"]: [0.0, 0.0] for e in frame["robots"]}
            else:
                held = {
                    e["id"]: [e["opt"]["result"]["vx"], e["opt"]["result"]["vy"]]
                    for e in sim[frame_i - 1]["robots"]
                }
            refs = build_ekf_raw_references(frame_like, bases, rng, sigma0=SIGMA0)
            outputs = service.step(
                frame_index=frame_i,
                raw_reference_groups=refs,
                held_commands=held,
            )
            for robot in frame["robots"]:
                rid = robot["id"]
                err = float(
                    np.linalg.norm(outputs[rid]["estimate"] - truth[rid])
                )
                total += 1
                if err > outputs[rid]["epsilon"]:
                    fails += 1
        pooled_fails += fails
        pooled_total += total
        per_run.append(1.0 - fails / total)
    return {
        "kappa": kappa,
        "pooled": 1.0 - pooled_fails / pooled_total,
        "per_run_min": min(per_run),
        "per_run_median": float(np.median(per_run)),
        "fails": pooled_fails,
        "total": pooled_total,
    }


def main() -> None:
    for kappa in (2.0, 3.0, 4.0):
        r = containment_for_kappa(kappa)
        print(
            "kappa %.1f: pooled containment %.5f (fails %d/%d), "
            "per-run min %.5f, median %.5f"
            % (
                r["kappa"],
                r["pooled"],
                r["fails"],
                r["total"],
                r["per_run_min"],
                r["per_run_median"],
            )
        )


if __name__ == "__main__":
    main()
