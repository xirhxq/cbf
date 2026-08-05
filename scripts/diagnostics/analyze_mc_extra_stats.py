"""Extra MC statistics: ANEES, FIM-vs-EKF ratio over all frames, velocity
saturation, and containment-failure spatiotemporal distribution."""

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

from analyze_mc_failure_frames import assigned_loc_refs


ROOT = Path("/private/tmp/cbf2026-mc-ei/jr_j5_full")
SIGMA0 = 0.5
RANGE0 = 850.0
KAPPA = 3.0
SEEDS = [f"202608{s}" for s in range(1501, 1521)]


def fim_epsilon(
    my_estimate: np.ndarray,
    references: list[tuple[str, int, np.ndarray, np.ndarray]],
) -> float | None:
    rows: list[np.ndarray] = []
    weights: list[float] = []
    for kind, ident, anchor_mean, anchor_cov in references:
        diff = my_estimate - anchor_mean
        dist = float(np.linalg.norm(diff))
        if dist < 1e-6:
            return None
        n = diff / dist
        sigma_sq = (SIGMA0 * (1.0 + (dist / RANGE0) ** 2)) ** 2
        w = sigma_sq + float(n @ anchor_cov @ n)
        if w <= 0.0 or not np.isfinite(w):
            return None
        rows.append(n)
        weights.append(1.0 / w)
    if len(rows) < 2:
        return None
    J = np.stack(rows, axis=0)
    Phi = J.T @ np.diag(weights) @ J
    try:
        cov = np.linalg.inv(Phi)
    except np.linalg.LinAlgError:
        return None
    if not np.all(np.isfinite(cov)):
        return None
    return 3.0 * float(np.sqrt(max(np.linalg.eigvalsh(cov)[-1], 0.0)))


def main() -> None:
    ratios_all: list[float] = []
    nees_all: list[float] = []
    sat_frames = 0
    max_speed = 0.0
    max_vx = 0.0
    max_vy = 0.0
    max_yaw = 0.0
    fails: list[dict] = []
    per_run_anees: list[float] = []
    per_run_ratio_med: list[float] = []
    total = 0
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
            anchor_covariance_scale=KAPPA,
        )
        rng = np.random.default_rng(int(seed))
        run_nees = []
        run_ratios = []
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
                cov = service.state[rid]["cov"]
                err = outputs[rid]["estimate"] - truth[rid]
                eps_ekf = outputs[rid]["epsilon"]
                nees = float(err @ np.linalg.solve(cov, err)) / 2.0
                run_nees.append(nees)
                nees_all.append(nees)
                total += 1
                cinfo = next(
                    c for c in frame["covariance_formation"] if c["id"] == rid
                )
                ref_list = []
                for base_id in cinfo["baseIds"]:
                    ref_list.append(
                        ("base", int(base_id), np.asarray(bases[base_id]), np.zeros((2, 2)))
                    )
                for anchor_id in cinfo["anchorIds"]:
                    ref_list.append(
                        (
                            "uav",
                            int(anchor_id),
                            service.state[anchor_id]["mean"],
                            service.state[anchor_id]["cov"],
                        )
                    )
                eps_fim = fim_epsilon(np.asarray(outputs[rid]["estimate"]), ref_list)
                if eps_fim is not None and eps_fim > 1e-9:
                    run_ratios.append(eps_ekf / eps_fim)
                    ratios_all.append(eps_ekf / eps_fim)
                # saturation and speeds
                opt = robot.get("opt", {})
                lim = opt.get("input_limits", {})
                if lim.get("saturated", {}).get("any", False):
                    sat_frames += 1
                res = opt.get("result", {})
                speed = float(np.hypot(res.get("vx", 0.0), res.get("vy", 0.0)))
                max_speed = max(max_speed, speed)
                max_vx = max(max_vx, abs(res.get("vx", 0.0)))
                max_vy = max(max_vy, abs(res.get("vy", 0.0)))
                max_yaw = max(max_yaw, abs(res.get("yawRateRad", 0.0)))
                if float(np.linalg.norm(err)) > eps_ekf:
                    fails.append(
                        {
                            "seed": seed,
                            "frame": frame_i,
                            "uav": rid,
                            "t": frame_i * 0.5,
                            "err": float(np.linalg.norm(err)),
                            "eps": eps_ekf,
                        }
                    )
        per_run_anees.append(float(np.mean(run_nees)))
        per_run_ratio_med.append(float(np.median(run_ratios)))

    ratios = np.asarray(ratios_all)
    nees = np.asarray(nees_all)
    print("=" * 70)
    print("ANEES pooled: %.3f  (per-run mean %.3f, min %.3f, max %.3f)"
          % (nees.mean(), np.mean(per_run_anees), min(per_run_anees), max(per_run_anees)))
    print("FIM ratio (EKF/FIM) pooled: median %.3f p95 %.3f; FIM< EKF in %.1f%%"
          % (np.median(ratios), np.percentile(ratios, 95),
             100.0 * np.mean(ratios > 1.0)))
    print("per-run FIM ratio medians: min %.3f max %.3f"
          % (min(per_run_ratio_med), max(per_run_ratio_med)))
    print("saturated robot-frames: %d / %d (%.4f%%)"
          % (sat_frames, total, 100.0 * sat_frames / total))
    print("max commanded speed %.2f m/s | vx | %.2f | vy | %.2f | yaw | %.4f rad/s"
          % (max_speed, max_vx, max_vy, max_yaw))
    print("fails:", len(fails))
    uav_counts = {}
    tmin, tmax = 1e9, -1
    for f in fails:
        uav_counts[f["uav"]] = uav_counts.get(f["uav"], 0) + 1
        tmin = min(tmin, f["t"])
        tmax = max(tmax, f["t"])
    print("fail UAV histogram:", dict(sorted(uav_counts.items())))
    print("fail time range: %.1f - %.1f s" % (tmin, tmax))
    from collections import Counter
    print("fails per seed:", dict(Counter(f["seed"] for f in fails)))


if __name__ == "__main__":
    main()
