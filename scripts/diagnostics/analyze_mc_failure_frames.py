"""Joint analysis of MC containment failures, FIM-optimism, and true-distance
margins across the 20 estimator-in-loop Monte Carlo runs (jr_j5_full).
"""

from __future__ import annotations

import json
import glob
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
KAPPA = 3.0
SAFE = 10.0
SEEDS = [f"202608{s}" for s in range(1501, 1521)]


def assigned_loc_refs(rid: int) -> list[int]:
    """Paper triangular-ladder assigned CBF references (global UAV ids)."""
    if rid <= 7:
        squad1, local = True, rid
    else:
        squad1, local = False, rid - 7
    if local == 1:
        return [16, 15] if squad1 else [16, 17]
    if local == 2:
        return [rid - 1, 16]
    return [rid - 1, rid - 2]


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


def analyze_run(seed: str) -> dict:
    run = ROOT / seed
    data = json.loads((run / "config.materialized.json").read_text())
    bases = _load_materialized_bases({"config": data})
    base_pos = {15: np.asarray(bases[0]), 16: np.asarray(bases[1]), 17: np.asarray(bases[2])}
    deployment = {
        index + 1: [float(x) for x in position]
        for index, position in enumerate(data["initial"]["position"]["positions"])
    }
    data_path = glob.glob(str(run / "*" / "data.json"))[0]
    sim = json.loads(Path(data_path).read_text())["state"]
    log = [
        json.loads(line)
        for line in (run / "estimates-log.jsonl").read_text().splitlines()
    ]
    service = EKFInLoopService(
        deployment_positions=deployment,
        bases=bases,
        anchor_covariance_scale=KAPPA,
    )
    rng = np.random.default_rng(int(seed))

    max_loc = 0.0
    min_pair = np.inf
    fails = []
    total = 0
    mismatches = 0
    checked = 0
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
            held = {entry["id"]: [0.0, 0.0] for entry in frame["robots"]}
        else:
            held = {
                entry["id"]: [entry["opt"]["result"]["vx"], entry["opt"]["result"]["vy"]]
                for entry in sim[frame_i - 1]["robots"]
            }
        refs = build_ekf_raw_references(frame_like, bases, rng, sigma0=SIGMA0)
        outputs = service.step(
            frame_index=frame_i,
            raw_reference_groups=refs,
            held_commands=held,
        )
        log_frame = log[frame_i]
        for entry in log_frame["robots"]:
            checked += 1
            if abs(outputs[entry["id"]]["epsilon"] - entry["epsilon"]) > 1e-6:
                mismatches += 1
        for robot in frame["robots"]:
            rid = robot["id"]
            err = float(np.linalg.norm(outputs[rid]["estimate"] - truth[rid]))
            eps_ekf = outputs[rid]["epsilon"]
            total += 1
            # true distances on assigned loc links
            for ref in assigned_loc_refs(rid):
                pos = base_pos[ref] if ref in base_pos else truth[ref]
                max_loc = max(max_loc, float(np.linalg.norm(truth[rid] - pos)))
            if err > eps_ekf:
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
                loc_true = [
                    float(np.linalg.norm(truth[rid] - (base_pos[ref] if ref in base_pos else truth[ref])))
                    for ref in assigned_loc_refs(rid)
                ]
                pair_min = min(
                    float(np.linalg.norm(truth[rid] - truth[o]))
                    for o in truth if o != rid
                )
                fails.append(
                    {
                        "frame": frame_i,
                        "uav": rid,
                        "err": err,
                        "eps": eps_ekf,
                        "ratio": err / eps_ekf,
                        "fim_optimistic": (eps_fim is not None and eps_fim < eps_ekf),
                        "max_loc": max(loc_true),
                        "min_pair": pair_min,
                    }
                )
        # all-pair minimum distance
        ids = list(truth)
        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                min_pair = min(
                    min_pair,
                    float(np.linalg.norm(truth[ids[a]] - truth[ids[b]])),
                )
    return {
        "seed": seed,
        "max_loc": max_loc,
        "min_pair": min_pair,
        "containment": 1.0 - len(fails) / total,
        "fails": fails,
        "total": total,
        "mismatches": mismatches,
        "checked": checked,
    }


def main() -> None:
    results = []
    for seed in SEEDS:
        print("analyzing", seed, flush=True)
        results.append(analyze_run(seed))
    max_loc = max(r["max_loc"] for r in results)
    min_pair = min(r["min_pair"] for r in results)
    all_fails = [f for r in results for f in r["fails"]]
    pooled = 1.0 - len(all_fails) / sum(r["total"] for r in results)
    print("replay-vs-log epsilon mismatches:", sum(r["mismatches"] for r in results),
          "/", sum(r["checked"] for r in results))
    opt_frac = (
        sum(1 for f in all_fails if f["fim_optimistic"]) / len(all_fails)
        if all_fails else 0.0
    )
    ratios = sorted(f["ratio"] for f in all_fails)
    loc_margins = [850.0 - f["max_loc"] for f in all_fails]
    pair_margins = [f["min_pair"] - SAFE for f in all_fails]
    print("=" * 70)
    print("max true assigned-loc distance across runs: %.2f m (margin to 850: %.2f)"
          % (max_loc, 850.0 - max_loc))
    print("min true inter-UAV distance across runs: %.2f m (margin to 10: %.2f)"
          % (min_pair, min_pair - SAFE))
    print("pooled containment: %.5f (fails %d / %d)"
          % (pooled, len(all_fails), sum(r["total"] for r in results)))
    print("fails per run:", {r["seed"]: len(r["fails"]) for r in results})
    if all_fails:
        print("fail breach ratio: min %.3f median %.3f max %.3f"
              % (min(ratios), ratios[len(ratios)//2], max(ratios)))
        print("fraction of fail frames with FIM< EKF: %.4f" % opt_frac)
        print("loc margin at fail frames: min %.2f median %.2f"
              % (min(loc_margins), sorted(loc_margins)[len(loc_margins)//2]))
        print("pair margin at fail frames: min %.2f median %.2f"
              % (min(pair_margins), sorted(pair_margins)[len(pair_margins)//2]))


if __name__ == "__main__":
    main()
