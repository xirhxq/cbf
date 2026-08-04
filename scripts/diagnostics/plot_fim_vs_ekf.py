"""Candidate M2 figure: FIM-approximated vs EKF-reported uncertainty radius."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from scripts.diagnostics.ekf_estimator_service import (
    EKFInLoopService,
    build_ekf_raw_references,
)
from scripts.diagnostics.replay_r1h_estimator import _load_materialized_bases


ROOT = Path("/private/tmp/cbf2026-route1-dynamic")
RUN = Path("/private/tmp/r1h-ekf-avail/2026081303")
SIM_DATA = Path(
    "/Users/xirhxq/Documents/Clones/cbf/data/"
    "2026-08-04_19-09-15_R1H-EI_seed_20260727_350s/data.json"
)
OUT = Path(
    "/Users/xirhxq/Documents/Clones/cbf/papers/CBF2026/assets/"
    "2026-08-04_19-09-15_R1H-EI_seed_20260727_350s/"
    "fim-vs-ekf-epsilon.png"
)
SIGMA0 = 0.5
RANGE0 = 850.0
KAPPA = 3.0
RANGE_NOISE_SEED = 2026081303


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
    data = json.loads((RUN / "config.materialized.json").read_text())
    bases = _load_materialized_bases({"config": data})
    deployment = {
        index + 1: [float(x) for x in position]
        for index, position in enumerate(data["initial"]["position"]["positions"])
    }
    sim = json.loads(SIM_DATA.read_text())["state"]
    service = EKFInLoopService(
        deployment_positions=deployment,
        bases=bases,
        anchor_covariance_scale=KAPPA,
    )
    rng = np.random.default_rng(RANGE_NOISE_SEED)
    series = {uav: {"t": [], "fim": [], "ekf": []} for uav in (1, 3, 5, 7)}
    all_fim: list[float] = []
    all_ekf: list[float] = []
    for frame_i, frame in enumerate(sim):
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
        held = {
            entry["id"]: [entry["opt"]["result"]["vx"], entry["opt"]["result"]["vy"]]
            for entry in frame["robots"]
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
            eps_ekf = 3.0 * float(np.sqrt(max(np.linalg.eigvalsh(cov)[-1], 0.0)))
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
            eps_fim = fim_epsilon(
                np.asarray(outputs[rid]["estimate"]), ref_list
            )
            if eps_fim is None:
                continue
            all_fim.append(eps_fim)
            all_ekf.append(eps_ekf)
            if rid in series:
                series[rid]["t"].append(frame_i * 0.5)
                series[rid]["fim"].append(eps_fim)
                series[rid]["ekf"].append(eps_ekf)

    ratio = np.asarray(all_ekf) / np.asarray(all_fim)
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
    fig, axes = plt.subplots(1, 1, figsize=(4.0, 3.0))
    axes.scatter(all_fim, all_ekf, s=3, alpha=0.25, color="#1f77b4",
                 rasterized=True)
    lim = [0, max(max(all_fim), max(all_ekf)) * 1.05]
    axes.plot(lim, lim, "k--", lw=1)
    axes.set_xlabel(r"FIM $\varepsilon$ (m)")
    axes.set_ylabel(r"EKF $\varepsilon$ (m)")
    axes.set_xlim(lim)
    axes.set_ylim(lim)
    axes.set_title("All robot-frames")
    axes.grid(True, alpha=0.3)
    axes.text(
        0.05, 0.96,
        f"median EKF/FIM = {np.median(ratio):.2f}\n"
        f"p95 = {np.percentile(ratio, 95):.2f}\n"
        f"FIM > EKF in {100.0*np.mean(np.asarray(all_fim) > np.asarray(all_ekf)):.1f}%",
        transform=axes.transAxes, va="top", fontsize=8,
        bbox=dict(boxstyle="round", fc="white", alpha=0.8),
    )
    fig.tight_layout()
    fig.savefig(OUT, dpi=300, bbox_inches="tight")
    print("saved", OUT)
    print("median ratio", round(float(np.median(ratio)), 3),
          "p95", round(float(np.percentile(ratio, 95)), 3),
          "fim>ekf", round(float(100.0*np.mean(np.asarray(all_fim) > np.asarray(all_ekf))), 1))


if __name__ == "__main__":
    main()
