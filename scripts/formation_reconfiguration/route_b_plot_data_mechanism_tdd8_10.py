"""Figure 5: degraded-channel Monte Carlo statistics and nonuniform beacons.
(a) 16-trial degraded-UWB Monte Carlo statistics (makespan & position RMSE),
(b) Nonuniform-beacon convex pentagon closed-loop coverage & trajectory inset.

Reads tdd8_multiseed_monte_carlo.json and tdd10_nonuniform_beacon_pentagon.json.
Outputs data_mechanism_and_multiseed.{pdf,png} in Times New Roman + STIX serif.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Polygon as MplPolygon
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from route_b_plot_style import (
    PALETTE,
    add_panel_label,
    apply_ieee_style,
    save_figure_pair,
)


def make_figure(sim_dir: Path, geo_dir: Path | None, out_dir: Path) -> tuple[Path, Path]:
    del geo_dir
    apply_ieee_style(base_font_size=7.0)

    t8 = json.loads((sim_dir / "tdd8_multiseed_monte_carlo.json").read_text(encoding="utf-8"))
    t10 = json.loads((sim_dir / "tdd10_nonuniform_beacon_pentagon.json").read_text(encoding="utf-8"))

    fig = plt.figure(figsize=(7.16, 2.55))
    # Explicit 3-column GridSpec so the central gutter cleanly separates (a)'s right
    # y-axis (Position RMSE) from (b)'s left y-axis (Certified coverage).
    gs = GridSpec(
        1,
        3,
        figure=fig,
        width_ratios=[1.02, 0.36, 1.06],
        wspace=0.0,
        left=0.075,
        right=0.988,
        top=0.87,
        bottom=0.25,
    )

    # ------------------------------------------------------------------
    # (a) 16-Trial Degraded-UWB Monte Carlo (48/48 safe, 0 violations)
    # ------------------------------------------------------------------
    ax_b = fig.add_subplot(gs[0, 0])
    sc_order = ["n6_converging_to_dual", "n6_dual_to_converging", "n14_dual_to_single_chain"]
    sc_labels = [
        "Converging\n$\\rightarrow$ Dual\n$N=6$",
        "Dual $\\rightarrow$\nConverging\n$N=6$",
        "Dual $\\rightarrow$\nSingle chain\n$N=14$",
    ]
    sc_map = {str(s["key"]): s for s in t8["scenarios"]}

    rng = np.random.default_rng(42)
    x_idx = np.arange(len(sc_order), dtype=float)
    ms_lists = [[float(r["makespan_s"]) for r in sc_map[k]["runs"]] for k in sc_order]
    rmse_lists = [[float(r["position_rmse_m"]) for r in sc_map[k]["runs"]] for k in sc_order]

    bp1 = ax_b.boxplot(
        ms_lists,
        positions=x_idx - 0.16,
        widths=0.22,
        patch_artist=True,
        showfliers=False,
        medianprops=dict(color=PALETTE["navy"], linewidth=1.2),
        boxprops=dict(facecolor="#d9e6ef", edgecolor=PALETTE["blue"], linewidth=0.75),
        whiskerprops=dict(color=PALETTE["blue"], linewidth=0.8),
        capprops=dict(color=PALETTE["blue"], linewidth=0.8),
    )
    for i, vals in enumerate(ms_lists):
        xj = (x_idx[i] - 0.16) + rng.uniform(-0.045, 0.045, size=len(vals))
        ax_b.scatter(xj, vals, s=8.5, facecolor=PALETTE["blue"], edgecolor="white", linewidths=0.3, alpha=0.88, zorder=5)

    ax_b.set_ylabel("Makespan (s)", color=PALETTE["blue"])
    ax_b.tick_params(axis="y", colors=PALETTE["blue"])
    ax_b.spines["left"].set_color(PALETTE["blue"])
    ax_b.set_ylim(470.0, 960.0)
    ax_b.set_yticks([500, 650, 800, 950])

    ax_b2 = ax_b.twinx()
    ax_b2.spines["top"].set_visible(False)
    ax_b2.spines["right"].set_visible(True)
    ax_b2.spines["right"].set_color(PALETTE["vermilion"])
    bp2 = ax_b2.boxplot(
        rmse_lists,
        positions=x_idx + 0.16,
        widths=0.22,
        patch_artist=True,
        showfliers=False,
        medianprops=dict(color="#7a4518", linewidth=1.2),
        boxprops=dict(facecolor="#f3e4d4", edgecolor=PALETTE["vermilion"], linewidth=0.75),
        whiskerprops=dict(color=PALETTE["vermilion"], linewidth=0.8),
        capprops=dict(color=PALETTE["vermilion"], linewidth=0.8),
    )
    for i, vals in enumerate(rmse_lists):
        xj = (x_idx[i] + 0.16) + rng.uniform(-0.045, 0.045, size=len(vals))
        ax_b2.scatter(xj, vals, s=8.5, facecolor=PALETTE["vermilion"], edgecolor="white", linewidths=0.3, alpha=0.88, zorder=5)

    ax_b2.set_ylabel("Position RMSE (m)", color=PALETTE["vermilion"])
    ax_b2.tick_params(axis="y", colors=PALETTE["vermilion"], labelsize=6.2)
    ax_b2.set_ylim(0.30, 0.44)
    ax_b2.set_yticks([0.30, 0.34, 0.38, 0.42])

    ax_b.set_xlim(-0.55, 2.55)
    ax_b.set_xticks(x_idx)
    ax_b.set_xticklabels(sc_labels, fontsize=5.3)

    leg_b = ax_b2.legend(
        [bp1["boxes"][0], bp2["boxes"][0]],
        ["Makespan, left", "Position RMSE, right"],
        loc="upper left",
        frameon=False,
        fontsize=5.2,
        handlelength=1.6,
        borderaxespad=0.15,
        labelspacing=0.25,
    )
    leg_b.set_zorder(10)
    ax_b2.text(2.48, 0.304, "48/48 trials safe, 0 violations", ha="right", va="bottom", fontsize=5.6, color=PALETTE["teal"])
    add_panel_label(ax_b, "(a)", "16-trial degraded-UWB Monte Carlo", fontsize=7.6)

    # ------------------------------------------------------------------
    # (b) Nonuniform Beacons (320, 400, 360 m) & Pentagon Domain
    # ------------------------------------------------------------------
    ax_c = fig.add_subplot(gs[0, 2])
    mis10 = t10["mission"]
    t_st10 = np.array([float(p["time_s"]) for p in mis10["stay"]["coverage_curve"]], dtype=float)
    c_st10 = np.array([float(p["certified_coverage"]) * 100.0 for p in mis10["stay"]["coverage_curve"]], dtype=float)
    t_sw10 = np.array([float(p["time_s"]) for p in mis10["switch"]["coverage_curve"]], dtype=float)
    c_sw10 = np.array([float(p["certified_coverage"]) * 100.0 for p in mis10["switch"]["coverage_curve"]], dtype=float)
    if t_sw10[-1] < 900.0:
        t_sw10 = np.append(t_sw10, 900.0)
        c_sw10 = np.append(c_sw10, c_sw10[-1])

    stay_fin_pct = float(mis10["stay"]["final_certified_coverage"]) * 100.0
    sw_fin_pct = float(mis10["switch"]["final_certified_coverage"]) * 100.0
    ax_c.plot(t_st10, c_st10, color=PALETTE["vermilion"], lw=1.45, ls="-", label=f"Stay two-chain: {stay_fin_pct:.1f}%")
    ax_c.plot(t_sw10, c_sw10, color=PALETTE["blue"], lw=1.55, ls="-", label=f"Switch single chain: {sw_fin_pct:.0f}%")
    t100_10 = float(mis10["switch"]["t100_certified_s"])
    ax_c.scatter([t100_10], [100.0], s=22, facecolor=PALETTE["blue"], edgecolor="white", linewidths=0.65, zorder=6)
    ax_c.annotate(
        f"$T_{{100}}={t100_10:.1f}$ s",
        (t100_10, 100.0),
        xytext=(10, -13),
        textcoords="offset points",
        ha="left",
        fontsize=5.8,
        fontweight="bold",
        color=PALETTE["blue"],
        arrowprops=dict(arrowstyle="-", color=PALETTE["blue"], lw=0.55),
    )

    ax_c.set_xlim(0.0, 900.0)
    ax_c.set_ylim(48.0, 104.0)
    ax_c.set_xticks([0, 300, 600, 900])
    ax_c.set_yticks([50, 65, 80, 95, 100])
    ax_c.set_xlabel("Time $t$ (s)")
    ax_c.set_ylabel("Certified coverage (%)")
    # Place frameless legend neatly in upper-left open pocket under the 98.7% curve,
    # keeping lower-right completely dedicated to the pentagon trajectory inset.
    ax_c.legend(
        loc="upper left",
        bbox_to_anchor=(0.01, 0.87),
        frameon=False,
        fontsize=5.1,
        handlelength=1.7,
        borderaxespad=0.15,
        labelspacing=0.22,
    )

    ax_ins = inset_axes(ax_c, width="62%", height="58%", loc="lower right", borderpad=0.35)
    ax_ins.set_facecolor("none")
    verts = np.array(t10["vertices_m"], dtype=float)
    poly_p = MplPolygon(verts, closed=True, facecolor="#e2e8f0", edgecolor=PALETTE["navy"], linestyle="--", linewidth=0.55, alpha=0.72, zorder=1)
    ax_ins.add_patch(poly_p)
    bxs = [float(x) for x in t10["beacon_xs_m"]]
    ax_ins.axhline(0.0, color=PALETTE["navy"], lw=0.75, zorder=2)
    ax_ins.scatter(bxs, [0.0] * len(bxs), marker="s", s=11, facecolor=PALETTE["navy"], edgecolor="white", linewidths=0.45, zorder=6)
    for bi, bx_val in enumerate(bxs, start=1):
        ax_ins.text(bx_val, -75.0, f"$b_{bi}$", fontsize=4.8, ha="center", va="top", color=PALETTE["navy"])

    traj = mis10["switch"]["trajectory_samples"]
    node_ids = sorted(traj[0]["positions"].keys(), key=int)
    for nid in node_ids:
        tx = [float(fr["positions"][nid][0]) for fr in traj]
        ty = [float(fr["positions"][nid][1]) for fr in traj]
        ax_ins.plot(tx, ty, color=PALETTE["blue"], lw=0.65, alpha=0.75, zorder=3)
        ax_ins.scatter([tx[-1]], [ty[-1]], s=9.0, facecolor=PALETTE["blue"], edgecolor="white", linewidths=0.35, zorder=5)

    ax_ins.set_xlim(-60.0, 1200.0)
    ax_ins.set_ylim(-230.0, 1080.0)
    ax_ins.set_xticks([])
    ax_ins.set_yticks([])
    ax_ins.set_title("(320, 400, 360) m", fontsize=5.0, pad=1.5, color=PALETTE["navy"])
    for sp in ("top", "right", "bottom", "left"):
        ax_ins.spines[sp].set_visible(False)
    ax_ins.set_aspect("equal")

    add_panel_label(ax_c, "(b)", "Nonuniform beacons, pentagon domain", fontsize=7.6)

    return save_figure_pair(fig, out_dir, "data_mechanism_and_multiseed")


def main() -> None:
    parser = argparse.ArgumentParser()
    root = Path(__file__).resolve().parents[2]
    parser.add_argument("--sim-dir", type=Path, default=root / "outputs" / "sim")
    parser.add_argument("--geo-dir", type=Path, default=root / "data" / "geo_outputs")
    parser.add_argument("--out-dir", type=Path, default=root / "outputs" / "figures")
    args = parser.parse_args()
    pdf_p, png_p = make_figure(args.sim_dir, args.geo_dir, args.out_dir)
    print(f"Wrote {pdf_p} and {png_p}")


if __name__ == "__main__":
    main()
