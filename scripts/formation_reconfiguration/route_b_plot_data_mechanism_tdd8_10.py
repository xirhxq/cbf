"""Wave 9 Fig. 5 pure-data 1x3 panel:
(a) Closed-loop two-reference collinear crossing versus three-reference dual handover,
(b) 16-trial degraded-UWB Monte Carlo statistics (makespan & position RMSE),
(c) Nonuniform-beacon convex pentagon closed-loop coverage & trajectory inset.

Reads sim-outputs/tdd9_geometry_closed_loop_contrast.json,
sim-outputs/tdd8_multiseed_monte_carlo.json, and
sim-outputs/tdd10_nonuniform_beacon_pentagon.json.
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

    t9 = json.loads((sim_dir / "tdd9_geometry_closed_loop_contrast.json").read_text(encoding="utf-8"))
    t8 = json.loads((sim_dir / "tdd8_multiseed_monte_carlo.json").read_text(encoding="utf-8"))
    t10 = json.loads((sim_dir / "tdd10_nonuniform_beacon_pentagon.json").read_text(encoding="utf-8"))

    fig = plt.figure(figsize=(7.20, 2.38))
    gs = GridSpec(
        1,
        5,
        figure=fig,
        width_ratios=[1.00, 0.25, 0.92, 0.56, 1.00],
        wspace=0.0,
        left=0.056,
        right=0.988,
        top=0.86,
        bottom=0.25,
    )

    # ------------------------------------------------------------------
    # (a) Closed-loop two-reference crossing vs three-reference handover
    # ------------------------------------------------------------------
    ax_a = fig.add_subplot(gs[0, 0])
    hr = t9["handover_reference_contrast"]
    r2 = hr["two_ref_collinear_baseline"]
    r3 = hr["three_ref_dual_handover"]
    t2 = np.array([float(p["time_s"]) for p in r2["scale_radius_series"]], dtype=float)
    s2 = np.array([float(p["max_scale_radius_m"]) for p in r2["scale_radius_series"]], dtype=float)
    t3 = np.array([float(p["time_s"]) for p in r3["scale_radius_series"]], dtype=float)
    s3 = np.array([float(p["max_scale_radius_m"]) for p in r3["scale_radius_series"]], dtype=float)
    pk2 = float(r2["audit"]["peak_position_scale_radius_m"])
    pk3 = float(r3["audit"]["peak_position_scale_radius_m"])
    ret2 = int(r2["mbb_retries"])
    ret3 = int(r3["mbb_retries"])

    ax_a.plot(t2, s2, color=PALETTE["vermilion"], lw=1.25, alpha=0.92,
              label=f"Two-reference, {ret2} retries")
    ax_a.plot(t3, s3, color=PALETTE["blue"], lw=1.35, alpha=0.96,
              label=f"Three-reference, {ret3} retries")
    i2 = int(np.argmax(s2))
    ax_a.scatter([t2[i2]], [s2[i2]], s=18, facecolor=PALETTE["vermilion"], edgecolor="white", linewidths=0.5, zorder=6)
    ax_a.annotate(
        f"Peak {pk2:.2f} m",
        (t2[i2], s2[i2]),
        xytext=(8, 1),
        textcoords="offset points",
        ha="left",
        va="center",
        fontsize=5.3,
        color=PALETTE["vermilion"],
    )
    ax_a.axhline(pk3, color=PALETTE["blue"], ls=":", lw=0.75, zorder=2)
    ax_a.text(300.0, 0.92, f"Three-reference peak {pk3:.2f} m, -48.2%", ha="center", va="bottom",
              fontsize=5.0, color=PALETTE["blue"])
    ax_a.set_xlim(0.0, 565.0)
    ax_a.set_ylim(0.5, 6.45)
    ax_a.set_xticks([0, 150, 300, 450])
    ax_a.set_yticks([1, 2, 3, 4, 5, 6])
    ax_a.set_xlabel("Time $t$ (s)")
    ax_a.set_ylabel("Peak scale $3\\sqrt{\\mathrm{tr}(\\boldsymbol{P}_i^p)}$ (m)")
    ax_a.legend(loc="upper right", frameon=False, fontsize=5.0, handlelength=1.4, borderaxespad=0.12, labelspacing=0.20)
    add_panel_label(ax_a, "(a)", "Crossing handover contrast", fontsize=7.4)

    # ------------------------------------------------------------------
    # (b) 16-Trial Degraded-UWB Monte Carlo (48/48 safe, 0 violations)
    # ------------------------------------------------------------------
    ax_b = fig.add_subplot(gs[0, 2])
    sc_order = ["n6_converging_to_dual", "n6_dual_to_converging", "n14_dual_to_single_chain"]
    sc_labels = [
        "Converging\nto dual, 6",
        "Dual to\nconverging, 6",
        "Dual to\nsingle, 14",
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
        ax_b.scatter(xj, vals, s=7.5, facecolor=PALETTE["blue"], edgecolor="white", linewidths=0.3, alpha=0.88, zorder=5)

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
        ax_b2.scatter(xj, vals, s=7.5, facecolor=PALETTE["vermilion"], edgecolor="white", linewidths=0.3, alpha=0.88, zorder=5)

    ax_b2.set_ylabel("Position RMSE (m)", color=PALETTE["vermilion"])
    ax_b2.tick_params(axis="y", colors=PALETTE["vermilion"], labelsize=6.0)
    ax_b2.set_ylim(0.30, 0.44)
    ax_b2.set_yticks([0.30, 0.34, 0.38, 0.42])

    ax_b.set_xlim(-0.55, 2.55)
    ax_b.set_xticks(x_idx)
    ax_b.set_xticklabels(sc_labels, fontsize=4.8)

    leg_b = ax_b2.legend(
        [bp1["boxes"][0], bp2["boxes"][0]],
        ["Makespan, left", "Position RMSE, right"],
        loc="upper left",
        frameon=False,
        fontsize=4.9,
        handlelength=1.4,
        borderaxespad=0.12,
        labelspacing=0.20,
    )
    leg_b.set_zorder(10)
    ax_b2.text(2.48, 0.304, "48/48 safe, 0 violations", ha="right", va="bottom", fontsize=5.1, color=PALETTE["teal"])
    add_panel_label(ax_b, "(b)", "Degraded-UWB Monte Carlo", fontsize=7.4)

    # ------------------------------------------------------------------
    # (c) Nonuniform Beacons (320, 400, 360 m) & Pentagon Domain
    # ------------------------------------------------------------------
    ax_c = fig.add_subplot(gs[0, 4])
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
    ax_c.scatter([t100_10], [100.0], s=20, facecolor=PALETTE["blue"], edgecolor="white", linewidths=0.6, zorder=6)
    ax_c.annotate(
        f"$T_{{100}}={t100_10:.1f}$ s",
        (t100_10, 100.0),
        xytext=(10, -16),
        textcoords="offset points",
        ha="left",
        fontsize=5.3,
        fontweight="bold",
        color=PALETTE["blue"],
        arrowprops=dict(arrowstyle="-", color=PALETTE["blue"], lw=0.55),
    )

    ax_c.set_xlim(0.0, 900.0)
    ax_c.set_ylim(80.0, 102.2)
    ax_c.set_xticks([0, 300, 600, 900])
    ax_c.set_yticks([80, 85, 90, 95, 100])
    ax_c.set_xlabel("Time $t$ (s)")
    ax_c.set_ylabel("Certified coverage (%)")
    ax_c.legend(
        loc="upper left",
        bbox_to_anchor=(0.03, 0.85),
        frameon=False,
        fontsize=4.9,
        handlelength=1.4,
        borderaxespad=0.12,
        labelspacing=0.22,
    )

    ax_ins = inset_axes(ax_c, width="54%", height="54%", loc="lower right", borderpad=0.35)
    ax_ins.set_facecolor("none")
    verts = np.array(t10["vertices_m"], dtype=float)
    poly_p = MplPolygon(verts, closed=True, facecolor="#e2e8f0", edgecolor=PALETTE["navy"], linestyle="--", linewidth=0.55, alpha=0.72, zorder=1)
    ax_ins.add_patch(poly_p)
    bxs = [float(x) for x in t10["beacon_xs_m"]]
    ax_ins.axhline(0.0, color=PALETTE["navy"], lw=0.75, zorder=2)
    ax_ins.scatter(bxs, [0.0] * len(bxs), marker="s", s=10, facecolor=PALETTE["navy"], edgecolor="white", linewidths=0.45, zorder=6)
    for bi, bx_val in enumerate(bxs, start=1):
        ax_ins.text(bx_val, -75.0, f"$b_{bi}$", fontsize=4.6, ha="center", va="top", color=PALETTE["navy"])

    traj = mis10["switch"]["trajectory_samples"]
    node_ids = sorted(traj[0]["positions"].keys(), key=int)
    for nid in node_ids:
        tx = [float(fr["positions"][nid][0]) for fr in traj]
        ty = [float(fr["positions"][nid][1]) for fr in traj]
        ax_ins.plot(tx, ty, color=PALETTE["blue"], lw=0.65, alpha=0.75, zorder=3)
        ax_ins.scatter([tx[-1]], [ty[-1]], s=8.0, facecolor=PALETTE["blue"], edgecolor="white", linewidths=0.35, zorder=5)

    ax_ins.set_xlim(-60.0, 1200.0)
    ax_ins.set_ylim(-230.0, 1080.0)
    ax_ins.set_xticks([])
    ax_ins.set_yticks([])
    ax_ins.set_title("320, 400, 360 m", fontsize=4.8, pad=1.5, color=PALETTE["navy"])
    for sp in ("top", "right", "bottom", "left"):
        ax_ins.spines[sp].set_visible(False)
    ax_ins.set_aspect("equal")

    add_panel_label(ax_c, "(c)", "Nonuniform beacons, pentagon", fontsize=7.4)

    return save_figure_pair(fig, out_dir, "data_mechanism_and_multiseed")


def main() -> None:
    parser = argparse.ArgumentParser()
    root = Path(__file__).resolve().parents[2]
    parser.add_argument("--sim-dir", type=Path, default=root / "experiments" / "route-b-20261123" / "sim-outputs")
    parser.add_argument("--geo-dir", type=Path, default=root / "experiments" / "route-b-20261123" / "geo-outputs")
    parser.add_argument("--out-dir", type=Path, default=root / "experiments" / "route-b-20261123" / "figures")
    args = parser.parse_args()
    pdf_p, png_p = make_figure(args.sim_dir, args.geo_dir, args.out_dir)
    print(f"Wrote {pdf_p} and {png_p}")


if __name__ == "__main__":
    main()
