"""Figure 4: closed-loop reconfiguration timing and scaling.

(a) N=14 closed-loop flight windows, (b) N=20 dual-spine flight windows,
(c) closed-loop pipelined versus serial makespan against the
kinematic Theta(N) / Theta(N^2) reference curves, (d) break-even area A_crit.

Reads tdd5_pinball_closed_loop.json, tdd5_multiscale_closed_loop.json,
and multiscale_pipeline_timings.json. Outputs data_closed_loop_reconfig.{pdf,png}.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from route_b_plot_style import PALETTE, add_panel_label, apply_ieee_style, save_figure_pair

BLUE = PALETTE["blue"]
OCHRE = PALETTE["vermilion"]
OCHRE_L = PALETTE["amber"]
MOSS = PALETTE["teal"]
SLATE = PALETTE["navy"]
GRAY = PALETTE["slate"]
SKY = PALETTE["sky"]
CRIMSON = PALETTE["crimson"]


def load_tdd5_data(sim_dir: Path) -> tuple[dict, dict]:
    p_path = sim_dir / "tdd5_pinball_closed_loop.json"
    m_path = sim_dir / "tdd5_multiscale_closed_loop.json"
    if not p_path.exists() or not m_path.exists():
        raise FileNotFoundError(f"Expected {p_path} and {m_path}")
    return (
        json.loads(p_path.read_text(encoding="utf-8")),
        json.loads(m_path.read_text(encoding="utf-8")),
    )


def load_kinematic_reference(geo_dir: Path) -> dict[str, dict[str, list[float]]]:
    """Kinematic (geometry-layer) serial / pipelined makespan and A_crit per cruise speed."""
    raw = json.loads((geo_dir / "multiscale_pipeline_timings.json").read_text(encoding="utf-8"))
    rows = raw["multiscale"]
    out: dict[str, dict[str, list[float]]] = {}
    for v_cruise, tag in ((4.0, "v4"), (10.0, "v10")):
        ser_t, pip_t, pip_d_t, ser_a, pip_a, pip_d_a = [], [], [], [], [], []
        for n_uav in (6, 14, 20):
            row = next(r for r in rows if int(r["scale_N"]) == n_uav and abs(float(r["v_cruise_mps"]) - v_cruise) < 1e-6)
            ser_t.append(float(row["serial_reversible_berth"]["makespan_s"]))
            pip_t.append(float(row["pipelined_deterministic"]["makespan_s"]))
            pip_d_t.append(float(row["pipelined_with_delays"]["makespan_s"]))
            ser_a.append(float(row["serial_reversible_berth"]["a_crit_km2"]))
            pip_a.append(float(row["pipelined_deterministic"]["a_crit_km2"]))
            pip_d_a.append(float(row["pipelined_with_delays"]["a_crit_km2"]))
        out[tag] = {
            "serial_s": ser_t, "pipelined_s": pip_t, "pipelined_delayed_s": pip_d_t,
            "serial_acrit": ser_a, "pipelined_acrit": pip_a, "pipelined_delayed_acrit": pip_d_a,
        }
    return out


def make_figure(sim_dir: Path, geo_dir: Path, out_dir: Path) -> tuple[Path, Path]:
    apply_ieee_style(base_font_size=7.0)
    _pin_raw, multi_raw = load_tdd5_data(sim_dir)
    kin = load_kinematic_reference(geo_dir)
    scales_by_key = {(int(r["scale_N"]), str(r["channel_group"])): r for r in multi_raw["scales"]}

    fig = plt.figure(figsize=(7.2, 3.85))
    gs = GridSpec(2, 2, figure=fig, height_ratios=[1.0, 1.0], hspace=0.62, wspace=0.26,
                  left=0.065, right=0.985, top=0.93, bottom=0.11)

    # ------------------------------------------------------------------
    # (a), (b) closed-loop flight windows
    # ------------------------------------------------------------------
    for idx_p, (n_uav, tag, title_str, order_nodes, wing_split, tick_nodes) in enumerate(
        [
            (14, "(a)", "Closed-loop flight windows, $N=14$",
             [7, 6, 5, 4, 3, 2, 1, 14, 13, 12, 11, 10, 9, 8], 6.5, [7, 4, 1, 13, 10, 8]),
            (20, "(b)", "Dual-spine flight windows, $N=20$",
             [10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11], 9.5, [10, 6, 1, 19, 15, 11]),
        ]
    ):
        ax = fig.add_subplot(gs[0, idx_p])
        row_nom = scales_by_key[(n_uav, "nominal")]["pipelined"]
        row_deg = scales_by_key[(n_uav, "degraded")]["pipelined"]
        g_nom = {int(item["node"]): item for item in row_nom["gantt"]}
        g_deg = {int(item["node"]): item for item in row_deg["gantt"]}
        y_map = {nid: len(order_nodes) - 1 - i for i, nid in enumerate(order_nodes)}
        for nid in order_nodes:
            yp = y_map[nid]
            col = BLUE if yp > wing_split else MOSS
            ax.barh(yp, float(g_nom[nid]["duration_s"]), left=float(g_nom[nid]["start_s"]), height=0.62,
                    color=col, alpha=0.9, edgecolor="none", zorder=3)
            ax.plot([float(g_deg[nid]["finish_s"])], [yp], marker="|", color=OCHRE,
                    markersize=4.0 if n_uav == 14 else 3.2, markeredgewidth=0.9, zorder=5)
        ax.axhline(wing_split, color=GRAY, ls=":", lw=0.6, zorder=2)
        ms_nom = float(row_nom["makespan_s"])
        ms_deg = float(row_deg["makespan_s"])
        ax.axvline(ms_nom, color=OCHRE, ls="--", lw=0.9, zorder=4)
        ax.text(ms_nom + 12.0, len(order_nodes) - 1.0, f"{ms_nom:.1f} s", ha="left", va="top",
                fontsize=6.4, color=OCHRE)
        ax.text(ms_nom + 12.0, len(order_nodes) - 1.0 - 0.17 * len(order_nodes), f"Degraded {ms_deg:.1f} s",
                ha="left", va="top", fontsize=5.8, color=OCHRE)
        ax.set_yticks([y_map[n] for n in tick_nodes])
        ax.set_yticklabels([f"UAV {n}" for n in tick_nodes], fontsize=5.8)
        ax.tick_params(axis="y", length=2.0)
        ax.set_xlim(0.0, 1130.0)
        ax.set_ylim(-0.8, len(order_nodes) - 0.2)
        ax.set_xticks([0, 200, 400, 600, 800])
        ax.set_xlabel("Closed-loop time (s)")
        add_panel_label(ax, tag, title_str, fontsize=7.6)

    # ------------------------------------------------------------------
    # (c) makespan scaling: closed loop versus kinematic reference
    # ------------------------------------------------------------------
    ax_c = fig.add_subplot(gs[1, 0])
    ns = np.array([6, 14, 20], dtype=float)
    k10 = kin["v10"]
    ax_c.plot(ns, np.array(k10["serial_s"]) / 1e3, color=OCHRE_L, ls=(0, (4, 2)), lw=1.0, zorder=2)
    ax_c.plot(ns, np.array(k10["pipelined_s"]) / 1e3, color=SKY, ls=(0, (4, 2)), lw=1.0, zorder=2)

    pip_nom = np.array([float(scales_by_key[(int(n), "nominal")]["pipelined"]["makespan_s"]) for n in ns])
    pip_deg = np.array([float(scales_by_key[(int(n), "degraded")]["pipelined"]["makespan_s"]) for n in ns])
    ser_rows = [scales_by_key[(int(n), "nominal")]["serial"] for n in ns]
    ser_val = np.array([float(r["makespan_s"]) for r in ser_rows])
    ser_done = [bool(r["completed"]) for r in ser_rows]

    ax_c.plot(ns, pip_nom / 1e3, color=BLUE, lw=1.5, marker="o", markersize=3.6, zorder=5)
    ax_c.plot(ns, pip_deg / 1e3, color=BLUE, lw=0.0, marker="o", markersize=5.2, markerfacecolor="none",
              markeredgewidth=0.8, zorder=4)
    for x, v, done in zip(ns, ser_val, ser_done):
        if done:
            ax_c.plot([x], [v / 1e3], marker="s", markersize=3.8, color=OCHRE, zorder=5)
        else:
            ax_c.plot([x], [v / 1e3], marker="s", markersize=4.2, markerfacecolor="white",
                      markeredgecolor=OCHRE, markeredgewidth=0.9, zorder=5)
            ax_c.annotate("", xy=(x, v / 1e3 + 0.55), xytext=(x, v / 1e3 + 0.12),
                          arrowprops=dict(arrowstyle="-|>", color=OCHRE, lw=0.8, mutation_scale=6))
            ax_c.text(x + 0.45, v / 1e3 - 0.38, f"{v:.0f} s limit", fontsize=5.8, color=OCHRE, ha="left", va="center")
    ax_c.plot([6, 14, 20], ser_val / 1e3, color=OCHRE, lw=0.9, ls="-", alpha=0.6, zorder=3)

    # Speedup labels; a timed-out serial run only gives a lower bound, so floor to one decimal.
    speedups = []
    for sv, done, pv in zip(ser_val, ser_done, pip_nom):
        ratio = sv / pv
        speedups.append(f"{ratio:.2f}$\\times$" if done else f"$>${math.floor(ratio * 10.0) / 10.0:.1f}$\\times$")
    for x, txt, pv in zip(ns, speedups, pip_nom):
        if x < 10.0:  # N=6: completed serial marker sits just above, so label to the right
            ax_c.text(x + 0.6, pv / 1e3 + 0.38, txt, fontsize=6.0, color=BLUE, ha="left", va="center")
        else:
            ax_c.text(x, pv / 1e3 + 0.30, txt, fontsize=6.0, color=BLUE, ha="center", va="bottom")

    handles_c = [
        Line2D([0], [0], color=BLUE, lw=1.5, marker="o", markersize=3.6, label="Closed-loop pipelined"),
        Line2D([0], [0], color=BLUE, lw=0, marker="o", markersize=5.2, markerfacecolor="none", label="Degraded channel"),
        Line2D([0], [0], color=OCHRE, lw=0.9, marker="s", markersize=3.8, label="Closed-loop serial"),
        Line2D([0], [0], color=SKY, lw=1.0, ls=(0, (4, 2)), label="Kinematic $\\Theta(N)$, 10 m/s"),
        Line2D([0], [0], color=OCHRE_L, lw=1.0, ls=(0, (4, 2)), label="Kinematic $\\Theta(N^2)$, 10 m/s"),
    ]
    ax_c.legend(handles=handles_c, loc="upper left", fontsize=5.7, handlelength=2.2, borderaxespad=0.1,
                labelspacing=0.3)
    ax_c.set_xlim(4.5, 25.0)
    ax_c.set_ylim(-0.25, 5.2)
    ax_c.set_xticks([6, 14, 20])
    ax_c.set_xlabel("Squad size $N$")
    ax_c.set_ylabel("Makespan ($10^3$ s)")
    add_panel_label(ax_c, "(c)", "Makespan scaling", fontsize=7.6)

    # ------------------------------------------------------------------
    # (d) break-even area A_crit
    # ------------------------------------------------------------------
    ax_d = fig.add_subplot(gs[1, 1])
    ax_d.axhspan(5.0, 25.0, color=MOSS, alpha=0.08, lw=0, zorder=1)
    ax_d.text(23.3, 15.0, "5–25 km$^2$", fontsize=5.9, color=MOSS, ha="right", va="center")
    styles = [
        ("v4", "serial_acrit", None, OCHRE, (0, (4, 2)), "s", "Serial, 4 m/s"),
        ("v10", "serial_acrit", None, OCHRE_L, (0, (4, 2)), "s", "Serial, 10 m/s"),
        ("v4", "pipelined_acrit", "pipelined_delayed_acrit", BLUE, "-", "o", "Pipelined, 4 m/s"),
        ("v10", "pipelined_acrit", "pipelined_delayed_acrit", SKY, "-", "o", "Pipelined, 10 m/s"),
    ]
    for tag_v, key, key_d, col, ls, mk, lbl in styles:
        y = np.array(kin[tag_v][key])
        if key_d:
            ax_d.fill_between(ns, y, np.array(kin[tag_v][key_d]), color=col, alpha=0.18, lw=0, zorder=2)
        ax_d.plot(ns, y, color=col, ls=ls, lw=1.3, marker=mk, markersize=3.2, label=lbl, zorder=4)
    ax_d.set_xlim(4.5, 23.5)
    ax_d.set_ylim(0.0, 170.0)
    ax_d.set_xticks([6, 14, 20])
    ax_d.set_xlabel("Squad size $N$")
    ax_d.set_ylabel("$A_{\\mathrm{crit}}$ (km$^2$)")
    ax_d.legend(loc="upper left", fontsize=5.7, handlelength=2.2, borderaxespad=0.1, labelspacing=0.3)
    add_panel_label(ax_d, "(d)", "Break-even area, $v_{\\mathrm{axis}}=25$ m/s", fontsize=7.6)

    return save_figure_pair(fig, out_dir, "data_closed_loop_reconfig")


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
