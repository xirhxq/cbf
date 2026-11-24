"""Figure 6: 2x3 closed-loop spatio-temporal mission snapshots for the
9.65 km^2 broad coastal convex domain (N=10, M=6, 1-chain -> 2-chain, 10 m grid).

Reads tdd7_mission_snapshots.json and outputs data_mission_snapshots_tdd7.{pdf,png}.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Polygon as MplPolygon
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

BLUE = PALETTE["blue"]       # #1f628e searching UAVs
SLATE = PALETTE["navy"]      # #334155 boundary & beacons
EDGE_COL = PALETTE["slate"]  # #64748b active reference links
CERT_RGBA = (219 / 255.0, 229 / 255.0, 237 / 255.0, 1.0)  # #dbe5ed certified shading


def decode_rle_grid(rle: list[list[int]], x_num: int, y_num: int) -> np.ndarray:
    """Decode linear_index = ix * y_num + iy runs into a (y_num, x_num) boolean mask."""
    flat = np.zeros(x_num * y_num, dtype=bool)
    for start, length in rle:
        flat[int(start) : int(start) + int(length)] = True
    return flat.reshape((x_num, y_num)).T


def pick_snapshot(snaps: list[dict], target_t: float) -> dict:
    return min(snaps, key=lambda s: abs(float(s["time_s"]) - target_t))


def make_figure(sim_dir: Path, out_dir: Path) -> tuple[Path, Path]:
    apply_ieee_style(base_font_size=7.0)
    raw = json.loads((sim_dir / "tdd7_mission_snapshots.json").read_text(encoding="utf-8"))

    verts_km = np.asarray(raw["vertices_m"], dtype=float) / 1000.0
    x_lim_km = [float(v) / 1000.0 for v in raw["x_lim_m"]]
    y_lim_km = [float(v) / 1000.0 for v in raw["y_lim_m"]]
    x_num = int(raw["x_num"])
    y_num = int(raw["y_num"])
    snaps = raw["snapshots"]

    panels = [
        ("(a)", "Initial search", 10.0),
        ("(b)", "Pipelined transfer", 380.0),
        ("(c)", "Dual chains seated", 603.8),
        ("(d)", "Parallel sweep", 950.0),
        ("(e)", "Near-complete", 1531.4),
        ("(f)", "Search complete", 1700.3),
    ]

    fig = plt.figure(figsize=(7.2, 4.35))
    gs = GridSpec(
        2,
        3,
        figure=fig,
        hspace=0.34,
        wspace=0.10,
        left=0.068,
        right=0.988,
        top=0.925,
        bottom=0.145,
    )

    for idx, (tag, stage_lbl, t_req) in enumerate(panels):
        row, col = divmod(idx, 3)
        ax = fig.add_subplot(gs[row, col])
        snap = pick_snapshot(snaps, t_req)
        t_val = float(snap["time_s"])
        cov_pct = float(snap["certified_coverage"]) * 100.0

        # Decode certified grid and clip cleanly to the convex domain polygon
        mask = decode_rle_grid(snap["certified_rle"], x_num, y_num)
        rgba = np.zeros((y_num, x_num, 4), dtype=float)
        rgba[mask] = CERT_RGBA

        clip_poly = MplPolygon(verts_km, closed=True, facecolor="none", edgecolor="none", transform=ax.transData)
        ax.add_patch(clip_poly)
        im = ax.imshow(
            rgba,
            origin="lower",
            extent=[x_lim_km[0], x_lim_km[1], y_lim_km[0], y_lim_km[1]],
            interpolation="nearest",
            aspect="auto",
            zorder=1,
        )
        im.set_clip_path(clip_poly)

        # Convex domain outline and shoreline baseline
        ax.add_patch(
            MplPolygon(
                verts_km,
                closed=True,
                facecolor="none",
                edgecolor=SLATE,
                linewidth=0.85,
                zorder=3,
            )
        )
        ax.plot([-2.15, 4.15], [0.0, 0.0], color=PALETTE["gray_mid"], lw=0.55, ls="-", zorder=2)

        # Active topology reference links
        pos_km = {str(k): (float(v[0]) / 1000.0, float(v[1]) / 1000.0) for k, v in snap["positions"].items()}
        bcn_km = {str(k): (float(v[0]) / 1000.0, float(v[1]) / 1000.0) for k, v in snap["beacons"].items()}
        all_nodes = {**bcn_km, **pos_km}

        for edge in snap["active_edges"]:
            u_id, v_id = str(edge[0]), str(edge[1])
            if u_id in all_nodes and v_id in all_nodes:
                p0, p1 = all_nodes[u_id], all_nodes[v_id]
                ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color=EDGE_COL, lw=0.60, alpha=0.85, zorder=4)

        # Coastal beacon UAVs (squares along y=0)
        bxs = [p[0] for p in bcn_km.values()]
        bys = [p[1] for p in bcn_km.values()]
        ax.scatter(
            bxs,
            bys,
            marker="s",
            s=11.0,
            facecolor="#e8dcc8",
            edgecolor=SLATE,
            linewidths=0.55,
            zorder=5,
        )

        # Searching UAVs (steel-blue circles)
        uxs = [p[0] for p in pos_km.values()]
        uys = [p[1] for p in pos_km.values()]
        ax.scatter(
            uxs,
            uys,
            marker="o",
            s=15.0,
            facecolor=BLUE,
            edgecolor="white",
            linewidths=0.45,
            zorder=6,
        )

        # Compact coverage readout in guaranteed clear upper-left sky outside trapezoid
        ax.text(
            0.03,
            0.95,
            f"Certified: {cov_pct:.1f}%",
            transform=ax.transAxes,
            ha="left",
            va="top",
            fontsize=6.2,
            color=SLATE,
            zorder=7,
        )

        ax.set_xlim(-2.25, 4.25)
        ax.set_ylim(-0.22, 2.38)
        ax.set_xticks([-2, 0, 2, 4])
        ax.set_yticks([0.0, 1.0, 2.0])

        if row == 1:
            ax.set_xlabel("$x$ (km)")
        else:
            ax.set_xticklabels([])
        if col == 0:
            ax.set_ylabel("$y$ (km)")
        else:
            ax.set_yticklabels([])

        add_panel_label(ax, tag, f"{stage_lbl}, {t_val:.1f} s", x=0.0, y=1.04, fontsize=7.4)

    # Single-line frameless bottom caption-legend strip
    fig.text(
        0.528,
        0.028,
        "Shading: certified cells     Circles: searching UAVs     Squares: beacon UAVs     Lines: active references",
        ha="center",
        va="bottom",
        fontsize=6.6,
        color=SLATE,
    )

    return save_figure_pair(fig, out_dir, "data_mission_snapshots_tdd7")


def main() -> None:
    parser = argparse.ArgumentParser()
    root = Path(__file__).resolve().parents[2]
    parser.add_argument("--sim-dir", type=Path, default=root / "outputs" / "sim")
    parser.add_argument("--out-dir", type=Path, default=root / "outputs" / "figures")
    args = parser.parse_args()
    pdf_p, png_p = make_figure(args.sim_dir, args.out_dir)
    print(f"Wrote {pdf_p} and {png_p}")


if __name__ == "__main__":
    main()
