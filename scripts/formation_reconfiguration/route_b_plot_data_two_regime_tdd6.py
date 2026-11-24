"""Figure 7: two-regime closed-loop convex-domain coverage comparison.

Combines tdd6_two_regime_missions.json (Regimes 1, 2a, 2b) and
tdd7_regime2_positive_breakeven.json (Regime 2c). Each panel carries a
borderless inset that sketches the convex domain, the shore beacons, the Stay
formation (hollow ochre) and the Switch formation (solid steel blue). Formation
geometry comes from convex_domains.json, matching the closed-loop
runner domain specifications, and from the staggered-chain rule for Regime 2c.
Outputs data_two_regime_coverage.{pdf,png}.
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

from route_b_plot_style import PALETTE, add_panel_label, apply_ieee_style, save_figure_pair

BLUE = PALETTE["blue"]
OCHRE = PALETTE["vermilion"]
OCHRE_L = PALETTE["amber"]
MOSS = PALETTE["teal"]
SLATE = PALETTE["navy"]
GRAY = PALETTE["slate"]
SKY = PALETTE["sky"]

Formation = tuple[dict[str, tuple[float, float]], list[tuple[tuple[float, float], tuple[float, float]]]]


def load_tdd6_missions(sim_dir: Path) -> dict[tuple[str, str], dict]:
    raw = json.loads((sim_dir / "tdd6_two_regime_missions.json").read_text(encoding="utf-8"))
    return {(str(m["domain_name"]), str(m["channel_group"])): m for m in raw["missions"]}


def load_tdd7(sim_dir: Path) -> dict:
    return json.loads((sim_dir / "tdd7_regime2_positive_breakeven.json").read_text(encoding="utf-8"))


def _curve_xy(arm: dict, extend_to: float | None = None) -> tuple[np.ndarray, np.ndarray]:
    pts = arm["coverage_curve"]
    ts = [float(p["time_s"]) for p in pts]
    cs = [float(p["certified_coverage"]) * 100.0 for p in pts]
    if extend_to is not None and ts[-1] < extend_to:
        ts.append(extend_to)
        cs.append(cs[-1])
    return np.array(ts, dtype=float), np.array(cs, dtype=float)


def formation_from_geo(spec: dict) -> Formation:
    beacons = {str(k): (float(v[0]), float(v[1])) for k, v in spec["beacons"].items()}
    pos = {str(k): (float(v[0]), float(v[1])) for k, v in spec["positions"].items()}
    allp = {**beacons, **pos}
    edges = []
    for nid, par in spec["parents"].items():
        for p in par:
            edges.append((pos[str(nid)], allp[str(p)]))
    return pos, edges


def staggered_formation(b_xs: list[float], units: list[list[int]], h1: float, dy: float) -> Formation:
    """Staggered triangular chain rule of the closed-loop runner (x alternates mid/right of the interval)."""
    pos: dict[str, tuple[float, float]] = {}
    edges = []
    for r, unit in enumerate(units):
        span = b_xs[r + 1] - b_xs[r]
        chain: list[tuple[float, float]] = [(b_xs[r], 0.0), (b_xs[r + 1], 0.0)]
        for j, nid in enumerate(unit, start=1):
            x = b_xs[r] + (0.5 * span if j % 2 == 1 else span)
            y = h1 + (j - 1) * dy
            pos[str(nid)] = (x, y)
            edges.append(((x, y), chain[j - 1]))
            edges.append(((x, y), chain[j]))
            chain.append((x, y))
    return pos, edges


def draw_inset(ax: plt.Axes, bounds: list[float], verts: list[list[float]], b_xs: list[float],
               stay: Formation, switch: Formation, anchor: str) -> None:
    ins = ax.inset_axes(bounds)
    ins.set_facecolor("none")
    ins.add_patch(MplPolygon(np.asarray(verts, dtype=float), closed=True, facecolor="#e2e8f0",
                             edgecolor=SLATE, linestyle="--", linewidth=0.5, alpha=0.75, zorder=1))
    xs = [v[0] for v in verts] + list(b_xs)
    ys = [v[1] for v in verts] + [0.0]
    for pos, _ in (stay, switch):
        xs += [p[0] for p in pos.values()]
        ys += [p[1] for p in pos.values()]
    span = max(max(xs) - min(xs), max(ys) - min(ys))
    pad = 0.05 * span
    ins.plot([min(xs) - pad, max(xs) + pad], [0.0, 0.0], color=SLATE, lw=0.6, zorder=2)
    for (pos, edges), col, size, hollow in ((stay, OCHRE, 9.0, True), (switch, BLUE, 4.5, False)):
        for a, b in edges:
            ins.plot([a[0], b[0]], [a[1], b[1]], color=col, lw=0.45, alpha=0.85, zorder=3)
        px = [p[0] for p in pos.values()]
        py = [p[1] for p in pos.values()]
        ins.scatter(px, py, s=size, facecolor="white" if hollow else col, edgecolor=col,
                    linewidths=0.6, zorder=5 if not hollow else 4)
    ins.scatter(b_xs, [0.0] * len(b_xs), marker="s", s=7, color=SLATE, zorder=6, linewidths=0)
    ins.set_xlim(min(xs) - pad, max(xs) + pad)
    ins.set_ylim(min(ys) - pad, max(ys) + pad)
    ins.set_aspect("equal", anchor=anchor)
    ins.axis("off")


def coverage_panel(ax: plt.Axes, nom: dict, deg: dict, labels: tuple[str, str], extend_to: float | None) -> None:
    t, c = _curve_xy(nom["stay"], extend_to)
    ax.plot(t, c, color=OCHRE, lw=1.4, ls="-", label=f"Stay {labels[0]}, nominal")
    t, c = _curve_xy(deg["stay"], extend_to)
    ax.plot(t, c, color=OCHRE_L, lw=1.1, ls=":", label=f"Stay {labels[0]}, degraded")
    t, c = _curve_xy(nom["switch"], extend_to)
    ax.plot(t, c, color=BLUE, lw=1.5, ls="-", label=f"Switch {labels[1]}, nominal")
    t, c = _curve_xy(deg["switch"], extend_to)
    ax.plot(t, c, color=MOSS, lw=1.1, ls="--", label=f"Switch {labels[1]}, degraded")


def make_figure(sim_dir: Path, geo_dir: Path, out_dir: Path) -> tuple[Path, Path]:
    apply_ieee_style(base_font_size=7.0)
    m6 = load_tdd6_missions(sim_dir)
    t7 = load_tdd7(sim_dir)
    m7 = {str(m["channel_group"]): m for m in t7["missions"]}
    geo = {d["name"]: d for d in json.loads((geo_dir / "convex_domains.json").read_text(encoding="utf-8"))["domains"]}

    def geo_inset(name: str):
        d = geo[name]
        b_xs = [float(v[0]) for v in d["recommended_initial_formation"]["beacons"].values()]
        return (d["vertices"], b_xs, formation_from_geo(d["recommended_initial_formation"]),
                formation_from_geo(d["recommended_target_formation"]))

    fig = plt.figure(figsize=(7.2, 4.10))
    gs = GridSpec(2, 2, figure=fig, hspace=0.55, wspace=0.24, left=0.065, right=0.985, top=0.935, bottom=0.10)
    leg_kw = dict(fontsize=5.4, handlelength=1.9, borderaxespad=0.2, labelspacing=0.24)

    # (a) Regime 1: narrow deep
    ax_a = fig.add_subplot(gs[0, 0])
    nd_nom, nd_deg = m6[("narrow_deep_offshore", "nominal")], m6[("narrow_deep_offshore", "degraded")]
    coverage_panel(ax_a, nd_nom, nd_deg, ("2-chain", "1-chain"), 1350.0)
    ax_a.axhline(95.0, color=GRAY, ls=":", lw=0.6)
    ax_a.axhline(100.0, color=GRAY, ls="-.", lw=0.6)
    t95, t100 = float(nd_nom["switch"]["t95_certified_s"]), float(nd_nom["switch"]["t100_certified_s"])
    ax_a.scatter([t95, t100], [95.0, 100.0], s=18, facecolor=BLUE, edgecolor="white", linewidths=0.6, zorder=6)
    ax_a.annotate(f"$T_{{100}}={t100:.1f}$ s", (t100, 100.0), xytext=(12, -12), textcoords="offset points",
                  fontsize=5.9, color=BLUE, arrowprops=dict(arrowstyle="-", color=BLUE, lw=0.5))
    plateau = float(nd_nom["stay"]["final_certified_coverage"]) * 100.0
    ax_a.text(1330.0, plateau - 2.5, f"Stay plateau {plateau:.1f}%", ha="right", va="top", fontsize=5.9, color=OCHRE)
    ax_a.set_xlim(0.0, 1350.0)
    ax_a.set_ylim(40.0, 103.0)
    ax_a.set_xticks([0, 300, 600, 900, 1200])
    ax_a.set_xlabel("Time $t$ (s)")
    ax_a.set_ylabel("Certified coverage (%)")
    ax_a.legend(loc="lower right", **leg_kw)
    v, bx, st, sw = geo_inset("narrow_deep_offshore")
    draw_inset(ax_a, [0.08, 0.03, 0.42, 0.62], v, bx, st, sw, anchor="S")
    add_panel_label(ax_a, "(a)", "Regime 1: narrow deep, $N=14$, $0.49\\,\\mathrm{km}^2$", fontsize=7.6)

    # (b) Regime 2a: wide shallow (bar comparison)
    ax_b = fig.add_subplot(gs[0, 1])
    ws_nom, ws_deg = m6[("wide_shallow_coastal", "nominal")], m6[("wide_shallow_coastal", "degraded")]
    mx = np.array([0.0, 1.0, 2.0])
    stay_t = [float(ws_nom["stay"]["t95_certified_s"]), float(ws_nom["stay"]["t100_certified_s"]), float(ws_nom["stay"]["final_time_s"])]
    sw_n = [float(ws_nom["switch"]["t95_certified_s"]), float(ws_nom["switch"]["t100_certified_s"]), float(ws_nom["switch"]["reconfig_complete_s"])]
    sw_d = [float(ws_deg["switch"]["t95_certified_s"]), float(ws_deg["switch"]["t100_certified_s"]), float(ws_deg["switch"]["reconfig_complete_s"])]
    bw = 0.23
    ax_b.bar(mx - bw, stay_t, width=bw, color=OCHRE, label="Stay 4-chain, nominal / degraded")
    ax_b.bar(mx, sw_n, width=bw, color=BLUE, label="Switch rebalanced 4-chain, nominal")
    ax_b.bar(mx + bw, sw_d, width=bw, color=SKY, label="Switch rebalanced 4-chain, degraded")
    ax_b.set_yscale("log")
    ax_b.set_ylim(0.8, 12000.0)
    ax_b.set_xlim(-0.55, 2.55)
    ax_b.set_xticks(mx)
    ax_b.set_xticklabels(["$T_{95}$", "$T_{100}$", "Mission / switch"], fontsize=6.5)
    ax_b.set_yticks([1, 10, 100, 1000])
    ax_b.set_yticklabels(["1", "10", "100", "1000"])
    ax_b.minorticks_off()
    ax_b.set_ylabel("Duration (s, log scale)")
    ax_b.text(1.0 - bw, stay_t[1] * 1.25, f"{stay_t[1]:.1f} s", ha="center", va="bottom", fontsize=5.7, color=OCHRE)
    ax_b.text(1.0 + 0.5 * bw, max(sw_n[1], sw_d[1]) * 1.18,
              f"{min(sw_n[1], sw_d[1]):.0f}–{max(sw_n[1], sw_d[1]):.0f} s", ha="center", va="bottom", fontsize=5.7, color=BLUE)
    ax_b.text(2.0 - bw, stay_t[2] * 1.25, f"{stay_t[2]:.1f} s", ha="center", va="bottom", fontsize=5.7, color=OCHRE)
    ax_b.text(2.0 + 0.5 * bw, max(sw_n[2], sw_d[2]) * 1.18,
              f"{min(sw_n[2], sw_d[2]):.0f}–{max(sw_n[2], sw_d[2]):.0f} s", ha="center", va="bottom", fontsize=5.7, color=BLUE)
    ax_b.legend(loc="upper left", **leg_kw)
    v, bx, st, sw = geo_inset("wide_shallow_coastal")
    draw_inset(ax_b, [0.02, 0.25, 0.33, 0.40], v, bx, st, sw, anchor="W")
    add_panel_label(ax_b, "(b)", "Regime 2a: wide shallow, $N=12$, $0.80\\,\\mathrm{km}^2$", fontsize=7.6)

    # (c) Regime 2b: oblique pentagon (Stay saturates below 95%; Switch surpasses T_95)
    ax_c = fig.add_subplot(gs[1, 0])
    ob_nom, ob_deg = m6[("oblique_asymmetric", "nominal")], m6[("oblique_asymmetric", "degraded")]
    coverage_panel(ax_c, ob_nom, ob_deg, ("converging", "2-chain"), None)
    ax_c.axhline(95.0, color=GRAY, ls=":", lw=0.6)
    t95_sw = float(ob_nom["switch"]["t95_certified_s"])
    ax_c.scatter([t95_sw], [95.0], s=18, facecolor=BLUE, edgecolor="white", linewidths=0.6, zorder=6)
    ax_c.annotate(f"$T_{{95}}={t95_sw:.1f}$ s", (t95_sw, 95.0), xytext=(-14, -18), textcoords="offset points",
                  ha="right", fontsize=5.9, color=BLUE, arrowprops=dict(arrowstyle="-", color=BLUE, lw=0.5))
    st_fin_lo = min(float(ob_nom["stay"]["final_certified_coverage"]), float(ob_deg["stay"]["final_certified_coverage"])) * 100.0
    st_fin_hi = max(float(ob_nom["stay"]["final_certified_coverage"]), float(ob_deg["stay"]["final_certified_coverage"])) * 100.0
    sw_fin = float(ob_nom["switch"]["final_certified_coverage"]) * 100.0
    ax_c.text(300.0, st_fin_hi + 1.4, f"Stay {st_fin_lo:.1f}–{st_fin_hi:.1f}%", ha="center", va="bottom", fontsize=5.9, color=OCHRE)
    ax_c.text(645.0, sw_fin + 1.4, f"Switch {sw_fin:.1f}%", ha="right", va="bottom", fontsize=5.9, color=BLUE)
    ax_c.set_xlim(0.0, 650.0)
    ax_c.set_ylim(42.0, 103.5)
    ax_c.set_xticks([0, 150, 300, 450, 600])
    ax_c.set_xlabel("Time $t$ (s)")
    ax_c.set_ylabel("Certified coverage (%)")
    ax_c.legend(loc="lower right", **leg_kw)
    v, bx, st, sw = geo_inset("oblique_asymmetric")
    draw_inset(ax_c, [0.06, 0.04, 0.44, 0.56], v, bx, st, sw, anchor="SW")
    add_panel_label(ax_c, "(c)", "Regime 2b: oblique pentagon, $N=6$, $0.84\\,\\mathrm{km}^2$", fontsize=7.6)

    # (d) Regime 2c: broad coastal domain, net positive breakeven
    ax_d = fig.add_subplot(gs[1, 1])
    n7, d7 = m7["nominal"], m7["degraded"]
    t_end7 = max(float(n7["stay"]["final_time_s"]), float(d7["stay"]["final_time_s"]))
    coverage_panel(ax_d, n7, d7, ("1-chain", "2-chain"), t_end7)
    t_reconf7 = float(n7["switch"]["reconfig_complete_s"])
    t100_sw7, t100_st7 = float(n7["switch"]["t100_certified_s"]), float(n7["stay"]["t100_certified_s"])
    ax_d.axvline(t_reconf7, ymax=0.47, color=GRAY, ls="--", lw=0.7, zorder=2)
    ax_d.text(t_reconf7 + 28.0, 9.5, f"Switch complete\n{t_reconf7:.1f} s", fontsize=5.6, color=GRAY, ha="left", va="bottom")
    ax_d.axhline(95.0, color=GRAY, ls=":", lw=0.6)
    ax_d.axhline(100.0, color=GRAY, ls="-.", lw=0.6)
    ax_d.scatter([t100_sw7, t100_st7], [100.0, 100.0], s=18, facecolor=[BLUE, OCHRE], edgecolor="white", linewidths=0.6, zorder=6)
    ax_d.annotate("", xy=(t100_sw7, 92.5), xytext=(t100_st7, 92.5),
                  arrowprops=dict(arrowstyle="<->", color=BLUE, lw=0.8, shrinkA=0, shrinkB=0))
    ax_d.text(0.5 * (t100_sw7 + t100_st7), 90.0, f"$-{t100_st7 - t100_sw7:.1f}$ s", ha="center", va="top", fontsize=5.9, color=BLUE)
    ax_d.set_xlim(0.0, t_end7 * 1.03)
    ax_d.set_ylim(4.0, 105.0)
    ax_d.set_xticks([0, 500, 1000, 1500, 2000])
    ax_d.set_xlabel("Time $t$ (s)")
    ax_d.set_ylabel("Certified coverage (%)")
    ax_d.legend(loc="lower right", **leg_kw)
    b7 = [float(x) for x in t7["beacon_xs_m"]]
    st7 = staggered_formation(b7, [[], [], list(range(1, 11)), [], []], 380.0, 90.0)
    sw7 = staggered_formation(b7, [list(range(1, 6)), [], [], [], list(range(6, 11))], 380.0, 90.0)
    draw_inset(ax_d, [0.03, 0.50, 0.44, 0.36], t7["vertices_m"], b7, st7, sw7, anchor="NW")
    area7_km2 = float(n7["area_m2"]) / 1e6
    add_panel_label(ax_d, "(d)", f"Regime 2c: broad coastal, $N=10$, ${area7_km2:.2f}\\,\\mathrm{{km}}^2$", fontsize=7.6)

    return save_figure_pair(fig, out_dir, "data_two_regime_coverage")


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
