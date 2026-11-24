"""Figure 4: Three typical convex search domains with matched anchor-rooted
formations, plus cantilever tip uncertainty scaling and certified maximum depth n_max
under the CBF2026 UWB channel model and 3*sqrt(tr(P)).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec, GridSpecFromSubplotSpec
from matplotlib.lines import Line2D
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import route_b_geometry as geom
import route_b_scenarios as scen
from route_b_plot_style import (
    DOUBLE_COL_WIDTH,
    PALETTE,
    add_panel_label,
    apply_ieee_style,
    draw_beacons,
    draw_coastal_band,
    draw_polygon_domain,
    save_figure_pair,
)


def p_avail(d: float, d_r: float, d_max: float) -> float:
    return 0.0 if d > d_max else 1.0 / (1.0 + (d / d_r) ** 2)


def sigma_uwb(d: float, sigma_0: float, d_r: float) -> float:
    return sigma_0 * (1.0 + (d / d_r) ** 2)


def eff_info_weight(d: float, sigma_0: float, d_r: float, d_max: float) -> float:
    pa = p_avail(d, d_r, d_max)
    if pa == 0.0:
        return 0.0
    s = sigma_uwb(d, sigma_0, d_r)
    return pa / (s * s)


def cond_info_weight(d: float, sigma_0: float, d_r: float, d_max: float) -> float:
    if d > d_max:
        return 0.0
    s = sigma_uwb(d, sigma_0, d_r)
    return 1.0 / (s * s)


def build_ladder_jacobian_weights(
    n: int,
    graph_mode: str,
    sigma_0: float,
    d_r: float = 600.0,
    d_max: float = 1200.0,
    use_expected_avail: bool = True,
    s: float = 400.0,
    h1: float = 300.0,
    dy: float = 100.0,
) -> tuple[np.ndarray, np.ndarray]:
    beacons = [(0.0, 0.0), (400.0, 0.0), (800.0, 0.0), (1200.0, 0.0)]
    nodes = {
        j: (400.0 + (0.5 * s if j % 2 else s), h1 + (j - 1) * dy)
        for j in range(1, n + 1)
    }
    rows: list[np.ndarray] = []
    ws: list[float] = []

    def add_edge(i: int, j_pos: tuple[float, float], j_idx: int | None) -> None:
        pi = nodes[i]
        dx, dy_v = pi[0] - j_pos[0], pi[1] - j_pos[1]
        dist = math.hypot(dx, dy_v)
        w = (
            eff_info_weight(dist, sigma_0, d_r, d_max)
            if use_expected_avail
            else cond_info_weight(dist, sigma_0, d_r, d_max)
        )
        if w <= 0.0:
            return
        row = np.zeros(2 * n, dtype=float)
        row[2 * (i - 1)] = dx / dist
        row[2 * (i - 1) + 1] = dy_v / dist
        if j_idx is not None:
            row[2 * (j_idx - 1)] = -dx / dist
            row[2 * (j_idx - 1) + 1] = -dy_v / dist
        rows.append(row)
        ws.append(w)

    if graph_mode == "parents_only":
        for j in range(1, n + 1):
            for p in (j - 2, j - 1):
                if p == -1:
                    add_edge(j, beacons[1], None)
                elif p == 0:
                    add_edge(j, beacons[2], None)
                else:
                    add_edge(j, nodes[p], p)
    elif graph_mode == "all_le_dmax":
        for j in range(1, n + 1):
            for b in beacons:
                add_edge(j, b, None)
        for i in range(1, n + 1):
            for j in range(i + 1, n + 1):
                add_edge(i, nodes[j], j)
    else:
        raise ValueError(graph_mode)

    return np.vstack(rows), np.array(ws, dtype=float)


def solve_steady_state_4n_dare(
    n: int,
    h_pos: np.ndarray,
    w_step: np.ndarray,
    dt: float = 0.1,
    sigma_a: float = 0.25,
    max_iter: int = 12000,
    tol: float = 1e-9,
) -> np.ndarray:
    dim_p = 2 * n
    dim = 4 * n
    f_mat = np.eye(dim, dtype=float)
    f_mat[:dim_p, dim_p:] = dt * np.eye(dim_p)
    g_mat = np.zeros((dim, dim_p), dtype=float)
    g_mat[:dim_p, :] = 0.5 * (dt ** 2) * np.eye(dim_p)
    g_mat[dim_p:, :] = dt * np.eye(dim_p)
    q_mat = (sigma_a ** 2) * (g_mat @ g_mat.T)

    m_pos = (h_pos.T * w_step) @ h_pos
    m_mat = np.zeros((dim, dim), dtype=float)
    m_mat[:dim_p, :dim_p] = m_pos

    p_mat = np.eye(dim, dtype=float)
    eye_dim = np.eye(dim, dtype=float)
    for _ in range(max_iter):
        pm = f_mat @ p_mat @ f_mat.T + q_mat
        p_next = np.linalg.solve(eye_dim + pm @ m_mat, pm)
        p_next = 0.5 * (p_next + p_next.T)
        if np.max(np.abs(p_next - p_mat)) < tol * max(1.0, float(np.max(np.abs(p_next)))):
            return p_next
        p_mat = p_next
    return p_mat


def load_depth_scaling(geo_dir: Path) -> dict[str, np.ndarray]:
    """Load official geo-outputs/convex_domains.json depth_scaling_uwb curves."""
    json_path = geo_dir / "convex_domains.json"
    if not json_path.exists():
        raise FileNotFoundError(f"Expected {json_path}")
    raw = json.loads(json_path.read_text(encoding="utf-8"))
    series_map = raw["depth_scaling_uwb"]["series"]
    out: dict[str, np.ndarray] = {}
    for raw_key, tag in (("sigma_0_0.5", "s05"), ("sigma_0_1.0", "s10")):
        rows = series_map[raw_key]
        out["n"] = np.array([int(r["n"]) for r in rows], dtype=int)
        out[f"crlb_{tag}_par"] = np.array([float(r["crlb_parents_only_m"]) for r in rows], dtype=float)
        out[f"crlb_{tag}_all"] = np.array([float(r["crlb_all_le_dmax_m"]) for r in rows], dtype=float)
        out[f"dare25_{tag}_par"] = np.array([float(r["dare_parents_only_sigma_a_025_m"]) for r in rows], dtype=float)
        out[f"dare25_{tag}_all"] = np.array([float(r["dare_all_le_dmax_sigma_a_025_m"]) for r in rows], dtype=float)
        out[f"dare50_{tag}_par"] = np.array([float(r["dare_parents_only_sigma_a_050_m"]) for r in rows], dtype=float)
        out[f"dare50_{tag}_all"] = np.array([float(r["dare_all_le_dmax_sigma_a_050_m"]) for r in rows], dtype=float)
    return out


def _tri_from_json_dict(d: dict[str, object], params: geom.GeometryParams) -> geom.AnchorRootedTriangulation:
    beacons = {k: (float(v[0]), float(v[1])) for k, v in d["beacons"].items()}  # type: ignore[union-attr]
    positions = {int(k): (float(v[0]), float(v[1])) for k, v in d["positions"].items()}  # type: ignore[union-attr]
    parents = {
        int(k): (v[0] if v[0] in beacons else int(v[0]), v[1] if v[1] in beacons else int(v[1]))
        for k, v in d["parents"].items()  # type: ignore[union-attr]
    }
    faces = tuple(
        tuple(x if x in beacons else int(x) for x in f)
        for f in d["faces"]  # type: ignore[union-attr]
    )
    return geom.AnchorRootedTriangulation(
        beacons=beacons,
        beacon_order=tuple(d["beacon_order"]),  # type: ignore[arg-type]
        positions=positions,
        parents=parents,
        faces=faces,  # type: ignore[arg-type]
        root_faces=(),
        support_face={},
        h_b=float(d["h_b_m"]),  # type: ignore[arg-type]
        h_c=float(d["h_c_m"]),  # type: ignore[arg-type]
        params=params,
    )


def _draw_triangulation_overlay(
    ax: plt.Axes,
    tri: geom.AnchorRootedTriangulation,
    edge_color: str,
    node_color: str,
    show_beacon_labels: bool = True,
) -> None:
    coords = tri.all_coords()
    for nid, ps in tri.parents.items():
        for p in ps:
            pa, pb = coords[nid], coords[p]
            ax.plot([pa[0], pb[0]], [pa[1], pb[1]], color=edge_color, lw=0.95, alpha=0.85, zorder=4)
    if show_beacon_labels:
        draw_beacons(ax, tri.beacons, size=20.0, y_offset_pts=-5.0, fontsize=5.8, zorder=7)
    else:
        xs = [pt[0] for pt in tri.beacons.values()]
        ys = [pt[1] for pt in tri.beacons.values()]
        ax.scatter(xs, ys, marker="s", s=20.0, facecolor=PALETTE["navy"], edgecolor="white", linewidths=0.6, zorder=7)
    xs_u = [pt[0] for pt in tri.positions.values()]
    ys_u = [pt[1] for pt in tri.positions.values()]
    ax.scatter(xs_u, ys_u, s=20, facecolor=node_color, edgecolor="white", linewidths=0.6, zorder=8)


def _load_domains_and_formations(geo_dir: Path) -> list[dict[str, object]]:
    """Load all 3 convex polygonal domains & recommended standard formations from convex_domains.json."""
    params = geom.GeometryParams(d_dep=850.0, d_clear=40.0, v_axis_max=25.0)
    json_path = geo_dir / "convex_domains.json"
    if not json_path.exists():
        raise FileNotFoundError(f"Expected {json_path}")
    raw = json.loads(json_path.read_text(encoding="utf-8"))
    entries = {item["name"]: item for item in raw["domains"]}

    def dom_from_entry(name: str) -> scen.ConvexSearchDomain:
        item = entries[name]
        return scen.ConvexSearchDomain(
            name=name,
            description=str(item["description"]),
            vertices=tuple((float(v[0]), float(v[1])) for v in item["vertices"]),
        )

    tri_wide = _tri_from_json_dict(entries["wide_shallow_coastal"]["recommended_initial_formation"], params)
    tri_deep = _tri_from_json_dict(entries["narrow_deep_offshore"]["recommended_target_formation"], params)
    tri_oblique = _tri_from_json_dict(entries["oblique_asymmetric"]["recommended_initial_formation"], params)

    return [
        {
            "tag": "(a)",
            "title": "Wide Shallow Coastal",
            "subtitle": "Area: {area:.2f} $\\mathrm{{km}}^2$  |  Parallel ladders ($M=5, N=12$)",
            "domain": dom_from_entry("wide_shallow_coastal"),
            "tri": tri_wide,
            "color": PALETTE["blue"],
            "xlim": (-120.0, 1720.0),
            "ylim": (-185.0, 990.0),
            "badge_pos": (0.50, 0.96, "center", "top"),
            "xticks": [0, 400, 800, 1200, 1600],
        },
        {
            "tag": "(b)",
            "title": "Narrow Deep Offshore",
            "subtitle": "Area: {area:.2f} $\\mathrm{{km}}^2$\nMerged ladder\n($M=4, N=14$)",
            "domain": dom_from_entry("narrow_deep_offshore"),
            "tri": tri_deep,
            "color": PALETTE["vermilion"],
            "xlim": (-720.0, 1920.0),
            "ylim": (-150.0, 1720.0),
            "badge_pos": (0.02, 0.96, "left", "top"),
            "xticks": [0, 400, 800, 1200],
        },
        {
            "tag": "(c)",
            "title": "Oblique Asymmetric",
            "subtitle": "Area: {area:.2f} $\\mathrm{{km}}^2$\nPinball $P_6$\n($M=4, N=6$)",
            "domain": dom_from_entry("oblique_asymmetric"),
            "tri": tri_oblique,
            "color": PALETTE["teal"],
            "xlim": (-520.0, 1720.0),
            "ylim": (-140.0, 1420.0),
            "badge_pos": (0.02, 0.96, "left", "top"),
            "xticks": [0, 400, 800, 1200],
        },
    ]


def make_figure(geo_dir: Path, out_dir: Path) -> tuple[Path, Path]:
    apply_ieee_style(base_font_size=8.0)
    domain_specs = _load_domains_and_formations(geo_dir)
    depth_data = load_depth_scaling(geo_dir)

    fig = plt.figure(figsize=(DOUBLE_COL_WIDTH, 5.10))
    outer = GridSpec(1, 2, figure=fig, width_ratios=[1.02, 1.15], wspace=0.28,
                     left=0.075, right=0.985, top=0.945, bottom=0.085)
    left_gs = GridSpecFromSubplotSpec(3, 1, subplot_spec=outer[0], height_ratios=[0.95, 1.12, 1.0], hspace=0.40)
    right_gs = GridSpecFromSubplotSpec(2, 1, subplot_spec=outer[1], height_ratios=[1.15, 1.0], hspace=0.35)

    # ------------------------------------------------------------------
    # Left Column: (a)-(c) Three Typical Convex Search Domains
    # ------------------------------------------------------------------
    for idx, spec in enumerate(domain_specs):
        ax = fig.add_subplot(left_gs[idx])
        dom: scen.ConvexSearchDomain = spec["domain"]  # type: ignore[assignment]
        tri: geom.AnchorRootedTriangulation = spec["tri"]  # type: ignore[assignment]
        col: str = spec["color"]  # type: ignore[assignment]

        xmin, xmax = spec["xlim"]  # type: ignore[misc]
        ymin, ymax = spec["ylim"]  # type: ignore[misc]
        draw_coastal_band(ax, xmin, xmax, ymin, ymax, shore_y=0.0, zorder_base=0)
        draw_polygon_domain(ax, dom.vertices, edgecolor=col, facecolor=col, fill_alpha=0.14, lw=1.35, ls="--", zorder=2)

        if dom.name == "narrow_deep_offshore":
            ax.axhline(1200.0, color=PALETTE["amber"], ls=":", lw=1.05, zorder=3)
            ax.text(1880.0, 1235.0, "$d_{\\max}=1200\\,\\mathrm{m}$", ha="right", va="bottom",
                    fontsize=5.9, color="#9E6200", fontweight="bold")

        _draw_triangulation_overlay(ax, tri, edge_color=col, node_color=col, show_beacon_labels=(idx == 0))

        # Area & formation badge in guaranteed open whitespace
        area_km2 = dom.area_m2 / 1e6
        bx, by, bha, bva = spec["badge_pos"]  # type: ignore[misc]
        badge_txt = str(spec["subtitle"]).format(area=area_km2)
        ax.text(
            bx,
            by,
            badge_txt,
            transform=ax.transAxes,
            ha=bha,
            va=bva,
            fontsize=5.7,
            color=PALETTE["black"],
            bbox=dict(boxstyle="round,pad=0.18", facecolor="white", edgecolor=PALETTE["gray_light"], alpha=0.94, lw=0.65),
            zorder=10,
        )

        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
        ax.set_aspect("equal", adjustable="box")
        ax.set_ylabel("Offshore (m)", fontsize=7.2)
        ax.set_xticks(spec["xticks"])  # type: ignore[arg-type]
        if idx == 2:
            ax.set_xlabel("Alongshore (m)", fontsize=7.2)
        elif idx == 0:
            ax.set_xticklabels([str(x) for x in spec["xticks"]], fontsize=6.4)  # type: ignore[union-attr]
        else:
            ax.set_xticklabels([])
        ax.tick_params(labelsize=6.5)
        add_panel_label(ax, spec["tag"], spec["title"], fontsize=7.8)

    # ------------------------------------------------------------------
    # Top-Right: (d) Steady-State 4D DARE 3*sqrt(tr(P)) & Certified n_max
    # ------------------------------------------------------------------
    ax_d = fig.add_subplot(right_gs[0])
    ns = depth_data["n"]

    series = [
        ("s05_all", PALETTE["teal"], "-", "o", "$\\sigma_0=0.5\\,\\mathrm{m}$, all $\\leq 1200\\,\\mathrm{m}$"),
        ("s05_par", PALETTE["blue"], "--", "s", "$\\sigma_0=0.5\\,\\mathrm{m}$, parents only"),
        ("s10_all", PALETTE["amber"], "-", "^", "$\\sigma_0=1.0\\,\\mathrm{m}$, all $\\leq 1200\\,\\mathrm{m}$"),
        ("s10_par", PALETTE["vermilion"], "--", "d", "$\\sigma_0=1.0\\,\\mathrm{m}$, parents only"),
    ]
    for key_suf, col, ls, mk, lbl in series:
        y25 = depth_data[f"dare25_{key_suf}"]
        y50 = depth_data[f"dare50_{key_suf}"]
        ax_d.fill_between(ns, y25, y50, color=col, alpha=0.14, zorder=2)
        ax_d.plot(ns, y25, color=col, ls=ls, lw=1.6, marker=mk, markersize=3.5, label=lbl, zorder=4)

    # Certification thresholds eps_allow in {2.0, 3.0, 5.0} m in guaranteed curve-free positions
    ax_d.axhline(2.0, color=PALETTE["slate"], ls=":", lw=0.85, alpha=0.85, zorder=1)
    ax_d.text(14.25, 1.94, "$\\bar{\\epsilon}=2\\,\\mathrm{m}$", fontsize=5.9, color=PALETTE["slate"], ha="left", va="top")

    ax_d.axhline(3.0, color=PALETTE["slate"], ls=":", lw=0.85, alpha=0.85, zorder=1)
    ax_d.text(2.15, 3.12, "$\\bar{\\epsilon}=3\\,\\mathrm{m}$", fontsize=5.9, color=PALETTE["slate"], ha="left", va="bottom")

    ax_d.axhline(5.0, color=PALETTE["slate"], ls=":", lw=0.85, alpha=0.85, zorder=1)
    ax_d.text(2.15, 5.12, "$\\bar{\\epsilon}=5\\,\\mathrm{m}$", fontsize=5.9, color=PALETTE["slate"], ha="left", va="bottom")

    nmax_notes = [
        (4, float(depth_data["dare25_s10_par"][2]), PALETTE["vermilion"], PALETTE["vermilion"], (-10, 16), "$n_{\\max}=4$"),
        (9, float(depth_data["dare25_s05_par"][7]), PALETTE["blue"], PALETTE["blue"], (-18, 16), "$n_{\\max}=9$"),
        (11, float(depth_data["dare25_s10_all"][9]), PALETTE["amber"], "#9E6200", (-2, -14), "$n_{\\max}=11$"),
        (13, float(depth_data["dare25_s05_all"][11]), PALETTE["teal"], PALETTE["teal"], (-16, -15), "$n_{\\max}=13$"),
    ]
    for nx, ny, mk_col, txt_col, off, txt in nmax_notes:
        ax_d.scatter([nx], [ny], s=42, facecolor=mk_col, edgecolor="white", linewidths=0.8, zorder=6)
        ax_d.annotate(
            txt,
            (nx, ny),
            xytext=off,
            textcoords="offset points",
            fontsize=6.0,
            fontweight="bold",
            color=txt_col,
            bbox=dict(boxstyle="round,pad=0.10", facecolor="white", edgecolor="none", alpha=0.85),
            arrowprops=dict(arrowstyle="-", color=mk_col, lw=0.6),
            zorder=7,
        )

    ax_d.set_xlim(1.7, 15.8)
    ax_d.set_ylim(0.5, 8.8)
    ax_d.grid(True, axis="both")
    ax_d.set_xticks(range(2, 15, 2))
    ax_d.set_ylabel("Tip $3\\sqrt{\\mathrm{tr}(\\boldsymbol{P}_{n,\\infty}^p)}$ (m)")
    ax_d.legend(loc="upper left", bbox_to_anchor=(0.01, 0.99), frameon=True, facecolor="white",
                edgecolor=PALETTE["gray_light"], framealpha=0.94, fontsize=5.7,
                title="Shaded band: $\\sigma_a \\in [0.25, 0.50]\\,\\mathrm{m/s}^2$",
                title_fontsize=5.5)
    add_panel_label(ax_d, "(d)", "4D DARE tip uncertainty & certified $n_{\\max}$", fontsize=8.0)

    # ------------------------------------------------------------------
    # Bottom-Right: (e) Log-Log Scaling Verification: CRLB O(n^{3/2}) vs DARE O(n^{9/8})
    # ------------------------------------------------------------------
    ax_e = fig.add_subplot(right_gs[1])
    ax_e.loglog(ns, depth_data["crlb_s10_par"], color=PALETTE["purple"], ls="-.", lw=1.4, marker="^",
                markersize=3.2, label="Static CRLB ($\\sigma_0=1.0$, parents)")
    ax_e.loglog(ns, depth_data["crlb_s05_par"], color=PALETTE["navy"], ls="-.", lw=1.4, marker="o",
                markersize=3.2, label="Static CRLB ($\\sigma_0=0.5$, parents)")
    ax_e.loglog(ns, depth_data["dare25_s10_par"], color=PALETTE["vermilion"], ls="--", lw=1.5, marker="d",
                markersize=3.2, label="4D DARE ($\\sigma_0=1.0$, parents)")
    ax_e.loglog(ns, depth_data["dare25_s05_par"], color=PALETTE["blue"], ls="--", lw=1.5, marker="s",
                markersize=3.2, label="4D DARE ($\\sigma_0=0.5$, parents)")
    ax_e.loglog(ns, depth_data["dare25_s05_all"], color=PALETTE["teal"], ls="-", lw=1.6, marker="o",
                markersize=3.2, label="4D DARE ($\\sigma_0=0.5$, $\\leq 1200\\,\\mathrm{m}$)")

    # Reference slope guides in log-log space
    n_ref = np.array([5.0, 12.0])
    y_ref_32 = 1.65 * (n_ref ** 1.5)
    ax_e.loglog(n_ref, y_ref_32, color=PALETTE["slate"], ls=":", lw=0.95)
    ax_e.text(7.2, 46.0, "$\\propto n^{3/2}$", fontsize=6.4, color=PALETTE["slate"], fontweight="bold")

    y_ref_98 = 0.19 * (n_ref ** 1.125)
    ax_e.loglog(n_ref, y_ref_98, color=PALETTE["slate"], ls=":", lw=0.95)
    ax_e.text(7.2, 1.48, "$\\propto n^{9/8}$", fontsize=6.4, color=PALETTE["slate"], fontweight="bold")

    ax_e.axvline(10.0, color=PALETTE["amber"], ls=":", lw=1.0, zorder=2)
    ax_e.text(10.25, 0.75, "$y_{10}=d_{\\max}$\n(beacon cutoff)", fontsize=5.6, color="#9E6200", va="bottom")

    ax_e.set_xlim(1.85, 15.0)
    ax_e.set_ylim(0.70, 165.0)
    ax_e.grid(True, which="major")
    ax_e.set_xticks([2, 3, 4, 6, 8, 10, 14])
    ax_e.set_xticklabels(["2", "3", "4", "6", "8", "10", "14"])
    ax_e.set_yticks([1, 2, 5, 10, 20, 50, 100])
    ax_e.set_yticklabels(["1", "2", "5", "10", "20", "50", "100"])
    ax_e.minorticks_off()
    ax_e.set_xlabel("Ladder depth $n$ (aircraft)")
    ax_e.set_ylabel("Tip $3\\sqrt{\\mathrm{tr}(P)}$ (m, log)")
    ax_e.legend(loc="upper left", frameon=True, facecolor="white", edgecolor=PALETTE["gray_light"],
                framealpha=0.95, fontsize=5.3, ncol=1)
    add_panel_label(ax_e, "(e)", "Cantilever power-law scaling verification", fontsize=8.0)

    return save_figure_pair(fig, out_dir, "fig4_convex_domains_and_depth_limit")


def main() -> None:
    parser = argparse.ArgumentParser()
    root = Path(__file__).resolve().parents[2]
    parser.add_argument(
        "--geo-dir",
        type=Path,
        default=root / "data" / "geo_outputs",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=root / "outputs" / "figures",
    )
    args = parser.parse_args()
    pdf_p, png_p = make_figure(args.geo_dir, args.out_dir)
    print(f"Wrote {pdf_p} and {png_p}")


if __name__ == "__main__":
    main()
