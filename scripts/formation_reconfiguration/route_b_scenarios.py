"""Multiscale scenario generation and typical convex polygonal search domains."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from route_b_geometry import (
    AnchorRootedTriangulation,
    GeometryParams,
    Move,
    audit_triangulation_dual_paths,
    build_forked_y_tree_triangulation,
    build_pinball_triangulation,
    build_staggered_triangulation,
    circumcenter,
    default_berth_layout,
    min_berth_height,
    plan_direct_matched_transfer,
    plan_labelled_transfer,
    segment_three_ref_min_eigenvalue,
    staggered_circumcenter,
    worst_berth_eigenvalue,
)
from route_b_pipeline import (
    simulate_pipelined_execution,
    simulate_serial_execution,
)


@dataclass(frozen=True)
class ConvexSearchDomain:
    """Convex polygonal offshore search domain with geometric metrics."""

    name: str
    description: str
    vertices: tuple[tuple[float, float], ...]

    def is_convex(self) -> bool:
        n = len(self.vertices)
        if n < 3:
            return False
        signs: list[float] = []
        for i in range(n):
            a = self.vertices[i]
            b = self.vertices[(i + 1) % n]
            c = self.vertices[(i + 2) % n]
            cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
            if abs(cross) > 1e-9:
                signs.append(cross)
        return bool(signs) and (all(s > 0.0 for s in signs) or all(s < 0.0 for s in signs))

    @property
    def area_m2(self) -> float:
        n = len(self.vertices)
        s = 0.0
        for i in range(n):
            x1, y1 = self.vertices[i]
            x2, y2 = self.vertices[(i + 1) % n]
            s += x1 * y2 - x2 * y1
        return 0.5 * abs(s)

    @property
    def centroid(self) -> tuple[float, float]:
        xs = [v[0] for v in self.vertices]
        ys = [v[1] for v in self.vertices]
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    @property
    def max_offshore_y(self) -> float:
        return max(v[1] for v in self.vertices)

    @property
    def alongshore_span(self) -> float:
        xs = [v[0] for v in self.vertices]
        return max(xs) - min(xs)

    def contains_point(self, pt: tuple[float, float], tol: float = 1e-9) -> bool:
        n = len(self.vertices)
        signs: list[float] = []
        for i in range(n):
            a = self.vertices[i]
            b = self.vertices[(i + 1) % n]
            cross = (b[0] - a[0]) * (pt[1] - a[1]) - (b[1] - a[1]) * (pt[0] - a[0])
            signs.append(cross)
        return all(c >= -tol for c in signs) or all(c <= tol for c in signs)


@dataclass(frozen=True)
class TransferScenario:
    """Paired standard deployments with both reversible and direct-matched plans."""

    scale_key: tuple[int, int]
    source: AnchorRootedTriangulation
    target: AnchorRootedTriangulation
    reversible_moves: tuple[Move, ...]
    direct_matched_moves: tuple[Move, ...]
    direct_matched_target: AnchorRootedTriangulation


def standard_convex_search_domains() -> tuple[ConvexSearchDomain, ...]:
    """Three typical convex polygonal search domains for downstream closed-loop evaluation."""
    return (
        ConvexSearchDomain(
            name="wide_shallow_coastal",
            description="Wide shallow coastal convex polygon favoring parallel multi-interval coverage",
            vertices=((0.0, 220.0), (1600.0, 220.0), (1480.0, 760.0), (120.0, 760.0)),
        ),
        ConvexSearchDomain(
            name="narrow_deep_offshore",
            description="Narrow deep offshore convex polygon triggering deep-reach merging",
            vertices=((380.0, 250.0), (820.0, 250.0), (760.0, 1550.0), (440.0, 1550.0)),
        ),
        ConvexSearchDomain(
            name="oblique_asymmetric",
            description="Oblique asymmetric convex pentagon skewed across offshore intervals",
            vertices=((100.0, 240.0), (920.0, 260.0), (1420.0, 880.0), (960.0, 1280.0), (320.0, 920.0)),
        ),
    )


def generate_multiscale_scenarios(
    params: GeometryParams | None = None,
    h_b: float = 100.0,
    h_c: float = 140.0,
) -> dict[tuple[int, int], TransferScenario]:
    """Generate (N, M) = (6, 3), (14, 4), and (20, 5) standard transfer scenarios."""
    cfg = params or GeometryParams()
    specs = [
        (
            (6, 3),
            (0.0, 400.0, 800.0),
            (tuple(range(1, 7)), ()),
            ((), tuple(range(6, 0, -1))),
        ),
        (
            (14, 4),
            (0.0, 400.0, 800.0, 1200.0),
            (tuple(range(1, 8)), (), tuple(range(8, 15))),
            ((), tuple(range(14, 0, -1)), ()),
        ),
        (
            (20, 5),
            (0.0, 400.0, 800.0, 1200.0, 1600.0),
            (tuple(range(1, 11)), (), (), tuple(range(11, 21))),
            ((), tuple(range(10, 0, -1)), tuple(range(20, 10, -1)), ()),
        ),
    ]

    bundle: dict[tuple[int, int], TransferScenario] = {}
    for key, beacon_xs, src_units, dst_units in specs:
        src = build_staggered_triangulation(
            beacon_xs=beacon_xs,
            units=src_units,
            h1=380.0,
            dy=90.0,
            h_b=h_b,
            h_c=h_c,
            params=cfg,
        )
        dst = build_staggered_triangulation(
            beacon_xs=beacon_xs,
            units=dst_units,
            h1=380.0,
            dy=90.0,
            h_b=h_b,
            h_c=h_c,
            params=cfg,
        )
        rev_moves = tuple(plan_labelled_transfer(src, dst, h_b=h_b, h_c=h_c))
        dir_moves, matched_dst = plan_direct_matched_transfer(src, dst, h_c=h_c)
        bundle[key] = TransferScenario(
            scale_key=key,
            source=src,
            target=dst,
            reversible_moves=rev_moves,
            direct_matched_moves=tuple(dir_moves),
            direct_matched_target=matched_dst,
        )
    return bundle


def _fim_min_eig(pt: tuple[float, float], refs: list[tuple[float, float]], weights: list[float] | None = None) -> float:
    mat = np.zeros((2, 2), dtype=float)
    for idx, ref in enumerate(refs):
        dx, dy = pt[0] - ref[0], pt[1] - ref[1]
        norm = math.hypot(dx, dy)
        u = np.array([dx / norm, dy / norm], dtype=float)
        w = 1.0 if weights is None else weights[idx]
        mat += w * np.outer(u, u)
    return float(np.linalg.eigvalsh(mat)[0])


def _uwb_weights(d: float, sigma_0: float, d_r: float, d_max: float) -> tuple[float, float]:
    """Return (conditional_weight, expected_weight) under CBF2026 UWB attenuation model."""
    if d > d_max:
        return 0.0, 0.0
    factor = 1.0 + (d / d_r) ** 2
    sigma = sigma_0 * factor
    w_cond = 1.0 / (sigma * sigma)
    p_avail = 1.0 / factor
    return w_cond, p_avail * w_cond


def _solve_ladder_4n_dare(
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
    g_mat[:dim_p, :] = 0.5 * (dt * dt) * np.eye(dim_p)
    g_mat[dim_p:, :] = dt * np.eye(dim_p)
    q_mat = (sigma_a * sigma_a) * (g_mat @ g_mat.T)
    m_pos = (h_pos.T * w_step) @ h_pos
    m_mat = np.zeros((dim, dim), dtype=float)
    m_mat[:dim_p, :dim_p] = m_pos
    p_cov = np.eye(dim, dtype=float)
    eye = np.eye(dim, dtype=float)
    for _ in range(max_iter):
        p_minus = f_mat @ p_cov @ f_mat.T + q_mat
        p_next = np.linalg.solve(eye + p_minus @ m_mat, p_minus)
        p_next = 0.5 * (p_next + p_next.T)
        if float(np.max(np.abs(p_next - p_cov))) < tol * max(1.0, float(np.max(np.abs(p_next)))):
            return p_next
        p_cov = p_next
    return p_cov


def _compute_depth_scaling_uwb(params: GeometryParams) -> dict[str, object]:
    """Compute n=2..14 tip 3*sqrt(tr(P)) under CBF2026 R&R UWB channel (parents_only vs all<=d_max)."""
    s = 400.0
    h1 = 300.0
    dy = 100.0
    dt = 0.1
    beacons = [(0.0, 0.0), (400.0, 0.0), (800.0, 0.0), (1200.0, 0.0)]
    results: dict[str, list[dict[str, float | int]]] = {}

    for sigma_0 in (0.5, 1.0):
        rows_out: list[dict[str, float | int]] = []
        for n in range(2, 15):
            nodes = {j: (400.0 + (0.5 * s if j % 2 else s), h1 + (j - 1) * dy) for j in range(1, n + 1)}

            def build_system(mode: str, expected: bool) -> tuple[np.ndarray, np.ndarray]:
                jac_rows: list[np.ndarray] = []
                ws: list[float] = []

                def add_edge(i: int, p_j: tuple[float, float], j_idx: int | None) -> None:
                    dx, dy_v = nodes[i][0] - p_j[0], nodes[i][1] - p_j[1]
                    dist_ij = math.hypot(dx, dy_v)
                    wc, we = _uwb_weights(dist_ij, sigma_0, params.d_r, params.d_max)
                    w = we if expected else wc
                    if w <= 0.0:
                        return
                    row = np.zeros(2 * n, dtype=float)
                    row[2 * (i - 1)] = dx / dist_ij
                    row[2 * (i - 1) + 1] = dy_v / dist_ij
                    if j_idx is not None:
                        row[2 * (j_idx - 1)] = -dx / dist_ij
                        row[2 * (j_idx - 1) + 1] = -dy_v / dist_ij
                    jac_rows.append(row)
                    ws.append(w)

                if mode == "parents_only":
                    for j in range(1, n + 1):
                        for p in (j - 2, j - 1):
                            if p == -1:
                                add_edge(j, beacons[1], None)
                            elif p == 0:
                                add_edge(j, beacons[2], None)
                            else:
                                add_edge(j, nodes[p], p)
                else:
                    for j in range(1, n + 1):
                        for b_pt in beacons:
                            add_edge(j, b_pt, None)
                    for i in range(1, n + 1):
                        for j in range(i + 1, n + 1):
                            add_edge(i, nodes[j], j)
                return np.vstack(jac_rows), np.array(ws, dtype=float)

            hp_par, w_par_c = build_system("parents_only", False)
            hp_all, w_all_c = build_system("all_le_dmax", False)
            _, w_par_e = build_system("parents_only", True)
            _, w_all_e = build_system("all_le_dmax", True)

            c_par = np.linalg.inv((hp_par.T * w_par_c) @ hp_par)
            c_all = np.linalg.inv((hp_all.T * w_all_c) @ hp_all)
            p_par_25 = _solve_ladder_4n_dare(n, hp_par, w_par_e, dt=dt, sigma_a=0.25)
            p_all_25 = _solve_ladder_4n_dare(n, hp_all, w_all_e, dt=dt, sigma_a=0.25)
            p_par_50 = _solve_ladder_4n_dare(n, hp_par, w_par_e, dt=dt, sigma_a=0.50)
            p_all_50 = _solve_ladder_4n_dare(n, hp_all, w_all_e, dt=dt, sigma_a=0.50)

            sl = slice(2 * (n - 1), 2 * n)
            rows_out.append(
                {
                    "n": n,
                    "y_tip_m": h1 + (n - 1) * dy,
                    "crlb_parents_only_m": round(3.0 * math.sqrt(float(np.trace(c_par[sl, sl]))), 4),
                    "crlb_all_le_dmax_m": round(3.0 * math.sqrt(float(np.trace(c_all[sl, sl]))), 4),
                    "dare_parents_only_sigma_a_025_m": round(3.0 * math.sqrt(float(np.trace(p_par_25[sl, sl]))), 4),
                    "dare_all_le_dmax_sigma_a_025_m": round(3.0 * math.sqrt(float(np.trace(p_all_25[sl, sl]))), 4),
                    "dare_parents_only_sigma_a_050_m": round(3.0 * math.sqrt(float(np.trace(p_par_50[sl, sl]))), 4),
                    "dare_all_le_dmax_sigma_a_050_m": round(3.0 * math.sqrt(float(np.trace(p_all_50[sl, sl]))), 4),
                }
            )
        results[f"sigma_0_{sigma_0:.1f}"] = rows_out
    return {
        "channel": {
            "d_dep_m": params.d_dep,
            "d_max_m": params.d_max,
            "d_r_m": params.d_r,
            "d_safe_m": params.d_safe,
            "v_axis_max_mps": params.v_axis_max,
            "uncertainty_scale": "3*sqrt(tr(P_p))",
            "dt_s": dt,
        },
        "series": results,
    }


def _serialize_triangulation(tri: AnchorRootedTriangulation) -> dict[str, object]:
    return {
        "beacons": {str(k): list(v) for k, v in tri.beacons.items()},
        "beacon_order": [str(b) for b in tri.beacon_order],
        "positions": {str(k): list(v) for k, v in tri.positions.items()},
        "parents": {str(k): [str(p) if isinstance(p, str) else int(p) for p in ps] for k, ps in tri.parents.items()},
        "faces": [[str(v) if isinstance(v, str) else int(v) for v in f] for f in tri.faces],
        "h_b_m": tri.h_b,
        "h_c_m": tri.h_c,
    }


def _serialize_moves(moves: tuple[Move, ...] | list[Move]) -> list[dict[str, object]]:
    return [
        {
            "node": int(m.node),
            "legs": [
                {
                    "start": list(leg.start),
                    "end": list(leg.end),
                    "parents": [str(p) if isinstance(p, str) else int(p) for p in leg.parents],
                    "auxiliary": (str(leg.auxiliary) if isinstance(leg.auxiliary, str) else leg.auxiliary),
                    "prerequest_next_parent": (
                        str(leg.prerequest_next_parent)
                        if isinstance(leg.prerequest_next_parent, str)
                        else leg.prerequest_next_parent
                    ),
                    "kind": leg.kind,
                }
                for leg in m.legs
            ],
        }
        for m in moves
    ]


def export_geo_outputs(output_dir: Path | str, params: GeometryParams | None = None) -> dict[str, Path]:
    """Export clean JSON datasets for downstream visualization and simulation."""
    cfg = params or GeometryParams()
    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    # 1. berth_sweep.json
    beacons_4 = [(0.0, 0.0), (400.0, 0.0), (800.0, 0.0), (1200.0, 0.0)]
    xs_legacy = [320.0 + 40.0 * i for i in range(14)]
    sweep_rows: list[dict[str, object]] = []
    for hb in range(20, 165, 5):
        hb_f = float(hb)
        lam_local = worst_berth_eigenvalue(400.0, hb_f)
        lam_vis_mid = _fim_min_eig((600.0, hb_f), beacons_4)
        w2 = [_uwb_weights(math.dist((600.0, hb_f), b), 0.5, cfg.d_r, cfg.d_max)[0] for b in [(400.0, 0.0), (800.0, 0.0)]]
        w4 = [_uwb_weights(math.dist((600.0, hb_f), b), 0.5, cfg.d_r, cfg.d_max)[0] for b in beacons_4]
        lam_uwb_2 = _fim_min_eig((600.0, hb_f), [(400.0, 0.0), (800.0, 0.0)], w2)
        lam_uwb_4 = _fim_min_eig((600.0, hb_f), beacons_4, w4)
        legacy_2 = min(_fim_min_eig((x, hb_f), [(400.0, 0.0), (800.0, 0.0)]) for x in xs_legacy)
        legacy_max_d = max(max(math.dist((x, hb_f), (400.0, 0.0)), math.dist((x, hb_f), (800.0, 0.0))) for x in xs_legacy)
        sweep_rows.append(
            {
                "h_b_m": hb_f,
                "interval_local_worst_lambda_min": round(lam_local, 6),
                "interval_local_midpoint_dist_m": round(math.hypot(200.0, hb_f), 3),
                "interval_local_max_dist_m": round(math.hypot(400.0, hb_f), 3),
                "central_all_visible_4bcn_lambda_min": round(lam_vis_mid, 6),
                "uwb_weighted_2bcn_eig_at_600": round(lam_uwb_2, 6),
                "uwb_weighted_4bcn_eig_at_600": round(lam_uwb_4, 6),
                "legacy_bc_worst_lambda_min": round(legacy_2, 6),
                "legacy_bc_max_dist_m": round(legacy_max_d, 3),
                "meets_iota_req": bool(lam_local >= cfg.iota_req - 1e-9),
                "meets_d_clear": bool(hb_f >= cfg.d_clear - 1e-9),
            }
        )
    berth_pos_14, berth_par_14 = default_berth_layout((0.0, 400.0, 800.0, 1200.0), ("b0", "b1", "b2", "b3"), list(range(1, 15)), 100.0, cfg.berth_spacing)
    berth_payload = {
        "params": {
            "s_m": 400.0,
            "iota_req": cfg.iota_req,
            "h_b_min_m": min_berth_height(400.0, cfg.iota_req),
            "d_dep_m": cfg.d_dep,
            "d_max_m": cfg.d_max,
            "d_r_m": cfg.d_r,
            "d_clear_m": cfg.d_clear,
            "berth_spacing_m": cfg.berth_spacing,
        },
        "sweep": sweep_rows,
        "recommended_berths_14_at_100m": {
            str(k): {"position": list(berth_pos_14[k]), "parents": [str(p) for p in berth_par_14[k]]}
            for k in sorted(berth_pos_14)
        },
    }
    berth_file = out_path / "berth_sweep.json"
    berth_file.write_text(json.dumps(berth_payload, indent=2) + "\n")

    # 2. pinball_and_ladder_geometry.json
    pinball_data: dict[str, object] = {}
    for w_ord in (2, 3, 4):
        tri_p = build_pinball_triangulation(order_w=w_ord, s=400.0, h_b=100.0, h_c=140.0, params=cfg)
        n_uav = w_ord * (w_ord + 1) // 2
        up_centers = {str(f): list(circumcenter(*(tri_p.coord(v) for v in f))) for f in tri_p.faces[:n_uav]}
        dn_centers = {str(f): list(circumcenter(*(tri_p.coord(v) for v in f))) for f in tri_p.faces[n_uav:]}
        pinball_data[f"P_{n_uav}"] = {
            "triangulation": _serialize_triangulation(tri_p),
            "upward_circumcenters": up_centers,
            "inverted_circumcenters": dn_centers,
            "audit": audit_triangulation_dual_paths(tri_p, h_c=140.0, params=cfg),
        }

    y_tree = build_forked_y_tree_triangulation(s=400.0, h_b=100.0, h_c=140.0, params=cfg)
    ladder_300 = build_staggered_triangulation((0.0, 400.0), (tuple(range(1, 5)),), h1=300.0, dy=100.0, h_b=100.0, h_c=140.0, params=cfg)
    ladder_380_src = build_staggered_triangulation((0.0, 400.0, 800.0, 1200.0), ((1, 2, 3, 4), (5, 6), ()), h1=380.0, dy=90.0, h_b=100.0, h_c=140.0, params=cfg)
    ladder_380_dst = build_staggered_triangulation((0.0, 400.0, 800.0, 1200.0), ((1,), (4, 3, 2), (6, 5)), h1=380.0, dy=90.0, h_b=100.0, h_c=140.0, params=cfg)
    ladder_moves = plan_labelled_transfer(ladder_380_src, ladder_380_dst, h_b=100.0, h_c=140.0)
    c1_300 = staggered_circumcenter(0.0, 400.0, 300.0, 100.0, 1)
    c2_300 = staggered_circumcenter(0.0, 400.0, 300.0, 100.0, 2)
    c3_300 = staggered_circumcenter(0.0, 400.0, 300.0, 100.0, 3)

    pinball_ladder_payload = {
        "pinball": pinball_data,
        "forked_y_tree": {
            "triangulation": _serialize_triangulation(y_tree),
            "audit": audit_triangulation_dual_paths(y_tree, h_c=140.0, params=cfg),
        },
        "staggered_ladder": {
            "root_face_exit_example_h1_300_dy_100": {
                "triangulation": _serialize_triangulation(ladder_300),
                "circumcenters": {
                    "C1_raw": list(c1_300),
                    "C1_truncated_exit_at_hc_140": [285.0, 140.0],
                    "C2": list(c2_300),
                    "C3": list(c3_300),
                },
                "k3_segment_three_ref_min_eig": round(
                    segment_three_ref_min_eigenvalue(c3_300, c2_300, ((400.0, 0.0), (400.0, 400.0), (200.0, 300.0))),
                    6,
                ),
            },
            "symmetric_make_before_break_sample": {
                "source": _serialize_triangulation(ladder_380_src),
                "target": _serialize_triangulation(ladder_380_dst),
                "moves": _serialize_moves(ladder_moves),
            },
        },
    }
    pinball_file = out_path / "pinball_and_ladder_geometry.json"
    pinball_file.write_text(json.dumps(pinball_ladder_payload, indent=2) + "\n")

    # 3. multiscale_pipeline_timings.json
    bundle = generate_multiscale_scenarios(params=cfg)
    r0_sweep = 6700.0
    r1_sweep = 13400.0
    a_crit_factor = (r0_sweep * r1_sweep) / (r1_sweep - r0_sweep)
    speed_configs = [(4.0, 10.0), (10.0, 5.0)]
    timings_records: list[dict[str, object]] = []
    for (n_uav, m_bcn), sc in bundle.items():
        for v_c, t_set in speed_configs:
            d_pipe_val = 60.0
            ser = simulate_serial_execution(sc.source, sc.direct_matched_target, sc.direct_matched_moves, v_cruise=v_c, t_settle=t_set, dt=0.5)
            ser_rev = simulate_serial_execution(sc.source, sc.target, sc.reversible_moves, v_cruise=v_c, t_settle=t_set, dt=0.5)
            ser_del = simulate_serial_execution(sc.source, sc.direct_matched_target, sc.direct_matched_moves, v_cruise=v_c, t_settle=t_set, dt=0.5, seed=20261123, stall_prob=0.15)
            pip_det = simulate_pipelined_execution(sc.source, sc.direct_matched_target, sc.direct_matched_moves, d_pipe=d_pipe_val, v_cruise=v_c, t_settle=t_set, dt=0.5, stall_prob=0.0)
            pip_del = simulate_pipelined_execution(sc.source, sc.direct_matched_target, sc.direct_matched_moves, d_pipe=d_pipe_val, v_cruise=v_c, t_settle=t_set, dt=0.5, seed=20261123, stall_prob=0.15)
            timings_records.append(
                {
                    "scale_N": n_uav,
                    "beacons_M": m_bcn,
                    "v_cruise_mps": v_c,
                    "t_settle_s": t_set,
                    "d_pipe_m": d_pipe_val,
                    "stall_prob": 0.15,
                    "serial": {**ser, "a_crit_km2": round(a_crit_factor * float(ser["makespan_s"]) / 1e6, 3)},
                    "serial_reversible_berth": {**ser_rev, "a_crit_km2": round(a_crit_factor * float(ser_rev["makespan_s"]) / 1e6, 3)},
                    "serial_with_delays": {**ser_del, "a_crit_km2": round(a_crit_factor * float(ser_del["makespan_s"]) / 1e6, 3)},
                    "pipelined_deterministic": {
                        **pip_det,
                        "speedup_vs_serial": round(float(ser["makespan_s"]) / float(pip_det["makespan_s"]), 3),
                        "speedup_vs_reversible_serial": round(float(ser_rev["makespan_s"]) / float(pip_det["makespan_s"]), 3),
                        "a_crit_km2": round(a_crit_factor * float(pip_det["makespan_s"]) / 1e6, 3),
                    },
                    "pipelined_with_delays": {
                        **pip_del,
                        "speedup_vs_serial_delayed": round(float(ser_del["makespan_s"]) / float(pip_del["makespan_s"]), 3),
                        "speedup_vs_serial_det": round(float(ser["makespan_s"]) / float(pip_del["makespan_s"]), 3),
                        "a_crit_km2": round(a_crit_factor * float(pip_del["makespan_s"]) / 1e6, 3),
                    },
                }
            )
    # Include Pinball P_6 -> dual ladder (3, 0, 3) at 10 m/s
    p6_src = build_pinball_triangulation(order_w=3, s=400.0, h_b=100.0, h_c=140.0, params=cfg)
    dl6_dst = build_staggered_triangulation((0.0, 400.0, 800.0, 1200.0), ((1, 2, 3), (), (4, 5, 6)), h1=380.0, dy=90.0, h_b=100.0, h_c=140.0, params=cfg)
    p6_moves = plan_labelled_transfer(p6_src, dl6_dst, h_b=100.0, h_c=140.0)
    p6_ser = simulate_serial_execution(p6_src, dl6_dst, p6_moves, v_cruise=10.0, t_settle=5.0, dt=0.5)
    p6_pip = simulate_pipelined_execution(p6_src, dl6_dst, p6_moves, d_pipe=120.0, v_cruise=10.0, t_settle=5.0, dt=0.5, seed=7, stall_prob=0.1)

    timings_payload = {
        "a_crit_assumptions": {"r0_m2_per_s": r0_sweep, "r1_m2_per_s": r1_sweep, "v_axis_max_mps": cfg.v_axis_max},
        "multiscale": timings_records,
        "pinball_p6_to_dual_ladder_10mps": {
            "serial": p6_ser,
            "pipelined_with_delays": {
                **p6_pip,
                "speedup_vs_serial": round(float(p6_ser["makespan_s"]) / float(p6_pip["makespan_s"]), 3),
            },
        },
    }
    timings_file = out_path / "multiscale_pipeline_timings.json"
    timings_file.write_text(json.dumps(timings_payload, indent=2) + "\n")

    # 4. convex_domains.json (3 convex search domains + depth scaling curve under CBF2026 UWB)
    domains = standard_convex_search_domains()
    dom_recommendations = {
        "wide_shallow_coastal": (
            build_staggered_triangulation((0.0, 400.0, 800.0, 1200.0, 1600.0), ((1, 2, 3), (4, 5, 6), (7, 8, 9), (10, 11, 12)), h1=320.0, dy=80.0, h_b=100.0, h_c=140.0, params=cfg),
            build_staggered_triangulation((0.0, 400.0, 800.0, 1200.0, 1600.0), ((1, 2, 3, 4), (5, 6), (7, 8), (9, 10, 11, 12)), h1=320.0, dy=80.0, h_b=100.0, h_c=140.0, params=cfg),
        ),
        "narrow_deep_offshore": (
            build_staggered_triangulation((0.0, 400.0, 800.0, 1200.0), ((1, 2, 3, 4, 5, 6, 7), (), (8, 9, 10, 11, 12, 13, 14)), h1=380.0, dy=90.0, h_b=100.0, h_c=140.0, params=cfg),
            build_staggered_triangulation((0.0, 400.0, 800.0, 1200.0), ((), tuple(range(1, 15)), ()), h1=380.0, dy=90.0, h_b=100.0, h_c=140.0, params=cfg),
        ),
        "oblique_asymmetric": (
            build_pinball_triangulation(order_w=3, s=400.0, h_b=100.0, h_c=140.0, params=cfg),
            build_staggered_triangulation((0.0, 400.0, 800.0, 1200.0), ((), (1, 2, 3), (4, 5, 6)), h1=380.0, dy=90.0, h_b=100.0, h_c=140.0, params=cfg),
        ),
    }
    domains_payload = {
        "domains": [
            {
                "name": dom.name,
                "description": dom.description,
                "vertices": [list(v) for v in dom.vertices],
                "area_m2": dom.area_m2,
                "centroid": list(dom.centroid),
                "max_offshore_y_m": dom.max_offshore_y,
                "alongshore_span_m": dom.alongshore_span,
                "recommended_initial_formation": _serialize_triangulation(dom_recommendations[dom.name][0]),
                "recommended_target_formation": _serialize_triangulation(dom_recommendations[dom.name][1]),
            }
            for dom in domains
        ],
        "depth_scaling_uwb": _compute_depth_scaling_uwb(cfg),
    }
    domains_file = out_path / "convex_domains.json"
    domains_file.write_text(json.dumps(domains_payload, indent=2) + "\n")

    return {
        "berth_sweep": berth_file,
        "pinball_and_ladder_geometry": pinball_file,
        "multiscale_pipeline_timings": timings_file,
        "convex_domains": domains_file,
    }


if __name__ == "__main__":
    default_out = Path(__file__).resolve().parents[2] / "data" / "geo_outputs"
    paths = export_geo_outputs(default_out)
    for k, p in paths.items():
        print(f"{k}: {p}")
