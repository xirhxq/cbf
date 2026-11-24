"""Parametric anchor-rooted acute triangulation, dual-graph path generation,
Make-Before-Break symmetric handover verification, and symbolic checks.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from itertools import combinations
from typing import Iterable, Sequence

import numpy as np

DEFAULT_D_DEP = 850.0
DEFAULT_D_MAX = 1200.0
DEFAULT_D_R = 600.0
DEFAULT_D_SAFE = 10.0
DEFAULT_V_AXIS_MAX = 25.0


@dataclass(frozen=True)
class GeometryParams:
    """Explicit physical, safety, and speed bounds for formation geometry."""

    d_dep: float = DEFAULT_D_DEP
    d_max: float = DEFAULT_D_MAX
    d_r: float = DEFAULT_D_R
    d_safe: float = DEFAULT_D_SAFE
    d_clear: float = 40.0
    e_track: float = 0.0
    iota_req: float = 0.40
    berth_spacing: float = 50.0
    v_axis_max: float = DEFAULT_V_AXIS_MAX
    a_max: float | None = 4.0

    @property
    def max_allowed_reference(self) -> float:
        return self.d_dep - 2.0 * self.e_track


@dataclass(frozen=True)
class Leg:
    """Single target segment with primary parents, crossing auxiliary, and pre-request."""

    start: tuple[float, float]
    end: tuple[float, float]
    parents: tuple[object, object]
    auxiliary: object = None
    prerequest_next_parent: object = None
    kind: str = "root"


@dataclass(frozen=True)
class Move:
    """Sequence of legs executed by one aircraft."""

    node: int
    legs: tuple[Leg, ...]


@dataclass(frozen=True)
class AnchorRootedTriangulation:
    """Anchor-rooted acute planar triangulation complex with dual faces."""

    beacons: dict[object, tuple[float, float]]
    beacon_order: tuple[object, ...]
    positions: dict[int, tuple[float, float]]
    parents: dict[int, tuple[object, object]]
    faces: tuple[tuple[object, object, object], ...]
    root_faces: tuple[tuple[object, object, object], ...]
    support_face: dict[int, tuple[object, object, object]]
    h_b: float
    h_c: float
    params: GeometryParams
    unit_assignment: tuple[tuple[int, ...], ...] = ()
    ladder_params: tuple[tuple[float, float], ...] = ()

    def coord(self, vertex: object) -> tuple[float, float]:
        if vertex in self.positions:
            return self.positions[int(vertex)]
        return self.beacons[vertex]

    def all_coords(self) -> dict[object, tuple[float, float]]:
        out: dict[object, tuple[float, float]] = dict(self.beacons)
        out.update(self.positions)
        return out


def _det(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _point_segment_distance(
    p: tuple[float, float], a: tuple[float, float], b: tuple[float, float]
) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    denom = dx * dx + dy * dy
    if denom <= 0.0:
        return math.dist(p, a)
    u = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / denom))
    return math.dist(p, (a[0] + u * dx, a[1] + u * dy))


def circumcenter(
    a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]
) -> tuple[float, float]:
    """Exact circumcenter of three non-collinear 2D points."""
    ax, ay = a
    bx, by = b
    cx, cy = c
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        raise ValueError("Collinear triangle has no finite circumcenter")
    a2 = ax * ax + ay * ay
    b2 = bx * bx + by * by
    c2 = cx * cx + cy * cy
    ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d
    uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d
    return (ux, uy)


def triangle_max_angle_deg(
    a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]
) -> float:
    """Maximum interior angle (degrees) of triangle (a, b, c)."""
    ab2 = (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
    bc2 = (c[0] - b[0]) ** 2 + (c[1] - b[1]) ** 2
    ca2 = (a[0] - c[0]) ** 2 + (a[1] - c[1]) ** 2
    sides2 = sorted((ab2, bc2, ca2))
    if sides2[0] <= 0.0:
        raise ValueError("Degenerate triangle with zero-length edge")
    cos_max = (sides2[0] + sides2[1] - sides2[2]) / (2.0 * math.sqrt(sides2[0] * sides2[1]))
    cos_max = max(-1.0, min(1.0, cos_max))
    return math.degrees(math.acos(cos_max))


def min_berth_height(s_max: float, iota_req: float = 0.40) -> float:
    """Analytical lower bound on berth height guaranteeing cross-shore conditioning >= iota_req."""
    if not 0.0 < iota_req < 1.0:
        raise ValueError("iota_req must lie strictly in (0, 1)")
    return 0.5 * s_max * math.sqrt(iota_req / (2.0 - iota_req))


def worst_berth_eigenvalue(span: float, h_b: float) -> float:
    """Exact worst-case two-beacon geometric FIM minimum eigenvalue at interval midpoint."""
    half_s = 0.5 * span
    return (2.0 * h_b * h_b) / (half_s * half_s + h_b * h_b)


def check_staggered_acute_conditions(span: float, h1: float, dy: float) -> bool:
    """Return True iff staggered ladder parameters yield strictly acute faces."""
    return span > 0.0 and h1 > 0.5 * span and dy > 0.0 and (h1 * dy) < (0.25 * span * span)


def validate_staggered_acute_conditions(span: float, h1: float, dy: float) -> None:
    """Raise ValueError unless staggered ladder parameters are strictly acute."""
    if not check_staggered_acute_conditions(span, h1, dy):
        raise ValueError(
            f"Staggered ladder parameters (span={span}, h1={h1}, dy={dy}) violate strict acuteness: "
            f"require h1 > span/2 ({0.5 * span}) and h1*dy < span^2/4 ({0.25 * span * span})"
        )


def staggered_circumcenter(x_left: float, span: float, h1: float, dy: float, k: int) -> tuple[float, float]:
    """Closed-form circumcenter C_{r,k} for a staggered ladder on [x_left, x_left + span]."""
    validate_staggered_acute_conditions(span, h1, dy)
    if k < 1:
        raise ValueError("Face index k must be >= 1")
    if k == 1:
        return (x_left + 0.5 * span, (h1 * h1 - 0.25 * span * span) / (2.0 * h1))
    if k == 2:
        return (x_left + 0.75 * span + (h1 * dy) / span, 0.5 * (h1 + dy))
    return (x_left + 0.75 * span + ((-1) ** k) * (dy * dy) / span, h1 + (k - 2) * dy)


def check_coastal_three_beacon_coverage(
    beacon_xs: Sequence[float], h_c: float, d_dep: float, e_track: float = 0.0
) -> bool:
    """Verify Proposition 2.3 three-beacon coastal corridor coverage condition for M >= 3."""
    if len(beacon_xs) < 2 or any(beacon_xs[i] >= beacon_xs[i + 1] for i in range(len(beacon_xs) - 1)):
        return False
    if h_c >= d_dep - e_track:
        return False
    if len(beacon_xs) == 2:
        return math.hypot(beacon_xs[1] - beacon_xs[0], h_c) < d_dep - e_track
    r_c = math.sqrt((d_dep - e_track) ** 2 - h_c * h_c)
    spans = [beacon_xs[i + 1] - beacon_xs[i] for i in range(len(beacon_xs) - 1)]
    m_int = len(spans)
    lhs = max(max(spans), 0.5 * spans[0] + spans[1], 0.5 * spans[-1] + spans[-2])
    for r in range(1, m_int - 1):
        lhs = max(lhs, min(0.5 * spans[r] + spans[r - 1], 0.5 * spans[r] + spans[r + 1]))
    return lhs <= r_c + 1e-9


def berth_capacity_1d(
    beacon_xs: Sequence[float],
    berth_spacing: float,
    d_dep: float,
    h_c: float,
    e_track: float = 0.0,
    inner_corridor: bool = True,
) -> int:
    """Maximum single-row berth capacity along the coastal corridor."""
    spans = [beacon_xs[i + 1] - beacon_xs[i] for i in range(len(beacon_xs) - 1)]
    if math.hypot(max(spans), h_c) >= d_dep - e_track:
        return 0
    total = beacon_xs[-1] - beacon_xs[0]
    if inner_corridor:
        total -= 0.5 * (spans[0] + spans[-1])
    if total < 0.0:
        return 0
    return int(math.floor(total / berth_spacing)) + 1


def _validate_vertical_layering(
    s_max: float, h_b: float, h_c: float, h_min: float, params: GeometryParams
) -> None:
    hb_floor = max(params.d_clear, min_berth_height(s_max, params.iota_req))
    if h_b < hb_floor - 1e-9:
        raise ValueError(
            f"Berth height h_b={h_b} violates conditioning/clearance bound {hb_floor:.3f}"
        )
    if h_b > h_c - params.d_clear + 1e-9:
        raise ValueError(
            f"Berth height h_b={h_b} must leave >= d_clear={params.d_clear} below corridor h_c={h_c}"
        )
    y_mid_min = 0.5 * h_min
    upper = min(y_mid_min, h_min - params.d_clear)
    if h_c >= upper - 1e-9:
        raise ValueError(
            f"Corridor h_c={h_c} must satisfy h_c < min(0.5*h_min, h_min - d_clear) = {upper:.3f}"
        )


def _closest_two_and_third(
    x: float, y: float, beacons: dict[object, tuple[float, float]], beacon_order: Sequence[object]
) -> tuple[tuple[object, object], object | None]:
    ranked = sorted(
        beacon_order,
        key=lambda lbl: (math.dist((x, y), beacons[lbl]), str(lbl)),
    )
    return (ranked[0], ranked[1]), (ranked[2] if len(ranked) >= 3 else None)


def default_berth_layout(
    beacon_xs: Sequence[float],
    beacon_labels: Sequence[object],
    node_ids: Sequence[int],
    h_b: float,
    berth_spacing: float,
) -> tuple[dict[int, tuple[float, float]], dict[int, tuple[object, object]]]:
    """Place unique physical-label berths centered inside the inner coastal corridor."""
    n = len(node_ids)
    if n == 0:
        return {}, {}
    beacons = {lbl: (float(x), 0.0) for lbl, x in zip(beacon_labels, beacon_xs)}
    x_left = 0.5 * (beacon_xs[0] + beacon_xs[1])
    x_right = 0.5 * (beacon_xs[-2] + beacon_xs[-1])
    width = x_right - x_left
    needed = (n - 1) * berth_spacing
    if needed > width + 1e-9:
        # Fallback to full [x_0, x_{M-1}] span if inner corridor cannot hold N berths
        x_left, x_right = beacon_xs[0], beacon_xs[-1]
        width = x_right - x_left
        if needed > width + 1e-9:
            raise ValueError(f"Coast span {width} cannot accommodate {n} berths at spacing {berth_spacing}")
    x_start = 0.5 * (x_left + x_right) - 0.5 * needed
    sorted_ids = sorted(node_ids)
    berth_pos: dict[int, tuple[float, float]] = {}
    berth_parents: dict[int, tuple[object, object]] = {}
    for idx, nid in enumerate(sorted_ids):
        bx = x_start + idx * berth_spacing
        berth_pos[nid] = (bx, float(h_b))
        parents, _ = _closest_two_and_third(bx, float(h_b), beacons, beacon_labels)
        berth_parents[nid] = parents
    return berth_pos, berth_parents


def build_staggered_triangulation(
    beacon_xs: Sequence[float],
    units: Sequence[Sequence[int]],
    h1: float | Sequence[float],
    dy: float | Sequence[float],
    h_b: float = 100.0,
    h_c: float = 140.0,
    params: GeometryParams | None = None,
    beacon_labels: Sequence[object] | None = None,
) -> AnchorRootedTriangulation:
    """Construct a parametric non-equidistant M-beacon staggered single/multi-ladder."""
    cfg = params or GeometryParams()
    m_bcn = len(beacon_xs)
    if m_bcn < 2:
        raise ValueError("At least 2 coastal beacons are required")
    if any(beacon_xs[i] >= beacon_xs[i + 1] for i in range(m_bcn - 1)):
        raise ValueError("Beacon x-coordinates must be strictly increasing")
    m_int = m_bcn - 1
    if len(units) != m_int:
        raise ValueError(f"Expected {m_int} interval unit lists, got {len(units)}")
    labels = [nid for unit in units for nid in unit]
    if labels and sorted(labels) != list(range(1, len(labels) + 1)):
        raise ValueError("Units must partition physical labels 1..N")

    b_labels = tuple(beacon_labels) if beacon_labels is not None else tuple(f"b{i}" for i in range(m_bcn))
    if len(b_labels) != m_bcn:
        raise ValueError("beacon_labels length must match beacon_xs")
    beacons = {lbl: (float(x), 0.0) for lbl, x in zip(b_labels, beacon_xs)}

    h1_seq = [float(h1)] * m_int if isinstance(h1, (int, float)) else [float(v) for v in h1]
    dy_seq = [float(dy)] * m_int if isinstance(dy, (int, float)) else [float(v) for v in dy]
    if len(h1_seq) != m_int or len(dy_seq) != m_int:
        raise ValueError("h1 and dy lengths must match number of coastal intervals")

    spans = [beacon_xs[r + 1] - beacon_xs[r] for r in range(m_int)]
    for r in range(m_int):
        validate_staggered_acute_conditions(spans[r], h1_seq[r], dy_seq[r])

    if not check_coastal_three_beacon_coverage(beacon_xs, h_c, cfg.d_dep, cfg.e_track):
        raise ValueError("Coastal three-beacon coverage condition violated")

    _validate_vertical_layering(max(spans), h_b, h_c, min(h1_seq), cfg)

    positions: dict[int, tuple[float, float]] = {}
    parents: dict[int, tuple[object, object]] = {}
    faces: list[tuple[object, object, object]] = []
    root_faces: list[tuple[object, object, object]] = []
    support_face: dict[int, tuple[object, object, object]] = {}

    for r, unit in enumerate(units):
        chain: list[object] = [b_labels[r], b_labels[r + 1]] + list(unit)
        for j, nid in enumerate(unit, 1):
            x = beacon_xs[r] + (0.5 * spans[r] if j % 2 else spans[r])
            y = h1_seq[r] + (j - 1) * dy_seq[r]
            positions[nid] = (float(x), float(y))
            p_pair = (chain[j - 1], chain[j])
            parents[nid] = p_pair
            face = (chain[j - 1], chain[j], nid)
            faces.append(face)
            support_face[nid] = face
            if j == 1:
                root_faces.append(face)

    return AnchorRootedTriangulation(
        beacons=beacons,
        beacon_order=b_labels,
        positions=positions,
        parents=parents,
        faces=tuple(faces),
        root_faces=tuple(root_faces),
        support_face=support_face,
        h_b=float(h_b),
        h_c=float(h_c),
        params=cfg,
        unit_assignment=tuple(tuple(u) for u in units),
        ladder_params=tuple(zip(h1_seq, dy_seq)),
    )


def build_pinball_triangulation(
    order_w: int,
    s: float = 400.0,
    h_b: float = 100.0,
    h_c: float = 140.0,
    params: GeometryParams | None = None,
    x_offset: float = 0.0,
    beacon_labels: Sequence[object] | None = None,
) -> AnchorRootedTriangulation:
    """Construct Pinball P_W (N = W*(W+1)/2) with upward and inverted acute faces."""
    if order_w < 2:
        raise ValueError("Pinball order W must be >= 2")
    cfg = params or GeometryParams()
    beacon_xs = [x_offset + i * s for i in range(order_w + 1)]
    b_labels = tuple(beacon_labels) if beacon_labels is not None else tuple(f"b{i}" for i in range(order_w + 1))
    beacons = {lbl: (float(x), 0.0) for lbl, x in zip(b_labels, beacon_xs)}
    h_eq = 0.5 * math.sqrt(3.0) * s
    _validate_vertical_layering(s, h_b, h_c, h_eq, cfg)

    grid_ids: dict[tuple[int, int], object] = {}
    for c in range(order_w + 1):
        grid_ids[(0, c)] = b_labels[c]

    positions: dict[int, tuple[float, float]] = {}
    parents: dict[int, tuple[object, object]] = {}
    support_face: dict[int, tuple[object, object, object]] = {}
    faces: list[tuple[object, object, object]] = []
    root_faces: list[tuple[object, object, object]] = []

    next_id = 1
    for r in range(1, order_w + 1):
        for c in range(order_w + 1 - r):
            nid = next_id
            next_id += 1
            grid_ids[(r, c)] = nid
            positions[nid] = (x_offset + (c + 0.5 * r) * s, r * h_eq)
            p1, p2 = grid_ids[(r - 1, c)], grid_ids[(r - 1, c + 1)]
            parents[nid] = (p1, p2)
            face = (p1, p2, nid)
            faces.append(face)
            support_face[nid] = face
            if r == 1:
                root_faces.append(face)

    # Inverted acute equilateral faces filling shore-side gaps between converging branches
    for r in range(1, order_w):
        for c in range(order_w - r):
            faces.append((grid_ids[(r, c)], grid_ids[(r - 1, c + 1)], grid_ids[(r, c + 1)]))

    return AnchorRootedTriangulation(
        beacons=beacons,
        beacon_order=b_labels,
        positions=positions,
        parents=parents,
        faces=tuple(faces),
        root_faces=tuple(root_faces),
        support_face=support_face,
        h_b=float(h_b),
        h_c=float(h_c),
        params=cfg,
    )


def build_forked_y_tree_triangulation(
    s: float = 400.0,
    h_b: float = 100.0,
    h_c: float = 140.0,
    params: GeometryParams | None = None,
    x_offset: float = 0.0,
) -> AnchorRootedTriangulation:
    """Construct the 7-aircraft equilateral lattice forked Y-branch tree."""
    cfg = params or GeometryParams()
    beacon_xs = [x_offset + i * s for i in range(4)]
    b_labels = ("b0", "b1", "b2", "b3")
    beacons = {lbl: (float(x), 0.0) for lbl, x in zip(b_labels, beacon_xs)}
    h_eq = 0.5 * math.sqrt(3.0) * s
    _validate_vertical_layering(s, h_b, h_c, h_eq, cfg)

    positions = {
        1: (x_offset + 1.5 * s, h_eq),
        2: (x_offset + 0.5 * s, h_eq),
        3: (x_offset + 1.0 * s, 2.0 * h_eq),
        4: (x_offset + 0.0 * s, 2.0 * h_eq),
        5: (x_offset + 0.5 * s, 3.0 * h_eq),
        6: (x_offset + 2.0 * s, 2.0 * h_eq),
        7: (x_offset + 1.5 * s, 3.0 * h_eq),
    }
    parents = {
        1: ("b1", "b2"),
        2: ("b1", 1),
        3: (2, 1),
        4: (2, 3),
        5: (4, 3),
        6: (3, 1),
        7: (3, 6),
    }
    support_face = {nid: (p[0], p[1], nid) for nid, p in parents.items()}
    faces = tuple(support_face[nid] for nid in sorted(positions))
    return AnchorRootedTriangulation(
        beacons=beacons,
        beacon_order=b_labels,
        positions=positions,
        parents=parents,
        faces=faces,
        root_faces=(support_face[1],),
        support_face=support_face,
        h_b=float(h_b),
        h_c=float(h_c),
        params=cfg,
    )


def _descendants(parents: dict[int, tuple[object, object]], node: int) -> set[int]:
    children: dict[object, list[int]] = {}
    for u, ps in parents.items():
        for p in ps:
            children.setdefault(p, []).append(u)
    seen: set[int] = set()
    queue = deque([node])
    while queue:
        cur = queue.popleft()
        for ch in children.get(cur, ()):
            if ch not in seen:
                seen.add(ch)
                queue.append(ch)
    return seen


def _face_circumcenter(tri: AnchorRootedTriangulation, face: tuple[object, object, object]) -> tuple[float, float]:
    return circumcenter(*(tri.coord(v) for v in face))


def _dual_face_path(
    tri: AnchorRootedTriangulation, node: int
) -> list[tuple[object, object, object]]:
    start_face = tri.support_face[node]
    if start_face in tri.root_faces:
        return [start_face]
    forbidden = _descendants(tri.parents, node) | {node}
    face_sets = {f: set(f) for f in tri.faces}
    face_centers = {f: _face_circumcenter(tri, f) for f in tri.faces}

    adj: dict[tuple[object, object, object], list[tuple[object, object, object]]] = {f: [] for f in tri.faces}
    for fa, fb in combinations(tri.faces, 2):
        if len(face_sets[fa] & face_sets[fb]) == 2:
            adj[fa].append(fb)
            adj[fb].append(fa)

    for f in adj:
        adj[f].sort(key=lambda nb: (face_centers[nb][1], face_centers[nb][0]))

    root_set = set(tri.root_faces)
    queue = deque([(start_face, [start_face])])
    visited = {start_face}
    while queue:
        cur, path = queue.popleft()
        for nxt in adj[cur]:
            if nxt in visited or (face_sets[nxt] & forbidden):
                continue
            new_path = path + [nxt]
            if nxt in root_set:
                return new_path
            visited.add(nxt)
            queue.append((nxt, new_path))
    raise ValueError(f"No legal non-descendant dual path to shore for node {node}")


def _corridor_cuts(x_start: float, x_end: float, beacon_xs: Sequence[float]) -> list[float]:
    low, high = sorted((x_start, x_end))
    cuts_set: set[float] = set()
    for i in range(len(beacon_xs) - 2):
        mid_skip = 0.5 * (beacon_xs[i] + beacon_xs[i + 2])
        if low + 1e-9 < mid_skip < high - 1e-9:
            cuts_set.add(mid_skip)
    cuts = sorted(cuts_set)
    return (cuts if x_start < x_end else cuts[::-1]) + [x_end]


def _attach_symmetric_prerequests(legs: list[Leg]) -> tuple[Leg, ...]:
    out: list[Leg] = []
    for idx, leg in enumerate(legs):
        prereq = leg.prerequest_next_parent
        if idx + 1 < len(legs):
            nxt_parents = legs[idx + 1].parents
            incoming = [p for p in nxt_parents if p not in leg.parents]
            if len(incoming) > 1:
                raise ValueError(f"Handover changed more than one parent: {leg.parents} -> {nxt_parents}")
            if incoming:
                prereq = incoming[0]
        out.append(
            Leg(
                start=leg.start,
                end=leg.end,
                parents=leg.parents,
                auxiliary=leg.auxiliary,
                prerequest_next_parent=prereq,
                kind=leg.kind,
            )
        )
    return tuple(out)


def _dual_spine_to_port_legs(
    tri: AnchorRootedTriangulation, node: int, h_c: float
) -> tuple[list[Leg], tuple[float, float], tuple[object, object]]:
    """Build dual withdrawal legs from node's slot down to its root-face port on y=h_c."""
    face_path = _dual_face_path(tri, node)
    here = tri.positions[node]
    legs: list[Leg] = []

    def append_leg(
        end: tuple[float, float],
        parents: tuple[object, object],
        auxiliary: object = None,
        kind: str = "root",
    ) -> None:
        nonlocal here
        if math.dist(here, end) > 1e-9:
            legs.append(Leg(start=here, end=end, parents=parents, auxiliary=auxiliary, kind=kind))
            here = end

    if len(face_path) >= 2:
        f_top = face_path[0]
        f_next = face_path[1]
        aux_top = next(v for v in f_next if v not in f_top)
        append_leg(_face_circumcenter(tri, f_top), tri.parents[node], auxiliary=aux_top, kind="tip")

        for idx in range(len(face_path) - 1):
            f_cur = face_path[idx]
            f_nxt = face_path[idx + 1]
            shared = tuple(v for v in f_cur if v in f_nxt)
            aux = next(v for v in f_nxt if v not in shared)
            target_pt = _face_circumcenter(tri, f_nxt)
            if idx == len(face_path) - 2 and target_pt[1] < h_c:
                # Truncate root-face perpendicular bisector segment at coastal corridor y = h_c
                y_a = here[1]
                y_b = target_pt[1]
                if y_a <= h_c:
                    raise ValueError(f"Expected entry into root face above h_c={h_c}, got {y_a}")
                frac = (h_c - y_a) / (y_b - y_a)
                target_pt = (here[0] + frac * (target_pt[0] - here[0]), float(h_c))
            append_leg(target_pt, (shared[0], shared[1]), auxiliary=aux, kind="bisector")

    root_face = face_path[-1]
    root_beacons = sorted(
        (v for v in root_face if v in tri.beacons),
        key=lambda lbl: tri.beacons[lbl][0],
    )
    if len(root_beacons) != 2:
        raise ValueError(f"Root face {root_face} must contain exactly two coastal beacons")
    b_left, b_right = root_beacons[0], root_beacons[1]
    port = (0.5 * (tri.beacons[b_left][0] + tri.beacons[b_right][0]), float(h_c))
    append_leg(port, (b_left, b_right), auxiliary=None, kind="root")
    return legs, port, (b_left, b_right)


def _coast_legs(
    x_start: float,
    x_end: float,
    h_c: float,
    tri: AnchorRootedTriangulation,
) -> list[Leg]:
    beacon_xs = [tri.beacons[b][0] for b in tri.beacon_order]
    legs: list[Leg] = []
    cur_x = x_start
    for nxt_x in _corridor_cuts(x_start, x_end, beacon_xs):
        if abs(nxt_x - cur_x) <= 1e-9:
            continue
        mid_x = 0.5 * (cur_x + nxt_x)
        parents, aux = _closest_two_and_third(mid_x, h_c, tri.beacons, tri.beacon_order)
        legs.append(
            Leg(
                start=(cur_x, float(h_c)),
                end=(nxt_x, float(h_c)),
                parents=parents,
                auxiliary=aux,
                kind="coast",
            )
        )
        cur_x = nxt_x
    return legs


def dual_withdrawal_path(
    tri: AnchorRootedTriangulation,
    node: int,
    h_c: float,
    h_b: float,
    berth_pos: tuple[float, float],
    berth_parents: tuple[object, object],
) -> Move:
    """Build complete dual-graph withdrawal Move from deployed slot to berth."""
    legs, port, _ = _dual_spine_to_port_legs(tri, node, h_c)
    legs.extend(_coast_legs(port[0], berth_pos[0], h_c, tri))
    _, dock_aux = _closest_two_and_third(berth_pos[0], 0.5 * (h_c + h_b), tri.beacons, tri.beacon_order)
    if math.dist((berth_pos[0], float(h_c)), berth_pos) > 1e-9:
        legs.append(
            Leg(
                start=(berth_pos[0], float(h_c)),
                end=berth_pos,
                parents=berth_parents,
                auxiliary=dock_aux,
                kind="dock",
            )
        )
    return Move(node=node, legs=_attach_symmetric_prerequests(legs))


def _invert_withdrawal_to_installation(wd_move: Move) -> Move:
    """Convert a node's withdrawal Move into an installation Move with symmetric pre-requests."""
    rev_legs = [
        Leg(
            start=leg.end,
            end=leg.start,
            parents=leg.parents,
            auxiliary=leg.auxiliary,
            prerequest_next_parent=None,
            kind=leg.kind,
        )
        for leg in reversed(wd_move.legs)
    ]
    return Move(node=wd_move.node, legs=_attach_symmetric_prerequests(rev_legs))


def _topological_order(parents: dict[int, tuple[object, object]], beacons: dict[object, tuple[float, float]]) -> list[int]:
    children: dict[int, list[int]] = {u: [] for u in parents}
    indeg: dict[int, int] = {}
    for u, ps in parents.items():
        non_beacon = [p for p in ps if p not in beacons]
        indeg[u] = len(non_beacon)
        for p in non_beacon:
            children[int(p)].append(u)
    q = deque(sorted(u for u, d in indeg.items() if d == 0))
    order: list[int] = []
    while q:
        u = q.popleft()
        order.append(u)
        for ch in sorted(children[u]):
            indeg[ch] -= 1
            if indeg[ch] == 0:
                q.append(ch)
    if len(order) != len(parents):
        raise ValueError("Parent graph contains a cycle")
    return order


def maximal_shared_prefix(
    src: AnchorRootedTriangulation, dst: AnchorRootedTriangulation
) -> set[int]:
    """Maximal anchor-rooted closed subset with identical coordinates and physical parents."""
    topo = _topological_order(src.parents, src.beacons)
    prefix: set[int] = set()
    for u in topo:
        if u not in dst.positions:
            continue
        if math.dist(src.positions[u], dst.positions[u]) > 1e-6:
            continue
        if set(src.parents[u]) != set(dst.parents[u]):
            continue
        if all(p in src.beacons or int(p) in prefix for p in src.parents[u]):
            prefix.add(u)
    return prefix


def plan_labelled_transfer(
    source_tri: AnchorRootedTriangulation,
    target_tri: AnchorRootedTriangulation,
    h_b: float = 100.0,
    h_c: float = 140.0,
    berth_positions: dict[int, tuple[float, float]] | None = None,
) -> list[Move]:
    """Canonical reversible berth-mediated transfer preserving arbitrary physical labels."""
    if source_tri.beacons != target_tri.beacons:
        raise ValueError("Source and target must share identical coastal beacons")
    if set(source_tri.positions) != set(target_tri.positions):
        raise ValueError("Source and target must share the same physical aircraft set")

    prefix = maximal_shared_prefix(source_tri, target_tri)
    moving_ids = sorted(set(source_tri.positions) - prefix)
    if not moving_ids:
        return []

    beacon_xs = [source_tri.beacons[b][0] for b in source_tri.beacon_order]
    if berth_positions is None:
        b_pos, b_par = default_berth_layout(
            beacon_xs, source_tri.beacon_order, moving_ids, h_b, source_tri.params.berth_spacing
        )
    else:
        b_pos = berth_positions
        b_par = {
            nid: _closest_two_and_third(b_pos[nid][0], b_pos[nid][1], source_tri.beacons, source_tri.beacon_order)[0]
            for nid in moving_ids
        }

    src_topo = _topological_order(source_tri.parents, source_tri.beacons)
    dst_topo = _topological_order(target_tri.parents, target_tri.beacons)

    wd_moves = [
        dual_withdrawal_path(source_tri, nid, h_c, h_b, b_pos[nid], b_par[nid])
        for nid in reversed(src_topo)
        if nid not in prefix
    ]
    inst_moves = [
        _invert_withdrawal_to_installation(
            dual_withdrawal_path(target_tri, nid, h_c, h_b, b_pos[nid], b_par[nid])
        )
        for nid in dst_topo
        if nid not in prefix
    ]
    return wd_moves + inst_moves


def _relabel_triangulation(
    tri: AnchorRootedTriangulation, mapping: dict[int, int]
) -> AnchorRootedTriangulation:
    """Relabel aircraft nodes according to bijection old_id -> new_id."""
    def map_v(v: object) -> object:
        return mapping[int(v)] if v in tri.positions else v

    new_positions = {mapping[k]: pos for k, pos in tri.positions.items()}
    new_parents = {mapping[k]: (map_v(p[0]), map_v(p[1])) for k, p in tri.parents.items()}
    new_faces = tuple((map_v(f[0]), map_v(f[1]), map_v(f[2])) for f in tri.faces)
    new_root = tuple((map_v(f[0]), map_v(f[1]), map_v(f[2])) for f in tri.root_faces)
    new_sup = {mapping[k]: (map_v(f[0]), map_v(f[1]), map_v(f[2])) for k, f in tri.support_face.items()}
    new_units = tuple(tuple(mapping[k] for k in u) for u in tri.unit_assignment)
    return AnchorRootedTriangulation(
        beacons=tri.beacons,
        beacon_order=tri.beacon_order,
        positions=new_positions,
        parents=new_parents,
        faces=new_faces,
        root_faces=new_root,
        support_face=new_sup,
        h_b=tri.h_b,
        h_c=tri.h_c,
        params=tri.params,
        unit_assignment=new_units,
        ladder_params=tri.ladder_params,
    )


def _hungarian_min_assignment(cost: np.ndarray) -> tuple[list[int], list[int]]:
    """Exact O(n^3) Hungarian algorithm for square minimum-weight bipartite assignment."""
    n = cost.shape[0]
    u = np.zeros(n + 1, dtype=float)
    v = np.zeros(n + 1, dtype=float)
    p = [0] * (n + 1)
    way = [0] * (n + 1)
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = [math.inf] * (n + 1)
        used = [False] * (n + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = math.inf
            j1 = 0
            for j in range(1, n + 1):
                if not used[j]:
                    cur = float(cost[i0 - 1, j - 1]) - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j
            for j in range(n + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    row_ind = [0] * n
    col_ind = [0] * n
    for j in range(1, n + 1):
        row_ind[p[j] - 1] = p[j] - 1
        col_ind[p[j] - 1] = j - 1
    return row_ind, col_ind


def plan_direct_matched_transfer(
    source_tri: AnchorRootedTriangulation,
    target_tri: AnchorRootedTriangulation,
    h_c: float = 140.0,
) -> tuple[list[Move], AnchorRootedTriangulation]:
    """Corollary 1a: optimal role matching + direct coastal transfer skipping berths when disjoint.

    Returns (moves, matched_target_triangulation) where matched_target_triangulation has the
    physical aircraft IDs relabeled to match the optimal bijection.
    """
    if source_tri.beacons != target_tri.beacons:
        raise ValueError("Source and target must share identical coastal beacons")
    if len(source_tri.positions) != len(target_tri.positions):
        raise ValueError("Source and target must have the same number of aircraft")

    # 1. Greedy maximal geometric prefix matching (identical position + matched parent positions)
    src_topo = _topological_order(source_tri.parents, source_tri.beacons)
    dst_topo = _topological_order(target_tri.parents, target_tri.beacons)
    dst_by_pos = {tuple(round(v, 5) for v in target_tri.positions[d]): d for d in dst_topo}
    src_to_dst: dict[int, int] = {}
    used_dst: set[int] = set()
    for s_id in src_topo:
        key = tuple(round(v, 5) for v in source_tri.positions[s_id])
        if key not in dst_by_pos:
            continue
        d_id = dst_by_pos[key]
        if d_id in used_dst:
            continue
        s_ps = source_tri.parents[s_id]
        d_ps = target_tri.parents[d_id]
        # Check whether parents match (either same beacon or already identically mapped prefix nodes)
        s_mapped_ps = {src_to_dst[int(p)] if p in src_to_dst else p for p in s_ps}
        if all(p in source_tri.beacons or int(p) in src_to_dst for p in s_ps) and s_mapped_ps == set(d_ps):
            src_to_dst[s_id] = d_id
            used_dst.add(d_id)

    rem_src = [u for u in reversed(src_topo) if u not in src_to_dst]
    rem_dst = [u for u in dst_topo if u not in used_dst]
    if not rem_src:
        relabeled_dst = _relabel_triangulation(target_tri, {d: s for s, d in src_to_dst.items()})
        return [], relabeled_dst

    # Group remaining source nodes by source root interval (preserving per-interval withdrawal order)
    def root_interval(tri: AnchorRootedTriangulation, nid: int) -> tuple[object, object]:
        face_path = _dual_face_path(tri, nid)
        rb = sorted((v for v in face_path[-1] if v in tri.beacons), key=lambda b: tri.beacons[b][0])
        return (rb[0], rb[1])

    rem_src.sort(key=lambda u: source_tri.beacons[root_interval(source_tri, u)[0]][0])

    src_intervals = {root_interval(source_tri, u) for u in rem_src}
    dst_intervals = {root_interval(target_tri, u) for u in rem_dst}
    disjoint = src_intervals.isdisjoint(dst_intervals)

    # Precompute spine legs & ports for bipartite cost matrix
    src_spines = {u: _dual_spine_to_port_legs(source_tri, u, h_c) for u in rem_src}
    dst_spines = {v: _dual_spine_to_port_legs(target_tri, v, h_c) for v in rem_dst}

    def path_len(legs: Sequence[Leg]) -> float:
        return sum(math.dist(leg.start, leg.end) for leg in legs)

    m = len(rem_src)
    cost = np.zeros((m, m), dtype=float)
    for i, u in enumerate(rem_src):
        legs_u, port_u, _ = src_spines[u]
        len_u = path_len(legs_u)
        for j, v in enumerate(rem_dst):
            legs_v, port_v, _ = dst_spines[v]
            len_v = path_len(legs_v)
            # Tiny FIFO tie-breaker encourages earlier-withdrawing outer leaves -> earlier-installing inner roots
            fifo_bias = 1e-4 * ((i - j) ** 2)
            cost[i, j] = len_u + abs(port_u[0] - port_v[0]) + len_v + fifo_bias

    row_ind, col_ind = _hungarian_min_assignment(cost)
    dst_to_src = {d: s for s, d in src_to_dst.items()}
    pairs: list[tuple[int, int]] = []
    for r_idx, c_idx in zip(row_ind, col_ind):
        u = rem_src[r_idx]
        v = rem_dst[c_idx]
        dst_to_src[v] = u
        pairs.append((u, v))

    relabeled_dst = _relabel_triangulation(target_tri, dst_to_src)

    if not disjoint:
        return plan_labelled_transfer(source_tri, relabeled_dst, h_b=source_tri.h_b, h_c=h_c), relabeled_dst

    # Build direct inter-interval moves (one Move per aircraft, ordered by withdrawal precedence)
    moves: list[Move] = []
    for u, _ in sorted(pairs, key=lambda pair: rem_src.index(pair[0])):
        wd_legs, port_src, _ = src_spines[u]
        dst_wd_legs, port_dst, _ = _dual_spine_to_port_legs(relabeled_dst, u, h_c)
        coast = _coast_legs(port_src[0], port_dst[0], h_c, source_tri)
        inst_legs = [
            Leg(start=leg.end, end=leg.start, parents=leg.parents, auxiliary=leg.auxiliary, kind=leg.kind)
            for leg in reversed(dst_wd_legs)
        ]
        full_legs = _attach_symmetric_prerequests(list(wd_legs) + coast + inst_legs)
        moves.append(Move(node=u, legs=full_legs))
    return moves, relabeled_dst


def segment_three_ref_min_eigenvalue(
    start: tuple[float, float],
    end: tuple[float, float],
    refs: Sequence[tuple[float, float]],
    weights: Sequence[float] | None = None,
    samples: int = 101,
) -> float:
    """Minimum smallest eigenvalue of the 2x2 geometric/weighted FIM along [start, end]."""
    if len(refs) < 2:
        raise ValueError("Need at least two references")
    ws = [1.0] * len(refs) if weights is None else list(weights)
    min_eig = math.inf
    for k in range(samples):
        t = k / float(max(1, samples - 1))
        pt = (start[0] + t * (end[0] - start[0]), start[1] + t * (end[1] - start[1]))
        fim = np.zeros((2, 2), dtype=float)
        for r_pt, w in zip(refs, ws):
            dx, dy = pt[0] - r_pt[0], pt[1] - r_pt[1]
            norm = math.hypot(dx, dy)
            if norm < 1e-12:
                return 0.0
            n = np.array([dx / norm, dy / norm], dtype=float)
            fim += w * np.outer(n, n)
        eig = float(np.linalg.eigvalsh(fim)[0])
        min_eig = min(min_eig, eig)
    return min_eig


def audit_triangulation_dual_paths(
    tri: AnchorRootedTriangulation,
    h_c: float | None = None,
    params: GeometryParams | None = None,
) -> dict[str, float | int]:
    """Audit all dual withdrawal paths of an anchor-rooted acute triangulation."""
    cfg = params or tri.params
    corridor_y = tri.h_c if h_c is None else h_c
    all_pts = tri.all_coords()

    total_crossings = 0
    min_angle = 90.0
    min_three_ref = math.inf
    max_ref = 0.0
    min_clear = math.inf

    for nid in sorted(tri.positions):
        legs, _, _ = _dual_spine_to_port_legs(tri, nid, corridor_y)
        fixed = {k: v for k, v in all_pts.items() if k != nid}
        for leg in legs:
            a, b = leg.start, leg.end
            refs = [fixed[r] for r in leg.parents]
            if leg.auxiliary is not None:
                refs.append(fixed[leg.auxiliary])
            for pt in (a, b):
                for r_pt in refs:
                    max_ref = max(max_ref, math.dist(pt, r_pt))
            for k, pt in fixed.items():
                min_clear = min(min_clear, _point_segment_distance(pt, a, b))
            if leg.kind == "bisector":
                total_crossings += 1
                p1, p2 = fixed[leg.parents[0]], fixed[leg.parents[1]]
                p3 = fixed[leg.auxiliary]
                mid = (0.5 * (p1[0] + p2[0]), 0.5 * (p1[1] + p2[1]))
                e_vec = np.array([p2[0] - p1[0], p2[1] - p1[1]], dtype=float)
                a_vec = np.array([mid[0] - p3[0], mid[1] - p3[1]], dtype=float)
                cos_t = abs(float(np.dot(e_vec, a_vec))) / (
                    float(np.linalg.norm(e_vec)) * float(np.linalg.norm(a_vec))
                )
                min_angle = min(min_angle, math.degrees(math.acos(min(1.0, max(0.0, cos_t)))))
                min_three_ref = min(min_three_ref, segment_three_ref_min_eigenvalue(a, b, (p1, p2, p3)))

    if min_three_ref is math.inf:
        min_three_ref = 1.0
    return {
        "total_crossings": total_crossings,
        "min_crossing_angle_deg": min_angle,
        "min_three_ref_eig_full_segment": min_three_ref,
        "max_reference_m": max_ref,
        "min_offshore_clearance_m": min_clear,
        "within_d_dep": float(max_ref < cfg.max_allowed_reference),
    }


def verify_transfer_geometry(
    source_tri: AnchorRootedTriangulation,
    target_tri: AnchorRootedTriangulation,
    moves: Sequence[Move],
    params: GeometryParams | None = None,
) -> dict[str, float | int]:
    """Verify continuous clearance, reference range, crossing angles, 3-ref conditioning,
    and symmetric Make-Before-Break pre-requests across an entire transfer move sequence.
    """
    cfg = params or source_tri.params
    pos: dict[object, tuple[float, float]] = source_tri.all_coords()
    graph: dict[int, tuple[object, object]] = dict(source_tri.parents)

    min_clear = min(math.dist(a, b) for a, b in combinations(pos.values(), 2))
    max_ref = max(math.dist(pos[i], pos[p]) for i, ps in graph.items() for p in ps)
    min_three_ref = math.inf
    crossings = 0
    segments = 0

    for move in moves:
        nid = move.node
        if any(nid in ps for ps in graph.values()):
            raise AssertionError(f"Moving node {nid} still has active children in graph")
        if not move.legs:
            raise AssertionError(f"Empty move for node {nid}")
        if set(move.legs[0].parents) != set(graph[nid]):
            raise AssertionError(f"Initial parents mismatch for node {nid}")

        fixed = {k: v for k, v in pos.items() if k != nid}
        for idx, leg in enumerate(move.legs):
            a, b = leg.start, leg.end
            if math.dist(a, pos[nid]) > 1e-6 or math.dist(a, b) <= 1e-9:
                raise AssertionError(f"Discontinuous or zero-length leg for node {nid}")

            refs = list(leg.parents)
            if len(set(refs)) != 2:
                raise AssertionError(f"Leg must hold 2 distinct primary parents: {refs}")
            p1, p2 = fixed[refs[0]], fixed[refs[1]]
            side_a, side_b = _det(p1, p2, a), _det(p1, p2, b)

            if leg.auxiliary is not None:
                if leg.auxiliary in refs:
                    raise AssertionError(f"Auxiliary {leg.auxiliary} duplicates primary parent")
                refs.append(leg.auxiliary)
            if leg.prerequest_next_parent is not None:
                refs.append(leg.prerequest_next_parent)

            if idx + 1 < len(move.legs):
                incoming = set(move.legs[idx + 1].parents) - set(leg.parents)
                if len(incoming) > 1:
                    raise AssertionError(f"More than 1 parent changed at handover for node {nid}")
                if incoming and leg.prerequest_next_parent not in incoming:
                    raise AssertionError(
                        f"Missing symmetric pre-request for incoming parent {incoming} on node {nid}"
                    )

            if side_a * side_b <= 0.0:
                if leg.kind != "bisector" or side_a * side_b >= 0.0:
                    raise AssertionError(f"Illegal parent-line touch/crossing on node {nid} leg {leg}")
                if leg.auxiliary is None or abs(_det(p1, p2, fixed[leg.auxiliary])) < 1e-9:
                    raise AssertionError(f"Collinear auxiliary on bisector crossing for node {nid}")
                if any(abs(math.dist(pt, p1) - math.dist(pt, p2)) > 1e-6 for pt in (a, b)):
                    raise AssertionError(f"Bisector leg not equidistant to crossed edge on node {nid}")
                min_three_ref = min(
                    min_three_ref,
                    segment_three_ref_min_eigenvalue(a, b, (p1, p2, fixed[leg.auxiliary])),
                )
                crossings += 1
            elif leg.kind == "bisector":
                raise AssertionError(f"Bisector leg failed to cross parent line on node {nid}")

            for pt in (a, b):
                for r in refs:
                    max_ref = max(max_ref, math.dist(pt, fixed[r]))
            for pt in fixed.values():
                min_clear = min(min_clear, _point_segment_distance(pt, a, b))

            pos[nid] = b
            graph[nid] = leg.parents
            segments += 1

    for k, pt in target_tri.positions.items():
        if math.dist(pos[k], pt) > 1e-6:
            raise AssertionError(f"Final position mismatch for node {k}")
        if set(graph[k]) != set(target_tri.parents[k]):
            raise AssertionError(f"Final parent graph mismatch for node {k}")

    if min_three_ref is math.inf:
        min_three_ref = 1.0
    if min_clear < cfg.d_clear - 1e-6:
        raise AssertionError(f"Minimum clearance {min_clear:.3f} < {cfg.d_clear}")
    if max_ref >= cfg.max_allowed_reference:
        raise AssertionError(
            f"Max reference distance {max_ref:.3f} >= d_dep - 2*e_track ({cfg.max_allowed_reference})"
        )

    return {
        "min_clearance_m": min_clear,
        "max_reference_m": max_ref,
        "min_three_ref_eig_full_segment": min_three_ref,
        "baseline_crossings": crossings,
        "segments": segments,
    }


def verify_symbolic_identities() -> dict[str, bool]:
    """Verify Proposition 2.1, berth conditioning, and Cauchy-Binet identities.

    Uses SymPy symbolic simplification when available, and exact rational
    (`fractions.Fraction`) multi-point polynomial identity proofs otherwise.
    """
    try:
        import sympy as sp
    except ImportError:
        return _verify_exact_rational_identities()

    s, h1, dy, hb, rho = sp.symbols("s h1 dy hb rho", positive=True)

    # 1. Proposition 2.1 strict acuteness dot-product equivalences
    v_m1 = sp.Matrix([0, 0])
    v_0 = sp.Matrix([s, 0])
    v_1 = sp.Matrix([s / 2, h1])
    v_2 = sp.Matrix([s, h1 + dy])
    v_3 = sp.Matrix([s / 2, h1 + 2 * dy])

    dot_t1_apex = sp.simplify((v_m1 - v_1).dot(v_0 - v_1))
    dot_t2_v1 = sp.simplify((v_0 - v_1).dot(v_2 - v_1))
    dot_t3_v2 = sp.simplify((v_1 - v_2).dot(v_3 - v_2))
    prop_acute_ok = (
        sp.simplify(dot_t1_apex - (h1**2 - s**2 / 4)) == 0
        and sp.simplify(dot_t2_v1 - (s**2 / 4 - h1 * dy)) == 0
        and sp.simplify(dot_t3_v2 - (s**2 / 4 - dy**2)) == 0
    )

    # 2. Closed-form circumcenter equidistance across face 1, face 2, face 3 (odd), face 4 (even)
    c1 = sp.Matrix([s / 2, (h1**2 - s**2 / 4) / (2 * h1)])
    c2 = sp.Matrix([sp.Rational(3, 4) * s + h1 * dy / s, (h1 + dy) / 2])
    c3 = sp.Matrix([sp.Rational(3, 4) * s - dy**2 / s, h1 + dy])
    v_4 = sp.Matrix([s, h1 + 3 * dy])
    c4 = sp.Matrix([sp.Rational(3, 4) * s + dy**2 / s, h1 + 2 * dy])

    def eq_dist(c_pt: sp.Matrix, p_a: sp.Matrix, p_b: sp.Matrix, p_c: sp.Matrix) -> bool:
        d_a = (c_pt - p_a).dot(c_pt - p_a)
        d_b = (c_pt - p_b).dot(c_pt - p_b)
        d_c = (c_pt - p_c).dot(c_pt - p_c)
        return sp.simplify(d_a - d_b) == 0 and sp.simplify(d_b - d_c) == 0

    circ_ok = (
        eq_dist(c1, v_m1, v_0, v_1)
        and eq_dist(c2, v_0, v_1, v_2)
        and eq_dist(c3, v_1, v_2, v_3)
        and eq_dist(c4, v_2, v_3, v_4)
    )

    # 3. Optimal aspect ratio rho* = 1/sqrt(12) gives sin(phi) = 1
    sin2_phi = (64 * rho**2) / ((1 + 4 * rho**2) * (1 + 36 * rho**2))
    one_minus = sp.factor(1 - sin2_phi)
    rho_star = 1 / sp.sqrt(12)
    opt_ok = (
        sp.simplify(sin2_phi.subs(rho, rho_star) - 1) == 0
        and sp.simplify(sp.diff(sin2_phi, rho).subs(rho, rho_star)) == 0
        and sp.simplify(one_minus - (1 - 12 * rho**2) ** 2 / ((1 + 4 * rho**2) * (1 + 36 * rho**2))) == 0
    )

    # 4. Berth midpoint worst-case eigenvalue formula
    p_mid = sp.Matrix([s / 2, hb])
    n0 = (p_mid - v_m1) / sp.sqrt((s / 2) ** 2 + hb**2)
    n1 = (p_mid - v_0) / sp.sqrt((s / 2) ** 2 + hb**2)
    i_geom = n0 * n0.T + n1 * n1.T
    expected_eig = (2 * hb**2) / ((s / 2) ** 2 + hb**2)
    eig_vals = list(i_geom.eigenvals().keys())
    berth_ok = any(sp.simplify(ev - expected_eig) == 0 for ev in eig_vals)

    # 5. Cauchy-Binet 3-vector determinant identity
    t1, t2, t3 = sp.symbols("t1 t2 t3", real=True)
    w1, w2, w3 = sp.symbols("w1 w2 w3", positive=True)
    u1 = sp.Matrix([sp.cos(t1), sp.sin(t1)])
    u2 = sp.Matrix([sp.cos(t2), sp.sin(t2)])
    u3 = sp.Matrix([sp.cos(t3), sp.sin(t3)])
    i3 = w1 * u1 * u1.T + w2 * u2 * u2.T + w3 * u3 * u3.T
    rhs = (
        w1 * w2 * sp.sin(t1 - t2) ** 2
        + w1 * w3 * sp.sin(t1 - t3) ** 2
        + w2 * w3 * sp.sin(t2 - t3) ** 2
    )
    cb_ok = sp.simplify(sp.trigsimp(sp.det(i3) - rhs)) == 0

    return {
        "prop_2_1_acute_iff_verified": bool(prop_acute_ok),
        "circumcenter_equidistance_verified": bool(circ_ok),
        "optimal_aspect_ratio_sin_phi_one_verified": bool(opt_ok),
        "berth_worst_eigenvalue_formula_verified": bool(berth_ok),
        "cauchy_binet_three_vector_det_verified": bool(cb_ok),
    }


def _verify_exact_rational_identities() -> dict[str, bool]:
    """Zero-rounding-error exact rational polynomial identity proofs via fractions.Fraction."""
    from fractions import Fraction

    def fdot(a: tuple[Fraction, Fraction], b: tuple[Fraction, Fraction]) -> Fraction:
        return a[0] * b[0] + a[1] * b[1]

    def fsub(a: tuple[Fraction, Fraction], b: tuple[Fraction, Fraction]) -> tuple[Fraction, Fraction]:
        return (a[0] - b[0], a[1] - b[1])

    s_vals = [Fraction(320), Fraction(400), Fraction(500)]
    h1_vals = [Fraction(280), Fraction(300), Fraction(380)]
    dy_vals = [Fraction(70), Fraction(90), Fraction(100)]

    prop_acute_ok = True
    circ_ok = True
    for s in s_vals:
        for h1 in h1_vals:
            for dy in dy_vals:
                v_m1 = (Fraction(0), Fraction(0))
                v_0 = (s, Fraction(0))
                v_1 = (s / 2, h1)
                v_2 = (s, h1 + dy)
                v_3 = (s / 2, h1 + 2 * dy)
                v_4 = (s, h1 + 3 * dy)

                if fdot(fsub(v_m1, v_1), fsub(v_0, v_1)) != h1 * h1 - (s * s) / 4:
                    prop_acute_ok = False
                if fdot(fsub(v_0, v_1), fsub(v_2, v_1)) != (s * s) / 4 - h1 * dy:
                    prop_acute_ok = False
                if fdot(fsub(v_1, v_2), fsub(v_3, v_2)) != (s * s) / 4 - dy * dy:
                    prop_acute_ok = False

                c1 = (s / 2, (h1 * h1 - (s * s) / 4) / (2 * h1))
                c2 = (Fraction(3, 4) * s + (h1 * dy) / s, (h1 + dy) / 2)
                c3 = (Fraction(3, 4) * s - (dy * dy) / s, h1 + dy)
                c4 = (Fraction(3, 4) * s + (dy * dy) / s, h1 + 2 * dy)

                for cpt, va, vb, vc in [
                    (c1, v_m1, v_0, v_1),
                    (c2, v_0, v_1, v_2),
                    (c3, v_1, v_2, v_3),
                    (c4, v_2, v_3, v_4),
                ]:
                    da = fdot(fsub(cpt, va), fsub(cpt, va))
                    db = fdot(fsub(cpt, vb), fsub(cpt, vb))
                    dc = fdot(fsub(cpt, vc), fsub(cpt, vc))
                    if da != db or db != dc:
                        circ_ok = False

    # 3. Exact rational identity: (1 + 4*rho^2)*(1 + 36*rho^2) - 64*rho^2 == (1 - 12*rho^2)^2
    opt_ok = True
    for num, den in [(1, 6), (1, 4), (1, 3), (2, 5), (1, 2), (3, 7)]:
        r2 = Fraction(num, den) ** 2
        lhs = (1 + 4 * r2) * (1 + 36 * r2) - 64 * r2
        rhs = (1 - 12 * r2) ** 2
        if lhs != rhs:
            opt_ok = False
    # Exact rho^2 = 1/12 evaluation (sin^2(phi) == 1)
    r2_star = Fraction(1, 12)
    sin2_star = (64 * r2_star) / ((1 + 4 * r2_star) * (1 + 36 * r2_star))
    if sin2_star != Fraction(1):
        opt_ok = False

    # 4. Exact rational berth midpoint eigenvalue & inverse formula
    berth_ok = True
    for s in s_vals:
        half_s = s / 2
        for hb in [Fraction(40), Fraction(100), Fraction(120), Fraction(150)]:
            l2 = half_s * half_s + hb * hb
            dx0, dy0 = half_s, hb
            dx1, dy1 = -half_s, hb
            # Unnormalized I_geom = (1/l2) * [[dx0^2 + dx1^2, dx0*dy0 + dx1*dy1], [..., dy0^2 + dy1^2]]
            m00 = (dx0 * dx0 + dx1 * dx1) / l2
            m01 = (dx0 * dy0 + dx1 * dy1) / l2
            m11 = (dy0 * dy0 + dy1 * dy1) / l2
            if m01 != 0:
                berth_ok = False
            if m11 != (2 * hb * hb) / (half_s * half_s + hb * hb):
                berth_ok = False
            if m00 != (2 * half_s * half_s) / (half_s * half_s + hb * hb):
                berth_ok = False
        # Verify hb^2 = (s/2)^2 * iota / (2 - iota) inverts exact eigenvalue to iota
        for iota in [Fraction(1, 5), Fraction(2, 5), Fraction(3, 5)]:
            hb2 = (half_s * half_s) * iota / (2 - iota)
            if (2 * hb2) / (half_s * half_s + hb2) != iota:
                berth_ok = False

    # 5. Exact rational Cauchy-Binet 3-vector determinant identity using stereographic circle points
    cb_ok = True
    t_triples = [
        (Fraction(0), Fraction(1, 2), Fraction(2)),
        (Fraction(-1, 3), Fraction(1, 4), Fraction(3, 2)),
        (Fraction(1, 5), Fraction(4, 3), Fraction(-2)),
    ]
    w_triples = [
        (Fraction(1), Fraction(1), Fraction(1)),
        (Fraction(2, 3), Fraction(5, 4), Fraction(7, 5)),
    ]
    for ts in t_triples:
        vecs = [
            ((1 - t * t) / (1 + t * t), (2 * t) / (1 + t * t))
            for t in ts
        ]
        for ws in w_triples:
            a00 = sum(w * u[0] * u[0] for w, u in zip(ws, vecs))
            a01 = sum(w * u[0] * u[1] for w, u in zip(ws, vecs))
            a11 = sum(w * u[1] * u[1] for w, u in zip(ws, vecs))
            det_i3 = a00 * a11 - a01 * a01
            rhs = Fraction(0)
            for i in range(3):
                for j in range(i + 1, 3):
                    sin_ij = vecs[i][0] * vecs[j][1] - vecs[i][1] * vecs[j][0]
                    rhs += ws[i] * ws[j] * sin_ij * sin_ij
            if det_i3 != rhs:
                cb_ok = False

    return {
        "prop_2_1_acute_iff_verified": prop_acute_ok,
        "circumcenter_equidistance_verified": circ_ok,
        "optimal_aspect_ratio_sin_phi_one_verified": opt_ok,
        "berth_worst_eigenvalue_formula_verified": berth_ok,
        "cauchy_binet_three_vector_det_verified": cb_ok,
    }
