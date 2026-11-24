"""Event-triggered pipelined concurrency scheduler and similarity-preserving polar entry."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from itertools import combinations
from typing import Sequence

import numpy as np

from route_b_geometry import (
    AnchorRootedTriangulation,
    DEFAULT_V_AXIS_MAX,
    GeometryParams,
    Leg,
    Move,
    maximal_shared_prefix,
)


def _is_acyclic_and_min_indegree(
    parents: dict[int, tuple[object, ...]], beacons: dict[object, tuple[float, float]]
) -> tuple[bool, int]:
    min_deg = min((len(ps) for ps in parents.values()), default=2)
    color: dict[int, int] = {}

    def dfs(u: int) -> bool:
        color[u] = 1
        for p in parents[u]:
            if p in beacons:
                continue
            p_int = int(p)
            st = color.get(p_int, 0)
            if st == 1:
                return False
            if st == 0 and not dfs(p_int):
                return False
        color[u] = 2
        return True

    for nid in parents:
        if color.get(nid, 0) == 0 and not dfs(nid):
            return False, min_deg
    return True, min_deg


def _leg_references(leg: Leg) -> set[object]:
    refs: set[object] = set(leg.parents)
    if leg.auxiliary is not None:
        refs.add(leg.auxiliary)
    if leg.prerequest_next_parent is not None:
        refs.add(leg.prerequest_next_parent)
    return refs


def simulate_serial_execution(
    source_tri: AnchorRootedTriangulation,
    target_tri: AnchorRootedTriangulation,
    moves: Sequence[Move],
    v_cruise: float = 10.0,
    t_settle: float = 5.0,
    dt: float = 0.5,
    seed: int | None = None,
    stall_prob: float = 0.0,
) -> dict[str, float | int | bool]:
    """Serial baseline execution of moves for makespan comparison."""
    if v_cruise <= 0.0 or v_cruise > source_tri.params.v_axis_max:
        raise ValueError(f"v_cruise={v_cruise} exceeds v_axis_max={source_tri.params.v_axis_max}")
    rng = random.Random(seed)
    total_time = 0.0
    total_dist = 0.0
    gantt: list[dict[str, float | int]] = []
    step_dist = v_cruise * dt
    for move in moves:
        t_start = total_time
        for leg in move.legs:
            d = math.dist(leg.start, leg.end)
            total_dist += d
            if stall_prob <= 0.0:
                total_time += math.ceil((d / v_cruise) / dt) * dt
            else:
                prog = 0.0
                while prog < d - 1e-9:
                    total_time += dt
                    if rng.random() >= stall_prob:
                        prog = min(d, prog + step_dist)
        total_time += t_settle
        gantt.append(
            {
                "node": int(move.node),
                "start_s": round(t_start, 3),
                "end_s": round(total_time, 3),
                "duration_s": round(total_time - t_start, 3),
            }
        )
    return {
        "makespan_s": total_time,
        "total_distance_m": total_dist,
        "num_moves": len(moves),
        "gantt": gantt,
    }


@dataclass
class _AgentMovePlan:
    node: int
    moves: list[Move]
    move_idx: int = 0
    leg_idx: int = 0
    leg_progress: float = 0.0
    state: str = "queued_withdraw"  # queued_withdraw, moving, settling, berthed, seated_target
    settle_remaining: float = 0.0
    launch_rank: int = 0
    first_move_s: float | None = None
    last_finish_s: float | None = None


def simulate_pipelined_execution(
    source_tri: AnchorRootedTriangulation,
    target_tri: AnchorRootedTriangulation,
    moves: Sequence[Move],
    d_pipe: float = 120.0,
    v_cruise: float = 10.0,
    t_settle: float = 5.0,
    dt: float = 0.5,
    seed: int | None = None,
    stall_prob: float = 0.0,
    max_ticks: int = 40000,
) -> dict[str, float | int | bool]:
    """Simulate tick-by-tick event-triggered pipelined concurrency with static parent sets.

    Asserts at every tick:
    - Every reference (primary parents, auxiliary, pre-request) of every moving aircraft is 100% static.
    - Pairwise distance between moving aircraft >= d_pipe.
    - Distance from moving aircraft to stationary nodes >= d_clear.
    - Global dependency graph is beacon-rooted, acyclic, and every aircraft has in-degree >= 2.
    """
    cfg = source_tri.params
    if v_cruise <= 0.0 or v_cruise > cfg.v_axis_max:
        raise ValueError(f"v_cruise={v_cruise} exceeds v_axis_max={cfg.v_axis_max}")

    rng = random.Random(seed)
    beacons = dict(source_tri.beacons)
    prefix = maximal_shared_prefix(source_tri, target_tri)

    pos: dict[int, tuple[float, float]] = dict(source_tri.positions)
    active_parents: dict[int, tuple[object, ...]] = dict(source_tri.parents)

    moves_by_node: dict[int, list[Move]] = {}
    first_seen_order: list[int] = []
    for m in moves:
        if m.node not in moves_by_node:
            moves_by_node[m.node] = []
            first_seen_order.append(m.node)
        moves_by_node[m.node].append(m)

    agents: dict[int, _AgentMovePlan] = {}
    for nid in sorted(source_tri.positions):
        if nid in prefix or nid not in moves_by_node:
            agents[nid] = _AgentMovePlan(node=nid, moves=[], state="seated_target")
        else:
            agents[nid] = _AgentMovePlan(
                node=nid,
                moves=moves_by_node[nid],
                state="queued_withdraw",
                launch_rank=first_seen_order.index(nid),
            )

    def current_move(ag: _AgentMovePlan) -> Move:
        return ag.moves[ag.move_idx]

    def current_leg(ag: _AgentMovePlan) -> Leg:
        return ag.moves[ag.move_idx].legs[ag.leg_idx]

    def current_target_for_node(nid: int) -> dict[object, tuple[float, float]]:
        out: dict[object, tuple[float, float]] = dict(beacons)
        out.update(pos)
        return out

    def is_stationary(v: object) -> bool:
        if v in beacons:
            return True
        return agents[int(v)].state != "moving"

    def ref_ready_for_leg(ag: _AgentMovePlan, leg: Leg) -> bool:
        # First, every referenced node must be stationary right now
        for r in _leg_references(leg):
            if r in beacons:
                continue
            r_id = int(r)
            if not is_stationary(r_id):
                return False
            # If this move reaches target slot (installation / direct second half), ensure target parents are seated
            if ag.state in ("berthed", "moving") and ag.move_idx == len(ag.moves) - 1:
                # If leg belongs to installation portion (either move_idx==1 in 2-stage, or after coast in direct),
                # require any non-beacon ref that equals target parent/ancestor to be seated at target!
                if ag.moves[ag.move_idx].legs[-1].end == target_tri.positions[ag.node]:
                    if leg.kind in ("root", "bisector", "tip") and ag.leg_idx >= _first_install_leg_idx(current_move(ag), target_tri):
                        if agents[r_id].state != "seated_target":
                            return False
        return True

    def _first_install_leg_idx(mv: Move, tgt: AnchorRootedTriangulation) -> int:
        # In two-stage transfer, installation Move starts with dock/coast/root -> install starts at index 0.
        # In direct transfer, installation starts immediately after coast (or at last root leg).
        for idx, lg in enumerate(mv.legs):
            if lg.kind == "coast":
                # Installation starts after the last coast leg
                last_coast = max(i for i, l in enumerate(mv.legs) if l.kind == "coast")
                return last_coast + 1
        # If no coast leg (same-port direct), find where legs start ascending toward target slot
        if len(mv.legs) >= 2 and mv.legs[0].start != tgt.positions[mv.node]:
            for idx, lg in enumerate(mv.legs):
                if lg.kind == "root" and idx > 0:
                    return idx
        return 0

    def remaining_withdrawal_refs(other: _AgentMovePlan) -> set[int]:
        """Nodes that `other` still references before finishing withdrawal (until berth or coast)."""
        if other.state == "seated_target" or other.state == "berthed":
            return set()
        mv = current_move(other)
        # If other is already in installation move (move_idx == 1 in 2-stage), it doesn't hold source nodes
        if len(other.moves) == 2 and other.move_idx == 1:
            return set()
        refs: set[int] = set()
        # In direct move, withdrawal stops once entering coast/install
        stop_idx = len(mv.legs)
        if mv.legs[-1].end == target_tri.positions[other.node]:
            for idx, lg in enumerate(mv.legs):
                if lg.kind == "coast":
                    stop_idx = idx
                    break
        start_idx = other.leg_idx if other.state == "moving" else 0
        for idx in range(start_idx, stop_idx):
            for r in _leg_references(mv.legs[idx]):
                if r not in beacons:
                    refs.add(int(r))
        return refs

    def can_release_from_source(ag: _AgentMovePlan) -> bool:
        """Token check: no remaining source-withdrawal leg of any other agent still depends on ag.node."""
        nid = ag.node
        for other_id, other in agents.items():
            if other_id == nid:
                continue
            if nid in remaining_withdrawal_refs(other):
                return False
        return True

    def _segment_point_distance(p: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        dx, dy = b[0] - a[0], b[1] - a[1]
        denom = dx * dx + dy * dy
        if denom <= 0.0:
            return math.dist(p, a)
        u = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / denom))
        return math.dist(p, (a[0] + u * dx, a[1] + u * dy))

    def funnel_corridor_conflict(ag: _AgentMovePlan, cand_pt: tuple[float, float], entering_leg_idx: int) -> bool:
        """Hold above y = h_c + d_pipe before descending below corridor_top if any ahead moving
        aircraft is currently in the coastal band or has remaining coastal transit crossing this funnel.
        """
        mv = current_move(ag)
        leg = mv.legs[entering_leg_idx]
        corridor_top = source_tri.h_c + d_pipe
        is_withdrawal_descent = (
            leg.kind in ("tip", "bisector", "root")
            and leg.end[1] < leg.start[1] - 1e-6
            and cand_pt[1] < corridor_top - 1e-6
            and pos[ag.node][1] >= corridor_top - 1e-6
        )
        if not is_withdrawal_descent:
            return False
        funnel_legs: list[Leg] = []
        for idx in range(entering_leg_idx, len(mv.legs)):
            lg = mv.legs[idx]
            funnel_legs.append(lg)
            if lg.kind == "root":
                break
        xs = [pt[0] for lg in funnel_legs for pt in (lg.start, lg.end)]
        fx_lo, fx_hi = min(xs) - d_pipe, max(xs) + d_pipe
        for oid, oag in agents.items():
            if oid == ag.node or oag.state != "moving":
                continue
            opt = pos[oid]
            if opt[1] >= corridor_top:
                continue
            # Check both instantaneous position and remaining coastal span of oag
            if fx_lo <= opt[0] <= fx_hi:
                return True
            omv = current_move(oag)
            for idx in range(oag.leg_idx, len(omv.legs)):
                olg = omv.legs[idx]
                if olg.kind == "coast":
                    x_start_rem = opt[0] if idx == oag.leg_idx else olg.start[0]
                    ox_lo, ox_hi = sorted((x_start_rem, olg.end[0]))
                    if min(fx_hi, ox_hi) >= max(fx_lo, ox_lo):
                        return True
                elif olg.kind != "coast" and idx > oag.leg_idx:
                    break
        return False

    def _ladder_climb_bands() -> dict[tuple[object, object], tuple[float, float]]:
        """Precompute unified climb-zone x-band [bx_lo, bx_hi] per target root interval."""
        corridor_top = source_tri.h_c + d_pipe
        interval_xs: dict[tuple[object, object], list[float]] = {}
        # Map each node's target root beacon pair
        root_pair_of: dict[int, tuple[object, object]] = {}

        def find_root_pair(nid: int) -> tuple[object, object]:
            if nid in root_pair_of:
                return root_pair_of[nid]
            bset: list[object] = []
            stack: list[object] = list(target_tri.parents[nid])
            seen: set[object] = set()
            while stack:
                cur = stack.pop()
                if cur in seen:
                    continue
                seen.add(cur)
                if cur in beacons:
                    bset.append(cur)
                else:
                    stack.extend(target_tri.parents[int(cur)])
            b_sorted = tuple(sorted(bset, key=lambda b: beacons[b][0]))
            rp = (b_sorted[0], b_sorted[-1])
            root_pair_of[nid] = rp
            return rp

        for ag in agents.values():
            if not ag.moves:
                continue
            mv = ag.moves[-1]
            if mv.legs[-1].end != target_tri.positions[ag.node]:
                continue
            rp = find_root_pair(ag.node)
            xs = interval_xs.setdefault(rp, [])
            start_i = _first_install_leg_idx(mv, target_tri)
            h_c = source_tri.h_c
            for idx in range(start_i, len(mv.legs)):
                lg = mv.legs[idx]
                for s_i in range(11):
                    frac = s_i / 10.0
                    px = lg.start[0] + frac * (lg.end[0] - lg.start[0])
                    py = lg.start[1] + frac * (lg.end[1] - lg.start[1])
                    if py <= corridor_top + 1e-9:
                        dx_pad = math.sqrt(max(0.0, d_pipe * d_pipe - (py - h_c) ** 2))
                        xs.extend((px - dx_pad, px + dx_pad))
        return {rp: (min(xs), max(xs)) for rp, xs in interval_xs.items() if xs}

    climb_bands = _ladder_climb_bands()

    def _target_root_pair(nid: int) -> tuple[object, object]:
        bset: list[object] = []
        stack: list[object] = list(target_tri.parents[nid])
        seen: set[object] = set()
        while stack:
            cur = stack.pop()
            if cur in seen:
                continue
            seen.add(cur)
            if cur in beacons:
                bset.append(cur)
            else:
                stack.extend(target_tri.parents[int(cur)])
        b_sorted = tuple(sorted(bset, key=lambda b: beacons[b][0]))
        return (b_sorted[0], b_sorted[-1])

    def install_funnel_x_band(ag: _AgentMovePlan) -> tuple[float, float] | None:
        """Return unified climb-zone x-band clipped at y <= corridor_top for target ladder."""
        mv = current_move(ag)
        if mv.legs[-1].end != target_tri.positions[ag.node]:
            return None
        return climb_bands.get(_target_root_pair(ag.node))

    def _climbing_past_corridor(pid: int) -> bool:
        """True if parent pid is seated OR already climbing above corridor_top."""
        pag = agents[pid]
        if pag.state == "seated_target":
            return True
        if pag.state == "settling" and math.dist(pos[pid], target_tri.positions[pid]) <= 1e-6:
            return True
        if pag.state == "moving":
            pmv = current_move(pag)
            if pmv.legs[-1].end == target_tri.positions[pid]:
                first_inst = _first_install_leg_idx(pmv, target_tri)
                if pag.leg_idx >= first_inst and pos[pid][1] >= source_tri.h_c + d_pipe - 1e-6:
                    return True
        return False

    def _climbing_started(pid: int) -> bool:
        """True if parent pid is seated OR already climbing inside installation legs past port."""
        pag = agents[pid]
        if pag.state == "seated_target":
            return True
        if pag.state == "settling" and math.dist(pos[pid], target_tri.positions[pid]) <= 1e-6:
            return True
        if pag.state == "moving":
            pmv = current_move(pag)
            if pmv.legs[-1].end == target_tri.positions[pid]:
                first_inst = _first_install_leg_idx(pmv, target_tri)
                if pag.leg_idx >= first_inst:
                    if pos[pid][1] >= source_tri.h_c + d_pipe - 1e-6:
                        return True
                    # Also allow trailing west-approacher if parent already moved >= d_pipe past port_x
                    port_x = pmv.legs[first_inst].start[0]
                    if math.dist(pos[pid], (port_x, source_tri.h_c)) >= d_pipe - 1e-6:
                        return True
        return False

    def install_funnel_conflict(ag: _AgentMovePlan, cand_pt: tuple[float, float]) -> bool:
        """Hold on coast outside target climb-zone until target parents cleared port and path is clear."""
        leg = current_leg(ag)
        if leg.kind != "coast":
            return False
        band = install_funnel_x_band(ag)
        if band is None:
            return False
        bx_lo, bx_hi = band
        cur_x = pos[ag.node][0]
        entering = (cur_x < bx_lo or cur_x > bx_hi) and (bx_lo <= cand_pt[0] <= bx_hi)
        if not entering:
            return False
        corridor_top = source_tri.h_c + d_pipe
        mv = current_move(ag)
        port_x = mv.legs[_first_install_leg_idx(mv, target_tri)].start[0]
        from_west = cur_x < port_x
        for p in target_tri.parents[ag.node]:
            if p not in beacons and not _climbing_past_corridor(int(p)):
                return True
        for oid, oag in agents.items():
            if oid == ag.node or oag.state != "moving":
                continue
            opt = pos[oid]
            if opt[1] >= corridor_top:
                continue
            if from_west:
                if port_x - d_pipe <= opt[0] <= port_x + d_pipe:
                    return True
            else:
                if bx_lo <= opt[0] <= bx_hi:
                    return True
        return False

    def berth_undock_conflict(ag: _AgentMovePlan) -> bool:
        """Keep berthed aircraft at y = h_b while any moving aircraft occupies its coastal undock band."""
        if ag.state != "berthed":
            return False
        mv = current_move(ag)
        xs = [pos[ag.node][0]]
        for lg in mv.legs:
            if lg.kind in ("dock", "coast"):
                xs.extend((lg.start[0], lg.end[0]))
            else:
                break
        ux_lo, ux_hi = min(xs) - d_pipe, max(xs) + d_pipe
        corridor_top = source_tri.h_c + d_pipe
        for oid, oag in agents.items():
            if oid == ag.node or oag.state != "moving":
                continue
            opt = pos[oid]
            if opt[1] < corridor_top and ux_lo <= opt[0] <= ux_hi:
                return True
        return False

    def target_parents_seated(ag: _AgentMovePlan) -> bool:
        """A berthed aircraft launches for installation only once its target parents are seated."""
        if ag.state != "berthed":
            return True
        for p in target_tri.parents[ag.node]:
            if p not in beacons and agents[int(p)].state != "seated_target":
                return False
        return True

    def target_slot_occupied_by_source(ag: _AgentMovePlan) -> bool:
        """Prevent launching installation from berth before overlapping source slots vacate."""
        if ag.state != "berthed":
            return False
        mv = current_move(ag)
        for oid, oag in agents.items():
            if oid == ag.node or oag.state != "queued_withdraw":
                continue
            opt = pos[oid]
            for lg in mv.legs:
                if _segment_point_distance(opt, lg.start, lg.end) < cfg.d_clear + 1.0:
                    return True
        return False

    def coast_direction_conflict(ag: _AgentMovePlan, leg: Leg) -> bool:
        """Prevent head-on corridor deadlock if two agents move opposite x-directions with interior overlap."""
        if leg.kind != "coast":
            return False
        dx = leg.end[0] - leg.start[0]
        if abs(dx) < 1e-6:
            return False
        x_lo, x_hi = sorted((leg.start[0], leg.end[0]))
        for oid, oag in agents.items():
            if oid == ag.node or oag.state != "moving":
                continue
            oleg = current_leg(oag)
            if oleg.kind != "coast":
                continue
            odx = oleg.end[0] - oleg.start[0]
            if dx * odx < 0.0:
                ox_lo, ox_hi = sorted((oleg.start[0], oleg.end[0]))
                # Strict interior overlap (touching only at a shared merge port is not head-on crossing)
                if min(x_hi, ox_hi) - 1e-6 > max(x_lo, ox_lo):
                    return True
        return False

    def spacing_ok(cand_pt: tuple[float, float], ag_id: int, moving_positions: dict[int, tuple[float, float]]) -> bool:
        for oid, opt in moving_positions.items():
            if oid == ag_id:
                continue
            if math.dist(cand_pt, opt) < d_pipe - 1e-6:
                return False
        return True

    def static_clearance_ok(cand_pt: tuple[float, float], ag: _AgentMovePlan) -> bool:
        cur_pt = pos[ag.node]
        for bpt in beacons.values():
            if math.dist(cand_pt, bpt) < cfg.d_clear - 1e-6:
                return False
        for oid, oag in agents.items():
            if oid == ag.node or oag.state == "moving":
                continue
            if math.dist(cand_pt, pos[oid]) < cfg.d_clear - 1e-6:
                # Only allow departing from own start or arriving at own final endpoint if already safe
                if math.dist(cur_pt, pos[oid]) < cfg.d_clear - 1e-6:
                    return False
                return False
        return True

    static_parent_violations = 0
    acyclic_all_ticks = True
    min_in_deg = 2
    min_moving_sep = math.inf
    min_clearance = math.inf
    max_reference = 0.0

    # Pre-check initial state
    all_coords = current_target_for_node(-1)
    for nid, ps in active_parents.items():
        for p in ps:
            max_reference = max(max_reference, math.dist(pos[nid], all_coords[p]))

    tick = 0
    step_dist = v_cruise * dt
    while tick < max_ticks:
        if all(ag.state == "seated_target" for ag in agents.values()):
            break

        # 1. Advance settling timers first (release 1-bit seating tokens)
        for ag in agents.values():
            if ag.state == "settling":
                ag.settle_remaining -= dt
                if ag.settle_remaining <= 1e-9:
                    if ag.move_idx == len(ag.moves) - 1 and math.dist(pos[ag.node], target_tri.positions[ag.node]) <= 1e-6:
                        ag.state = "seated_target"
                        ag.last_finish_s = tick * dt
                        active_parents[ag.node] = target_tri.parents[ag.node]
                    else:
                        ag.state = "berthed"
                        ag.move_idx += 1
                        ag.leg_idx = 0
                        ag.leg_progress = 0.0

        # 2. Check launches from static queues (queued_withdraw / berthed)
        # Prioritize closer-to-completion / lower launch_rank
        moving_pos_snapshot = {nid: pos[nid] for nid, ag in agents.items() if ag.state == "moving"}
        for ag in sorted(agents.values(), key=lambda a: (0 if a.state == "berthed" else 1, a.launch_rank, a.node)):
            if ag.state == "queued_withdraw":
                leg0 = ag.moves[0].legs[0]
                if (
                    can_release_from_source(ag)
                    and ref_ready_for_leg(ag, leg0)
                    and not funnel_corridor_conflict(ag, pos[ag.node], 0)
                    and not coast_direction_conflict(ag, leg0)
                    and spacing_ok(pos[ag.node], ag.node, moving_pos_snapshot)
                ):
                    ag.state = "moving"
                    if ag.first_move_s is None:
                        ag.first_move_s = tick * dt
                    active_parents[ag.node] = leg0.parents
                    moving_pos_snapshot[ag.node] = pos[ag.node]
            elif ag.state == "berthed":
                leg0 = current_move(ag).legs[0]
                if (
                    target_parents_seated(ag)
                    and ref_ready_for_leg(ag, leg0)
                    and not berth_undock_conflict(ag)
                    and not target_slot_occupied_by_source(ag)
                    and not coast_direction_conflict(ag, leg0)
                    and spacing_ok(pos[ag.node], ag.node, moving_pos_snapshot)
                ):
                    ag.state = "moving"
                    active_parents[ag.node] = leg0.parents
                    moving_pos_snapshot[ag.node] = pos[ag.node]

        # 3. Advance moving aircraft in priority order (further along in their path moves first)
        def moving_priority(ag: _AgentMovePlan) -> tuple[int, int, int]:
            mv = current_move(ag)
            rem_legs = len(mv.legs) - ag.leg_idx
            return (0 if ag.move_idx == len(ag.moves) - 1 else 1, rem_legs, ag.launch_rank)

        moving_agents = sorted(
            (ag for ag in agents.values() if ag.state == "moving"),
            key=moving_priority,
        )
        updated_moving_pos = dict(moving_pos_snapshot)

        for ag in moving_agents:
            if stall_prob > 0.0 and rng.random() < stall_prob:
                continue
            leg = current_leg(ag)
            if not ref_ready_for_leg(ag, leg):
                continue

            seg_len = math.dist(leg.start, leg.end)
            new_prog = min(seg_len, ag.leg_progress + step_dist)
            frac = new_prog / seg_len if seg_len > 0.0 else 1.0
            cand = (
                leg.start[0] + frac * (leg.end[0] - leg.start[0]),
                leg.start[1] + frac * (leg.end[1] - leg.start[1]),
            )

            # Hold above y = h_c + d_pipe while an ahead aircraft clears the coastal funnel
            if funnel_corridor_conflict(ag, cand, ag.leg_idx):
                continue
            if install_funnel_conflict(ag, cand):
                continue

            # Verify spacing against all other currently moving aircraft and static clearance
            if not spacing_ok(cand, ag.node, updated_moving_pos):
                continue
            if not static_clearance_ok(cand, ag):
                continue

            # If reaching end of leg, verify transition to next leg (Make-Before-Break token)
            if new_prog >= seg_len - 1e-9:
                mv = current_move(ag)
                if ag.leg_idx + 1 < len(mv.legs):
                    nxt_leg = mv.legs[ag.leg_idx + 1]
                    # Check readiness using the target leg index ag.leg_idx + 1
                    orig_idx = ag.leg_idx
                    ag.leg_idx = orig_idx + 1
                    nxt_ready = ref_ready_for_leg(ag, nxt_leg)
                    ag.leg_idx = orig_idx
                    if (
                        not nxt_ready
                        or funnel_corridor_conflict(ag, leg.end, orig_idx + 1)
                        or coast_direction_conflict(ag, nxt_leg)
                    ):
                        # Hold at handover waypoint until next leg is clear and references are ready
                        pos[ag.node] = leg.end
                        ag.leg_progress = seg_len
                        updated_moving_pos[ag.node] = leg.end
                        continue
                    pos[ag.node] = leg.end
                    ag.leg_idx += 1
                    ag.leg_progress = 0.0
                    active_parents[ag.node] = nxt_leg.parents
                    updated_moving_pos[ag.node] = leg.end
                else:
                    # Arrived at terminal point of this Move (berth or target slot)
                    pos[ag.node] = leg.end
                    ag.leg_progress = seg_len
                    updated_moving_pos.pop(ag.node, None)
                    ag.state = "settling"
                    ag.settle_remaining = t_settle
            else:
                pos[ag.node] = cand
                ag.leg_progress = new_prog
                updated_moving_pos[ag.node] = cand

        # 4. Per-tick hard invariant verification
        tick += 1
        all_coords = current_target_for_node(-1)
        moving_ids = [nid for nid, ag in agents.items() if ag.state == "moving"]
        for nid in moving_ids:
            leg = current_leg(agents[nid])
            for ref in _leg_references(leg):
                if ref not in beacons and agents[int(ref)].state == "moving":
                    static_parent_violations += 1
                max_reference = max(max_reference, math.dist(pos[nid], all_coords[ref]))

        for a_id, b_id in combinations(moving_ids, 2):
            d_mv = math.dist(pos[a_id], pos[b_id])
            min_moving_sep = min(min_moving_sep, d_mv)

        for nid in moving_ids:
            for oid in agents:
                if oid == nid:
                    continue
                min_clearance = min(min_clearance, math.dist(pos[nid], pos[oid]))
            for bpt in beacons.values():
                min_clearance = min(min_clearance, math.dist(pos[nid], bpt))

        acyclic, min_deg = _is_acyclic_and_min_indegree(active_parents, beacons)
        acyclic_all_ticks = acyclic_all_ticks and acyclic
        min_in_deg = min(min_in_deg, min_deg)

    deadlocked = not all(ag.state == "seated_target" for ag in agents.values())
    final_matched = (
        not deadlocked
        and all(math.dist(pos[k], target_tri.positions[k]) <= 1e-6 for k in target_tri.positions)
        and all(set(active_parents[k]) == set(target_tri.parents[k]) for k in target_tri.parents)
    )

    if min_moving_sep is math.inf:
        min_moving_sep = d_pipe
    if min_clearance is math.inf:
        min_clearance = cfg.d_clear

    gantt = [
        {
            "node": nid,
            "start_s": round(ag.first_move_s if ag.first_move_s is not None else 0.0, 3),
            "end_s": round(ag.last_finish_s if ag.last_finish_s is not None else tick * dt, 3),
            "duration_s": round(
                (ag.last_finish_s - ag.first_move_s)
                if (ag.first_move_s is not None and ag.last_finish_s is not None)
                else 0.0,
                3,
            ),
        }
        for nid, ag in sorted(agents.items())
        if ag.moves
    ]

    return {
        "makespan_s": tick * dt,
        "ticks": tick,
        "deadlocked": deadlocked,
        "final_matched": final_matched,
        "static_parent_violations": static_parent_violations,
        "acyclic_all_ticks": acyclic_all_ticks,
        "min_in_degree": min_in_deg,
        "min_moving_separation_m": min_moving_sep,
        "min_clearance_m": min_clearance,
        "max_reference_m": max_reference,
        "gantt": gantt,
    }


def _rot2d(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def check_polar_entry_cone(
    standard_positions: dict[int, tuple[float, float]],
    beacons: dict[object, tuple[float, float]],
    parents: dict[int, tuple[object, object]],
    pivot: tuple[float, float],
    alpha_range: tuple[float, float],
    theta_max_rad: float,
    params: GeometryParams | None = None,
) -> bool:
    """Verify Proposition 1 admissible polar cone bounds on [alpha_min, alpha_max] x [-theta_max, theta_max]."""
    cfg = params or GeometryParams()
    piv = np.array(pivot, dtype=float)
    alphas = np.linspace(alpha_range[0], alpha_range[1], 21)
    thetas = np.linspace(-theta_max_rad, theta_max_rad, 31)
    max_edge_allowed = cfg.d_dep - 2.0 * cfg.e_track
    min_sep_allowed = cfg.d_safe + 2.0 * cfg.e_track

    for alpha in alphas:
        for th in thetas:
            rot = _rot2d(float(th))
            pos: dict[object, np.ndarray] = {k: np.array(v, dtype=float) for k, v in beacons.items()}
            for nid, q_std in standard_positions.items():
                pt = piv + float(alpha) * (rot @ (np.array(q_std, dtype=float) - piv))
                if float(pt[1]) <= cfg.d_safe + cfg.e_track:
                    return False
                pos[nid] = pt

            for nid, ps in parents.items():
                for p in ps:
                    if float(np.linalg.norm(pos[nid] - pos[p])) >= max_edge_allowed:
                        return False

            nids = sorted(standard_positions)
            for a, b in combinations(nids, 2):
                if float(np.linalg.norm(pos[a] - pos[b])) <= min_sep_allowed:
                    return False
            for nid in nids:
                for b_lbl in beacons:
                    if float(np.linalg.norm(pos[nid] - pos[b_lbl])) <= min_sep_allowed:
                        return False
    return True


def _trapezoidal_profile(total_s: float, v_s: float, a_s: float, dt: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Compute time grid and s(t), s_dot(t), s_ddot(t) for a symmetric trapezoidal/triangular profile."""
    if v_s <= 0.0 or a_s <= 0.0:
        raise ValueError("v_s and a_s must be positive")
    t_acc = v_s / a_s
    s_acc = 0.5 * a_s * t_acc * t_acc
    if 2.0 * s_acc >= total_s:
        t_acc = math.sqrt(total_s / a_s)
        t_cruise = 0.0
    else:
        t_cruise = (total_s - 2.0 * s_acc) / v_s
    t_total = 2.0 * t_acc + t_cruise
    n_steps = max(2, int(math.ceil(t_total / dt)) + 1)
    times = np.linspace(0.0, t_total, n_steps)

    s_arr = np.zeros_like(times)
    sd_arr = np.zeros_like(times)
    sdd_arr = np.zeros_like(times)
    v_peak = a_s * t_acc
    for idx, t in enumerate(times):
        if t <= t_acc:
            sdd_arr[idx] = a_s
            sd_arr[idx] = a_s * t
            s_arr[idx] = 0.5 * a_s * t * t
        elif t <= t_acc + t_cruise:
            dt_c = t - t_acc
            sdd_arr[idx] = 0.0
            sd_arr[idx] = v_peak
            s_arr[idx] = 0.5 * a_s * t_acc * t_acc + v_peak * dt_c
        else:
            dt_d = t_total - t
            sdd_arr[idx] = -a_s
            sd_arr[idx] = max(0.0, a_s * dt_d)
            s_arr[idx] = total_s - 0.5 * a_s * dt_d * dt_d
    s_arr[0] = 0.0
    s_arr[-1] = total_s
    sd_arr[0] = 0.0
    sd_arr[-1] = 0.0
    return times, s_arr, sd_arr, sdd_arr


def plan_similarity_preserving_entry(
    standard_positions: dict[int, tuple[float, float]],
    beacons: dict[object, tuple[float, float]],
    parents: dict[int, tuple[object, object]],
    pivot: tuple[float, float],
    alpha0: float,
    theta0: float,
    v_axis_max: float = DEFAULT_V_AXIS_MAX,
    a_max: float | None = 4.0,
    dt: float = 0.1,
    params: GeometryParams | None = None,
) -> dict[str, object]:
    """Generate a similarity-preserving polar-homotopy entry trajectory with bounded acceleration."""
    cfg = params or GeometryParams(v_axis_max=v_axis_max, a_max=a_max)
    if v_axis_max <= 0.0 or v_axis_max > cfg.v_axis_max:
        raise ValueError(f"v_axis_max={v_axis_max} must lie in (0, {cfg.v_axis_max}]")

    piv = np.array(pivot, dtype=float)
    r_vecs = {nid: np.array(q, dtype=float) - piv for nid, q in standard_positions.items()}
    r_max = max(float(np.linalg.norm(rv)) for rv in r_vecs.values())

    d_alpha = 1.0 - alpha0
    d_theta = -theta0
    alpha_max = max(alpha0, 1.0)
    l_s = max(1e-9, r_max * math.hypot(d_alpha, alpha_max * d_theta))
    k_s = r_max * abs(d_theta) * math.hypot(2.0 * d_alpha, alpha_max * d_theta)

    # Speed limit per axis <= v_axis_max (bounded by Euclidean norm <= v_axis_max)
    v_s_max = v_axis_max / l_s
    eff_a_max = cfg.a_max if a_max is None else a_max
    if eff_a_max is None or eff_a_max <= 0.0:
        a_s_max = 2.0 * v_s_max
    else:
        if k_s > 0.0:
            v_s_max = min(v_s_max, math.sqrt((0.5 * eff_a_max) / k_s))
        a_s_max = max(1e-6, (eff_a_max - k_s * (v_s_max ** 2)) / l_s)

    times, s_arr, sd_arr, sdd_arr = _trapezoidal_profile(1.0, v_s_max, a_s_max, dt)

    # Precompute standard pairwise angles among aircraft to verify 100% angle invariance
    nids = sorted(standard_positions)
    triples = [
        (a, b, c)
        for a, b, c in combinations(nids, 3)
    ]

    def angle_at(pts: dict[int, tuple[float, float]], a: int, b: int, c: int) -> float:
        u = (pts[a][0] - pts[b][0], pts[a][1] - pts[b][1])
        v = (pts[c][0] - pts[b][0], pts[c][1] - pts[b][1])
        cross = abs(u[0] * v[1] - u[1] * v[0])
        dot = u[0] * v[0] + u[1] * v[1]
        return math.degrees(math.atan2(cross, dot))

    std_angles = {tri: angle_at(standard_positions, *tri) for tri in triples}

    pos_trace: list[dict[int, tuple[float, float]]] = []
    vel_trace: list[dict[int, tuple[float, float]]] = []
    acc_trace: list[dict[int, tuple[float, float]]] = []
    max_axis_speed = 0.0
    max_accel = 0.0
    max_angle_drift = 0.0
    max_ref = 0.0
    min_sep = math.inf

    for s_val, sd_val, sdd_val in zip(s_arr, sd_arr, sdd_arr):
        alpha = (1.0 - float(s_val)) * alpha0 + float(s_val)
        theta = (1.0 - float(s_val)) * theta0
        rot = _rot2d(theta)
        rot_d = np.array([[-math.sin(theta), -math.cos(theta)], [math.cos(theta), -math.sin(theta)]], dtype=float)
        rot_dd = -rot

        tick_pos: dict[int, tuple[float, float]] = {}
        tick_vel: dict[int, tuple[float, float]] = {}
        tick_acc: dict[int, tuple[float, float]] = {}

        for nid, r_vec in r_vecs.items():
            q_pt = piv + alpha * (rot @ r_vec)
            qp_s = (d_alpha * rot + alpha * d_theta * rot_d) @ r_vec
            qpp_s = (2.0 * d_alpha * d_theta * rot_d + alpha * (d_theta ** 2) * rot_dd) @ r_vec
            v_pt = qp_s * float(sd_val)
            a_pt = qp_s * float(sdd_val) + qpp_s * (float(sd_val) ** 2)

            tick_pos[nid] = (float(q_pt[0]), float(q_pt[1]))
            tick_vel[nid] = (float(v_pt[0]), float(v_pt[1]))
            tick_acc[nid] = (float(a_pt[0]), float(a_pt[1]))

            max_axis_speed = max(max_axis_speed, abs(float(v_pt[0])), abs(float(v_pt[1])))
            max_accel = max(max_accel, float(np.linalg.norm(a_pt)))

        for tri_key, base_ang in std_angles.items():
            max_angle_drift = max(max_angle_drift, abs(angle_at(tick_pos, *tri_key) - base_ang))

        all_nodes = {**beacons, **tick_pos}
        for nid, ps in parents.items():
            for p in ps:
                max_ref = max(max_ref, math.dist(tick_pos[nid], all_nodes[p]))
        for a, b in combinations(nids, 2):
            min_sep = min(min_sep, math.dist(tick_pos[a], tick_pos[b]))
        for nid in nids:
            for b_pt in beacons.values():
                min_sep = min(min_sep, math.dist(tick_pos[nid], b_pt))

        pos_trace.append(tick_pos)
        vel_trace.append(tick_vel)
        acc_trace.append(tick_acc)

    final_speed = max(math.hypot(*v) for v in vel_trace[-1].values())
    return {
        "times": times.tolist(),
        "positions_by_tick": pos_trace,
        "velocities_by_tick": vel_trace,
        "accelerations_by_tick": acc_trace,
        "entry_duration_s": float(times[-1]),
        "max_axis_speed_mps": max_axis_speed,
        "max_accel_mps2": max_accel,
        "final_speed_mps": final_speed,
        "max_angle_drift_deg": max_angle_drift,
        "max_reference_m": max_ref,
        "min_separation_m": min_sep,
    }
