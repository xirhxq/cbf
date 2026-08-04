"""Replay the real qualified WNLS estimator on an R1H simulator trajectory.

Builds per-frame reference groups and held commands from an R1H ``data.json``,
then runs the same estimator producer path used by the qualified pipeline
(``run_qualified_closure_campaign._build_condition_replay_row``) so the output
rows are schema-identical to the official replay.  Ranges are true distances
plus deterministic noise drawn from ``numpy.default_rng(range_noise_seed)``.
"""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

import numpy as np

from scripts.diagnostics.run_qualified_closure_campaign import (
    _build_condition_replay_row,
)


def _load_materialized_bases(data: dict) -> list[list[float]]:
    for key in ("bases", "base-stations", "baseStations"):
        if key in data.get("config", {}):
            return data["config"][key]
    # fall back to the known mission bases
    return [[-1550.0, -300.0], [-1550.0, 0.0], [-1550.0, 300.0]]


def build_raw_reference_groups(
    data: dict,
    bases: list[list[float]],
    range_noise_seed: int,
    frames: int,
    reference_mode: str = "fixed",
    max_references: int | None = None,
) -> dict[tuple[int, int], list[dict]]:
    """Return raw reference records keyed by (frame, robot_id)."""
    state = data["state"]
    rng = np.random.default_rng(range_noise_seed)
    groups: dict[tuple[int, int], list[dict]] = {}
    for frame_index, frame in enumerate(state[:frames]):
        fixed_formations = {
            entry["id"]: entry for entry in frame.get("formation", [])
        }
        formations = {
            entry["id"]: entry
            for entry in frame.get(
                "formation" if reference_mode == "fixed" else "covariance_formation",
                [],
            )
        }
        for robot in frame["robots"]:
            robot_id = robot["id"]
            position = np.asarray(
                [robot["state"]["x"], robot["state"]["y"]], dtype=float
            )
            formation = formations.get(robot_id, {})
            references: list[dict] = []
            for base_id in formation.get("baseIds", []):
                base_position = np.asarray(bases[base_id], dtype=float)
                true_range = float(np.linalg.norm(position - base_position))
                noise = float(rng.normal(0.0, 0.5))
                references.append({
                    "key": ("base", int(base_id)),
                    "position": list(base_position),
                    "range": true_range + noise,
                    "true_range": true_range,
                    "covariance": [[0.0, 0.0], [0.0, 0.0]],
                    "ranging_sigma": 0.5,
                    "base_anchor_provenance": [int(base_id)],
                })
            for anchor_id in formation.get("anchorIds", []):
                anchor = next(
                    (r for r in frame["robots"] if r["id"] == anchor_id),
                    None,
                )
                if anchor is None:
                    continue
                anchor_position = np.asarray(
                    [anchor["state"]["x"], anchor["state"]["y"]], dtype=float
                )
                true_range = float(np.linalg.norm(position - anchor_position))
                noise = float(rng.normal(0.0, 0.5))
                references.append({
                    "key": ("uav", int(anchor_id)),
                    "range": true_range + noise,
                    "true_range": true_range,
                    "ranging_sigma": 0.5,
                })
            references.sort(key=lambda ref: ref["key"])
            if (
                reference_mode == "dynamic"
                and max_references is not None
                and len(references) > max_references
            ):
                fixed_keys = {
                    ("base", int(base_id))
                    for base_id in fixed_formations.get(robot_id, {}).get(
                        "baseIds", []
                    )
                } | {
                    ("uav", int(anchor_id))
                    for anchor_id in fixed_formations.get(robot_id, {}).get(
                        "anchorIds", []
                    )
                }
                fixed_refs = [
                    ref for ref in references if ref["key"] in fixed_keys
                ]
                extra_refs = [
                    ref for ref in references if ref["key"] not in fixed_keys
                ]
                extra_refs.sort(key=lambda ref: float(ref["true_range"]))
                references = fixed_refs + extra_refs[: max(0, max_references - len(fixed_refs))]
            groups[(frame_index, robot_id)] = references
    return groups


def build_held_commands(data: dict, frames: int) -> list[dict[int, list[float]]]:
    """Held planar command per frame: command applied in the previous interval."""
    state = data["state"]
    commands: list[dict[int, list[float]]] = []
    for frame_index in range(frames):
        if frame_index == 0:
            commands.append({robot: [0.0, 0.0] for robot in range(1, 15)})
            continue
        previous = state[frame_index - 1]
        frame_commands = {}
        for robot in previous["robots"]:
            result = robot["opt"].get("result", {})
            frame_commands[robot["id"]] = [
                float(result.get("vx", 0.0)),
                float(result.get("vy", 0.0)),
            ]
        commands.append(frame_commands)
    return commands


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--estimator-config", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--range-noise-seed", type=int, default=2026081301)
    parser.add_argument(
        "--reference-mode",
        choices=("fixed", "dynamic"),
        default="fixed",
        help="fixed=comm-fixed references; dynamic=full FIM visible set",
    )
    parser.add_argument("--max-references", type=int, default=None)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()

    data = json.loads(arguments.data.read_text())
    frames = arguments.frames or len(data["state"])
    bases = _load_materialized_bases(data)
    groups = build_raw_reference_groups(
        data,
        bases,
        arguments.range_noise_seed,
        frames,
        reference_mode=arguments.reference_mode,
        max_references=arguments.max_references,
    )
    commands = build_held_commands(data, frames)

    estimator_config = json.loads(arguments.estimator_config.read_text())
    deployment = estimator_config["qualified-estimator"]["deployment"]
    domain = {key.replace("-", "_"): value for key, value in deployment.items()}
    history = estimator_config["qualified-estimator"]["history"]
    innovation_limit = history.get("q-threshold")

    from scripts.diagnostics.predictive_wnls import (
        canonical_qualified_solver,
        solve_finite_budget_wnls,
    )

    def solver_from_start(start, references):
        return solve_finite_budget_wnls(
            [record["position"] for record in references],
            [record["covariance"] for record in references],
            [record["range"] for record in references],
            start.estimate,
            references[0]["ranging_sigma"],
        )

    solver = canonical_qualified_solver(solver_from_start)
    state = {
        robot: {"public": None, "private": None, "history": 0}
        for robot in range(1, 15)
    }
    rows = []
    for frame_index in range(frames):
        held = commands[frame_index]
        for robot_id in range(1, 15):
            references = groups.get((frame_index, robot_id), [])
            try:
                row = _build_condition_replay_row(
                    frame=frame_index,
                    robot_id=robot_id,
                    raw_references=references,
                    condition_state=state,
                    held_velocity=held[robot_id],
                    deployment_domain=domain,
                    innovation_limit=innovation_limit,
                    solver=solver,
                    mission_horizon_frames=frames,
                )
                lifecycle = row["audit_bundle"]["lifecycle"]
                state[robot_id]["public"] = lifecycle["public_output"]
                state[robot_id]["private"] = lifecycle["next_private_state"]
                state[robot_id]["history"] = (
                    lifecycle["history_version"] + 1
                )
                rows.append(row)
            except Exception as error:  # fail-closed unavailable record
                rows.append({
                    "frame_index": frame_index,
                    "robot_id": robot_id,
                    "unavailable_reason": f"{type(error).__name__}: {error}",
                })
                state[robot_id]["public"] = {"output_status": "unavailable"}
                state[robot_id]["private"] = None

    summary = {
        "schema_version": "cbf2026-r1h-estimator-replay-v1",
        "range_noise_seed": arguments.range_noise_seed,
        "frames": frames,
        "row_count": len(rows),
        "rows": rows,
    }
    arguments.output.write_text(
        json.dumps(summary, indent=1, ensure_ascii=False) + "\n"
    )
    print(f"replay rows: {len(rows)} -> {arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
