"""Reusable per-frame estimator service for in-loop experiments.

Wraps the qualified WNLS solve chain (recursive prior A+B, warm-start,
anchor-covariance calibration, hold fallback) behind a stateful service so
that a simulator driver can request one estimate per robot per frame.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from scripts.diagnostics.predictive_wnls import (
    canonical_qualified_solver,
    solve_finite_budget_wnls,
)
from scripts.diagnostics.replay_r1h_estimator import (
    _hold_record,
    build_condition_row,
)


class EstimatorInLoopService:
    def __init__(
        self,
        *,
        estimator_config: dict,
        deployment_positions: dict[int, list[float]],
        mission_horizon_frames: int,
        warm_start: bool = True,
        anchor_inflation: float | None = 1.26,
    ) -> None:
        self.deployment_positions = deployment_positions
        self.mission_horizon_frames = mission_horizon_frames
        self.warm_start = warm_start
        self.anchor_inflation = anchor_inflation
        deployment = estimator_config["qualified-estimator"]["deployment"]
        self.deployment_domain = {
            key.replace("-", "_"): value
            for key, value in deployment.items()
        }
        history = estimator_config["qualified-estimator"]["history"]
        self.innovation_limit = history.get("q-threshold")
        self.state = {
            robot: {"public": None, "private": None, "history": 0}
            for robot in sorted(deployment_positions)
        }
        self.hold_age = {
            robot: 0 for robot in sorted(deployment_positions)
        }
        self.hold_speed = {
            robot: 0.0 for robot in sorted(deployment_positions)
        }

        def solver_from_start(start, references):
            return solve_finite_budget_wnls(
                [record["position"] for record in references],
                [record["covariance"] for record in references],
                [record["range"] for record in references],
                start.estimate,
                references[0]["ranging_sigma"],
            )

        self.solver = canonical_qualified_solver(solver_from_start)

    def reset(self) -> None:
        for robot in self.state:
            self.state[robot] = {
                "public": None,
                "private": None,
                "history": 0,
            }
            self.hold_age[robot] = 0
            self.hold_speed[robot] = 0.0

    def step(
        self,
        *,
        frame_index: int,
        raw_reference_groups: dict[tuple[int, int], list[dict]],
        held_commands: dict[int, list[float]],
    ) -> dict[int, dict]:
        """Run one frame for every robot in topological order."""
        outputs: dict[int, dict] = {}
        for robot_id in sorted(self.state):
            references = raw_reference_groups.get((frame_index, robot_id), [])
            try:
                row = build_condition_row(
                    frame=frame_index,
                    robot_id=robot_id,
                    raw_references=references,
                    condition_state=self.state,
                    held_velocity=held_commands[robot_id],
                    deployment_domain=self.deployment_domain,
                    innovation_limit=self.innovation_limit,
                    solver=self.solver,
                    mission_horizon_frames=self.mission_horizon_frames,
                    warm_start=self.warm_start,
                    anchor_inflation=self.anchor_inflation,
                )
                lifecycle = row["audit_bundle"]["lifecycle"]
                public = lifecycle["public_output"]
                status = public.get("output_status")
                if status in ("fresh", "predicted"):
                    estimate = public["estimate"]
                    covariance = public.get("modeled_covariance")
                    eigen = np.linalg.eigvalsh(
                        np.asarray(covariance, dtype=float)
                    )
                    epsilon = 3.0 * float(np.sqrt(max(eigen[-1], 0.0)))
                    self.state[robot_id]["public"] = public
                    self.state[robot_id]["private"] = (
                        lifecycle["next_private_state"]
                    )
                    self.state[robot_id]["history"] = (
                        lifecycle["history_version"] + 1
                    )
                    self.hold_age[robot_id] = 0
                    velocity = np.asarray(
                        held_commands[robot_id], dtype=float
                    )
                    self.hold_speed[robot_id] = float(
                        np.linalg.norm(velocity)
                    )
                    outputs[robot_id] = {
                        "estimate": estimate,
                        "epsilon": epsilon,
                        "tier": status,
                        "hold_age": 0,
                    }
                    continue
            except Exception:
                pass
            # fallback: hold
            hold = _hold_record(
                frame_index=frame_index,
                robot_id=robot_id,
                state=self.state,
                deployment_positions=self.deployment_positions,
                hold_age=self.hold_age[robot_id] + 1,
                growth_rate=(
                    self.hold_speed[robot_id]
                    if self.hold_speed[robot_id] > 0.0
                    else 50.0
                ),
            )
            self.hold_age[robot_id] += 1
            self.state[robot_id]["public"] = None
            self.state[robot_id]["private"] = None
            outputs[robot_id] = {
                "estimate": hold["estimate"],
                "epsilon": hold["epsilon"],
                "tier": "hold",
                "hold_age": hold["hold_age"],
            }
        return outputs
