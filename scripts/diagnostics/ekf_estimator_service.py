"""Basic EKF estimator for the estimator-in-the-loop experiments.

Each robot runs a standard extended Kalman filter over its fixed reference
formation:

- predict: x <- x + v_cmd * dt, P <- P + Q, with Q from a process-noise
  speed (commanded-velocity uncertainty);
- update: sequential scalar range updates from every assigned reference
  (bases and lower-index UAV anchors) with distance-dependent measurement
  noise sigma(d) = sigma0 * (1 + (d / range0)^2), plus the anchor position
  uncertainty projected along the line of sight;
- coast: when no measurement is accepted, the filter output is the
  prediction and the covariance grows only through Q (no artificial
  deployment reset, no speed-bound epsilon inflation);
- epsilon = 3 * sqrt(lambda_max(P)) comes directly from the filter.

The reference set is the fixed formation, which matches the set of
communication rows enforced by the controller.
"""

from __future__ import annotations

import numpy as np


def ranging_sigma(distance: float, sigma0: float, range0: float) -> float:
    """Distance-dependent ranging noise standard deviation."""
    return sigma0 * (1.0 + (distance / range0) ** 2)


def build_ekf_raw_references(
    frame_like: dict,
    bases: list[list[float]],
    rng: np.random.Generator,
    *,
    sigma0: float = 0.5,
    range0: float = 850.0,
    availability_range0: float = 850.0,
) -> dict[int, list[dict]]:
    """Build raw range records from the fixed formation with soft noise.

    Each reference measurement is available with probability
    p(d) = 1 / (1 + (d / availability_range0)^2), so the effective ranging
    update rate decreases with distance (link-budget model).
    """
    formations = {
        entry["id"]: entry for entry in frame_like.get("formation", [])
    }
    positions = {
        robot["id"]: np.asarray(
            [robot["state"]["x"], robot["state"]["y"]], dtype=float
        )
        for robot in frame_like["robots"]
    }
    groups: dict[int, list[dict]] = {}
    for robot in frame_like["robots"]:
        robot_id = robot["id"]
        position = positions[robot_id]
        formation = formations.get(robot_id, {})
        references: list[dict] = []
        for base_id in formation.get("baseIds", []):
            base_position = np.asarray(bases[base_id], dtype=float)
            true_range = float(np.linalg.norm(position - base_position))
            sigma = ranging_sigma(true_range, sigma0, range0)
            availability = 1.0 / (
                1.0 + (true_range / availability_range0) ** 2
            )
            if rng.random() > availability:
                continue
            noise = float(rng.normal(0.0, sigma))
            references.append({
                "key": ("base", int(base_id)),
                "range": true_range + noise,
                "true_range": true_range,
                "ranging_sigma": sigma,
            })
        for anchor_id in formation.get("anchorIds", []):
            if anchor_id not in positions:
                continue
            anchor_position = positions[anchor_id]
            true_range = float(
                np.linalg.norm(position - anchor_position)
            )
            sigma = ranging_sigma(true_range, sigma0, range0)
            availability = 1.0 / (
                1.0 + (true_range / availability_range0) ** 2
            )
            if rng.random() > availability:
                continue
            noise = float(rng.normal(0.0, sigma))
            references.append({
                "key": ("uav", int(anchor_id)),
                "range": true_range + noise,
                "true_range": true_range,
                "ranging_sigma": sigma,
            })
        references.sort(key=lambda ref: ref["key"])
        groups[robot_id] = references
    return groups


class EKFInLoopService:
    """Stateful per-frame EKF service mirroring the in-loop driver."""

    def __init__(
        self,
        *,
        deployment_positions: dict[int, list[float]],
        bases: list[list[float]],
        sigma0: float = 0.5,
        range0: float = 850.0,
        process_noise_mps: float = 1.0,
        p0_std: float = 1.0,
        dt: float = 0.5,
        innovation_gate: float = 3.0,
        anchor_covariance_scale: float = 1.0,
    ) -> None:
        self.deployment_positions = deployment_positions
        self.bases = [np.asarray(base, dtype=float) for base in bases]
        self.sigma0 = sigma0
        self.range0 = range0
        self.process_noise_mps = process_noise_mps
        self.p0_std = p0_std
        self.dt = dt
        self.innovation_gate = innovation_gate
        self.anchor_covariance_scale = anchor_covariance_scale
        self.state = {
            robot_id: {
                "mean": np.asarray(
                    deployment_positions[robot_id], dtype=float
                ).copy(),
                "cov": np.eye(2) * p0_std**2,
                "updates": 0,
                "rejected": 0,
            }
            for robot_id in sorted(deployment_positions)
        }

    def reset(self) -> None:
        for robot_id in self.state:
            self.state[robot_id]["mean"] = np.asarray(
                self.deployment_positions[robot_id], dtype=float
            ).copy()
            self.state[robot_id]["cov"] = np.eye(2) * self.p0_std**2
            self.state[robot_id]["updates"] = 0
            self.state[robot_id]["rejected"] = 0

    def step(
        self,
        *,
        frame_index: int,
        raw_reference_groups: dict[int, list[dict]],
        held_commands: dict[int, list[float]],
    ) -> dict[int, dict]:
        outputs: dict[int, dict] = {}
        for robot_id in sorted(self.state):
            mean = self.state[robot_id]["mean"].copy()
            cov = self.state[robot_id]["cov"].copy()
            velocity = np.asarray(
                held_commands[robot_id], dtype=float
            )
            mean = mean + velocity * self.dt
            process_cov = (
                (self.process_noise_mps * self.dt) ** 2
            )
            cov = cov + np.eye(2) * process_cov
            updates = 0
            rejected = 0
            for reference in raw_reference_groups.get(robot_id, []):
                kind, identifier = reference["key"]
                if kind == "base":
                    anchor_mean = self.bases[identifier]
                    anchor_cov = np.zeros((2, 2))
                elif kind == "uav":
                    if identifier not in self.state:
                        continue
                    anchor_mean = self.state[identifier]["mean"]
                    anchor_cov = self.state[identifier]["cov"]
                else:
                    continue
                diff = mean - anchor_mean
                distance = float(np.linalg.norm(diff))
                if distance < 1e-6:
                    continue
                direction = diff / distance
                innovation = float(reference["range"]) - distance
                measurement_cov = (
                    float(reference["ranging_sigma"]) ** 2
                    + self.anchor_covariance_scale**2
                    * float(direction @ anchor_cov @ direction)
                )
                innovation_cov = (
                    float(direction @ cov @ direction)
                    + measurement_cov
                )
                if (
                    abs(innovation)
                    > self.innovation_gate
                    * float(np.sqrt(innovation_cov))
                ):
                    rejected += 1
                    continue
                gain = cov @ direction / innovation_cov
                mean = mean + gain * innovation
                cov = (
                    (np.eye(2) - np.outer(gain, direction)) @ cov
                )
                cov = (cov + cov.T) / 2.0
                updates += 1
            self.state[robot_id]["mean"] = mean
            self.state[robot_id]["cov"] = cov
            self.state[robot_id]["updates"] = updates
            self.state[robot_id]["rejected"] = rejected
            eigen = np.linalg.eigvalsh(cov)
            epsilon = 3.0 * float(
                np.sqrt(max(eigen[-1], 0.0))
            )
            outputs[robot_id] = {
                "estimate": [float(mean[0]), float(mean[1])],
                "epsilon": epsilon,
                "tier": "fresh" if updates > 0 else "coast",
                "updates": updates,
                "rejected": rejected,
            }
        return outputs
