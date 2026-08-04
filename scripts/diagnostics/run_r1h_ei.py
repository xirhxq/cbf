"""Python driver for the estimator-in-the-loop R1H-EI experiment.

Runs the Swarm simulator as a subprocess; at every control frame the simulator
writes truth state to a file and waits, the driver runs a basic extended
Kalman filter estimator for all robots, writes estimates back, and the
simulator applies them to its CBF construction.  This is the true closed loop
(estimates shape the control, control shapes the trajectory, trajectory
shapes the measurements).
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
from pathlib import Path

import numpy as np

from scripts.diagnostics.ekf_estimator_service import (
    EKFInLoopService,
    build_ekf_raw_references,
)
from scripts.diagnostics.run_diagnostic import materialize_config
from scripts.diagnostics.replay_r1h_estimator import _load_materialized_bases


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def _wait_for(path: Path, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if path.exists():
            return
        time.sleep(0.005)
    raise TimeoutError(f"timed out waiting for {path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--case-config", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--horizon", type=float, required=True)
    parser.add_argument("--seed", type=int, default=20260727)
    parser.add_argument("--range-noise-seed", type=int, default=2026081301)
    parser.add_argument(
        "--ranging-sigma0", type=float, default=0.5,
        help="short-range ranging noise sigma0 for the EKF measurement model",
    )
    parser.add_argument(
        "--initial-jitter-m", type=float, default=0.0,
        help="per-axis uniform jitter (m) added to the nominal deployment "
             "positions, deterministic in the simulation seed",
    )
    arguments = parser.parse_args()

    run_root = arguments.output_root
    run_root.mkdir(parents=True, exist_ok=True)
    config_path = run_root / "config.materialized.json"
    config = materialize_config(
        PROJECT_ROOT / "config" / "config.json",
        arguments.case_config,
        config_path,
        arguments.horizon,
        arguments.seed,
    )
    estimator_cfg = config["estimator-in-loop"]
    state_path = Path(estimator_cfg["state-path"])
    estimates_path = Path(estimator_cfg["estimates-path"])
    state_ready = Path(str(state_path) + ".ready")
    state_path.parent.mkdir(parents=True, exist_ok=True)
    for marker in (state_ready, estimates_path, state_path):
        marker.unlink(missing_ok=True)

    data = json.loads(config_path.read_text())
    if arguments.initial_jitter_m > 0.0:
        position_rng = np.random.default_rng(arguments.seed)
        positions = data["initial"]["position"]["positions"]
        jittered = [
            [
                float(x) + position_rng.uniform(
                    -arguments.initial_jitter_m, arguments.initial_jitter_m
                ),
                float(y) + position_rng.uniform(
                    -arguments.initial_jitter_m, arguments.initial_jitter_m
                ),
            ]
            for x, y in positions
        ]
        data["initial"]["position"]["positions"] = jittered
        config_path.write_text(json.dumps(data, indent=2) + "\n")
    bases = _load_materialized_bases({"config": data})
    deployment = {
        index + 1: [float(x) for x in position]
        for index, position in enumerate(
            data["initial"]["position"]["positions"]
        )
    }
    frames = int(round(arguments.horizon / 0.5))
    service = EKFInLoopService(
        deployment_positions=deployment,
        bases=bases,
        anchor_covariance_scale=3.0,
    )
    rng = np.random.default_rng(arguments.range_noise_seed)
    estimates_log = run_root / "estimates-log.jsonl"
    estimates_log_handle = estimates_log.open("a", buffering=1)

    process = subprocess.Popen(
        [str(arguments.binary), str(config_path)],
        stdout=(run_root / "stdout.log").open("wb"),
        stderr=(run_root / "stderr.log").open("wb"),
    )
    timing = []
    early_termination = False
    try:
        for frame in range(frames):
            if process.poll() is not None:
                early_termination = True
                break
            try:
                _wait_for(state_ready, 5.0)
            except TimeoutError:
                if process.poll() is not None:
                    early_termination = True
                    break
                raise
            state = json.loads(state_path.read_text())
            state_ready.unlink(missing_ok=True)
            assert state["frame_index"] == frame
            frame_like = {
                "robots": [
                    {
                        "id": entry["id"],
                        "state": {"x": entry["x"], "y": entry["y"]},
                    }
                    for entry in state["robots"]
                ],
                "covariance_formation": [
                    entry["covariance_formation"]
                    for entry in state["robots"]
                ],
                "formation": [
                    entry["formation"] for entry in state["robots"]
                ],
            }
            held = {
                entry["id"]: [entry["vx"], entry["vy"]]
                for entry in state["robots"]
            }
            references_by_robot = build_ekf_raw_references(
                frame_like, bases, rng,
                sigma0=arguments.ranging_sigma0,
            )
            started = time.monotonic()
            outputs = service.step(
                frame_index=frame,
                raw_reference_groups=references_by_robot,
                held_commands=held,
            )
            elapsed = time.monotonic() - started
            timing.append(elapsed)
            payload = {
                "frame_index": frame,
                "robots": [
                    {
                        "id": robot_id,
                        "estimate": outputs[robot_id]["estimate"],
                        "epsilon": outputs[robot_id]["epsilon"],
                        "tier": outputs[robot_id]["tier"],
                    }
                    for robot_id in sorted(outputs)
                ],
            }
            estimates_log_handle.write(json.dumps(payload) + "\n")
            tmp = estimates_path.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(payload) + "\n")
            os.replace(tmp, estimates_path)
        process.wait(timeout=30)
    except Exception:
        process.kill()
        raise

    (run_root / "estimator-timing.json").write_text(
        json.dumps({"per_frame_seconds": timing}, indent=1) + "\n"
    )
    termination_reason = None
    if early_termination:
        stdout_text = (run_root / "stdout.log").read_text(
            errors="ignore"
        )
        if "Coverage complete" in stdout_text:
            termination_reason = "coverage_complete"
        else:
            termination_reason = "sim_exited_early"
    (run_root / "termination.json").write_text(
        json.dumps(
            {
                "reason": termination_reason,
                "frames_served": len(timing),
                "returncode": process.returncode,
            },
            indent=1,
        )
        + "\n"
    )
    print("EI run finished, returncode", process.returncode)
    print("termination:", termination_reason, "frames_served", len(timing))
    print("frames", frames, "mean estimator solve s", round(float(np.mean(timing)), 4))
    return 0 if process.returncode == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
