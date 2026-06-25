#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
import pathlib
import subprocess
from typing import Any


ROW_SETTINGS: dict[str, dict[str, Any]] = {
    "R1": {
        "search-policy": "coverage",
        "topology-policy": "fixed",
        "safety-filter": "first-order-cbf",
        "model": "SingleIntegrate2D",
        "high-order": False,
        "sampled-data-reserve": 0.0,
        "time-step": 1.0,
        "safety-tightening-margin": 5.0,
    },
    "R2": {
        "search-policy": "active",
        "topology-policy": "fixed",
        "safety-filter": "first-order-cbf",
        "model": "SingleIntegrate2D",
        "high-order": False,
        "sampled-data-reserve": 0.0,
        "time-step": 1.0,
        "safety-tightening-margin": 5.0,
    },
    "R3": {
        "search-policy": "active",
        "topology-policy": "adaptive-relay",
        "safety-filter": "first-order-cbf",
        "model": "SingleIntegrate2D",
        "high-order": False,
        "sampled-data-reserve": 0.0,
        "time-step": 1.0,
        "safety-tightening-margin": 5.0,
    },
    "R4": {
        "search-policy": "active",
        "topology-policy": "adaptive-relay",
        "safety-filter": "second-order-hocbf",
        "model": "DoubleIntegrate2D",
        "high-order": True,
        "sampled-data-reserve": 2.0,
        "time-step": 0.5,
        "safety-tightening-margin": 0.0,
        "acceleration-bound": 8.0,
    },
}


def build_row_config(base: dict[str, Any], row: str) -> dict[str, Any]:
    if row not in ROW_SETTINGS:
        raise ValueError(f"Unknown bridge row: {row}")
    config = copy.deepcopy(base)
    settings = ROW_SETTINGS[row]
    config["bridge"]["row"] = row
    config["bridge"]["search-policy"] = settings["search-policy"]
    config["bridge"]["topology-policy"] = settings["topology-policy"]
    config["bridge"]["safety-filter"] = settings["safety-filter"]
    config["model"] = settings["model"]
    config["cbfs"]["high-order"]["enabled"] = settings["high-order"]
    config["cbfs"]["high-order"]["sampled-data-reserve"] = settings["sampled-data-reserve"]
    if "acceleration-bound" in settings:
        config["cbfs"]["high-order"]["acceleration-bound"] = settings["acceleration-bound"]
    config["execute"]["time-step"] = settings["time-step"]
    config["cbfs"]["without-slack"]["safety"]["safe-distance-tightening-margin"] = settings["safety-tightening-margin"]
    config["run_suffix"] = f"_bridge_{row.lower()}"
    if settings["model"] == "DoubleIntegrate2D":
        config["dim"] = 6
        config["initial"]["velocity"]["values"] = [[0.0, 0.0] for _ in range(config["num"])]
    else:
        config["dim"] = 4
        config["initial"].pop("velocity", None)
    return config


def write_row_configs(base: dict[str, Any], output_dir: pathlib.Path) -> dict[str, pathlib.Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    paths: dict[str, pathlib.Path] = {}
    for row in ("R1", "R2", "R3", "R4"):
        path = output_dir / f"bridge_{row.lower()}.json"
        path.write_text(json.dumps(build_row_config(base, row), indent=2), encoding="utf-8")
        paths[row] = path
    return paths


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate and run full-simulator bridge rows")
    parser.add_argument("--base", type=pathlib.Path, default=pathlib.Path("config/bridge_full_simulator_base.json"))
    parser.add_argument("--config-dir", type=pathlib.Path, default=pathlib.Path("config/generated_bridge"))
    parser.add_argument("--binary", type=pathlib.Path, default=pathlib.Path("build-codex/Swarm"))
    parser.add_argument("--rows", nargs="+", default=["R1", "R2", "R3", "R4"])
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    base = json.loads(args.base.read_text(encoding="utf-8"))
    paths = write_row_configs(base, args.config_dir)
    for row in args.rows:
        if row not in paths:
            raise ValueError(f"Unknown bridge row: {row}")
        if args.dry_run:
            print(paths[row])
            continue
        subprocess.run([str(args.binary), str(paths[row])], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
