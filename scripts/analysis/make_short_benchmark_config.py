#!/usr/bin/env python3
"""Create a reproducible short-run Swarm benchmark config."""

import argparse
import copy
import json
from pathlib import Path
from typing import Any


def build_config(
    base_config: dict[str, Any],
    *,
    optimiser: str,
    execution_mode: str,
    time_total: float,
    random_seed: int,
    output_path: str,
    run_suffix: str,
) -> dict[str, Any]:
    config = copy.deepcopy(base_config)
    execute = config.setdefault("execute", {})

    config["optimiser"] = optimiser
    execute["execution-mode"] = execution_mode
    execute["time-total"] = time_total
    execute["random-seed"] = random_seed
    config["output_path"] = output_path
    config["run_suffix"] = run_suffix

    return config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a fixed-seed short benchmark config from a Swarm base config."
    )
    parser.add_argument("base", type=Path, help="Base Swarm config JSON path")
    parser.add_argument("output", type=Path, help="Output benchmark config JSON path")
    parser.add_argument("--optimiser", default="OSQP")
    parser.add_argument("--execution-mode", default="distributed")
    parser.add_argument("--time-total", type=float, default=5.0)
    parser.add_argument("--random-seed", type=int, default=20260611)
    parser.add_argument("--output-path", default="/private/tmp/cbf_benchmark_verify")
    parser.add_argument("--run-suffix", default="_benchmark")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    base_config = json.loads(args.base.read_text())
    benchmark_config = build_config(
        base_config,
        optimiser=args.optimiser,
        execution_mode=args.execution_mode,
        time_total=args.time_total,
        random_seed=args.random_seed,
        output_path=args.output_path,
        run_suffix=args.run_suffix,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(benchmark_config, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
