#!/usr/bin/env python3
"""Run repeated fixed-seed Swarm benchmarks and summarize rationality metrics."""

import argparse
import hashlib
import json
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from make_short_benchmark_config import build_config
from validate_rationality import RationalityOptions, load_json, validate_data


@dataclass(frozen=True)
class BenchmarkRun:
    index: int
    elapsed_seconds: float
    output_dir: Path
    data_json: Path
    rationality_passed: bool
    state_checksum: str
    metrics: dict[str, float]


def parse_output_dir(stdout: str) -> Path:
    matches = re.findall(r"^\[OUTPUT_DIR\]\s+(.+)$", stdout, flags=re.MULTILINE)
    if not matches:
        raise ValueError("Swarm output did not contain an [OUTPUT_DIR] marker")
    return Path(matches[-1].strip())


def state_checksum(data: dict[str, Any]) -> str:
    payload = json.dumps(data.get("state", []), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def series_stats(values: list[float]) -> dict[str, float]:
    if not values:
        return {"mean": 0.0, "min": 0.0, "max": 0.0}
    return {
        "mean": sum(values) / len(values),
        "min": min(values),
        "max": max(values),
    }


def summarize_runs(runs: list[BenchmarkRun]) -> dict[str, Any]:
    metric_names = sorted({name for run in runs for name in run.metrics})
    checksums = {run.state_checksum for run in runs}
    return {
        "runs": len(runs),
        "all_rational": all(run.rationality_passed for run in runs),
        "state_reproducible": len(checksums) <= 1,
        "elapsed_seconds": series_stats([run.elapsed_seconds for run in runs]),
        "metrics": {
            name: series_stats([run.metrics[name] for run in runs if name in run.metrics])
            for name in metric_names
        },
    }


def write_run_config(
    base_config: dict[str, Any],
    config_path: Path,
    *,
    optimiser: str,
    execution_mode: str,
    time_total: float,
    random_seed: int,
    output_path: str,
    run_suffix: str,
) -> None:
    config = build_config(
        base_config,
        optimiser=optimiser,
        execution_mode=execution_mode,
        time_total=time_total,
        random_seed=random_seed,
        output_path=output_path,
        run_suffix=run_suffix,
    )
    config_path.write_text(json.dumps(config, indent=2) + "\n")


def run_once(index: int, swarm_bin: Path, config_path: Path, cwd: Path) -> BenchmarkRun:
    started = time.perf_counter()
    completed = subprocess.run(
        [str(swarm_bin), str(config_path)],
        cwd=str(cwd),
        capture_output=True,
        text=True,
        check=False,
    )
    elapsed = time.perf_counter() - started

    if completed.returncode != 0:
        sys.stderr.write(completed.stdout)
        sys.stderr.write(completed.stderr)
        raise RuntimeError(f"Swarm run {index} failed with exit code {completed.returncode}")

    output_dir = parse_output_dir(completed.stdout)
    data_json = output_dir / "data.json"
    data = load_json(data_json)
    rationality = validate_data(data, RationalityOptions())
    return BenchmarkRun(
        index=index,
        elapsed_seconds=elapsed,
        output_dir=output_dir,
        data_json=data_json,
        rationality_passed=rationality.passed,
        state_checksum=state_checksum(data),
        metrics=rationality.metrics,
    )


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run repeated fixed-seed Swarm benchmarks")
    parser.add_argument("base_config", type=Path, help="Base Swarm config JSON path")
    parser.add_argument("--swarm-bin", type=Path, default=Path("./cmake-build-release/Swarm"))
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--optimiser", default="OSQP")
    parser.add_argument("--execution-mode", default="distributed")
    parser.add_argument("--time-total", type=float, default=5.0)
    parser.add_argument("--random-seed", type=int, default=20260611)
    parser.add_argument("--output-path", default="/private/tmp/cbf_repro_benchmark")
    parser.add_argument("--run-suffix", default="_benchmark")
    parser.add_argument("--summary-json", type=Path)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv or sys.argv[1:])
    if args.runs < 1:
        raise ValueError("--runs must be at least 1")

    output_path = Path(args.output_path)
    output_path.mkdir(parents=True, exist_ok=True)
    base_config = load_json(args.base_config)
    runs = []

    for index in range(1, args.runs + 1):
        config_path = output_path / f"benchmark_config_{index:02d}.json"
        write_run_config(
            base_config,
            config_path,
            optimiser=args.optimiser,
            execution_mode=args.execution_mode,
            time_total=args.time_total,
            random_seed=args.random_seed,
            output_path=str(output_path),
            run_suffix=f"{args.run_suffix}_{index:02d}",
        )
        run = run_once(index, args.swarm_bin, config_path, Path.cwd())
        runs.append(run)
        status = "PASS" if run.rationality_passed else "FAIL"
        print(
            f"run {run.index}: elapsed={run.elapsed_seconds:.4f}s "
            f"rationality={status} output={run.output_dir}"
        )

    summary = summarize_runs(runs)
    summary_text = json.dumps(summary, indent=2, sort_keys=True)
    if args.summary_json:
        args.summary_json.write_text(summary_text + "\n")
    print(summary_text)
    return 0 if summary["all_rational"] and summary["state_reproducible"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
