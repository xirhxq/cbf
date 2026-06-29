#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
import pathlib
import sys

SCRIPT_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

from link_denied_experiments.run_full_simulator_link_denied_bridge import (
    run_link_denied_bridge_suite,
)


def build_geo_variant_base(base: dict, max_range: float) -> dict:
    variant = copy.deepcopy(base)
    comm_fixed = variant.setdefault("cbfs", {}).setdefault("without-slack", {}).setdefault("comm-fixed", {})
    comm_fixed["max-range"] = float(max_range)
    comm_fixed["range-tightening-margin"] = 10.0
    topology = variant.setdefault("bridge", {}).setdefault("topology", {})
    topology["max-range"] = float(max_range)
    variant["run_suffix"] = f"_bridge_geo_mr{int(max_range)}"
    return variant


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Geometry sweep for LinkDenied completion stress")
    parser.add_argument("--base", type=pathlib.Path, required=True)
    parser.add_argument("--binary", type=pathlib.Path, default=pathlib.Path("build-codex/Swarm"))
    parser.add_argument("--data-root", type=pathlib.Path, default=pathlib.Path("data"))
    parser.add_argument("--config-root", type=pathlib.Path, default=pathlib.Path("config/generated_geo_sweep"))
    parser.add_argument("--trials", type=int, default=6)
    parser.add_argument("--base-seed", type=int, default=20260617)
    parser.add_argument("--max-ranges", type=str, default="1125,1100,1075,1050")
    parser.add_argument("--state-reserve-base-margin", type=float, default=25.0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    base = json.loads(args.base.read_text(encoding="utf-8"))
    max_ranges = [float(v) for v in args.max_ranges.split(",") if v.strip()]
    summary_rows = []
    for max_range in max_ranges:
        variant = build_geo_variant_base(base, max_range)
        tag = f"mr{int(max_range)}"
        config_dir = args.config_root / tag
        output_dir = args.data_root / "link_denied_completion_geo_sweep_24" / tag
        data_dir = output_dir / "runs"
        data_dir.mkdir(parents=True, exist_ok=True)
        variant["output_path"] = str(data_dir.resolve())
        print(f"[geo-sweep] max-range={max_range} -> {output_dir}", flush=True)
        rows = run_link_denied_bridge_suite(
            base=variant,
            config_dir=config_dir,
            binary=args.binary,
            output_dir=output_dir,
            trials=args.trials,
            base_seed=args.base_seed,
            data_dir=data_dir,
            include_task_aware_reserve=True,
            include_task_aware_state_reserve=True,
            predictive_reserve_margin=25.0,
            state_reserve_base_margin=args.state_reserve_base_margin,
            completion_stress=True,
            dry_run=args.dry_run,
        )
        agg_path = output_dir / "link_denied_full_bridge_aggregate.csv"
        if agg_path.exists():
            summary_rows.append((max_range, agg_path))
    summary_path = args.data_root / "link_denied_completion_geo_sweep_24" / "geo_sweep_summary.csv"
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    with summary_path.open("w", encoding="utf-8") as fh:
        fh.write("max_range,aggregate_csv\n")
        for max_range, agg in summary_rows:
            fh.write(f"{max_range},{agg}\n")
    print(f"[geo-sweep] summary -> {summary_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
