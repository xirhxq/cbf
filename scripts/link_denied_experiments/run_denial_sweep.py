#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import csv
import json
import pathlib
import sys

SCRIPT_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

from link_denied_experiments.run_full_simulator_link_denied_bridge import (
    run_link_denied_bridge_suite,
)
from bridge_experiments import run_full_simulator_bridge_mc as _mc


def install_fixed_denial_override(dhx: float, dhy: float, center_x: float, center_y: float) -> None:
    def _fixed(_config, _rng):
        return [float(center_x), float(center_y)], [float(dhx), float(dhy)]

    _mc._sample_denial_zone = _fixed

AGG_COLS = [
    "row",
    "trial_count",
    "completion_count",
    "coverage_completion_count",
    "coverage_completion_rate",
    "certified_graph_ratio_mean",
    "certified_graph_ratio_min",
    "fail_safe_ratio_max",
    "fail_safe_steps_max",
    "solver_failures_max",
    "mean_qp_success_ratio",
]

MANIFEST_FIELDS = [
    "denial_half_x",
    "denial_half_y",
    "denial_center_x",
    "denial_center_y",
    "row",
    "trial",
    "seed",
    "source_data",
    "completed_horizon",
    "coverage_completed",
    "coverage_completion_time_s",
    "detected",
    "final_coverage",
    "min_physical_comm_margin",
    "certified_graph_ratio",
    "fail_safe_ratio",
    "min_support_chain_margin_after",
    "terminal_support_chain_margin_after",
    "support_chain_guard_active_ratio",
    "qp_success_ratio",
    "solver_failures",
]


def build_denial_variant_base(base: dict, denial_half_x: float, denial_half_y: float) -> dict:
    variant = copy.deepcopy(base)
    topology = variant.setdefault("bridge", {}).setdefault("topology", {})
    topology["denial-half-size"] = [float(denial_half_x), float(denial_half_y)]
    tag = f"dhx{int(denial_half_x)}dhy{int(denial_half_y)}"
    variant["run_suffix"] = f"_bridge_denial_{tag}"
    return variant


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Denial-zone sweep for LinkDenied completion stress")
    parser.add_argument("--base", type=pathlib.Path, required=True)
    parser.add_argument("--binary", type=pathlib.Path, default=pathlib.Path("build-codex/Swarm"))
    parser.add_argument("--data-root", type=pathlib.Path, default=pathlib.Path("data"))
    parser.add_argument(
        "--config-root",
        type=pathlib.Path,
        default=pathlib.Path("config/generated_denial_sweep"),
    )
    parser.add_argument("--trials", type=int, default=8)
    parser.add_argument("--base-seed", type=int, default=20260617)
    parser.add_argument(
        "--denial-half-x",
        type=str,
        default="480,560,640",
    )
    parser.add_argument(
        "--denial-half-y",
        type=str,
        default="700,820,940",
    )
    parser.add_argument(
        "--denial-center-x",
        type=str,
        default="",
        help="Optional fixed denial-center x values (same length as half-x). Defaults to base center.",
    )
    parser.add_argument(
        "--denial-center-y",
        type=str,
        default="",
        help="Optional fixed denial-center y values (same length as half-y). Defaults to base center.",
    )
    parser.add_argument("--max-range", type=float, default=None, help="Override comm max-range for all cells.")
    parser.add_argument("--state-reserve-base-margin", type=float, default=25.0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    base = json.loads(args.base.read_text(encoding="utf-8"))
    if args.max_range is not None:
        comm = base.setdefault("cbfs", {}).setdefault("without-slack", {}).setdefault("comm-fixed", {})
        comm["max-range"] = float(args.max_range)
        base.setdefault("bridge", {}).setdefault("topology", {})["max-range"] = float(args.max_range)
    xs = [float(v) for v in args.denial_half_x.split(",") if v.strip()]
    ys = [float(v) for v in args.denial_half_y.split(",") if v.strip()]
    if len(xs) != len(ys):
        raise SystemExit("denial-half-x and denial-half-y must have the same length")
    base_topology = base.setdefault("bridge", {}).setdefault("topology", {})
    base_cx = float(base_topology.get("denial-center", [1650.0, 1500.0])[0])
    base_cy = float(base_topology.get("denial-center", [1650.0, 1500.0])[1])
    cxs = [float(v) for v in args.denial_center_x.split(",") if v.strip()] if args.denial_center_x else [base_cx] * len(xs)
    cys = [float(v) for v in args.denial_center_y.split(",") if v.strip()] if args.denial_center_y else [base_cy] * len(ys)
    if len(cxs) != len(xs) or len(cys) != len(xs):
        raise SystemExit("denial-center-x/y must match denial-half-x length when provided")
    sweep = list(zip(xs, ys, cxs, cys))

    pkg_root = args.data_root / "link_denied_completion_denial_sweep_24"
    pkg_root.mkdir(parents=True, exist_ok=True)
    summary_rows = []
    manifest_rows = []

    for dhx, dhy, dcx, dcy in sweep:
        variant = build_denial_variant_base(base, dhx, dhy)
        topology = variant.setdefault("bridge", {}).setdefault("topology", {})
        topology["denial-center"] = [float(dcx), float(dcy)]
        install_fixed_denial_override(dhx, dhy, float(dcx), float(dcy))
        tag = f"dhx{int(dhx)}dhy{int(dhy)}cx{int(dcx)}cy{int(dcy)}"
        config_dir = args.config_root / tag
        output_dir = pkg_root / tag
        data_dir = output_dir / "runs"
        data_dir.mkdir(parents=True, exist_ok=True)
        variant["output_path"] = str(data_dir.resolve())
        print(f"[denial-sweep] half=({dhx},{dhy}) center=({dcx},{dcy}) -> {output_dir}", flush=True)
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
        trials_path = output_dir / "link_denied_full_bridge_trials.csv"
        if agg_path.exists():
            summary_rows.append((dhx, dhy, dcx, dcy, agg_path))
        if trials_path.exists():
            with trials_path.open() as fh:
                for rec in csv.DictReader(fh):
                    out = {
                        "denial_half_x": f"{dhx:.0f}",
                        "denial_half_y": f"{dhy:.0f}",
                        "denial_center_x": f"{dcx:.0f}",
                        "denial_center_y": f"{dcy:.0f}",
                    }
                    for f in MANIFEST_FIELDS[4:]:
                        out[f] = rec.get(f, "")
                    out["row"] = rec.get("row", "")
                    out["source_data"] = rec.get("source_data", "")
                    manifest_rows.append(out)

    summary_path = pkg_root / "denial_sweep_summary.csv"
    with summary_path.open("w", encoding="utf-8") as fh:
        fh.write("denial_half_x,denial_half_y,denial_center_x,denial_center_y,aggregate_csv\n")
        for dhx, dhy, dcx, dcy, agg in summary_rows:
            fh.write(f"{dhx:.0f},{dhy:.0f},{dcx:.0f},{dcy:.0f},{agg}\n")
    print(f"[denial-sweep] summary -> {summary_path}", flush=True)

    manifest_path = pkg_root / "source_data_manifest.csv"
    with manifest_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=MANIFEST_FIELDS)
        writer.writeheader()
        for r in manifest_rows:
            writer.writerow(r)
    print(f"[denial-sweep] manifest -> {manifest_path} ({len(manifest_rows)} rows)", flush=True)

    table_path = pkg_root / "denial_sweep_completion_table.csv"
    table_header = ["denial_half_x", "denial_half_y", "denial_center_x", "denial_center_y"] + AGG_COLS
    with table_path.open("w", encoding="utf-8") as fh:
        fh.write(",".join(table_header) + "\n")
        for dhx, dhy, dcx, dcy, agg in summary_rows:
            with agg.open() as fh2:
                for rec in csv.DictReader(fh2):
                    row_vals = [f"{dhx:.0f}", f"{dhy:.0f}", f"{dcx:.0f}", f"{dcy:.0f}"] + [
                        rec.get(c, "") for c in AGG_COLS
                    ]
                    fh.write(",".join(row_vals) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
