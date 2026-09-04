#!/usr/bin/env python3
"""Rebuild Task 23 residuals and deadline dynamics from first-party logs."""

import argparse
import json
from collections import Counter
from pathlib import Path


def decode_mask(value, count):
    return [bool(int(value[index // 4], 16) & (1 << (index % 4)))
            for index in range(count)]


def quantiles(values):
    ordered = sorted(values)
    if not ordered:
        return {}
    return {str(percent): ordered[round((len(ordered) - 1) * percent / 100)]
            for percent in (0, 5, 25, 50, 75, 95, 100)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("oracle_dir", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    deltas = [json.loads(line) for line in
              (args.run_dir / "progress/gridworld-delta.jsonl").read_text().splitlines()
              if line.strip()]
    initial, final = deltas[0], deltas[-1]
    count = final["valid_count"]
    x_cells = initial["x_cells"]
    y_cells = initial["y_cells"]
    mask = decode_mask(final["certified_bits_hex"], count)
    residual = [f"{index // y_cells}:{index % y_cells}"
                for index, covered in enumerate(mask) if not covered]
    witness = {}
    for line in (args.oracle_dir / "witnesses.jsonl").read_text().splitlines():
        row = json.loads(line)
        witness[row["cell_id"]] = row
    telemetry = [json.loads(line) for line in
                 (args.run_dir / "telemetry.jsonl").read_text().splitlines()
                 if line.strip()]
    evaluated = [row for row in telemetry
                 if row["task23"]["allocation_evaluated"]]
    assignments = [assignment for row in evaluated
                   for assignment in row["task23"]["assignments"]]
    last_new_time = max((row.get("runtime_s", 0.0) for row in deltas[1:-1]
                         if row.get("certified_new_ids")), default=0.0)
    residual_witness = [witness[item] for item in residual if item in witness]
    result = {
        "protocol": "task23-deadline-run-reconstruction-v1",
        "run_dir": str(args.run_dir),
        "oracle_dir": str(args.oracle_dir),
        "final_certified_count_rebuilt": count - len(residual),
        "residual_count": len(residual),
        "residual_ids": residual,
        "residual_x_quantiles": quantiles(
            [int(item.split(":")[0]) for item in residual]),
        "residual_y_quantiles": quantiles(
            [int(item.split(":")[1]) for item in residual]),
        "residual_pass_counts": dict(sorted(Counter(
            row["pass_index"] for row in residual_witness).items())),
        "residual_tier_counts": dict(Counter(
            row["tier"] for row in residual_witness)),
        "residual_sensing_margin_quantiles_m": quantiles(
            [row["sensing_margin_m"] for row in residual_witness]),
        "residual_reference_quantiles_m": quantiles(
            [row["maximum_target_reference_m"] for row in residual_witness]),
        "last_new_certified_time_s": last_new_time,
        "allocation_evaluations": len(evaluated),
        "unit_evaluations": len(assignments),
        "route_invalid_unit_evaluations": sum(
            not item["route_valid"] for item in assignments),
        "holding_unit_evaluations": sum(item["holding"] for item in assignments),
        "covered_deadlines_skipped": sum(
            row["task23"]["covered_deadlines_skipped"] for row in evaluated),
        "last_assignments": evaluated[-1]["task23"]["assignments"],
        "boundary": "exact mask/delta reconstruction; no interpolation and no "
                    "counterfactual plant replay"
    }
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
