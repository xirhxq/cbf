#!/usr/bin/env python3
"""Classify whether frozen aggregate evidence can attribute coverage by UAV."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


PER_OWNER_KEYS = {
    "per_owner_new_truth_cells",
    "per_owner_unique_truth_cells",
    "per_owner_leave_one_out_union_loss",
    "owner_coverage_contributions",
    "owner_footprints",
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def find_keys(value: object, path: str = "$") -> list[str]:
    found: list[str] = []
    if isinstance(value, dict):
        for key, child in value.items():
            child_path = f"{path}.{key}"
            if key in PER_OWNER_KEYS:
                found.append(child_path)
            found.extend(find_keys(child, child_path))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            found.extend(find_keys(child, f"{path}[{index}]"))
    return found


def target_summary(evidence: dict) -> dict:
    ledger = evidence.get("target_ledger", {})
    coordinates = [tuple(value) for value in ledger.values()]
    return {
        "owner_target_count": len(ledger),
        "unique_target_count": len(set(coordinates)),
        "target_epoch": evidence.get("target_epoch"),
    }


def robust_reference_span(evidence: dict) -> float | None:
    direct = evidence.get("maximum_robust_reference_distance_m")
    if isinstance(direct, (int, float)):
        return float(direct)
    minimum_h = evidence.get("minimum_reference_h_m")
    if isinstance(minimum_h, (int, float)):
        return 850.0 - float(minimum_h)
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("maximum", type=Path)
    parser.add_argument("least_intervention", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    maximum = json.loads(args.maximum.read_text())
    least = json.loads(args.least_intervention.read_text())
    maximum_fields = find_keys(maximum)
    least_fields = find_keys(least)
    attributable = bool(maximum_fields and least_fields)
    result = {
        "protocol": "task10p11t-existing-coverage-attribution-v1",
        "trajectory_run_performed": False,
        "attribution_decidable": attributable,
        "classification": (
            "per_owner_attribution_available"
            if attributable
            else "undetermined_missing_per_owner_footprints"
        ),
        "scientific_claim": (
            "Existing aggregate evidence cannot distinguish global conservative "
            "unfolding from one or a few low-contribution UAVs."
        ),
        "maximum": {
            "sha256": sha256(args.maximum),
            "truth_coverage": maximum.get("truth_coverage"),
            "covered_cells": maximum.get("covered_cells"),
            "denominator_cells": maximum.get("denominator_cells"),
            "maximum_robust_reference_distance_m":
                robust_reference_span(maximum),
            "per_owner_evidence_fields": maximum_fields,
            **target_summary(maximum),
        },
        "least_intervention_tau14": {
            "sha256": sha256(args.least_intervention),
            "truth_coverage": least.get("truth_coverage"),
            "covered_cells": least.get("covered_cells"),
            "denominator_cells": least.get("denominator_cells"),
            "maximum_robust_reference_distance_m":
                robust_reference_span(least),
            "per_owner_evidence_fields": least_fields,
        },
        "weak_noncausal_indicators": {
            "all_14_owners_have_distinct_final_targets": (
                target_summary(maximum)["owner_target_count"] == 14
                and target_summary(maximum)["unique_target_count"] == 14
            ),
            "allocator_continued_retargeting": (
                isinstance(maximum.get("target_epoch"), int)
                and maximum["target_epoch"] > 0
            ),
            "maximum_reference_span_is_smaller_than_tau14": (
                robust_reference_span(maximum) is not None
                and robust_reference_span(least) is not None
                and robust_reference_span(maximum) < robust_reference_span(least)
            ),
        },
        "minimal_future_measurement_if_a_run_is_later_authorized": [
            "per-owner newly covered truth cells per cycle",
            "order-independent leave-one-owner-out union loss",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
