#!/usr/bin/env python3
"""Independent verifier for the Task 10.11u 132.9 s evidence gate."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


EXPECTED_MISSING = {
    "runtime",
    "estimator",
    "dekf_internal",
    "canonical_request",
    "actual_rows",
    "owner_row_counts",
    "nominal_controls",
    "objective_28d",
    "successor_parameters",
    "preflight",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify(input_path: Path, result_path: Path, binary_path: Path) -> dict:
    frozen = json.loads(input_path.read_text())
    result = json.loads(result_path.read_text())
    source = result["source"]
    actual_missing = {field for field in EXPECTED_MISSING if field not in frozen}
    assertions = {
        "input_is_exact_132p9_stop": (
            frozen["protocol"] == "task10p11t-online-pair-v1"
            and frozen["reason"]
            == "pair_coordination_failed:dynamic_pair_responsibility_interval_empty"
            and math.isclose(float(frozen["simulated_time_s"]),
                             132.89999999999674, abs_tol=1e-9)
            and frozen["active_pair_base_id"] == "reference:2->4"
        ),
        "input_hash_matches_record": source["input_sha256"] == sha256(input_path),
        "binary_hash_matches_record": source["binary_sha256"] == sha256(binary_path),
        "source_hashes_well_formed": all(
            len(source[field]) == 40
            for field in ("parent_commit", "parent_tree", "cbf_commit", "cbf_tree")
        ),
        "missing_fields_independently_confirmed": (
            actual_missing == EXPECTED_MISSING
            and set(result["snapshot_preflight"]["missing_fields"])
            == EXPECTED_MISSING
            and result["snapshot_preflight"]["complete"] is False
        ),
        "local_signed_transfer_only_is_determined": (
            result["local_signed_transfer"]["determined"] is True
            and result["local_signed_transfer"]["feasible"] is False
            and result["local_signed_transfer"]["status"] == "infeasible"
        ),
        "full_pair_not_claimed": (
            result["current_full_pair_28d"]["performed"] is False
            and result["current_full_pair_28d"]["status"] == "undetermined"
            and result["independent_full_row_residual_recompute"]["performed"]
            is False
        ),
        "successor_not_claimed": (
            result["successor"]["performed"] is False
            and result["successor"]["status"] == "undetermined"
        ),
        "component_reduction_not_claimed": (
            result["pair_2_4_component_4d"]["performed"] is False
            and result["pair_2_4_component_4d"]["status"] == "undetermined"
            and result["pair_2_4_component_4d"][
                "equivalent_reduction_established"] is False
        ),
        "registered_stop_reason": (
            result["reason"]
            == "frozen_132p9_snapshot_incomplete_for_full_28d"
        ),
        "no_trajectory_controller_or_task11": (
            result["trajectory_run_performed"] is False
            and result["production_controller_modified"] is False
            and result["task11_performed"] is False
        ),
    }
    return {
        "protocol": "task10p11u-independent-evidence-gate-v1",
        "passed": all(assertions.values()),
        "assertions": assertions,
        "input_sha256": sha256(input_path),
        "result_sha256": sha256(result_path),
        "binary_sha256": sha256(binary_path),
        "claim_boundary": (
            "local_signed_transfer_infeasible_current_full_pair_successor_"
            "and_4d_component_undetermined"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("result", type=Path)
    parser.add_argument("binary", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    output = verify(args.input, args.result, args.binary)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n")
    print(json.dumps(output, indent=2, sort_keys=True))
    return 0 if output["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
