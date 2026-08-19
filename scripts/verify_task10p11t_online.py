#!/usr/bin/env python3
"""Read-only verifier for the unique Task 10.11t online pair run."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


TOL = 1.0e-7


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify(
    online_path: Path,
    gate_b_path: Path,
    least_intervention_path: Path,
    binary_path: Path,
) -> dict:
    online = json.loads(online_path.read_text())
    gate_b = json.loads(gate_b_path.read_text())
    least = json.loads(least_intervention_path.read_text())
    trace = online["responsibility_trace"]
    progress = online["coverage_progress"]
    source = online["source"]
    intervals_valid = all(
        float(entry["interval_lower_mps2"]) - TOL
        <= float(entry["selected_transfer_mps2"])
        <= float(entry["interval_upper_mps2"]) + TOL
        for entry in trace
    )
    rows_valid = all(
        float(entry["minimum_local_residual_mps2"]) >= -TOL
        and float(entry["once_reserve_full_pair_residual_mps2"]) >= -TOL
        for entry in trace
    )
    coverage_monotone = all(
        float(right["truth_coverage"]) + 1.0e-12
        >= float(left["truth_coverage"])
        for left, right in zip(progress, progress[1:])
    )
    first_trace_time = (
        float(trace[0]["decision_time_s"]) if trace else math.nan
    )
    first_intervention = online["first_intervention_time_s"]
    terminal_consistent = (
        online["outcome"] == "t100"
        and online["reason"] == "t100_reached"
        and math.isclose(float(online["truth_coverage"]), 1.0,
                         abs_tol=1.0e-12)
        and online["t100_true_s"] is not None
    ) or (
        online["outcome"] != "t100"
        and online["reason"] != "t100_reached"
    )
    assertions = {
        "protocol_and_unique_run": (
            online["protocol"] == "task10p11t-online-pair-v1"
            and online["unique_run_index"] == 1
        ),
        "binary_hash_matches": (
            source["binary_sha256"] == sha256(binary_path)
        ),
        "source_hashes_well_formed": all(
            len(source[field]) == 40
            for field in ("parent_commit", "parent_tree",
                          "cbf_commit", "cbf_tree")
        ),
        "frozen_config_matches_gate_b": (
            online["frozen_model_config_digest"]
            == gate_b["frozen_model_config_digest"]
        ),
        "authority_and_topology_unchanged": (
            online["authority_commit"] == gate_b["authority_commit"]
            and online["topology_canonical"]
            == gate_b["topology_canonical"]
            and online["topology_frozen"] is True
        ),
        "least_intervention_tau14_before_pair": (
            online["legacy_selection_mode_before_intervention"]
            == "least_intervention"
            and math.isclose(float(online["predictive_tau_mps2"]), 14.0,
                             abs_tol=1.0e-12)
        ),
        "intervention_matches_frozen_failure_time": (
            first_intervention is not None
            and math.isclose(float(first_intervention),
                             float(least["simulated_time_s"]),
                             abs_tol=1.0e-9)
        ),
        "one_pair_and_every_cycle_recomputed": (
            len(trace) == online["responsibility_cycles"]
            and len(trace) > 0
            and online["active_pair_base_id"] == "reference:2->4"
            and all(entry["pair_base_id"] == "reference:2->4"
                    for entry in trace)
            and math.isclose(first_trace_time, float(first_intervention),
                             abs_tol=1.0e-12)
        ),
        "all_intervals_contain_selected_transfer": intervals_valid,
        "all_recorded_dynamic_rows_and_full_pair_safe": rows_valid,
        "reported_dynamic_minima_match_trace": (
            math.isclose(
                float(online["minimum_dynamic_local_residual_mps2"]),
                min(float(entry["minimum_local_residual_mps2"])
                    for entry in trace),
                abs_tol=1.0e-12,
            )
            and math.isclose(
                float(online[
                    "minimum_once_reserve_full_pair_residual_mps2"]),
                min(float(entry[
                    "once_reserve_full_pair_residual_mps2"])
                    for entry in trace),
                abs_tol=1.0e-12,
            )
        ),
        "hard_safety_minima_nonnegative": (
            float(online["minimum_robust_hard_residual_mps2"]) >= -TOL
            and float(online["minimum_collision_residual_mps2"]) >= -TOL
            and float(online["minimum_reference_residual_mps2"]) >= -TOL
            and float(online["minimum_truth_mobile_mobile_distance_m"])
                >= 10.0 - 1.0e-9
            and float(online["minimum_truth_mobile_fixed_distance_m"])
                >= 10.0 - 1.0e-9
            and float(online["minimum_truth_speed_margin_mps"])
                >= -1.0e-9
        ),
        "coverage_progress_monotone_and_terminal_matches": (
            coverage_monotone
            and math.isclose(float(progress[-1]["truth_coverage"]),
                             float(online["truth_coverage"]),
                             abs_tol=1.0e-12)
        ),
        "terminal_outcome_consistent": terminal_consistent,
        "no_multi_pair_conflict_hidden": (
            len(online["multi_pair_conflict_events"]) == 0
            or online["reason"] == "multiple_mobile_pair_conflict"
        ),
        "no_scan_second_run_or_task11": (
            online["parameters_scanned"] is False
            and online["second_run_performed"] is False
            and online["task11_performed"] is False
            and online["recursive_feasibility_claimed"] is False
        ),
    }
    return {
        "protocol": "task10p11t-online-independent-verifier-v1",
        "passed": all(assertions.values()),
        "assertions": assertions,
        "online_evidence_sha256": sha256(online_path),
        "gate_b_evidence_sha256": sha256(gate_b_path),
        "least_intervention_evidence_sha256": sha256(
            least_intervention_path
        ),
        "binary_sha256": sha256(binary_path),
        "first_intervention_time_s": first_intervention,
        "responsibility_cycles": online["responsibility_cycles"],
        "final_time_s": online["simulated_time_s"],
        "final_coverage": online["truth_coverage"],
        "final_reason": online["reason"],
        "claim_boundary": (
            "one_unique_deterministic_online_run_not_recursive_feasibility"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("online", type=Path)
    parser.add_argument("gate_b", type=Path)
    parser.add_argument("least_intervention", type=Path)
    parser.add_argument("binary", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    result = verify(
        args.online, args.gate_b, args.least_intervention, args.binary
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
