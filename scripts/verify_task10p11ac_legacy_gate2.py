#!/usr/bin/env python3
"""Audit whether legacy D20/D22 evidence can satisfy Task 10.11ac Gate 2."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def load_finite(path: Path) -> dict:
    def reject(value: str) -> None:
        raise ValueError(f"non-finite JSON token {value} in {path}")

    return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def audit_profile(directory: Path, binary: Path) -> dict:
    result_path = directory / "result.json"
    result = load_finite(result_path)
    checkpoints = sorted((directory / "checkpoints").glob("*.json"))
    sparse = [path for path in checkpoints if path.name.startswith("sparse-")]
    packed = [path for path in checkpoints if path.name.startswith("checkpoint-")]
    all_checkpoints_finite = True
    for path in checkpoints:
        load_finite(path)
    trajectory_keys = {
        "applied_control_trajectory",
        "applied_controls_by_cycle",
        "owner_decision_ledger",
        "applied_control_digest_by_cycle",
    }
    present_trajectory_keys = sorted(trajectory_keys.intersection(result))
    expected_binary = result["source"]
    binary_matches = binary.is_file() and sha256(binary) == expected_binary[
        "binary_sha256"
    ]
    return {
        "profile": result["profile"],
        "result_sha256": sha256(result_path),
        "advanced_cycles": result["advanced_cycles"],
        "simulated_time_s": result["simulated_time_s"],
        "stop_reason": result["stop_reason"],
        "sparse_checkpoint_count": len(sparse),
        "packed_checkpoint_count": len(packed),
        "checkpoint_json_finite": all_checkpoints_finite,
        "legacy_binary_present_and_hash_matches": binary_matches,
        "per_cycle_applied_control_trajectory_present": bool(
            present_trajectory_keys
        ),
        "present_trajectory_keys": present_trajectory_keys,
        "scientific_state_sampling_note": (
            "Sparse checkpoints preserve restart state only every 10 simulated "
            "seconds; packed checkpoints preserve event frames. Neither is a "
            "2504/1666-cycle applied-control trajectory."
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--d20", type=Path, required=True)
    parser.add_argument("--d22", type=Path, required=True)
    parser.add_argument("--legacy-binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()

    profiles = [
        audit_profile(arguments.d20, arguments.legacy_binary),
        audit_profile(arguments.d22, arguments.legacy_binary),
    ]
    literal_gate2_pass = all(
        profile["per_cycle_applied_control_trajectory_present"]
        for profile in profiles
    )
    output = {
        "protocol": "task10p11ac-legacy-gate2-audit-v1",
        "valid": True,
        "profiles": profiles,
        "literal_applied_control_trajectory_comparison_available": (
            literal_gate2_pass
        ),
        "gate2_status": "pass" if literal_gate2_pass else "blocked",
        "reason": (
            "legacy_per_cycle_applied_control_trajectory_available"
            if literal_gate2_pass
            else "legacy_per_cycle_applied_control_trajectory_absent"
        ),
        "extra_legacy_reference_trajectory_run_authorized": False,
        "new_stage_zero_trajectory_started": False,
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(
        json.dumps(output, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(output, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
