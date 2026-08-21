#!/usr/bin/env python3
"""Freeze the Task 10.11ac eight-cell matrix before any trajectory."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


ORDER = [
    "I0-tau20",
    "I0-tau22",
    "P1-tau20",
    "P1-tau22",
    "P2-tau20",
    "P2-tau22",
    "P3-tau20",
    "P3-tau22",
]


def load_finite(path: Path) -> dict:
    def reject(value: str) -> None:
        raise ValueError(f"non-finite JSON token {value} in {path}")

    return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject)


def canonical_bytes(value: object) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def digest(value: object) -> str:
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--identity", type=Path, required=True)
    parser.add_argument("--parent-commit", required=True)
    parser.add_argument("--parent-tree", required=True)
    parser.add_argument("--cbf-commit", required=True)
    parser.add_argument("--cbf-tree", required=True)
    parser.add_argument("--binary-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--task-ae", action="store_true")
    arguments = parser.parse_args()

    manifest = load_finite(arguments.manifest)
    identity = load_finite(arguments.identity)
    if manifest["protocol"] != "task10p11ac-initialization-manifest-v1":
        raise ValueError("unexpected initialization manifest protocol")
    if manifest["matrix_order"] != ORDER or manifest["replacement_permitted"]:
        raise ValueError("initialization manifest is not the frozen matrix")
    if identity["protocol"] != "task10p11ac-identity-v1" or not identity["valid"]:
        raise ValueError("invalid Task 10.11ac identity")
    for value, length in (
        (arguments.parent_commit, 40),
        (arguments.parent_tree, 40),
        (arguments.cbf_commit, 40),
        (arguments.cbf_tree, 40),
        (arguments.binary_sha256, 64),
    ):
        if len(value) != length or any(character not in "0123456789abcdef" for character in value):
            raise ValueError("invalid source digest")

    profiles = {
        cell: {
            "initialization": cell[:2],
            "tau_mps2": float(cell[-2:]),
            "gamma_feedback_selection": "least_intervention_with_tau_unattained_maximum_margin_fallback",
        }
        for cell in ORDER
    }
    record_hashes = {
        identifier: manifest["initializations"][identifier]["record_sha256"]
        for identifier in ("I0", "P1", "P2", "P3")
    }
    base = identity["base_config_without_tau"]
    hard_gates = identity["hard_gates"]
    preregistration = {
        "protocol": (
            "task10p11ae-preregistration-v1"
            if arguments.task_ae
            else "task10p11ac-preregistration-v1"
        ),
        "frozen_before_first_trajectory": True,
        "profile_order": ORDER,
        "profiles": profiles,
        "candidate_count": 9,
        "maximum_cells": 8,
        "maximum_cumulative_wall_hours": 40.0,
        "maximum_wall_hours_per_cell": 6.0,
        "initialization_manifest_sha256": sha256_file(arguments.manifest),
        "initialization_record_sha256": record_hashes,
        "base_config_without_tau": base,
        "base_config_sha256_without_tau": digest(base),
        "hard_gates": hard_gates,
        "hard_gate_sha256": digest(hard_gates),
        "source": {
            "parent_commit": arguments.parent_commit,
            "parent_tree": arguments.parent_tree,
            "cbf_commit": arguments.cbf_commit,
            "cbf_tree": arguments.cbf_tree,
            "binary_sha256": arguments.binary_sha256,
        },
        "gate2": (
            {
                "full_same_binary_eight_cell_rerun": True,
                "i0_tau22_t100_required_before_perturbation_matrix": True,
                "task10p11ad_results_preserved_but_not_substituted": True,
                "task10p11ad_p2_tau20_is_shared_ledger_error": True,
                "extra_legacy_reference_trajectory_authorized": False,
            }
            if arguments.task_ae
            else {
                "requires_legacy_D20_D22_scientific_field_reproduction": True,
                "requires_per_cycle_applied_control_trajectory_comparison": True,
                "failure_stops_before_P1_P2_P3": True,
                "extra_legacy_reference_trajectory_authorized": False,
            }
        ),
        "prohibitions": {
            "additional_tau": True,
            "g1_g2": True,
            "dynamic_topology": True,
            "gain_variant": True,
            "mpc": True,
            "backup_cbf": True,
            "d1": True,
            "task11": True,
            "production_centralized_controller": True,
        },
    }
    if arguments.task_ae:
        preregistration["ledger"] = {
            "exact_owner_entries_per_advanced_cycle": 14,
            "statuses": [
                "valid_feedback_decision",
                "not_applicable_dynamic_pair_override",
                "not_applicable_other_frozen_control_path",
                "invalid",
            ],
            "branch_denominator": "feedback_applicable_owner_decisions_only",
            "observation_only": True,
        }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(
        json.dumps(preregistration, indent=2, sort_keys=True, allow_nan=False)
        + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
