#!/usr/bin/env python3
"""Independently rebuild Task 10.11s full-pair rows and residuals."""

import argparse
import hashlib
import json
import math
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def pair_base(row_id: str) -> str:
    marker = ":owner:"
    if marker not in row_id:
        raise ValueError(f"mobile pair row lacks owner suffix: {row_id}")
    return row_id.rsplit(marker, 1)[0]


def rebuild_rows(snapshot):
    owners = snapshot["canonical_request"]["mobile_ids"]
    offsets = {owner: 2 * index for index, owner in enumerate(owners)}
    if owners != list(range(1, 15)):
        raise ValueError("unexpected 14-owner ordering")
    pairs = {}
    rebuilt = []
    for row in snapshot["actual_rows"]:
        peer = row["peer"]
        pair_kind = row["kind"] in (0, 1)
        if pair_kind and peer in offsets:
            pairs.setdefault(pair_base(row["id"]), []).append(row)
            continue
        coefficient = [0.0] * 28
        start = offsets[row["owner"]]
        coefficient[start:start + 2] = row["control_coefficient"]
        rebuilt.append((row["id"], coefficient, row["constant"], False))
    for base, halves in pairs.items():
        if len(halves) != 2:
            raise ValueError(f"pair does not have two halves: {base}")
        first, second = halves
        if (first["owner"] != second["peer"] or
                second["owner"] != first["peer"] or
                abs(first["responsibility"] - 0.5) > 1e-12 or
                abs(second["responsibility"] - 0.5) > 1e-12 or
                max(abs(a + b) for a, b in zip(
                    first["control_coefficient"],
                    second["control_coefficient"])) > 1e-9 or
                abs(first["constant"] - second["constant"]) > 1e-8 or
                abs(first["coefficient_uncertainty_reserve"] -
                    second["coefficient_uncertainty_reserve"]) > 1e-10):
            raise ValueError(f"incoherent pair halves: {base}")
        coefficient = [0.0] * 28
        owner_start = offsets[first["owner"]]
        peer_start = offsets[first["peer"]]
        coefficient[owner_start:owner_start + 2] = \
            first["control_coefficient"]
        coefficient[peer_start:peer_start + 2] = \
            [-value for value in first["control_coefficient"]]
        constant = (2.0 * first["constant"] +
                    first["coefficient_uncertainty_reserve"])
        rebuilt.append((base + ":full-pair-once-reserve",
                        coefficient, constant, True))
    return sorted(rebuilt), len(pairs)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("gate_a", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--expected-snapshot-sha256", required=True)
    arguments = parser.parse_args()
    actual_sha = sha256(arguments.snapshot)
    snapshot = json.loads(arguments.snapshot.read_text())
    gate_a = json.loads(arguments.gate_a.read_text())
    rows, pair_count = rebuild_rows(snapshot)
    controls_by_owner = gate_a["full_pair"]["controls"]
    if not gate_a["full_pair"]["feasible"] or controls_by_owner is None:
        raise ValueError("current full-pair solution is not feasible")
    controls = []
    for owner in range(1, 15):
        controls.extend(controls_by_owner[str(owner)])
    residuals = []
    for row_id, coefficient, constant, _ in rows:
        residual = sum(a * b for a, b in zip(coefficient, controls)) + constant
        if not math.isfinite(residual):
            raise ValueError(f"nonfinite residual: {row_id}")
        residuals.append((residual, row_id))
    minimum, limiting = min(residuals)
    tolerance = 1e-8
    output = {
        "protocol": "task10p11s-independent-full28d-residual-v1",
        "snapshot_sha256": actual_sha,
        "hash_matches": actual_sha == arguments.expected_snapshot_sha256,
        "row_count": len(rows),
        "coupled_mobile_pair_count": pair_count,
        "minimum_residual_mps2": minimum,
        "limiting_row_id": limiting,
        "all_full_row_residuals_nonnegative_within_tolerance":
            minimum >= -tolerance,
        "tolerance_mps2": tolerance,
        "compiled_minimum_residual_mps2":
            gate_a["full_pair"]["minimum_compiled_residual_mps2"],
        "compiled_independent_minimum_match": abs(
            minimum - gate_a["full_pair"][
                "minimum_compiled_residual_mps2"]) <= 1e-10,
        "gate_b_run_performed": False,
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(output, indent=2) + "\n")
    print(json.dumps(output, indent=2))
    return 0 if (output["hash_matches"] and
                 output["all_full_row_residuals_nonnegative_within_tolerance"] and
                 output["compiled_independent_minimum_match"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
