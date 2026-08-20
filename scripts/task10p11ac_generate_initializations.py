#!/usr/bin/env python3
"""Generate the four preregistered Task 10.11ac initializations."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


DOMAIN = (
    "task10p11ac-v1|task10p11pStandardLaunchPositions|"
    "task10p11rFixedReferenceTopology|2026-08-21"
)
BASE_POSITIONS = [
    [1380.0, 10.0], [1370.0, 30.0], [1360.0, 50.0],
    [1350.0, 70.0], [1340.0, 90.0], [1330.0, 110.0],
    [1320.0, 130.0], [1620.0, 10.0], [1630.0, 30.0],
    [1640.0, 50.0], [1650.0, 70.0], [1660.0, 90.0],
    [1670.0, 110.0], [1680.0, 130.0],
]


def canonical_bytes(value: object) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")


def mapped_offset(initialization: str, owner: int, field: str, bound: float):
    label = f"{DOMAIN}|initialization={initialization}|owner={owner}|field={field}"
    digest = hashlib.sha256(label.encode("utf-8")).digest()
    integer = int.from_bytes(digest[:8], "big") >> 11
    unit = integer / ((1 << 53) - 1)
    return -bound + 2.0 * bound * unit, hashlib.sha256(
        label.encode("utf-8")
    ).hexdigest()


def initialization(identifier: str) -> dict:
    states = []
    field_digests = {}
    for owner, base in enumerate(BASE_POSITIONS, start=1):
        offsets = {}
        digests = {}
        for field, bound in (("px", 2.0), ("py", 2.0), ("vx", 0.2), ("vy", 0.2)):
            if identifier == "I0":
                offsets[field] = 0.0
                digests[field] = None
            else:
                offsets[field], digests[field] = mapped_offset(
                    identifier, owner, field, bound
                )
        field_digests[str(owner)] = digests
        states.append(
            {
                "owner": owner,
                "position_offset_m": [offsets["px"], offsets["py"]],
                "velocity_offset_mps": [offsets["vx"], offsets["vy"]],
                "position_m": [base[0] + offsets["px"], base[1] + offsets["py"]],
                "velocity_mps": [offsets["vx"], offsets["vy"]],
            }
        )
    record = {
        "id": identifier,
        "perturbed": identifier != "I0",
        "mobile_states": states,
        "field_sha256": field_digests,
    }
    record["record_sha256"] = hashlib.sha256(canonical_bytes(record)).hexdigest()
    return record


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    initializations = {
        identifier: initialization(identifier)
        for identifier in ("I0", "P1", "P2", "P3")
    }
    manifest = {
        "protocol": "task10p11ac-initialization-manifest-v1",
        "generation_domain": DOMAIN,
        "mapping": {
            "digest": "SHA-256",
            "integer": "big-endian first 64 bits shifted right by 11",
            "unit_interval": "integer / (2^53 - 1)",
            "position_each_axis_m": [-2.0, 2.0],
            "velocity_each_axis_mps": [-0.2, 0.2],
        },
        "replacement_permitted": False,
        "base_positions_m": BASE_POSITIONS,
        "base_velocities_mps": [[0.0, 0.0] for _ in BASE_POSITIONS],
        "initializations": initializations,
        "matrix_order": [
            "I0-tau20", "I0-tau22", "P1-tau20", "P1-tau22",
            "P2-tau20", "P2-tau22", "P3-tau20", "P3-tau22",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
