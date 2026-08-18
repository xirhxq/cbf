#!/usr/bin/env python3
"""Read-only structural/hash check for the minimal 14-owner snapshot."""

import argparse
import hashlib
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--expected-sha256", required=True)
    args = parser.parse_args()

    raw = args.snapshot.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    data = json.loads(raw)
    ids = data["canonical_request"]["mobile_ids"]
    counts = data["owner_row_counts"]
    controls = data["nominal_controls"]
    rows = data["actual_rows"]
    complete = (
        data.get("protocol") == "task10p11s-minimal-snapshot-v1"
        and ids == list(range(1, 15))
        and len(data["estimator"]["mean"]) == 56
        and len(data["estimator"]["covariance"]) == 56
        and len(controls) == 14
        and len(data["objective_28d"]) == 28
        and all(int(counts[str(owner)]) > 0 for owner in ids)
        and {int(row["owner"]) for row in rows} == set(ids)
        and data["preflight"]["complete"] is True
        and data["preflight"]["rows_match"] is True
        and data["preflight"]["objective_matches"] is True
        and digest == args.expected_sha256
    )
    print(json.dumps({
        "complete": complete,
        "owner_count": len(ids),
        "row_count": len(rows),
        "objective_dimension": len(data["objective_28d"]),
        "sha256": digest,
        "hash_matches": digest == args.expected_sha256,
    }, indent=2, sort_keys=True))
    return 0 if complete else 1


if __name__ == "__main__":
    raise SystemExit(main())
