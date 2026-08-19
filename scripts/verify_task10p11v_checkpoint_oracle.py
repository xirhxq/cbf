#!/usr/bin/env python3
"""Read-only verification for the Task 10.11v short checkpoint gate."""

import hashlib
import json
import pathlib
import sys


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    if len(sys.argv) != 4:
        print("usage: verify_task10p11v_checkpoint_oracle.py CHECKPOINT ORACLE OUTPUT")
        return 2
    checkpoint_path, oracle_path, output_path = map(pathlib.Path, sys.argv[1:])
    checkpoint = json.loads(checkpoint_path.read_text())
    oracle = json.loads(oracle_path.read_text())
    restart = checkpoint.get("restart_checkpoint", {})
    audit = oracle.get("checkpoint_audit", {})
    gate = oracle.get("gate_a", {})
    checks = {
        "fourteen_owners": len(restart.get("plant", {}).get("robots", [])) == 14,
        "rows_rebuilt": gate.get("snapshot_complete") is True,
        "objective_rebuilt": checkpoint.get("preflight", {}).get("objective_matches") is True,
        "capture_fields_complete": audit.get("capture_fields_complete") is True,
        "full_pair_feasible": gate.get("full_pair", {}).get("feasible") is True,
        "successor_performed": gate.get("successor", {}).get("performed") is True,
        "successor_feasible": gate.get("successor", {}).get("feasible") is True,
        "not_a_restart_claim": audit.get("deterministic_restart_complete") is False,
        "no_trajectory": oracle.get("trajectory_run") is False,
    }
    result = {
        "protocol": "task10p11v-read-only-verification-v1",
        "complete": all(checks.values()),
        "checks": checks,
        "checkpoint_sha256": sha256(checkpoint_path),
        "oracle_sha256": sha256(oracle_path),
        "checkpoint_bytes": checkpoint_path.stat().st_size,
        "oracle_bytes": oracle_path.stat().st_size,
    }
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["complete"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
