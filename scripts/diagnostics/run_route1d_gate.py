"""Run one route1d (target-architecture) diagnostic gate on a v6 family seed.

Materializes the qualified v3 primary overlay (dynamic FIM + analytic-topological
allocated-pairwise + hard ±25) with the frozen v6 initial family, then invokes
the Swarm binary exactly once (no retry).  The caller passes a mission id and
horizon; a preregistration record is written next to the materialized config.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
import uuid
from pathlib import Path

from scripts.diagnostics.run_qualified_closure_campaign import (
    V6_INITIAL_FAMILY_PATH,
    _derive_runtime_initial_state,
    materialize_primary_config,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
BASE_CONFIG = PROJECT_ROOT / "config" / "config.json"
PRIMARY_OVERLAY = (
    PROJECT_ROOT / "config" / "diagnostics" / "qualified_mode_hybrid_dcbf_development_v3.json"
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--mission-id", required=True)
    parser.add_argument("--horizon", type=float, required=True)
    parser.add_argument("--trajectory-seed", type=int, default=2026080201)
    parser.add_argument("--range-noise-seed", type=int, default=2026081301)
    parser.add_argument("--planar-component-max", type=float, default=None)
    arguments = parser.parse_args()

    run_id = f"{time.strftime('%Y%m%dT%H%M%S')}_{uuid.uuid4().hex[:12]}"
    run_root = arguments.output_root / run_id
    run_root.mkdir(parents=True, exist_ok=False)

    mission = {
        "campaign_id": "development-v6",
        "mission_id": arguments.mission_id,
        "trajectory_seed": arguments.trajectory_seed,
        "range_noise_seed": arguments.range_noise_seed,
        "horizon_s": arguments.horizon,
    }
    _, family_audit, _ = _derive_runtime_initial_state(V6_INITIAL_FAMILY_PATH)
    by_seed = {item.seed: item for item in family_audit.registered.audits}
    selected = by_seed.get(arguments.trajectory_seed)
    if selected is None or not selected.accepted:
        raise ValueError("trajectory seed is not admitted by the frozen family")
    mission["initial_positions_sha256"] = selected.positions_sha256
    config_path = run_root / "config.materialized.json"
    config = materialize_primary_config(
        BASE_CONFIG,
        PRIMARY_OVERLAY,
        config_path,
        mission,
        initial_family_path=V6_INITIAL_FAMILY_PATH,
    )
    if arguments.planar_component_max is not None:
        if arguments.planar_component_max <= 0.0:
            raise ValueError("planar component max must be positive")
        config["cbfs"]["input-limits"]["planar-component-max"] = (
            arguments.planar_component_max
        )
        config_path.write_text(
            json.dumps(config, indent=2, ensure_ascii=False) + "\n"
        )
    family = json.loads(V6_INITIAL_FAMILY_PATH.read_text())

    preregistration = {
        "mission_id": arguments.mission_id,
        "trajectory_seed": arguments.trajectory_seed,
        "range_noise_seed": arguments.range_noise_seed,
        "horizon_s": arguments.horizon,
        "family_semantic_sha256": family["semantic_sha256"],
        "family_schema_version": family["schema_version"],
        "family_namespace": family["namespace"],
        "config_sha256": sha256_file(config_path),
        "binary_path": str(arguments.binary),
        "binary_sha256": sha256_file(arguments.binary),
        "no_retry": True,
    }
    (run_root / "preregistration.json").write_text(
        json.dumps(preregistration, indent=2, ensure_ascii=False) + "\n"
    )

    stdout_path = run_root / "stdout.log"
    stderr_path = run_root / "stderr.log"
    started = time.perf_counter()
    with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
        completed = subprocess.run(
            [str(arguments.binary), str(config_path)],
            stdout=stdout,
            stderr=stderr,
        )
    elapsed = time.perf_counter() - started

    record = {
        **preregistration,
        "run_root": str(run_root),
        "returncode": completed.returncode,
        "elapsed_seconds": elapsed,
    }
    (run_root / "gate-record.json").write_text(
        json.dumps(record, indent=2, ensure_ascii=False) + "\n"
    )
    print(json.dumps(record, indent=2, ensure_ascii=False))
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
