"""Extract the single approved two-range reacquisition mechanism fixture."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import os
import stat
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.diagnostics.replay_predictive_wnls_recovery import (
    _file_identity,
    _lstat_components,
    _read_trusted_bytes,
)
from scripts.diagnostics.two_range_reacquisition import (
    propagate_private_state,
    reset_private_state,
)


V4_ROOT = Path("/private/tmp/cbf2026-predictive-wnls-development/stage1-v4")
V4_MANIFEST_SHA256 = (
    "123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223"
)
V4_COMPRESSED_SHA256 = (
    "9710d1a5792e682bb272d0bb3305162180d1e0b9a0636647d6e071d1782a699b"
)
V4_DECOMPRESSED_SHA256 = (
    "7047989804fd03bd4cadffac8212349b7183857623ac201d14ef2fc3e30ee2e1"
)
MECHANISM_KEY = (20260727, 180, 12)
FIXTURE_FIELDS = (
    "schema_id",
    "fixture_id",
    "source_identities",
    "key",
    "mandatory_references",
    "optional_references",
    "current_reference_outputs",
    "measurements",
    "preceding_public_output",
    "preceding_private_state",
    "held_command",
    "expected_mechanism",
)
FIXTURE_SCHEMA_ID = "cbf2026-two-range-reacquisition-fixture-v1"
FIXTURE_ID = "mechanism_20260727_180_12"
V4_PROCESS_NAME = "predictive-wnls-development.jsonl.gz"


def read_v4_manifest(v4_root: Path) -> tuple[dict, dict]:
    payload, identity = _read_trusted_bytes(
        Path(v4_root) / "manifest.json",
        expected_sha256=V4_MANIFEST_SHA256,
    )
    if payload is None:
        raise RuntimeError("v4 manifest payload was not captured")
    manifest = json.loads(payload)
    if not isinstance(manifest, dict):
        raise ValueError("v4 manifest must be a JSON object")
    return manifest, identity


def _public_output(row: dict) -> dict:
    return {
        "output_status": row["output_status"],
        "prediction_age": row["prediction_age"],
        "estimate": row["estimate"],
        "fresh_modeled_covariance": row["fresh_modeled_covariance"],
        "fresh_epsilon": row["fresh_epsilon"],
        "aged_modeled_covariance": row["aged_modeled_covariance"],
        "aged_modeled_radius": row["aged_modeled_radius"],
        "base_anchor_provenance": row["base_anchor_provenance"],
    }


def _stream_approved_rows(
    process_path: Path,
) -> tuple[dict, dict[int, dict], dict, dict, int, str]:
    _lstat_components(process_path, leaf_required=True)
    flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptor = os.open(process_path, flags)
    digest = hashlib.sha256()
    decompressed_size = 0
    mechanism = None
    references: dict[int, dict] = {}
    last_fresh = None
    robot_rows: dict[int, dict] = {}
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode):
            raise ValueError("v4 process must be a regular file")
        with os.fdopen(descriptor, "rb", closefd=False) as raw:
            with gzip.GzipFile(fileobj=raw, mode="rb") as compressed:
                for line in compressed:
                    digest.update(line)
                    decompressed_size += len(line)
                    row = json.loads(line)
                    if (
                        row.get("variant") != "predictive_multistart"
                        or row.get("seed") != MECHANISM_KEY[0]
                    ):
                        continue
                    frame = row.get("frame_index")
                    robot = row.get("robot_id")
                    if robot == MECHANISM_KEY[2] and frame <= MECHANISM_KEY[1]:
                        if row.get("output_status") == "fresh":
                            last_fresh = row
                            robot_rows = {frame: row}
                        elif last_fresh is not None:
                            robot_rows[frame] = row
                        if frame == MECHANISM_KEY[1]:
                            mechanism = row
                    if frame == MECHANISM_KEY[1] and robot in (10, 11):
                        references[robot] = row
    finally:
        os.close(descriptor)
    if digest.hexdigest() != V4_DECOMPRESSED_SHA256:
        raise ValueError("v4 decompressed process hash mismatch")
    if mechanism is None or set(references) != {10, 11} or last_fresh is None:
        raise ValueError("approved mechanism rows are incomplete")
    return (
        mechanism,
        references,
        last_fresh,
        robot_rows,
        decompressed_size,
        digest.hexdigest(),
    )


def _propagated_private_state(
    last_fresh: dict,
    robot_rows: dict[int, dict],
) -> dict:
    source_frame = int(last_fresh["frame_index"])
    state = reset_private_state(
        {
            "estimate": last_fresh["estimate"],
            "modeled_covariance": last_fresh["fresh_modeled_covariance"],
        },
        frame_index=source_frame,
    )
    for frame in range(source_frame + 1, MECHANISM_KEY[1] + 1):
        row = robot_rows.get(frame)
        if row is None or row["applied_command_source_frame"] != frame - 1:
            raise ValueError("recorded held-command sequence is incomplete")
        state = propagate_private_state(
            state,
            row["applied_command"],
            next_frame_index=frame,
        )
        if state is None:
            raise ValueError("recorded private-state propagation failed")
    incoming = robot_rows[MECHANISM_KEY[1]]["private_reacquisition_seed"]
    if not (
        state["estimate"] == incoming["estimate"]
        and state["modeled_covariance"] == incoming["modeled_covariance"]
    ):
        raise ValueError("replayed private state differs from v4 incoming seed")
    return state


def _strict_fixture_bytes(value: dict) -> bytes:
    if tuple(value) != FIXTURE_FIELDS:
        raise ValueError("fixture differs from declared field order")
    return json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=False,
    ).encode("utf-8") + b"\n"


def extract_mechanism_fixture(*, v4_root: Path, output: Path) -> Path:
    v4_root = Path(v4_root)
    if not v4_root.is_absolute() or ".." in v4_root.parts:
        raise ValueError("v4 root must be normalized and absolute")
    manifest, manifest_identity = read_v4_manifest(v4_root)
    if (
        manifest.get("status") != "completed"
        or manifest.get("schema_id")
        != "cbf2026-predictive-wnls-development-rows-v3"
        or manifest.get("selected_invocation") != "registered_replay"
        or manifest.get("compressed_process_sha256") != V4_COMPRESSED_SHA256
        or manifest.get("decompressed_process_sha256")
        != V4_DECOMPRESSED_SHA256
    ):
        raise ValueError("v4 manifest differs from approved source contract")
    process_path = v4_root / V4_PROCESS_NAME
    _, process_identity = _read_trusted_bytes(
        process_path,
        expected_sha256=V4_COMPRESSED_SHA256,
        capture_payload=False,
    )
    (
        mechanism,
        reference_rows,
        last_fresh,
        robot_rows,
        decompressed_size,
        decompressed_sha,
    ) = _stream_approved_rows(process_path)
    if (
        mechanism["mandatory_references"]
        != {"base_ids": [], "uav_ids": [10, 11]}
        or mechanism["optional_candidates"] != []
        or mechanism["active_references"] != [["uav", 10], ["uav", 11]]
    ):
        raise ValueError("approved mechanism reference structure changed")
    evidence = mechanism["reference_evidence"]
    if [record[:2] for record in evidence] != [["uav", 10], ["uav", 11]]:
        raise ValueError("approved mechanism reference order changed")
    if any(
        record[2] != "mandatory"
        or record[3] is not True
        or record[6] != "fresh"
        or record[7] is not True
        or record[8] is not True
        for record in evidence
    ):
        raise ValueError("approved mechanism reference evidence changed")
    private_state = _propagated_private_state(last_fresh, robot_rows)
    truth_declaration = manifest["source_identities"]["truth_data"]
    truth_identity = _file_identity(Path(truth_declaration["path"]))
    if truth_identity != truth_declaration:
        raise ValueError("truth-data identity differs from v4 binding")
    current_outputs = [
        {
            "reference_key": ["uav", robot_id],
            "public_output": _public_output(reference_rows[robot_id]),
        }
        for robot_id in (10, 11)
    ]
    old_candidates = mechanism["candidates"]
    if len(old_candidates) != 3 or any(
        not isinstance(candidate, list)
        or len(candidate) < 3
        or candidate[2] != "converged"
        for candidate in old_candidates
    ):
        raise ValueError("approved old candidate mechanism changed")
    fixture = {
        "schema_id": FIXTURE_SCHEMA_ID,
        "fixture_id": FIXTURE_ID,
        "source_identities": {
            "v4_manifest": manifest_identity,
            "v4_compressed_process": process_identity,
            "v4_decompressed_process": {
                "path": f"{process_path}#decompressed",
                "size": decompressed_size,
                "sha256": decompressed_sha,
            },
            "truth_data": truth_identity,
        },
        "key": {
            "variant": "predictive_multistart",
            "seed": MECHANISM_KEY[0],
            "frame_index": MECHANISM_KEY[1],
            "robot_id": MECHANISM_KEY[2],
            "squad_local_index": mechanism["squad_local_index"],
            "truth_position": mechanism["offline_truth_position"],
        },
        "mandatory_references": mechanism["mandatory_references"],
        "optional_references": mechanism["optional_candidates"],
        "current_reference_outputs": current_outputs,
        "measurements": {
            "reference_keys": [record[:2] for record in evidence],
            "ranges": [record[4] for record in evidence],
            "noise_seeds": [record[5] for record in evidence],
            "ranging_sigma": 0.5,
            "base_anchor_provenance": mechanism[
                "attempt_base_anchor_provenance"
            ],
        },
        "preceding_public_output": _public_output(robot_rows[179]),
        "preceding_private_state": private_state,
        "held_command": {
            "source_frame": mechanism["applied_command_source_frame"],
            "command": mechanism["applied_command"],
        },
        "expected_mechanism": {
            "active_reference_count": 2,
            "active_reference_keys": [["uav", 10], ["uav", 11]],
            "old_failure_reason": "reacquisition_requires_three_active_references",
            "old_candidate_count": 3,
            "old_all_candidates_converged": True,
        },
    }
    payload = _strict_fixture_bytes(fixture)
    output = Path(output)
    if not output.is_absolute():
        output = Path.cwd() / output
    if output.parts[:2] == ("/", "var"):
        output = Path("/private").joinpath(*output.parts[1:])
    if ".." in output.parts:
        raise ValueError("output path must be normalized")
    output.parent.mkdir(parents=True, exist_ok=True)
    _lstat_components(output.parent, leaf_required=False)
    descriptor = os.open(
        output,
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0),
        0o600,
    )
    try:
        written = os.write(descriptor, payload)
        if written != len(payload):
            raise OSError("short fixture write")
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return output


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--v4-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args(argv)
    extract_mechanism_fixture(
        v4_root=arguments.v4_root,
        output=arguments.output,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
