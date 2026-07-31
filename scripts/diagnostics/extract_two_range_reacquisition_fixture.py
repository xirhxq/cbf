"""Extract the single approved two-range reacquisition mechanism fixture."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import importlib.machinery
import importlib.util
import json
import os
import stat
import sys
from pathlib import Path

if __package__ in (None, ""):
    _SCRIPT_REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(_SCRIPT_REPOSITORY_ROOT))
    _SCRIPT_PACKAGE_SPEC = importlib.machinery.PathFinder.find_spec(
        "scripts",
        [str(_SCRIPT_REPOSITORY_ROOT)],
    )
    if (
        _SCRIPT_PACKAGE_SPEC is None
        or _SCRIPT_PACKAGE_SPEC.submodule_search_locations is None
    ):
        raise ImportError("implementation-root scripts package is unavailable")
    sys.modules["scripts"] = importlib.util.module_from_spec(
        _SCRIPT_PACKAGE_SPEC
    )

from scripts.diagnostics.replay_predictive_wnls_recovery import (
    _file_identity,
    _lstat_components,
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


def _stable_file_identity(metadata: os.stat_result) -> tuple[int, ...]:
    return (
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_mode,
        metadata.st_size,
        metadata.st_mtime_ns,
    )


def _open_v4_child(
    root_descriptor: int,
    child_name: str,
    *,
    description: str,
) -> tuple[int, tuple[int, ...]]:
    if (
        not child_name
        or child_name in {".", ".."}
        or Path(child_name).name != child_name
    ):
        raise ValueError(f"{description} child name is invalid")
    descriptor = os.open(
        child_name,
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0),
        dir_fd=root_descriptor,
    )
    try:
        metadata = os.fstat(descriptor)
        linked = os.stat(
            child_name,
            dir_fd=root_descriptor,
            follow_symlinks=False,
        )
        identity = _stable_file_identity(metadata)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or not stat.S_ISREG(linked.st_mode)
            or _stable_file_identity(linked) != identity
        ):
            raise ValueError(f"{description} identity is invalid")
        return descriptor, identity
    except BaseException:
        os.close(descriptor)
        raise


def _assert_v4_child_identity(
    root_descriptor: int,
    child_name: str,
    descriptor: int,
    identity: tuple[int, ...],
    *,
    description: str,
) -> None:
    try:
        metadata = os.fstat(descriptor)
        linked = os.stat(
            child_name,
            dir_fd=root_descriptor,
            follow_symlinks=False,
        )
    except (FileNotFoundError, OSError) as error:
        raise ValueError(f"{description} identity changed") from error
    if (
        not stat.S_ISREG(metadata.st_mode)
        or not stat.S_ISREG(linked.st_mode)
        or _stable_file_identity(metadata) != identity
        or _stable_file_identity(linked) != identity
    ):
        raise ValueError(f"{description} identity changed")


def _source_identity(
    path: Path,
    metadata: os.stat_result,
    sha256: str,
) -> dict:
    return {
        "path": str(path),
        "sha256": sha256,
        "device": metadata.st_dev,
        "inode": metadata.st_ino,
        "size": metadata.st_size,
        "mtime_ns": metadata.st_mtime_ns,
    }


def _read_pinned_gzip_lines(
    path: Path,
    *,
    expected_compressed_sha256: str,
    expected_decompressed_sha256: str,
    identity_sink: dict | None = None,
    descriptor: int | None = None,
    root_descriptor: int | None = None,
    child_name: str | None = None,
    child_identity: tuple[int, ...] | None = None,
):
    """Yield decompressed lines from the descriptor whose bytes were hashed."""
    process_path = Path(path)
    if process_path.parts[:2] == ("/", "var"):
        process_path = Path("/private").joinpath(*process_path.parts[1:])
    owns_descriptor = descriptor is None
    owns_root_descriptor = descriptor is None
    if owns_descriptor:
        if any(
            value is not None
            for value in (root_descriptor, child_name, child_identity)
        ):
            raise ValueError("v4 process descriptor binding is incomplete")
        _lstat_components(process_path.parent, leaf_required=False)
        root_descriptor = os.open(
            process_path.parent,
            os.O_RDONLY
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_DIRECTORY", 0)
            | getattr(os, "O_NOFOLLOW", 0),
        )
        try:
            child_name = process_path.name
            descriptor, child_identity = _open_v4_child(
                root_descriptor,
                child_name,
                description="v4 process",
            )
        except BaseException:
            os.close(root_descriptor)
            raise
    elif (
        root_descriptor is None
        or child_name is None
        or child_identity is None
    ):
        raise ValueError("v4 process descriptor binding is incomplete")
    if descriptor is None:
        raise RuntimeError("v4 process descriptor was not opened")
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise ValueError("v4 process must be a regular file")
        _assert_v4_child_identity(
            root_descriptor,
            child_name,
            descriptor,
            child_identity,
            description="v4 process",
        )
        compressed_digest = hashlib.sha256()
        offset = 0
        while offset < before.st_size:
            chunk = os.pread(
                descriptor,
                min(1024 * 1024, before.st_size - offset),
                offset,
            )
            if not chunk:
                raise ValueError("v4 compressed process short read")
            compressed_digest.update(chunk)
            offset += len(chunk)
        compressed_sha256 = compressed_digest.hexdigest()
        if compressed_sha256 != expected_compressed_sha256:
            raise ValueError("v4 compressed process hash mismatch")
        identity = _source_identity(
            process_path,
            before,
            compressed_sha256,
        )
        if identity_sink is not None:
            identity_sink.clear()
            identity_sink.update(identity)
        os.lseek(descriptor, 0, os.SEEK_SET)
        decompressed_digest = hashlib.sha256()
        with os.fdopen(descriptor, "rb", closefd=False) as raw:
            with gzip.GzipFile(fileobj=raw, mode="rb") as compressed:
                for line in compressed:
                    decompressed_digest.update(line)
                    yield line
        if decompressed_digest.hexdigest() != expected_decompressed_sha256:
            raise ValueError("v4 decompressed process hash mismatch")
        after = os.fstat(descriptor)
        if _stable_file_identity(after) != child_identity:
            raise ValueError("v4 process identity changed during extraction")
        _assert_v4_child_identity(
            root_descriptor,
            child_name,
            descriptor,
            child_identity,
            description="v4 process",
        )
    finally:
        if owns_descriptor:
            os.close(descriptor)
        if owns_root_descriptor:
            os.close(root_descriptor)


def _normalize_v4_root(v4_root: Path) -> Path:
    root = Path(v4_root)
    if root.parts[:2] == ("/", "var"):
        root = Path("/private").joinpath(*root.parts[1:])
    if not root.is_absolute() or ".." in root.parts:
        raise ValueError("v4 root must be normalized and absolute")
    return root


def _open_v4_root(v4_root: Path) -> tuple[int, tuple[int, int]]:
    _lstat_components(v4_root, leaf_required=False)
    descriptor = os.open(
        v4_root,
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_NOFOLLOW", 0),
    )
    try:
        metadata = os.fstat(descriptor)
        linked = v4_root.lstat()
        identity = (metadata.st_dev, metadata.st_ino)
        if (
            not stat.S_ISDIR(metadata.st_mode)
            or not stat.S_ISDIR(linked.st_mode)
            or (linked.st_dev, linked.st_ino) != identity
        ):
            raise ValueError("v4 root identity is invalid")
        return descriptor, identity
    except BaseException:
        os.close(descriptor)
        raise


def _assert_v4_root_identity(
    v4_root: Path,
    descriptor: int,
    identity: tuple[int, int],
) -> None:
    try:
        _lstat_components(v4_root, leaf_required=False)
        metadata = os.fstat(descriptor)
        linked = v4_root.lstat()
    except (FileNotFoundError, OSError) as error:
        raise ValueError("v4 root identity changed") from error
    if (
        not stat.S_ISDIR(metadata.st_mode)
        or not stat.S_ISDIR(linked.st_mode)
        or (metadata.st_dev, metadata.st_ino) != identity
        or (linked.st_dev, linked.st_ino) != identity
    ):
        raise ValueError("v4 root identity changed")


def _read_descriptor_bytes(
    descriptor: int,
    size: int,
    *,
    description: str,
) -> bytes:
    chunks = []
    offset = 0
    while offset < size:
        chunk = os.pread(
            descriptor,
            min(1024 * 1024, size - offset),
            offset,
        )
        if not chunk:
            raise ValueError(f"{description} short read")
        chunks.append(chunk)
        offset += len(chunk)
    return b"".join(chunks)


def _read_bound_v4_manifest(
    v4_root: Path,
    root_descriptor: int,
    root_identity: tuple[int, int],
) -> tuple[dict, dict, int, tuple[int, ...]]:
    _assert_v4_root_identity(v4_root, root_descriptor, root_identity)
    descriptor, child_identity = _open_v4_child(
        root_descriptor,
        "manifest.json",
        description="v4 manifest",
    )
    try:
        metadata = os.fstat(descriptor)
        payload = _read_descriptor_bytes(
            descriptor,
            metadata.st_size,
            description="v4 manifest",
        )
        sha256 = hashlib.sha256(payload).hexdigest()
        if sha256 != V4_MANIFEST_SHA256:
            raise ValueError("v4 manifest hash mismatch")
        _assert_v4_child_identity(
            root_descriptor,
            "manifest.json",
            descriptor,
            child_identity,
            description="v4 manifest",
        )
        _assert_v4_root_identity(v4_root, root_descriptor, root_identity)
        manifest = json.loads(payload)
        if not isinstance(manifest, dict):
            raise ValueError("v4 manifest must be a JSON object")
        if manifest.get("output_root") != str(v4_root):
            raise ValueError(
                "v4 manifest output_root differs from requested root",
            )
        return (
            manifest,
            _source_identity(
                v4_root / "manifest.json",
                metadata,
                sha256,
            ),
            descriptor,
            child_identity,
        )
    except BaseException:
        os.close(descriptor)
        raise


def read_v4_manifest(
    v4_root: Path,
    *,
    root_descriptor: int | None = None,
    root_identity: tuple[int, int] | None = None,
) -> tuple[dict, dict]:
    v4_root = _normalize_v4_root(v4_root)
    owns_descriptor = root_descriptor is None
    if owns_descriptor:
        root_descriptor, root_identity = _open_v4_root(v4_root)
    if root_descriptor is None or root_identity is None:
        raise ValueError("v4 root descriptor binding is incomplete")
    try:
        manifest, identity, descriptor, _ = _read_bound_v4_manifest(
            v4_root,
            root_descriptor,
            root_identity,
        )
        try:
            return manifest, identity
        finally:
            os.close(descriptor)
    finally:
        if owns_descriptor:
            os.close(root_descriptor)


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
    *,
    process_descriptor: int | None = None,
    root_descriptor: int | None = None,
    process_child_identity: tuple[int, ...] | None = None,
) -> tuple[dict, dict[int, dict], dict, dict, int, str, dict]:
    digest = hashlib.sha256()
    decompressed_size = 0
    mechanism = None
    references: dict[int, dict] = {}
    last_fresh = None
    robot_rows: dict[int, dict] = {}
    process_identity: dict = {}
    for line in _read_pinned_gzip_lines(
        process_path,
        expected_compressed_sha256=V4_COMPRESSED_SHA256,
        expected_decompressed_sha256=V4_DECOMPRESSED_SHA256,
        identity_sink=process_identity,
        descriptor=process_descriptor,
        root_descriptor=root_descriptor,
        child_name=(
            V4_PROCESS_NAME if process_descriptor is not None else None
        ),
        child_identity=process_child_identity,
    ):
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
    if mechanism is None or set(references) != {10, 11} or last_fresh is None:
        raise ValueError("approved mechanism rows are incomplete")
    return (
        mechanism,
        references,
        last_fresh,
        robot_rows,
        decompressed_size,
        digest.hexdigest(),
        process_identity,
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


def _extract_mechanism_fixture_from_root(
    *,
    v4_root: Path,
    output: Path,
    root_descriptor: int,
    root_identity: tuple[int, int],
) -> Path:
    (
        manifest,
        manifest_identity,
        manifest_descriptor,
        manifest_child_identity,
    ) = _read_bound_v4_manifest(
        v4_root,
        root_descriptor,
        root_identity,
    )
    process_descriptor = None
    try:
        if (
            manifest.get("status") != "completed"
            or manifest.get("schema_id")
            != "cbf2026-predictive-wnls-development-rows-v3"
            or manifest.get("selected_invocation") != "registered_replay"
            or manifest.get("compressed_process_sha256")
            != V4_COMPRESSED_SHA256
            or manifest.get("decompressed_process_sha256")
            != V4_DECOMPRESSED_SHA256
        ):
            raise ValueError(
                "v4 manifest differs from approved source contract",
            )
        process_descriptor, process_child_identity = _open_v4_child(
            root_descriptor,
            V4_PROCESS_NAME,
            description="v4 process",
        )
        return _extract_mechanism_fixture_from_bound_sources(
            v4_root=v4_root,
            output=output,
            root_descriptor=root_descriptor,
            root_identity=root_identity,
            manifest=manifest,
            manifest_identity=manifest_identity,
            manifest_descriptor=manifest_descriptor,
            manifest_child_identity=manifest_child_identity,
            process_descriptor=process_descriptor,
            process_child_identity=process_child_identity,
        )
    finally:
        if process_descriptor is not None:
            os.close(process_descriptor)
        os.close(manifest_descriptor)


def _extract_mechanism_fixture_from_bound_sources(
    *,
    v4_root: Path,
    output: Path,
    root_descriptor: int,
    root_identity: tuple[int, int],
    manifest: dict,
    manifest_identity: dict,
    manifest_descriptor: int,
    manifest_child_identity: tuple[int, ...],
    process_descriptor: int,
    process_child_identity: tuple[int, ...],
) -> Path:
    process_path = v4_root / V4_PROCESS_NAME
    (
        mechanism,
        reference_rows,
        last_fresh,
        robot_rows,
        decompressed_size,
        decompressed_sha,
        process_identity,
    ) = _stream_approved_rows(
        process_path,
        process_descriptor=process_descriptor,
        root_descriptor=root_descriptor,
        process_child_identity=process_child_identity,
    )
    _assert_v4_root_identity(v4_root, root_descriptor, root_identity)
    _assert_v4_child_identity(
        root_descriptor,
        "manifest.json",
        manifest_descriptor,
        manifest_child_identity,
        description="v4 manifest",
    )
    _assert_v4_child_identity(
        root_descriptor,
        V4_PROCESS_NAME,
        process_descriptor,
        process_child_identity,
        description="v4 process",
    )
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
    _assert_v4_root_identity(v4_root, root_descriptor, root_identity)
    _assert_v4_child_identity(
        root_descriptor,
        "manifest.json",
        manifest_descriptor,
        manifest_child_identity,
        description="v4 manifest",
    )
    _assert_v4_child_identity(
        root_descriptor,
        V4_PROCESS_NAME,
        process_descriptor,
        process_child_identity,
        description="v4 process",
    )
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
    _assert_v4_root_identity(v4_root, root_descriptor, root_identity)
    _assert_v4_child_identity(
        root_descriptor,
        "manifest.json",
        manifest_descriptor,
        manifest_child_identity,
        description="v4 manifest",
    )
    _assert_v4_child_identity(
        root_descriptor,
        V4_PROCESS_NAME,
        process_descriptor,
        process_child_identity,
        description="v4 process",
    )
    return output


def extract_mechanism_fixture(*, v4_root: Path, output: Path) -> Path:
    v4_root = _normalize_v4_root(v4_root)
    root_descriptor, root_identity = _open_v4_root(v4_root)
    try:
        return _extract_mechanism_fixture_from_root(
            v4_root=v4_root,
            output=output,
            root_descriptor=root_descriptor,
            root_identity=root_identity,
        )
    finally:
        os.close(root_descriptor)


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
