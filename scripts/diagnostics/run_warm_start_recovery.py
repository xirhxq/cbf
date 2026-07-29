"""Run one supervised strict-versus-restart localization replay."""

import argparse
import copy
import gzip
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import scripts.diagnostics.replay_localization_calibration as replay_module
from scripts.diagnostics.replay_localization_calibration import (
    ESTIMATOR_CONTRACT_ID,
    RESTART_BEFORE_FIRST_FINITE_POLICY,
    STRICT_PREVIOUS_POLICY,
    replay_calibration,
)
from scripts.diagnostics.run_diagnostic import (
    HARD_FLOOR_BYTES,
    OUTPUT_ROOT_CAP_BYTES,
    RUN_CAP_BYTES,
    SOURCE_SNAPSHOT_NAME,
    SOURCE_SNAPSHOT_POLICY,
    START_BYTES,
    DiskSpaceError,
    _allocate_run_root,
    _create_source_snapshot,
    _git_output,
    _nearest_existing_ancestor,
    _sha256,
    _validate_output_root,
    allocated_bytes,
    available_bytes,
)


PARENT_SCHEMA_ID = "cbf2026-warm-start-recovery-parent-v1"
POLICIES = (
    STRICT_PREVIOUS_POLICY,
    RESTART_BEFORE_FIRST_FINITE_POLICY,
)
BASELINE_FILES = (
    "manifest.json",
    "summary.json",
    "summary.md",
    "calibration.jsonl.gz",
)


def _strict_json(path: Path) -> dict:
    def reject_constant(value: str):
        raise ValueError(f"non-finite JSON constant {value}")

    with path.open() as source:
        value = json.load(source, parse_constant=reject_constant)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _decompressed_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with gzip.open(path, "rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _expected_hash(manifest: dict, field: str) -> str:
    value = manifest.get(field)
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise ValueError(f"immutable baseline manifest has invalid {field}")
    return value


def _verify_immutable_baseline(baseline_dir: Path) -> dict[str, str]:
    baseline_dir = Path(baseline_dir)
    if baseline_dir.is_symlink() or not baseline_dir.is_dir():
        raise ValueError("immutable baseline must be a real directory")
    paths = {name: baseline_dir / name for name in BASELINE_FILES}
    if any(path.is_symlink() or not path.is_file() for path in paths.values()):
        raise ValueError("immutable baseline is missing a required regular file")

    first_hashes = {name: _sha256(path) for name, path in paths.items()}
    manifest = _strict_json(paths["manifest.json"])
    if manifest.get("termination_reason") != "completed":
        raise ValueError("immutable baseline manifest is not completed")
    expected = {
        "summary.json": _expected_hash(manifest, "summary_json_sha256"),
        "summary.md": _expected_hash(manifest, "summary_markdown_sha256"),
        "calibration.jsonl.gz": _expected_hash(
            manifest, "compressed_process_sha256"
        ),
    }
    for name, expected_hash in expected.items():
        if first_hashes[name] != expected_hash:
            raise ValueError(f"immutable baseline hash mismatch for {name}")
    if _decompressed_sha256(paths["calibration.jsonl.gz"]) != _expected_hash(
        manifest, "decompressed_process_sha256"
    ):
        raise ValueError(
            "immutable baseline hash mismatch for decompressed process stream"
        )

    second_hashes = {name: _sha256(path) for name, path in paths.items()}
    if second_hashes != first_hashes:
        raise ValueError("immutable baseline changed while hashes were verified")
    return first_hashes


def _validated_git_state(project_root: Path) -> tuple[str, str, str]:
    source_commit = _git_output(project_root, "rev-parse", "HEAD")
    source_branch = _git_output(project_root, "branch", "--show-current")
    tracked_status = _git_output(
        project_root, "status", "--short", "--untracked-files=no"
    )
    if source_commit in {"", "unknown"} or source_branch in {"", "unknown"}:
        raise ValueError("source branch and commit must be available")
    if tracked_status == "unknown":
        raise ValueError("tracked working-tree status could not be verified")
    if tracked_status:
        raise ValueError("tracked working tree must be clean")
    return source_commit, source_branch, tracked_status


def _paths_overlap(first: Path, second: Path) -> bool:
    first_resolved = first.resolve()
    second_resolved = second.resolve()
    return (
        first_resolved == second_resolved
        or first_resolved in second_resolved.parents
        or second_resolved in first_resolved.parents
    )


def _validate_output_separation(
    output_root: Path,
    *,
    project_root: Path,
    immutable_baseline_dir: Path,
    data_path: Path,
    input_manifest_path: Path,
) -> None:
    protected = {
        "project root": project_root,
        "immutable baseline": immutable_baseline_dir,
        "input data": data_path,
        "input data bundle": data_path.parent,
        "input manifest": input_manifest_path,
        "input manifest bundle": input_manifest_path.parent,
    }
    for label, path in protected.items():
        if _paths_overlap(output_root, path):
            raise ValueError(f"output_root overlap with protected {label}")


def _validate_run_arguments(seeds: list[int], max_frames: int) -> list[int]:
    if not isinstance(max_frames, int) or max_frames <= 0:
        raise ValueError("max_frames must be a positive integer")
    normalized = [int(seed) for seed in seeds]
    if not normalized:
        raise ValueError("at least one seed is required")
    if len(normalized) != len(set(normalized)):
        raise ValueError("duplicate seeds are not allowed")
    return normalized


def _allocation(path: Path) -> int:
    return allocated_bytes(path) if path.exists() else 0


def _limit_reason(
    output_root: Path,
    parent_root: Path,
    free_bytes: int,
) -> str | None:
    if free_bytes < HARD_FLOOR_BYTES:
        return "disk_hard_floor"
    if _allocation(output_root) > OUTPUT_ROOT_CAP_BYTES:
        return "cache_root_cap"
    if _allocation(parent_root) > RUN_CAP_BYTES:
        return "cache_run_cap"
    return None


def _manifest_bytes(manifest: dict) -> bytes:
    return (
        json.dumps(
            manifest,
            allow_nan=False,
            indent=2,
            sort_keys=True,
        )
        + "\n"
    ).encode("utf-8")


def _stage_manifest(path: Path, manifest: dict) -> None:
    path.write_bytes(_manifest_bytes(manifest))


def run_warm_start_recovery(
    data_path: Path,
    input_manifest_path: Path,
    immutable_baseline_dir: Path,
    output_root: Path,
    seeds: list[int],
    project_root: Path,
    max_frames: int,
) -> dict:
    """Create one parent with strict and restart child bundles."""
    data_path = Path(data_path)
    input_manifest_path = Path(input_manifest_path)
    immutable_baseline_dir = Path(immutable_baseline_dir)
    output_root = Path(output_root)
    project_root = Path(project_root)

    _validate_output_separation(
        output_root,
        project_root=project_root,
        immutable_baseline_dir=immutable_baseline_dir,
        data_path=data_path,
        input_manifest_path=input_manifest_path,
    )
    baseline_hashes = _verify_immutable_baseline(immutable_baseline_dir)
    normalized_seeds = _validate_run_arguments(seeds, max_frames)
    _validate_output_root(project_root, output_root)
    free_before = available_bytes(_nearest_existing_ancestor(output_root))
    if free_before < START_BYTES:
        raise DiskSpaceError(
            f"available={free_before} below start threshold={START_BYTES}"
        )
    source_commit, source_branch, tracked_status = _validated_git_state(project_root)
    if not data_path.is_file() or not input_manifest_path.is_file():
        raise ValueError("data and input manifest must be regular files")
    input_manifest = _strict_json(input_manifest_path)
    input_hashes = {
        "data": _sha256(data_path),
        "input_manifest": _sha256(input_manifest_path),
    }
    implementation_path = Path(replay_module.__file__).resolve()
    implementation_sha256 = _sha256(implementation_path)

    parent_root = _allocate_run_root(output_root / "warm-start-recovery")
    started_at = datetime.now(timezone.utc).isoformat()
    snapshot_path = parent_root / SOURCE_SNAPSHOT_NAME
    free_probes = [free_before]
    children: dict[str, dict] = {}
    termination_reason = "completed"
    error: Exception | None = None
    source_snapshot_sha256: str | None = None
    child_started = False

    def probe_limits() -> str | None:
        free = available_bytes(output_root)
        free_probes.append(free)
        return _limit_reason(output_root, parent_root, free)

    def inputs_unchanged() -> None:
        if _sha256(data_path) != input_hashes["data"]:
            raise ValueError("input data changed during paired replay")
        if _sha256(input_manifest_path) != input_hashes["input_manifest"]:
            raise ValueError("input manifest changed during paired replay")
        if _sha256(implementation_path) != implementation_sha256:
            raise ValueError("replay executable changed during paired replay")

    try:
        reason = probe_limits()
        if reason is not None:
            termination_reason = reason
        else:
            _create_source_snapshot(project_root, snapshot_path, output_root)
            source_snapshot_sha256 = _sha256(snapshot_path)
            reason = probe_limits()
            if reason is not None:
                termination_reason = reason

        if termination_reason == "completed":
            for label, policy in zip(("strict", "restart"), POLICIES):
                inputs_unchanged()
                reason = probe_limits()
                if reason is not None:
                    termination_reason = reason
                    break
                child_output_root = parent_root / label
                child_started = True
                child = replay_calibration(
                    data_path,
                    input_manifest_path,
                    child_output_root,
                    normalized_seeds,
                    project_root,
                    max_frames,
                    initialization_policy=policy,
                    supervisor_probe=probe_limits,
                )
                children[label] = child
                inputs_unchanged()
                reason = probe_limits()
                if reason is not None:
                    termination_reason = reason
                    break

        if termination_reason == "completed":
            final_baseline_hashes = _verify_immutable_baseline(
                immutable_baseline_dir
            )
            if final_baseline_hashes != baseline_hashes:
                raise ValueError("immutable baseline changed during paired replay")
            final_commit, final_branch, final_status = _validated_git_state(
                project_root
            )
            if (final_commit, final_branch, final_status) != (
                source_commit,
                source_branch,
                tracked_status,
            ):
                raise ValueError("source Git state changed during paired replay")
            if any(
                children.get(label, {}).get("termination_reason") != "completed"
                for label in ("strict", "restart")
            ):
                termination_reason = "child_failure"
    except Exception as caught:
        error = caught
        termination_reason = (
            "child_failure" if child_started else "runner_setup_error"
        )

    try:
        before_manifest_reason = probe_limits()
    except Exception as caught:
        if error is None:
            error = caught
        if termination_reason == "completed":
            termination_reason = "runner_setup_error"
    else:
        if termination_reason == "completed" and before_manifest_reason is not None:
            termination_reason = before_manifest_reason

    manifest = {
        "schema": PARENT_SCHEMA_ID,
        "termination_reason": termination_reason,
        "estimator_contract": ESTIMATOR_CONTRACT_ID,
        "output_dir": str(parent_root),
        "started_at": started_at,
        "ended_at": datetime.now(timezone.utc).isoformat(),
        "source_commit": source_commit,
        "source_branch": source_branch,
        "tracked_worktree_status": tracked_status,
        "source_snapshot_path": (
            str(snapshot_path) if source_snapshot_sha256 is not None else None
        ),
        "source_snapshot_sha256": source_snapshot_sha256,
        "source_snapshot_policy": SOURCE_SNAPSHOT_POLICY,
        "replay_implementation": {
            "path": str(implementation_path),
            "sha256": implementation_sha256,
        },
        "immutable_baseline_dir": str(immutable_baseline_dir.resolve()),
        "immutable_baseline_hashes": baseline_hashes,
        "input_data": {
            "path": str(data_path.resolve()),
            "sha256": input_hashes["data"],
        },
        "input_manifest": {
            "path": str(input_manifest_path.resolve()),
            "sha256": input_hashes["input_manifest"],
            "source_commit": input_manifest.get("base_commit"),
            "materialized_config_sha256": input_manifest.get("config_sha256"),
        },
        "seeds": normalized_seeds,
        "max_frames": max_frames,
        "policies": list(POLICIES),
        "children": children,
        "disk_limits": {
            "start_bytes": START_BYTES,
            "hard_floor_bytes": HARD_FLOOR_BYTES,
            "output_root_cap_bytes": OUTPUT_ROOT_CAP_BYTES,
            "parent_cap_bytes": RUN_CAP_BYTES,
        },
        "free_bytes_before": free_before,
        "minimum_live_free_bytes": min(free_probes),
        "free_bytes_after": free_probes[-1],
        "output_root_allocated_bytes": _allocation(output_root),
        "parent_allocated_bytes": _allocation(parent_root),
    }
    if error is not None:
        manifest["error"] = {
            "type": type(error).__name__,
            "message": str(error),
        }

    manifest_path = parent_root / "manifest.json"
    staging_path = parent_root / ".manifest.json.finalizing.tmp"
    finalizing = copy.deepcopy(manifest)
    finalizing["termination_reason"] = "finalizing"
    _stage_manifest(staging_path, finalizing)
    completed_ready_for_publication = False

    if manifest["termination_reason"] == "completed":
        try:
            staged_reason = probe_limits()
        except Exception as caught:
            error = caught
            manifest["termination_reason"] = "runner_setup_error"
            manifest["error"] = {
                "type": type(caught).__name__,
                "message": str(caught),
            }
        else:
            if staged_reason is not None:
                manifest["termination_reason"] = staged_reason
            manifest["ended_at"] = datetime.now(timezone.utc).isoformat()
            manifest["minimum_live_free_bytes"] = min(free_probes)
            manifest["free_bytes_after"] = free_probes[-1]
            manifest["output_root_allocated_bytes"] = _allocation(output_root)
            manifest["parent_allocated_bytes"] = _allocation(parent_root)
            if manifest["termination_reason"] == "completed":
                _stage_manifest(staging_path, manifest)
                try:
                    completed_staging_reason = probe_limits()
                except Exception as caught:
                    error = caught
                    manifest["termination_reason"] = "runner_setup_error"
                    manifest["error"] = {
                        "type": type(caught).__name__,
                        "message": str(caught),
                    }
                else:
                    if completed_staging_reason is not None:
                        manifest["termination_reason"] = (
                            completed_staging_reason
                        )
                    else:
                        completed_ready_for_publication = True

    if completed_ready_for_publication:
        staging_path.replace(manifest_path)
        return manifest

    manifest["ended_at"] = datetime.now(timezone.utc).isoformat()
    manifest["minimum_live_free_bytes"] = min(free_probes)
    manifest["free_bytes_after"] = free_probes[-1]
    manifest["output_root_allocated_bytes"] = _allocation(output_root)
    manifest["parent_allocated_bytes"] = _allocation(parent_root)
    _stage_manifest(staging_path, manifest)
    staging_path.replace(manifest_path)
    return manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", required=True, type=Path)
    parser.add_argument("--input-manifest", required=True, type=Path)
    parser.add_argument("--immutable-baseline", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--seed", action="append", type=int, default=[])
    parser.add_argument("--max-frames", required=True, type=int)
    arguments = parser.parse_args(argv)
    project_root = Path(__file__).resolve().parents[2]
    try:
        manifest = run_warm_start_recovery(
            arguments.data,
            arguments.input_manifest,
            arguments.immutable_baseline,
            arguments.output_root,
            arguments.seed,
            project_root,
            arguments.max_frames,
        )
    except (DiskSpaceError, ValueError, OSError) as caught:
        parser.error(str(caught))
    print(json.dumps(manifest, indent=2, allow_nan=False))
    return 0 if manifest["termination_reason"] == "completed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
