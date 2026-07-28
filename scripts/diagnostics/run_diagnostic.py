"""Materialize and supervise reproducible diagnostic simulations."""

import argparse
import copy
import hashlib
import json
import os
import shutil
import stat
import subprocess
import tarfile
import time
import uuid
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path


START_BYTES = 8_000_000_000
HARD_FLOOR_BYTES = 6_000_000_000
OUTPUT_ROOT_CAP_BYTES = 2_000_000_000
RUN_CAP_BYTES = 250_000_000
SOURCE_SNAPSHOT_NAME = "source-snapshot.tar.gz"
SOURCE_SNAPSHOT_FORMAT = "tar.gz"
SOURCE_SNAPSHOT_POLICY = "cbf2026-source-snapshot-v1"
RUNNER_FAILURE_REASONS = {
    "runner_setup_error",
    "simulator_launch_error",
    "runner_monitor_error",
    "simulator_nonzero_exit",
    "simulator_signal",
    "disk_hard_floor",
    "cache_root_cap",
    "cache_run_cap",
}


class DiskSpaceError(RuntimeError):
    """Raised when a diagnostic run cannot safely use the available disk."""


def deep_merge(base: dict, patch_value: dict) -> dict:
    """Recursively merge a configuration patch without mutating either input."""
    result = copy.deepcopy(base)
    for key, value in patch_value.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def available_bytes(path: Path) -> int:
    """Return free bytes on the filesystem containing *path*."""
    return shutil.disk_usage(path).free


def allocated_bytes(path: Path) -> int:
    """Return recursively allocated bytes without following symbolic links."""
    path = Path(path)

    def allocated_at(current: Path) -> int:
        metadata = current.lstat()
        total = metadata.st_blocks * 512
        if not stat.S_ISDIR(metadata.st_mode):
            return total
        with os.scandir(current) as entries:
            for entry in entries:
                total += allocated_at(Path(entry.path))
        return total

    return allocated_at(path)


def require_start_space(path: Path, threshold_bytes: int = START_BYTES) -> int:
    """Return free bytes, or stop before a run that lacks its reserved space."""
    free = available_bytes(path)
    if free < threshold_bytes:
        raise DiskSpaceError(f"available={free} below start threshold={threshold_bytes}")
    return free


def _nearest_existing_ancestor(path: Path) -> Path:
    """Return *path* or its closest parent that already exists."""
    candidate = path
    while not candidate.exists():
        parent = candidate.parent
        if parent == candidate:
            raise FileNotFoundError(f"no existing ancestor for {path}")
        candidate = parent
    return candidate


def _allocate_run_root(case_root: Path) -> Path:
    """Atomically allocate a timestamped, UUID-qualified directory."""
    case_root.mkdir(parents=True, exist_ok=True)
    for _ in range(100):
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        run_root = case_root / f"{timestamp}_{uuid.uuid4().hex}"
        try:
            run_root.mkdir()
        except FileExistsError:
            continue
        return run_root
    raise RuntimeError(f"could not allocate a unique run directory below {case_root}")


def _validate_output_root(project_root: Path, output_root: Path) -> None:
    """Reject output paths whose artifacts could enter the source worktree."""
    resolved_project_root = project_root.resolve()
    resolved_output_root = output_root.resolve()
    if (
        resolved_output_root == resolved_project_root
        or resolved_project_root in resolved_output_root.parents
    ):
        raise ValueError("output_root must resolve outside project_root")


def materialize_config(
    base_path: Path,
    patch_path: Path,
    output_path: Path,
    horizon_s: float,
    seed: int,
) -> dict:
    """Create a case-specific simulator configuration and return its contents."""
    base = json.loads(base_path.read_text())
    patch = json.loads(patch_path.read_text())
    case = patch["case"]
    config = deep_merge(base, patch["overrides"])
    config["execute"]["time-total"] = horizon_s
    config["execute"]["random-seed"] = seed
    config["output_path"] = str(output_path.parent)
    config["run_suffix"] = f"_{case}_seed_{seed}_{horizon_s:g}s"

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(config, indent=2) + "\n")
    return config


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _is_within(relative_path: Path, root: Path) -> bool:
    return relative_path == root or root in relative_path.parents


def _is_source_snapshot_excluded(
    relative_path: Path,
    additional_exclusions: tuple[Path, ...],
) -> bool:
    """Apply the versioned source-snapshot exclusion policy."""
    if any(_is_within(relative_path, excluded) for excluded in additional_exclusions):
        return True
    if _is_within(
        relative_path,
        Path("docs") / "diagnostics" / "source-snapshots",
    ):
        return True
    if any(
        part in {".git", ".superpowers", "__pycache__"}
        or part == "build"
        or part.startswith("build-")
        or part.startswith("cmake-build-")
        for part in relative_path.parts
    ):
        return True
    if relative_path.parts:
        top_level_name = relative_path.parts[0]
        if top_level_name == "plot" or top_level_name.startswith("data"):
            return True
    return relative_path.suffix in {".pyc", ".pyo"}


def _source_snapshot_exclusions(
    project_root: Path,
    output_root: Path,
) -> tuple[Path, ...]:
    try:
        return (output_root.resolve().relative_to(project_root.resolve()),)
    except ValueError:
        return ()


def _create_source_snapshot(
    project_root: Path,
    snapshot_path: Path,
    output_root: Path,
) -> None:
    """Archive the source working tree with Python's standard library only."""
    exclusions = _source_snapshot_exclusions(project_root, output_root)
    temporary_path = snapshot_path.with_name(f".{snapshot_path.name}.tmp")
    with tarfile.open(temporary_path, mode="w:gz") as archive:
        for directory, directory_names, filenames in os.walk(
            project_root,
            topdown=True,
            followlinks=False,
        ):
            directory_path = Path(directory)
            relative_directory = directory_path.relative_to(project_root)
            directory_names[:] = sorted(
                name
                for name in directory_names
                if not _is_source_snapshot_excluded(
                    relative_directory / name,
                    exclusions,
                )
            )
            for filename in sorted(filenames):
                source_path = directory_path / filename
                relative_path = source_path.relative_to(project_root)
                if _is_source_snapshot_excluded(relative_path, exclusions):
                    continue
                archive.add(
                    source_path,
                    arcname=relative_path.as_posix(),
                    recursive=False,
                )
    temporary_path.replace(snapshot_path)


def _git_output(project_root: Path, *arguments: str) -> str:
    result = subprocess.run(
        ["git", *arguments],
        cwd=project_root,
        check=False,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def _termination_reason(returncode: int | None) -> str:
    if returncode == 0:
        return "completed"
    if returncode is not None and returncode < 0:
        return "simulator_signal"
    return "simulator_nonzero_exit"


def _safe_available_bytes(path: Path) -> int | None:
    """Return a final disk measurement without hiding the terminal manifest."""
    try:
        return available_bytes(path)
    except Exception:
        return None


def _safe_allocated_bytes(path: Path) -> int | None:
    """Return allocated bytes without hiding a terminal manifest on failure."""
    try:
        return allocated_bytes(path)
    except Exception:
        return None


def _stop_process(process) -> None:
    """Best-effort stop with a five-second terminate-to-kill escalation."""
    try:
        process.terminate()
    except BaseException:
        pass
    try:
        process.wait(timeout=5)
        return
    except subprocess.TimeoutExpired:
        pass
    except BaseException:
        pass
    try:
        process.kill()
    except BaseException:
        pass
    try:
        process.wait()
    except BaseException:
        pass


def _write_manifest(run_root: Path, manifest: dict) -> None:
    """Atomically publish a terminal manifest inside an allocated run."""
    manifest_path = run_root / "manifest.json"
    temporary_path = run_root / ".manifest.json.tmp"
    temporary_path.write_text(json.dumps(manifest, indent=2) + "\n")
    temporary_path.replace(manifest_path)


def run_diagnostic(
    case: str,
    horizon_s: float,
    seed: int,
    binary_path: Path,
    output_root: Path,
    project_root: Path,
) -> dict:
    """Run one configured diagnostic case, stopping it before disk exhaustion."""
    case = case.upper()
    patch_paths = {
        "H0": project_root / "config" / "diagnostics" / "h0_historical.json",
        "C1": project_root / "config" / "diagnostics" / "c1_claim_aligned.json",
        "U0": project_root / "config" / "diagnostics" / "u0_uncertainty_ablation.json",
        "C0": project_root / "config" / "diagnostics" / "c0_corrected.json",
        "R": project_root / "config" / "diagnostics" / "r_rate_aware.json",
        "RB": project_root / "config" / "diagnostics" / "rb_bounded.json",
        "RBP": project_root / "config" / "diagnostics" / "rbp_pairwise.json",
    }
    if case not in patch_paths:
        raise ValueError(f"unknown diagnostic case: {case}")

    _validate_output_root(project_root, output_root)
    free_bytes_before = require_start_space(_nearest_existing_ancestor(output_root))
    base_commit = _git_output(project_root, "rev-parse", "HEAD")
    branch = _git_output(project_root, "rev-parse", "--abbrev-ref", "HEAD")
    working_tree_status = _git_output(
        project_root,
        "status",
        "--porcelain",
        "--untracked-files=all",
    )
    case_root = output_root / case
    run_root = _allocate_run_root(case_root)
    config_path = run_root / "config.materialized.json"
    source_snapshot_path = run_root / SOURCE_SNAPSHOT_NAME
    started_at = datetime.now(timezone.utc).isoformat()
    stdout_path = run_root / "stdout.log"
    stderr_path = run_root / "stderr.log"
    config = None
    config_sha256 = None
    binary_sha256 = None
    source_snapshot_sha256 = None

    def finalize_manifest(
        returncode: int | None,
        termination_reason: str,
        error: BaseException | None = None,
    ) -> dict:
        manifest = {
            "case": case,
            "base_commit": base_commit,
            "working_tree_dirty": (
                None if working_tree_status == "unknown" else bool(working_tree_status)
            ),
            "branch": branch,
            "seed": seed,
            "horizon_s": horizon_s,
            "solver": config.get("optimiser") if config is not None else None,
            "config_path": config_path.name,
            "config_sha256": config_sha256,
            "binary_path": str(binary_path),
            "binary_sha256": binary_sha256,
            "source_snapshot_sha256": source_snapshot_sha256,
            "source_snapshot_path": source_snapshot_path.name,
            "source_snapshot_format": SOURCE_SNAPSHOT_FORMAT,
            "source_snapshot_policy": SOURCE_SNAPSHOT_POLICY,
            "free_bytes_before": free_bytes_before,
            "free_bytes_after": _safe_available_bytes(output_root),
            "output_root_allocated_bytes": _safe_allocated_bytes(output_root),
            "run_allocated_bytes": _safe_allocated_bytes(run_root),
            "output_root_cap_bytes": OUTPUT_ROOT_CAP_BYTES,
            "run_cap_bytes": RUN_CAP_BYTES,
            "started_at": started_at,
            "ended_at": datetime.now(timezone.utc).isoformat(),
            "returncode": returncode,
            "termination_reason": termination_reason,
            "output_dir": str(run_root),
        }
        if error is not None:
            manifest["error"] = {
                "type": type(error).__name__,
                "message": str(error),
            }
        _write_manifest(run_root, manifest)
        return manifest

    setup_resources = ExitStack()
    try:
        stdout_path.touch()
        stderr_path.touch()
        binary_sha256 = _sha256(binary_path)
        config = materialize_config(
            project_root / "config" / "config.json",
            patch_paths[case],
            config_path,
            horizon_s,
            seed,
        )
        config_sha256 = _sha256(config_path)
        _create_source_snapshot(project_root, source_snapshot_path, output_root)
        source_snapshot_sha256 = _sha256(source_snapshot_path)
        output_root_allocation = allocated_bytes(output_root)
        run_allocation = allocated_bytes(run_root)
        if output_root_allocation > OUTPUT_ROOT_CAP_BYTES:
            setup_resources.close()
            return finalize_manifest(
                returncode=None,
                termination_reason="cache_root_cap",
            )
        if run_allocation > RUN_CAP_BYTES:
            setup_resources.close()
            return finalize_manifest(
                returncode=None,
                termination_reason="cache_run_cap",
            )
        stdout_file = setup_resources.enter_context(stdout_path.open("w"))
        stderr_file = setup_resources.enter_context(stderr_path.open("w"))
    except BaseException as error:
        setup_resources.close()
        for log_path in (stdout_path, stderr_path):
            try:
                log_path.touch()
            except BaseException:
                pass
        return finalize_manifest(
            returncode=None,
            termination_reason="runner_setup_error",
            error=error,
        )

    try:
        process = subprocess.Popen(
            [str(binary_path), str(config_path)],
            cwd=project_root,
            stdout=stdout_file,
            stderr=stderr_file,
            text=True,
        )
    except BaseException as error:
        setup_resources.close()
        return finalize_manifest(
            returncode=None,
            termination_reason="simulator_launch_error",
            error=error,
        )

    termination_reason = None
    monitor_error = None
    try:
        while process.poll() is None:
            if available_bytes(output_root) < HARD_FLOOR_BYTES:
                _stop_process(process)
                termination_reason = "disk_hard_floor"
                break
            if allocated_bytes(output_root) > OUTPUT_ROOT_CAP_BYTES:
                _stop_process(process)
                termination_reason = "cache_root_cap"
                break
            if allocated_bytes(run_root) > RUN_CAP_BYTES:
                _stop_process(process)
                termination_reason = "cache_run_cap"
                break
            time.sleep(0.5)
    except BaseException as error:
        monitor_error = error
        _stop_process(process)
    finally:
        setup_resources.close()

    try:
        returncode = process.poll()
    except BaseException:
        returncode = getattr(process, "returncode", None)
    if monitor_error is not None:
        return finalize_manifest(
            returncode,
            "runner_monitor_error",
            error=monitor_error,
        )
    if termination_reason is None:
        termination_reason = _termination_reason(returncode)
    return finalize_manifest(returncode, termination_reason)


def main(argv: list[str] | None = None) -> int:
    """Run the command-line diagnostic interface."""
    project_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--case",
        required=True,
        choices=("H0", "C1", "U0", "C0", "R", "RB", "RBP"),
    )
    parser.add_argument("--horizon", required=True, type=float, metavar="SECONDS")
    parser.add_argument("--seed", required=True, type=int)
    parser.add_argument("--binary", type=Path, default=project_root / "cmake-build-release" / "Swarm")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=project_root.parent / "cbf2026-diagnostic-results",
    )
    arguments = parser.parse_args(argv)

    try:
        manifest = run_diagnostic(
            case=arguments.case,
            horizon_s=arguments.horizon,
            seed=arguments.seed,
            binary_path=arguments.binary,
            output_root=arguments.output_root,
            project_root=project_root,
        )
    except DiskSpaceError as error:
        parser.error(str(error))
    print(json.dumps(manifest, indent=2))
    return 2 if manifest["termination_reason"] in RUNNER_FAILURE_REASONS else 0


if __name__ == "__main__":
    raise SystemExit(main())
