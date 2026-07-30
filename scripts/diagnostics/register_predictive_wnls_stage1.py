"""Freeze the exact Stage-1 predictive-WNLS development protocol."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import re
import shlex
import stat
import subprocess
from pathlib import Path
from typing import Mapping

from scripts.diagnostics import replay_predictive_wnls_recovery as replay


REPOSITORY_SOURCE_PATHS = {
    "replay_source": Path(
        "scripts/diagnostics/replay_predictive_wnls_recovery.py"
    ),
    "estimator_source": Path("scripts/diagnostics/predictive_wnls.py"),
    "legacy_solver_source": Path(
        "scripts/diagnostics/replay_localization_calibration.py"
    ),
    "diagnostic_integrity_source": Path(
        "scripts/diagnostics/run_diagnostic.py"
    ),
    "analyzer_source": Path(
        "scripts/diagnostics/analyze_predictive_wnls_recovery.py"
    ),
}


def _absolute(path: Path, *, base: Path | None = None) -> Path:
    path = Path(path)
    if ".." in path.parts:
        raise ValueError(f"path must not contain parent traversal: {path}")
    if not path.is_absolute():
        path = (base if base is not None else Path.cwd()) / path
    if not path.is_absolute():
        raise ValueError(f"path must be absolute after normalization: {path}")
    return path


def _lstat_path(path: Path, *, leaf: str) -> os.stat_result | None:
    """Reject symlink components and validate an existing or absent leaf."""
    path = _absolute(path)
    chain = list(reversed(path.parents)) + [path]
    for index, component in enumerate(chain):
        is_leaf = index == len(chain) - 1
        try:
            metadata = component.lstat()
        except FileNotFoundError:
            if is_leaf and leaf == "absent":
                return None
            raise ValueError(f"missing trusted path component: {component}") from None
        if stat.S_ISLNK(metadata.st_mode):
            raise ValueError(
                f"symbolic-link path component is forbidden: {component}"
            )
        if is_leaf:
            if leaf == "absent":
                raise FileExistsError(f"protocol target already exists: {path}")
            if leaf == "file" and not stat.S_ISREG(metadata.st_mode):
                raise ValueError(f"trusted leaf must be a regular file: {path}")
            if leaf == "directory" and not stat.S_ISDIR(metadata.st_mode):
                raise ValueError(f"trusted leaf must be a directory: {path}")
            return metadata
    raise RuntimeError("unreachable path validation state")


def _read_bound_source(
    path: Path,
    *,
    expected_sha256: str | None = None,
    expected_identity: Mapping | None = None,
) -> dict:
    """Hash stable bytes from a no-follow descriptor and retain path identity."""
    path = _absolute(path)
    _lstat_path(path, leaf="file")
    flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptor = os.open(path, flags)
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise ValueError(f"trusted leaf must be a regular file: {path}")
        digest = hashlib.sha256()
        bytes_read = 0
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
            bytes_read += len(chunk)
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    before_state = (
        before.st_dev,
        before.st_ino,
        before.st_size,
        before.st_mtime_ns,
    )
    after_state = (
        after.st_dev,
        after.st_ino,
        after.st_size,
        after.st_mtime_ns,
    )
    if before_state != after_state or bytes_read != after.st_size:
        raise ValueError(f"trusted source changed while reading: {path}")
    observed_path = _lstat_path(path, leaf="file")
    assert observed_path is not None
    path_state = (
        observed_path.st_dev,
        observed_path.st_ino,
        observed_path.st_size,
        observed_path.st_mtime_ns,
    )
    if path_state != after_state:
        raise ValueError(f"trusted source path changed while reading: {path}")
    identity = {
        "path": str(path),
        "sha256": digest.hexdigest(),
        "device": after.st_dev,
        "inode": after.st_ino,
        "size": after.st_size,
        "mtime_ns": after.st_mtime_ns,
    }
    if expected_sha256 is not None and identity["sha256"] != expected_sha256:
        raise ValueError(f"trusted source hash changed: {path}")
    if expected_identity is not None and identity != dict(expected_identity):
        raise ValueError(f"trusted source identity changed: {path}")
    return identity


def _run_git(
    repository_root: Path,
    arguments: list[str],
    *,
    text: bool,
) -> subprocess.CompletedProcess:
    completed = subprocess.run(
        ["git", "-C", str(repository_root), *arguments],
        capture_output=True,
        check=False,
        text=text,
    )
    if completed.returncode != 0:
        stderr = completed.stderr if text else completed.stderr.decode(
            "utf-8", errors="replace"
        )
        raise ValueError(
            f"Git command failed ({' '.join(arguments)}): {stderr.strip()}"
        )
    return completed


def _git_head(repository_root: Path) -> str:
    head = _run_git(
        repository_root,
        ["rev-parse", "--verify", "HEAD"],
        text=True,
    ).stdout.strip()
    if re.fullmatch(r"[0-9a-f]{40}", head) is None:
        raise ValueError("current Git HEAD is not a 40-character SHA-1 commit")
    return head


def _verify_git_repository(
    repository_root: Path,
    *,
    expected_head: str | None = None,
) -> str:
    top_level = _run_git(
        repository_root,
        ["rev-parse", "--show-toplevel"],
        text=True,
    ).stdout.strip()
    if Path(top_level) != repository_root:
        raise ValueError("repository_root is not the Git worktree top level")
    head = _git_head(repository_root)
    if expected_head is not None and head != expected_head:
        raise ValueError("implementation parent HEAD changed during registration")
    return head


def _verify_repository_sources(
    repository_root: Path,
    *,
    head: str,
    identities: Mapping[str, Mapping],
) -> None:
    relative_paths = [
        relative.as_posix() for relative in REPOSITORY_SOURCE_PATHS.values()
    ]
    dirty = _run_git(
        repository_root,
        [
            "status",
            "--porcelain=v1",
            "--untracked-files=no",
            "--",
            *relative_paths,
        ],
        text=True,
    ).stdout
    if dirty:
        raise ValueError(
            "dirty tracked implementation files cannot be registered: "
            + dirty.strip()
        )
    for name, relative in REPOSITORY_SOURCE_PATHS.items():
        tracked = _run_git(
            repository_root,
            ["ls-files", "--error-unmatch", "--", relative.as_posix()],
            text=True,
        ).stdout.strip()
        if tracked != relative.as_posix():
            raise ValueError(f"implementation source is not tracked: {relative}")
        blob = _run_git(
            repository_root,
            ["show", f"{head}:{relative.as_posix()}"],
            text=False,
        ).stdout
        blob_sha256 = hashlib.sha256(blob).hexdigest()
        if blob_sha256 != identities[name]["sha256"]:
            raise ValueError(
                f"live implementation source differs from HEAD blob: {relative}"
            )


def _external_source_declarations() -> dict[str, tuple[Path, str]]:
    return {
        "truth_data": (
            Path(replay.PRODUCTION_TRUTH_DATA_PATH),
            replay.PRODUCTION_TRUTH_DATA_SHA256,
        ),
        "input_manifest": (
            Path(replay.PRODUCTION_INPUT_MANIFEST_PATH),
            replay.PRODUCTION_INPUT_MANIFEST_SHA256,
        ),
        "baseline_process": (
            Path(replay.PRODUCTION_BASELINE_PROCESS_PATH),
            replay.PRODUCTION_BASELINE_PROCESS_SHA256,
        ),
    }


def _source_contract(
    repository_root: Path,
) -> tuple[dict[str, dict[str, str]], dict[str, dict]]:
    sources: dict[str, dict[str, str]] = {}
    identities: dict[str, dict] = {}
    for name, relative in REPOSITORY_SOURCE_PATHS.items():
        path = repository_root / relative
        expected = (
            replay.LEGACY_SOLVER_SHA256
            if name == "legacy_solver_source"
            else None
        )
        identity = _read_bound_source(path, expected_sha256=expected)
        identities[name] = identity
        sources[name] = {
            "path": identity["path"],
            "sha256": identity["sha256"],
        }
    for name, (path, expected_sha256) in _external_source_declarations().items():
        path = _absolute(path)
        identity = _read_bound_source(path, expected_sha256=expected_sha256)
        identities[name] = identity
        sources[name] = {
            "path": identity["path"],
            "sha256": identity["sha256"],
        }
    return sources, identities


def _build_protocol(*, head: str, sources: Mapping) -> dict:
    experiment = copy.deepcopy(replay.EXPERIMENT_CONTRACT)
    experiment.update(
        {
            "range_noise_seeds": list(replay.PRODUCTION_RANGE_NOISE_SEEDS),
            "max_frames": None,
        }
    )
    return {
        "schema_id": replay.PROTOCOL_SCHEMA_ID,
        "protocol_id": replay.PROTOCOL_ID,
        "implementation_parent_commit": head,
        "binding_design": copy.deepcopy(replay.BINDING_DESIGN),
        "sources": copy.deepcopy(dict(sources)),
        "experiment": experiment,
        "estimator_constants": copy.deepcopy(replay.ESTIMATOR_CONSTANTS),
        "status_contract": copy.deepcopy(replay.STATUS_CONTRACT),
        "ablation_contracts": copy.deepcopy(replay.ABLATION_CONTRACTS),
        "raw_schema": copy.deepcopy(replay.RAW_SCHEMA_DECLARATION),
        "analysis_schema": copy.deepcopy(replay.ANALYSIS_SCHEMA),
        "gates": copy.deepcopy(replay.GATES),
        "disk_contract": copy.deepcopy(replay.DISK_CONTRACT),
        "invocations": replay.production_invocation_contract(),
        "evidence_lifecycle": copy.deepcopy(replay.EVIDENCE_LIFECYCLE),
        "commands": replay.production_command_contract(sources),
    }


def _strict_json_bytes(value: object) -> bytes:
    return (
        json.dumps(
            value,
            allow_nan=False,
            indent=2,
            sort_keys=True,
        )
        + "\n"
    ).encode("utf-8")


def _markdown_bytes(protocol: Mapping) -> bytes:
    source_lines = [
        f"| `{name}` | `{declaration['path']}` | `{declaration['sha256']}` |"
        for name, declaration in sorted(protocol["sources"].items())
    ]
    command_sections = []
    for name, command in protocol["commands"].items():
        command_sections.extend(
            [
                f"### `{name}`",
                "",
                "```text",
                shlex.join(command),
                "```",
                "",
            ]
        )
    lines = [
        "# CBF2026 Predictive WNLS Stage 1 Protocol",
        "",
        "This protocol freezes a paired, single-trajectory development replay.",
        "It is not confirmatory paper evidence, and the paper gate is `CLOSED`.",
        "",
        f"- Protocol schema: `{protocol['schema_id']}`",
        f"- Protocol ID: `{protocol['protocol_id']}`",
        (
            "- Implementation parent: "
            f"`{protocol['implementation_parent_commit']}`"
        ),
        "- Registered retry allowed: `false`",
        "- Exact output roots, exclusive creation, and terminal manifests: required",
        "",
        "## Frozen sources",
        "",
        "| Name | Absolute path | SHA-256 |",
        "|---|---|---|",
        *source_lines,
        "",
        "## Frozen cohort and gates",
        "",
        "- Range-noise seeds: `20260727` through `20260746` (20 exact seeds)",
        (
            "- Variants: `prediction_expiry`, "
            "`fresh_reference_qualification`, `predictive_multistart`"
        ),
        "- Maximum published/fresh error gate: strictly below `50 m`",
        "- Fresh-or-predicted availability gate: at least `0.95`",
        "- Maximum public prediction age: `2` frames",
        "- Paper gate: `CLOSED`",
        "",
        "## Exact commands",
        "",
        *command_sections,
        "## Machine-readable contract",
        "",
        "The companion JSON file is authoritative for every constant, schema,",
        "ablation, invocation, command token, disk cap, gate, and lifecycle rule.",
        "",
    ]
    return "\n".join(lines).encode("utf-8")


def _directory_flags() -> int:
    return (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )


def _open_output_parent(path: Path) -> tuple[int, tuple[int, int]]:
    metadata = _lstat_path(path.parent, leaf="directory")
    assert metadata is not None
    descriptor = os.open(path.parent, _directory_flags())
    try:
        observed = os.fstat(descriptor)
        if (
            not stat.S_ISDIR(observed.st_mode)
            or (observed.st_dev, observed.st_ino)
            != (metadata.st_dev, metadata.st_ino)
        ):
            raise ValueError(f"output parent identity changed: {path.parent}")
    except BaseException as primary_error:
        close_faults = _close_descriptors((descriptor,))
        _add_cleanup_notes(
            primary_error,
            [
                ("output parent descriptor close", fault)
                for fault in close_faults
            ],
        )
        raise
    return descriptor, (observed.st_dev, observed.st_ino)


def _write_all(descriptor: int, payload: bytes) -> None:
    offset = 0
    while offset < len(payload):
        written = os.write(descriptor, payload[offset:])
        if written <= 0:
            raise OSError("short write while freezing protocol")
        offset += written


def _path_matches_descriptor(
    *,
    parent_descriptor: int,
    name: str,
    descriptor: int,
) -> bool:
    owned = os.fstat(descriptor)
    observed = os.stat(
        name,
        dir_fd=parent_descriptor,
        follow_symlinks=False,
    )
    return (
        stat.S_ISREG(observed.st_mode)
        and (observed.st_dev, observed.st_ino)
        == (owned.st_dev, owned.st_ino)
    )


def _rollback_owned_entry(
    *,
    parent_descriptor: int,
    name: str,
    descriptor: int,
) -> list[BaseException]:
    """Remove one owned entry durably, retaining state across transient faults."""
    faults: list[BaseException] = []
    removed = False
    for _ in range(3):
        try:
            if not removed:
                try:
                    owned = _path_matches_descriptor(
                        parent_descriptor=parent_descriptor,
                        name=name,
                        descriptor=descriptor,
                    )
                except FileNotFoundError:
                    removed = True
                else:
                    if not owned:
                        return faults
                    os.unlink(name, dir_fd=parent_descriptor)
                    removed = True
            os.fsync(parent_descriptor)
            return faults
        except BaseException as error:
            faults.append(error)
    return faults


def _close_descriptors(descriptors: tuple[int | None, ...]) -> list[BaseException]:
    """Attempt every close without allowing one fault to skip later resources."""
    faults: list[BaseException] = []
    for descriptor in descriptors:
        if descriptor is None:
            continue
        for _ in range(3):
            try:
                os.close(descriptor)
                break
            except BaseException as error:
                faults.append(error)
                try:
                    os.fstat(descriptor)
                except OSError:
                    break
    return faults


def _add_cleanup_notes(
    primary_error: BaseException,
    faults: list[tuple[str, BaseException]],
) -> None:
    for context, fault in faults:
        primary_error.add_note(
            "protocol cleanup fault "
            f"[{context}]: {type(fault).__name__}: {fault}"
        )


def _write_protocol_outputs(
    *,
    output_markdown: Path,
    output_json: Path,
    markdown_payload: bytes,
    json_payload: bytes,
    final_probe,
) -> None:
    if output_markdown == output_json:
        raise ValueError("Markdown and JSON protocol targets must differ")
    _lstat_path(output_markdown, leaf="absent")
    _lstat_path(output_json, leaf="absent")
    markdown_parent = json_parent = None
    markdown_descriptor = json_descriptor = None
    markdown_identity = json_identity = None
    flags = (
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    try:
        markdown_parent, markdown_identity = _open_output_parent(output_markdown)
        json_parent, json_identity = _open_output_parent(output_json)
        json_descriptor = os.open(
            output_json.name,
            flags,
            0o644,
            dir_fd=json_parent,
        )
        markdown_descriptor = os.open(
            output_markdown.name,
            flags,
            0o644,
            dir_fd=markdown_parent,
        )
        _write_all(json_descriptor, json_payload)
        _write_all(markdown_descriptor, markdown_payload)
        os.fsync(json_descriptor)
        os.fsync(markdown_descriptor)
        os.fsync(json_parent)
        if markdown_parent != json_parent:
            os.fsync(markdown_parent)
        final_probe()
        for path, parent, identity, descriptor in (
            (
                output_markdown,
                markdown_parent,
                markdown_identity,
                markdown_descriptor,
            ),
            (output_json, json_parent, json_identity, json_descriptor),
        ):
            parent_path = _lstat_path(path.parent, leaf="directory")
            assert parent_path is not None
            if (parent_path.st_dev, parent_path.st_ino) != identity:
                raise ValueError(f"output parent path changed: {path.parent}")
            if not _path_matches_descriptor(
                parent_descriptor=parent,
                name=path.name,
                descriptor=descriptor,
            ):
                raise ValueError(f"protocol target identity changed: {path}")
    except BaseException as primary_error:
        cleanup_faults: list[tuple[str, BaseException]] = []
        if markdown_parent is not None and markdown_descriptor is not None:
            cleanup_faults.extend(
                ("markdown target", fault)
                for fault in _rollback_owned_entry(
                    parent_descriptor=markdown_parent,
                    name=output_markdown.name,
                    descriptor=markdown_descriptor,
                )
            )
        if json_parent is not None and json_descriptor is not None:
            cleanup_faults.extend(
                ("JSON target", fault)
                for fault in _rollback_owned_entry(
                    parent_descriptor=json_parent,
                    name=output_json.name,
                    descriptor=json_descriptor,
                )
            )
        close_faults = _close_descriptors(
            (
                markdown_descriptor,
                json_descriptor,
                markdown_parent,
                json_parent,
            )
        )
        cleanup_faults.extend(
            ("descriptor close", fault) for fault in close_faults
        )
        markdown_descriptor = json_descriptor = None
        markdown_parent = json_parent = None
        _add_cleanup_notes(primary_error, cleanup_faults)
        raise
    finally:
        _close_descriptors(
            (
                markdown_descriptor,
                json_descriptor,
                markdown_parent,
                json_parent,
            )
        )


def register_stage1_protocol(
    *,
    repository_root: Path,
    output_markdown: Path,
    output_json: Path,
) -> dict:
    """Bind parent commit, exact file/input hashes, commands, seeds, and gates."""
    repository_root = _absolute(Path(repository_root))
    _lstat_path(repository_root, leaf="directory")
    output_markdown = _absolute(Path(output_markdown), base=repository_root)
    output_json = _absolute(Path(output_json), base=repository_root)
    head = _verify_git_repository(repository_root)
    sources, identities = _source_contract(repository_root)
    _verify_repository_sources(
        repository_root,
        head=head,
        identities=identities,
    )
    protocol = _build_protocol(head=head, sources=sources)
    json_payload = _strict_json_bytes(protocol)
    markdown_payload = _markdown_bytes(protocol)

    def final_probe() -> None:
        _verify_git_repository(repository_root, expected_head=head)
        for name, identity in identities.items():
            _read_bound_source(
                Path(identity["path"]),
                expected_sha256=identity["sha256"],
                expected_identity=identity,
            )
        _verify_repository_sources(
            repository_root,
            head=head,
            identities=identities,
        )

    _write_protocol_outputs(
        output_markdown=output_markdown,
        output_json=output_json,
        markdown_payload=markdown_payload,
        json_payload=json_payload,
        final_probe=final_probe,
    )
    return protocol


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Freeze the exact predictive-WNLS Stage-1 protocol."
    )
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--output-markdown", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    register_stage1_protocol(
        repository_root=arguments.repository_root,
        output_markdown=arguments.output_markdown,
        output_json=arguments.output_json,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
