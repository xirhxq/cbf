"""Run an immutable qualified-closure campaign.

The implementation is intentionally built test-first.  Public helpers in this
module are also used by the registrar and analyzer to keep root and identity
checks consistent.
"""

import gzip
import copy
import ctypes
import dataclasses
import hashlib
import argparse
import sys
import json
import math
import os
import re
import selectors
import shutil
import stat
import subprocess
import tempfile
import time
import uuid
from pathlib import Path


START_FREE_BYTES = 8_000_000_000
STOP_FREE_BYTES = 6_000_000_000
CACHE_CAP_BYTES = 2_000_000_000
DEVELOPMENT_RAW_ROOT = Path(
    "/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v1"
)
DEVELOPMENT_ANALYSIS_ROOT = Path(
    "/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v1"
)


def project_raw_estimator_tuple(
    raw_row: dict,
    *,
    campaign_id: str,
    condition: str,
    trajectory_seed: int,
    range_noise_seed: int,
    depth: int,
) -> dict:
    """Validate one raw producer row, then project its public lifecycle."""
    from scripts.diagnostics.replay_qualified_estimator import (
        validate_qualified_replay_row,
    )

    validate_qualified_replay_row(raw_row)
    public = raw_row["audit_bundle"]["lifecycle"]["public_output"]
    status = public["output_status"]
    available = status in {"fresh", "predicted"}
    if status == "fresh":
        radius = public["epsilon"]
    elif status == "predicted":
        radius = public["aged_modeled_radius"]
    else:
        radius = None
    return {
        "record_type": "estimator_tuple",
        "schema_version": "cbf2026-qualified-evidence-v1",
        "campaign_id": campaign_id,
        "condition": condition,
        "trajectory_seed": trajectory_seed,
        "range_noise_seed": range_noise_seed,
        "frame_index": raw_row["frame_index"],
        "robot_id": raw_row["robot_id"],
        "depth": depth,
        "output_status": status,
        "estimate": public["estimate"] if available else None,
        "radius": radius,
    }


def orchestrate_estimator_conditions(
    *,
    measurement_path: Path,
    measurement_manifest: dict,
    command_history_path: Path,
    config_paths: dict,
    output_paths: dict,
    producer,
) -> dict:
    """Run both registered estimator producers over one immutable input."""
    measurement_path = Path(measurement_path)
    command_history_path = Path(command_history_path)
    conditions = ("dynamic_primary", "fixed_fim_ablation")
    if set(config_paths) != set(conditions) or set(output_paths) != set(conditions):
        raise ValueError("exactly the two registered estimator conditions are required")
    if (
        not isinstance(measurement_manifest, dict)
        or measurement_manifest.get("terminal") is not True
        or measurement_manifest.get("status") != "completed"
    ):
        raise ValueError("measurement manifest is not terminal and successful")
    observed_sha = sha256_path(measurement_path)
    if measurement_manifest.get("sha256") != observed_sha:
        raise ValueError("measurement SHA-256 does not match its manifest")
    if not command_history_path.is_file() or command_history_path.is_symlink():
        raise ValueError("held-command history must be an immutable regular file")
    forbidden = ("truth", "noiseless", "sampled_noise", "audit")
    for path in (measurement_path, command_history_path):
        lowered = str(path).lower()
        if any(fragment in lowered for fragment in forbidden):
            raise ValueError("producer input path contains analyzer-only provenance")

    results = {}
    for condition in conditions:
        config_path = Path(config_paths[condition])
        output_path = Path(output_paths[condition])
        if not config_path.is_file() or config_path.is_symlink():
            raise ValueError(f"{condition} config must be a regular file")
        if output_path.exists() or output_path.is_symlink():
            raise FileExistsError(f"producer output must be absent: {output_path}")
        lifecycle_state = {"public": {}, "private": {}}
        results[condition] = producer(
            condition=condition,
            measurement_path=measurement_path,
            measurement_sha256=observed_sha,
            measurement_stream_id=measurement_manifest.get(
                "measurement_stream_id"
            ),
            command_history_path=command_history_path,
            config_path=config_path,
            output_path=output_path,
            lifecycle_state=lifecycle_state,
        )
    return results


def sha256_path(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def development_schedule() -> list[dict]:
    """Return the frozen ten-mission development schedule."""
    return [
        {
            "campaign_id": "development-v1",
            "mission_id": f"mission-{index:02d}",
            "trajectory_seed": trajectory_seed,
            "range_noise_seed": range_noise_seed,
            "frames": 1000,
            "horizon_s": 500.0,
            "conditions": ["dynamic_primary", "fixed_fim_ablation"],
        }
        for index, (trajectory_seed, range_noise_seed) in enumerate(
            zip(
                range(2026080101, 2026080111),
                range(2026081101, 2026081111),
                strict=True,
            ),
            start=1,
        )
    ]


def write_schedule_no_replace(path: Path, schedule: list[dict]) -> None:
    """Publish a complete mission schedule before any child launch."""
    _publish_json_no_replace(
        Path(path),
        {
            "schema_version": "cbf2026-qualified-campaign-schedule-v1",
            "mission_count": len(schedule),
            "missions": schedule,
        },
    )


def materialize_primary_config(
    base_path: Path,
    overlay_path: Path,
    output_path: Path,
    mission: dict,
) -> dict:
    """Deep-merge the registered primary overlay and bind one mission."""
    from scripts.diagnostics.qualified_config import validate_qualified_config

    base = _read_json_object(Path(base_path), "base config")
    overlay = _read_json_object(Path(overlay_path), "primary overlay")
    if not validate_qualified_config(overlay):
        raise ValueError("primary overlay fails strict qualified validation")
    if overlay["position_covariance"]["reference-selection"] != "dynamic-lower-index":
        raise ValueError("Swarm may only receive the dynamic primary overlay")
    config = deep_merge(base, overlay)
    config.setdefault("execute", {})["random-seed"] = mission["trajectory_seed"]
    config["execute"]["time-total"] = mission["horizon_s"]
    config["output_path"] = str(Path(output_path).parent)
    config["run_suffix"] = f"_{mission['mission_id']}"
    config["evidence-stream"] = {
        "enabled": True,
        "schema-version": "cbf2026-qualified-evidence-v1",
        "campaign-id": mission["campaign_id"],
        "trajectory-seed": mission["trajectory_seed"],
        "range-noise-seed": mission["range_noise_seed"],
        "condition": "dynamic_primary",
    }
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    _publish_json_no_replace(Path(output_path), config)
    return config


def deep_merge(base: dict, overlay: dict) -> dict:
    """Recursively merge JSON objects without mutating either operand."""
    merged = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _read_json_object(path: Path, label: str) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object")
    return value


def validate_new_campaign_root(
    root: Path,
    project_root: Path,
    *,
    available_bytes_fn=None,
) -> None:
    """Validate a not-yet-allocated campaign root."""
    root = Path(root)
    project_root = Path(project_root).resolve()
    if root.is_symlink():
        raise ValueError("campaign root must not be a symbolic link")
    if root.exists():
        raise FileExistsError(f"campaign root must be absent: {root}")

    candidate = root.parent.resolve() / root.name
    if candidate == project_root or project_root in candidate.parents:
        raise ValueError("campaign root must resolve outside the repository")

    ancestor = root.parent
    while not ancestor.exists():
        if ancestor.parent == ancestor:
            raise FileNotFoundError(f"no existing ancestor for {root}")
        ancestor = ancestor.parent
    probe = available_bytes_fn or (lambda path: shutil.disk_usage(path).free)
    free_bytes = probe(ancestor)
    if free_bytes < START_FREE_BYTES:
        raise RuntimeError(
            f"campaign launch requires at least 8 GB free; available={free_bytes}"
        )


def claim_campaign_root(
    root: Path,
    project_root: Path,
    *,
    available_bytes_fn=None,
    identity_loader=None,
) -> Path:
    """Claim a campaign root after validation."""
    validate_new_campaign_root(
        root,
        project_root,
        available_bytes_fn=available_bytes_fn,
    )
    if identity_loader is not None:
        identity_loader()
    root = Path(root)
    root.parent.mkdir(parents=True, exist_ok=True)
    root.mkdir(exist_ok=False)
    return root


def supervise_child_to_gzip(
    argv,
    *,
    stream_path: Path,
    stderr_path: Path,
    manifest_path: Path,
    expected_keys=(),
    key_from_row=None,
    available_bytes_fn=None,
    cache_root: Path | None = None,
    allocated_bytes_fn=None,
    poll_interval_s=0.1,
    terminate_grace_s=5.0,
    synthetic_path: Path | None = None,
    wallclock_timeout_s=3600.0,
    line_stall_timeout_s=300.0,
    row_validator=None,
    max_line_bytes=16 * 1024 * 1024,
    synthesized_row_factory=None,
    cwd: Path | None = None,
) -> dict:
    """Stream exact ordered rows with O(1) schedule accounting."""
    import itertools

    stream_path = Path(stream_path)
    stderr_path = Path(stderr_path)
    manifest_path = Path(manifest_path)
    synthetic_path = Path(synthetic_path) if synthetic_path else stream_path.with_name(
        "missing.jsonl.gz"
    )
    for path in (stream_path, synthetic_path, stderr_path, manifest_path):
        if path.exists() or path.is_symlink():
            raise FileExistsError(f"output must be absent: {path}")
        path.parent.mkdir(parents=True, exist_ok=True)

    sentinel = object()
    enforce_order = expected_keys is not None
    expected_iterator = iter(()) if expected_keys is None else iter(expected_keys)
    next_expected = next(expected_iterator, sentinel)
    previous_key = sentinel
    available_probe = available_bytes_fn or (
        lambda path: shutil.disk_usage(path).free
    )
    allocation_probe = allocated_bytes_fn or allocated_bytes
    status, reason = "completed", "completed"
    valid_rows = 0
    process = None

    with stderr_path.open("xb") as stderr_file, stream_path.open("xb") as raw:
        with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as sink:
            try:
                process = subprocess.Popen(
                    list(argv),
                    stdout=subprocess.PIPE,
                    stderr=stderr_file,
                    bufsize=0,
                    close_fds=True,
                    cwd=None if cwd is None else Path(cwd),
                )
            except BaseException as error:
                status, reason = "failed", "child_launch_error"
                stderr_file.write(f"{type(error).__name__}: {error}\n".encode())

            if process is not None:
                selector = selectors.DefaultSelector()
                selector.register(process.stdout, selectors.EVENT_READ)
                started = time.monotonic()
                last_complete_line = started
                buffer = bytearray()
                stdout_eof = False
                try:
                    while True:
                        try:
                            now = time.monotonic()
                            if available_probe(stream_path.parent) < STOP_FREE_BYTES:
                                status, reason = "failed", "disk_hard_floor"
                                break
                            if cache_root is not None and (
                                allocation_probe(Path(cache_root)) > CACHE_CAP_BYTES
                            ):
                                status, reason = "failed", "cache_cap"
                                break
                        except BaseException as error:
                            status, reason = "failed", "runner_monitor_error"
                            stderr_file.write(
                                f"{type(error).__name__}: {error}\n".encode()
                            )
                            break
                        if now - started >= wallclock_timeout_s:
                            status, reason = "failed", "wallclock_timeout"
                            break
                        if now - last_complete_line >= line_stall_timeout_s:
                            status, reason = "failed", "line_stall_timeout"
                            break
                        events = selector.select(poll_interval_s)
                        for selected, _mask in events:
                            chunk = os.read(selected.fileobj.fileno(), 64 * 1024)
                            if not chunk:
                                selector.unregister(selected.fileobj)
                                stdout_eof = True
                                continue
                            buffer.extend(chunk)
                            if len(buffer) > max_line_bytes and b"\n" not in buffer:
                                status, reason = "failed", "line_too_large"
                                break
                            while b"\n" in buffer:
                                line, _, remainder = buffer.partition(b"\n")
                                complete_line = bytes(line) + b"\n"
                                buffer[:] = remainder
                                if len(complete_line) > max_line_bytes:
                                    status, reason = "failed", "line_too_large"
                                    break
                                try:
                                    row = _strict_json_object(complete_line)
                                except ValueError:
                                    status, reason = "failed", "malformed_json"
                                    break
                                try:
                                    valid_schema = (
                                        True if row_validator is None else row_validator(row)
                                    )
                                except BaseException:
                                    valid_schema = False
                                if valid_schema is not True:
                                    status, reason = "failed", "schema_error"
                                    break
                                try:
                                    row_key = key_from_row(row) if key_from_row else valid_rows
                                except BaseException:
                                    status, reason = "failed", "schema_error"
                                    break
                                if enforce_order:
                                    if previous_key is not sentinel and row_key == previous_key:
                                        status, reason = "failed", "duplicate_key"
                                        break
                                    if next_expected is sentinel or row_key != next_expected:
                                        status, reason = "failed", "unexpected_key"
                                        break
                                sink.write(complete_line)
                                previous_key = row_key
                                if enforce_order:
                                    next_expected = next(expected_iterator, sentinel)
                                valid_rows += 1
                                last_complete_line = time.monotonic()
                            if status == "failed":
                                break
                        if status == "failed":
                            break
                        returncode = process.poll()
                        if returncode is not None and stdout_eof:
                            if buffer:
                                status, reason = "failed", "malformed_json"
                            elif returncode < 0:
                                status, reason = "failed", "child_signal"
                            elif returncode > 0:
                                status, reason = "failed", "child_nonzero_exit"
                            break
                except BaseException as error:
                    status, reason = "failed", "runner_monitor_error"
                    stderr_file.write(f"{type(error).__name__}: {error}\n".encode())
                finally:
                    selector.close()
                if status == "failed":
                    _stop_child(process, terminate_grace_s)
                process.stdout.close()

    if status == "completed" and enforce_order and next_expected is not sentinel:
        status, reason = "failed", "incomplete_key_universe"
    remaining = (
        () if not enforce_order or next_expected is sentinel
        else itertools.chain((next_expected,), expected_iterator)
    )
    missing_count = _write_synthetic_rows(
        synthetic_path,
        remaining,
        reason=reason,
        include_mission=status == "failed",
        row_factory=synthesized_row_factory,
    )
    manifest = {
        "schema_version": "cbf2026-qualified-campaign-manifest-v1",
        "terminal": True,
        "status": status,
        "reason": reason,
        "valid_rows": valid_rows,
        "missing_rows": missing_count,
        "returncode": None if process is None else process.poll(),
        "accounting_mode": "ordered_streaming",
        "max_line_bytes": max_line_bytes,
    }
    _publish_json_no_replace(manifest_path, manifest)
    return manifest


def _strict_json_object(line: bytes) -> dict:
    try:
        text = line.decode("utf-8", errors="strict")
        value = json.loads(
            text,
            parse_constant=lambda token: (_ for _ in ()).throw(
                ValueError(f"nonfinite JSON token: {token}")
            ),
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("stdout line is not strict JSON") from error
    if not isinstance(value, dict):
        raise ValueError("stdout line must contain one JSON object")
    _validate_finite_json(value)
    return value


def _validate_finite_json(value) -> None:
    if isinstance(value, float) and not (float("-inf") < value < float("inf")):
        raise ValueError("JSON value is not finite")
    if isinstance(value, dict):
        if not all(isinstance(key, str) for key in value):
            raise ValueError("JSON object key is not a string")
        for nested in value.values():
            _validate_finite_json(nested)
    elif isinstance(value, list):
        for nested in value:
            _validate_finite_json(nested)


def _json_compatible(value):
    if dataclasses.is_dataclass(value):
        return {
            key: _json_compatible(nested)
            for key, nested in dataclasses.asdict(value).items()
        }
    if isinstance(value, tuple):
        return [_json_compatible(item) for item in value]
    if isinstance(value, list):
        return [_json_compatible(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _json_compatible(nested) for key, nested in value.items()}
    return value


def _write_synthetic_rows(
    path: Path,
    missing_keys,
    *,
    reason: str,
    include_mission: bool,
    row_factory=None,
) -> int:
    count = 0
    with Path(path).open("xb") as raw:
        with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as sink:
            for key in missing_keys:
                row = (
                    row_factory(key, reason)
                    if row_factory is not None
                    else {
                        "record_type": "missing",
                        "key": _json_compatible(key),
                        "reason": reason,
                    }
                )
                sink.write(_strict_json_bytes(row))
                sink.write(b"\n")
                count += 1
            if include_mission:
                sink.write(_strict_json_bytes({
                    "record_type": "mission",
                    "success": False,
                    "reason": reason,
                }))
                sink.write(b"\n")
    return count


def _strict_json_bytes(value, *, indent=None) -> bytes:
    _validate_finite_json(value)
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":") if indent is None else None,
        indent=indent,
    ).encode("utf-8")


def allocated_bytes(path: Path) -> int:
    """Count allocated blocks recursively without following symbolic links."""
    path = Path(path)

    def visit(current: Path) -> int:
        metadata = current.lstat()
        total = metadata.st_blocks * 512
        if not stat.S_ISDIR(metadata.st_mode):
            return total
        with os.scandir(current) as entries:
            for entry in entries:
                total += visit(Path(entry.path))
        return total

    return visit(path)


def _stop_child(process, grace_s: float) -> None:
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def _publish_json_no_replace(path: Path, payload: dict) -> None:
    encoded = json.dumps(
        payload,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8") + b"\n"
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("xb") as output:
            output.write(encoded)
            output.flush()
            os.fsync(output.fileno())
        os.link(temporary, path)
        parent_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(parent_fd)
        finally:
            os.close(parent_fd)
    finally:
        temporary.unlink(missing_ok=True)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run one immutable qualified-closure campaign"
    )
    parser.add_argument("--kind", choices=("development", "confirmatory", "confirmatory-smoke"), required=True)
    parser.add_argument("--version")
    parser.add_argument("--smoke-id", choices=("a", "b"))
    parser.add_argument("--protocol", type=Path, required=True)
    parser.add_argument("--authorization", type=Path, required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--base-config", type=Path, required=True)
    parser.add_argument("--primary-config", type=Path, required=True)
    parser.add_argument("--ablation-config", type=Path, required=True)
    parser.add_argument("--trajectory-seeds", required=True)
    parser.add_argument("--range-noise-seeds", required=True)
    parser.add_argument("--frames", type=int, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    return parser


def main(argv=None) -> int:
    incoming = list(sys.argv[1:] if argv is None else argv)
    arguments = build_argument_parser().parse_args(incoming)
    try:
        return int(run_campaign_from_arguments(arguments))
    except Exception as error:
        print(f"qualified campaign failed: {error}", file=sys.stderr)
        return 1


def run_campaign_from_arguments(arguments) -> int:
    operations = ProductionOperations()
    manifest = execute_campaign(arguments, operations)
    return 0 if manifest.get("status") == "completed" else 1


def _validate_runtime_runner_argv(protocol, arguments) -> None:
    """Rebuild the exact registered invocation from live parsed arguments."""
    actual = _runtime_runner_argv(arguments)
    if arguments.kind == "confirmatory-smoke":
        expected = protocol.get("smoke_argv", {}).get(
            arguments.smoke_id, {}
        ).get("runner")
    else:
        expected = protocol.get("runner_argv")
    if not isinstance(expected, list) or actual != expected:
        raise ValueError("runtime runner argv differs from registered protocol")


def _runtime_runner_argv(arguments) -> list[str]:
    prefix = [
        "conda", "run", "-n", "cbf_env", "python",
        "scripts/diagnostics/run_qualified_closure_campaign.py",
    ]
    shared = [
        "--protocol", str(arguments.protocol),
        "--authorization", str(arguments.authorization),
        "--binary", str(arguments.binary),
        "--base-config", str(arguments.base_config),
        "--primary-config", str(arguments.primary_config),
        "--ablation-config", str(arguments.ablation_config),
        "--trajectory-seeds", arguments.trajectory_seeds,
        "--range-noise-seeds", arguments.range_noise_seeds,
        "--frames", str(arguments.frames),
        "--output-root", str(arguments.output_root),
    ]
    if arguments.kind == "confirmatory-smoke":
        return prefix + [
            "--kind", arguments.kind,
            "--smoke-id", arguments.smoke_id,
            *shared,
        ]
    return prefix + [
        "--kind", arguments.kind,
        "--version", arguments.version,
        *shared,
    ]


class ProductionOperations:
    """Concrete, fail-closed operations used by the public campaign CLI."""

    def __init__(self):
        self.protocol = None

    def validate_registration(self, arguments):
        from scripts.diagnostics.register_qualified_closure_campaign import (
            validate_authorization_binding,
        )
        authorization = validate_authorization_binding(
            arguments.protocol, arguments.authorization
        )
        protocol = _read_json_object(arguments.protocol, "registered protocol")
        registered_kind = "confirmatory" if arguments.kind == "confirmatory-smoke" else arguments.kind
        if protocol.get("kind") != registered_kind:
            raise ValueError("runner kind differs from registered protocol")
        if arguments.kind != "confirmatory-smoke" and protocol.get("version") != arguments.version:
            raise ValueError("runner version differs from registered protocol")
        root_label = (
            f"smoke_{arguments.smoke_id}_raw"
            if arguments.kind == "confirmatory-smoke" else "raw"
        )
        registered_root = protocol.get("roots", {}).get(root_label)
        if registered_root != str(Path(arguments.output_root).resolve()):
            raise ValueError("runner output root differs from registered root")
        schedule = (
            protocol.get("smoke_schedule", {})
            if arguments.kind == "confirmatory-smoke" else protocol.get("schedule", {})
        )
        observed_trajectory = _parse_seed_expression(arguments.trajectory_seeds)
        observed_noise = _parse_seed_expression(arguments.range_noise_seeds)
        expected_trajectory = (
            [schedule.get("trajectory_seed")]
            if arguments.kind == "confirmatory-smoke"
            else schedule.get("trajectory_seeds")
        )
        expected_noise = (
            [schedule.get("range_noise_seed")]
            if arguments.kind == "confirmatory-smoke"
            else schedule.get("range_noise_seeds")
        )
        if expected_trajectory != observed_trajectory:
            raise ValueError("runner trajectory schedule differs from protocol")
        if expected_noise != observed_noise:
            raise ValueError("runner range-noise schedule differs from protocol")
        if schedule.get("frames") != arguments.frames or protocol.get("no_retry") is not True:
            raise ValueError("runner frame/retry policy differs from protocol")
        _validate_runtime_runner_argv(protocol, arguments)
        self.protocol = protocol
        return {
            "protocol_sha256": sha256_path(arguments.protocol),
            "authorization_sha256": sha256_path(arguments.authorization),
            "implementation_identity": authorization["implementation_identity"],
        }

    def collect_identities(self, arguments):
        if self.protocol is None:
            raise RuntimeError("registration must be validated first")
        mapping = {
            "binary": arguments.binary,
            "base_config": arguments.base_config,
            "primary_config": arguments.primary_config,
            "ablation_config": arguments.ablation_config,
        }
        result = {}
        for label, path in mapping.items():
            path = Path(path)
            registered = self.protocol["bindings"][label]
            if path.is_symlink() or not path.is_file():
                raise ValueError(f"{label} is not a regular file")
            observed = sha256_path(path)
            if (
                path.resolve() != Path(registered.get("path", "")).resolve()
                or observed != registered.get("sha256")
                or path.stat().st_size != registered.get("bytes")
            ):
                raise ValueError(f"{label} identity differs from protocol")
            result[f"{label}_sha256"] = observed
        return result

    def schedule_frozen(self, schedule):
        if schedule and schedule[0]["campaign_id"].startswith("confirmatory-smoke"):
            smoke = self.protocol.get("smoke_schedule", {})
            registered = [{
                "mission_id": "mission-01",
                "trajectory_seed": smoke.get("trajectory_seed"),
                "range_noise_seed": smoke.get("range_noise_seed"),
                "frames": smoke.get("frames"),
            }]
        else:
            registered = self.protocol.get("schedule", {}).get("missions")
        projection = [
            {
                "mission_id": mission["mission_id"],
                "trajectory_seed": mission["trajectory_seed"],
                "range_noise_seed": mission["range_noise_seed"],
                "frames": mission["frames"],
            }
            for mission in schedule
        ]
        if projection != registered:
            raise ValueError("materialized schedule differs from registered missions")

    def materialize_config(self, arguments, mission, config_path):
        config = materialize_primary_config(
            arguments.base_config,
            arguments.primary_config,
            config_path,
            mission,
        )
        return {"config": config, "sha256": sha256_path(config_path)}

    def run_swarm(self, mission, mission_stage, config_path):
        state = _SwarmStreamState(mission)
        supervisor = supervise_child_to_gzip(
            [str(self.protocol["bindings"]["binary"]["path"]), str(config_path)],
            stream_path=mission_stage / "swarm.jsonl.gz",
            stderr_path=mission_stage / "swarm.stderr.log",
            manifest_path=mission_stage / "swarm.supervisor.manifest.json",
            synthetic_path=mission_stage / "swarm.missing.jsonl.gz",
            expected_keys=None,
            row_validator=state,
            max_line_bytes=64 * 1024 * 1024,
        )
        if supervisor["status"] != "completed" or not state.complete:
            return {
                "terminal": True,
                "status": "failed",
                "reason": supervisor["reason"] if supervisor["status"] != "completed" else state.reason,
            }
        if not state.success:
            return {
                "terminal": True,
                "status": "failed",
                "reason": state.reason,
            }
        inputs = _extract_swarm_inputs(
            mission_stage / "swarm.jsonl.gz", mission_stage / "swarm-inputs"
        )
        return {
            "terminal": True,
            "status": "completed",
            "reason": "completed",
            "evidence_path": mission_stage / "swarm.jsonl.gz",
            "evidence_sha256": sha256_path(mission_stage / "swarm.jsonl.gz"),
            "commands_path": inputs["commands_path"],
            "truth_path": inputs["truth_path"],
            "truth_manifest": inputs["truth_manifest"],
            "edge_schedule": inputs["edge_schedule"],
            "config_sha256": sha256_path(config_path),
        }

    def generate_measurements(self, mission, mission_stage, swarm):
        from scripts.diagnostics.generate_qualified_measurements import (
            generate_measurement_bundle,
        )
        root = mission_stage / "measurements"
        manifest = generate_measurement_bundle(
            swarm["truth_path"],
            root,
            range_noise_seed=mission["range_noise_seed"],
            ranging_sigma=0.5,
            config_sha256=swarm["config_sha256"],
            required_edges_by_condition=swarm["edge_schedule"],
            truth_manifest=swarm["truth_manifest"],
        )
        return {
            **manifest,
            "runtime_path": root / "runtime.jsonl.gz",
            "manifest_path": root / "manifest.json",
            "sha256": manifest["runtime_sha256"],
        }

    def run_replay(self, mission, mission_stage, condition, measurements, state):
        config_path = mission_stage / f"materialized-{condition}.json"
        if condition == "dynamic_primary":
            source_config = mission_stage / "materialized-primary.json"
            if config_path != source_config:
                os.link(source_config, config_path)
        else:
            base = Path(self.protocol["bindings"]["base_config"]["path"])
            overlay = Path(self.protocol["bindings"]["ablation_config"]["path"])
            config = deep_merge(
                _read_json_object(base, "base config"),
                _read_json_object(overlay, "ablation config"),
            )
            config.setdefault("execute", {})["random-seed"] = mission["trajectory_seed"]
            config["execute"]["time-total"] = mission["horizon_s"]
            _publish_json_no_replace(config_path, config)
        producer_root = _prepare_replay_namespace(
            mission_stage,
            condition,
            measurements["runtime_path"],
            measurements,
            mission_stage / "swarm-inputs" / "commands.jsonl.gz",
            config_path,
        )
        output = mission_stage / f"replay-{condition}.raw.jsonl.gz"
        expected = (
            (frame, robot)
            for frame in range(1, mission["frames"])
            for robot in range(1, 15)
        )
        argv = [
            sys.executable,
            str(
                Path(__file__).with_name(
                    "replay_qualified_estimator.py"
                ).resolve()
            ),
            "--condition", condition,
            "--measurements", "measurements.jsonl.gz",
            "--measurement-sha256", measurements["sha256"],
            "--measurement-manifest", "measurements.manifest.json",
            "--commands", "commands.jsonl.gz",
            "--commands-manifest", "commands.manifest.json",
            "--config", "config.json",
            "--frames", str(mission["frames"]),
        ]
        producer_input_identities = _directory_member_identities(producer_root)
        try:
            manifest = supervise_child_to_gzip(
                argv,
                stream_path=output,
                stderr_path=mission_stage / f"replay-{condition}.stderr.log",
                manifest_path=mission_stage / f"replay-{condition}.manifest.json",
                synthetic_path=mission_stage / f"replay-{condition}.missing.jsonl.gz",
                expected_keys=expected,
                key_from_row=lambda row: (row["frame_index"], row["robot_id"]),
                row_validator=_validate_raw_replay_row,
                cwd=producer_root,
            )
        finally:
            shutil.rmtree(producer_root)
        return {
            **manifest,
            "path": output,
            "sha256": sha256_path(output),
            "measurement_sha256": measurements["sha256"],
            "campaign_id": mission["campaign_id"],
            "condition": condition,
            "trajectory_seed": mission["trajectory_seed"],
            "range_noise_seed": mission["range_noise_seed"],
            "frames": mission["frames"],
            "config_sha256": sha256_path(config_path),
            "producer_input_identities": producer_input_identities,
            "producer_argv": argv,
        }

    def synthesize_failed_mission(self, mission, mission_stage, reason):
        from scripts.diagnostics.qualified_closure_evidence import (
            FrozenMissionSchedule,
            synthesize_missing_mission,
        )
        frozen = FrozenMissionSchedule(
            mission["campaign_id"], mission["trajectory_seed"],
            mission["range_noise_seed"], mission["frames"],
            tuple(range(1, 15)), tuple(mission["conditions"]), "dynamic_primary",
        )
        missing = synthesize_missing_mission(frozen, reason)
        observed = _observed_failed_mission_keys(mission_stage, mission)
        with (mission_stage / "synthetic-missing.jsonl.gz").open("xb") as raw:
            with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as sink:
                for kind in ("initialization", "estimator", "controller", "endpoint", "reconstructed", "reset"):
                    for key in getattr(missing, kind):
                        if key in observed[kind]:
                            continue
                        sink.write(_strict_json_bytes({
                            "record_type": f"missing_{kind}",
                            "key": _serialize_frozen_key(key),
                            "reason": reason,
                        }) + b"\n")
                sink.write(_strict_json_bytes({"record_type": "missing_mission", "reason": reason}) + b"\n")


def _observed_failed_mission_keys(mission_stage, mission):
    """Reconstruct frozen keys already committed in retained valid prefixes."""
    from scripts.diagnostics.qualified_closure_evidence import (
        CanonicalEdgeId,
        _PAPER_EDGES,
    )

    observed = {
        kind: set()
        for kind in (
            "initialization", "estimator", "controller", "endpoint",
            "reconstructed", "reset",
        )
    }
    swarm_path = Path(mission_stage) / "swarm.jsonl.gz"
    if swarm_path.is_file() and not swarm_path.is_symlink():
        state = _SwarmStreamState(mission)
        with gzip.open(swarm_path, "rb") as source:
            for line in source:
                row = _strict_json_object(line)
                if state(row) is not True:
                    raise ValueError("retained Swarm prefix is not valid")
                identity = (
                    mission["campaign_id"], row.get("condition"),
                    mission["trajectory_seed"], mission["range_noise_seed"],
                )
                record_type = row["record_type"]
                if record_type == "initialization":
                    observed["initialization"].add((
                        mission["campaign_id"], mission["trajectory_seed"],
                        mission["range_noise_seed"], 0, row["robot_id"],
                    ))
                elif record_type == "endpoint_row":
                    edge = row["edge"]
                    observed["endpoint"].add((
                        *identity, row["frame_index"],
                        CanonicalEdgeId(
                            edge["kind"], edge["low"], edge["high"],
                            edge["base_id"],
                        ),
                        row["owner"],
                    ))
                elif record_type == "controller_interval":
                    observed["controller"].add(
                        (*identity, row["frame_index"])
                    )
                    observed["reconstructed"].update(
                        (*identity, row["frame_index"], edge)
                        for edge in _PAPER_EDGES
                    )
                # Reset rows have no pre-registered frozen-key universe.

    for condition in mission["conditions"]:
        replay_path = Path(mission_stage) / f"replay-{condition}.raw.jsonl.gz"
        if not replay_path.is_file() or replay_path.is_symlink():
            continue
        expected_order = iter(
            (frame, robot)
            for frame in range(1, mission["frames"])
            for robot in range(1, 15)
        )
        with gzip.open(
            replay_path, "rt", encoding="utf-8", errors="strict"
        ) as source:
            for line in source:
                row = json.loads(
                    line,
                    parse_constant=lambda token: (_ for _ in ()).throw(
                        ValueError(token)
                    ),
                )
                _validate_raw_replay_row(row)
                order_key = (row["frame_index"], row["robot_id"])
                if order_key != next(expected_order, None):
                    raise ValueError(
                        "retained replay differs from frozen prefix order"
                    )
                observed["estimator"].add((
                    mission["campaign_id"], condition,
                    mission["trajectory_seed"], mission["range_noise_seed"],
                    row["frame_index"], row["robot_id"],
                ))
    return observed


def _serialize_frozen_key(value):
    """Convert frozen schedule keys, including edge dataclasses, to JSON."""
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return {
            field.name: _serialize_frozen_key(getattr(value, field.name))
            for field in dataclasses.fields(value)
        }
    if isinstance(value, (tuple, list)):
        return [_serialize_frozen_key(item) for item in value]
    if isinstance(value, dict):
        return {key: _serialize_frozen_key(item) for key, item in value.items()}
    return value


class _SwarmStreamState:
    """Validate exact record schemas and the bounded Swarm lifecycle order."""

    def __init__(self, mission):
        self.mission = mission
        self.initialization = 0
        self.last_controller = -1
        self.endpoint_frame = None
        self.endpoint_count = 0
        self.pending_reset = None
        self.snapshot_version = 0
        self.terminal = False
        self.success = False
        self.reason = "missing_mission_terminal"

    @property
    def complete(self):
        return self.terminal

    def __call__(self, row):
        from scripts.diagnostics.qualified_closure_evidence import (
            CanonicalEdgeId,
            _PAPER_ENDPOINTS,
            audit_reset_primitives,
            validate_controller_primitive_schema,
            validate_endpoint_primitive_schema,
            validate_initialization_schema,
            validate_mission_terminal_schema,
        )
        if self.terminal:
            self.reason = "row_after_mission_terminal"
            return False
        record_type = row.get("record_type")
        if not _swarm_row_matches_mission(row, self.mission, record_type):
            self.reason = "swarm_identity_mismatch"
            return False
        if record_type == "initialization":
            valid = (
                self.last_controller < 0
                and row.get("robot_id") == self.initialization + 1
                and validate_initialization_schema(row)
            )
            if valid:
                self.initialization += 1
            return valid
        if self.initialization != 14 and record_type != "mission_terminal":
            self.reason = "incomplete_initialization_prefix"
            return False
        if record_type == "reset":
            runtime = row.get("runtime", {})
            valid = (
                self.pending_reset is None
                and row.get("frame_index") == self.last_controller + 1
                and runtime.get("predecessor_version") == self.snapshot_version
                and runtime.get("proposed_version") == self.snapshot_version + 1
                and not audit_reset_primitives(row, tuple(range(1, 15)), 119)
            )
            if valid:
                self.pending_reset = row
            else:
                self.reason = "reset_lifecycle_mismatch"
            return valid
        if record_type == "endpoint_row":
            frame = row.get("frame_index")
            if self.endpoint_frame is None or frame != self.endpoint_frame:
                if self.endpoint_count not in {0, 232}:
                    self.reason = "incomplete_endpoint_frame"
                    return False
                self.endpoint_frame, self.endpoint_count = frame, 0
            edge = row.get("edge", {})
            try:
                observed = (
                    CanonicalEdgeId(
                        edge["kind"], edge["low"], edge["high"], edge["base_id"]
                    ),
                    row.get("owner"),
                )
            except (KeyError, TypeError, ValueError):
                observed = None
            expected = _PAPER_ENDPOINTS[self.endpoint_count]
            valid = (
                frame == self.last_controller + 1
                and observed == expected
                and validate_endpoint_primitive_schema(row)
            )
            if valid:
                self.endpoint_count += 1
            elif observed != expected:
                self.reason = "endpoint_order_mismatch"
            return valid and self.endpoint_count <= 232
        if record_type == "controller_interval":
            frame = row.get("frame_index")
            runtime = row.get("runtime", {})
            reset = runtime.get("reset", {})
            proposed = (
                None if self.pending_reset is None
                else self.pending_reset["runtime"]
            )
            reset_accepted = (
                proposed is not None
                and proposed.get("guard_decision") == "accepted"
            )
            expected_version = self.snapshot_version + int(reset_accepted)
            expected_attempted = proposed is not None
            expected_guard = (
                "accepted" if reset_accepted else
                "rejected" if expected_attempted else "not-attempted"
            )
            valid = (
                frame == self.last_controller + 1
                and self.endpoint_frame == frame
                and self.endpoint_count == 232
                and validate_controller_primitive_schema(row)
                and runtime.get("snapshot_version") == expected_version
                and runtime.get("allocation_version") == 1
                and reset.get("attempted") == expected_attempted
                and reset.get("guard_status") == expected_guard
                and reset.get("committed") == reset_accepted
            )
            if valid:
                self.last_controller = frame
                self.snapshot_version = expected_version
                self.pending_reset = None
                self.endpoint_frame = None
                self.endpoint_count = 0
            return valid
        if record_type == "mission_terminal":
            runtime = row.get("runtime", {})
            valid = (
                validate_mission_terminal_schema(row)
                and runtime.get("declared_frames") == self.mission["frames"]
                and runtime.get("completed_intervals") == self.last_controller + 1
                and (
                    runtime.get("success") is not True
                    or (
                        self.initialization == 14
                        and self.last_controller + 1 == self.mission["frames"]
                    )
                )
            )
            if valid:
                self.terminal = True
                self.success = runtime["success"] is True
                self.reason = runtime["reason"]
            return valid
        self.reason = "unknown_swarm_record_type"
        return False


def _swarm_row_matches_mission(row, mission, record_type):
    if not isinstance(row, dict):
        return False
    expected_condition = "dynamic_primary"
    if (
        row.get("campaign_id") != mission.get("campaign_id")
        or row.get("condition") != expected_condition
        or row.get("trajectory_seed") != mission.get("trajectory_seed")
        or row.get("range_noise_seed") != mission.get("range_noise_seed")
        or type(row.get("frame_index")) is not int
    ):
        return False
    frame = row["frame_index"]
    if record_type in {"initialization", "reset", "endpoint_row", "controller_interval"}:
        return 0 <= frame < mission["frames"]
    if record_type == "mission_terminal":
        return frame == mission["frames"]
    return False


def _extract_swarm_inputs(evidence_path: Path, output_root: Path) -> dict:
    """Read committed Swarm evidence once and atomically publish producer inputs."""
    output_root = Path(output_root)
    if output_root.exists() or output_root.is_symlink():
        raise FileExistsError(f"Swarm input bundle must be absent: {output_root}")
    stage = output_root.with_name(f".{output_root.name}.{uuid.uuid4().hex}.tmp")
    stage.mkdir(mode=0o700)
    truth_path = stage / "controller-truth.jsonl.gz"
    commands_path = stage / "commands.jsonl.gz"
    required = {
        "dynamic_primary": "controller_references",
        "fixed_fim_ablation": "fixed_paper_localization_references",
    }
    controller_count = 0
    try:
        with (
            gzip.open(evidence_path, "rt", encoding="utf-8", errors="strict") as source,
            truth_path.open("xb") as truth_raw,
            commands_path.open("xb") as command_raw,
            gzip.GzipFile(filename="", mode="wb", fileobj=truth_raw, mtime=0) as truth_sink,
            gzip.GzipFile(filename="", mode="wb", fileobj=command_raw, mtime=0) as command_sink,
        ):
            for line_number, line in enumerate(source, start=1):
                row = _strict_json_object(line.encode("utf-8"))
                if row.get("record_type") != "controller_interval":
                    continue
                controller_count += 1
                truth_sink.write(line.encode("utf-8"))
                commands = []
                for node in row["runtime"]["nodes"]:
                    robot_id = node["robot_id"]
                    commands.append({
                        "robot_id": robot_id,
                        "applied_command": node["applied_command"],
                    })
                command_sink.write(_strict_json_bytes({
                    "frame_index": row["frame_index"],
                    "commands": commands,
                }) + b"\n")
        truth_manifest = {
            "schema_version": "cbf2026-qualified-controller-truth-manifest-v1",
            "terminal": True,
            "status": "completed",
            "row_count": controller_count,
            "sha256": sha256_path(truth_path),
        }
        _publish_json_no_replace(stage / "truth.manifest.json", truth_manifest)
        _publish_json_no_replace(stage / "edge-schedule.json", {
            "schema_version": "cbf2026-qualified-edge-schedule-v1",
            "conditions": required,
        })
        _publish_json_no_replace(stage / "manifest.json", {
            "schema_version": "cbf2026-qualified-swarm-input-bundle-v1",
            "terminal": True,
            "status": "completed",
            "truth_sha256": truth_manifest["sha256"],
            "commands_sha256": sha256_path(commands_path),
            "controller_count": controller_count,
        })
        _rename_directory_no_replace(stage, output_root)
        return {
            "truth_path": output_root / truth_path.name,
            "commands_path": output_root / commands_path.name,
            "truth_manifest": truth_manifest,
            "edge_schedule": required,
        }
    finally:
        if stage.exists():
            shutil.rmtree(stage)


def _validate_raw_replay_row(row):
    from scripts.diagnostics.replay_qualified_estimator import (
        validate_qualified_replay_row,
    )
    validate_qualified_replay_row(row)

    def visit(value):
        if isinstance(value, dict):
            for key, nested in value.items():
                if key == "diameter_m" and (
                    type(nested) not in {int, float}
                    or not math.isfinite(float(nested))
                    or nested < 0.0
                ):
                    raise ValueError("qualified replay diameter is invalid")
                if key == "base_anchor_provenance" and (
                    not isinstance(nested, list)
                    or nested != sorted(set(nested))
                    or any(
                        type(anchor) is not int or anchor not in {0, 1, 2}
                        for anchor in nested
                    )
                ):
                    raise ValueError("qualified replay provenance is invalid")
                visit(nested)
        elif isinstance(value, list):
            for nested in value:
                visit(nested)

    visit(row)
    return True


def _prepare_replay_namespace(
    mission_stage, condition, measurement_path, measurement_manifest,
    commands_path, config_path,
):
    """Prepare the producer-only input namespace for one replay condition."""
    mission_stage = Path(mission_stage)
    target = Path(tempfile.mkdtemp(prefix=f"cbf2026-replay-{condition}-"))
    sources = {
        "measurements.jsonl.gz": Path(measurement_path),
        "commands.jsonl.gz": Path(commands_path),
        "config.json": Path(config_path),
    }
    if any(path.is_symlink() or not path.is_file() for path in sources.values()):
        raise ValueError("replay producer inputs must be regular files")
    if (
        not isinstance(measurement_manifest, dict)
        or measurement_manifest.get("terminal") is not True
        or measurement_manifest.get("status") != "completed"
        or sha256_path(sources["measurements.jsonl.gz"])
            != measurement_manifest.get("runtime_sha256")
    ):
        raise ValueError("replay producer measurement identity is invalid")
    try:
        for name, source in sources.items():
            os.link(source, target / name)
        _publish_json_no_replace(target / "measurements.manifest.json", {
            "schema_version": "cbf2026-qualified-producer-measurements-v1",
            "terminal": True,
            "status": "completed",
            "sha256": measurement_manifest["runtime_sha256"],
            "measurement_stream_id": measurement_manifest[
                "measurement_stream_id"
            ],
            "config_sha256": measurement_manifest["config_sha256"],
            "row_count": measurement_manifest["row_count"],
        })
        _publish_json_no_replace(target / "commands.manifest.json", {
            "schema_version": "cbf2026-qualified-producer-commands-v1",
            "terminal": True,
            "status": "completed",
            "sha256": sha256_path(sources["commands.jsonl.gz"]),
        })
        return target
    except BaseException:
        shutil.rmtree(target)
        raise


def _replay_producer_main(argv) -> int:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--condition", choices=("dynamic_primary", "fixed_fim_ablation"), required=True)
    parser.add_argument("--measurements", type=Path, required=True)
    parser.add_argument("--measurement-sha256", required=True)
    parser.add_argument("--measurement-manifest", type=Path, required=True)
    parser.add_argument("--commands", type=Path, required=True)
    parser.add_argument("--commands-manifest", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--frames", type=int, required=True)
    arguments = parser.parse_args(argv)
    try:
        _emit_replay_rows(arguments)
    except Exception as error:
        print(f"qualified replay producer failed: {error}", file=sys.stderr)
        return 1
    return 0


def _emit_replay_rows(arguments) -> None:
    """Task-8 process adapter; its argv/FDs contain no truth or audit path."""
    from scripts.diagnostics.predictive_wnls import (
        canonical_qualified_solver,
        solve_finite_budget_wnls,
    )
    forbidden = ("truth", "audit", "noiseless", "sampled_noise")
    for path in (
        arguments.measurements, arguments.measurement_manifest,
        arguments.commands, arguments.commands_manifest, arguments.config,
    ):
        if any(fragment in str(path).lower() for fragment in forbidden):
            raise ValueError("replay producer path contains analyzer-only provenance")
        if path.is_symlink() or not path.is_file():
            raise ValueError("replay producer input must be a regular file")
    if sha256_path(arguments.measurements) != arguments.measurement_sha256:
        raise ValueError("replay measurement identity changed")
    measurement_manifest = _read_json_object(
        arguments.measurement_manifest, "producer measurement manifest"
    )
    if (
        set(measurement_manifest) != {
            "schema_version", "terminal", "status", "sha256",
            "measurement_stream_id", "config_sha256", "row_count",
        }
        or measurement_manifest["schema_version"]
            != "cbf2026-qualified-producer-measurements-v1"
        or measurement_manifest["terminal"] is not True
        or measurement_manifest["status"] != "completed"
        or measurement_manifest["sha256"] != arguments.measurement_sha256
        or type(measurement_manifest["row_count"]) is not int
        or measurement_manifest["row_count"] < 0
    ):
        raise ValueError("producer measurement manifest is invalid")
    with gzip.open(
        arguments.measurements, "rt", encoding="utf-8", errors="strict"
    ) as measurement_source:
        observed_measurement_rows = sum(1 for _line in measurement_source)
    if observed_measurement_rows != measurement_manifest["row_count"]:
        raise ValueError("producer measurement row count changed")
    commands_manifest = _read_json_object(
        arguments.commands_manifest, "producer commands manifest"
    )
    if (
        set(commands_manifest) != {
            "schema_version", "terminal", "status", "sha256"
        }
        or commands_manifest["schema_version"]
            != "cbf2026-qualified-producer-commands-v1"
        or commands_manifest["terminal"] is not True
        or commands_manifest["status"] != "completed"
        or commands_manifest["sha256"] != sha256_path(arguments.commands)
    ):
        raise ValueError("producer commands manifest is invalid")
    config = _read_json_object(arguments.config, "selected replay config")
    selection = config.get("position_covariance", {}).get("reference-selection")
    expected_selection = (
        "dynamic-lower-index" if arguments.condition == "dynamic_primary"
        else "fixed-cbf-only"
    )
    if selection != expected_selection:
        raise ValueError("replay condition/config selection mismatch")

    deployment = config.get("qualified-estimator", {}).get("deployment")
    if not isinstance(deployment, dict):
        raise ValueError("qualified deployment contract is absent")
    domain = {key.replace("-", "_"): value for key, value in deployment.items()}
    history = config.get("qualified-estimator", {}).get("history")
    if not isinstance(history, dict):
        raise ValueError("qualified history contract is absent")
    innovation_limit = history.get("q-threshold")
    state = {robot: {"public": None, "private": None, "history": 0} for robot in range(1, 15)}

    def solver_from_start(start, references):
        return solve_finite_budget_wnls(
            [record["position"] for record in references],
            [record["covariance"] for record in references],
            [record["range"] for record in references],
            start.estimate,
            references[0]["ranging_sigma"],
        )

    solver = canonical_qualified_solver(solver_from_start)
    measurement_groups = iter(_iter_runtime_measurement_groups(
        arguments.measurements, arguments.condition, measurement_manifest
    ))
    next_measurement = next(measurement_groups, None)
    command_frames = iter(_iter_command_frames(arguments.commands))
    next_commands = next(command_frames, None)
    for frame in range(arguments.frames):
        if frame == 0:
            commands = {robot_id: [0.0, 0.0, 0.0] for robot_id in range(1, 15)}
        else:
            if next_commands is None or next_commands[0] != frame - 1:
                raise ValueError(f"missing held-command frame {frame - 1}")
            commands = next_commands[1]
            next_commands = next(command_frames, None)
        for robot_id in range(1, 15):
            expected_key = (frame, robot_id)
            if next_measurement is None or next_measurement[0] != expected_key:
                raise ValueError(f"missing replay references at {(frame, robot_id)}")
            references = next_measurement[1]
            next_measurement = next(measurement_groups, None)
            held = commands.get(robot_id)
            if held is None:
                raise ValueError(f"missing held command at {(frame - 1, robot_id)}")
            row = _build_condition_replay_row(
                frame=frame,
                robot_id=robot_id,
                raw_references=references,
                condition_state=state,
                held_velocity=list(held[:2]),
                deployment_domain=domain,
                innovation_limit=innovation_limit,
                solver=solver,
                mission_horizon_frames=arguments.frames,
            )
            if frame > 0:
                encoded = json.dumps(
                    row, ensure_ascii=False, allow_nan=False, separators=(",", ":")
                ).encode("utf-8") + b"\n"
                sys.stdout.buffer.write(encoded)
        if frame > 0:
            sys.stdout.buffer.flush()
    if next_measurement is not None or next_commands is not None:
        raise ValueError("replay input contains rows beyond the frozen horizon")


def _state_seed(state):
    if not isinstance(state, dict):
        return None
    estimate = state.get("estimate")
    covariance = state.get("modeled_covariance")
    if estimate is None or covariance is None:
        return None
    return {"estimate": estimate, "modeled_covariance": covariance}


def _resolve_condition_references(raw_references, condition_state, *, owner_id):
    """Resolve raw reference records against one condition-local lifecycle."""
    resolved = []
    for raw_reference in raw_references:
        reference = {
            "key": tuple(raw_reference["key"]),
            "range": raw_reference["range"],
            "ranging_sigma": raw_reference["ranging_sigma"],
        }
        kind, identifier = reference["key"]
        if kind == "base":
            reference.update({
                "position": copy.deepcopy(raw_reference["position"]),
                "covariance": copy.deepcopy(raw_reference["covariance"]),
                "base_anchor_provenance": copy.deepcopy(
                    raw_reference["base_anchor_provenance"]
                ),
            })
        elif kind == "uav":
            if type(identifier) is not int or identifier >= owner_id:
                raise ValueError("UAV reference is not a lower-index predecessor")
            predecessor = condition_state.get(identifier)
            public = None if predecessor is None else predecessor.get("public")
            if not isinstance(public, dict):
                continue
            status = public.get("output_status")
            if status == "unavailable":
                continue
            if status not in {"fresh", "predicted"}:
                raise ValueError("condition-local predecessor publication is invalid")
            predecessor_provenance = public.get("base_anchor_provenance")
            if (
                not isinstance(predecessor_provenance, list)
                or len(predecessor_provenance) < 2
            ):
                continue
            reference.update({
                "position": copy.deepcopy(public.get("estimate")),
                "covariance": copy.deepcopy(public.get("modeled_covariance")),
                "base_anchor_provenance": copy.deepcopy(
                    predecessor_provenance
                ),
            })
        else:
            raise ValueError("unsupported replay reference kind")
        resolved.append(reference)
    return resolved


def _build_condition_replay_row(
    *, frame, robot_id, raw_references, condition_state, held_velocity,
    deployment_domain, innovation_limit, solver, mission_horizon_frames,
):
    """Build one condition-local replay row and advance its owner state."""
    from scripts.diagnostics.replay_qualified_estimator import (
        build_qualified_replay_row,
    )

    robot_state = condition_state.get(robot_id)
    if not isinstance(robot_state, dict):
        raise ValueError("condition-local owner state is absent")
    references = _resolve_condition_references(
        raw_references, condition_state, owner_id=robot_id
    )
    provenance = sorted({
        anchor
        for reference in references
        for anchor in reference["base_anchor_provenance"]
    })
    if len(provenance) < 2:
        raise ValueError("fewer than two condition-local reference anchors remain")
    previous_public = robot_state.get("public")
    previous_private = robot_state.get("private")
    if frame == 0:
        qualifier_kind = "deployment"
        qualifier_payload = {"domain": deployment_domain}
        applied_command_frame = None
    elif previous_private is not None:
        qualifier_kind = "history"
        qualifier_payload = {"innovation_limit": innovation_limit}
        applied_command_frame = frame - 1
    else:
        qualifier_kind = "unavailable"
        qualifier_payload = {"reason": "propagated_private_prior_unavailable"}
        applied_command_frame = frame - 1
    row = build_qualified_replay_row(
        frame_index=frame,
        robot_id=robot_id,
        squad_local_index=robot_id if robot_id <= 7 else robot_id - 7,
        schedule_id=(
            f"frame-{frame}:robot-{robot_id}:squad-local-"
            f"{robot_id if robot_id <= 7 else robot_id - 7}"
        ),
        references=references,
        solver_from_start=solver,
        live_seed=_state_seed(previous_public),
        private_seed=_state_seed(previous_private),
        qualifier_kind=qualifier_kind,
        qualifier_payload=qualifier_payload,
        previous_public=previous_public,
        previous_private=previous_private,
        held_velocity=list(held_velocity),
        applied_command_frame=applied_command_frame,
        history_version=robot_state["history"],
        mission_horizon_frames=mission_horizon_frames,
        active_reference_count=len(references),
        base_anchor_provenance=provenance,
    )
    lifecycle = row["audit_bundle"]["lifecycle"]
    robot_state["public"] = lifecycle["public_output"]
    robot_state["private"] = lifecycle["next_private_state"]
    robot_state["history"] = lifecycle["history_version"] + 1
    return row


def _iter_runtime_measurement_groups(path: Path, condition: str, manifest=None):
    """Yield one strictly ordered owner/frame reference group at a time."""
    current_key = None
    current_references = []
    stream_identity = None
    previous_reference_key = None
    with gzip.open(path, "rt", encoding="utf-8", errors="strict") as source:
        for line in source:
            row = _strict_json_object(line.encode("utf-8"))
            if manifest is not None:
                _validate_runtime_measurement_row(
                    row, condition=None, manifest=manifest
                )
            elif row.get("record_type") != "runtime_measurement":
                raise ValueError("runtime measurement schema mismatch")
            identity = row.get("measurement_stream_id")
            if stream_identity is None:
                stream_identity = identity
            elif stream_identity != identity:
                raise ValueError("runtime measurement stream identity changed")
            if condition not in row.get("role_tags", []):
                continue
            key = (row.get("frame_index"), row.get("owner_id"))
            reference_key = tuple(row.get("reference_key", ()))
            ordered_key = (*key, reference_key)
            if previous_reference_key is not None and ordered_key <= previous_reference_key:
                raise ValueError("runtime measurements are not strictly ordered")
            previous_reference_key = ordered_key
            if current_key is not None and key != current_key:
                yield current_key, current_references
                current_references = []
            current_key = key
            current_references.append({
                "key": reference_key,
                "position": row["reference_position"],
                "range": row["noisy_range"],
                "covariance": row["reference_covariance"],
                "ranging_sigma": row["ranging_sigma"],
                "base_anchor_provenance": row["base_anchor_provenance"],
            })
    if current_key is not None:
        yield current_key, current_references


def _validate_runtime_measurement_row(row, *, condition, manifest):
    """Validate one exact producer-visible measurement row."""
    expected_fields = {
        "schema_version", "record_type", "frame_index", "owner_id",
        "reference_key", "role_tags", "reference_position",
        "reference_covariance", "base_anchor_provenance", "noisy_range",
        "ranging_sigma", "config_sha256", "measurement_stream_id",
    }
    if not isinstance(row, dict) or set(row) != expected_fields:
        raise ValueError("runtime measurement schema is not exact")
    if (
        row["schema_version"] != "cbf2026-qualified-measurements-v1"
        or row["record_type"] != "runtime_measurement"
        or type(row["frame_index"]) is not int
        or row["frame_index"] < 0
        or type(row["owner_id"]) is not int
        or row["owner_id"] not in range(1, 15)
    ):
        raise ValueError("runtime measurement identity is invalid")
    key = row["reference_key"]
    if (
        not isinstance(key, list)
        or len(key) != 2
        or key[0] not in {"base", "uav"}
        or type(key[1]) is not int
        or key[1] < 0
        or (key[0] == "base" and key[1] not in {0, 1, 2})
        or (key[0] == "uav" and key[1] not in range(1, row["owner_id"]))
    ):
        raise ValueError("runtime measurement reference identity is invalid")
    roles = row["role_tags"]
    allowed_conditions = {"dynamic_primary", "fixed_fim_ablation"}
    if (
        not isinstance(roles, list)
        or roles != sorted(set(roles))
        or not set(roles) <= allowed_conditions
        or (condition is not None and condition not in roles)
    ):
        raise ValueError("runtime measurement role tags are invalid")

    def finite_number(value):
        return type(value) in {int, float} and math.isfinite(float(value))

    position = row["reference_position"]
    covariance = row["reference_covariance"]
    if (
        not isinstance(position, list)
        or len(position) != 2
        or not all(finite_number(value) for value in position)
        or not isinstance(covariance, list)
        or len(covariance) != 2
        or any(
            not isinstance(matrix_row, list)
            or len(matrix_row) != 2
            or not all(finite_number(value) for value in matrix_row)
            for matrix_row in covariance
        )
        or covariance[0][1] != covariance[1][0]
        or covariance[0][0] < 0.0
        or covariance[1][1] < 0.0
        or covariance[0][0] * covariance[1][1]
            < covariance[0][1] * covariance[1][0]
    ):
        raise ValueError("runtime measurement reference geometry is invalid")
    provenance = row["base_anchor_provenance"]
    if (
        not isinstance(provenance, list)
        or provenance != sorted(set(provenance))
        or any(type(anchor) is not int or anchor not in {0, 1, 2} for anchor in provenance)
        or (key[0] == "base" and provenance != [key[1]])
        or (key[0] == "uav" and len(provenance) < 1)
    ):
        raise ValueError("runtime measurement provenance is invalid")
    if (
        not finite_number(row["noisy_range"])
        or row["noisy_range"] < 0.0
        or type(row["ranging_sigma"]) is not float
        or row["ranging_sigma"] != 0.5
        or row["config_sha256"] != manifest.get("config_sha256")
        or row["measurement_stream_id"] != manifest.get("measurement_stream_id")
    ):
        raise ValueError("runtime measurement stream identity is invalid")


def _iter_command_frames(path: Path):
    previous_frame = -1
    with gzip.open(path, "rt", encoding="utf-8", errors="strict") as source:
        for line in source:
            row = _strict_json_object(line.encode("utf-8"))
            if set(row) != {"frame_index", "commands"}:
                raise ValueError("held-command row schema is not exact")
            frame = row.get("frame_index")
            if type(frame) is not int or frame != previous_frame + 1:
                raise ValueError("held-command frames are not contiguous")
            by_robot = {}
            raw_commands = row.get("commands")
            if not isinstance(raw_commands, list):
                raise ValueError("held-command list is invalid")
            for command in raw_commands:
                if not isinstance(command, dict) or set(command) != {
                    "robot_id", "applied_command"
                }:
                    raise ValueError("held-command schema is not exact")
                robot_id = command.get("robot_id")
                applied = command.get("applied_command")
                if (
                    type(robot_id) is not int
                    or robot_id not in range(1, 15)
                    or robot_id in by_robot
                    or not isinstance(applied, list)
                    or len(applied) != 3
                    or any(
                        type(value) not in {int, float}
                        or not math.isfinite(float(value))
                        for value in applied
                    )
                ):
                    raise ValueError("duplicate held-command robot")
                by_robot[robot_id] = applied
            if set(by_robot) != set(range(1, 15)):
                raise ValueError("held-command robot universe is incomplete")
            yield frame, by_robot
            previous_frame = frame


def execute_campaign(arguments, operations) -> dict:
    """Execute a frozen campaign through a tested operations boundary."""
    registration = operations.validate_registration(arguments)
    identities = operations.collect_identities(arguments)
    schedule = _schedule_from_arguments(arguments)
    operations.schedule_frozen(schedule)

    output_root = claim_campaign_root(
        Path(arguments.output_root),
        Path.cwd(),
    )
    failure_reason = None
    completed = 0
    try:
        write_schedule_no_replace(output_root / "schedule.json", schedule)
        _publish_json_no_replace(
            output_root / "identities.json",
            {
                "schema_version": "cbf2026-qualified-identities-v1",
                "registration": registration,
                "identities": identities,
            },
        )
    except BaseException as error:
        failure_reason = _post_claim_failure_reason("campaign_setup", error)
        _terminalize_remaining_schedule(
            operations, schedule, 0, output_root, failure_reason
        )

    if failure_reason is None:
      for index, mission in enumerate(schedule):
        mission_stage = output_root / f".{mission['mission_id']}.{uuid.uuid4().hex}.tmp"
        mission_stage.mkdir(exist_ok=False)
        phase = "materialized_config"
        try:
            config_path = mission_stage / "materialized-primary.json"
            materializer = getattr(operations, "materialize_config", None)
            if materializer is None:
                _publish_json_no_replace(
                    config_path,
                    {
                        "schema_version": "cbf2026-qualified-materialized-config-v1",
                        "mission": mission,
                        "base_config": str(arguments.base_config),
                        "primary_config": str(arguments.primary_config),
                    },
                )
            else:
                materializer(arguments, mission, config_path)

            phase = "swarm"
            swarm = operations.run_swarm(mission, mission_stage, config_path)
            _require_terminal_operation(swarm, "Swarm")
            if swarm.get("status") != "completed":
                failure_reason = str(swarm.get("reason") or "swarm_failed")
                _synthesize_and_publish_failed_mission(
                    operations, mission, mission_stage, output_root,
                    failure_reason,
                )
                _terminalize_remaining_schedule(
                    operations, schedule, index + 1, output_root,
                    failure_reason,
                )
                break

            phase = "measurements"
            measurements = operations.generate_measurements(
                mission, mission_stage, swarm
            )
            _require_terminal_operation(measurements, "measurement generator")
            if measurements.get("status") != "completed":
                raise RuntimeError("measurement generator did not complete")

            replay_manifests = {}
            for condition in mission["conditions"]:
                phase = f"replay_{condition}"
                lifecycle_state = {"public": {}, "private": {}}
                replay = operations.run_replay(
                    mission,
                    mission_stage,
                    condition,
                    measurements,
                    lifecycle_state,
                )
                _require_terminal_operation(replay, f"{condition} replay")
                if replay.get("status") != "completed":
                    raise RuntimeError(f"{condition} replay did not complete")
                replay_manifests[condition] = _json_path_summary(replay, mission_stage)

            phase = "mission_publication"
            _publish_json_no_replace(
                mission_stage / "manifest.json",
                {
                    "schema_version": "cbf2026-qualified-mission-manifest-v1",
                    "terminal": True,
                    "status": "completed",
                    "mission": mission,
                    "swarm": _json_path_summary(swarm, mission_stage),
                    "measurements": _json_path_summary(measurements, mission_stage),
                    "replays": replay_manifests,
                    "member_identities": _directory_member_identities(mission_stage),
                },
            )
            _rename_directory_no_replace(
                mission_stage, output_root / mission["mission_id"]
            )
            completed += 1
        except BaseException as error:
            failure_reason = _post_claim_failure_reason(phase, error)
            retained_reason = _retry_exact_failed_stage_publication(
                mission, mission_stage, output_root
            )
            if retained_reason is not None:
                failure_reason = retained_reason
            else:
                _remove_owned_partial_synthetic(mission_stage)
                _synthesize_and_publish_failed_mission(
                    operations, mission, mission_stage, output_root,
                    failure_reason,
                )
            _terminalize_remaining_schedule(
                operations, schedule, index + 1, output_root,
                failure_reason,
            )
            break

    status = "failed" if failure_reason is not None else "completed"
    schedule_path = output_root / "schedule.json"
    mission_manifest_identities = {}
    for mission in schedule:
        mission_manifest_path = (
            output_root / mission["mission_id"] / "manifest.json"
        )
        mission_manifest_identities[mission["mission_id"]] = {
            "sha256": sha256_path(mission_manifest_path),
            "bytes": mission_manifest_path.stat().st_size,
        }
    manifest = {
        "schema_version": "cbf2026-qualified-campaign-manifest-v1",
        "terminal": True,
        "status": status,
        "reason": failure_reason or "completed",
        "mission_count": len(schedule),
        "completed_mission_count": completed,
        "protocol_sha256": registration.get("protocol_sha256"),
        "authorization_sha256": registration.get("authorization_sha256"),
        "schedule_sha256": (
            sha256_path(schedule_path) if schedule_path.is_file() else None
        ),
        "schedule_bytes": (
            schedule_path.stat().st_size if schedule_path.is_file() else None
        ),
        "mission_manifest_identities": mission_manifest_identities,
    }
    _publish_json_no_replace(output_root / "manifest.json", manifest)
    return manifest


def _post_claim_failure_reason(phase: str, error: BaseException) -> str:
    return f"{phase}_exception:{type(error).__name__}"


def _terminalize_remaining_schedule(
    operations, schedule, start_index: int, output_root: Path, reason: str
) -> None:
    for mission in schedule[start_index:]:
        mission_stage = output_root / (
            f".{mission['mission_id']}.{uuid.uuid4().hex}.tmp"
        )
        mission_stage.mkdir(exist_ok=False)
        try:
            _synthesize_and_publish_failed_mission(
                operations, mission, mission_stage, output_root, reason
            )
        except BaseException:
            if mission_stage.exists():
                shutil.rmtree(mission_stage)
            raise


def _schedule_from_arguments(arguments) -> list[dict]:
    trajectory = _parse_seed_expression(arguments.trajectory_seeds)
    noise = _parse_seed_expression(arguments.range_noise_seeds)
    if len(trajectory) != len(noise):
        raise ValueError("trajectory and range-noise seed counts differ")
    if arguments.kind == "development":
        expected_trajectory = list(range(2026080101, 2026080111))
        expected_noise = list(range(2026081101, 2026081111))
        if trajectory != expected_trajectory or noise != expected_noise:
            raise ValueError("development seed schedule is not the registered schedule")
        if arguments.frames != 1000 or arguments.version != "v1":
            raise ValueError("development run requires version v1 and 1000 frames")
        campaign_id = "development-v1"
    elif arguments.kind == "confirmatory":
        expected_trajectory = list(range(2026082001, 2026082061))
        expected_noise = list(range(2026083001, 2026083061))
        if trajectory != expected_trajectory or noise != expected_noise:
            raise ValueError("confirmatory seed schedule is not the registered schedule")
        if arguments.frames != 1000 or not arguments.version:
            raise ValueError("confirmatory run requires a version and 1000 frames")
        campaign_id = f"confirmatory-{arguments.version}"
    elif arguments.kind == "confirmatory-smoke":
        if trajectory != [2026089001] or noise != [2026089101]:
            raise ValueError("confirmatory-smoke seed schedule is not registered")
        if arguments.frames != 20 or arguments.smoke_id not in {"a", "b"}:
            raise ValueError("confirmatory-smoke requires smoke-id a/b and 20 frames")
        campaign_id = f"confirmatory-smoke-{arguments.smoke_id}"
    else:  # argparse rejects this, but callers may construct a Namespace.
        raise ValueError(f"unsupported campaign kind: {arguments.kind}")

    horizon_s = arguments.frames / 2.0
    return [
        {
            "campaign_id": campaign_id,
            "mission_id": f"mission-{position:02d}",
            "trajectory_seed": trajectory_seed,
            "range_noise_seed": range_noise_seed,
            "frames": arguments.frames,
            "horizon_s": horizon_s,
            "conditions": ["dynamic_primary", "fixed_fim_ablation"],
        }
        for position, (trajectory_seed, range_noise_seed) in enumerate(
            zip(trajectory, noise, strict=True), start=1
        )
    ]


def _parse_seed_expression(expression: str) -> list[int]:
    if re.fullmatch(r"[0-9]+:[0-9]+", expression):
        start_text, stop_text = expression.split(":", 1)
        start, stop = int(start_text), int(stop_text)
        if stop < start:
            raise ValueError("seed range must be increasing")
        return list(range(start, stop + 1))
    if not re.fullmatch(r"[0-9]+(?:,[0-9]+)*", expression):
        raise ValueError("seeds must be an inclusive A:B range or comma list")
    return [int(value) for value in expression.split(",")]


def _require_terminal_operation(result: dict, label: str) -> None:
    if not isinstance(result, dict) or result.get("terminal") is not True:
        raise RuntimeError(f"{label} did not return a terminal manifest")


def _json_path_summary(value, relative_to=None):
    if isinstance(value, Path):
        if relative_to is not None:
            try:
                return str(value.relative_to(relative_to))
            except ValueError:
                pass
        return str(value)
    if isinstance(value, dict):
        return {key: _json_path_summary(item, relative_to) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_path_summary(item, relative_to) for item in value]
    return value


def _synthesize_and_publish_failed_mission(
    operations, mission, stage: Path, root: Path, reason: str
) -> None:
    operations.synthesize_failed_mission(mission, stage, reason)
    _publish_or_replace_owned_stage_manifest(stage / "manifest.json", {
        "schema_version": "cbf2026-qualified-mission-manifest-v1",
        "terminal": True,
        "status": "failed",
        "reason": reason,
        "mission": mission,
        "member_identities": _directory_member_identities(stage),
    })
    _rename_directory_no_replace(stage, root / mission["mission_id"])


def _retry_exact_failed_stage_publication(
    mission: dict, stage: Path, root: Path
):
    """Retry only a fully committed failed bundle after its rename failed."""
    stage = Path(stage)
    manifest_path = stage / "manifest.json"
    if not manifest_path.exists() and not manifest_path.is_symlink():
        return None
    if (
        not stage.name.startswith(".")
        or manifest_path.is_symlink()
        or not manifest_path.is_file()
    ):
        raise ValueError("owned failed mission stage is not retryable")
    manifest = _read_json_object(
        manifest_path, "owned failed mission stage manifest"
    )
    if (
        set(manifest) != {
            "schema_version", "terminal", "status", "reason", "mission",
            "member_identities",
        }
        or manifest.get("schema_version")
            != "cbf2026-qualified-mission-manifest-v1"
        or manifest.get("terminal") is not True
        or manifest.get("status") != "failed"
        or not isinstance(manifest.get("reason"), str)
        or not manifest["reason"]
        or manifest.get("mission") != mission
        or manifest.get("member_identities")
            != _directory_member_identities(stage)
    ):
        return None
    _rename_directory_no_replace(stage, Path(root) / mission["mission_id"])
    return manifest["reason"]


def _remove_owned_partial_synthetic(stage: Path) -> None:
    """Remove only an uncommitted synthetic stream in our hidden stage."""
    stage = Path(stage)
    manifest_path = stage / "manifest.json"
    synthetic_path = stage / "synthetic-missing.jsonl.gz"
    if manifest_path.exists() or manifest_path.is_symlink():
        return
    if not synthetic_path.exists() and not synthetic_path.is_symlink():
        return
    if (
        not stage.name.startswith(".")
        or synthetic_path.is_symlink()
        or not synthetic_path.is_file()
    ):
        raise ValueError("owned partial synthetic stream is not removable")
    synthetic_path.unlink()
    directory_fd = os.open(stage, os.O_RDONLY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


def _publish_or_replace_owned_stage_manifest(path: Path, payload: dict) -> None:
    """Commit a failed manifest inside an exclusively owned hidden stage."""
    path = Path(path)
    if not path.exists() and not path.is_symlink():
        _publish_json_no_replace(path, payload)
        return
    if (
        path.is_symlink()
        or not path.is_file()
        or not path.parent.name.startswith(".")
    ):
        raise ValueError("owned mission stage manifest is not replaceable")
    existing = _read_json_object(path, "owned mission stage manifest")
    if (
        existing.get("schema_version")
            != "cbf2026-qualified-mission-manifest-v1"
        or existing.get("terminal") is not True
        or existing.get("status") != "completed"
        or existing.get("mission") != payload.get("mission")
    ):
        raise ValueError("owned mission stage manifest is not completed evidence")
    encoded = json.dumps(
        payload,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8") + b"\n"
    replacement = path.with_name(f".{path.name}.{uuid.uuid4().hex}.replacement")
    try:
        with replacement.open("xb") as output:
            output.write(encoded)
            output.flush()
            os.fsync(output.fileno())
        os.replace(replacement, path)
        parent_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(parent_fd)
        finally:
            os.close(parent_fd)
    finally:
        replacement.unlink(missing_ok=True)


def _rename_directory_no_replace(stage: Path, target: Path) -> None:
    stage = Path(stage)
    target = Path(target)
    directories = [stage]
    for path in stage.rglob("*"):
        if path.is_symlink():
            raise ValueError("publication stage must not contain symbolic links")
        if path.is_file():
            file_fd = os.open(path, os.O_RDONLY)
            try:
                os.fsync(file_fd)
            finally:
                os.close(file_fd)
        elif path.is_dir():
            directories.append(path)
    for directory in sorted(
        directories, key=lambda path: len(path.parts), reverse=True
    ):
        directory_fd = os.open(directory, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    libc = ctypes.CDLL(None, use_errno=True)
    source_bytes = os.fsencode(stage)
    target_bytes = os.fsencode(target)
    if sys.platform == "darwin" and hasattr(libc, "renamex_np"):
        result = libc.renamex_np(source_bytes, target_bytes, 0x4)
    elif sys.platform.startswith("linux") and hasattr(libc, "renameat2"):
        result = libc.renameat2(-100, source_bytes, -100, target_bytes, 1)
    else:
        raise RuntimeError("atomic no-replace directory publication is unavailable")
    if result != 0:
        error_number = ctypes.get_errno()
        if error_number in {17, 39}:
            raise FileExistsError(error_number, os.strerror(error_number), target)
        raise OSError(error_number, os.strerror(error_number), target)
    directory_fd = os.open(target.parent, os.O_RDONLY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


def _directory_member_identities(root: Path) -> dict:
    identities = {}
    for path in sorted(Path(root).rglob("*")):
        if path.is_symlink():
            raise ValueError("mission bundle must not contain symbolic links")
        if not path.is_file() or path.name == "manifest.json" and path.parent == root:
            continue
        relative = str(path.relative_to(root))
        identities[relative] = {
            "sha256": sha256_path(path),
            "bytes": path.stat().st_size,
        }
    return identities


if __name__ == "__main__":
    raise SystemExit(main())
