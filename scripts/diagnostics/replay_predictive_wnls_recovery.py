"""Exact, truth-boundary-only Stage-1 replay for predictive WNLS recovery."""

import argparse
import gzip
import hashlib
import json
import math
import os
import stat
import tempfile
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from scripts.diagnostics.predictive_wnls import (
    ATTEMPT_STATUSES,
    candidate_acceptance,
    finalize_attempt,
    qualify_active_references,
    reference_is_eligible,
    solve_finite_budget_wnls,
    solve_predictive_multistart,
)
from scripts.diagnostics.replay_localization_calibration import (
    _frames,
    _initial_positions,
    _strict_json_bytes,
    _strict_load,
    _truth_positions,
    active_references,
    fixed_references,
    stable_measurement_seed,
)
from scripts.diagnostics.run_diagnostic import (
    HARD_FLOOR_BYTES,
    START_BYTES,
    DiskSpaceError,
    _nearest_existing_ancestor,
    _sha256,
    _validate_output_root,
    allocated_bytes,
    available_bytes,
    require_start_space,
)


DEVELOPMENT_VARIANTS = (
    "prediction_expiry",
    "fresh_reference_qualification",
    "predictive_multistart",
)
RAW_SCHEMA_ID = "cbf2026-predictive-wnls-development-rows-v2"
RAW_PROCESS_NAME = "predictive-wnls-development.jsonl.gz"
TERMINAL_MANIFEST_NAME = "manifest.json"
RAW_BUNDLE_CAP_BYTES = 2_000_000_000

# These fields deliberately remain compact arrays in raw rows.  The protocol
# binds both orders, so a later reader cannot mistake an unordered dict for a
# numerical trace.
CANDIDATE_FIELDS = (
    "source", "initial_estimate", "status", "estimate", "covariance", "cost",
    "accepted", "rejection_reason", "q_innov", "gate_diagnostics", "proposal_trace",
)
PROPOSAL_TRACE_FIELDS = (
    "proposal", "damping", "cost", "stationarity_norm", "raw_step_norm",
    "trial_cost", "invalid_trial_reason", "accepted",
)


def _strict_regular(path: Path, label: str) -> None:
    try:
        metadata = path.lstat()
    except FileNotFoundError:
        raise ValueError(f"{label} does not exist: {path}") from None
    if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISREG(metadata.st_mode):
        raise ValueError(f"{label} must be a non-symlink regular file: {path}")


def _within(child: Path, parent: Path) -> bool:
    child, parent = child.resolve(), parent.resolve()
    return child == parent or parent in child.parents


def _safe_parent_and_exclusive_root(output_root: Path) -> None:
    """Create only absent parents, then the registered root exactly once."""
    if output_root.exists() or output_root.is_symlink():
        raise FileExistsError(f"registered output_root already exists: {output_root}")
    ancestor = _nearest_existing_ancestor(output_root)
    if ancestor.is_symlink() or not ancestor.is_dir():
        raise ValueError("output_root ancestor must be a non-symlink directory")
    pending = []
    cursor = output_root.parent
    while cursor != ancestor:
        pending.append(cursor)
        cursor = cursor.parent
    for directory in reversed(pending):
        try:
            directory.mkdir()
        except FileExistsError:
            pass
        if directory.is_symlink() or not directory.is_dir():
            raise ValueError(f"unsafe output parent: {directory}")
    output_root.mkdir(exist_ok=False)


def _declared_source(protocol: dict, *names: str) -> dict:
    sources = protocol.get("sources")
    if not isinstance(sources, dict):
        raise ValueError("protocol must declare a sources object")
    for name in names:
        entry = sources.get(name)
        if isinstance(entry, dict):
            return entry
    raise ValueError(f"protocol lacks required source: {names[0]}")


def _identity(entry: dict, actual: Path, label: str, *, require_path: bool = True) -> dict:
    declared_path = entry.get("path")
    declared_hash = entry.get("sha256")
    if not isinstance(declared_hash, str) or len(declared_hash) != 64:
        raise ValueError(f"protocol {label} hash is invalid")
    _strict_regular(actual, label)
    if require_path:
        if not isinstance(declared_path, str) or Path(declared_path).resolve() != actual.resolve():
            raise ValueError(f"protocol {label} path differs from runtime input")
    observed = _sha256(actual)
    if observed != declared_hash:
        raise ValueError(f"protocol {label} SHA-256 mismatch")
    return {"path": str(actual.resolve()), "sha256": observed}


def _verify_bound_identities(
    protocol: dict,
    data_path: Path,
    input_manifest_path: Path,
) -> dict:
    """Verify all source and runtime identities; callable before and after work."""
    root = Path(__file__).resolve().parents[2]
    replay_path = Path(__file__).resolve()
    estimator_path = root / "scripts" / "diagnostics" / "predictive_wnls.py"
    identities = {
        "truth_data": _identity(_declared_source(protocol, "truth_data", "data"), data_path, "truth_data"),
        "input_manifest": _identity(_declared_source(protocol, "input_manifest"), input_manifest_path, "input_manifest"),
        "replay_source": _identity(_declared_source(protocol, "replay_source"), replay_path, "replay_source"),
        "estimator_source": _identity(_declared_source(protocol, "estimator_source"), estimator_path, "estimator_source"),
    }
    sources = protocol.get("sources", {})
    if isinstance(sources, dict) and isinstance(sources.get("baseline_process"), dict):
        baseline = sources["baseline_process"]
        if not isinstance(baseline.get("path"), str):
            raise ValueError("protocol baseline_process path is invalid")
        identities["baseline_process"] = _identity(
            baseline, Path(baseline["path"]), "baseline_process"
        )
    return identities


def _protected_overlap(
    output_root: Path,
    project_root: Path,
    data_path: Path,
    input_manifest_path: Path,
    protocol: dict,
) -> None:
    _validate_output_root(project_root, output_root)
    protected = [project_root, data_path, data_path.parent, input_manifest_path, input_manifest_path.parent]
    sources = protocol.get("sources", {})
    baseline = sources.get("baseline_process") if isinstance(sources, dict) else None
    if isinstance(baseline, dict) and isinstance(baseline.get("path"), str):
        baseline_path = Path(baseline["path"])
        _strict_regular(baseline_path, "baseline_process")
        protected.extend((baseline_path, baseline_path.parent))
    target = output_root.resolve()
    for root in protected:
        # An input/output parent-child relation can overwrite or contaminate
        # evidence even when the exact leaf currently does not exist.
        resolved = root.resolve()
        if target == resolved or target in resolved.parents or resolved in target.parents:
            raise ValueError("output_root overlaps a protected project or evidence path")


def _sensor_records(
    config: dict,
    observer_id: int,
    truth: dict[int, np.ndarray],
    seed: int,
    frame_index: int,
    sigma: float,
) -> tuple[dict, list[tuple[str, int]], dict[tuple[str, int], dict]]:
    """The sole truth-facing boundary: return only present noisy scalar ranges."""
    mandatory = fixed_references(config, observer_id)
    legacy_active = active_references(config, observer_id, truth)
    mandatory_keys = {("base", item) for item in mandatory["base_ids"]} | {
        ("uav", item) for item in mandatory["uav_ids"]
    }
    optional = sorted(
        ({("base", item) for item in legacy_active["base_ids"]} | {("uav", item) for item in legacy_active["uav_ids"]}) - mandatory_keys,
        key=lambda item: (0 if item[0] == "base" else 1, item[1]),
    )
    records = {}
    for key in sorted(mandatory_keys | set(optional), key=lambda item: (0 if item[0] == "base" else 1, item[1])):
        kind, identifier = key
        reference = np.asarray(config["bases"][identifier], dtype=float) if kind == "base" else truth[identifier]
        noisy = float(np.linalg.norm(truth[observer_id] - reference) + np.random.default_rng(
            stable_measurement_seed(seed, frame_index, observer_id, kind, identifier)
        ).normal(0.0, sigma))
        records[key] = {"present": True, "noisy_range": noisy}
    return mandatory, optional, records


def _compact_candidates(candidates: object) -> list[list]:
    compact = []
    for candidate in candidates if isinstance(candidates, list) else []:
        if not isinstance(candidate, dict):
            continue
        result = candidate.get("result", {}) if isinstance(candidate.get("result"), dict) else {}
        trace = []
        for proposal in result.get("proposal_trace", []) if isinstance(result.get("proposal_trace"), list) else []:
            trace.append([proposal.get(field) for field in PROPOSAL_TRACE_FIELDS])
        compact.append([
            candidate.get("source"), candidate.get("initial_estimate"), candidate.get("status"),
            candidate.get("estimate"), candidate.get("covariance"), candidate.get("cost"),
            candidate.get("accepted"), candidate.get("rejection_reason"), candidate.get("q_innov"),
            candidate.get("gate_diagnostics"), trace,
        ])
    return compact


def _output_fields(output: dict) -> tuple[object, object, object, object, object]:
    if output.get("output_status") == "fresh":
        return output.get("estimate"), output.get("modeled_covariance"), output.get("epsilon"), None, None
    if output.get("output_status") == "predicted":
        return output.get("estimate"), None, None, output.get("modeled_covariance"), output.get("aged_modeled_radius")
    return None, None, None, None, None


def _q_error(error: np.ndarray, covariance: object) -> float | None:
    try:
        matrix = np.asarray(covariance, dtype=float)
        value = float(error @ np.linalg.solve(matrix, error))
    except (TypeError, ValueError, np.linalg.LinAlgError):
        return None
    return value if math.isfinite(value) else None


def _reference_arrays(config: dict, qualification: dict, current: dict[int, dict]) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[tuple[str, int]]] | None:
    positions, covariances, measurements, keys = [], [], [], []
    for record in qualification["active_records"]:
        key = record["key"]
        if key[0] == "base":
            position, covariance = np.asarray(config["bases"][key[1]], dtype=float), np.zeros((2, 2))
        else:
            output = current.get(key[1])
            if not isinstance(output, dict) or output.get("estimate") is None or output.get("modeled_covariance") is None:
                return None
            position, covariance = np.asarray(output["estimate"], dtype=float), np.asarray(output["modeled_covariance"], dtype=float)
        positions.append(position); covariances.append(covariance); measurements.append(record["noisy_range"]); keys.append(key)
    if len(keys) < 2:
        return None
    return np.asarray(positions), np.asarray(covariances), np.asarray(measurements), keys


def _write_row(compressed, line: bytes, digest) -> None:
    compressed.write(line)
    digest.update(line)


def _safe_probe(function, path: Path) -> int | None:
    try:
        return function(path)
    except BaseException:
        return None


def _single_start_attempt(
    *, positions: np.ndarray, covariances: np.ndarray, measurements: np.ndarray,
    initial: np.ndarray, source: str, live_prediction: dict | None,
    sigma: float, provenance: object,
) -> dict:
    """The ablations retain one deterministic start while applying validity gates."""
    solved = solve_finite_budget_wnls(positions, covariances, measurements, initial, sigma)
    accepted, reason, diagnostics = candidate_acceptance(
        solved,
        live_prediction=live_prediction,
        active_reference_count=len(positions),
        base_anchor_provenance=provenance,
    )
    candidate = {
        "source": source, "initial_estimate": initial.tolist(), "result": solved,
        "status": solved["status"], "estimate": solved.get("estimate"),
        "covariance": solved.get("covariance"), "cost": solved.get("cost"),
        "accepted": accepted, "rejection_reason": None if accepted else reason,
        "q_innov": diagnostics.get("q_innov"), "gate_diagnostics": diagnostics,
    }
    if accepted:
        return {"attempt_status": "accepted", "candidates": [candidate], "candidate": {
            "estimate": solved["estimate"], "modeled_covariance": solved["covariance"],
            "epsilon": solved["epsilon"], "base_anchor_provenance": list(provenance),
        }, "selected_candidate": candidate, "failure_reason": None}
    status = "failed" if solved["status"] == "failed" else "invalid" if solved["status"] == "invalid" else "rejected"
    return {"attempt_status": status, "candidates": [candidate], "candidate": None,
            "selected_candidate": None, "failure_reason": reason}


def _write_terminal(output_root: Path, manifest: dict, *, finalizing: bool) -> None:
    """Strict staging plus link-based no-replace publication of terminal state."""
    content = _strict_json_bytes(manifest, indent=2) + b"\n"
    staging = output_root / ".manifest.finalizing.json"
    staging.write_bytes(content)
    if finalizing:
        return
    target = output_root / TERMINAL_MANIFEST_NAME
    try:
        os.link(staging, target)
    except FileExistsError:
        raise RuntimeError("terminal manifest already exists") from None
    finally:
        if staging.exists():
            staging.unlink()


def replay_predictive_recovery(*, data_path: Path, input_manifest_path: Path, protocol_path: Path, output_root: Path, run_seeds: tuple[int, ...], max_frames: int | None = None) -> dict:
    """Create one exact, disk-guarded raw Stage-1 bundle."""
    data_path, input_manifest_path, protocol_path, output_root = map(Path, (data_path, input_manifest_path, protocol_path, output_root))
    _strict_regular(protocol_path, "protocol")
    protocol = _strict_load(protocol_path)
    identities = _verify_bound_identities(protocol, data_path, input_manifest_path)
    data, input_manifest = _strict_load(data_path), _strict_load(input_manifest_path)
    # Hash again after parsing so a time-of-check/time-of-read source swap can
    # never reach allocation, even when it leaves syntactically valid JSON.
    identities = _verify_bound_identities(protocol, data_path, input_manifest_path)
    project_root = Path(__file__).resolve().parents[2]
    _protected_overlap(output_root, project_root, data_path, input_manifest_path, protocol)
    if not run_seeds or any(isinstance(seed, bool) or not isinstance(seed, int) for seed in run_seeds) or len(set(run_seeds)) != len(run_seeds):
        raise ValueError("run_seeds must be unique non-empty integers")
    config = data.get("config")
    if not isinstance(config, dict):
        raise ValueError("truth replay must contain config")
    number = int(config.get("num", 0)); expected_ids = set(range(1, number + 1))
    if number <= 0:
        raise ValueError("config.num must be positive")
    frames = _frames(data)
    if max_frames is not None:
        if not isinstance(max_frames, int) or max_frames <= 0:
            raise ValueError("max_frames must be a positive integer")
        frames = frames[:max_frames]
    sigma = float(config.get("position_covariance", {}).get("ranging_sigma", 0.5))
    if not math.isfinite(sigma) or sigma <= 0:
        raise ValueError("ranging sigma must be finite and positive")
    deployment = _initial_positions(config, expected_ids)
    free_before = require_start_space(_nearest_existing_ancestor(output_root))
    _safe_parent_and_exclusive_root(output_root)
    started = datetime.now(timezone.utc).isoformat()
    process_path = output_root / RAW_PROCESS_NAME
    rows_written = 0
    decompressed = hashlib.sha256()
    error = None

    def manifest(status: str, exc: BaseException | None = None) -> dict:
        result = {
            "status": status, "schema_id": RAW_SCHEMA_ID, "process_name": RAW_PROCESS_NAME,
            "raw_schema": {"candidate_fields": list(CANDIDATE_FIELDS), "proposal_trace_fields": list(PROPOSAL_TRACE_FIELDS)},
            "started_at": started, "ended_at": datetime.now(timezone.utc).isoformat(),
            "rows_written": rows_written, "expected_rows": len(DEVELOPMENT_VARIANTS) * len(run_seeds) * len(frames) * number,
            "input_identities": identities, "input_manifest_termination_reason": input_manifest.get("termination_reason"),
            "free_bytes_before": free_before, "free_bytes_after": _safe_probe(available_bytes, output_root),
            "allocated_bytes": _safe_probe(allocated_bytes, output_root), "raw_bundle_cap_bytes": RAW_BUNDLE_CAP_BYTES,
            "compressed_process_sha256": _sha256(process_path) if process_path.exists() else None,
            "decompressed_process_sha256": decompressed.hexdigest() if process_path.exists() else None,
        }
        if exc is not None:
            result["error"] = {"type": type(exc).__name__, "message": str(exc)}
        return result

    try:
        if available_bytes(output_root) < HARD_FLOOR_BYTES:
            raise DiskSpaceError("available bytes below live floor")
        states = {}
        with process_path.open("xb") as raw, gzip.GzipFile(filename="", fileobj=raw, mode="wb", mtime=0) as compressed:
            for variant in DEVELOPMENT_VARIANTS:
                for seed in run_seeds:
                    state = states.setdefault((variant, seed), {})
                    for frame_index, frame in enumerate(frames):
                        truth = _truth_positions(frame, expected_ids)
                        current = {}
                        robots = {int(robot["id"]): robot for robot in frame["robots"]}
                        previous_frame_robots = None if frame_index == 0 else {int(robot["id"]): robot for robot in frames[frame_index - 1]["robots"]}
                        for robot_id in range(1, number + 1):
                            previous = state.get(robot_id, {})
                            if previous_frame_robots is None:
                                prior = {"public_prediction": None, "private_reacquisition_seed": None}
                                command, command_source = None, None
                            else:
                                result = previous_frame_robots[robot_id].get("opt", {}).get("result", {})
                                command = [result.get("vx"), result.get("vy")]
                                command_source = frame_index - 1
                                from scripts.diagnostics.predictive_wnls import propagate_estimator_prior
                                prior = propagate_estimator_prior(previous.get("output"), previous.get("private_seed"), command)
                            mandatory, optional, records = _sensor_records(config, robot_id, truth, seed, frame_index, sigma)
                            qualification = qualify_active_references(mandatory=mandatory, optional_keys=optional, measurement_records=records, uav_outputs=current, variant=variant)
                            refs = _reference_arrays(config, qualification, current) if qualification["status"] == "ok" else None
                            live = prior.get("public_prediction")
                            private = prior.get("private_reacquisition_seed")
                            if refs is None:
                                attempt = {"attempt_status": "reference_unavailable", "candidates": [], "candidate": None, "failure_reason": "reference_unavailable"}
                            else:
                                positions, covariances, measurements, keys = refs
                                if variant == "predictive_multistart":
                                    attempt = solve_predictive_multistart(reference_positions=positions, reference_covariances=covariances, measurements=measurements, reference_keys=keys, live_prediction=live if isinstance(live, dict) and live.get("output_status") == "predicted" else None, private_seed=private, ranging_sigma=sigma, base_anchor_provenance=qualification["base_anchor_provenance"])
                                else:
                                    initial = (np.asarray(live["estimate"], dtype=float) if isinstance(live, dict) and live.get("estimate") is not None else np.asarray(deployment[robot_id], dtype=float))
                                    source = "prediction" if isinstance(live, dict) and live.get("estimate") is not None else "deployment"
                                    attempt = _single_start_attempt(
                                        positions=positions, covariances=covariances,
                                        measurements=measurements, initial=initial,
                                        source=source,
                                        live_prediction=live if isinstance(live, dict) and live.get("output_status") == "predicted" else None,
                                        sigma=sigma,
                                        provenance=qualification["base_anchor_provenance"],
                                    )
                            output = finalize_attempt(attempt, prior, frame_index=frame_index)
                            estimate, fresh_cov, epsilon, aged_cov, aged_radius = _output_fields(output)
                            error_vector = None if estimate is None else truth[robot_id] - np.asarray(estimate, dtype=float)
                            error_norm = None if error_vector is None else float(np.linalg.norm(error_vector))
                            fresh_q = _q_error(error_vector, fresh_cov) if error_vector is not None and fresh_cov is not None else None
                            aged_q = _q_error(error_vector, aged_cov) if error_vector is not None and aged_cov is not None else None
                            row = {
                                "variant": variant, "seed": seed, "frame_index": frame_index, "robot_id": robot_id,
                                "squad_local_index": robot_id, "applied_command_source_frame": command_source, "applied_command": command,
                                "attempt_status": output.get("attempt_status", attempt.get("attempt_status", "invalid")), "attempt_failure_reason": attempt.get("failure_reason"),
                                "output_status": output["output_status"], "prediction_age": output.get("prediction_age"), "estimate": estimate,
                                "fresh_modeled_covariance": fresh_cov, "fresh_epsilon": epsilon, "aged_modeled_covariance": aged_cov, "aged_modeled_radius": aged_radius,
                                "private_reacquisition_seed": private, "base_anchor_provenance": output.get("base_anchor_provenance", []),
                                "mandatory_references": mandatory, "optional_candidates": [[kind, identifier] for kind, identifier in optional],
                                "active_references": [[item["key"][0], item["key"][1]] for item in qualification["active_records"]],
                                "reference_freshness": {str(robot): current[robot].get("output_status") for robot in sorted(current)},
                                "excluded_references": qualification["excluded"], "reference_violations": qualification["violations"],
                                "candidates": _compact_candidates(attempt.get("candidates")), "selected_candidate_source": (attempt.get("selected_candidate") or {}).get("source"),
                                "offline_truth_position": truth[robot_id].tolist(), "offline_error_norm": error_norm,
                                "offline_fresh_containment": None if output["output_status"] != "fresh" else bool(error_norm <= epsilon),
                                "offline_aged_radius_containment": None if output["output_status"] != "predicted" else bool(error_norm <= aged_radius),
                                "offline_fresh_q_error": fresh_q if output["output_status"] == "fresh" else None,
                                "offline_aged_q_error": aged_q if output["output_status"] == "predicted" else None,
                            }
                            line = _strict_json_bytes(row) + b"\n"
                            _write_row(compressed, line, decompressed)
                            rows_written += 1
                            current[robot_id] = output
                            state[robot_id] = {"output": output, "private_seed": private}
                        compressed.flush()
                        if available_bytes(output_root) < HARD_FLOOR_BYTES:
                            raise DiskSpaceError("available bytes below live floor")
                        if allocated_bytes(output_root) > RAW_BUNDLE_CAP_BYTES:
                            raise DiskSpaceError("raw bundle exceeds allocated-byte cap")
        if allocated_bytes(output_root) > RAW_BUNDLE_CAP_BYTES:
            raise DiskSpaceError("raw bundle exceeds allocated-byte cap")
        _write_terminal(output_root, manifest("finalizing"), finalizing=True)
        identities = _verify_bound_identities(protocol, data_path, input_manifest_path)
        if available_bytes(output_root) < HARD_FLOOR_BYTES or allocated_bytes(output_root) > RAW_BUNDLE_CAP_BYTES:
            raise DiskSpaceError("final disk guard failed")
        completed = manifest("completed")
        _write_terminal(output_root, completed, finalizing=False)
        return completed
    except BaseException as caught:
        error = caught
        try:
            _write_terminal(output_root, manifest("failed", caught), finalizing=False)
        except BaseException:
            pass
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-path", type=Path, required=True)
    parser.add_argument("--input-manifest-path", type=Path, required=True)
    parser.add_argument("--protocol-json", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--run-seeds", required=True)
    parser.add_argument("--max-frames", type=int)
    args = parser.parse_args(argv)
    seeds = tuple(int(value) for value in args.run_seeds.split(",") if value)
    replay_predictive_recovery(data_path=args.data_path, input_manifest_path=args.input_manifest_path, protocol_path=args.protocol_json, output_root=args.output_root, run_seeds=seeds, max_frames=args.max_frames)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
