"""Create and execute immutable Stage 0 predictive-WNLS mechanism fixtures."""

import argparse
import gzip
import hashlib
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.diagnostics.predictive_wnls import (
    finalize_attempt,
    make_unavailable_output,
    propagate_estimator_prior,
    qualify_active_references,
    solve_predictive_multistart,
)
from scripts.diagnostics.replay_localization_calibration import (
    _frames,
    _initial_positions,
    _parse_ranging_sigma,
    _strict_load,
    _truth_positions,
    active_references,
    fixed_references,
    stable_measurement_seed,
)


STAGE0_SCHEMA_ID = "cbf2026-predictive-wnls-stage0-fixtures-v2"
FROZEN_SOURCE_REGISTRY = {
    "truth_data": {
        "path": Path(
            "/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/"
            "20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/"
            "2026-07-28_14-27-53_R_seed_20260727_250s/data.json"
        ),
        "sha256": "3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527",
    },
    "input_manifest": {
        "path": Path(
            "/private/tmp/cbf2026-results/mc-first-mechanism-250s/R/"
            "20260728T062752.357599Z_9cb3d4b121eb438c8688f0a121f01725/"
            "manifest.json"
        ),
        "sha256": "6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb",
    },
    "baseline_process": {
        "path": Path(
            "/private/tmp/cbf2026-warm-start-recovery/warm-start-recovery/"
            "20260729T112153.797813Z_54338549bff348a093dd71f29366d29b/"
            "restart/localization-calibration/"
            "20260729T112437.413270Z_efb8e7137776415c833f11204cf519d5/"
            "calibration.jsonl.gz"
        ),
        "sha256": "c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003",
    },
    "legacy_replay": {
        "path": Path(
            "/private/tmp/cbf2026-diagnostic/scripts/diagnostics/"
            "replay_localization_calibration.py"
        ),
        "sha256": "0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8",
    },
}
LEGACY_REPLAY_SHA256 = FROZEN_SOURCE_REGISTRY["legacy_replay"]["sha256"]
HISTORICAL_CASES = (
    ("frame44_recovery", 20260736, 44, 14),
    ("frame177_cascade", 20260730, 177, 14),
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _strict_json_bytes(value: object) -> bytes:
    return json.dumps(
        value,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _write_json_exclusive(path: Path, value: object) -> str:
    payload = _strict_json_bytes(value)
    with path.open("xb") as destination:
        destination.write(payload)
        destination.write(b"\n")
    return hashlib.sha256(payload + b"\n").hexdigest()


def _registered_source_identity(name: str, supplied_path: Path) -> dict:
    entry = FROZEN_SOURCE_REGISTRY.get(name)
    if not isinstance(entry, dict):
        raise ValueError(f"unknown frozen source: {name}")
    registered_path = entry.get("path")
    expected_hash = entry.get("sha256")
    if not isinstance(registered_path, Path) or not registered_path.is_absolute():
        raise ValueError(f"invalid frozen path registry entry: {name}")
    if not isinstance(expected_hash, str) or len(expected_hash) != 64:
        raise ValueError(f"invalid frozen hash registry entry: {name}")
    supplied = Path(supplied_path)
    if not supplied.is_absolute() or supplied != registered_path:
        raise ValueError(f"source path is not the exact registered spelling: {name}")
    try:
        resolved = supplied.resolve(strict=True)
        metadata = os.stat(supplied, follow_symlinks=False)
    except OSError as error:
        raise ValueError(f"registered source is unavailable: {name}") from error
    if resolved != registered_path:
        raise ValueError(f"registered source path contains an alias or symlink: {name}")
    return {
        "path": str(registered_path),
        "sha256": expected_hash,
        "_path_state": (
            metadata.st_dev,
            metadata.st_ino,
            metadata.st_mode,
            metadata.st_size,
            metadata.st_mtime_ns,
        ),
    }


def _verified_before(name: str, path: Path) -> dict:
    identity = _registered_source_identity(name, path)
    digest = _sha256(path)
    if digest != identity["sha256"]:
        raise ValueError(f"source SHA-256 mismatch before read: {path}")
    return identity


def _verify_after(name: str, path: Path, before: dict) -> dict:
    after = _registered_source_identity(name, path)
    digest = _sha256(path)
    if after != before or digest != before["sha256"]:
        raise ValueError(f"source changed while read: {path}")
    return {"path": before["path"], "sha256": before["sha256"]}


def _read_registered_json(name: str, path: Path) -> tuple[dict, dict]:
    before = _verified_before(name, path)
    value = _strict_load(path)
    identity = _verify_after(name, path, before)
    return value, identity


def _read_registered_baseline(name: str, path: Path) -> tuple[list[dict], dict]:
    before = _verified_before(name, path)
    rows = []
    with gzip.open(path, "rt", encoding="utf-8") as source:
        for line in source:
            if line.strip():
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError("baseline process row must be an object")
                rows.append(row)
    identity = _verify_after(name, path, before)
    return rows, identity


def _validated_predecessor_commands(
    frame: dict,
    expected_ids: set[int],
) -> dict[int, list[float]]:
    if not isinstance(frame, dict) or not isinstance(frame.get("robots"), list):
        raise ValueError("predecessor frame must contain a robots list")
    commands = {}
    for robot in frame["robots"]:
        if not isinstance(robot, dict):
            raise ValueError("predecessor robot records must be objects")
        identifier = robot.get("id")
        if (
            isinstance(identifier, bool)
            or not isinstance(identifier, int)
            or identifier in commands
        ):
            raise ValueError("predecessor UAV IDs must be unique integers")
        try:
            opt = robot["opt"]
            if opt["status"] != "success":
                raise ValueError("predecessor optimizer status is not success")
            result = opt["result"]
            if isinstance(result["vx"], bool) or isinstance(result["vy"], bool):
                raise ValueError("predecessor command components cannot be booleans")
            velocity = [float(result["vx"]), float(result["vy"])]
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            raise ValueError(
                "every predecessor must provide successful finite opt.result.vx/vy"
            ) from error
        if not np.isfinite(np.asarray(velocity, dtype=float)).all():
            raise ValueError("predecessor command must be finite")
        commands[identifier] = velocity
    if set(commands) != set(expected_ids):
        raise ValueError("predecessor frame UAV IDs differ from configured IDs")
    return {identifier: commands[identifier] for identifier in sorted(commands)}


def _sensor_record(
    *,
    truth: dict[int, np.ndarray],
    config: dict,
    seed: int,
    frame_index: int,
    observer_id: int,
    kind: str,
    reference_id: int,
    ranging_sigma: float,
) -> dict:
    reference = (
        np.asarray(config["bases"][reference_id], dtype=float)
        if kind == "base"
        else truth[reference_id]
    )
    true_range = float(np.linalg.norm(truth[observer_id] - reference))
    noise = float(np.random.default_rng(stable_measurement_seed(
        seed, frame_index, observer_id, kind, reference_id
    )).normal(0.0, ranging_sigma))
    # This is the sensor boundary: only presence and the noisy scalar cross it.
    return {
        "kind": kind,
        "id": int(reference_id),
        "present": True,
        "noisy_range": true_range + noise,
    }


def _runtime_prefix(data: dict, *, seed: int, target_frame: int) -> tuple[dict, dict]:
    config = data["config"]
    number = int(config["num"])
    expected_ids = set(range(1, number + 1))
    frames = _frames(data)
    if target_frame >= len(frames):
        raise ValueError("target frame is outside the immutable truth source")
    ranging_sigma = _parse_ranging_sigma(config)
    runtime_frames = []
    offline_truth = []
    for frame_index, source_frame in enumerate(frames[: target_frame + 1]):
        truth = _truth_positions(source_frame, expected_ids)
        predecessor_commands = (
            None
            if frame_index == 0
            else _validated_predecessor_commands(
                frames[frame_index - 1],
                expected_ids,
            )
        )
        entries = []
        for robot_id in range(1, number + 1):
            fixed = fixed_references(config, robot_id)
            active = active_references(config, robot_id, truth)
            fixed_keys = {
                *(('base', identifier) for identifier in fixed['base_ids']),
                *(('uav', identifier) for identifier in fixed['uav_ids']),
            }
            active_keys = [
                (kind, identifier)
                for kind, identifiers in (("base", active["base_ids"]), ("uav", active["uav_ids"]))
                for identifier in identifiers
            ]
            records = [
                _sensor_record(
                    truth=truth,
                    config=config,
                    seed=seed,
                    frame_index=frame_index,
                    observer_id=robot_id,
                    kind=kind,
                    reference_id=identifier,
                    ranging_sigma=ranging_sigma,
                )
                for kind, identifier in active_keys
            ]
            entries.append({
                "robot_id": robot_id,
                "transition_velocity": (
                    [0.0, 0.0]
                    if frame_index == 0
                    else predecessor_commands[robot_id]
                ),
                "mandatory": fixed,
                "optional_keys": [
                    [kind, identifier]
                    for kind, identifier in active_keys
                    if (kind, identifier) not in fixed_keys
                ],
                "measurements": records,
            })
        runtime_frames.append({"frame_index": frame_index, "uavs": entries})
        offline_truth.append({
            "frame_index": frame_index,
            "positions": {str(identifier): truth[identifier].tolist() for identifier in sorted(truth)},
        })
    runtime = {
        "kind": "historical_prefix",
        "seed": seed,
        "target": {"frame_index": target_frame, "robot_id": 14},
        "config": config,
        "frames": runtime_frames,
    }
    offline = {"truth_positions": offline_truth}
    return runtime, offline


def _comparison_row(rows: list[dict], *, seed: int, frame_index: int, robot_id: int) -> dict | None:
    selected = [
        row for row in rows
        if row.get("graph_case") == "dynamic_dag_wnls"
        and row.get("seed") == seed
        and row.get("frame_index") == frame_index
        and row.get("robot_id") == robot_id
    ]
    if len(selected) != 1:
        raise ValueError("baseline target comparison row is not unique")
    row = selected[0]
    output_fields = (
        "status",
        "estimate",
        "covariance",
        "epsilon",
        "error_norm",
        "finite",
        "failure_reason",
    )
    attempt_fields = (
        "status",
        "estimate",
        "covariance",
        "epsilon",
        "iterations",
        "cost",
        "stationarity_norm",
        "phi_min_eigenvalue",
        "phi_condition",
        "failure_reason",
    )
    attempt_source = (
        row["failure"]
        if row.get("status") == "stale"
        and isinstance(row.get("failure"), dict)
        else row
    )
    attempt = {
        field: attempt_source.get(field)
        for field in attempt_fields
    }
    if attempt_source is row:
        attempt["status"] = row.get("attempt_status", row.get("status"))
    return {
        "baseline_output": {
            field: row.get(field)
            for field in output_fields
        },
        "baseline_attempt": attempt,
    }


def _reacquisition_fixture() -> dict:
    target = [3.0, 4.0]
    bases = [[0.0, 0.0], [10.0, 0.0], [0.0, 10.0]]
    ranges = [math.dist(target, base) for base in bases]
    return {
        "schema_id": STAGE0_SCHEMA_ID,
        "fixture": "reacquisition",
        "runtime": {
            "kind": "synthetic_reacquisition",
            "config": {"num": 1, "bases": bases, "ranging_sigma": 0.5},
            "initial_public_output": {
                "output_status": "predicted",
                "estimate": target,
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
                "epsilon": None,
                "prediction_age": 2,
                "aged_modeled_radius": 3.0,
                "base_anchor_provenance": [],
            },
            "initial_private_seed": {
                "estimate": target,
                "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
            },
            "frames": [
                {
                    "frame_index": 1,
                    "transition_velocity": [0.0, 0.0],
                    "mandatory": {"base_ids": [0, 1], "uav_ids": []},
                    "optional_keys": [],
                    "measurements": [
                        {"kind": "base", "id": 0, "present": True, "noisy_range": ranges[0]},
                        {"kind": "base", "id": 1, "present": True, "noisy_range": ranges[1]},
                    ],
                },
                {
                    "frame_index": 2,
                    "transition_velocity": [0.0, 0.0],
                    "mandatory": {"base_ids": [0, 1, 2], "uav_ids": []},
                    "optional_keys": [],
                    "measurements": [
                        {"kind": "base", "id": 0, "present": True, "noisy_range": ranges[0]},
                        {"kind": "base", "id": 1, "present": True, "noisy_range": ranges[1]},
                        {"kind": "base", "id": 2, "present": True, "noisy_range": ranges[2]},
                    ],
                },
            ],
        },
        "offline": {"truth_positions": [{"frame_index": 1, "positions": {"1": target}}, {"frame_index": 2, "positions": {"1": target}}]},
    }


def extract_stage0_fixtures(
    *,
    truth_data_path: Path,
    input_manifest_path: Path,
    baseline_process_path: Path,
    output_dir: Path,
) -> dict:
    """Write deterministic prefix-replay runtime/offline fixture subtrees."""
    truth_data_path = Path(truth_data_path)
    input_manifest_path = Path(input_manifest_path)
    baseline_process_path = Path(baseline_process_path)
    output_dir = Path(output_dir)
    if output_dir.exists():
        raise FileExistsError(f"fixture output directory already exists: {output_dir}")
    legacy_path = FROZEN_SOURCE_REGISTRY["legacy_replay"]["path"]
    legacy_before = _verified_before("legacy_replay", legacy_path)
    data, truth_identity = _read_registered_json(
        "truth_data",
        truth_data_path,
    )
    _, input_manifest_identity = _read_registered_json(
        "input_manifest",
        input_manifest_path,
    )
    baseline_rows, baseline_identity = _read_registered_baseline(
        "baseline_process",
        baseline_process_path,
    )

    output_dir.mkdir(parents=True)
    fixture_hashes = {}
    try:
        for name, seed, frame_index, robot_id in HISTORICAL_CASES:
            runtime, offline = _runtime_prefix(data, seed=seed, target_frame=frame_index)
            offline.update(
                _comparison_row(
                    baseline_rows,
                    seed=seed,
                    frame_index=frame_index,
                    robot_id=robot_id,
                )
            )
            fixture_hashes[f"{name}.json"] = _write_json_exclusive(output_dir / f"{name}.json", {
                "schema_id": STAGE0_SCHEMA_ID,
                "fixture": name,
                "runtime": runtime,
                "offline": offline,
            })
        fixture_hashes["reacquisition.json"] = _write_json_exclusive(
            output_dir / "reacquisition.json", _reacquisition_fixture()
        )
        legacy_identity = _verify_after(
            "legacy_replay",
            legacy_path,
            legacy_before,
        )
        source_identities = {
            "truth_data": truth_identity,
            "input_manifest": input_manifest_identity,
            "baseline_process": baseline_identity,
            "legacy_replay": legacy_identity,
        }
        manifest = {
            "schema_id": STAGE0_SCHEMA_ID,
            "legacy_replay_sha256": legacy_identity["sha256"],
            "sources": source_identities,
            "fixtures": fixture_hashes,
        }
        manifest_hash = _write_json_exclusive(output_dir / "manifest.json", manifest)
    except Exception:
        raise
    return {"manifest_sha256": manifest_hash, **manifest}


def _measurement_mapping(entries: list[dict]) -> dict[tuple[str, int], dict]:
    return {
        (entry["kind"], int(entry["id"])): {
            "present": entry.get("present"),
            "noisy_range": entry.get("noisy_range"),
        }
        for entry in entries
    }


def _offline_position(offline: dict, frame_index: int, robot_id: int) -> np.ndarray | None:
    for frame in offline.get("truth_positions", []):
        if frame.get("frame_index") == frame_index:
            value = frame.get("positions", {}).get(str(robot_id))
            try:
                position = np.asarray(value, dtype=float)
            except (TypeError, ValueError):
                return None
            return position if position.shape == (2,) and np.isfinite(position).all() else None
    return None


def _result_row(
    *,
    frame_index: int,
    robot_id: int,
    output: dict,
    attempt: dict,
    qualification: dict,
    uav_outputs: dict[int, dict],
) -> dict:
    references = []
    active = set(qualification.get("active_keys", []))
    for record in qualification.get("active_records", []):
        kind, identifier = record["key"]
        references.append({
            "kind": kind,
            "id": identifier,
            "used": (kind, identifier) in active,
            "output_status": (
                "fresh" if kind == "base"
                else uav_outputs.get(identifier, {}).get("output_status")
            ),
        })
    for excluded in qualification.get("excluded", []):
        kind, identifier = excluded["key"]
        references.append({
            "kind": kind,
            "id": identifier,
            "used": False,
            "output_status": (
                "fresh" if kind == "base"
                else uav_outputs.get(identifier, {}).get("output_status")
            ),
            "reason": excluded["reason"],
        })
    selected = attempt.get("selected_candidate") if isinstance(attempt, dict) else None
    return {
        "frame_index": frame_index,
        "robot_id": robot_id,
        "attempt_status": output.get("attempt_status"),
        "output_status": output.get("output_status"),
        "estimate": output.get("estimate"),
        "prediction_age": output.get("prediction_age"),
        "selected_candidate_source": None if selected is None else selected.get("source"),
        "candidate_diagnostics": attempt.get("candidates", []),
        "reference_freshness": references,
    }


def _run_historical(runtime: dict) -> list[dict]:
    config = runtime["config"]
    number = int(config["num"])
    ranging_sigma = _parse_ranging_sigma(config)
    initial = _initial_positions(config, set(range(1, number + 1)))
    outputs = {robot_id: make_unavailable_output("before_frame_zero") for robot_id in range(1, number + 1)}
    private = {robot_id: None for robot_id in range(1, number + 1)}
    rows = []
    for frame in runtime["frames"]:
        frame_index = int(frame["frame_index"])
        entries = frame["uavs"]
        if [entry["robot_id"] for entry in entries] != list(range(1, number + 1)):
            raise ValueError("fixture must process all UAVs in ascending global ID")
        current = {}
        current_private = {}
        for entry in entries:
            robot_id = int(entry["robot_id"])
            if frame_index == 0:
                prior = {
                    "public_prediction": make_unavailable_output("initial_measurement"),
                    "private_reacquisition_seed": {
                        "estimate": initial[robot_id].tolist(),
                        "modeled_covariance": [[1.0, 0.0], [0.0, 1.0]],
                    },
                }
            else:
                prior = propagate_estimator_prior(
                    outputs[robot_id], private[robot_id], entry["transition_velocity"]
                )
            records = _measurement_mapping(entry["measurements"])
            optional = [tuple(item) for item in entry["optional_keys"]]
            qualification = qualify_active_references(
                mandatory=entry["mandatory"],
                optional_keys=optional,
                measurement_records=records,
                uav_outputs=current,
                variant="predictive_multistart",
            )
            if qualification["status"] == "ok":
                positions, covariances, ranges = [], [], []
                for record in qualification["active_records"]:
                    kind, identifier = record["key"]
                    if kind == "base":
                        positions.append(config["bases"][identifier])
                        covariances.append([[0.0, 0.0], [0.0, 0.0]])
                    else:
                        anchor = current[identifier]
                        positions.append(anchor["estimate"])
                        covariances.append(anchor["modeled_covariance"])
                    ranges.append(record["noisy_range"])
                attempt = solve_predictive_multistart(
                    reference_positions=positions,
                    reference_covariances=covariances,
                    measurements=ranges,
                    reference_keys=qualification["active_keys"],
                    live_prediction=(prior["public_prediction"] if prior["public_prediction"]["output_status"] == "predicted" else None),
                    private_seed=prior["private_reacquisition_seed"],
                    ranging_sigma=ranging_sigma,
                    base_anchor_provenance=qualification["base_anchor_provenance"],
                )
            else:
                attempt = {"attempt_status": qualification["status"], "candidates": [], "selected_candidate": None}
            output = finalize_attempt(attempt, prior, frame_index=frame_index)
            current[robot_id] = output
            current_private[robot_id] = (
                {"estimate": output["estimate"], "modeled_covariance": output["modeled_covariance"]}
                if output["output_status"] == "fresh"
                else prior["private_reacquisition_seed"]
            )
            rows.append(_result_row(
                frame_index=frame_index, robot_id=robot_id, output=output,
                attempt=attempt, qualification=qualification,
                uav_outputs=current,
            ))
        outputs, private = current, current_private
    return rows


def _run_reacquisition(runtime: dict) -> list[dict]:
    output = runtime["initial_public_output"]
    private = runtime["initial_private_seed"]
    rows = []
    for frame in runtime["frames"]:
        prior = propagate_estimator_prior(output, private, frame["transition_velocity"])
        records = _measurement_mapping(frame["measurements"])
        qualification = qualify_active_references(
            mandatory=frame["mandatory"], optional_keys=[], measurement_records=records,
            uav_outputs={}, variant="predictive_multistart",
        )
        if qualification["status"] == "ok":
            positions = [runtime["config"]["bases"][record["key"][1]] for record in qualification["active_records"]]
            ranges = [record["noisy_range"] for record in qualification["active_records"]]
            attempt = solve_predictive_multistart(
                reference_positions=positions,
                reference_covariances=[[[0.0, 0.0], [0.0, 0.0]] for _ in positions],
                measurements=ranges,
                reference_keys=qualification["active_keys"],
                live_prediction=(prior["public_prediction"] if prior["public_prediction"]["output_status"] == "predicted" else None),
                private_seed=prior["private_reacquisition_seed"],
                ranging_sigma=float(runtime["config"]["ranging_sigma"]),
                base_anchor_provenance=qualification["base_anchor_provenance"],
            )
        else:
            attempt = {"attempt_status": qualification["status"], "candidates": [], "selected_candidate": None}
        output = finalize_attempt(attempt, prior, frame_index=int(frame["frame_index"]))
        private = (
            {"estimate": output["estimate"], "modeled_covariance": output["modeled_covariance"]}
            if output["output_status"] == "fresh" else prior["private_reacquisition_seed"]
        )
        rows.append(_result_row(
            frame_index=int(frame["frame_index"]), robot_id=1, output=output,
            attempt=attempt, qualification=qualification,
            uav_outputs={},
        ))
    return rows


def _annotate_offline_rows(rows: list[dict], offline: dict) -> list[dict]:
    annotated = []
    for runtime_row in rows:
        row = dict(runtime_row)
        estimate = row.get("estimate")
        truth = _offline_position(
            offline,
            row["frame_index"],
            row["robot_id"],
        )
        error = None
        if estimate is not None and truth is not None:
            point = np.asarray(estimate, dtype=float)
            if point.shape == (2,) and np.isfinite(point).all():
                error = float(np.linalg.norm(point - truth))
        row["offline_error_norm"] = error
        annotated.append(row)
    return annotated


def run_stage0_fixture(path: Path) -> dict:
    """Run the online estimator first, then attach offline fixture evaluation."""
    fixture = _strict_load(Path(path))
    if fixture.get("schema_id") != STAGE0_SCHEMA_ID:
        raise ValueError("unrecognized Stage 0 fixture schema")
    runtime = fixture.get("runtime")
    offline = fixture.get("offline")
    if not isinstance(runtime, dict) or not isinstance(offline, dict):
        raise ValueError("fixture requires separate runtime and offline objects")
    kind = runtime.get("kind")
    try:
        if kind == "historical_prefix":
            runtime_rows = _run_historical(runtime)
            target = runtime["target"]
        elif kind == "synthetic_reacquisition":
            runtime_rows = _run_reacquisition(runtime)
            target = {
                "frame_index": runtime_rows[-1]["frame_index"],
                "robot_id": runtime_rows[-1]["robot_id"],
            }
        else:
            raise ValueError("unknown runtime fixture kind")
    except (KeyError, TypeError, IndexError) as error:
        label = (
            "historical"
            if kind == "historical_prefix"
            else "reacquisition"
        )
        raise ValueError(f"malformed {label} runtime") from error
    rows = _annotate_offline_rows(runtime_rows, offline)
    selected = next(
        row for row in rows
        if row["frame_index"] == target["frame_index"]
        and row["robot_id"] == target["robot_id"]
    )
    return {**selected, "rows": rows, "offline": fixture.get("offline", {})}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--truth-data-path", type=Path, required=True)
    parser.add_argument("--input-manifest-path", type=Path, required=True)
    parser.add_argument("--baseline-process-path", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    arguments = parser.parse_args(argv)
    extract_stage0_fixtures(
        truth_data_path=arguments.truth_data_path,
        input_manifest_path=arguments.input_manifest_path,
        baseline_process_path=arguments.baseline_process_path,
        output_dir=arguments.output_dir,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
