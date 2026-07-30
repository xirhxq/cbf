"""Create and execute immutable Stage 0 predictive-WNLS mechanism fixtures."""

import argparse
import gzip
import hashlib
import json
import math
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
LEGACY_REPLAY_SHA256 = "0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8"
FROZEN_INPUT_SHA256 = {
    "truth_data": "3defc62d11bb5996301b21b95ab3902c998bb982640b0f9be7b9536005145527",
    "input_manifest": "6731444b7a4cdaaaba010a6297b8b258500978e8856f239c04c157bc4878d6fb",
    "baseline_process": "c964d415e68425e4b64e5acc9be925a237165a72c59a0b98342b9033734f2003",
}
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


def _read_json_verified(path: Path, expected_hash: str | None) -> tuple[dict, str]:
    before = _sha256(path)
    if expected_hash is not None and before != expected_hash:
        raise ValueError(f"source SHA-256 mismatch before read: {path}")
    value = _strict_load(path)
    after = _sha256(path)
    if after != before:
        raise ValueError(f"source changed while read: {path}")
    return value, before


def _read_baseline_verified(path: Path, expected_hash: str | None) -> tuple[list[dict], str]:
    before = _sha256(path)
    if expected_hash is not None and before != expected_hash:
        raise ValueError(f"source SHA-256 mismatch before read: {path}")
    rows = []
    with gzip.open(path, "rt", encoding="utf-8") as source:
        for line in source:
            if line.strip():
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError("baseline process row must be an object")
                rows.append(row)
    after = _sha256(path)
    if after != before:
        raise ValueError(f"source changed while read: {path}")
    return rows, before


def _expected_hash(path: Path, frozen_name: str) -> str | None:
    """Enforce identities for the exact registered inputs, permit tiny test data."""
    known = {
        "truth_data": "data.json",
        "input_manifest": "manifest.json",
        "baseline_process": "calibration.jsonl.gz",
    }
    return FROZEN_INPUT_SHA256[frozen_name] if path.name == known[frozen_name] and "/private/tmp/" in str(path) else None


def _velocity(frame: dict, robot_id: int) -> list[float]:
    robot = next((item for item in frame["robots"] if int(item["id"]) == robot_id), None)
    if robot is None:
        raise ValueError(f"missing robot {robot_id} for applied command")
    try:
        result = robot["opt"]["result"]
        velocity = [float(result["vx"]), float(result["vy"])]
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        raise ValueError("every transition must provide finite opt.result.vx/vy") from error
    if not np.isfinite(np.asarray(velocity, dtype=float)).all():
        raise ValueError("applied command must be finite")
    return velocity


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
                    else _velocity(frames[frame_index - 1], robot_id)
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
    return {
        "status": row.get("status"),
        "attempt_status": row.get("attempt_status"),
        "error_norm": row.get("error_norm"),
        "failure_reason": row.get("failure_reason"),
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
    data, truth_hash = _read_json_verified(
        truth_data_path, _expected_hash(truth_data_path, "truth_data")
    )
    _, manifest_hash = _read_json_verified(
        input_manifest_path, _expected_hash(input_manifest_path, "input_manifest")
    )
    baseline_rows, baseline_hash = _read_baseline_verified(
        baseline_process_path, _expected_hash(baseline_process_path, "baseline_process")
    )
    legacy_path = Path(__file__).with_name("replay_localization_calibration.py")
    if _sha256(legacy_path) != LEGACY_REPLAY_SHA256:
        raise ValueError("immutable legacy replay source SHA-256 mismatch")

    output_dir.mkdir(parents=True)
    fixture_hashes = {}
    try:
        for name, seed, frame_index, robot_id in HISTORICAL_CASES:
            runtime, offline = _runtime_prefix(data, seed=seed, target_frame=frame_index)
            offline["baseline_target_comparison"] = _comparison_row(
                baseline_rows, seed=seed, frame_index=frame_index, robot_id=robot_id
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
        manifest = {
            "schema_id": STAGE0_SCHEMA_ID,
            "legacy_replay_sha256": LEGACY_REPLAY_SHA256,
            "sources": {
                "truth_data_sha256": truth_hash,
                "input_manifest_sha256": manifest_hash,
                "baseline_process_sha256": baseline_hash,
            },
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
    offline: dict,
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
    estimate = output.get("estimate")
    truth = _offline_position(offline, frame_index, robot_id)
    error = None
    if estimate is not None and truth is not None:
        point = np.asarray(estimate, dtype=float)
        if point.shape == (2,) and np.isfinite(point).all():
            error = float(np.linalg.norm(point - truth))
    selected = attempt.get("selected_candidate") if isinstance(attempt, dict) else None
    return {
        "frame_index": frame_index,
        "robot_id": robot_id,
        "attempt_status": output.get("attempt_status"),
        "output_status": output.get("output_status"),
        "prediction_age": output.get("prediction_age"),
        "selected_candidate_source": None if selected is None else selected.get("source"),
        "candidate_diagnostics": attempt.get("candidates", []),
        "reference_freshness": references,
        "offline_error_norm": error,
    }


def _run_historical(runtime: dict, offline: dict) -> list[dict]:
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
                attempt=attempt, qualification=qualification, offline=offline,
                uav_outputs=current,
            ))
        outputs, private = current, current_private
    return rows


def _run_reacquisition(runtime: dict, offline: dict) -> list[dict]:
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
            attempt=attempt, qualification=qualification, offline=offline,
            uav_outputs={},
        ))
    return rows


def run_stage0_fixture(path: Path) -> dict:
    """Run the online estimator first, then attach offline fixture evaluation."""
    fixture = _strict_load(Path(path))
    if fixture.get("schema_id") != STAGE0_SCHEMA_ID:
        raise ValueError("unrecognized Stage 0 fixture schema")
    runtime = fixture.get("runtime")
    offline = fixture.get("offline")
    if not isinstance(runtime, dict) or not isinstance(offline, dict):
        raise ValueError("fixture requires separate runtime and offline objects")
    if runtime.get("kind") == "historical_prefix":
        rows = _run_historical(runtime, offline)
        target = runtime["target"]
        selected = next(
            row for row in rows
            if row["frame_index"] == target["frame_index"] and row["robot_id"] == target["robot_id"]
        )
    elif runtime.get("kind") == "synthetic_reacquisition":
        rows = _run_reacquisition(runtime, offline)
        selected = rows[-1]
    else:
        raise ValueError("unknown runtime fixture kind")
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
