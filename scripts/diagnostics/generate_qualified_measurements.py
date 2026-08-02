"""Generate immutable runtime and analyzer-only range streams."""

import gzip
import argparse
import hashlib
import io
import json
import math
import os
import shutil
import uuid
import sys
from pathlib import Path

import numpy as np


SCHEMA_VERSION = "cbf2026-qualified-measurements-v1"
STOP_FREE_BYTES = 6_000_000_000
RUNTIME_FIELDS = {
    "schema_version", "record_type", "frame_index", "owner_id",
    "reference_key", "role_tags", "reference_position",
    "reference_covariance", "base_anchor_provenance", "noisy_range",
    "ranging_sigma", "config_sha256", "measurement_stream_id",
}


def generate_measurement_bundle(
    truth_path: Path,
    output_root: Path,
    *,
    range_noise_seed: int,
    ranging_sigma: float,
    config_sha256: str,
    required_edges_by_condition,
    truth_manifest: dict,
    truth_opener=None,
    publish_hook=None,
    available_bytes_fn=None,
) -> dict:
    """Consume committed truth once and transactionally publish both streams."""
    truth_path = Path(truth_path)
    output_root = Path(output_root)
    if output_root.exists() or output_root.is_symlink():
        raise FileExistsError(f"measurement bundle root must be absent: {output_root}")
    if truth_path.is_symlink() or not truth_path.is_file():
        raise ValueError("truth stream must be a regular non-symlink file")
    if (
        not isinstance(truth_manifest, dict)
        or truth_manifest.get("terminal") is not True
        or truth_manifest.get("status") != "completed"
        or not _sha256_text(truth_manifest.get("sha256"))
    ):
        raise ValueError("truth manifest must be terminal and identity-bound")
    _validate_generation_parameters(
        range_noise_seed, ranging_sigma, config_sha256
    )
    required = _normalize_required(required_edges_by_condition)
    selector_policy = all(isinstance(value, str) for value in required.values())
    union = None if selector_policy else set().union(*required.values())
    edge_identity = (
        required if selector_policy else {
            condition: [
                [frame, owner, list(reference)]
                for frame, owner, reference in sorted(keys, key=repr)
            ]
            for condition, keys in sorted(required.items())
        }
    )
    edge_universe_sha256 = hashlib.sha256(_json_bytes(edge_identity)).hexdigest()
    identity_payload = {
        "schema_version": SCHEMA_VERSION,
        "truth_sha256": truth_manifest["sha256"],
        "range_noise_seed": range_noise_seed,
        "ranging_sigma": float(ranging_sigma),
        "config_sha256": config_sha256,
        "edge_universe_sha256": edge_universe_sha256,
    }
    measurement_stream_id = hashlib.sha256(
        _json_bytes(identity_payload)
    ).hexdigest()
    output_root.parent.mkdir(parents=True, exist_ok=True)
    stage = output_root.with_name(f".{output_root.name}.{uuid.uuid4().hex}.tmp")
    stage.mkdir(mode=0o700)
    runtime_path = stage / "runtime.jsonl.gz"
    audit_path = stage / "audit.jsonl.gz"
    seen = set() if not selector_policy else None
    previous_key = None
    rng = np.random.default_rng(range_noise_seed)
    row_count = 0
    open_truth = truth_opener or (lambda path: Path(path).open("rb"))
    available_probe = available_bytes_fn or (
        lambda path: shutil.disk_usage(path).free
    )
    disk_failure = None
    try:
        try:
            with (
                open_truth(truth_path) as truth_raw,
                runtime_path.open("xb") as runtime_raw,
                audit_path.open("xb") as audit_raw,
                gzip.GzipFile(fileobj=_HashingReader(truth_raw), mode="rb") as truth_gzip,
                io.TextIOWrapper(truth_gzip, encoding="utf-8", errors="strict") as truth,
                gzip.GzipFile(filename="", mode="wb", fileobj=runtime_raw, mtime=0) as runtime_sink,
                gzip.GzipFile(filename="", mode="wb", fileobj=audit_raw, mtime=0) as audit_sink,
            ):
                hashing_reader = truth_gzip.fileobj
                for line_number, line in enumerate(truth, start=1):
                    source_row = _strict_object(line, f"truth row {line_number}")
                    for source in _expand_truth_source(source_row, required):
                        if available_probe(stage) < STOP_FREE_BYTES:
                            raise _MeasurementDiskFloor
                        key = _truth_key(source)
                        if selector_policy:
                            if previous_key is not None and key <= previous_key:
                                raise ValueError("controller measurement keys are not strictly ordered")
                            previous_key = key
                        else:
                            if key in seen:
                                raise ValueError(f"duplicate truth key: {key}")
                            seen.add(key)
                        expected_roles = (
                            sorted(source.get("role_tags", [])) if selector_policy else sorted(
                                condition for condition, keys in required.items() if key in keys
                            )
                        )
                        roles = source.get("role_tags")
                        if not isinstance(roles, list) or sorted(roles) != expected_roles:
                            raise ValueError(f"role-tagged requested edge mismatch: {key}")
                        endpoint = _vec2(source.get("endpoint_truth_position"), "endpoint truth")
                        reference_truth = _vec2(
                            source.get("reference_truth_position"), "reference truth"
                        )
                        reference_position = _vec2(
                            source.get("reference_position"), "reference position"
                        )
                        covariance = _matrix2(
                            source.get("reference_covariance"), "reference covariance"
                        )
                        provenance = source.get("base_anchor_provenance")
                        if not isinstance(provenance, list) or not all(
                            type(item) is int and item >= 0 for item in provenance
                        ):
                            raise ValueError("base anchor provenance is invalid")
                        noiseless_range = math.dist(endpoint, reference_truth)
                        sampled_noise = float(rng.normal(0.0, ranging_sigma))
                        runtime_row = _runtime_row(
                            key, roles, reference_position, covariance, provenance,
                            noiseless_range + sampled_noise, ranging_sigma,
                            config_sha256, measurement_stream_id,
                        )
                        audit_row = _audit_row(
                            key, endpoint, reference_truth, noiseless_range,
                            sampled_noise, range_noise_seed, measurement_stream_id,
                        )
                        runtime_sink.write(_json_bytes(runtime_row) + b"\n")
                        audit_sink.write(_json_bytes(audit_row) + b"\n")
                        row_count += 1
                observed_truth_sha256 = hashing_reader.hexdigest()
        except _MeasurementDiskFloor:
            disk_failure = "disk_hard_floor"
        if disk_failure is not None:
            runtime_manifest = _stream_manifest(
                runtime_path, "runtime", row_count, measurement_stream_id
            )
            audit_manifest = _stream_manifest(
                audit_path, "audit", row_count, measurement_stream_id
            )
            for manifest in (runtime_manifest, audit_manifest):
                manifest.update(identity_payload)
                manifest.update({
                    "status": "failed",
                    "reason": disk_failure,
                })
            _publish_json(stage / "runtime.manifest.json", runtime_manifest)
            _publish_json(stage / "audit.manifest.json", audit_manifest)
            manifest = {
                "schema_version": "cbf2026-qualified-measurement-bundle-v1",
                "terminal": True,
                "status": "failed",
                "reason": disk_failure,
                "truth_sha256": truth_manifest["sha256"],
                "config_sha256": config_sha256,
                "range_noise_seed": range_noise_seed,
                "rng_identity": f"numpy.default_rng.PCG64:{range_noise_seed}",
                "edge_universe_sha256": edge_universe_sha256,
                "measurement_stream_id": measurement_stream_id,
                "runtime_sha256": runtime_manifest["sha256"],
                "audit_sha256": audit_manifest["sha256"],
                "row_count": row_count,
            }
            _publish_json(stage / "manifest.json", manifest)
            _rename_directory_no_replace(stage, output_root)
            return manifest
        if observed_truth_sha256 != truth_manifest["sha256"]:
            raise ValueError("truth SHA-256 changed during its single read")
        if not selector_policy and seen != union:
            raise ValueError("requested edge union does not match truth stream")
        runtime_manifest = _stream_manifest(
            runtime_path, "runtime", row_count, measurement_stream_id
        )
        audit_manifest = _stream_manifest(
            audit_path, "audit", row_count, measurement_stream_id
        )
        for manifest in (runtime_manifest, audit_manifest):
            manifest.update(identity_payload)
        _publish_json(stage / "runtime.manifest.json", runtime_manifest)
        _publish_json(stage / "audit.manifest.json", audit_manifest)
        manifest = {
            "schema_version": "cbf2026-qualified-measurement-bundle-v1",
            "terminal": True,
            "status": "completed",
            "truth_sha256": observed_truth_sha256,
            "config_sha256": config_sha256,
            "range_noise_seed": range_noise_seed,
            "rng_identity": f"numpy.default_rng.PCG64:{range_noise_seed}",
            "edge_universe_sha256": edge_universe_sha256,
            "measurement_stream_id": measurement_stream_id,
            "runtime_sha256": runtime_manifest["sha256"],
            "audit_sha256": audit_manifest["sha256"],
            "row_count": row_count,
        }
        _publish_json(stage / "manifest.json", manifest)
        if publish_hook is not None:
            publish_hook(stage)
        _rename_directory_no_replace(stage, output_root)
        return manifest
    finally:
        if stage.exists():
            shutil.rmtree(stage)


class _MeasurementDiskFloor(Exception):
    pass


class _HashingReader:
    def __init__(self, raw):
        self.raw = raw
        self.digest = hashlib.sha256()

    def read(self, size=-1):
        data = self.raw.read(size)
        self.digest.update(data)
        return data

    def seek(self, offset, whence=0):
        if offset == self.raw.tell() and whence == 0:
            return offset
        raise OSError("qualified truth stream is single-pass")

    def tell(self):
        return self.raw.tell()

    def hexdigest(self):
        return self.digest.hexdigest()


def _validate_generation_parameters(seed, sigma, config_sha256):
    if type(seed) is not int or seed < 0:
        raise ValueError("range-noise seed must be a nonnegative integer")
    if type(sigma) not in (int, float) or sigma != 0.5:
        raise ValueError("qualified ranging_sigma must equal 0.5")
    if not _sha256_text(config_sha256):
        raise ValueError("config_sha256 must be a lowercase SHA-256 identity")


def _runtime_row(
    key, roles, reference_position, covariance, provenance, noisy_range,
    sigma, config_sha256, stream_id,
):
    return {
        "schema_version": SCHEMA_VERSION,
        "record_type": "runtime_measurement",
        "frame_index": key[0],
        "owner_id": key[1],
        "reference_key": list(key[2]),
        "role_tags": roles,
        "reference_position": reference_position,
        "reference_covariance": covariance,
        "base_anchor_provenance": provenance,
        "noisy_range": noisy_range,
        "ranging_sigma": float(sigma),
        "config_sha256": config_sha256,
        "measurement_stream_id": stream_id,
    }


def _audit_row(
    key, endpoint, reference_truth, noiseless_range, sampled_noise, seed,
    stream_id,
):
    audit_key = hashlib.sha256(_json_bytes([
        key[0], key[1], list(key[2]), stream_id
    ])).hexdigest()
    return {
        "schema_version": SCHEMA_VERSION,
        "record_type": "measurement_audit",
        "frame_index": key[0],
        "owner_id": key[1],
        "reference_key": list(key[2]),
        "truth_endpoint_position": endpoint,
        "truth_reference_position": reference_truth,
        "noiseless_range": noiseless_range,
        "sampled_noise": sampled_noise,
        "rng_identity": f"numpy.default_rng.PCG64:{seed}",
        "audit_key": audit_key,
        "measurement_stream_id": stream_id,
    }


def _rename_directory_no_replace(stage: Path, target: Path) -> None:
    from scripts.diagnostics.run_qualified_closure_campaign import (
        _rename_directory_no_replace as durable_publish,
    )
    durable_publish(stage, target)


def _normalize_required(value) -> dict[str, set[tuple[int, int, tuple[object, ...]]]]:
    if not isinstance(value, dict) or set(value) != {
        "dynamic_primary", "fixed_fim_ablation"
    }:
        raise ValueError("both registered measurement conditions are required")
    if all(isinstance(item, str) for item in value.values()):
        if (
            value["dynamic_primary"] != "controller_references"
            or value["fixed_fim_ablation"] not in {
                "controller_references",
                "fixed_paper_localization_references",
            }
        ):
            raise ValueError("measurement edge selector policy is invalid")
        return dict(value)
    if any(isinstance(item, str) for item in value.values()):
        raise ValueError("measurement edge selectors must use one frozen policy")
    normalized = {}
    for condition, keys in value.items():
        condition_keys = set()
        for item in keys:
            if not isinstance(item, (list, tuple)) or len(item) != 3:
                raise ValueError("requested edge key is invalid")
            frame, owner, reference = item
            if type(frame) is not int or frame < 0 or type(owner) is not int or owner <= 0:
                raise ValueError("requested edge key is invalid")
            if not isinstance(reference, (list, tuple)) or len(reference) != 2:
                raise ValueError("requested edge key is invalid")
            key = (frame, owner, tuple(reference))
            if key in condition_keys:
                raise ValueError("requested edge is duplicated")
            condition_keys.add(key)
        normalized[condition] = condition_keys
    return normalized


def _truth_key(row: dict) -> tuple[int, int, tuple[object, ...]]:
    frame = row.get("frame_index")
    owner = row.get("owner_id")
    reference = row.get("reference_key")
    if (
        type(frame) is not int or frame < 0
        or type(owner) is not int or owner <= 0
        or not isinstance(reference, list) or len(reference) != 2
    ):
        raise ValueError("truth measurement key is invalid")
    return frame, owner, tuple(reference)


def _expand_truth_source(source: dict, required: dict):
    """Adapt either a pre-expanded edge row or one real Swarm controller row."""
    if source.get("record_type") != "controller_interval":
        return (source,)
    if source.get("schema_version") != "cbf2026-qualified-evidence-v1":
        raise ValueError("controller truth schema identity is invalid")
    frame = source.get("frame_index")
    runtime = source.get("runtime")
    analyzer = source.get("analyzer_only")
    if type(frame) is not int or frame < 0 or not isinstance(runtime, dict):
        raise ValueError("controller truth boundary is invalid")
    nodes = runtime.get("nodes")
    truth_items = analyzer.get("truth") if isinstance(analyzer, dict) else None
    if not isinstance(nodes, list) or not isinstance(truth_items, list):
        raise ValueError("controller truth boundary is invalid")
    node_by_id = {}
    for node in nodes:
        if not isinstance(node, dict) or type(node.get("robot_id")) is not int:
            raise ValueError("controller node identity is invalid")
        if node["robot_id"] in node_by_id:
            raise ValueError("controller node identity is duplicated")
        node_by_id[node["robot_id"]] = node
    truth_by_id = {}
    for item in truth_items:
        if not isinstance(item, dict) or type(item.get("robot_id")) is not int:
            raise ValueError("controller truth identity is invalid")
        if item["robot_id"] in truth_by_id:
            raise ValueError("controller truth identity is duplicated")
        truth_by_id[item["robot_id"]] = _vec2(item.get("position"), "controller truth")
    if set(node_by_id) != set(truth_by_id):
        raise ValueError("controller runtime/truth robot universe differs")

    if all(isinstance(value, str) for value in required.values()):
        requested_by_condition = {}
        dynamic_requested = []
        for node in sorted(nodes, key=lambda item: item.get("robot_id", -1)):
            for reference in sorted(
                node.get("references", []),
                key=lambda item: (item.get("reference_kind", ""), item.get("canonical_reference_id", -1)),
            ):
                normalized = _controller_reference_key(reference)
                dynamic_requested.append((
                    frame,
                    node["robot_id"],
                    normalized,
                ))
        requested_by_condition["dynamic_primary"] = set(dynamic_requested)
        if required["fixed_fim_ablation"] == "controller_references":
            requested_by_condition["fixed_fim_ablation"] = set(dynamic_requested)
        else:
            requested_by_condition["fixed_fim_ablation"] = set(
                _fixed_paper_reference_keys(frame)
            )
        requested = sorted(
            set().union(*requested_by_condition.values()),
            key=lambda key: (key[0], key[1], key[2][0], key[2][1]),
        )
    else:
        requested = sorted(
            set().union(*required.values()), key=lambda key: (key[0], key[1], repr(key[2]))
        )
        requested = [key for key in requested if key[0] == frame]
    provenance_cache = {}

    def provenance(robot_id, active=()):
        if robot_id in provenance_cache:
            return provenance_cache[robot_id]
        if robot_id in active:
            raise ValueError("controller reference graph is cyclic")
        node = node_by_id.get(robot_id)
        if node is None or not isinstance(node.get("references"), list):
            raise ValueError("controller reference graph is incomplete")
        anchors = set()
        for reference in node["references"]:
            if not isinstance(reference, dict):
                raise ValueError("controller reference record is invalid")
            kind = reference.get("reference_kind")
            identifier = reference.get("canonical_reference_id")
            _normalized_kind, normalized_identifier = _controller_reference_key(reference)
            if kind == "base":
                anchors.add(normalized_identifier)
            else:
                anchors.update(provenance(normalized_identifier, (*active, robot_id)))
        result = sorted(anchors)
        provenance_cache[robot_id] = result
        return result

    rows = []
    for key in requested:
        _frame, owner_id, (kind, reference_id) = key
        owner = node_by_id.get(owner_id)
        if owner is None:
            raise ValueError(f"requested owner is absent: {key}")
        roles = (
            sorted(
                condition for condition, keys in requested_by_condition.items()
                if key in keys
            )
            if all(isinstance(value, str) for value in required.values())
            else sorted(condition for condition, keys in required.items() if key in keys)
        )
        matching = [
            item for item in owner.get("references", [])
            if _controller_reference_key(item) == (kind, reference_id)
        ]
        if len(matching) > 1:
            raise ValueError(f"requested controller reference is duplicated: {key}")
        if kind == "uav":
            reference = node_by_id.get(reference_id)
            if reference is None:
                raise ValueError(f"requested UAV reference is absent: {key}")
            reference_position = _vec2(reference.get("interface_estimate"), "reference position")
            covariance = _matrix2(reference.get("covariance"), "reference covariance")
            reference_truth = truth_by_id[reference_id]
            anchors = provenance(reference_id)
        elif kind == "base" and reference_id in _fixed_base_positions():
            reference_position = list(_fixed_base_positions()[reference_id])
            covariance = [[0.0, 0.0], [0.0, 0.0]]
            reference_truth = list(reference_position)
            anchors = [reference_id]
        else:
            raise ValueError(f"requested base reference is absent: {key}")
        rows.append({
            "frame_index": frame,
            "owner_id": owner_id,
            "reference_key": [kind, reference_id],
            "role_tags": roles,
            "endpoint_truth_position": truth_by_id[owner_id],
            "reference_truth_position": reference_truth,
            "reference_position": reference_position,
            "reference_covariance": covariance,
            "base_anchor_provenance": anchors,
        })
    return rows


def _controller_reference_key(reference):
    if not isinstance(reference, dict):
        raise ValueError("controller reference record is invalid")
    kind = reference.get("reference_kind")
    identifier = reference.get("canonical_reference_id")
    if type(identifier) is not int or kind not in {"base", "uav"}:
        raise ValueError("controller reference identity is invalid")
    if kind == "base":
        base_id = -identifier - 1
        if identifier >= 0 or base_id not in _fixed_base_positions():
            raise ValueError("controller base identity is invalid")
        return kind, base_id
    if identifier <= 0:
        raise ValueError("controller UAV identity is invalid")
    return kind, identifier


def _fixed_paper_reference_keys(frame: int):
    from scripts.diagnostics.qualified_closure_evidence import _LOCALIZATION_EDGES

    for edge in _LOCALIZATION_EDGES:
        if edge.base_id >= 0:
            yield (frame, edge.low, ("base", edge.base_id))
        else:
            yield (frame, edge.high, ("uav", edge.low))


def _fixed_base_positions():
    from scripts.diagnostics.qualified_closure_evidence import _PAPER_BASE_POSITIONS
    return _PAPER_BASE_POSITIONS


def _vec2(value, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{label} must be a finite 2-vector")
    result = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item):
            raise ValueError(f"{label} must be a finite 2-vector")
        result.append(float(item))
    return result


def _matrix2(value, label: str) -> list[list[float]]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{label} must be a finite 2x2 matrix")
    return [_vec2(row, label) for row in value]


def _strict_object(line: str, label: str) -> dict:
    try:
        value = json.loads(
            line,
            parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)),
        )
    except (json.JSONDecodeError, ValueError) as error:
        raise ValueError(f"{label} is not strict JSON") from error
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object")
    return value


def _json_bytes(value) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _sha256_text(value) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _stage_path(target: Path) -> Path:
    return target.with_name(f".{target.name}.{uuid.uuid4().hex}.tmp")


def _link_no_replace(stage: Path, target: Path) -> None:
    os.link(stage, target)
    parent_fd = os.open(target.parent, os.O_RDONLY)
    try:
        os.fsync(parent_fd)
    finally:
        os.close(parent_fd)


def _stream_manifest(path: Path, kind: str, count: int, stream_id: str) -> dict:
    return {
        "schema_version": "cbf2026-qualified-measurement-manifest-v1",
        "terminal": True,
        "status": "completed",
        "stream_kind": kind,
        "row_count": count,
        "measurement_stream_id": stream_id,
        "sha256": _sha256(path),
    }


def _publish_json(path: Path, value: dict) -> None:
    stage = _stage_path(path)
    try:
        with stage.open("xb") as output:
            output.write(_json_bytes(value) + b"\n")
            output.flush()
            os.fsync(output.fileno())
        _link_no_replace(stage, path)
    finally:
        stage.unlink(missing_ok=True)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate immutable qualified measurement streams"
    )
    parser.add_argument("--truth", type=Path, required=True)
    parser.add_argument("--truth-manifest", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--range-noise-seed", type=int, required=True)
    parser.add_argument("--ranging-sigma", type=float, required=True)
    parser.add_argument("--config-sha256", required=True)
    parser.add_argument("--edge-schedule", type=Path, required=True)
    return parser


def main(argv=None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    try:
        edge_schedule = _strict_object(
            arguments.edge_schedule.read_text(encoding="utf-8"),
            "edge schedule",
        )
        if set(edge_schedule) == {"schema_version", "conditions"}:
            edge_schedule = edge_schedule["conditions"]
        truth_manifest = _strict_object(
            arguments.truth_manifest.read_text(encoding="utf-8"),
            "truth manifest",
        )
        generate_measurement_bundle(
            arguments.truth,
            arguments.output_root,
            range_noise_seed=arguments.range_noise_seed,
            ranging_sigma=arguments.ranging_sigma,
            config_sha256=arguments.config_sha256,
            required_edges_by_condition=edge_schedule,
            truth_manifest=truth_manifest,
        )
    except Exception as error:
        print(f"qualified measurement generation failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
