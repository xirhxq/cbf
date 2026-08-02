"""Analyze immutable qualified-closure campaign evidence."""

from collections import Counter
import argparse
import ctypes
import hashlib
import json
import math
import os
import re
import uuid
import sys
import gzip
import io
import sqlite3
import tempfile
import shutil
from pathlib import Path


COMPACT_CAP_BYTES = 25_000_000
ANALYSIS_START_FREE_BYTES = 8_000_000_000
ANALYSIS_STOP_FREE_BYTES = 6_000_000_000
ANALYSIS_CACHE_CAP_BYTES = 2_000_000_000
_INITIAL_FAMILY_AUDIT_CACHE = {}


def validate_analysis_output_root(root, *, available_bytes_fn=None):
    root = Path(root)
    if root.is_symlink():
        raise ValueError("analysis output root must not be symbolic")
    if root.exists():
        raise FileExistsError("analysis output root must be absent")
    ancestor = root.parent
    while not ancestor.exists():
        if ancestor.parent == ancestor:
            raise FileNotFoundError("analysis output root has no existing ancestor")
        ancestor = ancestor.parent
    probe = available_bytes_fn or (lambda path: shutil.disk_usage(path).free)
    available = probe(ancestor)
    if available < ANALYSIS_START_FREE_BYTES:
        raise RuntimeError(
            f"analysis requires at least 8 GB free; available={available}"
        )


def semantic_analysis_sha256(report: dict) -> str:
    if not isinstance(report, dict):
        raise ValueError("analysis report must be an object")
    semantic = dict(report)
    semantic.pop("semantic_analysis_sha256", None)
    semantic.pop("raw_campaign_manifest_sha256", None)
    payload = json.dumps(
        semantic, ensure_ascii=False, allow_nan=False, sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def stream_validated_replay_rows(path: Path, expected_sha256: str, *, on_valid_row) -> int:
    """Stream exact raw rows through the independent recomputation validator."""
    from scripts.diagnostics.analyze_qualified_estimator import (
        validate_and_recompute_qualified_row,
    )
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ValueError("raw replay stream must be a regular file")
    if not callable(on_valid_row):
        raise ValueError("raw replay consumer must be callable")
    count = 0
    previous_key = None
    with path.open("rb") as raw:
        hashing = _HashingReader(raw)
        with (
            gzip.GzipFile(fileobj=hashing, mode="rb") as compressed,
            io.TextIOWrapper(compressed, encoding="utf-8", errors="strict") as source,
        ):
            for line_number, line in enumerate(source, start=1):
                try:
                    row = json.loads(
                        line,
                        parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)),
                    )
                except (json.JSONDecodeError, ValueError) as error:
                    raise ValueError(f"raw replay row {line_number} is not strict JSON") from error
                validate_and_recompute_qualified_row(row)
                key = (row["frame_index"], row["robot_id"])
                if previous_key is not None and key <= previous_key:
                    raise ValueError("raw replay keys are not strictly ordered")
                previous_key = key
                on_valid_row(row)
                count += 1
        observed = hashing.hexdigest()
    if observed != expected_sha256:
        raise ValueError("raw replay SHA-256 differs from terminal manifest")
    return count


class _HashingReader:
    def __init__(self, raw):
        self.raw = raw
        self.digest = hashlib.sha256()

    def read(self, size=-1):
        data = self.raw.read(size)
        self.digest.update(data)
        return data

    def seek(self, offset, whence=0):
        if whence == 0 and offset == self.raw.tell():
            return offset
        raise OSError("qualified raw stream is single-pass")

    def tell(self):
        return self.raw.tell()

    def hexdigest(self):
        return self.digest.hexdigest()


def validate_measurement_pair_streams(
    runtime_path: Path,
    audit_path: Path,
    *,
    runtime_sha256: str,
    audit_sha256: str,
    on_pair=None,
    config_sha256=None,
    measurement_stream_id=None,
    expected_row_count=None,
    range_noise_seed=None,
) -> dict:
    """Validate and join one immutable runtime/audit measurement pair."""
    from scripts.diagnostics.generate_qualified_measurements import RUNTIME_FIELDS
    from scripts.diagnostics.run_qualified_closure_campaign import (
        _validate_runtime_measurement_row,
    )
    audit_fields = {
        "schema_version", "record_type", "frame_index", "owner_id",
        "reference_key", "truth_endpoint_position", "truth_reference_position",
        "noiseless_range", "sampled_noise", "rng_identity", "audit_key",
        "measurement_stream_id",
    }
    consumer = on_pair or (lambda _runtime, _audit: None)
    count = 0
    previous_key = None
    with Path(runtime_path).open("rb") as runtime_raw, Path(audit_path).open("rb") as audit_raw:
        runtime_hashing = _HashingReader(runtime_raw)
        audit_hashing = _HashingReader(audit_raw)
        with (
            gzip.GzipFile(fileobj=runtime_hashing, mode="rb") as runtime_gzip,
            gzip.GzipFile(fileobj=audit_hashing, mode="rb") as audit_gzip,
            io.TextIOWrapper(runtime_gzip, encoding="utf-8", errors="strict") as runtime_source,
            io.TextIOWrapper(audit_gzip, encoding="utf-8", errors="strict") as audit_source,
        ):
            while True:
                runtime_line = runtime_source.readline()
                audit_line = audit_source.readline()
                if not runtime_line and not audit_line:
                    break
                if not runtime_line or not audit_line:
                    raise ValueError("runtime/audit measurement counts differ")
                runtime = _strict_line_object(runtime_line, "runtime measurement")
                audit = _strict_line_object(audit_line, "audit measurement")
                if set(runtime) != RUNTIME_FIELDS or set(audit) != audit_fields:
                    raise ValueError("measurement stream differs from exact schema")
                _validate_runtime_measurement_row(
                    runtime,
                    condition=None,
                    manifest={
                        "config_sha256": (
                            runtime["config_sha256"]
                            if config_sha256 is None else config_sha256
                        ),
                        "measurement_stream_id": (
                            runtime["measurement_stream_id"]
                            if measurement_stream_id is None
                            else measurement_stream_id
                        ),
                    },
                )
                _validate_measurement_audit_row(
                    audit, range_noise_seed=range_noise_seed
                )
                if runtime["record_type"] != "runtime_measurement" or audit["record_type"] != "measurement_audit":
                    raise ValueError("measurement record type differs")
                key = (
                    runtime["frame_index"], runtime["owner_id"],
                    tuple(runtime["reference_key"]),
                )
                audit_key = (
                    audit["frame_index"], audit["owner_id"],
                    tuple(audit["reference_key"]),
                )
                if key != audit_key or previous_key is not None and key <= previous_key:
                    raise ValueError("measurement join keys differ or are not ordered")
                previous_key = key
                if runtime["measurement_stream_id"] != audit["measurement_stream_id"]:
                    raise ValueError("measurement stream identities differ")
                if runtime["ranging_sigma"] != 0.5:
                    raise ValueError("measurement sigma differs from frozen value")
                if not math.isclose(
                    runtime["noisy_range"],
                    audit["noiseless_range"] + audit["sampled_noise"],
                    rel_tol=0.0, abs_tol=1e-12,
                ):
                    raise ValueError("noisy range differs from audit reconstruction")
                expected_audit_key = hashlib.sha256(json.dumps(
                    [key[0], key[1], list(key[2]), runtime["measurement_stream_id"]],
                    ensure_ascii=False, allow_nan=False, sort_keys=True,
                    separators=(",", ":"),
                ).encode("utf-8")).hexdigest()
                if audit["audit_key"] != expected_audit_key:
                    raise ValueError("measurement audit key differs")
                if any(fragment in field.lower() for field in runtime for fragment in (
                    "truth", "noiseless", "sampled_noise", "audit"
                )):
                    raise ValueError("runtime measurement contains analyzer-only fields")
                consumer(runtime, audit)
                count += 1
        if runtime_hashing.hexdigest() != runtime_sha256:
            raise ValueError("runtime measurement SHA-256 differs")
        if audit_hashing.hexdigest() != audit_sha256:
            raise ValueError("audit measurement SHA-256 differs")
    if expected_row_count is not None and count != expected_row_count:
        raise ValueError("measurement row count differs from frozen manifest")
    return {"row_count": count, "runtime_truth_read_count": 0}


def _validate_measurement_audit_row(audit, *, range_noise_seed=None):
    def finite(value):
        return type(value) in {int, float} and math.isfinite(float(value))

    key = audit.get("reference_key")
    vectors = (
        audit.get("truth_endpoint_position"),
        audit.get("truth_reference_position"),
    )
    if (
        audit.get("schema_version") != "cbf2026-qualified-measurements-v1"
        or audit.get("record_type") != "measurement_audit"
        or type(audit.get("frame_index")) is not int
        or audit["frame_index"] < 0
        or type(audit.get("owner_id")) is not int
        or audit["owner_id"] not in range(1, 15)
        or not isinstance(key, list)
        or len(key) != 2
        or key[0] not in {"base", "uav"}
        or type(key[1]) is not int
        or any(
            not isinstance(vector, list)
            or len(vector) != 2
            or not all(finite(value) for value in vector)
            for vector in vectors
        )
        or not finite(audit.get("noiseless_range"))
        or audit["noiseless_range"] < 0.0
        or not finite(audit.get("sampled_noise"))
        or not isinstance(audit.get("rng_identity"), str)
        or not re.fullmatch(r"numpy\.default_rng\.PCG64:[0-9]+", audit["rng_identity"])
        or (
            range_noise_seed is not None
            and audit["rng_identity"]
                != f"numpy.default_rng.PCG64:{range_noise_seed}"
        )
        or not _lower_sha256(audit.get("audit_key"))
        or not _lower_sha256(audit.get("measurement_stream_id"))
    ):
        raise ValueError("measurement audit row types are invalid")
    if not math.isclose(
        audit["noiseless_range"],
        math.dist(vectors[0], vectors[1]),
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise ValueError("measurement audit noiseless range differs from truth")


def _lower_sha256(value):
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def validate_terminal_mission_manifest(manifest: dict) -> bool:
    """Return whether a mission manifest has one exact terminal schema."""
    if not isinstance(manifest, dict) or manifest.get("terminal") is not True:
        return False
    common = {
        "schema_version", "terminal", "status", "mission", "member_identities"
    }
    status = manifest.get("status")
    expected = (
        common | {"swarm", "measurements", "replays"}
        if status == "completed" else common | {"reason"}
        if status == "failed" else set()
    )
    return (
        set(manifest) == expected
        and manifest.get("schema_version") == "cbf2026-qualified-mission-manifest-v1"
        and isinstance(manifest.get("mission"), dict)
        and isinstance(manifest.get("member_identities"), dict)
    )


def analyze_qualified_closure_campaign(campaign: dict) -> dict:
    """Reconstruct and evaluate every registered gate from supplied rows."""
    if not isinstance(campaign, dict):
        raise ValueError("campaign must be a JSON object")
    manifests = _list(campaign, "manifests")
    initialization = _list(campaign, "initialization_rows")
    estimator = _list(campaign, "estimator_rows")
    primary_estimator = [
        row for row in estimator if row.get("condition") == "dynamic_primary"
    ]
    frozen_primary_count = campaign.get("expected_primary_tuple_count")
    if (
        type(frozen_primary_count) is not int
        or frozen_primary_count < len(primary_estimator)
    ):
        raise ValueError("frozen primary tuple count is absent or inconsistent")
    ablation_estimator = [
        row for row in estimator if row.get("condition") == "fixed_fim_ablation"
    ]
    controller = _list(campaign, "controller_rows")
    resets = _list(campaign, "reset_rows")
    missions = _list(campaign, "missions")
    gates = {}

    complete_items = sum(
        manifest.get("terminal") is True
        and manifest.get("status") == "completed"
        for manifest in manifests
    )
    keys_complete = (
        type(campaign.get("expected_key_count")) is int
        and campaign.get("expected_key_count")
            == campaign.get("observed_key_count")
    )
    gates["complete_keys_and_manifests"] = _ratio_gate(
        complete_items + int(keys_complete), len(manifests) + 1, 1.0
    )
    truth_reads = campaign.get("runtime_truth_read_count")
    gates["zero_runtime_truth_reads"] = _zero_gate(
        truth_reads if type(truth_reads) is int and truth_reads >= 0 else 1,
        max(1, len(estimator)),
    )
    gates["zero_initialization_accounting_omissions"] = _zero_gate(
        sum(row.get("accounted") is not True for row in initialization),
        len(initialization),
    )
    gates["zero_cross_mode_source_order_publication"] = _zero_gate(
        sum(
            row.get("output_status") == "fresh"
            and row.get("source_order_valid") is not True
            for row in estimator
        ),
        len(estimator),
    )
    gates["zero_fresh_publication_with_multiple_admissible_modes"] = _zero_gate(
        sum(
            row.get("output_status") == "fresh"
            and row.get("admissible_mode_count") != 1
            for row in primary_estimator
        ),
        len(primary_estimator),
    )
    gates["zero_wrong_mode_fresh_publication"] = _zero_gate(
        sum(
            row.get("output_status") == "fresh"
            and row.get("wrong_mode") is True
            for row in primary_estimator
        ),
        len(primary_estimator),
    )
    finite_errors = [
        float(row["error_m"])
        for row in primary_estimator
        if _finite(row.get("error_m"))
        and row.get("output_status") in {"fresh", "predicted"}
    ]
    gates["zero_finite_error_above_50m"] = _zero_gate(
        sum(error > 50.0 for error in finite_errors), len(finite_errors)
    )

    all_available = [
        row for row in estimator
        if row.get("output_status") in {"fresh", "predicted"}
    ]
    available = [
        row for row in primary_estimator
        if row.get("output_status") in {"fresh", "predicted"}
    ]
    gates["aggregate_containment"] = _ratio_gate(
        sum(row.get("contained") is True for row in available),
        len(available),
        0.98,
    )
    depth_results = {}
    passing_depths = 0
    registered_depth_count = 0
    registered_depths = campaign.get("registered_depths")
    if not isinstance(registered_depths, dict):
        registered_depths = {}
    for condition in sorted(registered_depths):
        depths = registered_depths[condition]
        if not isinstance(depths, list):
            depths = []
        for depth in depths:
            if condition == "dynamic_primary":
                registered_depth_count += 1
            rows = [
                row for row in all_available
                if row.get("condition") == condition and row.get("depth") == depth
            ]
            gate = _ratio_gate(
                sum(row.get("contained") is True for row in rows),
                len(rows),
                0.95,
            )
            depth_results[f"{condition}:depth-{depth}"] = gate
            if condition == "dynamic_primary":
                passing_depths += gate["passed"]
    gates["every_registered_depth_containment"] = _ratio_gate(
        passing_depths, registered_depth_count, 1.0
    )
    gates["full_universe_joint_available_and_contained"] = _ratio_gate(
        sum(
            row.get("output_status") in {"fresh", "predicted"}
            and row.get("contained") is True
            for row in primary_estimator
        ),
        frozen_primary_count,
        0.93,
    )
    gates["fresh_retention"] = _ratio_gate(
        sum(
            row.get("output_status") == "fresh"
            for row in primary_estimator
        ),
        frozen_primary_count,
        0.98,
    )
    gates["fresh_or_bounded_predicted_availability"] = _ratio_gate(
        sum(
            row.get("output_status") == "fresh"
            or (
                row.get("output_status") == "predicted"
                and row.get("bounded_prediction") is True
            )
            for row in primary_estimator
        ),
        frozen_primary_count,
        0.95,
    )

    hard_rows = campaign.get("hard_rows")
    if not isinstance(hard_rows, dict):
        hard_rows = {}
    gates["zero_missing_duplicate_hard_rows"] = _zero_gate(
        _nonnegative_count(hard_rows.get("missing"))
        + _nonnegative_count(hard_rows.get("duplicate")),
        max(1, len(controller)),
    )
    gates["controller_certificate_availability"] = _ratio_gate(
        sum(row.get("certificate_available") is True for row in controller),
        len(controller),
        0.99,
    )
    gates["zero_nu_bound_excess"] = _zero_gate(
        sum(
            row.get("certificate_available") is True
            and (
                not _finite(row.get("nu_inst"))
                or not _finite(row.get("bar_nu"))
                or float(row["nu_inst"]) > float(row["bar_nu"]) + 1e-9
            )
            for row in controller
        ),
        sum(row.get("certificate_available") is True for row in controller),
    )
    gates["zero_accepted_reset_violations"] = _zero_gate(
        sum(row.get("accepted_violation") is True for row in resets),
        len(resets),
    )
    gates["zero_input_limit_violations"] = _zero_gate(
        sum(
            not _finite(row.get("input_limit_excess"))
            or float(row["input_limit_excess"]) > 1e-7
            for row in controller
        ),
        len(controller),
    )
    gates["zero_primary_local_hard_qp_infeasibility"] = _zero_gate(
        sum(row.get("qp_status") != "optimal" for row in controller),
        len(controller),
    )
    gates["zero_negative_hard_residual"] = _zero_gate(
        sum(
            not _finite(row.get("local_residual_min"))
            or not _finite(row.get("reconstructed_residual_min"))
            or float(row["local_residual_min"]) < -1e-7
            or float(row["reconstructed_residual_min"]) < -1e-7
            for row in controller
        ),
        len(controller),
    )
    successful_ids = {
        mission.get("mission_id")
        for mission in missions
        if mission.get("success") is True
    }
    relevant_controller = [
        row for row in controller
        if "mission_id" not in row or row.get("mission_id") in successful_ids
    ]
    gates["zero_true_distance_violations"] = _zero_gate(
        sum(
            row.get("true_localization_violation") is True
            or row.get("true_collision_violation") is True
            for row in relevant_controller
        ),
        len(relevant_controller),
    )
    gates["successful_mission_fraction"] = _ratio_gate(
        sum(
            mission.get("terminal") is True and mission.get("success") is True
            for mission in missions
        ),
        len(missions),
        0.95,
    )
    identities = campaign.get("measurement_sha256_by_condition")
    identical_measurements = (
        isinstance(identities, dict)
        and set(identities) == {"dynamic_primary", "fixed_fim_ablation"}
        and len(set(identities.values())) == 1
        and campaign.get("measurement_regenerated_by_analyzer") is False
    )
    gates["identical_immutable_measurements"] = _ratio_gate(
        int(identical_measurements), 1, 1.0
    )

    errors_by_depth = {}
    for row in estimator:
        if _finite(row.get("error_m")):
            key = f"{row.get('condition')}:depth-{row.get('depth')}"
            errors_by_depth.setdefault(key, []).append(float(row["error_m"]))
    report = {
        "schema_version": "cbf2026-qualified-closure-analysis-v1",
        "passed": all(gate["passed"] for gate in gates.values()),
        "gates": gates,
        "depth_containment": depth_results,
        "errors_m": _distribution(finite_errors),
        "ablation": {
            "row_count": len(ablation_estimator),
            "finite_error_above_50m": sum(
                _finite(row.get("error_m")) and float(row["error_m"]) > 50.0
                for row in ablation_estimator
            ),
            "errors_m": _distribution([
                float(row["error_m"])
                for row in ablation_estimator
                if _finite(row.get("error_m"))
            ]),
        },
        "depth_errors_m": {
            key: _distribution(values) for key, values in sorted(errors_by_depth.items())
        },
        "private_age_strata": dict(sorted(Counter(
            str(row.get("private_age")) for row in estimator
        ).items())),
        "reset_reasons": dict(sorted(Counter(
            str(row.get("reason")) for row in resets
        ).items())),
        "certificate_failures": [
            index for index, row in enumerate(controller)
            if row.get("certificate_available") is not True
        ],
        "allocation_conservatism_stress": dict(sorted(Counter(
            str(row.get("allocation_stress", "unavailable"))
            for row in controller
        ).items())),
        "deadlocks": [
            index for index, row in enumerate(controller)
            if row.get("deadlock") is True
        ],
        "incomplete_missions": [
            str(mission.get("mission_id")) for mission in missions
            if mission.get("terminal") is not True or mission.get("success") is not True
        ],
    }
    return report


def publish_analysis_bundle(
    output_root: Path,
    report: dict,
    markdown: str,
    *,
    allocated_size_fn=None,
    publish_hook=None,
) -> dict:
    """Stage and no-replace publish a bounded compact analysis bundle."""
    output_root = Path(output_root)
    if output_root.exists() or output_root.is_symlink():
        raise FileExistsError(f"analysis root must be absent: {output_root}")
    output_root.parent.mkdir(parents=True, exist_ok=True)
    bundle_stage = output_root.with_name(f".{output_root.name}.{uuid.uuid4().hex}.tmp")
    bundle_stage.mkdir(mode=0o700)
    json_stage = bundle_stage / "analysis.json"
    markdown_stage = bundle_stage / "analysis.md"
    try:
        _write_exclusive(
            json_stage,
            json.dumps(
                report,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8") + b"\n",
        )
        _write_exclusive(markdown_stage, markdown.encode("utf-8"))
        size_probe = allocated_size_fn or _allocated_files
        allocated = size_probe((json_stage, markdown_stage))
        if allocated > COMPACT_CAP_BYTES:
            manifest = {
                "schema_version": "cbf2026-qualified-analysis-manifest-v1",
                "terminal": True,
                "status": "failed",
                "reason": "compact_bundle_cap",
                "allocated_bytes": allocated,
                "cap_bytes": COMPACT_CAP_BYTES,
            }
            json_stage.unlink()
            markdown_stage.unlink()
            _publish_json_no_replace(bundle_stage / "manifest.json", manifest)
            if publish_hook is not None:
                publish_hook(bundle_stage)
            _rename_directory_no_replace(bundle_stage, output_root)
            return manifest

        json_identity = _identity(json_stage)
        markdown_identity = _identity(markdown_stage)
        manifest = {
            "schema_version": "cbf2026-qualified-analysis-manifest-v1",
            "terminal": True,
            "status": "completed",
            "reason": "completed",
            "allocated_bytes": allocated,
            "cap_bytes": COMPACT_CAP_BYTES,
            "analysis_json": json_identity,
            "analysis_markdown": markdown_identity,
        }
        _publish_json_no_replace(bundle_stage / "manifest.json", manifest)
        if publish_hook is not None:
            publish_hook(bundle_stage)
        _rename_directory_no_replace(bundle_stage, output_root)
        return manifest
    finally:
        if bundle_stage.exists():
            import shutil
            shutil.rmtree(bundle_stage)


def _publish_claimed_analysis_bundle(output_root, report, markdown):
    """Publish a terminal compact bundle inside an already claimed root."""
    output_root = Path(output_root)
    stage = output_root.with_name(
        f".{output_root.name}.bundle.{uuid.uuid4().hex}.tmp"
    )
    stage.mkdir(mode=0o700)
    json_stage = stage / "analysis.json"
    markdown_stage = stage / "analysis.md"
    try:
        _write_exclusive(
            json_stage,
            json.dumps(
                report, ensure_ascii=False, allow_nan=False, sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8") + b"\n",
        )
        _write_exclusive(markdown_stage, markdown.encode("utf-8"))
        allocated = _allocated_files((json_stage, markdown_stage))
        if allocated > COMPACT_CAP_BYTES:
            raise RuntimeError("compact_bundle_cap")
        manifest = {
            "schema_version": "cbf2026-qualified-analysis-manifest-v1",
            "terminal": True,
            "status": "completed",
            "reason": "completed",
            "allocated_bytes": allocated,
            "cap_bytes": COMPACT_CAP_BYTES,
            "analysis_json": _identity(json_stage),
            "analysis_markdown": _identity(markdown_stage),
        }
        manifest_stage = stage / "manifest.json"
        _write_exclusive(
            manifest_stage,
            json.dumps(
                manifest, sort_keys=True, separators=(",", ":")
            ).encode() + b"\n",
        )
        _exchange_claimed_analysis_root(stage, output_root)
        return manifest
    finally:
        if stage.exists():
            shutil.rmtree(stage)


def _terminalize_claimed_analysis_failure(output_root, error):
    """Atomically replace owned partial outputs with one failure bundle."""
    output_root = Path(output_root)
    reason = f"analysis_exception:{type(error).__name__}"
    allocated = 0
    stage = output_root.with_name(
        f".{output_root.name}.failure.{uuid.uuid4().hex}.tmp"
    )
    stage.mkdir(mode=0o700)
    try:
        _publish_json_no_replace(stage / "manifest.json", {
            "schema_version": "cbf2026-qualified-analysis-manifest-v1",
            "terminal": True,
            "status": "failed",
            "reason": reason,
            "allocated_bytes": allocated,
            "cap_bytes": COMPACT_CAP_BYTES,
        })
        _exchange_claimed_analysis_root(stage, output_root)
    finally:
        if stage.exists():
            shutil.rmtree(stage)


def _exchange_claimed_analysis_root(stage: Path, target: Path) -> None:
    """Atomically swap a complete bundle with its continuously owned claim."""
    stage = Path(stage)
    target = Path(target)
    if (
        stage.parent.resolve() != target.parent.resolve()
        or stage.is_symlink()
        or target.is_symlink()
        or not stage.is_dir()
        or not target.is_dir()
    ):
        raise ValueError("analysis directory exchange operands are invalid")
    stage_fd = os.open(stage, os.O_RDONLY)
    try:
        os.fsync(stage_fd)
    finally:
        os.close(stage_fd)
    libc = ctypes.CDLL(None, use_errno=True)
    source_bytes = os.fsencode(stage)
    target_bytes = os.fsencode(target)
    if sys.platform == "darwin" and hasattr(libc, "renamex_np"):
        result = libc.renamex_np(source_bytes, target_bytes, 0x2)
    elif sys.platform.startswith("linux") and hasattr(libc, "renameat2"):
        result = libc.renameat2(-100, source_bytes, -100, target_bytes, 0x2)
    else:
        raise RuntimeError("atomic analysis directory exchange is unavailable")
    if result != 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number), target)
    parent_fd = os.open(target.parent, os.O_RDONLY)
    try:
        os.fsync(parent_fd)
    finally:
        os.close(parent_fd)


def _enforce_analysis_disk_policy(output_parent, spool_root):
    """Enforce the registered 6 GB hard floor and 2 GB SQLite cache cap."""
    if shutil.disk_usage(output_parent).free < ANALYSIS_STOP_FREE_BYTES:
        raise RuntimeError("analysis_disk_hard_floor")
    allocated = sum(
        path.stat().st_blocks * 512
        for path in Path(spool_root).rglob("*")
        if path.is_file() and not path.is_symlink()
    )
    if allocated > ANALYSIS_CACHE_CAP_BYTES:
        raise RuntimeError("analysis_cache_cap")


def _rename_directory_no_replace(stage: Path, target: Path) -> None:
    from scripts.diagnostics.run_qualified_closure_campaign import (
        _rename_directory_no_replace as durable_publish,
    )
    durable_publish(stage, target)


def _list(campaign: dict, key: str) -> list[dict]:
    value = campaign.get(key)
    if not isinstance(value, list) or not all(isinstance(row, dict) for row in value):
        raise ValueError(f"{key} must be a list of JSON objects")
    return value


def _finite(value) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(value)
    )


def _nonnegative_count(value) -> int:
    return value if type(value) is int and value >= 0 else 1


def _gate(numerator, denominator, comparison, threshold, passed):
    return {
        "numerator": int(numerator),
        "denominator": int(denominator),
        "comparison": comparison,
        "threshold": threshold,
        "passed": bool(passed),
        "pass_fail": "PASS" if passed else "FAIL",
    }


def _zero_gate(violations: int, denominator: int) -> dict:
    return _gate(violations, denominator, "<=", 0, violations == 0)


def _ratio_gate(numerator: int, denominator: int, threshold: float) -> dict:
    value = None if denominator == 0 else numerator / denominator
    return _gate(
        numerator,
        denominator,
        ">=",
        threshold,
        value is not None and value >= threshold,
    )


def _distribution(values: list[float]) -> dict:
    ordered = sorted(values)
    return {
        "count": len(ordered),
        "max": None if not ordered else ordered[-1],
        "p95": _quantile(ordered, 0.95),
        "p99": _quantile(ordered, 0.99),
    }


def _quantile(ordered: list[float], probability: float):
    if not ordered:
        return None
    position = (len(ordered) - 1) * probability
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return ordered[low]
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def _write_exclusive(path: Path, payload: bytes) -> None:
    with path.open("xb") as output:
        output.write(payload)
        output.flush()
        os.fsync(output.fileno())


def _publish_json_no_replace(path: Path, payload: dict) -> None:
    stage = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        _write_exclusive(
            stage,
            json.dumps(
                payload,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8") + b"\n",
        )
        os.link(stage, path)
        parent_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(parent_fd)
        finally:
            os.close(parent_fd)
    finally:
        stage.unlink(missing_ok=True)


def _allocated_files(paths) -> int:
    return sum(Path(path).stat().st_blocks * 512 for path in paths)


def _identity(path: Path) -> dict:
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    return {"sha256": digest, "bytes": path.stat().st_size}


def _exact_file_identity(identity) -> bool:
    return (
        isinstance(identity, dict)
        and set(identity) == {"sha256", "bytes"}
        and isinstance(identity["sha256"], str)
        and re.fullmatch(r"[0-9a-f]{64}", identity["sha256"]) is not None
        and type(identity["bytes"]) is int
        and identity["bytes"] >= 0
    )


def _canonical_json_identity(payload: dict) -> dict:
    encoded = json.dumps(
        payload,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8") + b"\n"
    return {
        "sha256": hashlib.sha256(encoded).hexdigest(),
        "bytes": len(encoded),
    }


def _validate_producer_namespace_identities(
    producer_identities,
    *,
    declared_members,
    replay,
    condition,
    measurement_manifest,
    runtime_sha,
    mission_id,
) -> None:
    expected_members = {
        "commands.jsonl.gz", "commands.manifest.json", "config.json",
        "measurements.jsonl.gz", "measurements.manifest.json",
    }
    measurement_member = declared_members.get("measurements/runtime.jsonl.gz")
    command_member = declared_members.get("swarm-inputs/commands.jsonl.gz")
    condition_config_member = declared_members.get(
        f"materialized-{condition}.json"
    )
    primary_config_member = declared_members.get("materialized-primary.json")
    expected_measurement_manifest = _canonical_json_identity({
        "schema_version": "cbf2026-qualified-producer-measurements-v1",
        "terminal": True,
        "status": "completed",
        "sha256": runtime_sha,
        "measurement_stream_id": measurement_manifest["measurement_stream_id"],
        "config_sha256": measurement_manifest["config_sha256"],
        "row_count": measurement_manifest["row_count"],
    })
    expected_commands_manifest = _canonical_json_identity({
        "schema_version": "cbf2026-qualified-producer-commands-v1",
        "terminal": True,
        "status": "completed",
        "sha256": command_member.get("sha256")
            if isinstance(command_member, dict) else None,
    })
    if (
        not isinstance(producer_identities, dict)
        or set(producer_identities) != expected_members
        or any(
            not _exact_file_identity(identity)
            for identity in producer_identities.values()
        )
        or producer_identities["measurements.jsonl.gz"] != measurement_member
        or producer_identities["commands.jsonl.gz"] != command_member
        or producer_identities["config.json"] != condition_config_member
        or producer_identities["config.json"]["sha256"]
            != replay.get("config_sha256")
        or (
            condition == "dynamic_primary"
            and producer_identities["config.json"] != primary_config_member
        )
        or producer_identities["measurements.manifest.json"]
            != expected_measurement_manifest
        or producer_identities["commands.manifest.json"]
            != expected_commands_manifest
    ):
        raise ValueError(f"{mission_id} producer namespace identity differs")


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze one immutable qualified-closure campaign"
    )
    parser.add_argument("--kind", choices=("development", "confirmatory", "confirmatory-smoke"), required=True)
    parser.add_argument("--version", required=True)
    parser.add_argument("--smoke-id", choices=("a", "b"))
    parser.add_argument("--protocol", type=Path, required=True)
    parser.add_argument("--authorization", type=Path, required=True)
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--ablation-config", type=Path)
    parser.add_argument("--output-root", type=Path, required=True)
    return parser


def main(argv=None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    try:
        return int(analyze_campaign_from_arguments(arguments))
    except Exception as error:
        print(f"qualified analysis failed: {error}", file=sys.stderr)
    return 1


def validate_analysis_registration_contract(protocol, arguments):
    """Validate kind/root/schedule binding before any analysis allocation."""
    smoke = arguments.kind == "confirmatory-smoke"
    if smoke:
        if arguments.smoke_id not in {"a", "b"}:
            raise ValueError("confirmatory-smoke analysis requires smoke-id a/b")
        registered_kind = "confirmatory"
        claimed_root = f"smoke_{arguments.smoke_id}_raw"
        output_label = f"smoke_{arguments.smoke_id}_analysis"
    else:
        registered_kind = arguments.kind
        claimed_root = "raw"
        output_label = "analysis"
    if protocol.get("kind") != registered_kind:
        raise ValueError("analysis kind differs from registered protocol")
    expected_version = "v5" if registered_kind == "development" else "v1"
    if (
        arguments.version != expected_version
        or protocol.get("version") != arguments.version
    ):
        if registered_kind == "development":
            raise ValueError("development analysis requires version v5")
        raise ValueError("analysis version differs from registered protocol")
    input_root = Path(arguments.input_root)
    output_root = Path(arguments.output_root)
    roots = protocol.get("roots", {})
    if roots.get(claimed_root) != str(input_root.resolve()):
        raise ValueError("analysis input root differs from protocol")
    if roots.get(output_label) != str(output_root.resolve()):
        raise ValueError("analysis output root differs from protocol")
    binding = protocol.get("bindings", {}).get("ablation_config")
    if arguments.kind == "development":
        path = arguments.ablation_config
        if (
            path is None
            or not isinstance(binding, dict)
            or Path(path).is_symlink()
            or not Path(path).is_file()
            or Path(path).resolve() != Path(binding.get("path", "")).resolve()
            or _sha256(Path(path)) != binding.get("sha256")
            or Path(path).stat().st_size != binding.get("bytes")
        ):
            raise ValueError("analysis ablation config identity differs")
    elif arguments.ablation_config is not None:
        raise ValueError("analysis argv contains an unregistered ablation config")
    _validate_runtime_analyzer_argv(protocol, arguments)
    initial_audits = (
        _load_development_initial_state_contract(protocol)
        if registered_kind == "development" else None
    )
    if smoke:
        smoke_schedule = protocol.get("smoke_schedule")
        if (
            not isinstance(smoke_schedule, dict)
            or smoke_schedule.get("included_in_scientific_denominator") is not False
            or not isinstance(smoke_schedule.get("universes"), dict)
        ):
            raise ValueError("registered smoke analysis contract is invalid")
        registered_missions = [{
            "mission_id": "mission-01",
            "trajectory_seed": smoke_schedule.get("trajectory_seed"),
            "range_noise_seed": smoke_schedule.get("range_noise_seed"),
            "frames": smoke_schedule.get("frames"),
        }]
        analysis_protocol = {
            **protocol,
            "universes": dict(smoke_schedule["universes"]),
        }
    else:
        registered_missions = protocol.get("schedule", {}).get("missions")
        analysis_protocol = protocol
    campaign_id = (
        f"confirmatory-smoke-{arguments.smoke_id}"
        if smoke
        else f"{registered_kind}-{expected_version}"
    )
    registered_schedule = _registered_schedule_envelope(
        registered_missions, campaign_id=campaign_id
    )
    return {
        "claimed_root": claimed_root,
        "output_label": output_label,
        "registered_missions": registered_missions,
        "registered_schedule": registered_schedule,
        "analysis_protocol": analysis_protocol,
        "initial_audits": initial_audits,
    }


def _registered_schedule_envelope(registered_missions, *, campaign_id):
    """Derive the full trusted raw schedule envelope from registered missions."""
    expected_fields = {
        "mission_id", "trajectory_seed", "range_noise_seed", "frames",
    }
    development_v5 = campaign_id == "development-v5"
    if development_v5:
        expected_fields.add("initial_positions_sha256")
    if not isinstance(registered_missions, list) or not isinstance(campaign_id, str):
        raise ValueError("registered mission schedule is invalid")
    schedule = []
    for mission in registered_missions:
        if (
            not isinstance(mission, dict)
            or set(mission) != expected_fields
            or not isinstance(mission.get("mission_id"), str)
            or type(mission.get("trajectory_seed")) is not int
            or type(mission.get("range_noise_seed")) is not int
            or type(mission.get("frames")) is not int
            or mission["frames"] <= 0
            or (
                development_v5
                and not _lower_sha256(mission.get("initial_positions_sha256"))
            )
        ):
            raise ValueError("registered mission schedule is invalid")
        schedule.append({
            "campaign_id": campaign_id,
            **mission,
            "horizon_s": mission["frames"] / 2.0,
            "conditions": ["dynamic_primary", "fixed_fim_ablation"],
        })
    return schedule


def _load_development_initial_state_contract(protocol):
    """Recompute the full frozen v5 family and bind its registered missions."""
    from scripts.diagnostics.qualified_initial_state import (
        audit_frozen_initial_family,
        load_qualified_initial_family,
    )

    if not isinstance(protocol, dict) or protocol.get("kind") != "development":
        raise ValueError("development initial-state protocol is invalid")
    if protocol.get("version") != "v5":
        raise ValueError("development initial state requires version v5")
    identity = protocol.get("bindings", {}).get("initial_family")
    if (
        not isinstance(identity, dict)
        or set(identity) != {"path", "sha256", "bytes"}
        or not isinstance(identity.get("path"), str)
        or not _lower_sha256(identity.get("sha256"))
        or type(identity.get("bytes")) is not int
        or identity["bytes"] < 0
    ):
        raise ValueError("development initial-family binding is invalid")
    family_path = Path(identity["path"])
    observed_sha256 = None
    if (
        family_path.is_symlink()
        or not family_path.is_file()
        or (observed_sha256 := _sha256(family_path)) != identity["sha256"]
        or family_path.stat().st_size != identity["bytes"]
    ):
        raise ValueError("development initial-family identity differs")
    family = load_qualified_initial_family(family_path)
    if (
        _sha256(family_path) != observed_sha256
        or family_path.stat().st_size != identity["bytes"]
    ):
        raise ValueError("development initial-family changed during validation")
    cache_key = (observed_sha256, family["semantic_sha256"])
    audited = _INITIAL_FAMILY_AUDIT_CACHE.get(cache_key)
    if audited is None:
        audited = audit_frozen_initial_family(family)
        _INITIAL_FAMILY_AUDIT_CACHE[cache_key] = audited
    expected_initial_state = {
        "family_schema_version": family["schema_version"],
        "namespace": family["namespace"],
        "family_semantic_sha256": family["semantic_sha256"],
        "registered_trajectory_seeds": [
            audit.seed for audit in audited.registered.audits
        ],
        "audit_trajectory_seeds": [
            audit.seed for audit in audited.audit.audits
        ],
        "missions": [
            {
                "trajectory_seed": audit.seed,
                "positions_sha256": audit.positions_sha256,
            }
            for audit in audited.registered.audits
        ],
        "frozen_summary": family["frozen_summary"],
        "admission": family["admission"],
        "perturbation_policy": {
            "clamp": family["perturbation"]["clamp"],
            "resample": family["perturbation"]["resample"],
        },
    }
    if protocol.get("initial_state") != expected_initial_state:
        raise ValueError("development initial state differs from frozen family")
    scheduled = protocol.get("schedule", {}).get("missions")
    if not isinstance(scheduled, list) or [
        {
            "trajectory_seed": mission.get("trajectory_seed"),
            "positions_sha256": mission.get("initial_positions_sha256"),
        }
        for mission in scheduled
    ] != expected_initial_state["missions"]:
        raise ValueError("development schedule differs from initial state")
    return {
        int(audit.seed): audit for audit in audited.registered.audits
    }


def _read_identity_bound_json(path, identity, label):
    """Authenticate and parse one immutable byte snapshot of a JSON member."""
    path = Path(path)
    if (
        not _exact_file_identity(identity)
        or path.is_symlink()
        or not path.is_file()
    ):
        raise ValueError(f"{label} identity differs")
    try:
        payload = path.read_bytes()
    except OSError as error:
        raise ValueError(f"{label} identity differs") from error
    if (
        len(payload) != identity["bytes"]
        or hashlib.sha256(payload).hexdigest() != identity["sha256"]
    ):
        raise ValueError(f"{label} identity differs")
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError as error:
        raise ValueError(f"{label} is not strict JSON") from error
    return _strict_line_object(text, label)


def _validate_completed_mission_initial_configs(
    mission_root, manifest, mission, initial_audit
):
    """Bind every completed v5 config to one admitted deterministic state."""
    mission_root = Path(mission_root)
    if (
        getattr(initial_audit, "seed", None) != mission.get("trajectory_seed")
        or getattr(initial_audit, "positions_sha256", None)
            != mission.get("initial_positions_sha256")
    ):
        raise ValueError("mission initial position identity differs")
    names = (
        "materialized-primary.json",
        "materialized-dynamic_primary.json",
        "materialized-fixed_fim_ablation.json",
    )
    declared = manifest.get("member_identities")
    documents = {}
    if not isinstance(declared, dict):
        raise ValueError("materialized initial member identities are absent")
    for name in names:
        path = mission_root / name
        identity = declared.get(name)
        documents[name] = _read_identity_bound_json(
            path, identity, f"{name} materialized initial config"
        )
    expected_position = {
        "method": "specified",
        "positions": [list(position) for position in initial_audit.positions],
    }
    initial_subtrees = [document.get("initial") for document in documents.values()]
    observed_position = (
        initial_subtrees[0].get("position")
        if isinstance(initial_subtrees[0], dict) else None
    )
    if (
        not all(isinstance(initial, dict) for initial in initial_subtrees)
        or any(initial != initial_subtrees[0] for initial in initial_subtrees[1:])
        or not isinstance(observed_position, dict)
        or set(observed_position) != {"method", "positions"}
        or observed_position != expected_position
    ):
        raise ValueError("materialized initial state differs from frozen positions")
    return declared["materialized-primary.json"]


def _validate_measurement_config_link(
    measurement_manifest, mission_manifest, primary_identity, mission_id
):
    """Close the primary-config identity chain through Swarm and measurements."""
    expected = (
        primary_identity.get("sha256")
        if _exact_file_identity(primary_identity) else None
    )
    if (
        expected is None
        or measurement_manifest.get("config_sha256") != expected
        or mission_manifest.get("swarm", {}).get("config_sha256") != expected
    ):
        raise ValueError(f"{mission_id} config SHA identity chain differs")


def _validate_swarm_initial_truth(row, initial_audit):
    """Compare both emitted initial-state witnesses with the frozen positions."""
    if initial_audit is None:
        return
    expected = tuple(tuple(position) for position in initial_audit.positions)
    if row.get("record_type") == "initialization":
        robot_id = row.get("robot_id")
        observed = row.get("analyzer_only", {}).get("truth_position")
        if (
            type(robot_id) is not int
            or robot_id not in range(1, 15)
            or not isinstance(observed, list)
            or tuple(observed) != expected[robot_id - 1]
        ):
            raise ValueError("Swarm initialization truth differs from frozen positions")
    elif (
        row.get("record_type") == "controller_interval"
        and row.get("frame_index") == 0
    ):
        observed = row.get("analyzer_only", {}).get("truth")
        if not isinstance(observed, list) or len(observed) != 14:
            raise ValueError("Swarm frame-zero truth differs from frozen positions")
        by_robot = {}
        for item in observed:
            if (
                not isinstance(item, dict)
                or type(item.get("robot_id")) is not int
                or item["robot_id"] not in range(1, 15)
                or item["robot_id"] in by_robot
                or not isinstance(item.get("position"), list)
            ):
                raise ValueError("Swarm frame-zero truth differs from frozen positions")
            by_robot[item["robot_id"]] = tuple(item["position"])
        if by_robot != {
            robot_id: expected[robot_id - 1] for robot_id in range(1, 15)
        }:
            raise ValueError("Swarm frame-zero truth differs from frozen positions")


def _validate_runtime_analyzer_argv(protocol, arguments) -> None:
    actual = _runtime_analyzer_argv(arguments)
    if arguments.kind == "confirmatory-smoke":
        expected = protocol.get("smoke_argv", {}).get(
            arguments.smoke_id, {}
        ).get("analyzer")
    else:
        expected = protocol.get("analyzer_argv")
    if not isinstance(expected, list) or actual != expected:
        raise ValueError("runtime analyzer argv differs from registered protocol")


def _runtime_analyzer_argv(arguments) -> list[str]:
    tokens = [
        "conda", "run", "-n", "cbf_env", "python", "-m",
        "scripts.diagnostics.analyze_qualified_closure_campaign",
        "--kind", arguments.kind,
        "--version", arguments.version,
    ]
    if arguments.kind == "confirmatory-smoke":
        tokens.extend(["--smoke-id", arguments.smoke_id])
    tokens.extend([
        "--protocol", str(arguments.protocol),
        "--authorization", str(arguments.authorization),
        "--input-root", str(arguments.input_root),
    ])
    if arguments.kind == "development":
        tokens.extend(["--ablation-config", str(arguments.ablation_config)])
    tokens.extend(["--output-root", str(arguments.output_root)])
    return tokens


def analyze_campaign_from_arguments(arguments) -> int:
    from scripts.diagnostics.register_qualified_closure_campaign import (
        validate_authorization_binding,
    )
    protocol = _read_json(arguments.protocol, "registered protocol")
    contract = validate_analysis_registration_contract(protocol, arguments)
    authorization = validate_authorization_binding(
        arguments.protocol,
        arguments.authorization,
        allowed_claimed_roots={contract["claimed_root"]},
    )
    if arguments.kind == "development" and not isinstance(
        contract.get("initial_audits"), dict
    ):
        contract["initial_audits"] = _load_development_initial_state_contract(
            protocol
        )
    input_root = Path(arguments.input_root)
    output_root = Path(arguments.output_root)
    validate_analysis_output_root(output_root)
    raw_terminal = _read_json(input_root / "manifest.json", "campaign manifest")
    if raw_terminal.get("terminal") is not True:
        raise ValueError("raw campaign is not terminal")
    output_root.parent.mkdir(parents=True, exist_ok=True)
    output_root.mkdir(mode=0o700, exist_ok=False)
    try:
        return _analyze_claimed_campaign(
            arguments, protocol, contract, authorization, input_root, output_root
        )
    except BaseException as error:
        _terminalize_claimed_analysis_failure(output_root, error)
        raise


def _preflight_raw_campaign(
    input_root, registered_schedule, *, protocol_sha256, authorization_sha256
):
    """Validate terminal raw identities before claiming analysis output."""
    campaign_manifest = _read_json(
        input_root / "manifest.json", "campaign manifest"
    )
    schedule_doc = _read_json(input_root / "schedule.json", "campaign schedule")
    expected_campaign_fields = {
        "schema_version", "terminal", "status", "reason", "mission_count",
        "completed_mission_count", "protocol_sha256", "authorization_sha256",
        "schedule_sha256", "schedule_bytes", "mission_manifest_identities",
    }
    if (
        set(campaign_manifest) != expected_campaign_fields
        or campaign_manifest.get("schema_version")
            != "cbf2026-qualified-campaign-manifest-v1"
        or campaign_manifest.get("terminal") is not True
        or campaign_manifest.get("status") not in {"completed", "failed"}
        or campaign_manifest.get("protocol_sha256") != protocol_sha256
        or campaign_manifest.get("authorization_sha256") != authorization_sha256
        or campaign_manifest.get("schedule_sha256")
            != _sha256(input_root / "schedule.json")
        or campaign_manifest.get("schedule_bytes")
            != (input_root / "schedule.json").stat().st_size
        or type(campaign_manifest.get("mission_count")) is not int
        or campaign_manifest["mission_count"] != len(registered_schedule)
        or type(campaign_manifest.get("completed_mission_count")) is not int
        or not 0 <= campaign_manifest["completed_mission_count"]
            <= campaign_manifest["mission_count"]
        or not isinstance(campaign_manifest.get("reason"), str)
        or not campaign_manifest["reason"]
        or (
            campaign_manifest["status"] == "completed"
            and (
                campaign_manifest["completed_mission_count"]
                    != campaign_manifest["mission_count"]
                or campaign_manifest["reason"] != "completed"
            )
        )
        or (
            campaign_manifest["status"] == "failed"
            and (
                campaign_manifest["completed_mission_count"]
                    >= campaign_manifest["mission_count"]
                or campaign_manifest["reason"] == "completed"
            )
        )
    ):
        raise ValueError("raw campaign manifest identity is invalid")
    if (
        not isinstance(schedule_doc, dict)
        or set(schedule_doc) != {
            "schema_version", "mission_count", "missions"
        }
        or schedule_doc["schema_version"]
            != "cbf2026-qualified-campaign-schedule-v1"
        or type(schedule_doc["mission_count"]) is not int
        or schedule_doc["mission_count"] != len(registered_schedule)
        or not isinstance(schedule_doc["missions"], list)
        or schedule_doc["mission_count"] != len(schedule_doc["missions"])
    ):
        raise ValueError("raw schedule schema is not exact")
    expected_schedule_fields = {
        "campaign_id", "mission_id", "trajectory_seed", "range_noise_seed",
        "frames", "horizon_s", "conditions",
    }
    if registered_schedule and registered_schedule[0].get(
        "campaign_id"
    ) == "development-v5":
        expected_schedule_fields.add("initial_positions_sha256")
    if any(
        not isinstance(row, dict) or set(row) != expected_schedule_fields
        for row in schedule_doc["missions"]
    ):
        raise ValueError("raw schedule mission schema is not exact")
    if schedule_doc["missions"] != registered_schedule:
        raise ValueError("raw schedule differs from registered protocol")
    mission_identities = campaign_manifest.get("mission_manifest_identities")
    if (
        not isinstance(mission_identities, dict)
        or set(mission_identities)
            != {mission["mission_id"] for mission in registered_schedule}
    ):
        raise ValueError("campaign mission-manifest universe is invalid")
    for frozen_mission in registered_schedule:
        registered_mission = frozen_mission
        root = input_root / registered_mission["mission_id"]
        manifest = _read_json(root / "manifest.json", "mission manifest")
        identity = mission_identities[registered_mission["mission_id"]]
        manifest_path = root / "manifest.json"
        if (
            not isinstance(identity, dict)
            or set(identity) != {"sha256", "bytes"}
            or identity["sha256"] != _sha256(manifest_path)
            or identity["bytes"] != manifest_path.stat().st_size
        ):
            raise ValueError("campaign mission-manifest identity differs")
        if not validate_terminal_mission_manifest(manifest):
            raise ValueError("mission manifest schema is not exact and terminal")
        if manifest.get("mission") != frozen_mission:
            raise ValueError("mission manifest identity differs from schedule")
        declared = manifest["member_identities"]
        deferred = set()
        if manifest["status"] == "completed":
            candidates = {
                "swarm.jsonl.gz": manifest["swarm"].get("evidence_sha256"),
                "replay-dynamic_primary.raw.jsonl.gz": manifest["replays"].get(
                    "dynamic_primary", {}
                ).get("sha256"),
                "replay-fixed_fim_ablation.raw.jsonl.gz": manifest["replays"].get(
                    "fixed_fim_ablation", {}
                ).get("sha256"),
                "measurements/runtime.jsonl.gz": manifest["measurements"].get(
                    "runtime_sha256"
                ),
                "measurements/audit.jsonl.gz": manifest["measurements"].get(
                    "audit_sha256"
                ),
            }
            deferred = {
                relative for relative, identity in candidates.items()
                if declared.get(relative, {}).get("sha256") == identity
            }
        _verify_mission_members(root, manifest, deferred_hashes=deferred)


def _analyze_claimed_campaign(
    arguments, protocol, contract, authorization, input_root, output_root
) -> int:
    _preflight_raw_campaign(
        input_root,
        contract["registered_schedule"],
        protocol_sha256=_sha256(arguments.protocol),
        authorization_sha256=_sha256(arguments.authorization),
    )
    campaign_manifest = _read_json(input_root / "manifest.json", "campaign manifest")
    if campaign_manifest.get("terminal") is not True:
        raise ValueError("raw campaign is not terminal")
    schedule_doc = _read_json(input_root / "schedule.json", "campaign schedule")
    registered = contract["registered_schedule"]
    if schedule_doc.get("missions") != registered:
        raise ValueError("raw schedule differs from registered protocol")

    with tempfile.TemporaryDirectory(prefix="qualified-analysis-spool-") as directory:
        spool_root = Path(directory)
        database = sqlite3.connect(spool_root / "analysis.sqlite3")
        try:
            report = _stream_raw_campaign(
                input_root, contract["analysis_protocol"],
                registered, database,
                initial_audits=contract.get("initial_audits"),
                disk_guard=lambda: _enforce_analysis_disk_policy(
                    output_root.parent, spool_root
                ),
            )
        finally:
            database.close()
    report.update({
        "protocol_sha256": _sha256(arguments.protocol),
        "authorization_sha256": _sha256(arguments.authorization),
        "authorization_implementation_identity": authorization["implementation_identity"],
        "raw_campaign_manifest_sha256": _sha256(input_root / "manifest.json"),
        "campaign_kind": arguments.kind,
        "included_in_scientific_denominator": arguments.kind != "confirmatory-smoke",
    })
    report["semantic_analysis_sha256"] = semantic_analysis_sha256(report)
    markdown = _analysis_markdown(report)
    manifest = _publish_claimed_analysis_bundle(output_root, report, markdown)
    return 0 if manifest["status"] == "completed" and report["passed"] else 1


def _stream_raw_campaign(
    input_root, protocol, missions, database, disk_guard=None,
    initial_audits=None,
):
    database.execute(
        "CREATE TABLE truth (mission TEXT, frame INTEGER, robot INTEGER, x REAL, y REAL, PRIMARY KEY(mission,frame,robot))"
    )
    database.execute(
        "CREATE TABLE initialization (mission TEXT, robot INTEGER, "
        "PRIMARY KEY(mission,robot))"
    )
    database.execute(
        "CREATE TABLE initialization_audit (condition TEXT, mission TEXT, "
        "robot INTEGER, public_status TEXT, private_present INTEGER, "
        "PRIMARY KEY(condition,mission,robot))"
    )
    database.execute(
        "CREATE TABLE estimator (condition TEXT, mission TEXT, frame INTEGER, robot INTEGER, depth INTEGER, status TEXT, error REAL, contained INTEGER, bounded INTEGER, admissible INTEGER, wrong INTEGER, private_age INTEGER, source_order_valid INTEGER, PRIMARY KEY(condition,mission,frame,robot))"
    )
    database.execute(
        "CREATE TABLE measurement (mission TEXT, condition TEXT, frame INTEGER, owner INTEGER, ordinal INTEGER, kind TEXT, reference_id INTEGER, noisy_range REAL, ranging_sigma REAL, position_json TEXT, covariance_json TEXT, provenance_json TEXT, PRIMARY KEY(mission,condition,frame,owner,ordinal))"
    )
    counters = Counter()
    incomplete = []
    measurement_hashes = []
    reset_reasons = Counter()
    manifests_complete = 1
    for mission in missions:
        if disk_guard is not None:
            disk_guard()
        mission_id = mission["mission_id"]
        root = input_root / mission_id
        manifest = _read_json(root / "manifest.json", f"{mission_id} manifest")
        if not validate_terminal_mission_manifest(manifest):
            raise ValueError(f"{mission_id} manifest schema is not exact and terminal")
        declared_members = manifest.get("member_identities", {})
        deferred = set()
        candidates = {
            "swarm.jsonl.gz": manifest.get("swarm", {}).get("evidence_sha256"),
            "replay-dynamic_primary.raw.jsonl.gz": manifest.get("replays", {}).get("dynamic_primary", {}).get("sha256"),
            "replay-fixed_fim_ablation.raw.jsonl.gz": manifest.get("replays", {}).get("fixed_fim_ablation", {}).get("sha256"),
        }
        for relative, expected in candidates.items():
            if declared_members.get(relative, {}).get("sha256") == expected:
                deferred.add(relative)
        measurement_declared = manifest.get("measurements", {})
        for relative, field in (
            ("measurements/runtime.jsonl.gz", "runtime_sha256"),
            ("measurements/audit.jsonl.gz", "audit_sha256"),
        ):
            expected = measurement_declared.get(field)
            if declared_members.get(relative, {}).get("sha256") == expected:
                deferred.add(relative)
        _verify_mission_members(root, manifest, deferred_hashes=deferred)
        if manifest.get("status") != "completed":
            _validate_synthesized_missing_stream(
                root / "synthetic-missing.jsonl.gz",
                mission,
                manifest["reason"],
                mission_root=root,
            )
            incomplete.append(mission_id)
            manifests_complete = 0
            continue
        initial_audit = None
        if initial_audits is not None:
            initial_audit = initial_audits.get(mission["trajectory_seed"])
            if initial_audit is None:
                raise ValueError(f"{mission_id} has no frozen initial-state audit")
            primary_config_identity = _validate_completed_mission_initial_configs(
                root, manifest, mission, initial_audit
            )
        else:
            primary_config_identity = manifest.get("member_identities", {}).get(
                "materialized-primary.json"
            )
        swarm_path = root / "swarm.jsonl.gz"
        declared_swarm_sha = manifest.get("swarm", {}).get("evidence_sha256")
        mission_success = _stream_swarm_evidence(
            swarm_path, declared_swarm_sha, mission, database,
            counters, reset_reasons, disk_guard=disk_guard,
            expected_initial_audit=initial_audit,
        )
        measurement_root = root / "measurements"
        measurement_manifest = _read_json(
            measurement_root / "manifest.json", f"{mission_id} measurement manifest"
        )
        if (
            set(measurement_manifest) != {
                "schema_version", "terminal", "status", "truth_sha256",
                "config_sha256", "range_noise_seed", "rng_identity",
                "edge_universe_sha256", "measurement_stream_id",
                "runtime_sha256", "audit_sha256", "row_count",
            }
            or measurement_manifest.get("schema_version")
                != "cbf2026-qualified-measurement-bundle-v1"
            or measurement_manifest.get("terminal") is not True
            or measurement_manifest.get("status") != "completed"
            or measurement_manifest.get("range_noise_seed")
                != mission["range_noise_seed"]
            or measurement_manifest.get("rng_identity")
                != f"numpy.default_rng.PCG64:{mission['range_noise_seed']}"
            or type(measurement_manifest.get("row_count")) is not int
            or measurement_manifest["row_count"] < 0
            or any(
                not _lower_sha256(measurement_manifest.get(field))
                for field in (
                    "truth_sha256", "config_sha256", "edge_universe_sha256",
                    "measurement_stream_id", "runtime_sha256", "audit_sha256",
                )
            )
        ):
            raise ValueError(f"{mission_id} measurement bundle is incomplete")
        for field in (
            "truth_sha256", "config_sha256", "range_noise_seed",
            "measurement_stream_id", "runtime_sha256", "audit_sha256",
            "row_count",
        ):
            if measurement_declared.get(field) != measurement_manifest[field]:
                raise ValueError(
                    f"{mission_id} measurement manifest identity differs"
                )
        if initial_audit is not None:
            _validate_measurement_config_link(
                measurement_manifest,
                manifest,
                primary_config_identity,
                mission_id,
            )
        _stream_measurement_pair(
            measurement_root / "runtime.jsonl.gz",
            measurement_root / "audit.jsonl.gz",
            measurement_manifest,
            mission_id,
            database,
            counters, disk_guard=disk_guard,
        )
        runtime_sha = measurement_manifest["runtime_sha256"]
        measurement_hashes.append(runtime_sha)
        for condition in ("dynamic_primary", "fixed_fim_ablation"):
            replay = manifest.get("replays", {}).get(condition)
            if (
                not isinstance(replay, dict)
                or replay.get("measurement_sha256") != runtime_sha
                or replay.get("campaign_id") != mission["campaign_id"]
                or replay.get("condition") != condition
                or replay.get("trajectory_seed") != mission["trajectory_seed"]
                or replay.get("range_noise_seed") != mission["range_noise_seed"]
                or replay.get("frames") != mission["frames"]
            ):
                raise ValueError(f"{mission_id} replay substituted measurements")
            producer_identities = replay.get("producer_input_identities")
            _validate_producer_namespace_identities(
                producer_identities,
                declared_members=declared_members,
                replay=replay,
                condition=condition,
                measurement_manifest=measurement_manifest,
                runtime_sha=runtime_sha,
                mission_id=mission_id,
            )
            argv = replay.get("producer_argv")
            allowed_inputs = {
                "measurements.jsonl.gz",
                "measurements.manifest.json",
                "commands.jsonl.gz",
                "commands.manifest.json",
                "config.json",
            }
            counters["runtime_truth_read_violation"] += _producer_input_violations(
                argv,
                allowed_inputs,
                condition=condition,
                measurement_sha256=runtime_sha,
                frames=mission["frames"],
            )
            replay_path = root / f"replay-{condition}.raw.jsonl.gz"
            expected_sha = replay.get("sha256")
            replay_publications = {}
            expected_replay_order = iter(
                (frame, robot)
                for frame in range(mission["frames"])
                for robot in range(1, 15)
            )
            initialization_audit_count = 0
            post_initialization_count = 0

            def consume(row, condition=condition, mission_id=mission_id):
                nonlocal initialization_audit_count
                nonlocal post_initialization_count
                if disk_guard is not None and counters["estimator"] % 256 == 0:
                    disk_guard()
                order_key = (row["frame_index"], row["robot_id"])
                if order_key != next(expected_replay_order, None):
                    raise ValueError(
                        "replay differs from the frozen initialization/tuple order"
                    )
                source_order_valid = _validate_replay_reference_provenance(
                    database, mission_id, condition, row, replay_publications
                )
                if row["frame_index"] == 0:
                    _account_initialization_audit(
                        database, mission_id, condition, row
                    )
                    initialization_audit_count += 1
                else:
                    _account_raw_estimator_row(
                        database, mission_id, condition, row, counters,
                        source_order_valid=source_order_valid,
                    )
                    post_initialization_count += 1
                replay_publications[(row["frame_index"], row["robot_id"])] = (
                    row["audit_bundle"]["lifecycle"]["public_output"]
                )

            count = stream_validated_replay_rows(
                replay_path, expected_sha, on_valid_row=consume
            )
            if (
                count != mission["frames"] * 14
                or next(expected_replay_order, None) is not None
                or initialization_audit_count != 14
                or post_initialization_count
                    != max(0, mission["frames"] - 1) * 14
            ):
                raise ValueError(
                    f"{mission_id} replay initialization/tuple universe is incomplete"
                )
            counters["initialization_audit"] += initialization_audit_count
            counters["estimator"] += post_initialization_count
            counters["source_order_checked"] += post_initialization_count
        counters["missions_success"] += mission_success
    database.commit()
    return _report_from_streamed_counts(
        database, protocol, len(missions), counters, manifests_complete,
        incomplete, measurement_hashes, reset_reasons,
    )


def _validate_synthesized_missing_stream(
    path, mission, reason, *, mission_root=None
):
    from scripts.diagnostics.qualified_closure_evidence import (
        FrozenMissionSchedule,
        synthesize_missing_mission,
    )
    from scripts.diagnostics.run_qualified_closure_campaign import (
        _serialize_frozen_key,
    )

    frozen = FrozenMissionSchedule(
        mission["campaign_id"], mission["trajectory_seed"],
        mission["range_noise_seed"], mission["frames"], tuple(range(1, 15)),
        tuple(mission["conditions"]), "dynamic_primary",
    )
    missing = synthesize_missing_mission(frozen, reason)
    observed = _analyzer_observed_failed_mission_keys(
        mission_root, mission
    ) if mission_root is not None else {
        kind: set()
        for kind in (
            "initialization", "estimator", "controller", "endpoint",
            "reconstructed", "reset",
        )
    }
    with gzip.open(path, "rt", encoding="utf-8", errors="strict") as source:
        line_number = 0
        for kind in (
            "initialization", "estimator", "controller", "endpoint",
            "reconstructed", "reset",
        ):
            unmatched_observed = set(observed[kind])
            for expected_key in getattr(missing, kind):
                if expected_key in unmatched_observed:
                    unmatched_observed.remove(expected_key)
                    continue
                line_number += 1
                line = source.readline()
                if not line:
                    raise ValueError("synthetic missing universe is truncated")
                row = _strict_line_object(line, f"synthetic row {line_number}")
                if row != {
                    "record_type": f"missing_{kind}",
                    "key": _serialize_frozen_key(expected_key),
                    "reason": reason,
                }:
                    raise ValueError("synthetic missing key universe differs")
            if unmatched_observed:
                raise ValueError("retained prefix key is outside frozen universe")
        line_number += 1
        terminal = _strict_line_object(
            source.readline(), f"synthetic row {line_number}"
        )
        if terminal != {"record_type": "missing_mission", "reason": reason}:
            raise ValueError("synthetic mission terminal differs")
        if source.readline():
            raise ValueError("synthetic missing universe has trailing rows")


def _analyzer_observed_failed_mission_keys(mission_root, mission):
    """Independently reconstruct retained prefix keys for complement audit."""
    from scripts.diagnostics.analyze_qualified_estimator import (
        validate_and_recompute_qualified_row,
    )
    from scripts.diagnostics.qualified_closure_evidence import (
        CanonicalEdgeId,
        _PAPER_EDGES,
    )
    from scripts.diagnostics.run_qualified_closure_campaign import (
        _SwarmStreamState,
    )

    observed = {
        kind: set()
        for kind in (
            "initialization", "estimator", "controller", "endpoint",
            "reconstructed", "reset",
        )
    }
    swarm_initialization = set()
    replay_initialization = {
        condition: set() for condition in mission["conditions"]
    }
    mission_root = Path(mission_root)
    swarm_path = mission_root / "swarm.jsonl.gz"
    if swarm_path.is_file() and not swarm_path.is_symlink():
        state = _SwarmStreamState(mission)
        with gzip.open(swarm_path, "rt", encoding="utf-8", errors="strict") as source:
            for line_number, line in enumerate(source, start=1):
                row = _strict_line_object(line, f"retained Swarm row {line_number}")
                if state(row) is not True:
                    raise ValueError("retained Swarm prefix is not valid")
                identity = (
                    mission["campaign_id"], row.get("condition"),
                    mission["trajectory_seed"], mission["range_noise_seed"],
                )
                record_type = row["record_type"]
                if record_type == "initialization":
                    key = (
                        mission["campaign_id"], mission["trajectory_seed"],
                        mission["range_noise_seed"], 0, row["robot_id"],
                    )
                    if key in swarm_initialization:
                        raise ValueError("retained prefix key is duplicated")
                    swarm_initialization.add(key)
                elif record_type == "endpoint_row":
                    edge = row["edge"]
                    key = (
                        *identity, row["frame_index"],
                        CanonicalEdgeId(
                            edge["kind"], edge["low"], edge["high"],
                            edge["base_id"],
                        ),
                        row["owner"],
                    )
                    if key in observed["endpoint"]:
                        raise ValueError("retained prefix key is duplicated")
                    observed["endpoint"].add(key)
                elif (
                    record_type == "controller_interval"
                    and row.get("runtime", {}).get("complete") is True
                ):
                    key = (*identity, row["frame_index"])
                    if key in observed["controller"]:
                        raise ValueError("retained prefix key is duplicated")
                    observed["controller"].add(key)
                    observed["reconstructed"].update(
                        (*identity, row["frame_index"], edge)
                        for edge in _PAPER_EDGES
                    )

    for condition in mission["conditions"]:
        replay_path = mission_root / f"replay-{condition}.raw.jsonl.gz"
        if not replay_path.is_file() or replay_path.is_symlink():
            continue
        expected_order = iter(
            (frame, robot)
            for frame in range(mission["frames"])
            for robot in range(1, 15)
        )
        with gzip.open(replay_path, "rt", encoding="utf-8", errors="strict") as source:
            for line_number, line in enumerate(source, start=1):
                row = _strict_line_object(
                    line, f"retained replay row {line_number}"
                )
                validate_and_recompute_qualified_row(row)
                order_key = (row["frame_index"], row["robot_id"])
                if order_key != next(expected_order, None):
                    raise ValueError(
                        "retained replay differs from frozen prefix order"
                    )
                key = (
                    mission["campaign_id"], condition,
                    mission["trajectory_seed"], mission["range_noise_seed"],
                    row["frame_index"], row["robot_id"],
                )
                if row["frame_index"] == 0:
                    replay_initialization[condition].add((
                        mission["campaign_id"], mission["trajectory_seed"],
                        mission["range_noise_seed"], 0, row["robot_id"],
                    ))
                else:
                    observed["estimator"].add(key)
    observed["initialization"] = set(swarm_initialization)
    for condition in mission["conditions"]:
        observed["initialization"].intersection_update(
            replay_initialization[condition]
        )
    return observed


def _stream_swarm_evidence(
    path, expected_sha, mission, database, counters, reset_reasons,
    disk_guard=None, expected_initial_audit=None,
):
    from scripts.diagnostics.run_qualified_closure_campaign import (
        _swarm_row_matches_mission,
    )
    from scripts.diagnostics.qualified_closure_evidence import (
        CanonicalEdgeId,
        _PAPER_ENDPOINTS,
        audit_reset_primitives,
        validate_controller_primitive_schema,
        validate_endpoint_primitive_schema,
        validate_initialization_schema,
        validate_mission_terminal_schema,
    )
    initialization = 0
    terminal = False
    terminal_runtime = None
    previous_controller = -1
    endpoint_frame = None
    endpoint_keys = set()
    endpoint_edges = {}
    endpoint_rows = []
    pending_reset = None
    snapshot_version = 0
    mission_true_violations = 0
    local_counter_names = (
        "certificate_failure", "negative_residual", "input_violation",
        "nu_violation", "qp_infeasible", "deadlock", "reset_violations",
        "hard_duplicate",
    )
    counter_baseline = {
        name: counters[name] for name in local_counter_names
    }
    with Path(path).open("rb") as raw:
        hashing = _HashingReader(raw)
        with (
            gzip.GzipFile(fileobj=hashing, mode="rb") as compressed,
            io.TextIOWrapper(compressed, encoding="utf-8", errors="strict") as source,
        ):
          for line_number, line in enumerate(source, start=1):
            if disk_guard is not None and line_number % 256 == 0:
                disk_guard()
            row = _strict_line_object(line, f"Swarm row {line_number}")
            record_type = row.get("record_type")
            if not _swarm_row_matches_mission(row, mission, record_type):
                raise ValueError("Swarm row identity differs from frozen mission")
            if terminal:
                raise ValueError("row follows mission terminal")
            if record_type == "initialization":
                if not validate_initialization_schema(row) or row["robot_id"] != initialization + 1:
                    raise ValueError("initialization schema/order mismatch")
                _validate_swarm_initial_truth(row, expected_initial_audit)
                initialization += 1
                counters["initialization"] += 1
                database.execute(
                    "INSERT INTO initialization VALUES (?,?)",
                    (mission["mission_id"], row["robot_id"]),
                )
            elif record_type == "reset":
                errors = audit_reset_primitives(row, tuple(range(1, 15)), 119)
                counters["reset_violations"] += bool(errors)
                reset_reasons[row.get("runtime", {}).get("reason", "malformed")] += 1
                runtime = row.get("runtime", {})
                if (
                    errors
                    or pending_reset is not None
                    or row.get("frame_index") != previous_controller + 1
                    or runtime.get("predecessor_version") != snapshot_version
                    or runtime.get("proposed_version") != snapshot_version + 1
                ):
                    raise ValueError("reset lifecycle/version mismatch")
                pending_reset = row
            elif record_type == "endpoint_row":
                if not validate_endpoint_primitive_schema(row):
                    raise ValueError("endpoint schema mismatch")
                frame = row["frame_index"]
                if endpoint_frame is None or endpoint_frame != frame:
                    if endpoint_keys:
                        raise ValueError("endpoint frame ended before controller")
                    endpoint_frame = frame
                edge = row["edge"]
                key = (edge["kind"], edge["low"], edge["high"], edge["base_id"], row["owner"])
                observed_endpoint = (
                    CanonicalEdgeId(
                        edge["kind"], edge["low"], edge["high"], edge["base_id"]
                    ),
                    row["owner"],
                )
                if (
                    len(endpoint_rows) >= len(_PAPER_ENDPOINTS)
                    or observed_endpoint != _PAPER_ENDPOINTS[len(endpoint_rows)]
                ):
                    raise ValueError("endpoint source order mismatch")
                if key in endpoint_keys:
                    counters["hard_duplicate"] += 1
                endpoint_keys.add(key)
                endpoint_edges[(edge["kind"], edge["low"], edge["high"], edge["base_id"])] = edge
                endpoint_rows.append(row)
                counters["endpoint"] += 1
            elif record_type == "controller_interval":
                if (
                    not validate_controller_primitive_schema(row)
                    or row["frame_index"] != previous_controller + 1
                    or endpoint_frame != row["frame_index"]
                    or len(endpoint_keys) != 232
                ):
                    raise ValueError("controller schema/order/universe mismatch")
                _validate_swarm_initial_truth(row, expected_initial_audit)
                reset_runtime = (
                    None if pending_reset is None else pending_reset["runtime"]
                )
                reset_accepted = (
                    reset_runtime is not None
                    and reset_runtime.get("guard_decision") == "accepted"
                )
                expected_snapshot_version = snapshot_version + int(reset_accepted)
                expected_attempted = reset_runtime is not None
                expected_guard = (
                    "accepted" if reset_accepted else
                    "rejected" if expected_attempted else "not-attempted"
                )
                runtime = row["runtime"]
                reset_marker = runtime["reset"]
                if (
                    runtime["snapshot_version"] != expected_snapshot_version
                    or runtime["allocation_version"] != 1
                    or reset_marker["attempted"] != expected_attempted
                    or reset_marker["guard_status"] != expected_guard
                    or reset_marker["committed"] != reset_accepted
                ):
                    raise ValueError("controller reset projection mismatch")
                metrics = _controller_frame_metrics(row, endpoint_rows)
                previous_controller = row["frame_index"]
                snapshot_version = expected_snapshot_version
                pending_reset = None
                counters["controller"] += 1
                available = bool(runtime["complete_finite_snapshot"] and runtime["complete"])
                counters["controller_available"] += available
                if not available:
                    counters["certificate_failure"] += 1
                counters["negative_residual"] += (
                    metrics["local_residual_minimum"] < -1e-7
                    or metrics["reconstructed_residual_minimum"] < -1e-7
                )
                maxima = metrics["component_maxima"]
                counters["input_violation"] += (
                    maxima["vx"] > 25.0 + 1e-7
                    or maxima["vy"] > 25.0 + 1e-7
                    or maxima["yaw_rate"] > 0.35 + 1e-7
                )
                truth = {item["robot_id"]: item["position"] for item in row["analyzer_only"]["truth"]}
                for robot_id, position in truth.items():
                    database.execute(
                        "INSERT INTO truth VALUES (?,?,?,?,?)",
                        (mission["mission_id"], row["frame_index"], robot_id, position[0], position[1]),
                    )
                mission_true_violations += _true_distance_violations(truth, endpoint_edges)
                margins = []
                all_planar_zero = True
                for node in runtime["nodes"]:
                    counters["requested_measurement"] += len(node["references"])
                    reconstructed_node = metrics["nodes"][node["robot_id"]]
                    if available:
                        counters["nu_violation"] += (
                            reconstructed_node["nu_inst"]
                            > reconstructed_node["bar_nu"] + 1e-9
                        )
                        margins.append(
                            reconstructed_node["bar_nu"]
                            - reconstructed_node["nu_inst"]
                        )
                    all_planar_zero = all_planar_zero and max(
                        abs(node["applied_command"][0]), abs(node["applied_command"][1])
                    ) <= 1e-9
                    status = str(node["normal_qp"]["status"]).lower()
                    counters["qp_infeasible"] += "optimal" not in status
                if margins:
                    counters["allocation_stress_near"] += min(margins) <= 1e-6
                    counters["allocation_stress_conservative"] += min(margins) > 1e-6
                counters["deadlock"] += all_planar_zero
                endpoint_keys.clear()
                endpoint_edges.clear()
                endpoint_rows.clear()
                endpoint_frame = None
            elif record_type == "mission_terminal":
                if not validate_mission_terminal_schema(row):
                    raise ValueError("mission-terminal schema mismatch")
                terminal = True
                terminal_runtime = row["runtime"]
            else:
                raise ValueError(f"unknown Swarm record type: {record_type}")
    if initialization != 14 or not terminal or previous_controller + 1 != mission["frames"]:
        raise ValueError("Swarm lifecycle is incomplete")
    if hashing.hexdigest() != expected_sha:
        raise ValueError(f"{mission['mission_id']} Swarm hash differs")
    counters["true_distance_violation"] += mission_true_violations
    local_violations = mission_true_violations + sum(
        counters[name] - counter_baseline[name]
        for name in local_counter_names
    )
    return _derive_mission_success(
        terminal_runtime, mission, local_violations=local_violations
    )


def _derive_mission_success(terminal_runtime, mission, *, local_violations):
    """Derive success from complete evidence, never from the success boolean."""
    return bool(
        isinstance(terminal_runtime, dict)
        and terminal_runtime.get("process_outcome") == "completed"
        and terminal_runtime.get("reason") == "completed"
        and terminal_runtime.get("declared_frames") == mission["frames"]
        and terminal_runtime.get("completed_intervals") == mission["frames"]
        and type(local_violations) is int
        and local_violations == 0
    )


def _controller_frame_metrics(controller, endpoint_rows):
    """Return controller metrics reconstructed from primitive evidence."""
    from scripts.diagnostics.qualified_closure_evidence import (
        reconstruct_controller_primitives,
    )

    runtime = controller["runtime"]
    _verify_controller_interior_evidence(runtime["nodes"])
    reconstruction = reconstruct_controller_primitives(
        controller,
        endpoint_rows,
        expected_endpoint_count=232,
        expected_reconstructed_count=119,
    )
    if reconstruction.integrity_errors:
        raise ValueError(
            "controller primitive reconstruction failed: "
            + ",".join(reconstruction.integrity_errors[:8])
        )
    commands = [node["applied_command"] for node in runtime["nodes"]]
    return {
        "local_residual_minimum": min(reconstruction.local_residuals.values()),
        "reconstructed_residual_minimum": min(
            reconstruction.full_residuals.values()
        ),
        "component_maxima": {
            "vx": max(abs(command[0]) for command in commands),
            "vy": max(abs(command[1]) for command in commands),
            "yaw_rate": max(abs(command[2]) for command in commands),
        },
        "nodes": reconstruction.nodes,
    }


def _verify_controller_interior_evidence(nodes):
    """Recompute every local interior policy before aggregating a frame."""
    from scripts.diagnostics.hard_interior_selection import (
        frozen_interior_floor,
        solve_planar_hard_row_chebyshev,
    )

    policy_presence = ["hard_interior_selection" in node for node in nodes]
    if not any(policy_presence):
        return
    if not all(policy_presence):
        raise ValueError("controller interior policy provenance differs")
    for node in nodes:
        if "normal_problem" not in node:
            raise ValueError("controller interior normal problem is absent")
        policy = node.get("hard_interior_selection")
        required = {
            "mode", "fraction", "cap_mps", "feasibility_tolerance_mps",
            "planar_chebyshev_radius_mps", "enforced_floor_mps",
            "minimum_original_hard_residual_mps",
        }
        if not isinstance(policy, dict) or set(policy) != required:
            raise ValueError("controller interior policy schema differs")
        continuous = (
            "fraction", "cap_mps", "feasibility_tolerance_mps",
            "planar_chebyshev_radius_mps", "enforced_floor_mps",
            "minimum_original_hard_residual_mps",
        )
        if not all(
            type(policy[field]) is float and math.isfinite(policy[field])
            for field in continuous
        ):
            raise ValueError("controller interior policy numeric token differs")
        if (policy["mode"] != "planar-chebyshev-fraction-cap-v1"
                or (policy["fraction"], policy["cap_mps"],
                    policy["feasibility_tolerance_mps"]) != (0.1, 0.1, 1e-9)):
            raise ValueError("controller interior policy differs")
        audit = solve_planar_hard_row_chebyshev(
            node["normal_problem"],
            tolerance_mps=policy["feasibility_tolerance_mps"],
        )
        floor = frozen_interior_floor(
            audit.radius_mps,
            fraction=policy["fraction"],
            cap_mps=policy["cap_mps"],
            tolerance_mps=policy["feasibility_tolerance_mps"],
        )
        command = node["applied_command"]
        original_residual = min(
            row["constant"] + sum(
                coefficient * value
                for coefficient, value in zip(row["coefficients"], command)
            )
            for row in node["normal_problem"]["rows"]
        )
        if (not math.isclose(policy["planar_chebyshev_radius_mps"], audit.radius_mps,
                             rel_tol=0.0, abs_tol=1e-12)
                or not math.isclose(policy["enforced_floor_mps"], floor,
                                    rel_tol=0.0, abs_tol=1e-12)
                or not math.isclose(policy["minimum_original_hard_residual_mps"],
                                    original_residual, rel_tol=1e-9, abs_tol=1e-9)
                or original_residual < floor - 1e-7):
            raise ValueError("controller interior policy reconstruction failed")


def _validate_replay_reference_provenance(
    database, mission_id, condition, row, publications
):
    """Bind raw replay sources to frozen measurements and condition state."""
    records = database.execute(
        "SELECT kind,reference_id,noisy_range,ranging_sigma,position_json,"
        "covariance_json,provenance_json FROM measurement "
        "WHERE mission=? AND condition=? AND frame=? AND owner=? "
        "ORDER BY ordinal",
        (mission_id, condition, row["frame_index"], row["robot_id"]),
    ).fetchall()
    expected = []
    for (
        kind, reference_id, noisy_range, ranging_sigma, position_json,
        covariance_json, provenance_json,
    ) in records:
        if kind == "base":
            position = json.loads(position_json)
            covariance = json.loads(covariance_json)
            provenance = json.loads(provenance_json)
        else:
            public = publications.get((row["frame_index"], reference_id))
            if not isinstance(public, dict) or public.get("output_status") == "unavailable":
                continue
            if public.get("output_status") not in {"fresh", "predicted"}:
                raise ValueError("condition predecessor publication is invalid")
            position = public["estimate"]
            covariance = public["modeled_covariance"]
            provenance = public["base_anchor_provenance"]
            if len(provenance) < 2:
                continue
        expected.append({
            "key": [kind, reference_id],
            "position": position,
            "range": noisy_range,
            "covariance": covariance,
            "ranging_sigma": ranging_sigma,
            "base_anchor_provenance": provenance,
        })
    observed = row["runtime_inputs"]["references"]
    if not isinstance(observed, list) or any(
        reference.get("key", [None, None])[0] == "uav"
        and (
            type(reference["key"][1]) is not int
            or reference["key"][1] >= row["robot_id"]
        )
        for reference in observed
        if isinstance(reference, dict)
    ):
        raise ValueError("replay reference order/topology is invalid")
    if observed != expected:
        raise ValueError(
            "replay reference source/provenance differs from condition-local inputs"
        )
    return True


def _account_raw_estimator_row(
    database, mission_id, condition, row, counters, *, source_order_valid
):
    public = row["audit_bundle"]["lifecycle"]["public_output"]
    status = public["output_status"]
    truth = database.execute(
        "SELECT x,y FROM truth WHERE mission=? AND frame=? AND robot=?",
        (mission_id, row["frame_index"], row["robot_id"]),
    ).fetchone()
    error = None
    contained = 0
    if status in {"fresh", "predicted"} and truth is not None:
        error = math.dist(public["estimate"], truth)
        radius = public["epsilon"] if status == "fresh" else public["aged_modeled_radius"]
        contained = int(error <= radius)
    wrong = int(_wrong_mode_from_truth(row, truth)) if status == "fresh" and truth else 0
    database.execute(
        "INSERT INTO estimator VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)",
        (
            condition,
            mission_id,
            row["frame_index"],
            row["robot_id"],
            row["squad_local_index"],
            status,
            error,
            contained,
            int(status == "predicted" and public["aged_modeled_radius"] is not None),
            row["admissible_mode_count"],
            wrong,
            row["private_age"],
            int(source_order_valid is True),
        ),
    )


def _account_initialization_audit(database, mission_id, condition, row):
    """Bind one independently reconstructed frame-zero audit to its key."""
    if (
        row.get("frame_index") != 0
        or row.get("qualification_kind") != "deployment"
    ):
        raise ValueError("qualified initialization audit is not frame zero")
    lifecycle = row["audit_bundle"]["lifecycle"]
    public = lifecycle["public_output"]
    private = lifecycle["next_private_state"]
    database.execute(
        "INSERT INTO initialization_audit VALUES (?,?,?,?,?)",
        (
            condition,
            mission_id,
            row["robot_id"],
            public["output_status"],
            int(private is not None),
        ),
    )


def _wrong_mode_from_truth(row, truth):
    if truth is None or row["published_mode_id"] is None:
        return False
    audit = row["audit_bundle"]
    representatives = {item["attempt_id"]: item for item in audit["representatives"]}
    candidates = []
    for mode in audit["clustering"]["modes"]:
        representative = next(
            (representatives[member] for member in mode["member_ids"] if member in representatives),
            None,
        )
        if representative is not None:
            candidates.append((math.dist(representative["estimate"], truth), mode["mode_id"]))
    return bool(candidates and min(candidates)[1] != row["published_mode_id"])


def _true_distance_violations(truth, edges):
    violations = 0
    for (_kind, low, high, base_id), edge in edges.items():
        first = truth[low]
        second = edge["base_position"] if base_id >= 0 else truth[high]
        distance = math.dist(first, second)
        if edge["kind"] == "localization":
            violations += distance > edge["threshold"] + 1e-9
        else:
            violations += distance < edge["threshold"] - 1e-9
    return violations


def _stream_measurement_pair(
    runtime_path, audit_path, manifest, mission_id, database, counters,
    disk_guard=None,
):
    ordinals = Counter()

    def consume(runtime, audit):
        if disk_guard is not None and counters["measurement"] % 256 == 0:
            disk_guard()
        endpoint = database.execute(
            "SELECT x,y FROM truth WHERE mission=? AND frame=? AND robot=?",
            (mission_id, runtime["frame_index"], runtime["owner_id"]),
        ).fetchone()
        if endpoint is None or not all(
            math.isclose(audit["truth_endpoint_position"][index], endpoint[index], rel_tol=0.0, abs_tol=1e-12)
            for index in range(2)
        ):
            raise ValueError("measurement endpoint truth differs from controller truth")
        kind, identifier = runtime["reference_key"]
        if kind == "uav":
            reference = database.execute(
                "SELECT x,y FROM truth WHERE mission=? AND frame=? AND robot=?",
                (mission_id, runtime["frame_index"], identifier),
            ).fetchone()
            if reference is None or not all(
                math.isclose(audit["truth_reference_position"][index], reference[index], rel_tol=0.0, abs_tol=1e-12)
                for index in range(2)
            ):
                raise ValueError("measurement reference truth differs from controller truth")
        for condition in runtime["role_tags"]:
            ordinal_key = (
                condition, runtime["frame_index"], runtime["owner_id"]
            )
            ordinal = ordinals[ordinal_key]
            ordinals[ordinal_key] += 1
            database.execute(
                "INSERT INTO measurement VALUES (?,?,?,?,?,?,?,?,?,?,?,?)",
                (
                    mission_id, condition, runtime["frame_index"],
                    runtime["owner_id"], ordinal, kind, identifier,
                    runtime["noisy_range"], runtime["ranging_sigma"],
                    json.dumps(runtime["reference_position"], separators=(",", ":")),
                    json.dumps(runtime["reference_covariance"], separators=(",", ":")),
                    json.dumps(runtime["base_anchor_provenance"], separators=(",", ":")),
                ),
            )
        counters["measurement"] += 1

    summary = validate_measurement_pair_streams(
        runtime_path,
        audit_path,
        runtime_sha256=manifest["runtime_sha256"],
        audit_sha256=manifest["audit_sha256"],
        on_pair=consume,
        config_sha256=manifest["config_sha256"],
        measurement_stream_id=manifest["measurement_stream_id"],
        expected_row_count=manifest["row_count"],
        range_noise_seed=manifest["range_noise_seed"],
    )
    counters["runtime_truth_read_violation"] += summary["runtime_truth_read_count"]
    return summary


def _producer_input_violations(
    argv,
    allowed_inputs,
    *,
    condition,
    measurement_sha256,
    frames,
):
    if not isinstance(argv, list) or not all(isinstance(token, str) for token in argv):
        return 1
    forbidden = ("truth", "audit", "noiseless", "sampled_noise")
    if any(fragment in token.lower() for token in argv for fragment in forbidden):
        return 1
    expected_flags = {
        "--condition", "--measurements", "--measurement-sha256",
        "--measurement-manifest", "--commands", "--commands-manifest",
        "--config", "--frames",
    }
    expected_prefix = [
        sys.executable,
        str(
            Path(__file__).with_name(
                "replay_qualified_estimator.py"
            ).resolve()
        ),
    ]
    if argv[:2] != expected_prefix or len(argv) != 18:
        return 1
    parsed = {}
    for index in range(2, len(argv), 2):
        flag, value = argv[index:index + 2]
        if flag in parsed:
            return 1
        parsed[flag] = value
    if set(parsed) != expected_flags:
        return 1
    return int(
        {
            parsed["--measurements"], parsed["--measurement-manifest"],
            parsed["--commands"], parsed["--commands-manifest"],
            parsed["--config"],
        } != allowed_inputs
        or parsed["--condition"] != condition
        or parsed["--measurement-sha256"] != measurement_sha256
        or parsed["--frames"] != str(frames)
    )


def _report_from_streamed_counts(
    database, protocol, mission_count, counters, manifests_complete,
    incomplete, measurement_hashes, reset_reasons,
):
    universes = protocol["universes"]
    expected_estimator = universes["estimator_total"]
    expected_primary = universes["estimator_per_condition"]
    initialization_complete = database.execute(
        "SELECT COUNT(*) FROM initialization AS base "
        "WHERE EXISTS ("
        "SELECT 1 FROM initialization_audit AS dynamic "
        "WHERE dynamic.mission=base.mission "
        "AND dynamic.robot=base.robot "
        "AND dynamic.condition='dynamic_primary') "
        "AND EXISTS ("
        "SELECT 1 FROM initialization_audit AS fixed "
        "WHERE fixed.mission=base.mission "
        "AND fixed.robot=base.robot "
        "AND fixed.condition='fixed_fim_ablation')"
    ).fetchone()[0]
    complete = (
        manifests_complete
        and initialization_complete == universes["initialization"]
        and counters["initialization_audit"]
            == universes["initialization"] * 2
        and counters["controller"] == universes["controller"]
        and counters["endpoint"] == universes["endpoint"]
        and counters["estimator"] == expected_estimator
        and counters["measurement"] == counters["requested_measurement"]
    )
    scalar = lambda query: database.execute(query).fetchone()[0] or 0
    available = scalar("SELECT COUNT(*) FROM estimator WHERE condition='dynamic_primary' AND status IN ('fresh','predicted')")
    contained = scalar("SELECT COUNT(*) FROM estimator WHERE condition='dynamic_primary' AND status IN ('fresh','predicted') AND contained=1")
    finite_count = scalar("SELECT COUNT(*) FROM estimator WHERE condition='dynamic_primary' AND error IS NOT NULL")
    gates = {
        "complete_keys_and_manifests": _ratio_gate(int(complete), 1, 1.0),
        "zero_runtime_truth_reads": _zero_gate(
            counters["runtime_truth_read_violation"], max(1, expected_estimator)
        ),
        "zero_initialization_accounting_omissions": _zero_gate(
            max(0, universes["initialization"] - initialization_complete),
            universes["initialization"],
        ),
        "zero_cross_mode_source_order_publication": _zero_gate(
            _cross_mode_key_mismatch_count(database)
            + scalar(
                "SELECT COUNT(*) FROM estimator WHERE source_order_valid != 1"
            ),
            counters["estimator"],
        ),
        "zero_fresh_publication_with_multiple_admissible_modes": _zero_gate(
            scalar("SELECT COUNT(*) FROM estimator WHERE condition='dynamic_primary' AND status='fresh' AND admissible != 1"),
            expected_primary,
        ),
        "zero_wrong_mode_fresh_publication": _zero_gate(
            scalar("SELECT SUM(wrong) FROM estimator WHERE condition='dynamic_primary'"), expected_primary
        ),
        "zero_finite_error_above_50m": _zero_gate(
            scalar("SELECT COUNT(*) FROM estimator WHERE condition='dynamic_primary' AND error > 50.0"), finite_count
        ),
        "aggregate_containment": _ratio_gate(contained, available, 0.98),
        "full_universe_joint_available_and_contained": _ratio_gate(
            contained, expected_primary, 0.93
        ),
        "fresh_retention": _ratio_gate(
            scalar(
                "SELECT COUNT(*) FROM estimator "
                "WHERE condition='dynamic_primary' AND status='fresh'"
            ),
            expected_primary,
            0.98,
        ),
        "fresh_or_bounded_predicted_availability": _ratio_gate(
            scalar("SELECT COUNT(*) FROM estimator WHERE condition='dynamic_primary' AND (status='fresh' OR (status='predicted' AND bounded=1))"),
            expected_primary,
            0.95,
        ),
        "zero_missing_duplicate_hard_rows": _zero_gate(
            max(0, universes["endpoint"] - counters["endpoint"])
            + counters["hard_duplicate"],
            universes["endpoint"],
        ),
        "controller_certificate_availability": _ratio_gate(
            counters["controller_available"], universes["controller"], 0.99
        ),
        "zero_nu_bound_excess": _zero_gate(
            counters["nu_violation"], counters["controller_available"] * 14
        ),
        "zero_accepted_reset_violations": _zero_gate(
            counters["reset_violations"], max(1, sum(reset_reasons.values()))
        ),
        "zero_input_limit_violations": _zero_gate(
            counters["input_violation"], universes["controller"]
        ),
        "zero_primary_local_hard_qp_infeasibility": _zero_gate(
            counters["qp_infeasible"], universes["controller"] * 14
        ),
        "zero_negative_hard_residual": _zero_gate(
            counters["negative_residual"], universes["controller"]
        ),
        "zero_true_distance_violations": _zero_gate(
            counters["true_distance_violation"], universes["reconstructed"]
        ),
        "successful_mission_fraction": _ratio_gate(
            counters["missions_success"], mission_count, 0.95
        ),
        "identical_immutable_measurements": _ratio_gate(
            int(len(measurement_hashes) == counters["missions_success"]), 1, 1.0
        ),
    }
    depth_results = {}
    depth_passes = 0
    depth_count = 0
    for condition in ("dynamic_primary", "fixed_fim_ablation"):
        for depth in range(1, 8):
            numerator, denominator = database.execute(
                "SELECT COALESCE(SUM(contained),0), COUNT(*) FROM estimator WHERE condition=? AND depth=? AND status IN ('fresh','predicted')",
                (condition, depth),
            ).fetchone()
            gate = _ratio_gate(numerator, denominator, 0.95)
            depth_results[f"{condition}:depth-{depth}"] = gate
            if condition == "dynamic_primary":
                depth_passes += gate["passed"]
                depth_count += 1
    gates["every_registered_depth_containment"] = _ratio_gate(
        depth_passes, depth_count, 1.0
    )
    errors = _sql_distribution(database, ("dynamic_primary", None))
    depth_errors = {}
    for condition in ("dynamic_primary", "fixed_fim_ablation"):
        for depth in range(1, 8):
            depth_errors[f"{condition}:depth-{depth}"] = _sql_distribution(
                database, (condition, depth)
            )
    ages = {
        str(age): count
        for age, count in database.execute(
            "SELECT private_age,COUNT(*) FROM estimator GROUP BY private_age ORDER BY private_age"
        )
    }
    return {
        "schema_version": "cbf2026-qualified-closure-analysis-v1",
        "passed": all(gate["passed"] for gate in gates.values()),
        "gates": gates,
        "depth_containment": depth_results,
        "errors_m": errors,
        "ablation": {
            "row_count": scalar(
                "SELECT COUNT(*) FROM estimator WHERE condition='fixed_fim_ablation'"
            ),
            "finite_error_above_50m": scalar(
                "SELECT COUNT(*) FROM estimator WHERE condition='fixed_fim_ablation' AND error > 50.0"
            ),
            "errors_m": _sql_distribution(
                database, ("fixed_fim_ablation", None)
            ),
        },
        "depth_errors_m": depth_errors,
        "private_age_strata": ages,
        "reset_reasons": dict(sorted(reset_reasons.items())),
        "certificate_failures": {"count": counters["certificate_failure"]},
        "allocation_conservatism_stress": {
            "near_bound": counters["allocation_stress_near"],
            "conservative": counters["allocation_stress_conservative"],
        },
        "deadlocks": {"count": counters["deadlock"]},
        "incomplete_missions": incomplete,
    }


def _cross_mode_key_mismatch_count(database) -> int:
    query = """
        SELECT COUNT(*) FROM (
            SELECT dynamic.mission, dynamic.frame, dynamic.robot
            FROM estimator AS dynamic
            WHERE dynamic.condition = 'dynamic_primary'
              AND NOT EXISTS (
                  SELECT 1 FROM estimator AS fixed
                  WHERE fixed.condition = 'fixed_fim_ablation'
                    AND fixed.mission = dynamic.mission
                    AND fixed.frame = dynamic.frame
                    AND fixed.robot = dynamic.robot
              )
            UNION ALL
            SELECT fixed.mission, fixed.frame, fixed.robot
            FROM estimator AS fixed
            WHERE fixed.condition = 'fixed_fim_ablation'
              AND NOT EXISTS (
                  SELECT 1 FROM estimator AS dynamic
                  WHERE dynamic.condition = 'dynamic_primary'
                    AND dynamic.mission = fixed.mission
                    AND dynamic.frame = fixed.frame
                    AND dynamic.robot = fixed.robot
              )
        )
    """
    return int(database.execute(query).fetchone()[0])


def _sql_distribution(database, selector):
    where, parameters = "error IS NOT NULL", ()
    if selector is not None:
        condition, depth = selector
        where += " AND condition=?"
        parameters = (condition,)
        if depth is not None:
            where += " AND depth=?"
            parameters += (depth,)
    count, maximum = database.execute(
        f"SELECT COUNT(*),MAX(error) FROM estimator WHERE {where}", parameters
    ).fetchone()
    def quantile(probability):
        if not count:
            return None
        position = (count - 1) * probability
        low, high = math.floor(position), math.ceil(position)
        values = [
            row[0] for row in database.execute(
                f"SELECT error FROM estimator WHERE {where} ORDER BY error LIMIT ? OFFSET ?",
                (*parameters, high - low + 1, low),
            )
        ]
        if low == high:
            return values[0]
        return values[0] + (values[-1] - values[0]) * (position - low)
    return {"count": count, "max": maximum, "p95": quantile(0.95), "p99": quantile(0.99)}


def _strict_line_object(line, label):
    def reject_duplicate_keys(pairs):
        value = {}
        for key, item in pairs:
            if key in value:
                raise ValueError(f"{label} contains duplicate JSON key: {key}")
            value[key] = item
        return value

    try:
        value = json.loads(
            line,
            parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)),
            object_pairs_hook=reject_duplicate_keys,
        )
    except (json.JSONDecodeError, ValueError) as error:
        raise ValueError(f"{label} is not strict JSON") from error
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object")
    return value


def _read_json(path, label):
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"{label} must be a regular file")
    return _strict_line_object(path.read_text(encoding="utf-8"), label)


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _analysis_markdown(report):
    outcome = "PASS" if report["passed"] else "FAIL"
    lines = [f"# Qualified Closure Analysis: {outcome}", ""]
    for name, gate in sorted(report["gates"].items()):
        lines.append(
            f"- `{name}`: {gate['pass_fail']} ({gate['numerator']}/{gate['denominator']} {gate['comparison']} {gate['threshold']})"
        )
    return "\n".join(lines) + "\n"


def _verify_mission_members(root: Path, manifest: dict, *, deferred_hashes=()) -> None:
    declared = manifest.get("member_identities")
    if not isinstance(declared, dict):
        raise ValueError(f"{root.name} member manifest is absent")
    actual_paths = {}
    for path in sorted(root.rglob("*")):
        if path.is_symlink():
            raise ValueError(f"{root.name} contains a symbolic member")
        if not path.is_file() or path == root / "manifest.json":
            continue
        actual_paths[str(path.relative_to(root))] = path
    if set(actual_paths) != set(declared):
        raise ValueError(f"{root.name} member universe differs from manifest")
    deferred_hashes = set(deferred_hashes)
    if not deferred_hashes <= set(actual_paths):
        deferred_hashes &= set(actual_paths)
    for relative, path in actual_paths.items():
        identity = declared[relative]
        if (
            not isinstance(identity, dict)
            or set(identity) != {"sha256", "bytes"}
            or identity["bytes"] != path.stat().st_size
            or (
                relative not in deferred_hashes
                and identity["sha256"] != _sha256(path)
            )
        ):
            raise ValueError(f"{root.name} member identity differs: {relative}")


if __name__ == "__main__":
    raise SystemExit(main())
