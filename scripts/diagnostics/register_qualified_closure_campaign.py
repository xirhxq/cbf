"""Register immutable qualified-closure campaign protocols."""

import hashlib
import argparse
import json
import os
import uuid
import sys
import subprocess
from collections.abc import Mapping
from datetime import date
from pathlib import Path


FROZEN_THRESHOLDS = {
    "aggregate_containment": 0.98,
    "depth_containment": 0.95,
    "joint_available_contained": 0.93,
    "fresh_retention": 0.98,
    "availability": 0.95,
    "controller_certificate_availability": 0.99,
    "mission_success_fraction": 0.95,
    "maximum_finite_error_m": 50.0,
    "nu_tolerance": 1e-9,
    "input_limit_tolerance": 1e-7,
    "residual_tolerance": -1e-7,
    "start_free_bytes": 8_000_000_000,
    "stop_free_bytes": 6_000_000_000,
    "cache_cap_bytes": 2_000_000_000,
    "compact_cap_bytes": 25_000_000,
}

FROZEN_EXECUTION_ROOTS = {
    "development": {
        "raw": "/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v5",
        "analysis": "/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v5",
    },
    "confirmatory": {
        "smoke_a_raw": "/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-a",
        "smoke_a_analysis": "/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-a-analysis",
        "smoke_b_raw": "/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-b",
        "smoke_b_analysis": "/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-smoke-v1-b-analysis",
        "raw": "/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory/v1",
        "analysis": "/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-confirmatory-analysis/v1",
    },
}

FROZEN_SUPERVISION = {
    "wallclock_timeout_s": 3600.0,
    "line_stall_timeout_s": 300.0,
    "termination_grace_s": 5.0,
}

FROZEN_AUTHORIZATION_REQUIREMENTS = {
    "required": True,
    "must_bind_protocol_sha256": True,
    "must_bind_implementation_identity": True,
    "must_bind_preflight_sha256": True,
    "must_bind_user_authorization": True,
}

SUPPORTED_PROTOCOL_VERSIONS = {
    "development": "v5",
    "confirmatory": "v1",
}

FROZEN_INITIAL_FAMILY_PATH = (
    Path(__file__).resolve().parents[2]
    / "config" / "diagnostics" / "qualified_initial_family_v1.json"
)

FROZEN_V6_PREDECESSOR_IDENTITY_PATH = (
    Path(__file__).resolve().parents[2]
    / "config" / "diagnostics" / "qualified_v6_predecessor_v5_identity_v1.json"
)
FROZEN_V6_PREDECESSOR_IDENTITY_RELATIVE_PATH = (
    "config/diagnostics/qualified_v6_predecessor_v5_identity_v1.json"
)

FROZEN_V6_PREDECESSOR_V5_IDENTITY = {
    "schema_version": "cbf2026-qualified-v6-predecessor-v5-identity-v1",
    "terminal": {
        "raw": {
            "root": "/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v5",
            "files": 28,
            "logical_bytes": 12764382,
            "tree_sha256": "7ead4049e0d13f63458ba8d1a522f055ffed56357ffa1719e1bfc1da38a66870",
            "manifest_name": "manifest.json",
            "manifest_sha256": "a3970bcf588e938f8487a794b00d1bbbe8bbd181c0d5a919cc0297b258e64a32",
        },
        "analysis": {
            "root": "/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v5",
            "files": 1,
            "logical_bytes": 176,
            "tree_sha256": "e42ce3e1cd83c60366b2901e3adec58a4282cfc70df7f9b13609a16f600020f5",
            "manifest_name": "manifest.json",
            "manifest_sha256": "82457d6a4838b2396b16fa36d99b15b6ccd0ab16823cfbf486be51613e862a63",
        },
    },
}


def _require_literal_initial_family_path(path: Path) -> Path:
    """Authorize only the frozen family at its literal non-symbolic path."""
    path = Path(path)
    if (
        path.is_symlink()
        or _has_symbolic_ancestor(path)
        or path.resolve() != FROZEN_INITIAL_FAMILY_PATH.resolve()
    ):
        raise ValueError("development requires the literal initial-family path")
    return path


FROZEN_DEVELOPMENT_PREDECESSOR_STATE = {
    "absent": tuple(
        Path(f"/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development{suffix}/v{version}")
        for version in range(1, 4)
        for suffix in ("", "-analysis")
    ),
    "terminal": {
        "raw": {
            "root": "/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v4",
            "files": 28,
            "logical_bytes": 12618979,
            "tree_sha256": "9b915ad84aeda2b22aafbd259bf61bbc9515e3b082559bf3e5f217e14af9b5ac",
            "manifest_name": "manifest.json",
            "manifest_sha256": "12f71bb00720636ca610ad8d9d380dde7e92147e7bdf57589bb8915c1bf72f71",
        },
        "analysis": {
            "root": "/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v4",
            "files": 3,
            "logical_bytes": 8639,
            "tree_sha256": "155b6b9372f91bb7b4b2476581b4270e7a372c680cd8b2ea1847ef9c5766729a",
            "manifest_name": "manifest.json",
            "manifest_sha256": "890174908e5df83e367edd33cb0d83960635cc94ac57fd79fafd553aef40e6f0",
        },
    },
}

AUTHORIZATION_FIELDS = {
    "schema_version", "authorized", "kind", "version",
    "protocol_sha256", "implementation_identity", "preflight_sha256",
    "user_authorization_date", "user_authorization_text",
    "user_authorization_text_sha256",
}

_INITIAL_AUDIT_CACHE = {}


def build_qualified_closure_protocol(**kwargs) -> dict:
    """Validate and bind one development or confirmatory protocol."""
    kind = kwargs.get("kind")
    version = kwargs.get("version")
    if kind not in SUPPORTED_PROTOCOL_VERSIONS:
        raise ValueError("only registered development/confirmatory protocols are supported")
    if version != SUPPORTED_PROTOCOL_VERSIONS[kind]:
        raise ValueError(
            "development requires v5 and confirmatory requires v1"
        )
    project_root = Path(kwargs["project_root"]).resolve()
    trajectory = _seed_list(kwargs.get("trajectory_seeds"), "trajectory")
    ranges = _seed_list(kwargs.get("range_noise_seeds"), "range-noise")
    expected_count = 10 if kind == "development" else 60
    if len(trajectory) != expected_count or len(ranges) != expected_count:
        raise ValueError(f"{kind} requires exactly {expected_count} seed pairs")
    if len(set(trajectory)) != len(trajectory) or len(set(ranges)) != len(ranges):
        raise ValueError("each registered seed list must be distinct")
    if set(trajectory) & set(ranges):
        raise ValueError("trajectory and range-noise seed lists must be disjoint")
    development = set(range(2026080201, 2026080211)) | set(
        range(2026081201, 2026081211)
    )
    if kind == "confirmatory" and (set(trajectory) | set(ranges)) & development:
        raise ValueError("confirmatory seeds must be development-seed disjoint")
    expected_trajectory = list(
        range(2026080201, 2026080211)
        if kind == "development"
        else range(2026082001, 2026082061)
    )
    expected_ranges = list(
        range(2026081201, 2026081211)
        if kind == "development"
        else range(2026083001, 2026083061)
    )
    if trajectory != expected_trajectory or ranges != expected_ranges:
        raise ValueError(f"{kind} seed schedule does not match the frozen sequence")
    frames = kwargs.get("frames")
    if frames != 1000 or type(frames) is not int:
        raise ValueError("scientific campaigns require exactly 1000 frames")
    if kwargs.get("dirty_relevant_paths"):
        raise ValueError("dirty relevant source is not registrable")
    thresholds = kwargs.get("thresholds")
    if _canonical_bytes(thresholds) != _canonical_bytes(FROZEN_THRESHOLDS):
        raise ValueError("threshold binding does not match the frozen contract")

    bindings = kwargs.get("bindings")
    expected_bindings = {
        "source", "binary", "base_config", "primary_config",
        "ablation_config", "dependencies", "schema",
    }
    if kind == "development":
        expected_bindings.add("initial_family")
    if not isinstance(bindings, dict) or set(bindings) != expected_bindings:
        raise ValueError("source/config/binary/dependency/schema bindings are incomplete")
    if kind == "development":
        _require_literal_initial_family_path(bindings["initial_family"])
    bound = {}
    for label in sorted(bindings):
        path = Path(bindings[label])
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"{label} binding must be a regular file")
        bound[label] = {
            "path": str(path.resolve()),
            "sha256": _sha256(path),
            "bytes": path.stat().st_size,
        }
    primary = _json_object(Path(bindings["primary_config"]), "primary config")
    ablation = _json_object(Path(bindings["ablation_config"]), "ablation config")
    if _reference_selection(primary) != "dynamic-lower-index":
        raise ValueError("primary config must select dynamic-lower-index references")
    if _reference_selection(ablation) != "fixed-cbf-only":
        raise ValueError("ablation config must select fixed-cbf-only references")

    roots = kwargs.get("roots")
    expected_roots = (
        {"raw", "analysis"}
        if kind == "development"
        else {
            "smoke_a_raw", "smoke_a_analysis", "smoke_b_raw",
            "smoke_b_analysis", "raw", "analysis",
        }
    )
    if not isinstance(roots, dict) or set(roots) != expected_roots:
        raise ValueError("execution root binding is incomplete")
    bound_roots = {}
    resolved_roots = set()
    for label in sorted(roots):
        path = Path(roots[label])
        if path.is_symlink():
            raise ValueError(f"{label} root must not be symbolic")
        if _has_symbolic_ancestor(path):
            raise ValueError(f"{label} root has a symbolic ancestor")
        if path.exists():
            raise FileExistsError(f"{label} root must be absent")
        resolved = path.parent.resolve() / path.name
        if resolved == project_root or project_root in resolved.parents:
            raise ValueError("execution roots must be outside the source repository")
        if resolved in resolved_roots:
            raise ValueError("execution roots must be distinct")
        resolved_roots.add(resolved)
        bound_roots[label] = str(resolved)

    initial_state = None
    initial_hashes = {}
    if kind == "development":
        initial_state = _derive_initial_state(Path(bindings["initial_family"]))
        initial_hashes = {
            item["trajectory_seed"]: item["positions_sha256"]
            for item in initial_state["missions"]
        }

    missions = [
        {
            "mission_id": f"mission-{index:02d}",
            "trajectory_seed": trajectory_seed,
            "range_noise_seed": range_seed,
            "frames": frames,
            **(
                {"initial_positions_sha256": initial_hashes[trajectory_seed]}
                if kind == "development" else {}
            ),
        }
        for index, (trajectory_seed, range_seed) in enumerate(
            zip(trajectory, ranges, strict=True), start=1
        )
    ]
    smoke_schedule = None
    if kind == "confirmatory":
        smoke_trajectory = kwargs.get("smoke_trajectory_seed")
        smoke_range = kwargs.get("smoke_range_noise_seed")
        smoke_frames = kwargs.get("smoke_frames")
        if (
            smoke_trajectory != 2026089001
            or smoke_range != 2026089101
            or smoke_frames != 20
        ):
            raise ValueError("confirmatory smoke schedule is not frozen")
        if {smoke_trajectory, smoke_range} & (
            set(trajectory) | set(ranges) | development
        ):
            raise ValueError("smoke seeds must be disjoint from scientific seeds")
        semantic = {
            "campaign_id": "confirmatory-smoke",
            "trajectory_seed": smoke_trajectory,
            "range_noise_seed": smoke_range,
            "frames": smoke_frames,
            "horizon_s": 10.0,
        }
        smoke_schedule = {
            "trajectory_seed": smoke_trajectory,
            "range_noise_seed": smoke_range,
            "frames": smoke_frames,
            "semantic_schedule_a": semantic,
            "semantic_schedule_b": dict(semantic),
            "universes": _universes(1, smoke_frames),
            "included_in_scientific_denominator": False,
        }

    protocol = {
        "schema_version": (
            "cbf2026-qualified-closure-protocol-v2"
            if kind == "development"
            else "cbf2026-qualified-closure-protocol-v1"
        ),
        "kind": kind,
        "version": version,
        "conditions": ["dynamic_primary", "fixed_fim_ablation"],
        "schedule": {
            "trajectory_seeds": trajectory,
            "range_noise_seeds": ranges,
            "frames": frames,
            "horizon_s": frames / 2.0,
            "missions": missions,
        },
        "smoke_schedule": smoke_schedule,
        "universes": _universes(expected_count, frames),
        "bindings": bound,
        "thresholds": dict(FROZEN_THRESHOLDS),
        "roots": bound_roots,
        "authorization": dict(FROZEN_AUTHORIZATION_REQUIREMENTS),
        "no_retry": True,
    }
    if kind == "development":
        protocol["initial_state"] = initial_state
    protocol["semantic_sha256"] = hashlib.sha256(
        _canonical_bytes(protocol)
    ).hexdigest()
    return protocol


def verify_registered_protocol(protocol: dict, *, allowed_claimed_roots=()) -> None:
    """Recompute every mutable file/root/threshold binding."""
    if not isinstance(protocol, dict):
        raise ValueError("protocol is not an object")
    _validate_exact_protocol_schema(protocol)
    if protocol.get("kind") == "development":
        verify_development_predecessor_state()
    expected_semantic = protocol.get("semantic_sha256")
    without_identity = dict(protocol)
    without_identity.pop("semantic_sha256", None)
    if hashlib.sha256(_canonical_bytes(without_identity)).hexdigest() != expected_semantic:
        raise ValueError("registered protocol semantic binding mutated")
    if _canonical_bytes(protocol.get("thresholds")) != _canonical_bytes(FROZEN_THRESHOLDS):
        raise ValueError("registered threshold binding mutated")
    _verify_derived_protocol_contract(protocol)
    bindings = protocol.get("bindings")
    if not isinstance(bindings, dict):
        raise ValueError("registered bindings are invalid")
    for label, identity in bindings.items():
        path = Path(identity.get("path", ""))
        if (
            not path.is_file()
            or path.is_symlink()
            or _sha256(path) != identity.get("sha256")
            or path.stat().st_size != identity.get("bytes")
        ):
            raise ValueError(f"binding mutated after registration: {label}")
    for section in ("review_artifacts", "tooling"):
        declarations = protocol.get(section)
        if declarations is not None:
            if not isinstance(declarations, dict):
                raise ValueError(f"registered {section} declarations are invalid")
            for label, identity in declarations.items():
                path = Path(identity.get("path", ""))
                if (
                    path.is_symlink() or not path.is_file()
                    or _sha256(path) != identity.get("sha256")
                    or path.stat().st_size != identity.get("bytes")
                ):
                    raise ValueError(f"registered {section} binding mutated: {label}")
    if protocol.get("kind") == "development":
        _verify_initial_state_binding(protocol)
    repository = protocol.get("repository")
    observed = _repository_identity(Path(repository["root"]))
    if observed["head"] == repository["head"]:
        if _canonical_bytes(observed) != _canonical_bytes(repository):
            raise ValueError(
                "registered repository identity or relevant Git status mutated"
            )
    else:
        _verify_committed_registration_state(protocol, observed)
    build = protocol.get("build")
    if build is not None:
        cmake = build.get("cmake_cache", {})
        cmake_path = Path(cmake.get("path", ""))
        if (
            cmake_path.is_symlink() or not cmake_path.is_file()
            or _sha256(cmake_path) != cmake.get("sha256")
            or "ENABLE_GUROBI:BOOL=ON" not in cmake_path.read_text(encoding="utf-8").splitlines()
        ):
            raise ValueError("registered CMake/Gurobi identity mutated")
        dependency_identity = build.get("binary_dependencies")
        binary_identity = bindings.get("binary", {})
        if (
            not isinstance(dependency_identity, dict)
            or _canonical_bytes(_dependency_identity(Path(binary_identity["path"])))
                != _canonical_bytes(dependency_identity)
        ):
            raise ValueError("registered binary dependency identity mutated")
        conda_identity = build.get("conda_explicit")
        if (
            not isinstance(repository, dict)
            or not isinstance(conda_identity, dict)
            or conda_identity.get("argv") != ["conda", "list", "--explicit"]
            or _canonical_bytes(_command_identity(
                conda_identity["argv"], Path(repository["root"])
            )) != _canonical_bytes(conda_identity)
        ):
            raise ValueError("registered conda identity mutated")
    roots = protocol.get("roots")
    if not isinstance(roots, dict):
        raise ValueError("registered roots are invalid")
    allowed_claimed_roots = set(allowed_claimed_roots)
    if not allowed_claimed_roots <= set(roots):
        raise ValueError("unknown claimed-root allowance")
    for label, serialized in roots.items():
        path = Path(serialized)
        if path.is_symlink():
            raise ValueError(f"registered root became symbolic: {label}")
        if _has_symbolic_ancestor(path):
            raise ValueError(
                f"registered root gained a symbolic ancestor: {label}"
            )
        if path.exists() and label not in allowed_claimed_roots:
            raise FileExistsError(f"registered root must remain absent: {label}")
        if label in allowed_claimed_roots and (
            not path.is_dir() or path.is_symlink()
        ):
            raise ValueError(f"claimed registered root is invalid: {label}")


def publish_protocol(
    protocol: dict,
    json_path: Path,
    markdown_path: Path,
    *,
    publish_hook=None,
) -> None:
    """Publish Markdown first and JSON last as the registration commit point."""
    verify_registered_protocol(protocol)
    json_path = Path(json_path)
    markdown_path = Path(markdown_path)
    repository_root = Path(protocol["repository"]["root"])
    declared_json = _absolute_registered_path(
        protocol["publication"]["json_path"], repository_root
    )
    declared_markdown = _absolute_registered_path(
        protocol["publication"]["markdown_path"], repository_root
    )
    if (
        _absolute_registered_path(json_path, Path.cwd()) != declared_json
        or _absolute_registered_path(markdown_path, Path.cwd()) != declared_markdown
    ):
        raise ValueError("protocol outputs differ from registered publication paths")
    if json_path == markdown_path:
        raise ValueError("protocol JSON and Markdown paths must be distinct")
    json_path.parent.mkdir(parents=True, exist_ok=True)
    markdown_path.parent.mkdir(parents=True, exist_ok=True)
    if json_path.parent.resolve() != markdown_path.parent.resolve():
        raise ValueError("protocol pair must share one publication directory")
    json_payload = _canonical_bytes(protocol) + b"\n"
    markdown_payload = _protocol_markdown_bytes(protocol)
    if json_path.exists() or json_path.is_symlink():
        raise FileExistsError(f"protocol output must be absent: {json_path}")
    if markdown_path.exists() or markdown_path.is_symlink():
        if (
            markdown_path.is_symlink()
            or not markdown_path.is_file()
            or markdown_path.read_bytes() != markdown_payload
        ):
            raise RuntimeError(
                "unrecoverable orphaned protocol Markdown blocks registration"
            )
        # JSON is absent, so an exact companion can only be pre-commit debris.
        markdown_path.unlink()
    bundle_stage = json_path.parent / f".qualified-protocol.{uuid.uuid4().hex}.tmp"
    bundle_stage.mkdir(mode=0o700)
    json_stage = bundle_stage / json_path.name
    markdown_stage = bundle_stage / markdown_path.name
    published = []
    try:
        _write_stage(json_stage, json_payload)
        _write_stage(markdown_stage, markdown_payload)
        os.link(markdown_stage, markdown_path)
        published.append(markdown_path)
        if publish_hook is not None:
            publish_hook(bundle_stage)
        os.link(json_stage, json_path)
        published.append(json_path)
    except BaseException:
        for path in reversed(published):
            path.unlink(missing_ok=True)
        raise
    finally:
        import shutil
        if bundle_stage.exists():
            shutil.rmtree(bundle_stage)


def validate_authorization_binding(
    protocol_path: Path,
    authorization_path: Path,
    *,
    allowed_claimed_roots=(),
) -> dict:
    """Validate a separately published authorization against exact bytes."""
    protocol_path = Path(protocol_path)
    authorization_path = Path(authorization_path)
    protocol = _json_object(protocol_path, "registered protocol")
    _validate_protocol_pair(protocol_path, protocol)
    repository = protocol.get("repository")
    if not isinstance(repository, dict):
        raise ValueError("registered repository identity is invalid")
    repository_root = Path(repository.get("root", ""))
    expected_authorization = _absolute_registered_path(
        _authorization_path(protocol_path), Path.cwd()
    )
    if _absolute_registered_path(authorization_path, Path.cwd()) != expected_authorization:
        raise ValueError("authorization path is not the named registered artifact")
    authorization = _json_object(authorization_path, "authorization")
    if set(authorization) != AUTHORIZATION_FIELDS:
        raise ValueError("authorization schema is not exact")
    if (
        authorization["schema_version"]
            != "cbf2026-qualified-authorization-v1"
        or authorization["authorized"] is not True
        or authorization["kind"] != protocol.get("kind")
        or authorization["version"] != protocol.get("version")
        or not _lower_hex(authorization["protocol_sha256"], 64)
        or not _lower_hex(authorization["implementation_identity"], 40)
        or not _lower_hex(authorization["preflight_sha256"], 64)
        or not _lower_hex(authorization["user_authorization_text_sha256"], 64)
    ):
        raise ValueError("authorization identity is invalid")
    if authorization["protocol_sha256"] != _sha256(protocol_path):
        raise ValueError("authorization protocol SHA-256 binding does not match")
    if authorization["implementation_identity"] != repository.get("head"):
        raise ValueError("authorization implementation identity does not match")
    preflight_path = Path(_preflight_path(protocol_path))
    if (
        preflight_path.is_symlink()
        or not preflight_path.is_file()
        or authorization["preflight_sha256"] != _sha256(preflight_path)
    ):
        raise ValueError("authorization preflight SHA-256 binding does not match")
    authorization_text = authorization["user_authorization_text"]
    if (
        not isinstance(authorization_text, str)
        or not authorization_text
        or hashlib.sha256(authorization_text.encode("utf-8")).hexdigest()
            != authorization["user_authorization_text_sha256"]
    ):
        raise ValueError("authorization text binding does not match")
    authorization_date = authorization["user_authorization_date"]
    try:
        parsed_date = date.fromisoformat(authorization_date)
    except (TypeError, ValueError):
        raise ValueError("authorization date is not canonical ISO YYYY-MM-DD") from None
    if parsed_date.isoformat() != authorization_date:
        raise ValueError("authorization date is not canonical ISO YYYY-MM-DD")
    observed = _repository_identity(repository_root)
    if observed["head"] == repository["head"]:
        raise ValueError("authorization artifacts require a committed artifact commit")
    verify_registered_protocol(
        protocol, allowed_claimed_roots=allowed_claimed_roots
    )
    return authorization


def _seed_list(value, label: str) -> list[int]:
    if not isinstance(value, list) or not all(
        type(seed) is int and 0 <= seed < 2**64 for seed in value
    ):
        raise ValueError(f"{label} seeds must be serialized uint64 integers")
    return list(value)


def _reference_selection(config: dict):
    covariance = config.get("position_covariance")
    return covariance.get("reference-selection") if isinstance(covariance, dict) else None


def _json_object(path: Path, label: str) -> dict:
    def reject_duplicate(pairs):
        output = {}
        for key, value in pairs:
            if key in output:
                raise ValueError(f"{label} contains duplicate key: {key}")
            output[key] = value
        return output

    try:
        value = json.loads(
            path.read_text(encoding="utf-8"), object_pairs_hook=reject_duplicate
        )
    except (OSError, json.JSONDecodeError, ValueError) as error:
        raise ValueError(f"{label} is unreadable") from error
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object")
    return value


def _require_literal_v6_predecessor_identity_path(path: Path) -> Path:
    """Authorize only the committed, non-symbolic v6 predecessor record."""
    spelling = os.fspath(path)
    if spelling != FROZEN_V6_PREDECESSOR_IDENTITY_RELATIVE_PATH:
        raise ValueError("v5 predecessor identity path is not literal")
    path = Path(spelling)
    if (
        path.is_symlink()
        or _has_symbolic_ancestor(path)
        or not path.is_file()
        or path.resolve() != FROZEN_V6_PREDECESSOR_IDENTITY_PATH.resolve()
    ):
        raise ValueError("v5 predecessor identity path is not literal")
    return path


def _validate_v6_predecessor_identity(identity: Mapping) -> None:
    """Reject any extensible or self-rehashed v5 predecessor declaration."""
    expected = FROZEN_V6_PREDECESSOR_V5_IDENTITY
    if not isinstance(identity, Mapping) or set(identity) != set(expected):
        raise ValueError("v5 predecessor identity schema is not exact")
    if identity.get("schema_version") != expected["schema_version"]:
        raise ValueError("v5 predecessor identity schema version mutated")
    terminal = identity.get("terminal")
    if not isinstance(terminal, Mapping) or set(terminal) != set(expected["terminal"]):
        raise ValueError("v5 predecessor terminal identities are incomplete")
    for label, expected_record in expected["terminal"].items():
        record = terminal.get(label)
        if not isinstance(record, Mapping) or set(record) != set(expected_record):
            raise ValueError(f"v5 predecessor {label} identity schema is not exact")
        for key in ("files", "logical_bytes"):
            if type(record.get(key)) is not int or record[key] < 0:
                raise ValueError(f"v5 predecessor {label} identity has invalid {key}")
        for key in ("tree_sha256", "manifest_sha256"):
            if not _lower_hex(record.get(key), 64):
                raise ValueError(f"v5 predecessor {label} identity has noncanonical hash")
        if record != expected_record:
            raise ValueError(f"v5 predecessor {label} identity mutated")


def load_v6_predecessor_identity(path: Path) -> dict:
    """Load the one immutable v5 terminal identity authorized for v6."""
    path = _require_literal_v6_predecessor_identity_path(path)
    before = _sha256(path)
    identity = _json_object(path, "v5 predecessor identity")
    after = _sha256(path)
    if before != after:
        raise ValueError("v5 predecessor identity changed while being read")
    _validate_v6_predecessor_identity(identity)
    return identity


def verify_v6_predecessor_state(identity: Mapping) -> None:
    """Recompute the frozen v5 trees and manifests without modifying them."""
    _validate_v6_predecessor_identity(identity)
    for label, expected in identity["terminal"].items():
        root = Path(expected["root"])
        if root.is_symlink() or _has_symbolic_ancestor(root):
            raise ValueError(f"v5 predecessor {label} root is symbolic")
        try:
            observed = _directory_tree_identity(root)
        except ValueError as error:
            raise ValueError(f"v5 predecessor {label} tree is unavailable") from error
        expected_tree = {
            key: expected[key]
            for key in ("files", "logical_bytes", "tree_sha256")
        }
        manifest = root / expected["manifest_name"]
        if (
            observed != expected_tree
            or manifest.is_symlink()
            or not manifest.is_file()
            or _sha256(manifest) != expected["manifest_sha256"]
        ):
            raise ValueError(f"v5 predecessor {label} tree or manifest mutated")


def _initial_family_semantic_sha256(family: dict) -> str:
    from scripts.diagnostics.qualified_initial_state import family_semantic_sha256

    return family_semantic_sha256(family)


def _derive_initial_state(path: Path) -> dict:
    """Reload and independently derive the exact frozen v5 family declaration."""
    from scripts.diagnostics.qualified_initial_state import (
        audit_frozen_initial_family,
        load_qualified_initial_family,
    )

    path = Path(path)
    before = _sha256(path)
    family = load_qualified_initial_family(path)
    after = _sha256(path)
    if before != after:
        raise ValueError("initial family changed while being validated")
    cache_key = (after, family["semantic_sha256"])
    audit = _INITIAL_AUDIT_CACHE.get(cache_key)
    if audit is None:
        audit = audit_frozen_initial_family(family)
        _INITIAL_AUDIT_CACHE[cache_key] = audit
    schedule = family["schedule"]
    return {
        "family_schema_version": family["schema_version"],
        "namespace": family["namespace"],
        "family_semantic_sha256": family["semantic_sha256"],
        "registered_trajectory_seeds": list(
            schedule["registered_trajectory_seeds"]
        ),
        "audit_trajectory_seeds": list(range(
            schedule["audit_seed_first"], schedule["audit_seed_last"] + 1,
        )),
        "missions": [
            {
                "trajectory_seed": item.seed,
                "positions_sha256": item.positions_sha256,
            }
            for item in audit.registered.audits
        ],
        "frozen_summary": family["frozen_summary"],
        "admission": family["admission"],
        "perturbation_policy": {
            "clamp": family["perturbation"]["clamp"],
            "resample": family["perturbation"]["resample"],
        },
    }


def _verify_initial_state_binding(protocol: dict) -> None:
    """Reject a self-rehashed substitution of any derived v5 initial state."""
    binding = protocol.get("bindings", {}).get("initial_family", {})
    family_path = _require_literal_initial_family_path(binding.get("path", ""))
    expected = _derive_initial_state(family_path)
    if protocol.get("initial_state") != expected:
        raise ValueError("registered initial-state derivation mutated")
    mission_hashes = {
        item["trajectory_seed"]: item["positions_sha256"]
        for item in expected["missions"]
    }
    schedule = protocol.get("schedule", {}).get("missions")
    if not isinstance(schedule, list) or any(
        mission.get("initial_positions_sha256")
            != mission_hashes.get(mission.get("trajectory_seed"))
        for mission in schedule
    ):
        raise ValueError("registered initial-state mission binding mutated")


def _directory_tree_identity(root: Path) -> dict:
    """Hash one immutable regular-file tree by sorted logical members."""
    root = Path(root)
    if root.is_symlink() or not root.is_dir():
        raise ValueError("development predecessor tree is absent or symbolic")
    members = []
    for path in sorted(root.rglob("*"), key=lambda item: item.relative_to(root).as_posix()):
        if path.is_symlink() or not (path.is_file() or path.is_dir()):
            raise ValueError("development predecessor tree contains a special member")
        if path.is_dir():
            continue
        relative = path.relative_to(root).as_posix()
        size = path.stat().st_size
        members.append((relative, size, _sha256(path)))
    payload = "".join(
        f"{relative}\t{size}\t{digest}\n"
        for relative, size, digest in members
    ).encode("utf-8")
    return {
        "files": len(members),
        "logical_bytes": sum(size for _, size, _ in members),
        "tree_sha256": hashlib.sha256(payload).hexdigest(),
    }


def verify_development_predecessor_state() -> None:
    """Require v1-v3 absence and the byte-exact terminal v4 evidence trees."""
    state = FROZEN_DEVELOPMENT_PREDECESSOR_STATE
    for path in state["absent"]:
        path = Path(path)
        if path.exists() or path.is_symlink():
            raise ValueError("development predecessor consumed root unexpectedly exists")
    for label, expected in state["terminal"].items():
        root = Path(expected["root"])
        observed = _directory_tree_identity(root)
        expected_tree = {
            key: expected[key]
            for key in ("files", "logical_bytes", "tree_sha256")
        }
        manifest = root / expected["manifest_name"]
        if (
            observed != expected_tree
            or manifest.is_symlink()
            or not manifest.is_file()
            or _sha256(manifest) != expected["manifest_sha256"]
        ):
            raise ValueError(f"development predecessor {label} tree mutated")


def _universes(missions: int, frames: int) -> dict:
    estimator = missions * max(0, frames - 1) * 14
    return {
        "initialization": missions * 14,
        "estimator_per_condition": estimator,
        "estimator_total": estimator * 2,
        "controller": missions * frames,
        "endpoint": missions * frames * 232,
        "reconstructed": missions * frames * 119,
        "mission": missions,
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_bytes(value) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _stage(path: Path) -> Path:
    return path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")


def _write_stage(path: Path, payload: bytes) -> None:
    with path.open("xb") as output:
        output.write(payload)
        output.flush()
        os.fsync(output.fileno())


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Register one immutable qualified-closure protocol"
    )
    parser.add_argument("--kind", choices=("development", "confirmatory"), required=True)
    parser.add_argument("--version", required=True)
    parser.add_argument("--implementation-report", type=Path)
    parser.add_argument("--implementation-review", type=Path)
    parser.add_argument("--development-protocol", type=Path)
    parser.add_argument("--development-report", type=Path)
    parser.add_argument("--development-review", type=Path)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--base-config", type=Path, required=True)
    parser.add_argument("--primary-config", type=Path, required=True)
    parser.add_argument("--ablation-config", type=Path, required=True)
    parser.add_argument("--initial-family", type=Path)
    parser.add_argument("--trajectory-seeds", required=True)
    parser.add_argument("--range-noise-seeds", required=True)
    parser.add_argument("--frames", type=int, required=True)
    parser.add_argument("--smoke-trajectory-seed", type=int)
    parser.add_argument("--smoke-range-noise-seed", type=int)
    parser.add_argument("--smoke-frames", type=int)
    parser.add_argument("--smoke-a-raw-root", type=Path)
    parser.add_argument("--smoke-a-analysis-root", type=Path)
    parser.add_argument("--smoke-b-raw-root", type=Path)
    parser.add_argument("--smoke-b-analysis-root", type=Path)
    parser.add_argument("--raw-root", type=Path, required=True)
    parser.add_argument("--analysis-root", type=Path, required=True)
    parser.add_argument("--protocol-json", type=Path, required=True)
    parser.add_argument("--protocol-md", type=Path, required=True)
    return parser


def main(argv=None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    try:
        register_from_arguments(arguments)
    except Exception as error:
        print(f"qualified registration failed: {error}", file=sys.stderr)
        return 1
    return 0


def register_from_arguments(arguments, *, project_root=None) -> dict:
    """Collect authoritative local identities and publish one frozen protocol."""
    project_root = Path(project_root or Path.cwd()).resolve()
    repository = _repository_identity(project_root)
    if repository["dirty_relevant_paths"]:
        raise ValueError("dirty relevant source is not registrable")
    if arguments.kind == "development":
        verify_development_predecessor_state()
        if getattr(arguments, "initial_family", None) is None:
            raise ValueError("development initial-family binding is required")
    elif getattr(arguments, "initial_family", None) is not None:
        raise ValueError("confirmatory protocol must not accept --initial-family")
    for label in ("binary", "base_config", "primary_config", "ablation_config"):
        path = Path(getattr(arguments, label))
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"{label} must be a regular non-symlink file")
    if not os.access(arguments.binary, os.X_OK):
        raise ValueError("binary must be executable")

    if arguments.kind == "development":
        report_paths = [
            arguments.implementation_report, arguments.implementation_review,
        ]
    else:
        report_paths = [
            arguments.development_protocol,
            arguments.development_report,
            arguments.development_review,
        ]
    if any(path is None for path in report_paths):
        raise ValueError("reviewed implementation/development artifacts are required")
    report_labels = (
        ("implementation_report", "implementation_review")
        if arguments.kind == "development"
        else ("development_protocol", "development_report", "development_review")
    )
    reports = {}
    for label, path in zip(report_labels, report_paths, strict=True):
        path = Path(path)
        if not path.is_file() or path.is_symlink():
            raise ValueError("review artifact must be a regular non-symlink file")
        reports[label] = _file_identity(path)

    cmake_cache = Path(arguments.binary).parent / "CMakeCache.txt"
    if not cmake_cache.is_file() or cmake_cache.is_symlink():
        raise ValueError("binary CMakeCache.txt identity is unavailable")
    cmake_text = cmake_cache.read_text(encoding="utf-8", errors="strict")
    if "ENABLE_GUROBI:BOOL=ON" not in cmake_text.splitlines():
        raise ValueError("registered binary must have Gurobi enabled")
    tool_root = Path(__file__).resolve().parent
    source_path = tool_root / "run_qualified_closure_campaign.py"
    schema_path = tool_root / "qualified_closure_evidence.py"
    roots = {
        "raw": arguments.raw_root,
        "analysis": arguments.analysis_root,
    }
    if arguments.kind == "confirmatory":
        roots.update({
            "smoke_a_raw": arguments.smoke_a_raw_root,
            "smoke_a_analysis": arguments.smoke_a_analysis_root,
            "smoke_b_raw": arguments.smoke_b_raw_root,
            "smoke_b_analysis": arguments.smoke_b_analysis_root,
        })
        if any(value is None for value in roots.values()):
            raise ValueError("all confirmatory smoke/scientific roots are required")

    protocol = build_qualified_closure_protocol(
        kind=arguments.kind,
        version=arguments.version,
        project_root=project_root,
        trajectory_seeds=_parse_seed_expression(arguments.trajectory_seeds),
        range_noise_seeds=_parse_seed_expression(arguments.range_noise_seeds),
        frames=arguments.frames,
        smoke_trajectory_seed=arguments.smoke_trajectory_seed,
        smoke_range_noise_seed=arguments.smoke_range_noise_seed,
        smoke_frames=arguments.smoke_frames,
        roots=roots,
        bindings={
            "source": source_path,
            "binary": arguments.binary,
            "base_config": arguments.base_config,
            "primary_config": arguments.primary_config,
            "ablation_config": arguments.ablation_config,
            "dependencies": cmake_cache,
            "schema": schema_path,
            **(
                {"initial_family": arguments.initial_family}
                if arguments.kind == "development" else {}
            ),
        },
        thresholds=FROZEN_THRESHOLDS,
        dirty_relevant_paths=repository["dirty_relevant_paths"],
    )
    dependency_probe = _dependency_identity(
        Path(protocol["bindings"]["binary"]["path"])
    )
    conda_probe = _command_identity(["conda", "list", "--explicit"], project_root)
    protocol.update({
        "repository": repository,
        "review_artifacts": reports,
        "build": {
            "cmake_cache": _file_identity(cmake_cache),
            "gurobi_enabled": True,
            "binary_dependencies": dependency_probe,
            "conda_explicit": conda_probe,
        },
        "tooling": {
            path.name: _file_identity(path)
            for path in (
                tool_root / "run_qualified_closure_campaign.py",
                tool_root / "generate_qualified_measurements.py",
                tool_root / "analyze_qualified_closure_campaign.py",
                tool_root / "register_qualified_closure_campaign.py",
                tool_root / "replay_qualified_estimator.py",
                tool_root / "analyze_qualified_estimator.py",
                *(
                    (tool_root / "qualified_initial_state.py",)
                    if arguments.kind == "development" else ()
                ),
            )
        },
        "publication": {
            "json_path": _path_token(arguments.protocol_json),
            "markdown_path": _path_token(arguments.protocol_md),
        },
        "supervision": dict(FROZEN_SUPERVISION),
    })
    commands = _registered_argv(protocol)
    protocol["runner_argv"] = commands["runner"]
    protocol["analyzer_argv"] = commands["analyzer"]
    if arguments.kind == "confirmatory":
        protocol["smoke_argv"] = commands["smoke"]
    protocol["semantic_sha256"] = _semantic_sha256(protocol)
    publish_protocol(protocol, arguments.protocol_json, arguments.protocol_md)
    return protocol


def _parse_seed_expression(expression: str) -> list[int]:
    if not isinstance(expression, str):
        raise ValueError("seed expression must be serialized")
    pieces = expression.split(":")
    if len(pieces) == 2 and all(piece.isdecimal() for piece in pieces):
        start, stop = map(int, pieces)
        return list(range(start, stop + 1)) if stop >= start else []
    if expression and all(piece.isdecimal() for piece in expression.split(",")):
        return [int(piece) for piece in expression.split(",")]
    raise ValueError("seed expression must be an inclusive A:B range or comma list")


def _repository_identity(project_root: Path) -> dict:
    def git(*tokens):
        result = subprocess.run(
            ["git", *tokens], cwd=project_root, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
        )
        if result.returncode != 0:
            raise ValueError(f"Git identity unavailable: {result.stderr.strip()}")
        return result.stdout.strip()
    top = Path(git("rev-parse", "--show-toplevel")).resolve()
    if top != project_root:
        raise ValueError("registration project root must equal the Git worktree root")
    status = git("status", "--porcelain=v1", "--untracked-files=all")
    dirty_tracked = []
    dirty_relevant = []
    allowed_untracked = []
    for line in status.splitlines():
        if not line:
            continue
        path = line[3:]
        if line.startswith("?? "):
            if path == "build-diagnostic" or path.startswith("build-diagnostic/"):
                allowed_untracked.append(path)
            else:
                dirty_relevant.append(path)
        else:
            dirty_tracked.append(path)
            dirty_relevant.append(path)
    return {
        "root": str(top),
        "head": git("rev-parse", "HEAD"),
        "tree": git("rev-parse", "HEAD^{tree}"),
        "dirty_tracked_paths": sorted(dirty_tracked),
        "dirty_relevant_paths": sorted(dirty_relevant),
        "allowed_untracked_paths": sorted(allowed_untracked),
    }


def _git_bytes(project_root: Path, *tokens: str) -> bytes:
    result = subprocess.run(
        ["git", *tokens], cwd=project_root,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
    )
    if result.returncode != 0:
        raise ValueError(
            "registered Git state is unavailable: "
            + result.stderr.decode("utf-8", errors="replace").strip()
        )
    return result.stdout


def _registered_artifact_paths(protocol: dict) -> tuple[Path, ...]:
    repository_root = Path(protocol["repository"]["root"])
    protocol_path = _absolute_registered_path(
        protocol["publication"]["json_path"], repository_root,
    )
    markdown_path = _absolute_registered_path(
        protocol["publication"]["markdown_path"], repository_root,
    )
    return (
        protocol_path,
        markdown_path,
        Path(_preflight_path(protocol_path)),
        Path(_authorization_path(protocol_path)),
    )


def _verify_committed_registration_state(protocol: dict, observed: dict) -> None:
    """Accept only the one add-only authorization commit after implementation."""
    repository = protocol["repository"]
    project_root = Path(repository["root"]).resolve()
    implementation = repository["head"]
    current = observed["head"]
    ancestry = subprocess.run(
        ["git", "merge-base", "--is-ancestor", implementation, current],
        cwd=project_root, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        check=False,
    )
    if ancestry.returncode != 0:
        raise ValueError("registered HEAD is not a descendant of implementation")
    parents = _git_bytes(
        project_root, "rev-list", "--parents", "-n", "1", current,
    ).decode("ascii", errors="strict").split()
    if parents != [current, implementation]:
        raise ValueError("registered artifact HEAD is not the direct child of implementation")

    artifacts = _registered_artifact_paths(protocol)
    relative = []
    for path in artifacts:
        try:
            relative.append(path.resolve().relative_to(project_root).as_posix())
        except ValueError:
            raise ValueError("registered artifacts must remain inside repository") from None
    expected_changes = sorted(("A", path) for path in relative)
    diff = _git_bytes(
        project_root, "diff", "--name-status", "--no-renames",
        implementation, current,
    ).decode("utf-8", errors="strict")
    observed_changes = []
    for line in diff.splitlines():
        fields = line.split("\t")
        if len(fields) != 2:
            raise ValueError("registered artifact-only diff is malformed")
        observed_changes.append((fields[0], fields[1]))
    if sorted(observed_changes) != expected_changes:
        raise ValueError(
            "registered direct child must be an exact four-artifact add-only commit"
        )

    if (
        observed.get("root") != repository.get("root")
        or observed.get("dirty_tracked_paths")
        or observed.get("dirty_relevant_paths")
        or observed.get("allowed_untracked_paths")
            != repository.get("allowed_untracked_paths")
    ):
        raise ValueError("registered repository has dirty relevant or protected paths")
    for path, token in zip(artifacts, relative, strict=True):
        if path.is_symlink() or not path.is_file():
            raise ValueError("registered artifact is not a regular live file")
        blob = _git_bytes(project_root, "show", f"{current}:{token}")
        if blob != path.read_bytes():
            raise ValueError("registered artifact differs from exact HEAD Git blob")

    identities = {}
    for section in ("bindings", "review_artifacts", "tooling"):
        identities.update(protocol.get(section, {}))
    allowed_untracked = set(repository.get("allowed_untracked_paths", ()))
    for label, identity in identities.items():
        path = Path(identity["path"])
        try:
            token = path.resolve().relative_to(project_root).as_posix()
        except ValueError:
            continue
        if token in allowed_untracked:
            if (
                path.is_symlink()
                or not path.is_file()
                or _sha256(path) != identity["sha256"]
                or path.stat().st_size != identity["bytes"]
            ):
                raise ValueError(
                    f"allowed-untracked implementation file differs: {label}"
                )
            continue
        blob = _git_bytes(project_root, "show", f"{implementation}:{token}")
        if (
            hashlib.sha256(blob).hexdigest() != identity["sha256"]
            or len(blob) != identity["bytes"]
        ):
            raise ValueError(f"implementation Git blob differs: {label}")


def _file_identity(path: Path) -> dict:
    path = Path(path)
    return {
        "path": str(path.resolve()),
        "sha256": _sha256(path),
        "bytes": path.stat().st_size,
    }


def _command_identity(argv, cwd: Path) -> dict:
    result = subprocess.run(
        argv, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        check=False,
    )
    payload = result.stdout + result.stderr
    if result.returncode != 0:
        raise ValueError(f"identity command failed: {' '.join(argv)}")
    return {
        "argv": list(argv),
        "sha256": hashlib.sha256(payload).hexdigest(),
        "bytes": len(payload),
    }


def _dependency_identity(binary: Path) -> dict:
    command = ["otool", "-L", str(binary)] if sys.platform == "darwin" else ["ldd", str(binary)]
    result = subprocess.run(
        command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False
    )
    payload = result.stdout + result.stderr
    if result.returncode != 0:
        raise ValueError(
            f"dependency identity command failed with status {result.returncode}"
        )
    return {
        "argv": command,
        "returncode": result.returncode,
        "sha256": hashlib.sha256(payload).hexdigest(),
        "bytes": len(payload),
    }


def _protocol_markdown_bytes(protocol: dict) -> bytes:
    json_sha256 = hashlib.sha256(
        _canonical_bytes(protocol) + b"\n"
    ).hexdigest()
    return (
        "# Qualified Closure Campaign Protocol\n\n"
        f"- Kind: `{protocol['kind']}`\n"
        f"- Version: `{protocol['version']}`\n"
        f"- Semantic SHA-256: `{protocol['semantic_sha256']}`\n"
        f"- Protocol JSON SHA-256: `{json_sha256}`\n"
        f"- Scientific missions: `{protocol['universes']['mission']}`\n"
        "- Retry policy: `forbidden`\n"
    ).encode("utf-8")


def _validate_protocol_pair(protocol_path: Path, protocol: dict) -> None:
    repository_root = Path(protocol.get("repository", {}).get("root", ""))
    declared_json = _absolute_registered_path(
        protocol.get("publication", {}).get("json_path", ""), repository_root,
    )
    markdown_path = _absolute_registered_path(
        protocol.get("publication", {}).get("markdown_path", ""), repository_root,
    )
    if _absolute_registered_path(protocol_path, Path.cwd()) != declared_json:
        raise ValueError("protocol path differs from registered publication path")
    expected_json = _canonical_bytes(protocol) + b"\n"
    if protocol_path.is_symlink() or protocol_path.read_bytes() != expected_json:
        raise ValueError("registered protocol JSON is not canonical")
    if (
        markdown_path.is_symlink()
        or not markdown_path.is_file()
        or markdown_path.read_bytes() != _protocol_markdown_bytes(protocol)
    ):
        raise ValueError("registered protocol companion Markdown is invalid")


def _path_token(path) -> str:
    return str(Path(path))


def _absolute_registered_path(path, base: Path) -> Path:
    lexical = Path(path)
    if not lexical.is_absolute():
        lexical = Path(base) / lexical
    return Path(os.path.abspath(os.fspath(lexical)))


def _repository_path_token(path, repository_root: Path) -> str:
    absolute = Path(path).resolve()
    try:
        return absolute.relative_to(repository_root.resolve()).as_posix()
    except ValueError:
        return str(absolute)


def _runner_argv(arguments) -> list[str]:
    authorization = _authorization_path(arguments.protocol_json)
    tokens = [
        "conda", "run", "-n", "cbf_env", "python", "-m",
        "scripts.diagnostics.run_qualified_closure_campaign",
        "--kind", arguments.kind,
    ]
    tokens.extend(["--version", arguments.version])
    tokens.extend([
        "--protocol", _path_token(arguments.protocol_json),
        "--authorization", authorization,
        "--binary", _path_token(arguments.binary),
        "--base-config", _path_token(arguments.base_config),
        "--primary-config", _path_token(arguments.primary_config),
        "--ablation-config", _path_token(arguments.ablation_config),
    ])
    if arguments.kind == "development":
        if getattr(arguments, "initial_family", None) is None:
            raise ValueError("development runner requires --initial-family")
        tokens.extend(["--initial-family", _path_token(arguments.initial_family)])
    elif getattr(arguments, "initial_family", None) is not None:
        raise ValueError("confirmatory runner must not accept --initial-family")
    tokens.extend([
        "--trajectory-seeds", arguments.trajectory_seeds,
        "--range-noise-seeds", arguments.range_noise_seeds,
        "--frames", str(arguments.frames),
        "--output-root", _path_token(arguments.raw_root),
    ])
    return tokens


def _analyzer_argv(arguments) -> list[str]:
    authorization = _authorization_path(arguments.protocol_json)
    tokens = [
        "conda", "run", "-n", "cbf_env", "python", "-m",
        "scripts.diagnostics.analyze_qualified_closure_campaign",
        "--kind", arguments.kind,
    ]
    tokens.extend(["--version", arguments.version])
    tokens.extend([
        "--protocol", _path_token(arguments.protocol_json),
        "--authorization", authorization,
        "--input-root", _path_token(arguments.raw_root),
    ])
    if arguments.kind == "development":
        tokens.extend(["--ablation-config", _path_token(arguments.ablation_config)])
    tokens.extend(["--output-root", _path_token(arguments.analysis_root)])
    return tokens


def _smoke_argv(arguments) -> dict:
    authorization = _authorization_path(arguments.protocol_json)
    result = {}
    for smoke_id in ("a", "b"):
        raw = getattr(arguments, f"smoke_{smoke_id}_raw_root")
        analysis = getattr(arguments, f"smoke_{smoke_id}_analysis_root")
        shared = [
            "--smoke-id", smoke_id,
            "--protocol", _path_token(arguments.protocol_json),
            "--authorization", authorization,
        ]
        runner = [
            "conda", "run", "-n", "cbf_env", "python", "-m",
            "scripts.diagnostics.run_qualified_closure_campaign",
            "--kind", "confirmatory-smoke", *shared,
            "--binary", _path_token(arguments.binary),
            "--base-config", _path_token(arguments.base_config),
            "--primary-config", _path_token(arguments.primary_config),
            "--ablation-config", _path_token(arguments.ablation_config),
            "--trajectory-seeds",
            f"{arguments.smoke_trajectory_seed}:{arguments.smoke_trajectory_seed}",
            "--range-noise-seeds",
            f"{arguments.smoke_range_noise_seed}:{arguments.smoke_range_noise_seed}",
            "--frames", str(arguments.smoke_frames),
            "--output-root", _path_token(raw),
        ]
        analyzer = [
            "conda", "run", "-n", "cbf_env", "python", "-m",
            "scripts.diagnostics.analyze_qualified_closure_campaign",
            "--kind", "confirmatory-smoke", "--version", arguments.version,
            *shared,
            "--input-root", _path_token(raw),
            "--output-root", _path_token(analysis),
        ]
        result[smoke_id] = {"runner": runner, "analyzer": analyzer}
    return result


def _authorization_path(protocol_json) -> str:
    protocol_path = Path(protocol_json)
    name = protocol_path.name
    if not name.endswith("-protocol.json"):
        raise ValueError("protocol JSON name must end in -protocol.json")
    authorization_name = (
        name.removesuffix("-protocol.json") + "-authorization.json"
    )
    return str(protocol_path.parent / "reviews" / authorization_name)


def _preflight_path(protocol_json) -> str:
    protocol_path = Path(protocol_json)
    name = protocol_path.name
    if not name.endswith("-protocol.json"):
        raise ValueError("protocol JSON name must end in -protocol.json")
    preflight_name = name.removesuffix("-protocol.json") + "-preflight.md"
    return str(protocol_path.parent / "reviews" / preflight_name)


def _lower_hex(value, length: int) -> bool:
    return (
        isinstance(value, str)
        and len(value) == length
        and all(character in "0123456789abcdef" for character in value)
    )


def _has_symbolic_ancestor(path: Path) -> bool:
    lexical = Path(os.path.abspath(os.fspath(path)))
    return any(parent.is_symlink() for parent in lexical.parents)


def _validate_exact_protocol_schema(protocol: dict) -> None:
    """Reject protocols whose declarations are missing or extensible by rehash."""
    kind = protocol.get("kind")
    expected_top = {
        "schema_version", "kind", "version", "conditions", "schedule",
        "smoke_schedule", "universes", "bindings", "thresholds", "roots",
        "authorization", "supervision", "no_retry", "repository",
        "review_artifacts", "build", "tooling", "publication",
        "runner_argv", "analyzer_argv", "semantic_sha256",
    }
    if kind == "confirmatory":
        expected_top.add("smoke_argv")
    else:
        expected_top.add("initial_state")
    if set(protocol) != expected_top:
        raise ValueError("registered protocol top-level schema is not exact")

    def exact(value, fields, label):
        if not isinstance(value, dict) or set(value) != set(fields):
            raise ValueError(f"registered protocol {label} schema is not exact")

    exact(
        protocol["schedule"],
        {"trajectory_seeds", "range_noise_seeds", "frames", "horizon_s", "missions"},
        "schedule",
    )
    if not isinstance(protocol["schedule"]["missions"], list):
        raise ValueError("registered protocol mission schema is not exact")
    for mission in protocol["schedule"]["missions"]:
        exact(
            mission,
            {
                "mission_id", "trajectory_seed", "range_noise_seed", "frames",
                *({"initial_positions_sha256"} if kind == "development" else set()),
            },
            "mission",
        )
    universe_fields = {
        "initialization", "estimator_per_condition", "estimator_total",
        "controller", "endpoint", "reconstructed", "mission",
    }
    exact(protocol["universes"], universe_fields, "universe")
    binding_fields = {
        "source", "binary", "base_config", "primary_config",
        "ablation_config", "dependencies", "schema",
    }
    if kind == "development":
        binding_fields.add("initial_family")
    exact(protocol["bindings"], binding_fields, "binding")
    for identity in protocol["bindings"].values():
        exact(identity, {"path", "sha256", "bytes"}, "file identity")
    exact(protocol["thresholds"], set(FROZEN_THRESHOLDS), "threshold")
    exact(
        protocol["authorization"],
        set(FROZEN_AUTHORIZATION_REQUIREMENTS),
        "authorization",
    )
    exact(
        protocol["supervision"],
        {"wallclock_timeout_s", "line_stall_timeout_s", "termination_grace_s"},
        "supervision",
    )
    exact(
        protocol["repository"],
        {"root", "head", "tree", "dirty_tracked_paths",
         "dirty_relevant_paths", "allowed_untracked_paths"},
        "repository",
    )
    review_fields = (
        {"implementation_report", "implementation_review"}
        if kind == "development"
        else {"development_protocol", "development_report", "development_review"}
    )
    exact(protocol["review_artifacts"], review_fields, "review artifact")
    for identity in protocol["review_artifacts"].values():
        exact(identity, {"path", "sha256", "bytes"}, "file identity")
    exact(
        protocol["build"],
        {"cmake_cache", "gurobi_enabled", "binary_dependencies", "conda_explicit"},
        "build",
    )
    exact(protocol["build"]["cmake_cache"], {"path", "sha256", "bytes"}, "file identity")
    exact(
        protocol["build"]["binary_dependencies"],
        {"argv", "returncode", "sha256", "bytes"},
        "binary dependency identity",
    )
    exact(
        protocol["build"]["conda_explicit"],
        {"argv", "sha256", "bytes"},
        "conda identity",
    )
    tooling_fields = {
        "run_qualified_closure_campaign.py", "generate_qualified_measurements.py",
        "analyze_qualified_closure_campaign.py", "register_qualified_closure_campaign.py",
        "replay_qualified_estimator.py", "analyze_qualified_estimator.py",
    }
    if kind == "development":
        tooling_fields.add("qualified_initial_state.py")
    exact(protocol["tooling"], tooling_fields, "tooling")
    for identity in protocol["tooling"].values():
        exact(identity, {"path", "sha256", "bytes"}, "file identity")
    exact(protocol["publication"], {"json_path", "markdown_path"}, "publication")
    for label in ("runner_argv", "analyzer_argv"):
        if not _string_argv(protocol[label]):
            raise ValueError(f"registered protocol {label} schema is not exact")
    if kind == "confirmatory":
        exact(protocol["smoke_argv"], {"a", "b"}, "smoke argv")
        for smoke in protocol["smoke_argv"].values():
            exact(smoke, {"runner", "analyzer"}, "smoke argv member")
            if not _string_argv(smoke["runner"]) or not _string_argv(smoke["analyzer"]):
                raise ValueError("registered protocol smoke argv schema is not exact")
        exact(
            protocol["smoke_schedule"],
            {"trajectory_seed", "range_noise_seed", "frames", "semantic_schedule_a",
             "semantic_schedule_b", "universes", "included_in_scientific_denominator"},
            "smoke schedule",
        )
        for label in ("semantic_schedule_a", "semantic_schedule_b"):
            exact(
                protocol["smoke_schedule"][label],
                {"campaign_id", "trajectory_seed", "range_noise_seed", "frames", "horizon_s"},
                "smoke semantic schedule",
            )
        exact(protocol["smoke_schedule"]["universes"], universe_fields, "smoke universe")
    elif protocol["smoke_schedule"] is not None:
        raise ValueError("registered protocol smoke schedule schema is not exact")
    if kind == "development":
        exact(protocol["initial_state"], {
            "family_schema_version", "namespace", "family_semantic_sha256",
            "registered_trajectory_seeds", "audit_trajectory_seeds", "missions",
            "frozen_summary", "admission", "perturbation_policy",
        }, "initial state")
        if not isinstance(protocol["initial_state"]["missions"], list):
            raise ValueError("registered protocol initial-state missions are invalid")
        for mission in protocol["initial_state"]["missions"]:
            exact(mission, {"trajectory_seed", "positions_sha256"}, "initial-state mission")
        exact(
            protocol["initial_state"]["perturbation_policy"],
            {"clamp", "resample"}, "initial-state perturbation policy",
        )


def _string_argv(value) -> bool:
    return isinstance(value, list) and bool(value) and all(
        isinstance(token, str) and token for token in value
    )


def _verify_derived_protocol_contract(protocol: dict) -> None:
    kind = protocol.get("kind")
    if (
        protocol.get("schema_version")
            != (
                "cbf2026-qualified-closure-protocol-v2"
                if kind == "development"
                else "cbf2026-qualified-closure-protocol-v1"
            )
        or kind not in {"development", "confirmatory"}
        or protocol.get("version") != SUPPORTED_PROTOCOL_VERSIONS.get(kind)
        or protocol.get("conditions")
            != ["dynamic_primary", "fixed_fim_ablation"]
        or protocol.get("no_retry") is not True
    ):
        raise ValueError("registered derived protocol contract is invalid")
    if (
        protocol.get("supervision") != FROZEN_SUPERVISION
        or protocol.get("authorization") != FROZEN_AUTHORIZATION_REQUIREMENTS
    ):
        raise ValueError("registered supervision/authorization frozen contract is invalid")
    mission_count = 10 if kind == "development" else 60
    trajectory_start = 2026080201 if kind == "development" else 2026082001
    range_start = 2026081201 if kind == "development" else 2026083001
    initial_state = protocol.get("initial_state")
    initial_hashes = (
        {
            item["trajectory_seed"]: item["positions_sha256"]
            for item in initial_state.get("missions", [])
        }
        if isinstance(initial_state, dict) else {}
    )
    trajectory = list(range(trajectory_start, trajectory_start + mission_count))
    ranges = list(range(range_start, range_start + mission_count))
    missions = [
        {
            "mission_id": f"mission-{index:02d}",
            "trajectory_seed": trajectory_seed,
            "range_noise_seed": range_seed,
            "frames": 1000,
            **(
                {"initial_positions_sha256": initial_hashes.get(trajectory_seed)}
                if kind == "development" else {}
            ),
        }
        for index, (trajectory_seed, range_seed) in enumerate(
            zip(trajectory, ranges, strict=True), start=1
        )
    ]
    expected_schedule = {
        "trajectory_seeds": trajectory,
        "range_noise_seeds": ranges,
        "frames": 1000,
        "horizon_s": 500.0,
        "missions": missions,
    }
    if protocol.get("schedule") != expected_schedule:
        raise ValueError("registered derived protocol contract schedule is invalid")
    if protocol.get("universes") != _universes(mission_count, 1000):
        raise ValueError("registered derived protocol contract universe is invalid")
    expected_root_keys = (
        {"raw", "analysis"}
        if kind == "development"
        else {
            "smoke_a_raw", "smoke_a_analysis", "smoke_b_raw",
            "smoke_b_analysis", "raw", "analysis",
        }
    )
    if set(protocol.get("roots", {})) != expected_root_keys:
        raise ValueError("registered derived protocol contract roots are invalid")
    if protocol.get("roots") != FROZEN_EXECUTION_ROOTS[kind]:
        raise ValueError("registered literal execution roots are invalid")
    _verify_registered_argv(protocol)
    if kind == "development":
        if protocol.get("smoke_schedule") is not None:
            raise ValueError("registered derived protocol contract smoke is invalid")
        _verify_initial_state_binding(protocol)
        return
    semantic = {
        "campaign_id": "confirmatory-smoke",
        "trajectory_seed": 2026089001,
        "range_noise_seed": 2026089101,
        "frames": 20,
        "horizon_s": 10.0,
    }
    expected_smoke = {
        "trajectory_seed": 2026089001,
        "range_noise_seed": 2026089101,
        "frames": 20,
        "semantic_schedule_a": semantic,
        "semantic_schedule_b": dict(semantic),
        "universes": _universes(1, 20),
        "included_in_scientific_denominator": False,
    }
    if protocol.get("smoke_schedule") != expected_smoke:
        raise ValueError("registered derived protocol contract smoke is invalid")


def _registered_argv(protocol: dict) -> dict:
    """Rebuild every executable command from independently frozen protocol fields."""
    schedule = protocol["schedule"]
    roots = protocol["roots"]
    bindings = protocol["bindings"]
    publication = protocol["publication"]
    arguments = argparse.Namespace(
        kind=protocol["kind"],
        version=protocol["version"],
        protocol_json=Path(publication["json_path"]),
        binary=Path(_repository_path_token(
            bindings["binary"]["path"], Path(protocol["repository"]["root"])
        )),
        base_config=Path(_repository_path_token(
            bindings["base_config"]["path"], Path(protocol["repository"]["root"])
        )),
        primary_config=Path(_repository_path_token(
            bindings["primary_config"]["path"], Path(protocol["repository"]["root"])
        )),
        ablation_config=Path(_repository_path_token(
            bindings["ablation_config"]["path"], Path(protocol["repository"]["root"])
        )),
        initial_family=(
            Path(_repository_path_token(
                bindings["initial_family"]["path"],
                Path(protocol["repository"]["root"]),
            ))
            if protocol["kind"] == "development" else None
        ),
        trajectory_seeds=(
            f"{schedule['trajectory_seeds'][0]}:{schedule['trajectory_seeds'][-1]}"
        ),
        range_noise_seeds=(
            f"{schedule['range_noise_seeds'][0]}:{schedule['range_noise_seeds'][-1]}"
        ),
        frames=schedule["frames"],
        raw_root=Path(roots["raw"]),
        analysis_root=Path(roots["analysis"]),
        smoke_trajectory_seed=None,
        smoke_range_noise_seed=None,
        smoke_frames=None,
        smoke_a_raw_root=None,
        smoke_a_analysis_root=None,
        smoke_b_raw_root=None,
        smoke_b_analysis_root=None,
    )
    if protocol["kind"] == "confirmatory":
        smoke = protocol["smoke_schedule"]
        arguments.smoke_trajectory_seed = smoke["trajectory_seed"]
        arguments.smoke_range_noise_seed = smoke["range_noise_seed"]
        arguments.smoke_frames = smoke["frames"]
        for smoke_id in ("a", "b"):
            setattr(arguments, f"smoke_{smoke_id}_raw_root", Path(roots[f"smoke_{smoke_id}_raw"]))
            setattr(arguments, f"smoke_{smoke_id}_analysis_root", Path(roots[f"smoke_{smoke_id}_analysis"]))
    result = {
        "runner": _runner_argv(arguments),
        "analyzer": _analyzer_argv(arguments),
    }
    if protocol["kind"] == "confirmatory":
        result["smoke"] = _smoke_argv(arguments)
    return result


def _verify_registered_argv(protocol: dict) -> None:
    expected = _registered_argv(protocol)
    if (
        protocol["runner_argv"] != expected["runner"]
        or protocol["analyzer_argv"] != expected["analyzer"]
        or (
            protocol["kind"] == "confirmatory"
            and protocol["smoke_argv"] != expected["smoke"]
        )
    ):
        raise ValueError("registered runner/analyzer argv does not match the frozen contract")


def _semantic_sha256(protocol: dict) -> str:
    unbound = dict(protocol)
    unbound.pop("semantic_sha256", None)
    return hashlib.sha256(_canonical_bytes(unbound)).hexdigest()


if __name__ == "__main__":
    raise SystemExit(main())
