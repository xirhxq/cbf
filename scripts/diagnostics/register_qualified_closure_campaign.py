"""Register immutable qualified-closure campaign protocols."""

import hashlib
import argparse
import copy
import json
import math
import os
import stat
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
    "development_v6": {
        "raw": "/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development/v6",
        "analysis": "/private/tmp/cbf2026-qualified-mode-hybrid-dcbf-development-analysis/v6",
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
    "development": ("v5", "v6"),
    "confirmatory": ("v1",),
}

FROZEN_INITIAL_FAMILY_PATH = (
    Path(__file__).resolve().parents[2]
    / "config" / "diagnostics" / "qualified_initial_family_v1.json"
)
FROZEN_V6_INITIAL_FAMILY_PATH = (
    Path(__file__).resolve().parents[2]
    / "config" / "diagnostics" / "qualified_initial_family_v3.json"
)
FROZEN_V6_BINARY_PATH = (
    Path(__file__).resolve().parents[2] / "build-diagnostic" / "Swarm"
)
FROZEN_V6_BASE_CONFIG_PATH = (
    Path(__file__).resolve().parents[2] / "config" / "config.json"
)
FROZEN_V6_PRIMARY_CONFIG_PATH = (
    Path(__file__).resolve().parents[2]
    / "config" / "diagnostics"
    / "qualified_mode_hybrid_dcbf_development_v3.json"
)
FROZEN_V6_ABLATION_CONFIG_PATH = (
    Path(__file__).resolve().parents[2]
    / "config" / "diagnostics"
    / "qualified_mode_hybrid_dcbf_fixed_fim_ablation_v3.json"
)
FROZEN_V6_GATE_CLAIM_PATH = (
    Path(__file__).resolve().parents[2]
    / "docs/diagnostics/reviews/"
    "2026-08-02-cbf2026-qualified-v6-one-step-viability-v2-claim.json"
)
FROZEN_V6_GATE_PATH = (
    Path(__file__).resolve().parents[2]
    / "docs/diagnostics/reviews/"
    "2026-08-02-cbf2026-qualified-v6-one-step-viability-v2.json"
)
FROZEN_V6_GATE_REVIEW_PATH = (
    Path(__file__).resolve().parents[2]
    / "docs/diagnostics/reviews/"
    "2026-08-02-cbf2026-qualified-v6-one-step-viability-v2-review.md"
)
FROZEN_V6_CONTROLLER_MARGIN_AMENDMENT_REVIEW_PATH = (
    Path(__file__).resolve().parents[2]
    / "docs/superpowers/plans/reviews/"
    "2026-08-02-cbf2026-qualified-v6-controller-margin-amendment-review.md"
)
FROZEN_V6_CONTROLLER_MARGIN_IMPLEMENTATION_REVIEW_PATH = (
    Path(__file__).resolve().parents[2]
    / "docs/superpowers/plans/reviews/"
    "2026-08-02-cbf2026-qualified-v6-controller-margin-implementation-review.md"
)

FROZEN_V6_FILE_SHA256 = {
    "primary_config": "830bf6a0dc0c62596fe25f3a988e96bf28430fa2400783237eedf21e50d86d8a",
    "ablation_config": "2c2cbac9c772aa738812a5b18c1ae1c68efd74e220e822e8c8300cfae91f30b5",
    "initial_family": "8492960b57ba2bba0efd7453359060e81b434ee12b89abdf28aa9a691225fae5",
    "predecessor_identity": "b7fdceec59eaf9324cc646e9951bfc1585e6b17828346acae4d46ea37e9194d3",
    "controller_margin_amendment_review": "c4cb33250da05e12cceb9e729a72f7f29ccd77e0253ad397d97206468a222692",
    "controller_margin_implementation_review": "937e44a214d5ce6592654a3ce69e8c592186b8df56d5aabf7539de0339a43c04",
}

FROZEN_V6_CONTROLLER_MARGIN_IMPLEMENTATION = {
    "schema_version": "cbf2026-qualified-v6-controller-margin-implementation-v1",
    "implementation_commit": "de6939228f01e2600aeeeba2151ec2162f737cc4",
    "implementation_tree": "f9bdc4e72d962f4881f52a8ad69db2d42ea34554",
    "review_commit": "a8586c3518f437c68cdeb38a9e10605935f2a48d",
    "review_tree": "e754679247301f52887924d0a842ac68534e8123",
}

FROZEN_V6_GATE_SCHEMA = "cbf2026-qualified-v6-one-step-viability-v2"
FROZEN_V6_GATE_CAMPAIGN_ID = "qualified-v6-one-step-development-gate-v2"
FROZEN_V6_GATE_BOUNDARY = (
    "one-step development admission only; not long-horizon safety "
    "or recursive feasibility"
)
FROZEN_V6_TRAJECTORY_SEEDS = tuple(range(2026080201, 2026080211))
FROZEN_V6_RANGE_NOISE_SEEDS = tuple(range(2026081301, 2026081311))
FROZEN_V6_AUDIT_SEEDS = tuple(range(2026080201, 2026080301))
FROZEN_V6_TASK7_PATHS = (
    "scripts/diagnostics/register_qualified_closure_campaign.py",
    "scripts/diagnostics/run_qualified_closure_campaign.py",
    "tests/test_register_qualified_closure_campaign.py",
    "tests/test_run_qualified_closure_campaign.py",
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


def _require_literal_initial_family_path(path: Path, version: str = "v5") -> Path:
    """Authorize only the frozen family at its literal non-symbolic path."""
    path = Path(path)
    expected = (
        FROZEN_V6_INITIAL_FAMILY_PATH
        if version == "v6" else FROZEN_INITIAL_FAMILY_PATH
    )
    if (
        path.is_symlink()
        or _has_symbolic_ancestor(path)
        or path.resolve() != expected.resolve()
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
    if version not in SUPPORTED_PROTOCOL_VERSIONS[kind]:
        raise ValueError(
            "development requires v5/v6 and confirmatory requires v1"
        )
    is_v6 = kind == "development" and version == "v6"
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
    development = (
        set(FROZEN_V6_TRAJECTORY_SEEDS)
        | set(range(2026081201, 2026081211))
        | set(FROZEN_V6_RANGE_NOISE_SEEDS)
    )
    if kind == "confirmatory" and (set(trajectory) | set(ranges)) & development:
        raise ValueError("confirmatory seeds must be development-seed disjoint")
    expected_trajectory = list(
        range(2026080201, 2026080211)
        if kind == "development"
        else range(2026082001, 2026082061)
    )
    expected_ranges = list(
        (
            FROZEN_V6_RANGE_NOISE_SEEDS
            if is_v6 else range(2026081201, 2026081211)
        )
        if kind == "development" else range(2026083001, 2026083061)
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
        _require_literal_initial_family_path(
            bindings["initial_family"], version
        )
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
        initial_state = _derive_initial_state(
            Path(bindings["initial_family"]), version=version
        )
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
    if is_v6:
        protocol.update(_build_v6_lifecycle_declarations(kwargs, bound))
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
        if protocol.get("version") == "v6":
            _verify_v6_lifecycle_protocol(protocol)
        else:
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
    preflight_path = Path(_preflight_path(protocol_path))
    if preflight_path.is_symlink() or not preflight_path.is_file():
        raise ValueError("authorization preflight SHA-256 binding does not match")
    authorization = validate_authorization_payload(
        protocol,
        authorization,
        protocol_sha256=_sha256(protocol_path),
        preflight_sha256=_sha256(preflight_path),
    )
    observed = _repository_identity(repository_root)
    if observed["head"] == repository["head"]:
        raise ValueError("authorization artifacts require a committed artifact commit")
    verify_registered_protocol(
        protocol, allowed_claimed_roots=allowed_claimed_roots
    )
    return authorization


def validate_authorization_payload(
    protocol: Mapping,
    authorization: Mapping,
    *,
    protocol_sha256: str,
    preflight_sha256: str,
) -> dict:
    """Purely validate exact authorization values, bytes and supplied hashes.

    This precommit helper intentionally performs no Git, filesystem, or
    publication-topology inspection.  Production callers separately bind the
    supplied hashes to regular files and verify committed topology.
    """
    if not isinstance(protocol, Mapping):
        raise ValueError("authorization protocol is invalid")
    if not isinstance(authorization, Mapping) or set(authorization) != AUTHORIZATION_FIELDS:
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
    if (
        not _lower_hex(protocol_sha256, 64)
        or authorization["protocol_sha256"] != protocol_sha256
    ):
        raise ValueError("authorization protocol SHA-256 binding does not match")
    repository = protocol.get("repository")
    if (
        not isinstance(repository, Mapping)
        or authorization["implementation_identity"] != repository.get("head")
    ):
        raise ValueError("authorization implementation identity does not match")
    if (
        not _lower_hex(preflight_sha256, 64)
        or authorization["preflight_sha256"] != preflight_sha256
    ):
        raise ValueError("authorization preflight SHA-256 binding does not match")
    authorization_text = authorization["user_authorization_text"]
    if (
        not isinstance(authorization_text, str)
        or not authorization_text
    ):
        raise ValueError("authorization text binding does not match")
    try:
        authorization_text_bytes = authorization_text.encode("utf-8", errors="strict")
    except UnicodeEncodeError:
        raise ValueError("authorization text is not valid UTF-8") from None
    if (
        hashlib.sha256(authorization_text_bytes).hexdigest()
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
    return dict(authorization)


def _seed_list(value, label: str) -> list[int]:
    if not isinstance(value, list) or not all(
        type(seed) is int and 0 <= seed < 2**64 for seed in value
    ):
        raise ValueError(f"{label} seeds must be serialized uint64 integers")
    return list(value)


def _reference_selection(config: dict):
    covariance = config.get("position_covariance")
    return covariance.get("reference-selection") if isinstance(covariance, dict) else None


def _read_stable_regular_file(path: Path, label: str) -> tuple[bytes, dict]:
    """Read one non-symbolic regular file from one stable descriptor."""
    path = Path(path)
    if path.is_symlink() or _has_symbolic_ancestor(path):
        raise ValueError(f"{label} must be a non-symbolic regular file")
    flags = os.O_RDONLY
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise ValueError(f"{label} must be a regular file") from error
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise ValueError(f"{label} must be a regular file")
        chunks = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        after = os.fstat(descriptor)
        stable_fields = (
            "st_dev", "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns",
        )
        if tuple(getattr(before, key) for key in stable_fields) != tuple(
            getattr(after, key) for key in stable_fields
        ):
            raise ValueError(f"{label} changed during its stable read")
    finally:
        os.close(descriptor)
    raw = b"".join(chunks)
    if len(raw) != before.st_size:
        raise ValueError(f"{label} changed during its stable read")
    return raw, {
        "path": str(path.resolve()),
        "bytes": len(raw),
        "sha256": hashlib.sha256(raw).hexdigest(),
    }


def _json_object_bytes(raw: bytes, label: str) -> dict:
    def reject_duplicate(pairs):
        output = {}
        for key, value in pairs:
            if key in output:
                raise ValueError(f"{label} contains duplicate key: {key}")
            output[key] = value
        return output

    try:
        value = json.loads(
            raw.decode("utf-8"), object_pairs_hook=reject_duplicate
        )
    except (UnicodeError, json.JSONDecodeError, ValueError) as error:
        raise ValueError(f"{label} is unreadable") from error
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object")
    return value


def _json_object(path: Path, label: str) -> dict:
    try:
        return _json_object_bytes(Path(path).read_bytes(), label)
    except (OSError, ValueError) as error:
        raise ValueError(f"{label} is unreadable") from error


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


def _require_exact_regular_path(path, expected: Path, label: str) -> Path:
    try:
        path = Path(path)
    except TypeError:
        raise ValueError(f"{label} path is absent") from None
    if (
        path.is_symlink()
        or _has_symbolic_ancestor(path)
        or not path.is_file()
        or path.resolve() != Path(expected).resolve()
    ):
        raise ValueError(f"{label} path is not the exact registered path")
    return path


def _require_frozen_file(
    path, expected_path: Path, label: str, expected_sha256: str | None = None
) -> dict:
    path = _require_exact_regular_path(path, expected_path, label)
    identity = _file_identity(path)
    if expected_sha256 is not None and identity["sha256"] != expected_sha256:
        raise ValueError(f"{label} identity differs from the frozen contract")
    return identity


def _validate_gate_source(value, label: str) -> dict:
    if not isinstance(value, Mapping) or set(value) != {"root", "head", "tree"}:
        raise ValueError(f"one-step gate {label} schema is invalid")
    if (
        not isinstance(value["root"], str)
        or not value["root"]
        or not _lower_hex(value["head"], 40)
        or not _lower_hex(value["tree"], 40)
    ):
        raise ValueError(f"one-step gate {label} identity is invalid")
    return dict(value)


def _validate_gate_file_identity(value, label: str) -> dict:
    if not isinstance(value, Mapping) or set(value) != {"path", "bytes", "sha256"}:
        raise ValueError(f"one-step gate {label} identity schema is invalid")
    if (
        not isinstance(value["path"], str)
        or not value["path"]
        or type(value["bytes"]) is not int
        or value["bytes"] < 0
        or not _lower_hex(value["sha256"], 64)
    ):
        raise ValueError(f"one-step gate {label} identity is invalid")
    return dict(value)


def _finite_float(value) -> bool:
    return type(value) is float and math.isfinite(value)


def _require_exact_json_tokens(actual, expected, label: str) -> None:
    """Compare JSON values without conflating bool/int/float token types."""
    if type(actual) is not type(expected):
        raise ValueError(f"one-step gate {label} JSON token type differs")
    if isinstance(expected, dict):
        if set(actual) != set(expected):
            raise ValueError(f"one-step gate {label} schema differs")
        for key in expected:
            _require_exact_json_tokens(actual[key], expected[key], f"{label}.{key}")
        return
    if isinstance(expected, list):
        if len(actual) != len(expected):
            raise ValueError(f"one-step gate {label} length differs")
        for index, (observed, frozen) in enumerate(zip(actual, expected, strict=True)):
            _require_exact_json_tokens(observed, frozen, f"{label}[{index}]")
        return
    if type(expected) is float and (
        not math.isfinite(actual) or not math.isfinite(expected)
    ):
        raise ValueError(f"one-step gate {label} is not finite")
    if actual != expected:
        raise ValueError(f"one-step gate {label} value differs")


def _require_canonical_problem_tokens(
    actual, expected, label: str, *, numeric_tolerance: float = 1e-9
) -> None:
    """Match the full canonical hard problem, tolerating only finite floats."""
    if type(actual) is not type(expected):
        raise ValueError(f"one-step gate {label} token type differs")
    if isinstance(expected, dict):
        if set(actual) != set(expected):
            raise ValueError(f"one-step gate {label} schema differs")
        for key in expected:
            _require_canonical_problem_tokens(
                actual[key], expected[key], f"{label}.{key}",
                numeric_tolerance=numeric_tolerance,
            )
        return
    if isinstance(expected, list):
        if len(actual) != len(expected):
            raise ValueError(f"one-step gate {label} length differs")
        for index, (observed, canonical) in enumerate(
            zip(actual, expected, strict=True)
        ):
            _require_canonical_problem_tokens(
                observed, canonical, f"{label}[{index}]",
                numeric_tolerance=numeric_tolerance,
            )
        return
    if type(expected) is float:
        if not math.isfinite(actual) or not math.isclose(
            actual, expected, rel_tol=0.0, abs_tol=numeric_tolerance
        ):
            raise ValueError(f"one-step gate {label} numeric value differs")
        return
    if actual != expected:
        raise ValueError(f"one-step gate {label} value differs")


def _v6_edge_payload(edge) -> dict:
    return {
        "kind": edge.kind,
        "low": edge.low,
        "high": edge.high,
        "base_id": edge.base_id,
    }


def _v6_certificate_payload(certificate) -> dict:
    return {
        "robot_id": certificate.robot_id,
        "reference_ids": [
            {"kind": kind, "id": reference_id}
            for kind, reference_id in certificate.reference_ids
        ],
        "covariance": [list(row) for row in certificate.covariance],
        "covariance_rate_bound": certificate.covariance_rate_bound,
        "epsilon": certificate.epsilon,
        "bar_nu": certificate.bar_nu,
    }


def _validate_v6_seed_result(row: Mapping, *, seed: int, family: Mapping) -> dict:
    """Independently replay every retained, reconstructible v6 seed relation."""
    from scripts.diagnostics.hard_interior_selection import (
        frozen_interior_floor,
        solve_planar_hard_row_chebyshev,
    )
    from scripts.diagnostics.qualified_v6_initial_state import (
        _local_hard_problem,
        materialize_v6_seed_positions,
        reconstruct_v6_one_step_state,
    )

    row_fields = {
        "seed", "launched", "status", "passed", "reasons",
        "config_sha256", "positions_sha256",
        "minimum_applied_original_residual_mps",
        "minimum_enforced_floor_mps", "maximum_planar_component_mps",
        "minimum_next_barrier_m", "minimum_next_local_radius_mps",
        "barrier_count", "local_radius_count", "recomputation_evidence",
    }
    if not isinstance(row, Mapping) or set(row) != row_fields:
        raise ValueError("one-step gate seed result schema is not exact")
    if (
        row.get("seed") != seed or type(row.get("seed")) is not int
        or row.get("launched") is not True
        or row.get("status") != "passed"
        or row.get("passed") is not True
        or row.get("reasons") != []
    ):
        raise ValueError("one-step gate seed result failed or mutated")
    # The producer hashes a config containing a randomized temporary
    # output_path and deliberately retains no such path.  Its digest can be
    # revalidated as a canonical SHA token and is made immutable by the exact
    # gate-commit/blob binding below, but cannot be regenerated post hoc.
    if not _lower_hex(row.get("config_sha256"), 64):
        raise ValueError("one-step gate seed result config hash is not canonical")

    evidence = row.get("recomputation_evidence")
    evidence_fields = {
        "current_positions_m", "robots", "next_positions_m",
        "next_dynamic_fim_certificates", "next_barriers",
        "next_local_problems",
    }
    if not isinstance(evidence, Mapping) or set(evidence) != evidence_fields:
        raise ValueError("one-step gate recomputation evidence schema is not exact")
    expected_positions = materialize_v6_seed_positions(family, seed)
    expected_position_payload = [list(position) for position in expected_positions]
    _require_exact_json_tokens(
        evidence.get("current_positions_m"), expected_position_payload,
        "seed result current positions",
    )

    robots = evidence.get("robots")
    if not isinstance(robots, list) or len(robots) != 14:
        raise ValueError("one-step gate seed result robot evidence is incomplete")
    commands = []
    normal_problems = []
    residual_floor_pairs = []
    robot_fields = {
        "robot_id", "applied_command", "normal_problem",
        "hard_interior_selection",
    }
    policy_fields = {
        "schema_version", "mode", "fraction", "cap_mps",
        "feasibility_tolerance_mps", "planar_chebyshev_radius_mps",
        "enforced_floor_mps", "minimum_original_hard_residual_mps",
    }
    for robot_id, robot in enumerate(robots, start=1):
        if (
            not isinstance(robot, Mapping) or set(robot) != robot_fields
            or robot.get("robot_id") != robot_id
            or type(robot.get("robot_id")) is not int
        ):
            raise ValueError("one-step gate seed result robot evidence schema differs")
        command = robot.get("applied_command")
        if (
            not isinstance(command, list) or len(command) != 3
            or any(not _finite_float(component) for component in command)
        ):
            raise ValueError("one-step gate seed result applied command is invalid")
        commands.append(tuple(command))
        problem = robot.get("normal_problem")
        policy = robot.get("hard_interior_selection")
        if not isinstance(policy, Mapping) or set(policy) != policy_fields:
            raise ValueError("one-step gate seed result policy evidence schema differs")
        if (
            policy.get("schema_version") != "hard-interior-v3"
            or policy.get("mode") != "planar-chebyshev-fraction-cap-v2"
            or policy.get("fraction") != 0.131
            or policy.get("cap_mps") != 0.1
            or policy.get("feasibility_tolerance_mps") != 1e-9
            or any(
                not _finite_float(policy.get(label))
                for label in (
                    "fraction", "cap_mps", "feasibility_tolerance_mps",
                    "planar_chebyshev_radius_mps", "enforced_floor_mps",
                    "minimum_original_hard_residual_mps",
                )
            )
        ):
            raise ValueError("one-step gate seed result policy evidence differs")
        try:
            interior = solve_planar_hard_row_chebyshev(
                problem, tolerance_mps=policy["feasibility_tolerance_mps"]
            )
        except (TypeError, ValueError) as error:
            raise ValueError("one-step gate seed result hard problem differs") from error
        if problem.get("owner") != robot_id:
            raise ValueError("one-step gate seed result hard problem owner differs")
        observed_problem_rows = problem.get("rows")
        normal_problems.append(problem)
        expected_floor = frozen_interior_floor(
            interior.radius_mps, fraction=0.131, cap_mps=0.1,
            tolerance_mps=1e-9,
        )
        if (
            not math.isclose(
                policy["planar_chebyshev_radius_mps"], interior.radius_mps,
                rel_tol=0.0, abs_tol=1e-12,
            )
            or not math.isclose(
                policy["enforced_floor_mps"], expected_floor,
                rel_tol=0.0, abs_tol=1e-12,
            )
        ):
            raise ValueError("one-step gate seed result hard-interior reconstruction differs")
        residuals = [
            hard_row["constant"] + sum(
                coefficient * component
                for coefficient, component in zip(
                    hard_row["coefficients"], command, strict=True
                )
            )
            for hard_row in observed_problem_rows
        ]
        if not residuals or not math.isclose(
            policy["minimum_original_hard_residual_mps"], min(residuals),
            rel_tol=1e-9, abs_tol=1e-9,
        ):
            raise ValueError("one-step gate seed result residual reconstruction differs")
        residual_floor_pairs.extend((residual, expected_floor) for residual in residuals)

    reconstruction = reconstruct_v6_one_step_state(
        family, expected_positions,
        tuple(command[:2] for command in commands),
    )
    expected_current_rows = {
        robot_id: tuple(
            endpoint for endpoint in reconstruction.current_audit.endpoint_rows
            if endpoint.owner == robot_id
        )
        for robot_id in range(1, 15)
    }
    for robot_id, problem in enumerate(normal_problems, start=1):
        canonical_problem = _local_hard_problem(
            robot_id, expected_current_rows[robot_id], 25.0
        )
        _require_canonical_problem_tokens(
            problem, canonical_problem,
            f"seed result current hard problem UAV {robot_id}",
        )
    expected_next_positions = [
        list(position) for position in reconstruction.next_positions_m
    ]
    expected_certificates = [
        _v6_certificate_payload(certificate)
        for certificate in reconstruction.next_certificates
    ]
    expected_barriers = [
        {"edge": _v6_edge_payload(barrier.edge), "value_m": barrier.value}
        for barrier in reconstruction.next_barriers
    ]
    next_rows = tuple(
        tuple(
            endpoint for endpoint in reconstruction.next_endpoint_rows
            if endpoint.owner == robot_id
        )
        for robot_id in range(1, 15)
    )
    expected_local_problems = [
        {
            "robot_id": robot_id,
            "problem": _local_hard_problem(
                robot_id, next_rows[robot_id - 1], 25.0
            ),
            "radius_mps": reconstruction.next_local_radii_mps[robot_id - 1],
        }
        for robot_id in range(1, 15)
    ]
    for label, observed, expected in (
        ("next positions", evidence.get("next_positions_m"), expected_next_positions),
        (
            "next dynamic FIM certificates",
            evidence.get("next_dynamic_fim_certificates"), expected_certificates,
        ),
        ("next barriers", evidence.get("next_barriers"), expected_barriers),
        (
            "next local problems", evidence.get("next_local_problems"),
            expected_local_problems,
        ),
    ):
        _require_exact_json_tokens(observed, expected, f"seed result {label}")

    computed = {
        "positions_sha256": reconstruction.current_audit.positions_sha256,
        "minimum_applied_original_residual_mps": min(
            residual for residual, _floor in residual_floor_pairs
        ),
        "minimum_enforced_floor_mps": min(
            floor for _residual, floor in residual_floor_pairs
        ),
        "maximum_planar_component_mps": max(
            abs(component) for command in commands for component in command[:2]
        ),
        "minimum_next_barrier_m": min(
            barrier.value for barrier in reconstruction.next_barriers
        ),
        "minimum_next_local_radius_mps": min(
            reconstruction.next_local_radii_mps
        ),
        "barrier_count": len(reconstruction.next_barriers),
        "local_radius_count": len(reconstruction.next_local_radii_mps),
    }
    for label, expected in computed.items():
        observed = row.get(label)
        if type(expected) is float:
            if not _finite_float(observed) or not math.isclose(
                observed, expected, rel_tol=0.0, abs_tol=1e-12
            ):
                raise ValueError(f"one-step gate seed result {label} differs")
        elif type(observed) is not type(expected) or observed != expected:
            raise ValueError(f"one-step gate seed result {label} differs")
    if (
        computed["minimum_next_barrier_m"] <= 0.0
        or computed["minimum_next_local_radius_mps"] < 0.05
        or computed["maximum_planar_component_mps"] > 25.0 + 1e-7
        or any(
            residual < floor - 1e-7
            for residual, floor in residual_floor_pairs
        )
    ):
        raise ValueError("one-step gate seed result failed the frozen predicate")
    return dict(row)


def _validate_v6_gate(
    *, claim_path: Path, gate_path: Path, gate_review_path: Path,
    protocol_bindings: Mapping,
) -> dict:
    claim_path = _require_exact_regular_path(
        claim_path, FROZEN_V6_GATE_CLAIM_PATH, "one-step gate claim"
    )
    gate_path = _require_exact_regular_path(
        gate_path, FROZEN_V6_GATE_PATH, "one-step gate"
    )
    gate_review_path = _require_exact_regular_path(
        gate_review_path, FROZEN_V6_GATE_REVIEW_PATH, "one-step gate review"
    )
    claim_bytes, claim_identity = _read_stable_regular_file(
        claim_path, "one-step gate claim"
    )
    gate_bytes, gate_identity = _read_stable_regular_file(
        gate_path, "one-step gate"
    )
    review_bytes, review_identity = _read_stable_regular_file(
        gate_review_path, "one-step gate review"
    )
    claim = _json_object_bytes(claim_bytes, "one-step gate claim")
    gate = _json_object_bytes(gate_bytes, "one-step gate")
    claim_fields = {
        "schema_version", "claimed", "source", "bindings", "seed_universe",
        "execution_provenance", "qualifying", "gate", "boundary",
    }
    gate_fields = {
        "schema_version", "terminal", "status", "passed", "predicate_passed",
        "reason", "terminal_failure_reason", "claim_sha256",
        "claimed_identity_sha256", "source", "terminal_source", "bindings",
        "seed_universe", "execution_provenance", "qualifying", "seed_results",
        "audit_summary", "registered_summary", "launch_count", "retry_count",
        "boundary",
    }
    if set(claim) != claim_fields or set(gate) != gate_fields:
        raise ValueError("one-step gate schema is not exact")
    claim_sha256 = claim_identity["sha256"]
    if (
        claim.get("schema_version") != f"{FROZEN_V6_GATE_SCHEMA}-claim"
        or claim.get("claimed") is not True
        or gate.get("schema_version") != FROZEN_V6_GATE_SCHEMA
        or gate.get("terminal") is not True
        or gate.get("status") != "completed"
        or gate.get("passed") is not True
        or gate.get("predicate_passed") is not True
        or gate.get("reason") != "completed"
        or gate.get("terminal_failure_reason") is not None
        or gate.get("execution_provenance") != "exact-binary-subprocess"
        or claim.get("execution_provenance") != "exact-binary-subprocess"
        or gate.get("qualifying") is not True
        or claim.get("qualifying") is not True
        or gate.get("claim_sha256") != claim_sha256
        or gate.get("claimed_identity_sha256") != claim_sha256
        or gate.get("launch_count") != 100
        or type(gate.get("launch_count")) is not int
        or gate.get("retry_count") != 0
        or type(gate.get("retry_count")) is not int
    ):
        raise ValueError("one-step gate is absent, failed, or not qualifying")
    source = _validate_gate_source(gate.get("source"), "source")
    terminal_source = _validate_gate_source(
        gate.get("terminal_source"), "terminal source"
    )
    if source != terminal_source or source != claim.get("source"):
        raise ValueError("one-step gate HEAD/tree identity mismatch")
    expected_seeds = list(FROZEN_V6_AUDIT_SEEDS)
    if gate.get("seed_universe") != expected_seeds or claim.get("seed_universe") != expected_seeds:
        raise ValueError("one-step gate seed universe or order mismatch")
    expected_gate_contract = {
        "dt_s": 0.5,
        "minimum_next_barrier_m": 0.0,
        "barrier_comparison": "strictly-greater",
        "minimum_next_local_radius_mps": 0.05,
        "component_max_mps": 25.0,
        "numeric_tolerance": 1e-7,
        "clamp": False,
        "resample": False,
        "retry": False,
    }
    _require_exact_json_tokens(
        claim.get("gate"), expected_gate_contract, "threshold contract"
    )
    if (
        claim.get("boundary") != FROZEN_V6_GATE_BOUNDARY
        or gate.get("boundary") != FROZEN_V6_GATE_BOUNDARY
    ):
        raise ValueError("one-step gate scientific boundary mismatch")

    binding_labels = {
        "implementation", "binary", "base_config", "primary_config",
        "initial_family",
    }
    gate_bindings = gate.get("bindings")
    claim_bindings = claim.get("bindings")
    if (
        not isinstance(gate_bindings, Mapping)
        or set(gate_bindings) != binding_labels
        or gate_bindings != claim_bindings
    ):
        raise ValueError("one-step gate binding schema or claim binding mismatch")
    implementation = gate_bindings.get("implementation")
    if (
        not isinstance(implementation, Mapping)
        or set(implementation)
            != {"path", "repository_path", "repository_head", "bytes", "sha256"}
        or implementation.get("repository_path")
            != "scripts/diagnostics/audit_qualified_v6_one_step_viability.py"
        or implementation.get("repository_head") != source["head"]
    ):
        raise ValueError("one-step gate implementation/HEAD binding mismatch")
    implementation_file = {
        key: implementation[key] for key in ("path", "bytes", "sha256")
    }
    observed_implementation = _file_identity(
        Path(__file__).with_name("audit_qualified_v6_one_step_viability.py")
    )
    if implementation_file != observed_implementation:
        raise ValueError("one-step gate producer identity mismatch")
    for gate_label, protocol_label in (
        ("binary", "binary"),
        ("base_config", "base_config"),
        ("primary_config", "primary_config"),
        ("initial_family", "initial_family"),
    ):
        identity = _validate_gate_file_identity(
            gate_bindings.get(gate_label), gate_label
        )
        if identity != protocol_bindings.get(protocol_label):
            raise ValueError(f"one-step gate {gate_label} binding mismatch")

    rows = gate.get("seed_results")
    if not isinstance(rows, list) or len(rows) != 100:
        raise ValueError("one-step gate seed results are incomplete")
    from scripts.diagnostics.qualified_v6_initial_state import (
        load_qualified_v6_initial_family,
    )
    family = load_qualified_v6_initial_family(
        Path(protocol_bindings["initial_family"]["path"])
    )
    for seed, row in zip(FROZEN_V6_AUDIT_SEEDS, rows, strict=True):
        _validate_v6_seed_result(row, seed=seed, family=family)
    expected_summary_counts = {
        "audit_summary": (100, 100, 100),
        "registered_summary": (10, 10, 10),
    }
    for label, counts in expected_summary_counts.items():
        summary = gate.get(label)
        if (
            not isinstance(summary, Mapping)
            or set(summary) != {
                "proposed_count", "launched_count", "passed_count",
                "minimum_next_barrier_m", "minimum_next_local_radius_mps",
            }
            or tuple(summary.get(key) for key in (
                "proposed_count", "launched_count", "passed_count"
            )) != counts
            or not _finite_float(summary.get("minimum_next_barrier_m"))
            or summary["minimum_next_barrier_m"] <= 0.0
            or not _finite_float(summary.get("minimum_next_local_radius_mps"))
            or summary["minimum_next_local_radius_mps"] < 0.05
        ):
            raise ValueError(f"one-step gate {label} is not a complete PASS")
        selected_rows = rows if label == "audit_summary" else rows[:10]
        if (
            summary["minimum_next_barrier_m"]
                != min(row["minimum_next_barrier_m"] for row in selected_rows)
            or summary["minimum_next_local_radius_mps"]
                != min(row["minimum_next_local_radius_mps"] for row in selected_rows)
        ):
            raise ValueError(f"one-step gate {label} minima mismatch seed results")
    try:
        review_text = review_bytes.decode("utf-8", errors="strict")
    except UnicodeError as error:
        raise ValueError("one-step gate review is not valid UTF-8") from error
    normalized_review = review_text.replace(" ", "").lower()
    alphanumeric_review = "".join(
        character for character in review_text.lower()
        if character.isalnum()
    )
    explicit_zero_counts = all(
        token in alphanumeric_review
        for token in ("critical0", "important0", "minor0")
    )
    if "c0/i0/m0" not in normalized_review and not explicit_zero_counts:
        raise ValueError("one-step gate review is not C0/I0/M0")
    return {
        "schema_version": FROZEN_V6_GATE_SCHEMA,
        "campaign_id": FROZEN_V6_GATE_CAMPAIGN_ID,
        "status": "completed",
        "passed": True,
        "claim": claim_identity,
        "artifact": gate_identity,
        "review": review_identity,
        "source": source,
        "bindings": copy.deepcopy(gate_bindings),
        "seed_universe": expected_seeds,
        "launch_count": 100,
        "retry_count": 0,
        "audit_summary": dict(gate["audit_summary"]),
        "registered_summary": dict(gate["registered_summary"]),
    }


def _build_v6_lifecycle_declarations(kwargs: Mapping, bound: Mapping) -> dict:
    binary = _require_frozen_file(
        kwargs["bindings"]["binary"],
        FROZEN_V6_BINARY_PATH,
        "v6 production binary",
    )
    base = _require_frozen_file(
        kwargs["bindings"]["base_config"],
        FROZEN_V6_BASE_CONFIG_PATH,
        "v6 base config",
    )
    primary = _require_frozen_file(
        kwargs["bindings"]["primary_config"],
        FROZEN_V6_PRIMARY_CONFIG_PATH,
        "v6 primary config",
        FROZEN_V6_FILE_SHA256["primary_config"],
    )
    ablation = _require_frozen_file(
        kwargs["bindings"]["ablation_config"],
        FROZEN_V6_ABLATION_CONFIG_PATH,
        "v6 ablation config",
        FROZEN_V6_FILE_SHA256["ablation_config"],
    )
    family = _require_frozen_file(
        kwargs["bindings"]["initial_family"],
        FROZEN_V6_INITIAL_FAMILY_PATH,
        "v6 initial family",
        FROZEN_V6_FILE_SHA256["initial_family"],
    )
    if (
        binary != bound.get("binary")
        or base != bound.get("base_config")
        or primary != bound.get("primary_config")
        or ablation != bound.get("ablation_config")
        or family != bound.get("initial_family")
    ):
        raise ValueError("v6 config/family identities differ after binding")
    predecessor_path = _require_exact_regular_path(
        kwargs.get("predecessor_identity"),
        FROZEN_V6_PREDECESSOR_IDENTITY_PATH,
        "v5 predecessor identity",
    )
    if _sha256(predecessor_path) != FROZEN_V6_FILE_SHA256["predecessor_identity"]:
        raise ValueError("v5 predecessor identity file hash differs")
    predecessor = _json_object(predecessor_path, "v5 predecessor identity")
    _validate_v6_predecessor_identity(predecessor)
    verify_v6_predecessor_state(predecessor)
    amendment = _require_frozen_file(
        kwargs.get("controller_margin_amendment_review"),
        FROZEN_V6_CONTROLLER_MARGIN_AMENDMENT_REVIEW_PATH,
        "controller-margin amendment review",
        FROZEN_V6_FILE_SHA256["controller_margin_amendment_review"],
    )
    implementation_review = _require_frozen_file(
        kwargs.get("controller_margin_implementation_review"),
        FROZEN_V6_CONTROLLER_MARGIN_IMPLEMENTATION_REVIEW_PATH,
        "controller-margin implementation review",
        FROZEN_V6_FILE_SHA256["controller_margin_implementation_review"],
    )
    development_gate = _validate_v6_gate(
        claim_path=kwargs.get("development_gate_claim"),
        gate_path=kwargs.get("development_gate"),
        gate_review_path=kwargs.get("development_gate_review"),
        protocol_bindings=bound,
    )
    return {
        "development_gate": development_gate,
        "predecessor": {
            "identity_file": _file_identity(predecessor_path),
            **copy.deepcopy(predecessor),
        },
        "controller_margin_implementation": {
            **FROZEN_V6_CONTROLLER_MARGIN_IMPLEMENTATION,
            "amendment_review": amendment,
            "implementation_review": implementation_review,
        },
    }


def _git_name_status(project_root: Path, older: str, newer: str) -> list[tuple[str, str]]:
    raw = _git_bytes(
        project_root, "diff", "--name-status", "--no-renames", older, newer
    ).decode("utf-8", errors="strict")
    changes = []
    for line in raw.splitlines():
        fields = line.split("\t")
        if len(fields) != 2:
            raise ValueError("one-step gate Git diff is malformed")
        changes.append((fields[0], fields[1]))
    return changes


def _verify_v6_gate_git_topology(
    repository: Mapping, declaration: Mapping, controller: Mapping,
) -> str:
    """Bind Task 7 source and the reviewed gate to exact Git transitions."""
    if not isinstance(repository, Mapping) or not isinstance(declaration, Mapping):
        raise ValueError("one-step gate Git declarations are invalid")
    project_root = Path(repository.get("root", "")).resolve()
    repository_head = repository.get("head")
    repository_tree = repository.get("tree")
    if (
        not _lower_hex(repository_head, 40)
        or not _lower_hex(repository_tree, 40)
        or _git_bytes(
            project_root, "rev-parse", f"{repository_head}^{{tree}}"
        ).decode("ascii", errors="strict").strip() != repository_tree
    ):
        raise ValueError("one-step gate repository HEAD/tree identity differs")
    source = declaration.get("source")
    if (
        not isinstance(source, Mapping)
        or Path(source.get("root", "")).resolve() != project_root
        or not _lower_hex(source.get("head"), 40)
        or not _lower_hex(source.get("tree"), 40)
    ):
        raise ValueError("one-step gate source repository identity differs")
    source_head = source["head"]
    source_tree = _git_bytes(
        project_root, "rev-parse", f"{source_head}^{{tree}}"
    ).decode("ascii", errors="strict").strip()
    if source_tree != source["tree"]:
        raise ValueError("one-step gate source HEAD/tree identity differs")
    task6b_review = controller.get("review_commit")
    task6b_tree = controller.get("review_tree")
    if (
        not _lower_hex(task6b_review, 40)
        or not _lower_hex(task6b_tree, 40)
        or _git_bytes(
            project_root, "rev-parse", f"{task6b_review}^{{tree}}"
        ).decode("ascii", errors="strict").strip() != task6b_tree
    ):
        raise ValueError("controller-margin review commit/tree identity differs")
    source_parents = _git_bytes(
        project_root, "rev-list", "--parents", "-n", "1", source_head
    ).decode("ascii", errors="strict").split()
    if source_parents != [source_head, task6b_review]:
        raise ValueError("Task 7 source is not the direct child of Task 6b review")
    task7_changes = _git_name_status(project_root, task6b_review, source_head)
    if (
        sorted(path for _status, path in task7_changes)
            != sorted(FROZEN_V6_TASK7_PATHS)
        or any(status not in {"A", "M"} for status, _path in task7_changes)
    ):
        raise ValueError("Task 7 source commit is not the exact four-file change")
    for token in FROZEN_V6_TASK7_PATHS:
        source_blob = _git_bytes(project_root, "show", f"{source_head}:{token}")
        live, _identity = _read_stable_regular_file(
            project_root / token, f"Task 7 source {token}"
        )
        if live != source_blob:
            raise ValueError("live Task 7 source differs from source commit blob")

    ancestry = _git_bytes(
        project_root, "rev-list", "--ancestry-path", "--reverse",
        f"{source_head}..{repository_head}",
    ).decode("ascii", errors="strict").splitlines()
    if not ancestry:
        raise ValueError("one-step gate artifact commit is absent")
    gate_commit = ancestry[0]
    gate_parents = _git_bytes(
        project_root, "rev-list", "--parents", "-n", "1", gate_commit
    ).decode("ascii", errors="strict").split()
    if gate_parents != [gate_commit, source_head]:
        raise ValueError("one-step gate artifact commit is not a direct child")

    gate_paths = []
    for label in ("claim", "artifact", "review"):
        identity = declaration.get(label)
        _validate_gate_file_identity(identity, label)
        try:
            token = Path(identity["path"]).resolve().relative_to(project_root).as_posix()
        except ValueError:
            raise ValueError("one-step gate artifact is outside the repository") from None
        gate_paths.append(token)
    gate_changes = _git_name_status(project_root, source_head, gate_commit)
    if sorted(gate_changes) != sorted(("A", token) for token in gate_paths):
        raise ValueError("one-step gate artifact commit is not exact three add-only")

    implementation = declaration.get("bindings", {}).get("implementation")
    if not isinstance(implementation, Mapping):
        raise ValueError("one-step gate producer Git binding is absent")
    implementation_token = implementation.get("repository_path")
    if not isinstance(implementation_token, str) or not implementation_token:
        raise ValueError("one-step gate producer Git path is invalid")
    implementation_path = project_root / implementation_token
    implementation_blob = _git_bytes(
        project_root, "show", f"{source_head}:{implementation_token}"
    )
    implementation_live, implementation_live_identity = _read_stable_regular_file(
        implementation_path, "one-step gate producer"
    )
    if (
        implementation.get("repository_head") != source_head
        or implementation.get("sha256")
            != hashlib.sha256(implementation_blob).hexdigest()
        or implementation.get("bytes") != len(implementation_blob)
        or implementation_live_identity != {
            key: implementation[key] for key in ("path", "sha256", "bytes")
        }
        or implementation_live != implementation_blob
    ):
        raise ValueError("one-step gate producer differs from source Git blob")

    for label, token in zip(("claim", "artifact", "review"), gate_paths, strict=True):
        identity = declaration[label]
        live_path = Path(identity["path"])
        blob = _git_bytes(project_root, "show", f"{gate_commit}:{token}")
        live, live_identity = _read_stable_regular_file(
            live_path, f"one-step gate {label}"
        )
        if (
            hashlib.sha256(blob).hexdigest() != identity["sha256"]
            or len(blob) != identity["bytes"]
            or live_identity != dict(identity)
            or live != blob
        ):
            raise ValueError(f"one-step gate {label} differs from gate commit blob")
    return gate_commit


def _verify_v6_lifecycle_protocol(protocol: Mapping) -> None:
    """Recompute every v6 gate, predecessor and Task 6b declaration."""
    bindings = protocol.get("bindings")
    if not isinstance(bindings, Mapping):
        raise ValueError("registered v6 bindings are invalid")
    exact_paths = {
        "binary": FROZEN_V6_BINARY_PATH,
        "base_config": FROZEN_V6_BASE_CONFIG_PATH,
        "primary_config": FROZEN_V6_PRIMARY_CONFIG_PATH,
        "ablation_config": FROZEN_V6_ABLATION_CONFIG_PATH,
        "initial_family": FROZEN_V6_INITIAL_FAMILY_PATH,
    }
    for label, path in exact_paths.items():
        if bindings.get(label) != _file_identity(path):
            raise ValueError(f"registered v6 {label} exact identity mutated")
    for label in ("primary_config", "ablation_config", "initial_family"):
        if bindings[label]["sha256"] != FROZEN_V6_FILE_SHA256[label]:
            raise ValueError(f"registered v6 {label} frozen hash mutated")
    predecessor_declaration = protocol.get("predecessor")
    if not isinstance(predecessor_declaration, Mapping):
        raise ValueError("registered v5 predecessor declaration is invalid")
    predecessor_identity = predecessor_declaration.get("identity_file")
    predecessor = {
        key: copy.deepcopy(value)
        for key, value in predecessor_declaration.items()
        if key != "identity_file"
    }
    if (
        not isinstance(predecessor_identity, Mapping)
        or predecessor_identity != _file_identity(FROZEN_V6_PREDECESSOR_IDENTITY_PATH)
        or predecessor_identity.get("sha256")
            != FROZEN_V6_FILE_SHA256["predecessor_identity"]
    ):
        raise ValueError("registered v5 predecessor identity file mutated")
    _validate_v6_predecessor_identity(predecessor)
    verify_v6_predecessor_state(predecessor)

    controller = protocol.get("controller_margin_implementation")
    expected_controller_fields = {
        *FROZEN_V6_CONTROLLER_MARGIN_IMPLEMENTATION,
        "amendment_review", "implementation_review",
    }
    if not isinstance(controller, Mapping) or set(controller) != expected_controller_fields:
        raise ValueError("registered controller-margin identity schema is invalid")
    for key, expected in FROZEN_V6_CONTROLLER_MARGIN_IMPLEMENTATION.items():
        if controller.get(key) != expected:
            raise ValueError("registered controller-margin implementation identity mutated")
    expected_reviews = {
        "amendment_review": (
            FROZEN_V6_CONTROLLER_MARGIN_AMENDMENT_REVIEW_PATH,
            FROZEN_V6_FILE_SHA256["controller_margin_amendment_review"],
        ),
        "implementation_review": (
            FROZEN_V6_CONTROLLER_MARGIN_IMPLEMENTATION_REVIEW_PATH,
            FROZEN_V6_FILE_SHA256["controller_margin_implementation_review"],
        ),
    }
    for label, (path, digest) in expected_reviews.items():
        observed = _file_identity(path)
        if controller.get(label) != observed or observed["sha256"] != digest:
            raise ValueError(f"registered {label} identity mutated")

    declaration = protocol.get("development_gate")
    if not isinstance(declaration, Mapping):
        raise ValueError("registered one-step gate declaration is invalid")
    for label in ("claim", "artifact", "review"):
        _validate_gate_file_identity(declaration.get(label), label)
    observed_gate = _validate_v6_gate(
        claim_path=Path(declaration["claim"]["path"]),
        gate_path=Path(declaration["artifact"]["path"]),
        gate_review_path=Path(declaration["review"]["path"]),
        protocol_bindings=protocol.get("bindings", {}),
    )
    if declaration != observed_gate:
        raise ValueError("registered one-step gate declaration mutated")

    repository = protocol.get("repository")
    if repository is None:
        return
    if not isinstance(repository, Mapping):
        raise ValueError("registered repository identity is invalid")
    project_root = Path(repository.get("root", "")).resolve()
    for commit_key, tree_key in (
        ("implementation_commit", "implementation_tree"),
        ("review_commit", "review_tree"),
    ):
        observed_tree = _git_bytes(
            project_root, "rev-parse", f"{controller[commit_key]}^{{tree}}"
        ).decode("ascii", errors="strict").strip()
        if observed_tree != controller[tree_key]:
            raise ValueError("controller-margin commit/tree identity mismatch")
    review_parents = _git_bytes(
        project_root, "rev-list", "--parents", "-n", "1",
        controller["review_commit"],
    ).decode("ascii", errors="strict").split()
    if review_parents != [
        controller["review_commit"], controller["implementation_commit"]
    ]:
        raise ValueError("controller-margin implementation/review topology mismatch")
    _verify_v6_gate_git_topology(repository, declaration, controller)


def _initial_family_semantic_sha256(family: dict) -> str:
    from scripts.diagnostics.qualified_initial_state import family_semantic_sha256

    return family_semantic_sha256(family)


def _derive_initial_state(path: Path, *, version: str = "v5") -> dict:
    """Reload and independently derive the exact versioned family declaration."""
    if version == "v6":
        from scripts.diagnostics.qualified_v6_initial_state import (
            audit_frozen_v6_initial_family as audit_frozen_initial_family,
            load_qualified_v6_initial_family as load_qualified_initial_family,
        )
    else:
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
    """Reject a self-rehashed substitution of any derived initial state."""
    binding = protocol.get("bindings", {}).get("initial_family", {})
    version = protocol.get("version")
    family_path = _require_literal_initial_family_path(
        binding.get("path", ""), version
    )
    expected = _derive_initial_state(family_path, version=version)
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
    with Path(path).open("rb") as source:
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
    parser.add_argument("--one-step-gate-claim", type=Path)
    parser.add_argument("--one-step-gate", type=Path)
    parser.add_argument("--one-step-gate-review", type=Path)
    parser.add_argument("--predecessor-identity", type=Path)
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
        if arguments.version == "v6":
            predecessor = load_v6_predecessor_identity(
                arguments.predecessor_identity
            )
            verify_v6_predecessor_state(predecessor)
        else:
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
    if arguments.kind == "development" and arguments.version == "v6":
        expected_review_paths = (
            FROZEN_V6_CONTROLLER_MARGIN_AMENDMENT_REVIEW_PATH,
            FROZEN_V6_CONTROLLER_MARGIN_IMPLEMENTATION_REVIEW_PATH,
        )
        for path, expected in zip(report_paths, expected_review_paths, strict=True):
            if Path(path).resolve() != expected.resolve():
                raise ValueError("v6 controller-margin review path is not exact")

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
        **(
            {
                "development_gate_claim": arguments.one_step_gate_claim,
                "development_gate": arguments.one_step_gate,
                "development_gate_review": arguments.one_step_gate_review,
                "predecessor_identity": arguments.predecessor_identity,
                "controller_margin_amendment_review": arguments.implementation_report,
                "controller_margin_implementation_review": arguments.implementation_review,
            }
            if arguments.kind == "development" and arguments.version == "v6"
            else {}
        ),
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
                    (
                        tool_root / "qualified_initial_state.py",
                        *(
                            (tool_root / "qualified_v6_initial_state.py",)
                            if arguments.version == "v6" else ()
                        ),
                    )
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
    if protocol.get("version") == "v6":
        gate = protocol.get("development_gate", {})
        for label in ("claim", "artifact", "review"):
            identities[f"development_gate_{label}"] = gate.get(label, {})
        predecessor = protocol.get("predecessor", {})
        identities["predecessor_identity"] = predecessor.get("identity_file", {})
        controller = protocol.get("controller_margin_implementation", {})
        identities["controller_margin_amendment_review"] = controller.get(
            "amendment_review", {}
        )
        identities["controller_margin_implementation_review"] = controller.get(
            "implementation_review", {}
        )
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
        if protocol.get("version") == "v6":
            expected_top.update({
                "development_gate", "predecessor",
                "controller_margin_implementation",
            })
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
        if protocol.get("version") == "v6":
            tooling_fields.add("qualified_v6_initial_state.py")
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
        if protocol.get("version") == "v6":
            exact(protocol["development_gate"], {
                "schema_version", "campaign_id", "status", "passed",
                "claim", "artifact", "review", "source", "bindings",
                "seed_universe", "launch_count", "retry_count",
                "audit_summary", "registered_summary",
            }, "development gate")
            exact(protocol["predecessor"], {
                "identity_file", "schema_version", "terminal",
            }, "v5 predecessor")
            exact(protocol["controller_margin_implementation"], {
                "schema_version", "implementation_commit", "implementation_tree",
                "review_commit", "review_tree", "amendment_review",
                "implementation_review",
            }, "controller-margin implementation")


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
        or protocol.get("version") not in SUPPORTED_PROTOCOL_VERSIONS.get(kind, ())
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
    range_start = (
        2026081301
        if kind == "development" and protocol.get("version") == "v6"
        else 2026081201 if kind == "development"
        else 2026083001
    )
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
    frozen_root_key = (
        "development_v6"
        if kind == "development" and protocol.get("version") == "v6"
        else kind
    )
    if protocol.get("roots") != FROZEN_EXECUTION_ROOTS[frozen_root_key]:
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
