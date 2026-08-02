from __future__ import annotations

import math
from typing import Any, Mapping


def _keys(value: Any, expected: set[str]) -> bool:
    return isinstance(value, Mapping) and set(value) == expected


def _number(value: Any, expected: float) -> bool:
    return type(value) in (int, float) and math.isfinite(value) and value == expected


def _float(value: Any, expected: float) -> bool:
    return type(value) is float and math.isfinite(value) and value == expected


def _integer(value: Any, expected: int) -> bool:
    return type(value) is int and value == expected


def _boolean(value: Any, expected: bool) -> bool:
    return type(value) is bool and value is expected


def _number_vector(value: Any, expected: list[float]) -> bool:
    return (
        type(value) is list
        and len(value) == len(expected)
        and all(_number(actual, target) for actual, target in zip(value, expected))
    )


def validate_qualified_config(config: Mapping[str, Any]) -> bool:
    try:
        v1_root_keys = {
            "qualified-estimator",
            "position_covariance",
            "cbfs",
            "execute",
        }
        v2_root_keys = v1_root_keys | {"qualified-controller"}
        if set(config) != v1_root_keys and set(config) != v2_root_keys:
            return False
        has_controller = set(config) == v2_root_keys
        controller_schema = None
        if has_controller:
            marker = config["qualified-controller"]
            if not _keys(marker, {"schema-version"}):
                return False
            controller_schema = marker["schema-version"]
            if type(controller_schema) is not str:
                return False
            if controller_schema not in {"hard-interior-v2", "hard-interior-v3"}:
                return False

        estimator = config["qualified-estimator"]
        if not _keys(
            estimator,
            {"mode-tolerance-m", "sensitivity-tolerances-m", "deployment", "history"},
        ):
            return False
        if not _number(estimator["mode-tolerance-m"], 0.001):
            return False
        if not _number_vector(
            estimator["sensitivity-tolerances-m"], [0.0005, 0.001, 0.002]
        ):
            return False

        deployment = estimator["deployment"]
        if not _keys(
            deployment,
            {
                "anchor-ids",
                "anchor-coordinates",
                "deployment-vertices",
                "unit-normal",
                "offset",
                "ocean-side",
                "margin-m",
                "domain-version",
            },
        ):
            return False
        if deployment["anchor-ids"] != [0, 2] or any(
            type(value) is not int for value in deployment["anchor-ids"]
        ):
            return False
        if deployment["anchor-coordinates"] != [
            [-1550.0, -300.0],
            [-1550.0, 300.0],
        ]:
            return False
        if deployment["deployment-vertices"] != [
            [-1490.0, -200.0],
            [-1370.0, -200.0],
            [-1370.0, 200.0],
            [-1490.0, 200.0],
        ]:
            return False
        normal = deployment["unit-normal"]
        if not _number_vector(normal, [1.0, 0.0]) or not math.isclose(
            math.hypot(*normal), 1.0, rel_tol=0.0, abs_tol=1e-12
        ):
            return False
        if not _number(deployment["offset"], 1550.0):
            return False
        if not _integer(deployment["ocean-side"], 1):
            return False
        if not _number(deployment["margin-m"], 1.0):
            return False
        if deployment["domain-version"] != "ocean-side-v1":
            return False
        offset = deployment["offset"]
        for anchor in deployment["anchor-coordinates"]:
            if not math.isclose(
                normal[0] * anchor[0] + normal[1] * anchor[1] + offset,
                0.0,
                rel_tol=0.0,
                abs_tol=1e-12,
            ):
                return False
        for vertex in deployment["deployment-vertices"]:
            signed = deployment["ocean-side"] * (
                normal[0] * vertex[0] + normal[1] * vertex[1] + offset
            )
            if signed < deployment["margin-m"]:
                return False

        history = estimator["history"]
        if not _keys(
            history,
            {
                "q-threshold",
                "process-noise-diagonal",
                "public-max-age-frames",
                "private-max-age-policy",
            },
        ):
            return False
        if not _number(history["q-threshold"], 11.829007011943707):
            return False
        if not _number_vector(history["process-noise-diagonal"], [0.25, 0.25]):
            return False
        if not _integer(history["public-max-age-frames"], 2):
            return False
        if history["private-max-age-policy"] != "mission-frames-minus-one":
            return False

        covariance = config["position_covariance"]
        if not _keys(
            covariance,
            {
                "reference-selection",
                "ranging_sigma",
                "singular-distance-tolerance-m",
                "relative-spectral-threshold",
            },
        ):
            return False
        if covariance["reference-selection"] not in {
            "dynamic-lower-index",
            "fixed-cbf-only",
        }:
            return False
        if not _number(covariance["ranging_sigma"], 0.5):
            return False
        if not _number(covariance["singular-distance-tolerance-m"], 1e-8):
            return False
        if not _number(covariance["relative-spectral-threshold"], 1e-12):
            return False

        cbfs = config["cbfs"]
        v1_cbf_keys = {"uncertainty-rate", "input-limits", "without-slack"}
        v2_cbf_keys = v1_cbf_keys | {"hard-interior-selection"}
        expected_cbf_keys = v2_cbf_keys if has_controller else v1_cbf_keys
        if set(cbfs) != expected_cbf_keys:
            return False
        if cbfs["uncertainty-rate"] != {"mode": "analytic-topological"}:
            return False
        input_limits = cbfs["input-limits"]
        if not _keys(
            input_limits,
            {"on", "planar-component-max", "yaw-rate-max"},
        ) or not _boolean(input_limits["on"], True):
            return False
        if not _number(input_limits["planar-component-max"], 25.0) or not _number(
            input_limits["yaw-rate-max"], 0.35
        ):
            return False
        if not _keys(cbfs["without-slack"], {"safety", "comm-fixed"}):
            return False
        safety = cbfs["without-slack"]["safety"]
        hard_class_keys = {"on", "mode", "alpha"} if has_controller else {"on", "mode"}
        if not _keys(safety, hard_class_keys) or not _boolean(
            safety["on"], True
        ) or safety["mode"] != "allocated-pairwise":
            return False
        comm_fixed = cbfs["without-slack"]["comm-fixed"]
        if not _keys(comm_fixed, hard_class_keys) or not _boolean(
            comm_fixed["on"], True
        ) or comm_fixed["mode"] != "allocated-pairwise":
            return False
        if has_controller:
            expected_policy = {
                "hard-interior-v2": ("planar-chebyshev-fraction-cap-v1", 0.1),
                "hard-interior-v3": ("planar-chebyshev-fraction-cap-v2", 0.131),
            }[controller_schema]
            if not _keys(
                cbfs["hard-interior-selection"],
                {"mode", "fraction", "cap-mps", "feasibility-tolerance-mps"},
            ) or cbfs["hard-interior-selection"]["mode"] != expected_policy[0]:
                return False
            if not _float(
                cbfs["hard-interior-selection"]["fraction"], expected_policy[1]
            ):
                return False
            if not _float(cbfs["hard-interior-selection"]["cap-mps"], 0.1):
                return False
            if not _float(
                cbfs["hard-interior-selection"]["feasibility-tolerance-mps"],
                1e-9,
            ):
                return False
            if not _keys(safety["alpha"], {"coe", "pow"}) or not _number(
                safety["alpha"]["coe"], 0.1
            ) or not _integer(safety["alpha"]["pow"], 1):
                return False
            if not _keys(comm_fixed["alpha"], {"coe", "pow"}) or not _number(
                comm_fixed["alpha"]["coe"], 0.1
            ) or not _integer(comm_fixed["alpha"]["pow"], 1):
                return False

        execute = config["execute"]
        return execute == {"execution-mode": "distributed", "time-step": 0.5}
    except (KeyError, TypeError, ValueError, OverflowError):
        return False
