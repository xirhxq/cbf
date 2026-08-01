"""Independent recomputation for qualified global-mode estimator rows."""

from collections.abc import Mapping
from numbers import Integral, Real
import math

import numpy as np

from scripts.diagnostics.estimator_lifecycle import (
    PRIVATE_STATE_FIELDS,
    PriorBundle,
    _canonical_public_state,
    _canonical_unavailable_reason,
    advance_qualified_prior,
    canonical_private_state,
    finalize_qualified_lifecycle,
)
from scripts.diagnostics.predictive_wnls import candidate_local_eligibility
from scripts.diagnostics.replay_localization_calibration import (
    _linearized_terms,
    fim_radius,
)
from scripts.diagnostics.qualified_modes import (
    MODE_TOLERANCE_M,
    ModeQualification,
    LocalCandidate,
    canonical_mode_id,
    cluster_candidates,
    enumerate_qualified_starts,
    publish_unique_mode,
    qualify_all,
    select_representative,
    sensitivity_cluster_counts,
    stable_attempt_id,
)


EXPECTED_RAW_SCHEMA_ID = (
    "cbf2026-qualified-mode-hybrid-dcbf-estimator-raw-v1"
)
EXPECTED_METHOD_ID = "qualified-mode-hybrid-dcbf-estimator-v1"
ROW_FIELDS = {
    "schema_id",
    "method_id",
    "frame_index",
    "robot_id",
    "squad_local_index",
    "schedule_id",
    "runtime_inputs",
    "local_candidate_count",
    "mode_count",
    "mode_members",
    "nonseparable_chain",
    "qualification_kind",
    "qualifications",
    "admissible_mode_count",
    "published_mode_id",
    "sensitivity_count_0_5_mm",
    "sensitivity_count_1_mm",
    "sensitivity_count_2_mm",
    "primary_tolerance_m",
    "public_status",
    "public_age",
    "private_age",
    "history_version",
    "audit_bundle",
}
AUDIT_FIELDS = {
    "runtime_inputs",
    "starts",
    "solver_attempts",
    "local_candidates",
    "clustering",
    "representatives",
    "transition_inputs",
    "prior_bundle",
    "qualifier_context",
    "qualifications",
    "sensitivity",
    "decision",
    "lifecycle",
}
RUNTIME_INPUT_FIELDS = {
    "references",
    "live_seed",
    "private_seed",
    "active_reference_count",
    "base_anchor_provenance",
}
TRANSITION_FIELDS = {
    "previous_public",
    "previous_private",
    "held_velocity",
    "frame_index",
    "applied_command_frame",
    "history_version",
    "mission_horizon_frames",
}
SOLVER_RESULT_FIELDS = {
    "status",
    "estimate",
    "covariance",
    "epsilon",
    "phi_min_eigenvalue",
    "phi_condition",
    "fim_valid",
    "proposal_count",
    "iterations",
    "cost",
    "stationarity_norm",
    "failure_reason",
    "proposal_trace",
}
LOCAL_DIAGNOSTIC_FIELDS = {
    "failure_reason",
    "geometry_failure_reason",
    "reduced_whitened_cost",
    "recomputed_estimate",
    "recomputed_covariance",
    "recomputed_objective",
    "recomputed_residual_cost",
    "recomputed_residual",
    "recomputed_fim",
    "recomputed_gauss_newton",
    "recomputed_stationarity_norm",
    "solver_geometry_consistent",
    "active_reference_count",
    "base_anchor_provenance",
}


def validate_and_recompute_qualified_row(row) -> dict:
    """Recompute every runtime-derived qualified-estimator row field."""
    _validate_exact_protocol(row)
    audit = row["audit_bundle"]
    runtime = audit["runtime_inputs"]
    transition = audit["transition_inputs"]

    if not _strict_equal(row["runtime_inputs"], runtime):
        raise ValueError("runtime input projection mismatch")
    reference_keys = [tuple(reference["key"]) for reference in runtime["references"]]
    if reference_keys != sorted(reference_keys):
        raise ValueError("reference order inconsistent with stable order")
    if transition["frame_index"] != row["frame_index"]:
        raise ValueError("transition frame input mismatch")

    expected_starts = enumerate_qualified_starts(
        runtime["references"],
        live_seed=runtime["live_seed"],
        private_seed=runtime["private_seed"],
    )
    expected_start_records = [
        _serialize_start(start) for start in expected_starts
    ]
    if not _strict_equal(audit["starts"], expected_start_records):
        raise ValueError("stable start order mismatch")

    attempts = audit["solver_attempts"]
    if len(attempts) != len(expected_starts):
        raise ValueError("solver attempt completeness mismatch")
    local_candidates = []
    for index, (attempt, start) in enumerate(
        zip(attempts, expected_starts, strict=True)
    ):
        expected_start = expected_start_records[index]
        if (
            attempt["attempt_id"] != expected_start["attempt_id"]
            or not _strict_equal(attempt["start_record"], expected_start)
        ):
            raise ValueError("solver attempt stable order mismatch")
        result = attempt["solver_result"]
        locally_valid, reason, _ = candidate_local_eligibility(
            result,
            active_reference_count=runtime["active_reference_count"],
            base_anchor_provenance=runtime["base_anchor_provenance"],
        )
        geometry = _independent_solver_geometry(
            result,
            runtime["references"],
        )
        eligible = locally_valid and geometry["solver_geometry_consistent"]
        if locally_valid and not geometry["solver_geometry_consistent"]:
            reason = "solver_geometry_mismatch"
        if (
            attempt["local_eligible"] is not eligible
            or attempt["local_eligibility_reason"] != reason
        ):
            raise ValueError("local eligibility outcome mismatch")
        expected_diagnostics = _local_diagnostics(
            eligible=eligible,
            eligibility_reason=reason,
            geometry=geometry,
            active_reference_count=runtime["active_reference_count"],
            base_anchor_provenance=runtime["base_anchor_provenance"],
        )
        if not _strict_equal(
            attempt["local_diagnostics"],
            expected_diagnostics,
        ):
            raise ValueError("local eligibility diagnostics mismatch")
        if eligible:
            result_copy = dict(geometry["verified_result"])
            result_copy["base_anchor_provenance"] = list(
                runtime["base_anchor_provenance"]
            )
            local_candidates.append(LocalCandidate(
                attempt["attempt_id"],
                tuple(result_copy["estimate"]),
                float(result_copy["cost"]),
                result_copy,
            ))

    expected_local = [_serialize_candidate(item) for item in local_candidates]
    if not _strict_equal(audit["local_candidates"], expected_local):
        raise ValueError("local candidate completeness mismatch")
    if row["local_candidate_count"] != len(local_candidates):
        raise ValueError("local candidate count mismatch")

    clustering = cluster_candidates(local_candidates, MODE_TOLERANCE_M)
    serialized_clustering = audit["clustering"]
    if serialized_clustering["tolerance_m"] != MODE_TOLERANCE_M:
        raise ValueError("audit primary tolerance mismatch")
    if row["primary_tolerance_m"] != MODE_TOLERANCE_M:
        raise ValueError("primary tolerance mismatch")
    if (
        serialized_clustering["separable"] is not clustering.separable
        or serialized_clustering["reason"] != clustering.reason
        or serialized_clustering["mode_count"] != len(clustering.modes)
    ):
        raise ValueError("primary clustering status mismatch")
    if len(serialized_clustering["modes"]) != len(clustering.modes):
        raise ValueError("cluster mode completeness mismatch")
    for raw_mode, mode in zip(
        serialized_clustering["modes"],
        clustering.modes,
        strict=True,
    ):
        if raw_mode["member_ids"] != list(mode.member_ids):
            raise ValueError("mode membership mismatch")
        if raw_mode["diameter_m"] != mode.diameter_m:
            raise ValueError("cluster diameter mismatch")
        if raw_mode["mode_id"] != canonical_mode_id(mode):
            raise ValueError("mode ID mismatch")
    expected_members = [list(mode.member_ids) for mode in clustering.modes]
    if row["mode_members"] != expected_members:
        raise ValueError("row mode membership mismatch")
    if row["mode_count"] != len(clustering.modes):
        raise ValueError("row mode count mismatch")
    expected_nonseparable = clustering.reason == "nonseparable_chain"
    if row["nonseparable_chain"] is not expected_nonseparable:
        raise ValueError("nonseparable chain mismatch")

    representatives = tuple(
        select_representative(mode) for mode in clustering.modes
    )
    if not _strict_equal(
        audit["representatives"],
        [_serialize_candidate(item) for item in representatives],
    ):
        raise ValueError("mode representative mismatch")

    prior_bundle = advance_qualified_prior(
        transition["previous_public"],
        transition["previous_private"],
        transition["held_velocity"],
        next_frame_index=transition["frame_index"],
        applied_command_frame=transition["applied_command_frame"],
        history_version=transition["history_version"],
        mission_horizon_frames=transition["mission_horizon_frames"],
    )
    expected_prior = {
        "history_version": prior_bundle.history_version,
        "private_prior": prior_bundle.private_prior,
        "public_prediction": prior_bundle.public_prediction,
    }
    if not _strict_equal(audit["prior_bundle"], expected_prior):
        raise ValueError("pre-decision prior bundle mismatch")

    context = audit["qualifier_context"]
    if context["qualifier_kind"] != row["qualification_kind"]:
        raise ValueError("qualification kind mismatch")
    expected_context_prior = (
        prior_bundle.private_prior
        if row["qualification_kind"] == "history"
        else None
    )
    if not _strict_equal(
        context["propagated_private_prior"],
        expected_context_prior,
    ):
        raise ValueError("qualifier propagated private prior mismatch")
    qualifications = _recompute_qualifications(
        clustering.modes,
        representatives,
        qualifier_kind=row["qualification_kind"],
        qualifier_payload=context["qualifier_payload"],
        private_prior=prior_bundle.private_prior,
    )
    expected_qualifications = [
        _serialize_qualification(item) for item in qualifications
    ]
    if not _strict_equal(audit["qualifications"], expected_qualifications):
        raise ValueError("audit qualification mismatch")
    if not _strict_equal(row["qualifications"], expected_qualifications):
        raise ValueError("row qualification mismatch")
    admissible_count = sum(item.admissible for item in qualifications)
    if row["admissible_mode_count"] != admissible_count:
        raise ValueError("admissible mode count mismatch")

    sensitivity = sensitivity_cluster_counts(local_candidates)
    if not _strict_equal(audit["sensitivity"], sensitivity):
        raise ValueError("audit sensitivity mismatch")
    sensitivity_fields = (
        ("sensitivity_count_0_5_mm", 0.0005.hex(), "0.5 mm"),
        ("sensitivity_count_1_mm", 0.001.hex(), "1 mm"),
        ("sensitivity_count_2_mm", 0.002.hex(), "2 mm"),
    )
    for field, key, label in sensitivity_fields:
        if row[field] != sensitivity[key]:
            raise ValueError(f"sensitivity {label} count mismatch")

    decision = publish_unique_mode(clustering, qualifications)
    expected_decision = {
        "mode_id": decision.mode_id,
        "reason": decision.reason,
        "representative_attempt_id": (
            None
            if decision.representative is None
            else decision.representative.attempt_id
        ),
        "status": decision.status,
    }
    if not _strict_equal(audit["decision"], expected_decision):
        raise ValueError("publication decision mismatch")
    if row["published_mode_id"] != decision.mode_id:
        raise ValueError("published mode ID mismatch")

    lifecycle = finalize_qualified_lifecycle(
        decision,
        prior_bundle,
        frame_index=transition["frame_index"],
        mission_horizon_frames=transition["mission_horizon_frames"],
    )
    if not _strict_equal(audit["lifecycle"], lifecycle):
        raise ValueError("qualified lifecycle mismatch")
    public = lifecycle["public_output"]
    private = lifecycle["next_private_state"]
    if row["public_status"] != public["output_status"]:
        raise ValueError("public status mismatch")
    if row["public_age"] != public["prediction_age"]:
        raise ValueError("public age mismatch")
    expected_private_age = None if private is None else private["age_frames"]
    if row["private_age"] != expected_private_age:
        raise ValueError("private age mismatch")
    if row["history_version"] != lifecycle["history_version"]:
        raise ValueError("lifecycle history version mismatch")

    return {
        "valid": True,
        "mode_count": len(clustering.modes),
        "admissible_mode_count": admissible_count,
        "published_mode_id": decision.mode_id,
        "public_status": public["output_status"],
    }


def _validate_exact_protocol(row):
    if not isinstance(row, Mapping) or set(row) != ROW_FIELDS:
        raise ValueError("qualified row differs from exact schema")
    if row["schema_id"] != EXPECTED_RAW_SCHEMA_ID:
        raise ValueError("qualified row schema identity mismatch")
    if row["method_id"] != EXPECTED_METHOD_ID:
        raise ValueError("qualified row method identity mismatch")
    for field in ("frame_index", "robot_id", "squad_local_index"):
        if not _nonnegative_int(row[field]):
            raise ValueError("qualified schedule index mismatch")
    expected_schedule = (
        f"frame-{row['frame_index']}:robot-{row['robot_id']}:"
        f"squad-local-{row['squad_local_index']}"
    )
    if row["schedule_id"] != expected_schedule:
        raise ValueError("qualified schedule identity mismatch")
    for field in (
        "local_candidate_count",
        "mode_count",
        "admissible_mode_count",
        "sensitivity_count_0_5_mm",
        "sensitivity_count_1_mm",
        "sensitivity_count_2_mm",
        "history_version",
    ):
        if not _nonnegative_int(row[field]):
            raise ValueError(f"{field} is not a non-boolean integer")
    for field in ("public_age", "private_age"):
        if row[field] is not None and not _nonnegative_int(row[field]):
            raise ValueError(f"{field} is not a non-boolean integer")
    if not isinstance(row["nonseparable_chain"], bool):
        raise ValueError("nonseparable_chain is not boolean")
    if _finite_nonnegative(row["primary_tolerance_m"]) is None:
        raise ValueError("primary_tolerance_m is not a finite float")
    if not isinstance(row["mode_members"], list) or any(
        not isinstance(members, list)
        or any(not isinstance(member, str) or not member for member in members)
        for members in row["mode_members"]
    ):
        raise ValueError("mode_members are invalid")
    _reject_forbidden(row)
    audit = row["audit_bundle"]
    if not isinstance(audit, Mapping) or set(audit) != AUDIT_FIELDS:
        raise ValueError("qualified audit differs from exact schema")
    runtime = audit["runtime_inputs"]
    if not isinstance(runtime, Mapping) or set(runtime) != RUNTIME_INPUT_FIELDS:
        raise ValueError("qualified runtime inputs differ from exact schema")
    transition = audit["transition_inputs"]
    if not isinstance(transition, Mapping) or set(transition) != TRANSITION_FIELDS:
        raise ValueError("transition inputs differ from exact schema")
    _validate_runtime_contract(row, runtime, transition)
    if not isinstance(audit["starts"], list):
        raise ValueError("start records differ from exact schema")
    for start in audit["starts"]:
        if not isinstance(start, Mapping) or set(start) != {
            "attempt_id", "kind", "estimate", "reference_keys", "branch",
        }:
            raise ValueError("start record differs from exact schema")
    if not isinstance(audit["solver_attempts"], list):
        raise ValueError("solver attempts differ from exact schema")
    for attempt in audit["solver_attempts"]:
        if not isinstance(attempt, Mapping) or set(attempt) != {
            "attempt_id",
            "start_record",
            "solver_result",
            "local_eligible",
            "local_eligibility_reason",
            "local_diagnostics",
        }:
            raise ValueError("solver attempt differs from exact schema")
        if not isinstance(attempt["solver_result"], Mapping) or set(
            attempt["solver_result"]
        ) != SOLVER_RESULT_FIELDS:
            raise ValueError("solver result differs from exact schema")
        if not isinstance(attempt["local_diagnostics"], Mapping) or set(
            attempt["local_diagnostics"]
        ) != LOCAL_DIAGNOSTIC_FIELDS:
            raise ValueError("local diagnostics differ from exact schema")
    if not isinstance(audit["local_candidates"], list) or any(
        not isinstance(candidate, Mapping)
        or set(candidate) != {
            "attempt_id", "estimate", "objective_cost", "payload",
        }
        or not isinstance(candidate["attempt_id"], str)
        or not candidate["attempt_id"]
        or _finite_vec2(candidate["estimate"]) is None
        or _finite_nonnegative(candidate["objective_cost"]) is None
        or not isinstance(candidate["payload"], Mapping)
        for candidate in audit["local_candidates"]
    ):
        raise ValueError("local candidates differ from exact schema")
    clustering = audit["clustering"]
    if not isinstance(clustering, Mapping) or set(clustering) != {
        "tolerance_m", "separable", "reason", "mode_count", "modes",
    }:
        raise ValueError("clustering differs from exact schema")
    if (
        _finite_nonnegative(clustering["tolerance_m"]) is None
        or not isinstance(clustering["separable"], bool)
        or not isinstance(clustering["reason"], str)
        or not _nonnegative_int(clustering["mode_count"])
    ):
        raise ValueError("clustering scalar fields are invalid")
    if not isinstance(clustering["modes"], list) or any(
        not isinstance(mode, Mapping)
        or set(mode) != {"mode_id", "member_ids", "diameter_m"}
        or not isinstance(mode["mode_id"], str)
        or not mode["mode_id"]
        or not isinstance(mode["member_ids"], list)
        or any(
            not isinstance(member, str) or not member
            for member in mode["member_ids"]
        )
        or _finite_nonnegative(mode["diameter_m"]) is None
        for mode in clustering["modes"]
    ):
        raise ValueError("cluster diameter or mode record is invalid")
    if not isinstance(audit["representatives"], list) or any(
        not isinstance(candidate, Mapping)
        or set(candidate) != {
            "attempt_id", "estimate", "objective_cost", "payload",
        }
        or not isinstance(candidate["attempt_id"], str)
        or not candidate["attempt_id"]
        or _finite_vec2(candidate["estimate"]) is None
        or _finite_nonnegative(candidate["objective_cost"]) is None
        or not isinstance(candidate["payload"], Mapping)
        for candidate in audit["representatives"]
    ):
        raise ValueError("representatives differ from exact schema")
    if not isinstance(audit["prior_bundle"], Mapping) or set(
        audit["prior_bundle"]
    ) != {"public_prediction", "private_prior", "history_version"}:
        raise ValueError("prior bundle differs from exact schema")
    if not isinstance(audit["qualifier_context"], Mapping) or set(
        audit["qualifier_context"]
    ) != {
        "qualifier_kind", "qualifier_payload", "propagated_private_prior",
    }:
        raise ValueError("qualifier context differs from exact schema")
    for field in ("qualifications",):
        records = audit[field]
        if not isinstance(records, list) or any(
            not isinstance(item, Mapping)
            or set(item) != {"mode_id", "admissible", "reason", "score"}
            or not isinstance(item["mode_id"], str)
            or not item["mode_id"]
            or not isinstance(item["admissible"], bool)
            or not isinstance(item["reason"], str)
            or (
                item["score"] is not None
                and _finite_number(item["score"]) is None
            )
            for item in records
        ):
            raise ValueError("qualification records differ from exact schema")
    if not isinstance(audit["sensitivity"], Mapping) or set(
        audit["sensitivity"]
    ) != {0.0005.hex(), 0.001.hex(), 0.002.hex()}:
        raise ValueError("sensitivity record differs from exact schema")
    if any(
        not _nonnegative_int(count)
        for count in audit["sensitivity"].values()
    ):
        raise ValueError("sensitivity count is not a non-boolean integer")
    if not isinstance(audit["decision"], Mapping) or set(
        audit["decision"]
    ) != {"status", "reason", "mode_id", "representative_attempt_id"}:
        raise ValueError("decision record differs from exact schema")
    lifecycle = audit["lifecycle"]
    if not isinstance(lifecycle, Mapping) or set(lifecycle) != {
        "public_output", "next_private_state", "history_version",
    }:
        raise ValueError("lifecycle record differs from exact schema")
    private = lifecycle["next_private_state"]
    if private is not None and (
        not isinstance(private, Mapping)
        or set(private) != set(PRIVATE_STATE_FIELDS)
    ):
        raise ValueError("next private state differs from exact schema")


def _validate_runtime_contract(row, runtime, transition):
    references = runtime["references"]
    if not isinstance(references, list) or len(references) < 2:
        raise ValueError("runtime references are invalid")
    keys = []
    for reference in references:
        if not isinstance(reference, Mapping) or set(reference) != {
            "key", "position", "range", "covariance", "ranging_sigma",
            "base_anchor_provenance",
        }:
            raise ValueError("runtime reference schema mismatch")
        key = reference["key"]
        reference_provenance = reference["base_anchor_provenance"]
        if (
            not isinstance(key, list)
            or len(key) != 2
            or not isinstance(key[0], str)
            or not key[0]
            or not _nonnegative_int(key[1])
            or _finite_vec2(reference["position"]) is None
            or _finite_nonnegative(reference["range"]) is None
            or not _finite_psd_2x2(reference["covariance"])
            or _finite_positive(reference["ranging_sigma"]) is None
            or not _canonical_provenance(reference_provenance)
            or (
                key[0] == "base"
                and reference_provenance != [key[1]]
            )
            or (key[0] != "base" and len(reference_provenance) < 2)
        ):
            raise ValueError("runtime reference value mismatch")
        keys.append(tuple(key))
    if len(set(keys)) != len(keys):
        raise ValueError("runtime reference keys are not unique")
    sigmas = {float(reference["ranging_sigma"]).hex() for reference in references}
    if len(sigmas) != 1:
        raise ValueError("runtime references do not share one ranging sigma")
    for field in ("live_seed", "private_seed"):
        seed = runtime[field]
        if seed is None:
            continue
        if not isinstance(seed, Mapping) or set(seed) != {
            "estimate", "modeled_covariance",
        }:
            raise ValueError(f"{field} schema mismatch")
        if (
            _finite_vec2(seed["estimate"]) is None
            or not _finite_spd_2x2(seed["modeled_covariance"])
        ):
            raise ValueError(f"{field} value mismatch")
    if (
        not _nonnegative_int(runtime["active_reference_count"])
        or runtime["active_reference_count"] != len(references)
    ):
        raise ValueError("active reference count mismatch")
    provenance = runtime["base_anchor_provenance"]
    derived_provenance = sorted({
        anchor
        for reference in references
        for anchor in reference["base_anchor_provenance"]
    })
    if (
        not _canonical_provenance(provenance)
        or len(provenance) < 2
        or provenance != derived_provenance
    ):
        raise ValueError("base anchor provenance mismatch")

    for field in (
        "frame_index", "history_version", "mission_horizon_frames",
    ):
        if not _nonnegative_int(transition[field]):
            raise ValueError(f"transition {field} mismatch")
    if (
        transition["mission_horizon_frames"] == 0
        or transition["frame_index"] >= transition["mission_horizon_frames"]
        or _finite_vec2(transition["held_velocity"]) is None
    ):
        raise ValueError("transition chronology mismatch")
    frame = transition["frame_index"]
    command_frame = transition["applied_command_frame"]
    if frame == 0:
        if (
            command_frame is not None
            or transition["previous_public"] is not None
            or transition["previous_private"] is not None
            or row["qualification_kind"] != "deployment"
        ):
            raise ValueError("frame-zero transition chronology mismatch")
    elif not _nonnegative_int(command_frame) or command_frame != frame - 1:
        raise ValueError("applied command chronology mismatch")
    previous_public = transition["previous_public"]
    if previous_public is not None:
        status = (
            previous_public.get("output_status")
            if isinstance(previous_public, Mapping)
            else None
        )
        valid = (
            _canonical_public_state(previous_public) is not None
            if status in {"fresh", "predicted"}
            else _canonical_unavailable_reason(previous_public) is not None
            if status == "unavailable"
            else False
        )
        if not valid:
            raise ValueError("previous public state schema mismatch")
    previous_private = transition["previous_private"]
    if (
        previous_private is not None
        and canonical_private_state(previous_private) is None
    ):
        raise ValueError("previous private state schema mismatch")


def _nonnegative_int(value):
    return (
        not isinstance(value, bool)
        and isinstance(value, Integral)
        and value >= 0
    )


def _canonical_provenance(value):
    return (
        isinstance(value, list)
        and all(_nonnegative_int(item) for item in value)
        and value == sorted(set(value))
    )


def _finite_number(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def _finite_nonnegative(value):
    number = _finite_number(value)
    return number if number is not None and number >= 0.0 else None


def _finite_positive(value):
    number = _finite_number(value)
    return number if number is not None and number > 0.0 else None


def _finite_vec2(value):
    if not isinstance(value, list) or len(value) != 2:
        return None
    numbers = [_finite_number(item) for item in value]
    return numbers if all(item is not None for item in numbers) else None


def _finite_spd_2x2(value):
    try:
        covariance = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return False
    if (
        covariance.shape != (2, 2)
        or not np.isfinite(covariance).all()
        or not np.allclose(covariance, covariance.T, rtol=1e-12, atol=1e-12)
    ):
        return False
    try:
        eigenvalues = np.linalg.eigvalsh(0.5 * (covariance + covariance.T))
    except np.linalg.LinAlgError:
        return False
    return bool(
        np.isfinite(eigenvalues).all()
        and eigenvalues[-1] > 0.0
        and eigenvalues[0] > 1e-12 * eigenvalues[-1]
    )


def _finite_psd_2x2(value):
    try:
        covariance = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return False
    if (
        covariance.shape != (2, 2)
        or not np.isfinite(covariance).all()
        or not np.allclose(covariance, covariance.T, rtol=1e-12, atol=1e-12)
    ):
        return False
    try:
        eigenvalues = np.linalg.eigvalsh(0.5 * (covariance + covariance.T))
    except np.linalg.LinAlgError:
        return False
    return bool(
        np.isfinite(eigenvalues).all()
        and eigenvalues[0] >= -1e-12 * max(1.0, eigenvalues[-1])
    )


def _serialize_start(start):
    return {
        "attempt_id": stable_attempt_id(start),
        "kind": start.kind,
        "estimate": list(start.estimate),
        "reference_keys": [list(key) for key in start.reference_keys],
        "branch": start.branch,
    }


def _independent_solver_geometry(result, references):
    geometry = {
        "geometry_failure_reason": None,
        "recomputed_covariance": None,
        "recomputed_estimate": None,
        "recomputed_fim": None,
        "recomputed_gauss_newton": None,
        "recomputed_objective": None,
        "recomputed_residual": None,
        "recomputed_stationarity_norm": None,
        "solver_geometry_consistent": False,
        "verified_result": None,
    }
    if result.get("status") != "converged":
        geometry["geometry_failure_reason"] = "solver_result_not_converged"
        return geometry
    estimate_values = _finite_vec2(result.get("estimate"))
    if estimate_values is None:
        geometry["geometry_failure_reason"] = "invalid_solver_estimate"
        return geometry
    estimate = np.asarray(estimate_values, dtype=float)
    positions = np.asarray(
        [reference["position"] for reference in references],
        dtype=float,
    )
    covariances = np.asarray(
        [reference["covariance"] for reference in references],
        dtype=float,
    )
    ranges = np.asarray(
        [reference["range"] for reference in references],
        dtype=float,
    )
    sigma = float(references[0]["ranging_sigma"])
    terms = _linearized_terms(
        estimate,
        positions,
        covariances,
        ranges,
        sigma,
    )
    fim = fim_radius(estimate, positions, covariances, sigma)
    if terms is None or fim.get("status") != "converged":
        geometry["geometry_failure_reason"] = "geometry_recomputation_failed"
        return geometry
    _, _, residual, gauss_newton, gradient, cost = terms
    covariance = np.asarray(fim["covariance"], dtype=float)
    try:
        phi = np.linalg.inv(covariance)
    except np.linalg.LinAlgError:
        geometry["geometry_failure_reason"] = "geometry_fim_inversion_failed"
        return geometry
    stationarity = float(np.linalg.norm(gradient, ord=np.inf))
    geometry.update({
        "recomputed_covariance": covariance.tolist(),
        "recomputed_estimate": estimate.tolist(),
        "recomputed_fim": phi.tolist(),
        "recomputed_gauss_newton": gauss_newton.tolist(),
        "recomputed_objective": float(cost),
        "recomputed_residual": residual.tolist(),
        "recomputed_stationarity_norm": stationarity,
    })
    consistent = (
        _same_geometry_scalar(result.get("cost"), cost)
        and _same_geometry_matrix(result.get("covariance"), covariance)
        and _same_geometry_scalar(result.get("epsilon"), fim["epsilon"])
        and _same_geometry_scalar(
            result.get("phi_min_eigenvalue"),
            fim["phi_min_eigenvalue"],
        )
        and _same_geometry_scalar(
            result.get("phi_condition"),
            fim["phi_condition"],
        )
        and _same_geometry_scalar(
            result.get("stationarity_norm"),
            stationarity,
        )
    )
    geometry["solver_geometry_consistent"] = consistent
    if not consistent:
        geometry["geometry_failure_reason"] = "solver_geometry_mismatch"
        return geometry
    verified = dict(result)
    verified.update({
        "estimate": estimate.tolist(),
        "covariance": covariance.tolist(),
        "epsilon": float(fim["epsilon"]),
        "phi_min_eigenvalue": float(fim["phi_min_eigenvalue"]),
        "phi_condition": float(fim["phi_condition"]),
        "cost": float(cost),
        "stationarity_norm": stationarity,
    })
    geometry["verified_result"] = verified
    return geometry


def _same_geometry_scalar(first, second):
    first_number = _finite_number(first)
    second_number = _finite_number(second)
    return (
        first_number is not None
        and second_number is not None
        and math.isclose(
            first_number,
            second_number,
            rel_tol=1e-9,
            abs_tol=1e-12,
        )
    )


def _same_geometry_matrix(first, second):
    try:
        first_array = np.asarray(first, dtype=float)
        second_array = np.asarray(second, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return False
    return bool(
        first_array.shape == second_array.shape
        and np.isfinite(first_array).all()
        and np.isfinite(second_array).all()
        and np.allclose(
            first_array,
            second_array,
            rtol=1e-9,
            atol=1e-12,
        )
    )


def _local_diagnostics(
    *,
    eligible,
    eligibility_reason,
    geometry,
    active_reference_count,
    base_anchor_provenance,
):
    objective = geometry["recomputed_objective"]
    return {
        "active_reference_count": int(active_reference_count),
        "base_anchor_provenance": list(base_anchor_provenance),
        "failure_reason": None if eligible else eligibility_reason,
        "geometry_failure_reason": geometry["geometry_failure_reason"],
        "recomputed_covariance": geometry["recomputed_covariance"],
        "recomputed_estimate": geometry["recomputed_estimate"],
        "recomputed_fim": geometry["recomputed_fim"],
        "recomputed_gauss_newton": geometry["recomputed_gauss_newton"],
        "recomputed_objective": objective,
        "recomputed_residual": geometry["recomputed_residual"],
        "recomputed_residual_cost": objective,
        "recomputed_stationarity_norm": geometry[
            "recomputed_stationarity_norm"
        ],
        "reduced_whitened_cost": (
            None
            if objective is None
            else objective / max(1, int(active_reference_count) - 2)
        ),
        "solver_geometry_consistent": geometry[
            "solver_geometry_consistent"
        ],
    }


def _serialize_candidate(candidate):
    return {
        "attempt_id": candidate.attempt_id,
        "estimate": list(candidate.estimate),
        "objective_cost": candidate.objective_cost,
        "payload": _canonical_json(candidate.payload),
    }


def _recompute_qualifications(
    modes,
    representatives,
    *,
    qualifier_kind,
    qualifier_payload,
    private_prior,
):
    if qualifier_kind == "unavailable":
        if set(qualifier_payload) != {"reason"}:
            raise ValueError("unavailable qualifier payload mismatch")
        return tuple(
            ModeQualification(
                canonical_mode_id(mode),
                False,
                qualifier_payload["reason"],
                None,
            )
            for mode in modes
        )
    if qualifier_kind == "history":
        if set(qualifier_payload) != {"innovation_limit"}:
            raise ValueError("history qualifier payload mismatch")
        if private_prior is None:
            return tuple(
                ModeQualification(
                    canonical_mode_id(mode),
                    False,
                    "propagated_private_prior_unavailable",
                    None,
                )
                for mode in modes
            )
        payload = {
            "propagated_private_prior": private_prior,
            "innovation_limit": qualifier_payload["innovation_limit"],
        }
    elif qualifier_kind == "deployment":
        if set(qualifier_payload) != {"domain"}:
            raise ValueError("deployment qualifier payload mismatch")
        payload = qualifier_payload
    else:
        raise ValueError("qualification kind mismatch")
    return qualify_all(modes, representatives, qualifier_kind, payload)


def _serialize_qualification(item):
    return {
        "mode_id": item.mode_id,
        "admissible": item.admissible,
        "reason": item.reason,
        "score": item.score,
    }


def _canonical_json(value):
    if isinstance(value, Mapping):
        return {
            str(key): _canonical_json(value[key])
            for key in sorted(value)
        }
    if isinstance(value, (tuple, list)):
        return [_canonical_json(item) for item in value]
    if isinstance(value, np.ndarray):
        return _canonical_json(value.tolist())
    if isinstance(value, np.generic):
        return _canonical_json(value.item())
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, Integral):
        return int(value)
    if isinstance(value, Real) and math.isfinite(float(value)):
        return float(value)
    raise ValueError("noncanonical analyzer value")


def _strict_equal(first, second):
    if isinstance(first, bool) or isinstance(second, bool):
        return isinstance(first, bool) and isinstance(second, bool) and first is second
    if isinstance(first, Mapping) or isinstance(second, Mapping):
        return (
            isinstance(first, Mapping)
            and isinstance(second, Mapping)
            and set(first) == set(second)
            and all(
                _strict_equal(first[key], second[key])
                for key in first
            )
        )
    if isinstance(first, list) or isinstance(second, list):
        return (
            isinstance(first, list)
            and isinstance(second, list)
            and len(first) == len(second)
            and all(
                _strict_equal(left, right)
                for left, right in zip(first, second, strict=True)
            )
        )
    return first == second


def _reject_forbidden(value, active_ids=None):
    if active_ids is None:
        active_ids = set()
    if isinstance(value, Mapping):
        identifier = id(value)
        if identifier in active_ids:
            raise ValueError("cyclic qualified analyzer payload")
        active_ids.add(identifier)
        try:
            for key, nested in value.items():
                if not isinstance(key, str):
                    raise ValueError("qualified analyzer key is invalid")
                if any(fragment in key.lower() for fragment in (
                    "truth", "analyzer", "future", "realized",
                )):
                    raise ValueError("forbidden qualified runtime field")
                _reject_forbidden(nested, active_ids)
        finally:
            active_ids.remove(identifier)
    elif isinstance(value, list):
        identifier = id(value)
        if identifier in active_ids:
            raise ValueError("cyclic qualified analyzer payload")
        active_ids.add(identifier)
        try:
            for item in value:
                _reject_forbidden(item, active_ids)
        finally:
            active_ids.remove(identifier)
