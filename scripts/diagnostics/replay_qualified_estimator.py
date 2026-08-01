"""Exact raw-row producer for the qualified global-mode estimator."""

from collections.abc import Mapping
from numbers import Integral, Real
import math

import numpy as np

from scripts.diagnostics.predictive_wnls import solve_qualified_multistart


RAW_SCHEMA_ID = "cbf2026-qualified-mode-hybrid-dcbf-estimator-raw-v1"
METHOD_ID = "qualified-mode-hybrid-dcbf-estimator-v1"
ROW_FIELDS = (
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
)
AUDIT_FIELDS = (
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
)
SENSITIVITY_KEYS = {
    "sensitivity_count_0_5_mm": 0.0005.hex(),
    "sensitivity_count_1_mm": 0.001.hex(),
    "sensitivity_count_2_mm": 0.002.hex(),
}
RUNTIME_INPUT_FIELDS = (
    "active_reference_count", "base_anchor_provenance", "live_seed",
    "private_seed", "references",
)
REFERENCE_FIELDS = (
    "base_anchor_provenance", "covariance", "key", "position", "range",
    "ranging_sigma",
)
START_FIELDS = (
    "attempt_id", "kind", "estimate", "reference_keys", "branch",
)
SOLVER_ATTEMPT_FIELDS = (
    "attempt_id", "start_record", "solver_result", "local_eligible",
    "local_eligibility_reason", "local_diagnostics",
)
SOLVER_RESULT_FIELDS = (
    "cost", "covariance", "epsilon", "estimate", "failure_reason",
    "fim_valid", "iterations", "phi_condition", "phi_min_eigenvalue",
    "proposal_count", "proposal_trace", "stationarity_norm", "status",
)
PROPOSAL_TRACE_FIELDS = (
    "accepted", "cost", "damping", "invalid_trial_reason", "proposal",
    "raw_step_norm", "stationarity_norm", "trial_cost",
)
LOCAL_DIAGNOSTIC_FIELDS = (
    "active_reference_count", "base_anchor_provenance", "failure_reason",
    "geometry_failure_reason",
    "recomputed_covariance", "recomputed_estimate", "recomputed_fim",
    "recomputed_gauss_newton", "recomputed_objective",
    "recomputed_residual", "recomputed_residual_cost",
    "recomputed_stationarity_norm", "reduced_whitened_cost",
    "solver_geometry_consistent",
)
CANDIDATE_FIELDS = ("attempt_id", "estimate", "objective_cost", "payload")
CANDIDATE_PAYLOAD_FIELDS = (
    "base_anchor_provenance", "cost", "covariance", "epsilon", "estimate",
    "failure_reason", "fim_valid", "iterations", "phi_condition",
    "phi_min_eigenvalue", "proposal_count", "proposal_trace",
    "stationarity_norm", "status",
)
CLUSTERING_FIELDS = (
    "tolerance_m", "separable", "reason", "mode_count", "modes",
)
MODE_FIELDS = ("mode_id", "member_ids", "diameter_m")
TRANSITION_FIELDS = (
    "applied_command_frame", "frame_index", "held_velocity",
    "history_version", "mission_horizon_frames", "previous_private",
    "previous_public",
)
PRIOR_FIELDS = ("history_version", "private_prior", "public_prediction")
QUALIFIER_CONTEXT_FIELDS = (
    "propagated_private_prior", "qualifier_kind", "qualifier_payload",
)
QUALIFICATION_FIELDS = ("mode_id", "admissible", "reason", "score")
SENSITIVITY_FIELDS = (
    "0x1.0624dd2f1a9fcp-11",
    "0x1.0624dd2f1a9fcp-10",
    "0x1.0624dd2f1a9fcp-9",
)
DECISION_FIELDS = (
    "status", "reason", "mode_id", "representative_attempt_id",
)
LIFECYCLE_FIELDS = ("history_version", "next_private_state", "public_output")
MAX_WNLS_PROPOSALS = 50
MIN_WNLS_DAMPING = 1e-15
MAX_WNLS_DAMPING = 1e15
INITIAL_WNLS_DAMPING = 1e-3
WNLS_DAMPING_FACTOR = 10.0
RELATIVE_SPECTRAL_THRESHOLD = 1e-12
RELATIVE_TIE_TOLERANCE = 1e-12
EVIDENCE_MAX_DEPTH = 8
EVIDENCE_MAX_NODES = 4096
FORBIDDEN_FRAGMENTS = ("truth", "analyzer", "future", "realized")
SERIALIZED_PRIVATE_STATE_FIELDS = (
    "age_frames", "estimate", "history_version", "last_command_frame",
    "last_held_velocity", "modeled_covariance", "propagated_to_frame",
    "source_fresh_frame", "status",
)
SERIALIZED_FRESH_PUBLIC_FIELDS = (
    "aged_modeled_radius", "base_anchor_provenance", "epsilon", "estimate",
    "mode_id", "modeled_covariance", "output_status", "prediction_age",
    "reason",
)
SERIALIZED_PREDICTED_PUBLIC_FIELDS = (
    "aged_modeled_radius", "base_anchor_provenance", "epsilon", "estimate",
    "modeled_covariance", "output_status", "prediction_age",
)
SERIALIZED_UNAVAILABLE_PUBLIC_FIELDS = (
    "aged_modeled_radius", "base_anchor_provenance", "epsilon", "estimate",
    "modeled_covariance", "output_status", "prediction_age", "reason",
)
DEPLOYMENT_DOMAIN_FIELDS = (
    "anchor_coordinates", "anchor_ids", "deployment_vertices",
    "domain_version", "margin_m", "ocean_side", "offset", "unit_normal",
)
SEED_FIELDS = ("estimate", "modeled_covariance")


def build_qualified_replay_row(
    *,
    frame_index,
    robot_id,
    squad_local_index,
    schedule_id,
    references,
    solver_from_start,
    live_seed,
    private_seed,
    qualifier_kind,
    qualifier_payload,
    previous_public,
    previous_private,
    held_velocity,
    applied_command_frame,
    history_version,
    mission_horizon_frames,
    active_reference_count,
    base_anchor_provenance,
) -> dict:
    """Return one independently replayable scheduled estimator row."""
    _validate_schedule_identity(
        frame_index,
        robot_id,
        squad_local_index,
        schedule_id,
    )
    audit = solve_qualified_multistart(
        references=references,
        solver_from_start=solver_from_start,
        live_seed=live_seed,
        private_seed=private_seed,
        qualifier_kind=qualifier_kind,
        qualifier_payload=qualifier_payload,
        previous_public=previous_public,
        previous_private=previous_private,
        held_velocity=held_velocity,
        frame_index=frame_index,
        applied_command_frame=applied_command_frame,
        history_version=history_version,
        mission_horizon_frames=mission_horizon_frames,
        active_reference_count=active_reference_count,
        base_anchor_provenance=base_anchor_provenance,
    )
    clustering = audit["clustering"]
    lifecycle = audit["lifecycle"]
    public = lifecycle["public_output"]
    next_private = lifecycle["next_private_state"]
    qualifications = audit["qualifications"]
    sensitivity = audit["sensitivity"]
    row = {
        "schema_id": RAW_SCHEMA_ID,
        "method_id": METHOD_ID,
        "frame_index": int(frame_index),
        "robot_id": int(robot_id),
        "squad_local_index": int(squad_local_index),
        "schedule_id": schedule_id,
        "runtime_inputs": audit["runtime_inputs"],
        "local_candidate_count": len(audit["local_candidates"]),
        "mode_count": clustering["mode_count"],
        "mode_members": [
            list(mode["member_ids"]) for mode in clustering["modes"]
        ],
        "nonseparable_chain": (
            clustering["reason"] == "nonseparable_chain"
        ),
        "qualification_kind": qualifier_kind,
        "qualifications": qualifications,
        "admissible_mode_count": sum(
            qualification["admissible"] for qualification in qualifications
        ),
        "published_mode_id": audit["decision"]["mode_id"],
        "sensitivity_count_0_5_mm": sensitivity[
            SENSITIVITY_KEYS["sensitivity_count_0_5_mm"]
        ],
        "sensitivity_count_1_mm": sensitivity[
            SENSITIVITY_KEYS["sensitivity_count_1_mm"]
        ],
        "sensitivity_count_2_mm": sensitivity[
            SENSITIVITY_KEYS["sensitivity_count_2_mm"]
        ],
        "primary_tolerance_m": clustering["tolerance_m"],
        "public_status": public["output_status"],
        "public_age": public["prediction_age"],
        "private_age": (
            None if next_private is None else next_private["age_frames"]
        ),
        "history_version": lifecycle["history_version"],
        "audit_bundle": audit,
    }
    validate_qualified_replay_row(row)
    return row


def validate_qualified_replay_row(row: Mapping) -> None:
    """Validate the producer-side exact row protocol without recomputation."""
    if not isinstance(row, Mapping) or tuple(row) != ROW_FIELDS:
        raise ValueError("qualified row differs from exact field order")
    if row["schema_id"] != RAW_SCHEMA_ID:
        raise ValueError("qualified row schema identity is invalid")
    if row["method_id"] != METHOD_ID:
        raise ValueError("qualified row method identity is invalid")
    _validate_schedule_identity(
        row["frame_index"],
        row["robot_id"],
        row["squad_local_index"],
        row["schedule_id"],
    )
    audit = row["audit_bundle"]
    if not isinstance(audit, Mapping) or tuple(audit) != AUDIT_FIELDS:
        raise ValueError("qualified audit bundle differs from exact schema")
    _validate_exact_audit_schema(audit)
    if (
        not isinstance(row["runtime_inputs"], Mapping)
        or tuple(row["runtime_inputs"]) != RUNTIME_INPUT_FIELDS
        or row["runtime_inputs"] != audit["runtime_inputs"]
    ):
        raise ValueError("qualified runtime input projection is inconsistent")
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
            raise ValueError(f"{field} is invalid")
    if not isinstance(row["mode_members"], list) or any(
        not isinstance(members, list)
        or any(not isinstance(member, str) or not member for member in members)
        for members in row["mode_members"]
    ):
        raise ValueError("mode members are invalid")
    if not isinstance(row["nonseparable_chain"], bool):
        raise ValueError("nonseparable chain flag is invalid")
    if row["qualification_kind"] not in {
        "deployment",
        "history",
        "unavailable",
    }:
        raise ValueError("qualification kind is invalid")
    if not isinstance(row["qualifications"], list):
        raise ValueError("qualifications are invalid")
    for qualification in row["qualifications"]:
        if (
            not isinstance(qualification, Mapping)
            or tuple(qualification) != QUALIFICATION_FIELDS
            or not isinstance(qualification["mode_id"], str)
            or not qualification["mode_id"]
            or not isinstance(qualification["admissible"], bool)
            or not isinstance(qualification["reason"], str)
            or not qualification["reason"]
            or (
                qualification["score"] is not None
                and _finite_number(qualification["score"]) is None
            )
        ):
            raise ValueError("qualification record is invalid")
    if row["published_mode_id"] is not None and (
        not isinstance(row["published_mode_id"], str)
        or not row["published_mode_id"]
    ):
        raise ValueError("published mode ID is invalid")
    if row["primary_tolerance_m"] != 0.001:
        raise ValueError("primary tolerance is invalid")
    if row["public_status"] not in {"fresh", "predicted", "unavailable"}:
        raise ValueError("public status is invalid")
    if row["public_age"] is not None and not _nonnegative_int(row["public_age"]):
        raise ValueError("public age is invalid")
    if row["private_age"] is not None and not _nonnegative_int(row["private_age"]):
        raise ValueError("private age is invalid")
    _reject_forbidden_runtime_fields(row)


def _validate_exact_audit_schema(audit):
    runtime = audit["runtime_inputs"]
    if not isinstance(runtime, Mapping) or tuple(runtime) != RUNTIME_INPUT_FIELDS:
        raise ValueError("runtime inputs differ from exact schema")
    references = runtime["references"]
    if not isinstance(references, list) or len(references) < 2 or any(
        not isinstance(reference, Mapping)
        or tuple(reference) != REFERENCE_FIELDS
        for reference in references
    ):
        raise ValueError("runtime references differ from exact schema")
    reference_keys = []
    for reference in references:
        key = reference["key"]
        provenance = reference["base_anchor_provenance"]
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
            or not _canonical_provenance(provenance)
            or (key[0] == "base" and provenance != [key[1]])
            or (key[0] != "base" and len(provenance) < 2)
        ):
            raise ValueError("runtime reference value is invalid")
        reference_keys.append(tuple(key))
    if (
        len(set(reference_keys)) != len(reference_keys)
        or reference_keys != sorted(reference_keys)
    ):
        raise ValueError("runtime reference order is invalid")
    if len({float(item["ranging_sigma"]).hex() for item in references}) != 1:
        raise ValueError("runtime ranging sigma is inconsistent")
    aggregate_provenance = runtime["base_anchor_provenance"]
    derived_provenance = sorted({
        anchor
        for reference in references
        for anchor in reference["base_anchor_provenance"]
    })
    if (
        not _canonical_provenance(aggregate_provenance)
        or len(aggregate_provenance) < 2
        or aggregate_provenance != derived_provenance
    ):
        raise ValueError("base anchor provenance is invalid")
    if (
        not _nonnegative_int(runtime["active_reference_count"])
        or runtime["active_reference_count"] != len(references)
    ):
        raise ValueError("active reference count is invalid")
    for field in ("live_seed", "private_seed"):
        seed = runtime[field]
        if seed is None:
            continue
        if (
            not isinstance(seed, Mapping)
            or tuple(seed) != SEED_FIELDS
            or _finite_vec2(seed["estimate"]) is None
            or not _finite_spd_2x2(seed["modeled_covariance"])
        ):
            raise ValueError(f"{field} differs from exact schema")
    starts = audit["starts"]
    if not isinstance(starts, list) or any(
        not isinstance(start, Mapping)
        or tuple(start) != START_FIELDS
        for start in starts
    ):
        raise ValueError("start records differ from exact schema")
    attempts = audit["solver_attempts"]
    if not isinstance(attempts, list):
        raise ValueError("solver attempts differ from exact schema")
    for attempt in attempts:
        if (
            not isinstance(attempt, Mapping)
            or tuple(attempt) != SOLVER_ATTEMPT_FIELDS
        ):
            raise ValueError("solver attempt differs from exact schema")
        if (
            not isinstance(attempt["attempt_id"], str)
            or not attempt["attempt_id"]
            or not isinstance(attempt["local_eligible"], bool)
            or not isinstance(attempt["local_eligibility_reason"], str)
        ):
            raise ValueError("solver attempt scalar is invalid")
        if (
            not isinstance(attempt["start_record"], Mapping)
            or tuple(attempt["start_record"]) != START_FIELDS
        ):
            raise ValueError("solver attempt start differs from exact schema")
        _validate_solver_result_evidence(attempt["solver_result"])
        if (
            not isinstance(attempt["local_diagnostics"], Mapping)
            or tuple(attempt["local_diagnostics"]) != LOCAL_DIAGNOSTIC_FIELDS
        ):
            raise ValueError("local diagnostics differ from exact schema")
        local = attempt["local_diagnostics"]
        if (
            not _nonnegative_int(local["active_reference_count"])
            or not _canonical_provenance(local["base_anchor_provenance"])
            or local["base_anchor_provenance"] != aggregate_provenance
            or not isinstance(local["solver_geometry_consistent"], bool)
        ):
            raise ValueError("local diagnostics values are invalid")
    candidates = audit["local_candidates"]
    representatives = audit["representatives"]
    for name, records in (
        ("local candidates", candidates),
        ("representatives", representatives),
    ):
        if not isinstance(records, list) or any(
            not isinstance(record, Mapping)
            or tuple(record) != CANDIDATE_FIELDS
            or not isinstance(record["attempt_id"], str)
            or not record["attempt_id"]
            or _finite_vec2(record["estimate"]) is None
            or _finite_nonnegative(record["objective_cost"]) is None
            or not isinstance(record["payload"], Mapping)
            or tuple(record["payload"]) != CANDIDATE_PAYLOAD_FIELDS
            for record in records
        ):
            raise ValueError(f"{name} differ from exact schema")
        for record in records:
            _validate_candidate_payload_evidence(record["payload"])
    clustering = audit["clustering"]
    if (
        not isinstance(clustering, Mapping)
        or tuple(clustering) != CLUSTERING_FIELDS
    ):
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
        or tuple(mode) != MODE_FIELDS
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
    transition = audit["transition_inputs"]
    if (
        not isinstance(transition, Mapping)
        or tuple(transition) != TRANSITION_FIELDS
    ):
        raise ValueError("transition inputs differ from exact schema")
    _validate_optional_public_order(transition["previous_public"])
    _validate_optional_private_order(transition["previous_private"])
    prior = audit["prior_bundle"]
    if not isinstance(prior, Mapping) or tuple(prior) != PRIOR_FIELDS:
        raise ValueError("prior bundle differs from exact schema")
    _validate_optional_public_order(prior["public_prediction"])
    _validate_optional_private_order(prior["private_prior"])
    context = audit["qualifier_context"]
    if (
        not isinstance(context, Mapping)
        or tuple(context) != QUALIFIER_CONTEXT_FIELDS
    ):
        raise ValueError("qualifier context differs from exact schema")
    _validate_qualifier_payload_order(
        context["qualifier_kind"],
        context["qualifier_payload"],
    )
    _validate_optional_private_order(context["propagated_private_prior"])
    audit_qualifications = audit["qualifications"]
    if not isinstance(audit_qualifications, list) or any(
        not isinstance(item, Mapping)
        or tuple(item) != QUALIFICATION_FIELDS
        or not isinstance(item["mode_id"], str)
        or not item["mode_id"]
        or not isinstance(item["admissible"], bool)
        or not isinstance(item["reason"], str)
        or (
            item["score"] is not None
            and _finite_number(item["score"]) is None
        )
        for item in audit_qualifications
    ):
        raise ValueError("audit qualifications differ from exact schema")
    if (
        not isinstance(audit["sensitivity"], Mapping)
        or tuple(audit["sensitivity"]) != SENSITIVITY_FIELDS
    ):
        raise ValueError("sensitivity record differs from exact schema")
    if any(
        not _nonnegative_int(count)
        for count in audit["sensitivity"].values()
    ):
        raise ValueError("sensitivity count is invalid")
    decision = audit["decision"]
    if not isinstance(decision, Mapping) or tuple(decision) != DECISION_FIELDS:
        raise ValueError("decision differs from exact schema")
    lifecycle = audit["lifecycle"]
    if not isinstance(lifecycle, Mapping) or tuple(lifecycle) != LIFECYCLE_FIELDS:
        raise ValueError("lifecycle differs from exact schema")
    _validate_optional_public_order(lifecycle["public_output"])
    _validate_optional_private_order(lifecycle["next_private_state"])


def _validate_optional_public_order(value):
    if value is None:
        return
    if not isinstance(value, Mapping):
        raise ValueError("public state differs from exact field order")
    expected = {
        "fresh": SERIALIZED_FRESH_PUBLIC_FIELDS,
        "predicted": SERIALIZED_PREDICTED_PUBLIC_FIELDS,
        "unavailable": SERIALIZED_UNAVAILABLE_PUBLIC_FIELDS,
    }.get(value.get("output_status"))
    if expected is None or tuple(value) != expected:
        raise ValueError("public state differs from exact field order")


def _validate_optional_private_order(value):
    if value is not None and (
        not isinstance(value, Mapping)
        or tuple(value) != SERIALIZED_PRIVATE_STATE_FIELDS
    ):
        raise ValueError("private state differs from exact field order")


def _validate_qualifier_payload_order(kind, payload):
    if not isinstance(payload, Mapping):
        raise ValueError("qualifier payload differs from exact field order")
    _reject_forbidden_runtime_fields(payload)
    expected = {
        "deployment": ("domain",),
        "history": ("innovation_limit",),
        "unavailable": ("reason",),
    }.get(kind)
    if expected is None or tuple(payload) != expected:
        raise ValueError("qualifier payload differs from exact field order")
    if kind == "deployment":
        domain = payload["domain"]
        if (
            not isinstance(domain, Mapping)
            or tuple(domain) != DEPLOYMENT_DOMAIN_FIELDS
        ):
            raise ValueError("deployment domain differs from exact field order")


def _validate_candidate_payload_evidence(payload):
    if (
        not isinstance(payload, Mapping)
        or tuple(payload) != CANDIDATE_PAYLOAD_FIELDS
        or not _canonical_provenance(payload["base_anchor_provenance"])
    ):
        raise ValueError("candidate payload differs from exact field order")
    solver_result = {
        field: payload[field]
        for field in SOLVER_RESULT_FIELDS
    }
    _validate_solver_result_evidence(solver_result)


def _bounded_solver_evidence(value):
    active_ids = set()
    nodes = 0

    def visit(item, depth):
        nonlocal nodes
        nodes += 1
        if nodes > EVIDENCE_MAX_NODES or depth > EVIDENCE_MAX_DEPTH:
            return False
        if isinstance(item, Mapping):
            identifier = id(item)
            if identifier in active_ids:
                return False
            active_ids.add(identifier)
            try:
                for key, nested in item.items():
                    if not isinstance(key, str) or any(
                        fragment in key.lower()
                        for fragment in FORBIDDEN_FRAGMENTS
                    ):
                        return False
                    if not visit(nested, depth + 1):
                        return False
                return True
            finally:
                active_ids.remove(identifier)
        if isinstance(item, list):
            identifier = id(item)
            if identifier in active_ids:
                return False
            active_ids.add(identifier)
            try:
                return all(visit(nested, depth + 1) for nested in item)
            finally:
                active_ids.remove(identifier)
        return item is None or isinstance(item, (str, bool, Real))

    return visit(value, 0)


def _same_solver_cost(first, second):
    tolerance = RELATIVE_TIE_TOLERANCE * max(
        1.0,
        abs(first),
        abs(second),
    )
    return abs(first - second) <= tolerance


def _validate_solver_proposal_trace(trace, *, converged):
    if not isinstance(trace, list) or len(trace) > MAX_WNLS_PROPOSALS:
        raise ValueError("solver result evidence proposal trace is invalid")
    previous_damping = None
    previous_cost = None
    previous_trial_cost = None
    previous_accepted = None
    for index, row in enumerate(trace):
        if not isinstance(row, Mapping) or tuple(row) != PROPOSAL_TRACE_FIELDS:
            raise ValueError(
                "solver result evidence proposal field order is invalid"
            )
        proposal = row["proposal"]
        damping = _finite_number(row["damping"])
        cost = _finite_nonnegative(row["cost"])
        stationarity = _finite_nonnegative(row["stationarity_norm"])
        raw_step = (
            None
            if row["raw_step_norm"] is None
            else _finite_nonnegative(row["raw_step_norm"])
        )
        trial = (
            None
            if row["trial_cost"] is None
            else _finite_nonnegative(row["trial_cost"])
        )
        invalid_reason = row["invalid_trial_reason"]
        accepted = row["accepted"]
        if (
            not _nonnegative_int(proposal)
            or proposal != index
            or damping is None
            or damping < MIN_WNLS_DAMPING
            or damping > MAX_WNLS_DAMPING
            or cost is None
            or stationarity is None
            or (row["raw_step_norm"] is not None and raw_step is None)
            or (row["trial_cost"] is not None and trial is None)
            or (
                invalid_reason is not None
                and (not isinstance(invalid_reason, str) or not invalid_reason)
            )
            or not isinstance(accepted, bool)
        ):
            raise ValueError("solver result evidence proposal value is invalid")
        if index == 0:
            if damping != INITIAL_WNLS_DAMPING:
                raise ValueError("solver result evidence damping is invalid")
        else:
            expected_damping = (
                max(MIN_WNLS_DAMPING, previous_damping / WNLS_DAMPING_FACTOR)
                if previous_accepted
                else previous_damping * WNLS_DAMPING_FACTOR
            )
            expected_cost = (
                previous_trial_cost if previous_accepted else previous_cost
            )
            if (
                damping != expected_damping
                or expected_cost is None
                or not _same_solver_cost(cost, expected_cost)
            ):
                raise ValueError(
                    "solver result evidence proposal sequence is invalid"
                )
        if accepted:
            if invalid_reason is not None or trial is None or not trial < cost:
                raise ValueError("solver result evidence acceptance is invalid")
        elif trial is None and invalid_reason is None:
            raise ValueError("solver result evidence rejection is incomplete")
        elif trial is not None and invalid_reason is not None:
            raise ValueError("solver result evidence rejection is ambiguous")
        elif trial is not None and trial < cost:
            raise ValueError("solver result evidence rejection is inconsistent")
        if converged and (
            raw_step is None
            or (
                invalid_reason is not None
                and invalid_reason not in {
                    "invalid_trial_terms",
                    "non-finite_trial_cost",
                }
            )
        ):
            raise ValueError("solver result evidence converged trace is invalid")
        previous_damping = damping
        previous_cost = cost
        previous_trial_cost = trial
        previous_accepted = accepted


def _solver_spd_covariance(value):
    try:
        covariance = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return None
    if (
        covariance.shape != (2, 2)
        or not np.isfinite(covariance).all()
        or not np.allclose(covariance, covariance.T, rtol=1e-12, atol=1e-12)
    ):
        return None
    canonical = 0.5 * (covariance + covariance.T)
    try:
        eigenvalues = np.linalg.eigvalsh(canonical)
    except np.linalg.LinAlgError:
        return None
    if (
        not np.isfinite(eigenvalues).all()
        or eigenvalues[0]
        <= RELATIVE_SPECTRAL_THRESHOLD * eigenvalues[-1]
    ):
        return None
    return canonical, eigenvalues


def _validate_converged_solver_evidence(result, trace):
    covariance_record = _solver_spd_covariance(result["covariance"])
    epsilon = _finite_nonnegative(result["epsilon"])
    phi_minimum = _finite_nonnegative(result["phi_min_eigenvalue"])
    phi_condition = _finite_nonnegative(result["phi_condition"])
    cost = _finite_nonnegative(result["cost"])
    stationarity = _finite_nonnegative(result["stationarity_norm"])
    if (
        _finite_vec2(result["estimate"]) is None
        or covariance_record is None
        or epsilon is None
        or epsilon <= 0.0
        or phi_minimum is None
        or phi_minimum <= 0.0
        or phi_condition is None
        or phi_condition < 1.0
        or result["fim_valid"] is not True
        or result["failure_reason"] is not None
        or cost is None
        or stationarity is None
    ):
        raise ValueError("solver result evidence converged fields are invalid")
    _, eigenvalues = covariance_record
    expected_epsilon = 3.0 * math.sqrt(float(eigenvalues[-1]))
    expected_phi_minimum = 1.0 / float(eigenvalues[-1])
    expected_phi_condition = float(eigenvalues[-1] / eigenvalues[0])
    if not (
        math.isclose(epsilon, expected_epsilon, rel_tol=1e-9, abs_tol=1e-12)
        and math.isclose(
            phi_minimum,
            expected_phi_minimum,
            rel_tol=1e-9,
            abs_tol=1e-12,
        )
        and math.isclose(
            phi_condition,
            expected_phi_condition,
            rel_tol=1e-9,
            abs_tol=1e-12,
        )
    ):
        raise ValueError("solver result evidence FIM fields are inconsistent")
    if trace and (
        trace[-1]["accepted"] is not True
        or trace[-1]["trial_cost"] != cost
    ):
        raise ValueError("solver result evidence terminal proposal is invalid")


def _validate_nonconverged_solver_evidence(result):
    status = result["status"]
    estimate = result["estimate"]
    cost = result["cost"]
    stationarity = result["stationarity_norm"]
    phi_minimum = result["phi_min_eigenvalue"]
    if (
        result["covariance"] is not None
        or result["epsilon"] is not None
        or result["phi_condition"] is not None
        or result["fim_valid"] is not False
        or not isinstance(result["failure_reason"], str)
        or not result["failure_reason"]
        or (estimate is not None and _finite_vec2(estimate) is None)
        or (cost is not None and _finite_nonnegative(cost) is None)
        or (
            stationarity is not None
            and _finite_nonnegative(stationarity) is None
        )
        or (status == "failed" and phi_minimum is not None)
        or (
            status == "invalid"
            and phi_minimum is not None
            and _finite_number(phi_minimum) is None
        )
    ):
        raise ValueError("solver result evidence failure fields are invalid")


def _validate_solver_result_evidence(result):
    if not isinstance(result, Mapping):
        raise ValueError("solver result evidence is not a mapping")
    if tuple(result) != SOLVER_RESULT_FIELDS:
        raise ValueError("solver result evidence field order is invalid")
    if not _bounded_solver_evidence(result):
        raise ValueError("solver result evidence is unbounded or forbidden")
    status = result["status"]
    if status not in {"converged", "failed", "invalid"}:
        raise ValueError("solver result evidence status is invalid")
    count = result["proposal_count"]
    iterations = result["iterations"]
    trace = result["proposal_trace"]
    if (
        not _nonnegative_int(count)
        or not _nonnegative_int(iterations)
        or not isinstance(trace, list)
        or count != iterations
        or count != len(trace)
    ):
        raise ValueError("solver result evidence counts are invalid")
    _validate_solver_proposal_trace(trace, converged=status == "converged")
    if status == "converged":
        _validate_converged_solver_evidence(result, trace)
    else:
        _validate_nonconverged_solver_evidence(result)


def _validate_schedule_identity(frame, robot, local, schedule_id):
    if not all(_nonnegative_int(value) for value in (frame, robot, local)):
        raise ValueError("qualified schedule indices are invalid")
    expected = f"frame-{int(frame)}:robot-{int(robot)}:squad-local-{int(local)}"
    if schedule_id != expected:
        raise ValueError("qualified schedule identity is invalid")


def _nonnegative_int(value):
    return (
        not isinstance(value, bool)
        and isinstance(value, Integral)
        and value >= 0
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
    result = [_finite_number(item) for item in value]
    return result if all(item is not None for item in result) else None


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
        and eigenvalues[0] > RELATIVE_SPECTRAL_THRESHOLD * eigenvalues[-1]
    )


def _canonical_provenance(value):
    return (
        isinstance(value, list)
        and all(_nonnegative_int(item) for item in value)
        and value == sorted(set(value))
    )


def _reject_forbidden_runtime_fields(value, active_ids=None):
    if active_ids is None:
        active_ids = set()
    if isinstance(value, Mapping):
        identifier = id(value)
        if identifier in active_ids:
            raise ValueError("cyclic qualified row payload")
        active_ids.add(identifier)
        try:
            for key, nested in value.items():
                if not isinstance(key, str):
                    raise ValueError("qualified row key is invalid")
                lowered = key.lower()
                if any(fragment in lowered for fragment in (
                    "truth",
                    "analyzer",
                    "future",
                    "realized",
                )):
                    raise ValueError("forbidden qualified runtime field")
                _reject_forbidden_runtime_fields(nested, active_ids)
        finally:
            active_ids.remove(identifier)
    elif isinstance(value, list):
        identifier = id(value)
        if identifier in active_ids:
            raise ValueError("cyclic qualified row payload")
        active_ids.add(identifier)
        try:
            for item in value:
                _reject_forbidden_runtime_fields(item, active_ids)
        finally:
            active_ids.remove(identifier)
