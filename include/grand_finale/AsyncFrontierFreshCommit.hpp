#pragma once

#include "grand_finale/AsyncFrontierProposal.hpp"
#include "grand_finale/BrakingSnapshotOracle.hpp"
#include "grand_finale/CanonicalGammaStarFeedback.hpp"
#include "grand_finale/CanonicalHocbfQpController.hpp"
#include "grand_finale/SharedCollisionViableFrontierAllocator.hpp"

#include <set>

namespace gf {

struct FreshCommitRequest {
    AsyncFrontierProposalRequest proposal_request;
    AsyncFrontierProposalRequest latest;
    std::map<NodeId,FrontierCell> targets;
    std::set<std::string> uncovered_cell_ids;
    std::set<std::string> denominator_cell_ids;
    std::string expected_bundle_id;
    std::string current_priority_bundle_id;
    bool pending_union = false;
    bool pending_retreat = false;
    double maximum_estimator_age_s = 1.0;
    std::size_t minimum_effective_reference_count = 0;
    double minimum_robust_fim_margin =
        -std::numeric_limits<double>::infinity();
    double minimum_posterior_margin =
        -std::numeric_limits<double>::infinity();
    double minimum_aoi_margin =
        -std::numeric_limits<double>::infinity();
    double position_gain = 0.0;
    double velocity_gain = 0.0;
    double dt_s = 0.0;
    double estimator_acceleration_variance = 0.0;
    double residual_tolerance = 1e-7;
    CanonicalGammaFeedbackConfig gamma_config;
};

struct FreshCommitResult {
    bool accepted = false;
    std::string reason;
    std::map<NodeId,Eigen::Vector2d> provisional_nominal;
    std::map<NodeId,Eigen::Vector2d> controls;
    std::vector<CanonicalHardRow> row_ledger;
    double minimum_current_gamma = std::numeric_limits<double>::infinity();
    double minimum_robust_residual = std::numeric_limits<double>::infinity();
    std::size_t forbidden_search_calls = 0;
};

inline std::string frontierBundleId(
    const std::map<NodeId,FrontierCell>& targets) {
    std::string result;
    for (const auto& [owner,cell] : targets)
        result+=std::to_string(owner)+"="+cell.id()+";";
    return result;
}

inline bool sameFreshCommitStructure(
    const AsyncFrontierProposalRequest& lhs,
    const AsyncFrontierProposalRequest& rhs) {
    return lhs.topology_version==rhs.topology_version &&
        lhs.topology_digest==rhs.topology_digest &&
        lhs.mode==rhs.mode &&
        lhs.target_epoch==rhs.target_epoch &&
        lhs.denominator_version==rhs.denominator_version &&
        lhs.tube_policy_version==rhs.tube_policy_version &&
        lhs.config_version==rhs.config_version;
}

inline FreshCommitResult evaluateFreshCommit(const FreshCommitRequest& request) {
    const auto reject=[](const std::string& gate) {
        FreshCommitResult result;
        result.reason="fresh_commit_rejected:"+gate;
        return result;
    };
    if (request.latest.mode!=SupervisorMode::Search)
        return reject("mode");
    if (request.pending_union || request.pending_retreat)
        return reject("pending_transition");
    if (!sameFreshCommitStructure(request.proposal_request,request.latest))
        return reject("stale_structure");
    if (!std::isfinite(request.maximum_estimator_age_s) ||
        request.maximum_estimator_age_s<0.0 ||
        request.latest.estimator_time_s-request.proposal_request.estimator_time_s>
            request.maximum_estimator_age_s)
        return reject("estimator_age");
    if (request.targets.empty() ||
        frontierBundleId(request.targets)!=request.expected_bundle_id ||
        request.expected_bundle_id!=request.current_priority_bundle_id)
        return reject("fairness_or_bundle");
    std::set<std::string> unique;
    for (const auto& [owner,cell] : request.targets) {
        (void)owner;
        if (request.denominator_cell_ids.count(cell.id())==0)
            return reject("cell_denominator");
        if (request.uncovered_cell_ids.count(cell.id())==0)
            return reject("cell_covered");
        if (!unique.insert(cell.id()).second)
            return reject("target_uniqueness");
    }
    if (request.minimum_effective_reference_count<2)
        return reject("double_reference");
    if (!std::isfinite(request.minimum_robust_fim_margin) ||
        request.minimum_robust_fim_margin<0.0)
        return reject("robust_fim");
    if (!std::isfinite(request.minimum_posterior_margin) ||
        request.minimum_posterior_margin<0.0)
        return reject("posterior");
    if (!std::isfinite(request.minimum_aoi_margin) ||
        request.minimum_aoi_margin<0.0)
        return reject("aoi");
    if (!std::isfinite(request.dt_s) || request.dt_s<=0.0 ||
        !std::isfinite(request.position_gain) || request.position_gain<0.0 ||
        !std::isfinite(request.velocity_gain) || request.velocity_gain<0.0)
        return reject("configuration");

    const auto hard=request.latest.canonical_blueprint.build(
        request.latest.estimator_snapshot);
    const auto braking=evaluateBrakingSnapshot(
        hard,request.dt_s,request.residual_tolerance);
    if (!braking.hard_polytope_nonempty)
        return reject("hard_polytope_empty");
    if (!braking.snapshot_braking_admissible)
        return reject("snapshot_braking_inadmissible");

    std::map<NodeId,Eigen::Vector2d> nominal;
    for (std::size_t index=0;
         index<request.latest.estimator_snapshot.mobile_ids.size();++index) {
        const NodeId owner=request.latest.estimator_snapshot.mobile_ids[index];
        const auto target=request.targets.find(owner);
        if (target==request.targets.end()) return reject("target_incomplete");
        const Eigen::Vector4d state=
            request.latest.estimator_snapshot.mean.segment<4>(4*index);
        Eigen::Vector2d value=request.position_gain*
            (target->second.center-state.head<2>())-
            request.velocity_gain*state.tail<2>();
        value.x()=std::clamp(value.x(),-hard.acceleration_half_box,
                            hard.acceleration_half_box);
        value.y()=std::clamp(value.y(),-hard.acceleration_half_box,
                            hard.acceleration_half_box);
        nominal[owner]=value;
    }
    CanonicalGammaFeedbackEvaluationContext context;
    const auto blueprint=request.latest.canonical_blueprint;
    const auto feedback=evaluateCanonicalGammaFeedbackBatch(
        request.latest.estimator_snapshot,nominal,request.gamma_config,
        request.dt_s,request.estimator_acceleration_variance,
        [blueprint](const JointEstimateSnapshot& snapshot) {
            return blueprint.build(snapshot);
        },context);
    if (!feedback.valid) {
        if (feedback.reason=="current_gamma_negative")
            return reject("hard_polytope_empty");
        return reject("gamma_feedback:"+feedback.reason);
    }
    FreshCommitResult result;
    // The formal adapter must see the original estimator-based target nominal
    // and execute the canonical selector exactly once on the applied path.
    // The dry-run below verifies that this nominal has a certifiable current
    // realization; its selected control is diagnostic, not re-injected.
    result.provisional_nominal=nominal;
    result.row_ledger=feedback.current_rows;
    CanonicalHocbfQpController controller;
    for (NodeId owner : request.latest.estimator_snapshot.mobile_ids) {
        result.minimum_current_gamma=std::min(
            result.minimum_current_gamma,feedback.stages.at(owner).current_gamma);
        const auto solved=controller.solve({
            request.latest.profile,owner,request.latest.estimator_version,
            request.latest.topology_version,SupervisorMode::Search,
            feedback.selected_controls.at(owner),hard.acceleration_half_box,
            feedback.current_rows,request.residual_tolerance});
        if (!controlMayBeApplied(
                solved,request.latest.estimator_version,
                request.latest.topology_version,SupervisorMode::Search))
            return reject("online_qp:"+solved.failure_reason);
        const double residual=minimumCanonicalOwnerResidual(
            feedback.current_rows,owner,solved.control);
        if (residual < -request.residual_tolerance)
            return reject("residual");
        result.minimum_robust_residual=std::min(
            result.minimum_robust_residual,residual);
        result.controls[owner]=solved.control;
    }
    result.accepted=true;
    result.reason="fresh_commit_accepted";
    return result;
}

}  // namespace gf
