#pragma once

#include "grand_finale/BrakingSnapshotOracle.hpp"
#include "grand_finale/ExactHardPolytopeDiagnostic.hpp"
#include "grand_finale/Task10p11hCoastalScenarios.hpp"
#include "grand_finale/Task10p11hSimpleCoverageController.hpp"

#include <cstdint>
#include <iomanip>
#include <sstream>

namespace gf {

struct Task10p11jOwnerTick {
    Eigen::Vector4d estimate_before = Eigen::Vector4d::Zero();
    Eigen::Vector4d estimate_after = Eigen::Vector4d::Zero();
    Eigen::Vector4d truth_before = Eigen::Vector4d::Zero();
    Eigen::Vector4d truth_after = Eigen::Vector4d::Zero();
    Eigen::Vector2d raw_target = Eigen::Vector2d::Zero();
    bool raw_target_present = false;
    bool lifted_target_active = false;
    Eigen::Vector2d nominal = Eigen::Vector2d::Zero();
    Eigen::Vector2d hard_projection = Eigen::Vector2d::Zero();
    bool hard_projection_available = false;
    Eigen::Vector2d selected_gamma_candidate = Eigen::Vector2d::Zero();
    bool selected_gamma_candidate_available = false;
    Eigen::Vector2d maximum_margin_control = Eigen::Vector2d::Zero();
    bool maximum_margin_control_available = false;
    Eigen::Vector2d applied_control = Eigen::Vector2d::Zero();
    bool control_applied = false;
    double current_gamma = -std::numeric_limits<double>::infinity();
    double no_measurement_predicted_next_gamma =
        -std::numeric_limits<double>::infinity();
    double local_pair_residual = -std::numeric_limits<double>::infinity();
};

struct Task10p11jCausalTick {
    double time_before_s = 0.0;
    double time_after_s = 0.0;
    bool advanced = false;
    std::string reason;
    bool t100_latched = false;
    double truth_coverage_before = 0.0;
    double truth_coverage_after = 0.0;
    std::uint64_t estimator_version_before = 0;
    std::uint64_t estimator_version_after = 0;
    std::uint64_t topology_version = 0;
    SupervisorMode mode = SupervisorMode::Search;
    std::size_t target_epoch_before = 0;
    std::size_t target_epoch_after = 0;
    std::uint64_t target_digest_before = 0;
    std::uint64_t target_digest_after = 0;
    bool target_changed = false;
    std::map<NodeId,FrontierCell> raw_targets_after;
    bool pending_transition = false;
    std::vector<DirectedEdge> topology;
    PairwiseSnapshotTube pair_tube;
    Task10p11jOwnerTick first;
    Task10p11jOwnerTick second;
    MobilePairBrakingAudit braking;
    double pair_h = 0.0;
    double pair_hdot = 0.0;
    double pair_psi1 = 0.0;
    double pair_half_constant = 0.0;
    double pair_coefficient_reserve_each = 0.0;
    double centralized_applied_residual =
        -std::numeric_limits<double>::infinity();
    double minimum_hard_residual =
        std::numeric_limits<double>::infinity();
    bool exact_owner10_feasible = false;
    bool exact_owner11_feasible = false;
    std::vector<std::string> owner10_conflict;
    std::vector<std::string> owner11_conflict;
    double estimator_jump_norm = 0.0;
    double truth_jump_norm = 0.0;
    double nominal_target_change_effect_mps2 = 0.0;
    double measurement_update_correction_norm = 0.0;
};

struct Task10p11jCausalReplay {
    bool reproduced = false;
    std::string reason;
    std::optional<double> t100_s;
    double failure_time_s = -1.0;
    double first_raw_target_change_s = -1.0;
    double first_significant_nominal_change_s = -1.0;
    double first_negative_braking_slack_s = -1.0;
    double first_empty_polytope_s = -1.0;
    std::vector<Task10p11jCausalTick> ticks;
};

struct Task10p11jFrozenTargetCounterfactual {
    bool prefix_valid = false;
    bool advanced_through_4p6_s = false;
    std::string reason;
    double failure_time_s = -1.0;
    double minimum_current_gamma = std::numeric_limits<double>::infinity();
    double minimum_braking_slack_m = std::numeric_limits<double>::infinity();
    std::map<NodeId,FrontierCell> frozen_targets;
};

namespace task10p11j_detail {

inline std::uint64_t targetDigest(
    const std::map<NodeId,FrontierCell>& targets) {
    std::ostringstream out;
    out<<std::setprecision(17);
    for (const auto& [owner,target] : targets)
        out<<owner<<':'<<target.id()<<':'<<target.center.x()<<':'
           <<target.center.y()<<';';
    return leader_coverage_detail::hashText(out.str());
}

inline Eigen::Vector4d truthState(const Swarm& swarm,NodeId owner) {
    const auto found=std::find_if(swarm.robots.begin(),swarm.robots.end(),
        [&](const auto& robot) { return robot->id==owner; });
    if (found==swarm.robots.end())
        throw std::invalid_argument("missing truth robot in causal fixture");
    const Point position=(*found)->model->xy();
    const Point velocity=(*found)->model->getVelocity();
    return {position.x,position.y,velocity.x,velocity.y};
}

inline double yaw(const Swarm& swarm,NodeId owner) {
    const auto found=std::find_if(swarm.robots.begin(),swarm.robots.end(),
        [&](const auto& robot) { return robot->id==owner; });
    if (found==swarm.robots.end())
        throw std::invalid_argument("missing yaw robot in causal fixture");
    return (*found)->model->getStateVariable("yawRad");
}

inline Eigen::Vector4d estimatedState(
    const JointEstimateSnapshot& snapshot,NodeId owner) {
    const auto found=std::find(
        snapshot.mobile_ids.begin(),snapshot.mobile_ids.end(),owner);
    if (found==snapshot.mobile_ids.end())
        throw std::invalid_argument("missing estimated owner in causal fixture");
    const auto index=std::distance(snapshot.mobile_ids.begin(),found);
    return snapshot.mean.segment<4>(4*index);
}

inline const CanonicalHardRow& pairRow(
    const std::vector<CanonicalHardRow>& rows,NodeId owner) {
    const std::string id="collision:10--11:owner:"+std::to_string(owner);
    const auto found=std::find_if(rows.begin(),rows.end(),
        [&](const auto& row) { return row.id==id; });
    if (found==rows.end())
        throw std::invalid_argument("missing 10--11 collision row");
    return *found;
}

inline Task10p11gSoftTaskResult nominalFor(
    const GrandFinaleRuntimeSnapshot& runtime,const Swarm& swarm,
    NodeId owner,const std::map<NodeId,FrontierCell>& targets,
    const GrandFinaleSwarmAdapterConfig& config) {
    auto model=task10p11gFrozenModel();
    model.acceleration_half_box_mps2=config.acceleration_half_box;
    model.position_gain=config.position_gain;
    model.velocity_gain=config.velocity_gain;
    model.maximum_yaw_rate_radps=config.maximum_yaw_rate_radps;
    const Eigen::Vector4d state=estimatedState(runtime.estimate,owner);
    const auto target=targets.find(owner);
    const std::optional<Eigen::Vector2d> point=target==targets.end()
        ?std::optional<Eigen::Vector2d>{}
        :std::optional<Eigen::Vector2d>{target->second.center};
    return task10p11gSoftTask({state.head<2>(),state.tail<2>(),
        yaw(swarm,owner),point,runtime.mode},model);
}

inline void fillOwnerBefore(
    Task10p11jOwnerTick& owner_tick,NodeId owner,
    const GrandFinaleRuntimeSnapshot& before,const Swarm& swarm) {
    owner_tick.estimate_before=estimatedState(before.estimate,owner);
    owner_tick.truth_before=truthState(swarm,owner);
}

inline void fillOwnerAfter(
    Task10p11jOwnerTick& owner_tick,NodeId owner,
    const GrandFinaleRuntimeSnapshot& after,const Swarm& swarm) {
    owner_tick.estimate_after=estimatedState(after.estimate,owner);
    owner_tick.truth_after=truthState(swarm,owner);
}

}  // namespace task10p11j_detail

inline Task10p11jCausalReplay replayTask10p11jOpenSourceEasy() {
    using namespace task10p11j_detail;
    BoundaryPolicyConfig boundary;
    boundary.policy=BoundaryPolicy::None;
    auto fixture=makeTask10p11gFixture(
        task10p11hCoastalLeaderEasyScenario(),
        SolverProfile::OpenSource,boundary,0.1);
    Task10p11jCausalReplay replay;
    const auto stage=fixture->adapter.initializeStageZero();
    if (!stage.initialized) {
        replay.reason=stage.reason;
        return replay;
    }
    Task10p11hSimpleCoverageController controller(
        fixture->swarm,fixture->adapter);
    for (int cycle=0;cycle<60;++cycle) {
        const auto before=fixture->adapter.runtimeSnapshot();
        const auto targets_before=controller.committedTargets();
        const auto rows=fixture->adapter.currentSnapshotHardRows(before.topology);
        const auto request=
            fixture->adapter.currentSnapshotHardRowRequest(before.topology);
        const auto& first_row=pairRow(rows,10);
        const auto& second_row=pairRow(rows,11);
        Task10p11jCausalTick tick;
        tick.time_before_s=before.runtime_s;
        tick.truth_coverage_before=fixture->adapter.coverage().truthFraction();
        tick.estimator_version_before=before.estimator_token;
        tick.topology_version=before.topology_token;
        tick.mode=before.mode;
        tick.target_epoch_before=controller.targetEpoch();
        tick.target_digest_before=targetDigest(targets_before);
        tick.pending_transition=before.adapter_transition_pending ||
            before.supervisor_transition_pending;
        tick.topology=before.topology;
        tick.pair_tube=request.collision_snapshot_tubes.at("10--11");
        fillOwnerBefore(tick.first,10,before,fixture->swarm);
        fillOwnerBefore(tick.second,11,before,fixture->swarm);
        tick.braking=evaluateMobilePairBrakingRequestRows(
            request,rows,UndirectedEdge::canonical(10,11));
        tick.pair_h=first_row.barrier_h;
        tick.pair_hdot=first_row.barrier_hdot;
        tick.pair_psi1=first_row.barrier_psi1;
        tick.pair_half_constant=first_row.constant;
        tick.pair_coefficient_reserve_each=
            first_row.coefficient_uncertainty_reserve;
        const auto advanced=controller.advance();
        const auto targets_after=controller.committedTargets();
        tick.advanced=advanced.step.advanced;
        tick.reason=advanced.reason;
        tick.t100_latched=advanced.t100_event_latched;
        tick.target_epoch_after=controller.targetEpoch();
        tick.target_digest_after=targetDigest(targets_after);
        tick.target_changed=tick.target_digest_after!=tick.target_digest_before;
        tick.raw_targets_after=targets_after;
        const auto assign_target=[&](Task10p11jOwnerTick& value,NodeId owner) {
            const auto target=targets_after.find(owner);
            if (target!=targets_after.end()) {
                value.raw_target_present=true;
                value.raw_target=target->second.center;
            }
            const auto nominal=nominalFor(
                before,fixture->swarm,owner,targets_after,
                fixture->adapter.config());
            value.nominal=nominal.acceleration;
            const auto prior=targets_before.find(owner);
            if (prior!=targets_before.end() && target!=targets_after.end() &&
                !prior->second.center.isApprox(target->second.center,1e-12)) {
                auto old_targets=targets_after;
                old_targets[owner]=prior->second;
                const auto old_nominal=nominalFor(
                    before,fixture->swarm,owner,old_targets,
                    fixture->adapter.config());
                tick.nominal_target_change_effect_mps2=std::max(
                    tick.nominal_target_change_effect_mps2,
                    (nominal.acceleration-old_nominal.acceleration).norm());
            }
            const auto gamma=solveCanonicalGammaStar(
                rows,owner,fixture->adapter.config().acceleration_half_box);
            value.current_gamma=gamma.gamma;
            if (gamma.valid) {
                value.maximum_margin_control={gamma.accelX,gamma.accelY};
                value.maximum_margin_control_available=true;
            }
            const auto projected=evaluateProgressCompatibility(
                rows,owner,value.nominal,
                fixture->adapter.config().acceleration_half_box,
                {std::numeric_limits<double>::max(),0.0,1e-10,true});
            value.hard_projection_available=projected.polytope_nonempty;
            if (projected.polytope_nonempty)
                value.hard_projection=projected.projection;
            const auto feedback=advanced.step.gamma_feedback.find(owner);
            if (feedback!=advanced.step.gamma_feedback.end()) {
                value.hard_projection=feedback->second.current_hard_projection;
                value.hard_projection_available=true;
                value.selected_gamma_candidate=
                    feedback->second.selected_nominal;
                value.selected_gamma_candidate_available=true;
                value.maximum_margin_control=
                    feedback->second.maximum_margin_control;
                value.maximum_margin_control_available=true;
            }
            const auto control=advanced.step.applied_controls.find(owner);
            if (control!=advanced.step.applied_controls.end()) {
                value.applied_control=control->second;
                value.control_applied=true;
            }
        };
        assign_target(tick.first,10);
        assign_target(tick.second,11);
        const auto audit_owner=[&](NodeId owner) {
            return diagnoseHardPolytope(owner,before.runtime_s,before.mode,
                before.topology,rows,
                fixture->adapter.config().acceleration_half_box);
        };
        const auto first_exact=audit_owner(10);
        const auto second_exact=audit_owner(11);
        tick.exact_owner10_feasible=first_exact.exact_feasible;
        tick.exact_owner11_feasible=second_exact.exact_feasible;
        tick.owner10_conflict=first_exact.minimal_conflict_row_ids;
        tick.owner11_conflict=second_exact.minimal_conflict_row_ids;
        if (tick.first.control_applied && tick.second.control_applied) {
            tick.first.local_pair_residual=
                first_row.margin(tick.first.applied_control);
            tick.second.local_pair_residual=
                second_row.margin(tick.second.applied_control);
            tick.centralized_applied_residual=
                tick.first.local_pair_residual+
                tick.second.local_pair_residual;
        }
        tick.minimum_hard_residual=advanced.step.minimum_hard_residual;
        const auto after=fixture->adapter.runtimeSnapshot();
        tick.time_after_s=after.runtime_s;
        tick.truth_coverage_after=fixture->adapter.coverage().truthFraction();
        tick.estimator_version_after=after.estimator_token;
        fillOwnerAfter(tick.first,10,after,fixture->swarm);
        fillOwnerAfter(tick.second,11,after,fixture->swarm);
        tick.estimator_jump_norm=std::max(
            (tick.first.estimate_after-tick.first.estimate_before).norm(),
            (tick.second.estimate_after-tick.second.estimate_before).norm());
        tick.truth_jump_norm=std::max(
            (tick.first.truth_after-tick.first.truth_before).norm(),
            (tick.second.truth_after-tick.second.truth_before).norm());
        if (advanced.step.advanced) {
            const auto predicted=predictNoMeasurementSnapshot(
                before.estimate,advanced.step.applied_controls,
                fixture->adapter.config().dt_s,
                fixture->adapter.config().estimator_acceleration_variance);
            tick.measurement_update_correction_norm=std::max(
                (estimatedState(after.estimate,10)-
                 estimatedState(predicted,10)).norm(),
                (estimatedState(after.estimate,11)-
                 estimatedState(predicted,11)).norm());
            const auto predicted_rows=buildCanonicalHardRows(
                fixture->adapter.snapshotHardRowRequest(
                    predicted,before.topology));
            tick.first.no_measurement_predicted_next_gamma=
                solveCanonicalGammaStar(predicted_rows,10,
                    fixture->adapter.config().acceleration_half_box).gamma;
            tick.second.no_measurement_predicted_next_gamma=
                solveCanonicalGammaStar(predicted_rows,11,
                    fixture->adapter.config().acceleration_half_box).gamma;
        }
        if (tick.time_before_s>=3.0-1e-12)
            replay.ticks.push_back(tick);
        if (tick.t100_latched && !replay.t100_s.has_value())
            replay.t100_s=after.runtime_s;
        if (tick.target_changed && tick.time_before_s>=3.0-1e-12 &&
            replay.first_raw_target_change_s<0.0)
            replay.first_raw_target_change_s=tick.time_before_s;
        if (tick.nominal_target_change_effect_mps2>0.1 &&
            replay.first_significant_nominal_change_s<0.0)
            replay.first_significant_nominal_change_s=tick.time_before_s;
        if (tick.braking.valid && tick.braking.braking_slack_m<0.0 &&
            replay.first_negative_braking_slack_s<0.0)
            replay.first_negative_braking_slack_s=tick.time_before_s;
        if ((!tick.exact_owner10_feasible || !tick.exact_owner11_feasible) &&
            replay.first_empty_polytope_s<0.0)
            replay.first_empty_polytope_s=tick.time_before_s;
        if (!advanced.step.advanced) {
            replay.failure_time_s=before.runtime_s;
            replay.reason=advanced.reason;
            replay.reproduced=replay.t100_s.has_value() &&
                std::abs(*replay.t100_s-3.9)<=1e-9 &&
                std::abs(replay.failure_time_s-4.5)<=1e-9 &&
                !tick.exact_owner10_feasible && !tick.exact_owner11_feasible &&
                tick.truth_jump_norm<=1e-12 &&
                tick.estimator_version_after==tick.estimator_version_before;
            break;
        }
    }
    return replay;
}

inline Task10p11jFrozenTargetCounterfactual
replayTask10p11jFrozenPreT100Targets() {
    using namespace task10p11j_detail;
    BoundaryPolicyConfig boundary;
    boundary.policy=BoundaryPolicy::None;
    auto fixture=makeTask10p11gFixture(
        task10p11hCoastalLeaderEasyScenario(),
        SolverProfile::OpenSource,boundary,0.1);
    Task10p11jFrozenTargetCounterfactual result;
    if (!fixture->adapter.initializeStageZero().initialized) {
        result.reason="stage_zero_failed";
        return result;
    }
    Task10p11hSimpleCoverageController controller(
        fixture->swarm,fixture->adapter);
    while (fixture->swarm.robots.front()->runtime<4.0-1e-12) {
        const auto step=controller.advance();
        if (!step.step.advanced) {
            result.reason="prefix_"+step.reason;
            result.failure_time_s=fixture->swarm.robots.front()->runtime;
            return result;
        }
    }
    result.prefix_valid=true;
    result.frozen_targets=controller.committedTargets();
    while (fixture->swarm.robots.front()->runtime<4.6-1e-12) {
        const auto runtime=fixture->adapter.runtimeSnapshot();
        const auto rows=fixture->adapter.currentSnapshotHardRows(
            runtime.topology);
        const auto braking=evaluateMobilePairBraking(
            fixture->adapter.currentSnapshotHardRowRequest(runtime.topology),
            UndirectedEdge::canonical(10,11));
        result.minimum_braking_slack_m=std::min(
            result.minimum_braking_slack_m,braking.braking_slack_m);
        result.minimum_current_gamma=std::min({
            result.minimum_current_gamma,
            solveCanonicalGammaStar(rows,10,
                fixture->adapter.config().acceleration_half_box).gamma,
            solveCanonicalGammaStar(rows,11,
                fixture->adapter.config().acceleration_half_box).gamma});
        std::map<NodeId,Eigen::Vector2d> nominal;
        std::map<NodeId,double> yaw_rates;
        for (NodeId owner:runtime.estimate.mobile_ids) {
            const auto soft=nominalFor(runtime,fixture->swarm,owner,
                result.frozen_targets,fixture->adapter.config());
            nominal[owner]=soft.acceleration;
            yaw_rates[owner]=soft.yaw_rate_radps;
        }
        const auto step=fixture->adapter.stepWithNominalAndYawRates(
            nominal,yaw_rates);
        if (!step.advanced) {
            result.reason=step.reason;
            result.failure_time_s=runtime.runtime_s;
            return result;
        }
    }
    result.advanced_through_4p6_s=true;
    result.reason="diagnostic_advanced";
    return result;
}

inline Task10p11jFrozenTargetCounterfactual
replayTask10p11jLinearTargetHomotopy() {
    using namespace task10p11j_detail;
    const auto actual=replayTask10p11jOpenSourceEasy();
    const auto centroid_tick=std::find_if(
        actual.ticks.begin(),actual.ticks.end(),[](const auto& tick) {
            return std::abs(tick.time_before_s-4.0)<=1e-12;
        });
    if (centroid_tick==actual.ticks.end())
        throw std::runtime_error("missing direct-centroid target ledger");
    BoundaryPolicyConfig boundary;
    boundary.policy=BoundaryPolicy::None;
    auto fixture=makeTask10p11gFixture(
        task10p11hCoastalLeaderEasyScenario(),
        SolverProfile::OpenSource,boundary,0.1);
    Task10p11jFrozenTargetCounterfactual result;
    if (!fixture->adapter.initializeStageZero().initialized) {
        result.reason="stage_zero_failed";
        return result;
    }
    Task10p11hSimpleCoverageController controller(
        fixture->swarm,fixture->adapter);
    while (fixture->swarm.robots.front()->runtime<4.0-1e-12) {
        const auto step=controller.advance();
        if (!step.step.advanced) {
            result.reason="prefix_"+step.reason;
            return result;
        }
    }
    result.prefix_valid=true;
    result.frozen_targets=controller.committedTargets();
    while (fixture->swarm.robots.front()->runtime<4.6-1e-12) {
        const auto runtime=fixture->adapter.runtimeSnapshot();
        const double alpha=std::clamp((runtime.runtime_s-4.0)/0.5,0.0,1.0);
        std::map<NodeId,FrontierCell> targets=result.frozen_targets;
        for (auto& [owner,target]:targets) {
            const auto final=centroid_tick->raw_targets_after.find(owner);
            if (final==centroid_tick->raw_targets_after.end())
                throw std::runtime_error("incomplete centroid target ledger");
            target.center=(1.0-alpha)*target.center+alpha*final->second.center;
        }
        const auto rows=fixture->adapter.currentSnapshotHardRows(
            runtime.topology);
        const auto braking=evaluateMobilePairBraking(
            fixture->adapter.currentSnapshotHardRowRequest(runtime.topology),
            UndirectedEdge::canonical(10,11));
        result.minimum_braking_slack_m=std::min(
            result.minimum_braking_slack_m,braking.braking_slack_m);
        result.minimum_current_gamma=std::min({
            result.minimum_current_gamma,
            solveCanonicalGammaStar(rows,10,
                fixture->adapter.config().acceleration_half_box).gamma,
            solveCanonicalGammaStar(rows,11,
                fixture->adapter.config().acceleration_half_box).gamma});
        std::map<NodeId,Eigen::Vector2d> nominal;
        std::map<NodeId,double> yaw_rates;
        for (NodeId owner:runtime.estimate.mobile_ids) {
            const auto soft=nominalFor(runtime,fixture->swarm,owner,targets,
                fixture->adapter.config());
            nominal[owner]=soft.acceleration;
            yaw_rates[owner]=soft.yaw_rate_radps;
        }
        const auto step=fixture->adapter.stepWithNominalAndYawRates(
            nominal,yaw_rates);
        if (!step.advanced) {
            result.reason=step.reason;
            result.failure_time_s=runtime.runtime_s;
            return result;
        }
    }
    result.advanced_through_4p6_s=true;
    result.reason="diagnostic_advanced";
    return result;
}

inline CanonicalHardRowRequest task10p11jMinimalPairRequest(
    const Task10p11jCausalTick& tick,double radial_velocity_delta_mps=0.0) {
    CanonicalHardRowRequest request;
    request.mobile_ids={10,11};
    Eigen::Vector4d first=tick.first.estimate_before;
    Eigen::Vector4d second=tick.second.estimate_before;
    const Eigen::Vector2d direction=
        (first.head<2>()-second.head<2>()).normalized();
    first.tail<2>()+=0.5*radial_velocity_delta_mps*direction;
    second.tail<2>()-=0.5*radial_velocity_delta_mps*direction;
    request.states[10]={{first.x(),first.y()},first.tail<2>(),{0.0,0.0}};
    request.states[11]={{second.x(),second.y()},second.tail<2>(),{0.0,0.0}};
    request.collision_pairs={UndirectedEdge::canonical(10,11)};
    request.reference_spec={
        PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0,0.0,1.0,1.0,1.0,0.0};
    request.collision_spec={
        PairwiseSecondOrderBarrierKind::CollisionLower,
        0.1,0.0,1.0,1.0,1.0,0.0};
    request.acceleration_half_box=4.0;
    request.require_snapshot_robust_rows=true;
    request.collision_snapshot_tubes["10--11"]=tick.pair_tube;
    return request;
}

}  // namespace gf
