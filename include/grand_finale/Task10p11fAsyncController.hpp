#pragma once

#include "grand_finale/AsyncFrontierFreshCommit.hpp"
#include "grand_finale/AsyncFrontierProposalKernel.hpp"
#include "grand_finale/Task10p11SharedFrontierFixture.hpp"
#include "grand_finale/Task10p11gFrozenModel.hpp"

#include <sstream>
#include <iomanip>

namespace gf {

struct Task10p11fCommitApplyResult {
    bool committed = false;
    std::string reason;
    FreshCommitResult fresh;
    GrandFinaleSwarmStep step;
};

class Task10p11fAtomicController {
public:
    Task10p11fAtomicController(Swarm& swarm,GrandFinaleSwarmAdapter& adapter)
        : swarm_(swarm),adapter_(adapter) {}

    FreshCommitRequest freshCommitRequest(
        const std::map<NodeId,FrontierCell>& targets) {
        const auto runtime=adapter_.runtimeSnapshot();
        FreshCommitRequest request;
        request.latest=provenance(runtime);
        request.proposal_request=request.latest;
        request.targets=targets;
        request.expected_bundle_id=frontierBundleId(targets);
        request.current_priority_bundle_id=request.expected_bundle_id;
        request.pending_union=runtime.mode==SupervisorMode::Union ||
            runtime.adapter_transition_pending;
        request.pending_retreat=runtime.mode==SupervisorMode::Retreat ||
            runtime.pending_is_retreat;
        for (const auto& cell :
             swarm_.robots.front()->gridWorld.getUnexploredCellCenters()) {
            const std::string id=std::to_string(cell.x_index)+":"+
                std::to_string(cell.y_index);
            request.uncovered_cell_ids.insert(id);
            request.denominator_cell_ids.insert(id);
        }
        for (const auto& [owner,cell] : targets) {
            (void)owner;
            request.denominator_cell_ids.insert(cell.id());
        }
        const auto reference=adapter_.currentReferenceAudit();
        request.minimum_effective_reference_count=
            reference.minimum_effective_reference_count;
        request.minimum_robust_fim_margin=
            reference.minimum_robust_fim_cone_lower_bound-1e-6;
        request.minimum_posterior_margin=
            adapter_.config().maximum_posterior_eigenvalue_m2-
            reference.maximum_posterior_eigenvalue;
        request.minimum_aoi_margin=reference.minimum_range_aoi_margin_s;
        request.position_gain=adapter_.config().position_gain;
        request.velocity_gain=adapter_.config().velocity_gain;
        request.dt_s=adapter_.config().dt_s;
        request.estimator_acceleration_variance=
            adapter_.config().estimator_acceleration_variance;
        request.gamma_config={
            adapter_.config().acceleration_half_box,
            adapter_.config().gamma_feedback_homotopy_segments,
            adapter_.config().gamma_feedback_selection,
            adapter_.config().predictive_gamma_tau_mps2,
            adapter_.config().gamma_feedback_tolerance};
        return request;
    }

    Task10p11fCommitApplyResult commitAndApply(
        const FreshCommitRequest& request) {
        Task10p11fCommitApplyResult result;
        result.fresh=evaluateFreshCommit(request);
        if (!result.fresh.accepted) {
            result.reason=result.fresh.reason;
            return result;
        }
        result.step=adapter_.config().maximum_yaw_rate_radps>0.0
            ? adapter_.stepWithNominalAndYawRates(
                result.fresh.provisional_nominal,
                yawRates(request.targets,adapter_.runtimeSnapshot()))
            : adapter_.stepWithNominal(result.fresh.provisional_nominal);
        if (!result.step.advanced) {
            result.reason="fresh_commit_rejected:online_apply:"+
                result.step.reason;
            return result;
        }
        committed_targets_=request.targets;
        ++target_epoch_;
        result.committed=true;
        result.reason="fresh_commit_applied";
        return result;
    }

    GrandFinaleSwarmStep advanceWithoutProposal() {
        const auto runtime=adapter_.runtimeSnapshot();
        if (runtime.mode!=SupervisorMode::Search) return adapter_.step();
        std::map<NodeId,Eigen::Vector2d> nominal;
        std::map<NodeId,double> yaw_rates;
        auto model=task10p11gFrozenModel();
        model.acceleration_half_box_mps2=adapter_.config().acceleration_half_box;
        model.maximum_acceleration_norm_mps2=
            model.acceleration_half_box_mps2*std::sqrt(2.0);
        model.position_gain=adapter_.config().position_gain;
        model.velocity_gain=adapter_.config().velocity_gain;
        for (std::size_t index=0;index<runtime.estimate.mobile_ids.size();++index) {
            const NodeId owner=runtime.estimate.mobile_ids[index];
            const Eigen::Vector4d state=runtime.estimate.mean.segment<4>(4*index);
            const auto target=committed_targets_.find(owner);
            const auto soft=task10p11gSoftTask({
                state.head<2>(),state.tail<2>(),currentYaw(owner),
                target==committed_targets_.end()
                    ? std::optional<Eigen::Vector2d>{}
                    : std::optional<Eigen::Vector2d>{target->second.center},
                runtime.mode},model);
            nominal[owner]=soft.acceleration;
            yaw_rates[owner]=soft.yaw_rate_radps;
        }
        return adapter_.stepWithNominalAndYawRates(nominal,yaw_rates);
    }

    std::uint64_t targetEpoch() const { return target_epoch_; }
    const std::map<NodeId,FrontierCell>& committedTargets() const {
        return committed_targets_;
    }

    AsyncFrontierProposalRequest currentProvenance() const {
        return provenance(adapter_.runtimeSnapshot());
    }

private:
    double currentYaw(NodeId owner) const {
        for (const auto& robot : swarm_.robots)
            if (robot->id==owner)
                return robot->model->getStateVariable("yawRad");
        throw std::invalid_argument("missing mobile yaw state");
    }

    std::map<NodeId,double> yawRates(
        const std::map<NodeId,FrontierCell>& targets,
        const GrandFinaleRuntimeSnapshot& runtime) const {
        std::map<NodeId,double> result;
        const auto model=task10p11gFrozenModel();
        for (std::size_t index=0;index<runtime.estimate.mobile_ids.size();++index) {
            const NodeId owner=runtime.estimate.mobile_ids[index];
            const Eigen::Vector4d state=runtime.estimate.mean.segment<4>(4*index);
            const auto found=targets.find(owner);
            if (found==targets.end())
                throw std::invalid_argument("fresh commit target batch incomplete");
            result[owner]=task10p11gSoftTask({
                state.head<2>(),state.tail<2>(),currentYaw(owner),
                found->second.center,runtime.mode},model).yaw_rate_radps;
        }
        return result;
    }

    static std::uint64_t hashText(const std::string& text) {
        std::uint64_t hash=1469598103934665603ULL;
        for (const unsigned char value : text) {
            hash^=value;
            hash*=1099511628211ULL;
        }
        return hash==0?1:hash;
    }

    AsyncFrontierProposalRequest provenance(
        const GrandFinaleRuntimeSnapshot& runtime) const {
        AsyncFrontierProposalRequest value;
        value.request_id=next_request_id_++;
        value.launch_tick=static_cast<std::uint64_t>(std::llround(
            runtime.runtime_s/adapter_.config().dt_s));
        value.logical_ready_tick=value.launch_tick+10;
        value.estimator_version=runtime.estimator_token;
        value.estimator_time_s=runtime.runtime_s;
        value.topology_version=runtime.topology_token;
        std::ostringstream topology;
        for (const auto& edge : runtime.topology) topology<<edge.id()<<';';
        value.topology_digest=topology.str();
        value.mode=runtime.mode;
        value.target_epoch=target_epoch_;
        value.denominator_version=hashText(
            swarm_.config.at("world").dump()+"|cells="+
            std::to_string(swarm_.robots.front()->gridWorld.vis.size()));
        value.frontier_mark_version=static_cast<std::uint64_t>(std::count(
            swarm_.robots.front()->gridWorld.vis.begin(),
            swarm_.robots.front()->gridWorld.vis.end(),true));
        std::ostringstream tube;
        tube<<std::setprecision(17)<<adapter_.config().uncertainty_sigma<<'|'
            <<adapter_.config().certified_shadow_single_position_support_m<<'|'
            <<adapter_.config().certified_shadow_single_velocity_support_mps<<'|'
            <<adapter_.config().certified_shadow_relative_position_support_m<<'|'
            <<adapter_.config().certified_shadow_relative_velocity_support_mps<<'|'
            <<adapter_.config().estimator_acceleration_variance;
        value.tube_policy_version=hashText(tube.str());
        std::ostringstream config;
        config<<std::setprecision(17)<<adapter_.config().dt_s<<'|'
            <<adapter_.config().acceleration_half_box<<'|'
            <<adapter_.config().speed_limit_mps<<'|'
            <<adapter_.config().speed_cbf_gain<<'|'
            <<adapter_.config().maximum_yaw_rate_radps<<'|'
            <<adapter_.config().reference_distance_m<<'|'
            <<adapter_.config().add_reference_distance_m<<'|'
            <<adapter_.config().collision_distance_m<<'|'
            <<adapter_.config().position_gain<<'|'
            <<adapter_.config().velocity_gain<<'|'
            <<adapter_.config().gamma_feedback_homotopy_segments<<'|'
            <<static_cast<int>(adapter_.config().gamma_feedback_selection)<<'|'
            <<static_cast<int>(adapter_.config().solver_profile);
        value.config_version=hashText(config.str());
        value.profile=adapter_.config().solver_profile;
        value.tie_break_seed=2027;
        value.estimator_snapshot=runtime.estimate;
        value.canonical_blueprint={
            adapter_.currentSnapshotHardRowRequest(runtime.topology),
            adapter_.config().uncertainty_sigma,
            adapter_.config().certified_shadow_single_position_support_m,
            adapter_.config().certified_shadow_relative_position_support_m,
            adapter_.config().certified_shadow_relative_velocity_support_mps};
        value.planner_period_s=1.0;
        return value;
    }

    Swarm& swarm_;
    GrandFinaleSwarmAdapter& adapter_;
    std::map<NodeId,FrontierCell> committed_targets_;
    std::uint64_t target_epoch_=0;
    mutable std::uint64_t next_request_id_=1;
};

struct Task10p11fAsyncBoundaryResult {
    AsyncProposalClassification classification=
        AsyncProposalClassification::None;
    std::string reason;
    Task10p11fCommitApplyResult commit;
    GrandFinaleSwarmStep online_step;
};

class Task10p11fAsyncCoordinator {
public:
    Task10p11fAsyncCoordinator(
        Swarm& swarm,GrandFinaleSwarmAdapter& adapter,
        AsyncProposalTimingConfig timing)
        : swarm_(swarm),adapter_(adapter),controller_(swarm,adapter),
          state_(timing) {}

    AsyncFrontierWorkRequest makeWorkRequest() {
        const auto runtime=adapter_.runtimeSnapshot();
        const auto hard=adapter_.currentSnapshotHardRowRequest(runtime.topology);
        AsyncFrontierWorkRequest work;
        work.provenance=controller_.currentProvenance();
        work.allocator_config=task10p11AllocatorConfig();
        work.allocation.snapshot_token=runtime.estimator_token;
        work.allocation.topology_token=runtime.topology_token;
        work.allocation.grid_token=static_cast<std::uint64_t>(std::count(
            swarm_.robots.front()->gridWorld.vis.begin(),
            swarm_.robots.front()->gridWorld.vis.end(),true));
        for (std::size_t index=0;index<runtime.estimate.mobile_ids.size();++index)
            work.allocation.agents.push_back({
                runtime.estimate.mobile_ids[index],
                runtime.estimate.mean.segment<2>(4*index),
                runtime.estimate.mean.segment<2>(4*index+2)});
        for (const auto& cell :
             swarm_.robots.front()->gridWorld.getUnexploredCellCenters())
            work.allocation.cells.push_back({
                cell.x_index,cell.y_index,{cell.center.x,cell.center.y}});
        work.allocation.acceleration_half_box=
            adapter_.config().acceleration_half_box;
        work.allocation.collision_distance_m=
            adapter_.config().collision_distance_m;
        const auto rows=buildCanonicalHardRows(hard);
        for (const auto& row : rows) {
            if (row.kind!=CanonicalHardRowKind::Collision) continue;
            work.allocation.position_reserve_m=std::max(
                work.allocation.position_reserve_m,
                row.position_uncertainty_reserve_m);
            work.allocation.velocity_reserve_mps=std::max(
                work.allocation.velocity_reserve_mps,
                row.velocity_uncertainty_reserve_mps);
        }
        work.allocation.position_gain=adapter_.config().position_gain;
        work.allocation.velocity_gain=adapter_.config().velocity_gain;
        const auto fairness=fairness_.snapshot(work.allocation.cells);
        work.allocation.use_external_fairness=true;
        work.allocation.priority_epoch=fairness.priority_epoch;
        work.allocation.fairness_ages=fairness.ages;
        for (NodeId id : hard.fixed_ids) {
            const auto& state=hard.states.at(id);
            work.fixed_positions[id]={state.position.x,state.position.y};
        }
        work.dt_s=adapter_.config().dt_s;
        work.estimator_acceleration_variance=
            adapter_.config().estimator_acceleration_variance;
        work.gamma_feedback_config={
            adapter_.config().acceleration_half_box,
            adapter_.config().gamma_feedback_homotopy_segments,
            adapter_.config().gamma_feedback_selection,
            adapter_.config().predictive_gamma_tau_mps2,
            adapter_.config().gamma_feedback_tolerance};
        return work;
    }

    bool launch(AsyncFrontierWorkRequest work) {
        if (!state_.launch(work.provenance).accepted) return false;
        active_ready_tick_=work.provenance.logical_ready_tick;
        active_cells_=work.allocation.cells;
        worker_in_flight_=worker_.launch(std::move(work));
        if (!worker_in_flight_) state_.cancel("worker_launch_failed");
        return worker_in_flight_;
    }

    void pollWorkerBookkeeping() {
        if (!worker_in_flight_) return;
        const auto completed=worker_.tryCollect();
        if (!completed.has_value()) return;
        worker_in_flight_=false;
        state_.complete(completed->work.proposal,completed->wall_s);
    }

    Task10p11fAsyncBoundaryResult advanceBoundary(std::uint64_t tick) {
        Task10p11fAsyncBoundaryResult result;
        const auto runtime=adapter_.runtimeSnapshot();
        if (runtime.mode!=SupervisorMode::Search ||
            runtime.adapter_transition_pending || runtime.pending_is_retreat) {
            state_.cancel("supervisor_precedence");
            result.classification=AsyncProposalClassification::ProposalCancelled;
            result.reason="proposal_cancelled:supervisor_precedence";
            result.online_step=controller_.advanceWithoutProposal();
            return result;
        }
        pollWorkerBookkeeping();
        if (worker_in_flight_ && tick>=active_ready_tick_) {
            state_.cancel("proposal_deadline_missed");
            state_.recordFailure(
                AsyncProposalClassification::ProposalDeadlineMissed);
            result.classification=
                AsyncProposalClassification::ProposalDeadlineMissed;
            result.reason="proposal_deadline_missed";
            result.online_step=controller_.advanceWithoutProposal();
            return result;
        }
        const auto observed=state_.observe(tick,controller_.currentProvenance());
        result.classification=observed.classification;
        result.reason=observed.reason;
        if (observed.classification!=AsyncProposalClassification::Ready) {
            if (observed.classification!=AsyncProposalClassification::NotReady)
                state_.recordFailure(observed.classification);
            result.online_step=controller_.advanceWithoutProposal();
            return result;
        }
        if (!observed.result->accepted) {
            result.classification=
                AsyncProposalClassification::AllocatorSearchExhausted;
            result.reason="allocator_search_exhausted";
            state_.recordFailure(result.classification);
            result.online_step=controller_.advanceWithoutProposal();
            return result;
        }
        auto fresh=controller_.freshCommitRequest(observed.result->targets);
        fresh.proposal_request=observed.result->provenance;
        fresh.expected_bundle_id=observed.result->bundle_id;
        fresh.current_priority_bundle_id=observed.result->bundle_id;
        result.commit=controller_.commitAndApply(fresh);
        if (!result.commit.committed) {
            result.classification=
                AsyncProposalClassification::FreshCommitRejected;
            result.reason=result.commit.reason;
            state_.recordFailure(result.classification);
            result.online_step=controller_.advanceWithoutProposal();
            return result;
        }
        fairness_.recordCommit(active_cells_,observed.result->targets);
        state_.recordCommit();
        result.reason="fresh_commit_applied";
        result.online_step=result.commit.step;
        return result;
    }

    bool workerInFlight() const { return worker_in_flight_; }
    Task10p11fAtomicController& controller() { return controller_; }

private:
    Swarm& swarm_;
    GrandFinaleSwarmAdapter& adapter_;
    Task10p11fAtomicController controller_;
    AsyncProposalStateMachine state_;
    AsyncFrontierProposalWorker worker_;
    AllocatorFairnessLedger fairness_;
    bool worker_in_flight_=false;
    std::uint64_t active_ready_tick_=0;
    std::vector<FrontierCell> active_cells_;
};

}  // namespace gf
