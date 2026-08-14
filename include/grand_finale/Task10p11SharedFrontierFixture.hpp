#pragma once

#include "grand_finale/SharedFrontierAllocationPipeline.hpp"
#include "grand_finale/Task10p10CompletionFixture.hpp"
#include "grand_finale/ExactHardPolytopeDiagnostic.hpp"

namespace gf {

inline SharedFrontierAllocatorConfig task10p11AllocatorConfig() {
    return {8,32,4,5,5,0.8,0.05,1e-10};
}

struct Task10p11AdvanceResult {
    bool advanced = false;
    std::string reason;
    GrandFinaleSwarmStep step;
    SharedFrontierPipelineResult allocation;
};

class Task10p11SharedFrontierController {
public:
    Task10p11SharedFrontierController(
        Swarm& swarm,GrandFinaleSwarmAdapter& adapter,
        std::map<NodeId,Eigen::Vector2d> fixed_positions)
        : swarm_(swarm),adapter_(adapter),fixed_positions_(std::move(fixed_positions)),
          pipeline_(task10p11AllocatorConfig()) {}

    SharedFrontierPipelineRequest currentPipelineRequest() {
        const auto runtime=adapter_.runtimeSnapshot();
        SharedFrontierPipelineRequest request;
        request.profile=adapter_.config().solver_profile;
        request.dt_s=adapter_.config().dt_s;
        request.fixed_positions=fixed_positions_;
        request.hard_row_request=adapter_.currentSnapshotHardRowRequest(
            adapter_.supervisor().topology());
        request.allocation.snapshot_token=runtime.estimator_token;
        request.allocation.topology_token=runtime.topology_token;
        const GridWorld& grid=swarm_.robots.front()->gridWorld;
        request.allocation.grid_token=static_cast<std::uint64_t>(
            std::count(grid.vis.begin(),grid.vis.end(),true));
        for (std::size_t index=0;index<runtime.estimate.mobile_ids.size();++index) {
            request.allocation.agents.push_back({
                runtime.estimate.mobile_ids[index],
                runtime.estimate.mean.segment<2>(4*index),
                runtime.estimate.mean.segment<2>(4*index+2)});
        }
        for (const auto& cell : swarm_.robots.front()->gridWorld
                                      .getUnexploredCellCenters()) {
            request.allocation.cells.push_back({
                cell.x_index,cell.y_index,
                Eigen::Vector2d(cell.center.x,cell.center.y)});
        }
        request.allocation.hard_rows=buildCanonicalHardRows(
            request.hard_row_request);
        request.allocation.acceleration_half_box=
            adapter_.config().acceleration_half_box;
        request.allocation.collision_distance_m=
            adapter_.config().collision_distance_m;
        for (const auto& row : request.allocation.hard_rows) {
            if (row.kind!=CanonicalHardRowKind::Collision) continue;
            request.allocation.position_reserve_m=std::max(
                request.allocation.position_reserve_m,
                row.position_uncertainty_reserve_m);
            request.allocation.velocity_reserve_mps=std::max(
                request.allocation.velocity_reserve_mps,
                row.velocity_uncertainty_reserve_mps);
        }
        request.allocation.position_gain=adapter_.config().position_gain;
        request.allocation.velocity_gain=adapter_.config().velocity_gain;
        return request;
    }

    Task10p11AdvanceResult advance() {
        Task10p11AdvanceResult result;
        if (remaining_epoch_cycles_==0) {
            last_allocation_=pipeline_.choose(currentPipelineRequest());
            if (!last_allocation_.accepted) {
                result.reason="allocator_search_exhausted";
                result.allocation=last_allocation_;
                return result;
            }
            targets_=last_allocation_.allocation.targets;
            remaining_epoch_cycles_=task10p11AllocatorConfig().epoch_cycles;
        }
        const auto runtime=adapter_.runtimeSnapshot();
        std::map<NodeId,Eigen::Vector2d> nominal;
        for (std::size_t index=0;index<runtime.estimate.mobile_ids.size();++index) {
            const NodeId id=runtime.estimate.mobile_ids[index];
            Eigen::Vector2d control=adapter_.config().position_gain*
                (targets_.at(id).center-runtime.estimate.mean.segment<2>(4*index))-
                adapter_.config().velocity_gain*
                    runtime.estimate.mean.segment<2>(4*index+2);
            control.x()=std::clamp(control.x(),
                -adapter_.config().acceleration_half_box,
                adapter_.config().acceleration_half_box);
            control.y()=std::clamp(control.y(),
                -adapter_.config().acceleration_half_box,
                adapter_.config().acceleration_half_box);
            nominal[id]=control;
        }
        result.step=adapter_.stepWithNominal(nominal);
        result.advanced=result.step.advanced;
        result.reason=result.step.reason;
        result.allocation=last_allocation_;
        if (result.advanced) --remaining_epoch_cycles_;
        return result;
    }

private:
    Swarm& swarm_;
    GrandFinaleSwarmAdapter& adapter_;
    std::map<NodeId,Eigen::Vector2d> fixed_positions_;
    SharedFrontierAllocationPipeline pipeline_;
    SharedFrontierPipelineResult last_allocation_;
    std::map<NodeId,FrontierCell> targets_;
    std::size_t remaining_epoch_cycles_=0;
};

struct Task10p11Fixture {
    json settings;
    Swarm swarm;
    GrandFinaleSwarmAdapter adapter;
    Task10p11SharedFrontierController controller;

    Task10p11Fixture(const Task10p10Scenario& scenario,SolverProfile profile)
        : settings(task10p10SwarmSettings(scenario,profile)),swarm(settings),
          adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,
                  scenario.initial_topology,[&]() {
                      auto config=task10p10AdapterConfig(profile);
                      config.enforce_workspace_rows=true;
                      return config;
                  }()),
          controller(swarm,adapter,scenario.fixed_positions) {}
};

inline std::unique_ptr<Task10p11Fixture> makeTask10p11Fixture(
    const Task10p10Scenario& scenario,SolverProfile profile) {
    return std::make_unique<Task10p11Fixture>(scenario,profile);
}

struct Task10p11SmokeResult {
    bool completed_horizon=false;
    std::string reason;
    double runtime_s=0.0;
    double initial_coverage=0.0;
    double final_coverage=0.0;
    double minimum_robust_residual=std::numeric_limits<double>::infinity();
    std::size_t allocation_epochs=0;
    std::size_t fast_rejections=0;
    std::size_t exact_rejections=0;
    std::size_t rollout_rejections=0;
    AllocatorExhaustion final_exhaustion=AllocatorExhaustion::None;
    bool search_budget_truncated=false;
    std::size_t final_candidate_bundles=0;
    std::string last_rollout_reason;
    std::size_t failed_rollout_cycle=std::numeric_limits<std::size_t>::max();
    std::vector<HardPolytopeCertificate> rollout_conflicts;
    std::vector<FrontierRolloutResult> rejected_rollouts;
    std::string first_negative_source;
    double minimum_braking_slack_m=
        std::numeric_limits<double>::infinity();
};

inline Task10p11SmokeResult runTask10p11SharedSmoke(
    const Task10p10Scenario& scenario,SolverProfile profile,double horizon_s) {
    auto fixture=makeTask10p11Fixture(scenario,profile);
    Task10p11SmokeResult result;
    const auto initialized=fixture->adapter.initializeStageZero();
    if (!initialized.initialized) {
        result.reason=initialized.reason;
        return result;
    }
    result.initial_coverage=initialized.truth_coverage;
    const std::size_t cycles=static_cast<std::size_t>(
        std::ceil(horizon_s/fixture->adapter.config().dt_s));
    for (std::size_t cycle=0;cycle<cycles;++cycle) {
        const auto step=fixture->controller.advance();
        if (!step.advanced) {
            result.reason=step.reason;
            result.runtime_s=fixture->swarm.robots.front()->runtime;
            result.final_coverage=fixture->adapter.coverage().truthFraction();
            result.final_exhaustion=step.allocation.exhaustion;
            result.search_budget_truncated=
                step.allocation.search_budget_truncated;
            result.final_candidate_bundles=
                step.allocation.candidate_bundles;
            result.last_rollout_reason=step.allocation.rollout.reason;
            result.failed_rollout_cycle=step.allocation.rollout.failed_cycle;
            result.rejected_rollouts=step.allocation.rejected_rollouts;
            for (const auto& rejected : result.rejected_rollouts) {
                for (const auto& trace : rejected.braking_trace) {
                    result.minimum_braking_slack_m=std::min(
                        result.minimum_braking_slack_m,
                        trace.snapshot.minimum_braking_slack_m);
                    if (result.first_negative_source.empty() &&
                        !trace.snapshot.first_negative_source.empty()) {
                        result.first_negative_source=
                            trace.snapshot.first_negative_source;
                    }
                }
            }
            if (!step.allocation.rollout.final_request.mobile_ids.empty()) {
                const auto rows=buildCanonicalHardRows(
                    step.allocation.rollout.final_request);
                for (NodeId owner :
                     step.allocation.rollout.final_request.mobile_ids) {
                    auto certificate=diagnoseHardPolytope(
                        owner,result.runtime_s,
                        fixture->adapter.supervisor().mode(),
                        fixture->adapter.supervisor().topology(),rows,
                        fixture->adapter.config().acceleration_half_box);
                    if (!certificate.exact_feasible)
                        result.rollout_conflicts.push_back(
                            std::move(certificate));
                }
            }
            result.fast_rejections+=step.allocation.fast_rejections;
            result.exact_rejections+=step.allocation.exact_rejections;
            result.rollout_rejections+=step.allocation.rollout_rejections;
            return result;
        }
        result.minimum_robust_residual=std::min(
            result.minimum_robust_residual,step.step.minimum_hard_residual);
        if (cycle%task10p11AllocatorConfig().epoch_cycles==0) {
            ++result.allocation_epochs;
            result.fast_rejections+=step.allocation.fast_rejections;
            result.exact_rejections+=step.allocation.exact_rejections;
            result.rollout_rejections+=step.allocation.rollout_rejections;
        }
    }
    result.completed_horizon=true;
    result.reason="completed_horizon";
    result.runtime_s=fixture->swarm.robots.front()->runtime;
    result.final_coverage=fixture->adapter.coverage().truthFraction();
    return result;
}

}  // namespace gf
