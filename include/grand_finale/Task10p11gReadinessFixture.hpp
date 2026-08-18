#pragma once

#include "grand_finale/Task10p11SharedFrontierFixture.hpp"
#include "grand_finale/Task10p11gFrozenModel.hpp"
#include "grand_finale/Task10p11fAsyncController.hpp"

namespace gf {

inline SharedFrontierAllocatorConfig task10p11gAllocatorConfig() {
    // The accepted level-A rollout predicate remains a five-cycle snapshot
    // heuristic.  The separate offline-exact coordinator owns the 1.0 s
    // logical planner tick; extending this rollout would change its frozen
    // scientific budget.
    return task10p11AllocatorConfig();
}

inline json task10p11gSwarmSettings(
    const Task10p10Scenario& scenario,SolverProfile profile) {
    json settings=task10p10SwarmSettings(scenario,profile);
    settings["initial"]["yawDeg"]=90.0;
    return settings;
}

inline GrandFinaleSwarmAdapterConfig task10p11gAdapterConfig(
    SolverProfile profile,
    std::optional<BoundaryPolicyConfig> boundary_override=std::nullopt,
    std::optional<double> historical_collision_override_m=std::nullopt) {
    const auto model=task10p11gFrozenModel();
    auto config=task10p10AdapterConfig(profile);
    config.acceleration_half_box=model.acceleration_half_box_mps2;
    config.collision_distance_m=model.collision_distance_m;
    if (historical_collision_override_m.has_value())
        config.collision_distance_m=*historical_collision_override_m;
    config.speed_limit_mps=model.speed_limit_mps;
    config.plant_speed_facet_count=64;
    config.maximum_yaw_rate_radps=model.maximum_yaw_rate_radps;
    config.boundary.policy=BoundaryPolicy::HardFlightBoundary;
    config.boundary.flight_polygon_source=FlightPolygonSource::SearchPolygon;
    if (boundary_override.has_value()) config.boundary=*boundary_override;
    config.gamma_feedback_selection=model.gamma_selection;
    config.predictive_gamma_tau_mps2=model.predictive_tau_mps2;
    config.position_gain=model.position_gain;
    config.velocity_gain=model.velocity_gain;
    return config;
}

inline Task10p10Scenario task10p11gBindingMechanismScenario() {
    Task10p10Scenario result;
    result.id="binding_mechanism";
    result.width_m=1800.0;
    result.height_m=500.0;
    result.mobile_ids=task10p10MobileIds(14);
    const std::vector<double> xs{790.0,798.0,806.0,814.0};
    const std::vector<double> ys{220.0,228.0,236.0,244.0};
    for (double y : ys) for (double x : xs) {
        if (result.mobile_positions.size()==14) break;
        result.mobile_positions.push_back({x,y});
    }
    result.fixed_positions={
        {100,{0.0,250.0}},{101,{800.0,450.0}},{102,{1600.0,250.0}}};
    for (NodeId owner : result.mobile_ids) {
        result.initial_topology.push_back({100,owner});
        result.initial_topology.push_back({101,owner});
    }
    return result;
}

struct Task10p11gCoverageBudget {
    CoverageScaleBudget cells;
    double farthest_nearest_initial_m=0.0;
    RestToRestTime motion_lower_bound;
    double timeout_factor=0.0;
};

inline Task10p11gCoverageBudget evaluateTask10p11gCoverageBudget(
    const Task10p10Scenario& scenario,double timeout_s=240.0) {
    const auto model=task10p11gFrozenModel();
    const CoverageGridSpec grid{scenario.width_m,scenario.height_m,10.0};
    const SectorFootprintSpec footprint{0.0,400.0,M_PI/3.0};
    std::vector<SectorFootprintStage> initial;
    for (const auto& position : scenario.mobile_positions)
        initial.push_back({position,model.initial_yaw_rad});
    std::vector<SectorFootprintStage> optimistic;
    double farthest=0.0;
    const int nx=static_cast<int>(std::llround(scenario.width_m/10.0));
    const int ny=static_cast<int>(std::llround(scenario.height_m/10.0));
    for (int ix=0;ix<nx;++ix) for (int iy=0;iy<ny;++iy) {
        const Eigen::Vector2d cell((ix+0.5)*10.0,(iy+0.5)*10.0);
        double nearest=std::numeric_limits<double>::infinity();
        for (const auto& position : scenario.mobile_positions)
            nearest=std::min(nearest,(cell-position).norm());
        farthest=std::max(farthest,nearest);
        optimistic.push_back({cell,0.0});
    }
    Task10p11gCoverageBudget result;
    result.cells=evaluateCoverageScaleBudget(
        grid,footprint,initial,optimistic,0.95);
    result.farthest_nearest_initial_m=farthest;
    result.motion_lower_bound=minimumRestToRestTime(
        farthest,model.acceleration_half_box_mps2,model.speed_limit_mps);
    result.timeout_factor=timeout_s/result.motion_lower_bound.seconds;
    return result;
}

struct Task10p11gFixture {
    json settings;
    Swarm swarm;
    GrandFinaleSwarmAdapter adapter;
    Task10p11SharedFrontierController controller;

    Task10p11gFixture(
        const Task10p10Scenario& scenario,SolverProfile profile,
        std::optional<BoundaryPolicyConfig> boundary_override=std::nullopt,
        std::optional<double> historical_collision_override_m=std::nullopt)
        : settings(task10p11gSwarmSettings(scenario,profile)),swarm(settings),
          adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,
                  scenario.initial_topology,
                  task10p11gAdapterConfig(profile,boundary_override,
                      historical_collision_override_m)),
          controller(swarm,adapter,scenario.fixed_positions,
                     task10p11gAllocatorConfig()) {}
};

struct Task10p11gOfflineExactStep {
    bool proposal_due=false;
    AsyncFrontierWorkResult proposal;
    Task10p11fCommitApplyResult commit;
    GrandFinaleSwarmStep step;
    std::string reason;
};

class Task10p11gOfflineExactCoordinator {
public:
    Task10p11gOfflineExactCoordinator(
        Swarm& swarm,GrandFinaleSwarmAdapter& adapter)
        : async_(swarm,adapter,{10,10,3}),controller_(async_.controller()) {}

    Task10p11gOfflineExactStep advance(std::uint64_t control_tick) {
        Task10p11gOfflineExactStep result;
        if (control_tick%10!=0) {
            result.step=controller_.advanceWithoutProposal();
            result.reason=result.step.reason;
            return result;
        }
        result.proposal_due=true;
        const auto work=async_.makeWorkRequest();
        result.proposal=runAsyncFrontierProposal(work);
        if (!result.proposal.proposal.accepted) {
            result.reason="allocator_search_exhausted";
            return result;
        }
        auto fresh=controller_.freshCommitRequest(
            result.proposal.proposal.targets);
        fresh.proposal_request=result.proposal.proposal.provenance;
        fresh.expected_bundle_id=result.proposal.proposal.bundle_id;
        fresh.current_priority_bundle_id=result.proposal.proposal.bundle_id;
        result.commit=controller_.commitAndApply(fresh);
        result.step=result.commit.step;
        result.reason=result.commit.reason;
        return result;
    }

private:
    Task10p11fAsyncCoordinator async_;
    Task10p11fAtomicController& controller_;
};

inline std::unique_ptr<Task10p11gFixture> makeTask10p11gFixture(
    const Task10p10Scenario& scenario,SolverProfile profile,
    std::optional<BoundaryPolicyConfig> boundary_override=std::nullopt,
    std::optional<double> historical_collision_override_m=std::nullopt) {
    return std::make_unique<Task10p11gFixture>(
        scenario,profile,boundary_override,historical_collision_override_m);
}

}  // namespace gf
