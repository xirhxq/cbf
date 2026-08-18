#pragma once

#include "grand_finale/Task10p11gReadinessFixture.hpp"
#include "grand_finale/Task10p11hCoastalScenarios.hpp"
#include "grand_finale/Task10p11hLeaderCoveragePolicy.hpp"

#include <limits>

namespace gf {

inline const char* task10p11pCbf2026SourceCommit() {
    return "47e7b3d6f5b1be74a3dfa32150b4246c619e9a0d";
}

inline std::map<NodeId,Eigen::Vector2d> task10p11pStandardCoastalAnchors() {
    return {{100,{1200.0,-50.0}},
            {101,{1500.0,-50.0}},
            {102,{1800.0,-50.0}}};
}

inline std::vector<Eigen::Vector2d> task10p11pStandardLaunchPositions() {
    // Rigid coastal transform of the fourteen deterministic positions listed
    // in CBF2026@47e7b3d config/config.json:
    // x_GF=y_CBF+1500, y_GF=x_CBF+1500.
    return {{1380.0,10.0},{1370.0,30.0},{1360.0,50.0},
            {1350.0,70.0},{1340.0,90.0},{1330.0,110.0},
            {1320.0,130.0},{1620.0,10.0},{1630.0,30.0},
            {1640.0,50.0},{1650.0,70.0},{1660.0,90.0},
            {1670.0,110.0},{1680.0,130.0}};
}

inline std::vector<DirectedEdge> task10p11pStandardInitialTopology() {
    std::vector<DirectedEdge> result;
    for (NodeId owner=1;owner<=7;++owner) {
        result.emplace_back(101,owner);
        result.emplace_back(100,owner);
    }
    for (NodeId owner=8;owner<=14;++owner) {
        result.emplace_back(101,owner);
        result.emplace_back(102,owner);
    }
    return result;
}

inline Task10p10Scenario task10p11pStandardCoastalScenario() {
    return {"standard_coastal_3000",3000.0,3000.0,
        task10p10MobileIds(14),task10p11pStandardLaunchPositions(),
        task10p11pStandardCoastalAnchors(),
        task10p11pStandardInitialTopology()};
}

inline std::vector<LeaderCoverageBranchSpec>
task10p11pStandardCoverageBranches() {
    return {{{1,2,3,4,5,6,7},7,{1500.0,-50.0},-M_PI/3.0,{101,100}},
            {{8,9,10,11,12,13,14},14,{1500.0,-50.0},M_PI/3.0,{101,102}}};
}

inline json task10p11pSwarmSettings(
    const Task10p10Scenario& scenario,SolverProfile profile) {
    json result=task10p11gSwarmSettings(scenario,profile);
    result["execute"]["time-total"]=500.0;
    result["execute"]["time-step"]=0.1;
    return result;
}

inline GrandFinaleSwarmAdapterConfig task10p11pAdapterConfig(
    SolverProfile profile,double operational_speed_mps,
    GammaFeedbackSelectionMode selection,
    std::optional<double> predictive_tau_mps2) {
    BoundaryPolicyConfig boundary;
    boundary.policy=BoundaryPolicy::None;
    auto result=task10p11gAdapterConfig(profile,boundary);
    result.speed_limit_mps=operational_speed_mps;
    result.plant_speed_facet_count=64;
    result.gamma_feedback_selection=selection;
    result.predictive_gamma_tau_mps2=predictive_tau_mps2;
    return result;
}

struct Task10p11pAnalyticStageZero {
    std::size_t valid_cell_count=0;
    double minimum_mobile_mobile_distance_m=
        std::numeric_limits<double>::infinity();
    double minimum_mobile_fixed_distance_m=
        std::numeric_limits<double>::infinity();
    double maximum_initial_reference_distance_m=0.0;
    double initial_truth_fraction=0.0;
};

inline Task10p11pAnalyticStageZero task10p11pAnalyticStageZeroAudit() {
    const auto scenario=task10p11pStandardCoastalScenario();
    Task10p11pAnalyticStageZero result;
    const auto budget=evaluateCoverageScaleBudget(
        {3000.0,3000.0,10.0},{0.0,400.0,M_PI/3.0},
        [&]() {
            std::vector<SectorFootprintStage> stages;
            for (const auto& position:scenario.mobile_positions)
                stages.push_back({position,M_PI/2.0});
            return stages;
        }(),{},0.95);
    result.valid_cell_count=budget.valid_cell_count;
    result.initial_truth_fraction=budget.initial_fraction;
    for (std::size_t first=0;first<scenario.mobile_positions.size();++first) {
        for (std::size_t second=first+1;
             second<scenario.mobile_positions.size();++second) {
            result.minimum_mobile_mobile_distance_m=std::min(
                result.minimum_mobile_mobile_distance_m,
                (scenario.mobile_positions[first]-
                 scenario.mobile_positions[second]).norm());
        }
        for (const auto& [id,fixed]:scenario.fixed_positions) {
            (void)id;
            result.minimum_mobile_fixed_distance_m=std::min(
                result.minimum_mobile_fixed_distance_m,
                (scenario.mobile_positions[first]-fixed).norm());
        }
    }
    std::map<NodeId,Eigen::Vector2d> mobile;
    for (std::size_t index=0;index<scenario.mobile_ids.size();++index)
        mobile[scenario.mobile_ids[index]]=scenario.mobile_positions[index];
    for (const auto& edge:scenario.initial_topology)
        result.maximum_initial_reference_distance_m=std::max(
            result.maximum_initial_reference_distance_m,
            (mobile.at(edge.owner)-scenario.fixed_positions.at(edge.reference)).norm());
    return result;
}

struct Task10p11pNominalControllerStep {
    LeaderCoverageResult allocation;
    GrandFinaleNominalOnlyDiagnosticStep diagnostic;
    std::map<NodeId,FrontierCell> targets;
    std::size_t target_epoch=0;
};

class Task10p11pNominalController {
public:
    Task10p11pNominalController(
        Swarm& swarm,GrandFinaleSwarmAdapter& adapter,
        std::vector<LeaderCoverageBranchSpec> branches=
            task10p11pStandardCoverageBranches())
        : swarm_(swarm),adapter_(adapter),branches_(std::move(branches)) {
        GridWorld& grid=swarm_.robots.front()->gridWorld;
        for (int x=0;x<grid.xNum;++x) for (int y=0;y<grid.yNum;++y)
            domain_.push_back({x,y,{
                grid.getCellCenterX(x),grid.getCellCenterY(y)}});
    }

    Task10p11pNominalControllerStep advance() {
        Task10p11pNominalControllerStep result;
        const auto runtime=adapter_.runtimeSnapshot();
        if (targets_.empty() || control_tick_%10==0) {
            LeaderCoverageRequest request;
            request.branches=branches_;
            request.domain_cells=domain_;
            for (const auto& cell:
                 swarm_.robots.front()->gridWorld.getUnexploredCellCenters())
                request.uncovered_cells.push_back(
                    {cell.x_index,cell.y_index,{cell.center.x,cell.center.y}});
            for (std::size_t index=0;
                 index<runtime.estimate.mobile_ids.size();++index)
                request.agents.push_back({runtime.estimate.mobile_ids[index],
                    runtime.estimate.mean.segment<2>(4*index),
                    runtime.estimate.mean.segment<2>(4*index+2)});
            result.allocation=allocateLeaderCoverageTargets(std::move(request));
            if (!result.allocation.valid) {
                result.diagnostic.reason=result.allocation.reason;
                return result;
            }
            targets_=result.allocation.targets;
            ++target_epoch_;
        }
        std::map<NodeId,Eigen::Vector2d> nominal;
        std::map<NodeId,double> yaw_rates;
        auto model=task10p11gFrozenModel();
        model.acceleration_half_box_mps2=adapter_.config().acceleration_half_box;
        model.position_gain=adapter_.config().position_gain;
        model.velocity_gain=adapter_.config().velocity_gain;
        model.maximum_yaw_rate_radps=adapter_.config().maximum_yaw_rate_radps;
        for (std::size_t index=0;
             index<runtime.estimate.mobile_ids.size();++index) {
            const NodeId owner=runtime.estimate.mobile_ids[index];
            const Eigen::Vector4d state=runtime.estimate.mean.segment<4>(4*index);
            const auto soft=task10p11gSoftTask({
                state.head<2>(),state.tail<2>(),currentYaw(owner),
                targets_.at(owner).center,runtime.mode},model);
            nominal[owner]=soft.acceleration;
            yaw_rates[owner]=soft.yaw_rate_radps;
        }
        result.diagnostic=adapter_.stepNominalOnlyCausalDiagnostic(
            nominal,yaw_rates);
        result.targets=targets_;
        result.target_epoch=target_epoch_;
        ++control_tick_;
        return result;
    }

    std::size_t targetEpoch() const { return target_epoch_; }

private:
    double currentYaw(NodeId owner) const {
        for (const auto& robot:swarm_.robots)
            if (robot->id==owner)
                return robot->model->getStateVariable("yawRad");
        throw std::invalid_argument("missing nominal diagnostic yaw state");
    }

    Swarm& swarm_;
    GrandFinaleSwarmAdapter& adapter_;
    std::vector<LeaderCoverageBranchSpec> branches_;
    std::vector<FrontierCell> domain_;
    std::map<NodeId,FrontierCell> targets_;
    std::size_t target_epoch_=0;
    std::size_t control_tick_=0;
};

struct Task10p11pNominalFixture {
    Task10p10Scenario scenario;
    json settings;
    Swarm swarm;
    GrandFinaleSwarmAdapter adapter;
    Task10p11pNominalController controller;

    Task10p11pNominalFixture(SolverProfile profile,double speed_limit_mps)
        : scenario(task10p11pStandardCoastalScenario()),
          settings(task10p11pSwarmSettings(scenario,profile)),swarm(settings),
          adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,
              scenario.initial_topology,task10p11pAdapterConfig(
                  profile,speed_limit_mps,
                  GammaFeedbackSelectionMode::LeastIntervention,0.0)),
          controller(swarm,adapter,task10p11pStandardCoverageBranches()) {}
};

inline std::unique_ptr<Task10p11pNominalFixture>
makeTask10p11pNominalFixture(SolverProfile profile,double speed_limit_mps) {
    return std::make_unique<Task10p11pNominalFixture>(profile,speed_limit_mps);
}

struct Task10p11pNominalObservation {
    double simulated_time_s=0.0;
    double truth_coverage=0.0;
    double minimum_mobile_mobile_distance_m=
        std::numeric_limits<double>::infinity();
    double minimum_mobile_fixed_distance_m=
        std::numeric_limits<double>::infinity();
    double maximum_speed_mps=0.0;
    double minimum_current_gamma_mps2=
        std::numeric_limits<double>::infinity();
    double minimum_local_predicted_gamma_mps2=
        std::numeric_limits<double>::infinity();
    double minimum_full_hard_residual_mps2=
        std::numeric_limits<double>::infinity();
    std::size_t covered_cells=0;
    double dt_s=0.0;
    double minimum_plant_speed_applied_control_residual_mps2=
        std::numeric_limits<double>::infinity();
    double maximum_estimated_speed_mps=0.0;
    double maximum_tube_robust_speed_mps=0.0;
};

struct Task10p11pNominalMetrics {
    std::optional<double> t95_s;
    std::optional<double> t100_s;
    double minimum_truth_mobile_distance_m=
        std::numeric_limits<double>::infinity();
    double minimum_truth_mobile_fixed_distance_m=
        std::numeric_limits<double>::infinity();
    double maximum_speed_mps=0.0;
    double minimum_current_gamma_mps2=
        std::numeric_limits<double>::infinity();
    double minimum_local_predicted_gamma_mps2=
        std::numeric_limits<double>::infinity();
    double minimum_full_hard_residual_mps2=
        std::numeric_limits<double>::infinity();
    double minimum_plant_speed_applied_control_residual_mps2=
        std::numeric_limits<double>::infinity();
    double maximum_estimated_speed_mps=0.0;
    double maximum_tube_robust_speed_mps=0.0;
    double deepest_collision_intrusion_m=0.0;
    std::size_t collision_violation_pair_ticks=0;
    std::size_t speed_violation_ticks=0;
    double any_collision_violation_duration_s=0.0;
    double first_current_gamma_negative_s=
        std::numeric_limits<double>::quiet_NaN();
    double first_local_predicted_gamma_negative_s=
        std::numeric_limits<double>::quiet_NaN();
    std::size_t local_predicted_gamma_invalid_ticks=0;
    std::size_t maximum_covered_cells=0;

    void observe(const Task10p11pNominalObservation& value) {
        if (!std::isfinite(value.simulated_time_s) ||
            !std::isfinite(value.truth_coverage) ||
            !std::isfinite(value.minimum_mobile_mobile_distance_m) ||
            !std::isfinite(value.minimum_mobile_fixed_distance_m) ||
            !std::isfinite(value.maximum_speed_mps) ||
            !std::isfinite(value.minimum_current_gamma_mps2) ||
            !std::isfinite(value.minimum_full_hard_residual_mps2) ||
            !std::isfinite(
                value.minimum_plant_speed_applied_control_residual_mps2) ||
            !std::isfinite(value.maximum_estimated_speed_mps) ||
            !std::isfinite(value.maximum_tube_robust_speed_mps) ||
            !std::isfinite(value.dt_s) || value.dt_s<0.0)
            throw std::invalid_argument("invalid nominal metric observation");
        if (!t95_s.has_value() && value.truth_coverage>=0.95-1e-12)
            t95_s=value.simulated_time_s;
        if (!t100_s.has_value() && value.truth_coverage>=1.0-1e-12)
            t100_s=value.simulated_time_s;
        minimum_truth_mobile_distance_m=std::min(
            minimum_truth_mobile_distance_m,
            value.minimum_mobile_mobile_distance_m);
        minimum_truth_mobile_fixed_distance_m=std::min(
            minimum_truth_mobile_fixed_distance_m,
            value.minimum_mobile_fixed_distance_m);
        maximum_speed_mps=std::max(maximum_speed_mps,value.maximum_speed_mps);
        minimum_current_gamma_mps2=std::min(
            minimum_current_gamma_mps2,value.minimum_current_gamma_mps2);
        if (std::isfinite(value.minimum_local_predicted_gamma_mps2)) {
            minimum_local_predicted_gamma_mps2=std::min(
                minimum_local_predicted_gamma_mps2,
                value.minimum_local_predicted_gamma_mps2);
        } else {
            ++local_predicted_gamma_invalid_ticks;
        }
        minimum_full_hard_residual_mps2=std::min(
            minimum_full_hard_residual_mps2,
            value.minimum_full_hard_residual_mps2);
        minimum_plant_speed_applied_control_residual_mps2=std::min(
            minimum_plant_speed_applied_control_residual_mps2,
            value.minimum_plant_speed_applied_control_residual_mps2);
        maximum_estimated_speed_mps=std::max(
            maximum_estimated_speed_mps,value.maximum_estimated_speed_mps);
        maximum_tube_robust_speed_mps=std::max(
            maximum_tube_robust_speed_mps,
            value.maximum_tube_robust_speed_mps);
        const double minimum_distance=std::min(
            value.minimum_mobile_mobile_distance_m,
            value.minimum_mobile_fixed_distance_m);
        if (minimum_distance<10.0) {
            ++collision_violation_pair_ticks;
            any_collision_violation_duration_s+=value.dt_s;
            deepest_collision_intrusion_m=std::max(
                deepest_collision_intrusion_m,10.0-minimum_distance);
        }
        if (value.maximum_speed_mps>30.0+1e-9) ++speed_violation_ticks;
        if (!std::isfinite(first_current_gamma_negative_s) &&
            value.minimum_current_gamma_mps2<0.0)
            first_current_gamma_negative_s=value.simulated_time_s;
        if (std::isfinite(value.minimum_local_predicted_gamma_mps2) &&
            !std::isfinite(first_local_predicted_gamma_negative_s) &&
            value.minimum_local_predicted_gamma_mps2<0.0)
            first_local_predicted_gamma_negative_s=value.simulated_time_s;
        maximum_covered_cells=std::max(
            maximum_covered_cells,value.covered_cells);
    }
};

}  // namespace gf
