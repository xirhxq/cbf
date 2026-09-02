#pragma once

#include "grand_finale/Task17ExperimentFixture.hpp"

namespace gf {

inline GrandFinaleSwarmAdapterConfig task18ExperimentAdapterConfig(
    bool common_governor_enabled=false,
    Task18YawObjective yaw_objective=
        Task18YawObjective::IndividualFormationTarget,
    double range_noise_std_m=0.0,
    double range_dropout_probability=0.0,
    unsigned int range_random_seed=2027,
    bool velocity_augmented_rows=true,
    GammaFeedbackSelectionMode gamma_selection=
        GammaFeedbackSelectionMode::LeastIntervention,
    bool collision_only_vaug=false) {
    auto config=task17ExperimentAdapterConfig(
        Task17PeriodicArm::Voronoi,range_noise_std_m,
        range_dropout_probability,range_random_seed,
        common_governor_enabled,5);
    config.target_policy_task17_periodic=false;
    config.target_policy_task18_cbf2026_outer=true;
    config.task18_update_period_cycles=5;
    config.task18_common_governor_enabled=common_governor_enabled;
    config.task18_yaw_objective=yaw_objective;
    config.velocity_augmented_rows=velocity_augmented_rows;
    config.gamma_feedback_selection=gamma_selection;
    config.task18_collision_only_vaug=collision_only_vaug;
    config.predictive_gamma_tau_mps2=
        gamma_selection==GammaFeedbackSelectionMode::LeastIntervention
            ?std::optional<double>{22.0}:std::nullopt;
    return config;
}

struct Task18ExperimentFixture {
    Task10p10Scenario scenario;
    json settings;
    Swarm swarm;
    GrandFinaleSwarmAdapter adapter;
    Task10p11hSimpleCoverageController controller;

    Task18ExperimentFixture(bool common_governor_enabled,
        Task18YawObjective yaw_objective,double noise_std_m,
        double dropout_probability,unsigned int seed,
        bool collision_only_vaug=false)
        : scenario(task10p11rFixedBaselineScenario()),
          settings(task10p11pSwarmSettings(scenario,SolverProfile::Gurobi)),
          swarm(settings),
          adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,
              scenario.initial_topology,task18ExperimentAdapterConfig(
                  common_governor_enabled,yaw_objective,noise_std_m,
                  dropout_probability,seed,true,
                  GammaFeedbackSelectionMode::LeastIntervention,
                  collision_only_vaug)),
          controller(swarm,adapter,{}, {},
              task10p11rAuthorityContract().branches) {}
};

inline std::unique_ptr<Task18ExperimentFixture> makeTask18ExperimentFixture(
    bool common_governor_enabled=false,
    Task18YawObjective yaw_objective=
        Task18YawObjective::IndividualFormationTarget,
    double noise_std_m=0.0,double dropout_probability=0.0,
    unsigned int seed=2027,bool collision_only_vaug=false) {
    return std::make_unique<Task18ExperimentFixture>(
        common_governor_enabled,yaw_objective,noise_std_m,
        dropout_probability,seed,collision_only_vaug);
}

}  // namespace gf
