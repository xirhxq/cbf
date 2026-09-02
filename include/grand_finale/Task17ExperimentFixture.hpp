#pragma once

#include "grand_finale/Task10p11rFixedBaseline.hpp"

namespace gf {

inline GrandFinaleSwarmAdapterConfig task17ExperimentAdapterConfig(
    Task17PeriodicArm arm,double range_noise_std_m=0.0,
    double range_dropout_probability=0.0,
    unsigned int range_random_seed=2027,
    bool common_governor_enabled=true,
    std::size_t update_period_cycles=5,
    bool reference_compatible_formation=false,
    bool member_aware_wide_formation=false,
    bool coherent_service_wide_formation=false) {
    auto config=task10p11rFixedAdapterConfig(
        GammaFeedbackSelectionMode::LeastIntervention,22.0);
    config.speed_row_nominal=true;
    config.speed_row_nominal_limit_mps=29.9;
    config.plant_speed_facet_count=0;
    config.nominal_speed_saturation_mps=0.0;
    config.speed_preflight_demoted=true;
    config.speed_initial_set_truth_gate=true;
    config.speed_cbf_gain=7.0;
    config.velocity_augmented_rows=true;
    config.row_slack_epsilon_m=0.5;
    config.acceleration_half_box=4.0;
    config.boundary.policy=BoundaryPolicy::None;
    config.target_policy_v2=false;
    config.target_policy_v3=false;
    config.target_policy_v6=false;
    config.target_policy_unified_h2=false;
    config.target_policy_task15_forward=false;
    config.target_policy_task16_cbf2026=false;
    config.target_homotopy_enabled=false;
    config.target_policy_task17_periodic=true;
    config.task17_periodic_arm=arm;
    config.task17_update_period_cycles=update_period_cycles;
    config.task17_common_governor_enabled=common_governor_enabled;
    config.task17_reference_compatible_formation=
        reference_compatible_formation;
    config.task17_member_aware_wide_formation=
        member_aware_wide_formation;
    config.task17_coherent_service_wide_formation=
        coherent_service_wide_formation;
    config.range_noise_std_m=range_noise_std_m;
    config.range_dropout_probability=range_dropout_probability;
    config.range_random_seed=range_random_seed;
    return config;
}

struct Task17ExperimentFixture {
    Task10p10Scenario scenario;
    json settings;
    Swarm swarm;
    GrandFinaleSwarmAdapter adapter;
    Task10p11hSimpleCoverageController controller;

    Task17ExperimentFixture(Task17PeriodicArm arm,double noise_std_m,
        double dropout_probability,unsigned int seed,
        bool common_governor_enabled,std::size_t update_period_cycles,
        bool reference_compatible_formation,
        bool member_aware_wide_formation,
        bool coherent_service_wide_formation)
        : scenario(task10p11rFixedBaselineScenario()),
          settings(task10p11pSwarmSettings(scenario,SolverProfile::Gurobi)),
          swarm(settings),
          adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,
              scenario.initial_topology,task17ExperimentAdapterConfig(
                  arm,noise_std_m,dropout_probability,seed,
                  common_governor_enabled,update_period_cycles,
                  reference_compatible_formation,
                  member_aware_wide_formation,
                  coherent_service_wide_formation)),
          controller(swarm,adapter,{}, {},
              task10p11rAuthorityContract().branches) {}
};

inline std::unique_ptr<Task17ExperimentFixture> makeTask17ExperimentFixture(
    Task17PeriodicArm arm,double noise_std_m=0.0,
    double dropout_probability=0.0,unsigned int seed=2027,
    bool common_governor_enabled=true,
    std::size_t update_period_cycles=5,
    bool reference_compatible_formation=false,
    bool member_aware_wide_formation=false,
    bool coherent_service_wide_formation=false) {
    return std::make_unique<Task17ExperimentFixture>(arm,noise_std_m,
        dropout_probability,seed,common_governor_enabled,
        update_period_cycles,reference_compatible_formation,
        member_aware_wide_formation,coherent_service_wide_formation);
}

}  // namespace gf
