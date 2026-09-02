#pragma once

#include "grand_finale/Task10p11rFixedBaseline.hpp"

namespace gf {

inline GrandFinaleSwarmAdapterConfig task16ExperimentAdapterConfig(
    Task16CoverageArm arm,double range_noise_std_m=0.0,
    double range_dropout_probability=0.0,
    unsigned int range_random_seed=2027) {
    auto config=task10p11rFixedAdapterConfig(
        GammaFeedbackSelectionMode::LeastIntervention,22.0);
    config.speed_row_nominal=true;
    config.speed_row_nominal_limit_mps=30.0;
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
    config.target_homotopy_enabled=false;
    config.target_policy_task16_cbf2026=true;
    config.task16_coverage_arm=arm;
    config.task16_cvt_update_period_cycles=5;
    config.range_noise_std_m=range_noise_std_m;
    config.range_dropout_probability=range_dropout_probability;
    config.range_random_seed=range_random_seed;
    return config;
}

struct Task16ExperimentFixture {
    Task10p10Scenario scenario;
    json settings;
    Swarm swarm;
    GrandFinaleSwarmAdapter adapter;
    Task10p11hSimpleCoverageController controller;

    Task16ExperimentFixture(Task16CoverageArm arm,double noise_std_m,
        double dropout_probability,unsigned int seed)
        : scenario(task10p11rFixedBaselineScenario()),
          settings(task10p11pSwarmSettings(scenario,SolverProfile::Gurobi)),
          swarm(settings),
          adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,
              scenario.initial_topology,task16ExperimentAdapterConfig(
                  arm,noise_std_m,dropout_probability,seed)),
          controller(swarm,adapter,{}, {},
              task10p11rAuthorityContract().branches) {}
};

inline std::unique_ptr<Task16ExperimentFixture> makeTask16ExperimentFixture(
    Task16CoverageArm arm,double noise_std_m=0.0,
    double dropout_probability=0.0,unsigned int seed=2027) {
    return std::make_unique<Task16ExperimentFixture>(
        arm,noise_std_m,dropout_probability,seed);
}

}  // namespace gf
