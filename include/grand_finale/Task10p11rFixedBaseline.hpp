#pragma once

#include "grand_finale/Task10p11pOperationalEnvelope.hpp"
#include "grand_finale/Task10p11hSimpleCoverageController.hpp"

namespace gf {

struct Task10p11rAuthorityContract {
    std::string source_commit;
    std::vector<NodeId> leaders;
    std::vector<LeaderCoverageBranchSpec> branches;
    double initial_yaw_rad=0.0;
    double authority_control_period_s=0.0;
    double grand_finale_control_period_s=0.0;
};

inline Task10p11rAuthorityContract task10p11rAuthorityContract() {
    return {task10p11pCbf2026SourceCommit(),{7,14},
        task10p11pStandardCoverageBranches(),M_PI/2.0,0.5,0.1};
}

inline std::vector<DirectedEdge> task10p11rFixedReferenceTopology() {
    return {{101,1},{100,1},{101,2},{1,2},{1,3},{2,3},{2,4},{3,4},
            {3,5},{4,5},{4,6},{5,6},{5,7},{6,7},
            {101,8},{102,8},{101,9},{8,9},{8,10},{9,10},
            {9,11},{10,11},{10,12},{11,12},{11,13},{12,13},
            {12,14},{13,14}};
}

inline Task10p10Scenario task10p11rFixedBaselineScenario() {
    auto scenario=task10p11pStandardCoastalScenario();
    scenario.id="standard_coastal_fixed_cbf2026_integration";
    scenario.initial_topology=task10p11rFixedReferenceTopology();
    return scenario;
}

inline GrandFinaleSwarmAdapterConfig task10p11rFixedAdapterConfig(
    GammaFeedbackSelectionMode selection=
        GammaFeedbackSelectionMode::LeastIntervention,
    std::optional<double> predictive_tau_mps2=14.0) {
    return task10p11pAdapterConfig(SolverProfile::Gurobi,30.0,
        selection,predictive_tau_mps2);
}

struct Task10p11rFixedMetricSnapshot {
    double accepted_information_nominal_fim=
        std::numeric_limits<double>::infinity();
    double accepted_information_robust_fim=
        std::numeric_limits<double>::infinity();
    double reference_topology_fim_proxy=
        std::numeric_limits<double>::infinity();
    double reference_topology_robust_fim_proxy=
        std::numeric_limits<double>::infinity();
    double maximum_posterior_eigenvalue_m2=0.0;
    double minimum_aoi_margin_s=std::numeric_limits<double>::infinity();
    std::size_t minimum_effective_reference_count=0;
    std::size_t minimum_information_edge_count=0;
    double minimum_collision_h=std::numeric_limits<double>::infinity();
    double minimum_collision_psi1=std::numeric_limits<double>::infinity();
    double minimum_reference_h=std::numeric_limits<double>::infinity();
    double minimum_reference_psi1=std::numeric_limits<double>::infinity();
};

inline Task10p11rFixedMetricSnapshot task10p11rFixedMetricSnapshot(
    const CurrentReferenceAudit& information,
    const std::vector<CanonicalHardRow>& rows) {
    Task10p11rFixedMetricSnapshot result;
    result.accepted_information_nominal_fim=information.minimum_fim_eigenvalue;
    result.accepted_information_robust_fim=
        information.minimum_robust_fim_cone_lower_bound;
    result.reference_topology_fim_proxy=
        information.minimum_reference_only_fim_eigenvalue;
    result.reference_topology_robust_fim_proxy=
        information.minimum_reference_only_robust_fim_cone_lower_bound;
    result.maximum_posterior_eigenvalue_m2=
        information.maximum_posterior_eigenvalue;
    result.minimum_aoi_margin_s=information.minimum_range_aoi_margin_s;
    result.minimum_effective_reference_count=
        information.minimum_effective_reference_count;
    result.minimum_information_edge_count=
        information.minimum_information_edge_count;
    for (const auto& row:rows) {
        if (row.kind==CanonicalHardRowKind::Collision) {
            result.minimum_collision_h=std::min(
                result.minimum_collision_h,row.barrier_h);
            result.minimum_collision_psi1=std::min(
                result.minimum_collision_psi1,row.barrier_psi1);
        } else if (row.kind==CanonicalHardRowKind::ReferenceDistance) {
            result.minimum_reference_h=std::min(
                result.minimum_reference_h,row.barrier_h);
            result.minimum_reference_psi1=std::min(
                result.minimum_reference_psi1,row.barrier_psi1);
        }
    }
    return result;
}

inline GrandFinaleSwarmAdapterConfig task10p11rFixtureAdapterConfig(
    GammaFeedbackSelectionMode selection,
    std::optional<double> predictive_tau_mps2,
    bool speed_row_nominal,bool tau_margin_gate=false,
    bool tau_family_predict=false,bool tau_analytic_first_order=false,
    bool s1_v3_preflight_demoted=false,bool nominal_throttle=false,
    bool throttle_v2=false,bool speed_rows_removed=false,
    bool s1_rung_b=false,bool s1_v4_prime=false,
    bool s1_v4_bare=false,bool s1_rung_b_prime=false,
    bool s1_rung_b2=false,bool target_policy_v2=false,
    bool velocity_augmented_rows=false,bool target_policy_v3=false,
    bool target_policy_v6=false,bool leader_reachability_filter=false,
    bool target_policy_unified_h2=false,
    double unified_h2_service_standoff_m=350.0) {
    auto config=task10p11rFixedAdapterConfig(selection,predictive_tau_mps2);
    config.speed_row_nominal=speed_row_nominal;
    config.tau_margin_gate_enabled=tau_margin_gate;
    config.tau_family_predict=tau_family_predict;
    config.tau_analytic_first_order=tau_analytic_first_order;
    config.speed_preflight_demoted=s1_v3_preflight_demoted;
    config.nominal_throttle_enabled=nominal_throttle;
    config.throttle_v2_enabled=throttle_v2;
    config.speed_rows_removed=speed_rows_removed;
    if (speed_rows_removed) config.nominal_speed_saturation_mps=29.9;
    if (s1_v4_bare) {
        // Trajectory-level adjudication arm: identical dynamics to s1_v4
        // (speed rows removed) except the nominal saturation is disabled -
        // the true bare-nominal control that s1_v4_prime intended to test.
        config.speed_rows_removed=true;
        config.nominal_speed_saturation_mps=0.0;
    }
    if (s1_rung_b) {
        // Ladder rung B: keep the single speed row at the FULL 30 m/s
        // limit, saturate the nominal at 29.9, demote the preflight to the
        // 31 m/s fuse.
        config.speed_row_nominal=true;
        config.speed_row_nominal_limit_mps=30.0;
        config.plant_speed_facet_count=0;
        config.nominal_speed_saturation_mps=29.9;
        config.speed_preflight_demoted=true;
    }
    if (s1_rung_b_prime) {
        // Ladder rung B' (post-adjudication): single speed row at the full
        // 30 m/s limit, UNSATURATED nominal (the saturation wrapper was the
        // adjudicated collapse artifact), preflight demoted to the 31 m/s
        // fuse, and the QP-side estimate SpeedLimit initial-set precheck
        // replaced by runner truth telemetry (boundary-numerics cliff).
        config.speed_row_nominal=true;
        config.speed_row_nominal_limit_mps=30.0;
        config.plant_speed_facet_count=0;
        config.nominal_speed_saturation_mps=0.0;
        config.speed_preflight_demoted=true;
        config.speed_initial_set_truth_gate=true;
    }
    if (s1_rung_b2) {
        // Ladder rung B'' (researcher-approved 2026-09-01): rung B' with
        // speed_cbf_gain raised to 7.0.  The first-order discretization
        // asymptote becomes h* = -(dt/gain)|u|^2 = -|u|^2/70, i.e. a speed
        // asymptote of ~30.0076 at the box-corner |u|=5.657 - inside the
        // 30.01 truth gate (B' at gain=1 asymptoted at 30.038-30.053 and
        // was rejected).
        config.speed_row_nominal=true;
        config.speed_row_nominal_limit_mps=30.0;
        config.plant_speed_facet_count=0;
        config.nominal_speed_saturation_mps=0.0;
        config.speed_preflight_demoted=true;
        config.speed_initial_set_truth_gate=true;
        config.speed_cbf_gain=7.0;
    }
    if (target_policy_v2) config.target_policy_v2=true;
    if (target_policy_v3) config.target_policy_v3=true;
    if (target_policy_v6) config.target_policy_v6=true;
    if (target_policy_unified_h2) {
        config.target_policy_unified_h2=true;
        config.unified_h2_service_standoff_m=
            unified_h2_service_standoff_m;
        // The campaign's hard plant limit is 30 m/s, not the historical
        // 30.01 m/s adjudication tolerance.  Keep the certified speed row
        // and add nominal headroom; this does not relax any safety gate.
        config.nominal_speed_saturation_mps=29.9;
    }
    if (leader_reachability_filter)
        config.leader_reachability_filter=true;
    if (velocity_augmented_rows) {
        config.velocity_augmented_rows=true;
        config.row_slack_epsilon_m=0.5;
    }
    return config;
}

struct Task10p11rFixedBaselineFixture {
    Task10p10Scenario scenario;
    json settings;
    Swarm swarm;
    GrandFinaleSwarmAdapter adapter;
    Task10p11hSimpleCoverageController controller;
    const std::vector<DirectedEdge> frozen_topology;
    const TopologyVersion initial_topology_version;

    explicit Task10p11rFixedBaselineFixture(
        GammaFeedbackSelectionMode selection=
            GammaFeedbackSelectionMode::LeastIntervention,
        std::optional<double> predictive_tau_mps2=14.0)
        : scenario(task10p11rFixedBaselineScenario()),
          settings(task10p11pSwarmSettings(scenario,SolverProfile::Gurobi)),
          swarm(settings),
          adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,
              scenario.initial_topology,task10p11rFixedAdapterConfig(
                  selection,predictive_tau_mps2)),
          controller(swarm,adapter,{}, {},task10p11rAuthorityContract().branches),
          frozen_topology(scenario.initial_topology),
          initial_topology_version(adapter.supervisor().topologyVersion()) {}

    Task10p11rFixedBaselineFixture(
        Task10p10Scenario scenario_value,json settings_value,
        GammaFeedbackSelectionMode selection,
        std::optional<double> predictive_tau_mps2,
        bool speed_row_nominal=false,bool tau_margin_gate=false,
    bool tau_family_predict=false,bool tau_analytic_first_order=false,
    bool s1_v3_preflight_demoted=false,bool nominal_throttle=false,
    bool throttle_v2=false,bool speed_rows_removed=false,
    bool s1_rung_b=false,bool s1_v4_prime=false,bool s1_v4_bare=false,
    bool s1_rung_b_prime=false,bool s1_rung_b2=false,
    bool target_policy_v2=false,bool velocity_augmented_rows=false,
    bool target_policy_v3=false,bool target_policy_v6=false,
    bool leader_reachability_filter=false,
    bool target_policy_unified_h2=false,
    double unified_h2_service_standoff_m=350.0)
        : scenario(std::move(scenario_value)),settings(std::move(settings_value)),
          swarm(settings),
          adapter(swarm,scenario.mobile_ids,scenario.fixed_positions,
              scenario.initial_topology,task10p11rFixtureAdapterConfig(
                  selection,predictive_tau_mps2,speed_row_nominal,
                  tau_margin_gate,tau_family_predict,
                  tau_analytic_first_order,s1_v3_preflight_demoted,
                  nominal_throttle,throttle_v2,speed_rows_removed,
                  s1_rung_b,s1_v4_prime,s1_v4_bare,s1_rung_b_prime,
                  s1_rung_b2,target_policy_v2,velocity_augmented_rows,
                  target_policy_v3,target_policy_v6,
                  leader_reachability_filter,target_policy_unified_h2,
                  unified_h2_service_standoff_m)),
          controller(swarm,adapter,{}, {},task10p11rAuthorityContract().branches),
          frozen_topology(scenario.initial_topology),
          initial_topology_version(adapter.supervisor().topologyVersion()) {}

    bool topologyFrozen() const {
        const auto runtime=adapter.runtimeSnapshot();
        return runtime.topology==frozen_topology &&
            runtime.topology_token==initial_topology_version &&
            runtime.mode==SupervisorMode::Search &&
            !runtime.adapter_transition_pending &&
            !runtime.supervisor_transition_pending;
    }
};

inline std::unique_ptr<Task10p11rFixedBaselineFixture>
makeTask10p11rFixedBaselineFixture() {
    return std::make_unique<Task10p11rFixedBaselineFixture>();
}

inline std::unique_ptr<Task10p11rFixedBaselineFixture>
makeTask10p11rFixedBaselineFixture(
    GammaFeedbackSelectionMode selection,
    std::optional<double> predictive_tau_mps2) {
    return std::make_unique<Task10p11rFixedBaselineFixture>(
        selection,predictive_tau_mps2);
}

inline std::unique_ptr<Task10p11rFixedBaselineFixture>
makeTask10p11rFixedBaselineFixture(
    Task10p10Scenario scenario,json settings,
    GammaFeedbackSelectionMode selection,
    std::optional<double> predictive_tau_mps2) {
    return std::make_unique<Task10p11rFixedBaselineFixture>(
        std::move(scenario),std::move(settings),selection,
        predictive_tau_mps2);
}

}  // namespace gf
