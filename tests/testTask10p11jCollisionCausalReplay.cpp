#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11jCollisionCausalFixture.hpp"

#include <iostream>

TEST_CASE("Task 10.11j reproduces the frozen easy failure without advancing") {
    const auto replay=gf::replayTask10p11jOpenSourceEasy();
    REQUIRE(replay.reproduced);
    REQUIRE(replay.t100_s.has_value());
    CHECK(*replay.t100_s==doctest::Approx(3.9));
    CHECK(replay.failure_time_s==doctest::Approx(4.5));
    CHECK(replay.reason=="current_gamma_negative");
    REQUIRE_FALSE(replay.ticks.empty());
    const auto& failed=replay.ticks.back();
    CHECK_FALSE(failed.advanced);
    CHECK(failed.first.current_gamma==doctest::Approx(-0.0454972).epsilon(1e-5));
    CHECK(failed.second.current_gamma==doctest::Approx(-0.0454972).epsilon(1e-5));
    CHECK(failed.truth_jump_norm==doctest::Approx(0.0));
    CHECK(failed.estimator_version_after==failed.estimator_version_before);
    CHECK(failed.braking.snapshot_coherent);
    CHECK(failed.braking.local_state_constants_coherent);
    CHECK(failed.centralized_applied_residual==
          -std::numeric_limits<double>::infinity());
}

TEST_CASE("Task 10.11j emits the complete focused causal window") {
    const auto replay=gf::replayTask10p11jOpenSourceEasy();
    REQUIRE(replay.reproduced);
    REQUIRE(replay.ticks.size()==16);
    std::cout<<"TASK10P11J_EVENTS,t100="<<*replay.t100_s
             <<",raw_target_change="<<replay.first_raw_target_change_s
             <<",nominal_change="<<replay.first_significant_nominal_change_s
             <<",negative_braking="<<replay.first_negative_braking_slack_s
             <<",empty_polytope="<<replay.first_empty_polytope_s<<'\n';
    std::cout<<"TASK10P11J_TICK,t,advanced,coverage,epoch,target_changed,"
                "p10x,p10y,v10x,v10y,p11x,p11y,v11x,v11y,"
                "tube_p,tube_v,h,hdot,psi1,constant,coeff_reserve,"
                "closing,available,stop_distance,braking_slack,"
                "gamma10,gamma11,u10x,u10y,u11x,u11y,"
                "nom10x,nom10y,nom11x,nom11y,u0_10x,u0_10y,u0_11x,u0_11y,"
                "selected10x,selected10y,selected11x,selected11y,"
                "max10x,max10y,max11x,max11y,predicted_gamma10,"
                "predicted_gamma11,measurement_update,pair_residual,exact10,"
                "exact11,estimator_versions,target10x,target10y,target11x,"
                "target11y,target_nominal_effect,reason\n";
    for (const auto& tick:replay.ticks) {
        std::cout<<"TASK10P11J_TICK,"<<tick.time_before_s<<','
                 <<tick.advanced<<','<<tick.truth_coverage_before<<','
                 <<tick.target_epoch_after<<','<<tick.target_changed<<','
                 <<tick.first.estimate_before.transpose()<<','
                 <<tick.second.estimate_before.transpose()<<','
                 <<tick.pair_tube.position_radius_m<<','
                 <<tick.pair_tube.velocity_radius_mps<<','
                 <<tick.pair_h<<','<<tick.pair_hdot<<','<<tick.pair_psi1<<','
                 <<tick.pair_half_constant<<','
                 <<tick.pair_coefficient_reserve_each<<','
                 <<tick.braking.robust_radial_closing_speed_mps<<','
                 <<tick.braking.full_hard_relative_separation_support_mps2<<','
                 <<tick.braking.stop_distance_m<<','
                 <<tick.braking.braking_slack_m<<','
                 <<tick.first.current_gamma<<','<<tick.second.current_gamma<<','
                 <<tick.first.applied_control.transpose()<<','
                 <<tick.second.applied_control.transpose()<<','
                 <<tick.first.nominal.transpose()<<','
                 <<tick.second.nominal.transpose()<<','
                 <<tick.first.hard_projection.transpose()<<','
                 <<tick.second.hard_projection.transpose()<<','
                 <<tick.first.selected_gamma_candidate.transpose()<<','
                 <<tick.second.selected_gamma_candidate.transpose()<<','
                 <<tick.first.maximum_margin_control.transpose()<<','
                 <<tick.second.maximum_margin_control.transpose()<<','
                 <<tick.first.no_measurement_predicted_next_gamma<<','
                 <<tick.second.no_measurement_predicted_next_gamma<<','
                 <<tick.measurement_update_correction_norm<<','
                 <<tick.centralized_applied_residual<<','
                 <<tick.exact_owner10_feasible<<','
                 <<tick.exact_owner11_feasible<<','
                 <<tick.estimator_version_before<<"->"
                 <<tick.estimator_version_after<<','
                 <<tick.first.raw_target.transpose()<<','
                 <<tick.second.raw_target.transpose()<<','
                 <<tick.nominal_target_change_effect_mps2<<','
                 <<tick.reason<<'\n';
    }
    const auto& failed=replay.ticks.back();
    CHECK(failed.owner10_conflict==std::vector<std::string>{
        "$input_box","collision:10--11:owner:10"});
    CHECK(failed.owner11_conflict==std::vector<std::string>{
        "$input_box","collision:10--11:owner:11"});
}

TEST_CASE("Task 10.11j frozen pre-T100 target is diagnostic-only counterfactual") {
    const auto counterfactual=gf::replayTask10p11jFrozenPreT100Targets();
    REQUIRE(counterfactual.prefix_valid);
    CHECK(counterfactual.advanced_through_4p6_s);
    CHECK(counterfactual.reason=="diagnostic_advanced");
    CHECK(counterfactual.minimum_current_gamma>=0.0);
    CHECK(counterfactual.minimum_braking_slack_m>=0.0);
    MESSAGE("TASK10P11J_COUNTERFACTUAL kind=frozen_pre_t100_target",
        " advanced_4p6=",counterfactual.advanced_through_4p6_s,
        " failure_t=",counterfactual.failure_time_s,
        " min_gamma=",counterfactual.minimum_current_gamma,
        " min_braking_slack=",counterfactual.minimum_braking_slack_m,
        " claim=diagnostic_only");
}

TEST_CASE("Task 10.11j linear target homotopy is diagnostic-only counterfactual") {
    const auto counterfactual=gf::replayTask10p11jLinearTargetHomotopy();
    REQUIRE(counterfactual.prefix_valid);
    MESSAGE("TASK10P11J_COUNTERFACTUAL kind=linear_target_homotopy_0p5s",
        " advanced_4p6=",counterfactual.advanced_through_4p6_s,
        " failure_t=",counterfactual.failure_time_s,
        " reason=",counterfactual.reason,
        " min_gamma=",counterfactual.minimum_current_gamma,
        " min_braking_slack=",counterfactual.minimum_braking_slack_m,
        " claim=diagnostic_only");
    CHECK(counterfactual.minimum_braking_slack_m>=0.0);
}

TEST_CASE("Task 10.11j locks the A-B primary and C-boundary causal facts") {
    const auto replay=gf::replayTask10p11jOpenSourceEasy();
    REQUIRE(replay.reproduced);
    double maximum_truth_estimate_error=0.0;
    for (const auto& tick:replay.ticks) {
        CHECK(tick.pair_h>=0.0);
        CHECK(tick.pair_psi1>=0.0);
        CHECK(tick.braking.braking_slack_m>=0.0);
        CHECK(tick.braking.snapshot_coherent);
        CHECK(tick.braking.independent_central_verified);
        CHECK(tick.measurement_update_correction_norm==doctest::Approx(0.0));
        if (tick.advanced)
            CHECK(tick.centralized_applied_residual>=-1e-7);
        maximum_truth_estimate_error=std::max({
            maximum_truth_estimate_error,
            (tick.first.truth_before-tick.first.estimate_before).norm(),
            (tick.second.truth_before-tick.second.estimate_before).norm()});
    }
    CHECK(maximum_truth_estimate_error==doctest::Approx(0.0));
    const auto centroid=std::find_if(
        replay.ticks.begin(),replay.ticks.end(),[](const auto& tick) {
            return std::abs(tick.time_before_s-4.0)<=1e-12;
        });
    REQUIRE(centroid!=replay.ticks.end());
    CHECK(centroid->target_changed);
    CHECK(centroid->nominal_target_change_effect_mps2>9.0);
    CHECK((centroid->first.raw_target-centroid->second.raw_target).norm()>100.0);
    const auto last_safe=std::find_if(
        replay.ticks.begin(),replay.ticks.end(),[](const auto& tick) {
            return std::abs(tick.time_before_s-4.4)<=1e-12;
        });
    REQUIRE(last_safe!=replay.ticks.end());
    CHECK(last_safe->first.applied_control.isApprox(
        last_safe->first.maximum_margin_control,1e-10));
    CHECK(last_safe->second.applied_control.isApprox(
        last_safe->second.maximum_margin_control,1e-10));
    CHECK(last_safe->first.no_measurement_predicted_next_gamma<0.0);
    CHECK(last_safe->second.no_measurement_predicted_next_gamma<0.0);
    CHECK(last_safe->measurement_update_correction_norm==doctest::Approx(0.0));
    MESSAGE("TASK10P11J_CAUSAL_FACTS max_truth_estimate_error=",
        maximum_truth_estimate_error,
        " t4_target_nominal_effect=",
        centroid->nominal_target_change_effect_mps2,
        " t4_target_separation=",
        (centroid->first.raw_target-centroid->second.raw_target).norm(),
        " t4p4_predicted_gamma=",
        last_safe->first.no_measurement_predicted_next_gamma,
        " t4p4_braking_slack=",last_safe->braking.braking_slack_m);
}
