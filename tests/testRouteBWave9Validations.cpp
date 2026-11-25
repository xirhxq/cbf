#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/RouteBWave9Validations.hpp"

#include <cmath>
#include <iostream>

TEST_CASE("Online two-regime criterion selects Switch in Regime 1, Stay below Acrit, and Switch above Acrit") {
    // 1. Regime 1: remaining uncertified domain exceeds initial formation reach
    gf::OnlineTwoRegimeInputs reg1;
    reg1.uncertified_max_reach_m = 1550.0;
    reg1.current_formation_reach_m = 1180.0;
    reg1.target_formation_reach_m = 1680.0;
    reg1.remaining_area_m2 = 0.49e6;
    reg1.measured_r0_m2ps = 2200.0;
    reg1.predicted_r1_m2ps = 1800.0;
    reg1.predicted_t_trans_s = 600.0;
    reg1.predicted_a_trans_m2 = 0.0;
    const auto dec1 = gf::evaluateOnlineTwoRegimeDecision(reg1);
    CHECK(dec1.regime == gf::TwoRegimeDecisionKind::Regime1ReachabilitySwitch);
    CHECK(dec1.should_switch);

    // 2. Regime 2 (sub-critical): reachable by both, A_rem <= A_crit -> Stay
    gf::OnlineTwoRegimeInputs reg2_stay;
    reg2_stay.uncertified_max_reach_m = 1500.0;
    reg2_stay.current_formation_reach_m = 1650.0;
    reg2_stay.target_formation_reach_m = 1650.0;
    reg2_stay.remaining_area_m2 = 1.8e6;
    reg2_stay.measured_r0_m2ps = 2500.0;
    reg2_stay.predicted_r1_m2ps = 4500.0;
    reg2_stay.predicted_t_trans_s = 604.0;
    reg2_stay.predicted_a_trans_m2 = 0.8e6;
    const auto dec2_stay = gf::evaluateOnlineTwoRegimeDecision(reg2_stay);
    CHECK(dec2_stay.regime == gf::TwoRegimeDecisionKind::Regime2SubcriticalStay);
    CHECK_FALSE(dec2_stay.should_switch);
    CHECK(dec2_stay.remaining_area_m2 <= dec2_stay.a_crit_m2);

    // 3. Regime 2 (super-critical): reachable by both, A_rem > A_crit -> Switch
    gf::OnlineTwoRegimeInputs reg2_sw = reg2_stay;
    reg2_sw.remaining_area_m2 = 8.4e6;
    const auto dec2_sw = gf::evaluateOnlineTwoRegimeDecision(reg2_sw);
    CHECK(dec2_sw.regime == gf::TwoRegimeDecisionKind::Regime2EfficiencySwitch);
    CHECK(dec2_sw.should_switch);
    CHECK(dec2_sw.remaining_area_m2 > dec2_sw.a_crit_m2);
}

TEST_CASE("Sub-critical coastal belt starts below 40% coverage, achieves 100% in both arms, and Stay wins") {
    const auto bundle = gf::runSubcriticalCoastalMission("nominal");
    std::cout << "[Wave9 Subcritical] area_km2=" << (bundle.comparison.area_m2 / 1.0e6)
              << " fork_cov=" << bundle.comparison.fork_certified_coverage
              << " stay_t100=" << bundle.comparison.stay.t100_certified_s.value_or(-1.0)
              << " sw_reconf=" << bundle.comparison.switch_arm.reconfig_complete_s
              << " sw_t100=" << bundle.comparison.switch_arm.t100_certified_s.value_or(-1.0)
              << " decision=" << static_cast<int>(bundle.online_decision.regime)
              << " a_crit_km2=" << (bundle.online_decision.a_crit_m2 / 1.0e6) << std::endl;

    CHECK(bundle.comparison.checkpoint_byte_identical);
    CHECK(bundle.comparison.fork_certified_coverage < 0.45);
    CHECK(bundle.comparison.stay.reached_t100);
    CHECK(bundle.comparison.switch_arm.reached_t100);
    CHECK(bundle.comparison.switch_arm.reconfig_completed);
    CHECK(bundle.comparison.stay.false_certified_count == 0);
    CHECK(bundle.comparison.switch_arm.false_certified_count == 0);
    CHECK(bundle.comparison.stay.audit.collision_violations == 0);
    CHECK(bundle.comparison.stay.audit.reference_violations == 0);
    CHECK(bundle.comparison.switch_arm.audit.collision_violations == 0);
    CHECK(bundle.comparison.switch_arm.audit.reference_violations == 0);
    REQUIRE(bundle.comparison.stay.t100_certified_s.has_value());
    REQUIRE(bundle.comparison.switch_arm.t100_certified_s.has_value());
    CHECK(*bundle.comparison.stay.t100_certified_s < *bundle.comparison.switch_arm.t100_certified_s);
    CHECK_FALSE(bundle.online_decision.should_switch);
}

TEST_CASE("Online PIQP + D-EKF per-tick compute timing and radio bandwidth telemetry are measured") {
    const auto stats = gf::measureRouteBComputeAndComm(14);
    std::cout << "[Wave9 Timing N=14] qp_mean_ms=" << stats.qp_solve_mean_ms
              << " qp_p99_ms=" << stats.qp_solve_p99_ms
              << " est_mean_ms=" << stats.estimator_mean_ms
              << " tick_mean_ms=" << stats.tick_total_mean_ms
              << " pkt_per_tick=" << stats.mean_packets_per_tick
              << " bytes_per_s=" << stats.mean_bandwidth_bytes_per_s << std::endl;

    CHECK(stats.ticks > 100);
    CHECK(std::isfinite(stats.qp_solve_mean_ms));
    CHECK(stats.qp_solve_mean_ms > 0.0);
    CHECK(stats.qp_solve_p99_ms >= stats.qp_solve_mean_ms);
    CHECK(stats.estimator_mean_ms > 0.0);
    CHECK(stats.tick_total_mean_ms < 100.0);
    CHECK(stats.mean_packets_per_tick > 0.0);
    CHECK(stats.mean_bandwidth_bytes_per_s > 0.0);
}

TEST_CASE("Mild wind disturbance, positive range bias, and decimeter beacon drift preserve zero violations") {
    const auto pert = gf::runMildMarinePerturbationCheck();
    std::cout << "[Wave9 Perturbation] makespan=" << pert.makespan_s
              << " col=" << pert.audit.collision_violations
              << " ref=" << pert.audit.reference_violations
              << " rmse=" << pert.audit.position_rmse_m
              << " env_cov=" << pert.audit.overall_envelope_coverage << std::endl;

    CHECK(pert.completed);
    CHECK(pert.final_topology_matched);
    CHECK(pert.static_parent_violations == 0);
    CHECK(pert.dag_violations == 0);
    CHECK(pert.audit.collision_violations == 0);
    CHECK(pert.audit.reference_violations == 0);
    CHECK(pert.audit.axis_speed_violations == 0);
}
