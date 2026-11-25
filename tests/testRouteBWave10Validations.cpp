#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/RouteBWave10Validations.hpp"

#include <cmath>
#include <iostream>
#include <vector>

TEST_CASE("Exact 3-beacon corridor check catches 4-beacon blind zone counterexample and certifies valid layouts") {
    // Reviewer M=7 counterexample: s = (10, 10, 80, 80, 10, 10), r_cor = 82 m.
    // 4-beacon window (b3..b6) midpoint has 3rd-closest distance (10 + 80 + 80)/2 = 85 m > 82 m.
    const std::vector<double> counter_xs = {0.0, 10.0, 20.0, 100.0, 180.0, 190.0, 200.0};
    CHECK_FALSE(gf::checkThreeBeaconCorridorCondition(counter_xs, 82.0));
    CHECK(gf::checkThreeBeaconCorridorCondition(counter_xs, 85.0));

    // Uniform M=4, s=400 m: 3/2 * 400 = 600 <= r_cor = 834.3 m
    CHECK(gf::checkThreeBeaconCorridorCondition({0.0, 400.0, 800.0, 1200.0}, 834.3));

    // Nonuniform M=4, s=(320, 400, 360) m: max(160+400, 180+400, (320+400+360)/2) = 580 <= 834.3 m
    CHECK(gf::checkThreeBeaconCorridorCondition({0.0, 320.0, 720.0, 1080.0}, 834.3));

    // M=3 endpoints: s=(300, 500) -> max(150+500, 250+300) = 650 m
    CHECK_FALSE(gf::checkThreeBeaconCorridorCondition({0.0, 300.0, 800.0}, 640.0));
    CHECK(gf::checkThreeBeaconCorridorCondition({0.0, 300.0, 800.0}, 650.0));
}

TEST_CASE("Reversible round-trip Dual -> Single -> Dual completes 100% convex search where static arms stall") {
    const auto rev = gf::runReversibleConvexSearchComparison("degraded");
    std::cout << "[Wave10 Reversible] dual_final=" << rev.fixed_dual.final_certified_coverage
              << " single_final=" << rev.fixed_single.final_certified_coverage
              << " rt_t100=" << rev.round_trip.t100_certified_s.value_or(-1.0)
              << " fwd_reconf=" << rev.round_trip.forward_reconfig_s
              << " rev_reconf=" << rev.round_trip.return_reconfig_s
              << " total_time=" << rev.round_trip.total_time_s
              << " col=" << rev.round_trip.collision_violations
              << " ref=" << rev.round_trip.reference_violations
              << " fc=" << rev.round_trip.false_certified_count << std::endl;

    CHECK(rev.fixed_dual.final_certified_coverage < 0.96);
    CHECK(rev.fixed_single.final_certified_coverage < 0.96);
    CHECK(rev.round_trip.reached_t100);
    CHECK(rev.round_trip.return_completed);
    CHECK(rev.round_trip.final_topology_matched);
    CHECK(rev.round_trip.collision_violations == 0);
    CHECK(rev.round_trip.reference_violations == 0);
    CHECK(rev.round_trip.axis_speed_violations == 0);
    CHECK(rev.round_trip.static_parent_violations == 0);
    CHECK(rev.round_trip.dag_violations == 0);
    CHECK(rev.round_trip.false_certified_count == 0);
}

TEST_CASE("8-trial degraded marine perturbation batch completes with zero physical violations") {
    const auto batch = gf::runMarinePerturbationMonteCarlo(8);
    std::cout << "[Wave10 Perturb 8-trial] safe=" << batch.safe_pass_count << "/" << batch.runs.size()
              << " makespan=" << batch.makespan_mean_s << "+-" << batch.makespan_std_s
              << " rmse=" << batch.rmse_mean_m << "+-" << batch.rmse_std_m
              << " cenv=" << batch.cenv_mean << "+-" << batch.cenv_std << std::endl;

    REQUIRE(batch.runs.size() == 8);
    CHECK(batch.safe_pass_count == 8);
    CHECK(batch.makespan_mean_s > 500.0);
    CHECK(batch.makespan_mean_s < 700.0);
    CHECK(batch.rmse_mean_m > 0.5);
    CHECK(batch.cenv_mean < 0.98);
    CHECK(batch.cenv_mean > 0.70);
}