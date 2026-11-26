#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/RouteBWave11Validations.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

TEST_CASE("Exact two-sided berth condition catches endpoint breakdown counterexample and matches interval minimum") {
    // Nonuniform (320, 400, 360) m, iota_req = 0.40: [100.0, 240.0] m
    const std::vector<double> nu_xs = {0.0, 320.0, 720.0, 1080.0};
    const auto bounds = gf::computeTwoSidedBerthBounds(nu_xs, 0.40);
    CHECK(bounds.feasible);
    CHECK(bounds.h_b_min_m == doctest::Approx(100.0).epsilon(1.0e-6));
    CHECK(bounds.h_b_max_m == doctest::Approx(240.0).epsilon(1.0e-6));
    CHECK(gf::checkTwoSidedBerthCondition(nu_xs, 100.0, 0.40));
    CHECK_FALSE(gf::checkTwoSidedBerthCondition(nu_xs, 99.0, 0.40));
    CHECK_FALSE(gf::checkTwoSidedBerthCondition(nu_xs, 241.0, 0.40));

    // Referee counterexample: s = (400, 400) m, iota_req = 0.80 > 2/3 -> infeasible at endpoint x_r!
    const std::vector<double> u_xs = {0.0, 400.0, 800.0};
    const auto bad_bounds = gf::computeTwoSidedBerthBounds(u_xs, 0.80);
    CHECK_FALSE(bad_bounds.feasible);
    CHECK_FALSE(gf::checkTwoSidedBerthCondition(u_xs, 164.0, 0.80));
    const double eig_endpoint = 1.0 - 164.0 / std::hypot(400.0, 164.0);
    CHECK(eig_endpoint < 0.80);

    // Numerical verification over 200 random nonuniform layouts against dense grid minimum
    std::mt19937 rng(20261126);
    std::uniform_real_distribution<double> s_dist(220.0, 460.0);
    std::uniform_real_distribution<double> iota_dist(0.10, 0.65);
    std::uniform_real_distribution<double> h_dist(15.0, 350.0);
    for (int t = 0; t < 200; ++t) {
        std::vector<double> xs{0.0};
        for (int k = 0; k < 3; ++k) xs.push_back(xs.back() + s_dist(rng));
        const double iota = iota_dist(rng);
        const double h = h_dist(rng);
        double grid_min = 1.0;
        for (std::size_t r = 0; r + 1 < xs.size(); ++r) {
            const double sr = xs[r + 1] - xs[r];
            for (int m = 0; m <= 200; ++m) {
                const double dx1 = (static_cast<double>(m) / 200.0) * sr;
                const double dx2 = dx1 - sr;
                const double cos_th = (dx1 * dx2 + h * h) /
                    (std::hypot(dx1, h) * std::hypot(dx2, h));
                grid_min = std::min(grid_min, 1.0 - std::abs(cos_th));
            }
        }
        const bool pred = gf::checkTwoSidedBerthCondition(xs, h, iota);
        const bool truth = (grid_min >= iota - 1.0e-5);
        if (std::abs(grid_min - iota) > 1.0e-4) {
            CHECK(pred == truth);
        }
    }
}

TEST_CASE("2D DAG-hop-bounded certifiable region classifies all four coastal domains cleanly") {
    const auto summary = gf::evaluateAllPlanarTwoRegimeDomains();
    REQUIRE(summary.size() == 4);
    // narrow_deep_offshore -> Regime 1 Switch
    CHECK(summary[0].regime == gf::TwoRegimeDecisionKind::Regime1ReachabilitySwitch);
    CHECK(summary[0].should_switch);
    // oblique_asymmetric -> Regime 1 Switch (catching lateral flank unreachability of K_3!)
    CHECK(summary[1].regime == gf::TwoRegimeDecisionKind::Regime1ReachabilitySwitch);
    CHECK(summary[1].should_switch);
    // subcritical_coastal_regime2 -> Regime 2 Stay
    CHECK(summary[2].regime == gf::TwoRegimeDecisionKind::Regime2SubcriticalStay);
    CHECK_FALSE(summary[2].should_switch);
    // wide_broad_regime2 -> Regime 2 Switch
    CHECK(summary[3].regime == gf::TwoRegimeDecisionKind::Regime2EfficiencySwitch);
    CHECK(summary[3].should_switch);
    for (const auto& d : summary) {
        std::cout << "[Wave11 2D Selector] " << d.domain_name
                  << " src_cov_frac=" << d.src_reachable_fraction
                  << " dst_cov_frac=" << d.dst_reachable_fraction
                  << " union_cov_frac=" << d.union_reachable_fraction
                  << " switch=" << d.should_switch << std::endl;
    }
}

TEST_CASE("Nonuniform pentagon nominal arm and perturbed reversible coastal search complete safely") {
    const auto nu_nom = gf::runNonuniformPentagonNominalMission();
    std::cout << "[Wave11 Nonuniform Nominal] stay_final=" << nu_nom.stay_final_cov
              << " switch_t100=" << nu_nom.switch_t100_s.value_or(-1.0)
              << " reconf_s=" << nu_nom.reconfig_s
              << " col=" << nu_nom.collision_violations
              << " ref=" << nu_nom.reference_violations
              << " fc=" << nu_nom.false_certified_count << std::endl;
    CHECK(nu_nom.switch_reached_t100);
    CHECK(nu_nom.collision_violations == 0);
    CHECK(nu_nom.reference_violations == 0);
    CHECK(nu_nom.false_certified_count == 0);

    const auto pert = gf::runPerturbedReversibleSearchValidation();
    std::cout << "[Wave11 Perturbed Reversible] t95=" << pert.t95_certified_s.value_or(-1.0)
              << " t100=" << pert.t100_certified_s.value_or(-1.0)
              << " final_cov=" << pert.final_certified_coverage
              << " fwd_reconf=" << pert.forward_reconfig_s
              << " ret_entry=" << pert.return_entry_s
              << " ret_reconf=" << pert.return_reconfig_s
              << " total_time=" << pert.total_time_s
              << " col=" << pert.collision_violations
              << " ref=" << pert.reference_violations
              << " fc=" << pert.false_certified_count
              << " cenv=" << pert.overall_envelope_coverage
              << " worst_cenv=" << pert.worst_agent_envelope_coverage
              << " rmse=" << pert.position_rmse_m << std::endl;
    CHECK(pert.reached_t95);
    CHECK(pert.final_certified_coverage >= 0.98);
    CHECK(pert.return_completed);
    CHECK(pert.final_topology_matched);
    CHECK(pert.false_certified_count == 0);
    CHECK(pert.collision_violations == 0);
    CHECK(pert.reference_violations == 0);
    CHECK(pert.axis_speed_violations == 0);
    CHECK(pert.static_parent_violations == 0);
    CHECK(pert.dag_violations == 0);
}
