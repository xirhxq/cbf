#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/RouteBPipelinedExecutor.hpp"
#include "grand_finale/RouteBPlanner.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>

TEST_CASE("TDD-8 Confirmatory multi-seed Monte Carlo check under degraded UWB tracks peak scale radius") {
    const std::vector<double> b4 = {0.0, 400.0, 800.0, 1200.0};
    const auto p6 = gf::buildPinballTriangulation(3, 400.0, 100.0, 140.0);
    const auto dl3 = gf::buildStaggeredTriangulation(
        b4, {{1, 2, 3}, {}, {4, 5, 6}}, 380.0, 90.0, 100.0, 140.0);
    const auto p6_to_dl3 = gf::planLabelledTransfer(p6, dl3);

    const auto src14 = gf::buildStaggeredTriangulation(
        b4, {{1, 2, 3, 4, 5, 6, 7}, {}, {8, 9, 10, 11, 12, 13, 14}},
        380.0, 90.0, 100.0, 140.0);
    const auto dst14 = gf::buildStaggeredTriangulation(
        b4, {{}, {14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1}, {}},
        380.0, 90.0, 100.0, 140.0);
    const auto [moves14, matched_dst14] = gf::planDirectMatchedTransfer(src14, dst14);

    // Sample two confirmatory seeds in unit test (full 16-seed campaign 1001..1016 runs in GrandFinaleRouteBRunner)
    for (unsigned int seed : {1001u, 1002u}) {
        gf::RouteBPipelinedConfig cfg;
        cfg.serial_mode = false;
        cfg.v_cruise_mps = 14.0;
        cfg.accel_mps2 = 2.0;
        cfg.d_pipe_m = 60.0;
        cfg.d_clear_m = 32.0;
        cfg.t_settle_s = 1.0;
        cfg.uwb_d_r_m = 600.0;
        cfg.uwb_sigma0_m = 1.0;
        cfg.uwb_d_max_m = 1200.0;
        cfg.link_seed = seed;

        const auto r6 = gf::runClosedLoopReconfiguration(p6, dl3, p6_to_dl3, cfg);
        CHECK(r6.completed);
        CHECK(r6.static_parent_violations == 0);
        CHECK(r6.dag_violations == 0);
        CHECK(r6.audit.collision_violations == 0);
        CHECK(r6.audit.reference_violations == 0);
        CHECK(r6.audit.axis_speed_violations == 0);
        CHECK(r6.audit.peak_position_scale_radius_m > 0.0);
        CHECK(r6.audit.mean_position_scale_radius_m > 0.0);
        CHECK(r6.audit.peak_position_covariance_trace_m2 > 0.0);

        cfg.v_cruise_mps = 15.0;
        cfg.max_sim_time_s = 1600.0;
        const auto r14 = gf::runClosedLoopReconfiguration(src14, matched_dst14, moves14, cfg);
        CHECK(r14.completed);
        CHECK(r14.static_parent_violations == 0);
        CHECK(r14.dag_violations == 0);
        CHECK(r14.audit.collision_violations == 0);
        CHECK(r14.audit.reference_violations == 0);
        CHECK(r14.audit.axis_speed_violations == 0);
        CHECK(r14.audit.peak_position_scale_radius_m > 0.0);
    }
}
