#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/RouteBMissionRunner.hpp"

#include <iostream>

TEST_CASE("TDD-7 Regime 2 broad convex domain achieves positive net T95 and T100 breakeven after full reconfiguration") {
    gf::RouteBConvexDomainSpec dom;
    dom.name = "wide_broad_regime2";
    dom.description = "Broad 10+ km^2 convex coastal trapezoid exceeding critical area where parallel multi-unit expansion outperforms single deep column";
    dom.vertices = {
        {-2050.0, 120.0},
        {4050.0, 120.0},
        {2950.0, 2050.0},
        {-950.0, 2050.0}
    };
    const std::vector<double> b_xs = {0.0, 400.0, 800.0, 1200.0, 1600.0, 2000.0};
    dom.initial_tri = gf::buildStaggeredTriangulation(
        b_xs, {{}, {}, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}, {}, {}},
        380.0, 90.0, 100.0, 140.0);
    dom.target_tri = gf::buildStaggeredTriangulation(
        b_xs, {{1, 2, 3, 4, 5}, {}, {}, {}, {6, 7, 8, 9, 10}},
        380.0, 90.0, 100.0, 140.0);
    dom.use_direct_matched = true;

    for (const auto [d_r, sigma0] : std::vector<std::pair<double, double>>{{850.0, 0.5}, {600.0, 1.0}}) {
        gf::RouteBMissionConfig mcfg;
        mcfg.domain_name = dom.name;
        mcfg.custom_domain = dom;
        mcfg.cell_size_m = 40.0;
        mcfg.sensor_radius_m = 400.0;
        mcfg.dynamic_chain_sweep = true;
        mcfg.search_speed_cap_mps = 5.0;
        mcfg.patrol_radius_m = 2600.0;
        mcfg.v_cruise_mps = 23.5;
        mcfg.accel_mps2 = 3.5;
        mcfg.d_pipe_m = 45.0;
        mcfg.t_settle_s = 0.2;
        mcfg.t_fork_s = 10.0;
        mcfg.max_mission_s = 2800.0;
        mcfg.uwb_d_r_m = d_r;
        mcfg.uwb_sigma0_m = sigma0;
        mcfg.uwb_d_max_m = 1200.0;
        mcfg.link_seed = 20261123;
        mcfg.curve_sample_interval_s = 10.0;

        const auto res = gf::runTwoRegimeMissionComparison(mcfg);
        std::cout << "[TDD-7 d_r=" << d_r << " s0=" << sigma0 << "] area_km2="
                  << (res.area_m2 / 1.0e6)
                  << " stay(t95=" << res.stay.t95_certified_s.value_or(-1.0)
                  << ", t100=" << res.stay.t100_certified_s.value_or(-1.0)
                  << ", cov=" << res.stay.final_certified_coverage
                  << ", refv=" << res.stay.audit.reference_violations
                  << ", pk=" << res.stay.audit.peak_position_scale_radius_m
                  << ", st=" << res.stay.status_reason << ")"
                  << " switch(reconf=" << res.switch_arm.reconfig_complete_s
                  << ", t95=" << res.switch_arm.t95_certified_s.value_or(-1.0)
                  << ", t100=" << res.switch_arm.t100_certified_s.value_or(-1.0)
                  << ", cov=" << res.switch_arm.final_certified_coverage
                  << ", refv=" << res.switch_arm.audit.reference_violations
                  << ", pk=" << res.switch_arm.audit.peak_position_scale_radius_m
                  << ", st=" << res.switch_arm.status_reason << ")";
        std::cout << " sw_peaks=[";
        for (const auto& [aid, pk] : res.switch_arm.audit.per_agent_peak_position_scale_radius_m) {
            std::cout << aid << ":" << pk << " ";
        }
        std::cout << "]" << std::endl;

        CHECK(res.checkpoint_byte_identical);
        CHECK(res.area_m2 >= 9.5e6);
        CHECK(res.area_m2 <= 14.5e6);

        CHECK(res.stay.reached_t95);
        CHECK(res.stay.reached_t100);
        CHECK(res.stay.false_certified_count == 0);
        CHECK(res.stay.certified_subset_of_truth);
        CHECK(res.stay.audit.collision_violations == 0);
        CHECK(res.stay.audit.reference_violations == 0);
        CHECK(res.stay.audit.axis_speed_violations == 0);

        CHECK(res.switch_arm.reconfig_completed);
        CHECK(res.switch_arm.reached_t95);
        CHECK(res.switch_arm.reached_t100);
        CHECK(res.switch_arm.false_certified_count == 0);
        CHECK(res.switch_arm.certified_subset_of_truth);
        CHECK(res.switch_arm.static_parent_violations == 0);
        CHECK(res.switch_arm.dag_violations == 0);
        CHECK(res.switch_arm.audit.collision_violations == 0);
        CHECK(res.switch_arm.audit.reference_violations == 0);
        CHECK(res.switch_arm.audit.axis_speed_violations == 0);

        REQUIRE(res.stay.t95_certified_s.has_value());
        REQUIRE(res.stay.t100_certified_s.has_value());
        REQUIRE(res.switch_arm.t95_certified_s.has_value());
        REQUIRE(res.switch_arm.t100_certified_s.has_value());
        CHECK(*res.switch_arm.t95_certified_s < *res.stay.t95_certified_s);
        CHECK(*res.switch_arm.t100_certified_s < *res.stay.t100_certified_s);
    }
}
