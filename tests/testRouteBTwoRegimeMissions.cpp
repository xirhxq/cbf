#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/RouteBMissionRunner.hpp"
#include "grand_finale/SearchPolygonDomain.hpp"

#include <cmath>
#include <string>
#include <vector>

TEST_CASE("Standard convex domains match SearchPolygonDomain 10 m grid contract") {
    const auto domains = gf::standardRouteBConvexDomains();
    CHECK(domains.size() == 3);
    CHECK(domains[0].name == "wide_shallow_coastal");
    CHECK(domains[1].name == "narrow_deep_offshore");
    CHECK(domains[2].name == "oblique_asymmetric");

    for (const auto& dom : domains) {
        CHECK(dom.vertices.size() >= 4);
        json base = {{"world", {{"boundary", {{0.0, 0.0}, {100.0, 0.0}, {100.0, 100.0}, {0.0, 100.0}}},
                                {"spacing", 10.0}}}};
        json site = {
            {"schema", "gf-search-polygon-v1"},
            {"id", dom.name},
            {"vertices_m", json::array()},
            {"cell_size_m", 10.0}
        };
        for (const auto& v : dom.vertices) {
            site["vertices_m"].push_back({v.x(), v.y()});
        }
        const json applied = gf::searchPolygonSettings(base, site);
        GridWorld grid(applied.at("world"));
        CHECK(grid.validCount > 4000);
        const double dx = (grid.xLim.second - grid.xLim.first) / static_cast<double>(grid.xNum);
        const double dy = (grid.yLim.second - grid.yLim.first) / static_cast<double>(grid.yNum);
        CHECK(dx * dy == doctest::Approx(100.0));
    }
}

TEST_CASE("Regime 1 narrow_deep_offshore: byte-identical fork where Stay fails T100 and Switch L14 completes T100") {
    gf::RouteBMissionConfig mcfg;
    mcfg.domain_name = "narrow_deep_offshore";
    mcfg.t_fork_s = 15.0;
    mcfg.max_mission_s = 1350.0;
    mcfg.patrol_radius_m = 80.0;
    mcfg.v_cruise_mps = 15.0;
    mcfg.uwb_d_r_m = 850.0;
    mcfg.uwb_sigma0_m = 0.5;
    mcfg.uwb_d_max_m = 1200.0;
    mcfg.link_seed = 20261123;

    const auto paired = gf::runTwoRegimeMissionComparison(mcfg);
    std::printf(
        "[TDD-6 Regime1 narrow_deep_offshore] cells=%d stay_cov=%.4f stay_t100=%d | "
        "sw_reconf=%d(%.1fs) sw_cov=%.4f sw_t95=%.1f sw_t100=%.1f col=%zu ref=%zu spd=%zu\n",
        paired.valid_cells, paired.stay.final_certified_coverage, paired.stay.reached_t100 ? 1 : 0,
        paired.switch_arm.reconfig_completed ? 1 : 0, paired.switch_arm.reconfig_complete_s,
        paired.switch_arm.final_certified_coverage,
        paired.switch_arm.t95_certified_s.value_or(-1.0),
        paired.switch_arm.t100_certified_s.value_or(-1.0),
        paired.switch_arm.audit.collision_violations,
        paired.switch_arm.audit.reference_violations,
        paired.switch_arm.audit.axis_speed_violations);
    CHECK(paired.checkpoint_byte_identical);
    CHECK(paired.fork_time_s == doctest::Approx(15.0));

    // Stay arm: acute 7-layer side ladders physically cannot illuminate deep offshore y=1400..1550
    CHECK_FALSE(paired.stay.reached_t100);
    CHECK(paired.stay.final_certified_coverage < 0.95);
    CHECK(paired.stay.false_certified_count == 0);
    CHECK(paired.stay.certified_subset_of_truth);
    CHECK(paired.stay.audit.collision_violations == 0);
    CHECK(paired.stay.audit.reference_violations == 0);
    CHECK(paired.stay.audit.axis_speed_violations == 0);

    // Switch arm: polar entry + pipelined direct-matched merge to L14 completes T100
    CHECK(paired.switch_arm.reconfig_completed);
    CHECK(paired.switch_arm.reached_t100);
    CHECK(paired.switch_arm.t100_certified_s.has_value());
    CHECK(paired.switch_arm.false_certified_count == 0);
    CHECK(paired.switch_arm.certified_subset_of_truth);
    CHECK(paired.switch_arm.static_parent_violations == 0);
    CHECK(paired.switch_arm.dag_violations == 0);
    CHECK(paired.switch_arm.audit.collision_violations == 0);
    CHECK(paired.switch_arm.audit.reference_violations == 0);
    CHECK(paired.switch_arm.audit.axis_speed_violations == 0);
    CHECK(!paired.switch_arm.coverage_curve.empty());
}

TEST_CASE("Regime 2 wide_shallow_coastal and oblique_asymmetric full-clock paired comparison from byte-identical checkpoints") {
    for (const auto& [dom_name, t_fork, t_cap] :
         std::vector<std::tuple<std::string, double, double>>{
             {"wide_shallow_coastal", 1.5, 700.0},
             {"oblique_asymmetric", 12.0, 650.0},
         }) {
        gf::RouteBMissionConfig mcfg;
        mcfg.domain_name = dom_name;
        mcfg.t_fork_s = t_fork;
        mcfg.max_mission_s = t_cap;
        mcfg.patrol_radius_m = 105.0;
        mcfg.v_cruise_mps = 14.0;
        mcfg.uwb_d_r_m = 600.0;
        mcfg.uwb_sigma0_m = 1.0;
        mcfg.uwb_d_max_m = 1200.0;
        mcfg.link_seed = 20261123;

        const auto paired = gf::runTwoRegimeMissionComparison(mcfg);
        std::printf(
            "[TDD-6 Regime2 %s] stay_cov=%.4f(t95=%.1f,t100=%.1f) | "
            "sw_reconf=%d(%.1fs) sw_cov=%.4f(t95=%.1f,t100=%.1f) col=%zu ref=%zu spd=%zu\n",
            dom_name.c_str(),
            paired.stay.final_certified_coverage,
            paired.stay.t95_certified_s.value_or(-1.0),
            paired.stay.t100_certified_s.value_or(-1.0),
            paired.switch_arm.reconfig_completed ? 1 : 0,
            paired.switch_arm.reconfig_complete_s,
            paired.switch_arm.final_certified_coverage,
            paired.switch_arm.t95_certified_s.value_or(-1.0),
            paired.switch_arm.t100_certified_s.value_or(-1.0),
            paired.switch_arm.audit.collision_violations,
            paired.switch_arm.audit.reference_violations,
            paired.switch_arm.audit.axis_speed_violations);
        CHECK(paired.checkpoint_byte_identical);

        CHECK(paired.stay.false_certified_count == 0);
        CHECK(paired.stay.certified_subset_of_truth);
        CHECK(paired.stay.audit.collision_violations == 0);
        CHECK(paired.stay.audit.reference_violations == 0);
        CHECK(paired.stay.audit.axis_speed_violations == 0);

        CHECK(paired.switch_arm.reconfig_completed);
        CHECK(paired.switch_arm.false_certified_count == 0);
        CHECK(paired.switch_arm.certified_subset_of_truth);
        CHECK(paired.switch_arm.static_parent_violations == 0);
        CHECK(paired.switch_arm.dag_violations == 0);
        CHECK(paired.switch_arm.audit.collision_violations == 0);
        CHECK(paired.switch_arm.audit.reference_violations == 0);
        CHECK(paired.switch_arm.audit.axis_speed_violations == 0);

        if (dom_name == "oblique_asymmetric") {
            CHECK(paired.switch_arm.final_certified_coverage >=
                  paired.stay.final_certified_coverage);
        } else {
            CHECK(paired.stay.reached_t100);
        }
    }
}
