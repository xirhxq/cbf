#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/GrandFinaleExperiment.hpp"

TEST_CASE("The 4+2 estimator-in-loop chain reconfigures and reaches certified T100") {
    const auto summary = gf::GrandFinaleExperiment::runCanonical4p2(
        gf::SolverProfile::OpenSource, false, 2027);

    CHECK(summary.mobile_count == 4);
    CHECK(summary.fixed_count == 2);
    CHECK(summary.estimator_updates > 0);
    CHECK(summary.second_order_control_steps > 0);
    CHECK(summary.successful_reconfigurations == 1);
    CHECK(summary.visited_union_state);
    CHECK(summary.certified_coverage_final > summary.certified_coverage_initial);
    CHECK(summary.reached_certified_t100);
    CHECK(summary.minimum_reference_count >= 2);
    CHECK(summary.maximum_robust_reference_distance_m <= 850.0);
    CHECK(summary.minimum_gamma_star > 0.0);
    CHECK(summary.minimum_hard_margin >= 0.0);
    CHECK(summary.topology_version == 3);
    CHECK(summary.hard_gate_violations == 0);
}

#ifdef ENABLE_GUROBI
TEST_CASE("The Gurobi profile closes the same 4+2 research chain") {
    const auto summary = gf::GrandFinaleExperiment::runCanonical4p2(
        gf::SolverProfile::Gurobi, false, 2027);
    CHECK(summary.successful_reconfigurations == 1);
    CHECK(summary.visited_union_state);
    CHECK(summary.reached_certified_t100);
    CHECK(summary.hard_gate_violations == 0);
}
#endif

TEST_CASE("An uncertifiable candidate enters RETREAT or HOLD without hard-gate violation") {
    const auto summary = gf::GrandFinaleExperiment::runCanonical4p2(
        gf::SolverProfile::OpenSource, true, 2027);

    CHECK(summary.successful_reconfigurations == 0);
    CHECK((summary.final_mode == gf::SupervisorMode::Retreat ||
           summary.final_mode == gf::SupervisorMode::Hold));
    CHECK(summary.minimum_reference_count >= 2);
    CHECK(summary.maximum_robust_reference_distance_m <= 850.0);
    CHECK(summary.hard_gate_violations == 0);
}

TEST_CASE("The canonical 14+3 interface completes a deterministic short smoke") {
    const auto summary = gf::GrandFinaleExperiment::runScaleSmoke14p3(2027, 3);

    CHECK(summary.mobile_count == 14);
    CHECK(summary.fixed_count == 3);
    CHECK(summary.second_order_control_steps == 42);
    CHECK(summary.estimator_updates > 0);
    CHECK(summary.hard_gate_violations == 0);
}
