#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11lSNearBoundaryFamily.hpp"

TEST_CASE("Task 10.11l-S frozen near-boundary family has complete exact classes") {
    const auto cases=gf::task10p11lSNearBoundaryFamily();
    REQUIRE(cases.size()==6);
    CHECK(cases[0].id=="clear_positive_margin");
    CHECK(cases[1].id=="near_zero_positive");
    CHECK(cases[2].id=="exact_zero_boundary");
    CHECK(cases[3].id=="exact_negative_infeasible");
    CHECK(cases[4].id=="single_active_row");
    CHECK(cases[5].id=="multi_active_collision_reference_input");
    for (const auto& item : cases) {
        const auto exact=gf::evaluateTask10p11lSExactCase(item);
        CHECK(exact.feasible==item.exact_feasible);
        if (item.expected_gamma_mps2.has_value()) {
            CHECK(exact.gamma_mps2==doctest::Approx(
                *item.expected_gamma_mps2).epsilon(1e-9));
        }
    }
}
TEST_CASE("Task 10.11l-S one OSQP configuration matches exact and Gurobi") {
    gf::CanonicalHocbfQpController controller;
    for (const auto& item : gf::task10p11lSNearBoundaryFamily()) {
        CAPTURE(item.id);
        const auto exact=gf::evaluateTask10p11lSExactCase(item);
        auto open_request=item.request;
        open_request.profile=gf::SolverProfile::OpenSource;
        auto gurobi_request=item.request;
        gurobi_request.profile=gf::SolverProfile::Gurobi;
        const auto cold=controller.solve(open_request);
        const auto reused=controller.solve(open_request);
        const auto gurobi=controller.solve(gurobi_request);
        CHECK(cold.hard_polytope_nonempty==exact.feasible);
        CHECK(reused.hard_polytope_nonempty==exact.feasible);
        CHECK(gurobi.hard_polytope_nonempty==exact.feasible);
        if (!exact.feasible) {
            CHECK_FALSE(cold.control_available);
            CHECK(cold.failure_reason=="hard_polytope_empty");
            continue;
        }
        REQUIRE(cold.control_available);
        REQUIRE(reused.control_available);
        REQUIRE(gurobi.control_available);
        CHECK(cold.solver_status=="optimal");
        CHECK(reused.solver_status=="optimal");
        CHECK(cold.minimum_hard_residual>=-item.request.residual_tolerance);
        CHECK(reused.minimum_hard_residual>=-item.request.residual_tolerance);
        CHECK((cold.control-exact.control).norm()<=1e-6);
        CHECK((reused.control-exact.control).norm()<=1e-6);
        CHECK((gurobi.control-exact.control).norm()<=1e-7);
        CHECK((cold.control-reused.control).norm()<=1e-9);
    }
}
