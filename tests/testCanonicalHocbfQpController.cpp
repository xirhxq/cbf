#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CanonicalHocbfQpController.hpp"
#include "grand_finale/ProgressCompatibility.hpp"

#include <limits>

namespace {

std::vector<gf::CanonicalHardRow> quadrantRows() {
    return {
        gf::makeCanonicalGammaRow("x-nonnegative", 1, {1.0, 0.0}, 0.0),
        gf::makeCanonicalGammaRow("y-nonnegative", 1, {0.0, 1.0}, 0.0),
    };
}

gf::CanonicalQpRequest request(gf::SolverProfile profile) {
    return gf::CanonicalQpRequest{
        profile, 1, 7, 11, gf::SupervisorMode::Search,
        {-0.25, 0.5}, 1.0, quadrantRows(), 1e-7};
}

}  // namespace

TEST_CASE("Exact progress predicate accepts equality boundaries") {
    const gf::ProgressCompatibilityConfig config{0.25, 0.0, 1e-10, true};
    const auto result = gf::evaluateProgressCompatibility(
        quadrantRows(), 1, {-0.25, 0.5}, 1.0, config);
    REQUIRE(result.polytope_nonempty);
    CHECK(result.projection.x() == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(result.projection.y() == doctest::Approx(0.5).epsilon(1e-12));
    CHECK(result.projection_norm == doctest::Approx(0.25).epsilon(1e-12));
    CHECK(result.direction_inner_product == doctest::Approx(0.25));
    CHECK(result.compatible);

    const auto rejected = gf::evaluateProgressCompatibility(
        quadrantRows(), 1, {-0.25, 0.5}, 1.0,
        gf::ProgressCompatibilityConfig{
            std::nextafter(0.25, 0.0), 0.0, 0.0, true});
    CHECK_FALSE(rejected.compatible);
    CHECK(rejected.reason == "projection_norm");
}

TEST_CASE("Exact progress predicate tests direction equality and empty polytope") {
    const auto equality = gf::evaluateProgressCompatibility(
        quadrantRows(), 1, {-0.25, 0.5}, 1.0,
        gf::ProgressCompatibilityConfig{1.0, 0.8, 1e-12, true});
    CHECK(equality.direction_inner_product == doctest::Approx(0.25));
    CHECK(equality.required_direction_inner_product == doctest::Approx(0.25));
    CHECK(equality.compatible);

    auto contradictory = quadrantRows();
    contradictory.push_back(
        gf::makeCanonicalGammaRow("x-less-than-minus-one", 1, {-1.0, 0.0}, -1.1));
    const auto empty = gf::evaluateProgressCompatibility(
        contradictory, 1, {0.0, 0.0}, 1.0,
        gf::ProgressCompatibilityConfig{1.0, 0.0, 1e-12, true});
    CHECK_FALSE(empty.polytope_nonempty);
    CHECK_FALSE(empty.compatible);
    CHECK(empty.reason == "hard_polytope_empty");
}

#ifdef ENABLE_OSQP
TEST_CASE("OSQP HOCBF-QP matches exact projection and independently audits residuals") {
    gf::CanonicalHocbfQpController controller;
    const auto result = controller.solve(request(gf::SolverProfile::OpenSource));
    REQUIRE(result.solver_succeeded);
    REQUIRE(result.residual_verified);
    CHECK(result.control.x() == doctest::Approx(0.0).epsilon(1e-5));
    CHECK(result.control.y() == doctest::Approx(0.5).epsilon(1e-5));
    CHECK(result.minimum_hard_residual >= -1e-7);
    CHECK(result.exact_oracle_error <= 1e-5);
    CHECK(gf::controlMayBeApplied(result, 7, 11, gf::SupervisorMode::Search));
    CHECK_FALSE(gf::controlMayBeApplied(result, 8, 11, gf::SupervisorMode::Search));
}

TEST_CASE("OSQP controller reuses solver initialization without changing the QP result") {
    gf::CanonicalHocbfQpController controller;
    const auto cold=controller.solve(request(gf::SolverProfile::OpenSource));
    const auto steady=controller.solve(request(gf::SolverProfile::OpenSource));
    REQUIRE(cold.control_available);
    REQUIRE(steady.control_available);
    CHECK(cold.solver_cold_start);
    CHECK_FALSE(cold.solver_instance_reused);
    CHECK_FALSE(steady.solver_cold_start);
    CHECK(steady.solver_instance_reused);
    CHECK((cold.control-steady.control).norm()<=1e-10);
    CHECK(steady.exact_projection_wall_s>=0.0);
    CHECK(steady.solver_model_update_wall_s>=0.0);
    CHECK(steady.solver_solve_wall_s>=0.0);
    CHECK(steady.residual_token_audit_wall_s>=0.0);
}
#endif

#ifdef ENABLE_GUROBI
TEST_CASE("Gurobi HOCBF-QP matches exact projection and independently audits residuals") {
    gf::CanonicalHocbfQpController controller;
    const auto result = controller.solve(request(gf::SolverProfile::Gurobi));
    REQUIRE(result.solver_succeeded);
    REQUIRE(result.residual_verified);
    CHECK(result.control.x() == doctest::Approx(0.0).epsilon(1e-8));
    CHECK(result.control.y() == doctest::Approx(0.5).epsilon(1e-8));
    CHECK(result.minimum_hard_residual >= -1e-7);
    CHECK(result.exact_oracle_error <= 1e-7);
}
#endif

TEST_CASE("Empty hard polytope is a failed premise and no zero control is certified") {
    auto impossible = request(gf::SolverProfile::OpenSource);
    impossible.rows.push_back(
        gf::makeCanonicalGammaRow("x-at-most-minus-two", 1, {-1.0, 0.0}, -2.0));
    gf::CanonicalHocbfQpController controller;
    const auto result = controller.solve(impossible);
    CHECK_FALSE(result.hard_polytope_nonempty);
    CHECK_FALSE(result.residual_verified);
    CHECK_FALSE(result.control_available);
    CHECK(result.failure_reason == "hard_polytope_empty");
    CHECK_FALSE(gf::controlMayBeApplied(result, 7, 11, gf::SupervisorMode::Search));
}
