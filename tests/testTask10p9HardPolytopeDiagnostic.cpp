#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/ExactHardPolytopeDiagnostic.hpp"

namespace {
gf::CanonicalHardRow row(
    std::string id, const Eigen::Vector2d& a, double c,
    gf::CanonicalHardRowKind kind = gf::CanonicalHardRowKind::Auxiliary) {
    gf::CanonicalHardRow result;
    result.id = std::move(id);
    result.kind = kind;
    result.owner = 1;
    result.normal = a;
    result.control_coefficient = a;
    result.constant = c;
    result.barrier_h = 0.5;
    result.barrier_psi1 = 0.25;
    result.position_uncertainty_reserve_m = 0.02;
    result.velocity_uncertainty_reserve_mps = 0.03;
    result.coefficient_uncertainty_reserve = 0.04;
    return result;
}
}

TEST_CASE("Task 10.9 exact diagnostic accepts a feasible owner polytope") {
    const std::vector<gf::CanonicalHardRow> rows{
        row("x-lower", Eigen::Vector2d::UnitX(), 0.2),
        row("y-lower", Eigen::Vector2d::UnitY(), 0.2)};
    const auto certificate = gf::diagnoseHardPolytope(
        1, 2.5, gf::SupervisorMode::Search, {}, rows, 0.4);
    CHECK(certificate.exact_feasible);
    CHECK(certificate.minimal_conflict_row_ids.empty());
    CHECK(certificate.input_half_box == doctest::Approx(0.4));
}

TEST_CASE("Task 10.9 exact diagnostic returns irreducible deterministic conflict") {
    const std::vector<gf::CanonicalHardRow> rows{
        row("b:x-upper", -Eigen::Vector2d::UnitX(), -0.3),
        row("a:x-lower", Eigen::Vector2d::UnitX(), -0.3),
        row("irrelevant", Eigen::Vector2d::UnitY(), 1.0)};
    const auto certificate = gf::diagnoseHardPolytope(
        1, 3.0, gf::SupervisorMode::Hold, {{100, 1}, {101, 1}},
        rows, 0.4);
    CHECK_FALSE(certificate.exact_feasible);
    REQUIRE(certificate.minimal_conflict_row_ids.size() == 2);
    CHECK(certificate.minimal_conflict_row_ids[0] == "a:x-lower");
    CHECK(certificate.minimal_conflict_row_ids[1] == "b:x-upper");
    REQUIRE(certificate.rows.size() == 3);
    CHECK(certificate.rows[0].position_reserve_m == doctest::Approx(0.02));
    CHECK(certificate.rows[0].velocity_reserve_mps == doctest::Approx(0.03));
    CHECK(certificate.rows[0].coefficient_reserve == doctest::Approx(0.04));
    CHECK(certificate.mode == gf::SupervisorMode::Hold);
}

TEST_CASE("Task 10.9 exact diagnostic identifies one row conflicting with input box") {
    const std::vector<gf::CanonicalHardRow> rows{
        row("requires-ax-one", Eigen::Vector2d::UnitX(), -1.0)};
    const auto certificate = gf::diagnoseHardPolytope(
        1, 4.0, gf::SupervisorMode::Search, {}, rows, 0.4);
    CHECK_FALSE(certificate.exact_feasible);
    REQUIRE(certificate.minimal_conflict_row_ids.size() == 2);
    CHECK(certificate.minimal_conflict_row_ids[0] == "$input_box");
    CHECK(certificate.minimal_conflict_row_ids[1] == "requires-ax-one");
}
