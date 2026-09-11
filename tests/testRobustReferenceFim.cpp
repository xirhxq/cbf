#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/RobustReferenceFim.hpp"

#include <Eigen/Dense>

#include <map>
#include <string>
#include <vector>

namespace {

gf::JointEstimateSnapshot snapshot() {
    Eigen::VectorXd mean(8);
    mean << 8.0, 8.0, 0.0, 0.0,
            16.0, 8.0, 0.0, 0.0;
    return {{1, 2}, mean, 0.006922555446745518 *
        Eigen::MatrixXd::Identity(8, 8),
        {{100, {2.0, 2.0}}, {102, {38.0, 2.0}}}};
}

std::map<std::string, double> variances() {
    return {{"1--2", 1.0}, {"2--100", 1.0}, {"2--102", 1.0}};
}

}  // namespace

TEST_CASE("Robust cone FIM rejects the frozen owner two topology at terminal support") {
    const std::vector<gf::DirectedEdge> old_edges{{100, 2}, {1, 2}};
    const std::map<std::string, double> supports{
        {"100->2", 2.8853201331533249},
        {"1->2", 4.1051145121802746}};
    const auto robust = gf::robustReferenceFimLowerBound(
        2, old_edges, snapshot(), variances(), supports);
    REQUIRE(robust.valid);
    CHECK(robust.lower_eigenvalue == doctest::Approx(0.0).epsilon(1.0e-12));

    const double nominal = Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
        gf::referenceFim(2, old_edges, snapshot(), variances()))
        .eigenvalues().minCoeff();
    CHECK(nominal > 1.0e-6);
}

TEST_CASE("Fixed 100 and 102 preserve a positive robust FIM and certify the union") {
    const std::map<std::string, double> supports{
        {"100->2", 2.8853201331533249},
        {"1->2", 4.1051145121802746},
        {"102->2", 2.8853201331533249}};
    const auto successor = gf::robustReferenceFimLowerBound(
        2, {{100, 2}, {102, 2}}, snapshot(), variances(), supports);
    REQUIRE(successor.valid);
    CHECK(successor.lower_eigenvalue ==
          doctest::Approx(0.0618996).epsilon(1.0e-5));
    CHECK(successor.lower_eigenvalue > 1.0e-6);

    const auto union_state = gf::robustReferenceFimLowerBound(
        2, {{100, 2}, {1, 2}, {102, 2}},
        snapshot(), variances(), supports);
    REQUIRE(union_state.valid);
    CHECK(union_state.lower_eigenvalue >= successor.lower_eigenvalue - 1.0e-12);
    CHECK(union_state.first_reference_edge_id == "100->2");
    CHECK(union_state.second_reference_edge_id == "102->2");
}

TEST_CASE("Robust FIM fails closed on missing support or an unbounded direction cone") {
    const std::vector<gf::DirectedEdge> edges{{100, 2}, {102, 2}};
    const auto missing = gf::robustReferenceFimLowerBound(
        2, edges, snapshot(), variances(), {{"100->2", 0.1}});
    CHECK_FALSE(missing.valid);
    CHECK(missing.reason == "direction_support_missing");

    const auto unbounded = gf::robustReferenceFimLowerBound(
        2, edges, snapshot(), variances(),
        {{"100->2", 100.0}, {"102->2", 100.0}});
    REQUIRE(unbounded.valid);
    CHECK(unbounded.lower_eigenvalue == doctest::Approx(0.0));
    CHECK(unbounded.reason == "direction_cone_unbounded");
}
