#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11tDynamicPairResponsibility.hpp"

namespace {

std::vector<gf::NodeId> mobileIds() {
    std::vector<gf::NodeId> ids;
    for (gf::NodeId id = 1; id <= 14; ++id) ids.push_back(id);
    return ids;
}

gf::CanonicalHardRow halfRow(gf::NodeId owner, gf::NodeId peer,
                             const Eigen::Vector2d& coefficient) {
    gf::CanonicalHardRow row;
    row.id = "reference:2->4:owner:" + std::to_string(owner);
    row.kind = gf::CanonicalHardRowKind::ReferenceDistance;
    row.owner = owner;
    row.peer = peer;
    row.control_coefficient = coefficient;
    row.constant = 0.5;
    row.responsibility = 0.5;
    row.coefficient_uncertainty_reserve = 0.5;
    return row;
}

}  // namespace

TEST_CASE("signed transfer retains both local reserves and implies full-pair safety") {
    const auto first = halfRow(2, 4, {1.0, 0.0});
    auto second = halfRow(4, 2, {-1.0, 0.0});
    const auto pair = gf::makeTask10p11tPairRows(first, second);

    CHECK(pair.central_constant_lower == doctest::Approx(2.0));
    CHECK(pair.coefficient_reserve_per_owner == doctest::Approx(0.5));

    const double transfer = 0.25;
    const auto dynamic = gf::task10p11tDynamicConstants(pair, transfer);
    CHECK(dynamic.first == doctest::Approx(0.75));
    CHECK(dynamic.second == doctest::Approx(0.25));
    CHECK(gf::task10p11tFullPairResidualFromLocalSum(
              pair, 0.4, 0.6) == doctest::Approx(1.5));
}

TEST_CASE("zero transfer exactly recovers frozen canonical half rows") {
    const auto first = halfRow(2, 4, {1.0, 0.0});
    const auto second = halfRow(4, 2, {-1.0, 0.0});
    const auto pair = gf::makeTask10p11tPairRows(first, second);
    const auto constants = gf::task10p11tDynamicConstants(pair, 0.0);

    CHECK(constants.first == doctest::Approx(first.constant));
    CHECK(constants.second == doctest::Approx(second.constant));
}

TEST_CASE("pair-only coordinator finds a non-half responsibility and local replay") {
    std::vector<gf::CanonicalHardRow> rows;
    auto first = halfRow(2, 4, {1.0, 0.0});
    auto second = halfRow(4, 2, {-1.0, 0.0});
    rows.push_back(first);
    rows.push_back(second);

    gf::CanonicalHardRow first_other;
    first_other.id = "fixed:first";
    first_other.owner = 2;
    first_other.control_coefficient = {-1.0, 0.0};
    first_other.constant = -1.6;
    rows.push_back(first_other);

    gf::CanonicalHardRow second_other = first_other;
    second_other.id = "fixed:second";
    second_other.owner = 4;
    second_other.control_coefficient = {1.0, 0.0};
    second_other.constant = 2.0;
    rows.push_back(second_other);

    std::map<gf::NodeId, Eigen::Vector2d> nominal{
        {2, Eigen::Vector2d::Zero()}, {4, Eigen::Vector2d::Zero()}};
    const auto result = gf::solveTask10p11tDynamicPair(
        rows, mobileIds(), nominal, 4.0, "reference:2->4");

    REQUIRE(result.valid);
    REQUIRE(result.feasible);
    CHECK(result.legacy_half_first_gamma < 0.0);
    CHECK(result.selected_transfer_mps2 > 0.0);
    CHECK_FALSE(result.bounded_fraction_feasible);
    CHECK(result.first_local_replay_feasible);
    CHECK(result.second_local_replay_feasible);
    CHECK(result.minimum_independent_local_residual >= -1e-8);
    CHECK(result.full_pair_residual >= -1e-8);
    CHECK(result.dynamic_local_sum_residual <=
          result.full_pair_residual + 1e-12);
}

TEST_CASE("only one coherent mobile row pair is accepted") {
    auto first = halfRow(2, 4, {1.0, 0.0});
    auto second = halfRow(4, 2, {-1.0, 0.0});
    second.coefficient_uncertainty_reserve += 0.1;
    CHECK_THROWS(gf::makeTask10p11tPairRows(first, second));
}

TEST_CASE("distributed step solves all fourteen local QPs around one pair token") {
    std::vector<gf::CanonicalHardRow> rows;
    auto first = halfRow(2, 4, {1.0, 0.0});
    auto second = halfRow(4, 2, {-1.0, 0.0});
    rows.push_back(first);
    rows.push_back(second);

    gf::CanonicalHardRow first_other;
    first_other.id = "fixed:first";
    first_other.owner = 2;
    first_other.control_coefficient = {-1.0, 0.0};
    first_other.constant = -1.6;
    rows.push_back(first_other);

    gf::CanonicalHardRow second_other = first_other;
    second_other.id = "fixed:second";
    second_other.owner = 4;
    second_other.control_coefficient = {1.0, 0.0};
    second_other.constant = 2.0;
    rows.push_back(second_other);

    std::map<gf::NodeId, Eigen::Vector2d> nominal;
    for (gf::NodeId owner = 1; owner <= 14; ++owner) {
        nominal.emplace(owner, Eigen::Vector2d::Zero());
        gf::CanonicalHardRow lower;
        lower.id = "local:" + std::to_string(owner);
        lower.owner = owner;
        lower.control_coefficient = {1.0, 0.0};
        lower.constant = 4.0;
        rows.push_back(lower);
    }

    const auto result = gf::solveTask10p11tDistributedLocalStep(
        rows, mobileIds(), nominal, 4.0, "reference:2->4");

    REQUIRE(result.valid);
    REQUIRE(result.feasible);
    CHECK(result.controls.size() == 14);
    CHECK(result.owner_results.size() == 14);
    CHECK(result.pair.selected_transfer_mps2 > 0.0);
    CHECK(result.minimum_local_residual >= -1e-8);
}
