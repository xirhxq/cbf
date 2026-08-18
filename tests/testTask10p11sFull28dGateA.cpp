#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11sFull28dGateA.hpp"

TEST_CASE("same-half expansion remains block local") {
    std::vector<gf::CanonicalHardRow> rows;
    for (gf::NodeId owner = 1; owner <= 14; ++owner) {
        gf::CanonicalHardRow row;
        row.id = "input:" + std::to_string(owner);
        row.owner = owner;
        row.kind = gf::CanonicalHardRowKind::InputBox;
        row.control_coefficient = {1.0, 0.0};
        row.constant = 4.0;
        rows.push_back(row);
    }
    std::vector<gf::NodeId> ids;
    for (gf::NodeId id = 1; id <= 14; ++id) ids.push_back(id);
    const auto expanded = gf::buildTask10p11sRows28d(rows, ids, false);
    REQUIRE(expanded.rows.size() == 14);
    for (const auto& row : expanded.rows) {
        CHECK((row.coefficient.array() != 0.0).count() == 1);
        CHECK_FALSE(row.coupled_mobile_pair);
    }
}

TEST_CASE("full-pair expansion joins two halves and counts reserve once") {
    gf::CanonicalHardRow first;
    first.id = "reference:2->4:owner:2";
    first.kind = gf::CanonicalHardRowKind::ReferenceDistance;
    first.owner = 2;
    first.peer = 4;
    first.control_coefficient = {-1.0, 0.5};
    first.constant = 1.5;
    first.responsibility = 0.5;
    first.coefficient_uncertainty_reserve = 0.25;
    gf::CanonicalHardRow second = first;
    second.id = "reference:2->4:owner:4";
    second.owner = 4;
    second.peer = 2;
    second.control_coefficient = -first.control_coefficient;

    std::vector<gf::NodeId> ids;
    for (gf::NodeId id = 1; id <= 14; ++id) ids.push_back(id);
    const auto coupled = gf::buildTask10p11sRows28d(
        {first, second}, ids, true);
    REQUIRE(coupled.rows.size() == 1);
    CHECK(coupled.coupled_mobile_pair_count == 1);
    CHECK(coupled.rows.front().constant == doctest::Approx(3.25));
    CHECK(coupled.rows.front().uncertainty_reserve_counted_once ==
          doctest::Approx(0.25));
    Eigen::VectorXd controls = Eigen::VectorXd::Zero(28);
    controls.segment<2>(2) = Eigen::Vector2d{2.0, 0.0};
    controls.segment<2>(6) = Eigen::Vector2d{-1.0, 0.0};
    CHECK(coupled.rows.front().residual(controls) == doctest::Approx(0.25));
}

TEST_CASE("incoherent peer halves are rejected") {
    gf::CanonicalHardRow first;
    first.id = "collision:1--2:owner:1";
    first.kind = gf::CanonicalHardRowKind::Collision;
    first.owner = 1;
    first.peer = 2;
    first.control_coefficient = {1.0, 0.0};
    first.constant = 1.0;
    first.responsibility = 0.5;
    gf::CanonicalHardRow second = first;
    second.id = "collision:1--2:owner:2";
    second.owner = 2;
    second.peer = 1;

    std::vector<gf::NodeId> ids;
    for (gf::NodeId id = 1; id <= 14; ++id) ids.push_back(id);
    CHECK_THROWS(gf::buildTask10p11sRows28d({first, second}, ids, true));
}
