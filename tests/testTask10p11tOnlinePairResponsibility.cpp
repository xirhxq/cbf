#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11tOnlinePairResponsibility.hpp"

namespace {

std::vector<gf::NodeId> mobileIds() {
    std::vector<gf::NodeId> ids;
    for (gf::NodeId owner = 1; owner <= 14; ++owner) ids.push_back(owner);
    return ids;
}

void addBenignOwnerRows(std::vector<gf::CanonicalHardRow>& rows) {
    for (gf::NodeId owner = 1; owner <= 14; ++owner) {
        gf::CanonicalHardRow row;
        row.id = "benign:" + std::to_string(owner);
        row.owner = owner;
        row.control_coefficient = {1.0, 0.0};
        row.constant = 4.0;
        rows.push_back(row);
    }
}

void addConflictPair(
    std::vector<gf::CanonicalHardRow>& rows,
    gf::NodeId first_owner, gf::NodeId second_owner) {
    const std::string base = "reference:" + std::to_string(first_owner) +
        "->" + std::to_string(second_owner);
    gf::CanonicalHardRow first;
    first.id = base + ":owner:" + std::to_string(first_owner);
    first.kind = gf::CanonicalHardRowKind::ReferenceDistance;
    first.owner = first_owner;
    first.peer = second_owner;
    first.control_coefficient = {1.0, 0.0};
    first.constant = 0.5;
    first.responsibility = 0.5;
    first.coefficient_uncertainty_reserve = 0.5;
    rows.push_back(first);
    auto second = first;
    second.id = base + ":owner:" + std::to_string(second_owner);
    second.owner = second_owner;
    second.peer = first_owner;
    second.control_coefficient = {-1.0, 0.0};
    rows.push_back(second);

    gf::CanonicalHardRow blocker;
    blocker.id = "fixed:" + std::to_string(first_owner);
    blocker.owner = first_owner;
    blocker.control_coefficient = {-1.0, 0.0};
    blocker.constant = -1.6;
    rows.push_back(blocker);
}

}  // namespace

TEST_CASE("online diagnostic identifies one coherent conflict pair without scanning") {
    std::vector<gf::CanonicalHardRow> rows;
    addBenignOwnerRows(rows);
    addConflictPair(rows, 2, 4);
    const auto diagnostic = gf::diagnoseTask10p11tOnlineConflicts(
        rows, mobileIds(), 132.4, gf::SupervisorMode::Search,
        {{2, 4}}, 4.0);

    REQUIRE(diagnostic.valid);
    REQUIRE(diagnostic.infeasible);
    CHECK(diagnostic.mobile_pair_base_ids ==
          std::vector<std::string>{"reference:2->4"});
    CHECK(diagnostic.limiting_owner == 2);
}

TEST_CASE("two distinct pair conflicts are reported and cannot be selected") {
    std::vector<gf::CanonicalHardRow> rows;
    addBenignOwnerRows(rows);
    addConflictPair(rows, 2, 4);
    addConflictPair(rows, 5, 6);
    const auto diagnostic = gf::diagnoseTask10p11tOnlineConflicts(
        rows, mobileIds(), 132.4, gf::SupervisorMode::Search,
        {{2, 4}, {5, 6}}, 4.0);

    REQUIRE(diagnostic.valid);
    REQUIRE(diagnostic.infeasible);
    CHECK(diagnostic.mobile_pair_base_ids.size() == 2);
    CHECK_FALSE(gf::task10p11tUniqueOnlinePair(diagnostic).has_value());
}
