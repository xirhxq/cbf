#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11rFailureWindow.hpp"

TEST_CASE("failure-window audit selects limiting owner and exact minimal conflict") {
    std::vector<gf::CanonicalHardRow> rows;
    gf::CanonicalHardRow left;
    left.id="collision:1:2:left";
    left.kind=gf::CanonicalHardRowKind::Collision;
    left.owner=1;
    left.peer=2;
    left.control_coefficient={1.0,0.0};
    left.constant=-3.0;
    rows.push_back(left);
    gf::CanonicalHardRow right=left;
    right.id="collision:1:3:right";
    right.peer=3;
    right.control_coefficient={-1.0,0.0};
    rows.push_back(right);
    gf::CanonicalHardRow feasible=left;
    feasible.id="collision:2:1";
    feasible.owner=2;
    feasible.peer=1;
    feasible.control_coefficient={1.0,0.0};
    feasible.constant=1.0;
    rows.push_back(feasible);

    const auto audit=gf::auditTask10p11rFailureSnapshot(
        {1,2},12.34,gf::SupervisorMode::Search,{{101,1},{1,2}},rows,4.0);
    REQUIRE(audit.valid);
    CHECK(audit.limiting_owner==1);
    CHECK(audit.current_gamma==doctest::Approx(-3.0));
    CHECK_FALSE(audit.hard_polytope.exact_feasible);
    CHECK(audit.hard_polytope.minimal_conflict_row_ids==
          std::vector<std::string>{"collision:1:2:left",
                                   "collision:1:3:right"});
}

TEST_CASE("failure-window audit remains diagnostic for a feasible snapshot") {
    gf::CanonicalHardRow row;
    row.id="reference:1:101";
    row.kind=gf::CanonicalHardRowKind::ReferenceDistance;
    row.owner=1;
    row.peer=101;
    row.control_coefficient={1.0,0.0};
    row.constant=0.25;
    const auto audit=gf::auditTask10p11rFailureSnapshot(
        {1},0.0,gf::SupervisorMode::Search,{{101,1}},{row},4.0);
    REQUIRE(audit.valid);
    CHECK(audit.limiting_owner==1);
    CHECK(audit.current_gamma>0.0);
    CHECK(audit.hard_polytope.exact_feasible);
    CHECK(audit.hard_polytope.minimal_conflict_row_ids.empty());
}
