#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task15ReferenceGovernor.hpp"

namespace {
std::map<gf::NodeId,Eigen::Vector2d> ledger(double x) {
    std::map<gf::NodeId,Eigen::Vector2d> value;
    for (gf::NodeId id=1;id<=14;++id) value[id]={x+id,2.0*x-id};
    return value;
}
}

TEST_CASE("Task 15 governor selects the largest registered safe fraction") {
    const auto result=gf::task15AdvanceReferenceGovernor(
        ledger(0.0),ledger(100.0),
        [](const auto&,double fraction) { return fraction<=0.25; });
    REQUIRE(result.valid);
    CHECK_FALSE(result.reselect_required);
    CHECK(result.common_fraction==doctest::Approx(0.25));
    CHECK(result.targets.at(7).isApprox(
        ledger(0.0).at(7)+0.25*(ledger(100.0).at(7)-ledger(0.0).at(7))));
}

TEST_CASE("Task 15 governor requests reselection when no positive step is safe") {
    const auto result=gf::task15AdvanceReferenceGovernor(
        ledger(0.0),ledger(100.0),[](const auto&,double) { return false; });
    CHECK_FALSE(result.valid);
    CHECK(result.reselect_required);
    CHECK(result.reason=="no_positive_reference_safe_step");
}
