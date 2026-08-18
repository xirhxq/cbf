#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CertifiedTargetLifting.hpp"

namespace {

gf::TargetLiftRequest positive4p2() {
    gf::TargetLiftRequest request;
    request.mobile_ids={1,2,3,4};
    request.fixed_ids={100,101};
    request.reference_edges={{100,1},{101,1},{100,2},{1,2},
                             {101,3},{1,3},{100,4},{2,4}};
    request.fixed_targets={{100,{-100.0,0.0}},{101,{100.0,0.0}}};
    request.raw_targets={{1,{0.0,900.0}},{2,{900.0,900.0}},
                         {3,{-900.0,900.0}},{4,{1600.0,900.0}}};
    request.add_distance_m=849.0;
    request.target_margin_m=1.0;
    return request;
}

}

TEST_CASE("4+2 fixed DAG has deterministic nonempty strict target lift") {
    const auto first=gf::liftReferenceFeasibleTargets(positive4p2());
    const auto second=gf::liftReferenceFeasibleTargets(positive4p2());
    REQUIRE(first.valid);
    REQUIRE(second.valid);
    CHECK(first.reason=="certified_target_lift");
    CHECK(first.r_plan_m==doctest::Approx(848.0));
    CHECK(first.targets.size()==6);
    CHECK(first.topological_order==std::vector<gf::NodeId>{100,101,1,2,3,4});
    CHECK(first.canonical_digest==second.canonical_digest);
    for (const auto& edge : positive4p2().reference_edges) {
        const double distance=(first.targets.at(edge.owner)-
                               first.targets.at(edge.reference)).norm();
        CHECK(distance<=first.r_plan_m+1e-9);
    }
}

TEST_CASE("Disc projection exposes strict planning-margin boundary") {
    const std::vector<gf::TargetDisc2D> discs={
        {{0.0,0.0},848.0,"100->1"},
        {{0.0,10.0},848.0,"101->1"}};
    const auto result=gf::projectOntoTargetLens({2000.0,5.0},discs,1e-9);
    REQUIRE(result.valid);
    CHECK(result.point.x()==doctest::Approx(
        std::sqrt(848.0*848.0-25.0)).epsilon(1e-10));
    CHECK(result.maximum_constraint_violation_m<=1e-9);
}

TEST_CASE("Separated reference discs produce lens-empty rejection") {
    auto request=positive4p2();
    request.fixed_targets.at(101)={2000.0,0.0};
    const auto result=gf::liftReferenceFeasibleTargets(request);
    CHECK_FALSE(result.valid);
    CHECK(result.reason=="lens_empty:owner=1");
}

TEST_CASE("Cyclic candidate is rejected before recursive lifting") {
    auto request=positive4p2();
    request.reference_edges={{100,1},{2,1},{101,2},{1,2},
                             {100,3},{1,3},{101,4},{2,4}};
    const auto result=gf::liftReferenceFeasibleTargets(request);
    CHECK_FALSE(result.valid);
    CHECK(result.reason=="target_graph_cycle");
}

TEST_CASE("Heterogeneous fixed and mobile references use processed targets") {
    const auto result=gf::liftReferenceFeasibleTargets(positive4p2());
    REQUIRE(result.valid);
    CHECK((result.targets.at(2)-result.targets.at(1)).norm()<=848.0+1e-9);
    CHECK((result.targets.at(2)-result.targets.at(100)).norm()<=848.0+1e-9);
    CHECK((result.targets.at(4)-result.targets.at(2)).norm()<=848.0+1e-9);
    CHECK((result.targets.at(4)-result.targets.at(100)).norm()<=848.0+1e-9);
}

TEST_CASE("Roundoff budget cannot consume the strict add-distance margin") {
    auto request=positive4p2();
    request.target_margin_m=5e-11;
    request.feasibility_tolerance_m=1e-10;
    const auto rejected=gf::liftReferenceFeasibleTargets(request);
    CHECK_FALSE(rejected.valid);
    CHECK(rejected.reason=="invalid_target_lift_request");
}
