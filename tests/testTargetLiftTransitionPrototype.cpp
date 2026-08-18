#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/TargetLiftTransitionPrototype.hpp"

namespace {

gf::TargetGeometryGateRequest geometryRequest() {
    gf::TargetGeometryGateRequest request;
    request.mobile_ids={1,2};
    request.fixed_ids={100,101};
    request.reference_edges={{100,1},{101,1},{100,2},{1,2}};
    request.targets={{100,{-100.0,0.0}},{101,{100.0,0.0}},
                     {1,{0.0,200.0}},{2,{300.0,300.0}}};
    request.raw_targets={{1,{0.0,210.0}},{2,{310.0,300.0}}};
    request.position_support_m={{1,1.0},{2,1.0},{100,0.0},{101,0.0}};
    request.range_variances_m2={{"1--100",100.0},{"1--101",100.0},
                                {"2--100",100.0},{"1--2",100.0}};
    request.edge_information_valid={{"100->1",true},{"101->1",true},
                                    {"100->2",true},{"1->2",true}};
    request.posterior_valid={{1,true},{2,true}};
    request.minimum_collision_distance_m=30.0;
    request.minimum_target_fim_planning_score=1e-5;
    request.minimum_target_cone_planning_score=1e-6;
    request.maximum_target_deformation_m=100.0;
    return request;
}

gf::TargetLiftTransitionRequest unionRequest() {
    gf::TargetLiftTransitionRequest request;
    request.mobile_ids={1};
    request.fixed_ids={100,101,102};
    request.old_edges={{100,1},{101,1}};
    request.successor_edges={{100,1},{102,1}};
    request.raw_targets={{1,{0.0,900.0}}};
    request.fixed_targets={{100,{-100.0,0.0}},{101,{100.0,0.0}},
                           {102,{0.0,100.0}}};
    request.add_distance_m=849.0;
    request.target_margin_m=1.0;
    request.topology_version=7;
    request.estimator_version=11;
    request.raw_ledger_version=3;
    request.config_version=5;
    return request;
}

}

TEST_CASE("Exact target gates accept separated informative geometry") {
    const auto audit=gf::evaluateTargetGeometryGates(geometryRequest());
    REQUIRE(audit.valid);
    CHECK(audit.minimum_target_collision_distance_m>30.0);
    CHECK(audit.minimum_target_fim_planning_score>=1e-5);
    CHECK(audit.minimum_target_cone_planning_score>=1e-6);
}

TEST_CASE("Target collision and FIM degeneracy are exact rejections") {
    auto collision=geometryRequest();
    collision.targets.at(2)={1.0,200.0};
    CHECK(gf::evaluateTargetGeometryGates(collision).reason=="target_collision");

    auto fim=geometryRequest();
    fim.targets.at(100)={-100.0,200.0};
    fim.targets.at(101)={100.0,200.0};
    fim.targets.at(1)={0.0,200.0};
    CHECK(gf::evaluateTargetGeometryGates(fim).reason==
          "target_fim_planning_score");

    auto robust=geometryRequest();
    robust.position_support_m.at(1)=150.0;
    CHECK(gf::evaluateTargetGeometryGates(robust).reason==
          "target_cone_planning_score");

    auto fixed_collision=geometryRequest();
    fixed_collision.targets.at(1)=fixed_collision.targets.at(100);
    CHECK(gf::evaluateTargetGeometryGates(fixed_collision).reason==
          "target_collision");
}

TEST_CASE("Union lift simultaneously satisfies old new and successor edges") {
    const auto frozen=gf::freezeUnionTargetLift(unionRequest());
    REQUIRE(frozen.valid);
    CHECK(frozen.r_plan_m==doctest::Approx(848.0));
    CHECK(frozen.union_edges.size()==3);
    CHECK(frozen.completed_union_cycles==0);
    for (const auto& edge : frozen.union_edges)
        CHECK((frozen.lifted_targets.at(edge.owner)-
               frozen.lifted_targets.at(edge.reference)).norm()<=848.0+1e-9);
    CHECK(gf::successorTargetCompatible(frozen,1e-9));
}

TEST_CASE("Successor-only lens can be nonempty while union lens is empty") {
    gf::TargetLiftTransitionRequest request;
    request.mobile_ids={1};
    request.fixed_ids={100,101,102,103};
    request.old_edges={{100,1},{101,1}};
    request.successor_edges={{102,1},{103,1}};
    request.raw_targets={{1,{2.0,0.0}}};
    request.fixed_targets={{100,{-2.0,0.0}},{101,{-2.0,0.0}},
                           {102,{2.0,0.0}},{103,{2.0,0.0}}};
    request.add_distance_m=1.1;
    request.target_margin_m=0.1;
    auto successor=request;
    successor.old_edges={};
    const auto successor_only=gf::freezeUnionTargetLift(successor);
    REQUIRE(successor_only.valid);
    const auto union_lift=gf::freezeUnionTargetLift(request);
    CHECK_FALSE(union_lift.valid);
    CHECK(union_lift.reason=="lens_empty:owner=1");
}

TEST_CASE("Frozen union target survives T100 bookkeeping and one full cycle") {
    const auto frozen=gf::freezeUnionTargetLift(unionRequest());
    REQUIRE(frozen.valid);
    const auto after_t100=gf::recordT100StatisticOnly(frozen,3.9);
    CHECK(after_t100.lifted_digest==frozen.lifted_digest);
    CHECK(after_t100.raw_ledger_version==frozen.raw_ledger_version);
    const auto cycled=gf::recordFreshUnionControlCycle(after_t100,
        {gf::SupervisorMode::Union,after_t100.union_edges,
         after_t100.lifted_digest,after_t100.raw_ledger_version,
         after_t100.raw_ledger_digest,after_t100.config_version,
         after_t100.config_digest,8,11,12,true,true,0.0,1e-9});
    REQUIRE(cycled.valid);
    CHECK(cycled.completed_union_cycles==1);
    CHECK(cycled.estimator_version==12);
    CHECK(gf::freshBreakTargetReady(cycled,8,12));
    CHECK_FALSE(gf::freshBreakTargetReady(cycled,7,12));

    auto forged=after_t100;
    forged=gf::recordFreshUnionControlCycle(forged,
        {gf::SupervisorMode::Union,forged.union_edges,forged.lifted_digest,
         forged.raw_ledger_version,forged.raw_ledger_digest+1,
         forged.config_version,forged.config_digest,8,11,12,true,true,0.0,1e-9});
    CHECK_FALSE(forged.valid);
}

TEST_CASE("Information and progress failures reject without changing lift") {
    auto information=geometryRequest();
    information.edge_information_valid.at("100->2")=false;
    CHECK(gf::evaluateTargetGeometryGates(information).reason==
          "target_information:edge=100->2");
    auto progress=geometryRequest();
    progress.maximum_target_deformation_m=1.0;
    CHECK(gf::evaluateTargetGeometryGates(progress).reason==
          "target_deformation_budget");
}
