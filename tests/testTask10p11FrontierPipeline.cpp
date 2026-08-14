#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/SharedFrontierAllocationPipeline.hpp"

namespace {

gf::SharedFrontierPipelineRequest request() {
    gf::SharedFrontierPipelineRequest request;
    request.allocation.snapshot_token=1;
    request.allocation.topology_token=1;
    request.allocation.grid_token=1;
    request.allocation.agents={{1,{0.0,0.0},{0.0,0.0}}};
    request.allocation.cells={
        {0,0,{10.0,0.0}},
        {1,0,{0.0,10.0}}};
    request.allocation.acceleration_half_box=0.4;
    request.allocation.collision_distance_m=0.1;
    request.allocation.position_reserve_m=0.02;
    request.allocation.velocity_reserve_mps=0.01;
    request.allocation.position_gain=0.4;
    request.allocation.velocity_gain=0.8;
    request.fixed_positions={{100,{5.0,0.0}}};

    auto& hard=request.hard_row_request;
    hard.mobile_ids={1};
    hard.fixed_ids={100};
    hard.states[1]={{0.0,0.0},{0.0,0.0},{0.0,0.0}};
    hard.states[100]={{5.0,0.0},{0.0,0.0},{0.0,0.0}};
    hard.collision_pairs={gf::UndirectedEdge::canonical(1,100)};
    hard.reference_spec={
        PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0,0.0,1.0,1.0,1.0,0.0};
    hard.collision_spec={
        PairwiseSecondOrderBarrierKind::CollisionLower,
        0.1,0.0,1.0,1.0,1.0,0.0};
    hard.acceleration_half_box=0.4;
    hard.require_snapshot_robust_rows=true;
    hard.collision_snapshot_tubes["1--100"]={
        0.02,0.01,gf::SnapshotTubeProvenance::CovarianceSigmaDevelopment};
    request.profile=gf::SolverProfile::OpenSource;
    request.dt_s=0.1;
    return request;
}

}

TEST_CASE("Task 10.11 pipeline rejects fast-gate path before exact and rollout") {
    gf::SharedFrontierAllocationPipeline pipeline(
        {8,32,4,5,5,0.8,0.05,1e-10});
    const auto result=pipeline.choose(request());

    REQUIRE(result.accepted);
    CHECK(result.allocation.targets.at(1).x_index == 1);
    CHECK(result.fast_rejections == 1);
    CHECK(result.exact_rejections == 0);
    CHECK(result.rollout_rejections == 0);
    CHECK(result.rollout_attempts == 1);
    CHECK(result.rollout.completed_cycles == 5);
}

TEST_CASE("Task 10.11 pipeline reports bounded search exhaustion without deadlock claim") {
    gf::SharedFrontierAllocationPipeline pipeline(
        {1,1,1,5,5,0.8,0.05,1e-10});
    auto input=request();
    input.allocation.cells.resize(1);
    const auto result=pipeline.choose(input);

    CHECK_FALSE(result.accepted);
    CHECK(result.reason == "allocator_search_exhausted");
    CHECK(result.exhaustion == gf::AllocatorExhaustion::FastGateRejected);
    CHECK_FALSE(result.physical_deadlock_claimed);
}

TEST_CASE("Task 10.11b pipeline preserves bounded exhaustion and candidate braking reason") {
    gf::SharedFrontierAllocationPipeline pipeline(
        {2,2,2,5,5,0.8,0.05,1e-10});
    auto input=request();
    input.fixed_positions.clear();
    input.hard_row_request.fixed_ids.clear();
    input.hard_row_request.states.erase(100);
    input.hard_row_request.collision_pairs.clear();
    input.hard_row_request.collision_snapshot_tubes.clear();
    input.hard_row_request.workspace_facets={
        {"x-upper",{1.0,0.0},10.0}};
    input.hard_row_request.workspace_snapshot_tubes[1]={0.0,0.0};
    input.hard_row_request.states[1].position={6.0,0.0};
    input.hard_row_request.states[1].velocity={2.0,0.0};
    const auto result=pipeline.choose(input);

    CHECK_FALSE(result.accepted);
    CHECK(result.reason == "allocator_search_exhausted");
    CHECK(result.last_candidate_failure_reason ==
        "snapshot_braking_inadmissible");
    CHECK(result.rollout.reason == "snapshot_braking_inadmissible");
    REQUIRE(result.rejected_rollouts.size() == result.rollout_rejections);
    REQUIRE_FALSE(result.rejected_rollouts.empty());
    for (const auto& rejected : result.rejected_rollouts) {
        CHECK(rejected.reason == "snapshot_braking_inadmissible");
        REQUIRE_FALSE(rejected.braking_trace.empty());
        CHECK(rejected.braking_trace.front().cycle == 0);
    }
    CHECK_FALSE(result.physical_deadlock_claimed);
}
