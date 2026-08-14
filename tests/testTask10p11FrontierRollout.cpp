#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/FrontierViabilityRollout.hpp"

gf::FrontierRolloutRequest rolloutRequest(gf::SolverProfile profile) {
    gf::FrontierRolloutRequest request;
    request.profile=profile;
    request.hard_row_request.mobile_ids={1};
    request.hard_row_request.states[1]={{5,5},{0,0},{0,0}};
    request.hard_row_request.reference_spec={
        PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850,0,1,1,1,0};
    request.hard_row_request.collision_spec={
        PairwiseSecondOrderBarrierKind::CollisionLower,
        0.1,0,1,1,1,0};
    request.hard_row_request.acceleration_half_box=0.4;
    request.hard_row_request.require_snapshot_robust_rows=true;
    request.hard_row_request.workspace_facets={
        {"x-upper",{1,0},100},{"x-lower",{-1,0},0},
        {"y-upper",{0,1},100},{"y-lower",{0,-1},0}};
    request.hard_row_request.workspace_snapshot_tubes[1]={0.1,0.2};
    request.targets={{1,{20,5}}};
    request.position_gain=0.4;
    request.velocity_gain=0.8;
    request.dt_s=0.1;
    request.cycles=5;
    return request;
}

TEST_CASE("Task 10.11 rollout spans the full allocation epoch and expands tube") {
    for (const auto profile : {gf::SolverProfile::OpenSource,gf::SolverProfile::Gurobi}) {
        const auto result=gf::evaluateFrontierRollout(rolloutRequest(profile));
        INFO("reason=",result.reason," failed_cycle=",result.failed_cycle,
             " source=",result.first_negative_source,
             " trace=",result.braking_trace.size());
        REQUIRE(result.accepted);
        CHECK(result.completed_cycles == 5);
        CHECK(result.duration_s == doctest::Approx(0.5));
        CHECK(result.final_request.workspace_snapshot_tubes.at(1)
            .position_radius_m == doctest::Approx(0.2));
        CHECK(result.final_request.workspace_snapshot_tubes.at(1)
            .velocity_radius_mps == doctest::Approx(0.2));
        CHECK(result.minimum_robust_residual >= -1e-7);
        REQUIRE(result.braking_trace.size() == 5);
        for (std::size_t cycle=0;cycle<result.braking_trace.size();++cycle) {
            CHECK(result.braking_trace[cycle].cycle == cycle);
            CHECK(result.braking_trace[cycle].snapshot.hard_polytope_nonempty);
            CHECK(result.braking_trace[cycle].snapshot.snapshot_braking_admissible);
        }
        CHECK(result.first_negative_source.empty());
    }
}

TEST_CASE("Task 10.11 rollout rejects an empty hard polytope without advancing") {
    auto request=rolloutRequest(gf::SolverProfile::OpenSource);
    request.hard_row_request.states[1].position={99.9,5};
    request.hard_row_request.states[1].velocity={1,0};
    const auto result=gf::evaluateFrontierRollout(request);
    CHECK_FALSE(result.accepted);
    CHECK(result.reason == "hard_polytope_empty");
    CHECK(result.completed_cycles == 0);
    CHECK(result.failed_cycle == 0);
    REQUIRE(result.braking_trace.size() == 1);
    CHECK(result.braking_trace.front().snapshot.reason == "hard_polytope_empty");
}

TEST_CASE("Task 10.11b rollout rejects snapshot braking inadmissibility before applying control") {
    auto request=rolloutRequest(gf::SolverProfile::OpenSource);
    request.hard_row_request.states[1].position={95.4,5.0};
    request.hard_row_request.states[1].velocity={2.0,0.0};
    const auto result=gf::evaluateFrontierRollout(request);
    CHECK_FALSE(result.accepted);
    CHECK(result.reason == "snapshot_braking_inadmissible");
    CHECK(result.completed_cycles == 0);
    CHECK(result.failed_cycle == 0);
    REQUIRE(result.braking_trace.size() == 1);
    CHECK(result.first_negative_source == "workspace:1:x-upper");
    CHECK(result.braking_trace.front().snapshot.minimum_braking_slack_m ==
        doctest::Approx(-1.55));
}

TEST_CASE("Task 10.11b rollout records horizon insufficiency without claiming terminal safety") {
    auto request=rolloutRequest(gf::SolverProfile::OpenSource);
    request.hard_row_request.states[1].position={10.0,5.0};
    request.hard_row_request.states[1].velocity={1.0,0.0};
    request.targets[1]={20.0,5.0};
    const auto result=gf::evaluateFrontierRollout(request);
    REQUIRE_FALSE(result.braking_trace.empty());
    CHECK(result.braking_trace.front().snapshot.reason ==
        "braking_horizon_insufficient");
    CHECK(result.braking_trace.front().snapshot.snapshot_braking_admissible);
}
