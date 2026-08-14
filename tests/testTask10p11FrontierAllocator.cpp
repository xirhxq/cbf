#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/SharedCollisionViableFrontierAllocator.hpp"

namespace {

gf::FrontierAllocationRequest basicRequest() {
    gf::FrontierAllocationRequest request;
    request.snapshot_token = 1;
    request.topology_token = 2;
    request.grid_token = 3;
    request.agents = {
        {1,{0.0,0.0},{0.0,0.0}}, {2,{10.0,0.0},{0.0,0.0}}};
    request.cells = {
        {0,0,{0.0,10.0}}, {1,0,{10.0,10.0}},
        {2,0,{5.0,10.0}}, {3,0,{20.0,0.0}}};
    request.acceleration_half_box = 0.4;
    request.collision_distance_m = 0.1;
    request.position_reserve_m = 0.2;
    request.velocity_reserve_mps = 0.1;
    request.position_gain = 0.4;
    request.velocity_gain = 0.8;
    return request;
}

}

TEST_CASE("Task 10.11 allocator assigns unique Voronoi frontiers deterministically") {
    gf::SharedCollisionViableFrontierAllocator allocator(
        {8,32,4,5,5,0.8,0.05,1e-10});
    auto result = allocator.allocate(basicRequest());
    REQUIRE(result.accepted);
    CHECK(result.reason == "accepted");
    CHECK(result.targets.size() == 2);
    CHECK(result.targets.at(1) != result.targets.at(2));
    CHECK(result.targets.at(1).x_index == 0);
    CHECK(result.targets.at(2).x_index == 1);
    CHECK(result.bundle_attempts <= 32);
}

TEST_CASE("Task 10.11 Voronoi ownership uses distance then stable agent id") {
    const std::vector<gf::FrontierAgentState> agents = {
        {7,{0.0,0.0},{0.0,0.0}}, {3,{10.0,0.0},{0.0,0.0}}};

    CHECK(gf::deterministicVoronoiOwner({0,0,{1.0,0.0}},agents) == 7);
    CHECK(gf::deterministicVoronoiOwner({1,0,{9.0,0.0}},agents) == 3);
    CHECK(gf::deterministicVoronoiOwner({2,0,{5.0,0.0}},agents) == 3);
}

TEST_CASE("Task 10.11 no-good and rotating priority change bounded retry order") {
    gf::SharedCollisionViableFrontierAllocator allocator(
        {8,32,4,5,5,0.8,0.05,1e-10});
    auto request = basicRequest();
    const auto first = allocator.allocate(request);
    REQUIRE(first.accepted);
    allocator.rejectCurrentBundle(request);
    const auto second = allocator.allocate(request);
    CHECK(second.accepted);
    CHECK(second.bundle_id != first.bundle_id);
    CHECK(second.priority_epoch == 1);
    for (const auto& cell : request.cells)
        CHECK(allocator.rejectionAge(cell.id()) >= 2);
}

TEST_CASE("Task 10.11 bounded search reports exhaustion rather than physical deadlock") {
    gf::SharedCollisionViableFrontierAllocator allocator(
        {1,1,1,5,5,0.8,0.05,1e-10});
    auto request = basicRequest();
    request.cells.clear();
    const auto empty = allocator.allocate(request);
    CHECK_FALSE(empty.accepted);
    CHECK(empty.reason == "allocator_search_exhausted");
    CHECK(empty.exhaustion == gf::AllocatorExhaustion::CandidateUniverseEmpty);
    CHECK_FALSE(empty.physical_deadlock_claimed);
}
