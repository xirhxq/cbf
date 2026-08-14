#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/FrontierViabilityGate.hpp"

TEST_CASE("Task 10.11 fast gate rejects duplicate swap and crossing targets") {
    gf::BrakingGateRequest duplicate;
    duplicate.agents = {{{1,{0,0},{0,0}},{1,0}},{{2,{2,0},{0,0}},{1,0}}};
    duplicate.acceleration_half_box = 0.4;
    duplicate.collision_distance_m = 0.1;
    CHECK(gf::evaluateBrakingGate(duplicate).reason == "duplicate_target");

    gf::BrakingGateRequest swap;
    swap.agents = {{{1,{0,0},{0,0}},{2,0}},{{2,{2,0},{0,0}},{0,0}}};
    swap.acceleration_half_box = 0.4;
    swap.collision_distance_m = 0.1;
    CHECK(gf::evaluateBrakingGate(swap).reason == "segment_crossing");

    gf::BrakingGateRequest crossing;
    crossing.agents = {{{1,{0,0},{0,0}},{2,2}},{{2,{0,2},{0,0}},{2,0}}};
    crossing.acceleration_half_box = 0.4;
    crossing.collision_distance_m = 0.1;
    CHECK(gf::evaluateBrakingGate(crossing).reason == "segment_crossing");
}

TEST_CASE("Task 10.11 braking boundary distinguishes mobile and fixed authority") {
    gf::BrakingGateRequest mobile;
    mobile.agents = {{{1,{0,0},{1,0}},{0,10}},{{2,{3,0},{-1,0}},{3,10}}};
    mobile.acceleration_half_box = 0.4;
    mobile.collision_distance_m = 0.1;
    mobile.position_reserve_m = 0.1;
    mobile.velocity_reserve_mps = 0.0;
    const auto mobile_result = gf::evaluateBrakingGate(mobile);
    CHECK(mobile_result.minimum_braking_margin == doctest::Approx(0.3));
    CHECK(mobile_result.accepted);

    gf::BrakingGateRequest fixed;
    fixed.agents = {{{1,{0,0},{1,0}},{0,10}}};
    fixed.fixed_positions = {{100,{3,0}}};
    fixed.acceleration_half_box = 0.4;
    fixed.collision_distance_m = 0.1;
    fixed.position_reserve_m = 0.1;
    const auto fixed_result = gf::evaluateBrakingGate(fixed);
    CHECK(fixed_result.minimum_braking_margin == doctest::Approx(1.55));
    CHECK(fixed_result.accepted);
}

TEST_CASE("Task 10.11 complete row gate catches jointly empty collision demands") {
    std::vector<gf::CanonicalHardRow> opposing{
        gf::makeCanonicalGammaRow("left",1,{1,0},-0.3),
        gf::makeCanonicalGammaRow("right",1,{-1,0},-0.3)};
    CHECK(gf::evaluateProgressCompatibility(
        {opposing[0]},1,{0,0},0.4,{1.0,0.0,1e-10,true}).polytope_nonempty);
    CHECK(gf::evaluateProgressCompatibility(
        {opposing[1]},1,{0,0},0.4,{1.0,0.0,1e-10,true}).polytope_nonempty);
    const auto joint = gf::evaluateCurrentBundleGate(
        opposing,{{1,{0,0}}},0.4,{1.0,0.0,1e-10,true});
    CHECK_FALSE(joint.accepted);
    CHECK(joint.reason == "hard_polytope_empty");
}
