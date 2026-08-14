#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p10CompletionFixture.hpp"

TEST_CASE("Task 10.10 restores CBF2026 scale without inventing a speed limit") {
    const auto frozen = gf::loadTask10p10Config();
    CHECK(frozen.cbf2026_source_commit ==
          "47e7b3d6f5b1be74a3dfa32150b4246c619e9a0d");
    CHECK(frozen.grid_spacing_m == doctest::Approx(10.0));
    CHECK(frozen.sector_outer_radius_m == doctest::Approx(400.0));
    CHECK(frozen.sector_half_angle_deg == doctest::Approx(60.0));
    CHECK(frozen.acceleration_half_box_mps2 == doctest::Approx(0.4));
    CHECK(frozen.control_period_s == doctest::Approx(0.1));
    CHECK_FALSE(frozen.speed_limit_mps.has_value());
}

TEST_CASE("Task 10.10 heading is initialized state and remains frozen in SEARCH and HOLD") {
    auto fixture = gf::makeTask10p10Fixture(
        gf::task10p10EasyScenario(),gf::SolverProfile::OpenSource);
    for (const auto& robot : fixture->swarm.robots)
        CHECK(robot->model->getStateVariable("yawRad") == doctest::Approx(0.0));

    const auto search = fixture->adapter.step();
    REQUIRE(search.advanced);
    CHECK(search.mode == gf::SupervisorMode::Search);
    for (const auto& robot : fixture->swarm.robots)
        CHECK(robot->model->getStateVariable("yawRad") == doctest::Approx(0.0));

    const auto hold_mode = fixture->adapter.supervisor().requestReformation(
        fixture->swarm.robots.front()->runtime,false,false);
    REQUIRE(hold_mode == gf::SupervisorMode::Hold);
    const auto hold = fixture->adapter.step();
    REQUIRE(hold.advanced);
    CHECK(hold.mode == gf::SupervisorMode::Hold);
    for (const auto& robot : fixture->swarm.robots)
        CHECK(robot->model->getStateVariable("yawRad") == doctest::Approx(0.0));
}
