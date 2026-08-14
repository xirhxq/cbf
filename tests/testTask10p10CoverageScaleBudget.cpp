#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/CoverageScaleBudget.hpp"
#include "grand_finale/Task10p10CompletionFixture.hpp"

#include <cmath>

TEST_CASE("Task 10.10 finite-cell union never double counts overlapping sectors") {
    const gf::CoverageGridSpec grid{4.0, 3.0, 1.0};
    const gf::SectorFootprintSpec footprint{0.0, 1.1, M_PI};
    const std::vector<gf::SectorFootprintStage> initialized{{{0.0,0.0},0.0}};
    const std::vector<gf::SectorFootprintStage> reachable{
        {{0.0,0.0},0.0}, {{1.0,0.0},0.0}};

    const auto budget = gf::evaluateCoverageScaleBudget(
        grid, footprint, initialized, reachable, 0.95);

    CHECK(budget.valid_cell_count == 12);
    CHECK(budget.initial_covered_cells == 1);
    CHECK(budget.reachable_union_cells == 2);
    CHECK(budget.initial_fraction == doctest::Approx(1.0/12.0));
    CHECK(budget.reachable_union_fraction == doctest::Approx(2.0/12.0));
    CHECK(budget.initial_below_target);
}

TEST_CASE("Task 10.10 rejects a candidate already complete at initialization") {
    const auto budget = gf::evaluateCoverageScaleBudget(
        {1.0,1.0,1.0}, {0.0,1.0,M_PI},
        {{{0.0,0.0},0.0}}, {{{0.0,0.0},0.0}}, 0.95);
    CHECK_FALSE(budget.initial_below_target);
    CHECK(budget.initial_fraction == doctest::Approx(1.0));
}

TEST_CASE("Task 10.10 rest-to-rest budget distinguishes absent and finite speed") {
    const auto unbounded_speed = gf::minimumRestToRestTime(100.0,0.4,std::nullopt);
    CHECK(unbounded_speed.profile == gf::MotionProfile::Triangular);
    CHECK(unbounded_speed.seconds == doctest::Approx(31.6227766017));
    CHECK_FALSE(unbounded_speed.speed_limit_mps.has_value());

    const auto trapezoidal = gf::minimumRestToRestTime(100.0,0.4,2.0);
    CHECK(trapezoidal.profile == gf::MotionProfile::Trapezoidal);
    CHECK(trapezoidal.seconds == doctest::Approx(55.0));
    CHECK(trapezoidal.speed_limit_mps == doctest::Approx(2.0));
}

TEST_CASE("Task 10.10 nonbinding and binding candidates have exact completion headroom") {
    const auto nonbinding = gf::evaluateTask10p10ScenarioBudget(
        gf::task10p10NonbindingScenario());
    CHECK(nonbinding.coverage.map_area_m2 == doctest::Approx(400000.0));
    CHECK(nonbinding.coverage.valid_cell_count == 4000);
    CHECK(nonbinding.coverage.initial_fraction < 0.95);
    CHECK(nonbinding.coverage.reachable_union_cells == 4000);
    CHECK(nonbinding.coverage.reachable_union_fraction == doctest::Approx(1.0));
    CHECK(nonbinding.timeout_factor >= 1.5);
    CHECK_FALSE(nonbinding.motion.speed_limit_mps.has_value());
    MESSAGE("nonbinding initial=",nonbinding.coverage.initial_fraction,
        " cells=",nonbinding.coverage.valid_cell_count,
        " farthest=",nonbinding.farthest_required_center_m,
        " lower_time=",nonbinding.motion.seconds,
        " timeout_factor=",nonbinding.timeout_factor);

    const auto binding = gf::evaluateTask10p10ScenarioBudget(
        gf::task10p10BindingScenario());
    CHECK(binding.coverage.map_area_m2 == doctest::Approx(900000.0));
    CHECK(binding.coverage.valid_cell_count == 9000);
    CHECK(binding.coverage.initial_fraction < 0.95);
    CHECK(binding.coverage.reachable_union_cells == 9000);
    CHECK(binding.coverage.reachable_union_fraction == doctest::Approx(1.0));
    CHECK(binding.timeout_factor >= 1.5);
    CHECK(binding.farthest_required_center_m > 1600.0);
    MESSAGE("binding initial=",binding.coverage.initial_fraction,
        " cells=",binding.coverage.valid_cell_count,
        " farthest=",binding.farthest_required_center_m,
        " lower_time=",binding.motion.seconds,
        " timeout_factor=",binding.timeout_factor);
}
