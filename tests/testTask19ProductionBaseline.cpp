#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"

#include "grand_finale/Task19ProductionBaseline.hpp"

TEST_CASE("GrandFinale production defaults are the selected Task 18 policy") {
    const auto defaults=gf::task19ProductionDefaults();
    CHECK(defaults.policy=="production");
    CHECK(defaults.predictive_tau_mps2==doctest::Approx(14.0));
    CHECK(defaults.coverage_checkpoint_s==doctest::Approx(500.0));
    CHECK(defaults.resource_watchdog_s==doctest::Approx(900.0));
    CHECK(defaults.terminate_at_certified_t100);

    const auto config=gf::task19ProductionAdapterConfig();
    CHECK(config.target_policy_task18_cbf2026_outer);
    CHECK_FALSE(config.target_policy_task17_periodic);
    CHECK_FALSE(config.target_policy_task16_cbf2026);
    CHECK_FALSE(config.target_policy_task15_forward);
    CHECK_FALSE(config.target_policy_task20_dag_lattice);
    CHECK(config.task18_update_period_cycles==5);
    CHECK_FALSE(config.task18_common_governor_enabled);
    CHECK(config.task18_yaw_objective==
        gf::Task18YawObjective::ActualVelocity);
    REQUIRE(config.predictive_gamma_tau_mps2.has_value());
    CHECK(*config.predictive_gamma_tau_mps2==doctest::Approx(14.0));
    CHECK(config.velocity_augmented_rows);
    CHECK_FALSE(config.task18_collision_only_vaug);
    CHECK(config.acceleration_half_box==doctest::Approx(4.0));
    CHECK(config.speed_row_nominal_limit_mps==doctest::Approx(29.9));
    CHECK(config.boundary.policy==gf::BoundaryPolicy::None);
}

TEST_CASE("Production run outcome distinguishes success censoring and safety failure") {
    CHECK(gf::classifyTask19Run(4822,false,false)==
        gf::Task19RunOutcome::CertifiedT100);
    CHECK(gf::classifyTask19Run(std::nullopt,false,false)==
        gf::Task19RunOutcome::BudgetCensored);
    CHECK(gf::classifyTask19Run(std::nullopt,true,false)==
        gf::Task19RunOutcome::SafetyFailure);
    CHECK(gf::classifyTask19Run(std::nullopt,false,true)==
        gf::Task19RunOutcome::SafetyFailure);
    CHECK(gf::task19RunOutcomeName(
        gf::Task19RunOutcome::BudgetCensored)=="budget_censored");
}

TEST_CASE("Production fixture exposes only real Task 18 targets") {
    auto fixture=gf::makeTask19ProductionFixture();
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    const auto step=fixture->controller.advance();
    REQUIRE(step.step.advanced);
    REQUIRE(step.task18_allocation_evaluated);
    REQUIRE(step.task18_allocation.valid);
    REQUIRE(step.committed_targets.size()==14);
    for (const auto& [owner,target]:step.committed_targets) {
        const gf::NodeId captured_owner=owner;
        CAPTURE(captured_owner);
        CHECK(target.x_index>=0);
        CHECK(target.x_index<300);
        CHECK(target.y_index>=0);
        CHECK(target.y_index<300);
    }
}

TEST_CASE("Production efficiency accumulator uses uncovered-cell waiting time") {
    gf::Task19EfficiencyAccumulator metrics(10,0.5);
    metrics.observe(0);
    metrics.observe(5);
    metrics.observe(9);
    metrics.observe(10);
    CHECK(metrics.j_uncovered_cell_seconds()==doctest::Approx(8.0));
    REQUIRE(metrics.t50_tick().has_value());
    REQUIRE(metrics.t95_tick().has_value());
    REQUIRE(metrics.t99_tick().has_value());
    REQUIRE(metrics.t100_tick().has_value());
    CHECK(*metrics.t50_tick()==1);
    CHECK(*metrics.t95_tick()==3);
    CHECK(*metrics.t99_tick()==3);
    CHECK(*metrics.t100_tick()==3);
}

TEST_CASE("Production energy proxy distinguishes per-axis box from horizontal norm") {
    const std::map<gf::NodeId,Eigen::Vector2d> controls{
        {1,{3.0,4.0}},{2,{0.0,2.0}}};
    CHECK(gf::task19SquaredControlEnergyIncrement(controls,0.1)==
        doctest::Approx(2.9));
}
