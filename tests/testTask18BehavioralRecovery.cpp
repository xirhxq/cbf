#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"

#include "grand_finale/Task18ExperimentFixture.hpp"

TEST_CASE("Task 18 direct and common-lambda arms dispatch only the recovered CBF2026 outer") {
    auto direct=gf::makeTask18ExperimentFixture(false);
    auto governed=gf::makeTask18ExperimentFixture(true);
    for (const auto* fixture:{direct.get(),governed.get()}) {
        const auto config=fixture->adapter.config();
        CHECK(config.target_policy_task18_cbf2026_outer);
        CHECK_FALSE(config.target_policy_task17_periodic);
        CHECK_FALSE(config.target_policy_task16_cbf2026);
        CHECK_FALSE(config.target_policy_task15_forward);
        CHECK(config.task18_update_period_cycles==5);
        CHECK(config.task18_yaw_objective==
            gf::Task18YawObjective::IndividualFormationTarget);
        CHECK(config.boundary.policy==gf::BoundaryPolicy::None);
        CHECK(config.acceleration_half_box==doctest::Approx(4.0));
        CHECK(config.speed_row_nominal);
        CHECK(config.speed_row_nominal_limit_mps==doctest::Approx(29.9));
        CHECK(config.velocity_augmented_rows);
        REQUIRE(config.predictive_gamma_tau_mps2.has_value());
        CHECK(*config.predictive_gamma_tau_mps2==doctest::Approx(22.0));
    }
    CHECK_FALSE(direct->adapter.config().task18_common_governor_enabled);
    CHECK(governed->adapter.config().task18_common_governor_enabled);
}

TEST_CASE("Task 18 first public controller step exposes real tasks and source yaw diagnostics") {
    auto fixture=gf::makeTask18ExperimentFixture(false);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    const auto step=fixture->controller.advance();
    REQUIRE(step.step.advanced);
    REQUIRE(step.task18_allocation_evaluated);
    REQUIRE(step.task18_allocation.valid);
    CHECK(step.committed_targets.size()==14);
    CHECK(step.applied_target_centers.size()==14);
    CHECK(step.desired_yaw_rad.size()==14);
    CHECK(step.task_bearing_rad.size()==14);
    for (const auto& [owner,target]:step.committed_targets) {
        CAPTURE(owner);
        CAPTURE(target.id());
        CHECK(target.x_index>=0);
        CHECK(target.y_index>=0);
        CHECK(target.x_index<300);
        CHECK(target.y_index<300);
        CHECK(step.applied_target_centers.count(owner)==1);
        CHECK(step.desired_yaw_rad.count(owner)==1);
        CHECK(step.task_bearing_rad.count(owner)==1);
        CHECK(step.step.applied_yaw_rates_radps.count(owner)==1);
    }
}

TEST_CASE("Task 18 yaw objectives are source-level alternatives with unchanged translational target") {
    for (const auto objective:{gf::Task18YawObjective::SharedTask,
                               gf::Task18YawObjective::ActualVelocity}) {
        auto fixture=gf::makeTask18ExperimentFixture(false,objective);
        REQUIRE(fixture->adapter.initializeStageZero().initialized);
        const auto before=fixture->adapter.runtimeSnapshot();
        const auto step=fixture->controller.advance();
        REQUIRE(step.step.advanced);
        for (std::size_t index=0;index<before.estimate.mobile_ids.size();++index) {
            const auto owner=before.estimate.mobile_ids[index];
            const auto state=before.estimate.mean.segment<4>(4*index);
            if (objective==gf::Task18YawObjective::SharedTask) {
                CHECK(step.desired_yaw_rad.at(owner)==
                    doctest::Approx(step.task_bearing_rad.at(owner)));
            } else {
                // The frozen launch starts at rest, so velocity-heading mode
                // must hold actual yaw rather than invent a direction.
                CHECK(state.tail<2>().norm()==doctest::Approx(0.0));
                CHECK(step.step.applied_yaw_rates_radps.at(owner)==
                    doctest::Approx(0.0));
            }
        }
    }
}

TEST_CASE("Task 18 vaug and tau-selector ablations change only their declared safety selector") {
    const auto no_vaug=gf::task18ExperimentAdapterConfig(false,
        gf::Task18YawObjective::ActualVelocity,0.0,0.0,2027,false);
    CHECK_FALSE(no_vaug.velocity_augmented_rows);
    CHECK(no_vaug.speed_row_nominal_limit_mps==doctest::Approx(29.9));
    CHECK(no_vaug.gamma_feedback_selection==
        gf::GammaFeedbackSelectionMode::LeastIntervention);
    REQUIRE(no_vaug.predictive_gamma_tau_mps2.has_value());
    CHECK(*no_vaug.predictive_gamma_tau_mps2==doctest::Approx(22.0));

    const auto max_margin=gf::task18ExperimentAdapterConfig(false,
        gf::Task18YawObjective::ActualVelocity,0.0,0.0,2027,true,
        gf::GammaFeedbackSelectionMode::MaximumPredictedMargin);
    CHECK(max_margin.velocity_augmented_rows);
    CHECK(max_margin.gamma_feedback_selection==
        gf::GammaFeedbackSelectionMode::MaximumPredictedMargin);
    CHECK_FALSE(max_margin.predictive_gamma_tau_mps2.has_value());

    const auto hard_projection=gf::task18ExperimentAdapterConfig(false,
        gf::Task18YawObjective::ActualVelocity,0.0,0.0,2027,true,
        gf::GammaFeedbackSelectionMode::DiagnosticsOnly);
    CHECK(hard_projection.gamma_feedback_selection==
        gf::GammaFeedbackSelectionMode::DiagnosticsOnly);
    CHECK_FALSE(hard_projection.predictive_gamma_tau_mps2.has_value());
}

TEST_CASE("Task 18 collision-only vaug keeps collision braking rows and restores classic reference rows") {
    auto fixture=gf::makeTask18ExperimentFixture(false,
        gf::Task18YawObjective::ActualVelocity,0.0,0.0,2027,true);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    const auto snapshot=fixture->adapter.runtimeSnapshot();
    const auto request=fixture->adapter.snapshotHardRowRequest(
        snapshot.estimate,snapshot.topology);
    CHECK(request.velocity_augmented_collision_rows);
    CHECK_FALSE(request.velocity_augmented_reference_rows);
    CHECK(fixture->adapter.config().task18_collision_only_vaug);
}

TEST_CASE("Task 18 velocity-task cone yaw keeps motion inside the sixty-degree sensor sector") {
    const double half=M_PI/3.0;
    CHECK(gf::task18VelocityTaskConeDesiredYaw(0.0,0.0,half)==
        doctest::Approx(0.0));
    CHECK(gf::task18VelocityTaskConeDesiredYaw(0.0,M_PI/4.0,half)==
        doctest::Approx(M_PI/4.0));
    CHECK(gf::task18VelocityTaskConeDesiredYaw(0.0,M_PI,half)==
        doctest::Approx(-half));
    CHECK(gf::task18VelocityTaskConeDesiredYaw(0.0,-M_PI/2.0,half)==
        doctest::Approx(-half));
}

TEST_CASE("Task 18 legacy CVT yaw reproduces the source soft-CBF active-row rate") {
    const double error=0.5;
    CHECK(gf::task18LegacyCvtYawRate(0.0,error,1.0)==
        doctest::Approx(std::tan(error/2.0)));
    CHECK(gf::task18LegacyCvtYawRate(0.0,-error,1.0)==
        doctest::Approx(-std::tan(error/2.0)));
    CHECK(gf::task18LegacyCvtYawRate(0.0,M_PI-1.0e-6,1.0)==
        doctest::Approx(1.0));
}

TEST_CASE("Task 18 global-pool fallback keeps both squad ledgers real when one Voronoi share empties") {
    gf::Task16CoverageRequest request;
    request.arm=gf::Task16CoverageArm::BoundaryDecoupled;
    request.search_min={0.0,0.0};
    request.search_max={3000.0,3000.0};
    request.fixed_positions[101]={1500.0,-50.0};
    request.global_pool_fallback_for_empty_share=true;
    for (int id=1;id<=14;++id) {
        const bool squad_a=id<=7;
        request.agents.push_back({static_cast<gf::NodeId>(id),
            squad_a?Eigen::Vector2d{100.0,100.0}:Eigen::Vector2d{2900.0,2900.0},
            Eigen::Vector2d::Zero(),0.0,0.0});
    }
    // All cells are in A's Voronoi share, but the global pool is non-empty.
    request.uncovered_cells={
        {1,1,{15.0,15.0}},
        {2,1,{25.0,15.0}},
        {3,1,{35.0,15.0}},
    };
    const auto result=gf::allocateTask16Cbf2026Coverage(request);
    REQUIRE(result.valid);
    REQUIRE(result.assignments.size()==2);
    REQUIRE(result.targets.size()==14);
    const auto a_id=result.assignments.at("A").task.id();
    const auto b_id=result.assignments.at("B").task.id();
    CHECK(a_id!=b_id);
    for (const auto& [owner,target]:result.targets) {
        CAPTURE(owner);
        CAPTURE(target.id());
        CHECK(target.x_index>=0);
        CHECK(target.y_index>=0);
    }

    SUBCASE("a singleton global pool remains a real task for both squads") {
        request.uncovered_cells={{4,2,{45.0,25.0}}};
        const auto singleton=gf::allocateTask16Cbf2026Coverage(request);
        REQUIRE(singleton.valid);
        REQUIRE(singleton.assignments.size()==2);
        REQUIRE(singleton.targets.size()==14);
        for (const auto& [owner,target]:singleton.targets) {
            CAPTURE(owner);
            CHECK(target.id()=="4:2");
        }
    }
}
