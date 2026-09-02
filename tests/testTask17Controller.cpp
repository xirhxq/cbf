#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"

#include "grand_finale/Task17ExperimentFixture.hpp"

namespace {

std::set<std::string> certifiedUncovered(
    const gf::Task17ExperimentFixture& fixture) {
    std::set<std::string> result;
    const GridWorld& grid=fixture.adapter.coverage().certifiedGrid();
    for (int x=0;x<grid.xNum;++x) for (int y=0;y<grid.yNum;++y) {
        const int index=grid.getIndex(x,y);
        if (grid.valid[index]&&!grid.vis[index])
            result.insert(std::to_string(x)+":"+std::to_string(y));
    }
    return result;
}

}  // namespace

TEST_CASE("Task 17 fixtures differ only in the declared partition arm") {
    std::vector<gf::GrandFinaleSwarmAdapterConfig> configs;
    for (const auto arm:{gf::Task17PeriodicArm::Voronoi,
                         gf::Task17PeriodicArm::SuccessorServiceTime,
                         gf::Task17PeriodicArm::CurrentMemberDistance}) {
        auto fixture=gf::makeTask17ExperimentFixture(arm);
        const auto config=fixture->adapter.config();
        CHECK(config.target_policy_task17_periodic);
        CHECK_FALSE(config.target_policy_task16_cbf2026);
        CHECK(config.task17_periodic_arm==arm);
        CHECK(config.task17_update_period_cycles==5);
        CHECK(config.task17_common_governor_enabled);
        CHECK_FALSE(config.task17_reference_compatible_formation);
        CHECK_FALSE(config.task17_member_aware_wide_formation);
        CHECK_FALSE(config.task17_coherent_service_wide_formation);
        CHECK(config.boundary.policy==gf::BoundaryPolicy::None);
        CHECK(config.predictive_gamma_tau_mps2==doctest::Approx(22.0));
        CHECK(config.acceleration_half_box==doctest::Approx(4.0));
        CHECK(config.speed_row_nominal_limit_mps==doctest::Approx(29.9));
        configs.push_back(config);
    }
    for (std::size_t index=1;index<configs.size();++index) {
        CHECK(configs[index].reference_distance_m==
            doctest::Approx(configs[0].reference_distance_m));
        CHECK(configs[index].collision_distance_m==
            doctest::Approx(configs[0].collision_distance_m));
        CHECK(configs[index].range_noise_std_m==
            doctest::Approx(configs[0].range_noise_std_m));
        CHECK(configs[index].range_dropout_probability==
            doctest::Approx(configs[0].range_dropout_probability));
    }
}

TEST_CASE("Task 17 reallocates after five completed cycles without resetting the applied ledger") {
    auto fixture=gf::makeTask17ExperimentFixture(
        gf::Task17PeriodicArm::Voronoi);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    const auto first=fixture->controller.advance();
    REQUIRE(first.step.advanced);
    REQUIRE(first.task17_allocation_evaluated);
    REQUIRE(first.task17_allocation.valid);
    const auto first_epoch=first.target_epoch;
    const auto first_targets=first.committed_targets;
    auto previous=first.applied_target_centers;
    REQUIRE(previous.size()==14);
    for (int cycle=1;cycle<5;++cycle) {
        const auto step=fixture->controller.advance();
        REQUIRE(step.step.advanced);
        CHECK_FALSE(step.task17_allocation_evaluated);
        CHECK(step.target_epoch==first_epoch);
        previous=step.applied_target_centers;
    }
    const auto pool_before=certifiedUncovered(*fixture);
    const auto periodic=fixture->controller.advance();
    REQUIRE(periodic.step.advanced);
    REQUIRE(periodic.task17_allocation_evaluated);
    REQUIRE(periodic.task17_allocation.valid);
    CHECK(periodic.target_epoch==first_epoch+1);
    REQUIRE(periodic.committed_targets.size()==14);
    REQUIRE(periodic.applied_target_centers.size()==14);
    bool replaced_an_uncovered_task=false;
    for (const auto& [owner,target]:periodic.committed_targets) {
        CHECK(pool_before.count(target.id())==1);
        const auto old=first_targets.find(owner);
        if (old!=first_targets.end()&&old->second.id()!=target.id()&&
            pool_before.count(old->second.id())==1)
            replaced_an_uncovered_task=true;
        const double displacement=(periodic.applied_target_centers.at(owner)-
            previous.at(owner)).norm();
        CHECK(displacement<=fixture->adapter.config().dt_s*
            gf::task16AnalyticReferenceSpeedMps(
                fixture->adapter.config().speed_limit_mps,
                fixture->adapter.config().acceleration_half_box,
                fixture->adapter.config().velocity_gain)+1e-9);
        CHECK(periodic.step.applied_controls.at(owner).cwiseAbs().maxCoeff()
            <=4.0+1e-9);
    }
    CHECK(replaced_an_uncovered_task);
}

TEST_CASE("Task 17 direct ablation updates the nominal ledger but leaves hard control active") {
    auto fixture=gf::makeTask17ExperimentFixture(
        gf::Task17PeriodicArm::Voronoi,0.0,0.0,2027,false,5);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    for (int cycle=0;cycle<6;++cycle) {
        const auto step=fixture->controller.advance();
        REQUIRE(step.step.advanced);
        CHECK_FALSE(step.target_governor_evaluated);
        for (const auto& [owner,target]:step.committed_targets) {
            CHECK((step.applied_target_centers.at(owner)-target.center).norm()
                ==doctest::Approx(0.0));
            CHECK(step.step.applied_controls.at(owner).cwiseAbs().maxCoeff()
                <=4.0+1e-9);
        }
    }
}

TEST_CASE("Task 17 no-workspace scenario permits an observer outside the map") {
    auto fixture=gf::makeTask17ExperimentFixture(
        gf::Task17PeriodicArm::Voronoi);
    fixture->swarm.robots.front()->model->setPosition2D({4000.0,1500.0});
    CHECK_NOTHROW(fixture->swarm.prepareCertifiedControlStep(false));
    CHECK_THROWS(fixture->swarm.prepareCertifiedControlStep(true));
}
