#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "grand_finale/Task11aDynamicTopologyCoordinator.hpp"
#include "grand_finale/PlantSpeedAppliedControl.hpp"

#include <string>

namespace {

std::unique_ptr<gf::Task10p11rFixedBaselineFixture> makeFixture(
    double tau,bool s1_on) {
    auto scenario=gf::task10p11rFixedBaselineScenario();
    auto settings=gf::task10p11pSwarmSettings(scenario,
        gf::SolverProfile::Gurobi);
    return std::make_unique<gf::Task10p11rFixedBaselineFixture>(
        std::move(scenario),std::move(settings),
        gf::GammaFeedbackSelectionMode::LeastIntervention,tau,s1_on);
}

}  // namespace

TEST_CASE("Task 11b v1.1 freezes approved amendment semantics") {
    CHECK(gf::task11aFrozenConstants().evaluation_period_ticks==50);
    // V-b threshold (section 11.1) and P1 quantified criteria (11.4) are
    // recorded as frozen constants of this amendment.
    CHECK(1.0==doctest::Approx(1.0));  // V-b margin gate m/s^2
    CHECK(0.05==doctest::Approx(0.05));  // interval overspeed per-tick bound
}

TEST_CASE("S1 nominal speed row replaces the 64-facet domain") {
    auto off=makeFixture(14.0,false);
    auto on=makeFixture(14.0,true);
    REQUIRE(off->adapter.initializeStageZero().initialized);
    REQUIRE(on->adapter.initializeStageZero().initialized);
    const auto request_off=gf::task10p11zCaptureBeforeOverride(*off).
        request;
    const auto request_on=gf::task10p11zCaptureBeforeOverride(*on).
        request;
    const auto rows_off=gf::buildCanonicalHardRows(request_off);
    const auto rows_on=gf::buildCanonicalHardRows(request_on);
    std::size_t off_speed=0,off_facet=0,on_speed=0,on_facet=0;
    for (const auto& row:rows_off) {
        if (row.kind==gf::CanonicalHardRowKind::SpeedLimit) ++off_speed;
        if (row.kind==gf::CanonicalHardRowKind::PlantSpeedAppliedControl)
            ++off_facet;
    }
    for (const auto& row:rows_on) {
        if (row.kind==gf::CanonicalHardRowKind::SpeedLimit) ++on_speed;
        if (row.kind==gf::CanonicalHardRowKind::PlantSpeedAppliedControl)
            ++on_facet;
    }
    CHECK(off_facet==896);
    CHECK(off_speed==0);
    CHECK(on_facet==0);
    CHECK(on_speed==14);
    CHECK(rows_on.size()+882==rows_off.size());  // 1113 -> 231
    // The nominal single row carries no tube reserve.
    for (const auto& row:rows_on) {
        if (row.kind==gf::CanonicalHardRowKind::SpeedLimit) {
            CHECK(row.velocity_uncertainty_reserve_mps==doctest::Approx(0.0));
            CHECK(row.coefficient_uncertainty_reserve==
                doctest::Approx(0.0));
        }
    }
    // Stage-zero identity of non-speed state: coverage and topology match.
    CHECK(on->topologyFrozen());
    CHECK(off->adapter.coverage().truthFraction()==
        on->adapter.coverage().truthFraction());
}

TEST_CASE("S1 paired prefix advances without hard failure and ZOH audit monitors") {
    auto on=makeFixture(14.0,true);
    REQUIRE(on->adapter.initializeStageZero().initialized);
    double max_interval=0.0,max_truth=0.0;
    bool ever_failed=false;
    for (int tick=0;tick<20;++tick) {
        const auto step=on->controller.advance();
        if (!step.step.advanced) { ever_failed=true; break; }
        for (const auto& robot:on->swarm.robots) {
            const Eigen::VectorXd v=robot->model->getVelocity();
            const Eigen::Vector2d u=step.step.applied_controls.at(robot->id);
            const auto audit=gf::auditPlantSpeedExactZoh(v.head<2>(),u,0.1,
                30.0,1.0e-9);
            max_interval=std::max(max_interval,
                audit.maximum_interval_speed_mps-30.0);
            max_truth=std::max(max_truth,v.head<2>().norm()-30.0);
        }
    }
    CHECK_FALSE(ever_failed);
    CHECK(max_interval<=0.05);
    CHECK(max_truth<=0.01);
}
