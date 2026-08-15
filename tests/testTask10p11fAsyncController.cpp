#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11fAsyncController.hpp"

#include <thread>

namespace {

std::map<gf::NodeId,gf::FrontierCell> targets(
    gf::Task10p11Fixture& fixture) {
    const auto runtime=fixture.adapter.runtimeSnapshot();
    const auto cells=fixture.swarm.robots.front()->gridWorld
        .getUnexploredCellCenters();
    std::map<gf::NodeId,gf::FrontierCell> result;
    for (std::size_t index=0;index<runtime.estimate.mobile_ids.size();++index) {
        const auto& cell=cells.at(index);
        result[runtime.estimate.mobile_ids[index]]={
            cell.x_index,cell.y_index,{cell.center.x,cell.center.y}};
    }
    return result;
}

}

TEST_CASE("Provisional target commits only after certified physical advance") {
    for (const auto profile : {gf::SolverProfile::OpenSource,
                               gf::SolverProfile::Gurobi}) {
        auto fixture=gf::makeTask10p11Fixture(
            gf::task10p10EasyScenario(),profile);
        REQUIRE(fixture->adapter.initializeStageZero().initialized);
        const auto information=fixture->adapter.currentReferenceAudit();
        CHECK(std::isfinite(
            information.minimum_robust_fim_cone_lower_bound));
        CHECK(information.minimum_range_aoi_margin_s>=0.0);
        gf::Task10p11fAtomicController controller(
            fixture->swarm,fixture->adapter);
        const auto candidate=targets(*fixture);
        auto commit=controller.freshCommitRequest(candidate);
        const double before=fixture->swarm.robots.front()->runtime;
        auto rejected=commit;
        rejected.uncovered_cell_ids.clear();
        const auto failed=controller.commitAndApply(rejected);
        CHECK_FALSE(failed.committed);
        CHECK(controller.targetEpoch()==0);
        CHECK(fixture->swarm.robots.front()->runtime==doctest::Approx(before));

        const auto accepted=controller.commitAndApply(commit);
        CAPTURE(accepted.reason);
        REQUIRE(accepted.committed);
        CHECK(controller.targetEpoch()==1);
        CHECK(controller.committedTargets()==candidate);
        CHECK(accepted.step.minimum_hard_residual>=-1e-7);
        CHECK(fixture->swarm.robots.front()->runtime==doctest::Approx(before+0.1));

        const auto continued=controller.advanceWithoutProposal();
        REQUIRE(continued.advanced);
        CHECK(controller.targetEpoch()==1);
        CHECK(continued.minimum_hard_residual>=-1e-7);
    }
}

TEST_CASE("No committed target uses certified braking without changing SEARCH mode") {
    auto fixture=gf::makeTask10p11Fixture(
        gf::task10p10EasyScenario(),gf::SolverProfile::OpenSource);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::Task10p11fAtomicController controller(fixture->swarm,fixture->adapter);
    const auto step=controller.advanceWithoutProposal();
    REQUIRE(step.advanced);
    CHECK(step.mode==gf::SupervisorMode::Search);
    CHECK(controller.targetEpoch()==0);
    CHECK(step.minimum_hard_residual>=-1e-7);
}

TEST_CASE("Empty online hard polytope stops physics even with background architecture") {
    auto scenario=gf::task10p10EasyScenario();
    scenario.mobile_positions.front().x()=scenario.width_m+20.0;
    auto fixture=gf::makeTask10p11Fixture(
        scenario,gf::SolverProfile::OpenSource);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::Task10p11fAtomicController controller(fixture->swarm,fixture->adapter);
    const double before=fixture->swarm.robots.front()->runtime;
    const auto step=controller.advanceWithoutProposal();
    CHECK_FALSE(step.advanced);
    CHECK(fixture->swarm.robots.front()->runtime==doctest::Approx(before));
}

TEST_CASE("Asynchronous coordinator delivers only at logical ready boundary") {
    auto fixture=gf::makeTask10p11Fixture(
        gf::task10p10EasyScenario(),gf::SolverProfile::OpenSource);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::Task10p11fAsyncCoordinator coordinator(
        fixture->swarm,fixture->adapter,{10,10,3});
    const auto request=coordinator.makeWorkRequest();
    REQUIRE(coordinator.launch(request));
    for (std::uint64_t tick=1;tick<10;++tick) {
        const auto waiting=coordinator.advanceBoundary(tick);
        CHECK(waiting.classification==
              gf::AsyncProposalClassification::NotReady);
        CHECK(waiting.online_step.advanced);
    }
    for (std::size_t spin=0;spin<1000000 && coordinator.workerInFlight();++spin) {
        coordinator.pollWorkerBookkeeping();
        std::this_thread::yield();
    }
    REQUIRE_FALSE(coordinator.workerInFlight());
    const auto committed=coordinator.advanceBoundary(10);
    CAPTURE(committed.reason);
    CHECK(committed.classification==gf::AsyncProposalClassification::Ready);
    CHECK(committed.commit.committed);
    CHECK(coordinator.controller().targetEpoch()==1);
}

TEST_CASE("Runtime provenance fingerprints denominator and tube policy") {
    auto easy=gf::makeTask10p11Fixture(
        gf::task10p10EasyScenario(),gf::SolverProfile::OpenSource);
    auto nonbinding=gf::makeTask10p11Fixture(
        gf::task10p10NonbindingScenario(),gf::SolverProfile::OpenSource);
    REQUIRE(easy->adapter.initializeStageZero().initialized);
    REQUIRE(nonbinding->adapter.initializeStageZero().initialized);
    gf::Task10p11fAtomicController easy_controller(easy->swarm,easy->adapter);
    gf::Task10p11fAtomicController nonbinding_controller(
        nonbinding->swarm,nonbinding->adapter);
    const auto first=easy_controller.currentProvenance();
    const auto second=nonbinding_controller.currentProvenance();
    CHECK(first.denominator_version!=second.denominator_version);
    CHECK(first.tube_policy_version==second.tube_policy_version);
    CHECK(first.config_version!=0);
}
