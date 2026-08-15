#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/D1DevelopmentExperiment.hpp"
#include "grand_finale/Task10p11gReadinessFixture.hpp"
#include "grand_finale/Task10p9MechanismFixture.hpp"

namespace {

void requireInitialFeasible(gf::Task10p11gFixture& fixture) {
    const auto initialized=fixture.adapter.initializeStageZero();
    REQUIRE(initialized.initialized);
    CHECK(initialized.truth_coverage>0.0);
    CHECK(initialized.truth_coverage<0.95);
    const auto rows=fixture.adapter.currentSnapshotHardRows(
        fixture.adapter.supervisor().topology());
    for (const auto owner : fixture.adapter.runtimeSnapshot().estimate.mobile_ids) {
        CHECK(std::count_if(rows.begin(),rows.end(),[&](const auto& row) {
            return row.owner==owner &&
                row.kind==gf::CanonicalHardRowKind::SpeedLimit;
        })==1);
        const auto exact=gf::evaluateProgressCompatibility(
            rows,owner,Eigen::Vector2d::Zero(),4.0,
            {std::numeric_limits<double>::max(),0.0,1e-10,true});
        INFO("owner=",owner," reason=",exact.reason);
        REQUIRE(exact.polytope_nonempty);
    }
}

}

TEST_CASE("Frozen V1 hard polytope is nonempty for basic 4+2 and 14+3 stage zero") {
    for (const auto profile : {gf::SolverProfile::OpenSource,
                               gf::SolverProfile::Gurobi}) {
        auto four=gf::makeTask10p11gFixture(
            gf::task10p10EasyScenario(),profile);
        requireInitialFeasible(*four);
        auto fourteen=gf::makeTask10p11gFixture(
            gf::task10p10NonbindingScenario(),profile);
        requireInitialFeasible(*fourteen);
    }
}

TEST_CASE("Candidate maps have fixed finite denominators and optimistic T95 headroom") {
    for (const auto scenario : {gf::task10p10EasyScenario(),
                                gf::task10p10NonbindingScenario(),
                                gf::task10p10BindingScenario()}) {
        const auto budget=gf::evaluateTask10p11gCoverageBudget(scenario);
        INFO(scenario.id," initial=",budget.cells.initial_fraction,
             " optimistic=",budget.cells.reachable_union_fraction,
             " t_motion=",budget.motion_lower_bound.seconds,
             " factor=",budget.timeout_factor);
        CHECK(budget.cells.valid_cell_count>0);
        CHECK(budget.cells.initial_below_target);
        CHECK(budget.cells.reachable_union_fraction>0.95);
        CHECK(budget.timeout_factor>3.0);
        CHECK(budget.motion_lower_bound.speed_limit_mps.has_value());
    }
}

TEST_CASE("Easy offline-exact shared allocator advances yaw and new coverage under both profiles") {
    for (const auto profile : {gf::SolverProfile::OpenSource,
                               gf::SolverProfile::Gurobi}) {
        auto fixture=gf::makeTask10p11gFixture(
            gf::task10p10EasyScenario(),profile);
        const auto stage=fixture->adapter.initializeStageZero();
        REQUIRE(stage.initialized);
        const double initial=stage.truth_coverage;
        gf::Task10p11gOfflineExactCoordinator coordinator(
            fixture->swarm,fixture->adapter);
        bool yaw_changed=false;
        double min_residual=std::numeric_limits<double>::infinity();
        for (int cycle=0;cycle<12;++cycle) {
            const auto advanced=coordinator.advance(cycle);
            INFO("profile=",static_cast<int>(profile)," cycle=",cycle,
                 " reason=",advanced.reason);
            REQUIRE(advanced.step.advanced);
            min_residual=std::min(
                min_residual,advanced.step.minimum_hard_residual);
            for (const auto& robot : fixture->swarm.robots)
                yaw_changed=yaw_changed || std::abs(
                    robot->model->getStateVariable("yawRad")-M_PI/2.0)>1e-8;
        }
        INFO("initial=",initial," final=",
             fixture->adapter.coverage().truthFraction(),
             " min_residual=",min_residual," yaw_changed=",yaw_changed);
        CHECK(fixture->adapter.coverage().truthFraction()>initial);
        CHECK(yaw_changed);
        CHECK(min_residual>=-fixture->adapter.config().residual_tolerance);
    }
}

TEST_CASE("Nonbinding records bounded allocator exhaustion without misclassifying hard feasibility") {
    for (const auto profile : {gf::SolverProfile::OpenSource,
                               gf::SolverProfile::Gurobi}) {
        auto fixture=gf::makeTask10p11gFixture(
            gf::task10p10NonbindingScenario(),profile);
        REQUIRE(fixture->adapter.initializeStageZero().initialized);
        gf::Task10p11gOfflineExactCoordinator coordinator(
            fixture->swarm,fixture->adapter);
        const auto advanced=coordinator.advance(0);
        const auto& diagnostic=advanced.proposal.diagnostic;
        INFO(advanced.reason," fast=",diagnostic.fast_rejections,
             " exact=",diagnostic.exact_rejections,
             " rollout=",diagnostic.rollout_rejections,
             " bundles=",diagnostic.candidate_bundles);
        CHECK_FALSE(advanced.step.advanced);
        CHECK(advanced.reason=="allocator_search_exhausted");
        CHECK(diagnostic.fast_rejections==diagnostic.candidate_bundles);
        const auto rows=fixture->adapter.currentSnapshotHardRows(
            fixture->adapter.supervisor().topology());
        for (const auto owner : fixture->adapter.runtimeSnapshot().estimate.mobile_ids)
            CHECK(gf::evaluateProgressCompatibility(rows,owner,
                Eigen::Vector2d::Zero(),4.0,
                {std::numeric_limits<double>::max(),0.0,1e-10,true})
                .polytope_nonempty);
        CHECK(gf::minimumReferenceBarrierH(
            rows)>100.0);
        CHECK(fixture->adapter.transitionStackSize()==0);
    }
}

TEST_CASE("Binding V1 fixture closes MIQP certifier fresh UNION successor under both profiles") {
    const auto scenario=gf::task10p11gBindingMechanismScenario();
    CHECK(gf::referenceBallOverlap(scenario.fixed_positions.at(100),
        scenario.fixed_positions.at(102),850.0,849.0).strict);
    for (const auto profile : {gf::SolverProfile::OpenSource,
                               gf::SolverProfile::Gurobi}) {
        for (const auto strategy : {gf::D1TopologyStrategy::FixedDag,
                 gf::D1TopologyStrategy::ProposedCertified,
                 gf::D1TopologyStrategy::GreedyCertified}) {
            auto fixture=gf::makeTask10p11gFixture(scenario,profile);
            REQUIRE(fixture->adapter.initializeStageZero().initialized);
            REQUIRE(fixture->adapter.step().advanced);
            const double old_h=gf::minimumReferenceBarrierH(
                fixture->adapter.currentSnapshotHardRows(
                    fixture->adapter.supervisor().topology()));
            CHECK(old_h>0.0);
            CHECK(old_h<100.0);
            if (strategy==gf::D1TopologyStrategy::FixedDag) {
                const auto fixed=fixture->adapter.step();
                REQUIRE(fixed.advanced);
                CHECK(gf::containsEdge(
                    fixture->adapter.supervisor().topology(),{100,2}));
                CHECK_FALSE(gf::containsEdge(
                    fixture->adapter.supervisor().topology(),{102,2}));
                continue;
            }
            if (strategy==gf::D1TopologyStrategy::ProposedCertified) {
                const auto proposal=fixture->adapter.proposeAndBegin(
                    gf::task10p9ActiveProposal(fixture->adapter));
                INFO(proposal.reason);
                REQUIRE(proposal.transition_started);
                CHECK(gf::containsEdge(proposal.selected_topology,{102,2}));
            } else {
                std::vector<gf::D1ReplacementCandidate> candidates;
                for (const auto& edge :
                     gf::task10p9ActiveCandidateUniverse(fixture->adapter))
                    candidates.push_back({edge,{100,2},
                        edge.id()=="102->2"?1.0:0.0});
                const auto selected=gf::chooseD1Replacement(strategy,candidates);
                REQUIRE(selected.has_value());
                REQUIRE(fixture->adapter.beginReplacement(
                    selected->addition,selected->removal));
            }
            const auto union_step=fixture->adapter.step();
            REQUIRE(union_step.advanced);
            CHECK(union_step.mode==gf::SupervisorMode::Union);
            CHECK(union_step.minimum_hard_residual>=
                  -fixture->adapter.config().residual_tolerance);
            REQUIRE(fixture->adapter.finishReplacementAfterFreshCycle());
            CHECK(gf::containsEdge(
                fixture->adapter.supervisor().topology(),{102,2}));
            CHECK_FALSE(gf::containsEdge(
                fixture->adapter.supervisor().topology(),{100,2}));
        }
    }
}
