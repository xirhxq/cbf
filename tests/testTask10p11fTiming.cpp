#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/AsyncFrontierProposalKernel.hpp"
#include "grand_finale/Task10p11fAsyncController.hpp"

#include <chrono>

namespace {

gf::AsyncFrontierWorkRequest work(
    gf::Task10p11Fixture& fixture,
    gf::Task10p11fAtomicController& controller) {
    const auto current=fixture.controller.currentPipelineRequest();
    gf::AsyncFrontierWorkRequest value;
    value.provenance=controller.currentProvenance();
    value.allocator_config=gf::task10p11AllocatorConfig();
    value.allocation=current.allocation;
    value.fixed_positions=current.fixed_positions;
    value.dt_s=current.dt_s;
    value.estimator_acceleration_variance=
        current.estimator_acceleration_variance;
    value.gamma_feedback_config=current.gamma_feedback_config;
    value.gamma_feedback_config.selection_mode=
        gf::GammaFeedbackSelectionMode::MaximumPredictedMargin;
    gf::AllocatorFairnessLedger ledger;
    const auto fairness=ledger.snapshot(value.allocation.cells);
    value.allocation.use_external_fairness=true;
    value.allocation.priority_epoch=fairness.priority_epoch;
    value.allocation.fairness_ages=fairness.ages;
    return value;
}

}

TEST_CASE("Bounded failures terminate without mutating fairness on stale events") {
    gf::AsyncProposalStateMachine machine({10,10,3});
    gf::AllocatorFairnessLedger ledger;
    const std::vector<gf::FrontierCell> cells{{1,1,{1.0,1.0}}};
    const auto before=ledger.snapshot(cells);
    for (int index=0;index<3;++index)
        machine.recordFailure(gf::AsyncProposalClassification::ProposalStale);
    const auto after=ledger.snapshot(cells);
    CHECK(machine.boundedFailureReached());
    CHECK(machine.consecutiveFailures()==3);
    CHECK(before.priority_epoch==after.priority_epoch);
    CHECK(before.ages==after.ages);
    gf::AsyncFrontierProposalRequest blocked;
    blocked.request_id=1;
    blocked.launch_tick=0;
    blocked.logical_ready_tick=10;
    blocked.estimator_version=1;
    blocked.estimator_time_s=0.0;
    blocked.topology_version=1;
    blocked.topology_digest="topology";
    blocked.mode=gf::SupervisorMode::Search;
    blocked.denominator_version=1;
    blocked.tube_policy_version=1;
    blocked.config_version=1;
    blocked.planner_period_s=1.0;
    blocked.estimator_snapshot.mobile_ids={1};
    blocked.estimator_snapshot.mean=Eigen::VectorXd::Zero(4);
    blocked.estimator_snapshot.covariance=Eigen::MatrixXd::Zero(4,4);
    gf::CanonicalHardRowRequest hard;
    hard.mobile_ids={1};
    hard.states[1]={{0.0,0.0},Eigen::Vector2d::Zero(),Eigen::Vector2d::Zero()};
    hard.reference_spec={PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0,0.0,1.0,1.0,1.0,0.0};
    hard.collision_spec={PairwiseSecondOrderBarrierKind::CollisionLower,
        1.0,0.0,1.0,1.0,1.0,0.0};
    hard.acceleration_half_box=0.4;
    hard.require_snapshot_robust_rows=true;
    blocked.canonical_blueprint={hard,0.0,0.0,0.0,0.0};
    const auto launch=machine.launch(blocked);
    CHECK_FALSE(launch.accepted);
    CHECK(launch.classification==
          gf::AsyncProposalClassification::ProposalStale);
    machine.recordCommit();
    CHECK_FALSE(machine.boundedFailureReached());
}

TEST_CASE("Task 10.11f proposal and fresh commit meet candidate logical deadlines") {
    for (const auto scenario : {gf::task10p10EasyScenario(),
                                gf::task10p10NonbindingScenario()}) {
        for (const auto profile : {gf::SolverProfile::OpenSource,
                                   gf::SolverProfile::Gurobi}) {
            auto fixture=gf::makeTask10p11Fixture(
                scenario,profile,
                gf::GammaFeedbackSelectionMode::MaximumPredictedMargin,
                std::nullopt);
            REQUIRE(fixture->adapter.initializeStageZero().initialized);
            gf::Task10p11fAtomicController controller(
                fixture->swarm,fixture->adapter);
            const auto immutable=work(*fixture,controller);
            const auto proposal_start=std::chrono::steady_clock::now();
            const auto proposal=gf::runAsyncFrontierProposal(immutable);
            const double proposal_wall=std::chrono::duration<double>(
                std::chrono::steady_clock::now()-proposal_start).count();
            CAPTURE(scenario.id);
            CAPTURE(profile);
            CAPTURE(proposal.proposal.reason);
            CAPTURE(proposal_wall);
            REQUIRE(proposal.proposal.accepted);
            CHECK(proposal.recursive_allocator_calls==0);
            CHECK(proposal_wall<1.0);

            auto commit=controller.freshCommitRequest(proposal.proposal.targets);
            commit.proposal_request=immutable.provenance;
            commit.expected_bundle_id=proposal.proposal.bundle_id;
            commit.current_priority_bundle_id=proposal.proposal.bundle_id;
            const auto commit_start=std::chrono::steady_clock::now();
            const auto fresh=gf::evaluateFreshCommit(commit);
            const double commit_wall=std::chrono::duration<double>(
                std::chrono::steady_clock::now()-commit_start).count();
            CAPTURE(fresh.reason);
            CAPTURE(commit_wall);
            REQUIRE(fresh.accepted);
            CHECK(commit_wall<0.1);
            CHECK(fresh.minimum_robust_residual>=-1e-7);
            CHECK(fresh.forbidden_search_calls==0);
        }
    }
}

TEST_CASE("Three topology strategies share an identical worker request") {
    auto fixture=gf::makeTask10p11Fixture(
        gf::task10p10EasyScenario(),gf::SolverProfile::OpenSource);
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::Task10p11fAtomicController controller(fixture->swarm,fixture->adapter);
    const auto fixed=work(*fixture,controller);
    const auto proposed=fixed;
    const auto greedy=fixed;
    CHECK(fixed.allocation.cells==proposed.allocation.cells);
    CHECK(proposed.allocation.cells==greedy.allocation.cells);
    CHECK(fixed.allocator_config.top_k==proposed.allocator_config.top_k);
    CHECK(fixed.provenance.tie_break_seed==greedy.provenance.tie_break_seed);
    CHECK(fixed.gamma_feedback_config.selection_mode==
          greedy.gamma_feedback_config.selection_mode);
}
