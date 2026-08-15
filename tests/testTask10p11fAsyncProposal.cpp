#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/AsyncFrontierProposal.hpp"
#include "grand_finale/AsyncFrontierProposalKernel.hpp"

#include <thread>

namespace {

gf::JointEstimateSnapshot snapshot(double x=2.0) {
    gf::JointEstimateSnapshot value;
    value.mobile_ids={1};
    value.mean=Eigen::VectorXd::Zero(4);
    value.mean<<x,3.0,0.2,-0.1;
    value.covariance=1e-4*Eigen::MatrixXd::Identity(4,4);
    return value;
}

gf::CanonicalHardRowRequest hardTemplate() {
    gf::CanonicalHardRowRequest value;
    value.mobile_ids={1};
    value.fixed_ids={100};
    value.states[1]={{2.0,3.0},{0.2,-0.1},Eigen::Vector2d::Zero()};
    value.states[100]={{9.0,3.0},Eigen::Vector2d::Zero(),Eigen::Vector2d::Zero()};
    value.reference_edges={{100,1}};
    value.collision_pairs={gf::UndirectedEdge::canonical(1,100)};
    value.reference_spec={PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0,0.0,1.0,1.0,1.0,0.0};
    value.collision_spec={PairwiseSecondOrderBarrierKind::CollisionLower,
        1.0,0.0,1.0,1.0,1.0,0.0};
    value.acceleration_half_box=0.4;
    value.require_snapshot_robust_rows=true;
    value.workspace_facets={{"x-upper",{1.0,0.0},20.0}};
    return value;
}

gf::CanonicalPredictionBlueprint blueprint() {
    return gf::CanonicalPredictionBlueprint{
        hardTemplate(),3.0,0.02,0.03,0.04};
}

gf::AsyncFrontierProposalRequest request() {
    gf::AsyncFrontierProposalRequest value;
    value.request_id=7;
    value.launch_tick=10;
    value.logical_ready_tick=20;
    value.estimator_version=100;
    value.estimator_time_s=1.0;
    value.topology_version=3;
    value.topology_digest="100->1";
    value.mode=gf::SupervisorMode::Search;
    value.target_epoch=4;
    value.denominator_version=5;
    value.frontier_mark_version=6;
    value.tube_policy_version=7;
    value.config_version=8;
    value.profile=gf::SolverProfile::OpenSource;
    value.tie_break_seed=2027;
    value.estimator_snapshot=snapshot();
    value.canonical_blueprint=blueprint();
    value.planner_period_s=1.0;
    return value;
}

}

TEST_CASE("Canonical prediction blueprint is a value copy and rebuilds snapshot rows") {
    auto source=hardTemplate();
    gf::CanonicalPredictionBlueprint value{source,3.0,0.02,0.03,0.04};
    source.states[100].position.x=999.0;
    const auto rebuilt=value.build(snapshot(4.0));
    const auto rows=gf::buildCanonicalHardRows(rebuilt);

    CHECK(rebuilt.states.at(1).position.x==doctest::Approx(4.0));
    CHECK(rebuilt.states.at(100).position.x==doctest::Approx(9.0));
    CHECK(rebuilt.workspace_snapshot_tubes.at(1).position_radius_m>
          value.shadow_single_position_support_m);
    CHECK_FALSE(rows.empty());
}

TEST_CASE("Async request validates structural and timing provenance") {
    auto value=request();
    CHECK_NOTHROW(value.validate());
    value.logical_ready_tick=value.launch_tick;
    CHECK_THROWS_WITH(value.validate(),"async proposal ready tick must follow launch");
}

TEST_CASE("Logical mailbox is single-flight and observes only at ready tick") {
    gf::AsyncProposalStateMachine machine({10,10,3});
    const auto launched=machine.launch(request());
    REQUIRE(launched.accepted);
    CHECK_FALSE(machine.launch(request()).accepted);
    auto result=gf::AsyncFrontierProposalResult::acceptedFor(request(),"1=3:4;");
    CHECK(machine.complete(result,0.4).accepted);
    CHECK(machine.observe(19,request()).classification==
          gf::AsyncProposalClassification::NotReady);
    CHECK(machine.observe(20,request()).classification==
          gf::AsyncProposalClassification::Ready);
}

TEST_CASE("Structural change cancels while estimator realization uses age bound") {
    gf::AsyncProposalStateMachine machine({10,10,3});
    auto launched=request();
    REQUIRE(machine.launch(launched).accepted);
    REQUIRE(machine.complete(
        gf::AsyncFrontierProposalResult::acceptedFor(launched,"1=3:4;"),0.4).accepted);
    auto current=launched;
    current.estimator_version+=1000;
    current.estimator_time_s=1.2;
    CHECK(machine.observe(20,current).classification==
          gf::AsyncProposalClassification::Ready);

    launched.request_id++;
    REQUIRE(machine.launch(launched).accepted);
    REQUIRE(machine.complete(
        gf::AsyncFrontierProposalResult::acceptedFor(launched,"1=3:4;"),0.4).accepted);
    current=launched;
    current.topology_version++;
    CHECK(machine.observe(20,current).classification==
          gf::AsyncProposalClassification::ProposalStale);
}

TEST_CASE("Coverage marks may advance but denominator tube config and mode invalidate") {
    const auto completed_result=[](
        gf::AsyncProposalStateMachine& machine,
        const gf::AsyncFrontierProposalRequest& launched) {
        REQUIRE(machine.launch(launched).accepted);
        REQUIRE(machine.complete(
            gf::AsyncFrontierProposalResult::acceptedFor(launched,"1=3:4;"),
            0.1).accepted);
    };

    SUBCASE("ordinary frontier marks are revalidated at commit") {
        gf::AsyncProposalStateMachine machine({10,10,3});
        const auto launched=request();
        completed_result(machine,launched);
        auto current=launched;
        ++current.frontier_mark_version;
        CHECK(machine.observe(20,current).classification==
              gf::AsyncProposalClassification::Ready);
    }
    SUBCASE("denominator changes are structural") {
        gf::AsyncProposalStateMachine machine({10,10,3});
        const auto launched=request();
        completed_result(machine,launched);
        auto current=launched;
        ++current.denominator_version;
        CHECK(machine.observe(20,current).classification==
              gf::AsyncProposalClassification::ProposalStale);
    }
    SUBCASE("tube policy changes are structural") {
        gf::AsyncProposalStateMachine machine({10,10,3});
        const auto launched=request();
        completed_result(machine,launched);
        auto current=launched;
        ++current.tube_policy_version;
        CHECK(machine.observe(20,current).classification==
              gf::AsyncProposalClassification::ProposalStale);
    }
    SUBCASE("configuration changes are structural") {
        gf::AsyncProposalStateMachine machine({10,10,3});
        const auto launched=request();
        completed_result(machine,launched);
        auto current=launched;
        ++current.config_version;
        CHECK(machine.observe(20,current).classification==
              gf::AsyncProposalClassification::ProposalStale);
    }
    SUBCASE("mode changes are structural") {
        gf::AsyncProposalStateMachine machine({10,10,3});
        const auto launched=request();
        completed_result(machine,launched);
        auto current=launched;
        current.mode=gf::SupervisorMode::Union;
        CHECK(machine.observe(20,current).classification==
              gf::AsyncProposalClassification::ProposalStale);
    }
}

TEST_CASE("Request identifiers must increase monotonically") {
    gf::AsyncProposalStateMachine machine({10,10,3});
    auto value=request();
    REQUIRE(machine.launch(value).accepted);
    REQUIRE(machine.complete(
        gf::AsyncFrontierProposalResult::acceptedFor(value,"1=3:4;"),0.1).accepted);
    REQUIRE(machine.observe(20,value).classification==
            gf::AsyncProposalClassification::Ready);
    const auto replay=machine.launch(value);
    CHECK_FALSE(replay.accepted);
    CHECK(replay.classification==gf::AsyncProposalClassification::OutOfOrder);
}

TEST_CASE("Late cancelled and out-of-order results never become ready") {
    gf::AsyncProposalStateMachine machine({10,10,3});
    auto first=request();
    REQUIRE(machine.launch(first).accepted);
    auto late=gf::AsyncFrontierProposalResult::acceptedFor(first,"1=3:4;");
    CHECK(machine.complete(late,1.01).classification==
          gf::AsyncProposalClassification::ProposalDeadlineMissed);
    CHECK(machine.observe(20,first).classification==
          gf::AsyncProposalClassification::ProposalDeadlineMissed);

    first.request_id=8;
    first.launch_tick=20;
    first.logical_ready_tick=30;
    REQUIRE(machine.launch(first).accepted);
    machine.cancel("mode_changed");
    CHECK(machine.complete(
        gf::AsyncFrontierProposalResult::acceptedFor(first,"1=3:4;"),0.1)
              .classification==gf::AsyncProposalClassification::ProposalCancelled);
}

TEST_CASE("Fairness ledger changes only on explicit online decisions") {
    gf::AllocatorFairnessLedger ledger;
    const std::vector<gf::FrontierCell> cells{
        {1,1,{1.0,1.0}},{2,2,{2.0,2.0}}};
    const auto before=ledger.snapshot(cells);
    const auto repeated=ledger.snapshot(cells);
    CHECK(before.priority_epoch==repeated.priority_epoch);
    CHECK(before.ages==repeated.ages);
    ledger.recordCommit(cells,{{1,cells.front()}});
    const auto committed=ledger.snapshot(cells);
    CHECK(committed.priority_epoch==1);
    CHECK(committed.ages.at(cells.front().id())==0);
    CHECK(committed.ages.at(cells.back().id())==1);
}

TEST_CASE("Pure worker repeats candidate and keeps no-good request-local") {
    gf::AsyncFrontierWorkRequest work;
    work.provenance=request();
    work.allocator_config={8,32,4,5,5,0.8,0.05,1e-10};
    work.allocation.snapshot_token=100;
    work.allocation.topology_token=3;
    work.allocation.grid_token=6;
    work.allocation.agents={{1,{2.0,3.0},{0.2,-0.1}}};
    work.allocation.cells={{1,1,{4.0,3.0}},{2,2,{2.0,5.0}}};
    work.allocation.acceleration_half_box=0.4;
    work.allocation.collision_distance_m=1.0;
    work.allocation.position_reserve_m=0.05;
    work.allocation.velocity_reserve_mps=0.05;
    work.allocation.position_gain=0.4;
    work.allocation.velocity_gain=0.8;
    work.fixed_positions={{100,{9.0,3.0}}};
    work.dt_s=0.1;
    work.estimator_acceleration_variance=0.0;
    work.gamma_feedback_config={0.4,2,
        gf::GammaFeedbackSelectionMode::DiagnosticsOnly,std::nullopt,1e-10};
    const auto first=gf::runAsyncFrontierProposal(work);
    const auto second=gf::runAsyncFrontierProposal(work);
    REQUIRE(first.proposal.accepted);
    CHECK(second.proposal.bundle_id==first.proposal.bundle_id);
    CHECK(second.candidate_bundles==first.candidate_bundles);
    CHECK(first.recursive_allocator_calls==0);
}

TEST_CASE("Slow worker owns an immutable value copy and is single-flight") {
    gf::AsyncFrontierWorkRequest work;
    work.provenance=request();
    work.allocator_config={8,32,4,5,5,0.8,0.05,1e-10};
    work.allocation.snapshot_token=100;
    work.allocation.topology_token=3;
    work.allocation.grid_token=6;
    work.allocation.agents={{1,{2.0,3.0},{0.2,-0.1}}};
    work.allocation.cells={{1,1,{4.0,3.0}},{2,2,{2.0,5.0}}};
    work.allocation.acceleration_half_box=0.4;
    work.allocation.collision_distance_m=1.0;
    work.allocation.position_reserve_m=0.05;
    work.allocation.velocity_reserve_mps=0.05;
    work.allocation.position_gain=0.4;
    work.allocation.velocity_gain=0.8;
    work.fixed_positions={{100,{9.0,3.0}}};
    work.dt_s=0.1;
    work.gamma_feedback_config={0.4,2,
        gf::GammaFeedbackSelectionMode::DiagnosticsOnly,std::nullopt,1e-10};
    gf::AsyncFrontierProposalWorker worker;
    REQUIRE(worker.launch(work));
    CHECK_FALSE(worker.launch(work));
    work.allocation.cells.front().x_index=999;
    std::optional<gf::AsyncWorkerCompletion> completed;
    for (std::size_t spin=0;spin<1000000 && !completed.has_value();++spin) {
        completed=worker.tryCollect();
        std::this_thread::yield();
    }
    REQUIRE(completed.has_value());
    CHECK(completed->work.proposal.accepted);
    CHECK(completed->work.proposal.bundle_id.find("999:")==std::string::npos);
    CHECK(completed->wall_s<1.0);
}
