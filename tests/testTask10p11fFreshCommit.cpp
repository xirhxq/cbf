#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/AsyncFrontierFreshCommit.hpp"

namespace {

gf::FreshCommitRequest fresh(gf::SolverProfile profile) {
    gf::FreshCommitRequest value;
    value.proposal_request.request_id=1;
    value.proposal_request.launch_tick=0;
    value.proposal_request.logical_ready_tick=10;
    value.proposal_request.estimator_version=1;
    value.proposal_request.estimator_time_s=0.0;
    value.proposal_request.topology_version=1;
    value.proposal_request.topology_digest="100->1";
    value.proposal_request.mode=gf::SupervisorMode::Search;
    value.proposal_request.target_epoch=1;
    value.proposal_request.denominator_version=1;
    value.proposal_request.frontier_mark_version=1;
    value.proposal_request.tube_policy_version=1;
    value.proposal_request.config_version=1;
    value.proposal_request.profile=profile;
    value.proposal_request.tie_break_seed=2027;
    value.proposal_request.planner_period_s=1.0;
    value.latest=value.proposal_request;
    value.latest.estimator_version=2;
    value.latest.estimator_time_s=0.1;
    value.latest.estimator_snapshot.mobile_ids={1};
    value.latest.estimator_snapshot.mean=Eigen::VectorXd::Zero(4);
    value.latest.estimator_snapshot.mean<<2.0,2.0,0.0,0.0;
    value.latest.estimator_snapshot.covariance=
        1e-6*Eigen::MatrixXd::Identity(4,4);
    gf::CanonicalHardRowRequest hard;
    hard.mobile_ids={1};
    hard.fixed_ids={100};
    hard.states[1]={{2.0,2.0},Eigen::Vector2d::Zero(),Eigen::Vector2d::Zero()};
    hard.states[100]={{5.0,2.0},Eigen::Vector2d::Zero(),Eigen::Vector2d::Zero()};
    hard.reference_edges={{100,1}};
    hard.collision_pairs={gf::UndirectedEdge::canonical(1,100)};
    hard.reference_spec={PairwiseSecondOrderBarrierKind::CommunicationUpper,
        850.0,0.0,1.0,1.0,1.0,0.0};
    hard.collision_spec={PairwiseSecondOrderBarrierKind::CollisionLower,
        1.0,0.0,1.0,1.0,1.0,0.0};
    hard.acceleration_half_box=0.4;
    hard.require_snapshot_robust_rows=true;
    hard.workspace_facets={{"x-upper",{1.0,0.0},20.0}};
    value.latest.canonical_blueprint={hard,3.0,0.02,0.03,0.04};
    value.proposal_request.estimator_snapshot=value.latest.estimator_snapshot;
    value.proposal_request.canonical_blueprint=value.latest.canonical_blueprint;
    value.targets={{1,{4,2,{4.0,2.0}}}};
    value.uncovered_cell_ids={"4:2"};
    value.denominator_cell_ids={"4:2","5:2"};
    value.expected_bundle_id="1=4:2;";
    value.current_priority_bundle_id="1=4:2;";
    value.minimum_effective_reference_count=2;
    value.minimum_robust_fim_margin=0.1;
    value.minimum_posterior_margin=0.1;
    value.minimum_aoi_margin=0.1;
    value.position_gain=0.4;
    value.velocity_gain=0.8;
    value.dt_s=0.1;
    value.estimator_acceleration_variance=0.0;
    value.gamma_config={0.4,2,
        gf::GammaFeedbackSelectionMode::DiagnosticsOnly,std::nullopt,1e-10};
    return value;
}

}

TEST_CASE("Fresh commit rejects every structural and information gate before control") {
    auto value=fresh(gf::SolverProfile::OpenSource);
    value.latest.mode=gf::SupervisorMode::Union;
    CHECK(gf::evaluateFreshCommit(value).reason=="fresh_commit_rejected:mode");
    value=fresh(gf::SolverProfile::OpenSource);
    value.pending_union=true;
    CHECK(gf::evaluateFreshCommit(value).reason=="fresh_commit_rejected:pending_transition");
    value=fresh(gf::SolverProfile::OpenSource);
    value.latest.topology_version++;
    CHECK(gf::evaluateFreshCommit(value).reason=="fresh_commit_rejected:stale_structure");
    value=fresh(gf::SolverProfile::OpenSource);
    value.proposal_request.mode=gf::SupervisorMode::Hold;
    CHECK(gf::evaluateFreshCommit(value).reason=="fresh_commit_rejected:stale_structure");
    value=fresh(gf::SolverProfile::OpenSource);
    value.uncovered_cell_ids.clear();
    CHECK(gf::evaluateFreshCommit(value).reason=="fresh_commit_rejected:cell_covered");
    value=fresh(gf::SolverProfile::OpenSource);
    value.minimum_effective_reference_count=1;
    CHECK(gf::evaluateFreshCommit(value).reason=="fresh_commit_rejected:double_reference");
    value=fresh(gf::SolverProfile::OpenSource);
    value.minimum_robust_fim_margin=-1e-6;
    CHECK(gf::evaluateFreshCommit(value).reason=="fresh_commit_rejected:robust_fim");
}

TEST_CASE("Fresh commit uses latest canonical gamma QP and residual without search") {
    for (const auto profile : {gf::SolverProfile::OpenSource,
                               gf::SolverProfile::Gurobi}) {
        const auto result=gf::evaluateFreshCommit(fresh(profile));
        CAPTURE(result.reason);
        REQUIRE(result.accepted);
        CHECK(result.forbidden_search_calls==0);
        CHECK(result.minimum_current_gamma>=-1e-10);
        CHECK(result.minimum_robust_residual>=-1e-7);
        CHECK(result.controls.size()==1);
        CHECK_FALSE(result.row_ledger.empty());
    }
}

TEST_CASE("Negative current gamma fails closed before any provisional apply") {
    auto value=fresh(gf::SolverProfile::OpenSource);
    value.latest.canonical_blueprint.hard_template.workspace_facets=
        {{"x-upper",{1.0,0.0},1.0}};
    const auto result=gf::evaluateFreshCommit(value);
    CHECK_FALSE(result.accepted);
    CHECK(result.reason=="fresh_commit_rejected:hard_polytope_empty");
    CHECK(result.controls.empty());
}
