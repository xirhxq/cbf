#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11rFixedBaseline.hpp"

TEST_CASE("Fixed baseline authority contract matches CBF2026 commit literals") {
    const auto authority=gf::task10p11rAuthorityContract();
    CHECK(authority.source_commit==
          "47e7b3d6f5b1be74a3dfa32150b4246c619e9a0d");
    CHECK(authority.leaders==std::vector<gf::NodeId>{7,14});
    REQUIRE(authority.branches.size()==2);
    CHECK(authority.branches[0].members==
          std::vector<gf::NodeId>{1,2,3,4,5,6,7});
    CHECK(authority.branches[1].members==
          std::vector<gf::NodeId>{8,9,10,11,12,13,14});
    CHECK(authority.branches[0].preferred_fixed_roots==
          std::vector<gf::NodeId>{101,100});
    CHECK(authority.branches[1].preferred_fixed_roots==
          std::vector<gf::NodeId>{101,102});
    CHECK(authority.initial_yaw_rad==doctest::Approx(M_PI/2.0));
    CHECK(authority.authority_control_period_s==doctest::Approx(0.5));
    CHECK(authority.grand_finale_control_period_s==doctest::Approx(0.1));
}

TEST_CASE("Fixed reference graph exactly follows two predecessor branch structure") {
    const auto topology=gf::task10p11rFixedReferenceTopology();
    const std::vector<gf::DirectedEdge> expected{
        {101,1},{100,1},{101,2},{1,2},{1,3},{2,3},{2,4},{3,4},
        {3,5},{4,5},{4,6},{5,6},{5,7},{6,7},
        {101,8},{102,8},{101,9},{8,9},{8,10},{9,10},{9,11},{10,11},
        {10,12},{11,12},{11,13},{12,13},{12,14},{13,14}};
    CHECK(topology==expected);
    CHECK(gf::referenceCountsValid(gf::task10p10MobileIds(14),topology,2));
    gf::TopologyRequest request;
    request.mobile_ids=gf::task10p10MobileIds(14);
    request.fixed_ids={100,101,102};
    request.old_edges=topology;
    request.eligible_edges=topology;
    request.min_indegree=2;
    request.max_indegree=2;
    CHECK(gf::TopologyModel(request).evaluate(topology).valid);
}

TEST_CASE("Fixed baseline target ledger uses authoritative four-section rule") {
    gf::LeaderCoverageRequest request;
    request.branches=gf::task10p11rAuthorityContract().branches;
    for (int id=1;id<=14;++id)
        request.agents.push_back({static_cast<gf::NodeId>(id),
            {id<=7?1400.0:1600.0,100.0},Eigen::Vector2d::Zero()});
    request.domain_cells={{0,0,{1400.0,1000.0}},{1,0,{1600.0,1000.0}}};
    request.uncovered_cells=request.domain_cells;
    const auto result=gf::allocateLeaderCoverageTargets(request);
    REQUIRE(result.valid);
    const Eigen::Vector2d q=result.targets.at(7).center;
    const Eigen::Vector2d origin(1500.0,-50.0);
    const Eigen::Vector2d section=(q-origin)/4.0;
    CHECK(result.targets.at(1).center.isApprox(q-3.0*section));
    CHECK(result.targets.at(2).center.isApprox(
        q-3.0*section+Eigen::Rotation2Dd(-M_PI/3.0)*section));
    CHECK(result.targets.at(7).center.isApprox(q));
}

TEST_CASE("Fixed fixture has no dynamic topology coordinator and preserves graph") {
    auto fixture=gf::makeTask10p11rFixedBaselineFixture();
    const auto initialized=fixture->adapter.initializeStageZero();
    INFO(initialized.reason);
    REQUIRE(initialized.initialized);
    const auto before=fixture->adapter.runtimeSnapshot();
    CHECK(before.mode==gf::SupervisorMode::Search);
    CHECK_FALSE(before.adapter_transition_pending);
    const auto step=fixture->controller.advance();
    INFO(step.reason);
    REQUIRE(step.step.advanced);
    const auto after=fixture->adapter.runtimeSnapshot();
    CHECK(after.topology==before.topology);
    CHECK(after.topology_token==before.topology_token);
    CHECK(after.mode==gf::SupervisorMode::Search);
    CHECK_FALSE(after.adapter_transition_pending);
    CHECK(step.step.minimum_hard_residual>=
          -fixture->adapter.config().residual_tolerance);
    CHECK(step.compute_profile.summary(
        gf::Task10p11ComputePhase::GridWorldTarget).calls==1);
    CHECK(step.compute_profile.summary(
        gf::Task10p11ComputePhase::CurrentCanonicalRowRebuild).calls==1);
    CHECK(step.compute_profile.summary(
        gf::Task10p11ComputePhase::PredictedCanonicalRowRebuild).calls>0);
    CHECK(step.compute_profile.summary(
        gf::Task10p11ComputePhase::FinalQp).calls==14);
    CHECK(step.compute_profile.summary(
        gf::Task10p11ComputePhase::OnlineEstimator).calls==1);
    CHECK(step.compute_profile.summary(
        gf::Task10p11ComputePhase::PlantPreflightZoh).calls==1);
}

TEST_CASE("Fixed baseline metric snapshot separates information and reference graph semantics") {
    auto fixture=gf::makeTask10p11rFixedBaselineFixture();
    const auto initialized=fixture->adapter.initializeStageZero();
    INFO(initialized.reason);
    REQUIRE(initialized.initialized);
    const auto step=fixture->controller.advance();
    INFO(step.reason);
    REQUIRE(step.step.advanced);
    CHECK(std::isfinite(step.step.minimum_collision_h));
    CHECK(std::isfinite(step.step.minimum_collision_psi1));
    CHECK(std::isfinite(step.step.minimum_collision_residual));
    CHECK(std::isfinite(step.step.minimum_reference_h));
    CHECK(std::isfinite(step.step.minimum_reference_psi1));
    CHECK(std::isfinite(step.step.minimum_reference_residual));

    const auto information=fixture->adapter.currentReferenceAudit();
    const auto rows=fixture->adapter.currentSnapshotHardRows(
        fixture->adapter.runtimeSnapshot().topology);
    const auto metrics=gf::task10p11rFixedMetricSnapshot(information,rows);

    CHECK(metrics.accepted_information_nominal_fim==
          doctest::Approx(information.minimum_fim_eigenvalue));
    CHECK(metrics.accepted_information_robust_fim==
          doctest::Approx(information.minimum_robust_fim_cone_lower_bound));
    CHECK(metrics.reference_topology_fim_proxy==
          doctest::Approx(information.minimum_reference_only_fim_eigenvalue));
    CHECK(metrics.reference_topology_robust_fim_proxy==
          doctest::Approx(
              information.minimum_reference_only_robust_fim_cone_lower_bound));
    CHECK(metrics.minimum_effective_reference_count>=2);
    CHECK(std::isfinite(metrics.minimum_collision_h));
    CHECK(std::isfinite(metrics.minimum_collision_psi1));
    CHECK(std::isfinite(metrics.minimum_reference_h));
    CHECK(std::isfinite(metrics.minimum_reference_psi1));
}
