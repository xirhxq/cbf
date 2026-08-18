#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/GurobiTopologySolver.hpp"
#include "grand_finale/TargetLiftProposalPrototype.hpp"
#include "grand_finale/TargetLiftUnionExecutionPrototype.hpp"
#include "grand_finale/Task10p11hCoastalScenarios.hpp"
#include "grand_finale/Task10p11hSimpleCoverageController.hpp"

namespace {

gf::BoundaryPolicyConfig noneBoundary() {
    gf::BoundaryPolicyConfig config;
    config.policy=gf::BoundaryPolicy::None;
    return config;
}

std::map<gf::NodeId,Eigen::Vector2d> rawLedger(
    const gf::Task10p11hSimpleCoverageController& controller) {
    std::map<gf::NodeId,Eigen::Vector2d> result;
    for (const auto& [owner,target] : controller.committedTargets())
        result[owner]=target.center;
    return result;
}

gf::TargetGeometryGateRequest geometryFromRuntime(
    const gf::GrandFinaleSwarmAdapter& adapter,
    const gf::FrozenTargetLiftToken& token,
    const std::map<gf::NodeId,Eigen::Vector2d>& raw) {
    const auto runtime=adapter.runtimeSnapshot();
    gf::TargetGeometryGateRequest request;
    request.mobile_ids=runtime.estimate.mobile_ids;
    for (const auto& [fixed,position] : runtime.estimate.fixed_positions) {
        request.fixed_ids.push_back(fixed);
        request.position_support_m[fixed]=0.0;
        (void)position;
    }
    request.reference_edges=token.union_edges;
    request.targets=token.lifted_targets;
    request.raw_targets=raw;
    for (const auto owner : request.mobile_ids) {
        request.position_support_m[owner]=adapter.config().uncertainty_sigma*
            std::sqrt(std::max(0.0,
                gf::detail::maximumPositionEigenvalue(runtime.estimate,owner)));
        request.posterior_valid[owner]=gf::posteriorPositionHealthy(
            runtime.estimate,owner,
            adapter.config().maximum_posterior_eigenvalue_m2);
    }
    for (const auto& edge : request.reference_edges) {
        const auto id=gf::UndirectedEdge::canonical(
            edge.owner,edge.reference).id();
        const auto link=runtime.range_links.find(id);
        request.edge_information_valid[edge.id()]=
            link!=runtime.range_links.end() &&
            link->second.age_s<=adapter.config().maximum_range_aoi_s+1e-12 &&
            link->second.quality+1e-12>=adapter.config().minimum_range_quality;
        if (link!=runtime.range_links.end())
            request.range_variances_m2[id]=link->second.variance_m2;
    }
    request.minimum_collision_distance_m=adapter.config().collision_distance_m;
    request.minimum_target_fim_planning_score=1e-6;
    request.minimum_target_cone_planning_score=1e-6;
    request.maximum_target_deformation_m=token.r_plan_m;
    return request;
}

}

TEST_CASE("14+3 coastal initial lift is nonempty from estimator target ledger") {
    auto fixture=gf::makeTask10p11gFixture(
        gf::task10p11hCoastalBindingActiveScenario(),
        gf::SolverProfile::Gurobi,noneBoundary());
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::Task10p11hSimpleCoverageController controller(
        fixture->swarm,fixture->adapter);
    REQUIRE(controller.advance().step.advanced);
    const auto runtime=fixture->adapter.runtimeSnapshot();
    gf::TargetLiftRequest request;
    request.mobile_ids=runtime.estimate.mobile_ids;
    request.reference_edges=runtime.topology;
    request.raw_targets=rawLedger(controller);
    request.fixed_targets=runtime.estimate.fixed_positions;
    for (const auto& [fixed,position] : runtime.estimate.fixed_positions) {
        request.fixed_ids.push_back(fixed);
        (void)position;
    }
    request.add_distance_m=fixture->adapter.config().add_reference_distance_m;
    request.target_margin_m=1.0;
    const auto lift=gf::liftReferenceFeasibleTargets(request);
    INFO(lift.reason);
    REQUIRE(lift.valid);
    CHECK(lift.targets.size()==17);
    CHECK(lift.maximum_edge_distance_m<=848.0+1e-10);
}

TEST_CASE("14+3 Gurobi proposal passes lifting gates certifier fresh UNION successor") {
    auto fixture=gf::makeTask10p11gFixture(
        gf::task10p11hCoastalBindingActiveScenario(),
        gf::SolverProfile::Gurobi,noneBoundary());
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::Task10p11hSimpleCoverageController controller(
        fixture->swarm,fixture->adapter);
    REQUIRE(controller.advance().step.advanced);
    REQUIRE(controller.advance().step.advanced);
    const auto raw=rawLedger(controller);
    const auto before=fixture->adapter.runtimeSnapshot();
    auto topology_request=gf::task10p11hCoastalRelayProposal(fixture->adapter);
    gf::TargetLiftProposalRequest proposal_request;
    proposal_request.topology=topology_request;
    auto& transition=proposal_request.transition;
    transition.mobile_ids=before.estimate.mobile_ids;
    for (const auto& [fixed,position] : before.estimate.fixed_positions) {
        transition.fixed_ids.push_back(fixed);
        (void)position;
    }
    transition.old_edges=before.topology;
    transition.raw_targets=raw;
    transition.fixed_targets=before.estimate.fixed_positions;
    transition.add_distance_m=fixture->adapter.config().add_reference_distance_m;
    transition.target_margin_m=1.0;
    transition.topology_version=before.topology_token;
    transition.estimator_version=before.estimator_token;
    transition.raw_ledger_version=controller.targetEpoch();
    transition.config_version=1;
    proposal_request.geometry=geometryFromRuntime(
        fixture->adapter,gf::freezeUnionTargetLift(
            gf::TargetLiftTransitionRequest{transition}),raw);
    // The candidate universe may contain edges absent from the old graph.
    for (const auto& edge : topology_request.eligible_edges) {
        const auto id=gf::UndirectedEdge::canonical(
            edge.owner,edge.reference).id();
        const auto link=before.range_links.find(id);
        proposal_request.geometry.edge_information_valid[edge.id()]=
            link!=before.range_links.end() &&
            link->second.age_s<=fixture->adapter.config().maximum_range_aoi_s+1e-12 &&
            link->second.quality+1e-12>=
                fixture->adapter.config().minimum_range_quality;
        if (link!=before.range_links.end())
            proposal_request.geometry.range_variances_m2[id]=
                link->second.variance_m2;
    }
    proposal_request.maximum_attempts=64;
    gf::GurobiTopologySolver solver;
    const auto proposal=gf::proposeTargetGeometryCandidate(
        proposal_request,solver);
    INFO(proposal.reason);
    REQUIRE(proposal.target_geometry_candidate);
    auto frozen=proposal.frozen_union;
    const auto geometry_request=geometryFromRuntime(
        fixture->adapter,frozen,raw);

    std::vector<gf::DirectedEdge> additions;
    std::vector<gf::DirectedEdge> removals;
    for (const auto& edge : proposal.selected_topology)
        if (std::none_of(before.topology.begin(),before.topology.end(),
            [&](const auto& old) { return old.id()==edge.id(); }))
            additions.push_back(edge);
    for (const auto& edge : before.topology)
        if (std::none_of(proposal.selected_topology.begin(),
            proposal.selected_topology.end(),
            [&](const auto& next) { return next.id()==edge.id(); }))
            removals.push_back(edge);
    REQUIRE(additions.size()==1);
    REQUIRE(removals.size()==1);
    gf::TargetLiftUnionExecutionPrototype executor(
        fixture->swarm,fixture->adapter);
    const auto before_wrong_edge=fixture->adapter.runtimeSnapshot();
    CHECK_FALSE(executor.begin(removals.front(),additions.front(),
        frozen,geometry_request));
    const auto after_wrong_edge=fixture->adapter.runtimeSnapshot();
    CHECK(after_wrong_edge.mode==gf::SupervisorMode::Search);
    CHECK(after_wrong_edge.topology==before_wrong_edge.topology);
    CHECK(after_wrong_edge.topology_token==before_wrong_edge.topology_token);
    REQUIRE(executor.begin(additions.front(),removals.front(),
        frozen,geometry_request));
    CHECK(fixture->adapter.lastCertificationReason()=="certified");
    const auto union_step=executor.advanceUnion();
    REQUIRE(union_step.step.advanced);
    CHECK(union_step.step.mode==gf::SupervisorMode::Union);
    CHECK(union_step.applied_target_digest==frozen.lifted_digest);
    CHECK(union_step.step.minimum_hard_residual>=
          -fixture->adapter.config().residual_tolerance);
    REQUIRE(executor.token()!=nullptr);
    CHECK(executor.token()->completed_union_cycles==1);
    REQUIRE(executor.finishAfterFreshCycle());
    const auto successor=fixture->adapter.runtimeSnapshot();
    CHECK(successor.topology==proposal.selected_topology);
    CHECK(frozen.lifted_digest!=0);
}

TEST_CASE("Rejected target gate cannot advance physical or topology state") {
    auto fixture=gf::makeTask10p11gFixture(
        gf::task10p11hCoastalBindingActiveScenario(),
        gf::SolverProfile::Gurobi,noneBoundary());
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    const auto before=fixture->adapter.runtimeSnapshot();
    gf::GurobiTopologySolver solver;
    gf::TargetLiftProposalRequest request;
    request.topology=gf::task10p11hCoastalRelayProposal(fixture->adapter);
    request.transition.mobile_ids=before.estimate.mobile_ids;
    for (const auto& [fixed,position] : before.estimate.fixed_positions) {
        request.transition.fixed_ids.push_back(fixed);
        request.transition.fixed_targets[fixed]=position;
    }
    for (std::size_t index=0;index<before.estimate.mobile_ids.size();++index)
        request.transition.raw_targets[before.estimate.mobile_ids[index]]=
            before.estimate.mean.segment<2>(4*index);
    request.transition.old_edges=before.topology;
    request.transition.topology_version=before.topology_token;
    request.transition.estimator_version=before.estimator_token;
    request.transition.raw_ledger_version=1;
    request.transition.config_version=1;
    request.geometry.mobile_ids=before.estimate.mobile_ids;
    for (const auto& [fixed,position] : before.estimate.fixed_positions) {
        request.geometry.fixed_ids.push_back(fixed);
        request.geometry.position_support_m[fixed]=0.0;
        (void)position;
    }
    for (const auto owner : before.estimate.mobile_ids) {
        request.geometry.position_support_m[owner]=0.0;
        request.geometry.posterior_valid[owner]=true;
    }
    for (const auto& edge : request.topology.eligible_edges) {
        request.geometry.edge_information_valid[edge.id()]=true;
        request.geometry.range_variances_m2[
            gf::UndirectedEdge::canonical(edge.owner,edge.reference).id()]=1.0;
    }
    request.geometry.minimum_collision_distance_m=1e6;
    request.geometry.maximum_target_deformation_m=1e6;
    request.maximum_attempts=1;
    const auto rejected=gf::proposeTargetGeometryCandidate(request,solver);
    CHECK_FALSE(rejected.target_geometry_candidate);
    CHECK(rejected.reason=="target_lift_exhausted");
    const auto after=fixture->adapter.runtimeSnapshot();
    CHECK(after.runtime_s==doctest::Approx(before.runtime_s));
    CHECK(after.estimator_token==before.estimator_token);
    CHECK(after.topology_token==before.topology_token);
    CHECK(after.topology==before.topology);
}
