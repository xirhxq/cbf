#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include "grand_finale/Task10p11qSafetyOnCoordinator.hpp"
#include "grand_finale/Task10p11hCoastalScenarios.hpp"
#include "grand_finale/Task10p11hSimpleCoverageController.hpp"

namespace {

class SequenceTopologySolver final : public gf::TopologySolver {
public:
    explicit SequenceTopologySolver(std::vector<gf::TopologySolution> values)
        : values_(std::move(values)) {}

    gf::TopologySolution solve(const gf::TopologyModel& model) override {
        seen_forbidden_.push_back(model.request().forbidden_topologies);
        if (cursor_>=values_.size())
            return {gf::TopologySolveStatus::Infeasible,{}, {}, {},
                    "sequence_exhausted"};
        return values_[cursor_++];
    }

    const auto& seenForbidden() const { return seen_forbidden_; }

private:
    std::vector<gf::TopologySolution> values_;
    std::size_t cursor_=0;
    std::vector<std::vector<std::vector<gf::DirectedEdge>>> seen_forbidden_;
};

gf::BoundaryPolicyConfig noBoundary() {
    gf::BoundaryPolicyConfig result;
    result.policy=gf::BoundaryPolicy::None;
    return result;
}

gf::Task10p10Scenario exactRejectionScenario() {
    auto result=gf::task10p11hCoastalBindingActiveScenario();
    // Owner 10 lies on the owner-14 -> owner-12 ray.  Replacing 11->14 by
    // 10->14 makes the two successor bearing directions collinear, while the
    // second frozen candidate 13->14 remains non-collinear.
    result.mobile_positions.at(9)={337.5,900.0};
    return result;
}

std::map<gf::NodeId,Eigen::Vector2d> rawTargets(
    const gf::Task10p11hSimpleCoverageController& controller) {
    std::map<gf::NodeId,Eigen::Vector2d> result;
    for (const auto& [owner,target]:controller.committedTargets())
        result[owner]=target.center;
    return result;
}

gf::TargetLiftProposalRequest requestFor(
    const gf::GrandFinaleSwarmAdapter& adapter,
    const std::map<gf::NodeId,Eigen::Vector2d>& raw) {
    const auto runtime=adapter.runtimeSnapshot();
    gf::TargetLiftProposalRequest request;
    request.topology=gf::task10p11hCoastalRelayProposal(adapter);
    request.transition.mobile_ids=runtime.estimate.mobile_ids;
    request.transition.old_edges=runtime.topology;
    request.transition.raw_targets=raw;
    request.transition.fixed_targets=runtime.estimate.fixed_positions;
    request.transition.add_distance_m=adapter.config().add_reference_distance_m;
    request.transition.target_margin_m=1.0;
    request.transition.topology_version=runtime.topology_token;
    request.transition.estimator_version=runtime.estimator_token;
    request.transition.raw_ledger_version=1;
    request.transition.config_version=1;
    request.geometry.mobile_ids=runtime.estimate.mobile_ids;
    request.geometry.raw_targets=raw;
    request.geometry.minimum_collision_distance_m=
        adapter.config().collision_distance_m;
    request.geometry.maximum_target_deformation_m=1.0e6;
    for (const auto& [fixed,position]:runtime.estimate.fixed_positions) {
        request.transition.fixed_ids.push_back(fixed);
        request.geometry.fixed_ids.push_back(fixed);
        request.geometry.position_support_m[fixed]=0.0;
        (void)position;
    }
    for (const auto owner:runtime.estimate.mobile_ids) {
        request.geometry.position_support_m[owner]=0.0;
        request.geometry.posterior_valid[owner]=true;
    }
    auto audited_edges=request.topology.eligible_edges;
    audited_edges.insert(audited_edges.end(),runtime.topology.begin(),
        runtime.topology.end());
    for (const auto& edge:audited_edges) {
        const auto range=gf::UndirectedEdge::canonical(
            edge.owner,edge.reference).id();
        const auto link=runtime.range_links.find(range);
        request.geometry.edge_information_valid[edge.id()]=
            link!=runtime.range_links.end();
        if (link!=runtime.range_links.end())
            request.geometry.range_variances_m2[range]=link->second.variance_m2;
    }
    request.maximum_attempts=4;
    return request;
}

gf::TopologySolution replacement(
    const std::vector<gf::DirectedEdge>& old,
    gf::DirectedEdge addition,gf::DirectedEdge removal) {
    auto next=old;
    next.erase(std::remove_if(next.begin(),next.end(),[&](const auto& edge) {
        return edge.id()==removal.id();
    }),next.end());
    next.push_back(addition);
    return {gf::TopologySolveStatus::Optimal,next,{}, {},"sequence_optimal"};
}

}

TEST_CASE("Exact-certifier rejection becomes request-local no-good before reproposal") {
    auto fixture=gf::makeTask10p11gFixture(
        exactRejectionScenario(),
        gf::SolverProfile::Gurobi,noBoundary());
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::Task10p11hSimpleCoverageController controller(
        fixture->swarm,fixture->adapter);
    REQUIRE(controller.advance().step.advanced);
    REQUIRE(controller.advance().step.advanced);
    const auto before=fixture->adapter.runtimeSnapshot();
    const auto raw=rawTargets(controller);
    auto request=requestFor(fixture->adapter,raw);
    std::string eligible;
    for (const auto& edge:request.topology.eligible_edges)
        eligible+=edge.id()+",";
    CAPTURE(eligible);
    const auto rejected=replacement(
        before.topology,{10,14},{11,14});
    const auto accepted=replacement(
        before.topology,{13,14},{11,14});
    SequenceTopologySolver solver({rejected,accepted});
    gf::Task10p11qSafetyOnCoordinator coordinator(
        fixture->swarm,fixture->adapter);

    const auto result=coordinator.proposeAndBegin(request,solver);
    std::string reasons;
    for (const auto& reason:result.rejection_reasons) reasons+=reason+";";
    INFO(result.reason," ",fixture->adapter.lastCertificationReason(),
         " target=",result.target_gate_rejections,
         " exact=",result.exact_certifier_rejections," ",reasons);
    REQUIRE(result.transition_started);
    CHECK(result.selected_topology==accepted.edges);
    CHECK(result.exact_certifier_rejections==1);
    REQUIRE(result.candidate_attributions.size()==1);
    CHECK(result.candidate_attributions.front().removal.id()=="11->14");
    CHECK(result.candidate_attributions.front().addition.id()=="10->14");
    CHECK(result.candidate_attributions.front().dag_valid);
    CHECK_FALSE(result.candidate_attributions.front().exact_rejection_reason.empty());
    CHECK(result.candidate_attributions.front().raw_targets.size()==14);
    REQUIRE(solver.seenForbidden().size()==2);
    CHECK(solver.seenForbidden()[1].size()==1);
    CHECK(solver.seenForbidden()[1][0]==rejected.edges);
    CHECK(coordinator.activeLiftedTargets()!=nullptr);
}

TEST_CASE("Rejected production candidates cannot mutate runtime before acceptance") {
    auto fixture=gf::makeTask10p11gFixture(
        exactRejectionScenario(),
        gf::SolverProfile::Gurobi,noBoundary());
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::Task10p11hSimpleCoverageController controller(
        fixture->swarm,fixture->adapter);
    REQUIRE(controller.advance().step.advanced);
    const auto before=fixture->adapter.runtimeSnapshot();
    auto request=requestFor(fixture->adapter,rawTargets(controller));
    std::string eligible;
    for (const auto& edge:request.topology.eligible_edges)
        eligible+=edge.id()+",";
    CAPTURE(eligible);
    const auto rejected=replacement(before.topology,{10,14},{11,14});
    SequenceTopologySolver solver({rejected});
    gf::Task10p11qSafetyOnCoordinator coordinator(
        fixture->swarm,fixture->adapter);

    const auto result=coordinator.proposeAndBegin(request,solver);
    std::string reasons;
    for (const auto& reason:result.rejection_reasons) reasons+=reason+";";
    INFO(result.reason," target=",result.target_gate_rejections,
         " exact=",result.exact_certifier_rejections," ",reasons);
    const auto after=fixture->adapter.runtimeSnapshot();
    CHECK_FALSE(result.transition_started);
    CHECK(result.exact_certifier_rejections==1);
    CHECK(after.runtime_s==doctest::Approx(before.runtime_s));
    CHECK(after.mode==before.mode);
    CHECK(after.topology==before.topology);
    CHECK(after.topology_token==before.topology_token);
    CHECK(after.estimator_token==before.estimator_token);
    CHECK_FALSE(after.adapter_transition_pending);
    CHECK(coordinator.activeLiftedTargets()==nullptr);
}

TEST_CASE("Formal UNION control consumes the frozen lifted target ledger") {
    auto fixture=gf::makeTask10p11gFixture(
        gf::task10p11hCoastalBindingActiveScenario(),
        gf::SolverProfile::Gurobi,noBoundary());
    REQUIRE(fixture->adapter.initializeStageZero().initialized);
    gf::Task10p11hSimpleCoverageController controller(
        fixture->swarm,fixture->adapter);
    REQUIRE(controller.advance().step.advanced);
    const auto before=fixture->adapter.runtimeSnapshot();
    auto request=requestFor(fixture->adapter,rawTargets(controller));
    const auto accepted=replacement(before.topology,{13,14},{11,14});
    SequenceTopologySolver solver({accepted});
    gf::Task10p11qSafetyOnCoordinator coordinator(
        fixture->swarm,fixture->adapter);
    REQUIRE(coordinator.proposeAndBegin(request,solver).transition_started);
    REQUIRE(coordinator.activeToken()!=nullptr);
    const auto raw_digest=gf::canonicalTargetLedgerDigest(rawTargets(controller));
    CHECK(coordinator.activeToken()->raw_ledger_digest==raw_digest);

    const auto step=controller.advanceWithFrozenTargetLift(
        *coordinator.activeToken());
    REQUIRE(step.step.advanced);
    CHECK(step.step.mode==gf::SupervisorMode::Union);
    CHECK(step.applied_target_digest==coordinator.activeToken()->lifted_digest);
    CHECK(step.step.minimum_hard_residual>=
          -fixture->adapter.config().residual_tolerance);
    CHECK(gf::canonicalTargetLedgerDigest(rawTargets(controller))==raw_digest);
    REQUIRE(coordinator.recordUnionCycle(step));
    REQUIRE(coordinator.finishFreshSuccessor());
    CHECK(gf::canonicalTargetEdges(
        fixture->adapter.runtimeSnapshot().topology)==
        gf::canonicalTargetEdges(accepted.edges));
    CHECK(coordinator.activeToken()==nullptr);
}
