#pragma once

#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "grand_finale/TargetLiftProposalPrototype.hpp"
#include "grand_finale/Task10p11hSimpleCoverageController.hpp"
#include "grand_finale/Task10p11rStopAttribution.hpp"

namespace gf {

struct Task10p11qProposalAudit {
    bool transition_started=false;
    std::string reason;
    std::vector<DirectedEdge> selected_topology;
    std::size_t target_gate_rejections=0;
    std::size_t exact_certifier_rejections=0;
    std::size_t no_good_rejections=0;
    std::vector<std::vector<DirectedEdge>> request_local_no_goods;
    std::vector<std::string> rejection_reasons;
    std::vector<ReplacementCandidateAttribution> candidate_attributions;
    std::string solver_detail;
};

// Production orchestration only: the adapter remains the sole owner of the
// exact transition certifier, hybrid transition state, robust QP, and physical
// advance.  No-good state lives only for the duration of proposeAndBegin().
class Task10p11qSafetyOnCoordinator {
public:
    Task10p11qSafetyOnCoordinator(
        Swarm& swarm,GrandFinaleSwarmAdapter& adapter)
        : swarm_(swarm),adapter_(adapter) {}

    Task10p11qProposalAudit proposeAndBegin(
        TargetLiftProposalRequest request,TopologySolver& solver) {
        Task10p11qProposalAudit audit;
        if (active_.has_value()) {
            audit.reason="target_lift_transition_already_active";
            return audit;
        }
        const auto initial=adapter_.runtimeSnapshot();
        if (initial.mode!=SupervisorMode::Search ||
            initial.adapter_transition_pending ||
            request.maximum_attempts==0) {
            audit.reason="target_lift_request_not_admissible";
            return audit;
        }
        request.topology.old_edges=initial.topology;
        request.transition.old_edges=initial.topology;
        request.transition.topology_version=initial.topology_token;
        request.transition.estimator_version=initial.estimator_token;

        for (std::size_t attempt=0;attempt<request.maximum_attempts;++attempt) {
            const TopologyModel model(request.topology);
            const TopologySolution proposal=solver.solve(model);
            audit.solver_detail=proposal.detail;
            if (proposal.status!=TopologySolveStatus::Optimal) {
                audit.reason=proposal.status==TopologySolveStatus::Infeasible
                    ?(audit.no_good_rejections==0?
                        "no_topology_geometry_candidate":
                        "certifier_candidate_exhausted")
                    :"topology_solver_failure:"+proposal.detail;
                return audit;
            }
            const auto model_audit=model.evaluate(proposal.edges);
            if (!model_audit.valid) {
                audit.reason="topology_solver_invalid_solution:"+
                    model_audit.reason;
                return audit;
            }

            TargetLiftTransitionRequest transition=request.transition;
            transition.successor_edges=proposal.edges;
            FrozenTargetLiftToken token=freezeUnionTargetLift(transition);
            TargetGeometryGateRequest geometry=request.geometry;
            geometry.mobile_ids=transition.mobile_ids;
            geometry.fixed_ids=transition.fixed_ids;
            geometry.reference_edges=token.union_edges;
            geometry.targets=token.lifted_targets;
            geometry.raw_targets=transition.raw_targets;
            const TargetGeometryGateAudit target_gate=token.valid
                ?evaluateTargetGeometryGates(geometry):TargetGeometryGateAudit{};
            std::string rejection;
            bool exact_rejection=false;
            if (!token.valid) rejection=token.reason;
            else if (!target_gate.valid) rejection=target_gate.reason;
            else {
                const auto additions=targetEdgeDifference(
                    token.successor_edges,token.old_edges);
                const auto removals=targetEdgeDifference(
                    token.old_edges,token.successor_edges);
                if (additions.size()!=1 || removals.size()!=1) {
                    rejection="not_single_replacement";
                } else if (!adapter_.beginReplacement(
                        additions.front(),removals.front(),false)) {
                    rejection=adapter_.lastCertificationReason();
                    exact_rejection=true;
                } else {
                    const auto after=adapter_.runtimeSnapshot();
                    if (after.mode!=SupervisorMode::Union ||
                        !after.adapter_transition_pending ||
                        canonicalTargetEdges(after.topology)!=token.union_edges) {
                        audit.reason="accepted_union_state_mismatch";
                        return audit;
                    }
                    active_=Active{std::move(token),std::move(geometry)};
                    audit.transition_started=true;
                    audit.reason="certified_target_lift_transition_started";
                    audit.selected_topology=proposal.edges;
                    return audit;
                }
            }
            if (exact_rejection) ++audit.exact_certifier_rejections;
            else ++audit.target_gate_rejections;
            audit.candidate_attributions.push_back(attributeCandidate(
                initial,proposal.edges,token,target_gate,
                request.transition.raw_targets,rejection));
            ++audit.no_good_rejections;
            request.topology.forbidden_topologies.push_back(proposal.edges);
            audit.request_local_no_goods.push_back(proposal.edges);
            audit.rejection_reasons.push_back(rejection);
            audit.reason="candidate_rejected:"+rejection;
        }
        audit.reason="certifier_candidate_exhausted";
        return audit;
    }

    const std::map<NodeId,Eigen::Vector2d>* activeLiftedTargets() const {
        return active_.has_value()?&active_->token.lifted_targets:nullptr;
    }

    const FrozenTargetLiftToken* activeToken() const {
        return active_.has_value()?&active_->token:nullptr;
    }

    bool recordUnionCycle(const SimpleCoverageControlStep& control) {
        if (!active_.has_value()) return false;
        const auto after=adapter_.runtimeSnapshot();
        FrozenUnionCycleEvidence evidence;
        evidence.mode=control.step.mode;
        evidence.active_edges=after.topology;
        evidence.applied_target_digest=control.applied_target_digest;
        evidence.raw_ledger_version=active_->token.raw_ledger_version;
        evidence.raw_ledger_digest=active_->token.raw_ledger_digest;
        evidence.config_version=active_->token.config_version;
        evidence.config_digest=active_->token.config_digest;
        evidence.topology_version=control.step.topology_version;
        evidence.estimator_version_before=
            control.step.estimator_version_before;
        evidence.estimator_version_after=control.step.estimator_version_after;
        evidence.physical_advanced=control.step.advanced;
        evidence.exact_zoh_completed=control.step.advanced;
        evidence.minimum_hard_residual=control.step.minimum_hard_residual;
        evidence.residual_tolerance=adapter_.config().residual_tolerance;
        active_->token=recordFreshUnionControlCycle(
            active_->token,std::move(evidence));
        return active_->token.valid &&
            active_->token.completed_union_cycles>=1;
    }

    bool finishFreshSuccessor() {
        if (!active_.has_value()) return false;
        const auto runtime=adapter_.runtimeSnapshot();
        if (!freshBreakTargetReady(active_->token,runtime.topology_token,
                runtime.estimator_token) ||
            !adapter_.finishReplacementAfterFreshCycle())
            return false;
        active_.reset();
        return true;
    }

private:
    ReplacementCandidateAttribution attributeCandidate(
        const GrandFinaleRuntimeSnapshot& runtime,
        const std::vector<DirectedEdge>& successor,
        const FrozenTargetLiftToken& token,
        const TargetGeometryGateAudit& target_gate,
        const std::map<NodeId,Eigen::Vector2d>& raw_targets,
        const std::string& rejection) const {
        ReplacementCandidateAttribution value;
        const auto additions=targetEdgeDifference(successor,runtime.topology);
        const auto removals=targetEdgeDifference(runtime.topology,successor);
        if (additions.size()!=1 || removals.size()!=1) {
            value.exact_rejection_reason=rejection;
            return value;
        }
        value.addition=additions.front();
        value.removal=removals.front();
        value.raw_targets=raw_targets;
        value.lifted_targets=token.lifted_targets;
        value.reference_lens_valid=token.valid;
        value.target_fim_planning_score=
            target_gate.minimum_target_fim_planning_score;
        value.target_cone_planning_score=
            target_gate.minimum_target_cone_planning_score;
        value.exact_rejection_reason=rejection;

        TopologyRequest topology_request;
        topology_request.mobile_ids=runtime.estimate.mobile_ids;
        for (const auto& [fixed,position]:runtime.estimate.fixed_positions) {
            (void)position;
            topology_request.fixed_ids.push_back(fixed);
        }
        topology_request.old_edges=runtime.topology;
        topology_request.eligible_edges=successor;
        topology_request.min_indegree=2;
        topology_request.max_indegree=2;
        value.dag_valid=TopologyModel(topology_request).evaluate(successor).valid;

        const auto& config=adapter_.config();
        const Eigen::Vector2d owner=detail::nodePosition(
            runtime.estimate,value.addition.owner);
        const Eigen::Vector2d reference=detail::nodePosition(
            runtime.estimate,value.addition.reference);
        const double support=config.uncertainty_sigma*(std::sqrt(std::max(
            0.0,detail::maximumPositionEigenvalue(
                runtime.estimate,value.addition.owner)))+std::sqrt(std::max(
            0.0,detail::maximumPositionEigenvalue(
                runtime.estimate,value.addition.reference))));
        const double robust_distance=(owner-reference).norm()+support;
        value.add_distance_valid=robust_distance<=
            config.add_reference_distance_m+1e-12;
        value.keep_distance_valid=robust_distance<=
            config.reference_distance_m+1e-12;
        const std::string range_id=UndirectedEdge::canonical(
            value.addition.owner,value.addition.reference).id();
        const auto link=runtime.range_links.find(range_id);
        value.aoi_valid=link!=runtime.range_links.end() &&
            link->second.age_s<=config.maximum_range_aoi_s+1e-12;
        value.quality_valid=link!=runtime.range_links.end() &&
            link->second.quality+1e-12>=config.minimum_range_quality;
        value.posterior_valid=posteriorPositionHealthy(runtime.estimate,
            value.addition.owner,config.maximum_posterior_eigenvalue_m2);

        std::vector<DirectedEdge> owner_edges;
        std::map<std::string,double> variances;
        bool ranges_complete=true;
        for (const auto& edge:successor) if (edge.owner==value.addition.owner) {
            owner_edges.push_back(edge);
            const std::string id=UndirectedEdge::canonical(
                edge.owner,edge.reference).id();
            const auto found=runtime.range_links.find(id);
            if (found==runtime.range_links.end()) ranges_complete=false;
            else variances[id]=found->second.variance_m2;
        }
        if (ranges_complete && owner_edges.size()>=2) {
            Eigen::Matrix2d nominal=Eigen::Matrix2d::Zero();
            for (const auto& edge:owner_edges) {
                const Eigen::Vector2d delta=detail::nodePosition(
                    runtime.estimate,edge.owner)-detail::nodePosition(
                    runtime.estimate,edge.reference);
                const Eigen::Vector2d direction=delta.normalized();
                nominal+=direction*direction.transpose()/variances.at(
                    UndirectedEdge::canonical(edge.owner,edge.reference).id());
            }
            value.nominal_fim_eigenvalue=
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(nominal)
                    .eigenvalues().minCoeff();
            value.posterior_fim_proxy_eigenvalue=
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(referenceFim(
                    value.addition.owner,owner_edges,runtime.estimate,variances))
                    .eigenvalues().minCoeff();
            value.robust_cone_fim_lower_bound=robustReferenceFimConeLowerBound(
                value.addition.owner,owner_edges,runtime.estimate,variances,
                config.uncertainty_sigma);
            value.fim_valid=value.robust_cone_fim_lower_bound>=1e-6;
        }
        return value;
    }

    struct Active {
        FrozenTargetLiftToken token;
        TargetGeometryGateRequest geometry;
    };

    Swarm& swarm_;
    GrandFinaleSwarmAdapter& adapter_;
    std::optional<Active> active_;
};

}  // namespace gf
