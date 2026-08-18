#pragma once

#include "grand_finale/TargetLiftTransitionPrototype.hpp"
#include "grand_finale/TopologySolver.hpp"

namespace gf {

struct TargetLiftProposalRequest {
    TopologyRequest topology;
    TargetLiftTransitionRequest transition;
    TargetGeometryGateRequest geometry;
    std::size_t maximum_attempts=1;
};

struct TargetLiftProposalResult {
    bool target_geometry_candidate=false;
    std::string reason;
    std::vector<DirectedEdge> selected_topology;
    FrozenTargetLiftToken frozen_union;
    TargetGeometryGateAudit geometry;
    std::size_t no_good_rejections=0;
    std::vector<std::vector<DirectedEdge>> request_local_no_goods;
    std::vector<std::string> rejection_reasons;
    std::string solver_detail;
};

// This function deliberately stops before the exact transition certifier.  It
// cannot produce a certified topology: it only returns a typed union-lifted
// candidate that has passed the built-in target planning gates.
inline TargetLiftProposalResult proposeTargetGeometryCandidate(
    TargetLiftProposalRequest request,TopologySolver& solver) {
    TargetLiftProposalResult result;
    if (request.maximum_attempts==0) {
        result.reason="invalid_target_lift_attempt_limit";
        return result;
    }
    request.topology.old_edges=request.transition.old_edges;
    for (std::size_t attempt=0;attempt<request.maximum_attempts;++attempt) {
        const TopologySolution proposal=solver.solve(
            TopologyModel(request.topology));
        result.solver_detail=proposal.detail;
        if (proposal.status!=TopologySolveStatus::Optimal) {
            result.reason=proposal.status==TopologySolveStatus::Infeasible
                ?(result.no_good_rejections==0?
                    "no_topology_geometry_candidate":
                                              "target_lift_exhausted")
                :"topology_solver_failure";
            return result;
        }
        TargetLiftTransitionRequest transition=request.transition;
        transition.successor_edges=proposal.edges;
        FrozenTargetLiftToken frozen=freezeUnionTargetLift(transition);
        TargetGeometryGateAudit geometry;
        std::string rejection;
        if (!frozen.valid) {
            rejection=frozen.reason;
        } else {
            TargetGeometryGateRequest gate=request.geometry;
            gate.mobile_ids=transition.mobile_ids;
            gate.fixed_ids=transition.fixed_ids;
            gate.reference_edges=frozen.union_edges;
            gate.targets=frozen.lifted_targets;
            gate.raw_targets=transition.raw_targets;
            geometry=evaluateTargetGeometryGates(gate);
            if (!geometry.valid) rejection=geometry.reason;
        }
        if (rejection.empty()) {
            result.target_geometry_candidate=true;
            result.reason="target_geometry_candidate";
            result.selected_topology=proposal.edges;
            result.frozen_union=std::move(frozen);
            result.geometry=std::move(geometry);
            return result;
        }
        request.topology.forbidden_topologies.push_back(proposal.edges);
        result.request_local_no_goods.push_back(proposal.edges);
        result.rejection_reasons.push_back(std::move(rejection));
        ++result.no_good_rejections;
    }
    result.reason="target_lift_exhausted";
    return result;
}

}  // namespace gf
