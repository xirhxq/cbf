#pragma once

#include "grand_finale/CertifiedTargetLifting.hpp"
#include "grand_finale/HybridSupervisor.hpp"

#include <Eigen/Eigenvalues>

#include <optional>

namespace gf {

struct TargetGeometryGateRequest {
    std::vector<NodeId> mobile_ids;
    std::vector<NodeId> fixed_ids;
    std::vector<DirectedEdge> reference_edges;
    std::map<NodeId,Eigen::Vector2d> targets;
    std::map<NodeId,Eigen::Vector2d> raw_targets;
    std::map<NodeId,double> position_support_m;
    std::map<std::string,double> range_variances_m2;
    std::map<std::string,bool> edge_information_valid;
    std::map<NodeId,bool> posterior_valid;
    double minimum_collision_distance_m=0.0;
    double minimum_target_fim_planning_score=0.0;
    double minimum_target_cone_planning_score=0.0;
    double maximum_target_deformation_m=
        std::numeric_limits<double>::infinity();
    double tolerance=1e-9;
};

struct TargetGeometryGateAudit {
    bool valid=false;
    std::string reason;
    double minimum_target_collision_distance_m=
        std::numeric_limits<double>::infinity();
    double minimum_target_fim_planning_score=
        std::numeric_limits<double>::infinity();
    NodeId minimum_target_fim_owner=0;
    double minimum_target_cone_planning_score=
        std::numeric_limits<double>::infinity();
    NodeId minimum_target_cone_owner=0;
    double maximum_target_deformation_m=0.0;
};

inline TargetGeometryGateAudit evaluateTargetGeometryGates(
    const TargetGeometryGateRequest& request) {
    TargetGeometryGateAudit audit;
    if (request.mobile_ids.empty() ||
        !std::isfinite(request.minimum_collision_distance_m) ||
        request.minimum_collision_distance_m<0.0 ||
        !std::isfinite(request.minimum_target_fim_planning_score) ||
        request.minimum_target_fim_planning_score<0.0 ||
        !std::isfinite(request.minimum_target_cone_planning_score) ||
        request.minimum_target_cone_planning_score<0.0 ||
        !std::isfinite(request.maximum_target_deformation_m) ||
        request.maximum_target_deformation_m<0.0 ||
        !std::isfinite(request.tolerance) || request.tolerance<0.0) {
        audit.reason="invalid_target_geometry_request";
        return audit;
    }
    for (std::size_t first=0;first<request.mobile_ids.size();++first) {
        const auto first_target=request.targets.find(request.mobile_ids[first]);
        if (first_target==request.targets.end()) {
            audit.reason="target_missing";
            return audit;
        }
        for (std::size_t second=first+1;second<request.mobile_ids.size();++second) {
            const auto second_target=request.targets.find(
                request.mobile_ids[second]);
            if (second_target==request.targets.end()) {
                audit.reason="target_missing";
                return audit;
            }
            audit.minimum_target_collision_distance_m=std::min(
                audit.minimum_target_collision_distance_m,
                (first_target->second-second_target->second).norm());
        }
    }
    if (request.mobile_ids.size()<2)
        audit.minimum_target_collision_distance_m=
            std::numeric_limits<double>::infinity();
    for (NodeId mobile : request.mobile_ids) {
        const auto mobile_target=request.targets.find(mobile);
        if (mobile_target==request.targets.end()) {
            audit.reason="target_missing";
            return audit;
        }
        for (NodeId fixed : request.fixed_ids) {
            const auto fixed_target=request.targets.find(fixed);
            if (fixed_target==request.targets.end()) {
                audit.reason="target_missing";
                return audit;
            }
            audit.minimum_target_collision_distance_m=std::min(
                audit.minimum_target_collision_distance_m,
                (mobile_target->second-fixed_target->second).norm());
        }
    }
    if (audit.minimum_target_collision_distance_m+request.tolerance<
        request.minimum_collision_distance_m) {
        audit.reason="target_collision";
        return audit;
    }

    for (NodeId owner : request.mobile_ids) {
        const auto owner_target=request.targets.find(owner);
        const auto raw=request.raw_targets.find(owner);
        const auto owner_support=request.position_support_m.find(owner);
        const auto posterior=request.posterior_valid.find(owner);
        if (owner_target==request.targets.end() || raw==request.raw_targets.end() ||
            owner_support==request.position_support_m.end() ||
            posterior==request.posterior_valid.end() || !posterior->second ||
            !std::isfinite(owner_support->second) || owner_support->second<0.0) {
            audit.reason="target_information:owner="+std::to_string(owner);
            return audit;
        }
        audit.maximum_target_deformation_m=std::max(
            audit.maximum_target_deformation_m,
            (owner_target->second-raw->second).norm());
        Eigen::Matrix2d fim=Eigen::Matrix2d::Zero();
        double projector_reserve=0.0;
        std::size_t references=0;
        for (const auto& edge : request.reference_edges) {
            if (edge.owner!=owner) continue;
            ++references;
            const auto reference=request.targets.find(edge.reference);
            const auto support=request.position_support_m.find(edge.reference);
            const auto information=request.edge_information_valid.find(edge.id());
            const std::string range_id=
                UndirectedEdge::canonical(owner,edge.reference).id();
            const auto variance=request.range_variances_m2.find(range_id);
            if (reference==request.targets.end() ||
                support==request.position_support_m.end() ||
                information==request.edge_information_valid.end() ||
                !information->second || variance==request.range_variances_m2.end() ||
                !std::isfinite(support->second) || support->second<0.0 ||
                !std::isfinite(variance->second) || variance->second<=0.0) {
                audit.reason="target_information:edge="+edge.id();
                return audit;
            }
            const Eigen::Vector2d delta=
                owner_target->second-reference->second;
            const double distance=delta.norm();
            const double radius=owner_support->second+support->second;
            if (!std::isfinite(distance) || distance<=1e-12) {
                audit.reason="target_fim";
                return audit;
            }
            const Eigen::Vector2d direction=delta/distance;
            fim+=direction*direction.transpose()/variance->second;
            if (distance<=radius+request.tolerance)
                projector_reserve=std::numeric_limits<double>::infinity();
            else
                projector_reserve+=radius/(distance*variance->second);
        }
        if (references<2) {
            audit.reason="target_information:owner="+std::to_string(owner)+
                ":reference_count";
            return audit;
        }
        const double nominal=Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(
            0.5*(fim+fim.transpose())).eigenvalues().minCoeff();
        const double robust=nominal-projector_reserve;
        if (nominal<audit.minimum_target_fim_planning_score) {
            audit.minimum_target_fim_planning_score=nominal;
            audit.minimum_target_fim_owner=owner;
        }
        if (robust<audit.minimum_target_cone_planning_score) {
            audit.minimum_target_cone_planning_score=robust;
            audit.minimum_target_cone_owner=owner;
        }
        if (!std::isfinite(nominal) ||
            nominal+request.tolerance<
                request.minimum_target_fim_planning_score) {
            audit.reason="target_fim_planning_score";
            return audit;
        }
        if (!std::isfinite(robust) ||
            robust+request.tolerance<
                request.minimum_target_cone_planning_score) {
            audit.reason="target_cone_planning_score";
            return audit;
        }
    }
    if (audit.maximum_target_deformation_m>request.maximum_target_deformation_m+
        request.tolerance) {
        audit.reason="target_deformation_budget";
        return audit;
    }
    audit.valid=true;
    audit.reason="target_geometry_planning_gates_passed";
    return audit;
}

struct TargetLiftTransitionRequest {
    std::vector<NodeId> mobile_ids;
    std::vector<NodeId> fixed_ids;
    std::vector<DirectedEdge> old_edges;
    std::vector<DirectedEdge> successor_edges;
    std::map<NodeId,Eigen::Vector2d> raw_targets;
    std::map<NodeId,Eigen::Vector2d> fixed_targets;
    double add_distance_m=849.0;
    double target_margin_m=1.0;
    TopologyVersion topology_version=0;
    std::uint64_t estimator_version=0;
    std::uint64_t raw_ledger_version=0;
    std::uint64_t config_version=0;
};

struct FrozenTargetLiftToken {
    bool valid=false;
    std::string reason;
    std::vector<DirectedEdge> old_edges;
    std::vector<DirectedEdge> union_edges;
    std::vector<DirectedEdge> successor_edges;
    std::map<NodeId,Eigen::Vector2d> lifted_targets;
    std::uint64_t lifted_digest=0;
    TopologyVersion topology_version=0;
    std::uint64_t estimator_version=0;
    std::uint64_t raw_ledger_version=0;
    std::uint64_t raw_ledger_digest=0;
    std::uint64_t config_version=0;
    std::uint64_t config_digest=0;
    TopologyVersion union_topology_version=0;
    std::size_t completed_union_cycles=0;
    double r_plan_m=0.0;
    std::optional<double> t100_coverage_s;
};

inline std::uint64_t canonicalTargetLedgerDigest(
    const std::map<NodeId,Eigen::Vector2d>& targets) {
    std::ostringstream stream;
    stream.precision(17);
    for (const auto& [owner,target] : targets)
        stream<<owner<<':'<<target.x()<<','<<target.y()<<';';
    return certified_target_lifting_detail::hashText(stream.str());
}

inline std::uint64_t targetLiftConfigDigest(
    double add_distance_m,double target_margin_m) {
    std::ostringstream stream;
    stream.precision(17);
    stream<<"add="<<add_distance_m<<";margin="<<target_margin_m<<';';
    return certified_target_lifting_detail::hashText(stream.str());
}

inline std::vector<DirectedEdge> canonicalTargetEdges(
    std::vector<DirectedEdge> edges) {
    std::sort(edges.begin(),edges.end(),[](const auto& lhs,const auto& rhs) {
        return lhs.id()<rhs.id();
    });
    edges.erase(std::unique(edges.begin(),edges.end(),[](const auto& lhs,
        const auto& rhs) { return lhs.id()==rhs.id(); }),edges.end());
    return edges;
}

inline std::vector<DirectedEdge> targetEdgeDifference(
    const std::vector<DirectedEdge>& lhs,
    const std::vector<DirectedEdge>& rhs) {
    std::vector<DirectedEdge> result;
    for (const auto& edge : lhs)
        if (std::none_of(rhs.begin(),rhs.end(),[&](const auto& other) {
                return edge.id()==other.id();
            }))
            result.push_back(edge);
    return canonicalTargetEdges(std::move(result));
}

inline bool targetEdgesCompatible(
    const std::vector<DirectedEdge>& edges,
    const std::map<NodeId,Eigen::Vector2d>& targets,double radius,
    double tolerance) {
    for (const auto& edge : edges) {
        const auto reference=targets.find(edge.reference);
        const auto owner=targets.find(edge.owner);
        if (reference==targets.end() || owner==targets.end() ||
            (owner->second-reference->second).norm()>radius+tolerance)
            return false;
    }
    return true;
}

inline FrozenTargetLiftToken freezeUnionTargetLift(
    const TargetLiftTransitionRequest& request) {
    FrozenTargetLiftToken token;
    token.old_edges=canonicalTargetEdges(request.old_edges);
    token.successor_edges=canonicalTargetEdges(request.successor_edges);
    token.union_edges=token.old_edges;
    token.union_edges.insert(token.union_edges.end(),
        token.successor_edges.begin(),token.successor_edges.end());
    token.union_edges=canonicalTargetEdges(std::move(token.union_edges));
    TargetLiftRequest lift;
    lift.mobile_ids=request.mobile_ids;
    lift.fixed_ids=request.fixed_ids;
    lift.reference_edges=token.union_edges;
    lift.raw_targets=request.raw_targets;
    lift.fixed_targets=request.fixed_targets;
    lift.add_distance_m=request.add_distance_m;
    lift.target_margin_m=request.target_margin_m;
    const auto result=liftReferenceFeasibleTargets(std::move(lift));
    if (!result.valid) {
        token.reason=result.reason;
        return token;
    }
    token.lifted_targets=result.targets;
    token.lifted_digest=result.canonical_digest;
    token.r_plan_m=result.r_plan_m;
    token.topology_version=request.topology_version;
    token.estimator_version=request.estimator_version;
    token.raw_ledger_version=request.raw_ledger_version;
    token.raw_ledger_digest=canonicalTargetLedgerDigest(request.raw_targets);
    token.config_version=request.config_version;
    token.config_digest=targetLiftConfigDigest(
        request.add_distance_m,request.target_margin_m);
    if (!targetEdgesCompatible(token.old_edges,token.lifted_targets,
            result.r_plan_m,1e-9) ||
        !targetEdgesCompatible(token.successor_edges,token.lifted_targets,
            result.r_plan_m,1e-9)) {
        token.reason="union_target_edge_audit_failed";
        return token;
    }
    token.valid=true;
    token.reason="union_target_frozen";
    return token;
}

inline bool successorTargetCompatible(
    const FrozenTargetLiftToken& token,double tolerance) {
    return token.valid && targetEdgesCompatible(
        token.successor_edges,token.lifted_targets,token.r_plan_m,tolerance);
}

inline FrozenTargetLiftToken recordT100StatisticOnly(
    FrozenTargetLiftToken token,double time_s) {
    if (token.valid && !token.t100_coverage_s.has_value() &&
        std::isfinite(time_s) && time_s>=0.0)
        token.t100_coverage_s=time_s;
    return token;
}

struct FrozenUnionCycleEvidence {
    SupervisorMode mode=SupervisorMode::Search;
    std::vector<DirectedEdge> active_edges;
    std::uint64_t applied_target_digest=0;
    std::uint64_t raw_ledger_version=0;
    std::uint64_t raw_ledger_digest=0;
    std::uint64_t config_version=0;
    std::uint64_t config_digest=0;
    TopologyVersion topology_version=0;
    std::uint64_t estimator_version_before=0;
    std::uint64_t estimator_version_after=0;
    bool physical_advanced=false;
    bool exact_zoh_completed=false;
    double minimum_hard_residual=-std::numeric_limits<double>::infinity();
    double residual_tolerance=0.0;
};

inline FrozenTargetLiftToken recordFreshUnionControlCycle(
    FrozenTargetLiftToken token,FrozenUnionCycleEvidence evidence) {
    evidence.active_edges=canonicalTargetEdges(std::move(evidence.active_edges));
    if (!token.valid || evidence.mode!=SupervisorMode::Union ||
        evidence.active_edges!=token.union_edges ||
        token.lifted_digest!=evidence.applied_target_digest ||
        token.raw_ledger_version!=evidence.raw_ledger_version ||
        token.raw_ledger_digest!=evidence.raw_ledger_digest ||
        token.config_version!=evidence.config_version ||
        token.config_digest!=evidence.config_digest ||
        !evidence.physical_advanced || !evidence.exact_zoh_completed ||
        evidence.estimator_version_before<token.estimator_version ||
        evidence.estimator_version_after<=evidence.estimator_version_before ||
        !std::isfinite(evidence.minimum_hard_residual) ||
        !std::isfinite(evidence.residual_tolerance) ||
        evidence.residual_tolerance<0.0 ||
        evidence.minimum_hard_residual<-evidence.residual_tolerance) {
        token.valid=false;
        token.reason="stale_union_target_cycle";
        return token;
    }
    token.estimator_version=evidence.estimator_version_after;
    token.union_topology_version=evidence.topology_version;
    ++token.completed_union_cycles;
    token.reason="fresh_union_target_cycle";
    return token;
}

inline bool freshBreakTargetReady(
    const FrozenTargetLiftToken& token,TopologyVersion topology_version,
    std::uint64_t fresh_estimator_version) {
    return token.valid && token.completed_union_cycles>=1 &&
        token.union_topology_version==topology_version &&
        fresh_estimator_version==token.estimator_version &&
        successorTargetCompatible(token,1e-9);
}

}  // namespace gf
