#pragma once

#include "grand_finale/Task10p11rFixedBaseline.hpp"

namespace gf {

struct FimGraphSemanticsThresholds {
    double maximum_information_distance_m=0.0;
    double maximum_aoi_s=0.0;
    double minimum_quality=0.0;
    double uncertainty_sigma=0.0;
    double minimum_robust_fim=0.0;
};

struct FimEdgeSetAudit {
    std::vector<DirectedEdge> edges;
    std::map<std::string,double> range_variances_m2;
    double nominal_eigenvalue=-std::numeric_limits<double>::infinity();
    double posterior_proxy_eigenvalue=-std::numeric_limits<double>::infinity();
    double robust_cone_lower_bound=-std::numeric_limits<double>::infinity();
    bool robust_gate_valid=false;
};

struct FimGraphSemanticsAudit {
    bool valid=false;
    std::string reason;
    NodeId owner=0;
    FimEdgeSetAudit reference;
    FimEdgeSetAudit authority_augmented;
    FimEdgeSetAudit all_accepted;
    std::map<std::string,std::string> rejected_information_edges;
};

namespace task10p11r_fim_detail {

inline bool finiteThresholds(const FimGraphSemanticsThresholds& value) {
    return std::isfinite(value.maximum_information_distance_m) &&
        value.maximum_information_distance_m>0.0 &&
        std::isfinite(value.maximum_aoi_s) && value.maximum_aoi_s>=0.0 &&
        std::isfinite(value.minimum_quality) && value.minimum_quality>=0.0 &&
        std::isfinite(value.uncertainty_sigma) && value.uncertainty_sigma>=0.0 &&
        std::isfinite(value.minimum_robust_fim) &&
        value.minimum_robust_fim>=0.0;
}

inline void sortEdges(std::vector<DirectedEdge>& edges) {
    std::sort(edges.begin(),edges.end(),[](const auto& lhs,const auto& rhs) {
        return std::tie(lhs.reference,lhs.owner)<
            std::tie(rhs.reference,rhs.owner);
    });
    edges.erase(std::unique(edges.begin(),edges.end(),[](const auto& lhs,
        const auto& rhs) { return lhs.id()==rhs.id(); }),edges.end());
}

inline FimEdgeSetAudit evaluateSet(NodeId owner,
    std::vector<DirectedEdge> edges,const GrandFinaleRuntimeSnapshot& runtime,
    const FimGraphSemanticsThresholds& thresholds) {
    sortEdges(edges);
    FimEdgeSetAudit result;
    result.edges=std::move(edges);
    for (const auto& edge:result.edges) {
        const std::string id=UndirectedEdge::canonical(
            edge.owner,edge.reference).id();
        result.range_variances_m2[id]=runtime.range_links.at(id).variance_m2;
    }
    if (result.edges.size()<2) return result;
    Eigen::Matrix2d nominal=Eigen::Matrix2d::Zero();
    const auto owner_position=detail::nodePosition(runtime.estimate,owner);
    for (const auto& edge:result.edges) {
        const Eigen::Vector2d delta=owner_position-
            detail::nodePosition(runtime.estimate,edge.reference);
        const Eigen::Vector2d direction=delta/delta.norm();
        nominal+=direction*direction.transpose()/
            result.range_variances_m2.at(UndirectedEdge::canonical(
                owner,edge.reference).id());
    }
    result.nominal_eigenvalue=
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(nominal)
            .eigenvalues().minCoeff();
    result.posterior_proxy_eigenvalue=
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(referenceFim(
            owner,result.edges,runtime.estimate,result.range_variances_m2))
            .eigenvalues().minCoeff();
    result.robust_cone_lower_bound=robustReferenceFimConeLowerBound(
        owner,result.edges,runtime.estimate,result.range_variances_m2,
        thresholds.uncertainty_sigma);
    result.robust_gate_valid=std::isfinite(result.robust_cone_lower_bound) &&
        result.robust_cone_lower_bound+1.0e-12>=thresholds.minimum_robust_fim;
    return result;
}

inline std::optional<std::size_t> branchIndex(
    const std::vector<LeaderCoverageBranchSpec>& branches,NodeId id) {
    for (const auto& branch:branches) {
        const auto iterator=std::find(branch.members.begin(),branch.members.end(),id);
        if (iterator!=branch.members.end())
            return static_cast<std::size_t>(iterator-branch.members.begin());
    }
    return std::nullopt;
}

inline const LeaderCoverageBranchSpec* ownerBranch(
    const std::vector<LeaderCoverageBranchSpec>& branches,NodeId owner) {
    for (const auto& branch:branches)
        if (std::find(branch.members.begin(),branch.members.end(),owner)!=
            branch.members.end()) return &branch;
    return nullptr;
}

}  // namespace task10p11r_fim_detail

inline FimGraphSemanticsAudit auditFimGraphSemantics(
    const GrandFinaleRuntimeSnapshot& runtime,
    const std::vector<AcceptedRangeUpdateAudit>& accepted_batch,NodeId owner,
    const std::vector<LeaderCoverageBranchSpec>& branches,
    const FimGraphSemanticsThresholds& thresholds) {
    FimGraphSemanticsAudit result;
    result.owner=owner;
    if (!task10p11r_fim_detail::finiteThresholds(thresholds)) {
        result.reason="invalid_thresholds";
        return result;
    }
    const auto* branch=task10p11r_fim_detail::ownerBranch(branches,owner);
    const auto owner_index=task10p11r_fim_detail::branchIndex(branches,owner);
    if (branch==nullptr || !owner_index.has_value()) {
        result.reason="owner_branch_missing";
        return result;
    }

    std::set<std::string> accepted_ids;
    for (const auto& update:accepted_batch) {
        if (update.measurement.edge.first!=owner &&
            update.measurement.edge.second!=owner) continue;
        const std::string id=update.measurement.edge.id();
        if (!accepted_ids.insert(id).second) {
            result.reason="duplicate_accepted_measurement:"+id;
            return result;
        }
    }

    auto qualified=[&](NodeId reference)->bool {
        const std::string id=UndirectedEdge::canonical(owner,reference).id();
        if (accepted_ids.count(id)==0) {
            result.rejected_information_edges[id]="not_accepted";
            return false;
        }
        const auto link=runtime.range_links.find(id);
        if (link==runtime.range_links.end()) {
            result.rejected_information_edges[id]="range_history_missing";
            return false;
        }
        if (!std::isfinite(link->second.variance_m2) ||
            link->second.variance_m2<=0.0) {
            result.rejected_information_edges[id]="invalid_variance";
            return false;
        }
        if (link->second.age_s>thresholds.maximum_aoi_s+1.0e-12) {
            result.rejected_information_edges[id]="stale";
            return false;
        }
        if (link->second.quality+1.0e-12<thresholds.minimum_quality) {
            result.rejected_information_edges[id]="low_quality";
            return false;
        }
        return true;
    };

    std::vector<DirectedEdge> reference_edges;
    for (const auto& edge:runtime.topology)
        if (edge.owner==owner && qualified(edge.reference))
            reference_edges.push_back(edge);

    std::vector<DirectedEdge> all_edges;
    for (const auto& update:accepted_batch) {
        const auto& measured=update.measurement.edge;
        if (measured.first!=owner && measured.second!=owner) continue;
        const NodeId reference=measured.first==owner?measured.second:measured.first;
        if (qualified(reference)) all_edges.emplace_back(reference,owner);
    }

    std::vector<DirectedEdge> authority_edges=reference_edges;
    const auto owner_position=detail::nodePosition(runtime.estimate,owner);
    for (const auto& edge:all_edges) {
        const NodeId reference=edge.reference;
        const double distance=(owner_position-
            detail::nodePosition(runtime.estimate,reference)).norm();
        if (distance>thresholds.maximum_information_distance_m+1.0e-12)
            continue;
        if (runtime.estimate.fixed_positions.count(reference)!=0) {
            authority_edges.push_back(edge);
            continue;
        }
        const auto reference_index=task10p11r_fim_detail::branchIndex(
            branches,reference);
        if (reference_index.has_value() &&
            *reference_index<*owner_index &&
            std::find(branch->members.begin(),branch->members.end(),reference)!=
                branch->members.end()) authority_edges.push_back(edge);
    }

    try {
        result.reference=task10p11r_fim_detail::evaluateSet(
            owner,reference_edges,runtime,thresholds);
        result.authority_augmented=task10p11r_fim_detail::evaluateSet(
            owner,authority_edges,runtime,thresholds);
        result.all_accepted=task10p11r_fim_detail::evaluateSet(
            owner,all_edges,runtime,thresholds);
    } catch (const std::exception& error) {
        result.reason=std::string("fim_rebuild_failed:")+error.what();
        return result;
    }
    result.valid=true;
    result.reason="fim_graph_semantics_audited";
    return result;
}

}  // namespace gf
