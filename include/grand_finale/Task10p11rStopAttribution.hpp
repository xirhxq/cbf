#pragma once

#include "grand_finale/GrandFinaleSwarmAdapter.hpp"
#include "grand_finale/InformationGateDiagnostic.hpp"

#include <cstdint>
#include <sstream>

namespace gf {

struct ReferenceEdgeAttribution {
    DirectedEdge edge;
    double distance_m=0.0;
    double range_variance_m2=0.0;
    double position_support_m=0.0;
    double aoi_s=0.0;
    double quality=0.0;
};

struct ReferenceOwnerAttribution {
    bool valid=false;
    std::string reason;
    NodeId owner=0;
    std::vector<DirectedEdge> effective_edges;
    std::vector<ReferenceEdgeAttribution> edges;
    double nominal_fim_eigenvalue=
        -std::numeric_limits<double>::infinity();
    double posterior_fim_proxy_eigenvalue=
        -std::numeric_limits<double>::infinity();
    double robust_cone_fim_lower_bound=
        -std::numeric_limits<double>::infinity();
    double reference_angle_rad=0.0;
    Eigen::Matrix2d owner_position_covariance=Eigen::Matrix2d::Zero();
};

inline ReferenceOwnerAttribution attributeReferenceOwner(
    const GrandFinaleRuntimeSnapshot& runtime,NodeId owner,
    double uncertainty_sigma) {
    ReferenceOwnerAttribution result;
    result.owner=owner;
    if (!std::isfinite(uncertainty_sigma) || uncertainty_sigma<0.0) {
        result.reason="invalid_uncertainty_sigma";
        return result;
    }
    std::map<std::string,double> variances;
    std::vector<Eigen::Vector2d> directions;
    const Eigen::Vector2d owner_position=
        detail::nodePosition(runtime.estimate,owner);
    const double owner_support=uncertainty_sigma*std::sqrt(std::max(
        0.0,detail::maximumPositionEigenvalue(runtime.estimate,owner)));
    result.owner_position_covariance=detail::positionCovariance(
        runtime.estimate,owner);
    for (const auto& edge:runtime.topology) {
        if (edge.owner!=owner) continue;
        const std::string range_id=UndirectedEdge::canonical(
            owner,edge.reference).id();
        const auto link=runtime.range_links.find(range_id);
        if (link==runtime.range_links.end()) {
            result.reason="range_missing:"+range_id;
            return result;
        }
        const Eigen::Vector2d delta=owner_position-
            detail::nodePosition(runtime.estimate,edge.reference);
        const double distance=delta.norm();
        if (!std::isfinite(distance) || distance<=1e-12 ||
            !std::isfinite(link->second.variance_m2) ||
            link->second.variance_m2<=0.0) {
            result.reason="invalid_reference_geometry:"+edge.id();
            return result;
        }
        const double reference_support=uncertainty_sigma*std::sqrt(std::max(
            0.0,detail::maximumPositionEigenvalue(
                runtime.estimate,edge.reference)));
        result.effective_edges.push_back(edge);
        result.edges.push_back({edge,distance,link->second.variance_m2,
            owner_support+reference_support,link->second.age_s,
            link->second.quality});
        directions.push_back(delta/distance);
        variances[range_id]=link->second.variance_m2;
    }
    if (result.effective_edges.size()<2) {
        result.reason="insufficient_effective_references";
        return result;
    }
    Eigen::Matrix2d nominal=Eigen::Matrix2d::Zero();
    for (std::size_t index=0;index<directions.size();++index) {
        const auto& edge=result.effective_edges[index];
        nominal+=directions[index]*directions[index].transpose()/
            variances.at(UndirectedEdge::canonical(
                owner,edge.reference).id());
    }
    result.nominal_fim_eigenvalue=
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(nominal)
            .eigenvalues().minCoeff();
    result.posterior_fim_proxy_eigenvalue=
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(referenceFim(
            owner,result.effective_edges,runtime.estimate,variances))
            .eigenvalues().minCoeff();
    result.robust_cone_fim_lower_bound=robustReferenceFimConeLowerBound(
        owner,result.effective_edges,runtime.estimate,variances,
        uncertainty_sigma);
    const double cosine=std::clamp(std::abs(directions[0].dot(directions[1])),
        0.0,1.0);
    result.reference_angle_rad=std::acos(cosine);
    result.valid=std::isfinite(result.nominal_fim_eigenvalue) &&
        std::isfinite(result.posterior_fim_proxy_eigenvalue) &&
        std::isfinite(result.robust_cone_fim_lower_bound);
    result.reason=result.valid?"reference_attribution_complete":
        "nonfinite_reference_attribution";
    return result;
}

struct ReplacementCandidateAttribution {
    DirectedEdge removal{0,1};
    DirectedEdge addition{0,1};
    bool dag_valid=false;
    bool keep_distance_valid=false;
    bool add_distance_valid=false;
    bool aoi_valid=false;
    bool quality_valid=false;
    bool posterior_valid=false;
    bool fim_valid=false;
    bool reference_lens_valid=false;
    double nominal_fim_eigenvalue=
        -std::numeric_limits<double>::infinity();
    double posterior_fim_proxy_eigenvalue=
        -std::numeric_limits<double>::infinity();
    double robust_cone_fim_lower_bound=
        -std::numeric_limits<double>::infinity();
    double target_fim_planning_score=
        -std::numeric_limits<double>::infinity();
    double target_cone_planning_score=
        -std::numeric_limits<double>::infinity();
    std::map<NodeId,Eigen::Vector2d> raw_targets;
    std::map<NodeId,Eigen::Vector2d> lifted_targets;
    std::string exact_rejection_reason;
};

struct ReplacementCandidateAttributionLedger {
    bool valid=false;
    std::string reason;
    std::vector<ReplacementCandidateAttribution> candidates;
    std::uint64_t digest=0;
};

inline ReplacementCandidateAttributionLedger freezeCandidateAttributionLedger(
    std::vector<ReplacementCandidateAttribution> candidates) {
    ReplacementCandidateAttributionLedger result;
    std::set<std::string> identities;
    std::ostringstream canonical;
    canonical.precision(17);
    for (const auto& candidate:candidates) {
        if (candidate.removal.reference==0 || candidate.removal.owner==0 ||
            candidate.addition.reference==0 || candidate.addition.owner==0 ||
            candidate.removal.owner!=candidate.addition.owner ||
            candidate.exact_rejection_reason.empty()) {
            result.reason="invalid_candidate_attribution";
            return result;
        }
        const std::string identity=candidate.removal.id()+"=>"+
            candidate.addition.id();
        if (!identities.insert(identity).second) {
            result.reason="duplicate_candidate_attribution";
            return result;
        }
        canonical<<identity<<':'<<candidate.target_cone_planning_score<<':'
                 <<candidate.exact_rejection_reason<<';';
    }
    std::uint64_t hash=1469598103934665603ULL;
    for (const unsigned char byte:canonical.str()) {
        hash^=byte;
        hash*=1099511628211ULL;
    }
    result.valid=true;
    result.reason="candidate_attribution_frozen";
    result.candidates=std::move(candidates);
    result.digest=hash==0?1:hash;
    return result;
}

}  // namespace gf
